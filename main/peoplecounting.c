#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "webserver.h"
#include "common.h"


#define TOP_PD_PIN GPIO_NUM_23
#define BOTTOM_PD_PIN GPIO_NUM_18

#define TOP_LED_PIN GPIO_NUM_9
#define BOTTOM_LED_PIN GPIO_NUM_10
#define BUZZER_PIN GPIO_NUM_12

#define MIN_BEAM_BREAK_TIME 25 /* ms */ * 1000 /* us/ms */
#define BEAM_BREAK_TIMEOUT_TIME 500 /* ms */ * 1000 /* us/ms */
#define BLOCKED_TIME 2500 /* ms */ * 1000 /* us/ms */
#define MIN_BETWEEN_FIRST_AND_SECOND_TIME 75 /* ms */ * 1000 /* us/ms */ 

typedef enum {
    PHOTODIODE,
    MIN_BEAM_BREAK_TIME_INTERVAL,
    BLOCKED_TIME_INTERVAL,
    TIMEOUT_TIME_INTERVAL,
    MIN_TIME_BETWEEN_INTERVAL
} EventType;

typedef struct {
    int64_t time;
    gpio_num_t pin;
    uint32_t state;
    EventType type;
} StateMachineEvent;

typedef struct {
    gpio_num_t pdpin;
    gpio_num_t ledpin;
} PDLEDPair;

static const char *TAG = "PEOPLE COUNTING";
static const PDLEDPair topPair = {
        .pdpin = TOP_PD_PIN,
        .ledpin = TOP_LED_PIN
    };
static const PDLEDPair bottomPair = {
        .pdpin = BOTTOM_PD_PIN,
        .ledpin = BOTTOM_LED_PIN
    };
static int* count = NULL;
static QueueHandle_t state_machine_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    PDLEDPair* gpio_pair = (PDLEDPair*) arg;
    StateMachineEvent state = {
        .time = esp_timer_get_time(),
        .pin = gpio_pair->pdpin,
        .state = gpio_get_level(gpio_pair->pdpin),
        .type = PHOTODIODE
    };
    gpio_set_level(gpio_pair->ledpin, state.state);
    xQueueSendFromISR(state_machine_evt_queue, &state, NULL);
}

static void IRAM_ATTR timer_isr_handler(void* arg)
{
    EventType TimerType = (EventType)arg;
    StateMachineEvent state = {
        .time = esp_timer_get_time(),
        .type = TimerType
    };
    xQueueSendFromISR(state_machine_evt_queue, &state, NULL);
}

static void peopleCountingAlgoritmMain(){
    esp_timer_create_args_t minBreakTimeTimer_config = {
        .callback = timer_isr_handler,
        .arg = (void*) MIN_BEAM_BREAK_TIME_INTERVAL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Min Break Time Vaild Timer",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t minBreakTimeTimer;
    ESP_ERROR_CHECK(esp_timer_create(&minBreakTimeTimer_config, &minBreakTimeTimer));

    esp_timer_create_args_t blockedTimeTimer_config = {
        .callback = timer_isr_handler,
        .arg = (void*) BLOCKED_TIME_INTERVAL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Blocked Timer",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t blockedTimeTimer;
    ESP_ERROR_CHECK(esp_timer_create(&blockedTimeTimer_config, &blockedTimeTimer));

    esp_timer_create_args_t timeoutTimer_config = {
        .callback = timer_isr_handler,
        .arg = (void*) TIMEOUT_TIME_INTERVAL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Timeout Timer",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t timeoutTimer;
    ESP_ERROR_CHECK(esp_timer_create(&timeoutTimer_config, &timeoutTimer));

    esp_timer_create_args_t minTimeBetweenTimer_config = {
        .callback = timer_isr_handler,
        .arg = (void*) MIN_TIME_BETWEEN_INTERVAL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Min Time Between Timer",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t minTimeBetweenTimer;
    ESP_ERROR_CHECK(esp_timer_create(&minTimeBetweenTimer_config, &minTimeBetweenTimer));

    enum state {
        STATE_IDLE,
        STATE_FIRST_TRIGGER_DEBOUNCE,
        STATE_WAIT_MIN_TRANSIT,
        STATE_WAIT_UNTIL_SECOND_TRIGGER,
        STATE_SECOND_TRIGGER_DEBOUNCE,
        STATE_SECOND_TRIGGER_VALID,
        STATE_BLOCKED
    } fsm_state = STATE_IDLE;

    StateMachineEvent event;
    gpio_num_t firstTriggerPin = 0, secondTriggerPin = 0;
    for(;;){
        if (xQueueReceive(state_machine_evt_queue, &event, portMAX_DELAY)) {
            if(event.type == PHOTODIODE) {
                ESP_LOGI(TAG, "PD State Change, PIN = %u, STATE = %lu, TIME = %lli", event.pin, event.state, event.time);
            } else {
                ESP_LOGI(TAG, "State Machine Timer Expired, Timer = %u, TIME = %lli", event.type, event.time);
            }
            switch(fsm_state){
                case STATE_IDLE:
                    if(event.type == PHOTODIODE && event.state == 1){
                        fsm_state = STATE_FIRST_TRIGGER_DEBOUNCE;
                        esp_timer_start_once(minBreakTimeTimer, MIN_BEAM_BREAK_TIME);
                        esp_timer_start_once(blockedTimeTimer, BLOCKED_TIME);
                        esp_timer_start_once(minTimeBetweenTimer, MIN_BETWEEN_FIRST_AND_SECOND_TIME);
                        firstTriggerPin = event.pin;
                        ESP_LOGV(TAG, "First Trigger");
                    }
                    break;
                case STATE_FIRST_TRIGGER_DEBOUNCE:
                    if(event.type == MIN_BEAM_BREAK_TIME_INTERVAL){
                        fsm_state = STATE_WAIT_MIN_TRANSIT;
                    } else {
                        fsm_state = STATE_IDLE;
                        esp_timer_stop(minBreakTimeTimer);
                        esp_timer_stop(blockedTimeTimer);
                        esp_timer_stop(minTimeBetweenTimer);
                        ESP_LOGE(TAG, "First Trigger Debounce Fail");
                    }
                    break;
                case STATE_WAIT_MIN_TRANSIT:
                    if(event.type == MIN_TIME_BETWEEN_INTERVAL){
                        fsm_state = STATE_WAIT_UNTIL_SECOND_TRIGGER;
                    } else {
                        fsm_state = STATE_IDLE;
                        esp_timer_stop(blockedTimeTimer);
                        esp_timer_stop(minTimeBetweenTimer);
                        ESP_LOGE(TAG, "Min transit time failure");
                    }
                    break;
                case STATE_WAIT_UNTIL_SECOND_TRIGGER:
                    if(event.type == PHOTODIODE && event.pin != firstTriggerPin && event.state == 1){
                        fsm_state = STATE_SECOND_TRIGGER_DEBOUNCE;
                        secondTriggerPin = event.pin;
                        esp_timer_start_once(minBreakTimeTimer, MIN_BEAM_BREAK_TIME);
                        if(esp_timer_is_active(blockedTimeTimer)){
                            esp_timer_restart(blockedTimeTimer, BLOCKED_TIME);
                        } else {
                            esp_timer_start_once(blockedTimeTimer, BLOCKED_TIME);
                        }
                        esp_timer_stop(timeoutTimer);
                        ESP_LOGI(TAG, "Second Trigger Activated");
                    } else if(event.type == PHOTODIODE && event.pin == firstTriggerPin && event.state == 0) {
                        esp_timer_stop(blockedTimeTimer);
                        esp_timer_start_once(timeoutTimer, BEAM_BREAK_TIMEOUT_TIME);
                        ESP_LOGI(TAG, "First beam restored: starting timeout for second beam to break");
                    } else if(event.type == PHOTODIODE && event.pin == firstTriggerPin && event.state == 1) {
                        esp_timer_start_once(blockedTimeTimer, BLOCKED_TIME);
                        esp_timer_stop(timeoutTimer);
                        ESP_LOGI(TAG, "First beam broke again: dealing with timers");
                    } else if(event.type == TIMEOUT_TIME_INTERVAL){
                        fsm_state = STATE_IDLE;
                        ESP_LOGI(TAG, "Timeout after first trigger");
                    } else if(event.type == BLOCKED_TIME_INTERVAL){
                        fsm_state = STATE_BLOCKED;
                        gpio_set_level(BUZZER_PIN, 1);
                        ESP_LOGI(TAG, "BLOCKED");
                    }
                    break;
                case STATE_SECOND_TRIGGER_DEBOUNCE:
                    if(event.type == MIN_BEAM_BREAK_TIME_INTERVAL){
                        fsm_state = STATE_SECOND_TRIGGER_VALID;
                    } else {
                        if(gpio_get_level(firstTriggerPin)){
                            fsm_state = STATE_WAIT_UNTIL_SECOND_TRIGGER;
                            esp_timer_restart(blockedTimeTimer, BLOCKED_TIME);
                        } else {
                            fsm_state = STATE_IDLE;
                            esp_timer_stop(blockedTimeTimer);
                        }
                        esp_timer_stop(minBreakTimeTimer);
                        ESP_LOGE(TAG, "Second Trigger Debounce Fail");
                    }
                    break;
                case STATE_SECOND_TRIGGER_VALID:
                    if(event.type == PHOTODIODE && event.pin == secondTriggerPin && event.state == 0 && gpio_get_level(firstTriggerPin) == 0){
                        fsm_state = STATE_IDLE;
                        esp_timer_stop(blockedTimeTimer);
                        if(firstTriggerPin == TOP_PD_PIN && secondTriggerPin == BOTTOM_PD_PIN){
                            (*count)++;
                        } else {
                            (*count)--;
                        }

                        char payload[32];
                        httpd_ws_frame_t frame = {
                            .type = HTTPD_WS_TYPE_TEXT,
                            .len = sprintf(payload, "{\n\"count\": %i\n}", *count),
                            .payload = (uint8_t *)payload
                        };
                        httpd_ws_send_frame_to_all_clients(&frame);
                        writeCountToNVS();
                        ESP_LOGI(TAG, "FSM Cycle Complete. New count: %i", *count);
                    } else if (event.type == PHOTODIODE && event.pin == secondTriggerPin && event.state == 0 && gpio_get_level(firstTriggerPin) == 1){
                        fsm_state = STATE_WAIT_UNTIL_SECOND_TRIGGER;
                        esp_timer_restart(blockedTimeTimer, BLOCKED_TIME);
                    } else if(event.type == BLOCKED_TIME_INTERVAL){
                        fsm_state = STATE_BLOCKED;
                        gpio_set_level(BUZZER_PIN, 1);
                        ESP_LOGI(TAG, "BLOCKED");
                    }
                    break;
                case STATE_BLOCKED:
                    if(gpio_get_level(TOP_PD_PIN) == 0 && gpio_get_level(BOTTOM_PD_PIN) == 0){
                        gpio_set_level(BUZZER_PIN, 0);
                        fsm_state = STATE_IDLE;
                        ESP_LOGI(TAG, "UNBLOCKED");
                    }
                    break;
            }
        }
    }
}

esp_err_t startPeopleCountingAlgorithm(int* countp){

    count = countp;

    /*SETUP LEDS AND BUZZER*/
    gpio_config_t io_conf_leds = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TOP_LED_PIN | 1ULL << BOTTOM_LED_PIN | 1ULL << BUZZER_PIN,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_leds);

    /*SETUP PD INPUTS*/
    gpio_config_t io_conf_pd = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << TOP_PD_PIN | 1ULL << BOTTOM_PD_PIN,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_pd);

    gpio_set_level(TOP_LED_PIN, gpio_get_level(TOP_PD_PIN));
    gpio_set_level(BOTTOM_LED_PIN, gpio_get_level(BOTTOM_PD_PIN));
    gpio_set_level(BUZZER_PIN, 1);

    state_machine_evt_queue = xQueueCreate(10, sizeof(StateMachineEvent));
    // gpio_install_isr_service(0); done in main

    gpio_isr_handler_add(TOP_PD_PIN, gpio_isr_handler, (void*)&topPair);
    gpio_isr_handler_add(BOTTOM_PD_PIN, gpio_isr_handler, (void*)&bottomPair);

    xTaskCreate(peopleCountingAlgoritmMain, "PeopleCountingAlg", 4096, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));   

    gpio_set_level(BUZZER_PIN, 0);

    return ESP_OK;

}

