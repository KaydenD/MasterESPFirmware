#include "battery.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "webserver.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BATTERY_CHARGER_ADDRESS 0x6A

#define BATTERY_VOLTAGE_READ_EN_PIN GPIO_NUM_33
#define BATTERY_VOLTAGE_ADC_CHANNEL ADC_CHANNEL_4
#define ADC_ATTEN ADC_ATTEN_DB_11

#define BATTERY_CHARGER_INT GPIO_NUM_26

#define BATTERY_CHECK_INTERVAL 5000 /* ms */ * 1000 /* us/ms */


static const char *TAG = "BATTERY";

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_battvolt_channel;

static QueueHandle_t irq_evt_queue = NULL;

typedef enum {
    CHARGER_INTERUPT,
    TIMER
} EventType;

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void batteryEventHandler(){
    EventType event;
    while(1){
        if (xQueueReceive(irq_evt_queue, &event, portMAX_DELAY)) {
            if(event == CHARGER_INTERUPT){
                ESP_LOGW(TAG, "Battery Charger Interupt");
            } else {
                ESP_LOGW(TAG, "Battery Watchdog Interupt");
            }

            int rawBatADC, batVoltage;
            gpio_set_level(BATTERY_VOLTAGE_READ_EN_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_VOLTAGE_ADC_CHANNEL, &rawBatADC));
            gpio_set_level(BATTERY_VOLTAGE_READ_EN_PIN, 0);
            if(adc_cali_raw_to_voltage(adc1_cali_battvolt_channel, rawBatADC, &batVoltage) == ESP_OK){
                batVoltage *= 2;
                ESP_LOGI(TAG, "Battery Voltage (mv): %u", batVoltage);
            } else {
                ESP_LOGE(TAG, "Failed to use calibration to convert to voltage");
                batVoltage = 0;
            }

            char payload[512];
            int lenWritten = sprintf(payload, "{\n\"batteryVoltage\": %i", batVoltage);

            uint8_t value;
            for(uint8_t i = 0x00; i <= 12; i++){
                //ESP_LOGI(TAG, "%X", i);
                ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_MASTER_NUM, 0x6A, &i, 1, &value, 1, 1000 / portTICK_PERIOD_MS));
                //ESP_LOGI(TAG, "REG[0x%hhX] = 0b"BYTE_TO_BINARY_PATTERN, i, BYTE_TO_BINARY(value));
                lenWritten += sprintf(payload + lenWritten, ",\n\"REG0x%02hhX\": \"0b"BYTE_TO_BINARY_PATTERN"\"", i, BYTE_TO_BINARY(value)); 
            }
            lenWritten += sprintf(payload + lenWritten, "\n}");
            //ESP_LOGI(TAG, "%s", payload);
            httpd_ws_frame_t frame = {
                .type = HTTPD_WS_TYPE_TEXT,
                .len = lenWritten,
                .payload = (uint8_t *)payload
            };
            httpd_ws_send_frame_to_all_clients(&frame);
        }
    }
}

static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
}

static void IRAM_ATTR batteryIRQ(void* arg)
{
    EventType evtType = CHARGER_INTERUPT;
    xQueueSendFromISR(irq_evt_queue, &evtType, NULL);
}

static void IRAM_ATTR timer_isr_handler(void* arg)
{
    EventType evtType = TIMER;
    xQueueSendFromISR(irq_evt_queue, &evtType, NULL);
}


esp_err_t batteryManagmentInit(){
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << BATTERY_VOLTAGE_READ_EN_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(BATTERY_VOLTAGE_READ_EN_PIN, 0);

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_VOLTAGE_ADC_CHANNEL, &config));
    adc_calibration_init(ADC_UNIT_1, BATTERY_VOLTAGE_ADC_CHANNEL, ADC_ATTEN, &adc1_cali_battvolt_channel);


    gpio_config_t io_conf_pd = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << BATTERY_CHARGER_INT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_pd);
    //gpio_install_isr_service(0); done in main
    gpio_isr_handler_add(BATTERY_CHARGER_INT, batteryIRQ, NULL);

    irq_evt_queue = xQueueCreate(10, sizeof(EventType));

    esp_timer_create_args_t checkBatteryStatusTimer_config = {
        .callback = timer_isr_handler,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Battery Status Watchdog",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t checkBatteryStatusTimer;
    ESP_ERROR_CHECK(esp_timer_create(&checkBatteryStatusTimer_config, &checkBatteryStatusTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(checkBatteryStatusTimer, BATTERY_CHECK_INTERVAL));

    ESP_ERROR_CHECK(i2c_master_init());
    i2c_master_write_to_device(I2C_MASTER_NUM, BATTERY_CHARGER_ADDRESS, (const uint8_t[]){0x04, 0b01000011}, 2, 1000 / portTICK_PERIOD_MS); /* Charger Current*/
    i2c_master_write_to_device(I2C_MASTER_NUM, BATTERY_CHARGER_ADDRESS, (const uint8_t[]){0x07, 0b10000111}, 2, 1000 / portTICK_PERIOD_MS); /* I forget */
    i2c_master_write_to_device(I2C_MASTER_NUM, BATTERY_CHARGER_ADDRESS, (const uint8_t[]){0x08, 0b01001111}, 2, 1000 / portTICK_PERIOD_MS); /* I forget */
    i2c_master_write_to_device(I2C_MASTER_NUM, BATTERY_CHARGER_ADDRESS, (const uint8_t[]){0x0A, 0b01000001}, 2, 1000 / portTICK_PERIOD_MS); /* I forget */

    xTaskCreate(batteryEventHandler, "BatteryManagement", 4096, NULL, 5, NULL);
    return ESP_OK;
}