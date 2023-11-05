#include "common.h"
#include "esp_log.h"
#include "webserver.h"
#include "battery.h"
#include "wifi.h"
#include "peoplecounting.h"
#include "nvs_flash.h"
#include "driver/gpio.h"


static const char *TAG = "MAIN";

static int count = 0;

esp_err_t writeCountToNVS(){
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if(ret != ESP_OK){
        return ret;
    }
    ret = nvs_set_i32(nvs_handle, "people_counter", count);
    if(ret != ESP_OK){
        return ret;
    }
    nvs_close(nvs_handle);
    return ret;
}

void app_main(void)
{

    ESP_LOGI(TAG, "app_main started");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    ret = nvs_get_i32(nvs_handle, "people_counter", (int32_t*) &count);
    switch (ret) {
        case ESP_OK:
            ESP_LOGI(TAG, "Read from NVS");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "The value is not initialized yet!");
            break;
        default :
            ESP_LOGI(TAG, "Error (%s) reading!", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "count = %i", count);
        nvs_close(nvs_handle);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(wifi_init_softap());
    ESP_ERROR_CHECK(start_webserver(&count));
    ESP_ERROR_CHECK(batteryManagmentInit());
    ESP_ERROR_CHECK(startPeopleCountingAlgorithm(&count));
}
