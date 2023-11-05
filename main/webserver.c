#include "webserver.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "common.h"

#define TASK_STACK_SIZE 8192
#define FILE_CHUNK_SIZE 4096

static const char *TAG = "WEB SERVER";

httpd_handle_t server = NULL;
static int* count = NULL;

esp_err_t httpd_ws_send_frame_to_all_clients(httpd_ws_frame_t *ws_pkt) {
    size_t fds = CONFIG_LWIP_MAX_LISTENING_TCP;
    int client_fds[CONFIG_LWIP_MAX_LISTENING_TCP] = {0};

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < fds; i++) {
        httpd_ws_client_info_t client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(server, client_fds[i], ws_pkt);
        }
    }

    return ESP_OK;
}

esp_err_t init_fs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

esp_err_t get_handler(httpd_req_t *req)
{
    char* filename = "/www/index.html";
    FILE* f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "File Opened");

    httpd_resp_set_type(req, "text/html");

    size_t read_bytes;
    char buf[FILE_CHUNK_SIZE];
    while(!feof(f)) {
        read_bytes = fread(buf, sizeof(buf[0]), sizeof(buf)/sizeof(buf[0]), f);
        if(ferror(f)){
            ESP_LOGE(TAG, "Error reading file");
            fclose(f);
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }
        if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK) {
            fclose(f);
            ESP_LOGE(TAG, "File sending failed!");
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }
        
        ESP_LOGI(TAG, "File Read");
    } 
    fclose(f);
    ESP_LOGI(TAG, "File sending complete");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}


esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    uint8_t buf[128] = { 0 };
    httpd_ws_frame_t ws_pkt = {
        .payload = buf
    };
    esp_err_t ret  = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf));
    if(ret != ESP_OK){
        ESP_LOGI(TAG, "Some weird error reading packet");
        return ret;
    }
    if(ws_pkt.type == HTTPD_WS_TYPE_TEXT){
        if(strcmp((char*)ws_pkt.payload,"counterreset") == 0){
            *count = 0;

            char payload[32];
            httpd_ws_frame_t frame = {
                .type = HTTPD_WS_TYPE_TEXT,
                .len = sprintf(payload, "{\n\"count\": %i\n}", *count),
                .payload = (uint8_t *)payload
            };
            ret = httpd_ws_send_frame(req, &frame);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
            }
            writeCountToNVS();
        } else if(strcmp((char*)ws_pkt.payload,"requestcounter") == 0){
            char payload[32];
            httpd_ws_frame_t frame = {
                .type = HTTPD_WS_TYPE_TEXT,
                .len = sprintf(payload, "{\n\"count\": %i\n}", *count),
                .payload = (uint8_t *)payload
            };
            ret = httpd_ws_send_frame(req, &frame);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
            }
        } else {
            ESP_LOGI(TAG, "Some odd string received");
        }
    } else {
        ESP_LOGI(TAG, "Some odd type of packet received");
    }

    return ESP_OK;
}

httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};

httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};

esp_err_t start_webserver(int* countp)
{
    count = countp;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = TASK_STACK_SIZE;

    server = NULL;

    ESP_ERROR_CHECK(init_fs());

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &ws);
    }
    return ESP_OK;
}