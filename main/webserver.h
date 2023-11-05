#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t httpd_ws_send_frame_to_all_clients(httpd_ws_frame_t *ws_pkt);
esp_err_t start_webserver(int* countRef);