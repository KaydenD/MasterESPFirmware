idf_component_register(SRCS "main.c" "wifi.c" "webserver.c" "battery.c" "peoplecounting.c"
                    INCLUDE_DIRS ".")

set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../www")
if(EXISTS ${WEB_SRC_DIR})
    spiffs_create_partition_image(spiffs ${WEB_SRC_DIR} FLASH_IN_PROJECT)
else()
    message(FATAL_ERROR "${WEB_SRC_DIR} doesn't exit.")
endif()
