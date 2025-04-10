set(SOURCES "main.cpp")
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "../Inc"
                       REQUIRES freertos
                                esp_common 
                                driver 
                                nvs_flash  
                                esp_timer
                                esp_wifi
                                esp_http_server
                                json
                                bt)