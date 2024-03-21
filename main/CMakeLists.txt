set(srcs )
set(include_dirs ".")

idf_component_register(SRCS "beep.c" "ota.c" "tesla_steering_wheel_main.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES driver esp_adc app_update esp_http_client esp_wifi
                       PRIV_REQUIRES nvs_flash)