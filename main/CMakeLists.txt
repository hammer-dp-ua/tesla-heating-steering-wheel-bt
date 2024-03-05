set(srcs )
set(include_dirs ".")

idf_component_register(SRCS "esp_hid_gap.c" "beep.c" "ota.c" "tesla_steering_wheel_main.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp_adc app_update esp_http_client
                       PRIV_REQUIRES nvs_flash)
