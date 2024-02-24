set(srcs
         "esp_hid_gap.c")
set(include_dirs ".")

idf_component_register(SRCS "tesla_steering_wheel_main.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid driver esp_adc
                       PRIV_REQUIRES nvs_flash)
