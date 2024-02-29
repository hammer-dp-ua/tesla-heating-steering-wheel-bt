#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include "esp_adc/adc_oneshot.h"

#include "driver/gpio.h"

#include "smart_remote_keys.h"
#include "tesla_steering_wheel.h"
#include "ota.h"
#include "beep.h"
#include "wifi.h"

static const char *TAG = "TESLA_STEERING_WHEEL";

static bt_input_t bt_input;

static TimerHandle_t scan_bt_timer;
static TimerHandle_t temperature_read_timer;

static const TaskHandle_t update_beep_task;

static esp_hidh_dev_t *opened_bt_device = NULL;

static heating_temperature_t heating_temperature = HEATING_TEMP_MIDDLE;

static unsigned int general_flags;

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_temperature_sensor_handle;
static bool do_calibration_temperature_sensor = false;

static unsigned char wifi_retry_num = 0;

void set_flag(unsigned int *flags, unsigned int flag) {
   *flags |= flag;
}

void reset_flag(unsigned int *flags, unsigned int flag) {
   *flags &= ~(*flags & flag);
}

bool read_flag(unsigned int flags, unsigned int flag) {
   return (flags & flag);
}

static void clear_bt_input()
{
    bt_input = (bt_input_t) {};
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();

            wifi_retry_num++;

            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            //xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            wifi_retry_num = 0;

            vTaskDelete(update_beep_task);
            turn_beeper_off();
        }

        ESP_LOGI(TAG, "Connection to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        wifi_retry_num = 0;
        //xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        xTaskCreate(&ota_task, "ota", 8 * 1024, &update_beep_task, 5, NULL);
    }
}

static void connect_to_wifi()
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WIFI_SAE_MODE,
            .sae_h2e_identifier = WIFI_H2E_IDENTIFIER,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void update_software()
{
    infinite_beep(&update_beep_task);

    connect_to_wifi();
}

static void increase_heating_temperature()
{
    if (heating_temperature < HEATING_TEMP_HIGH) {
        heating_temperature++;
    }

    beep(heating_temperature);
}

static void decrease_heating_temperature()
{
    if (heating_temperature > HEATING_TEMP_LOW) {
        heating_temperature--;
    }

    beep(heating_temperature);
}

static void process_bt_input(esp_hidh_event_data_t *esp_hidh_event_data)
{
    //esp_hidh_event_data_t *esp_hidh_event_data = (esp_hidh_event_data_t *)pvParameters;

    if (esp_hidh_event_data == NULL) {
        return;
    }

    unsigned char *input_data = esp_hidh_event_data->input.data;
    unsigned short input_data_length = esp_hidh_event_data->input.length;
    
    if (input_data == NULL || input_data_length == 0) {
        return;
    }
    
    if (bt_input.validated_bytes_cnt == 0) {
        // detect a button by the first bytes data
        for (unsigned char i = 0; i < bt_button_commands_length; i++) {
            bt_button_command_t bt_button_command = bt_button_commands[i];
            const char *bt_button_command_data = bt_button_command.data;
            bool match = false;

            for (unsigned char input_data_index = 0; input_data_index < input_data_length; input_data_index++) {
                match = input_data[input_data_index] == bt_button_command_data[input_data_index];

                if (!match) {
                    break;
                }
            }

            if (match) {
                bt_input.button_candidate = bt_button_command;
                bt_input.validated_bytes_cnt = input_data_length;

                break;
            }
        }

        if (bt_input.validated_bytes_cnt == 0) {
            // Button not detected
            clear_bt_input();
        }
    } else if (bt_input.validated_bytes_cnt == bt_input.button_candidate.data_length) {
        // End of processing
        if (input_data_length == 2 && input_data[0] == 0x00 && input_data[1] == 0x00) {
            //ESP_LOGI(TAG, "Button pressed: %d", bt_input.button_candidate.button);

            switch (bt_input.button_candidate.button) {
                case START_STOP_BUTTON: {
                    beep(1);

                    if (read_flag(general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG)) {
                        reset_flag(&general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG);
                        reset_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);

                        gpio_set_level(GPIO_OUTPUT_IO_TRANSISTOR, 0);
                    } else {
                        set_flag(&general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG);
                    }
                    break;
                }
                case START_STOP_LONG_PRESS_BUTTON: {
                    single_beep_ms(500);
                    break;
                }
                case VOLUME_PLUS_BUTTON: {
                    increase_heating_temperature();
                    break;
                }
                case VOLUME_MINUS_BUTTON: {
                    decrease_heating_temperature();
                    break;
                }
                case VOLUME_PLUS_OR_MINUS_LONG_PRESS_BUTTON: {
                    break;
                }
                case FORVARD_BUTTON: {
                    break;
                }
                case FORVARD_LONG_PRESS_BUTTON: {
                    update_software();
                    break;
                }
                case BACK_BUTTON: {
                    break;
                }
                case BACK_LONG_PRESS_BUTTON: {
                    break;
                }
                default: {
                    break;
                }
            }
        }

        clear_bt_input();
    } else {
        bool match = false;

        for (unsigned char input_data_index = 0;
            (input_data_index < input_data_length) && (bt_input.validated_bytes_cnt < bt_input.button_candidate.data_length);
            input_data_index++) {

            match = input_data[input_data_index] == bt_input.button_candidate.data[bt_input.validated_bytes_cnt];

            if (!match) {
                break;
            }

            bt_input.validated_bytes_cnt++;
        }

        if (!match) {
            // Button not detected
            clear_bt_input();
        }
    }
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
            if (param->open.status == ESP_OK) {
                const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
                
                beep(3);
            } else {
                ESP_LOGE(TAG, " OPEN failed!");
                beep(10);
            }
            break;
        }
        case ESP_HIDH_BATTERY_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
            break;
        }
        case ESP_HIDH_INPUT_EVENT: {
            /* const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:",
                    ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length); */

            process_bt_input(param);

            break;
        }
        case ESP_HIDH_FEATURE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);

            ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
        }
        case ESP_HIDH_CLOSE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
            break;
        }
        default: {
            ESP_LOGI(TAG, "EVENT: %d", event);
            break;
        }
    }
}

#define SCAN_DURATION_SECONDS 1

static void hid_scanning_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    ESP_LOGI(TAG, "SCAN...");

    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);

    ESP_LOGI(TAG, "SCAN: %u results", results_len);

    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;

        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));

#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */

#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */

            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");

            r = r->next;
        }

        if (cr) {
            //open the last result
            esp_hidh_dev_t *new_opened_bt_device = esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);

            if (new_opened_bt_device != NULL) {
                opened_bt_device = new_opened_bt_device;
            }
        }

        //free the results
        esp_hid_scan_results_free(results);
    }

    vTaskDelete(NULL);
}

static void scan_bt(TimerHandle_t arg)
{
    if (!esp_hidh_dev_exists(opened_bt_device)) {
        xTaskCreate(&hid_scanning_task, "hid_scanning", 4 * 1024, NULL, 2, NULL);
    }
}

static float calculate_ntc_temperature(float temp_sensor_resistance)
{
    float temperature = temp_sensor_resistance / TEMPERATURE_SENSOR_NTC_R_NOMINAL;

    temperature = logf(temperature);
    temperature /= TEMPERATURE_SENSOR_NTC_BETA;
    temperature += 1.0f / (TEMPERATURE_SENSOR_NTC_TEMP_NOMINAL + 273.15f);
    temperature = 1.0f / temperature;
    temperature -= 273.15f;
    return temperature;
}

static void read_temperature_task(void *pvParameters)
{
    int voltage_accumulator = 0;

    for (unsigned char i = 0; i < ADC_SAMPLES; i++) {
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, TEMPERATURE_SENSOR_ADC1_CHANNEL, &adc_raw));
        //ESP_LOGI(TAG, "ADC Raw Data: %d", adc_raw);

        if (do_calibration_temperature_sensor) {
            int voltage = 0;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_temperature_sensor_handle, adc_raw, &voltage));

            voltage_accumulator += voltage;
        }
    
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    int voltage_avarage_mv = voltage_accumulator / ADC_SAMPLES;
    ESP_LOGI(TAG, "ADC calibration Voltage: %d mV", voltage_avarage_mv);

    float series_resistor_voltage = (float)voltage_avarage_mv / 1000.0f;
 
    float current = series_resistor_voltage / TEMPERATURE_SENSOR_SERIES_RESISTOR;
    float temp_sensor_resistance = (TEMPERATURE_SENSOR_V_REF - series_resistor_voltage) / current;

    ESP_LOGI(TAG, "Temperature sensor resistance: %f Ohms", temp_sensor_resistance);

    float temp = calculate_ntc_temperature(temp_sensor_resistance);

    ESP_LOGI(TAG, "Temperature: %f C", temp);

    float expected_temperature = 0.0f;
    switch (heating_temperature)
    {
        case HEATING_TEMP_LOW:
            expected_temperature = 20.0f;
            break;
        case HEATING_TEMP_MIDDLE:
            expected_temperature = 30.0f;
            break;
        case HEATING_TEMP_HIGH:
            expected_temperature = 40.0f;
            break;
        default:
            break;
    }

    if (read_flag(general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG)) {
        expected_temperature += TEMPERATURE_HISTERESIS;
    } else {
        expected_temperature -= TEMPERATURE_HISTERESIS;
    }

    bool turn_heating_on = temp < expected_temperature;
    gpio_set_level(GPIO_OUTPUT_IO_TRANSISTOR, turn_heating_on);

    if (turn_heating_on) {
        set_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);
    } else {
        reset_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);
    }

    vTaskDelete(NULL);
}

static void read_temperature(TimerHandle_t arg)
{
    if (read_flag(general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG)) {
        xTaskCreate(&read_temperature_task, "read_temperature", 3 * 1024, NULL, 2, NULL);
    }
}

static void pins_config()
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_config_t io_diag_conf = {};
    io_diag_conf.intr_type = GPIO_INTR_DISABLE;
    io_diag_conf.mode = GPIO_MODE_OUTPUT;
    io_diag_conf.pin_bit_mask = (1ULL<<GPIO_OUTPUT_IO_DIAGNOSTIC);
    io_diag_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_diag_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_diag_conf);
}

// ADC Calibration
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");

        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };

        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);

        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_init()
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, // 4096
        .atten = ADC_ATTEN
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TEMPERATURE_SENSOR_ADC1_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration_temperature_sensor = adc_calibration_init(ADC_UNIT_1, TEMPERATURE_SENSOR_ADC1_CHANNEL, ADC_ATTEN, &adc_cali_temperature_sensor_handle);
}

static void get_debug_task_info(char *task_name)
{
    TaskHandle_t xHandle = xTaskGetHandle(task_name);
    configASSERT( xHandle );

    TaskStatus_t xTaskDetails;
    vTaskGetInfo(xHandle, &xTaskDetails, pdTRUE, eInvalid );
    ESP_LOGI(TAG, "Task '%s'. Stack high water mark: %u. Task number: %u", xTaskDetails.pcTaskName, (unsigned int)xTaskDetails.usStackHighWaterMark, xTaskDetails.xTaskNumber);
}

static bool diagnostic(void)
{
    ESP_LOGI(TAG, "Diagnostics (5 sec)...");

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(GPIO_OUTPUT_IO_DIAGNOSTIC);

    gpio_reset_pin(GPIO_OUTPUT_IO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

void app_main(void)
{
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif

    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // get sha256 digest for the partition table
    partition.address   = ESP_PARTITION_TABLE_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type      = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    pins_config();

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // run diagnostic function ...
            bool diagnostic_is_ok = diagnostic();

            if (diagnostic_is_ok) {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
                blocking_beep(2);
            } else {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    adc_init();

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );

#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */

    esp_hidh_config_t config = {
        .callback = hidh_callback, // "esp_ble_hidh_ev" task
        .event_stack_size = 3 * 1024,
        .callback_arg = NULL,
    };

    ESP_ERROR_CHECK(esp_hidh_init(&config));

    scan_bt(NULL);

    int scan_bt_timer_tmr_id = 0;
    scan_bt_timer = xTimerCreate("scan_bt_timer", (20000 / portTICK_PERIOD_MS), pdTRUE, (void *) &scan_bt_timer_tmr_id, scan_bt);
    xTimerStart(scan_bt_timer, portMAX_DELAY);

    int temperature_read_timer_tmr_id = 1;
    temperature_read_timer = xTimerCreate("temperature_read_timer", (2000 / portTICK_PERIOD_MS), pdTRUE, (void *) &temperature_read_timer_tmr_id, read_temperature);
    xTimerStart(temperature_read_timer, portMAX_DELAY);

    /* while (1) {
        get_debug_task_info("BTU_TASK");
        get_debug_task_info("read_temperature");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    } */
}
