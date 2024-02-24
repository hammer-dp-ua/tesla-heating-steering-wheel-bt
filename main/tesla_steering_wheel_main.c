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

static const char *TAG = "TESLA_STEERING_WHEEL";

static bt_input_t bt_input;

static TimerHandle_t scan_bt_timer;
static TimerHandle_t temperature_read_timer;

static esp_hidh_dev_t *opened_bt_device = NULL;

static heating_temperature_t heating_temperature = HEATING_TEMP_MIDDLE;

static unsigned int general_flags;

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_temperature_sensor_handle;
static bool do_calibration_temperature_sensor = false;

void set_flag(unsigned int *flags, unsigned int flag) {
   *flags |= flag;
}

void reset_flag(unsigned int *flags, unsigned int flag) {
   *flags &= ~(*flags & flag);
}

bool read_flag(unsigned int flags, unsigned int flag) {
   return (flags & flag);
}

void beep_task(void *pvParameters)
{
    unsigned int beeps = (unsigned int) pvParameters;

    if (beeps == LONG_BEEP) {
        // Long beep
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);
    } else {
        for (unsigned char i = 0; i < beeps; i++) {
            gpio_set_level(BUZZER_PIN, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelete(NULL);
}

static void beep(unsigned int beeps)
{
    xTaskCreate(&beep_task, "beep_task", 1024, (void *) beeps, 2, NULL);
}

static void clear_bt_input()
{
    bt_input = (bt_input_t) {};
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
                    beep(LONG_BEEP);
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

    //vTaskDelete(NULL);
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
            if (param->open.status == ESP_OK) {
                //const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
                //ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                //esp_hidh_dev_dump(param->open.dev, stdout);
                beep(3);
            } else {
                //ESP_LOGE(TAG, " OPEN failed!");
                beep(10);
            }
            break;
        }
        case ESP_HIDH_BATTERY_EVENT: {
            //const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
            //ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
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
            /*ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);*/
            break;
        }
        case ESP_HIDH_CLOSE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
            //ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
            break;
        }
        default: {
            //ESP_LOGI(TAG, "EVENT: %d", event);
            break;
        }
    }
}

#define SCAN_DURATION_SECONDS 1

static void hid_scanning_task(void *pvParameters)
{
    if (esp_hidh_dev_exists(opened_bt_device)) {
        vTaskDelete(NULL);
        return;
    }
    
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
    xTaskCreate(&hid_scanning_task, "hid_scanning_task", 6 * 1024, NULL, 2, NULL);
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
        xTaskCreate(&read_temperature_task, "read_temperature_task", 3 * 1024, NULL, 2, NULL);
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
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
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

void app_main(void)
{
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif

    pins_config();

    adc_init();

    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );

#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 5 * 1024,
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
}
