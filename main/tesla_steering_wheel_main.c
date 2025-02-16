#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "esp_adc/adc_oneshot.h"

#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "tesla_steering_wheel.h"
#include "ota.h"
#include "beep.h"
#include "wifi.h"

static const char *TAG = "TESLA_STEERING_WHEEL";

static TaskHandle_t light_sleep_task_handle = NULL;

static gptimer_handle_t gptimer = NULL;

static mcpwm_cap_channel_handle_t mcpwm_cap_channel_handle = NULL;
static mcpwm_cap_timer_handle_t mcpwm_cap_timer_handle = NULL;

static heating_temperature_t heating_temperature = HEATING_TEMP_MIDDLE;

static SemaphoreHandle_t chip_sleep_semaphore = NULL;

static uint32_t general_flags = 0;

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc_cali_temperature_sensor_handle = NULL;
static adc_cali_handle_t adc_cali_tesla_led_pwm_handle = NULL;
static bool do_calibration_temperature_sensor = false;
static bool do_calibration_tesla_led_pwm = false;

static uint32_t external_led_pwm_ticks = 0;

void set_flag(uint32_t *flags, uint32_t flag)
{
    *flags |= flag;
}

void reset_flag(uint32_t *flags, uint32_t flag)
{
    *flags &= ~(*flags & flag);
}

bool read_flag(uint32_t flags, uint32_t flag)
{
    return (flags & flag);
}

static uint32_t utils_get_system_timestamp_ms()
{
    return esp_log_timestamp();
}

static void ota_event_handler(ota_status_t ota_status, unsigned char error_number)
{
    if (ota_status == OTA_ERROR) {
        turn_beeper_off();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        beep_setting_t beep_settings = {
            .beep_type = HUMAN_COUNTABLE,
            .beeps = error_number,
            .blocking_type = BLOCKING
        };
        beep(beep_settings);

        esp_wifi_disconnect();

        create_light_sleep_task();
    } else if (ota_status == OTA_OK) {
        beep_setting_t beep_settings = {
            .beep_type = SINGLE_BEEP,
            .blocking_type = BLOCKING,
            .single_beep_duration_ms = 2000
        };
        beep(beep_settings);
    } else if (ota_status == OTA_START_DOWNLOADING) {
        beep_setting_t beep_settings = {
            .beep_type = FAST_INFINITE_BEEPS,
            .blocking_type = NON_BLOCKING
        };
        beep(beep_settings);
    }
}

static void create_ota_task()
{
    xTaskCreate(&ota_task, "ota", 8 * 1024, &ota_event_handler, 5, NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        reset_flag(&general_flags, WIFI_CONNECTED_FLAG);

        turn_beeper_off();

        ESP_LOGI(TAG, "Connection to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        set_flag(&general_flags, WIFI_CONNECTED_FLAG);

        create_ota_task();
    }
}

static void init_connect_to_wifi()
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

    set_flag(&general_flags, WIFI_INITIALIZED_FLAG);

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void update_software()
{
    beep_setting_t beep_settings = {
        .beep_type = INFINITE_BEEPS,
        .blocking_type = NON_BLOCKING
    };
    beep(beep_settings);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    if (read_flag(general_flags, WIFI_INITIALIZED_FLAG)) {
        esp_wifi_connect();
    } else {
        init_connect_to_wifi();
    }
}

static void increase_heating_temperature()
{
    if (heating_temperature < HEATING_TEMP_HIGH) {
        heating_temperature++;

        beep_setting_t beep_settings = {
            .beep_type = BEEPS,
            .beeps = heating_temperature,
            .blocking_type = BLOCKING
        };
        beep(beep_settings);
    }
}

static void decrease_heating_temperature()
{
    if (heating_temperature > HEATING_TEMP_LOW) {
        heating_temperature--;

        beep_setting_t beep_settings = {
            .beep_type = BEEPS,
            .beeps = heating_temperature,
            .blocking_type = BLOCKING
        };
        beep(beep_settings);
    }
}

static void turn_steering_wheel_off()
{
    reset_flag(&general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG);
    reset_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);

    gpio_set_level(GPIO_OUTPUT_IO_TRANSISTOR, 0);
    gpio_hold_dis(GPIO_OUTPUT_IO_TRANSISTOR);
}

static void turn_steering_wheel_on()
{
    set_flag(&general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG);
}

static bool is_steering_wheel_turned_on()
{
    return read_flag(general_flags, STEERING_WHEEL_IS_TURNED_ON_FLAG);
}

/*static uint32_t get_pwm_frequency(uint32_t pwm_ticks)
{
    uint32_t processor_frequency = (uint32_t)esp_clk_apb_freq();
    return (pwm_ticks > 0 && processor_frequency > pwm_ticks)
            ? (uint32_t)(processor_frequency / pwm_ticks)
            : 0;
}*/

static bool is_tesla_led_turned_on()
{
    //uint32_t pwm_frequency = get_pwm_frequency(external_led_pwm_ticks);
    //return pwm_frequency > (EXPECTED_PWM_LED_FREQUENCY_HZ - 50) && pwm_frequency < (EXPECTED_PWM_LED_FREQUENCY_HZ + 50);
    uint32_t voltage_mv =
            read_adc_voltage(TESLA_LED_PWM_ADC1_CHANNEL, adc_cali_tesla_led_pwm_handle, do_calibration_tesla_led_pwm);
    return voltage_mv > 250; // Actual measured value at 15V was 330mV
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

static uint32_t read_adc_voltage(adc_channel_t adc_channel, adc_cali_handle_t adc_cali_handle, bool do_calibration)
{
    uint32_t voltage_accumulator = 0;

    for (unsigned char i = 0; i < ADC_SAMPLES; i++) {
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel, &adc_raw));
        //ESP_LOGI(TAG, "ADC Raw Data: %d", adc_raw);

        if (do_calibration) {
            int voltage = 0;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage));

            voltage_accumulator += voltage;
        }
    
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    
    uint32_t voltage_avarage_mv = voltage_accumulator / ADC_SAMPLES;
    //ESP_LOGI(TAG, "ADC calibration Voltage: %d mV", (unsigned int)voltage_avarage_mv);
    return voltage_avarage_mv;
}

static void read_temperature_and_apply()
{
    uint32_t voltage_mv =
            read_adc_voltage(TEMPERATURE_SENSOR_ADC1_CHANNEL, adc_cali_temperature_sensor_handle, do_calibration_temperature_sensor);

    float series_resistor_voltage = (float)voltage_mv / 1000.0f;
 
    float current = series_resistor_voltage / TEMPERATURE_SENSOR_SERIES_RESISTOR;
    float temp_sensor_resistance = (TEMPERATURE_SENSOR_V_REF - series_resistor_voltage) / current;

    //ESP_LOGI(TAG, "Temperature sensor resistance: %f Ohms", temp_sensor_resistance);

    float temp = calculate_ntc_temperature(temp_sensor_resistance);

    //ESP_LOGI(TAG, "Temperature: %f C", temp);

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

    // histeresis calculation
    /*if (read_flag(general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG)) {
        expected_temperature += TEMPERATURE_HISTERESIS;
    } else {
        expected_temperature -= TEMPERATURE_HISTERESIS;
    }*/

    bool turn_heating_on = temp < expected_temperature;
    gpio_set_level(GPIO_OUTPUT_IO_TRANSISTOR, turn_heating_on);

    if (is_notify_heating_state() &&
            turn_heating_on != read_flag(general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG)) {
        beep_setting_t beep_settings = {
            .beep_type = BEEPS,
            .beeps = 1,
            .blocking_type = BLOCKING
        };
        beep(beep_settings);
    }
    
    if (turn_heating_on) {
        gpio_hold_en(GPIO_OUTPUT_IO_TRANSISTOR);
        set_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);
    } else {
        gpio_hold_dis(GPIO_OUTPUT_IO_TRANSISTOR);
        reset_flag(&general_flags, STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG);
    }
}

/*static void enable_pwm_measurement()
{
    if (!read_flag(general_flags, MCPWM_ENABLED_FLAG)) {
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_handle));
        ESP_ERROR_CHECK(mcpwm_capture_timer_enable(mcpwm_cap_timer_handle));
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer_handle));

        set_flag(&general_flags, MCPWM_ENABLED_FLAG);
    }
}

static void disable_pwm_measurement()
{
    reset_flag(&general_flags, MCPWM_ENABLED_FLAG);
}*/

/*static bool external_led_pwm_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_prev = 0;
    static unsigned char callback_counter = 0;

    BaseType_t task_wakeup = pdFALSE;

    callback_counter++;

    external_led_pwm_ticks = edata->cap_value - cap_val_prev;
    cap_val_prev = edata->cap_value;

    if (callback_counter >= PWM_LED_MISURE_CYCLES) {
        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_handle));
        ESP_ERROR_CHECK(mcpwm_capture_timer_stop(mcpwm_cap_timer_handle));
        ESP_ERROR_CHECK(mcpwm_capture_timer_disable(mcpwm_cap_timer_handle));

        disable_pwm_measurement();

        cap_val_prev = 0;
        callback_counter = 0;

        xTaskNotifyFromISR(light_sleep_task_handle, NULL, eSetValueWithOverwrite, &task_wakeup);
    }
    
    return task_wakeup == pdTRUE;
}*/

/*static void external_led_pwm_config()
{
    // Install capture timer
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
        // In ESP32, the parameter is invalid, the capture timer resolution is always equal to the MCPWM_CAPTURE_CLK_SRC_APB
        //.resolution_hz 
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &mcpwm_cap_timer_handle));

    // Install capture channel
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = GPIO_INPUT_TESLA_PIN_16_LED_PWM,
        .prescale = 1,
        .flags.neg_edge = false, // capture on pos edge only
        .flags.pos_edge = true,
        .flags.pull_up = false // not to pull up internally
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(mcpwm_cap_timer_handle, &cap_ch_conf, &mcpwm_cap_channel_handle));

    // Register capture callback
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = external_led_pwm_callback
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_handle, &cbs, NULL));
}*/

static hold_button_status_t wait_gpio_inactive(gpio_num_t gpio_num)
{
    uint32_t button_pressed_time_ms = utils_get_system_timestamp_ms();
    hold_button_status_t status = HOLD_BUTTON_NO_STATUS;
    
    while (gpio_get_level(gpio_num) == GPIO_WAKEUP_LEVEL) {
        uint32_t hold_time_ms = utils_get_system_timestamp_ms() - button_pressed_time_ms;

        if (hold_time_ms >= 3000 && status == (HOLD_BUTTON_OTA_STATUS - 1)) {
            status = HOLD_BUTTON_OTA_STATUS;

            beep_setting_t beep_setting = {
                .beep_type = BEEPS,
                .beeps = HOLD_BUTTON_OTA_STATUS,
                .blocking_type = BLOCKING
            };
            beep(beep_setting);
        } else if (hold_time_ms >= 5000 && status == (HOLD_BUTTON_LED_PWM_IGNORED_STATUS - 1)) {
            status = HOLD_BUTTON_LED_PWM_IGNORED_STATUS;

            beep_setting_t beep_setting = {
                .beep_type = BEEPS,
                .beeps = HOLD_BUTTON_LED_PWM_IGNORED_STATUS,
                .blocking_type = BLOCKING
            };
            beep(beep_setting);
        } else if (hold_time_ms >= 7000 && status == (HOLD_BUTTON_NOTIFY_HEATING_STATUS - 1)) {
            status = HOLD_BUTTON_NOTIFY_HEATING_STATUS;

            beep_setting_t beep_setting = {
                .beep_type = BEEPS,
                .beeps = HOLD_BUTTON_NOTIFY_HEATING_STATUS,
                .blocking_type = BLOCKING
            };
            beep(beep_setting);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return status;
}

static esp_err_t register_gpio_wakeup()
{
    /* Initialize GPIO */
    gpio_config_t config = {
        .pin_bit_mask = (BIT64(GPIO_WAKEUP_NUM)),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = false,
        .pull_up_en = false, // 'false' for steering wheel, 'true' for a development board
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    /* Enable wake up from GPIO */
    ESP_ERROR_CHECK(gpio_wakeup_enable(GPIO_WAKEUP_NUM, GPIO_WAKEUP_LEVEL == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
    
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    wait_gpio_inactive(GPIO_WAKEUP_NUM);
    return ESP_OK;
}

static bool register_rtc_gpio_wakeup(gpio_num_t pins[], unsigned char pins_amount)
{
    /*for (unsigned char i = 0; i < pins_amount; i++) {
        ESP_LOGI(TAG, "rtc_gpio_get_level: %d", (int) rtc_gpio_get_level(pins[i]));
    }*/

    if (pins_amount > 1) {
        uint32_t first_pin_level = rtc_gpio_get_level(pins[0]);

        for (unsigned char i = 1; i < pins_amount; i++) {
            if (first_pin_level != rtc_gpio_get_level(pins[i])) {
                return false;
            }
        }
    }
    
    esp_sleep_ext1_wakeup_mode_t ext_wakeup_mode;

    if (rtc_gpio_get_level(pins[0])) {
        ext_wakeup_mode = ESP_EXT1_WAKEUP_ANY_LOW;
    } else {
        ext_wakeup_mode = ESP_EXT1_WAKEUP_ANY_HIGH;
    }

    uint64_t pins_mask = 0;
    for (unsigned char i = 0; i < pins_amount; i++) {
        pins_mask |= BIT64(pins[i]);
    }
    //ESP_LOGI(TAG, "pins_mask: %d", (unsigned int)pins_mask);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(pins_mask, ext_wakeup_mode));
    
/* If there are no external pull-up/downs, tie wakeup pins to inactive level with internal pull-up/downs via RTC IO
 * during deepsleep. However, RTC IO relies on the RTC_PERIPH power domain. Keeping this power domain on will
 * increase some power comsumption. However, if we turn off the RTC_PERIPH domain or if certain chips lack the RTC_PERIPH
 * domain, we will use the HOLD feature to maintain the pull-up and pull-down on the pins during sleep.*/
#if EXT1_USE_INTERNAL_PULLUPS
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
    for (unsigned char i = 0; i < pins_amount; i++) {
        gpio_num_t pin = pins[i];

        // For dev board:
        /*if (ext_wakeup_mode) {
            ESP_ERROR_CHECK(rtc_gpio_pullup_dis(pin));
            ESP_ERROR_CHECK(rtc_gpio_pulldown_en(pin));
        } else {
            ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(pin));
            ESP_ERROR_CHECK(rtc_gpio_pullup_en(pin));
        }*/

       ESP_ERROR_CHECK(rtc_gpio_pullup_dis(pin));
       ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(pin));
    }
#else // SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
    for (unsigned char i = 0; i < pins_amount; i++) {
        gpio_num_t pin = pins[i];

        if (ext_wakeup_mode) {
            ESP_ERROR_CHECK(gpio_pullup_dis(pin));
            ESP_ERROR_CHECK(gpio_pulldown_en(pin));
        } else {
            ESP_ERROR_CHECK(gpio_pulldown_dis(pin));
            ESP_ERROR_CHECK(gpio_pullup_en(pin));
        }
    }
#endif
#endif // EXT1_USE_INTERNAL_PULLUPS
    return true;
}

static void pins_config()
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set, e.g.GPIO18/19
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
    io_diag_conf.pin_bit_mask = BIT64(GPIO_OUTPUT_IO_DIAGNOSTIC);
    io_diag_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_diag_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_diag_conf);

    gpio_config_t rtc_gpio_conf = {};
    rtc_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    rtc_gpio_conf.mode = GPIO_MODE_INPUT;
    rtc_gpio_conf.pin_bit_mask = (BIT64(RTC_GPIO_INPUT_TESLA_PIN_5_UP) | BIT64(RTC_GPIO_INPUT_TESLA_PIN_6_DOWN));
    rtc_gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE; //EXT1_USE_INTERNAL_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    rtc_gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; //EXT1_USE_INTERNAL_PULLDOWNS ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_config(&rtc_gpio_conf);
}

// ADC Calibration
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        //ESP_LOGI(TAG, "Calibration scheme version is %s", "Curve Fitting");

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
        //ESP_LOGI(TAG, "Calibration success");
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
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, // 4096
        .atten = ADC_ATTEN
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TEMPERATURE_SENSOR_ADC1_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TESLA_LED_PWM_ADC1_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration_temperature_sensor = adc_calibration_init(ADC_UNIT_1, TEMPERATURE_SENSOR_ADC1_CHANNEL, ADC_ATTEN, &adc_cali_temperature_sensor_handle);
    do_calibration_tesla_led_pwm = adc_calibration_init(ADC_UNIT_1, TESLA_LED_PWM_ADC1_CHANNEL, ADC_ATTEN, &adc_cali_tesla_led_pwm_handle);
}

// Enable configUSE_TRACE_FACILITY, configUSE_STATS_FORMATTING_FUNCTIONS, xCoreID
#if INCLUDE_xTaskGetHandle == 1 && configUSE_TRACE_FACILITY == 1
static void get_debug_task_info(char *task_name)
{
    TaskHandle_t xHandle = xTaskGetHandle(task_name);
    configASSERT( xHandle );

    TaskStatus_t xTaskDetails;
    vTaskGetInfo(xHandle, &xTaskDetails, pdTRUE, eInvalid );
    ESP_LOGI(TAG, "Task '%s'. Stack high water mark: %u. Task number: %u",
            xTaskDetails.pcTaskName,
            (uint32_t)xTaskDetails.usStackHighWaterMark,
            xTaskDetails.xTaskNumber);
}
#endif

static bool diagnostic(void)
{
    ESP_LOGI(TAG, "Diagnostics (5 sec)...");

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(GPIO_OUTPUT_IO_DIAGNOSTIC);

    gpio_reset_pin(GPIO_OUTPUT_IO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

static void reset_ignore_led_pwm() {
    reset_flag(&general_flags, LED_PWM_IS_IGNORED_FLAG);
}

static void ignore_led_pwm() {
    set_flag(&general_flags, LED_PWM_IS_IGNORED_FLAG);
}

static bool is_led_pwm_ignored() {
    return read_flag(general_flags, LED_PWM_IS_IGNORED_FLAG);
}

static void reset_notify_heating_state() {
    reset_flag(&general_flags, NOTIFY_HEATING_FLAG);
}

static void set_notify_heating_state() {
    set_flag(&general_flags, NOTIFY_HEATING_FLAG);
}

static bool is_notify_heating_state() {
    return read_flag(general_flags, NOTIFY_HEATING_FLAG);
}

static void configure_scroll_pins() {
    gpio_num_t rtc_pins[] = {RTC_GPIO_INPUT_TESLA_PIN_5_UP, RTC_GPIO_INPUT_TESLA_PIN_6_DOWN};
    bool configured = register_rtc_gpio_wakeup(rtc_pins, 2);

    if (configured) {
        reset_flag(&general_flags, SCROLL_TRIGGERED_FLAG);
    } else {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static void light_sleep_task(void *args)
{
    //float expected_pwm_led_cycle_sec = 1.0f / (float)EXPECTED_PWM_LED_FREQUENCY_HZ;
    //uint32_t time_to_wait_for_pwm_led_measurement_ms = (uint32_t)(2.0f * 1000.0f * (float)PWM_LED_MISURE_CYCLES * expected_pwm_led_cycle_sec);

    bool turning_steering_wheel_on = false;
    
    while (true) {
        xSemaphoreTake(chip_sleep_semaphore, portMAX_DELAY);

        /* Enter sleep mode */
        esp_light_sleep_start();
        xSemaphoreGive(chip_sleep_semaphore);

        /* Determine wake up reason */
        const char* wakeup_reason;
        esp_sleep_wakeup_cause_t esp_sleep_wakeup_cause = esp_sleep_get_wakeup_cause();
        switch (esp_sleep_wakeup_cause) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                wakeup_reason = "pin";
                break;
            case ESP_SLEEP_WAKEUP_EXT1:
                uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
                
                if (wakeup_pin_mask != 0) {
                    int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                    unsigned int pin_level = rtc_gpio_get_level(pin);

                    if (read_flag(general_flags, SCROLL_TRIGGERED_FLAG)) {
                        configure_scroll_pins();

                        if (is_steering_wheel_turned_on()) {
                            if (pin == RTC_GPIO_INPUT_TESLA_PIN_6_DOWN) {
                                increase_heating_temperature();
                            } else if (pin == RTC_GPIO_INPUT_TESLA_PIN_5_UP) {
                                decrease_heating_temperature();
                            }
                        }

                        /*beep_setting_t beep_setting = {
                            .beep_type = SINGLE_BEEP,
                            .blocking_type = BLOCKING,
                            .single_beep_duration_ms = 500
                        };
                        beep(beep_setting);

                        ESP_LOGI(TAG, "beep");*/
                    } else {
                        set_flag(&general_flags, SCROLL_TRIGGERED_FLAG);

                        if (pin == RTC_GPIO_INPUT_TESLA_PIN_5_UP) {
                            gpio_num_t rtc_pins[] = {RTC_GPIO_INPUT_TESLA_PIN_6_DOWN};
                            register_rtc_gpio_wakeup(rtc_pins, 1);
                        } else if (pin == RTC_GPIO_INPUT_TESLA_PIN_6_DOWN) {
                            gpio_num_t rtc_pins[] = {RTC_GPIO_INPUT_TESLA_PIN_5_UP};
                            register_rtc_gpio_wakeup(rtc_pins, 1);
                        }
                    }

                    //ESP_LOGI(TAG, "Wake up from EXT1 %d. Level: %d", (unsigned int) pin, pin_level);
                } else {
                    configure_scroll_pins();

                    //ESP_LOGI(TAG, "Wake up from EXT1");
                }

                //rtc_gpio_deinit(RTC_GPIO_INPUT_TESLA_PIN_5_UP);
                //rtc_gpio_deinit(RTC_GPIO_INPUT_TESLA_PIN_6_DOWN);

                continue;
            case ESP_SLEEP_WAKEUP_UART:
                wakeup_reason = "uart";
                /* Hang-up for a while to switch and execuse the uart task
                 * Otherwise the chip may fall sleep again before running uart task */
                vTaskDelay(1);
                break;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
            case ESP_SLEEP_WAKEUP_TOUCHPAD:
                wakeup_reason = "touch";
                break;
#endif
            default:
                wakeup_reason = "other";
                
                configure_scroll_pins();
                break;
        }

        /*if (esp_sleep_wakeup_cause) {
            ESP_LOGI(TAG, "Returned from light sleep, reason: %s. Wakeup cause: %d.", wakeup_reason, (unsigned int) esp_sleep_wakeup_cause);
        } else {
            printf(".");
        }*/
    
        if (esp_sleep_wakeup_cause == ESP_SLEEP_WAKEUP_GPIO) {
            /*uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                ESP_LOGI(TAG, "Wake up from GPIO %d", pin);
            } else {
                ESP_LOGI(TAG, "Wake up from GPIO");
            }*/

            /* Waiting for the gpio inactive, or the chip will continously trigger wakeup*/
            hold_button_status_t hold_button_status = wait_gpio_inactive(GPIO_WAKEUP_NUM);

            //esp_sleep_get_ext1_wakeup_status();

            if (hold_button_status == HOLD_BUTTON_OTA_STATUS) {
                turn_steering_wheel_off();
                update_software();
                vTaskDelete(NULL);
            } else if (hold_button_status == HOLD_BUTTON_LED_PWM_IGNORED_STATUS) {
                if (is_led_pwm_ignored()) {
                    reset_ignore_led_pwm();
                } else {
                    ignore_led_pwm();
                }
            } else if (hold_button_status == HOLD_BUTTON_NOTIFY_HEATING_STATUS) {
                if (is_notify_heating_state()) {
                    reset_notify_heating_state();
                } else {
                    set_notify_heating_state();
                }
            } else {
                if (is_steering_wheel_turned_on()) {
                    turn_steering_wheel_off();

                    beep_setting_t beep_setting = {
                        .beep_type = BEEPS,
                        .beeps = 2,
                        .blocking_type = BLOCKING
                    };
                    beep(beep_setting);
                } else {
                    turn_steering_wheel_on();
                    turning_steering_wheel_on = true;
                }
            }
        }

        if (!is_steering_wheel_turned_on()) {
            continue;
        }

        bool led_pwm_ignored = is_led_pwm_ignored();

        /*if (!led_pwm_ignored) {
            enable_pwm_measurement();
        }*/
        
        //if (led_pwm_ignored || xTaskNotifyWait(0x00, ULONG_MAX, NULL, pdMS_TO_TICKS(time_to_wait_for_pwm_led_measurement_ms)) == pdTRUE) {
        if (led_pwm_ignored || is_tesla_led_turned_on()) {
            if (turning_steering_wheel_on) {
                beep_setting_t beep_setting = {
                    .beep_type = BEEPS,
                    .beeps = 1,
                    .blocking_type = BLOCKING
                };
                beep(beep_setting);
            }
        } else {
            turn_steering_wheel_off();
            turning_steering_wheel_on = false;
            continue;
        }

        turning_steering_wheel_on = false;

        read_temperature_and_apply();
    }
}

static void create_light_sleep_task() {
    xTaskCreate(light_sleep_task, "light_sleep", 4 * 1024, NULL, 2, &light_sleep_task_handle);
}

void app_main(void)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
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
    //external_led_pwm_config();
    register_gpio_wakeup();

    configure_scroll_pins();

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // run diagnostic function ...
            bool diagnostic_is_ok = diagnostic();

            if (diagnostic_is_ok) {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
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

    /*beep_setting_t beep_setting = {
        .beep_type = SINGLE_BEEP,
        .blocking_type = BLOCKING,
        .single_beep_duration_ms = 500
    };
    beep(beep_setting);*/

    adc_init();
    
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_TIME_US));

#if INCLUDE_xTaskGetHandle == 1 && configUSE_TRACE_FACILITY == 1
    while (1) {
        get_debug_task_info("BTU_TASK");
        get_debug_task_info("read_temperature");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
#endif

    chip_sleep_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(chip_sleep_semaphore);

    create_light_sleep_task();
}
