#define GPIO_OUTPUT_IO_BUZZER       BUZZER_PIN
#define GPIO_OUTPUT_IO_TRANSISTOR   GPIO_NUM_8
#define GPIO_OUTPUT_IO_DIAGNOSTIC   GPIO_NUM_45
/*
 * Let's say, GPIO_OUTPUT_IO_BUZZER=18, GPIO_OUTPUT_IO_TRANSISTOR=19
 * In binary representation,
 * 1ULL<<GPIO_OUTPUT_IO_BUZZER is equal to      0000000000000000000001000000000000000000 and
 * 1ULL<<GPIO_OUTPUT_IO_TRANSISTOR is equal to  0000000000000000000010000000000000000000
 * GPIO_OUTPUT_PIN_SEL                          0000000000000000000011000000000000000000
 * */
#define GPIO_OUTPUT_PIN_SEL (BIT64(GPIO_OUTPUT_IO_BUZZER) | BIT64(GPIO_OUTPUT_IO_TRANSISTOR))

#define GPIO_INPUT_TESLA_PIN_16_LED_PWM GPIO_NUM_10
#define RTC_GPIO_INPUT_TESLA_PIN_5_UP   GPIO_NUM_16
#define RTC_GPIO_INPUT_TESLA_PIN_6_DOWN GPIO_NUM_6
#define GPIO_INPUT_TESLA_PIN_5_UP_WAKEUP_LEVEL   0
#define GPIO_INPUT_TESLA_PIN_6_DOWN_WAKEUP_LEVEL 0

#define EXT1_USE_INTERNAL_PULLUPS   1
#define EXT1_USE_INTERNAL_PULLDOWNS 0

#define BOOT_BUTTON_NUM     GPIO_NUM_0
/* Use boot button as gpio input */
#define GPIO_WAKEUP_NUM     BOOT_BUTTON_NUM
/* "Boot" button is active low */
#define GPIO_WAKEUP_LEVEL   0

#define STEERING_WHEEL_IS_TURNED_ON_FLAG        1 // Overall state
#define STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG   2 // Depends on current temperature
#define WIFI_CONNECTED_FLAG                     4
#define WIFI_INITIALIZED_FLAG                   8
#define LED_PWM_IS_IGNORED_FLAG                 16
//#define MCPWM_ENABLED_FLAG                      32
#define NOTIFY_HEATING_FLAG                     64
#define SCROLL_TRIGGERED_FLAG                   128 // There are 2 triggers on 1 scroll click

#define TEMPERATURE_SENSOR_ADC1_CHANNEL ADC_CHANNEL_2 // GPIO3
#define TESLA_LED_PWM_ADC1_CHANNEL      ADC_CHANNEL_9 // GPIO10

#define ADC_ATTEN ADC_ATTEN_DB_0 // Atten = 0, effective measurement range of 100 ∼ 950 mV
#define ADC_SAMPLES 10
#define TEMPERATURE_SENSOR_SERIES_RESISTOR  1500.0f // Ohms
#define TEMPERATURE_SENSOR_V_REF            3.3f // V
#define TEMPERATURE_SENSOR_NTC_R_NOMINAL    10000.0f
#define TEMPERATURE_SENSOR_NTC_TEMP_NOMINAL 25.0f
#define TEMPERATURE_SENSOR_NTC_BETA         3380.0f

#define TEMPERATURE_HISTERESIS 1.0f

#define TIMER_WAKEUP_TIME_US (2 * 1000 * 1000)

//#define EXPECTED_PWM_LED_FREQUENCY_HZ 250
//#define PWM_LED_MISURE_CYCLES 10

typedef enum {
    HEATING_TEMP_LOW = 1,
    HEATING_TEMP_MIDDLE,
    HEATING_TEMP_HIGH
} heating_temperature_t;

typedef enum {
    HOLD_BUTTON_NO_STATUS,
    HOLD_BUTTON_OTA_STATUS,
    HOLD_BUTTON_LED_PWM_IGNORED_STATUS,
    HOLD_BUTTON_NOTIFY_HEATING_STATUS
} hold_button_status_t;

void set_flag(uint32_t *flags, uint32_t flag);
void reset_flag(uint32_t *flags, uint32_t flag);
bool read_flag(uint32_t flags, uint32_t flag);
static void reset_notify_heating_state();
static void set_notify_heating_state();
static bool is_notify_heating_state();
static void create_light_sleep_task();
static uint32_t read_adc_voltage(adc_channel_t adc_channel, adc_cali_handle_t adc_cali_handle, bool do_calibration);