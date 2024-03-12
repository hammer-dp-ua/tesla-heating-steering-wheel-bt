#define GPIO_OUTPUT_IO_BUZZER       GPIO_NUM_18
#define GPIO_OUTPUT_IO_TRANSISTOR   GPIO_NUM_8
#define GPIO_OUTPUT_IO_LED          GPIO_NUM_4
#define GPIO_OUTPUT_IO_DIAGNOSTIC   GPIO_NUM_45
/*
 * Let's say, GPIO_OUTPUT_IO_BUZZER=18, GPIO_OUTPUT_IO_TRANSISTOR=19
 * In binary representation,
 * 1ULL<<GPIO_OUTPUT_IO_BUZZER is equal to      0000000000000000000001000000000000000000 and
 * 1ULL<<GPIO_OUTPUT_IO_TRANSISTOR is equal to  0000000000000000000010000000000000000000
 * GPIO_OUTPUT_PIN_SEL                          0000000000000000000011000000000000000000
 * */
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_IO_BUZZER) | (1ULL<<GPIO_OUTPUT_IO_TRANSISTOR) | (1ULL<<GPIO_OUTPUT_IO_LED))

//#define GPIO_INPUT_IO_EXTERNAL_LED GPIO_NUM_10

#define STEERING_WHEEL_IS_TURNED_ON_FLAG        1 // Overall state
#define STEERING_WHEEL_HEATING_IS_ACTIVE_FLAG   2 // Depends on current temperature
#define WIFI_CONNECTED_FLAG                     4
#define WIFI_INITIALIZED_FLAG                   8
#define TESLA_LED_IS_TURNED_ON                  16

#define TEMPERATURE_SENSOR_ADC1_CHANNEL ADC_CHANNEL_2 // GPIO3
#define EXTERNAL_TESLA_LED_ADC1_CHANNEL ADC_CHANNEL_9 // GPIO_NUM_10

#define ADC_ATTEN ADC_ATTEN_DB_0 // Atten = 0, effective measurement range of 100 ∼ 950 mV
#define ADC_SAMPLES 5
#define TEMPERATURE_SENSOR_SERIES_RESISTOR  1500.0f // Ohms
#define TEMPERATURE_SENSOR_V_REF            3.3f // V
#define TEMPERATURE_SENSOR_NTC_R_NOMINAL    10000.0f
#define TEMPERATURE_SENSOR_NTC_TEMP_NOMINAL 25.0f
#define TEMPERATURE_SENSOR_NTC_BETA         3380.0f

#define TEMPERATURE_HISTERESIS 1.0f

typedef enum {
    HEATING_TEMP_LOW = 1,
    HEATING_TEMP_MIDDLE,
    HEATING_TEMP_HIGH
} heating_temperature_t;

struct bt_input_s {
    bt_button_command_t button_candidate;
    unsigned char validated_bytes_cnt;

};

typedef struct bt_input_s bt_input_t;

void set_flag(unsigned int *flags, unsigned int flag);
void reset_flag(unsigned int *flags, unsigned int flag);
bool read_flag(unsigned int flags, unsigned int flag);