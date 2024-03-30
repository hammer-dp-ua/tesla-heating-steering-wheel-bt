#include "freertos/semphr.h"

#define BUZZER_PIN GPIO_NUM_18

typedef enum {
    BLOCKING,
    NON_BLOCKING
} block_t;

typedef enum {
    BEEPS,
    SINGLE_BEEP,
    INFINITE_BEEPS,
    FAST_INFINITE_BEEPS,
    HUMAN_COUNTABLE
} beep_type_t;

struct beep_setting_s {
    beep_type_t beep_type;
    unsigned int beeps;
    block_t blocking_type;
    unsigned int single_beep_duration_ms;
    SemaphoreHandle_t chip_sleep_semaphore;
};

typedef struct beep_setting_s beep_setting_t;

void beep(beep_setting_t beep_setting);
void turn_beeper_off();