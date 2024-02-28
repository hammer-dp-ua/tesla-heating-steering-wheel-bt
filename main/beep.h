#define LONG_BEEP 0xFF
#define INFINITE_BEEP 0xFE
#define FAST_INFINITE_BEEP 0xFD

#define BUZZER_PIN GPIO_NUM_18

void blocking_beep(unsigned int beeps);
void long_blocking_beep();
void beep(unsigned int beeps);
void long_beep();
void infinite_beep(TaskHandle_t * const pxCreatedTask);
void fast_infinite_beep(TaskHandle_t * const pxCreatedTask);
void turn_beeper_off();