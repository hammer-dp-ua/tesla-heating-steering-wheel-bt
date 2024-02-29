#define INFINITE_BEEP 0xFE
#define FAST_INFINITE_BEEP 0xFD

#define BUZZER_PIN GPIO_NUM_18

void blocking_beep(unsigned int beeps);
void blocking_single_beep_ms(unsigned int duration_ms);
void long_blocking_beep();
void beep(unsigned int beeps);
void single_beep_ms(unsigned int duration_ms);
void infinite_beep(TaskHandle_t * pxCreatedTask);
void fast_infinite_beep(TaskHandle_t * pxCreatedTask);
void turn_beeper_off();