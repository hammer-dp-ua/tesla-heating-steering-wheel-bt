#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "beep.h"

void blocking_beep(unsigned int beeps)
{
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
}

void long_blocking_beep()
{
    blocking_beep(LONG_BEEP);
}

static void beep_task(void *pvParameters)
{
    unsigned int beeps = (unsigned int) pvParameters;

    blocking_beep(beeps);

    vTaskDelete(NULL);
}

void beep(unsigned int beeps)
{
    xTaskCreate(&beep_task, "beep_task", 1024, (void *) beeps, 2, NULL);
}

void long_beep()
{
    beep(LONG_BEEP);
}

static void infinite_beep_task(void *pvParameters)
{
    unsigned int beep_type = (unsigned int) pvParameters;

    while (1) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);

        if (beep_type == INFINITE_BEEP) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else if (beep_type == FAST_INFINITE_BEEP) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}

void infinite_beep(TaskHandle_t * const pxCreatedTask)
{
    xTaskCreate(&infinite_beep_task, "infinite_beep_task", 1024, (void *) INFINITE_BEEP, 2, pxCreatedTask);
}

void fast_infinite_beep(TaskHandle_t * const pxCreatedTask)
{
    xTaskCreate(&infinite_beep_task, "infinite_beep_task", 1024, (void *) FAST_INFINITE_BEEP, 2, pxCreatedTask);
}

void turn_beeper_off()
{
    gpio_set_level(BUZZER_PIN, 0);
}