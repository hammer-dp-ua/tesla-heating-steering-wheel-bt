#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "beep.h"

void blocking_beep(unsigned int beeps)
{
    for (unsigned char i = 0; i < beeps; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void blocking_single_beep_ms(unsigned int duration_ms)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    gpio_set_level(BUZZER_PIN, 0);
}

static void blocking_single_beep_ms_task(void *pvParameters)
{
    unsigned int duration_ms = (unsigned int) pvParameters;

    blocking_single_beep_ms(duration_ms);

    vTaskDelete(NULL);
}

static void beep_task(void *pvParameters)
{
    unsigned int beeps = (unsigned int) pvParameters;

    blocking_beep(beeps);

    vTaskDelete(NULL);
}

void beep(unsigned int beeps)
{
    xTaskCreate(&beep_task, "beep", 1024, (void *) beeps, 2, NULL);
}

void single_beep_ms(unsigned int duration_ms)
{
    xTaskCreate(&blocking_single_beep_ms_task, "blocking_single_beep_ms", 1024, (void *) duration_ms, 2, NULL);
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

void infinite_beep(TaskHandle_t * pxCreatedTask)
{
    xTaskCreate(&infinite_beep_task, "infinite_beep", 1024, (void *) INFINITE_BEEP, 2, pxCreatedTask);
}

void fast_infinite_beep(TaskHandle_t * pxCreatedTask)
{
    xTaskCreate(&infinite_beep_task, "infinite_beep", 1024, (void *) FAST_INFINITE_BEEP, 2, pxCreatedTask);
}

void turn_beeper_off()
{
    gpio_set_level(BUZZER_PIN, 0);
}