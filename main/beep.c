#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "beep.h"

static TaskHandle_t beep_task_handle = NULL;
static SemaphoreHandle_t chip_sleep_semaphore = NULL;

static void human_countable_blocking_beep(unsigned int beeps)
{
    for (unsigned char i = 1; i <= beeps; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);

        if (i % 3 == 0) {
            vTaskDelay(700 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

static void blocking_beep(unsigned int beeps)
{
    for (unsigned char i = 0; i < beeps; i++) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void blocking_single_beep_ms(unsigned int duration_ms)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    gpio_set_level(BUZZER_PIN, 0);
}

static void unlock()
{
    if (chip_sleep_semaphore != NULL) {
        xSemaphoreGive(chip_sleep_semaphore);
        chip_sleep_semaphore = NULL;
    }
}

static void single_beep_ms_task(void *pvParameters)
{
    unsigned int duration_ms = (unsigned int) pvParameters;

    blocking_single_beep_ms(duration_ms);

    unlock();
    vTaskDelete(NULL);
}

static void beep_task(void *pvParameters)
{
    unsigned int beeps = (unsigned int) pvParameters;

    blocking_beep(beeps);

    unlock();
    vTaskDelete(NULL);
}

static void infinite_beep_task(void *pvParameters)
{
    beep_type_t beep_type = (beep_type_t) pvParameters;

    while (1) {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);

        if (beep_type == INFINITE_BEEPS) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else if (beep_type == FAST_INFINITE_BEEPS) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}

void beep(beep_setting_t beep_setting)
{
    if (beep_setting.blocking_type == BLOCKING) {
        switch (beep_setting.beep_type) {
            case BEEPS:
                blocking_beep(beep_setting.beeps);
                break;
            case SINGLE_BEEP:
                ESP_ERROR_CHECK(beep_setting.single_beep_duration_ms > 0 ? ESP_OK : ESP_FAIL);
                blocking_single_beep_ms(beep_setting.single_beep_duration_ms);
                break;
            case HUMAN_COUNTABLE:
                human_countable_blocking_beep(beep_setting.beeps);
                break;
            default:
                break;
        }
    } else if (beep_setting.blocking_type == NON_BLOCKING) {
        turn_beeper_off();
        chip_sleep_semaphore = beep_setting.chip_sleep_semaphore;

        if (chip_sleep_semaphore != NULL) {
            xSemaphoreTake(chip_sleep_semaphore, portMAX_DELAY);
        }

        switch (beep_setting.beep_type) {
            case BEEPS:
                xTaskCreate(&beep_task, "beep", 1024, (void *) beep_setting.beeps, 3, &beep_task_handle);
                break;
            case INFINITE_BEEPS:
                xTaskCreate(&infinite_beep_task, "infinite_beep", 1024, (void *) INFINITE_BEEPS, 3, &beep_task_handle);
                break;
            case FAST_INFINITE_BEEPS:
                xTaskCreate(&infinite_beep_task, "infinite_beep", 1024, (void *) FAST_INFINITE_BEEPS, 3, &beep_task_handle);
                break;
            case SINGLE_BEEP:
                ESP_ERROR_CHECK(beep_setting.single_beep_duration_ms > 0 ? ESP_OK : ESP_FAIL);
                xTaskCreate(&single_beep_ms_task, "single_beep_ms", 1024, (void *) beep_setting.single_beep_duration_ms, 3, &beep_task_handle);
                break;
            default:
                break;
        }
    }
}

void turn_beeper_off()
{
    if (beep_task_handle != NULL && eTaskGetState(beep_task_handle) != eDeleted) {
        vTaskDelete(beep_task_handle);
    }
    beep_task_handle = NULL;

    gpio_set_level(BUZZER_PIN, 0);
}