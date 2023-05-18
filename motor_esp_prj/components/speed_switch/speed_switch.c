#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "speed_switch.h"

#define GPIO_SPEED_1 GPIO_NUM_9
#define GPIO_SPEED_2 GPIO_NUM_21
#define GPIO_LED_SPEED_1 GPIO_NUM_8
#define GPIO_LED_SPEED_10 GPIO_NUM_7
#define GPIO_LED_SPEED_100 GPIO_NUM_6

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_LED_LEVEL_ON 0
#define GPIO_LED_LEVEL_OFF 1

// static const char *TAG = "speed_switch";

SemaphoreHandle_t motor_speed_semphr = NULL;
uint32_t motor_speed = 1;

TaskHandle_t task_speed_switch_handle;
#define task_speed_switch_stackdepth 1024 * 2
#define task_speed_switch_priority 1

static void task_speed_switch(void *arg)
{
    static int gpio_speed_level = 2;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(20 / portTICK_PERIOD_MS);       // 20ms soft filter

        gpio_speed_level = gpio_get_level(GPIO_SPEED_1);
        gpio_speed_level = (gpio_speed_level << 1) + gpio_get_level(GPIO_SPEED_2);
        //ESP_LOGI(TAG, "switch value : %d", gpio_speed_level);

        gpio_set_level(GPIO_LED_SPEED_1, GPIO_LED_LEVEL_OFF);
        gpio_set_level(GPIO_LED_SPEED_10, GPIO_LED_LEVEL_OFF);
        gpio_set_level(GPIO_LED_SPEED_100, GPIO_LED_LEVEL_OFF);

        xSemaphoreTake(motor_speed_semphr, portMAX_DELAY);
        switch (gpio_speed_level)
        {
        case 1:
            gpio_set_level(GPIO_LED_SPEED_1, GPIO_LED_LEVEL_ON);
            motor_speed = 1;
            break;
        case 3:
            gpio_set_level(GPIO_LED_SPEED_10, GPIO_LED_LEVEL_ON);
            motor_speed = 10;
            break;
        case 2:
            gpio_set_level(GPIO_LED_SPEED_100, GPIO_LED_LEVEL_ON);
            motor_speed = 100;
            break;
        default:
            break;
        }
        xSemaphoreGive(motor_speed_semphr);

        xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    vTaskNotifyGiveFromISR(task_speed_switch_handle, NULL);
}

void speed_switch_activate(void)
{
    // zero-initialize the config structure.
    gpio_config_t sw_io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << GPIO_SPEED_1) | (1ULL << GPIO_SPEED_2)),
        .pull_up_en = 0,
        .pull_down_en = 0,
    };
    gpio_config(&sw_io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_SPEED_1, gpio_isr_handler, (void *)GPIO_SPEED_1);
    gpio_isr_handler_add(GPIO_SPEED_2, gpio_isr_handler, (void *)GPIO_SPEED_2);

    gpio_config_t led_io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << GPIO_LED_SPEED_1) | (1ULL << GPIO_LED_SPEED_10) | (1ULL << GPIO_LED_SPEED_100)),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&led_io_conf);
    gpio_set_level(GPIO_LED_SPEED_1, GPIO_LED_LEVEL_OFF);
    gpio_set_level(GPIO_LED_SPEED_10, GPIO_LED_LEVEL_OFF);
    gpio_set_level(GPIO_LED_SPEED_100, GPIO_LED_LEVEL_OFF);

    xTaskCreate(task_speed_switch,
                "task_speed_switch",
                task_speed_switch_stackdepth,
                NULL,
                task_speed_switch_priority,
                &task_speed_switch_handle);

    motor_speed_semphr = xSemaphoreCreateBinary();
    xSemaphoreGive(motor_speed_semphr);
    xTaskNotifyGive(task_speed_switch_handle);
}