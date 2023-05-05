#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define GPIO_SETP_X GPIO_NUM_36 // Define the output GPIO
#define GPIO_DIR_X GPIO_NUM_35  // Define the output GPIO
#define FREQ_TEST_BASIC (18000) // Frequency in Hertz. Set frequency at 5 kHz

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (128)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

TaskHandle_t task_freq_test_handle;
#define task_freq_test_stackdepth 1024 * 2
#define task_freq_test_priority 1

QueueHandle_t freq_test_X_queue = NULL;

static void task_freq_test_handler(void *Param)
{
    int freq_set = FREQ_TEST_BASIC;
    int freq_rev = 0;

    for (;;)
    {
        if (xQueueReceive(freq_test_X_queue, &freq_rev, portMAX_DELAY))
        {
            freq_set += (freq_rev * 200);
            if (freq_set > 50000)
                freq_set = 50000;

            else if (freq_set < 5000)
                freq_set = 5000;

            ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_set));

            ESP_LOGI("freq_test", "set freq:%d Hz, pulse frequency: %lu Hz", freq_set, ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0));
        }
    }
}

void freq_test_activate(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_DIR_X),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_DIR_X, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = FREQ_TEST_BASIC, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_SETP_X,
        .duty = LEDC_DUTY, // Set duty to 50%
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    freq_test_X_queue = xQueueCreate(10, sizeof(int));

    xTaskCreate(task_freq_test_handler,
                "task_freq_test_handler",
                task_freq_test_stackdepth,
                NULL,
                task_freq_test_priority,
                &task_freq_test_handle);
}