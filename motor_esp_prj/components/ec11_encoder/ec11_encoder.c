#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "stepper_app.h"

#define EC11_GPIO_X_A GPIO_NUM_10
#define EC11_GPIO_X_B GPIO_NUM_11
#define EC11_GLITCH_ns 1000

#define EC11_COUNT_HIGH_LIMIT 100
#define EC11_COUNT_LOW_LIMIT -100

pcnt_unit_handle_t pcnt_uint_X = NULL;
QueueHandle_t pcnt_X_watch_event_queue = NULL;
extern QueueHandle_t step_X_queue;

TaskHandle_t task_ec11_handle;
#define task_ec11_stackdepth 1024 * 2
#define task_ec11_priority 1

// on_reach
static bool ec11_pcnt_X_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int watch_dat = edata->watch_point_value;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to pcnt_X_watch_event_queue, from this interrupt callback
    xQueueSendFromISR(queue, &watch_dat, NULL);
    return pdTRUE;
}

static void task_ec11_handler(void *Param)
{
    static int circute_count_X = 0, step_count_X = 0;
    static int step_sum_X = 0, step_sum_last_X = 0;
    int event_count = 0;

    for (;;)
    {
        if (xQueueReceive(pcnt_X_watch_event_queue, &event_count, pdMS_TO_TICKS(10)))
        {
            circute_count_X += event_count;
        }
        else
        {
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_uint_X, &step_count_X));
        step_sum_X = circute_count_X + step_count_X;

        if (step_sum_X != step_sum_last_X)
        {
            int step_sub = step_sum_X - step_sum_last_X;
            xQueueSend(step_X_queue, &step_sub, 0);
            step_sum_last_X = step_sum_X;
        }
    }
}

void ec11_activate(void)
{
    // ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EC11_COUNT_HIGH_LIMIT,
        .low_limit = EC11_COUNT_LOW_LIMIT,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_uint_X));

    // ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = EC11_GLITCH_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_uint_X, &filter_config));

    // ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EC11_GPIO_X_A,
        .level_gpio_num = EC11_GPIO_X_B,
    };
    pcnt_channel_handle_t pcnt_chan_X = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_uint_X, &chan_a_config, &pcnt_chan_X));

    // ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_X, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_X, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_X, EC11_COUNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_X, EC11_COUNT_LOW_LIMIT));

    pcnt_event_callbacks_t cbs = {
        .on_reach = ec11_pcnt_X_on_reach,
    };

    pcnt_X_watch_event_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_uint_X, &cbs, pcnt_X_watch_event_queue));

    // ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_uint_X));
    // ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_uint_X));
    ESP_LOGI("EC11_encoder", "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_uint_X));

    xTaskCreate(task_ec11_handler,
                "task_ec11_handler",
                task_ec11_stackdepth,
                NULL,
                task_ec11_priority,
                &task_ec11_handle);
}