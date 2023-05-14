#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "stepper_app.h"

static const char *TAG = "ec11 encoder";

#define EC11_GPIO_X_A GPIO_NUM_10
#define EC11_GPIO_X_B GPIO_NUM_11
#define EC11_GPIO_Y_A GPIO_NUM_12
#define EC11_GPIO_Y_B GPIO_NUM_13
#define EC11_GPIO_Z_A GPIO_NUM_14
#define EC11_GPIO_Z_B GPIO_NUM_17

#define EC11_GLITCH_ns 1000

#define EC11_COUNT_HIGH_LIMIT 100
#define EC11_COUNT_LOW_LIMIT -100

pcnt_unit_handle_t pcnt_uint_X = NULL;
pcnt_unit_handle_t pcnt_uint_Y = NULL;
pcnt_unit_handle_t pcnt_uint_Z = NULL;
QueueHandle_t pcnt_X_watch_event_queue = NULL;
QueueHandle_t pcnt_Y_watch_event_queue = NULL;
QueueHandle_t pcnt_Z_watch_event_queue = NULL;
extern QueueHandle_t step_X_queue;
extern QueueHandle_t step_Y_queue;
extern QueueHandle_t step_Z_queue;

TaskHandle_t task_ec11_handle;
#define task_ec11_stackdepth 1024 * 2
#define task_ec11_priority 1

// watch point X
static bool ec11_pcnt_X_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int watch_dat = edata->watch_point_value;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to pcnt_X_watch_event_queue, from this interrupt callback
    xQueueSendFromISR(queue, &watch_dat, NULL);
    return pdTRUE;
}

// watch point Y
static bool ec11_pcnt_Y_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int watch_dat = edata->watch_point_value;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to pcnt_X_watch_event_queue, from this interrupt callback
    xQueueSendFromISR(queue, &watch_dat, NULL);
    return pdTRUE;
}

// watch point Z
static bool ec11_pcnt_Z_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
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
    static int circute_count_Y = 0, step_count_Y = 0;
    static int circute_count_Z = 0, step_count_Z = 0;

    static int step_sum_X = 0, step_sum_last_X = 0;
    static int step_sum_Y = 0, step_sum_last_Y = 0;
    static int step_sum_Z = 0, step_sum_last_Z = 0;

    static int event_count_X = 0;
    static int event_count_Y = 0;
    static int event_count_Z = 0;

    for (;;)
    {
        // X channel
        if (xQueueReceive(pcnt_X_watch_event_queue, &event_count_X, 0))
        {
            circute_count_X += event_count_X;
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_uint_X, &step_count_X));
        step_sum_X = circute_count_X + step_count_X;
        if (step_sum_X != step_sum_last_X)
        {
            int step_sub_X = step_sum_X - step_sum_last_X;
            xQueueSend(step_X_queue, &step_sub_X, 0);
            step_sum_last_X = step_sum_X;
        }

        // Y channel
        if (xQueueReceive(pcnt_Y_watch_event_queue, &event_count_Y, 0))
        {
            circute_count_Y += event_count_Y;
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_uint_Y, &step_count_Y));
        step_sum_Y = circute_count_Y + step_count_Y;
        if (step_sum_Y != step_sum_last_Y)
        {
            int step_sub_Y = step_sum_Y - step_sum_last_Y;
            xQueueSend(step_Y_queue, &step_sub_Y, 0);
            step_sum_last_Y = step_sum_Y;
        }

        // Z channel
        if (xQueueReceive(pcnt_Z_watch_event_queue, &event_count_Z, 0))
        {
            circute_count_Z += event_count_Z;
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_uint_Z, &step_count_Z));
        step_sum_Z = circute_count_Z + step_count_Z;
        if (step_sum_Z != step_sum_last_Z)
        {
            int step_sub_Z = step_sum_Z - step_sum_last_Z;
            xQueueSend(step_Z_queue, &step_sub_Z, 0);
            step_sum_last_Z = step_sum_Z;
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
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_uint_Y));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_uint_Z));

    // ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = EC11_GLITCH_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_uint_X, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_uint_Y, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_uint_Z, &filter_config));

    // ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_X_config = {
        .edge_gpio_num = EC11_GPIO_X_A,
        .level_gpio_num = EC11_GPIO_X_B,
    };
    pcnt_chan_config_t chan_Y_config = {
        .edge_gpio_num = EC11_GPIO_Y_A,
        .level_gpio_num = EC11_GPIO_Y_B,
    };
    pcnt_chan_config_t chan_Z_config = {
        .edge_gpio_num = EC11_GPIO_Z_A,
        .level_gpio_num = EC11_GPIO_Z_B,
    };
    pcnt_channel_handle_t pcnt_chan_X = NULL;
    pcnt_channel_handle_t pcnt_chan_Y = NULL;
    pcnt_channel_handle_t pcnt_chan_Z = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_uint_X, &chan_X_config, &pcnt_chan_X));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_uint_Y, &chan_Y_config, &pcnt_chan_Y));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_uint_Z, &chan_Z_config, &pcnt_chan_Z));

    // ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_X, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_X, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_Y, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_Y, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_Z, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_Z, PCNT_CHANNEL_LEVEL_ACTION_HOLD, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_X, EC11_COUNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_X, EC11_COUNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_Y, EC11_COUNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_Y, EC11_COUNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_Z, EC11_COUNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_uint_Z, EC11_COUNT_LOW_LIMIT));

    pcnt_event_callbacks_t cbs_X = {
        .on_reach = ec11_pcnt_X_on_reach,
    };
    pcnt_event_callbacks_t cbs_Y = {
        .on_reach = ec11_pcnt_Y_on_reach,
    };
    pcnt_event_callbacks_t cbs_Z = {
        .on_reach = ec11_pcnt_Z_on_reach,
    };

    pcnt_X_watch_event_queue = xQueueCreate(10, sizeof(int));
    pcnt_Y_watch_event_queue = xQueueCreate(10, sizeof(int));
    pcnt_Z_watch_event_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_uint_X, &cbs_X, pcnt_X_watch_event_queue));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_uint_Y, &cbs_Y, pcnt_Y_watch_event_queue));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_uint_Z, &cbs_Z, pcnt_Z_watch_event_queue));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_uint_X));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_uint_Y));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_uint_Z));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_uint_X));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_uint_Y));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_uint_Z));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_uint_X));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_uint_Y));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_uint_Z));

    ESP_LOGI(TAG, "enable XYZ pcnt unit");

    xTaskCreate(task_ec11_handler,
                "task_ec11_handler",
                task_ec11_stackdepth,
                NULL,
                task_ec11_priority,
                &task_ec11_handle);
}