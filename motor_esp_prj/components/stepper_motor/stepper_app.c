#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <inttypes.h>
#include "esp_system.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "user_console.h"
#include "stepper_motor_encoder.h"
#include "stepper_app.h"
#include "speed_switch.h"
#include "user_nvs.h"

static const char *TAG = "stepper motor";

/***********************************************************************************/
#define STEP_MOTOR_GPIO_DIR_X GPIO_NUM_35
#define STEP_MOTOR_GPIO_STEP_X GPIO_NUM_36
#define STEP_MOTOR_GPIO_DIR_Y GPIO_NUM_33
#define STEP_MOTOR_GPIO_STEP_Y GPIO_NUM_34
#define STEP_MOTOR_GPIO_DIR_Z GPIO_NUM_48
#define STEP_MOTOR_GPIO_STEP_Z GPIO_NUM_47

#define FREQ_DEFAULT_x1 3000
#define FREQ_DEFAULT_x10 15000
#define FREQ_DEFAULT_x100 18000
#define STEP_BASIC_DEFAULT 64

#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static uint32_t freq_x1 = FREQ_DEFAULT_x1,
                freq_x10 = FREQ_DEFAULT_x10,
                freq_x100 = FREQ_DEFAULT_x100;
static uint32_t step_basic = STEP_BASIC_DEFAULT;

// rmt channel
rmt_channel_handle_t motor_chan_X = NULL;
rmt_channel_handle_t motor_chan_Y = NULL;
rmt_channel_handle_t motor_chan_Z = NULL;

// rmt message queue
QueueHandle_t step_X_queue = NULL;
QueueHandle_t step_Y_queue = NULL;
QueueHandle_t step_Z_queue = NULL;

// stepper motor encoder
rmt_encoder_handle_t uniform_motor_encoder = NULL;

rmt_transmit_config_t tx_config = {
    .loop_count = 0,
};

TaskHandle_t task_stepper_motor_X_handle;
#define task_stepper_motor_X_stackdepth 1024 * 3
#define task_stepper_motor_X_priority 1

TaskHandle_t task_stepper_motor_Y_handle;
#define task_stepper_motor_Y_stackdepth 1024 * 3
#define task_stepper_motor_Y_priority 1

TaskHandle_t task_stepper_motor_Z_handle;
#define task_stepper_motor_Z_stackdepth 1024 * 3
#define task_stepper_motor_Z_priority 1

// speed_switch control
extern SemaphoreHandle_t motor_speed_semphr;
extern uint32_t motor_speed;
extern nvs_handle_t motor_nvs_handle;

static uint32_t get_current_motor_speed(void)
{
    uint32_t freq_run = 0;

    xSemaphoreTake(motor_speed_semphr, portMAX_DELAY);
    switch (motor_speed)
    {
    case 1:
        freq_run = freq_x1;
        break;
    case 10:
        freq_run = freq_x10;
        break;
    case 100:
        freq_run = freq_x100;
        break;
    default:
        break;
    }
    xSemaphoreGive(motor_speed_semphr);

    return freq_run;
}

static void task_stepper_motor_X_handler(void *Param)
{
    static int step_rev_X = 0;
    static uint32_t freq_run = 0;

    for (;;)
    {
        // get speed changed by switch(3x)
        if (xQueueReceive(step_X_queue, &step_rev_X, portMAX_DELAY))
        {
            if (step_rev_X > 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_X, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            else if (step_rev_X < 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_X, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                step_rev_X *= (-1);
            }

            freq_run = get_current_motor_speed();
            tx_config.loop_count = step_rev_X * step_basic * motor_speed;

            ESP_ERROR_CHECK(rmt_transmit(motor_chan_X, uniform_motor_encoder, &freq_run, sizeof(freq_run), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_X, -1));
        }
    }
}

static void task_stepper_motor_Y_handler(void *Param)
{
    static int step_rev_Y = 0;
    static uint32_t freq_run = 0;

    for (;;)
    {
        // get speed changed by switch(3x)
        if (xQueueReceive(step_Y_queue, &step_rev_Y, portMAX_DELAY))
        {
            if (step_rev_Y > 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_Y, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            else if (step_rev_Y < 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_Y, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                step_rev_Y *= (-1);
            }

            freq_run = get_current_motor_speed();
            tx_config.loop_count = step_rev_Y * step_basic * motor_speed;

            ESP_ERROR_CHECK(rmt_transmit(motor_chan_Y, uniform_motor_encoder, &freq_run, sizeof(freq_run), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_Y, -1));
        }
    }
}

static void task_stepper_motor_Z_handler(void *Param)
{
    static int step_rev_Z = 0;
    static uint32_t freq_run = 0;

    for (;;)
    {
        // get speed changed by switch(3x)
        if (xQueueReceive(step_Z_queue, &step_rev_Z, portMAX_DELAY))
        {
            if (step_rev_Z > 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_Z, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            else if (step_rev_Z < 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_Z, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                step_rev_Z *= (-1);
            }

            freq_run = get_current_motor_speed();
            tx_config.loop_count = step_rev_Z * step_basic * motor_speed;

            ESP_ERROR_CHECK(rmt_transmit(motor_chan_Z, uniform_motor_encoder, &freq_run, sizeof(freq_run), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan_Z, -1));
        }
    }
}

void stepper_motor_activate(void)
{
    // DIR gpio
    gpio_config_t stepper_dir_io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << STEP_MOTOR_GPIO_DIR_X) | (1ULL << STEP_MOTOR_GPIO_DIR_Y) | (1ULL << STEP_MOTOR_GPIO_DIR_Z)),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&stepper_dir_io);

    //
    rmt_tx_channel_config_t tx_chan_X_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_X,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    rmt_tx_channel_config_t tx_chan_Y_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_Y,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    rmt_tx_channel_config_t tx_chan_Z_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_Z,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_X_config, &motor_chan_X));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_Y_config, &motor_chan_Y));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_Z_config, &motor_chan_Z));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_ERROR_CHECK(rmt_enable(motor_chan_X));

    step_X_queue = xQueueCreate(2, sizeof(int));
    xTaskCreate(task_stepper_motor_X_handler,
                "task_stepper_motor_X_handler",
                task_stepper_motor_X_stackdepth,
                NULL,
                task_stepper_motor_X_priority,
                &task_stepper_motor_X_handle);

    xTaskCreate(task_stepper_motor_Y_handler,
                "task_stepper_motor_Y_handler",
                task_stepper_motor_Y_stackdepth,
                NULL,
                task_stepper_motor_Y_priority,
                &task_stepper_motor_Y_handle);

    xTaskCreate(task_stepper_motor_Z_handler,
                "task_stepper_motor_Z_handler",
                task_stepper_motor_Z_stackdepth,
                NULL,
                task_stepper_motor_Z_priority,
                &task_stepper_motor_Z_handle);

    // freq x1
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x1", &freq_x1) == ESP_OK)
        ESP_LOGI(TAG, "get freq_set_x1 = %luHz from nvs", freq_x1);
    else
        ESP_LOGW(TAG, "cannot get freq_set_x1 from nvs, using default value: %lu", freq_x1);

    // freq x10
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x10", &freq_x10) == ESP_OK)
        ESP_LOGI(TAG, "get freq_set_x10 = %luHz from nvs", freq_x10);
    else
        ESP_LOGW(TAG, "cannot get freq_set_x10 from nvs, using default value: %lu", freq_x10);

    // freq x100
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x100", &freq_x100) == ESP_OK)
        ESP_LOGI(TAG, "get freq_set_x100 = %luHz from nvs", freq_x100);
    else
        ESP_LOGW(TAG, "cannot get freq_set_x100 from nvs, using default value: %lu", freq_x100);

    // basic step
    if (nvs_get_u32(motor_nvs_handle, "step_basic_set", &step_basic) == ESP_OK)
        ESP_LOGI(TAG, "get step_basic_set = %lu from nvs", step_basic);
    else
        ESP_LOGW(TAG, "cannot get step_basic_set from nvs, using default value: %lu", step_basic);
}

/*************************************************/
// command tools:

static struct
{
    struct arg_int *freq_set_x1;
    struct arg_int *freq_set_x10;
    struct arg_int *freq_set_x100;
    struct arg_int *step_basic_set;
    struct arg_end *end;
} motor_set_args;

static int do_motor_set_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&motor_set_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, motor_set_args.end, argv[0]);
        return 0;
    }

    // args
    if (motor_set_args.freq_set_x1->count)
    {
        freq_x1 = motor_set_args.freq_set_x1->ival[0];
        ESP_LOGI(TAG, "freq(x1) set successfully");
        if (nvs_set_u32(motor_nvs_handle, "freq_set_x1", freq_x1) == ESP_OK)
        {
            ESP_LOGI(TAG, "freq(x1) saved");
        }
        else
        {
            ESP_LOGW(TAG, "cannot save freq(x1)");
        }
    }

    if (motor_set_args.freq_set_x10->count)
    {
        freq_x10 = motor_set_args.freq_set_x10->ival[0];
        ESP_LOGI(TAG, "freq(x10) set successfully");
        if (nvs_set_u32(motor_nvs_handle, "freq_set_x10", freq_x10) == ESP_OK)
        {
            ESP_LOGI(TAG, "freq(x10) saved");
        }
        else
        {
            ESP_LOGW(TAG, "cannot save freq(x10)");
        }
    }

    if (motor_set_args.freq_set_x100->count)
    {
        freq_x100 = motor_set_args.freq_set_x100->ival[0];
        ESP_LOGI(TAG, "freq(x100) set successfully");
        if (nvs_set_u32(motor_nvs_handle, "freq_set_x100", freq_x100) == ESP_OK)
        {
            ESP_LOGI(TAG, "freq(x100) saved");
        }
        else
        {
            ESP_LOGW(TAG, "cannot save freq(x100)");
        }
    }

    if (motor_set_args.step_basic_set->count)
    {
        step_basic = motor_set_args.step_basic_set->ival[0];
        ESP_LOGI(TAG, "step basic set successfully");
        if (nvs_set_u32(motor_nvs_handle, "step_basic_set", step_basic) == ESP_OK)
        {
            ESP_LOGI(TAG, "step basic saved");
        }
        else
        {
            ESP_LOGW(TAG, "cannot save step basic");
        }
    }

    return 0;
}

static void register_motor_set(void)
{
    motor_set_args.freq_set_x1 = arg_int0(NULL, "fx1", "<Hz>", "Set the frequency of speed x1 (500 ~ 60000 Hz)");
    motor_set_args.freq_set_x10 = arg_int0(NULL, "fx10", "<Hz>", "Set the frequency of speed x10 (500 ~ 60000 Hz)");
    motor_set_args.freq_set_x100 = arg_int0(NULL, "fx100", "<Hz>", "Set the frequency of speed x100 (500 ~ 60000 Hz)");
    motor_set_args.step_basic_set = arg_int0(NULL, "step", "<number>", "Set the basic step number of speed x1");
    motor_set_args.end = arg_end(2);
    const esp_console_cmd_t motor_set_cmd = {
        .command = "set",
        .help = "Config 42_motor",
        .hint = NULL,
        .func = &do_motor_set_cmd,
        .argtable = &motor_set_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&motor_set_cmd));
}

void register_motortools(void)
{
    register_motor_set();
}
