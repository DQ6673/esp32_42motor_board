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

static const char *TAG = "motor";

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

rmt_channel_handle_t motor_chan = NULL;
QueueHandle_t step_X_queue = NULL;
rmt_encoder_handle_t uniform_motor_encoder = NULL;
rmt_transmit_config_t tx_config = {
    .loop_count = 0,
};

TaskHandle_t task_stepper_motor_handle;
#define task_stepper_motor_stackdepth 1024 * 4
#define task_stepper_motor_priority 1

// speed_switch control
extern SemaphoreHandle_t motor_speed_semphr;
extern uint32_t motor_speed;
extern nvs_handle_t motor_nvs_handle;

static void task_stepper_motor_handler(void *Param)
{
    int step_rev = 0;
    static uint32_t freq_run = 0;
    ESP_LOGI(TAG, "start motor task");
    // freq x1
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x1", &freq_x1) == ESP_OK)
    {
        ESP_LOGI(TAG, "get freq_set_x1 = %luHz from nvs", freq_x1);
    }
    else
    {
        ESP_LOGW(TAG, "cannot get freq_set_x1 from nvs, using default value: %lu", freq_x1);
    }
    // freq x10
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x10", &freq_x10) == ESP_OK)
    {
        ESP_LOGI(TAG, "get freq_set_x10 = %luHz from nvs", freq_x10);
    }
    else
    {
        ESP_LOGW(TAG, "cannot get freq_set_x10 from nvs, using default value: %lu", freq_x10);
    }
    // freq x100
    if (nvs_get_u32(motor_nvs_handle, "freq_set_x100", &freq_x100) == ESP_OK)
    {
        ESP_LOGI(TAG, "get freq_set_x100 = %luHz from nvs", freq_x100);
    }
    else
    {
        ESP_LOGW(TAG, "cannot get freq_set_x100 from nvs, using default value: %lu", freq_x100);
    }
    // basic step
    if (nvs_get_u32(motor_nvs_handle, "step_basic_set", &step_basic) == ESP_OK)
    {
        ESP_LOGI(TAG, "get step_basic_set = %lu from nvs", step_basic);
    }
    else
    {
        ESP_LOGW(TAG, "cannot get step_basic_set from nvs, using default value: %lu", step_basic);
    }
    ////////////////////

    for (;;)
    {
        if (xQueueReceive(step_X_queue, &step_rev, portMAX_DELAY))
        {
            if (step_rev > 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_X, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            else if (step_rev < 0)
            {
                gpio_set_level(STEP_MOTOR_GPIO_DIR_X, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                step_rev *= (-1);
            }

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
            tx_config.loop_count = step_rev * step_basic * motor_speed;
            xSemaphoreGive(motor_speed_semphr);

            ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &freq_run, sizeof(freq_run), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
        }
    }
}

void stepper_motor_activate(void)
{
    //
    gpio_config_t stepper_dir_io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << STEP_MOTOR_GPIO_DIR_X) | (1ULL << STEP_MOTOR_GPIO_DIR_Y) | (1ULL << STEP_MOTOR_GPIO_DIR_Z)),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&stepper_dir_io);

    //
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP_X,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    step_X_queue = xQueueCreate(2, sizeof(int));
    xTaskCreate(task_stepper_motor_handler,
                "task_stepper_motor_handler",
                task_stepper_motor_stackdepth,
                NULL,
                task_stepper_motor_priority,
                &task_stepper_motor_handle);
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
