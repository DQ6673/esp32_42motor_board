#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "ec11_encoder.h"
#include "freq_test.h"
#include "stepper_app.h"
#include "speed_switch.h"
#include "user_console.h"
#include "user_nvs.h"

void app_main(void)
{
    user_nvs_init();
    ec11_activate();
    // freq_test_activate();
    speed_switch_activate();
    stepper_motor_activate();

    user_console_activate();
}