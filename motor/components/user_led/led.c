#include "led.h"
#include "driver/ledc.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO GPIO_NUM_4 // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_12_BIT // Set duty resolution to 12 bits
#define LEDC_MAX_DUTY_NUM (4096 - 1)
#define LEDC_FREQUENCY (1000) // Frequency in Hertz. Set frequency at 10 kHz

/// @brief LED初始化
/// @param
void LED_init(void)
{
    // LED GPIO初始化
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;       // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;             // set as output mode
    io_conf.pin_bit_mask = (1U << LEDC_OUTPUT_IO); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = 0;                    // disable pull-down mode
    io_conf.pull_up_en = 0;                      // disable pull-up mode
    gpio_config(&io_conf);                       // configure GPIO with the given settings

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_fade_func_install(0);
    LED_off();
}

/// @brief LED亮起
/// @param duty_set 设置占空比（0.00 ~ 100.00）
/// @param fade_time 设置渐变时间（ms）
void LED_PWM_on(float duty_set, uint32_t fade_time)
{
    uint32_t dutynum = (uint32_t)(duty_set * LEDC_MAX_DUTY_NUM / 100);

    ledc_set_fade_time_and_start(LEDC_MODE, LEDC_CHANNEL, dutynum, fade_time, LEDC_FADE_NO_WAIT);
}

/// @brief LED关闭（占空比设置为0）
/// @param
void LED_off(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}