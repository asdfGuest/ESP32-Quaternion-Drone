#include "pwm_esc.hpp"


void mylib::PWMESC::set_timer(ledc_timer_t timer, protocol_t protocol) {
    ledc_timer_config_t timer_cfg = timer_cfg_table[(int)protocol];
    timer_cfg.timer_num = timer;

    ledc_timer_config(&timer_cfg);
    return;
}

mylib::PWMESC::PWMESC(ledc_timer_t timer, ledc_channel_t channel, protocol_t protocol, int gpio_num) {
    this->timer = timer;
    this->channel = channel;
    this->speed_mode = timer_cfg_table[(int)protocol].speed_mode;
    this->protocol = protocol;
    this->gpio_num = gpio_num;
}

void mylib::PWMESC::pwm_begin(void) {
    ledc_channel_config_t channel_cfg = {
        .gpio_num = gpio_num,
        .speed_mode = speed_mode,
        .channel = channel,
        .timer_sel = timer,
        .duty = DUTY_MIN + CONTROL_DUTY_OFFSET,
        .hpoint = 0,
    };
    ledc_channel_config(&channel_cfg);
    return;
}

void mylib::PWMESC::update(uint32_t duty) {
    duty = std::clamp(duty, DUTY_MIN, DUTY_MAX) + CONTROL_DUTY_OFFSET;
    
    ledc_set_duty(speed_mode, channel, duty);
    ledc_update_duty(speed_mode, channel);
    return;
}
