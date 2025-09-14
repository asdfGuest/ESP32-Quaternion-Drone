#ifndef MYLIB_PWM_ESC_HPP
#define MYLIB_PWM_ESC_HPP

#include "driver/ledc.h"
#include <algorithm>


namespace mylib {
    class PWMESC {
    public:
        enum class protocol_t {
            ONESHOT_125 = 0,
            ONESHOT_42 = 1,
        };
        
        static constexpr uint32_t DUTY_MIN = 0;
        static constexpr uint32_t DUTY_MAX = 1 << 10;

    private:
        static constexpr uint32_t CONTROL_DUTY_OFFSET = 1 << 10;

        static constexpr ledc_timer_config_t timer_cfg_table[] = {
            {
                /*
                OneShot 125
                - high: 125us - 250us
                - low: 250us
                - period: 500us
                - frequency: 2000Hz
                - total resolution: 12 bits
                - control resolution: 10 bits
                */
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_12_BIT,
                .timer_num = LEDC_TIMER_MAX,
                .freq_hz = 2000,
                .clk_cfg = LEDC_AUTO_CLK,
            },
            {
                /*
                OneShot 42
                - high: 42us - 84us
                - low: 84us
                - period: 168us
                - frequency: 5952Hz
                - total resolution: 12 bits
                - control resolution: 10 bits
                */
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_12_BIT,
                .timer_num = LEDC_TIMER_MAX,
                .freq_hz = 5952,
                .clk_cfg = LEDC_AUTO_CLK,
            }
        };

        ledc_timer_t timer;
        ledc_channel_t channel;
        ledc_mode_t speed_mode;
        protocol_t protocol;
        int gpio_num;

    public:
        static void set_timer(ledc_timer_t timer, protocol_t protocol);

        PWMESC(ledc_timer_t timer, ledc_channel_t channel, protocol_t protocol, int gpio_num);
        
        void pwm_begin(void);
        void update(uint32_t duty);
    };
}

#endif
