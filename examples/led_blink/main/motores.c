#include "motores.h"

void initMotorPins(){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
}

void avanzar(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0B_OUT);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0A_OUT, 50);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0A_OUT, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void retroceder(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0A_OUT);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0B_OUT, 50);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0B_OUT, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void parar(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0A_OUT);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0B_OUT);
}

void izquierda(){

}

void derecha(){

}