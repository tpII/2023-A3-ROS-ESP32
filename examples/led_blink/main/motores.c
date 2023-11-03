#include "motores.h"

ledc_timer_config_t myTimer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t myChannel[2];

void initMotorPins(){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings

    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    
}

void SetIzqWc(float Wc){
    float cicloDeTrabajo = Wc * 100;
    if (Wc > 0){
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, cicloDeTrabajo);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -cicloDeTrabajo);
         mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }


    /*int cicloDeTrabajo = Wc * 1023;
    if (Wc > 0){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, cicloDeTrabajo));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, -cicloDeTrabajo));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }*/
}

void SetDerWc(float Wc){
    float cicloDeTrabajo = Wc * 100;
    if (Wc > 0){
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, cicloDeTrabajo);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, -cicloDeTrabajo);
         mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
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