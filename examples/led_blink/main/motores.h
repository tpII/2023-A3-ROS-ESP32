#ifndef MOTORES_H
#define MOTORES_H

#include "driver/mcpwm.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

#ifndef AIA
    #define AIA 13
#endif
#ifndef AIB
    #define AIB 12
#endif
#ifndef BIA
    #define BIA 10
#endif
#ifndef BIB
    #define BIB 9
#endif

#define GPIO_PWM0A_OUT AIA   //PWM0A = AIA
#define GPIO_PWM0B_OUT AIB   //PWM0B = AIB
#define GPIO_PWM1A_OUT BIA   //PWM1A = BIA
#define GPIO_PWM1B_OUT BIB   //PWM1B = BIB

void initMotorPins();

void avanzar();

void retroceder();

void parar();

void izquierda();

void derecha();



#endif