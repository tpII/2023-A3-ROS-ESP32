#ifndef ULTRASONIDO_H
#define ULTRASONIDO_H

#include <ultrasonic.h>
#include <esp_err.h>

#define MAX_DISTANCE_CM 100 // 1m max

#define TRIGGER 5
#define ECHO 18

ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

void initUltrasonido();

float sensarUltrasonido();


#endif