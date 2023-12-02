#include "ultrasonido.h"
ultrasonic_sensor_t miSensor = {
        .trigger_pin = TRIGGER,
        .echo_pin = ECHO
    };

void initUltrasonido(){
    ultrasonic_init(&miSensor);
}

int sensarUltrasonido(){
    float distance;
    esp_err_t res = ultrasonic_measure(&miSensor, MAX_DISTANCE_cd M, &distance);
    if (res != ESP_OK)
    {
        distance = -1;
    }
    else
        distance*=100;
    return (int)distance;
}