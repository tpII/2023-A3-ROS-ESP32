#include "ultrasonido.h"

void initUltrasonido(){
    ultrasonic_init(&sensor);
}

float sensarUltrasonido(){
    float distance;
    esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
    if (res != ESP_OK)
    {
        distance = -1;
    }
    else
        distance*=100;
    return distance;
}