#ifndef ULTRASONIDO_H
#define ULTRASONIDO_H

#include "ultrasonic.h"
#include <esp_err.h>

#define MAX_DISTANCE_M 0.5 // 1m max

#define TRIGGER 18
#define ECHO 5



void initUltrasonido();

int sensarUltrasonido();


#endif