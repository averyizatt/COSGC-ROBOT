#ifndef GLOBALS_H
#define GLOBALS_H

#include "motor_control.h"
#include "hall_encoder.h"
#include "occupancy_map.h"
#include <stdint.h>

extern MotorControl motors;
extern HallEncoder encoders;
extern OccupancyMap envMap;
extern uint8_t currentMode;
extern float odomX, odomY, odomTheta;

#endif // GLOBALS_H
