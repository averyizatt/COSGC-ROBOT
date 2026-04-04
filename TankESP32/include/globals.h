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
extern float batteryVoltage;   // Filtered battery pack voltage (V), 0 = not yet read

#endif // GLOBALS_H
