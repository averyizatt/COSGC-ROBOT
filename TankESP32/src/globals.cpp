#include "globals.h"
#include "motor_control.h"
#include "hall_encoder.h"
#include "occupancy_map.h"

MotorControl motors;
HallEncoder encoders;
OccupancyMap envMap;
uint8_t currentMode = 0;
float odomX = 0, odomY = 0, odomTheta = 0;
float batteryVoltage = 0.0f;
