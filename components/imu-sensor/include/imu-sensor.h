#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} axis;

typedef struct {
    axis accelerometer;
    axis gyroscope;
    axis magnetometer;
    axis rpy;
    float muscleSensor;
} IMUData;

void init_imu();
#ifdef __cplusplus
extern "C" {
#endif

IMUData getIMUValues();

#ifdef __cplusplus
}
#endif
