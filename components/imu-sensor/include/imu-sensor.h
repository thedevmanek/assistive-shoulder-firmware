#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    float x;
    float y;
    float z;
} axis;

typedef struct {
    axis accelerometer;
    axis gyroscope;
    axis magnetometer;
    float muscleSensor;
} IMUData;

void init_imu();

IMUData getIMUValues();