#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "cJSON.h"
#include "imu-sensor.h"


IMUData generateIMUData() {
    IMUData data=getIMUValues();
    return data;
}

// Function to create cJSON object from IMUData
cJSON *imuDataToJson(const IMUData *data) {
    cJSON *jsonObj = cJSON_CreateObject();

    cJSON *accelerometer = cJSON_CreateObject();
    cJSON_AddItemToObject(accelerometer, "x", cJSON_CreateNumber(data->accelerometer.x));
    cJSON_AddItemToObject(accelerometer, "y", cJSON_CreateNumber(data->accelerometer.y));
    cJSON_AddItemToObject(accelerometer, "z", cJSON_CreateNumber(data->accelerometer.z));
    cJSON_AddItemToObject(jsonObj, "accelerometer", accelerometer);

    cJSON *gyroscope = cJSON_CreateObject();
    cJSON_AddItemToObject(gyroscope, "x", cJSON_CreateNumber(data->gyroscope.x));
    cJSON_AddItemToObject(gyroscope, "y", cJSON_CreateNumber(data->gyroscope.y));
    cJSON_AddItemToObject(gyroscope, "z", cJSON_CreateNumber(data->gyroscope.z));
    cJSON_AddItemToObject(jsonObj, "gyroscope", gyroscope);

    cJSON *magnetometer = cJSON_CreateObject();
    cJSON_AddItemToObject(magnetometer, "x", cJSON_CreateNumber(data->magnetometer.x));
    cJSON_AddItemToObject(magnetometer, "y", cJSON_CreateNumber(data->magnetometer.y));
    cJSON_AddItemToObject(magnetometer, "z", cJSON_CreateNumber(data->magnetometer.z));
    cJSON_AddItemToObject(jsonObj, "magnetometer", magnetometer);

    cJSON_AddItemToObject(jsonObj, "muscleSensor", cJSON_CreateNumber(data->muscleSensor));

    return jsonObj;
}

// Function to generate JSON string from IMUData
char *generateJsonString() {

    IMUData randomData = generateIMUData();

    cJSON *jsonObj = imuDataToJson(&randomData);

    char *jsonString = cJSON_Print(jsonObj);

    cJSON_Delete(jsonObj);

    return jsonString;
}