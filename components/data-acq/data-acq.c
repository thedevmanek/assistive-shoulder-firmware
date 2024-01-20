#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "cJSON.h"

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
float generateRandomFloat(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

IMUData generateRandomIMUData() {
    IMUData data;

    // Generate random accelerometer data
    data.accelerometer.x = generateRandomFloat(-10.0, 10.0);
    data.accelerometer.y = generateRandomFloat(-10.0, 10.0);
    data.accelerometer.z = generateRandomFloat(-10.0, 10.0);

    // Generate random gyroscope data
    data.gyroscope.x = generateRandomFloat(-5.0, 5.0);
    data.gyroscope.y = generateRandomFloat(-5.0, 5.0);
    data.gyroscope.z = generateRandomFloat(-5.0, 5.0);

    // Generate random magnetometer data
    data.magnetometer.x = generateRandomFloat(-2.0, 2.0);
    data.magnetometer.y = generateRandomFloat(-2.0, 2.0);
    data.magnetometer.z = generateRandomFloat(-2.0, 2.0);

    // Generate random muscle sensor data
    data.muscleSensor = generateRandomFloat(0.0, 100.0);
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
    // Seed the random number generator with the current time
    srand(time(NULL));

    // Generate random IMUData
    IMUData randomData = generateRandomIMUData();

    // Convert IMUData to cJSON
    cJSON *jsonObj = imuDataToJson(&randomData);

    // Dump cJSON to string
    char *jsonString = cJSON_Print(jsonObj);

    // Free allocated memory
    cJSON_Delete(jsonObj);

    return jsonString;
}