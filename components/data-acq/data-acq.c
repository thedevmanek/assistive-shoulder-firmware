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
    cJSON *rpy = cJSON_CreateObject();
    cJSON_AddItemToObject(rpy, "roll", cJSON_CreateNumber(data->rpy.x));
    cJSON_AddItemToObject(rpy, "pitch", cJSON_CreateNumber(data->rpy.y));
    cJSON_AddItemToObject(rpy, "yaw", cJSON_CreateNumber(data->rpy.z));
    cJSON_AddItemToObject(jsonObj, "rpy", rpy);
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