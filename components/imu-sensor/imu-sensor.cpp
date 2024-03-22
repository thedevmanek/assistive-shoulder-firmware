#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include "Fusion.h"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "imu-sensor.h"

#include "I2Cbus.hpp"

static I2C_t &i2c = i2c0;  // i2c0 or i2c1
static constexpr gpio_num_t
SDA = GPIO_NUM_6;
static constexpr gpio_num_t
SCL = GPIO_NUM_7;
static constexpr uint32_t
CLOCK_SPEED = 10000;  // 400 KHz
static constexpr gpio_num_t
PUMP = GPIO_NUM_4;
static constexpr gpio_num_t
PIN_KEY = GPIO_NUM_5;
IMUData values;
SemaphoreHandle_t imuMutex;
/* MPU configuration */

static constexpr uint16_t
kSampleRate = 250;  // Hz
static constexpr mpud::accel_fs_t
kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t
kGyroFS = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t
kDLPF = mpud::DLPF_10HZ;

/*-*/

static const char *TAG = "imu";

static void mpuTask(void *);

            float setPoint = 20;
float Kp= 15;
float Kd=5;
float Ki=0.01;
uint8_t check = 0;
uint16_t count = 0;
double input, output, error, lastError, integral, derivative;
int PumpValue;

static MPU_t MPU;
float roll{0}, pitch{0}, yaw{0};
float temp_roll=0;
// Main
void init_imu() {
    imuMutex = xSemaphoreCreateMutex();
    fflush(stdout);
    // Initialize bus through either the Library API or esp-idf API
    i2c.begin(SDA, SCL, CLOCK_SPEED);

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
}

void arm_check(void)
{
    if( roll > -80)
    {
        if(check == 0)
            check=1;
    }
    else if(roll <-85)
    {
        if(check == 1)
        {
            check=0;
            count++;
        }
    }
}

void adjustPump(void) {

    error = -temp_roll - setPoint;

    integral += error;

    derivative = error - lastError;

    output = Kp * error + Ki * integral + Kd * derivative;
    printf("Output:-%f\n",output);

    if (output > 255) output = 1    ;
    else if (output < 0) output = 0;


    esp_rom_gpio_pad_select_gpio(PUMP);
    gpio_set_direction(PUMP, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP, output);

    lastError = error;

}
/* Tasks */



extern "C" IMUData getIMUValues() {
    xSemaphoreTake(imuMutex, portMAX_DELAY);
    IMUData copy = values;
    xSemaphoreGive(imuMutex);
    return copy;
}

static void mpuTask(void *) {
// Let MPU know which bus and address to use
    MPU.setBus(i2c);
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    MPU.setBus(i2c0);  // set communication bus, for SPI -> pass 'hspi'
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address or handle, for SPI -> pass 'mpu_spi_handle'
    MPU.testConnection();  // test connection with the chip, return is a error code
    MPU.initialize();  // this will initialize the chip and set default configurations
    MPU.setSampleRate(250);  // in (Hz)
    MPU.setAccelFullScale(mpud::ACCEL_FS_4G);
    MPU.setGyroFullScale(mpud::GYRO_FS_500DPS);
    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
    MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin

    // Reading Loop
    while (true) {
        xSemaphoreTake(imuMutex, portMAX_DELAY);
        mpud::raw_axes_t rawAccel;     // holds x, y, z axes as int16
        mpud::raw_axes_t rawGyro;      // holds x, y, z axes as int16
        MPU.acceleration(&rawAccel);  // fetch raw data from the registers
        MPU.rotation(&rawGyro);       // fetch raw data from the registers
        constexpr double kRadToDeg = 57.2957795131;
        constexpr float kDeltaTime = 1.f / kSampleRate;
        float gyroRoll = roll + mpud::math::gyroDegPerSec(rawGyro.x, mpud::GYRO_FS_500DPS) * kDeltaTime;
        float gyroPitch = pitch + mpud::math::gyroDegPerSec(rawGyro.y, mpud::GYRO_FS_500DPS) * kDeltaTime;
        float gyroYaw = yaw + mpud::math::gyroDegPerSec(rawGyro.z, mpud::GYRO_FS_500DPS) * kDeltaTime;
        float accelRoll = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
        float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
        // Fusion
        roll = gyroRoll * 0.95f + accelRoll * 0.05f;
        pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
        yaw = gyroYaw;

        // correct yaw
        if (yaw > 180.f)
            yaw -= 360.f;
        else if (yaw < -180.f)
            yaw += 360.f;
        values.accelerometer.x = rawAccel.x;
        values.accelerometer.y = rawAccel.y;
        values.accelerometer.z = rawAccel.z;
        values.gyroscope.x = rawGyro.x;
        values.gyroscope.y = rawGyro.y;
        values.gyroscope.z = rawGyro.z;
        values.rpy.x = pitch;
        values.rpy.y = roll;
        values.rpy.z = yaw;
        temp_roll=(temp_roll+roll)/2;
        (void) printf("Roll= %lf,Pitch= %lf,Yaw= %lf\n", temp_roll, pitch, yaw);
        (void) printf("Gyroscope: X=%d, Y=%d, Z=%d\n", rawGyro.x, rawGyro.y, rawGyro.z);
        (void) printf("Accelerometer: X=%d, Y=%d, Z=%d\n", rawAccel.x, rawAccel.y, rawAccel.z);
        xSemaphoreGive(imuMutex);
        arm_check();
        printf("Count:-%d\n",count);
        if(count>3){
            printf("Trigger");
            adjustPump();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

