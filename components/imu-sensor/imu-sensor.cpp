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
static I2C_t& i2c                     = i2c0;  // i2c0 or i2c1
static constexpr gpio_num_t SDA       = GPIO_NUM_5;
static constexpr gpio_num_t SCL       = GPIO_NUM_6;
static constexpr uint32_t CLOCK_SPEED = 10000;  // 400 KHz

IMUData values;
SemaphoreHandle_t imuMutex;
/* MPU configuration */

static constexpr uint16_t kSampleRate      = 250;  // Hz
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_10HZ;

/*-*/

static const char* TAG = "example";

static void mpuTask(void*);


// Main
void init_imu()
{   imuMutex = xSemaphoreCreateMutex();
    printf("$ MPU Driver Example: Advanced\n");
    fflush(stdout);
    // Initialize bus through either the Library API or esp-idf API
    i2c.begin(SDA, SCL, CLOCK_SPEED);

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
}

/* Tasks */

static MPU_t MPU;
float roll{0}, pitch{0}, yaw{0};


extern "C" IMUData getIMUValues() {
    xSemaphoreTake(imuMutex, portMAX_DELAY);
    IMUData copy = values;
    xSemaphoreGive(imuMutex);
    return copy;
}

static void mpuTask(void*)
{
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
//    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
    MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin

    // Reading Loop
    while (true) {
        xSemaphoreTake(imuMutex, portMAX_DELAY);
        mpud::raw_axes_t rawAccel;     // holds x, y, z axes as int16
        mpud::raw_axes_t rawGyro;      // holds x, y, z axes as int16
        MPU.acceleration(&rawAccel);  // fetch raw data from the registers
        MPU.rotation(&rawGyro);       // fetch raw data from the registers

        const FusionVector accelerometer = {{(float)rawAccel.x, (float)rawAccel.y,(float) rawAccel.z}};
        const FusionVector gyroscope = {{(float)rawGyro.x, (float)rawGyro.y,(float) rawGyro.z}};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.01f);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        (void)printf("Roll= %lf,Pitch= %lf,Yaw= %lf\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        (void)printf("Gyroscope: X=%d, Y=%d, Z=%d\n", rawGyro.x, rawGyro.y, rawGyro.z);
        (void)printf("Accelerometer: X=%d, Y=%d, Z=%d\n", rawAccel.x, rawAccel.y,
               rawAccel.z);

        values.accelerometer.x=rawAccel.x;
        values.accelerometer.y=rawAccel.y;
        values.accelerometer.z=rawAccel.z;
        values.gyroscope.x=rawGyro.x;
        values.gyroscope.y=rawGyro.y;
        values.gyroscope.z=rawGyro.z;
        values.rpy.x=euler.angle.roll;
        values.rpy.y=euler.angle.pitch;
        values.rpy.z=euler.angle.yaw;

        xSemaphoreGive(imuMutex);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

