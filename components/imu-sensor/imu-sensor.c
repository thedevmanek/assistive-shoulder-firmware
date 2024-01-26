#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include <math.h>
#include "imu-sensor.h"

#define MPU6050_ADDR            0x68
#define MPU6050_REG_ACCEL_CONFIG 0x14
#define MPU6050_REG_GYRO_CONFIG 0x02
#define MPU6050_REG_ACCEL_XOUT  0x2D
#define MPU6050_REG_TEMP_OUT    0x39
#define MPU6050_REG_GYRO_XOUT   0x33

SemaphoreHandle_t imuMutex;
IMUData values;
static i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 5, // Replace with your actual SDA pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 6, // Replace with your actual SCL pin
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // I2C clock frequency
};


// Calibration offset values
int16_t gyro_offset_x = 0;
int16_t gyro_offset_y = 0;
int16_t gyro_offset_z = 0;

void i2c_init() {
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}

void write_register(uint8_t reg_address, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint16_t read_register_16(uint8_t reg_address) {
    uint8_t data_h, data_l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (data_h << 8) | data_l;
}

void calibrateIMU() {
    // Read accelerometer and gyroscope values multiple times to obtain offsets
    int16_t sum_gyro_x = 0;
    int16_t sum_gyro_y = 0;
    int16_t sum_gyro_z = 0;
    int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
        sum_gyro_x += read_register_16(MPU6050_REG_GYRO_XOUT);
        sum_gyro_y += read_register_16(MPU6050_REG_GYRO_XOUT + 2);
        sum_gyro_z += read_register_16(MPU6050_REG_GYRO_XOUT + 4);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10 ms between samples
    }


    gyro_offset_x = sum_gyro_x / num_samples;
    gyro_offset_y = sum_gyro_y / num_samples;
    gyro_offset_z = sum_gyro_z / num_samples;

}

IMUData getIMUValues(){
    xSemaphoreTake(imuMutex, portMAX_DELAY);
    IMUData copy = values;
    xSemaphoreGive(imuMutex);
    return copy;
}
void imu_task(void *pvParameter) {
    write_register(MPU6050_REG_ACCEL_CONFIG, 0x11);
    write_register(MPU6050_REG_GYRO_CONFIG, 0x11);
    calibrateIMU();
    while (1) {
// Read accelerometer data
        xSemaphoreTake(imuMutex, portMAX_DELAY);
        values.accelerometer.x = (read_register_16(MPU6050_REG_ACCEL_XOUT)) / 2048.0;
        values.accelerometer.y= (read_register_16(MPU6050_REG_ACCEL_XOUT + 2)) / 2048.0;
        values.accelerometer.z = (read_register_16(MPU6050_REG_ACCEL_XOUT + 4)) / 2048.0;

        float gyro_scale=2000/32767;
// Read gyroscope data
        values.gyroscope.x= (read_register_16(MPU6050_REG_GYRO_XOUT) - gyro_offset_x) * gyro_scale;
        values.gyroscope.y = (read_register_16(MPU6050_REG_GYRO_XOUT + 2) - gyro_offset_y) * gyro_scale;
        values.gyroscope.z =(read_register_16(MPU6050_REG_GYRO_XOUT + 4) - gyro_offset_z) * gyro_scale;
        xSemaphoreGive(imuMutex);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void init_imu() {
    // Initialize I2C
    imuMutex = xSemaphoreCreateMutex();

    i2c_init();

    // Write value 40 to register 0x06
    write_register(0x06, 40);

    // Create and start the IMU task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
