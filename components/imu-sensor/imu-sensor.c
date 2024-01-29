#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include <math.h>
#include "imu-sensor.h"


#define ICM20948_ADDR_1            0x68
#define ICM20948_ADDR_2            0x0C
#define PWR_MGMT_1 0x06
#define MAGNET_MODE 0x31
#define ICM20948_REG_ACCEL_CONFIG 0x14
#define ICM20948_REG_GYRO_CONFIG 0x02
#define ICM20948_REG_ACCEL_XOUT  0x2D
#define ICM20948_REG_TEMP_OUT    0x39
#define ICM20948_REG_GYRO_XOUT   0x33
#define ICM20948_REG_MAG_XOUT 0x11
#define SDA_PIN 5
#define SCL_PIN 6


SemaphoreHandle_t imuMutex;
IMUData values;
static i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN, // Replace with your actual SDA pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_PIN, // Replace with your actual SCL pin
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // I2C clock frequency
};


void i2c_init() {
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}

void write_register(uint8_t master_reg_address,uint8_t reg_address, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (master_reg_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint16_t read_register_16(uint8_t master_reg_address,uint8_t reg_address) {
    uint8_t data_h, data_l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (master_reg_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (master_reg_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (data_h << 8) | data_l;
}


IMUData getIMUValues() {
    xSemaphoreTake(imuMutex, portMAX_DELAY);
    IMUData copy = values;
    xSemaphoreGive(imuMutex);
    return copy;
}


void imu_task(void *pvParameter) {
    write_register(ICM20948_ADDR_1,ICM20948_REG_ACCEL_CONFIG, 0b00000000);
    write_register(ICM20948_ADDR_1,ICM20948_REG_GYRO_CONFIG, 0b00000000);

    while (1) {
// Read accelerometer data
        xSemaphoreTake(imuMutex, portMAX_DELAY);
        values.accelerometer.x = (read_register_16(ICM20948_ADDR_1,ICM20948_REG_ACCEL_XOUT)) / 2048.0;
        values.accelerometer.y = (read_register_16(ICM20948_ADDR_1,ICM20948_REG_ACCEL_XOUT + 2)) / 2048.0;
        values.accelerometer.z = (read_register_16(ICM20948_ADDR_1,ICM20948_REG_ACCEL_XOUT + 4)) / 2048.0;

        float gyro_scale = 250 / 32767.0;
// Read gyroscope data
        values.gyroscope.x = read_register_16(ICM20948_ADDR_1,ICM20948_REG_GYRO_XOUT)*gyro_scale;
        values.gyroscope.y = read_register_16(ICM20948_ADDR_1,ICM20948_REG_GYRO_XOUT + 2)*gyro_scale;
        values.gyroscope.z = read_register_16(ICM20948_ADDR_1,ICM20948_REG_GYRO_XOUT + 4)*gyro_scale ;
        values.magnetometer.x = read_register_16(ICM20948_ADDR_2,ICM20948_REG_MAG_XOUT);
        values.magnetometer.y = read_register_16(ICM20948_ADDR_2,ICM20948_REG_MAG_XOUT+2);
        values.magnetometer.z = read_register_16(ICM20948_ADDR_2,ICM20948_REG_MAG_XOUT+4);
        printf("Accelerometer: X=%.2f, Y=%.2f, Z=%.2f\n", values.accelerometer.x, values.accelerometer.y, values.accelerometer.z);
        printf("Gyroscope: X=%.2f, Y=%.2f, Z=%.2f\n", values.gyroscope.x, values.gyroscope.y, values.gyroscope.z);
        printf("Magnetometer: X=%f, Y=%f, Z=%f\n", values.magnetometer.x, values.magnetometer.y, values.magnetometer.z);

        xSemaphoreGive(imuMutex);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void init_imu() {
    // Initialize I2C
    imuMutex = xSemaphoreCreateMutex();

    i2c_init();

    // Write value 40 to register 0x06
    write_register(ICM20948_ADDR_1,PWR_MGMT_1, 40);
    write_register(ICM20948_ADDR_2,MAGNET_MODE, 4);
    // Create and start the IMU task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}