idf_component_register(SRCS "imu-sensor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver freertos fusion-algorithm I2Cbus esp32-MPU-driver)
