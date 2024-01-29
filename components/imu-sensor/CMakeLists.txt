idf_component_register(SRCS "imu-sensor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver freertos fusion-algorithm)
