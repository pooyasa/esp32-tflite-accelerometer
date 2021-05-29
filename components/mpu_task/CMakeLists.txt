idf_component_register(SRCS ./mpu_task.cpp
                       INCLUDE_DIRS include
                       REQUIRES I2Cbus
                       mpu_driver)

