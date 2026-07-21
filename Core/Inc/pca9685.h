#ifndef PCA9685_H
#define PCA9685_H

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef pca9685_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);
HAL_StatusTypeDef pca9685_set_servo_pulse_us(uint8_t channel, uint16_t pulse_us);
HAL_StatusTypeDef pca9685_set_servo_angle(uint8_t channel, uint16_t angle);

#endif
