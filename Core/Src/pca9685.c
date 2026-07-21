#include "pca9685.h"

#define PCA9685_ADDR        (0x40 << 1)

#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_PERIOD_US     20000

static I2C_HandleTypeDef *g_pca9685_i2c = NULL;

static HAL_StatusTypeDef pca9685_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(g_pca9685_i2c,
                             PCA9685_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             100);
}

HAL_StatusTypeDef pca9685_init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;

    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    g_pca9685_i2c = hi2c;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x10);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = pca9685_write_reg(PCA9685_PRESCALE, 121);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = pca9685_write_reg(PCA9685_MODE1, 0x20);
    if (ret != HAL_OK) {
        return ret;
    }

    HAL_Delay(5);

    ret = pca9685_write_reg(PCA9685_MODE1, 0xA1);
    if (ret != HAL_OK) {
        return ret;
    }

    HAL_Delay(5);

    return HAL_OK;
}

HAL_StatusTypeDef pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) {
        return HAL_ERROR;
    }

    uint8_t data[4];

    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;

    return HAL_I2C_Mem_Write(g_pca9685_i2c,
                             PCA9685_ADDR,
                             PCA9685_LED0_ON_L + 4 * channel,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             4,
                             100);
}

HAL_StatusTypeDef pca9685_set_servo_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    uint16_t off = (uint16_t)((pulse_us * 4096UL) / SERVO_PERIOD_US);

    if (off > 4095) {
        off = 4095;
    }

    return pca9685_set_pwm(channel, 0, off);
}

HAL_StatusTypeDef pca9685_set_servo_angle(uint8_t channel, uint16_t angle)
{
    if (angle > 180) {
        angle = 180;
    }

    uint16_t pulse_us = SERVO_MIN_PULSE_US +
                        (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;

    return pca9685_set_servo_pulse_us(channel, pulse_us);
}
