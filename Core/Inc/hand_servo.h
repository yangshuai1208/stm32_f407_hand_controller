#ifndef HAND_SERVO_H
#define HAND_SERVO_H

#include "main.h"
#include "hand_protocol.h"
#include <stdint.h>

#define HAND_SERVO_NUM	5

typedef enum
{
	HAND_FINGER_THUMB=0,
	HAND_FINGER_INDEX,
	HAND_FINGER_MIDDLE,
	HAND_FINGER_RING,
	HAND_FINGER_LITTLE	
}hand_finger_t;	

HAL_StatusTypeDef hand_servo_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef hand_servo_apply_action(hand_action_t action);
HAL_StatusTypeDef hand_servo_set_all_stop(void);
HAL_StatusTypeDef hand_servo_set_finger_angle(hand_finger_t finger,uint16_t angle);

#endif
