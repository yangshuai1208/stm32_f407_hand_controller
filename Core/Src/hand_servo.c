#include "hand_servo.h"
#include "pca9685.h"

#define THUMB_SERVO_CHANNEL     0
#define INDEX_SERVO_CHANNEL     1
#define MIDDLE_SERVO_CHANNEL    2
#define RING_SERVO_CHANNEL      3
#define LITTLE_SERVO_CHANNEL    4

#define SERVO_MOVE_STEP         5
#define SERVO_MOVE_DELAY_MS     20

static const uint8_t hand_servo_channels[HAND_SERVO_NUM] =
{
    THUMB_SERVO_CHANNEL,
    INDEX_SERVO_CHANNEL,
    MIDDLE_SERVO_CHANNEL,
    RING_SERVO_CHANNEL,
    LITTLE_SERVO_CHANNEL
};

static const uint16_t finger_min_angles[HAND_SERVO_NUM] =
{
    20, 20, 20, 20, 20
};

static const uint16_t finger_max_angles[HAND_SERVO_NUM] =
{
    150, 150, 150, 150, 150
};

/*
 * current_angles ������Ҫ���޸ģ����Բ���д const
 */
static uint16_t current_angles[HAND_SERVO_NUM] =
{
    90, 90, 90, 90, 90
};

static uint16_t target_angles[HAND_SERVO_NUM] =
{
    90, 90, 90, 90, 90
};
static hand_motion_state_t motion_state = HAND_MOTION_IDLE;

static uint32_t last_move_tick=0;


static const uint16_t hand_open_angles[HAND_SERVO_NUM] =
{
    40, 40, 40, 40, 40
};

static const uint16_t hand_grab_angles[HAND_SERVO_NUM] =
{
    120, 120, 120, 120, 120
};

static const uint16_t hand_release_angles[HAND_SERVO_NUM] =
{
    70, 70, 70, 70, 70
};

static const uint16_t hand_stop_angles[HAND_SERVO_NUM] =
{
    90, 90, 90, 90, 90
};

static uint16_t hand_servo_limit_angle(uint8_t index, uint16_t angle)
{
    if (index >= HAND_SERVO_NUM) {
        return 90;
    }

    if (angle < finger_min_angles[index]) {
        return finger_min_angles[index];
    }

    if (angle > finger_max_angles[index]) {
        return finger_max_angles[index];
    }

    return angle;
}
static void hand_servo_set_target(const uint16_t angles[])
{
    for(uint8_t i=0;i<HAND_SERVO_NUM;i++)
    {
        target_angles[i]=hand_servo_limit_angle(i,angles[i]);
    }
}
static HAL_StatusTypeDef hand_servo_move_one_smooth(uint8_t index, uint16_t target_angle)
{
    if (index >= HAND_SERVO_NUM) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;
    uint8_t channel = hand_servo_channels[index];

    /*
     * ����������Ŀ��Ƕȣ����Ǹĳ�ͨ����
     */
    target_angle = hand_servo_limit_angle(index, target_angle);

    while (current_angles[index] != target_angle) {
        if (current_angles[index] < target_angle) {
            if ((target_angle - current_angles[index]) > SERVO_MOVE_STEP) {
                current_angles[index] += SERVO_MOVE_STEP;
            } else {
                current_angles[index] = target_angle;
            }
        } else {
            if ((current_angles[index] - target_angle) > SERVO_MOVE_STEP) {
                current_angles[index] -= SERVO_MOVE_STEP;
            } else {
                current_angles[index] = target_angle;
            }
        }

        ret = pca9685_set_servo_angle(channel, current_angles[index]);
        if (ret != HAL_OK) {
            return ret;
        }

        HAL_Delay(SERVO_MOVE_DELAY_MS);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef hand_servo_apply_angles(const uint16_t angles[])
{
    HAL_StatusTypeDef ret;

    for (uint8_t i = 0; i < HAND_SERVO_NUM; i++) {
        ret = hand_servo_move_one_smooth(i, angles[i]);
        if (ret != HAL_OK) {
            return ret;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef hand_servo_init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;

    ret = pca9685_init(hi2c);
    if (ret != HAL_OK) {
        return ret;
    }

    for (uint8_t i = 0; i < HAND_SERVO_NUM; i++) {
        current_angles[i] = hand_stop_angles[i];

        ret = pca9685_set_servo_angle(hand_servo_channels[i], current_angles[i]);
        if (ret != HAL_OK) {
            return ret;
        }

        HAL_Delay(50);
    }

    return HAL_OK;
}

HAL_StatusTypeDef hand_servo_set_all_stop(void)
{
    return hand_servo_apply_angles(hand_stop_angles);
}

HAL_StatusTypeDef hand_servo_set_finger_angle(hand_finger_t finger, uint16_t angle)
{
    if (finger >= HAND_SERVO_NUM) {
        return HAL_ERROR;
    }

    return hand_servo_move_one_smooth((uint8_t)finger, angle);
}

HAL_StatusTypeDef hand_servo_apply_action(hand_action_t action)
{
    switch (action) {
    case HAND_ACTION_OPEN:
        return hand_servo_apply_angles(hand_open_angles);

    case HAND_ACTION_GRAB:
        return hand_servo_apply_angles(hand_grab_angles);

    case HAND_ACTION_RELEASE:
        return hand_servo_apply_angles(hand_release_angles);

    case HAND_ACTION_STOP:
        return hand_servo_apply_angles(hand_stop_angles);

    case HAND_ACTION_NONE:
    default:
        return hand_servo_apply_angles(hand_stop_angles);
    }
}
HAL_StatusTypeDef hand_servo_start_action(
    hand_action_t action)
{
    switch (action)
    {
    case HAND_ACTION_OPEN:

        hand_servo_set_target(
            hand_open_angles);

        motion_state =
            HAND_MOTION_OPENING;

        break;


    case HAND_ACTION_GRAB:

        hand_servo_set_target(
            hand_grab_angles);

        motion_state =
            HAND_MOTION_GRABBING;

        break;


    case HAND_ACTION_RELEASE:

        hand_servo_set_target(
            hand_release_angles);

        motion_state =
            HAND_MOTION_RELEASING;

        break;


    case HAND_ACTION_STOP:

        hand_servo_set_target(
            hand_stop_angles);

        motion_state =
            HAND_MOTION_STOPPING;

        break;


    case HAND_ACTION_NONE:
    default:

        return HAL_ERROR;
    }


    last_move_tick = HAL_GetTick();

    return HAL_OK;
}
HAL_StatusTypeDef hand_servo_update(void)
{
    if(motion_state==HAND_MOTION_IDLE)
    {
        return HAL_OK;
    }
    if (motion_state == HAND_MOTION_FAULT)
    {
        return HAL_ERROR;
    }
    uint32_t now=HAL_GetTick();

    if((now-last_move_tick)<SERVO_MOVE_DELAY_MS)
    {
        return HAL_OK;
    }
    last_move_tick=now;

    uint8_t all_reached=1;

    
    for (uint8_t i = 0;
         i < HAND_SERVO_NUM;
         i++)
    {
        if (current_angles[i] <
            target_angles[i])
        {
            all_reached = 0;

            if ((target_angles[i] -
                 current_angles[i]) >
                SERVO_MOVE_STEP)
            {
                current_angles[i] +=
                    SERVO_MOVE_STEP;
            }
            else
            {
                current_angles[i] =
                    target_angles[i];
            }
        }
        else if (current_angles[i] >
                 target_angles[i])
        {
            all_reached = 0;

            if ((current_angles[i] -
                 target_angles[i]) >
                SERVO_MOVE_STEP)
            {
                current_angles[i] -=
                    SERVO_MOVE_STEP;
            }
            else
            {
                current_angles[i] =
                    target_angles[i];
            }
        }


        if (pca9685_set_servo_angle(
                hand_servo_channels[i],
                current_angles[i])
            != HAL_OK)
        {
            motion_state =
                HAND_MOTION_FAULT;

            return HAL_ERROR;
        }
    }


    if (all_reached)
    {
        motion_state =
            HAND_MOTION_IDLE;
    }


    return HAL_OK;
}
uint8_t hand_servo_is_busy(void)
{
    return motion_state != HAND_MOTION_IDLE &&
           motion_state != HAND_MOTION_FAULT;
}
hand_motion_state_t hand_servo_get_state(void)
{
    return motion_state;
}
