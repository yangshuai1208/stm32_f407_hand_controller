/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hand_protocol.h"
#include "hand_servo.h"
#include <string.h>
#include <stdio.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  typedef enum
  {
    ACK_STATUS_NONE=0,
    ACK_STATUS_IN_PROGRESS,
    ACK_STATUS_OK,
    ACK_STATUS_PREEMPTED,
    ACK_STATUS_BUSY,
    ACK_STATUS_ERROR
  }ack_status_t;

  #define ACK_CACHE_SIZE 4

  typedef struct
  {
    uint8_t valid;
    uint32_t seq;
    ack_status_t status;
  }ack_record_t;

  static ack_record_t ack_cache[ACK_CACHE_SIZE];

  static uint8_t ack_cache_next=0;

  static const char *ack_status_to_string(
    ack_status_t status)
{
    switch (status)
    {
    case ACK_STATUS_IN_PROGRESS:
        return "IN_PROGRESS";

    case ACK_STATUS_OK:
        return "OK";

    case ACK_STATUS_PREEMPTED:
        return "PREEMPTED";

    case ACK_STATUS_BUSY:
        return "BUSY";

    case ACK_STATUS_ERROR:
        return "ERROR";

    case ACK_STATUS_NONE:
    default:
        return "UNKNOWN";
    }
}

static ack_record_t *ack_cache_find(
    uint32_t seq)
{
    for (uint8_t i = 0;
         i < ACK_CACHE_SIZE;
         i++)
    {
        if (ack_cache[i].valid &&
            ack_cache[i].seq == seq)
        {
            return &ack_cache[i];
        }
    }

    return NULL;
}

static void ack_cache_set(
    uint32_t seq,
    ack_status_t status)
{
    ack_record_t *record =
        ack_cache_find(seq);

    if (record != NULL)
    {
        record->status = status;
        return;
    }

    ack_cache[ack_cache_next].valid = 1;
    ack_cache[ack_cache_next].seq = seq;
    ack_cache[ack_cache_next].status = status;

    ack_cache_next =
        (ack_cache_next + 1) %
        ACK_CACHE_SIZE;
}
static void send_ack(
    uint32_t seq,
    ack_status_t status)
{
    char buf[64];

    int len =
        snprintf(
            buf,
            sizeof(buf),
            "ACK:%lu %s\r\n",
            (unsigned long)seq,
            ack_status_to_string(status));

    if (len > 0)
    {
        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)buf,
            strlen(buf),
            100);
    }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define UART_RX_BUF_SIZE	64

uint8_t uart_rx_byte=0;
char uart_line_buf[UART_RX_BUF_SIZE];
uint8_t uart_line_index=0;


uint8_t action_in_progress = 0;

hand_action_t active_action =
    HAND_ACTION_NONE;
// volatile uint8_t hand_action_pending=0;
// volatile hand_action_t hand_pending_action=HAND_ACTION_NONE;
char  uart_complete_line[UART_RX_BUF_SIZE];
volatile uint8_t uart_line_ready=0;
uint32_t active_seq=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static const char * hand_action_to_ack_name(hand_action_t  action);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	if(hand_servo_init(&hi2c1)==HAL_OK)
	{
		char msg[]="PCA9685 init OK\r\n";
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),100);
	}
	else
	{
		char msg[]="PCA9685 init FAIL\r\n";
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),100);
	}
	HAL_UART_Receive_IT(&huart1,&uart_rx_byte,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    /* 1. 处理完整UART命令 */
    if (uart_line_ready)
    {
        char line[UART_RX_BUF_SIZE];

        __disable_irq();

        memcpy(
            line,
            uart_complete_line,
            sizeof(line));

        uart_line_ready = 0;

        __enable_irq();


        hand_command_t command;

        if (!hand_protocol_parse_frame(
                line,
                &command))
        {
            char msg[] =
                "ERR:INVALID_FRAME\r\n";

            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)msg,
                strlen(msg),
                100);
        }
        else
        {
            /*
             * 2. 先查SEQ缓存
             */
            ack_record_t *old_record =
                ack_cache_find(
                    command.seq);

            if (old_record != NULL)
            {
                /*
                 * 重复SEQ：
                 * 不重新执行动作，
                 * 只重发缓存状态。
                 */
                send_ack(
                    command.seq,
                    old_record->status);
            }

            /*
             * 3. STOP最高优先级
             */
            else if (
                command.action ==
                HAND_ACTION_STOP)
            {
                uint8_t had_active =
                    action_in_progress;

                uint32_t old_seq =
                    active_seq;

                if (hand_servo_start_action(
                        HAND_ACTION_STOP)
                    == HAL_OK)
                {
                    /*
                     * STOP成功接管之后，
                     * 原动作才正式标记PREEMPTED。
                     */
                    if (had_active)
                    {
                        ack_cache_set(
                            old_seq,
                            ACK_STATUS_PREEMPTED);

                        send_ack(
                            old_seq,
                            ACK_STATUS_PREEMPTED);
                    }

                    active_seq =
                        command.seq;

                    active_action =
                        HAND_ACTION_STOP;

                    action_in_progress = 1;

                    ack_cache_set(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);

                    send_ack(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);
                }
                else
                {
                    ack_cache_set(
                        command.seq,
                        ACK_STATUS_ERROR);

                    send_ack(
                        command.seq,
                        ACK_STATUS_ERROR);
                }
            }

            /*
             * 4. 普通动作：
             * 只有空闲时允许启动
             */
            else if (!hand_servo_is_busy())
            {
                if (hand_servo_start_action(
                        command.action)
                    == HAL_OK)
                {
                    active_seq =
                        command.seq;

                    active_action =
                        command.action;

                    action_in_progress = 1;

                    ack_cache_set(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);

                    send_ack(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);
                }
                else
                {
                    ack_cache_set(
                        command.seq,
                        ACK_STATUS_ERROR);

                    send_ack(
                        command.seq,
                        ACK_STATUS_ERROR);
                }
            }

            /*
             * 5. 当前busy，
             * 普通动作拒绝
             */
            else
            {
                ack_cache_set(
                    command.seq,
                    ACK_STATUS_BUSY);

                send_ack(
                    command.seq,
                    ACK_STATUS_BUSY);
            }
        }
    }


    /*
     * 6. 每次主循环继续推进非阻塞状态机
     */
    HAL_StatusTypeDef update_ret =
        hand_servo_update();


    /*
     * 7. 执行发生错误
     */
    if (update_ret != HAL_OK)
    {
        if (action_in_progress)
        {
            ack_cache_set(
                active_seq,
                ACK_STATUS_ERROR);

            send_ack(
                active_seq,
                ACK_STATUS_ERROR);
        }

        action_in_progress = 0;

        active_action =
            HAND_ACTION_NONE;

        active_seq = 0;
    }


    /*
     * 8. 当前动作完成
     */
    if (action_in_progress &&
        !hand_servo_is_busy())
    {
        ack_cache_set(
            active_seq,
            ACK_STATUS_OK);

        send_ack(
            active_seq,
            ACK_STATUS_OK);

        action_in_progress = 0;

        active_action =
            HAND_ACTION_NONE;

        active_seq = 0;
    }
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// static const char * hand_action_to_ack_name(hand_action_t action)
// {
// 	switch(action){
// 		case	HAND_ACTION_OPEN:
// 			return "HAND_OPEN";
  
// 		case	HAND_ACTION_GRAB:
// 			return "HAND_GRAB";
				
// 		case	HAND_ACTION_RELEASE:
// 			return "HAND_RELEASE";
						
// 		case	HAND_ACTION_STOP:
// 			return "HAND_STOP";
								
// 		case	HAND_ACTION_NONE:
// 		default:
// 			return "HAND_NONE";
// 	}
// }		

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_byte == '\n')
        {
            uart_line_buf[
                uart_line_index] = '\0';

            if (uart_line_index > 0 &&
                !uart_line_ready)
            {
                memcpy(
                    uart_complete_line,
                    uart_line_buf,
                    uart_line_index + 1);

                uart_line_ready = 1;
            }

            uart_line_index = 0;
        }

        else if (uart_rx_byte != '\r')
        {
            if (uart_line_index <
                UART_RX_BUF_SIZE - 1)
            {
                uart_line_buf[
                    uart_line_index++] =
                    uart_rx_byte;
            }
            else
            {
                uart_line_index = 0;
            }
        }

        HAL_UART_Receive_IT(
            &huart1,
            &uart_rx_byte,
            1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	
		
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
