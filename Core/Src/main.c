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
volatile uint8_t hand_action_pending=0;
volatile hand_action_t hand_pending_action=HAND_ACTION_NONE;

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
    /* 1. 有新命令，并且当前没有动作 */
    if (hand_action_pending &&
        !hand_servo_is_busy())
    {
        hand_action_t action;

        __disable_irq();

        action = hand_pending_action;
        hand_action_pending = 0;

        __enable_irq();


        if (hand_servo_start_action(action) == HAL_OK)
        {
            active_action = action;
            action_in_progress = 1;
        }
    }


    /* 2. 每一次主循环都必须推进状态机 */
    HAL_StatusTypeDef update_ret =
        hand_servo_update();


    if (update_ret != HAL_OK)
    {
        action_in_progress = 0;
        active_action = HAND_ACTION_NONE;
    }


    /* 3. 动作刚刚执行完成 */
    if (action_in_progress &&
        !hand_servo_is_busy())
    {
        char ack_msg[64];

        snprintf(
            ack_msg,
            sizeof(ack_msg),
            "ACTION:%s OK\r\n",
            hand_action_to_ack_name(active_action));

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)ack_msg,
            strlen(ack_msg),
            100);

        action_in_progress = 0;
        active_action = HAND_ACTION_NONE;
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
static const char * hand_action_to_ack_name(hand_action_t action)
{
	switch(action){
		case	HAND_ACTION_OPEN:
			return "HAND_OPEN";
		
		case	HAND_ACTION_GRAB:
			return "HAND_GRAB";
				
		case	HAND_ACTION_RELEASE:
			return "HAND_RELEASE";
						
		case	HAND_ACTION_STOP:
			return "HAND_STOP";
								
		case	HAND_ACTION_NONE:
		default:
			return "HAND_NONE";
	}
}		


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (uart_rx_byte == '\n') {
            uart_line_buf[uart_line_index] = '\0';

            if (uart_line_index > 0) {
                hand_pending_action = hand_protocol_parse(uart_line_buf);
                hand_action_pending = 1;
            }

            uart_line_index = 0;
        } else if (uart_rx_byte != '\r') {
            if (uart_line_index < UART_RX_BUF_SIZE - 1) {
                uart_line_buf[uart_line_index++] = uart_rx_byte;
            } else {
                uart_line_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
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
