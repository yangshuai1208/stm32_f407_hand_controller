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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hand_protocol.h"
#include "hand_servo.h"
#include "hand_ack_cache.h"
#include "hand_comm_safety.h"
#include "hand_command_dispatch.h"
#include "uart_dma_rx.h"
#include "uart_line.h"
#include "uart_rx_recovery.h"
#include <string.h>
#include <stdio.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

static void send_seq_conflict(void)
{
    const char msg[] =
        "ERR:SEQ_CONFLICT\r\n";

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)msg,
        sizeof(msg) - 1U,
        100);
}

static const hand_command_dispatch_ops_t hand_command_dispatch_ops =
{
    hand_servo_start_action,
    hand_servo_is_busy,
    send_ack,
    send_seq_conflict
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA直接写入的缓冲区 */
static uint8_t dma_rx_buf[UART_DMA_RX_BUF_SIZE];

/* UART DMA位置和软件队列/组帧状态由独立生产模块管理。 */
static uart_dma_rx_t uart_dma_rx;
static uart_line_rx_t uart_line_rx;
static uart_rx_recovery_t uart_rx_recovery;
static hand_ack_cache_t hand_ack_cache;
static hand_comm_safety_t hand_comm_safety;


uint8_t action_in_progress = 0;

hand_action_t active_action =
    HAND_ACTION_NONE;
		
uint32_t active_seq=0;
// volatile uint8_t hand_action_pending=0;
// volatile hand_action_t hand_pending_action=HAND_ACTION_NONE;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	 uart_dma_rx_init(&uart_dma_rx);
  uart_line_rx_init(&uart_line_rx);
  uart_rx_recovery_init(&uart_rx_recovery);
  hand_ack_cache_init(&hand_ack_cache);
  hand_comm_safety_init(&hand_comm_safety);

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
	HAL_StatusTypeDef uart_start_status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        dma_rx_buf,
		UART_DMA_RX_BUF_SIZE);
	if (uart_start_status == HAL_OK)
	{
		uart_rx_recovery_mark_started(&uart_rx_recovery);
	}
	else
	{
		/* 模块没有初次启动失败入口，将状态置为待恢复并由服务重试。 */
		uart_rx_recovery.state = UART_RX_RECOVERY_WAIT_DMA;
		uart_rx_recovery.last_failure = UART_RX_FAILURE_START;
		uart_rx_recovery.state_tick = HAL_GetTick();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
	/* Consume a latched UART error and cancel motion before recovery can block. */
	hand_comm_safety_service(
		&hand_comm_safety,
		&action_in_progress,
		&active_seq,
		&active_action,
		&hand_ack_cache,
		hand_servo_start_action);

	/* Sole main-loop owner of DMA abort/restart; call once per iteration. */
	uart_rx_recovery_service(
		&uart_rx_recovery,
		&huart1,
		dma_rx_buf,
		&uart_dma_rx,
		&uart_line_rx);

	/* Cancel the active trajectory before dispatching commands or updating servos. */
	hand_comm_safety_service(
		&hand_comm_safety,
		&action_in_progress,
		&active_seq,
		&active_action,
		&hand_ack_cache,
		hand_servo_start_action);

	if (uart_rx_recovery_commands_allowed(&uart_rx_recovery))
	{
	if (uart_line_rx_has_overflow(&uart_line_rx))
	{
    uart_line_rx_recover_overflow(&uart_line_rx);
	}

    /* 1. 处理完整UART命令 */
  char line[UART_LINE_BUFFER_SIZE];

	if (uart_line_rx_poll(&uart_line_rx, line) &&
	    uart_rx_recovery_commands_allowed(&uart_rx_recovery))
		 {
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
            hand_command_dispatch(
                &command,
                &action_in_progress,
                &active_seq,
                &active_action,
                &hand_ack_cache,
                &hand_comm_safety,
                &hand_command_dispatch_ops);
        }

	}

    /*
     * 6. 每次主循环继续推进非阻塞状态机
     */
            }

    HAL_StatusTypeDef update_ret =
        hand_servo_update();

	
    /*
     * 7. 执行发生错误
     */
    if (update_ret != HAL_OK)
    {
        
        if (action_in_progress)
        {
        hand_ack_cache_set(&hand_ack_cache,active_seq,active_action, ACK_STATUS_ERROR);

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
        hand_ack_cache_set(&hand_ack_cache,active_seq,active_action,ACK_STATUS_OK);

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

void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    uart_dma_rx_collect(
        &uart_dma_rx,
        dma_rx_buf,
        Size,
        &uart_line_rx);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL ||
        huart != &huart1 ||
        huart->Instance != USART1)
    {
        return;
    }

    uart_rx_recovery_on_error(
        &uart_rx_recovery,
        huart,
        &huart1);
    hand_comm_safety_notify_uart_error_isr(&hand_comm_safety);
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
