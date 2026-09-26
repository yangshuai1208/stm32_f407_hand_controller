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
    hand_action_t action;
    ack_status_t status;
} ack_record_t;

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
    hand_action_t action,
    ack_status_t status)
{
    ack_record_t *record = ack_cache_find(seq);

   
    if (record != NULL)
    {
        record->status = status;
        return;
    }

    ack_cache[ack_cache_next].valid = 1;
    ack_cache[ack_cache_next].seq = seq;
    ack_cache[ack_cache_next].action = action;
    ack_cache[ack_cache_next].status = status;

    ack_cache_next =
        (ack_cache_next + 1U) % ACK_CACHE_SIZE;
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
#define UART_RX_BUF_SIZE       64U
#define UART_DMA_RX_BUF_SIZE  256U
#define UART_SW_RING_SIZE     512U

/* DMA直接写入的缓冲区 */
static uint8_t dma_rx_buf[UART_DMA_RX_BUF_SIZE];

/* 上一次已经处理到的DMA位置 */
static uint16_t dma_old_pos = 0;

/* 软件环形缓冲区 */
static uint8_t uart_ring[UART_SW_RING_SIZE];

static volatile uint16_t uart_ring_head = 0;
static volatile uint16_t uart_ring_tail = 0;

static volatile uint8_t uart_rx_overflow = 0;

/* 主循环解析ASCII行协议 */
static char uart_line_buf[UART_RX_BUF_SIZE];

static uint8_t uart_line_index = 0;
static uint8_t uart_line_discard = 0;


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
static void uart_ring_push_isr(uint8_t byte)
{
    if (uart_rx_overflow)
    {
        return;
    }

    uint16_t next =
        (uart_ring_head + 1U) %
        UART_SW_RING_SIZE;

    if (next == uart_ring_tail)
    {
        uart_rx_overflow = 1;
        return;
    }

    uart_ring[uart_ring_head] = byte;

    uart_ring_head = next;
}
static uint8_t uart_ring_pop(uint8_t *byte)
{
    if (byte == NULL)
    {
        return 0;
    }

    uint8_t has_data = 0;

    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if (uart_ring_tail != uart_ring_head)
    {
        *byte = uart_ring[uart_ring_tail];

        uart_ring_tail =
            (uart_ring_tail + 1U) %
            UART_SW_RING_SIZE;

        has_data = 1;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_data;
}
static void uart_dma_collect(uint16_t pos)
{
    if (pos > UART_DMA_RX_BUF_SIZE)
    {
        uart_rx_overflow = 1;
        return;
    }

    uint16_t old = dma_old_pos;

    /*
     * 情况1：
     * DMA写入位置没有回绕
     */
    if (pos > old)
    {
        for (uint16_t i = old; i < pos; i++)
        {
            uart_ring_push_isr(dma_rx_buf[i]);
        }
    }

    /*
     * 情况2：
     * DMA写入位置已经回绕
     */
    else if (pos < old)
    {
        for (uint16_t i = old;
             i < UART_DMA_RX_BUF_SIZE;
             i++)
        {
            uart_ring_push_isr(dma_rx_buf[i]);
        }

        for (uint16_t i = 0; i < pos; i++)
        {
            uart_ring_push_isr(dma_rx_buf[i]);
        }
    }

    /*
     * Size可能等于整个DMA缓冲区大小。
     * 处理完后归一化到0。
     */
    dma_old_pos =
        (pos == UART_DMA_RX_BUF_SIZE)
        ? 0U
        : pos;
}

static uint8_t uart_poll_line(
    char out[UART_RX_BUF_SIZE])
{
    uint8_t byte;

    if (!uart_ring_pop(&byte))
    {
        return 0;
    }

    /*
     * 一行接收完成
     */
    if (byte == '\n')
    {
        uint8_t valid =
            !uart_line_discard &&
            uart_line_index > 0;

        if (valid)
        {
            uart_line_buf[uart_line_index] = '\0';

            memcpy(
                out,
                uart_line_buf,
                uart_line_index + 1U);
        }

        uart_line_index = 0;
        uart_line_discard = 0;

        return valid;
    }

    /* 忽略回车 */
    if (byte == '\r')
    {
        return 0;
    }

    /*
     * 行过长时丢弃到下一个换行符，
     * 避免把残缺数据当成新命令。
     */
    if (!uart_line_discard)
    {
        if (uart_line_index <
            UART_RX_BUF_SIZE - 1U)
        {
            uart_line_buf[uart_line_index++] =
                (char)byte;
        }
        else
        {
            uart_line_discard = 1;
        }
    }

    return 0;
}

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
	if (HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        dma_rx_buf,
        UART_DMA_RX_BUF_SIZE) != HAL_OK)
	{
    Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
	if (uart_rx_overflow)
	{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    uart_ring_tail = uart_ring_head;
    uart_rx_overflow = 0;

    if (primask == 0U)
    {
        __enable_irq();
    }

    uart_line_index = 0;

    /*
     * 丢弃至下一个换行符，
     * 防止将残缺数据当作有效命令。
     */
    uart_line_discard = 1;
	}

    /* 1. 处理完整UART命令 */
  char line[UART_RX_BUF_SIZE];

	if (uart_poll_line(line))
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
            /*
             * 2. 先查SEQ缓存
             */
            ack_record_t *old_record =
                ack_cache_find(
                    command.seq);

       if (old_record != NULL)
    {
 
        if (old_record->action != command.action)
        {
            const char msg[] =
            "ERR:SEQ_CONFLICT\r\n";

            HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1U,
            100);
        }
    else
    {
      
        send_ack(
            command.seq,
            old_record->status);
    }
    }

            /*
             * 3. STOP最高优先级
             */
            else if (
                command.action ==
                HAND_ACTION_STOP)
            {
                uint8_t had_active =action_in_progress;

                uint32_t old_seq =active_seq;

                hand_action_t old_action = active_action;

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
                       ack_cache_set(old_seq,old_action,ACK_STATUS_PREEMPTED);

                        send_ack(
                            old_seq,
                            ACK_STATUS_PREEMPTED);
                    }

                    active_seq =
                        command.seq;

                    active_action =
                        HAND_ACTION_STOP;

                    action_in_progress = 1;

                ack_cache_set(command.seq,command.action,ACK_STATUS_IN_PROGRESS);

                    send_ack(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);
                }
                else
                {
                  ack_cache_set(command.seq,command.action,ACK_STATUS_ERROR);

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

                  ack_cache_set(command.seq,command.action,ACK_STATUS_IN_PROGRESS);
                    send_ack(
                        command.seq,
                        ACK_STATUS_IN_PROGRESS);
                }
                else
                {
            ack_cache_set(command.seq,command.action,ACK_STATUS_ERROR);

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
            ack_cache_set(command.seq,command.action,ACK_STATUS_BUSY);

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
        ack_cache_set(active_seq,active_action, ACK_STATUS_ERROR);

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
        ack_cache_set(active_seq,active_action,ACK_STATUS_OK);

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

    uart_dma_collect(Size);
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
