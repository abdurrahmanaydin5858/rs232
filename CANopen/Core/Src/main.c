/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os.h"
#include <string.h>
#include <stdbool.h>
#include "CANopen/CO_app_STM32.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_14
#define LED_PORT GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

// CANopenNode STM32 structure
CANopenNodeSTM32 canopenNodeSTM32;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANopenTask */
osThreadId_t CANopenTaskHandle;
const osThreadAttr_t CANopenTask_attributes = {
  .name = "CANopenTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RS232Task */
osThreadId_t RS232TaskHandle;
const osThreadAttr_t RS232Task_attributes = {
  .name = "RS232Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartCANopenTask(void *argument);
void StartRS232Task(void *argument);

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize CANopenNode STM32
  canopenNodeSTM32.desiredNodeID = 1;  // Set your desired node ID
  canopenNodeSTM32.baudrate = 500000;  // 500 kbps
  canopenNodeSTM32.timerHandle = &htim2;
  canopenNodeSTM32.CANHandle = &hcan1;
  canopenNodeSTM32.HWInitFunction = MX_CAN1_Init;

  // Initialize CANopenNode
  if (canopen_app_init(&canopenNodeSTM32) != 0) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CANopenTask */
  CANopenTaskHandle = osThreadNew(StartCANopenTask, NULL, &CANopenTask_attributes);

  /* creation of RS232Task */
  RS232TaskHandle = osThreadNew(StartRS232Task, NULL, &RS232Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  
  /* Enable CAN interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  /* Configure LED pin */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  
  /* Turn off LED initially */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief CAN1 RX0 interrupt handler
 */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCANopenTask */
/**
* @brief Function implementing the CANopenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANopenTask */
void StartCANopenTask(void *argument)
{
  /* USER CODE BEGIN StartCANopenTask */
  /* Infinite loop */
  for(;;)
  {
    /* Process CANopen stack */
    canopen_app_process();
    
    /* Task delay */
    osDelay(1);
  }
  /* USER CODE END StartCANopenTask */
}

/* USER CODE BEGIN Header_StartRS232Task */
/**
* @brief Function implementing the RS232Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRS232Task */
void StartRS232Task(void *argument)
{
  /* USER CODE BEGIN StartRS232Task */
  const char* connected_msg = "Connected\r\n";
  const char* led_on_msg = "LED ON\r\n";
  const char* led_off_msg = "LED OFF\r\n";
  const char* help_msg = "Commands: ON, OFF, STATUS\r\n";
  const char* status_msg = "Status: ";
  const char* unknown_cmd_msg = "Unknown command\r\n";
  const char* prompt_msg = "> ";
  
  uint16_t connected_length = strlen(connected_msg);
  uint16_t led_on_length = strlen(led_on_msg);
  uint16_t led_off_length = strlen(led_off_msg);
  uint16_t help_length = strlen(help_msg);
  uint16_t status_length = strlen(status_msg);
  uint16_t unknown_cmd_length = strlen(unknown_cmd_msg);
  uint16_t prompt_length = strlen(prompt_msg);
  
  uint8_t rx_buffer[50];
  char cmd_buffer[20];
  uint8_t cmd_index = 0;
  bool led_state = false;
  
  /* Send initial connection message */
  HAL_UART_Transmit(&huart1, (uint8_t*)connected_msg, connected_length, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)help_msg, help_length, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)prompt_msg, prompt_length, 1000);
  
  /* Infinite loop */
  for(;;)
  {
    /* Check if data received with longer timeout */
    if (HAL_UART_Receive(&huart1, &rx_buffer[0], 1, 100) == HAL_OK)
    {
      char received_char = rx_buffer[0];
      
      /* Echo character back immediately */
      HAL_UART_Transmit(&huart1, &rx_buffer[0], 1, 100);
      
      /* Handle backspace */
      if (received_char == '\b' || received_char == 127)
      {
        if (cmd_index > 0)
        {
          cmd_index--;
          /* Send backspace to terminal */
          HAL_UART_Transmit(&huart1, (uint8_t*)" \b", 2, 100);
        }
      }
      /* Handle enter key */
      else if (received_char == '\r' || received_char == '\n')
      {
        /* Send new line */
        HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
        
        if (cmd_index > 0)
        {
          cmd_buffer[cmd_index] = '\0';
          
          /* Process command */
          if (strcmp(cmd_buffer, "ON") == 0)
          {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            led_state = true;
            HAL_UART_Transmit(&huart1, (uint8_t*)led_on_msg, led_on_length, 1000);
          }
          else if (strcmp(cmd_buffer, "OFF") == 0)
          {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            led_state = false;
            HAL_UART_Transmit(&huart1, (uint8_t*)led_off_msg, led_off_length, 1000);
          }
          else if (strcmp(cmd_buffer, "STATUS") == 0)
          {
            HAL_UART_Transmit(&huart1, (uint8_t*)status_msg, status_length, 1000);
            if (led_state)
            {
              HAL_UART_Transmit(&huart1, (uint8_t*)led_on_msg, led_on_length, 1000);
            }
            else
            {
              HAL_UART_Transmit(&huart1, (uint8_t*)led_off_msg, led_off_length, 1000);
            }
          }
          else
          {
            HAL_UART_Transmit(&huart1, (uint8_t*)unknown_cmd_msg, unknown_cmd_length, 1000);
          }
          
          cmd_index = 0;
        }
        
        /* Send prompt */
        HAL_UART_Transmit(&huart1, (uint8_t*)prompt_msg, prompt_length, 1000);
      }
      /* Handle normal character */
      else if (received_char >= 32 && received_char <= 126 && cmd_index < 19)
      {
        cmd_buffer[cmd_index++] = received_char;
      }
    }
    
    /* Small delay to prevent blocking */
    osDelay(1);
  }
  /* USER CODE END StartRS232Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM2) {
    /* CANopenNode 1ms interrupt */
    canopen_app_interrupt();
  }
  /* USER CODE END Callback 1 */
}

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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;  /* 84MHz / 84 = 1MHz */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  
  /* Start TIM2 */
  HAL_TIM_Base_Start(&htim2);
  
  /* USER CODE END TIM2_Init 2 */

}
