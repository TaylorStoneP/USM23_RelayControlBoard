/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_utils.h"
#include "can_utils.h"
#include "control.h"
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t STARTUP_COMPLETE = False;
uint8_t INITIAL_PC = False;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TaskScheduleHandler(){
SCHEDULE_HANDLE(SCH_LED_FAULT)
	HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
}
SCHEDULE_HANDLE(SCH_LED_INDICATOR)
	HAL_GPIO_TogglePin(LED_INDICATOR_GPIO_Port, LED_INDICATOR_Pin);
}
SCHEDULE_HANDLE(SCH_LED_OKAY)
	HAL_GPIO_TogglePin(LED_OKAY_GPIO_Port, LED_OKAY_Pin);
}
SCHEDULE_HANDLE(SCH_CAN_STATE)
	CAN_Transmit_State();
}
SCHEDULE_HANDLE(SCH_DEBOUNCE_RELAYS)
	Debounce_Relays();
}
SCHEDULE_HANDLE(SCH_DEBOUNCE_PC)
	Debounce_PC();
}
SCHEDULE_HANDLE(SCH_DEBOUNCE_TS_REQUEST)
	Debounce_TS();
}
SCHEDULE_HANDLE(SCH_DEBOUNCE_PWR)
	Debounce_Power();
}
SCHEDULE_HANDLE(SCH_DEVCON_CHARGER)
	DevCon_Charger(False);
}
SCHEDULE_HANDLE(SCH_DEVCON_OVERRIDE)
	DevCon_Override(False);
}
SCHEDULE_HANDLE(SCH_CHARGER_CHECK)
	if(Status_Check(STATUS_Idle)){
		if(hrcb.DEVCON_CHARGER){
			ScheduleTaskStop(SCH_CHARGER_CHECK);
			PC_Routine_Charge_Start();
		}
	}
}
SCHEDULE_HANDLE(SCH_COMPLETE_STARTUP)
	STARTUP_COMPLETE = True;
}
SCHEDULE_HANDLE(SCH_PC_OVERFLOW)
	PC_Overflow();
}
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Start CAN
  HAL_CAN_Start(&hcan);
  if(hcan.State!=HAL_CAN_STATE_LISTENING){
	  HAL_NVIC_SystemReset();
  }
  HAL_TIM_Base_Start_IT(&SOFTCLK_TIMER_TYPE);


  hrcb.STATUS = STATUS_Startup;
  // Activate interrupts for the TX Mailbox Empty and for an incoming message
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);

  CAN_UTIL_Setup(CAN_TX_STATE, 0x00000A84, 8);

  Startup();

  ScheduleTask(SCH_CAN_STATE, 500, True, 0);

  Input_Check();

  Status_Set(STATUS_Idle);

  ScheduleTask(SCH_CHARGER_CHECK, 1000, True, 0);

  ScheduleTask(SCH_COMPLETE_STARTUP, 1200, False, 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  CAN_RX_Handler();
	  TaskScheduleSoftClock();
	  TaskScheduleHandler();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef CAN_Filter;
  CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
  CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN_Filter.FilterBank = 0;
  CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;

  uint32_t filter_id = 	 0x18FF50E5;
  uint32_t filter_mask = 0x1FFFFFFF;

  CAN_Filter.FilterIdHigh = filter_id >> 13 & 0xFFFF;
  CAN_Filter.FilterIdLow = filter_id << 3 & 0xFFF8;
  CAN_Filter.FilterMaskIdHigh = filter_mask >> 13 & 0xFFFF;
  CAN_Filter.FilterMaskIdLow = filter_mask << 3 & 0xFFF8;

  HAL_CAN_ConfigFilter(&hcan, &CAN_Filter);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY_TRIGGER_AIRPOS_Pin|RELAY_TRIGGER_AIRNEG_Pin|RELAY_TRIGGER_PC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_FAULT_Pin|LED_INDICATOR_Pin|LED_OKAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_TRIGGER_AIRPOS_Pin RELAY_TRIGGER_AIRNEG_Pin RELAY_TRIGGER_PC_Pin */
  GPIO_InitStruct.Pin = RELAY_TRIGGER_AIRPOS_Pin|RELAY_TRIGGER_AIRNEG_Pin|RELAY_TRIGGER_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IND_PC_Pin */
  GPIO_InitStruct.Pin = IND_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IND_PC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_FAULT_Pin LED_INDICATOR_Pin LED_OKAY_Pin */
  GPIO_InitStruct.Pin = LED_FAULT_Pin|LED_INDICATOR_Pin|LED_OKAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IND_PWR_FAN1_Pin IND_PWR_FAN2_Pin IND_PWR_IMD_Pin */
  GPIO_InitStruct.Pin = IND_PWR_FAN1_Pin|IND_PWR_FAN2_Pin|IND_PWR_IMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : REQUEST_TS_Pin */
  GPIO_InitStruct.Pin = REQUEST_TS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(REQUEST_TS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IND_60V_Pin RELAY_AUX_AIRPOS_Pin RELAY_AUX_AIRNEG_Pin RELAY_AUX_PC_Pin */
  GPIO_InitStruct.Pin = IND_60V_Pin|RELAY_AUX_AIRPOS_Pin|RELAY_AUX_AIRNEG_Pin|RELAY_AUX_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
	if (htim == &SOFTCLK_TIMER_TYPE) {
		TaskScheduleSoftClock_FlagSet();
	}else if(htim == &PC_TIMER_TYPE){
		if(INITIAL_PC==True){
			ScheduleTask(SCH_PC_OVERFLOW, 0, False, 0);
		}else{
			INITIAL_PC = True;
		}
	}
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	if(Status_Check(STATUS_Startup)){
		return;
	}
	if(Status_Check(STATUS_Error)){
		return;
	}
	if(STARTUP_COMPLETE==False){
		return;
	}
	switch(GPIO_Pin){
	case IND_PWR_FAN1_Pin:
		ScheduleTask(SCH_DEBOUNCE_PWR, DEBOUNCE_PERIOD, 0, 0);
		break;
	case IND_PWR_FAN2_Pin:
		ScheduleTask(SCH_DEBOUNCE_PWR, DEBOUNCE_PERIOD, 0, 0);
		break;
	case IND_PWR_IMD_Pin:
		ScheduleTask(SCH_DEBOUNCE_PWR, DEBOUNCE_PERIOD, 0, 0);
		break;
	case IND_PC_Pin:
		ScheduleTask(SCH_DEBOUNCE_PC, DEBOUNCE_PERIOD, 0, 0);
		break;
	case REQUEST_TS_Pin:
		ScheduleTask(SCH_DEBOUNCE_TS_REQUEST, DEBOUNCE_PERIOD, 0, 0);
		break;
	case RELAY_AUX_AIRNEG_Pin:
		if(!BITCHECK(hrcb.RELAY_IT_IGNORE, RI_AIRNEG)){
			ScheduleTask(SCH_DEBOUNCE_RELAYS, DEBOUNCE_PERIOD, 0, 0);
		}
		break;
	case RELAY_AUX_AIRPOS_Pin:
		if(!BITCHECK(hrcb.RELAY_IT_IGNORE, RI_AIRPOS)){
			ScheduleTask(SCH_DEBOUNCE_RELAYS, DEBOUNCE_PERIOD, 0, 0);
		}
		break;
	case RELAY_AUX_PC_Pin:
		if(!BITCHECK(hrcb.RELAY_IT_IGNORE, RI_PC)){
			ScheduleTask(SCH_DEBOUNCE_RELAYS, DEBOUNCE_PERIOD, 0, 0);
		}
		break;
	case IND_60V_Pin:
		ScheduleTask(SCH_DEBOUNCE_RELAYS, DEBOUNCE_PERIOD, 0, 0);
		break;
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
