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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "lwip/sockets.h"
#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MESSAGE_TEXT_MAXLEN (128UL)
typedef struct message
{
	uint32_t length;
	char text[MESSAGE_TEXT_MAXLEN];
} Message;

typedef enum net_msg_type
{
	NET_MSG_PRESENCE = 0x00
} NetMessageType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEL_AXES              (3)
#define ACCEL_FSR               (3330)
#define ACCEL_SAMPLES           (ACCEL_AXES * ACCEL_SAMPLES_PER_AXIS)
#define ACCEL_SAMPLES_PER_AXIS  (10)
#define ACCEL_SENSITIVITY       (333)
#define ACCEL_X_BIAS            (1670)
#define ACCEL_Y_BIAS            (1655)
#define ACCEL_Z_BIAS            (1715)
#define ADC_MAX_VALUE           (4095)
#define BROAD_ERROR_DELAY       (1000)
#define BROAD_PAYLOAD_BUFLEN    (128)
#define BROAD_PORT              (12345)
#define BROAD_SEND_DELAY        (10000)
#define DEBUG_MEMORY            (1)
#define DEBUG_POLLING_DELAY     (10000)
#define ENDL                    "\r\n"
#define ETH_LINK_DOWN           (0)
#define ETH_LINK_PAUSED         (2)
#define ETH_LINK_POLLING_DELAY  (200)
#define ETH_LINK_STARTUP_DELAY  (2000)
#define ETH_LINK_UP             (1)
#define MAILQ_LENGTH            (0x08)
#define MAILQ_GET_TIMEOUT       (osWaitForever)
#define NODE_ID                 "nucleo-03"
#define NODE_IP                 "192.168.1.183"
#define SIG_BUTTON              (0x00000001)
#define SIG_LWIP                (0x00000002)
#define SIG_LINK_UP             (0x00000004)
#define SIG_PAUSE               (0x00000010)
#define SIG_RESUME              (0x00000100)
#define SYS_DEBOUNCE_MSEC       (100)
#define SYS_TASKS_RUNNING       (0x00)
#define SYS_TASKS_PAUSED        (0x01)
#define TID_SYS                 "<SYSTEM> "
#define TID_DEBUG               "<MEMORY>"
#define TID_HEART               "<HEART>  "
#define TID_LOGGR               "<LOGGER> "
#define TID_ACCEL               "<ACCEL>  "
#define TID_ETH                 "<ETHNET> "
#define TID_BROAD               "<BROAD>  "
#define UART_TIMEOUT            (100)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

osThreadId systemConductorHandle;
/* USER CODE BEGIN PV */
osThreadId heartbeatHandle;
osThreadId loggerHandle;
osThreadId accelerometerHandle;
osThreadId ethernetLinkMonitorHandle;
osThreadId networkBroadcastHandle;

#if DEBUG_MEMORY
osThreadId memoryAnalyserHandle;
#endif

osMailQId mailQueueHandle;

osMutexId uartMutexHandle;

osSemaphoreId accelerometerSemHandle;

uint8_t tasksState = SYS_TASKS_PAUSED;
uint8_t ethernetLinkState = ETH_LINK_DOWN;
uint16_t accelDmaBuffer[ACCEL_SAMPLES] = {0x00};
const char *net_presence_format =
		"{\n"
		"  \"type\": \"presence\",\n"
		"  \"id\": \"" NODE_ID "\",\n"
		"  \"ip\": \"" NODE_IP "\",\n"
		"  \"timestamp\": \"%s\"\n"
		"}";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
void SystemConductor(void const * argument);

/* USER CODE BEGIN PFP */
void Heartbeat(void const * argument);
void Logger(void const * argument);
void Accelerometer(void const * argument);
void EthernetLinkMonitor(void const * argument);
void NetworkBroadcast(void const * argument);
#if DEBUG_MEMORY
void MemoryAnalyser(void const * argument);
#endif

osStatus logMessage(const char *__restrict format, ...);
ssize_t formatNetMessage(NetMessageType type, char *messageBuffer, size_t bufferSize);
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
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreDef(accelerometerSem);
  accelerometerSemHandle = osSemaphoreCreate(osSemaphore(accelerometerSem), 1);
  if (tasksState == SYS_TASKS_PAUSED)
  {
  	// take the binary semaphore to block the task after initialisation.
  	osSemaphoreWait(accelerometerSemHandle, osWaitForever);
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMailQDef(mailQueue, MAILQ_LENGTH, Message);
  mailQueueHandle = osMailCreate(osMailQ(mailQueue), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of systemConductor */
  osThreadDef(systemConductor, SystemConductor, osPriorityHigh, 0, 256);
  systemConductorHandle = osThreadCreate(osThread(systemConductor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(logger, Logger, osPriorityNormal, 0, 128);
  loggerHandle = osThreadCreate(osThread(logger), NULL);

  osThreadDef(heartbeat, Heartbeat, osPriorityNormal, 0, 128);
  heartbeatHandle = osThreadCreate(osThread(heartbeat), NULL);

  osThreadDef(accelerometer, Accelerometer, osPriorityNormal, 0, 256);
  accelerometerHandle = osThreadCreate(osThread(accelerometer), NULL);

  osThreadDef(ethernetLinkMonitor, EthernetLinkMonitor, osPriorityNormal, 0, 128);
  ethernetLinkMonitorHandle = osThreadCreate(osThread(ethernetLinkMonitor), NULL);

  osThreadDef(networkBroadcast, NetworkBroadcast, osPriorityNormal, 0, 384);
  networkBroadcastHandle = osThreadCreate(osThread(networkBroadcast), NULL);

#if DEBUG_MEMORY
  osThreadDef(memoryAnalyser, MemoryAnalyser, osPriorityNormal, 0, 256);
  memoryAnalyserHandle = osThreadCreate(osThread(memoryAnalyser), NULL);
#endif
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 800-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 50000-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 80-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_PAUSE_Pin|LD_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_PAUSE_Pin LD_ERROR_Pin */
  GPIO_InitStruct.Pin = LD_PAUSE_Pin|LD_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Heartbeat(void const * argument)
{
	logMessage(TID_HEART "Heartbeat ready." ENDL);
	while (1)
	{
		osSignalWait(SIG_RESUME, osWaitForever);
		HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
		logMessage(TID_HEART "Task resumed." ENDL);

		osSignalWait(SIG_PAUSE, osWaitForever);
		HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		logMessage(TID_HEART "Task suspended." ENDL);
	}
}

osStatus logMessage(const char *__restrict format, ...)
{
	// osMailAlloc() is never blocking, millisec is discarded in v1 implementation
	Message *message = (Message *)osMailAlloc(mailQueueHandle, 0);
	if (message == NULL)
		return osErrorNoMemory;

	va_list args;

	va_start(args, format);
	vsnprintf(message->text, MESSAGE_TEXT_MAXLEN, format, args);
	va_end(args);

	message->length = strlen(message->text);

	return osMailPut(mailQueueHandle, message);
}

void Logger(void const * argument)
{
	logMessage(TID_LOGGR "Started Logger." ENDL);
	osEvent event = {0x00};
	Message *message;
	char timestamp[16];
	uint32_t ticks;
	div_t div_result;
	while (1)
	{
		event = osMailGet(mailQueueHandle, MAILQ_GET_TIMEOUT);
		if (osEventMail != event.status)
			continue;

		ticks = osKernelSysTick();
		div_result = div(ticks, osKernelSysTickFrequency);
		snprintf(timestamp, 16, "[%8d.%03d] ", div_result.quot, div_result.rem);
		message = (Message *)event.value.p;
		osMutexWait(uartMutexHandle, osWaitForever);
		HAL_UART_Transmit(&huart3, (uint8_t*)timestamp, 16, UART_TIMEOUT);
		HAL_UART_Transmit(&huart3, (uint8_t*)message->text, message->length, UART_TIMEOUT);
		osMutexRelease(uartMutexHandle);
		osMailFree(mailQueueHandle, event.value.p);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	osSemaphoreRelease(accelerometerSemHandle);
}

void Accelerometer(void const * argument)
{
	int32_t sumX = 0;
	int32_t sumY = 0;
	int32_t sumZ = 0;
	float accelX;
	float accelY;
	float accelZ;
	uint8_t bufferIndex = 0;

	logMessage(TID_ACCEL "Accelerometer ready." ENDL);
	while (1)
	{
		osSemaphoreWait(accelerometerSemHandle, osWaitForever);

		if (tasksState == SYS_TASKS_PAUSED)
		{
			HAL_TIM_Base_Stop_IT(&htim6);
			logMessage(TID_ACCEL "Task suspended." ENDL);
			continue;
		}

		// Only restart the timer once after resume.
		if (htim6.State == HAL_TIM_STATE_READY)
		{
			HAL_TIM_Base_Start_IT(&htim6);
			logMessage(TID_ACCEL "Task resumed." ENDL);
			// Wait until the next interrupt, DMA transfer may not be done after resume.
			osSemaphoreWait(accelerometerSemHandle, osWaitForever);
		}

		while (bufferIndex < 10)
		{
			sumX += accelDmaBuffer[ACCEL_AXES * bufferIndex];
			sumY += accelDmaBuffer[ACCEL_AXES * bufferIndex + 1];
			sumZ += accelDmaBuffer[ACCEL_AXES * bufferIndex + 2];
			bufferIndex++;
		}
		bufferIndex = 0;

		accelX = (((float)sumX * ACCEL_FSR ) / (ACCEL_SAMPLES_PER_AXIS * ADC_MAX_VALUE) - ACCEL_X_BIAS) / ACCEL_SENSITIVITY;
		accelY = (((float)sumY * ACCEL_FSR ) / (ACCEL_SAMPLES_PER_AXIS * ADC_MAX_VALUE) - ACCEL_Y_BIAS) / ACCEL_SENSITIVITY;
		accelZ = (((float)sumZ * ACCEL_FSR ) / (ACCEL_SAMPLES_PER_AXIS * ADC_MAX_VALUE) - ACCEL_Z_BIAS) / ACCEL_SENSITIVITY;
		logMessage(TID_ACCEL "x=%6.2f, y=%6.2f, z=%6.2f" ENDL, accelX, accelY, accelZ);
		sumX = sumY = sumZ = 0;
	}
}

void EthernetLinkMonitor(void const * argument)
{
	uint8_t currentLinkState = ETH_LINK_DOWN;

	osSignalWait(SIG_LWIP, osWaitForever);
	// Wait for LWIP to finalise its initialisation.
	osDelay(ETH_LINK_STARTUP_DELAY);

	logMessage(TID_ETH "Ethernet link monitor ready." ENDL);
  while (1)
  {
  	currentLinkState = netif_is_link_up(netif_default);

  	if (tasksState == SYS_TASKS_PAUSED)
  		currentLinkState = ETH_LINK_PAUSED;

  	if (currentLinkState == ethernetLinkState)
  	{
  		osDelay(ETH_LINK_POLLING_DELAY);
  		continue;
  	}

  	ethernetLinkState = currentLinkState;
  	switch (ethernetLinkState)
  	{
  	case ETH_LINK_DOWN:
  		logMessage(TID_ETH "Link is down." ENDL);
  		break;
  	case ETH_LINK_UP:
  		logMessage(TID_ETH "Link is up." ENDL);
  		osSignalSet(networkBroadcastHandle, SIG_LINK_UP);
  		break;
  	case ETH_LINK_PAUSED:
  		logMessage(TID_ETH "Network paused." ENDL);
  		osSignalWait(SIG_RESUME, osWaitForever);
  		logMessage(TID_ETH "Network resumed." ENDL);
  		break;
  	}
  }
}

ssize_t formatNetMessage(NetMessageType type, char *messageBuffer, size_t bufferSize)
{
	if (messageBuffer == NULL)
		return -1;

	// TODO: fetch timestamp with external RTC (step-3)
	const char *timestamp = "2025-12-26T12:31:42Z";

	switch (type)
	{
	case NET_MSG_PRESENCE:
		return snprintf(messageBuffer, bufferSize, net_presence_format, timestamp);
	default:
		logMessage("<ERROR>  Net message type invalid - 0x%08X" ENDL, type);
		return -1;
	}
}

void NetworkBroadcast(void const * argument)
{
  int32_t hBroadcast = -1;
  ssize_t bytesSent;
  uint32_t previousTimeStamp;
  struct sockaddr_in broadcast_addr = {0};
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(BROAD_PORT);
  broadcast_addr.sin_addr.s_addr = IPADDR_BROADCAST;

  char payload[BROAD_PAYLOAD_BUFLEN];
  ssize_t formattedLength = 0;

  logMessage(TID_BROAD "Network broadcast ready."ENDL);
  while (1)
  {
  	while (hBroadcast < 0)
  	{
  		if (ethernetLinkState != ETH_LINK_UP)
  			osSignalWait(SIG_LINK_UP, osWaitForever);

      hBroadcast = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (hBroadcast == -1)
      {
      	logMessage(TID_BROAD "Unable to create a socket - ERRNO:%d." ENDL, errno);
      	osDelay(BROAD_ERROR_DELAY);
      	continue;
      }

			int32_t enable = 0x01;
			if (setsockopt(hBroadcast, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)))
			{
				logMessage(TID_BROAD "Unable to configure broadcast - ERRNO:%d" ENDL, errno);
				close(hBroadcast);
				hBroadcast = -1;
				osDelay(BROAD_ERROR_DELAY);
			}
  	}

  	bytesSent = 1;
  	previousTimeStamp = osKernelSysTick();
  	while ((ethernetLinkState == ETH_LINK_UP) && (bytesSent > 0))
  	{
  		formattedLength = formatNetMessage(NET_MSG_PRESENCE, payload, BROAD_PAYLOAD_BUFLEN-1);
  		bytesSent = sendto(hBroadcast, payload, formattedLength, 0, (struct sockaddr *)&broadcast_addr, sizeof(struct sockaddr_in));

  		osDelayUntil(&previousTimeStamp, BROAD_SEND_DELAY);
  	}
  	close(hBroadcast);
  	hBroadcast = -1;
  }
}

#if DEBUG_MEMORY
void MemoryAnalyser(void const * argument)
{
	logMessage(TID_DEBUG "Memory analyser ready." ENDL);
  while (1)
  {
  	if (tasksState == SYS_TASKS_PAUSED)
  	{
  		osSignalWait(SIG_RESUME, osWaitForever);
  	}

  	logMessage(
  			"Stack high watermarks in 4-byte words: " ENDL
				"\tSYS: %3d" ENDL "\tLOG: %3d" ENDL "\tLED: %3d" ENDL
				"\tADC: %3d" ENDL "\tNET: %3d" ENDL "\tUDP: %3d" ENDL,
				(uint32_t)uxTaskGetStackHighWaterMark(systemConductorHandle),
				(uint32_t)uxTaskGetStackHighWaterMark(loggerHandle),
				(uint32_t)uxTaskGetStackHighWaterMark(heartbeatHandle),
				(uint32_t)uxTaskGetStackHighWaterMark(accelerometerHandle),
				(uint32_t)uxTaskGetStackHighWaterMark(ethernetLinkMonitorHandle),
				(uint32_t)uxTaskGetStackHighWaterMark(networkBroadcastHandle)
    );

    osDelay(DEBUG_POLLING_DELAY);
  }
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == USER_Btn_Pin)
	{
		osSignalSet(systemConductorHandle, SIG_BUTTON);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_SystemConductor */
/**
  * @brief  Function implementing the systemConductor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SystemConductor */
void SystemConductor(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  osSignalSet(ethernetLinkMonitorHandle, SIG_LWIP);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)accelDmaBuffer, ACCEL_SAMPLES);
  HAL_GPIO_WritePin(LD_PAUSE_GPIO_Port, LD_PAUSE_Pin, GPIO_PIN_SET);
  logMessage(TID_SYS "Started System Conductor." ENDL);
  /* Infinite loop */
  osEvent event;
  uint32_t lastTicks = 0;
  uint32_t currentTicks;
  for(;;)
  {
  	event = osSignalWait(SIG_BUTTON, osWaitForever);
  	if (event.status != osEventSignal)
  		continue;

  	currentTicks = osKernelSysTick();
  	if (lastTicks > currentTicks)
  		lastTicks =  0;
  	if ((currentTicks - lastTicks) < SYS_DEBOUNCE_MSEC)
  		continue;
  	lastTicks = currentTicks;

  	switch (tasksState)
  	{
  	case SYS_TASKS_RUNNING:
  		tasksState = SYS_TASKS_PAUSED;
  		osSignalSet(heartbeatHandle, SIG_PAUSE);
  		osSemaphoreRelease(accelerometerSemHandle);
			HAL_GPIO_WritePin(LD_PAUSE_GPIO_Port, LD_PAUSE_Pin, GPIO_PIN_SET);
			break;
  	case SYS_TASKS_PAUSED:
			tasksState = SYS_TASKS_RUNNING;
			HAL_GPIO_WritePin(LD_PAUSE_GPIO_Port, LD_PAUSE_Pin, GPIO_PIN_RESET);
			osSignalSet(heartbeatHandle, SIG_RESUME);
			osSemaphoreRelease(accelerometerSemHandle);
			osSignalSet(ethernetLinkMonitorHandle, SIG_RESUME);
#if DEBUG_MEMORY
			osSignalSet(memoryAnalyserHandle, SIG_RESUME);
#endif
			break;
  	default:
  		logMessage(TID_SYS "Invalid task state - 0x%02X." ENDL, tasksState);
  		Error_Handler();
  	}
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  HAL_GPIO_WritePin(LD_ERROR_GPIO_Port, LD_ERROR_Pin, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
