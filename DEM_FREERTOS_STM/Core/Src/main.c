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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "lis2dw12_reg.h"
#include "stdio.h"
#include "string.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNELS 4
#define SENSOR_BUS     hi2c1
#define BOOT_TIME      20 //ms
#define SOURCE_ADC   0
#define SOURCE_ACCEL 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
	float roll;
	float pitch;
} Angles;
stmdev_ctx_t dev_ctx;
Angles angle;

typedef struct {
	uint16_t source;
	uint16_t val1;
	uint16_t val2;
} SystemMessage_t;
// Accelerometer Data and Status
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
volatile uint16_t ADC_VAL[ADC_CHANNELS];
float temperature;
float light;
uint8_t rgb_r, rgb_g, rgb_b;
uint16_t last_temp = 0, last_light = 0;
uint16_t last_pitch = 0, last_roll = 0;
uint8_t mutex_flag = 0, queue_flag = 0;
uint8_t error_queue1 = 0, error_queue = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

osThreadId TaskADCHandle;
osThreadId TaskLis2dw12Handle;
osThreadId TaskUARTHandle;
osThreadId TaskSsd1306Handle;
osTimerId KnightTimerHandle;
osMutexId i2cMutexHandle;
osSemaphoreId adcSemHandle;
/* USER CODE BEGIN PV */
QueueHandle_t dataQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void vTaskADC(void const * argument);
void vTaskLis2dw12(void const * argument);
void vTaskUART(void const * argument);
void vTaskSsd1306(void const * argument);
void vTimerCallback_KnightRider(void const * argument);

/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);

float Termistor(uint32_t analogValue) {
	float temperature;
	float adcval = (float) (4096 - analogValue);

	if ((4095.0f - adcval) == 0)
		return 0.0f;

	float resistance = (adcval * 10000.0f) / (4096.0f - adcval);
	temperature = logf(resistance);

	temperature = 1.0f
			/ (0.001129148f
					+ (0.000234125f
							+ (0.0000000876741f * temperature * temperature))
							* temperature);
	temperature = temperature - 273.15f;

	return temperature;
}

void TrimpotToRGB(uint32_t pot1, uint32_t pot2, uint8_t *rgb_r, uint8_t *rgb_g,
		uint8_t *rgb_b) {
	float h = ((float) pot1 - 2150.0f) * (360.0f / 1945.0f);
	float s = ((float) pot2 - 2100.0f) * (1.0f / 1995.0f);
	float v = 1.0f;

	float c = v * s;
	float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
	float m = v - c;

	float r1, g1, b1;

	if (h < 60) {
		r1 = c;
		g1 = x;
		b1 = 0;
	} else if (h < 120) {
		r1 = x;
		g1 = c;
		b1 = 0;
	} else if (h < 180) {
		r1 = 0;
		g1 = c;
		b1 = x;
	} else if (h < 240) {
		r1 = 0;
		g1 = x;
		b1 = c;
	} else if (h < 300) {
		r1 = x;
		g1 = 0;
		b1 = c;
	} else {
		r1 = c;
		g1 = 0;
		b1 = x;
	}

	*rgb_r = (uint8_t) ((r1 + m) * 255.0f);
	*rgb_g = (uint8_t) ((g1 + m) * 255.0f);
	*rgb_b = (uint8_t) ((b1 + m) * 255.0f);
}
/**
 * @brief Initializes the LIS2DW12 accelerometer sensor.
 */
static void LIS2DW12_Init(void) {
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;

	lis2dw12_device_id_get(&dev_ctx, &whoamI);
	lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lis2dw12_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
	/* Configure filtering chain
	 * Accelerometer - filter path / bandwidth
	 */
	lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
	lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
	/* Configure power mode */
	lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
	/* Set Output Data Rate */
	lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);
}

/**
 * @brief Reads the latest data from the accelerometer if available.
 */
static void Read_Accelerometer(void) {
	uint8_t reg;
	/* Read output only if new value is available */
	lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read acceleration data */
		memset(data_raw_acceleration, 0, sizeof(data_raw_acceleration));
		lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		acceleration_mg[0] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[2]);
	}
}

/**
 * @brief Calculates roll and pitch angles from accelerometer data.
 * @param x: X-axis acceleration
 * @param y: Y-axis acceleration
 * @param z: Z-axis acceleration
 * @retval Angles struct containing roll and pitch
 */
static Angles Calculate_Angles(float x, float y, float z) {
	Angles angles_calc;

	const float PI_F = 3.14159265f;

	angles_calc.roll = atan2f(y, z) * 180.0f / PI_F;
	angles_calc.pitch = atan2f(-x, sqrtf(y * y + z * z)) * 180.0f / PI_F;

	if (angles_calc.roll < 0) {
		angles_calc.roll += 360.0f;
	}
	if (angles_calc.pitch < 0) {
		angles_calc.pitch += 360.0f;
	}

	return angles_calc;
}

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	ssd1306_TestDrawBitmap();
	ssd1306_Init();
	ssd1306_SetCursor(15, 2);
	ssd1306_WriteString("STARTING", Font_11x18, White);
	ssd1306_SetCursor(14, 27);
	ssd1306_WriteString("FREERTOS", Font_11x18, White);
	ssd1306_SetCursor(45, 46);
	ssd1306_WriteString("STM", Font_11x18, White);
	ssd1306_UpdateScreen();
	HAL_Delay(1000);
	ssd1306_Fill(Black);

	LIS2DW12_Init();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of i2cMutex */
  osMutexDef(i2cMutex);
  i2cMutexHandle = osMutexCreate(osMutex(i2cMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of adcSem */
  osSemaphoreDef(adcSem);
  adcSemHandle = osSemaphoreCreate(osSemaphore(adcSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of KnightTimer */
  osTimerDef(KnightTimer, vTimerCallback_KnightRider);
  KnightTimerHandle = osTimerCreate(osTimer(KnightTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	osTimerStart(KnightTimerHandle, 100);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	dataQueueHandle = xQueueCreate(5, sizeof(SystemMessage_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskADC */
  osThreadDef(TaskADC, vTaskADC, osPriorityHigh, 0, 96);
  TaskADCHandle = osThreadCreate(osThread(TaskADC), NULL);

  /* definition and creation of TaskLis2dw12 */
  osThreadDef(TaskLis2dw12, vTaskLis2dw12, osPriorityAboveNormal, 0, 96);
  TaskLis2dw12Handle = osThreadCreate(osThread(TaskLis2dw12), NULL);

  /* definition and creation of TaskUART */
  osThreadDef(TaskUART, vTaskUART, osPriorityBelowNormal, 0, 96);
  TaskUARTHandle = osThreadCreate(osThread(TaskUART), NULL);

  /* definition and creation of TaskSsd1306 */
  osThreadDef(TaskSsd1306, vTaskSsd1306, osPriorityBelowNormal, 0, 128);
  TaskSsd1306Handle = osThreadCreate(osThread(TaskSsd1306), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	osSemaphoreRelease(adcSemHandle);
}
/**
 * @brief Write generic device register (platform dependent)
 *
 * @param handle I2C handle
 * @param reg register to write
 * @param bufp pointer to data to write
 * @param len number of bytes to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/**
 * @brief Read generic device register (platform dependent)
 *
 * @param handle I2C handle
 * @param reg register to read
 * @param bufp pointer to buffer that store the data read
 * @param len number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}
void Uart_Append_Int(char *buff, char *label, int value) {
	char temp[10];
	strcat(buff, label);
	itoa(value, temp, 10);
	strcat(buff, temp);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_vTaskADC */
/**
 * @brief  Function implementing the TaskADC thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTaskADC */
void vTaskADC(void const * argument)
{
  /* USER CODE BEGIN 5 */
	SystemMessage_t *pMsg;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_VAL, ADC_CHANNELS);
	/* Infinite loop */
	for (;;) {
		if (osSemaphoreWait(adcSemHandle, osWaitForever) == osOK) {

			pMsg = pvPortMalloc(sizeof(SystemMessage_t));

			TrimpotToRGB(ADC_VAL[0], ADC_VAL[1], &rgb_r, &rgb_g, &rgb_b);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
					(uint32_t )rgb_r * 257);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
					(uint32_t )rgb_g * 257);

			__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,
					(uint32_t )rgb_b * 257);

			/* --- Sıcaklık hesapla (IN2) --- */
			temperature = Termistor(ADC_VAL[2]);

			light = (float) ADC_VAL[3] * (100.0f / 4095.0f);

			if (temperature < 0)
				temperature = 0;

			pMsg->source = SOURCE_ADC;
			pMsg->val1 = (uint16_t) temperature;
			pMsg->val2 = (uint16_t) light;

			if (xQueueSend(dataQueueHandle, &pMsg, 0) != pdTRUE) {
				vPortFree(pMsg);
				error_queue = 1;
			} else {
				error_queue = 0;
			}
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_VAL, ADC_CHANNELS);

			osDelay(100);
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTaskLis2dw12 */
/**
 * @brief Function implementing the TaskLis2dw12 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTaskLis2dw12 */
void vTaskLis2dw12(void const * argument)
{
  /* USER CODE BEGIN vTaskLis2dw12 */
	SystemMessage_t *pMsg;
	pMsg = pvPortMalloc(sizeof(SystemMessage_t));
	/* Infinite loop */
	for (;;) {
		if (osMutexWait(i2cMutexHandle, osWaitForever) == osOK) {
			Read_Accelerometer();
			osMutexRelease(i2cMutexHandle);
		}
		angle = Calculate_Angles(acceleration_mg[1], acceleration_mg[0],
				acceleration_mg[2]);

		pMsg->source = SOURCE_ACCEL;
		pMsg->val1 = (uint16_t) angle.pitch;
		pMsg->val2 = (uint16_t) angle.roll;

		if (xQueueSend(dataQueueHandle, &pMsg, 0) != pdTRUE) {
			vPortFree(pMsg); // Gönderilemezse hafızayı temizle
			error_queue1 = 1;
		} else {
			error_queue1 = 0;
		}
		osDelay(100);
	}
  /* USER CODE END vTaskLis2dw12 */
}

/* USER CODE BEGIN Header_vTaskUART */
/**
 * @brief Function implementing the TaskUART thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTaskUART */
void vTaskUART(void const * argument)
{
  /* USER CODE BEGIN vTaskUART */
	static char uart_buff[32];

	/* Infinite loop */
	for (;;) {

		ulTaskNotifyTake(pdTRUE, osWaitForever);

		uart_buff[0] = '\0';

		Uart_Append_Int(uart_buff, "P:", last_pitch);
		Uart_Append_Int(uart_buff, "R:", last_roll);
		Uart_Append_Int(uart_buff, "T:", last_temp);
		Uart_Append_Int(uart_buff, "L:", last_light);

		strcat(uart_buff, "\r\n");

		HAL_UART_Transmit(&huart2, (uint8_t*) uart_buff, strlen(uart_buff),
				100);
	}
  /* USER CODE END vTaskUART */
}

/* USER CODE BEGIN Header_vTaskSsd1306 */
/**
 * @brief Function implementing the TaskSsd1306 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTaskSsd1306 */
void vTaskSsd1306(void const * argument)
{
  /* USER CODE BEGIN vTaskSsd1306 */
	SystemMessage_t *receivedMsg;
	char line1_buff[32];
	char line2_buff[32];
	char line3_buff[32];
	char temp_str[10];
	/* Infinite loop */
	for (;;) {

		if (xQueueReceive(dataQueueHandle, &receivedMsg,
		osWaitForever) == pdTRUE) {
			queue_flag = 1;

			if (receivedMsg->source == SOURCE_ADC) {
				last_temp = receivedMsg->val1;
				last_light = receivedMsg->val2;
			} else if (receivedMsg->source == SOURCE_ACCEL) {
				last_pitch = receivedMsg->val1;
				last_roll = receivedMsg->val2;
			}
			vPortFree(receivedMsg);

			xTaskNotifyGive(TaskUARTHandle);

			strcpy(line1_buff, "PITCH:");
			if (last_pitch < 10)
				strcat(line1_buff, "  ");
			else if (last_pitch < 100)
				strcat(line1_buff, "  ");
			itoa(last_pitch, temp_str, 10);
			strcat(line1_buff, temp_str);
			strcat(line1_buff, " ");

			strcpy(line2_buff, "ROLL:");
			if (last_roll < 10)
				strcat(line2_buff, "  ");
			else if (last_roll < 100)
				strcat(line2_buff, "  ");
			itoa(last_roll, temp_str, 10);
			strcat(line2_buff, temp_str);
			strcat(line2_buff, " ");

			strcpy(line3_buff, "T:");
			itoa(last_temp, temp_str, 10);
			strcat(line3_buff, temp_str);

			strcat(line3_buff, "C L:");
			if (last_light < 10) {
				strcat(line3_buff, " ");
			} else if (last_light < 100) {
				strcat(line3_buff, "");
			}
			itoa(last_light, temp_str, 10);
			strcat(line3_buff, temp_str);
			strcat(line3_buff, "%");

			if (osMutexWait(i2cMutexHandle, osWaitForever) == osOK) {
				ssd1306_SetCursor(0, 1);
				ssd1306_WriteString(line1_buff, Font_11x18, White);
				ssd1306_SetCursor(0, 23);
				ssd1306_WriteString(line2_buff, Font_11x18, White);
				ssd1306_SetCursor(0, 45);
				ssd1306_WriteString(line3_buff, Font_11x18, White);
				ssd1306_UpdateScreen();
				osMutexRelease(i2cMutexHandle);
			}
		}

	}
  /* USER CODE END vTaskSsd1306 */
}

/* vTimerCallback_KnightRider function */
void vTimerCallback_KnightRider(void const * argument)
{
  /* USER CODE BEGIN vTimerCallback_KnightRider */
	static uint8_t led_state = 0b00001;
	static uint8_t direction = 1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,
			(led_state & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,
			(led_state & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,
			(led_state & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,
			(led_state & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,
			(led_state & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	if (direction) {
		led_state <<= 1;
		if (led_state & 0x10)
			direction = 0;
	} else {
		led_state >>= 1;
		if (led_state & 0x01)
			direction = 1;
	}
  /* USER CODE END vTimerCallback_KnightRider */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
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
	while (1) {
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
