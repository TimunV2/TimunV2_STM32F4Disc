/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"//
#include "string.h"
#include "math.h"
#include "pid.h"
#include "ms58xx.h"
#include "16Bitmanipulator.h"
#include "WS2812LED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Timer Define PWM Thruster
#define send_H_FR TIM4->CCR1
#define send_H_FL TIM4->CCR4
#define send_H_BR TIM4->CCR2
#define send_H_BL TIM4->CCR3
#define send_V_FR TIM5->CCR1
#define send_V_FL TIM5->CCR4
#define send_V_BR TIM5->CCR3
#define send_V_BL TIM5->CCR2

//Timer Define PWM Lumen
#define send_Lumen TIM9->CCR1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
DMA_HandleTypeDef hdma_tim3_ch2;

UART_HandleTypeDef huart2;

osThreadId cdcSerialTaskHandle;
osThreadId movementTaskHandle;
osThreadId getPressureTaskHandle;
osThreadId stroboTaskHandle;
osThreadId otherTaskHandle;
osThreadId pidTaskHandle;
/* USER CODE BEGIN PV */

//PC Serial Variable
	//Receive
uint8_t serialBuffer[64];
int vel_linear[3], vel_angular[3]; // x , y, z
int throtle_scale; // 0 - 500
int set_point[4]; // yaw, pitch, roll, depth
int lumen_power; // 0 - 100
int movement_mode, operation_mode;
int arm_hw, arm_sw; // bool
int imu_reset;
int c_yaw[3], c_pitch[3], c_roll[3], c_depth[3]; //pid constans [kp, ki, kd]

	//Transmit
uint8_t transmitBuffer[30];
int16_t Message_IMU[3];
int16_t Message_inPressure, Message_depth;
int16_t Message_batt1, Message_batt2;
int16_t Message_H_FR, Message_H_FL, Message_H_BR, Message_H_BL;
int16_t Message_V_FR, Message_V_FL, Message_V_BR, Message_V_BL;
//int16_t Message_ranges, Message_confidence;

	//Failsafe
uint32_t lastReceiveTime = 0;
uint32_t lastSendTime = 0;
int cdcHeartBeat = 1;
int cdcTimeOut = 3000;

//IMU Variable
uint8_t atmega_buffer[13];
uint8_t status_rx_atmega;
int32_t raw_bno055_yaw;
int32_t raw_bno055_pitch;
int32_t raw_bno055_roll;

float bno055_yaw = 0.00;
float bno055_pitch = 0.00;
float bno055_roll = 0.00;

int IMU_resetStatus_new, IMU_resetStatus_old;
char* IMUStatus = "Running..";

//Pressure Sensor Variable
HAL_StatusTypeDef MS5803_status, MS5837_status;
uint8_t MS5803_address = 0x77;
uint8_t MS5837_address = 0x76;

uint16_t MS5803_coefficient[6], MS5837_coefficient[6];
float MS5803_pressure, MS5803_temperature;
float MS5837_pressure, MS5837_temperature;

//Echosounder Variable
//uint8_t sonar_buffer[16];
//uint8_t status_rx_sonar;
//uint16_t scan_ranges = 99;
//uint16_t confidence = 98;

//Compute Variable
	//PID Control
float pressureInside; //Bar
float depthValue; //Centimeters
float imu_yaw, imu_pitch, imu_roll;

PID_t yaw;
PID_t pitch;
PID_t roll;
PID_t depth;

float setpoint_yaw;
float setpoint_pitch;
float setpoint_roll;
float setpoint_depth;

//ESC - Thruster
int pwmCenter = 1500, pwmMax = 2000, pwmMin = 1000;
int HorizontalMax = 0, VerticalMax = 0;

int H_FR, H_FL, H_BR, H_BL;
int V_FR, V_FL, V_BR, V_BL;

int H_FRoffset = -20, H_FLoffset = -20, H_BRoffset = -20, H_BLoffset = -20;
int V_FRoffset = -20, V_FLoffset = -20, V_BRoffset = -20, V_BLoffset = -20;

//Voltmeter Variable
uint32_t adc1_value[2];
int batt1_adc, batt2_adc;
int batt1adc_min = 3243, batt1adc_max = 3639;
int batt2adc_min = 3243, batt2adc_max = 3639;
float batt1_volt, batt2_volt;
float batt1_min = 22.2, batt1_max = 25.2;
float batt2_min = 22.2, batt2_max = 25.2;

//Lumen
int lumen_pwm;

float convertADCtoVoltage(uint16_t adc_value, uint16_t min_adc, uint16_t max_adc, float min_voltage, float max_voltage);

//Other Variable
	//WS2812LED
int datasentflag = 0;
uint8_t led_status = 0; //0 >> Normal || 1 >> Some Other
	//Leaks Sensor
int leaks = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartCdcSerialTask(void const * argument);
void StartMovementTask(void const * argument);
void StartGetPressureTask(void const * argument);
void StartStroboTask(void const * argument);
void StartOtherTask(void const * argument);
void StartPidTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	HAL_UART_Receive_IT(&huart2, atmega_buffer, 1);
	if (huart->Instance == USART2) {
		if (atmega_buffer[0] == 0xAA && status_rx_atmega == 0) {
			status_rx_atmega = 1;
			HAL_UART_Receive_IT(&huart2, atmega_buffer + 1, 1);
		} else if (atmega_buffer[1] == 0xBB && status_rx_atmega == 1){
			status_rx_atmega = 2;
			HAL_UART_Receive_IT(&huart2, atmega_buffer + 2, 11);
		} else if ( status_rx_atmega == 2) {
			raw_bno055_yaw = (atmega_buffer[4] << 16) | (atmega_buffer[3] << 8) | atmega_buffer[2];
			raw_bno055_pitch = (atmega_buffer[7] << 16) | (atmega_buffer[6] << 8) | atmega_buffer[5];
			raw_bno055_roll = (atmega_buffer[10] << 16) | (atmega_buffer[9] << 8) | atmega_buffer[8];

			if (raw_bno055_yaw > 8388607) {
				bno055_yaw = ((raw_bno055_yaw - 16777216) / 100.0);
			} else {
				bno055_yaw = (raw_bno055_yaw / 100.0);
			}

			if (raw_bno055_pitch > 8388607) {
				bno055_pitch = ((raw_bno055_pitch - 16777216) / 100.0);
			} else {
				bno055_pitch = (raw_bno055_pitch / 100.0);
			}

			if (raw_bno055_roll > 8388607) {
				bno055_roll = ((raw_bno055_roll - 16777216) / 100.0);
			} else {
				bno055_roll = (raw_bno055_roll / 100.0);
			}

			status_rx_atmega = 0;
			HAL_UART_Receive_IT(&huart2, atmega_buffer, 1);
		} else {
			status_rx_atmega = 0;
			HAL_UART_Receive_IT(&huart2, atmega_buffer, 1);
		}
	}
}

float convertADCtoVoltage(uint16_t adc_value, uint16_t min_adc, uint16_t max_adc, float min_voltage, float max_voltage) {
    float a = (max_voltage - min_voltage) / (float)(max_adc - min_adc);
    float b = min_voltage - a * min_adc;
    return a * adc_value + b;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	datasentflag=1;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //ESC PWM Timer Init
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  //ADC
  HAL_ADC_Start_DMA(&hadc1, adc1_value, 2);

  /* USER CODE END 2 */

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
  /* definition and creation of cdcSerialTask */
  osThreadDef(cdcSerialTask, StartCdcSerialTask, osPriorityNormal, 0, 128);
  cdcSerialTaskHandle = osThreadCreate(osThread(cdcSerialTask), NULL);

  /* definition and creation of movementTask */
  osThreadDef(movementTask, StartMovementTask, osPriorityNormal, 0, 128);
  movementTaskHandle = osThreadCreate(osThread(movementTask), NULL);

  /* definition and creation of getPressureTask */
  osThreadDef(getPressureTask, StartGetPressureTask, osPriorityHigh, 0, 128);
  getPressureTaskHandle = osThreadCreate(osThread(getPressureTask), NULL);

  /* definition and creation of stroboTask */
  osThreadDef(stroboTask, StartStroboTask, osPriorityBelowNormal, 0, 128);
  stroboTaskHandle = osThreadCreate(osThread(stroboTask), NULL);

  /* definition and creation of otherTask */
  osThreadDef(otherTask, StartOtherTask, osPriorityIdle, 0, 128);
  otherTaskHandle = osThreadCreate(osThread(otherTask), NULL);

  /* definition and creation of pidTask */
  osThreadDef(pidTask, StartPidTask, osPriorityHigh, 0, 128);
  pidTaskHandle = osThreadCreate(osThread(pidTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 105-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, atmega_buffer, 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCdcSerialTask */
/**
  * @brief  Function implementing the cdcSerialTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCdcSerialTask */
void StartCdcSerialTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  //Receive
		int i = 0;
		vel_linear[0] = bit16ToInt(i, serialBuffer); //linear x
		vel_linear[1] = bit16ToInt(i + 2, serialBuffer); //linear y
		vel_linear[2] = bit16ToInt(i + 4, serialBuffer); //linear z
		vel_angular[0] = bit16ToInt(i + 6, serialBuffer); //angular z
		vel_angular[1] = bit16ToInt(i + 8, serialBuffer); //angular y
		vel_angular[2] = bit16ToInt(i + 10, serialBuffer); //angular z

		i = 12;
		throtle_scale = bit16ToInt(i, serialBuffer);

		i = 14;
		set_point[0] = bit16ToInt(i, serialBuffer);
		set_point[1] = bit16ToInt(i + 2, serialBuffer);
		set_point[2] = bit16ToInt(i + 4, serialBuffer);
		set_point[3] = bit16ToInt(i + 6, serialBuffer);

		i = 22;
		c_yaw[0] = bit16ToInt(i, serialBuffer);
		c_yaw[1] = bit16ToInt(i + 2, serialBuffer);
		c_yaw[2] = bit16ToInt(i + 4, serialBuffer);
		c_pitch[0] = bit16ToInt(i + 6, serialBuffer);
		c_pitch[1] = bit16ToInt(i + 8, serialBuffer);
		c_pitch[2] = bit16ToInt(i + 10, serialBuffer);
		c_roll[0] = bit16ToInt(i + 12, serialBuffer);
		c_roll[1] = bit16ToInt(i + 14, serialBuffer);
		c_roll[2] = bit16ToInt(i + 16, serialBuffer);
		c_depth[0] = bit16ToInt(i + 18, serialBuffer);
		c_depth[1] = bit16ToInt(i + 20, serialBuffer);
		c_depth[2] = bit16ToInt(i + 22, serialBuffer);

		i = 46;
		lumen_power = bit16ToInt(i, serialBuffer);

		i = 48;
		movement_mode = serialBuffer[i];
		operation_mode = serialBuffer[i + 1];
		arm_hw = serialBuffer[i + 2];
		arm_sw = serialBuffer[i + 3];
		imu_reset = serialBuffer[i + 4];

		if (HAL_GetTick() - lastReceiveTime > cdcTimeOut) {
			cdcHeartBeat = 0;
		} else {
			cdcHeartBeat = 1;
		}

		if (cdcHeartBeat == 0) {
			vel_linear[0] = 0; vel_linear[1] = 0; vel_linear[2] = 0;
			vel_angular[0] = 0; vel_angular[1] = 0; vel_angular[2] = 0;
			set_point[0] = 0; set_point[1] = 0; set_point[2] = 0; set_point[3] = 0;
			movement_mode = 0;
			operation_mode = 0;
		}

	//Transmit
		Message_IMU[0] = bno055_yaw * 10;
		Message_IMU[1] = bno055_pitch * 10;
		Message_IMU[2] = bno055_roll * 10;

		Message_depth = depthValue * 100;
		Message_inPressure = pressureInside * 100;

		Message_batt1 = batt1_volt * 100;
		Message_batt2 = batt2_volt * 100;

		int16_t Message_Values[] = { Message_IMU[0],Message_IMU[1], Message_IMU[2], Message_depth, Message_inPressure,
				Message_batt1, Message_batt2, Message_H_FR, Message_H_FL, Message_H_BR, Message_H_BL, Message_V_FR, Message_V_FL, Message_V_BR, Message_V_BL};


		merge16(Message_Values, transmitBuffer,
				sizeof(Message_Values) / sizeof(Message_Values[0]));

		CDC_Transmit_FS(transmitBuffer, sizeof(transmitBuffer));

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMovementTask */
/**
* @brief Function implementing the movementTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMovementTask */
void StartMovementTask(void const * argument)
{
  /* USER CODE BEGIN StartMovementTask */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
  /* Infinite loop */
  for(;;)
  {
		//Movement Calculation
		if (movement_mode == 0) { // Fully Manual
			//Horizontal Thruster  Forwaed  |||   Lateral   |||    Yaw
			H_FR = +vel_linear[0] - vel_linear[1] + vel_angular[2];
			H_FL = -vel_linear[0] - vel_linear[1] - vel_angular[2];
			H_BR = +vel_linear[0] + vel_linear[1] - vel_angular[2];
			H_BL = -vel_linear[0] + vel_linear[1] + vel_angular[2];

			//Vertical Thruster    Up/Down  |||   Pitch     |||   Roll
			V_FR = vel_linear[2] - vel_angular[0] - vel_angular[1];
			V_FL = vel_linear[2] - vel_angular[0] + vel_angular[1];
			V_BR = vel_linear[2] + vel_angular[0] + vel_angular[1];
			V_BL = vel_linear[2] + vel_angular[0] - vel_angular[1];
		}

		else if (movement_mode == 1) { // Stabilize
			//Horizontal Thruster  Forwaed  |||   Lateral   |||    Yaw
			H_FR = +vel_linear[0] - vel_linear[1] + yaw.output;
			H_FL = -vel_linear[0] - vel_linear[1] - yaw.output;
			H_BR = +vel_linear[0] + vel_linear[1] - yaw.output;
			H_BL = -vel_linear[0] + vel_linear[1] + yaw.output;

			//Vertical Thruster    Up/Down  |||   Pitch     |||   Roll
			V_FR = vel_linear[2] - pitch.output + roll.output;
			V_FL = vel_linear[2] - pitch.output - roll.output;
			V_BR = vel_linear[2] + pitch.output - roll.output;
			V_BL = vel_linear[2] + pitch.output + roll.output;
		}

		else if (movement_mode == 2) { // Depthhold
			//Horizontal Thruster  Forwaed  |||   Lateral   |||    Yaw
			H_FR = +vel_linear[0] - vel_linear[1] + vel_angular[2];
			H_FL = -vel_linear[0] - vel_linear[1] - vel_angular[2];
			H_BR = +vel_linear[0] + vel_linear[1] - vel_angular[2];
			H_BL = -vel_linear[0] + vel_linear[1] + vel_angular[2];

			//Vertical Thruster    Up/Down  |||   Pitch     |||   Roll
			V_FR = -depth.output - vel_angular[0] - vel_angular[1];
			V_FL = -depth.output - vel_angular[0] + vel_angular[1];
			V_BR = -depth.output + vel_angular[0] + vel_angular[1];
			V_BL = -depth.output + vel_angular[0] - vel_angular[1];
		}

		else if (movement_mode == 3) { // Fully Assisted
			//Horizontal Thruster  Forwaed  |||   Lateral   |||    Yaw
			H_FR = +vel_linear[0] - vel_linear[1] + yaw.output;
			H_FL = -vel_linear[0] - vel_linear[1] - yaw.output;
			H_BR = +vel_linear[0] + vel_linear[1] - yaw.output;
			H_BL = -vel_linear[0] + vel_linear[1] + yaw.output;

			//Vertical Thruster    Up/Down  |||   Pitch     |||   Roll
			V_FR = -depth.output - pitch.output + roll.output;
			V_FL = -depth.output - pitch.output - roll.output;
			V_BR = -depth.output + pitch.output - roll.output;
			V_BL = -depth.output + pitch.output + roll.output;
		}

		HorizontalMax = fmax(fmax(abs(H_FR), abs(H_FL)),fmax(abs(H_BR), abs(H_BL)));
		VerticalMax = fmax(fmax(abs(V_FR), abs(V_FL)),fmax(abs(V_BR), abs(V_BL)));

		if (HorizontalMax > 500) {
			H_FR = H_FR / HorizontalMax;
			H_FL = H_FL / HorizontalMax;
			H_BR = H_BR / HorizontalMax;
			H_BL = H_BL / HorizontalMax;
		}

		if (VerticalMax > 500) {
			V_FR = V_FR / VerticalMax;
			V_FL = V_FL / VerticalMax;
			V_BR = V_BR / VerticalMax;
			V_BL = V_BL / VerticalMax;
		}

		H_FR = H_FR + pwmCenter + H_FRoffset;
		H_FL = H_FL + pwmCenter + H_FLoffset;
		H_BR = H_BR + pwmCenter + H_FRoffset;
		H_BL = H_BL + pwmCenter + H_BLoffset;
		V_FR = V_FR + pwmCenter + V_FRoffset;
		V_FL = V_FL + pwmCenter + V_FLoffset;
		V_BR = V_BR + pwmCenter + V_BRoffset;
		V_BL = V_BL + pwmCenter + V_BLoffset;

		//Send to ESC
		send_H_FR = H_FR;
		send_H_FL = H_FL;
		send_H_BR = H_BR;
		send_H_BL = H_BL;
		send_V_FR = V_FR;
		send_V_FL = V_FL;
		send_V_BR = V_BR;
		send_V_BL = V_BL;

		//Lumen
		lumen_pwm = map(lumen_power, 0, 100, 1000, 2000);
		send_Lumen = lumen_pwm;

		//Capture last value to send back to ROS
		Message_H_FR = H_FR;
		Message_H_FL = H_FL;
		Message_H_BR = H_BR;
		Message_H_BL = H_BL;
		Message_V_FR = V_FR;
		Message_V_FL = V_FL;
		Message_V_BR = V_BR;
		Message_V_BL = V_BL;

		//Reset IMU
		IMU_resetStatus_new = imu_reset;
		if (IMU_resetStatus_new == 1 && IMU_resetStatus_old == 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
			IMUStatus = "Resetting..";
			led_status = 1;
			osDelay(200);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
			IMUStatus = "Running..";
		}
		IMU_resetStatus_old = IMU_resetStatus_new;

    osDelay(1);
  }
  /* USER CODE END StartMovementTask */
}

/* USER CODE BEGIN Header_StartGetPressureTask */
/**
* @brief Function implementing the getPressureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetPressureTask */
void StartGetPressureTask(void const * argument)
{
  /* USER CODE BEGIN StartGetPressureTask */

	// check sensor status
	MS5803_status = MS58xx_reset(&hi2c1, MS5803_address);
	MS5837_status = MS58xx_reset(&hi2c1, MS5837_address);

	// Initialize MS5803 sensor if attached
	if (MS5803_status == HAL_OK) {
		for (int i = 0; i < 6; i++) {
			MS58xx_coeff(&hi2c1, &MS5803_coefficient[i], MS5803_address, i + 1);
		}
	}

	// Initialize MS5837 sensor if attached
	if (MS5837_status == HAL_OK) {
		for (int i = 0; i < 6; i++) {
			MS58xx_coeff(&hi2c1, &MS5837_coefficient[i], MS5837_address, i + 1);
		}
	}

  /* Infinite loop */
  for(;;)
  {
		if (MS5803_status  == HAL_OK ) {
			MS58xx_get_values(&hi2c1, ADC_4096, MS5803_coefficient,
					&MS5803_temperature, &MS5803_pressure, MS5803_address, MS5803);
		}

		if (MS5837_status  == HAL_OK ) {
			MS58xx_get_values(&hi2c1, ADC_4096, MS5837_coefficient,
					&MS5837_temperature, &MS5837_pressure, MS5837_address,MS5837);
		}

		//pressure(Bar)
		pressureInside = MS5803_pressure / 1000;
		//depth(cm) = pressure(mbar) / (water density(kg/m3) * gravity(m/s2) * 10)
		depthValue = round(MS5837_pressure / (9.81 * 100.0)) / 100.0;;

    osDelay(1);
  }
  /* USER CODE END StartGetPressureTask */
}

/* USER CODE BEGIN Header_StartStroboTask */
/**
* @brief Function implementing the stroboTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStroboTask */
void StartStroboTask(void const * argument)
{
  /* USER CODE BEGIN StartStroboTask */
  /* Infinite loop */
  for(;;)
  {
		if (led_status == 0) {
			if (movement_mode == 0) {
				for (int i = 1; i <= 5; i++) {
					// Clear all LEDs
					for (int j = 0; j < 11; j++) {
						Set_LED(j, 0, 0, 0); // Turn off LED
					}
					for (int j = 1; j <= i + 1; j++) {
						Set_LED(j, 0, 0, 255);
						Set_LED(j - j, 0, 0, 255);
					}
					for (int j = 10; j >= 11 - i; j--) {
						Set_LED(j, 0, 0, 255);
					}
					WS2812_Send(&htim3, TIM_CHANNEL_2, 105);
					osDelay(120); // Adjust as needed
				}
			} else if (movement_mode == 1) {
				for (int i = 1; i <= 5; i++) {
					// Clear all LEDs
					for (int j = 0; j < 11; j++) {
						Set_LED(j, 0, 0, 0); // Turn off LED
					}
					for (int j = 1; j <= i + 1; j++) {
						Set_LED(j, 255, 0, 255);
						Set_LED(j - j, 255, 0, 255);
					}
					for (int j = 10; j >= 11 - i; j--) {
						Set_LED(j, 255, 0, 255);
					}
					WS2812_Send(&htim3, TIM_CHANNEL_2, 105);
					osDelay(120); // Adjust as needed
				}
			} else if (movement_mode == 2) {
				for (int i = 1; i <= 5; i++) {
					// Clear all LEDs
					for (int j = 0; j < 11; j++) {
						Set_LED(j, 0, 0, 0); // Turn off LED
					}
					for (int j = 1; j <= i + 1; j++) {
						Set_LED(j, 255, 255, 0);
						Set_LED(j - j, 255, 255, 0);
					}
					for (int j = 10; j >= 11 - i; j--) {
						Set_LED(j, 255, 255, 0);
					}
					WS2812_Send(&htim3, TIM_CHANNEL_2, 105);
					osDelay(120); // Adjust as needed
				}
			} else if (movement_mode == 3) {
				for (int i = 1; i <= 5; i++) {
					// Clear all LEDs
					for (int j = 0; j < 11; j++) {
						Set_LED(j, 0, 0, 0); // Turn off LED
					}
					for (int j = 1; j <= i + 1; j++) {
						Set_LED(j, 255, 0, 0);
						Set_LED(j - j, 255, 0, 0);
					}
					for (int j = 10; j >= 11 - i; j--) {
						Set_LED(j, 255, 0, 0);
					}
					WS2812_Send(&htim3, TIM_CHANNEL_2, 105);
					osDelay(120); // Adjust as needed
				}
			}
		}

		else {
			// Loop through each LED and light it up progressively
			for (int i = 0; i < 11; i++) {
				// Clear all LEDs
				for (int j = 0; j < 11; j++) {
					Set_LED(j, 0, 0, 0); // Turn off LED
				}
				// Light up LEDs from index 0 to i
				for (int j = 0; j <= i; j++) {
					Set_LED(j, 255, 0, 0); // Set LED color to red
				}
				// Send data to LEDs
				WS2812_Send(&htim3, TIM_CHANNEL_2, 105); // Assuming you're sending 105 bits of data
				osDelay(350); // Delay for 500 ms (adjust as needed for desired speed)
			}
			led_status = 0;
		}

    osDelay(1);
  }
  /* USER CODE END StartStroboTask */
}

/* USER CODE BEGIN Header_StartOtherTask */
/**
* @brief Function implementing the otherTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOtherTask */
void StartOtherTask(void const * argument)
{
  /* USER CODE BEGIN StartOtherTask */
  /* Infinite loop */
  for(;;)
  {
		//Switch Actuator Power
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, arm_hw);

		//Leaks Sensor
		leaks = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);

		//Volt Meter Battery
		batt1_volt = convertADCtoVoltage(adc1_value[0], batt1adc_min, batt1adc_max, batt1_min, batt1_max);
		batt2_volt = convertADCtoVoltage(adc1_value[1], batt2adc_min, batt2adc_max, batt2_min, batt2_max);

		//IMU Calibration
//		if(IMUStatus == "Resetting.."){
//
//		}

    osDelay(1);
  }
  /* USER CODE END StartOtherTask */
}

/* USER CODE BEGIN Header_StartPidTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPidTask */
void StartPidTask(void const * argument)
{
  /* USER CODE BEGIN StartPidTask */
  /* Infinite loop */
  for(;;)
  {
	  pid_param(&yaw, c_yaw[0], c_yaw[1], c_yaw[2]);
	  pid_param(&pitch, c_pitch[0], c_pitch[1], c_pitch[2]);
	  pid_param(&roll, c_roll[0], c_roll[1], c_roll[2]);
	  pid_param(&depth, c_depth[0], c_depth[1], c_depth[2]);

	  yaw.input = bno055_yaw;
	  pitch.input = bno055_pitch;
	  roll.input = bno055_roll;
	  depth.input = depthValue;

	  yaw.setpoint = set_point[0];
	  pitch.setpoint = set_point[1];
	  roll.setpoint = set_point[2];
	  depth.setpoint = set_point[3];

	  pid_calculate_rad(&yaw);
	  pid_calculate_rad(&pitch);
	  pid_calculate_rad(&roll);
	  pid_calculate(&depth);

    osDelay(1);
  }
  /* USER CODE END StartPidTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
