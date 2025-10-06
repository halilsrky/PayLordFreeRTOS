/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for SkyLord2 Flight Computer
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

/**
  ******************************************************************************
  * SkyLord2 Flight Computer - Main Application
  *
  * This application implements a comprehensive flight computer system for rocket
  * telemetry and control, featuring:
  *
  * SENSORS:
  * - BME280: Environmental sensor (temperature, humidity, pressure)
  * - BMI088: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
  * - HMC1021: Single-axis magnetometer for magnetic field measurement
  * - L86 GPS/GNSS: Position and navigation data
  *
  * COMMUNICATION:
  * - E22 LoRa module: Long-range wireless telemetry transmission
  * - UART: Debug and telemetry output
  *
  * PROCESSING:
  * - Real-time sensor fusion using quaternion-based algorithms
  * - Kalman filtering for noise reduction
  * - Flight state estimation and control
  *
  * TIMING:
  * - 100ms periodic operations for data acquisition
  * - 1s periodic operations for magnetometer calibration
  * - Continuous sensor monitoring and data fusion
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Standard library includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Sensor drivers */
#include "bme280.h"          // Environmental sensor (temp, humidity, pressure)
#include "bmi088.h"          // 6-axis IMU (accelerometer + gyroscope)
#include "l86_gnss.h"        // GPS/GNSS module

/* Communication modules */
#include "e22_lib.h"         // LoRa wireless communication
#include "packet.h"          // Telemetry packet handling
#include "data_logger.h"     // SD card data logging

/* Algorithm and processing modules */
#include "queternion.h"      // Quaternion mathematics
#include "sensor_fusion.h"   // Sensor fusion algorithms
#include "flight_algorithm.h" // Flight state detection and control

/*MATHEMATICAL CONSTANTS*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_I2C_HNDLR	hi2c1 //put your I2C handler

#define CRNT_COEF					(float)0.0008056641	// Current coefficient (A)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* Definitions for bmiTask */
osThreadId_t bmiTaskHandle;
const osThreadAttr_t bmiTask_attributes = {
  .name = "bmiTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,  // En yüksek öncelik
};

/* Definitions for bmeTask */
osThreadId_t bmeTaskHandle;
const osThreadAttr_t bmeTask_attributes = {
  .name = "bmeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for sensorFusionTask */
osThreadId_t sensorFusionTaskHandle;
const osThreadAttr_t sensorFusionTask_attributes = {
  .name = "sensorFusionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for gnssTask */
osThreadId_t gnssTaskHandle;
const osThreadAttr_t gnssTask_attributes = {
  .name = "gnssTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for dataLoggerTask */
osThreadId_t dataLoggerTaskHandle;
const osThreadAttr_t dataLoggerTask_attributes = {
  .name = "dataLoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/*==================== SENSOR STRUCTURES ====================*/
// Environmental sensor (BME280) - temperature, humidity, pressure
static BME_280_t BME280_sensor;         // BME280 barometric pressure sensor
bmi088_struct_t BMI_sensor;             // BMI088 IMU sensor (accelerometer + gyroscope)
sensor_fusion_t sensor_output;          // Sensor fusion output data
BME_parameters_t bme_params;            // BME280 calibration parameters
gps_data_t gnss_data;                   // L86 GNSS receiver data
static e22_conf_struct_t lora_1;        // LoRa configuration structure

/*==================== COMMUNICATION BUFFERS ====================*/
// UART communication buffers
static char uart_buffer[128];
extern unsigned char normal_paket[50];  // Normal mode telemetry packet
extern unsigned char sd_paket[64];      // SD card data packet

volatile uint8_t usart4_tx_busy = 0;    // UART4 transmission busy flag
volatile uint8_t usart2_tx_busy = 0;    // UART2 transmission busy flag

/*==================== SENSOR STATUS VARIABLES ====================*/
// Sensor initialization and health status
int is_BME_ok = 0;
int is_BMI_ok = 0;
int bmi_status_ok = 0;

/*==================== ADC BUFFERS AND MAGNETOMETER DATA ====================*/
// General ADC buffers
uint16_t adc1_buffer[1];
uint16_t adc2_buffer[1];
float current_mA = 0.0f;
float voltage_V = 0.0f;

// HMC1021 magnetometer (single axis) data
volatile uint16_t hmc1021_adc_buffer[1];
float hmc1021_voltage = 0.0f;
float hmc1021_gauss = 0.0f;

/*==================== INTERRUPT FLAGS ====================*/
// BMI088 data ready interrupt flags
volatile uint8_t bmi088_accel_data_ready = 0;  // EXTI4 - Accelerometer data ready
volatile uint8_t bmi088_gyro_data_ready = 0;   // EXTI3 - Gyroscope data ready

/*==================== FREERTOS SYNCHRONIZATION ====================*/
// Binary semaphore for BMI088 interrupt-driven reading
osSemaphoreId_t bmiDataReadySemaphoreHandle = NULL;

// I2C bus mutex for thread-safe access to I2C1 (shared by BME280 and BMI088)
osMutexId_t i2cMutexHandle = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void bme280_begin(void);
uint8_t bmi_imu_init(void);
void read_ADC(void);
void lora_init(void);
void lora_send_packet_dma(uint8_t *data, uint16_t size);
void uart2_send_packet_dma(uint8_t *data, uint16_t size);

// FreeRTOS Task Functions
void StartBMITask(void *argument);
void StartBMETask(void *argument);
void StartSensorFusionTask(void *argument);
void StartADCTask(void *argument);
void StartGNSSTask(void *argument);
void StartTelemetryTask(void *argument);
void StartDataLoggerTask(void *argument);

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
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	/*==================== TIMER AND INTERRUPT CONFIGURATION ====================*/
	// Configure external interrupt priorities for sensor data ready signals
	// Bu öncelikler FreeRTOS için uygun seviyede ayarlanmıştır (>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
	DWT->CTRL |= (1 << 0);
	SEGGER_SYSVIEW_Conf();

	HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 1);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 1);

	// Enable external interrupts for sensor data ready signals
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	/* ==== GPS/GNSS INITIALIZATION ==== */
	// Initialize L86 GPS/GNSS module
	HAL_UART_Transmit(&huart6, (uint8_t*)"$PMTK251,57600*2C\r\n", 19, 100);
	HAL_UART_DeInit(&huart6);
	huart6.Init.BaudRate = 57600;
	HAL_UART_Init(&huart6);
	HAL_DMA_Init(&hdma_usart6_rx);
	L86_GNSS_Init(&huart6);

	/* ==== SENSOR INITIALIZATION ==== */
	// Initialize BME280 sensor (temperature, humidity, pressure)
	bme280_begin();
	HAL_Delay(1000);
	bme280_config();
	bme280_update();

	// Initialize BMI088 IMU (accelerometer and gyroscope)
	bmi_imu_init();
	bmi088_config(&BMI_sensor);
	get_offset(&BMI_sensor);

	/*==================== SENSOR FUSION INITIALIZATION ====================*/
	// Initialize quaternion-based sensor fusion
	getInitialQuaternion();
	sensor_fusion_init(&BME280_sensor);

	/* ==== LORA COMMUNICATION SETUP ==== */
	e22_config_mode(&lora_1);
	HAL_Delay(20);
	lora_init();
	HAL_Delay(20);
	e22_transmit_mode(&lora_1);

	/* ==== DATA LOGGER INITIALIZATION ==== */
	//data_logger_init();
	BME280_sensor.altitude = BME280_sensor.altitude - BME280_sensor.base_altitude;

	// Startup beep to indicate system ready
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Create I2C mutex for thread-safe I2C bus access */
  const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex"
  };
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);
  if(i2cMutexHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Create binary semaphore for BMI088 interrupt synchronization */
  bmiDataReadySemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  if(bmiDataReadySemaphoreHandle == NULL) {
    Error_Handler();
  }
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of bmiTask */
  bmiTaskHandle = osThreadNew(StartBMITask, NULL, &bmiTask_attributes);
  
  /* creation of bmeTask */
  bmeTaskHandle = osThreadNew(StartBMETask, NULL, &bmeTask_attributes);
  
  /* creation of sensorFusionTask */
  sensorFusionTaskHandle = osThreadNew(StartSensorFusionTask, NULL, &sensorFusionTask_attributes);
  
  /* creation of adcTask */
  adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);
  
  /* creation of gnssTask */
  gnssTaskHandle = osThreadNew(StartGNSSTask, NULL, &gnssTask_attributes);
  
  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  
  /* creation of dataLoggerTask */
  dataLoggerTaskHandle = osThreadNew(StartDataLoggerTask, NULL, &dataLoggerTask_attributes);
  
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
    // Main loop is now empty - all work is done in FreeRTOS tasks
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_M0_Pin|RF_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin|W25_FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RF_M0_Pin RF_M1_Pin */
  GPIO_InitStruct.Pin = RF_M0_Pin|RF_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin W25_FLASH_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|W25_FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize BMI088 IMU sensor
 * @return Initialization status
 * @note Configures both accelerometer and gyroscope with optimal settings
 */
uint8_t bmi_imu_init(void)
{
  // Accelerometer configuration
  BMI_sensor.device_config.acc_bandwith = ACC_BWP_OSR4;
  BMI_sensor.device_config.acc_outputDateRate = ACC_ODR_400;
  BMI_sensor.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
  BMI_sensor.device_config.acc_range = ACC_RANGE_24G;

  // Gyroscope configuration
  BMI_sensor.device_config.gyro_bandWidth = GYRO_BW_23;
  BMI_sensor.device_config.gyro_range = GYRO_RANGE_2000;
  BMI_sensor.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

  // Interrupt and I2C configuration
  BMI_sensor.device_config.acc_IRQ = EXTI3_IRQn;
  BMI_sensor.device_config.gyro_IRQ = EXTI4_IRQn;
  BMI_sensor.device_config.BMI_I2c = &IMU_I2C_HNDLR;
  BMI_sensor.device_config.offsets = NULL; // Offset data stored in backup SRAM

  return bmi088_init(&BMI_sensor);
}

/**
 * @brief GPIO external interrupt callback
 * @param GPIO_Pin The pin that triggered the interrupt
 * @note Handles BMI088 accelerometer and gyroscope data ready interrupts
 * @note Releases semaphore to wake up BMI task (interrupt-driven approach)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(GPIO_Pin == GPIO_PIN_3)
  {
    // Gyroscope data ready interrupt (EXTI3)
    bmi088_set_gyro_INT(&BMI_sensor);

    // Release semaphore from ISR to wake up BMI task
    osSemaphoreRelease(bmiDataReadySemaphoreHandle);
  }
  if(GPIO_Pin == GPIO_PIN_4)
  {
    // Accelerometer data ready interrupt (EXTI4)
    bmi088_set_accel_INT(&BMI_sensor);

    // Release semaphore from ISR to wake up BMI task
    osSemaphoreRelease(bmiDataReadySemaphoreHandle);
  }

  // Request context switch if higher priority task was woken
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief UART transmission complete callback
 * @param huart UART handle
 * @note Clears transmission busy flag
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        usart2_tx_busy = 0;
    }
}

/**
 * @brief Send packet via UART2 using DMA
 * @param data Pointer to data buffer
 * @param size Size of data to send
 * @note Non-blocking transmission using DMA
 */
void uart2_send_packet_dma(uint8_t *data, uint16_t size)
{
    if (!usart2_tx_busy) {
        usart2_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, data, size);
    }
}

/**
 * @brief I2C Memory read complete callback (DMA)
 * @param hi2c I2C handle
 * @note Handles BMI088 sensor data DMA transfer completion
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Debug: increment a counter to see if this callback is called
    static uint32_t i2c_callback_counter = 0;
    i2c_callback_counter++;

    if (hi2c->Instance == I2C1) {
        if (hi2c->Devaddress == ACC_I2C_ADD) {
            // Accelerometer data received (9 bytes: XYZ + sensor time)
            bmi088_accel_dma_complete_callback(&BMI_sensor);
        }
        else if (hi2c->Devaddress == GYRO_I2C_ADD) {
            // Gyroscope data received (6 bytes: XYZ)
            bmi088_gyro_dma_complete_callback(&BMI_sensor);
        }
    }
}

/**
  * @brief  Function implementing the telemetry task.
  * @param  argument: Not used
  * @retval None
  */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */
  for(;;)
  {
    // Create telemetry packets
    addDataPacketNormal(&BME280_sensor, &BMI_sensor, &sensor_output, &gnss_data, hmc1021_gauss, voltage_V, current_mA);

    // Send data via UART2 using DMA
    uart2_send_packet_dma((uint8_t*)normal_paket, 50);

    // Send data via LoRa using DMA
    //lora_send_packet_dma((uint8_t*)normal_paket, 50);

    // Wait for 100ms before next transmission (10Hz telemetry rate)
    osDelay(100);
  }
  /* USER CODE END StartTelemetryTask */
}

/**
  * @brief  Function implementing the BMI088 sensor task.
  * @param  argument: Not used
  * @retval None
  * @note   Interrupt-driven: Task blocks on semaphore, wakes up on EXTI interrupt
  * @note   Uses system tick for delta_time calculation (more reliable in FreeRTOS)
  */
void StartBMITask(void *argument)
{
  /* USER CODE BEGIN StartBMITask */
  /* Infinite loop */
  for(;;)
  {
    // Block and wait for BMI088 data ready interrupt (semaphore from ISR)
    if(osSemaphoreAcquire(bmiDataReadySemaphoreHandle, osWaitForever) == osOK)
    {
      // Acquire I2C bus mutex for thread-safe access
      osMutexAcquire(i2cMutexHandle, osWaitForever);

      // Read BMI088 sensor data
      bmi088_update(&BMI_sensor);

      // Release I2C bus mutex
      osMutexRelease(i2cMutexHandle);
    }
    osDelay(1);
  }
  /* USER CODE END StartBMITask */
}

/**
  * @brief  Function implementing the BME280 sensor task.
  * @param  argument: Not used
  * @retval None
  */
void StartBMETask(void *argument)
{
  /* USER CODE BEGIN StartBMETask */
  /* Infinite loop */
  for(;;)
  {
    // Acquire I2C bus mutex for thread-safe access
    osMutexAcquire(i2cMutexHandle, osWaitForever);

    // Update BME280 barometric pressure sensor data
    bme280_update();

    // Release I2C bus mutex
    osMutexRelease(i2cMutexHandle);

    // Short delay - barometer updates don't need to be as fast as IMU
    osDelay(25);
  }
  /* USER CODE END StartBMETask */
}

/**
  * @brief  Function implementing the sensor fusion task.
  * @param  argument: Not used
  * @retval None
  */
void StartSensorFusionTask(void *argument)
{
  /* USER CODE BEGIN StartSensorFusionTask */
  /* Infinite loop */
  for(;;)
  {
    // Perform sensor fusion using Kalman filter
    sensor_fusion_update_kalman(&BME280_sensor, &BMI_sensor, &sensor_output);

    // Update flight algorithm (launch detection, apogee, etc.)
    flight_algorithm_update(&BME280_sensor, &BMI_sensor, &sensor_output);

    // Run at 10Hz (100ms period) - matches telemetry rate
    osDelay(100);
  }
  /* USER CODE END StartSensorFusionTask */
}

/**
  * @brief  Function implementing the ADC reading task.
  * @param  argument: Not used
  * @retval None
  */
void StartADCTask(void *argument)
{
  /* USER CODE BEGIN StartADCTask */
  /* Infinite loop */
  for(;;)
  {
    // Read ADC channels (magnetometer, voltage, current)
    read_ADC();

    // Run at 10Hz (100ms period)
    osDelay(100);
  }
  /* USER CODE END StartADCTask */
}

/**
  * @brief  Function implementing the GNSS task.
  * @param  argument: Not used
  * @retval None
  */
void StartGNSSTask(void *argument)
{
  /* USER CODE BEGIN StartGNSSTask */
  /* Infinite loop */
  for(;;)
  {
    // Update GPS/GNSS data
    L86_GNSS_Update(&gnss_data);

    // Run at 10Hz (100ms period)
    osDelay(100);
  }
  /* USER CODE END StartGNSSTask */
}

/**
  * @brief  Function implementing the data logger task.
  * @param  argument: Not used
  * @retval None
  */
void StartDataLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartDataLoggerTask */
  /* Infinite loop */
  for(;;)
  {
    // Create SD card data packet
    addDataPacketSD(&BME280_sensor, &BMI_sensor, &sensor_output, &gnss_data, hmc1021_gauss, voltage_V, current_mA);

    // Log data to SD card
    log_normal_packet_data(sd_paket);

    // Run at 10Hz (100ms period) - matches telemetry rate
    osDelay(100);
  }
  /* USER CODE END StartDataLoggerTask */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize LoRa E22 module
 * @note Configures LoRa module for telemetry transmission
 */
void lora_init(void)
{
	lora_1.baud_rate 		= 	E22_BAUD_RATE_115200;
	lora_1.parity_bit		=	E22_PARITY_8N1;
	lora_1.air_rate			=	E22_AIR_DATA_RATE_38400;
	lora_1.packet_size		=	E22_PACKET_SIZE_64;
	lora_1.rssi_noise		=	E22_RSSI_NOISE_DISABLE;
	lora_1.power			=	E22_TRANSMITTING_POWER_22;
	lora_1.rssi_enable		=	E22_ENABLE_RSSI_DISABLE;
	lora_1.mode				= 	E22_TRANSMISSION_MODE_TRANSPARENT;
	lora_1.repeater_func	=	E22_REPEATER_FUNC_DISABLE;
	lora_1.lbt				=	E22_LBT_DISABLE;
	lora_1.wor				=	E22_WOR_RECEIVER;
	lora_1.wor_cycle		=	E22_WOR_CYCLE_1000;
	lora_1.channel			=	28;

	e22_init(&lora_1, &huart4);

	HAL_UART_DeInit(&huart4);
	HAL_Delay(20);
	huart4.Init.BaudRate = 115200;
	HAL_Delay(20);
	HAL_UART_Init(&huart4);
}

/**
 * @brief Initialize BME280 environmental sensor
 * @note Configures BME280 with predefined settings for normal operation
 */
void bme280_begin()
{
  BME280_sensor.device_config.bme280_filter = BME280_FILTER_2;
  BME280_sensor.device_config.bme280_mode = BME280_MODE_NORMAL;
  BME280_sensor.device_config.bme280_output_speed = BME280_OS_4;
  BME280_sensor.device_config.bme280_standby_time = BME280_STBY_125;
  bme280_init(&BME280_sensor, &hi2c3);
}

/**
 * @brief Read HMC1021 magnetometer ADC values
 * @note Converts ADC readings to magnetic field strength, voltage and current
 */
void read_ADC()
{
    static uint16_t adc1_raw = 0;  // ADC1 value (Channel 9 - Magnetometer)
    static uint16_t adc2_raw = 0;  // ADC2 value (Channel 10 - Voltage)
    static uint16_t adc3_raw = 0;  // ADC3 value (Channel 11 - Current)

    // Read ADC1 (Magnetometer)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        adc1_raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // Read ADC2 (Voltage)
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 5) == HAL_OK) {
        adc2_raw = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);

    // Read ADC3 (Current)
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 5) == HAL_OK) {
        adc3_raw = HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);

    // Calculate calibrated values
    hmc1021_voltage = (adc1_raw * 3.3f) / 4096.0f;  // 3.3V reference, 12-bit ADC
    voltage_V = (adc2_raw * 13.2f) / 4096.0f;        // Voltage divider calculation
    current_mA = (adc3_raw * CRNT_COEF);             // Current sensor calibration
    hmc1021_gauss = (hmc1021_voltage - 1.65f) / 1.0f; // 1V/Gauss sensitivity

    // LoRa power management based on battery voltage
    if(voltage_V < 7.0){
        e22_sleep_mode(&lora_1);
    }else if(voltage_V >= 7.0){
        e22_transmit_mode(&lora_1);
    }
}


/**
 * @brief Send packet with LORA using DMA
 * @param data Pointer to data buffer
 * @param size Size of data to send
 * @note Non-blocking transmission using DMA
 */
void lora_send_packet_dma(uint8_t *data, uint16_t size)
{
	if (!usart4_tx_busy) {
		usart4_tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart4, data, size);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the BMI088 sensor update task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Start SEGGER SystemView once at the beginning */
  SEGGER_SYSVIEW_Start();

  /* Infinite loop */
  for(;;)
  {
    osDelay(100); // 100ms delay - reduced frequency for lighter task
  }
  /* USER CODE END 5 */
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
