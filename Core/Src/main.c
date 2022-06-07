/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define ARM_MATH_CM4

#include <string.h>
#include <stdio.h>
#include "fatfs_sd.h"
#include "string.h"
#include "i2c-lcd.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_TAP_NUM 30
#define N_BUF_FAST_DELAY 5
#define DELAY_ATT(X) (((X) * 9) / 10)

#define INPUT_PROCESS_BUF_SIZE 512
#define INPUT_BUF_SIZE (INPUT_PROCESS_BUF_SIZE * 2)
#define OUTPUT_PROCESS_BUF_SIZE INPUT_PROCESS_BUF_SIZE
#define OUTPUT_BUF_SIZE INPUT_BUF_SIZE
#define OUTPUT_PROCESS_CHAR_BUF_SIZE (INPUT_PROCESS_BUF_SIZE * 5)
#define OUTPUT_BUF_CHAR_SIZE (OUTPUT_PROCESS_CHAR_BUF_SIZE * 2)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for ProcessInput */
osThreadId_t ProcessInputHandle;
const osThreadAttr_t ProcessInput_attributes = {
  .name = "ProcessInput",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for saveFile */
osThreadId_t saveFileHandle;
const osThreadAttr_t saveFile_attributes = {
  .name = "saveFile",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendLCD */
osThreadId_t sendLCDHandle;
const osThreadAttr_t sendLCD_attributes = {
  .name = "sendLCD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 48000 Hz
* 0 Hz - 150 Hz
  gain = 0
  desired ripple = -20 dB
  actual ripple = -20.08 dB

* 220 Hz - 2000 Hz
  gain = 1
  desired attenuation = 5 dB
  actual attenuation = 4.14 dB
*/
static float32_t filter_taps_float[FILTER_TAP_NUM] = {
		  0.1196913088823815,
		  0.004347010228791285,
		  0.0010578249672436534,
		  -0.004422105134559838,
		  -0.01193656282028789,
		  -0.020996621559203277,
		  -0.031709398956511294,
		  -0.04297005892678021,
		  -0.054406571927099606,
		  -0.06550117002110387,
		  -0.07560335150390765,
		  -0.0842508638101497,
		  -0.09077684558266322,
		  -0.09481647180425504,
		  0.9037917577572666,
		  -0.09481647180425504,
		  -0.09077684558266322,
		  -0.0842508638101497,
		  -0.07560335150390765,
		  -0.06550117002110387,
		  -0.054406571927099606,
		  -0.04297005892678021,
		  -0.031709398956511294,
		  -0.020996621559203277,
		  -0.01193656282028789,
		  -0.004422105134559838,
		  0.0010578249672436534,
		  0.004347010228791285,
		  0.1,
		  0
};

static q15_t filter_taps_q15[FILTER_TAP_NUM];


/*********** FILE I/O **************/
FATFS fs;  // File System to load
FIL fil; // File input pointer
FRESULT fstatus;  // result
UINT br, bw;  // File read/write count

/**** PONG BUFFERS ******/
q15_t in_buf[INPUT_BUF_SIZE]; // Input buffer for capturing microphone inputs
q15_t out_buf[OUTPUT_BUF_SIZE]; // Output buffer after processing
char out_buf_char[OUTPUT_BUF_CHAR_SIZE]; // Output character buffer for sending in string from via UART and file I/O

// Pointers to buffer's start to currently fill/process
q15_t* in_buf_ptr = &in_buf[0];
q15_t* out_buf_ptr = &out_buf[0];
char* out_buf_char_ptr = &out_buf_char[0];


/********* INTERMEDIATE BUFFERS ************/
static arm_fir_instance_q15 fir_settings;
static q15_t fir_state [INPUT_PROCESS_BUF_SIZE + FILTER_TAP_NUM - 1];

static arm_rfft_instance_q15 fft_settings;
static q15_t fft_input[INPUT_PROCESS_BUF_SIZE];
static q15_t fft_output[INPUT_PROCESS_BUF_SIZE * 2]; //has to be twice FFT size cuz real and imag
static q15_t robot_buf[INPUT_PROCESS_BUF_SIZE];

/********* FLAGS **************/

static uint8_t write_file_chunk_flag = 0; // When pong buffer is full, signal to write to file
static uint8_t robot_effect = 0; // Toggle robot voice in input
static uint8_t sound_activity_detected = 1;
static int8_t fft_counter = 0;
// Sample TX data to send to LCD

static long unsigned int max_fft_freq;

/********* STATUSES **************/
HAL_StatusTypeDef dma_status;
arm_status dsp_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartProcessInput(void *argument);
void StartSaveFile(void *argument);
void StartSendLCD(void *argument);

/* USER CODE BEGIN PFP */
FRESULT open_append(FIL* fp, const char* path);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void fast_delay() {
	static int16_t bound_check = 0;
	memset(robot_buf, '\0', sizeof(q15_t) * INPUT_PROCESS_BUF_SIZE);
	if (&out_buf[0] == out_buf_ptr) {
		bound_check = OUTPUT_BUF_SIZE;
	} else {
		bound_check = OUTPUT_PROCESS_BUF_SIZE;
	}

	for (int16_t i = 0; i < INPUT_PROCESS_BUF_SIZE; i++) {
		for (int16_t j = 0; j < N_BUF_FAST_DELAY; j++) {
			robot_buf[i] += DELAY_ATT(out_buf[(bound_check + i - j) % OUTPUT_BUF_SIZE]);
		}
	}

//	for (int16_t i = N_BUF_FAST_DELAY; i < INPUT_PROCESS_BUF_SIZE; i++) {
//		// robot_buf[i] = robot_buf[i - 1] + DELAY_ATT(out_buf_ptr[i] - out_buf_ptr[i - N_BUF_FAST_DELAY]);
//		for (int16_t j = 0; j < N_BUF_FAST_DELAY; j++) {
//			robot_buf[i] += DELAY_ATT(out_buf_ptr[i - j]);
//		}
//	}
}


void send_uart(char* string, uint16_t len)
{
	dma_status = HAL_UART_Transmit(&huart2, (uint8_t *) string, len, 2000);
	if (dma_status != HAL_OK) {
		Error_Handler();
	}
}


FRESULT open_append(FIL* fp, /* [OUT] File object to create */
const TCHAR* path /* [IN]  File name to be opened */
) {
	/* Opens an existing file. If not exist, creates a new file. */
	fstatus = f_open(fp, path, FA_OPEN_ALWAYS | FA_WRITE);
	return fstatus;
}

void mount_sd_card() {
	char buffer[128];
	char mount_status_msg[50];
	FATFS *pfs;
	DWORD fre_clust;
	uint32_t total, free_space;

	 /* Mount SD card*/
	fstatus = f_mount(&fs, "/", 1);

	if (fstatus != FR_OK) {
		strcpy(mount_status_msg, "ERROR!!! in mounting SD CARD...\n\n");
	} else {
		strcpy(mount_status_msg, "SD CARD mounted successfully...\n\n");
	}

	send_uart (mount_status_msg, sizeof mount_status_msg);


	/* Check free space and pointer for calculating total size in SD Card */
	f_getfree("/", &fre_clust, &pfs);

	total = (pfs->n_fatent - 2) * pfs -> csize * 0.5;
	sprintf(buffer, "SD card total size:\t%lu\n", total);
	send_uart(buffer, sizeof buffer);
	memset(buffer, '\0', sizeof buffer);


	free_space = fre_clust * pfs -> csize * 0.5;
	sprintf(buffer, "SD card free space:\t%lu\n", free_space);
	send_uart(buffer, sizeof buffer);
	memset(buffer, '\0', sizeof buffer);

}

void process_data() {
	char sample_out_msg[6];
	q15_t max_ampl;

	// 1. Initial HPF
	arm_fir_fast_q15(&fir_settings, in_buf_ptr, out_buf_ptr, INPUT_PROCESS_BUF_SIZE);
	arm_offset_q15(out_buf_ptr, 2000, out_buf_ptr, INPUT_PROCESS_BUF_SIZE);
	// 2. TODO: Correct Robot effect from out_buf_ptr -> robot_buf
	if (robot_effect) {
		fast_delay();
		/*
		for (int i = 0; i < INPUT_PROCESS_BUF_SIZE; i++) {
			robot_buf[i] >>= 2;
		}
		*/
		arm_offset_q15(robot_buf, -7000, robot_buf, OUTPUT_PROCESS_BUF_SIZE);
//		// TODO: (Probably) Generate correct tap for Another HPF, for that change settings? and also try it with fast
		arm_fir_q15(&fir_settings, robot_buf, out_buf_ptr, INPUT_PROCESS_BUF_SIZE);
//		// TODO: (Probably) Change offset?
		arm_offset_q15(out_buf_ptr, 2000, out_buf_ptr, INPUT_PROCESS_BUF_SIZE);
	}

	// 4. Compute FFT and maximum frequency
	// http://gaidi.ca/weblog/configuring-cmsis-dsp-package-and-performing-a-real-fft
	// https://m0agx.eu/2018/05/23/practical-fft-on-microcontrollers-using-cmsis-dsp/
	memcpy(fft_input, out_buf_ptr, sizeof(q15_t) * OUTPUT_PROCESS_BUF_SIZE);
	arm_rfft_q15(&fft_settings, (q15_t* ) fft_input, fft_output);
	for (int i = 0; i < OUTPUT_PROCESS_BUF_SIZE * 2; i++) {
		fft_output[i] <<= 8;
	}
	arm_cmplx_mag_q15(fft_output, fft_output, OUTPUT_PROCESS_BUF_SIZE);
 	arm_max_q15(&fft_output[1], OUTPUT_PROCESS_BUF_SIZE - 1, &max_ampl, &max_fft_freq);


 	// 5. Sound activity detected flag (extension)
	if (max_fft_freq <= 5) {
		if (fft_counter > 20) {
			sound_activity_detected = 0; // TODO: Change
		} else {
			fft_counter++;
		}

	} else {
		if (fft_counter <= 0) {
			sound_activity_detected = 1;
		} else {
			fft_counter-=4;
		}
	}

	for (int i = 0; i < INPUT_PROCESS_BUF_SIZE; i++) {
		// Process 4 digit number as string in processed buffer to send in UART/File
		sprintf (sample_out_msg, "%4.4hd\n", out_buf_ptr[i]);
		strncpy(&out_buf_char_ptr[5*i], sample_out_msg, (sizeof sample_out_msg) - 1);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) in_buf_ptr, INPUT_BUF_SIZE);

  arm_float_to_q15(filter_taps_float, filter_taps_q15, FILTER_TAP_NUM);
  dsp_status = arm_fir_init_q15(&fir_settings, FILTER_TAP_NUM, &filter_taps_q15[0], &fir_state[0], INPUT_PROCESS_BUF_SIZE);
  dsp_status = arm_rfft_init_q15(&fft_settings, INPUT_PROCESS_BUF_SIZE/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);

  // Initial check for sd card
  mount_sd_card();


  // Initial operations on LCD
  lcd_init();
  lcd_clear();

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
  /* creation of ProcessInput */
  ProcessInputHandle = osThreadNew(StartProcessInput, NULL, &ProcessInput_attributes);

  /* creation of saveFile */
  saveFileHandle = osThreadNew(StartSaveFile, NULL, &saveFile_attributes);

  /* creation of sendLCD */
  sendLCDHandle = osThreadNew(StartSendLCD, NULL, &sendLCD_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 10-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CSI_GPIO_Port, SD_CSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : toggle_robot_btn_Pin */
  GPIO_InitStruct.Pin = toggle_robot_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(toggle_robot_btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CSI_Pin */
  GPIO_InitStruct.Pin = SD_CSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : save_file_btn_Pin */
  GPIO_InitStruct.Pin = save_file_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(save_file_btn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	in_buf_ptr = &in_buf[0];
	out_buf_ptr = &out_buf[OUTPUT_PROCESS_BUF_SIZE];
	out_buf_char_ptr = &out_buf_char[OUTPUT_PROCESS_CHAR_BUF_SIZE];
	write_file_chunk_flag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	in_buf_ptr = &in_buf[INPUT_PROCESS_BUF_SIZE];
	out_buf_ptr = &out_buf[0];
	out_buf_char_ptr = &out_buf_char[0];
	write_file_chunk_flag = 1;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &in_buf[0], INPUT_BUF_SIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == toggle_robot_btn_Pin) {
		robot_effect ^= 1;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartProcessInput */
/**
  * @brief  Function implementing the ProcessInput thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProcessInput */
void StartProcessInput(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (write_file_chunk_flag == 1) {
		  process_data();
		  if (sound_activity_detected) {
			  send_uart(out_buf_char_ptr, OUTPUT_PROCESS_CHAR_BUF_SIZE);
		  }
		  write_file_chunk_flag = 2;
	  }
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSaveFile */
/**
* @brief Function implementing the saveFile thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSaveFile */
void StartSaveFile(void *argument)
{
  /* USER CODE BEGIN StartSaveFile */

	char saved_file_name[32] = "SDcard_save_1.txt";
	int current_file_no = 1;
	fstatus = open_append(&fil,saved_file_name);
  /* Infinite loop */
  for(;;)
  {
	  // Check whether the other half of the buffer has been filled after processing the current chunk
	  if (write_file_chunk_flag == 2) {
		  if (sound_activity_detected) {
			  fstatus = f_write(&fil, out_buf_char_ptr, OUTPUT_PROCESS_CHAR_BUF_SIZE, &bw);
		  }

		  // Check whether a file needs to be closed (can only be done after writing)
		  if (HAL_GPIO_ReadPin (save_file_btn_GPIO_Port, save_file_btn_Pin) == 1) {
			  fstatus = f_close(&fil);

			  lcd_clear();
			  lcd_send_string("Saving file...\n");
			  osDelay(1000);
			  lcd_clear();
			  sprintf(saved_file_name, "SDcard_save_%d.txt", ++current_file_no);
			  fstatus= open_append(&fil,saved_file_name);
		  }
		  write_file_chunk_flag = 0;
	  }
    osDelay(1);
  }
  /* USER CODE END StartSaveFile */
}

/* USER CODE BEGIN Header_StartSendLCD */
/**
* @brief Function implementing the sendLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendLCD */
void StartSendLCD(void *argument)
{

  /* USER CODE BEGIN StartSendLCD */
	char max_freq_char[21];
  /* Infinite loop */
  for(;;)
  {
	  lcd_clear();
	  if (sound_activity_detected) {
		  	  sprintf(max_freq_char, "Max %4.4lu     R %d", max_fft_freq * 8, robot_effect);
		  	  lcd_send_string(max_freq_char);
	  } else {
		  lcd_send_string("No sound detected!");
	  }
	  osDelay(1000);
	  // osDelay(1);
  }
  /* USER CODE END StartSendLCD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
