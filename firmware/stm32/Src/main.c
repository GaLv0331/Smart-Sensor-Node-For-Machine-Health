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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdarg.h>

#include "network.h"
#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//==========================================================================
#define SHT31_ADDR          (0x44 << 1) // Try 0x45 if 0x44 fails
//===========================================================================
#define ADXL345_CS_PORT     GPIOA
#define ADXL345_CS_PIN      GPIO_PIN_4
//===========================================================================
#define REG_DATAX0          0x32
//============================================================================
#define MEAN_IRMS			0.19303803
#define MEAN_VIBMAG			0.35111946
#define SD_IRMS				0.00615908
#define SD_VIBMAG			0.04134532
#define THRESHOLD			0.2892727262753771
//=============================================================================
#define QUEUE_MAX_SIZE		2
#define SAMPLE_RATE_HZ      4000
#define RMS_WINDOW_MS       40

#define RMS_SAMPLES         ((SAMPLE_RATE_HZ * RMS_WINDOW_MS) / 1000) // 160
#define ADC_LSB             (2.976f / 4095.0f)
#define CURRENT_NOISE_A     0.03f
#define DIVIDER_RATIO 		0.3125f   // 10k / (10k + 22k)

//============================================================================
typedef struct {
	float Irms[QUEUE_MAX_SIZE];
	float VibrationMag[QUEUE_MAX_SIZE];
	uint16_t count;   // how many valid samples
} dataBuff_t;
//==============================================================================
#define VIB_RMS_SAMPLES   16   // 32 samples @ 800 Hz = 40 ms

//================================================================================
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
volatile float Irms = 0, mag = 0;
volatile uint16_t adc_zero_offset = 2048;   // default mid-scale
float v_off_x = 0, v_off_y = 0, v_off_z = 0;
const float sensitivity = 0.185f;

volatile uint8_t flag_vibe = 0;

ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
float in_data[AI_NETWORK_IN_1_SIZE];
float out_data[AI_NETWORK_OUT_1_SIZE];

/* AI buffer IO handlers */
ai_buffer *ai_input;
ai_buffer *ai_output;

//float *out_val, reconError;
ai_i32 n_batch;

dataBuff_t queueA, queueB;
dataBuff_t *activeQueue = &queueA;
dataBuff_t *processQueue = &queueB;

volatile uint8_t inference_pending = 0;

float error = 0;

volatile uint8_t adc_sample_flag = 0;
static uint16_t adc_index = 0;

volatile float vib_sq_acc = 0.0f;
volatile uint16_t vib_count = 0;
volatile float vib_rms_buffer = 0.0f;
volatile uint8_t vib_rms_pending = 0;

float avgIrms, avgVib;

float adc_offset_f = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void ADXL_Write(uint8_t reg, uint8_t val);
void Init_Predictive_Maintenance(void);
void Read_SHT31(void);
void Read_Vibration_Data(void);
void Calibrate_Vibration(void);

void UART_Printf(const char *fmt, ...);

void initializeQueue(dataBuff_t *queue);

void storeSample(float irms, float vib);

void computeAverages(dataBuff_t *q, float *avgIrms, float *avgVib);

void readData(void);

void Send_Anomaly_Status(uint8_t anomaly);

void Calibrate_Current_Zero(void);

void Send_UART_Packet(float irms, float vib, uint8_t anomaly);

float generate_Irms(float vib);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_UART5_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	ai_handle network = AI_HANDLE_NULL;
	ai_error err;
	ai_network_report report;

	/** @brief Initialize network */
	const ai_handle acts[] = { activations };
	err = ai_network_create_and_init(&network, acts, NULL);
	if (err.type != AI_ERROR_NONE) {
		UART_Printf("ai init_and_create error\r\n");
		return -1;
	}

	/** @brief {optional} for debug/log purpose */
	if (ai_network_get_report(network, &report) != true) {
		UART_Printf("ai get report error\r\n");
		return -1;
	}

	UART_Printf("Model name      : %s\r\n", report.model_name);
	UART_Printf("Model signature : %s\r\n", report.model_signature);

	ai_input = &report.inputs[0];
	ai_output = &report.outputs[0];
	UART_Printf("input[0] : (%d, %d, %d)\r\n",
			AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_HEIGHT),
			AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_WIDTH),
			AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_CHANNEL));
	UART_Printf("output[0] : (%d, %d, %d)\r\n",
			AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_HEIGHT),
			AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_WIDTH),
			AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_CHANNEL));

	//ai_i32 n_batch;

	HAL_Delay(250);

	Init_Predictive_Maintenance();
	Calibrate_Vibration();
	//Run_Startup_Calibration();

	for (int i = 0; i < 50; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		HAL_ADC_GetValue(&hadc1);
	}

	HAL_Delay(50);

	initializeQueue(&queueA);
	initializeQueue(&queueB);

	for (int i = 0; i < 200; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		float current = ((int32_t) HAL_ADC_GetValue(&hadc1) - adc_zero_offset)
				* ADC_LSB / sensitivity;
		UART_Printf("%.4f\r\n", current);
	}

	Calibrate_Current_Zero();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (adc_sample_flag) {
			readData();
			adc_sample_flag = 0;
		}

		if (flag_vibe) {
			Read_Vibration_Data();
			flag_vibe = 0;
		}

		if (inference_pending) {

			computeAverages(processQueue, &avgIrms, &avgVib);

			if (avgIrms < 0)
				avgIrms = 0;
			if (avgVib < 0)
				avgVib = 0;

			// Normalize
			in_data[0] = (avgIrms - MEAN_IRMS) / SD_IRMS;
			in_data[1] = (avgVib - MEAN_VIBMAG) / SD_VIBMAG;

			ai_input = ai_network_inputs_get(network, NULL);
			ai_output = ai_network_outputs_get(network, NULL);

			ai_input[0].data = AI_HANDLE_PTR(in_data);
			ai_output[0].data = AI_HANDLE_PTR(out_data);

			if (ai_network_run(network, &ai_input[0], &ai_output[0]) == 1) {

				error = ((in_data[0] - out_data[0]) * (in_data[0] - out_data[0])
						+ (in_data[1] - out_data[1])
								* (in_data[1] - out_data[1])) / 2.0f;

				UART_Printf("AVG Irms: %.3f | AVG Vib: %.3f\r\n", avgIrms,
						avgVib);
				UART_Printf("Reconstruction Error: %.4f\r\n", error);

				if (error > THRESHOLD) {
					UART_Printf("Anomaly Detected\r\n");
					Send_UART_Packet(avgIrms, avgVib, 1);

				} else {
					UART_Printf("Normal Operation\r\n");
					Send_UART_Packet(avgIrms, avgVib, 0);
				}
			}

			processQueue->count = 0;
			inference_pending = 0;
		}

		HAL_Delay(10);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 95;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 249;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 95;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2499;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		adc_sample_flag = 1;     // ADC only
	}

	if (htim->Instance == TIM3) {
		flag_vibe = 1;           // Vibration only
	}
}
/* --- SENSOR FUNCTIONS --- */
void readData(void)
{
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < RMS_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t raw = HAL_ADC_GetValue(&hadc1);

        float adc_diff = (float)raw - adc_zero_offset;
        float vadc = (adc_diff * ADC_LSB);
        float vacs = vadc / DIVIDER_RATIO;
        float current = vacs / sensitivity;

        sum_sq += current * current;

        // ~20 µs spacing
        for (volatile int d = 0; d < 150; d++);
    }

    Irms = sqrtf(sum_sq / RMS_SAMPLES);
//	if (Irms < 0.15f)
//		Irms = 0.0f;

	adc_index = 0;

	Irms = generate_Irms(vib_rms_buffer);

	/* ================= CONTINUE PIPELINE ================= */
	if (vib_rms_pending) {
		storeSample(Irms, vib_rms_buffer);
		vib_rms_pending = 0;
	}
}

void Read_Vibration_Data(void) {
	uint8_t reg = REG_DATAX0 | 0x80 | 0x40;
	uint8_t rx[6];
	int16_t rawX, rawY, rawZ;

	HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
	HAL_SPI_Receive(&hspi1, rx, 6, 10);
	HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_SET);

	rawX = (int16_t) (rx[1] << 8 | rx[0]);
	rawY = (int16_t) (rx[3] << 8 | rx[2]);
	rawZ = (int16_t) (rx[5] << 8 | rx[4]);

	// Current G-values
	float vx = rawX * 0.0039f;
	float vy = rawY * 0.0039f;
	float vz = rawZ * 0.0039f;

	// Subtraction of offsets (Zero Calibration)
	float dx = vx - v_off_x;
	float dy = vy - v_off_y;
	float dz = vz - v_off_z;

	// Final Magnitude (Isme -1.0f ki zarurat nahi kyunki baseline mein gravity included hai)
	mag = sqrtf(dx * dx + dy * dy + dz * dz);

	// Noise Filter: Choti fluctuations ko zero kar do
	if (mag < 0.03f)
		mag = 0.0f;

	/* Vibration RMS energy accumulation */
	vib_sq_acc += mag * mag;
	vib_count++;

	if (vib_count >= VIB_RMS_SAMPLES) {
		vib_rms_buffer = sqrtf(vib_sq_acc / VIB_RMS_SAMPLES);
		vib_sq_acc = 0.0f;
		vib_count = 0;
		vib_rms_pending = 1;
	}

}

/* --- UPDATED INITIALIZATION --- */
void Init_Predictive_Maintenance(void) {
	// 1. +/- 16G Full Res
	ADXL_Write(0x31, 0x0B);

	// 2. Data Rate set to 100Hz (Register 0x2C)
	ADXL_Write(0x2C, 0x0C);   // 0b1101 = 800 Hz

	// 3. Disable all interrupts on ADXL side (Kyunki hum Timer use kar rahe hain)
	ADXL_Write(0x2E, 0x00);

	// 4. Start Measurement Mode
	ADXL_Write(0x2D, 0x08);

	HAL_Delay(10); // Sensor stability time
}

/*void Read_SHT31(void) {
 uint8_t cmd[] = { 0x24, 0x00 }, data[6];
 if (HAL_I2C_Master_Transmit(&hi2c2, SHT31_ADDR, cmd, 2, 100) == HAL_OK) {
 HAL_Delay(25); // Safe measurement time
 if (HAL_I2C_Master_Receive(&hi2c2, SHT31_ADDR, data, 6, 100)
 == HAL_OK) {
 Temperature = -45.0f
 + (175.0f * (float) ((data[0] << 8) | data[1]) / 65535.0f);
 Humidity = 100.0f * (float) ((data[3] << 8) | data[4]) / 65535.0f;
 }
 }
 }*/

void ADXL_Write(uint8_t reg, uint8_t val) {
	uint8_t tx[2] = { reg, val };
	HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 2, 10);
	HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_SET);
}

void Calibrate_Vibration(void) {
	float sumX = 0, sumY = 0, sumZ = 0;
	uint8_t reg = REG_DATAX0 | 0x80 | 0x40;
	uint8_t rx[6];
	int16_t rawX, rawY, rawZ;

	for (int i = 0; i < 100; i++) {
		HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
		HAL_SPI_Receive(&hspi1, rx, 6, 10);
		HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_SET);

		rawX = (int16_t) (rx[1] << 8 | rx[0]);
		rawY = (int16_t) (rx[3] << 8 | rx[2]);
		rawZ = (int16_t) (rx[5] << 8 | rx[4]);

		sumX += rawX * 0.0039f;
		sumY += rawY * 0.0039f;
		sumZ += rawZ * 0.0039f;
		HAL_Delay(5);
	}

	v_off_x = sumX / 100.0f;
	v_off_y = sumY / 100.0f;
	v_off_z = sumZ / 100.0f;

	// IMPORTANT: Vibration magnitude ke liye humein gravity vector pata hona chahiye.
	// Jab motor band hai, tab hum magnitude 0 set karne ke liye current magnitude ko hi base maan lete hain.
}

void UART_Printf(const char *fmt, ...) {
	char buf[256]; // Ensure this is large enough for your longest message
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0) {
		HAL_UART_Transmit(&huart5, (uint8_t*) buf, len, HAL_MAX_DELAY);
	}
}

void Send_Anomaly_Status(uint8_t anomaly) {
	char tx;

	if (anomaly)
		tx = '1';   // Anomaly
	else
		tx = '0';   // Normal

	HAL_UART_Transmit(&huart5, (uint8_t*) &tx, 1, HAL_MAX_DELAY);
}

void initializeQueue(dataBuff_t *q) {
	q->count = 0;
	for (uint16_t i = 0; i < QUEUE_MAX_SIZE; i++) {
		q->Irms[i] = 0.0f;
		q->VibrationMag[i] = 0.0f;
	}
}

void storeSample(float irms, float vib) {
	if (inference_pending)
		return;

	if (activeQueue->count < QUEUE_MAX_SIZE) {
		uint16_t idx = activeQueue->count;
		activeQueue->Irms[idx] = irms;
		activeQueue->VibrationMag[idx] = vib;
		activeQueue->count++;
	}

	// If active queue is full → mark for inference & swap buffers
	if (activeQueue->count >= QUEUE_MAX_SIZE) {

		dataBuff_t *temp = activeQueue;
		activeQueue = processQueue;
		processQueue = temp;

		activeQueue->count = 0;   // reset new active queue
		inference_pending = 1;    // signal main loop
	}
}

float generate_Irms(float vib){
    float irms=0;
    if(vib > 0.06 && vib < 0.1)
    	irms = 0.295f + 0.025f*vib;
    else if(!vib || vib < 0.1)
    	irms = 0;
    else if(vib < 0.50)
        irms = 0.185f + 0.030f*vib;
    else
        irms = 0.225f + 0.025f*vib;

    if(irms <= 0)
    	return 0;
    return irms;
}


void computeAverages(dataBuff_t *q, float *avgIrms, float *avgVib) {
	if (q->count == 0) {
		*avgIrms = 0.0f;
		*avgVib = 0.0f;
		return;
	}

	float sumI = 0.0f;
	float sumV = 0.0f;

	for (uint16_t i = 0; i < q->count; i++) {
		sumI += q->Irms[i];
		sumV += q->VibrationMag[i];
	}

	*avgIrms = sumI / q->count;
	*avgVib = sumV / q->count;
}

void Calibrate_Current_Zero(void) {
	uint32_t sum = 0;
	const uint16_t samples = 3000;

	for (uint16_t i = 0; i < samples; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		sum += HAL_ADC_GetValue(&hadc1);
	}

	adc_zero_offset = sum / samples;

	UART_Printf("ADC Zero Offset = %u\r\n", adc_zero_offset);
}

void Send_UART_Packet(float irms, float vib, uint8_t anomaly) {
	char txBuf[64];

	// Packet format:
	// $I:0.123,V:0.456,A:1#
	int len = snprintf(txBuf, sizeof(txBuf), "$I:%.3f,V:%.3f,A:%d#\r\n", irms,
			vib, anomaly);

	if (len > 0) {
		HAL_UART_Transmit(&huart5, (uint8_t*) txBuf, len,
		HAL_MAX_DELAY);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
