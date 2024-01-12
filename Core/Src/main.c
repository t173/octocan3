/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define true 1
#define false 0

// Controller Variables //
#define NUMBER_OF_SENSORS 16 							// Total number of sensors (16)
#define TARGET 26844									// Calibration target, represents 1.024 Volts
#define DAC_CONFIG_BITS 0b0101							// DAC'a configuration bits(DACa:Buffered:2x:PowerDown)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
enum States {sCALIB = 'C', sLOG = 'L', sINCR = 'I',		// Command States
sMUX = 'M', sSTOP = 'S', sSetup = 'E', sRaw = 'X'};
uint16_t sen_dac[16]; 			  						// Sensors calibrated DAC value
uint16_t cal_dac;										// Calibrated DAC value
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
uint16_t mux_func(uint16_t n);							// MUX selector function
void calibrate(); 										// Calibration function
uint16_t get_adc(); 									// Get ADC function
void dac_write(uint16_t value); 						// Sets DAC function
void log_adc(uint8_t data);								// Logs ADC function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */
	uint8_t i,log;
	uint16_t dac_increment = 0;
	uint8_t log_en = false;
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
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	puts("Initialized");
	dac_write(0); // Sets DAC
	//Sensor_SD GPIO Port SET To HIGH
	HAL_GPIO_WritePin(Sensor_SD_GPIO_Port, Sensor_SD_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		char command = 0;
		if(READ_BIT(hlpuart1.Instance->ISR, USART_ISR_RXNE_RXFNE) != 0 ) {
					command = (char)hlpuart1.Instance->RDR;
		}
		//Receive command
		switch(command) {

		// Start Calibration //
			case sCALIB:
				printf("Calibrating\r\n");
				for(i=0; i<NUMBER_OF_SENSORS; i++) {
					mux_func(i);
					calibrate();
					sen_dac[i] = cal_dac;
					printf("cDAC: %hu\r\n", sen_dac[i]);
					printf("Sensor: %hu Success\r\n", i);
				}
				break;

		// Logs ADC Values //
			case sLOG:
				printf("Logging\r\n");
				log = 1;
				log_en = true;
				break;

		// Set DAC Increment Value //
			case sINCR:
				dac_increment += 1;
				printf("DAC Increment is: %hu \r\n", dac_increment);
				break;

		// Select MUX (1-16) //
			case sMUX:
				printf("Mux\r\n");
				mux_func(1);
				break;

		// Stops Logging ADC //
			case sSTOP:
				printf("STOPPING\r\n");
				log_en = false;
				break;

		// Set up Firmware  //
			case sSetup:
				printf("Setup\r\n");
				break;

		// Log raw data	 	//
			case sRaw:
				printf("Logging\r\n");
				log = 2;
				log_en = true;

		//   Default 		//
			default:
				break;
		}


	if (log_en) { log_adc(log); }
	HAL_Delay(1);
    /* USER CODE END WHILE */
	}
    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim6.Init.Prescaler = 64-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sensor_SD_GPIO_Port, Sensor_SD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LDAC_Pin|A3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX_EN_Pin|A0_Pin|A1_Pin|A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_SD_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = Sensor_SD_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LDAC_Pin A3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LDAC_Pin|A3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_EN_Pin A0_Pin A1_Pin A2_Pin */
  GPIO_InitStruct.Pin = MUX_EN_Pin|A0_Pin|A1_Pin|A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE {

 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}





/**
 * @brief Selects Output from MUX
 * @param uint8_t n, uint8_t val
 * @retval 2 if failed, 0 if completed
 */
uint16_t mux_func(uint16_t n) {

	HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_SET);

    if(n>=0 && n<=15) {
    	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, n&1);
    	n >>= 1;
    	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, n&1);
    	n >>= 1;
    	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, n&1);
    	n >>= 1;
    	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, n&1);
    }
    else {
        /* Out of Bounds */
        return 1;
    }
    return 0;
}



/**
 * @brief creates a delay in microseconds
 * @param t total delay in microseconds
 * @retval None
 */
void delay_us(uint16_t t) {

	// Start timer
	SET_BIT(TIM6->CR1, TIM_CR1_CEN);

	// Reset the timer's counter register value to zero
	TIM6->CNT = 0;

	// Busy checking the counter
	while(TIM6->CNT < t);

	// Stop the timer
	CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
}



/**
 * @brief Calibrates sensor
 * @param None
 * @retval true if successful, false otherwise
 */
void calibrate() {

	// Variables //
	uint16_t upper_bound = TARGET + 250;			// upper bounds of target
	uint16_t lower_bound = TARGET - 250;			// lower bounds of target
	uint8_t flag = 0;								// flags 1 if ADC > UB, 0 if ADC < LB
	uint8_t i = 0;									// number of iterations
    uint16_t val_adc[32]; 							// current ADC value
    uint16_t val_dac[32]; 							// current DAC value
    uint16_t incr_dac[32]; 							// current DAC increment

    // sets indexes to 0 //
    for (int i = 0; i < 32; i++) {
    	val_adc[i] = 0;
    	val_dac[i] = 0;
    	incr_dac[i] = 0;
    }

    // First Iteration (i = 0) //
    incr_dac[i] = 4096;
    val_dac[i] = 0;
    dac_write(val_dac[i]);
    delay_us(26);
    val_adc[i] = get_adc();

    // flags initial value //
	if (val_adc[i] > upper_bound) {
		flag = 0;
	}
	else if (val_adc[i] < lower_bound) {
		flag = 1;
	}

	// Start of calibration loop //
    while ((val_adc[i] > upper_bound) || (val_adc[i] < lower_bound)) {

    	// Iteration (1 to 25) //
        i = i + 1;
        if (i >= 32) {
        	break;
        }

        // Checks voltage across event //
        if((val_adc[i-1] < lower_bound) && (flag == 1)) {
            incr_dac[i] = (uint16_t)(incr_dac[i-1]>>1);
            flag = 0;
        }
        else if ((val_adc[i-1] > upper_bound) && (flag == 0)) {
            incr_dac[i] = (uint16_t)(incr_dac[i-1]>>1);
            flag = 1;
        }
        else {
        	incr_dac[i] = incr_dac[i-1];
        }

        // Updates DAC for next iteration //
        if (flag == 1) {
        	val_dac[i] = (val_dac[i-1] + incr_dac[i] > 4095) ? 4095 : val_dac[i-1] + incr_dac[i];
        }
        else {
        	val_dac[i] = (val_dac[i-1] < incr_dac[i]) ? 1 : val_dac[i-1] - incr_dac[i];
        }

        // Update ADC value //
        dac_write(val_dac[i]);
        delay_us(26);
        val_adc[i] = get_adc();
    }

	HAL_Delay(1);

	// Print status //
	printf("Completed in %d steps\r\n", i-1);
	printf("ADC: %d\r\n", val_adc[i-1]);
	printf("MOE: %d from target\r\n", TARGET - val_adc[i-1]);
	cal_dac = val_dac[i-1];
}



/**
 * @brief Reads ADC Value
 * @param None
 * @retval ADC Value
 */
uint16_t get_adc() {

	uint8_t adc_value[2];
	// Start ADC conversion by toggling CNVST pin //
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi2, adc_value, 2, HAL_MAX_DELAY);

	return (adc_value[0] << 8) | adc_value[1];
}



/**
 * @brief Logs ADC Value
 * @param None
 * @retval None
 */
void log_adc(uint8_t data) {
		for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
		        mux_func(i);
		        delay_us(10);
		        dac_write(sen_dac[i]); // 1us to write
		        delay_us(26);			// 26us for adc to update
		        uint32_t cur_adc = get_adc(); // 10us

		        // Average of values //
		        for (int i = 0; i < 10; i++){
		        	cur_adc = cur_adc + get_adc();
		        }
		        cur_adc = cur_adc/10;


		        if (data == 1) {
		        	printf("S%d:%lu, \r\n", i, cur_adc);
		        }
		        else if(data == 2) {
		        	// Address format //
		        	uint8_t address[5] = { // Data address for visualizer
		        			0x55, 					// Start of address
							0x10 + i, // Selected MUX and channel
							0x00, 				// Padding
							(cur_adc>>8) & 0xFF, 			// MSB
							cur_adc & 0xFF		// LSB
		        	};

					// Transmit address //
					HAL_UART_Transmit(&hlpuart1, address, 5, HAL_MAX_DELAY);
		        }
		    }
	}



/**
 * @brief Sends data through SPI Bus
 * @param 16bit Value
 * @retval None
 */
void dac_write(uint16_t value) {

	value = (DAC_CONFIG_BITS << 12) | value;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)&value, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
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
