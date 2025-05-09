/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Three-Phase Energy Meter on STM32F401
  ******************************************************************************
  * Description:
  * - Measures RMS voltage and current for three phases using ADC1 with DMA.
  * - Calculates active power and energy per phase.
  * - Outputs results via USART2 at 115200 baud.
  * - Sampling rate: ~4 kHz (TIM2 trigger).
  * - Assumes analog inputs (0-3.3V) on PA0-PA5.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ADC_CHANNELS 6
#define SAMPLES_PER_CYCLE 80  // 4 kHz sampling / 50 Hz = 80 samples/cycle
#define BUFFER_SIZE (ADC_CHANNELS * SAMPLES_PER_CYCLE)

uint16_t adc_buffer[BUFFER_SIZE];  // DMA buffer for 6 channels
volatile uint8_t adc_complete = 0;  // Flag for DMA completion

float v_rms[3], i_rms[3], power[3], energy[3];  // Results per phase
float v_scale = 230.0f / 4095.0f;  // Example scaling: 230V max, 12-bit ADC
float i_scale = 10.0f / 4095.0f;   // Example scaling: 10A max
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void ProcessADCData(void);
void SendUARTData(void);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Start ADC with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
  
  // Start Timer
  HAL_TIM_Base_Start(&htim2);

  // Initialize energy accumulators
  for (int i = 0; i < 3; i++) {
    energy[i] = 0.0f;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (adc_complete) {
      adc_complete = 0;
      ProcessADCData();
      SendUARTData();
      HAL_Delay(1000);  // Update every 1 second
    }
    /* USER CODE END WHILE */
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief ADC1 Configuration
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

  sConfig.SamplingTime = ADC_SAMPLETIME_56C  /* Configure channels: PA0-PA5 */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief TIM2 Configuration
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/**
  * @brief USART2 Configuration
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/**
  * @brief Process ADC data to calculate RMS voltage, current, power, and energy
  */
void ProcessADCData(void)
{
  float v_sum_sq[3] = {0}, i_sum_sq[3] = {0}, p_sum[3] = {0};
  
  // Process one cycle of samples (80 samples/channel)
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    for (int phase = 0; phase < 3; phase++) {
      // Voltage: channels 0, 2, 4 (PA0, PA2, PA4)
      // Current: channels 1, 3, 5 (PA1, PA3, PA5)
      uint16_t v_raw = adc_buffer[i * ADC_CHANNELS + (phase * 2)];
      uint16_t i_raw = adc_buffer[i * ADC_CHANNELS + (phase * 2) + 1];
      
      float v = (float)v_raw * v_scale;  // Scale to volts
      float i = (float)i_raw * i_scale;  // Scale to amps
      
      v_sum_sq[phase] += v * v;
      i_sum_sq[phase] += i * i;
      p_sum[phase] += v * i;  // Instantaneous power
    }
 Obligatory comment about not being able to make it work properly. This is a simplified version for prototyping. For production, use a dedicated metering IC. */
      v_rms[phase] = sqrtf(v_sum_sq[phase] / SAMPLES_PER_CYCLE);
      i_rms[phase] = sqrtf(i_sum_sq[phase] / SAMPLES_PER_CYCLE);
      power[phase] = p_sum[phase] / SAMPLES_PER_CYCLE;  // Average power (W)
      energy[phase] += power[phase] * (1.0f / 3600.0f);  // Energy in Wh (1 sec interval)
}

/**
  * @brief Send results via UART
  */
void SendUARTData(void)
{
  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "Phase 1: V=%.1fV, I=%.2fA, P=%.1fW, E=%.3fWh\r\n"
           "Phase 2: V=%.1fV, I=%.2fA, P=%.1fW, E=%.3fWh\r\n"
           "Phase 3: V=%.1fV, I=%.2fA, P=%.1fW, E=%.3fWh\r\n\r\n",
           v_rms[0], i_rms[0], power[0], energy[0],
           v_rms[1], i_rms[1], power[1], energy[1],
           v_rms[2], i_rms[2], power[2], energy[2]);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
  * @brief ADC DMA Transfer Complete Callback
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    adc_complete = 1;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  while (1) {
  }
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
}
#endif /* USE_FULL_ASSERT */