/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * STM32 PAT receiver migration user code sections
  * Target: NUCLEO-G431KB / STM32G431KBTx
  * CubeMX config assumed:
  *   TIM1_CH1 PA8 PWM 40 kHz, ARR=4249, CCR1=2125, TRGO=Update Event
  *   TIM3_CH1 PA6 Input Capture, ARR=4249, Reset Mode, Trigger=ITR0/TIM1_TRGO, IRQ enabled
  *   ADC1 regular sequence: Rank1=ADC1_IN1/PA0, Rank2=ADC1_IN2/PA1, single-ended
  *   USART2 PA2/PA3 115200 8N1 VCP
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
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

/* USER CODE BEGIN PV */
/* ===== Phase capture ===== */
volatile uint32_t ic1_raw_ticks = 0;
volatile uint32_t ic1_count = 0;
volatile uint8_t  ic1_new_data = 0;

/* TIM1/TIM3 clock = 170 MHz, 40 kHz period = 4250 ticks. */
#define PAT_PERIOD_TICKS       4250L
#define TIMER_CLOCK_HZ         170000000L

/* Set this after calibration. Loopback PA8->PA6 usually gives about 5 ticks. */
#define ZERO_OFFSET_TICKS      0L

/* ===== ADC / analog front-end =====
 * Recommended hardware divider for 5 V MCP6022 outputs:
 * signal -- 10k -- ADC pin -- 18k -- GND
 * ADC pin voltage = original voltage * 18 / (10 + 18)
 * Original voltage = ADC pin voltage * 28 / 18
 */
#define ADC_REF_MV             3300UL
#define ADC_FULL_SCALE         4095UL
#define ADC_DIVIDER_NUM        28UL
#define ADC_DIVIDER_DEN        18UL

/* Original Arduino constants converted to fixed-point friendly values. */
#define PEAK_ZERO_OFFSET_MV    (-70L)   /* original PEAK_ZERO_OFFSET = -0.07 V */
#define AMP_GAIN_X1000         3200UL   /* 3.2 */
#define RX_SENS_UV_PER_PA      1780UL   /* 0.00178 V/Pa = 1780 uV/Pa */

#define ADC_AVG_N              50U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t read_adc_pair_once(uint16_t *amp, uint16_t *bias);
static uint8_t read_adc_pair_average(uint16_t *amp, uint16_t *bias);
static int32_t wrap_to_signed_period(int32_t ticks);
static uint32_t adc_raw_to_original_mv(uint16_t raw);
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  const char boot_msg[] = "BOOT UART DIRECT\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, sizeof(boot_msg) - 1, HAL_MAX_DELAY);
  HAL_Delay(500);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* Start input capture before PWM so TIM3 is ready for the first TIM1 trigger. */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  printf("PAT STM32 migrated from Arduino\r\n");
  printf("PA8=40kHz PWM, PA6=phase capture, PA0=amp, PA1=bias\r\n");
  printf("holdRaw,biasRaw,Vhold_mV,Vbias_mV,Vpeak_mV,Vpp_mV,Vrms_mV,p_cPa,rawTicks,correctedTicks,dt_ns,phase_mdeg,capCount,adc_ok\r\n");
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_report_ms = 0;

    if (HAL_GetTick() - last_report_ms >= 200)
    {
        last_report_ms = HAL_GetTick();

        uint16_t hold_raw = 0;
        uint16_t bias_raw = 0;
        uint8_t adc_ok = read_adc_pair_average(&hold_raw, &bias_raw);

        uint32_t vhold_mV = adc_raw_to_original_mv(hold_raw);
        uint32_t vbias_mV = adc_raw_to_original_mv(bias_raw);

        int32_t vpeak_mV_signed = (int32_t)vhold_mV - (int32_t)vbias_mV - PEAK_ZERO_OFFSET_MV;
        if (vpeak_mV_signed < 0)
        {
            vpeak_mV_signed = 0;
        }

        uint32_t vpeak_mV = (uint32_t)vpeak_mV_signed;
        uint32_t vpp_mV   = 2UL * vpeak_mV;
        uint32_t vrms_mV  = (vpeak_mV * 707UL) / 1000UL;  /* Vpeak / sqrt(2) */

        /* p_rms = Vrms / (sensitivity * gain)
        * p_cPa = p_rms * 100
        * Units: vrms_mV * 1000 = uV, sensitivity in uV/Pa, gain x1000.
        */
        uint32_t p_cPa = 0;
        uint64_t denom = (uint64_t)RX_SENS_UV_PER_PA * (uint64_t)AMP_GAIN_X1000;
        if (denom > 0)
        {
            p_cPa = (uint32_t)(((uint64_t)vrms_mV * 100000000ULL) / denom);
        }

        uint32_t raw_ticks_local = 0;
        uint32_t cap_count_local = 0;
        uint8_t has_capture = 0;

        __disable_irq();
        if (ic1_new_data)
        {
            raw_ticks_local = ic1_raw_ticks;
            cap_count_local = ic1_count;
            ic1_new_data = 0;
            has_capture = 1;
        }
        else
        {
            cap_count_local = ic1_count;
        }
        __enable_irq();

        int32_t raw_ticks = (int32_t)(raw_ticks_local % PAT_PERIOD_TICKS);
        int32_t corrected_ticks = wrap_to_signed_period(raw_ticks - ZERO_OFFSET_TICKS);
        int32_t dt_ns = (corrected_ticks * 1000L) / 170L; /* 1 tick = 1000/170 ns */
        int32_t phase_mdeg = (corrected_ticks * 360000L) / PAT_PERIOD_TICKS;

        char msg[256];
        int n;
        if (has_capture)
        {
            n = snprintf(msg, sizeof(msg),
                        "holdRaw=%u,biasRaw=%u,Vhold_mV=%lu,Vbias_mV=%lu,Vpeak_mV=%lu,Vpp_mV=%lu,Vrms_mV=%lu,p_cPa=%lu,rawTicks=%ld,correctedTicks=%ld,dt_ns=%ld,phase_mdeg=%ld,capCount=%lu,adc_ok=%u\r\n",
                        hold_raw,
                        bias_raw,
                        vhold_mV,
                        vbias_mV,
                        vpeak_mV,
                        vpp_mV,
                        vrms_mV,
                        p_cPa,
                        raw_ticks,
                        corrected_ticks,
                        dt_ns,
                        phase_mdeg,
                        cap_count_local,
                        adc_ok);
        }
        else
        {
            n = snprintf(msg, sizeof(msg),
                        "holdRaw=%u,biasRaw=%u,Vhold_mV=%lu,Vbias_mV=%lu,Vpeak_mV=%lu,Vpp_mV=%lu,Vrms_mV=%lu,p_cPa=%lu,no capture,capCount=%lu,adc_ok=%u\r\n",
                        hold_raw,
                        bias_raw,
                        vhold_mV,
                        vbias_mV,
                        vpeak_mV,
                        vpp_mV,
                        vrms_mV,
                        p_cPa,
                        cap_count_local,
                        adc_ok);
        }

        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
    }
    /* USER CODE END 3 */
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            ic1_raw_ticks = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ic1_count++;
            ic1_new_data = 1;
        }
    }
}

static int32_t wrap_to_signed_period(int32_t ticks)
{
    ticks %= PAT_PERIOD_TICKS;

    if (ticks >= PAT_PERIOD_TICKS / 2)
    {
        ticks -= PAT_PERIOD_TICKS;
    }

    if (ticks < -PAT_PERIOD_TICKS / 2)
    {
        ticks += PAT_PERIOD_TICKS;
    }

    return ticks;
}

static uint32_t adc_raw_to_original_mv(uint16_t raw)
{
    /* Convert STM32 ADC raw -> original 5 V analog-front-end voltage before divider. */
    uint64_t num = (uint64_t)raw * ADC_REF_MV * ADC_DIVIDER_NUM;
    uint64_t den = (uint64_t)ADC_FULL_SCALE * ADC_DIVIDER_DEN;
    return (uint32_t)(num / den);
}

static uint8_t read_adc_pair_once(uint16_t *amp, uint16_t *bias)
{
    HAL_StatusTypeDef status;

    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
        return 0;
    }

    status = HAL_ADC_PollForConversion(&hadc1, 10);
    if (status != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }
    *amp = (uint16_t)HAL_ADC_GetValue(&hadc1);

    status = HAL_ADC_PollForConversion(&hadc1, 10);
    if (status != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }
    *bias = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
    return 1;
}

static uint8_t read_adc_pair_average(uint16_t *amp, uint16_t *bias)
{
    uint32_t amp_sum = 0;
    uint32_t bias_sum = 0;
    uint16_t amp_tmp = 0;
    uint16_t bias_tmp = 0;

    /* Dummy read after ADC sequence/channel switching, like the Arduino version. */
    (void)read_adc_pair_once(&amp_tmp, &bias_tmp);

    for (uint8_t i = 0; i < ADC_AVG_N; i++)
    {
        if (!read_adc_pair_once(&amp_tmp, &bias_tmp))
        {
            return 0;
        }
        amp_sum += amp_tmp;
        bias_sum += bias_tmp;
    }

    *amp = (uint16_t)(amp_sum / ADC_AVG_N);
    *bias = (uint16_t)(bias_sum / ADC_AVG_N);
    return 1;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

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

/**
  * @}
  */

/**
  * @}
  */

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
