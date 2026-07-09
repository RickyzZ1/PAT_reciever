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

/* ===== PAT timing ===== */
#define PAT_PERIOD_TICKS      4250UL     /* 170 MHz / 40 kHz = 4250 */
#define TIMER_CLOCK_MHZ       170L       /* TIM1/TIM3 timer clock */

/*
 * In the PA8 -> PA6 loopback test, the measured raw capture was 5.
 * Use 5 as the initial zero offset.
 * Recalibrate this value when using the real receiver path.
 */
#define ZERO_OFFSET_TICKS     5L

/* ===== ADC conversion ===== */
#define ADC_VREF_MV           3300UL
#define ADC_MAX_COUNTS        4095UL

/*
 * PA0 envelope divider:
 *
 * envelope ---- 1M ----+---- PA0
 *                      |
 *                    470k
 *                      |
 *                     GND
 */
#define ENV_DIV_TOP_OHM       1000000UL
#define ENV_DIV_BOTTOM_OHM    470000UL

/*
 * Zero-offset correction for envelope / bias.
 * Start with 0. If Vpeak is not zero with no signal, adjust this value.
 */
#define PEAK_ZERO_OFFSET_MV   0L

/*
 * Pressure conversion parameters.
 * Update these to match the actual transducer sensitivity and total analog gain.
 *
 * RX_SENS_UV_PER_PA:
 *   Receiver transducer sensitivity in uV/Pa
 *
 * AMP_GAIN_X1000:
 *   Total analog gain multiplied by 1000
 *   For example, use 100000 for a total gain of 100x
 */
#define RX_SENS_UV_PER_PA     1000UL
#define AMP_GAIN_X1000        100000UL

/* ===== Input capture state ===== */
volatile uint32_t ic1_raw_ticks = 0;
volatile uint32_t ic1_count = 0;
volatile uint8_t ic1_new_data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static uint8_t read_adc_pair_once(uint16_t *hold, uint16_t *bias);
static uint8_t read_adc_pair_average(uint16_t *hold, uint16_t *bias);

static uint32_t adc_raw_to_adc_mv(uint16_t raw);
static uint32_t adc_raw_to_envelope_mv(uint16_t raw);
static int32_t wrap_to_signed_period(int32_t ticks);

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

  const char boot_msg[] = "PAT STM32 started\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, sizeof(boot_msg) - 1, HAL_MAX_DELAY);

  /* ADC calibration for single-ended input */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /*
  * Start input capture first, then PWM.
  * TIM1 generates 40 kHz on PA8.
  * TIM3 captures phase on PA6.
  */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  printf("TIM1 PA8 PWM 40kHz\r\n");
  printf("TIM3 PA6 input capture CH1\r\n");
  printf("PA0 envelope via 1M/470k divider, PA1 bias direct\r\n");

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

      /*
       * PA0 / hold / envelope:
       *   This input goes through a 1M + 470k divider,
       *   so convert it back to the original envelope voltage.
       *
       * PA1 / bias / virtual GND:
       *   This input is connected directly with no divider,
       *   so use the normal ADC voltage conversion.
       */
      uint32_t vhold_mV = adc_raw_to_envelope_mv(hold_raw);
      uint32_t vbias_mV = adc_raw_to_adc_mv(bias_raw);

      /*
       * amplitude = envelope peak - bias
       */
      int32_t vpeak_mV_signed = (int32_t)vhold_mV - (int32_t)vbias_mV - PEAK_ZERO_OFFSET_MV;
      if (vpeak_mV_signed < 0)
      {
          vpeak_mV_signed = 0;
      }

      uint32_t vpeak_mV = (uint32_t)vpeak_mV_signed;

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

      int32_t phase_mdeg = 0;

      if (has_capture)
      {
          int32_t raw_ticks = (int32_t)(raw_ticks_local % PAT_PERIOD_TICKS);
          int32_t corrected_ticks = wrap_to_signed_period(raw_ticks - ZERO_OFFSET_TICKS);

          /*
           * phase_mdeg = phase in milli-degrees.
           * 1000 mdeg = 1 degree.
           */
          phase_mdeg = (corrected_ticks * 360000L) / (int32_t)PAT_PERIOD_TICKS;
      }

      /*
       * New compact output:
       * Only amplitude and phase.
       */
      char msg[96];

      if (has_capture && adc_ok)
      {
          int32_t phase_abs = phase_mdeg;
          const char *phase_sign = "";

          if (phase_abs < 0)
          {
              phase_abs = -phase_abs;
              phase_sign = "-";
          }

          int n = snprintf(msg, sizeof(msg),
                           "amp_mV=%lu,phase_deg=%s%ld.%03ld\r\n",
                           vpeak_mV,
                           phase_sign,
                           phase_abs / 1000,
                           phase_abs % 1000);

          if (n > 0)
          {
              HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
          }
      }
      else if (!has_capture && adc_ok)
      {
          int n = snprintf(msg, sizeof(msg),
                           "amp_mV=%lu,phase_deg=no_capture\r\n",
                           vpeak_mV);

          if (n > 0)
          {
              HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
          }
      }
      else
      {
          int n = snprintf(msg, sizeof(msg),
                           "amp_mV=adc_error,phase_deg=no_data\r\n");

          if (n > 0)
          {
              HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
          }
      }

#if 0
      /*
       * Old verbose debug output.
       * Keep this commented out. Re-enable only when debugging.
       */

      uint32_t vpp_mV   = 2UL * vpeak_mV;
      uint32_t vrms_mV  = (vpeak_mV * 707UL) / 1000UL;

      uint32_t p_cPa = 0;
      uint64_t denom = (uint64_t)RX_SENS_UV_PER_PA * (uint64_t)AMP_GAIN_X1000;

      if (denom > 0)
      {
          p_cPa = (uint32_t)(((uint64_t)vrms_mV * 100000000ULL) / denom);
      }

      int32_t raw_ticks = (int32_t)(raw_ticks_local % PAT_PERIOD_TICKS);
      int32_t corrected_ticks = wrap_to_signed_period(raw_ticks - ZERO_OFFSET_TICKS);
      int32_t dt_ns = (corrected_ticks * 1000L) / TIMER_CLOCK_MHZ;

      char debug_msg[320];
      int debug_n;

      if (has_capture)
      {
          debug_n = snprintf(debug_msg, sizeof(debug_msg),
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
          debug_n = snprintf(debug_msg, sizeof(debug_msg),
                             "holdRaw=%u,biasRaw=%u,Vhold_mV=%lu,Vbias_mV=%lu,Vpeak_mV=%lu,Vpp_mV=%lu,Vrms_mV=%lu,p_cPa=%lu,no_capture,capCount=%lu,adc_ok=%u\r\n",
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

      if (debug_n > 0)
      {
          HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, (uint16_t)debug_n, 100);
      }
#endif
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

static uint8_t read_adc_pair_once(uint16_t *hold, uint16_t *bias)
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

    *hold = (uint16_t)HAL_ADC_GetValue(&hadc1);

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

static uint8_t read_adc_pair_average(uint16_t *hold, uint16_t *bias)
{
    const uint8_t samples = 16;

    uint32_t hold_sum = 0;
    uint32_t bias_sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        uint16_t h = 0;
        uint16_t b = 0;

        if (!read_adc_pair_once(&h, &b))
        {
            return 0;
        }

        hold_sum += h;
        bias_sum += b;
    }

    *hold = (uint16_t)(hold_sum / samples);
    *bias = (uint16_t)(bias_sum / samples);

    return 1;
}

static uint32_t adc_raw_to_adc_mv(uint16_t raw)
{
    return ((uint32_t)raw * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static uint32_t adc_raw_to_envelope_mv(uint16_t raw)
{
    uint32_t pa0_mv = adc_raw_to_adc_mv(raw);

    return (pa0_mv * (ENV_DIV_TOP_OHM + ENV_DIV_BOTTOM_OHM)) / ENV_DIV_BOTTOM_OHM;
}

static int32_t wrap_to_signed_period(int32_t ticks)
{
    while (ticks > (int32_t)(PAT_PERIOD_TICKS / 2))
    {
        ticks -= PAT_PERIOD_TICKS;
    }

    while (ticks < -(int32_t)(PAT_PERIOD_TICKS / 2))
    {
        ticks += PAT_PERIOD_TICKS;
    }

    return ticks;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
