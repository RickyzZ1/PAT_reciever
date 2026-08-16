/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PAT 4-channel buffered-waveform receiver
  ******************************************************************************
  *
  * Target:
  *   NUCLEO-G431KB / STM32G431KBTx
  *
  * 40 kHz transmit/reference:
  *   PA8 / TIM1_CH1 = 40 kHz PWM
  *   TIM1 TRGO      = Update Event
  *
  * Phase capture:
  *   TIM3 slave mode = Reset Mode
  *   TIM3 trigger    = ITR0 / TIM1_TRGO
  *
  *   PA6 / TIM3_CH1 = RX1 comparator
  *   PA7 / TIM3_CH2 = RX2 comparator
  *   PB0 / TIM3_CH3 = RX3 comparator
  *   PB7 / TIM3_CH4 = RX4 comparator
  *
  * Buffered waveform ADC inputs:
  *   PA0 / ADC1_IN1  = CH1_TO_ADC
  *   PA1 / ADC1_IN2  = CH2_TO_ADC
  *   PA4 / ADC2_IN17 = CH3_TO_ADC
  *   PA5 / ADC2_IN13 = CH4_TO_ADC
  *
  * Each CHx_TO_ADC signal is taken after the 10k/15k divider and
  * MCP6022 voltage follower.  The firmware reports both the voltage
  * measured at the ADC buffer output and the estimated amplifier-output
  * voltage restored by 5/3.
  *
  * ADC acquisition used by this file:
  *   ADC1: scan 2 ranks continuously, DMA normal
  *         rank 1 = ADC1_IN1, rank 2 = ADC1_IN2
  *   ADC2: scan 2 ranks continuously, DMA normal
  *         rank 1 = ADC2_IN17, rank 2 = ADC2_IN13
  *   Software trigger, continuous conversion enabled.
  *
  * USART2:
  *   PA2 / PA3, 115200 baud
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint16_t adc_min;
  uint16_t adc_max;

  uint32_t buffer_dc_mV;
  uint32_t buffer_rms_mV;
  uint32_t buffer_vpp_mV;

  uint32_t amp_rms_mV;
  uint32_t amp_vpp_mV;

  uint32_t pressure_cPa;

  uint8_t clipping;
} RxAmplitudeResult;

typedef struct
{
  uint32_t raw_ticks;
  uint32_t capture_count;
  uint32_t capture_delta;

  int32_t corrected_ticks;
  int32_t delay_ns;
  int32_t phase_mdeg;

  uint8_t has_capture;
  uint8_t edge_rate_valid;
  uint8_t phase_valid;
} RxPhaseResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_CHANNEL_COUNT              4U

/*
 * TIM1/TIM3 timer clock = 170 MHz.
 * 170 MHz / 40 kHz = 4250 timer ticks.
 */
#define PAT_PERIOD_TICKS              4250UL
#define TIMER_CLOCK_MHZ               170L

/* Per-channel phase calibration offsets. */
#define RX1_ZERO_OFFSET_TICKS         0L
#define RX2_ZERO_OFFSET_TICKS         0L
#define RX3_ZERO_OFFSET_TICKS         0L
#define RX4_ZERO_OFFSET_TICKS         0L

#define REPORT_INTERVAL_MS            200UL
#define ADC_CAPTURE_TIMEOUT_MS        50UL

#define ADC_VREF_MV                   3300UL
#define ADC_MAX_COUNTS                4095UL

/*
 * ADC1 and ADC2 each scan two ranks continuously.
 * The DMA stream therefore contains:
 *
 *   ADC1: CH1, CH2, CH1, CH2, ...
 *   ADC2: CH3, CH4, CH3, CH4, ...
 *
 * 2048 points per receiver channel gives a useful waveform block while
 * staying comfortably inside the STM32G431KB SRAM budget.
 */
#define ADC_SAMPLES_PER_CHANNEL       2048U
#define ADC_CHANNELS_PER_ADC          2U
#define ADC_DMA_LENGTH                \
    (ADC_SAMPLES_PER_CHANNEL * ADC_CHANNELS_PER_ADC)

/* ADC rail checks. */
#define ADC_CLIP_LOW_COUNTS           16U
#define ADC_CLIP_HIGH_COUNTS          4079U

/*
 * Schematic divider before each CHx_TO_ADC buffer:
 *
 *   amplifier output -- 10k -- ADC-buffer node -- 15k -- GND
 *
 * Vbuffer = Vamp * 15 / (10 + 15) = Vamp * 3/5
 * Vamp    = Vbuffer * 5/3
 */
#define ADC_DIV_RESTORE_NUMERATOR     5UL
#define ADC_DIV_RESTORE_DENOMINATOR   3UL

/*
 * HC10T-40TR-P approximate receive sensitivity:
 * -75 dBV/uBar ~= 1.78 mV/Pa = 1780 uV/Pa.
 */
#define RX_SENS_UV_PER_PA             1780UL

/* MCP6022 non-inverting amplifier gain: 1 + 22k/10k = 3.2. */
#define AMP_GAIN_X1000                3200UL

/* Optional validity checks. */
#define MIN_VALID_BUFFER_RMS_MV       5UL
#define MIN_VALID_CAPTURE_DELTA       7000UL
#define MAX_VALID_CAPTURE_DELTA       9000UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static uint16_t adc1_dma_buffer[ADC_DMA_LENGTH]
    __attribute__((aligned(4)));

static uint16_t adc2_dma_buffer[ADC_DMA_LENGTH]
    __attribute__((aligned(4)));

static volatile uint8_t adc1_capture_done = 0U;
static volatile uint8_t adc2_capture_done = 0U;
static volatile uint8_t adc1_capture_error = 0U;
static volatile uint8_t adc2_capture_error = 0U;

/* TIM3_CH1..CH4 capture state for RX1..RX4. */
static volatile uint32_t rx_raw_ticks[RX_CHANNEL_COUNT] = {0U};
static volatile uint32_t rx_capture_count[RX_CHANNEL_COUNT] = {0U};
static volatile uint8_t rx_new_data[RX_CHANNEL_COUNT] = {0U};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static uint8_t capture_adc_block(void);

static uint8_t analyse_interleaved_channel(
    const uint16_t *buffer,
    uint32_t buffer_length,
    uint32_t channel_offset,
    RxAmplitudeResult *result);

static void snapshot_phase_state(
    RxPhaseResult phase[RX_CHANNEL_COUNT],
    uint32_t previous_count[RX_CHANNEL_COUNT]);

static uint64_t integer_sqrt_u64(uint64_t value);
static int32_t wrap_to_signed_period(int32_t ticks);
static int32_t phase_zero_offset_ticks(uint32_t channel_index);

static void print_channel_report(
    uint32_t channel_index,
    const RxAmplitudeResult *amplitude,
    uint8_t amplitude_ok,
    const RxPhaseResult *phase);

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */

  /*
   * Verbose startup text temporarily disabled.
   * Keep UART output limited to CHx amplitude + phase.
   */
#if 0
  const char boot_message[] =
      "\r\n"
      "PAT STM32 4-channel buffered receiver started\r\n"
      "PA8  TIM1_CH1 = 40 kHz transmit/reference PWM\r\n"
      "PA6  TIM3_CH1 = RX1 comparator\r\n"
      "PA7  TIM3_CH2 = RX2 comparator\r\n"
      "PB0  TIM3_CH3 = RX3 comparator\r\n"
      "PB7  TIM3_CH4 = RX4 comparator\r\n"
      "PA0  ADC1_IN1  = CH1_TO_ADC\r\n"
      "PA1  ADC1_IN2  = CH2_TO_ADC\r\n"
      "PA4  ADC2_IN17 = CH3_TO_ADC\r\n"
      "PA5  ADC2_IN13 = CH4_TO_ADC\r\n";

  HAL_UART_Transmit(
      &huart2,
      (uint8_t *)boot_message,
      sizeof(boot_message) - 1U,
      HAL_MAX_DELAY);
#endif

  /* Calibrate both ADCs for single-ended operation. */
  if (HAL_ADCEx_Calibration_Start(
          &hadc1,
          ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(
          &hadc2,
          ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * TIM3 must already be configured in CubeMX as:
   *   Slave Mode = Reset Mode
   *   Trigger    = ITR0 / TIM1_TRGO
   *
   * Start all four receiver captures before starting the 40 kHz PWM.
   */
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * TIM1_CH1 is both the transmitter PWM output and the phase reference.
   * TIM1 TRGO must be configured as Update Event in CubeMX.
   */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* printf("4-channel acquisition active\r\n"); */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_report_ms = 0U;
  uint32_t previous_capture_count[RX_CHANNEL_COUNT] = {0U};

  while (1)
  {
    uint32_t current_ms = HAL_GetTick();

    if ((current_ms - last_report_ms) >= REPORT_INTERVAL_MS)
    {
      last_report_ms = current_ms;

      RxAmplitudeResult amplitude[RX_CHANNEL_COUNT] = {{0}};
      uint8_t amplitude_ok[RX_CHANNEL_COUNT] = {0U};
      RxPhaseResult phase[RX_CHANNEL_COUNT] = {{0}};

      if (capture_adc_block() != 0U)
      {
        amplitude_ok[0] = analyse_interleaved_channel(
            adc1_dma_buffer,
            ADC_DMA_LENGTH,
            0U,
            &amplitude[0]);

        amplitude_ok[1] = analyse_interleaved_channel(
            adc1_dma_buffer,
            ADC_DMA_LENGTH,
            1U,
            &amplitude[1]);

        amplitude_ok[2] = analyse_interleaved_channel(
            adc2_dma_buffer,
            ADC_DMA_LENGTH,
            0U,
            &amplitude[2]);

        amplitude_ok[3] = analyse_interleaved_channel(
            adc2_dma_buffer,
            ADC_DMA_LENGTH,
            1U,
            &amplitude[3]);
      }

      snapshot_phase_state(
          phase,
          previous_capture_count);

      for (uint32_t channel = 0U;
           channel < RX_CHANNEL_COUNT;
           channel++)
      {
        /* Require a non-clipped, non-trivial waveform for phase validity. */
        uint8_t amplitude_valid =
            ((amplitude_ok[channel] != 0U)
             && (amplitude[channel].clipping == 0U)
             && (amplitude[channel].buffer_rms_mV
                 >= MIN_VALID_BUFFER_RMS_MV))
                ? 1U
                : 0U;

        phase[channel].phase_valid =
            ((phase[channel].has_capture != 0U)
             && (phase[channel].raw_ticks < PAT_PERIOD_TICKS)
             && (phase[channel].edge_rate_valid != 0U)
             && (amplitude_valid != 0U))
                ? 1U
                : 0U;

        print_channel_report(
            channel,
            &amplitude[channel],
            amplitude_ok[channel],
            &phase[channel]);
      }
    }

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

/**
  * @brief TIM3 input-capture callback for RX1..RX4.
  */
void HAL_TIM_IC_CaptureCallback(
    TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3)
  {
    return;
  }

  uint32_t index;
  uint32_t channel;

  switch (htim->Channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      index = 0U;
      channel = TIM_CHANNEL_1;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_2:
      index = 1U;
      channel = TIM_CHANNEL_2;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      index = 2U;
      channel = TIM_CHANNEL_3;
      break;

    case HAL_TIM_ACTIVE_CHANNEL_4:
      index = 3U;
      channel = TIM_CHANNEL_4;
      break;

    default:
      return;
  }

  rx_raw_ticks[index] =
      HAL_TIM_ReadCapturedValue(
          htim,
          channel);

  rx_capture_count[index]++;
  rx_new_data[index] = 1U;
}

/**
  * @brief ADC DMA conversion-complete callback.
  */
void HAL_ADC_ConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc1_capture_done = 1U;
  }
  else if (hadc->Instance == ADC2)
  {
    adc2_capture_done = 1U;
  }
}

/**
  * @brief ADC DMA error callback.
  */
void HAL_ADC_ErrorCallback(
    ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc1_capture_error = 1U;
  }
  else if (hadc->Instance == ADC2)
  {
    adc2_capture_error = 1U;
  }
}

/**
  * @brief Capture one four-channel waveform block using ADC1 + ADC2 DMA.
  * @retval 1 on success, 0 on timeout or HAL error.
  */
static uint8_t capture_adc_block(void)
{
  adc1_capture_done = 0U;
  adc2_capture_done = 0U;
  adc1_capture_error = 0U;
  adc2_capture_error = 0U;

  /* Make repeated captures deterministic. */
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);

  /*
   * Start ADC2 first, then ADC1.  Exact alignment is not required for the
   * RMS calculation; phase comes from TIM3 comparator captures.
   */
  if (HAL_ADC_Start_DMA(
          &hadc2,
          (uint32_t *)adc2_dma_buffer,
          ADC_DMA_LENGTH) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_Start_DMA(
          &hadc1,
          (uint32_t *)adc1_dma_buffer,
          ADC_DMA_LENGTH) != HAL_OK)
  {
    HAL_ADC_Stop_DMA(&hadc2);
    return 0U;
  }

  uint32_t start_ms = HAL_GetTick();

  while (((adc1_capture_done == 0U)
          || (adc2_capture_done == 0U))
         && (adc1_capture_error == 0U)
         && (adc2_capture_error == 0U))
  {
    if ((HAL_GetTick() - start_ms)
        >= ADC_CAPTURE_TIMEOUT_MS)
    {
      HAL_ADC_Stop_DMA(&hadc1);
      HAL_ADC_Stop_DMA(&hadc2);
      return 0U;
    }
  }

  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);

  if ((adc1_capture_error != 0U)
      || (adc2_capture_error != 0U)
      || (adc1_capture_done == 0U)
      || (adc2_capture_done == 0U))
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief Analyse one channel inside a two-rank interleaved DMA buffer.
  *
  * The DC bias is removed mathematically before AC RMS is calculated.
  * channel_offset must be 0 or 1.
  */
static uint8_t analyse_interleaved_channel(
    const uint16_t *buffer,
    uint32_t buffer_length,
    uint32_t channel_offset,
    RxAmplitudeResult *result)
{
  if ((buffer == NULL)
      || (result == NULL)
      || (buffer_length < 2U)
      || (channel_offset >= ADC_CHANNELS_PER_ADC))
  {
    return 0U;
  }

  uint32_t count = 0U;
  uint64_t sum = 0ULL;
  uint64_t sum_squared = 0ULL;
  uint16_t sample_min = (uint16_t)ADC_MAX_COUNTS;
  uint16_t sample_max = 0U;

  for (uint32_t i = channel_offset;
       i < buffer_length;
       i += ADC_CHANNELS_PER_ADC)
  {
    uint32_t sample = buffer[i];

    sum += sample;
    sum_squared +=
        (uint64_t)sample * (uint64_t)sample;
    count++;

    if (sample < sample_min)
    {
      sample_min = (uint16_t)sample;
    }

    if (sample > sample_max)
    {
      sample_max = (uint16_t)sample;
    }
  }

  if (count == 0U)
  {
    return 0U;
  }

  uint64_t count_u64 = (uint64_t)count;

  /*
   * Centered energy:
   *   E = N*sum(x^2) - sum(x)^2
   *   RMS_counts = sqrt(E) / N
   *
   * x1000 keeps sub-count precision without floating point.
   */
  uint64_t centered_energy =
      count_u64 * sum_squared
      - sum * sum;

  uint64_t rms_counts_x1000 =
      integer_sqrt_u64(
          centered_energy * 1000000ULL)
      / count_u64;

  uint64_t buffer_dc_mV =
      sum * ADC_VREF_MV
      / (count_u64 * ADC_MAX_COUNTS);

  uint64_t buffer_rms_mV =
      rms_counts_x1000 * ADC_VREF_MV
      / (ADC_MAX_COUNTS * 1000ULL);

  uint64_t buffer_vpp_mV =
      (uint64_t)(sample_max - sample_min)
      * ADC_VREF_MV
      / ADC_MAX_COUNTS;

  uint64_t amp_rms_mV =
      buffer_rms_mV
      * ADC_DIV_RESTORE_NUMERATOR
      / ADC_DIV_RESTORE_DENOMINATOR;

  uint64_t amp_vpp_mV =
      buffer_vpp_mV
      * ADC_DIV_RESTORE_NUMERATOR
      / ADC_DIV_RESTORE_DENOMINATOR;

  result->adc_min = sample_min;
  result->adc_max = sample_max;
  result->buffer_dc_mV = (uint32_t)buffer_dc_mV;
  result->buffer_rms_mV = (uint32_t)buffer_rms_mV;
  result->buffer_vpp_mV = (uint32_t)buffer_vpp_mV;
  result->amp_rms_mV = (uint32_t)amp_rms_mV;
  result->amp_vpp_mV = (uint32_t)amp_vpp_mV;

  result->clipping =
      ((sample_min <= ADC_CLIP_LOW_COUNTS)
       || (sample_max >= ADC_CLIP_HIGH_COUNTS))
          ? 1U
          : 0U;

  /*
   * Approximate acoustic pressure from the restored amplifier-output RMS.
   * Result is centi-Pascal (100 cPa = 1 Pa).
   */
  uint64_t pressure_denominator =
      (uint64_t)RX_SENS_UV_PER_PA
      * (uint64_t)AMP_GAIN_X1000;

  if (pressure_denominator != 0ULL)
  {
    result->pressure_cPa =
        (uint32_t)(
            amp_rms_mV * 100000000ULL
            / pressure_denominator);
  }
  else
  {
    result->pressure_cPa = 0U;
  }

  return 1U;
}

/**
  * @brief Atomically snapshot all four phase channels and calculate phase.
  */
static void snapshot_phase_state(
    RxPhaseResult phase[RX_CHANNEL_COUNT],
    uint32_t previous_count[RX_CHANNEL_COUNT])
{
  uint32_t ticks_local[RX_CHANNEL_COUNT];
  uint32_t count_local[RX_CHANNEL_COUNT];
  uint8_t new_data_local[RX_CHANNEL_COUNT];

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t channel = 0U;
       channel < RX_CHANNEL_COUNT;
       channel++)
  {
    ticks_local[channel] = rx_raw_ticks[channel];
    count_local[channel] = rx_capture_count[channel];
    new_data_local[channel] = rx_new_data[channel];
    rx_new_data[channel] = 0U;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  for (uint32_t channel = 0U;
       channel < RX_CHANNEL_COUNT;
       channel++)
  {
    phase[channel].raw_ticks = ticks_local[channel];
    phase[channel].capture_count = count_local[channel];
    phase[channel].capture_delta =
        count_local[channel] - previous_count[channel];
    previous_count[channel] = count_local[channel];
    phase[channel].has_capture = new_data_local[channel];

    phase[channel].edge_rate_valid =
        ((phase[channel].capture_delta >= MIN_VALID_CAPTURE_DELTA)
         && (phase[channel].capture_delta <= MAX_VALID_CAPTURE_DELTA))
            ? 1U
            : 0U;

    phase[channel].corrected_ticks = 0L;
    phase[channel].delay_ns = 0L;
    phase[channel].phase_mdeg = 0L;
    phase[channel].phase_valid = 0U;

    if ((phase[channel].has_capture != 0U)
        && (phase[channel].raw_ticks < PAT_PERIOD_TICKS))
    {
      phase[channel].corrected_ticks =
          wrap_to_signed_period(
              (int32_t)phase[channel].raw_ticks
              - phase_zero_offset_ticks(channel));

      phase[channel].phase_mdeg =
          (phase[channel].corrected_ticks * 360000L)
          / (int32_t)PAT_PERIOD_TICKS;

      phase[channel].delay_ns =
          (phase[channel].corrected_ticks * 1000L)
          / TIMER_CLOCK_MHZ;
    }
  }
}

/**
  * @brief Print one compact UART report line.
  */
static void print_channel_report(
    uint32_t channel_index,
    const RxAmplitudeResult *amplitude,
    uint8_t amplitude_ok,
    const RxPhaseResult *phase)
{
  char message[96];
  int length;

  if ((amplitude == NULL) || (phase == NULL))
  {
    return;
  }

  /*
   * Minimal UART report for now:
   *   CHx,amplitude=<amplifier-output RMS mV>,phase=<degrees>
   *
   * amp_rms_mV is restored to the signal level before the 10k/15k divider.
   */
  if (amplitude_ok == 0U)
  {
    length = snprintf(
        message,
        sizeof(message),
        "CH%lu,amplitude=adc_error,phase=invalid\r\n",
        (unsigned long)(channel_index + 1U));
  }
  else if (phase->phase_valid == 0U)
  {
    length = snprintf(
        message,
        sizeof(message),
        "CH%lu,amplitude=%lu mV RMS,phase=invalid\r\n",
        (unsigned long)(channel_index + 1U),
        (unsigned long)amplitude->amp_rms_mV);
  }
  else
  {
    int32_t phase_absolute = phase->phase_mdeg;
    const char *phase_sign = "";

    if (phase_absolute < 0L)
    {
      phase_absolute = -phase_absolute;
      phase_sign = "-";
    }

    length = snprintf(
        message,
        sizeof(message),
        "CH%lu,amplitude=%lu mV RMS,phase=%s%ld.%03ld deg\r\n",
        (unsigned long)(channel_index + 1U),
        (unsigned long)amplitude->amp_rms_mV,
        phase_sign,
        (long)(phase_absolute / 1000L),
        (long)(phase_absolute % 1000L));
  }

  if (length > 0)
  {
    if (length > (int)(sizeof(message) - 1U))
    {
      length = (int)(sizeof(message) - 1U);
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)message,
        (uint16_t)length,
        100U);
  }

#if 0
  /*
   * Verbose UART fields temporarily disabled:
   * bufDC, bufRMS, bufVpp, ampVpp, p_rms,
   * adcMin, adcMax, clip, rawTicks, correctedTicks, dt,
   * phaseValid, capCount and capDelta.
   */
#endif
}

/**
  * @brief Return the calibration offset for one receiver channel.
  */
static int32_t phase_zero_offset_ticks(uint32_t channel_index)
{
  switch (channel_index)
  {
    case 0U:
      return RX1_ZERO_OFFSET_TICKS;

    case 1U:
      return RX2_ZERO_OFFSET_TICKS;

    case 2U:
      return RX3_ZERO_OFFSET_TICKS;

    case 3U:
      return RX4_ZERO_OFFSET_TICKS;

    default:
      return 0L;
  }
}

/**
  * @brief Integer square root for a 64-bit unsigned value.
  */
static uint64_t integer_sqrt_u64(uint64_t value)
{
  uint64_t result = 0ULL;
  uint64_t bit = 1ULL << 62;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0ULL)
  {
    if (value >= result + bit)
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }

    bit >>= 2;
  }

  return result;
}

/**
  * @brief Wrap timer ticks to approximately -180 to +180 degrees.
  */
static int32_t wrap_to_signed_period(int32_t ticks)
{
  while (ticks >= (int32_t)(PAT_PERIOD_TICKS / 2UL))
  {
    ticks -= (int32_t)PAT_PERIOD_TICKS;
  }

  while (ticks < -(int32_t)(PAT_PERIOD_TICKS / 2UL))
  {
    ticks += (int32_t)PAT_PERIOD_TICKS;
  }

  return ticks;
}

/**
  * @brief Redirect printf to USART2.
  */
int _write(
    int file,
    char *pointer,
    int length)
{
  (void)file;

  HAL_UART_Transmit(
      &huart2,
      (uint8_t *)pointer,
      (uint16_t)length,
      HAL_MAX_DELAY);

  return length;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  * @param  line: source line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */