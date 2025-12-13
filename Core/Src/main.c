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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_pwm-buzzer.h"
#include "driver_scd4x.h"
#include "driver_scd4x_debug.h"
#include "driver_scd4x_interface.h"
#include "driver_sen5x.h"
#include "driver_sen5x_debug.h"
#include "driver_sen5x_interface.h"
#include "driver_serial.h"
#include "oled_graph.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
I2C_HandleTypeDef hi2c4;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static scd4x_handle_t scd4x_handle; // Initialized in SCD4x_Init()
static sen5x_handle_t sen5x_handle; // Initialized in SEN5X_Init()

static const Buzzer_Note alert_melody[] = {
    {NOTE_C4, 166}, {NOTE_A4, 166}, {NOTE_F4, 166}, {0, 100}, // 0 = rest
    {0xFFFF, 0},                                              // terminator
};

/* === Runtime state (UI + sampling) === */
typedef struct {
  uint16_t co2_ppm;
  int16_t co2_delta_ppm;

  float pm1_ug_m3;
  float pm2p5_ug_m3;
  float pm4_ug_m3;
  float pm10_ug_m3;

  float voc_index;
  float nox_index;
} air_data_t;

static air_data_t g_air = {0};

typedef enum {
  BTN_EVENT_NONE = 0,
  BTN_EVENT_SHORT = 1,
  BTN_EVENT_LONG = 2,
} btn_event_t;

static volatile btn_event_t g_btn_event = BTN_EVENT_NONE;
static volatile uint8_t g_btn_pressed = 0U;
static volatile uint32_t g_btn_press_ms = 0U;
static volatile uint32_t g_btn_last_edge_ms = 0U;

static volatile uint8_t g_in_low_power = 0U;
static volatile uint8_t g_btn_wake = 0U;

/* schedules */
static uint32_t g_next_oled_ms = 0U;
static uint32_t g_next_legend_ms = 0U;
static uint32_t g_next_sample_ms = 0U;

static uint8_t g_active_item = 0U;

/* sampling pending flags */
static uint8_t g_scd_pending = 0U;
static uint8_t g_sen_pending = 0U;
static uint32_t g_scd_start_ms = 0U;
static uint32_t g_sen_start_ms = 0U;

static uint8_t g_serial_dirty = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void SCD4x_Init(scd4x_handle_t *scd4x_handle_p);
static void SEN5X_Init(sen5x_handle_t *sen5x_handle_p);
static void run_debug_tests(void);
static void ssd1306_DisplayDebugMode(void);
bool poll_is_debug_mode(void);

static uint32_t time_elapsed_ms(uint32_t now, uint32_t then);
static float normalize_for_graph(pollutant_id_t id, float value);
static aq_level_t aq_level_for(pollutant_id_t id, float value);
static void leds_set_level(aq_level_t level);

static uint8_t sensors_are_oneshot(void);
static void sensors_start_if_needed(uint32_t now);
static void sensors_stop_if_oneshot(void);
static void poll_scd4x(uint32_t now);
static void poll_sen5x(uint32_t now);

static void advance_legend(void);
static pollutant_id_t active_pollutant(void);

static void display_group_co2(void);
static void display_group_voc_nox(pollutant_id_t active);
static void display_group_pm1_pm25(pollutant_id_t active);
static void display_group_pm4_pm10(pollutant_id_t active);

static void serial_print_latest(pollutant_id_t active);
static void enter_low_power_mode(void);
static void ssd1306_DisplayCenteredMessage_11x18(const char *msg);

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  bool in_debug_mode = poll_is_debug_mode(); //

  if (in_debug_mode) {
    // Buzzer and LED blinking sequence
    Buzzer_PlayMelody(alert_melody);
    for (int8_t ctr = 0; ctr < 6; ++ctr) {
      HAL_GPIO_TogglePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin);
      HAL_Delay(33);
      HAL_GPIO_TogglePin(LED_MID_GPIO_Port, LED_MID_Pin);
      HAL_Delay(33);
      HAL_GPIO_TogglePin(LED_LOW_GPIO_Port, LED_LOW_Pin);
      HAL_Delay(33);
    }
  }

  ssd1306_Init();

  // Loading Screen 1 & Scrolling Project Name and Version

  serial_print("\n===== Initializing Hardware =====\n");
  ssd1306_DisplayCenteredMessage_11x18("LOADING"); // Leave some space for dots
  ssd1306_UpdateScreen();

  serial_print("Project Name: " PROJECT_NAME "\n");
  serial_print("Project Version: " PROJECT_VERSION "\n");
  
  char project_name_and_version_msg[] = PROJECT_NAME_AND_VERSION_PADDED;
  for (uint8_t i = 0; i < 20; i++) {
    ssd1306_SetCursor(0, SSD1306_HEIGHT - Font_7x10.height);
    ssd1306_WriteString(project_name_and_version_msg, Font_7x10, White);
    ssd1306_UpdateScreen();

    char ch = project_name_and_version_msg[0];
    memmove(project_name_and_version_msg, project_name_and_version_msg + 1,
            sizeof(project_name_and_version_msg) - 2);
    project_name_and_version_msg[sizeof(project_name_and_version_msg) - 2] = ch;
  }

  // Initialize SCD4X
  SCD4x_Init(&scd4x_handle);
  SEN5X_Init(&sen5x_handle);

  if (in_debug_mode) {
    ssd1306_DisplayDebugMode();
    serial_print("DEBUG MODE: Running peripheral tests...\n");
    while (1) {
      run_debug_tests();
      ssd1306_DisplayDebugMode();
      HAL_Delay(1000);
    }
  }

  uint32_t now = HAL_GetTick();
  g_active_item = 0U;
  g_next_oled_ms = now;
  g_next_legend_ms = now + UI_LEGEND_STEP_MS;
  g_next_sample_ms = now; /* sample ASAP */

  if (!sensors_are_oneshot()) {
    (void)scd4x_start_periodic_measurement(&scd4x_handle);
    (void)sen5x_start_measurement(&sen5x_handle);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* --- handle button events --- */
    btn_event_t ev = g_btn_event;
    if (ev != BTN_EVENT_NONE) {
      __disable_irq();
      g_btn_event = BTN_EVENT_NONE;
      __enable_irq();

      if (ev == BTN_EVENT_SHORT) {
        advance_legend();
        g_next_legend_ms = now + UI_LEGEND_STEP_MS;
      } else if (ev == BTN_EVENT_LONG) {
        enter_low_power_mode();
        now = HAL_GetTick();
      }
    }

    /* --- auto-cycle legend/page --- */
    if (now >= g_next_legend_ms) {
      advance_legend();
      g_next_legend_ms = now + UI_LEGEND_STEP_MS;
    }

    /* --- trigger sampling --- */
    if (now >= g_next_sample_ms) {
      g_next_sample_ms = now + AIR_SENSOR_SAMPLE_INTERVAL_MS;

      g_scd_pending = 1U;
      g_sen_pending = 1U;

      /* IMPORTANT:
       * - In one-shot mode (>30s), keep start_ms == 0 so
       * sensors_start_if_needed() can start the sensors.
       * - In continuous mode, refresh the timeout base EVERY sample, otherwise
       * it goes stale and you timeout later.
       */
      if (!sensors_are_oneshot()) {
        g_scd_start_ms = now;
        g_sen_start_ms = now;
      }

      /* in continuous mode, these are just timeout bases */
      if (g_scd_start_ms == 0U)
        g_scd_start_ms = now;
      if (g_sen_start_ms == 0U)
        g_sen_start_ms = now;
    }

    /* oneshot start (if interval > 30s) */
    sensors_start_if_needed(now);

    /* poll sensors (non-blocking) */
    poll_scd4x(now);
    poll_sen5x(now);
    sensors_stop_if_oneshot();

    /* LEDs follow the active pollutant */
    pollutant_id_t active = active_pollutant();
    float active_value = 0.0f;

    switch (active) {
    case POL_CO2_DELTA:
      active_value = (float)g_air.co2_delta_ppm;
      break;
    case POL_VOC_INDEX:
      active_value = g_air.voc_index;
      break;
    case POL_NOX_INDEX:
      active_value = g_air.nox_index;
      break;
    case POL_PM1_UGM3:
      active_value = g_air.pm1_ug_m3;
      break;
    case POL_PM2P5_UGM3:
      active_value = g_air.pm2p5_ug_m3;
      break;
    case POL_PM4_UGM3:
      active_value = g_air.pm4_ug_m3;
      break;
    case POL_PM10_UGM3:
      active_value = g_air.pm10_ug_m3;
      break;
    default:
      active_value = 0.0f;
      break;
    }
    leds_set_level(aq_level_for(active, active_value));

    /* serial output after new measurements */
    if (g_serial_dirty) {
      g_serial_dirty = 0U;
      serial_print_latest(active);
    }

    /* OLED refresh (drives blinking) */
    if (now >= g_next_oled_ms) {
      g_next_oled_ms = now + UI_OLED_REFRESH_MS;

      ssd1306_Fill(Black);

      switch (active) {
      case POL_CO2_DELTA:
        display_group_co2();
        break;

      case POL_VOC_INDEX:
      case POL_NOX_INDEX:
        display_group_voc_nox(active);
        break;

      case POL_PM1_UGM3:
      case POL_PM2P5_UGM3:
        display_group_pm1_pm25(active);
        break;

      case POL_PM4_UGM3:
      case POL_PM10_UGM3:
        display_group_pm4_pm10(active);
        break;

      default:
        break;
      }

      ssd1306_UpdateScreen();
    }

    /* give CPU a chance to sleep between ticks/interrupts */
    __WFI();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) !=
      HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 |
                                     RCC_OSCILLATORTYPE_HSI |
                                     RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void) {

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x30A175AB;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

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
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_LOW_Pin | LED_MID_Pin | LED_HIGH_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_1_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LOW_Pin LED_MID_Pin LED_HIGH_Pin */
  GPIO_InitStruct.Pin = LED_LOW_Pin | LED_MID_Pin | LED_HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SCD4x_Init(scd4x_handle_t *scd4x_handle_p) {
  uint8_t res;

  /* Link the interface functions */
  DRIVER_SCD4X_LINK_INIT(scd4x_handle_p, scd4x_handle_t);
  DRIVER_SCD4X_LINK_IIC_INIT(scd4x_handle_p, scd4x_interface_iic_init);
  DRIVER_SCD4X_LINK_IIC_DEINIT(scd4x_handle_p, scd4x_interface_iic_deinit);
  DRIVER_SCD4X_LINK_IIC_WRITE_COMMAND(scd4x_handle_p,
                                      scd4x_interface_iic_write_cmd);
  DRIVER_SCD4X_LINK_IIC_READ_COMMAND(scd4x_handle_p,
                                     scd4x_interface_iic_read_cmd);
  DRIVER_SCD4X_LINK_DELAY_MS(scd4x_handle_p, scd4x_interface_delay_ms);
  DRIVER_SCD4X_LINK_DEBUG_PRINT(scd4x_handle_p, scd4x_interface_debug_print);

  (void)scd4x_set_type(scd4x_handle_p, SCD41);

  /* Initialize the chip (calls iic_init internally) */
  res = scd4x_init(scd4x_handle_p);
  if (res != 0) {
    scd4x_interface_debug_print("scd4x_init failed: %u\n", res);
    Error_Handler();
  }
}

static void SEN5X_Init(sen5x_handle_t *sen5x_handle_p) {
  uint8_t res;

  /* Link the interface functions */
  DRIVER_SEN5X_LINK_INIT(sen5x_handle_p, sen5x_handle_t);
  DRIVER_SEN5X_LINK_IIC_INIT(sen5x_handle_p, sen5x_interface_iic_init);
  DRIVER_SEN5X_LINK_IIC_DEINIT(sen5x_handle_p, sen5x_interface_iic_deinit);
  DRIVER_SEN5X_LINK_IIC_WRITE_COMMAND(sen5x_handle_p,
                                      sen5x_interface_iic_write_cmd);
  DRIVER_SEN5X_LINK_IIC_READ_COMMAND(sen5x_handle_p,
                                     sen5x_interface_iic_read_cmd);
  DRIVER_SEN5X_LINK_DELAY_MS(sen5x_handle_p, sen5x_interface_delay_ms);
  DRIVER_SEN5X_LINK_DEBUG_PRINT(sen5x_handle_p, sen5x_interface_debug_print);

  (void)sen5x_set_type(sen5x_handle_p, SEN55);

  /* Initialize the chip (calls iic_init internally) */
  res = sen5x_init(sen5x_handle_p);
  if (res != 0) {
    sen5x_interface_debug_print("sen5x_init failed: %u\n", res);
    Error_Handler();
  }
}

// This function will run various debug tests for different components
static void run_debug_tests(void) {
  // Call SEN5X debug test (once because uses onboard EEPROM)
  sen5x_run_full_test_once();

  // Call SCD4x debug test (once because uses onboard EEPROM)
  scd4x_run_full_test_once();

  // Call SSD1305 debug test
  ssd1306_TestAll();
}

// Poll for 1s for user pressing button
bool poll_is_debug_mode() {
  uint32_t startup_time = HAL_GetTick();
  while (HAL_GetTick() - startup_time < 1000) {
    if (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET) {
      // Button is pressed during startup window
      uint32_t btn_press_start = HAL_GetTick();
      while (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - btn_press_start >= 100) {
          return true;
        }
        HAL_Delay(10); // Small delay to prevent tight loop
      }
      break;
    }
    HAL_Delay(100); // Check periodically
  }
  return false;
}

/* === thresholds (tune these later to your chosen standards) === */
static const pollutant_thresholds_t g_thresh[POL_COUNT] = {
    [POL_CO2_DELTA] = {550.0f, 800.0f, 1350.0f}, /* Δppm */
    [POL_VOC_INDEX] = {100.0f, 150.0f, 500.0f},  /* VOC index */
    [POL_NOX_INDEX] = {1.0f, 10.0f, 50.0f},      /* NOx index */
    [POL_PM1_UGM3] = {5.0f, 10.0f, 20.0f},       /* μg/m3 */
    [POL_PM2P5_UGM3] = {5.0f, 10.0f, 20.0f},     /* μg/m3 */
    [POL_PM4_UGM3] = {15.0f, 20.0f, 50.0f},      /* μg/m3 */
    [POL_PM10_UGM3] = {15.0f, 20.0f, 50.0f},     /* μg/m3 */
};

static uint32_t time_elapsed_ms(uint32_t now, uint32_t then) {
  return (uint32_t)(now - then);
}

static float clampf(float v, float lo, float hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static aq_level_t aq_level_for(pollutant_id_t id, float value) {
  if (value <= g_thresh[id].low_max)
    return AQ_LEVEL_LOW;
  if (value <= g_thresh[id].mid_max)
    return AQ_LEVEL_MID;
  return AQ_LEVEL_HIGH;
}

static float normalize_for_graph(pollutant_id_t id, float value) {
  float maxv = g_thresh[id].graph_max;
  if (maxv <= 0.0f)
    return 0.0f;
  return (clampf(value, 0.0f, maxv) * 100.0f) / maxv;
}

static void leds_set_level(aq_level_t level) {
  HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin,
                    (level == AQ_LEVEL_LOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin,
                    (level == AQ_LEVEL_MID) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin,
                    (level == AQ_LEVEL_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Center big message using Font_11x18 (11x18 px) */
static void ssd1306_DisplayCenteredMessage_11x18(const char *msg) {
  const uint8_t W = 11U;
  const uint8_t H = 18U;
  uint8_t x = (uint8_t)((SSD1306_WIDTH - ((uint32_t)strlen(msg) * W)) / 2U);
  uint8_t y = (uint8_t)((SSD1306_HEIGHT - H) / 2U);
  ssd1306_SetCursor(x, y);
  ssd1306_WriteString(msg, Font_11x18, White);
}

static void fmt_fixed_1(char *dst, size_t dst_sz, float v) {
  if (!isfinite(v)) {
    (void)snprintf(dst, dst_sz, "-");
    return;
  }

  /* 1 decimal without using %f */
  int32_t x10 = (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
  int32_t ip = x10 / 10;
  int32_t fp = x10 % 10;
  if (fp < 0)
    fp = -fp;

  (void)snprintf(dst, dst_sz, "%ld.%01ld", (long)ip, (long)fp);
}

static void fmt_round_int(char *dst, size_t dst_sz, float v) {
  if (!isfinite(v)) {
    (void)snprintf(dst, dst_sz, "-");
    return;
  }
  long iv = (long)(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
  (void)snprintf(dst, dst_sz, "%ld", iv);
}

/* Override your debug splash to reuse the centering helper */
void ssd1306_DisplayDebugMode(void) {
  ssd1306_Fill(Black);
  ssd1306_DisplayCenteredMessage_11x18("DEBUG MODE");
  ssd1306_UpdateScreen();
}

/* legend sequence (drives which “page” is shown + which pollutant drives LEDs)
 */
static const pollutant_id_t kLegendSeq[] = {
    POL_CO2_DELTA,  POL_VOC_INDEX, POL_NOX_INDEX, POL_PM1_UGM3,
    POL_PM2P5_UGM3, POL_PM4_UGM3,  POL_PM10_UGM3,
};

static void advance_legend(void) {
  g_active_item =
      (uint8_t)((g_active_item + 1U) %
                (uint8_t)(sizeof(kLegendSeq) / sizeof(kLegendSeq[0])));
}

static pollutant_id_t active_pollutant(void) {
  return kLegendSeq[g_active_item];
}

/* --- OLED group renderers --- */
static void display_group_co2(void) {
  char l1[24];
  char l2[24];

  snprintf(l1, sizeof(l1), "CO2: %u ppm", (unsigned)g_air.co2_ppm);
  snprintf(l2, sizeof(l2), "dCO2: %d ppm", (int)g_air.co2_delta_ppm);

  /* left hidden (text), right shows CO2 graph */
  graph_plots_1t1b_ex(NULL, &sensor_ring_buffer_1, 0, 0, SSD1306_WIDTH,
                      SSD1306_HEIGHT, 2, "", "CO2", 1, /* blink right */
                      0);

  ssd1306_SetCursor(2, 2);
  ssd1306_WriteString(l1, Font_7x10, White);
  ssd1306_SetCursor(2, 14);
  ssd1306_WriteString(l2, Font_7x10, White);
}

static void display_group_voc_nox(pollutant_id_t active) {
  char l1[24], l2[24];
  char voc_s[29], nox_s[29];

  fmt_round_int(voc_s, sizeof(voc_s), g_air.voc_index);
  fmt_round_int(nox_s, sizeof(nox_s), g_air.nox_index);

  snprintf(l1, sizeof(l1), "VOC: %s", voc_s);
  snprintf(l2, sizeof(l2), "NOx: %s", nox_s);

  graph_plots_1t2b_ex(NULL, &sensor_ring_buffer_2, &sensor_ring_buffer_3, 0, 0,
                      SSD1306_WIDTH, SSD1306_HEIGHT, 2, "", "VOC", "NOx",
                      (active == POL_VOC_INDEX) ? 1 : 2, 0);

  ssd1306_SetCursor(2, 2);
  ssd1306_WriteString(l1, Font_7x10, White);
  ssd1306_SetCursor(2, 14);
  ssd1306_WriteString(l2, Font_7x10, White);
}

static void display_group_pm1_pm25(pollutant_id_t active) {
  char l1[24], l2[24];
  char pm_1[29], pm_25[29];

  fmt_round_int(pm_1, sizeof(pm_1), g_air.pm1_ug_m3);
  fmt_round_int(pm_25, sizeof(pm_25), g_air.pm2p5_ug_m3);

  snprintf(l1, sizeof(l1), "PM1: %s ug/m^3", pm_1);
  snprintf(l2, sizeof(l2), "PM2.5: %s ug/m^3", pm_25);

  graph_plots_1t2b_ex(NULL, &sensor_ring_buffer_4, &sensor_ring_buffer_5, 0, 0,
                      SSD1306_WIDTH, SSD1306_HEIGHT, 2, "", "PM1", "PM2.5",
                      (active == POL_PM1_UGM3) ? 1 : 2, 0);

  ssd1306_SetCursor(2, 2);
  ssd1306_WriteString(l1, Font_7x10, White);
  ssd1306_SetCursor(2, 14);
  ssd1306_WriteString(l2, Font_7x10, White);
}

static void display_group_pm4_pm10(pollutant_id_t active) {
  char l1[24], l2[24];
  char pm_4[29], pm_10[29];

  fmt_round_int(pm_4, sizeof(pm_4), g_air.pm4_ug_m3);
  fmt_round_int(pm_10, sizeof(pm_10), g_air.pm10_ug_m3);

  snprintf(l1, sizeof(l1), "PM4: %s ug/m^3", pm_4);
  snprintf(l2, sizeof(l2), "PM10: %s ug/m^3", pm_10);

  graph_plots_1t2b_ex(NULL, &sensor_ring_buffer_6, &sensor_ring_buffer_7, 0, 0,
                      SSD1306_WIDTH, SSD1306_HEIGHT, 2, "", "PM4", "PM10",
                      (active == POL_PM4_UGM3) ? 1 : 2, 0);

  ssd1306_SetCursor(2, 2);
  ssd1306_WriteString(l1, Font_7x10, White);
  ssd1306_SetCursor(2, 14);
  ssd1306_WriteString(l2, Font_7x10, White);
}

/* --- sensors --- */
static uint8_t sensors_are_oneshot(void) {
  return (AIR_SENSOR_SAMPLE_INTERVAL_S > AIR_SENSOR_STOP_IF_INTERVAL_GT_S) ? 1U
                                                                           : 0U;
}

static void sensors_start_if_needed(uint32_t now) {
  if (!sensors_are_oneshot())
    return;

  if (g_scd_pending && g_scd_start_ms == 0U) {
    if (scd4x_start_periodic_measurement(&scd4x_handle) == 0) {
      g_scd_start_ms = now;
    } else {
      g_scd_pending = 0U;
    }
  }
  if (g_sen_pending && g_sen_start_ms == 0U) {
    if (sen5x_start_measurement(&sen5x_handle) == 0) {
      g_sen_start_ms = now;
    } else {
      g_sen_pending = 0U;
    }
  }
}

static void sensors_stop_if_oneshot(void) {
  if (!sensors_are_oneshot())
    return;

  if (g_scd_pending == 0U && g_scd_start_ms != 0U) {
    (void)scd4x_stop_periodic_measurement(&scd4x_handle);
    g_scd_start_ms = 0U;
  }
  if (g_sen_pending == 0U && g_sen_start_ms != 0U) {
    (void)sen5x_stop_measurement(&sen5x_handle);
    g_sen_start_ms = 0U;
  }
}

static void poll_scd4x(uint32_t now) {
  if (!g_scd_pending)
    return;

  if (g_scd_start_ms != 0U && time_elapsed_ms(now, g_scd_start_ms) > 20000U) {
    serial_print("SCD4x: timeout\n");
    g_scd_pending = 0U;
    g_serial_dirty = 1U;
    return;
  }

  scd4x_bool_t ready = SCD4X_BOOL_FALSE;
  if (scd4x_get_data_ready_status(&scd4x_handle, &ready) != 0)
    return;
  if (ready == SCD4X_BOOL_FALSE)
    return;

  uint16_t co2_raw = 0, co2_ppm = 0;
  uint16_t t_raw = 0, rh_raw = 0;
  float t_c = 0.0f, rh = 0.0f;

  if (scd4x_read(&scd4x_handle, &co2_raw, &co2_ppm, &t_raw, &t_c, &rh_raw,
                 &rh) != 0)
    return;
  if (co2_ppm == 0U && co2_raw != 0U)
    co2_ppm = co2_raw;

  g_air.co2_ppm = co2_ppm;
  int32_t d = (int32_t)co2_ppm - (int32_t)AIR_CO2_OUTDOOR_PPM;
  if (d < 0)
    d = 0;
  g_air.co2_delta_ppm = (int16_t)d;

  ring_buffer_push(
      &sensor_ring_buffer_1,
      normalize_for_graph(POL_CO2_DELTA, (float)g_air.co2_delta_ppm));

  g_scd_pending = 0U;
  g_serial_dirty = 1U;
}

static void poll_sen5x(uint32_t now) {
  if (!g_sen_pending) {
    return;
  }

  /* timeout window starts when measurement was started (one-shot) or when
   * pending was armed (continuous) */
  if (g_sen_start_ms != 0U && time_elapsed_ms(now, g_sen_start_ms) > 20000U) {
    serial_print("SEN5x: timeout (no data-ready)\n");
    g_sen_pending = 0U;
    g_serial_dirty = 1U;
    return;
  }

  sen5x_data_ready_flag_t flag = SEN5X_DATA_READY_FLAG_NOT_READY;
  if (sen5x_read_data_flag(&sen5x_handle, &flag) != 0) {
    return;
  }
  if (flag != SEN5X_DATA_READY_FLAG_AVAILABLE) {
    return;
  }

  /* SEN55: read PM + T/RH + VOC/NOx in one transaction */
  sen55_data_t d = {0};
  if (sen55_read(&sen5x_handle, &d) != 0) {
    return;
  }

  if (d.pm_valid) {
    g_air.pm1_ug_m3 = d.pm1p0_ug_m3;
    g_air.pm2p5_ug_m3 = d.pm2p5_ug_m3;
    g_air.pm4_ug_m3 = d.pm4p0_ug_m3;
    g_air.pm10_ug_m3 = d.pm10_ug_m3;
  }
  g_air.voc_index = d.voc_index;
  g_air.nox_index = d.nox_index;

  ring_buffer_push(&sensor_ring_buffer_2,
                   normalize_for_graph(POL_VOC_INDEX, g_air.voc_index));
  ring_buffer_push(&sensor_ring_buffer_3,
                   normalize_for_graph(POL_NOX_INDEX, g_air.nox_index));
  ring_buffer_push(&sensor_ring_buffer_4,
                   normalize_for_graph(POL_PM1_UGM3, g_air.pm1_ug_m3));
  ring_buffer_push(&sensor_ring_buffer_5,
                   normalize_for_graph(POL_PM2P5_UGM3, g_air.pm2p5_ug_m3));
  ring_buffer_push(&sensor_ring_buffer_6,
                   normalize_for_graph(POL_PM4_UGM3, g_air.pm4_ug_m3));
  ring_buffer_push(&sensor_ring_buffer_7,
                   normalize_for_graph(POL_PM10_UGM3, g_air.pm10_ug_m3));

  g_sen_pending = 0U;
  g_serial_dirty = 1U;
}

static void serial_print_latest(pollutant_id_t active) {
  char pm1_s[12], pm25_s[12], pm4_s[12], pm10_s[12];
  char voc_s[12], nox_s[12];

  fmt_fixed_1(pm1_s, sizeof(pm1_s), g_air.pm1_ug_m3);
  fmt_fixed_1(pm25_s, sizeof(pm25_s), g_air.pm2p5_ug_m3);
  fmt_fixed_1(pm4_s, sizeof(pm4_s), g_air.pm4_ug_m3);
  fmt_fixed_1(pm10_s, sizeof(pm10_s), g_air.pm10_ug_m3);

  fmt_round_int(voc_s, sizeof(voc_s), g_air.voc_index);
  fmt_round_int(nox_s, sizeof(nox_s), g_air.nox_index);

  serial_print("AIR: CO2=%u ppm (d=%d), PM1=%s, PM2.5=%s, PM4=%s, PM10=%s, "
               "VOC=%s, NOx=%s | active=%u\n",
               (unsigned)g_air.co2_ppm, (int)g_air.co2_delta_ppm, pm1_s, pm25_s,
               pm4_s, pm10_s, voc_s, nox_s, (unsigned)active);
}

static void enter_low_power_mode(void) {
  g_in_low_power = 1U;

  ssd1306_Fill(Black);
  ssd1306_DisplayCenteredMessage_11x18("LOW POWER");
  ssd1306_UpdateScreen();
  HAL_Delay(1000);

  (void)scd4x_stop_periodic_measurement(&scd4x_handle);
  (void)sen5x_stop_measurement(&sen5x_handle);

  leds_set_level(AQ_LEVEL_LOW);
  HAL_GPIO_WritePin(LED_LOW_GPIO_Port, LED_LOW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_MID_GPIO_Port, LED_MID_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_HIGH_GPIO_Port, LED_HIGH_Pin, GPIO_PIN_RESET);

  ssd1306_SetDisplayOn(0);

  HAL_SuspendTick();
  g_btn_wake = 0U;

  while (g_btn_wake == 0U) {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }

  HAL_ResumeTick();

  ssd1306_SetDisplayOn(1);
  ssd1306_Init();

  g_in_low_power = 0U;

  uint32_t now = HAL_GetTick();
  g_next_oled_ms = now;
  g_next_legend_ms = now + UI_LEGEND_STEP_MS;
  g_next_sample_ms = now;

  if (!sensors_are_oneshot()) {
    (void)scd4x_start_periodic_measurement(&scd4x_handle);
    (void)sen5x_start_measurement(&sen5x_handle);
  }
}

/* Button EXTI: short press = next page; long press = low power */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != BTN_1_Pin)
    return;

  if (g_in_low_power) {
    g_btn_wake = 1U;
    return;
  }

  uint32_t now = HAL_GetTick();
  if (time_elapsed_ms(now, g_btn_last_edge_ms) < BTN_DEBOUNCE_MS)
    return;
  g_btn_last_edge_ms = now;

  GPIO_PinState st = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin);

  if (st == GPIO_PIN_SET) {
    g_btn_pressed = 1U;
    g_btn_press_ms = now;
  } else {
    if (g_btn_pressed) {
      g_btn_pressed = 0U;
      uint32_t dur = time_elapsed_ms(now, g_btn_press_ms);
      g_btn_event =
          (dur >= BTN_LONG_PRESS_MS) ? BTN_EVENT_LONG : BTN_EVENT_SHORT;
    }
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
   number, ex: printf("Wrong parameters value: file %s on line %d\n", file,
   line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
