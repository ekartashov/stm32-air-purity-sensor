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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver_serial.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include "driver_pwm-buzzer.h"
#include "oled_graph.h"
#include "driver_scd4x.h"
#include "driver_scd4x_interface.h"
#include "driver_scd4x_debug.h"
#include "driver_sen5x.h"
#include "driver_sen5x_interface.h"
#include "driver_sen5x_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROJECT_NAME "Air Purity Sensor"
#define PROJECT_VERSION "2.0.0"
#define PROJECT_NAME_AND_VERSION_PADDED "    " PROJECT_NAME " VERSION: " PROJECT_VERSION "  "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static scd4x_handle_t scd4x_handle; // Initialized in SCD4x_Init()
static sen5x_handle_t sen5x_handle; // Initialized in SEN5X_Init()

static const Buzzer_Note alert_melody[] = {
    { NOTE_C4, 166 },
    { NOTE_A4, 166 },
    { NOTE_F4, 166 },
    { 0,       100 },  // 0 = rest
    { 0xFFFF,  0 },    // terminator
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void SCD4x_Init(scd4x_handle_t *scd4x_handle_p);
static void SEN5X_Init(sen5x_handle_t *sen5x_handle_p);
static void run_debug_tests(void);
static void ssd1306_DisplayDebugMode(void);
bool poll_is_debug_mode(void);
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
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  bool in_debug_mode = poll_is_debug_mode(); //
  
  if (in_debug_mode)
  {
    // Buzzer and LED blinking sequence
    Buzzer_PlayMelody(alert_melody);
    for (int8_t ctr = 0; ctr < 6; ++ctr){
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
  char loading_screen_msg[] = "LOADING .";
  serial_print("\r\n===== Initializing Hardware =====\r\n");
  uint8_t loading_screen_msg_offset_x = (strlen(loading_screen_msg) + 2) * Font_11x18.width;
  uint8_t loading_screen_msg_x = (uint8_t)((SSD1306_WIDTH - loading_screen_msg_offset_x) / 2); // Leave some space for loading dots
  uint8_t loading_screen_msg_y = (uint8_t)((SSD1306_HEIGHT - Font_11x18.height) / 2);
  ssd1306_SetCursor(loading_screen_msg_x, loading_screen_msg_y);
  ssd1306_WriteString(loading_screen_msg, Font_11x18, White);
  ssd1306_UpdateScreen();

  serial_print("Project Name: " PROJECT_NAME "\n");
  serial_print("Project Version: " PROJECT_VERSION "\n");
  char project_name_and_version_msg[] = PROJECT_NAME_AND_VERSION_PADDED;
  for(uint8_t i = 0; i < sizeof(project_name_and_version_msg) / sizeof(char); i++) {
    ssd1306_SetCursor(loading_screen_msg_x, loading_screen_msg_y);
    ssd1306_WriteString(loading_screen_msg, Font_11x18, White);
    ssd1306_SetCursor(0, SSD1306_HEIGHT - Font_6x8.height);
    ssd1306_WriteString(project_name_and_version_msg, Font_6x8, White);
    ssd1306_UpdateScreen();
    
    memmove(project_name_and_version_msg, project_name_and_version_msg+1, sizeof(project_name_and_version_msg)-2);
    project_name_and_version_msg[sizeof(project_name_and_version_msg)-2] = ' ';
  }
  
  // Initialize SCD4X & Loading Screen 2
  SCD4x_Init(&scd4x_handle);
  loading_screen_msg_x = loading_screen_msg_x + strlen(loading_screen_msg) * Font_11x18.width;
  ssd1306_SetCursor(loading_screen_msg_x, loading_screen_msg_y);
  ssd1306_WriteString(".", Font_11x18, White);
  ssd1306_UpdateScreen();

  // Initialize SEN5X & Loading Screen 3
  SEN5X_Init(&sen5x_handle);
  loading_screen_msg_x = loading_screen_msg_x + Font_11x18.width;
  ssd1306_SetCursor(loading_screen_msg_x, loading_screen_msg_y);
  ssd1306_WriteString(".", Font_11x18, White);
  ssd1306_UpdateScreen();
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  if (in_debug_mode) 
  {
    ssd1306_DisplayDebugMode();
    serial_print("DEBUG MODE: Running peripheral tests...\r\n");
    while (1) {
      run_debug_tests();
      ssd1306_DisplayDebugMode();
      HAL_Delay(1000);
    }
  }

  int counter = 0;
  int arrangement = 1;
  uint32_t start_time = HAL_GetTick();

  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    /* Change arrangement every 10 seconds */
    if ((current_time - start_time) > 10000U) {
      arrangement = (arrangement + 1) % 7;  // 0..6
      start_time  = current_time;
    }

    /* Generate some demo data */
    float new_value_1 = 50.0f + 20.0f * sinf(counter * 0.2f);
    float new_value_2 = 50.0f + 20.0f * cosf(counter * 0.2f);
    float new_value_3 = 50.0f + 20.0f * tanf(counter * 0.1f);
    float new_value_4 = (counter % 20 < 10) ? 20.0f : 80.0f;

    /* Clamp crazy tan() spikes */
    if (new_value_3 > 100.0f) new_value_3 = 100.0f;
    if (new_value_3 <   0.0f) new_value_3 =   0.0f;

    ring_buffer_push(&sensor_ring_buffer_1, new_value_1);
    ring_buffer_push(&sensor_ring_buffer_2, new_value_2);
    ring_buffer_push(&sensor_ring_buffer_3, new_value_3);
    ring_buffer_push(&sensor_ring_buffer_4, new_value_4);

    ssd1306_Fill(Black);

    /* Decide which label should blink (one at a time).
       Blink slot changes every 2 seconds, modulo number of graphs. */
    uint32_t blink_slot = (current_time / 2000U);
    uint8_t  blinking_index = GRAPH_BLINK_NONE;

    switch (arrangement) {
      case 0:
        blinking_index = 0;                // only one graph
        break;
      case 1:
      case 2:
        blinking_index = (uint8_t)(blink_slot % 2U);
        break;
      case 3:
      case 4:
      case 5:
        blinking_index = (uint8_t)(blink_slot % 3U);
        break;
      case 6:
        blinking_index = (uint8_t)(blink_slot % 4U);
        break;
      default:
        blinking_index = GRAPH_BLINK_NONE;
        break;
    }

    /* By default no graphs are hidden */
    uint8_t hidden_mask = 0U;

    switch (arrangement) {
      case 0:
        /* Single full-screen graph, blinking label */
        graph_plot_ex(&sensor_ring_buffer_1,
                      0, 0,
                      SSD1306_WIDTH, SSD1306_HEIGHT,
                      1,
                      0.0f, 100.0f,
                      "CO2",
                      1,              /* blink label */
                      0);             /* not hidden */
        break;

      case 1:
        /* 2 graphs: top/bottom */
        graph_plots_1t1b_ex(&sensor_ring_buffer_1, &sensor_ring_buffer_2,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC",
                            blinking_index,
                            hidden_mask);
        break;

      case 2:
        /* 2 graphs: left/right */
        graph_plots_1l1r_ex(&sensor_ring_buffer_1, &sensor_ring_buffer_2,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC",
                            blinking_index,
                            hidden_mask);
        break;

      case 3:
        /* 3 graphs: 1 top, 2 bottom */
        graph_plots_1t2b_ex(&sensor_ring_buffer_1,
                            &sensor_ring_buffer_2,
                            &sensor_ring_buffer_3,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC", "NOx",
                            blinking_index,
                            hidden_mask);
        break;

      case 4:
        /* 3 graphs: 2 top, 1 bottom */
        graph_plots_2t1b_ex(&sensor_ring_buffer_1,
                            &sensor_ring_buffer_2,
                            &sensor_ring_buffer_3,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC", "PM",
                            blinking_index,
                            hidden_mask);
        break;

      case 5:
        /* 3 graphs: 1 left, 2 right */
        graph_plots_1l2r_ex(&sensor_ring_buffer_1,
                            &sensor_ring_buffer_2,
                            &sensor_ring_buffer_3,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC", "NOx",
                            blinking_index,
                            hidden_mask);
        break;

      case 6:
      default:
      {
        /* 4 graphs: 2x2 grid
           --- Option 1 demo ---
           Bottom-right graph (index 3) is *hidden*:
           - we set GRAPH_HIDE_3 in the mask
           - we pass buffer4 = NULL
           The library will treat that slot as blank. */

        hidden_mask = GRAPH_HIDE_3;

        ring_buffer_t *br_buffer = NULL;      /* bottom-right omitted */
        const char    *br_label  = "PM";      /* label ignored because slot hidden */

        graph_plots_2t2b_ex(&sensor_ring_buffer_1, &sensor_ring_buffer_2,
                            &sensor_ring_buffer_3, br_buffer,
                            0, 0,
                            SSD1306_WIDTH, SSD1306_HEIGHT,
                            2,
                            "CO2", "VOC", "NOx", br_label,
                            blinking_index,
                            hidden_mask);
        break;
      }
    }

    ssd1306_UpdateScreen();

    counter++;
    /* you can re-enable a small delay if needed for FPS control */
    /* HAL_Delay(50); */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
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
  HAL_GPIO_WritePin(GPIOG, LED_LOW_Pin|LED_MID_Pin|LED_HIGH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_1_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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
  GPIO_InitStruct.Pin = LED_LOW_Pin|LED_MID_Pin|LED_HIGH_Pin;
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
static void SCD4x_Init(scd4x_handle_t *scd4x_handle_p)
{
    uint8_t res;

    /* Link the interface functions */
    DRIVER_SCD4X_LINK_INIT(scd4x_handle_p, scd4x_handle_t);
    DRIVER_SCD4X_LINK_IIC_INIT(scd4x_handle_p, scd4x_interface_iic_init);
    DRIVER_SCD4X_LINK_IIC_DEINIT(scd4x_handle_p, scd4x_interface_iic_deinit);
    DRIVER_SCD4X_LINK_IIC_WRITE_COMMAND(scd4x_handle_p, scd4x_interface_iic_write_cmd);
    DRIVER_SCD4X_LINK_IIC_READ_COMMAND(scd4x_handle_p, scd4x_interface_iic_read_cmd);
    DRIVER_SCD4X_LINK_DELAY_MS(scd4x_handle_p, scd4x_interface_delay_ms);
    DRIVER_SCD4X_LINK_DEBUG_PRINT(scd4x_handle_p, scd4x_interface_debug_print);


    /* Tell the driver which chip you have */
    res = scd4x_set_type(scd4x_handle_p, SCD41);
    if (res != 0)
    {
        scd4x_interface_debug_print("scd4x_set_type failed: %u\r\n", res);
        Error_Handler();
    }

    /* Initialize the chip (calls iic_init internally) */
    res = scd4x_init(scd4x_handle_p);
    if (res != 0)
    {
        scd4x_interface_debug_print("scd4x_init failed: %u\r\n", res);
        Error_Handler();
    }
}

static void SEN5X_Init(sen5x_handle_t *sen5x_handle_p)
{
    uint8_t res;

    /* Link the interface functions */
    DRIVER_SEN5X_LINK_INIT(sen5x_handle_p, sen5x_handle_t);
    DRIVER_SEN5X_LINK_IIC_INIT(sen5x_handle_p, sen5x_interface_iic_init);
    DRIVER_SEN5X_LINK_IIC_DEINIT(sen5x_handle_p, sen5x_interface_iic_deinit);
    DRIVER_SEN5X_LINK_IIC_WRITE_COMMAND(sen5x_handle_p, sen5x_interface_iic_write_cmd);
    DRIVER_SEN5X_LINK_IIC_READ_COMMAND(sen5x_handle_p, sen5x_interface_iic_read_cmd);
    DRIVER_SEN5X_LINK_DELAY_MS(sen5x_handle_p, sen5x_interface_delay_ms);
    DRIVER_SEN5X_LINK_DEBUG_PRINT(sen5x_handle_p, sen5x_interface_debug_print);

    /* Initialize the chip (calls iic_init internally) */
    res = sen5x_init(sen5x_handle_p);
    if (res != 0)
    {
        sen5x_interface_debug_print("sen5x_init failed: %u\r\n", res);
        Error_Handler();
    }

    /* Start measurement (similar to SCD4X periodic measurement) */
    res = sen5x_start_measurement(sen5x_handle_p);
    if (res != 0)
    {
        sen5x_interface_debug_print("sen5x_start_measurement failed: %u\r\n", res);
        Error_Handler();
    }

    /* Wait for sensor to start producing data (similar to SCD4X debug test) */
    sen5x_interface_delay_ms(5000); // Wait 5 seconds for first measurement
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
// This function will run various debug tests for different components
static void run_debug_tests(void)
{
    // Call SEN5X debug test (once because uses onboard EEPROM)
    sen5x_run_full_test_once();

    // Call SCD4x debug test (once because uses onboard EEPROM)
    scd4x_run_full_test_once();

    // Call SSD1305 debug test
    ssd1306_TestAll();
}

void ssd1306_DisplayDebugMode(void) {
    char debug_screen_msg[] = "DEBUG MODE";
    uint8_t debug_screen_msg_x = (uint8_t)((SSD1306_WIDTH - (strlen(debug_screen_msg)) * 11) / 2);
    uint8_t debug_screen_msg_y = (uint8_t)((SSD1306_HEIGHT - 18) / 2);
    ssd1306_SetCursor(debug_screen_msg_x, debug_screen_msg_y);
    ssd1306_Fill(Black);
    ssd1306_WriteString(debug_screen_msg, Font_11x18, White);
    ssd1306_UpdateScreen();
}

// Poll for 1s for user pressing button
bool poll_is_debug_mode()
{
  uint32_t startup_time = HAL_GetTick();
  while (HAL_GetTick() - startup_time < 1000)
  {
    if (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET)
    {
      // Button is pressed during startup window
      uint32_t btn_press_start = HAL_GetTick();
      while (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET)
      {
        if (HAL_GetTick() - btn_press_start >= 100)
        {
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
/* USER CODE END 5 */

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
