/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* === Thresholds (LOW / MID / HIGH) ===
 * Used for: LED level + normalization to 0..100 for graphs.
 * Units:
 *  - CO2 is Δppm above AIR_CO2_OUTDOOR_PPM.
 *  - PM values are μg/m3 (sensor native).
 *  - VOC/NOx are Sensirion Index values.
 */
typedef enum {
  AQ_LEVEL_LOW = 0,
  AQ_LEVEL_MID = 1,
  AQ_LEVEL_HIGH = 2,
} aq_level_t;

typedef enum {
  POL_CO2_DELTA = 0,
  POL_VOC_INDEX,
  POL_NOX_INDEX,
  POL_PM1_UGM3,
  POL_PM2P5_UGM3,
  POL_PM4_UGM3,
  POL_PM10_UGM3,
  POL_COUNT
} pollutant_id_t;

typedef struct {
  float low_max;
  float mid_max;
  float graph_max; /* value mapped to 100% */
} pollutant_thresholds_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SENSORS_HI2C hi2c4
#define BUZZER_TIM htim2
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void serial_vprint(const char *fmt, va_list args);
void serial_print(const char *fmt, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_1_Pin GPIO_PIN_0
#define BTN_1_GPIO_Port GPIOE
#define BTN_1_EXTI_IRQn EXTI0_IRQn
#define BUZZ_0_Pin GPIO_PIN_15
#define BUZZ_0_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOD
#define LED_LOW_Pin GPIO_PIN_12
#define LED_LOW_GPIO_Port GPIOG
#define LED_MID_Pin GPIO_PIN_10
#define LED_MID_GPIO_Port GPIOG
#define LED_HIGH_Pin GPIO_PIN_9
#define LED_HIGH_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
/* === Project Data === */
#define PROJECT_NAME "Air Purity Sensor"
#define PROJECT_VERSION "2.4.2"
#define PROJECT_NAME_AND_VERSION_PADDED                                        \
  "   " PROJECT_NAME " VERSION: " PROJECT_VERSION "   "
/* === Sampling / UI timing === */
#define AIR_SENSOR_SAMPLE_INTERVAL_S (5U)
#define AIR_SENSOR_SAMPLE_INTERVAL_MS (AIR_SENSOR_SAMPLE_INTERVAL_S * 1000U)

/* If interval > this: start sensors, read once, then stop them */
#define AIR_SENSOR_STOP_IF_INTERVAL_GT_S (30U)

/* OLED refresh for blinking legends (frame rate) */
#define UI_OLED_REFRESH_MS (100U)

/* Auto-advance active legend/page */
#define UI_LEGEND_STEP_S (10U)
#define UI_LEGEND_STEP_MS (UI_LEGEND_STEP_S * 1000U)

/* Button handling */
#define BTN_DEBOUNCE_MS        (50U)
#define BTN_LONG_PRESS_MS      (1000U)  /* display sleep toggle */
#define BTN_MUTE_TOGGLE_MS     (5000U)  /* mute/unmute */

/* CO2 delta uses an "outdoor baseline" */
#define AIR_CO2_OUTDOOR_PPM (420U)

/* Alarm behavior */
#define AIR_ALARM_SUSTAIN_S            (120U)                 /* 2 minutes */
#define AIR_ALARM_SUSTAIN_MS           (AIR_ALARM_SUSTAIN_S * 1000U)

#define AIR_ALARM_EXTREME_S            (300U)                 /* 5 minutes */
#define AIR_ALARM_EXTREME_MS           (AIR_ALARM_EXTREME_S * 1000U)

/* Any pollutant >= (HIGH threshold * multiplier) is “extreme” */
#define AIR_ALARM_EXTREME_MULTIPLIER   (1.5f)

/* If 1: alarm will auto-wake the display from display-sleep */
#define AIR_ALARM_WAKE_ON_ALARM        (1U)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
