/**
 * @file driver_pwm-buzzer_interface.c
 * @brief Implementation of PWM buzzer interface functions
 *
 * This file implements the low-level interface functions for controlling
 * a buzzer using PWM signals on TIM2 channel 1.
 */

#include <stdint.h>
#include "main.h"
#include "driver_pwm-buzzer_interface.h"

extern TIM_HandleTypeDef htim2;

/**
 * @brief Delay for specified duration
 *
 * This function provides a blocking delay for the specified number of
 * milliseconds using the HAL_Delay function.
 *
 * @param duration_ms Delay duration in milliseconds
 */
void Buzzer_Delay(uint32_t duration_ms)
{
    HAL_Delay(duration_ms);
}

/**
 * @brief Start PWM output on buzzer
 *
 * Enables PWM output on TIM2 channel 1 for the buzzer.
 */
void Buzzer_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief Stop PWM output on buzzer
 *
 * Disables PWM output on TIM2 channel 1 for the buzzer.
 */
void Buzzer_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief Set PWM frequency for buzzer
 *
 * Calculates and sets the PWM frequency for the buzzer based on the
 * desired frequency. The timer is configured to generate the specified
 * frequency using the timer's auto-reload register.
 *
 * @param freq_hz Target frequency in Hertz
 */
void Buzzer_PWM_SetFrequency(uint32_t freq_hz)
{
    if (freq_hz == 0U) {
        return;    // nothing to do for 0 Hz here; driver handles rests
    }

    const uint32_t timer_tick_hz = 1000000U;  // 1 MHz from MX_TIM2_Init

    // Compute ARR+1 = tick / freq
    uint32_t arr_plus1 = timer_tick_hz / freq_hz;
    if (arr_plus1 == 0U) {
        arr_plus1 = 1U;                      // clamp to minimum period
    }

    uint32_t arr = arr_plus1 - 1U;
    if (arr > 0xFFFFU) {
        arr = 0xFFFFU;                       // 16-bit timer guard
    }

    // Update period and reset counter using HAL helpers
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/**
 * @brief Set PWM duty cycle for buzzer
 *
 * Sets the duty cycle for the PWM signal on TIM2 channel 1.
 * The duty cycle is expressed as a percentage (0-100).
 *
 * @param duty_percent Duty cycle percentage (0-100)
 */
void Buzzer_PWM_SetDutyCycle(uint8_t duty_percent)
{
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    uint32_t arr   = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = ((arr + 1U) * duty_percent) / 100U;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
