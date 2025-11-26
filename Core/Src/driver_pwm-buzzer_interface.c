#include <stdint.h>
#include "main.h"
#include "driver_pwm-buzzer_interface.h"

extern TIM_HandleTypeDef htim2;

//---------------------------------------------------------------------------
// Simple delay wrapper
//---------------------------------------------------------------------------
void Buzzer_Delay(uint32_t duration_ms)
{
    HAL_Delay(duration_ms);
}

//---------------------------------------------------------------------------
// Start / stop PWM on TIM2 CH1
//---------------------------------------------------------------------------
void Buzzer_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void Buzzer_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

//---------------------------------------------------------------------------
// Set PWM frequency (Hz) assuming TIM2 tick = 1 MHz (from CubeMX config)
// f = 1 MHz / (ARR + 1)  =>  ARR = (1 MHz / f) - 1
//---------------------------------------------------------------------------
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

//---------------------------------------------------------------------------
// Set duty cycle (0..100 %) on TIM2 CH1
//---------------------------------------------------------------------------
void Buzzer_PWM_SetDutyCycle(uint8_t duty_percent)
{
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    uint32_t arr   = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = ((arr + 1U) * duty_percent) / 100U;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
