#include <stdint.h>
#include "main.h"
#include "driver_pwm-buzzer.h"

extern TIM_HandleTypeDef htim2;

void Buzzer_PlayTone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0) {
        // Pure rest
        HAL_Delay(duration_ms);
        return;
    }

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1_bits = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t tim_clk = (ppre1_bits <= 3U) ? pclk1 : (pclk1 * 2U);

    uint32_t prescaler = (tim_clk / 1000000U) - 1U;   // 1 MHz tick
    if (prescaler > 0xFFFFU) prescaler = 0xFFFFU;

    uint32_t timer_tick = tim_clk / (prescaler + 1U);

    uint32_t arr = (timer_tick / freq_hz) - 1U;
    if (arr > 0xFFFFU) arr = 0xFFFFU;
    if (arr == 0U) arr = 1U;

    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (arr + 1U) / 2U);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);

    // --- NEW: split into tone + gap ---
    // e.g. 80% sound, 20% silence
    uint32_t tone_ms = (duration_ms * 8U) / 10U;
    uint32_t gap_ms  = duration_ms - tone_ms;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_Delay(tone_ms);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    if (gap_ms > 0) {
        HAL_Delay(gap_ms);
    }
}

void Buzzer_PlayMelody(const Buzzer_Note *melody)
{
    for (size_t i = 0; ; i++)
    {
        uint16_t note = melody[i].note;
        uint16_t duration = melody[i].duration;

        // Terminator encountered
        if (note == 0xFFFF)
            break;

        // Rest
        if (note == 0)
        {
            HAL_Delay(duration);
            
            continue;
        }

        // Play normal note
        Buzzer_PlayTone(note, duration);
    }
}
