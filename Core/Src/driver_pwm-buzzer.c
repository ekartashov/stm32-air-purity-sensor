#include <stdint.h>
#include "main.h"
#include "driver_pwm-buzzer_interface.h"
#include "driver_pwm-buzzer.h"

extern TIM_HandleTypeDef htim2;

void Buzzer_PlayTone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0U) {
        // Pure rest
        Buzzer_Delay(duration_ms);
        return;
    }

    // Configure hardware for this tone (HAL / timer stuff is hidden in interface)
    Buzzer_PWM_SetFrequency(freq_hz);
    Buzzer_PWM_SetDutyCycle(50U);   // fixed 50% duty for the buzzer

    // Split into tone + gap (e.g. 80% sound, 20% silence)
    uint32_t tone_ms = (duration_ms * 8U) / 10U;
    uint32_t gap_ms  = duration_ms - tone_ms;

    Buzzer_PWM_Start();
    Buzzer_Delay(tone_ms);
    Buzzer_PWM_Stop();

    if (gap_ms > 0U) {
        Buzzer_Delay(gap_ms);
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
            Buzzer_Delay(duration);
            
            continue;
        }

        // Play normal note
        Buzzer_PlayTone(note, duration);
    }
}
