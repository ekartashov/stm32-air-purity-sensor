/**
 * @file driver_pwm-buzzer_interface.h
 * @brief Interface for PWM buzzer driver functions
 *
 * This header file declares the interface functions for controlling
 * a buzzer using PWM signals on TIM2 channel 1.
 */

#ifndef DRIVER_PWM_BUZZER_INTERFACE_H
#define DRIVER_PWM_BUZZER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Delay for specified duration
 * @param duration_ms Delay duration in milliseconds
 */
void Buzzer_Delay(uint32_t duration_ms);

/**
 * @brief Start PWM output on buzzer
 */
void Buzzer_PWM_Start(void);

/**
 * @brief Stop PWM output on buzzer
 */
void Buzzer_PWM_Stop(void);

/**
 * @brief Set PWM frequency for buzzer
 * @param freq_hz Target frequency in Hertz
 */
void Buzzer_PWM_SetFrequency(uint32_t freq_hz);

/**
 * @brief Set PWM duty cycle for buzzer
 * @param duty_percent Duty cycle percentage (0-100)
 */
void Buzzer_PWM_SetDutyCycle(uint8_t duty_percent);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PWM_BUZZER_INTERFACE_H */