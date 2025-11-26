#ifndef DRIVER_PWM_BUZZER_INTERFACE_H
#define DRIVER_PWM_BUZZER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif


/* Define functions ---------------------------------------------------------*/
void Buzzer_Delay(uint32_t duration_ms);
void Buzzer_PWM_Start(void);
void Buzzer_PWM_Stop(void);
void Buzzer_PWM_SetFrequency(uint32_t freq_hz);
void Buzzer_PWM_SetDutyCycle(uint8_t duty_percent);


#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PWM_BUZZER_INTERFACE_H */