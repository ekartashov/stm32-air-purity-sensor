/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVER_BUZZER_INTERFACE_H
#define DRIVER_BUZZER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>


/* Define notes --------------------------------------------------------------*/

// Note frequencies
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

// Tempo
#define MEGA_UNIT 166

// Note
typedef struct {
    uint16_t note;
    uint16_t duration;
} Buzzer_Note;


/* Define functions-----------------------------------------------------------*/
void Buzzer_PlayTone(uint32_t freq_hz, uint32_t duration_ms);
void Buzzer_PlayMelody(const Buzzer_Note *melody);


#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BUZZER_INTERFACE_H */
