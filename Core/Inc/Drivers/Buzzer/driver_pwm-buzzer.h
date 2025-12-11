/**
 * @file driver_pwm-buzzer.h
 * @brief PWM buzzer driver implementation
 *
 * This header file defines constants and structures for playing tones
 * and melodies using a buzzer connected to TIM2 channel 1.
 */

#ifndef DRIVER_PWM_BUZZER_H
#define DRIVER_PWM_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "driver_pwm-buzzer_interface.h"

/**
 * @brief Note frequencies (in Hz)
 *
 * Standard musical note frequencies for C4 through B5
 */
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

/**
 * @brief Base unit for tempo calculation
 *
 * Used to define note durations in terms of a base unit
 */
#define MEGA_UNIT 166

/**
 * @brief Structure representing a musical note
 *
 * Contains a note frequency and its duration
 */
typedef struct {
    uint16_t note;      /**< Frequency of the note in Hz */
    uint16_t duration;  /**< Duration of the note in milliseconds */
} Buzzer_Note;

/**
 * @brief Play a single tone
 * @param freq_hz Frequency of the tone in Hertz
 * @param duration_ms Duration of the tone in milliseconds
 */
void Buzzer_PlayTone(uint32_t freq_hz, uint32_t duration_ms);

/**
 * @brief Play a melody sequence
 * @param melody Pointer to array of Buzzer_Note structures
 */
void Buzzer_PlayMelody(const Buzzer_Note *melody);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PWM_BUZZER_H */
