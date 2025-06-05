/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SOUND_H
#define __SOUND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//------------------ Sound defenitions -------------------------------------------

//------------------ Global variables of Sound ------------------------------------------
typedef enum
{
  NO_SOUND = 0,
  BUTTON_SOUND,
  SETTING_SOUND,
  WARNING_SOUND
} t_soundType;

typedef enum
{
  SOUND_STOP = 0,
  SOUND_PAUSE,
  SOUND_PLAY,
} t_soundState;

typedef struct
{
  t_soundType   type;                                           // What kind of sound is going on
  t_soundState  state;                                          // The sound output is going on, paused, or stopped
  uint8_t       periodCounter;                                  // How many times the sound is going on.
  uint16_t      periodNumber;                                   // How many times should be ON. If the value is higher than 8 bit the sound never stop.
  uint16_t      period;                                         // The sound frequency, duty is 50%
  uint16_t      onDuration;                                     // On time in ms
  uint16_t      offDuration;                                    // Off time in ms
  
  uint16_t      timerPeriod;                                    // Timer for checking duration
  uint16_t      timerDuration;                                  // Timer for checking onTime and offTime;
  uint16_t      timerPause;                                     // Timer for checking pausing time
  
} t_sound;

extern t_sound sound;
//------------------ Sound functions -------------------------------------------
void sound_clear(uint8_t force);
void sound_process();

void set_button_sound();
void set_setting_sound();
void set_warning_sound(uint16_t _periodNumber, uint16_t _period, uint16_t _onDuration, uint16_t _offDuration);


#ifdef __cplusplus
}
#endif

#endif /* __SOUND_H */