
#include "sound.h"
#include "output.h"

//------------------ Global variables of Sound ------------------------------------------
t_sound sound;
t_sound savedSound;
t_sound prevSound;

void pushSound()
{
  savedSound.type = sound.type;
  savedSound.state = sound.state;
  savedSound.periodCounter = sound.periodCounter;
  savedSound.periodNumber = sound.periodNumber;
  savedSound.period = sound.period;
  savedSound.onDuration = sound.onDuration;
  savedSound.offDuration = sound.offDuration;
  savedSound.timerPeriod = sound.timerPeriod;
  savedSound.timerDuration = sound.timerDuration;
  savedSound.timerPause = sound.timerPause;
}

void pullSound()
{
  sound.type = savedSound.type;
  sound.state = savedSound.state;
  sound.periodCounter = savedSound.periodCounter;
  sound.periodNumber = savedSound.periodNumber;
  sound.period = savedSound.period;
  sound.onDuration = savedSound.onDuration;
  sound.offDuration = savedSound.offDuration;
  sound.timerPeriod = savedSound.timerPeriod;
  sound.timerDuration = savedSound.timerDuration;
  sound.timerPause = savedSound.timerPause;
}

void sound_clear(uint8_t force)
{
  if((sound.type == WARNING_SOUND) || (force == TRUE))
  {
    sound.type = NO_SOUND;
    sound.state = SOUND_STOP;
    sound.periodCounter = 0;
    sound.periodNumber = 0;
    sound.period = 0;
    sound.onDuration = 0;
    sound.offDuration = 0;
    sound.timerPeriod = 0;
    sound.timerDuration = 0;
    sound.timerPause = 0;
      
    prevSound.type = sound.type;
    prevSound.state = sound.state;
    prevSound.periodCounter = sound.periodCounter;
    prevSound.periodNumber = sound.periodNumber;
    prevSound.period = sound.period;
    prevSound.onDuration = sound.onDuration;
    prevSound.offDuration = sound.offDuration;
    prevSound.timerPeriod = sound.timerPeriod;
    prevSound.timerDuration = sound.timerDuration;
    prevSound.timerPause = sound.timerPause;
    
      
    pushSound();
  }
}

void set_button_sound()
{
//  if(sound.type != WARNING_SOUND)
  {
    // There is no sound going on
    sound.type = BUTTON_SOUND;
    sound.state = SOUND_PLAY;
    sound.periodCounter = 0;
    sound.periodNumber = 1;
    sound.period = 300;
    sound.onDuration = 300;
    sound.offDuration = 1000;
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause = 0;
  }
  /*
  else
  {
    // There is a sound action going on, thus it should be paused
    pushSound();
    sound.state = SOUND_PAUSE;
    sound.timerPause = 0;
  }
  */
}

void set_setting_sound()
{
  if((sound.type == NO_SOUND) && (prevSound.type != SETTING_SOUND))
  {
    sound.type = SETTING_SOUND;
    sound.state = SOUND_PLAY;
    sound.periodCounter = 0;
    sound.periodNumber = 3;
    sound.period = 1000;
    sound.onDuration = 6000;
    sound.offDuration = 0;
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause = 0;
  }
}

void set_warning_sound(uint16_t _periodNumber, uint16_t _period, uint16_t _onDuration, uint16_t _offDuration)
{
  if((sound.type == NO_SOUND) && (prevSound.type != WARNING_SOUND))
  {
    sound.type = WARNING_SOUND;
    sound.state = SOUND_PLAY;
    sound.periodCounter = 0;
    sound.periodNumber = _periodNumber;
    sound.period = _period;
    sound.onDuration = _onDuration;
    sound.offDuration = _offDuration;
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause = 0;
  }
}

//------------------ Sound variables ------------------------------------------
void sound_process()
{
  // The function is called every 2ms
  if(sound.state == SOUND_PLAY)
  {
    sound.timerDuration += 2;
    sound.timerPeriod += 2;
    sound.timerPause = 0;
  }
  else if(sound.state == SOUND_PAUSE)
  {
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause += 2;
    
    if(sound.timerPause >= 1000)
    {
      pullSound();
    }
  }
  else if(sound.state == SOUND_STOP)
  {
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause = 0;
  }
  
  if(sound.type == NO_SOUND)
  {
    sound.timerDuration = 0;
    sound.timerPeriod = 0;
    sound.timerPause = 0;
    flagOutput.buzzer = OFF;
    flagOutput.alarm = OFF;
/*
#if defined(USER_DEBUG)
    if(flagTimer.oneSecond == TRUE)
    {
      printf("No sound ...\r\n");
    }
#endif
*/
    return;
  }  

#if defined(USER_DEBUG)
  if(flagTimer.hundredMs == TRUE)
  {
    if(sound.type == BUTTON_SOUND)
    {
      printf("Button sound ...\r\n");
    }
    else if(sound.type == SETTING_SOUND)
    {
      printf("Setting sound ...\r\n");
    }
    else if(sound.type == WARNING_SOUND)
    {
      printf("Warning sound ...\r\n");
    }
    else
    {
      printf("No sound ...\r\n");
    }
  }
#endif  

  if(sound.state == SOUND_PLAY)
  {
    if(sound.timerDuration <= sound.onDuration)
    {
      // The repetation time is lower than on time
      if(sound.periodCounter < sound.periodNumber)
      {
        if(sound.timerPeriod < (sound.period / 2))
        {
          flagOutput.buzzer = ON;
          if(sound.type != BUTTON_SOUND)
          {
            flagOutput.alarm = ON;
          }
        }
        else if(sound.timerPeriod < sound.period)
        {
          flagOutput.buzzer = OFF;
          flagOutput.alarm = OFF;
        }
        else
        {
          sound.timerPeriod = 0;
          flagOutput.buzzer = OFF;
          flagOutput.alarm = OFF;
          sound.periodCounter++;
        }
      }
      else
      {
        // The sound repetation is DONE, thus the sound should be stopped
        if(sound.type != NO_SOUND)
        {
          prevSound.type = sound.type;
        }
        
        flagOutput.buzzer = OFF;
        flagOutput.alarm = OFF;
        sound.state = SOUND_STOP;
        sound.type = NO_SOUND;
      }
    }
    else if(sound.timerDuration <= sound.offDuration)
    {
      // The repetation time is lower than off time, sleep in here
      flagOutput.buzzer = OFF;
      flagOutput.alarm = OFF;
    }
    else
    {
      // Clear the repetation time, the 
      sound.timerDuration = 0;
    }
  }
  else
  {
    // In here, the sound state is STOP or PAUSE
    flagOutput.buzzer = OFF;
    flagOutput.alarm = OFF;
  }
}