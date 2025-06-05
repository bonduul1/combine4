#include "memory.h"
#include "watchdog.h"
#include "AT93C56.h"



void memory_update(uint8_t isWrite, uint8_t selectConfigure)
{
  uint8_t isFirstTimeUseRom;
  uint32_t newData;
  uint16_t lowData;
  uint16_t highData;

  isFirstTimeUseRom = read_word_93c56(EEPROM_CHECK_ADDRESS);    watchdog_trigger();  // Added on 2021.06.28
  if((isWrite == 1) || (isFirstTimeUseRom != 1))                    // this case only called one time in whole program, because the MCU run first time after download the flash program
  {
    write_enable_93c56();
    watchdog_trigger();                                                              // Added on 2021.06.08
    if(isFirstTimeUseRom != 1)
    {
      write_93c56(EEPROM_CHECK_ADDRESS, 1);                     watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_ENGINE_OIL_COUNTER,  1);               watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_MISSION_OIL_COUNTER,  1);              watchdog_trigger();  // Added on 2021.06.08
    }
    
    if(selectConfigure == CONFIGURE_JOB_HOUR)
    {         // select timer 2,0
      newData = (uint32_t)(jobHour * 100);
      lowData  = (uint16_t)(newData);
      highData = (uint16_t)(newData >> 16);
      write_93c56(EEPROM_JOBHOUR_TIME + 0,  lowData);         watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_JOBHOUR_TIME + 2,  highData);        watchdog_trigger();  // Added on 2021.06.08
      
      // read and update jobHour
      lowData = read_word_93c56(EEPROM_JOBHOUR_TIME + 0);     watchdog_trigger();  // Added on 2021.06.08
      highData = read_word_93c56(EEPROM_JOBHOUR_TIME + 2);    watchdog_trigger();  // Added on 2021.06.08
      jobHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;
    }
    
    if(selectConfigure == CONFIGURE_ENGINE_HOUR)
    {         // select timer 1,0
      newData = (uint32_t)(engineHour * 100);
      lowData  = (uint16_t)(newData);
      highData = (uint16_t)(newData >> 16);
      write_93c56(EEPROM_ENGINE_TIME + 0,  lowData);            watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_ENGINE_TIME + 2,  highData);           watchdog_trigger();  // Added on 2021.06.08
      
      lowData = read_word_93c56(EEPROM_ENGINE_TIME + 0);        watchdog_trigger();  // Added on 2021.06.08
      highData = read_word_93c56(EEPROM_ENGINE_TIME + 2);       watchdog_trigger();  // Added on 2021.06.08   
      engineHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;
    }
    
    if(selectConfigure == CONFIGURE_ENGINE_OIL_HOUR)
    {         // select timer 3,0 - Engine oil changed time
      newData = (uint32_t)(engineOilHour * 100);
      lowData  = (uint16_t)(newData);
      highData = (uint16_t)(newData >> 16);
      write_93c56(EEPROM_ENGINE_OIL_TIME + 0,  lowData);        watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_ENGINE_OIL_TIME + 2,  highData);       watchdog_trigger();  // Added on 2021.06.08

      lowData = read_word_93c56(EEPROM_ENGINE_OIL_TIME + 0);    watchdog_trigger();  // Added on 2021.06.08
      highData = read_word_93c56(EEPROM_ENGINE_OIL_TIME + 2);   watchdog_trigger();  // Added on 2021.06.28
      engineOilHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;
    }
    
    if(selectConfigure == CONFIGURE_MISSION_OIL_HOUR)
    {         // select timer 4,0 - Mission oil changed time
      newData = (uint32_t)(missionOilHour * 100);
      lowData  = (uint16_t)(newData);
      highData = (uint16_t)(newData >> 16);
      write_93c56(EEPROM_MISSION_OIL_TIME + 0,  lowData);               watchdog_trigger();  // Added on 2021.06.08
      write_93c56(EEPROM_MISSION_OIL_TIME + 2,  highData);              watchdog_trigger();  // Added on 2021.06.08

      lowData = read_word_93c56(EEPROM_MISSION_OIL_TIME + 0);
      highData = read_word_93c56(EEPROM_MISSION_OIL_TIME + 2);          watchdog_trigger();  // Added on 2021.06.28
      missionOilHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;
    }
    
    if(selectConfigure == CONFIGURE_ENGINE_OIL_EXCHANGE_COUNTER)
    {
      write_93c56(EEPROM_ENGINE_OIL_COUNTER, numberOfEngineOilExchange);
      numberOfEngineOilExchange  = read_word_93c56(EEPROM_ENGINE_OIL_COUNTER);        watchdog_trigger();  // Added on 2021.06.28
    }
        
    if(selectConfigure == CONFIGURE_MISSION_OIL_EXCHANGE_COUNTER)
    {
      write_93c56(EEPROM_ENGINE_OIL_COUNTER, numberOfEngineOilExchange);
      numberOfEngineOilExchange  = read_word_93c56(EEPROM_ENGINE_OIL_COUNTER);        watchdog_trigger();  // Added on 2021.06.28
    }
    
    if(selectConfigure == CONFIGURE_LCD_BRIGHTNESS_DAY)
    {
      write_93c56(EEPROM_LCD_BRIGHTNESS_DAY, lcd_brigthness_day);
      lcd_brigthness_day = (uint8_t)read_word_93c56(EEPROM_LCD_BRIGHTNESS_DAY);
    }

    if(selectConfigure == CONFIGURE_LCD_BRIGHTNESS_NIGHT)
    {
      write_93c56(EEPROM_LCD_BRIGHTNESS_NIGHT, lcd_brigthness_night);
      lcd_brigthness_night = (uint8_t)read_word_93c56(EEPROM_LCD_BRIGHTNESS_NIGHT);
    }
    write_disable_93c56();
  }
  else
  {
    // reading data for Initialization
    lowData = read_word_93c56(EEPROM_JOBHOUR_TIME + 0);       watchdog_trigger();  // Added on 2021.06.08
    highData = read_word_93c56(EEPROM_JOBHOUR_TIME + 2);      watchdog_trigger();  // Added on 2021.06.08
    jobHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;
    
    lowData = read_word_93c56(EEPROM_ENGINE_TIME + 0);          watchdog_trigger();  // Added on 2021.06.08
    highData = read_word_93c56(EEPROM_ENGINE_TIME + 2);         watchdog_trigger();  // Added on 2021.06.08
    engineHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;    
    
    lowData = read_word_93c56(EEPROM_ENGINE_OIL_TIME + 0);      watchdog_trigger();  // Added on 2021.06.08
    highData = read_word_93c56(EEPROM_ENGINE_OIL_TIME + 2);     watchdog_trigger();  // Added on 2021.06.28
    engineOilHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;

    lowData = read_word_93c56(EEPROM_MISSION_OIL_TIME + 0);     watchdog_trigger();  // Added on 2021.06.08
    highData = read_word_93c56(EEPROM_MISSION_OIL_TIME + 2);    watchdog_trigger();  // Added on 2021.06.28
    missionOilHour = (float)(((uint32_t)highData << 16) + lowData) * 0.01;

    numberOfEngineOilExchange  = read_word_93c56(EEPROM_ENGINE_OIL_COUNTER);    watchdog_trigger();  // Added on 2021.06.08
    numberOfMissionOilExchange = read_word_93c56(EEPROM_MISSION_OIL_COUNTER);   watchdog_trigger();        // Added on 2021.06.28
    
    lcd_brigthness_day = (uint8_t)read_word_93c56(EEPROM_LCD_BRIGHTNESS_DAY);   watchdog_trigger();        // Added on 2021.06.28
    lcd_brigthness_night = (uint8_t)read_word_93c56(EEPROM_LCD_BRIGHTNESS_NIGHT);   watchdog_trigger();        // Added on 2021.06.28
    
    lcd_brigthness_day = (uint8_t)read_word_93c56(EEPROM_LCD_BRIGHTNESS_DAY);
    if(lcd_brigthness_day > 100)
    {
      lcd_brigthness_day = 100;
      write_enable_93c56();
      write_93c56(EEPROM_LCD_BRIGHTNESS_DAY, lcd_brigthness_day);       watchdog_trigger();  // Added on 2021.06.08
      write_disable_93c56();
    }
    else if(lcd_brigthness_day < 10)
    {
      lcd_brigthness_day = 10;
      write_enable_93c56();
      write_93c56(EEPROM_LCD_BRIGHTNESS_DAY, lcd_brigthness_day);       watchdog_trigger();  // Added on 2021.06.08
      write_disable_93c56();
    }
    
    if(lcd_brigthness_night > 100)
    {
      lcd_brigthness_night = 100;
      write_enable_93c56();
      write_93c56(EEPROM_LCD_BRIGHTNESS_NIGHT, lcd_brigthness_night);   watchdog_trigger();  // Added on 2021.06.08
      write_disable_93c56(); 
    }
    else if(lcd_brigthness_night < 10)
    {
      lcd_brigthness_night = 10;
      write_enable_93c56();
      write_93c56(EEPROM_LCD_BRIGHTNESS_NIGHT, lcd_brigthness_night);   watchdog_trigger();  // Added on 2021.06.08
      write_disable_93c56(); 
    }
  }
}