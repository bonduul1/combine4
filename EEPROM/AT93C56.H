
#ifndef __93C56_H_
#define __93C56_H_

#ifdef __cplusplus
extern "C" {
#endif

 /*
  *     1. EEPROM Size 2Kbit = 256KByte = 128 word
  *
  *
  */
  
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define E_CLK_PIN               GPIO_PIN_4
#define E_CLK_Port              GPIOD
#define E_CS_PIN                GPIO_PIN_5
#define E_CS_PORT               GPIOD
#define E_DO_PIN                GPIO_PIN_6
#define E_DO_PORT               GPIOD
#define E_DI_PIN                GPIO_PIN_7
#define E_DI_PORT               GPIOD

/*
#define EEPROM_MODEL_SELECTION                  0x00                            // 2 bytes
#define EEPROM_AXEL_THRESHING_DELAY             0x02                            // 2 bytes
#define EEPROM_AXEL_AUGER_DELAY                 0x04                            // 2 bytes
#define EEPROM_AXEL_AUGER_AUTO_DELAY            0x06                            // 2 bytes
#define EEPROM_AXEL_YEACHE_KU_TIME              0x08                            // 2 bytes
#define EEPROM_AXEL_TBS_KU_TIME                 0x0A                            // 2 bytes
#define EEPROM_AXEL_AUGER_KU_DELAY              0x0C                            // 2 bytes
#define EEPROM_AXEL_C_SPEED_KU_TIME             0x0E                            // 2 bytes
#define EEPROM_AXEL_THRESHING_RPM               0x10                            // 2 bytes
#define EEPROM_AXEL_KEEP_TIME_MODE              0x12                            // 2 bytes
#define EEPROM_AXEL_APP_SENSOR_TOTAL_ERROR      0x14                            // 2 bytes
#define EEPROM_AXEL_DELAY_MODE                  0x16                            // 2 bytes
#define EEPROM_AXEL_APP_SENSOR1_POSITION_MAX    0x18                            // 2 bytes
#define EEPROM_AXEL_APP_SENSOR1_POSITION_MIN    0x1A                            // 2 bytes
#define EEPROM_AXEL_APP_SENSOR2_POSITION_MAX    0x1C                            // 2 bytes
#define EEPROM_AXEL_APP_SENSOR2_POSITION_MIN    0x1E                            // 2 bytes

#define EEPROM_MISSION_OIL_COUNTER              0x20                            // Mission Oil counter 2 bytes
#define EEPROM_ENGINE_OIL_COUNTER               0x22                            // Engine Oil counter 2 bytes
#define EEPROM_MISSION_OIL_TIME                 0x24                            // Mission oil time 4 bytes
#define EEPROM_ENGINE_OIL_TIME                  0x28                            // Engine oil time 4 bytes
#define EEPROM_ENGINE_TIME                      0x2C                            // Engine time 4 bytes
#define EEPROM_JOBHOUR_TIME                     0x30                            // Job time 4 bytes
#define EEPROM_LCD_BRIGHTNESS_DAY               0x34                            // LCD brightness day 2 bytes
#define EEPROM_LCD_BRIGHTNESS_NIGHT             0x36                            // LCD brightness night 2 bytes
#define EEPROM_CHECK_ADDRESS                    0x70                            // Check address 2 bytes
*/

void init_93c56();
extern void write_enable_93c56(void);
extern void write_disable_93c56(void);

extern void write_93c56(uint8_t addr, uint16_t data);
extern void erase_addr_93c56(uint8_t addr);
extern uint16_t read_93c56(uint8_t addr);
extern 


#ifdef __cplusplus
}
#endif

#endif /* __93C56_H_ */