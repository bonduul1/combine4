
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OUTPUT_H
#define __OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
//------------------ Outputs definitions -------------------------------------------
#define FCO_RES4_Pin            GPIO_PIN_12
#define FCO_RES4_GPIO_Port      GPIOD
#define FCO_RES3_Pin            GPIO_PIN_13
#define FCO_RES3_GPIO_Port      GPIOD
#define FCO_RES2_Pin            GPIO_PIN_14
#define FCO_RES2_GPIO_Port      GPIOD
#define FCO_RES1_Pin            GPIO_PIN_15
#define FCO_RES1_GPIO_Port      GPIOD

#define FCO_ALARM_Pin           GPIO_PIN_5
#define FCO_ALARM_GPIO_Port     GPIOC  
#define FCO_BUZZER_Pin          GPIO_PIN_10
#define FCO_BUZZER_GPIO_Port    GPIOE
  
//------------------ Global variables of corresponding inputs -------------------------------------------

typedef union {
  uint32_t data;
  struct {
    uint32_t buzzer             : 1;
    uint32_t alarm              : 1;     
    uint32_t lamp_1             : 1; 
    uint32_t lamp_2             : 1; 
    uint32_t lamp_3             : 1;
    uint32_t lamp_4             : 1;
    
    uint32_t res                : 26;
  };
} flagOutput_t;

extern flagOutput_t flagOutput;

//------------------ Global functions of IO -------------------------------------------
void output_init();
void write_outputs();

#ifdef __cplusplus
}
#endif

#endif /* __OUTPUT_H */
