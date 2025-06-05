//-----------------------------------------------------------------------------
// TW9900.h
//-----------------------------------------------------------------------------

#ifndef __TW9900_H
#define __TW9900_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Defines  ------------------------------------------------------------------*/

#define I_INTREQ_Pin            GPIO_PIN_13
#define I_INTREQ_GPIO_Port      GPIOE
#define I_INTREQ_EXTI_IRQn      EXTI15_10_IRQn
  
#define I_RST_Pin               GPIO_PIN_14
#define I_RST_GPIO_Port         GPIOE
  
#define I_PDN_Pin               GPIO_PIN_15
#define I_PDN_GPIO_Port         GPIOE

#define TW9990_RST_DISABLE      HAL_GPIO_WritePin(I_RST_GPIO_Port, I_RST_Pin, GPIO_PIN_SET)
#define TW9990_RST_ENABLE       HAL_GPIO_WritePin(I_RST_GPIO_Port, I_RST_Pin, GPIO_PIN_RESET)
  
#define TW9990_PDN_ENABLE       HAL_GPIO_WritePin(I_PDN_GPIO_Port, I_PDN_Pin, GPIO_PIN_SET)
#define TW9990_PDN_DISABLE      HAL_GPIO_WritePin(I_PDN_GPIO_Port, I_PDN_Pin, GPIO_PIN_RESET)

#define DEVICE_ID_TW9990	0x88
#define TW9990AID	        0x00

#define FWVER			0x03

#define COMPOSITE		1
#define SVIDEO			2
#define INPUTSOURCEKEY	        0x80

#define ComponentOut            0
#define EnbSTB                  0

#define DIFFMODE                1
#define INTREQ                  0

void tw9990_init();
void tw9990_initialize(uint8_t init);

#ifdef __cplusplus
}
#endif

#endif /* __TW9900_H */