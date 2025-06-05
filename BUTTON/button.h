/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

//------------------ Buttons defenitions -------------------------------------------
#define NUMBER_OF_BUTTONS       4
#define BTN_LONG_PRESSED        2
#define BTN_PRESSED             1
#define BTN_RELEASED            0
#define BTN_PRESSED_PERIOD      50
#define BTN_LONG_PRESSED_PERIOD 3000

#define FIC_SW_A4_Pin           GPIO_PIN_3
#define FIC_SW_A4_GPIO_Port     GPIOE
#define FIC_SW_A3_Pin           GPIO_PIN_4
#define FIC_SW_A3_GPIO_Port     GPIOE
#define FIC_SW_A2_Pin           GPIO_PIN_5
#define FIC_SW_A2_GPIO_Port     GPIOE
#define FIC_SW_A1_Pin           GPIO_PIN_6
#define FIC_SW_A1_GPIO_Port     GPIOE

//------------------ Global variables of corresponding buttons -------------------------------------------
extern uint8_t btnState[NUMBER_OF_BUTTONS];
extern uint8_t prevBtnState[NUMBER_OF_BUTTONS];

//------------------ Global functions of IO -------------------------------------------
// void button_init(); tseveen 20250421  
// void button_timer();  tseveen 20250421
// void check_buttons(); tseveen 20250421

#ifdef __cplusplus
}
#endif

#endif /* __BUTTON_H */