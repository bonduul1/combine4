
#include "button.h"
#include "sound.h"

//------------------ Global variables of corresponding buttons -------------------------------------------
GPIO_TypeDef*   BTN_PORTS[NUMBER_OF_BUTTONS] = {FIC_SW_A1_GPIO_Port, FIC_SW_A2_GPIO_Port, FIC_SW_A3_GPIO_Port, FIC_SW_A4_GPIO_Port };
uint16_t        BTN_PINS[NUMBER_OF_BUTTONS]  = {FIC_SW_A1_Pin, FIC_SW_A2_Pin, FIC_SW_A3_Pin, FIC_SW_A4_Pin };

uint8_t btnState[NUMBER_OF_BUTTONS];
uint8_t btnFlags[NUMBER_OF_BUTTONS];
uint16_t btnTimers[NUMBER_OF_BUTTONS];

uint8_t prevBtnState[NUMBER_OF_BUTTONS];

void button_init()
{
  uint8_t i;

  GPIO_InitTypeDef GPIO_InitStructure = { 0 };

  for(i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    GPIO_InitStructure.Pin = BTN_PINS[i];
    GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BTN_PORTS[i], &GPIO_InitStructure);
  }
}

void check_buttons(void)
{
  uint8_t i;
    
  for(i = 0 ; i < NUMBER_OF_BUTTONS; i++)
  {
    btnTimers[i] += 2;
    
    if(HAL_GPIO_ReadPin(BTN_PORTS[i], BTN_PINS[i]) == GPIO_PIN_SET)             // No action on button
    {
      btnTimers[i] = 0;
    }
    
    if(btnTimers[i] >= BTN_PRESSED_PERIOD)                                      // Button pressed
    {
      btnTimers[i] = BTN_PRESSED_PERIOD;
      btnFlags[i] = BTN_PRESSED;
    }
    else
    {
      btnFlags[i] = BTN_RELEASED;
    }
    prevBtnState[i] = btnState[i];
    btnState[i] = btnFlags[i];
    
    if((btnState[i] == BTN_PRESSED) && (prevBtnState[i] == BTN_RELEASED))       // Rising edge
    {
//      set_button_sound();  tseveen 20250421
    }

#if defined(USER_DEBUG)
    if(btnState[i] == BTN_PRESSED)
    {
      if(flagTimer.hundredMs == TRUE)
      {
        printf("btnState[%d] pressed.\r\n", i + 1);
      }
    }
#endif
  }
}