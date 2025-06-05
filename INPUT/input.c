#include "input.h"

//------------------ Global variables of corresponding inputs -------------------------------------------
flagInput_t flagInput;

uint8_t input_flags[NUMBER_OF_INPUTS];
uint16_t input_timers[NUMBER_OF_INPUTS];
//------------------ Inputs variables -------------------------------------------
GPIO_TypeDef* input_ports[NUMBER_OF_INPUTS] = 
{ 
    FIC_SW_5_GPIO_Port, 
    FIC_SW_6_GPIO_Port, 
    FIC_SW_7_GPIO_Port, 
    FIC_SW_8_GPIO_Port,   
    FIC_SW_9_GPIO_Port, 
    FIC_SW_4_GPIO_Port, 
    FIC_SW_3_GPIO_Port, 
    FIC_SW_2_GPIO_Port, 
    FIC_SW_1_GPIO_Port,
    FIC_SW_10_GPIO_Port,
    FIC_SW_11_GPIO_Port
};

uint16_t input_pins[NUMBER_OF_INPUTS] = 
{
    FIC_SW_5_Pin, 
    FIC_SW_6_Pin, 
    FIC_SW_7_Pin, 
    FIC_SW_8_Pin, 
    FIC_SW_9_Pin, 
    FIC_SW_4_Pin, 
    FIC_SW_3_Pin, 
    FIC_SW_2_Pin, 
    FIC_SW_1_Pin,
    FIC_SW_10_Pin,
    FIC_SW_11_Pin
};

void input_init()
{
  uint8_t i;

  GPIO_InitTypeDef GPIO_InitStructure = { 0 };

  for(i = 0; i < NUMBER_OF_INPUTS; i++)
  {
    GPIO_InitStructure.Pin = input_pins[i];
    GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(input_ports[i], &GPIO_InitStructure);
  }
  
  GPIO_InitStructure.Pin = GPIO_PIN_7;      //tseveen 20250422
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_RISING;      //tseveen 20250422;
  GPIO_InitStructure.Pull = GPIO_NOPULL;      //tseveen 20250422
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);     //tseveen 20250422
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);   //tseveen 20250422
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);            //tseveen 20250422  
}

uint8_t read_tail_light()
{
  if(HAL_GPIO_ReadPin(FIC_SW_8_GPIO_Port, FIC_SW_8_Pin) == GPIO_PIN_SET)
    return FALSE;                                                           
  else               
    return TRUE;
}

void read_inputs()
{
  uint8_t i;
  
  for( i = 0 ; i < NUMBER_OF_INPUTS; i++)
  {
    input_timers[i] += 2;
    
    if(HAL_GPIO_ReadPin(input_ports[i], input_pins[i]) == GPIO_PIN_RESET)
    {
      input_timers[i] = 0;
    }
    if(input_timers[i] >= INPUT_SCAN_PERIOD)
    {
      input_timers[i] = INPUT_SCAN_PERIOD;
      input_flags[i] = OFF;
    }
    else
    {
      if(input_flags[i] == OFF)
      {
        input_flags[i] = ON;
      }
    }
  }
  
  flagInput.rightLamp = input_flags[RIGHT_LIGHT_INPUT];
  flagInput.leftLamp = input_flags[LEFT_LIGHT_INPUT];
  flagInput.tailLamp = input_flags[TAIL_LIGHT_INPUT];
  
  flagInput.charge = (input_flags[CHARGE_INPUT] == ON) ? OFF : ON;
  
  flagInput.oilPressure = (input_flags[OIL_PRESSURE_INPUT] == ON) ? OFF : ON;
  flagInput.grain_1 = (input_flags[GRAIN_SENSOR_1]== ON) ? OFF : ON;
  flagInput.grain_2 = (input_flags[GRAIN_SENSOR_2]== ON) ? OFF : ON;
  flagInput.grain_3 = (input_flags[GRAIN_SENSOR_3]== ON) ? OFF : ON;
  flagInput.grain_4 = (input_flags[GRAIN_SENSOR_4]== ON) ? OFF : ON;
  flagInput.buzzerStop = (input_flags[SW_BUZZER_STOP]== ON) ? OFF : ON;
  flagInput.waterTemperature = (input_flags[WATER_TEMPERATURE_INPUT] == ON) ? OFF : ON;
}

void EXTI9_5_IRQHandler(void)    //tseveen 20250422
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);  //tseveen 20250422
}