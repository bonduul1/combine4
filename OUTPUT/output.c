#include "output.h"
#include "timer.h"
#include "Yvc1_HAL.h"

flagOutput_t flagOutput;

void output_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  HAL_GPIO_WritePin(FCO_BUZZER_GPIO_Port, FCO_BUZZER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FCO_ALARM_GPIO_Port , FCO_ALARM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FCO_RES1_GPIO_Port, FCO_RES1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FCO_RES2_GPIO_Port, FCO_RES2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FCO_RES3_GPIO_Port, FCO_RES3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FCO_RES4_GPIO_Port, FCO_RES4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FCO_RES1_Pin */
  GPIO_InitStruct.Pin = FCO_RES1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_RES1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_RES2_Pin */
  GPIO_InitStruct.Pin = FCO_RES2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_RES2_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_RES3_Pin */
  GPIO_InitStruct.Pin = FCO_RES3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_RES3_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_RES4_Pin */
  GPIO_InitStruct.Pin = FCO_RES4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_RES4_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_ALARM_Pin */
  GPIO_InitStruct.Pin = FCO_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_ALARM_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FCO_BUZZER_Pin */
  GPIO_InitStruct.Pin = FCO_BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FCO_BUZZER_GPIO_Port, &GPIO_InitStruct);
}

void clear_outputs()
{
  flagOutput.lamp_1 = OFF;
  flagOutput.lamp_2 = OFF;
  flagOutput.lamp_3 = OFF;
  flagOutput.lamp_4 = OFF;
  
  // flagOutput.alarm = OFF;
  // flagOutput.buzzer = OFF;
}

void write_outputs()
{  
  if(flagOutput.lamp_1 == ON)   {    HAL_GPIO_WritePin(FCO_RES1_GPIO_Port, FCO_RES1_Pin, GPIO_PIN_SET); }
  else                          {    HAL_GPIO_WritePin(FCO_RES1_GPIO_Port, FCO_RES1_Pin, GPIO_PIN_RESET);}
  
  if(flagOutput.lamp_2 == ON)   {    HAL_GPIO_WritePin(FCO_RES2_GPIO_Port, FCO_RES2_Pin, GPIO_PIN_SET); }
  else                          {    HAL_GPIO_WritePin(FCO_RES2_GPIO_Port, FCO_RES2_Pin, GPIO_PIN_RESET);}

  if(flagOutput.lamp_3 == ON)   {    HAL_GPIO_WritePin(FCO_RES3_GPIO_Port, FCO_RES3_Pin, GPIO_PIN_SET); }
  else                          {    HAL_GPIO_WritePin(FCO_RES3_GPIO_Port, FCO_RES3_Pin, GPIO_PIN_RESET);}

  if(flagOutput.lamp_4 == ON)   {    HAL_GPIO_WritePin(FCO_RES4_GPIO_Port, FCO_RES4_Pin, GPIO_PIN_SET); }
  else                          {    HAL_GPIO_WritePin(FCO_RES4_GPIO_Port, FCO_RES4_Pin, GPIO_PIN_RESET);}
  
  if(flagOutput.alarm == ON)    { 
                                     HAL_GPIO_WritePin(FCO_ALARM_GPIO_Port, FCO_ALARM_Pin, GPIO_PIN_SET);
                                     flagWarning.fuel = TRUE; // tseveen 20250425 added flagwarning
                                }
  else                          {    HAL_GPIO_WritePin(FCO_ALARM_GPIO_Port, FCO_ALARM_Pin, GPIO_PIN_RESET);}
  
  if(flagOutput.buzzer == ON)   {    HAL_GPIO_WritePin(FCO_BUZZER_GPIO_Port, FCO_BUZZER_Pin, GPIO_PIN_SET); }
  else                          {    HAL_GPIO_WritePin(FCO_BUZZER_GPIO_Port, FCO_BUZZER_Pin, GPIO_PIN_RESET);}
}