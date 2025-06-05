#include "AT93C56.h"
#include "dwt_stm32_delay.h"


#define E_93C46_ADDR_LEN        8

#define E_93C56_START_BIT       1

#define E_93C56_READ            0x02
#define E_93C56_ERASE           0x03
#define E_93C56_WRITE           0x01
#define E_93C56_EWEN            0x00
#define E_93C56_EWDS            0x00
#define E_93C56_ERAL            0x00
#define E_93C56_WRAL            0x00


#define E_93C56_EWEN_ADDR       (0x03<<(E_93C46_ADDR_LEN-2))
#define E_93C56_EWDS_ADDR       (0x00<<(E_93C46_ADDR_LEN-2))
#define E_93C56_ERAL_ADDR       (0x02<<(E_93C46_ADDR_LEN-2))
#define E_93C56_WRAL_ADDR       (0x01<<(E_93C46_ADDR_LEN-2))

#define E_93C56_POLL_TIMES      10
#define E_93C56_POLL_DELAY_US   1000 

#define E_93C56_PULSE_DELAY_US  1

#define SK_93C56_HIGH           HAL_GPIO_WritePin(E_CLK_Port, E_CLK_PIN, GPIO_PIN_SET)
#define SK_93C56_LOW            HAL_GPIO_WritePin(E_CLK_Port, E_CLK_PIN, GPIO_PIN_RESET)

#define CS_93C56_HIGH           HAL_GPIO_WritePin(E_CS_PORT, E_CS_PIN, GPIO_PIN_SET)
#define CS_93C56_LOW            HAL_GPIO_WritePin(E_CS_PORT, E_CS_PIN, GPIO_PIN_RESET)

#define DI_93C56_HIGH           HAL_GPIO_WritePin(E_DI_PORT, E_DI_PIN, GPIO_PIN_SET)
#define DI_93C56_LOW            HAL_GPIO_WritePin(E_DI_PORT, E_DI_PIN, GPIO_PIN_RESET)

#define DO_93C56                HAL_GPIO_ReadPin(E_DO_PORT, E_DO_PIN)

void send_cmd_93c56(uint8_t opcode, uint8_t addr);
void shift_word_93c56(uint16_t out_word);
void shift_out_bit_93c56(uint8_t bit);
uint8_t shift_in_bit_93c56(void);
uint8_t poll_93c56(void);


void init_93c56(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(E_CLK_Port, E_CLK_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E_CS_PORT, E_CS_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E_DI_PORT, E_DI_PIN,  GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = E_CLK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(E_CLK_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = E_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(E_CS_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = E_DI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(E_DI_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = E_DO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(E_DO_PORT, &GPIO_InitStruct);
  
  DWT_Delay_Init(); 
}

void shift_out_bit_93c56(uint8_t bit)
{
    /* shifts out the bit value in "bit" to MOSI */
  SK_93C56_LOW;  
  if(bit == 0)
    DI_93C56_LOW;
  else
    DI_93C56_HIGH;  
  DWT_Delay_us(E_93C56_PULSE_DELAY_US);
  SK_93C56_HIGH;
}

uint8_t shift_in_bit_93c56(void)
{
  /* toggles the SK and reads in the bit value on MISO */
  SK_93C56_LOW;
  DWT_Delay_us(E_93C56_PULSE_DELAY_US);
  SK_93C56_HIGH;
  if(DO_93C56 == GPIO_PIN_SET)
    return 1;
  else
    return 0;
}

void send_cmd_93c56(uint8_t opcode, uint8_t addr)
{
  //sends an opcode and address in the command format for the 93c56
  uint16_t temp_bits;
  int8_t temp_len;
  
  __disable_irq();
  shift_out_bit_93c56(E_93C56_START_BIT); //start bit
  temp_bits = ((uint16_t)(opcode << E_93C46_ADDR_LEN) | addr);
  temp_len = E_93C46_ADDR_LEN + 2;                                      // op code 2 + address is 8 = 10 bit
  while(temp_len > 0){
    temp_len--;
    shift_out_bit_93c56((temp_bits>>temp_len) & 1);
  }
  __enable_irq();
}

void shift_word_93c56(uint16_t out_word)
{
  //shifts out a word (16-bits) to the 93c56
  int8_t temp_len = 16;
  __disable_irq();
  while(temp_len > 0){
    temp_len--;
    shift_out_bit_93c56((out_word >> temp_len) & 1);
  }
  __enable_irq();
}

void write_enable_93c56(void)
{
  //sends the write enable command
  CS_93C56_HIGH;
  send_cmd_93c56(E_93C56_EWEN, E_93C56_EWEN_ADDR);
  SK_93C56_LOW;
  CS_93C56_LOW;
  DWT_Delay_us(E_93C56_PULSE_DELAY_US);
  
  watchdog_trigger();
}

void write_disable_93c56(void)
{
  //sends the write disable command
  CS_93C56_HIGH;
  send_cmd_93c56(E_93C56_EWDS, E_93C56_EWDS_ADDR);
  SK_93C56_LOW;
  CS_93C56_LOW;
  DWT_Delay_us(E_93C56_PULSE_DELAY_US);
  
  watchdog_trigger();
}

                                               
void write_93c56(uint8_t addr, uint16_t data)
{
  //writes "data" to an "addr" 
  CS_93C56_HIGH;
  send_cmd_93c56(E_93C56_WRITE, addr);
  shift_word_93c56(data);
  SK_93C56_LOW;
  CS_93C56_LOW;
  poll_93c56();
  
  watchdog_trigger();
}


void erase_addr_93c56(uint8_t addr)
{
  //erases from an "addr" 
  CS_93C56_HIGH;
  send_cmd_93c56(E_93C56_ERASE, addr);
  SK_93C56_LOW;
  CS_93C56_LOW;
  poll_93c56();
}

uint16_t read_93c56(uint8_t addr)
{
  uint16_t temp_store = 0;
  int8_t temp_len = 16;
  
  CS_93C56_HIGH;
  __disable_irq();
  send_cmd_93c56(E_93C56_READ, addr);
  // shift_in_bit_93c56();                         //get the dummy bit out of there
  while(temp_len > 0)
  {
    temp_len--;
    temp_store |= (uint16_t)(shift_in_bit_93c56() << temp_len);
  }
  __enable_irq();
  CS_93C56_LOW;
  SK_93C56_LOW;
  DWT_Delay_us(E_93C56_PULSE_DELAY_US);
  
  watchdog_trigger();
  return temp_store;
}

uint8_t poll_93c56(void)
{
  // polls the MISO (DO on 93c56) to see whan it's ready for more input
  // returns 1 (TRUE) if succesful within amount of poll times
  int8_t temp_poll_num = E_93C56_POLL_TIMES;
  CS_93C56_HIGH;
  while(temp_poll_num > 0){
    temp_poll_num--;
    if(DO_93C56 == GPIO_PIN_SET){
        CS_93C56_LOW;
        return 1;
    }
    DWT_Delay_us(E_93C56_POLL_DELAY_US);
  }
  CS_93C56_LOW;
  return 0;
}