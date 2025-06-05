#include "DS1302.h"
#include "dwt_stm32_delay.h"

enum Register
{
  kSecondReg       = 0,
  kMinuteReg       = 1,
  kHourReg         = 2,
  kDateReg         = 3,
  kMonthReg        = 4,
  kDayReg          = 5,
  kYearReg         = 6,
  kWriteProtectReg = 7,

  // The RAM register space follows the clock register space.
  kRamAddress0     = 32
};

enum Command
{
  kClockBurstRead  = 0xBF,
  kClockBurstWrite = 0xBE,
  kRamBurstRead    = 0xFF,
  kRamBurstWrite   = 0xFE
};

// SDA Write(output) Mode
void writeSDA(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DS1302_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS1302_SDA_PORT, &GPIO_InitStruct);
}

// SDA Read(input) Mode
void readSDA(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DS1302_SDA_PIN;
  GPIO_InitStruct.Mode =  GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS1302_SDA_PORT, &GPIO_InitStruct);	
}

// Returns the decoded decimal value from a binary-coded decimal (BCD) byte.
// Assumes 'bcd' is coded with 4-bits per digit, with the tens place digit in
// the upper 4 MSBs.
uint8_t bcdToDec(const uint8_t bcd)
{
  return (10 * ((bcd & 0xF0) >> 4) + (bcd & 0x0F));
}

// Returns the binary-coded decimal of 'dec'. Inverse of bcdToDec.
uint8_t decToBcd(const uint8_t dec)
{
  const uint8_t tens = dec / 10;
  const uint8_t ones = dec % 10;
  return (tens << 4) | ones;
}

// Returns the hour in 24-hour format from the hour register value.
uint8_t hourFromRegisterValue(const uint8_t value)
{
  uint8_t adj;
  if (value & 128)  // 12-hour mode
    adj = 12 * ((value & 32) >> 5);
  else           // 24-hour mode
    adj = 10 * ((value & (32 + 16)) >> 4);
  return (value & 15) + adj;
}

/* Sends an address or command */
void DS1302_writeOut(uint8_t value, uint8_t readAfter)
{
  uint8_t i;
  writeSDA();

  for (i = 0; i < 8; ++i)
  {
    HAL_GPIO_WritePin(DS1302_SDA_PORT, DS1302_SDA_PIN, ((value >> i) & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
    DWT_Delay_us(1);
    HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_SET);
    DWT_Delay_us(1);

    if (readAfter && i == 7) {
      // We're about to read data -- ensure the pin is back in input mode
      // before the clock is lowered.
      readSDA();
    } 
    else 
    {
      HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
      DWT_Delay_us(1);
    }
  }
}

uint8_t DS1302_readIn()
{
  uint8_t input_value = 0;
  uint8_t bit = 0, i;
  readSDA();

  // Bits from the DS1302 are output on the falling edge of the clock
  // cycle. This is called after readIn (which will leave the clock low) or
  // writeOut(..., true) (which will leave it high).
  for (i = 0; i < 8; ++i)
  {
    HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_SET);
    DWT_Delay_us(1);
    HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
    DWT_Delay_us(1);
    
    bit = HAL_GPIO_ReadPin(DS1302_SDA_PORT, DS1302_SDA_PIN);
    input_value |= (bit << i);  // Bits are read LSB first.
  }

  return input_value;
}

uint8_t DS1302_readRegister(const uint8_t reg)
{
  uint8_t cmd_byte = (0x81 | (reg << 1));
  uint8_t ret;
  
  HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_SET);
  DWT_Delay_us(4);  // tCC

  DS1302_writeOut(cmd_byte, 1);
  ret = DS1302_readIn();
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_RESET);
  DWT_Delay_us(4);  // tCWH
  return ret;
}

void DS1302_writeRegister(const uint8_t reg, const uint8_t value)
{
  uint8_t cmd_byte = (0x80 | (reg << 1));
  
  HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_SET);
  DWT_Delay_us(4);  // tCC
  DS1302_writeOut(cmd_byte,0);
  DS1302_writeOut(value, 0);
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_RESET);
  DWT_Delay_us(4);  // tCWH
}

void DS1302_writeProtect(const uint8_t enable)
{
  DS1302_writeRegister(kWriteProtectReg, (enable << 7));
}

void DS1302_halt(uint8_t enable)
{
  uint8_t sec = DS1302_readRegister(kSecondReg);
  sec &= ~(1 << 7);
  sec |= (enable << 7);
  DS1302_writeRegister(kSecondReg, sec);
}

void rtc_getTime(uint8_t *buf) 
{
  HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_SET);
  DWT_Delay_us(4);  // tCC

  DS1302_writeOut(kClockBurstRead, 1);
  buf[6] = bcdToDec(DS1302_readIn() & 0x7F);
  buf[5] = bcdToDec(DS1302_readIn());
  buf[4] = hourFromRegisterValue(DS1302_readIn());
  buf[3] = bcdToDec(DS1302_readIn());
  buf[2] = bcdToDec(DS1302_readIn());
  buf[7] = bcdToDec(DS1302_readIn());
  buf[1] = bcdToDec(DS1302_readIn());
  
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_RESET);
  DWT_Delay_us(4);  // tCWH
}

void rtc_setTime(uint8_t *buf)
{
  // We want to maintain the Clock Halt flag if it is set.
  uint8_t ch_value = DS1302_readRegister(kSecondReg) & 0x80;

  HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_SET);
  DWT_Delay_us(4);  // tCC

  DS1302_writeOut(kClockBurstWrite, 0);
  DS1302_writeOut(ch_value | decToBcd(buf[6]), 0);
  DS1302_writeOut(decToBcd(buf[5]), 0);
  DS1302_writeOut(decToBcd(buf[4]), 0);
  DS1302_writeOut(decToBcd(buf[3]), 0);
  DS1302_writeOut(decToBcd(buf[2]), 0);
  DS1302_writeOut(decToBcd(buf[7]), 0);
  DS1302_writeOut(decToBcd(buf[1]), 0);
  // All clock registers *and* the WP register have to be written for the time
  // to be set.
  DS1302_writeOut(0, 0);  // Write protection register.
  
  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_RESET);
  DWT_Delay_us(4);  // tCWH  
}

/* Initialization */
void rtc_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(DS1302_RST_PORT, DS1302_RST_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN,  GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = DS1302_SCLK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS1302_SCLK_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = DS1302_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS1302_RST_PORT, &GPIO_InitStruct);
  
  readSDA();

  // DWT Init
  DWT_Delay_Init();
  DS1302_writeProtect(0);
  DS1302_halt(0);
}

