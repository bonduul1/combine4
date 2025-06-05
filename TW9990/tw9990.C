//-----------------------------------------------------------------------------
// TW9900.c
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "tw9990.h"

void STBon(void);
void STBoff(void);
uint8_t GetColorSystem(void);

void ChangeYMode(uint8_t Current_YMode);
void ChangeCVBS(void);
void ChangeSVideo(void);
void CheckSTB(void);

void I2CDeviceInitialize( const uint8_t *RegSet);
void I2CADV793xInitialize( const uint8_t *RegSet);


extern I2C_HandleTypeDef hi2c1;
extern void External_Video_Input_Init();

uint8_t ADV793xI2CAddress = 0xD6;

uint8_t MD0 = 0; //  = P1^0;		//Interlace or Prog; Not needed for TW9990
uint8_t MD1 = 0; // 

uint8_t Last_STD = 99; //2;

uint8_t YMode =     1;				// differential input
uint8_t InputSelection = 0;
uint8_t Last_STB = 99;

uint8_t EncoderID;
uint8_t ChipID;

/*===========================================================================*/
/*                              TW9912 EVB                                   */
/*===========================================================================*/

const uint8_t ADV793x_NTSC_DataSet[]   = {	
	0x56, 0x00,
	0x17, 0x02,
	0x00, 0x1C,
	0x01, 0x00,
	0x80, 0x10,
	0x82, 0xCB,	
	0xff, 0xff
};

const uint8_t ADV793x_PAL_DataSet[]   = {	
	0x56, 0x00,
	0x17, 0x02,
	0x00, 0x1C,
	0x01, 0x00,
	0x80, 0x11,
	0x82, 0xC3,
	0x8C, 0xCB,
	0x8D, 0x8A,
	0x8E, 0x09,
	0x8F, 0x2A,
	0xff, 0xff
};

const uint8_t TW9990_CCIR_NTSC_DataSet[] = {


	0x8a, 0x00, 	//for now is same as NTSC_Int
	0x02, 0x40,
	0x03, 0xa0,
	0x12, 0x21,
	0x1A, 0x0A,
	0x20, 0xA0,
	0x2C, 0x40,
	0x35, 0xA0,
//	0x0D, 0x74,
//	0x02, 0xC0,	// sequence to improve clamping enable change
//	0xAF, 0x14,
//	0xB0, 0x80,
//	0x0F, 0x31,

	0xff, 0xff
};

const uint8_t TW9990_NTSC_Int[] = {

	0x8a, 0x00, //using Shadow Register so no Input cropping setting for NTSC
	0x02, 0xC0,
	0x03, 0xa0,
	0x12, 0x21,
	0x1A, 0x0A,
	0x20, 0xA0,
	0x2C, 0x40,	
	0x35, 0xA0,
//	0x0D, 0x74,
//	0x02, 0xC0,
//	0xAF, 0x14,
//	0xB0, 0x80,
//	0x0F, 0x31,

	0xff, 0xff
};

const uint8_t TW9990_PAL_Int[] = {

	0x8a, 0x00, //using Shadow Register so no Input cropping setting for PAL
	0x02, 0xC0,
	0x03, 0xa0,
	0x12, 0x21,
	0x1A, 0x0A,
	0x20, 0xA0,
	0x2C, 0x40,
	0x35, 0xA0,
//	0x0D, 0x74,
//	0x02, 0xC0,
//	0xAF, 0x14,
//	0xB0, 0x80,
//	0x0F, 0x31,

	0xff, 0xff
};
  
uint8_t ReadI2C(uint8_t _dev_addr, uint8_t _addr)
{
  uint8_t _data[1] = {_addr};
  HAL_I2C_Master_Transmit(&hi2c1, _dev_addr, _data, 1, 100);
  HAL_I2C_Master_Transmit(&hi2c1, _dev_addr | 0x01, _data, 1, 100);
  return _data[0];
}

void WriteI2C(uint8_t _dev_addr, uint8_t _addr, uint8_t _val)
{
  uint8_t _data[2] = {_addr, _val};
  HAL_I2C_Master_Transmit(&hi2c1, _dev_addr, _data, 2, 100);
}
        
void WriteTW99(uint8_t _addr, uint8_t _val)
{
  WriteI2C(DEVICE_ID_TW9990, _addr, _val);
}

uint8_t ReadTW99(uint8_t _addr)
{
  return ReadI2C(DEVICE_ID_TW9990, _addr);
}

void STBon(void)
{
  WriteTW99(0xAF, 0x14);
  WriteTW99(0xB0, 0x80);
  WriteTW99(0x0F, 0x31);
}
	
void STBoff(void)
{
  WriteTW99(0xAF, 0x80);
  WriteTW99(0xB0, 0x00);
  WriteTW99(0x0F, 0x00);
}

uint8_t GetColorSystem(void)
{
  user_delay(100);
  
  if( (ReadTW99(0x1c) & 0x70) != 0x00 )
    return 1;	// PAL
  else
    return 0;	// NTSC
}

void ChangeYMode(uint8_t Current_YMode)
{
  if (Current_YMode){
    WriteTW99(0x0D, 0x74);
    WriteTW99(0x02, 0xC0);		
    if(EnbSTB == 1)
      STBon();
    else
      STBoff();
  }
  else {
    WriteTW99(0x0D, 0x00);
    WriteTW99(0x02, 0x40);
    STBoff();
  }
}

void ChangeCVBS(void)
{
  uint8_t systemstd;

  I2CDeviceInitialize(TW9990_CCIR_NTSC_DataSet);
  systemstd = GetColorSystem();
  if(!systemstd)
  {
    I2CDeviceInitialize(TW9990_NTSC_Int);
    I2CADV793xInitialize(ADV793x_NTSC_DataSet);
    MD1 = 0;
  }
  else
  {	
    I2CDeviceInitialize(TW9990_PAL_Int);
    I2CADV793xInitialize(ADV793x_PAL_DataSet);
    MD1 = 1;
  }
}


void ChangeSVideo(void)
{
  uint8_t systemstd;

  I2CDeviceInitialize(TW9990_CCIR_NTSC_DataSet);
  WriteTW99(0x02, 0x58); //0x54);//to check SVideo Standard
  WriteTW99(0x35, 0x80);
  systemstd = GetColorSystem();
  if(!systemstd)
  {
    I2CDeviceInitialize(TW9990_NTSC_Int);
    I2CADV793xInitialize(ADV793x_NTSC_DataSet);
    MD1 = 0;
  }
  else
  {
    I2CDeviceInitialize(TW9990_PAL_Int);
    I2CADV793xInitialize(ADV793x_PAL_DataSet);
    MD1 = 1;
  }
  WriteTW99(0x02, 0x58); //0x54); //change mux from NTSC reg set of CVBS
  WriteTW99(0x35, 0x80);
}

void CheckSTB(void)
{
  uint8_t STB, STG;
                                                //BOTH STB and STG; Differentiate GND and BAT SHORT
  WriteTW99(0xAF, 0x04);			//Turn off Short to GND and check
  STB = ReadTW99(0xB6);
  while (((STB & 0x80) == 0x80) && (EnbSTB == 1)){
    STB = ReadTW99(0xB6);
  }

  WriteTW99(0xAF, 0x10);			//Turn off Short to BAT and check
  STG = ReadTW99(0xB6);
  while ((STG & 0x80) == 0x80 && (EnbSTB == 1)){
    STG = ReadTW99(0xB6);
  }

  WriteTW99(0xB3, 0x80);			//Clear INTREQ
  WriteTW99(0xAF, 0x14);			//Enable Both Short to GND and BAT again 
}


//-----------------------------------------------------------------------------
// Main Routines
//-----------------------------------------------------------------------------

void I2CDeviceInitialize( const uint8_t *RegSet)
{
  uint8_t addr, index, val;

  addr = *RegSet;
  RegSet+=2;

  while (( RegSet[0] != 0xFF ) || ( RegSet[1]!= 0xFF )) {			// 0xff, 0xff is end of data
    index = *RegSet;
    val = *(RegSet+1);
    WriteI2C(addr, index, val);
    RegSet+=2;
  }
  WriteTW99(0xff, 0x00);		// set page 0
}

//------------------------------------------------------------
//	AD initialize, AD has two I2C address
//------------------------------------------------------------
void I2CADV793xInitialize( const uint8_t *RegSet)
{
  uint8_t addr, index, val;

  addr = *RegSet;
  RegSet+=2;

  addr = ADV793xI2CAddress;			//

  while (( RegSet[0] != 0xFF ) || ( RegSet[1]!= 0xFF )) {			// 0xff, 0xff is end of data
    index = *RegSet;
    val = *(RegSet+1);
    WriteI2C(addr, index, val);

    RegSet+=2;
  }
  if ( ComponentOut==1 )
    WriteI2C(ADV793xI2CAddress, 0x82, 0xC9);
  WriteTW99(0xff, 0x00);		// set page 0
}

void AutoDetectRegSet(void)
{
  uint8_t Current_STD, Current_STB;
  Current_STD = GetColorSystem();
  Current_STB = EnbSTB;

  user_delay(100);

  if(Last_STD != Current_STD){
    if(InputSelection == SVIDEO)
      ChangeSVideo();
    else 
      ChangeCVBS();
  }
  Last_STD = Current_STD;
          
  if(YMode == 1){
    if(Last_STB != Current_STB){
      if(EnbSTB == 1)
        STBon();
      else
        STBoff();
    }	
    Last_STB = Current_STB;
  }

  if (YMode == 1){ 	
    if(EnbSTB == 1){
      if (INTREQ == 0)			//check Interrupt Request Signal
        CheckSTB();
    }
  }
}


void InitRegisterSet(void)
{
  uint8_t EncoderID;
  uint8_t ChipID;

  EncoderID = ReadI2C(ADV793xI2CAddress, 0x00);				//Initialize Encoder Register Set

  if(EncoderID != 0xff)	//== 0x54)
  {
    I2CADV793xInitialize(ADV793x_NTSC_DataSet);
  }
  else {
    ADV793xI2CAddress = 0x56;								// retry with other address
    EncoderID = ReadI2C(ADV793xI2CAddress, 0x00);				//Initialize Encoder Register Set
    if (EncoderID != 0xff) {
      I2CADV793xInitialize(ADV793x_NTSC_DataSet);
    }
  }

  ChipID = ReadTW99(0x00);									//Initialize TW99xx Register Set

  switch( ChipID ) {
    case TW9990AID:
      I2CDeviceInitialize(TW9990_NTSC_Int); 	//TW9990_CCIR_NTSC_DataSet);
      ChangeYMode(YMode);
      break;
    default:
      break;
  }
  EncoderID = ReadI2C(ADV793xI2CAddress, 0x00);				//Initialize Encoder Register Set
}  

void tw9990_initialize(uint8_t init)
{ 
  user_delay(100);
  
  TW9990_RST_ENABLE;
  TW9990_PDN_ENABLE;
   
  user_delay(5);
      
  TW9990_RST_DISABLE;
  TW9990_PDN_DISABLE;

  user_delay(2);  
  
  External_Video_Input_Init();

  if(init)
  {
    InitRegisterSet();
    user_delay(100);
    MD0 = 1;
    Last_STD = GetColorSystem();
  }

  WriteTW99(0x02, 0x40);
  WriteTW99(0x03, 0x20);
  WriteTW99(0x12, 0x21);
  WriteTW99(0x1A, 0x0A);
  WriteTW99(0x20, 0xA0); 

  WriteTW99(0x02, 0x40);
  WriteTW99(0x03, 0x20);
  WriteTW99(0x12, 0x21);
  WriteTW99(0x1A, 0x0A);
  WriteTW99(0x20, 0xA0);
}

void TW9990_select_channel(uint8_t channel)
{
  if(channel == 1)
    WriteTW99(0x02, 0x40);
  else
    WriteTW99(0x02, 0x44);
  External_Video_Input_Init();
}

void tw9990_init()
{
  GPIO_InitTypeDef GPIO_InitStructure = { 0 };
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(I_RST_GPIO_Port, I_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(I_PDN_GPIO_Port, I_PDN_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStructure.Pin = I_PDN_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I_PDN_GPIO_Port, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = I_RST_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I_RST_GPIO_Port, &GPIO_InitStructure);
  
  /*Configure GPIO pin : I_INTREQ_Pin */
  GPIO_InitStructure.Pin = I_INTREQ_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I_INTREQ_GPIO_Port, &GPIO_InitStructure);
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
