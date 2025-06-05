/********************************************************************************
 *
 *		CONFIDENTIAL
 *
 *		Copyright (C) 2009-2014 Yamaha Corporation. All rights reserved.
 *
 *		Module			: $Workfile: EvaFlashMemVc1w_Winbond.c $
 *
 *		Description		: EVB-VC1W Evaluation Board FlashROM Access Function
 *
 *		Version			: $Rev: 0 $
 *
 *		Last UpDate Time: $Date:: 2014-09-11 00:00:00#$
 *
 *		FOOT NOTE		: adjust 4tab
 *
 *		AUTHOR			: Yamaha Corporation
 *
 ********************************************************************************/


/*------------------------------------------------------------------------------*/
/*						I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc1.h"

/*--- SPI Flash Memory Select	---		set only 1 to (1)						*/
#define USE_VC1H_IS25LP128D				(1)		/* 0:Not use	1:Use			*/


/*----------------------------------*/
/* Flash Command                    */
/*----------------------------------*/
enum eSPICMD
{
	SPICMD_E_READ_ID = 0,                                                   /* [Extended] Read ID                                                       */
	SPICMD_Q_WRITE_ENABLE,                                                  /* [Quad]    Write Enable                                                   */
	SPICMD_Q_WRITE_REGISTER,                                                /* [Quad]    Write Register                                                 */
	SPICMD_Q_WRITE_REGISTER2,                                                /* [Quad]    Write Register                                                 */
	SPICMD_Q_READ_STATUS_REGISTER,                                          /* [Quad]    Read Status Register                                           */
	SPICMD_Q_READ_CONFIGURATION_REGISTER,                                   /* [Quad]    Read Configuration Register                                           */

	SPICMD_Q_BULK_ERASE,                                                    /* [Quad]    Bulk Erase                                                     */
	SPICMD_Q_SECTOR_ERASE,                                                  /* [Quad]    Sector Erase                                                   */
	SPICMD_Q_QUAD_PAGE_PROGRAM,                                             /* [Quad]    Quad Page Program                                              */
	SPICMD_Q_QUAD_IO_READ,                                                  /* [Quad]    Quad Input/Ouput Read                                          */
    
	SPICMD_4BYTE_ADDRESS_MODE,
	SPICMD_NUM,
} SPICMD;


U08 u08PatternMemorySettings[SPICMD_NUM][7] = {

	/* R#55h  R#56h  R#57h  R#58h  R#59h  R#5Ah  R#5Bh */
	{  0x02U, 0x03U, 0x00U, 0x03U, 0x9FU, 0x00U, 0x00U }, /* [Extended] Read ID    			*/ 
	{  0x02U, 0x00U, 0x00U, 0x03U, 0x06U, 0x00U, 0x00U }, /* Write Enable              	*/
	{  0x02U, 0x03U, 0x00U, 0x03U, 0x01U, 0x00U, 0x00U }, /* Write Status Register-1   	*/
	{  0x02U, 0x03U, 0x00U, 0x03U, 0xC0U, 0x00U, 0x00U }, /* Write Status Register-2   	*/
	{  0x02U, 0x03U, 0x00U, 0x03U, 0x05U, 0x00U, 0x00U }, /* Read Status Register-1    	*/
	{  0x02U, 0x03U, 0x00U, 0x03U, 0x61U, 0x00U, 0x00U }, /* Read Configuration Register*/
	{  0x02U, 0x00U, 0x00U, 0x03U, 0xC7U, 0x00U, 0x00U }, /* [Quad]    Bulk Erase	*/ 
	{  0x02U, 0x18U, 0x18U, 0x03U, 0xD8U, 0x00U, 0x00U }, /* [Quad]    Sector Erase	*/ 
	//{  0x02U, 0x19U, 0x30U, 0x03U, 0x32U, 0x00U, 0x00U }, /* [Quad]    Quad Page Program	*/ 
	{  0x02U, 0x19U, 0x18U, 0x03U, 0x32U, 0x00U, 0x00U }, /* [Quad]    Quad Page Program	*/ 
	{  0x02U, 0x81U, 0x0AU, 0x1BU, 0xEBU, 0x00U, 0x00U }, /* Quad I/O Fast Read        	*/
	
	{  0x02U, 0x03U, 0x00U, 0x03U, 0xB7U, 0x00U, 0x00U }, /* 4byte Address Mode        	*/ 
};

/*------------------------------------------------------------------------------*/
/*						P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
#define POLLING_COUNT_MAX             (3000000UL)
#define POLLING_COUNT_MAX_CHIPERASE   (100000000UL)
#define YVC1_PTNMEM_READACS_PMRREQ    (0x80U)
#define YVC1_PTNMEM_READACS_PWBUSY    (0x40U)
#define YVC1_PTNMEM_READACS_CSNEG     (0x10U)

#define VCX_SP               (0U)
#define SECTOR_SIZE   (0x10000U)
#define PAGE_BUFFER_SIZE     (0x100U)
#define SPI_STATUS_REG1_DATA   (0x40U)        /* QE=1'1                        */
#define SPI_STATUS_REG2_DATA  (0xF0U)        /* LC[1:0]=2'b10, DRV[1:0]=2'b10 */
#define READ_REGISTER_MASK    (0x03U)


/*------------------------------------------------------------------------------*/
/*                      V A R I A B L E S                                       */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
static BOOL writePtnmemCtrlRegs(U08 u08CmdNo);
static void waitClearFlag(U08 u08Mask);
static void writePmaRegs(U32 Addr);
static BOOL readStatusRegCommand(U32 Addr, U32 PollingCnt);
static void writeStatusRegCommand(U32 Addr, U08 SpiCmd, U08 Data);
static void readPtnmemDataPort(U08 *Data);
static void writeEnableCommand(U32 Addr);
//static void addr4bytemodeCommand(U32 Addr);
//static void readidCommand(U32 Addr);

/*------------------------------------------------------------------------------*/
/* Function name    : initPatternMemory                                         */
/* Contents         : Initialize Pattern Memory                                 */
/* Argument         : void                                                      */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL initPatternMemory(void)
{
	BOOL bRet = TRUE;
	U32  u32Addr = 0x00000000UL;


	/*--------------------------------------*/
	/* Write Enable                         */
	/*--------------------------------------*/
	writeEnableCommand(u32Addr);
	
	/*--------------------------------------*/
	/* Write Status Regsiter-1              */
	/*--------------------------------------*/
	writeStatusRegCommand(u32Addr, (U08)SPICMD_Q_WRITE_REGISTER, SPI_STATUS_REG1_DATA);

	/*--------------------------------------*/
	/* Read Status Register                 */
	/*--------------------------------------*/
	bRet = readStatusRegCommand(u32Addr, POLLING_COUNT_MAX);

	/*--------------------------------------*/
	/* Write Enable                         */
	/*--------------------------------------*/
	//writeEnableCommand(u32Addr);
	
	/*--------------------------------------*/
	/* Write Status Regsiter-2              */
	/*--------------------------------------*/
	writeStatusRegCommand(u32Addr, (U08)SPICMD_Q_WRITE_REGISTER2, SPI_STATUS_REG2_DATA);

	/*--------------------------------------*/
	/* Read Status Register                 */
	/*--------------------------------------*/
	bRet = readStatusRegCommand(u32Addr, POLLING_COUNT_MAX);


	/*--------------------------------------*/
	/* Quad I/O Fast Read                   */
	/*--------------------------------------*/
	if( writePtnmemCtrlRegs((U08)SPICMD_Q_QUAD_IO_READ) == FALSE )
	{
			return FALSE;
	}
	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : readPatternMemory                                         */
/* Contents         : Read Pattern Memory                                       */
/* Arguments        : U32    Addr    (W) Flash memory address                   */
/*                  : U08    *Buff   (R) Buffer pointer                         */
/*                  : U32    Num     (W) Read byte size                         */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
BOOL readPatternMemory(U32 Addr, U08 *Buff, U32 Num)
{
	U32  u32RCount   = 0UL;
	BOOL bRetWReg;

	/*--------------------------------------*/
	/* Quad I/O Fast Read                   */
	/*--------------------------------------*/
	/* (1), (2) Quad I/O Fast Read Command */
	if( writePtnmemCtrlRegs((U08)SPICMD_Q_QUAD_IO_READ) == FALSE )
	{
			return FALSE;
	}
	/* (3) Write R#27h: AIPM = 1 */
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x04U);

	/* (4) Write Address */
	writePmaRegs(Addr);

	/* (5) Write P#2: CSNEG = 0 */
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);

	while(u32RCount < Num)
	{
		/* P#2 PMRREQ data read request set */
		if(u32RCount == (Num-1UL))
		{
			/* In the case of the last */
			YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, (YVC1_PTNMEM_READACS_PMRREQ | YVC1_PTNMEM_READACS_CSNEG));
		}
		else
		{
			/* When there is nothing at the last */
			YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_PMRREQ);
		}

		/* Waiting for ready to write */
		waitClearFlag(YVC1_PTNMEM_READACS_PMRREQ);

		/* Pattern memory data Read */
		Buff[u32RCount] = YVC_READ_PORT((U08)YVC1_PORT_PTNMEM_DATA);

		u32RCount++;
	}

	/* Auto increment bit reset */
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x00U);
	
	return bRetWReg;
}



/*------------------------------------------------------------------------------*/
/* Function name    : writePtnmemCtrlRegs                                       */
/* Contents         : Write Pattern Memory Control registers                    */
/* Argument         : U08    u08CmdNo    (W) Command No                         */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
static BOOL writePtnmemCtrlRegs(U08 u08CmdNo)
{
  BOOL bRetWReg;
	if( u08PatternMemorySettings[u08CmdNo][0] == 0x00 && 
			u08PatternMemorySettings[u08CmdNo][1] == 0x00 &&
			u08PatternMemorySettings[u08CmdNo][2] == 0x00 )
	{
		return FALSE;
	}		
		
	

	/* Waiting for ready to write */
	waitClearFlag(YVC1_PTNMEM_READACS_PMRREQ | YVC1_PTNMEM_READACS_PWBUSY);

	/* Set command to R#55h-#5Bh */
	bRetWReg = YVC1_WriteRegs((U08)REG_R55H, u08PatternMemorySettings[u08CmdNo], 7U);
	
	return bRetWReg;
}



/*------------------------------------------------------------------------------*/
/* Function name    : waitClearFlag                                             */
/* Contents         : Wait to clear flags                                       */
/* Argument         : U08    u08Mask    (W) Mask value                          */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
static void waitClearFlag(U08 u08Mask)
{
	U08 u08ReadData;
	U32 u32Polling  = 0UL;
	BOOL bRet       = TRUE;

	do
	{
		u08ReadData = YVC_READ_PORT((U08)YVC1_PORT_PTNMEM_READACS);

		/* Polling max count check */
		if(u32Polling >= POLLING_COUNT_MAX)
		{
			bRet = FALSE;
		}

		u32Polling++;

	} while (((u08ReadData & u08Mask) != 0U) && (bRet == TRUE));
}



/*------------------------------------------------------------------------------*/
/* Function name    : writePmaRegs                                              */
/* Contents         : Write Pattern Memory Address register                     */
/* Argument         : U32    Addr     (W) Pattern Memory Address                */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
static void writePmaRegs(U32 Addr)
{
	U08 Pma[4];

	/*---	Pattern memory address set	---*/
	Pma[0] = (U08)(( Addr >> SHIFT_24BIT ) & MASK_03H);
	Pma[1] = (U08)(( Addr >> SHIFT_16BIT ) & MASK_FFH);
	Pma[2] = (U08)(( Addr >> SHIFT_08BIT ) & MASK_FFH);
	Pma[3] = (U08)(( Addr                ) & MASK_FFH);

	YVC1_WriteRegs((U08)REG_R2CH, &Pma[0], 4U);
}



/*------------------------------------------------------------------------------*/
/* Function name    : readStatusRegCommand                                      */
/* Contents         : Read Status Register Command                              */
/* Arguments        : U32 Addr       (W) Pattern Memory Address                 */
/*                  : U32 PollingCnt (W) Polling Count                          */
/* Returned value   : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL readStatusRegCommand(U32 Addr, U32 PollingCnt)
{
	BOOL bRet       = TRUE;
	U32  u32Polling = 0UL;
	U08  u08ReadData[2] = {0U, 0U};
	U08  u08DataMask[2] = {0U, 0U};
	U08  u08DataNum;
	U08  u08Cnt;

	/* Write Pattern Memory Control register */
	if( writePtnmemCtrlRegs((U08)SPICMD_Q_READ_STATUS_REGISTER) == FALSE )
	{
			return FALSE;
	}
#if VCX_SP
	/* Write R#27h: AIPM = 1 */
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x04U);

	convSP1Data(READ_REGISTER_MASK, &u08DataMask[0]);
	u08DataNum = 2U;
#else
	/* Write R#27h: AIPM = 0 */
	YVC1_WriteReg((U08)REG_R27H, 0x00U);

	/* Write R#2C-R#2F: PMA[25:0] */
	writePmaRegs(Addr);

	/* Write P#2: CSNEG = 1 */
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_CSNEG);

	u08DataMask[0] = READ_REGISTER_MASK;
	u08DataNum = 1U;
#endif

	do
	{
#if VCX_SP
		/* Write R#2C-R#2F: PMA[25:0] */
		writePmaRegs(Addr);

		/* Write P#2: CSNEG = 0 */
		YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);
#endif

		for(u08Cnt=0U; u08Cnt<u08DataNum; u08Cnt++)
		{
#if VCX_SP
			if(u08Cnt >= u08DataNum-1U)
			{
				/* Write P#2: CSNEG = 1 */
				YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_CSNEG);
			}
#endif

			/* Read pattern memory data */
			readPtnmemDataPort(&u08ReadData[u08Cnt]);
		}

		/* Polling max count check */
		if(u32Polling >= PollingCnt)
		{
			bRet = FALSE;
		}

		u32Polling++;

	} while ((((u08ReadData[0] & u08DataMask[0]) != 0U) || ((u08ReadData[1] & u08DataMask[1]) != 0U)) && (bRet == TRUE));
	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : writeStatusRegCommand                                     */
/* Contents         : Write Status Register Command                             */
/* Arguments        : U32 Addr   (W) Pattern Memory Address                     */
/*                  : U08 SpiCmd (W) SPI Command                                */
/*                  : U08 Data   (W) Write Status Register Data                 */
/* Returned value   : void                                                      */
/*------------------------------------------------------------------------------*/
static void writeStatusRegCommand(U32 Addr, U08 SpiCmd, U08 Data)
{
	U08  u08Cnt;
	U08  u08Data[2];
	U08  u08DataNum;
#if VCX_SP
	BOOL bRetWReg;
#endif

	/* Write Status Register command */
	if( writePtnmemCtrlRegs(SpiCmd) == FALSE )
	{
			return ;
	}
#if VCX_SP
	/* Write R#27h: AIPM = 1 */
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x04U);
#endif

#if USE_VC1H_S25FL256S
	/* Write R#27h: AIPM = 1 */
	YVC1_WriteReg((U08)REG_R27H, 0x04U);
	
		/* Write P#2: CSNEG = 0 */
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);

#endif

	/* Write R#2Ch-#2Fh: PMA[25:0] = 26h'000_0000 */
	writePmaRegs(Addr);

#if VCX_SP
	/* Write P#2: CSNEG = 0 */
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);
#endif

#if	VCX_SP
	convSP1Data(Data, &u08Data[0]);
	u08DataNum = 2U;
#else
	u08Data[0] = Data;
	u08DataNum = 1U;//u08DataNum = 2;

	#if USE_VC1H_S25FL256S
	u08Data[1] = SPI_STATUS_REG2_DATA;
		u08DataNum = 2U;//u08DataNum = 2;

	#endif
#endif

	for(u08Cnt=0U; u08Cnt<u08DataNum; u08Cnt++)
	{
		if(u08Cnt >= u08DataNum-1U)
		{
			/* Write P#2: CSNEG = 1 */
			YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_CSNEG);
		}

		/* Write P#3: MD[7:0] = 8'hxx */
		YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_DATA, u08Data[u08Cnt]);

		/* Polling P#2: PWBUSY */
		waitClearFlag(YVC1_PTNMEM_READACS_PWBUSY);
	}
		#if USE_VC1H_S25FL256S
	/* Write R#27h: AIPM = 0 */
	YVC1_WriteReg((U08)REG_R27H, 0x00U);
	#endif
}



/*------------------------------------------------------------------------------*/
/* Function name    : readPtnmemDataPort                                        */
/* Contents         : Read Pattern Memory Data Port                             */
/*                  : U08 *Data    (R) Data pointer                             */
/* Argument         : void                                                      */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
static void readPtnmemDataPort(U08 *Data)
{
	U08 u08Data;

	/* Write P#2: PMRREQ */
	u08Data = YVC_READ_PORT((U08)YVC1_PORT_PTNMEM_READACS);
	u08Data |= YVC1_PTNMEM_READACS_PMRREQ;
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, u08Data);

	/* PMRREQ Polling */
	waitClearFlag(YVC1_PTNMEM_READACS_PMRREQ);

	/* Read P#3: MD[7:0] */
	*Data = YVC_READ_PORT((U08)YVC1_PORT_PTNMEM_DATA);
}


/*------------------------------------------------------------------------------*/
/* Function name    : writeEnableCommand                                        */
/* Contents         : Write Enable Command                                      */
/* Argument         : U32    Addr     (W) Pattern Memory Address                */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
/*
static void addr4bytemodeCommand(U32 Addr)
{
	U08	 u08Cnt;
	U08  u08DataNum = 1U;
#if VCX_SP
	BOOL bRetWReg;
#endif

	// Write Pattern Memory Control Register 
	if( writePtnmemCtrlRegs((U08)SPICMD_4BYTE_ADDRESS_MODE) == FALSE )
	{
			return ;
	}
#if VCX_SP
	// Write R#27h: AIPM = 1 
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x04U);
#endif

	// Write R#2Ch-#2Fh: PMA[25:0] = 26h'000_0000 
	writePmaRegs(Addr);

#if VCX_SP
	// Write P#2: CSNEG = 0 
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);

	u08DataNum = 2U;
#endif

	for(u08Cnt=0U; u08Cnt<u08DataNum; u08Cnt++)
	{
		if(u08Cnt >= u08DataNum-1U)
		{
			// Write P#2: CSNEG = 1 
			YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_CSNEG);
		}

		// Write P#3: MD[7:0] = 8'h00 
		YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_DATA, 0x00U);

		// Polling P#2: PWBUSY 
		waitClearFlag(YVC1_PTNMEM_READACS_PWBUSY);
	}
}
*/

/*------------------------------------------------------------------------------*/
/* Function name    : writeEnableCommand                                        */
/* Contents         : Write Enable Command                                      */
/* Argument         : U32    Addr     (W) Pattern Memory Address                */
/* Return value     : void                                                      */
/*------------------------------------------------------------------------------*/
static void writeEnableCommand(U32 Addr)
{
	U08	 u08Cnt;
	U08  u08DataNum = 1U;
#if VCX_SP
	BOOL bRetWReg;
#endif

	/* Write Pattern Memory Control Register */
	if( writePtnmemCtrlRegs((U08)SPICMD_Q_WRITE_ENABLE) == FALSE )
	{
			return ;
	}

#if VCX_SP
	/* Write R#27h: AIPM = 1 */
	bRetWReg = YVC1_WriteReg((U08)REG_R27H, 0x04U);
#endif

	/* Write R#2Ch-#2Fh: PMA[25:0] = 26h'000_0000 */
	writePmaRegs(Addr);

#if VCX_SP
	/* Write P#2: CSNEG = 0 */
	YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, 0x00U);

	u08DataNum = 2U;
#endif

	for(u08Cnt=0U; u08Cnt<u08DataNum; u08Cnt++)
	{
		if(u08Cnt >= u08DataNum-1U)
		{
			/* Write P#2: CSNEG = 1 */
			YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_READACS, YVC1_PTNMEM_READACS_CSNEG);
		}

		/* Write P#3: MD[7:0] = 8'h00 */
		YVC_WRITE_PORT((U08)YVC1_PORT_PTNMEM_DATA, 0x00U);

		/* Polling P#2: PWBUSY */
		waitClearFlag(YVC1_PTNMEM_READACS_PWBUSY);
	}
}
