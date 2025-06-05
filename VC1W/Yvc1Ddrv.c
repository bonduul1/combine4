/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2015 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Ddrv.c $                                   */
/*  Description     : XFL Driver Function                                       */
/*  Version         : $Rev: 50 $                                                */
/*  Last UpDate Time: $Date:: 2015-01-29 15:29:26#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/*                    H.Nakahara                                                */
/********************************************************************************/


/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc1Ddrv.h"
#include "Yvc1Pmem.h"

/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
#define YVC1_PLL_LOCKUP_WAIT            (4)                     /* ms           */
#define YVC1_INIEND_CHK                 (0x20)
#define YVC1_INIEND_WAIT                (2)                     /* ms           */
#define YVC1_INIEND_POLLING             (3000)                  /* ms           */
#define YVC1_INIEND_COUNT_UP            (YVC1_POLLING_MAX / (YVC1_INIEND_POLLING / YVC1_INIEND_WAIT))
#define YVC1_SR                         (0x80)
#define YVC1_GTBL_INIT_END_ADDR         (YVC_GENERAL_TABLE_INIT_ADDR + (YVC_GENERAL_TABLE_INIT_NUM * YVC1_LYR_SIZE))
#define YVC1_CPULYR_ADDR_MAX            (4080)
#define YVC1_CPULYR_ADDR_12N            (12)
#define YVC1_CPULYR_NUM_MAX             (341)
#define YVC1_CPULYR_ADDR_END_MAX        (4092)
#define YVC1_REG_CSNEG_MASK             (0x10U)

/*--------------------------------------*/
/*  DMA                                 */
/*--------------------------------------*/
#define YVC1_DMA_SDCOMP_GAM_MAX         (0)
#define YVC1_DMA_SDCOMP_WRP_MAX         (1)
#define YVC1_DMA_DESTSEL_MAX            (1)
#define YVC1_DMA_SOCEAD_MAX             (YVC1_PMEM_END_ADDR)
#define YVC1_DMA_DESTAD_MAX             (0x1FFF)
#define YVC1_DMA_DESTAD_4N_MAX          (0x1FFC)
#define YVC1_DMA_DESTAD_4N              (4)
#define YVC1_DMA_BYTECNT_GAM_MAX        (768)
#define YVC1_DMA_BYTECNT_WRP_MAX        (8192)
#define YVC1_DMA_BYTECNT_WRP_4N_MAX     (8192)
#define YVC1_DMA_BYTECNT_WRP_4N         (4)
#define YVC1_DMA_SDCOMP_ON              (1)
#define YVC1_DMA_SDCOMP_OFF             (0)
#define YVC1_DMA_DESTSEL_WRP            (0)
#define YVC1_DMA_DESTSEL_GAM            (1)

/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      V A R I A B L E S                                       */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      E X T E R N                                             */
/*------------------------------------------------------------------------------*/
//extern void  YGV643_SPI_TX(U08 data);
//extern U08 YGV643_SPI_RX(void);
//extern void YGV643_SPI_WAIT_RXRDY(void);
//extern void YGV643_SPI_WAIT_TXRDY(void);
//extern void YGV643_SPI_TX_Wait(U08 x);  // 데이타 발송 하고 종료 될 때 까지 대기..
extern void Y643_CS_Control(U08 x);
extern U08 YGV643_SPI_TXRX(U08 *tx, U08 *rx, U08 num);
/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
static BOOL s_INIENDWaitPolling(void);
#if YVC_GENERAL_TABLE_INIT
static BOOL s_GeneralTblInit(void);
#endif
static BOOL s_FBWait(void);
static BOOL s_VBWaitPolling(U08 Mode);
static BOOL s_WriteRegPll(U08 RegNo, U08 Data);
static T_Y643_PORT_PTNMEM s_PtnMemReadAcs_Reg7_Data(void);

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_PmemSet                                              */
/* Contents         : Pattern memory setting                                    */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_PmemSet(void)
{
	BOOL bRet = TRUE;
        initPatternMemory();
	/*--------------------------------------------------------------------------*/
	/* Pattern memory setting                                                   */
	/* e.g.) Set the pattern memory processing according to use environment.    */
	/*                                                                          */
	/* Please set up a pattern memory using a register (R#55h-R#5Bh) and        */
	/* a port (P#2,P#3)                                       .                 */
	/*                                                                          */
	/*--------------------------------------------------------------------------*/



	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WriteReg                                             */
/* Contents         : Register 1-byte write                                     */
/* Argument         : U08   RegNo   (W)Register number                          */
/*                  : U08   Data    (W)Write data                               */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WriteReg(U08 RegNo, U08 Data)
{
	BOOL bRet = TRUE;
	T_Y643_PORT_REGSEL tY643PortRegSel = {0};
	T_Y643_PORT_PTNMEM tY643PortPtnMem = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
/*	if (RegNo >= YVC1_REG_SIZE)
	{
		bRet = FALSE;
	}
Enkhbat disbaled*/
	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		EXCTRL_PORT_LOCK_REG

		/*--- Register data set ---*/
		tY643PortPtnMem           = s_PtnMemReadAcs_Reg7_Data();
		tY643PortPtnMem.BIT.RGN_7 = ((RegNo >> SHIFT_07BIT) & MASK_01H);
		tY643PortRegSel.BIT.RGN_0 =  (RegNo                 & MASK_7FH);
		YVC_WRITE_PORT(YVC1_PORT_PTNMEM_READACS, tY643PortPtnMem.BYTE);
		YVC_WRITE_PORT(YVC1_PORT_REG_SEL       , tY643PortRegSel.BYTE);
		YVC_WRITE_PORT(YVC1_PORT_REG_DATA, Data);

		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadReg                                              */
/* Contents         : Register 1-byte read                                      */
/* Argument         : U08   RegNo   (W)Register number                          */
/*                    U08   *Data   (R)Read data pointer                        */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ReadReg(U08 RegNo, U08 *Data)
{
	BOOL bRet = TRUE;
	T_Y643_PORT_REGSEL tY643PortRegSel = {0};
	T_Y643_PORT_PTNMEM tY643PortPtnMem = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if  (Data == (U08*)NULL) //||
//		 (RegNo >= YVC1_REG_SIZE) )
	{
		bRet = FALSE;
	}
//Enkhbat disbaled
	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		EXCTRL_PORT_LOCK_REG

		/*--- Register data set ---*/
		tY643PortPtnMem           = s_PtnMemReadAcs_Reg7_Data();
		tY643PortPtnMem.BIT.RGN_7 = ((RegNo >> SHIFT_07BIT) & MASK_01H);
		tY643PortRegSel.BIT.RGN_0 =  (RegNo                 & MASK_7FH);
		YVC_WRITE_PORT(YVC1_PORT_PTNMEM_READACS, tY643PortPtnMem.BYTE);
		YVC_WRITE_PORT(YVC1_PORT_REG_SEL       , tY643PortRegSel.BYTE);
		*Data = (U08)YVC_READ_PORT(YVC1_PORT_REG_DATA);

		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WriteRegs                                            */
/* Contents         : Register continuous write                                 */
/* Argument         : U08   RegNo;  (W)Start register number                    */
/*                    U08   *Buff;  (W)Write buffer pointer                     */
/*                    U08   Num;    (W)Number of bytes to write                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WriteRegs(U08 RegNo, const U08 *Buff, U08 Num)
{
	BOOL bRet = TRUE;
	T_Y643_PORT_REGSEL tY643PortRegSel = {0};
	T_Y643_PORT_PTNMEM tY643PortPtnMem = {0};

	#if !(YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST)
		U08 u08Cnt = 0;
	#endif

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( //(RegNo >= YVC1_REG_SIZE) ||
		 (Num == 0) ||
		 ((RegNo + Num) > YVC1_REG_SIZE) ||
		 (Buff == (U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		EXCTRL_PORT_LOCK_REG

		/*--- Auto increment bit set ---*/
		tY643PortPtnMem           = s_PtnMemReadAcs_Reg7_Data();
		tY643PortPtnMem.BIT.RGN_7 = ((RegNo >> SHIFT_07BIT) & MASK_01H);
		tY643PortRegSel.BIT.RGN_0 =  (RegNo                 & MASK_7FH);
		tY643PortRegSel.BIT.AIRG  = 1;
		YVC_WRITE_PORT(YVC1_PORT_PTNMEM_READACS, tY643PortPtnMem.BYTE);
		YVC_WRITE_PORT(YVC1_PORT_REG_SEL       , tY643PortRegSel.BYTE);

		#if YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST

			YVC_BURST_WRITE_PORT(YVC1_PORT_REG_DATA, Buff, Num);
//                        YVC_BurstWritePort(YVC1_PORT_REG_DATA, Buff, Num);

		#else

			/*--- It writes register data in the auto increment ---*/
			for (u08Cnt=0; u08Cnt<Num ;u08Cnt++)
			{
				YVC_WRITE_PORT(YVC1_PORT_REG_DATA, Buff[u08Cnt]);
			}

		#endif

		/*--- Auto increment bit reset ---*/
		tY643PortRegSel.BIT.AIRG = 0;
		YVC_WRITE_PORT(YVC1_PORT_REG_SEL, tY643PortRegSel.BYTE);

		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadTbls                                             */
/* Contents         : General table continuous read                             */
/* Argument         : U16   Addr;   (W)General table address                    */
/*                    U08   *Buff;  (R)Read buffer pointer                      */
/*                    U16   Num;    (W)Number of bytes to read                  */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ReadTbls(U16 Addr, U08 *Buff, U16 Num)
{
	BOOL bRet = TRUE;
	#if !(YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST)
		U16  u16Count = 0;
	#endif

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Buff == (U08 *)NULL) ||
	     (Num  == 0) )
	{
		bRet = FALSE;
	}

	/*--- General Table ---*/
	if(Addr < YVC1_EXGTBL1_START_ADDR)
	{
		if((Addr + Num) > YVC1_EXGTBL1_START_ADDR)
		{
			bRet = FALSE;
		}
	}
	/*--- Extended General Table1 ---*/
	else if(Addr < YVC1_EXGTBL1_START_ADDR + YVC1_EXGTBL_SIZE)
	{
		if((Addr + Num) > YVC1_EXGTBL1_START_ADDR + YVC1_EXGTBL_SIZE)
		{
			bRet = FALSE;
		}
	}
	/*--- Extended General Table2 ---*/
	else if((YVC1_EXGTBL2_START_ADDR <= Addr) && (Addr < YVC1_EXGTBL2_START_ADDR + YVC1_EXGTBL_SIZE))
	{
		if((Addr + Num) > YVC1_EXGTBL2_START_ADDR + YVC1_EXGTBL_SIZE)
		{
			bRet = FALSE;
		}
	}
	else
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- General table address set ---*/
		bRet = YVC1_WriteReg(REG_R28H, (U08)(Addr >> SHIFT_08BIT));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R29H, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x02);	/* AIST=1	*/
		}

		/*--- Birst Read ---*/
		if (bRet == TRUE)
		{
			#if YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST

				YVC_BURST_READ_PORT(YVC1_PORT_GENTBL_DATA, Buff, Num);

			#else

				for (u16Count=0; u16Count<Num ;u16Count++)
				{
					Buff[u16Count] = YVC_READ_PORT(YVC1_PORT_GENTBL_DATA);
				}

			#endif

			/*--- Auto increment bit reset ---*/
			bRet = YVC1_WriteReg(REG_R27H, 0);		/* AIST=0	*/
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WriteTbls                                            */
/* Contents         : General table continuous write                            */
/* Argument         : U16   Addr;   (W)General table address                    */
/*                    U08   *Buff;  (W)Write buffer pointer                     */
/*                    U16   Num;    (W)Number of bytes to write                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WriteTbls(U16 Addr, const U08 *Buff, U16 Num)
{
	BOOL bRet = TRUE;
	#if !(YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST)
		U16 u16Count = 0;
	#endif

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Buff == (U08 *)NULL) ||
	     (Num  == 0) )
	{
		bRet = FALSE;
	}

	/*--- General Table ---*/
	if(Addr < YVC1_EXGTBL1_START_ADDR)
	{
		if((Addr + Num) > YVC1_EXGTBL1_START_ADDR)
		{
			bRet = FALSE;
		}
	}
	/*--- Extended General Table1 ---*/
	else if(Addr < YVC1_EXGTBL1_START_ADDR + YVC1_EXGTBL_SIZE)
	{
		if((Addr + Num) > YVC1_EXGTBL1_START_ADDR + YVC1_EXGTBL_SIZE)
		{
			bRet = FALSE;
		}
	}
	/*--- Extended General Table2 ---*/
	else if((YVC1_EXGTBL2_START_ADDR <= Addr) && (Addr < YVC1_EXGTBL2_START_ADDR + YVC1_EXGTBL_SIZE))
	{
		if((Addr + Num) > YVC1_EXGTBL2_START_ADDR + YVC1_EXGTBL_SIZE)
		{
			bRet = FALSE;
		}
	}
	else
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- General table address set ---*/
		bRet = YVC1_WriteReg(REG_R28H, (U08)(Addr >> SHIFT_08BIT));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R29H, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x02);	/* AIST=1	*/
		}

		/*--- Birst Write ---*/
		if (bRet == TRUE)
		{
			#if YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST

				YVC_BURST_WRITE_PORT(YVC1_PORT_GENTBL_DATA, Buff, Num);
//                              YVC_BurstWritePort(YVC1_PORT_REG_DATA, Buff, Num);

			#else

				for (u16Count=0; u16Count<Num ;u16Count++)
				{
					YVC_WRITE_PORT(YVC1_PORT_GENTBL_DATA, Buff[u16Count]);
				}

			#endif

			/*--- Auto increment bit reset ---*/
			bRet = YVC1_WriteReg(REG_R27H, 0);		/* AIST=0	*/
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadPlts                                             */
/* Contents         : Color palette continuous read                             */
/* Argument         : U16   Addr;   (W)Color palette address                    */
/*                    U08   *Buff;  (R)Read buffer pointer                      */
/*                    U16   Num;    (W)Number of bytes to read                  */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ReadPlts(U16 Addr, U08 *Buff, U16 Num)
{
	BOOL bRet = TRUE;
	#if !(YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST)
		U16  u16Cnt = 0;
	#endif

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_COLORPLT_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_COLORPLT_SIZE) || 
		 (Buff == (U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- Color palette address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)(Addr >> SHIFT_08BIT));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01);	/* AICP=1	*/
		}

		if (bRet == TRUE)
		{
			#if YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST

				YVC_BURST_READ_PORT(YVC1_PORT_PLT_GAM_WRP_DATA, Buff, Num);

			#else

				/*--- It writes pallette data in the auto increment ---*/
				for (u16Cnt=0; u16Cnt<Num ;u16Cnt++)
				{
					Buff[u16Cnt] = YVC_READ_PORT(YVC1_PORT_PLT_GAM_WRP_DATA);
				}

			#endif

			/*--- Auto increment bit set ---*/
			bRet = YVC1_WriteReg(REG_R27H, 0);		/* AICP=0	*/
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WritePlts                                            */
/* Contents         : Color palette continuous write                            */
/* Argument         : U16   Addr;   (W)Color palette address                    */
/*                    U08   *Buff;  (W)Write buffer pointer                     */
/*                    U16   Num;    (W)Number of bytes to write                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WritePlts(U16 Addr, const U08 *Buff, U16 Num)
{
	BOOL bRet = TRUE;
	#if !(YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST)
		U16  u16Cnt = 0;
	#endif

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_COLORPLT_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_COLORPLT_SIZE) ||
		 (Buff == (const U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- Color palette address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)(Addr >> SHIFT_08BIT));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01);	/* AICP=1	*/
		}

		/*--- It writes pallette data in the auto increment ---*/
		if(bRet == TRUE)
		{
			#if YVC_BUS_SERIAL && YVC_BUS_SERIAL_BURST

        		YVC_BURST_WRITE_PORT(YVC1_PORT_PLT_GAM_WRP_DATA, Buff, Num);
//                      YVC_BurstWritePort(YVC1_PORT_REG_DATA, Buff, Num);
                                

			#else

				for (u16Cnt=0; u16Cnt<Num ;u16Cnt++)
				{
					YVC_WRITE_PORT(YVC1_PORT_PLT_GAM_WRP_DATA, Buff[u16Cnt]);
				}

			#endif

			/*--- Auto increment bit set ---*/
			bRet = YVC1_WriteReg(REG_R27H, 0);		/* AICP=0	*/
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadGamTbls                                          */
/* Contents         : Gamma Correction Table continuous read                    */
/* Argument         : U16   Addr;   (W)Gamma Correction Table address           */
/*                    U08   *Buff;  (R)Read buffer pointer                      */
/*                    U16   Num;    (W)Number of bytes to read                  */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ReadGamTbls(U16 Addr, U08 *Buff, U16 Num)
{
	BOOL bRet     = TRUE;
	U16  u16Count = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_WRPTBL_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_GAMTBL_SIZE) || 
		 (Buff == (const U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		Addr += YVC1_INTTBL_GAMTBL_SADDR;

		/*--- Warping Parameter Table address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)((Addr >> SHIFT_08BIT) & MASK_3FH));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01); /* AICP=1 */
		}

		for (u16Count=0; u16Count<Num ;u16Count++)
		{
			Buff[u16Count] = YVC_READ_PORT(YVC1_PORT_PLT_GAM_WRP_DATA);
		}

		/*--- Auto increment bit reset ---*/
		bRet = YVC1_WriteReg(REG_R27H, 0x00); /* AICP=0 */
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WriteGamTbls                                         */
/* Contents         : Gamma Correction Table continuous write                   */
/* Argument         : U16   Addr;   (W)Gamma Correction Table  address          */
/*                    U08   *Buff;  (W)Write buffer pointer                     */
/*                    U16   Num;    (W)Number of bytes to write                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WriteGamTbls(U16 Addr, const U08 *Buff, U16 Num)
{
	BOOL bRet     = TRUE;
	U16  u16Count = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_WRPTBL_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_GAMTBL_SIZE) ||
		 (Buff == (const U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		Addr += YVC1_INTTBL_GAMTBL_SADDR;

		/*--- Warping Parameter Table address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)((Addr >> SHIFT_08BIT) & MASK_3FH));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01); /* AICP=1 */
		}

		for (u16Count=0; u16Count<Num ;u16Count++)
		{
			YVC_WRITE_PORT(YVC1_PORT_PLT_GAM_WRP_DATA, Buff[u16Count]);
		}

		/*--- Auto increment bit reset ---*/
		bRet = YVC1_WriteReg(REG_R27H, 0x00); /* AICP=0 */
	}

	return bRet;

}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadWrpTbls                                          */
/* Contents         : Warping Parameter Table continuous read                   */
/* Argument         : U16   Addr;   (W)Warping Parameter Table address          */
/*                    U08   *Buff;  (R)Read buffer pointer                      */
/*                    U16   Num;    (W)Number of bytes to read                  */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ReadWrpTbls(U16 Addr, U08 *Buff, U16 Num)
{
	BOOL bRet     = TRUE;
	U16  u16Count = 0;
	
	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_WRPTBL_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_WRPTBL_SIZE) ||
		 (Buff == (const U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		Addr += YVC1_INTTBL_WRPTBL_SADDR;

		/*--- Warping Parameter Table address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)((Addr >> SHIFT_08BIT) & MASK_7FH));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01); /* AICP=1 */
		}

		for (u16Count=0; u16Count<Num ;u16Count++)
		{
			Buff[u16Count] = YVC_READ_PORT(YVC1_PORT_PLT_GAM_WRP_DATA);
		}

		/*--- Auto increment bit reset ---*/
		bRet = YVC1_WriteReg(REG_R27H, 0); /* AICP=0 */
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WriteWrpTbls                                         */
/* Contents         : Warping Parameter Table continuous write                  */
/* Argument         : U16   Addr;   (W)Warping Parameter Table  address         */
/*                    U08   *Buff;  (W)Write buffer pointer                     */
/*                    U16   Num;    (W)Number of bytes to write                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WriteWrpTbls(U16 Addr, const U08 *Buff, U16 Num)
{
	BOOL bRet     = TRUE;
	U16  u16Count = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (Addr >= YVC1_WRPTBL_SIZE) ||
		 (Num == 0) ||
		 ((Addr + Num) > YVC1_WRPTBL_SIZE) ||
		 (Buff == (const U08 *)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		Addr += YVC1_INTTBL_WRPTBL_SADDR;

		/*--- Warping Parameter Table address set ---*/
		bRet = YVC1_WriteReg(REG_R2AH, (U08)((Addr >> SHIFT_08BIT) & MASK_7FH));
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R2BH, (U08)(Addr & MASK_FFH));
		}

		/*--- Auto increment bit set ---*/
		if (bRet == TRUE)
		{
			bRet = YVC1_WriteReg(REG_R27H, 0x01); /* AICP=1 */
		}

		for (u16Count=0; u16Count<Num ;u16Count++)
		{
			YVC_WRITE_PORT(YVC1_PORT_PLT_GAM_WRP_DATA, Buff[u16Count]);
		}

		/*--- Auto increment bit reset ---*/
		bRet = YVC1_WriteReg(REG_R27H, 0); /* AICP=0 */
	}

	return bRet;

}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_DmaInit                                              */
/* Contents         : DMA Initialization                                        */
/* Argument         : T_YVC1_DMA_INIT *tYvc1DmaInit                             */
/*                    [-> U08 SDCOMP    (W) Source-data compression selection ] */
/*                    [-> U08 DESTSEL   (W) Destination selection             ] */
/*                    [-> U32 SOCEAD    (W) Source address                    ] */
/*                    [-> U16 DESTAD    (W) Destination address               ] */
/*                    [-> U16 BYTECNT   (W) Transmission number of bytes      ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_DmaInit(const T_YVC1_DMA_INIT *tYvc1DmaInit)
{
	BOOL bRet             = TRUE;
	T_Y643_RB0H tY643RB0h = {0};
	T_Y643_RB1H tY643RB1h = {0};
	T_Y643_RB2H tY643RB2h = {0};
	T_Y643_RB3H tY643RB3h = {0};
	T_Y643_RB4H tY643RB4h = {0};
	T_Y643_RB5H tY643RB5h = {0};
	T_Y643_RB6H tY643RB6h = {0};
	T_Y643_RB7H tY643RB7h = {0};
	U08 u08SDCOMP         = 0;
	U08 u08DESTSEL        = 0;
	U32 u32SOCEAD         = 0;
	U16 u16DESTAD         = 0;
	U16 u16BYTECNT        = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if (tYvc1DmaInit == (const T_YVC1_DMA_INIT*)NULL)
	{
		bRet = FALSE;
	}

	if (bRet == TRUE)
	{
		u08SDCOMP  = tYvc1DmaInit->SDCOMP;
		u08DESTSEL = tYvc1DmaInit->DESTSEL;
		u32SOCEAD  = tYvc1DmaInit->SOCEAD;
		u16DESTAD  = tYvc1DmaInit->DESTAD;
		u16BYTECNT = tYvc1DmaInit->BYTECNT;

		if ( (u08DESTSEL > YVC1_DMA_DESTSEL_MAX) ||
			 (u32SOCEAD > YVC1_DMA_SOCEAD_MAX) )
		{
			bRet = FALSE;
		}

		if (u08DESTSEL == YVC1_DMA_DESTSEL_GAM)
		{
			if ( (u08SDCOMP > YVC1_DMA_SDCOMP_GAM_MAX) ||
				 (u16DESTAD > YVC1_DMA_DESTAD_MAX) ||
				 (u16BYTECNT > YVC1_DMA_BYTECNT_GAM_MAX) )
			{
				bRet = FALSE;
			}
		}
		else
		{	/* YVC1_DMA_DESTSEL_WRP	*/
			if (u08SDCOMP > YVC1_DMA_SDCOMP_WRP_MAX)
			{
				bRet = FALSE;
			}
			if (u08SDCOMP == YVC1_DMA_SDCOMP_ON)
			{
				if ( (u16DESTAD > YVC1_DMA_DESTAD_MAX) ||
					 (u16BYTECNT > YVC1_DMA_BYTECNT_WRP_MAX) )
				{
					bRet = FALSE;
				}
			}
			else
			{	/* YVC1_DMA_SDCOMP_OFF	*/
				if ( ((u16DESTAD > YVC1_DMA_DESTAD_4N_MAX) || ((u16DESTAD % YVC1_DMA_DESTAD_4N)!=0)) ||
					 ((u16BYTECNT > YVC1_DMA_BYTECNT_WRP_4N_MAX) || ((u16BYTECNT % YVC1_DMA_BYTECNT_WRP_4N)!=0)) )
				{
					bRet = FALSE;
				}
			}
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643RB0h.BIT.SDCOMP    = u08SDCOMP;
		tY643RB0h.BIT.SOCEAD_24 = (U08)(( u32SOCEAD  >> SHIFT_24BIT ) & MASK_03H);
		tY643RB1h.BIT.SOCEAD_16 = (U08)(( u32SOCEAD  >> SHIFT_16BIT ) & MASK_FFH);
		tY643RB2h.BIT.SOCEAD_8  = (U08)(( u32SOCEAD  >> SHIFT_08BIT ) & MASK_FFH);
		tY643RB3h.BIT.SOCEAD_0  = (U08)(( u32SOCEAD                 ) & MASK_FFH);
		tY643RB4h.BIT.DESTSEL   = u08DESTSEL;
		tY643RB4h.BIT.DESTAD_8  = (U08)(( u16DESTAD  >> SHIFT_08BIT ) & MASK_1FH);
		tY643RB5h.BIT.DESTAD_0  = (U08)(( u16DESTAD                 ) & MASK_FFH);
		tY643RB6h.BIT.BYTECNT_8 = (U08)(( u16BYTECNT >> SHIFT_08BIT ) & MASK_3FH);
		tY643RB7h.BIT.BYTECNT_0 = (U08)(( u16BYTECNT                ) & MASK_FFH);

		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_RB0H,tY643RB0h.BYTE);
		YVC1_WriteReg(REG_RB1H,tY643RB1h.BYTE);
		YVC1_WriteReg(REG_RB2H,tY643RB2h.BYTE);
		YVC1_WriteReg(REG_RB3H,tY643RB3h.BYTE);
		YVC1_WriteReg(REG_RB4H,tY643RB4h.BYTE);
		YVC1_WriteReg(REG_RB5H,tY643RB5h.BYTE);
		YVC1_WriteReg(REG_RB6H,tY643RB6h.BYTE);
		YVC1_WriteReg(REG_RB7H,tY643RB7h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_DmaCtrl                                              */
/* Contents         : DMA Control                                               */
/* Argument         : BOOL Enable   (W) TRUE  : DMA Start                       */
/*                                      FALSE : DMA Forced termination          */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_DmaCtrl(BOOL Enable)
{
	BOOL bRet             = TRUE;
	BOOL bPlgFlg          = FALSE;
	BOOL bPlgMaxFlg       = FALSE;
	U32 u32Count          = 0UL;
	T_Y643_RB8H tY643RB8h = {0};

	EXCTRL_PORT_LOCK_REG

	/*--- DMA Clear ---*/
	tY643RB8h.BIT.DMACLR = 1;
	YVC1_WriteReg(REG_RB8H,tY643RB8h.BYTE);

	/*--- DMA Clear Wait ---*/
	do {
		YVC1_ReadReg(REG_RB8H,&tY643RB8h.BYTE);

		/*--- R#B8h DMACLR polling ---*/
		if(tY643RB8h.BIT.DMACLR == 0)
		{
			bPlgFlg = TRUE;
		}
		else
		{
			bPlgFlg = FALSE;
		}

		/*--- TimeOut Check ---*/
		if(bPlgFlg == FALSE)
		{
			u32Count++;
			if(u32Count > Y643_POLLING_MAX)
			{
				bRet       = FALSE;
				bPlgMaxFlg = TRUE;
			}
		}
	}while( (bPlgFlg == FALSE) && (bPlgMaxFlg == FALSE) );

	/*--- DMA Start ---*/
	if( (bPlgFlg == TRUE) && (Enable == TRUE) )
	{
		tY643RB8h.BIT.DSTART = 1;
		YVC1_WriteReg(REG_RB8H,tY643RB8h.BYTE);
	}

	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_Init                                                 */
/* Contents         : Device initialization                                     */
/* Argument         : T_YVC1_DATA *tYvc1Data    (W) Initialize Data             */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_Init(const T_YVC1_DATA *tYvc1Data)
{
	BOOL bRet       = TRUE;
	BOOL bRet1      = TRUE;
	BOOL bRet2      = TRUE;
	BOOL bRet3      = TRUE;
	BOOL bRet4      = TRUE;
	BOOL bRet5      = TRUE;
	BOOL bRet6      = TRUE;
	BOOL bRet7      = TRUE;
	BOOL bRet8      = TRUE;
	BOOL bRet9      = TRUE;
	BOOL bRet10     = TRUE;
	U08 u08PLL_R[2] = {0};
	T_Y643_R30H tY643R30h = {0};
	T_Y643_R3EH tY643R3EH = {0};
	T_Y643_R3FH tY643R3FH = {0};
	T_Y643_R40H tY643R40H = {0};
	T_Y643_R41H tY643R41H = {0};

	U16 u16LyrEndAddr = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if( (tYvc1Data == (T_YVC1_DATA*)NULL) ||
	    (YVC_LYR_ADDR_CPU > YVC1_CPULYR_ADDR_MAX) ||
	    (YVC_LYR_ADDR_CPU % YVC1_CPULYR_ADDR_12N != 0) ||
	    (YVC_LYR_NUM_CPU  > YVC1_CPULYR_NUM_MAX) ||
	    ((YVC_LYR_ADDR_CPU + (YVC_LYR_NUM_CPU * YVC1_LYR_SIZE)) > YVC1_CPULYR_ADDR_END_MAX) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- In the case of PLL setting judgment PLLR and PLLF=0,  */
		/*                    it is judged as a terminal setup.  ---*/
		if ( ((tYvc1Data->Clk[0] & YVC1_PLLR_MASK) == 0x00) &&
			 ((tYvc1Data->Clk[1] & YVC1_PLLF_MASK) == 0x00) )
		{
			/*------------------------------------------------------------------*/
			/* Pin Controlled PLL Configuration                                 */
			/*------------------------------------------------------------------*/
			/* 1.Hardware Reset                                                 */
			/*   It needs to be carried out before this function.               */
			/*                                                                  */
			/* 2.PLL Lock-up Phase                                              */
			/*   P#2: INIEND becomes 1 when PLL achieves lock.                  */
			/*   *PLL Lock-up Phase (XIN clock cycle x 20000)                   */
			/*    takes 1 ms (at 20 MHz) to 3.4 ms (at 6 MHz).                  */

			TIMER_WAIT_MSEC(YVC1_PLL_LOCKUP_WAIT);
			bRet = s_INIENDWaitPolling();

			/* 3.Reconfiguration Registers                                      */
			/*   (1) Set Clock Control register (R#01h,R#03h).                  */
			/*   (2) Set Display Scan Control register (R#06h-#1Ch).            */
			/*       (*To use the timing controller function,set Timing         */
			/*         Controller register (R#67h-#7Dh).                        */

			if (bRet == TRUE)
			{
				bRet1 = YVC1_WriteReg(REG_R01H, tYvc1Data->Clk[0]);		/* (1)	*/
				bRet2 = YVC1_WriteReg(REG_R03H, tYvc1Data->Clk[2]);
				if((bRet1 == TRUE) && (bRet2 == TRUE))
				{
					bRet = TRUE;
				}
				else
				{
					bRet = FALSE;
				}

				if (bRet == TRUE)
				{														/* (2)	*/
					bRet1 = YVC1_WriteRegs(REG_R06H, tYvc1Data->Disp, 23);
					bRet2 = YVC1_WriteReg(REG_R31H, tYvc1Data->VideoOut);
					bRet3 = YVC1_WriteRegs(REG_R67H, tYvc1Data->TCON, 23);
					if ( (bRet1 == TRUE) && (bRet2 == TRUE) && (bRet3 == TRUE) )
					{
						bRet = TRUE;
					}
					else
					{
						bRet = FALSE;
					}
				}
			}
		}
		else
		{
			/*------------------------------------------------------------------*/
			/* Register Controlled PLL Configuration                            */
			/*------------------------------------------------------------------*/
			/* 1.Hardware Reset                                                 */
			/*   It needs to be carried out before this function.               */
			/* 2.PLL Lock-up Phase                                              */
			/*   P#2: INIEND becomes 1 when PLL achieves lock.                  */
			/*   *PLL Lock-up Phase (XIN clock cycle x 20000) takes             */
			/*    1ms (at 20MHz) to 3.4ms (at 6 MHz).                           */

			TIMER_WAIT_MSEC(YVC1_PLL_LOCKUP_WAIT);
			bRet = s_INIENDWaitPolling();

			/* 3.Configurating PLL Clock Divider Ratio                          */
			/*   (1) Set the following registers: R#01h: PLLR[4:0],             */
			/*                                    R#01h-#02h: PLLF[8:0].        */
			/*   (2) Write 1 into R#00h: SR to execute the software reset.      */
			/*       *The values specified with R#01h: PLLR[4:0]                */
			/*        and R#01h-#02h: PLLF[8:0] are used for the PLL operation. */
			/*   (3) Wait until P#2: INIEND becomes 1 which indicates PLL Reset */
			/*       Phase and PLL Lock-up Phase have completed.                */
			/*       *Reset Phase (256 x XIN clock cycles) takes                */
			/*        12.8us(at 20MHz) to 42.7us (at 6 MHz).                    */
			/*       *Lock-up Phase (20000 x XIN clock cycles) takes            */
			/*        1ms(at 20 MHz) to 3.4ms (at 6 MHz).                       */
			/*   (4) Read from the registers R#04h: RPLLR[4:0] and R#04h-#05h:  */
			/*       RPLLF[8:0] to confirm that their values match the value    */
			/*       written in the step (1).                                   */

			if (bRet == TRUE)
			{
				bRet1 = s_WriteRegPll(REG_R01H, tYvc1Data->Clk[0]);		/* (1)	*/
				bRet2 = s_WriteRegPll(REG_R02H, tYvc1Data->Clk[1]);
				if((bRet1 == TRUE) && (bRet2 == TRUE))
				{
					bRet = TRUE;
				}
				else
				{
					bRet = FALSE;
				}

				if (bRet == TRUE)
				{
					bRet = s_WriteRegPll(REG_R00H, YVC1_SR);			/* (2)	*/
					if (bRet == TRUE)
					{
						TIMER_WAIT_MSEC(YVC1_PLL_LOCKUP_WAIT);			/* (3)	*/
						bRet = s_INIENDWaitPolling();
						if (bRet == TRUE)
						{												/* (4)	*/
							bRet1 = YVC1_ReadReg(REG_R04H, &u08PLL_R[0]);
							bRet2 = YVC1_ReadReg(REG_R05H, &u08PLL_R[1]);
							if ( (bRet1 == TRUE) &&
								 (bRet2 == TRUE) && 
								 ((tYvc1Data->Clk[0] & YVC1_PLLR_MASK) == (u08PLL_R[0] & YVC1_PLLR_MASK)) &&
								 ((tYvc1Data->Clk[1] & YVC1_PLLF_MASK) == (u08PLL_R[1] & YVC1_PLLF_MASK)) )
							{
								bRet = TRUE;
							}
							else
							{
								bRet = FALSE;
							}
						}
					}
				}
			}

			/* 4.Reconfiguration Registers                                      */
			/*   (1) Set Clock Control register (R#01h,R#03h).                  */
			/*   (2) Set Display Scan Control register (R#06h-#1Ch).            */
			/*       (*To use the timing controller function,                   */
			/*         set Timing Controller register (R#67h-#7Dh).             */

			if (bRet == TRUE)
			{															/* (1)	*/
				bRet1 = YVC1_WriteReg(REG_R01H, tYvc1Data->Clk[0]);
				bRet2 = YVC1_WriteReg(REG_R03H, tYvc1Data->Clk[2]);
				if((bRet1 == TRUE) && (bRet2 == TRUE))
				{
					bRet = TRUE;
				}
				else
				{
					bRet = FALSE;
				}

				if (bRet == TRUE)
				{														/* (2)	*/
					bRet1 = YVC1_WriteRegs(REG_R06H, tYvc1Data->Disp, 23);
					bRet2 = YVC1_WriteReg(REG_R31H, tYvc1Data->VideoOut);
					bRet3 = YVC1_WriteRegs(REG_R67H, tYvc1Data->TCON, 23);
					if ( (bRet1 == TRUE) && (bRet2 == TRUE) && (bRet3 == TRUE) )
					{
						bRet = TRUE;
					}
					else
					{
						bRet = FALSE;
					}
				}
			}
		}

		/*----------------------------------------------------------------------*/
		/* R#55h-R#5Bh pattern memory control setup                             */
		/* Set the YVC1_PmemSet() processing according to use environment.      */
		/*----------------------------------------------------------------------*/
		if (bRet == TRUE)
		{
			bRet = YVC1_PmemSet();
		}

		/*----------------------------------------------------------------------*/
		/* General table Init                                                   */
		/*----------------------------------------------------------------------*/
		#if YVC_GENERAL_TABLE_INIT   // 2018.06.21. 주석 처리 함..
		if (bRet == TRUE)
		{
			bRet = s_GeneralTblInit();    // 많은 시간 지연 되는 루틴..
		}
		#endif

		/*----------------------------------------------------------------------*/
		/* Set the extended layer function and the size of alpha values         */
		/*----------------------------------------------------------------------*/
		if (bRet == TRUE)
		{
			bRet4 = YVC1_ReadReg(REG_R30H, &tY643R30h.BYTE);
			tY643R30h.BIT.EXLY    = YVC1_EXLY;
			tY643R30h.BIT.ALPBSEL = YVC1_ALPBSEL;
			bRet5 = YVC1_WriteReg(REG_R30H, tY643R30h.BYTE);

			if((bRet4 == TRUE) && (bRet5 == TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}
		}

		/*----------------------------------------------------------------------*/
		/* Set Host Controlled Layer Registers                                  */
		/*----------------------------------------------------------------------*/
		if(bRet == TRUE)
		{
			bRet6 = YVC1_ReadReg(REG_R3EH, &tY643R3EH.BYTE);

			if(YVC_LYR_NUM_CPU == 0)
			{
				tY643R3EH.BIT.LYDC = 0;
				bRet7 = YVC1_WriteReg(REG_R3EH, tY643R3EH.BYTE);
			}
			else
			{

				u16LyrEndAddr   = YVC_LYR_ADDR_CPU + (YVC_LYR_NUM_CPU - 1) * YVC1_LYR_SIZE;
				EXCTRL_PORT_LOCK_REG

				tY643R3EH.BIT.LYSAC_8 = ((YVC_LYR_ADDR_CPU >> SHIFT_08BIT) & MASK_0FH);
				tY643R3FH.BIT.LYSAC_2 = ((YVC_LYR_ADDR_CPU >> SHIFT_02BIT) & MASK_3FH);
				tY643R40H.BIT.LYEAC_8 = ((u16LyrEndAddr    >> SHIFT_08BIT) & MASK_0FH);
				tY643R41H.BIT.LYEAC_2 = ((u16LyrEndAddr    >> SHIFT_02BIT) & MASK_3FH);
				bRet7  = YVC1_WriteReg(REG_R3EH, tY643R3EH.BYTE);
				bRet8  = YVC1_WriteReg(REG_R3FH, tY643R3FH.BYTE);
				bRet9  = YVC1_WriteReg(REG_R40H, tY643R40H.BYTE);
				bRet10 = YVC1_WriteReg(REG_R41H, tY643R41H.BYTE);

				EXCTRL_PORT_UNLOCK_REG

			}


			if( (bRet6 == TRUE) && (bRet7 == TRUE) && (bRet8 == TRUE) && (bRet9 == TRUE) && (bRet10 == TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_GetVer                                               */
/* Contents         : Device & Driver version acquisition                       */
/* Argument         : U08 *DevVer   (R) Device version read pointer             */
/*                    C08 *DrvVer   (R) Driver version read pointer             */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_GetVer(U08 *DevVer, C08 *DrvVer)
{
	BOOL bRet  = TRUE;
	C08 *c08p  = (C08*)NULL;
	U08 u08Cnt = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (DevVer == (U08*)NULL) ||
		 (DrvVer == (C08*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- Device Ver ---*/
		/* Register data set */
		bRet = YVC1_ReadReg(REG_R00H, DevVer);

		/*--- Driver Ver ---*/
		if (bRet == TRUE)
		{
			c08p = YVC1_DRIVER_VERSION;
			for (u08Cnt=0; u08Cnt<sizeof(YVC1_DRIVER_VERSION); u08Cnt++)
			{
				DrvVer[u08Cnt] = c08p[u08Cnt];
			}
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_FlipTbl                                              */
/* Contents         : CPU control layer flip setting                            */
/* Argument         : void                                                      */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_FlipTbl(void)
{
	BOOL bRet       = TRUE;
	BOOL bRet1      = TRUE;
	BOOL bRet2      = TRUE;
	BOOL bRet3      = TRUE;
	BOOL bRet4      = TRUE;
	BOOL bRet5      = TRUE;
	BOOL bRet6      = TRUE;
	BOOL bRet7      = TRUE;
	BOOL bRet8      = TRUE;
	BOOL bRet9      = TRUE;

	U08 Reg3EHData  = 0;
	U08 Reg3FHData  = 0;
	U08 Reg40HData  = 0;
	U08 Reg41HData  = 0;

	T_Y643_PORT_FLAG1 tY643PortFlag1 = {0};

	bRet1 = YVC1_ReadReg(REG_R3EH, &Reg3EHData);
	bRet2 = YVC1_ReadReg(REG_R3FH, &Reg3FHData);
	bRet3 = YVC1_ReadReg(REG_R40H, &Reg40HData);
	bRet4 = YVC1_ReadReg(REG_R41H, &Reg41HData);

	if ((Reg3EHData & YVC1_LYBSELC_MASK) != YVC1_LYBSELC_TBL1)
	{
		/*--- Flip General Table 1 ---*/
		Reg3EHData &= (U08)(~YVC1_LYBSELC_MASK);
	}
	else
	{
		/*--- Flip General Table 2 ---*/
		Reg3EHData |= (U08)(YVC1_LYBSELC_MASK);
	}

	bRet5 = YVC1_WriteReg(REG_R3EH, Reg3EHData);
	bRet6 = YVC1_WriteReg(REG_R3FH, Reg3FHData);
	bRet7 = YVC1_WriteReg(REG_R40H, Reg40HData);
	bRet8 = YVC1_WriteReg(REG_R41H, Reg41HData);

	/*--- P#6 FB flag reset ---*/
	tY643PortFlag1.BIT.FB = 1;
	YVC_WRITE_PORT(YVC1_PORT_FLG_STATUS1, tY643PortFlag1.BYTE);

	/*--- FB Wait ---*/
	bRet9 = s_FBWait();

	if ( (bRet1 == TRUE) && (bRet2 == TRUE) && (bRet3 == TRUE) && (bRet4 == TRUE) &&
		 (bRet5 == TRUE) && (bRet6 == TRUE) && (bRet7 == TRUE) && (bRet8 == TRUE) &&
		 (bRet9 == TRUE) )
	{
		bRet = TRUE;
	}
	else
	{
		bRet = FALSE;
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : s_FBWait                                                  */
/* Contents         : General table switching completion wait                   */
/* Argument         : void                                                      */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_FBWait(void)
{
	BOOL bRet        = TRUE;
	BOOL bPollingFlg = FALSE;
	U32 u32Count     = 0UL;
	T_Y643_PORT_FLAG1 tY643PortFlag1 = {0};

	/*--- P#6 Status FB flag pauling 	---*/
	do
	{
		tY643PortFlag1.BYTE = YVC_READ_PORT(Y643_PORT_FLG_STATUS1);
		if(tY643PortFlag1.BIT.FB != 0)
		{
			bPollingFlg = TRUE;
		}
		else
		{
			bPollingFlg = FALSE;
			u32Count++;
			if(u32Count > Y643_POLLING_MAX)
			{
				bRet = FALSE;
				bPollingFlg = TRUE;
			}
		}
	}while(bPollingFlg == FALSE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VBWait                                               */
/* Contents         : Vertical blank wait                                       */
/* Argument         : U32 Count (W) Number of vertical blank periods            */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VBWait(U32 Count)
{
	BOOL bRet      = TRUE;
	BOOL bRet1     = TRUE;
	BOOL bRet2     = TRUE;
	BOOL bRet3     = TRUE;
	U32 u32VBCount = 0UL;

	/*--- VB Start Wait ---*/
	bRet1 = s_VBWaitPolling(0);

	for(u32VBCount=0; u32VBCount<Count; u32VBCount++)
	{
		/*--- VB End Wait ---*/
		bRet2 = s_VBWaitPolling(1);

		/*--- VB Start Wait ---*/
		bRet3 = s_VBWaitPolling(0);
	}

	if ( (bRet1 == TRUE) && (bRet2 == TRUE) && (bRet3 == TRUE) )
	{
		bRet = TRUE;
	}
	else
	{
		bRet = FALSE;
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : s_VBWaitPolling                            (Sub Function) */
/* Contents         : VB status acquisition polling                             */
/* Argument         : U08 Mode      (W) Polling mode  0:VB Start Wait           */
/*                                                    1:VB End   Wait           */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_VBWaitPolling(U08 Mode)
{
	BOOL bRet        = TRUE;
	BOOL bPollingFlg = FALSE;
	U32 u32Count     = 0UL;
	T_Y643_PORT_FLAG1 tY643PortFlag1 = {0};

	if(Mode == 0)
	{
		/*--------------------------------------*/
		/* VB Start Wait                        */
		/*--------------------------------------*/
		do
		{
			/*---	P#6 Flag & Status read 	---*/
			tY643PortFlag1.BYTE = YVC_READ_PORT(Y643_PORT_FLG_STATUS1);
			
			if(tY643PortFlag1.BIT.VB != 0)
			{
				bPollingFlg = TRUE;
			}
			else
			{
				bPollingFlg = FALSE;
				u32Count++;
				if(u32Count > Y643_POLLING_MAX)
				{
					bRet = FALSE;
					bPollingFlg = TRUE;
				}
			}
		}while(bPollingFlg == FALSE);
	}
	else
	{
		do
		{
			/*-----------------------------------------------------------
			 * VB End Wait
			 *----------------------------------------------------------*/

			/*---	P#6 Flag & Status read 	---*/
			tY643PortFlag1.BYTE = YVC_READ_PORT(Y643_PORT_FLG_STATUS1);

			if(tY643PortFlag1.BIT.VB == 0)
			{
				bPollingFlg = TRUE;
			}
			else
			{
				bPollingFlg = FALSE;
				u32Count++;
				if(u32Count > Y643_POLLING_MAX)
				{
					bRet = FALSE;
					bPollingFlg = TRUE;
				}
			}
		}while(bPollingFlg == FALSE);
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : s_INIENDWaitPolling                       (Sub Function)  */
/* Contents         : Initialization completion status(INIEND) polling          */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_INIENDWaitPolling(void)
{
	BOOL bRet        = TRUE;
	BOOL bPollingFlg = FALSE;
	U32  u32Count    = 0;
        
        U08 data = 0;

	/*--- INIEND polling 	---*/
	do
	{
		//if((YVC_READ_PORT(YVC1_PORT_PTNMEM_READACS) & YVC1_INIEND_CHK) != 0)
          data = YVC_READ_PORT(YVC1_PORT_PTNMEM_READACS);
	      //if((YVC_READ_PORT(YVC1_PORT_PTNMEM_READACS) & YVC1_INIEND_CHK) == YVC1_INIEND_CHK)
		if((data & YVC1_INIEND_CHK) == YVC1_INIEND_CHK)
		
              {
			bPollingFlg = TRUE;
		}
		else
		{
			u32Count += YVC1_INIEND_COUNT_UP;
			TIMER_WAIT_MSEC(YVC1_INIEND_WAIT);
			if(u32Count > YVC1_POLLING_MAX)
			{
				bRet = FALSE;
				bPollingFlg = TRUE;
			}
		}
	}while(bPollingFlg == FALSE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : s_GeneralTblInit                          (Sub Function)  */
/* Contents         : General table Init                                        */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
#if YVC_GENERAL_TABLE_INIT
static BOOL s_GeneralTblInit(void)
{
	BOOL bRet      = TRUE;
	BOOL bRet1     = TRUE;
	BOOL bRet2     = TRUE;
	BOOL bRet3     = TRUE;
	BOOL bRet4     = TRUE;
	U16  u16Addr   = 0;
	U16  u16ExAddr = 0;

	const U08 u08YvcLyrAttrData[YVC1_LYR_SIZE]  = SET_LYR_ERASE_ATTR_DEF;   /* MX4 = 1 */
#if YVC1_EXLY
	const U08 u08YVC1ExLyrData[YVC1_EXLYR_SIZE] = SET_EXLYR_ERASE_ATTR_DEF; /* All zero */
#endif

	if( (YVC_GENERAL_TABLE_INIT_ADDR > YVC1_CPULYR_ADDR_MAX) ||
	    (YVC_GENERAL_TABLE_INIT_ADDR % YVC1_CPULYR_ADDR_12N != 0) ||
	    (YVC_GENERAL_TABLE_INIT_NUM  > YVC1_CPULYR_NUM_MAX) ||
	    ((YVC_GENERAL_TABLE_INIT_ADDR + (YVC_GENERAL_TABLE_INIT_NUM * YVC1_LYR_SIZE)) > YVC1_CPULYR_ADDR_END_MAX) )
	{
		bRet = FALSE;
	}

	/*--- General table Init ---*/
	for (u16Addr=YVC_GENERAL_TABLE_INIT_ADDR; (u16Addr<YVC1_GTBL_INIT_END_ADDR); u16Addr+=YVC1_LYR_SIZE)
	{
		/*--- Standard general table Init ---*/
		bRet1 = YVC1_WriteTbls((u16Addr | YVC1_DOUBLE_BUFF_ADDR), &u08YvcLyrAttrData[0], YVC1_LYR_SIZE);
		bRet2 = YVC1_WriteTbls( u16Addr,                          &u08YvcLyrAttrData[0], YVC1_LYR_SIZE);

		/*--- Extended general table Init ---*/
#if YVC1_EXLY
		/*- Address of extension attribute data = 0x2000 or 0x3000 + (Layer attribude start address / 8 -*/
		u16ExAddr = YVC1_EXGTBL_ADDR + (u16Addr / YVC1_EXGTBL_ADDR_CALC_PARAM);
		bRet3 = YVC1_WriteTbls( u16ExAddr,                        &u08YVC1ExLyrData[0], YVC1_EXLYR_SIZE);
		bRet4 = YVC1_WriteTbls((u16ExAddr|YVC1_DOUBLE_BUFF_ADDR), &u08YVC1ExLyrData[0], YVC1_EXLYR_SIZE);
#endif
	}

	if ( (bRet1 == TRUE) && (bRet2 == TRUE) && (bRet3 == TRUE) && (bRet4 == TRUE) )
	{
		bRet = TRUE;
	}
	else
	{
		bRet = FALSE;
	}

	return bRet;
}
#endif

/*------------------------------------------------------------------------------*/
/* Function name    : s_WriteRegPll                             (Sub Function)  */
/* Contents         : Pll Register 1-byte write                                 */
/* Argument         : U08   RegNo   (W)Register number                          */
/*                  : U08   Data    (W)Write data                               */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_WriteRegPll(U08 RegNo, U08 Data)
{
	BOOL bRet = TRUE;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
/*	if (RegNo >= YVC1_REG_SIZE)
	{
		bRet = FALSE;
	}
Enkhbat disbaled*/
	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		EXCTRL_PORT_LOCK_REG

		/*--- Register data set ---*/
		YVC_WRITE_PORT(YVC1_PORT_REG_SEL, RegNo);
		TIMER_WAIT_MSEC(1);

		YVC_WRITE_PORT(YVC1_PORT_REG_DATA, Data);
		TIMER_WAIT_MSEC(1);

		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : s_PtnMemReadAcs_Reg7_Data                 (Sub Function)  */
/* Contents         : Y643_PORT_PTNMEM_READACS Port Data(CSNEG) read            */
/* Argument         : void                                                      */
/* Return value     : T_Y643_PORT_PTNMEM                                        */
/*------------------------------------------------------------------------------*/
static T_Y643_PORT_PTNMEM s_PtnMemReadAcs_Reg7_Data(void)
{
	T_Y643_PORT_PTNMEM tY643PortPtnMem;
	BOOL bPlgMaxFlg = FALSE;
	U32  u32Count   = 0;

	do {
		tY643PortPtnMem.BYTE = YVC_READ_PORT(Y643_PORT_PTNMEM_READACS);

		/*--- TimeOut Check ---*/
		u32Count++;
		if(u32Count > Y643_POLLING_MAX)
		{
			bPlgMaxFlg = TRUE;
		}
	} while((tY643PortPtnMem.BIT.PMRREQ != 0) && (bPlgMaxFlg == FALSE));

	tY643PortPtnMem.BYTE &= YVC1_REG_CSNEG_MASK;

	return tY643PortPtnMem;
}

/***********************************************************************************************************************
* Function Name: YVC1_GetChecksum
* Description  : 
* Arguments    : 
* Return Value : 
***********************************************************************************************************************/
BOOL YVC1_GetChecksum(U32 startAddr, U32 endAddr , U16 *Checksum)
{
	U08 CSSA[4];
	U08 CSEA[4];
	U08 SUMD[2];
	U08 CHS = 1;
	U16 t_Checksum = 0;
	static U16 calc_checksum_wait = 0;

	// set start address
	CSSA[0] = (U08)((startAddr >> SHIFT_24BIT) );
	CSSA[1] = (U08)((startAddr >> SHIFT_16BIT) );
	CSSA[2] = (U08)((startAddr >> SHIFT_08BIT) );
	CSSA[3] = (U08)((startAddr               ) );

	// set end address
	CSEA[0]    = (U08)((endAddr >> SHIFT_24BIT) );
	CSEA[1]    = (U08)((endAddr >> SHIFT_16BIT) );
	CSEA[2]    = (U08)((endAddr >> SHIFT_08BIT) );
	CSEA[3]    = (U08)((endAddr               ) );

	// write address
	YVC1_WriteRegs(REG_R34H,CSSA,4);
	YVC1_WriteRegs(REG_R38H,CSEA,4);
	// start calc checksum
	YVC1_WriteReg(REG_R33H,CHS);
	do
	{
		TIMER_WAIT_MSEC(5);
		// read status calc checksum
		YVC1_ReadReg(REG_R33H,&CHS);
		
		if( calc_checksum_wait++ >= 199 )
		{
		    calc_checksum_wait = 0;
		    return FALSE;
		}
	}while(CHS != 0);
	YVC1_ReadReg(REG_R3CH,&SUMD[0]);
	YVC1_ReadReg(REG_R3DH,&SUMD[1]);

	t_Checksum = (int)(SUMD[0]) << 8;
	t_Checksum += SUMD[1];
	*Checksum = t_Checksum;
	return TRUE;
}


/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ReadFlag                          									 */
/* Contents         :  Read Flags and Status   	                               							  */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : UINT16                                             */
/*------------------------------------------------------------------------------*/

U16 YVC1_ReadFlag(void )
{
	U16 u16bRet = 0;
	u16bRet = YVC_READ_PORT(YVC1_PORT_FLG_STATUS1);
	u16bRet = YVC_READ_PORT(YVC1_PORT_FLG_STATUS2) | (u16bRet << 8);
	
	return u16bRet;
     
}

/***********************************************************************************************************************
* Function Name: Vsync_Filp_Check
* Description  : Check for Vsync flag and Filp Complete Flag
* Arguments    : void
* Return Value : BOOL TRUE or FALSE
***********************************************************************************************************************/
BOOL Vsync_Filp_Check(void)
{
	T_Y643_PORT_FLAG1 tY643PortFlag1 = {0};
	BOOL bRet = TRUE;	
	
	tY643PortFlag1.BYTE = (U08)(YVC1_ReadFlag() >> 8);
	if( tY643PortFlag1.BIT.FP != 0 )
	{
		tY643PortFlag1.BIT.FP = 1; // clear FP flag
		YVC_WRITE_PORT(YVC1_PORT_FLG_STATUS1,tY643PortFlag1.BYTE);

		bRet = TRUE;
	}
	else
	{
		bRet = FALSE;
	}
	return bRet;
}
/*------------------------------ E O F -----------------------------------------*/

