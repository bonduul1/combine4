/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2015 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Lyr.c $                                    */
/*  Description     : Layer setting function                                    */
/*  Version         : $Rev: 39 $                                               */
/*  Last UpDate Time: $Date:: 2015-01-08 15:38:06#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Nakahara                                                */
/********************************************************************************/

/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc1Ddrv.h"
#include "Yvc1Lyr.h"

/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						P R O T O T Y P E
 *------------------------------------------------------------------------------*/
static BOOL s_TblWrite(U16 Addr, const U08 *Buff, U16 Num);
static BOOL s_TblRead(U16 Addr, U08 *Buff, U16 Num);
static U16 s_LYBSELC_Chk(U16 Addr, U08 Reg3EHData);

#if YVC1_EXLY
static BOOL s_Exly_TblWrite(U16 Addr, const U08 *Buff, U16 Num);
static BOOL s_Exly_TblRead(U16 Addr, U08 *Buff, U16 Num);
static U16 s_EXLY_LYBSELC_Chk(U16 Addr, U08 Reg3EHData);
#endif

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetSprtAttr                                          */
/* Contents         : Sprite Attribute Data setting                             */
/* Argument         : U16 LyrId                      (W) Layer ID               */
/*                  : T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr                     */
/*                                                   (W) Sprite Attribute Data  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetSprtAttr(U16 LyrId, const T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr)
{
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/* Input Check                          */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrSprtAttr == (const T_Y643_LYR_SPRTATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Set Sprite Attribute Data            */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet1 = s_TblWrite(u16Addr, (const U08*)&tY643LyrSprtAttr->BYTE[0], Y643_LYR_SIZE);

		#if YVC1_EXLY
			/*--- Set Extended Layer ---*/
			bRet2 = s_Exly_TblWrite(u16Addr, (const U08*)&tY643LyrSprtAttr->BYTE[12], 1);
			if((bRet1==TRUE) && (bRet2==TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}

		#else

			bRet = bRet1;

		#endif

	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_GetSprtAttr                                          */
/* Contents         : Sprite Attribute Data acquisition                         */
/* Argument         : U16 LyrId                      (W) Layer ID               */
/*                  : T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr                     */
/*                                                   (R) Sprite Attribute Data  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_GetSprtAttr(U16 LyrId, T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr)
{
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrSprtAttr == (const T_Y643_LYR_SPRTATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Get Sprite Attribute Data            */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet1 = s_TblRead(u16Addr, &tY643LyrSprtAttr->BYTE[0], Y643_LYR_SIZE);

		#if YVC1_EXLY
			/*--- Set Extended Layer ---*/
			bRet2 = s_Exly_TblRead(u16Addr, &tY643LyrSprtAttr->BYTE[12], 1);
			if((bRet1==TRUE) && (bRet2==TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}

		#else

			bRet = bRet1;

		#endif
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetFontAttr                                          */
/* Contents         : Font Attribute Data setting                               */
/* Argument         : U16 LyrId                      (W) Layer ID               */
/*                  : T_Y643_LYR_FONTATTR *tY643LyrFontAttr                     */
/*                                                   (W) Font Attribute Data    */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetFontAttr(U16 LyrId, const T_Y643_LYR_FONTATTR *tY643LyrFontAttr)
{
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/* Input Check                          */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrFontAttr == (const T_Y643_LYR_FONTATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Set Font Attribute Data              */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet1 = s_TblWrite(u16Addr, (const U08*)&tY643LyrFontAttr->BYTE[0], Y643_LYR_SIZE);

		#if YVC1_EXLY
			/*--- Set Extended Layer ---*/
			bRet2 = s_Exly_TblWrite(u16Addr, (const U08*)&tY643LyrFontAttr->BYTE[12], 1);
			if((bRet1==TRUE) && (bRet2==TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}

		#else

			bRet = bRet1;

		#endif
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_GetFontAttr                                          */
/* Contents         : Font Attribute Data acquisition                           */
/* Argument         : U16 LyrId                      (W) Layer ID               */
/*                  : T_Y643_LYR_FONTATTR *tY643LyrFontAttr                     */
/*                                                   (R) Font Attribute Data    */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_GetFontAttr(U16 LyrId, T_Y643_LYR_FONTATTR *tY643LyrFontAttr)
{
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/* Input Check                          */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrFontAttr == (const T_Y643_LYR_FONTATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Get Font Attribute Data              */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet1 = s_TblRead(u16Addr, &tY643LyrFontAttr->BYTE[0], Y643_LYR_SIZE);

		#if YVC1_EXLY
			/*--- Set Extended Layer ---*/
			bRet2 = s_Exly_TblRead(u16Addr, &tY643LyrFontAttr->BYTE[12], 1);
			if((bRet1==TRUE) && (bRet2==TRUE))
			{
				bRet = TRUE;
			}
			else
			{
				bRet = FALSE;
			}

		#else

			bRet = bRet1;

		#endif
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetFontTypAttr                                       */
/* Contents         : Font Type Attribute Data setting                          */
/* Argument         : U08 FontTypId                (W) Font Type ID             */
/*                  : T_Y643_FONT_TYPATTR *tY643FontTypAttr                     */
/*                                                 (W) Font Type Attribute Data */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetFontTypAttr(U08 FontTypId, const T_Y643_FONT_TYPATTR *tY643FontTypAttr)
{
	BOOL bRet   = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/*	Input Check                         */
	/*--------------------------------------*/
	if( (FontTypId >= YVC_TBL_FONT_NUM) ||
		(tY643FontTypAttr == (const T_Y643_FONT_TYPATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*      Set Font Type Attribute Data    */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_TBL_FONT_ADDR + (U16)(FontTypId * Y643_FONTTYPATTR_SIZE));

		/*--- Data Write ---*/
		bRet = s_TblWrite(u16Addr, &tY643FontTypAttr->BYTE[0], Y643_FONTTYPATTR_SIZE);
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetPCharCode                                         */
/* Contents         : U16 CharCodeId           (W) U16 Character Code ID        */
/* Argument         : T_Y643_TEXT_PFONT *tY643TextPFont                         */
/*                                             (W) Proportional Character Codes */
/*                  : U16 CharCodeNum          (W) Number of character codes    */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetPCharCodes(U16 CharCodeId, const T_Y643_TEXT_PFONT *tY643TextPFont, U16 CharCodeNum)
{
	BOOL bRet   = TRUE;
	U16 u16Addr = 0;
	U16 u16Num  = 0;

	/*--------------------------------------*/
	/*	Input Check                         */
	/*--------------------------------------*/
	if( (CharCodeId  >= YVC_TBL_PTEXT_NUM) ||
		(tY643TextPFont == (const T_Y643_TEXT_PFONT*)NULL)||
		(CharCodeNum > YVC_TBL_PTEXT_NUM) ||
		(CharCodeId + CharCodeNum > YVC_TBL_PTEXT_NUM) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*      Set Charcter Code Data          */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_TBL_PTEXT_ADDR + (U16)(CharCodeId * Y643_PTEXT_SIZE));
		u16Num  = (U16)(CharCodeNum * Y643_PTEXT_SIZE);
		bRet = s_TblWrite(u16Addr, &tY643TextPFont->BYTE[0], u16Num);
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_ConvFCode                                            */
/* Contents         : Convert FCODE                                             */
/* Argument         : U16 Utf16                      (W) UTF-16                 */
/*                  : T_YGFONT_DATA *tYgfontData     (W) Font Data              */
/*                  : U16 *Fcode                     (R) FCODE                  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_ConvFCode(U16 Utf16, const T_YGFONT_DATA *tYgfontData, U16 *Fcode)
{
	BOOL bRet           = TRUE;
	const T_YGFONT_TBL *ptYgfntTbl = (const T_YGFONT_TBL*)NULL;
	U16  u16YgfntTblNum = 0;
	U16  u16TblCnt      = 0;
	BOOL bValid         = FALSE;

	ptYgfntTbl     = &tYgfontData->YgfontTbl[0];
	u16YgfntTblNum = tYgfontData->YgfontTblSize;

	/*--------------------------------------*/
	/*	Input Check                         */
	/*--------------------------------------*/
	if( (tYgfontData == (T_YGFONT_DATA*)NULL) ||
		(Fcode    == (U16*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*	Convert FCODE                       */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		for(u16TblCnt = 0; ((u16TblCnt < u16YgfntTblNum) && (bValid == FALSE)); u16TblCnt++)
		{
			if( (Utf16 >= ptYgfntTbl[u16TblCnt].StartCode1) &&
				(Utf16 <= ptYgfntTbl[u16TblCnt].EndCode1  ) )
			{
				bValid = TRUE;
				*Fcode = ptYgfntTbl[u16TblCnt].SeqOffset +
						 (Utf16 - ptYgfntTbl[u16TblCnt].StartCode1);
			}
		}

		if(bValid == FALSE)
		{
			bRet = FALSE;
		}
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetClipAttr                                          */
/* Contents         : Clipping Attribute Data setting                           */
/* Argument         : U16 LyrId                     (W) Layer ID                */
/*                  : T_Y643_LYR_CLIPATTR *tY643LyrClipAttr                     */
/*                                                  (W) Clipping Attribute Data */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetClipAttr(U16 LyrId, const T_Y643_LYR_CLIPATTR *tY643LyrClipAttr)
{
	BOOL bRet   = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/* Input Check                          */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrClipAttr == (const T_Y643_LYR_CLIPATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Set Sprite Attribute Data            */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet = s_TblWrite(u16Addr, (const U08*)&tY643LyrClipAttr->BYTE[0], Y643_LYR_SIZE);
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_GetSprtAttr                                          */
/* Contents         : Sprite Attribute Data acquisition                         */
/* Argument         : U16 LyrId                     (W) Layer ID                */
/*                  : T_Y643_LYR_CLIPATTR *tY643LyrClipAttr                     */
/*                                                  (R) Clipping Attribute Data */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_GetClipAttr(U16 LyrId, T_Y643_LYR_CLIPATTR *tY643LyrClipAttr)
{
	BOOL bRet   = TRUE;
	U16 u16Addr = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
		(tY643LyrClipAttr == (const T_Y643_LYR_CLIPATTR*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/* Get Sprite Attribute Data            */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet = s_TblRead(u16Addr, &tY643LyrClipAttr->BYTE[0], Y643_LYR_SIZE);
	}

	return bRet;
}

/*------------------------------------------------------------------------------*/
/* Function name    : s_TblWrite                                 (Sub Function) */
/* Contents         : Write  General table                                      */
/*                    General tables 1 and 2 are judged by LYBSELC              */
/* Argument         : U16 Addr                     (W) General table address    */
/*                  : U08 *Buff                    (W) Write buffer pointer     */
/*                  : U16 Num                      (W) Number of bytes to write */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_TblWrite(U16 Addr, const U08 *Buff, U16 Num)
{
	U16 u16Addr = 0;
	U08 u08Data = 0;
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;

	/*--------------------------------------*/
	/* Get LYBSELC Data                     */
	/*--------------------------------------*/
	bRet1   = YVC1_ReadReg(REG_R3EH, &u08Data);
	u16Addr = s_LYBSELC_Chk(Addr, u08Data);

	/*--------------------------------------*/
	/* Data Set                             */
	/*--------------------------------------*/
	bRet2 = YVC1_WriteTbls(u16Addr, Buff, Num);

	if((bRet1==TRUE) && (bRet2==TRUE))
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
/* Function name    : s_TblRead                                  (Sub Function) */
/* Contents         : Read General table                                        */
/*                    General tables 1 and 2 are judged by LYBSELC              */
/* Argument         : U16 Addr                     (W) General table address    */
/*                  : U08 *Buff                    (R) Read buffer pointer      */
/*                  : U16 Num                      (W) Number of bytes to Read  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_TblRead(U16 Addr, U08 *Buff, U16 Num)
{
	U16 u16Addr = 0;
	U08 u08Data = 0;
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;

	/*--------------------------------------*/
	/* Get LYBSELC Data                     */
	/*--------------------------------------*/
	bRet1 = YVC1_ReadReg(REG_R3EH, &u08Data);
	u16Addr = s_LYBSELC_Chk(Addr, u08Data);

	/*--------------------------------------*/
	/* Data Set                             */
	/*--------------------------------------*/
	bRet2 = YVC1_ReadTbls(u16Addr, Buff, Num);

	if((bRet1==TRUE) && (bRet2==TRUE))
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
/* Function name    : s_LYBSELC_Chk                              (Sub Function) */
/* Contents         : Calculation of a write address from LYBSELC value         */
/* Argument         : U16 Addr                     (W) Write address            */
/*                  : U08 Reg3EHData               (W) 3Eh Register data        */
/* Return value     : U16                                                       */
/*------------------------------------------------------------------------------*/
static U16 s_LYBSELC_Chk(U16 Addr, U08 Reg3EHData)
{
	U16 u16Result = 0;

	if ((Reg3EHData & YVC1_LYBSELC_MASK) != YVC1_LYBSELC_TBL1)
	{
		/*--- Set General Table1 ---*/
		u16Result = Addr;
	}
	else
	{	/*--- Set General Table2 ---*/
		u16Result = Addr | Y643_DOUBLE_BUFF_ADDR;
	}

	return u16Result;
}

#if YVC1_EXLY
/*------------------------------------------------------------------------------*/
/* Function name    : s_Exly_TblWrite                            (Sub Function) */
/* Contents         : Write Extended General table                              */
/*                    General tables 1 and 2 are judged by LYBSELC              */
/* Argument         : U16 Addr                     (W) General table address    */
/*                  : U08 *Buff                    (W) Write buffer pointer     */
/*                  : U16 Num                      (W) Number of bytes to write */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_Exly_TblWrite(U16 Addr, const U08 *Buff, U16 Num)
{
	U16 u16Addr = 0;
	U08 u08Data = 0;
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;

	/*--------------------------------------*/
	/* Get LYBSELC Data                     */
	/*--------------------------------------*/
	bRet1   = YVC1_ReadReg(REG_R3EH, &u08Data);
	u16Addr = s_EXLY_LYBSELC_Chk(Addr, u08Data);

	/*--------------------------------------*/
	/* Data Set                             */
	/*--------------------------------------*/
	bRet2 = YVC1_WriteTbls(u16Addr, Buff, Num);

	if((bRet1==TRUE) && (bRet2==TRUE))
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
/* Function name    : s_Exly_TblRead                             (Sub Function) */
/* Contents         : Read General table                                        */
/*                    General tables 1 and 2 are judged by LYBSELC              */
/* Argument         : U16 Addr                     (W) General table address    */
/*                  : U08 *Buff                    (R) Read buffer pointer      */
/*                  : U16 Num                      (W) Number of bytes to Read  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
static BOOL s_Exly_TblRead(U16 Addr, U08 *Buff, U16 Num)
{
	U16 u16Addr = 0;
	U08 u08Data = 0;
	BOOL bRet   = TRUE;
	BOOL bRet1  = TRUE;
	BOOL bRet2  = TRUE;

	/*--------------------------------------*/
	/* Get LYBSELC Data                     */
	/*--------------------------------------*/
	bRet1   = YVC1_ReadReg(REG_R3EH, &u08Data);
	u16Addr = s_EXLY_LYBSELC_Chk(Addr, u08Data);

	/*--------------------------------------*/
	/* Data Set                             */
	/*--------------------------------------*/
	bRet2 = YVC1_ReadTbls(u16Addr, Buff, Num);

	if((bRet1==TRUE) && (bRet2==TRUE))
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
/* Function name    : s_EXLY_LYBSELC_Chk                         (Sub Function) */
/* Contents         : Calculation of a write address from LYBSELC value         */
/* Argument         : U16 Addr                     (W) Write address            */
/*                  : U08 Reg3EHData               (W) 3Eh Register data        */
/* Return value     : U16                                                       */
/*------------------------------------------------------------------------------*/
static U16 s_EXLY_LYBSELC_Chk(U16 Addr, U08 Reg3EHData)
{
	U16 u16Result = (U16)(Addr / 8);

	if ((Reg3EHData & YVC1_LYBSELC_MASK) != YVC1_LYBSELC_TBL1)
	{
		/*--- Set General Table1 ---*/
		u16Result += YVC1_EXGTBL_ADDR;
	}
	else
	{	/*--- Set General Table2 ---*/
		u16Result += YVC1_EXGTBL_ADDR + Y643_DOUBLE_BUFF_ADDR;
	}

	return u16Result;
}
#endif


//Add SKLEE	171030


/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_GetMacroStatus                         				*/
/* Contents         :  Read Currnet macro playing status							*/
/* Argument         : void 													*/
/*                  											             */
/* Return value     : UINT8 Status 1byte                                                            */
/*------------------------------------------------------------------------------*/
U08 YVC1_GetMacroStatus(void)
{
	U08 Reg60HData = 0;
	
	YVC1_ReadReg(REG_R60H,&Reg60HData);
	
	if( (Reg60HData&0x60) != 0x00)
	{
		return 0x80;
	}	
	return YVC_READ_PORT(YVC1_PORT_FLG_STATUS2);
}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_PlayMacro                         				*/
/* Contents         :  Play start macro							*/
/* Argument         : UINT32 u32SceneAddr :  Scene address 													*/
/*                  											             */
/* Return value     : UINT8 Status 1byte                                                            */
/*------------------------------------------------------------------------------*/
BOOL YVC1_PlayMacro(U32 u32SceneAddr)
{
	BOOL bRet = TRUE;
	U08 u08SSA[4];
	
	u08SSA[0]    = (U08)((u32SceneAddr >> SHIFT_24BIT) & MASK_03H);
	u08SSA[1]    = (U08)((u32SceneAddr >> SHIFT_16BIT) & MASK_FFH);
	u08SSA[2]    = (U08)((u32SceneAddr >> SHIFT_08BIT) & MASK_FFH);
	u08SSA[3]    = (U08)((u32SceneAddr               ) & MASK_FCH);
	bRet = YVC1_WriteRegs(REG_R5CH, &u08SSA[0], 4);
	if (bRet == TRUE)
	{
			/*--- Macro Command ---*/
		YVC_WRITE_PORT(YVC1_PORT_FLG_STATUS2, 0x80);
		bRet = YVC1_WriteReg(REG_R60H, 0x80);
	}

	return bRet;
}
/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_StopMacro                         				*/
/* Contents         :  Play stop macro							*/
/* Argument         :void												*/
/*                  											             */
/* Return value     :void                                                            */
/*------------------------------------------------------------------------------*/
void YVC1_StopMacro(void)
{
	YVC1_WriteReg(REG_R60H, 0x20);
	YVC1_WriteReg(REG_R60H, 0x08);
}


/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_McrSetAllLyrDisp                         */
/* Contents         :  Display all  On/Off Macro controller layer				    */
/* Argument         : BOOL Disp   : Ture is On / False is Off                  */
/*                  											             */
/* Return value     : void                                                       */
/*------------------------------------------------------------------------------*/
void YVC1_McrSetAllLyrDisp(BOOL Disp)
{
	U08 tY643R42h;

	/*----------------------------------------
	 *	R#42h register set
	 *----------------------------------------*/
	YVC1_ReadReg(REG_R42H, &tY643R42h);

	if( Disp == TRUE )
	{
		tY643R42h = tY643R42h | 0x80;
	}
	else
	{
		tY643R42h = tY643R42h & 0x7F;
	}

	YVC1_WriteReg(REG_R42H, tY643R42h);

}

/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_CpuSetAllLyrDisp                         				*/
/* Contents         :  Display all On/Off Host controller layer				    */
/* Argument         : BOOL Disp   : Ture is On / False is Off                  */
/*                  											             */
/* Return value     : void                                                       */
/*------------------------------------------------------------------------------*/
void YVC1_CpuSetAllLyrDisp(BOOL Disp)
{
	T_Y643_R3EH tY643R3Eh = {0};

	/*----------------------------------------
	 *	R#42h register set
	 *----------------------------------------*/
	YVC1_ReadReg(REG_R3EH, &tY643R3Eh.BYTE);

	if( Disp == TRUE )
	{
		tY643R3Eh.BIT.LYDC = 1;
	}
	else
	{
		tY643R3Eh.BIT.LYDC = 0;
	}

	YVC1_WriteReg(REG_R3EH, tY643R3Eh.BYTE);

}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetSprtAttrWithParam                                 */
/* Contents         : Sprite Attribute Data setting                             */
/* Argument         : U16 LyrId                             (W) Layer ID        */
/*                  : T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr                     */
/*                                                   (W) Sprite Attribute Data  */
/*                                             : S16 s16x                       */
/*                                             : S16 s16y                       */
/*                                             : U16 u16Alpha                   */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_SetSprtAttrWithParam(U16 LyrId, const T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr , S16 s16x , S16 s16y, U16 u16Alpha)
{
	BOOL bRet  = TRUE;
	BOOL bRet1 = TRUE;
	BOOL bRet2 = TRUE;
	U16 u16Addr = 0;
	U08 i = 0;
	S16 Doy = 0;
	S16 Dox= 0;
	T_Y643_LYR_SPRTATTR tLyrSprtAttr = {0};
	
	/*--------------------------------------*/
	/* Input Check                          */
	/*--------------------------------------*/
	if( (LyrId >= YVC_LYR_NUM_CPU) ||
	   (tY643LyrSprtAttr == (const T_Y643_LYR_SPRTATTR*)NULL) )
	{
	             bRet = FALSE;
	}
	
	/*--------------------------------------*/
	/* Copy Data                                */
	/*--------------------------------------*/
	for( i = 0 ; i < Y643_LYR_SIZE ; i++)
	{
		tLyrSprtAttr.BYTE[i] = tY643LyrSprtAttr->BYTE[i];
	}
	
	Doy = (S16)(tLyrSprtAttr.BIT.DOY_8) << SHIFT_08BIT;
	Doy |= tLyrSprtAttr.BIT.DOY_0;
	
	Dox = (S16)(tLyrSprtAttr.BIT.DOX_8) << SHIFT_08BIT;
	Dox |= tLyrSprtAttr.BIT.DOX_0;
	
	Doy = Doy + s16y;
	Dox = Dox + s16x;
	
	/*----------------------------------------*/
	/*          Set Data                                                                                                                                                                                                      */
	/*----------------------------------------*/
	
	tLyrSprtAttr.BIT.DOY_0 = (U08)(Doy               ) & MASK_FFH;
	tLyrSprtAttr.BIT.DOY_8 = (U08)(Doy >> SHIFT_08BIT) & MASK_07H;
	
	tLyrSprtAttr.BIT.DOX_0 = (U08)(Dox               ) & MASK_FFH;
	tLyrSprtAttr.BIT.DOX_8 = (U08)(Dox >> SHIFT_08BIT) & MASK_07H;
	
	tLyrSprtAttr.BIT.MX8  = (U08)(u16Alpha >> SHIFT_08BIT) & MASK_01H;
	tLyrSprtAttr.BIT.MX_4 = (U08)(u16Alpha >> SHIFT_04BIT) & MASK_0FH;
	
	#if YVC1_EXLY                        
		tLyrSprtAttr.BIT.EXLY.MX_0 = (U08)(u16Alpha)&MASK_0FH;
	#endif   
	
	/*--------------------------------------*/
	/* Set Sprite Attribute Data            */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Addr = (U16)(YVC_LYR_ADDR_CPU + (U16)(LyrId * Y643_LYR_SIZE));
		bRet1 = s_TblWrite(u16Addr, (const U08*)&tLyrSprtAttr.BYTE[0], Y643_LYR_SIZE);
		
		#if YVC1_EXLY           
		/*--- Set Extended Layer ---*/
			bRet2 = s_Exly_TblWrite(u16Addr, (const U08*)&tLyrSprtAttr.BYTE[12], 1);
			if((bRet1==TRUE) && (bRet2==TRUE))
			{
			             bRet = TRUE;
			}
			else
			{
			             bRet = FALSE;
			}                         

		#else
		
			bRet = bRet1;

		#endif
	
	}
	
	return bRet;
}


/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetHostContLyrWithFlip                         				*/
/* Contents         : Host controller layer configuration								*/
/* Argument         : UINT16 Addr	:  start address                 */
/*                  											             */
/* Return value     : TRUE or FALSE                                                            */
/*------------------------------------------------------------------------------*/
void YVC1_SetHostContLyrWithFlip(U16 u16LyrStartAddr,U16 NumofLayers)
{
	T_Y643_R3EH tY643R3EH = {0};
	T_Y643_R3FH tY643R3FH = {0};
	T_Y643_R40H tY643R40H = {0};
	T_Y643_R41H tY643R41H = {0};
	T_Y643_PORT_FLAG1 tY643PortFlag1 = {0};

	YVC1_ReadReg(REG_R3EH, &tY643R3EH.BYTE);
	YVC1_ReadReg(REG_R3FH, &tY643R3FH.BYTE);
	YVC1_ReadReg(REG_R40H, &tY643R40H.BYTE);
	YVC1_ReadReg(REG_R41H, &tY643R41H.BYTE);


	if( tY643R3EH.BIT.LYBSELC != YVC1_LYBSELC_TBL1)
	{
		/*--- Flip General Table 1 ---*/
		tY643R3EH.BIT.LYBSELC = 0;
	}
	else
	{
		/*--- Flip General Table 2 ---*/
		tY643R3EH.BIT.LYBSELC = 1;
	}
	
	tY643R3EH.BIT.LYSAC_8 = ((u16LyrStartAddr >> SHIFT_08BIT) & MASK_0FH);
	tY643R3FH.BIT.LYSAC_2 = ((u16LyrStartAddr >> SHIFT_02BIT) & MASK_3FH);
	
	tY643R40H.BIT.LYEAC_8 = (((u16LyrStartAddr+(NumofLayers-1)*Y643_LYR_SIZE) >> SHIFT_08BIT) & MASK_0FH);
	tY643R41H.BIT.LYEAC_2 = (((u16LyrStartAddr+(NumofLayers-1)*Y643_LYR_SIZE) >> SHIFT_02BIT) & MASK_3FH);
	
	YVC1_WriteReg(REG_R3EH, tY643R3EH.BYTE);
	YVC1_WriteReg(REG_R3FH, tY643R3FH.BYTE);
	YVC1_WriteReg(REG_R40H, tY643R40H.BYTE);
	YVC1_WriteReg(REG_R41H, tY643R41H.BYTE);

	tY643PortFlag1.BYTE = YVC_READ_PORT(Y643_PORT_FLG_STATUS1);


	if( tY643PortFlag1.BIT.FD == 1)
	{
		tY643PortFlag1.BIT.FD = 1;
	}

	/*--- P#6 FB flag and FP flag reset ---*/
	tY643PortFlag1.BIT.FB = 1;
	tY643PortFlag1.BIT.FP = 1;
	YVC_WRITE_PORT(YVC1_PORT_FLG_STATUS1, tY643PortFlag1.BYTE);
}


/*------------------------------ E O F -----------------------------------------*/

