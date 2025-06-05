/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2012 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Fontctrl.c $                                    */
/*  Description     : Layer setting function                                    */
/*  Version         : $Rev: 154 $                                               */
/*  Last UpDate Time: $Date:: 2012-08-28 08:41:35#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          :                                               */
/********************************************************************************/

/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc1Ddrv.h"

#include "Yvc1Lyr.h"
#include "layoutData.h"
#include "Yvc1Fontctrl.h"

#include <string.h>
/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						P R O T O T Y P E
 *------------------------------------------------------------------------------*/


/*===========================================================================*/
/* Function name    : GetFSize                                              */
/* Contents         : Get font size                                    */
/* Argument         : U08 *pu1_FontTypeAttr Font type attr                                                      */
/*                                                                              */
/* Return value     : U08 font size                                      */															
/*===========================================================================*/
static U08 GetFSize(U08 *pu1_FontTypeAttr)
{
	U08	u1_FSZX;
	U08	u1_FSZY;
	U08	u1_FSize;
	u1_FSize = (U08)FNT_INVALID;

	if(pu1_FontTypeAttr != NULL){
		/*---	폰트 속성 데이터로 부터 FSZX를 리탛E---*/
		u1_FSZX = pu1_FontTypeAttr[(U08)FNT_FSZX_POS] & (U08)FNT_FSZX_MASK;
		u1_FSZX += 1;
		u1_FSize = u1_FSZX;
		/*---	폰트 속성 데이터로 부터 FSZY를 리탛E	---*/
		u1_FSZY = pu1_FontTypeAttr[(U08)FNT_FSZY_POS] & (U08)FNT_FSZY_MASK;
		u1_FSZY += 1;

		if(u1_FSize > u1_FSZY){
			u1_FSize = u1_FSZY;
		}
	}
	return u1_FSize;
}
/*===========================================================================*/
/* Function name    : GetFontTypeAttr                                              */
/* Contents         : Get Font Type Attr                                  */
/* Argument         : const T_YGFONTCTRL *t_fontctrl                                                     */
/*                                                                              */
/* Return value     : U08 font type attr pointer                                  */															
/*===========================================================================*/

static U08 *GetFontTypeAttr(const T_YGFONTCTRL *t_fontctrl)
{
	U08* pu1_FontTypAttr;

	pu1_FontTypAttr = NULL;
	/*---	폰트 속성 데이터를 리탛E--*/
	if(t_fontctrl != NULL){
		if(t_fontctrl->tYgfontData != NULL){
			pu1_FontTypAttr = (U08 *)&t_fontctrl->tYgfontData->FontTypAttr[0];
		}
	}
	return pu1_FontTypAttr;
}


/*===========================================================================*/
/* Function name    : GetXsXe                                              */
/* Contents         : Get Xs Xe 생성된 Layoutdata의 장?E자간 데이터를 get                                    */
/* Argument         : U16 u2_FCode : 문자열
				  const T_YGFONT_DATA_LYT *tYgfontData,		font layer attr
				  U08 *pu1_XS, U08 *pu1_XE                                                  */
/*                                                                              */
/* Return value     : U08   status                               */															
/*===========================================================================*/
static U08 GetXsXe(U16 u2_FCode, const T_YGFONT_DATA_LYT *tYgfontData, U08 *pu1_XS, U08 *pu1_XE)
{
	U08						u1_Ret;
	const T_YGFONT_PDATA	*tYgfontPData;

	u1_Ret = (U08)FNT_ERROR;

	if((tYgfontData != NULL) && (pu1_XS != NULL) && (pu1_XE != NULL)){
		/*-------------------------------------*/
		/*---	XS,XE Read					---*/
		/*-------------------------------------*/
		tYgfontPData = &tYgfontData->tYgfontPData[0];
		if(tYgfontPData != NULL){
			*pu1_XS = tYgfontPData[u2_FCode].XS;
			*pu1_XE = tYgfontPData[u2_FCode].XE;

			u1_Ret = (U08)FNT_NO_ERROR;
		}
	}
	return u1_Ret;
}

/*===========================================================================*/
/* Function name    : ConvFCode                                        */
/* Contents         : UFT16 - > FCODE convert                               */
/* Argument         : U16 u2_Utf16
				 const T_YGFONT_DATA_LYT *tYgfontData
				 U16 *u2_FCode                */
/*                                                                              */
/* Return value     : U08   status                               */															
/*===========================================================================*/

static U08 ConvFCode(U16 u2_Utf16, const T_YGFONT_DATA_LYT *tYgfontData, U16 *u2_FCode)
{
	U08	u1_Ret;
	U08	u1_Found;
	U08	u1_YgfntTblSize;
	U08	u1_TblCnt;
	const T_YGFONT_TBL_LYT   *tYgfontTbl;

	u1_Ret		= (U08)FNT_ERROR;
	u1_Found	= (U08)FNT_NOT_FOUND;

	if((tYgfontData != NULL) || (u2_FCode != NULL)){
		tYgfontTbl		= &tYgfontData->tYgfontTbl[0];
		u1_YgfntTblSize	= tYgfontData->YgfontTblSize;

		/*-------------------------------------*/
		/*---	FCODE read					---*/
		/*-------------------------------------*/
		for(u1_TblCnt=0; (u1_TblCnt<u1_YgfntTblSize)&&(u1_Found==(U08)FNT_NOT_FOUND); u1_TblCnt++)
		{
			if( (u2_Utf16 >= tYgfontTbl[u1_TblCnt].StartCode) &&
			    (u2_Utf16 <= tYgfontTbl[u1_TblCnt].EndCode  ) )
			{
				u1_Found = (U08)FNT_FOUND;
				*u2_FCode = tYgfontTbl[u1_TblCnt].SeqOffset + 
                                         (u2_Utf16 - tYgfontTbl[u1_TblCnt].StartCode);
			}
		}
		if(u1_Found == (U08)FNT_FOUND)
		{
			u1_Ret = (U08)FNT_NO_ERROR;
		}
	}
	return u1_Ret;
}



/*===========================================================================*/
/* Function name    : ConvFCodeString                                        */
/* Contents         : UFT16 - > FCODE 문자열 변환 함펯E                            */
/* Argument         : const ST_FONTCTRL *t_fontctrl	font control data			 */
/*		      U08 *pu1_Utf16		UTF16 code string pointer			 */
/*		      U08 u1_Utf16Num		UTF16 cide string number						 */
/*		      U08 *pu1_FCode		FCODE 를 저장하는 pointer				 */
/*		      U16 u2_FCodeByteNum	FCode를 저장할 영역의 바이트펯E                   */
/*                                                                              */
/* Return value     : U16 저장한 바이트의펯E                           */															
/*===========================================================================*/

U16 ConvFCodeString(const T_YGFONTCTRL *t_fontctrl, U08 *pu1_Utf16, U08 u1_Utf16Num,U08 *pu1_FCode, U16 u2_FCodeByteNum)
{
	U08	u1_error;
	U08	u1_Utf16Cnt;
	U16	u2_Utf16;
	U16	u2_FCode;
	U08	u1_XS;
	U08	u1_XE;
	U16	u2_ByteCnt;
	U16	u2_TextWidth;
	U08	u1_FCodeStr[FNT_DISPLAYTEXT_MAX * FNT_PFONT_SIZE];
	U16	u2_DispWidth;
	U16	u2_SpaceDot;
	U16	u2_FCode_Space;
	U08	*pu1_FontTypeAttr;
	U08	u1_FontXSize;
	U16	u2_SetFCodeByte;
	U16	u2_SetDataByte;


	u1_error		= (U08)FNT_ERROR;
	if(t_fontctrl != NULL){
		if((pu1_Utf16 != NULL) && (u1_Utf16Num <= (U08)FNT_DISPLAYTEXT_MAX)){
			if((pu1_FCode != NULL) && ((u2_FCodeByteNum % 4) == 0)){
				u1_error = (U08)FNT_NO_ERROR;
			}
		}
	}

	/*-----------------------------------------*/
	/*---	FCode Convert						---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u2_ByteCnt		= 0;
		u2_TextWidth	= 0;
		for(u1_Utf16Cnt=0; (u1_Utf16Cnt<u1_Utf16Num)&&(u1_error==(U08)FNT_NO_ERROR); u1_Utf16Cnt++){
			/*---	FCode Convert					---*/
			u2_Utf16 = ((U16)pu1_Utf16[u1_Utf16Cnt*2]<<(U08)FNT_SHIFT_8BIT) | pu1_Utf16[u1_Utf16Cnt*2+1];
			u1_error = ConvFCode(u2_Utf16, t_fontctrl->tYgfontData, &u2_FCode);
			if(u1_error == (U08)FNT_NO_ERROR){
				/*---	지정문자열의 폭을 calculate	---*/
				u1_error = GetXsXe(u2_FCode, t_fontctrl->tYgfontData, &u1_XS, &u1_XE);
				if(u1_error == (U08)FNT_NO_ERROR){
					u1_FCodeStr[u2_ByteCnt++] = (U08)(u2_FCode >> (U08)FNT_SHIFT_8BIT);
					u1_FCodeStr[u2_ByteCnt++] = (U08)u2_FCode;
					u1_FCodeStr[u2_ByteCnt++] = u1_XS;
					u1_FCodeStr[u2_ByteCnt++] = u1_XE;
					u2_TextWidth += (u1_XE - u1_XS) + 1;
				}
			}
		}
	}

	/*-----------------------------------------*/
	/*---	문자열 표시 폭을 get			---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u2_DispWidth = t_fontctrl->Width;
	}

	/*-----------------------------------------*/
	/*--- 들엉黔 공퉩E도트수를 Calculate--*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		switch(t_fontctrl->AlignX)
		{
		case TEXTALIGNX_CENTER:
			if(u2_DispWidth > u2_TextWidth){
				u2_SpaceDot = (u2_DispWidth - u2_TextWidth) / 2;
			}
			break;
		case TEXTALIGNX_RIGHT:
			if(u2_DispWidth > u2_TextWidth){
				u2_SpaceDot = u2_DispWidth - u2_TextWidth;
			}
			break;
		case TEXTALIGNX_LEFT:
		default:
			u2_SpaceDot = 0;
			break;
		}
	}

	/*-----------------------------------------*/
	/*---	스페이스 Fcode get		---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u1_error = ConvFCode((U16)FNT_SPACE_CODE, t_fontctrl->tYgfontData, &u2_FCode_Space);
	}

	/*-----------------------------------------*/
	/*---	font size get			---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		pu1_FontTypeAttr = GetFontTypeAttr(t_fontctrl);
		if(pu1_FontTypeAttr != NULL){
			u1_FontXSize = GetFSize(pu1_FontTypeAttr);
			if(u1_FontXSize == (U08)FNT_INVALID){
				u1_error = (U08)FNT_ERROR;
			}
		}
	}

	/*-----------------------------------------*/
	/*---	Fcode setting						---*/
	/*-----------------------------------------*/
	u2_SetFCodeByte	= 0;
	u2_SetDataByte	= 0;
	if(u1_error == (U08)FNT_NO_ERROR){

		/*---	Align지정에 의해 문자열의 좌횁E공백을 삽입	---*/
		while((u2_SpaceDot > 0) && (u2_SetDataByte < u2_FCodeByteNum)){
			pu1_FCode[u2_SetDataByte + 0] = (U08)(u2_FCode_Space >> (U08)FNT_SHIFT_8BIT);
			pu1_FCode[u2_SetDataByte + 1] = (U08)u2_FCode_Space;
			pu1_FCode[u2_SetDataByte + 2] = 0;
			if(u2_SpaceDot > u1_FontXSize){
				pu1_FCode[u2_SetDataByte + 3] = u1_FontXSize - 1;
				u2_SpaceDot -= u1_FontXSize;
			}
			else{
				pu1_FCode[u2_SetDataByte + 3] = (U08)(u2_SpaceDot - 1);
				u2_SpaceDot -= u2_SpaceDot;
			}
			u2_SetDataByte += 4;
		}

		/*---	FCODE 삽입										---*/
		for(u1_Utf16Cnt=0; u1_Utf16Cnt < u1_Utf16Num; u1_Utf16Cnt++){
			if(u2_SetDataByte < u2_FCodeByteNum){
				pu1_FCode[u2_SetDataByte + 0] = u1_FCodeStr[u2_SetFCodeByte + 0];
				pu1_FCode[u2_SetDataByte + 1] = u1_FCodeStr[u2_SetFCodeByte + 1];
				pu1_FCode[u2_SetDataByte + 2] = u1_FCodeStr[u2_SetFCodeByte + 2];
				pu1_FCode[u2_SetDataByte + 3] = u1_FCodeStr[u2_SetFCodeByte + 3];
				u2_SetDataByte += 4;
				u2_SetFCodeByte+= 4;
			}
		}

		/*---	Align지정에 의해 문자열의 퓖E갋공백을 삽입				---*/
		while(u2_SetDataByte < u2_FCodeByteNum){
			pu1_FCode[u2_SetDataByte + 0] = (U08)(u2_FCode_Space >> (U08)FNT_SHIFT_8BIT);
			pu1_FCode[u2_SetDataByte + 1] = (U08)u2_FCode_Space;
			pu1_FCode[u2_SetDataByte + 2] = 0;
			pu1_FCode[u2_SetDataByte + 3] = u1_FontXSize - 1;
			u2_SetDataByte += 4;
		}
	}
	return u2_SetDataByte;
}
/* ----------------------------- E O F -----------------------------------------*/




/*===========================================================================*/
/* Function name    : ConvFCodeString                                        */
/* Contents         : UFT16 - > FCODE 문자열 변환 함펯E                            */
/* Argument         : const ST_FONTCTRL *t_fontctrl	font control data			 */
/*				 U08 *pu1_Utf16		UTF16 code string pointer			 */
/*				 U08 u1_Utf16Num		UTF16 cide string number						 */
/*				 U08 *pu1_FCode		FCODE 를 저장하는 pointer				 */
/*				 U16 u2_FCodeByteNum	FCode를 저장할 영역의 바이트펯E            */
/*                                                                              */
/* Return value     : U16 저장한 바이트의펯E                           */															
/*===========================================================================*/

U16 ConvFCodeString_utf8(const T_YGFONTCTRL *t_fontctrl, C08 *pu1_Utf8, U16 u1_Utf16Num,U08 *pu1_FCode, U16 u2_FCodeByteNum)
{
	U08	u1_error;
	U08	u1_Utf16Cnt;
	U16	u2_Utf16;
	U16	u2_FCode;
	U08	u1_XS;
	U08	u1_XE;
	U16	u2_ByteCnt;
	U16	u2_TextWidth;
	U08	u1_FCodeStr[FNT_DISPLAYTEXT_MAX * FNT_PFONT_SIZE];
	U16	u2_DispWidth;
	U16	u2_SpaceDot;
	U16	u2_FCode_Space;
	U08	*pu1_FontTypeAttr;
	U08	u1_FontXSize;
	U16	u2_SetFCodeByte;
	U16	u2_SetDataByte;


	u1_error		= (U08)FNT_ERROR;
	if(t_fontctrl != NULL){
		if((pu1_Utf8 != NULL) && (u1_Utf16Num <= (U08)FNT_DISPLAYTEXT_MAX)){
			if((pu1_FCode != NULL) && ((u2_FCodeByteNum % 4) == 0)){
				u1_error = (U08)FNT_NO_ERROR;
			}
		}
	}

	/*-----------------------------------------*/
	/*---	FCode Convert						---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u2_ByteCnt		= 0;
		u2_TextWidth	= 0;
		for(u1_Utf16Cnt=0; (u1_Utf16Cnt<u1_Utf16Num)&&(u1_error==(U08)FNT_NO_ERROR); u1_Utf16Cnt++){
			/*---	FCode Convert					---*/
			u2_Utf16 = 0x00FF&(U16)(pu1_Utf8[u1_Utf16Cnt]);
			u1_error = ConvFCode(u2_Utf16, t_fontctrl->tYgfontData, &u2_FCode);
			if(u1_error == (U08)FNT_NO_ERROR){
				/*---	지정문자열의 폭을 calculate	---*/
				u1_error = GetXsXe(u2_FCode, t_fontctrl->tYgfontData, &u1_XS, &u1_XE);
				if(u1_error == (U08)FNT_NO_ERROR){
					u1_FCodeStr[u2_ByteCnt++] = (U08)(u2_FCode >> (U08)FNT_SHIFT_8BIT);
					u1_FCodeStr[u2_ByteCnt++] = (U08)u2_FCode;
					u1_FCodeStr[u2_ByteCnt++] = u1_XS;
					u1_FCodeStr[u2_ByteCnt++] = u1_XE;
					u2_TextWidth += (u1_XE - u1_XS) + 1;
				}
			}
		}
	}

	/*-----------------------------------------*/
	/*---	문자열 표시 폭을 get			---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u2_DispWidth = t_fontctrl->Width;
	}

	/*-----------------------------------------*/
	/*--- 들엉黔 공퉩E도트수를 Calculate--*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		switch(t_fontctrl->AlignX)
		{
		case TEXTALIGNX_CENTER:
			if(u2_DispWidth > u2_TextWidth){
				u2_SpaceDot = (u2_DispWidth - u2_TextWidth) / 2;
			}
			break;
		case TEXTALIGNX_RIGHT:
			if(u2_DispWidth > u2_TextWidth){
				u2_SpaceDot = u2_DispWidth - u2_TextWidth;
			}
			break;
		case TEXTALIGNX_LEFT:
		default:
			u2_SpaceDot = 0;
			break;
		}
	}

	/*-----------------------------------------*/
	/*---	스페이스 Fcode get		---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		u1_error = ConvFCode((U16)FNT_SPACE_CODE, t_fontctrl->tYgfontData, &u2_FCode_Space);
	}

	/*-----------------------------------------*/
	/*---	font size get			---*/
	/*-----------------------------------------*/
	if(u1_error == (U08)FNT_NO_ERROR){
		pu1_FontTypeAttr = GetFontTypeAttr(t_fontctrl);
		if(pu1_FontTypeAttr != NULL){
			u1_FontXSize = GetFSize(pu1_FontTypeAttr);
			if(u1_FontXSize == (U08)FNT_INVALID){
				u1_error = (U08)FNT_ERROR;
			}
		}
	}

	/*-----------------------------------------*/
	/*---	Fcode setting						---*/
	/*-----------------------------------------*/
	u2_SetFCodeByte	= 0;
	u2_SetDataByte	= 0;
	if(u1_error == (U08)FNT_NO_ERROR){

		/*---	Align지정에 의해 문자열의 좌횁E공백을 삽입	---*/
		while((u2_SpaceDot > 0) && (u2_SetDataByte < u2_FCodeByteNum)){
			pu1_FCode[u2_SetDataByte + 0] = (U08)(u2_FCode_Space >> (U08)FNT_SHIFT_8BIT);
			pu1_FCode[u2_SetDataByte + 1] = (U08)u2_FCode_Space;
			pu1_FCode[u2_SetDataByte + 2] = 0;
			if(u2_SpaceDot > u1_FontXSize){
				pu1_FCode[u2_SetDataByte + 3] = u1_FontXSize - 1;
				u2_SpaceDot -= u1_FontXSize;
			}
			else{
				pu1_FCode[u2_SetDataByte + 3] = (U08)(u2_SpaceDot - 1);
				u2_SpaceDot -= u2_SpaceDot;
			}
			u2_SetDataByte += 4;
		}

		/*---	FCODE 삽입										---*/
		for(u1_Utf16Cnt=0; u1_Utf16Cnt < u1_Utf16Num; u1_Utf16Cnt++){
			if(u2_SetDataByte < u2_FCodeByteNum){
				pu1_FCode[u2_SetDataByte + 0] = u1_FCodeStr[u2_SetFCodeByte + 0];
				pu1_FCode[u2_SetDataByte + 1] = u1_FCodeStr[u2_SetFCodeByte + 1];
				pu1_FCode[u2_SetDataByte + 2] = u1_FCodeStr[u2_SetFCodeByte + 2];
				pu1_FCode[u2_SetDataByte + 3] = u1_FCodeStr[u2_SetFCodeByte + 3];
				u2_SetDataByte += 4;
				u2_SetFCodeByte+= 4;
			}
		}

		/*---	Align지정에 의해 문자열의 퓖E갋공백을 삽입				---*/
		while(u2_SetDataByte < u2_FCodeByteNum){
			pu1_FCode[u2_SetDataByte + 0] = (U08)(u2_FCode_Space >> (U08)FNT_SHIFT_8BIT);
			pu1_FCode[u2_SetDataByte + 1] = (U08)u2_FCode_Space;
			pu1_FCode[u2_SetDataByte + 2] = 0;
			pu1_FCode[u2_SetDataByte + 3] = u1_FontXSize - 1;
			u2_SetDataByte += 4;
		}
	}
	return u2_TextWidth;//u2_SetDataByte;

}
/*   
const T_YGFONTCTRL Data1_value_FC =
{
	&MESSAGE_HANDLER_GT[0][0],	&T3_IDS_DATA1[0][0],	&tFontData[1],	83,	YTEXTALIGNX_RIGHT,	1,	5,	0,	NULL
};
	const U08				*FontAttr;
	const U08				*FCode;	
	const T_YGFONT_DATA_LYT	*tYgfontData;	
	U16						Width;
	U08						AlignX;
	U08						FontAttrNum;
	U16						MaxCharNum;
	U16						LayerIndex;
	const struct t_fontctrl	*tRelation;		
        Data1_value_FC
          #define YVC_TBL_PTEXT_ADDR             (0x0F7C)
          #define YVC1_PTEXT_SIZE             Y643_PTEXT_SIZE
          #define Y643_PTEXT_SIZE         (4)            
     
        {0xC0,0xBB,0x02,0x62,0x73,0xFE,0x00,0x77,0xE6,0xF6,0x00,0x00},		<GRP0><TXT9>Text 
        {0xC1,0x47,0x01,0x3B,0x73,0xFC,0x00,0x77,0xDC,0xE4,0x00,0x00},		<GRP1><TXT5>value
        {0xC0,0x96,0x02,0x56,0x73,0xFC,0x00,0x77,0xD2,0xDA,0x00,0x00},		<GRP2><TXT5>Data1_value 
	{0xC0,0xC8,0x02,0x56,0x73,0xFC,0x00,0x77,0xC8,0xD0,0x00,0x00},		<GRP3><TXT5>Data2_value 
	{0xC0,0xFA,0x02,0x56,0x73,0xFC,0x00,0x77,0xBE,0xC6,0x00,0x00},		<GRP4><TXT5>Data3_value 
	{0xC1,0x2C,0x02,0x56,0x73,0xFC,0x00,0x77,0xB4,0xBC,0x00,0x00},		<GRP5><TXT5>Data4_value 
        
        0x0F7C - 0x14 = F68
          7E6<<1 = FCC - F7C(YVC_TBL_PTEXT_ADDR) = 0x50 (80) / 4 (Y643_PTEXT_SIZE) = 20                     // Text
          7DC<<1 = FB8 - F7C = 0x3C (60) / 4 = 15                      // value
          7D2<<1 = FA4 - F7C = 0x28 (40) / 4 = 10                       // Data1
          7C8<<1 = F90 - F7C = 0x14 (20) / 4 = 5                     // Data2
          7BE<<1 = F7C - F7C = 0x0  (0)  / 4 = 0                  // Data3
          7B4<<1 = F68 - F7C = -0x20                      // Data4 - problem
            */
/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_SetChar                                 							*/
/* Contents         : Sprite Attribute Data setting                             */
/* Argument         : T_YGFONTCTRL tYgfontCtrl                  								*/
/*                  : C08 *text 							                   							  */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
U16 YVC1_SetChar(T_YGFONTCTRL tYgfontCtrl , C08 *text )
{
	U08 t_ConvFcode[4*40];
	U16 u16RetPos = 0;
	U16 u16Addr = 0;
	U16 u16id = 0;
	
	if( tYgfontCtrl.MaxCharNum*4 > 4*40 )
	{
		// t_ConvFcode buffer size error
		return 0;
	}
	
	u16Addr = (((U16)(tYgfontCtrl.FontAttr[7] & 0xf0) << 4 ) & 0xff00) |  tYgfontCtrl.FontAttr[8];
	u16Addr = u16Addr << 1;
	u16id = ((u16Addr - YVC_TBL_PTEXT_ADDR )/ YVC1_PTEXT_SIZE)  ;
	
	u16RetPos = ConvFCodeString_utf8(&tYgfontCtrl, text, strlen( text ),t_ConvFcode ,tYgfontCtrl.MaxCharNum*4);
	YVC1_SetPCharCodes(u16id , (const T_Y643_TEXT_PFONT *)(t_ConvFcode), tYgfontCtrl.MaxCharNum);

	return u16RetPos;
}
/* ----------------------------- E O F -----------------------------------------*/



