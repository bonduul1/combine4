/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2015 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Lyr.h $                                    */
/*  Description     : Layer setting function include header                     */
/*  Version         : $Rev: 39 $                                               */
/*  Last UpDate Time: $Date:: 2015-01-08 15:38:06#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Nakahara                                                */
/********************************************************************************/

#ifndef _def_YVC1_LYR_H_
#define _def_YVC1_LYR_H_

/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/

/*----------------------------------*/
/* YGF Search table data            */
/*----------------------------------*/
typedef struct {
	U16	StartCode1;						/* Top character code 1 in the block				*/
	U16	EndCode1;						/* End character code 1 in the block				*/
	U16	StartCode2;						/* Top character code 2 in the block				*/
	U16	EndCode2;						/* End character code 2 in the block				*/
	U16	SeqOffset;						/* Search character code offset						*/
	U08	PropData;						/* Proportional data existence  0:not 1:existence	*/
	U08	HalfSizeFont;					/* Half-size control  0 : Normal 1 : Half Size Font	*/
} T_YGFONT_TBL;

/*----------------------------------*/
/* YGF Proportional data            */
/*----------------------------------*/
typedef struct {
	U08	XS;										/* Proportional data on the font left side	*/
	U08	XE;										/* Proportional data on the font right side	*/
} T_YGFONT_PDATA;

/*----------------------------------*/
/* Font data structure              */
/*----------------------------------*/
typedef struct {
	const T_Y643_FONT_TYPATTR	*FontTypAttr;	/* Font type attribute data					*/
	U08							YgfontTblSize;	/* Number of Search table data		        */
	const T_YGFONT_TBL			*YgfontTbl;	    /* YGF Search table data					*/
	const T_YGFONT_PDATA		*YgfontPData;   /* YGF Proportional data					*/
} T_YGFONT_DATA;


/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
/*--- Sprite Layer ---*/
BOOL YVC1_SetSprtAttr(U16 LyrId, const T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr);
BOOL YVC1_GetSprtAttr(U16 LyrId, T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr);

/*--- Text Layer ---*/
BOOL YVC1_SetFontAttr(U16 LyrId, const T_Y643_LYR_FONTATTR *tY643LyrFontAttr);
BOOL YVC1_GetFontAttr(U16 LyrId, T_Y643_LYR_FONTATTR *tY643LyrFontAttr);
BOOL YVC1_SetFontTypAttr(U08 FontTypId, const T_Y643_FONT_TYPATTR *tY643FontTypAttr);
BOOL YVC1_SetPCharCodes(U16 CharCodeId, const T_Y643_TEXT_PFONT *tY643TextPFont, U16 CharCodeNum);
BOOL YVC1_ConvFCode(U16 Utf16, const T_YGFONT_DATA *tYgfontData, U16 *Fcode);

/*--- Clipping Layer ---*/
U08 YVC1_GetMacroStatus(void);
BOOL YVC1_PlayMacro(U32 u32SceneAddr);
void YVC1_StopMacro(void);
void YVC1_McrSetAllLyrDisp(BOOL Disp);
void YVC1_CpuSetAllLyrDisp(BOOL Disp);
BOOL YVC1_SetSprtAttrWithParam(U16 LyrId, const T_Y643_LYR_SPRTATTR *tY643LyrSprtAttr , S16 s16x , S16 s16y, U16 u16Alpha);
void YVC1_SetHostContLyrWithFlip(U16 u16LyrStartAddr,U16 NumofLayers);

#endif


/* ----------------------------- E O F -----------------------------------------*/

