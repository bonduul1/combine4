/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2012 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Fontctrl.h $                                    */
/*  Description     : Layer setting function include header                     */
/*  Version         : $Rev: 146 $                                               */
/*  Last UpDate Time: $Date:: 2012-08-24 14:06:43#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          :                                              */
/********************************************************************************/

#ifndef _def_YVC1_FONTCTRL_H_
#define _def_YVC1_FONTCTRL_H_

/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
#define	FNT_TYPE_SET_DAT		(0)
#define	FNT_TYPE_SET_DAT_EX		(1)
#define	FNT_TYPE_SET_DAT_TOP	(2)

#define ITEM_SPRITE		(1)				
#define ITEM_LINE		(2)				
#define ITEM_STR		(3)				
#define ITEM_CHAR		(4)				
#define ITEM_FONT		(5)				
#define ITEM_NONE		(0)			

#define	LAYER_SIZE		(12)

#define	FNT_ITEM_SPRITE			(ITEM_SPRITE)
#define	FNT_ITEM_FONT			(ITEM_FONT)
#define	FNT_LAYER_SIZE			(LAYER_SIZE)

#define	FNT_NO_ERROR			(0)
#define	FNT_ERROR				(1)
#define	FNT_NOT_FOUND			(0)
#define	FNT_FOUND				(1)
#define	FNT_INVALID				(0xFFFFFFFF)

#define	FNT_FTYP_POS_HI			(4)
#define	FNT_FTYP_POS_LO			(5)
#define	FNT_FCSA_POS_HI			(7)
#define	FNT_FCSA_POS_LO			(8)
#define	FNT_FTYP_MASK_HI		(0x03)
#define	FNT_FTYP_MASK_LO		(0xFF)
#define	FNT_FCSA_MASK_HI		(0x70)
#define	FNT_FCSA_MASK_LO		(0xFF)
#define	FNT_FSZX_POS			(4)
#define	FNT_FSZY_POS			(5)
#define	FNT_FSZX_MASK			(0x3F)
#define	FNT_FSZY_MASK			(0x3F)

#define	FNT_SHIFT_1BIT			(1)
#define	FNT_SHIFT_2BIT			(2)
#define	FNT_SHIFT_4BIT			(4)
#define	FNT_SHIFT_8BIT			(8)

#define	FNT_DISPLAYTEXT_MAX		(64)
#define	FNT_PFONT_SIZE			(4)
#define	FNT_SPACE_CODE			(0x0020)

#define TEXTALIGNX_LEFT					(0)
#define TEXTALIGNX_CENTER				(1)
#define TEXTALIGNX_RIGHT				(2)
/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/

U16 ConvFCodeString(const T_YGFONTCTRL *t_fontctrl, U08 *pu1_Utf16, U08 u1_Utf16Num,U08 *pu1_FCode, U16 u2_FCodeByteNum);
U16 ConvFCodeString_utf8(const T_YGFONTCTRL *t_fontctrl, C08 *pu1_Utf8, U16 u1_Utf16Num,U08 *pu1_FCode, U16 u2_FCodeByteNum);
U16 YVC1_SetChar(T_YGFONTCTRL tYgfontCtrl , C08 *text );
#endif


/* ----------------------------- E O F -----------------------------------------*/

