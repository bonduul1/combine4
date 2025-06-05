/********************************************************************************
 *
 *		CONFIDENTIAL
 *
 *		Copyright (C) 2014-2015 Yamaha Corporation. All rights reserved.
 *
 *		Module			: $Workfile: YVC1Wrp.c $
 *
 *		Description		: VC1W Device Driver Warping control Function
 *
 *		Version			: $Rev: 192 $
 *
 *		Last UpDate Time: $Date:: 2014-09-19 17:15:23#$
 *
 *		FOOT NOTE		: adjust 4tab
 *
 *		AUTHOR			: H.Katayama
 *
 ********************************************************************************/

/*------------------------------------------------------------------------------
 *						I N C L U D E
 *------------------------------------------------------------------------------*/
#include "Yvc1.h"
#include "Yvc1Wrp.h"


/*------------------------------------------------------------------------------
 *						D E F I N E
 *------------------------------------------------------------------------------*/
#define YVC1WRP_WPEN_ON					(1)
#define YVC1WRP_WPEN_OFF				(0)
#define YVC1WRP_WCEN_ON					(1)
#define YVC1WRP_WCEN_OFF				(0)

#define YVC1WRP_CPTR_SX_MAX				(1023)
#define YVC1WRP_CPTR_SY_MAX				(1023)
#define YVC1WRP_CPTR_W_MIN				(1)
#define YVC1WRP_CPTR_W_MAX				(960)
#define YVC1WRP_CPTR_H_MIN				(1)
#define YVC1WRP_CPTR_H_MAX				(640)
#define YVC1WRP_CPTR_BUFFSIZE_X_MIN		(8)
#define YVC1WRP_CPTR_BUFFSIZE_X_MAX		(1024)
#define YVC1WRP_CPTR_BUFFSIZE_X_8N		(8)
#define YVC1WRP_CPTR_LINEDELAY_MIN		(2)
#define YVC1WRP_CPTR_LINEDELAY_MAX		(1025)
#define YVC1WRP_DISP_SETTINGS_BLF_MAX	(1)
#define YVC1WRP_DISP_SETTINGS_EEF_MAX	(1)
#define YVC1WRP_DISP_SETTINGS_GME_MAX	(1)
#define YVC1WRP_DISP_SETTINGS_CTC_MAX	(1)
#define YVC1WRP_BRIGHT_MIN				(-256)
#define YVC1WRP_BRIGHT_MAX				(255)
#define YVC1WRP_HUE_MIN					(-125)
#define YVC1WRP_HUE_MAX					(125)
#define YVC1WRP_COF_WCSX_MAX			(1023)
#define YVC1WRP_COF_WCSY_MAX			(1023)
#define YVC1WRP_COF_WCSTPX_MAX			(1023)
#define YVC1WRP_COF_WCSTPY_MAX			(1023)
#define YVC1WRP_COF_WCNUMX_MAX			(1023)
#define YVC1WRP_COF_WCNUMY_MAX			(1023)
#define YVC1WRP_COF_WCIPMH_MAX			(1)
#define YVC1WRP_COF_WCIPMV_MAX			(1)
#define YVC1WRP_COF_WTBLSEL_MAX			(1)
#define YVC1WRP_COF_WPBA_WTBLSEL_0_MAX	(0x1FFF)
#define YVC1WRP_COF_WPBA_WTBLSEL_1_MAX	(0x3FFFFFF)
#define YVC1WRP_COF_WCCOMP_MAX			(1)
#define YVC1WRP_COF_WPBA_WTBLSEL_0_MASK	(0x1FFF)
#define YVC1WRP_COF_WPBA_WTBLSEL_1_MASK	(0x3FFFFFF)
#define YVC1WRP_COF_CODR_MAX			(5)
#define YVC1WRP_COF_CC0TY_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC0TX_MAX			(0x1FFFF)
#define YVC1WRP_COF_CA_MAX				(0xFFF)
#define YVC1WRP_COF_CB_MAX				(0xFFF)
#define YVC1WRP_COF_CC_MAX				(0xFFF)
#define YVC1WRP_COF_CD_MAX				(0xFFF)
#define YVC1WRP_COF_CC1SELA_MAX			(1)
#define YVC1WRP_COF_CC1SELY_MAX			(3)
#define YVC1WRP_COF_CC1SELX_MAX			(3)
#define YVC1WRP_COF_CC1SELSH_MAX		(1)
#define YVC1WRP_COF_CC1TY_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC1TX_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC1KC_MAX			(0x3FFF)
#define YVC1WRP_COF_CC2SELA_MAX			(1)
#define YVC1WRP_COF_CC2SELB_MAX			(1)
#define YVC1WRP_COF_CC2SELC_MAX			(1)
#define YVC1WRP_COF_CC2SELY_MAX			(2)
#define YVC1WRP_COF_CC2SELX_MAX			(2)
#define YVC1WRP_COF_CC2SELSH_MAX		(1)
#define YVC1WRP_COF_CC2TY_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC2TX_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC2KC_MAX			(0x3FFF)
#define YVC1WRP_COF_CC3TY_MAX			(0x1FFFF)
#define YVC1WRP_COF_CC3TX_MAX			(0x1FFFF)


/*------------------------------------------------------------------------------
 *						S T R U C T
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						V A R I A B L E S
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						E X T E R N
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						P R O T O T Y P E
 *------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpEnable                                            */
/* Contents         : Warping Enable / Disable                                  */
/* Argument         : BOOL Enable    (W) Warping function - ON/OFF              */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpEnable(BOOL Enable)
{
	BOOL        bRet      = TRUE;
	T_Y643_R80H tY643R80h = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	/*-----------------------------------------
	 *	R#E5h register set
	 *	(External Video Input)
	 *-----------------------------------------*/
	if (Enable == TRUE)
	{
		tY643R80h.BIT.WPEN = YVC1WRP_WPEN_ON;
	}
	else
	{
		tY643R80h.BIT.WPEN = YVC1WRP_WPEN_OFF;
	}
	YVC1_WriteReg(REG_R80H, tY643R80h.BYTE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpCptr                                              */
/* Contents         : Capture Buffer Controls setting                           */
/* Argument         : T_YVC1_WRP_CPTR *tYvc1WrpCptr                             */
/*                    [-> U16 Sx        (W) Capture area start X coordinates  ] */
/*                    [-> U16 Sy        (W) Capture area start Y coordinates  ] */
/*                    [-> U16 Width     (W) Capture area width coordinates    ] */
/*                    [-> U16 Height    (W) Capture area height coordinates   ] */
/*                    [-> U16 BuffSizeX (W) Capture Buffer X size             ] */
/*                    [-> U16 LineDelay (W) Capture lead delay timing (line)  ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpCptr(const T_YVC1_WRP_CPTR *tYvc1WrpCptr)
{
	BOOL        bRet      = TRUE;
	T_Y643_R0AH tY643R0Ah = {0};
	T_Y643_R0BH tY643R0Bh = {0};
	T_Y643_R0CH tY643R0Ch = {0};
	T_Y643_R0DH tY643R0Dh = {0};
	T_Y643_R81H tY643R81h = {0};
	T_Y643_R82H tY643R82h = {0};
	T_Y643_R83H tY643R83h = {0};
	T_Y643_R84H tY643R84h = {0};
	T_Y643_R85H tY643R85h = {0};
	T_Y643_R86H tY643R86h = {0};
	T_Y643_R87H tY643R87h = {0};
	T_Y643_R88H tY643R88h = {0};
	T_Y643_R89H tY643R89h = {0};
	T_Y643_R92H tY643R92h = {0};
	T_Y643_R93H tY643R93h = {0};
	U16 u16Sx             = 0;
	U16 u16Sy             = 0;
	U16 u16Ex             = 0;
	U16 u16Ey             = 0;
	U16 u16BuffSizeX      = 0;
	U16 u16LineDelay      = 0;
	U16 u16LIDL           = 0;
	U16 u16Hbls           = 0;
	U16 u16Vbls           = 0;
	U08 u08FBX            = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1WrpCptr == (const T_YVC1_WRP_CPTR*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		u16Sx        = tYvc1WrpCptr->Sx;
		u16Sy        = tYvc1WrpCptr->Sy;
		u16Ex        = tYvc1WrpCptr->Sx + tYvc1WrpCptr->Width;
		u16Ey        = tYvc1WrpCptr->Sy + tYvc1WrpCptr->Height;
		u16BuffSizeX = tYvc1WrpCptr->BuffSizeX;
		u16LineDelay = tYvc1WrpCptr->LineDelay;
		YVC1_ReadReg(REG_R0AH, &tY643R0Ah.BYTE);
		YVC1_ReadReg(REG_R0BH, &tY643R0Bh.BYTE);
		YVC1_ReadReg(REG_R0CH, &tY643R0Ch.BYTE);
		YVC1_ReadReg(REG_R0DH, &tY643R0Dh.BYTE);
		u16Hbls = ((U16)(tY643R0Ch.BIT.HBLS_8) << SHIFT_08BIT) | (U16)(tY643R0Dh.BIT.HBLS_0);
		u16Vbls = ((U16)(tY643R0Ah.BIT.VBLS_8) << SHIFT_08BIT) | (U16)(tY643R0Bh.BIT.VBLS_0);

		if ( (u16Sx > YVC1WRP_CPTR_SX_MAX) ||
			 (u16Sy > YVC1WRP_CPTR_SY_MAX) ||
			 (tYvc1WrpCptr->Width < YVC1WRP_CPTR_W_MIN) ||
			 (tYvc1WrpCptr->Height < YVC1WRP_CPTR_H_MIN) ||
			 (u16Ex > YVC1WRP_CPTR_W_MAX) ||
			 (u16Ey > YVC1WRP_CPTR_H_MAX) ||
			 (u16Ex > u16Hbls) ||
			 (u16Ey > u16Vbls) ||
			 ((u16BuffSizeX < YVC1WRP_CPTR_BUFFSIZE_X_MIN) || (u16BuffSizeX > YVC1WRP_CPTR_BUFFSIZE_X_MAX)) ||
			 ((u16BuffSizeX % YVC1WRP_CPTR_BUFFSIZE_X_8N) != 0) ||
			 ((u16LineDelay < YVC1WRP_CPTR_LINEDELAY_MIN) || (u16LineDelay > YVC1WRP_CPTR_LINEDELAY_MAX)) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Ex--;
		u16Ey--;
		u16LIDL = u16LineDelay - 2;					/* LIDL[9:0]+2 The number of delay lines				*/
		u08FBX  = (U08)((u16BuffSizeX / 4) - 1);	/* (FBX[7:0]+1)x4 = X Size, FBX[7:0]+1 = Even number	*/

		tY643R81h.BIT.CAPSY_8 = (U08)((u16Sy >> SHIFT_08BIT) & MASK_03H);
		tY643R82h.BIT.CAPSY_0 = (U08)(u16Sy                  & MASK_FFH);
		tY643R83h.BIT.CAPSX_8 = (U08)((u16Sx >> SHIFT_08BIT) & MASK_07H);
		tY643R84h.BIT.CAPSX_0 = (U08)(u16Sx                  & MASK_FFH);
		tY643R85h.BIT.CAPEY_8 = (U08)((u16Ey >> SHIFT_08BIT) & MASK_03H);
		tY643R86h.BIT.CAPEY_0 = (U08)(u16Ey                  & MASK_FFH);
		tY643R87h.BIT.CAPEX_8 = (U08)((u16Ex >> SHIFT_08BIT) & MASK_07H);
		tY643R88h.BIT.CAPEX_0 = (U08)(u16Ex                  & MASK_FFH);
		tY643R89h.BIT.FBX     = u08FBX;
		tY643R92h.BIT.LIDL_8  = (U08)((u16LIDL >> SHIFT_08BIT) & MASK_03H);
		tY643R93h.BIT.LIDL_0  = (U08)(u16LIDL                  & MASK_FFH);

		/*-----------------------------------------
		 *	R#81h-R#89hÅAR#92h-R#93h register set
		 *	(Capture Controls , Capture Buffer Control)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_R81H, tY643R81h.BYTE);
		YVC1_WriteReg(REG_R82H, tY643R82h.BYTE);
		YVC1_WriteReg(REG_R83H, tY643R83h.BYTE);
		YVC1_WriteReg(REG_R84H, tY643R84h.BYTE);
		YVC1_WriteReg(REG_R85H, tY643R85h.BYTE);
		YVC1_WriteReg(REG_R86H, tY643R86h.BYTE);
		YVC1_WriteReg(REG_R87H, tY643R87h.BYTE);
		YVC1_WriteReg(REG_R88H, tY643R88h.BYTE);
		YVC1_WriteReg(REG_R89H, tY643R89h.BYTE);
		YVC1_WriteReg(REG_R92H, tY643R92h.BYTE);
		YVC1_WriteReg(REG_R93H, tY643R93h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpDispSettings                                      */
/* Contents         : Video output setting                                      */
/* Argument         : U08 BLF        (W) Bilinear filter-Enable  - ON/OFF       */
/*                  : U08 EEF        (W) Edge enhancement filter - ON/OFF       */
/*                  : U08 GME        (W) Gamma Correction        - ON/OFF       */
/*                  : U08 CTC        (W) Color Tone controls     - ON/OFF       */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpDispSettings(U08 BLF, U08 EEF, U08 GME, U08 CTC)
{
	BOOL        bRet      = TRUE;
	T_Y643_R94H tY643R94h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (BLF > YVC1WRP_DISP_SETTINGS_BLF_MAX) ||
		 (EEF > YVC1WRP_DISP_SETTINGS_EEF_MAX) ||
		 (GME > YVC1WRP_DISP_SETTINGS_GME_MAX) ||
		 (CTC > YVC1WRP_DISP_SETTINGS_CTC_MAX) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643R94h.BIT.BLF = BLF;
		tY643R94h.BIT.EEF = EEF;
		tY643R94h.BIT.GME = GME;
		tY643R94h.BIT.CTC = CTC;

		/*-----------------------------------------
		 *	R#94 register set
		 *	(Video output)
		 *-----------------------------------------*/
		YVC1_WriteReg(REG_R94H, tY643R94h.BYTE);
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpEdgFlt                                            */
/* Contents         : Edge Enhancement Filter setting                           */
/* Argument         : U08 EEFC       (W) Edge Enhancement Filter parameter      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpEdgFlt(U08 EEFC)
{
	BOOL        bRet      = TRUE;
	T_Y643_R9AH tY643R9Ah = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	tY643R9Ah.BIT.EEFC = EEFC;

	/*-----------------------------------------
	 *	R#9A register set
	 *	(Edge Enhancement Filter)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_R9AH, tY643R9Ah.BYTE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1WrpCont                                               */
/* Contents         : contrast adjustment control setting                       */
/* Argument         : U08 Cont       (W) Contrast adjustment value              */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpCont(U08 Cont)
{
	BOOL        bRet      = TRUE;
	T_Y643_R95H tY643R95h = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	tY643R95h.BIT.DCON = Cont;

	/*-----------------------------------------
	 *	R#95 register set
	 *	(Color Tone Controls)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_R95H, tY643R95h.BYTE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1WrpBright                                             */
/* Contents         : Brightness adjustment control setting                     */
/* Argument         : S16 Bright     (W) Brightness adjustment value            */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpBright(S16 Bright)
{
	BOOL        bRet      = TRUE;
	T_Y643_R96H tY643R96h = {0};
	T_Y643_R97H tY643R97h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ((Bright < YVC1WRP_BRIGHT_MIN) || (Bright > YVC1WRP_BRIGHT_MAX))
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643R96h.BIT.DBRI_8 = (U08)(((U16)Bright >> SHIFT_08BIT) & MASK_01H);
		tY643R97h.BIT.DBRI_0 =  (U08)((U16)Bright                 & MASK_FFH);

		/*-----------------------------------------
		 *	R#94 register set
		 *	(âfëúèoóÕ)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_R96H, tY643R96h.BYTE);
		YVC1_WriteReg(REG_R97H, tY643R97h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpSat                                               */
/* Contents         : Saturation adjustment function                            */
/* Argument         : U08 Sat        (W) Saturation adjustment value            */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpSat(U08 Sat)
{
	BOOL        bRet      = TRUE;
	T_Y643_R98H tY643R98h = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	tY643R98h.BIT.DSATU = Sat;

	/*-----------------------------------------
	 *	R#98 register set
	 *	(Color tone adjustment)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_R98H, tY643R98h.BYTE);

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpHue                                               */
/* Contents         : Hue regulating function                                   */
/* Argument         : S08 Hue        (W) Hue adjustment value                   */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpHue(S08 Hue)
{
	BOOL        bRet      = TRUE;
	T_Y643_R99H tY643R99h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ((Hue < YVC1WRP_HUE_MIN) || (Hue > YVC1WRP_HUE_MAX))
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643R99h.BIT.DHUE = (U08)Hue;

		/*-----------------------------------------
		 *	R#99 register set
		 *	(Color tone adjustment)
		 *-----------------------------------------*/
		YVC1_WriteReg(REG_R99H, tY643R99h.BYTE);
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpCofEnable                                         */
/* Contents         : Warping coefficient Enable / Disable                      */
/* Argument         : BOOL Enable    (W) Warping coefficient function - ON/OFF  */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpCofEnable(BOOL Enable)
{
	BOOL        bRet      = TRUE;
	T_Y643_R9DH tY643R9Dh = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	EXCTRL_PORT_LOCK_REG
	YVC1_ReadReg(REG_R9DH, &tY643R9Dh.BYTE);
	if (Enable == TRUE)
	{
		tY643R9Dh.BIT.WCEN = YVC1WRP_WCEN_ON;
	}
	else
	{
		tY643R9Dh.BIT.WCEN = YVC1WRP_WCEN_OFF;
	}

	/*-----------------------------------------
	 *	R#9D register set
	 *	(Warping coefficient)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_R9DH, tY643R9Dh.BYTE);
	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}



/*------------------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpCofInit                                                       */
/* Contents         : Warping coefficient setting                                           */
/* Argument         : T_YVC1_WRP_COF_INIT *tYvc1WrpCofInit                                  */
/*                    [-> U16 WCSX      (W) Warping parameter start X coordinate          ] */
/*                    [-> U16 WCSY      (W) Warping parameter start Y coordinate          ] */
/*                    [-> U16 WCSTPX    (W) Warping parameter X direction intervals       ] */
/*                    [-> U16 WCSTPY    (W) Warping parameter Y direction intervals       ] */
/*                    [-> U16 WCNUMX    (W) Number of X-direction warping parameters      ] */
/*                    [-> U16 WCNUMY    (W) Number of Y-direction warping parameters      ] */
/*                    [-> U08 WCIPMH    (W) X-direction interpolation mode                ] */
/*                    [-> U08 WCIPMV    (W) Y-direction interpolation mode                ] */
/*                    [-> U08 WTBLSEL   (W) Space to store warping parameters to use      ] */
/*                    [-> U32 WPBA      (W) First address of warping parameters stored    ] */
/*                    [-> U08 WCCOMP    (W) Compresses or Uncompresses warping parameters ] */
/*                                                                                          */
/* Return value     : TRUE or FALSE                                                         */
/*------------------------------------------------------------------------------------------*/
BOOL YVC1_WrpCofInit(const T_YVC1_WRP_COF_INIT *tYvc1WrpCofInit)
{
	BOOL        bRet      = TRUE;
	T_Y643_R9DH tY643R9Dh = {0};
	T_Y643_R9EH tY643R9Eh = {0};
	T_Y643_R9FH tY643R9Fh = {0};
	T_Y643_RA0H tY643RA0h = {0};
	T_Y643_RA1H tY643RA1h = {0};
	T_Y643_RA2H tY643RA2h = {0};
	T_Y643_RA3H tY643RA3h = {0};
	T_Y643_RA4H tY643RA4h = {0};
	T_Y643_RA5H tY643RA5h = {0};
	T_Y643_RA6H tY643RA6h = {0};
	T_Y643_RA7H tY643RA7h = {0};
	T_Y643_RA8H tY643RA8h = {0};
	T_Y643_RA9H tY643RA9h = {0};
	T_Y643_RAAH tY643RAAh = {0};
	T_Y643_RABH tY643RABh = {0};
	T_Y643_RACH tY643RACh = {0};
	T_Y643_RADH tY643RADh = {0};

	U16 u16WCSX    = 0;
	U16 u16WCSY    = 0;
	U16 u16WCSTPX  = 0;
	U16 u16WCSTPY  = 0;
	U16 u16WCNUMX  = 0;
	U16 u16WCNUMY  = 0;
	U08 u08WCIPMH  = 0;
	U08 u08WCIPMV  = 0;
	U08 u08WTBLSEL = 0;
	U32 u32WPBA    = 0;
	U08 u08WCCOMP  = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1WrpCofInit == (const T_YVC1_WRP_COF_INIT*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		u16WCSX    = tYvc1WrpCofInit->WCSX;
		u16WCSY    = tYvc1WrpCofInit->WCSY;
		u16WCSTPX  = tYvc1WrpCofInit->WCSTPX;
		u16WCSTPY  = tYvc1WrpCofInit->WCSTPY;
		u16WCNUMX  = tYvc1WrpCofInit->WCNUMX;
		u16WCNUMY  = tYvc1WrpCofInit->WCNUMY;
		u08WCIPMH  = tYvc1WrpCofInit->WCIPMH;
		u08WCIPMV  = tYvc1WrpCofInit->WCIPMV;
		u08WTBLSEL = tYvc1WrpCofInit->WTBLSEL;
		u32WPBA    = tYvc1WrpCofInit->WPBA;
		u08WCCOMP  = tYvc1WrpCofInit->WCCOMP;

		if ( (u16WCSX > YVC1WRP_COF_WCSX_MAX) ||
			 (u16WCSY > YVC1WRP_COF_WCSY_MAX) ||
			 (u16WCSTPX > YVC1WRP_COF_WCSTPX_MAX) ||
			 (u16WCSTPY > YVC1WRP_COF_WCSTPY_MAX) ||
			 ( ((u16WCSTPX == 0) && (u16WCSTPY != 0)) || ((u16WCSTPX != 0) && (u16WCSTPY == 0)) ) ||
			 ( ((u16WCSTPX == 0) && (u16WCNUMX == 0)) ) ||						/* Inhibition condition 0: WCSTPX=0	No Interpolation		*/
			 ( ((u16WCSTPY == 0) && (u16WCNUMY == 0)) ) ||						/*                       : WCSTPY=0	No Interpolation		*/
			 ( ((u16WCSTPX != 0) && (u08WCIPMH == 0) && (u16WCNUMX <= 1)) ) ||	/* Inhibition condition 1: WCIPMH=0 1st Order Interpolation	*/
			 ( ((u16WCSTPY != 0) && (u08WCIPMV == 0) && (u16WCNUMY <= 1)) ) ||	/*                       : WCIPMV=0 1st Order Interpolation	*/
			 ( ((u16WCSTPX != 0) && (u08WCIPMH == 1) && (u16WCNUMX <= 2)) ) ||	/* Inhibition condition 2: WCIPMH=1 2nd Order Interpolation	*/
			 ( ((u16WCSTPY != 0) && (u08WCIPMV == 1) && (u16WCNUMY <= 2)) ) ||	/*                       : WCIPMV=1 2nd Order Interpolation	*/
			 (u16WCNUMX > YVC1WRP_COF_WCNUMX_MAX) ||
			 (u16WCNUMY > YVC1WRP_COF_WCNUMY_MAX) ||
			 (u08WCIPMH > YVC1WRP_COF_WCIPMH_MAX) ||
			 (u08WCIPMV > YVC1WRP_COF_WCIPMV_MAX) ||
			 (u08WTBLSEL > YVC1WRP_COF_WTBLSEL_MAX) )
		{
			bRet = FALSE;
		}

		if (u08WTBLSEL == 0)
		{
			if (u32WPBA > YVC1WRP_COF_WPBA_WTBLSEL_0_MAX)
			{
				bRet = FALSE;
			}
			u32WPBA &= YVC1WRP_COF_WPBA_WTBLSEL_0_MASK;
		}
		else
		{
			if (u32WPBA > YVC1WRP_COF_WPBA_WTBLSEL_1_MAX)
			{
				bRet = FALSE;
			}
			u32WPBA &= YVC1WRP_COF_WPBA_WTBLSEL_1_MASK;
		}
		if (u08WCCOMP > YVC1WRP_COF_WCCOMP_MAX)
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		YVC1_ReadReg(REG_R9DH, &tY643R9Dh.BYTE);
		tY643R9Dh.BIT.WCIPMV   = u08WCIPMV;
		tY643R9Dh.BIT.WCIPMH   = u08WCIPMH;
		tY643R9Dh.BIT.WTBLSEL  = u08WTBLSEL;
		tY643R9Dh.BIT.WCCOMP   = u08WCCOMP;
		tY643R9Eh.BIT.WCSY_8   = (U08)((u16WCSY >> SHIFT_08BIT)   & MASK_03H);
		tY643R9Fh.BIT.WCSY_0   = (U08)( u16WCSY                   & MASK_FFH);
		tY643RA0h.BIT.WCSX_8   = (U08)((u16WCSX >> SHIFT_08BIT)   & MASK_03H);
		tY643RA1h.BIT.WCSX_0   = (U08)( u16WCSX                   & MASK_FFH);
		tY643RA2h.BIT.WCSTPY_8 = (U08)((u16WCSTPY >> SHIFT_08BIT) & MASK_03H);
		tY643RA3h.BIT.WCSTPY_0 = (U08)( u16WCSTPY                 & MASK_FFH);
		tY643RA4h.BIT.WCSTPX_8 = (U08)((u16WCSTPX >> SHIFT_08BIT) & MASK_03H);
		tY643RA5h.BIT.WCSTPX_0 = (U08)( u16WCSTPX                 & MASK_FFH);
		tY643RA6h.BIT.WCNUMY_8 = (U08)((u16WCNUMY >> SHIFT_08BIT) & MASK_03H);
		tY643RA7h.BIT.WCNUMY_0 = (U08)( u16WCNUMY                 & MASK_FFH);
		tY643RA8h.BIT.WCNUMX_8 = (U08)((u16WCNUMX >> SHIFT_08BIT) & MASK_03H);
		tY643RA9h.BIT.WCNUMX_0 = (U08)( u16WCNUMX                 & MASK_FFH);
		tY643RAAh.BIT.WPBA_24  = (U08)((u32WPBA >> SHIFT_24BIT)   & MASK_03H);
		tY643RABh.BIT.WPBA_16  = (U08)((u32WPBA >> SHIFT_16BIT)   & MASK_FFH);
		tY643RACh.BIT.WPBA_8   = (U08)((u32WPBA >> SHIFT_08BIT)   & MASK_FFH);
		tY643RADh.BIT.WPBA_0   = (U08)((u32WPBA               )   & MASK_FFH);

		/*-----------------------------------------
		 *	R#9Dh-R#ADh register set
		 *	(Warping coefficient)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_R9DH, tY643R9Dh.BYTE);
		YVC1_WriteReg(REG_R9EH, tY643R9Eh.BYTE);
		YVC1_WriteReg(REG_R9FH, tY643R9Fh.BYTE);
		YVC1_WriteReg(REG_RA0H, tY643RA0h.BYTE);
		YVC1_WriteReg(REG_RA1H, tY643RA1h.BYTE);
		YVC1_WriteReg(REG_RA2H, tY643RA2h.BYTE);
		YVC1_WriteReg(REG_RA3H, tY643RA3h.BYTE);
		YVC1_WriteReg(REG_RA4H, tY643RA4h.BYTE);
		YVC1_WriteReg(REG_RA5H, tY643RA5h.BYTE);
		YVC1_WriteReg(REG_RA6H, tY643RA6h.BYTE);
		YVC1_WriteReg(REG_RA7H, tY643RA7h.BYTE);
		YVC1_WriteReg(REG_RA8H, tY643RA8h.BYTE);
		YVC1_WriteReg(REG_RA9H, tY643RA9h.BYTE);
		YVC1_WriteReg(REG_RAAH, tY643RAAh.BYTE);
		YVC1_WriteReg(REG_RABH, tY643RABh.BYTE);
		YVC1_WriteReg(REG_RACH, tY643RACh.BYTE);
		YVC1_WriteReg(REG_RADH, tY643RADh.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_WrpCofCmps                                           */
/* Contents         : Warping Parameter Corrections setting                     */
/* Argument         : T_YVC1_WRP_COF_CMPS *tYvc1WrpCofCmps                      */
/*                                      === Correction calculation circuit ==== */
/*                    [-> U08 CODR      (W) - Computation sequence selection    */
/*                                      === Coefficient calculation circuit 0 = */
/*                    [-> U32 CC0TY     (W) - Coefficient Ty                    */
/*                    [-> U32 CC0TX     (W) - Coefficient Tx                    */
/*                    [-> U16 CA        (W) - Coefficient Ca                    */
/*                    [-> U16 CB        (W) - Coefficient Cb                    */
/*                    [-> U16 CC        (W) - Coefficient Cc                    */
/*                    [-> U16 CD        (W) - Coefficient Cd                    */
/*                                      === Coefficient calculation circuit 1 = */
/*                    [-> U08 CC1SELA   (W) - Multiplier input A selection      */
/*                    [-> U08 CC1SELY   (W) - Y output selection                */
/*                    [-> U08 CC1SELX   (W) - X output selection                */
/*                    [-> U08 CC1SELSH  (W) - Output multiplier shift count     */
/*                                                                    selection */
/*                    [-> U32 CC1TY     (W) - Coefficient Ty                    */
/*                    [-> U32 CC1TX     (W) - Coefficient Tx                    */
/*                    [-> U16 CC1KC     (W) - Coefficient Kc                    */
/*                                      === Coefficient calculation circuit 2 = */
/*                    [-> U08 CC2SELA   (W) - Multiplier input A selection      */
/*                    [-> U08 CC2SELB   (W) - Multiplier input B selection      */
/*                    [-> U08 CC2SELC   (W) - Adding machine input selection    */
/*                    [-> U08 CC2SELY   (W) - Y output selection                */
/*                    [-> U08 CC2SELX   (W) - X output selection                */
/*                    [-> U08 CC2SELSH  (W) - Output multiplier shift count     */
/*                                                                    selection */
/*                    [-> U32 CC2TY     (W) - Coefficient Ty                    */
/*                    [-> U32 CC2TX     (W) - Coefficient Tx                    */
/*                    [-> U16 CC2KC     (W) - Coefficient Kc                    */
/*                                      === Coefficient calculation circuit 3 = */
/*                    [-> U32 CC3TY     (W) - Coefficient Ty                    */
/*                    [-> U32 CC3TX     (W) - Coefficient Tx                    */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_WrpCofCmps(const T_YVC1_WRP_COF_CMPS *tYvc1WrpCofCmps)
{
	BOOL        bRet      = TRUE;
	T_Y643_RBBH tY643RBBh = {0};
	T_Y643_RBCH tY643RBCh = {0};
	T_Y643_RBDH tY643RBDh = {0};
	T_Y643_RBEH tY643RBEh = {0};
	T_Y643_RBFH tY643RBFh = {0};
	T_Y643_RC0H tY643RC0h = {0};
	T_Y643_RC1H tY643RC1h = {0};
	T_Y643_RC2H tY643RC2h = {0};
	T_Y643_RC3H tY643RC3h = {0};
	T_Y643_RC4H tY643RC4h = {0};
	T_Y643_RC5H tY643RC5h = {0};
	T_Y643_RC6H tY643RC6h = {0};
	T_Y643_RC7H tY643RC7h = {0};
	T_Y643_RC8H tY643RC8h = {0};
	T_Y643_RC9H tY643RC9h = {0};
	T_Y643_RCAH tY643RCAh = {0};
	T_Y643_RCBH tY643RCBh = {0};
	T_Y643_RCCH tY643RCCh = {0};
	T_Y643_RCDH tY643RCDh = {0};
	T_Y643_RCEH tY643RCEh = {0};
	T_Y643_RCFH tY643RCFh = {0};
	T_Y643_RD0H tY643RD0h = {0};
	T_Y643_RD1H tY643RD1h = {0};
	T_Y643_RD2H tY643RD2h = {0};
	T_Y643_RD3H tY643RD3h = {0};
	T_Y643_RD4H tY643RD4h = {0};
	T_Y643_RD5H tY643RD5h = {0};
	T_Y643_RD6H tY643RD6h = {0};
	T_Y643_RD7H tY643RD7h = {0};
	T_Y643_RD8H tY643RD8h = {0};
	T_Y643_RD9H tY643RD9h = {0};
	T_Y643_RDAH tY643RDAh = {0};
	T_Y643_RDBH tY643RDBh = {0};
	T_Y643_RDCH tY643RDCh = {0};
	T_Y643_RDDH tY643RDDh = {0};
	T_Y643_RDEH tY643RDEh = {0};
	T_Y643_RDFH tY643RDFh = {0};
	T_Y643_RE0H tY643RE0h = {0};
	T_Y643_RE1H tY643RE1h = {0};
	U32 u32CC0TY = 0;
	U32 u32CC0TX = 0;
	U16 u16CA    = 0;
	U16 u16CB    = 0;
	U16 u16CC    = 0;
	U16 u16CD    = 0;
	U32 u32CC1TY = 0;
	U32 u32CC1TX = 0;
	U16 u16CC1KC = 0;
	U32 u32CC2TY = 0;
	U32 u32CC2TX = 0;
	U16 u16CC2KC = 0;
	U32 u32CC3TY = 0;
	U32 u32CC3TX = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1WrpCofCmps == (const T_YVC1_WRP_COF_CMPS*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		u32CC0TY    = tYvc1WrpCofCmps->CC0TY;
		u32CC0TX    = tYvc1WrpCofCmps->CC0TX;
		u16CA       = tYvc1WrpCofCmps->CA;
		u16CB       = tYvc1WrpCofCmps->CB;
		u16CC       = tYvc1WrpCofCmps->CC;
		u16CD       = tYvc1WrpCofCmps->CD;
		u32CC1TY    = tYvc1WrpCofCmps->CC1TY;
		u32CC1TX    = tYvc1WrpCofCmps->CC1TX;
		u16CC1KC    = tYvc1WrpCofCmps->CC1KC;
		u32CC2TY    = tYvc1WrpCofCmps->CC2TY;
		u32CC2TX    = tYvc1WrpCofCmps->CC2TX;
		u16CC2KC    = tYvc1WrpCofCmps->CC2KC;
		u32CC3TY    = tYvc1WrpCofCmps->CC3TY;
		u32CC3TX    = tYvc1WrpCofCmps->CC3TX;

		if ( (tYvc1WrpCofCmps->CODR > YVC1WRP_COF_CODR_MAX) ||
			 (u32CC0TY > YVC1WRP_COF_CC0TY_MAX) ||
			 (u32CC0TX > YVC1WRP_COF_CC0TX_MAX) ||
			 (u16CA > YVC1WRP_COF_CA_MAX) ||
			 (u16CB > YVC1WRP_COF_CB_MAX) ||
			 (u16CC > YVC1WRP_COF_CC_MAX) ||
			 (u16CD > YVC1WRP_COF_CD_MAX) ||
			 (tYvc1WrpCofCmps->CC1SELA > YVC1WRP_COF_CC1SELA_MAX) ||
			 (tYvc1WrpCofCmps->CC1SELY > YVC1WRP_COF_CC1SELY_MAX) ||
			 (tYvc1WrpCofCmps->CC1SELX > YVC1WRP_COF_CC1SELX_MAX) ||
			 (tYvc1WrpCofCmps->CC1SELSH > YVC1WRP_COF_CC1SELSH_MAX) ||
			 (u32CC1TY > YVC1WRP_COF_CC1TY_MAX) ||
			 (u32CC1TX > YVC1WRP_COF_CC1TX_MAX) ||
			 (u16CC1KC > YVC1WRP_COF_CC1KC_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELA > YVC1WRP_COF_CC2SELA_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELB > YVC1WRP_COF_CC2SELB_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELC > YVC1WRP_COF_CC2SELC_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELY > YVC1WRP_COF_CC2SELY_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELX > YVC1WRP_COF_CC2SELX_MAX) ||
			 (tYvc1WrpCofCmps->CC2SELSH > YVC1WRP_COF_CC2SELSH_MAX) ||
			 (u32CC2TY > YVC1WRP_COF_CC2TY_MAX) ||
			 (u32CC2TX > YVC1WRP_COF_CC2TX_MAX) ||
			 (u16CC2KC > YVC1WRP_COF_CC2KC_MAX) ||
			 (u32CC3TY > YVC1WRP_COF_CC3TY_MAX) ||
			 (u32CC3TX > YVC1WRP_COF_CC3TX_MAX) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643RBBh.BIT.CODR     = tYvc1WrpCofCmps->CODR;
		tY643RBCh.BIT.CC0TY_16 = (U08)((u32CC0TY >> SHIFT_16BIT) & MASK_01H);
		tY643RBDh.BIT.CC0TY_8  = (U08)((u32CC0TY >> SHIFT_08BIT) & MASK_FFH);
		tY643RBEh.BIT.CC0TY_0  = (U08)((u32CC0TY               ) & MASK_FFH);
		tY643RBFh.BIT.CC0TX_16 = (U08)((u32CC0TX >> SHIFT_16BIT) & MASK_01H);
		tY643RC0h.BIT.CC0TX_8  = (U08)((u32CC0TX >> SHIFT_08BIT) & MASK_FFH);
		tY643RC1h.BIT.CC0TX_0  = (U08)((u32CC0TX               ) & MASK_FFH);
		tY643RC2h.BIT.CA_8     = (U08)((u16CA >> SHIFT_08BIT)    & MASK_FFH);
		tY643RC3h.BIT.CA_0     = (U08)((u16CA               )    & MASK_FFH);
		tY643RC4h.BIT.CB_8     = (U08)((u16CB >> SHIFT_08BIT)    & MASK_FFH);
		tY643RC5h.BIT.CB_0     = (U08)((u16CB               )    & MASK_FFH);
		tY643RC6h.BIT.CC_8     = (U08)((u16CC >> SHIFT_08BIT)    & MASK_FFH);
		tY643RC7h.BIT.CC_0     = (U08)((u16CC               )    & MASK_FFH);
		tY643RC8h.BIT.CD_8     = (U08)((u16CD >> SHIFT_08BIT)    & MASK_FFH);
		tY643RC9h.BIT.CD_0     = (U08)((u16CD               )    & MASK_FFH);
		tY643RCAh.BIT.CC1SELA  = tYvc1WrpCofCmps->CC1SELA;
		tY643RCAh.BIT.CC1SELY  = tYvc1WrpCofCmps->CC1SELY;
		tY643RCAh.BIT.CC1SELX  = tYvc1WrpCofCmps->CC1SELX;
		tY643RCAh.BIT.CC1SELSH = tYvc1WrpCofCmps->CC1SELSH;
		tY643RCBh.BIT.CC1TY_16 = (U08)((u32CC1TY >> SHIFT_16BIT) & MASK_01H);
		tY643RCCh.BIT.CC1TY_8  = (U08)((u32CC1TY >> SHIFT_08BIT) & MASK_FFH);
		tY643RCDh.BIT.CC1TY_0  = (U08)((u32CC1TY               ) & MASK_FFH);
		tY643RCEh.BIT.CC1TX_16 = (U08)((u32CC1TX >> SHIFT_16BIT) & MASK_01H);
		tY643RCFh.BIT.CC1TX_8  = (U08)((u32CC1TX >> SHIFT_08BIT) & MASK_FFH);
		tY643RD0h.BIT.CC1TX_0  = (U08)((u32CC1TX               ) & MASK_FFH);
		tY643RD1h.BIT.CC1KC_8  = (U08)((u16CC1KC >> SHIFT_08BIT) & MASK_FFH);
		tY643RD2h.BIT.CC1KC_0  = (U08)((u16CC1KC               ) & MASK_FFH);
		tY643RD3h.BIT.CC2SELA  = tYvc1WrpCofCmps->CC2SELA;
		tY643RD3h.BIT.CC2SELB  = tYvc1WrpCofCmps->CC2SELB;
		tY643RD3h.BIT.CC2SELC  = tYvc1WrpCofCmps->CC2SELC;
		tY643RD3h.BIT.CC2SELY  = tYvc1WrpCofCmps->CC2SELY;
		tY643RD3h.BIT.CC2SELX  = tYvc1WrpCofCmps->CC2SELX;
		tY643RD3h.BIT.CC2SELSH = tYvc1WrpCofCmps->CC2SELSH;
		tY643RD4h.BIT.CC2TY_16 = (U08)((u32CC2TY >> SHIFT_16BIT) & MASK_01H);
		tY643RD5h.BIT.CC2TY_8  = (U08)((u32CC2TY >> SHIFT_08BIT) & MASK_FFH);
		tY643RD6h.BIT.CC2TY_0  = (U08)((u32CC2TY               ) & MASK_FFH);
		tY643RD7h.BIT.CC2TX_16 = (U08)((u32CC2TX >> SHIFT_16BIT) & MASK_01H);
		tY643RD8h.BIT.CC2TX_8  = (U08)((u32CC2TX >> SHIFT_08BIT) & MASK_FFH);
		tY643RD9h.BIT.CC2TX_0  = (U08)((u32CC2TX               ) & MASK_FFH);
		tY643RDAh.BIT.CC2KC_8  = (U08)((u16CC2KC >> SHIFT_08BIT) & MASK_FFH);
		tY643RDBh.BIT.CC2KC_0  = (U08)((u16CC2KC               ) & MASK_FFH);
		tY643RDCh.BIT.CC3TY_16 = (U08)((u32CC3TY >> SHIFT_16BIT) & MASK_01H);
		tY643RDDh.BIT.CC3TY_8  = (U08)((u32CC3TY >> SHIFT_08BIT) & MASK_FFH);
		tY643RDEh.BIT.CC3TY_0  = (U08)((u32CC3TY               ) & MASK_FFH);
		tY643RDFh.BIT.CC3TX_16 = (U08)((u32CC3TX >> SHIFT_16BIT) & MASK_01H);
		tY643RE0h.BIT.CC3TX_8  = (U08)((u32CC3TX >> SHIFT_08BIT) & MASK_FFH);
		tY643RE1h.BIT.CC3TX_0  = (U08)((u32CC3TX               ) & MASK_FFH);
		/*-----------------------------------------
		 *	R#BBh-R#E1h register set
		 *	(Warping coefficient)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_RBBH, tY643RBBh.BYTE);
		YVC1_WriteReg(REG_RBCH, tY643RBCh.BYTE);
		YVC1_WriteReg(REG_RBDH, tY643RBDh.BYTE);
		YVC1_WriteReg(REG_RBEH, tY643RBEh.BYTE);
		YVC1_WriteReg(REG_RBFH, tY643RBFh.BYTE);
		YVC1_WriteReg(REG_RC0H, tY643RC0h.BYTE);
		YVC1_WriteReg(REG_RC1H, tY643RC1h.BYTE);
		YVC1_WriteReg(REG_RC2H, tY643RC2h.BYTE);
		YVC1_WriteReg(REG_RC3H, tY643RC3h.BYTE);
		YVC1_WriteReg(REG_RC4H, tY643RC4h.BYTE);
		YVC1_WriteReg(REG_RC5H, tY643RC5h.BYTE);
		YVC1_WriteReg(REG_RC6H, tY643RC6h.BYTE);
		YVC1_WriteReg(REG_RC7H, tY643RC7h.BYTE);
		YVC1_WriteReg(REG_RC8H, tY643RC8h.BYTE);
		YVC1_WriteReg(REG_RC9H, tY643RC9h.BYTE);
		YVC1_WriteReg(REG_RCAH, tY643RCAh.BYTE);
		YVC1_WriteReg(REG_RCBH, tY643RCBh.BYTE);
		YVC1_WriteReg(REG_RCCH, tY643RCCh.BYTE);
		YVC1_WriteReg(REG_RCDH, tY643RCDh.BYTE);
		YVC1_WriteReg(REG_RCEH, tY643RCEh.BYTE);
		YVC1_WriteReg(REG_RCFH, tY643RCFh.BYTE);
		YVC1_WriteReg(REG_RD0H, tY643RD0h.BYTE);
		YVC1_WriteReg(REG_RD1H, tY643RD1h.BYTE);
		YVC1_WriteReg(REG_RD2H, tY643RD2h.BYTE);
		YVC1_WriteReg(REG_RD3H, tY643RD3h.BYTE);
		YVC1_WriteReg(REG_RD4H, tY643RD4h.BYTE);
		YVC1_WriteReg(REG_RD5H, tY643RD5h.BYTE);
		YVC1_WriteReg(REG_RD6H, tY643RD6h.BYTE);
		YVC1_WriteReg(REG_RD7H, tY643RD7h.BYTE);
		YVC1_WriteReg(REG_RD8H, tY643RD8h.BYTE);
		YVC1_WriteReg(REG_RD9H, tY643RD9h.BYTE);
		YVC1_WriteReg(REG_RDAH, tY643RDAh.BYTE);
		YVC1_WriteReg(REG_RDBH, tY643RDBh.BYTE);
		YVC1_WriteReg(REG_RDCH, tY643RDCh.BYTE);
		YVC1_WriteReg(REG_RDDH, tY643RDDh.BYTE);
		YVC1_WriteReg(REG_RDEH, tY643RDEh.BYTE);
		YVC1_WriteReg(REG_RDFH, tY643RDFh.BYTE);
		YVC1_WriteReg(REG_RE0H, tY643RE0h.BYTE);
		YVC1_WriteReg(REG_RE1H, tY643RE1h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}


/*------------------------------ E O F -----------------------------------------*/

