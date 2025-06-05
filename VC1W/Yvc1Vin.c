/********************************************************************************
 *
 *		CONFIDENTIAL
 *
 *		Copyright (C) 2014-2015 Yamaha Corporation. All rights reserved.
 *
 *		Module			: $Workfile: YVC1Vin.c $
 *
 *		Description		: VC1W External video image input Function
 *
 *		Version			: $Rev: 37 $
 *
 *		Last UpDate Time: $Date:: 2015-01-07 17:44:06#$
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
#include "Yvc1Vin.h"


/*------------------------------------------------------------------------------
 *						D E F I N E
 *------------------------------------------------------------------------------*/
#define YVC1VIN_DIGITAL_DVIF_MAX		(2)
#define YVC1VIN_DIGITAL_DVINTL_MAX		(1)
#define YVC1VIN_DIGITAL_DVPAL_MAX		(1)
#define YVC1VIN_DIGITAL_DVSP_MAX		(1)
#define YVC1VIN_DIGITAL_CRCBS_MAX		(1)

#define YVC1VIN_DIGITAL_DVCSE_MAX		(1)
#define YVC1VIN_DIGITAL_DVCSM_MAX		(1)
#define YVC1VIN_DIGITAL_DVHTL_MAX		(2047)
#define YVC1VIN_DIGITAL_DVHSW_MAX		(511)

#define YVC1VIN_BCD_STADOT_MIN			(8.0F)
#define YVC1VIN_BCD_STADOT_MAX			(2047.75F)
#define YVC1VIN_BCD_AREA_SX_MIN			(1)
#define YVC1VIN_BCD_AREA_SY_MIN			(1)
#define YVC1VIN_BCD_AREA_SX_MAX			(2047)
#define YVC1VIN_BCD_AREA_SY_MAX			(1023)
#define YVC1VIN_BCD_AREA_W_MIN			(1)
#define YVC1VIN_BCD_AREA_W_MAX			(2047)
#define YVC1VIN_BCD_AREA_H_MIN			(1)
#define YVC1VIN_BCD_AREA_H_MAX			(1023)
#define YVC1VIN_BCD_SCALE_MODE_MAX		(1)
#define YVC1VIN_BCD_SCALE_FX_MIN		(0.25F)
#define YVC1VIN_BCD_SCALE_FX_MAX		(4096.0F)
#define YVC1VIN_BCD_SCALE_FY_MIN		(0.25F)
#define YVC1VIN_BCD_SCALE_FY_MAX		(4096.0F)

#define YVC1VIN_BCD_XPOS_CALC_DATA1		(4.0F)
#define YVC1VIN_BCD_SCALE_ADJ			(0x4000U)	/* Fx and Fy correction(x2.5)		*/
#define YVC1VIN_BCD_SCALE_ADJ_DATA		(0x3FFFU)
#define YVC1VIN_BCD_SCALE_CALC_DATA1	(1.0F)		/* For inverse number calculation	*/
#define YVC1VIN_BCD_SCALE_CALC_DATA2	(4096)		/* For inverse number calculation	*/
#define YVC1VIN_BCD_VSA_VSADJ_MIN		(-32)
#define YVC1VIN_BCD_VSA_VSADJ_MAX		(31)

#define YVC1VIN_BCD_EXTSYNC_ON			(1)
#define YVC1VIN_BCD_EXTSYNC_OFF			(0)
#define YVC1VIN_BCD_DISP_ON				(1)
#define YVC1VIN_BCD_DISP_OFF			(0)
#define YVC1VIN_BCD_AUTOOFF_ON			(1)
#define YVC1VIN_BCD_AUTOOFF_OFF			(0)
#define YVC1VIN_BCD_XFLIP_ON			(1)
#define YVC1VIN_BCD_XFLIP_OFF			(0)
#define YVC1VIN_BCD_HLFE_ON				(1)
#define YVC1VIN_BCD_HLFE_OFF			(0)



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
/* Function name    : YVC1_VinGamma                                             */
/* Contents         : Gamma correction setting                                  */
/* Argument         : U08 GAM        (W) Gamma value                            */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinGamma(U08 GAM)
{
	BOOL        bRet      = TRUE;
	T_Y643_RE5H tY643RE5h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if (GAM > 2)
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*-----------------------------------------
		 *	R#E5h register set
		 *	(External Video Input)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_ReadReg(REG_RE5H, &tY643RE5h.BYTE);
		tY643RE5h.BIT.EXGAM = GAM;
		YVC1_WriteReg(REG_RE5H, tY643RE5h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinDigitalIn                                         */
/* Contents         : Digital video image signal setting                        */
/* Argument         : T_YVC1_VIN_DIGITAL_IN *tYvc1VinDigitalIn                  */
/*                    [-> U08 DVIF   (W) Digital video input format           ] */
/*                    [-> U08 DVINTL (W) Digital video input scanning mode    ] */
/*                    [-> U08 DVPAL  (W) Digital video PAL selection          ] */
/*                    [-> U08 DVSP   (W) Sync polarity selection              ] */
/*                    [-> U08 CRCBS  (W) Color-difference signal swap         ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinDigitalIn(const T_YVC1_VIN_DIGITAL_IN *tYvc1VinDigitalIn)
{
	BOOL bRet             = TRUE;
	T_Y643_RE5H tY643RE5h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1VinDigitalIn == (const T_YVC1_VIN_DIGITAL_IN*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		if ( (tYvc1VinDigitalIn->DVIF > YVC1VIN_DIGITAL_DVIF_MAX) ||
			 (tYvc1VinDigitalIn->DVINTL > YVC1VIN_DIGITAL_DVINTL_MAX) ||
			 (tYvc1VinDigitalIn->DVPAL > YVC1VIN_DIGITAL_DVPAL_MAX) ||
			 (tYvc1VinDigitalIn->DVSP > YVC1VIN_DIGITAL_DVSP_MAX) ||
			 (tYvc1VinDigitalIn->CRCBS > YVC1VIN_DIGITAL_CRCBS_MAX) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643RE5h.BIT.DVIF   = tYvc1VinDigitalIn->DVIF;
		tY643RE5h.BIT.DVINTL = tYvc1VinDigitalIn->DVINTL;
		tY643RE5h.BIT.DVPAL  = tYvc1VinDigitalIn->DVPAL;
		tY643RE5h.BIT.DVSP   = tYvc1VinDigitalIn->DVSP;
		tY643RE5h.BIT.CRCBS  = tYvc1VinDigitalIn->CRCBS;

		/*-----------------------------------------
		 *	R#E5h register set
		 *	(External Video Input)
		 *-----------------------------------------*/
		YVC1_WriteReg(REG_RE5H, tY643RE5h.BYTE);
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinDigitalSync                                       */
/* Contents         : Digital video composite sync setting                      */
/* Argument         : T_YVC1_VIN_DIGITAL_SYNC *tYvc1VinDigitalSync              */
/*                    [-> U08 DVCSE  (W) Digital Video Composite Sync Input     */
/*                                                                    Enable  ] */
/*                    [-> U08 DVCSM  (W) Digital Video Composite sync           */
/*                                                            Separation Mode ] */
/*                    [-> U16 DVHTL  (W) Digital Video horizontal Total       ] */
/*                    [-> U16 DVHSW  (W) Digital Video Horizontal Sync Width  ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinDigitalSync(const T_YVC1_VIN_DIGITAL_SYNC *tYvc1VinDigitalSync)
{
	BOOL bRet             = TRUE;
	T_Y643_RE6H tY643RE6h = {0};
	T_Y643_RE7H tY643RE7h = {0};
	T_Y643_RE8H tY643RE8h = {0};
	T_Y643_RE9H tY643RE9h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1VinDigitalSync == (const T_YVC1_VIN_DIGITAL_SYNC*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		if ( (tYvc1VinDigitalSync->DVCSE > YVC1VIN_DIGITAL_DVCSE_MAX) ||
			 (tYvc1VinDigitalSync->DVCSM > YVC1VIN_DIGITAL_DVCSM_MAX) ||
			 (tYvc1VinDigitalSync->DVHTL > YVC1VIN_DIGITAL_DVHTL_MAX) ||
			 (tYvc1VinDigitalSync->DVHSW > YVC1VIN_DIGITAL_DVHSW_MAX) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		tY643RE6h.BIT.DVCSE   = tYvc1VinDigitalSync->DVCSE;
		tY643RE6h.BIT.DVCSM   = tYvc1VinDigitalSync->DVCSM;
		tY643RE6h.BIT.DVHTL_8 = (U08)((tYvc1VinDigitalSync->DVHTL >> SHIFT_08BIT) & MASK_07H);
		tY643RE7h.BIT.DVHTL_0 = (U08)(tYvc1VinDigitalSync->DVHTL                  & MASK_FFH);
		tY643RE8h.BIT.DVHSW_8 = (U08)((tYvc1VinDigitalSync->DVHSW >> SHIFT_08BIT) & MASK_01H);
		tY643RE9h.BIT.DVHSW_0 = (U08)(tYvc1VinDigitalSync->DVHSW                  & MASK_FFH);

		/*-----------------------------------------
		 *	R#E6h-R#E9h register set
		 *	(External Video Input)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_RE6H, tY643RE6h.BYTE);
		YVC1_WriteReg(REG_RE7H, tY643RE7h.BYTE);
		YVC1_WriteReg(REG_RE8H, tY643RE8h.BYTE);
		YVC1_WriteReg(REG_RE9H, tY643RE9h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdExtSync                                        */
/* Contents         : External synchronization control                          */
/* Argument         : BOOL Enable    (W) External synchronization setting       */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdExtSync(BOOL Enable)
{
	BOOL bRet             = TRUE;
	T_Y643_RE4H tY643RE4h = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	/*-----------------------------------------
	 *	R#E4h register set
	 *	(External Synchronization)
	 *-----------------------------------------*/
	EXCTRL_PORT_LOCK_REG
	YVC1_ReadReg(REG_RE4H, &tY643RE4h.BYTE);
	if (Enable == TRUE)
	{
		tY643RE4h.BIT.SYNCE = YVC1VIN_BCD_EXTSYNC_ON;
	}
	else
	{
		tY643RE4h.BIT.SYNCE = YVC1VIN_BCD_EXTSYNC_OFF;
	}
	YVC1_WriteReg(REG_RE4H, tY643RE4h.BYTE);
	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdDisp                                           */
/* Contents         : Backdrop plane - Display control                          */
/* Argument         : BOOL Disp      (W) Backdrop plane - Display enable setting */
/*                    BOOL AutoOff   (W) Backdrop plane display permission      */
/*                                       setup at the time of a non-signal      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdDisp(BOOL Disp, BOOL AutoOff)
{
	BOOL bRet             = TRUE;
	T_Y643_REAH tY643REAh = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	/*-----------------------------------------
	 *	R#EAh register set
	 *	(Backdrop Plane)
	 *-----------------------------------------*/
	EXCTRL_PORT_LOCK_REG
	YVC1_ReadReg(REG_REAH, &tY643REAh.BYTE);

	if (Disp == TRUE)
	{
		tY643REAh.BIT.BPD = YVC1VIN_BCD_DISP_ON;
	}
	else
	{
		tY643REAh.BIT.BPD = YVC1VIN_BCD_DISP_OFF;
	}

	if (AutoOff == TRUE)
	{
		tY643REAh.BIT.BPAOFF = YVC1VIN_BCD_AUTOOFF_ON;
	}
	else
	{
		tY643REAh.BIT.BPAOFF = YVC1VIN_BCD_AUTOOFF_OFF;
	}

	YVC1_WriteReg(REG_REAH, tY643REAh.BYTE);
	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdXPos                                           */
/* Contents         : Backdrop plane display start coordinate setting           */
/* Argument         : F32 StaDot     (W) Display start dot setting of the       */
/*                                                         external video       */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdXPos(F32 StaDot)
{
	BOOL bRet             = TRUE;
	T_Y643_REBH tY643REBh = {0};
	T_Y643_RECH tY643RECh = {0};
	U16 u16Eisd           = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ((StaDot < YVC1VIN_BCD_STADOT_MIN) || (StaDot > YVC1VIN_BCD_STADOT_MAX))
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*-----------------------------------------
		 *	R#EBh-R#ECh register set
		 *	(Backdrop Plane)
		 *-----------------------------------------*/
		u16Eisd = (U16)(StaDot * YVC1VIN_BCD_XPOS_CALC_DATA1);

		tY643REBh.BIT.EISD_8 = (U08)((u16Eisd >> SHIFT_08BIT) & MASK_1FH);
		tY643RECh.BIT.EISD_0 = (U08)(u16Eisd                  & MASK_FFH);

		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_REBH, tY643REBh.BYTE);
		YVC1_WriteReg(REG_RECH, tY643RECh.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdArea                                           */
/* Contents         : Backdrop plane display area specification                 */
/* Argument         : T_YVC1_VIN_BCD_AREA *tYvc1VinBcdArea                      */
/* 					  [-> U16 Sx     (W) Backdrop plane start dot             ] */
/* 					  [-> U16 Sy     (W) Backdrop plane start line            ] */
/* 					  [-> U16 Width  (W) Backdrop plane display width         ] */
/* 					  [-> U16 Height (W) Backdrop plane display height        ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdArea(const T_YVC1_VIN_BCD_AREA *tYvc1VinBcdArea)
{
	BOOL bRet             = TRUE;
	T_Y643_REDH tY643REDh = {0};
	T_Y643_REEH tY643REEh = {0};
	T_Y643_REFH tY643REFh = {0};
	T_Y643_RF0H tY643RF0h = {0};
	T_Y643_RF1H tY643RF1h = {0};
	T_Y643_RF2H tY643RF2h = {0};
	T_Y643_RF3H tY643RF3h = {0};
	T_Y643_RF4H tY643RF4h = {0};
	U16 u16Sx = 0;
	U16 u16Sy = 0;
	U16 u16Ex = 0;
	U16 u16Ey = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1VinBcdArea == (const T_YVC1_VIN_BCD_AREA*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		u16Sx = tYvc1VinBcdArea->Sx;
		u16Sy = tYvc1VinBcdArea->Sy;
		u16Ex = tYvc1VinBcdArea->Sx + tYvc1VinBcdArea->Width;
		u16Ey = tYvc1VinBcdArea->Sy + tYvc1VinBcdArea->Height;

		if ( ((u16Sx < YVC1VIN_BCD_AREA_SX_MIN) || (u16Sx > YVC1VIN_BCD_AREA_SX_MAX)) ||
			 ((u16Sy < YVC1VIN_BCD_AREA_SY_MIN) || (u16Sy > YVC1VIN_BCD_AREA_SY_MAX)) ||
			 (tYvc1VinBcdArea->Width < YVC1VIN_BCD_AREA_W_MIN) ||
			 (tYvc1VinBcdArea->Height < YVC1VIN_BCD_AREA_H_MIN) ||
			 (u16Ex > YVC1VIN_BCD_AREA_W_MAX) ||
			 (u16Ey > YVC1VIN_BCD_AREA_H_MAX) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		u16Sx--;
		u16Sy--;
		u16Ex--;
		u16Ey--;

		tY643REDh.BIT.BPSL_8 = (U08)((u16Sy >> SHIFT_08BIT) & MASK_03H);
		tY643REEh.BIT.BPSL_0 = (U08)(u16Sy                  & MASK_FFH);
		tY643REFh.BIT.BPSD_8 = (U08)((u16Sx >> SHIFT_08BIT) & MASK_07H);
		tY643RF0h.BIT.BPSD_0 = (U08)(u16Sx                  & MASK_FFH);
		tY643RF1h.BIT.BPEL_8 = (U08)((u16Ey >> SHIFT_08BIT) & MASK_03H);
		tY643RF2h.BIT.BPEL_0 = (U08)(u16Ey                  & MASK_FFH);
		tY643RF3h.BIT.BPED_8 = (U08)((u16Ex >> SHIFT_08BIT) & MASK_07H);
		tY643RF4h.BIT.BPED_0 = (U08)(u16Ex                  & MASK_FFH);

		/*-----------------------------------------
		 *	R#EDh-R#F4h register set
		 *	(Backdrop Plane)
		 *-----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_WriteReg(REG_REDH, tY643REDh.BYTE);
		YVC1_WriteReg(REG_REEH, tY643REEh.BYTE);
		YVC1_WriteReg(REG_REFH, tY643REFh.BYTE);
		YVC1_WriteReg(REG_RF0H, tY643RF0h.BYTE);
		YVC1_WriteReg(REG_RF1H, tY643RF1h.BYTE);
		YVC1_WriteReg(REG_RF2H, tY643RF2h.BYTE);
		YVC1_WriteReg(REG_RF3H, tY643RF3h.BYTE);
		YVC1_WriteReg(REG_RF4H, tY643RF4h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdScale                                          */
/* Contents         : Backdrop plane Scaling setting                            */
/* Argument         : T_YVC1_VIN_BCD_SCALE *tYvc1VinBcdScale                    */
/*                    [-> BOOL Enable (W) Scaling function ON/OFF             ] */
/*                    [-> U08 Mode    (W) Interpolation Method Selection      ] */
/*                    [-> F32 Fx      (W) X Scaling Setting                   ] */
/*                    [-> F32 Fy      (W) Y Scaling Setting                   ] */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdScale(const T_YVC1_VIN_BCD_SCALE *tYvc1VinBcdScale)
{
	BOOL bRet             = TRUE;
	T_Y643_REAH tY643REAh = {0};
	T_Y643_RF5H tY643RF5h = {0};
	T_Y643_RF6H tY643RF6h = {0};
	T_Y643_RF7H tY643RF7h = {0};
	T_Y643_RF8H tY643RF8h = {0};
	U16 u16Fx = 0;
	U16 u16Fy = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	/*--- NULL Check	---*/
	if(tYvc1VinBcdScale == (const T_YVC1_VIN_BCD_SCALE*)NULL)
	{
		bRet = FALSE;
	}
	if (bRet == TRUE)
	{
		if ( (tYvc1VinBcdScale->Mode > YVC1VIN_BCD_SCALE_MODE_MAX) ||
			 ((tYvc1VinBcdScale->Fx < YVC1VIN_BCD_SCALE_FX_MIN) || (tYvc1VinBcdScale->Fx > YVC1VIN_BCD_SCALE_FX_MAX)) ||
			 ((tYvc1VinBcdScale->Fy < YVC1VIN_BCD_SCALE_FY_MIN) || (tYvc1VinBcdScale->Fy > YVC1VIN_BCD_SCALE_FY_MAX)) )
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/* The register value is set to 0x3FFF when the value of Fx and Fy is 0.25.		*/
		u16Fx = (U16)((YVC1VIN_BCD_SCALE_CALC_DATA1 / tYvc1VinBcdScale->Fx) * YVC1VIN_BCD_SCALE_CALC_DATA2);
		u16Fy = (U16)((YVC1VIN_BCD_SCALE_CALC_DATA1 / tYvc1VinBcdScale->Fy) * YVC1VIN_BCD_SCALE_CALC_DATA2);
		if (u16Fx == YVC1VIN_BCD_SCALE_ADJ)
		{
			u16Fx = (U16)YVC1VIN_BCD_SCALE_ADJ_DATA;
		}
		if (u16Fy == YVC1VIN_BCD_SCALE_ADJ)
		{
			u16Fy = (U16)YVC1VIN_BCD_SCALE_ADJ_DATA;
		}

		EXCTRL_PORT_LOCK_REG
		YVC1_ReadReg(REG_REAH, &tY643REAh.BYTE);
		if (tYvc1VinBcdScale->Enable == TRUE)
		{
			tY643REAh.BIT.SCE = 1;
		}
		else
		{
			tY643REAh.BIT.SCE = 0;
		}
		tY643REAh.BIT.SFSEL  = tYvc1VinBcdScale->Mode;
		tY643RF5h.BIT.SCFY_8 = (U08)((u16Fy >> SHIFT_08BIT) & MASK_3FH);
		tY643RF6h.BIT.SCFY_0 = (U08)(u16Fy                  & MASK_FFH);
		tY643RF7h.BIT.SCFX_8 = (U08)((u16Fx >> SHIFT_08BIT) & MASK_3FH);
		tY643RF8h.BIT.SCFX_0 = (U08)(u16Fx                  & MASK_FFH);

		/*-----------------------------------------
		 *	R#EAh,R#F5h-R#F8h register set
		 *	(Backdrop Plane)
		 *-----------------------------------------*/
		YVC1_WriteReg(REG_REAH, tY643REAh.BYTE);
		YVC1_WriteReg(REG_RF5H, tY643RF5h.BYTE);
		YVC1_WriteReg(REG_RF6H, tY643RF6h.BYTE);
		YVC1_WriteReg(REG_RF7H, tY643RF7h.BYTE);
		YVC1_WriteReg(REG_RF8H, tY643RF8h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdXFlip                                          */
/* Contents         : Backdrop plane mirror inversion function                  */
/* Argument         : BOOL Enable    (W) Backdrop plane mirror inversion        */
/*                                                             enable setting   */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdXFlip(BOOL Enable)
{
	BOOL        bRet      = TRUE;
	T_Y643_REAH tY643REAh = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	EXCTRL_PORT_LOCK_REG
	YVC1_ReadReg(REG_REAH, &tY643REAh.BYTE);
	if (Enable == TRUE)
	{
		tY643REAh.BIT.BPFLX = YVC1VIN_BCD_XFLIP_ON;
	}
	else
	{
		tY643REAh.BIT.BPFLX = YVC1VIN_BCD_XFLIP_OFF;
	}

	/*-----------------------------------------
	 *	R#EAh register set
	 *	(Backdrop Plane)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_REAH, tY643REAh.BYTE);
	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdVSA                                            */
/* Contents         : Backdrop plane display position adjustment function       */
/* Argument         : S08 VsAdj      (W) Backdrop plane display position        */
/*                                                      adjustment setting      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdVSA(S08 VsAdj)
{
	BOOL        bRet      = TRUE;
	T_Y643_RE4H tY643RE4h = {0};

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ((VsAdj < YVC1VIN_BCD_VSA_VSADJ_MIN) || (VsAdj > YVC1VIN_BCD_VSA_VSADJ_MAX))
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*----------------------------------------
		 *	R#E4h register set
		 *	(External Synchronization)
		 *----------------------------------------*/
		EXCTRL_PORT_LOCK_REG
		YVC1_ReadReg(REG_RE4H, &tY643RE4h.BYTE);
		tY643RE4h.BIT.VSADJ = (U08)VsAdj;
		YVC1_WriteReg(REG_RE4H, tY643RE4h.BYTE);
		EXCTRL_PORT_UNLOCK_REG
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YVC1_VinBcdHLFE                                           */
/* Contents         : 2nd field image position correction function              */
/* Argument         : BOOL Enable    (W) 2nd field image position correction    */
/*                                                 function enable setting      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YVC1_VinBcdHLFE(BOOL Enable)
{
	BOOL bRet             = TRUE;
	T_Y643_REAH tY643REAh = {0};

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	EXCTRL_PORT_LOCK_REG
	YVC1_ReadReg(REG_REAH, &tY643REAh.BYTE);
	if (Enable == TRUE)
	{
		tY643REAh.BIT.BPHLFE = YVC1VIN_BCD_HLFE_ON;
	}
	else
	{
		tY643REAh.BIT.BPHLFE = YVC1VIN_BCD_HLFE_OFF;
	}

	/*-----------------------------------------
	 *	R#EAh register set
	 *	(Backdrop Plane)
	 *-----------------------------------------*/
	YVC1_WriteReg(REG_REAH, tY643REAh.BYTE);
	EXCTRL_PORT_UNLOCK_REG

	return bRet;
}


/*------------------------------- E O F ----------------------------------------*/

