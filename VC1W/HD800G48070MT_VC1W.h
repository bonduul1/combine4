/***********************************************************************************************************************
* File Name    : HD800G48070MT_VC1W.h
* Version      : Excel Macro AutoGenerate for VC1x [Ver 1.1]
* Device       : VC1W(YGV643) 
* Description  : This file lcd timming setting values for XFL Driver.
* Creation Date: 2018-02-06- 14:23:54
* Xin Clock    : 8MHz
***********************************************************************************************************************/

#ifndef LCD_TIMMING_H
#define LCD_TIMMING_H

static const T_YVC1_DATA tYvc1Data = {
        // INNOLUX and MIND LCD --> CLOCK = 8 * 22 / 5 = 35.2MHz
//#if defined (LCD_AT070TN94) || defined(LCD_MD070NF04_60ID_18A_AM)
        {
		0x00,  // |                 PLLR[4:0]                  |   0    |   0    | DTSEL1 |
		0x16,  // |   0    |                      PLLF[6:0]                               |
		0x85   // | DTSEL0 | REVCK  |                          DTDV[5:0]                  |
	},
	{
		0x12,  // |  INTL  | CSYPAL | CSYOE  | VSTM   | REVSY  | REVBL  |     VTL[9:8]    |
		0x0C,  // |                               VTL[7:0]                                |
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HTL[10:8]         |
		0xD1,  // |                               HTL[7:0]                                |
		0x01,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VBLS[9:8]    |
		0xE5,  // |                               VBLS[7:0]                               |
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HBLS[10:8]        |
		0x60,  // |                               HBLS[7:0]                               |
		0x00,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VBLE[9:8]    |
		0x05,  // |                               VBLE[7:0]                               |
		0x00,  // |   0    |   0    |   0    |   0    |   0    |        HBLE[10:8]        |
		0x40,  // |                               HBLE[7:0]                               |
		0x08,  // |                 PDDS[4:0]                  |  NMCMD |    VDS[9:8]     |
		0x05,  // |                               VDS[7:0]                                |
		0x00,  // |   0    |   0    |   0    |   0    |   0    |        HDS[10:8]         |
		0x40,  // |                               HDS[7:0]                                |
		0x01,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VDE[9:8]     |
		0xE5,  // |                               VDE[7:0]                                |
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HDE[10:8]         |
		0x60,  // |                               HDE[7:0]                                |
		0x04,  // |   0    |   0    |     HSW[9:8]    |              VSW[3:0]             |
		0x2F,  // |                               HSW[7:0]                                |
		0x03   // |DMASEL1 |DMASEL0 |   0    |   0    |  EVEN  |   0    |  SYEN  |  DTEN  |
	},
        /*
#elif defined(LCD_RFF700A9_AWH_DNN)
        // RAYSTART LCD --> CLOCK = 8 * 17 / 5 = 27.2
        {
		0x00,  // |                 PLLR[4:0]                  |   0    |   0    | DTSEL1 |
		0x11,  // |   0    |                      PLLF[6:0]                               |
		0x85   // | DTSEL0 | REVCK  |                          DTDV[5:0]                  |
	},
        {
              0x12,  // |  INTL  | CSYPAL | CSYOE  | VSTM   | REVSY  | REVBL  |     VTL[9:8]    |
              0x10,  // |                               VTL[7:0]                                |       // 528         --> V Total Line
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HTL[10:8]         |
		0x5C,  // |                               HTL[7:0]                                |             // 860 --> H Total Line
              0x01,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VBLS[9:8]    |
              0xE5,  // |                               VBLS[7:0]                               |       // 485         --> V Blank Start
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HBLS[10:8]        |
		0x30,  // |                               HBLS[7:0]                               |             // 816 --> H Blank start position
              0x00,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VBLE[9:8]    |
              0x05,  // |                               VBLE[7:0]                               |       // 5          --> Top start position
		0x00,  // |   0    |   0    |   0    |   0    |   0    |        HBLE[10:8]        |
		0x10,  // |                               HBLE[7:0]                               |             // 16 --> Left start position
              0x08,  // |                 PDDS[4:0]                  |  NMCMD |    VDS[9:8]     |
              0x05,  // |                               VDS[7:0]                                |       // 5          --> V Data start position
		0x00,  // |   0    |   0    |   0    |   0    |   0    |        HDS[10:8]         |
		0x10,  // |                               HDS[7:0]                                |             // 16 --> H Data start position
              0x01,  // |   0    |   0    |   0    |   0    |   0    |   0    |    VDE[9:8]     |
              0xE5,  // |                               VDE[7:0]                                |       // 485        --> Data End
		0x03,  // |   0    |   0    |   0    |   0    |   0    |        HDE[10:8]         |
		0x30,  // |                               HDE[7:0]                                |             // 816 --> Blank start position
              0x02,  // |   0    |   0    |     HSW[9:8]    |              VSW[3:0]             |       // 2         --> V pulse width
		0x02,  // |                               HSW[7:0]                                |             // 2 --> H pulse width
		0x03   // |DMASEL1 |DMASEL0 |   0    |   0    |  EVEN  |   0    |  SYEN  |  DTEN  |
	},
#endif
        */
		0x00,  // |   0    |   0    |   0    |   0    |   0    |  FRCE  |  FRCM  | VOBSEL |
	{
		0x00,  // | REVSH  |   0    |       STHE[10:8]         |       STHS[10:8]         |
		0x00,  // |                               STHE[7:0]                               |
		0x00,  // |                               STHS[7:0]                               |
		0x00,  // | REVLH  |   0    |       LDHE[10:8]         |       LDHS[10:8]         |
		0x00,  // |                               LDHE[7:0]                               |
		0x00,  // |                               LDHS[7:0]                               |
		0x00,  // | REVCV  |   0    |       CKVE[10:8]         |       CKVS[10:8]         |
		0x00,  // |                               CKVE[7:0]                               |
		0x00,  // |                               CKVS[7:0]                               |
		0x00,  // | REVSV  | REVSVV |   0    |   0    |   STVEV[9:8]    |   STVSV[9:8]    |
		0x00,  // |                               STVEV[7:0]                              |
		0x00,  // |                               STVSV[7:0]                              |
		0x00,  // | REVSVH |   0    |        STVEH[10:8]       |       STVSH[10:8]        |
		0x00,  // |                               STVEH[7:0]                              |
		0x00,  // |                               STVSH[7:0]                              |
		0x00,  // | REVOE  | REVOEV |   0    |   0    |    OEEV[9:8]    |    OESV[9:8]    |
		0x00,  // |                               OEEV[7:0]                               |
		0x00,  // |                               OESV[7:0]                               |
		0x00,  // | REVOEH |   0    |         OEEH[10:8]       |        OESH[10:8]        |
		0x00,  // |                               OEEH[7:0]                               |
		0x00,  // |                               OESH[7:0]                               |
		0x00,  // | POLEN  |   0    |         POLM[2:0]        |        POLP[10:8]        |
		0x00   // |                               POLP[7:0]                               |
	}
};
#endif
