
/*
 * @File name - pcode.h
 * @Author    - Enkhbat Batbayar
 * @Date      - 2020 Nov 12
 * 
 *
 *
 */
#ifndef __PCODE_H
#define __PCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define NUMBER_OF_PCODE         286
#define PCODE_LENGTH            5
#define SPN_LENGTH              6

#define FAIL                    1
#define AWL                     2
#define RSL                     4

#define NO_COMES_ON             0
#define APP_S                   FAIL
#define FAIL_AWL                FAIL | AWL
#define FAIL_RSL                FAIL | RSL

// J1939-73 Diagnostic Layer
#define PGN_DM1             65226
#define PGN_DM2             65227
#define PGN_DM3             65228
#define PGN_DM5             65230
#define PGN_DM11            65235
#define PGN_DM13            57088                               // Stop & Start Broadcast

#define ECU_ID_DM1          0x18FECA00
#define DCU_ID_DM1          0x18FECA3D
#define ECU_ID_DM2          0x18FECB00
#define DCU_ID_DM2          0x18FECB3D
//#define EDCU_ID_DM2       0x18FECB3D                          // currently inactive in this case
#define ECU_ID_DM5          0x18FECE00
//#define EDCU_ID_DM11      0x18FECB3D                          // currently inactive in this case
#define EDCU_ID_DM13        0x18DFFF00                          // the last two byte should be SA

extern const uint16_t pcode_pgn[NUMBER_OF_PCODE];
extern const uint8_t pcode_failure_led[NUMBER_OF_PCODE];
extern const uint8_t pcode_fmi[NUMBER_OF_PCODE];
extern const char pcode[NUMBER_OF_PCODE][PCODE_LENGTH];
extern const char pcode_spn_in_string[NUMBER_OF_PCODE][SPN_LENGTH];
extern const uint32_t pcode_spn[NUMBER_OF_PCODE];

#ifdef __cplusplus
}
#endif

#endif /* __PCODE_H */