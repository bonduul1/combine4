/******************************
	Sequence Data
	Filename	: sequenceData.c
	Format Ver	: 1.4.0.60
	Date		: 2025-05-29 오후 4:00:38 +09:00
*******************************/

#include "Yvc1.h"

#define GENTBL_START_ADDR	(0x0000)
#define GENTBL_END_ADDR		(0x0000)

#define PLTTBL_START_ADDR	(0x0300)
#define PLTTBL_END_ADDR		(0x0000)

/*----------------------------------------
 *	 Sequence Information Data
 *----------------------------------------*/

enum SequenceName {
	QN__CPU_WARNING_1_1 = 0,
	QN__CPU_CHECK_MODE_1,
	QN__CPU_WARNING_2_1,
	QN__CPU_WARNING_3_1,
	QN__CPU_UNITS_1,
	QN__CPU_BTN_PAGE_1,
	QN__CPU_MAIN_1,
	QN__CPU_MAIN_RPM_1,
	QN__CPU_MODE000_1,
	QN__CPU_MODE1xx_1,
	QN__CPU_MODE1111_1,
	QN__CPU_MODE1112_1,
	QN__CPU_MODE1113_1,
	QN__CPU_MODE1114_1,
	QN__CPU_MODE1115_1,
	QN__CPU_MODE1116_1,
	QN__CPU_MODE2xx_1,
	QN__CPU_MODE3xx_1,
	QN__CPU_MODE31x_1,
	QN__CPU_MODE311_1,
	QN__CPU_MODE312_1,
	QN__CPU_MODE313_1,
	QN__CPU_MODE314_1,
	QN__CPU_MODE315_1,
	QN__CPU_MODE32x_1,
	QN__CPU_MODE33x_1,
	QN__CPU_MODE34x_1,
	QN__CPU_MODE35x_1,
	QN__CPU_MODE36x_1,
	QN__CPU_MODE4xx_1,
	QN__CPU_MODE5xx_1,
	QN__CPU_MODE6xx_1
};

static const T_YSEQ_INFO tYSeqInfo[32] = {
	/*   SceneAddr,           Action, YSeqButtonNum,        tYSeqButton,    NextSeqID */
	{ 0x00421230UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 0: #1: <CPU>WARNING_1: #1 */
	{ 0x00421244UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 1: #2: <CPU>CHECK_MODE: #1 */
	{ 0x00421248UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 2: #3: <CPU>WARNING_2: #1 */
	{ 0x0042124CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 3: #4: <CPU>WARNING_3: #1 */
	{ 0x00421250UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 4: #5: <CPU>UNITS: #1 */
	{ 0x00421254UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 5: #6: <CPU>BTN_PAGE: #1 */
	{ 0x00421258UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 6: #7: <CPU>MAIN: #1 */
	{ 0x0042125CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 7: #8: <CPU>MAIN_RPM: #1 */
	{ 0x00421260UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 8: #9: <CPU>MODE000: #1 */
	{ 0x00421264UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 9: #10: <CPU>MODE1xx: #1 */
	{ 0x00421268UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 10: #11: <CPU>MODE1111: #1 */
	{ 0x0042126CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 11: #12: <CPU>MODE1112: #1 */
	{ 0x00421270UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 12: #13: <CPU>MODE1113: #1 */
	{ 0x00421274UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 13: #14: <CPU>MODE1114: #1 */
	{ 0x00421278UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 14: #15: <CPU>MODE1115: #1 */
	{ 0x0042127CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 15: #17: <CPU>MODE1116: #1 */
	{ 0x00421280UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 16: #18: <CPU>MODE2xx: #1 */
	{ 0x00421284UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 17: #19: <CPU>MODE3xx: #1 */
	{ 0x00421288UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 18: #20: <CPU>MODE31x: #1 */
	{ 0x0042128CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 19: #21: <CPU>MODE311: #1 */
	{ 0x00421290UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 20: #22: <CPU>MODE312: #1 */
	{ 0x00421294UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 21: #23: <CPU>MODE313: #1 */
	{ 0x00421298UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 22: #24: <CPU>MODE314: #1 */
	{ 0x0042129CUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 23: #25: <CPU>MODE315: #1 */
	{ 0x004212A0UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 24: #26: <CPU>MODE32x: #1 */
	{ 0x004212A4UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 25: #27: <CPU>MODE33x: #1 */
	{ 0x004212A8UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 26: #28: <CPU>MODE34x: #1 */
	{ 0x004212ACUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 27: #29: <CPU>MODE35x: #1 */
	{ 0x004212B0UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 28: #30: <CPU>MODE36x: #1 */
	{ 0x004212B4UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 29: #31: <CPU>MODE4xx: #1 */
	{ 0x004212B8UL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID },	/* 30: #32: <CPU>MODE5xx: #1 */
	{ 0x004212BCUL, YSEQ_ACTION_NONE,             0, NULL              , YSEQ_INVALID }		/* 31: #33: <CPU>MODE6xx: #1 */
};

/*----------------------------------------
 *	 Sequence Data
 *----------------------------------------*/
const T_YSEQ_DATA tYSeqData = {
	/*  tYSeqInfo, YSeqInfoNum */
	&tYSeqInfo[0], 32
};
