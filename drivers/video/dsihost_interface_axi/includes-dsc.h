/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file includes.dsc.h
 * @brief Synopsys MIPI DSC Vesa library
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#ifndef MIPI_DSIH_VESA_H_
#define MIPI_DSIH_VESA_H_

/* DSC encoder COREID.*/
#define DSC_ENC_COREID 0x200

/* Vesa DSC Version. */
#define DSC_ENC_VERSION 0x204

/* DSC encoder flatness determination.*/
#define DSC_ENC_FLATNESS_DET_THRES 0x208

/* This register is use to delay DPI events in order to be
 * compliant with DSC encoder latency.*/
#define DSC_ENC_DELAY 0x20c

/* Compressed line size in units of 1/16th of a bit.*/
#define DSC_ENC_COMPRESSED_LINE_SIZE 0x210

/* Calculation of the number of lines in excess.*/
#define DSC_ENC_LINES_IN_EXCESS 0x214

/* Adjustment needed to calculate end of last line of a slice.*/
#define DSC_ENC_RBUF_ADDR_LAST_LINE_ADJ	0x218

/* This register is to enable DSC encoder.*/
#define DSC_MODE 0x21c

/* This register contains the status of interrupt sources from
 * DSC encoder.*/
#define DSC_ENC_INT_ST 0x220

/* This register configures masks for the sources of interrupts
 * that affect the INT_ST_DSC register....*/
#define DSC_ENC_INT_MSK 0x224

/* This register forces interrupts that affect the INT_ST_DSC register.*/
#define DSC_ENC_INT_FORCE 0x228

/* This register holds the bytes in the
 * range [0,3] of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS0_3 0x260

/* This register holds the bytes in the
 * range [4,7] of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS4_7 0x264

/* This register holds the bytes in the range [8,11]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS8_11 0x268

/* This register holds the bytes in the range [12,15]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_12_15 0x26c

/* This register holds the bytes in the range [16,19]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_16_19 0x270

/* This register holds the bytes in the range [20,23]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_20_23 0x274

/* This register holds the bytes in the range [24,27]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_24_27 0x278

/* This register holds the bytes in the  range [28,31]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_28_31 0x27c

/* This register holds the bytes in the range [32,35]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_32_35 0x280

/* This register holds the bytes in the range [36,39]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_36_39 0x284

/* This register holds the bytes in the range [40,43]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_40_43 0x288

/* This register holds the bytes in the range [44,47]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_44_47 0x28c

/* This register holds the bytes in the range [48,51]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_48_51 0x290

/* This register holds the bytes in the range [52,55]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_52_55 0x294

/* This register holds the bytes in the range [56,59]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_56_59 0x298

/* This register holds the bytes in the range [60,63]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_60_63 0x29c

/* This register holds the bytes in the range [64,67]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_64_67 0x2a0

/* This register holds the bytes in the range [68,71]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_68_71 0x2a4

/* This register holds the bytes in the range [72,75]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_72_75 0x2a8

/* This register holds the bytes in the range [76,79]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_76_79 0x2ac

/* This register holds the bytes in the range [80,83]
 * of the Picture Parameter Set (PPS).*/
#define DSC_ENC_PPS_80_83 0x2b0

/* This register holds the bytes in the range [84,87]
 * of the Picture Parameter Set (PPS). */
#define DSC_ENC_PPS_84_87 0x2b4

/* This register holds the bytes in the  range [0,3]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS0_3_T1 0x2e0

/* This register holds the bytes in the range [4,7]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS4_7_T1 0x2e4

/* This register holds the bytes in the range [8,11]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS8_11_T1 0x2e8

/* This register holds the bytes in the range [12,15]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_12_15_T1 0x2ec

/* This register holds the bytes in the range [16,19]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_16_19_T1 0x2f0

/* This register holds the bytes in the range [20,23]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_20_23_T1 0x2f4

/* This register holds the bytes in the range [24,27]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_24_27_T1 0x2f8

/* This register holds the bytes in the range [28,31]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_28_31_T1 0x2fc

/* This register holds the bytes in the range [32,35]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_32_35_T1 0x300

/* This register holds the bytes in the range [36,39]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_36_39_T1 0x304

/* This register holds the bytes in the range [40,43]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_40_43_T1 0x308

/* This register holds the bytes in the range [44,47]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_44_47_T1 0x30c

/* This register holds the bytes in the range [48,51]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_48_51_T1 0x310

/* This register holds the bytes in the range [52,55]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_52_55_T1 0x314

/* This register holds the bytes in the range [56,59]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_56_59_T1 0x318

/* This register holds the bytes in the range [60,63]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_60_63_T1 0x31c

/* This register holds the bytes in the range [64,67]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_64_67_T1 0x320

/* This register holds the bytes in the range [68,71]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_68_71_T1 0x324

/* This register holds the bytes in the range [72,75]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_72_75_T1 0x328

/* This register holds the bytes in the range [76,79]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_76_79_T1 0x32c

/* This register holds the bytes in the range [80,83]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_80_83_T1 0x330

/* This register holds the bytes in the range [84,87]
 * of the Picture Parameter Set (PPS) - Table 1.*/
#define DSC_ENC_PPS_84_87_T1 0x334

/* This register holds the bytes in the range [0,3]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS0_3_T2 0x360

/* This register holds the bytes in the range [4,7]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS4_7_T2 0x364

/* This register holds the bytes in the range [8,11]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS8_11_T2 0x368

/* This register holds the bytes in the range [12,15]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_12_15_T2 0x36c

/* This register holds the bytes in the range [16,19]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_16_19_T2 0x370

/* This register holds the bytes in the range [20,23]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_20_23_T2 0x374

/* This register holds the bytes in the range [24,27]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_24_27_T2 0x378

/* This register holds the bytes in the range [28,31]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_28_31_T2 0x37c

/* This register holds the bytes in the range [32,35]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_32_35_T2 0x380

/* This register holds the bytes in the range [36,39]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_36_39_T2 0x384

/* This register holds the bytes in the range [40,43]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_40_43_T2 0x388

/* This register holds the bytes in the range [44,47]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_44_47_T2 0x38c

/* This register holds the bytes in the range [48,51]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_48_51_T2 0x390

/* This register holds the bytes in the range [52,55]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_52_55_T2 0x394

/* This register holds the bytes in the range [56,59]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_56_59_T2 0x398

/* This register holds the bytes in the range [60,63]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_60_63_T2 0x39c

/* This register holds the bytes in the range [64,67]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_64_67_T2 0x3a0

/* This register holds the bytes in the range [68,71]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_68_71_T2 0x3a4

/* This register holds the bytes in the range [72,75]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_72_75_T2 0x3a8

/* This register holds the bytes in the range [76,79]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_76_79_T2 0x3ac

/* This register holds the bytes in the range [80,83]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_80_83_T2 0x3b0

/* This register holds the bytes in the range [84,87]
 * of the Picture Parameter Set (PPS) - Table 2.*/
#define DSC_ENC_PPS_84_87_T2 0x3b4

/* This register holds the bytes in the range [0,3]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS0_3_T3 0x3e0

/* This register holds the bytes in the range [4,7]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS4_7_T3 0x3e4

/* This register holds the bytes in the range [8,11]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS8_11_T3 0x3e8

/* This register holds the bytes in the range [12,15]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_12_15_T3 0x3ec

/* This register holds the bytes in the range [16,19]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_16_19_T3 0x3f0

/* This register holds the bytes in the range [20,23]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_20_23_T3 0x3f4

/* This register holds the bytes in the range [24,27]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_24_27_T3 0x3f8

/* This register holds the bytes in the range [28,31]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_28_31_T3 0x3fc

/* This register holds the bytes in the range [32,35]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_32_35_T3 0x400

/* This register holds the bytes in the range [36,39]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_36_39_T3 0x404

/* This register holds the bytes in the range [40,43]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_40_43_T3 0x408

/* This register holds the bytes in the range [44,47]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_44_47_T3 0x40c

/* This register holds the bytes in the range [48,51]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_48_51_T3 0x410

/* This register holds the bytes in the range [52,55]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_52_55_T3 0x414

/* This register holds the bytes in the range [56,59]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_56_59_T3 0x418

/* This register holds the bytes in the range [60,63]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_60_63_T3 0x41c

/* This register holds the bytes in the range [64,67]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_64_67_T3 0x420

/* This register holds the bytes in the range [68,71]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_68_71_T3 0x424

/* This register holds the bytes in the range [72,75]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_72_75_T3 0x428

/* This register holds the bytes in the range [76,79]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_76_79_T3 0x42c

/* This register holds the bytes in the range [80,83]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_80_83_T3 0x430

/* This register holds the bytes in the range [84,87]
 * of the Picture Parameter Set (PPS) - Table 3.*/
#define DSC_ENC_PPS_84_87_T3 0x434

struct rc_rate_control {
	signed int MinQp;
	signed int MaxQp;
	signed int Ofs;
};
struct rc_rate_control rc_range_parameters_8_8[] = {
	{ 0, 4,  2},
	{ 0, 4,  0},
	{ 1, 5,  0},
	{ 1, 6, -2},
	{ 3, 7, -4},
	{ 3, 7, -6},
	{ 3, 7, -8},
	{ 3, 8, -8},
	{ 3, 9, -8},
	{ 3,10,-10},
	{ 5,11,-10},
	{ 5,12,-12},
	{ 5,13,-12},
	{ 7,13,-12},
	{13,15,-12}
};

struct rc_rate_control rc_range_parameters_8_10[] = {
	{ 0, 4, 2},
	{ 4, 8, 0},
	{ 5, 9, 0},
	{ 5,10,-2},
	{ 7,11,-4},
	{ 7,11,-6},
	{ 7,11,-8},
	{ 7,12,-8},
	{ 7,13,-8},
	{ 7,14,-10},
	{ 9,15,-10},
	{ 9,16,-12},
	{ 9,17,-12},
	{11,17,-12},
	{17,19,-12}
};

struct rc_rate_control rc_range_parameters_8_12[] = {
	{ 0,12, 2},
	{ 0,12, 0},
	{ 9,13, 0},
	{ 9,14,-2},
	{11,15,-4},
	{11,15,-6},
	{11,15,-8},
	{11,16,-8},
	{11,17,-8},
	{11,18,-10},
	{13,19,-10},
	{13,20,-12},
	{13,21,-12},
	{15,21,-12},
	{21,23,-12}
};

struct rc_rate_control rc_range_parameters_12_8[] = {
	{ 0, 2,  2},
	{ 0, 4,  0},
	{ 1, 5,  0},
	{ 1, 6, -2},
	{ 3, 7, -4},
	{ 3, 7, -6},
	{ 3, 7, -8},
	{ 3, 8, -8},
	{ 3, 9, -8},
	{ 5,10,-10},
	{ 5,11,-10},
	{ 5,12,-12},
	{ 5,13,-12},
	{ 7,13,-12},
	{13,15,-12}
};

struct rc_rate_control rc_range_parameters_12_10[] = {
	{ 0, 2,  2},
	{ 2, 5,  0},
	{ 3, 7,  0},
	{ 4, 8, -2},
	{ 6, 9, -4},
	{ 7,10, -6},
	{ 7,11, -8},
	{ 7,12, -8},
	{ 7,13, -8},
	{ 7,14,-10},
	{ 9,15,-10},
	{ 9,16,-12},
	{ 9,17,-12},
	{11,17,-12},
	{17,19,-12}
};

struct rc_rate_control rc_range_parameters_12_12[] = {
	{ 0, 6,  2},
	{ 4, 9,  0},
	{ 7,11,  0},
	{ 8,12, -2},
	{10,13, -4},
	{11,14, -6},
	{11,15, -8},
	{11,16, -8},
	{11,17, -8},
	{11,18,-10},
	{13,19,-10},
	{13,20,-12},
	{13,21,-12},
	{15,21,-12},
	{21,25,-12}
};

#endif /*MIPI_DSIH_VESA_H_*/
