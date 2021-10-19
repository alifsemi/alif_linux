// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_dsc.c
 * @brief Synopsys MIPI DSI VESA functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "includes-dsc.h"
#include "dsih_api.h"
#include "dsih_hal.h"
#include "dsih_dphy.h"
#include "dsih_dsc.h"
#include "dsih_core.h"

#ifdef VESASUPPORT

void mipi_dsih_vesa_get_coreid(struct mipi_dsi_dev *dev)
{
	uint32_t ret;
	uint16_t design_id;
	uint16_t product_id;

	ret = mipi_dsih_read_word(dev, DSC_ENC_COREID);
	//pr_info("read id: %X\n", ret);
	product_id = (ret >> 16);
	design_id = (ret & 0xFFFF);
	pr_info("***** VESA DSC ******\n");
	pr_info("VESA DSC: product id: %X\n", product_id);
	pr_info("VESA DSC: design id: %X\n", design_id);
}

uint32_t mipi_dsih_vesa_get_version(struct mipi_dsi_dev *dev)
{
	return mipi_dsih_read_word(dev, DSC_ENC_VERSION);
}

int mipi_dsih_vesa_configure(struct mipi_dsi_dev *dev)
{
	int DSC_BITS_PER_COMPONENT;
	int i;
	uint32_t DPITHSA;
	uint32_t DPITHBP;
	uint32_t DPITHACT;
	uint32_t DPITHFP;
	uint32_t DPIVACTLINES;
	uint32_t DSC_LINEBUF_DEPTH;
	uint32_t DSC_SLICE_HEIGHT;
	uint32_t DSC_SLIDE_WIDTH;
	uint32_t bpp;
	uint32_t HRD_DELAY;
	uint32_t lines_in_excess;
	uint32_t DSC_INITIAL_XMIT_DELAY;
	uint32_t DSC_INITIAL_DEC_DELAY;
	uint32_t DSC_CHUNK_SIZE;
	uint32_t compressed_line_size;
	uint32_t compressed_line_size_ceil8;
	uint32_t compressed_line_size_ceil64;
	uint32_t excess_frac_bits_per_line;
	uint32_t excess_frac_bits_per_slice;
	uint32_t initial_offset;
	uint32_t final_offset;
	uint32_t final_value;
	uint32_t nfl_bpg_offset;
	uint32_t slice_bpg_offset;
	uint32_t MAX_SE_SIZE;
	uint32_t SOMEWHAT_FLAT_QP_THRESH;
	uint32_t initial_scale_value;
	uint32_t final_scale_value;
	uint32_t groupsPerLine;
	uint32_t groupsTotal;
	uint32_t first_line_bpg_offset;
	uint32_t min_rate_buffer_size;
	uint32_t scale_decrement_interval;
	uint32_t scale_increment_interval;
	uint32_t muxWordSize;
	uint32_t numExtraMuxBits;
	uint32_t rb_flatness_min_qp_r;
	uint32_t rb_flatness_max_qp_r;
	uint32_t rc_edge_factor_r;
	uint32_t rc_tgt_offset_hi_r;
	uint32_t rc_tgt_offset_lo_r;
	uint32_t rc_quant_incr_limit0_r;
	uint32_t rc_quant_incr_limit1_r;
	uint32_t temp;

	int rc_buf_thresh_r[] = {896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};

	dsih_dpi_video_t *video = &dev->dpi_video;

	struct rc_rate_control *rc_range_parameters;

	switch (video->color_coding)
	{
	/* DSC_BITS_PER_COMPONENT : 8, 10, 12 */
	case 5:
		DSC_BITS_PER_COMPONENT = 8;
		initial_offset = 6144;
		first_line_bpg_offset = 12;
		muxWordSize = 48;
		break;
	case 9:
		DSC_BITS_PER_COMPONENT = 10;
		initial_offset = 4096;
		first_line_bpg_offset = 15;
		muxWordSize = 48;
	default:
		initial_offset = 2048;
		muxWordSize = 64;
		pr_info("Invalid color configuration for DSC!\n");
		return 0;
	}
	bpp = 8; /*DSC_BITS_PER_PIXEL*/

	RANGE_CHECK("rc_model_size", rc_model_size, 0, 65535);
	RANGE_CHECK("initial_offset", initial_offset, 0, rc_model_size);
	RANGE_CHECK("first_line_bpg_offset", first_line_bpg_offset, 0, 31);

	DSC_LINEBUF_DEPTH = DSC_BITS_PER_COMPONENT + 1;
	DSC_SLICE_HEIGHT = video->v_active_lines;
	DSC_SLIDE_WIDTH = video->h_active_pixels / 2;

	RANGE_CHECK("linebuf_depth", DSC_LINEBUF_DEPTH, 8, 13);
	RANGE_CHECK("pic_width", video->h_active_pixels, 0, 65535);
	RANGE_CHECK("pic_height", video->v_active_lines, 0, 65535);

	nfl_bpg_offset = ceil(first_line_bpg_offset, DSC_SLICE_HEIGHT - 1);
	nfl_bpg_offset = 103;
	groupsPerLine = ceil(DSC_SLIDE_WIDTH + 2, 3);

	mipi_dsih_write_word(dev, DSC_ENC_FLATNESS_DET_THRES,
						 2 ^ (DSC_BITS_PER_COMPONENT - 7));

	DPITHSA = video->h_sync_pixels;
	DPITHBP = video->h_back_porch_pixels;
	DPITHACT = video->h_active_pixels;
	DPITHFP = video->h_total_pixels - (video->h_back_porch_pixels + video->h_sync_pixels + video->h_active_pixels);
	DPIVACTLINES = video->v_active_lines;

	mipi_dsih_write_word(dev, DSC_ENC_DELAY, 3087);
	compressed_line_size = 16 * bpp * DSC_SLIDE_WIDTH;
	mipi_dsih_write_word(dev, DSC_ENC_COMPRESSED_LINE_SIZE,
						 16 * bpp * DSC_SLIDE_WIDTH);

	/* number of additional lines that need to be output, due to extra
	 * bits sent in dsc_data[63:0] at the end of each line.
	 * Value of adjustment, determined iteratively, that needs to be applied
	 * to the calculation of the end address of last line of a slice in the
	 * RAM of rate buffer. */
	MAX_SE_SIZE = (4 * DSC_BITS_PER_COMPONENT + 4);
	SOMEWHAT_FLAT_QP_THRESH = (7 + (2 * (DSC_BITS_PER_COMPONENT - 8)));

	compressed_line_size_ceil8 = ceil(compressed_line_size, 128) * 128;
	compressed_line_size_ceil64 = ceil(compressed_line_size, 1024) * 1024;

	excess_frac_bits_per_line = compressed_line_size_ceil64 - compressed_line_size_ceil8;
	excess_frac_bits_per_slice = excess_frac_bits_per_line * DSC_SLICE_HEIGHT;

	lines_in_excess = floor(excess_frac_bits_per_slice, compressed_line_size_ceil64);

	mipi_dsih_write_word(dev, DSC_ENC_LINES_IN_EXCESS, lines_in_excess);
	mipi_dsih_write_word(dev, DSC_ENC_RBUF_ADDR_LAST_LINE_ADJ,
						 excess_frac_bits_per_slice % compressed_line_size_ceil64);
	mipi_dsih_write_word(dev, DSC_MODE, 1);
	pr_debug("VESA: groupsPerLine: %d\n", groupsPerLine);
	mipi_dsih_write_word(dev, DSC_ENC_PPS0_3,
						 (0x11000000 + (DSC_BITS_PER_COMPONENT << 4) + DSC_LINEBUF_DEPTH));

	if (DSC_BITS_PER_COMPONENT == 8)
		mipi_dsih_write_word(dev, DSC_ENC_PPS4_7,
							 (1 << 29) + (1 << 28) + ((16 * DSC_BITS_PER_COMPONENT) << 16) + DPIVACTLINES);

	mipi_dsih_write_word(dev, DSC_ENC_PPS8_11,
						 (DPITHACT << 16) + DSC_SLICE_HEIGHT);

	DSC_CHUNK_SIZE = ceil(bpp * DSC_SLIDE_WIDTH, 8);

	pr_debug("VESA: DSC_CHUNK_SIZE: %d\n", DSC_CHUNK_SIZE);
	RANGE_CHECK("DSC_CHUNK_SIZE", DSC_CHUNK_SIZE, 0, 65535);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_12_15,
						 (DSC_SLIDE_WIDTH << 16) + (DSC_CHUNK_SIZE));

	DSC_INITIAL_XMIT_DELAY = (rc_model_size / 2 / bpp);

	pr_debug("VESA: DSC_INITIAL_XMIT_DELAY: %d\n", DSC_INITIAL_XMIT_DELAY);
	RANGE_CHECK("initial_xmit_delay", DSC_INITIAL_XMIT_DELAY, 0, 1023);

	min_rate_buffer_size =
		(rc_model_size - initial_offset + ceil(DSC_INITIAL_XMIT_DELAY * bpp, 1) + groupsPerLine * first_line_bpg_offset);
	pr_debug("VESA: min_rate_buffer_size: %d\n", min_rate_buffer_size);

	HRD_DELAY = ceil(min_rate_buffer_size, bpp);
	pr_debug("VESA: HRD_DELAY: %d\n", HRD_DELAY);

	DSC_INITIAL_DEC_DELAY = HRD_DELAY - DSC_INITIAL_XMIT_DELAY;

	pr_debug("VESA: DSC_INITIAL_DEC_DELAY: %d\n", DSC_INITIAL_DEC_DELAY);
	RANGE_CHECK("initial_dec_delay", DSC_INITIAL_DEC_DELAY, 0, 65535);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_16_19,
						 (DSC_INITIAL_XMIT_DELAY << 16) + DSC_INITIAL_DEC_DELAY);

	if (rc_model_size <= initial_offset)

		pr_err("INITIAL_OFFSET must be less than RC_MODEL_SIZE\n");
	initial_scale_value = 8 * rc_model_size / (rc_model_size - initial_offset);

	pr_debug("VESA: initial_scale_value: %d\n", initial_scale_value);
	RANGE_CHECK("initial_scale_value", initial_scale_value, 0, 63);

	if (groupsPerLine < initial_scale_value - 8)
		initial_scale_value = groupsPerLine + 8;
	initial_scale_value = 32;
	if (initial_scale_value > 8)
		scale_decrement_interval = groupsPerLine / (initial_scale_value - 8);
	else
		scale_decrement_interval = 4095;

	pr_debug("VESA: scale_decrement_interval: %d\n", scale_decrement_interval);
	RANGE_CHECK("scale_decrement_interval", scale_decrement_interval, 0, 4095);

	numExtraMuxBits = 3 * (muxWordSize + (4 * DSC_BITS_PER_COMPONENT + 4) - 2);

	final_value = (uint32_t)rc_model_size - ((DSC_INITIAL_XMIT_DELAY * bpp * 16 + 8) >> 4) + numExtraMuxBits;
	final_offset = final_value;

	RANGE_CHECK("final_offset", final_offset, 0, 65535);
	if (final_value >= rc_model_size)
		pr_err("The final_offset must be less than the rc_model_size.  Try increasing initial_xmit_delay.\n");

	final_scale_value = 8 * rc_model_size / (rc_model_size - final_value);
	pr_debug("VESA: final_scale_value: %d\n", final_scale_value);
	if (final_scale_value > 63)
		pr_debug("WARNING: A final scale value > than 63/8 may have undefined behavior on some implementations.  Try increasing initial_xmit_delay.\n");

	if (DSC_SLICE_HEIGHT > 1)
		nfl_bpg_offset = (int)ceil((first_line_bpg_offset << OFFSET_FRACTIONAL_BITS), DSC_SLICE_HEIGHT - 1);
	else
		nfl_bpg_offset = 0;
	RANGE_CHECK("nfl_bpg_offset", nfl_bpg_offset, 0, 65535);

	groupsTotal = groupsPerLine * DSC_SLICE_HEIGHT;
	pr_debug("VESA: groupsTotal: %d\n", groupsTotal);

	slice_bpg_offset = (int)ceil((1 << OFFSET_FRACTIONAL_BITS) * (rc_model_size - initial_offset + numExtraMuxBits), groupsTotal);

	RANGE_CHECK("slice_bpg_offset", slice_bpg_offset, 0, 65535);

	if (DSC_SLICE_HEIGHT == 1)
	{
		if (first_line_bpg_offset > 0)
			pr_err("For slice_height == 1, the FIRST_LINE_BPG_OFFSET must be 0\n");
	}
	first_line_bpg_offset = 12;
	if (final_scale_value > 9)
	{
		scale_increment_interval = (uint32_t)(double)((1 << OFFSET_FRACTIONAL_BITS) * final_offset / ((double)(final_scale_value - 9) * (nfl_bpg_offset + slice_bpg_offset)));
		pr_debug("VESA: scale_increment_interval: %d\n", scale_increment_interval);
		if (scale_increment_interval > 65535)
			pr_err("ERROR: required scale increment interval is too high.  Consider using smaller slices or increase initial delay\n");
	}
	else
		scale_increment_interval = 0;

	mipi_dsih_write_word(dev, DSC_ENC_PPS_20_23,
						 (initial_scale_value << 16) + scale_increment_interval);
	mipi_dsih_write_word(dev, DSC_ENC_PPS_24_27,
						 (scale_decrement_interval << 16) + first_line_bpg_offset);
	mipi_dsih_write_word(dev, DSC_ENC_PPS_28_31,
						 (nfl_bpg_offset << 16) + slice_bpg_offset);
	mipi_dsih_write_word(dev, DSC_ENC_PPS_32_35,
						 (initial_offset << 16) + final_offset);

	if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 8))
		rb_flatness_min_qp_r = 3;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 10))
		rb_flatness_min_qp_r = 7;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 12))
		rb_flatness_min_qp_r = 11;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 8))
		rb_flatness_min_qp_r = 3;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 10))
		rb_flatness_min_qp_r = 7;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 12))
		rb_flatness_min_qp_r = 11;

	if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 8))
		rb_flatness_max_qp_r = 12;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 10))
		rb_flatness_max_qp_r = 16;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 12))
		rb_flatness_max_qp_r = 20;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 8))
		rb_flatness_max_qp_r = 12;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 10))
		rb_flatness_max_qp_r = 16;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 12))
		rb_flatness_max_qp_r = 20;

	mipi_dsih_write_word(dev, DSC_ENC_PPS_36_39, (rb_flatness_min_qp_r << 24) + (rb_flatness_max_qp_r << 16) + rc_model_size);

	if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 8))
		rc_quant_incr_limit0_r = 11;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 10))
		rc_quant_incr_limit0_r = 15;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 12))
		rc_quant_incr_limit0_r = 19;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 8))
		rc_quant_incr_limit0_r = 11;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 10))
		rc_quant_incr_limit0_r = 15;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 12))
		rc_quant_incr_limit0_r = 19;

	if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 8))
		rc_quant_incr_limit1_r = 11;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 10))
		rc_quant_incr_limit1_r = 15;
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 12))
		rc_quant_incr_limit1_r = 19;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 8))
		rc_quant_incr_limit1_r = 11;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 10))
		rc_quant_incr_limit1_r = 15;
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 12))
		rc_quant_incr_limit1_r = 19;

	rc_edge_factor_r = 6;
	rc_tgt_offset_hi_r = 3;
	rc_tgt_offset_lo_r = 3;

	mipi_dsih_write_word(dev, DSC_ENC_PPS_40_43, (rc_edge_factor_r << 24) | (rc_quant_incr_limit0_r << 16) | (rc_quant_incr_limit1_r << 8) | (rc_tgt_offset_hi_r << 4) | rc_tgt_offset_lo_r);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_44_47,
						 ((rc_buf_thresh_r[0] >> 6) << 24) |
							 ((rc_buf_thresh_r[1] >> 6) << 16) | ((rc_buf_thresh_r[2] >> 6) << 8) | rc_buf_thresh_r[3] >> 6);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_48_51, ((rc_buf_thresh_r[4] >> 6) << 24) | ((rc_buf_thresh_r[5] >> 6) << 16) | ((rc_buf_thresh_r[6] >> 6) << 8) | (rc_buf_thresh_r[7] >> 6));

	mipi_dsih_write_word(dev, DSC_ENC_PPS_52_55, ((rc_buf_thresh_r[8] >> 6) << 24) | ((rc_buf_thresh_r[9] >> 6) << 16) | ((rc_buf_thresh_r[10] >> 6) << 8) | rc_buf_thresh_r[11] >> 6);

	if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 8))
	{
		rc_range_parameters = rc_range_parameters_8_8;
		pr_debug("VESA: rc_range_parameters_8_8\n");
	}
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 10))
	{
		*rc_range_parameters = *rc_range_parameters_8_10;
		pr_debug("VESA: rc_range_parameters_8_10\n");
	}
	else if ((bpp == 8) && (DSC_BITS_PER_COMPONENT == 12))
	{
		*rc_range_parameters = *rc_range_parameters_8_12;
		pr_debug("VESA: rc_range_parameters_8_12\n");
	}
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 8))
	{
		*rc_range_parameters = *rc_range_parameters_12_8;
		pr_debug("VESA: rc_range_parameters_12_8\n");
	}
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 10))
	{
		*rc_range_parameters = *rc_range_parameters_12_10;
		pr_debug("VESA: rc_range_parameters_12_10\n");
	}
	else if ((bpp == 12) && (DSC_BITS_PER_COMPONENT == 12))
	{
		*rc_range_parameters = *rc_range_parameters_12_12;
		pr_debug("VESA: rc_range_parameters_12_12\n");
	}

	mipi_dsih_write_word(dev, DSC_ENC_PPS_56_59, ((rc_buf_thresh_r[12] >> 6) << 24) | ((rc_buf_thresh_r[13] >> 6) << 16) | (rc_range_parameters[0].MinQp << 11) | (rc_range_parameters[0].MaxQp << 6) | rc_range_parameters[0].Ofs);

	for (i = 1; i <= 14; i++)
	{
		pr_debug("VESA: rc_buf_thresh_r[%d]: %d\n", i, rc_buf_thresh_r[i] >> 6);

		pr_debug("rc_range__%d_min_qp_r: %d\nrc_range__%d_max_qp_r: %d\nrc_range__%d_bpg_offset_r: %d\n", i, rc_range_parameters[i].MinQp, i, rc_range_parameters[i].MaxQp, i, rc_range_parameters[i].Ofs);

		if (rc_range_parameters[i].Ofs < 0)
			rc_range_parameters[i].Ofs = rc_range_parameters[i].Ofs + 64;

		pr_debug("2s complement_rc_range__%d_bpg_offset_r: %d\n", i,
				 rc_range_parameters[i].Ofs);
	}

	mipi_dsih_write_word(dev, DSC_ENC_PPS_60_63, (rc_range_parameters[1].MinQp << 27) | (rc_range_parameters[1].MaxQp << 22) | (rc_range_parameters[1].Ofs << 16) | (rc_range_parameters[2].MinQp << 11) | (rc_range_parameters[2].MaxQp << 6) | rc_range_parameters[2].Ofs);

	temp = mipi_dsih_read_word(dev, DSC_ENC_PPS_60_63);

	pr_debug("temp: 0x%X\n", temp);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_64_67, (rc_range_parameters[3].MinQp << 27) | (rc_range_parameters[3].MaxQp << 22) | (rc_range_parameters[3].Ofs << 16) | (rc_range_parameters[4].MinQp << 11) | (rc_range_parameters[4].MaxQp << 6) | rc_range_parameters[4].Ofs);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_68_71, (rc_range_parameters[5].MinQp << 27) | (rc_range_parameters[5].MaxQp << 22) | (rc_range_parameters[5].Ofs << 16) | (rc_range_parameters[6].MinQp << 11) | (rc_range_parameters[6].MaxQp << 6) | rc_range_parameters[6].Ofs);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_72_75, (rc_range_parameters[7].MinQp << 27) | (rc_range_parameters[7].MaxQp << 22) | (rc_range_parameters[7].Ofs << 16) | (rc_range_parameters[8].MinQp << 11) | (rc_range_parameters[8].MaxQp << 6) | rc_range_parameters[8].Ofs);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_76_79, (rc_range_parameters[9].MinQp << 27) | (rc_range_parameters[9].MaxQp << 22) | (rc_range_parameters[9].Ofs << 16) | (rc_range_parameters[10].MinQp << 11) | (rc_range_parameters[10].MaxQp << 6) | rc_range_parameters[10].Ofs);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_80_83, (rc_range_parameters[11].MinQp << 27) | (rc_range_parameters[11].MaxQp << 22) | (rc_range_parameters[11].Ofs << 16) | (rc_range_parameters[12].MinQp << 11) | (rc_range_parameters[12].MaxQp << 6) | rc_range_parameters[12].Ofs);

	mipi_dsih_write_word(dev, DSC_ENC_PPS_84_87, (rc_range_parameters[14].MinQp << 27) | (rc_range_parameters[14].MaxQp << 22) | (rc_range_parameters[14].Ofs << 16) | (rc_range_parameters[13].MinQp << 11) | (rc_range_parameters[13].MaxQp << 6) | rc_range_parameters[13].Ofs);
	return 1;
}
#endif
