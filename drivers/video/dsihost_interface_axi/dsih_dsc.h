/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright (C) 2018 Synopsys, Inc.
 *
 * @file dsih_dsc.c
 * @brief Synopsys MIPI DSI VESA Display Stream Compression functions
 * included as a part of Synopsys MIPI DSI Host controller driver
 *
 * @author Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include "includes.h"

#ifdef VESASUPPORT

#define rc_model_size 8192
#define NUM_BUF_RANGES 15
#define NUM_COMPONENTS 3
#define MAX_UNITS_PER_GROUP 3
#define SAMPLES_PER_UNIT 3
#define PIXELS_PER_GROUP 3
#define GROUPS_PER_SUPERGROUP 4
#define BP_RANGE 10
#define BP_SIZE 3
#define PRED_BLK_SIZE 3
#define ICH_BITS 5
#define ICH_SIZE 1<<ICH_BITS)
#define ICH_PIXELS_ABOVE 7
#define OFFSET_FRACTIONAL_BITS 11

#define PPS_SIZE 128
#define BP_EDGE_COUNT 3
#define BP_EDGE_STRENGTH 32
#define PADDING_LEFT 5  // Pixels to pad line arrays to the left
#define PADDING_RIGHT 5 // Pixels to pad line arrays to the right
#define RC_SCALE_BINARY_POINT 3
#define OVERFLOW_AVOID_THRESHOLD 172)

/*  Implement ceiling, floor functions.  */
#define ceil(n, d) (((n) < 0) ? (-((-(n)) / (d))) : (n) / (d) + ((n) % (d) != 0))
#define floor(n, d) (((n) < 0) ? (-((-(n)) / (d))) - ((n) % (d) != 0) : (n) / (d))
#define RANGE_CHECK(s, a, b, c)                                                  \
    {                                                                            \
        if (((a) < (b)) || ((a) > (c)))                                          \
        {                                                                        \
            pr_err("%s out of range, needs to be between %d and %d\n", s, b, c); \
        }                                                                        \
        else                                                                     \
        {                                                                        \
            pr_debug("%s: %d\n", s, a);                                          \
        }                                                                        \
    }

void mipi_dsih_vesa_get_coreid(struct mipi_dsi_dev *dev);
uint32_t mipi_dsih_vesa_get_version(struct mipi_dsi_dev *dev);
int mipi_dsih_vesa_configure(struct mipi_dsi_dev *dev);

#endif /*VESASUPPORT*/
