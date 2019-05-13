/**
 * Copyright (c) 2016 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _H264_PRIV_H_
#define _H264_PRIV_H_

#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ULOG_TAG h264
#include <ulog.h>

#include <h264/h264.h>

#include "h264_macroblock.h"
#include "h264_slice_data.h"

#include "h264_bac.h"
#include "h264_cabac.h"


/* 5.7 Mathematical functions */
#define Abs(x) ((x) >= 0 ? (x) : -(x))
#define Min(x, y) ((x) < (y) ? (x) : (y))
#define Max(x, y) ((x) > (y) ? (x) : (y))
#define Clip3(x, y, z) ((z) < (x) ? (x) : (z) > (y) ? (y) : (z))


#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


struct h264_ctx {
	struct {
		enum h264_nalu_type type;
		struct h264_nalu_header hdr;
		int unknown;
		int is_first_vcl;
		int is_prev_vcl;
	} nalu;

	struct h264_aud aud;

	struct h264_sps *sps;
	struct h264_pps *pps;

	struct h264_sps *sps_table[32];
	struct h264_pps *pps_table[256];

	struct h264_sei *sei_table;
	uint32_t sei_count;

	struct {
		enum h264_slice_type type;
		struct h264_slice_header hdr;
		size_t hdr_len;
		struct h264_slice_header saved_hdr;
		struct {
			uint8_t partial;
			uint8_t partialbits;
			const uint8_t *buf;
			size_t len;
		} rawdata;

		struct {
			struct h264_macroblock_info *info;
			size_t len;
			size_t maxlen;
		} mb_table;

		/* Count is PicSizeInMapUnits */
		uint32_t *group_map;
		size_t group_map_maxlen;

		/* For AU change detection */
		struct h264_nalu_header prev_slice_nalu_hdr;
		struct h264_slice_header prev_slice_hdr;
	} slice;

	struct h264_macroblock _mb;
	struct h264_macroblock *mb;

	struct h264_sps_derived sps_derived;

	struct {
		/* Derived from PPS */
		uint32_t SliceGroupChangeRate;

		/* Derived from slice header */
		int MbaffFrameFlag;
		uint32_t PicHeightInMbs;
		uint32_t PicHeightInSamplesLuma;
		uint32_t PicHeightInSamplesChroma;
		uint32_t PicSizeInMbs;
		uint32_t MaxPicNum;
		uint32_t CurrPicNum;
		int32_t SliceQPLuma;
		int32_t QSLuma;
		uint32_t FilterOffsetA;
		uint32_t FilterOffsetB;
		uint32_t MapUnitsInSliceGroup0;
	} derived;
};


int h264_ctx_set_active_sps(struct h264_ctx *ctx, uint32_t sps_id);


int h264_ctx_set_active_pps(struct h264_ctx *ctx, uint32_t pps_id);


int h264_ctx_clear_sei_table(struct h264_ctx *ctx);


int h264_ctx_add_sei_internal(struct h264_ctx *ctx, struct h264_sei **ret_obj);


int h264_ctx_clear_slice(struct h264_ctx *ctx);


int h264_write_one_sei(struct h264_bitstream *bs,
		       struct h264_ctx *ctx,
		       const struct h264_sei *sei);


int h264_sei_update_internal_buf(struct h264_sei *sei);


int h264_gen_slice_group_map(struct h264_ctx *ctx);


void h264_clear_slice_group_map(struct h264_ctx *ctx);


uint32_t h264_next_mb_addr(struct h264_ctx *ctx, uint32_t mbAddr);


/**
 * Calculate the log base 2 of the argument, rounded up.
 * Zero arguments return zero
 */
static inline uint32_t h264_intlog2(uint32_t x)
{
	uint32_t r = 0;
	while ((x >> r) > 0)
		r++;
	if (r > 0 && x == (1u << (r - 1)))
		r--;
	return r;
}


static inline uint32_t h264_get_mb_addr_off(struct h264_ctx *ctx,
					    uint32_t mbAddr)
{
	return mbAddr - ctx->slice.hdr.first_mb_in_slice *
				(1 + ctx->derived.MbaffFrameFlag);
}


#endif /* !_H264_PRIV_H_ */
