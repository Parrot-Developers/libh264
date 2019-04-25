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

#include "h264_priv.h"


/**
 * 8.2.2.1 Specification for interleaved slice group map type
 */
static void h264_slice_group_map_type_0(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;

	uint32_t i = 0;
	do {
		for (uint32_t grp = 0; grp <= pps->num_slice_groups_minus1 &&
				       i < PicSizeInMapUnits;
		     i += pps->run_length_minus1[grp++] + 1) {
			for (uint32_t j = 0; j <= pps->run_length_minus1[grp] &&
					     i + j < PicSizeInMapUnits;
			     j++) {
				ctx->slice.group_map[i + j] = grp;
			}
		}
	} while (i < PicSizeInMapUnits);
}


/**
 * 8.2.2.2 Specification for dispersed slice group map type
 */
static void h264_slice_group_map_type_1(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;

	for (uint32_t i = 0; i < PicSizeInMapUnits; i++) {
		ctx->slice.group_map[i] =
			((i % PicWidthInMbs) +
			 (((i / PicWidthInMbs) *
			   (pps->num_slice_groups_minus1 + 1)) /
			  2)) %
			(pps->num_slice_groups_minus1 + 1);
	}
}


/**
 * 8.2.2.3 Specification for foreground with left-over slice group map type
 */
static void h264_slice_group_map_type_2(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;

	for (uint32_t i = 0; i < PicSizeInMapUnits; i++)
		ctx->slice.group_map[i] = pps->num_slice_groups_minus1;

	for (uint32_t grp = pps->num_slice_groups_minus1 - 1; (int32_t)grp >= 0;
	     grp--) {
		uint32_t yTopLeft = pps->top_left[grp] / PicWidthInMbs;
		uint32_t xTopLeft = pps->top_left[grp] % PicWidthInMbs;
		uint32_t yBottomRight = pps->bottom_right[grp] / PicWidthInMbs;
		uint32_t xBottomRight = pps->bottom_right[grp] % PicWidthInMbs;
		for (uint32_t y = yTopLeft; y <= yBottomRight; y++) {
			uint32_t base = y * PicWidthInMbs;
			for (uint32_t x = xTopLeft; x <= xBottomRight; x++)
				ctx->slice.group_map[base + x] = grp;
		}
	}
}


/**
 * 8.2.2.4 Specification for box-out slice group map types
 */
static void h264_slice_group_map_type_3(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	int direction_flag = pps->slice_group_change_direction_flag;
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;
	uint32_t PicHeightInMapUnits = ctx->sps_derived.PicHeightInMapUnits;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;
	uint32_t MapUnitsInSliceGroup0 = ctx->derived.MapUnitsInSliceGroup0;

	for (uint32_t i = 0; i < PicSizeInMapUnits; i++)
		ctx->slice.group_map[i] = 1;

	uint32_t x = (PicWidthInMbs - direction_flag) / 2;
	uint32_t y = (PicHeightInMapUnits - direction_flag) / 2;
	uint32_t left = x;
	uint32_t top = y;
	uint32_t right = x;
	uint32_t bottom = y;
	int32_t xDir = direction_flag - 1;
	int32_t yDir = direction_flag;

	uint32_t mapUnitVacant = 0;
	for (uint32_t k = 0; k < MapUnitsInSliceGroup0; k += mapUnitVacant) {
		if (ctx->slice.group_map[y * PicWidthInMbs + x] == 1) {
			mapUnitVacant = 1;
			ctx->slice.group_map[y * PicWidthInMbs + x] = 0;
		} else {
			mapUnitVacant = 0;
		}

		if (xDir == -1 && x == left) {
			left = Max((int32_t)left - 1, 0);
			x = left;
			xDir = 0;
			yDir = 2 * direction_flag - 1;
		} else if (xDir == 1 && x == right) {
			right = Min(right + 1, PicWidthInMbs - 1);
			x = right;
			xDir = 0;
			yDir = 1 - 2 * direction_flag;
		} else if (yDir == -1 && y == top) {
			top = Max((int32_t)top - 1, 0);
			y = top;
			xDir = 1 - 2 * direction_flag;
			yDir = 0;
		} else if (yDir == 1 && y == bottom) {
			bottom = Min(bottom + 1, PicHeightInMapUnits - 1);
			y = bottom;
			xDir = 2 * direction_flag - 1;
			yDir = 0;
		} else {
			x += xDir;
			y += yDir;
		}
	}
}


/**
 * 8.2.2.5 Specification for raster scan slice group map types
 */
static void h264_slice_group_map_type_4(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	int direction_flag = pps->slice_group_change_direction_flag;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;
	uint32_t MapUnitsInSliceGroup0 = ctx->derived.MapUnitsInSliceGroup0;

	uint32_t sizeOfUpperLeftGroup =
		direction_flag ? (PicSizeInMapUnits - MapUnitsInSliceGroup0)
			       : MapUnitsInSliceGroup0;

	for (uint32_t i = 0; i < PicSizeInMapUnits; i++) {
		ctx->slice.group_map[i] = (i < sizeOfUpperLeftGroup)
						  ? direction_flag
						  : 1 - direction_flag;
	}
}


/**
 * 8.2.2.6 Specification for wipe slice group map types
 */
static void h264_slice_group_map_type_5(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	int direction_flag = pps->slice_group_change_direction_flag;
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;
	uint32_t PicHeightInMapUnits = ctx->sps_derived.PicHeightInMapUnits;
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;
	uint32_t MapUnitsInSliceGroup0 = ctx->derived.MapUnitsInSliceGroup0;

	uint32_t sizeOfUpperLeftGroup =
		direction_flag ? (PicSizeInMapUnits - MapUnitsInSliceGroup0)
			       : MapUnitsInSliceGroup0;

	uint32_t k = 0;
	for (uint32_t j = 0; j < PicWidthInMbs; j++) {
		for (uint32_t i = 0; i < PicHeightInMapUnits; i++) {
			ctx->slice.group_map[i * PicWidthInMbs + j] =
				(k++ < sizeOfUpperLeftGroup)
					? direction_flag
					: 1 - direction_flag;
		}
	}
}


/**
 * 8.2.2.7 Specification for explicit slice group map type
 */
static void h264_slice_group_map_type_6(struct h264_ctx *ctx)
{
	for (uint32_t i = 0; i < ctx->sps_derived.PicSizeInMapUnits; i++)
		ctx->slice.group_map[i] = ctx->pps->slice_group_id[i];
}


/**
 * 8.2.2.8 Specification for conversion of map unit to slice group map to
 *         macroblock to slice group map
 */
static uint32_t h264_mb_to_slice_group(struct h264_ctx *ctx, uint32_t mbAddr)
{
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;

	if (ctx->sps->frame_mbs_only_flag || ctx->slice.hdr.field_pic_flag) {
		return ctx->slice.group_map[mbAddr];
	} else if (ctx->derived.MbaffFrameFlag) {
		return ctx->slice.group_map[mbAddr / 2];
	} else {
		return ctx->slice.group_map[(mbAddr / (2 * PicWidthInMbs)) *
						    PicWidthInMbs +
					    (mbAddr % PicWidthInMbs)];
	}
}


/**
 * 8.2.2 Decoding process for macroblock to slice group map
 */
int h264_gen_slice_group_map(struct h264_ctx *ctx)
{
	uint32_t PicSizeInMapUnits = ctx->sps_derived.PicSizeInMapUnits;

	/* Map not use if no group */
	if (ctx->pps->num_slice_groups_minus1 == 0)
		return 0;

	/* Grow table if needed */
	void *newmap = NULL;
	if (PicSizeInMapUnits > ctx->slice.group_map_maxlen) {
		newmap = realloc(ctx->slice.group_map,
				 /* codecheck_ignore[POINTER_LOCATION] */
				 PicSizeInMapUnits * sizeof(uint32_t));
		if (newmap == NULL)
			return -ENOMEM;
		ctx->slice.group_map = newmap;
		ctx->slice.group_map_maxlen = PicSizeInMapUnits;
	}

	switch (ctx->pps->slice_group_map_type) {
	case 0:
		h264_slice_group_map_type_0(ctx);
		break;
	case 1:
		h264_slice_group_map_type_1(ctx);
		break;
	case 2:
		h264_slice_group_map_type_2(ctx);
		break;
	case 3:
		h264_slice_group_map_type_3(ctx);
		break;
	case 4:
		h264_slice_group_map_type_4(ctx);
		break;
	case 5:
		h264_slice_group_map_type_5(ctx);
		break;
	case 6:
		h264_slice_group_map_type_6(ctx);
		break;
	default:
		return -EIO;
	}

	return 0;
}


void h264_clear_slice_group_map(struct h264_ctx *ctx)
{
	if (ctx->slice.group_map != NULL) {
		memset(ctx->slice.group_map,
		       0,
		       ctx->slice.group_map_maxlen * sizeof(uint32_t));
	}
}


/**
 * 8.2.2 Decoding process for macroblock to slice group map
 */
uint32_t h264_next_mb_addr(struct h264_ctx *ctx, uint32_t mbAddr)
{
	if (ctx->pps->num_slice_groups_minus1 == 0)
		return mbAddr + 1;

	uint32_t group = h264_mb_to_slice_group(ctx, mbAddr);
	uint32_t i = mbAddr + 1;
	while (i < ctx->derived.PicSizeInMbs &&
	       h264_mb_to_slice_group(ctx, i) != group) {
		i++;
	}
	return i;
}
