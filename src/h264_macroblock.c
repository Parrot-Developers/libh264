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
 * 6.4.3 Inverse 4x4 luma block scanning process
 * 6.4.4 Inverse 4x4 Cb or Cr block scanning process for ChromaArrayType
 *       equal to 3
 */
static void h264_inv_luma_cb_cr_4x4(uint32_t idx, uint32_t *x, uint32_t *y)
{
	static const uint32_t table[16][2] = {
		{0, 0},
		{4, 0},
		{0, 4},
		{4, 4},
		{8, 0},
		{12, 0},
		{8, 4},
		{12, 4},
		{0, 8},
		{4, 8},
		{0, 12},
		{4, 12},
		{8, 8},
		{12, 8},
		{8, 12},
		{12, 12},
	};
	*x = table[idx][0];
	*y = table[idx][1];
}


/**
 * 6.4.7 Inverse 4x4 chroma block scanning process
 */
static void h264_inv_chroma_4x4(uint32_t idx, uint32_t *x, uint32_t *y)
{
	static const uint32_t table[8][2] = {
		{0, 0},
		{4, 0},
		{0, 4},
		{4, 4},
		{0, 8},
		{4, 8},
		{0, 12},
		{4, 12},
	};
	*x = table[idx][0];
	*y = table[idx][1];
}


/**
 * 6.4.12.1 Specification for neighbouring locations in fields and non-MBAFF
 *          frames
 */
static void
h264_get_neighbouring_locations_non_mbaff(struct h264_ctx *ctx,
					  struct h264_macroblock *mb,
					  uint32_t maxW,
					  uint32_t maxH,
					  int32_t xN,
					  int32_t yN,
					  uint32_t *mbAddrN,
					  uint32_t *xW,
					  uint32_t *yW)
{
	if (xN < 0)
		*mbAddrN = mb->mbAddrA;
	else if (yN < 0)
		*mbAddrN = mb->mbAddrB;
	else
		*mbAddrN = mb->mbAddr;

	*xW = xN < 0 ? (uint32_t)(xN + maxW) : (uint32_t)xN;
	*yW = yN < 0 ? (uint32_t)(yN + maxH) : (uint32_t)yN;
}


/**
 * 6.4.12.2 Specification for neighbouring locations in MBAFF frames
 */
static void h264_get_neighbouring_locations_mbaff(struct h264_ctx *ctx,
						  struct h264_macroblock *mb,
						  uint32_t maxW,
						  uint32_t maxH,
						  int32_t xN,
						  int32_t yN,
						  uint32_t *mbAddrN,
						  uint32_t *xW,
						  uint32_t *yW)
{
	int currMbFrameFlag = !mb->mb_field_decoding_flag;
	int mbIsTopMbFlag = mb->mbAddr % 2 == 0;
	int32_t yM = 0;
	if (xN < 0) {
		uint32_t mbAddrX = mb->mbAddrA;
		if (mbAddrX == H264_MB_ADDR_INVALID) {
			*mbAddrN = H264_MB_ADDR_INVALID;
			*xW = *yW = 0;
			return;
		}
		int mbAddrXFrameFlag = !mb->mbAddrAInfo->field_flag;
		if (currMbFrameFlag) {
			if (mbIsTopMbFlag) {
				if (mbAddrXFrameFlag) {
					*mbAddrN = mb->mbAddrA;
					yM = yN;
				} else {
					if (yN % 2 == 0) {
						*mbAddrN = mb->mbAddrA;
						yM = yN >> 1;
					} else {
						*mbAddrN = mb->mbAddrA + 1;
						yM = yN >> 1;
					}
				}
			} else {
				if (mbAddrXFrameFlag) {
					*mbAddrN = mb->mbAddrA + 1;
					yM = yN;
				} else {
					if (yN % 2 == 0) {
						*mbAddrN = mb->mbAddrA;
						yM = (yN + maxH) >> 1;
					} else {
						*mbAddrN = mb->mbAddrA + 1;
						yM = (yN + maxH) >> 1;
					}
				}
			}
		} else {
			if (mbIsTopMbFlag) {
				if (mbAddrXFrameFlag) {
					if ((uint32_t)yN < (maxH / 2)) {
						*mbAddrN = mb->mbAddrA;
						yM = yN << 1;
					} else {
						*mbAddrN = mb->mbAddrA + 1;
						yM = (yN << 1) - maxH;
					}
				} else {
					*mbAddrN = mb->mbAddrA;
					yM = yN;
				}
			} else {
				if (mbAddrXFrameFlag) {
					if ((uint32_t)yN < (maxH / 2)) {
						*mbAddrN = mb->mbAddrA;
						yM = (yN << 1) + 1;
					} else {
						*mbAddrN = mb->mbAddrA + 1;
						yM = (yN << 1) + 1 - maxH;
					}
				} else {
					*mbAddrN = mb->mbAddrA + 1;
					yM = yN;
				}
			}
		}
	} else if (yN < 0) {
		if (currMbFrameFlag) {
			if (mbIsTopMbFlag) {
				uint32_t mbAddrX = mb->mbAddrB;
				if (mbAddrX == H264_MB_ADDR_INVALID) {
					*mbAddrN = H264_MB_ADDR_INVALID;
					*xW = *yW = 0;
					return;
				}
				*mbAddrN = mb->mbAddrB + 1;
				yM = yN;
			} else {
				*mbAddrN = mb->mbAddr - 1;
				yM = yN;
			}
		} else {
			uint32_t mbAddrX = mb->mbAddrB;
			if (mbAddrX == H264_MB_ADDR_INVALID) {
				*mbAddrN = H264_MB_ADDR_INVALID;
				*xW = *yW = 0;
				return;
			}
			int mbAddrXFrameFlag = !mb->mbAddrBInfo->field_flag;
			if (mbIsTopMbFlag) {
				if (mbAddrXFrameFlag) {
					*mbAddrN = mb->mbAddrB + 1;
					yM = 2 * yN;
				} else {
					*mbAddrN = mb->mbAddrB;
					yM = yN;
				}
			} else {
				*mbAddrN = mb->mbAddrB + 1;
				yM = yN;
			}
		}
	} else {
		*mbAddrN = mb->mbAddr;
		yM = yN;
	}

	*xW = xN < 0 ? (uint32_t)(xN + maxW) : (uint32_t)xN;
	*yW = yM < 0 ? (uint32_t)(yM + maxH) : (uint32_t)yM;
}


/**
 * 6.4.12 Derivation process for neighbouring locations
 *
 * (xN, yN): luma or chroma location expressed relative to the upper left corner
 *           of the current macroblock.
 * (maxW, maxH): maximum values of the location components xN, xW, and yN, yW.
 * mbAddrN: CurrMbAddr or to the address of neighbouring macroblock that
 *          contains (xN, yN) and its availability status.
 * (xW, yW): the location (xN, yN) expressed relative to the upper-left corner
 *           of the macroblock mbAddrN (rather than relative to the upper-left
 *           corner of the current macroblock).
 */
static void h264_get_neighbouring_locations(struct h264_ctx *ctx,
					    struct h264_macroblock *mb,
					    uint32_t maxW,
					    uint32_t maxH,
					    int32_t xN,
					    int32_t yN,
					    uint32_t *mbAddrN,
					    uint32_t *xW,
					    uint32_t *yW)
{
	if (!ctx->derived.MbaffFrameFlag) {
		h264_get_neighbouring_locations_non_mbaff(
			ctx, mb, maxW, maxH, xN, yN, mbAddrN, xW, yW);
	} else {
		h264_get_neighbouring_locations_mbaff(
			ctx, mb, maxW, maxH, xN, yN, mbAddrN, xW, yW);
	}
}


/**
 * 6.4.13.1 Derivation process for 4x4 luma block indices
 * Derivation process for 4x4 Cb or Cr block indices for ChromaArrayType
 *  equal to 3
 *
 * (xP, yP): luma/cb/cr location relative to the upper-left luma sample of a
 *           macroblock
 * idx: 4x4 luma/cb/cr block index
 */
static void h264_idx_luma_cb_cr_4x4(uint32_t xP, uint32_t yP, uint32_t *idx)
{
	static const uint32_t table[4][4] = {
		{0, 2, 8, 10},
		{1, 3, 9, 11},
		{4, 6, 12, 14},
		{5, 7, 13, 15},
	};
	*idx = table[xP / 4][yP / 4];
}


/**
 * 6.4.13.2 Derivation process for 4x4 chroma block indices
 *
 * (xP, yP): chroma location relative to the upper-left chroma sample of a
 *           macroblock
 * idx: chroma block index
 */
static void h264_idx_chroma_4x4(uint32_t xP, uint32_t yP, uint32_t *idx)
{
	*idx = 2 * (yP / 4) + (xP / 4);
}


/**
 * 6.4.9 Derivation process for neighbouring macroblock addresses and their
 *       availability
 * 6.4.10 Derivation process for neighbouring macroblock addresses and their
 *        availability in MBAFF frames
 */
void h264_compute_neighbouring_macroblocks(struct h264_ctx *ctx,
					   struct h264_macroblock *mb)
{
	uint32_t PicWidthInMbs = ctx->sps_derived.PicWidthInMbs;
	uint32_t first_mb_in_slice = ctx->slice.hdr.first_mb_in_slice;

	mb->mbAddrA = H264_MB_ADDR_INVALID;
	mb->mbAddrB = H264_MB_ADDR_INVALID;
	mb->mbAddrAInfo = NULL;
	mb->mbAddrBInfo = NULL;

	/* Determine left and top macroblock */
	if (!ctx->derived.MbaffFrameFlag) {
		if (mb->mbAddr >= first_mb_in_slice + 1 &&
		    mb->mbAddr % PicWidthInMbs != 0) {
			mb->mbAddrA = mb->mbAddr - 1;
		}

		if (mb->mbAddr >= first_mb_in_slice + PicWidthInMbs)
			mb->mbAddrB = mb->mbAddr - PicWidthInMbs;
	} else {
		uint32_t half_addr = mb->mbAddr / 2;
		if (half_addr >= first_mb_in_slice + 1 &&
		    half_addr % PicWidthInMbs != 0) {
			mb->mbAddrA = 2 * (half_addr - 1);
		}
		if (half_addr >= first_mb_in_slice + PicWidthInMbs)
			mb->mbAddrB = 2 * (half_addr - PicWidthInMbs);
	}

	/* Check availability and determine field flag */
	if (mb->mbAddrA != H264_MB_ADDR_INVALID) {
		uint32_t offA = h264_get_mb_addr_off(ctx, mb->mbAddrA);
		if (!ctx->slice.mb_table.info[offA].available)
			mb->mbAddrA = H264_MB_ADDR_INVALID;
		else
			mb->mbAddrAInfo = &ctx->slice.mb_table.info[offA];
	}
	if (mb->mbAddrB != H264_MB_ADDR_INVALID) {
		uint32_t offB = h264_get_mb_addr_off(ctx, mb->mbAddrB);
		if (!ctx->slice.mb_table.info[offB].available)
			mb->mbAddrB = H264_MB_ADDR_INVALID;
		else
			mb->mbAddrBInfo = &ctx->slice.mb_table.info[offB];
	}
}


/**
 * 6.4.11.4 Derivation process for neighbouring 4x4 luma blocks
 * 6.4.11.6 Derivation process for neighbouring 4x4 chroma blocks for
 *          ChromaArrayType equal to 3
 *
 * idx: 4x4 luma block index.
 * mbAddrA: either equal to CurrMbAddr or the address of the macroblock to the
 *          left of the current macroblock and its availability status.
 * idxA: the index of the 4x4 luma/cb/cr block to the left of the 4x4 block with
 *       index idx and its availability status.
 * mbAddrB: either equal to CurrMbAddr or the address of the macroblock above
 *          the current macroblock and its availability status.
 * idxB: the index of the 4x4 luma/cb/cr block above the 4x4 block with index
 *       idx and its availability status.
 */
void h264_get_neighbouring_luma_cb_cr_4x4(struct h264_ctx *ctx,
					  struct h264_macroblock *mb,
					  uint32_t idx,
					  uint32_t *mbAddrA,
					  uint32_t *idxA,
					  uint32_t *mbAddrB,
					  uint32_t *idxB)
{
	uint32_t x = 0, y = 0;
	uint32_t xW = 0, yW = 0;
	h264_inv_luma_cb_cr_4x4(idx, &x, &y);
	h264_get_neighbouring_locations(
		ctx, mb, 16, 16, x - 1, y, mbAddrA, &xW, &yW);
	h264_idx_luma_cb_cr_4x4(xW, yW, idxA);
	h264_get_neighbouring_locations(
		ctx, mb, 16, 16, x, y - 1, mbAddrB, &xW, &yW);
	h264_idx_luma_cb_cr_4x4(xW, yW, idxB);
}


/**
 * 6.4.11.5 Derivation process for neighbouring 4x4 chroma blocks
 *
 * idx: 4x4 chroma block index.
 * mbAddrA: either equal to CurrMbAddr or the address of the macroblock to the
 *          left of the current macroblock and its availability status.
 * idxA: the index of the 4x4 chroma block to the left of the 4x4 chroma block
 *       with index idx and its availability status.
 * mbAddrB: either equal to CurrMbAddr or the address of the macroblock above
 *          the current macroblock and its availability status.
 * idxB: the index of the 4x4 chroma block above the 4x4 chroma block with index
 *       idx and its availability status.
 */
void h264_get_neighbouring_chroma_4x4(struct h264_ctx *ctx,
				      struct h264_macroblock *mb,
				      uint32_t idx,
				      uint32_t *mbAddrA,
				      uint32_t *idxA,
				      uint32_t *mbAddrB,
				      uint32_t *idxB)
{
	uint32_t x = 0, y = 0;
	uint32_t xW = 0, yW = 0;
	h264_inv_chroma_4x4(idx, &x, &y);
	h264_get_neighbouring_locations(ctx,
					mb,
					ctx->sps_derived.MbWidthC,
					ctx->sps_derived.MbHeightC,
					x - 1,
					y,
					mbAddrA,
					&xW,
					&yW);
	h264_idx_chroma_4x4(xW, yW, idxA);
	h264_get_neighbouring_locations(ctx,
					mb,
					ctx->sps_derived.MbWidthC,
					ctx->sps_derived.MbHeightC,
					x,
					y - 1,
					mbAddrB,
					&xW,
					&yW);
	h264_idx_chroma_4x4(xW, yW, idxB);
}
