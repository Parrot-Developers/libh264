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

/* codecheck_ignore_file[GCC_BINARY_CONSTANT] */

/* clang-format off */
/* codecheck_ignore[COMPLEX_MACRO] */
#define CHECK(_x) if ((res = (_x)) < 0) goto out
/* clang-format on */

#if 0
#	define CABAC_LOGV(_fmt, ...) fprintf(stderr, _fmt "\n", ##__VA_ARGS__)
#else
#	define CABAC_LOGV(_fmt, ...)
#endif


struct binstring {
	uint8_t value;
	uint8_t numbits;
	uint8_t maxBinIdxCtx;
	uint16_t ctxIdxOffset;
	int bypassFlag;
};


static int binstring_get_bit(const struct binstring *bins, uint32_t idx)
{
	return (bins->value & (1 << (bins->numbits - idx - 1))) != 0;
}


/**
 * 9.3.3.1.1.1 Derivation process of ctxIdxInc for the syntax element
 * mb_skip_flag
 */
static uint32_t
get_ctx_idx_mb_skip_flag_cond_term(const struct h264_macroblock_info *info)
{
	return info == NULL || info->skipped ? 0 : 1;
}


/**
 * 9.3.3.1.1.1 Derivation process of ctxIdxInc for the syntax element
 * mb_skip_flag
 */
static uint32_t get_ctx_idx_mb_skip_flag(const struct h264_macroblock *mb,
					 uint32_t ctxIdxOffset)
{
	uint32_t condTermFlagA =
		get_ctx_idx_mb_skip_flag_cond_term(mb->mbAddrAInfo);
	uint32_t condTermFlagB =
		get_ctx_idx_mb_skip_flag_cond_term(mb->mbAddrBInfo);
	return ctxIdxOffset + condTermFlagA + condTermFlagB;
}


/**
 * 9.3.3.1.1.3 Derivation process of ctxIdxInc for the syntax element mb_type
 */
static uint32_t
get_ctx_idx_mb_type_cond_term(const struct h264_macroblock_info *info,
			      uint32_t ctxIdxOffset)
{
	enum h264_mb_type mb_type =
		info != NULL ? info->mb_type : H264_MB_TYPE_UNKNOWN;
	if (info == NULL)
		return 0;
	else if (ctxIdxOffset == 0 && mb_type == H264_MB_TYPE_SI)
		return 0;
	else if (ctxIdxOffset == 3 && mb_type == H264_MB_TYPE_I_NxN)
		return 0;
	else if (ctxIdxOffset == 27 && mb_type == H264_MB_TYPE_B_SKIP)
		return 0;
	else if (ctxIdxOffset == 27 && mb_type == H264_MB_TYPE_B_Direct_16x16)
		return 0;
	else
		return 1;
}


/**
 * 9.3.3.1.1.3 Derivation process of ctxIdxInc for the syntax element mb_type
 */
static uint32_t get_ctx_idx_mb_type(const struct h264_macroblock *mb,
				    uint32_t ctxIdxOffset)
{
	uint32_t condTermFlagA =
		get_ctx_idx_mb_type_cond_term(mb->mbAddrAInfo, ctxIdxOffset);
	uint32_t condTermFlagB =
		get_ctx_idx_mb_type_cond_term(mb->mbAddrBInfo, ctxIdxOffset);
	return ctxIdxOffset + condTermFlagA + condTermFlagB;
}


/**
 * 9.3.3.1.1.5 Derivation process of ctxIdxInc for the syntax element
 * mb_qp_delta
 */
static uint32_t get_ctx_idx_mb_qp_delta(const struct h264_macroblock *mb,
					uint32_t ctxIdxOffset)
{
	/* TODO */
	return ctxIdxOffset;
}


/**
 * 9.3.3.1.1.8 Derivation process of ctxIdxInc for the syntax element
 * intra_chroma_pred_mode
 */
static uint32_t get_ctx_idx_intra_chroma_pred_mode_cond_term(
	const struct h264_macroblock_info *info)
{
	if (info == NULL)
		return 0;
	else if (h264_mb_type_is_inter(info->mb_type))
		return 0;
	else if (info->mb_type == H264_MB_TYPE_I_PCM)
		return 0;
	else if (info->intra_chroma_pred_mode == 0)
		return 0;
	else
		return 1;
}


/**
 * 9.3.3.1.1.8 Derivation process of ctxIdxInc for the syntax element
 * intra_chroma_pred_mode
 */
static uint32_t
get_ctx_idx_intra_chroma_pred_mode(const struct h264_macroblock *mb,
				   uint32_t ctxIdxOffset)
{
	uint32_t condTermFlagA =
		get_ctx_idx_intra_chroma_pred_mode_cond_term(mb->mbAddrAInfo);
	uint32_t condTermFlagB =
		get_ctx_idx_intra_chroma_pred_mode_cond_term(mb->mbAddrBInfo);
	return ctxIdxOffset + condTermFlagA + condTermFlagB;
}


/**
 * 9.3.3.1.1.9 Derivation process of ctxIdxInc for the syntax element
 * coded_block_flag
 */
static uint32_t
get_ctx_idx_coded_block_flag_cond_term(const struct h264_ctx *ctx,
				       const struct h264_macroblock *mb,
				       const struct h264_macroblock_info *info,
				       uint32_t ctxBlockCat)
{
	/* TODO */
	int transBlockAvailable = 0;
	int transBlockCodedBlockFlag = 0;

	if (info == NULL && h264_mb_type_is_inter(mb->mb_type)) {
		return 0;
	} else if (info != NULL && !transBlockAvailable &&
		   info->mb_type != H264_MB_TYPE_I_PCM) {
		return 0;
	} else if (h264_mb_type_is_intra(mb->mb_type) &&
		   ctx->pps->constrained_intra_pred_flag == 1 && info != NULL &&
		   h264_mb_type_is_inter(info->mb_type) &&
		   ctx->nalu.hdr.nal_unit_type >= 2 &&
		   ctx->nalu.hdr.nal_unit_type <= 4) {
		return 0;
	} else if (info == NULL && h264_mb_type_is_intra(mb->mb_type)) {
		return 1;
	} else if (info != NULL && info->mb_type == H264_MB_TYPE_I_PCM) {
		return 1;
	} else {
		return transBlockCodedBlockFlag;
	}
}


/**
 * 9.3.3.1.1.9 Derivation process of ctxIdxInc for the syntax element
 * coded_block_flag
 */
static uint32_t get_ctx_idx_coded_block_flag(const struct h264_ctx *ctx,
					     const struct h264_macroblock *mb,
					     uint32_t ctxIdxOffset,
					     uint32_t ctxIdxBlockCatOffset,
					     uint32_t ctxBlockCat)
{
	uint32_t condTermFlagA = get_ctx_idx_coded_block_flag_cond_term(
		ctx, mb, mb->mbAddrAInfo, ctxBlockCat);
	uint32_t condTermFlagB = get_ctx_idx_coded_block_flag_cond_term(
		ctx, mb, mb->mbAddrBInfo, ctxBlockCat);
	return ctxIdxOffset + ctxIdxBlockCatOffset + condTermFlagA +
	       2 * condTermFlagB;
}


/**
 * 9.3.3.1 Derivation process for ctxIdx
 * 9.3.3.1.2 Assignment process of ctxIdxInc using prior decoded bin values
 * Table 9-39 - Assignment of ctxIdxInc to binIdx for all ctxIdxOffset
 * Table 9-41 - Specification of ctxIdxInc for specific values of ctxIdxOffset
 * and binIdx
 */
static uint32_t get_ctx_idx(const struct h264_macroblock *mb,
			    const struct binstring *bins,
			    uint32_t binIdx)
{
	switch (bins->ctxIdxOffset) {
	/* mb_type (SI slices only): prefix */
	case 0:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_type(mb, bins->ctxIdxOffset);
		default:
			break;
		}
		break;

	/* mb_type (I slices only) */
	/* mb_type (SI slices only): suffix */
	case 3:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_type(mb, bins->ctxIdxOffset);
		case 1:
			return 276;
		case 2:
			return bins->ctxIdxOffset + 3;
		case 3:
			return bins->ctxIdxOffset + 4;
		case 4:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 3) != 0 ? 5 : 6);
		case 5:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 3) != 0 ? 6 : 7);
		default:
			return 7;
		}
		break;

	/* mb_skip_flag (P, SP slices only) */
	case 11:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_skip_flag(mb, bins->ctxIdxOffset);
		default:
			break;
		}
		break;

	/* mb_type (P, SP slices only): prefix */
	case 14:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		case 1:
			return bins->ctxIdxOffset + 1;
		case 2:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 1) != 0 ? 2 : 3);
		default:
			break;
		}
		break;

	/* mb_type (P, SP slices only): suffix */
	case 17:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		case 1:
			return 276;
		case 2:
			return bins->ctxIdxOffset + 1;
		case 3:
			return bins->ctxIdxOffset + 2;
		case 4:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 3) != 0 ? 2 : 3);
		default:
			return bins->ctxIdxOffset + 3;
		}
		break;

	/* sub_mb_type[ ] (P, SP slices only) */
	case 21:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		case 1:
			return bins->ctxIdxOffset + 1;
		case 2:
			return bins->ctxIdxOffset + 2;
		default:
			break;
		}
		break;

	/* mb_skip_flag (B slices only) */
	case 24:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_skip_flag(mb, bins->ctxIdxOffset);
		default:
			break;
		}
		break;

	/* mb_type (B slices only): prefix */
	case 27:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_type(mb, bins->ctxIdxOffset);
		case 1:
			return bins->ctxIdxOffset + 3;
		case 2:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 1) != 0 ? 4 : 5);
		default:
			return bins->ctxIdxOffset + 5;
		}
		break;

	/* mb_type (B slices only): suffix */
	case 32:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		case 1:
			return 276;
		case 2:
			return bins->ctxIdxOffset + 1;
		case 3:
			return bins->ctxIdxOffset + 2;
		case 4:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 3) != 0 ? 2 : 3);
		default:
			return bins->ctxIdxOffset + 3;
		}
		break;

	/* sub_mb_type[ ] (B slices only) */
	case 36:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		case 1:
			return bins->ctxIdxOffset + 1;
		case 2:
			return bins->ctxIdxOffset +
			       (binstring_get_bit(bins, 1) != 0 ? 2 : 3);
		case 3:
			return bins->ctxIdxOffset + 3;
		case 4:
			return bins->ctxIdxOffset + 3;
		case 5:
			return bins->ctxIdxOffset + 3;
		default:
			break;
		}
		break;

	/* mvd_l0[ ][ ][ 0 ], mvd_l1[ ][ ][ 0 ]: prefix */
	case 40:
		break;

	/* mvd_l0[ ][ ][ 1 ], mvd_l1[ ][ ][ 1 ]: prefix */
	case 47:
		break;

	/* ref_idx_l0, ref_idx_l1 */
	case 54:
		break;

	/* mb_qp_delta */
	case 60:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_mb_qp_delta(mb, bins->ctxIdxOffset);
		case 1:
			return bins->ctxIdxOffset + 2;
		default:
			return bins->ctxIdxOffset + 3;
		}
		break;

	/* intra_chroma_pred_mode */
	case 64:
		switch (binIdx) {
		case 0:
			return get_ctx_idx_intra_chroma_pred_mode(
				mb, bins->ctxIdxOffset);
		case 1:
			return bins->ctxIdxOffset + 3;
		case 2:
			return bins->ctxIdxOffset + 3;
		default:
			break;
		}
		break;

	/* prev_intra4x4_pred_mode_flag, prev_intra8x8_pred_mode_flag */
	case 68:
		break;

	/* rem_intra4x4_pred_mode, rem_intra8x8_pred_mode */
	case 69:
		break;

	/* mb_field_decoding_flag */
	case 70:
		break;

	/* coded_block_pattern: prefix */
	case 73:
		break;

	/* coded_block_pattern: suffix */
	case 77:
		break;

	/* end_of_slice_flag */
	case 276:
		switch (binIdx) {
		case 0:
			return bins->ctxIdxOffset;
		default:
			break;
		}
		break;

	/* transform_size_8x8_flag */
	case 399:
		break;
	}

	ULOGW("%s:%d: unsupported ctxIdxOffset %u",
	      __func__,
	      __LINE__,
	      bins->ctxIdxOffset);
	return 0;
}


/**
 * 9.3.3.1.3 Assignment process of ctxIdxInc for syntax elements
 * significant_coeff_flag, last_significant_coeff_flag, and
 * coeff_abs_level_minus1
 * Table 9-42 - Specification of ctxBlockCat for the different blocks
 */
static uint32_t get_ctx_block_cat(uint32_t mode)
{
	switch (mode) {
	case Intra16x16DCLevel:
		return 0;
	case Intra16x16ACLevel:
		return 1;
	case LumaLevel4x4:
		return 2;
	case ChromaDCLevel:
		return 3;
	case ChromaACLevel:
		return 4;
	/* TODO: case LumaLevel8x8: return 5; */
	case CbIntra16x16DCLevel:
		return 6;
	case CbIntra16x16ACLevel:
		return 7;
	case CbLevel4x4:
		return 8;
	/* TODO: case CbLevel8x8: return 9; */
	case CrIntra16x16DCLevel:
		return 10;
	case CrIntra16x16ACLevel:
		return 11;
	case CrLevel4x4:
		return 12;
	/* TODO: case CrLevel8x8: return 13; */
	default:
		ULOGW("%s:%d: unsupported mode %u", __func__, __LINE__, mode);
		return 0;
	}
}


static int write_binstring(struct h264_cabac *cabac,
			   const struct binstring *bins,
			   const struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t ctxIdx = 0;
	int bin = 0;

	for (uint32_t binIdx = 0; binIdx < bins->numbits; binIdx++) {
		bin = binstring_get_bit(bins, binIdx);
		if (bins->bypassFlag) {
			CHECK(h264_bac_encode_bypass(&cabac->enc, bin));
		} else {
			ctxIdx = get_ctx_idx(mb, bins, binIdx);
			CABAC_LOGV("%s ctxIdx=%u", __func__, ctxIdx);
			if (ctxIdx == 276) {
				CHECK(h264_bac_encode_terminate(&cabac->enc,
								bin));
			} else {
				CHECK(h264_bac_encode_bin(
					&cabac->enc,
					&cabac->states[ctxIdx],
					bin));
			}
		}
	}

out:
	return res;
}


static int write_binstring_with_ctx_idx(struct h264_cabac *cabac,
					const struct binstring *bins,
					uint32_t ctxIdx)
{
	int res = 0;
	int bin = 0;

	CABAC_LOGV("%s ctxIdx=%u", __func__, ctxIdx);
	for (uint32_t binIdx = 0; binIdx < bins->numbits; binIdx++) {
		bin = binstring_get_bit(bins, binIdx);
		CHECK(h264_bac_encode_bin(
			&cabac->enc, &cabac->states[ctxIdx], bin));
	}

out:
	return res;
}


/**
 * 9.3.2.1 Unary (U) binarization process
 */
static void binarize_u(uint32_t val, struct binstring *bins)
{
	if (val > 7)
		ULOGW("%s:%d: val too big %u", __func__, __LINE__, val);
	bins->value = ((1 << val) - 1) << 1;
	bins->numbits = val + 1;
}


/**
 * 9.3.2.2 Truncated unary (TU) binarization process
 */
static void binarize_tu(uint32_t cMax, uint32_t val, struct binstring *bins)
{
	if (val > cMax)
		ULOGW("%s:%d: val too big %u", __func__, __LINE__, val);
	if (val < cMax) {
		binarize_u(val, bins);
	} else {
		bins->value = (1 << val) - 1;
		bins->numbits = val;
	}
}


/**
 * 9.3.1.1 Initialisation process for context variables
 */
int h264_cabac_init_enc(struct h264_cabac *cabac,
			struct h264_ctx *ctx,
			struct h264_bitstream *bs)
{
	int res = 0;
	int first_slice = ctx->slice.hdr.first_mb_in_slice == 0;
	CABAC_LOGV("%s cabac_init_idc=%u SliceQPLuma=%d",
		   __func__,
		   ctx->slice.hdr.cabac_init_idc,
		   ctx->derived.SliceQPLuma);
	CHECK(h264_bac_encode_init(&cabac->enc, bs, first_slice));
	h264_cabac_init_states(cabac, ctx);
out:
	return res;
}


int h264_cabac_init_dec(struct h264_cabac *cabac,
			struct h264_ctx *ctx,
			struct h264_bitstream *bs)
{
	int res = 0;
	CHECK(h264_bac_decode_init(&cabac->dec, bs));
	h264_cabac_init_states(cabac, ctx);
out:
	return res;
}


/**
 * 9.3.2.5 Binarization process for macroblock type and sub-macroblock type
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_mb_type(struct h264_cabac *cabac,
			     struct h264_ctx *ctx,
			     struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t raw_mb_type = 0;
	struct binstring prefix;
	struct binstring suffix;
	CABAC_LOGV("%s", __func__);

	/* clang-format off */

	/* Table 9-36
	 * - Binarization for macroblock types in I slices */
	static const uint8_t table_I[26][2] = {
		[0]  = { 0b0, 1 },
		[1]  = { 0b100000, 6 },
		[2]  = { 0b100001, 6 },
		[3]  = { 0b100010, 6 },
		[4]  = { 0b100011, 6 },
		[5]  = { 0b1001000, 7 },
		[6]  = { 0b1001001, 7 },
		[7]  = { 0b1001010, 7 },
		[8]  = { 0b1001011, 7 },
		[9]  = { 0b1001100, 7 },
		[10] = { 0b1001101, 7 },
		[11] = { 0b1001110, 7 },
		[12] = { 0b1001111, 7 },
		[13] = { 0b101000, 6 },
		[14] = { 0b101001, 6 },
		[15] = { 0b101010, 6 },
		[16] = { 0b101011, 6 },
		[17] = { 0b1011000, 7 },
		[18] = { 0b1011001, 7 },
		[19] = { 0b1011010, 7 },
		[20] = { 0b1011011, 7 },
		[21] = { 0b1011100, 7 },
		[22] = { 0b1011101, 7 },
		[23] = { 0b1011110, 7 },
		[24] = { 0b1011111, 7 },
		[25] = { 0b11, 2 },
	};

	/* Table 9-37
	 * - Binarization for macroblock types in P, SP, and B slices */
	static const uint8_t table_P[5][2] = {
		[0]  = { 0b000, 3 },
		[1]  = { 0b011, 3 },
		[2]  = { 0b010, 3 },
		[3]  = { 0b001, 3 },
		[4]  = { 0, 0}, /* NA */
	};
	static const uint8_t table_B[23][2] = {
		[0]  = { 0b0, 1 },
		[1]  = { 0b100, 3 },
		[2]  = { 0b101, 3 },
		[3]  = { 0b110000, 6 },
		[4]  = { 0b110001, 6 },
		[5]  = { 0b110010, 6 },
		[6]  = { 0b110011, 6 },
		[7]  = { 0b110100, 6 },
		[8]  = { 0b110101, 6 },
		[9]  = { 0b110110, 6 },
		[10] = { 0b110111, 6 },
		[11] = { 0b111110, 6 },
		[12] = { 0b1110000, 7 },
		[13] = { 0b1110001, 7 },
		[14] = { 0b1110010, 7 },
		[15] = { 0b1110011, 7 },
		[16] = { 0b1110100, 7 },
		[17] = { 0b1110101, 7 },
		[18] = { 0b1110110, 7 },
		[19] = { 0b1110111, 7 },
		[20] = { 0b1111000, 7 },
		[21] = { 0b1111001, 7 },
		[22] = { 0b111111, 6 },
	};

	/* clang-format on */

	raw_mb_type = mb->raw_mb_type;
	prefix.numbits = 0;
	prefix.bypassFlag = 0;
	suffix.numbits = 0;
	suffix.bypassFlag = 0;

	switch (ctx->slice.type) {
	case H264_SLICE_TYPE_I:
		ULOG_ERRNO_RETURN_ERR_IF(raw_mb_type > 25, EIO);
		prefix.value = table_I[raw_mb_type][0];
		prefix.numbits = table_I[raw_mb_type][1];
		prefix.maxBinIdxCtx = 6;
		prefix.ctxIdxOffset = 3;
		break;

	case H264_SLICE_TYPE_SI:
		ULOG_ERRNO_RETURN_ERR_IF(raw_mb_type > 26, EIO);
		if (raw_mb_type == 0) {
			prefix.value = 0;
			prefix.numbits = 1;
			prefix.maxBinIdxCtx = 0;
			prefix.ctxIdxOffset = 0;
		} else {
			prefix.value = 1;
			prefix.numbits = 1;
			prefix.maxBinIdxCtx = 0;
			prefix.ctxIdxOffset = 0;

			suffix.value = table_I[raw_mb_type - 1][0];
			suffix.numbits = table_I[raw_mb_type - 1][1];
			suffix.maxBinIdxCtx = 6;
			suffix.ctxIdxOffset = 3;
		}
		break;

	case H264_SLICE_TYPE_P: /* NO BREAK */
	case H264_SLICE_TYPE_SP:
		ULOG_ERRNO_RETURN_ERR_IF(raw_mb_type > 30, EIO);
		ULOG_ERRNO_RETURN_ERR_IF(raw_mb_type == 4, EIO);
		if (raw_mb_type <= 4) {
			prefix.value = table_P[raw_mb_type][0];
			prefix.numbits = table_P[raw_mb_type][1];
			prefix.maxBinIdxCtx = 2;
			prefix.ctxIdxOffset = 14;
		} else {
			prefix.value = 1;
			prefix.numbits = 1;
			prefix.maxBinIdxCtx = 2;
			prefix.ctxIdxOffset = 14;

			suffix.value = table_I[raw_mb_type - 5][0];
			suffix.numbits = table_I[raw_mb_type - 5][1];
			suffix.maxBinIdxCtx = 5;
			suffix.ctxIdxOffset = 17;
		}
		break;

	case H264_SLICE_TYPE_B:
		ULOG_ERRNO_RETURN_ERR_IF(raw_mb_type > 48, EIO);
		if (raw_mb_type <= 22) {
			prefix.value = table_B[raw_mb_type][0];
			prefix.numbits = table_B[raw_mb_type][1];
			prefix.maxBinIdxCtx = 3;
			prefix.ctxIdxOffset = 27;
		} else {
			prefix.value = 0b111101;
			prefix.numbits = 6;
			prefix.maxBinIdxCtx = 3;
			prefix.ctxIdxOffset = 27;

			suffix.value = table_I[raw_mb_type - 23][0];
			suffix.numbits = table_I[raw_mb_type - 23][1];
			suffix.maxBinIdxCtx = 5;
			suffix.ctxIdxOffset = 32;
		}
		break;

	case H264_SLICE_TYPE_UNKNOWN: /* NO BREAK */
	default:
		return -EIO;
	}

	if (prefix.numbits > 0)
		CHECK(write_binstring(cabac, &prefix, mb));
	if (suffix.numbits > 0)
		CHECK(write_binstring(cabac, &suffix, mb));

out:
	return res;
}


/**
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_intra_chroma_pred_mode(struct h264_cabac *cabac,
					    struct h264_ctx *ctx,
					    struct h264_macroblock *mb)
{
	int res = 0;
	struct binstring bins;
	CABAC_LOGV("%s", __func__);

	binarize_tu(3, mb->intra_chroma_pred_mode, &bins);
	bins.bypassFlag = 0;
	bins.maxBinIdxCtx = 1;
	bins.ctxIdxOffset = 64;
	CHECK(write_binstring(cabac, &bins, mb));

out:
	return res;
}


/**
 * 9.3.2.7 Binarization process for mb_qp_delta
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_mb_qp_delta(struct h264_cabac *cabac,
				 struct h264_ctx *ctx,
				 struct h264_macroblock *mb)
{
	int res = 0;
	int32_t v = 0;
	struct binstring bins;
	CABAC_LOGV("%s", __func__);

	v = mb->mb_qp_delta;
	if (v <= 0)
		binarize_u((uint32_t)(-2 * v), &bins);
	else
		binarize_u((uint32_t)(2 * v - 1), &bins);

	bins.bypassFlag = 0;
	bins.maxBinIdxCtx = 2;
	bins.ctxIdxOffset = 60;
	CHECK(write_binstring(cabac, &bins, mb));

out:
	return res;
}


/**
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_coded_block_flag(struct h264_cabac *cabac,
				      struct h264_ctx *ctx,
				      struct h264_macroblock *mb,
				      uint32_t mode,
				      int flag)
{
	int res = 0;
	uint32_t ctxBlockCat = 0;
	uint32_t ctxIdxBlockCatOffset = 0;
	uint32_t ctxIdx = 0;
	struct binstring bins;

	/* Table 9-34 - Syntax elements and associated types of binarization,
	 * maxBinIdxCtx, and ctxIdxOffset
	 * Table 9-40 - Assignment of ctxIdxBlockCatOffset to ctxBlockCat for
	 * syntax elements coded_block_flag */
	static const uint16_t table[14][2] = {
		[0] = {0, 85},
		[1] = {4, 85},
		[2] = {8, 85},
		[3] = {12, 85},
		[4] = {16, 85},
		[5] = {0, 1012},
		[6] = {0, 460},
		[7] = {4, 460},
		[8] = {8, 460},
		[9] = {4, 1012},
		[10] = {0, 472},
		[11] = {4, 472},
		[12] = {8, 472},
		[13] = {8, 1012},
	};

	CABAC_LOGV("%s", __func__);

	bins.value = flag;
	bins.numbits = 1;
	bins.bypassFlag = 0;
	bins.maxBinIdxCtx = 0;

	ctxBlockCat = get_ctx_block_cat(mode);
	ctxIdxBlockCatOffset = table[ctxBlockCat][0];
	bins.ctxIdxOffset = table[ctxBlockCat][1];

	ctxIdx = get_ctx_idx_coded_block_flag(
		ctx, mb, bins.ctxIdxOffset, ctxIdxBlockCatOffset, ctxBlockCat);
	CHECK(write_binstring_with_ctx_idx(cabac, &bins, ctxIdx));

out:
	return res;
}


/**
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_mb_skip_flag(struct h264_cabac *cabac,
				  struct h264_ctx *ctx,
				  struct h264_macroblock *mb,
				  int flag)
{
	int res = 0;
	struct binstring bins;
	CABAC_LOGV("%s", __func__);

	bins.bypassFlag = 0;
	bins.maxBinIdxCtx = 0;

	switch (ctx->slice.type) {
	case H264_SLICE_TYPE_P: /* NO BREAK */
	case H264_SLICE_TYPE_SP:
		bins.value = flag;
		bins.numbits = 1;
		bins.ctxIdxOffset = 11;
		break;

	case H264_SLICE_TYPE_B: /* NO BREAK */
		bins.value = flag;
		bins.numbits = 1;
		bins.ctxIdxOffset = 24;
		break;

	default:
		return -EIO;
	}

	CHECK(write_binstring(cabac, &bins, mb));

out:
	return res;
}


/**
 * Table 9-34 - Syntax elements and associated types of binarization,
 * maxBinIdxCtx, and ctxIdxOffset
 */
int h264_cabac_write_end_of_slice_flag(struct h264_cabac *cabac,
				       struct h264_ctx *ctx,
				       struct h264_macroblock *mb,
				       int flag)
{
	int res = 0;
	struct binstring bins;
	CABAC_LOGV("%s", __func__);

	bins.value = flag;
	bins.numbits = 1;
	bins.bypassFlag = 0;
	bins.maxBinIdxCtx = 0;
	bins.ctxIdxOffset = 276;
	CHECK(write_binstring(cabac, &bins, mb));

out:
	return res;
}
