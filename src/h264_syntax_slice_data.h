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

#ifndef _H264_SYNTAX_SLICE_DATA_H_
#define _H264_SYNTAX_SLICE_DATA_H_

/* codecheck_ignore_file[LONG_LINE] */


#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ ||                         \
	H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP


/* clang-format off */
/* Overwrite macros so that dump also do a read first */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP

#	undef H264_BITS
#	undef H264_BITS_U
#	undef H264_BITS_I
#	undef H264_BITS_UE
#	undef H264_BITS_SE
#	undef H264_BITS_TE

#	define _H264_DUMP_BITS(_name, ...)                                     \
		do {                                                           \
			H264_READ_BITS_##_name(__VA_ARGS__);                   \
			H264_DUMP_BITS_##_name(__VA_ARGS__);                   \
		} while (0)

#	define H264_BITS(_f, _n) _H264_DUMP_BITS(U, _f, _n)
#	define H264_BITS_U(_f, _n) _H264_DUMP_BITS(U, _f, _n)
#	define H264_BITS_I(_f, _n) _H264_DUMP_BITS(I, _f, _n)
#	define H264_BITS_UE(_f) _H264_DUMP_BITS(UE, _f)
#	define H264_BITS_SE(_f) _H264_DUMP_BITS(SE, _f)
#	define H264_BITS_TE(_f, _m) _H264_DUMP_BITS(TE, _f, _m)

#endif /* H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP */
/* clang-format on */


/* clang-format off */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP
/* clang-format on */
static const char *get_mode_str(uint32_t mode, uint32_t comp)
{
	switch (mode) {
	case Intra16x16DCLevel:
		return "Luma16DC";
	case Intra16x16ACLevel:
		return "Luma16AC";
	case CbIntra16x16DCLevel:
		return "Cb16DC";
	case CbIntra16x16ACLevel:
		return "Cb16AC";
	case CrIntra16x16DCLevel:
		return "Cr16DC";
	case CrIntra16x16ACLevel:
		return "Cr16AC";
	case LumaLevel4x4:
		return "Luma";
	case CbLevel4x4:
		return "Cb";
	case CrLevel4x4:
		return "Cr";
	case ChromaDCLevel:
		return comp == Cb ? "CbDC" : "CrDC";
	case ChromaACLevel:
		return comp == Cb ? "CbAC" : "CrAC";

	default:
		return "??";
	}
}
/* clang-format off */
#endif
/* clang-format on */


static int H264_SYNTAX_FCT(residual_block)(struct h264_bitstream *bs,
					   struct h264_ctx *ctx,
					   struct h264_macroblock *mb,
					   int16_t coeffLevel[],
					   uint32_t startIdx,
					   uint32_t endIdx,
					   uint32_t maxNumCoeff,
					   uint32_t mode,
					   uint32_t comp,
					   uint32_t blkIdx)
{
	int res = 0;
	int bit = 0;
	uint32_t total_coeff = 0;
	uint32_t trailing_ones = 0;
	uint32_t suffixLength = 0;

	/* clang-format off */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP
	const char *name = get_mode_str(mode, comp);
#endif
	/* clang-format on */

	int16_t levelVal[64];
	int runVal[64];

	for (uint32_t i = 0; i < maxNumCoeff; i++)
		coeffLevel[i] = 0;

	res = h264_read_coeff_token(
		bs, ctx, mb, mode, comp, blkIdx, &trailing_ones, &total_coeff);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (total_coeff == 0)
		return 0;

	if (total_coeff > 10 && trailing_ones < 3)
		suffixLength = 1;
	else
		suffixLength = 0;

	for (uint32_t i = 0; i < total_coeff; i++) {
		if (i < trailing_ones) {
			int trailing_ones_sign_flag = 0;
			H264_READ_BITS(trailing_ones_sign_flag, 1);
			levelVal[i] = 1 - 2 * trailing_ones_sign_flag;
		} else {
			/* 9.2.2.1 Parsing process for level_prefix */
			uint32_t level_prefix = 0;
			H264_READ_BITS(bit, 1);
			/* Limit level_prefix to 25;
			 * spec says it is at max 15 or 11 + bitDepthLuma
			 * bitDepthLuma itself is at max 14 */
			while (bit == 0) {
				level_prefix++;
				ULOG_ERRNO_RETURN_ERR_IF(level_prefix > 25,
							 EIO);
				H264_READ_BITS(bit, 1);
			}

			uint32_t levelCode =
				(Min(15, level_prefix) << suffixLength);
			if (suffixLength > 0 || level_prefix >= 14) {
				uint32_t levelSuffixSize = 0;
				if (level_prefix == 14 && suffixLength == 0)
					levelSuffixSize = 4;
				else if (level_prefix >= 15)
					levelSuffixSize = level_prefix - 3;
				else
					levelSuffixSize = suffixLength;

				if (levelSuffixSize != 0) {
					uint32_t level_suffix = 0;
					H264_READ_BITS(level_suffix,
						       levelSuffixSize);
					levelCode += level_suffix;
				}
			}

			if (level_prefix >= 15 && suffixLength == 0)
				levelCode += 15;

			if (level_prefix >= 16)
				levelCode += (1 << (level_prefix - 3)) - 4096;

			if (i == trailing_ones && trailing_ones < 3)
				levelCode += 2;

			if (levelCode % 2 == 0)
				levelVal[i] = (levelCode + 2) >> 1;
			else
				levelVal[i] = (-(int32_t)levelCode - 1) >> 1;

			if (suffixLength == 0)
				suffixLength = 1;

			if (Abs(levelVal[i]) > (3 << (suffixLength - 1)) &&
			    suffixLength < 6) {
				suffixLength++;
			}
		}
	}

	uint32_t zerosLeft = 0;
	uint32_t total_zeros = 0;
	res = h264_read_total_zeros(
		bs, total_coeff, endIdx - startIdx + 1, &total_zeros);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	zerosLeft = total_zeros;

	for (uint32_t i = 0; i < total_coeff - 1; i++) {
		uint32_t run_before = 0;
		res = h264_read_run_before(bs, zerosLeft, &run_before);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		runVal[i] = run_before;
		ULOG_ERRNO_RETURN_ERR_IF(run_before > zerosLeft, EIO);
		zerosLeft = zerosLeft - runVal[i];
	}

	runVal[total_coeff - 1] = zerosLeft;
	int32_t coeffNum = -1;
	for (int32_t i = total_coeff - 1; i >= 0; i--) {
		coeffNum += runVal[i] + 1;
		coeffLevel[startIdx + coeffNum] = levelVal[i];
		/* clang-format off */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP
		char field[32] = "";
		snprintf(field,
			 sizeof(field),
			 "%s(%d,%d)",
			 name,
			 blkIdx,
			 startIdx + coeffNum);
		H264_FIELD_S(field, levelVal[i]);
#endif
		/* clang-format on */
	}

	return 0;
}


/* clang-format off */

static int H264_SYNTAX_FCT(residual_luma)(struct h264_bitstream *bs,
					  struct h264_ctx *ctx,
					  struct h264_macroblock *mb,
					  int16_t i16x16DClevel[16],
					  int16_t i16x16AClevel[16][15],
					  int16_t level4x4[16][16],
					  int16_t level8x8[4][64],
					  uint32_t startIdx,
					  uint32_t endIdx,
					  uint32_t comp)
{
	int res = 0;
	static const uint32_t modes[][3] = {
		[Intra16x16DCLevel] = {
			[Luma] = Intra16x16DCLevel,
			[Cb] = CbIntra16x16DCLevel,
			[Cr] = CrIntra16x16DCLevel,
		},
		[Intra16x16ACLevel] = {
			[Luma] = Intra16x16ACLevel,
			[Cb] = CbIntra16x16ACLevel,
			[Cr] = CrIntra16x16ACLevel,
		},
		[LumaLevel4x4] = {
			[Luma] = LumaLevel4x4,
			[Cb] = CbLevel4x4,
			[Cr] = CrLevel4x4,
		},
	};

	if (startIdx == 0 && mb->MbPartPredMode[0] == PredMode_Intra_16x16) {
		res = H264_SYNTAX_FCT(residual_block(bs, ctx, mb,
				i16x16DClevel,
				0,
				15,
				16,
				modes[Intra16x16DCLevel][comp],
				comp,
				0));
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	for (uint32_t i8x8 = 0; i8x8 < 4; i8x8++) {
		for (uint32_t i4x4 = 0; i4x4 < 4; i4x4++) {
			if (mb->CodedBlockPatternLuma & (1 << i8x8)) {
				if (mb->MbPartPredMode[0] == PredMode_Intra_16x16) {
					res = H264_SYNTAX_FCT(residual_block(
							bs, ctx, mb,
							i16x16AClevel[i8x8 * 4 + i4x4],
							startIdx > 0 ? startIdx - 1 : 0,
							endIdx - 1,
							15,
							modes[Intra16x16ACLevel][comp],
							comp,
							i8x8 * 4 + i4x4));
					ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
				} else {
					res = H264_SYNTAX_FCT(residual_block(
							bs, ctx, mb,
							level4x4[i8x8 * 4 + i4x4],
							startIdx,
							endIdx,
							16,
							modes[LumaLevel4x4][comp],
							comp,
							i8x8 * 4 + i4x4));
					ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
				}
			} else if (mb->MbPartPredMode[0] == PredMode_Intra_16x16) {
				for (uint32_t i = 0; i < 15; i++)
					i16x16AClevel[i8x8 * 4 + i4x4][i] = 0;
			} else {
				for (uint32_t i = 0; i < 16; i++)
					level4x4[i8x8 * 4 + i4x4][i] = 0;
			}

			if (mb->transform_size_8x8_flag) {
				for (uint32_t i = 0; i < 16; i++)
					level8x8[i8x8][4 * i + i4x4] = level4x4[i8x8 * 4 + i4x4][i];
			}
		}
	}

	return 0;
}


static int H264_SYNTAX_FCT(residual)(struct h264_bitstream *bs,
				     struct h264_ctx *ctx,
				     struct h264_macroblock *mb,
				     uint32_t startIdx,
				     uint32_t endIdx)
{
	int res = 0;

	res = H264_SYNTAX_FCT(residual_luma(bs, ctx, mb,
			mb->Intra16x16DCLevel,
			mb->Intra16x16ACLevel,
			mb->LumaLevel4x4,
			mb->LumaLevel8x8,
			startIdx,
			endIdx,
			Luma));
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (ctx->sps_derived.ChromaArrayType == 1 ||
			ctx->sps_derived.ChromaArrayType == 2) {
		uint32_t NumC8x8 = 4 / (ctx->sps_derived.SubWidthC *
				ctx->sps_derived.SubHeightC);
		for (uint32_t iCbCr = 0; iCbCr < 2; iCbCr++) {
			if ((mb->CodedBlockPatternChroma & 3) && startIdx == 0) {
				res = H264_SYNTAX_FCT(residual_block(
						bs, ctx, mb,
						mb->ChromaDCLevel[iCbCr],
						0,
						4 * NumC8x8 - 1,
						4 * NumC8x8,
						ChromaDCLevel,
						iCbCr == 0 ? Cb : Cr,
						0));
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			} else {
				for (uint32_t i = 0; i < 4 * NumC8x8; i++)
					mb->ChromaDCLevel[iCbCr][i] = 0;
			}
		}

		for (uint32_t iCbCr = 0; iCbCr < 2; iCbCr++) {
			for (uint32_t i8x8 = 0; i8x8 < NumC8x8; i8x8++) {
				for (uint32_t i4x4 = 0; i4x4 < 4; i4x4++) {
					if (mb->CodedBlockPatternChroma & 2) {
						res = H264_SYNTAX_FCT(residual_block(
								bs, ctx, mb,
								mb->ChromaACLevel[iCbCr][i8x8 * 4 + i4x4],
								startIdx > 0 ? startIdx - 1 : 0,
								endIdx - 1,
								15,
								ChromaACLevel,
								iCbCr == 0 ? Cb : Cr,
								i8x8 * 4 + i4x4));
						ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
					} else {
						/* codecheck_ignore_file[DEEP_INDENTATION] */
						for (uint32_t i = 0; i < 15; i++)
							mb->ChromaACLevel[iCbCr][i8x8 * 4 + i4x4][i] = 0;
					}
				}
			}
		}
	} else if (ctx->sps_derived.ChromaArrayType == 3) {
		res = H264_SYNTAX_FCT(residual_luma(bs, ctx, mb,
				mb->CbIntra16x16DCLevel,
				mb->CbIntra16x16ACLevel,
				mb->CbLevel4x4,
				mb->CbLevel8x8,
				startIdx,
				endIdx,
				Cb));
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = H264_SYNTAX_FCT(residual_luma(bs, ctx, mb,
				mb->CrIntra16x16DCLevel,
				mb->CrIntra16x16ACLevel,
				mb->CrLevel4x4,
				mb->CrLevel8x8,
				startIdx,
				endIdx,
				Cr));
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	return 0;
}


static int H264_SYNTAX_FCT(sub_mb_pred)(struct h264_bitstream *bs,
					struct h264_ctx *ctx,
					struct h264_macroblock *mb)
{
	int res = 0;
	struct h264_slice_header *sh = &ctx->slice.hdr;

	res = h264_read_sub_mb_type(bs, ctx, mb);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	H264_BEGIN_ARRAY(sub_mb_type);
	for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++)
		H264_FIELD(sub_mb_type, mb->raw_sub_mb_type[mbPartIdx]);
	H264_END_ARRAY(sub_mb_type);

	if ((sh->num_ref_idx_l0_active_minus1 > 0 ||
			mb->mb_field_decoding_flag != sh->field_pic_flag) &&
			mb->mb_type != H264_MB_TYPE_P_8x8ref0) {
		H264_BEGIN_ARRAY(ref_idx_l0);
		for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
			if (mb->sub_mb_type[mbPartIdx] != SubMbType_B_Direct_8x8 &&
					mb->SubMbPredMode[mbPartIdx] != PredMode_Pred_L1) {
				H264_BITS_TE(mb->ref_idx_l0[mbPartIdx], mb->max_ref_idx_0);
			} else {
				H264_FIELD(ref_idx_l0, 0);
			}
		}
		H264_END_ARRAY(ref_idx_l0);
	}

	if ((sh->num_ref_idx_l1_active_minus1 > 0 ||
			mb->mb_field_decoding_flag != sh->field_pic_flag)) {
		H264_BEGIN_ARRAY(ref_idx_l1);
		for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
			if (mb->sub_mb_type[mbPartIdx] != SubMbType_B_Direct_8x8 &&
					mb->SubMbPredMode[mbPartIdx] != PredMode_Pred_L0) {
				H264_BITS_TE(mb->ref_idx_l1[mbPartIdx], mb->max_ref_idx_1);
			} else {
				H264_FIELD(ref_idx_l1, 0);
			}
		}
		H264_END_ARRAY(ref_idx_l1);
	}

	H264_BEGIN_ARRAY(mvd_l0);
	for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
		H264_BEGIN_ARRAY(mvd_l0[mbPartIdx]);
		if (mb->sub_mb_type[mbPartIdx] != SubMbType_B_Direct_8x8 &&
				mb->SubMbPredMode[mbPartIdx] != PredMode_Pred_L1) {
			for (uint32_t subMbPartIdx = 0;
					subMbPartIdx < mb->NumSubMbPart[mbPartIdx];
					subMbPartIdx++) {
				H264_BEGIN_ARRAY(mvd_l0[mbPartIdx][subMbPartIdx]);
				for (uint32_t compIdx = 0; compIdx < 2; compIdx++)
					H264_BITS_SE(mb->mvd_l0[mbPartIdx][subMbPartIdx][compIdx]);
				H264_END_ARRAY(mvd_l0[mbPartIdx][subMbPartIdx]);
			}
		}
		H264_END_ARRAY(mvd_l0[mbPartIdx]);
	}
	H264_END_ARRAY(mvd_l0);

	H264_BEGIN_ARRAY(mvd_l1);
	for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
		H264_BEGIN_ARRAY(mvd_l1[mbPartIdx]);
		if (mb->sub_mb_type[mbPartIdx] != SubMbType_B_Direct_8x8 &&
				mb->SubMbPredMode[mbPartIdx] != PredMode_Pred_L0) {
			for (uint32_t subMbPartIdx = 0;
					subMbPartIdx < mb->NumSubMbPart[mbPartIdx];
					subMbPartIdx++) {
				H264_BEGIN_ARRAY(mvd_l1[mbPartIdx][subMbPartIdx]);
				for (uint32_t compIdx = 0; compIdx < 2; compIdx++)
					H264_BITS_SE(mb->mvd_l1[mbPartIdx][subMbPartIdx][compIdx]);
				H264_END_ARRAY(mvd_l1[mbPartIdx][subMbPartIdx]);
			}
		}
		H264_END_ARRAY(mvd_l1[mbPartIdx]);
	}
	H264_END_ARRAY(mvd_l1);

	return 0;
}


static int H264_SYNTAX_FCT(mb_pred)(struct h264_bitstream *bs,
				    struct h264_ctx *ctx,
				    struct h264_macroblock *mb)
{
	struct h264_slice_header *sh = &ctx->slice.hdr;
	uint32_t pred_mode_flag = 0;

	if (mb->MbPartPredMode[0] == PredMode_Intra_4x4 ||
			mb->MbPartPredMode[0] == PredMode_Intra_8x8 ||
			mb->MbPartPredMode[0] == PredMode_Intra_16x16) {
		if (mb->MbPartPredMode[0] == PredMode_Intra_4x4) {
			H264_BEGIN_ARRAY(intra4x4_pred_mode);
			for (uint32_t luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; luma4x4BlkIdx++) {
				H264_READ_BITS(pred_mode_flag, 1);
				if (!pred_mode_flag)
					H264_READ_BITS(mb->intra4x4_pred_mode[luma4x4BlkIdx], 3);
				else
					mb->intra4x4_pred_mode[luma4x4BlkIdx] = -1;
				H264_FIELD(pred_mode, mb->intra4x4_pred_mode[luma4x4BlkIdx]);
			}
			H264_END_ARRAY(intra4x4_pred_mode);
		}

		if (mb->MbPartPredMode[0] == PredMode_Intra_8x8) {
			H264_BEGIN_ARRAY(intra8x8_pred_mode);
			for (uint32_t luma8x8BlkIdx = 0; luma8x8BlkIdx < 4; luma8x8BlkIdx++) {
				H264_READ_BITS(pred_mode_flag, 1);
				if (!pred_mode_flag)
					H264_READ_BITS(mb->intra8x8_pred_mode[luma8x8BlkIdx], 3);
				else
					mb->intra8x8_pred_mode[luma8x8BlkIdx] = -1;
				H264_FIELD(pred_mode, mb->intra8x8_pred_mode[luma8x8BlkIdx]);
			}
			H264_END_ARRAY(intra8x8_pred_mode);
		}

		if (ctx->sps_derived.ChromaArrayType == 1 ||
				ctx->sps_derived.ChromaArrayType == 2) {
			/* TODO: set in mb table */
			H264_BITS_UE(mb->intra_chroma_pred_mode);
		}
	} else if (mb->MbPartPredMode[0] != PredMode_Direct) {
		if (sh->num_ref_idx_l0_active_minus1 > 0 ||
				mb->mb_field_decoding_flag != sh->field_pic_flag) {
			H264_BEGIN_ARRAY(ref_idx_l0);
			for (uint32_t mbPartIdx = 0; mbPartIdx < mb->NumMbPart; mbPartIdx++) {
				if (mb->MbPartPredMode[mbPartIdx] != PredMode_Pred_L1)
					H264_BITS_TE(mb->ref_idx_l0[mbPartIdx], mb->max_ref_idx_0);
				else
					H264_FIELD(ref_idx_l0, 0);
			}
			H264_END_ARRAY(ref_idx_l0);
		}

		if (sh->num_ref_idx_l1_active_minus1 > 0 ||
				mb->mb_field_decoding_flag != sh->field_pic_flag) {
			H264_BEGIN_ARRAY(ref_idx_l1);
			for (uint32_t mbPartIdx = 0; mbPartIdx < mb->NumMbPart; mbPartIdx++) {
				if (mb->MbPartPredMode[mbPartIdx] != PredMode_Pred_L0)
					H264_BITS_TE(mb->ref_idx_l1[mbPartIdx], mb->max_ref_idx_1);
				else
					H264_FIELD(ref_idx_l1, 0);
			}
			H264_END_ARRAY(ref_idx_l1);
		}

		H264_BEGIN_ARRAY(mvd_l0);
		for (uint32_t mbPartIdx = 0; mbPartIdx < mb->NumMbPart; mbPartIdx++) {
			H264_BEGIN_ARRAY(mvd_l0[mbPartIdx]);
			if (mb->MbPartPredMode[mbPartIdx] != PredMode_Pred_L1) {
				H264_BEGIN_ARRAY(mvd_l0[mbPartIdx][0]);
				for (uint32_t compIdx = 0; compIdx < 2; compIdx++)
					H264_BITS_SE(mb->mvd_l0[mbPartIdx][0][compIdx]);
				H264_END_ARRAY(mvd_l0[mbPartIdx][0]);
			}
			H264_END_ARRAY(mvd_l0[mbPartIdx]);
		}
		H264_END_ARRAY(mvd_l0);

		H264_BEGIN_ARRAY(mvd_l1);
		for (uint32_t mbPartIdx = 0; mbPartIdx < mb->NumMbPart; mbPartIdx++) {
			H264_BEGIN_ARRAY(mvd_l1[mbPartIdx]);
			if (mb->MbPartPredMode[mbPartIdx] != PredMode_Pred_L0) {
				H264_BEGIN_ARRAY(mvd_l1[mbPartIdx][0]);
				for (uint32_t compIdx = 0; compIdx < 2; compIdx++)
					H264_BITS_SE(mb->mvd_l1[mbPartIdx][0][compIdx]);

				H264_END_ARRAY(mvd_l1[mbPartIdx][0]);
			}
			H264_END_ARRAY(mvd_l1[mbPartIdx]);
		}
		H264_END_ARRAY(mvd_l1);
	}

	return 0;
}


static int H264_SYNTAX_FCT(macroblock_layer)(struct h264_bitstream *bs,
					     struct h264_ctx *ctx,
					     struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t i = 0;

	int transform_8x8_mode_flag = ctx->pps->transform_8x8_mode_flag;
	int direct_8x8_inference_flag = ctx->sps->direct_8x8_inference_flag;

	int pcm_alignment_zero_bit = 0;
	int noSubMbPartSizeLessThan8x8Flag = 0;

	res = h264_read_mb_type(bs, ctx, mb);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	H264_FIELD(mb_addr, ctx->slice.hdr.frame_num * 10000 + mb->mbAddr);
	H264_FIELD(mb_type, mb->raw_mb_type);

	if (mb->mb_type == H264_MB_TYPE_I_PCM) {
		while (!h264_bs_byte_aligned(bs)) {
			H264_READ_BITS(pcm_alignment_zero_bit, 1);
			ULOG_ERRNO_RETURN_ERR_IF(pcm_alignment_zero_bit != 0, EIO);
		}

		H264_BEGIN_ARRAY(pcm_sample_luma);
		for (i = 0; i < 256; i++)
			H264_BITS(mb->pcm_sample_luma[i], ctx->sps_derived.BitDepthLuma);
		H264_END_ARRAY(pcm_sample_luma);

		H264_BEGIN_ARRAY(pcm_sample_chroma);
		for (uint32_t iCbCr = 0; iCbCr < 2; iCbCr++) {
			H264_BEGIN_ARRAY(pcm_sample_chroma[iCbCr]);
			for (i = 0; i < ctx->sps_derived.MbWidthC * ctx->sps_derived.MbHeightC; i++)
				H264_BITS(mb->pcm_sample_chroma[iCbCr][i], ctx->sps_derived.BitDepthChroma);
			H264_END_ARRAY(pcm_sample_chroma[iCbCr]);
		}
		H264_END_ARRAY(pcm_sample_chroma);

		for (uint32_t comp = 0; comp < 3; comp++) {
			for (uint32_t blkIdx = 0; blkIdx < 16; blkIdx++)
				h264_set_nz_coeff(ctx, mb->mbAddr, comp, blkIdx, 16);
		}
	} else {
		noSubMbPartSizeLessThan8x8Flag = 1;
		if (mb->mb_type != H264_MB_TYPE_I_NxN &&
				mb->MbPartPredMode[0] != PredMode_Intra_16x16 &&
				mb->NumMbPart == 4) {
			res = H264_SYNTAX_FCT(sub_mb_pred(bs, ctx, mb));
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
				if (mb->sub_mb_type[mbPartIdx] != SubMbType_B_Direct_8x8) {
					if (mb->NumSubMbPart[mbPartIdx] > 1)
						noSubMbPartSizeLessThan8x8Flag = 0;
				} else if (!direct_8x8_inference_flag) {
					noSubMbPartSizeLessThan8x8Flag = 0;
				}
			}
		} else {
			if (transform_8x8_mode_flag && mb->mb_type == H264_MB_TYPE_I_NxN) {
				H264_BITS(mb->transform_size_8x8_flag, 1);
				if (mb->transform_size_8x8_flag)
					mb->MbPartPredMode[0] = PredMode_Intra_8x8;
			}
			res = H264_SYNTAX_FCT(mb_pred(bs, ctx, mb));
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}

		if (mb->MbPartPredMode[0] != PredMode_Intra_16x16) {
			res = h264_read_coded_block_pattern(bs, ctx, mb);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			H264_FIELD(coded_block_pattern, mb->coded_block_pattern);
			if (mb->CodedBlockPatternLuma > 0 &&
					transform_8x8_mode_flag &&
					mb->mb_type != H264_MB_TYPE_I_NxN &&
					noSubMbPartSizeLessThan8x8Flag &&
					(mb->mb_type != H264_MB_TYPE_B_Direct_16x16 || direct_8x8_inference_flag)) {
				H264_BITS(mb->transform_size_8x8_flag, 1);
			}
		}

		if (mb->CodedBlockPatternLuma > 0 ||
				mb->CodedBlockPatternChroma > 0 ||
				mb->MbPartPredMode[0] == PredMode_Intra_16x16) {
			H264_BITS_SE(mb->mb_qp_delta);
			H264_BEGIN_STRUCT(residual);
			res = H264_SYNTAX_FCT(residual(bs, ctx, mb, 0, 15));
			H264_END_STRUCT(residual);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}

	return 0;
}

/* clang-format on */


static int H264_SYNTAX_FCT(slice_data_internal)(struct h264_bitstream *bs,
						struct h264_ctx *ctx,
						const struct h264_ctx_cbs *cbs,
						void *userdata)
{
	int res = 0;
	uint32_t i = 0;
	uint32_t CurrMbAddr = 0;
	int prev_mb_skipped = 0;
	uint32_t mb_count = 0;
	struct h264_slice_header *sh = &ctx->slice.hdr;
	uint32_t mb_skip_run = 0;
	int mb_field_decoding_flag = 0;

	/* CABAC not supported for parsing */
	if (ctx->pps->entropy_coding_mode_flag)
		return 0;

	/* Start of slice data, reset MB info table */
	H264_CB(ctx, cbs, userdata, slice_data_begin, &ctx->slice.hdr);
	h264_clear_macroblock_table(ctx);

	h264_gen_slice_group_map(ctx);

	CurrMbAddr = sh->first_mb_in_slice * (1 + ctx->derived.MbaffFrameFlag);
	prev_mb_skipped = 0;
	do {
		if (ctx->slice.type != H264_SLICE_TYPE_I &&
		    ctx->slice.type != H264_SLICE_TYPE_SI) {
			H264_READ_BITS_UE(mb_skip_run);
			prev_mb_skipped = (mb_skip_run > 0);
			H264_BEGIN_ARRAY_ITEM();
			H264_FIELD(mb_skip_run, mb_skip_run);
			H264_END_ARRAY_ITEM();
			for (i = 0; i < mb_skip_run; i++) {
				h264_new_macroblock(ctx, CurrMbAddr, 1, -1);
				H264_CB(ctx,
					cbs,
					userdata,
					slice_data_mb,
					&ctx->slice.hdr,
					ctx->mb->mbAddr,
					ctx->mb->mb_type);
				CurrMbAddr = h264_next_mb_addr(ctx, CurrMbAddr);
				mb_count++;
			}
			if (mb_skip_run > 0 && !h264_bs_more_rbsp_data(bs))
				break;
		}

		H264_BEGIN_ARRAY_ITEM();
		H264_FIELD(mbAddr, CurrMbAddr);
		H264_FIELD(MbaffFrameFlag, ctx->derived.MbaffFrameFlag);

		mb_field_decoding_flag = -1;
		if (ctx->derived.MbaffFrameFlag) {
			if (CurrMbAddr % 2 == 0)
				H264_BITS(mb_field_decoding_flag, 1);
			else if (prev_mb_skipped)
				H264_BITS(mb_field_decoding_flag, 1);
		}

		h264_new_macroblock(ctx, CurrMbAddr, 0, mb_field_decoding_flag);
		res = H264_SYNTAX_FCT(macroblock_layer(bs, ctx, ctx->mb));

		H264_END_ARRAY_ITEM();

		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		H264_CB(ctx,
			cbs,
			userdata,
			slice_data_mb,
			&ctx->slice.hdr,
			ctx->mb->mbAddr,
			ctx->mb->mb_type);

		CurrMbAddr = h264_next_mb_addr(ctx, CurrMbAddr);
		mb_count++;

	} while (h264_bs_more_rbsp_data(bs));

	/* End of slice data */
	H264_CB(ctx, cbs, userdata, slice_data_end, &ctx->slice.hdr, mb_count);

	return 0;
}

#endif /* H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ ||                   \
		H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP */


static int H264_SYNTAX_FCT(slice_data)(struct h264_bitstream *bs,
				       struct h264_ctx *ctx,
				       const struct h264_ctx_cbs *cbs,
				       void *userdata)
{
	int res = 0;

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ

	/* Save raw information about data, then parse if needed */
	ctx->slice.rawdata.partial = bs->cache;
	ctx->slice.rawdata.partialbits = bs->cachebits;
	ctx->slice.rawdata.buf = bs->cdata + bs->off;
	ctx->slice.rawdata.len = bs->len - bs->off;
	if ((H264_READ_FLAGS() & H264_READER_FLAGS_SLICE_DATA) != 0) {
		res = H264_SYNTAX_FCT(slice_data_internal)(
			bs, ctx, cbs, userdata);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_WRITE
	if (ctx->slice.rawdata.partialbits != 0 ||
	    ctx->slice.rawdata.len != 0) {
		/* Write partial bits, must be byte aligned after that */
		if (ctx->slice.rawdata.partialbits != 0) {
			H264_BITS(ctx->slice.rawdata.partial,
				  ctx->slice.rawdata.partialbits);
		}
		ULOG_ERRNO_RETURN_ERR_IF(!h264_bs_byte_aligned(bs), EIO);

		/* Write raw data */
		ULOG_ERRNO_RETURN_ERR_IF(ctx->slice.rawdata.len != 0 &&
						 ctx->slice.rawdata.buf == NULL,
					 EIO);
		res = h264_bs_write_raw_bytes(
			bs, ctx->slice.rawdata.buf, ctx->slice.rawdata.len);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP

	/* Re-construct a bitstream to re-parse data for dump */
	struct h264_bitstream bs_dump;
	if ((H264_DUMP_FLAGS() & H264_DUMP_FLAGS_SLICE_DATA) != 0 &&
	    (ctx->slice.rawdata.partialbits != 0 ||
	     ctx->slice.rawdata.len != 0)) {
		ULOG_ERRNO_RETURN_ERR_IF(ctx->slice.rawdata.len != 0 &&
						 ctx->slice.rawdata.buf == NULL,
					 EIO);
		h264_bs_cinit(&bs_dump,
			      ctx->slice.rawdata.buf,
			      ctx->slice.rawdata.len,
			      bs->emulation_prevention);
		bs_dump.cache = ctx->slice.rawdata.partial;
		bs_dump.cachebits = ctx->slice.rawdata.partialbits;
		bs_dump.priv = bs->priv;
		H264_BEGIN_STRUCT(slice_data);
		H264_BEGIN_ARRAY(mb);
		res = H264_SYNTAX_FCT(slice_data_internal)(
			&bs_dump, ctx, cbs, userdata);
		H264_END_ARRAY(mb);
		H264_END_STRUCT(slice_data);
		h264_bs_clear(&bs_dump);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

#endif

	return 0;
}


#endif /* !_H264_SYNTAX_SLICE_DATA_H_ */
