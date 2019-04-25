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

#ifndef _H264_MACROBLOCK_H_
#define _H264_MACROBLOCK_H_

#define H264_MB_ADDR_INVALID ((uint32_t)-1)


/**
 * 7.4.5.2 Sub-macroblock prediction semantics
 */
enum SubMbType {
	SubMbType_P_8x8,
	SubMbType_P_8x4,
	SubMbType_P_4x8,
	SubMbType_P_4x4,
	SubMbType_B_Direct_8x8,
	SubMbType_B_8x8,
	SubMbType_B_8x4,
	SubMbType_B_4x8,
	SubMbType_B_4x4,
};


/**
 * 7.4.5 Macroblock layer semantics
 */
enum PredMode {
	PredMode_Intra_4x4,
	PredMode_Intra_8x8,
	PredMode_Intra_16x16,
	PredMode_Pred_L0,
	PredMode_Pred_L1,
	PredMode_BiPred,
	PredMode_Direct,
};


enum Level {
	Intra16x16DCLevel,
	Intra16x16ACLevel,
	CbIntra16x16DCLevel,
	CbIntra16x16ACLevel,
	CrIntra16x16DCLevel,
	CrIntra16x16ACLevel,
	LumaLevel4x4,
	CbLevel4x4,
	CrLevel4x4,
	ChromaDCLevel,
	ChromaACLevel,
};


enum Component {
	Luma,
	Cb,
	Cr,
};


enum IntraChroma {
	IntraChromaDC = 0,
	IntraChromaHorizontal = 1,
	IntraChromaVertical = 2,
	IntraChromaPlane = 3,
};


/* clang-format off */
struct h264_macroblock_info {
	uint32_t mb_type:4;
	uint32_t intra_chroma_pred_mode:2;
	uint32_t available:1;
	uint32_t skipped:1;
	uint32_t field_flag:1;
	uint8_t nz_coeff[3 * 16];
};
/* clang-format on */


struct h264_macroblock {
	uint32_t mbAddr;
	int mb_field_decoding_flag;
	int mb_skip_flag;

	uint32_t mbAddrA;
	uint32_t mbAddrB;
	const struct h264_macroblock_info *mbAddrAInfo;
	const struct h264_macroblock_info *mbAddrBInfo;

	enum h264_mb_type mb_type;
	uint32_t raw_mb_type;
	uint32_t NumMbPart;
	uint32_t MbPartPredMode[2];

	uint32_t raw_sub_mb_type[4];
	uint32_t sub_mb_type[4];
	uint32_t NumSubMbPart[4];
	uint32_t SubMbPredMode[4];

	int transform_size_8x8_flag;
	int32_t mb_qp_delta;

	/* PCM MB only (BitDepthLuma and BitDepthChroma 8-14 bits) */
	uint16_t pcm_sample_luma[256];
	uint16_t pcm_sample_chroma[2][256];

	/* Intra MB only */
	int8_t intra4x4_pred_mode[16];
	int8_t intra8x8_pred_mode[4];
	uint8_t intra_chroma_pred_mode;
	uint8_t Intra16x16PredMode;

	/* Inter MB only */
	uint8_t max_ref_idx_0;
	uint8_t max_ref_idx_1;
	uint8_t ref_idx_l0[4];
	uint8_t ref_idx_l1[4];
	int16_t mvd_l0[4][4][2];
	int16_t mvd_l1[4][4][2];

	/* Residuals */
	uint8_t coded_block_pattern;
	uint8_t CodedBlockPatternLuma;
	uint8_t CodedBlockPatternChroma;

	int16_t Intra16x16DCLevel[16];
	int16_t Intra16x16ACLevel[16][15];
	int16_t LumaLevel4x4[16][16];
	int16_t LumaLevel8x8[4][64];
	int16_t ChromaDCLevel[2][16];
	int16_t ChromaACLevel[2][16][15];

	int16_t CbIntra16x16DCLevel[16];
	int16_t CbIntra16x16ACLevel[16][15];
	int16_t CbLevel4x4[16][16];
	int16_t CbLevel8x8[4][64];

	int16_t CrIntra16x16DCLevel[16];
	int16_t CrIntra16x16ACLevel[16][15];
	int16_t CrLevel4x4[16][16];
	int16_t CrLevel8x8[4][64];
};


void h264_compute_neighbouring_macroblocks(struct h264_ctx *ctx,
					   struct h264_macroblock *mb);


void h264_get_neighbouring_luma_cb_cr_4x4(struct h264_ctx *ctx,
					  struct h264_macroblock *mb,
					  uint32_t idx,
					  uint32_t *mbAddrA,
					  uint32_t *idxA,
					  uint32_t *mbAddrB,
					  uint32_t *idxB);


void h264_get_neighbouring_chroma_4x4(struct h264_ctx *ctx,
				      struct h264_macroblock *mb,
				      uint32_t idx,
				      uint32_t *mbAddrA,
				      uint32_t *idxA,
				      uint32_t *mbAddrB,
				      uint32_t *idxB);


#endif /* _H264_MACROBLOCK_H_ */
