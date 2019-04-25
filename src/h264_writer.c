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


#define H264_SYNTAX_OP_NAME write
#define H264_SYNTAX_OP_KIND H264_SYNTAX_OP_KIND_WRITE

#define H264_BITS(_f, _n) H264_WRITE_BITS(_f, _n)
#define H264_BITS_U(_f, _n) H264_WRITE_BITS_U(_f, _n)
#define H264_BITS_I(_f, _n) H264_WRITE_BITS_I(_f, _n)
#define H264_BITS_UE(_f) H264_WRITE_BITS_UE(_f)
#define H264_BITS_SE(_f) H264_WRITE_BITS_SE(_f)
#define H264_BITS_TE(_f, _m) H264_WRITE_BITS_TE(_f, _m)

#define H264_BITS_RBSP_TRAILING()                                              \
	do {                                                                   \
		int _res = h264_bs_write_rbsp_trailing_bits(bs);               \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#include "h264_syntax.h"


static int h264_setup_grey_i_macroblock(struct h264_ctx *ctx, uint32_t i)
{
	int res = 0;
	struct h264_macroblock *mb = NULL;

	/* Setup macroblock */
	uint32_t mbAddr = ctx->slice.hdr.first_mb_in_slice + i;
	res = h264_new_macroblock(ctx, mbAddr, 0, -1);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	mb = ctx->mb;

	/* I_16x16_2_0_0 */
	mb->raw_mb_type = 3;
	mb->mb_type = H264_MB_TYPE_I_16x16;
	mb->coded_block_pattern = 0;
	mb->CodedBlockPatternLuma = 0;
	mb->CodedBlockPatternChroma = 0;
	mb->Intra16x16PredMode = 0;
	mb->NumMbPart = 1;
	mb->MbPartPredMode[0] = PredMode_Intra_16x16;
	mb->intra_chroma_pred_mode = IntraChromaDC;

	ctx->slice.mb_table.info[i].mb_type = mb->mb_type;
	ctx->slice.mb_table.info[i].intra_chroma_pred_mode =
		mb->intra_chroma_pred_mode;

	return 0;
}


static int h264_write_grey_i_slice_cabac(struct h264_bitstream *bs,
					 struct h264_ctx *ctx,
					 uint32_t mb_count)
{
	int res = 0;
	struct h264_cabac cabac;
	struct h264_macroblock *mb = NULL;

	/* cabac_alignment_one_bit */
	while (!h264_bs_byte_aligned(bs)) {
		res = h264_bs_write_bits(bs, 1, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	/* Initialize cabac encoder */
	memset(&cabac, 0, sizeof(cabac));
	res = h264_cabac_init_enc(&cabac, ctx, bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	for (uint32_t i = 0; i < mb_count; i++) {
		res = h264_setup_grey_i_macroblock(ctx, i);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		mb = ctx->mb;

		res = h264_cabac_write_mb_type(&cabac, ctx, mb);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_cabac_write_intra_chroma_pred_mode(&cabac, ctx, mb);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_cabac_write_mb_qp_delta(&cabac, ctx, mb);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_cabac_write_coded_block_flag(
			&cabac, ctx, mb, Intra16x16DCLevel, 0);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_cabac_write_end_of_slice_flag(
			&cabac, ctx, mb, i == mb_count - 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	/* rbsp_stop_one_bit already written, simply align stream */
	while (!h264_bs_byte_aligned(bs)) {
		/* Write rbsp_alignment_zero_bit */
		res = h264_bs_write_bits(bs, 0, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	return 0;
}


static int h264_write_grey_i_slice_cavlc(struct h264_bitstream *bs,
					 struct h264_ctx *ctx,
					 uint32_t mb_count)
{
	int res = 0;

	for (uint32_t i = 0; i < mb_count; i++) {
		/* mb_type = 3 I_16x16_2_0_0
		 * Intra16x16PredMode = 2/DC
		 * CodedBlockPatternLuma = 0
		 * CodedBlockPatternChroma = 0
		 * -> 5 bits
		 */
		res = h264_bs_write_bits_ue(bs, 3);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		/* mb_pred
		 * intra_chroma_pred_mode = 0 (DC)
		 * -> 1 bit */
		res = h264_bs_write_bits_ue(bs, 0);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		/* mb_qp_delta = 0
		 * -> 1 bit */
		res = h264_bs_write_bits_se(bs, 0);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		/* residual(0, 15)
		 * residual_luma(i16x16DClevel, i16x16AClevel,
		 *               level4x4, level8x8, 0, 15)
		 * residual_block_cavlc(i16x16DClevel, 0, 15, 16)
		 * coeff_token = 1 (nC = 0)
		 * -> 1 bit */
		res = h264_bs_write_bits(bs, 1, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	/* Finish nalu */
	res = h264_bs_write_rbsp_trailing_bits(bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


static int h264_write_skipped_p_slice_cabac(struct h264_bitstream *bs,
					    struct h264_ctx *ctx,
					    uint32_t mb_count)
{
	int res = 0;
	struct h264_cabac cabac;
	struct h264_macroblock *mb = NULL;

	/* cabac_alignment_one_bit */
	while (!h264_bs_byte_aligned(bs)) {
		res = h264_bs_write_bits(bs, 1, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	/* Initialize cabac encoder */
	memset(&cabac, 0, sizeof(cabac));
	res = h264_cabac_init_enc(&cabac, ctx, bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	for (uint32_t i = 0; i < mb_count; i++) {
		/* Setup macroblock */
		uint32_t mbAddr = ctx->slice.hdr.first_mb_in_slice + i;
		res = h264_new_macroblock(ctx, mbAddr, 1, -1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		mb = ctx->mb;

		res = h264_cabac_write_mb_skip_flag(&cabac, ctx, mb, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_cabac_write_end_of_slice_flag(
			&cabac, ctx, mb, i == mb_count - 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	/* rbsp_stop_one_bit already written, simply align stream */
	while (!h264_bs_byte_aligned(bs)) {
		/* Write rbsp_alignment_zero_bit */
		res = h264_bs_write_bits(bs, 0, 1);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	return 0;
}


static int h264_write_skipped_p_slice_cavlc(struct h264_bitstream *bs,
					    struct h264_ctx *ctx,
					    uint32_t mb_count)
{
	int res = 0;

	/* Slice data is simply mb_skip_run */
	res = h264_bs_write_bits_ue(bs, mb_count);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	/* Finish NALU */
	res = h264_bs_write_rbsp_trailing_bits(bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


int h264_write_nalu(struct h264_bitstream *bs, struct h264_ctx *ctx)
{
	return _h264_write_nalu(bs, ctx, NULL, NULL);
}


int h264_write_one_sei(struct h264_bitstream *bs,
		       struct h264_ctx *ctx,
		       const struct h264_sei *sei)
{
	return _h264_write_one_sei(bs, ctx, NULL, NULL, sei);
}


int h264_write_grey_i_slice(struct h264_bitstream *bs,
			    struct h264_ctx *ctx,
			    uint32_t mb_count)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(bs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mb_count == 0, EINVAL);

	/* Make sure SPS/PPS are correctly set */
	res = h264_ctx_set_active_pps(ctx, ctx->slice.hdr.pic_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	/* Write slice header */
	res = h264_write_nalu(bs, ctx);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (ctx->pps->entropy_coding_mode_flag) {
		res = h264_write_grey_i_slice_cabac(bs, ctx, mb_count);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	} else {
		res = h264_write_grey_i_slice_cavlc(bs, ctx, mb_count);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	return 0;
}


int h264_write_skipped_p_slice(struct h264_bitstream *bs,
			       struct h264_ctx *ctx,
			       uint32_t mb_count)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(bs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mb_count == 0, EINVAL);

	/* Make sure SPS/PPS are correctly set */
	res = h264_ctx_set_active_pps(ctx, ctx->slice.hdr.pic_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	/* Write slice header */
	res = h264_write_nalu(bs, ctx);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (ctx->pps->entropy_coding_mode_flag) {
		res = h264_write_skipped_p_slice_cabac(bs, ctx, mb_count);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	} else {
		res = h264_write_skipped_p_slice_cavlc(bs, ctx, mb_count);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	return 0;
}


int h264_rewrite_slice_header(struct h264_bitstream *bs,
			      struct h264_ctx *ctx,
			      const struct h264_slice_header *sh)
{
	int res = 0;
	size_t saved_sh_len;
	struct h264_bitstream tmp_bs;
	uint8_t data[64];

	ULOG_ERRNO_RETURN_ERR_IF(bs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sh == NULL, EINVAL);

	h264_bs_init(&tmp_bs, data, 64, 1);

	/* Save the old slice header */
	ctx->slice.saved_hdr = ctx->slice.hdr;
	saved_sh_len = ctx->slice.hdr_len;

	/* Set the slice header */
	res = h264_ctx_set_slice_header(ctx, sh);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	/* Write slice header */
	res = h264_write_nalu(&tmp_bs, ctx);
	if (res < 0) {
		ULOG_ERRNO("h264_write_nalu", -res);
		goto error;
	}

	/* Make sure the new slice header has the same length */
	if (ctx->slice.hdr_len != saved_sh_len) {
		res = -EPROTO;
		ULOGE("slice header length mismatch (new %zu vs. old %zu)",
		      ctx->slice.hdr_len,
		      saved_sh_len);
		goto error;
	}

	/* Copy the new slice header to the original buffer
	 * (except the last % 8 bits) */
	memcpy(bs->data, tmp_bs.data, tmp_bs.off);

	/* Blend the last % 8 bits of the new slice header
	 * with the first bits of the slice data */
	if (tmp_bs.cachebits != 0) {
		uint8_t mask = (1 << (8 - tmp_bs.cachebits)) - 1;
		bs->data[tmp_bs.off] =
			(tmp_bs.cache & ~mask) | (bs->data[tmp_bs.off] & mask);
	}

	return 0;

error:
	/* Restore the old slice header in case of error */
	ctx->slice.hdr = ctx->slice.saved_hdr;
	ctx->slice.hdr_len = saved_sh_len;
	return res;
}
