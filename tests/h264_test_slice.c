/**
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "h264_test.h"

/*
 * Full grey-I / skipped-P slice round trip: h264_write_grey_i_slice()/
 * h264_write_skipped_p_slice() (src/h264_writer.c) synthesize a whole,
 * syntactically valid CAVLC slice (fixed content, independent of any real
 * pixel data), then h264_reader_parse() with H264_READER_FLAGS_SLICE_DATA
 * re-parses the macroblock loop back. This is the exact recipe already
 * proven to work in packages/libvideo-streaming/tests/vstrm_test_frame.c
 * (h264_fixture_build_sps/_pps/_slice) -- reused here directly against
 * libh264 itself, without going through vstrm at all. CAVLC is required:
 * the reader's slice-data parsing only supports CAVLC (H264_READER_FLAGS_
 * SLICE_DATA's doc comment: "Parse slice data (CAVLC only)"), so
 * entropy_coding_mode_flag must stay 0.
 */


struct slice_fixture {
	struct h264_ctx *ctx;
	struct h264_sps sps;
	struct h264_pps pps;
};


static int slice_fixture_new(struct slice_fixture *fx,
			     uint32_t mb_width,
			     uint32_t mb_height)
{
	int res;

	memset(fx, 0, sizeof(*fx));

	res = h264_ctx_new(&fx->ctx);
	if (res < 0)
		return res;

	fx->sps.profile_idc = H264_PROFILE_BASELINE;
	fx->sps.chroma_format_idc = 1; /* 4:2:0 */
	fx->sps.log2_max_frame_num_minus4 = 4;
	fx->sps.pic_order_cnt_type = 2;
	fx->sps.max_num_ref_frames = 1;
	fx->sps.pic_width_in_mbs_minus1 = mb_width - 1;
	fx->sps.pic_height_in_map_units_minus1 = mb_height - 1;
	fx->sps.frame_mbs_only_flag = 1;

	fx->pps.pic_parameter_set_id = 0;
	fx->pps.seq_parameter_set_id = 0;
	fx->pps.entropy_coding_mode_flag = 0; /* CAVLC */

	res = h264_ctx_set_sps(fx->ctx, &fx->sps);
	if (res < 0)
		goto error;
	res = h264_ctx_set_pps(fx->ctx, &fx->pps);
	if (res < 0)
		goto error;

	return 0;

error:
	h264_ctx_destroy(fx->ctx);
	fx->ctx = NULL;
	return res;
}


static void slice_fixture_clear(struct slice_fixture *fx)
{
	if (fx->ctx != NULL)
		h264_ctx_destroy(fx->ctx);
	memset(fx, 0, sizeof(*fx));
}


static size_t
slice_fixture_build_sps(struct slice_fixture *fx, uint8_t *buf, size_t cap)
{
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_SPS};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res >= 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


static size_t
slice_fixture_build_pps(struct slice_fixture *fx, uint8_t *buf, size_t cap)
{
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_PPS};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = h264_write_nalu(&bs, fx->ctx);
	if (res >= 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


static size_t slice_fixture_build_slice(struct slice_fixture *fx,
					int is_idr,
					uint32_t mb_count,
					uint8_t *buf,
					size_t cap)
{
	struct h264_nalu_header nh = {0};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	size_t len = 0;
	int res;

	nh.nal_ref_idc = 3;
	nh.nal_unit_type =
		is_idr ? H264_NALU_TYPE_SLICE_IDR : H264_NALU_TYPE_SLICE;

	res = h264_ctx_clear_nalu(fx->ctx);
	if (res < 0)
		return 0;
	res = h264_ctx_set_nalu_header(fx->ctx, &nh);
	if (res < 0)
		return 0;

	sh.first_mb_in_slice = 0;
	sh.slice_type = is_idr ? H264_SLICE_TYPE_I : H264_SLICE_TYPE_P;
	sh.pic_parameter_set_id = fx->pps.pic_parameter_set_id;
	sh.frame_num = 0;
	sh.disable_deblocking_filter_idc = 2;
	res = h264_ctx_set_slice_header(fx->ctx, &sh);
	if (res < 0)
		return 0;

	h264_bs_init(&bs, buf, cap, 1);
	res = is_idr ? h264_write_grey_i_slice(&bs, fx->ctx, mb_count)
		     : h264_write_skipped_p_slice(&bs, fx->ctx, mb_count);
	if (res >= 0)
		len = bs.off;
	h264_bs_clear(&bs);
	return len;
}


struct slice_test_userdata {
	int (*mb_type_predicate)(enum h264_mb_type);
	int nalu_end_count;
	int slice_count;
	int slice_data_begin_count;
	int slice_data_end_count;
	int mb_count;
	int mb_predicate_failures;
	uint32_t slice_data_end_mb_count;
	struct h264_slice_header last_sh;
};


static void slice_on_nalu_end(struct h264_ctx *ctx,
			      enum h264_nalu_type type,
			      const uint8_t *buf,
			      size_t len,
			      const struct h264_nalu_header *nh,
			      void *userdata)
{
	struct slice_test_userdata *u = userdata;
	(void)ctx;
	(void)type;
	(void)buf;
	(void)len;
	(void)nh;
	u->nalu_end_count++;
}


static void slice_on_slice(struct h264_ctx *ctx,
			   const uint8_t *buf,
			   size_t len,
			   const struct h264_slice_header *sh,
			   void *userdata)
{
	struct slice_test_userdata *u = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	u->slice_count++;
	u->last_sh = *sh;
}


static void slice_on_slice_data_begin(struct h264_ctx *ctx,
				      const struct h264_slice_header *sh,
				      void *userdata)
{
	struct slice_test_userdata *u = userdata;
	(void)ctx;
	(void)sh;
	u->slice_data_begin_count++;
}


static void slice_on_slice_data_end(struct h264_ctx *ctx,
				    const struct h264_slice_header *sh,
				    uint32_t mb_count,
				    void *userdata)
{
	struct slice_test_userdata *u = userdata;
	(void)ctx;
	(void)sh;
	u->slice_data_end_count++;
	u->slice_data_end_mb_count = mb_count;
}


static void slice_on_slice_data_mb(struct h264_ctx *ctx,
				   const struct h264_slice_header *sh,
				   uint32_t mb_addr,
				   enum h264_mb_type mb_type,
				   void *userdata)
{
	struct slice_test_userdata *u = userdata;
	(void)ctx;
	(void)sh;
	(void)mb_addr;
	u->mb_count++;
	if (!u->mb_type_predicate(mb_type))
		u->mb_predicate_failures++;
}


static const struct h264_ctx_cbs s_slice_cbs = {
	.nalu_end = &slice_on_nalu_end,
	.slice = &slice_on_slice,
	.slice_data_begin = &slice_on_slice_data_begin,
	.slice_data_end = &slice_on_slice_data_end,
	.slice_data_mb = &slice_on_slice_data_mb,
};


/* Build an Annex-B stream (SPS + PPS + one grey-I or skipped-P slice
 * covering the whole 2x2-macroblock picture) and check the reader's
 * per-macroblock callbacks see exactly what was written. */
static void run_slice_roundtrip(int is_idr)
{
	struct slice_fixture fx;
	uint8_t sps_buf[256];
	uint8_t pps_buf[256];
	uint8_t slice_buf[256];
	size_t sps_len, pps_len, slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	uint8_t start_code[4] = {0x00, 0x00, 0x00, 0x01};
	uint8_t stream[4 + 256 + 4 + 256 + 4 + 256];
	size_t off = 0;
	size_t total;
	struct slice_test_userdata u = {0};
	struct h264_reader *reader = NULL;
	size_t parse_off = 0;
	int res;

	CU_ASSERT_EQUAL_FATAL(slice_fixture_new(&fx, mb_width, mb_height), 0);

	sps_len = slice_fixture_build_sps(&fx, sps_buf, sizeof(sps_buf));
	CU_ASSERT_TRUE_FATAL(sps_len > 0);
	pps_len = slice_fixture_build_pps(&fx, pps_buf, sizeof(pps_buf));
	CU_ASSERT_TRUE_FATAL(pps_len > 0);
	slice_len = slice_fixture_build_slice(
		&fx, is_idr, mb_count, slice_buf, sizeof(slice_buf));
	CU_ASSERT_TRUE_FATAL(slice_len > 0);

	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, sps_buf, sps_len);
	off += sps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, pps_buf, pps_len);
	off += pps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, slice_buf, slice_len);
	off += slice_len;
	total = off;

	u.mb_type_predicate =
		is_idr ? &h264_mb_type_is_intra : &h264_mb_type_is_inter;

	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&s_slice_cbs, &u, &reader), 0);
	res = h264_reader_parse(reader,
				H264_READER_FLAGS_SLICE_DATA,
				stream,
				total,
				&parse_off);
	CU_ASSERT_EQUAL(res, 0);

	/* 3 NALUs (SPS, PPS, slice) fully parsed without error -- a genuine
	 * mid-slice parse failure would return early and skip both the
	 * "slice" callback and this NALU's "nalu_end" (src/h264_syntax.h). */
	CU_ASSERT_EQUAL(u.nalu_end_count, 3);
	CU_ASSERT_EQUAL(u.slice_count, 1);
	CU_ASSERT_EQUAL(u.last_sh.first_mb_in_slice, 0);
	CU_ASSERT_EQUAL(H264_SLICE_TYPE(u.last_sh.slice_type),
			is_idr ? H264_SLICE_TYPE_I : H264_SLICE_TYPE_P);

	CU_ASSERT_EQUAL(u.slice_data_begin_count, 1);
	CU_ASSERT_EQUAL(u.slice_data_end_count, 1);
	CU_ASSERT_EQUAL(u.slice_data_end_mb_count, mb_count);
	CU_ASSERT_EQUAL(u.mb_count, mb_count);
	CU_ASSERT_EQUAL(u.mb_predicate_failures, 0);

	h264_reader_destroy(reader);
	slice_fixture_clear(&fx);
}


static void test_slice_grey_i_roundtrip(void)
{
	run_slice_roundtrip(1);
}


static void test_slice_skipped_p_roundtrip(void)
{
	run_slice_roundtrip(0);
}


/*
 * h264_rewrite_slice_header() (src/h264_writer.c): patches just the slice
 * header of an already-written slice buffer in place, as long as the new
 * header serializes to the exact same bit length as the original (it
 * compares ctx->slice.hdr_len before/after) -- otherwise -EPROTO. No caller
 * exists anywhere else in the workspace, so this is exercised purely
 * synthetically here, reusing the same slice_fixture as the round trips
 * above.
 */


/* Changing only frame_num (a fixed-width u(v) field, log2_max_frame_num_
 * minus4 + 4 = 8 bits here) can never change the header's bit length. */
static void test_rewrite_slice_header_success(void)
{
	struct slice_fixture fx;
	uint8_t sps_buf[256];
	uint8_t pps_buf[256];
	uint8_t slice_buf[256];
	size_t sps_len, pps_len, slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	struct h264_nalu_header nh = {
		.nal_ref_idc = 3, .nal_unit_type = H264_NALU_TYPE_SLICE_IDR};
	struct h264_slice_header sh = {0};
	struct h264_slice_header new_sh;
	struct h264_bitstream bs;
	uint8_t start_code[4] = {0x00, 0x00, 0x00, 0x01};
	uint8_t stream[4 + 256 + 4 + 256 + 4 + 256];
	size_t off = 0;
	struct slice_test_userdata u = {0};
	struct h264_reader *reader = NULL;
	size_t parse_off = 0;
	int res;

	CU_ASSERT_EQUAL_FATAL(slice_fixture_new(&fx, mb_width, mb_height), 0);
	sps_len = slice_fixture_build_sps(&fx, sps_buf, sizeof(sps_buf));
	CU_ASSERT_TRUE_FATAL(sps_len > 0);
	pps_len = slice_fixture_build_pps(&fx, pps_buf, sizeof(pps_buf));
	CU_ASSERT_TRUE_FATAL(pps_len > 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(fx.ctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(fx.ctx, &nh), 0);
	sh.first_mb_in_slice = 0;
	sh.slice_type = H264_SLICE_TYPE_I;
	sh.pic_parameter_set_id = fx.pps.pic_parameter_set_id;
	sh.frame_num = 3;
	sh.disable_deblocking_filter_idc = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);

	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_write_grey_i_slice(&bs, fx.ctx, mb_count);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	slice_len = bs.off;

	new_sh = sh;
	new_sh.frame_num = 9;
	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_rewrite_slice_header(&bs, fx.ctx, &new_sh);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	/* Re-parse the patched buffer end-to-end: frame_num must reflect the
	 * rewrite, and the macroblock data -- untouched by the rewrite --
	 * must still parse exactly as before. */
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, sps_buf, sps_len);
	off += sps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, pps_buf, pps_len);
	off += pps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, slice_buf, slice_len);
	off += slice_len;

	u.mb_type_predicate = &h264_mb_type_is_intra;
	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&s_slice_cbs, &u, &reader), 0);
	res = h264_reader_parse(
		reader, H264_READER_FLAGS_SLICE_DATA, stream, off, &parse_off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.slice_count, 1);
	CU_ASSERT_EQUAL(u.last_sh.frame_num, 9);
	CU_ASSERT_EQUAL(u.mb_count, mb_count);
	CU_ASSERT_EQUAL(u.mb_predicate_failures, 0);

	h264_reader_destroy(reader);
	slice_fixture_clear(&fx);
}


/* Changing first_mb_in_slice from 0 (ue(v) code "1", 1 bit) to 1 (ue(v) code
 * "010", 3 bits) changes the header's bit length -- must be rejected, and
 * the buffer must be left untouched. */
static void test_rewrite_slice_header_length_mismatch(void)
{
	struct slice_fixture fx;
	uint8_t tmp_buf[256];
	uint8_t slice_buf[256];
	uint8_t slice_buf_copy[256];
	size_t slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	struct h264_nalu_header nh = {
		.nal_ref_idc = 3, .nal_unit_type = H264_NALU_TYPE_SLICE_IDR};
	struct h264_slice_header sh = {0};
	struct h264_slice_header new_sh;
	struct h264_bitstream bs;
	int res;

	CU_ASSERT_EQUAL_FATAL(slice_fixture_new(&fx, mb_width, mb_height), 0);
	CU_ASSERT_TRUE_FATAL(
		slice_fixture_build_sps(&fx, tmp_buf, sizeof(tmp_buf)) > 0);
	CU_ASSERT_TRUE_FATAL(
		slice_fixture_build_pps(&fx, tmp_buf, sizeof(tmp_buf)) > 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(fx.ctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(fx.ctx, &nh), 0);
	sh.first_mb_in_slice = 0;
	sh.slice_type = H264_SLICE_TYPE_I;
	sh.pic_parameter_set_id = fx.pps.pic_parameter_set_id;
	sh.frame_num = 0;
	sh.disable_deblocking_filter_idc = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);

	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_write_grey_i_slice(&bs, fx.ctx, mb_count);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	slice_len = bs.off;
	memcpy(slice_buf_copy, slice_buf, slice_len);

	new_sh = sh;
	new_sh.first_mb_in_slice = 1;
	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_rewrite_slice_header(&bs, fx.ctx, &new_sh);
	CU_ASSERT_EQUAL(res, -EPROTO);

	CU_ASSERT_EQUAL(memcmp(slice_buf, slice_buf_copy, slice_len), 0);

	slice_fixture_clear(&fx);
}


/*
 * pred_weight_table() (7.3.3.2, src/h264_syntax.h) and explicit
 * dec_ref_pic_marking() (7.3.3.3, MMCO) round trips: x264 (the only real
 * encoder available for generating test assets here) never emits either --
 * it defaults to implicit weighted prediction and a sliding-window DPB, and
 * Baseline profile forbids weighted prediction outright -- so these are
 * only reachable synthetically, the same way FMO and h264_rewrite_slice_
 * header were.
 */


/* Explicit weighted P-slice prediction: pps->weighted_pred_flag = 1, one
 * active L0 reference with luma+chroma weights/offsets set. */
static void test_slice_pred_weight_table_roundtrip(void)
{
	struct slice_fixture fx;
	uint8_t sps_buf[256];
	uint8_t pps_buf[256];
	uint8_t slice_buf[256];
	size_t sps_len, pps_len, slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_SLICE};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	uint8_t start_code[4] = {0x00, 0x00, 0x00, 0x01};
	uint8_t stream[4 + 256 + 4 + 256 + 4 + 256];
	size_t off = 0;
	struct slice_test_userdata u = {0};
	struct h264_reader *reader = NULL;
	size_t parse_off = 0;
	int res;

	CU_ASSERT_EQUAL_FATAL(slice_fixture_new(&fx, mb_width, mb_height), 0);
	fx.pps.weighted_pred_flag = 1;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_pps(fx.ctx, &fx.pps), 0);

	sps_len = slice_fixture_build_sps(&fx, sps_buf, sizeof(sps_buf));
	CU_ASSERT_TRUE_FATAL(sps_len > 0);
	pps_len = slice_fixture_build_pps(&fx, pps_buf, sizeof(pps_buf));
	CU_ASSERT_TRUE_FATAL(pps_len > 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(fx.ctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(fx.ctx, &nh), 0);
	sh.first_mb_in_slice = 0;
	sh.slice_type = H264_SLICE_TYPE_P;
	sh.pic_parameter_set_id = fx.pps.pic_parameter_set_id;
	sh.frame_num = 0;
	sh.disable_deblocking_filter_idc = 2;
	sh.num_ref_idx_active_override_flag = 1;
	sh.num_ref_idx_l0_active_minus1 = 0;
	sh.pwt.luma_log2_weight_denom = 5;
	sh.pwt.chroma_log2_weight_denom = 5;
	sh.pwt.l0[0].luma_weight_flag = 1;
	sh.pwt.l0[0].luma_weight = 40;
	sh.pwt.l0[0].luma_offset = -3;
	sh.pwt.l0[0].chroma_weight_flag = 1;
	sh.pwt.l0[0].chroma_weight[0] = 32;
	sh.pwt.l0[0].chroma_offset[0] = 1;
	sh.pwt.l0[0].chroma_weight[1] = 30;
	sh.pwt.l0[0].chroma_offset[1] = -2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);

	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_write_skipped_p_slice(&bs, fx.ctx, mb_count);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	slice_len = bs.off;

	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, sps_buf, sps_len);
	off += sps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, pps_buf, pps_len);
	off += pps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, slice_buf, slice_len);
	off += slice_len;

	u.mb_type_predicate = &h264_mb_type_is_inter;
	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&s_slice_cbs, &u, &reader), 0);
	res = h264_reader_parse(
		reader, H264_READER_FLAGS_SLICE_DATA, stream, off, &parse_off);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(u.slice_count, 1);
	CU_ASSERT_EQUAL(u.last_sh.pwt.luma_log2_weight_denom, 5);
	CU_ASSERT_EQUAL(u.last_sh.pwt.chroma_log2_weight_denom, 5);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].luma_weight, 40);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].luma_offset, -3);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].chroma_weight[0], 32);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].chroma_offset[0], 1);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].chroma_weight[1], 30);
	CU_ASSERT_EQUAL(u.last_sh.pwt.l0[0].chroma_offset[1], -2);
	/* The macroblock data right after the weight table must still land
	 * on a byte-consistent boundary -- confirms the pwt bit count
	 * matched exactly between write and read. */
	CU_ASSERT_EQUAL(u.mb_count, mb_count);
	CU_ASSERT_EQUAL(u.mb_predicate_failures, 0);

	h264_reader_destroy(reader);
	slice_fixture_clear(&fx);
}


/* Explicit adaptive reference picture marking (MMCO): a non-IDR reference
 * slice (nal_ref_idc != 0) with adaptive_ref_pic_marking_mode_flag = 1 and
 * one MMCO command (op 1: "mark a short-term picture as unused") followed
 * by the mandatory op-0 terminator. */
static void test_slice_explicit_mmco_roundtrip(void)
{
	struct slice_fixture fx;
	uint8_t sps_buf[256];
	uint8_t pps_buf[256];
	uint8_t slice_buf[256];
	size_t sps_len, pps_len, slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	struct h264_nalu_header nh = {.nal_ref_idc = 3,
				      .nal_unit_type = H264_NALU_TYPE_SLICE};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	uint8_t start_code[4] = {0x00, 0x00, 0x00, 0x01};
	uint8_t stream[4 + 256 + 4 + 256 + 4 + 256];
	size_t off = 0;
	struct slice_test_userdata u = {0};
	struct h264_reader *reader = NULL;
	size_t parse_off = 0;
	int res;

	CU_ASSERT_EQUAL_FATAL(slice_fixture_new(&fx, mb_width, mb_height), 0);
	sps_len = slice_fixture_build_sps(&fx, sps_buf, sizeof(sps_buf));
	CU_ASSERT_TRUE_FATAL(sps_len > 0);
	pps_len = slice_fixture_build_pps(&fx, pps_buf, sizeof(pps_buf));
	CU_ASSERT_TRUE_FATAL(pps_len > 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(fx.ctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(fx.ctx, &nh), 0);
	sh.first_mb_in_slice = 0;
	sh.slice_type = H264_SLICE_TYPE_P;
	sh.pic_parameter_set_id = fx.pps.pic_parameter_set_id;
	sh.frame_num = 1;
	sh.disable_deblocking_filter_idc = 2;
	sh.drpm.adaptive_ref_pic_marking_mode_flag = 1;
	sh.drpm.mm[0].memory_management_control_operation = 1;
	sh.drpm.mm[0].difference_of_pic_nums_minus1 = 0;
	sh.drpm.mm[1].memory_management_control_operation = 0; /* terminator */
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);

	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_write_skipped_p_slice(&bs, fx.ctx, mb_count);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	slice_len = bs.off;

	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, sps_buf, sps_len);
	off += sps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, pps_buf, pps_len);
	off += pps_len;
	memcpy(stream + off, start_code, 4);
	off += 4;
	memcpy(stream + off, slice_buf, slice_len);
	off += slice_len;

	u.mb_type_predicate = &h264_mb_type_is_inter;
	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&s_slice_cbs, &u, &reader), 0);
	res = h264_reader_parse(
		reader, H264_READER_FLAGS_SLICE_DATA, stream, off, &parse_off);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(u.slice_count, 1);
	CU_ASSERT_EQUAL(u.last_sh.drpm.adaptive_ref_pic_marking_mode_flag, 1);
	CU_ASSERT_EQUAL(
		u.last_sh.drpm.mm[0].memory_management_control_operation, 1);
	CU_ASSERT_EQUAL(u.last_sh.drpm.mm[0].difference_of_pic_nums_minus1, 0);
	CU_ASSERT_EQUAL(
		u.last_sh.drpm.mm[1].memory_management_control_operation, 0);
	CU_ASSERT_EQUAL(u.mb_count, mb_count);
	CU_ASSERT_EQUAL(u.mb_predicate_failures, 0);

	h264_reader_destroy(reader);
	slice_fixture_clear(&fx);
}


CU_TestInfo g_h264_test_slice[] = {
	{FN("grey-i-roundtrip"), &test_slice_grey_i_roundtrip},
	{FN("skipped-p-roundtrip"), &test_slice_skipped_p_roundtrip},
	{FN("rewrite-slice-header-success"),
	 &test_rewrite_slice_header_success},
	{FN("rewrite-slice-header-length-mismatch"),
	 &test_rewrite_slice_header_length_mismatch},
	{FN("pred-weight-table-roundtrip"),
	 &test_slice_pred_weight_table_roundtrip},
	{FN("explicit-mmco-roundtrip"), &test_slice_explicit_mmco_roundtrip},

	CU_TEST_INFO_NULL,
};
