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

#include <json-c/json.h>


/*
 * h264_dump_nalu() (src/h264_dump.c) runs the same generated syntax
 * functions as the reader/writer (src/h264_syntax.h), but in DUMP mode every
 * H264_BITS*() macro just reports the field's current in-memory value to
 * JSON (src/h264_syntax_ops.h: H264_DUMP_BITS() expands to a plain field
 * report, it never touches the bitstream) -- so, unlike the reader/writer
 * tests, no NALU byte encoding is needed at all here: a ctx populated
 * directly through the public setters is enough.
 */


static void test_dump_null_args(void)
{
	struct h264_dump *dump = NULL;
	struct h264_ctx *ctx = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *jobj = NULL;
	const char *str = NULL;
	int ret;

	ret = h264_dump_new(NULL, &dump);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = h264_dump_new(&cfg, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = h264_dump_new(&cfg, &dump);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(dump);

	CU_ASSERT_EQUAL(h264_dump_clear(NULL), -EINVAL);

	CU_ASSERT_EQUAL(h264_dump_get_json_object(NULL, &jobj), -EINVAL);
	CU_ASSERT_EQUAL(h264_dump_get_json_object(dump, NULL), -EINVAL);

	CU_ASSERT_EQUAL(h264_dump_get_json_str(NULL, &str), -EINVAL);
	CU_ASSERT_EQUAL(h264_dump_get_json_str(dump, NULL), -EINVAL);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);
	CU_ASSERT_EQUAL(h264_dump_nalu(NULL, ctx, 0), -EINVAL);
	CU_ASSERT_EQUAL(h264_dump_nalu(dump, NULL, 0), -EINVAL);
	h264_ctx_destroy(ctx);

	h264_dump_destroy(dump);
	CU_ASSERT_EQUAL(h264_dump_destroy(NULL), 0);
}


/* Dump a ctx holding a hand-filled SPS (no VUI) and check the JSON mirrors
 * the fields that were set, plus the sibling "nalu_header" struct. */
static void test_dump_sps_no_vui(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_nalu_header nh = {0};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *nalu_header = NULL;
	json_object *jsps = NULL;
	json_object *val = NULL;
	int ret;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	sps.profile_idc = H264_PROFILE_MAIN;
	sps.level_idc = 40;
	sps.chroma_format_idc = 1;
	sps.pic_width_in_mbs_minus1 = 10;
	sps.pic_height_in_map_units_minus1 = 8;
	sps.frame_mbs_only_flag = 1;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_sps(ctx, &sps), 0);

	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_SPS;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	ret = h264_dump_nalu(dump, ctx, 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(top);

	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(top, "nalu_header", &nalu_header));
	CU_ASSERT_TRUE(
		json_object_object_get_ex(nalu_header, "nal_unit_type", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), H264_NALU_TYPE_SPS);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(nalu_header, "nal_ref_idc", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 3);

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "sps", &jsps));
	CU_ASSERT_TRUE(json_object_object_get_ex(jsps, "profile_idc", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), H264_PROFILE_MAIN);
	CU_ASSERT_TRUE(json_object_object_get_ex(jsps, "level_idc", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 40);
	CU_ASSERT_TRUE(json_object_object_get_ex(
		jsps, "pic_width_in_mbs_minus1", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 10);

	/* vui_parameters_present_flag is 0: no nested "vui" object */
	CU_ASSERT_FALSE(json_object_object_get_ex(jsps, "vui", &val));

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


/* Same, but with VUI (timing + aspect ratio) present: exercises the nested
 * "vui" struct dump. */
static void test_dump_sps_with_vui(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_nalu_header nh = {0};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *jsps = NULL;
	json_object *jvui = NULL;
	json_object *val = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	sps.profile_idc = H264_PROFILE_MAIN;
	sps.chroma_format_idc = 1;
	sps.frame_mbs_only_flag = 1;
	sps.vui_parameters_present_flag = 1;
	sps.vui.aspect_ratio_info_present_flag = 1;
	sps.vui.aspect_ratio_idc = H264_ASPECT_RATIO_4_3;
	sps.vui.timing_info_present_flag = 1;
	sps.vui.num_units_in_tick = 1;
	sps.vui.time_scale = 60;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_sps(ctx, &sps), 0);

	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_SPS;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "sps", &jsps));
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(jsps, "vui", &jvui));
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jvui, "aspect_ratio_idc", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), H264_ASPECT_RATIO_4_3);
	CU_ASSERT_TRUE(json_object_object_get_ex(jvui, "time_scale", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 60);
	/* nal_hrd_parameters_present_flag is 0: no nested "nal_hrd" object */
	CU_ASSERT_FALSE(json_object_object_get_ex(jvui, "nal_hrd", &val));

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


static void test_dump_pps(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	struct h264_nalu_header nh = {0};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *jpps = NULL;
	json_object *val = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	/* Dumping a PPS resolves its referenced SPS via seq_parameter_set_id
	 * (_h264_dump_pps_with_ctx() -> h264_ctx_set_active_sps()), so an SPS
	 * with a matching id must be set first, exactly like h264_parse_pps()
	 * requires one to be passed in explicitly. */
	sps.chroma_format_idc = 1;
	sps.seq_parameter_set_id = 0;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_sps(ctx, &sps), 0);

	pps.seq_parameter_set_id = 0;
	pps.entropy_coding_mode_flag = 1;
	pps.num_slice_groups_minus1 = 0; /* keep the slice-group-map path out */
	pps.pic_init_qp_minus26 = -2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_pps(ctx, &pps), 0);

	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_PPS;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "pps", &jpps));
	CU_ASSERT_TRUE(json_object_object_get_ex(
		jpps, "entropy_coding_mode_flag", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 1);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jpps, "pic_init_qp_minus26", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), -2);

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


static void test_dump_aud(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_aud aud = {0};
	struct h264_nalu_header nh = {0};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *jaud = NULL;
	json_object *val = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	aud.primary_pic_type = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_aud(ctx, &aud), 0);

	nh.nal_ref_idc = 0;
	nh.nal_unit_type = H264_NALU_TYPE_AUD;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "aud", &jaud));
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jaud, "primary_pic_type", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 2);

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


static void test_dump_get_json_str(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_aud aud = {.primary_pic_type = 1};
	struct h264_nalu_header nh = {.nal_ref_idc = 0,
				      .nal_unit_type = H264_NALU_TYPE_AUD};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	const char *str = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_aud(ctx, &aud), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx, 0), 0);
	CU_ASSERT_EQUAL(h264_dump_get_json_str(dump, &str), 0);
	CU_ASSERT_PTR_NOT_NULL(str);
	CU_ASSERT_TRUE(strstr(str, "primary_pic_type") != NULL);

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


/* h264_dump_nalu() calls h264_dump_clear() internally before dumping, so a
 * second dump on a different ctx must not carry over the first one's
 * content. */
static void test_dump_clear_starts_fresh(void)
{
	struct h264_ctx *ctx_sps = NULL;
	struct h264_ctx *ctx_aud = NULL;
	struct h264_sps sps = {.profile_idc = H264_PROFILE_HIGH,
			       .chroma_format_idc = 1,
			       .frame_mbs_only_flag = 1};
	struct h264_nalu_header sps_nh = {.nal_ref_idc = 3,
					  .nal_unit_type = H264_NALU_TYPE_SPS};
	struct h264_aud aud = {.primary_pic_type = 0};
	struct h264_nalu_header aud_nh = {.nal_ref_idc = 0,
					  .nal_unit_type = H264_NALU_TYPE_AUD};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *val = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx_sps), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_sps(ctx_sps, &sps), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx_sps, &sps_nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx_aud), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_aud(ctx_aud, &aud), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx_aud, &aud_nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx_sps, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "sps", &val));
	CU_ASSERT_FALSE(json_object_object_get_ex(top, "aud", &val));

	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx_aud, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "aud", &val));
	CU_ASSERT_FALSE(json_object_object_get_ex(top, "sps", &val));

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx_sps);
	h264_ctx_destroy(ctx_aud);
}


/* h264_ctx_add_sei() + dump: the "sei" key is a JSON array (H264_BEGIN_ARRAY
 * in the nalu()'s SEI case); H264_SEI() itself (src/h264_syntax_ops.h) has
 * no extra struct wrapping, so a recovery_point SEI's fields land flat
 * inside its array item, alongside "payload_type"/"payload_size". */
static void test_dump_sei_recovery_point(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sei sei = {0};
	struct h264_nalu_header nh = {0};
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *jsei_array = NULL;
	json_object *jsei_item = NULL;
	json_object *val = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	sei.type = H264_SEI_TYPE_RECOVERY_POINT;
	sei.recovery_point.recovery_frame_cnt = 5;
	sei.recovery_point.exact_match_flag = 1;
	sei.recovery_point.broken_link_flag = 0;
	sei.recovery_point.changing_slice_group_idc = 0;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_add_sei(ctx, &sei), 0);

	nh.nal_ref_idc = 0;
	nh.nal_unit_type = H264_NALU_TYPE_SEI;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_nalu(dump, ctx, 0), 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);

	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(top, "sei", &jsei_array));
	CU_ASSERT_EQUAL_FATAL(json_object_get_type(jsei_array),
			      json_type_array);
	CU_ASSERT_EQUAL_FATAL(json_object_array_length(jsei_array), 1);
	jsei_item = json_object_array_get_idx(jsei_array, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jsei_item);
	CU_ASSERT_TRUE(json_object_object_get_ex(
		jsei_item, "recovery_frame_cnt", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 5);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jsei_item, "exact_match_flag", &val));
	CU_ASSERT_EQUAL(json_object_get_int64(val), 1);

	h264_dump_destroy(dump);
	h264_ctx_destroy(ctx);
}


/* h264_dump_nalu(dump, ctx, H264_DUMP_FLAGS_SLICE_DATA): the one dump path
 * that needs a prior real READ pass first -- src/h264_syntax.h's slice_
 * data()'s DUMP branch reconstructs a bitstream from ctx->slice.rawdata,
 * which is only ever populated by an H264_SYNTAX_OP_KIND_READ pass, never
 * by the writer. Reuses the write-then-read recipe from h264_test_slice.c,
 * but dumps the result instead of asserting on parsed fields directly. */
static void test_dump_slice_data(void)
{
	struct h264_ctx *wctx = NULL;
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	struct h264_nalu_header sps_nh = {.nal_ref_idc = 3,
					  .nal_unit_type = H264_NALU_TYPE_SPS};
	struct h264_nalu_header pps_nh = {.nal_ref_idc = 3,
					  .nal_unit_type = H264_NALU_TYPE_PPS};
	struct h264_nalu_header slice_nh = {
		.nal_ref_idc = 3, .nal_unit_type = H264_NALU_TYPE_SLICE_IDR};
	struct h264_slice_header sh = {0};
	struct h264_bitstream bs;
	uint8_t sps_buf[256];
	uint8_t pps_buf[256];
	uint8_t slice_buf[256];
	size_t sps_len, pps_len, slice_len;
	uint32_t mb_width = 2;
	uint32_t mb_height = 2;
	uint32_t mb_count = mb_width * mb_height;
	struct h264_ctx_cbs empty_cbs = {0};
	struct h264_reader *reader = NULL;
	struct h264_dump *dump = NULL;
	struct h264_dump_cfg cfg = {.type = H264_DUMP_TYPE_JSON};
	json_object *top = NULL;
	json_object *jslice = NULL;
	json_object *jslice_data = NULL;
	json_object *jmb = NULL;
	int res;

	/* Write SPS/PPS/slice via the public writer API (CAVLC). */
	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&wctx), 0);
	sps.chroma_format_idc = 1;
	sps.pic_width_in_mbs_minus1 = mb_width - 1;
	sps.pic_height_in_map_units_minus1 = mb_height - 1;
	sps.frame_mbs_only_flag = 1;
	sps.pic_order_cnt_type = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_sps(wctx, &sps), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_pps(wctx, &pps), 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(wctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(wctx, &sps_nh), 0);
	h264_bs_init(&bs, sps_buf, sizeof(sps_buf), 1);
	CU_ASSERT_TRUE_FATAL(h264_write_nalu(&bs, wctx) >= 0);
	sps_len = bs.off;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(wctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(wctx, &pps_nh), 0);
	h264_bs_init(&bs, pps_buf, sizeof(pps_buf), 1);
	CU_ASSERT_TRUE_FATAL(h264_write_nalu(&bs, wctx) >= 0);
	pps_len = bs.off;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_clear_nalu(wctx), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(wctx, &slice_nh), 0);
	sh.first_mb_in_slice = 0;
	sh.slice_type = H264_SLICE_TYPE_I;
	sh.disable_deblocking_filter_idc = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(wctx, &sh), 0);
	h264_bs_init(&bs, slice_buf, sizeof(slice_buf), 1);
	res = h264_write_grey_i_slice(&bs, wctx, mb_count);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	slice_len = bs.off;
	h264_ctx_destroy(wctx);

	/* Read it back with SLICE_DATA so ctx->slice.rawdata gets populated
	 * for the dump to reconstruct from. */
	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&empty_cbs, NULL, &reader), 0);
	CU_ASSERT_TRUE_FATAL(
		h264_reader_parse_nalu(reader, 0, sps_buf, sps_len) >= 0);
	CU_ASSERT_TRUE_FATAL(
		h264_reader_parse_nalu(reader, 0, pps_buf, pps_len) >= 0);
	res = h264_reader_parse_nalu(
		reader, H264_READER_FLAGS_SLICE_DATA, slice_buf, slice_len);
	CU_ASSERT_TRUE_FATAL(res >= 0);

	CU_ASSERT_EQUAL_FATAL(h264_dump_new(&cfg, &dump), 0);
	res = h264_dump_nalu(
		dump, h264_reader_get_ctx(reader), H264_DUMP_FLAGS_SLICE_DATA);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL_FATAL(h264_dump_get_json_object(dump, &top), 0);

	/* Both "slice_header" and "slice_data" are nested inside a "slice"
	 * struct: the nalu() dispatch (src/h264_syntax.h) wraps the whole
	 * slice_layer() call in H264_BEGIN_STRUCT(slice) before ever
	 * reaching slice_layer()'s own slice_header/slice_data sub-structs. */
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(top, "slice", &jslice));
	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(jslice, "slice_data", &jslice_data));
	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(jslice_data, "mb", &jmb));
	CU_ASSERT_EQUAL_FATAL(json_object_get_type(jmb), json_type_array);
	CU_ASSERT_EQUAL(json_object_array_length(jmb), mb_count);

	h264_dump_destroy(dump);
	h264_reader_destroy(reader);
}


CU_TestInfo g_h264_test_dump[] = {
	{FN("null-args"), &test_dump_null_args},
	{FN("sps-no-vui"), &test_dump_sps_no_vui},
	{FN("sps-with-vui"), &test_dump_sps_with_vui},
	{FN("pps"), &test_dump_pps},
	{FN("aud"), &test_dump_aud},
	{FN("get-json-str"), &test_dump_get_json_str},
	{FN("clear-starts-fresh"), &test_dump_clear_starts_fresh},
	{FN("sei-recovery-point"), &test_dump_sei_recovery_point},
	{FN("slice-data"), &test_dump_slice_data},

	CU_TEST_INFO_NULL,
};
