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


static void test_ctx_new_destroy(void)
{
	struct h264_ctx *ctx = NULL;

	CU_ASSERT_EQUAL(h264_ctx_new(NULL), -EINVAL);

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ctx);

	CU_ASSERT_EQUAL(h264_ctx_destroy(ctx), 0);
	CU_ASSERT_EQUAL(h264_ctx_destroy(NULL), 0);
}


static void test_ctx_null_arg_guards(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_nalu_header nh = {0};
	struct h264_aud aud = {0};
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	struct h264_sei sei = {0};
	struct h264_slice_header sh = {0};
	struct h264_info info;

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);

	CU_ASSERT_EQUAL(h264_ctx_clear(NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_clear_nalu(NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_nalu_header(NULL, &nh), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_nalu_header(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_is_nalu_unknown(NULL), 0);
	CU_ASSERT_EQUAL(h264_ctx_set_aud(NULL, &aud), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_aud(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_sps(NULL, &sps), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_pps(NULL, &pps), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_pps(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_filler(NULL, 4), -EINVAL);
	CU_ASSERT_PTR_NULL(h264_ctx_get_sps(NULL));
	CU_ASSERT_PTR_NULL(h264_ctx_get_pps(NULL));
	CU_ASSERT_EQUAL(h264_ctx_add_sei(NULL, &sei), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_add_sei(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_get_sei_count(NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_slice_header(NULL, &sh), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_set_slice_header(ctx, NULL), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_get_info(NULL, &info), -EINVAL);
	CU_ASSERT_EQUAL(h264_ctx_get_info(ctx, NULL), -EINVAL);

	h264_ctx_destroy(ctx);
}


static void test_ctx_set_sps_id_out_of_range(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);
	sps.seq_parameter_set_id = 32; /* table has 32 entries: 0..31 */
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, &sps), -EINVAL);
	h264_ctx_destroy(ctx);
}


static void test_ctx_set_pps_id_out_of_range(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_pps pps = {0};

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);
	pps.pic_parameter_set_id = 256; /* table has 256 entries: 0..255 */
	CU_ASSERT_EQUAL(h264_ctx_set_pps(ctx, &pps), -EINVAL);
	h264_ctx_destroy(ctx);
}


static void test_ctx_set_get_sps_pps_roundtrip(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	const struct h264_sps *got_sps;
	const struct h264_pps *got_pps;

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);

	sps.profile_idc = H264_PROFILE_BASELINE;
	sps.seq_parameter_set_id = 0;
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, &sps), 0);
	got_sps = h264_ctx_get_sps(ctx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(got_sps);
	CU_ASSERT_EQUAL(got_sps->profile_idc, H264_PROFILE_BASELINE);

	pps.pic_parameter_set_id = 0;
	pps.entropy_coding_mode_flag = 1;
	CU_ASSERT_EQUAL(h264_ctx_set_pps(ctx, &pps), 0);
	got_pps = h264_ctx_get_pps(ctx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(got_pps);
	CU_ASSERT_EQUAL(got_pps->entropy_coding_mode_flag, 1);

	h264_ctx_destroy(ctx);
}


static void test_ctx_clear_nalu_keeps_sps_pps(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_nalu_header nh = {0};

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);
	sps.profile_idc = H264_PROFILE_HIGH;
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, &sps), 0);

	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_AUD;
	CU_ASSERT_EQUAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	CU_ASSERT_EQUAL(h264_ctx_clear_nalu(ctx), 0);
	/* SPS/PPS survive a per-NALU clear (comment in h264_ctx.c: "Keep
	 * current SPS/PPS") */
	CU_ASSERT_PTR_NOT_NULL_FATAL(h264_ctx_get_sps(ctx));
	CU_ASSERT_EQUAL(h264_ctx_get_sps(ctx)->profile_idc, H264_PROFILE_HIGH);

	h264_ctx_destroy(ctx);
}


static void test_ctx_add_sei_recovery_point(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sei sei = {0};

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);
	CU_ASSERT_EQUAL(h264_ctx_get_sei_count(ctx), 0);

	sei.type = H264_SEI_TYPE_RECOVERY_POINT;
	sei.recovery_point.recovery_frame_cnt = 5;
	sei.recovery_point.exact_match_flag = 1;
	sei.recovery_point.broken_link_flag = 0;
	sei.recovery_point.changing_slice_group_idc = 0;

	CU_ASSERT_EQUAL(h264_ctx_add_sei(ctx, &sei), 0);
	CU_ASSERT_EQUAL(h264_ctx_get_sei_count(ctx), 1);

	h264_ctx_destroy(ctx);
}


static void test_ctx_get_info_requires_sps_and_pps(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_info info;

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);

	/* No SPS/PPS set yet */
	CU_ASSERT_EQUAL(h264_ctx_get_info(ctx, &info), -EAGAIN);

	sps.chroma_format_idc = 1;
	sps.pic_width_in_mbs_minus1 = 9;
	sps.pic_height_in_map_units_minus1 = 9;
	sps.frame_mbs_only_flag = 1;
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, &sps), 0);

	/* SPS only, still no PPS */
	CU_ASSERT_EQUAL(h264_ctx_get_info(ctx, &info), -EAGAIN);

	struct h264_pps pps = {0};
	CU_ASSERT_EQUAL(h264_ctx_set_pps(ctx, &pps), 0);

	CU_ASSERT_EQUAL(h264_ctx_get_info(ctx, &info), 0);
	CU_ASSERT_EQUAL(info.width, 160);
	CU_ASSERT_EQUAL(info.height, 160);

	h264_ctx_destroy(ctx);
}


/* h264_ctx_sei_pic_timing_to_ts()/to_us(): 1-second timestamp at a
 * time_scale of 1200 (matching the "time_scale is twice the frame rate"
 * convention from E.2.1) with a single clk_ts entry and no time_offset. */
static void test_ctx_sei_pic_timing_to_ts_and_us(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_sps sps = {0};
	struct h264_sei_pic_timing pic_timing = {0};
	uint64_t ts;
	uint64_t us;

	CU_ASSERT_EQUAL(h264_ctx_new(&ctx), 0);

	sps.vui.time_scale = 1200;
	sps.vui.num_units_in_tick = 1;
	CU_ASSERT_EQUAL(h264_ctx_set_sps(ctx, &sps), 0);

	pic_timing.clk_ts[0].hours_value = 0;
	pic_timing.clk_ts[0].minutes_value = 0;
	pic_timing.clk_ts[0].seconds_value = 1;
	pic_timing.clk_ts[0].n_frames = 0;
	pic_timing.clk_ts[0].time_offset = 0;

	ts = h264_ctx_sei_pic_timing_to_ts(ctx, &pic_timing);
	CU_ASSERT_EQUAL(ts, 1200);

	us = h264_ctx_sei_pic_timing_to_us(ctx, &pic_timing);
	CU_ASSERT_EQUAL(us, 1000000);

	h264_ctx_destroy(ctx);
}


CU_TestInfo g_h264_test_ctx[] = {
	{FN("new-destroy"), &test_ctx_new_destroy},
	{FN("null-arg-guards"), &test_ctx_null_arg_guards},
	{FN("set-sps-id-out-of-range"), &test_ctx_set_sps_id_out_of_range},
	{FN("set-pps-id-out-of-range"), &test_ctx_set_pps_id_out_of_range},
	{FN("set-get-sps-pps-roundtrip"), &test_ctx_set_get_sps_pps_roundtrip},
	{FN("clear-nalu-keeps-sps-pps"), &test_ctx_clear_nalu_keeps_sps_pps},
	{FN("add-sei-recovery-point"), &test_ctx_add_sei_recovery_point},
	{FN("get-info-requires-sps-and-pps"),
	 &test_ctx_get_info_requires_sps_and_pps},
	{FN("sei-pic-timing-to-ts-and-us"),
	 &test_ctx_sei_pic_timing_to_ts_and_us},

	CU_TEST_INFO_NULL,
};
