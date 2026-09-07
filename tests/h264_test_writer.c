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


/* h264_write_nalu() for a SPS/PPS pair (via h264_test_build_ps()), then
 * h264_parse_sps()/h264_parse_pps() read the produced bytes back: the
 * re-parsed struct must match the field values that were encoded. Mirrors
 * packages/libvideo-encode/tests/venc_test_h264.c's own SPS/PPS round trip,
 * using the fixture builder from h264_test.h instead of a venc-side copy. */
static void test_writer_sps_pps_roundtrip(void)
{
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	struct h264_sps parsed_sps = {0};
	struct h264_pps parsed_pps = {0};
	uint8_t *sps_buf = NULL;
	uint8_t *pps_buf = NULL;
	size_t sps_len = 0;
	size_t pps_len = 0;
	int res;

	res = h264_test_build_ps(
		11, 9, &sps, &pps, &sps_buf, &sps_len, &pps_buf, &pps_len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps_buf);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps_buf);
	CU_ASSERT_TRUE(sps_len > 0);
	CU_ASSERT_TRUE(pps_len > 0);

	res = h264_parse_sps(sps_buf, sps_len, &parsed_sps);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_EQUAL(parsed_sps.profile_idc, sps.profile_idc);
	CU_ASSERT_EQUAL(parsed_sps.level_idc, sps.level_idc);
	CU_ASSERT_EQUAL(parsed_sps.pic_width_in_mbs_minus1,
			sps.pic_width_in_mbs_minus1);
	CU_ASSERT_EQUAL(parsed_sps.pic_height_in_map_units_minus1,
			sps.pic_height_in_map_units_minus1);
	CU_ASSERT_EQUAL(parsed_sps.frame_mbs_only_flag,
			sps.frame_mbs_only_flag);
	CU_ASSERT_EQUAL(parsed_sps.vui.time_scale, sps.vui.time_scale);
	CU_ASSERT_EQUAL(parsed_sps.vui.num_units_in_tick,
			sps.vui.num_units_in_tick);

	res = h264_parse_pps(pps_buf, pps_len, &parsed_sps, &parsed_pps);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_EQUAL(parsed_pps.entropy_coding_mode_flag,
			pps.entropy_coding_mode_flag);

	free(sps_buf);
	free(pps_buf);
}


/* AUD/filler writing needs only a bare ctx (no SPS/PPS at all), confirmed by
 * packages/libvideo-encode/tests/venc_test_h264.c's own AUD/SEI tests. */
static void test_writer_aud_roundtrip(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_nalu_header nh = {0};
	struct h264_aud aud = {0};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	struct h264_nalu_header parsed_nh = {0};
	int res;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	aud.primary_pic_type = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_aud(ctx, &aud), 0);

	nh.nal_ref_idc = 0;
	nh.nal_unit_type = H264_NALU_TYPE_AUD;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(ctx, &nh), 0);

	buf = malloc(32);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	h264_bs_init(&bs, buf, 32, 1);
	res = h264_write_nalu(&bs, ctx);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_TRUE(bs.off > 0);

	res = h264_parse_nalu_header(buf, bs.off, &parsed_nh);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_EQUAL(parsed_nh.nal_unit_type, H264_NALU_TYPE_AUD);
	CU_ASSERT_EQUAL(parsed_nh.nal_ref_idc, 0);

	free(buf);
	h264_ctx_destroy(ctx);
}


static void test_rewrite_slice_header_null_args(void)
{
	struct h264_ctx *ctx = NULL;
	struct h264_bitstream bs;
	uint8_t buf[32];
	struct h264_slice_header sh = {0};

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);
	h264_bs_init(&bs, buf, sizeof(buf), 1);

	CU_ASSERT_EQUAL(h264_rewrite_slice_header(NULL, ctx, &sh), -EINVAL);
	CU_ASSERT_EQUAL(h264_rewrite_slice_header(&bs, NULL, &sh), -EINVAL);
	CU_ASSERT_EQUAL(h264_rewrite_slice_header(&bs, ctx, NULL), -EINVAL);

	h264_ctx_destroy(ctx);
}


CU_TestInfo g_h264_test_writer[] = {
	{FN("sps-pps-roundtrip"), &test_writer_sps_pps_roundtrip},
	{FN("aud-roundtrip"), &test_writer_aud_roundtrip},
	{FN("rewrite-slice-header-null-args"),
	 &test_rewrite_slice_header_null_args},

	CU_TEST_INFO_NULL,
};
