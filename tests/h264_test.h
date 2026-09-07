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

#ifndef _H264_TEST_H_
#define _H264_TEST_H_

#include <h264/h264.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define FN(_name) (char *)_name
#define ARRAY_SIZE(_x) (sizeof(_x) / sizeof((_x)[0]))


extern CU_TestInfo g_h264_test_bitstream[];
extern CU_TestInfo g_h264_test_types[];
extern CU_TestInfo g_h264_test_ctx[];
extern CU_TestInfo g_h264_test_writer[];
extern CU_TestInfo g_h264_test_reader[];
extern CU_TestInfo g_h264_test_dump[];
extern CU_TestInfo g_h264_test_slice[];
extern CU_TestInfo g_h264_test_fmo[];
extern CU_TestInfo g_h264_test_files[];


/* Build a minimal, valid Main-profile H.264 SPS/PPS pair purely through the
 * public API (h264_ctx_new/set_sps/set_pps/set_nalu_header + h264_bs_init +
 * h264_write_nalu), so tests get real, parseable NALU byte buffers without
 * needing an external elementary stream file. Mirrors the fixture already
 * used by packages/libvideo-encode/tests/venc_test_h264.c
 * (venc_test_build_h264_ps), made self-contained here (no vdef dependency).
 * out_sps/out_pps receive the struct values that were encoded (useful to
 * compare against a re-parsed result). The caller must free() *ret_sps_buf
 * and *ret_pps_buf. */
static inline int h264_test_build_ps(unsigned int mb_width,
				     unsigned int mb_height,
				     struct h264_sps *out_sps,
				     struct h264_pps *out_pps,
				     uint8_t **ret_sps_buf,
				     size_t *ret_sps_len,
				     uint8_t **ret_pps_buf,
				     size_t *ret_pps_len)
{
	struct h264_ctx *ctx = NULL;
	struct h264_nalu_header nh = {0};
	struct h264_bitstream bs;
	uint8_t *sps_buf = NULL;
	uint8_t *pps_buf = NULL;
	int res;

	memset(out_sps, 0, sizeof(*out_sps));
	memset(out_pps, 0, sizeof(*out_pps));

	out_sps->profile_idc = H264_PROFILE_MAIN;
	out_sps->constraint_set1_flag = 1;
	out_sps->level_idc = 40;
	out_sps->chroma_format_idc = 1; /* 4:2:0 */
	out_sps->log2_max_frame_num_minus4 = 4;
	out_sps->pic_order_cnt_type = 0;
	out_sps->log2_max_pic_order_cnt_lsb_minus4 = 4;
	out_sps->max_num_ref_frames = 1;
	out_sps->pic_width_in_mbs_minus1 = mb_width - 1;
	out_sps->pic_height_in_map_units_minus1 = mb_height - 1;
	out_sps->frame_mbs_only_flag = 1;
	out_sps->direct_8x8_inference_flag = 1;
	out_sps->vui_parameters_present_flag = 1;
	out_sps->vui.aspect_ratio_info_present_flag = 1;
	out_sps->vui.aspect_ratio_idc = H264_ASPECT_RATIO_1_1;
	out_sps->vui.timing_info_present_flag = 1;
	out_sps->vui.num_units_in_tick = 1;
	out_sps->vui.time_scale = 60;
	out_sps->vui.fixed_frame_rate_flag = 1;

	out_pps->entropy_coding_mode_flag = 1;

	res = h264_ctx_new(&ctx);
	if (res < 0)
		goto out;

	res = h264_ctx_set_sps(ctx, out_sps);
	if (res < 0)
		goto out;
	res = h264_ctx_set_pps(ctx, out_pps);
	if (res < 0)
		goto out;

	res = h264_ctx_clear_nalu(ctx);
	if (res < 0)
		goto out;
	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_SPS;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0)
		goto out;
	sps_buf = malloc(256);
	if (sps_buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	h264_bs_init(&bs, sps_buf, 256, 1);
	res = h264_write_nalu(&bs, ctx);
	if (res < 0)
		goto out;
	*ret_sps_len = bs.off;

	res = h264_ctx_clear_nalu(ctx);
	if (res < 0)
		goto out;
	nh.nal_unit_type = H264_NALU_TYPE_PPS;
	res = h264_ctx_set_nalu_header(ctx, &nh);
	if (res < 0)
		goto out;
	pps_buf = malloc(256);
	if (pps_buf == NULL) {
		res = -ENOMEM;
		goto out;
	}
	h264_bs_init(&bs, pps_buf, 256, 1);
	res = h264_write_nalu(&bs, ctx);
	if (res < 0)
		goto out;
	*ret_pps_len = bs.off;

	*ret_sps_buf = sps_buf;
	*ret_pps_buf = pps_buf;
	sps_buf = NULL;
	pps_buf = NULL;
	res = 0;

out:
	free(sps_buf);
	free(pps_buf);
	if (ctx != NULL)
		h264_ctx_destroy(ctx);
	return res;
}


#endif /* _H264_TEST_H_ */
