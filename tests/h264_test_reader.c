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


static void test_parse_nalu_header(void)
{
	/* 0x67 = 0b0_11_00111: forbidden_zero_bit=0, nal_ref_idc=3,
	 * nal_unit_type=7 (SPS) -- the well-known real-world SPS NAL header
	 * byte. */
	uint8_t sps_hdr[] = {0x67};
	/* 0x68 = 0b0_11_01000: nal_unit_type=8 (PPS) -- likewise a common
	 * real-world PPS NAL header byte. */
	uint8_t pps_hdr[] = {0x68};
	/* 0xe7: forbidden_zero_bit=1 -> must be rejected. */
	uint8_t bad_hdr[] = {0xe7};
	struct h264_nalu_header nh;
	int res;

	res = h264_parse_nalu_header(sps_hdr, sizeof(sps_hdr), &nh);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_EQUAL(nh.forbidden_zero_bit, 0);
	CU_ASSERT_EQUAL(nh.nal_ref_idc, 3);
	CU_ASSERT_EQUAL(nh.nal_unit_type, H264_NALU_TYPE_SPS);

	res = h264_parse_nalu_header(pps_hdr, sizeof(pps_hdr), &nh);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	CU_ASSERT_EQUAL(nh.nal_unit_type, H264_NALU_TYPE_PPS);

	res = h264_parse_nalu_header(bad_hdr, sizeof(bad_hdr), &nh);
	CU_ASSERT_EQUAL(res, -EIO);

	CU_ASSERT_EQUAL(h264_parse_nalu_header(NULL, 1, &nh), -EINVAL);
	CU_ASSERT_EQUAL(h264_parse_nalu_header(sps_hdr, 1, NULL), -EINVAL);
}


static void test_reader_new_destroy(void)
{
	struct h264_ctx_cbs cbs = {0};
	struct h264_reader *reader = NULL;

	CU_ASSERT_EQUAL(h264_reader_new(NULL, NULL, &reader), -EINVAL);
	CU_ASSERT_EQUAL(h264_reader_new(&cbs, NULL, NULL), -EINVAL);

	CU_ASSERT_EQUAL(h264_reader_new(&cbs, NULL, &reader), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(reader);
	CU_ASSERT_PTR_NOT_NULL(h264_reader_get_ctx(reader));

	CU_ASSERT_EQUAL(h264_reader_destroy(reader), 0);
	CU_ASSERT_EQUAL(h264_reader_destroy(NULL), 0);
	CU_ASSERT_PTR_NULL(h264_reader_get_ctx(NULL));
}


static void test_reader_stop_and_parse_null_args(void)
{
	struct h264_ctx_cbs cbs = {0};
	struct h264_reader *reader = NULL;
	uint8_t buf[4] = {0};
	size_t off = 0;

	CU_ASSERT_EQUAL(h264_reader_stop(NULL), -EINVAL);

	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&cbs, NULL, &reader), 0);
	CU_ASSERT_EQUAL(h264_reader_stop(reader), 0);
	CU_ASSERT_EQUAL(h264_reader_parse(NULL, 0, buf, sizeof(buf), &off),
			-EINVAL);
	CU_ASSERT_EQUAL(h264_reader_parse(reader, 0, NULL, sizeof(buf), &off),
			-EINVAL);
	CU_ASSERT_EQUAL(h264_reader_parse(reader, 0, buf, sizeof(buf), NULL),
			-EINVAL);
	h264_reader_destroy(reader);
}


struct reader_test_userdata {
	int sps_count;
	int pps_count;
	int aud_count;
	int nalu_begin_count;
	int nalu_end_count;
	struct h264_sps last_sps;
	struct h264_pps last_pps;
	struct h264_aud last_aud;
};


static void on_sps(struct h264_ctx *ctx,
		   const uint8_t *buf,
		   size_t len,
		   const struct h264_sps *sps,
		   void *userdata)
{
	struct reader_test_userdata *u = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	u->sps_count++;
	u->last_sps = *sps;
}


static void on_pps(struct h264_ctx *ctx,
		   const uint8_t *buf,
		   size_t len,
		   const struct h264_pps *pps,
		   void *userdata)
{
	struct reader_test_userdata *u = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	u->pps_count++;
	u->last_pps = *pps;
}


static void on_aud(struct h264_ctx *ctx,
		   const uint8_t *buf,
		   size_t len,
		   const struct h264_aud *aud,
		   void *userdata)
{
	struct reader_test_userdata *u = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	u->aud_count++;
	u->last_aud = *aud;
}


static void on_nalu_begin(struct h264_ctx *ctx,
			  enum h264_nalu_type type,
			  const uint8_t *buf,
			  size_t len,
			  const struct h264_nalu_header *nh,
			  void *userdata)
{
	struct reader_test_userdata *u = userdata;
	(void)ctx;
	(void)type;
	(void)buf;
	(void)len;
	(void)nh;
	u->nalu_begin_count++;
}


static void on_nalu_end(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			const struct h264_nalu_header *nh,
			void *userdata)
{
	struct reader_test_userdata *u = userdata;
	(void)ctx;
	(void)type;
	(void)buf;
	(void)len;
	(void)nh;
	u->nalu_end_count++;
}


static const struct h264_ctx_cbs s_cbs = {
	.sps = &on_sps,
	.pps = &on_pps,
	.aud = &on_aud,
	.nalu_begin = &on_nalu_begin,
	.nalu_end = &on_nalu_end,
};


/* Build an Annex-B byte stream (SPS + PPS + AUD, each preceded by a 4-byte
 * start code) purely via the public writer API, then parse it back with
 * h264_reader_parse() and check every callback fired with the expected
 * field values. */
static void test_reader_parse_sps_pps_aud_stream(void)
{
	struct h264_sps sps = {0};
	struct h264_pps pps = {0};
	uint8_t *sps_buf = NULL;
	uint8_t *pps_buf = NULL;
	size_t sps_len = 0;
	size_t pps_len = 0;
	struct h264_ctx *actx = NULL;
	struct h264_aud aud = {0};
	struct h264_nalu_header anh = {0};
	struct h264_bitstream abs;
	uint8_t aud_buf[32];
	uint8_t start_code[4] = {0x00, 0x00, 0x00, 0x01};
	uint8_t *stream = NULL;
	size_t total;
	size_t off = 0;
	struct reader_test_userdata u = {0};
	struct h264_reader *reader = NULL;
	size_t parse_off = 0;
	int res;

	res = h264_test_build_ps(
		11, 9, &sps, &pps, &sps_buf, &sps_len, &pps_buf, &pps_len);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&actx), 0);
	aud.primary_pic_type = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_aud(actx, &aud), 0);
	anh.nal_ref_idc = 0;
	anh.nal_unit_type = H264_NALU_TYPE_AUD;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_nalu_header(actx, &anh), 0);
	h264_bs_init(&abs, aud_buf, sizeof(aud_buf), 1);
	res = h264_write_nalu(&abs, actx);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	h264_ctx_destroy(actx);

	total = 4 + sps_len + 4 + pps_len + 4 + abs.off;
	stream = malloc(total);
	CU_ASSERT_PTR_NOT_NULL_FATAL(stream);

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
	memcpy(stream + off, aud_buf, abs.off);
	off += abs.off;
	CU_ASSERT_EQUAL_FATAL(off, total);

	CU_ASSERT_EQUAL_FATAL(h264_reader_new(&s_cbs, &u, &reader), 0);
	res = h264_reader_parse(reader, 0, stream, total, &parse_off);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(u.sps_count, 1);
	CU_ASSERT_EQUAL(u.pps_count, 1);
	CU_ASSERT_EQUAL(u.aud_count, 1);
	CU_ASSERT_EQUAL(u.nalu_begin_count, 3);
	CU_ASSERT_EQUAL(u.nalu_end_count, 3);
	CU_ASSERT_EQUAL(u.last_sps.profile_idc, sps.profile_idc);
	CU_ASSERT_EQUAL(u.last_sps.pic_width_in_mbs_minus1,
			sps.pic_width_in_mbs_minus1);
	CU_ASSERT_EQUAL(u.last_pps.entropy_coding_mode_flag,
			pps.entropy_coding_mode_flag);
	CU_ASSERT_EQUAL(u.last_aud.primary_pic_type, 2);

	CU_ASSERT_PTR_NOT_NULL(h264_ctx_get_sps(h264_reader_get_ctx(reader)));
	CU_ASSERT_PTR_NOT_NULL(h264_ctx_get_pps(h264_reader_get_ctx(reader)));

	h264_reader_destroy(reader);
	free(stream);
	free(sps_buf);
	free(pps_buf);
}


CU_TestInfo g_h264_test_reader[] = {
	{FN("parse-nalu-header"), &test_parse_nalu_header},
	{FN("new-destroy"), &test_reader_new_destroy},
	{FN("stop-and-parse-null-args"), &test_reader_stop_and_parse_null_args},
	{FN("parse-sps-pps-aud-stream"), &test_reader_parse_sps_pps_aud_stream},

	CU_TEST_INFO_NULL,
};
