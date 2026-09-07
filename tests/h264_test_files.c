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

#include <limits.h>
#include <stdio.h>
#include <unistd.h>

/*
 * Same convention as packages/libaac/tests/aac_test_files.c (itself matching
 * packages/libmp4/tests/mp4_test.h and packages/pdraw/libpdraw/tests/
 * test_api_common.hpp): a real NAS-hosted asset tree, overridable via the
 * ASSETS_ROOT env var, with a hard (fatal) failure when a fixture is
 * unreachable rather than a silent skip. This test area needs a mounted NAS
 * (or ASSETS_ROOT pointed at a local copy) to pass.
 */
#define H264_TEST_ASSETS_ROOT_DEFAULT "/mnt/DFS/MULTIMEDIA_DATA"


struct h264_asset_fixture {
	const char *relative_path;

	/* h264_reader_parse() flags -- H264_READER_FLAGS_SLICE_DATA only
	 * works with CAVLC content (the reader has no CABAC decode path),
	 * so it must stay 0 for any fixture that isn't known to be CAVLC. */
	uint32_t reader_flags;

	/* Extra assertion: fail unless at least one B slice was parsed
	 * (only meaningful together with H264_READER_FLAGS_SLICE_DATA). */
	int expect_b_slices;

	/* Extra assertion: fail unless a buffering_period SEI, at least one
	 * FILLER NALU, and an explicit weighted-prediction PPS
	 * (weighted_pred_flag or weighted_bipred_idc) were all seen --
	 * confirms this fixture's nal-hrd cbr + filler=1 + weightb 1
	 * -weightp 2 encode options actually landed in the bitstream.
	 * (cqm=flat does *not* set seq/pic_scaling_matrix_present_flag --
	 * "flat" means "no custom matrix", i.e. it disables signaling rather
	 * than forcing it, so that one isn't checked here.) */
	int expect_extras;
};

/* Add new real-file fixtures here as they become available -- no other code
 * needs to change. */
static const struct h264_asset_fixture s_assets[] = {
	{"Tests/miscellaneous/crowd_run_240p.264", 0, 0, 0},
	{"Tests/miscellaneous/crowd_run_240p_cavlc.264",
	 H264_READER_FLAGS_SLICE_DATA,
	 0,
	 0},
	{"Tests/miscellaneous/crowd_run_240p_bslices.264",
	 H264_READER_FLAGS_SLICE_DATA,
	 1,
	 0},
	{"Tests/miscellaneous/cvmp_mot_mbaff0_full_B.264",
	 H264_READER_FLAGS_SLICE_DATA,
	 1,
	 0},
	/* Synthetic ffmpeg testsrc, CAVLC: nal-hrd cbr -b:v 5M filler=1 (VUI
	 * HRD + buffering_period SEI + FILLER NALUs) + weightb 1 -weightp 2
	 * (weighted prediction flags in the PPS/slice header). */
	{"Tests/miscellaneous/ffmpeg_testsrc_cavlc_hrd_wpred.264",
	 H264_READER_FLAGS_SLICE_DATA,
	 1,
	 1},
};


static int
build_asset_path(const char *relative_path, char *path, size_t path_len)
{
	const char *root = getenv("ASSETS_ROOT");
	int n;

	if (root == NULL)
		root = H264_TEST_ASSETS_ROOT_DEFAULT;
	n = snprintf(path, path_len, "%s/%s", root, relative_path);
	if (n < 0 || (size_t)n >= path_len)
		return -ENAMETOOLONG;
	return 0;
}


/* Same robustness convention as aac_test_files.c's wait_for_asset(): retry a
 * few times with a short backoff before treating the asset as genuinely
 * missing, to absorb transient NFS/SMB mount glitches. */
static int wait_for_asset(const char *path)
{
	static const unsigned int backoff_ms[] = {100, 300, 800};

	if (access(path, R_OK) == 0)
		return 0;

	for (size_t i = 0; i < ARRAY_SIZE(backoff_ms); i++) {
		usleep(backoff_ms[i] * 1000);
		if (access(path, R_OK) == 0)
			return 0;
	}
	return -errno;
}


static uint8_t *read_whole_file(const char *path, size_t *out_len)
{
	FILE *f;
	long size;
	uint8_t *buf;
	size_t rd;

	f = fopen(path, "rb");
	if (f == NULL)
		return NULL;
	if (fseek(f, 0, SEEK_END) != 0 || (size = ftell(f)) < 0 ||
	    fseek(f, 0, SEEK_SET) != 0) {
		fclose(f);
		return NULL;
	}
	buf = malloc((size_t)size);
	if (buf == NULL) {
		fclose(f);
		return NULL;
	}
	rd = fread(buf, 1, (size_t)size, f);
	fclose(f);
	if (rd != (size_t)size) {
		free(buf);
		return NULL;
	}
	*out_len = (size_t)size;
	return buf;
}


struct file_test_userdata {
	int nalu_count;
	int sps_count;
	int pps_count;
	int slice_count;
	int got_info;
	struct h264_info info;

	/* Only populated when H264_READER_FLAGS_SLICE_DATA is set */
	int slice_data_begin_count;
	int slice_data_end_count;
	int mb_count;
	int mb_type_seen[H264_MB_TYPE_B_SKIP + 1];
	int b_slice_count;

	/* Only meaningful for the ffmpeg_testsrc_cavlc_hrd_wpred.264 fixture */
	int filler_count;
	int buffering_period_sei_count;
	int weighted_pred_seen;
};


static void on_nalu_end(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			const struct h264_nalu_header *nh,
			void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)buf;
	(void)len;
	(void)nh;

	t->nalu_count++;
	switch (type) {
	case H264_NALU_TYPE_SPS:
		t->sps_count++;
		break;
	case H264_NALU_TYPE_PPS:
		t->pps_count++;
		break;
	case H264_NALU_TYPE_SLICE:
	case H264_NALU_TYPE_SLICE_IDR:
		t->slice_count++;
		break;
	case H264_NALU_TYPE_FILLER:
		t->filler_count++;
		break;
	default:
		break;
	}

	if (!t->got_info) {
		struct h264_info info;
		if (h264_ctx_get_info(ctx, &info) == 0) {
			t->info = info;
			t->got_info = 1;
		}
	}
}


static void on_pps(struct h264_ctx *ctx,
		    const uint8_t *buf,
		    size_t len,
		    const struct h264_pps *pps,
		    void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	if (pps->weighted_pred_flag || pps->weighted_bipred_idc != 0)
		t->weighted_pred_seen = 1;
}


static void on_sei_buffering_period(struct h264_ctx *ctx,
				     const uint8_t *buf,
				     size_t len,
				     const struct h264_sei_buffering_period *sei,
				     void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	(void)sei;
	t->buffering_period_sei_count++;
}


static void on_slice(struct h264_ctx *ctx,
		     const uint8_t *buf,
		     size_t len,
		     const struct h264_slice_header *sh,
		     void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)buf;
	(void)len;
	if (H264_SLICE_TYPE(sh->slice_type) == H264_SLICE_TYPE_B)
		t->b_slice_count++;
}


static void on_slice_data_begin(struct h264_ctx *ctx,
				const struct h264_slice_header *sh,
				void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)sh;
	t->slice_data_begin_count++;
}


static void on_slice_data_end(struct h264_ctx *ctx,
			      const struct h264_slice_header *sh,
			      uint32_t mb_count,
			      void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)sh;
	(void)mb_count;
	t->slice_data_end_count++;
}


static void on_slice_data_mb(struct h264_ctx *ctx,
			     const struct h264_slice_header *sh,
			     uint32_t mb_addr,
			     enum h264_mb_type mb_type,
			     void *userdata)
{
	struct file_test_userdata *t = userdata;
	(void)ctx;
	(void)sh;
	(void)mb_addr;
	t->mb_count++;
	if ((size_t)mb_type < ARRAY_SIZE(t->mb_type_seen))
		t->mb_type_seen[mb_type]++;
}


static const struct h264_ctx_cbs s_cbs = {
	.nalu_end = &on_nalu_end,
	.pps = &on_pps,
	.sei_buffering_period = &on_sei_buffering_period,
	.slice = &on_slice,
	.slice_data_begin = &on_slice_data_begin,
	.slice_data_end = &on_slice_data_end,
	.slice_data_mb = &on_slice_data_mb,
};


static void run_asset_test(const struct h264_asset_fixture *fixture)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	struct h264_reader *reader = NULL;
	struct file_test_userdata userdata = {0};
	size_t off = 0;
	int res;

	int asset_path_ret =
		build_asset_path(fixture->relative_path, path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(asset_path_ret, 0);

	/* Hard failure if the NAS asset is unreachable, same convention as
	 * aac_test_files.c/libmp4/pdraw's own asset-backed tests -- not a
	 * silent skip. */
	int asset_access = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(asset_access, 0);

	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	res = h264_reader_new(&s_cbs, &userdata, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = h264_reader_parse(reader, fixture->reader_flags, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(userdata.nalu_count > 0);
	CU_ASSERT_TRUE(userdata.sps_count >= 1);
	CU_ASSERT_TRUE(userdata.pps_count >= 1);
	CU_ASSERT_TRUE(userdata.slice_count > 0);
	CU_ASSERT_TRUE(userdata.got_info);
	CU_ASSERT_TRUE(userdata.info.width > 0);
	CU_ASSERT_TRUE(userdata.info.height > 0);

	/* SPS/PPS/SEI/NALU-header-level checks: these are independent of the
	 * slice entropy coding, so they apply regardless of whether SLICE_
	 * DATA parsing is enabled for this fixture. */
	if (fixture->expect_b_slices)
		CU_ASSERT_TRUE(userdata.b_slice_count > 0);

	if (fixture->expect_extras) {
		CU_ASSERT_TRUE(userdata.buffering_period_sei_count > 0);
		CU_ASSERT_TRUE(userdata.filler_count > 0);
		CU_ASSERT_TRUE(userdata.weighted_pred_seen);
	}

	if (fixture->reader_flags & H264_READER_FLAGS_SLICE_DATA) {
		int distinct_mb_types = 0;

		/* A genuine per-macroblock CAVLC parse: every slice must
		 * have produced matching begin/end callbacks and at least
		 * one macroblock, across a real mix of macroblock types
		 * (not just a uniform skip/grey pattern like the synthetic
		 * fixtures in h264_test_slice.c). */
		CU_ASSERT_EQUAL(userdata.slice_data_begin_count,
				userdata.slice_count);
		CU_ASSERT_EQUAL(userdata.slice_data_end_count,
				userdata.slice_count);
		CU_ASSERT_TRUE(userdata.mb_count > 0);

		for (size_t i = 0; i < ARRAY_SIZE(userdata.mb_type_seen); i++) {
			if (userdata.mb_type_seen[i] > 0)
				distinct_mb_types++;
		}
		CU_ASSERT_TRUE(distinct_mb_types >= 2);
	}

	h264_reader_destroy(reader);
	free(buf);
}


static void test_crowd_run_240p(void)
{
	run_asset_test(&s_assets[0]);
}


static void test_crowd_run_240p_cavlc(void)
{
	run_asset_test(&s_assets[1]);
}


static void test_crowd_run_240p_bslices(void)
{
	run_asset_test(&s_assets[2]);
}


static void test_cvmp_mot_mbaff0_full_b(void)
{
	run_asset_test(&s_assets[3]);
}


static void test_ffmpeg_testsrc_cavlc_hrd_wpred(void)
{
	run_asset_test(&s_assets[4]);
}


CU_TestInfo g_h264_test_files[] = {
	{FN("crowd-run-240p"), &test_crowd_run_240p},
	{FN("crowd-run-240p-cavlc"), &test_crowd_run_240p_cavlc},
	{FN("crowd-run-240p-bslices"), &test_crowd_run_240p_bslices},
	{FN("cvmp-mot-mbaff0-full-b"), &test_cvmp_mot_mbaff0_full_b},
	{FN("ffmpeg-testsrc-cavlc-hrd-wpred"),
	 &test_ffmpeg_testsrc_cavlc_hrd_wpred},

	CU_TEST_INFO_NULL,
};
