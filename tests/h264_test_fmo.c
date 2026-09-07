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

/* h264_priv.h defines its own identical ARRAY_SIZE -- avoid the harmless
 * but noisy redefinition warning. */
#undef ARRAY_SIZE
#include "h264_priv.h"

/*
 * FMO (Flexible Macroblock Ordering, multi-slice-group) support in
 * src/h264_fmo.c is unreachable from any real-world fixture: it's a
 * Baseline-only feature essentially no modern encoder (including x264, the
 * only encoder available for local test-asset generation) ever produces.
 * But h264_gen_slice_group_map() is triggered automatically -- entirely
 * through the public API -- by h264_ctx_set_pps()/set_sps()/
 * set_slice_header() whenever pps->num_slice_groups_minus1 > 0 (see
 * h264_ctx_update_derived_vars_slice() in src/h264_ctx.c). Only reading the
 * resulting map back requires src/h264_priv.h (struct h264_ctx isn't
 * exposed publicly), so this is the one file in the suite that needs
 * LOCAL_C_INCLUDES := $(LOCAL_PATH)/src, mirroring packages/librtp's
 * tst-librtp.
 *
 * Expected maps below are hand-traced against src/h264_fmo.c's actual
 * algorithms (8.2.2.1/8.2.2.2/8.2.2.3), not copied from the spec text.
 */


struct fmo_fixture {
	struct h264_ctx *ctx;
};


static int fmo_fixture_setup(struct fmo_fixture *fx,
			     uint32_t mb_width,
			     uint32_t mb_height,
			     const struct h264_pps *pps)
{
	struct h264_sps sps = {0};
	int res;

	res = h264_ctx_new(&fx->ctx);
	if (res < 0)
		return res;

	sps.chroma_format_idc = 1;
	sps.pic_width_in_mbs_minus1 = mb_width - 1;
	sps.pic_height_in_map_units_minus1 = mb_height - 1;
	sps.frame_mbs_only_flag = 1;
	res = h264_ctx_set_sps(fx->ctx, &sps);
	if (res < 0)
		return res;

	return h264_ctx_set_pps(fx->ctx, pps);
}


static void fmo_fixture_clear(struct fmo_fixture *fx)
{
	if (fx->ctx != NULL)
		h264_ctx_destroy(fx->ctx);
	memset(fx, 0, sizeof(*fx));
}


/* 8.2.2.1 (h264_slice_group_map_type_0): a 4x2 map-unit picture (8 units),
 * 2 groups, run_length_minus1 = {2, 1}. Traced run by run:
 *   pass 1: i=0 grp=0 -> units 0,1,2; i=3 grp=1 -> units 3,4; i=5
 *   pass 2: i=5 grp=0 -> units 5,6,7 (only 3 left, all consumed); i=8, done
 */
static void test_fmo_type0_interleaved(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	static const uint32_t expected[] = {0, 0, 0, 1, 1, 0, 0, 0};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 0;
	pps.run_length_minus1[0] = 2;
	pps.run_length_minus1[1] = 1;

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.ctx->slice.group_map);

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected[i]);

	fmo_fixture_clear(&fx);
}


/* 8.2.2.2 (h264_slice_group_map_type_1): same 4x2 picture, 2 groups.
 * group_map[i] = ((i % W) + ((i / W) * N) / 2) % N, N = num_groups = 2. */
static void test_fmo_type1_dispersed(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	static const uint32_t expected[] = {0, 1, 0, 1, 1, 0, 1, 0};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 1;

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected[i]);

	fmo_fixture_clear(&fx);
}


/* 8.2.2.3 (h264_slice_group_map_type_2): same 4x2 picture. Only group 0's
 * box is explicit (top_left=1, bottom_right=2 -> row 0, columns 1..2);
 * every other map unit falls back to the leftover group (group 1). */
static void test_fmo_type2_foreground_leftover(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	static const uint32_t expected[] = {1, 0, 0, 1, 1, 1, 1, 1};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 2;
	pps.top_left[0] = 1;
	pps.bottom_right[0] = 2;

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected[i]);

	fmo_fixture_clear(&fx);
}


/* h264_next_mb_addr(): without slice groups it's a plain +1; with the
 * dispersed map above (8.2.2.8, frame_mbs_only_flag path is a direct
 * group_map[] lookup) it must skip to the next map unit in the same
 * group. */
static void test_next_mb_addr(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps_no_fmo = {0};
	struct h264_pps pps_dispersed = {0};

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps_no_fmo), 0);
	CU_ASSERT_EQUAL(h264_next_mb_addr(fx.ctx, 0), 1);
	CU_ASSERT_EQUAL(h264_next_mb_addr(fx.ctx, 5), 6);
	fmo_fixture_clear(&fx);

	pps_dispersed.num_slice_groups_minus1 = 1;
	pps_dispersed.slice_group_map_type = 1;
	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps_dispersed), 0);
	/* group_map = {0,1,0,1,1,0,1,0}: from mbAddr 0 (group 0), the next
	 * group-0 unit is index 2; from mbAddr 1 (group 1), index 3. */
	CU_ASSERT_EQUAL(h264_next_mb_addr(fx.ctx, 0), 2);
	CU_ASSERT_EQUAL(h264_next_mb_addr(fx.ctx, 1), 3);
	fmo_fixture_clear(&fx);
}


/* h264_clear_slice_group_map(): zeroes the map in place without freeing it
 * (the allocation is reused on the next h264_gen_slice_group_map() call). */
static void test_clear_slice_group_map(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 1;
	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);
	CU_ASSERT_EQUAL(fx.ctx->slice.group_map[1], 1);

	h264_clear_slice_group_map(fx.ctx);
	for (uint32_t i = 0; i < fx.ctx->sps_derived.PicSizeInMapUnits; i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], 0);

	fmo_fixture_clear(&fx);
}


/* 8.2.2.4 (h264_slice_group_map_type_3, box-out): a 2x2 picture (4 map
 * units), direction_flag=0, MapUnitsInSliceGroup0=2 (slice_group_change_
 * rate_minus1=0, slice_group_change_cycle=2). Traced step by step against
 * src/h264_fmo.c's h264_slice_group_map_type_3():
 *   init: x=y=left=top=right=bottom=1, xDir=-1, yDir=0, map=[1,1,1,1]
 *   k=0: (x=1,y=1) index=3 vacant -> map[3]=0; xDir==-1 && x==left(1) ->
 *        left=0, x=0, xDir=0, yDir=-1; k+=1 -> k=1
 *   k=1: (x=0,y=1) index=2 vacant -> map[2]=0; yDir==-1 && y==top(1) ->
 *        top=0, y=0, xDir=1, yDir=0; k+=1 -> k=2, loop ends (k<2 false)
 *   final map = [1,1,0,0]
 */
static void test_fmo_type3_box_out(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	struct h264_slice_header sh = {0};
	static const uint32_t expected[] = {1, 1, 0, 0};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 3;
	pps.slice_group_change_direction_flag = 0;
	pps.slice_group_change_rate_minus1 = 0;

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 2, 2, &pps), 0);
	sh.slice_group_change_cycle = 2;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);
	CU_ASSERT_EQUAL(fx.ctx->derived.MapUnitsInSliceGroup0, 2);

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected[i]);

	fmo_fixture_clear(&fx);
}


/* 8.2.2.5 (h264_slice_group_map_type_4, raster scan): 4x2 picture (8 map
 * units), slice_group_change_rate_minus1=0, slice_group_change_cycle=3 ->
 * MapUnitsInSliceGroup0=3. direction_flag=0: sizeOfUpperLeftGroup=3, first 3
 * map units get group 0, the rest group 1. direction_flag=1: mirrored
 * (sizeOfUpperLeftGroup=8-3=5, first 5 get group 1, rest group 0). */
static void test_fmo_type4_raster_scan(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	struct h264_slice_header sh = {0};
	static const uint32_t expected_dir0[] = {0, 0, 0, 1, 1, 1, 1, 1};
	static const uint32_t expected_dir1[] = {1, 1, 1, 1, 1, 0, 0, 0};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 4;
	pps.slice_group_change_direction_flag = 0;
	pps.slice_group_change_rate_minus1 = 0;
	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);
	sh.slice_group_change_cycle = 3;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);
	for (size_t i = 0; i < ARRAY_SIZE(expected_dir0); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected_dir0[i]);
	fmo_fixture_clear(&fx);

	pps.slice_group_change_direction_flag = 1;
	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);
	for (size_t i = 0; i < ARRAY_SIZE(expected_dir1); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected_dir1[i]);
	fmo_fixture_clear(&fx);
}


/* 8.2.2.6 (h264_slice_group_map_type_5, wipe): same 4x2 picture and
 * MapUnitsInSliceGroup0=3 as type 4, but the fill order is column-major
 * (index = i*PicWidthInMbs+j, j outer, i inner). Traced fill order (j,i):
 * (0,0)->idx0 k=0, (0,1)->idx4 k=1, (1,0)->idx1 k=2, (1,1)->idx5 k=3,
 * (2,0)->idx2 k=4, (2,1)->idx6 k=5, (3,0)->idx3 k=6, (3,1)->idx7 k=7;
 * k<3 -> group 0, else group 1 -> map[0,1,4]=0, map[2,3,5,6,7]=1. */
static void test_fmo_type5_wipe(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	struct h264_slice_header sh = {0};
	static const uint32_t expected[] = {0, 0, 1, 1, 0, 1, 1, 1};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 5;
	pps.slice_group_change_direction_flag = 0;
	pps.slice_group_change_rate_minus1 = 0;
	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);
	sh.slice_group_change_cycle = 3;
	CU_ASSERT_EQUAL_FATAL(h264_ctx_set_slice_header(fx.ctx, &sh), 0);

	for (size_t i = 0; i < ARRAY_SIZE(expected); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], expected[i]);

	fmo_fixture_clear(&fx);
}


/* 8.2.2.7 (h264_slice_group_map_type_6, explicit): a direct copy of
 * pps->slice_group_id[]. */
static void test_fmo_type6_explicit(void)
{
	struct fmo_fixture fx = {0};
	struct h264_pps pps = {0};
	static const uint32_t values[] = {0, 1, 0, 1, 0, 1, 0, 1};

	pps.num_slice_groups_minus1 = 1;
	pps.slice_group_map_type = 6;
	for (size_t i = 0; i < ARRAY_SIZE(values); i++)
		pps.slice_group_id[i] = values[i];

	CU_ASSERT_EQUAL_FATAL(fmo_fixture_setup(&fx, 4, 2, &pps), 0);

	for (size_t i = 0; i < ARRAY_SIZE(values); i++)
		CU_ASSERT_EQUAL(fx.ctx->slice.group_map[i], values[i]);

	fmo_fixture_clear(&fx);
}


/* h264_intlog2() (src/h264_priv.h): ceil(log2(x)), 0 for x == 0. */
static void test_intlog2(void)
{
	static const struct {
		uint32_t x;
		uint32_t expected;
	} cases[] = {
		{0, 0},
		{1, 0},
		{2, 1},
		{3, 2},
		{4, 2},
		{5, 3},
		{7, 3},
		{8, 3},
		{9, 4},
		{16, 4},
		{17, 5},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++)
		CU_ASSERT_EQUAL(h264_intlog2(cases[i].x), cases[i].expected);
}


/* h264_get_mb_addr_off() (src/h264_priv.h): mbAddr - first_mb_in_slice *
 * (1 + MbaffFrameFlag). Only touches ctx->slice.hdr/ctx->derived, no
 * SPS/PPS needed at all. */
static void test_get_mb_addr_off(void)
{
	struct h264_ctx *ctx = NULL;

	CU_ASSERT_EQUAL_FATAL(h264_ctx_new(&ctx), 0);

	ctx->slice.hdr.first_mb_in_slice = 5;
	ctx->derived.MbaffFrameFlag = 0;
	CU_ASSERT_EQUAL(h264_get_mb_addr_off(ctx, 10), 5);

	ctx->slice.hdr.first_mb_in_slice = 3;
	ctx->derived.MbaffFrameFlag = 1;
	CU_ASSERT_EQUAL(h264_get_mb_addr_off(ctx, 10), 4);

	h264_ctx_destroy(ctx);
}


CU_TestInfo g_h264_test_fmo[] = {
	{FN("type0-interleaved"), &test_fmo_type0_interleaved},
	{FN("type1-dispersed"), &test_fmo_type1_dispersed},
	{FN("type2-foreground-leftover"), &test_fmo_type2_foreground_leftover},
	{FN("type3-box-out"), &test_fmo_type3_box_out},
	{FN("type4-raster-scan"), &test_fmo_type4_raster_scan},
	{FN("type5-wipe"), &test_fmo_type5_wipe},
	{FN("type6-explicit"), &test_fmo_type6_explicit},
	{FN("next-mb-addr"), &test_next_mb_addr},
	{FN("clear-slice-group-map"), &test_clear_slice_group_map},
	{FN("intlog2"), &test_intlog2},
	{FN("get-mb-addr-off"), &test_get_mb_addr_off},

	CU_TEST_INFO_NULL,
};
