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

/* Exhaustive enum<->string checks below: each untested switch case in
 * src/h264_types.c is a source line lcov counts as uncovered, so walking
 * every enumerator (not just a representative sample) directly improves
 * line coverage. Macros key off the enumerator name itself (H264_NALU_TYPE_
 * ##_v etc.) so the expected string can never drift from the identifier. */
#define CHECK_NALU(_v)                                                         \
	CU_ASSERT_STRING_EQUAL(h264_nalu_type_str(H264_NALU_TYPE_##_v), #_v)
#define CHECK_SLICE(_v)                                                        \
	CU_ASSERT_STRING_EQUAL(h264_slice_type_str(H264_SLICE_TYPE_##_v), #_v)
#define CHECK_MB(_v)                                                           \
	CU_ASSERT_STRING_EQUAL(h264_mb_type_str(H264_MB_TYPE_##_v), #_v)
#define CHECK_PROFILE(_v)                                                      \
	CU_ASSERT_STRING_EQUAL(h264_profile_str(H264_PROFILE_##_v), #_v)
#define CHECK_COLOR(_v)                                                        \
	CU_ASSERT_STRING_EQUAL(h264_color_format_str(H264_COLOR_FORMAT_##_v),  \
			       #_v)
#define CHECK_SEI(_v)                                                          \
	CU_ASSERT_STRING_EQUAL(h264_sei_type_str(H264_SEI_TYPE_##_v), #_v)


static void test_nalu_type_str(void)
{
	CHECK_NALU(SLICE);
	CHECK_NALU(SLICE_DPA);
	CHECK_NALU(SLICE_DPB);
	CHECK_NALU(SLICE_DPC);
	CHECK_NALU(SLICE_IDR);
	CHECK_NALU(SEI);
	CHECK_NALU(SPS);
	CHECK_NALU(PPS);
	CHECK_NALU(AUD);
	CHECK_NALU(END_OF_SEQ);
	CHECK_NALU(END_OF_STREAM);
	CHECK_NALU(FILLER);
	CU_ASSERT_STRING_EQUAL(h264_nalu_type_str(H264_NALU_TYPE_UNKNOWN),
			       "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(h264_nalu_type_str((enum h264_nalu_type)999),
			       "UNKNOWN");
}


static void test_slice_type_str(void)
{
	CHECK_SLICE(P);
	CHECK_SLICE(B);
	CHECK_SLICE(I);
	CHECK_SLICE(SP);
	CHECK_SLICE(SI);
	CU_ASSERT_STRING_EQUAL(h264_slice_type_str(H264_SLICE_TYPE_UNKNOWN),
			       "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(h264_slice_type_str((enum h264_slice_type)999),
			       "UNKNOWN");
}


static void test_mb_type_str(void)
{
	CHECK_MB(I_NxN);
	CHECK_MB(I_16x16);
	CHECK_MB(I_PCM);
	CHECK_MB(SI);
	CHECK_MB(P_16x16);
	CHECK_MB(P_16x8);
	CHECK_MB(P_8x16);
	CHECK_MB(P_8x8);
	CHECK_MB(P_8x8ref0);
	CHECK_MB(P_SKIP);
	CHECK_MB(B_Direct_16x16);
	CHECK_MB(B_16x16);
	CHECK_MB(B_16x8);
	CHECK_MB(B_8x16);
	CHECK_MB(B_8x8);
	CHECK_MB(B_SKIP);
	CU_ASSERT_STRING_EQUAL(h264_mb_type_str(H264_MB_TYPE_UNKNOWN),
			       "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(h264_mb_type_str((enum h264_mb_type)999),
			       "UNKNOWN");
}


/* Exhaustive over every enum h264_mb_type value: intra and inter are
 * mutually exclusive predicates (SI counts as intra, matching 7.4.5). */
static void test_mb_type_is_intra_is_inter(void)
{
	static const enum h264_mb_type intra_types[] = {
		H264_MB_TYPE_I_NxN,
		H264_MB_TYPE_I_16x16,
		H264_MB_TYPE_I_PCM,
		H264_MB_TYPE_SI,
	};
	static const enum h264_mb_type inter_types[] = {
		H264_MB_TYPE_P_16x16,
		H264_MB_TYPE_P_16x8,
		H264_MB_TYPE_P_8x16,
		H264_MB_TYPE_P_8x8,
		H264_MB_TYPE_P_8x8ref0,
		H264_MB_TYPE_P_SKIP,
		H264_MB_TYPE_B_Direct_16x16,
		H264_MB_TYPE_B_16x16,
		H264_MB_TYPE_B_16x8,
		H264_MB_TYPE_B_8x16,
		H264_MB_TYPE_B_8x8,
		H264_MB_TYPE_B_SKIP,
	};

	for (size_t i = 0; i < ARRAY_SIZE(intra_types); i++) {
		CU_ASSERT_TRUE(h264_mb_type_is_intra(intra_types[i]));
		CU_ASSERT_FALSE(h264_mb_type_is_inter(intra_types[i]));
	}
	for (size_t i = 0; i < ARRAY_SIZE(inter_types); i++) {
		CU_ASSERT_FALSE(h264_mb_type_is_intra(inter_types[i]));
		CU_ASSERT_TRUE(h264_mb_type_is_inter(inter_types[i]));
	}
	CU_ASSERT_FALSE(h264_mb_type_is_intra(H264_MB_TYPE_UNKNOWN));
	CU_ASSERT_FALSE(h264_mb_type_is_inter(H264_MB_TYPE_UNKNOWN));
}


static void test_profile_str(void)
{
	CHECK_PROFILE(CAVLC_444);
	CHECK_PROFILE(BASELINE);
	CHECK_PROFILE(MAIN);
	CHECK_PROFILE(EXTENDED);
	CHECK_PROFILE(HIGH);
	CHECK_PROFILE(HIGH_10);
	CHECK_PROFILE(HIGH_422);
	CHECK_PROFILE(HIGH_444);
	CU_ASSERT_STRING_EQUAL(h264_profile_str((enum h264_profile)0),
			       "UNKNOWN");
}


static void test_color_format_str(void)
{
	CHECK_COLOR(MONO);
	CHECK_COLOR(YUV420);
	CHECK_COLOR(YUV422);
	CHECK_COLOR(YUV444);
	CU_ASSERT_STRING_EQUAL(
		h264_color_format_str((enum h264_color_format)99), "UNKNOWN");
}


static void test_aspect_ratio_str_alloc(void)
{
	static const struct {
		enum h264_aspect_ratio val;
		const char *expected;
	} cases[] = {
		{H264_ASPECT_RATIO_UNSPECIFIED, "UNSPECIFIED"},
		{H264_ASPECT_RATIO_1_1, "1:1"},
		{H264_ASPECT_RATIO_12_11, "12:11"},
		{H264_ASPECT_RATIO_10_11, "10:11"},
		{H264_ASPECT_RATIO_16_11, "16:11"},
		{H264_ASPECT_RATIO_40_33, "40:33"},
		{H264_ASPECT_RATIO_24_11, "24:11"},
		{H264_ASPECT_RATIO_20_11, "20:11"},
		{H264_ASPECT_RATIO_32_11, "32:11"},
		{H264_ASPECT_RATIO_80_33, "80:33"},
		{H264_ASPECT_RATIO_18_11, "18:11"},
		{H264_ASPECT_RATIO_15_11, "15:11"},
		{H264_ASPECT_RATIO_64_33, "64:33"},
		{H264_ASPECT_RATIO_160_99, "160:99"},
		{H264_ASPECT_RATIO_4_3, "4:3"},
		{H264_ASPECT_RATIO_3_2, "3:2"},
		{H264_ASPECT_RATIO_2_1, "2:1"},
	};
	char *str;

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		str = h264_aspect_ratio_str_alloc(cases[i].val, 0, 0);
		CU_ASSERT_STRING_EQUAL(str, cases[i].expected);
		free(str);
	}

	str = h264_aspect_ratio_str_alloc(H264_ASPECT_RATIO_EXTENDED_SAR, 7, 5);
	CU_ASSERT_STRING_EQUAL(str, "EXTENDED_SAR_7:5");
	free(str);

	str = h264_aspect_ratio_str_alloc((enum h264_aspect_ratio)123, 0, 0);
	CU_ASSERT_STRING_EQUAL(str, "UNKNOWN");
	free(str);
}


/* h264_sei_type_str(): exhaustive over every enum h264_sei_type value. Also
 * documents the library's current (buggy) behavior on an unrecognized value
 * -- the default branch in src/h264_types.c returns the misspelled
 * "UNKNONW" rather than "UNKNOWN". This characterizes what the code does
 * today; it is not asserting that this is desired behavior. */
static void test_sei_type_str(void)
{
	CHECK_SEI(BUFFERING_PERIOD);
	CHECK_SEI(PIC_TIMING);
	CHECK_SEI(PAN_SCAN_RECT);
	CHECK_SEI(FILLER_PAYLOAD);
	CHECK_SEI(USER_DATA_REGISTERED);
	CHECK_SEI(USER_DATA_UNREGISTERED);
	CHECK_SEI(RECOVERY_POINT);
	CHECK_SEI(DEC_REF_PIC_MARKING_REPETITION);
	CHECK_SEI(SPARE_PIC);
	CHECK_SEI(SCENE_INFO);
	CHECK_SEI(SUB_SEQ_INFO);
	CHECK_SEI(SUB_SEQ_LAYER_CHARACTERISTICS);
	CHECK_SEI(SUB_SEQ_CHARACTERISTICS);
	CHECK_SEI(FULL_FRAME_FREEZE);
	CHECK_SEI(FULL_FRAME_FREEZE_RELEASE);
	CHECK_SEI(FULL_FRAME_SNAPSHOT);
	CHECK_SEI(PROGRESSIVE_REFINEMENT_SEGMENT_START);
	CHECK_SEI(PROGRESSIVE_REFINEMENT_SEGMENT_END);
	CHECK_SEI(MOTION_CONSTRAINED_SLICE_GROUP_SET);
	CHECK_SEI(FILM_GRAIN_CHARACTERISTICS);
	CHECK_SEI(DEBLOCKING_FILTER_DISPLAY_PREFERENCE);
	CHECK_SEI(STEREO_VIDEO_INFO);
	CHECK_SEI(POST_FILTER_HINT);
	CHECK_SEI(TONE_MAPPING_INFO);
	CHECK_SEI(SCALABILITY_INFO);
	CHECK_SEI(SUB_PIC_SCALABLE_LAYER);
	CHECK_SEI(NON_REQUIRED_LAYER_REP);
	CHECK_SEI(PRIORITY_LAYER_INFO);
	CHECK_SEI(LAYERS_NOT_PRESENT);
	CHECK_SEI(LAYER_DEPENDENCY_CHANGE);
	CHECK_SEI(SCALABLE_NESTING);
	CHECK_SEI(BASE_LAYER_TEMPORAL_HRD);
	CHECK_SEI(QUALITY_LAYER_INTEGRITY_CHECK);
	CHECK_SEI(REDUNDANT_PIC_PROPERTY);
	CHECK_SEI(TL0_DEP_REP_INDEX);
	CHECK_SEI(TL_SWITCHING_POINT);
	CHECK_SEI(PARALLEL_DECODING_INFO);
	CHECK_SEI(MVC_SCALABLE_NESTING);
	CHECK_SEI(VIEW_SCALABILITY_INFO);
	CHECK_SEI(MULTIVIEW_SCENE_INFO);
	CHECK_SEI(MULTIVIEW_ACQUISITION_INFO);
	CHECK_SEI(NON_REQUIRED_VIEW_COMPONENT);
	CHECK_SEI(VIEW_DEPENDENCY_CHANGE);
	CHECK_SEI(OPERATION_POINTS_NOT_PRESENT);
	CHECK_SEI(BASE_VIEW_TEMPORAL_HRD);
	CHECK_SEI(FRAME_PACKING_ARRANGEMENT);
	CHECK_SEI(MULTIVIEW_VIEW_POSITION);
	CHECK_SEI(DISPLAY_ORIENTATION);
	CHECK_SEI(MVCD_SCALABLE_NESTING);
	CHECK_SEI(MVCD_VIEW_SCALABILITY_INFO);
	CHECK_SEI(DEPTH_REPRESENTATION_INFO);
	CHECK_SEI(THREE_DIMENSIONAL_REFERENCE_DISPLAYS_INFO);
	CHECK_SEI(DEPTH_TIMING);
	CHECK_SEI(DEPTH_SAMPLING_INFO);
	CHECK_SEI(CONSTRAINED_DEPTH_PARAMETER_SET_IDENTIFIER);
	CU_ASSERT_STRING_EQUAL(h264_sei_type_str((enum h264_sei_type)999),
			       "UNKNONW");
}


static void test_sar_to_aspect_ratio_idc(void)
{
	/* idx 0 and 1 both map to {1,1} in the internal table; the search
	 * starts at idx 1, so a 1:1 SAR resolves to H264_ASPECT_RATIO_1_1. */
	CU_ASSERT_EQUAL(h264_sar_to_aspect_ratio_idc(1, 1),
			H264_ASPECT_RATIO_1_1);
	CU_ASSERT_EQUAL(h264_sar_to_aspect_ratio_idc(4, 3),
			H264_ASPECT_RATIO_4_3);
	CU_ASSERT_EQUAL(h264_sar_to_aspect_ratio_idc(2, 1),
			H264_ASPECT_RATIO_2_1);
	CU_ASSERT_EQUAL(h264_sar_to_aspect_ratio_idc(7, 5),
			H264_ASPECT_RATIO_EXTENDED_SAR);
}


static void test_get_sps_derived_null_args(void)
{
	struct h264_sps sps = {0};
	struct h264_sps_derived derived;

	CU_ASSERT_EQUAL(h264_get_sps_derived(NULL, &derived), -EINVAL);
	CU_ASSERT_EQUAL(h264_get_sps_derived(&sps, NULL), -EINVAL);
}


/* A hand-filled 160x160 YUV420 SPS (10x10 macroblocks, no cropping). */
static void test_get_sps_derived_yuv420(void)
{
	struct h264_sps sps = {0};
	struct h264_sps_derived derived;
	int res;

	sps.chroma_format_idc = 1; /* YUV420 */
	sps.bit_depth_luma_minus8 = 0;
	sps.bit_depth_chroma_minus8 = 0;
	sps.log2_max_frame_num_minus4 = 0;
	sps.log2_max_pic_order_cnt_lsb_minus4 = 0;
	sps.pic_width_in_mbs_minus1 = 9;
	sps.pic_height_in_map_units_minus1 = 9;
	sps.frame_mbs_only_flag = 1;

	res = h264_get_sps_derived(&sps, &derived);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(derived.ChromaArrayType, H264_COLOR_FORMAT_YUV420);
	CU_ASSERT_EQUAL(derived.SubWidthC, 2);
	CU_ASSERT_EQUAL(derived.SubHeightC, 2);
	CU_ASSERT_EQUAL(derived.MbWidthC, 8);
	CU_ASSERT_EQUAL(derived.MbHeightC, 8);
	CU_ASSERT_EQUAL(derived.BitDepthLuma, 8);
	CU_ASSERT_EQUAL(derived.BitDepthChroma, 8);
	CU_ASSERT_EQUAL(derived.MaxFrameNum, 16);
	CU_ASSERT_EQUAL(derived.MaxPicOrderCntLsb, 16);
	CU_ASSERT_EQUAL(derived.PicWidthInMbs, 10);
	CU_ASSERT_EQUAL(derived.PicHeightInMapUnits, 10);
	CU_ASSERT_EQUAL(derived.PicSizeInMapUnits, 100);
	CU_ASSERT_EQUAL(derived.FrameHeightInMbs, 10);
	CU_ASSERT_EQUAL(derived.Width, 160);
	CU_ASSERT_EQUAL(derived.Height, 160);
}


CU_TestInfo g_h264_test_types[] = {
	{FN("nalu-type-str"), &test_nalu_type_str},
	{FN("slice-type-str"), &test_slice_type_str},
	{FN("mb-type-str"), &test_mb_type_str},
	{FN("mb-type-is-intra-is-inter"), &test_mb_type_is_intra_is_inter},
	{FN("profile-str"), &test_profile_str},
	{FN("color-format-str"), &test_color_format_str},
	{FN("aspect-ratio-str-alloc"), &test_aspect_ratio_str_alloc},
	{FN("sei-type-str"), &test_sei_type_str},
	{FN("sar-to-aspect-ratio-idc"), &test_sar_to_aspect_ratio_idc},
	{FN("get-sps-derived-null-args"), &test_get_sps_derived_null_args},
	{FN("get-sps-derived-yuv420"), &test_get_sps_derived_yuv420},

	CU_TEST_INFO_NULL,
};
