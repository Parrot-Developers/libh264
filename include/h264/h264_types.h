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

#ifndef _H264_TYPES_H_
#define _H264_TYPES_H_


/**
 * 7.4.1 NAL unit semantics
 */
enum h264_nalu_type {
	H264_NALU_TYPE_UNKNOWN = 0,
	H264_NALU_TYPE_SLICE = 1,
	H264_NALU_TYPE_SLICE_DPA = 2,
	H264_NALU_TYPE_SLICE_DPB = 3,
	H264_NALU_TYPE_SLICE_DPC = 4,
	H264_NALU_TYPE_SLICE_IDR = 5,
	H264_NALU_TYPE_SEI = 6,
	H264_NALU_TYPE_SPS = 7,
	H264_NALU_TYPE_PPS = 8,
	H264_NALU_TYPE_AUD = 9,
	H264_NALU_TYPE_END_OF_SEQ = 10,
	H264_NALU_TYPE_END_OF_STREAM = 11,
	H264_NALU_TYPE_FILLER = 12,
};


/**
 * 7.4.3 Slice header semantics
 */
enum h264_slice_type {
	/* Unknown slice type or non-VCL NAL unit */
	H264_SLICE_TYPE_UNKNOWN = -1,
	H264_SLICE_TYPE_P = 0,
	H264_SLICE_TYPE_B = 1,
	H264_SLICE_TYPE_I = 2,
	H264_SLICE_TYPE_SP = 3,
	H264_SLICE_TYPE_SI = 4,
};

#define H264_SLICE_TYPE(_val) ((_val) % 5)


/**
 * 7.4.5 Macroblock layer semantics
 * Actual value is not part of the specification
 * UNKNOWN is also not part of the specification but added for convenience
 */
enum h264_mb_type {
	H264_MB_TYPE_UNKNOWN = 0,

	H264_MB_TYPE_I_NxN,
	H264_MB_TYPE_I_16x16,
	H264_MB_TYPE_I_PCM,
	H264_MB_TYPE_SI,
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


/**
 * A.2 Profiles
 */
enum h264_profile {
	H264_PROFILE_CAVLC_444 = 44,
	H264_PROFILE_BASELINE = 66,
	H264_PROFILE_MAIN = 77,
	H264_PROFILE_EXTENDED = 88,
	H264_PROFILE_HIGH = 100,
	H264_PROFILE_HIGH_10 = 110,
	H264_PROFILE_HIGH_422 = 122,
	H264_PROFILE_HIGH_444 = 244,
};


/**
 * 6.2 Source, decoded, and output picture formats
 */
enum h264_color_format {
	/* Monochrome */
	H264_COLOR_FORMAT_MONO = 0,

	/* 4:2:0 */
	H264_COLOR_FORMAT_YUV420 = 1,

	/* 4:2:2 */
	H264_COLOR_FORMAT_YUV422 = 2,

	/* 4:4:4 */
	H264_COLOR_FORMAT_YUV444 = 3,
};


/**
 * E.2.1 Sample aspect ratio indicator.
 */
enum h264_aspect_ratio {
	H264_ASPECT_RATIO_UNSPECIFIED = 0,
	H264_ASPECT_RATIO_1_1 = 1,
	H264_ASPECT_RATIO_12_11 = 2,
	H264_ASPECT_RATIO_10_11 = 3,
	H264_ASPECT_RATIO_16_11 = 4,
	H264_ASPECT_RATIO_40_33 = 5,
	H264_ASPECT_RATIO_24_11 = 6,
	H264_ASPECT_RATIO_20_11 = 7,
	H264_ASPECT_RATIO_32_11 = 8,
	H264_ASPECT_RATIO_80_33 = 9,
	H264_ASPECT_RATIO_18_11 = 10,
	H264_ASPECT_RATIO_15_11 = 11,
	H264_ASPECT_RATIO_64_33 = 12,
	H264_ASPECT_RATIO_160_99 = 13,
	H264_ASPECT_RATIO_4_3 = 14,
	H264_ASPECT_RATIO_3_2 = 15,
	H264_ASPECT_RATIO_2_1 = 16,
	/* 17 .. 254 reserved */
	H264_ASPECT_RATIO_EXTENDED_SAR = 255,
};


/**
 * D.1 SEI payload syntax
 */
enum h264_sei_type {
	H264_SEI_TYPE_BUFFERING_PERIOD = 0,
	H264_SEI_TYPE_PIC_TIMING = 1,
	H264_SEI_TYPE_PAN_SCAN_RECT = 2,
	H264_SEI_TYPE_FILLER_PAYLOAD = 3,
	H264_SEI_TYPE_USER_DATA_REGISTERED = 4,
	H264_SEI_TYPE_USER_DATA_UNREGISTERED = 5,
	H264_SEI_TYPE_RECOVERY_POINT = 6,
	H264_SEI_TYPE_DEC_REF_PIC_MARKING_REPETITION = 7,
	H264_SEI_TYPE_SPARE_PIC = 8,
	H264_SEI_TYPE_SCENE_INFO = 9,
	H264_SEI_TYPE_SUB_SEQ_INFO = 10,
	H264_SEI_TYPE_SUB_SEQ_LAYER_CHARACTERISTICS = 11,
	H264_SEI_TYPE_SUB_SEQ_CHARACTERISTICS = 12,
	H264_SEI_TYPE_FULL_FRAME_FREEZE = 13,
	H264_SEI_TYPE_FULL_FRAME_FREEZE_RELEASE = 14,
	H264_SEI_TYPE_FULL_FRAME_SNAPSHOT = 15,
	H264_SEI_TYPE_PROGRESSIVE_REFINEMENT_SEGMENT_START = 16,
	H264_SEI_TYPE_PROGRESSIVE_REFINEMENT_SEGMENT_END = 17,
	H264_SEI_TYPE_MOTION_CONSTRAINED_SLICE_GROUP_SET = 18,
	H264_SEI_TYPE_FILM_GRAIN_CHARACTERISTICS = 19,
	H264_SEI_TYPE_DEBLOCKING_FILTER_DISPLAY_PREFERENCE = 20,
	H264_SEI_TYPE_STEREO_VIDEO_INFO = 21,
	H264_SEI_TYPE_POST_FILTER_HINT = 22,
	H264_SEI_TYPE_TONE_MAPPING_INFO = 23,
	H264_SEI_TYPE_SCALABILITY_INFO = 24,
	H264_SEI_TYPE_SUB_PIC_SCALABLE_LAYER = 25,
	H264_SEI_TYPE_NON_REQUIRED_LAYER_REP = 26,
	H264_SEI_TYPE_PRIORITY_LAYER_INFO = 27,
	H264_SEI_TYPE_LAYERS_NOT_PRESENT = 28,
	H264_SEI_TYPE_LAYER_DEPENDENCY_CHANGE = 29,
	H264_SEI_TYPE_SCALABLE_NESTING = 30,
	H264_SEI_TYPE_BASE_LAYER_TEMPORAL_HRD = 31,
	H264_SEI_TYPE_QUALITY_LAYER_INTEGRITY_CHECK = 32,
	H264_SEI_TYPE_REDUNDANT_PIC_PROPERTY = 33,
	H264_SEI_TYPE_TL0_DEP_REP_INDEX = 34,
	H264_SEI_TYPE_TL_SWITCHING_POINT = 35,
	H264_SEI_TYPE_PARALLEL_DECODING_INFO = 36,
	H264_SEI_TYPE_MVC_SCALABLE_NESTING = 37,
	H264_SEI_TYPE_VIEW_SCALABILITY_INFO = 38,
	H264_SEI_TYPE_MULTIVIEW_SCENE_INFO = 39,
	H264_SEI_TYPE_MULTIVIEW_ACQUISITION_INFO = 40,
	H264_SEI_TYPE_NON_REQUIRED_VIEW_COMPONENT = 41,
	H264_SEI_TYPE_VIEW_DEPENDENCY_CHANGE = 42,
	H264_SEI_TYPE_OPERATION_POINTS_NOT_PRESENT = 43,
	H264_SEI_TYPE_BASE_VIEW_TEMPORAL_HRD = 44,
	H264_SEI_TYPE_FRAME_PACKING_ARRANGEMENT = 45,
	H264_SEI_TYPE_MULTIVIEW_VIEW_POSITION = 46,
	H264_SEI_TYPE_DISPLAY_ORIENTATION = 47,
	H264_SEI_TYPE_MVCD_SCALABLE_NESTING = 48,
	H264_SEI_TYPE_MVCD_VIEW_SCALABILITY_INFO = 49,
	H264_SEI_TYPE_DEPTH_REPRESENTATION_INFO = 50,
	H264_SEI_TYPE_THREE_DIMENSIONAL_REFERENCE_DISPLAYS_INFO = 51,
	H264_SEI_TYPE_DEPTH_TIMING = 52,
	H264_SEI_TYPE_DEPTH_SAMPLING_INFO = 53,
	H264_SEI_TYPE_CONSTRAINED_DEPTH_PARAMETER_SET_IDENTIFIER = 54,
};


/**
 * 7.3.1 NAL unit syntax
 */
struct h264_nalu_header {
	uint32_t forbidden_zero_bit;
	uint32_t nal_ref_idc;
	uint32_t nal_unit_type;
};


/**
 * 7.3.2.1 Sequence parameter set RBSP syntax
 * 7.3.2.2 Picture parameter set RBSP syntax
 */
struct h264_scaling_matrix {
	int scaling_list_present_flag[12];
	int32_t scaling_list_4x4[6][16];
	int32_t scaling_list_8x8[6][64];
	int use_default_4x4[6];
	int use_default_8x8[6];

	int _optimized_4x4[6];
	int _optimized_8x8[6];
};


/**
 * E.1.2 HRD parameters syntax
 */
struct h264_hrd {
	uint32_t cpb_cnt_minus1; /* Range = 0-31 */
	uint32_t bit_rate_scale;
	uint32_t cpb_size_scale;

	/* Size is cpb_cnt_minus1 + 1 */
	struct {
		uint32_t bit_rate_value_minus1;
		uint32_t cpb_size_value_minus1;
		int cbr_flag;
	} cpb[32];

	uint32_t initial_cpb_removal_delay_length_minus1;
	uint32_t cpb_removal_delay_length_minus1;
	uint32_t dpb_output_delay_length_minus1;
	uint32_t time_offset_length;
};


/**
 * E.1.1 VUI parameters syntax
 */
struct h264_vui {
	int aspect_ratio_info_present_flag;
	uint32_t aspect_ratio_idc;
	uint32_t sar_width;
	uint32_t sar_height;
	int overscan_info_present_flag;
	int overscan_appropriate_flag;
	int video_signal_type_present_flag;
	uint32_t video_format;
	int video_full_range_flag;
	int colour_description_present_flag;
	uint32_t colour_primaries;
	uint32_t transfer_characteristics;
	uint32_t matrix_coefficients;
	int chroma_loc_info_present_flag;
	uint32_t chroma_sample_loc_type_top_field;
	uint32_t chroma_sample_loc_type_bottom_field;
	int timing_info_present_flag;
	uint32_t num_units_in_tick;
	uint32_t time_scale;
	int fixed_frame_rate_flag;
	int nal_hrd_parameters_present_flag;
	struct h264_hrd nal_hrd;
	int vcl_hrd_parameters_present_flag;
	struct h264_hrd vcl_hrd;
	int low_delay_hrd_flag;
	int pic_struct_present_flag;
	int bitstream_restriction_flag;
	int motion_vectors_over_pic_boundaries_flag;
	uint32_t max_bytes_per_pic_denom;
	uint32_t max_bits_per_mb_denom;
	uint32_t log2_max_mv_length_horizontal;
	uint32_t log2_max_mv_length_vertical;
	uint32_t max_num_reorder_frames;
	uint32_t max_dec_frame_buffering;
};


/**
 * 7.3.2.1 Sequence parameter set RBSP syntax
 */
struct h264_sps {
	uint32_t profile_idc;
	int constraint_set0_flag;
	int constraint_set1_flag;
	int constraint_set2_flag;
	int constraint_set3_flag;
	int constraint_set4_flag;
	int constraint_set5_flag;
	uint32_t reserved_zero_2bits;
	uint32_t level_idc;
	uint32_t seq_parameter_set_id;
	uint32_t chroma_format_idc;
	int separate_colour_plane_flag;
	uint32_t bit_depth_luma_minus8;
	uint32_t bit_depth_chroma_minus8;
	int qpprime_y_zero_transform_bypass_flag;
	int seq_scaling_matrix_present_flag;
	struct h264_scaling_matrix seq_scaling_matrix;
	uint32_t log2_max_frame_num_minus4;
	uint32_t pic_order_cnt_type;
	uint32_t log2_max_pic_order_cnt_lsb_minus4;
	int delta_pic_order_always_zero_flag;
	int32_t offset_for_non_ref_pic;
	int32_t offset_for_top_to_bottom_field;

	/* Range is 0-255 */
	uint32_t num_ref_frames_in_pic_order_cnt_cycle;

	/* Size is num_ref_frames_in_pic_order_cnt_cycle */
	int32_t offset_for_ref_frame[256];

	uint32_t max_num_ref_frames;
	int gaps_in_frame_num_value_allowed_flag;
	uint32_t pic_width_in_mbs_minus1;
	uint32_t pic_height_in_map_units_minus1;
	int frame_mbs_only_flag;
	int mb_adaptive_frame_field_flag;
	int direct_8x8_inference_flag;
	int frame_cropping_flag;
	uint32_t frame_crop_left_offset;
	uint32_t frame_crop_right_offset;
	uint32_t frame_crop_top_offset;
	uint32_t frame_crop_bottom_offset;
	int vui_parameters_present_flag;
	struct h264_vui vui;
};


/**
 * 7.3.2.2 Picture parameter set RBSP syntax
 */
struct h264_pps {
	uint32_t pic_parameter_set_id;
	uint32_t seq_parameter_set_id;
	int entropy_coding_mode_flag;
	int bottom_field_pic_order_in_frame_present_flag;

	/* Range is 0-7 */
	uint32_t num_slice_groups_minus1;

	uint32_t slice_group_map_type;

	/* Size is num_slice_groups_minus + 1 */
	uint32_t run_length_minus1[8];
	uint32_t top_left[8];
	uint32_t bottom_right[8];

	int slice_group_change_direction_flag;
	uint32_t slice_group_change_rate_minus1;
	uint32_t pic_size_in_map_units_minus1;

	/* Size is pic_size_in_map_units_minus1 + 1 */
	/* TODO: no upper limit in spec */
	uint32_t slice_group_id[256];

	uint32_t num_ref_idx_l0_default_active_minus1;
	uint32_t num_ref_idx_l1_default_active_minus1;
	int weighted_pred_flag;
	uint32_t weighted_bipred_idc;
	int32_t pic_init_qp_minus26;
	int32_t pic_init_qs_minus26;
	int32_t chroma_qp_index_offset;
	int deblocking_filter_control_present_flag;
	int constrained_intra_pred_flag;
	int redundant_pic_cnt_present_flag;

	int _more_rbsp_data_present;
	int transform_8x8_mode_flag;
	int pic_scaling_matrix_present_flag;
	struct h264_scaling_matrix pic_scaling_matrix;
	int32_t second_chroma_qp_index_offset;
};


/**
 * 7.3.2.4 Access unit delimiter RBSP syntax
 */
struct h264_aud {
	uint32_t primary_pic_type;
};


/**
 * 7.3.3.1 Reference picture list modification syntax
 * H.7.3.3.1.1 Reference picture list MVC modification syntax
 */
struct h264_rplm_item {
	uint32_t modification_of_pic_nums_idc;
	union {
		uint32_t abs_diff_pic_num_minus1;
		uint32_t long_term_pic_num;
		uint32_t abs_diff_view_idx_minus1;
	};
};


/**
 * 7.3.3.1 Reference picture list modification syntax
 * H.7.3.3.1.1 Reference picture list MVC modification syntax
 */
struct h264_rplm {
	int ref_pic_list_modification_flag_l0;

	/* Max is num_ref_idx_l0_active_minus1 + 1 */
	struct h264_rplm_item pic_num_l0[32];

	int ref_pic_list_modification_flag_l1;

	/* Max is num_ref_idx_l1_active_minus1 + 1 */
	struct h264_rplm_item pic_num_l1[32];
};


/**
 * 7.3.3.2 Prediction weight table syntax
 */
struct h264_pwt_item {
	int luma_weight_flag;
	int32_t luma_weight;
	int32_t luma_offset;
	int chroma_weight_flag;
	int32_t chroma_weight[2];
	int32_t chroma_offset[2];
};


/**
 * 7.3.3.2 Prediction weight table syntax
 */
struct h264_pwt {
	uint32_t luma_log2_weight_denom;
	uint32_t chroma_log2_weight_denom;

	/* Size is num_ref_idx_l0_active_minus1 + 1 */
	struct h264_pwt_item l0[32];

	/* Size is num_ref_idx_l1_active_minus1 + 1 */
	struct h264_pwt_item l1[32];
};


/**
 * 7.3.3.3 Decoded reference picture marking syntax
 */
struct h264_drpm_item {
	uint32_t memory_management_control_operation;
	uint32_t difference_of_pic_nums_minus1;
	uint32_t long_term_pic_num;
	uint32_t long_term_frame_idx;
	uint32_t max_long_term_frame_idx_plus1;
};


/**
 * 7.3.3.3 Decoded reference picture marking syntax
 */
struct h264_drpm {
	int no_output_of_prior_pics_flag;
	int long_term_reference_flag;
	int adaptive_ref_pic_marking_mode_flag;

	/* TODO: no upper limit in spec */
	struct h264_drpm_item mm[64];
};


/**
 * 7.3.3 Slice header syntax
 */
struct h264_slice_header {
	uint32_t first_mb_in_slice;
	uint32_t slice_type;
	uint32_t pic_parameter_set_id;
	uint32_t colour_plane_id;
	uint32_t frame_num;
	int field_pic_flag;
	int bottom_field_flag;
	uint32_t idr_pic_id;
	uint32_t pic_order_cnt_lsb;
	int32_t delta_pic_order_cnt_bottom;
	int32_t delta_pic_order_cnt[2];
	uint32_t redundant_pic_cnt;
	int direct_spatial_mv_pred_flag;
	int num_ref_idx_active_override_flag;

	/* Range is 0-31 */
	uint32_t num_ref_idx_l0_active_minus1;
	uint32_t num_ref_idx_l1_active_minus1;

	struct h264_rplm rplm;
	struct h264_pwt pwt;
	struct h264_drpm drpm;

	uint32_t cabac_init_idc;
	int32_t slice_qp_delta;
	int sp_for_switch_flag;
	int32_t slice_qs_delta;
	uint32_t disable_deblocking_filter_idc;
	int32_t slice_alpha_c0_offset_div2;
	int32_t slice_beta_offset_div2;
	uint32_t slice_group_change_cycle;
};


/**
 * D.1.1 Buffering period SEI message syntax
 */
struct h264_sei_buffering_period {
	uint32_t seq_parameter_set_id;

	/* Size is cpb_cnt_minus1 + 1 */
	struct {
		uint32_t initial_cpb_removal_delay;
		uint32_t initial_cpb_removal_delay_offset;
	} nal_hrd_cpb[32];

	/* Size is cpb_cnt_minus1 + 1 */
	struct {
		uint32_t initial_cpb_removal_delay;
		uint32_t initial_cpb_removal_delay_offset;
	} vcl_hrd_cpb[32];
};


/**
 * D.1.2 Picture timing SEI message syntax
 */
struct h264_sei_pic_timing {
	uint32_t cpb_removal_delay;
	uint32_t dpb_output_delay;
	uint32_t pic_struct;
	struct {
		int clock_timestamp_flag;
		uint32_t ct_type;
		int nuit_field_based_flag;
		uint32_t counting_type;
		int full_timestamp_flag;
		int discontinuity_flag;
		int cnt_dropped_flag;
		uint32_t n_frames;
		uint32_t seconds_value;
		uint32_t minutes_value;
		uint32_t hours_value;
		int seconds_flag;
		int minutes_flag;
		int hours_flag;
		int32_t time_offset;
	} clk_ts[3];
};


/**
 * D.1.3 Pan-scan rectangle SEI message syntax
 */
struct h264_sei_pan_scan_rect {
	uint32_t pan_scan_rect_id;
	int pan_scan_rect_cancel_flag;

	/* Range is 0-2 */
	uint32_t pan_scan_cnt_minus1;

	/* Size is pan_scan_cnt_minus1 + 1 */
	struct {
		int32_t left_offset;
		int32_t right_offset;
		int32_t top_offset;
		int32_t bottom_offset;
	} pan_scan_rect[4];

	uint32_t pan_scan_rect_repetition_period;
};


/**
 * D.1.4 Filler payload SEI message syntax
 */
struct h264_sei_filler_payload {
	const uint8_t *buf;
	size_t len;
};


/**
 * D.1.5 User data registered by Rec. ITU-T T.35 SEI message syntax
 */
struct h264_sei_user_data_registered {
	uint32_t country_code;
	uint32_t country_code_extension_byte;
	const uint8_t *buf;
	size_t len;
};


/**
 * D.1.6 User data unregistered SEI message syntax
 */
struct h264_sei_user_data_unregistered {
	uint8_t uuid[16];
	const uint8_t *buf;
	size_t len;
};


/**
 * D.1.7 Recovery point SEI message syntax
 */
struct h264_sei_recovery_point {
	uint32_t recovery_frame_cnt;
	int exact_match_flag;
	int broken_link_flag;
	uint32_t changing_slice_group_idc;
};


struct h264_sei {
	enum h264_sei_type type;

	union {
		struct h264_sei_buffering_period buffering_period;
		struct h264_sei_pic_timing pic_timing;
		struct h264_sei_pan_scan_rect pan_scan_rect;
		struct h264_sei_filler_payload filler_payload;
		struct h264_sei_user_data_registered user_data_registered;
		struct h264_sei_user_data_unregistered user_data_unregistered;
		struct h264_sei_recovery_point recovery_point;
	};

	/* Internal use only */
	struct {
		uint8_t *buf;
		size_t len;
	} raw;
};


/* Derived from SPS */
struct h264_sps_derived {
	uint32_t ChromaArrayType;
	uint32_t SubWidthC;
	uint32_t SubHeightC;
	uint32_t MbWidthC;
	uint32_t MbHeightC;
	uint32_t BitDepthLuma;
	uint32_t QpBdOffsetLuma;
	uint32_t BitDepthChroma;
	uint32_t QpBdOffsetChroma;
	uint32_t RawMbBits;
	uint32_t MaxFrameNum;
	uint32_t MaxPicOrderCntLsb;
	uint32_t PicWidthInMbs;
	uint32_t PicWidthInSamplesLuma;
	uint32_t PicWidthInSamplesChroma;
	uint32_t PicHeightInMapUnits;
	uint32_t PicSizeInMapUnits;
	uint32_t FrameHeightInMbs;
	uint32_t CropUnitX;
	uint32_t CropUnitY;

	uint32_t Width;
	uint32_t Height;
};


/* Extra info from SPS & PPS */
struct h264_info {
	/* Picture width in pixels */
	uint32_t width;

	/* Picture height in pixels */
	uint32_t height;

	/* Luma bit depth */
	uint8_t bit_depth_luma;

	/* Source aspect ratio width (1 if unknown) */
	uint32_t sar_width;

	/* Source aspect ratio width (1 if unknown) */
	uint32_t sar_height;

	/* Left crop in pixels */
	uint32_t crop_left;

	/* Top crop in pixels */
	uint32_t crop_top;

	/* Crop width in pixels (equal to picture width if no crop) */
	uint32_t crop_width;

	/* Crop height in pixels (equal to picture height if no crop) */
	uint32_t crop_height;

	/* Full range flag */
	int full_range;

	/* 1: colour_primaries, transfer_characteristics and
	 * matrix_coefficients are valid; 0 otherwise */
	int colour_description_present;

	/* Colour primaries */
	uint32_t colour_primaries;

	/* Transfer characteristics */
	uint32_t transfer_characteristics;

	/* Matrix coefficients */
	uint32_t matrix_coefficients;

	/* Number of time units of a clock tick (0 if unknown) */
	uint32_t num_units_in_tick;

	/* Number of time units in one second (0 if unknown) */
	uint32_t time_scale;

	/* Declared framerate from time_scale and num_units_in_tick
	 * (0 if unknown) */
	float framerate;

	/* Declared framerate from time_scale and num_units_in_tick
	 * (0 if unknown) - numerator */
	uint32_t framerate_num;

	/* Declared framerate from time_scale and num_units_in_tick
	 * (0 if unknown) - denominator */
	uint32_t framerate_den;

	/* NAL HRD bitrate (0 if unknown) */
	uint32_t nal_hrd_bitrate;

	/* NAL HRD CPB size (0 if unknown) */
	uint32_t nal_hrd_cpb_size;

	/* VCL HRD bitrate (0 if unknown) */
	uint32_t vcl_hrd_bitrate;

	/* VCL HRD CPB size (0 if unknown) */
	uint32_t vcl_hrd_cpb_size;
};


H264_API
const char *h264_nalu_type_str(enum h264_nalu_type val);


H264_API
const char *h264_slice_type_str(enum h264_slice_type val);


H264_API
const char *h264_mb_type_str(enum h264_mb_type val);


H264_API
int h264_mb_type_is_intra(enum h264_mb_type val);


H264_API
int h264_mb_type_is_inter(enum h264_mb_type val);


H264_API
const char *h264_profile_str(enum h264_profile val);


H264_API
const char *h264_color_format_str(enum h264_color_format val);


H264_API
char *h264_aspect_ratio_str_alloc(enum h264_aspect_ratio val,
				  uint32_t sar_width,
				  uint32_t sar_height);


H264_API
const char *h264_sei_type_str(enum h264_sei_type val);


#endif /* !_H264_TYPES_H_ */
