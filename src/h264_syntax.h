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

#ifndef _H264_SYNTAX_H_
#define _H264_SYNTAX_H_

#include "h264_syntax_ops.h"


/**
 * E.1.2 HRD parameters syntax
 */
static int H264_SYNTAX_FCT(hrd)(struct h264_bitstream *bs,
				H264_SYNTAX_CONST struct h264_hrd *hrd)
{
	H264_BITS_UE(hrd->cpb_cnt_minus1);
	H264_BITS(hrd->bit_rate_scale, 4);
	H264_BITS(hrd->cpb_size_scale, 4);

	ULOG_ERRNO_RETURN_ERR_IF(hrd->cpb_cnt_minus1 > ARRAY_SIZE(hrd->cpb),
				 EIO);

	H264_BEGIN_ARRAY(cpb);
	for (uint32_t i = 0; i <= hrd->cpb_cnt_minus1; i++) {
		H264_BEGIN_ARRAY_ITEM();
		H264_BITS_UE(hrd->cpb[i].bit_rate_value_minus1);
		H264_BITS_UE(hrd->cpb[i].cpb_size_value_minus1);
		H264_BITS(hrd->cpb[i].cbr_flag, 1);
		H264_END_ARRAY_ITEM();
	}
	H264_END_ARRAY(cpb);

	H264_BITS(hrd->initial_cpb_removal_delay_length_minus1, 5);
	H264_BITS(hrd->cpb_removal_delay_length_minus1, 5);
	H264_BITS(hrd->dpb_output_delay_length_minus1, 5);
	H264_BITS(hrd->time_offset_length, 5);

	return 0;
}


/**
 * E.1.1 VUI parameters syntax
 */
static int H264_SYNTAX_FCT(vui)(struct h264_bitstream *bs,
				H264_SYNTAX_CONST struct h264_vui *vui)
{
	int res = 0;

	H264_BITS(vui->aspect_ratio_info_present_flag, 1);
	if (vui->aspect_ratio_info_present_flag) {
		H264_BITS(vui->aspect_ratio_idc, 8);
		if (vui->aspect_ratio_idc == H264_ASPECT_RATIO_EXTENDED_SAR) {
			H264_BITS(vui->sar_width, 16);
			H264_BITS(vui->sar_height, 16);
		}
	}

	H264_BITS(vui->overscan_info_present_flag, 1);
	if (vui->overscan_info_present_flag)
		H264_BITS(vui->overscan_appropriate_flag, 1);

	H264_BITS(vui->video_signal_type_present_flag, 1);
	if (vui->video_signal_type_present_flag) {
		H264_BITS(vui->video_format, 3);
		H264_BITS(vui->video_full_range_flag, 1);
		H264_BITS(vui->colour_description_present_flag, 1);
		if (vui->colour_description_present_flag) {
			H264_BITS(vui->colour_primaries, 8);
			H264_BITS(vui->transfer_characteristics, 8);
			H264_BITS(vui->matrix_coefficients, 8);
		}
	}

	H264_BITS(vui->chroma_loc_info_present_flag, 1);
	if (vui->chroma_loc_info_present_flag) {
		H264_BITS_UE(vui->chroma_sample_loc_type_top_field);
		H264_BITS_UE(vui->chroma_sample_loc_type_bottom_field);
	}

	H264_BITS(vui->timing_info_present_flag, 1);
	if (vui->timing_info_present_flag) {
		H264_BITS(vui->num_units_in_tick, 32);
		H264_BITS(vui->time_scale, 32);
		H264_BITS(vui->fixed_frame_rate_flag, 1);
	}

	H264_BITS(vui->nal_hrd_parameters_present_flag, 1);
	if (vui->nal_hrd_parameters_present_flag) {
		H264_BEGIN_STRUCT(nal_hrd);
		res = H264_SYNTAX_FCT(hrd)(bs, &vui->nal_hrd);
		H264_END_STRUCT(nal_hrd);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	H264_BITS(vui->vcl_hrd_parameters_present_flag, 1);
	if (vui->vcl_hrd_parameters_present_flag) {
		H264_BEGIN_STRUCT(vcl_hrd);
		res = H264_SYNTAX_FCT(hrd)(bs, &vui->vcl_hrd);
		H264_END_STRUCT(vcl_hrd);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	if (vui->nal_hrd_parameters_present_flag ||
	    vui->vcl_hrd_parameters_present_flag) {
		H264_BITS(vui->low_delay_hrd_flag, 1);
	}

	H264_BITS(vui->pic_struct_present_flag, 1);
	H264_BITS(vui->bitstream_restriction_flag, 1);
	if (vui->bitstream_restriction_flag) {
		H264_BITS(vui->motion_vectors_over_pic_boundaries_flag, 1);
		H264_BITS_UE(vui->max_bytes_per_pic_denom);
		H264_BITS_UE(vui->max_bits_per_mb_denom);
		H264_BITS_UE(vui->log2_max_mv_length_horizontal);
		H264_BITS_UE(vui->log2_max_mv_length_vertical);
		H264_BITS_UE(vui->max_num_reorder_frames);
		H264_BITS_UE(vui->max_dec_frame_buffering);
	}

	return 0;
}


/**
 * 7.3.2.1.1.1 Scaling list syntax
 */
static int
	H264_SYNTAX_FCT(scaling_list)(struct h264_bitstream *bs,
				      H264_SYNTAX_CONST int32_t *scaling_list,
				      uint32_t size,
				      H264_SYNTAX_CONST int *use_default,
				      H264_SYNTAX_CONST int *optimized)
{
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	int32_t last = 8;
	int32_t next = 8;
	int32_t delta_scale = 0;

	for (uint32_t i = 0; i < size; i++) {
		if (next != 0) {
			H264_BITS_SE(delta_scale);
			next = (last + delta_scale + 256) % 256;
			*use_default = (i == 0 && next == 0);
			*optimized = (next == 0);
		}
		scaling_list[i] = (next == 0) ? last : next;
		last = scaling_list[i];
	}

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_WRITE
	int32_t last = 8;
	int32_t next = 8;
	int32_t delta_scale = 0;
	uint32_t n = 0;

	if (*optimized) {
		/* Count how many values are identical at the end of the array;
		 * if all values are identical, n will be size - 1 */
		for (uint32_t i = size - 1; i >= 1; i--) {
			if (scaling_list[i] == scaling_list[i - 1])
				n++;
			else
				break;
		}

		/* Special case: all values are 8 (the special initial value) */
		if (n == size - 1 && scaling_list[0] == last)
			n++;
	}

	for (uint32_t i = 0; i < size && next != 0; i++) {
		/* If following values are identical, encode a 0
		 * and stop the loop */
		next = i < size - n ? scaling_list[i] : 0;
		delta_scale = (int8_t)((next - last) % 256);
		H264_BITS_SE(delta_scale);
		last = scaling_list[i];
	}

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP

	for (uint32_t i = 0; i < size; i++)
		H264_BITS(scaling_list[i], 0);

#else
#	error "Unsupported H264_SYNTAX_OP_KIND"
#endif

	return 0;
}


/**
 * 7.3.2.1 Sequence parameter set RBSP syntax
 * 7.3.2.2 Picture parameter set RBSP syntax
 */
static int H264_SYNTAX_FCT(scaling_matrix)(
	struct h264_bitstream *bs,
	H264_SYNTAX_CONST struct h264_scaling_matrix *matrix,
	uint32_t size)
{
	int res = 0;

	for (uint32_t i = 0; i < size; i++) {
		H264_BEGIN_ARRAY_ITEM();
		H264_BITS(matrix->scaling_list_present_flag[i], 1);
		if (matrix->scaling_list_present_flag[i]) {
			if (i < 6) {
				H264_BEGIN_ARRAY(scaling_list_4x4);
				res = H264_SYNTAX_FCT(scaling_list)(
					bs,
					matrix->scaling_list_4x4[i],
					16,
					&matrix->use_default_4x4[i],
					&matrix->_optimized_4x4[i]);
				H264_END_ARRAY(scaling_list_4x4);
			} else {
				H264_BEGIN_ARRAY(scaling_list_8x8);
				res = H264_SYNTAX_FCT(scaling_list)(
					bs,
					matrix->scaling_list_8x8[i - 6],
					64,
					&matrix->use_default_8x8[i - 6],
					&matrix->_optimized_8x8[i - 6]);
				H264_END_ARRAY(scaling_list_8x8);
			}
		}
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		H264_END_ARRAY_ITEM();
	}

	return 0;
}


/**
 * 7.3.2.1 Sequence parameter set RBSP syntax
 */
static int H264_SYNTAX_FCT(sps)(struct h264_bitstream *bs,
				H264_SYNTAX_CONST struct h264_sps *sps)
{
	int res = 0;
	uint32_t i = 0, n = 0;

	H264_BITS(sps->profile_idc, 8);
	H264_BITS(sps->constraint_set0_flag, 1);
	H264_BITS(sps->constraint_set1_flag, 1);
	H264_BITS(sps->constraint_set2_flag, 1);
	H264_BITS(sps->constraint_set3_flag, 1);
	H264_BITS(sps->constraint_set4_flag, 1);
	H264_BITS(sps->constraint_set5_flag, 1);
	H264_BITS(sps->reserved_zero_2bits, 2);
	H264_BITS(sps->level_idc, 8);
	H264_BITS_UE(sps->seq_parameter_set_id);

	if (sps->profile_idc == 100 || sps->profile_idc == 110 ||
	    sps->profile_idc == 122 || sps->profile_idc == 244 ||
	    sps->profile_idc == 44 || sps->profile_idc == 83 ||
	    sps->profile_idc == 86 || sps->profile_idc == 118 ||
	    sps->profile_idc == 128 || sps->profile_idc == 138 ||
	    sps->profile_idc == 139 || sps->profile_idc == 134 ||
	    sps->profile_idc == 135) {
		H264_BITS_UE(sps->chroma_format_idc);
		if (sps->chroma_format_idc == 3)
			H264_BITS(sps->separate_colour_plane_flag, 1);

		H264_BITS_UE(sps->bit_depth_luma_minus8);
		ULOG_ERRNO_RETURN_ERR_IF(sps->bit_depth_luma_minus8 > 6, EIO);
		H264_BITS_UE(sps->bit_depth_chroma_minus8);
		ULOG_ERRNO_RETURN_ERR_IF(sps->bit_depth_chroma_minus8 > 6, EIO);

		H264_BITS(sps->qpprime_y_zero_transform_bypass_flag, 1);
		H264_BITS(sps->seq_scaling_matrix_present_flag, 1);
		if (sps->seq_scaling_matrix_present_flag) {
			n = (sps->chroma_format_idc != 3) ? 8 : 12;
			H264_BEGIN_ARRAY(seq_scaling_matrix);
			res = H264_SYNTAX_FCT(scaling_matrix)(
				bs, &sps->seq_scaling_matrix, n);
			H264_END_ARRAY(seq_scaling_matrix);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}

	H264_BITS_UE(sps->log2_max_frame_num_minus4);

	H264_BITS_UE(sps->pic_order_cnt_type);
	if (sps->pic_order_cnt_type == 0) {
		H264_BITS_UE(sps->log2_max_pic_order_cnt_lsb_minus4);
	} else if (sps->pic_order_cnt_type == 1) {
		H264_BITS(sps->delta_pic_order_always_zero_flag, 1);
		H264_BITS_SE(sps->offset_for_non_ref_pic);
		H264_BITS_SE(sps->offset_for_top_to_bottom_field);
		H264_BITS_UE(sps->num_ref_frames_in_pic_order_cnt_cycle);
		ULOG_ERRNO_RETURN_ERR_IF(
			sps->num_ref_frames_in_pic_order_cnt_cycle >=
				ARRAY_SIZE(sps->offset_for_ref_frame),
			EIO);
		H264_BEGIN_ARRAY(offset_for_ref_frame);
		for (i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
			H264_BITS_SE(sps->offset_for_ref_frame[i]);
		H264_END_ARRAY(offset_for_ref_frame);
	}

	H264_BITS_UE(sps->max_num_ref_frames);
	H264_BITS(sps->gaps_in_frame_num_value_allowed_flag, 1);
	H264_BITS_UE(sps->pic_width_in_mbs_minus1);
	H264_BITS_UE(sps->pic_height_in_map_units_minus1);

	H264_BITS(sps->frame_mbs_only_flag, 1);
	if (!sps->frame_mbs_only_flag)
		H264_BITS(sps->mb_adaptive_frame_field_flag, 1);

	H264_BITS(sps->direct_8x8_inference_flag, 1);

	H264_BITS(sps->frame_cropping_flag, 1);
	if (sps->frame_cropping_flag) {
		H264_BITS_UE(sps->frame_crop_left_offset);
		H264_BITS_UE(sps->frame_crop_right_offset);
		H264_BITS_UE(sps->frame_crop_top_offset);
		H264_BITS_UE(sps->frame_crop_bottom_offset);
	}

	H264_BITS(sps->vui_parameters_present_flag, 1);
	if (sps->vui_parameters_present_flag) {
		H264_BEGIN_STRUCT(vui);
		res = H264_SYNTAX_FCT(vui)(bs, &sps->vui);
		H264_END_STRUCT(vui);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	H264_BITS_RBSP_TRAILING();

	return 0;
}


/**
 * 7.3.2.2 Picture parameter set RBSP syntax
 */
static int H264_SYNTAX_FCT(pps_internal)(struct h264_bitstream *bs,
					 const struct h264_sps *sps,
					 H264_SYNTAX_CONST struct h264_pps *pps)
{
	int res = 0;
	uint32_t i = 0, n = 0;
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
	uint32_t len = 0;
#endif

	H264_BITS(pps->entropy_coding_mode_flag, 1);
	H264_BITS(pps->bottom_field_pic_order_in_frame_present_flag, 1);
	H264_BITS_UE(pps->num_slice_groups_minus1);
	if (pps->num_slice_groups_minus1 > 0) {
		H264_BITS_UE(pps->slice_group_map_type);
		switch (pps->slice_group_map_type) {
		case 0:
			ULOG_ERRNO_RETURN_ERR_IF(
				pps->num_slice_groups_minus1 >
					ARRAY_SIZE(pps->run_length_minus1),
				EIO);
			H264_BEGIN_ARRAY(run_length_minus1);
			for (i = 0; i <= pps->num_slice_groups_minus1; i++)
				H264_BITS_UE(pps->run_length_minus1[i]);
			H264_END_ARRAY(run_length_minus1);
			break;

		case 1:
			/* Nothing to do */
			break;

		case 2:
			ULOG_ERRNO_RETURN_ERR_IF(
				pps->num_slice_groups_minus1 >=
					ARRAY_SIZE(pps->top_left),
				EIO);
			ULOG_ERRNO_RETURN_ERR_IF(
				pps->num_slice_groups_minus1 >=
					ARRAY_SIZE(pps->bottom_right),
				EIO);
			H264_BEGIN_ARRAY(pos);
			for (i = 0; i < pps->num_slice_groups_minus1; i++) {
				H264_BEGIN_ARRAY_ITEM();
				H264_BITS_UE(pps->top_left[i]);
				H264_BITS_UE(pps->bottom_right[i]);
				H264_END_ARRAY_ITEM();
			}
			H264_END_ARRAY(pos);
			break;

		case 3:
		case 4:
		case 5:
			H264_BITS(pps->slice_group_change_direction_flag, 1);
			H264_BITS_UE(pps->slice_group_change_rate_minus1);
			break;

		case 6:
			H264_BITS_UE(pps->pic_size_in_map_units_minus1);
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
			len = h264_intlog2(pps->num_slice_groups_minus1 + 1);
#endif
			ULOG_ERRNO_RETURN_ERR_IF(
				pps->pic_size_in_map_units_minus1 >
					ARRAY_SIZE(pps->slice_group_id),
				EIO);
			H264_BEGIN_ARRAY(slice_group_id);
			for (i = 0; i <= pps->pic_size_in_map_units_minus1; i++)
				H264_BITS(pps->slice_group_id[i], len);
			H264_END_ARRAY(slice_group_id);
			break;

		default:
			return -EIO;
			break;
		}
	}

	H264_BITS_UE(pps->num_ref_idx_l0_default_active_minus1);
	H264_BITS_UE(pps->num_ref_idx_l1_default_active_minus1);
	H264_BITS(pps->weighted_pred_flag, 1);
	H264_BITS(pps->weighted_bipred_idc, 2);
	H264_BITS_SE(pps->pic_init_qp_minus26);
	H264_BITS_SE(pps->pic_init_qs_minus26);
	H264_BITS_SE(pps->chroma_qp_index_offset);
	H264_BITS(pps->deblocking_filter_control_present_flag, 1);
	H264_BITS(pps->constrained_intra_pred_flag, 1);
	H264_BITS(pps->redundant_pic_cnt_present_flag, 1);

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	if (h264_bs_more_rbsp_data(bs))
		pps->_more_rbsp_data_present = 1;
#endif
	if (pps->_more_rbsp_data_present) {
		H264_BITS(pps->transform_8x8_mode_flag, 1);
		H264_BITS(pps->pic_scaling_matrix_present_flag, 1);
		if (pps->pic_scaling_matrix_present_flag) {
			n = 6;
			if (pps->transform_8x8_mode_flag)
				n += (sps->chroma_format_idc != 3) ? 2 : 6;
			H264_BEGIN_ARRAY(pic_scaling_matrix);
			res = H264_SYNTAX_FCT(scaling_matrix)(
				bs, &pps->pic_scaling_matrix, n);
			H264_END_ARRAY(pic_scaling_matrix);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}

		H264_BITS_SE(pps->second_chroma_qp_index_offset);
	}

	H264_BITS_RBSP_TRAILING();

	return 0;
}


static int H264_SYNTAX_FCT(pps_with_ctx)(struct h264_bitstream *bs,
					 struct h264_ctx *ctx,
					 H264_SYNTAX_CONST struct h264_pps *pps)
{
	int res = 0;
	struct h264_sps *sps = NULL;

	/* Read PPS/SPS id, load SPS in context */
	H264_BITS_UE(pps->pic_parameter_set_id);
	H264_BITS_UE(pps->seq_parameter_set_id);
	res = h264_ctx_set_active_sps(ctx, pps->seq_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	sps = ctx->sps;

	/* Continue reading PPS */
	res = H264_SYNTAX_FCT(pps_internal)(bs, sps, pps);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


static int H264_SYNTAX_FCT(pps_with_sps)(struct h264_bitstream *bs,
					 const struct h264_sps *sps,
					 H264_SYNTAX_CONST struct h264_pps *pps)
{
	int res = 0;

	/* Read PPS/SPS id, make sure SPS id matches input */
	H264_BITS_UE(pps->pic_parameter_set_id);
	H264_BITS_UE(pps->seq_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(
		sps->seq_parameter_set_id != pps->seq_parameter_set_id, EIO);

	/* Continue reading PPS */
	res = H264_SYNTAX_FCT(pps_internal)(bs, sps, pps);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


/**
 * D.1.1 Buffering period SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_buffering_period)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_buffering_period *sei)
{
	int res = 0;
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
	uint32_t n = 0;
#endif
	const struct h264_hrd *hrd = NULL;

	H264_BITS_UE(sei->seq_parameter_set_id);
	res = h264_ctx_set_active_sps(ctx, sei->seq_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (ctx->sps->vui.nal_hrd_parameters_present_flag) {
		hrd = &ctx->sps->vui.nal_hrd;
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
		n = hrd->initial_cpb_removal_delay_length_minus1 + 1;
#endif
		ULOG_ERRNO_RETURN_ERR_IF(hrd->cpb_cnt_minus1 >
						 ARRAY_SIZE(sei->nal_hrd_cpb),
					 EIO);
		H264_BEGIN_ARRAY(nal_hrd_cpb);
		for (uint32_t i = 0; i <= hrd->cpb_cnt_minus1; i++) {
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS(sei->nal_hrd_cpb[i].initial_cpb_removal_delay,
				  n);
			H264_BITS(sei->nal_hrd_cpb[i]
					  .initial_cpb_removal_delay_offset,
				  n);
			H264_END_ARRAY_ITEM();
		}
		H264_END_ARRAY(nal_hrd_cpb);
	}

	if (ctx->sps->vui.vcl_hrd_parameters_present_flag) {
		hrd = &ctx->sps->vui.vcl_hrd;
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
		n = hrd->initial_cpb_removal_delay_length_minus1 + 1;
#endif
		ULOG_ERRNO_RETURN_ERR_IF(hrd->cpb_cnt_minus1 >
						 ARRAY_SIZE(sei->vcl_hrd_cpb),
					 EIO);
		H264_BEGIN_ARRAY(vcl_hrd_cpb);
		for (uint32_t i = 0; i <= hrd->cpb_cnt_minus1; i++) {
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS(sei->vcl_hrd_cpb[i].initial_cpb_removal_delay,
				  n);
			H264_BITS(sei->vcl_hrd_cpb[i]
					  .initial_cpb_removal_delay_offset,
				  n);
			H264_END_ARRAY_ITEM();
		}
		H264_END_ARRAY(vcl_hrd_cpb);
	}

	return 0;
}


/**
 * D.1.2 Picture timing SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_pic_timing)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_pic_timing *sei)
{
	static const uint32_t num_clock_ts[16] = {
		1, 1, 1, 2, 2, 3, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0};

	uint32_t n = 0;
	const struct h264_sps *sps = ctx->sps;
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EIO);

	if (sps->vui.nal_hrd_parameters_present_flag ||
	    sps->vui.vcl_hrd_parameters_present_flag) {
		n = sps->vui.nal_hrd_parameters_present_flag
			    ? sps->vui.nal_hrd.cpb_removal_delay_length_minus1 +
				      1
			    : sps->vui.vcl_hrd.cpb_removal_delay_length_minus1 +
				      1;
		H264_BITS(sei->cpb_removal_delay, n);

		n = sps->vui.nal_hrd_parameters_present_flag
			    ? sps->vui.nal_hrd.dpb_output_delay_length_minus1 +
				      1
			    : sps->vui.vcl_hrd.dpb_output_delay_length_minus1 +
				      1;
		H264_BITS(sei->dpb_output_delay, n);
	}

	if (sps->vui.pic_struct_present_flag) {
		H264_BITS(sei->pic_struct, 4);
		H264_BEGIN_ARRAY(clk_ts);
		for (uint32_t i = 0; i < num_clock_ts[sei->pic_struct]; i++) {
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS(sei->clk_ts[i].clock_timestamp_flag, 1);
			if (!sei->clk_ts[i].clock_timestamp_flag)
				goto next_ts;

			H264_BITS(sei->clk_ts[i].ct_type, 2);
			H264_BITS(sei->clk_ts[i].nuit_field_based_flag, 1);
			H264_BITS(sei->clk_ts[i].counting_type, 5);
			H264_BITS(sei->clk_ts[i].full_timestamp_flag, 1);
			H264_BITS(sei->clk_ts[i].discontinuity_flag, 1);
			H264_BITS(sei->clk_ts[i].cnt_dropped_flag, 1);
			H264_BITS(sei->clk_ts[i].n_frames, 8);

			if (sei->clk_ts[i].full_timestamp_flag) {
				H264_BITS(sei->clk_ts[i].seconds_value, 6);
				H264_BITS(sei->clk_ts[i].minutes_value, 6);
				H264_BITS(sei->clk_ts[i].hours_value, 5);
			} else {
				H264_BITS(sei->clk_ts[i].seconds_flag, 1);
				if (!sei->clk_ts[i].seconds_flag)
					goto time_offset;
				H264_BITS(sei->clk_ts[i].seconds_value, 6);
				H264_BITS(sei->clk_ts[i].minutes_flag, 1);
				if (!sei->clk_ts[i].minutes_flag)
					goto time_offset;
				H264_BITS(sei->clk_ts[i].minutes_value, 6);
				H264_BITS(sei->clk_ts[i].hours_flag, 1);
				if (!sei->clk_ts[i].hours_flag)
					goto time_offset;
				H264_BITS(sei->clk_ts[i].hours_value, 5);
			}

			/* clang-format off */
time_offset:
			/* clang-format on */
			n = sps->vui.nal_hrd_parameters_present_flag
				    ? sps->vui.nal_hrd.time_offset_length
				    : sps->vui.vcl_hrd_parameters_present_flag
					      ? sps->vui.vcl_hrd
							.time_offset_length
					      : 24;
			if (n > 0)
				H264_BITS_I(sei->clk_ts[i].time_offset, n);
			/* clang-format off */
next_ts:
			/* clang-format on */
			H264_END_ARRAY_ITEM();
		}
		H264_END_ARRAY(clk_ts);
	}

	return 0;
}


/**
 * D.1.3 Pan-scan rectangle SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_pan_scan_rect)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_pan_scan_rect *sei)
{
	H264_BITS_UE(sei->pan_scan_rect_id);
	H264_BITS(sei->pan_scan_rect_cancel_flag, 1);
	if (!sei->pan_scan_rect_cancel_flag) {
		H264_BITS_UE(sei->pan_scan_cnt_minus1);
		ULOG_ERRNO_RETURN_ERR_IF(sei->pan_scan_cnt_minus1 >
						 ARRAY_SIZE(sei->pan_scan_rect),
					 EIO);
		H264_BEGIN_ARRAY(pan_scan_rect);
		for (uint32_t i = 0; i <= sei->pan_scan_cnt_minus1; i++) {
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS_SE(sei->pan_scan_rect[i].left_offset);
			H264_BITS_SE(sei->pan_scan_rect[i].right_offset);
			H264_BITS_SE(sei->pan_scan_rect[i].top_offset);
			H264_BITS_SE(sei->pan_scan_rect[i].bottom_offset);
			H264_END_ARRAY_ITEM();
		}
		H264_END_ARRAY(pan_scan_rect);
		H264_BITS_UE(sei->pan_scan_rect_repetition_period);
	}
	return 0;
}


static int H264_SYNTAX_FCT(sei_data)(struct h264_bitstream *bs,
				     /* clang-format off */
				     const uint8_t *H264_SYNTAX_CONST*buf,
				     /* clang-format on */
				     H264_SYNTAX_CONST size_t *len)
{
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	ULOG_ERRNO_RETURN_ERR_IF(!h264_bs_byte_aligned(bs), EIO);
	*buf = bs->cdata + bs->off;
	*len = bs->len - bs->off;
#else
	ULOG_ERRNO_RETURN_ERR_IF(*len != 0 && *buf == NULL, EIO);
	H264_BEGIN_ARRAY(data);
	for (uint32_t i = 0; i < *len; i++)
		H264_BITS((*buf)[i], 8);
	H264_END_ARRAY(data);
#endif

	return 0;
}


/**
 * D.1.4 Filler payload SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_filler_payload)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_filler_payload *sei)
{
	int res = 0;

	res = H264_SYNTAX_FCT(sei_data)(bs, &sei->buf, &sei->len);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


/**
 * D.1.5 User data registered by Rec. ITU-T T.35 SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_user_data_registered)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_user_data_registered *sei)
{
	int res = 0;

	H264_BITS(sei->country_code, 8);
	if (sei->country_code == 0xff)
		H264_BITS(sei->country_code_extension_byte, 8);

	res = H264_SYNTAX_FCT(sei_data)(bs, &sei->buf, &sei->len);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


/**
 * D.1.6 User data unregistered SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_user_data_unregistered)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_user_data_unregistered *sei)
{
	int res = 0;

	H264_BEGIN_ARRAY(uuid);
	for (uint32_t i = 0; i < 16; i++)
		H264_BITS(sei->uuid[i], 8);
	H264_END_ARRAY(uuid);

	res = H264_SYNTAX_FCT(sei_data)(bs, &sei->buf, &sei->len);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	return 0;
}


/**
 * D.1.7 Recovery point SEI message syntax
 */
static int H264_SYNTAX_FCT(sei_recovery_point)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_sei_recovery_point *sei)
{
	H264_BITS_UE(sei->recovery_frame_cnt);
	H264_BITS(sei->exact_match_flag, 1);
	H264_BITS(sei->broken_link_flag, 1);
	H264_BITS(sei->changing_slice_group_idc, 2);
	return 0;
}


static int H264_SYNTAX_FCT(one_sei)(struct h264_bitstream *bs,
				    struct h264_ctx *ctx,
				    const struct h264_ctx_cbs *cbs,
				    void *userdata,
				    H264_SYNTAX_CONST struct h264_sei *sei)
{
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	int bit = 0;
#endif

	switch (sei->type) {
	case H264_SEI_TYPE_BUFFERING_PERIOD:
		H264_SEI(buffering_period);
		break;

	case H264_SEI_TYPE_PIC_TIMING:
		H264_SEI(pic_timing);
		break;

	case H264_SEI_TYPE_PAN_SCAN_RECT:
		H264_SEI(pan_scan_rect);
		break;

	case H264_SEI_TYPE_FILLER_PAYLOAD:
		H264_SEI(filler_payload);
		break;

	case H264_SEI_TYPE_USER_DATA_REGISTERED:
		H264_SEI(user_data_registered);
		break;

	case H264_SEI_TYPE_USER_DATA_UNREGISTERED:
		H264_SEI(user_data_unregistered);
		break;

	case H264_SEI_TYPE_RECOVERY_POINT:
		H264_SEI(recovery_point);
		break;

	default:
		/* TODO */
		return 0;
	}

	/* Align bitstream if needed */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	/* Should be 1 followed by 0, but ignore erroneous bit streams */
	while (!h264_bs_byte_aligned(bs))
		H264_BITS(bit, 1);
#else
	if (!h264_bs_byte_aligned(bs))
		H264_BITS_RBSP_TRAILING();
#endif

	return 0;
}


/**
 * 7.3.2.3 Supplemental enhancement information RBSP syntax
 */
static int H264_SYNTAX_FCT(sei)(struct h264_bitstream *bs,
				struct h264_ctx *ctx,
				const struct h264_ctx_cbs *cbs,
				void *userdata)
{
	int res = 0;
	uint32_t payload_type = 0;
	uint32_t payload_size = 0;
	struct h264_sei *sei = NULL;

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ

	struct h264_bitstream bs2;
	do {
		H264_BEGIN_ARRAY_ITEM();

		res = h264_bs_read_bits_ff_coded(bs, &payload_type);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		res = h264_bs_read_bits_ff_coded(bs, &payload_size);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		/* Allocate new SEI in internal table */
		res = h264_ctx_add_sei_internal(ctx, &sei);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		sei->type = payload_type;

		/* Setup raw buffer */
		sei->raw.buf = malloc(payload_size);
		ULOG_ERRNO_RETURN_ERR_IF(sei->raw.buf == NULL, ENOMEM);
		sei->raw.len = payload_size;

		for (uint32_t i = 0; i < sei->raw.len; i++)
			H264_BITS(sei->raw.buf[i], 8);

		/* Notify callback */
		H264_CB(ctx,
			cbs,
			userdata,
			sei,
			sei->type,
			sei->raw.buf,
			sei->raw.len);

		/* Construct a bitstream to parse SEI
		 * (without emulation prevention) */
		h264_bs_cinit(&bs2, sei->raw.buf, sei->raw.len, 0);
		res = H264_SYNTAX_FCT(one_sei)(&bs2, ctx, cbs, userdata, sei);
		h264_bs_clear(&bs2);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		H264_END_ARRAY_ITEM();
	} while (h264_bs_more_rbsp_data(bs));

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_WRITE

	ULOG_ERRNO_RETURN_ERR_IF(ctx->sei_count == 0, EIO);
	for (uint32_t i = 0; i < ctx->sei_count; i++) {
		H264_BEGIN_ARRAY_ITEM();
		sei = &ctx->sei_table[i];

		ULOG_ERRNO_RETURN_ERR_IF(sei->raw.buf == NULL, EIO);
		ULOG_ERRNO_RETURN_ERR_IF(sei->raw.len == 0, EIO);

		payload_type = sei->type;
		res = h264_bs_write_bits_ff_coded(bs, payload_type);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		payload_size = sei->raw.len;
		res = h264_bs_write_bits_ff_coded(bs, payload_size);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		/* Directly write raw buffer, SEI should already be encoded */
		for (uint32_t i = 0; i < sei->raw.len; i++)
			H264_BITS(sei->raw.buf[i], 8);

		H264_END_ARRAY_ITEM();
	}

#elif H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP

	ULOG_ERRNO_RETURN_ERR_IF(ctx->sei_count == 0, EIO);
	for (uint32_t i = 0; i < ctx->sei_count; i++) {
		H264_BEGIN_ARRAY_ITEM();
		sei = &ctx->sei_table[i];

		/* Use fake number of bits for dump (no actual bitstream) */
		payload_type = sei->type;
		H264_BITS(payload_type, 0);
		payload_size = sei->raw.len;
		H264_BITS(payload_size, 0);

		res = H264_SYNTAX_FCT(one_sei)(bs, ctx, cbs, userdata, sei);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

		H264_END_ARRAY_ITEM();
	}

#else
#	error "Unsupported H264_SYNTAX_OP_KIND"
#endif

	H264_BITS_RBSP_TRAILING();

	return 0;
}


/**
 * 7.3.2.4 Access unit delimiter RBSP syntax
 */
static int H264_SYNTAX_FCT(aud)(struct h264_bitstream *bs,
				H264_SYNTAX_CONST struct h264_aud *aud)
{
	H264_BITS(aud->primary_pic_type, 3);
	H264_BITS_RBSP_TRAILING();
	return 0;
}


/**
 * 7.3.3.1 Reference picture list modification syntax
 * H.7.3.3.1.1 Reference picture list MVC modification syntax
 */
static int H264_SYNTAX_FCT(rplm_items)(
	struct h264_bitstream *bs,
	H264_SYNTAX_CONST struct h264_rplm_item *items,
	uint32_t maxcount)
{
	uint32_t i = 0, val = 0;

	do {
		ULOG_ERRNO_RETURN_ERR_IF(i >= maxcount, EIO);
		H264_BEGIN_ARRAY_ITEM();

		H264_BITS_UE(items[i].modification_of_pic_nums_idc);
		val = items[i].modification_of_pic_nums_idc;

		if (val == 0 || val == 1)
			H264_BITS_UE(items[i].abs_diff_pic_num_minus1);
		else if (val == 2)
			H264_BITS_UE(items[i].long_term_pic_num);
		else if (val == 4 || val == 5)
			H264_BITS_UE(items[i].abs_diff_view_idx_minus1);

		H264_END_ARRAY_ITEM();
		i++;
	} while (val != 3);

	return 0;
}


/**
 * 7.3.3.1 Reference picture list modification syntax
 * H.7.3.3.1.1 Reference picture list MVC modification syntax
 */
static int H264_SYNTAX_FCT(ref_pic_list_modification)(
	struct h264_bitstream *bs,
	H264_SYNTAX_CONST struct h264_slice_header *sh)
{
	int res = 0;
	enum h264_slice_type type = H264_SLICE_TYPE(sh->slice_type);
	H264_SYNTAX_CONST struct h264_rplm *rplm = &sh->rplm;

	if (type != H264_SLICE_TYPE_I && type != H264_SLICE_TYPE_SI) {
		H264_BITS(rplm->ref_pic_list_modification_flag_l0, 1);
		if (rplm->ref_pic_list_modification_flag_l0) {
			H264_BEGIN_ARRAY(pic_num_l0);
			res = H264_SYNTAX_FCT(rplm_items)(
				bs,
				rplm->pic_num_l0,
				ARRAY_SIZE(rplm->pic_num_l0));
			H264_END_ARRAY(pic_num_l0);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}

	if (type == H264_SLICE_TYPE_B) {
		H264_BITS(rplm->ref_pic_list_modification_flag_l1, 1);
		if (rplm->ref_pic_list_modification_flag_l1) {
			H264_BEGIN_ARRAY(pic_num_l1);
			res = H264_SYNTAX_FCT(rplm_items)(
				bs,
				rplm->pic_num_l1,
				ARRAY_SIZE(rplm->pic_num_l1));
			H264_END_ARRAY(pic_num_l1);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}

	return 0;
}


/**
 * 7.3.3.2 Prediction weight table syntax
 */
static int
	H264_SYNTAX_FCT(pwt_item)(struct h264_bitstream *bs,
				  H264_SYNTAX_CONST struct h264_pwt_item *item,
				  uint32_t chroma_array_type)
{
	H264_BITS(item->luma_weight_flag, 1);
	if (item->luma_weight_flag) {
		H264_BITS_SE(item->luma_weight);
		H264_BITS_SE(item->luma_offset);
	}

	if (chroma_array_type != 0) {
		H264_BITS(item->chroma_weight_flag, 1);
		if (item->chroma_weight_flag) {
			H264_BEGIN_ARRAY(chroma);
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS_SE(item->chroma_weight[0]);
			H264_BITS_SE(item->chroma_offset[0]);
			H264_END_ARRAY_ITEM();
			H264_BEGIN_ARRAY_ITEM();
			H264_BITS_SE(item->chroma_weight[1]);
			H264_BITS_SE(item->chroma_offset[1]);
			H264_END_ARRAY_ITEM();
			H264_END_ARRAY(chroma);
		}
	}

	return 0;
}


/**
 * 7.3.3.2 Prediction weight table syntax
 */
static int H264_SYNTAX_FCT(pred_weight_table)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_slice_header *sh)
{
	int res = 0;
	uint32_t i = 0;
	uint32_t chroma_array_type = ctx->sps->separate_colour_plane_flag
					     ? 0
					     : ctx->sps->chroma_format_idc;
	enum h264_slice_type type = H264_SLICE_TYPE(sh->slice_type);
	H264_SYNTAX_CONST struct h264_pwt *pwt = &sh->pwt;

	H264_BITS_UE(pwt->luma_log2_weight_denom);

	if (chroma_array_type != 0)
		H264_BITS_UE(pwt->chroma_log2_weight_denom);

	ULOG_ERRNO_RETURN_ERR_IF(
		sh->num_ref_idx_l0_active_minus1 > ARRAY_SIZE(pwt->l0), EIO);
	H264_BEGIN_ARRAY(l0);
	for (i = 0; i <= sh->num_ref_idx_l0_active_minus1; i++) {
		H264_BEGIN_ARRAY_ITEM();
		res = H264_SYNTAX_FCT(pwt_item)(
			bs, &pwt->l0[i], chroma_array_type);
		H264_END_ARRAY_ITEM();
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}
	H264_END_ARRAY(l0);

	if (type != H264_SLICE_TYPE_B)
		goto out;

	ULOG_ERRNO_RETURN_ERR_IF(
		sh->num_ref_idx_l1_active_minus1 > ARRAY_SIZE(pwt->l1), EIO);
	H264_BEGIN_ARRAY(l1);
	for (i = 0; i <= sh->num_ref_idx_l1_active_minus1; i++) {
		H264_BEGIN_ARRAY_ITEM();
		res = H264_SYNTAX_FCT(pwt_item)(
			bs, &pwt->l1[i], chroma_array_type);
		H264_END_ARRAY_ITEM();
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}
	H264_END_ARRAY(l1);

out:
	return 0;
}


/**
 * 7.3.3.3 Decoded reference picture marking syntax
 */
static int H264_SYNTAX_FCT(drpm_items)(
	struct h264_bitstream *bs,
	H264_SYNTAX_CONST struct h264_drpm_item *items,
	uint32_t maxcount)
{
	uint32_t i = 0, val = 0;

	do {
		ULOG_ERRNO_RETURN_ERR_IF(i >= maxcount, EIO);
		H264_BEGIN_ARRAY_ITEM();

		H264_BITS_UE(items[i].memory_management_control_operation);
		val = items[i].memory_management_control_operation;

		if (val == 1 || val == 3)
			H264_BITS_UE(items[i].difference_of_pic_nums_minus1);
		if (val == 2)
			H264_BITS_UE(items[i].long_term_pic_num);
		if (val == 3 || val == 6)
			H264_BITS_UE(items[i].long_term_frame_idx);
		if (val == 4)
			H264_BITS_UE(items[i].max_long_term_frame_idx_plus1);

		H264_END_ARRAY_ITEM();
		i++;
	} while (val != 0);

	return 0;
}


/**
 * 7.3.3.3 Decoded reference picture marking syntax
 */
static int H264_SYNTAX_FCT(dec_ref_pic_marking)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_slice_header *sh)
{
	int res = 0;
	int idr_pic_flag = (ctx->nalu.type == H264_NALU_TYPE_SLICE_IDR);
	H264_SYNTAX_CONST struct h264_drpm *drpm = &sh->drpm;

	if (idr_pic_flag) {
		H264_BITS(drpm->no_output_of_prior_pics_flag, 1);
		H264_BITS(drpm->long_term_reference_flag, 1);
	} else {
		H264_BITS(drpm->adaptive_ref_pic_marking_mode_flag, 1);
		if (drpm->adaptive_ref_pic_marking_mode_flag) {
			H264_BEGIN_ARRAY(mm);
			res = H264_SYNTAX_FCT(drpm_items)(
				bs, drpm->mm, ARRAY_SIZE(drpm->mm));
			H264_END_ARRAY(mm);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}

	return 0;
}


/**
 * 7.3.3 Slice header syntax
 */
static int H264_SYNTAX_FCT(slice_header)(
	struct h264_bitstream *bs,
	struct h264_ctx *ctx,
	H264_SYNTAX_CONST struct h264_slice_header *sh)
{
	int res = 0;
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
	uint32_t n = 0;
#endif
	int idr_pic_flag = (ctx->nalu.type == H264_NALU_TYPE_SLICE_IDR);
	enum h264_slice_type type;
	ctx->slice.hdr_len = 0;

	H264_BITS_UE(sh->first_mb_in_slice);
	H264_BITS_UE(sh->slice_type);
	type = H264_SLICE_TYPE(sh->slice_type);

	H264_BITS_UE(sh->pic_parameter_set_id);
	res = h264_ctx_set_active_pps(ctx, sh->pic_parameter_set_id);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	sh->num_ref_idx_l0_active_minus1 =
		ctx->pps->num_ref_idx_l0_default_active_minus1;
	sh->num_ref_idx_l1_active_minus1 =
		ctx->pps->num_ref_idx_l1_default_active_minus1;
#endif

	if (ctx->sps->separate_colour_plane_flag)
		H264_BITS(sh->colour_plane_id, 2);

#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
	n = ctx->sps->log2_max_frame_num_minus4 + 4;
#endif
	H264_BITS(sh->frame_num, n);

	if (!ctx->sps->frame_mbs_only_flag) {
		H264_BITS(sh->field_pic_flag, 1);
		if (sh->field_pic_flag)
			H264_BITS(sh->bottom_field_flag, 1);
	}

	if (idr_pic_flag)
		H264_BITS_UE(sh->idr_pic_id);

	if (ctx->sps->pic_order_cnt_type == 0) {
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
		n = ctx->sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
#endif
		H264_BITS(sh->pic_order_cnt_lsb, n);
		if (ctx->pps->bottom_field_pic_order_in_frame_present_flag &&
		    !sh->field_pic_flag) {
			H264_BITS_SE(sh->delta_pic_order_cnt_bottom);
		}
	}

	if (ctx->sps->pic_order_cnt_type == 1 &&
	    !ctx->sps->delta_pic_order_always_zero_flag) {
		H264_BEGIN_ARRAY(delta_pic_order_cnt);
		H264_BITS_SE(sh->delta_pic_order_cnt[0]);
		if (ctx->pps->bottom_field_pic_order_in_frame_present_flag &&
		    !sh->field_pic_flag) {
			H264_BITS_SE(sh->delta_pic_order_cnt[1]);
		}
		H264_END_ARRAY(delta_pic_order_cnt);
	}

	if (ctx->pps->redundant_pic_cnt_present_flag)
		H264_BITS_UE(sh->redundant_pic_cnt);


	if (type == H264_SLICE_TYPE_B)
		H264_BITS(sh->direct_spatial_mv_pred_flag, 1);

	if (type == H264_SLICE_TYPE_P || type == H264_SLICE_TYPE_SP ||
	    type == H264_SLICE_TYPE_B) {
		H264_BITS(sh->num_ref_idx_active_override_flag, 1);
		if (sh->num_ref_idx_active_override_flag) {
			H264_BITS_UE(sh->num_ref_idx_l0_active_minus1);
			if (type == H264_SLICE_TYPE_B)
				H264_BITS_UE(sh->num_ref_idx_l1_active_minus1);
		}
	}

	H264_BEGIN_STRUCT(rplm);
	res = H264_SYNTAX_FCT(ref_pic_list_modification)(bs, sh);
	H264_END_STRUCT(rplm);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if ((ctx->pps->weighted_pred_flag &&
	     (type == H264_SLICE_TYPE_P || type == H264_SLICE_TYPE_SP)) ||
	    (ctx->pps->weighted_bipred_idc == 1 && type == H264_SLICE_TYPE_B)) {
		H264_BEGIN_STRUCT(pwt);
		res = H264_SYNTAX_FCT(pred_weight_table)(bs, ctx, sh);
		H264_END_STRUCT(pwt);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	if (ctx->nalu.hdr.nal_ref_idc != 0) {
		H264_BEGIN_STRUCT(drpm);
		res = H264_SYNTAX_FCT(dec_ref_pic_marking)(bs, ctx, sh);
		H264_END_STRUCT(drpm);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}

	if (ctx->pps->entropy_coding_mode_flag && type != H264_SLICE_TYPE_I &&
	    type != H264_SLICE_TYPE_SI) {
		H264_BITS_UE(sh->cabac_init_idc);
	}

	H264_BITS_SE(sh->slice_qp_delta);

	if (type == H264_SLICE_TYPE_SP || type == H264_SLICE_TYPE_SI) {
		if (type == H264_SLICE_TYPE_SP)
			H264_BITS(sh->sp_for_switch_flag, 1);
		H264_BITS_SE(sh->slice_qs_delta);
	}

	if (ctx->pps->deblocking_filter_control_present_flag) {
		H264_BITS_UE(sh->disable_deblocking_filter_idc);
		if (sh->disable_deblocking_filter_idc != 1) {
			H264_BITS_SE(sh->slice_alpha_c0_offset_div2);
			H264_BITS_SE(sh->slice_beta_offset_div2);
		}
	}

	if (ctx->pps->num_slice_groups_minus1 > 0 &&
	    ctx->pps->slice_group_map_type >= 3 &&
	    ctx->pps->slice_group_map_type <= 5) {
#if H264_SYNTAX_OP_KIND != H264_SYNTAX_OP_KIND_DUMP
		uint32_t pic_size_in_map_units =
			(ctx->sps->pic_width_in_mbs_minus1 + 1) *
			(ctx->sps->pic_height_in_map_units_minus1 + 1);
		n = h264_intlog2(
			(pic_size_in_map_units /
			 (ctx->pps->slice_group_change_rate_minus1 + 1)) +
			1);
#endif
		H264_BITS(sh->slice_group_change_cycle, n);
	}

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_WRITE
	ctx->slice.hdr_len = bs->off * 8 + bs->cachebits;
#else
	ctx->slice.hdr_len = bs->off * 8 - bs->cachebits;
#endif

	return 0;
}


static int H264_SYNTAX_FCT(slice_data)(struct h264_bitstream *bs,
				       struct h264_ctx *ctx,
				       const struct h264_ctx_cbs *cbs,
				       void *userdata);


/**
 * 7.3.2.8 Slice layer without partitioning RBSP syntax
 */
static int H264_SYNTAX_FCT(slice_layer)(struct h264_bitstream *bs,
					struct h264_ctx *ctx,
					const struct h264_ctx_cbs *cbs,
					void *userdata)
{
	int res = 0;
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	struct h264_slice_header _sh;
	struct h264_slice_header *sh = &_sh;
	memset(sh, 0, sizeof(*sh));
#else
	struct h264_slice_header *sh = &ctx->slice.hdr;
#endif

	H264_BEGIN_STRUCT(slice_header);
	res = H264_SYNTAX_FCT(slice_header)(bs, ctx, sh);
	H264_END_STRUCT(slice_header);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	res = h264_ctx_set_slice_header(ctx, sh);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif

	res = H264_SYNTAX_FCT(slice_data)(bs, ctx, cbs, userdata);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	/* rbsp_slice_trailing_bits directly handled by slice_data */

	return 0;
}


/**
 * 7.3.1 NAL unit syntax
 */
static int H264_SYNTAX_FCT(nalu_header)(
	struct h264_bitstream *bs,
	H264_SYNTAX_CONST struct h264_nalu_header *nh)
{
	H264_BITS(nh->forbidden_zero_bit, 1);
	ULOG_ERRNO_RETURN_ERR_IF(nh->forbidden_zero_bit != 0, EIO);
	H264_BITS(nh->nal_ref_idc, 2);
	H264_BITS(nh->nal_unit_type, 5);
	return 0;
}


static int H264_SYNTAX_FCT(nalu)(struct h264_bitstream *bs,
				 struct h264_ctx *ctx,
				 const struct h264_ctx_cbs *cbs,
				 void *userdata)
{
	int res = 0;
	const uint8_t *buf = NULL;
	size_t len = 0;

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	buf = bs->cdata + bs->off;
	len = bs->len;
	res = h264_ctx_clear_nalu(ctx);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif

	H264_BEGIN_STRUCT(nalu_header);
	res = H264_SYNTAX_FCT(nalu_header)(bs, &ctx->nalu.hdr);
	H264_END_STRUCT(nalu_header);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	ctx->nalu.type = ctx->nalu.hdr.nal_unit_type;

	H264_CB(ctx, cbs, userdata, nalu_begin, ctx->nalu.type, buf, len);

	switch (ctx->nalu.type) {
	case H264_NALU_TYPE_SLICE:
	case H264_NALU_TYPE_SLICE_IDR:
		H264_BEGIN_STRUCT(slice);
		res = H264_SYNTAX_FCT(slice_layer)(bs, ctx, cbs, userdata);
		H264_END_STRUCT(slice);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		H264_CB(ctx, cbs, userdata, slice, buf, len, &ctx->slice.hdr);
		break;

	case H264_NALU_TYPE_SLICE_DPA:
	case H264_NALU_TYPE_SLICE_DPB:
	case H264_NALU_TYPE_SLICE_DPC:
		/* TODO */
		ctx->nalu.unknown = 1;
		break;

	case H264_NALU_TYPE_SEI:
		H264_BEGIN_ARRAY(sei);
		res = H264_SYNTAX_FCT(sei)(bs, ctx, cbs, userdata);
		H264_END_ARRAY(sei);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		break;

	case H264_NALU_TYPE_SPS: {
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
		struct h264_sps _sps;
		struct h264_sps *sps = &_sps;
		memset(sps, 0, sizeof(*sps));
		/* 7.4.2.1.1 Sequence parameter set data semantics */
		sps->chroma_format_idc = 1;
#else
		struct h264_sps *sps = ctx->sps;
		ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EIO);
#endif
		ULOG_ERRNO_RETURN_ERR_IF(ctx->nalu.hdr.nal_ref_idc == 0, EIO);
		H264_BEGIN_STRUCT(sps);
		res = H264_SYNTAX_FCT(sps)(bs, sps);
		H264_END_STRUCT(sps);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
		res = h264_ctx_set_sps(ctx, sps);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif
		H264_CB(ctx, cbs, userdata, sps, buf, len, ctx->sps);
		break;
	}

	case H264_NALU_TYPE_PPS: {
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
		struct h264_pps _pps;
		struct h264_pps *pps = &_pps;
		memset(pps, 0, sizeof(*pps));
#else
		struct h264_pps *pps = ctx->pps;
		ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EIO);
#endif
		ULOG_ERRNO_RETURN_ERR_IF(ctx->nalu.hdr.nal_ref_idc == 0, EIO);
		H264_BEGIN_STRUCT(pps);
		res = H264_SYNTAX_FCT(pps_with_ctx)(bs, ctx, pps);
		H264_END_STRUCT(pps);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
		res = h264_ctx_set_pps(ctx, pps);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif
		H264_CB(ctx, cbs, userdata, pps, buf, len, ctx->pps);
		break;
	}

	case H264_NALU_TYPE_AUD: {
		ULOG_ERRNO_RETURN_ERR_IF(ctx->nalu.hdr.nal_ref_idc != 0, EIO);
		H264_BEGIN_STRUCT(aud);
		res = H264_SYNTAX_FCT(aud)(bs, &ctx->aud);
		H264_END_STRUCT(aud);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		H264_CB(ctx, cbs, userdata, aud, buf, len, &ctx->aud);
		break;
	}

	default:
		/* TODO */
		ctx->nalu.unknown = 1;
		break;
	}

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
	/* 7.4.1.2.4 Access unit change detection */
	if ((ctx->nalu.is_prev_vcl) &&
	    ((ctx->nalu.type == H264_NALU_TYPE_AUD) ||
	     (ctx->nalu.type == H264_NALU_TYPE_SPS) ||
	     (ctx->nalu.type == H264_NALU_TYPE_PPS) ||
	     (ctx->nalu.type == H264_NALU_TYPE_SEI) ||
	     (((unsigned)ctx->nalu.type >= 14) &&
	      ((unsigned)ctx->nalu.type <= 18)) ||
	     (ctx->nalu.is_first_vcl))) {
		H264_CB(ctx, cbs, userdata, au_end);
	}

	if ((ctx->nalu.type != H264_NALU_TYPE_SLICE) &&
	    (ctx->nalu.type != H264_NALU_TYPE_SLICE_IDR)) {
		ctx->nalu.is_prev_vcl = 0;
	} else {
		ctx->nalu.is_prev_vcl = 1;
	}
#endif

	H264_CB(ctx, cbs, userdata, nalu_end, ctx->nalu.type, buf, len);

	return res;
}


/* This header will overwrite H264_BITS_xxx macros so don't put any generic
 * syntax parsing function after */
#include "h264_syntax_slice_data.h"


#endif /* _H264_SYNTAX_H_ */
