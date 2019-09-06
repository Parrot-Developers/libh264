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

#include "h264_priv.h"


/* E.2.1 "aspect_ratio_idc" */
static const unsigned int h264_sar[17][2] = {
	{1, 1},
	{1, 1},
	{12, 11},
	{10, 11},
	{16, 11},
	{40, 33},
	{24, 11},
	{20, 11},
	{32, 11},
	{80, 33},
	{18, 11},
	{15, 11},
	{64, 33},
	{160, 99},
	{4, 3},
	{3, 2},
	{2, 1},
};


/**
 * 6.2 Source, decoded, and output picture formats
 * 7.4.2.1.1 Sequence parameter set data semantics
 */
static void h264_ctx_update_derived_vars_sps(struct h264_ctx *ctx)
{
	const struct h264_sps *sps = ctx->sps;
	if (sps != NULL)
		h264_get_sps_derived(sps, &ctx->sps_derived);
}


/**
 * 7.4.2.2 Picture parameter set RBSP semantics
 */
static void h264_ctx_update_derived_vars_pps(struct h264_ctx *ctx)
{
	const struct h264_pps *pps = ctx->pps;
	if (pps == NULL)
		return;

	ctx->derived.SliceGroupChangeRate =
		pps->slice_group_change_rate_minus1 + 1;
}


/**
 * 7.4.3 Slice header semantics
 */
static void h264_ctx_update_derived_vars_slice(struct h264_ctx *ctx)
{
	const struct h264_sps *sps = ctx->sps;
	const struct h264_pps *pps = ctx->pps;
	const struct h264_slice_header *sh = &ctx->slice.hdr;
	if (sps == NULL || pps == NULL)
		return;

	ctx->derived.MbaffFrameFlag =
		sps->mb_adaptive_frame_field_flag && !sh->field_pic_flag;

	ctx->derived.PicHeightInMbs =
		ctx->sps_derived.FrameHeightInMbs / (1 + sh->field_pic_flag);
	ctx->derived.PicSizeInMbs =
		/* codecheck_ignore[POINTER_LOCATION] */
		ctx->sps_derived.PicWidthInMbs * ctx->derived.PicHeightInMbs;

	ctx->derived.PicHeightInSamplesLuma = ctx->derived.PicHeightInMbs * 16;
	ctx->derived.PicHeightInSamplesChroma =
		/* codecheck_ignore[POINTER_LOCATION] */
		ctx->derived.PicHeightInMbs * ctx->sps_derived.MbHeightC;

	ctx->derived.MaxPicNum = sh->field_pic_flag
					 ? 2 * ctx->sps_derived.MaxFrameNum
					 : ctx->sps_derived.MaxFrameNum;
	ctx->derived.CurrPicNum =
		sh->field_pic_flag ? 2 * sh->frame_num + 1 : sh->frame_num;

	ctx->derived.SliceQPLuma =
		pps->pic_init_qp_minus26 + 26 + sh->slice_qp_delta;
	ctx->derived.QSLuma =
		pps->pic_init_qs_minus26 + 26 + sh->slice_qs_delta;

	ctx->derived.FilterOffsetA = sh->slice_alpha_c0_offset_div2 << 1;
	ctx->derived.FilterOffsetB = sh->slice_beta_offset_div2 << 1;

	ctx->derived.MapUnitsInSliceGroup0 =
		Min(sh->slice_group_change_cycle *
			    ctx->derived.SliceGroupChangeRate,
		    ctx->sps_derived.PicSizeInMapUnits);

	if (pps->num_slice_groups_minus1 > 0)
		h264_gen_slice_group_map(ctx);
}


/**
 * 7.4.1.2.4 Detection of the first VCL NAL unit of a primary coded picture
 */
static void h264_ctx_detect_first_vcl_nalu(struct h264_ctx *ctx)
{
	const struct h264_sps *sps = ctx->sps;
	const struct h264_nalu_header *nh = &ctx->nalu.hdr;
	const struct h264_slice_header *sh = &ctx->slice.hdr;

	ctx->nalu.is_first_vcl = 0;

	if ((!ctx->nalu.is_prev_vcl) && (!ctx->nalu.is_prev_filler)) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* frame_num differs in value. */
	if (sh->frame_num != ctx->slice.prev_slice_hdr.frame_num) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* pic_parameter_set_id differs in value. */
	if (sh->pic_parameter_set_id !=
	    ctx->slice.prev_slice_hdr.pic_parameter_set_id) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* field_pic_flag differs in value. */
	if (!sps->frame_mbs_only_flag &&
	    (sh->field_pic_flag != ctx->slice.prev_slice_hdr.field_pic_flag)) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* bottom_field_flag is present in both and differs in value. */
	if (!sps->frame_mbs_only_flag && sh->field_pic_flag &&
	    ctx->slice.prev_slice_hdr.field_pic_flag &&
	    (sh->bottom_field_flag !=
	     ctx->slice.prev_slice_hdr.bottom_field_flag)) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* nal_ref_idc differs in value with one of the nal_ref_idc
	 * values being equal to 0. */
	if (!nh->nal_ref_idc != !ctx->slice.prev_slice_nalu_hdr.nal_ref_idc) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* pic_order_cnt_type is equal to 0 for both and either
	 * pic_order_cnt_lsb differs in value, or delta_pic_order_cnt_bottom
	 * differs in value. */
	if ((sps->pic_order_cnt_type == 0) &&
	    ((sh->pic_order_cnt_lsb !=
	      ctx->slice.prev_slice_hdr.pic_order_cnt_lsb) ||
	     (sh->delta_pic_order_cnt_bottom !=
	      ctx->slice.prev_slice_hdr.delta_pic_order_cnt_bottom))) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* pic_order_cnt_type is equal to 1 for both and either
	 * delta_pic_order_cnt[ 0 ] differs in value, or
	 * delta_pic_order_cnt[ 1 ] differs in value. */
	if ((sps->pic_order_cnt_type == 1) &&
	    ((sh->delta_pic_order_cnt[0] !=
	      ctx->slice.prev_slice_hdr.delta_pic_order_cnt[0]) ||
	     (sh->delta_pic_order_cnt[1] !=
	      ctx->slice.prev_slice_hdr.delta_pic_order_cnt[1]))) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* IdrPicFlag differs in value. */
	if ((nh->nal_unit_type == H264_NALU_TYPE_SLICE_IDR) !=
	    (ctx->slice.prev_slice_nalu_hdr.nal_unit_type ==
	     H264_NALU_TYPE_SLICE_IDR)) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

	/* IdrPicFlag is equal to 1 for both and idr_pic_id differs in value. */
	if ((nh->nal_unit_type == H264_NALU_TYPE_SLICE_IDR) &&
	    (ctx->slice.prev_slice_nalu_hdr.nal_unit_type ==
	     H264_NALU_TYPE_SLICE_IDR) &&
	    (sh->idr_pic_id != ctx->slice.prev_slice_hdr.idr_pic_id)) {
		ctx->nalu.is_first_vcl = 1;
		goto out;
	}

out:
	ctx->slice.prev_slice_nalu_hdr = *nh;
	ctx->slice.prev_slice_hdr = *sh;
}


int h264_ctx_new(struct h264_ctx **ret_obj)
{
	struct h264_ctx *ctx = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;

	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		return -ENOMEM;

	*ret_obj = ctx;
	return 0;
}


int h264_ctx_destroy(struct h264_ctx *ctx)
{
	if (ctx == NULL)
		return 0;
	h264_ctx_clear(ctx);
	free(ctx);
	return 0;
}


int h264_ctx_clear(struct h264_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	h264_ctx_clear_nalu(ctx);
	for (size_t i = 0; i < ARRAY_SIZE(ctx->sps_table); i++)
		free(ctx->sps_table[i]);
	for (size_t i = 0; i < ARRAY_SIZE(ctx->pps_table); i++)
		free(ctx->pps_table[i]);
	free(ctx->slice.mb_table.info);
	free(ctx->slice.group_map);
	memset(ctx, 0, sizeof(*ctx));
	return 0;
}


int h264_ctx_clear_nalu(struct h264_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	/* Save and restore is_prev_vcl and is_prev_filler */
	int is_prev_vcl = ctx->nalu.is_prev_vcl;
	int is_prev_filler = ctx->nalu.is_prev_filler;
	memset(&ctx->nalu, 0, sizeof(ctx->nalu));
	ctx->nalu.is_prev_vcl = is_prev_vcl;
	ctx->nalu.is_prev_filler = is_prev_filler;
	memset(&ctx->aud, 0, sizeof(ctx->aud));
	/* Keep current SPS/PPS */
	h264_ctx_clear_sei_table(ctx);
	h264_ctx_clear_slice(ctx);
	return 0;
}


int h264_ctx_set_nalu_header(struct h264_ctx *ctx,
			     const struct h264_nalu_header *nh)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(nh == NULL, EINVAL);
	ctx->nalu.type = nh->nal_unit_type;
	ctx->nalu.hdr = *nh;
	return 0;
}


int h264_ctx_is_nalu_unknown(struct h264_ctx *ctx)
{
	return ctx == NULL ? 0 : ctx->nalu.unknown;
}


int h264_ctx_set_aud(struct h264_ctx *ctx, const struct h264_aud *aud)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(aud == NULL, EINVAL);
	ctx->aud = *aud;
	return 0;
}


int h264_ctx_set_sps(struct h264_ctx *ctx, const struct h264_sps *sps)
{
	struct h264_sps **p_sps = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps->seq_parameter_set_id >=
					 ARRAY_SIZE(ctx->sps_table),
				 EINVAL);

	p_sps = &ctx->sps_table[sps->seq_parameter_set_id];
	if (*p_sps == NULL) {
		*p_sps = calloc(1, sizeof(**p_sps));
		if (*p_sps == NULL)
			return -ENOMEM;
	}
	**p_sps = *sps;
	ctx->sps = *p_sps;
	h264_ctx_update_derived_vars_sps(ctx);
	h264_ctx_update_derived_vars_slice(ctx);

	return 0;
}


int h264_ctx_set_pps(struct h264_ctx *ctx, const struct h264_pps *pps)
{
	struct h264_pps **p_pps = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps->pic_parameter_set_id >=
					 ARRAY_SIZE(ctx->pps_table),
				 EINVAL);

	p_pps = &ctx->pps_table[pps->pic_parameter_set_id];
	if (*p_pps == NULL) {
		*p_pps = calloc(1, sizeof(**p_pps));
		if (*p_pps == NULL)
			return -ENOMEM;
	}
	**p_pps = *pps;
	ctx->pps = *p_pps;
	h264_ctx_update_derived_vars_pps(ctx);
	h264_ctx_update_derived_vars_slice(ctx);

	return 0;
}


int h264_ctx_set_filler(struct h264_ctx *ctx, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ctx->filler_len = len;
	return 0;
}


int h264_ctx_set_active_sps(struct h264_ctx *ctx, uint32_t sps_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_id >= ARRAY_SIZE(ctx->sps_table), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx->sps_table[sps_id] == NULL, EINVAL);
	ctx->sps = ctx->sps_table[sps_id];
	h264_ctx_update_derived_vars_sps(ctx);
	h264_ctx_update_derived_vars_slice(ctx);
	return 0;
}


int h264_ctx_set_active_pps(struct h264_ctx *ctx, uint32_t pps_id)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_id >= ARRAY_SIZE(ctx->pps_table), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx->pps_table[pps_id] == NULL, EINVAL);
	ctx->pps = ctx->pps_table[pps_id];
	h264_ctx_update_derived_vars_pps(ctx);
	return h264_ctx_set_active_sps(ctx, ctx->pps->seq_parameter_set_id);
}


const struct h264_sps *h264_ctx_get_sps(struct h264_ctx *ctx)
{
	return ctx == NULL ? NULL : ctx->sps;
}


const struct h264_pps *h264_ctx_get_pps(struct h264_ctx *ctx)
{
	return ctx == NULL ? NULL : ctx->pps;
}


int h264_ctx_clear_sei_table(struct h264_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	for (uint32_t i = 0; i < ctx->sei_count; i++)
		free(ctx->sei_table[i].raw.buf);
	free(ctx->sei_table);
	ctx->sei_table = NULL;
	ctx->sei_count = 0;
	return 0;
}


int h264_ctx_add_sei_internal(struct h264_ctx *ctx, struct h264_sei **ret_obj)
{
	struct h264_sei *newtable = NULL;
	struct h264_sei *sei = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);

	/* Increase table size */
	newtable = realloc(ctx->sei_table,
			   (ctx->sei_count + 1) * sizeof(struct h264_sei));
	if (newtable == NULL)
		return -ENOMEM;
	ctx->sei_table = newtable;

	/* Setup SEI pointer */
	sei = &ctx->sei_table[ctx->sei_count];
	memset(sei, 0, sizeof(*sei));
	ctx->sei_count++;
	*ret_obj = sei;
	return 0;
}


int h264_ctx_add_sei(struct h264_ctx *ctx, const struct h264_sei *sei)
{
	int res = 0;
	struct h264_bitstream bs;
	struct h264_sei *new_sei = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);

	/* Setup bitstream (without emulation prevention) */
	h264_bs_init(&bs, NULL, 0, 0);

	/* Allocate new SEI in internal table */
	res = h264_ctx_add_sei_internal(ctx, &new_sei);
	if (res < 0)
		goto error;
	*new_sei = *sei;

	/* Encode SEI payload in dynamic bitstream */
	res = h264_write_one_sei(&bs, ctx, new_sei);
	if (res < 0)
		goto error;

	/* Acquire buffer of bitstream */
	res = h264_bs_acquire_buf(&bs, &new_sei->raw.buf, &new_sei->raw.len);
	if (res < 0)
		goto error;

	/* Update internal buffer of SEI structures */
	res = h264_sei_update_internal_buf(new_sei);
	if (res < 0)
		goto error;

	h264_bs_clear(&bs);
	return 0;

error:
	if (new_sei != NULL) {
		free(new_sei->raw.buf);
		ctx->sei_count--;
	}
	h264_bs_clear(&bs);
	return res;
}


int h264_ctx_get_sei_count(struct h264_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	return ctx->sei_count;
}


uint64_t h264_ctx_sei_pic_timing_to_ts(struct h264_ctx *ctx,
				       const struct h264_sei_pic_timing *sei)
{
	uint64_t clock_timestamp;
	const struct h264_sps *sps;

	ULOG_ERRNO_RETURN_VAL_IF(ctx == NULL, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(sei == NULL, EINVAL, 0);

	sps = ctx->sps;
	ULOG_ERRNO_RETURN_VAL_IF(sps->vui.time_scale == 0, EPROTO, 0);
	ULOG_ERRNO_RETURN_VAL_IF(sps->vui.num_units_in_tick == 0, EPROTO, 0);

	clock_timestamp =
		(((uint64_t)sei->clk_ts[0].hours_value * 60 +
		  sei->clk_ts[0].minutes_value) *
			 60 +
		 sei->clk_ts[0].seconds_value) *
			sps->vui.time_scale +
		((uint64_t)sei->clk_ts[0].n_frames *
		 ((uint64_t)sps->vui.num_units_in_tick *
		  (1 + (uint64_t)sei->clk_ts[0].nuit_field_based_flag)));

	if (sei->clk_ts[0].time_offset < 0 &&
	    ((uint64_t)-sei->clk_ts[0].time_offset > clock_timestamp))
		clock_timestamp = 0;
	else
		clock_timestamp += sei->clk_ts[0].time_offset;

	return clock_timestamp;
}


uint64_t h264_ctx_sei_pic_timing_to_us(struct h264_ctx *ctx,
				       const struct h264_sei_pic_timing *sei)
{
	uint64_t clock_timestamp, clock_timestamp_us;
	const struct h264_sps *sps;

	ULOG_ERRNO_RETURN_VAL_IF(ctx == NULL, EINVAL, 0);
	ULOG_ERRNO_RETURN_VAL_IF(sei == NULL, EINVAL, 0);

	sps = ctx->sps;
	ULOG_ERRNO_RETURN_VAL_IF(sps->vui.time_scale == 0, EPROTO, 0);
	clock_timestamp = h264_ctx_sei_pic_timing_to_ts(ctx, sei);

	clock_timestamp_us =
		(clock_timestamp * 1000000 + sps->vui.time_scale / 2) /
		sps->vui.time_scale;

	return clock_timestamp_us;
}


int h264_ctx_clear_slice(struct h264_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ctx->slice.type = 0;
	memset(&ctx->slice.hdr, 0, sizeof(ctx->slice.hdr));
	ctx->slice.rawdata.partial = 0;
	ctx->slice.rawdata.partialbits = 0;
	ctx->slice.rawdata.buf = NULL;
	ctx->slice.rawdata.len = 0;
	h264_clear_macroblock_table(ctx);
	h264_clear_slice_group_map(ctx);
	memset(&ctx->_mb, 0, sizeof(ctx->_mb));
	ctx->mb = NULL;
	h264_ctx_update_derived_vars_slice(ctx);
	return 0;
}


int h264_ctx_set_slice_header(struct h264_ctx *ctx,
			      const struct h264_slice_header *sh)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sh == NULL, EINVAL);
	h264_ctx_clear_slice(ctx);
	ctx->slice.type = H264_SLICE_TYPE(sh->slice_type);
	ctx->slice.hdr = *sh;
	h264_ctx_update_derived_vars_slice(ctx);
	h264_ctx_detect_first_vcl_nalu(ctx);
	return 0;
}


int h264_get_info_from_ps(struct h264_sps *sps,
			  struct h264_pps *pps,
			  struct h264_sps_derived *sps_derived,
			  struct h264_info *info)
{
	memset(info, 0, sizeof(*info));

	info->width = sps_derived->PicWidthInSamplesLuma;
	info->height = sps_derived->FrameHeightInMbs * 16;
	info->bit_depth_luma = sps_derived->BitDepthLuma;
	info->crop_left = 0;
	info->crop_top = 0;
	info->crop_width = info->width;
	info->crop_height = info->height;
	if (sps->frame_cropping_flag) {
		info->crop_left =
			sps->frame_crop_left_offset * sps_derived->CropUnitX;
		info->crop_width = info->width - sps->frame_crop_right_offset *
							 sps_derived->CropUnitX;
		info->crop_top =
			sps->frame_crop_top_offset * sps_derived->CropUnitY;
		info->crop_height =
			info->height -
			sps->frame_crop_bottom_offset * sps_derived->CropUnitY;
	}

	info->sar_width = 1;
	info->sar_height = 1;
	if (sps->vui_parameters_present_flag) {
		if (sps->vui.aspect_ratio_info_present_flag) {
			if (sps->vui.aspect_ratio_idc ==
			    H264_ASPECT_RATIO_EXTENDED_SAR) {
				info->sar_width = sps->vui.sar_width;
				info->sar_height = sps->vui.sar_height;
			} else if (sps->vui.aspect_ratio_idc <= 16) {
				info->sar_width =
					h264_sar[sps->vui.aspect_ratio_idc][0];
				info->sar_height =
					h264_sar[sps->vui.aspect_ratio_idc][1];
			}
		}
		info->full_range = sps->vui.video_full_range_flag;
		if (sps->vui.colour_description_present_flag) {
			info->colour_description_present = 1;
			info->colour_primaries = sps->vui.colour_primaries;
			info->transfer_characteristics =
				sps->vui.transfer_characteristics;
			info->matrix_coefficients =
				sps->vui.matrix_coefficients;
		} else {
			/* No colour description present, set to default
			 * values (2 means unspecified) */
			info->colour_primaries = 2;
			info->transfer_characteristics = 2;
			info->matrix_coefficients = 2;
		}
		if (sps->vui.timing_info_present_flag) {
			info->num_units_in_tick = sps->vui.num_units_in_tick;
			info->time_scale = sps->vui.time_scale;
			info->framerate =
				(info->num_units_in_tick != 0)
					? (float)info->time_scale / 2.f /
						  info->num_units_in_tick
					: 0.f;
			info->framerate_num = info->time_scale;
			info->framerate_den = info->num_units_in_tick;
			if (info->framerate_num % 2 == 0)
				info->framerate_num /= 2;
			else
				info->framerate_den *= 2;
		}
		if (sps->vui.nal_hrd_parameters_present_flag) {
			info->nal_hrd_bitrate =
				(sps->vui.nal_hrd.cpb[0].bit_rate_value_minus1 +
				 1)
				<< (6 + sps->vui.nal_hrd.bit_rate_scale);
			info->nal_hrd_cpb_size =
				(sps->vui.nal_hrd.cpb[0].cpb_size_value_minus1 +
				 1)
				<< (4 + sps->vui.nal_hrd.cpb_size_scale);
		}
		if (sps->vui.vcl_hrd_parameters_present_flag) {
			info->vcl_hrd_bitrate =
				(sps->vui.vcl_hrd.cpb[0].bit_rate_value_minus1 +
				 1)
				<< (6 + sps->vui.vcl_hrd.bit_rate_scale);
			info->vcl_hrd_cpb_size =
				(sps->vui.vcl_hrd.cpb[0].cpb_size_value_minus1 +
				 1)
				<< (4 + sps->vui.vcl_hrd.cpb_size_scale);
		}
	}

	return 0;
}


int h264_ctx_get_info(struct h264_ctx *ctx, struct h264_info *info)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);

	if ((ctx->sps == NULL) || (ctx->pps == NULL))
		return -EAGAIN;

	return h264_get_info_from_ps(
		ctx->sps, ctx->pps, &ctx->sps_derived, info);
}


int h264_sar_to_aspect_ratio_idc(unsigned int sar_width,
				 unsigned int sar_height)
{
	unsigned int idx, found = 0;

	/* Start at index 1 as 0 is 'unspecified' */
	for (idx = 1; idx < ARRAY_SIZE(h264_sar); idx++) {
		if ((h264_sar[idx][0] == sar_width) &&
		    (h264_sar[idx][1] == sar_height)) {
			found = 1;
			break;
		}
	}

	return found ? idx : H264_ASPECT_RATIO_EXTENDED_SAR;
}
