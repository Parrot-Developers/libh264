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

ULOG_DECLARE_TAG(h264);


/* E.2.1 "aspect_ratio_idc" */
#define H264_EXTENDED_SAR 255
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
int h264_get_sps_derived(const struct h264_sps *sps,
			 struct h264_sps_derived *sps_derived)
{
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_derived == NULL, EINVAL);
	memset(sps_derived, 0, sizeof(*sps_derived));

	sps_derived->ChromaArrayType =
		sps->separate_colour_plane_flag ? 0 : sps->chroma_format_idc;

	switch (sps_derived->ChromaArrayType) {
	case H264_COLOR_FORMAT_MONO:
		sps_derived->SubWidthC = 0;
		sps_derived->SubHeightC = 0;
		sps_derived->MbWidthC = 0;
		sps_derived->MbHeightC = 0;
		break;
	case H264_COLOR_FORMAT_YUV420:
		sps_derived->SubWidthC = 2;
		sps_derived->SubHeightC = 2;
		sps_derived->MbWidthC = 8;
		sps_derived->MbHeightC = 8;
		break;
	case H264_COLOR_FORMAT_YUV422:
		sps_derived->SubWidthC = 2;
		sps_derived->SubHeightC = 1;
		sps_derived->MbWidthC = 8;
		sps_derived->MbHeightC = 16;
		break;
	case H264_COLOR_FORMAT_YUV444:
		sps_derived->SubWidthC = 1;
		sps_derived->SubHeightC = 1;
		sps_derived->MbWidthC = 16;
		sps_derived->MbHeightC = 16;
		break;
	}

	sps_derived->BitDepthLuma = sps->bit_depth_luma_minus8 + 8;
	sps_derived->QpBdOffsetLuma = 6 * sps->bit_depth_luma_minus8;
	sps_derived->BitDepthChroma = sps->bit_depth_chroma_minus8 + 8;
	sps_derived->QpBdOffsetChroma = 6 * sps->bit_depth_chroma_minus8;
	sps_derived->RawMbBits = 256 * sps_derived->BitDepthLuma +
				 2 * sps_derived->MbWidthC *
					 sps_derived->MbHeightC *
					 sps_derived->BitDepthChroma;

	sps_derived->MaxFrameNum = 1 << (sps->log2_max_frame_num_minus4 + 4);
	sps_derived->MaxPicOrderCntLsb =
		1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4);

	sps_derived->PicWidthInMbs = sps->pic_width_in_mbs_minus1 + 1;
	sps_derived->PicWidthInSamplesLuma = sps_derived->PicWidthInMbs * 16;
	sps_derived->PicWidthInSamplesChroma =
		/* codecheck_ignore[POINTER_LOCATION] */
		sps_derived->PicWidthInMbs * sps_derived->MbWidthC;
	sps_derived->PicHeightInMapUnits =
		sps->pic_height_in_map_units_minus1 + 1;
	sps_derived->PicSizeInMapUnits =
		/* codecheck_ignore[POINTER_LOCATION] */
		sps_derived->PicWidthInMbs * sps_derived->PicHeightInMapUnits;
	sps_derived->FrameHeightInMbs = (2 - sps->frame_mbs_only_flag) *
					sps_derived->PicHeightInMapUnits;

	if (sps_derived->ChromaArrayType == H264_COLOR_FORMAT_MONO) {
		sps_derived->CropUnitX = 1;
		sps_derived->CropUnitY = 2 - sps->frame_mbs_only_flag;
	} else {
		sps_derived->CropUnitX = sps_derived->SubWidthC;
		sps_derived->CropUnitY = sps_derived->SubHeightC *
					 (2 - sps->frame_mbs_only_flag);
	}

	sps_derived->Width =
		sps_derived->PicWidthInSamplesLuma -
		sps_derived->CropUnitX * (sps->frame_crop_left_offset +
					  sps->frame_crop_right_offset);
	sps_derived->Height =
		sps_derived->FrameHeightInMbs * 16 -
		sps_derived->CropUnitY * (sps->frame_crop_top_offset +
					  sps->frame_crop_bottom_offset);
	return 0;
}


int h264_get_info(const uint8_t *sps,
		  size_t sps_len,
		  const uint8_t *pps,
		  size_t pps_len,
		  struct h264_info *info)
{
	int res;
	struct h264_sps *_sps = NULL;
	struct h264_sps_derived sps_derived;

	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_len == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_len == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);

	memset(info, 0, sizeof(*info));

	_sps = calloc(1, sizeof(*_sps));
	if (_sps == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}

	res = h264_parse_sps(sps, sps_len, _sps);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_sps", -res);
		goto out;
	}

	res = h264_get_sps_derived(_sps, &sps_derived);
	if (res < 0) {
		ULOG_ERRNO("h264_get_sps_derived", -res);
		goto out;
	}

	info->width = sps_derived.PicWidthInSamplesLuma;
	info->height = sps_derived.FrameHeightInMbs * 16;
	info->crop_left = 0;
	info->crop_top = 0;
	info->crop_width = info->width;
	info->crop_height = info->height;
	if (_sps->frame_cropping_flag) {
		info->crop_left =
			_sps->frame_crop_left_offset * sps_derived.CropUnitX;
		info->crop_width = info->width - _sps->frame_crop_right_offset *
							 sps_derived.CropUnitX;
		info->crop_top =
			_sps->frame_crop_top_offset * sps_derived.CropUnitY;
		info->crop_height =
			info->height -
			_sps->frame_crop_bottom_offset * sps_derived.CropUnitY;
	}

	info->sar_width = 1;
	info->sar_height = 1;
	if (_sps->vui_parameters_present_flag) {
		if (_sps->vui.aspect_ratio_info_present_flag) {
			if (_sps->vui.aspect_ratio_idc == H264_EXTENDED_SAR) {
				info->sar_width = _sps->vui.sar_width;
				info->sar_height = _sps->vui.sar_height;
			} else if (_sps->vui.aspect_ratio_idc <= 16) {
				info->sar_width =
					h264_sar[_sps->vui.aspect_ratio_idc][0];
				info->sar_height =
					h264_sar[_sps->vui.aspect_ratio_idc][1];
			}
		}
		info->full_range = _sps->vui.video_full_range_flag;
		if (_sps->vui.timing_info_present_flag) {
			info->num_units_in_tick = _sps->vui.num_units_in_tick;
			info->time_scale = _sps->vui.time_scale;
			info->framerate = (float)info->time_scale / 2. /
					  info->num_units_in_tick;
		}
		if (_sps->vui.nal_hrd_parameters_present_flag) {
			info->nal_hrd_bitrate =
				(_sps->vui.nal_hrd.cpb[0]
					 .bit_rate_value_minus1 +
				 1)
				<< (6 + _sps->vui.nal_hrd.bit_rate_scale);
			info->nal_hrd_cpb_size =
				(_sps->vui.nal_hrd.cpb[0]
					 .cpb_size_value_minus1 +
				 1)
				<< (4 + _sps->vui.nal_hrd.cpb_size_scale);
		}
		if (_sps->vui.vcl_hrd_parameters_present_flag) {
			info->vcl_hrd_bitrate =
				(_sps->vui.vcl_hrd.cpb[0]
					 .bit_rate_value_minus1 +
				 1)
				<< (6 + _sps->vui.vcl_hrd.bit_rate_scale);
			info->vcl_hrd_cpb_size =
				(_sps->vui.vcl_hrd.cpb[0]
					 .cpb_size_value_minus1 +
				 1)
				<< (4 + _sps->vui.vcl_hrd.cpb_size_scale);
		}
	}

	res = 0;

out:
	free(_sps);
	return res;
}
