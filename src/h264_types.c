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

/* codecheck_ignore[COMPLEX_MACRO] */
#define H264_ENUM_CASE(_prefix, _name)                                         \
	case _prefix##_name:                                                   \
		return #_name


const char *h264_nalu_type_str(enum h264_nalu_type val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_NALU_TYPE_, SLICE);
	H264_ENUM_CASE(H264_NALU_TYPE_, SLICE_DPA);
	H264_ENUM_CASE(H264_NALU_TYPE_, SLICE_DPB);
	H264_ENUM_CASE(H264_NALU_TYPE_, SLICE_DPC);
	H264_ENUM_CASE(H264_NALU_TYPE_, SLICE_IDR);
	H264_ENUM_CASE(H264_NALU_TYPE_, SEI);
	H264_ENUM_CASE(H264_NALU_TYPE_, SPS);
	H264_ENUM_CASE(H264_NALU_TYPE_, PPS);
	H264_ENUM_CASE(H264_NALU_TYPE_, AUD);
	H264_ENUM_CASE(H264_NALU_TYPE_, END_OF_SEQ);
	H264_ENUM_CASE(H264_NALU_TYPE_, END_OF_STREAM);
	H264_ENUM_CASE(H264_NALU_TYPE_, FILLER);

	case H264_NALU_TYPE_UNKNOWN: /* NO BREAK */
	default: return "UNKNOWN";
	}
	/* clang-format on */
}


const char *h264_slice_type_str(enum h264_slice_type val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_SLICE_TYPE_, P);
	H264_ENUM_CASE(H264_SLICE_TYPE_, B);
	H264_ENUM_CASE(H264_SLICE_TYPE_, I);
	H264_ENUM_CASE(H264_SLICE_TYPE_, SP);
	H264_ENUM_CASE(H264_SLICE_TYPE_, SI);

	case H264_SLICE_TYPE_UNKNOWN: /* NO BREAK */
	default: return "UNKNOWN";
	}
	/* clang-format on */
}


const char *h264_mb_type_str(enum h264_mb_type val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_MB_TYPE_, I_NxN);
	H264_ENUM_CASE(H264_MB_TYPE_, I_16x16);
	H264_ENUM_CASE(H264_MB_TYPE_, I_PCM);

	H264_ENUM_CASE(H264_MB_TYPE_, SI);

	H264_ENUM_CASE(H264_MB_TYPE_, P_16x16);
	H264_ENUM_CASE(H264_MB_TYPE_, P_16x8);
	H264_ENUM_CASE(H264_MB_TYPE_, P_8x16);
	H264_ENUM_CASE(H264_MB_TYPE_, P_8x8);
	H264_ENUM_CASE(H264_MB_TYPE_, P_8x8ref0);
	H264_ENUM_CASE(H264_MB_TYPE_, P_SKIP);

	H264_ENUM_CASE(H264_MB_TYPE_, B_Direct_16x16);
	H264_ENUM_CASE(H264_MB_TYPE_, B_16x16);
	H264_ENUM_CASE(H264_MB_TYPE_, B_16x8);
	H264_ENUM_CASE(H264_MB_TYPE_, B_8x16);
	H264_ENUM_CASE(H264_MB_TYPE_, B_8x8);
	H264_ENUM_CASE(H264_MB_TYPE_, B_SKIP);

	case H264_MB_TYPE_UNKNOWN: /* NO BREAK */
	default: return "UNKNOWN";
	}
	/* clang-format on */
}


int h264_mb_type_is_intra(enum h264_mb_type val)
{
	switch (val) {
	case H264_MB_TYPE_I_NxN:
		return 1;
	case H264_MB_TYPE_I_16x16:
		return 1;
	case H264_MB_TYPE_I_PCM:
		return 1;
	case H264_MB_TYPE_SI:
		return 1;
	default:
		return 0;
	}
}


int h264_mb_type_is_inter(enum h264_mb_type val)
{
	switch (val) {
	case H264_MB_TYPE_P_16x16:
		return 1;
	case H264_MB_TYPE_P_16x8:
		return 1;
	case H264_MB_TYPE_P_8x16:
		return 1;
	case H264_MB_TYPE_P_8x8:
		return 1;
	case H264_MB_TYPE_P_8x8ref0:
		return 1;
	case H264_MB_TYPE_P_SKIP:
		return 1;
	case H264_MB_TYPE_B_Direct_16x16:
		return 1;
	case H264_MB_TYPE_B_16x16:
		return 1;
	case H264_MB_TYPE_B_16x8:
		return 1;
	case H264_MB_TYPE_B_8x16:
		return 1;
	case H264_MB_TYPE_B_8x8:
		return 1;
	case H264_MB_TYPE_B_SKIP:
		return 1;
	default:
		return 0;
	}
}


const char *h264_profile_str(enum h264_profile val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_PROFILE_, CAVLC_444);
	H264_ENUM_CASE(H264_PROFILE_, BASELINE);
	H264_ENUM_CASE(H264_PROFILE_, MAIN);
	H264_ENUM_CASE(H264_PROFILE_, EXTENDED);
	H264_ENUM_CASE(H264_PROFILE_, HIGH);
	H264_ENUM_CASE(H264_PROFILE_, HIGH_10);
	H264_ENUM_CASE(H264_PROFILE_, HIGH_422);
	H264_ENUM_CASE(H264_PROFILE_, HIGH_444);

	default: return "UNKNOWN";
	}
	/* clang-format on */
}


const char *h264_color_format_str(enum h264_color_format val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_COLOR_FORMAT_, MONO);
	H264_ENUM_CASE(H264_COLOR_FORMAT_, YUV420);
	H264_ENUM_CASE(H264_COLOR_FORMAT_, YUV422);
	H264_ENUM_CASE(H264_COLOR_FORMAT_, YUV444);

	default: return "UNKNOWN";
	}
	/* clang-format on */
}


char *h264_aspect_ratio_str_alloc(enum h264_aspect_ratio val,
				  uint32_t sar_width,
				  uint32_t sar_height)
{
	int res;
	char *str = NULL;

	switch (val) {
	case H264_ASPECT_RATIO_UNSPECIFIED:
		return strdup("UNSPECIFIED");
	case H264_ASPECT_RATIO_1_1:
		return strdup("1:1");
	case H264_ASPECT_RATIO_12_11:
		return strdup("12:11");
	case H264_ASPECT_RATIO_10_11:
		return strdup("10:11");
	case H264_ASPECT_RATIO_16_11:
		return strdup("16:11");
	case H264_ASPECT_RATIO_40_33:
		return strdup("40:33");
	case H264_ASPECT_RATIO_24_11:
		return strdup("24:11");
	case H264_ASPECT_RATIO_20_11:
		return strdup("20:11");
	case H264_ASPECT_RATIO_32_11:
		return strdup("32:11");
	case H264_ASPECT_RATIO_80_33:
		return strdup("80:33");
	case H264_ASPECT_RATIO_18_11:
		return strdup("18:11");
	case H264_ASPECT_RATIO_15_11:
		return strdup("15:11");
	case H264_ASPECT_RATIO_64_33:
		return strdup("64:33");
	case H264_ASPECT_RATIO_160_99:
		return strdup("160:99");
	case H264_ASPECT_RATIO_4_3:
		return strdup("4:3");
	case H264_ASPECT_RATIO_3_2:
		return strdup("3:2");
	case H264_ASPECT_RATIO_2_1:
		return strdup("2:1");
	case H264_ASPECT_RATIO_EXTENDED_SAR:
		res = asprintf(
			&str, "EXTENDED_SAR_%u:%u", sar_width, sar_height);
		return str;
	default:
		return strdup("UNKNOWN");
	}
}


const char *h264_sei_type_str(enum h264_sei_type val)
{
	/* clang-format off */
	switch (val) {
	H264_ENUM_CASE(H264_SEI_TYPE_, BUFFERING_PERIOD);
	H264_ENUM_CASE(H264_SEI_TYPE_, PIC_TIMING);
	H264_ENUM_CASE(H264_SEI_TYPE_, PAN_SCAN_RECT);
	H264_ENUM_CASE(H264_SEI_TYPE_, FILLER_PAYLOAD);
	H264_ENUM_CASE(H264_SEI_TYPE_, USER_DATA_REGISTERED);
	H264_ENUM_CASE(H264_SEI_TYPE_, USER_DATA_UNREGISTERED);
	H264_ENUM_CASE(H264_SEI_TYPE_, RECOVERY_POINT);
	H264_ENUM_CASE(H264_SEI_TYPE_, DEC_REF_PIC_MARKING_REPETITION);
	H264_ENUM_CASE(H264_SEI_TYPE_, SPARE_PIC);
	H264_ENUM_CASE(H264_SEI_TYPE_, SCENE_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, SUB_SEQ_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, SUB_SEQ_LAYER_CHARACTERISTICS);
	H264_ENUM_CASE(H264_SEI_TYPE_, SUB_SEQ_CHARACTERISTICS);
	H264_ENUM_CASE(H264_SEI_TYPE_, FULL_FRAME_FREEZE);
	H264_ENUM_CASE(H264_SEI_TYPE_, FULL_FRAME_FREEZE_RELEASE);
	H264_ENUM_CASE(H264_SEI_TYPE_, FULL_FRAME_SNAPSHOT);
	H264_ENUM_CASE(H264_SEI_TYPE_, PROGRESSIVE_REFINEMENT_SEGMENT_START);
	H264_ENUM_CASE(H264_SEI_TYPE_, PROGRESSIVE_REFINEMENT_SEGMENT_END);
	H264_ENUM_CASE(H264_SEI_TYPE_, MOTION_CONSTRAINED_SLICE_GROUP_SET);
	H264_ENUM_CASE(H264_SEI_TYPE_, FILM_GRAIN_CHARACTERISTICS);
	H264_ENUM_CASE(H264_SEI_TYPE_, DEBLOCKING_FILTER_DISPLAY_PREFERENCE);
	H264_ENUM_CASE(H264_SEI_TYPE_, STEREO_VIDEO_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, POST_FILTER_HINT);
	H264_ENUM_CASE(H264_SEI_TYPE_, TONE_MAPPING_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, SCALABILITY_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, SUB_PIC_SCALABLE_LAYER);
	H264_ENUM_CASE(H264_SEI_TYPE_, NON_REQUIRED_LAYER_REP);
	H264_ENUM_CASE(H264_SEI_TYPE_, PRIORITY_LAYER_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, LAYERS_NOT_PRESENT);
	H264_ENUM_CASE(H264_SEI_TYPE_, LAYER_DEPENDENCY_CHANGE);
	H264_ENUM_CASE(H264_SEI_TYPE_, SCALABLE_NESTING);
	H264_ENUM_CASE(H264_SEI_TYPE_, BASE_LAYER_TEMPORAL_HRD);
	H264_ENUM_CASE(H264_SEI_TYPE_, QUALITY_LAYER_INTEGRITY_CHECK);
	H264_ENUM_CASE(H264_SEI_TYPE_, REDUNDANT_PIC_PROPERTY);
	H264_ENUM_CASE(H264_SEI_TYPE_, TL0_DEP_REP_INDEX);
	H264_ENUM_CASE(H264_SEI_TYPE_, TL_SWITCHING_POINT);
	H264_ENUM_CASE(H264_SEI_TYPE_, PARALLEL_DECODING_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, MVC_SCALABLE_NESTING);
	H264_ENUM_CASE(H264_SEI_TYPE_, VIEW_SCALABILITY_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, MULTIVIEW_SCENE_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, MULTIVIEW_ACQUISITION_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, NON_REQUIRED_VIEW_COMPONENT);
	H264_ENUM_CASE(H264_SEI_TYPE_, VIEW_DEPENDENCY_CHANGE);
	H264_ENUM_CASE(H264_SEI_TYPE_, OPERATION_POINTS_NOT_PRESENT);
	H264_ENUM_CASE(H264_SEI_TYPE_, BASE_VIEW_TEMPORAL_HRD);
	H264_ENUM_CASE(H264_SEI_TYPE_, FRAME_PACKING_ARRANGEMENT);
	H264_ENUM_CASE(H264_SEI_TYPE_, MULTIVIEW_VIEW_POSITION);
	H264_ENUM_CASE(H264_SEI_TYPE_, DISPLAY_ORIENTATION);
	H264_ENUM_CASE(H264_SEI_TYPE_, MVCD_SCALABLE_NESTING);
	H264_ENUM_CASE(H264_SEI_TYPE_, MVCD_VIEW_SCALABILITY_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, DEPTH_REPRESENTATION_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_,
			THREE_DIMENSIONAL_REFERENCE_DISPLAYS_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_, DEPTH_TIMING);
	H264_ENUM_CASE(H264_SEI_TYPE_, DEPTH_SAMPLING_INFO);
	H264_ENUM_CASE(H264_SEI_TYPE_,
			CONSTRAINED_DEPTH_PARAMETER_SET_IDENTIFIER);

	default: return "UNKNONW";
	}
	/* clang-format on */
}


int h264_sei_update_internal_buf(struct h264_sei *sei)
{
	uint32_t start = 0;
	const uint8_t **buf = NULL;
	size_t *len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(sei == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sei->raw.buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sei->raw.len == 0, EINVAL);

	switch (sei->type) {
	case H264_SEI_TYPE_FILLER_PAYLOAD:
		start = 0;
		buf = &sei->filler_payload.buf;
		len = &sei->filler_payload.len;
		break;

	case H264_SEI_TYPE_USER_DATA_REGISTERED:
		start = sei->user_data_registered.country_code == 0xff ? 2 : 1;
		buf = &sei->user_data_registered.buf;
		len = &sei->user_data_registered.len;
		break;

	case H264_SEI_TYPE_USER_DATA_UNREGISTERED:
		start = 16;
		buf = &sei->user_data_unregistered.buf;
		len = &sei->user_data_unregistered.len;
		break;

	default:
		break;
	}

	if (buf != NULL && len != NULL) {
		ULOG_ERRNO_RETURN_ERR_IF(sei->raw.len < start, EINVAL);
		*buf = sei->raw.buf + start;
		*len = sei->raw.len - start;
	}
	return 0;
}
