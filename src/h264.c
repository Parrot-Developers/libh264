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
	int res = 0;
	struct h264_sps *_sps = NULL;
	struct h264_pps *_pps = NULL;
	struct h264_sps_derived sps_derived;

	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps_len == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps_len == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(info == NULL, EINVAL);

	memset(info, 0, sizeof(*info));

	_sps = calloc(1, sizeof(*_sps));
	if (_sps == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	res = h264_parse_sps(sps, sps_len, _sps);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_sps", -res);
		goto out;
	}

	_pps = calloc(1, sizeof(*_pps));
	if (_pps == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	res = h264_parse_pps(pps, pps_len, _sps, _pps);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_pps", -res);
		goto out;
	}

	res = h264_get_sps_derived(_sps, &sps_derived);
	if (res < 0) {
		ULOG_ERRNO("h264_get_sps_derived", -res);
		goto out;
	}

	res = h264_get_info_from_ps(_sps, _pps, &sps_derived, info);
	if (res < 0) {
		ULOG_ERRNO("h264_get_info_from_ps", -res);
		goto out;
	}

out:
	free(_sps);
	free(_pps);
	return res;
}


static int find_start_code(const uint8_t *buf, size_t len, size_t *start)
{
	const uint8_t *p = buf;

	while (len >= 4) {
		/* Search for the next 0x00 byte */
		if (*p != 0x00)
			goto next;

		/* Is it a 00 00 00 01 sequence? */
		if (p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01) {
			*start = p - buf;
			return 0;
		}

		/* clang-format off */
next:
		/* clang-format on */
		p++;
		len--;
	}

	return -ENOENT;
}


int h264_byte_stream_to_avcc(uint8_t *data, size_t len)
{
	int res;
	uint32_t nalu_len_be;
	size_t start, next_start, nalu_len;

	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	/* First NALU start code */
	res = find_start_code(data, len, &start);
	if (res < 0) {
		if (res != -ENOENT)
			return res;
		ULOGW("%s: no start code found", __func__);
		return 0;
	}
	data += start;
	len -= start;

	while (len > 4) {
		/* Find the next NALU start code */
		res = find_start_code(data + 4, len - 4, &next_start);
		if (res == -ENOENT) {
			/* Last NALU */
			nalu_len = len - 4;
		} else if (res < 0) {
			return res;
		} else {
			nalu_len = next_start;
		}
		nalu_len_be = htonl(nalu_len);
		memcpy(data, &nalu_len_be, sizeof(uint32_t));
		data += 4 + nalu_len;
		len -= (4 + nalu_len);
	}

	return 0;
}


int h264_avcc_to_byte_stream(uint8_t *data, size_t len)
{
	uint32_t nalu_len, start_code = htonl(0x00000001);
	size_t offset = 0;

	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	while (offset < len) {
		memcpy(&nalu_len, data, sizeof(uint32_t));
		nalu_len = ntohl(nalu_len);
		memcpy(data, &start_code, sizeof(uint32_t));
		data += 4 + nalu_len;
		offset += 4 + nalu_len;
	}

	return 0;
}
