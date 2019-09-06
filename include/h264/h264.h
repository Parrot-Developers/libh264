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

#ifndef _H264_H_
#define _H264_H_

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef H264_API_EXPORTS
#	ifdef _WIN32
#		define H264_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define H264_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !H264_API_EXPORTS */
#	define H264_API
#endif /* !H264_API_EXPORTS */

#include "h264/h264_types.h"

#include "h264/h264_bitstream.h"

#include "h264/h264_ctx.h"

#include "h264/h264_dump.h"
#include "h264/h264_reader.h"
#include "h264/h264_writer.h"


H264_API
int h264_get_sps_derived(const struct h264_sps *sps,
			 struct h264_sps_derived *sps_derived);


H264_API int h264_get_info(const uint8_t *sps,
			   size_t sps_len,
			   const uint8_t *pps,
			   size_t pps_len,
			   struct h264_info *info);


H264_API int h264_sar_to_aspect_ratio_idc(unsigned int sar_width,
					  unsigned int sar_height);


/* Note: this function expects start code length to be 4 bytes;
 * 3 bytes start codes are not supported */
H264_API int h264_byte_stream_to_avcc(uint8_t *data, size_t len);


H264_API int h264_avcc_to_byte_stream(uint8_t *data, size_t len);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_H264_H_ */
