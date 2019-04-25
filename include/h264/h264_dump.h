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

#ifndef _H264_DUMP_H_
#define _H264_DUMP_H_


struct json_object;
struct h264_dump;


/* Dump slice data (CAVLC only) */
#define H264_DUMP_FLAGS_SLICE_DATA 0x01


enum h264_dump_type {
	H264_DUMP_TYPE_JSON,
};


struct h264_dump_cfg {
	enum h264_dump_type type;
};


H264_API
int h264_dump_new(const struct h264_dump_cfg *cfg, struct h264_dump **ret_obj);


H264_API
int h264_dump_destroy(struct h264_dump *dump);


H264_API
int h264_dump_clear(struct h264_dump *dump);


H264_API
int h264_dump_get_json_object(struct h264_dump *dump,
			      struct json_object **jobj);


H264_API
int h264_dump_get_json_str(struct h264_dump *dump, const char **str);


H264_API
int h264_dump_nalu(struct h264_dump *dump,
		   struct h264_ctx *ctx,
		   uint32_t flags);


#endif /* !_H264_DUMP_H_ */
