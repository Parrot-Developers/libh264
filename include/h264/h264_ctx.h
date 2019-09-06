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

#ifndef _H264_CTX_H_
#define _H264_CTX_H_


struct h264_ctx;


struct h264_ctx_cbs {
	/* Warning: this function is called after the nalu_begin(), slice*(),
	 * sps(), pps(), aud() and sei*() callback functions for the first NAL
	 * unit of the next AU, but before the nalu_end() callback function
	 * for the first NAL unit of the next AU is called:
	 * - nalu_begin(NALU 1 of frame n+1)
	 * - then <other_cbs: sps,pps,slice,etc>(NALU 1 of frame n+1)
	 * - then au_end(frame n)
	 * - then nalu_end(NALU 1 of frame n+1)
	 * Warning: this function will not be called for the last AU of a
	 * bitstream, as no subsequent NAL units are available for AU change
	 * detection.
	 */
	void (*au_end)(struct h264_ctx *ctx, void *userdata);

	void (*nalu_begin)(struct h264_ctx *ctx,
			   enum h264_nalu_type type,
			   const uint8_t *buf,
			   size_t len,
			   const struct h264_nalu_header *nh,
			   void *userdata);

	void (*nalu_end)(struct h264_ctx *ctx,
			 enum h264_nalu_type type,
			 const uint8_t *buf,
			 size_t len,
			 const struct h264_nalu_header *nh,
			 void *userdata);

	void (*slice)(struct h264_ctx *ctx,
		      const uint8_t *buf,
		      size_t len,
		      const struct h264_slice_header *sh,
		      void *userdata);

	void (*slice_data_begin)(struct h264_ctx *ctx,
				 const struct h264_slice_header *sh,
				 void *userdata);

	void (*slice_data_end)(struct h264_ctx *ctx,
			       const struct h264_slice_header *sh,
			       uint32_t mb_count,
			       void *userdata);

	void (*slice_data_mb)(struct h264_ctx *ctx,
			      const struct h264_slice_header *sh,
			      uint32_t mb_addr,
			      enum h264_mb_type mb_type,
			      void *userdata);

	void (*sps)(struct h264_ctx *ctx,
		    const uint8_t *buf,
		    size_t len,
		    const struct h264_sps *sps,
		    void *userdata);

	void (*pps)(struct h264_ctx *ctx,
		    const uint8_t *buf,
		    size_t len,
		    const struct h264_pps *pps,
		    void *userdata);

	void (*aud)(struct h264_ctx *ctx,
		    const uint8_t *buf,
		    size_t len,
		    const struct h264_aud *aud,
		    void *userdata);

	void (*sei)(struct h264_ctx *ctx,
		    enum h264_sei_type type,
		    const uint8_t *buf,
		    size_t len,
		    void *userdata);

	void (*sei_buffering_period)(
		struct h264_ctx *ctx,
		const uint8_t *buf,
		size_t len,
		const struct h264_sei_buffering_period *sei,
		void *userdata);

	void (*sei_pic_timing)(struct h264_ctx *ctx,
			       const uint8_t *buf,
			       size_t len,
			       const struct h264_sei_pic_timing *sei,
			       void *userdata);

	void (*sei_pan_scan_rect)(struct h264_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h264_sei_pan_scan_rect *sei,
				  void *userdata);

	void (*sei_filler_payload)(struct h264_ctx *ctx,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_sei_filler_payload *sei,
				   void *userdata);

	void (*sei_user_data_registered)(
		struct h264_ctx *ctx,
		const uint8_t *buf,
		size_t len,
		const struct h264_sei_user_data_registered *sei,
		void *userdata);

	void (*sei_user_data_unregistered)(
		struct h264_ctx *ctx,
		const uint8_t *buf,
		size_t len,
		const struct h264_sei_user_data_unregistered *sei,
		void *userdata);

	void (*sei_recovery_point)(struct h264_ctx *ctx,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_sei_recovery_point *sei,
				   void *userdata);
};


H264_API
int h264_ctx_new(struct h264_ctx **ret_obj);


H264_API
int h264_ctx_destroy(struct h264_ctx *ctx);


H264_API
int h264_ctx_clear(struct h264_ctx *ctx);


H264_API
int h264_ctx_clear_nalu(struct h264_ctx *ctx);


H264_API
int h264_ctx_set_nalu_header(struct h264_ctx *ctx,
			     const struct h264_nalu_header *nh);


H264_API
int h264_ctx_is_nalu_unknown(struct h264_ctx *ctx);


H264_API
int h264_ctx_set_aud(struct h264_ctx *ctx, const struct h264_aud *aud);


H264_API
int h264_ctx_set_sps(struct h264_ctx *ctx, const struct h264_sps *sps);


H264_API
int h264_ctx_set_pps(struct h264_ctx *ctx, const struct h264_pps *pps);


H264_API
int h264_ctx_set_filler(struct h264_ctx *ctx, size_t len);


H264_API
const struct h264_sps *h264_ctx_get_sps(struct h264_ctx *ctx);


H264_API
const struct h264_pps *h264_ctx_get_pps(struct h264_ctx *ctx);


H264_API
int h264_ctx_add_sei(struct h264_ctx *ctx, const struct h264_sei *sei);


H264_API
int h264_ctx_get_sei_count(struct h264_ctx *ctx);


H264_API uint64_t
h264_ctx_sei_pic_timing_to_ts(struct h264_ctx *ctx,
			      const struct h264_sei_pic_timing *sei);


H264_API uint64_t
h264_ctx_sei_pic_timing_to_us(struct h264_ctx *ctx,
			      const struct h264_sei_pic_timing *sei);


H264_API
int h264_ctx_set_slice_header(struct h264_ctx *ctx,
			      const struct h264_slice_header *sh);


H264_API int h264_ctx_get_info(struct h264_ctx *ctx, struct h264_info *info);


#endif /* !_H264_CTX_H_ */
