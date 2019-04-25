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

#ifndef _H264_SLICE_DATA_H_
#define _H264_SLICE_DATA_H_


void h264_clear_macroblock_table(struct h264_ctx *ctx);


int h264_new_macroblock(struct h264_ctx *ctx,
			uint32_t mbAddr,
			int skipped,
			int field_flag);


int h264_set_nz_coeff(struct h264_ctx *ctx,
		      uint32_t mbAddr,
		      uint32_t comp,
		      uint32_t blkIdx,
		      uint32_t n);


int h264_read_mb_type(struct h264_bitstream *bs,
		      struct h264_ctx *ctx,
		      struct h264_macroblock *mb);


int h264_read_sub_mb_type(struct h264_bitstream *bs,
			  struct h264_ctx *ctx,
			  struct h264_macroblock *mb);


int h264_read_coded_block_pattern(struct h264_bitstream *bs,
				  struct h264_ctx *ctx,
				  struct h264_macroblock *mb);


int h264_read_coeff_token(struct h264_bitstream *bs,
			  struct h264_ctx *ctx,
			  struct h264_macroblock *mb,
			  uint32_t mode,
			  uint32_t comp,
			  uint32_t blkIdx,
			  uint32_t *trailing_ones,
			  uint32_t *total_coeff);


int h264_read_total_zeros(struct h264_bitstream *bs,
			  uint32_t total_coeff,
			  uint32_t max_num_coeff,
			  uint32_t *total_zeros);


int h264_read_run_before(struct h264_bitstream *bs,
			 uint32_t zeros_left,
			 uint32_t *run_before);


uint32_t h264_next_mb_addr(struct h264_ctx *ctx, uint32_t mbAddr);


#endif /* !_H264_SLICE_DATA_H_ */
