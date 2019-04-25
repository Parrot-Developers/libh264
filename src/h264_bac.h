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

#ifndef _H264_BAC_H_
#define _H264_BAC_H_


/* Binary arithmetic code, state */
struct h264_bac_state {
	/* Probability state index */
	uint8_t idx;

	/* Most probable symbol */
	uint8_t mps;
};


/* Binary arithmetic code, decoding context */
struct h264_bac_dec {
	struct h264_bitstream *bs;
	uint32_t codIRange;
	uint32_t codIOffset;
};


/* Binary arithmetic code, encoding context */
struct h264_bac_enc {
	struct h264_bitstream *bs;
	uint32_t codIRange;
	uint32_t codILow;
	int firstBitFlag;
	uint32_t bitsOutstanding;
	uint32_t BinCountsInNALunits;
};


void h264_bac_state_init(struct h264_bac_state *state,
			 int32_t SliceQPLuma,
			 int8_t m,
			 int8_t n);


int h264_bac_decode_init(struct h264_bac_dec *dec, struct h264_bitstream *bs);


int h264_bac_encode_init(struct h264_bac_enc *enc,
			 struct h264_bitstream *bs,
			 int first_slice);


int h264_bac_encode_bin(struct h264_bac_enc *enc,
			struct h264_bac_state *state,
			int bin);


int h264_bac_encode_bypass(struct h264_bac_enc *enc, int bin);


int h264_bac_encode_terminate(struct h264_bac_enc *enc, int bin);


#endif /* !_H264_BAC_H_ */
