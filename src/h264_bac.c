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

/* clang-format off */
/* codecheck_ignore[COMPLEX_MACRO] */
#define CHECK(_x) if ((res = (_x)) < 0) goto out
/* clang-format on */

#if 0
#	define BAC_LOGV(_fmt, ...) fprintf(stderr, _fmt "\n", ##__VA_ARGS__)
#else
#	define BAC_LOGV(_fmt, ...)
#endif


/* clang-format off */

/**
 * Table 9-44 Specification of rangeTabLPS depending on pStateIdx and
 * qCodIRangeIdx
 */
static const uint8_t s_h264_range_table_lps[64][4] = {
	[0]  = { 128, 176, 208, 240 },
	[1]  = { 128, 167, 197, 227 },
	[2]  = { 128, 158, 187, 216 },
	[3]  = { 123, 150, 178, 205 },
	[4]  = { 116, 142, 169, 195 },
	[5]  = { 111, 135, 160, 185 },
	[6]  = { 105, 128, 152, 175 },
	[7]  = { 100, 122, 144, 166 },
	[8]  = {  95, 116, 137, 158 },
	[9]  = {  90, 110, 130, 150 },
	[10] = {  85, 104, 123, 142 },
	[11] = {  81,  99, 117, 135 },
	[12] = {  77,  94, 111, 128 },
	[13] = {  73,  89, 105, 122 },
	[14] = {  69,  85, 100, 116 },
	[15] = {  66,  80,  95, 110 },
	[16] = {  62,  76,  90, 104 },
	[17] = {  59,  72,  86,  99 },
	[18] = {  56,  69,  81,  94 },
	[19] = {  53,  65,  77,  89 },
	[20] = {  51,  62,  73,  85 },
	[21] = {  48,  59,  69,  80 },
	[22] = {  46,  56,  66,  76 },
	[23] = {  43,  53,  63,  72 },
	[24] = {  41,  50,  59,  69 },
	[25] = {  39,  48,  56,  65 },
	[26] = {  37,  45,  54,  62 },
	[27] = {  35,  43,  51,  59 },
	[28] = {  33,  41,  48,  56 },
	[29] = {  32,  39,  46,  53 },
	[30] = {  30,  37,  43,  50 },
	[31] = {  29,  35,  41,  48 },
	[32] = {  27,  33,  39,  45 },
	[33] = {  26,  31,  37,  43 },
	[34] = {  24,  30,  35,  41 },
	[35] = {  23,  28,  33,  39 },
	[36] = {  22,  27,  32,  37 },
	[37] = {  21,  26,  30,  35 },
	[38] = {  20,  24,  29,  33 },
	[39] = {  19,  23,  27,  31 },
	[40] = {  18,  22,  26,  30 },
	[41] = {  17,  21,  25,  28 },
	[42] = {  16,  20,  23,  27 },
	[43] = {  15,  19,  22,  25 },
	[44] = {  14,  18,  21,  24 },
	[45] = {  14,  17,  20,  23 },
	[46] = {  13,  16,  19,  22 },
	[47] = {  12,  15,  18,  21 },
	[48] = {  12,  14,  17,  20 },
	[49] = {  11,  14,  16,  19 },
	[50] = {  11,  13,  15,  18 },
	[51] = {  10,  12,  15,  17 },
	[52] = {  10,  12,  14,  16 },
	[53] = {   9,  11,  13,  15 },
	[54] = {   9,  11,  12,  14 },
	[55] = {   8,  10,  12,  14 },
	[56] = {   8,   9,  11,  13 },
	[57] = {   7,   9,  11,  12 },
	[58] = {   7,   9,  10,  12 },
	[59] = {   7,   8,  10,  11 },
	[60] = {   6,   8,   9,  11 },
	[61] = {   6,   7,   9,  10 },
	[62] = {   6,   7,   8,   9 },
	[63] = {   2,   2,   2,   2 },
};


/**
 * Table 9-45 - State transition table
 */
static const uint8_t s_h264_trans_table_lps[64] = {
	 0,  0,  1,  2,  2,  4,  4,  5,
	 6,  7,  8,  9,  9, 11, 11, 12,
	13, 13, 15, 15, 16, 16, 18, 18,
	19, 19, 21, 21, 22, 22, 23, 24,
	24, 25, 26, 26, 27, 27, 28, 29,
	29, 30, 30, 30, 31, 32, 32, 33,
	33, 33, 34, 34, 35, 35, 35, 36,
	36, 36, 37, 37, 37, 38, 38, 63,
};


/**
 * Table 9-45 - State transition table
 */
static const uint8_t s_h264_trans_table_mps[64] = {
	 1,  2,  3,  4,  5,  6,  7,  8,
	 9, 10, 11, 12, 13, 14, 15, 16,
	17, 18, 19, 20, 21, 22, 23, 24,
	25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 37, 38, 39, 40,
	41, 42, 43, 44, 45, 46, 47, 48,
	49, 50, 51, 52, 53, 54, 55, 56,
	57, 58, 59, 60, 61, 62, 62, 63,
};

/* clang-format on */


/**
 * 9.3.4.3 Renormalization process in the arithmetic encoding engine
 * Figure 9-9 - Flowchart of PutBit(B)
 */
static int h264_bac_encode_put_bit(struct h264_bac_enc *enc, int bit)
{
	int res = 0;

	if (enc->firstBitFlag)
		enc->firstBitFlag = 0;
	else
		CHECK(h264_bs_write_bits(enc->bs, bit, 1));

	while (enc->bitsOutstanding > 0) {
		CHECK(h264_bs_write_bits(enc->bs, !bit, 1));
		enc->bitsOutstanding--;
	}

out:
	return res;
}


/**
 * 9.3.4.3 Renormalization process in the arithmetic encoding engine
 * Figure 9-8 - Flowchart of renormalization in the encoder
 */
static int h264_bac_encode_renorm(struct h264_bac_enc *enc)
{
	int res = 0;

	while (enc->codIRange < 256) {
		if (enc->codILow < 256) {
			CHECK(h264_bac_encode_put_bit(enc, 0));
		} else if (enc->codILow < 512) {
			enc->codILow -= 256;
			enc->bitsOutstanding++;
		} else {
			enc->codILow -= 512;
			CHECK(h264_bac_encode_put_bit(enc, 1));
		}
		enc->codIRange <<= 1;
		enc->codILow <<= 1;
	}

out:
	return res;
}


/**
 * 9.3.4.5 Encoding process for a binary decision before termination
 * Figure 9-12 - Flowchart of flushing at termination
 */
static int h264_bac_encode_flush(struct h264_bac_enc *enc)
{
	int res = 0;
	enc->codIRange = 2;
	CHECK(h264_bac_encode_renorm(enc));
	CHECK(h264_bac_encode_put_bit(enc, (enc->codILow >> 9) & 1));
	/* Force last bit to 1 (rbsp_stop_one_bit) */
	CHECK(h264_bs_write_bits(enc->bs, ((enc->codILow >> 7) & 3) | 1, 2));
out:
	return res;
}


/**
 * 9.3.1.1 Initialisation process for context variables
 */
void h264_bac_state_init(struct h264_bac_state *state,
			 int32_t SliceQPLuma,
			 int8_t m,
			 int8_t n)
{
	int32_t qp = Clip3(1, 51, SliceQPLuma);
	int32_t idx = ((m * qp) >> 4) + n;
	if (idx <= 63) {
		state->idx = 63 - Max(1, idx);
		state->mps = 0;
	} else {
		state->idx = Min(126, idx) - 64;
		state->mps = 1;
	}
}


/**
 * 9.3.1.2 Initialisation process for the arithmetic decoding engine
 */
int h264_bac_decode_init(struct h264_bac_dec *dec, struct h264_bitstream *bs)
{
	int res = 0;
	uint32_t v = 0;
	dec->bs = bs;
	dec->codIOffset = 0;
	CHECK(h264_bs_read_bits(bs, &v, 9));
	ULOG_ERRNO_RETURN_ERR_IF(v > 510, EIO);
	dec->codIRange = v;
out:
	return res;
}


/**
 * 9.3.4.1 Initialisation process for the arithmetic encoding engine
 */
int h264_bac_encode_init(struct h264_bac_enc *enc,
			 struct h264_bitstream *bs,
			 int first_slice)
{
	enc->bs = bs;
	enc->codILow = 0;
	enc->codIRange = 510;
	enc->firstBitFlag = 1;
	enc->bitsOutstanding = 0;
	if (first_slice)
		enc->BinCountsInNALunits = 0;
	return 0;
}


/**
 * 9.3.4.2 Encoding process for a binary decision
 * Figure 9-7 - Flowchart for encoding a decision
 */
int h264_bac_encode_bin(struct h264_bac_enc *enc,
			struct h264_bac_state *state,
			int bin)
{
	int res = 0;
	BAC_LOGV("%s state=(%u %u) bin=%u",
		 __func__,
		 state->idx,
		 state->mps,
		 bin);
	uint32_t qCodIRangeIdx = (enc->codIRange >> 6) & 3;
	uint32_t codIRangeLPS =
		s_h264_range_table_lps[state->idx][qCodIRangeIdx];
	enc->codIRange -= codIRangeLPS;

	bin = !!bin;
	if (bin == state->mps) {
		state->idx = s_h264_trans_table_mps[state->idx];
	} else {
		enc->codILow = enc->codILow + enc->codIRange;
		enc->codIRange = codIRangeLPS;
		if (state->idx == 0)
			state->mps = !state->mps;
		state->idx = s_h264_trans_table_lps[state->idx];
	}

	CHECK(h264_bac_encode_renorm(enc));
	enc->BinCountsInNALunits++;

out:
	return res;
}


/**
 * 9.3.4.4 Bypass encoding process for binary decisions
 * Figure 9-10 - Flowchart of encoding bypass
 */
int h264_bac_encode_bypass(struct h264_bac_enc *enc, int bin)
{
	int res = 0;
	BAC_LOGV("%s bin=%u", __func__, bin);

	enc->codILow <<= 1;
	if (bin)
		enc->codILow += enc->codIRange;

	if (enc->codILow >= 1024) {
		CHECK(h264_bac_encode_put_bit(enc, 1));
		enc->codILow -= 1024;
	} else if (enc->codILow >= 512) {
		enc->codILow -= 512;
		enc->bitsOutstanding++;
	} else {
		CHECK(h264_bac_encode_put_bit(enc, 0));
	}

	enc->BinCountsInNALunits++;

out:
	return res;
}


/**
 * 9.3.4.5 Encoding process for a binary decision before termination
 * Figure 9-11 - Flowchart of encoding a decision before termination
 */
int h264_bac_encode_terminate(struct h264_bac_enc *enc, int bin)
{
	int res = 0;
	BAC_LOGV("%s bin=%u", __func__, bin);

	enc->codIRange -= 2;

	if (bin) {
		enc->codILow += enc->codIRange;
		CHECK(h264_bac_encode_flush(enc));
	} else {
		CHECK(h264_bac_encode_renorm(enc));
	}

	enc->BinCountsInNALunits++;

out:
	return res;
}
