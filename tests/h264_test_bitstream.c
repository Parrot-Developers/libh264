/**
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "h264_test.h"


/* h264_bs_write_bits_u()/h264_bs_read_bits_u() round trip across a table of
 * bit widths chained in a single dynamic bitstream, mirroring
 * packages/libaac/tests/aac_test_bitstream.c's pattern. */
static void test_bitstream_read_write_bits_u_roundtrip(void)
{
	static const uint32_t widths[] = {1, 3, 7, 8, 9, 16, 31, 32};
	static const uint32_t values[] = {
		0x1, 0x5, 0x7f, 0xab, 0x1ff, 0xcafe, 0x7fffffff, 0xffffffff};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	for (size_t i = 0; i < ARRAY_SIZE(widths); i++) {
		res = h264_bs_write_bits_u(&bs, values[i], widths[i]);
		CU_ASSERT_TRUE_FATAL(res >= 0);
	}
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	for (size_t i = 0; i < ARRAY_SIZE(widths); i++) {
		uint32_t v = 0;
		res = h264_bs_read_bits_u(&bs, &v, widths[i]);
		CU_ASSERT_TRUE(res >= 0);
		CU_ASSERT_EQUAL(v,
				values[i] & (widths[i] == 32
						     ? 0xffffffffU
						     : (1U << widths[i]) - 1));
	}
	res = h264_bs_read_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL(res, 0);
	h264_bs_clear(&bs);
	free(buf);
}


/* Sign extension round trip for h264_bs_write_bits_i()/read_bits_i(). */
static void test_bitstream_read_write_bits_i_roundtrip(void)
{
	static const int32_t values[] = {0, 1, -1, 63, -64, 100, -128};
	static const uint32_t width = 8;
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		res = h264_bs_write_bits_i(&bs, values[i], width);
		CU_ASSERT_TRUE_FATAL(res >= 0);
	}
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		int32_t v = 0;
		res = h264_bs_read_bits_i(&bs, &v, width);
		CU_ASSERT_TRUE(res >= 0);
		CU_ASSERT_EQUAL(v, values[i]);
	}
	h264_bs_clear(&bs);
	free(buf);
}


/* Exp-Golomb unsigned (ue) round trip, 9.1. */
static void test_bitstream_read_write_bits_ue_roundtrip(void)
{
	static const uint32_t values[] = {0,
					  1,
					  2,
					  3,
					  4,
					  7,
					  8,
					  15,
					  16,
					  31,
					  32,
					  100,
					  255,
					  256,
					  1000,
					  65535};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		res = h264_bs_write_bits_ue(&bs, values[i]);
		CU_ASSERT_TRUE_FATAL(res >= 0);
	}
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		uint32_t v = 0;
		res = h264_bs_read_bits_ue(&bs, &v);
		CU_ASSERT_TRUE(res >= 0);
		CU_ASSERT_EQUAL(v, values[i]);
	}
	h264_bs_clear(&bs);
	free(buf);
}


/* Exp-Golomb signed (se) round trip, 9.1: v<=0 encodes as -2v (even code),
 * v>0 encodes as 2v-1 (odd code). */
static void test_bitstream_read_write_bits_se_roundtrip(void)
{
	static const int32_t values[] = {0, 1, -1, 2, -2, 5, -5, 1000, -1000};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		res = h264_bs_write_bits_se(&bs, values[i]);
		CU_ASSERT_TRUE_FATAL(res >= 0);
	}
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		int32_t v = 0;
		res = h264_bs_read_bits_se(&bs, &v);
		CU_ASSERT_TRUE(res >= 0);
		CU_ASSERT_EQUAL(v, values[i]);
	}
	h264_bs_clear(&bs);
	free(buf);
}


/* Truncated Exp-Golomb (te), 9.1: m==1 is a single inverted bit, m>1 falls
 * back to plain ue. */
static void test_bitstream_read_write_bits_te_roundtrip(void)
{
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	uint32_t v = 0;
	int res;

	/* m == 1: value is written as the inverted bit */
	h264_bs_init(&bs, NULL, 0, 0);
	res = h264_bs_write_bits_te(&bs, 0, 1);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	res = h264_bs_write_bits_te(&bs, 1, 1);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	res = h264_bs_read_bits_te(&bs, &v, 1);
	CU_ASSERT_TRUE(res >= 0);
	CU_ASSERT_EQUAL(v, 0);
	res = h264_bs_read_bits_te(&bs, &v, 1);
	CU_ASSERT_TRUE(res >= 0);
	CU_ASSERT_EQUAL(v, 1);
	h264_bs_clear(&bs);
	free(buf);

	/* m > 1: falls back to ue(v) */
	h264_bs_init(&bs, NULL, 0, 0);
	res = h264_bs_write_bits_te(&bs, 3, 5);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	res = h264_bs_write_rbsp_trailing_bits(&bs);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);

	h264_bs_cinit(&bs, buf, len, 0);
	res = h264_bs_read_bits_te(&bs, &v, 5);
	CU_ASSERT_TRUE(res >= 0);
	CU_ASSERT_EQUAL(v, 3);
	h264_bs_clear(&bs);
	free(buf);
}


/* h264_bs_write_bits_ff_coded()/read_bits_ff_coded() round trip: value is
 * coded as a run of 0xff bytes followed by a non-0xff remainder byte. */
static void test_bitstream_read_write_bits_ff_coded_roundtrip(void)
{
	static const uint32_t values[] = {0, 1, 254, 255, 256, 509, 510, 765};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	int res;

	for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
		uint32_t v = 0;

		h264_bs_init(&bs, NULL, 0, 0);
		res = h264_bs_write_bits_ff_coded(&bs, values[i]);
		CU_ASSERT_TRUE_FATAL(res >= 0);
		CU_ASSERT_EQUAL(res % 8, 0);
		res = h264_bs_acquire_buf(&bs, &buf, &len);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		h264_bs_clear(&bs);

		/* do { write min(v,0xff); v -= that } while (last byte was
		 * 0xff) always emits floor(v/255)+1 bytes, since the loop
		 * only keeps going after a full 0xff byte and integer
		 * division already accounts for the extra terminator byte
		 * that v == 255 (and its multiples) needs. */
		CU_ASSERT_EQUAL(len, values[i] / 255 + 1);

		h264_bs_cinit(&bs, buf, len, 0);
		res = h264_bs_read_bits_ff_coded(&bs, &v);
		CU_ASSERT_TRUE(res >= 0);
		CU_ASSERT_EQUAL(v, values[i]);
		h264_bs_clear(&bs);
		free(buf);
	}
}


/* h264_bs_more_rbsp_data(): the bit right after the current position decides
 * whether there is more data, hand-verified against 7.2's definition. */
static void test_bitstream_more_rbsp_data(void)
{
	struct h264_bitstream bs;

	/* 0b0100_0000: first bit is 0 (not the rbsp_stop_one_bit) -> more
	 * data is present without even needing to look further. */
	{
		uint8_t buf[] = {0x40};
		h264_bs_cinit(&bs, buf, sizeof(buf), 0);
		CU_ASSERT_EQUAL(h264_bs_more_rbsp_data(&bs), 1);
	}

	/* 0b1000_0000: stop bit followed by all-zero alignment padding and
	 * end of stream -> no more data. */
	{
		uint8_t buf[] = {0x80};
		h264_bs_cinit(&bs, buf, sizeof(buf), 0);
		CU_ASSERT_EQUAL(h264_bs_more_rbsp_data(&bs), 0);
	}

	/* 0b1000_0001: stop bit, then alignment padding whose last bit is a
	 * 1 (not the expected 0) -> that is real data, not padding. */
	{
		uint8_t buf[] = {0x81};
		h264_bs_cinit(&bs, buf, sizeof(buf), 0);
		CU_ASSERT_EQUAL(h264_bs_more_rbsp_data(&bs), 1);
	}
}


/* h264_bs_next_bits(): peeks without advancing the read position. */
static void test_bitstream_next_bits(void)
{
	uint8_t buf[] = {0xab, 0xcd};
	struct h264_bitstream bs;
	uint32_t peeked = 0;
	uint32_t read = 0;
	int res;

	h264_bs_cinit(&bs, buf, sizeof(buf), 0);
	res = h264_bs_next_bits(&bs, &peeked, 8);
	CU_ASSERT_TRUE(res >= 0);
	CU_ASSERT_EQUAL(peeked, 0xab);
	CU_ASSERT_EQUAL(bs.off, 0);
	CU_ASSERT_EQUAL(bs.cachebits, 0);

	res = h264_bs_read_bits_u(&bs, &read, 8);
	CU_ASSERT_TRUE(res >= 0);
	CU_ASSERT_EQUAL(read, peeked);
}


/* h264_bs_eos()/h264_bs_byte_aligned()/h264_bs_rem_raw_bits() sanity. */
static void test_bitstream_eos_and_alignment(void)
{
	uint8_t buf[] = {0xff, 0xff};
	struct h264_bitstream bs;
	uint32_t v = 0;

	h264_bs_cinit(&bs, buf, sizeof(buf), 0);
	CU_ASSERT_FALSE(h264_bs_eos(&bs));
	CU_ASSERT_TRUE(h264_bs_byte_aligned(&bs));
	CU_ASSERT_EQUAL(h264_bs_rem_raw_bits(&bs), 16);

	CU_ASSERT_TRUE(h264_bs_read_bits_u(&bs, &v, 4) >= 0);
	CU_ASSERT_FALSE(h264_bs_byte_aligned(&bs));
	CU_ASSERT_EQUAL(h264_bs_rem_raw_bits(&bs), 12);

	CU_ASSERT_TRUE(h264_bs_read_bits_u(&bs, &v, 12) >= 0);
	CU_ASSERT_TRUE(h264_bs_eos(&bs));
	CU_ASSERT_EQUAL(h264_bs_rem_raw_bits(&bs), 0);
}


/* h264_bs_write_bits(): n > 64 is rejected. */
static void test_bitstream_write_bits_n_too_large(void)
{
	struct h264_bitstream bs;
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	res = h264_bs_write_bits(&bs, 0, 65);
	CU_ASSERT_EQUAL(res, -EINVAL);
	h264_bs_clear(&bs);
}


/* h264_bs_read_raw_bytes()/write_raw_bytes(): byte-aligned raw copy, plus
 * the two -EIO guard paths (misalignment, and a length mismatch on read). */
static void test_bitstream_raw_bytes(void)
{
	uint8_t payload[] = {0x11, 0x22, 0x33, 0x44};
	struct h264_bitstream bs;
	uint8_t *buf = NULL;
	size_t len = 0;
	uint8_t out[sizeof(payload)] = {0};
	int res;

	h264_bs_init(&bs, NULL, 0, 0);
	res = h264_bs_write_raw_bytes(&bs, payload, sizeof(payload));
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	h264_bs_clear(&bs);
	CU_ASSERT_EQUAL(len, sizeof(payload));
	CU_ASSERT_EQUAL(memcmp(buf, payload, sizeof(payload)), 0);

	h264_bs_cinit(&bs, buf, len, 0);
	res = h264_bs_read_raw_bytes(&bs, out, sizeof(out));
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(out, payload, sizeof(payload)), 0);

	/* Length mismatch: bs is now fully consumed (len - off == 0) */
	res = h264_bs_read_raw_bytes(&bs, out, 1);
	CU_ASSERT_EQUAL(res, -EIO);
	h264_bs_clear(&bs);
	free(buf);

	/* Misaligned write */
	h264_bs_init(&bs, NULL, 0, 0);
	res = h264_bs_write_bits_u(&bs, 1, 1);
	CU_ASSERT_TRUE_FATAL(res >= 0);
	res = h264_bs_write_raw_bytes(&bs, payload, sizeof(payload));
	CU_ASSERT_EQUAL(res, -EIO);
	h264_bs_clear(&bs);
}


/* h264_bs_acquire_buf(): rejects a non-byte-aligned or non-dynamic
 * bitstream. */
static void test_bitstream_acquire_buf_guards(void)
{
	struct h264_bitstream bs;
	uint8_t staticbuf[4] = {0};
	uint8_t *buf = NULL;
	size_t len = 0;
	uint32_t v = 0;
	int res;

	/* Not byte-aligned */
	h264_bs_init(&bs, NULL, 0, 0);
	CU_ASSERT_TRUE_FATAL(h264_bs_write_bits_u(&bs, 1, 1) >= 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL(res, -EIO);
	h264_bs_clear(&bs);

	/* Not dynamic (static buffer passed to h264_bs_init) */
	h264_bs_init(&bs, staticbuf, sizeof(staticbuf), 0);
	CU_ASSERT_TRUE_FATAL(h264_bs_read_bits_u(&bs, &v, 8) >= 0);
	res = h264_bs_acquire_buf(&bs, &buf, &len);
	CU_ASSERT_EQUAL(res, -EIO);
	h264_bs_clear(&bs);
}


/* h264_find_nalu(): complete NALU delimited by a following 4-byte start
 * code. Byte offsets below were hand-traced against h264_find_start_code()/
 * h264_find_end_code() in src/h264_bitstream.c. */
static void test_find_nalu_complete_4byte_start_code(void)
{
	uint8_t buf[] = {0x00,
			 0x00,
			 0x00,
			 0x01,
			 0xaa,
			 0xbb,
			 0x00,
			 0x00,
			 0x00,
			 0x01,
			 0xcc};
	size_t start = 0;
	size_t end = 0;
	int res;

	res = h264_find_nalu(buf, sizeof(buf), &start, &end);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(start, 4);
	CU_ASSERT_EQUAL(end, 6);
}


/* Same, but the first start code is the 3-byte variant. */
static void test_find_nalu_complete_3byte_start_code(void)
{
	uint8_t buf[] = {0x00, 0x00, 0x01, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0xcc};
	size_t start = 0;
	size_t end = 0;
	int res;

	res = h264_find_nalu(buf, sizeof(buf), &start, &end);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(start, 3);
	CU_ASSERT_EQUAL(end, 5);
}


/* Truncated stream (no end code found): -EAGAIN, *end == len. */
static void test_find_nalu_truncated(void)
{
	uint8_t buf[] = {0x00, 0x00, 0x00, 0x01, 0xaa, 0xbb, 0xcc};
	size_t start = 0;
	size_t end = 0;
	int res;

	res = h264_find_nalu(buf, sizeof(buf), &start, &end);
	CU_ASSERT_EQUAL(res, -EAGAIN);
	CU_ASSERT_EQUAL(start, 4);
	CU_ASSERT_EQUAL(end, sizeof(buf));
}


/* No start code at all in the buffer. */
static void test_find_nalu_no_start_code(void)
{
	uint8_t buf[] = {0x01, 0x02, 0x03, 0x04, 0x05};
	size_t start = 0;
	size_t end = 0;
	int res;

	res = h264_find_nalu(buf, sizeof(buf), &start, &end);
	CU_ASSERT_EQUAL(res, -ENOENT);
}


static void test_find_nalu_null_args(void)
{
	size_t start = 0;
	size_t end = 0;
	uint8_t buf[4] = {0};

	CU_ASSERT_EQUAL(h264_find_nalu(NULL, 4, &start, &end), -EINVAL);
	CU_ASSERT_EQUAL(h264_find_nalu(buf, 4, NULL, &end), -EINVAL);
	CU_ASSERT_EQUAL(h264_find_nalu(buf, 4, &start, NULL), -EINVAL);
}


/* h264_get_start_code_length(). */
static void test_get_start_code_length(void)
{
	uint8_t sc4[] = {0x00, 0x00, 0x00, 0x01};
	uint8_t sc3[] = {0x00, 0x00, 0x01};
	/* 4 bytes available but last byte isn't 0x01: falls back to the
	 * 3-byte check on the first 3 bytes. */
	uint8_t sc3_with_extra[] = {0x00, 0x00, 0x01, 0xaa};
	uint8_t none[] = {0x01, 0x02, 0x03};

	CU_ASSERT_EQUAL(h264_get_start_code_length(sc4, sizeof(sc4)), 4);
	CU_ASSERT_EQUAL(h264_get_start_code_length(sc3, sizeof(sc3)), 3);
	CU_ASSERT_EQUAL(h264_get_start_code_length(sc3_with_extra,
						   sizeof(sc3_with_extra)),
			3);
	CU_ASSERT_EQUAL(h264_get_start_code_length(none, sizeof(none)),
			-ENOENT);
	CU_ASSERT_EQUAL(h264_get_start_code_length(NULL, 4), -EINVAL);
	CU_ASSERT_EQUAL(h264_get_start_code_length(sc3, 2), -EINVAL);
}


/* h264_byte_stream_to_avcc()/h264_avcc_to_byte_stream() round trip over a
 * 2-NALU Annex-B buffer (both start codes are 4 bytes, so byte-stream and
 * AVCC forms have identical total length). */
static void test_byte_stream_avcc_roundtrip(void)
{
	uint8_t orig[] = {0x00,
			  0x00,
			  0x00,
			  0x01,
			  0xaa,
			  0xbb,
			  0xcc,
			  0x00,
			  0x00,
			  0x00,
			  0x01,
			  0xdd,
			  0xee};
	uint8_t work[sizeof(orig)];
	int res;

	memcpy(work, orig, sizeof(orig));
	res = h264_byte_stream_to_avcc(work, sizeof(work));
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* [00 00 00 03] aa bb cc [00 00 00 02] dd ee */
	CU_ASSERT_EQUAL(memcmp(work, "\x00\x00\x00\x03\xaa\xbb\xcc", 7), 0);
	CU_ASSERT_EQUAL(memcmp(work + 7, "\x00\x00\x00\x02\xdd\xee", 6), 0);

	res = h264_avcc_to_byte_stream(work, sizeof(work));
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(work, orig, sizeof(orig)), 0);
}


static void test_byte_stream_to_avcc_no_start_code(void)
{
	uint8_t buf[] = {0x01, 0x02, 0x03, 0x04};
	uint8_t orig[sizeof(buf)];
	int res;

	memcpy(orig, buf, sizeof(buf));
	res = h264_byte_stream_to_avcc(buf, sizeof(buf));
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(memcmp(buf, orig, sizeof(buf)), 0);
}


static void test_avcc_to_byte_stream_invalid_nalu_len(void)
{
	uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0xaa, 0xbb};
	int res;

	res = h264_avcc_to_byte_stream(buf, sizeof(buf));
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_byte_stream_avcc_null_args(void)
{
	uint8_t buf[4] = {0};

	CU_ASSERT_EQUAL(h264_byte_stream_to_avcc(NULL, 4), -EINVAL);
	CU_ASSERT_EQUAL(h264_byte_stream_to_avcc(buf, 0), -EINVAL);
	CU_ASSERT_EQUAL(h264_avcc_to_byte_stream(NULL, 4), -EINVAL);
	CU_ASSERT_EQUAL(h264_avcc_to_byte_stream(buf, 0), -EINVAL);
}


CU_TestInfo g_h264_test_bitstream[] = {
	{FN("read-write-bits-u-roundtrip"),
	 &test_bitstream_read_write_bits_u_roundtrip},
	{FN("read-write-bits-i-roundtrip"),
	 &test_bitstream_read_write_bits_i_roundtrip},
	{FN("read-write-bits-ue-roundtrip"),
	 &test_bitstream_read_write_bits_ue_roundtrip},
	{FN("read-write-bits-se-roundtrip"),
	 &test_bitstream_read_write_bits_se_roundtrip},
	{FN("read-write-bits-te-roundtrip"),
	 &test_bitstream_read_write_bits_te_roundtrip},
	{FN("read-write-bits-ff-coded-roundtrip"),
	 &test_bitstream_read_write_bits_ff_coded_roundtrip},
	{FN("more-rbsp-data"), &test_bitstream_more_rbsp_data},
	{FN("next-bits"), &test_bitstream_next_bits},
	{FN("eos-and-alignment"), &test_bitstream_eos_and_alignment},
	{FN("write-bits-n-too-large"), &test_bitstream_write_bits_n_too_large},
	{FN("raw-bytes"), &test_bitstream_raw_bytes},
	{FN("acquire-buf-guards"), &test_bitstream_acquire_buf_guards},
	{FN("find-nalu-complete-4byte-start-code"),
	 &test_find_nalu_complete_4byte_start_code},
	{FN("find-nalu-complete-3byte-start-code"),
	 &test_find_nalu_complete_3byte_start_code},
	{FN("find-nalu-truncated"), &test_find_nalu_truncated},
	{FN("find-nalu-no-start-code"), &test_find_nalu_no_start_code},
	{FN("find-nalu-null-args"), &test_find_nalu_null_args},
	{FN("get-start-code-length"), &test_get_start_code_length},
	{FN("byte-stream-avcc-roundtrip"), &test_byte_stream_avcc_roundtrip},
	{FN("byte-stream-to-avcc-no-start-code"),
	 &test_byte_stream_to_avcc_no_start_code},
	{FN("avcc-to-byte-stream-invalid-nalu-len"),
	 &test_avcc_to_byte_stream_invalid_nalu_len},
	{FN("byte-stream-avcc-null-args"), &test_byte_stream_avcc_null_args},

	CU_TEST_INFO_NULL,
};
