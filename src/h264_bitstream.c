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


static int h264_bs_ensure_capacity(struct h264_bitstream *bs, size_t capacity)
{
	uint8_t *newbuf = NULL;

	if (capacity <= bs->len)
		return 0;
	if (!bs->dynamic)
		return -EIO;

	/* Wanted capacity round up */
	capacity = (capacity + 255) & ~255;

	/* Allocate new buffer */
	newbuf = realloc(bs->data, capacity);
	if (newbuf == NULL)
		return -ENOMEM;

	/* Setup new buffer */
	bs->data = newbuf;
	bs->len = capacity;
	return 0;
}


static int h264_bs_flush(struct h264_bitstream *bs)
{
	int res = 0;

	if (bs->emulation_prevention && bs->off >= 2 &&
	    bs->data[bs->off - 2] == 0x00 && bs->data[bs->off - 1] == 0x00 &&
	    bs->cache <= 0x03) {
		/* Insert escape byte */
		res = h264_bs_ensure_capacity(bs, bs->off + 2);
		if (res < 0)
			return res;
		bs->data[bs->off] = 0x03;
		bs->data[bs->off + 1] = bs->cache;
		bs->cache = 0;
		bs->cachebits = 0;
		bs->off += 2;
		return 0;
	} else {
		res = h264_bs_ensure_capacity(bs, bs->off + 1);
		if (res < 0)
			return res;
		bs->data[bs->off] = bs->cache;
		bs->cache = 0;
		bs->cachebits = 0;
		bs->off++;
		return 0;
	}
}


/**
 * B.1 Byte stream NAL unit syntax and semantics
 */
static int
h264_find_start_code(const uint8_t *buf, size_t len, size_t *start, size_t *end)
{
	const uint8_t *p = buf;

	while (len >= 3) {
		/* Search for the next 0x00 byte */
		if (*p != 0x00)
			goto next;

		/* Is it a 00 00 00 01 sequence? */
		if (len >= 4 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01) {
			*start = p - buf;
			*end = *start + 4;
			return 0;
		}

		/* Is it a 00 00 01 sequence? */
		if (p[1] == 0x00 && p[2] == 0x01) {
			*start = p - buf;
			*end = *start + 3;
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


/**
 * B.1 Byte stream NAL unit syntax and semantics
 */
static int h264_find_end_code(const uint8_t *buf, size_t len, size_t *end)
{
	const uint8_t *p = buf;

	while (len >= 3) {
		/* Search next 0x00 byte */
		if (*p != 0x00)
			goto next;

		/* Is it a 00 00 00 sequence? */
		if (p[1] == 0x00 && p[2] == 0x00) {
			*end = p - buf;
			return 0;
		}

		/* Is it a 00 00 01 sequence? */
		if (p[1] == 0x00 && p[2] == 0x01) {
			*end = p - buf;
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


/**
 */
int h264_find_nalu(const uint8_t *buf, size_t len, size_t *start, size_t *end)
{
	int res = 0;
	size_t sc1 = 0, sc2 = 0, ec = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(start == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(end == NULL, EINVAL);

	/* Search for start code */
	res = h264_find_start_code(buf, len, &sc1, &sc2);
	if (res < 0)
		return res;
	*start = sc2;

	/* Search for end code */
	res = h264_find_end_code(buf + *start, len - *start, &ec);
	if (res < 0) {
		/* End of buffer reached before finding next start code */
		res = -EAGAIN;
		*end = len;
	} else {
		*end = *start + ec;
	}

	return res;
}


/**
 * 9.1 Parsing process for Exp-Golomb codes
 */
int h264_bs_read_bits_ue(struct h264_bitstream *bs, uint32_t *v)
{
	int leadingzeros = -1;
	uint32_t bit = 0;

	for (bit = 0; !bit; leadingzeros++) {
		if (h264_bs_read_bits(bs, &bit, 1) < 0)
			return -EIO;
	}

	bit = 0;
	if (leadingzeros) {
		if (h264_bs_read_bits(bs, &bit, leadingzeros) < 0)
			return -EIO;
	}

	*v = (1 << leadingzeros) - 1 + bit;
	return leadingzeros * 2 + 1;
}


int h264_bs_write_bits(struct h264_bitstream *bs, uint32_t v, uint32_t n)
{
	int res = 0;
	uint32_t bits = 0;
	uint32_t mask = 0;
	uint32_t part = 0;

	while (n > 0) {
		/* Write as many bits to current byte */
		bits = 8 - bs->cachebits;
		if (bits >= n)
			bits = n;
		mask = (1 << bits) - 1;
		part = v >> (n - bits) & mask;
		bs->cache |= part << (8 - bs->cachebits - bits);
		n -= bits;
		bs->cachebits += bits;
		res += bits;

		/* Flush data if needed */
		if (bs->cachebits == 8 && h264_bs_flush(bs) < 0)
			return -EIO;
	}

	return res;
}


/**
 * 9.1 Parsing process for Exp-Golomb codes
 */
int h264_bs_write_bits_ue(struct h264_bitstream *bs, uint32_t v)
{
	/* clang-format off */
	static const uint32_t table[256] = {
		1,
		1,
		2, 2,
		3, 3, 3, 3,
		4, 4, 4, 4, 4, 4, 4, 4,
		5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	};
	/* clang-format on */
	uint32_t n = 0;

	if (v == 0) {
		return h264_bs_write_bits(bs, 1, 1);
	} else {
		v++;
		if (v >= 0x01000000)
			n = 24 + table[v >> 24];
		else if (v >= 0x00010000)
			n = 16 + table[v >> 16];
		else if (v >= 0x00000100)
			n = 8 + table[v >> 8];
		else
			n = table[v];
		return h264_bs_write_bits(bs, v, 2 * n - 1);
	}
}


int h264_bs_read_bits_ff_coded(struct h264_bitstream *bs, uint32_t *v)
{
	int res = 0, n = 0;
	uint32_t bits = 0;
	*v = 0;
	do {
		res = h264_bs_read_bits(bs, &bits, 8);
		if (res < 0)
			return res;
		n += res;
		*v += bits;
	} while (bits == 0xff);
	return n;
}


int h264_bs_write_bits_ff_coded(struct h264_bitstream *bs, uint32_t v)
{
	int res = 0, n = 0;
	uint32_t bits = 0;
	do {
		bits = v > 0xff ? 0xff : v;
		res = h264_bs_write_bits(bs, bits, 8);
		if (res < 0)
			return res;
		n += res;
		v -= bits;
	} while (bits == 0xff);
	return n;
}


/**
 * 7.2 Specification of syntax functions, categories, and descriptors
 */
int h264_bs_more_rbsp_data(const struct h264_bitstream *bs)
{
	/* Use a temp stream */
	struct h264_bitstream bs2 = *bs;
	uint32_t bit = 0;

	/* Try to read rbsp_stop_one_bit,
	 * - if it fails, there is no more data
	 * - if it is not '1', there is more data */
	if (h264_bs_read_bits(&bs2, &bit, 1) < 0)
		return 0;
	if (bit != 1)
		return 1;

	while (!h264_bs_byte_aligned(&bs2)) {
		/* Try to read rbsp_alignment_zero_bit,
		 * - if it fails, there is no more data
		 * - if it is not '0', there is more data */
		if (h264_bs_read_bits(&bs2, &bit, 1) < 0)
			return 0;
		if (bit != 0)
			return 1;
	}

	/* Have we reached end of stream? */
	if (h264_bs_eos(&bs2))
		return 0;

	/* Do we have a trailing_zero_8bits? */
	return bs2.off + 1 < bs2.len || bs2.cdata[bs2.off] != 0x00;
}


/**
 * 7.3.2.11 RBSP trailing bits syntax
 */
int h264_bs_read_rbsp_trailing_bits(struct h264_bitstream *bs)
{
	int res = 0;
	uint32_t bit = 0;

	/* Read rbsp_stop_one_bit, shall be '1' */
	res = h264_bs_read_bits(bs, &bit, 1);
	if (res < 0)
		return res;
	if (bit != 1)
		return -EIO;

	while (!h264_bs_byte_aligned(bs)) {
		/* Read rbsp_alignment_zero_bit, shall be '0' */
		res = h264_bs_read_bits(bs, &bit, 1);
		if (res < 0)
			return res;
		if (bit != 0)
			return -EIO;
	}

	return 0;
}


/**
 * 7.3.2.11 RBSP trailing bits syntax
 */
int h264_bs_write_rbsp_trailing_bits(struct h264_bitstream *bs)
{
	int res = 0;

	/* Write rbsp_stop_one_bit */
	res = h264_bs_write_bits(bs, 1, 1);
	if (res < 0)
		return res;

	while (!h264_bs_byte_aligned(bs)) {
		/* Write rbsp_alignment_zero_bit */
		res = h264_bs_write_bits(bs, 0, 1);
		if (res < 0)
			return res;
	}

	return 0;
}


int h264_bs_read_raw_bytes(struct h264_bitstream *bs, uint8_t *buf, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(!h264_bs_byte_aligned(bs), EIO);
	ULOG_ERRNO_RETURN_ERR_IF(bs->len - bs->off != len, EIO);
	memcpy(buf, bs->cdata + bs->off, len);
	bs->off += len;
	return 0;
}


int h264_bs_write_raw_bytes(struct h264_bitstream *bs,
			    const uint8_t *buf,
			    size_t len)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(!h264_bs_byte_aligned(bs), EIO);
	res = h264_bs_ensure_capacity(bs, bs->off + len);
	if (res < 0)
		return res;
	memcpy(bs->data + bs->off, buf, len);
	bs->off += len;
	return 0;
}


int h264_bs_acquire_buf(struct h264_bitstream *bs, uint8_t **buf, size_t *len)
{
	ULOG_ERRNO_RETURN_ERR_IF(!h264_bs_byte_aligned(bs), EIO);
	ULOG_ERRNO_RETURN_ERR_IF(!bs->dynamic, EIO);
	*buf = bs->data;
	*len = bs->off;
	bs->dynamic = 0;
	return 0;
}
