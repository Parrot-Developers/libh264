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

#ifndef _H264_BITSTREAM_H_
#define _H264_BITSTREAM_H_


struct h264_bitstream {
	union {
		/* Data pointer (const) */
		const uint8_t *cdata;

		/* Data pointer */
		uint8_t *data;
	};

	/* Data length */
	size_t len;

	/* Offset in data */
	size_t off;

	/* Partial read/write byte */
	uint8_t cache;

	/* Number of bits in cache */
	uint8_t cachebits;

	/* Enable emulation prevention */
	int emulation_prevention;

	/* Dynamic */
	int dynamic;

	/* Private data */
	void *priv;
};


H264_API
int h264_find_nalu(const uint8_t *buf, size_t len, size_t *start, size_t *end);


H264_API
int h264_bs_write_bits(struct h264_bitstream *bs, uint32_t v, uint32_t n);


H264_API
int h264_bs_read_bits_ue(struct h264_bitstream *bs, uint32_t *v);


H264_API
int h264_bs_write_bits_ue(struct h264_bitstream *bs, uint32_t v);


H264_API
int h264_bs_read_bits_ff_coded(struct h264_bitstream *bs, uint32_t *v);


H264_API
int h264_bs_write_bits_ff_coded(struct h264_bitstream *bs, uint32_t v);


H264_API
int h264_bs_more_rbsp_data(const struct h264_bitstream *bs);


H264_API int
h264_bs_next_bits(const struct h264_bitstream *bs, uint32_t *v, uint32_t n);


H264_API
int h264_bs_read_rbsp_trailing_bits(struct h264_bitstream *bs);


H264_API
int h264_bs_write_rbsp_trailing_bits(struct h264_bitstream *bs);


H264_API
int h264_bs_read_raw_bytes(struct h264_bitstream *bs, uint8_t *buf, size_t len);


H264_API
int h264_bs_write_raw_bytes(struct h264_bitstream *bs,
			    const uint8_t *buf,
			    size_t len);


H264_API
int h264_bs_acquire_buf(struct h264_bitstream *bs, uint8_t **buf, size_t *len);


static inline void h264_bs_cinit(struct h264_bitstream *bs,
				 const uint8_t *buf,
				 size_t len,
				 int emulation_prevention)
{
	memset(bs, 0, sizeof(*bs));
	bs->cdata = buf;
	bs->len = len;
	bs->emulation_prevention = emulation_prevention;
}


static inline void h264_bs_init(struct h264_bitstream *bs,
				uint8_t *buf,
				size_t len,
				int emulation_prevention)
{
	memset(bs, 0, sizeof(*bs));
	bs->data = buf;
	bs->len = len;
	bs->dynamic = (buf == NULL && len == 0);
	bs->emulation_prevention = emulation_prevention;
}


static inline void h264_bs_clear(struct h264_bitstream *bs)
{
	if (bs->dynamic)
		free(bs->data);
	memset(bs, 0, sizeof(*bs));
}


static inline int h264_bs_byte_aligned(const struct h264_bitstream *bs)
{
	return bs->cachebits % 8 == 0;
}


static inline int h264_bs_eos(const struct h264_bitstream *bs)
{
	return bs->off >= bs->len && bs->cachebits == 0;
}


static inline size_t h264_bs_rem_raw_bits(const struct h264_bitstream *bs)
{
	return (bs->len - bs->off) * 8 + bs->cachebits;
}


static inline int h264_bs_fetch(struct h264_bitstream *bs)
{
	/* Detect 0x00 0x00 0x03 sequence in the stream */
	if (bs->emulation_prevention && bs->off >= 2 &&
	    bs->cdata[bs->off - 2] == 0x00 && bs->cdata[bs->off - 1] == 0x00 &&
	    bs->cdata[bs->off] == 0x03) {
		if (bs->off + 1 >= bs->len)
			return -EIO;
		/* Skip escape byte */
		bs->cache = bs->cdata[bs->off + 1];
		bs->cachebits = 8;
		bs->off += 2;
		return 0;
	} else if (bs->off < bs->len) {
		bs->cache = bs->cdata[bs->off];
		bs->cachebits = 8;
		bs->off++;
		return 0;
	} else {
		/* End of stream reached */
		return -EIO;
	}
}


static inline int
h264_bs_read_bits(struct h264_bitstream *bs, uint32_t *v, uint32_t n)
{
	int res = 0;
	uint32_t bits = 0;
	uint32_t mask = 0;
	uint32_t part = 0;

	*v = 0;
	while (n > 0) {
		/* Fetch data if needed */
		if (bs->cachebits == 0 && h264_bs_fetch(bs) < 0)
			return -EIO;

		/* Read as many bits from cache */
		bits = n < bs->cachebits ? n : bs->cachebits;
		mask = (1 << bits) - 1;
		part = (bs->cache >> (bs->cachebits - bits)) & mask;
		*v = (*v << bits) | part;
		n -= bits;
		bs->cachebits -= bits;
		res += bits;
	}

	return res;
}


static inline int
h264_bs_read_bits_u(struct h264_bitstream *bs, uint32_t *v, uint32_t n)
{
	return h264_bs_read_bits(bs, v, n);
}


static inline int
h264_bs_write_bits_u(struct h264_bitstream *bs, uint32_t v, uint32_t n)
{
	return h264_bs_write_bits(bs, v, n);
}


static inline int
h264_bs_read_bits_i(struct h264_bitstream *bs, int32_t *v, uint32_t n)
{
	int res = 0;
	uint32_t u32 = 0;

	res = h264_bs_read_bits(bs, &u32, n);
	if (res >= 0) {
		/* Sign extend result */
		if ((u32 & (1 << (n - 1))) != 0)
			*v = (int32_t)(u32 | ((uint32_t)-1) << n);
		else
			*v = (int32_t)u32;
	}

	return res;
}


static inline int
h264_bs_write_bits_i(struct h264_bitstream *bs, int32_t v, uint32_t n)
{
	return h264_bs_write_bits_u(bs, ((uint32_t)v) & ((1 << n) - 1), n);
}


/**
 * 9.1 Parsing process for Exp-Golomb codes
 */
static inline int h264_bs_read_bits_se(struct h264_bitstream *bs, int32_t *v)
{
	int res = 0;
	uint32_t u32 = 0;

	res = h264_bs_read_bits_ue(bs, &u32);
	if (res >= 0) {
		*v = (u32 & 1) ? (((int32_t)u32 + 1) / 2)
			       : (-((int32_t)u32 + 1) / 2);
	}
	return res;
}


/**
 * 9.1 Parsing process for Exp-Golomb codes
 */
static inline int h264_bs_write_bits_se(struct h264_bitstream *bs, int32_t v)
{
	if (v <= 0)
		return h264_bs_write_bits_ue(bs, (uint32_t)(-2 * v));
	else
		return h264_bs_write_bits_ue(bs, (uint32_t)(2 * v - 1));
}


/**
 * 9.1 Parsing process for truncated Exp-Golomb codes
 */
static inline int
h264_bs_read_bits_te(struct h264_bitstream *bs, uint32_t *v, uint32_t m)
{
	int res = 0;
	if (m == 1) {
		res = h264_bs_read_bits(bs, v, 1);
		*v = !*v;
	} else {
		res = h264_bs_read_bits_ue(bs, v);
	}
	return res;
}


/**
 * 9.1 Parsing process for truncated Exp-Golomb codes
 */
static inline int
h264_bs_write_bits_te(struct h264_bitstream *bs, uint32_t v, uint32_t m)
{
	if (m == 1)
		return h264_bs_write_bits(bs, !v, 1);
	else
		return h264_bs_write_bits_ue(bs, v);
}


#endif /* !_H264_BITSTREAM_H_ */
