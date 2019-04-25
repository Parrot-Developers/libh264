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


struct h264_reader {
	struct h264_ctx_cbs cbs;
	void *userdata;
	int stop;
	struct h264_ctx *ctx;
	uint32_t flags;
};


#define H264_SYNTAX_OP_NAME read
#define H264_SYNTAX_OP_KIND H264_SYNTAX_OP_KIND_READ

#define H264_BITS(_f, _n) H264_READ_BITS(_f, _n)
#define H264_BITS_U(_f, _n) H264_READ_BITS_U(_f, _n)
#define H264_BITS_I(_f, _n) H264_READ_BITS_I(_f, _n)
#define H264_BITS_UE(_f) H264_READ_BITS_UE(_f)
#define H264_BITS_SE(_f) H264_READ_BITS_SE(_f)
#define H264_BITS_TE(_f, _m) H264_READ_BITS_TE(_f, _m)

#define H264_BITS_RBSP_TRAILING()                                              \
	do {                                                                   \
		int _res = h264_bs_read_rbsp_trailing_bits(bs);                \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#include "h264_syntax.h"


int h264_reader_new(const struct h264_ctx_cbs *cbs,
		    void *userdata,
		    struct h264_reader **ret_obj)
{
	int res = 0;
	struct h264_reader *reader = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);

	/* Allocate structure */
	reader = calloc(1, sizeof(*reader));
	if (reader == NULL)
		return -ENOMEM;

	/* Initialize structure */
	reader->cbs = *cbs;
	reader->userdata = userdata;
	res = h264_ctx_new(&reader->ctx);
	if (res < 0)
		goto error;

	/* Success */
	*ret_obj = reader;
	return 0;

	/* Cleanup in case of error */
error:
	h264_reader_destroy(reader);
	return res;
}


int h264_reader_destroy(struct h264_reader *reader)
{
	if (reader == NULL)
		return 0;
	if (reader->ctx != NULL)
		h264_ctx_destroy(reader->ctx);
	free(reader);
	return 0;
}


struct h264_ctx *h264_reader_get_ctx(struct h264_reader *reader)
{
	return reader == NULL ? NULL : reader->ctx;
}


int h264_reader_stop(struct h264_reader *reader)
{
	ULOG_ERRNO_RETURN_ERR_IF(reader == NULL, EINVAL);
	reader->stop = 1;
	return 0;
}


int h264_reader_parse(struct h264_reader *reader,
		      uint32_t flags,
		      const uint8_t *buf,
		      size_t len,
		      size_t *off)
{
	int res = 0;
	size_t start = 0, end = 0;

	ULOG_ERRNO_RETURN_ERR_IF(reader == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(off == NULL, EINVAL);

	reader->stop = 0;
	*off = 0;

	while (*off < len && !reader->stop) {
		res = h264_find_nalu(buf + *off, len - *off, &start, &end);
		if (res < 0 && res != -EAGAIN)
			break;
		h264_reader_parse_nalu(
			reader, flags, buf + *off + start, end - start);
		*off += end;
	}

	return 0;
}


int h264_reader_parse_nalu(struct h264_reader *reader,
			   uint32_t flags,
			   const uint8_t *buf,
			   size_t len)
{
	int res = 0;
	struct h264_bitstream bs;
	ULOG_ERRNO_RETURN_ERR_IF(reader == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	reader->stop = 0;
	reader->flags = flags;
	h264_bs_cinit(&bs, buf, len, 1);
	bs.priv = reader;
	res = _h264_read_nalu(&bs, reader->ctx, &reader->cbs, reader->userdata);
	h264_bs_clear(&bs);
	return res;
}


int h264_parse_nalu_header(const uint8_t *buf,
			   size_t len,
			   struct h264_nalu_header *nh)
{
	int res = 0;
	struct h264_bitstream bs;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(nh == NULL, EINVAL);
	memset(nh, 0, sizeof(*nh));

	h264_bs_cinit(&bs, buf, len, 1);
	res = _h264_read_nalu_header(&bs, nh);
	h264_bs_clear(&bs);

	return res;
}


int h264_parse_sps(const uint8_t *buf, size_t len, struct h264_sps *sps)
{
	int res = 0;
	struct h264_bitstream bs;
	struct h264_nalu_header nh;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	memset(sps, 0, sizeof(*sps));

	/* 7.4.2.1.1 Sequence parameter set data semantics */
	sps->chroma_format_idc = 1;

	/* Setup bitstream */
	h264_bs_cinit(&bs, buf, len, 1);

	/* Read NALU header, make sure it is a SPS */
	res = _h264_read_nalu_header(&bs, &nh);
	if (res < 0)
		goto out;
	if (nh.nal_unit_type != H264_NALU_TYPE_SPS) {
		res = -EIO;
		ULOGE("invalid nalu type: %u (%u)",
		      nh.nal_unit_type,
		      H264_NALU_TYPE_SPS);
		goto out;
	}

	/* Read SPS */
	res = _h264_read_sps(&bs, sps);

	/* Cleanup bitstream and exit */
out:
	h264_bs_clear(&bs);
	return res;
}


int h264_parse_pps(const uint8_t *buf,
		   size_t len,
		   const struct h264_sps *sps,
		   struct h264_pps *pps)
{
	int res = 0;
	struct h264_bitstream bs;
	struct h264_nalu_header nh;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sps == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pps == NULL, EINVAL);
	memset(pps, 0, sizeof(*pps));

	/* Setup bitstream */
	h264_bs_cinit(&bs, buf, len, 1);

	/* Read NALU header, make sure it is a PPS */
	res = _h264_read_nalu_header(&bs, &nh);
	if (res < 0)
		goto out;
	if (nh.nal_unit_type != H264_NALU_TYPE_PPS) {
		res = -EIO;
		ULOGE("invalid nalu type: %u (%u)",
		      nh.nal_unit_type,
		      H264_NALU_TYPE_PPS);
		goto out;
	}

	/* Read PPS */
	res = _h264_read_pps_with_sps(&bs, sps, pps);

	/* Cleanup bitstream and exit */
out:
	h264_bs_clear(&bs);
	return res;
}
