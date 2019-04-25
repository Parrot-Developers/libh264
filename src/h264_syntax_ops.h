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

#ifndef _H264_SYNTAX_OPS_
#define _H264_SYNTAX_OPS_


/* Available operation kinds */
#define H264_SYNTAX_OP_KIND_READ 0
#define H264_SYNTAX_OP_KIND_WRITE 1
#define H264_SYNTAX_OP_KIND_DUMP 2

#ifndef H264_SYNTAX_OP_NAME
#	error "H264_SYNTAX_OP_NAME shall be defined first"
#endif /* !H264_SYNTAX_OP_NAME */

#ifndef H264_SYNTAX_OP_KIND
#	error "H264_SYNTAX_OP_KIND shall be defined first"
#endif /* !H264_SYNTAX_OP_KIND */

#ifndef H264_BITS_U
#	error "H264_BITS_U shall be defined first"
#endif /* !H264_BITS_U */

#ifndef H264_BITS_I
#	error "H264_BITS_I shall be defined first"
#endif /* !H264_BITS_I */

#ifndef H264_BITS_UE
#	error "H264_BITS_UE shall be defined first"
#endif /* !H264_BITS_UE */

#ifndef H264_BITS_SE
#	error "H264_BITS_SE shall be defined first"
#endif /* !H264_BITS_SE */

#ifndef H264_BITS_RBSP_TRAILING
#	error "H264_BITS_RBSP_TRAILING shall be defined first"
#endif

#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_READ
#	define H264_SYNTAX_CONST
#else
#	define H264_SYNTAX_CONST const
#endif

#define _H264_CAT2(a, b) a##b
#define _H264_CAT3(a, b, c) a##b##c
#define _H264_CAT4(a, b, c, d) a##b##c##d

#define _H264_SYNTAX_FCT(_op, _name) _H264_CAT4(_h264_, _op, _, _name)

#define H264_SYNTAX_FCT(_name) _H264_SYNTAX_FCT(H264_SYNTAX_OP_NAME, _name)


#define _H264_READ_BITS(_name, _type, _field, ...)                             \
	do {                                                                   \
		_type _v = 0;                                                  \
		int _res = h264_bs_read_bits_##_name(bs, &_v, ##__VA_ARGS__);  \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
		(_field) = _v;                                                 \
	} while (0)

#define H264_READ_BITS(_f, _n) _H264_READ_BITS(u, uint32_t, _f, _n)
#define H264_READ_BITS_U(_f, _n) _H264_READ_BITS(u, uint32_t, _f, _n)
#define H264_READ_BITS_I(_f, _n) _H264_READ_BITS(i, int32_t, _f, _n)
#define H264_READ_BITS_UE(_f) _H264_READ_BITS(ue, uint32_t, _f)
#define H264_READ_BITS_SE(_f) _H264_READ_BITS(se, int32_t, _f)
#define H264_READ_BITS_TE(_f, _m) _H264_READ_BITS(te, uint32_t, _f, _m)

#define H264_READ_FLAGS() (((struct h264_reader *)(bs->priv))->flags)


#define _H264_WRITE_BITS(_name, _type, _field, ...)                            \
	do {                                                                   \
		_type _v = (_field);                                           \
		int _res = h264_bs_write_bits_##_name(bs, _v, ##__VA_ARGS__);  \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#define H264_WRITE_BITS(_f, _n) _H264_WRITE_BITS(u, uint32_t, _f, _n)
#define H264_WRITE_BITS_U(_f, _n) _H264_WRITE_BITS(u, uint32_t, _f, _n)
#define H264_WRITE_BITS_I(_f, _n) _H264_WRITE_BITS(i, int32_t, _f, _n)
#define H264_WRITE_BITS_UE(_f) _H264_WRITE_BITS(ue, uint32_t, _f)
#define H264_WRITE_BITS_SE(_f) _H264_WRITE_BITS(se, int32_t, _f)
#define H264_WRITE_BITS_TE(_f, _m) _H264_WRITE_BITS(te, uint32_t, _f, _m)


#define _H264_DUMP_CALL(_fct, ...)                                             \
	do {                                                                   \
		struct h264_dump *_dump = bs->priv;                            \
		int _res = (*_dump->cbs._fct)(_dump, ##__VA_ARGS__);           \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#define H264_DUMP_BITS(_f, _n) _H264_DUMP_CALL(field, #_f, _f)
#define H264_DUMP_BITS_U(_f, _n) _H264_DUMP_CALL(field, #_f, _f)
#define H264_DUMP_BITS_I(_f, _n) _H264_DUMP_CALL(field, #_f, _f)
#define H264_DUMP_BITS_UE(_f) _H264_DUMP_CALL(field, #_f, _f)
#define H264_DUMP_BITS_SE(_f) _H264_DUMP_CALL(field, #_f, _f)
#define H264_DUMP_BITS_TE(_f, _m) _H264_DUMP_CALL(field, #_f, _f)

#define H264_DUMP_FLAGS() (((struct h264_dump *)(bs->priv))->flags)


/* clang-format off */
#if H264_SYNTAX_OP_KIND == H264_SYNTAX_OP_KIND_DUMP
#  define H264_BEGIN_STRUCT(_name)   _H264_DUMP_CALL(begin_struct, #_name)
#  define H264_END_STRUCT(_name)     _H264_DUMP_CALL(end_struct, #_name)
#  define H264_BEGIN_ARRAY(_name)    _H264_DUMP_CALL(begin_array, #_name)
#  define H264_END_ARRAY(_name)      _H264_DUMP_CALL(end_array, #_name)
#  define H264_BEGIN_ARRAY_ITEM()    _H264_DUMP_CALL(begin_array_item)
#  define H264_END_ARRAY_ITEM()      _H264_DUMP_CALL(end_array_item)
#  define H264_FIELD(_name, _val)    _H264_DUMP_CALL(field, #_name, _val)
#  define H264_FIELD_S(_name, _val)  _H264_DUMP_CALL(field, _name, _val)
#else
#  define H264_BEGIN_STRUCT(_name)   do {} while (0)
#  define H264_END_STRUCT(_name)     do {} while (0)
#  define H264_BEGIN_ARRAY(_name)    do {} while (0)
#  define H264_END_ARRAY(_name)      do {} while (0)
#  define H264_BEGIN_ARRAY_ITEM()    do {} while (0)
#  define H264_END_ARRAY_ITEM()      do {} while (0)
#  define H264_FIELD(_name, _val)    do {} while (0)
#  define H264_FIELD_S(_name, _val)  do {} while (0)
#endif
/* clang-format on */


#define H264_CB(_ctx, _cbs, _userdata, _name, ...)                             \
	do {                                                                   \
		if ((_cbs) != NULL && (_cbs)->_name != NULL) {                 \
			(*(_cbs)->_name)((_ctx), ##__VA_ARGS__, (_userdata));  \
		}                                                              \
	} while (0)


#define H264_SEI(_name)                                                        \
	do {                                                                   \
		int _res = H264_SYNTAX_FCT(sei_##_name)(bs, ctx, &sei->_name); \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
		H264_CB(ctx,                                                   \
			cbs,                                                   \
			userdata,                                              \
			sei_##_name,                                           \
			sei->raw.buf,                                          \
			sei->raw.len,                                          \
			&sei->_name);                                          \
	} while (0)


#endif /* !_H264_SYNTAX_OPS_ */
