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

#include <assert.h>
#include <json-c/json.h>

#define H264_DUMP_MAX_STACK_SIZE 16


struct h264_dump {
	struct h264_dump_cfg cfg;
	uint32_t flags;
	struct {
		int (*begin_struct)(struct h264_dump *dump, const char *name);
		int (*end_struct)(struct h264_dump *dump, const char *name);
		int (*begin_array)(struct h264_dump *dump, const char *name);
		int (*end_array)(struct h264_dump *dump, const char *name);
		int (*begin_array_item)(struct h264_dump *dump);
		int (*end_array_item)(struct h264_dump *dump);
		int (*field)(struct h264_dump *dump,
			     const char *field,
			     int64_t val);
	} cbs;

	json_object *jcurrent;
	json_object *jstack[H264_DUMP_MAX_STACK_SIZE];
	uint32_t jstacksize;
};


#define H264_SYNTAX_OP_NAME dump
#define H264_SYNTAX_OP_KIND H264_SYNTAX_OP_KIND_DUMP

#define H264_BITS(_f, _n) H264_DUMP_BITS(_f, _n)
#define H264_BITS_U(_f, _n) H264_DUMP_BITS_U(_f, _n)
#define H264_BITS_I(_f, _n) H264_DUMP_BITS_I(_f, _n)
#define H264_BITS_UE(_f) H264_DUMP_BITS_UE(_f)
#define H264_BITS_SE(_f) H264_DUMP_BITS_SE(_f)
#define H264_BITS_TE(_f, _m) H264_DUMP_BITS_TE(_f, _m)

/* clang-format off */
#define H264_BITS_RBSP_TRAILING() do {} while (0)
/* clang-format on */

#include "h264_syntax.h"


static int h264_dump_json_push(struct h264_dump *dump, json_object *jobj)
{
	assert(dump->jstacksize < H264_DUMP_MAX_STACK_SIZE);
	dump->jstack[dump->jstacksize] = jobj;
	dump->jstacksize++;
	dump->jcurrent = jobj;
	return 0;
}


static int h264_dump_json_pop(struct h264_dump *dump, json_object **jobj)
{
	assert(dump->jstacksize > 0);
	dump->jstacksize--;
	assert(dump->jstack[dump->jstacksize] != NULL);
	assert(dump->jstack[dump->jstacksize] == dump->jcurrent);
	*jobj = dump->jstack[dump->jstacksize];
	if (dump->jstacksize == 0)
		dump->jcurrent = NULL;
	else
		dump->jcurrent = dump->jstack[dump->jstacksize - 1];
	return 0;
}


static int h264_dump_json_begin_struct(struct h264_dump *dump, const char *name)
{
	json_object *jobj = json_object_new_object();
	h264_dump_json_push(dump, jobj);
	return 0;
}


static int h264_dump_json_end_struct(struct h264_dump *dump, const char *name)
{
	json_object *jobj = NULL;
	h264_dump_json_pop(dump, &jobj);
	if (json_object_get_type(dump->jcurrent) == json_type_object)
		json_object_object_add(dump->jcurrent, name, jobj);
	return 0;
}


static int h264_dump_json_begin_array(struct h264_dump *dump, const char *name)
{
	json_object *jobj = json_object_new_array();
	h264_dump_json_push(dump, jobj);
	return 0;
}


static int h264_dump_json_end_array(struct h264_dump *dump, const char *name)
{
	json_object *jobj = NULL;
	h264_dump_json_pop(dump, &jobj);
	if (json_object_get_type(dump->jcurrent) == json_type_object)
		json_object_object_add(dump->jcurrent, name, jobj);
	else if (json_object_get_type(dump->jcurrent) == json_type_array)
		json_object_array_add(dump->jcurrent, jobj);
	return 0;
}


static int h264_dump_json_begin_array_item(struct h264_dump *dump)
{
	json_object *jobj = json_object_new_object();
	h264_dump_json_push(dump, jobj);
	return 0;
}


static int h264_dump_json_end_array_item(struct h264_dump *dump)
{
	json_object *jobj = NULL;
	h264_dump_json_pop(dump, &jobj);
	if (json_object_get_type(dump->jcurrent) == json_type_array)
		json_object_array_add(dump->jcurrent, jobj);
	return 0;
}


static void extract_key(const char *field, char *key, size_t maxkey)
{
	const char *start1 = NULL;
	const char *start2 = NULL;
	const char *end = NULL;

	/* Start after last '.' */
	start1 = strrchr(field, '.');
	if (start1 == NULL)
		start1 = field;
	else
		start1++;

	/* Start after last '>' */
	start2 = strrchr(start1, '>');
	if (start2 == NULL)
		start2 = start1;
	else
		start2++;

	/* Skip spaces */
	while (*start2 == ' ')
		start2++;

	/* Go up to last '[' */
	end = strrchr(start2, '[');
	if (end == NULL)
		end = start2 + strlen(start2);

	/* Copy key */
	if ((size_t)(end - start2) < maxkey) {
		memcpy(key, start2, end - start2);
		key[end - start2] = '\0';
	} else {
		memcpy(key, start2, maxkey - 1);
		key[maxkey - 1] = '\0';
	}
}


static int
h264_dump_json_field(struct h264_dump *dump, const char *field, int64_t val)
{
	char key[256] = "";
#if defined(JSON_C_MAJOR_VERSION) && defined(JSON_C_MINOR_VERSION) &&          \
	((JSON_C_MAJOR_VERSION == 0 && JSON_C_MINOR_VERSION >= 10) ||          \
	 (JSON_C_MAJOR_VERSION > 0))
	json_object *jval = json_object_new_int64(val);
#else
	json_object *jval = json_object_new_int(val);
#endif
	extract_key(field, key, sizeof(key));
	if (json_object_get_type(dump->jcurrent) == json_type_array)
		json_object_array_add(dump->jcurrent, jval);
	else
		json_object_object_add(dump->jcurrent, key, jval);
	return 0;
}


int h264_dump_new(const struct h264_dump_cfg *cfg, struct h264_dump **ret_obj)
{
	struct h264_dump *dump = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);

	/* Allocate structure */
	dump = calloc(1, sizeof(*dump));
	if (dump == NULL)
		return -ENOMEM;

	/* Initialize structure */
	dump->cfg = *cfg;
	switch (dump->cfg.type) {
	case H264_DUMP_TYPE_JSON:
		dump->cbs.begin_struct = &h264_dump_json_begin_struct;
		dump->cbs.end_struct = &h264_dump_json_end_struct;
		dump->cbs.begin_array = &h264_dump_json_begin_array;
		dump->cbs.end_array = &h264_dump_json_end_array;
		dump->cbs.begin_array_item = &h264_dump_json_begin_array_item;
		dump->cbs.end_array_item = &h264_dump_json_end_array_item;
		dump->cbs.field = &h264_dump_json_field;
		h264_dump_json_push(dump, json_object_new_object());
		break;
	}

	/* Success */
	*ret_obj = dump;
	return 0;
}


int h264_dump_destroy(struct h264_dump *dump)
{
	if (dump == NULL)
		return 0;
	for (uint32_t i = 0; i < dump->jstacksize; i++)
		json_object_put(dump->jstack[i]);
	free(dump);
	return 0;
}


int h264_dump_clear(struct h264_dump *dump)
{
	ULOG_ERRNO_RETURN_ERR_IF(dump == NULL, EINVAL);
	switch (dump->cfg.type) {
	case H264_DUMP_TYPE_JSON:
		for (uint32_t i = 0; i < dump->jstacksize; i++)
			json_object_put(dump->jstack[i]);
		memset(dump->jstack, 0, sizeof(dump->jstack));
		dump->jstacksize = 0;
		h264_dump_json_push(dump, json_object_new_object());
		break;
	}
	return 0;
}


int h264_dump_get_json_object(struct h264_dump *dump, json_object **jobj)
{
	ULOG_ERRNO_RETURN_ERR_IF(dump == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(jobj == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dump->cfg.type != H264_DUMP_TYPE_JSON, EINVAL);
	*jobj = dump->jcurrent;
	return 0;
}


int h264_dump_get_json_str(struct h264_dump *dump, const char **str)
{
	ULOG_ERRNO_RETURN_ERR_IF(dump == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dump->cfg.type != H264_DUMP_TYPE_JSON, EINVAL);
	*str = json_object_to_json_string(dump->jcurrent);
	return 0;
}


int h264_dump_nalu(struct h264_dump *dump, struct h264_ctx *ctx, uint32_t flags)
{
	int res = 0;
	struct h264_bitstream bs;
	ULOG_ERRNO_RETURN_ERR_IF(dump == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);

	/* Save flags */
	dump->flags = flags;

	/* Setup a fake bitstream */
	h264_bs_cinit(&bs, NULL, 0, 1);
	bs.priv = dump;

	/* Clear current contents and dump */
	res = h264_dump_clear(dump);
	if (res >= 0)
		res = _h264_dump_nalu(&bs, ctx, NULL, NULL);

	h264_bs_clear(&bs);
	return res;
}
