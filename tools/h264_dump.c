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

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include "windows.h"
#else
#	include <sys/mman.h>
#endif

#define ULOG_TAG h264_dump
#include <ulog.h>
ULOG_DECLARE_TAG(h264_dump);

#include <h264/h264.h>
#include <json-c/json.h>


struct app {
	const char *inpath;
	const char *outpath;
	void *data;
	size_t size;
#ifdef _WIN32
	HANDLE infile;
	HANDLE map;
#else
	int fd;
#endif
	struct h264_reader *reader;
	struct h264_dump *dump;
	FILE *fout;
	uint32_t json_flags;
};


/*
 * For normal usage: pass SLICE_DATA to dump
 * For valgrind perf of reader only: pass SLICE_DATA to reader
 */
#if 0
#	define READER_FLAGS H264_READER_FLAGS_SLICE_DATA
#	define DUMP_FLAGS 0
#else
#	define READER_FLAGS 0
#	define DUMP_FLAGS H264_DUMP_FLAGS_SLICE_DATA
#endif


static void unmap_file(struct app *app)
{
#ifdef _WIN32
	if (app->data != NULL)
		UnmapViewOfFile(app->data);
	app->data = NULL;
	if (app->map != INVALID_HANDLE_VALUE)
		CloseHandle(app->map);
	app->map = INVALID_HANDLE_VALUE;
	if (app->infile != INVALID_HANDLE_VALUE)
		CloseHandle(app->infile);
	app->infile = INVALID_HANDLE_VALUE;
#else
	if (app->fd >= 0) {
		if (app->data != NULL)
			munmap(app->data, app->size);
		app->data = NULL;
		close(app->fd);
		app->fd = -1;
	}
#endif
}


static int map_file(struct app *app)
{
	int res;

#ifdef _WIN32
	LARGE_INTEGER filesize;

	app->infile = CreateFileA(app->inpath,
				  GENERIC_READ,
				  0,
				  NULL,
				  OPEN_EXISTING,
				  FILE_ATTRIBUTE_NORMAL,
				  NULL);
	if (app->infile == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileA('%s')", -res, app->inpath);
		goto error;
	}

	app->map =
		CreateFileMapping(app->infile, NULL, PAGE_READONLY, 0, 0, NULL);
	if (app->map == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileMapping('%s')", -res, app->inpath);
		goto error;
	}

	res = GetFileSizeEx(app->infile, &filesize);
	if (res == 0) {
		res = -EIO;
		ULOG_ERRNO("GetFileSizeEx('%s')", -res, app->inpath);
		goto error;
	}
	app->size = filesize.QuadPart;

	app->data = MapViewOfFile(app->map, FILE_MAP_READ, 0, 0, 0);
	if (app->data == NULL) {
		res = -EIO;
		ULOG_ERRNO("MapViewOfFile('%s')", -res, app->inpath);
		goto error;
	}
#else
	/* Try to open input file */
	app->fd = open(app->inpath, O_RDONLY);
	if (app->fd < 0) {
		res = -errno;
		ULOG_ERRNO("open('%s')", -res, app->inpath);
		goto error;
	}

	/* Get size and map it */
	app->size = lseek(app->fd, 0, SEEK_END);
	if (app->size == (size_t)-1) {
		res = -errno;
		ULOG_ERRNO("lseek", -res);
		goto error;
	}

	app->data = mmap(NULL, app->size, PROT_READ, MAP_PRIVATE, app->fd, 0);
	if (app->data == MAP_FAILED) {
		res = -errno;
		ULOG_ERRNO("mmap", -res);
		goto error;
	}
#endif

	return 0;

error:
	unmap_file(app);
	return res;
}


static void dump_buf(FILE *fout, const uint8_t *buf, size_t len)
{
	fprintf(fout, "len=%zu\n", len);
	for (size_t i = 0; i < 128 && i < len; i++) {
		fprintf(fout, "  %02x", buf[i]);
		if (i % 16 == 15)
			fprintf(fout, "\n");
	}
	fprintf(fout, "\n");
}


static void nalu_end_cb(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			void *userdata)
{
	int res = 0;
	struct app *app = userdata;
	json_object *jobj = NULL;
	const char *jstr = NULL;
	struct h264_bitstream bs;

	res = h264_dump_nalu(app->dump, ctx, DUMP_FLAGS);
	if (res < 0)
		ULOG_ERRNO("h264_dump_nalu", -res);

	res = h264_dump_get_json_object(app->dump, &jobj);
	if (res < 0)
		ULOG_ERRNO("h264_dump_get_json_object", -res);

#if defined(JSON_C_MAJOR_VERSION) && defined(JSON_C_MINOR_VERSION) &&          \
	((JSON_C_MAJOR_VERSION == 0 && JSON_C_MINOR_VERSION >= 10) ||          \
	 (JSON_C_MAJOR_VERSION > 0))
	jstr = json_object_to_json_string_ext(jobj, app->json_flags);
#else
	jstr = json_object_to_json_string(jobj);
#endif
	if (jstr == NULL)
		ULOG_ERRNO("json_object_to_json_string_ext", EINVAL);

	fprintf(app->fout, "%s\n", jstr);
	fflush(app->fout);

	if (h264_ctx_is_nalu_unknown(ctx)) {
		fprintf(app->fout, "OK\n");
	} else {
		int ok = 1;
		h264_bs_init(&bs, NULL, 0, 1);
		res = h264_write_nalu(&bs, ctx);
		fflush(app->fout);

		if (res < 0) {
			ULOG_ERRNO("h264_write_nalu", -res);
			ok = 0;
		} else if (bs.off == len) {
			ok = (memcmp(bs.data, buf, len) == 0);
		} else if (bs.off < len) {
			/* trailing_zero_8bits */
			ok = (memcmp(bs.data, buf, bs.off) == 0);
			for (uint32_t i = bs.off; ok && i < len; i++)
				ok = (buf[i] == 0x00);
		} else {
			ok = 0;
		}

		if (ok) {
			fprintf(app->fout, "OK\n");
			fflush(app->fout);
		} else {
			ULOGE("write mismatch");
			dump_buf(app->fout, buf, len);
			dump_buf(app->fout, bs.data, bs.off);
			fflush(app->fout);
		}

		h264_bs_clear(&bs);
	}
}


static const struct h264_ctx_cbs cbs = {
	.nalu_end = &nalu_end_cb,
};


enum args_id {
	ARGS_ID_JSON_PRETTY = 256,
};


static const char short_options[] = "ho:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"output", required_argument, NULL, 'o'},
	{"pretty", no_argument, NULL, ARGS_ID_JSON_PRETTY},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - Parrot H.264 bitstream dump tool\n"
	       "Copyright (c) 2016 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	printf("Usage: %s [options] <input file>\n"
	       "\n"
	       "Options:\n"
	       "-h | --help                        Print this message\n"
	       "-o | --output <file>               Output file\n"
	       "     --pretty                      Pretty output for "
	       "JSON file\n"
	       "\n",
	       prog_name);
}


int main(int argc, char *argv[])
{
	int res = 0;
	int idx, c;
	size_t off = 0;
	struct h264_dump_cfg dump_cfg;
	struct app app;

	memset(&app, 0, sizeof(app));
#ifdef _WIN32
	app.infile = INVALID_HANDLE_VALUE;
	app.map = INVALID_HANDLE_VALUE;
#else
	app.fd = -1;
#endif

	welcome(argv[0]);

	if (argc < 2) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
			break;

		case 'o':
			app.outpath = optarg;
			break;

		case ARGS_ID_JSON_PRETTY:
#ifdef JSON_C_TO_STRING_PRETTY
			app.json_flags = JSON_C_TO_STRING_PRETTY;
#else
			fprintf(stderr,
				"JSON_C_TO_STRING_PRETTY "
				"not supported\n");
			exit(EXIT_FAILURE);
#endif
			break;

		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}
	if (argc - optind < 1) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	app.inpath = argv[optind];
	if (app.inpath == NULL) {
		fprintf(stderr, "Missing input path\n");
		exit(EXIT_FAILURE);
	}

	/* Map the input file */
	res = map_file(&app);
	if (res < 0)
		goto out;

	/* Create reader object */
	res = h264_reader_new(&cbs, &app, &app.reader);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_new", -res);
		goto out;
	}

	/* Create json dump object */
	memset(&dump_cfg, 0, sizeof(dump_cfg));
	dump_cfg.type = H264_DUMP_TYPE_JSON;
	res = h264_dump_new(&dump_cfg, &app.dump);
	if (res < 0) {
		ULOG_ERRNO("h264_dump_new", -res);
		goto out;
	}

	/* Create output file */
	if (app.outpath != NULL) {
		app.fout = fopen(app.outpath, "w");
		if (app.fout == NULL) {
			res = -errno;
			ULOG_ERRNO("fopen('%s')", -res, app.outpath);
			goto out;
		}
	} else {
		/* Use stdout by default */
		app.fout = stdout;
	}

	/* Parse stream */
	res = h264_reader_parse(
		app.reader, READER_FLAGS, app.data, app.size, &off);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse", -res);
		goto out;
	}

out:
	/* Cleanup */
	if (app.reader != NULL) {
		res = h264_reader_destroy(app.reader);
		if (res < 0)
			ULOG_ERRNO("h264_reader_destroy", -res);
	}
	if (app.dump != NULL) {
		res = h264_dump_destroy(app.dump);
		if (res < 0)
			ULOG_ERRNO("h264_dump_destroy", -res);
	}
	if (app.fout != NULL && app.fout != stderr)
		fclose(app.fout);
	unmap_file(&app);

	return res >= 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
