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


/**
 * Implementation of a read_vlc function with static parameters
 */
#define _READ_VLC(_maxbits, _maxcode)                                          \
	static int _h264_read_vlc_##_maxbits##_##_maxcode(                     \
		struct h264_bitstream *bs,                                     \
		const uint8_t(*table)[_maxbits + 1][_maxcode + 1],             \
		uint32_t *code)                                                \
	{                                                                      \
		int res = 0;                                                   \
		uint32_t bit = 0;                                              \
		uint32_t numbits = 0;                                          \
		uint32_t rawcode = 0;                                          \
		while (numbits < _maxbits) {                                   \
			/* Read one more bit */                                \
			res = h264_bs_read_bits(bs, &bit, 1);                  \
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);               \
			rawcode = (rawcode << 1) | bit;                        \
			numbits++;                                             \
			ULOG_ERRNO_RETURN_ERR_IF(rawcode > _maxcode, EIO);     \
			/* Are we done ? */                                    \
			if ((*table)[numbits][rawcode] != 0) {                 \
				*code = (*table)[numbits][rawcode];            \
				return 0;                                      \
			}                                                      \
		}                                                              \
		return -EIO;                                                   \
	}


/**
 * Call correct read_vlc function based on static parameters
 */
#define READ_VLC(_table, _maxbits, _maxcode, _code)                            \
	_h264_read_vlc_##_maxbits##_##_maxcode(bs, _table, _code)


/* Declare all read_vlc functions with static parameters */
_READ_VLC(16, 15)
_READ_VLC(9, 7)
_READ_VLC(3, 1)
_READ_VLC(5, 7)
_READ_VLC(11, 7)

/* clang-format off */


/**
 * 9.1.2 Mapping process for coded block pattern
 * ChromaArrayType is equal to 1 or 2
 */
static const uint8_t s_h264_coded_block_pattern_0[48][2] = {
	[0]  = {47,  0},
	[1]  = {31, 16},
	[2]  = {15,  1},
	[3]  = { 0,  2},
	[4]  = {23,  4},
	[5]  = {27,  8},
	[6]  = {29, 32},
	[7]  = {30,  3},
	[8]  = { 7,  5},
	[9]  = {11, 10},
	[10] = {13, 12},
	[11] = {14, 15},
	[12] = {39, 47},
	[13] = {43,  7},
	[14] = {45, 11},
	[15] = {46, 13},
	[16] = {16, 14},
	[17] = { 3,  6},
	[18] = { 5,  9},
	[19] = {10, 31},
	[20] = {12, 35},
	[21] = {19, 37},
	[22] = {21, 42},
	[23] = {26, 44},
	[24] = {28, 33},
	[25] = {35, 34},
	[26] = {37, 36},
	[27] = {42, 40},
	[28] = {44, 39},
	[29] = { 1, 43},
	[30] = { 2, 45},
	[31] = { 4, 46},
	[32] = { 8, 17},
	[33] = {17, 18},
	[34] = {18, 20},
	[35] = {20, 24},
	[36] = {24, 19},
	[37] = { 6, 21},
	[38] = { 9, 26},
	[39] = {22, 28},
	[40] = {25, 23},
	[41] = {32, 27},
	[42] = {33, 29},
	[43] = {34, 30},
	[44] = {36, 22},
	[45] = {40, 25},
	[46] = {38, 38},
	[47] = {41, 41},
};


/**
 * 9.1.2 Mapping process for coded block pattern
 * ChromaArrayType is equal to 0 or 3
 */
static const uint8_t s_h264_coded_block_pattern_1[16][2] = {
	[0]  = {15,  0},
	[1]  = { 0,  1},
	[2]  = { 7,  2},
	[3]  = {11,  4},
	[4]  = {13,  8},
	[5]  = {14,  3},
	[6]  = { 3,  5},
	[7]  = { 5, 10},
	[8]  = {10, 12},
	[9]  = {12, 15},
	[10] = { 1,  7},
	[11] = { 2, 11},
	[12] = { 4, 13},
	[13] = { 8, 14},
	[14] = { 6,  6},
	[15] = { 9,  9},
};


#define COEFF_TOKEN(_x, _y) (0x80 | ((_x) << 5) | (_y))


/**
 * 9.2.1 Parsing process for total number of non-zero transform coefficient
 * levels and number of trailing ones
 */

/* 0 <= nC < 2 */
static const uint8_t s_h264_coeff_token_0[17][16] = {
	[1][1]   = COEFF_TOKEN(0,  0), /* 1 */
	[6][5]   = COEFF_TOKEN(0,  1), /* 0001 01 */
	[2][1]   = COEFF_TOKEN(1,  1), /* 01 */
	[8][7]   = COEFF_TOKEN(0,  2), /* 0000 0111 */
	[6][4]   = COEFF_TOKEN(1,  2), /* 0001 00 */
	[3][1]   = COEFF_TOKEN(2,  2), /* 001 */
	[9][7]   = COEFF_TOKEN(0,  3), /* 0000 0011 1 */
	[8][6]   = COEFF_TOKEN(1,  3), /* 0000 0110 */
	[7][5]   = COEFF_TOKEN(2,  3), /* 0000 101 */
	[5][3]   = COEFF_TOKEN(3,  3), /* 0001 1 */
	[10][7]  = COEFF_TOKEN(0,  4), /* 0000 0001 11 */
	[9][6]   = COEFF_TOKEN(1,  4), /* 0000 0011 0 */
	[8][5]   = COEFF_TOKEN(2,  4), /* 0000 0101 */
	[6][3]   = COEFF_TOKEN(3,  4), /* 0000 11 */
	[11][7]  = COEFF_TOKEN(0,  5), /* 0000 0000 111 */
	[10][6]  = COEFF_TOKEN(1,  5), /* 0000 0001 10 */
	[9][5]   = COEFF_TOKEN(2,  5), /* 0000 0010 1 */
	[7][4]   = COEFF_TOKEN(3,  5), /* 0000 100 */
	[13][15] = COEFF_TOKEN(0,  6), /* 0000 0000 0111 1 */
	[11][6]  = COEFF_TOKEN(1,  6), /* 0000 0000 110 */
	[10][5]  = COEFF_TOKEN(2,  6), /* 0000 0001 01 */
	[8][4]   = COEFF_TOKEN(3,  6), /* 0000 0100 */
	[13][11] = COEFF_TOKEN(0,  7), /* 0000 0000 0101 1 */
	[13][14] = COEFF_TOKEN(1,  7), /* 0000 0000 0111 0 */
	[11][5]  = COEFF_TOKEN(2,  7), /* 0000 0000 101 */
	[9][4]   = COEFF_TOKEN(3,  7), /* 0000 0010 0 */
	[13][8]  = COEFF_TOKEN(0,  8), /* 0000 0000 0100 0 */
	[13][10] = COEFF_TOKEN(1,  8), /* 0000 0000 0101 0 */
	[13][13] = COEFF_TOKEN(2,  8), /* 0000 0000 0110 1 */
	[10][4]  = COEFF_TOKEN(3,  8), /* 0000 0001 00 */
	[14][15] = COEFF_TOKEN(0,  9), /* 0000 0000 0011 11 */
	[14][14] = COEFF_TOKEN(1,  9), /* 0000 0000 0011 10 */
	[13][9]  = COEFF_TOKEN(2,  9), /* 0000 0000 0100 1 */
	[11][4]  = COEFF_TOKEN(3,  9), /* 0000 0000 100 */
	[14][11] = COEFF_TOKEN(0, 10), /* 0000 0000 0010 11 */
	[14][10] = COEFF_TOKEN(1, 10), /* 0000 0000 0010 10 */
	[14][13] = COEFF_TOKEN(2, 10), /* 0000 0000 0011 01 */
	[13][12] = COEFF_TOKEN(3, 10), /* 0000 0000 0110 0 */
	[15][15] = COEFF_TOKEN(0, 11), /* 0000 0000 0001 111 */
	[15][14] = COEFF_TOKEN(1, 11), /* 0000 0000 0001 110 */
	[14][9]  = COEFF_TOKEN(2, 11), /* 0000 0000 0010 01 */
	[14][12] = COEFF_TOKEN(3, 11), /* 0000 0000 0011 00 */
	[15][11] = COEFF_TOKEN(0, 12), /* 0000 0000 0001 011 */
	[15][10] = COEFF_TOKEN(1, 12), /* 0000 0000 0001 010 */
	[15][13] = COEFF_TOKEN(2, 12), /* 0000 0000 0001 101 */
	[14][8]  = COEFF_TOKEN(3, 12), /* 0000 0000 0010 00 */
	[16][15] = COEFF_TOKEN(0, 13), /* 0000 0000 0000 1111 */
	[15][1]  = COEFF_TOKEN(1, 13), /* 0000 0000 0000 001 */
	[15][9]  = COEFF_TOKEN(2, 13), /* 0000 0000 0001 001 */
	[15][12] = COEFF_TOKEN(3, 13), /* 0000 0000 0001 100 */
	[16][11] = COEFF_TOKEN(0, 14), /* 0000 0000 0000 1011 */
	[16][14] = COEFF_TOKEN(1, 14), /* 0000 0000 0000 1110 */
	[16][13] = COEFF_TOKEN(2, 14), /* 0000 0000 0000 1101 */
	[15][8]  = COEFF_TOKEN(3, 14), /* 0000 0000 0001 000 */
	[16][7]  = COEFF_TOKEN(0, 15), /* 0000 0000 0000 0111 */
	[16][10] = COEFF_TOKEN(1, 15), /* 0000 0000 0000 1010 */
	[16][9]  = COEFF_TOKEN(2, 15), /* 0000 0000 0000 1001 */
	[16][12] = COEFF_TOKEN(3, 15), /* 0000 0000 0000 1100 */
	[16][4]  = COEFF_TOKEN(0, 16), /* 0000 0000 0000 0100 */
	[16][6]  = COEFF_TOKEN(1, 16), /* 0000 0000 0000 0110 */
	[16][5]  = COEFF_TOKEN(2, 16), /* 0000 0000 0000 0101 */
	[16][8]  = COEFF_TOKEN(3, 16), /* 0000 0000 0000 1000 */
};

/* 2 <= nC < 4 */
static const uint8_t s_h264_coeff_token_1[17][16] = {
	[2][3]   = COEFF_TOKEN(0,  0), /* 11 */
	[6][11]  = COEFF_TOKEN(0,  1), /* 0010 11 */
	[2][2]   = COEFF_TOKEN(1,  1), /* 10 */
	[6][7]   = COEFF_TOKEN(0,  2), /* 0001 11 */
	[5][7]   = COEFF_TOKEN(1,  2), /* 0011 1 */
	[3][3]   = COEFF_TOKEN(2,  2), /* 011 */
	[7][7]   = COEFF_TOKEN(0,  3), /* 0000 111 */
	[6][10]  = COEFF_TOKEN(1,  3), /* 0010 10 */
	[6][9]   = COEFF_TOKEN(2,  3), /* 0010 01 */
	[4][5]   = COEFF_TOKEN(3,  3), /* 0101 */
	[8][7]   = COEFF_TOKEN(0,  4), /* 0000 0111 */
	[6][6]   = COEFF_TOKEN(1,  4), /* 0001 10 */
	[6][5]   = COEFF_TOKEN(2,  4), /* 0001 01 */
	[4][4]   = COEFF_TOKEN(3,  4), /* 0100 */
	[8][4]   = COEFF_TOKEN(0,  5), /* 0000 0100 */
	[7][6]   = COEFF_TOKEN(1,  5), /* 0000 110 */
	[7][5]   = COEFF_TOKEN(2,  5), /* 0000 101 */
	[5][6]   = COEFF_TOKEN(3,  5), /* 0011 0 */
	[9][7]   = COEFF_TOKEN(0,  6), /* 0000 0011 1 */
	[8][6]   = COEFF_TOKEN(1,  6), /* 0000 0110 */
	[8][5]   = COEFF_TOKEN(2,  6), /* 0000 0101 */
	[6][8]   = COEFF_TOKEN(3,  6), /* 0010 00 */
	[11][15] = COEFF_TOKEN(0,  7), /* 0000 0001 111 */
	[9][6]   = COEFF_TOKEN(1,  7), /* 0000 0011 0 */
	[9][5]   = COEFF_TOKEN(2,  7), /* 0000 0010 1 */
	[6][4]   = COEFF_TOKEN(3,  7), /* 0001 00 */
	[11][11] = COEFF_TOKEN(0,  8), /* 0000 0001 011 */
	[11][14] = COEFF_TOKEN(1,  8), /* 0000 0001 110 */
	[11][13] = COEFF_TOKEN(2,  8), /* 0000 0001 101 */
	[7][4]   = COEFF_TOKEN(3,  8), /* 0000 100 */
	[12][15] = COEFF_TOKEN(0,  9), /* 0000 0000 1111 */
	[11][10] = COEFF_TOKEN(1,  9), /* 0000 0001 010 */
	[11][9]  = COEFF_TOKEN(2,  9), /* 0000 0001 001 */
	[9][4]   = COEFF_TOKEN(3,  9), /* 0000 0010 0 */
	[12][11] = COEFF_TOKEN(0, 10), /* 0000 0000 1011 */
	[12][14] = COEFF_TOKEN(1, 10), /* 0000 0000 1110 */
	[12][13] = COEFF_TOKEN(2, 10), /* 0000 0000 1101 */
	[11][12] = COEFF_TOKEN(3, 10), /* 0000 0001 100 */
	[12][8]  = COEFF_TOKEN(0, 11), /* 0000 0000 1000 */
	[12][10] = COEFF_TOKEN(1, 11), /* 0000 0000 1010 */
	[12][9]  = COEFF_TOKEN(2, 11), /* 0000 0000 1001 */
	[11][8]  = COEFF_TOKEN(3, 11), /* 0000 0001 000 */
	[13][15] = COEFF_TOKEN(0, 12), /* 0000 0000 0111 1 */
	[13][14] = COEFF_TOKEN(1, 12), /* 0000 0000 0111 0 */
	[13][13] = COEFF_TOKEN(2, 12), /* 0000 0000 0110 1 */
	[12][12] = COEFF_TOKEN(3, 12), /* 0000 0000 1100 */
	[13][11] = COEFF_TOKEN(0, 13), /* 0000 0000 0101 1 */
	[13][10] = COEFF_TOKEN(1, 13), /* 0000 0000 0101 0 */
	[13][9]  = COEFF_TOKEN(2, 13), /* 0000 0000 0100 1 */
	[13][12] = COEFF_TOKEN(3, 13), /* 0000 0000 0110 0 */
	[13][7]  = COEFF_TOKEN(0, 14), /* 0000 0000 0011 1 */
	[14][11] = COEFF_TOKEN(1, 14), /* 0000 0000 0010 11 */
	[13][6]  = COEFF_TOKEN(2, 14), /* 0000 0000 0011 0 */
	[13][8]  = COEFF_TOKEN(3, 14), /* 0000 0000 0100 0 */
	[14][9]  = COEFF_TOKEN(0, 15), /* 0000 0000 0010 01 */
	[14][8]  = COEFF_TOKEN(1, 15), /* 0000 0000 0010 00 */
	[14][10] = COEFF_TOKEN(2, 15), /* 0000 0000 0010 10 */
	[13][1]  = COEFF_TOKEN(3, 15), /* 0000 0000 0000 1 */
	[14][7]  = COEFF_TOKEN(0, 16), /* 0000 0000 0001 11 */
	[14][6]  = COEFF_TOKEN(1, 16), /* 0000 0000 0001 10 */
	[14][5]  = COEFF_TOKEN(2, 16), /* 0000 0000 0001 01 */
	[14][4]  = COEFF_TOKEN(3, 16), /* 0000 0000 0001 00 */
};

/* 4 <= nC < 8 */
static const uint8_t s_h264_coeff_token_2[17][16] = {
	[4][15]  = COEFF_TOKEN(0,  0), /* 1111 */
	[6][15]  = COEFF_TOKEN(0,  1), /* 0011 11 */
	[4][14]  = COEFF_TOKEN(1,  1), /* 1110 */
	[6][11]  = COEFF_TOKEN(0,  2), /* 0010 11 */
	[5][15]  = COEFF_TOKEN(1,  2), /* 0111 1 */
	[4][13]  = COEFF_TOKEN(2,  2), /* 1101 */
	[6][8]   = COEFF_TOKEN(0,  3), /* 0010 00 */
	[5][12]  = COEFF_TOKEN(1,  3), /* 0110 0 */
	[5][14]  = COEFF_TOKEN(2,  3), /* 0111 0 */
	[4][12]  = COEFF_TOKEN(3,  3), /* 1100 */
	[7][15]  = COEFF_TOKEN(0,  4), /* 0001 111 */
	[5][10]  = COEFF_TOKEN(1,  4), /* 0101 0 */
	[5][11]  = COEFF_TOKEN(2,  4), /* 0101 1 */
	[4][11]  = COEFF_TOKEN(3,  4), /* 1011 */
	[7][11]  = COEFF_TOKEN(0,  5), /* 0001 011 */
	[5][8]   = COEFF_TOKEN(1,  5), /* 0100 0 */
	[5][9]   = COEFF_TOKEN(2,  5), /* 0100 1 */
	[4][10]  = COEFF_TOKEN(3,  5), /* 1010 */
	[7][9]   = COEFF_TOKEN(0,  6), /* 0001 001 */
	[6][14]  = COEFF_TOKEN(1,  6), /* 0011 10 */
	[6][13]  = COEFF_TOKEN(2,  6), /* 0011 01 */
	[4][9]   = COEFF_TOKEN(3,  6), /* 1001 */
	[7][8]   = COEFF_TOKEN(0,  7), /* 0001 000 */
	[6][10]  = COEFF_TOKEN(1,  7), /* 0010 10 */
	[6][9]   = COEFF_TOKEN(2,  7), /* 0010 01 */
	[4][8]   = COEFF_TOKEN(3,  7), /* 1000 */
	[8][15]  = COEFF_TOKEN(0,  8), /* 0000 1111 */
	[7][14]  = COEFF_TOKEN(1,  8), /* 0001 110 */
	[7][13]  = COEFF_TOKEN(2,  8), /* 0001 101 */
	[5][13]  = COEFF_TOKEN(3,  8), /* 0110 1 */
	[8][11]  = COEFF_TOKEN(0,  9), /* 0000 1011 */
	[8][14]  = COEFF_TOKEN(1,  9), /* 0000 1110 */
	[7][10]  = COEFF_TOKEN(2,  9), /* 0001 010 */
	[6][12]  = COEFF_TOKEN(3,  9), /* 0011 00 */
	[9][15]  = COEFF_TOKEN(0, 10), /* 0000 0111 1 */
	[8][10]  = COEFF_TOKEN(1, 10), /* 0000 1010 */
	[8][13]  = COEFF_TOKEN(2, 10), /* 0000 1101 */
	[7][12]  = COEFF_TOKEN(3, 10), /* 0001 100 */
	[9][11]  = COEFF_TOKEN(0, 11), /* 0000 0101 1 */
	[9][14]  = COEFF_TOKEN(1, 11), /* 0000 0111 0 */
	[8][9]   = COEFF_TOKEN(2, 11), /* 0000 1001 */
	[8][12]  = COEFF_TOKEN(3, 11), /* 0000 1100 */
	[9][8]   = COEFF_TOKEN(0, 12), /* 0000 0100 0 */
	[9][10]  = COEFF_TOKEN(1, 12), /* 0000 0101 0 */
	[9][13]  = COEFF_TOKEN(2, 12), /* 0000 0110 1 */
	[8][8]   = COEFF_TOKEN(3, 12), /* 0000 1000 */
	[10][13] = COEFF_TOKEN(0, 13), /* 0000 0011 01 */
	[9][7]   = COEFF_TOKEN(1, 13), /* 0000 0011 1 */
	[9][9]   = COEFF_TOKEN(2, 13), /* 0000 0100 1 */
	[9][12]  = COEFF_TOKEN(3, 13), /* 0000 0110 0 */
	[10][9]  = COEFF_TOKEN(0, 14), /* 0000 0010 01 */
	[10][12] = COEFF_TOKEN(1, 14), /* 0000 0011 00 */
	[10][11] = COEFF_TOKEN(2, 14), /* 0000 0010 11 */
	[10][10] = COEFF_TOKEN(3, 14), /* 0000 0010 10 */
	[10][5]  = COEFF_TOKEN(0, 15), /* 0000 0001 01 */
	[10][8]  = COEFF_TOKEN(1, 15), /* 0000 0010 00 */
	[10][7]  = COEFF_TOKEN(2, 15), /* 0000 0001 11 */
	[10][6]  = COEFF_TOKEN(3, 15), /* 0000 0001 10 */
	[10][1]  = COEFF_TOKEN(0, 16), /* 0000 0000 01 */
	[10][4]  = COEFF_TOKEN(1, 16), /* 0000 0001 00 */
	[10][3]  = COEFF_TOKEN(2, 16), /* 0000 0000 11 */
	[10][2]  = COEFF_TOKEN(3, 16), /* 0000 0000 10 */
};

/* nC == −1 */
static const uint8_t s_h264_coeff_token_3[17][16] = {
	[2][1] = COEFF_TOKEN(0,  0), /* 01 */
	[6][7] = COEFF_TOKEN(0,  1), /* 0001 11 */
	[1][1] = COEFF_TOKEN(1,  1), /* 1 */
	[6][4] = COEFF_TOKEN(0,  2), /* 0001 00 */
	[6][6] = COEFF_TOKEN(1,  2), /* 0001 10 */
	[3][1] = COEFF_TOKEN(2,  2), /* 001 */
	[6][3] = COEFF_TOKEN(0,  3), /* 0000 11 */
	[7][3] = COEFF_TOKEN(1,  3), /* 0000 011 */
	[7][2] = COEFF_TOKEN(2,  3), /* 0000 010 */
	[6][5] = COEFF_TOKEN(3,  3), /* 0001 01 */
	[6][2] = COEFF_TOKEN(0,  4), /* 0000 10 */
	[8][3] = COEFF_TOKEN(1,  4), /* 0000 0011 */
	[8][2] = COEFF_TOKEN(2,  4), /* 0000 0010 */
	[7][0] = COEFF_TOKEN(3,  4), /* 0000 000 */
};

/* nC == −2 */
static const uint8_t s_h264_coeff_token_4[17][16] = {
	[1][1]   = COEFF_TOKEN(0,  0), /* 1 */
	[7][15]  = COEFF_TOKEN(0,  1), /* 0001 111 */
	[2][1]   = COEFF_TOKEN(1,  1), /* 01 */
	[7][14]  = COEFF_TOKEN(0,  2), /* 0001 110 */
	[7][13]  = COEFF_TOKEN(1,  2), /* 0001 101 */
	[3][1]   = COEFF_TOKEN(2,  2), /* 001 */
	[9][7]   = COEFF_TOKEN(0,  3), /* 0000 0011 1 */
	[7][12]  = COEFF_TOKEN(1,  3), /* 0001 100 */
	[7][11]  = COEFF_TOKEN(2,  3), /* 0001 011 */
	[5][1]   = COEFF_TOKEN(3,  3), /* 0000 1 */
	[9][6]   = COEFF_TOKEN(0,  4), /* 0000 0011 0 */
	[9][5]   = COEFF_TOKEN(1,  4), /* 0000 0010 1 */
	[7][10]  = COEFF_TOKEN(2,  4), /* 0001 010 */
	[6][1]   = COEFF_TOKEN(3,  4), /* 0000 01 */
	[10][7]  = COEFF_TOKEN(0,  5), /* 0000 0001 11 */
	[10][6]  = COEFF_TOKEN(1,  5), /* 0000 0001 10 */
	[9][4]   = COEFF_TOKEN(2,  5), /* 0000 0010 0 */
	[7][9]   = COEFF_TOKEN(3,  5), /* 0001 001 */
	[11][7]  = COEFF_TOKEN(0,  6), /* 0000 0000 111 */
	[11][6]  = COEFF_TOKEN(1,  6), /* 0000 0000 110 */
	[10][5]  = COEFF_TOKEN(2,  6), /* 0000 0001 01 */
	[7][8]   = COEFF_TOKEN(3,  6), /* 0001 000 */
	[12][7]  = COEFF_TOKEN(0,  7), /* 0000 0000 0111 */
	[12][6]  = COEFF_TOKEN(1,  7), /* 0000 0000 0110 */
	[11][5]  = COEFF_TOKEN(2,  7), /* 0000 0000 101 */
	[10][4]  = COEFF_TOKEN(3,  7), /* 0000 0001 00 */
	[13][7]  = COEFF_TOKEN(0,  8), /* 0000 0000 0011 1 */
	[12][5]  = COEFF_TOKEN(1,  8), /* 0000 0000 0101 */
	[12][4]  = COEFF_TOKEN(2,  8), /* 0000 0000 0100 */
	[11][4]  = COEFF_TOKEN(3,  8), /* 0000 0000 100 */
};

/* nC >= 8 */
static const uint8_t s_h264_coeff_token_5[64] = {
	[3]  = COEFF_TOKEN(0,  0), /* 0000 11 */
	[0]  = COEFF_TOKEN(0,  1), /* 0000 00 */
	[1]  = COEFF_TOKEN(1,  1), /* 0000 01 */
	[4]  = COEFF_TOKEN(0,  2), /* 0001 00 */
	[5]  = COEFF_TOKEN(1,  2), /* 0001 01 */
	[6]  = COEFF_TOKEN(2,  2), /* 0001 10 */
	[8]  = COEFF_TOKEN(0,  3), /* 0010 00 */
	[9]  = COEFF_TOKEN(1,  3), /* 0010 01 */
	[10] = COEFF_TOKEN(2,  3), /* 0010 10 */
	[11] = COEFF_TOKEN(3,  3), /* 0010 11 */
	[12] = COEFF_TOKEN(0,  4), /* 0011 00 */
	[13] = COEFF_TOKEN(1,  4), /* 0011 01 */
	[14] = COEFF_TOKEN(2,  4), /* 0011 10 */
	[15] = COEFF_TOKEN(3,  4), /* 0011 11 */
	[16] = COEFF_TOKEN(0,  5), /* 0100 00 */
	[17] = COEFF_TOKEN(1,  5), /* 0100 01 */
	[18] = COEFF_TOKEN(2,  5), /* 0100 10 */
	[19] = COEFF_TOKEN(3,  5), /* 0100 11 */
	[20] = COEFF_TOKEN(0,  6), /* 0101 00 */
	[21] = COEFF_TOKEN(1,  6), /* 0101 01 */
	[22] = COEFF_TOKEN(2,  6), /* 0101 10 */
	[23] = COEFF_TOKEN(3,  6), /* 0101 11 */
	[24] = COEFF_TOKEN(0,  7), /* 0110 00 */
	[25] = COEFF_TOKEN(1,  7), /* 0110 01 */
	[26] = COEFF_TOKEN(2,  7), /* 0110 10 */
	[27] = COEFF_TOKEN(3,  7), /* 0110 11 */
	[28] = COEFF_TOKEN(0,  8), /* 0111 00 */
	[29] = COEFF_TOKEN(1,  8), /* 0111 01 */
	[30] = COEFF_TOKEN(2,  8), /* 0111 10 */
	[31] = COEFF_TOKEN(3,  8), /* 0111 11 */
	[32] = COEFF_TOKEN(0,  9), /* 1000 00 */
	[33] = COEFF_TOKEN(1,  9), /* 1000 01 */
	[34] = COEFF_TOKEN(2,  9), /* 1000 10 */
	[35] = COEFF_TOKEN(3,  9), /* 1000 11 */
	[36] = COEFF_TOKEN(0, 10), /* 1001 00 */
	[37] = COEFF_TOKEN(1, 10), /* 1001 01 */
	[38] = COEFF_TOKEN(2, 10), /* 1001 10 */
	[39] = COEFF_TOKEN(3, 10), /* 1001 11 */
	[40] = COEFF_TOKEN(0, 11), /* 1010 00 */
	[41] = COEFF_TOKEN(1, 11), /* 1010 01 */
	[42] = COEFF_TOKEN(2, 11), /* 1010 10 */
	[43] = COEFF_TOKEN(3, 11), /* 1010 11 */
	[44] = COEFF_TOKEN(0, 12), /* 1011 00 */
	[45] = COEFF_TOKEN(1, 12), /* 1011 01 */
	[46] = COEFF_TOKEN(2, 12), /* 1011 10 */
	[47] = COEFF_TOKEN(3, 12), /* 1011 11 */
	[48] = COEFF_TOKEN(0, 13), /* 1100 00 */
	[49] = COEFF_TOKEN(1, 13), /* 1100 01 */
	[50] = COEFF_TOKEN(2, 13), /* 1100 10 */
	[51] = COEFF_TOKEN(3, 13), /* 1100 11 */
	[52] = COEFF_TOKEN(0, 14), /* 1101 00 */
	[53] = COEFF_TOKEN(1, 14), /* 1101 01 */
	[54] = COEFF_TOKEN(2, 14), /* 1101 10 */
	[55] = COEFF_TOKEN(3, 14), /* 1101 11 */
	[56] = COEFF_TOKEN(0, 15), /* 1110 00 */
	[57] = COEFF_TOKEN(1, 15), /* 1110 01 */
	[58] = COEFF_TOKEN(2, 15), /* 1110 10 */
	[59] = COEFF_TOKEN(3, 15), /* 1110 11 */
	[60] = COEFF_TOKEN(0, 16), /* 1111 00 */
	[61] = COEFF_TOKEN(1, 16), /* 1111 01 */
	[62] = COEFF_TOKEN(2, 16), /* 1111 10 */
	[63] = COEFF_TOKEN(3, 16), /* 1111 11 */
};


#define TOTAL_ZEROS(_x) (0x80 | (_x))


/**
 * total_zeros tables for 4x4 blocks
 */

static const uint8_t s_h264_total_zeros_0[16][10][8] = {
	/* tzVlcIndex = 1 */
	[1] = {
		[1][1] = TOTAL_ZEROS(0),  /* 1 */
		[3][3] = TOTAL_ZEROS(1),  /* 011 */
		[3][2] = TOTAL_ZEROS(2),  /* 010 */
		[4][3] = TOTAL_ZEROS(3),  /* 0011 */
		[4][2] = TOTAL_ZEROS(4),  /* 0010 */
		[5][3] = TOTAL_ZEROS(5),  /* 0001 1 */
		[5][2] = TOTAL_ZEROS(6),  /* 0001 0 */
		[6][3] = TOTAL_ZEROS(7),  /* 0000 11 */
		[6][2] = TOTAL_ZEROS(8),  /* 0000 10 */
		[7][3] = TOTAL_ZEROS(9),  /* 0000 011 */
		[7][2] = TOTAL_ZEROS(10), /* 0000 010 */
		[8][3] = TOTAL_ZEROS(11), /* 0000 0011 */
		[8][2] = TOTAL_ZEROS(12), /* 0000 0010 */
		[9][3] = TOTAL_ZEROS(13), /* 0000 0001 1 */
		[9][2] = TOTAL_ZEROS(14), /* 0000 0001 0 */
		[9][1] = TOTAL_ZEROS(15), /* 0000 0000 1 */
	},
	/* tzVlcIndex = 2 */
	[2] = {
		[3][7] = TOTAL_ZEROS(0),  /* 111 */
		[3][6] = TOTAL_ZEROS(1),  /* 110 */
		[3][5] = TOTAL_ZEROS(2),  /* 101 */
		[3][4] = TOTAL_ZEROS(3),  /* 100 */
		[3][3] = TOTAL_ZEROS(4),  /* 011 */
		[4][5] = TOTAL_ZEROS(5),  /* 0101 */
		[4][4] = TOTAL_ZEROS(6),  /* 0100 */
		[4][3] = TOTAL_ZEROS(7),  /* 0011 */
		[4][2] = TOTAL_ZEROS(8),  /* 0010 */
		[5][3] = TOTAL_ZEROS(9),  /* 0001 1 */
		[5][2] = TOTAL_ZEROS(10), /* 0001 0 */
		[6][3] = TOTAL_ZEROS(11), /* 0000 11 */
		[6][2] = TOTAL_ZEROS(12), /* 0000 10 */
		[6][1] = TOTAL_ZEROS(13), /* 0000 01 */
		[6][0] = TOTAL_ZEROS(14), /* 0000 00 */
	},
	/* tzVlcIndex = 3 */
	[3] = {
		[4][5] = TOTAL_ZEROS(0),  /* 0101 */
		[3][7] = TOTAL_ZEROS(1),  /* 111 */
		[3][6] = TOTAL_ZEROS(2),  /* 110 */
		[3][5] = TOTAL_ZEROS(3),  /* 101 */
		[4][4] = TOTAL_ZEROS(4),  /* 0100 */
		[4][3] = TOTAL_ZEROS(5),  /* 0011 */
		[3][4] = TOTAL_ZEROS(6),  /* 100 */
		[3][3] = TOTAL_ZEROS(7),  /* 011 */
		[4][2] = TOTAL_ZEROS(8),  /* 0010 */
		[5][3] = TOTAL_ZEROS(9),  /* 0001 1 */
		[5][2] = TOTAL_ZEROS(10), /* 0001 0 */
		[6][1] = TOTAL_ZEROS(11), /* 0000 01 */
		[5][1] = TOTAL_ZEROS(12), /* 0000 1 */
		[6][0] = TOTAL_ZEROS(13), /* 0000 00 */
	},
	/* tzVlcIndex = 4 */
	[4] = {
		[5][3] = TOTAL_ZEROS(0),  /* 0001 1 */
		[3][7] = TOTAL_ZEROS(1),  /* 111 */
		[4][5] = TOTAL_ZEROS(2),  /* 0101 */
		[4][4] = TOTAL_ZEROS(3),  /* 0100 */
		[3][6] = TOTAL_ZEROS(4),  /* 110 */
		[3][5] = TOTAL_ZEROS(5),  /* 101 */
		[3][4] = TOTAL_ZEROS(6),  /* 100 */
		[4][3] = TOTAL_ZEROS(7),  /* 0011 */
		[3][3] = TOTAL_ZEROS(8),  /* 011 */
		[4][2] = TOTAL_ZEROS(9),  /* 0010 */
		[5][2] = TOTAL_ZEROS(10), /* 0001 0 */
		[5][1] = TOTAL_ZEROS(11), /* 0000 1 */
		[5][0] = TOTAL_ZEROS(12), /* 0000 0 */
	},
	/* tzVlcIndex = 5 */
	[5] = {
		[4][5] = TOTAL_ZEROS(0),  /* 0101 */
		[4][4] = TOTAL_ZEROS(1),  /* 0100 */
		[4][3] = TOTAL_ZEROS(2),  /* 0011 */
		[3][7] = TOTAL_ZEROS(3),  /* 111 */
		[3][6] = TOTAL_ZEROS(4),  /* 110 */
		[3][5] = TOTAL_ZEROS(5),  /* 101 */
		[3][4] = TOTAL_ZEROS(6),  /* 100 */
		[3][3] = TOTAL_ZEROS(7),  /* 011 */
		[4][2] = TOTAL_ZEROS(8),  /* 0010 */
		[5][1] = TOTAL_ZEROS(9),  /* 0000 1 */
		[4][1] = TOTAL_ZEROS(10), /* 0001 */
		[5][0] = TOTAL_ZEROS(11), /* 0000 0 */
	},
	/* tzVlcIndex = 6 */
	[6] = {
		[6][1] = TOTAL_ZEROS(0),  /* 0000 01 */
		[5][1] = TOTAL_ZEROS(1),  /* 0000 1 */
		[3][7] = TOTAL_ZEROS(2),  /* 111 */
		[3][6] = TOTAL_ZEROS(3),  /* 110 */
		[3][5] = TOTAL_ZEROS(4),  /* 101 */
		[3][4] = TOTAL_ZEROS(5),  /* 100 */
		[3][3] = TOTAL_ZEROS(6),  /* 011 */
		[3][2] = TOTAL_ZEROS(7),  /* 010 */
		[4][1] = TOTAL_ZEROS(8),  /* 0001 */
		[3][1] = TOTAL_ZEROS(9),  /* 001 */
		[6][0] = TOTAL_ZEROS(10), /* 0000 00 */
	},
	/* tzVlcIndex = 7 */
	[7] = {
		[6][1] = TOTAL_ZEROS(0),  /* 0000 01 */
		[5][1] = TOTAL_ZEROS(1),  /* 0000 1 */
		[3][5] = TOTAL_ZEROS(2),  /* 101 */
		[3][4] = TOTAL_ZEROS(3),  /* 100 */
		[3][3] = TOTAL_ZEROS(4),  /* 011 */
		[2][3] = TOTAL_ZEROS(5),  /* 11 */
		[3][2] = TOTAL_ZEROS(6),  /* 010 */
		[4][1] = TOTAL_ZEROS(7),  /* 0001 */
		[3][1] = TOTAL_ZEROS(8),  /* 001 */
		[6][0] = TOTAL_ZEROS(9),  /* 0000 00 */
	},
	/* tzVlcIndex = 8 */
	[8] = {
		[6][1] = TOTAL_ZEROS(0),  /* 0000 01 */
		[4][1] = TOTAL_ZEROS(1),  /* 0001 */
		[5][1] = TOTAL_ZEROS(2),  /* 0000 1 */
		[3][3] = TOTAL_ZEROS(3),  /* 011 */
		[2][3] = TOTAL_ZEROS(4),  /* 11 */
		[2][2] = TOTAL_ZEROS(5),  /* 10 */
		[3][2] = TOTAL_ZEROS(6),  /* 010 */
		[3][1] = TOTAL_ZEROS(7),  /* 001 */
		[6][0] = TOTAL_ZEROS(8),  /* 0000 00 */
	},
	/* tzVlcIndex = 9 */
	[9] = {
		[6][1] = TOTAL_ZEROS(0),  /* 0000 01 */
		[6][0] = TOTAL_ZEROS(1),  /* 0000 00 */
		[4][1] = TOTAL_ZEROS(2),  /* 0001 */
		[2][3] = TOTAL_ZEROS(3),  /* 11 */
		[2][2] = TOTAL_ZEROS(4),  /* 10 */
		[3][1] = TOTAL_ZEROS(5),  /* 001 */
		[2][1] = TOTAL_ZEROS(6),  /* 01 */
		[5][1] = TOTAL_ZEROS(7),  /* 0000 1 */
	},
	/* tzVlcIndex = 10 */
	[10] = {
		[5][1] = TOTAL_ZEROS(0), /* 0000 1 */
		[5][0] = TOTAL_ZEROS(1),   /* 0000 0 */
		[3][1] = TOTAL_ZEROS(2),  /* 001 */
		[2][3] = TOTAL_ZEROS(3),  /* 11 */
		[2][2] = TOTAL_ZEROS(4),  /* 10 */
		[2][1] = TOTAL_ZEROS(5),  /* 01 */
		[4][1] = TOTAL_ZEROS(6),  /* 0001 */
	},
	/* tzVlcIndex = 11 */
	[11] = {
		[4][0] = TOTAL_ZEROS(0),  /* 0000 */
		[4][1] = TOTAL_ZEROS(1),  /* 0001 */
		[3][1] = TOTAL_ZEROS(2),  /* 001 */
		[3][2] = TOTAL_ZEROS(3),  /* 010 */
		[1][1] = TOTAL_ZEROS(4),  /* 1 */
		[3][3] = TOTAL_ZEROS(5),  /* 011 */
	},
	/* tzVlcIndex = 12 */
	[12] = {
		[4][0] = TOTAL_ZEROS(0),  /* 0000 */
		[4][1] = TOTAL_ZEROS(1),  /* 0001 */
		[2][1] = TOTAL_ZEROS(2),  /* 01 */
		[1][1] = TOTAL_ZEROS(3),  /* 1 */
		[3][1] = TOTAL_ZEROS(4),  /* 001 */
	},
	/* tzVlcIndex = 13 */
	[13] = {
		[3][0] = TOTAL_ZEROS(0),  /* 000 */
		[3][1] = TOTAL_ZEROS(1),  /* 001 */
		[1][1] = TOTAL_ZEROS(2),  /* 1 */
		[2][1] = TOTAL_ZEROS(3),  /* 01 */
	},
	/* tzVlcIndex = 14 */
	[14] = {
		[2][0] = TOTAL_ZEROS(0),  /* 00 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[1][1] = TOTAL_ZEROS(2),  /* 1 */
	},
	/* tzVlcIndex = 15 */
	[15] = {
		[1][0] = TOTAL_ZEROS(0),  /* 0 */
		[1][1] = TOTAL_ZEROS(1),  /* 1 */
	},
};


/**
 * total_zeros tables for Chroma DC 2x2 block (4:2:0 chroma sampling)
 */
static const uint8_t s_h264_total_zeros_1[4][4][2] = {
	/* tzVlcIndex = 1 */
	[1] = {
		[1][1] = TOTAL_ZEROS(0),  /* 1 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[3][1] = TOTAL_ZEROS(2),  /* 001 */
		[3][0] = TOTAL_ZEROS(3),  /* 000 */
	},
	/* tzVlcIndex = 2 */
	[2] = {
		[1][1] = TOTAL_ZEROS(0),  /* 1 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[2][0] = TOTAL_ZEROS(2),  /* 00 */
	},
	/* tzVlcIndex = 3 */
	[3] = {
		[1][1] = TOTAL_ZEROS(0),  /* 1 */
		[1][0] = TOTAL_ZEROS(1),  /* 0 */
	},
};


/**
 * total_zeros tables for Chroma DC 2x4 block (4:2:2 chroma sampling)
 */
static const uint8_t s_h264_total_zeros_2[8][6][8] = {
	/* tzVlcIndex = 1 */
	[1] = {
		[1][1] = TOTAL_ZEROS(0),  /* 1 */
		[3][2] = TOTAL_ZEROS(1),  /* 010 */
		[3][3] = TOTAL_ZEROS(2),  /* 011 */
		[4][2] = TOTAL_ZEROS(3),  /* 0010 */
		[4][3] = TOTAL_ZEROS(4),  /* 0011 */
		[4][1] = TOTAL_ZEROS(5),  /* 0001 */
		[5][1] = TOTAL_ZEROS(6),  /* 0000 1 */
		[5][0] = TOTAL_ZEROS(7),  /* 0000 0 */
	},
	/* tzVlcIndex = 2 */
	[2] = {
		[3][0] = TOTAL_ZEROS(0),  /* 000 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[3][1] = TOTAL_ZEROS(2),  /* 001 */
		[3][4] = TOTAL_ZEROS(3),  /* 100 */
		[3][5] = TOTAL_ZEROS(4),  /* 101 */
		[3][6] = TOTAL_ZEROS(5),  /* 110 */
		[3][7] = TOTAL_ZEROS(6),  /* 111 */
	},
	/* tzVlcIndex = 3 */
	[3] = {
		[3][0] = TOTAL_ZEROS(0),  /* 000 */
		[3][1] = TOTAL_ZEROS(1),  /* 001 */
		[2][1] = TOTAL_ZEROS(2),  /* 01 */
		[2][2] = TOTAL_ZEROS(3),  /* 10 */
		[3][6] = TOTAL_ZEROS(4),  /* 110 */
		[3][7] = TOTAL_ZEROS(5),  /* 111 */
	},
	/* tzVlcIndex = 4 */
	[4] = {
		[3][6] = TOTAL_ZEROS(0),  /* 110 */
		[2][0] = TOTAL_ZEROS(1),  /* 00 */
		[2][1] = TOTAL_ZEROS(2),  /* 01 */
		[2][2] = TOTAL_ZEROS(3),  /* 10 */
		[3][7] = TOTAL_ZEROS(4),  /* 111 */
	},
	/* tzVlcIndex = 5 */
	[5] = {
		[2][0] = TOTAL_ZEROS(0),  /* 00 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[2][2] = TOTAL_ZEROS(2),  /* 10 */
		[2][3] = TOTAL_ZEROS(3),  /* 11 */
	},
	/* tzVlcIndex = 6 */
	[6] = {
		[2][0] = TOTAL_ZEROS(0),  /* 00 */
		[2][1] = TOTAL_ZEROS(1),  /* 01 */
		[1][1] = TOTAL_ZEROS(2),  /* 1 */
	},
	/* tzVlcIndex = 7 */
	[7] = {
		[1][0] = TOTAL_ZEROS(0),  /* 0 */
		[1][1] = TOTAL_ZEROS(1),  /* 1 */
	},
};


#define RUN_BEFORE(_x) (0x80 | (_x))


/**
 * Tables for run_before
 */
static const uint8_t s_h264_run_before[8][12][8] = {
	/* zerosLeft = 1 */
	[1] = {
		[1][1] = RUN_BEFORE(0),  /* 0 */
		[1][0] = RUN_BEFORE(1),  /* 1 */
	},
	/* zerosLeft = 2 */
	[2] = {
		[1][1] = RUN_BEFORE(0),  /* 1 */
		[2][1] = RUN_BEFORE(1),  /* 01 */
		[2][0] = RUN_BEFORE(2),  /* 00 */
	},
	/* zerosLeft = 3 */
	[3] = {
		[2][3] = RUN_BEFORE(0),  /* 11 */
		[2][2] = RUN_BEFORE(1),  /* 10 */
		[2][1] = RUN_BEFORE(2),  /* 01 */
		[2][0] = RUN_BEFORE(3),  /* 00 */
	},
	/* zerosLeft = 4 */
	[4] = {
		[2][3] = RUN_BEFORE(0), /* 11 */
		[2][2] = RUN_BEFORE(1),  /* 10 */
		[2][1] = RUN_BEFORE(2),  /* 01 */
		[3][1] = RUN_BEFORE(3),  /* 001 */
		[3][0] = RUN_BEFORE(4),  /* 000 */
	},
	/* zerosLeft = 5 */
	[5] = {
		[2][3] = RUN_BEFORE(0), /* 11 */
		[2][2] = RUN_BEFORE(1),   /* 10 */
		[3][3] = RUN_BEFORE(2),  /* 011 */
		[3][2] = RUN_BEFORE(3),  /* 010 */
		[3][1] = RUN_BEFORE(4),  /* 001 */
		[3][0] = RUN_BEFORE(5),  /* 000 */
	},
	/* zerosLeft = 6 */
	[6] = {
		[2][3] = RUN_BEFORE(0),  /* 11 */
		[3][0] = RUN_BEFORE(1),  /* 000 */
		[3][1] = RUN_BEFORE(2),  /* 001 */
		[3][3] = RUN_BEFORE(3),  /* 011 */
		[3][2] = RUN_BEFORE(4),  /* 010 */
		[3][5] = RUN_BEFORE(5),  /* 101 */
		[3][4] = RUN_BEFORE(6),  /* 100 */
	},
	/* zerosLeft > 6 */
	[7] = {
		[3][7]  = RUN_BEFORE(0),  /* 111 */
		[3][6]  = RUN_BEFORE(1),  /* 110 */
		[3][5]  = RUN_BEFORE(2),  /* 101 */
		[3][4]  = RUN_BEFORE(3),  /* 100 */
		[3][3]  = RUN_BEFORE(4),  /* 011 */
		[3][2]  = RUN_BEFORE(5),  /* 010 */
		[3][1]  = RUN_BEFORE(6),  /* 001 */
		[4][1]  = RUN_BEFORE(7),  /* 0001 */
		[5][1]  = RUN_BEFORE(8),  /* 00001 */
		[6][1]  = RUN_BEFORE(9),  /* 000001 */
		[7][1]  = RUN_BEFORE(10), /* 0000001 */
		[8][1]  = RUN_BEFORE(11), /* 00000001 */
		[9][1]  = RUN_BEFORE(12), /* 000000001 */
		[10][1] = RUN_BEFORE(13), /* 0000000001 */
		[11][1] = RUN_BEFORE(14), /* 00000000001 */
	},
};

/* clang-format on */


/**
 * 7.4.5 Macroblock layer semantics
 */
int h264_read_mb_type(struct h264_bitstream *bs,
		      struct h264_ctx *ctx,
		      struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t type = 0;
	uint32_t off = h264_get_mb_addr_off(ctx, mb->mbAddr);

	static const uint8_t table[][3] = {
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L0, PredMode_Pred_L0},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L0, PredMode_Pred_L0},
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L1, PredMode_Pred_L1},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L1, PredMode_Pred_L1},
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L0, PredMode_Pred_L1},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L0, PredMode_Pred_L1},
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L1, PredMode_Pred_L0},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L1, PredMode_Pred_L0},
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L0, PredMode_BiPred},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L0, PredMode_BiPred},
		{H264_MB_TYPE_B_16x8, PredMode_Pred_L1, PredMode_BiPred},
		{H264_MB_TYPE_B_8x16, PredMode_Pred_L1, PredMode_BiPred},
		{H264_MB_TYPE_B_16x8, PredMode_BiPred, PredMode_Pred_L0},
		{H264_MB_TYPE_B_8x16, PredMode_BiPred, PredMode_Pred_L0},
		{H264_MB_TYPE_B_16x8, PredMode_BiPred, PredMode_Pred_L1},
		{H264_MB_TYPE_B_8x16, PredMode_BiPred, PredMode_Pred_L1},
		{H264_MB_TYPE_B_16x8, PredMode_BiPred, PredMode_BiPred},
		{H264_MB_TYPE_B_8x16, PredMode_BiPred, PredMode_BiPred},
	};

	res = h264_bs_read_bits_ue(bs, &type);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	mb->raw_mb_type = type;

	switch (ctx->slice.type) {
	case H264_SLICE_TYPE_UNKNOWN:
		break;
	case H264_SLICE_TYPE_I:
		/* clang-format off */
type_i:
		/* clang-format on */
		if (type == 0) {
			mb->mb_type = H264_MB_TYPE_I_NxN;
			mb->NumMbPart = 1;
			/* Will be changed to PredMode_Intra_8x8 if
			 * transform_size_8x8_flag is set */
			mb->MbPartPredMode[0] = PredMode_Intra_4x4;
		} else if (type >= 1 && type <= 24) {
			mb->mb_type = H264_MB_TYPE_I_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Intra_16x16;
			mb->Intra16x16PredMode = (type - 1) % 4;
			mb->CodedBlockPatternLuma = (type <= 12 ? 0 : 15);
			mb->CodedBlockPatternChroma = ((type - 1) / 4) % 3;
		} else if (type == 25) {
			mb->mb_type = H264_MB_TYPE_I_PCM;
			mb->NumMbPart = 0;
		} else {
			return -EIO;
		}
		break;

	case H264_SLICE_TYPE_SI:
		if (type == 0) {
			mb->mb_type = H264_MB_TYPE_SI;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Intra_4x4;
		} else {
			type -= 1;
			goto type_i;
		}
		break;

	case H264_SLICE_TYPE_P: /* NO BREAK */
	case H264_SLICE_TYPE_SP:
		if (type == 0) {
			mb->mb_type = H264_MB_TYPE_P_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Pred_L0;
		} else if (type == 1 || type == 2) {
			mb->mb_type = type == 1 ? H264_MB_TYPE_P_16x8
						: H264_MB_TYPE_P_8x16;
			mb->NumMbPart = 2;
			mb->MbPartPredMode[0] = PredMode_Pred_L0;
			mb->MbPartPredMode[1] = PredMode_Pred_L0;
		} else if (type == 3) {
			mb->mb_type = H264_MB_TYPE_P_8x8;
			mb->NumMbPart = 4;
		} else if (type == 4) {
			mb->mb_type = H264_MB_TYPE_P_8x8ref0;
			mb->NumMbPart = 4;
		} else {
			type -= 5;
			goto type_i;
		}
		break;

	case H264_SLICE_TYPE_B:
		if (type == 0) {
			mb->mb_type = H264_MB_TYPE_B_Direct_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Direct;
		} else if (type == 1) {
			mb->mb_type = H264_MB_TYPE_B_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Pred_L0;
		} else if (type == 2) {
			mb->mb_type = H264_MB_TYPE_B_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_Pred_L1;
		} else if (type == 3) {
			mb->mb_type = H264_MB_TYPE_B_16x16;
			mb->NumMbPart = 1;
			mb->MbPartPredMode[0] = PredMode_BiPred;
		} else if (type >= 4 && type <= 21) {
			mb->mb_type = table[type - 4][0];
			mb->NumMbPart = 2;
			mb->MbPartPredMode[0] = table[type - 4][1];
			mb->MbPartPredMode[1] = table[type - 4][2];
		} else if (type == 22) {
			mb->mb_type = H264_MB_TYPE_B_8x8;
			mb->NumMbPart = 4;
		} else {
			type -= 23;
			goto type_i;
		}
		break;
	}

	ctx->slice.mb_table.info[off].mb_type = mb->mb_type;
	return 0;
}


/**
 * 7.4.5.2 Sub-macroblock prediction semantics
 */
int h264_read_sub_mb_type(struct h264_bitstream *bs,
			  struct h264_ctx *ctx,
			  struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t type = 0;

	static const uint32_t table_P[][3] = {
		{SubMbType_P_8x8, 1, PredMode_Pred_L0},
		{SubMbType_P_8x4, 2, PredMode_Pred_L0},
		{SubMbType_P_4x8, 2, PredMode_Pred_L0},
		{SubMbType_P_4x4, 4, PredMode_Pred_L0},
	};
	static const uint32_t table_B[][3] = {
		{SubMbType_B_Direct_8x8, 4, PredMode_Direct},
		{SubMbType_B_8x8, 1, PredMode_Pred_L0},
		{SubMbType_B_8x8, 1, PredMode_Pred_L1},
		{SubMbType_B_8x8, 1, PredMode_BiPred},
		{SubMbType_B_8x4, 2, PredMode_Pred_L0},
		{SubMbType_B_4x8, 2, PredMode_Pred_L0},
		{SubMbType_B_8x4, 2, PredMode_Pred_L1},
		{SubMbType_B_4x8, 2, PredMode_Pred_L1},
		{SubMbType_B_8x4, 2, PredMode_BiPred},
		{SubMbType_B_4x8, 2, PredMode_BiPred},
		{SubMbType_B_4x4, 4, PredMode_Pred_L0},
		{SubMbType_B_4x4, 4, PredMode_Pred_L1},
		{SubMbType_B_4x4, 4, PredMode_BiPred},
	};

	for (uint32_t mbPartIdx = 0; mbPartIdx < 4; mbPartIdx++) {
		res = h264_bs_read_bits_ue(bs, &type);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		mb->raw_sub_mb_type[mbPartIdx] = type;

		switch (ctx->slice.type) {
		case H264_SLICE_TYPE_UNKNOWN: /* NO BREAK */
		case H264_SLICE_TYPE_I: /* NO BREAK */
		case H264_SLICE_TYPE_SI:
			break;

		case H264_SLICE_TYPE_P: /* NO BREAK */
		case H264_SLICE_TYPE_SP:
			ULOG_ERRNO_RETURN_ERR_IF(type >= ARRAY_SIZE(table_P),
						 EIO);
			mb->sub_mb_type[mbPartIdx] = table_P[type][0];
			mb->NumSubMbPart[mbPartIdx] = table_P[type][1];
			mb->SubMbPredMode[mbPartIdx] = table_P[type][2];
			break;

		case H264_SLICE_TYPE_B:
			ULOG_ERRNO_RETURN_ERR_IF(type >= ARRAY_SIZE(table_B),
						 EIO);
			mb->sub_mb_type[mbPartIdx] = table_B[type][0];
			mb->NumSubMbPart[mbPartIdx] = table_B[type][1];
			mb->SubMbPredMode[mbPartIdx] = table_B[type][2];
			break;
		}
	}

	return 0;
}


/**
 * 9.1.2 Mapping process for coded block pattern
 */
int h264_read_coded_block_pattern(struct h264_bitstream *bs,
				  struct h264_ctx *ctx,
				  struct h264_macroblock *mb)
{
	int res = 0;
	uint32_t code = 0;
	const uint8_t(*table)[][2] = NULL;

	res = h264_bs_read_bits_ue(bs, &code);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (ctx->sps_derived.ChromaArrayType == 1 ||
	    ctx->sps_derived.ChromaArrayType == 2) {
		ULOG_ERRNO_RETURN_ERR_IF(
			code >= ARRAY_SIZE(s_h264_coded_block_pattern_0), EIO);
		table = &s_h264_coded_block_pattern_0;
	} else if (ctx->sps_derived.ChromaArrayType == 0 ||
		   ctx->sps_derived.ChromaArrayType == 3) {
		ULOG_ERRNO_RETURN_ERR_IF(
			code >= ARRAY_SIZE(s_h264_coded_block_pattern_1), EIO);
		table = &s_h264_coded_block_pattern_1;
	} else {
		return -EIO;
	}

	switch (mb->mb_type) {
	case H264_MB_TYPE_I_NxN: /* NO BREAK */
	case H264_MB_TYPE_I_16x16: /* NO BREAK */
	case H264_MB_TYPE_SI:
		mb->coded_block_pattern = (*table)[code][0];
		break;
	default:
		mb->coded_block_pattern = (*table)[code][1];
		break;
	}

	mb->CodedBlockPatternLuma = mb->coded_block_pattern % 16;
	mb->CodedBlockPatternChroma = mb->coded_block_pattern / 16;
	return 0;
}


void h264_clear_macroblock_table(struct h264_ctx *ctx)
{
	if (ctx->slice.mb_table.info != NULL) {
		memset(ctx->slice.mb_table.info,
		       0,
		       ctx->slice.mb_table.maxlen *
			       sizeof(*ctx->slice.mb_table.info));
	}
	ctx->slice.mb_table.len = 0;
}


/**
 * 7.4.4 Slice data semantics
 */
int h264_new_macroblock(struct h264_ctx *ctx,
			uint32_t mbAddr,
			int skipped,
			int field_flag)
{
	void *newinfo = NULL;
	size_t newmaxlen = 0;
	const struct h264_slice_header *sh = &ctx->slice.hdr;
	struct h264_macroblock *mb = NULL;
	uint32_t off = h264_get_mb_addr_off(ctx, mbAddr);

	/* Grow internal array if needed */
	if (off + 1 > ctx->slice.mb_table.maxlen) {
		/* Alloc by step of 128 */
		newmaxlen = (off + 1 + 127) & ~127;
		newinfo =
			realloc(ctx->slice.mb_table.info,
				newmaxlen * sizeof(*ctx->slice.mb_table.info));
		if (newinfo == NULL)
			return -ENOMEM;
		/* Reset at 0 newly allocated entries */
		memset((uint8_t *)newinfo +
			       ctx->slice.mb_table.maxlen *
				       sizeof(*ctx->slice.mb_table.info),
		       0,
		       (newmaxlen - ctx->slice.mb_table.maxlen) *
			       sizeof(*ctx->slice.mb_table.info));
		ctx->slice.mb_table.info = newinfo;
		ctx->slice.mb_table.maxlen = newmaxlen;
	}
	if (off + 1 >= ctx->slice.mb_table.len)
		ctx->slice.mb_table.len = off + 1;

	ctx->slice.mb_table.info[off].available = 1;
	ctx->slice.mb_table.info[off].skipped = skipped;

	/* Setup new macroblock */
	mb = ctx->mb = &ctx->_mb;
	memset(ctx->mb, 0, sizeof(*ctx->mb));
	ctx->mb->mbAddr = mbAddr;
	ctx->mb->mb_type = !skipped ? H264_MB_TYPE_UNKNOWN
				    : ctx->slice.type == H264_SLICE_TYPE_B
					      ? H264_MB_TYPE_B_SKIP
					      : H264_MB_TYPE_P_SKIP;
	h264_compute_neighbouring_macroblocks(ctx, ctx->mb);

	/* Handle field flag */
	if (!ctx->derived.MbaffFrameFlag) {
		/* By default use field_pic_flag when not in MbaffFrame mode */
		ULOG_ERRNO_RETURN_ERR_IF(field_flag != -1, EINVAL);
		mb->mb_field_decoding_flag = sh->field_pic_flag;
	} else if (skipped) {
		if (mbAddr % 2 == 0) {
			/* Need to wait for the bottom macroblock to make a
			 * decision */
		} else if (!ctx->slice.mb_table.info[off - 1].skipped) {
			/* Use same flag as top macroblock */
			mb->mb_field_decoding_flag =
				ctx->slice.mb_table.info[off - 1].field_flag;
		} else {
			/* Both top and bottom macroblock are skipped */
			if (mb->mbAddrA != H264_MB_ADDR_INVALID) {
				/* Use left macroblock (A) info if available */
				mb->mb_field_decoding_flag =
					mb->mbAddrAInfo->field_flag;
			} else if (mb->mbAddrB != H264_MB_ADDR_INVALID) {
				/* Use top macroblock (B) info if available */
				mb->mb_field_decoding_flag =
					mb->mbAddrBInfo->field_flag;
			} else {
				mb->mb_field_decoding_flag = 0;
			}
			/* Update top macroblock info as well */
			ctx->slice.mb_table.info[off - 1].field_flag =
				mb->mb_field_decoding_flag;
		}
	} else if (mbAddr % 2 == 0) {
		/* Top macroblock , we should have an explicit field flag */
		ULOG_ERRNO_RETURN_ERR_IF(field_flag == -1, EINVAL);
		mb->mb_field_decoding_flag = field_flag;
	} else if (field_flag != -1) {
		/* Bottom macroblock with an explicit field flag, we need to
		 * update the top macro block as well (that should have been
		 * skipped) */
		mb->mb_field_decoding_flag = field_flag;
		ULOG_ERRNO_RETURN_ERR_IF(
			!ctx->slice.mb_table.info[off - 1].skipped, EINVAL);
		ctx->slice.mb_table.info[off - 1].field_flag =
			mb->mb_field_decoding_flag;
	} else {
		/* Bottom macroblock without an explicit field flag, use info
		 * from top macroblock (that should not have been skipped) */
		ULOG_ERRNO_RETURN_ERR_IF(
			ctx->slice.mb_table.info[off - 1].skipped, EINVAL);
		mb->mb_field_decoding_flag =
			ctx->slice.mb_table.info[off - 1].field_flag;
	}

	ctx->slice.mb_table.info[off].field_flag = mb->mb_field_decoding_flag;

	/* Setup some other variables */
	if (!ctx->derived.MbaffFrameFlag || !mb->mb_field_decoding_flag) {
		mb->max_ref_idx_0 = sh->num_ref_idx_l0_active_minus1;
		mb->max_ref_idx_1 = sh->num_ref_idx_l1_active_minus1;
	} else {
		mb->max_ref_idx_0 = 2 * sh->num_ref_idx_l0_active_minus1 + 1;
		mb->max_ref_idx_1 = 2 * sh->num_ref_idx_l1_active_minus1 + 1;
	}

	return 0;
}


int h264_set_nz_coeff(struct h264_ctx *ctx,
		      uint32_t mbAddr,
		      uint32_t comp,
		      uint32_t idx,
		      uint32_t n)
{
	uint32_t off = h264_get_mb_addr_off(ctx, mbAddr);
	ctx->slice.mb_table.info[off].nz_coeff[comp * 16 + idx] = n;
	return 0;
}


static int h264_get_nz_coeff(struct h264_ctx *ctx,
			     uint32_t mbAddr,
			     uint32_t comp,
			     uint32_t idx,
			     uint32_t *n)
{
	uint32_t off = h264_get_mb_addr_off(ctx, mbAddr);
	*n = ctx->slice.mb_table.info[off].nz_coeff[comp * 16 + idx];
	return 0;
}


/**
 * 9.2.1 Parsing process for total number of non-zero transform coefficient
 * levels and number of trailing ones
 */
int h264_read_coeff_token(struct h264_bitstream *bs,
			  struct h264_ctx *ctx,
			  struct h264_macroblock *mb,
			  uint32_t mode,
			  uint32_t comp,
			  uint32_t blkIdx,
			  uint32_t *trailing_ones,
			  uint32_t *total_coeff)
{
	int res = 0;
	uint32_t code = 0;
	uint32_t coeff_token = 0;
	uint32_t mbAddrA = H264_MB_ADDR_INVALID;
	uint32_t mbAddrB = H264_MB_ADDR_INVALID;
	uint32_t blkIdxA = 0, blkIdxB = 0;
	uint32_t nA = 0;
	uint32_t nB = 0;
	uint32_t nC = 0;
	int availableFlagA = 1;
	int availableFlagB = 1;

	switch (mode) {
	case Intra16x16DCLevel: /* NO BREAK */
	case Intra16x16ACLevel: /* NO BREAK */
	case LumaLevel4x4:
		h264_get_neighbouring_luma_cb_cr_4x4(ctx,
						     mb,
						     blkIdx,
						     &mbAddrA,
						     &blkIdxA,
						     &mbAddrB,
						     &blkIdxB);
		break;

	case CbIntra16x16DCLevel: /* NO BREAK */
	case CbIntra16x16ACLevel: /* NO BREAK */
	case CbLevel4x4:
		h264_get_neighbouring_luma_cb_cr_4x4(ctx,
						     mb,
						     blkIdx,
						     &mbAddrA,
						     &blkIdxA,
						     &mbAddrB,
						     &blkIdxB);
		break;

	case CrIntra16x16DCLevel: /* NO BREAK */
	case CrIntra16x16ACLevel: /* NO BREAK */
	case CrLevel4x4:
		h264_get_neighbouring_luma_cb_cr_4x4(ctx,
						     mb,
						     blkIdx,
						     &mbAddrA,
						     &blkIdxA,
						     &mbAddrB,
						     &blkIdxB);
		break;

	case ChromaDCLevel:
		if (ctx->sps_derived.ChromaArrayType == 1) {
			res = READ_VLC(
				&s_h264_coeff_token_3, 16, 15, &coeff_token);
		} else {
			res = READ_VLC(
				&s_h264_coeff_token_4, 16, 15, &coeff_token);
		}
		goto out;

	case ChromaACLevel:
		if (blkIdx >= 8) {
			res = -EPROTO;
			goto out;
		}
		h264_get_neighbouring_chroma_4x4(ctx,
						 mb,
						 blkIdx,
						 &mbAddrA,
						 &blkIdxA,
						 &mbAddrB,
						 &blkIdxB);
		break;
	}

	availableFlagA = mbAddrA != H264_MB_ADDR_INVALID;
	availableFlagB = mbAddrB != H264_MB_ADDR_INVALID;

	if (availableFlagA)
		h264_get_nz_coeff(ctx, mbAddrA, comp, blkIdxA, &nA);
	if (availableFlagB)
		h264_get_nz_coeff(ctx, mbAddrB, comp, blkIdxB, &nB);

	if (availableFlagA && availableFlagB)
		nC = (nA + nB + 1) >> 1;
	else if (availableFlagA && !availableFlagB)
		nC = nA;
	else if (!availableFlagA && availableFlagB)
		nC = nB;
	else
		nC = 0;

	if (nC < 2) {
		res = READ_VLC(&s_h264_coeff_token_0, 16, 15, &coeff_token);
	} else if (nC < 4) {
		res = READ_VLC(&s_h264_coeff_token_1, 16, 15, &coeff_token);
	} else if (nC < 8) {
		res = READ_VLC(&s_h264_coeff_token_2, 16, 15, &coeff_token);
	} else {
		res = h264_bs_read_bits(bs, &code, 6);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		coeff_token = s_h264_coeff_token_5[code];
		ULOG_ERRNO_RETURN_ERR_IF(coeff_token == 0, EIO);
		res = 0;
	}

out:
	if (res == 0) {
		*trailing_ones = (coeff_token >> 5) & 0x3;
		*total_coeff = coeff_token & 0x1f;
		h264_set_nz_coeff(ctx, mb->mbAddr, comp, blkIdx, *total_coeff);
	}
	return res;
}


/**
 * 9.2.3 Parsing process for run information
 */
int h264_read_total_zeros(struct h264_bitstream *bs,
			  uint32_t total_coeff,
			  uint32_t max_num_coeff,
			  uint32_t *total_zeros)
{
	int res = 0;
	uint32_t tzVlcIndex = total_coeff;
	uint32_t code = 0;
	if (total_coeff >= max_num_coeff) {
		*total_zeros = 0;
		return 0;
	}

	if (max_num_coeff == 4)
		res = READ_VLC(&s_h264_total_zeros_1[tzVlcIndex], 3, 1, &code);
	else if (max_num_coeff == 8)
		res = READ_VLC(&s_h264_total_zeros_2[tzVlcIndex], 5, 7, &code);
	else if (max_num_coeff <= 16)
		res = READ_VLC(&s_h264_total_zeros_0[tzVlcIndex], 9, 7, &code);
	else
		res = -EIO;
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	*total_zeros = code & 0x7f;
	return 0;
}


/**
 * 9.2.3 Parsing process for run information
 */
int h264_read_run_before(struct h264_bitstream *bs,
			 uint32_t zeros_left,
			 uint32_t *run_before)
{
	int res = 0;
	uint32_t code = 0;

	if (zeros_left == 0) {
		*run_before = 0;
		return 0;
	} else if (zeros_left <= 6) {
		res = READ_VLC(&s_h264_run_before[zeros_left], 11, 7, &code);
	} else {
		res = READ_VLC(&s_h264_run_before[7], 11, 7, &code);
	}
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	*run_before = code & 0x7f;
	return 0;
}
