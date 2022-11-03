/*
 * H.26L/H.264/AVC/JVT/14496-10/... encoder/decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Context Adaptive Binary Arithmetic Coder inline functions
 */

#ifndef AVCODEC_CABAC_FUNCTIONS_H
#define AVCODEC_CABAC_FUNCTIONS_H

#include <stdint.h>

#include "cabac.h"
#include "config.h"

#ifndef UNCHECKED_BITSTREAM_READER
#define UNCHECKED_BITSTREAM_READER !CONFIG_SAFE_BITSTREAM_READER
#endif

#if ARCH_AARCH64
#   include "aarch64/cabac.h"
#endif
#if ARCH_ARM
#   include "arm/cabac.h"
#endif
#if ARCH_X86
#   include "x86/cabac.h"
#endif

static const uint8_t * const ff_h264_norm_shift = ff_h264_cabac_tables + H264_NORM_SHIFT_OFFSET;
static const uint8_t * const ff_h264_lps_range = ff_h264_cabac_tables + H264_LPS_RANGE_OFFSET;
static const uint8_t * const ff_h264_mlps_state = ff_h264_cabac_tables + H264_MLPS_STATE_OFFSET;
static const uint8_t * const ff_h264_last_coeff_flag_offset_8x8 = ff_h264_cabac_tables + H264_LAST_COEFF_FLAG_OFFSET_8x8_OFFSET;

#if !defined(get_cabac_bypass) || !defined(get_cabac_terminate)
static void refill(CABACContext *c){
#if CABAC_BITS == 16
        c->low+= (c->bytestream[0]<<9) + (c->bytestream[1]<<1);
#else
        c->low+= c->bytestream[0]<<1;
#endif
    c->low -= CABAC_MASK;
#if !UNCHECKED_BITSTREAM_READER
    if (c->bytestream < c->bytestream_end)
#endif
        c->bytestream += CABAC_BITS / 8;
}
#endif

#ifndef get_cabac_terminate
static inline void renorm_cabac_decoder_once(CABACContext *c){
    int shift= (uint32_t)(c->range - 0x100)>>31;
    c->range<<= shift;
    c->low  <<= shift;
    if(!(c->low & CABAC_MASK))
        refill(c);
}
#endif

#ifndef get_cabac_inline
static void refill2(CABACContext *c){
    int i;
    unsigned x;
#if !HAVE_FAST_CLZ
    x= c->low ^ (c->low-1);
    i= 7 - ff_h264_norm_shift[x>>(CABAC_BITS-1)];
#else
    i = ff_ctz(c->low) - CABAC_BITS;
#endif

    x= -CABAC_MASK;

#if CABAC_BITS == 16
        x+= (c->bytestream[0]<<9) + (c->bytestream[1]<<1);
#else
        x+= c->bytestream[0]<<1;
#endif

    c->low += x<<i;
#if !UNCHECKED_BITSTREAM_READER
    if (c->bytestream < c->bytestream_end)
#endif
        c->bytestream += CABAC_BITS/8;
}
#endif

#ifndef get_cabac_inline
static av_always_inline int get_cabac_inline(CABACContext *c, uint8_t * const state){
    int s = *state;
    int RangeLPS= ff_h264_lps_range[2*(c->range&0xC0) + s];
    int bit, lps_mask;

    c->range -= RangeLPS;
    lps_mask= ((c->range<<(CABAC_BITS+1)) - c->low)>>31;

    c->low -= (c->range<<(CABAC_BITS+1)) & lps_mask;
    c->range += (RangeLPS - c->range) & lps_mask;

    s^=lps_mask;
    *state= (ff_h264_mlps_state+128)[s];
    bit= s&1;

    lps_mask= ff_h264_norm_shift[c->range];
    c->range<<= lps_mask;
    c->low  <<= lps_mask;
    if(!(c->low & CABAC_MASK))
        refill2(c);
    return bit;
}
#endif

static int av_noinline av_unused get_cabac_noinline(CABACContext *c, uint8_t * const state){
    return get_cabac_inline(c,state);
}

static int av_unused get_cabac(CABACContext *c, uint8_t * const state){
    return get_cabac_inline(c,state);
}

#ifndef get_cabac_bypass
static int av_unused get_cabac_bypass(CABACContext *c){
    int range;
    c->low += c->low;

    if(!(c->low & CABAC_MASK))
        refill(c);

    range= c->range<<(CABAC_BITS+1);
    if(c->low < range){
        return 0;
    }else{
        c->low -= range;
        return 1;
    }
}
#endif

#ifndef get_cabac_bypass_sign
static av_always_inline int get_cabac_bypass_sign(CABACContext *c, int val){
    int range, mask;
    c->low += c->low;

    if(!(c->low & CABAC_MASK))
        refill(c);

    range= c->range<<(CABAC_BITS+1);
    c->low -= range;
    mask= c->low >> 31;
    range &= mask;
    c->low += range;
    return (val^mask)-mask;
}
#endif

/**
 * @return the number of bytes read or 0 if no end
 */
#ifndef get_cabac_terminate
static int av_unused get_cabac_terminate(CABACContext *c){
    c->range -= 2;
    if(c->low < c->range<<(CABAC_BITS+1)){
        renorm_cabac_decoder_once(c);
        return 0;
    }else{
        return c->bytestream - c->bytestream_start;
    }
}
#endif

/**
 * Skip @p n bytes and reset the decoder.
 * @return the address of the first skipped byte or NULL if there's less than @p n bytes left
 */
#ifndef skip_bytes
static av_unused const uint8_t* skip_bytes(CABACContext *c, int n) {
    const uint8_t *ptr = c->bytestream;

    if (c->low & 0x1)
        ptr--;
#if CABAC_BITS == 16
    if (c->low & 0x1FF)
        ptr--;
#endif
    if ((int) (c->bytestream_end - ptr) < n)
        return NULL;
    if (ff_init_cabac_decoder(c, ptr + n, c->bytestream_end - ptr - n) < 0)
        return NULL;

    return ptr;
}
#endif

#if RECOMPRESS
static av_always_inline void cabac_putbyte( CABACContext *cb )
{
    if( cb->queue >= 0 )
    {
        int out = cb->low >> (cb->queue+10);
        cb->low &= (0x400<<cb->queue)-1;
        cb->queue -= 8;

        if( (out & 0xff) == 0xff )//8位
            cb->outstanding_count++;
        else
        {
            int carry = out >> 8;//多余的8位保存
            int bytes_outstanding = cb->outstanding_count;
            // this can't modify before the beginning of the stream because
            // that would correspond to a probability > 1.
            // it will write before the beginning of the stream, which is ok
            // because a slice header always comes before cabac data.
            // this can't carry beyond the one byte, because any 0xff bytes
            // are in bytes_outstanding and thus not written yet.
            cb->pb.buf_ptr[-1] += carry;
            while( bytes_outstanding > 0 )
            {
                *(cb->pb.buf_ptr++) = (carry-1);//diff
                bytes_outstanding--;
            }
            *(cb->pb.buf_ptr++) = out & 0xff;//是否只要低8位 //diff
            cb->outstanding_count = 0;
        }
    }
}

static inline void cabac_encode_renorm( CABACContext *cb )
{
    static const uint8_t cabac_renorm_shift[64] =
    {
        6,5,4,4,3,3,3,3,2,2,2,2,2,2,2,2,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    };
    int shift = cabac_renorm_shift[cb->range>>3];
    cb->range <<= shift;
    cb->low   <<= shift;
    cb->queue  += shift;
    cabac_putbyte( cb );
}

static av_always_inline void cabac_encode_bin( CABACContext *cb, uint8_t *state, int b )
{
    static const uint8_t cabac_range_lps[64][4] =
    {
        {  2,   2,   2,   2}, {  6,   7,   8,   9}, {  6,   7,   9,  10}, {  6,   8,   9,  11},
        {  7,   8,  10,  11}, {  7,   9,  10,  12}, {  7,   9,  11,  12}, {  8,   9,  11,  13},
        {  8,  10,  12,  14}, {  9,  11,  12,  14}, {  9,  11,  13,  15}, { 10,  12,  14,  16},
        { 10,  12,  15,  17}, { 11,  13,  15,  18}, { 11,  14,  16,  19}, { 12,  14,  17,  20},
        { 12,  15,  18,  21}, { 13,  16,  19,  22}, { 14,  17,  20,  23}, { 14,  18,  21,  24},
        { 15,  19,  22,  25}, { 16,  20,  23,  27}, { 17,  21,  25,  28}, { 18,  22,  26,  30},
        { 19,  23,  27,  31}, { 20,  24,  29,  33}, { 21,  26,  30,  35}, { 22,  27,  32,  37},
        { 23,  28,  33,  39}, { 24,  30,  35,  41}, { 26,  31,  37,  43}, { 27,  33,  39,  45},
        { 29,  35,  41,  48}, { 30,  37,  43,  50}, { 32,  39,  46,  53}, { 33,  41,  48,  56},
        { 35,  43,  51,  59}, { 37,  45,  54,  62}, { 39,  48,  56,  65}, { 41,  50,  59,  69},
        { 43,  53,  63,  72}, { 46,  56,  66,  76}, { 48,  59,  69,  80}, { 51,  62,  73,  85},
        { 53,  65,  77,  89}, { 56,  69,  81,  94}, { 59,  72,  86,  99}, { 62,  76,  90, 104},
        { 66,  80,  95, 110}, { 69,  85, 100, 116}, { 73,  89, 105, 122}, { 77,  94, 111, 128},
        { 81,  99, 117, 135}, { 85, 104, 123, 142}, { 90, 110, 130, 150}, { 95, 116, 137, 158},
        {100, 122, 144, 166}, {105, 128, 152, 175}, {111, 135, 160, 185}, {116, 142, 169, 195},
        {123, 150, 178, 205}, {128, 158, 187, 216}, {128, 167, 197, 227}, {128, 176, 208, 240}
    };

    static const uint8_t cabac_transition[128][2] =
    {
        {  0,   0}, {  1,   1}, {  2,  50}, { 51,   3}, {  2,  50}, { 51,   3}, {  4,  52}, { 53,   5},
        {  6,  52}, { 53,   7}, {  8,  52}, { 53,   9}, { 10,  54}, { 55,  11}, { 12,  54}, { 55,  13},
        { 14,  54}, { 55,  15}, { 16,  56}, { 57,  17}, { 18,  56}, { 57,  19}, { 20,  56}, { 57,  21},
        { 22,  58}, { 59,  23}, { 24,  58}, { 59,  25}, { 26,  60}, { 61,  27}, { 28,  60}, { 61,  29},
        { 30,  60}, { 61,  31}, { 32,  62}, { 63,  33}, { 34,  62}, { 63,  35}, { 36,  64}, { 65,  37},
        { 38,  66}, { 67,  39}, { 40,  66}, { 67,  41}, { 42,  66}, { 67,  43}, { 44,  68}, { 69,  45},
        { 46,  68}, { 69,  47}, { 48,  70}, { 71,  49}, { 50,  72}, { 73,  51}, { 52,  72}, { 73,  53},
        { 54,  74}, { 75,  55}, { 56,  74}, { 75,  57}, { 58,  76}, { 77,  59}, { 60,  78}, { 79,  61},
        { 62,  78}, { 79,  63}, { 64,  80}, { 81,  65}, { 66,  82}, { 83,  67}, { 68,  82}, { 83,  69},
        { 70,  84}, { 85,  71}, { 72,  84}, { 85,  73}, { 74,  88}, { 89,  75}, { 76,  88}, { 89,  77},
        { 78,  90}, { 91,  79}, { 80,  90}, { 91,  81}, { 82,  94}, { 95,  83}, { 84,  94}, { 95,  85},
        { 86,  96}, { 97,  87}, { 88,  96}, { 97,  89}, { 90, 100}, {101,  91}, { 92, 100}, {101,  93},
        { 94, 102}, {103,  95}, { 96, 104}, {105,  97}, { 98, 104}, {105,  99}, {100, 108}, {109, 101},
        {102, 108}, {109, 103}, {104, 110}, {111, 105}, {106, 112}, {113, 107}, {108, 114}, {115, 109},
        {110, 116}, {117, 111}, {112, 118}, {119, 113}, {114, 118}, {119, 115}, {116, 122}, {123, 117},
        {118, 122}, {123, 119}, {120, 124}, {125, 121}, {122, 126}, {127, 123}, {124, 127}, {126, 125}
    };
    int i_state = *state;
    int i_range_lps = cabac_range_lps[i_state>>1][(cb->range>>6)-4];
    cb->range -= i_range_lps;
    if( b != (i_state & 1) )
    {
        cb->low += cb->range;
        cb->range = i_range_lps;
    }
    *state = cabac_transition[i_state][b];//diff
    cabac_encode_renorm( cb );
}

static av_always_inline void cabac_encode_bypass(CABACContext *cb, int b )
{
    //temp
    int low_old = cb->low;

    cb->low <<= 1;
    cb->low += b & cb->range;
    cb->queue += 1;
    cabac_putbyte( cb );
}

static const int bypass_lut[16] =
{
    -1,      0x2,     0x14,     0x68,     0x1d0,     0x7a0,     0x1f40,     0x7e80,
    0x1fd00, 0x7fa00, 0x1ff400, 0x7fe800, 0x1ffd000, 0x7ffa000, 0x1fff4000, 0x7ffe8000
};

static av_always_inline av_const unsigned ff_clz_huawei(unsigned x){
    unsigned i = sizeof(x) * 8;
    while(x){
        x >>= 1;//这里写错过
        i--;
    }
    return i;
}

static av_always_inline void cabac_encode_ue_bypass( CABACContext *cb, int exp_bits, int val )
{
    uint32_t v = val + (1<<exp_bits);
    int k = 31 - ff_clz_huawei( v );
    uint32_t x = ((uint32_t)bypass_lut[k-exp_bits]<<exp_bits) + v;
    k = 2*k+1-exp_bits;
    int i = ((k-1)&7)+1;
    do {
        k -= i;
        cb->low <<= i;
        cb->low += ((x>>k)&0xff) * cb->range;
        cb->queue += i;
        cabac_putbyte( cb );
        i = 8;
    } while( k > 0 );
}

//每个cb末尾编一个terminal
static av_always_inline void cabac_encode_terminal( CABACContext *cb )
{
    cb->range -= 2;
    cabac_encode_renorm( cb );
}

//cb编完后刷新到buf里面去
static av_always_inline void cabac_encode_flush( CABACContext *cb, int is_i_frame )
{

    cb->low += cb->range - 2;
    cb->low |= 1;
    cb->low <<= 9;
    cb->queue += 9;
    cabac_putbyte( cb );
    cabac_putbyte( cb );
    cb->low <<= -cb->queue;

    //cb->low |= (0x35a4e4f5 >> (is_i_frame & 31) & 1) << 10;
    cb->queue = 0;
    cabac_putbyte( cb );

    while( cb->outstanding_count > 0 )
    {
        *(cb->pb.buf_ptr++) = 0xff;
        cb->outstanding_count--;
    }
}

#endif /* RECOMPRESS */

#endif /* AVCODEC_CABAC_FUNCTIONS_H */
