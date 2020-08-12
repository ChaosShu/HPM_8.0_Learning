/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#include "com_tbl.h"
#include "com_mc.h"
#include "com_util.h"
#include <assert.h>

#define MAC_SFT_N0            (6)
#define MAC_ADD_N0            (1<<5)

#define MAC_SFT_0N            MAC_SFT_N0
#define MAC_ADD_0N            MAC_ADD_N0

#define BIO_IF_S1(BIT_DEPTH)  (MAC_SFT_N0 + BIT_DEPTH - 14)
#define BIO_IF_S2(BIT_DEPTH)  (MAC_SFT_N0 - BIT_DEPTH + 18)
#define BIO_IF_O1(BIT_DEPTH)  (1 << (BIO_IF_S1(BIT_DEPTH) - 1))
#define BIO_IF_O2(BIT_DEPTH)  (1 << (BIO_IF_S2(BIT_DEPTH) - 1))

#define MAC_8TAP(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3)+(c)[4]*(r4)+\
    (c)[5]*(r5)+(c)[6]*(r6)+(c)[7]*(r7))

#define MAC_4TAP(c, r0, r1, r2, r3) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3))

/* padding for store intermediate values, which should be larger than
1+ half of filter tap */
#define MC_IBUF_PAD_C          4
#define MC_IBUF_PAD_L          8

#if SIMD_MC
static const s16 tbl_bl_mc_l_coeff[4][2] =
#else
static const s8 tbl_bl_mc_l_coeff[4][2] =
#endif
{
    { 64,  0 },
    { 48, 16 },
    { 32, 32 },
    { 16, 48 },
};


/****************************************************************************
 * motion compensation for luma
 ****************************************************************************/

#if SIMD_MC
static const s8 shuffle_2Tap[16] = {0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9};

void mc_filter_bilin_horz_sse
(
    s16 const *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const short *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i row1, row11, row2, row22, row3, row33, row4, row44;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b, shuffle;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/
    shuffle = _mm_loadu_si128((__m128i*)shuffle_2Tap);
    rem_h = (height & 0x3);
    if(rem_w > 7)
    {
        for(row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                             /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));                        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));       /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row22 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 1));  /*b1 b2 b3 b4 b5 b6 b7 b8*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);            /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);          /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row4 = _mm_srai_epi32(row4, shift);
                row44 = _mm_srai_epi32(row44, shift);
                row1 = _mm_packs_epi32(row1, row2);
                row11 = _mm_packs_epi32(row11, row22);
                row3 = _mm_packs_epi32(row3, row4);
                row33 = _mm_packs_epi32(row33, row44);
                res0 = _mm_unpacklo_epi16(row1, row11);
                res1 = _mm_unpackhi_epi16(row1, row11);
                res2 = _mm_unpacklo_epi16(row3, row33);
                res3 = _mm_unpackhi_epi16(row3, row33);
                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);
                cnt += 8; /* To pointer updates*/
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        /*extra height to be done --- one row at a time*/
        for(row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));       /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));  /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);              /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);            /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                row1 = _mm_packs_epi32(row1, row11);    /*a0 a2 a4 a6 a1 a3 a5 a7*/
                res0 = _mm_unpackhi_epi64(row1, row1);  /*a1 a3 a5 a7*/
                res1 = _mm_unpacklo_epi16(row1, res0);  /*a0 a1 a2 a3 a4 a5 a6 a7*/
                if(is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if(rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for(row = height; row > 3; row -= 4)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
            row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));  /*a1 a2 a3 a4 a5 a6 a7 a8*/
            row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3));
            row1 = _mm_shuffle_epi8(row1, shuffle);  /*a0 a1 a1 a2 a2 a3 a3 a4 */
            row2 = _mm_shuffle_epi8(row2, shuffle);
            row3 = _mm_shuffle_epi8(row3, shuffle);
            row4 = _mm_shuffle_epi8(row4, shuffle);
            row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
            row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
            row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
            row1 = _mm_add_epi32(row1, offset_4x32b);
            row2 = _mm_add_epi32(row2, offset_4x32b);
            row3 = _mm_add_epi32(row3, offset_4x32b);
            row4 = _mm_add_epi32(row4, offset_4x32b);
            row1 = _mm_srai_epi32(row1, shift);
            row2 = _mm_srai_epi32(row2, shift);
            row3 = _mm_srai_epi32(row3, shift);
            row4 = _mm_srai_epi32(row4, shift);
            res0 = _mm_packs_epi32(row1, row2);
            res1 = _mm_packs_epi32(row3, row4);
            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }
            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for(row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));  /*a0 a1 a2 a3 a4 a5 a6 a7*/
            res0 = _mm_shuffle_epi8(row1, shuffle);        /*a0 a1 a1 a2 a2 a3 a3 a4 */
            res0 = _mm_madd_epi16(res0, coeff0_1_8x16b);   /*a0+a1 a1+a2 a2+a3 a3+a4*/
            res0 = _mm_add_epi32(res0, offset_4x32b);
            res0 = _mm_srai_epi32(res0, shift);
            res0 = _mm_packs_epi32(res0, res0);
            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if(rem_w)
    {
        int sum, sum1;
        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for(row = height; row > 3; row -= 4)
        {
            for(col = 0; col < rem_w; col++)
            {
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + col));                        /*a0 a1 x x x x x x*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));  /*b0 b1 x x x x x x*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + col));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + col));
                row1 = _mm_unpacklo_epi32(row1, row2);  /*a0 a1 b0 b1*/
                row3 = _mm_unpacklo_epi32(row3, row4);  /*c0 c1 d0 d1*/
                row1 = _mm_unpacklo_epi64(row1, row3);  /*a0 a1 b0 b1 c0 c1 d0 d1*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 b0+b1 c0+c1 d0+d1*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                res0 = _mm_packs_epi32(row1, row1);
                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                }
                /*extract 32 bit integer form register and store it in dst_copy*/
                sum = _mm_extract_epi32(res0, 0);
                sum1 = _mm_extract_epi32(res0, 1);
                dst_copy[col] = (s16)(sum & 0xffff);
                dst_copy[col + dst_stride] = (s16)(sum >> 16);
                dst_copy[col + (dst_stride << 1)] = (s16)(sum1 & 0xffff);
                dst_copy[col + (dst_stride * 3)] = (s16)(sum1 >> 16);
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for(row = 0; row < rem_h; row++)
        {
            for(col = 0; col < rem_w; col++)
            {
                int val;
                sum = inp_copy[col + 0] * coeff[0];
                sum += inp_copy[col + 1] * coeff[1];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

void mc_filter_bilin_vert_sse
(
    s16 const *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const short *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3, src_stride4;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i row1, row11, row2, row22, row3, row33, row4, row44, row5;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    src_stride4 = (stored_alf_para_num << 2);
    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/
    rem_h = height & 0x3;
    if (rem_w > 7)
    {
        for (row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row5 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride4 + cnt));
                row11 = _mm_unpacklo_epi16(row1, row2);   /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);    /*a4 b4 a5 b5 a6 b6 a7 b7*/
                row22 = _mm_unpacklo_epi16(row2, row3);
                row2 = _mm_unpackhi_epi16(row2, row3);
                row33 = _mm_unpacklo_epi16(row3, row4);
                row3 = _mm_unpackhi_epi16(row3, row4);
                row44 = _mm_unpacklo_epi16(row4, row5);
                row4 = _mm_unpackhi_epi16(row4, row5);
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                row1 = _mm_srai_epi32(row1, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row44 = _mm_srai_epi32(row44, shift);
                row4 = _mm_srai_epi32(row4, shift);
                res0 = _mm_packs_epi32(row11, row1);
                res1 = _mm_packs_epi32(row22, row2);
                res2 = _mm_packs_epi32(row33, row3);
                res3 = _mm_packs_epi32(row44, row4);
                if (is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);
                cnt += 8;  /* To pointer updates*/
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        /*extra height to be done --- one row at a time*/
        for (row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);   /*a4 b4 a5 b5 a6 b6 a7 b7*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                res1 = _mm_packs_epi32(row11, row1);
                if (is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = height; row > 3; row -= 4)
        {
            /*load 4 pixel values */
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));  /*b0 b1 b2 b3 x x x x*/
            row3 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
            row5 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride4));
            row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row22 = _mm_unpacklo_epi16(row2, row3);
            row33 = _mm_unpacklo_epi16(row3, row4);
            row44 = _mm_unpacklo_epi16(row4, row5);
            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
            row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
            row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
            row11 = _mm_add_epi32(row11, offset_4x32b);
            row22 = _mm_add_epi32(row22, offset_4x32b);
            row33 = _mm_add_epi32(row33, offset_4x32b);
            row44 = _mm_add_epi32(row44, offset_4x32b);
            row11 = _mm_srai_epi32(row11, shift);
            row22 = _mm_srai_epi32(row22, shift);
            row33 = _mm_srai_epi32(row33, shift);
            row44 = _mm_srai_epi32(row44, shift);
            res0 = _mm_packs_epi32(row11, row22);
            res1 = _mm_packs_epi32(row33, row44);
            if (is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }
            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for (row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));  /*b0 b1 b2 b3 x x x x*/
            row11 = _mm_unpacklo_epi16(row1, row2);         /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row11 = _mm_add_epi32(row11, offset_4x32b);
            row11 = _mm_srai_epi32(row11, shift);
            row11 = _mm_packs_epi32(row11, row11);
            if (is_last)
            {
                row11 = _mm_min_epi16(row11, mm_max);
                row11 = _mm_max_epi16(row11, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), row11);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif

#if SIMD_MC
#if AWP
void weight_average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, s16 *weight0, s16 *weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht)
{
    s16 *p0, *p1, *p2;
    s16 *w0, *w1;
    int rem_h = ht;
    int rem_w;
    int i, j;
    __m128i src_8x16b, src_8x16b_1, src_8x16b_2, src_8x16b_3;
    __m128i pred_8x16b, pred_8x16b_1, pred_8x16b_2, pred_8x16b_3;
    __m128i weight0_8x16b, weight0_8x16b_1, weight0_8x16b_2, weight0_8x16b_3;
    __m128i weight1_8x16b, weight1_8x16b_1, weight1_8x16b_2, weight1_8x16b_3;
    __m128i temp_0, temp_1, temp_2, temp_3;
    __m128i offset_8x16b;
    /* Can be changed for a generic avg fun. or taken as an argument! */
    short offset = 4;
    int shift = 3;
    p0 = src;
    p1 = ref;
    p2 = dst;
    w0 = weight0;
    w1 = weight1;

    offset_8x16b = _mm_set1_epi16(offset);

    /* Mult. of 4 Loop */
    if (rem_h >= 4)
    {
        for (i = 0; i < rem_h - 3; i += 4)
        {
            p0 = src + (i * s_src);
            p1 = ref + (i * s_ref);
            p2 = dst + (i * s_dst);
            w0 = weight0 + (i * s_weight);
            w1 = weight1 + (i * s_weight);

            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j > 7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    src_8x16b_2 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 2)));
                    src_8x16b_3 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 3)));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    pred_8x16b_2 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 2)));
                    pred_8x16b_3 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 3)));
                    weight0_8x16b = _mm_loadu_si128((__m128i *) (w0));
                    weight0_8x16b_1 = _mm_loadu_si128((__m128i *) (w0 + s_weight));
                    weight0_8x16b_2 = _mm_loadu_si128((__m128i *) (w0 + (s_weight * 2)));
                    weight0_8x16b_3 = _mm_loadu_si128((__m128i *) (w0 + (s_weight * 3)));
                    weight1_8x16b = _mm_loadu_si128((__m128i *) (w1));
                    weight1_8x16b_1 = _mm_loadu_si128((__m128i *) (w1 + s_weight));
                    weight1_8x16b_2 = _mm_loadu_si128((__m128i *) (w1 + (s_weight * 2)));
                    weight1_8x16b_3 = _mm_loadu_si128((__m128i *) (w1 + (s_weight * 3)));
                    //add mul
                    src_8x16b = _mm_mullo_epi16(src_8x16b, weight0_8x16b);
                    src_8x16b_1 = _mm_mullo_epi16(src_8x16b_1, weight0_8x16b_1);
                    src_8x16b_2 = _mm_mullo_epi16(src_8x16b_2, weight0_8x16b_2);
                    src_8x16b_3 = _mm_mullo_epi16(src_8x16b_3, weight0_8x16b_3);

                    pred_8x16b = _mm_mullo_epi16(pred_8x16b, weight1_8x16b);
                    pred_8x16b_1 = _mm_mullo_epi16(pred_8x16b_1, weight1_8x16b_1);
                    pred_8x16b_2 = _mm_mullo_epi16(pred_8x16b_2, weight1_8x16b_2);
                    pred_8x16b_3 = _mm_mullo_epi16(pred_8x16b_3, weight1_8x16b_3);

                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                    temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                    temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    temp_2 = _mm_srai_epi16(temp_2, shift);
                    temp_3 = _mm_srai_epi16(temp_3, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    _mm_storeu_si128((__m128i *)(p2 + 2 * s_dst), temp_2);
                    _mm_storeu_si128((__m128i *)(p2 + 3 * s_dst), temp_3);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                    w0 += 8;
                    w1 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                src_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 2)));
                src_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 3)));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                pred_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 2)));
                pred_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 3)));
                weight0_8x16b = _mm_loadl_epi64((__m128i *) (w0));
                weight0_8x16b_1 = _mm_loadl_epi64((__m128i *) (w0 + s_weight));
                weight0_8x16b_2 = _mm_loadl_epi64((__m128i *) (w0 + (s_weight * 2)));
                weight0_8x16b_3 = _mm_loadl_epi64((__m128i *) (w0 + (s_weight * 3)));
                weight1_8x16b = _mm_loadl_epi64((__m128i *) (w1));
                weight1_8x16b_1 = _mm_loadl_epi64((__m128i *) (w1 + s_weight));
                weight1_8x16b_2 = _mm_loadl_epi64((__m128i *) (w1 + (s_weight * 2)));
                weight1_8x16b_3 = _mm_loadl_epi64((__m128i *) (w1 + (s_weight * 3)));
                //add mul
                src_8x16b = _mm_mullo_epi16(src_8x16b, weight0_8x16b);
                src_8x16b_1 = _mm_mullo_epi16(src_8x16b_1, weight0_8x16b_1);
                src_8x16b_2 = _mm_mullo_epi16(src_8x16b_2, weight0_8x16b_2);
                src_8x16b_3 = _mm_mullo_epi16(src_8x16b_3, weight0_8x16b_3);

                pred_8x16b = _mm_mullo_epi16(pred_8x16b, weight1_8x16b);
                pred_8x16b_1 = _mm_mullo_epi16(pred_8x16b_1, weight1_8x16b_1);
                pred_8x16b_2 = _mm_mullo_epi16(pred_8x16b_2, weight1_8x16b_2);
                pred_8x16b_3 = _mm_mullo_epi16(pred_8x16b_3, weight1_8x16b_3);

                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                temp_2 = _mm_srai_epi16(temp_2, shift);
                temp_3 = _mm_srai_epi16(temp_3, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                _mm_storel_epi64((__m128i *)(p2 + 2 * s_dst), temp_2);
                _mm_storel_epi64((__m128i *)(p2 + 3 * s_dst), temp_3);
                p0 += 4;
                p1 += 4;
                p2 += 4;
                w0 += 4;
                w1 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] * weight0[j + 0 * s_weight] + p1[j + 0 * s_ref] * weight1[j + 0 * s_weight] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] * weight0[j + 1 * s_weight] + p1[j + 1 * s_ref] * weight1[j + 1 * s_weight] + offset) >> shift;
                    p2[j + 2 * s_dst] = (p0[j + 2 * s_src] * weight0[j + 2 * s_weight] + p1[j + 2 * s_ref] * weight1[j + 2 * s_weight] + offset) >> shift;
                    p2[j + 3 * s_dst] = (p0[j + 3 * s_src] * weight0[j + 3 * s_weight] + p1[j + 3 * s_ref] * weight1[j + 3 * s_weight] + offset) >> shift;
                }
            }
        }
    }
}
#endif
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht)
{
    s16 *p0, *p1, *p2;
    int rem_h = ht;
    int rem_w;
    int i, j;
    __m128i src_8x16b, src_8x16b_1, src_8x16b_2, src_8x16b_3;
    __m128i pred_8x16b, pred_8x16b_1, pred_8x16b_2, pred_8x16b_3;
    __m128i temp_0, temp_1, temp_2, temp_3;
    __m128i offset_8x16b;
    /* Can be changed for a generic avg fun. or taken as an argument! */
    short offset = 1;
    int shift = 1;
    p0 = src;
    p1 = ref;
    p2 = dst;
    offset_8x16b = _mm_set1_epi16(offset);
    /* Mult. of 4 Loop */
    if (rem_h >= 4)
    {
        for (i = 0; i < rem_h - 3; i += 4)
        {
            p0 = src + (i * s_src);
            p1 = ref + (i * s_ref);
            p2 = dst + (i * s_dst);
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    src_8x16b_2 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 2)));
                    src_8x16b_3 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 3)));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    pred_8x16b_2 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 2)));
                    pred_8x16b_3 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 3)));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                    temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                    temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    temp_2 = _mm_srai_epi16(temp_2, shift);
                    temp_3 = _mm_srai_epi16(temp_3, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    _mm_storeu_si128((__m128i *)(p2 + 2 * s_dst), temp_2);
                    _mm_storeu_si128((__m128i *)(p2 + 3 * s_dst), temp_3);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                src_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 2)));
                src_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 3)));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                pred_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 2)));
                pred_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 3)));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                temp_2 = _mm_srai_epi16(temp_2, shift);
                temp_3 = _mm_srai_epi16(temp_3, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                _mm_storel_epi64((__m128i *)(p2 + 2 * s_dst), temp_2);
                _mm_storel_epi64((__m128i *)(p2 + 3 * s_dst), temp_3);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                    p2[j + 2 * s_dst] = (p0[j + 2 * s_src] + p1[j + 2 * s_ref] + offset) >> shift;
                    p2[j + 3 * s_dst] = (p0[j + 3 * s_src] + p1[j + 3 * s_ref] + offset) >> shift;
                }
            }
        }
    }
    /* Remaining rows */
    rem_h &= 0x3;
    /* One 2 row case */
    if (rem_h >= 2)
    {
        p0 = src + ((ht >> 2) << 2) * s_src;
        p1 = ref + ((ht >> 2) << 2) * s_ref;
        p2 = dst + ((ht >> 2) << 2) * s_dst;
        /* One 2 row case */
        {
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                }
            }
        }
    }
    /* Remaining 1 row */
    if (rem_h &= 0x1)
    {
        p0 = src + ((ht >> 1) << 1) * s_src;
        p1 = ref + ((ht >> 1) << 1) * s_ref;
        p2 = dst + ((ht >> 1) << 1) * s_dst;
        /* One 1 row case */
        {
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j] = (p0[j] + p1[j] + offset) >> shift;
                }
            }
        }
    }
}

static void mc_filter_l_8pel_horz_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i    mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            int sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = (s16)(sum);
                    dst_copy[col + dst_stride] = (s16)(sum1);
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < (height - 1); row += 2)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
            //if (is_last)
            {
                res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            int sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = (s16)(sum);
                    dst_copy[col + dst_stride] = (s16)(sum1);
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                //if (is_last)
                {
                    sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                }
                dst_copy[col] = (s16)(sum);
            }
        }
    }
}

static void mc_filter_l_8pel_horz_no_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            int sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    dst_copy[col] = (s16)(sum);
                    dst_copy[col + dst_stride] = (s16)(sum1);
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < (height - 1); row += 2)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            int sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    dst_copy[col] = (s16)(sum);
                    dst_copy[col + dst_stride] = (s16)(sum1);
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                dst_copy[col] = (s16)(sum);
            }
        }
    }
}

static void mc_filter_l_8pel_vert_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /*load 8 pixel values.*/
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /*load 8 pixel values*/
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /*load 8 pixel values*/
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));
                /*load 8 pixel values*/
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);
                /*load 8 pixel values*/
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 2) + cnt));
                /*load 8 pixel values*/
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));
                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);
                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);
                /*load 8 pixel values*/
                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 6) + cnt));
                /*load 8 pixel values*/
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 7) + cnt));
                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff6_7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);
                /* i2_tmp = CLIP_U8(i2_tmp);*/
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                //if (is_last)
                {
                    s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            /*load 8 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            /*load 8 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 8 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            /*load 8 pixel values*/
            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));
            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
            /*load 8 pixel values*/
            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * stored_alf_para_num)));
            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s5_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s6_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);
            /*(i2_tmp + OFFSET_14_MINUS_BIT_DEPTH) >> SHIFT_14_MINUS_BIT_DEPTH */
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            /* i2_tmp = CLIP_U8(i2_tmp);*/
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            //if (is_last)
            {
                s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];
                sum += inp_copy[col + 6 * stored_alf_para_num] * coeff[6];
                sum += inp_copy[col + 7 * stored_alf_para_num] * coeff[7];
                val = (sum + offset) >> shift;
                //if (is_last)
                {
                    val = COM_CLIP3(min_val, max_val, val);
                }
                dst_copy[col] = (s16)val;
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

static void mc_filter_l_8pel_vert_no_clip_sse
(
    s16* ref,
    int stored_alf_para_num,
    s16* pred,
    int dst_stride,
    const s16* coeff,
    int width,
    int height,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const* src_tmp;
    s16 const* inp_copy;
    s16* dst_copy;

    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b;

    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */

    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /*load 8 pixel values.*/
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));

                /*load 8 pixel values*/
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));

                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);

                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /*load 8 pixel values*/
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));

                /*load 8 pixel values*/
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));

                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);

                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);

                /*load 8 pixel values*/
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 2) + cnt));

                /*load 8 pixel values*/
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));

                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);

                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);

                /*load 8 pixel values*/
                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 6) + cnt));

                /*load 8 pixel values*/
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 7) + cnt));

                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);

                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff6_7_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);

                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);

                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);

                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);

                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for (row = 0; row < height; row++)
        {
            /*load 8 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));

            /*load 8 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));

            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);

            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 8 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));

            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);

            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            /*load 8 pixel values*/
            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));

            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);

            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
            /*load 8 pixel values*/
            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * stored_alf_para_num)));

            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);

            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);

            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s5_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s6_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);

            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);

            /*(i2_tmp + OFFSET_14_MINUS_BIT_DEPTH) >> SHIFT_14_MINUS_BIT_DEPTH */
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);

            /* i2_tmp = CLIP_U8(i2_tmp);*/
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);

            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);

            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;

                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];
                sum += inp_copy[col + 6 * stored_alf_para_num] * coeff[6];
                sum += inp_copy[col + 7 * stored_alf_para_num] * coeff[7];

                val = (sum + offset) >> shift;

                dst_copy[col] = (s16)val;
            }

            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

static void mc_filter_c_4pel_horz_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h, cnt;
    int src_stride2, src_stride3;
    s16 *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;
    __m128i res0, res1, res2, res3;
    __m128i row11, row12, row13, row14, row21, row22, row23, row24;
    __m128i row31, row32, row33, row34, row41, row42, row43, row44;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    {
        rem_h = height & 0x3;
        rem_w = width;
        coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);  /*w2 w3 w2 w3 w2 w3 w2 w3*/
        coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);        /*w0 w1 w0 w1 w0 w1 w0 w1*/
        /* 8 pixels at a time */
        if (rem_w > 7)
        {
            cnt = 0;
            for (row = 0; row < height; row += 4)
            {
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                    /*load pixel values from row 2*/
                    row21 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                    row22 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 1));
                    row23 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 2));
                    row24 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 3));
                    /*load pixel values from row 3*/
                    row31 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                    row32 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                    row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 2));
                    row34 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 3));
                    /*load pixel values from row 4*/
                    row41 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                    row42 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));
                    row43 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 2));
                    row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 3));
                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/
                    row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                    row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                    row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                    row24 = _mm_madd_epi16(row24, coeff2_3_8x16b);
                    row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                    row32 = _mm_madd_epi16(row32, coeff0_1_8x16b);
                    row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                    row34 = _mm_madd_epi16(row34, coeff2_3_8x16b);
                    row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                    row42 = _mm_madd_epi16(row42, coeff0_1_8x16b);
                    row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);
                    row44 = _mm_madd_epi16(row44, coeff2_3_8x16b);
                    row11 = _mm_add_epi32(row11, row13);
                    row12 = _mm_add_epi32(row12, row14);
                    row21 = _mm_add_epi32(row21, row23);
                    row22 = _mm_add_epi32(row22, row24);
                    row31 = _mm_add_epi32(row31, row33);
                    row32 = _mm_add_epi32(row32, row34);
                    row41 = _mm_add_epi32(row41, row43);
                    row42 = _mm_add_epi32(row42, row44);
                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);
                    row21 = _mm_add_epi32(row21, offset_4x32b);
                    row22 = _mm_add_epi32(row22, offset_4x32b);
                    row31 = _mm_add_epi32(row31, offset_4x32b);
                    row32 = _mm_add_epi32(row32, offset_4x32b);
                    row41 = _mm_add_epi32(row41, offset_4x32b);
                    row42 = _mm_add_epi32(row42, offset_4x32b);
                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);
                    row21 = _mm_srai_epi32(row21, shift);
                    row22 = _mm_srai_epi32(row22, shift);
                    row31 = _mm_srai_epi32(row31, shift);
                    row32 = _mm_srai_epi32(row32, shift);
                    row41 = _mm_srai_epi32(row41, shift);
                    row42 = _mm_srai_epi32(row42, shift);
                    row11 = _mm_packs_epi32(row11, row21);
                    row12 = _mm_packs_epi32(row12, row22);
                    row31 = _mm_packs_epi32(row31, row41);
                    row32 = _mm_packs_epi32(row32, row42);
                    res0 = _mm_unpacklo_epi16(row11, row12);
                    res1 = _mm_unpackhi_epi16(row11, row12);
                    res2 = _mm_unpacklo_epi16(row31, row32);
                    res3 = _mm_unpackhi_epi16(row31, row32);
                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res1, mm_max);
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res2, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res2, mm_max);
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res3, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res3, mm_max);
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride << 1) + cnt), res2);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride * 3) + cnt), res3);
                    cnt += 8;
                }
                cnt = 0;
                inp_copy += (stored_alf_para_num << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }
            /*remaining ht */
            for (row = 0; row < rem_h; row++)
            {
                cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/
                    row11 = _mm_add_epi32(row11, row13); /*a0+a1+a2+a3 a2+a3+a4+a5 a4+a5+a6+a7 a6+a7+a8+a9*/
                    row12 = _mm_add_epi32(row12, row14); /*a1+a2+a3+a4 a3+a4+a5+a6 a5+a6+a7+a8 a7+a8+a9+a10*/
                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);
                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);
                    row11 = _mm_packs_epi32(row11, row12);
                    res3 = _mm_unpackhi_epi64(row11, row11);
                    res0 = _mm_unpacklo_epi16(row11, res3);
                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    cnt += 8;
                }
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        /* one 4 pixel wd for multiple rows */
        if (rem_w > 3)
        {
            inp_copy = ref + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 4)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                /*load pixel values from row 2*/
                row21 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));
                row22 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 1));
                row23 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 2));
                row24 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 3));
                /*load pixel values from row 3*/
                row31 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
                row32 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 1));
                row33 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 2));
                row34 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 3));
                /*load pixel values from row 4*/
                row41 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
                row42 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 1));
                row43 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 2));
                row44 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 3));
                row11 = _mm_unpacklo_epi32(row11, row12);
                row13 = _mm_unpacklo_epi32(row13, row14);
                row21 = _mm_unpacklo_epi32(row21, row22);
                row23 = _mm_unpacklo_epi32(row23, row24);
                row31 = _mm_unpacklo_epi32(row31, row32);
                row33 = _mm_unpacklo_epi32(row33, row34);
                row41 = _mm_unpacklo_epi32(row41, row42);
                row43 = _mm_unpacklo_epi32(row43, row44);
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);
                row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);
                row11 = _mm_add_epi32(row11, row13);
                row21 = _mm_add_epi32(row21, row23);
                row31 = _mm_add_epi32(row31, row33);
                row41 = _mm_add_epi32(row41, row43);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row21 = _mm_add_epi32(row21, offset_4x32b);
                row31 = _mm_add_epi32(row31, offset_4x32b);
                row41 = _mm_add_epi32(row41, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                row21 = _mm_srai_epi32(row21, shift);
                row31 = _mm_srai_epi32(row31, shift);
                row41 = _mm_srai_epi32(row41, shift);
                res0 = _mm_packs_epi32(row11, row21);
                res1 = _mm_packs_epi32(row31, row41);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res0, mm_max);
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res0);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride << 1)), res1);
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride * 3)), _mm_unpackhi_epi64(res1, res1));
                inp_copy += (stored_alf_para_num << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }
            for (row = 0; row < rem_h; row++)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                row11 = _mm_unpacklo_epi32(row11, row12);        /*a0 a1 a1 a2 a2 a3 a3 a4*/
                row13 = _mm_unpacklo_epi32(row13, row14);        /*a2 a3 a3 a4 a4 a5 a5 a6*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a1+a2 a2+a3 a3+a4*/
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a3+a4 a4+a5 a5+a6*/
                row11 = _mm_add_epi32(row11, row13);    /*r00 r01  r02  r03*/
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                res1 = _mm_packs_epi32(row11, row11);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res1);
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = ref + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            for (row = 0; row < height; row++)
            {
                for (col = 0; col < rem_w; col++)
                {
                    int val;
                    int sum;
                    sum = inp_copy[col + 0] * coeff[0];
                    sum += inp_copy[col + 1] * coeff[1];
                    sum += inp_copy[col + 2] * coeff[2];
                    sum += inp_copy[col + 3] * coeff[3];
                    val = (sum + offset) >> shift;
                    dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
                }
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
    }
}

static void mc_filter_c_4pel_vert_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;
    __m128i s0_8x16b, s1_8x16b, s4_8x16b, s5_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_4_16x8b, s3_5_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /* a0 a1 a2 a3 a4 a5 a6 a7 */
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /* b0 b1 b2 b3 b4 b5 b6 b7 */
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                /* a0 b0 a1 b1 a2 b2 a3 b3 */
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a4 b4 ... a7 b7 */
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a0+b0 a1+b1 a2+b2 a3+b3*/
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /* c0 c1 c2 c3 c4 c5 c6 c7 */
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));
                /* d0 d1 d2 d3 d4 d5 d6 d7 */
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                /* c0 d0 c1 d1 c2 d2 c3 d3 */
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                /* c0+d0 c1+d1 c2+d2 c3+d3*/
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);
                /* a0+b0+c0+d0 ... a3+b3+c3+d3 */
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                /* a4+b4+c4+d4 ... a7+b7+c7+d7 */
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            /*load 4 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            /*load 4 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 4 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            /*load 4 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s7_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            if (is_last)
            {
                mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif

void com_mc_l_00(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter
#if DMVR
    , int bDMVR
#endif
)
{
    int i, j;
    if (bHpFilter)
    {
        gmv_x >>= 4;
        gmv_y >>= 4;
    }
    else
    {
        gmv_x >>= 2;
        gmv_y >>= 2;
    }
#if DMVR
    if (!bDMVR)
    {
#endif
        ref += gmv_y * s_ref + gmv_x;
#if DMVR
    }
#endif
#if SIMD_MC
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;
        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));
                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i m00;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void com_mc_l_n0(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter
#if DMVR
    , int bDMVR
#endif
)
{
    int dx;
    if (bHpFilter)
    {
        dx = gmv_x & 15;
#if DMVR
        if (bDMVR)
        {
            ref = ref - 3;
        }
        else
        {
#endif
            ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4) - 3;
#if DMVR
        }
#endif
    }
    else
    {
        dx = gmv_x & 0x3;
#if DMVR
        if (bDMVR)
        {
            ref = ref - 3;
        }
        else
        {
#endif
            ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2) - 3;
#if DMVR
        }
#endif
    }
    const s16 *coeff_hor = bHpFilter ? tbl_mc_l_coeff_hp[dx] : tbl_mc_l_coeff[dx];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_horz_clip_sse(ref, s_ref, pred, s_pred, coeff_hor, w, h, min, max,
                                       MAC_ADD_N0, MAC_SFT_N0);
    }
#else
    {
        int i, j;
        s32 pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_hor, ref[j], ref[j + 1], ref[j + 2], ref[j + 3],    ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + MAC_ADD_N0) >> MAC_SFT_N0;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref  += s_ref;
            pred += s_pred;
        }
    }
#endif
}

void com_mc_l_0n(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter
#if DMVR
    , int bDMVR
#endif
)
{
    int dy;

    if (bHpFilter)
    {
        dy = gmv_y & 15;
#if DMVR
        if (bDMVR)
        {
            ref = ref - (3 * s_ref);
        }
        else
        {
#endif
            ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4);
#if DMVR
        }
#endif
    }
    else
    {
        dy = gmv_y & 0x3;
#if DMVR
        if (bDMVR)
        {
            ref = ref - (3 * s_ref);
        }
        else
        {
#endif
            ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2);
#if DMVR
        }
#endif
    }
    const s16 *coeff_ver = bHpFilter ? tbl_mc_l_coeff_hp[dy] : tbl_mc_l_coeff[dy];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_vert_clip_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max,
                                       MAC_ADD_0N, MAC_SFT_0N);
    }
#else
    {
        int i, j;
        s32 pt;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_ver, ref[j], ref[s_ref + j], ref[s_ref*2 + j], ref[s_ref*3 + j], ref[s_ref*4 + j], ref[s_ref*5 + j], ref[s_ref*6 + j], ref[s_ref*7 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref  += s_ref;
            pred += s_pred;
        }
    }
#endif
}

void com_mc_l_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int bHpFilter
#if DMVR
    , int bDMVR
#endif
)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*MAX_CU_SIZE];
    int         dx, dy;

    if (bHpFilter)
    {
        dx = gmv_x & 15;
        dy = gmv_y & 15;
#if DMVR
        if (bDMVR)
        {
            ref = ref - (3 * s_ref + 3);
        }
        else
        {
#endif
            ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4) - 3;
#if DMVR
        }
#endif
    }
    else
    {
        dx = gmv_x & 0x3;
        dy = gmv_y & 0x3;
#if DMVR
        if (bDMVR)
        {
            ref = ref - (3 * s_ref + 3);
        }
        else
        {
#endif
            ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
#if DMVR
        }
#endif
    }
    const s16 *coeff_hor = bHpFilter ? tbl_mc_l_coeff_hp[dx] : tbl_mc_l_coeff[dx];
    const s16 *coeff_ver = bHpFilter ? tbl_mc_l_coeff_hp[dy] : tbl_mc_l_coeff[dy];

    const int shift1 = bit_depth - 8;
    const int shift2 = 20 - bit_depth;
    const int add1 = (1 << shift1) >> 1;
    const int add2 = 1 << (shift2 - 1);
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 7), add1, shift1);
        mc_filter_l_8pel_vert_clip_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2);
    }
#else
    {
        int i, j;
        s16 * b;
        s32 pt;
        b = buf;

        for(i=0; i<h+7; i++)
        {
            for(j=0; j<w; j++)
            {
                b[j] = (MAC_8TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3], ref[j+4], ref[j+5], ref[j+6], ref[j+7]) + add1) >> shift1;
            }
            ref  += s_ref;
            b     += w;
        }
        b = buf;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_ver, b[j], b[j+w], b[j+w*2], b[j+w*3], b[j+w*4], b[j+w*5], b[j+w*6], b[j+w*7]) + add2) >> shift2;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
}

/****************************************************************************
 * motion compensation for chroma
 ****************************************************************************/

void com_mc_c_00(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int bHPFilter
#if USE_IBC
    , int bIBC
#endif
#if DMVR
    , int bDMVR
#endif
)
{
    int i, j;
    if (bHPFilter)
    {
        gmv_x >>= 5;
        gmv_y >>= 5;
    }
    else
    {
        gmv_x >>= 3;
        gmv_y >>= 3;
    }
#if DMVR
    if (!bDMVR)
    {
#endif
        ref += gmv_y * s_ref + gmv_x;
#if DMVR
    }
#endif
#if SIMD_MC
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;
        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));
                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if(((w & 0x3)==0))
    {
        __m128i m00;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void com_mc_c_n0(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int bHPFilter
#if USE_IBC
    , int bIBC
#endif
#if DMVR
    , int bDMVR
#endif
)
{
    int dx;
    if (bHPFilter)
    {
        dx = gmv_x & 31;
#if DMVR
        if( bDMVR )
            ref -= 1;
        else
#endif
        ref += (gmv_y >> 5) * s_ref + (gmv_x >> 5) - 1;
    }
    else
    {
        dx = gmv_x & 0x7;
#if DMVR
        if( bDMVR )
            ref -= 1;
        else
#endif
        ref += (gmv_y >> 3) * s_ref + (gmv_x >> 3) - 1;
    }
#if USE_IBC
#if DMVR
    const s16 *coeff_hor = bIBC? tbl_mc_c_coeff_ibc[dx] : ((bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
#else
    const s16 *coeff_hor = bIBC? tbl_mc_c_coeff_ibc[dx] : (bHPFilter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
#endif
#else
#if DMVR
    const s16 *coeff_hor = (bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
#else
    const s16 *coeff_hor = bHPFilter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
#endif
#endif

#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_horz_sse(ref, s_ref, pred, s_pred, coeff_hor,
                                  w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
    }
#else
    {
        int       i, j;
        s32       pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + MAC_ADD_N0) >> MAC_SFT_N0;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
}

void com_mc_c_0n(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int bHPFilter
#if USE_IBC
    , int bIBC
#endif
#if DMVR
    , int bDMVR
#endif
)
{
    int dy;

    if (bHPFilter)
    {
        dy = gmv_y & 31;
#if DMVR
        if( bDMVR )
            ref -= 1 * s_ref;
        else
#endif
        ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5);
    }
    else
    {
        dy = gmv_y & 0x7;
#if DMVR
        if( bDMVR )
            ref -= 1 * s_ref;
        else
#endif
        ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3);
    }
#if USE_IBC
#if DMVR
    const s16 *coeff_ver = bIBC? tbl_mc_c_coeff_ibc[dy]: ((bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#else
    const s16 *coeff_ver = bIBC? tbl_mc_c_coeff_ibc[dy]: (bHPFilter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
#if DMVR
    const s16 *coeff_ver = (bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#else
    const s16 *coeff_ver = bHPFilter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#endif
#endif

#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_vert_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
    }
#else
    {
        int i, j;
        s32 pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_ver, ref[j], ref[s_ref + j], ref[s_ref*2 + j], ref[s_ref*3 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
}

void com_mc_c_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int bHPFilter
#if USE_IBC
    , int bIBC
#endif
#if DMVR
    , int bDMVR
#endif
)
{
#if DMVR
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C + 16)*(MAX_CU_SIZE + MC_IBUF_PAD_C + 16)];
#else
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C)*MAX_CU_SIZE];
#endif
    int         dx, dy;

    if (bHPFilter)
    {
        dx = gmv_x & 31;
        dy = gmv_y & 31;
#if DMVR
        if( bDMVR )
            ref -= (1 * s_ref + 1);
        else
#endif
        ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5) - 1;
    }
    else
    {
        dx = gmv_x & 0x7;
        dy = gmv_y & 0x7;
#if DMVR
        if( bDMVR )
            ref -= (1 * s_ref + 1);
        else
#endif
        ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3) - 1;
    }
#if USE_IBC
#if DMVR
    const s16 *coeff_hor = bIBC ? tbl_mc_c_coeff_ibc[dx] : ((bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
    const s16 *coeff_ver = bIBC ? tbl_mc_c_coeff_ibc[dy] : ((bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#else
    const s16 *coeff_hor = bIBC ? tbl_mc_c_coeff_ibc[dx] : (bHPFilter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
    const s16 *coeff_ver = bIBC ? tbl_mc_c_coeff_ibc[dy] : (bHPFilter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
#if DMVR
    const s16 *coeff_hor = (bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
    const s16 *coeff_ver = (bHPFilter && !bDMVR) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#else
    const s16 *coeff_hor = bHPFilter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
    const s16 *coeff_ver = bHPFilter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#endif
#endif

    const int shift1 = bit_depth - 8;
    const int shift2 = 20 - bit_depth;
    const int add1 = (1 << shift1) >> 1;
    const int add2 = 1 << (shift2 - 1);
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_horz_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 3), min, max, add1, shift1, 0);
        mc_filter_c_4pel_vert_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2, 1);
    }
#else
    {
        s16        *b;
        int         i, j;
        s32         pt;
        b = buf;
        for(i=0; i<h+3; i++)
        {
            for(j=0; j<w; j++)
            {
                b[j] = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + add1) >> shift1;
            }
            ref  += s_ref;
            b     += w;
        }
        b = buf;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_ver, b[j], b[j+w], b[j+2*w], b[j+3*w]) + add2) >> shift2;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
}

COM_MC_L com_tbl_mc_l[2][2] =
{
    {
        com_mc_l_00, /* dx == 0 && dy == 0 */
        com_mc_l_0n  /* dx == 0 && dy != 0 */
    },
    {
        com_mc_l_n0, /* dx != 0 && dy == 0 */
        com_mc_l_nn  /* dx != 0 && dy != 0 */
    }
};

COM_MC_C com_tbl_mc_c[2][2] =
{
    {
        com_mc_c_00, /* dx == 0 && dy == 0 */
        com_mc_c_0n  /* dx == 0 && dy != 0 */
    },
    {
        com_mc_c_n0, /* dx != 0 && dy == 0 */
        com_mc_c_nn  /* dx != 0 && dy != 0 */
    }
};


#if BIO
/****************************************************************************
* BIO motion compensation for luma
****************************************************************************/

s16 grad_x[2][MAX_CU_SIZE*MAX_CU_SIZE];
s16 grad_y[2][MAX_CU_SIZE*MAX_CU_SIZE];
s32 sigma[5][(MAX_CU_SIZE + 2 * BIO_WINDOW_SIZE) *(MAX_CU_SIZE + 2 * BIO_WINDOW_SIZE)];

/****************************************************************************
* 8 taps gradient
****************************************************************************/

#if SIMD_MC
static const s16 tbl_bio_grad_l_coeff[4][8] =
#else
static const s8 tbl_bio_grad_l_coeff[4][8] =
#endif
{
    { -4, 11, -39, -1, 41, -14, 8, -2, },
    { -2, 6, -19, -31, 53, -12, 7, -2, },
    { 0, -1, 0, -50, 50, 0, 1, 0, },
    { 2, -7, 12, -53, 31, 19, -6, 2, }
};


static void com_grad_x_l_nn(s16* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16* pred, int w, int h, int bit_depth, int bDMVR)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L) * MAX_CU_SIZE];
    s16* b;
    int         i, j, dx, dy;
    s32         pt;

    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;

    if (bDMVR)
    {
        ref = ref - (3 * s_ref + 3);
    }
    else
    {
        ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
    }

#if SIMD_MC
    if (1)
    {
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_bio_grad_l_coeff[dx], w, (h + 7),
            BIO_IF_O1(bit_depth), BIO_IF_S1(bit_depth));

        mc_filter_l_8pel_vert_no_clip_sse(buf, w, pred, s_pred, tbl_mc_l_coeff[dy], w, h,
            BIO_IF_O2(bit_depth), BIO_IF_S2(bit_depth));
    }
    else
#endif
    {
        b = buf;
        for (i = 0; i < h + 7; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON X */
                b[j] = ((MAC_8TAP(tbl_bio_grad_l_coeff[dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + BIO_IF_O1(bit_depth)) >> BIO_IF_S1(bit_depth));
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON Y */
                pt = ((MAC_8TAP(tbl_mc_l_coeff[dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]) + BIO_IF_O2(bit_depth)) >> BIO_IF_S2(bit_depth));
                /* NO CLIP */
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
}

static void com_grad_y_l_nn(s16* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16* pred, int w, int h, int bit_depth, int bDMVR)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L) * MAX_CU_SIZE];
    s16* b;
    int         i, j, dx, dy;
    s32         pt;

    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;

    if (bDMVR)
    {
        ref = ref - (3 * s_ref + 3);
    }
    else
    {
        ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
    }

#if SIMD_MC
    if (1)
    {
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_mc_l_coeff[dx], w, (h + 7),
            BIO_IF_O1(bit_depth), BIO_IF_S1(bit_depth));

        mc_filter_l_8pel_vert_no_clip_sse(buf, w, pred, s_pred, tbl_bio_grad_l_coeff[dy], w, h,
            BIO_IF_O2(bit_depth), BIO_IF_S2(bit_depth));
    }
    else
#endif
    {
        b = buf;
        for (i = 0; i < h + 7; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON X */
                b[j] = ((MAC_8TAP(tbl_mc_l_coeff[dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + BIO_IF_O1(bit_depth)) >> BIO_IF_S1(bit_depth));
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON Y */
                pt = ((MAC_8TAP(tbl_bio_grad_l_coeff[dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]) + BIO_IF_O2(bit_depth)) >> BIO_IF_S2(bit_depth));
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
}

void bio_vxvy(s16* vx, s16* vy, int wb, s32* s1, s32* s2, s32* s3, s32* s5, s32* s6, int avg_size)
{
    int ii, jj;
    s32 s1a = 0; s32* s1t = s1;
    s32 s2a = 0; s32* s2t = s2;
    s32 s3a = 0; s32* s3t = s3;
    s32 s5a = 0; s32* s5t = s5;
    s32 s6a = 0; s32* s6t = s6;

#if SIMD_MC
    __m128i vzero = _mm_setzero_si128();
    __m128i mmS1a = _mm_setzero_si128();
    __m128i mmS2a = _mm_setzero_si128();
    __m128i mmS3a = _mm_setzero_si128();
    __m128i mmS5a = _mm_setzero_si128();
    __m128i mmS6a = _mm_setzero_si128();

    __m128i mmS1;
    __m128i mmS2;
    __m128i mmS3;
    __m128i mmS5;
    __m128i mmS6;
    for (jj = 0; jj < avg_size; jj++)
    {
        ii = 0;
        for (; ii < ((avg_size >> 2) << 2); ii += 4)
        {
            mmS1 = _mm_loadu_si128((__m128i*)(s1t + ii));
            mmS2 = _mm_loadu_si128((__m128i*)(s2t + ii));
            mmS3 = _mm_loadu_si128((__m128i*)(s3t + ii));
            mmS5 = _mm_loadu_si128((__m128i*)(s5t + ii));
            mmS6 = _mm_loadu_si128((__m128i*)(s6t + ii));

            mmS1a = _mm_add_epi32(mmS1a, mmS1);
            mmS2a = _mm_add_epi32(mmS2a, mmS2);
            mmS3a = _mm_add_epi32(mmS3a, mmS3);
            mmS5a = _mm_add_epi32(mmS5a, mmS5);
            mmS6a = _mm_add_epi32(mmS6a, mmS6);
        }
        for (; ii < ((avg_size >> 1) << 1); ii += 2)
        {
            mmS1 = _mm_loadl_epi64((__m128i*)(s1t + ii));
            mmS2 = _mm_loadl_epi64((__m128i*)(s2t + ii));
            mmS3 = _mm_loadl_epi64((__m128i*)(s3t + ii));
            mmS5 = _mm_loadl_epi64((__m128i*)(s5t + ii));
            mmS6 = _mm_loadl_epi64((__m128i*)(s6t + ii));

            mmS1a = _mm_add_epi32(mmS1a, mmS1);
            mmS2a = _mm_add_epi32(mmS2a, mmS2);
            mmS3a = _mm_add_epi32(mmS3a, mmS3);
            mmS5a = _mm_add_epi32(mmS5a, mmS5);
            mmS6a = _mm_add_epi32(mmS6a, mmS6);
        }
        s1t += wb; s2t += wb; s3t += wb; s5t += wb; s6t += wb;
    }

    mmS1a = _mm_hadd_epi32(_mm_hadd_epi32(mmS1a, vzero), vzero);
    mmS2a = _mm_hadd_epi32(_mm_hadd_epi32(mmS2a, vzero), vzero);
    mmS3a = _mm_hadd_epi32(_mm_hadd_epi32(mmS3a, vzero), vzero);
    mmS5a = _mm_hadd_epi32(_mm_hadd_epi32(mmS5a, vzero), vzero);
    mmS6a = _mm_hadd_epi32(_mm_hadd_epi32(mmS6a, vzero), vzero);

    s1a = _mm_cvtsi128_si32(mmS1a);
    s2a = _mm_cvtsi128_si32(mmS2a);
    s3a = _mm_cvtsi128_si32(mmS3a);
    s5a = _mm_cvtsi128_si32(mmS5a);
    s6a = _mm_cvtsi128_si32(mmS6a);
#else
    for (jj = 0; jj < avg_size; jj++)
    {
        for (ii = 0; ii < avg_size; ii++)
        {
            s1a += s1t[ii];
            s2a += s2t[ii];
            s3a += s3t[ii];
            s5a += s5t[ii];
            s6a += s6t[ii];
        }
        s1t += wb; s2t += wb; s3t += wb; s5t += wb; s6t += wb;
    }
#endif

    *vx = 0; *vy = 0;
    if (s1a > DENOMBIO)
    {
        *vx = divide_tbl((s3a << 5), (s1a + DENOMBIO));
        *vx = COM_CLIP3(-LIMITBIO, LIMITBIO, *vx);
    }
    if (s5a > DENOMBIO)
    {
        *vy = divide_tbl(((s6a << 6) - *vx*s2a), ((s5a + DENOMBIO) << 1));
        *vy = COM_CLIP3(-LIMITBIO, LIMITBIO, *vy);
    }
}

static void bio_block_average_16b_clip_sse(s16 *p0t, s16 *p1t,
    s16 *gx0t, s16 *gy0t, s16 *gx1t, s16 *gy1t,
    s16 vx, s16 vy,
    int bio_cluster_size, int s_gt, int bit_depth)
{
    int i, j, grad_on = 1;
    int s_gt_3 = s_gt * 3;

    __m128i gx0t_8x16b_0, gx0t_8x16b_1, gx0t_8x16b_2, gx0t_8x16b_3;
    __m128i gx1t_8x16b_0, gx1t_8x16b_1, gx1t_8x16b_2, gx1t_8x16b_3;
    __m128i gy0t_8x16b_0, gy0t_8x16b_1, gy0t_8x16b_2, gy0t_8x16b_3;
    __m128i gy1t_8x16b_0, gy1t_8x16b_1, gy1t_8x16b_2, gy1t_8x16b_3;

    __m128i p0t_8x16b_0, p0t_8x16b_1, p0t_8x16b_2, p0t_8x16b_3;
    __m128i p1t_8x16b_0, p1t_8x16b_1, p1t_8x16b_2, p1t_8x16b_3;

    __m128i vx_4x32b = _mm_set1_epi32(vx);
    __m128i vy_4x32b = _mm_set1_epi32(vy);

    __m128i offset0_4x32b = _mm_set1_epi32(32);
    __m128i offset1_4x32b = _mm_set1_epi32(1);
    __m128i max_4x32b = _mm_set1_epi32((1 << bit_depth) - 1);
    __m128i min_4x32b = _mm_setzero_si128();

    /* Should be multiple of 4 */
    assert(!(bio_cluster_size & 0x3));

    if ((vx == 0) && (vy == 0))
        grad_on = 0;

    for (i = 0; i < bio_cluster_size; i += 4)
    {
        for (j = 0; j < bio_cluster_size; j += 4)
        {
            if (grad_on)
            {
                gx0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gx0t));
                gx0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gx0t + s_gt));

                gx1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gx1t));
                gx1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gx1t + s_gt));

                gy0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gy0t));
                gy0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gy0t + s_gt));

                gy1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gy1t));
                gy1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gy1t + s_gt));

                gx0t_8x16b_0 = _mm_cvtepi16_epi32(gx0t_8x16b_0);
                gx0t_8x16b_1 = _mm_cvtepi16_epi32(gx0t_8x16b_1);

                gx1t_8x16b_0 = _mm_cvtepi16_epi32(gx1t_8x16b_0);
                gx1t_8x16b_1 = _mm_cvtepi16_epi32(gx1t_8x16b_1);

                gy0t_8x16b_0 = _mm_cvtepi16_epi32(gy0t_8x16b_0);
                gy0t_8x16b_1 = _mm_cvtepi16_epi32(gy0t_8x16b_1);

                gy1t_8x16b_0 = _mm_cvtepi16_epi32(gy1t_8x16b_0);
                gy1t_8x16b_1 = _mm_cvtepi16_epi32(gy1t_8x16b_1);
                /*gx0t[ii] - gx1t[ii]*/
                gx0t_8x16b_0 = _mm_sub_epi32(gx0t_8x16b_0, gx1t_8x16b_0);
                gx0t_8x16b_1 = _mm_sub_epi32(gx0t_8x16b_1, gx1t_8x16b_1);
                /*gy0t[ii] - gy1t[ii]*/
                gy0t_8x16b_0 = _mm_sub_epi32(gy0t_8x16b_0, gy1t_8x16b_0);
                gy0t_8x16b_1 = _mm_sub_epi32(gy0t_8x16b_1, gy1t_8x16b_1);
                /*vx * (gx0t[ii] - gx1t[ii])*/
                gx0t_8x16b_0 = _mm_mullo_epi32(gx0t_8x16b_0, vx_4x32b);
                gx0t_8x16b_1 = _mm_mullo_epi32(gx0t_8x16b_1, vx_4x32b);
                /*vy * (gy0t[ii] - gy1t[ii])*/
                gy0t_8x16b_0 = _mm_mullo_epi32(gy0t_8x16b_0, vy_4x32b);
                gy0t_8x16b_1 = _mm_mullo_epi32(gy0t_8x16b_1, vy_4x32b);

                gx0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gx0t + (s_gt << 1)));
                gx0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gx0t + s_gt_3));

                gx1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gx1t + (s_gt << 1)));
                gx1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gx1t + s_gt_3));

                gy0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gy0t + (s_gt << 1)));
                gy0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gy0t + s_gt_3));

                gy1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gy1t + (s_gt << 1)));
                gy1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gy1t + s_gt_3));

                gx0t_8x16b_2 = _mm_cvtepi16_epi32(gx0t_8x16b_2);
                gx0t_8x16b_3 = _mm_cvtepi16_epi32(gx0t_8x16b_3);

                gx1t_8x16b_2 = _mm_cvtepi16_epi32(gx1t_8x16b_2);
                gx1t_8x16b_3 = _mm_cvtepi16_epi32(gx1t_8x16b_3);

                gy0t_8x16b_2 = _mm_cvtepi16_epi32(gy0t_8x16b_2);
                gy0t_8x16b_3 = _mm_cvtepi16_epi32(gy0t_8x16b_3);

                gy1t_8x16b_2 = _mm_cvtepi16_epi32(gy1t_8x16b_2);
                gy1t_8x16b_3 = _mm_cvtepi16_epi32(gy1t_8x16b_3);

                gx0t_8x16b_2 = _mm_sub_epi32(gx0t_8x16b_2, gx1t_8x16b_2);
                gx0t_8x16b_3 = _mm_sub_epi32(gx0t_8x16b_3, gx1t_8x16b_3);

                gy0t_8x16b_2 = _mm_sub_epi32(gy0t_8x16b_2, gy1t_8x16b_2);
                gy0t_8x16b_3 = _mm_sub_epi32(gy0t_8x16b_3, gy1t_8x16b_3);

                gx0t_8x16b_2 = _mm_mullo_epi32(gx0t_8x16b_2, vx_4x32b);
                gx0t_8x16b_3 = _mm_mullo_epi32(gx0t_8x16b_3, vx_4x32b);

                gy0t_8x16b_2 = _mm_mullo_epi32(gy0t_8x16b_2, vy_4x32b);
                gy0t_8x16b_3 = _mm_mullo_epi32(gy0t_8x16b_3, vy_4x32b);
                /*b = vx * (gx0t[ii] - gx1t[ii]) + vy * (gy0t[ii] - gy1t[ii])*/
                gx0t_8x16b_0 = _mm_add_epi32(gx0t_8x16b_0, gy0t_8x16b_0);
                gx0t_8x16b_1 = _mm_add_epi32(gx0t_8x16b_1, gy0t_8x16b_1);
                gx0t_8x16b_2 = _mm_add_epi32(gx0t_8x16b_2, gy0t_8x16b_2);
                gx0t_8x16b_3 = _mm_add_epi32(gx0t_8x16b_3, gy0t_8x16b_3);
                /*b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));*/
                gy0t_8x16b_0 = _mm_abs_epi32(gx0t_8x16b_0);
                gy0t_8x16b_1 = _mm_abs_epi32(gx0t_8x16b_1);
                gy0t_8x16b_2 = _mm_abs_epi32(gx0t_8x16b_2);
                gy0t_8x16b_3 = _mm_abs_epi32(gx0t_8x16b_3);

                gy0t_8x16b_0 = _mm_add_epi32(gy0t_8x16b_0, offset0_4x32b);
                gy0t_8x16b_1 = _mm_add_epi32(gy0t_8x16b_1, offset0_4x32b);
                gy0t_8x16b_2 = _mm_add_epi32(gy0t_8x16b_2, offset0_4x32b);
                gy0t_8x16b_3 = _mm_add_epi32(gy0t_8x16b_3, offset0_4x32b);

                gy0t_8x16b_0 = _mm_srli_epi32(gy0t_8x16b_0, 6);
                gy0t_8x16b_1 = _mm_srli_epi32(gy0t_8x16b_1, 6);
                gy0t_8x16b_2 = _mm_srli_epi32(gy0t_8x16b_2, 6);
                gy0t_8x16b_3 = _mm_srli_epi32(gy0t_8x16b_3, 6);

                gx0t_8x16b_0 = _mm_sign_epi32(gy0t_8x16b_0, gx0t_8x16b_0);
                gx0t_8x16b_1 = _mm_sign_epi32(gy0t_8x16b_1, gx0t_8x16b_1);
                gx0t_8x16b_2 = _mm_sign_epi32(gy0t_8x16b_2, gx0t_8x16b_2);
                gx0t_8x16b_3 = _mm_sign_epi32(gy0t_8x16b_3, gx0t_8x16b_3);

                /*b + 1*/
                gx0t_8x16b_0 = _mm_add_epi32(gx0t_8x16b_0, offset1_4x32b);
                gx0t_8x16b_1 = _mm_add_epi32(gx0t_8x16b_1, offset1_4x32b);
                gx0t_8x16b_2 = _mm_add_epi32(gx0t_8x16b_2, offset1_4x32b);
                gx0t_8x16b_3 = _mm_add_epi32(gx0t_8x16b_3, offset1_4x32b);
            }
            else
            {
                /*b + 1*/
                gx0t_8x16b_0 = offset1_4x32b;
                gx0t_8x16b_1 = offset1_4x32b;
                gx0t_8x16b_2 = offset1_4x32b;
                gx0t_8x16b_3 = offset1_4x32b;
            }

            /*p0t[ii]*/
            p0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (p0t));
            p0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0t + s_gt));
            p0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0t + (s_gt << 1)));
            p0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0t + s_gt_3));
            /*p1t[ii]*/
            p1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (p1t));
            p1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1t + s_gt));
            p1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1t + (s_gt << 1)));
            p1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1t + s_gt_3));

            p0t_8x16b_0 = _mm_cvtepi16_epi32(p0t_8x16b_0);
            p0t_8x16b_1 = _mm_cvtepi16_epi32(p0t_8x16b_1);
            p0t_8x16b_2 = _mm_cvtepi16_epi32(p0t_8x16b_2);
            p0t_8x16b_3 = _mm_cvtepi16_epi32(p0t_8x16b_3);

            p1t_8x16b_0 = _mm_cvtepi16_epi32(p1t_8x16b_0);
            p1t_8x16b_1 = _mm_cvtepi16_epi32(p1t_8x16b_1);
            p1t_8x16b_2 = _mm_cvtepi16_epi32(p1t_8x16b_2);
            p1t_8x16b_3 = _mm_cvtepi16_epi32(p1t_8x16b_3);

            /*p0t[ii] + p1t[ii]*/
            p0t_8x16b_0 = _mm_add_epi32(p0t_8x16b_0, p1t_8x16b_0);
            p0t_8x16b_1 = _mm_add_epi32(p0t_8x16b_1, p1t_8x16b_1);
            p0t_8x16b_2 = _mm_add_epi32(p0t_8x16b_2, p1t_8x16b_2);
            p0t_8x16b_3 = _mm_add_epi32(p0t_8x16b_3, p1t_8x16b_3);
            /*(p0t[ii] + p1t[ii] + b + 1)*/
            p0t_8x16b_0 = _mm_add_epi32(p0t_8x16b_0, gx0t_8x16b_0);
            p0t_8x16b_1 = _mm_add_epi32(p0t_8x16b_1, gx0t_8x16b_1);
            p0t_8x16b_2 = _mm_add_epi32(p0t_8x16b_2, gx0t_8x16b_2);
            p0t_8x16b_3 = _mm_add_epi32(p0t_8x16b_3, gx0t_8x16b_3);
            /*(p0t[ii] + p1t[ii] + b + 1) >> 1*/
            p0t_8x16b_0 = _mm_srai_epi32(p0t_8x16b_0, 1);
            p0t_8x16b_1 = _mm_srai_epi32(p0t_8x16b_1, 1);
            p0t_8x16b_2 = _mm_srai_epi32(p0t_8x16b_2, 1);
            p0t_8x16b_3 = _mm_srai_epi32(p0t_8x16b_3, 1);
            /*COM_CLIP3(0, (1 << BIT_DEPTH) - 1, (s16)((p0t[ii] + p1t[ii] + b + 1) >> 1))*/
            p0t_8x16b_0 = _mm_min_epi32(p0t_8x16b_0, max_4x32b);
            p0t_8x16b_1 = _mm_min_epi32(p0t_8x16b_1, max_4x32b);
            p0t_8x16b_2 = _mm_min_epi32(p0t_8x16b_2, max_4x32b);
            p0t_8x16b_3 = _mm_min_epi32(p0t_8x16b_3, max_4x32b);

            p0t_8x16b_0 = _mm_max_epi32(p0t_8x16b_0, min_4x32b);
            p0t_8x16b_1 = _mm_max_epi32(p0t_8x16b_1, min_4x32b);
            p0t_8x16b_2 = _mm_max_epi32(p0t_8x16b_2, min_4x32b);
            p0t_8x16b_3 = _mm_max_epi32(p0t_8x16b_3, min_4x32b);

            p0t_8x16b_0 = _mm_packs_epi32(p0t_8x16b_0, p0t_8x16b_0);
            p0t_8x16b_1 = _mm_packs_epi32(p0t_8x16b_1, p0t_8x16b_1);
            p0t_8x16b_2 = _mm_packs_epi32(p0t_8x16b_2, p0t_8x16b_2);
            p0t_8x16b_3 = _mm_packs_epi32(p0t_8x16b_3, p0t_8x16b_3);

            _mm_storel_epi64((__m128i *) (p0t), p0t_8x16b_0);
            _mm_storel_epi64((__m128i *) (p0t + s_gt), p0t_8x16b_1);
            _mm_storel_epi64((__m128i *) (p0t + (s_gt << 1)), p0t_8x16b_2);
            _mm_storel_epi64((__m128i *) (p0t + s_gt_3), p0t_8x16b_3);

            p0t += 4; p1t += 4;  gx0t += 4; gy0t += 4; gx1t += 4; gy1t += 4;
        }

        p0t = p0t - bio_cluster_size + 4 * s_gt;
        p1t = p1t - bio_cluster_size + 4 * s_gt;
        gx0t = gx0t - bio_cluster_size + 4 * s_gt;
        gy0t = gy0t - bio_cluster_size + 4 * s_gt;
        gx1t = gx1t - bio_cluster_size + 4 * s_gt;
        gy1t = gy1t - bio_cluster_size + 4 * s_gt;
    }
}

void bio_sigma(const pel* p0, const pel* p1, const pel* gx0, const pel* gx1, const pel* gy0, const pel* gy1, int* s1, int* s2, int* s3, int* s5, int* s6, const int w, const int h, const int winSize, const int wb)
{
    int* s1t = s1 + wb * winSize;
    int* s2t = s2 + wb * winSize;
    int* s3t = s3 + wb * winSize;
    int* s5t = s5 + wb * winSize;
    int* s6t = s6 + wb * winSize;

    int y = 0;
    for (; y < h; y++)
    {
        int x = 0;

        for (; x < ((w >> 3) << 3); x += 8)
        {
            __m128i mm_p0 = _mm_loadu_si128((__m128i*)(p0 + x));
            __m128i mm_p1 = _mm_loadu_si128((__m128i*)(p1 + x));
            __m128i mm_gx0 = _mm_loadu_si128((__m128i*)(gx0 + x));
            __m128i mm_gx1 = _mm_loadu_si128((__m128i*)(gx1 + x));
            __m128i mm_gy0 = _mm_loadu_si128((__m128i*)(gy0 + x));
            __m128i mm_gy1 = _mm_loadu_si128((__m128i*)(gy1 + x));

            __m128i mm_t = _mm_sub_epi16(mm_p1, mm_p0);
            __m128i mm_tx = _mm_add_epi16(mm_gx0, mm_gx1);
            __m128i mm_ty = _mm_add_epi16(mm_gy0, mm_gy1);

            // s1t
            __m128i mm_b = _mm_mulhi_epi16(mm_tx, mm_tx);
            __m128i mm_a = _mm_mullo_epi16(mm_tx, mm_tx);

            __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            __m128i mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s1t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s1t + x + 4), mm_h);

            // s2t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_ty);
            mm_a = _mm_mullo_epi16(mm_tx, mm_ty);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s2t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s2t + x + 4), mm_h);

            // s3t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_t);
            mm_a = _mm_mullo_epi16(mm_tx, mm_t);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s3t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s3t + x + 4), mm_h);

            // s5t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_ty);
            mm_a = _mm_mullo_epi16(mm_ty, mm_ty);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s5t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s5t + x + 4), mm_h);

            // s6t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_t);
            mm_a = _mm_mullo_epi16(mm_ty, mm_t);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s6t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s6t + x + 4), mm_h);
        }

        for (; x < ((w >> 2) << 2); x += 4)
        {
            __m128i mm_p0 = _mm_loadl_epi64((__m128i*)(p0 + x));
            __m128i mm_p1 = _mm_loadl_epi64((__m128i*)(p1 + x));
            __m128i mm_gx0 = _mm_loadl_epi64((__m128i*)(gx0 + x));
            __m128i mm_gx1 = _mm_loadl_epi64((__m128i*)(gx1 + x));
            __m128i mm_gy0 = _mm_loadl_epi64((__m128i*)(gy0 + x));
            __m128i mm_gy1 = _mm_loadl_epi64((__m128i*)(gy1 + x));

            __m128i mm_t = _mm_sub_epi16(mm_p1, mm_p0);
            __m128i mm_tx = _mm_add_epi16(mm_gx0, mm_gx1);
            __m128i mm_ty = _mm_add_epi16(mm_gy0, mm_gy1);

            // s1t
            __m128i mm_b = _mm_mulhi_epi16(mm_tx, mm_tx);
            __m128i mm_a = _mm_mullo_epi16(mm_tx, mm_tx);
            __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s1t + x), mm_l);

            // s2t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_ty);
            mm_a = _mm_mullo_epi16(mm_tx, mm_ty);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s2t + x), mm_l);

            // s3t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_t);
            mm_a = _mm_mullo_epi16(mm_tx, mm_t);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s3t + x), mm_l);

            // s5t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_ty);
            mm_a = _mm_mullo_epi16(mm_ty, mm_ty);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s5t + x), mm_l);

            // s6t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_t);
            mm_a = _mm_mullo_epi16(mm_ty, mm_t);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s6t + x), mm_l);
        }

        for (; x < w; x++)
        {
            int t = (p0[x]) - (p1[x]);
            int tx = (gx0[x] + gx1[x]);
            int ty = (gy0[x] + gy1[x]);
            s1t[x] = tx * tx;
            s2t[x] = tx * ty;
            s3t[x] = -tx * t;
            s5t[x] = ty * ty;
            s6t[x] = -ty * t;
        }
        for (; x < w + winSize; x++)
        {
            s1t[x] = s1t[x - 1];
            s2t[x] = s2t[x - 1];
            s3t[x] = s3t[x - 1];
            s5t[x] = s5t[x - 1];
            s6t[x] = s6t[x - 1];
        }
        for (x = -1; x >= -winSize; x--)
        {
            s1t[x] = s1t[x + 1];
            s2t[x] = s2t[x + 1];
            s3t[x] = s3t[x + 1];
            s5t[x] = s5t[x + 1];
            s6t[x] = s6t[x + 1];
        }

        p0 += w;
        p1 += w;
        gx0 += w;
        gx1 += w;
        gy0 += w;
        gy1 += w;

        s1t += wb;
        s2t += wb;
        s3t += wb;
        s5t += wb;
        s6t += wb;
    }

    for (; y < h + winSize; y++)
    {
        memcpy(s1t - winSize, s1t - winSize - wb, sizeof(int)*(wb));
        memcpy(s2t - winSize, s2t - winSize - wb, sizeof(int)*(wb));
        memcpy(s3t - winSize, s3t - winSize - wb, sizeof(int)*(wb));
        memcpy(s5t - winSize, s5t - winSize - wb, sizeof(int)*(wb));
        memcpy(s6t - winSize, s6t - winSize - wb, sizeof(int)*(wb));
        s1t += wb;
        s2t += wb;
        s3t += wb;
        s5t += wb;
        s6t += wb;
    }
    s1t = s1 + wb * (winSize - 1);
    s2t = s2 + wb * (winSize - 1);
    s3t = s3 + wb * (winSize - 1);
    s5t = s5 + wb * (winSize - 1);
    s6t = s6 + wb * (winSize - 1);
    for (y = -1; y >= -winSize; y--)
    {
        memcpy(s1t - winSize, s1t - winSize + wb, sizeof(int)*(wb));
        memcpy(s2t - winSize, s2t - winSize + wb, sizeof(int)*(wb));
        memcpy(s3t - winSize, s3t - winSize + wb, sizeof(int)*(wb));
        memcpy(s5t - winSize, s5t - winSize + wb, sizeof(int)*(wb));
        memcpy(s6t - winSize, s6t - winSize + wb, sizeof(int)*(wb));
        s1t -= wb;
        s2t -= wb;
        s3t -= wb;
        s5t -= wb;
        s6t -= wb;
    }
}

void bio_grad(pel* p0, pel* p1, pel* gx0, pel* gx1, pel* gy0, pel* gy1, int w, int h)
{
    assert((w & 3) == 0);
    pel *src;
    pel *gx, *gy;
    for (int dir = 0; dir < 2; dir++)
    {
        if (dir == 0)
        {
            src = p0;
            gx = gx0;
            gy = gy0;
        }
        else
        {
            src = p1;
            gx = gx1;
            gy = gy1;
        }

        for (int y = 0; y < h; y++)
        {
            int x = 0;
            for (; x < ((w >> 3) << 3); x += 8)
            {
                pel tmp[8];
                __m128i mm_top, mm_bottom, mm_left, mm_right;
                mm_top = _mm_loadu_si128((__m128i*)(y == 0 ? src + x : src + x - w));
                mm_bottom = _mm_loadu_si128((__m128i*)(y == h - 1 ? src + x : src + x + w));
                if (x == 0)
                {
                    tmp[0] = src[0];
                    memcpy(tmp + 1, src, sizeof(pel)*(7));
                }
                mm_left = _mm_loadu_si128((__m128i*)(x == 0 ? tmp : src + x - 1));
                if (x == w - 8)
                {
                    memcpy(tmp, src + x + 1, sizeof(pel)*(7));
                    tmp[7] = tmp[6];
                }
                mm_right = _mm_loadu_si128((__m128i*)(x == w - 8 ? tmp : src + x + 1));

                __m128i mm_gx = _mm_srai_epi16(_mm_sub_epi16(mm_right, mm_left), 4);
                __m128i mm_gy = _mm_srai_epi16(_mm_sub_epi16(mm_bottom, mm_top), 4);

                _mm_storeu_si128((__m128i *)(gx + x), mm_gx);
                _mm_storeu_si128((__m128i *)(gy + x), mm_gy);
            }
            for (; x < w; x += 4)
            {
                pel tmp[4];
                __m128i mm_top, mm_bottom, mm_left, mm_right;
                mm_top = _mm_loadl_epi64((__m128i*)(y == 0 ? src + x : src + x - w));
                mm_bottom = _mm_loadl_epi64((__m128i*)(y == h - 1 ? src + x : src + x + w));
                if (x == 0)
                {
                    tmp[0] = src[x];
                    memcpy(tmp + 1, src + x, sizeof(pel)*(3));
                }
                mm_left = _mm_loadl_epi64((__m128i*)(x == 0 ? tmp : src + x - 1));
                if (x == w - 4)
                {
                    memcpy(tmp, src + x + 1, sizeof(pel)*(3));
                    tmp[3] = tmp[2];
                }
                mm_right = _mm_loadl_epi64((__m128i*)(x == w - 4 ? tmp : src + x + 1));

                __m128i mm_gx = _mm_srai_epi16(_mm_sub_epi16(mm_right, mm_left), 4);
                __m128i mm_gy = _mm_srai_epi16(_mm_sub_epi16(mm_bottom, mm_top), 4);

                _mm_storel_epi64((__m128i *)(gy + x), mm_gy);
                _mm_storel_epi64((__m128i *)(gx + x), mm_gx);
            }

            gx += w;
            gy += w;
            src += w;
        }
    }
}

void bio_opt(pel *pred_buf, pel *pred_snd, int w, int h, int bit_depth)
{
    s32 *s1, *s2, *s3, *s5, *s6;
    s16 *p0, *p1, *gx0, *gx1, *gy0, *gy1;
    pel *p0t, *p1t, *gx0t, *gy0t, *gx1t, *gy1t;
    int bio_win_size = BIO_WINDOW_SIZE, bio_cluster_size = BIO_CLUSTER_SIZE, bio_avg_size = BIO_AVG_WIN_SIZE;
#if !SIMD_MC
    int t, tx, ty;
    int jj, ii;
    s32 b;
#endif
    int wb = w, os, ob;
    s16 vx = 0, vy = 0;

    wb = w + (bio_win_size << 1);
    int i, j;

    p0 = pred_buf;
    p1 = pred_snd;
    s1 = sigma[0] + bio_win_size;
    s2 = sigma[1] + bio_win_size;
    s3 = sigma[2] + bio_win_size;
    s5 = sigma[3] + bio_win_size;
    s6 = sigma[4] + bio_win_size;

    gx0 = grad_x[0]; gy0 = grad_y[0]; gx1 = grad_x[1]; gy1 = grad_y[1];
#if SIMD_MC
    bio_sigma(p0, p1, gx0, gx1, gy0, gy1, s1, s2, s3, s5, s6, w, h, bio_win_size, wb);
#else
    for (j = -bio_win_size; j < h + bio_win_size; j++)
    {
        for (i = -bio_win_size; i < w + bio_win_size; i++)
        {
            ii = COM_CLIP3(0, w - 1, i);
            t = p0[ii] - p1[ii];
            tx = gx0[ii] + gx1[ii];
            ty = gy0[ii] + gy1[ii];

            s1[i] = tx * tx;
            s2[i] = tx * ty;
            s3[i] = -tx * t;
            s5[i] = ty * ty;
            s6[i] = -ty * t;
        }
        s1 += wb; s2 += wb; s3 += wb; s5 += wb; s6 += wb;
        if ((j >= 0) && (j < h - 1))
        {
            p0 += w;
            p1 += w;
            gx0 += w;
            gy0 += w;
            gx1 += w;
            gy1 += w;
        }
    }
#endif
    os = bio_win_size + bio_win_size*wb;
    s1 = sigma[0] + os;
    s2 = sigma[1] + os;
    s3 = sigma[2] + os;
    s5 = sigma[3] + os;
    s6 = sigma[4] + os;
    gx0 = grad_x[0];
    gy0 = grad_y[0];
    gx1 = grad_x[1];
    gy1 = grad_y[1];
    p0 = pred_buf;
    p1 = pred_snd;

    for (j = 0; j < h; j += bio_cluster_size)
    {
        for (i = 0; i < w; i += bio_cluster_size)
        {
            os = i - bio_win_size + wb*(j - bio_win_size);
            bio_vxvy(&vx, &vy, wb, s1 + os, s2 + os, s3 + os, s5 + os, s6 + os, bio_avg_size);

            ob = i + j * w;
            p0t = p0 + ob;
            p1t = p1 + ob;
            gx0t = gx0 + ob;
            gy0t = gy0 + ob;
            gx1t = gx1 + ob;
            gy1t = gy1 + ob;
#if SIMD_MC
            bio_block_average_16b_clip_sse(p0t, p1t, gx0t, gy0t, gx1t, gy1t, vx, vy, bio_cluster_size, w, bit_depth);
#else
            {
                for (jj = 0; jj < bio_cluster_size; jj++)
                {
                    for (ii = 0; ii < bio_cluster_size; ii++)
                    {
                        b = vx * (gx0t[ii] - gx1t[ii]) + vy * (gy0t[ii] - gy1t[ii]);
                        b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));
                        p0t[ii] = (s16)((p0t[ii] + p1t[ii] + b + 1) >> 1);
                        p0t[ii] = COM_CLIP3(0, (1 << BIT_DEPTH) - 1, p0t[ii]);
                    }
                    p0t += w;
                    p1t += w;
                    gx0t += w;
                    gy0t += w;
                    gx1t += w;
                    gy1t += w;
                }
            }
#endif
        }
    }
}
#endif

void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16 (*mv_t)[MV_D])
{
    int min_clip[MV_D], max_clip[MV_D];
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;
    min_clip[MV_X] = (-MAX_CU_SIZE - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE + 4) << 2;
    mv_t[REFP_0][MV_X] = mv[REFP_0][MV_X];
    mv_t[REFP_0][MV_Y] = mv[REFP_0][MV_Y];
    mv_t[REFP_1][MV_X] = mv[REFP_1][MV_X];
    mv_t[REFP_1][MV_Y] = mv[REFP_1][MV_Y];
    if(REFI_IS_VALID(refi[REFP_0]))
    {
        if (x + mv[REFP_0][MV_X] < min_clip[MV_X]) mv_t[REFP_0][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_0][MV_Y] < min_clip[MV_Y]) mv_t[REFP_0][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_0][MV_X] + w - 4 > max_clip[MV_X]) mv_t[REFP_0][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_0][MV_Y] + h - 4 > max_clip[MV_Y]) mv_t[REFP_0][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }
    if (REFI_IS_VALID(refi[REFP_1]))
    {
        if (x + mv[REFP_1][MV_X] < min_clip[MV_X]) mv_t[REFP_1][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_1][MV_Y] < min_clip[MV_Y]) mv_t[REFP_1][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_1][MV_X] + w - 4 > max_clip[MV_X]) mv_t[REFP_1][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_1][MV_Y] + h - 4 > max_clip[MV_Y]) mv_t[REFP_1][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }
}

#if DMVR
static BOOL mv_clip_only_one_ref_dmvr(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D], s16( *mv_t ))
{
    BOOL clip_flag = 0;
    int min_clip[MV_D], max_clip[MV_D];
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;
    min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;
    mv_t[MV_X] = mv[MV_X];
    mv_t[MV_Y] = mv[MV_Y];

    if (x + mv[MV_X] < min_clip[MV_X])
    {
        clip_flag = 1;
        mv_t[MV_X] = min_clip[MV_X] - x;
    }
    if (y + mv[MV_Y] < min_clip[MV_Y])
    {
        clip_flag = 1;
        mv_t[MV_Y] = min_clip[MV_Y] - y;
    }
    if (x + mv[MV_X] + w - 4 > max_clip[MV_X])
    {
        clip_flag = 1;
        mv_t[MV_X] = max_clip[MV_X] - x - w + 4;
    }
    if (y + mv[MV_Y] + h - 4 > max_clip[MV_Y])
    {
        clip_flag = 1;
        mv_t[MV_Y] = max_clip[MV_Y] - y - h + 4;
    }
    return clip_flag;
}

#ifdef X86_SSE
#define SSE_SAD_16B_4PEL(src1, src2, s00, s01, sac0) \
        s00 = _mm_loadl_epi64((__m128i*)(src1)); \
        s01 = _mm_loadl_epi64((__m128i*)(src2));\
        s00 = _mm_sub_epi16(s00, s01); \
        s00 = _mm_abs_epi16(s00); \
        s00 = _mm_cvtepi16_epi32(s00); \
        \
        sac0 = _mm_add_epi32(sac0, s00);
#endif

#define SSE_SAD_16B_8PEL(src1, src2, s00, s01, sac0) \
        s00 = _mm_loadu_si128((__m128i*)(src1)); \
        s01 = _mm_loadu_si128((__m128i*)(src2)); \
        s00 = _mm_sub_epi16(s00, s01); \
        s01 = _mm_abs_epi16(s00); \
        \
        s00 = _mm_hadd_epi16(s01, s01); \
        s00 = _mm_cvtepi16_epi32(s00); \
        \
        sac0 = _mm_add_epi32(sac0, s00);

s32 com_DMVR_cost( int w, int h, pel *src1, pel *src2, int s_src1, int s_src2 )
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;
    int i, j;
    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    if( w & 0x07 )
    {
        for( i = 0; i < (h >> 2); i++ )
        {
            for( j = 0; j < w; j += 4 )
            {
                SSE_SAD_16B_4PEL( s1 + j, s2 + j, s00, s01, sac0 );
                SSE_SAD_16B_4PEL( s1 + j + s_src1, s2 + j + s_src2, s00, s01, sac0 );
                SSE_SAD_16B_4PEL( s1 + j + s_src1 * 2, s2 + j + s_src2 * 2, s00, s01, sac0 );
                SSE_SAD_16B_4PEL( s1 + j + s_src1 * 3, s2 + j + s_src2 * 3, s00, s01, sac0 );
            }
            s1 += s_src1 << 2;
            s2 += s_src2 << 2;
        }
        sad = _mm_extract_epi32( sac0, 0 );
        sad += _mm_extract_epi32( sac0, 1 );
        sad += _mm_extract_epi32( sac0, 2 );
        sad += _mm_extract_epi32( sac0, 3 );
    }
    else
    {
        for( i = 0; i < (h >> 2); i++ )
        {
            for( j = 0; j < w; j += 8 )
            {
                SSE_SAD_16B_8PEL( s1 + j, s2 + j, s00, s01, sac0 );
                SSE_SAD_16B_8PEL( s1 + j + s_src1, s2 + j + s_src2, s00, s01, sac0 );
                SSE_SAD_16B_8PEL( s1 + j + s_src1 * 2, s2 + j + s_src2 * 2, s00, s01, sac0 );
                SSE_SAD_16B_8PEL( s1 + j + s_src1 * 3, s2 + j + s_src2 * 3, s00, s01, sac0 );
            }
            s1 += s_src1 << 2;
            s2 += s_src2 << 2;
        }
        sad = _mm_extract_epi32( sac0, 0 );
        sad += _mm_extract_epi32( sac0, 1 );
        sad += _mm_extract_epi32( sac0, 2 );
        sad += _mm_extract_epi32( sac0, 3 );
    }

    return sad;
}


#define ONCESAD_SIZE 21
#if ONCESAD_SIZE==25
#define RECURSIVE_MINSAD 1 
#define ONCESAD_SIZE_STEP 0
#define ONCESAD_SIZE_STRIDE 0
#elif ONCESAD_SIZE==21
#define RECURSIVE_MINSAD 1 
#define ONCESAD_SIZE_STEP 0
#define ONCESAD_SIZE_STRIDE 0
#endif

enum SAD_POINT_INDEX
{
    SAD_NOT_AVAILABLE = -1,
    SAD_BOTTOM = 0,
    SAD_TOP,
    SAD_RIGHT,
    SAD_LEFT,
    SAD_TOP_LEFT,
    SAD_TOP_RIGHT,
    SAD_BOTTOM_LEFT,
    SAD_BOTTOM_RIGHT,
    SAD_CENTER,
    SAD_COUNT
};

void com_DMVR_refine_ONCE( int w, int h, pel *ref_l0, int s_ref_l0, pel *ref_l1, int s_ref_l1,
    s32 *minCost, s16 *delta_mvX, s16 *delta_mvY, s32 *SAD_Array )
{
    enum SAD_POINT_INDEX idx;
    s32 searchOffsetXBack[5] = { 0, 0, 1, -1, 0 };
    s32 searchOffsetYBack[5] = { 1, -1, 0, 0, 0 };
#if RECURSIVE_MINSAD
    s32 j = 0;
    s32 start_x = 0;
    s32 start_y = 0;
#endif  
#if ONCESAD_SIZE==21
    s32 searchOffsetX[21] = { 0, 0, 0, 1, -1, 0, 1, 2, 1, 0, -1, -2, -1, 2, 1, 2, 1, -1, -2, -2, -1 };
    s32 searchOffsetY[21] = { 0, 1, -1, 0, 0, 2, 1, 0, -1, -2, -1, 0, 1, 1, 2, -1, -2, -2, -1, 1, 2 };
    s32 cost_temp[5][5] = { { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX } };
    s32 center_x = 2;
    s32 center_y = 2;
#elif ONCESAD_SIZE==25
    s32 searchOffsetX[25] = { 0, 0, 0, 1, -1, 0, 1, 2, 1, 0, -1, -2, -1, 2, 1, 2, 1, -1, -2, -2, -1, 2, 2, -2, -2 };
    s32 searchOffsetY[25] = { 0, 1, -1, 0, 0, 2, 1, 0, -1, -2, -1, 0, 1, 1, 2, -1, -2, -2, -1, 1, 2, 2, -2, 2, -2 };
    s32 cost_temp[5][5] = { { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX } };
    s32 center_x = 2;
    s32 center_y = 2;
#endif
    pel *ref_l0_Orig = ref_l0;
    pel *ref_l1_Orig = ref_l1;
    for( idx = 0; idx < ONCESAD_SIZE; ++idx )
    {
        int sum = 0;
        ref_l0 = ref_l0_Orig + (searchOffsetX[idx] << ONCESAD_SIZE_STEP) + ((searchOffsetY[idx] << ONCESAD_SIZE_STEP) * s_ref_l0);
        ref_l1 = ref_l1_Orig - (searchOffsetX[idx] << ONCESAD_SIZE_STEP) - ((searchOffsetY[idx] << ONCESAD_SIZE_STEP) * s_ref_l1);
#if ONCESAD_SIZE_STRIDE
        s32 cost = com_DMVR_cost( w, h >> ONCESAD_SIZE_STRIDE, ref_l0, ref_l1, s_ref_l0 << ONCESAD_SIZE_STRIDE, s_ref_l1 << ONCESAD_SIZE_STRIDE );
#else
        s32 cost = com_DMVR_cost( w, h, ref_l0, ref_l1, s_ref_l0, s_ref_l1 );
#endif
        cost_temp[center_y + searchOffsetY[idx]][center_x + searchOffsetX[idx]] = cost;
#if !RECURSIVE_MINSAD
        if( cost < *minCost )
        {
            *minCost = cost;
            *delta_mvX = (searchOffsetX[idx] << ONCESAD_SIZE_STEP);
            *delta_mvY = (searchOffsetY[idx] << ONCESAD_SIZE_STEP);
            if( cost == 0 ) break;
        }
#endif
    }/*end of search point loop*/
#if RECURSIVE_MINSAD==1
    * minCost = cost_temp[center_y][center_x];
    for( j = 0; j < 2; j++ )
    {
        s32 delta_x = 0;
        s32 delta_y = 0;
        for( idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx )
        {
            s32 cost = cost_temp[center_y + start_y + searchOffsetYBack[idx]][center_x + start_x + searchOffsetXBack[idx]];
            if( cost < *minCost )
            {
                *minCost = cost;
                delta_x = searchOffsetXBack[idx];
                delta_y = searchOffsetYBack[idx];
                if( cost == 0 ) break;
            }
        }
        if( *minCost == 0 ) break;
        if( (delta_x == 0) && (delta_y == 0) ) break;
        start_x += delta_x;
        start_y += delta_y;
    }
    *delta_mvX = (start_x << ONCESAD_SIZE_STEP);
    *delta_mvY = (start_y << ONCESAD_SIZE_STEP);
#endif
    if( (abs( *delta_mvX ) < center_x) && (abs( *delta_mvY ) < center_y) )
    {
        *(SAD_Array + SAD_CENTER) = *minCost;
        for( idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx )
        {
            *(SAD_Array + idx) = cost_temp[center_y + (*delta_mvY >> ONCESAD_SIZE_STEP) + searchOffsetYBack[idx]][center_x + (*delta_mvX >> ONCESAD_SIZE_STEP) + searchOffsetXBack[idx]];
        }
    }
    ref_l0 = ref_l0_Orig;
    ref_l1 = ref_l1_Orig;
}

s32 div_for_maxq7( s64 N, s64 D )
{
    s32 sign, q;

    sign = 0;
    if( N < 0 )
    {
        sign = 1;
        N = -N;
    }

    q = 0;
    D = (D << 3);
    if( N >= D )
    {
        N -= D;
        q++;
    }
    q = (q << 1);

    D = (D >> 1);
    if( N >= D )
    {
        N -= D;
        q++;
    }
    q = (q << 1);

    if( N >= (D >> 1) )
        q++;

    if( sign )
        return (-q);
    return(q);
}

void com_SubPelErrorSrfc( int *sadBuffer, int *deltaMv )
{
    s64 iNum, iDenom;
    int iMvDelta_SubPel;
    int MvSubPel_lvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
                                                          /*horizontal*/
    iNum = (s64)((sadBuffer[1] - sadBuffer[3]) << MvSubPel_lvl);
    iDenom = (s64)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

    if( 0 != iDenom )
    {
        if( (sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]) )
        {
            iMvDelta_SubPel = div_for_maxq7( iNum, iDenom );
            deltaMv[0] = (iMvDelta_SubPel);
        }
        else
        {
            if( sadBuffer[1] == sadBuffer[0] )
            {
                deltaMv[0] = -8;// half pel
            }
            else
            {
                deltaMv[0] = 8;// half pel
            }
        }
    }
    /*vertical*/
    iNum = (s64)((sadBuffer[2] - sadBuffer[4]) << MvSubPel_lvl);
    iDenom = (s64)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
    if( 0 != iDenom )
    {
        if( (sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]) )
        {
            iMvDelta_SubPel = div_for_maxq7( iNum, iDenom );
            deltaMv[1] = (iMvDelta_SubPel);
        }
        else
        {
            if( sadBuffer[2] == sadBuffer[0] )
            {
                deltaMv[1] = -8;// half pel
            }
            else
            {
                deltaMv[1] = 8;// half pel
            }
        }
    }
    return;
}

#if DMVR
void copy_buffer( pel *src, int src_stride, pel *dst, int dst_stride, int width, int height )
{
    int numBytes = width * sizeof( pel );
    for( int i = 0; i < height; i++ )
    {
        memcpy( dst + i * dst_stride, src + i * src_stride, numBytes );
    }
}

void padding( pel *ptr, int iStride, int iWidth, int iHeight, int PadLeftsize, int PadRightsize, int PadTopsize, int PadBottomSize )
{
    /*left padding*/
    pel *ptr_temp = ptr;
    int offset = 0;
    for( int i = 0; i < iHeight; i++ )
    {
        offset = iStride * i;
        for( int j = 1; j <= PadLeftsize; j++ )
        {
            *(ptr_temp - j + offset) = *(ptr_temp + offset);
        }
    }
    /*Right padding*/
    ptr_temp = ptr + (iWidth - 1);
    for( int i = 0; i < iHeight; i++ )
    {
        offset = iStride * i;
        for( int j = 1; j <= PadRightsize; j++ )
        {
            *(ptr_temp + j + offset) = *(ptr_temp + offset);
        }
    }
    /*Top padding*/
    int numBytes = (iWidth + PadLeftsize + PadRightsize) * sizeof( pel );
    ptr_temp = (ptr - PadLeftsize);
    for( int i = 1; i <= PadTopsize; i++ )
    {
        memcpy( ptr_temp - (i * iStride), (ptr_temp), numBytes );
    }
    /*Bottom padding*/
    numBytes = (iWidth + PadLeftsize + PadRightsize) * sizeof( pel );
    ptr_temp = (ptr + (iStride * (iHeight - 1)) - PadLeftsize);
    for( int i = 1; i <= PadBottomSize; i++ )
    {
        memcpy( ptr_temp + (i * iStride), (ptr_temp), numBytes );
    }
}

static void prefetch_for_Submc_s( int x, int y, int pic_w, int pic_h, int w, int h, int x_sub, int y_sub, int w_sub, int h_sub, s8 refi[REFP_NUM], s16( *mv )[MV_D], COM_REFP( *refp )[REFP_NUM]
    , int iteration
    , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
)
{
    s16 mv_temp[REFP_NUM][MV_D];
    int l_w = w, l_h = h;
    int c_w = w >> 1, c_h = h >> 1;
    int num_extra_pixel_left_for_filter;
    for( int i = 0; i < REFP_NUM; ++i )
    {
        int filtersize = NTAPS_LUMA;
        num_extra_pixel_left_for_filter = ((filtersize >> 1) - 1);
        int offset = ((DMVR_ITER_COUNT) * (PAD_BUFFER_STRIDE + 1));
        int padsize = DMVR_PAD_LENGTH;
        int qpel_gmv_x, qpel_gmv_y;
        COM_PIC    *ref_pic;
        mv_clip_only_one_ref_dmvr( x, y, pic_w, pic_h, w, h, mv[i], mv_temp[i] );

        qpel_gmv_x = (((x + x_sub) << 2) + mv_temp[i][MV_X]);
        qpel_gmv_y = (((y + y_sub) << 2) + mv_temp[i][MV_Y]);
        //offset += (x_sub + y_sub*PAD_BUFFER_STRIDE);
        ref_pic = refp[refi[i]][i].pic;
        pel *ref = ref_pic->y + ((qpel_gmv_y >> 2) - num_extra_pixel_left_for_filter) * ref_pic->stride_luma +
            (qpel_gmv_x >> 2) - num_extra_pixel_left_for_filter;
        pel *dst = dmvr_padding_buf[i][0] + offset;
        copy_buffer( ref, ref_pic->stride_luma, dst, PAD_BUFFER_STRIDE, (w_sub + filtersize), (h_sub + filtersize) );
        padding( dst, PAD_BUFFER_STRIDE, (w_sub + filtersize - 1), (h_sub + filtersize - 1), padsize,
            padsize, padsize, padsize );

        // chroma
        filtersize = NTAPS_CHROMA;
        num_extra_pixel_left_for_filter = ((filtersize >> 1) - 1);
        offset = (DMVR_ITER_COUNT);
        offset = offset *(PAD_BUFFER_STRIDE + 1);
        //offset += ((x_sub>>1) + (y_sub>>1)*PAD_BUFFER_STRIDE);
        padsize = DMVR_PAD_LENGTH >> 1;
        ref = ref_pic->u + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma + 
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][1] + offset;
        copy_buffer( ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, ((w_sub>>1) + filtersize), ((h_sub>>1) + filtersize) );
        padding( dst, PAD_BUFFER_STRIDE, ((w_sub>>1) + filtersize - 1), ((h_sub>>1) + filtersize - 1), padsize,
            padsize, padsize, padsize );

        ref = ref_pic->v + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma + 
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][2] + offset;
        copy_buffer( ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, ((w_sub>>1) + filtersize), ((h_sub>>1) + filtersize) );
        padding( dst, PAD_BUFFER_STRIDE, ((w_sub>>1) + filtersize - 1), ((h_sub>>1) + filtersize - 1), padsize,
            padsize, padsize, padsize );
    }
}
#endif

static void final_paddedMC_forDMVR_S( int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16( *inital_mv )[MV_D], s16( *refined_mv )[MV_D], COM_REFP( *refp )[REFP_NUM], pel pred[N_C][MAX_CU_DIM], pel pred1[N_C][MAX_CU_DIM]
    , int sub_pred_offset_x, int sub_pred_offset_y, int cu_pred_stride, int bitdepth, pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
    , int apply_BIO
)
{
    int i;
    COM_PIC *ref_pic;
    s16     mv_temp[REFP_NUM][MV_D];
    pel( *pred_buf )[MAX_CU_DIM] = pred;

    for( i = 0; i < REFP_NUM; ++i )
    {
        int qpel_gmv_x, qpel_gmv_y;
        if (i > 0)
        {
            pred_buf = pred1;
        }
        ref_pic = refp[refi[i]][i].pic;

        s16 temp_uncliped_mv[MV_D] = { refined_mv[i][MV_X], refined_mv[i][MV_Y] };
        BOOL clip_flag = mv_clip_only_one_ref_dmvr( x, y, pic_w, pic_h, w, h, temp_uncliped_mv, mv_temp[i] );

        if( clip_flag )
        {
            qpel_gmv_x = (x << 2) + (mv_temp[i][MV_X]);
            qpel_gmv_y = (y << 2) + (mv_temp[i][MV_Y]);
        }
        else
        {
            qpel_gmv_x = (x << 2) + (refined_mv[i][MV_X]);
            qpel_gmv_y = (y << 2) + (refined_mv[i][MV_Y]);
        }

        int delta_x_l = 0;
        int delta_y_l = 0;
        int delta_x_c = 0;
        int delta_y_c = 0;
        int offset = 0;
        int filter_size = NTAPS_LUMA;
        int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);

        if( clip_flag == 0 )
        {
            // int pixel movement from inital mv
            delta_x_l = (refined_mv[i][MV_X] >> 2) - (inital_mv[i][MV_X] >> 2);
            delta_y_l = (refined_mv[i][MV_Y] >> 2) - (inital_mv[i][MV_Y] >> 2);
            delta_x_c = (refined_mv[i][MV_X] >> 3) - (inital_mv[i][MV_X] >> 3);
            delta_y_c = (refined_mv[i][MV_Y] >> 3) - (inital_mv[i][MV_Y] >> 3);
        }
        else
        {
            // int pixel movement from inital mv
            delta_x_l = (mv_temp[i][MV_X] >> 2) - (inital_mv[i][MV_X] >> 2);
            delta_y_l = (mv_temp[i][MV_Y] >> 2) - (inital_mv[i][MV_Y] >> 2);
            delta_x_c = (mv_temp[i][MV_X] >> 3) - (inital_mv[i][MV_X] >> 3);
            delta_y_c = (mv_temp[i][MV_Y] >> 3) - (inital_mv[i][MV_Y] >> 3);
#if DMVR_MV_FIX
            int luma_pad = DMVR_ITER_COUNT;
            int chroma_pad = DMVR_ITER_COUNT >> 1;
            delta_x_l = COM_CLIP3( -luma_pad, luma_pad, delta_x_l );
            delta_y_l = COM_CLIP3( -luma_pad, luma_pad, delta_y_l );
            delta_x_c = COM_CLIP3( -chroma_pad, chroma_pad, delta_x_c );
            delta_y_c = COM_CLIP3( -chroma_pad, chroma_pad, delta_y_c );
#endif
        }
#if DMVR_MV_FIX
        assert( DMVR_ITER_COUNT == 2 );
        assert( delta_x_l >= -DMVR_ITER_COUNT && delta_x_l <= DMVR_ITER_COUNT );
        assert( delta_y_l >= -DMVR_ITER_COUNT && delta_y_l <= DMVR_ITER_COUNT );
        assert( delta_x_c >= -(DMVR_ITER_COUNT >> 1) && delta_x_c <= (DMVR_ITER_COUNT >> 1) );
        assert( delta_y_c >= -(DMVR_ITER_COUNT >> 1) && delta_y_c <= (DMVR_ITER_COUNT >> 1) );
#endif
        offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
        offset += (delta_y_l)* PAD_BUFFER_STRIDE;
        offset += (delta_x_l);

        pel *src = dmvr_padding_buf[i][0] + offset;// + sub_pred_offset_x + sub_pred_offset_y * PAD_BUFFER_STRIDE;;
        pel *temp = pred_buf[Y_C] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
        com_dmvr_mc_l( src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp, w, h, bitdepth );

        if (apply_BIO)
        {
            pel* temp_grad_x = grad_x[i] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
            pel* temp_grad_y = grad_y[i] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
            com_grad_x_l_nn(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp_grad_x, w, h, bitdepth, 1);
            com_grad_y_l_nn(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp_grad_y, w, h, bitdepth, 1);
        }

        filter_size = NTAPS_CHROMA;
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);
        offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
        offset += (delta_y_c)* PAD_BUFFER_STRIDE;
        offset += (delta_x_c);
        src = dmvr_padding_buf[i][1] + offset;// +(sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * PAD_BUFFER_STRIDE;;
        temp = pred_buf[U_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
        com_dmvr_mc_c( src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1, bitdepth );

        src = dmvr_padding_buf[i][2] + offset;// +(sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * PAD_BUFFER_STRIDE;;
        temp = pred_buf[V_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
        com_dmvr_mc_c( src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1, bitdepth );
    }
}

// from int pos, so with full error surface
void process_DMVR( int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16( *mv )[MV_D], COM_REFP( *refp )[REFP_NUM], pel pred[N_C][MAX_CU_DIM], pel pred1[N_C][MAX_CU_DIM]
    , int poc_c, pel *dmvr_current_template, pel( *dmvr_ref_pred_interpolated )[(MAX_CU_SIZE + (2 * (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + (2 * (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))]
    , int iteration, int bitdepth, pel( *dmvr_padding_buf )[N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
    , int apply_BIO
)
{
    s16 sub_pu_L0[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
    s16 sub_pu_L1[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
    s16 ref_pred_mv_scaled_step = 2;
    s16 refined_mv[REFP_NUM][MV_D] = { { mv[REFP_0][MV_X], mv[REFP_0][MV_Y] },
                                       { mv[REFP_1][MV_X], mv[REFP_1][MV_Y] }
    };
    s16 starting_mv[REFP_NUM][MV_D];
    mv_clip( x, y, pic_w, pic_h, w, h, refi, mv, starting_mv );
    //form int mv
    s16 sign_x = starting_mv[REFP_0][MV_X] >= 0 ? 1 : (-1);
    s16 sign_y = starting_mv[REFP_0][MV_Y] >= 0 ? 1 : (-1);
    s16 abs_x = abs( starting_mv[REFP_0][MV_X] );
    s16 abs_y = abs( starting_mv[REFP_0][MV_Y] );
    s16 xFrac = abs_x & 3;
    s16 yFrac = abs_y & 3;
    starting_mv[REFP_0][MV_X] = (((abs_x) >> 2) << 2);
    starting_mv[REFP_0][MV_Y] = (((abs_y) >> 2) << 2);
    starting_mv[REFP_0][MV_X] += (((xFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_0][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_0][MV_X] *= sign_x;
    starting_mv[REFP_0][MV_Y] *= sign_y;

    sign_x = starting_mv[REFP_1][MV_X] >= 0 ? 1 : (-1);
    sign_y = starting_mv[REFP_1][MV_Y] >= 0 ? 1 : (-1);
    abs_x = abs( starting_mv[REFP_1][MV_X] );
    abs_y = abs( starting_mv[REFP_1][MV_Y] );
    xFrac = abs_x & 3;
    yFrac = abs_y & 3;
    starting_mv[REFP_1][MV_X] = (((abs_x) >> 2) << 2);
    starting_mv[REFP_1][MV_Y] = (((abs_y) >> 2) << 2);
    starting_mv[REFP_1][MV_X] += (((xFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][MV_X] *= sign_x;
    starting_mv[REFP_1][MV_Y] *= sign_y;

    // centre address holder for pred
    pel *preds_array[REFP_NUM];
    pel *preds_centre_array[REFP_NUM];

    int stride = PAD_BUFFER_STRIDE;
    preds_array[REFP_0] = dmvr_padding_buf[REFP_0][Y_C];
    preds_array[REFP_1] = dmvr_padding_buf[REFP_1][Y_C];

    int filter_size = NTAPS_LUMA;
    int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);
    // go to the center point
    preds_centre_array[REFP_0] = preds_array[REFP_0] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
    preds_centre_array[REFP_1] = preds_array[REFP_1] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));

    int minCost = INT_MAX;
    int lastDirection = -1;
    int arrayCost[SAD_COUNT];
    int dx, dy;
    dy = min( h, DMVR_IMPLIFICATION_SUBCU_SIZE );
    dx = min( w, DMVR_IMPLIFICATION_SUBCU_SIZE );

    int num = 0;
    int subPuStartX, subPuStartY, startX, startY;
    for( startY = 0, subPuStartY = y; subPuStartY < (y + h); subPuStartY = subPuStartY + dy, startY += dy )
    {
        for( startX = 0, subPuStartX = x; subPuStartX < (x + w); subPuStartX = subPuStartX + dx, startX += dx )
        {
            s16 total_delta_mv[MV_D] = { 0, 0 };
            BOOL notZeroCost = 1;

            // produce padded buffer for exact MC
            prefetch_for_Submc_s( x, y, pic_w, pic_h, w, h, startX, startY, dx, dy, refi, starting_mv, refp, iteration, dmvr_padding_buf);

            pel *addr_subpu_l0 = preds_centre_array[REFP_0];// +startX + startY * stride;
            pel *addr_subpu_l1 = preds_centre_array[REFP_1];// +startX + startY * stride;

            for( int loop = 0; loop < SAD_COUNT; loop++ )
            {
                arrayCost[loop] = INT_MAX;
            }

            minCost = INT_MAX;
            com_DMVR_refine_ONCE( dx, dy, addr_subpu_l0, stride, addr_subpu_l1, stride
                , &minCost
                , &total_delta_mv[MV_X], &total_delta_mv[MV_Y]
                , arrayCost );

            if( minCost == 0 )
            {
                notZeroCost = 0;
            }

            total_delta_mv[MV_X] = (total_delta_mv[MV_X] << 2);
            total_delta_mv[MV_Y] = (total_delta_mv[MV_Y] << 2);

            if( notZeroCost && (minCost == arrayCost[SAD_CENTER])
                && (arrayCost[SAD_CENTER] != INT_MAX)
                && (arrayCost[SAD_LEFT] != INT_MAX)
                && (arrayCost[SAD_TOP] != INT_MAX)
                && (arrayCost[SAD_RIGHT] != INT_MAX)
                && (arrayCost[SAD_BOTTOM] != INT_MAX) )
            {
                int sadbuffer[5];
                int deltaMv[MV_D] = { 0, 0 };
                sadbuffer[0] = arrayCost[SAD_CENTER];
                sadbuffer[1] = arrayCost[SAD_LEFT];
                sadbuffer[2] = arrayCost[SAD_TOP];
                sadbuffer[3] = arrayCost[SAD_RIGHT];
                sadbuffer[4] = arrayCost[SAD_BOTTOM];
                com_SubPelErrorSrfc( sadbuffer, deltaMv );

                total_delta_mv[MV_X] += ((deltaMv[MV_X]) >> (2 - ONCESAD_SIZE_STEP));
                total_delta_mv[MV_Y] += ((deltaMv[MV_Y]) >> (2 - ONCESAD_SIZE_STEP));
            }

            refined_mv[REFP_0][MV_X] = (starting_mv[REFP_0][MV_X]) + (total_delta_mv[MV_X]);
            refined_mv[REFP_0][MV_Y] = (starting_mv[REFP_0][MV_Y]) + (total_delta_mv[MV_Y]);

            refined_mv[REFP_1][MV_X] = (starting_mv[REFP_1][MV_X]) - (total_delta_mv[MV_X]);
            refined_mv[REFP_1][MV_Y] = (starting_mv[REFP_1][MV_Y]) - (total_delta_mv[MV_Y]);

            sub_pu_L0[num][MV_X] = refined_mv[REFP_0][MV_X];
            sub_pu_L0[num][MV_Y] = refined_mv[REFP_0][MV_Y];

            sub_pu_L1[num][MV_X] = refined_mv[REFP_1][MV_X];
            sub_pu_L1[num][MV_Y] = refined_mv[REFP_1][MV_Y];

            final_paddedMC_forDMVR_S( subPuStartX, subPuStartY, pic_w, pic_h, dx, dy, refi, starting_mv, refined_mv, refp, pred, pred1
                , startX, startY, w, bitdepth, dmvr_padding_buf, apply_BIO);
            num++;
        }
    }
}
#endif


void com_mc(int x, int y, int w, int h, int pred_stride, pel pred_buf[N_C][MAX_CU_DIM], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], CHANNEL_TYPE channel, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
#if MVAP
    , int mvap_flag
#endif
#if SUB_TMVP
    , int sbTmvp_flag
#endif
#if BGC
    , s8 bgc_flag, s8 bgc_idx
#endif
)
{
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    s8 *refi = mod_info_curr->refi;
    s16 (*mv)[MV_D] = mod_info_curr->mv;
    COM_PIC *ref_pic;
    pel pred_snd[N_C][MAX_CU_DIM];
#if BGC
    pel *pred_fir = info->pred_tmp;
    pel *p0, *p1, *dest0;
#endif
    pel (*pred)[MAX_CU_DIM] = pred_buf;
#if !SIMD_MC
    pel *p2, *p3;
#endif
    int qpel_gmv_x, qpel_gmv_y;
    int bidx = 0;
    s16 mv_t[REFP_NUM][MV_D];

#if DMVR
    int poc0 = refp[refi[REFP_0]][REFP_0].ptr;
    int poc1 = refp[refi[REFP_1]][REFP_1].ptr;
    s16 mv_refine[REFP_NUM][MV_D] = {{mv[REFP_0][MV_X], mv[REFP_0][MV_Y]},
                                     {mv[REFP_1][MV_X], mv[REFP_1][MV_Y]}};
    s16 inital_mv[REFP_NUM][MV_D] = { { mv[REFP_0][MV_X], mv[REFP_0][MV_Y] },
                                      { mv[REFP_1][MV_X], mv[REFP_1][MV_Y] }};
    BOOL dmvr_poc_condition = ((BOOL)((dmvr->poc_c - poc0)*(dmvr->poc_c - poc1) < 0)) && (abs(dmvr->poc_c - poc0) == abs(dmvr->poc_c - poc1));
    s32 extend_width = (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT;
    s32 extend_width_minus1 = DMVR_NEW_VERSION_ITER_COUNT * REF_PRED_EXTENTION_PEL_COUNT;
    int stride = w + (extend_width << 1);
    s16 mv_offsets[REFP_NUM][MV_D] = {{0,},};
    int iterations_count = DMVR_ITER_COUNT;
#endif
#if BIO
    int bio_poc0 = REFI_IS_VALID(refi[REFP_0]) ? refp[refi[REFP_0]][REFP_0].pic->ptr : -1;
    int bio_poc1 = REFI_IS_VALID(refi[REFP_1]) ? refp[refi[REFP_1]][REFP_1].pic->ptr : -1;
    int bio_is_bi = (ptr >= 0 && REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]) && ((bio_poc0 - ptr) * (ptr - bio_poc1) > 0)) ? 1 : 0;
#endif

    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);
#if DMVR
    dmvr->apply_DMVR = dmvr->apply_DMVR && dmvr_poc_condition;
    dmvr->apply_DMVR = dmvr->apply_DMVR && (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]));
    dmvr->apply_DMVR = dmvr->apply_DMVR && !(refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr &&  mv_t[REFP_0][MV_X] == mv_t[REFP_1][MV_X] && mv_t[REFP_0][MV_Y] == mv_t[REFP_1][MV_Y]);
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!((w == 4 && h <= 8) || (w <= 8 && h == 4)));
    dmvr->apply_DMVR = dmvr->apply_DMVR && (w >= 8 && h >= 8);
#if MVAP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mvap_flag);
#endif
#if AWP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->awp_flag);
#endif
#if ETMVP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->etmvp_flag);
#endif
#endif
    if (REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_0][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_0][MV_Y];

        if (channel != CHANNEL_C)
        {
#if DMVR
            if (!dmvr->apply_DMVR)
            {
#endif
                com_mc_l(mv[REFP_0][0], mv[REFP_0][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
                if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE)
                {
                    com_grad_x_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_x[0], w, h, bit_depth, 0);
                    com_grad_y_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_y[0], w, h, bit_depth, 0);
                }
#if DMVR
            }
#endif
        }
        if (channel != CHANNEL_L)
        {
#if CHROMA_NOT_SPLIT
            assert(w >= 8 && h >= 8);
#endif
#if DMVR
            if(!REFI_IS_VALID(refi[REFP_1]) || !dmvr->apply_DMVR || !dmvr_poc_condition)
#endif
            {
                com_mc_c( mv[REFP_0][0], mv[REFP_0][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth );
                com_mc_c( mv[REFP_0][0], mv[REFP_0][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth );
            }
        }
        bidx++;
#if BGC
        if (bgc_flag)
        {
            p0 = pred_buf[Y_C];
            dest0 = pred_fir;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    dest0[i] = p0[i];
                }
                p0 += pred_stride;
                dest0 += pred_stride;
            }
        }
#endif
    }

    /* check identical motion */
    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
#if LIBVC_ON
        if( refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr && mv[REFP_0][MV_X] == mv[REFP_1][MV_X] && mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y] && refp[refi[REFP_0]][REFP_0].is_library_picture == refp[refi[REFP_1]][REFP_1].is_library_picture )
#else
        if( refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr && mv[REFP_0][MV_X] == mv[REFP_1][MV_X] && mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y] )
#endif
        {
            return;
        }
    }

    if (REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        if (bidx)
        {
            pred = pred_snd;
        }
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_1][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_1][MV_Y];

        if (channel != CHANNEL_C)
        {
#if DMVR
            if (!dmvr->apply_DMVR)
            {
#endif
                com_mc_l(mv[REFP_1][0], mv[REFP_1][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
                if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE)
                {
                    com_grad_x_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_x[bidx], w, h, bit_depth, 0);
                    com_grad_y_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_y[bidx], w, h, bit_depth, 0);
                }
#if DMVR
            }
#endif
        }

        if (channel != CHANNEL_L)
        {
#if CHROMA_NOT_SPLIT
            assert(w >= 8 && h >= 8);
#endif
#if DMVR
            if(!REFI_IS_VALID(refi[REFP_0]) || !dmvr->apply_DMVR || !dmvr_poc_condition)
#endif
            {
                com_mc_c( mv[REFP_1][0], mv[REFP_1][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth );
                com_mc_c( mv[REFP_1][0], mv[REFP_1][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth );
            }
        }
        bidx++;
    }

    if (bidx == 2)
    {
#if DMVR
        //only if the references are located on oppisite sides of the current frame
        if( dmvr->apply_DMVR )
        {
            process_DMVR( x, y, pic_w, pic_h, w, h, refi, mv, refp, pred_buf, pred_snd, dmvr->poc_c, dmvr->dmvr_current_template, dmvr->dmvr_ref_pred_interpolated
                , iterations_count
                , bit_depth
                , dmvr->dmvr_padding_buf
                , (info->sqh.bio_enable_flag&& bio_is_bi&& mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE)
            );

            mv[REFP_0][MV_X] = inital_mv[REFP_0][MV_X];
            mv[REFP_0][MV_Y] = inital_mv[REFP_0][MV_Y];

            mv[REFP_1][MV_X] = inital_mv[REFP_1][MV_X];
            mv[REFP_1][MV_Y] = inital_mv[REFP_1][MV_Y];
        }
#endif

        if (channel != CHANNEL_C)
        {
#if BIO
            if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE)
            {
                bio_opt(pred_buf[Y_C], pred_snd[Y_C], w, h, bit_depth);
            }
            else
            {
#endif
#if SIMD_MC
                average_16b_no_clip_sse(pred_buf[Y_C], pred_snd[Y_C], pred_buf[Y_C], pred_stride, pred_stride, pred_stride, w, h);
#else
                p0 = pred_buf[Y_C];
                p1 = pred_snd[Y_C];
                for (j = 0; j < h; j++)
                {
                    for (i = 0; i < w; i++)
                    {
                        p0[i] = (p0[i] + p1[i] + 1) >> 1;
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                }
#endif
#if BIO
            }
#endif
#if BGC
            if (bgc_flag)
            {
                p0 = pred_fir;
                p1 = pred_snd[Y_C];
                dest0 = pred_buf[Y_C];
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        if (bgc_idx)
                        {
                            dest0[i] += (p0[i] - p1[i]) >> 3;
                        }
                        else
                        {
                            dest0[i] += (p1[i] - p0[i]) >> 3;
                        }
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                    dest0 += pred_stride;
                }
            }
#endif
        }

        if (channel != CHANNEL_L)
        {
#if SIMD_MC
            w >>= 1;
            h >>= 1;
            pred_stride >>= 1;
            average_16b_no_clip_sse(pred_buf[U_C], pred_snd[U_C], pred_buf[U_C], pred_stride, pred_stride, pred_stride, w, h);
            average_16b_no_clip_sse(pred_buf[V_C], pred_snd[V_C], pred_buf[V_C], pred_stride, pred_stride, pred_stride, w, h);
#else
            p0 = pred_buf[U_C];
            p1 = pred_snd[U_C];
            p2 = pred_buf[V_C];
            p3 = pred_snd[V_C];
            w >>= 1;
            h >>= 1;
            pred_stride >>= 1;
            for (j = 0; j < h; j++)
            {
                for (i = 0; i < w; i++)
                {
                    p0[i] = (p0[i] + p1[i] + 1) >> 1;
                    p2[i] = (p2[i] + p3[i] + 1) >> 1;
                }
                p0 += pred_stride;
                p1 += pred_stride;
                p2 += pred_stride;
                p3 += pred_stride;
            }
#endif
        }
    }
}

#if AWP
#if AWP_SCC
#if REDUCE_AWP_BUFFER
void com_calculate_awp_weight(pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx, int refine_flag)
#else
void com_calculate_awp_weight(pel weight0[N_C][MAX_CU_DIM], pel weight1[N_C][MAX_CU_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx, int refine_flag)
#endif
#else
void com_calculate_awp_weight(pel weight0[N_C][MAX_CU_DIM], pel weight1[N_C][MAX_CU_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx)
#endif
{
    int FirstPos = 0;
    int DeltaPos_W = 0;
    int DeltaPos_H = 0;

    // Set half pixel length
    int ValidLength_W = (nCbW + (nCbH >> angleIdx)) << 1;
    int ValidLength_H = (nCbH + (nCbW >> angleIdx)) << 1;

    // Reference weight array
    int* Finalreference = NULL;

    pel *pWeight0 = weight0[compIdx];
    pel *pWeight1 = weight1[compIdx];
    const int weightStride = nCbW;
    int temp_W = ((nCbH << 1) >> angleIdx);
    int temp_H = ((nCbW << 1) >> angleIdx);

    DeltaPos_W = (ValidLength_W >> 3) - 1;
    DeltaPos_H = (ValidLength_H >> 3) - 1;
    DeltaPos_W = DeltaPos_W == 0 ? 1 : DeltaPos_W;
    DeltaPos_H = DeltaPos_H == 0 ? 1 : DeltaPos_H;
    DeltaPos_W = modeIdx * DeltaPos_W;
    DeltaPos_H = modeIdx * DeltaPos_H;

    int ReferenceWeights[MAX_AWP_SIZE << 2] = { 0 };

    switch (subangleIdx)
    {
    case 0:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) - 3 + DeltaPos_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) - 6 + DeltaPos_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) - 6 + DeltaPos_H;
#endif

#if AWP_SCC
        if (refine_flag)
        {
            for (int i = 0; i < ValidLength_H; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, (i - FirstPos) << 2);
            }
        }
        else
        {
            for (int i = 0; i < ValidLength_H; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
            }
        }
#else
        for (int i = 0; i < ValidLength_H; i++)
        {
            ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
        }
#endif
        //set Delta to align calculate [per block]
        Finalreference = ReferenceWeights;

        // Calculate Weight [per pixel]
        for (int y = 0; y < nCbH; y++)
        {
            for (int x = 0; x < nCbW; x++)
            {
                pWeight0[x] = Finalreference[(y << 1) + ((x << 1) >> angleIdx)];
                pWeight1[x] = 8 - pWeight0[x];
            }
            pWeight0 += weightStride;
            pWeight1 += weightStride;
        }
        break;
    case 1:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) - 1 + DeltaPos_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) - 4 + DeltaPos_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) - 4 + DeltaPos_H;
#endif

#if AWP_SCC
        if (refine_flag)
        {
            for (int i = 0; i < ValidLength_H; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, (i - FirstPos) << 2);
            }
        }
        else
        {
            for (int i = 0; i < ValidLength_H; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
            }
        }
#else
        for (int i = 0; i < ValidLength_H; i++)
        {
            ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
        }
#endif

        //set Delta to align calculate [per block]
        Finalreference = ReferenceWeights + temp_H;

        // Calculate Weight [per pixel]
        for (int y = 0; y < nCbH; y++)
        {
            for (int x = 0; x < nCbW; x++)
            {
                pWeight0[x] = Finalreference[(y << 1) - ((x << 1) >> angleIdx)];
                pWeight1[x] = 8 - pWeight0[x];
            }
            pWeight0 += weightStride;
            pWeight1 += weightStride;
        }
        break;
    case 2:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) - 1 + DeltaPos_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) - 4 + DeltaPos_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) - 4 + DeltaPos_W;
#endif

#if AWP_SCC
        if (refine_flag)
        {
            for (int i = 0; i < ValidLength_W; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, (i - FirstPos) << 2);
            }
        }
        else
        {
            for (int i = 0; i < ValidLength_W; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
            }
        }
#else
        for (int i = 0; i < ValidLength_W; i++)
        {
            ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
        }
#endif

        //set Delta to align calculate [per block]
        Finalreference = ReferenceWeights + temp_W;

        // Calculate Weight [per pixel]
        for (int y = 0; y < nCbH; y++)
        {
            for (int x = 0; x < nCbW; x++)
            {
                pWeight0[x] = Finalreference[(x << 1) - ((y << 1) >> angleIdx)];
                pWeight1[x] = 8 - pWeight0[x];
            }
            pWeight0 += weightStride;
            pWeight1 += weightStride;
        }
        break;
    case 3:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) - 3 + DeltaPos_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) - 6 + DeltaPos_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) - 6 + DeltaPos_W;
#endif

#if AWP_SCC
        if (refine_flag)
        {
            for (int i = 0; i < ValidLength_W; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, (i - FirstPos) << 2);
            }
        }
        else
        {
            for (int i = 0; i < ValidLength_W; i++)
            {
                ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
            }
        }
#else
        for (int i = 0; i < ValidLength_W; i++)
        {
            ReferenceWeights[i] = COM_CLIP3(0, 8, i - FirstPos);
        }
#endif
        //set Delta to align calculate [per block]
        Finalreference = ReferenceWeights;

        // Calculate Weight [per pixel]
        for (int y = 0; y < nCbH; y++)
        {
            for (int x = 0; x < nCbW; x++)
            {
                pWeight0[x] = Finalreference[(x << 1) + ((y << 1) >> angleIdx)];
                pWeight1[x] = 8 - pWeight0[x];
            }
            pWeight0 += weightStride;
            pWeight1 += weightStride;
        }
        break;
    default:
        printf("\nError: awp parameter not expected\n");
        assert(0);
    }
}

void com_calculate_awp_para(int AwpModeIdx, int blkWidth, int blkHeight, int *stepIdx, int *angleIdx, int *subangleIdx)
{
    *stepIdx = (AwpModeIdx / (AWP_RWFERENCE_SET_NUM + 1)) - (AWP_RWFERENCE_SET_NUM >> 1);
    u8 modAngNum = AwpModeIdx % AWP_ANGLE_NUM;
    *angleIdx = modAngNum % 2;
    *angleIdx = (modAngNum == 2) ? AWP_ANGLE_HOR : ((modAngNum == 6) ? AWP_ANGLE_VER : *angleIdx);
    *subangleIdx = modAngNum >> 1;
}

#if REDUCE_AWP_BUFFER
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM])
#else
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_CU_DIM], pel weight1[N_C][MAX_CU_DIM])
#endif
{
    int stepIdx, angleIdx, subangleIdx;
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;

    com_calculate_awp_para(mod_info_curr->skip_idx, cu_width, cu_height, &stepIdx, &angleIdx, &subangleIdx); // mod_info_curr->skip_idx is awp_idx
    if (compIdx == N_C) //all comp
    {
        // derive weights for luma
#if AWP_SCC
        com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, stepIdx, angleIdx, subangleIdx, mod_info_curr->ph_awp_refine_flag);
#else
        com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, stepIdx, angleIdx, subangleIdx);
#endif

        // derive weight for chroma actually only 1 chroma is enough(top left position for each 2x2 luma area)
        int weightStrideY = cu_width << 1; // each 2 lines
        cu_width >>= 1;
        cu_height >>= 1;
        int weightStrideUV = cu_width; // cu_width >> 1
        pel *pYweight0 = weight0[Y_C];
        pel *pUweight0 = weight0[U_C];
        pel *pVweight0 = weight0[V_C];
        pel *pYweight1 = weight1[Y_C];
        pel *pUweight1 = weight1[U_C];
        pel *pVweight1 = weight1[V_C];
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
                // U_C is sub
                pUweight0[x] = pYweight0[x << 1];
                pUweight1[x] = 8 - pUweight0[x];

                // V_C is the same as U_C
                pVweight0[x] = pUweight0[x];
                pVweight1[x] = pUweight1[x];
            }
            pYweight0 += weightStrideY;
            pYweight1 += weightStrideY;
            pUweight0 += weightStrideUV;
            pUweight1 += weightStrideUV;
            pVweight0 += weightStrideUV;
            pVweight1 += weightStrideUV;
        }
    }
    else
    {
        assert(compIdx == Y_C);
#if AWP_SCC
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, stepIdx, angleIdx, subangleIdx, mod_info_curr->ph_awp_refine_flag);
#else
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, stepIdx, angleIdx, subangleIdx);
#endif
    }
}

#if REDUCE_AWP_BUFFER
void com_derive_awp_pred(COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM])
#else
void com_derive_awp_pred(COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_CU_DIM], pel weight1[MAX_CU_DIM])
#endif
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    int pred_stride = cu_width;
    if (compIdx != Y_C)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        pred_stride >>= 1;
    }
#if SIMD_MC
    weight_average_16b_no_clip_sse(pred_buf0[compIdx], pred_buf1[compIdx], mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height);
#else
    for (int j = 0; j < cu_height; j++)
    {
        for (int i = 0; i < cu_width; i++)
        {
            mod_info_curr->pred[compIdx][i + j * pred_stride] = (pred_buf0[compIdx][i + j * pred_stride] * weight0[i + j * pred_stride] + pred_buf1[compIdx][i + j * pred_stride] * weight1[i + j * pred_stride] + 4) >> 3;
        }
    }
#endif

}
#endif

#if MVAP
void com_mvap_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
)
{
    s32 cu_width  = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x         = mod_info_curr->x_pos;
    s32 y         = mod_info_curr->y_pos;
    s32 sub_w     = MIN_SUB_BLOCK_SIZE;
    s32 sub_h     = MIN_SUB_BLOCK_SIZE;
    s32 h         = 0;
    s32 w         = 0;
    s32 tmp_x     = x;
    s32 tmp_y     = y;
    pel pred_tmp[N_C][MAX_CU_DIM];
    COM_MOTION *cu_mvfield = (COM_MOTION *)tmp_cu_mvfield;

    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            x = tmp_x + w;
            y = tmp_y + h;

            mod_info_curr->mv[REFP_0][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_Y];
            mod_info_curr->refi[REFP_0] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_0];
            mod_info_curr->refi[REFP_1] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_1];

            com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
                , dmvr
#endif
#if BIO
                , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 1
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );

            if (tree_status != CHANNEL_C)
            {
                for (int i = 0; i < MIN_SUB_BLOCK_SIZE; i++)
                {
                    for (int j = 0; j < MIN_SUB_BLOCK_SIZE; j++)
                    {
                        mod_info_curr->pred[Y_C][w + j + (i + h) * cu_width] = pred_tmp[Y_C][j + i * MIN_SUB_BLOCK_SIZE];
                    }
                }
            }

            if (tree_status != CHANNEL_L)
            {
                for (int i = 0; i < MIN_CU_SIZE; i++)
                {
                    for (int j = 0; j < MIN_CU_SIZE; j++)
                    {
                        mod_info_curr->pred[U_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][j + i * MIN_CU_SIZE];
                        mod_info_curr->pred[V_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][j + i * MIN_CU_SIZE];
                    }
                }
            }
        }
    }
}
#endif
#if SUB_TMVP
void com_sbTmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, s32 sub_blk_width, s32 sub_blk_height, COM_MOTION* sbTmvp, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
)
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;
    s32 sub_w = sub_blk_width;
    s32 sub_h = sub_blk_height;
    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    pel pred_tmp[N_C][MAX_CU_DIM];
    for (int k = 0; k<SBTMVP_NUM; k++)
    {
        w = (k % 2)*sub_w;
        h = (k / 2)*sub_h;
        x = tmp_x + w;
        y = tmp_y + h;
        copy_mv(mod_info_curr->mv[REFP_0], sbTmvp[k].mv[REFP_0]);
        copy_mv(mod_info_curr->mv[REFP_1], sbTmvp[k].mv[REFP_1]);
        mod_info_curr->refi[REFP_0] = sbTmvp[k].ref_idx[REFP_0];
        mod_info_curr->refi[REFP_1] = sbTmvp[k].ref_idx[REFP_1];
        
        com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
            , dmvr
#endif
#if BIO
            , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
            , 0
#endif
#if SUB_TMVP
            , 1
#endif
#if BGC
            , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
        );

        if (tree_status != CHANNEL_C)
        {
            for (int i = 0; i < sub_w; i++)
            {
                for (int j = 0; j < sub_h; j++)
                {
                    mod_info_curr->pred[Y_C][w + i + (j + h) * cu_width] = pred_tmp[Y_C][i + j * sub_w];
                }
            }
        }
        
        if (tree_status != CHANNEL_L)
        {
            for (int i = 0; i < (sub_w >> 1); i++)
            {
                for (int j = 0; j < (sub_h >> 1); j++)
                {
                    mod_info_curr->pred[U_C][(w >> 1) + i + (j + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][i + j * (sub_w >> 1)];
                    mod_info_curr->pred[V_C][(w >> 1) + i + (j + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][i + j * (sub_w >> 1)];
                }
            }
        }
    }
}
#endif
#if ETMVP
void com_etmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
)
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;
    s32 sub_w = MIN_ETMVP_MC_SIZE;
    s32 sub_h = MIN_ETMVP_MC_SIZE;
    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    pel pred_tmp[N_C][MAX_CU_DIM];
    COM_MOTION *cu_mvfield = (COM_MOTION *)tmp_cu_mvfield;

    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            x = tmp_x + w;
            y = tmp_y + h;

            mod_info_curr->mv[REFP_0][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_Y];
            mod_info_curr->refi[REFP_0] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_0];
            mod_info_curr->refi[REFP_1] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_1];

            com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
                , dmvr
#endif
#if BIO
                , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 0
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );

            if (tree_status != CHANNEL_C)
            {
                for (int i = 0; i < MIN_ETMVP_MC_SIZE; i++)
                {
                    for (int j = 0; j < MIN_ETMVP_MC_SIZE; j++)
                    {
                        mod_info_curr->pred[Y_C][w + j + (i + h) * cu_width] = pred_tmp[Y_C][j + i * MIN_ETMVP_MC_SIZE];
                    }
                }
            }

            if (tree_status != CHANNEL_L)
            {
                for (int i = 0; i < MIN_CU_SIZE; i++)
                {
                    for (int j = 0; j < MIN_CU_SIZE; j++)
                    {
                        mod_info_curr->pred[U_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][j + i * MIN_CU_SIZE];
                        mod_info_curr->pred[V_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][j + i * MIN_CU_SIZE];
                    }
                }
            }
        }
    }
}
#endif

#if USE_IBC
void com_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], COM_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM], CHANNEL_TYPE channel, int bit_depth)
{
    int          qpel_gmv_x, qpel_gmv_y;

    int w = 1 << log2_cuw;
    int h = 1 << log2_cuh;
    int pred_stride = w;

    qpel_gmv_x = (x << 2) + mv[MV_X];
    qpel_gmv_y = (y << 2) + mv[MV_Y];

    if (channel != CHANNEL_C)
    {
        com_mc_l(mv[0], mv[1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
    }
    if (channel != CHANNEL_L)
    {
#if CHROMA_NOT_SPLIT
        assert(w >= 8 && h >= 8);
#endif
        com_mc_c_ibc(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth);
        com_mc_c_ibc(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth);
    }
}
#endif

void com_affine_mc_l(int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth)
{
    assert(com_tbl_log2[cu_width] >= 4);
    assert(com_tbl_log2[cu_height] >= 4);
    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred;
    int w, h;
    int half_w, half_h;
    s32 dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
    s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
    s32 mv_scale_tmp_hor, mv_scale_tmp_ver;
    s32 hor_max, hor_min, ver_max, ver_min;
    s32 mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;

#if CPMV_BIT_DEPTH == 18
    for (int i = 0; i < cp_num; i++)
    {
        assert(ac_mv[i][MV_X] >= COM_CPMV_MIN && ac_mv[i][MV_X] <= COM_CPMV_MAX);
        assert(ac_mv[i][MV_Y] >= COM_CPMV_MIN && ac_mv[i][MV_Y] <= COM_CPMV_MAX);
    }
#endif

    // get clip MV Range
    hor_max = (pic_w + MAX_CU_SIZE + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE - 4 - y) << 4;

    // get sub block size
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_width];      // deltaMvHor
    dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_width];
    if (cp_num == 3)
    {
        dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_height]; // deltaMvVer
        dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_height];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                                    // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

    // get prediction block by block
    for (h = 0; h < cu_height; h += sub_h)
    {
        for(w = 0; w < cu_width; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

            if (w == 0 && h == 0)
            {
                pos_x = 0;
                pos_y = 0;
            }
            else if (w + sub_w == cu_width && h == 0)
            {
                pos_x = cu_width;
                pos_y = 0;
            }
            else if (w == 0 && h + sub_h == cu_height && cp_num == 3)
            {
                pos_x = 0;
                pos_y = cu_height;
            }

            mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
            mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

            // 1/16 precision, 18 bits, for MC
#if BD_AFFINE_AMVR
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
#else
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#endif
            mv_scale_tmp_hor = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_hor);
            mv_scale_tmp_ver = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_ver);

            // clip
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
            com_mc_l_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, (pred_y + w), sub_w, sub_h, bit_depth );
        }
        pred_y += (cu_width * sub_h);
    }
}

void com_affine_mc_lc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, pel pred[N_C][MAX_CU_DIM], int sub_w, int sub_h, int lidx, int bit_depth)
{
    int scup = mod_info_curr->scup;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;
    int cp_num = mod_info_curr->affine_flag + 1;
    s8 *refi = mod_info_curr->refi;
    COM_PIC* ref_pic = ref_pic = refp[refi[lidx]][lidx].pic;
    CPMV(*ac_mv)[MV_D] = mod_info_curr->affine_mv[lidx];
    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;

    assert(com_tbl_log2[cu_width] >= 4);
    assert(com_tbl_log2[cu_height] >= 4);

    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred[Y_C], *pred_u = pred[U_C], *pred_v = pred[V_C];
    int w, h;
    int half_w, half_h;
    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
    s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
    s32 mv_scale_tmp_hor, mv_scale_tmp_ver;
    s32 hor_max, hor_min, ver_max, ver_min;
    s32 mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;
    s32 mv_save[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MV_D];

#if CPMV_BIT_DEPTH == 18
    for (int i = 0; i < cp_num; i++)
    {
        assert(ac_mv[i][MV_X] >= COM_CPMV_MIN && ac_mv[i][MV_X] <= COM_CPMV_MAX);
        assert(ac_mv[i][MV_Y] >= COM_CPMV_MIN && ac_mv[i][MV_Y] <= COM_CPMV_MAX);
    }
#endif

    // get clip MV Range
    hor_max = (pic_w + MAX_CU_SIZE + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE - 4 - y) << 4;

    // get sub block size
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_width];      // deltaMvHor
    dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_width];
    if (cp_num == 3)
    {
        dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_height]; // deltaMvVer
        dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_height];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                                    // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

    // get prediction block by block (luma)
    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

            if (w == 0 && h == 0)
            {
                pos_x = 0;
                pos_y = 0;
            }
            else if (w + sub_w == cu_width && h == 0)
            {
                pos_x = cu_width;
                pos_y = 0;
            }
            else if (w == 0 && h + sub_h == cu_height && cp_num == 3)
            {
                pos_x = 0;
                pos_y = cu_height;
            }

            mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
            mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

            // 1/16 precision, 18 bits, for MC
#if BD_AFFINE_AMVR
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
#else
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#endif
            mv_scale_tmp_hor = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_hor);
            mv_scale_tmp_ver = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_ver);

#if AFFINE_MVF_VERIFY
            {
                int addr_scu = scup + (w >> MIN_CU_LOG2) + (h >> MIN_CU_LOG2) * pic_width_in_scu;
                s16 rounded_hor, rounded_ver;
                com_mv_rounding_s16(mv_scale_tmp_hor, mv_scale_tmp_ver, &rounded_hor, &rounded_ver, 2, 0); // 1/4 precision
                assert(rounded_hor == map_mv[addr_scu][lidx][MV_X]);
                assert(rounded_ver == map_mv[addr_scu][lidx][MV_Y]);
            }
#endif

            // save MVF for chroma interpolation
            int w_scu = PEL2SCU(w);
            int h_scu = PEL2SCU(h);
            mv_save[w_scu][h_scu][MV_X] = mv_scale_tmp_hor;
            mv_save[w_scu][h_scu][MV_Y] = mv_scale_tmp_ver;
            if (sub_w == 8 && sub_h == 8)
            {
                mv_save[w_scu + 1][h_scu][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu + 1][h_scu][MV_Y] = mv_scale_tmp_ver;
                mv_save[w_scu][h_scu + 1][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu][h_scu + 1][MV_Y] = mv_scale_tmp_ver;
                mv_save[w_scu + 1][h_scu + 1][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu + 1][h_scu + 1][MV_Y] = mv_scale_tmp_ver;
            }

            // clip
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
            com_mc_l_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, (pred_y + w), sub_w, sub_h, bit_depth );
        }
        pred_y += (cu_width * sub_h);
    }

    // get prediction block by block (chroma)
    sub_w = 8;
    sub_h = 8;
    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            int w_scu = PEL2SCU(w);
            int h_scu = PEL2SCU(h);

            mv_scale_tmp_hor = (mv_save[w_scu][h_scu][MV_X] + mv_save[w_scu + 1][h_scu][MV_X] + mv_save[w_scu][h_scu + 1][MV_X] + mv_save[w_scu + 1][h_scu + 1][MV_X] + 2) >> 2;
            mv_scale_tmp_ver = (mv_save[w_scu][h_scu][MV_Y] + mv_save[w_scu + 1][h_scu][MV_Y] + mv_save[w_scu][h_scu + 1][MV_Y] + mv_save[w_scu + 1][h_scu + 1][MV_Y] + 2) >> 2;
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
            com_mc_c_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_u + (w >> 1), sub_w >> 1, sub_h >> 1, bit_depth );
            com_mc_c_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_v + (w >> 1), sub_w >> 1, sub_h >> 1, bit_depth );
        }
        pred_u += (cu_width * sub_h) >> 2;
        pred_v += (cu_width * sub_h) >> 2;
    }
}

void com_affine_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int bit_depth)
{
    int scup = mod_info_curr->scup;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int w = mod_info_curr->cu_width; 
    int h = mod_info_curr->cu_height;
    
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;
    
    int vertex_num = mod_info_curr->affine_flag + 1;
    s8 *refi = mod_info_curr->refi; 
    CPMV (*mv)[VER_NUM][MV_D] = mod_info_curr->affine_mv;
    pel (*pred_buf)[MAX_CU_DIM] = mod_info_curr->pred;
#if BGC
    u8  bgc_flag = mod_info_curr->bgc_flag;
    u8  bgc_idx = mod_info_curr->bgc_idx;
    pel *pred_fir = info->pred_tmp;
    pel *p0, *p1, *dest;
#endif 

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;

    pel pred_snd[N_C][MAX_CU_DIM];
    pel(*pred)[MAX_CU_DIM] = pred_buf;
#if SIMD_MC
    int bidx = 0;
#else
    pel *p0, *p1, *p2, *p3;
    int i, j, bidx = 0;
#endif

    int sub_w = 4;
    int sub_h = 4;
    if (sh->affine_subblock_size_idx == 1)
    {
        sub_w = 8;
        sub_h = 8;
    }
    if (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
        sub_w = 8;
        sub_h = 8;
    }

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        com_affine_mc_lc(info, mod_info_curr, refp, pic_map, pred, sub_w, sub_h, REFP_0, bit_depth);
        bidx++;
#if BGC
        if (bgc_flag)
        {
            p0 = pred_buf[Y_C];
            dest = pred_fir;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    dest[i] = p0[i];
                }
                p0 += w;
                dest += w;
            }
        }
#endif
    }

    if(REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        if (bidx)
        {
            pred = pred_snd;
        }
        com_affine_mc_lc(info, mod_info_curr, refp, pic_map, pred, sub_w, sub_h, REFP_1, bit_depth);
        bidx++;
    }

    if(bidx == 2)
    {
#if SIMD_MC
        average_16b_no_clip_sse( pred_buf[Y_C], pred_snd[Y_C], pred_buf[Y_C], w, w, w, w, h );
        average_16b_no_clip_sse( pred_buf[U_C], pred_snd[U_C], pred_buf[U_C], w >> 1, w >> 1, w >> 1, w >> 1, h >> 1 );
        average_16b_no_clip_sse( pred_buf[V_C], pred_snd[V_C], pred_buf[V_C], w >> 1, w >> 1, w >> 1, w >> 1, h >> 1 );
#else
        p0 = pred_buf[Y_C];
        p1 = pred_snd[Y_C];
        for(j = 0; j < h; j++)
        {
            for(i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
        }
        p0 = pred_buf[U_C];
        p1 = pred_snd[U_C];
        p2 = pred_buf[V_C];
        p3 = pred_snd[V_C];
        w >>= 1;
        h >>= 1;
        for(j = 0; j < h; j++)
        {
            for(i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
                p2[i] = (p2[i] + p3[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
            p2 += w;
            p3 += w;
        }
#endif
#if BGC
        if (bgc_flag)
        {
            dest = pred_buf[Y_C];
            p0 = pred_fir;
            p1 = pred_snd[Y_C];
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    if (bgc_idx)
                    {
                        dest[i] += (p0[i] - p1[i]) >> 3;
                    }
                    else
                    {
                        dest[i] += (p1[i] - p0[i]) >> 3;
                    }
                }
                p0 += w;
                p1 += w;
                dest += w;
            }
        }
#endif
    }
}

#if AWP
void com_awp_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth)
{
    s32 cu_width  = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;

    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    pel pred_awp_tmp0[N_C][MAX_CU_DIM];
    pel pred_awp_tmp1[N_C][MAX_CU_DIM];
#if REDUCE_AWP_BUFFER
    pel awp_weight0[N_C][MAX_AWP_DIM];
    pel awp_weight1[N_C][MAX_AWP_DIM];
#else
    pel awp_weight0[N_C][MAX_CU_DIM];
    pel awp_weight1[N_C][MAX_CU_DIM];
#endif

#if DMVR
    /* disable DMVR*/
    COM_DMVR dmvr;
    dmvr.poc_c = 0;
    dmvr.apply_DMVR = 0;
#endif

    /* get two pred buf */
    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi0[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi0[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp0, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi1[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi1[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp1, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    mod_info_curr->mv[REFP_0][MV_X] = 0;
    mod_info_curr->mv[REFP_0][MV_Y] = 0;
    mod_info_curr->mv[REFP_1][MV_X] = 0;
    mod_info_curr->mv[REFP_1][MV_Y] = 0;
    mod_info_curr->refi[REFP_0] = REFI_INVALID;
    mod_info_curr->refi[REFP_1] = REFI_INVALID;

    if (tree_status == CHANNEL_LC)
    {
        /* derive weights */
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1);

        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
        com_derive_awp_pred(mod_info_curr, U_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[U_C], awp_weight1[U_C]);
        com_derive_awp_pred(mod_info_curr, V_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[V_C], awp_weight1[V_C]);
    }
    else // only for luma
    {
        /* derive weights */
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1);

        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
    }
}
#endif
