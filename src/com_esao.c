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

#include "com_esao.h"
#include <mmintrin.h>
#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
#if LINUX
#include <cpuid.h>
#else
#include <intrin.h>
#endif 
#if ESAO
void esao_on_block_for_luma_avx(pel* p_dst, int i_dst, pel * p_src, int  i_src, int i_block_w, int  i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int     x, y, z;
    int     shift_bo = bit_depth;
    int end_x_r0_16, start_x_r, end_x_r, start_y, end_y;
    __m256i s0, s1, s2, s4, s3, s5, s6, s7, s8;
    __m256i t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, etype;
    __m128i mask;
    start_x_r = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x_r = (lcu_available_right) ? i_block_w : i_block_w - 1;
    end_x_r0_16 = end_x_r - ((end_x_r - start_x_r) & 0x0f);
    const short max_pixel = (1 << bit_depth) - 1;
    __m256i     min_val = _mm256_setzero_si256();
    __m256i     max_val = _mm256_set1_epi16(max_pixel);
    __m256i     c4 = _mm256_set1_epi16(bo_value);
    if (luma_type == 0)
    {
        __m256i     c3 = _mm256_set1_epi16(NUM_ESAO_LUMA_TYPE0);
        __m256i     c2 = _mm256_set1_epi16(8);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;

        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 16)
            {
                s0 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src - 1]);
                s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
                s2 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src + 1]);
                s3 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src]);
                s4 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src + 1]);
                s5 = _mm256_loadu_si256((__m256i *)&p_src[x - 1]);
                s6 = _mm256_loadu_si256((__m256i *)&p_src[x + 1]);
                s7 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src - 1]);
                s8 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src]);

                t3 = _mm256_min_epu16(s0, s1);
                t1 = _mm256_cmpeq_epi16(t3, s0);
                t2 = _mm256_cmpeq_epi16(t3, s1);
                t0 = _mm256_subs_epi16(t1, t2);

                t3 = _mm256_min_epu16(s2, s1);
                t1 = _mm256_cmpeq_epi16(t3, s1);
                t2 = _mm256_cmpeq_epi16(t3, s2);
                t3 = _mm256_subs_epi16(t2, t1);

                t4 = _mm256_min_epu16(s1, s3);
                t1 = _mm256_cmpeq_epi16(t4, s1);
                t2 = _mm256_cmpeq_epi16(t4, s3);
                t4 = _mm256_subs_epi16(t2, t1);

                t5 = _mm256_min_epu16(s1, s4);
                t1 = _mm256_cmpeq_epi16(t5, s1);
                t2 = _mm256_cmpeq_epi16(t5, s4);
                t5 = _mm256_subs_epi16(t2, t1);

                t6 = _mm256_min_epu16(s1, s5);
                t1 = _mm256_cmpeq_epi16(t6, s1);
                t2 = _mm256_cmpeq_epi16(t6, s5);
                t6 = _mm256_subs_epi16(t2, t1);

                t7 = _mm256_min_epu16(s1, s6);
                t1 = _mm256_cmpeq_epi16(t7, s1);
                t2 = _mm256_cmpeq_epi16(t7, s6);
                t7 = _mm256_subs_epi16(t2, t1);

                t8 = _mm256_min_epu16(s1, s7);
                t1 = _mm256_cmpeq_epi16(t8, s1);
                t2 = _mm256_cmpeq_epi16(t8, s7);
                t8 = _mm256_subs_epi16(t2, t1);

                t9 = _mm256_min_epu16(s1, s8);
                t1 = _mm256_cmpeq_epi16(t9, s1);
                t2 = _mm256_cmpeq_epi16(t9, s8);
                t9 = _mm256_subs_epi16(t2, t1);

                etype = _mm256_add_epi16(t0, t3);
                etype = _mm256_add_epi16(etype, t4);
                etype = _mm256_add_epi16(etype, t5);
                etype = _mm256_add_epi16(etype, t6);
                etype = _mm256_add_epi16(etype, t7);
                etype = _mm256_add_epi16(etype, t8);
                etype = _mm256_add_epi16(etype, t9);
                etype = _mm256_adds_epi16(etype, c2);

                t10 = _mm256_mullo_epi16(s1, c4);
                t10 = _mm256_srli_epi16(t10, shift_bo);
                t10 = _mm256_mullo_epi16(t10, c3);
                etype = _mm256_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 16; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
                t0 = _mm256_add_epi16(t0, s1);
                t0 = _mm256_min_epi16(t0, max_val);
                t0 = _mm256_max_epi16(t0, min_val);

                if (x != end_x_r0_16)
                {
                    _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
                }
                else
                {
                    if (end_x_r - x >= 8)
                    {
                        _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));

                        if (end_x_r - x > 8)
                        {
                            mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 9]));
                            _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                        }
                    }
                    else
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                        _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                    }
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        __m256i     c3 = _mm256_set1_epi16(NUM_ESAO_LUMA_TYPE1);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 16)
            {
                s0 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src - 1]);
                s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
                s2 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src + 1]);
                s3 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src]);
                s4 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src + 1]);
                s5 = _mm256_loadu_si256((__m256i *)&p_src[x - 1]);
                s6 = _mm256_loadu_si256((__m256i *)&p_src[x + 1]);
                s7 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src - 1]);
                s8 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src]);

                t0 = _mm256_cmpgt_epi16(s0, s1);
                t1 = _mm256_cmpgt_epi16(s2, s1);
                t2 = _mm256_cmpgt_epi16(s3, s1);
                t3 = _mm256_cmpgt_epi16(s4, s1);
                t4 = _mm256_cmpgt_epi16(s5, s1);
                t5 = _mm256_cmpgt_epi16(s6, s1);
                t6 = _mm256_cmpgt_epi16(s7, s1);
                t7 = _mm256_cmpgt_epi16(s8, s1);

                etype = _mm256_add_epi16(t0, t1);
                etype = _mm256_add_epi16(etype, t2);
                etype = _mm256_add_epi16(etype, t3);
                etype = _mm256_add_epi16(etype, t4);
                etype = _mm256_add_epi16(etype, t5);
                etype = _mm256_add_epi16(etype, t6);
                etype = _mm256_add_epi16(etype, t7);
                etype = _mm256_abs_epi16(etype);

                t10 = _mm256_mullo_epi16(s1, c4);
                t10 = _mm256_srli_epi16(t10, shift_bo);
                t10 = _mm256_mullo_epi16(t10, c3);
                etype = _mm256_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 16; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
                t0 = _mm256_add_epi16(t0, s1);
                t0 = _mm256_min_epi16(t0, max_val);
                t0 = _mm256_max_epi16(t0, min_val);

                if (x != end_x_r0_16)
                {
                    _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
                }
                else
                {
                    if (end_x_r - x >= 8)
                    {
                        _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));

                        if (end_x_r - x > 8)
                        {
                            mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 9]));
                            _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                        }
                    }
                    else
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                        _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                    }
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_luma_sse(pel* p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int     x, y, z;
    int     shift_bo = bit_depth;
    int end_x_r0_16, start_x_r, end_x_r, start_y, end_y;
    __m128i s0, s1, s2, s4, s3, s5, s6, s7, s8;
    __m128i t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, etype;
    __m128i mask;

    start_x_r = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x_r = (lcu_available_right) ? i_block_w : i_block_w - 1;
    end_x_r0_16 = end_x_r - ((end_x_r - start_x_r) & 0x7);
    const short max_pixel = (1 << bit_depth) - 1;
    __m128i     c4 = _mm_set1_epi16(bo_value);
    __m128i     min_val = _mm_setzero_si128();
    __m128i     max_val = _mm_set1_epi16(max_pixel);
    if (luma_type == 0)
    {
        __m128i     c3 = _mm_set1_epi16(NUM_ESAO_LUMA_TYPE0);
        __m128i     c2 = _mm_set1_epi16(8);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++) {
            for (x = start_x_r; x < end_x_r; x += 8) {
                s0 = _mm_loadu_si128((__m128i *)&p_src[x - i_src - 1]);
                s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
                s2 = _mm_loadu_si128((__m128i *)&p_src[x + i_src + 1]);
                s3 = _mm_loadu_si128((__m128i *)&p_src[x - i_src]);
                s4 = _mm_loadu_si128((__m128i *)&p_src[x - i_src + 1]);
                s5 = _mm_loadu_si128((__m128i *)&p_src[x - 1]);
                s6 = _mm_loadu_si128((__m128i *)&p_src[x + 1]);
                s7 = _mm_loadu_si128((__m128i *)&p_src[x + i_src - 1]);
                s8 = _mm_loadu_si128((__m128i *)&p_src[x + i_src]);

                t3 = _mm_min_epu16(s0, s1);
                t1 = _mm_cmpeq_epi16(t3, s0);
                t2 = _mm_cmpeq_epi16(t3, s1);
                t0 = _mm_subs_epi16(t1, t2);

                t3 = _mm_min_epu16(s2, s1);
                t1 = _mm_cmpeq_epi16(t3, s1);
                t2 = _mm_cmpeq_epi16(t3, s2);
                t3 = _mm_subs_epi16(t2, t1);

                t4 = _mm_min_epu16(s1, s3);
                t1 = _mm_cmpeq_epi16(t4, s1);
                t2 = _mm_cmpeq_epi16(t4, s3);
                t4 = _mm_subs_epi16(t2, t1);

                t5 = _mm_min_epu16(s1, s4);
                t1 = _mm_cmpeq_epi16(t5, s1);
                t2 = _mm_cmpeq_epi16(t5, s4);
                t5 = _mm_subs_epi16(t2, t1);

                t6 = _mm_min_epu16(s1, s5);
                t1 = _mm_cmpeq_epi16(t6, s1);
                t2 = _mm_cmpeq_epi16(t6, s5);
                t6 = _mm_subs_epi16(t2, t1);

                t7 = _mm_min_epu16(s1, s6);
                t1 = _mm_cmpeq_epi16(t7, s1);
                t2 = _mm_cmpeq_epi16(t7, s6);
                t7 = _mm_subs_epi16(t2, t1);

                t8 = _mm_min_epu16(s1, s7);
                t1 = _mm_cmpeq_epi16(t8, s1);
                t2 = _mm_cmpeq_epi16(t8, s7);
                t8 = _mm_subs_epi16(t2, t1);

                t9 = _mm_min_epu16(s1, s8);
                t1 = _mm_cmpeq_epi16(t9, s1);
                t2 = _mm_cmpeq_epi16(t9, s8);
                t9 = _mm_subs_epi16(t2, t1);

                etype = _mm_add_epi16(t0, t3);
                etype = _mm_add_epi16(etype, t4);
                etype = _mm_add_epi16(etype, t5);
                etype = _mm_add_epi16(etype, t6);
                etype = _mm_add_epi16(etype, t7);
                etype = _mm_add_epi16(etype, t8);
                etype = _mm_add_epi16(etype, t9);
                etype = _mm_adds_epi16(etype, c2);

                t10 = _mm_mullo_epi16(s1, c4);
                t10 = _mm_srli_epi16(t10, shift_bo);
                t10 = _mm_mullo_epi16(t10, c3);
                etype = _mm_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 8; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm_loadu_si128((__m128i *)&t[0]);
                t0 = _mm_add_epi16(t0, s1);
                t0 = _mm_min_epi16(t0, max_val);
                t0 = _mm_max_epi16(t0, min_val);

                if (x != end_x_r0_16) {
                    _mm_storeu_si128((__m128i *)(p_dst + x), t0);
                }
                else {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                    _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                    break;
                }

            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        __m128i     c3 = _mm_set1_epi16(NUM_ESAO_LUMA_TYPE1);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 8)
            {
                s0 = _mm_loadu_si128((__m128i *)&p_src[x - i_src - 1]);
                s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
                s2 = _mm_loadu_si128((__m128i *)&p_src[x + i_src + 1]);
                s3 = _mm_loadu_si128((__m128i *)&p_src[x - i_src]);
                s4 = _mm_loadu_si128((__m128i *)&p_src[x - i_src + 1]);
                s5 = _mm_loadu_si128((__m128i *)&p_src[x - 1]);
                s6 = _mm_loadu_si128((__m128i *)&p_src[x + 1]);
                s7 = _mm_loadu_si128((__m128i *)&p_src[x + i_src - 1]);
                s8 = _mm_loadu_si128((__m128i *)&p_src[x + i_src]);

                t0 = _mm_cmpgt_epi16(s0, s1);
                t1 = _mm_cmpgt_epi16(s2, s1);
                t2 = _mm_cmpgt_epi16(s3, s1);
                t3 = _mm_cmpgt_epi16(s4, s1);
                t4 = _mm_cmpgt_epi16(s5, s1);
                t5 = _mm_cmpgt_epi16(s6, s1);
                t6 = _mm_cmpgt_epi16(s7, s1);
                t7 = _mm_cmpgt_epi16(s8, s1);

                etype = _mm_add_epi16(t0, t1);
                etype = _mm_add_epi16(etype, t2);
                etype = _mm_add_epi16(etype, t3);
                etype = _mm_add_epi16(etype, t4);
                etype = _mm_add_epi16(etype, t5);
                etype = _mm_add_epi16(etype, t6);
                etype = _mm_add_epi16(etype, t7);
                etype = _mm_abs_epi16(etype);

                t10 = _mm_mullo_epi16(s1, c4);
                t10 = _mm_srli_epi16(t10, shift_bo);
                t10 = _mm_mullo_epi16(t10, c3);
                etype = _mm_add_epi16(t10, etype);
                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 8; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm_loadu_si128((__m128i *)&(t[0]));
                t0 = _mm_add_epi16(t0, s1);
                t0 = _mm_min_epi16(t0, max_val);
                t0 = _mm_max_epi16(t0, min_val);

                if (x != end_x_r0_16) {
                    _mm_storeu_si128((__m128i *)(p_dst + x), t0);
                }
                else {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                    _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_luma_without_simd(pel* p_dst, int  i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int start_x, end_x, start_y, end_y, x, y;
    start_x = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x = (lcu_available_right) ? i_block_w : i_block_w - 1;
    p_dst += start_y * i_dst;
    p_src += start_y * i_src;
    if (luma_type == 0)
    {
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x; x < end_x; x++)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                //leftup
                if (p_src[x - 1 - i_src] > p_src[x])
                {
                    diff1 = 1;
                }
                else if (p_src[x - 1 - i_src] < p_src[x])
                {
                    diff1 = -1;
                }
                //up
                if (p_src[x - i_src] > p_src[x])
                {
                    diff2 = 1;
                }
                else if (p_src[x - i_src] < p_src[x])
                {
                    diff2 = -1;
                }
                //rightUp
                if (p_src[x + 1 - i_src] > p_src[x])
                {
                    diff3 = 1;
                }
                else if (p_src[x + 1 - i_src] < p_src[x])
                {
                    diff3 = -1;
                }
                //left
                if (p_src[x - 1] > p_src[x])
                {
                    diff4 = 1;
                }
                else if (p_src[x - 1] < p_src[x])
                {
                    diff4 = -1;
                }
                //right
                if (p_src[x + 1] > p_src[x])
                {
                    diff5 = 1;
                }
                else if (p_src[x + 1] < p_src[x])
                {
                    diff5 = -1;
                }
                //leftdown
                if (p_src[x - 1 + i_src] > p_src[x])
                {
                    diff6 = 1;
                }
                else if (p_src[x - 1 + i_src] < p_src[x])
                {
                    diff6 = -1;
                }
                //down
                if (p_src[x + i_src] > p_src[x])
                {
                    diff7 = 1;
                }
                else if (p_src[x + i_src] < p_src[x])
                {
                    diff7 = -1;
                }
                //rightDown
                if (p_src[x + 1 + i_src] > p_src[x])
                {
                    diff8 = 1;
                }
                else if (p_src[x + 1 + i_src] < p_src[x])
                {
                    diff8 = -1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                int band_type = (p_src[x] * bo_value) >> bit_depth;
                int true_index = band_type * NUM_ESAO_LUMA_TYPE0 + diff_count;
                p_dst[x] = (pel)COM_CLIP3(0, ((1 << bit_depth) - 1), p_src[x] + esao_offset[true_index]);
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x; x < end_x; x++)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                if (p_src[x - 1 - i_src] > p_src[x])
                {
                    diff1 = 1;
                }
                if (p_src[x - i_src] > p_src[x])
                {
                    diff2 = 1;
                }
                if (p_src[x + 1 - i_src] > p_src[x])
                {
                    diff3 = 1;
                }
                if (p_src[x - 1] > p_src[x])
                {
                    diff4 = 1;
                }
                if (p_src[x + 1] > p_src[x])
                {
                    diff5 = 1;
                }
                if (p_src[x - 1 + i_src] > p_src[x])
                {
                    diff6 = 1;
                }
                if (p_src[x + i_src] > p_src[x])
                {
                    diff7 = 1;
                }
                if (p_src[x + 1 + i_src] > p_src[x])
                {
                    diff8 = 1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8;
                int band_type = (p_src[x] * bo_value) >> bit_depth;
                int true_index = band_type * NUM_ESAO_LUMA_TYPE1 + diff_count;
                p_dst[x] = (pel)COM_CLIP3(0, ((1 << bit_depth) - 1), p_src[x] + esao_offset[true_index]);
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_chroma_avx(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    __m256i t0, src0, src1;
    __m256i     c1 = _mm256_set1_epi16(bo_value);
    __m128i mask = _mm_setzero_si128();
    __m256i s1;
    int     x, y, z;
    int     shift_bo = shift_value;
    const short   max_pixel = (1 << bit_depth) - 1;
    const __m256i min_val = _mm256_setzero_si256();
    __m256i max_val = _mm256_set1_epi16(max_pixel);
    int     end_x = i_block_w;
    int     end_x_16 = end_x - ((end_x - 0) & 0x0f);

    for (y = 0; y < i_block_h; y++)
    {
        for (x = 0; x < i_block_w; x += 16)
        {
            s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
            src0 = _mm256_mullo_epi16(s1, c1);
            src1 = _mm256_srli_epi16(src0, shift_bo);
            s16 *p_type = (s16*)&src1;
            s16 *t = (s16*)&t0;
            for (z = 0; z < 16; z++)
            {
                t[z] = esao_offset[p_type[z]];
            }
            t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
            t0 = _mm256_add_epi16(t0, s1);
            t0 = _mm256_min_epi16(t0, max_val);
            t0 = _mm256_max_epi16(t0, min_val);
            if (x < end_x_16)
            {
                _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
            }
            else
            {
                if (end_x - x >= 8)
                {
                    _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));
                    if (end_x - x > 8)
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 9]));
                        _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                    }
                }
                else
                {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 1]));
                    _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                }
                break;
            }
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

void esao_on_block_for_chroma_sse(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    __m128i t0, src0, src1;
    __m128i     c1 = _mm_set1_epi16(bo_value);
    __m128i mask = _mm_setzero_si128();
    __m128i s1;
    int     x, y, z;
    int     shift_bo = shift_value;
    const short   max_pixel = (1 << bit_depth) - 1;
    const __m128i min_val = _mm_setzero_si128();
    __m128i max_val = _mm_set1_epi16(max_pixel);
    int     end_x = i_block_w;
    int     end_x_16 = end_x - ((end_x - 0) & 0x7);

    for (y = 0; y < i_block_h; y++) {
        for (x = 0; x < i_block_w; x += 8) {
            s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
            src0 = _mm_mullo_epi16(s1, c1);
            src1 = _mm_srli_epi16(src0, shift_bo);
            s16 *p_type = (s16*)&src1;
            s16 *t = (s16*)&t0;
            for (z = 0; z < 8; z++)
            {
                t[z] = esao_offset[p_type[z]];
            }

            t0 = _mm_add_epi16(t0, s1);
            t0 = _mm_min_epi16(t0, max_val);
            t0 = _mm_max_epi16(t0, min_val);

            if (x < end_x_16) {
                _mm_storeu_si128((__m128i *)(p_dst + x), t0);
            }
            else {
                mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 1]));
                _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                break;
            }
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

void esao_on_block_for_chroma_without_simd(pel * p_dst, int i_dst, pel * p_src, int  i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    int x, y, z;
    int shift_bo = shift_value;
    int value_bo = bo_value;
    const int   max_pixel = (1 << bit_depth) - 1;
    for (y = 0; y < i_block_h; y++)
    {
        for (x = 0; x < i_block_w; x++)
        {
            z = (p_src[x] * value_bo) >> shift_bo;
            p_dst[x] = (pel)COM_CLIP3(0, max_pixel, p_src[x] + esao_offset[z]);
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

int is_support_sse_avx()
{
#if LINUX
    int avail = 0;
    __builtin_cpu_init();
    if (__builtin_cpu_supports("sse4.2"))
    {
        avail = 1;
    }
    if (__builtin_cpu_supports("avx2"))
    {
        avail = 2;
    }
    return avail;
#else
    int cpu_info[4] = { 0,0,0,0 };
    int avail = 0;
    //check sse4.2 available
    __cpuid(cpu_info, 1);
    if ((cpu_info[2] & 0x00100000) != 0)
    {
        avail = 1;
    }
    //check AVX2 available
    if ((cpu_info[2] & 0x18000000) == 0x18000000)
    {
        unsigned long long xcr_feature_mask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
        if ((xcr_feature_mask & 0x6) == 0x6) // Check for OS support 
        {
            memset(cpu_info, 0, sizeof(int) * 4);
            __cpuid(cpu_info, 7);
            if ((cpu_info[1] & 0x00000020) != 0)
            {
                avail = 2;
            }
        }
    }
    return avail;  // 0:C  1:SSE  2:AVX
#endif
}

void decide_esao_filter_func_pointer(ESAOFuncPointer *func_esao_filter)
{
    int avail = is_support_sse_avx();
    if (avail == 2)
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_avx;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_avx;
    }
    else if (avail == 1)
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_sse;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_sse;
    }
    else
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_without_simd;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_without_simd;
    }
}

int com_malloc_2d_esao_statdate(ESAOStatData *** array2D, int num_SMB, int num_comp)
{
    int i;
    if (((*array2D) = (ESAOStatData **)calloc(num_SMB, sizeof(ESAOStatData *))) == NULL)
    {
        printf("MALLOC FAILED: get_mem2DESAOBlkParam: array2D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_SMB; i++)
    {
        if ((*(*array2D + i) = (ESAOStatData *)calloc(num_comp, sizeof(ESAOStatData))) == NULL)
        {
            printf("MALLOC FAILED: get_mem1DESAOBlkParam: array1D");
            assert(0);
            exit(-1);
        }
    }
    return num_SMB * num_comp * sizeof(ESAOStatData);
}

void com_free_2d_esao_statdate(ESAOStatData **array2D, int num_SMB)
{
    int i;
    if (array2D)
    {
        for (i = 0; i < num_SMB; i++)
        {
            if (array2D[i])
            {
                free(array2D[i]);
                array2D[i] = NULL;
            }
        }
    }
}

void copy_esao_param_for_one_component(ESAOBlkParam *esao_para_dst, ESAOBlkParam *esao_para_src)
{
    int  j;
    for (j = 0; j < ESAO_LABEL_CLASSES_MAX; j++)
    {
        esao_para_dst->offset[j] = esao_para_src->offset[j];
    }
}

void copy_frame_for_esao(COM_PIC * pic_dst, COM_PIC * pic_src)
{
    int i, j;
    int src_stride, dst_stride;
    pel* src;
    pel* dst;
    src_stride = pic_src->stride_luma;
    dst_stride = pic_dst->stride_luma;
    src = pic_src->y;
    dst = pic_dst->y;
    for (j = 0; j < pic_src->height_luma; j++)
    {
        for (i = 0; i < pic_src->width_luma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src_stride = pic_src->stride_chroma;
    dst_stride = pic_dst->stride_chroma;
    src = pic_src->u;
    dst = pic_dst->u;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src = pic_src->v;
    dst = pic_dst->v;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
}

static BOOL is_same_patch(s8* map_patch_idx, int mb_nr1, int mb_nr2)
{
    assert(mb_nr1 >= 0);
    assert(mb_nr2 >= 0);
    return (map_patch_idx[mb_nr1] == map_patch_idx[mb_nr2]);
}

void check_boundary_available_for_esao(COM_INFO *info, COM_MAP *map, int pix_y, int pix_x, int lcu_pix_height, int lcu_pix_width, int comp, int *lcu_process_left, int *lcu_process_right,
    int *lcu_process_up, int *lcu_process_down, int *lcu_process_upleft, int *lcu_process_upright, int *lcu_process_leftdown, int *lcu_process_rightdwon, int filter_on)
{
    int pic_width = comp ? (info->pic_width >> 1) : info->pic_width;
    int pic_height = comp ? (info->pic_height >> 1) : info->pic_height;
    int pic_mb_width = info->pic_width_in_scu;
    int mb_size_in_bit = comp ? (MIN_CU_LOG2 - 1) : MIN_CU_LOG2;
    int mb_nr_cur, mb_nr_up, mb_nr_down, mb_nr_left, mb_nr_right, mb_nr_upleft, mb_nr_upright, mb_nr_leftdown, mb_nr_rightdown;
    s8 *map_patch_idx = map->map_patch_idx;
    int cross_patch_flag = info->sqh.cross_patch_loop_filter;

    mb_nr_cur = (pix_y >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_up = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_down = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_left = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_right = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_upleft = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_upright = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_leftdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_rightdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >>
        mb_size_in_bit);
    *lcu_process_up = (pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_up) ? 1 :
        cross_patch_flag;
    *lcu_process_down = (pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_down) ? 1 : cross_patch_flag;
    *lcu_process_left = (pix_x == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_left) ? 1 :
        cross_patch_flag;
    *lcu_process_right = (pix_x >= pic_width - lcu_pix_width) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_right) ? 1 : cross_patch_flag;
    *lcu_process_upleft = (pix_x == 0 ||
        pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upleft) ? 1 : cross_patch_flag;
    *lcu_process_upright = (pix_x >= pic_width - lcu_pix_width ||
        pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upright) ? 1 : cross_patch_flag;
    *lcu_process_leftdown = (pix_x == 0 ||
        pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_leftdown)
        ? 1 : cross_patch_flag;
    *lcu_process_rightdwon = (pix_x >= pic_width - lcu_pix_width ||
        pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_rightdown)
        ? 1 : cross_patch_flag;
    if (!filter_on)
    {
        *lcu_process_down = 0;
        *lcu_process_right = 0;
        *lcu_process_leftdown = 0;
        *lcu_process_rightdwon = 0;
    }
}

void esao_on_smb(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOFuncPointer *func_esao_filter, int pix_y, int pix_x, int lcu_pix_width, int lcu_pix_height, ESAOBlkParam *esao_blk_param, int sample_bit_depth,int lcu_pos)
{
    int comp_idx;
    int lcu_pix_height_t, lcu_pix_width_t, pix_x_t, pix_y_t;
    int  is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        if ( info->pic_header.pic_esao_on[comp_idx])
        {
            lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1) ) : (lcu_pix_width );
            lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1) ) : (lcu_pix_height );
            pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
            pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
            check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
                &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
            esao_on_block(info, map, pic_rec, pic_esao, esao_blk_param, func_esao_filter, comp_idx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,sample_bit_depth, lcu_pos, esao_blk_param[comp_idx].lcu_flag[lcu_pos],
                is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail,is_above_right_avail, is_below_left_avail, is_below_right_avail);
        }
    }
}

void esao_on_block(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOBlkParam *esao_blk_param, ESAOFuncPointer *func_esao_filter, int comp_idx, int pix_y, int pix_x, int lcu_pix_height,int lcu_pix_width, int sample_bit_depth, int lcu_pos,int lcu_open_flag,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    pel *src, *dst;
    int src_stride, dst_stride;
    src = dst = NULL;
    int label_index = info->pic_header.esao_adaptive_param[comp_idx];
    assert(label_index >= 0 && label_index < ESAO_LABEL_NUM_MAX);
    int shift_index = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        src_stride = pic_esao->stride_luma;
        dst_stride = pic_rec->stride_luma;
        src = pic_esao->y;
        dst = pic_rec->y;
        break;
    case U_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->u;
        dst = pic_rec->u;
        break;
    case V_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->v;
        dst = pic_rec->v;
        break;
    default:
        src_stride = 0;
        dst_stride = 0;
        src = NULL;
        dst = NULL;
        assert(0);
    }
    if (comp_idx == Y_C)
    {
        if (info->pic_header.esao_lcu_enable[comp_idx])
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                    sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset, lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                    lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon,info->pic_header.esao_luma_type);
            }
        }
        else 
        {
            func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset, lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon,info->pic_header.esao_luma_type);
        }
    }
    else
    {
        if (info->pic_header.esao_lcu_enable[comp_idx])
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[label_index][1], tab_esao_chroma_class_bit[label_index][2],
                    sample_bit_depth, esao_blk_param[comp_idx].offset);
            }
        }
        else
        {
            func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[label_index][1], tab_esao_chroma_class_bit[label_index][2],
                sample_bit_depth, esao_blk_param[comp_idx].offset);
        }
    }
}

void esao_on_frame(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOBlkParam *rec_esao_params, ESAOFuncPointer *func_esao_filter)
{
    if ((info->pic_header.pic_esao_on[Y_C] == 0) && (info->pic_header.pic_esao_on[U_C] == 0) && (info->pic_header.pic_esao_on[V_C] == 0))
    {
        return;
    }
    decide_esao_filter_func_pointer(func_esao_filter);
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_MaxSizeInBit = info->log2_max_cuwh;
    int bit_depth = info->bit_depth_internal;
    int pix_y, pix_x;
    int lcu_pix_height, lcu_pix_width;
    for (pix_y = 0; pix_y < pic_pix_height; pix_y += lcu_pix_height)
    {
        lcu_pix_height = min(1 << (input_MaxSizeInBit), (pic_pix_height - pix_y));
        for (pix_x = 0; pix_x < pic_pix_width; pix_x += lcu_pix_width)
        {
            int x_in_lcu = pix_x >> info->log2_max_cuwh;
            int y_in_lcu = pix_y >> info->log2_max_cuwh;
            int lcu_pos = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            lcu_pix_width = min(1 << (input_MaxSizeInBit), (pic_pix_width - pix_x));
            esao_on_smb(info, map, pic_rec, pic_esao, func_esao_filter, pix_y, pix_x, lcu_pix_width, lcu_pix_height, rec_esao_params, bit_depth,lcu_pos);
        }
    }
}
#endif