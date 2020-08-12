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

#include "dec_def.h"
#include "com_tbl.h"
#if USE_SP
#include "com_usp.h"
#endif


void read_byte(COM_BSR * bs, DEC_SBAC * sbac)
{
    if (sbac->bits_Needed >= 0)
    {
        u8 new_byte = (u8)com_bsr_read(bs, 8);
        sbac->prev_bytes = ((sbac->prev_bytes << 8) | new_byte);
        sbac->value += new_byte << sbac->bits_Needed;
        sbac->bits_Needed -= 8;
    }
}

static __inline int ace_get_shift(int v)
{
#ifdef _WIN32
    unsigned long index;
    _BitScanReverse(&index, v);
    return 8 - index;
#else
    return __builtin_clz(v) - 23;
#endif
}

static __inline int ace_get_log2(int v)
{
#ifdef _WIN32
    unsigned long index;
    _BitScanReverse(&index, v);
    return index;
#else
    return 31 - __builtin_clz(v);
#endif
}

u32 dec_sbac_decode_bin(COM_BSR * bs, DEC_SBAC * sbac, SBAC_CTX_MODEL * model)
{
#if TRACE_BIN
    SBAC_CTX_MODEL prev_model = *model;
#endif
#if CABAC_MULTI_PROB
    u8 cycno = (*model) >> CYCNO_SHIFT_BITS;
    if (cycno < 0)
        cycno = 0;
    int is_LPS = 0;
    u16 p0 = ((*model) >> PROB_BITS)& MCABAC_PROB_MASK;
    u16 p1 = ((*model) >> 1) & MCABAC_PROB_MASK;
    u16 prob_lps = (u16)(p0 + p1 + 1) >> 1;
    prob_lps = prob_lps < 6 ? 6 : prob_lps;
    u8 cwr = 0;
    if (g_compatible_back)
    {
        mCabac_ws = 6;
        cwr = (cycno <= 1) ? 3 : (cycno == 2) ? 4 : (mCabac_ws - 1);
    }
    else
    {
        cwr = (cycno < counter_thr1) ? (mCabac_ws - 2) : (mCabac_ws - 1);
    }
    cwr = COM_CLIP(cwr, MIN_WINSIZE, MAX_WINSIZE);
    u8 mcabac_flag = (cycno == counter_thr2) ? 1 : 0;
    u16 LG_S = cwr2LGS[cwr];
#endif

#if !CABAC_MULTI_PROB
    u16 prob_lps = ((*model) & PROB_MASK) >> 1;
#endif
    u16 cmps = (*model) & 1;
    u32 bin  = cmps;
    u32 rLPS = prob_lps >> LG_PMPS_SHIFTNO;
    u32 rMPS = sbac->range - rLPS;
    int s_flag = rMPS < QUAR_HALF_PROB;
    rMPS |= 0x100;
    u32 scaled_rMPS = rMPS << (8 - s_flag);
    assert(sbac->range >= rLPS);  //! this maybe triggered, so it can be removed
    if(sbac->value < scaled_rMPS) //! MPS
    {
#if CABAC_MULTI_PROB
        if (cycno == 0)
        {
            cycno = 1;
        }
#endif
        sbac->range = rMPS;
        if (s_flag)
        {
            (sbac)->value = sbac->value << 1;
            sbac->bits_Needed++;
            read_byte(bs, sbac);
        }
#if !CABAC_MULTI_PROB
#if CHECK_ALL_CTX
        *model = tab_cycno_lgpmps_mps[*model & 0xFFFF] | (((*model >> 15) | 1) << 16);
#else
        *model = tab_cycno_lgpmps_mps[*model];
#endif
#endif
    }
    else //! LPS
    {
#if CABAC_MULTI_PROB
        if (g_compatible_back)
        {
            cycno = (cycno <= 2) ? (cycno + 1) : 3;
        }
        else
        {
            if (mcabac_flag)
            {
                cycno = counter_thr2;
            }
            else
            {
                cycno = cycno + 1;
            }
        }
        is_LPS = 1;
#endif
        bin = 1 - bin;
        rLPS = (sbac->range << s_flag) - rMPS;
        int shift = ace_get_shift(rLPS);
        sbac->range = rLPS << shift;
        sbac->value = (sbac->value - scaled_rMPS) << (s_flag + shift);
        sbac->bits_Needed += (s_flag + shift);
        while (sbac->bits_Needed >= 0)
        {
            read_byte(bs, sbac);
        }
#if !CABAC_MULTI_PROB
#if CHECK_ALL_CTX
        *model = tab_cycno_lgpmps_mps[(*model & 0xFFFF) | (1 << 13)] | (((*model >> 15) | 1) << 16);
#else
        *model = tab_cycno_lgpmps_mps[(*model) | (1 << 13)];
#endif
#endif
    }

#if CABAC_MULTI_PROB
    //update probability estimation
    if (is_LPS)
    {
        if (g_compatible_back)
        {
            p0 = p0 + LG_S;
            p1 = p0;
        }
        else
        {
            if (mcabac_flag)
            {
                p0 = p0 + LG_S;
                p1 = p1 + cwr2LGS[mCabac_ws + 1];
            }
            else
            {
                p0 = p0 + LG_S;
                p1 = p0;
            }
        }

        if ((p0 >= (256 << LG_PMPS_SHIFTNO)) || (p1 >= (256 << LG_PMPS_SHIFTNO)))
        {
            if (p0 >= (256 << LG_PMPS_SHIFTNO))
            {
                p0 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p0;
            }
            if (p1 >= (256 << LG_PMPS_SHIFTNO))
            {
                p1 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p1;
            }
            cmps = !cmps;
        }
    }
    else
    {
        if (g_compatible_back)
        {
            p0 = p0 - (u16)(p0 >> cwr) - (u16)(p0 >> (cwr + 2));
            p1 = p0;
        }
        else
        {
            if (mcabac_flag)
            {
                p0 = p0 - (u16)(p0 >> cwr) - (u16)(p0 >> (cwr + 2));
                p1 = p1 - (u16)(p1 >> (mCabac_ws + 1)) - (u16)(p1 >> (mCabac_ws + 3));
            }
            else
            {
                p0 = p0 - (u16)(p0 >> cwr) - (u16)(p0 >> (cwr + 2));
                p1 = p0;
            }
        }
    }
    *model = (p1 << 1) + cmps + (cycno << CYCNO_SHIFT_BITS) + (p0 << PROB_BITS);
#endif

#if TRACE_BIN
    COM_TRACE_COUNTER;
    COM_TRACE_STR("model ");
    COM_TRACE_INT(prev_model);
    COM_TRACE_STR("-->");
    COM_TRACE_INT(*model);
    COM_TRACE_STR("MPS Range ");
    COM_TRACE_INT(sbac->range);
    COM_TRACE_STR("LPS Range ");
    COM_TRACE_INT(rLPS);
    COM_TRACE_STR("\n");
#endif
    return bin;
}

u32 dec_sbac_decode_binW(COM_BSR * bs, DEC_SBAC * sbac, SBAC_CTX_MODEL * model1, SBAC_CTX_MODEL * model2)
{
#if CABAC_MULTI_PROB
    if (g_compatible_back)
    {
        mCabac_ws = 6;
    }
    u8 cycno1 = (*model1) >> CYCNO_SHIFT_BITS;
    u8 cycno2 = (*model2) >> CYCNO_SHIFT_BITS;
    if (cycno1 < 0)
        cycno1 = 0;
    if (cycno2 < 0)
        cycno2 = 0;
    int is_LPS = 0;
    u16 p1_0 = ((*model1) >> PROB_BITS)& MCABAC_PROB_MASK;
    u16 p1_1 = ((*model1) >> 1) & MCABAC_PROB_MASK;
    u16 prob_lps1 = (u16)(p1_0 + p1_1 + 1) >> 1;
    prob_lps1 = prob_lps1 < 6 ? 6 : prob_lps1;
    u16 p2_0 = ((*model2) >> PROB_BITS)& MCABAC_PROB_MASK;
    u16 p2_1 = ((*model2) >> 1) & MCABAC_PROB_MASK;
    u16 prob_lps2 = (u16)(p2_0 + p2_1 + 1) >> 1;
    prob_lps2 = prob_lps2 < 6 ? 6 : prob_lps2;
    u8 cwr1 = 0;
    if (g_compatible_back)
    {
        mCabac_ws = 6;
        cwr1 = (cycno1 <= 1) ? 3 : (cycno1 == 2) ? 4 : (mCabac_ws - 1);
    }
    else
    {
        cwr1 = (cycno1 < counter_thr1) ? (mCabac_ws - 2) : (mCabac_ws - 1);
    }
    cwr1 = COM_CLIP(cwr1, MIN_WINSIZE, MAX_WINSIZE);
    u8 mcabac_flag1 = (cycno1 == counter_thr2) ? 1 : 0;
    u16 LG_S1 = cwr2LGS[cwr1];
    u8 cwr2 = 0;
    if (g_compatible_back)
    {
        mCabac_ws = 6;
        cwr2 = (cycno2 <= 1) ? 3 : (cycno2 == 2) ? 4 : (mCabac_ws - 1);
    }
    else
    {
        cwr2 = (cycno2 < counter_thr1) ? (mCabac_ws - 2) : (mCabac_ws - 1);
    }
    cwr2 = COM_CLIP(cwr2, MIN_WINSIZE, MAX_WINSIZE);
    u8 mcabac_flag2 = (cycno2 == counter_thr2) ? 1 : 0;
    u16 LG_S2 = cwr2LGS[cwr2];
#endif
    u16 prob_lps;
#if !CABAC_MULTI_PROB
    u16 prob_lps1 = ((*model1) & PROB_MASK) >> 1;
    u16 prob_lps2 = ((*model2) & PROB_MASK) >> 1;
#endif
    u16 cmps;
    u16 cmps1 = (*model1) & 1;
    u16 cmps2 = (*model2) & 1;
    u32 rLPS;
    u32 rMPS;
    int s_flag;
    u32 bin;
    u32 scaled_rMPS;

    if (cmps1 == cmps2)
    {
        cmps = cmps1;
        prob_lps = (prob_lps1 + prob_lps2) >> 1;
    }
    else
    {
        if (prob_lps1 < prob_lps2)
        {
            cmps = cmps1;
            prob_lps = (256 << LG_PMPS_SHIFTNO) - 1 - ((prob_lps2 - prob_lps1) >> 1);
        }
        else
        {
            cmps = cmps2;
            prob_lps = (256 << LG_PMPS_SHIFTNO) - 1 - ((prob_lps1 - prob_lps2) >> 1);
        }
    }

    rLPS = prob_lps >> LG_PMPS_SHIFTNO;
    rMPS = sbac->range - rLPS;
    s_flag = rMPS < QUAR_HALF_PROB;
    rMPS |= 0x100;
    bin = cmps;
    scaled_rMPS = rMPS << (8 - s_flag);
    assert(sbac->range >= rLPS);  //! this maybe triggered, so it can be removed

    if (sbac->value < scaled_rMPS) //! MPS
    {
        sbac->range = rMPS;
        if (s_flag)
        {
            (sbac)->value = sbac->value << 1;
            sbac->bits_Needed++;
            read_byte(bs, sbac);
        }
    }
    else //! LPS
    {
        bin = 1 - bin;
        rLPS = (sbac->range << s_flag) - rMPS;
        int shift = ace_get_shift(rLPS);
        sbac->range = rLPS << shift;
        sbac->value = (sbac->value - scaled_rMPS) << (s_flag + shift);
        sbac->bits_Needed += (s_flag + shift);
        while (sbac->bits_Needed >= 0)
        {
            read_byte(bs, sbac);
        }
    }
#if !CABAC_MULTI_PROB
#if CHECK_ALL_CTX
    if (bin != cmps1)
    {
        *model1 = tab_cycno_lgpmps_mps[(*model1 & 0xFFFF) | (1 << 13)] | (((*model1 >> 15) | 1) << 16);
    }
    else
    {
        *model1 = tab_cycno_lgpmps_mps[*model1 & 0xFFFF] | (((*model1 >> 15) | 1) << 16);
    }
    if (bin != cmps2)
    {
        *model2 = tab_cycno_lgpmps_mps[(*model2 & 0xFFFF) | (1 << 13)] | (((*model2 >> 15) | 1) << 16);
    }
    else
    {
        *model2 = tab_cycno_lgpmps_mps[*model2 & 0xFFFF] | (((*model2 >> 15) | 1) << 16);
    }
#else
    if (bin != cmps1)
    {
        *model1 = tab_cycno_lgpmps_mps[(*model1) | (1 << 13)];
    }
    else
    {
        *model1 = tab_cycno_lgpmps_mps[*model1];
    }
    if (bin != cmps2)
    {
        *model2 = tab_cycno_lgpmps_mps[(*model2) | (1 << 13)];
    }
    else
    {
        *model2 = tab_cycno_lgpmps_mps[*model2];
    }
#endif
#else
    if (bin != cmps1) // LPS 
    {
        if (g_compatible_back)
        {
            cycno1 = (cycno1 <= 2) ? (cycno1 + 1) : 3;
        }
        else
        {
            if (mcabac_flag1)
            {
                cycno1 = counter_thr2;
            }
            else
            {
                cycno1 = cycno1 + 1;
            }
        }

        if (g_compatible_back)
        {
            p1_0 = p1_0 + LG_S1;
            p1_1 = p1_0;
        }
        else
        {
            if (mcabac_flag1)
            {
                p1_0 = p1_0 + LG_S1;
                p1_1 = p1_1 + cwr2LGS[mCabac_ws + 1];
            }
            else
            {
                p1_0 = p1_0 + LG_S1;
                p1_1 = p1_0;
            }
        }

        if ((p1_0 >= (256 << LG_PMPS_SHIFTNO)) || (p1_1 >= (256 << LG_PMPS_SHIFTNO)))
        {
            if (p1_0 >= (256 << LG_PMPS_SHIFTNO))
            {
                p1_0 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p1_0;
            }
            if (p1_1 >= (256 << LG_PMPS_SHIFTNO))
            {
                p1_1 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p1_1;
            }

            cmps1 = !cmps1;
        }
    }
    else // MPS
    {
        if (cycno1 == 0)
        {
            cycno1 = 1;
        }
        if (g_compatible_back)
        {
            p1_0 = p1_0 - (u16)(p1_0 >> cwr1) - (u16)(p1_0 >> (cwr1 + 2));
            p1_1 = p1_0;
        }
        else
        {
            if (mcabac_flag1)
            {
                p1_0 = p1_0 - (u16)(p1_0 >> cwr1) - (u16)(p1_0 >> (cwr1 + 2));
                p1_1 = p1_1 - (u16)(p1_1 >> (mCabac_ws + 1)) - (u16)(p1_1 >> (mCabac_ws + 3));
            }
            else
            {
                p1_0 = p1_0 - (u16)(p1_0 >> cwr1) - (u16)(p1_0 >> (cwr1 + 2));
                p1_1 = p1_0;
            }
        }
    }

    *model1 = (p1_1 << 1) + cmps1 + (cycno1 << CYCNO_SHIFT_BITS) + (p1_0 << PROB_BITS);

    if (bin != cmps2) // LPS
    {
        if (g_compatible_back)
        {
            cycno2 = (cycno2 <= 2) ? (cycno2 + 1) : 3;
        }
        else
        {
            if (mcabac_flag2)
            {
                cycno2 = counter_thr2;
            }
            else
            {
                cycno2 = cycno2 + 1;
            }
        }

        if (g_compatible_back)
        {
            p2_0 = p2_0 + LG_S2;
            p2_1 = p2_0;
        }
        else
        {
            if (mcabac_flag2)
            {
                p2_0 = p2_0 + LG_S2;
                p2_1 = p2_1 + cwr2LGS[mCabac_ws + 1];
            }
            else
            {
                p2_0 = p2_0 + LG_S2;
                p2_1 = p2_0;
            }
        }

        if ((p2_0 >= (256 << LG_PMPS_SHIFTNO)) || (p2_1 >= (256 << LG_PMPS_SHIFTNO)))
        {
            if (p2_0 >= (256 << LG_PMPS_SHIFTNO))
            {
                p2_0 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p2_0;
            }
            if (p2_1 >= (256 << LG_PMPS_SHIFTNO))
            {
                p2_1 = (u16)(512 << LG_PMPS_SHIFTNO) - 1 - p2_1;
            }

            cmps2 = !cmps2;
        }
    }
    else // MPS
    {
        if (cycno2 == 0)
        {
            cycno2 = 1;
        }
        if (g_compatible_back)
        {
            p2_0 = p2_0 - (u16)(p2_0 >> cwr2) - (u16)(p2_0 >> (cwr2 + 2));
            p2_1 = p2_0;
        }
        else
        {
            if (mcabac_flag2)
            {
                p2_0 = p2_0 - (u16)(p2_0 >> cwr2) - (u16)(p2_0 >> (cwr2 + 2));
                p2_1 = p2_1 - (u16)(p2_1 >> (mCabac_ws + 1)) - (u16)(p2_1 >> (mCabac_ws + 3));
            }
            else
            {
                p2_0 = p2_0 - (u16)(p2_0 >> cwr2) - (u16)(p2_0 >> (cwr2 + 2));
                p2_1 = p2_0;
            }
        }

    }
    *model2 = (p2_1 << 1) + cmps2 + (cycno2 << CYCNO_SHIFT_BITS) + (p2_0 << PROB_BITS);
#endif
    return bin;
}


static u32 sbac_decode_bin_ep(COM_BSR * bs, DEC_SBAC * sbac)
{
    u32 bin;
    u32 scaled_range;
#if TRACE_BIN
    COM_TRACE_COUNTER;
    COM_TRACE_STR("range ");
    COM_TRACE_INT(sbac->range);
    COM_TRACE_STR("\n");
#endif
    scaled_range = sbac->range << DEC_RANGE_SHIFT;
    if(sbac->value < scaled_range)
    {
        bin = 0;
    }
    else
    {
        sbac->value -= scaled_range;
        bin = 1;
    }
    (sbac)->value = sbac->value << 1;
    sbac->bits_Needed++;
    read_byte(bs, sbac);
    return bin;
}

#if SRCC
static u32 sbac_decode_bins_ep(COM_BSR *bs, DEC_SBAC *sbac, int num_bin)
{
    int bin = 0;
    u32 value = 0;

    for (bin = num_bin - 1; bin >= 0; bin--)
    {
        if (sbac_decode_bin_ep(bs, sbac))
        {
            value += (1 << bin);
        }
    }
    return value;
}
#endif

int dec_sbac_decode_bin_trm(COM_BSR * bs, DEC_SBAC * sbac)
{
    u8 s_flag = (sbac->range - 1 < QUAR_HALF_PROB);
    u32 rMPS = (sbac->range - 1) | 0x100;
    u32 scaled_rMPS = rMPS << (8 - s_flag);
    if (sbac->value < scaled_rMPS) //! MPS
    {
        sbac->range = rMPS;
        if (s_flag)
        {
            (sbac)->value = sbac->value << 1;
            sbac->bits_Needed++;
            read_byte(bs, sbac);
        }
        return 0;
    }
    else
    {
        u32 rLPS = s_flag ? ((sbac->range << 1) - rMPS) : 1;
        int shift = ace_get_shift(rLPS);
        sbac->range = rLPS << shift;
        sbac->value = (sbac->value - scaled_rMPS) << (s_flag + shift);
        sbac->bits_Needed += (s_flag + shift);

        while (sbac->bits_Needed > 0) // the last byte is written, so read the last byte before reading the termination padding byte
        {
            read_byte(bs, sbac);
        }
        while (!COM_BSR_IS_BYTE_ALIGN(bs))
        {
            assert(com_bsr_read1(bs) == 0);
        }
        return 1; /* end of slice */
    }
}

static u32 sbac_read_bins_ep_msb(COM_BSR * bs, DEC_SBAC * sbac, int num_bin)
{
    int bin = 0;
    u32 val = 0;

    for (bin = num_bin - 1; bin >= 0; bin--)
    {
        val = (val << 1) | sbac_decode_bin_ep(bs, sbac);
    }
    return val;
}

static u32 sbac_read_unary_sym_ep(COM_BSR * bs, DEC_SBAC * sbac)
{
    u32 val = 0;
    u32 bin;

    do
    {
        bin = !sbac_decode_bin_ep(bs, sbac);
        val += bin;
    }
    while (bin);

    return val;
}

static u32 sbac_read_unary_sym(COM_BSR * bs, DEC_SBAC * sbac, SBAC_CTX_MODEL * model, u32 num_ctx)
{
    u32 val = 0;
    u32 bin;

    do
    {
        bin = !dec_sbac_decode_bin(bs, sbac, model + min(val, num_ctx - 1));
        val += bin;
    }
    while (bin);

    return val;
}

static u32 sbac_read_truncate_unary_sym(COM_BSR * bs, DEC_SBAC * sbac, SBAC_CTX_MODEL * model, u32 num_ctx, u32 max_num)
{
    u32 val = 0;
    u32 bin;

    do
    {
        bin = !dec_sbac_decode_bin(bs, sbac, model + min(val, num_ctx - 1));
        val += bin;
    }
    while (val < max_num - 1 && bin);

    return val;
}

static u32 sbac_read_truncate_unary_sym_ep(COM_BSR * bs, DEC_SBAC * sbac, u32 max_num)
{
    u32 val = 0;
    u32 bin;

    do
    {
        bin = !sbac_decode_bin_ep(bs, sbac);
        val += bin;
    }
    while (val < max_num - 1 && bin);

    return val;
}

#if ESAO
static u32 sbac_read_truncate_unary_sym_ep_esao(COM_BSR * bs, DEC_SBAC * sbac, u32 max_num)
{
    u32 val = 0;
    u32 bin;

    while (val < max_num - 1)
    {
        bin = !sbac_decode_bin_ep(bs, sbac);
        val += bin;
        if (!bin)
        {
            break;
        }
    }

    return val;
}
#endif

#if AWP
static u32 sbac_read_truncate_binary_sym_ep(COM_BSR * bs, DEC_SBAC * sbac, s32 max_num)
{
    s32 sym = 0;
    int thresh;
    if (max_num > 256)
    {
        s32 threshVal = 1 << 8;
        thresh = 8;
        while (threshVal <= max_num)
        {
            thresh++;
            threshVal <<= 1;
        }
        thresh--;
    }
    else
    {
        thresh = com_tbl_logmap[max_num];
    }

    s32 val = 1 << thresh;
    s32 b = max_num - val;
    sym = sbac_decode_bins_ep(bs, sbac, thresh);
    if (sym >= val - b)
    {
        s32 altSym;
        altSym = sbac_decode_bin_ep(bs, sbac);
        sym <<= 1;
        sym += altSym;
        sym -= (val - b);
    }
    return sym;
}
#endif

#if SBT
static int dec_eco_sbt_info( COM_BSR * bs, DEC_SBAC * sbac, int log2_cuw, int log2_cuh, u8* sbt_info, u8 sbt_avail )
{
    u8 mode_vert = (sbt_avail >> 0) & 0x1;
    u8 mode_hori = (sbt_avail >> 1) & 0x1;
    u8 mode_vert_quad = (sbt_avail >> 2) & 0x1;
    u8 mode_hori_quad = (sbt_avail >> 3) & 0x1;
    u8 num_sbt_mode_avail = mode_vert + mode_hori + mode_vert_quad + mode_hori_quad;

    if( num_sbt_mode_avail == 0 )
    {
        *sbt_info = 0;
        return COM_OK;
    }
    else
    {
        u8 sbt_flag = 0;
        u8 sbt_dir = 0;
        u8 sbt_quad = 0;
        u8 sbt_pos = 0;
        int size = 1 << (log2_cuw + log2_cuh);
        u8 ctx_sbt_flag = size >= 256 ? 0 : 1;
        u8 ctx_sbt_quad = 2;
        u8 ctx_sbt_dir = ((log2_cuw == log2_cuh) ? 0 : (log2_cuw < log2_cuh ? 1 : 2)) + 3;
        u8 ctx_sbt_pos = 6;

        COM_SBAC_CTX * sbac_ctx;
        sbac_ctx = &sbac->ctx;

        sbt_flag = dec_sbac_decode_bin( bs, sbac, sbac_ctx->sbt_info + ctx_sbt_flag );
        COM_TRACE_STR( "sbt_flag " );
        COM_TRACE_INT( sbt_flag );
        COM_TRACE_STR( "\n" );

        if( sbt_flag )
        {
            if( (mode_vert_quad || mode_hori_quad) && (mode_vert || mode_hori) )
            {
                sbt_quad = dec_sbac_decode_bin( bs, sbac, sbac_ctx->sbt_info + ctx_sbt_quad );
                COM_TRACE_STR( "sbt_quad " );
                COM_TRACE_INT( sbt_quad );
                COM_TRACE_STR( "\n" );
            }
            else
            {
                sbt_quad = 0;
            }

            if( (sbt_quad && mode_vert_quad && mode_hori_quad) || (!sbt_quad && mode_vert && mode_hori) )
            {
                sbt_dir = dec_sbac_decode_bin( bs, sbac, sbac_ctx->sbt_info + ctx_sbt_dir );
                COM_TRACE_STR( "sbt_dir " );
                COM_TRACE_INT( sbt_dir );
                COM_TRACE_STR( "\n" );
            }
            else
            {
                sbt_dir = (sbt_quad && mode_hori_quad) || (!sbt_quad && mode_hori);
            }

            sbt_pos = dec_sbac_decode_bin( bs, sbac, sbac_ctx->sbt_info + ctx_sbt_pos );
            COM_TRACE_STR( "sbt_pos " );
            COM_TRACE_INT( sbt_pos );
            COM_TRACE_STR( "\n" );
        }
        *sbt_info = get_sbt_info( (sbt_quad ? 2 : 0) + (sbt_dir ? 1 : 0) + sbt_flag, sbt_pos );

        return COM_OK;
    }
}
#endif

#if CHROMA_NOT_SPLIT
static int dec_eco_cbf_uv(COM_BSR * bs, DEC_SBAC * sbac, u8(*cbf)[N_C]
#if PMC
                        , s8 ipm_c
#endif
)
{
    COM_SBAC_CTX * sbac_ctx = &sbac->ctx;

    cbf[TBUV0][Y_C] = 0;

#if PMC
    int bMcpm = com_is_mcpm(ipm_c);
    if (bMcpm)
    {
        cbf[TBUV0][U_C] = IS_RIGHT_CBF_U(1) ? 1 : 0;
    }
    else
    {
#endif
        cbf[TBUV0][U_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
#if PMC
    }
#endif
    cbf[TBUV0][V_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);

    COM_TRACE_STR("cbf U ");
    COM_TRACE_INT(cbf[TBUV0][U_C]);
    COM_TRACE_STR("cbf V ");
    COM_TRACE_INT(cbf[TBUV0][V_C]);
    COM_TRACE_STR("\n");
    return COM_OK;
}
#endif

#if IPCM
static int dec_eco_cbf(COM_BSR * bs, DEC_SBAC * sbac, int tb_avaliable, const int pb_part_size, int* tb_part_size, u8 pred_mode, s8 ipm[MAX_NUM_PB][2], u8 (*cbf)[N_C], u8 tree_status, DEC_CTX * ctx)
#else
static int dec_eco_cbf(COM_BSR * bs, DEC_SBAC * sbac, int tb_avaliable, const int pb_part_size, int* tb_part_size, u8 pred_mode, u8 (*cbf)[N_C], u8 tree_status, DEC_CTX * ctx)
#endif
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    COM_SBAC_CTX * sbac_ctx = &sbac->ctx;
    int i, part_num;

    /* decode allcbf */
    if(pred_mode != MODE_INTRA)
    {
        int tb_split;
        if (pred_mode != MODE_DIR)
        {
#if CHROMA_NOT_SPLIT //ctp_zero_flag
            if (tree_status == TREE_LC)
            {
#endif
#if SEP_CONTEXT
                int ctp_zero_flag = 1;
                if (mod_info_curr->cu_width_log2 > 6 || mod_info_curr->cu_height_log2 > 6)
                {
                    ctp_zero_flag = dec_sbac_decode_bin(bs, sbac, sbac_ctx->ctp_zero_flag + 1);
                    assert(ctp_zero_flag == 1);
                }
                else
                    ctp_zero_flag = dec_sbac_decode_bin(bs, sbac, sbac_ctx->ctp_zero_flag);
#else
                int ctp_zero_flag = dec_sbac_decode_bin(bs, sbac, sbac_ctx->ctp_zero_flag);
#endif
                COM_TRACE_COUNTER;
                COM_TRACE_STR("ctp zero flag ");
                COM_TRACE_INT(ctp_zero_flag);
                COM_TRACE_STR("\n");

                if (ctp_zero_flag)
                {
                    for (i = 0; i < MAX_NUM_TB; i++)
                    {
                        cbf[i][Y_C] = cbf[i][U_C] = cbf[i][V_C] = 0;
                    }
                    *tb_part_size = SIZE_2Nx2N;
                    return COM_OK;
                }
#if CHROMA_NOT_SPLIT
            }
#endif
        }

        if (tb_avaliable)
        {
            tb_split = dec_sbac_decode_bin(bs, sbac, sbac_ctx->tb_split);
            *tb_part_size = tb_split? get_tb_part_size_by_pb(pb_part_size, pred_mode) : SIZE_2Nx2N;
        }
        else
        {
            tb_split = 0;
            *tb_part_size = SIZE_2Nx2N;
        }
        COM_TRACE_COUNTER;
        COM_TRACE_STR("tb_split ");
        COM_TRACE_INT(tb_split);
        COM_TRACE_STR("\n");

        if (tree_status == TREE_LC)
        {
            cbf[TBUV0][U_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
            cbf[TBUV0][V_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);
            COM_TRACE_STR("cbf U ");
            COM_TRACE_INT(cbf[TBUV0][U_C]);
            COM_TRACE_STR("cbf V ");
            COM_TRACE_INT(cbf[TBUV0][V_C]);
            COM_TRACE_STR("\n");
        }
        else
        {
            assert(tree_status == TREE_L);
            cbf[TBUV0][U_C] = 0;
            cbf[TBUV0][V_C] = 0;
            COM_TRACE_STR("[cbf uv at tree L]\n");
        }

        COM_TRACE_STR("cbf Y ");
#if CHROMA_NOT_SPLIT
        if(cbf[TBUV0][U_C] + cbf[TBUV0][V_C] == 0 && *tb_part_size == SIZE_2Nx2N && tree_status == TREE_LC)
#else
        if(cbf[TBUV0][U_C] + cbf[TBUV0][V_C] == 0 && *tb_part_size == SIZE_2Nx2N)
#endif
        {
            cbf[TB0][Y_C] = 1;
            COM_TRACE_INT(1);
        }
        else
        {
            part_num = get_part_num(*tb_part_size);
            for (i = 0; i < part_num; i++)
            {
                cbf[i][Y_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf);
                COM_TRACE_INT(cbf[i][Y_C]);
            }
        }
        COM_TRACE_STR("\n");

#if SBT
        u8 sbt_avail = com_sbt_allow( mod_info_curr, ctx->info.sqh.sbt_enable_flag, ctx->tree_status );
        assert( mod_info_curr->sbt_info == 0 );
        if( sbt_avail && *tb_part_size == SIZE_2Nx2N && cbf[TB0][Y_C])
        {
            dec_eco_sbt_info( bs, sbac, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, &mod_info_curr->sbt_info, sbt_avail );
        }
#endif
    }
    else
    {
#if IPCM
        if (!(ipm[PB0][0] == IPD_IPCM))
        {
#endif
            *tb_part_size = get_tb_part_size_by_pb(pb_part_size, pred_mode);
            part_num = get_part_num(*tb_part_size);

            COM_TRACE_STR("cbf Y ");
            for (i = 0; i < part_num; i++)
            {
                cbf[i][Y_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf);
                COM_TRACE_INT(cbf[i][Y_C]);
            }
            COM_TRACE_STR("\n");
#if IPCM
        }
#endif
        if (tree_status == TREE_LC)
        {
#if IPCM
            if (!(ipm[PB0][0] == IPD_IPCM && ipm[PB0][1] == IPD_DM_C))
            {
#endif
#if PMC
                s8 ipm_c = ipm[PB0][1];
                int bMcpm = com_is_mcpm(ipm_c);
                if (bMcpm)
                {
                    cbf[TBUV0][U_C] = IS_RIGHT_CBF_U(1) ? 1 : 0;
                }
                else
                {
#endif
                    cbf[TBUV0][U_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
#if PMC
                }
#endif
                cbf[TBUV0][V_C] = (u8)dec_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);
                COM_TRACE_STR("cbf U ");
                COM_TRACE_INT(cbf[TBUV0][U_C]);
                COM_TRACE_STR("cbf V ");
                COM_TRACE_INT(cbf[TBUV0][V_C]);
                COM_TRACE_STR("\n");
#if IPCM
            }
#endif
        }
        else
        {
            assert(tree_status == TREE_L);
            cbf[TBUV0][U_C] = 0;
            cbf[TBUV0][V_C] = 0;
            COM_TRACE_STR("[cbf uv at tree L]\n");
        }
    }

    return COM_OK;
}

static u32 dec_eco_run(COM_BSR *bs, DEC_SBAC *sbac, SBAC_CTX_MODEL *model)
{
    u32 act_sym = sbac_read_truncate_unary_sym(bs, sbac, model, 2, 17);

    if (act_sym == 16)
    {
        int golomb_order = 0;
        golomb_order = sbac_read_unary_sym_ep(bs, sbac);
        act_sym += (1 << golomb_order) - 1;
        act_sym += sbac_read_bins_ep_msb(bs, sbac, golomb_order);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("run:");
    COM_TRACE_INT(act_sym);
    COM_TRACE_STR("\n");
    return act_sym;
}

static u32 dec_eco_level(COM_BSR *bs, DEC_SBAC *sbac, SBAC_CTX_MODEL *model)
{
    u32 act_sym = sbac_read_truncate_unary_sym(bs, sbac, model, 2, 9);

    if (act_sym == 8)
    {
        int golomb_order = 0;
        golomb_order = sbac_read_unary_sym_ep(bs, sbac);
        act_sym += (1 << golomb_order) - 1;
        act_sym += sbac_read_bins_ep_msb(bs, sbac, golomb_order);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("level:");
    COM_TRACE_INT(act_sym);
    COM_TRACE_STR("\n");
    return act_sym;
}

static int dec_eco_run_length_cc(COM_BSR *bs, DEC_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type)
{
    COM_SBAC_CTX  *sbac_ctx;
    int            sign, level, prev_level, run, last_flag;
    int            t0, scan_pos_offset, num_coeff, i, coef_cnt = 0;
    const u16     *scanp;
    sbac_ctx = &sbac->ctx;
    scanp = com_scan_tbl[COEF_SCAN_ZIGZAG][log2_w - 1][log2_h - 1];
    num_coeff = 1 << (log2_w + log2_h);
    scan_pos_offset = 0;
    prev_level = 6;

    do
    {
        t0 = ((COM_MIN(prev_level - 1, 5)) * 2) + (ch_type == Y_C ? 0 : 12);
        /* Run parsing */
        run = dec_eco_run(bs, sbac, sbac_ctx->run + t0);

        for (i = scan_pos_offset; i < scan_pos_offset + run; i++)
        {
            coef[scanp[i]] = 0;
        }
        scan_pos_offset += run;
        /* Level parsing */
        level = dec_eco_level(bs, sbac, sbac_ctx->level + t0);
        level++;

        /* Sign parsing */
        sign = sbac_decode_bin_ep(bs, sbac);
        coef[scanp[scan_pos_offset]] = sign ? -(s16)level : (s16)level;
        coef_cnt++;
        if (scan_pos_offset >= num_coeff - 1)
        {
            break;
        }

        /* Last flag parsing */
        last_flag = dec_sbac_decode_binW(bs, sbac,
                                         &sbac_ctx->last1[COM_MIN(prev_level - 1, 5)        + (ch_type == Y_C ? 0 : NUM_SBAC_CTX_LAST1)],
                                         &sbac_ctx->last2[ace_get_log2(scan_pos_offset + 1) + (ch_type == Y_C ? 0 : NUM_SBAC_CTX_LAST2)]);
        prev_level = level;
        scan_pos_offset++;
    }
    while (!last_flag);

#if ENC_DEC_TRACE
    COM_TRACE_STR("coef");
    COM_TRACE_INT(ch_type);
    for (scan_pos_offset = 0; scan_pos_offset < num_coeff; scan_pos_offset++)
    {
        COM_TRACE_INT(coef[scan_pos_offset]);
    }
    COM_TRACE_STR("\n");
#endif
    return COM_OK;
}
#if SRCC
static void parse_scanregion(COM_BSR *bs, DEC_SBAC *sbac, int* sr_x, int* sr_y, int width, int height, int ch_type)
{
    COM_SBAC_CTX *sbac_ctx = &sbac->ctx;
    SBAC_CTX_MODEL* cm_x = sbac_ctx->cc_scanr_x + (ch_type == Y_C ? 0 : NUM_CTX_SCANR_LUMA);
    SBAC_CTX_MODEL* cm_y = sbac_ctx->cc_scanr_y + (ch_type == Y_C ? 0 : NUM_CTX_SCANR_LUMA);
    int last;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int pos_x, pos_y;
    int i, cnt, tmp;

    com_get_ctx_srxy_para(ch_type, width, height, &blk_offset_x, &blk_offset_y, &shift_x, &shift_y);

    // posX
    for (pos_x = 0; pos_x < g_group_idx[min(width, 32) - 1]; pos_x++)
    {
        last = dec_sbac_decode_bin(bs, sbac, cm_x + blk_offset_x + (pos_x >> shift_x));
        if (!last)
        {
            break;
        }
    }

    // posY
    for (pos_y = 0; pos_y < g_group_idx[min(height, 32) - 1]; pos_y++)
    {
        last = dec_sbac_decode_bin(bs, sbac, cm_y + blk_offset_y + (pos_y >> shift_y));
        if (!last)
        {
            break;
        }
    }

    // EP-coded part
    if (pos_x > 3)
    {
        tmp = 0;
        cnt = (pos_x - 2) >> 1;
        for (i = cnt - 1; i >= 0; i--)
        {
            last = sbac_decode_bin_ep(bs, sbac);
            tmp += last << i;
        }

        pos_x = g_min_in_group[pos_x] + tmp;
    }
    if (pos_y > 3)
    {
        tmp = 0;
        cnt = (pos_y - 2) >> 1;
        for (i = cnt - 1; i >= 0; i--)
        {
            last = sbac_decode_bin_ep(bs, sbac);
            tmp += last << i;
        }
        pos_y = g_min_in_group[pos_y] + tmp;
    }

    *sr_x = pos_x;
    *sr_y = pos_y;
}

static int parse_coef_remain_exgolomb(COM_BSR *bs, DEC_SBAC *sbac)
{
    u32 act_sym = 0;
    int golomb_order = 0;
    golomb_order = sbac_read_unary_sym_ep(bs, sbac);
    act_sym += (1 << golomb_order) - 1;
    act_sym += sbac_read_bins_ep_msb(bs, sbac, golomb_order);

    return act_sym;
}

static int dec_eco_srcc(COM_BSR *bs, DEC_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type
#if IST
    , int *ist_tu_flag
#endif
    , u8 is_intra
#if ISTS
    , u8 is_dst7
#endif
    )
{
    int width = 1 << log2_w;
    int height = 1 << log2_h;
    int offset0;
    SBAC_CTX_MODEL* cm_gt0;
    SBAC_CTX_MODEL* cm_gtx;
    int scan_type = COEF_SCAN_ZIGZAG;
    int log2_block_size = min(log2_w, log2_h);
    int *scan = com_scan_sr;
    int scan_pos_last = -1;
    int sr_x = 0, sr_y = 0;
    int sr_width, sr_height;
    int num_coeff;
    int ipos;
    int last_scan_set;
    int sub_set;
    int be_valid = 0;
    int ctx_gt0 = 0;
    int cg_log2_size = LOG2_CG_SIZE;
    int abs_sum = 0;
    int is_last_x = 0;
    int is_last_y = 0;
    int is_last_nz = 0;
    int pos_last = 0;
    int ctx_gt1 = 0;
    int ctx_gt2 = 0;
    int escape_data_present_ingroup = 0;
    int cnt_nz = 0;
    int blkpos, sx, sy;
    u32 sig;
    int prev_0val[NUM_PREV_0VAL] = { 0 }; //FIFO
    int prev_12val[NUM_PREV_12VAL] = { 0 }; //FIFO
    // Code scan region
    parse_scanregion(bs, sbac, &sr_x, &sr_y, width, height, ch_type);
    sr_width = sr_x + 1;
    sr_height = sr_y + 1;
    num_coeff = sr_width * sr_height;

    com_init_scan_sr(scan, sr_width, sr_height, width, scan_type);

    COM_POS_INFO cur_pos_info;
    com_init_pos_info(&cur_pos_info, sr_x, sr_y);

    //===== code significance flag =====
    offset0 = num_coeff <= 4 ? 0 : NUM_CTX_GT0_LUMA_TU << ( num_coeff <= 16 ? 0 : 1);
    cm_gt0 = (ch_type == Y_C) ? sbac->ctx.cc_gt0 + offset0 : sbac->ctx.cc_gt0 + NUM_CTX_GT0_LUMA;
    cm_gtx = (ch_type == Y_C) ? sbac->ctx.cc_gt1 : sbac->ctx.cc_gt1 + NUM_CTX_GT1_LUMA;

    last_scan_set = (num_coeff - 1) >> cg_log2_size;
    scan_pos_last = num_coeff - 1;

    ipos = scan_pos_last;

    for (sub_set = last_scan_set; sub_set >= 0; sub_set--)
    {
        int num_nz = 0;
        int sub_pos = sub_set << cg_log2_size;
        int abs_coef[1 << LOG2_CG_SIZE];
        int pos[1 << LOG2_CG_SIZE];
        int last_nz_pos_in_cg = -1;
        int first_nz_pos_in_cg = 1 << cg_log2_size;

        abs_sum = 0;
        for (; ipos >= sub_pos; ipos--)
        {
            blkpos = scan[ipos];
            sx = blkpos & (width - 1);
            sy = blkpos >> log2_w;

            // sigmap
            if (ipos == scan_pos_last)
            {
                ctx_gt0 = 0;
            }
            else
            {
                ctx_gt0 = com_get_ctx_gt0_inc(coef, blkpos, width, height, ch_type, sr_x, sr_y, prev_0val
                    , is_intra, &cur_pos_info);
            }

            if (!(sx == 0 && sy == sr_y && is_last_y == 0) && !(sy == 0 && sx == sr_x && is_last_x == 0))
            {
                sig = dec_sbac_decode_bin(bs, sbac, cm_gt0 + ctx_gt0);
            }
            else
            {
                sig = 1;
            }
            coef[blkpos] = (s16)sig;

            if (sig)
            {
                pos[num_nz] = blkpos;
                num_nz++;

                if (last_nz_pos_in_cg == -1)
                {
                    last_nz_pos_in_cg = ipos;
                }
                first_nz_pos_in_cg = ipos;

                if (sx == sr_x)
                {
                    is_last_x = 1;
                }
                if (sy == sr_y)
                {
                    is_last_y = 1;
                }

                if (is_last_nz == 0)
                {
                    pos_last = blkpos;
                    is_last_nz = 1;
                }

            }

            for (int i = NUM_PREV_0VAL - 1; i >0; i--)
            {
                prev_0val[i] = prev_0val[i - 1];
            }
            prev_0val[0] = sig;
        }
        if (num_nz > 0)
        {
            u32 bin;
            int i, idx;
            u32 coef_signs;
            int c2_idx = 0;
            abs_sum = 0;
            escape_data_present_ingroup = 0;
            for (i = 0; i < num_nz; i++)
            {
                abs_coef[i] = 1;
            }
            for (idx = 0; idx < num_nz; idx++)
            {
                if (pos[idx] != pos_last)
                {
                    ctx_gt1 = com_get_ctx_gt1_inc(coef, pos[idx], width, height, ch_type, sr_x, sr_y, prev_12val
                        , is_intra, &cur_pos_info);
                }
                bin = dec_sbac_decode_bin(bs, sbac, cm_gtx + ctx_gt1);
                coef[pos[idx]] += (s16)bin;
                abs_coef[idx] = bin + 1;

                if (bin)
                {
                    if (pos[idx] != pos_last)
                    {
                        ctx_gt2 = com_get_ctx_gt2_inc(coef, pos[idx], width, height, ch_type, sr_x, sr_y, prev_12val
                            , is_intra, &cur_pos_info);
                    }
                    bin = dec_sbac_decode_bin(bs, sbac, cm_gtx + ctx_gt2);
                    coef[pos[idx]] += (s16)bin;
                    abs_coef[idx] = bin + 2;
                    if (bin)
                    {
                        escape_data_present_ingroup = 1;

                    }
                }
                for (int i = NUM_PREV_12VAL - 1; i >0; i--)
                {
                    prev_12val[i] = prev_12val[i - 1];
                }
                prev_12val[0] = coef[pos[idx]];
                
            }
            if (escape_data_present_ingroup)
            {
                for (idx = 0; idx < num_nz; idx++)
                {
                    int base_level = 3;

                    if (abs_coef[idx] == base_level)
                    {
                        int level;
                        level = parse_coef_remain_exgolomb(bs, sbac);
                        coef[pos[idx]] = (s16)(level + base_level);
                        abs_coef[idx] = level + base_level;
                    }
                }
            }

            coef_signs = sbac_decode_bins_ep(bs, sbac, num_nz);
            coef_signs <<= 32 - num_nz;
                
            for (idx = 0; idx < num_nz; idx++)
            {
                blkpos = pos[idx];
                // Signs applied later.
                coef[blkpos] = (s16)abs_coef[idx];
                abs_sum += abs_coef[idx];

                int sign = (int)((coef_signs) >> 31);
                coef[blkpos] = sign > 0 ? -coef[blkpos] : coef[blkpos];
                coef_signs <<= 1;
            } // for non-zero coefs within cg
            cnt_nz += num_nz;
        } // if nnz
    } // for (cg)
#if IST
#if ISTS
    *ist_tu_flag = (cnt_nz % 2 ? 1 : 0);
    if (is_dst7 && (sr_x >= IST_MAX_COEF_SIZE || sr_y >= IST_MAX_COEF_SIZE))
    {
        *ist_tu_flag = 0;
    }
#else
    *ist_tu_flag = (sr_x >= IST_MAX_COEF_SIZE || sr_y >= IST_MAX_COEF_SIZE) ? 0 : (cnt_nz % 2 ? 1 : 0);
#endif
#endif

#if ENC_DEC_TRACE
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sr_x: ");
    COM_TRACE_INT(sr_x);
    COM_TRACE_STR("sr_y: ");
    COM_TRACE_INT(sr_y);
    if (ch_type == Y_C)
    {
        COM_TRACE_STR("\ncoef luma \n");
    }
    else
    {
        COM_TRACE_STR("\ncoef chroma \n");
    }
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            COM_TRACE_INT(coef[y*width + x]);
            COM_TRACE_STR(" ");
        }
        COM_TRACE_STR("\n");
    }
    COM_TRACE_STR("\n");
#endif
    return COM_OK;
}
#endif
static int dec_eco_xcoef(DEC_CTX * ctx, COM_BSR *bs, DEC_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type
#if IST
    , int *ist_tu_flag
#endif
    )
{
    if( (log2_w > MAX_TR_LOG2) || (log2_h > MAX_TR_LOG2) )
    {
        printf( "transform size > 64x64" );
        assert( 0 );
    }

#if SRCC
    if (ctx->info.sqh.srcc_enable_flag)
    {
        dec_eco_srcc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1)
#if IST
            , ist_tu_flag
#endif
            , ctx->core->mod_info_curr.cu_mode == MODE_INTRA ? 1 : 0
#if ISTS
            , ctx->core->mod_info_curr.cu_mode == MODE_INTRA && ctx->info.pic_header.ph_ists_enable_flag == 0
#endif
            );
    }
    else
    {
#endif
        dec_eco_run_length_cc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
#if SRCC
    }
#endif
    return COM_OK;
}

static int decode_refidx(COM_BSR * bs, DEC_SBAC * sbac, int num_refp)
{
    int ref_num = 0;

    if(num_refp > 1)
    {
        ref_num = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.refi, 3, num_refp);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("refi ");
    COM_TRACE_INT(ref_num);
    COM_TRACE_STR(" ctx ");
    COM_TRACE_INT( 0 );
    COM_TRACE_STR("\n");
    return ref_num;
}

#if MODE_CONS
u8 dec_eco_cons_pred_mode_child(COM_BSR * bs, DEC_SBAC * sbac)
{
    u8 flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.cons_mode);
    u8 cons_mode = flag ? ONLY_INTRA : ONLY_INTER;

    COM_TRACE_COUNTER;
    COM_TRACE_STR("cons mode ");
    COM_TRACE_INT(cons_mode);
    COM_TRACE_STR("\n");

    return cons_mode;
}
#endif

static u8 decode_skip_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
#if NUM_SBAC_CTX_SKIP_FLAG > 1
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u32 *map_scu = ctx->map.map_scu;
    u8   avail[2] = { 0, 0 };
    int  scun[2];
    int  ctx_inc = 0;
    u8   bSkipMode;
    scun[0] = mod_info_curr->scup - ctx->info.pic_width_in_scu;
    scun[1] = mod_info_curr->scup - 1;

    if (mod_info_curr->y_scu > 0)
    {
        avail[0] = MCU_GET_CODED_FLAG(map_scu[scun[0]]); // up
    }
    if (mod_info_curr->x_scu > 0)
    {
        avail[1] = MCU_GET_CODED_FLAG(map_scu[scun[1]]); // left
    }
    if (avail[0])
    {
        ctx_inc += MCU_GET_SF(map_scu[scun[0]]);
    }
    if (avail[1])
    {
        ctx_inc += MCU_GET_SF(map_scu[scun[1]]);
    }

#if SEP_CONTEXT
    assert(NUM_SBAC_CTX_SKIP_FLAG == 4);
    if (mod_info_curr->cu_width_log2 + mod_info_curr->cu_height_log2 < 6)
    {
        ctx_inc = 3;
    }
#endif

    bSkipMode = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.skip_flag + ctx_inc);
#else
    u8 bSkipMode = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.skip_flag);
#endif
    COM_TRACE_COUNTER;
    COM_TRACE_STR("skip flag ");
    COM_TRACE_INT(bSkipMode);
#if NUM_SBAC_CTX_SKIP_FLAG > 1
    COM_TRACE_STR(" skip ctx ");
    COM_TRACE_INT(ctx_inc);
#endif
    COM_TRACE_STR("\n");
    return bSkipMode;
}

#if INTERPF
static u8 decode_inter_filter_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    u8 bInterFilterFlag = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.inter_filter_flag[0]);
    if (bInterFilterFlag > 0)
    {
        bInterFilterFlag += dec_sbac_decode_bin(bs, sbac, &sbac->ctx.inter_filter_flag[1]);
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("inter filter flag ");
    COM_TRACE_INT(bInterFilterFlag);
    COM_TRACE_STR("\n");
    return bInterFilterFlag;
}
#endif

#if BGC
static u8 decode_bgc_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    u8 bgc_flag = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.bgc_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("bgc flag ");
    COM_TRACE_INT(bgc_flag);
    COM_TRACE_STR("\n");
    return bgc_flag;
}
static u8 decode_bgc_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    u8 bgc_idx = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.bgc_idx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("bgc idx ");
    COM_TRACE_INT(bgc_idx);
    COM_TRACE_STR("\n");
    return bgc_idx;
}
#endif

#if USE_IBC
static u8 decode_ibc_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
#if NUM_SBAC_CTX_IBC_FLAG > 1
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u32 *map_scu = ctx->map.map_scu;
#if USE_SP
    u8 * map_usp = ctx->map.map_usp;
#endif
    u8   avail[2] = { 0, 0 };
    int  scun[2];
    int  ctx_inc = 0;
    u8   bIBCMode;
    scun[0] = mod_info_curr->scup - ctx->info.pic_width_in_scu;
    scun[1] = mod_info_curr->scup - 1;

    if (mod_info_curr->y_scu > 0)
    {
        avail[0] = MCU_GET_CODED_FLAG(map_scu[scun[0]]); // up
    }
    if (mod_info_curr->x_scu > 0)
    {
        avail[1] = MCU_GET_CODED_FLAG(map_scu[scun[1]]); // left
    }
    if (avail[0])
    {
#if USE_SP
        ctx_inc += (MCU_GET_IBC(map_scu[scun[0]]) && !MSP_GET_SP_INFO(map_usp[scun[0]]));
#else
        ctx_inc += MCU_GET_IBC(map_scu[scun[0]]);
#endif
    }
    if (avail[1])
    {
#if USE_SP
        ctx_inc += (MCU_GET_IBC(map_scu[scun[1]]) && !MSP_GET_SP_INFO(map_usp[scun[1]]));
#else
        ctx_inc += MCU_GET_IBC(map_scu[scun[1]]);
#endif
    }

    bIBCMode = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.ibc_flag + ctx_inc);
#else
    u8 bIBCMode = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.ibc_flag);
#endif
    COM_TRACE_COUNTER;
    COM_TRACE_STR("ibc flag ");
    COM_TRACE_INT(bIBCMode);
#if NUM_SBAC_CTX_IBC_FLAG > 1
    COM_TRACE_STR(" ibc ctx ");
    COM_TRACE_INT(ctx_inc);
#endif
    COM_TRACE_STR("\n");
    return bIBCMode;
}
#endif

#if USE_SP
static int dec_eco_sp_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int sp_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_flag) ? 1 : 0;
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp flag ");
    COM_TRACE_INT(sp_flag);
    COM_TRACE_STR("\n");
    return sp_flag;
}

static int dec_eco_sp_copy_dir_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int sp_copy_direct_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_copy_direct_flag) ? 1 : 0;
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp copy direction ");
    COM_TRACE_INT(sp_copy_direct_flag);
    COM_TRACE_STR("\n");
    return sp_copy_direct_flag;
}

static int dec_eco_above_offset(COM_BSR * bs, DEC_SBAC * sbac)
{
    u8 AboveOffset_flag = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_above_offset);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp above offset mode ");
    COM_TRACE_INT(AboveOffset_flag);
    COM_TRACE_STR("\n");
    return AboveOffset_flag;
}

static int dec_eco_offset_zero(COM_BSR * bs, DEC_SBAC * sbac, u8 isOffsetX)
{
    u8 offset_zero;
    if (isOffsetX)
    {
        offset_zero = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_offset_x_zero);
    }
    else
    {
        offset_zero = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_offset_y_zero);
    }
    COM_TRACE_COUNTER;
    if (isOffsetX)
    {
        COM_TRACE_STR("sp OffsetXZero ");
    }
    else
    {
        COM_TRACE_STR("sp OffsetYZero ");
    }
    COM_TRACE_INT(offset_zero);
    COM_TRACE_STR("\n");
    return offset_zero;
}

static int dec_eco_sp_special_len_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int sp_slc_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_special_len_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp using special length ");
    COM_TRACE_INT(sp_slc_flag);
    COM_TRACE_STR("\n");
    return sp_slc_flag;
}

static int dec_eco_sp_is_matched_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int sp_is_matched_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_is_matched_flag) ? 1 : 0;
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp is matched ");
    COM_TRACE_INT(sp_is_matched_flag);
    COM_TRACE_STR("\n");
    return sp_is_matched_flag;
}

static unsigned int read_bins_ep(COM_BSR * bs, DEC_SBAC * sbac, int numBins)
{
    unsigned int i, ui_symbol;
    unsigned int value = 0;
    for (i = numBins; i > 0; i--)
    {
        ui_symbol = sbac_decode_bin_ep(bs, sbac);
        value += (ui_symbol << (i - 1));
    }
    return value;
}

static unsigned int dec_eco_len_in_suffix(COM_BSR * bs, DEC_SBAC * sbac, int n, int max_val_infix)
{
    //middle
    unsigned int b, c, k, uiInfSufSum;
    if (n > 0)
    {
        b = read_bins_ep(bs, sbac, n - 1);
    }
    else
    {
        b = 0;
    }
    // suffix 
    if (max_val_infix == 0)
    {
        k = 0;
    }
    else if ((int)b < ((1 << n) - max_val_infix - 1))
    {
        k = 0;
    }
    else
    {
        k = 1;
    }
    if (k == 0)
    {
        c = 0;
    }
    else
    {
        c = read_bins_ep(bs, sbac, k);
    }
    uiInfSufSum = (b << k) + c - ((1 << n) - max_val_infix - 1)*k;
    return uiInfSufSum;
}

static int dec_eco_sp_string_length(COM_BSR * bs, DEC_SBAC * sbac, int max_value)
{
    int i, a, n, b, d;
    int count, value, max_val_prefix, max_val_infix;
    unsigned int ui_symbol;
    value = 0;
    i = 0;
    a = 0;
    b = 0;
    if (max_value == 0)
    {
        COM_TRACE_COUNTER;
        COM_TRACE_STR("matched length ");
        COM_TRACE_INT(1);
        COM_TRACE_STR("\n");
        return 0;
    }
    else
    {
        /** prefix **/
        if (max_value < 4)
        {
            max_val_prefix = 0;
        }
        else if (max_value < 20)
        {
            max_val_prefix = 1;
        }
        else if (max_value < 276)
        {
            max_val_prefix = 2;
        }
        else
        {
            max_val_prefix = 3;
        }
        if (max_val_prefix == 0)
        {
            a = 0;
        }
        //table-58
        for (count = 0; count < 3; count++)
        {
            int ui_symbol = 0;
            if (max_val_prefix >= count + 1)
            {
                ui_symbol = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_string_length + count);
            }
            else
            {
                ui_symbol = 1;
            }
            if (ui_symbol == 1)
            {
                break;
            }
            else a++;
        }
        if (a == 0)
        {
            max_val_infix = min(max_value + 1 - 1, 3);
        }
        else if (a == 1)
        {
            max_val_infix = min(max_value + 1 - 5, 15);
        }
        else if (a == 2)
        {
            max_val_infix = min(max_value + 1 - 21, 255);
        }
        else
        {
            max_val_infix = max_value + 1 - 277;
        }
        n = get_msb_p1_idx(max_val_infix);
        // middle and suffix
        ui_symbol = dec_eco_len_in_suffix(bs, sbac, n, max_val_infix);
        //final
        if (a == 0)
        {
            d = 0;
        }
        else if (a == 1)
        {
            d = 4;
        }
        else if (a == 2)
        {
            d = 20;
        }
        else if (a == 3)
        {
            d = 276;
        }
        value = d + ui_symbol;
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("matched length ");
    COM_TRACE_INT(value + 1);
    COM_TRACE_STR("\n");
    return value;
}

static int dec_eco_pixel_y(COM_BSR * bs, DEC_SBAC * sbac, int bit_depth, pel *pixel)
{
    pixel[Y_C] = sbac_read_bins_ep_msb(bs, sbac, bit_depth);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("unmatched pixel Y ");
    COM_TRACE_INT(pixel[Y_C]);
    COM_TRACE_STR("\n");
    return COM_OK;
}

static int dec_eco_pixel_uv(COM_BSR * bs, DEC_SBAC * sbac, int bit_depth, pel *pixel)
{
    pixel[U_C] = sbac_read_bins_ep_msb(bs, sbac, bit_depth);
    pixel[V_C] = sbac_read_bins_ep_msb(bs, sbac, bit_depth);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("unmatched pixel U ");
    COM_TRACE_INT(pixel[U_C]);
    COM_TRACE_STR("\n");
    COM_TRACE_STR("unmatched pixel V ");
    COM_TRACE_INT(pixel[V_C]);
    COM_TRACE_STR("\n");
    return COM_OK;
}

static int dec_eco_sp_or_ibc_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int sp_or_ibc_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_or_ibc_flag) ? 1 : 0;
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp or ibc flag ");
    COM_TRACE_INT(sp_or_ibc_flag);
    COM_TRACE_STR("\n");
    return sp_or_ibc_flag;
}

static int decode_sp_or_ibc_cu_flag(COM_BSR * bs, DEC_SBAC * sbac, int cu_width, int cu_height, COM_MODE * mod_info_curr, DEC_CTX * ctx)
{
    u8 code_ibc, code_sp; //FLAG whether the IBC/SP mode should be coded
    u8 ibc_flag, sp_flag;
    code_ibc = code_sp = FALSE;
    ibc_flag = sp_flag = FALSE;
    u8 sp_or_ibc_flag = 0;
#if USE_IBC
    if (mod_info_curr->cu_width_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2 && mod_info_curr->cu_height_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2)
    {
        code_ibc = 1;
    }
    else
    {
        code_ibc = 0;
    }
#endif
    code_sp = IS_VALID_SP_CU_SIZE(cu_width, cu_height) && ctx->info.pic_header.sp_pic_flag;
    if (code_sp == 1 && code_ibc == 1)
    {
        sp_or_ibc_flag = dec_eco_sp_or_ibc_flag(bs, sbac);
        if (sp_or_ibc_flag == TRUE)
        {
            ibc_flag = decode_ibc_flag(bs, sbac, ctx);
            sp_flag = !ibc_flag;
        }
        else
        {
            ibc_flag = sp_flag = FALSE;
        }
    }
    else if (code_sp == 1 && code_ibc == 0)
    {
        ibc_flag = 0;
        sp_flag = dec_eco_sp_or_ibc_flag(bs, sbac);
        sp_or_ibc_flag = sp_flag || ibc_flag;
    }
    else if (code_sp == 0 && code_ibc == 1)
    {
        sp_flag = 0;
        ibc_flag = decode_ibc_flag(bs, sbac, ctx);
        sp_or_ibc_flag = sp_flag || ibc_flag;
    }
#if USE_IBC
    mod_info_curr->ibc_flag = ibc_flag;
#endif
#if USE_SP
    mod_info_curr->sp_flag = sp_flag;
#endif
    return sp_or_ibc_flag;
}
#endif

#if AWP
static u8 decode_umve_awp_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    u8 bUmveAwpSkipMode = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_awp_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("umve awp flag ");
    COM_TRACE_INT(bUmveAwpSkipMode);
    COM_TRACE_STR("\n");
    return bUmveAwpSkipMode;
}
#else
static u8 decode_umve_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    u8 bUmveSkipMode = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("umve flag ");
    COM_TRACE_INT(bUmveSkipMode);
    COM_TRACE_STR("\n");
    return bUmveSkipMode;
}
#endif

static u8 decode_umve_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int base_idx = 0, step_idx = 0, dir_idx;
    int umve_idx = 0;
    int num_cand_minus1_base = UMVE_BASE_NUM - 1;
    int num_cand_minus1_step = UMVE_REFINE_STEP - 1;
    int idx0;

    if (num_cand_minus1_base > 0)
    {
        idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.umve_base_idx);
        if (!idx0)
        {
            base_idx++;
            for (; base_idx < num_cand_minus1_base; base_idx++)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    break;
                }
            }
        }
    }
    if (num_cand_minus1_step > 0)
    {
        idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.umve_step_idx);
        if (!idx0)
        {
            step_idx++;
            for (; step_idx < num_cand_minus1_step; step_idx++)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    break;
                }
            }
        }
    }

    dir_idx = 0;
    idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[0]);
    if (idx0)
    {
        dir_idx += 2;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    else
    {
        dir_idx += 0;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    umve_idx += (base_idx * UMVE_MAX_REFINE_NUM + step_idx * 4 + dir_idx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("umve idx ");
    COM_TRACE_INT(umve_idx);
    COM_TRACE_STR("\n");
    return umve_idx;
}

#if UMVE_ENH
static u8 decode_umve_idx_sec_set(COM_BSR * bs, DEC_SBAC * sbac)
{
    int base_idx = 0, step_idx = 0, dir_idx;
    int umve_idx = 0;
    int num_cand_minus1_base = UMVE_BASE_NUM - 1;
    int idx0;

    if (num_cand_minus1_base > 0)
    {
        idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.umve_base_idx);
        if (!idx0)
        {
            base_idx++;
            for (; base_idx < num_cand_minus1_base; base_idx++)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    break;
                }
            }
        }
    }

    idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.umve_step_idx);
    if (idx0)
    {
        idx0 = sbac_decode_bin_ep(bs, sbac);
        if (idx0)
        {
            idx0 = sbac_decode_bin_ep(bs, sbac);
            if (idx0)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    step_idx = 7;
                }
                else
                {
                    step_idx = 6;
                }
            }
            else
            {
                step_idx = 5;
            }
        }
        else
        {
            step_idx = 4;
        }
    }
    else
    {
        idx0 = sbac_decode_bin_ep(bs, sbac);
        if (idx0)
        {
            idx0 = sbac_decode_bin_ep(bs, sbac);
            if (idx0)
            {
                step_idx = 2;
            }
            else
            {
                step_idx = 3;
            }
        }
        else
        {
            idx0 = sbac_decode_bin_ep(bs, sbac);
            if (idx0)
            {
                step_idx = 1;
            }
            else
            {
                step_idx = 0;
            }
        }
    }

    dir_idx = 0;
    idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[0]);
    if (idx0)
    {
        dir_idx += 2;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    else
    {
        dir_idx += 0;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    umve_idx += (base_idx * UMVE_MAX_REFINE_NUM_SEC_SET + step_idx * 4 + dir_idx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("umve idx ");
    COM_TRACE_INT(umve_idx);
    COM_TRACE_STR("\n");
    return umve_idx;
}
#endif

#if AFFINE_UMVE
static u8 decode_affine_umve_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u8  affine_umve_flag = 0;
    affine_umve_flag = (u8)dec_sbac_decode_bin((bs), (sbac), &(sbac)->ctx.affine_umve_flag);

    COM_TRACE_COUNTER;
    COM_TRACE_STR("affine umve flag ");
    COM_TRACE_INT(affine_umve_flag);
    COM_TRACE_STR("\n");
    return affine_umve_flag;
}

static u8 decode_affine_umve_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int step_idx = 0, dir_idx;
    u8 affine_umve_idx = -1; 
    int num_cand_minus1_base = AFFINE_UMVE_BASE_NUM - 1;
    int num_cand_minus1_step = AFFINE_UMVE_REFINE_STEP - 1;
    int idx0;

    if (num_cand_minus1_step > 0)
    {
        idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.affine_umve_step_idx);
        if (!idx0)
        {
            step_idx++;
            for (; step_idx < num_cand_minus1_step; step_idx++)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    break;
                }
            }
        }
    }

    dir_idx = 0;
    idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.affine_umve_dir_idx[0]);
    if (idx0)
    {
        dir_idx += 2;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.affine_umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    else
    {
        dir_idx += 0;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.affine_umve_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    affine_umve_idx = (step_idx * AFFINE_UMVE_DIR + dir_idx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("affine umve idx ");
    COM_TRACE_INT(affine_umve_idx);
    COM_TRACE_STR("\n");
    return affine_umve_idx;
}
#endif

static u8 decode_split_flag(DEC_CTX *c, int cu_width, int cu_height, int x, int y, COM_BSR * bs, DEC_SBAC * sbac/*, DEC_CTX * ctx*/)
{
    //split flag
    int ctx = 0;
    int x_scu = x >> MIN_CU_LOG2;
    int y_scu = y >> MIN_CU_LOG2;
    int pic_width_in_scu = c->info.pic_width >> MIN_CU_LOG2;
    u8  avail[2] = { 0, 0 };
    int scun[2];
    int scup = x_scu + y_scu * pic_width_in_scu;
    u8  bSplitFlag;

    scun[0] = scup - pic_width_in_scu;
    scun[1] = scup - 1;
    if (y_scu > 0)
    {
        avail[0] = MCU_GET_CODED_FLAG(c->map.map_scu[scun[0]]);  //up
    }
    if (x_scu > 0)
    {
        avail[1] = MCU_GET_CODED_FLAG(c->map.map_scu[scun[1]]); //left
    }

    if (avail[0])
    {
        ctx += (1 << MCU_GET_LOGW(c->map.map_cu_mode[scun[0]])) < cu_width;
    }
    if (avail[1])
    {
        ctx += (1 << MCU_GET_LOGH(c->map.map_cu_mode[scun[1]])) < cu_height;
    }
#if SEP_CONTEXT
    if (c->info.pic_header.slice_type == SLICE_I && cu_width == 128 && cu_height == 128)
    {
        ctx = 3;
    }
#endif

    bSplitFlag = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.split_flag + ctx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("split flag ");
    COM_TRACE_INT(bSplitFlag);
    COM_TRACE_STR("\n");
    return bSplitFlag;
}

static u8 decode_affine_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    int cu_width = 1 << mod_info_curr->cu_width_log2;
    int cu_height = 1 << mod_info_curr->cu_height_log2;
    u8  affine_flag = 0;
    if (cu_width >= AFF_SIZE && cu_height >= AFF_SIZE && ctx->info.sqh.affine_enable_flag)
    {
        affine_flag = (u8)dec_sbac_decode_bin((bs), (sbac), (sbac)->ctx.affine_flag);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("affine flag ");
    COM_TRACE_INT(affine_flag);
    COM_TRACE_STR("\n");
    return affine_flag;
}

#if ETMVP
static u8 decode_etmvp_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    int cu_width = 1 << mod_info_curr->cu_width_log2;
    int cu_height = 1 << mod_info_curr->cu_height_log2;
    u8  etmvp_flag = 0;

    if (cu_width >= MIN_ETMVP_SIZE && cu_height >= MIN_ETMVP_SIZE)
    {
        etmvp_flag = (u8)dec_sbac_decode_bin((bs), (sbac), (sbac)->ctx.etmvp_flag);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("etmvp flag ");
    COM_TRACE_INT(etmvp_flag);
    COM_TRACE_STR("\n");
    return etmvp_flag;
}

static u8 decode_etmvp_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int ctx_idx = 0;
    u32 etmvp_idx = 0;
    const u32 max_skip_num = MAX_ETMVP_NUM;

    int symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.etmvp_idx[ctx_idx]);
    while (!symbol && etmvp_idx < max_skip_num - 2)
    {
        ctx_idx++;
        etmvp_idx++;
        ctx_idx = min(ctx_idx, NUM_SBAC_CTX_ETMVP_IDX - 1);
        symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.etmvp_idx[ctx_idx]);
    }
    if (!symbol)
    {
        etmvp_idx++;
        assert(etmvp_idx == max_skip_num - 1);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("etmvp idx ");
    COM_TRACE_INT(etmvp_idx)
        COM_TRACE_STR("\n");
    return (u8)etmvp_idx;
}
#endif

static u8 decode_affine_mrg_idx( COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx )
{
    u8 mrg_idx = (u8)sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.affine_mrg_idx, NUM_SBAC_CTX_AFFINE_MRG, AFF_MAX_NUM_MRG);

    COM_TRACE_COUNTER;
    COM_TRACE_STR("affine merge index ");
    COM_TRACE_INT(mrg_idx);
    COM_TRACE_STR("\n");
    return mrg_idx;
}

#if AWP
static u8 decode_awp_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u8  awp_flag = (u8)dec_sbac_decode_bin((bs), (sbac), (sbac)->ctx.awp_flag);

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp flag ");
    COM_TRACE_INT(awp_flag);
    COM_TRACE_STR("\n");
    return awp_flag;
}

static u8 decode_awp_mode(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    int awp_partition_idx = sbac_read_truncate_binary_sym_ep(bs, sbac, AWP_MODE_NUM);//partition_idx

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp partition idx ");
    COM_TRACE_INT(awp_partition_idx);
    COM_TRACE_STR("\n");

    int numCandminus1 = AWP_MV_LIST_LENGTH - 1;
    int mergeCand0 = 0;
    int mergeCand1 = 0;
#if AWP_BIN_FIX
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]) == 0)
    {
        mergeCand0 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[1]) == 0)
    {
        mergeCand1 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1 - 1) + 1;
    }
#else
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]))
    {
        mergeCand0 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[1]))
    {
        mergeCand1 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1 - 1) + 1;
    }
#endif
    mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;

    mod_info_curr->awp_idx0 = mergeCand1;
    mod_info_curr->awp_idx1 = mergeCand0;

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp cand idx0 ");
    COM_TRACE_INT(mergeCand1);
    COM_TRACE_STR("\n");
    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp cand idx1 ");
    COM_TRACE_INT(mergeCand0);
    COM_TRACE_STR("\n");

    return awp_partition_idx;
}
#endif

#if AWP_MVR
static u8 decode_awp_mode1(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    int awp_partition_idx = sbac_read_truncate_binary_sym_ep(bs, sbac, AWP_MODE_NUM);//partition_idx

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp partition idx ");
    COM_TRACE_INT(awp_partition_idx);
    COM_TRACE_STR("\n");

    int numCandminus1 = AWP_MV_LIST_LENGTH - 1;
    int mergeCand0 = 0;
    int mergeCand1 = 0;
#if AWP_BIN_FIX
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]) == 0)
    {
        mergeCand0 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]) == 0)
    {
        mergeCand1 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
#else
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]))
    {
        mergeCand0 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
    if (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_idx[0]))
    {
        mergeCand1 += sbac_read_truncate_unary_sym_ep(bs, sbac, numCandminus1) + 1;
    }
#endif

    mod_info_curr->awp_idx0 = mergeCand1;
    mod_info_curr->awp_idx1 = mergeCand0;

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp cand idx0 ");
    COM_TRACE_INT(mergeCand1);
    COM_TRACE_STR("\n");
    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp cand idx1 ");
    COM_TRACE_INT(mergeCand0);
    COM_TRACE_STR("\n");

    return awp_partition_idx;
}

static u8 decode_awp_mvr_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u8  awp_mvr_flag = (u8)dec_sbac_decode_bin((bs), (sbac), &(sbac)->ctx.awp_mvr_flag);

    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp umve flag ");
    COM_TRACE_INT(awp_mvr_flag);
    COM_TRACE_STR("\n");
    return awp_mvr_flag;
}

static u8 decode_awp_mvr_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int step_idx = 0, dir_idx;
    u8  awp_mvr_idx = -1;
    int num_cand_minus1_step = AWP_MVR_REFINE_STEP - 1;
    int idx0;

    // step size
    if (num_cand_minus1_step > 0)
    {
        idx0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.awp_mvr_step_idx);  // 1 context
        if (!idx0)
        {
            step_idx++;
            for (; step_idx < num_cand_minus1_step; step_idx++)
            {
                idx0 = sbac_decode_bin_ep(bs, sbac);
                if (idx0)
                {
                    break;
                }
            }
        }
    }

    // direction
    dir_idx = 0;
    idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_mvr_dir_idx[0]);
    if (idx0)
    {
        dir_idx += 2;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_mvr_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    else
    {
        dir_idx += 0;
        idx0 = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.awp_mvr_dir_idx[1]);
        if (idx0)
        {
            dir_idx += 1;
        }
        else
        {
            dir_idx += 0;
        }
    }
    awp_mvr_idx = (step_idx * AWP_MVR_DIR + dir_idx);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("awp umve idx ");
    COM_TRACE_INT(awp_mvr_idx);
    COM_TRACE_STR("\n");
    return awp_mvr_idx;
}
#endif

#if SMVD
static int decode_smvd_flag( COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx )
{
    u32 smvd_flag = 0;

    smvd_flag = dec_sbac_decode_bin( bs, sbac, sbac->ctx.smvd_flag );

    COM_TRACE_COUNTER;
    COM_TRACE_STR("smvd flag ");
    COM_TRACE_INT(smvd_flag);
    COM_TRACE_STR("\n");

    return smvd_flag;
}
#endif

#if DT_SYNTAX
static int decode_part_size(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx, int cu_w, int cu_h, int pred_mode)
{
    int part_size = SIZE_2Nx2N;
    int allowDT = com_dt_allow(cu_w, cu_h, pred_mode, ctx->info.sqh.max_dt_size);
    int sym, dir, eq;

    if (!ctx->info.sqh.dt_intra_enable_flag && pred_mode == MODE_INTRA)
        return SIZE_2Nx2N;
    if (!allowDT)
        return SIZE_2Nx2N;

    sym = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 0);
    if (sym == 1)
    {
        int hori_allow = (allowDT >> 0) & 0x01;
        int vert_allow = (allowDT >> 1) & 0x01;
        if (hori_allow && vert_allow)
        {
            dir = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 1);
        }
        else
        {
            dir = hori_allow ? 1 : 0;
        }

        if (dir)
        {
            //hori
            eq = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 2);
            if (eq)
            {
                part_size = SIZE_2NxhN;
            }
            else
            {
                sym = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 3);
                part_size = sym ? SIZE_2NxnD : SIZE_2NxnU;
            }
        }
        else
        {
            //vert
            eq = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 4);
            if (eq)
            {
                part_size = SIZE_hNx2N;
            }
            else
            {
                sym = dec_sbac_decode_bin(bs, sbac, sbac->ctx.part_size + 5);
                part_size = sym ? SIZE_nRx2N : SIZE_nLx2N;
            }
        }
    }
    else
    {
        part_size = SIZE_2Nx2N;
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("part_size ");
    COM_TRACE_INT(part_size);
    COM_TRACE_STR("\n");
    return part_size;
}
#endif
static u8 decode_pred_mode(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
#if NUM_PRED_MODE_CTX > 1
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u32 *map_scu = ctx->map.map_scu;
    u8  avail[2] = { 0, 0 };
    int scun[2];
    int ctx_inc = 0;
    u8  pred_mode;
    scun[0] = mod_info_curr->scup - ctx->info.pic_width_in_scu;
    scun[1] = mod_info_curr->scup - 1;

    if (mod_info_curr->y_scu > 0)
    {
        avail[0] = MCU_GET_CODED_FLAG(map_scu[scun[0]]); // up
    }
    if (mod_info_curr->x_scu > 0)
    {
        avail[1] = MCU_GET_CODED_FLAG(map_scu[scun[1]]); // left
    }
    if (avail[0])
    {
        ctx_inc += MCU_GET_INTRA_FLAG(map_scu[scun[0]]);
    }
    if (avail[1])
    {
        ctx_inc += MCU_GET_INTRA_FLAG(map_scu[scun[1]]);
    }

    if (ctx_inc == 0)
    {
        int sample = (1 << mod_info_curr->cu_width_log2) * (1 << mod_info_curr->cu_height_log2);
        ctx_inc = (sample > 256) ? 0 : (sample > 64 ? 3 : 4);
    }

#if SEP_CONTEXT
    if (mod_info_curr->cu_width_log2 > 6 || mod_info_curr->cu_height_log2 > 6)
    {
        ctx_inc = 5;
    }
#endif
    pred_mode = dec_sbac_decode_bin(bs, sbac, sbac->ctx.pred_mode + ctx_inc) ? MODE_INTRA : MODE_INTER;
#else
    u8 pred_mode = dec_sbac_decode_bin(bs, sbac, sbac->ctx.pred_mode) ? MODE_INTRA : MODE_INTER;
#endif
    COM_TRACE_COUNTER;
    COM_TRACE_STR("pred mode ");
    COM_TRACE_INT(pred_mode);
#if NUM_PRED_MODE_CTX > 1
    COM_TRACE_STR(" pred mode ctx ");
    COM_TRACE_INT(ctx_inc);
#endif
    COM_TRACE_STR("\n");
    return pred_mode;
}

static int decode_mvr_idx(COM_BSR * bs, DEC_SBAC * sbac, BOOL is_affine)
{
#if !BD_AFFINE_AMVR
    if (is_affine)
        return 0;
#endif

#if BD_AFFINE_AMVR
    int mvr_idx;
    if (is_affine)
    {
        mvr_idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.affine_mvr_idx, NUM_AFFINE_MVR_IDX_CTX, MAX_NUM_AFFINE_MVR);
    }
    else
    {
        mvr_idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvr_idx, NUM_MVR_IDX_CTX, MAX_NUM_MVR);
    }
#else
    int mvr_idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvr_idx, NUM_MVR_IDX_CTX, MAX_NUM_MVR);
#endif
    COM_TRACE_COUNTER;
    COM_TRACE_STR("mvr idx ");
    COM_TRACE_INT(mvr_idx);
    COM_TRACE_STR("\n");
    return mvr_idx;
}

#if IBC_ABVR
static int decode_bvr_idx(COM_BSR * bs, DEC_SBAC * sbac)
{
    int bvr_idx;
    bvr_idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.bvr_idx, NUM_BVR_IDX_CTX, MAX_NUM_BVR) ? 2 : 0;
    COM_TRACE_COUNTER;
    COM_TRACE_STR("bvr idx ");
    COM_TRACE_INT(bvr_idx);
    COM_TRACE_STR("\n");
    return bvr_idx;
}
#endif

#if EXT_AMVR_HMVP
static int decode_extend_amvr_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    u32 mvp_from_hmvp_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.mvp_from_hmvp_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("extended amvr flag ");
    COM_TRACE_INT(mvp_from_hmvp_flag);
    COM_TRACE_STR("\n");
    return mvp_from_hmvp_flag;
}
#endif

#if IBC_BVP
static u8 decode_ibc_bvp_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int ctx_idx = 0;
    u8 bvp_idx = 0;
    const int max_skip_num = MAX_NUM_BVP;

    int symbol = max_skip_num > 1 ? dec_sbac_decode_bin(bs, sbac, &sbac->ctx.cbvp_idx[ctx_idx]) : 1;
    while (!symbol && bvp_idx < max_skip_num - 2)
    {
        ctx_idx++;
        bvp_idx++;
        ctx_idx = min(ctx_idx, max_skip_num - 1);
        symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.cbvp_idx[ctx_idx]);
    }
    if (!symbol)
    {
        bvp_idx++;
        assert(bvp_idx == max_skip_num - 1);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("bvp idx ");
    COM_TRACE_INT(bvp_idx);

    COM_TRACE_STR("\n");
    return bvp_idx;
}
#endif

static int decode_ipf_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    int ipf_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.ipf_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("ipf flag ");
    COM_TRACE_INT(ipf_flag);
    COM_TRACE_STR("\n");
    return ipf_flag;
}

static u32 dec_eco_abs_mvd(COM_BSR *bs, DEC_SBAC *sbac, SBAC_CTX_MODEL *model)
{
    u32 act_sym;

    if (!dec_sbac_decode_bin(bs, sbac, model))
    {
        act_sym = 0;
    }
    else if (!dec_sbac_decode_bin(bs, sbac, model + 1))
    {
        act_sym = 1;
    }
    else if (!dec_sbac_decode_bin(bs, sbac, model + 2))
    {
        act_sym = 2;
    }
    else
    {
        int add_one = sbac_decode_bin_ep(bs, sbac);
        int golomb_order = 0;
        act_sym = 0;
        golomb_order = sbac_read_unary_sym_ep(bs, sbac);
        act_sym += (1 << golomb_order) - 1;
        act_sym += sbac_read_bins_ep_msb(bs, sbac, golomb_order);
        act_sym = 3 + act_sym * 2 + add_one;
    }

    return act_sym;
}

static int decode_mvd(COM_BSR * bs, DEC_SBAC * sbac, s16 mvd[MV_D])
{
    u32 sign;
    s16 t16;
    /* MV_X */
    t16 = (s16)dec_eco_abs_mvd(bs, sbac, sbac->ctx.mvd[0]);
    if (t16 == 0)
    {
        mvd[MV_X] = 0;
    }
    else
    {
        /* sign */
        sign = sbac_decode_bin_ep(bs, sbac);
        if(sign) mvd[MV_X] = -t16;
        else mvd[MV_X] = t16;
    }
    /* MV_Y */
    t16 = (s16)dec_eco_abs_mvd(bs, sbac, sbac->ctx.mvd[1]);
    if (t16 == 0)
    {
        mvd[MV_Y] = 0;
    }
    else
    {
        /* sign */
        sign = sbac_decode_bin_ep(bs, sbac);
        if(sign) mvd[MV_Y] = -t16;
        else mvd[MV_Y] = t16;
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("mvd x ");
    COM_TRACE_INT(mvd[MV_X]);
    COM_TRACE_STR("mvd y ");
    COM_TRACE_INT(mvd[MV_Y]);
    COM_TRACE_STR("\n");
    return COM_OK;
}

#if USE_SP
static u32 dec_exgolomb_abs_mvd(COM_BSR *bs, DEC_SBAC *sbac, u8 golomb_order)
{
    int sig, l, act_sym, binary_symbol = 0;
    act_sym = 0;
    do
    {
        l = sbac_decode_bin_ep(bs, sbac);
        if (l == 0)
        {
            act_sym += (1 << golomb_order);
            golomb_order++;
        }
    } while (l != 1);
    while (golomb_order--)
    {
        //next binary part
        sig = sbac_decode_bin_ep(bs, sbac);
        if (sig == 1)
        {
            binary_symbol |= (1 << golomb_order);
        }
    }
    act_sym += binary_symbol;
    return act_sym;
}

static u8 dec_eco_sp_n_recent_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    u8 sp_n_recent_flag = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.sp_n_recent_flag);
    COM_TRACE_COUNTER;
    COM_TRACE_STR("sp_n_recent flag ");
    COM_TRACE_INT(sp_n_recent_flag);
    COM_TRACE_STR("\n");
    return sp_n_recent_flag;
}

static s8 dec_eco_sp_n_recent_index(COM_BSR * bs, DEC_SBAC * sbac)
{
    int ctx_idx = 0;
    s8 n_recent_idx = 0;
    const int max_skip_num = SP_RECENT_CANDS;
    int symbol = max_skip_num > 1 ? dec_sbac_decode_bin(bs, sbac, &sbac->ctx.sp_n_index[ctx_idx]) : 1;
    while (!symbol && n_recent_idx < max_skip_num - 2)
    {
        ctx_idx++;
        n_recent_idx++;
        ctx_idx = min(ctx_idx, max_skip_num - 1);
        symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.sp_n_index[ctx_idx]);
    }
    if (!symbol)
    {
        n_recent_idx++;
        assert(n_recent_idx == max_skip_num - 1);
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("n recent idx ");
    COM_TRACE_INT(n_recent_idx);
    COM_TRACE_STR("\n");
    return n_recent_idx;
}

static void dec_eco_sp_mvd_hor(DEC_CORE * core, COM_BSR * bs, DEC_SBAC * sbac, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag)
{
    u8 above_offset;
    u8 offset_y_zero, offset_x_zero;
    s16 t16;
    u32 sign;
    above_offset = dec_eco_above_offset(bs, sbac);
    if (above_offset == 0)
    {
        mvd[MV_X] = 0;
#if SP_ALIGN_SIGN
        mvd[MV_Y] = -1;
#else
        mvd[MV_Y] = 1;
#endif

    }
    else
    {
        if (dec_eco_sp_n_recent_flag(bs, sbac))
        {
            s8 sp_n_idx = dec_eco_sp_n_recent_index(bs, sbac);
            com_derive_sp_offset(core->n_offset_num, core->n_recent_offset, sp_n_idx, mvd);
        }
        else
        {
            offset_y_zero = dec_eco_offset_zero(bs, sbac, 0);
            if (offset_y_zero == 0)
            {
#if SP_ALIGN_SIGN
                sign = 1;
#else
                sign = 0;
#endif
                mvd[MV_Y] = offset_y_zero;
                if (offset_sign_flag == 1)
                {
                    sign = sbac_decode_bin_ep(bs, sbac);
                }
                t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                mvd[MV_X] = t16 + 1;
                mvd[MV_X] = (sign == 1) ? -mvd[MV_X] : mvd[MV_X];
            }
            else
            {
                sign = sbac_decode_bin_ep(bs, sbac);
                t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                mvd[MV_Y] = t16 + 1;
                mvd[MV_Y] = (sign == 1) ? -mvd[MV_Y] : mvd[MV_Y];
#if SP_ALIGN_SIGN
                if (mvd[MV_Y] > 0)
#else
                if (mvd[MV_Y] < 0)
#endif
                {
                    t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                    mvd[MV_X] = t16 + 1 + offset_in_cu;
#if SP_ALIGN_SIGN
                    mvd[MV_X] = -mvd[MV_X];
#endif

                }
                else
                {
                    offset_x_zero = dec_eco_offset_zero(bs, sbac, 1);
                    if (offset_x_zero == 0)
                    {
                        mvd[MV_X] = 0;
                    }
                    else
                    {
                        sign = sbac_decode_bin_ep(bs, sbac);
                        t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                        mvd[MV_X] = t16 + 1;
                        mvd[MV_X] = (sign == 1) ? -mvd[MV_X] : mvd[MV_X];
                    }
                }
            }
#if SP_ALIGN_SIGN
            if (mvd[MV_Y] <= -1 && mvd[MV_X] == 0)
            {
                mvd[MV_Y]--;
            }
#else
            if (mvd[MV_Y] >= 1 && mvd[MV_X] == 0)
            {
                mvd[MV_Y]++;
            }
#endif

        }
    }
}

static void dec_eco_sp_mvd_ver(DEC_CORE * core, COM_BSR * bs, DEC_SBAC * sbac, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag)
{
    u8 above_offset;
    u8 offset_y_zero, offset_x_zero;
    s16 t16;
    u32 sign;
    above_offset = dec_eco_above_offset(bs, sbac);
    if (above_offset == 0)
    {
#if SP_ALIGN_SIGN
        mvd[MV_X] = -1;
#else
        mvd[MV_X] = 1;
#endif
        mvd[MV_Y] = 0;
    }
    else
    {
        if (dec_eco_sp_n_recent_flag(bs, sbac))
        {
            s8 sp_n_idx = dec_eco_sp_n_recent_index(bs, sbac);
            com_derive_sp_offset(core->n_offset_num, core->n_recent_offset, sp_n_idx, mvd);
        }
        else
        {
            offset_y_zero = dec_eco_offset_zero(bs, sbac, 0);
            if (offset_y_zero == 0)
            {
                mvd[MV_Y] = offset_y_zero;
                t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
#if SP_ALIGN_SIGN
                mvd[MV_X] = -t16 - 1;
#else
                mvd[MV_X] = t16 + 1;
#endif
            }
            else
            {
                sign = sbac_decode_bin_ep(bs, sbac);
                t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                mvd[MV_Y] = t16 + 1;
                mvd[MV_Y] = (sign == 1) ? -mvd[MV_Y] : mvd[MV_Y];
#if SP_ALIGN_SIGN
                if (mvd[MV_Y] > 0)
#else
                if (mvd[MV_Y] < 0)
#endif
                {
                    t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                    if (offset_sign_flag == 1)
                    {
                        t16 -= 1;
                    }
                    else
                    {
                        t16 += 1;
                    }
#if SP_ALIGN_SIGN
                    mvd[MV_X] = -t16;
#else
                    mvd[MV_X] = t16;
#endif
                }
                else
                {
                    offset_x_zero = dec_eco_offset_zero(bs, sbac, 1);
                    if (offset_x_zero == 0)
                    {
                        mvd[MV_X] = 0;
                    }
                    else
                    {
                        sign = sbac_decode_bin_ep(bs, sbac);
                        t16 = dec_exgolomb_abs_mvd(bs, sbac, SP_EXG_ORDER);
                        mvd[MV_X] = t16 + 1;
                        mvd[MV_X] = (sign == 1) ? -mvd[MV_X] : mvd[MV_X];
                    }
                }
            }
#if SP_ALIGN_SIGN
            if (mvd[MV_X] <= -1 && mvd[MV_Y] == 0)
            {
                mvd[MV_X]--;
            }
#else
            if (mvd[MV_X] >= 1 && mvd[MV_Y] == 0)
            {
                mvd[MV_X]++;
            }
#endif
        }
    }
}

static void dec_eco_sp_mvd(DEC_CORE * core, COM_BSR * bs, DEC_SBAC * sbac, COM_SP_INFO *p_sp_info, u8 sp_copy_dir, int cu_width, int cu_height, int cur_pixel, s16 mvd[MV_D])
{
    int* p_trav_scan_order = com_tbl_raster2trav[sp_copy_dir][CONV_LOG2(cu_width) - MIN_CU_LOG2][CONV_LOG2(cu_height) - MIN_CU_LOG2];
    int  trav_order_index = p_trav_scan_order[cur_pixel];
    int  offset_sign_flag = 0, above_flag = 0, offset_in_cu = 0;
    int  offset_x_in_cu = GET_TRAV_X(trav_order_index, cu_width);
    int  offset_y_in_cu = GET_TRAV_Y(trav_order_index, CONV_LOG2(cu_width));
    if (sp_copy_dir == 1)// hor
    {
        int offset_map_value = offset_x_in_cu;
#if SP_ALIGN_SIGN
        if (p_sp_info->offset_x == 0 && p_sp_info->offset_y == -1)
#else
        if (p_sp_info->offset_x == 0 && p_sp_info->offset_y == 1) 
#endif
        {
            above_flag = 1;
        }
        if (1 == (offset_y_in_cu & 0x1))
        {
            offset_sign_flag = 1;
        }
        if (1 == (offset_y_in_cu & 0x1))
        {
            offset_map_value++;
        }
        else
        {
            offset_map_value += p_sp_info->length;
        }
        offset_in_cu = min(cu_width, offset_map_value);
        offset_in_cu--;
        dec_eco_sp_mvd_hor(core, bs, sbac, mvd, offset_in_cu, offset_sign_flag);
    }
    else
    {
        int offset_map_value = offset_y_in_cu;
#if SP_ALIGN_SIGN
        if (p_sp_info->offset_x == -1 && p_sp_info->offset_y == 0)
#else
        if (p_sp_info->offset_x == 1 && p_sp_info->offset_y == 0)
#endif
        {
            above_flag = 1;
        }
        if (1 == (offset_x_in_cu & 0x1))
        {
            offset_sign_flag = 1;
        }
        if (1 == (offset_x_in_cu & 0x1))
        {
            offset_map_value++;
        }
        else
        {
            offset_map_value += p_sp_info->length;
        }
        offset_in_cu = min(cu_height, offset_map_value);
        offset_in_cu--;
        dec_eco_sp_mvd_ver(core, bs, sbac, mvd, offset_in_cu, offset_sign_flag);
    }
}

static void dec_eco_sp(DEC_CTX * ctx, DEC_CORE * core, COM_BSR * bs, DEC_SBAC * sbac, COM_MODE * mod_info_curr, CHANNEL_TYPE channel)
{
    assert(ctx->tree_status != TREE_C);
    int bit_depth = ctx->info.bit_depth_internal;
    s16 sp_mv[MV_D];
    int cur_pixel = 0;
    int width = 1 << mod_info_curr->cu_width_log2;
    int height = 1 << mod_info_curr->cu_height_log2;
    int total_pixel = width * height;
    int sub_string_no = 0;
    u8 is_sp_special_len = FALSE;
    u8 str_len_mode = NONE_TYPE;
    COM_SP_INFO *p_sp_info = mod_info_curr->string_copy_info;
#if SP_BASE
    mod_info_curr->sp_copy_direction = 1;
#else
    mod_info_curr->sp_copy_direction = dec_eco_sp_copy_dir_flag(bs, sbac);
#endif
    while (cur_pixel < total_pixel) 
    {
        p_sp_info->is_matched = dec_eco_sp_is_matched_flag(bs, sbac);
        if (p_sp_info->is_matched == TRUE)
        {
            is_sp_special_len = dec_eco_sp_special_len_flag(bs, sbac);
            if (is_sp_special_len)
            {
                str_len_mode = LAST_STR;
                p_sp_info->length = width * height - cur_pixel;
            }
            else
            {
                int next_remianing_pixel_in_cu = dec_eco_sp_string_length(bs, sbac, total_pixel - cur_pixel - 1);
                next_remianing_pixel_in_cu++;
                p_sp_info->length = width * height - cur_pixel - next_remianing_pixel_in_cu;            
            }
            dec_eco_sp_mvd(core, bs, sbac, p_sp_info, mod_info_curr->sp_copy_direction, width, height, cur_pixel, sp_mv);
            p_sp_info->offset_x = sp_mv[MV_X];
            p_sp_info->offset_y = sp_mv[MV_Y];
            cur_pixel += p_sp_info->length;
            sub_string_no++;
        }
        else 
        {
            int* p_trav_scan_order = com_tbl_raster2trav[mod_info_curr->sp_copy_direction][mod_info_curr->cu_width_log2 - MIN_CU_LOG2][mod_info_curr->cu_height_log2 - MIN_CU_LOG2];
            int  trav_order_index = p_trav_scan_order[cur_pixel];
            int  trav_x = GET_TRAV_X(trav_order_index, 1 << mod_info_curr->cu_width_log2);
            int  trav_y = GET_TRAV_Y(trav_order_index, mod_info_curr->cu_width_log2);
            dec_eco_pixel_y(bs, sbac, bit_depth, p_sp_info->pixel);
            if (ctx->tree_status != TREE_L)
            {
                if (!(ctx->info.sqh.chroma_format <= 1 && (trav_x & 0x1 || trav_y & 0x1))) 
                {
                    dec_eco_pixel_uv(bs, sbac, bit_depth, p_sp_info->pixel);
                }
            }
            cur_pixel++;
            sub_string_no++;
        }
        p_sp_info++;
    }
    mod_info_curr->sub_string_no = sub_string_no;
}
#endif

#if EST
u8 decode_est_flag(COM_BSR * bs, DEC_SBAC * sbac)
{
    return (u8)dec_sbac_decode_bin((bs), (sbac), (sbac)->ctx.est_flag);
}
#endif

int decode_coef(DEC_CTX * ctx, DEC_CORE * core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    u8        cbf[MAX_NUM_TB][N_C];
    DEC_SBAC *sbac;
    COM_BSR   *bs;
#if IPCM
    int i, j, ret = 0;
#else
    int i, j, ret;
#endif
    int tb_avaliable = is_tb_avaliable(ctx->info, mod_info_curr);
    int start_comp, num_comp;
    start_comp = (ctx->tree_status == TREE_L || ctx->tree_status == TREE_LC) ? Y_C : U_C;
    num_comp = ctx->tree_status == TREE_LC ? 3 : (ctx->tree_status == TREE_L ? 1 : 2);

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    if (ctx->tree_status != TREE_C)
    {
#if IPCM
        ret = dec_eco_cbf(bs, sbac, tb_avaliable, mod_info_curr->pb_part, &mod_info_curr->tb_part, (u8)mod_info_curr->cu_mode, mod_info_curr->ipm, cbf, ctx->tree_status, ctx);
#else
        ret = dec_eco_cbf(bs, sbac, tb_avaliable, mod_info_curr->pb_part, &mod_info_curr->tb_part, (u8)mod_info_curr->cu_mode, cbf, ctx->tree_status, ctx);
#endif
    }
    else
    {
#if IPCM
        if (!(mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ipm[PB0][0] == IPD_IPCM && mod_info_curr->ipm[PB0][1] == IPD_DM_C))
        {
#endif
#if PMC
            s8 ipm_c = mod_info_curr->cu_mode == MODE_INTRA ? mod_info_curr->ipm[PB0][1] : -1;
#endif
            ret = dec_eco_cbf_uv(bs, sbac, cbf
#if PMC
                                ,ipm_c
#endif
            );
#if IPCM
        }
#endif
    }
    com_assert_rv(ret == COM_OK, ret);

    for (i = start_comp; i < start_comp + num_comp; i++)
    {
        int part_num;
        int log2_tb_w, log2_tb_h, tb_size;
        int plane_width_log2  = mod_info_curr->cu_width_log2  - (i != Y_C);
        int plane_height_log2 = mod_info_curr->cu_height_log2 - (i != Y_C);

#if IPCM
        if (mod_info_curr->cu_mode == MODE_INTRA && ((i == Y_C && mod_info_curr->ipm[PB0][0] == IPD_IPCM) || (i > Y_C && mod_info_curr->ipm[PB0][0] == IPD_IPCM && mod_info_curr->ipm[PB0][1] == IPD_DM_C)))
        {
            if (i == start_comp)
            {
                /* aec_ipcm_stuffing_bit and byte alignment (bit always = 1) inside the function */
                assert(dec_sbac_decode_bin_trm(bs, sbac) == 1);
            }
            int tb_w = plane_width_log2 > 5 ? 32 : (1 << plane_width_log2);
            int tb_h = plane_height_log2 > 5 ? 32 : (1 << plane_height_log2);
            int num_tb_w = plane_width_log2 > 5 ? 1 << (plane_width_log2 - 5) : 1;
            int num_tb_h = plane_height_log2 > 5 ? 1 << (plane_height_log2 - 5) : 1;
            for (int h = 0; h < num_tb_h; h++)
            {
                for (int w = 0; w < num_tb_w; w++)
                {
                    s16* coef_tb = mod_info_curr->coef[i] + (1 << plane_width_log2) * h * tb_h + w * tb_w;
                    decode_ipcm(bs, sbac, tb_w, tb_h, 1 << plane_width_log2, coef_tb, ctx->info.bit_depth_input, i);
                }
            }
            if ((i == Y_C && (ctx->tree_status == TREE_L || (ctx->tree_status == TREE_LC && mod_info_curr->ipm[0][1] != IPD_DM_C))) || i == V_C)
            {
                assert(COM_BSR_IS_BYTE_ALIGN(bs));
                dec_sbac_init(bs);
            }
        }
        else
        {
#endif
            part_num = get_part_num(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N);
            get_tb_width_height_log2(plane_width_log2, plane_height_log2, i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, &log2_tb_w, &log2_tb_h);
#if SBT
            get_sbt_tb_size( mod_info_curr->sbt_info, i, log2_tb_w, log2_tb_h, &log2_tb_w, &log2_tb_h );
#endif
            tb_size = 1 << (log2_tb_w + log2_tb_h);

            for (j = 0; j < part_num; j++)
            {
                if (cbf[j][i])
                {
#if IST
                    int ist_flag = 0;
#endif
                    ret = dec_eco_xcoef(ctx, bs, sbac, mod_info_curr->coef[i] + j * tb_size, log2_tb_w, log2_tb_h, i
#if IST
                        , &ist_flag
#endif
                        );
#if IST
                    if (ctx->info.sqh.ist_enable_flag)
                    {
                        int nz_cnt = 0;
#if ISTS
                        if (i == Y_C && part_num == 1 && plane_width_log2 < 6 && plane_height_log2 < 6 &&
                            ((mod_info_curr->cu_mode == MODE_INTRA && ctx->info.pic_header.ph_ists_enable_flag == 0) ||
                            (ctx->info.sqh.ists_enable_flag && ctx->info.pic_header.ph_ists_enable_flag && (mod_info_curr->cu_mode == MODE_INTRA || mod_info_curr->cu_mode == MODE_IBC))))
#else
                        if (mod_info_curr->cu_mode == MODE_INTRA && i == Y_C &&
                            part_num == 1 && plane_width_log2 < 6 && plane_height_log2 < 6)
#endif
                        {
#if SRCC
                            if (ctx->info.sqh.srcc_enable_flag)
                            {
                                mod_info_curr->ist_tu_flag = ist_flag;
                            }
                            else
#endif
                            {
                                int tb_w = 1 << log2_tb_w;
                                int coef_resitrict = 0;
                                for (int p = 0; p < tb_size; p++)
                                {
                                    if ((mod_info_curr->coef[i] + j * tb_size)[p])
                                    {
                                        nz_cnt++;
                                    }
#if ISTS
                                    if (mod_info_curr->cu_mode == MODE_INTRA && !ctx->info.pic_header.ph_ists_enable_flag &&
                                        (log2_tb_w >= 5 || log2_tb_h >= 5) && (mod_info_curr->coef[i] + j * tb_size)[p] && ((p / tb_w) >= 16 || (p % tb_w) >= 16))
#else
                                    if ((log2_tb_w >= 5 || log2_tb_h >= 5) && ((p / tb_w) >= 16 || (p % tb_w) >= 16) && (mod_info_curr->coef[i] + j * tb_size)[p])
#endif
                                    {
                                        coef_resitrict = 1;
                                    }
                                }
                                mod_info_curr->ist_tu_flag = nz_cnt % 2 ? 1 : 0;
                                if (coef_resitrict)
                                {
                                    mod_info_curr->ist_tu_flag = 0;
                                }
                            }
                        }
                    }
#endif
#if EST
                    if (ctx->info.sqh.est_enable_flag && i == Y_C)
                    {
                        mod_info_curr->est_flag = (part_num != 1); //initialization
                        if (part_num == 1 && mod_info_curr->cu_mode == MODE_INTRA &&
                            (mod_info_curr->ist_tu_flag == 0 || plane_width_log2 == 6 || plane_height_log2 == 6))
                        {
                            mod_info_curr->est_flag = decode_est_flag(bs, sbac);
#if ENC_DEC_TRACE
                            COM_TRACE_COUNTER;
                            COM_TRACE_STR("est_flag: ");
                            COM_TRACE_INT(mod_info_curr->est_flag);
                            COM_TRACE_STR("\n");
#endif
                        }
                    }
#endif
                    com_assert_rv(ret == COM_OK, ret);
                    mod_info_curr->num_nz[j][i] = 1;
                }
                else
                {
                    mod_info_curr->num_nz[j][i] = 0;
                }
            }
#if IPCM
        }
#endif
    }

    return COM_OK;
}

void dec_sbac_init(COM_BSR * bs)
{
    DEC_SBAC *sbac = GET_SBAC_DEC(bs);
    sbac->range = 0x1FF;
    sbac->bits_Needed = -8;
    sbac->value = com_bsr_read(bs, READ_BITS_INIT) << 1;
    sbac->bits_Needed++;
}

int decode_intra_dir(COM_BSR * bs, DEC_SBAC * sbac,
#if EIPM
    u8 eipm_flag,
#endif
    u8 mpm[2])
{
    u32 t0, t1, t2, t3, t4, t5, t6;
    int ipm = 0;
    t0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir);
    if (t0 == 1)
    {
        t1 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 6);
        ipm = t1 - 2;
    }
    else
    {
        t2 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 1);
        t3 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 2);
        t4 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 3);
        t5 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 4);
        t6 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 5);
        ipm = (t2 << 4) + (t3 << 3) + (t4 << 2) + (t5 << 1) + t6;
#if EIPM
        if (eipm_flag)
        {
            u32 t7 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 10);
            ipm += (t7 << 5);
        }
#endif
    }
    ipm = (ipm < 0) ? mpm[ipm + 2] : (ipm + (ipm >= mpm[0]) + ((ipm + 1) >= mpm[1]));
    COM_TRACE_COUNTER;
    COM_TRACE_STR("ipm Y ");
    COM_TRACE_INT(ipm);
    COM_TRACE_STR(" mpm_0 ");
    COM_TRACE_INT(mpm[0]);
    COM_TRACE_STR(" mpm_1 ");
    COM_TRACE_INT(mpm[1]);
    COM_TRACE_STR("\n");
    return ipm;
}

#if TSCPM
int decode_intra_dir_c(COM_BSR * bs, DEC_SBAC * sbac, u8 ipm_l, u8 tscpm_enable_flag
#if ENHANCE_TSPCM
    , u8 enhance_tscpm_enable_flag
#endif
#if PMC
    , u8 pmc_enable_flag
#endif
)
#else
int decode_intra_dir_c(COM_BSR * bs, DEC_SBAC * sbac, u8 ipm_l
#if PMC
    , u8 pmc_enable_flag
#endif
)
#endif
{
    u32 t0;
    int ipm = 0;
    u8 chk_bypass;

    COM_IPRED_CONV_L2C_CHK(ipm_l, chk_bypass);
    t0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 7);
    if (t0 == 0)
    {
#if TSCPM || PMC
        int cpm_enable = 0;
#if TSCPM
        cpm_enable |= tscpm_enable_flag;
#endif
#if PMC
        cpm_enable |= pmc_enable_flag;
#endif
        if (cpm_enable)
        {
            int tscpm_ctx = 0;
            if (dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 9))
            {
#if ENHANCE_TSPCM || PMC
                int mCpm_enable = 0;
#if ENHANCE_TSPCM
                mCpm_enable |= enhance_tscpm_enable_flag;
#endif
#if PMC
                mCpm_enable |= pmc_enable_flag;
#endif
                if(mCpm_enable)
                {
#if EIPM
                    if( dec_sbac_decode_bin( bs, sbac, sbac->ctx.intra_dir + 11 ) )
#else
                    if( dec_sbac_decode_bin( bs, sbac, sbac->ctx.intra_dir + 10 ) )
#endif
                    {
                        ipm = IPD_TSCPM_C;
                    }
                    else
                    {
                        if( sbac_decode_bin_ep( bs, sbac ) )
                        {
                            ipm = IPD_TSCPM_L_C;
                        }
                        else
                        {
                            ipm = IPD_TSCPM_T_C;
                        }
                    }
                }
                else
                {
                    ipm = IPD_TSCPM_C;
                }
#else
                ipm = IPD_TSCPM_C;
#endif
#if TSCPM && PMC
                if (tscpm_enable_flag && pmc_enable_flag)
                {
#if ENHANCE_TSPCM
                    if (!enhance_tscpm_enable_flag && ipm != IPD_TSCPM_C)
#else
                    if (ipm != IPD_TSCPM_C)
#endif
                    {
                        ipm = ipm + 3;
                    }
                    else
                    {
                        if (dec_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 12))
                        {
                            ipm = ipm + 3;
                        }
                    }
                }
                else if (pmc_enable_flag)
                {
                    ipm = ipm + 3;
                }
#elif PMC
                if (pmc_enable_flag)
                {
                    ipm = ipm + 3;
                }
#endif
                COM_TRACE_COUNTER;
                COM_TRACE_STR("ipm UV ");
                COM_TRACE_INT(ipm);
                COM_TRACE_STR("\n");
                return ipm;
            }
        }
#endif

        ipm = 1 + sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.intra_dir + 8, 1, IPD_CHROMA_CNT - 1);

        if (chk_bypass)
        {
            assert(ipm < IPD_BI_C); // if isRedundant, ipm, 1,2,3
        }

        if (chk_bypass &&  ipm >= ipm_l)
        {
            ipm++;
        }
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("ipm UV ");
    COM_TRACE_INT(ipm);
    COM_TRACE_STR("\n");
    return ipm;
}

#if IPCM
void decode_ipcm(COM_BSR * bs, DEC_SBAC * sbac, int tb_width, int tb_height, int cu_width, s16 pcm[MAX_CU_DIM], int bit_depth, int ch_type)
{
    int i, j;
#if ENC_DEC_TRACE
    COM_TRACE_STR("pcm_");
    COM_TRACE_INT(ch_type);
    COM_TRACE_STR(":\n");
#endif
    for (i = 0; i < tb_height; i++)
    {
        for (j = 0; j < tb_width; j++)
        {
            pcm[i * cu_width + j] = com_bsr_read(bs, bit_depth);
#if ENC_DEC_TRACE
            COM_TRACE_INT(pcm[i * cu_width + j]);
#endif
        }
#if ENC_DEC_TRACE
        COM_TRACE_STR("\n");
#endif
    }
}
#endif

int decode_inter_dir(COM_BSR * bs, DEC_SBAC * sbac, int part_size, DEC_CTX * ctx)
{
    assert(ctx->info.pic_header.slice_type == SLICE_B);

    int pred_dir;
#if SEP_CONTEXT
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    u32 symbol = 0;
    if (mod_info_curr->cu_width_log2 + mod_info_curr->cu_height_log2 < 6)
    {
        symbol = dec_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 2);
        assert(symbol == 0);
    }
    else
    {
        symbol = dec_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);
    }
#else
    u32 symbol = dec_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);
#endif
    if (symbol)
    {
        pred_dir = PRED_BI;
    }
    else
    {
        symbol = dec_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 1);
        pred_dir = symbol ? PRED_L1 : PRED_L0;
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("pred dir ");
    COM_TRACE_INT(pred_dir);
    COM_TRACE_STR("\n");
    return pred_dir;
}

static u8 decode_skip_idx(COM_BSR * bs, DEC_SBAC * sbac, int num_hmvp_cands, 
#if MVAP
    int num_mvap_cands,
#endif
    DEC_CTX * ctx)
{
    int ctx_idx = 0;
    u32 skip_idx = 0;
#if MVAP
    const u32 max_skip_num = (ctx->info.pic_header.slice_type == SLICE_P ? 2 : TRADITIONAL_SKIP_NUM) + max(num_hmvp_cands, num_mvap_cands);
#else
    const u32 max_skip_num = (ctx->info.pic_header.slice_type == SLICE_P ? 2 : TRADITIONAL_SKIP_NUM) + num_hmvp_cands;
#endif

    int symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.skip_idx_ctx[ctx_idx]);
    while (!symbol && skip_idx < max_skip_num - 2)
    {
        ctx_idx++;
        skip_idx++;
        ctx_idx = min(ctx_idx, NUM_SBAC_CTX_SKIP_IDX - 1);
        symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.skip_idx_ctx[ctx_idx]);
    }
    if (!symbol)
    {
        skip_idx++;
        assert(skip_idx == max_skip_num - 1);
    }

    // for P slice, change 1, 2, ..., 11 to 3, 4, ..., 13
    if (ctx->info.pic_header.slice_type == SLICE_P && skip_idx > 0)
    {
        skip_idx += 2;
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("skip idx ");
    COM_TRACE_INT(skip_idx);
    COM_TRACE_STR(" HmvpSpsNum ");
    COM_TRACE_INT(num_hmvp_cands);
    COM_TRACE_STR("\n");
    return (u8)skip_idx;
}

static u8 decode_direct_flag(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
#if SEP_CONTEXT
    COM_MODE *mod_info_curr = &ctx->core->mod_info_curr;
    int ctx_inc = 0;
    u32 direct_flag;
    if ((mod_info_curr->cu_width_log2 + mod_info_curr->cu_height_log2 < 6) || mod_info_curr->cu_width_log2 > 6 || mod_info_curr->cu_height_log2 > 6)
    {
        ctx_inc = 1;
    }

    direct_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.direct_flag + ctx_inc);
#else
    u32 direct_flag = dec_sbac_decode_bin(bs, sbac, sbac->ctx.direct_flag);
#endif
    COM_TRACE_COUNTER;
    COM_TRACE_STR("direct flag ");
    COM_TRACE_INT(direct_flag);
#if SEP_CONTEXT
    COM_TRACE_STR(" direct flag ctx ");
    COM_TRACE_INT(ctx_inc);
#endif
    COM_TRACE_STR("\n");
    return (u8)direct_flag;
}

#if IBC_BVP
static u8 decode_cbvp_idx(COM_BSR * bs, DEC_SBAC * sbac, DEC_CTX * ctx)
{
    int ctx_idx = 0;
    u8 bvp_idx = 0;
    const int max_skip_num = MAX_NUM_BVP;

    int symbol = max_skip_num > 1 ? dec_sbac_decode_bin(bs, sbac, &sbac->ctx.cbvp_idx[ctx_idx]) : 1;
    while (!symbol && bvp_idx < max_skip_num - 2)
    {
        ctx_idx++;
        bvp_idx++;
        ctx_idx = min(ctx_idx, max_skip_num - 1);
        symbol = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.cbvp_idx[ctx_idx]);
    }
    if (!symbol)
    {
        bvp_idx++;
        assert(bvp_idx == max_skip_num - 1);
    }

    COM_TRACE_COUNTER;
    COM_TRACE_STR("bvp idx ");
    COM_TRACE_INT(bvp_idx);

    COM_TRACE_STR("\n");
    return bvp_idx;
}
#endif

s8 dec_eco_split_mode(DEC_CTX * c, COM_BSR *bs, DEC_SBAC *sbac, int cu_width, int cu_height
                      , const int parent_split, int qt_depth, int bet_depth, int x, int y)
{
    s8 split_mode = NO_SPLIT;
    int ctx = 0;
    u32 t0;
    int split_allow[SPLIT_CHECK_NUM];
    int i, non_QT_split_mode_num;
    int boundary = 0, boundary_b = 0, boundary_r = 0;

    if(cu_width == MIN_CU_SIZE && cu_height == MIN_CU_SIZE)
    {
        split_mode = NO_SPLIT;
        return split_mode;
    }

    boundary = !(x + cu_width <= c->info.pic_width && y + cu_height <= c->info.pic_height);
    boundary_b = boundary && (y + cu_height > c->info.pic_height) && !(x + cu_width  > c->info.pic_width);
    boundary_r = boundary && (x + cu_width  > c->info.pic_width)  && !(y + cu_height > c->info.pic_height);

    com_check_split_mode(&c->info.sqh, split_allow, CONV_LOG2(cu_width), CONV_LOG2(cu_height), boundary, boundary_b, boundary_r, c->info.log2_max_cuwh, c->info.pic_header.temporal_id
                         , parent_split, qt_depth, bet_depth, c->info.pic_header.slice_type);
    non_QT_split_mode_num = 0;
    for(i = 1; i < SPLIT_QUAD; i++)
    {
        non_QT_split_mode_num += split_allow[i];
    }

    if (split_allow[SPLIT_QUAD] && !(non_QT_split_mode_num || split_allow[NO_SPLIT])) //only QT is allowed
    {
        split_mode = SPLIT_QUAD;
        return split_mode;
    }
    else if (split_allow[SPLIT_QUAD])
    {
        u8 bSplitFlag = decode_split_flag(c, cu_width, cu_height, x, y, bs, sbac);
        if (bSplitFlag)
        {
            split_mode = SPLIT_QUAD;
            return split_mode;
        }
    }

    if (non_QT_split_mode_num)
    {
        int cu_width_log2 = CONV_LOG2(cu_width);
        int cu_height_log2 = CONV_LOG2(cu_height);
        //split flag
        int x_scu = x >> MIN_CU_LOG2;
        int y_scu = y >> MIN_CU_LOG2;
        int pic_width_in_scu = c->info.pic_width >> MIN_CU_LOG2;
        u8  avail[2] = {0, 0};
        int scun[2];
        int scup = x_scu + y_scu * pic_width_in_scu;

        scun[0]  = scup - pic_width_in_scu;
        scun[1]  = scup - 1;
        if (y_scu > 0)
        {
            avail[0] = MCU_GET_CODED_FLAG(c->map.map_scu[scun[0]]);  //up
        }
        if (x_scu > 0)
        {
            avail[1] = MCU_GET_CODED_FLAG(c->map.map_scu[scun[1]]); //left
        }
        if (avail[0])
        {
            ctx += (1 << MCU_GET_LOGW(c->map.map_cu_mode[scun[0]])) < cu_width;
        }
        if (avail[1])
        {
            ctx += (1 << MCU_GET_LOGH(c->map.map_cu_mode[scun[1]])) < cu_height;
        }
#if NUM_SBAC_CTX_BT_SPLIT_FLAG == 9
        int sample = cu_width * cu_height;
        int ctx_set = (sample > 1024) ? 0 : (sample > 256 ? 1 : 2);
        int ctx_save = ctx;
        ctx += ctx_set * 3;
#endif

        if (split_allow[NO_SPLIT])
        {
            t0 = dec_sbac_decode_bin(bs, sbac, sbac->ctx.bt_split_flag + ctx);
        }
        else
        {
            t0 = 1;
        }
#if NUM_SBAC_CTX_BT_SPLIT_FLAG == 9
        ctx = ctx_save;
#endif

        if (!t0)
        {
            split_mode = NO_SPLIT;
        }
        else
        {
            int HBT = split_allow[SPLIT_BI_HOR];
            int VBT = split_allow[SPLIT_BI_VER];
            int EnableBT = HBT || VBT;
#if EQT
            int HEQT = split_allow[SPLIT_EQT_HOR];
            int VEQT = split_allow[SPLIT_EQT_VER];
            int EnableEQT = HEQT || VEQT;
#endif
            u8 ctx_dir = cu_width_log2 == cu_height_log2 ? 0 : (cu_width_log2 > cu_height_log2 ? 1 : 2);
            u8 split_dir = 0, split_typ = 0;

#if EQT
            if (EnableEQT && EnableBT)
            {
                split_typ = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx);
            }
            else if (EnableEQT)
            {
                split_typ = 1;
            }
#endif
            if (split_typ == 0)
            {
                if (HBT && VBT)
                {
#if SEP_CONTEXT
                    if (cu_width == 64 && cu_height == 128)
                    {
                        ctx_dir = 3;
                    }
                    if (cu_width == 128 && cu_height == 64)
                    {
                        ctx_dir = 4;
                    }
#endif
                    split_dir = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.split_dir + ctx_dir);
                }
                else
                {
                    split_dir = !HBT;
                }
            }
#if EQT
            if (split_typ == 1)
            {
                if (HEQT && VEQT)
                {
                    split_dir = (u8)dec_sbac_decode_bin(bs, sbac, sbac->ctx.split_dir + ctx_dir);
                }
                else
                {
                    split_dir = !HEQT;
                }
            }
#endif

#if EQT
            if (split_typ == 0) //BT
            {
#endif
                split_mode = split_dir ? SPLIT_BI_VER : SPLIT_BI_HOR;
#if EQT
            }
            else
            {
                split_mode = split_dir ? SPLIT_EQT_VER : SPLIT_EQT_HOR;
            }
#endif
        }
    }
    COM_TRACE_COUNTER;
    COM_TRACE_STR("split mode ");
    COM_TRACE_INT(split_mode);
    COM_TRACE_STR("\n");
    return split_mode;
}

#if CHROMA_NOT_SPLIT
int dec_decode_cu_chroma(DEC_CTX * ctx, DEC_CORE * core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    COM_BSR  *bs = &ctx->bs;
    DEC_SBAC *sbac = GET_SBAC_DEC(bs);
    s8(*map_refi)[REFP_NUM];
    s16(*map_mv)[REFP_NUM][MV_D];
    u32 *map_scu;
    s8  *map_ipm;
    int cu_width = 1 << mod_info_curr->cu_width_log2, cu_height = 1 << mod_info_curr->cu_height_log2;
    int ret;
    cu_nz_cln(mod_info_curr->num_nz);
    init_pb_part(mod_info_curr);
    init_tb_part(mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->tb_part, &mod_info_curr->tb_info);
#if SBT
    mod_info_curr->sbt_info = 0;
#endif

    //fetch luma pred mode
    int cu_w_scu = PEL2SCU(1 << mod_info_curr->cu_width_log2);
    int cu_h_scu = PEL2SCU(1 << mod_info_curr->cu_height_log2);
    int luma_scup = mod_info_curr->x_scu + (cu_w_scu - 1) + (mod_info_curr->y_scu + (cu_h_scu - 1)) * ctx->info.pic_width_in_scu;

#if USE_SP
    u8 * map_usp;
    map_usp = ctx->map.map_usp;
#endif
    map_scu  = ctx->map.map_scu;
    map_refi = ctx->map.map_refi;
    map_mv   = ctx->map.map_mv;
    map_ipm  = ctx->map.map_ipm;
    assert(MCU_GET_CODED_FLAG(map_scu[luma_scup]));

    //prediction
    u8 luma_pred_mode = MCU_GET_INTRA_FLAG(map_scu[luma_scup])? MODE_INTRA : MODE_INTER;
#if USE_IBC
    if (MCU_GET_IBC(map_scu[luma_scup]))
    {
        luma_pred_mode = MODE_INTRA;
    }
#endif
    if (luma_pred_mode != MODE_INTRA)
    {
        mod_info_curr->cu_mode = MODE_INTER;
        //prepare mv to mod_info_curr
        for (int lidx = 0; lidx < REFP_NUM; lidx++)
        {
            mod_info_curr->refi[lidx] = map_refi[luma_scup][lidx];
            mod_info_curr->mv[lidx][MV_X] = map_mv[luma_scup][lidx][MV_X];
            mod_info_curr->mv[lidx][MV_Y] = map_mv[luma_scup][lidx][MV_Y];
            if (!REFI_IS_VALID(mod_info_curr->refi[lidx]))
            {
                assert(map_mv[luma_scup][lidx][MV_X] == 0);
                assert(map_mv[luma_scup][lidx][MV_Y] == 0);
            }
        }

        COM_TRACE_STR("luma pred mode INTER");
        COM_TRACE_STR("\n");
        COM_TRACE_STR("L0: Ref ");
        COM_TRACE_INT(mod_info_curr->refi[0]);
        COM_TRACE_STR("MVX ");
        COM_TRACE_INT(mod_info_curr->mv[0][MV_X]);
        COM_TRACE_STR("MVY ");
        COM_TRACE_INT(mod_info_curr->mv[0][MV_Y]);
        COM_TRACE_STR("\n");
        COM_TRACE_STR("L1: Ref ");
        COM_TRACE_INT(mod_info_curr->refi[1]);
        COM_TRACE_STR("MVX ");
        COM_TRACE_INT(mod_info_curr->mv[1][MV_X]);
        COM_TRACE_STR("MVY ");
        COM_TRACE_INT(mod_info_curr->mv[1][MV_Y]);
        COM_TRACE_STR("\n");
    }
    else
    {
        COM_TRACE_STR("luma pred mode INTRA");
        COM_TRACE_STR("\n");

        mod_info_curr->cu_mode = MODE_INTRA;
        mod_info_curr->ipm[PB0][0] = map_ipm[luma_scup];
#if TSCPM
        mod_info_curr->ipm[PB0][1] = (s8)decode_intra_dir_c(bs, sbac, mod_info_curr->ipm[PB0][0], ctx->info.sqh.tscpm_enable_flag
#if ENHANCE_TSPCM
            , ctx->info.sqh.enhance_tscpm_enable_flag
#endif
#if PMC
            , ctx->info.sqh.pmc_enable_flag
#endif
        );
#else
        mod_info_curr->ipm[PB0][1] = (s8)decode_intra_dir_c(bs, sbac, mod_info_curr->ipm[PB0][0]
#if PMC
            , ctx->info.sqh.pmc_enable_flag
#endif
        );
#endif
    }

    //coeff
    /* clear coefficient buffer */
    com_mset(mod_info_curr->coef[Y_C], 0, cu_width * cu_height * sizeof(s16));
    com_mset(mod_info_curr->coef[U_C], 0, (cu_width >> 1) * (cu_height >> 1) * sizeof(s16));
    com_mset(mod_info_curr->coef[V_C], 0, (cu_width >> 1) * (cu_height >> 1) * sizeof(s16));
    ret = decode_coef(ctx, core);
    com_assert_rv(ret == COM_OK, ret);
    return COM_OK;
}
#endif

int dec_decode_cu(DEC_CTX * ctx, DEC_CORE * core) // this function can be optimized to better match with the text
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16      mvd[MV_D];
    DEC_SBAC *sbac;
    COM_BSR  *bs;
    int      ret, cu_width, cu_height, inter_dir = 0;
    u8       bSkipMode;
    CPMV     affine_mvp[VER_NUM][MV_D];
    mod_info_curr->cu_mode = MODE_INVALID;
    mod_info_curr->affine_flag = 0;
#if AFFINE_UMVE
    mod_info_curr->affine_umve_flag = 0;
#endif
#if AWP_MVR
    mod_info_curr->awp_flag = 0;
    mod_info_curr->awp_mvr_flag0 = 0;
    mod_info_curr->awp_mvr_flag1 = 0;
#endif
    mod_info_curr->mvr_idx = 0;
#if IBC_ABVR
    mod_info_curr->bvr_idx = 0;
#endif
#if EXT_AMVR_HMVP
    mod_info_curr->mvp_from_hmvp_flag = 0;
#endif
    mod_info_curr->ipf_flag = 0;
#if USE_IBC
    mod_info_curr->ibc_flag = 0;
#endif
#if USE_SP
    mod_info_curr->sp_flag = 0;
#endif
#if IBC_BVP
    mod_info_curr->cbvp_idx = 0;
#endif
#if INTERPF
    mod_info_curr->inter_filter_flag = 0;
#endif
#if BGC
    mod_info_curr->bgc_flag = 0;
    mod_info_curr->bgc_idx = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if UMVE_ENH 
    mod_info_curr->umve_flag = 0;
#endif

    bs   = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    cu_width  = (1 << mod_info_curr->cu_width_log2);
    cu_height = (1 << mod_info_curr->cu_height_log2);
#if TB_SPLIT_EXT
    cu_nz_cln(mod_info_curr->num_nz);
    init_pb_part(mod_info_curr);
    init_tb_part(mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->tb_part, &mod_info_curr->tb_info);
#endif

#if FIMC
    if (ctx->info.sqh.fimc_enable_flag)
    {
        int cu_x = mod_info_curr->x_scu << 2;
        int cu_y = mod_info_curr->y_scu << 2;
        int rem_x_128 = cu_x % 128;
        int rem_y_64  = cu_y %  64;
        if (rem_x_128 == 0 && rem_y_64  == 0)
        {
            com_cntmpm_reset(&core->cntmpm_cands);
        }
    }
#endif

    if (ctx->info.pic_header.slice_type != SLICE_I)
    {
        if (ctx->cons_pred_mode != ONLY_INTRA)
        {
            bSkipMode = decode_skip_flag(bs, sbac, ctx);
        }
        else
        {
            bSkipMode = 0;
        }

        if (bSkipMode)
        {
            mod_info_curr->cu_mode = MODE_SKIP;
            mod_info_curr->umve_flag = 0;
#if AWP
            mod_info_curr->awp_flag = 0;

            int UmveAwpFlag = 0;
            if (ctx->info.sqh.umve_enable_flag || (ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE)
#if ETMVP
                || (ctx->info.sqh.etmvp_enable_flag && cu_width >= MIN_ETMVP_SIZE && cu_height >= MIN_ETMVP_SIZE)
#endif
                )
            {
                UmveAwpFlag = decode_umve_awp_flag(bs, sbac, ctx);
            }
#if ETMVP
            if (ctx->info.sqh.etmvp_enable_flag && UmveAwpFlag)
            {
                mod_info_curr->etmvp_flag = decode_etmvp_flag(bs, sbac, ctx);
            }
            if (mod_info_curr->etmvp_flag)
            {
                mod_info_curr->umve_flag = 0;
                mod_info_curr->awp_flag = 0;
            }
            else
            {
                mod_info_curr->etmvp_flag = 0;
#endif
                int AwpFlag = 0;
                if (UmveAwpFlag && ctx->info.sqh.umve_enable_flag && ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                    && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE)
                {
                    AwpFlag = decode_awp_flag(bs, sbac, ctx);
                    mod_info_curr->awp_flag = AwpFlag;
                    mod_info_curr->umve_flag = AwpFlag == 0 ? 1 : 0;
                }
                else
                {
                    mod_info_curr->awp_flag = UmveAwpFlag && ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                        && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE;
                    mod_info_curr->umve_flag = UmveAwpFlag && ctx->info.sqh.umve_enable_flag;
                }
#if ETMVP
            }
#endif
            if (mod_info_curr->awp_flag)
            {
#if AWP_MVR
                if (ctx->info.sqh.awp_mvr_enable_flag)
                {
                    mod_info_curr->awp_mvr_flag0 = decode_awp_mvr_flag(bs, sbac, ctx);
                    if (mod_info_curr->awp_mvr_flag0)
                    {
                        mod_info_curr->awp_mvr_idx0 = decode_awp_mvr_idx(bs, sbac, ctx);
                    }
                    mod_info_curr->awp_mvr_flag1 = decode_awp_mvr_flag(bs, sbac, ctx);
                    if (mod_info_curr->awp_mvr_flag1)
                    {
                        mod_info_curr->awp_mvr_idx1 = decode_awp_mvr_idx(bs, sbac, ctx);
                    }

                    if (!mod_info_curr->awp_mvr_flag0 && !mod_info_curr->awp_mvr_flag1)
                    {
                        mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
                    }
                    else if (mod_info_curr->awp_mvr_flag0 && mod_info_curr->awp_mvr_flag1)
                    {
                        if (mod_info_curr->awp_mvr_idx0 == mod_info_curr->awp_mvr_idx1)
                        {
                            mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
                        }
                        else
                        {
                            mod_info_curr->skip_idx = decode_awp_mode1(bs, sbac, ctx);
                        }
                    }
                    else
                    {
                        mod_info_curr->skip_idx = decode_awp_mode1(bs, sbac, ctx);
                    }
                }
                else
#endif
                mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
            }
            else if (mod_info_curr->umve_flag)
            {
#if UMVE_ENH 
                if (ctx->info.pic_header.umve_set_flag)
                {
                    mod_info_curr->umve_idx = decode_umve_idx_sec_set(bs, sbac);
                }
                else
#endif
                mod_info_curr->umve_idx = decode_umve_idx(bs, sbac, ctx);
            }
#if ETMVP
            else if (mod_info_curr->etmvp_flag)
            {
                mod_info_curr->skip_idx = decode_etmvp_idx(bs, sbac, ctx);
            }
#endif
#else
            if (ctx->info.sqh.umve_enable_flag)
            {
                mod_info_curr->umve_flag = decode_umve_flag(bs, sbac, ctx);
            }

            if (mod_info_curr->umve_flag)
            {
#if ETMVP
                mod_info_curr->etmvp_flag = decode_etmvp_flag(bs, sbac, ctx);
                if (mod_info_curr->etmvp_flag)
                {
                    mod_info_curr->umve_flag = 0;
                    mod_info_curr->skip_idx = decode_etmvp_idx(bs, sbac, ctx);
                }
                else
                {
#endif
#if UMVE_ENH 
                    if (ctx->info.pic_header.umve_set_flag)
                    {
                        mod_info_curr->umve_idx = decode_umve_idx_sec_set(bs, sbac);
                    }
                    else
#endif
                    mod_info_curr->umve_idx = decode_umve_idx(bs, sbac, ctx);
#if ETMVP
                }
#endif
            }
#endif
            else
            {
                mod_info_curr->affine_flag = decode_affine_flag(bs, sbac, ctx);
                if (mod_info_curr->affine_flag)
                {
#if AFFINE_UMVE
                    if (ctx->info.sqh.affine_umve_enable_flag) 
                    {
                        mod_info_curr->affine_umve_flag = decode_affine_umve_flag(bs, sbac, ctx);
                    }
#endif
                    mod_info_curr->skip_idx = decode_affine_mrg_idx(bs, sbac, ctx);
#if AFFINE_UMVE
                    if (mod_info_curr->affine_umve_flag)
                    {
                        mod_info_curr->affine_umve_idx[0] = decode_affine_umve_idx(bs, sbac, ctx);
                        mod_info_curr->affine_umve_idx[1] = decode_affine_umve_idx(bs, sbac, ctx);
                    }
#endif
                }
                else
                {
                    mod_info_curr->skip_idx = decode_skip_idx(bs, sbac, ctx->info.sqh.num_of_hmvp_cand, 
#if MVAP
                        ctx->info.sqh.num_of_mvap_cand,
#endif
                        ctx);
                }
            }
        }
        else
        {
            u8 direct_flag = 0;
            if (ctx->cons_pred_mode != ONLY_INTRA)
            {
                direct_flag = decode_direct_flag(bs, sbac, ctx);
            }
            if (direct_flag)
            {
                mod_info_curr->cu_mode = MODE_DIR;
                mod_info_curr->umve_flag = 0;
#if AWP
                mod_info_curr->awp_flag = 0;

                int UmveAwpFlag = 0;
                if (ctx->info.sqh.umve_enable_flag || (ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                    && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE)
#if ETMVP
                    || (ctx->info.sqh.etmvp_enable_flag && cu_width >= MIN_ETMVP_SIZE && cu_height >= MIN_ETMVP_SIZE)
#endif
                    )
                {
                    UmveAwpFlag = decode_umve_awp_flag(bs, sbac, ctx);
                }
#if ETMVP
                if (ctx->info.sqh.etmvp_enable_flag && UmveAwpFlag)
                {
                    mod_info_curr->etmvp_flag = decode_etmvp_flag(bs, sbac, ctx);
                }
                if (mod_info_curr->etmvp_flag)
                {
                    mod_info_curr->umve_flag = 0;
                    mod_info_curr->awp_flag = 0;
                }
                else
                {
                    mod_info_curr->etmvp_flag = 0;
#endif
                    int AwpFlag = 0;
                    if (UmveAwpFlag && ctx->info.sqh.umve_enable_flag && ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                        && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE)
                    {
                        AwpFlag = decode_awp_flag(bs, sbac, ctx);
                        mod_info_curr->awp_flag = AwpFlag;
                        mod_info_curr->umve_flag = AwpFlag == 0 ? 1 : 0;
                    }
                    else
                    {
                        mod_info_curr->awp_flag = UmveAwpFlag && ctx->info.sqh.awp_enable_flag && ctx->info.pic_header.slice_type == SLICE_B
                            && cu_width >= MIN_AWP_SIZE && cu_height >= MIN_AWP_SIZE && cu_width <= MAX_AWP_SIZE && cu_height <= MAX_AWP_SIZE;
                        mod_info_curr->umve_flag = UmveAwpFlag && ctx->info.sqh.umve_enable_flag;
                    }
#if ETMVP
                }
#endif

                if (mod_info_curr->awp_flag)
                {
#if AWP_MVR
                    if (ctx->info.sqh.awp_mvr_enable_flag)
                    {
                        mod_info_curr->awp_mvr_flag0 = decode_awp_mvr_flag(bs, sbac, ctx);
                        if (mod_info_curr->awp_mvr_flag0)
                        {
                            mod_info_curr->awp_mvr_idx0 = decode_awp_mvr_idx(bs, sbac, ctx);
                        }
                        mod_info_curr->awp_mvr_flag1 = decode_awp_mvr_flag(bs, sbac, ctx);
                        if (mod_info_curr->awp_mvr_flag1)
                        {
                            mod_info_curr->awp_mvr_idx1 = decode_awp_mvr_idx(bs, sbac, ctx);
                        }

                        if (!mod_info_curr->awp_mvr_flag0 && !mod_info_curr->awp_mvr_flag1)
                        {
                            mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
                        }
                        else if (mod_info_curr->awp_mvr_flag0 && mod_info_curr->awp_mvr_flag1)
                        {
                            if (mod_info_curr->awp_mvr_idx0 == mod_info_curr->awp_mvr_idx1)
                            {
                                mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
                            }
                            else
                            {
                                mod_info_curr->skip_idx = decode_awp_mode1(bs, sbac, ctx);
                            }
                        }
                        else
                        {
                            mod_info_curr->skip_idx = decode_awp_mode1(bs, sbac, ctx);
                        }
                    }
                    else
#endif
                    mod_info_curr->skip_idx = decode_awp_mode(bs, sbac, ctx);
                }
                else if (mod_info_curr->umve_flag)
                {
#if UMVE_ENH
                    if (ctx->info.sqh.umve_enh_enable_flag && ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                        && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64))
                    {
                        mod_info_curr->inter_filter_flag = decode_inter_filter_flag(bs, sbac, ctx);
                    }
                    if (ctx->info.pic_header.umve_set_flag)
                    {
                        mod_info_curr->umve_idx = decode_umve_idx_sec_set(bs, sbac);
                    }
                    else
#endif
                    mod_info_curr->umve_idx = decode_umve_idx(bs, sbac, ctx);
                }
#if ETMVP
                else if (mod_info_curr->etmvp_flag)
                {
                    mod_info_curr->skip_idx = decode_etmvp_idx(bs, sbac, ctx);
                }
#endif
#else
                if (ctx->info.sqh.umve_enable_flag)
                {
                    mod_info_curr->umve_flag = decode_umve_flag(bs, sbac, ctx);
                }

                if (mod_info_curr->umve_flag)
                {
#if ETMVP
                    mod_info_curr->etmvp_flag = decode_etmvp_flag(bs, sbac, ctx);
                    if (mod_info_curr->etmvp_flag)
                    {
                        mod_info_curr->umve_flag = 0;
                        mod_info_curr->skip_idx = decode_etmvp_idx(bs, sbac, ctx);
                    }
                    else
                    {
#endif
#if UMVE_ENH
                        if (ctx->info.sqh.umve_enh_enable_flag && ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                            && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64))
                        {
                            mod_info_curr->inter_filter_flag = decode_inter_filter_flag(bs, sbac, ctx);
                        }
                        if (ctx->info.pic_header.umve_set_flag)
                        {
                            mod_info_curr->umve_idx = decode_umve_idx_sec_set(bs, sbac);
                        }
                        else
#endif
                        mod_info_curr->umve_idx = decode_umve_idx(bs, sbac, ctx);
#if ETMVP
                    }
#endif
                }
#endif
                else
                {
                    mod_info_curr->affine_flag = decode_affine_flag(bs, sbac, ctx);
                    if (mod_info_curr->affine_flag)
                    {
#if AFFINE_UMVE
                        if (ctx->info.sqh.affine_umve_enable_flag) 
                        {
                            mod_info_curr->affine_umve_flag = decode_affine_umve_flag(bs, sbac, ctx);
                        }
#endif
                        mod_info_curr->skip_idx = decode_affine_mrg_idx(bs, sbac, ctx);
#if AFFINE_UMVE
                        if (mod_info_curr->affine_umve_flag)
                        {
                            mod_info_curr->affine_umve_idx[0] = decode_affine_umve_idx(bs, sbac, ctx);
                            mod_info_curr->affine_umve_idx[1] = decode_affine_umve_idx(bs, sbac, ctx);
                        }
#endif
                    }
                    else
                    {
#if INTERPF
                        if( ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                            && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64) )
                        {
                            mod_info_curr->inter_filter_flag = decode_inter_filter_flag( bs, sbac, ctx );
                        }
#endif
                    
                        mod_info_curr->skip_idx = decode_skip_idx(bs, sbac, ctx->info.sqh.num_of_hmvp_cand, 
#if MVAP
                            ctx->info.sqh.num_of_mvap_cand,
#endif
                            ctx);
                    }
                }
            }
            else
            {
                if (ctx->cons_pred_mode == NO_MODE_CONS)
                {
                    mod_info_curr->cu_mode = decode_pred_mode(bs, sbac, ctx);
                }
                else
                {
                    mod_info_curr->cu_mode = ctx->cons_pred_mode == ONLY_INTRA ? MODE_INTRA : MODE_INTER;
                    //Note: when cu_mode is MODE_INTRA, it may be modified in below lines to MODE_IBC when IBC is allowed
                }

#if USE_IBC
                if (ctx->info.sqh.ibc_flag && ctx->info.pic_header.ibc_flag
                  && (ctx->cons_pred_mode == NO_MODE_CONS || ctx->cons_pred_mode == ONLY_INTRA)
                  && mod_info_curr->cu_mode != MODE_INTER
#if !USE_SP
                  && mod_info_curr->cu_width_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2 && mod_info_curr->cu_height_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2
#endif
                    )
                { 
#if !USE_SP
                    if (decode_ibc_flag(bs, sbac, ctx)) /* is ibc mode? */
#else
                    int sp_or_ibc_mode = 0;
                    sp_or_ibc_mode = decode_sp_or_ibc_cu_flag(bs, sbac, cu_width, cu_height, mod_info_curr, ctx);
                    if (sp_or_ibc_mode == TRUE)
#endif
                    {
#if USE_SP
                        if (mod_info_curr->ibc_flag)
                        {
#endif
                            mod_info_curr->cu_mode = MODE_IBC;
                            mod_info_curr->ibc_flag = 1;
#if IBC_BVP
                            if (ctx->info.sqh.num_of_hbvp_cand > 0)
                            {
                                mod_info_curr->cbvp_idx = decode_ibc_bvp_flag(bs, sbac, ctx);
                            }
#endif
#if IBC_ABVR
                            if (ctx->info.sqh.abvr_enable_flag)
                            {
                                mod_info_curr->bvr_idx = (u8)decode_bvr_idx(bs, sbac);
                            }
#endif
#if USE_SP
                        }
                        else if (mod_info_curr->sp_flag)
                        {
                            mod_info_curr->cu_mode = MODE_IBC;
                        }
#endif
                    }
                    else if (ctx->cons_pred_mode == ONLY_INTRA)
                    {
                        mod_info_curr->cu_mode = MODE_INTRA;
                    }
                }

                assert( mod_info_curr->cu_mode != MODE_INVALID );
                if (mod_info_curr->cu_mode != MODE_IBC)
                {
#endif

                    mod_info_curr->affine_flag = 0;
                    if (mod_info_curr->cu_mode != MODE_INTRA)
                    {
                        assert(mod_info_curr->cu_mode == MODE_INTER);
                        mod_info_curr->affine_flag = decode_affine_flag(bs, sbac, ctx);
                    }

                    if (mod_info_curr->cu_mode != MODE_INTRA && ctx->info.sqh.amvr_enable_flag)
                    {
#if EXT_AMVR_HMVP
                        if (ctx->info.sqh.emvr_enable_flag && !mod_info_curr->affine_flag) // also imply ctx->info.sqh.num_of_hmvp_cand is not zero
                        {
                            mod_info_curr->mvp_from_hmvp_flag = decode_extend_amvr_flag(bs, sbac);
                        }
#endif
                        mod_info_curr->mvr_idx = (u8)decode_mvr_idx(bs, sbac, mod_info_curr->affine_flag);
                    }
#if USE_IBC
                }
#endif
            }
        }
    }
#if USE_IBC
    else if (ctx->info.pic_header.ibc_flag)
    {
        mod_info_curr->cu_mode = MODE_INTRA;
        {
#if !USE_SP
            if (mod_info_curr->cu_width_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2 && mod_info_curr->cu_height_log2 <= IBC_BITSTREAM_FLAG_RESTRIC_LOG2)
#else
            int sp_or_ibc_mode = 0;
            sp_or_ibc_mode = decode_sp_or_ibc_cu_flag(bs, sbac, cu_width, cu_height, mod_info_curr, ctx);
            if (sp_or_ibc_mode == TRUE)
#endif
            { 
#if !USE_SP
                if (decode_ibc_flag(bs, sbac, ctx))
#else
                if (mod_info_curr->ibc_flag)
#endif
                {
                    mod_info_curr->cu_mode = MODE_IBC;
                    mod_info_curr->ibc_flag = 1;
#if IBC_BVP
                    if (ctx->info.sqh.num_of_hbvp_cand > 0)
                    {
                        mod_info_curr->cbvp_idx = decode_ibc_bvp_flag(bs, sbac, ctx);
                    }
#endif
#if IBC_ABVR
                    if (ctx->info.sqh.abvr_enable_flag)
                    {
                        mod_info_curr->bvr_idx = (u8)decode_bvr_idx(bs, sbac);
                    }
#endif
                }
#if USE_SP
                else if (mod_info_curr->sp_flag)
                {
                    assert(mod_info_curr->ibc_flag == 0);
                    mod_info_curr->cu_mode = MODE_IBC;
                }
#endif
            }
        }
    }
#endif
    else /* SLICE_I */
    {
        mod_info_curr->cu_mode = MODE_INTRA;
    }

    /* parse prediction info */
    if (mod_info_curr->cu_mode == MODE_SKIP || mod_info_curr->cu_mode == MODE_DIR)
    {
        dec_derive_skip_direct_info(ctx, core);
        if (mod_info_curr->cu_mode == MODE_SKIP)
        {
            assert(mod_info_curr->pb_part == SIZE_2Nx2N);
        }
    }
#if USE_IBC
    else if (mod_info_curr->cu_mode == MODE_IBC)
    {
#if USE_SP
        if (mod_info_curr->ibc_flag)
        {
#endif
            decode_mvd(bs, sbac, mvd);
#if IBC_BVP
            s16 mvp[MV_D];
            dec_derive_ibc_bvp_info(ctx, core, mvp);
#if IBC_ABVR
            mvd[MV_X] = mvd[MV_X] << (mod_info_curr->bvr_idx + 2);
            mvd[MV_Y] = mvd[MV_Y] << (mod_info_curr->bvr_idx + 2);
            mvp[MV_X] = (mvp[MV_X] >> (mod_info_curr->bvr_idx + 2)) << (mod_info_curr->bvr_idx + 2);
            mvp[MV_Y] = (mvp[MV_Y] >> (mod_info_curr->bvr_idx + 2)) << (mod_info_curr->bvr_idx + 2);
#endif
            mod_info_curr->mv[REFP_0][MV_X] = mvd[MV_X] + mvp[MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = mvd[MV_Y] + mvp[MV_Y];
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mod_info_curr->mv[REFP_0][MV_X]);
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mod_info_curr->mv[REFP_0][MV_Y]);
    
#else
#if IBC_ABVR
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_X] << (mod_info_curr->bvr_idx + 2));
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_Y] << (mod_info_curr->bvr_idx + 2));
#else
            mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_X] << 2);
            mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvd[MV_Y] << 2);
#endif
#endif
            mod_info_curr->refi[REFP_0] = -1;
            mod_info_curr->refi[REFP_1] = -1;
#if USE_SP
        }
        else if (mod_info_curr->sp_flag)
        {
            dec_eco_sp(ctx, core, bs, sbac, mod_info_curr, ctx->tree_status);
            mod_info_curr->refi[REFP_0] = -1;
            mod_info_curr->refi[REFP_1] = -1;
            mod_info_curr->mv[REFP_0][MV_X] = 0;
            mod_info_curr->mv[REFP_0][MV_Y] = 0;
        }
#endif
    }
#endif
    else if (mod_info_curr->cu_mode == MODE_INTER)
    {
        if (ctx->info.pic_header.slice_type == SLICE_P)
        {
            inter_dir = PRED_L0;
        }
        else /* if(ctx->info.pic_header.slice_type == SLICE_B) */
        {
            inter_dir = decode_inter_dir(bs, sbac, mod_info_curr->pb_part, ctx);
        }

        if (mod_info_curr->affine_flag) // affine inter motion vector
        {
            dec_eco_affine_motion_derive(ctx, core, bs, sbac, inter_dir, cu_width, cu_height, affine_mvp, mvd);
        }
        else
        {
#if SMVD
            if (ctx->info.sqh.smvd_enable_flag && inter_dir == PRED_BI
                && (ctx->ptr - ctx->refp[0][REFP_0].ptr == ctx->refp[0][REFP_1].ptr - ctx->ptr)
                && !mod_info_curr->mvp_from_hmvp_flag
               )
            {
                mod_info_curr->smvd_flag = (u8)decode_smvd_flag(bs, sbac, ctx);
            }
            else
            {
                mod_info_curr->smvd_flag = 0;
            }
#endif

            /* forward */
            if (inter_dir == PRED_L0 || inter_dir == PRED_BI)
            {
                s16 mvp[MV_D];
#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mod_info_curr->refi[REFP_0] = 0;
                }
                else
                {
#endif
                    mod_info_curr->refi[REFP_0] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_0]);
#if SMVD
                }
#endif
                com_derive_mvp(ctx->info, mod_info_curr, ctx->ptr, REFP_0, mod_info_curr->refi[REFP_0], core->cnt_hmvp_cands,
                    core->motion_cands, ctx->map, ctx->refp, mod_info_curr->mvr_idx, mvp);

                decode_mvd(bs, sbac, mvd);
                s32 mv_x = (s32)mvp[MV_X] + ((s32)mvd[MV_X] << mod_info_curr->mvr_idx);
                s32 mv_y = (s32)mvp[MV_Y] + ((s32)mvd[MV_Y] << mod_info_curr->mvr_idx);
                mod_info_curr->mv[REFP_0][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
                mod_info_curr->mv[REFP_0][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);

            }
            else
            {
                mod_info_curr->refi[REFP_0] = REFI_INVALID;
                mod_info_curr->mv[REFP_0][MV_X] = 0;
                mod_info_curr->mv[REFP_0][MV_Y] = 0;
            }

            /* backward */
            if (inter_dir == PRED_L1 || inter_dir == PRED_BI)
            {
                s16 mvp[MV_D];
#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mod_info_curr->refi[REFP_1] = 0;
                }
                else
                {
#endif
                    mod_info_curr->refi[REFP_1] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_1]);
#if SMVD
                }
#endif
                com_derive_mvp(ctx->info, mod_info_curr, ctx->ptr, REFP_1, mod_info_curr->refi[REFP_1], core->cnt_hmvp_cands,
                    core->motion_cands, ctx->map, ctx->refp, mod_info_curr->mvr_idx, mvp);

#if SMVD
                if (mod_info_curr->smvd_flag == 1)
                {
                    mvd[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, -mvd[MV_X]);
                    mvd[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, -mvd[MV_Y]);
                }
                else
                {
#endif
                    decode_mvd(bs, sbac, mvd);
#if SMVD
                }
#endif
                s32 mv_x = (s32)mvp[MV_X] + ((s32)mvd[MV_X] << mod_info_curr->mvr_idx);
                s32 mv_y = (s32)mvp[MV_Y] + ((s32)mvd[MV_Y] << mod_info_curr->mvr_idx);

                mod_info_curr->mv[REFP_1][MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
                mod_info_curr->mv[REFP_1][MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
            }
            else
            {
                mod_info_curr->refi[REFP_1] = REFI_INVALID;
                mod_info_curr->mv[REFP_1][MV_X] = 0;
                mod_info_curr->mv[REFP_1][MV_Y] = 0;
            }
        }
#if BGC
        if (ctx->info.sqh.bgc_enable_flag && (ctx->info.pic_header.slice_type == SLICE_B) && REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && REFI_IS_VALID(mod_info_curr->refi[REFP_1]) && (cu_width * cu_height >= 256))
        {
            int poc0 = ctx->refp[mod_info_curr->refi[REFP_0]][REFP_0].pic->ptr;
            int poc1 = ctx->refp[mod_info_curr->refi[REFP_1]][REFP_1].pic->ptr;
            if ((poc0 - ctx->ptr) * (ctx->ptr - poc1) > 0)
            {
                mod_info_curr->bgc_flag = decode_bgc_flag(bs, sbac, ctx);
                if (mod_info_curr->bgc_flag)
                {
                    mod_info_curr->bgc_idx = decode_bgc_idx(bs, sbac, ctx);
                }
            }
        }
#endif
    }
    else if (mod_info_curr->cu_mode == MODE_INTRA)
    {
#if DT_SYNTAX
        mod_info_curr->pb_part = decode_part_size(bs, sbac, ctx, cu_width, cu_height, mod_info_curr->cu_mode);
        get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
        assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
        for (int part_idx = 0; part_idx < core->mod_info_curr.pb_info.num_sub_part; part_idx++)
        {
            int pb_x = mod_info_curr->pb_info.sub_x[part_idx];
            int pb_y = mod_info_curr->pb_info.sub_y[part_idx];
            int pb_width  = mod_info_curr->pb_info.sub_w[part_idx];
            int pb_height = mod_info_curr->pb_info.sub_h[part_idx];
            int pb_scup   = mod_info_curr->pb_info.sub_scup[part_idx];
#endif
#if FIMC
            if (ctx->info.sqh.fimc_enable_flag && ctx->info.pic_header.fimc_pic_flag)
            {
                com_get_cntmpm( PEL2SCU( pb_x ), PEL2SCU( pb_y ), ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[part_idx], &core->cntmpm_cands );
            }
            else
            {
#endif
                com_get_mpm( PEL2SCU( pb_x ), PEL2SCU( pb_y ), ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[part_idx] );
#if FIMC
            }
#endif
            mod_info_curr->ipm[part_idx][0] = (s8)decode_intra_dir(bs, sbac,
#if EIPM
                ctx->info.sqh.eipm_enable_flag,
#endif
                mod_info_curr->mpm[part_idx]);
#if FIMC
            if (ctx->info.sqh.fimc_enable_flag)
            {
                assert(core->mod_info_curr.cu_mode == MODE_INTRA && !core->mod_info_curr.ibc_flag);
                com_cntmpm_update(&core->cntmpm_cands, mod_info_curr->ipm[part_idx][0]);
            }
#endif
            SET_REFI(mod_info_curr->refi, REFI_INVALID, REFI_INVALID);
            mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->mv[REFP_0][MV_Y] = 0;
            mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->mv[REFP_1][MV_Y] = 0;
            update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_width, pb_height, ctx->info.pic_width_in_scu, mod_info_curr->ipm[part_idx][0]);
#if DT_SYNTAX
        }
#endif
        if (ctx->tree_status != TREE_L)
        {
#if TSCPM
            mod_info_curr->ipm[PB0][1] = (s8)decode_intra_dir_c(bs, sbac, mod_info_curr->ipm[PB0][0], ctx->info.sqh.tscpm_enable_flag
#if ENHANCE_TSPCM
                , ctx->info.sqh.enhance_tscpm_enable_flag
#endif
#if PMC
                , ctx->info.sqh.pmc_enable_flag
#endif
            );
#else
            mod_info_curr->ipm[PB0][1] = (s8)decode_intra_dir_c(bs, sbac, mod_info_curr->ipm[PB0][0]
#if PMC
                , ctx->info.sqh.pmc_enable_flag
#endif
            );
#endif
        }

#if IPCM
        if (!((ctx->tree_status == TREE_C && mod_info_curr->ipm[PB0][0] == IPD_IPCM && mod_info_curr->ipm[PB0][1] == IPD_DM_C)
                || (ctx->tree_status != TREE_C && mod_info_curr->ipm[PB0][0] == IPD_IPCM)))
        {
#endif

#if DT_INTRA_BOUNDARY_FILTER_OFF
            if (ctx->info.sqh.ipf_enable_flag && (cu_width < MAX_CU_SIZE) && (cu_height < MAX_CU_SIZE) && core->mod_info_curr.pb_part == SIZE_2Nx2N)
#else
            if (ctx->info.sqh.ipf_enable_flag && (cu_width < MAX_CU_SIZE) && (cu_height < MAX_CU_SIZE))
#endif
            {
                mod_info_curr->ipf_flag = decode_ipf_flag(bs, sbac);
            }
#if IPCM
        }
#endif

        SET_REFI(mod_info_curr->refi, REFI_INVALID, REFI_INVALID);
        mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->mv[REFP_0][MV_Y] = 0;
        mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->mv[REFP_1][MV_Y] = 0;
    }
    else
    {
        com_assert_rv(0, COM_ERR_MALFORMED_BITSTREAM);
    }

    /* parse coefficients */
    if (mod_info_curr->cu_mode != MODE_SKIP
#if USE_SP
        &&mod_info_curr->sp_flag == FALSE 
#endif
        )
    {
        /* clear coefficient buffer */
        com_mset(mod_info_curr->coef[Y_C], 0, cu_width * cu_height * sizeof(s16));
        com_mset(mod_info_curr->coef[U_C], 0, (cu_width >> 1) * (cu_height >> 1) * sizeof(s16));
        com_mset(mod_info_curr->coef[V_C], 0, (cu_width >> 1) * (cu_height >> 1) * sizeof(s16));
        ret = decode_coef(ctx, core);
        com_assert_rv(ret == COM_OK, ret);
    }

#if DMVR
    mod_info_curr->dmvr_enable = 0;
    if( mod_info_curr->cu_mode == MODE_SKIP || mod_info_curr->cu_mode == MODE_DIR )
    {
        mod_info_curr->dmvr_enable = 1;
    }
    if( mod_info_curr->affine_flag || mod_info_curr->umve_flag )
    {
        mod_info_curr->dmvr_enable = 0;
    }
#endif

    return COM_OK;
}

int dec_eco_cnkh(COM_BSR * bs, COM_CNKH * cnkh)
{
    cnkh->ver = com_bsr_read(bs, 3);
    cnkh->ctype = com_bsr_read(bs, 4);
    cnkh->broken = com_bsr_read1(bs);
    return COM_OK;
}
#if HLS_RPL
#if LIBVC_ON
int dec_eco_rlp(COM_BSR * bs, COM_RPL * rpl, COM_SQH * sqh)
#else
int dec_eco_rlp(COM_BSR * bs, COM_RPL * rpl)
#endif
{
#if LIBVC_ON
    int ddoi_base = 0;

    rpl->reference_to_library_enable_flag = 0;
    if (sqh->library_picture_enable_flag)
    {
        rpl->reference_to_library_enable_flag = com_bsr_read1(bs);
    }
#endif

    rpl->ref_pic_num = (u32)com_bsr_read_ue(bs);
    //note, here we store DOI of each ref pic instead of delta DOI
    if (rpl->ref_pic_num > 0)
    {
#if LIBVC_ON
        rpl->library_index_flag[0] = 0;
        if (sqh->library_picture_enable_flag && rpl->reference_to_library_enable_flag)
        {
            rpl->library_index_flag[0] = com_bsr_read1(bs);
        }
        if (sqh->library_picture_enable_flag && rpl->library_index_flag[0])
        {
            rpl->ref_pics_ddoi[0] = (u32)com_bsr_read_ue(bs);
        }
        else
#endif
        {
            rpl->ref_pics_ddoi[0] = (u32)com_bsr_read_ue(bs);
            if (rpl->ref_pics_ddoi[0] != 0) rpl->ref_pics_ddoi[0] *= 1 - ((u32)com_bsr_read1(bs) << 1);
#if LIBVC_ON
            ddoi_base = rpl->ref_pics_ddoi[0];
#endif
        }

    }
    for (int i = 1; i < rpl->ref_pic_num; ++i)
    {
#if LIBVC_ON
        rpl->library_index_flag[i] = 0;
        if (sqh->library_picture_enable_flag && rpl->reference_to_library_enable_flag)
        {
            rpl->library_index_flag[i] = com_bsr_read1(bs);
        }
        if (sqh->library_picture_enable_flag && rpl->library_index_flag[i])
        {
            rpl->ref_pics_ddoi[i] = (u32)com_bsr_read_ue(bs);
        }
        else
#endif
        {
            int deltaRefPic = (u32)com_bsr_read_ue(bs);
            if (deltaRefPic != 0) deltaRefPic *= 1 - ((u32)com_bsr_read1(bs) << 1);
#if LIBVC_ON
            rpl->ref_pics_ddoi[i] = ddoi_base + deltaRefPic;
            ddoi_base = rpl->ref_pics_ddoi[i];
#else
            rpl->ref_pics_ddoi[i] = rpl->ref_pics_ddoi[i - 1] + deltaRefPic;
#endif
        }

    }
    return COM_OK;
}
#endif

void read_wq_matrix(COM_BSR *bs, u8 *m4x4, u8 *m8x8)
{
    int i;
    for (i = 0; i < 16; i++)
    {
        m4x4[i] = com_bsr_read_ue(bs);
    }
    for (i = 0; i < 64; i++)
    {
        m8x8[i] = com_bsr_read_ue(bs);
    }
}

int dec_eco_sqh(COM_BSR * bs, COM_SQH * sqh)
{
    //video_sequence_start_code
    unsigned int ret = com_bsr_read(bs, 24);
    assert(ret == 1);
    unsigned int start_code;
    start_code = com_bsr_read(bs, 8);
    assert(start_code==0xB0);

    sqh->profile_id = (u8)com_bsr_read(bs, 8);
#if PHASE_2_PROFILE
    if (sqh->profile_id != 0x22 && sqh->profile_id != 0x20
     && sqh->profile_id != 0x32 && sqh->profile_id != 0x30
        )
    {
        printf("unknow profile id: 0x%x\n", sqh->profile_id);
        assert(0);
    }
#endif
    sqh->level_id   = (u8)com_bsr_read(bs, 8);
    sqh->progressive_sequence = (u8)com_bsr_read1(bs);
    assert(sqh->progressive_sequence == 1);
    sqh->field_coded_sequence = (u8)com_bsr_read1(bs);
#if LIBVC_ON
    sqh->library_stream_flag = (u8)com_bsr_read1(bs);
    sqh->library_picture_enable_flag = 0;
    sqh->duplicate_sequence_header_flag = 0;
    if (!sqh->library_stream_flag)
    {
        sqh->library_picture_enable_flag = (u8)com_bsr_read1(bs);
        if (sqh->library_picture_enable_flag)
        {
            sqh->duplicate_sequence_header_flag = (u8)com_bsr_read1(bs);
        }
    }
#endif
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->horizontal_size  = com_bsr_read(bs, 14);                  //horizontal_size
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->vertical_size = com_bsr_read(bs, 14);                     //vertical_size
    sqh->chroma_format    = (u8)com_bsr_read(bs, 2);
    sqh->sample_precision = (u8)com_bsr_read(bs, 3);

#if PHASE_2_PROFILE
    if (sqh->profile_id == 0x22 || sqh->profile_id == 0x32)
#else
    if (sqh->profile_id == 0x22)
#endif
    {
        sqh->encoding_precision = (u8)com_bsr_read(bs, 3);
        assert(sqh->encoding_precision == 2);
    }
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->aspect_ratio = (u8)com_bsr_read(bs, 4);

    sqh->frame_rate_code = (u8)com_bsr_read(bs, 4);
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->bit_rate_lower  = com_bsr_read(bs, 18);
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->bit_rate_upper = com_bsr_read(bs, 12);
    sqh->low_delay      = (u8)com_bsr_read1(bs);

    sqh->temporal_id_enable_flag = (u8)com_bsr_read1(bs);
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->bbv_buffer_size = com_bsr_read(bs, 18);
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->max_dpb_size = com_bsr_read(bs, 4) + 1;

#if HLS_RPL
    sqh->rpl1_index_exist_flag  = (u32)com_bsr_read1(bs);
    sqh->rpl1_same_as_rpl0_flag = (u32)com_bsr_read1(bs);
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    sqh->rpls_l0_num            = (u32)com_bsr_read_ue(bs);
    for (int i = 0; i < sqh->rpls_l0_num; ++i)
    {
#if LIBVC_ON
        dec_eco_rlp(bs, &sqh->rpls_l0[i], sqh);
#else
        dec_eco_rlp(bs, &sqh->rpls_l0[i]);
#endif
    }

    if (!sqh->rpl1_same_as_rpl0_flag)
    {
        sqh->rpls_l1_num = (u32)com_bsr_read_ue(bs);
        for (int i = 0; i < sqh->rpls_l1_num; ++i)
        {
#if LIBVC_ON
            dec_eco_rlp(bs, &sqh->rpls_l1[i], sqh);
#else
            dec_eco_rlp(bs, &sqh->rpls_l1[i]);
#endif
        }
    }
    else
    {
        //Basically copy everything from sqh->rpls_l0 to sqh->rpls_l1
        //MX: LIBVC is not harmonization yet.
        sqh->rpls_l1_num = sqh->rpls_l0_num;
        for (int i = 0; i < sqh->rpls_l1_num; i++)
        {
#if LIBVC_ON
            sqh->rpls_l1[i].reference_to_library_enable_flag = sqh->rpls_l0[i].reference_to_library_enable_flag;
#endif
            sqh->rpls_l1[i].ref_pic_num = sqh->rpls_l0[i].ref_pic_num;
            for (int j = 0; j < sqh->rpls_l1[i].ref_pic_num; j++)
            {
#if LIBVC_ON
                sqh->rpls_l1[i].library_index_flag[j] = sqh->rpls_l0[i].library_index_flag[j];
#endif
                sqh->rpls_l1[i].ref_pics_ddoi[j] = sqh->rpls_l0[i].ref_pics_ddoi[j];
            }
        }
    }
    sqh->num_ref_default_active_minus1[0] = (u32)com_bsr_read_ue(bs);
    sqh->num_ref_default_active_minus1[1] = (u32)com_bsr_read_ue(bs);
#endif

    sqh->log2_max_cu_width_height = (u8)(com_bsr_read(bs, 3) + 2); //log2_lcu_size_minus2
    sqh->min_cu_size    = (u8)(1 << (com_bsr_read(bs, 2) + 2));    //log2_min_cu_size_minus2
    sqh->max_part_ratio = (u8)(1 << (com_bsr_read(bs, 2) + 2));    //log2_max_part_ratio_minus2
    sqh->max_split_times = (u8)(com_bsr_read(bs,    3) + 6 );      //max_split_times_minus6
    sqh->min_qt_size  = (u8)(1 << (com_bsr_read(bs, 3) + 2));      //log2_min_qt_size_minus2
    sqh->max_bt_size  = (u8)(1 << (com_bsr_read(bs, 3) + 2));      //log2_max_bt_size_minus2
    sqh->max_eqt_size = (u8)(1 << (com_bsr_read(bs, 2) + 3));      //log2_max_eqt_size_minus3
    assert(com_bsr_read1(bs) == 1);                                //marker_bit
    //verify parameters allowed in Profile
    assert(sqh->log2_max_cu_width_height >= 5 && sqh->log2_max_cu_width_height <= 7);
    assert(sqh->min_cu_size == 4);
    assert(sqh->max_part_ratio == 8);
    assert(sqh->max_split_times == 6);
    assert(sqh->min_qt_size == 4 || sqh->min_qt_size == 8 || sqh->min_qt_size == 16 || sqh->min_qt_size == 32 || sqh->min_qt_size == 64 || sqh->min_qt_size == 128);
    assert(sqh->max_bt_size == 64 || sqh->max_bt_size == 128);
    assert(sqh->max_eqt_size == 8 || sqh->max_eqt_size == 16 || sqh->max_eqt_size == 32 || sqh->max_eqt_size == 64);

    sqh->wq_enable = (u8)com_bsr_read1(bs);                        //weight_quant_enable_flag
    if (sqh->wq_enable)
    {
        sqh->seq_wq_mode = com_bsr_read1(bs);                      //load_seq_weight_quant_data_flag
        if (sqh->seq_wq_mode)
        {
            read_wq_matrix(bs, sqh->wq_4x4_matrix, sqh->wq_8x8_matrix);  //weight_quant_matrix( )
        }
        else
        {
            memcpy(sqh->wq_4x4_matrix, tab_WqMDefault4x4, sizeof(tab_WqMDefault4x4));
            memcpy(sqh->wq_8x8_matrix, tab_WqMDefault8x8, sizeof(tab_WqMDefault8x8));
        }
    }
    sqh->secondary_transform_enable_flag = (u8)com_bsr_read1(bs);
    sqh->sample_adaptive_offset_enable_flag = (u8)com_bsr_read1(bs);
    sqh->adaptive_leveling_filter_enable_flag = (u8)com_bsr_read1(bs);
    sqh->affine_enable_flag = (u8)com_bsr_read1(bs);
#if SMVD
    sqh->smvd_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if IPCM
    sqh->ipcm_enable_flag = com_bsr_read1(bs);
    assert(sqh->sample_precision == 1 || sqh->sample_precision == 2);
#endif
    sqh->amvr_enable_flag = (u8)com_bsr_read1(bs);
    sqh->num_of_hmvp_cand = (u8)com_bsr_read(bs, 4);
    sqh->umve_enable_flag = (u8)com_bsr_read1(bs);
#if EXT_AMVR_HMVP
    if (sqh->amvr_enable_flag && sqh->num_of_hmvp_cand)
    {
        sqh->emvr_enable_flag = (u8)com_bsr_read1(bs);
    }
    else
    {
        sqh->emvr_enable_flag = 0;
    }
#endif
    sqh->ipf_enable_flag = (u8)com_bsr_read1(bs); //ipf_enable_flag
#if TSCPM
    sqh->tscpm_enable_flag = (u8)com_bsr_read1(bs);
#endif
    assert(com_bsr_read1(bs) == 1);              //marker_bit
#if DT_PARTITION
    sqh->dt_intra_enable_flag = (u8)com_bsr_read1(bs);
    if (sqh->dt_intra_enable_flag)
    {
        u32 log2_max_dt_size_minus4 = com_bsr_read(bs, 2);
        assert(log2_max_dt_size_minus4 <= 2);
        sqh->max_dt_size = (u8)(1 << (log2_max_dt_size_minus4 + 4)); //log2_max_dt_size_minus4
    }
#endif
    sqh->position_based_transform_enable_flag = (u8)com_bsr_read1(bs); //pbt_enable_flag

#if PHASE_2_PROFILE
    if (sqh->profile_id == 0x30 || sqh->profile_id == 0x32)
    {
#endif
        // begin of phase-2 sqh
#if ESAO
        sqh->esao_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if UMVE_ENH
        if (sqh->umve_enable_flag)
        {
            sqh->umve_enh_enable_flag = (u8)com_bsr_read1(bs);
        }
#endif
#if AWP
        sqh->awp_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if AWP_MVR
        if (sqh->awp_enable_flag)
        {
            sqh->awp_mvr_enable_flag = (u8)com_bsr_read1(bs);
        }
        else
        {
            sqh->awp_mvr_enable_flag = 0;
        }
#endif
#if BIO
        sqh->bio_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if BGC
        sqh->bgc_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if DMVR
        sqh->dmvr_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if INTERPF
        sqh->interpf_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if IBC_ABVR
        sqh->abvr_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if USE_SP
        sqh->sp_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if IBC_BVP
        sqh->num_of_hbvp_cand = (u8)com_bsr_read(bs, 4);
#endif
#if AFFINE_UMVE
        if (sqh->affine_enable_flag && sqh->umve_enable_flag)
        {
            sqh->affine_umve_enable_flag = (u8)com_bsr_read1(bs);
        }
        else
        {
            sqh->affine_umve_enable_flag = 0;
        }
#endif
#if MIPF
        sqh->mipf_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if TSCPM && ENHANCE_TSPCM
        if (sqh->tscpm_enable_flag)
        {
            sqh->enhance_tscpm_enable_flag = (u8)com_bsr_read1(bs);
        }
        else
        {
            sqh->enhance_tscpm_enable_flag = 0;
        }
#endif
#if IPF_CHROMA
        if (sqh->ipf_enable_flag)
        {
            sqh->chroma_ipf_enable_flag = (u8)com_bsr_read1(bs);
        }
        else
        {
            sqh->chroma_ipf_enable_flag = 0;
        }
#endif
#if PMC
        sqh->pmc_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if FIMC
        sqh->fimc_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if USE_IBC
        sqh->ibc_flag = (u8)com_bsr_read1(bs);
#endif
#if SBT
        sqh->sbt_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if IST
        sqh->ist_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if ISTS
        sqh->ists_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if EST
        if (sqh->secondary_transform_enable_flag)
        {
            sqh->est_enable_flag = (u8)com_bsr_read1(bs);
        }
#endif
#if SRCC
        sqh->srcc_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if CABAC_MULTI_PROB
        sqh->mcabac_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if EIPM
        sqh->eipm_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if ETMVP
        sqh->etmvp_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if SUB_TMVP
        sqh->sbtmvp_enable_flag = (u8)com_bsr_read1(bs);
#endif
#if MVAP
        sqh->mvap_enable_flag = (u8)com_bsr_read1(bs);
        sqh->num_of_mvap_cand = sqh->mvap_enable_flag ? ALLOWED_MVAP_NUM : 0;
#endif
#if DBK_SCC
        sqh->loop_filter_type_enable_flag = (u8)com_bsr_read1(bs);
#endif
        // end of phase-2 sqh
#if PHASE_2_PROFILE
    }
    else
    {
#if ESAO
        sqh->esao_enable_flag = 0;
#endif 
#if UMVE_ENH
        sqh->umve_enh_enable_flag = 0;
#endif
#if AWP
        sqh->awp_enable_flag = 0;
#endif
#if AWP_MVR
        sqh->awp_mvr_enable_flag = 0;
#endif
#if BIO
        sqh->bio_enable_flag = 0;
#endif
#if BGC
        sqh->bgc_enable_flag = 0;
#endif
#if DMVR
        sqh->dmvr_enable_flag = 0;
#endif
#if INTERPF
        sqh->interpf_enable_flag = 0;
#endif
#if IBC_ABVR
        sqh->abvr_enable_flag = 0;
#endif
#if USE_SP
        sqh->sp_enable_flag = 0;
#endif
#if IBC_BVP
        sqh->num_of_hbvp_cand = 0;
#endif
#if AFFINE_UMVE
        sqh->affine_umve_enable_flag = 0;
#endif
#if MIPF
        sqh->mipf_enable_flag = 0;
#endif
#if TSCPM && ENHANCE_TSPCM
        sqh->enhance_tscpm_enable_flag = 0;
#endif
#if IPF_CHROMA
        sqh->chroma_ipf_enable_flag = 0;
#endif
#if PMC
        sqh->pmc_enable_flag = 0;
#endif
#if FIMC
        sqh->fimc_enable_flag = 0;
#endif
#if USE_IBC
        sqh->ibc_flag = 0;
#endif
#if SBT
        sqh->sbt_enable_flag = 0;
#endif
#if IST
        sqh->ist_enable_flag = 0;
#endif
#if ISTS
        sqh->ists_enable_flag = 0;
#endif
#if EST
        sqh->est_enable_flag = 0;
#endif
#if SRCC
        sqh->srcc_enable_flag = 0;
#endif
#if CABAC_MULTI_PROB
        sqh->mcabac_enable_flag = 0;
#endif
#if EIPM
        sqh->eipm_enable_flag = 0;
#endif
#if ETMVP
        sqh->etmvp_enable_flag = 0;
#endif
#if SUB_TMVP
        sqh->sbtmvp_enable_flag = 0;
#endif
#if MVAP
        sqh->mvap_enable_flag = 0;
        sqh->num_of_mvap_cand = 0;
#endif
#if DBK_SCC
        sqh->loop_filter_type_enable_flag = 0;
#endif
    }
#endif // end of PHASE_2_PROFILE

    if (sqh->low_delay == 0)
    {
        sqh->output_reorder_delay = (u8)com_bsr_read(bs, 5);
    }
    else
    {
        sqh->output_reorder_delay = 0;
    }
#if PATCH
    sqh->cross_patch_loop_filter = (u8)com_bsr_read1(bs); //cross_patch_loopfilter_enable_flag
    sqh->patch_ref_colocated = (u8)com_bsr_read1(bs);     //ref_colocated_patch_flag
    sqh->patch_stable = (u8)com_bsr_read1(bs);            //stable_patch_flag
    if (sqh->patch_stable)
    {
        sqh->patch_uniform = (u8)com_bsr_read1(bs);       //uniform_patch_flag
        if (sqh->patch_uniform)
        {
            assert(com_bsr_read1(bs) == 1);                      //marker_bit
            sqh->patch_width_minus1 = (u8)com_bsr_read_ue(bs);   //patch_width_minus1
            sqh->patch_height_minus1 = (u8)com_bsr_read_ue(bs);  //patch_height_minus1
        }
    }
#endif
    com_bsr_read(bs, 2); //reserved_bits r(2)

    //next_start_code()
    assert(com_bsr_read1(bs)==1); // stuffing_bit '1'
    while (!COM_BSR_IS_BYTE_ALIGN(bs))
    {
        assert(com_bsr_read1(bs) == 0); // stuffing_bit '0'
    }
    while (com_bsr_next(bs, 24) != 0x1)
    {
        assert(com_bsr_read(bs, 8) == 0); // stuffing_byte '00000000'
    };

    /* check values */
    int max_cuwh = 1 << sqh->log2_max_cu_width_height;
    if(max_cuwh < MIN_CU_SIZE || max_cuwh > MAX_CU_SIZE)
    {
        return COM_ERR_MALFORMED_BITSTREAM;
    }
    return COM_OK;
}

int dec_eco_pic_header(COM_BSR * bs, COM_PIC_HEADER * pic_header, COM_SQH * sqh, int* need_minus_256)
{
    unsigned int ret = com_bsr_read(bs, 24);
    assert(ret == 1);
    unsigned int start_code = com_bsr_read(bs, 8);
    if (start_code == 0xB3)
    {
        pic_header->slice_type = SLICE_I;
    }
    assert(start_code == 0xB6 || start_code == 0xB3);
    if (start_code != 0xB3)//MX: not SLICE_I
    {
      pic_header->random_access_decodable_flag = com_bsr_read1(bs);
    }
    pic_header->bbv_delay = com_bsr_read(bs, 8) << 24 | com_bsr_read(bs, 8) << 16 | com_bsr_read(bs, 8) << 8 | com_bsr_read(bs, 8);

    if (start_code == 0xB6)
    {
        //picture_coding_type
        u8 val = (u8)com_bsr_read(bs, 2);
        assert(val == 1 || val == 2);
        pic_header->slice_type = val == 1 ? SLICE_P : SLICE_B;
    }
    if (pic_header->slice_type == SLICE_I)
    {
        pic_header->time_code_flag = com_bsr_read1(bs);
        if (pic_header->time_code_flag == 1)
        {
            pic_header->time_code = com_bsr_read(bs, 24);
        }
    }

    pic_header->decode_order_index = com_bsr_read(bs, 8);

#if LIBVC_ON
    pic_header->library_picture_index = -1;
    if (pic_header->slice_type == SLICE_I)
    {
        if (sqh->library_stream_flag)
        {
            pic_header->library_picture_index = com_bsr_read_ue(bs);
        }
    }
#endif

    if (sqh->temporal_id_enable_flag == 1)
    {
        pic_header->temporal_id = (u8)com_bsr_read(bs, 3);
    }
    if (sqh->low_delay == 0)
    {
        pic_header->picture_output_delay = com_bsr_read_ue(bs);
        pic_header->bbv_check_times = 0;
    }
    else
    {
        pic_header->picture_output_delay = 0;
        pic_header->bbv_check_times = com_bsr_read_ue(bs);
    }

    //the field information below is not used by decoder -- start
    pic_header->progressive_frame = com_bsr_read1(bs);
    assert(pic_header->progressive_frame == 1);
    if (pic_header->progressive_frame == 0)
    {
        pic_header->picture_structure = com_bsr_read1(bs);
    }
    else
    {
        pic_header->picture_structure = 1;
    }
    pic_header->top_field_first = com_bsr_read1(bs);
    pic_header->repeat_first_field = com_bsr_read1(bs);
    if (sqh->field_coded_sequence == 1)
    {
        pic_header->top_field_picture_flag = com_bsr_read1(bs);
        com_bsr_read1(bs); // reserved_bits r(1)
    }
    // -- end

#if LIBVC_ON
    if (!sqh->library_stream_flag)
    {
#endif
      if (pic_header->decode_order_index < g_DOIPrev)
      {
        *need_minus_256 = 1;
        g_CountDOICyCleTime++;                    // initialized the number .
      }
      g_DOIPrev = pic_header->decode_order_index;
      pic_header->dtr = pic_header->decode_order_index + (DOI_CYCLE_LENGTH*g_CountDOICyCleTime) + pic_header->picture_output_delay - sqh->output_reorder_delay;//MX: in the decoder, the only usage of g_CountDOICyCleTime is to derive the POC/POC. here is the dtr
#if LIBVC_ON
    }
    else
    {
        pic_header->dtr = pic_header->decode_order_index + (DOI_CYCLE_LENGTH*g_CountDOICyCleTime) + pic_header->picture_output_delay - sqh->output_reorder_delay;
    }
#endif

    //initialization
    pic_header->rpl_l0_idx = pic_header->rpl_l1_idx = -1;
    pic_header->ref_pic_list_sps_flag[0] = pic_header->ref_pic_list_sps_flag[1] = 0;
    pic_header->rpl_l0.ref_pic_num = 0;
    pic_header->rpl_l1.ref_pic_num = 0;
    pic_header->rpl_l0.ref_pic_active_num = 0;
    pic_header->rpl_l1.ref_pic_active_num = 0;
    for (int i = 0; i < MAX_NUM_REF_PICS; i++)
    {
        pic_header->rpl_l0.ref_pics[i] = 0;
        pic_header->rpl_l1.ref_pics[i] = 0;
        pic_header->rpl_l0.ref_pics_ddoi[i] = 0;
        pic_header->rpl_l1.ref_pics_ddoi[i] = 0;
        pic_header->rpl_l0.ref_pics_doi[i] = 0;
        pic_header->rpl_l1.ref_pics_doi[i] = 0;
#if LIBVC_ON
        pic_header->rpl_l0.library_index_flag[i] = 0;
        pic_header->rpl_l1.library_index_flag[i] = 0;
#endif
    }

#if HLS_RPL
    //TBD(@Chernyak) if(!IDR) condition to be added here
    pic_header->poc = pic_header->dtr;
    //pic_header->poc = com_bsr_read_ue(bs);
    // L0 candidates signaling
    pic_header->ref_pic_list_sps_flag[0] = com_bsr_read1(bs);
    if (pic_header->ref_pic_list_sps_flag[0])
    {
        if (sqh->rpls_l0_num)
        {
            if (sqh->rpls_l0_num > 1)
            {
                pic_header->rpl_l0_idx = com_bsr_read_ue(bs);
            }
            else//if sps only have 1 RPL, no need to signal the idx
            {
                pic_header->rpl_l0_idx = 0;
            }
            memcpy(&pic_header->rpl_l0, &sqh->rpls_l0[pic_header->rpl_l0_idx], sizeof(pic_header->rpl_l0));
            pic_header->rpl_l0.poc = pic_header->poc;
        }
    }
    else
    {
#if LIBVC_ON
        dec_eco_rlp(bs, &pic_header->rpl_l0, sqh);
#else
        dec_eco_rlp(bs, &pic_header->rpl_l0);
#endif
        pic_header->rpl_l0.poc = pic_header->poc;
    }

    //L1 candidates signaling
    pic_header->ref_pic_list_sps_flag[1] = sqh->rpl1_index_exist_flag ? com_bsr_read1(bs) : pic_header->ref_pic_list_sps_flag[0];
    if (pic_header->ref_pic_list_sps_flag[1])
    {
        if (sqh->rpls_l1_num > 1 && sqh->rpl1_index_exist_flag)
        {
            pic_header->rpl_l1_idx = com_bsr_read_ue(bs);
        }
        else if (!sqh->rpl1_index_exist_flag)
        {
            pic_header->rpl_l1_idx = pic_header->rpl_l0_idx;
        }
        else//if sps only have 1 RPL, no need to signal the idx
        {
            assert(sqh->rpls_l1_num == 1);
            pic_header->rpl_l1_idx = 0;
        }
        memcpy(&pic_header->rpl_l1, &sqh->rpls_l1[pic_header->rpl_l1_idx], sizeof(pic_header->rpl_l1));
        pic_header->rpl_l1.poc = pic_header->poc;
    }
    else
    {
#if LIBVC_ON
        dec_eco_rlp(bs, &pic_header->rpl_l1, sqh);
#else
        dec_eco_rlp(bs, &pic_header->rpl_l1);
#endif
        pic_header->rpl_l1.poc = pic_header->poc;
    }

    if (pic_header->slice_type != SLICE_I)
    {
        pic_header->num_ref_idx_active_override_flag = com_bsr_read1(bs);
        if (pic_header->num_ref_idx_active_override_flag)
        {
            pic_header->rpl_l0.ref_pic_active_num = (u32)com_bsr_read_ue(bs) + 1;
            if (pic_header->slice_type == SLICE_P)
            {
                pic_header->rpl_l1.ref_pic_active_num = 0;
            }
            else if (pic_header->slice_type == SLICE_B)
            {
                pic_header->rpl_l1.ref_pic_active_num = (u32)com_bsr_read_ue(bs) + 1;
            }
        }
        else
        {
            //Hendry -- @Roman: we need to signal the num_ref_idx_default_active_minus1[ i ]. This syntax element is in the PPS in the spec document
            pic_header->rpl_l0.ref_pic_active_num = sqh->num_ref_default_active_minus1[0]+1;
            if (pic_header->slice_type == SLICE_P)
            {
              pic_header->rpl_l1.ref_pic_active_num = 0;
            }
            else
            {
              pic_header->rpl_l1.ref_pic_active_num = sqh->num_ref_default_active_minus1[1] + 1;
            }
        }
    }
    if (pic_header->slice_type == SLICE_I)
    {
        pic_header->rpl_l0.ref_pic_active_num = 0;
        pic_header->rpl_l1.ref_pic_active_num = 0;
    }
#if LIBVC_ON
    pic_header->is_RLpic_flag = 0;
    if (pic_header->slice_type != SLICE_I)
    {
        int only_ref_libpic_flag = 1;
        for (int i = 0; i < pic_header->rpl_l0.ref_pic_active_num; i++)
        {
            if (!pic_header->rpl_l0.library_index_flag[i])
            {
                only_ref_libpic_flag = 0;
                break;
            }
        }
        if (only_ref_libpic_flag)
        {
            for (int i = 0; i < pic_header->rpl_l1.ref_pic_active_num; i++)
            {
                if (!pic_header->rpl_l1.library_index_flag[i])
                {
                    only_ref_libpic_flag = 0;
                    break;
                }
            }
        }
        pic_header->is_RLpic_flag = only_ref_libpic_flag;
    }
#endif
#endif
    pic_header->fixed_picture_qp_flag = com_bsr_read1(bs);
    pic_header->picture_qp = com_bsr_read(bs, 7);
#if CABAC_MULTI_PROB
    if (pic_header->slice_type == SLICE_I)
    {
        mCabac_ws = MCABAC_SHIFT_I;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 0;
        counter_thr2 = COUNTER_THR_I;
    }
    else if (pic_header->slice_type == SLICE_B)
    {
        mCabac_ws = MCABAC_SHIFT_B;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 3;
        counter_thr2 = COUNTER_THR_B;
    }
    else
    {
        mCabac_ws = MCABAC_SHIFT_P;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 3;
        counter_thr2 = COUNTER_THR_P;
    }
    if (sqh->mcabac_enable_flag)
    {
        g_compatible_back = 0;
    }
    else
    {
        g_compatible_back = 1;
    }
#endif
    //the reserved_bits only appears in inter_picture_header, so add the non-I-slice check
    if( pic_header->slice_type != SLICE_I && !(pic_header->slice_type == SLICE_B && pic_header->picture_structure == 1) )
    {
        com_bsr_read1( bs ); // reserved_bits r(1)
    }
    dec_eco_DB_param(bs, pic_header
#if DBK_SCC
        , sqh
#endif
    );

    pic_header->chroma_quant_param_disable_flag = (u8)com_bsr_read1(bs);
    if (pic_header->chroma_quant_param_disable_flag == 0)
    {
        pic_header->chroma_quant_param_delta_cb = (u8)com_bsr_read_se(bs);
        pic_header->chroma_quant_param_delta_cr = (u8)com_bsr_read_se(bs);
    }
    else
    {
        pic_header->chroma_quant_param_delta_cb = pic_header->chroma_quant_param_delta_cr = 0;
    }

    if (sqh->wq_enable)
    {
        pic_header->pic_wq_enable = com_bsr_read1(bs);
        if (pic_header->pic_wq_enable)
        {
            pic_header->pic_wq_data_idx = com_bsr_read(bs, 2);
            if (pic_header->pic_wq_data_idx == 0)
            {
                memcpy(pic_header->wq_4x4_matrix, sqh->wq_4x4_matrix, sizeof(sqh->wq_4x4_matrix));
                memcpy(pic_header->wq_8x8_matrix, sqh->wq_8x8_matrix, sizeof(sqh->wq_8x8_matrix));
            }
            else if (pic_header->pic_wq_data_idx == 1)
            {
                int delta, i;
                com_bsr_read1( bs ); //reserved_bits r(1)
                pic_header->wq_param = com_bsr_read(bs, 2); //weight_quant_param_index
                pic_header->wq_model = com_bsr_read(bs, 2); //weight_quant_model
                if (pic_header->wq_param == 0)
                {
                    memcpy(pic_header->wq_param_vector, tab_wq_param_default[1], sizeof(pic_header->wq_param_vector));
                }
                else if (pic_header->wq_param == 1)
                {
                    for (i = 0; i < 6; i++)
                    {
                        delta = com_bsr_read_se(bs);
                        pic_header->wq_param_vector[i] = delta + tab_wq_param_default[0][i];
                    }
                }
                else
                {
                    for (i = 0; i < 6; i++)
                    {
                        delta = com_bsr_read_se(bs);
                        pic_header->wq_param_vector[i] = delta + tab_wq_param_default[1][i];
                    }
                }
                set_pic_wq_matrix_by_param(pic_header->wq_param_vector, pic_header->wq_model, pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
            }
            else
            {
                read_wq_matrix(bs, pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
            }
        }
        else
        {
            init_pic_wq_matrix(pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
        }
    }
    else
    {
        pic_header->pic_wq_enable = 0;
        init_pic_wq_matrix(pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
    }
#if ESAO
    if (sqh->esao_enable_flag)
    {
        dec_eco_esao_pic_header(bs, pic_header);
    }
    else
    {
        memset(pic_header->pic_esao_on, 0, N_C * sizeof(int));
    }
#endif
    if (pic_header->tool_alf_on)
    {
        /* decode ALF flag and ALF coeff */
        dec_eco_ALF_param(bs, pic_header);
    }
    else
    {
        memset( pic_header->pic_alf_on, 0, N_C * sizeof( int ) );
    }

    if (pic_header->slice_type != SLICE_I && sqh->affine_enable_flag)
    {
        pic_header->affine_subblock_size_idx = com_bsr_read(bs, 1);
    }
#if USE_IBC
#if PHASE_2_PROFILE
    if (sqh->ibc_flag)
    {
#endif
        pic_header->ibc_flag = (u8)com_bsr_read1(bs);
#if PHASE_2_PROFILE
    }
    else
    {
        pic_header->ibc_flag = 0;
    }
#endif
#endif
#if USE_SP
#if PHASE_2_PROFILE
    if (sqh->sp_enable_flag)
    {
#endif
        pic_header->sp_pic_flag = (u8)com_bsr_read1(bs);
#if PHASE_2_PROFILE
    }
    else
    {
        pic_header->sp_pic_flag = 0;
    }
#endif
#endif
#if FIMC
#if PHASE_2_PROFILE
    if (sqh->fimc_enable_flag)
    {
#endif
        pic_header->fimc_pic_flag = (u8)com_bsr_read1(bs);
#if PHASE_2_PROFILE
    }
    else
    {
        pic_header->fimc_pic_flag = 0;
    }
#endif
#endif
#if ISTS
    if (sqh->ists_enable_flag)
    {
        pic_header->ph_ists_enable_flag = (u8)com_bsr_read1(bs);
    }
#if PHASE_2_PROFILE
    else
    {
        pic_header->ph_ists_enable_flag = 0;
    }
#endif
#endif
#if AWP_SCC
    if (sqh->awp_enable_flag)
    {
        pic_header->ph_awp_refine_flag = (u8)com_bsr_read1(bs);
    }
#if PHASE_2_PROFILE
    else
    {
        pic_header->ph_awp_refine_flag = 0;
    }
#endif
#endif
#if UMVE_ENH 
    if (sqh->umve_enh_enable_flag && pic_header->slice_type != SLICE_I)
    {
        pic_header->umve_set_flag = (u8)com_bsr_read1(bs);
    }
    else
    {
        pic_header->umve_set_flag = 0;
    }
#endif
    /* byte align */
    ret = com_bsr_read1(bs);
    assert(ret == 1);
    while (!COM_BSR_IS_BYTE_ALIGN(bs))
    {
        assert(com_bsr_read1(bs) == 0);
    }
    while (com_bsr_next(bs, 24) != 0x1)
    {
        assert(com_bsr_read(bs, 8) == 0);
    }
    return COM_OK;
}

int dec_eco_patch_header(COM_BSR * bs, COM_SQH *sqh, COM_PIC_HEADER * ph, COM_SH_EXT * sh,PATCH_INFO *patch)
{
    //patch_start_code_prefix & patch_index
    unsigned int ret = com_bsr_read(bs, 24);
    assert(ret == 1);
    patch->idx = com_bsr_read(bs, 8);
    printf("%d ", patch->idx);
    assert(patch->idx >= 0x00 && patch->idx <= 0x8E);

    if (!ph->fixed_picture_qp_flag)
    {
        sh->fixed_slice_qp_flag = (u8)com_bsr_read1(bs);
        sh->slice_qp = (u8)com_bsr_read(bs, 7);
    }
    else
    {
        sh->fixed_slice_qp_flag = 1;
        sh->slice_qp = (u8)ph->picture_qp;
    }
    if (sqh->sample_adaptive_offset_enable_flag)
    {
        sh->slice_sao_enable[Y_C] = (u8)com_bsr_read1(bs);
        sh->slice_sao_enable[U_C] = (u8)com_bsr_read1(bs);
        sh->slice_sao_enable[V_C] = (u8)com_bsr_read1(bs);
    }
#if PATCH_HEADER_PARAM_TEST
    //for test each patch has different sao param
    printf("SAO:%d%d%d ", sh->slice_sao_enable[Y_C], sh->slice_sao_enable[U_C], sh->slice_sao_enable[V_C]);
#endif
    /* byte align */
    while (!COM_BSR_IS_BYTE_ALIGN(bs))
    {
        assert(com_bsr_read1(bs) == 1);
    }
    return COM_OK;
}

#if PATCH
int dec_eco_send(COM_BSR * bs)
{
    while (com_bsr_next(bs, 24) != 0x1)
    {
        com_bsr_read(bs, 8);
    }
    int ret = com_bsr_read(bs, 24);
    assert(ret == 1);
    ret = com_bsr_read(bs, 8);
    assert(ret == 0x8F);
    return COM_OK;
}
#endif

int dec_eco_udata(DEC_CTX * ctx, COM_BSR * bs)
{
    int    i;
    u32 code;
    /* should be aligned before adding user data */
    com_assert_rv(COM_BSR_IS_BYTE_ALIGN(bs), COM_ERR_UNKNOWN);
    code = com_bsr_read(bs, 8);
    while (code != COM_UD_END)
    {
        switch(code)
        {
        case COM_UD_PIC_SIGNATURE:
            /* read signature (HASH) from bitstream */
            for(i = 0; i < 16; i++)
            {
                ctx->pic_sign[i] = (u8)com_bsr_read(bs, 8);
                if (i % 2 == 1)
                {
                    u8 bit = com_bsr_read1(bs); //add one bit to prevent from start code, like a marker bit in SQH
                    assert(bit == 1);
            }
            }
            ctx->pic_sign_exist = 1;
            break;
        default:
            com_assert_rv(0, COM_ERR_UNEXPECTED);
        }
        code = com_bsr_read(bs, 8);
    }
    return COM_OK;
}

void dec_eco_affine_motion_derive(DEC_CTX *ctx, DEC_CORE *core, COM_BSR *bs, DEC_SBAC *sbac, int inter_dir, int cu_width, int cu_height, CPMV affine_mvp[VER_NUM][MV_D], s16 mvd[MV_D])
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int vertex;
    int vertex_num = mod_info_curr->affine_flag + 1;
    /* forward */
    if (inter_dir == PRED_L0 || inter_dir == PRED_BI)
    {
        mod_info_curr->refi[REFP_0] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_0]);
        com_get_affine_mvp_scaling(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_0, affine_mvp, vertex_num
#if BD_AFFINE_AMVR
                                   , mod_info_curr->mvr_idx
#endif
                                  );

        for (vertex = 0; vertex < vertex_num; vertex++)
        {
            decode_mvd(bs, sbac, mvd);
#if BD_AFFINE_AMVR
            u8 amvr_shift = Tab_Affine_AMVR(mod_info_curr->mvr_idx);
            s32 mv_x = (s32)affine_mvp[vertex][MV_X] + ((s32)mvd[MV_X] << amvr_shift);
            s32 mv_y = (s32)affine_mvp[vertex][MV_Y] + ((s32)mvd[MV_Y] << amvr_shift);
#else
            s32 mv_x = (s32)affine_mvp[vertex][MV_X] + (s32)mvd[MV_X];
            s32 mv_y = (s32)affine_mvp[vertex][MV_Y] + (s32)mvd[MV_Y];
#endif
            mod_info_curr->affine_mv[REFP_0][vertex][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mv_x);
            mod_info_curr->affine_mv[REFP_0][vertex][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mv_y);
        }
    }
    else
    {
        mod_info_curr->refi[REFP_0] = REFI_INVALID;
        for (vertex = 0; vertex < vertex_num; vertex++)
        {
            mod_info_curr->affine_mv[REFP_0][vertex][MV_X] = 0;
            mod_info_curr->affine_mv[REFP_0][vertex][MV_Y] = 0;
        }
        mod_info_curr->refi[REFP_0] = REFI_INVALID;
        mod_info_curr->mv[REFP_0][MV_X] = 0;
        mod_info_curr->mv[REFP_0][MV_Y] = 0;
    }

    /* backward */
    if (inter_dir == PRED_L1 || inter_dir == PRED_BI)
    {
        mod_info_curr->refi[REFP_1] = (s8)decode_refidx(bs, sbac, ctx->dpm.num_refp[REFP_1]);
        com_get_affine_mvp_scaling(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, ctx->ptr, REFP_1, affine_mvp, vertex_num
#if BD_AFFINE_AMVR
                                   , mod_info_curr->mvr_idx
#endif
                                  );

        for (vertex = 0; vertex < vertex_num; vertex++)
        {
            decode_mvd(bs, sbac, mvd);
#if BD_AFFINE_AMVR
            u8 amvr_shift = Tab_Affine_AMVR(mod_info_curr->mvr_idx);
            s32 mv_x = (s32)affine_mvp[vertex][MV_X] + ((s32)mvd[MV_X] << amvr_shift);
            s32 mv_y = (s32)affine_mvp[vertex][MV_Y] + ((s32)mvd[MV_Y] << amvr_shift);
#else
            s32 mv_x = (s32)affine_mvp[vertex][MV_X] + (s32)mvd[MV_X];
            s32 mv_y = (s32)affine_mvp[vertex][MV_Y] + (s32)mvd[MV_Y];
#endif
            mod_info_curr->affine_mv[REFP_1][vertex][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mv_x);
            mod_info_curr->affine_mv[REFP_1][vertex][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mv_y);
        }
    }
    else
    {
        mod_info_curr->refi[REFP_1] = REFI_INVALID;
        for (vertex = 0; vertex < vertex_num; vertex++)
        {
            mod_info_curr->affine_mv[REFP_1][vertex][MV_X] = 0;
            mod_info_curr->affine_mv[REFP_1][vertex][MV_Y] = 0;
        }
        mod_info_curr->refi[REFP_1] = REFI_INVALID;
        mod_info_curr->mv[REFP_1][MV_X] = 0;
        mod_info_curr->mv[REFP_1][MV_Y] = 0;
    }
}

int dec_eco_DB_param(COM_BSR *bs, COM_PIC_HEADER *pic_header
#if DBK_SCC
    , COM_SQH *sqh
#endif
)
{
    pic_header->loop_filter_disable_flag = (u8)com_bsr_read1(bs);
    if (pic_header->loop_filter_disable_flag == 0)
    {
#if DBK_SCC
        if (sqh->loop_filter_type_enable_flag)
        {
            pic_header->loop_fitler_type = (u8)com_bsr_read1(bs);
        }
#endif
        pic_header->loop_filter_parameter_flag = (u8)com_bsr_read(bs, 1);
        if (pic_header->loop_filter_parameter_flag)
        {
            pic_header->alpha_c_offset = com_bsr_read_se(bs);
            pic_header->beta_offset = com_bsr_read_se(bs);
        }
        else
        {
            pic_header->alpha_c_offset = 0;
            pic_header->beta_offset = 0;
        }
    }
    return  COM_OK;
}

int dec_eco_sao_mergeflag(DEC_SBAC *sbac, COM_BSR *bs, int mergeleft_avail, int mergeup_avail)
{
    int MergeLeft = 0;
    int MergeUp = 0;
    int act_ctx = mergeleft_avail + mergeup_avail;
    int act_sym = 0;

    if (act_ctx == 1)
    {
        act_sym = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.sao_merge_flag[0]);
    }
    else if (act_ctx == 2)
    {
        act_sym = dec_sbac_decode_bin(bs, sbac, &sbac->ctx.sao_merge_flag[1]);
        if (act_sym != 1)
        {
            act_sym += (dec_sbac_decode_bin(bs, sbac, &sbac->ctx.sao_merge_flag[2]) << 1);
        }
    }

    /* Get SaoMergeMode by SaoMergeTypeIndex */
    if (mergeleft_avail && !mergeup_avail)
    {
        return act_sym ? 2 : 0;
    }
    if (mergeup_avail && !mergeleft_avail)
    {
        return act_sym ? 1 : 0;
    }
    return act_sym ? 3 - act_sym : 0;
}

int dec_eco_sao_mode(DEC_SBAC *sbac, COM_BSR *bs)
{
    int  act_sym = 0;

    if (!dec_sbac_decode_bin(bs, sbac, sbac->ctx.sao_mode))
    {
        act_sym = 1 + !sbac_decode_bin_ep(bs, sbac);
    }
    return act_sym;
}

int dec_eco_sao_offset(DEC_SBAC *sbac, COM_BSR *bs, SAOBlkParam *saoBlkParam, int *offset)
{
    int i;
    assert(saoBlkParam->modeIdc == SAO_MODE_NEW);
    static const int EO_OFFSET_INV__MAP[] = { 1, 0, 2, -1, 3, 4, 5, 6 };

    for (i = 0; i < 4; i++)
    {
        int act_sym;
        if (saoBlkParam->typeIdc == SAO_TYPE_BO)
        {
            act_sym = !dec_sbac_decode_bin(bs, sbac, sbac->ctx.sao_offset);
            if (act_sym)
            {
                act_sym += sbac_read_truncate_unary_sym_ep(bs, sbac, saoclip[SAO_CLASS_BO][2]);

                if (sbac_decode_bin_ep(bs, sbac))
                {
                    act_sym = -act_sym;
                }
            }
        }
        else
        {
            int value2 = (i >= 2) ? (i + 1) : i;
            int maxvalue = saoclip[value2][2];
            int cnt = sbac_read_truncate_unary_sym_ep(bs, sbac, maxvalue + 1);

            if (value2 == SAO_CLASS_EO_FULL_VALLEY)
            {
                act_sym = EO_OFFSET_INV__MAP[cnt];
            }
            else if (value2 == SAO_CLASS_EO_FULL_PEAK)
            {
                act_sym = -EO_OFFSET_INV__MAP[cnt];
            }
            else if (value2 == SAO_CLASS_EO_HALF_PEAK)
            {
                act_sym = -cnt;
            }
            else
            {
                act_sym = cnt;
            }
        }
        offset[i] = act_sym;
        offset[i] = offset[i];
    }
    return 1;
}

int dec_eco_sao_EO_type(DEC_SBAC *sbac, COM_BSR *bs, SAOBlkParam *saoBlkParam)
{
    int bin0 = sbac_decode_bin_ep( bs, sbac ) << 0;
    int bin1 = sbac_decode_bin_ep( bs, sbac ) << 1;
    return bin0 + bin1;
}

void dec_eco_sao_BO_start(DEC_SBAC *sbac, COM_BSR *bs, SAOBlkParam *saoBlkParam, int stBnd[2])
{
    int act_sym = 0;
    int start = 0;
    start += (sbac_decode_bin_ep( bs, sbac ) << 0);
    start += (sbac_decode_bin_ep( bs, sbac ) << 1);
    start += (sbac_decode_bin_ep( bs, sbac ) << 2);
    start += (sbac_decode_bin_ep( bs, sbac ) << 3);
    start += (sbac_decode_bin_ep( bs, sbac ) << 4);

    int golomb_order = 1 + sbac_read_truncate_unary_sym_ep(bs, sbac, 4);

    act_sym = (1 << golomb_order) - 2;
    golomb_order %= 4;

    while (golomb_order--)
    {
        if (sbac_decode_bin_ep(bs, sbac))
        {
            act_sym += (1 << golomb_order);
        }
    }

    stBnd[0] = start;
    stBnd[1] = (start + 2 + act_sym) % 32;
}

#if ESAO
int dec_eco_esao_lcu_control_flag(DEC_SBAC *sbac, COM_BSR *bs)
{
    int  act_sym = 0;
    act_sym = dec_sbac_decode_bin(bs, sbac, sbac->ctx.esao_lcu_enable);
    return act_sym;
}

int dec_esao_chroma_band_flag(DEC_SBAC *sbac, COM_BSR *bs)
{
    int  act_sym = 0;
    act_sym = dec_sbac_decode_bin(bs, sbac, sbac->ctx.esao_chroma_mode_flag);
    return act_sym;
}

int dec_esao_chroma_len(DEC_SBAC *sbac, COM_BSR *bs, int mode_indx, int start_band[2])
{
    int class_count = tab_esao_chroma_class[mode_indx];
    int len = 0, count = 0;
    int i;
    while (class_count)
    {
        len++;
        class_count = class_count >> 1;
    }
    start_band[0] = 0;
    for (i = 0; i < len; i++)
    {
        start_band[0] += sbac_decode_bin_ep(bs, sbac) << i;
    }
    for (i = 0; i < len; i++)
    {
        count += sbac_decode_bin_ep(bs, sbac) << i;
    }
    return count;
}

void dec_eco_esao_offset(DEC_SBAC *sbac, COM_BSR *bs, int *offset, int num)
{
    int i;
    for (i = 0; i < num; i++)
    {
        int act_sym = 0;
        act_sym = !dec_sbac_decode_bin(bs, sbac, sbac->ctx.esao_offset);
        if (act_sym)
        {
            act_sym += sbac_read_truncate_unary_sym_ep_esao(bs, sbac, esao_clip[0][2]);
            if (sbac_decode_bin_ep(bs, sbac))
            {
                act_sym = -act_sym;
            }
        }
        offset[i] = act_sym;
    }
}

void dec_eco_esao_param(DEC_CTX * ctx, DEC_SBAC *sbac, COM_BSR *bs)
{
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        if (ctx->info.pic_header.pic_esao_on[comp_idx])
        {
            int num_count, num;
            if (comp_idx == 0)
            {
                num = (ctx->info.pic_header.esao_luma_type == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
                num_count = (ctx->info.pic_header.esao_adaptive_param[comp_idx] + 1) * num;
            }
            else
            {
                num_count = tab_esao_chroma_class[ctx->info.pic_header.esao_adaptive_param[comp_idx]];
            }
            int start_band[2];
            if (comp_idx != Y_C)
            {
                int chroma_optimal_flag = dec_esao_chroma_band_flag(sbac, bs);
                if (chroma_optimal_flag)
                {
                    memset(ctx->pic_esao_params[comp_idx].offset, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
                    num_count = dec_esao_chroma_len(sbac, bs, ctx->info.pic_header.esao_adaptive_param[comp_idx], start_band);
                    dec_eco_esao_offset(sbac, bs, ctx->pic_esao_params[comp_idx].offset + start_band[0], num_count);
                }
                else
                {
                    dec_eco_esao_offset(sbac, bs, ctx->pic_esao_params[comp_idx].offset, num_count);
                }
            }
            else
            {
                dec_eco_esao_offset(sbac, bs, ctx->pic_esao_params[comp_idx].offset, num_count);
            }
            if (ctx->info.pic_header.esao_lcu_enable[comp_idx])
            {
                for (int lcu_index = 0; lcu_index < ctx->lcu_cnt; lcu_index++)
                {
                    ctx->pic_esao_params[comp_idx].lcu_flag[lcu_index] = dec_eco_esao_lcu_control_flag(sbac, bs);
                }
            }
        }
    }
}

void dec_eco_esao_pic_header(COM_BSR *bs, COM_PIC_HEADER *pic_header)
{
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        int label_yuv_bit[N_C] = { ESAO_LABEL_NUM_IN_BIT_Y ,ESAO_LABEL_NUM_IN_BIT_U,ESAO_LABEL_NUM_IN_BIT_V };
        pic_header->pic_esao_on[comp_idx] = com_bsr_read1(bs);
        if (pic_header->pic_esao_on[comp_idx])
        {
            pic_header->esao_lcu_enable[comp_idx] = com_bsr_read1(bs);
            if (comp_idx == 0 && ESAO_LUMA_TYPES > 1)
            {
                pic_header->esao_luma_type = com_bsr_read1(bs);
            }
            pic_header->esao_adaptive_param[comp_idx] = com_bsr_read(bs, label_yuv_bit[comp_idx]);
        }
    }
}
#endif

int dec_eco_ALF_param(COM_BSR *bs, COM_PIC_HEADER *sh)
{
    ALFParam **m_alfPictureParam = sh->m_alfPictureParam;
    int *pic_alf_on = sh->pic_alf_on;
    int compIdx;

    pic_alf_on[Y_C] = com_bsr_read(bs, 1);
    pic_alf_on[U_C] = com_bsr_read(bs, 1);
    pic_alf_on[V_C] = com_bsr_read(bs, 1);
    m_alfPictureParam[Y_C]->alf_flag = pic_alf_on[Y_C];
    m_alfPictureParam[U_C]->alf_flag = pic_alf_on[U_C];
    m_alfPictureParam[V_C]->alf_flag = pic_alf_on[V_C];
    if (pic_alf_on[Y_C] || pic_alf_on[U_C] || pic_alf_on[V_C])
    {
        for (compIdx = 0; compIdx < N_C; compIdx++)
        {
            if (pic_alf_on[compIdx])
            {
                dec_eco_AlfCoeff(bs, m_alfPictureParam[compIdx]);
            }
        }
    }
    return COM_OK;
}

int dec_eco_AlfCoeff(COM_BSR * bs, ALFParam *Alfp)
{
    int pos;
    int f = 0, symbol, pre_symbole;
    const int numCoeff = (int)ALF_MAX_NUM_COEF;
    switch (Alfp->componentID)
    {
    case U_C:
    case V_C:
        for (pos = 0; pos < numCoeff; pos++)
        {
            Alfp->coeffmulti[0][pos] = com_bsr_read_se(bs);
        }
        break;
    case Y_C:
        Alfp->filters_per_group = com_bsr_read_ue(bs);
        Alfp->filters_per_group = Alfp->filters_per_group + 1;
        memset(Alfp->filterPattern, 0, NO_VAR_BINS * sizeof(int));
        pre_symbole = 0;
        symbol = 0;
        for (f = 0; f < Alfp->filters_per_group; f++)
        {
            if (f > 0)
            {
                if (Alfp->filters_per_group != 16)
                {
                    symbol = com_bsr_read_ue(bs);
                }
                else
                {
                    symbol = 1;
                }
                Alfp->filterPattern[symbol + pre_symbole] = 1;
                pre_symbole = symbol + pre_symbole;
            }
            for (pos = 0; pos < numCoeff; pos++)
            {
                Alfp->coeffmulti[f][pos] = com_bsr_read_se(bs);
            }
        }
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
    return COM_OK;
}

u32 dec_eco_AlfLCUCtrl(COM_BSR * bs, DEC_SBAC * sbac, int compIdx, int ctx_idx)
{
    return dec_sbac_decode_bin(bs, sbac, sbac->ctx.alf_lcu_enable);
}

int dec_eco_lcu_delta_qp(COM_BSR * bs, DEC_SBAC * sbac, int last_dqp)
{
    COM_SBAC_CTX * ctx = &sbac->ctx;
    int act_ctx;
    int act_sym;
    int dquant;

    act_ctx = ((last_dqp != 0) ? 1 : 0);
    act_sym = !dec_sbac_decode_bin(bs, sbac, ctx->delta_qp + act_ctx);

    if (act_sym != 0)
    {
        u32 bin;
        act_ctx = 2;

        do
        {
            bin = dec_sbac_decode_bin(bs, sbac, ctx->delta_qp + act_ctx);
            act_ctx = min(3, act_ctx + 1);
            act_sym += !bin;
        }
        while (!bin);
    }

    dquant = (act_sym + 1) >> 1;

    // lsb is signed bit
    if ((act_sym & 0x01) == 0)
    {
        dquant = -dquant;
    }

    return dquant;
}

#if EXTENSION_USER_DATA
int user_data(DEC_CTX * ctx, COM_BSR * bs)
{
    com_bsr_read(bs, 24);
    com_bsr_read(bs, 8);                            // user_data_start_code f(32)
    while (com_bsr_next(bs, 24) != 0x000001)
    {
#if WRITE_MD5_IN_USER_DATA
        /* parse user data: MD5*/
        dec_eco_udata(ctx, bs);
#endif
    }
    return 0;
}

int sequence_display_extension(COM_BSR * bs)
{
    com_bsr_read(bs, 3);                                            // video_format             u(3)
    com_bsr_read1(bs);                                              // sample_range             u(1)
    int colour_description = com_bsr_read1(bs);                     // colour_description       u(1)
    if (colour_description)
    {
        int colour_primaries = com_bsr_read(bs, 8);                // colour_primaries         u(8)
        int transfer_characteristics = com_bsr_read(bs, 8);        // transfer_characteristics u(8)
        com_bsr_read(bs, 8);                                       // matrix_coefficients      u(8)
    }
    com_bsr_read(bs, 14);                                           // display_horizontal_size  u(14)
    com_bsr_read1(bs);                                              // marker_bit               f(1)
    com_bsr_read(bs, 14);                                           // display_vertical_size    u(14)
    char td_mode_flag = com_bsr_read1(bs);                          // td_mode_flag             u(1)
    if (td_mode_flag == 1)
    {
        com_bsr_read(bs, 8);                                        // td_packing_mode          u(8)
        com_bsr_read1(bs);                                          // view_reverse_flag        u(1)
    }

    assert(com_bsr_read1(bs) == 1);
    while (!COM_BSR_IS_BYTE_ALIGN(bs))
    {
        assert(com_bsr_read1(bs) == 0);
    }
    while (com_bsr_next(bs, 24) != 0x1)
    {
        assert(com_bsr_read(bs, 8) == 0);
    }
    return 0;
}

int temporal_scalability_extension(COM_BSR * bs)
{
    int num_of_temporal_level_minus1 = com_bsr_read(bs, 3); // num_of_temporal_level_minus1 u(3)
    for (int i = 0; i < num_of_temporal_level_minus1; i++)
    {
        com_bsr_read(bs, 4);                                // temporal_frame_rate_code[i]  u(4)
        com_bsr_read(bs, 18);                               // temporal_bit_rate_lower[i]   u(18)
        com_bsr_read1(bs);                                  // marker_bit                   f(1)
        com_bsr_read(bs, 12);                               // temporal_bit_rate_upper[i]   u(12)
    }
    return 0;
}

int copyright_extension(COM_BSR * bs)
{
    com_bsr_read1(bs);              // copyright_flag       u(1)
    com_bsr_read(bs, 8);            // copyright_id         u(8)
    com_bsr_read1(bs);              // original_or_copy     u(1)
    com_bsr_read(bs, 7);            // reserved_bits        r(7)
    com_bsr_read1(bs);              //  marker_bit          f(1)
    com_bsr_read(bs, 20);           // copyright_number_1   u(20)
    com_bsr_read1(bs);              // marker_bit           f(1)
    com_bsr_read(bs, 22);           // copyright_number_2   u(22)
    com_bsr_read1(bs);              // marker_bit           f(1)
    com_bsr_read(bs, 22);           // copyright_number_3   u(22)
    return 0;
}

int mastering_display_and_content_metadata_extension(COM_BSR * bs)
{
    for (int c = 0; c < 3; c++)
    {
        com_bsr_read(bs, 16);       // display_primaries_x[c]           u(16)
        com_bsr_read1(bs);          // marker_bit                       f(1)
        com_bsr_read(bs, 16);       // display_primaries_y[c]           u(16)
        com_bsr_read1(bs);          // marker_bit                       f(1)
    }
    com_bsr_read(bs, 16);           // white_point_x                    u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // white_point_y                    u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // max_display_mastering_luminance  u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // min_display_mastering_luminance  u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // max_content_light_level          u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // max_picture_average_light_level  u(16)
    com_bsr_read1(bs);              // marker_bit                       f(1)
    com_bsr_read(bs, 16);           // reserved_bits                    r(16)
    return 0;
}

int camera_parameters_extension(COM_BSR * bs)
{
    com_bsr_read1(bs);              // reserved_bits            r(1)
    com_bsr_read(bs, 7);            // camera_id                u(7)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // height_of_image_device   u(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // focal_length             u(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // f_number                 u(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // vertical_angle_of_view   u(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_x_upper  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_x_lower  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_y_upper  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_y_lower  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_z_upper  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // camera_position_z_lower  i(16)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // camera_direction_x       i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // camera_direction_y       i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // camera_direction_z       i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // image_plane_vertical_x   i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // image_plane_vertical_y   i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 22);           // image_plane_vertical_z   i(22)
    com_bsr_read1(bs);              // marker_bit               f(1)
    com_bsr_read(bs, 16);           // reserved_bits            r(16)
    return 0;
}

#if CRR_EXTENSION_DATA
int cross_random_access_point_referencing_extension(COM_BSR * bs)
{
    unsigned int crr_lib_number;
    crr_lib_number = com_bsr_read(bs, 3);          // crr_lib_number           u(3)
    com_bsr_read1(bs);                             // marker_bit               f(1)
    unsigned int i = 0;
    while (i < crr_lib_number)
    {
        com_bsr_read(bs, 9);                       // camera_id                u(9)
        i++;
        if (i % 2 == 0)
        {
            com_bsr_read1(bs);                     // marker_bit               f(1)
        }
    }

    assert(com_bsr_read1(bs) == 1);
    while (!COM_BSR_IS_BYTE_ALIGN(bs))
    {
        assert(com_bsr_read1(bs) == 0);
    }
    while (com_bsr_next(bs, 24) != 0x1)
    {
        assert(com_bsr_read(bs, 8) == 0);
    }
    return 0;
}
#endif

int roi_parameters_extension(COM_BSR * bs, int slice_type)
{
    int current_picture_roi_num = com_bsr_read(bs, 8);  // current_picture_roi_num  u(8)
    int roiIndex = 0;
    if (slice_type != SLICE_I)
    {
        int PrevPictureROINum = com_bsr_read(bs, 8);    // prev_picture_roi_num     u(8)
        for (int i = 0; i < PrevPictureROINum; i++)
        {
            int roi_skip_run = com_bsr_read_ue(bs);
            if (roi_skip_run != 0)
            {
                for (int j = 0; j < roi_skip_run; j++)
                {
                    int skip_roi_mode = com_bsr_read1(bs);
                    if (j % 22 == 0)
                        com_bsr_read1(bs);              // marker_bit               f(1)
                }
                com_bsr_read1(bs);
            }
            else
            {
                com_bsr_read_se(bs);                    // roi_axisx_delta          se(v)
                com_bsr_read1(bs);                      // marker_bit               f(1)
                com_bsr_read_se(bs);                    // roi_axisy_delta          se(v)
                com_bsr_read1(bs);                      // marker_bit               f(1)
                com_bsr_read_se(bs);                    // roi_width_delta          se(v)
                com_bsr_read1(bs);                      // marker_bit               f(1)
                com_bsr_read_se(bs);                    // roi_height_delta         se(v)
                com_bsr_read1(bs);                      // marker_bit               f(1)

            }
        }
    }
    for (int i = roiIndex; i < current_picture_roi_num; i++)
    {
        com_bsr_read(bs, 6);                            // roi_axisx                u(6)
        com_bsr_read1(bs);                              // marker_bit               f(1)
        com_bsr_read(bs, 6);                            // roi_axisy                u(6)
        com_bsr_read1(bs);                              // marker_bit               f(1)
        com_bsr_read(bs, 6);                            // roi_width                u(6)
        com_bsr_read1(bs);                              // marker_bit               f(1)
        com_bsr_read(bs, 6);                            // roi_height               u(6)
        com_bsr_read1(bs);                              // marker_bit               f(1)

    }
    return 0;
}

int picture_display_extension( COM_BSR * bs, COM_SQH* sqh, COM_PIC_HEADER* pic_header )
{
    int num_frame_center_offsets;
    if( sqh->progressive_sequence == 1 )
    {
        if( pic_header->repeat_first_field == 1 )
        {
            num_frame_center_offsets = pic_header->top_field_first == 1 ? 3 : 2;
        }
        else
            num_frame_center_offsets = 1;
    }
    else
    {
        if( pic_header->picture_structure == 0 )
        {
            num_frame_center_offsets = 1;
        }
        else
        {
            num_frame_center_offsets = pic_header->repeat_first_field == 1 ? 3 : 2;
        }
    }

    for( int i = 0; i < num_frame_center_offsets; i++ )
    {
        com_bsr_read( bs, 16 );           // picture_centre_horizontal_offset i(16)
        com_bsr_read1( bs );              // marker_bit                       f(1)
        com_bsr_read( bs, 16 );           // picture_centre_vertical_offset   i(16)
        com_bsr_read1( bs );              // marker_bit                       f(1)
    }

    return 0;
}

int extension_data(COM_BSR * bs, int i, COM_SQH* sqh, COM_PIC_HEADER* pic_header)
{
    while (com_bsr_next(bs, 32) == 0x1B5)
    {
        com_bsr_read(bs, 32);        // extension_start_code            f(32)
        if (i == 0)
        {
            int ret = com_bsr_read(bs, 4);
            if (ret == 2)
            {
                sequence_display_extension(bs);
            }
            else if (ret == 3)
            {
                temporal_scalability_extension(bs);
            }
            else if (ret == 4)
            {
                copyright_extension(bs);
            }
            else if (ret == 0xa)
            {
                mastering_display_and_content_metadata_extension(bs);
            }
            else if (ret == 0xb)
            {
                camera_parameters_extension(bs);
            }
#if CRR_EXTENSION_DATA
            else if (ret == 0xd)
            {
                cross_random_access_point_referencing_extension(bs);
            }
#endif
            else
            {
                while (com_bsr_next(bs, 24) != 1)
                {
                    com_bsr_read(bs, 8);    // reserved_extension_data_byte u(8)
                }
            }
        }
        else
        {
            int ret = com_bsr_read(bs, 4);
            if (ret == 4)
            {
                copyright_extension(bs);
            }
            else if (ret == 7)
            {
                picture_display_extension(bs, sqh, pic_header);
            }
            else if (ret == 0xb)
            {
                camera_parameters_extension(bs);
            }
            else if (ret == 0xc)
            {
                roi_parameters_extension(bs, pic_header->slice_type);
            }
            else
            {
                while (com_bsr_next(bs, 24) != 1)
                {
                    com_bsr_read(bs, 8);    // reserved_extension_data_byte u(8)
                }
            }
        }
    }
    return 0;
}

int extension_and_user_data(DEC_CTX * ctx, COM_BSR * bs, int i, COM_SQH * sqh, COM_PIC_HEADER* pic_header)
{
    while ((com_bsr_next(bs, 32) == 0x1B5) || (com_bsr_next(bs, 32) == 0x1B2))
    {
        if (com_bsr_next(bs, 32) == 0x1B5)
        {
            extension_data(bs, i, sqh, pic_header);
        }
        if (com_bsr_next(bs, 32) == 0x1B2)
        {
            user_data(ctx, bs);
        }
    }
    return 0;
}
#endif
