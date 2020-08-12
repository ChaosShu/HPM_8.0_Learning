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

#include "enc_def.h"
#include <math.h>


#define QUANT(c, scale, offset, shift) ((s16)((((c)*(scale)) + (offset)) >> (shift)))

#if USE_RDOQ
static s64 err_scale_tbl[80][MAX_CU_DEPTH]; // [64 + 16][MAX_CU_DEPTH]
#endif

const int quant_scale[80] =   // [64 + 16]
{
    16302, 15024, 13777, 12634, 11626, 10624,  9742,  8958,
    8192,  7512,  6889,  6305,  5793,  5303,  4878,  4467,
    4091,  3756,  3444,  3161,  2894,  2654,  2435,  2235,
    2048,  1878,  1722,  1579,  1449,  1329,  1218,  1117,
    1024,   939,   861,   790,   724,   664,   609,   558,
    512,   470,   430,   395,   362,   332,   304,   279,
    256,   235,   215,   197,   181,   166,   152,   140,
    128,   116,   108,    99,    91,    83,    76,    69,
    64,    59,    54,    49,    45,    41,    38,    35,
    32,    30,    27,    25,    23,    21,    19,    18
};

/******************   DCT-2   ******************************************/

static void tx_dct2_pb2(s16 * src, s16 * dst, int shift, int line)
{
    int j;
    int E, O;
    int add = shift == 0 ? 0 : 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* E and O */
        E = src[j*2+0] + src[j*2+1];
        O = src[j*2+0] - src[j*2+1];
        dst[0*line+j] = (s16)((com_tbl_tm2[DCT2][0][0]*E + add)>>shift);
        dst[1*line+j] = (s16)((com_tbl_tm2[DCT2][1][0]*O + add)>>shift);
    }
}

static void tx_dct2_pb4(s16 * src, s16 * dst, int shift, int line)
{
    int j;
    int E[2], O[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* E and O */
        E[0] = src[j*4+0] + src[j*4+3];
        O[0] = src[j*4+0] - src[j*4+3];
        E[1] = src[j*4+1] + src[j*4+2];
        O[1] = src[j*4+1] - src[j*4+2];
        dst[0*line+j] = (s16)((com_tbl_tm4[DCT2][0][0]*E[0] + com_tbl_tm4[DCT2][0][1]*E[1] + add)>>shift);
        dst[2*line+j] = (s16)((com_tbl_tm4[DCT2][2][0]*E[0] + com_tbl_tm4[DCT2][2][1]*E[1] + add)>>shift);
        dst[1*line+j] = (s16)((com_tbl_tm4[DCT2][1][0]*O[0] + com_tbl_tm4[DCT2][1][1]*O[1] + add)>>shift);
        dst[3*line+j] = (s16)((com_tbl_tm4[DCT2][3][0]*O[0] + com_tbl_tm4[DCT2][3][1]*O[1] + add)>>shift);
    }
}

static void tx_dct2_pb8(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* E and O*/
        for(k = 0; k < 4; k++)
        {
            E[k] = src[j*8+k] + src[j*8+7-k];
            O[k] = src[j*8+k] - src[j*8+7-k];
        }
        /* EE and EO */
        EE[0] = E[0] + E[3];
        EO[0] = E[0] - E[3];
        EE[1] = E[1] + E[2];
        EO[1] = E[1] - E[2];
        dst[0*line+j] = (s16)((com_tbl_tm8[DCT2][0][0]*EE[0] + com_tbl_tm8[DCT2][0][1]*EE[1] + add)>>shift);
        dst[4*line+j] = (s16)((com_tbl_tm8[DCT2][4][0]*EE[0] + com_tbl_tm8[DCT2][4][1]*EE[1] + add)>>shift);
        dst[2*line+j] = (s16)((com_tbl_tm8[DCT2][2][0]*EO[0] + com_tbl_tm8[DCT2][2][1]*EO[1] + add)>>shift);
        dst[6*line+j] = (s16)((com_tbl_tm8[DCT2][6][0]*EO[0] + com_tbl_tm8[DCT2][6][1]*EO[1] + add)>>shift);
        dst[1*line+j] = (s16)((com_tbl_tm8[DCT2][1][0]*O[0] + com_tbl_tm8[DCT2][1][1]*O[1] + com_tbl_tm8[DCT2][1][2]*O[2] + com_tbl_tm8[DCT2][1][3]*O[3] + add)>>shift);
        dst[3*line+j] = (s16)((com_tbl_tm8[DCT2][3][0]*O[0] + com_tbl_tm8[DCT2][3][1]*O[1] + com_tbl_tm8[DCT2][3][2]*O[2] + com_tbl_tm8[DCT2][3][3]*O[3] + add)>>shift);
        dst[5*line+j] = (s16)((com_tbl_tm8[DCT2][5][0]*O[0] + com_tbl_tm8[DCT2][5][1]*O[1] + com_tbl_tm8[DCT2][5][2]*O[2] + com_tbl_tm8[DCT2][5][3]*O[3] + add)>>shift);
        dst[7*line+j] = (s16)((com_tbl_tm8[DCT2][7][0]*O[0] + com_tbl_tm8[DCT2][7][1]*O[1] + com_tbl_tm8[DCT2][7][2]*O[2] + com_tbl_tm8[DCT2][7][3]*O[3] + add)>>shift);
    }
}

static void tx_dct2_pb16(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* E and O*/
        for(k = 0; k < 8; k++)
        {
            E[k] = src[j*16+k] + src[j*16+15-k];
            O[k] = src[j*16+k] - src[j*16+15-k];
        }
        /* EE and EO */
        for(k = 0; k < 4; k++)
        {
            EE[k] = E[k] + E[7-k];
            EO[k] = E[k] - E[7-k];
        }
        /* EEE and EEO */
        EEE[0] = EE[0] + EE[3];
        EEO[0] = EE[0] - EE[3];
        EEE[1] = EE[1] + EE[2];
        EEO[1] = EE[1] - EE[2];
        dst[ 0*line+j] = (s16)((com_tbl_tm16[DCT2][ 0][0]*EEE[0] + com_tbl_tm16[DCT2][ 0][1]*EEE[1] + add)>>shift);
        dst[ 8*line+j] = (s16)((com_tbl_tm16[DCT2][ 8][0]*EEE[0] + com_tbl_tm16[DCT2][ 8][1]*EEE[1] + add)>>shift);
        dst[ 4*line+j] = (s16)((com_tbl_tm16[DCT2][ 4][0]*EEO[0] + com_tbl_tm16[DCT2][ 4][1]*EEO[1] + add)>>shift);
        dst[12*line+j] = (s16)((com_tbl_tm16[DCT2][12][0]*EEO[0] + com_tbl_tm16[DCT2][12][1]*EEO[1] + add)>>shift);
        for(k = 2; k < 16; k += 4)
        {
            dst[k*line + j] = (s16)((com_tbl_tm16[DCT2][k][0] * EO[0] + com_tbl_tm16[DCT2][k][1] * EO[1] + com_tbl_tm16[DCT2][k][2] * EO[2] + com_tbl_tm16[DCT2][k][3] * EO[3] + add) >> shift);
        }
        for(k = 1; k < 16; k += 2)
        {
            dst[k*line + j] = (s16)((com_tbl_tm16[DCT2][k][0] * O[0] + com_tbl_tm16[DCT2][k][1] * O[1] + com_tbl_tm16[DCT2][k][2] * O[2] + com_tbl_tm16[DCT2][k][3] * O[3] +
                                     com_tbl_tm16[DCT2][k][4] * O[4] + com_tbl_tm16[DCT2][k][5] * O[5] + com_tbl_tm16[DCT2][k][6] * O[6] + com_tbl_tm16[DCT2][k][7] * O[7] + add) >> shift);
        }
    }
}

static void tx_dct2_pb32(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        /* E and O*/
        for(k = 0; k < 16; k++)
        {
            E[k] = src[j*32+k] + src[j*32+31-k];
            O[k] = src[j*32+k] - src[j*32+31-k];
        }
        /* EE and EO */
        for(k = 0; k < 8; k++)
        {
            EE[k] = E[k] + E[15-k];
            EO[k] = E[k] - E[15-k];
        }
        /* EEE and EEO */
        for(k = 0; k < 4; k++)
        {
            EEE[k] = EE[k] + EE[7-k];
            EEO[k] = EE[k] - EE[7-k];
        }
        /* EEEE and EEEO */
        EEEE[0] = EEE[0] + EEE[3];
        EEEO[0] = EEE[0] - EEE[3];
        EEEE[1] = EEE[1] + EEE[2];
        EEEO[1] = EEE[1] - EEE[2];
        dst[ 0*line+j] = (s16)((com_tbl_tm32[DCT2][ 0][0]*EEEE[0] + com_tbl_tm32[DCT2][ 0][1]*EEEE[1] + add)>>shift);
        dst[16*line+j] = (s16)((com_tbl_tm32[DCT2][16][0]*EEEE[0] + com_tbl_tm32[DCT2][16][1]*EEEE[1] + add)>>shift);
        dst[ 8*line+j] = (s16)((com_tbl_tm32[DCT2][ 8][0]*EEEO[0] + com_tbl_tm32[DCT2][ 8][1]*EEEO[1] + add)>>shift);
        dst[24*line+j] = (s16)((com_tbl_tm32[DCT2][24][0]*EEEO[0] + com_tbl_tm32[DCT2][24][1]*EEEO[1] + add)>>shift);
        for(k = 4; k < 32; k += 8)
        {
            dst[k*line + j] = (s16)((com_tbl_tm32[DCT2][k][0] * EEO[0] + com_tbl_tm32[DCT2][k][1] * EEO[1] + com_tbl_tm32[DCT2][k][2] * EEO[2] + com_tbl_tm32[DCT2][k][3] * EEO[3] + add) >> shift);
        }
        for(k = 2; k < 32; k += 4)
        {
            dst[k*line + j] = (s16)((com_tbl_tm32[DCT2][k][0] * EO[0] + com_tbl_tm32[DCT2][k][1] * EO[1] + com_tbl_tm32[DCT2][k][2] * EO[2] + com_tbl_tm32[DCT2][k][3] * EO[3] +
                                     com_tbl_tm32[DCT2][k][4] * EO[4] + com_tbl_tm32[DCT2][k][5] * EO[5] + com_tbl_tm32[DCT2][k][6] * EO[6] + com_tbl_tm32[DCT2][k][7] * EO[7] + add) >> shift);
        }
        for(k = 1; k < 32; k += 2)
        {
            dst[k*line + j] = (s16)((com_tbl_tm32[DCT2][k][0] * O[0] + com_tbl_tm32[DCT2][k][1] * O[1] + com_tbl_tm32[DCT2][k][2] * O[2] + com_tbl_tm32[DCT2][k][3] * O[3] +
                                     com_tbl_tm32[DCT2][k][4] * O[4] + com_tbl_tm32[DCT2][k][5] * O[5] + com_tbl_tm32[DCT2][k][6] * O[6] + com_tbl_tm32[DCT2][k][7] * O[7] +
                                     com_tbl_tm32[DCT2][k][8] * O[8] + com_tbl_tm32[DCT2][k][9] * O[9] + com_tbl_tm32[DCT2][k][10] * O[10] + com_tbl_tm32[DCT2][k][11] * O[11] +
                                     com_tbl_tm32[DCT2][k][12] * O[12] + com_tbl_tm32[DCT2][k][13] * O[13] + com_tbl_tm32[DCT2][k][14] * O[14] + com_tbl_tm32[DCT2][k][15] * O[15] + add) >> shift);
        }
    }
}

static void tx_dct2_pb64(s16 *src, s16 *dst, int shift, int line)
{
    const int tx_size = 64;
    const s8 * tm = com_tbl_tm64[DCT2][0];
    int j, k;
    int E[32], O[32];
    int EE[16], EO[16];
    int EEE[8], EEO[8];
    int EEEE[4], EEEO[4];
    int EEEEE[2], EEEEO[2];
    int add = 1 << (shift - 1);
    for(j = 0; j < line; j++)
    {
        for(k = 0; k < 32; k++)
        {
            E[k] = src[k] + src[63-k];
            O[k] = src[k] - src[63-k];
        }
        for (k=0; k<16; k++)
        {
            EE[k] = E[k] + E[31-k];
            EO[k] = E[k] - E[31-k];
        }
        for (k=0; k<8; k++)
        {
            EEE[k] = EE[k] + EE[15-k];
            EEO[k] = EE[k] - EE[15-k];
        }
        for (k=0; k<4; k++)
        {
            EEEE[k] = EEE[k] + EEE[7-k];
            EEEO[k] = EEE[k] - EEE[7-k];
        }
        EEEEE[0] = EEEE[0] + EEEE[3];
        EEEEO[0] = EEEE[0] - EEEE[3];
        EEEEE[1] = EEEE[1] + EEEE[2];
        EEEEO[1] = EEEE[1] - EEEE[2];
        dst[ 0       ] = (s16)((tm[ 0*64+0]*EEEEE[0] + tm[ 0*64+1]*EEEEE[1] + add)>>shift);
        dst[ 16*line ] = (s16)((tm[16*64+0]*EEEEO[0] + tm[16*64+1]*EEEEO[1] + add)>>shift);
        dst[ 32*line ] = (s16)((tm[32*64+0]*EEEEE[0] + tm[32*64+1]*EEEEE[1] + add)>>shift);
        dst[ 48*line ] = (s16)((tm[48*64+0]*EEEEO[0] + tm[48*64+1]*EEEEO[1] + add)>>shift);
        for (k=8; k<64; k+=16)
        {
            dst[ k*line ] = (s16)((tm[k*64+0]*EEEO[0] + tm[k*64+1]*EEEO[1] + tm[k*64+2]*EEEO[2] + tm[k*64+3]*EEEO[3] + add)>>shift);
        }
        for (k=4; k<64; k+=8)
        {
            dst[ k*line ] = (s16)((tm[k*64+0]*EEO[0] + tm[k*64+1]*EEO[1] + tm[k*64+2]*EEO[2] + tm[k*64+3]*EEO[3] +
                                   tm[k*64+4]*EEO[4] + tm[k*64+5]*EEO[5] + tm[k*64+6]*EEO[6] + tm[k*64+7]*EEO[7] + add)>>shift);
        }
        for (k=2; k<64; k+=4)
        {
            dst[ k*line ] = (s16)((tm[k*64+ 0]*EO[ 0] + tm[k*64+ 1]*EO[ 1] + tm[k*64+ 2]*EO[ 2] + tm[k*64+ 3]*EO[ 3] +
                                   tm[k*64+ 4]*EO[ 4] + tm[k*64+ 5]*EO[ 5] + tm[k*64+ 6]*EO[ 6] + tm[k*64+ 7]*EO[ 7] +
                                   tm[k*64+ 8]*EO[ 8] + tm[k*64+ 9]*EO[ 9] + tm[k*64+10]*EO[10] + tm[k*64+11]*EO[11] +
                                   tm[k*64+12]*EO[12] + tm[k*64+13]*EO[13] + tm[k*64+14]*EO[14] + tm[k*64+15]*EO[15] + add)>>shift);
        }
        for (k=1; k<64; k+=2)
        {
            dst[ k*line ] = (s16)((tm[k*64+ 0]*O[ 0] + tm[k*64+ 1]*O[ 1] + tm[k*64+ 2]*O[ 2] + tm[k*64+ 3]*O[ 3] +
                                   tm[k*64+ 4]*O[ 4] + tm[k*64+ 5]*O[ 5] + tm[k*64+ 6]*O[ 6] + tm[k*64+ 7]*O[ 7] +
                                   tm[k*64+ 8]*O[ 8] + tm[k*64+ 9]*O[ 9] + tm[k*64+10]*O[10] + tm[k*64+11]*O[11] +
                                   tm[k*64+12]*O[12] + tm[k*64+13]*O[13] + tm[k*64+14]*O[14] + tm[k*64+15]*O[15] +
                                   tm[k*64+16]*O[16] + tm[k*64+17]*O[17] + tm[k*64+18]*O[18] + tm[k*64+19]*O[19] +
                                   tm[k*64+20]*O[20] + tm[k*64+21]*O[21] + tm[k*64+22]*O[22] + tm[k*64+23]*O[23] +
                                   tm[k*64+24]*O[24] + tm[k*64+25]*O[25] + tm[k*64+26]*O[26] + tm[k*64+27]*O[27] +
                                   tm[k*64+28]*O[28] + tm[k*64+29]*O[29] + tm[k*64+30]*O[30] + tm[k*64+31]*O[31] + add)>>shift);
        }
        src += tx_size;
        dst ++;
    }
}

/******************   DCT-8   ******************************************/

static void tx_dct8_pb4(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i;
    int rnd_factor = 1 << (shift - 1);

    s8 *iT = com_tbl_tm4[DCT8][0];

    int c[4];
    const int  reducedLine = line;
    for (i = 0; i<reducedLine; i++)
    {
        // Intermediate Variables
        c[0] = src[0] + src[3];
        c[1] = src[2] + src[0];
        c[2] = src[3] - src[2];
        c[3] = iT[1] * src[1];

        dst[0 * line] = (s16)((iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift);
        dst[1 * line] = (s16)((iT[1] * (src[0] - src[2] - src[3]) + rnd_factor) >> shift);
        dst[2 * line] = (s16)((iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift);
        dst[3 * line] = (s16)((iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift);

        src += 4;
        dst++;
    }
}

static void tx_dct8_pb8(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 8;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm8[DCT8][0];

        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            pCoef += line;
            iT += uiTrSize;
        }
        src += uiTrSize;
    }
}

static void tx_dct8_pb16(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 16;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm16[DCT8][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            pCoef += line;
            iT += uiTrSize;
        }
        src += uiTrSize;
    }
}

static void tx_dct8_pb32(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 32;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm32[DCT8][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            iT += uiTrSize;
            pCoef += line;
        }
        src += uiTrSize;
    }
}

static void tx_dct8_pb64(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 64;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm64[DCT8][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            iT += uiTrSize;
            pCoef += line;
        }
        src += uiTrSize;
    }
}

/******************   DST-7   ******************************************/

static void tx_dst7_pb4(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i;
    int rnd_factor = 1 << (shift - 1);

    s8 *iT = com_tbl_tm4[DST7][0];

    int c[4];
    const int  reducedLine = line;
    for (i = 0; i<reducedLine; i++)
    {
        // Intermediate Variables
        c[0] = src[0] + src[3];
        c[1] = src[1] + src[3];
        c[2] = src[0] - src[1];
        c[3] = iT[2] * src[2];

        dst[0 * line] = (s16)((iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
        dst[1 * line] = (s16)((iT[2] * (src[0] + src[1] - src[3]) + rnd_factor) >> shift);
        dst[2 * line] = (s16)((iT[0] * c[2] + iT[1] * c[0] - c[3] + rnd_factor) >> shift);
        dst[3 * line] = (s16)((iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);

        src += 4;
        dst++;
    }
}

static void tx_dst7_pb8(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 8;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm8[DST7][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            pCoef += line;
            iT += uiTrSize;
        }
        src += uiTrSize;
    }
}

static void tx_dst7_pb16(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 16;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm16[DST7][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            pCoef += line;
            iT += uiTrSize;
        }
        src += uiTrSize;
    }
}

static void tx_dst7_pb32(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 32;
    s8 *iT;
    s16 *pCoef;


    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm32[DST7][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            iT += uiTrSize;
            pCoef += line;
        }
        src += uiTrSize;
    }
}

static void tx_dst7_pb64(s16 *src, s16 *dst, int shift, int line)  // input src, output dst
{
    int i, j, k, iSum;
    int rnd_factor = 1 << (shift - 1);

    const int uiTrSize = 64;
    s8 *iT;
    s16 *pCoef;

    const int  reducedLine = line;
    const int  cutoff = uiTrSize;
    for (i = 0; i<reducedLine; i++)
    {
        pCoef = dst;
        iT = com_tbl_tm64[DST7][0];
        for (j = 0; j<cutoff; j++)
        {
            iSum = 0;
            for (k = 0; k<uiTrSize; k++)
            {
                iSum += src[k] * iT[k];
            }
            pCoef[i] = (s16)((iSum + rnd_factor) >> shift);
            iT += uiTrSize;
            pCoef += line;
        }
        src += uiTrSize;
    }
}


typedef void (*COM_TX)(s16 * coef, s16 * t, int shift, int line);
static COM_TX enc_tbl_tx[NUM_TRANS_TYPE][MAX_TR_LOG2] =
{
    {
        tx_dct2_pb2,
        tx_dct2_pb4,
        tx_dct2_pb8,
        tx_dct2_pb16,
        tx_dct2_pb32,
        tx_dct2_pb64
    },
    {
        NULL,
        tx_dct8_pb4,
        tx_dct8_pb8,
        tx_dct8_pb16,
        tx_dct8_pb32,
        tx_dct8_pb64
    },
    {
        NULL,
        tx_dst7_pb4,
        tx_dst7_pb8,
        tx_dst7_pb16,
        tx_dst7_pb32,
        tx_dst7_pb64
    }
};

static void xCTr_4_1d_Hor(s16 *src, int i_src, s16 *dst, int i_dst, int shift)
{
    int i, j, k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c4_trans[i][k] * tmpSrc[j][k];
            }
            dst[j * i_dst + i] = (s16)COM_CLIP3(-32768, 32767, sum >> shift);
        }
    }
}

static void xCTr_4_1d_Vert(s16 *src, int i_src, s16 *dst, int i_dst, int shift)
{
    int i, j, k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i* i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c4_trans[i][k] * tmpSrc[k][j];
            }
            dst[i* i_dst + j] = (s16)COM_CLIP3(-32768, 32767, sum >> shift);
        }
    }
}

static void xTr2nd_8_1d_Hor(s16 *src, int i_src)
{
    int i, j, k, sum;
    int rnd_factor;
    int tmpSrc[4][4];

    rnd_factor = 1 << (7 - 1);

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c8_trans[i][k] * tmpSrc[j][k];
            }
            src[j* i_src + i] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
        }
    }
}

static void xTr2nd_8_1d_Vert(s16 *src, int i_src)
{
    int i, j, k, sum;
    int rnd_factor;
    int tmpSrc[4][4];

    rnd_factor = 1 << (7 - 1);

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            tmpSrc[i][j] = src[i * i_src + j];
        }
    }
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sum = rnd_factor;
            for (k = 0; k < 4; k++)
            {
                sum += tab_c8_trans[i][k] * tmpSrc[k][j];
            }
            src[i* i_src + j] = (s16)COM_CLIP3(-32768, 32767, sum >> 7);
        }
    }
}


static int com_get_forward_trans_shift(int log2_size, int type, int bit_depth)
{
    assert(log2_size <= 6);
    return (type == 0) ? (log2_size + bit_depth - 10) : (log2_size + 5);
}

void enc_trans(COM_MODE *mode, int plane, int blk_idx, s16 * coef, s16 *resi, int cu_width_log2, int cu_height_log2, int bIntra, int ch_type, int bit_depth, int secT_Ver_Hor, int use_alt4x4Trans)
{
    s16 coef_temp[MAX_TR_DIM];

    if((cu_width_log2 > MAX_TR_LOG2) || (cu_height_log2 > MAX_TR_LOG2))
    {
        assert(0);
    }
    else
    {
        int shift1 = com_get_forward_trans_shift(cu_width_log2, 0, bit_depth);
        int shift2 = com_get_forward_trans_shift(cu_height_log2, 1, bit_depth);
        int stride_tu = (1 << cu_width_log2);
#if IST
        if (plane == Y_C && mode->ist_tu_flag && mode->cu_mode == MODE_INTRA
#if ISTS
            && mode->ph_ists_enable_flag == 0
#endif
            )
        {
            int nTrIdxHor = DST7;
            int nTrIdxVer = DST7;
            enc_tbl_tx[nTrIdxHor][cu_width_log2 - 1](resi, coef_temp, shift1, 1 << cu_height_log2);
            enc_tbl_tx[nTrIdxVer][cu_height_log2 - 1](coef_temp, coef, shift2, 1 << cu_width_log2);
            return;
        }
#endif
#if ISTS
        if (plane == Y_C && mode->ist_tu_flag
            && mode->ph_ists_enable_flag && (mode->cu_mode == MODE_INTRA || mode->cu_mode == MODE_IBC))
        {
            int shift = 15 - bit_depth - ((cu_width_log2 + cu_height_log2) >> 1);
            int stride_tu = (1 << cu_width_log2);
            int height_tu = (1 << cu_height_log2);
            if (shift >= 0)
            {
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        coef[i * stride_tu + j] = (s16)(resi[i * stride_tu + j] << shift);
                    }
                }
            }
            else
            {
                shift = -shift;
                int rnd_factor = 1 << (shift - 1);
                for (int i = 0; i < height_tu; i++)
                {
                    for (int j = 0; j < stride_tu; j++)
                    {
                        coef[i * stride_tu + j] = (s16)((resi[i * stride_tu + j] + rnd_factor) >> shift);
                    }
                }
            }
            return;
        }
#endif
        if (use_alt4x4Trans && cu_width_log2 == 2 && cu_height_log2 == 2)
        {
            s16 coef_temp2[16];
            assert(bIntra);
            xCTr_4_1d_Hor(resi, 4, coef_temp2, 4, shift1+1);
            xCTr_4_1d_Vert(coef_temp2, 4, coef, 4, shift2+1);
        }
        else
        {
            int nTrIdxHor = DCT2, nTrIdxVer = DCT2;

            if (plane == Y_C && mode->tb_part == SIZE_NxN)
            {
                nTrIdxHor = com_tbl_subset_inter[blk_idx &  1];
                nTrIdxVer = com_tbl_subset_inter[blk_idx >> 1];
            }
#if SBT
            if( plane == Y_C && mode->sbt_info )
            {
                nTrIdxHor = mode->sbt_hor_trans;
                nTrIdxVer = mode->sbt_ver_trans;
            }
#endif

            enc_tbl_tx[nTrIdxHor][cu_width_log2 - 1](resi, coef_temp, shift1, 1 << cu_height_log2);
            enc_tbl_tx[nTrIdxVer][cu_height_log2 - 1](coef_temp, coef, shift2, 1 << cu_width_log2);

            if (secT_Ver_Hor >> 1)
            {
                assert(bIntra);
                xTr2nd_8_1d_Vert(coef, stride_tu);
            }
            if (secT_Ver_Hor & 1)
            {
                assert(bIntra);
                xTr2nd_8_1d_Hor(coef, stride_tu);
            }
        }
    }
}

static int get_transform_shift(const int bit_depth, const int tr_size_log2)
{
    return MAX_TX_DYNAMIC_RANGE - bit_depth - tr_size_log2;
}

#if USE_RDOQ
void enc_init_err_scale(int bit_depth)
{
    int qp;
    int i;
    for (qp = 0; qp < 80; qp++)
    {
        int q_value = quant_scale[qp];
        for (i = 0; i < MAX_CU_DEPTH; i++)
        {
            int tr_shift = get_transform_shift(bit_depth, i + 1);
            double err_scale = pow(2.0, SCALE_BITS -tr_shift);
            err_scale = err_scale / q_value / (1 << ((bit_depth - 8)));
            err_scale_tbl[qp][i] = (s64)(err_scale * (double)(1 << ERR_SCALE_PRECISION_BITS));
        }
    }
    return;
}

#define GET_I_COST(rate, lamba)  (rate*lamba)
#define GET_IEP_RATE             (32768)

int get_bits_0_order_exp_golomb(int value)
{
    for (int i = 0; i < 12; i++)
    {
        if (value <= (1 << (i + 1)) - 2)
        {
            return i * 2 + 1;
        }
    }
    return 25; //maybe should not happen
}

static __inline s64 get_ic_rate_cost_rl(u32 abs_level, u32 run, s32 ctx_run, u32 ctx_level, s64 lambda, u32 prev_level, int last_pos)
{
    s32 rate;
    if (abs_level == 0)
    {
        rate = 0;
        if (run == 0)
        {
            rate += rdoq_est_run[ctx_run][0];
        }
        else
        {
            rate += rdoq_est_run[ctx_run + 1][0];
        }
    }
    else
    {
        rate = GET_IEP_RATE; // sign of coeff

        if (!last_pos)
        {
            if (run == 0)
            {
                rate += rdoq_est_run[ctx_run][1];
            }
            else
            {
                rate += rdoq_est_run[ctx_run + 1][1];
            }
        }
        if (abs_level == 1)
        {
            rate += rdoq_est_level[ctx_level][1];
        }
        else if (abs_level >= 9)
        {
            rate += rdoq_est_level[ctx_level][0];
            rate += rdoq_est_level[ctx_level + 1][0] * 7;
            //get bits of exp-golomb
            rate += GET_IEP_RATE * get_bits_0_order_exp_golomb(abs_level - 9);
        }
        else
        {
            rate += rdoq_est_level[ctx_level][0];
            rate += rdoq_est_level[ctx_level + 1][0] * (s32)(abs_level - 2);
            rate += rdoq_est_level[ctx_level + 1][1];
        }
    }
    return (s64)GET_I_COST(rate, lambda);
}

static __inline u32 get_coded_level_rl(s64* rd64_uncoded_cost, s64* rd64_coded_cost, s64 level_double, u32 max_abs_level,
                                       u32 run, u16 ctx_run, u16 ctx_level, s32 q_bits, s64 err_scale, s64 lambda, u32 prev_level, int last_pos)
{
    u32 best_abs_level = 0;
    s64 err1 = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
    u32 min_abs_level;
    u32 abs_level;
    *rd64_uncoded_cost = err1 * err1;
    *rd64_coded_cost = *rd64_uncoded_cost + get_ic_rate_cost_rl(0, run, ctx_run, ctx_level, lambda, prev_level, last_pos);
    min_abs_level = (max_abs_level > 1 ? max_abs_level - 1 : 1);
    for (abs_level = max_abs_level; abs_level >= min_abs_level; abs_level--)
    {
        s64 i64Delta = level_double - ((s64)abs_level << q_bits);
        s64 err = (i64Delta * err_scale) >> ERR_SCALE_PRECISION_BITS;
        s64 dCurrCost = err * err + get_ic_rate_cost_rl(abs_level, run, ctx_run, ctx_level, lambda, prev_level, last_pos);

        if (dCurrCost < *rd64_coded_cost)
        {
            best_abs_level = abs_level;
            *rd64_coded_cost = dCurrCost;
        }
    }
    return best_abs_level;
}
#endif

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

#if USE_RDOQ
int enc_rdoq_run_length_cc(int qp, double d_lambda, int is_intra, s16 *src_coef, s16 *dst_tmp, int cu_width_log2, int cu_height_log2, int ch_type, int bit_depth)
{
    const int scale = quant_scale[qp];
    const int ns_shift = ((cu_width_log2 + cu_height_log2) & 1) ? 7 : 0;
    const int ns_scale = ((cu_width_log2 + cu_height_log2) & 1) ? 181 : 1;
    const int ns_offset = ((cu_width_log2 + cu_height_log2) & 1) ? (1 << (ns_shift - 1)) : 0;
    const int q_value = (scale * ns_scale + ns_offset) >> ns_shift;
    const int log2_size = (cu_width_log2 + cu_height_log2) >> 1;
    const int tr_shift = get_transform_shift(bit_depth, log2_size);
    const u32 max_num_coef = 1 << (cu_width_log2 + cu_height_log2);
    const u16 *scan = com_scan_tbl[COEF_SCAN_ZIGZAG][cu_width_log2 - 1][cu_height_log2 - 1];
    const int ctx_last = (ch_type == Y_C) ? 0 : 1;
    const int q_bits = QUANT_SHIFT + tr_shift;
    int num_nz_coef = 0;
    int sum_all = 0;
    u32 scan_pos;
    u32 run;
    u32 prev_level;
    s32 ctx_qt_cbf;
    u32 best_last_idx_p1 = 0;
    s16 tmp_coef[MAX_TR_DIM];
    s64 tmp_level_double[MAX_TR_DIM];
    s16 tmp_dst_coef[MAX_TR_DIM];
    const s64 lambda = (s64)(d_lambda * (double)(1 << SCALE_BITS) + 0.5);
    s64 err_scale = err_scale_tbl[qp][log2_size - 1];
    s64 d64_best_cost = 0;
    s64 d64_base_cost = 0;
    s64 d64_coded_cost = 0;
    s64 d64_uncoded_cost = 0;
    s64 d64_block_uncoded_cost = 0;
    s64 err;

    /* ===== quantization ===== */
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        s64 level_double = src_coef[blk_pos];
        u32 max_abs_level;
        s8 lower_int;
        s64 temp_level;
        temp_level = ((s64)COM_ABS(src_coef[blk_pos]) * (s64)q_value);
        level_double = (int)COM_MIN(((s64)temp_level), (s64)COM_INT32_MAX - (s64)(1 << (q_bits - 1)));
        tmp_level_double[blk_pos] = level_double;
        max_abs_level = (u32)(level_double >> q_bits);
        lower_int = ((level_double - ((s64)max_abs_level << q_bits)) < (s64)(1 << (q_bits - 1))) ? 1 : 0;
        if (!lower_int)
        {
            max_abs_level++;
        }
        err = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
        d64_block_uncoded_cost += err * err;
        tmp_coef[blk_pos] = src_coef[blk_pos] > 0 ? (s16)max_abs_level : -(s16)(max_abs_level);
        sum_all += max_abs_level;
    }
    com_mset(dst_tmp, 0, sizeof(s16)*max_num_coef);
    if (sum_all == 0)
    {
        return num_nz_coef;
    }
    if (!is_intra && ch_type == Y_C)
    {
        d64_best_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_ctp_zero_flag[1], lambda);
        d64_base_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_ctp_zero_flag[0], lambda);
    }
    else
    {
        ctx_qt_cbf = ch_type;
        d64_best_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][0], lambda);
        d64_base_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][1], lambda);
    }
    run = 0;
    prev_level = 6;
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        u32 level;

        int ctx_run = ((COM_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);
        int ctx_level = ((COM_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);

        level = get_coded_level_rl(&d64_uncoded_cost, &d64_coded_cost, tmp_level_double[blk_pos], COM_ABS(tmp_coef[blk_pos]),
                                   run, (u16)ctx_run, (u16)ctx_level, q_bits, err_scale, lambda, prev_level, scan_pos == max_num_coef - 1);
        tmp_dst_coef[blk_pos] = (s16)(tmp_coef[blk_pos] < 0 ? -(s32)(level) : level);
        d64_base_cost -= d64_uncoded_cost;
        d64_base_cost += d64_coded_cost;
        if (level)
        {
            /* ----- check for last flag ----- */
            s64 d64_cost_last_zero = GET_I_COST(rdoq_est_last[ctx_last][COM_MIN(prev_level - 1, 5)][ace_get_log2(scan_pos + 1)][0], lambda);
            s64 d64_cost_last_one = GET_I_COST(rdoq_est_last[ctx_last][COM_MIN(prev_level - 1, 5)][ace_get_log2(scan_pos + 1)][1], lambda);
            s64 d64_cur_is_last_cost = d64_base_cost + d64_cost_last_one;


            d64_base_cost += d64_cost_last_zero;
            if (d64_cur_is_last_cost < d64_best_cost)
            {
                d64_best_cost = d64_cur_is_last_cost;
                best_last_idx_p1 = scan_pos + 1;
            }
            run = 0;
            prev_level = level;
        }
        else
        {
            run++;
        }
    }
    /* ===== clean uncoded coeficients ===== */
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        if (scan_pos < best_last_idx_p1)
        {
            if (tmp_dst_coef[blk_pos])
            {
                num_nz_coef++;
            }
        }
        else
        {
            tmp_dst_coef[blk_pos] = 0;
        }
        dst_tmp[blk_pos] = tmp_dst_coef[blk_pos];
    }
    return num_nz_coef;
}

#if SRCC
__inline static s64 get_rate_scanregion(int pos_x, int pos_y, int width, int height, int ch_type, s64 lambda)
{
    int group_idx_x;
    int group_idx_y;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int bin, cnt;
    int rate = 0;
    int offset_x = (ch_type == Y_C ? 0 : NUM_CTX_SCANR_LUMA);
    int offset_y = (ch_type == Y_C ? 0 : NUM_CTX_SCANR_LUMA);

    group_idx_x = g_group_idx[pos_x];
    group_idx_y = g_group_idx[pos_y];

    com_get_ctx_srxy_para(ch_type, width, height, &blk_offset_x, &blk_offset_y, &shift_x, &shift_y);

    // posX
    for (bin = 0; bin < group_idx_x; bin++)
    {
        rate += rdoq_est_scanr_x[offset_x + blk_offset_x + (bin >> shift_x)][1];
    }
    if (group_idx_x < g_group_idx[min(width, 32) - 1])
    {
        rate += rdoq_est_scanr_x[offset_x + blk_offset_x + (bin >> shift_x)][0];
    }

    // posY
    for (bin = 0; bin < group_idx_y; bin++)
    {
        rate += rdoq_est_scanr_y[offset_y + blk_offset_y + (bin >> shift_y)][1];
    }
    if (group_idx_y < g_group_idx[min(height, 32) - 1])
    {
        rate += rdoq_est_scanr_y[offset_y + blk_offset_y + (bin >> shift_y)][0];
    }

    // EP-coded part
    if (group_idx_x > 3)
    {
        cnt = (group_idx_x - 2) >> 1;
        pos_x = pos_x - g_min_in_group[group_idx_x];
        rate += (cnt * GET_IEP_RATE);
    }
    if (group_idx_y > 3)
    {
        cnt = (group_idx_y - 2) >> 1;
        pos_y = pos_y - g_min_in_group[group_idx_y];
        rate += (cnt * GET_IEP_RATE);
    }

    return GET_I_COST(rate, lambda);
}

__inline static s64 get_rate_gt0(int significance, int ctx_gt0, s64 lambda)
{
    s64 rate = rdoq_est_gt0[ctx_gt0][significance];
    return GET_I_COST(rate, lambda);
}

__inline static int get_ic_rate(int abs_level, int ctx_gt1, int ctx_gt2)
{
    int rate = GET_IEP_RATE; // cost of sign bit
    int base_level = 3;
    int rparam = 0;

    if (abs_level >= base_level)
    {
        int symbol = abs_level - base_level;
        int length;

        length = get_bits_0_order_exp_golomb(symbol);
        rate += length << 15;

        rate += rdoq_est_gt1[ctx_gt1][1];
        rate += rdoq_est_gt1[ctx_gt2][1];

    }
    else if (abs_level == 1)
    {
        rate += rdoq_est_gt1[ctx_gt1][0];
    }
    else if (abs_level == 2)
    {
        rate += rdoq_est_gt1[ctx_gt1][1];
        rate += rdoq_est_gt1[ctx_gt2][0];
    }
    else
    {
        rate = 0;
    }

    return  rate;
}

__inline static int get_coded_level(
    s64*    rd_coded_cost,          //< reference to coded cost
    s64*    rd_coded_cost0,         //< reference to cost when coefficient is 0
    s64*    rd_coded_cost_sig,       //< rd_coded_cost_sig reference to cost of significant coefficient
    s64     level_double,           //< reference to unscaled quantized level
    int     max_abs_level,          //< scaled quantized level
    int     ctx_gt0,          //< current ctxInc for coeff_abs_significant_flag
    int     ctx_gt1,          //< current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
    int     ctx_gt2,          //< current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
    int     qbits,                 //< quantization step size
    s64     error_scale,             //< 
    s64     lambda,
    int     bypass_sigmap,
    s64*    rd_coded_cost_max
#if SRCC_RDOQ_OPT
    , int     bypass_sigmap0
#endif
    )
{
    s64 curr_cost_sig = 0;
    s64 curr_cost;
    int best_abs_level = 0;
    int min_abs_level;
    int abs_level;
    int rate = 0;
#if SRCC_RDOQ_OPT
    if (bypass_sigmap0 == 0 && max_abs_level < 3)
    {
        *rd_coded_cost_sig = get_rate_gt0(0, ctx_gt0, lambda);
        *rd_coded_cost = *rd_coded_cost0 + *rd_coded_cost_sig;

        if (max_abs_level == 0)
        {
            return best_abs_level;
        }
    }
    else if (bypass_sigmap0 == 1 && max_abs_level < 3)
    {
        *rd_coded_cost = *rd_coded_cost0;
    }
    else
    {
        *rd_coded_cost = COM_INT64_MAX;
    }
#else
    if (bypass_sigmap == 0 && max_abs_level < 3)
    {
        *rd_coded_cost_sig = get_rate_gt0(0, ctx_gt0, lambda);
        *rd_coded_cost = *rd_coded_cost0 + *rd_coded_cost_sig;

        if (max_abs_level == 0)
        {
            return best_abs_level;
        }
    }
    else
    {
        *rd_coded_cost = COM_INT64_MAX;
    }
#endif

    if (bypass_sigmap == 0)
    {
        curr_cost_sig = get_rate_gt0(1, ctx_gt0, lambda);
    }

    min_abs_level = (max_abs_level > 1 ? max_abs_level - 1 : 1);
    for (abs_level = max_abs_level; abs_level >= min_abs_level; abs_level--)
    {
        s64 err = (s64)(level_double - ((s64)abs_level << qbits));
        rate = get_ic_rate(abs_level, ctx_gt1, ctx_gt2);
        err = (err * error_scale) >> ERR_SCALE_PRECISION_BITS;
        curr_cost = err * err + GET_I_COST(rate, lambda);
        curr_cost += curr_cost_sig;
        if (abs_level == max_abs_level)
        {
            *rd_coded_cost_max = curr_cost;
        }
        if (curr_cost < *rd_coded_cost)
        {
            best_abs_level = abs_level;
            *rd_coded_cost = curr_cost;
            *rd_coded_cost_sig = curr_cost_sig;
        }
    }

    return best_abs_level;
}

int enc_rdoq_srcc(ENC_CTX* ctx, COM_MODE *mode, u8 qp, double d_lambda, u8 is_intra, s16 *src_coef, s16 *dst_tmp, int log2_cuw, int log2_cuh, int ch_type, int bit_depth)
{
    const int ns_shift = ((log2_cuw + log2_cuh) & 1) ? 7 : 0;
    const int ns_scale = ((log2_cuw + log2_cuh) & 1) ? 181 : 1;
    const int q_value = (quant_scale[qp] * ns_scale) >> ns_shift;
    const int log2_size = (log2_cuw + log2_cuh) >> 1;
    const int tr_shift = get_transform_shift(bit_depth, log2_size);
    s64 err_scale = err_scale_tbl[qp][log2_size - 1];
    s64 lambda = (s64)(d_lambda * (double)(1 << SCALE_BITS) + 0.5);
    int q_bits;
    const int width = (1 << log2_cuw);
    const int height = (1 << log2_cuh);
    const int max_num_coef = width * height;
    int scan_type = COEF_SCAN_ZIGZAG;
    int log2_block_size = min(log2_cuw, log2_cuh);
    int *scan = com_scan_sr;
    int scan_pos_last = -1;
    int sr_x = 0, sr_y = 0;
    int sr_x_tmp = 0, sr_y_tmp = 0;
    int sr_width, sr_height;
    int num_coef;
    int ipos;
    int cg_log2_size = LOG2_CG_SIZE;
    int last_scan_set;
    int sub_set;
    int offset1 = (ch_type == Y_C) ? 0 : NUM_CTX_GT1_LUMA;

    s64 cost_base = 0;
    s64 cost_best = 0;
    s64 cbf_cost = 0;
    int abs_sum;
    int nnz = 0;
    
    s64 dcost_block_uncoded = 0;
    s64 pdcost_coeff[MAX_TR_DIM];
    s64 pdcost_sig[MAX_TR_DIM];
    s64 pdcost_coeff0[MAX_TR_DIM];
    static int sig_rate_delta[MAX_TR_DIM];
    static s16 coef_dst[MAX_TR_DIM];
    s64 pdcost[MAX_TR_DIM];
    s16 origin_coeff[MAX_TR_DIM];
    COM_POS_INFO cur_pos_info;

    int sum_all = 0;
    int blk_pos, sx, sy;
    static s64 tmp_level_double[MAX_TR_DIM];
    int num_nz = 0;
    int j, i;
    int is_last_nz = 0;
    
    static int row[MAX_TR_SIZE];
    static int col[MAX_TR_SIZE];
    static int row_tmp[MAX_TR_SIZE];
    static int col_tmp[MAX_TR_SIZE];
    s64 sig_last_cost[MAX_TR_DIM];
    s64 sig_last_cost0[MAX_TR_DIM];
    s64 sig_cost_delta[MAX_TR_DIM];

    com_mset(row, 0, sizeof(int) * height);
    com_mset(col, 0, sizeof(int) * width);

    q_bits = QUANT_SHIFT + tr_shift;

    for (blk_pos = 0; blk_pos < max_num_coef; blk_pos++)
    {
        int max_abs_level;
        s64 err;
        s64 temp_level;
        int level_double;
        temp_level = ((s64)COM_ABS(src_coef[blk_pos]) * (s64)q_value);
        level_double = (int)COM_MIN(((s64)temp_level), (s64)COM_INT32_MAX - (s64)(1 << (q_bits - 1)));
        tmp_level_double[blk_pos] = (s64)level_double;
        max_abs_level = COM_MIN(MAX_TX_VAL, ((level_double + ((int)1 << (q_bits - 1))) >> q_bits));
        origin_coeff[blk_pos] = src_coef[blk_pos] > 0 ? max_abs_level : -max_abs_level;
        err = (s64)level_double;
        err = (err * err_scale) >> ERR_SCALE_PRECISION_BITS;
        pdcost_coeff0[blk_pos] = err * err;
        dcost_block_uncoded += pdcost_coeff0[blk_pos];
        coef_dst[blk_pos] = (s16)max_abs_level;
        sum_all += max_abs_level;

        if (max_abs_level != 0)
        {
            i = blk_pos & (width - 1);
            j = blk_pos >> log2_cuw;
            sr_x = i > sr_x ? i : sr_x;
            sr_y = j > sr_y ? j : sr_y;
            num_nz++;
            row[j]++;
            col[i]++;
        }
    }

    if (sum_all == 0)
    {
        com_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
        return 0;
    }

    sr_width = sr_x + 1;
    sr_height = sr_y + 1;
    num_coef = sr_width * sr_height;
    com_init_scan_sr(scan, sr_width, sr_height, width, scan_type);

    
    last_scan_set = (num_coef - 1) >> cg_log2_size;
    scan_pos_last = sr_y * width + sr_x;
    ipos = num_coef - 1;
    cost_base = dcost_block_uncoded;

    int prev_0val[NUM_PREV_0VAL] = { 0 }; //FIFO
    int prev_12val[NUM_PREV_12VAL] = { 0 }; //FIFO

    for (sub_set = last_scan_set; sub_set >= 0; sub_set--)
    {
        int sub_pos = sub_set << cg_log2_size;

        for (; ipos >= sub_pos; ipos--)
        {
            //===== coefficient level estimation =====
            int  level;
            int  ctx_gt0 = 0;
            int  ctx_gt1 = 0;
            int  ctx_gt2 = 0;

            blk_pos = scan[ipos];
            sx = blk_pos & (width - 1);
            sy = blk_pos >> log2_cuw;
            pdcost[blk_pos] = -1;
            if (sx <= sr_x && sy <= sr_y)
            {
                s64 level_double = tmp_level_double[blk_pos];
                int max_abs_level = coef_dst[blk_pos];
                int bypass_sigmap = (max_abs_level) && ((sx == 0 && sy == sr_y && row[sr_y] == 1) || (sy == 0 && sx == sr_x && col[sr_x] == 1)) ? 1 : 0;
                int offset0 = (ch_type == Y_C) ? (num_coef <= 4 ? 0 : NUM_CTX_GT0_LUMA_TU << (num_coef <= 16 ? 0 : 1)) : NUM_CTX_GT0_LUMA;
#if SRCC_RDOQ_OPT
                int bypass_sigmap0 = (max_abs_level) && ((sy == sr_y && row[sr_y] == 1) || (sx == sr_x && col[sr_x] == 1)) ? 1 : 0;
#endif
                com_init_pos_info(&cur_pos_info, sr_x, sr_y);
                ctx_gt0 = com_get_ctx_gt0_inc(coef_dst, blk_pos, width, height, ch_type, sr_x, sr_y, prev_0val
                    , is_intra, &cur_pos_info);
                ctx_gt1 = com_get_ctx_gt1_inc(coef_dst, blk_pos, width, height, ch_type, sr_x, sr_y, prev_12val
                    , is_intra, &cur_pos_info);
                ctx_gt2 = com_get_ctx_gt2_inc(coef_dst, blk_pos, width, height, ch_type, sr_x, sr_y, prev_12val
                    , is_intra, &cur_pos_info);

                if (blk_pos == scan_pos_last)
                {
                    ctx_gt0 = 0;
                }

                ctx_gt0 += offset0;
                if (max_abs_level != 0 && is_last_nz == 0)
                {
                    ctx_gt1 = 0;
                    ctx_gt2 = 0;
                }
                ctx_gt1 += offset1;
                ctx_gt2 += offset1;

                level = get_coded_level(&pdcost_coeff[blk_pos], &pdcost_coeff0[blk_pos], &pdcost_sig[blk_pos], level_double, max_abs_level, ctx_gt0, ctx_gt1, ctx_gt2,
                    q_bits, err_scale, lambda, bypass_sigmap, &pdcost[blk_pos]
#if SRCC_RDOQ_OPT
                    ,bypass_sigmap0
#endif
                );

                sig_rate_delta[blk_pos] = rdoq_est_gt0[ctx_gt0][1] - rdoq_est_gt0[ctx_gt0][0];
                sig_cost_delta[blk_pos] = GET_I_COST(sig_rate_delta[blk_pos], lambda);
                sig_last_cost[blk_pos] = GET_I_COST(rdoq_est_gt0[offset0][!!(level)], lambda);
                sig_last_cost0[blk_pos] = GET_I_COST(rdoq_est_gt0[offset0][0], lambda);
                coef_dst[blk_pos] = (s16)level;

                for (int i = NUM_PREV_0VAL - 1; i >0; i--)
                {
                    prev_0val[i] = prev_0val[i - 1];
                }
                prev_0val[0] = (level > 0);

                if (level > 0)
                {

                    if (is_last_nz == 0)
                    {
                        is_last_nz = 1;
                    }

                    sr_x_tmp = sx > sr_x_tmp ? sx : sr_x_tmp;
                    sr_y_tmp = sy > sr_y_tmp ? sy : sr_y_tmp;

                    for (int i = NUM_PREV_12VAL - 1; i >0; i--)
                    {
                        prev_12val[i] = prev_12val[i - 1];
                    }
                    prev_12val[0] = level;

                }
                else if (max_abs_level)
                {
                    num_nz--;
                    if (num_nz == 0)
                    {
                        com_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
                        return 0;
                    }

                    row[sy]--;
                    col[sx]--;

                    if (row[sr_y] == 0)
                    {
                        while (row[sr_y] == 0 && sr_y >= 0)
                        {
                            sr_y--;
                        }
                    }

                    if (col[sr_x] == 0)
                    {
                        while (col[sr_x] == 0 && sr_x >= 0)
                        {
                            sr_x--;
                        }
                    }

                    num_coef = (sr_x + 1) * (sr_y + 1);
                    scan_pos_last = sr_y * width + sr_x;
                }
            }
        }
    }

    if (num_nz == 0)
    {
        com_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
        return 0;
    }

    {
        s64 in_sr_cost0 = 0;
        s64 in_sr_cost = 0;
        sr_x = sr_x_tmp;
        sr_y = sr_y_tmp;
        sr_width = sr_x + 1;
        sr_height = sr_y + 1;

        cost_base = 0;

        for (j = 0; j <= sr_y; j++)
        {
            blk_pos = j * width;
            for (i = 0; i <= sr_x; i++)
            {
                in_sr_cost += pdcost_coeff[blk_pos];
                in_sr_cost0 += pdcost_coeff0[blk_pos];
                ++blk_pos;
            }
        }
        // base cost in scan region
        cost_base = dcost_block_uncoded - in_sr_cost0 + in_sr_cost;
    }

    cost_best = 0;

    if (is_intra == 0 && ch_type == Y_C)
    {
        cost_best = dcost_block_uncoded + GET_I_COST(rdoq_est_ctp_zero_flag[1], lambda);
        cbf_cost = GET_I_COST(rdoq_est_ctp_zero_flag[0], lambda);
        cost_base += cbf_cost;
    }
    else
    {
        int ctx_qt_cbf = ch_type;
        cost_best = dcost_block_uncoded + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][0], lambda);
        cbf_cost = GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][1], lambda);
        cost_base += cbf_cost;
    }

    sr_x_tmp = sr_x;
    sr_y_tmp = sr_y;

    com_mcpy(row_tmp, row, sizeof(int) * height);
    com_mcpy(col_tmp, col, sizeof(int) * width);

    {
        int sr_x_best = -1;
        int sr_y_best = -1;
        s64 removal_cost = 0;
        s64 cost_last;
        s64 total_cost;
        s64 total_cost_tmp;
        int k = 0;
        int best_case = 0;
        int best_cutpos = 0;
        //encode only 0,0 coeff
        if (coef_dst[0])
        {
            s64 level_double = tmp_level_double[0];
            s64 err = (s64)(level_double - ((s64)coef_dst[0] << q_bits));
            s64 curr_cost0 = cbf_cost + dcost_block_uncoded - pdcost_coeff0[0];
            s64 cost1 = GET_I_COST(get_ic_rate((int)coef_dst[0], offset1, offset1), lambda);
            err = (err * err_scale) >> ERR_SCALE_PRECISION_BITS;
            curr_cost0 += err * err + cost1;
            curr_cost0 += get_rate_scanregion(0, 0, width, height, ch_type, lambda);
            if (curr_cost0 < cost_best)
            {
                cost_best = curr_cost0;
                sr_x_best = 0;
                sr_y_best = 0;
                best_case = 0;
            }
        }

        while (k < sr_width + sr_height)
        {
            if (sr_x == 0 && sr_y == 0)
            {
                break;
            }
            cost_last = get_rate_scanregion(sr_x, sr_y, width, height, ch_type, lambda);
            total_cost = cost_base + cost_last - removal_cost;
            if (sr_x == 0 || sr_y == 0)
            {
                total_cost -= pdcost_sig[sr_y * width + sr_x];
            }
            else
            {
                if (col[sr_x] == 1 && coef_dst[sr_x] != 0)
                {
                    total_cost -= pdcost_sig[sr_x];
                }

                if (row[sr_y] == 1 && coef_dst[sr_y*width] != 0)
                {
                    total_cost -= pdcost_sig[sr_y*width];
                }

                if (sr_x > 0 && sr_y > 0)
                {
                    blk_pos = sr_y * width + sr_x;
                    total_cost -= pdcost_sig[blk_pos];
                    total_cost += sig_last_cost[blk_pos];
                }
            }

            if (total_cost < cost_best)
            {
                cost_best = total_cost;
                sr_x_best = sr_x;
                sr_y_best = sr_y;
                best_case = 0;
            }

            if (col[sr_x] > 1 && coef_dst[sr_x] && (coef_dst[sr_y * width + sr_x] == 0 || row[sr_y] > 1))
            {
                total_cost_tmp = total_cost;
                if (sr_x > 0 && sr_y > 0 && coef_dst[sr_y * width + sr_x])
                {
                    blk_pos = sr_y * width + sr_x;
                    total_cost_tmp -= (pdcost_sig[blk_pos] - sig_cost_delta[blk_pos]);
                    total_cost_tmp += pdcost_sig[blk_pos];
                    total_cost_tmp -= sig_last_cost[blk_pos];
                    total_cost_tmp += sig_last_cost0[blk_pos];
                }
                j = sr_y;
                while (j > 0)
                {
                    blk_pos = j * width + sr_x;
                    if (coef_dst[blk_pos])
                    {
                        total_cost_tmp -= pdcost_coeff[blk_pos];
                        total_cost_tmp += pdcost_sig[blk_pos];
                        total_cost_tmp -= sig_cost_delta[blk_pos];
                        total_cost_tmp += pdcost_coeff0[blk_pos];
                    }
                    if (j == 1)
                    {
                        total_cost_tmp -= pdcost_sig[sr_x];
                    }
                    if (total_cost_tmp < cost_best)
                    {
                        cost_best = total_cost_tmp;
                        sr_x_best = sr_x;
                        sr_y_best = sr_y;
                        best_case = 1;
                        best_cutpos = j;
                    }
                    j--;
                }
            }

            if (row[sr_y] > 1 && coef_dst[sr_y * width] && (coef_dst[sr_y * width + sr_x] == 0 || col[sr_x] > 1))
            {
                total_cost_tmp = total_cost;
                if (sr_x > 0 && sr_y > 0 && coef_dst[sr_y * width + sr_x])
                {
                    blk_pos = sr_y * width + sr_x;
                    total_cost_tmp -= (pdcost_sig[blk_pos] - sig_cost_delta[blk_pos]);
                    total_cost_tmp += pdcost_sig[blk_pos];
                    total_cost_tmp -= sig_last_cost[blk_pos];
                    total_cost_tmp += sig_last_cost0[blk_pos];
                }

                i = sr_x;
                while (i > 0)
                {
                    blk_pos = sr_y * width + i;
                    if (coef_dst[blk_pos])
                    {
                        total_cost_tmp -= pdcost_coeff[blk_pos];
                        total_cost_tmp += pdcost_sig[blk_pos];
                        total_cost_tmp -= sig_cost_delta[blk_pos];
                        total_cost_tmp += pdcost_coeff0[blk_pos];
                    }
                    if (i == 1)
                    {
                        total_cost_tmp -= pdcost_sig[sr_y * width];
                    }
                    if (total_cost_tmp < cost_best)
                    {
                        cost_best = total_cost_tmp;
                        sr_x_best = sr_x;
                        sr_y_best = sr_y;
                        best_case = 2;
                        best_cutpos = i;
                    }
                    i--;
                }
            }

            if (col[sr_x] > 1 && coef_dst[sr_x] && row[sr_y] > 1 && coef_dst[sr_y * width])
            {
                total_cost_tmp = total_cost - pdcost_sig[sr_x] - pdcost_sig[sr_y * width];
                j = sr_y;
                while (j > 0)
                {
                    blk_pos = j * width + sr_x;
                    if (coef_dst[blk_pos])
                    {
                        total_cost_tmp -= pdcost_coeff[blk_pos];
                        total_cost_tmp += pdcost_sig[blk_pos];
                        total_cost_tmp -= sig_cost_delta[blk_pos];
                        total_cost_tmp += pdcost_coeff0[blk_pos];
                    }
                    j--;
                }

                i = sr_x - 1;
                while (i > 0)
                {
                    blk_pos = sr_y * width + i;
                    if (coef_dst[blk_pos])
                    {
                        total_cost_tmp -= pdcost_coeff[blk_pos];
                        total_cost_tmp += pdcost_sig[blk_pos];
                        total_cost_tmp -= sig_cost_delta[blk_pos];
                        total_cost_tmp += pdcost_coeff0[blk_pos];
                    }
                    i--;
                }

                if (sr_x > 0 && sr_y > 0 && coef_dst[sr_y * width + sr_x])
                {
                    blk_pos = sr_y * width + sr_x;
                    total_cost_tmp -= (pdcost_sig[blk_pos] - sig_cost_delta[blk_pos]);
                    total_cost_tmp -= sig_last_cost[blk_pos];
                    total_cost_tmp += sig_last_cost0[blk_pos];
                    total_cost_tmp += pdcost_sig[blk_pos];
                }

                if (total_cost_tmp < cost_best)
                {
                    cost_best = total_cost_tmp;
                    sr_x_best = sr_x;
                    sr_y_best = sr_y;
                    best_case = 3;
                }
            }

            if (sr_x == 0 && sr_y == 0)
            {
                break;
            }

            if ((col[sr_x] < row[sr_y] && sr_x > 0) || (col[sr_x] == row[sr_y] && sr_x > sr_y))
            {
                do
                {
                    for (j = sr_y; j >= 0; j--)
                    {
                        blk_pos = j * width + sr_x;
                        removal_cost += (pdcost_coeff[blk_pos] - pdcost_coeff0[blk_pos]);
                        if (coef_dst[blk_pos])
                        {
                            row[j]--;
                        }
                    }
                    sr_x--;
                    k++;
                } while (col[sr_x] == 0 && sr_x >= 0);

                if (sr_x < 0)
                {
                    break;
                }

                if (row[sr_y] == 0 && sr_y > 0)
                {
                    do
                    {
                        for (i = sr_x; i >= 0; i--)
                        {
                            blk_pos = sr_y * width + i;
                            removal_cost += (pdcost_coeff[blk_pos] - pdcost_coeff0[blk_pos]);
                        }
                        sr_y--;
                        k++;
                    } while (row[sr_y] == 0 && sr_y >= 0);

                    if (sr_y < 0)
                    {
                        break;
                    }
                }
            }
            else
            {
                if (sr_y > 0)
                {
                    do
                    {
                        for (i = sr_x; i >= 0; i--)
                        {
                            blk_pos = sr_y * width + i;
                            removal_cost += (pdcost_coeff[blk_pos] - pdcost_coeff0[blk_pos]);
                            if (coef_dst[blk_pos])
                            {
                                col[i]--;
                            }
                        }
                        sr_y--;
                        k++;
                    } while (row[sr_y] == 0 && sr_y >= 0);

                    if (sr_y < 0)
                    {
                        break;
                    }

                    if (col[sr_x] == 0 && sr_x > 0)
                    {
                        do
                        {
                            for (j = sr_y; j >= 0; j--)
                            {
                                blk_pos = j * width + sr_x;
                                removal_cost += (pdcost_coeff[blk_pos] - pdcost_coeff0[blk_pos]);
                            }
                            sr_x--;
                            k++;
                        } while (col[sr_x] == 0 && sr_x >= 0);

                        if (sr_x < 0)
                        {
                            break;
                        }
                    }
                }
            }
        }

        if (sr_x_best == -1 && sr_y_best == -1)
        {
            com_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
            return 0;
        }
        else if (sr_x_best == 0 && sr_y_best == 0)
        {
            s16 level = coef_dst[0];

            sr_x = sr_x_best;
            sr_y = sr_y_best;
            com_mset(coef_dst, 0, sizeof(s16) * max_num_coef);
            coef_dst[0] = (src_coef[0] < 0) ? -level : level;
            abs_sum = (int)level;
            nnz = 1;
        }
        else
        {
            abs_sum = 0;
            nnz = 0;
            sr_x = sr_x_best;
            sr_y = sr_y_best;

            for (j = 0; j <= sr_y_tmp; j++)
            {
                blk_pos = j * width;
                for (i = 0; i <= sr_x_tmp; i++)
                {
                    if (j > sr_y || i > sr_x)
                    {
                        coef_dst[blk_pos] = 0;
                    }
                    else
                    {
                        s16 level = coef_dst[blk_pos];
                        if (level && best_case && ((j == sr_y && i && ((best_case == 2 && i >= best_cutpos) || best_case == 3)) || (i == sr_x && j && ((best_case == 1 && j >= best_cutpos) || best_case == 3))))
                        {
                            level = 0;
                        }

                        abs_sum += (int)level;
                        coef_dst[blk_pos] = (src_coef[blk_pos] < 0) ? -level : level;
                        nnz += !!(level);
                    }
                    ++blk_pos;
                }
            }
        }
    }

    nnz = 0;
#if IST
    com_mset(row, 0, sizeof(int) * height);
    com_mset(col, 0, sizeof(int) * width);
    for (int j = 0; j < height; j++) 
    {
        for (int i = 0; i < width; i++) 
        {
            dst_tmp[j*width + i] = coef_dst[j*width + i];
            nnz += !!(coef_dst[j*width + i]);
            if (coef_dst[j*width + i]) 
            {
                col[i]++;
                row[j]++;
            }
        }
    }

#if ISTS
    if (nnz > 0 && ch_type == Y_C && 
        ((ctx->info.sqh.ist_enable_flag && sr_x < IST_MAX_COEF_SIZE && sr_y < IST_MAX_COEF_SIZE && mode->cu_mode == MODE_INTRA && !ctx->info.pic_header.ph_ists_enable_flag) ||
         (ctx->info.sqh.ists_enable_flag && ctx->info.pic_header.ph_ists_enable_flag && (mode->cu_mode == MODE_INTRA || mode->ibc_flag))))
#else
    if (ctx->info.sqh.ist_enable_flag && nnz > 0 && ch_type == Y_C && sr_x < IST_MAX_COEF_SIZE && sr_y < IST_MAX_COEF_SIZE)
#endif
    {
        s64 region_cost;
        s64 delta_cost;
        s64 final_cost = (s64)COM_INT64_MAX;
        s16 final_value = 0;
        int pos = -1;
#if ISTS
        if (mode->tb_part == 0 && log2_cuw < 6 && log2_cuh < 6 && 
            ((mode->cu_mode == MODE_INTRA && ctx->info.pic_header.ph_ists_enable_flag == 0) ||
             ((mode->cu_mode == MODE_INTRA || mode->ibc_flag) && ctx->info.pic_header.ph_ists_enable_flag && ctx->info.sqh.ists_enable_flag)) &&
             (((nnz % 2) && (!mode->ist_tu_flag)) || (!(nnz % 2) && (mode->ist_tu_flag))))
#else
        if (mode->tb_part == 0 && mode->cu_mode == MODE_INTRA && log2_cuw < 6 && log2_cuh < 6 && (((nnz % 2) && (!mode->ist_tu_flag)) || (!(nnz % 2) && (mode->ist_tu_flag))))
#endif
        {
            s64 delta_cbf_cost = GET_I_COST(rdoq_est_cbf[ch_type][0], lambda) - GET_I_COST(rdoq_est_cbf[ch_type][1], lambda);
            s64 origin_region_cost = get_rate_scanregion(sr_x, sr_y, width, height, ch_type, lambda);

            int w = (sr_width - 1) > (sr_x + 1) ? (sr_x + 1) : (sr_width - 1);
            int h = (sr_height - 1) > (sr_y + 1) ? (sr_y + 1) : (sr_height - 1);
            for (j = 0; j <= h; j++) 
            {
                blk_pos = j * width;
                for (i = 0; i <= w; i++) 
                {
                    int srx_tmp = sr_x;
                    int sry_tmp = sr_y;
                    if (dst_tmp[blk_pos]) 
                    {
                        if ((i == sr_x) && (col[sr_x] == 1)) 
                        {
                            srx_tmp = -1;
                            for (int c = 0; c < sr_x; c++)
                            {
                                if (col[c] != 0)
                                {
                                    srx_tmp = c;
                                }
                            }
                        }
                        if ((j == sr_y) && (row[sr_y] == 1)) 
                        {
                            sry_tmp = -1;
                            for (int r = 0; r < sr_y; r++)
                            {
                                if (row[r] != 0)
                                {
                                    sry_tmp = r;
                                }
                            }
                        }

                        if (srx_tmp == -1 || sry_tmp == -1) 
                        {
                            region_cost = delta_cbf_cost - origin_region_cost;
                        } 
                        else 
                        {
                            if ((srx_tmp != sr_x) || (sry_tmp != sr_y))
                            {
                                region_cost = get_rate_scanregion(srx_tmp, sry_tmp, width, height, ch_type, lambda) - origin_region_cost;
#if EST
                                if (ctx->info.sqh.est_enable_flag)
                                {
                                    if (srx_tmp != sr_x)
                                    {
                                        int tmp_pos = 0;
                                        for (int i = 0; i < sr_y; i++)
                                        {
                                            tmp_pos = i * width + sr_x;
                                            region_cost -= pdcost_sig[tmp_pos];
                                        }
                                    }
                                    if (sry_tmp != sr_y)
                                    {
                                        int tmp_pos = 0;
                                        for (int i = 0; i < sr_x; i++)
                                        {
                                            tmp_pos = sr_y * width + i;
                                            region_cost -= pdcost_sig[tmp_pos];
                                        }
                                    }
                                }
#endif
                            }
                            else
                            {
                                region_cost = 0;
                            }
                        }
                        if (srx_tmp == sr_x && sry_tmp == sr_y) 
                        {
                            delta_cost = (pdcost_coeff0[blk_pos] - sig_cost_delta[blk_pos] - pdcost_coeff[blk_pos] + pdcost_sig[blk_pos]);
                        }
                        else 
                        {
                            delta_cost = (pdcost_coeff0[blk_pos] - pdcost_coeff[blk_pos]);
                        }

                        s64 curr_cost = region_cost + delta_cost;
                        if (final_cost > curr_cost) 
                        {
                            final_cost = curr_cost;
                            pos = blk_pos;
                            final_value = 0;
                        }
                    } 
                    else if (dst_tmp[blk_pos] == 0) 
                    {
                        s16 tmp_value;
                        srx_tmp = srx_tmp > i ? srx_tmp : i;
                        sry_tmp = sry_tmp > j ? sry_tmp : j;
                        delta_cost = (pdcost[blk_pos] != -1) ? pdcost[blk_pos] : (s64)COM_INT64_MAX;
                        if ((abs(origin_coeff[blk_pos]) == 1)) 
                        {
                            tmp_value = origin_coeff[blk_pos];

                            if (srx_tmp == sr_x && sry_tmp == sr_y) 
                            {
                                delta_cost = delta_cost - pdcost_coeff0[blk_pos] + sig_cost_delta[blk_pos] - pdcost_sig[blk_pos]; 
                            }
                            else 
                            {
                                delta_cost = delta_cost - pdcost_coeff0[blk_pos];
                            }

                            if ((srx_tmp != sr_x) || (sry_tmp != sr_y))
                            {
                                region_cost = get_rate_scanregion(srx_tmp, sry_tmp, width, height, ch_type, lambda) - origin_region_cost;
#if EST
                                if (ctx->info.sqh.est_enable_flag)
                                {
                                    if (srx_tmp != sr_x)
                                    {
                                        int tmp_pos = 0;
                                        for (int i = 0; i < sr_y; i++)
                                        {
                                            tmp_pos = i * width + srx_tmp;
                                            region_cost += pdcost_sig[tmp_pos];
                                        }
                                    }
                                    if (sry_tmp != sr_y)
                                    {
                                        int tmp_pos = 0;
                                        for (int i = 0; i < sr_x; i++)
                                        {
                                            tmp_pos = sry_tmp * width + i;
                                            region_cost += pdcost_sig[tmp_pos];
                                        }
                                    }
                                }
#endif
                            }
                            else
                            {
                                region_cost = 0;
                            }

                            s64 curr_cost = region_cost + delta_cost;
                            if ((final_cost > curr_cost)) 
                            {
                                final_cost = curr_cost;
                                pos = blk_pos;
                                final_value = tmp_value;
                            }
                        }
                    }
                    blk_pos++;
                }
            }
            if (final_value) 
            {
                dst_tmp[pos] = final_value;
                nnz++;
            } 
            else 
            {
                if (nnz > 1) 
                {
                    dst_tmp[pos] = final_value;
                    nnz--;
                } 
                else 
                {
#if ISTS
                    if (ctx->info.pic_header.ph_ists_enable_flag)
                    {
                        assert(mode->ist_tu_flag == 0);
                        if (pos == 0)
                        {
                            dst_tmp[pos + 1] = 1;
                        }
                        else if (pos % 32 != 0)
                        {
                            dst_tmp[pos - 1] = 1;
                        }
                        else
                        {
                            dst_tmp[pos - width] = 1;
                        }
                    }
                    else
                    {
#endif
                        if (pos == width * height - 1)
                        {
                            dst_tmp[pos - 1] = 1;
                        }
                        else
                        {
                            dst_tmp[pos + 1] = 1;
                        }
#if ISTS
                    }
#endif
                    nnz++;
                }
            }
        }
    }
#else
    for (j = 0; j < (int)max_num_coef; j++)
    {
        dst_tmp[j] = coef_dst[j];
        nnz += !!(coef_dst[j]);
    }
#endif
    return nnz;
}
#endif //srcc
#endif

int enc_quant_nnz(ENC_CTX* ctx, COM_MODE *mode, int qp, double lambda, int is_intra, s16 * coef, int cu_width_log2, int cu_height_log2, int ch_type, int slice_type)
{
    int bit_depth = ctx->info.bit_depth_internal;
    int num_nz_coef = 0;
    int scale  = quant_scale[qp];
    int width = 1 << cu_width_log2;
    int height = 1 << cu_height_log2;

    if (width > 32)
    {
        int i, j;
        s16 *p = coef;
        for (i = 0; i < height; i++)
        {
            for (j = 32; j < width; j++)
            {
                p[j] = 0;
            }
            p += width;
        }
    }
    if (height > 32)
    {
        memset(coef + 32 * width, 0, sizeof(s16) * (width * height - 32 * width));
    }

#if USE_RDOQ
    if(!ctx->info.pic_header.pic_wq_enable)
    {
        s64 lev;
        s64 offset;
        int i;
        int shift;
        int tr_shift;
        int log2_size = (cu_width_log2 + cu_height_log2) >> 1;
        int ns_shift = ((cu_width_log2 + cu_height_log2) & 1) ? 7 : 0;
        int ns_scale = ((cu_width_log2 + cu_height_log2) & 1) ? 181 : 1;
        s64 zero_coeff_threshold;
        BOOL is_coded = 0;

        tr_shift = get_transform_shift(bit_depth, log2_size - ns_shift);
        shift = QUANT_SHIFT + tr_shift;
#define FAST_RDOQ_INTRA_RND_OFST  201 //171
#define FAST_RDOQ_INTER_RND_OFST  153 //85
        offset = (s64)((slice_type == SLICE_I) ? FAST_RDOQ_INTRA_RND_OFST : FAST_RDOQ_INTER_RND_OFST) << (s64)(shift - 9);
        zero_coeff_threshold = ((s64)1 << (s64)shift) - offset;
        for(i = 0; i < (1 << (cu_width_log2 + cu_height_log2)); i++)
        {
            lev = (s64)COM_ABS(coef[i]) * (s64)scale * ns_scale;
            if(lev >= zero_coeff_threshold)
            {
                is_coded = 1;
                break;
            }
        }
        if(!is_coded)
        {
            memset(coef, 0, sizeof(coef[0])*((s64)1 << (cu_width_log2 + cu_height_log2)));
            return num_nz_coef;
        }

        if((cu_height_log2 > MAX_TR_LOG2) || (cu_width_log2 > MAX_TR_LOG2))
        {
            s16 t[MAX_TR_DIM];
            int m, n;
            int nnz_tmp = 0;
            int tu_width_log2 = (cu_width_log2 > MAX_TR_LOG2) ? MAX_TR_LOG2 : cu_width_log2;
            int tu_height_log2 = (cu_height_log2 > MAX_TR_LOG2) ? MAX_TR_LOG2 : cu_height_log2;
            int log_w_loop2 = (cu_width_log2 > MAX_TR_LOG2) ? (1 << (cu_width_log2 - MAX_TR_LOG2)) : 1;
            int log_h_loop2 = (cu_height_log2 > MAX_TR_LOG2) ? (1 << (cu_height_log2 - MAX_TR_LOG2)) : 1;
            int stride = (1 << cu_width_log2);
            int stride1 = (1 << tu_width_log2);
            for(n = 0; n < log_h_loop2; n++)
            {
                for(m = 0; m < log_w_loop2; m++)
                {
                    int l;
                    s16 * coef_temp = &coef[n * MAX_TR_SIZE * stride + m * MAX_TR_SIZE];
                    //copy to temp
                    for(l = 0; l < (1 << tu_height_log2); l++)
                    {
                        memcpy(&t[l*stride1], coef_temp, sizeof(s16)*stride1);
                        coef_temp += stride;
                    }
#if SRCC
                    if (ctx->info.sqh.srcc_enable_flag)
                    {
                        nnz_tmp = enc_rdoq_srcc(ctx, mode, qp, lambda, is_intra, t, t, tu_width_log2, tu_height_log2, ch_type, bit_depth);
                    }
                    else
                    {
#endif
                        nnz_tmp = enc_rdoq_run_length_cc(qp, lambda, is_intra, t, t, tu_width_log2, tu_height_log2, ch_type, bit_depth);
#if SRCC
                    }
#endif
                    num_nz_coef += nnz_tmp;
                    //copy backto coefbuf
                    coef_temp = &coef[n * MAX_TR_SIZE * stride + m * MAX_TR_SIZE];
                    for(l = 0; l < (1 << tu_height_log2); l++)
                    {
                        memcpy(coef_temp, &t[l*stride1], sizeof(s16)*stride1);
                        coef_temp += stride;
                    }
                }
            }
        }
        else
        {
#if SRCC
            if (ctx->info.sqh.srcc_enable_flag)
            {
                num_nz_coef = enc_rdoq_srcc(ctx, mode, qp, lambda, is_intra, coef, coef, cu_width_log2, cu_height_log2, ch_type, bit_depth);
            }
            else
            {
#endif
                num_nz_coef = enc_rdoq_run_length_cc(qp, lambda, is_intra, coef, coef, cu_width_log2, cu_height_log2, ch_type, bit_depth);
#if SRCC
            }
#endif
        }
    }
    else
#endif
    {
        s64 offset;
        int i, j;
        int w = 1 << cu_width_log2;
        int h = 1 << cu_height_log2;
        int shift;
        int tr_shift;
        int log2_size = (cu_width_log2 + cu_height_log2) >> 1;
        int ns_shift = ((cu_width_log2 + cu_height_log2) & 1) ? 7 : 0;
        int ns_scale = ((cu_width_log2 + cu_height_log2) & 1) ? 181 : 1;
        int wq_width;
        int idx_shift;
        int idx_step;
        u8* wq;

        tr_shift = get_transform_shift(bit_depth, log2_size - ns_shift);
        shift = QUANT_SHIFT + tr_shift;
        offset = (s64)((slice_type == SLICE_I) ? 171 : 85) << (s64)(shift - 9);

        if (cu_width_log2 == 2 && cu_height_log2 == 2)
        {
            wq = ctx->wq[0];
            idx_shift = 0;
            idx_step = 1;
            wq_width = 4;
        }
        else
        {
            wq = ctx->wq[1];
            idx_shift = max(cu_width_log2, cu_height_log2) - 3;
            idx_step = 1 << idx_shift;
            wq_width = 8;
        }

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                int weight = wq[j >> idx_shift];
                int sign = COM_SIGN_GET(coef[j]);
                int lev = (s16)(((s64)COM_ABS(coef[j]) * (s64)scale * ns_scale * 64 / weight + offset) >> shift);
                coef[j] = (s16)COM_SIGN_SET(lev, sign);
                num_nz_coef += !!(coef[j]);
            }
            coef += w;

            if ((i + 1) % idx_step == 0)
            {
                wq += wq_width;
            }
        }
    }
    return num_nz_coef;
}

#if SBT
void copy_luma_tb_from_luma_cb( s16 luma_tb[MAX_TR_SIZE], s16 luma_cb[MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 sbt_info )
{
    int j;
    int cuw = 1 << log2_cuw;
    int log2_tuw, log2_tuh;
    int tuw, tuh;
    int tu_offset_x, tu_offset_y;

    get_sbt_tb_size( sbt_info, Y_C, log2_cuw, log2_cuh, &log2_tuw, &log2_tuh );
    get_sbt_tb_pos_offset( sbt_info, Y_C, log2_cuw, log2_cuh, &tu_offset_x, &tu_offset_y );
    tuw = 1 << log2_tuw;
    tuh = 1 << log2_tuh;

    for( j = tu_offset_y; j < tu_offset_y + tuh; j++ )
    {
        memcpy( luma_tb + (j - tu_offset_y) * tuw, luma_cb + tu_offset_x + j * cuw, sizeof( s16 )*tuw );
    }
}

void get_sbt_info_rdo_order( ENC_CORE *core, u8 sbt_avail, int* num_rdo, u8* sbt_info_list )
{
    int i;
    u8 idx = 0;
    if( sbt_avail == 0 )
    {
        sbt_info_list[idx++] = 0;
    }
    else
    {
        //add non-sbt mode
        sbt_info_list[idx++] = 0;

        //add sbt mode
        for( i = 0; i < 4; i++ )
        {
            if( (sbt_avail >> i) & 0x1 )
            {
                sbt_info_list[idx++] = get_sbt_info( i + 1, 0 );
                sbt_info_list[idx++] = get_sbt_info( i + 1, 1 );
            }
        }

        //toDO: add reordering fast algorithm based on estimated RDCost
    }

    *num_rdo = idx;
}
#endif

#if SBT_FAST
void calc_min_cost_sbt( ENC_CTX *ctx, ENC_CORE *core, pel pred[MAX_CU_DIM], pel* org, int s_pred_y, int s_org_y,
    u8 sbt_avail, s64* dist_no_resi, int* num_rdo, u8* sbt_info_list, s64* sbt_est_dist )
{
    int cuw = 1 << core->mod_info_curr.cu_width_log2;
    int cuh = 1 << core->mod_info_curr.cu_height_log2;
    int num_part_x = min( 16, cuw ) / 4;
    int num_part_y = min( 16, cuh ) / 4;
    s64 dist[4][4], dist_blk, dist_temp[9];
    s64 sum_dist = 0;
    u8  sbt_info_list_temp[9];
    int i, j, idx;
    int blk_luma_w = cuw / num_part_x;
    int blk_luma_h = cuh / num_part_y;
    int sbt_rdo_idx_list[4];
    int sbt_rdo_idx_num = 0;
    int num_half_sbt = ((sbt_avail & 0x1) ? 2 : 0) + ((sbt_avail & 0x2) ? 2 : 0);
    int num_quad_sbt = ((sbt_avail & 0x4) ? 2 : 0) + ((sbt_avail & 0x8) ? 2 : 0);
    assert( num_half_sbt + num_quad_sbt == *num_rdo - 1 );
    assert( ctx->tree_status != TREE_C );

    if( !sbt_avail )
        return;

    //SBT fast algorithm 1.1: not try SBT if the residual is too small to compensate bits for encoding residual info
    if( dist_no_resi[Y_C] < RATE_TO_COST_LAMBDA( ctx->lambda[0], 10 ) ) //10 extra bits for sbt residual encoding
    {
        *num_rdo = 1;
        return;
    }

    //SBT fast algorithm 1.2: derive estimated minDist of SBT = zero-residual part distortion + non-zero residual part distortion / 16
    memset( dist, 0, sizeof( s64 ) * 16 );
    int blk_w, blk_h;
    int log2_length_x = com_tbl_log2[blk_luma_w];
    int log2_length_y = com_tbl_log2[blk_luma_h];
    blk_w = 1 << log2_length_x;
    blk_h = 1 << log2_length_y;

    for( j = 0; j < num_part_y; j++ )
    {
        for( i = 0; i < num_part_x; i++ )
        {
            int offset_pred = j * blk_h * s_pred_y + i * blk_w;
            int offset_org = j * blk_h * s_org_y + i * blk_w;
            dist_blk = enc_ssd_16b( log2_length_x, log2_length_y, pred + offset_pred, org + offset_org, s_pred_y, s_org_y, ctx->info.bit_depth_internal );
            dist[j][i] += dist_blk;
            sum_dist += dist_blk;
        }
    }
    assert( abs( (int)(sum_dist - dist_no_resi[Y_C]) ) < 32 );

    //estimate rd cost for each sbt mode
    sbt_est_dist[0] = sum_dist;
    for( idx = 1; idx < 9; idx++ )
    {
        sbt_est_dist[idx] = UINT_MAX;
    }
    for( idx = 1; idx < *num_rdo; idx++ )
    {
        u8 sbt_info = sbt_info_list[idx];
        int log2_tuw, log2_tuh, tux, tuy, tuw, tuh;
        s64 dist_tu = 0;
        get_sbt_tb_size( sbt_info, Y_C, core->mod_info_curr.cu_width_log2, core->mod_info_curr.cu_height_log2, &log2_tuw, &log2_tuh );
        get_sbt_tb_pos_offset( sbt_info, Y_C, core->mod_info_curr.cu_width_log2, core->mod_info_curr.cu_height_log2, &tux, &tuy );
        tuw = 1 << log2_tuw;
        tuh = 1 << log2_tuh;
        for( j = tuy / blk_luma_h; j < (tuy + tuh) / blk_luma_h; j++ )
        {
            for( i = tux / blk_luma_w; i < (tux + tuw) / blk_luma_w; i++ )
            {
                dist_tu += dist[j][i];
            }
        }
        sbt_est_dist[idx] = (dist_tu / 16) + (sum_dist - dist_tu);
    }
    //try 2 half SBT modes with the lowest distortion
    memcpy( dist_temp, sbt_est_dist, sizeof( s64 ) * 9 );
    if( num_half_sbt > 0 )
    {
        for( i = sbt_rdo_idx_num; i < sbt_rdo_idx_num + 2; i++ )
        {
            s64 min_dist = UINT_MAX;
            for( idx = 1; idx < 1 + num_half_sbt; idx++ )
            {
                if( dist_temp[idx] < min_dist )
                {
                    min_dist = dist_temp[idx];
                    sbt_rdo_idx_list[i] = idx;
                }
            }
            dist_temp[sbt_rdo_idx_list[i]] = UINT_MAX;
        }
        sbt_rdo_idx_num += 2;
    }
    if( num_quad_sbt > 0 )
    {
        for( i = sbt_rdo_idx_num; i < sbt_rdo_idx_num + 2; i++ )
        {
            s64 min_dist = UINT_MAX;
            for( idx = 1 + num_half_sbt; idx < 1 + num_half_sbt + num_quad_sbt; idx++ )
            {
                if( dist_temp[idx] < min_dist )
                {
                    min_dist = dist_temp[idx];
                    sbt_rdo_idx_list[i] = idx;
                }
            }
            dist_temp[sbt_rdo_idx_list[i]] = UINT_MAX;
        }
        sbt_rdo_idx_num += 2;
    }
    //update sbt_info_list
    memcpy( dist_temp, sbt_est_dist, sizeof( s64 ) * 9 );
    memcpy( sbt_info_list_temp, sbt_info_list, sizeof( u8 ) * 9 );
    for( idx = 1; idx < 1 + sbt_rdo_idx_num; idx++ )
    {
        sbt_info_list[idx] = sbt_info_list_temp[sbt_rdo_idx_list[idx - 1]];
        sbt_est_dist[idx] = dist_temp[sbt_rdo_idx_list[idx - 1]];
    }
    for( idx = 1 + sbt_rdo_idx_num; idx < *num_rdo; idx++ )
    {
        sbt_info_list[idx] = 255;
        sbt_est_dist[idx] = UINT_MAX;
    }
    *num_rdo = sbt_rdo_idx_num + 1;
}

u8 skip_sbt_by_rd_cost( ENC_CTX *ctx, s64* sbt_est_dist, u8* sbt_info_list, int curr_idx, double cost_best, s64 dist_sbt0, double cost_sbt0, u8 root_cbf_sbt0, double chroma_cost )
{
    //SBT fast algorithm 2.2 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
    //                         if this cost is larger than the best cost, no need to try a specific SBT mode
    double cost_curr_sbt = sbt_est_dist[curr_idx] + RATE_TO_COST_LAMBDA( ctx->lambda[0], 6 ) + chroma_cost;
    if( cost_curr_sbt > cost_best )
    {
        return 1;
    }

    u8 sbt_idx = get_sbt_idx( sbt_info_list[curr_idx] );
    if( !root_cbf_sbt0 )
    {
        //SBT fast algorithm 3: skip SBT when the residual is too small (estCost is more accurate than fast algorithm 1, counting PU mode bits)
        int weight = is_sbt_quad_size( sbt_idx ) ? 6 : 9;
        s64 dist_resi_part = ((sbt_est_dist[0] - sbt_est_dist[curr_idx]) * weight) >> 4;
        //prediction info bits cost + minimum residual bits cost + estimate distortion
        double est_cost = RATE_TO_COST_LAMBDA( ctx->lambda[0], 6 ) + (sbt_est_dist[curr_idx] + dist_resi_part) + chroma_cost;
        if( est_cost > cost_best )
        {
            return 2;
        }
    }
    else
    {
        //SBT fast algorithm 4: skip SBT when an estimated RD cost is larger than the bestCost
        double weight = is_sbt_quad_size( sbt_idx ) ? 0.4 : 0.6;
        double est_cost = (cost_sbt0 - dist_sbt0) * weight + sbt_est_dist[curr_idx] + RATE_TO_COST_LAMBDA( ctx->lambda[0], 6 ) + chroma_cost;
        if( est_cost > cost_best )
        {
            return 3;
        }
    }
    return 0;
}
#endif

#if TR_EARLY_TERMINATE
int est_pred_info_bits(ENC_CORE* core)
{
    COM_MODE* mod_cur = &core->mod_info_curr;
    int bits_residual = 2 + 4 + (mod_cur->num_nz[TB0][Y_C] == 0 ? 5 : 10);//(cbf u + cbf v) + (4 cbf_y) + (run + sign + level + last)
    int bits_pred = 0;
    if (mod_cur->cu_mode == MODE_DIR)
    {
        bits_pred = 6;
    }
    else if (mod_cur->cu_mode == MODE_INTER)
    {
        bits_pred = 10;
    }
    else
    {
        bits_pred = 2;
    }

    return bits_pred + bits_residual;
}
#endif

int enc_tq_nnz(ENC_CTX* ctx, COM_MODE *mode, int plane, int blk_idx, int qp, double lambda, s16 * coef, s16 *resi, int cu_width_log2, int cu_height_log2, int slice_type, int ch_type, int is_intra, int secT_Ver_Hor, int use_alt4x4Trans)
{
#if IST
    assert( mode->slice_type == ctx->slice_type );
#endif
#if CHROMA_NOT_SPLIT
    assert(cu_width_log2 >= 2 && cu_height_log2 >= 2);
#endif
    int bit_depth = ctx->info.bit_depth_internal;
    enc_trans(mode, plane, blk_idx, coef, resi, cu_width_log2, cu_height_log2, is_intra, ch_type, bit_depth, secT_Ver_Hor, use_alt4x4Trans);
    return enc_quant_nnz(ctx, mode, qp, lambda, is_intra, coef, cu_width_log2, cu_height_log2, ch_type, slice_type);
}

int enc_tq_yuv_nnz(ENC_CTX *ctx, ENC_CORE *core, COM_MODE *cur_mode, s16 coef[N_C][MAX_CU_DIM], s16 resi[N_C][MAX_CU_DIM],
                   int slice_type, int is_intra, int use_secTrans[MAX_NUM_TB], int use_alt4x4Trans, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], u8 is_mv_from_mvf)
{
    int i;
    ENC_PINTER *pi = &ctx->pinter;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
#if SBT // sbt rdo
    double cost_best_luma_cb = MAX_COST;
    u8     sbt_info_best = 0;
    u8     tb_part_best = SIZE_2Nx2N;
    u8     sbt_avail = com_sbt_allow( cur_mode, ctx->info.sqh.sbt_enable_flag, ctx->tree_status );
    s64    dist_sbt0 = -1;
    double cost_sbt0 = MAX_COST;
    int    bit_cnt_2Nx2N = 0;
    int    bit_cnt_NxN = 0;
    double dist_luma_2Nx2N = 0;
    double dist_luma_NxN = 0;
#if SBT_FAST
    s64    sbt_est_dist[9];
#endif
    assert( cur_mode->sbt_info == 0 );
#endif
    cu_nz_cln(cur_mode->num_nz);
#if IST
    assert( cur_mode->slice_type == ctx->slice_type );
#endif

    // test 2Nx2N, loop luma and chroma
    cur_mode->tb_part = SIZE_2Nx2N;
    for (i = 2; i >= 0; i--)
    {
        int plane_width_log2 = cu_width_log2 - (i != Y_C);
        int plane_height_log2 = cu_height_log2 - (i != Y_C);
        int qp = (i == Y_C ? pi->qp_y : (i == U_C ? pi->qp_u : pi->qp_v));
        double lambda = (i == Y_C ? ctx->lambda[0] : (i == U_C ? ctx->lambda[1] : ctx->lambda[2]));
        int secT_VH = i == Y_C ? use_secTrans[TB0] : 0;
        int alt4x4 = i == Y_C ? use_alt4x4Trans : 0;

#if CHROMA_NOT_SPLIT
        //skip transform & quantization according to tree_status
        if (ctx->tree_status == TREE_L && i != Y_C)
        {
            cur_mode->num_nz[TB0][i] = 0;
            continue;
        }
        if (ctx->tree_status == TREE_C && i == Y_C)
        {
            cur_mode->num_nz[TB0][i] = 0;
            continue;
        }
#endif
#if TR_SAVE_LOAD
        if (core->best_tb_part_hist == 255 || core->best_tb_part_hist == SIZE_2Nx2N || i != Y_C)
        {
#endif
            cur_mode->num_nz[TB0][i] = enc_tq_nnz(ctx, cur_mode, i, 0, qp, lambda, coef[i], resi[i], plane_width_log2, plane_height_log2, slice_type, i, is_intra, secT_VH, alt4x4);
#if TR_SAVE_LOAD
        }
        else
        {
            cur_mode->num_nz[TB0][Y_C] = 0; //no need to try 2Nx2N transform
#if SBT && PLATFORM_GENERAL_DEBUG
            int cu_size = 1 << (plane_width_log2 + plane_height_log2);
            memset( coef[Y_C], 0, sizeof( s16 ) * cu_size );
#endif
        }
#endif
    }

    int try_sub_block_transform = is_tb_avaliable(ctx->info, cur_mode);
    if (ctx->tree_status == TREE_C)
        try_sub_block_transform = 0;

    //fast algorithm
    if (try_sub_block_transform)
    {
#if TR_SAVE_LOAD
        if (core->best_tb_part_hist == SIZE_2Nx2N)
            try_sub_block_transform = 0;
#endif
#if TR_EARLY_TERMINATE
        if (try_sub_block_transform && core->best_tb_part_hist == 255)
        {
            COM_MODE* mod_curr = &core->mod_info_curr;
            int bits_est = est_pred_info_bits(core);
            double bits_cost = RATE_TO_COST_LAMBDA(ctx->lambda[Y_C], bits_est);
            s64    dist_cost = core->dist_pred_luma >> (cur_mode->num_nz[TB0][Y_C] == 0 ? 4 : 6);
            if (bits_cost + dist_cost > core->cost_best)
                try_sub_block_transform = 0;
        }
#endif
    }

    if (try_sub_block_transform)
    {
        s16 bak_2Nx2N_coef[MAX_CU_DIM], *cur_coef = coef[Y_C];
        int bak_2Nx2N_num_nz = cur_mode->num_nz[TB0][Y_C];
        int cu_size = 1 << (cu_width_log2 + cu_height_log2);
        int cu_width = 1 << cu_width_log2;
        int cu_height = 1 << cu_height_log2;
        int x = mod_info_curr->x_pos;
        int y = mod_info_curr->y_pos;
        pel *pred = cur_mode->pred[Y_C];
        pel *org = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
        double cost_2Nx2N, cost_NxN;
        int part_num;
        int log2_tb_w, log2_tb_h, tb_size;
        int cu_w = 1 << cu_width_log2;
        int cu_h = 1 << cu_height_log2;
        PART_SIZE part_size = get_tb_part_size_by_pb(cur_mode->pb_part, is_intra? MODE_INTRA : MODE_INTER);

        if (bak_2Nx2N_num_nz)
        {
            memcpy(bak_2Nx2N_coef, cur_coef, sizeof(s16) *cu_size);
        }

        // check sub-TB
        cur_mode->tb_part = part_size;
        part_num = get_part_num(part_size);
        get_tb_width_height_log2(cu_width_log2, cu_height_log2, part_size, &log2_tb_w, &log2_tb_h);
#if SBT
        assert( cur_mode->sbt_info == 0 );
#endif
        tb_size = 1 << (log2_tb_w + log2_tb_h);

        for (i = 0; i < part_num; i++)
        {
            s16 resi_buf[MAX_CU_DIM], *tb_resi = resi[Y_C];

            if (part_num > 1)
            {
                int k, pos_x, pos_y;
                int tu_w = 1 << log2_tb_w;
                int tu_h = 1 << log2_tb_h;
                s16 *s, *d;

                get_tb_start_pos(cu_w, cu_h, part_size, i, &pos_x, &pos_y);

                s = resi[Y_C] + pos_y * cu_w + pos_x;
                d = resi_buf;

                for (k = 0; k < tu_h; k++)
                {
                    memcpy(d, s, sizeof(s16) * tu_w);
                    d += tu_w;
                    s += cu_w;
                }
                tb_resi = resi_buf;
            }

            cur_mode->num_nz[i][Y_C] = enc_tq_nnz(ctx, cur_mode, Y_C, i, pi->qp_y, ctx->lambda[Y_C], coef[Y_C] + i * tb_size, tb_resi, log2_tb_w, log2_tb_h, slice_type, Y_C, is_intra, use_secTrans[i], use_alt4x4Trans);
        }

        if (bak_2Nx2N_num_nz && is_cu_plane_nz(cur_mode->num_nz, Y_C))
        {
#if !SBT
            pel rec[MAX_CU_DIM];
            pel resi_it[MAX_CU_DIM];
#endif
            // cost for NxN
            int j, bit_cnt;
            s16 bak_NxN_coef[MAX_CU_DIM];
            int bak_NxN_num_nz[MAX_NUM_TB];

            memcpy(bak_NxN_coef, cur_coef, sizeof(s16) *cu_size);
            for (j = 0; j < MAX_NUM_TB; j++)
            {
                bak_NxN_num_nz[j] = cur_mode->num_nz[j][Y_C];
            }

#if SBT
            com_itdq_plane( cur_mode, Y_C, cur_coef, pi->resi_cb[Y_C], ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans );
            com_recon( cur_mode->tb_part, pi->resi_cb[Y_C], pred, cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, pi->reco_cb[Y_C], bit_depth, 0 );
            cost_NxN = (double)(enc_ssd_16b( cu_width_log2, cu_height_log2, pi->reco_cb[Y_C], org, cu_width, pi->stride_org[Y_C], bit_depth ));
#else
            com_itdq_plane(cur_mode, Y_C, cur_coef, resi_it, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans);
            com_recon(cur_mode->tb_part, resi_it, pred, cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, rec, bit_depth);
            cost_NxN = (double)(enc_ssd_16b(cu_width_log2, cu_height_log2, rec, org, cu_width, pi->stride_org[Y_C], bit_depth));
#endif

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);

            enc_bit_est_inter_comp(ctx, core, cur_coef, Y_C);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
#if SBT
            bit_cnt_NxN = bit_cnt;
            dist_luma_NxN = cost_NxN;
#endif
            cost_NxN += RATE_TO_COST_LAMBDA(ctx->lambda[Y_C], bit_cnt);

            //////////////////////////////////////////////////////////////////////////////////////////

            memcpy(cur_coef, bak_2Nx2N_coef, sizeof(s16) *cu_size);
            cu_plane_nz_cln(cur_mode->num_nz, Y_C);
            cur_mode->num_nz[TB0][Y_C] = bak_2Nx2N_num_nz;
            cur_mode->tb_part = SIZE_2Nx2N;

#if SBT
            com_itdq_plane( cur_mode, Y_C, cur_coef, pi->resi_cb[Y_C], ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans );
            com_recon( cur_mode->tb_part, pi->resi_cb[Y_C], pred, cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, pi->reco_cb[Y_C], bit_depth, 0);
            cost_2Nx2N = (double)(enc_ssd_16b( cu_width_log2, cu_height_log2, pi->reco_cb[Y_C], org, cu_width, pi->stride_org[Y_C], bit_depth ));
#else
            com_itdq_plane(cur_mode, Y_C, cur_coef, resi_it, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans);
            com_recon(cur_mode->tb_part, resi_it, pred, cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, rec, bit_depth);
            cost_2Nx2N = (double)(enc_ssd_16b(cu_width_log2, cu_height_log2, rec, org, cu_width, pi->stride_org[Y_C], bit_depth));
#endif

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);

            enc_bit_est_inter_comp(ctx, core, cur_coef, Y_C);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
#if SBT
            bit_cnt_2Nx2N = bit_cnt;
            dist_luma_2Nx2N = cost_2Nx2N;
#endif
            cost_2Nx2N += RATE_TO_COST_LAMBDA(ctx->lambda[Y_C], bit_cnt);

            if (cost_NxN < cost_2Nx2N)
            {
                memcpy(cur_coef, bak_NxN_coef, sizeof(s16) *cu_size);
                for (j = 0; j < MAX_NUM_TB; j++)
                {
                    cur_mode->num_nz[j][Y_C] = bak_NxN_num_nz[j];
                }
                cur_mode->tb_part = get_tb_part_size_by_pb(cur_mode->pb_part, is_intra ? MODE_INTRA : MODE_INTER);
            }
#if SBT
            cost_sbt0 = min( cost_2Nx2N, cost_NxN );
            dist_sbt0 = (s64)(cost_2Nx2N < cost_NxN ? dist_luma_2Nx2N : dist_luma_NxN);
#endif
        }
        else if (bak_2Nx2N_num_nz)
        {
            memcpy(cur_coef, bak_2Nx2N_coef, sizeof(s16) *cu_size);
            cu_plane_nz_cln(cur_mode->num_nz, Y_C);
            cur_mode->num_nz[TB0][Y_C] = bak_2Nx2N_num_nz;
            cur_mode->tb_part = SIZE_2Nx2N;
        }
    }

    check_set_tb_part(cur_mode);

#if SBT
#if SBT_SAVELOAD //skip if best mode is not sbt
    if( core->best_tb_part_hist == SIZE_NxN || core->best_sbt_info_hist == 0 )
    {
        sbt_avail = 0;
    }
#endif
    if( sbt_avail )
    {
        u8  sbt_info_list[9];
        int num_rdo;
        int nnz_best[MAX_NUM_TB][N_C];
        int sbt_mode_idx = 0;
        int cu_width = 1 << cu_width_log2;
        int cu_height = 1 << cu_height_log2;
        int cu_size = cu_width * cu_height;
        int tb_size;
        int chroma_cost_calc = 0;
        double chroma_cost = MAX_COST;
        double cost_sbt = MAX_COST;
        int bit_cnt;
        int plane_width_log2, plane_height_log2;
        int root_cbf_sbt0 = is_cu_nz( cur_mode->num_nz );
        pel *org[N_C];
        int x = cur_mode->x_pos;
        int y = cur_mode->y_pos;
        org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
        org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
        org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

        get_sbt_info_rdo_order( core, sbt_avail, &num_rdo, sbt_info_list );
#if SBT_FAST
#if SBT_SAVELOAD
        if( core->best_sbt_info_hist == 255 )
#endif
        {
            calc_min_cost_sbt( ctx, core, cur_mode->pred[Y_C], org[Y_C], cu_width, pi->stride_org[Y_C], sbt_avail, core->dist_no_resi, &num_rdo, sbt_info_list, sbt_est_dist );
        }
#endif
        if( cost_sbt0 == MAX_COST )
        {
            com_itdq_plane( cur_mode, Y_C, coef[Y_C], pi->resi_cb[Y_C], ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans );
            com_recon( cur_mode->tb_part, pi->resi_cb[Y_C], cur_mode->pred[Y_C], cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, pi->reco_cb[Y_C], bit_depth, 0 );
            cost_sbt0 = (double)(enc_ssd_16b( cu_width_log2, cu_height_log2, pi->reco_cb[Y_C], org[Y_C], cu_width, pi->stride_org[Y_C], bit_depth ));
            dist_sbt0 = (s64)cost_sbt0;

            SBAC_LOAD( core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2] );
            bit_cnt = enc_get_bit_number( &core->s_temp_run );
            enc_bit_est_inter_comp( ctx, core, coef[Y_C], Y_C );
            bit_cnt = enc_get_bit_number( &core->s_temp_run ) - bit_cnt;
            cost_sbt0 += RATE_TO_COST_LAMBDA( ctx->lambda[Y_C], bit_cnt );
        }
        cost_best_luma_cb = cost_sbt0;

        //save best info of non-SBT mode
        tb_part_best = cur_mode->tb_part;
        cu_plane_nz_cpy( nnz_best, cur_mode->num_nz, Y_C );

        //sbt mode rdo
        cu_plane_nz_cln( cur_mode->num_nz, Y_C ); //reset cbf of y to all 0
        for( sbt_mode_idx = 1; sbt_mode_idx < num_rdo; sbt_mode_idx++ )
        {
            cur_mode->sbt_info = sbt_info_list[sbt_mode_idx];
            cur_mode->tb_part = SIZE_2Nx2N;
            assert( get_sbt_idx( cur_mode->sbt_info ) >= 0 && get_sbt_idx( cur_mode->sbt_info ) <= 4 );
            assert( get_sbt_pos( cur_mode->sbt_info ) >= 0 && get_sbt_pos( cur_mode->sbt_info ) <= 1 );

            //early skip fast algorithm here
#if SBT_SAVELOAD //skip
            if( core->best_sbt_info_hist != 255 && cur_mode->sbt_info != core->best_sbt_info_hist )
            {
                continue;
            }
#endif
#if SBT_FAST
#if SBT_SAVELOAD
            if( sbt_mode_idx > 0 && core->best_sbt_info_hist == 255 )
#else
            if( sbt_mode_idx > 0 )
#endif
            {
                //derive chroma cost (chroma coeff bits + chroma distortion + prediction bits)
                if( sbt_est_dist[sbt_mode_idx] + RATE_TO_COST_LAMBDA( ctx->lambda[0], 11 ) > core->cost_best )
                {
                    continue;
                }
                assert( cost_best_luma_cb != MAX_COST );
                assert( cost_sbt0 != MAX_COST );
                if( sbt_est_dist[sbt_mode_idx] > cost_best_luma_cb )
                {
                    continue;
                }
                if( chroma_cost == MAX_COST )
                {

                    chroma_cost = 0;
                    if( ctx->tree_status == TREE_LC )
                    {
                        SBAC_LOAD( core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2] );
                        bit_cnt = enc_get_bit_number( &core->s_temp_run );
                        enc_bit_est_inter( ctx, core, ctx->slice_type, 0 );
                        for( i = U_C; i < N_C; i++ )
                        {
                            int qp_chroma = (i == U_C) ? pi->qp_u : pi->qp_v;
                            //chroma distortion
                            if( is_cu_plane_nz( cur_mode->num_nz, i ) )
                            {
                                com_itdq_plane( cur_mode, i, coef[i], pi->resi_cb[i], ctx->wq, cu_width_log2 - 1, cu_height_log2 - 1, qp_chroma, bit_depth, use_secTrans, use_alt4x4Trans );
                                com_recon( SIZE_2Nx2N, pi->resi_cb[i], cur_mode->pred[i], cur_mode->num_nz, i, cu_width / 2, cu_height / 2, cu_width / 2, pi->reco_cb[i], bit_depth, 0 );
                                chroma_cost += enc_ssd_16b( cu_width_log2 - 1, cu_height_log2 - 1, pi->reco_cb[i], org[i], cu_width / 2, pi->stride_org[i], bit_depth );
                            }
                            else
                            {
                                chroma_cost += core->dist_no_resi[i];
                            }
                            //chroma bits
                            enc_bit_est_inter_comp( ctx, core, coef[i], i );
                        }
                        bit_cnt = enc_get_bit_number( &core->s_temp_run ) - bit_cnt;
                        chroma_cost += RATE_TO_COST_LAMBDA( ctx->lambda[0], bit_cnt );
                    }
                }
                if( skip_sbt_by_rd_cost( ctx, sbt_est_dist, sbt_info_list, sbt_mode_idx, core->cost_best, dist_sbt0, cost_sbt0, root_cbf_sbt0, chroma_cost ) )
                {
                    continue;
                }
            }
#endif

            //prepare tu residual
            copy_luma_tb_from_luma_cb( pi->resi_cb[Y_C], resi[Y_C], cu_width_log2, cu_height_log2, cur_mode->sbt_info );
            get_sbt_tb_size( mod_info_curr->sbt_info, Y_C, cu_width_log2, cu_height_log2, &plane_width_log2, &plane_height_log2 );
#if SBT
            get_sbt_tr( mod_info_curr->sbt_info, cu_width_log2, cu_height_log2, &mod_info_curr->sbt_hor_trans, &mod_info_curr->sbt_ver_trans );
#endif
            tb_size = 1 << (plane_width_log2 + plane_height_log2);

            cur_mode->num_nz[TB0][Y_C] = enc_tq_nnz( ctx, cur_mode, Y_C, 0, pi->qp_y, ctx->lambda[0], pi->coff_cb, pi->resi_cb[Y_C], plane_width_log2, plane_height_log2, slice_type, Y_C, is_intra, use_secTrans[TB0], use_alt4x4Trans );
            if( cur_mode->num_nz[TB0][Y_C] )
            {
                com_itdq_plane( cur_mode, Y_C, pi->coff_cb, pi->resi_cb[Y_C], ctx->wq, plane_width_log2, plane_height_log2, pi->qp_y, bit_depth, use_secTrans, use_alt4x4Trans );
                com_recon( cur_mode->tb_part, pi->resi_cb[Y_C], cur_mode->pred[Y_C], cur_mode->num_nz, Y_C, cu_width, cu_height, cu_width, pi->reco_cb[Y_C], bit_depth, cur_mode->sbt_info );
                cost_sbt = (double)(enc_ssd_16b( cu_width_log2, cu_height_log2, pi->reco_cb[Y_C], org[Y_C], cu_width, pi->stride_org[Y_C], bit_depth ));

                SBAC_LOAD( core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2] );
                bit_cnt = enc_get_bit_number( &core->s_temp_run );
                enc_bit_est_inter_comp( ctx, core, pi->coff_cb, Y_C );
                bit_cnt = enc_get_bit_number( &core->s_temp_run ) - bit_cnt;
                cost_sbt += RATE_TO_COST_LAMBDA( ctx->lambda[Y_C], bit_cnt );

                if( cost_sbt < cost_best_luma_cb )
                {
                    cost_best_luma_cb = cost_sbt;
                    cu_plane_nz_cpy( nnz_best, cur_mode->num_nz, Y_C );
                    tb_part_best = cur_mode->tb_part;
                    sbt_info_best = cur_mode->sbt_info;
                    memcpy( coef[Y_C], pi->coff_cb, sizeof( s16 ) *tb_size );
#if PLATFORM_GENERAL_DEBUG
                    memset( coef[Y_C] + tb_size, 0, sizeof( s16 ) *(cu_size - tb_size) );
#endif
                }
            }
        }

        //recover the best info
        cur_mode->tb_part = tb_part_best;
        cur_mode->sbt_info = sbt_info_best;
        cu_plane_nz_cpy( cur_mode->num_nz, nnz_best, Y_C );
    }
#endif

    return is_cu_nz(cur_mode->num_nz);
}

