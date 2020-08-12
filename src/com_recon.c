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

#include "com_def.h"
#include "com_recon.h"


void com_recon(PART_SIZE part, s16 *resi, pel *pred, int (*is_coef)[N_C], int plane, int cu_width, int cu_height, int s_rec, pel *rec, int bit_depth
#if SBT
    , u8 sbt_info
#endif
)
{
    int i, j;
    s16 t0;
    int k, part_num = get_part_num(part);
    int tb_height, tb_width;

    get_tb_width_height(cu_width, cu_height, part, &tb_width, &tb_height);

    for (k = 0; k < part_num; k++)
    {
        int tb_x, tb_y;
        pel *p, *r;
        s16 *c;

        get_tb_start_pos(cu_width, cu_height, part, k, &tb_x, &tb_y);

        p = pred + tb_y * cu_width + tb_x;
        r = rec  + tb_y * s_rec + tb_x;

        if (is_coef[k][plane] == 0) /* just copy pred to rec */
        {
            for (i = 0; i < tb_height; i++)
            {
                for (j = 0; j < tb_width; j++)
                {
                    r[i * s_rec + j] = COM_CLIP3(0, (1 << bit_depth) - 1, p[i * cu_width + j]);
                }
            }
        }
        else  /* add b/w pred and coef and copy it into rec */
        {
#if SBT
            if( sbt_info != 0 && plane == Y_C )
            {
                u8  sbt_idx = get_sbt_idx( sbt_info );
                u8  sbt_pos = get_sbt_pos( sbt_info );
                int tu0_w, tu0_h;
                int tu1_w, tu1_h;
                assert( sbt_idx >= 1 && sbt_idx <= 4 );
                assert( p == pred && part_num == 1 );
                assert( cu_width == tb_width && cu_height == tb_height );
                if( !is_sbt_horizontal( sbt_idx ) )
                {
                    tu0_w = is_sbt_quad_size( sbt_idx ) ? (cu_width / 4) : (cu_width / 2);
                    tu0_w = sbt_pos == 0 ? tu0_w : cu_width - tu0_w;
                    tu1_w = cu_width - tu0_w;
                    for( i = 0; i < cu_height; i++ )
                    {
                        for( j = 0; j < tu0_w; j++ )
                        {
                            t0 = (sbt_pos == 0 ? resi[i * tu0_w + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                        for( j = tu0_w; j < cu_width; j++ )
                        {
                            t0 = (sbt_pos == 1 ? resi[i * tu1_w + j - tu0_w] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                    }
                }
                else
                {
                    tu0_h = is_sbt_quad_size( sbt_idx ) ? (cu_height / 4) : (cu_height / 2);
                    tu0_h = sbt_pos == 0 ? tu0_h : cu_height - tu0_h;
                    tu1_h = cu_height - tu0_h;
                    for( j = 0; j < cu_width; j++ )
                    {
                        for( i = 0; i < tu0_h; i++ )
                        {
                            t0 = (sbt_pos == 0 ? resi[i * cu_width + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                        for( i = tu0_h; i < cu_height; i++ )
                        {
                            t0 = (sbt_pos == 1 ? resi[(i - tu0_h) * cu_width + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                    }
                }
                return;
            }
#endif
            c = resi + k * tb_width * tb_height;
            for (i = 0; i < tb_height; i++)
            {
                for (j = 0; j < tb_width; j++)
                {
                    t0 = c[i * tb_width + j] + p[i * cu_width + j];
                    r[i * s_rec + j] = COM_CLIP3(0, (1 << bit_depth) - 1, t0);
                }
            }
        }
    }
}



void com_recon_yuv(PART_SIZE part_size, int x, int y, int cu_width, int cu_height, s16 resi[N_C][MAX_CU_DIM], pel pred[N_C][MAX_CU_DIM], int (*num_nz_coef)[N_C], COM_PIC *pic, CHANNEL_TYPE channel, int bit_depth 
#if SBT
    , u8 sbt_info
#endif
)
{
    pel * rec;
    int s_rec, off;

    /* Y */
    if (channel == CHANNEL_LC || channel == CHANNEL_L)
    {
        s_rec = pic->stride_luma;
        rec = pic->y + (y * s_rec) + x;
        com_recon(part_size, resi[Y_C], pred[Y_C], num_nz_coef, Y_C, cu_width, cu_height, s_rec, rec, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }

    /* chroma */
    if (channel == CHANNEL_LC || channel == CHANNEL_C)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[U_C], pred[U_C], num_nz_coef, U_C, cu_width, cu_height, pic->stride_chroma, pic->u + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
        com_recon(SIZE_2Nx2N, resi[V_C], pred[V_C], num_nz_coef, V_C, cu_width, cu_height, pic->stride_chroma, pic->v + off, bit_depth
#if SBT
            , sbt_info

#endif
        );
    }

#if PMC
    /* U only or V only */
    if (channel == CHANNEL_U)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[U_C], pred[U_C], num_nz_coef, U_C, cu_width, cu_height, pic->stride_chroma, pic->u + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }

    if (channel == CHANNEL_V)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[V_C], pred[V_C], num_nz_coef, V_C, cu_width, cu_height, pic->stride_chroma, pic->v + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }
#endif
}

#if USE_SP
void sp_recon_yuv(COM_MODE *mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC *pic, CHANNEL_TYPE channel)
{
    int total_pixel = 1 << (cu_width_log2 + cu_height_log2);
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int processed_count = 0;
    int trav_order_index = 0;
    int s_src, s_src_c, s_dst, s_dst_c, i, j;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
    u8 scale_x = 1;
    u8 scale_y = 1;
    s_src = pic->stride_luma;
    s_dst = pic->stride_luma;
    s_src_c = pic->stride_chroma;
    s_dst_c = pic->stride_chroma;
    u8 is_hor_scan = mod_info_curr->sp_copy_direction;
    COM_SP_INFO *p_sp_info = mod_info_curr->string_copy_info;
    int* p_trav_scan_order = com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
    int str_length = 0;
    int offset_x, offset_y;
    int curr_x, curr_y;
    for (i = 0; i < mod_info_curr->sub_string_no; i++) 
    {
        if (p_sp_info[i].is_matched) 
        {
            str_length = p_sp_info[i].length;
            offset_x = p_sp_info[i].offset_x;
            offset_y = p_sp_info[i].offset_y;
            for (j = 0; j < str_length; j++) 
            {
                trav_order_index = p_trav_scan_order[processed_count];
                curr_x = GET_TRAV_X(trav_order_index, cu_width);
                curr_y = GET_TRAV_Y(trav_order_index, cu_width_log2);
                // luma
                if (channel == CHANNEL_LC || channel == CHANNEL_L)
                {
                    cur_pos = (y + curr_y) * s_dst + (x + curr_x);
#if SP_ALIGN_SIGN
                    ref_pos = (y + curr_y + offset_y) * s_src + (x + curr_x + offset_x);
#else
                    ref_pos = (y + curr_y - offset_y) * s_src + (x + curr_x - offset_x);
#endif
                    pic->y[cur_pos] = pic->y[ref_pos];
                }
                // chroma
                if (channel == CHANNEL_LC || channel == CHANNEL_C)
                {
                    cur_pos_c = ((y + curr_y) >> scale_y) * s_dst_c + ((x + curr_x) >> scale_x);
#if SP_ALIGN_SIGN
                    ref_pos_c = ((y + curr_y + offset_y) >> scale_y) * s_src_c + ((x + curr_x + offset_x) >> scale_x);
#else
                    ref_pos_c = ((y + curr_y - offset_y) >> scale_y) * s_src_c + ((x + curr_x - offset_x) >> scale_x);
#endif
                    if (!((curr_x & scale_x) || (curr_y & scale_y)))
                    {
                        pic->u[cur_pos_c] = pic->u[ref_pos_c];
                        pic->v[cur_pos_c] = pic->v[ref_pos_c];
                    }
                }
                processed_count++;
            }
        }
        else 
        {
            trav_order_index = p_trav_scan_order[processed_count];
            curr_x = GET_TRAV_X(trav_order_index, cu_width);
            curr_y = GET_TRAV_Y(trav_order_index, cu_width_log2);
            // luma
            if (channel == CHANNEL_LC || channel == CHANNEL_L)
            {
                cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                pic->y[cur_pos] = p_sp_info[i].pixel[Y_C];
            }
            // chroma
            if (channel == CHANNEL_LC || channel == CHANNEL_C)
            {
                cur_pos_c = ((y + curr_y) >> scale_y) * s_dst_c + ((x + curr_x) >> scale_x);
                if (!((curr_x & scale_x) || (curr_y & scale_y)))
                {
                    pic->u[cur_pos_c] = p_sp_info[i].pixel[U_C];
                    pic->v[cur_pos_c] = p_sp_info[i].pixel[V_C];
                }
            }
            processed_count++;
        }
    }
}
#endif