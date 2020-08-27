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


#if ENC_DEC_TRACE
FILE *fp_trace;
#if TRACE_RDO
#if TRACE_RDO_EXCLUDE_I
int fp_trace_print = 0;
#else
int fp_trace_print = 1;
#endif
#else
int fp_trace_print = 0;
#endif
int fp_trace_counter = 0;
#endif


COM_PIC * com_pic_alloc(PICBUF_ALLOCATOR * pa, int * ret)
{
    return com_picbuf_alloc(pa->width, pa->height, pa->pad_l, pa->pad_c, ret);
}

void com_pic_free(PICBUF_ALLOCATOR *pa, COM_PIC *pic)
{
    com_picbuf_free(pic);
}

int com_atomic_inc(volatile int *pcnt)
{
    int ret;
    ret = *pcnt;
    ret++;
    *pcnt = ret;
    return ret;
}

int com_atomic_dec(volatile int *pcnt)
{
    int ret;
    ret = *pcnt;
    ret--;
    *pcnt = ret;
    return ret;
}

COM_PIC * com_picbuf_alloc(int width, int height, int pad_l, int pad_c, int *err)
{
    COM_PIC *pic = NULL;
    COM_IMGB *imgb = NULL;
    int ret, opt, align[COM_IMGB_MAX_PLANE], pad[COM_IMGB_MAX_PLANE];
    int pic_width_in_scu, h_scu, f_scu, size;
    /* allocate PIC structure */
    pic = com_malloc(sizeof(COM_PIC));
    com_assert_gv(pic != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    com_mset(pic, 0, sizeof(COM_PIC));
    opt = COM_IMGB_OPT_NONE;
    /* set align value*/
    align[0] = MIN_CU_SIZE;
    align[1] = MIN_CU_SIZE >> 1;
    align[2] = MIN_CU_SIZE >> 1;
    /* set padding value*/
    pad[0] = pad_l;
    pad[1] = pad_c;
    pad[2] = pad_c;
    imgb = com_imgb_create(width, height, COM_COLORSPACE_YUV420, pad, align);
    com_assert_gv(imgb != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    /* set COM_PIC */
    pic->y     = imgb->addr_plane[0];
    pic->u     = imgb->addr_plane[1];
    pic->v     = imgb->addr_plane[2];
    pic->width_luma   = imgb->width[0];
    pic->height_luma   = imgb->height[0];
    pic->width_chroma   = imgb->width[1];
    pic->height_chroma   = imgb->height[1];
    pic->stride_luma   = STRIDE_IMGB2PIC(imgb->stride[0]);
    pic->stride_chroma   = STRIDE_IMGB2PIC(imgb->stride[1]);
    pic->padsize_luma = pad_l;
    pic->padsize_chroma = pad_c;
    pic->imgb  = imgb;
    /* allocate maps */
    pic_width_in_scu = (pic->width_luma + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    h_scu = (pic->height_luma + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    f_scu = pic_width_in_scu * h_scu;
    size = sizeof(s8) * f_scu * REFP_NUM;
    pic->map_refi = com_malloc_fast(size);
    com_assert_gv(pic->map_refi, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    com_mset_x64a(pic->map_refi, -1, size);
    size = sizeof(s16) * f_scu * REFP_NUM * MV_D;
    pic->map_mv = com_malloc_fast(size);
    com_assert_gv(pic->map_mv, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    com_mset_x64a(pic->map_mv, 0, size);
    if(err)
    {
        *err = COM_OK;
    }
    return pic;
ERR:
    if(pic)
    {
        com_mfree(pic->map_mv);
        com_mfree(pic->map_refi);
        com_mfree(pic);
    }
    if(err) *err = ret;
    return NULL;
}

void com_picbuf_free(COM_PIC *pic)
{
    COM_IMGB *imgb;
    if(pic)
    {
        imgb = pic->imgb;
        if(imgb)
        {
            imgb->release(imgb);
            pic->y = NULL;
            pic->u = NULL;
            pic->v = NULL;
            pic->width_luma = 0;
            pic->height_luma = 0;
            pic->width_chroma = 0;
            pic->height_chroma = 0;
            pic->stride_luma = 0;
            pic->stride_chroma = 0;
        }
        com_mfree(pic->map_mv);
        com_mfree(pic->map_refi);
        com_mfree(pic);
    }
}

static void picbuf_expand(pel *a, int s, int width, int height, int exp)
{
    int i, j;
    pel pixel;
    pel *src, *dst;
    /* left */
    src = a;
    dst = a - exp;
    for(i = 0; i < height; i++)
    {
        pixel = *src; /* get boundary pixel */
        for(j = 0; j < exp; j++)
        {
            dst[j] = pixel;
        }
        dst += s;
        src += s;
    }
    /* right */
    src = a + (width - 1);
    dst = a + width;
    for(i = 0; i < height; i++)
    {
        pixel = *src; /* get boundary pixel */
        for(j = 0; j < exp; j++)
        {
            dst[j] = pixel;
        }
        dst += s;
        src += s;
    }
    /* upper */
    src = a - exp;
    dst = a - exp - (exp * s);
    for(i = 0; i < exp; i++)
    {
        com_mcpy(dst, src, s*sizeof(pel));
        dst += s;
    }
    /* below */
    src = a + ((height - 1)*s) - exp;
    dst = a + ((height - 1)*s) - exp + s;
    for(i = 0; i < exp; i++)
    {
        com_mcpy(dst, src, s*sizeof(pel));
        dst += s;
    }
}

void com_picbuf_expand(COM_PIC *pic, int exp_l, int exp_c)
{
    picbuf_expand(pic->y, pic->stride_luma, pic->width_luma, pic->height_luma, exp_l);
    picbuf_expand(pic->u, pic->stride_chroma, pic->width_chroma, pic->height_chroma, exp_c);
    picbuf_expand(pic->v, pic->stride_chroma, pic->width_chroma, pic->height_chroma, exp_c);
}

int get_colocal_scup(int scup, int pic_width_in_scu, int pic_height_in_scu)
{
    const int mask = (-1) ^ 3;
    int bx = scup % pic_width_in_scu;
    int by = scup / pic_width_in_scu;
    int xpos = (bx & mask) + 2;
    int ypos = (by & mask) + 2;

    if (ypos >= pic_height_in_scu)
    {
        ypos = ((by & mask) + pic_height_in_scu) >> 1;
    }
    if (xpos >= pic_width_in_scu)
    {
        xpos = ((bx & mask) + pic_width_in_scu) >> 1;
    }

    return ypos * pic_width_in_scu + xpos;
}


void scaling_mv1(int ptr_cur, int ptr_cur_ref, int ptr_neb, int ptr_neb_ref, s16 mvp[MV_D], s16 mv[MV_D])
{
    int ratio;
    int t0 = ptr_neb * 2 - ptr_neb_ref * 2;
    int t1 = ptr_cur * 2 - ptr_cur_ref * 2;

    assert(t0 != 0 && t1 != 0);
    ratio = (1 << MV_SCALE_PREC) / t0 * t1; // note: divide first for constraining bit-depth

    int offset = 1 << (MV_SCALE_PREC - 1);
    s64 tmp_mv;
    tmp_mv = (s64)mvp[MV_X] * ratio;
    tmp_mv = tmp_mv == 0 ? 0 : tmp_mv > 0 ? ((tmp_mv + offset) >> MV_SCALE_PREC) : -((-tmp_mv + offset) >> MV_SCALE_PREC);
    mv[MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, tmp_mv);

    tmp_mv = (s64)mvp[MV_Y] * ratio;
    tmp_mv = tmp_mv == 0 ? 0 : tmp_mv > 0 ? ((tmp_mv + offset) >> MV_SCALE_PREC) : -((-tmp_mv + offset) >> MV_SCALE_PREC);
    mv[MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, tmp_mv);
}

void check_mvp_motion_availability(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, int neb_addr[NUM_AVS2_SPATIAL_MV], int valid_flag[NUM_AVS2_SPATIAL_MV], int lidx)
{
    int scup = mod_info_curr->scup; 
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_width_in_scu = info->pic_width_in_scu; 
    int h_scu = info->pic_height_in_scu;

    u32* map_scu = pic_map->map_scu; 
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;

    
    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    neb_addr[0] = scup - 1;                                     // A 正左
    neb_addr[1] = scup - pic_width_in_scu;                      // B 正上
    neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu;    // C 右上
    valid_flag[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]) && REFI_IS_VALID(map_refi[neb_addr[0]][lidx]);//是否超界and是否已编解码and是否帧内and0
    valid_flag[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]) && REFI_IS_VALID(map_refi[neb_addr[1]][lidx]);
    valid_flag[2] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]) && REFI_IS_VALID(map_refi[neb_addr[2]][lidx]);//需额外检测右上是否超界
    if (!valid_flag[2])
    {
        neb_addr[2] = scup - pic_width_in_scu - 1;              // D
        valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]) && REFI_IS_VALID(map_refi[neb_addr[2]][lidx]);
    }
}

__inline void copy_mv(s16 dst[MV_D], const s16 src[MV_D])
{
    dst[MV_X] = src[MV_X];
    dst[MV_Y] = src[MV_Y];
}

void init_motion(COM_MOTION *motion_dst)
{
    int i;
    for (i = 0; i < PRED_BI; i++)
    {
        motion_dst->ref_idx[i] = -1;
        motion_dst->mv[i][MV_X] = 0;
        motion_dst->mv[i][MV_Y] = 0;
    }
}

void create_motion(COM_MOTION *motion_dst, s16 mv_new[REFP_NUM][MV_D], s8 refi_new[REFP_NUM])
{
    int i;
    for (i = 0; i < PRED_BI; i++)
    {
        copy_mv(motion_dst->mv[i], mv_new[i]);
        motion_dst->ref_idx[i] = refi_new[i];
    }
}

void copy_motion(COM_MOTION *motion_dst, COM_MOTION motion_src)
{
    int i;
    for (i = 0; i < PRED_BI; i++)
    {
        copy_mv(motion_dst->mv[i], motion_src.mv[i]);
        motion_dst->ref_idx[i] = motion_src.ref_idx[i];
    }
}

void copy_motion_table(COM_MOTION *motion_dst, s8 *cnt_cands_dst, const COM_MOTION *motion_src, const s8 cnt_cands_src)
{
    *cnt_cands_dst = cnt_cands_src;
    memcpy(motion_dst, motion_src, sizeof(COM_MOTION) * cnt_cands_src);
}

int same_motion(COM_MOTION motion1, COM_MOTION motion2)
{
    if( motion1.ref_idx[PRED_L0] != motion2.ref_idx[PRED_L0] )
    {
        return 0;
    }
    if( REFI_IS_VALID( motion1.ref_idx[PRED_L0] ) && !SAME_MV( motion1.mv[PRED_L0], motion2.mv[PRED_L0] ) )
    {
        return 0;
    }

    if( motion1.ref_idx[PRED_L1] != motion2.ref_idx[PRED_L1] )
    {
        return 0;
    }
    if( REFI_IS_VALID( motion1.ref_idx[PRED_L1] ) && !SAME_MV( motion1.mv[PRED_L1], motion2.mv[PRED_L1] ) )
    {
        return 0;
    }
    return 1;
}

#if IBC_BVP
void create_block_motion(COM_BLOCK_MOTION *motion_dst, s16 mv_new[MV_D], int x, int y, int w, int h, int cnt
#if SP_PRED
    , int len
#endif
)
{

    motion_dst->mv[MV_X] = mv_new[MV_X] >> 2;
    motion_dst->mv[MV_Y] = mv_new[MV_Y] >> 2;

    motion_dst->x = x;
    motion_dst->y = y;
    motion_dst->w = w;
    motion_dst->h = h;
    motion_dst->cnt = cnt;
#if SP_PRED
    motion_dst->len = len;
#endif
}

void copy_block_motion(COM_BLOCK_MOTION *motion_dst, COM_BLOCK_MOTION motion_src)
{
    copy_mv(motion_dst->mv, motion_src.mv);
    motion_dst->x = motion_src.x;
    motion_dst->y = motion_src.y;
    motion_dst->w = motion_src.w;
    motion_dst->h = motion_src.h;
    motion_dst->cnt = motion_src.cnt;
#if SP_PRED
    motion_dst->len = motion_src.len;
#endif
}

void copy_block_motion_table(COM_BLOCK_MOTION *motion_dst, s8 *cnt_cands_dst, const COM_BLOCK_MOTION *motion_src, const s8 cnt_cands_src)
{
    *cnt_cands_dst = cnt_cands_src;
    memcpy(motion_dst, motion_src, sizeof(COM_BLOCK_MOTION) * cnt_cands_src);
}

int same_block_motion(COM_BLOCK_MOTION motion1, COM_BLOCK_MOTION motion2)
{
    if (!SAME_MV(motion1.mv, motion2.mv))
    {
        return 0;
    }
    return 1;
}
#endif

void check_umve_motion_availability(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, int neb_addr[NUM_AVS2_SPATIAL_MV], int valid_flag[NUM_AVS2_SPATIAL_MV])
{
    int scup = mod_info_curr->scup;
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;
    int tmp_flag[5];
    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    int cu_height_in_scu = cu_height >> MIN_CU_LOG2;

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    u32* map_scu = pic_map->map_scu;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;

    neb_addr[0] = scup + pic_width_in_scu * (cu_height_in_scu - 1) - 1; //F
    neb_addr[1] = scup - pic_width_in_scu + cu_width_in_scu - 1; //G
    neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu; //C
    neb_addr[3] = scup - 1; //A
    neb_addr[4] = scup - pic_width_in_scu - 1; //D
    tmp_flag[0] = x_scu > 0 && MCU_GET_CODED_FLAG( map_scu[neb_addr[0]] ) && !MCU_GET_INTRA_FLAG( map_scu[neb_addr[0]] );
    tmp_flag[1] = y_scu > 0 && MCU_GET_CODED_FLAG( map_scu[neb_addr[1]] ) && !MCU_GET_INTRA_FLAG( map_scu[neb_addr[1]] );
    tmp_flag[2] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG( map_scu[neb_addr[2]] ) && !MCU_GET_INTRA_FLAG( map_scu[neb_addr[2]] );
    tmp_flag[3] = x_scu > 0 && MCU_GET_CODED_FLAG( map_scu[neb_addr[3]] ) && !MCU_GET_INTRA_FLAG( map_scu[neb_addr[3]] );
    tmp_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG( map_scu[neb_addr[4]] ) && !MCU_GET_INTRA_FLAG( map_scu[neb_addr[4]] );
#if IBC_CHECK_BUGFIX
    tmp_flag[0] = tmp_flag[0] && !MCU_GET_IBC(map_scu[neb_addr[0]]);
    tmp_flag[1] = tmp_flag[1] && !MCU_GET_IBC(map_scu[neb_addr[1]]);
    tmp_flag[2] = tmp_flag[2] && !MCU_GET_IBC(map_scu[neb_addr[2]]);
    tmp_flag[3] = tmp_flag[3] && !MCU_GET_IBC(map_scu[neb_addr[3]]);
    tmp_flag[4] = tmp_flag[4] && !MCU_GET_IBC(map_scu[neb_addr[4]]);
#endif

    COM_MOTION m0, m1, m2;
    //F
    valid_flag[0] = tmp_flag[0];
    //G
    if( tmp_flag[0] && tmp_flag[1] )
    {
        create_motion( &m0, map_mv[neb_addr[1]], map_refi[neb_addr[1]] );
        create_motion( &m1, map_mv[neb_addr[0]], map_refi[neb_addr[0]] );
        valid_flag[1] = !same_motion( m0, m1 );
    }
    else if( !tmp_flag[0] && tmp_flag[1] )
    {
        valid_flag[1] = 1;
    }
    //C
    if( tmp_flag[1] && tmp_flag[2] )
    {
        create_motion( &m0, map_mv[neb_addr[2]], map_refi[neb_addr[2]] );
        create_motion( &m1, map_mv[neb_addr[1]], map_refi[neb_addr[1]] );
        valid_flag[2] = !same_motion( m0, m1 );
    }
    else if( !tmp_flag[1] && tmp_flag[2] )
    {
        valid_flag[2] = 1;
    }
    //A
    if( tmp_flag[0] && tmp_flag[3] )
    {
        create_motion( &m0, map_mv[neb_addr[3]], map_refi[neb_addr[3]] );
        create_motion( &m1, map_mv[neb_addr[0]], map_refi[neb_addr[0]] );
        valid_flag[3] = !same_motion( m0, m1 );
    }
    else if( !tmp_flag[0] && tmp_flag[3] )
    {
        valid_flag[3] = 1;
    }
    //D
    if( tmp_flag[4] )
    {
        create_motion( &m0, map_mv[neb_addr[4]], map_refi[neb_addr[4]] );
        if( tmp_flag[3] )
            create_motion( &m1, map_mv[neb_addr[3]], map_refi[neb_addr[3]] );
        else
            init_motion( &m1 );
        if( tmp_flag[1] )
            create_motion( &m2, map_mv[neb_addr[1]], map_refi[neb_addr[1]] );
        else
            init_motion( &m2 );
        valid_flag[4] = (!tmp_flag[3] || !same_motion( m0, m1 )) && (!tmp_flag[1] || !same_motion( m0, m2 ));
    }
}

#if EXT_AMVR_HMVP
void com_get_mvp_hmvp(COM_MOTION motion, int lidx, int ptr_cur, s8 cur_refi, s16 mvp[MV_D], COM_REFP(*refp)[REFP_NUM], int amvr_idx)
{
    int ptr_hmvp_ref;
    s8  refi_hmvp = motion.ref_idx[lidx];
    int ptr_cur_ref = refp[cur_refi][lidx].ptr;
    if (REFI_IS_VALID(refi_hmvp))
    {
        ptr_hmvp_ref = refp[refi_hmvp][lidx].ptr;
        scaling_mv1(ptr_cur, ptr_cur_ref, ptr_cur, ptr_hmvp_ref, motion.mv[lidx], mvp);
    }
    else
    {
        int lidx1 = (lidx == PRED_L0) ? PRED_L1 : PRED_L0;
        refi_hmvp = motion.ref_idx[lidx1];
        ptr_hmvp_ref = refp[refi_hmvp][lidx1].ptr;
        scaling_mv1(ptr_cur, ptr_cur_ref, ptr_cur, ptr_hmvp_ref, motion.mv[lidx1], mvp);
    }

    // clip MVP after rounding (rounding process might result in 32768)
    int mvp_x, mvp_y;
    com_mv_rounding_s32((s32)mvp[MV_X], (s32)mvp[MV_Y], &mvp_x, &mvp_y, amvr_idx, amvr_idx);
    mvp[MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_x);
    mvp[MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_y);
}
#endif

void com_derive_mvp(COM_INFO info, COM_MODE *mod_info_curr, int ptr, int ref_list, int ref_idx, int cnt_hmvp_cands, COM_MOTION *motion_cands, COM_MAP map, COM_REFP(*refp)[REFP_NUM], int mvr_idx, s16 mvp[MV_D])
{
    int scup = mod_info_curr->scup;
    int emvp_flag = mod_info_curr->mvp_from_hmvp_flag;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

#if EXT_AMVR_HMVP
    if (!emvp_flag)
    {
#endif
        com_get_mvp_default(&info, mod_info_curr, refp, &map, ptr, ref_list, ref_idx, mvr_idx, mvp);
#if EXT_AMVR_HMVP
    }
    else
    {
        if (cnt_hmvp_cands == 0)
        {
            mvp[MV_X] = 0;
            mvp[MV_Y] = 0;
        }
        else if (cnt_hmvp_cands < mvr_idx + 1)
        {
            COM_MOTION motion = motion_cands[cnt_hmvp_cands - 1];
            com_get_mvp_hmvp(motion, ref_list, ptr, ref_idx, mvp, refp, mvr_idx);
        }
        else
        {
            COM_MOTION motion = motion_cands[cnt_hmvp_cands - 1 - mvr_idx];
            com_get_mvp_hmvp(motion, ref_list, ptr, ref_idx, mvp, refp, mvr_idx);
        }
    }
#endif
}

#if IBC_BVP
void com_derive_bvp_list(u8 allowed_hbvp_num, COM_MODE *mod_info_curr, COM_BLOCK_MOTION* block_motion_cands, s8 cnt_hbvp_cands, s16(*bvp_cands)[MV_D], int *num_cands_all)
{
    u8 num_cands = 0;
    if (allowed_hbvp_num)
    {
        int idx;
        int cntCheck = 0;
        int cbvp_valid[MAX_NUM_BVP];
        s16 cbvp_cands[MAX_NUM_BVP][MV_D];

        for (idx = 0;idx < MAX_NUM_BVP;idx++)
        {
            cbvp_valid[idx] = 0;
            cbvp_cands[idx][MV_X] = 0;
            cbvp_cands[idx][MV_Y] = 0;
        }

        // fill BVP candidates
        for (idx = 0; idx < cnt_hbvp_cands; idx++)
        {
            COM_BLOCK_MOTION motion = block_motion_cands[idx];
#if SP_PRED
            if (motion.len > CBVP_TH_SIZE)
#else
            if (motion.w * motion.h > CBVP_TH_SIZE)
#endif
            {
                copy_mv(cbvp_cands[0], motion.mv);
                cbvp_valid[0] = 1;
            }
            if (motion.cnt > CBVP_TH_CNT)
            {
                copy_mv(cbvp_cands[1], motion.mv);
                cbvp_valid[1] = 1;
            }

            int x_pos = mod_info_curr->x_pos;
            int y_pos = mod_info_curr->y_pos;
            int w = mod_info_curr->cu_width;
            int h = mod_info_curr->cu_height;

            if (motion.x < x_pos && motion.y < y_pos)   // Left-Top
            {
                copy_mv(cbvp_cands[4], motion.mv);
                cbvp_valid[4] = 1;
            }
            else if (motion.x >= x_pos + w)             // Right-Top
            {
                copy_mv(cbvp_cands[5], motion.mv);
                cbvp_valid[5] = 1;
            }
            else if (motion.y >= y_pos + h)             // Left-Bottom
            {
                copy_mv(cbvp_cands[6], motion.mv);
                cbvp_valid[6] = 1;
            }
            else if (motion.y < y_pos)                  // Top
            {
                copy_mv(cbvp_cands[3], motion.mv);
                cbvp_valid[3] = 1;
            }
            else if (motion.x < x_pos)                  // Left
            {
                copy_mv(cbvp_cands[2], motion.mv);
                cbvp_valid[2] = 1;
            }
        }

        for (idx = 0;idx < MAX_NUM_BVP;idx++)
        {
            if (cbvp_valid[idx])
            {
                fill_bvp_list(bvp_cands, &num_cands, cbvp_cands[idx], cntCheck);
                if (idx < 2)
                {
                    cntCheck = num_cands;
                }
            }
        }
    }
    *num_cands_all = num_cands;
}
#endif


void com_get_mvp_default(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int ptr_cur, int lidx, s8 cur_refi,
                         u8 amvr_idx, s16 mvp[MV_D])
{
    int scup = mod_info_curr->scup;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;
    u32* map_scu = pic_map->map_scu;

    int cnt, hv, ptr_cur_ref;
    int mvPredType = MVPRED_xy_MIN;
    int rFrameL, rFrameU, rFrameUR;
    int neb_addr[NUM_AVS2_SPATIAL_MV], valid_flag[NUM_AVS2_SPATIAL_MV];
    s8 refi[NUM_AVS2_SPATIAL_MV];
    s16 MVPs[NUM_AVS2_SPATIAL_MV][MV_D];

    check_mvp_motion_availability(info, mod_info_curr, pic_map, neb_addr, valid_flag, lidx);
    ptr_cur_ref = refp[cur_refi][lidx].ptr;
    for (cnt = 0; cnt < NUM_AVS2_SPATIAL_MV; cnt++)//AVS3里，MVP的空间候选列表有三个？
    {
        if (valid_flag[cnt])
        {
            refi[cnt] = map_refi[neb_addr[cnt]][lidx];
            assert(REFI_IS_VALID(refi[cnt]));
#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
            if (refp[refi[cnt]][lidx].is_library_picture || refp[cur_refi][lidx].is_library_picture)
            {
                refi[cnt] = REFI_INVALID;
                MVPs[cnt][MV_X] = 0;
                MVPs[cnt][MV_Y] = 0;
            }
            else
#endif
            {
                int ptr_neb_ref = refp[refi[cnt]][lidx].ptr;
                scaling_mv1(ptr_cur, ptr_cur_ref, ptr_cur, ptr_neb_ref, map_mv[neb_addr[cnt]][lidx], MVPs[cnt]);
            }
        }
        else
        {
            refi[cnt] = REFI_INVALID;
            MVPs[cnt][MV_X] = 0;
            MVPs[cnt][MV_Y] = 0;
        }
    }
    rFrameL = refi[0];
    rFrameU = refi[1];
    rFrameUR = refi[2];
    if ((rFrameL != REFI_INVALID) && (rFrameU == REFI_INVALID) && (rFrameUR == REFI_INVALID))
    {
        mvPredType = MVPRED_L;
    }
    else if ((rFrameL == REFI_INVALID) && (rFrameU != REFI_INVALID) && (rFrameUR == REFI_INVALID))
    {
        mvPredType = MVPRED_U;
    }
    else if ((rFrameL == REFI_INVALID) && (rFrameU == REFI_INVALID) && (rFrameUR != REFI_INVALID))
    {
        mvPredType = MVPRED_UR;
    }

    for (hv = 0; hv < MV_D; hv++)
    {
        s32 mva = (s32)MVPs[0][hv], mvb = (s32)MVPs[1][hv], mvc = (s32)MVPs[2][hv];
        switch (mvPredType)
        {
        case MVPRED_xy_MIN:
            if ((mva < 0 && mvb > 0 && mvc > 0) || (mva > 0 && mvb < 0 && mvc < 0))
            {
                mvp[hv] = (s16)((mvb + mvc) / 2);
            }
            else if ((mvb < 0 && mva > 0 && mvc > 0) || (mvb > 0 && mva < 0 && mvc < 0))
            {
                mvp[hv] = (s16)((mvc + mva) / 2);
            }
            else if ((mvc < 0 && mva > 0 && mvb > 0) || (mvc > 0 && mva < 0 && mvb < 0))
            {
                mvp[hv] = (s16)((mva + mvb) / 2);
            }
            else
            {
                s32 mva_ext = abs(mva - mvb);
                s32 mvb_ext = abs(mvb - mvc);
                s32 mvc_ext = abs(mvc - mva);
                s32 pred_vec = min(mva_ext, min(mvb_ext, mvc_ext));
                if (pred_vec == mva_ext)
                {
                    mvp[hv] = (s16)((mva + mvb) / 2);
                }
                else if (pred_vec == mvb_ext)
                {
                    mvp[hv] = (s16)((mvb + mvc) / 2);
                }
                else
                {
                    mvp[hv] = (s16)((mvc + mva) / 2);
                }
            }
            break;
        case MVPRED_L:
            mvp[hv] = (s16)mva;
            break;
        case MVPRED_U:
            mvp[hv] = (s16)mvb;
            break;
        case MVPRED_UR:
            mvp[hv] = (s16)mvc;
            break;
        default:
            assert(0);
            break;
        }
    }

    // clip MVP after rounding (rounding process might result in 32768)
    int mvp_x, mvp_y;
    com_mv_rounding_s32((s32)mvp[MV_X], (s32)mvp[MV_Y], &mvp_x, &mvp_y, amvr_idx, amvr_idx);
    mvp[MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_x);
    mvp[MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mvp_y);
}

void derive_MHBskip_spatial_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 skip_pmv[PRED_DIR_NUM][REFP_NUM][MV_D], s8 skip_refi[PRED_DIR_NUM][REFP_NUM])
{

    int scup = mod_info_curr->scup;
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;

    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    int cu_height_in_scu = cu_height >> MIN_CU_LOG2;

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    u32* map_scu = pic_map->map_scu;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;

    int k;
    int last_bi_idx = 0;
    u8 L0_motion_found = 0, L1_motion_found = 0, BI_motion_found = 0;
    int neb_addr[NUM_SKIP_SPATIAL_MV];
    int valid_flag[NUM_SKIP_SPATIAL_MV];
    s8 refi[REFP_NUM];

    for (k = 0; k < PRED_DIR_NUM; k++)
    {
        skip_pmv[k][REFP_0][MV_X] = 0;
        skip_pmv[k][REFP_0][MV_Y] = 0;
        skip_pmv[k][REFP_1][MV_X] = 0;
        skip_pmv[k][REFP_1][MV_Y] = 0;
    }
    skip_refi[2][REFP_0] = 0;
    skip_refi[2][REFP_1] = REFI_INVALID;
    skip_refi[1][REFP_0] = REFI_INVALID;
    skip_refi[1][REFP_1] = 0;
    skip_refi[0][REFP_0] = 0;
    skip_refi[0][REFP_1] = 0;

    //! F: left-below neighbor (inside)
    neb_addr[0] = scup + (cu_height_in_scu - 1) * pic_width_in_scu - 1;
    valid_flag[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]);

    //! G: above-right neighbor (inside)
    neb_addr[1] = scup - pic_width_in_scu + cu_width_in_scu - 1;
    valid_flag[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]);

    //! C: above-right neighbor (outside)
    neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu;
    valid_flag[2] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]);

    //! A: left neighbor
    neb_addr[3] = scup - 1;
    valid_flag[3] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[3]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[3]]);

    //! B: above neighbor
    neb_addr[4] = scup - pic_width_in_scu;
    valid_flag[4] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[4]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[4]]);

    //! D: above-left neighbor
    neb_addr[5] = scup - pic_width_in_scu - 1;
    valid_flag[5] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[5]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[5]]);
#if IBC_CHECK_BUGFIX
    valid_flag[0] = valid_flag[0] && !MCU_GET_IBC(map_scu[neb_addr[0]]);
    valid_flag[1] = valid_flag[1] && !MCU_GET_IBC(map_scu[neb_addr[1]]);
    valid_flag[2] = valid_flag[2] && !MCU_GET_IBC(map_scu[neb_addr[2]]);
    valid_flag[3] = valid_flag[3] && !MCU_GET_IBC(map_scu[neb_addr[3]]);
    valid_flag[4] = valid_flag[4] && !MCU_GET_IBC(map_scu[neb_addr[4]]);
    valid_flag[5] = valid_flag[5] && !MCU_GET_IBC(map_scu[neb_addr[5]]);
#endif

    for (k = 0; k < NUM_SKIP_SPATIAL_MV; k++)
    {
        if (valid_flag[k])
        {
            refi[REFP_0] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_0]) ? map_refi[neb_addr[k]][REFP_0] : REFI_INVALID;
            refi[REFP_1] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_1]) ? map_refi[neb_addr[k]][REFP_1] : REFI_INVALID;
            // Search first L0
            if (REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]) && !L0_motion_found)
            {
                L0_motion_found = 1;
                skip_pmv[2][REFP_0][MV_X] = map_mv[neb_addr[k]][REFP_0][MV_X];
                skip_pmv[2][REFP_0][MV_Y] = map_mv[neb_addr[k]][REFP_0][MV_Y];
                skip_refi[2][REFP_0] = refi[REFP_0];
            }
            // Search first L1
            if (!REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]) && !L1_motion_found)
            {
                L1_motion_found = 1;
                skip_pmv[1][REFP_1][MV_X] = map_mv[neb_addr[k]][REFP_1][MV_X];
                skip_pmv[1][REFP_1][MV_Y] = map_mv[neb_addr[k]][REFP_1][MV_Y];
                skip_refi[1][REFP_1] = refi[REFP_1];
            }
            // Search first and last BI
            if (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
            {
                if (!BI_motion_found)
                {
                    skip_pmv[0][REFP_0][MV_X] = map_mv[neb_addr[k]][REFP_0][MV_X];
                    skip_pmv[0][REFP_0][MV_Y] = map_mv[neb_addr[k]][REFP_0][MV_Y];
                    skip_refi[0][REFP_0] = refi[REFP_0];
                    skip_pmv[0][REFP_1][MV_X] = map_mv[neb_addr[k]][REFP_1][MV_X];
                    skip_pmv[0][REFP_1][MV_Y] = map_mv[neb_addr[k]][REFP_1][MV_Y];
                    skip_refi[0][REFP_1] = refi[REFP_1];
                }
                BI_motion_found++;
                last_bi_idx = k; // save last BI index (first in order: D, B, A, C, G, F)
            }
        }
    }

    // combine L0+L1 to BI
    if (L0_motion_found && L1_motion_found && !BI_motion_found)
    {
        skip_pmv[0][REFP_0][MV_X] = skip_pmv[2][REFP_0][MV_X];
        skip_pmv[0][REFP_0][MV_Y] = skip_pmv[2][REFP_0][MV_Y];
        skip_refi[0][REFP_0] = skip_refi[2][REFP_0];
        skip_pmv[0][REFP_1][MV_X] = skip_pmv[1][REFP_1][MV_X];
        skip_pmv[0][REFP_1][MV_Y] = skip_pmv[1][REFP_1][MV_Y];
        skip_refi[0][REFP_1] = skip_refi[1][REFP_1];
    }
    // Separate last BI to L0
    if (!L0_motion_found && BI_motion_found)
    {
        skip_pmv[2][REFP_0][MV_X] = map_mv[neb_addr[last_bi_idx]][REFP_0][MV_X];
        skip_pmv[2][REFP_0][MV_Y] = map_mv[neb_addr[last_bi_idx]][REFP_0][MV_Y];
        skip_refi[2][REFP_0] = map_refi[neb_addr[last_bi_idx]][REFP_0];
        assert(REFI_IS_VALID(map_refi[neb_addr[last_bi_idx]][REFP_0]));
    }
    // Separate last BI to L1
    if (!L1_motion_found && BI_motion_found)
    {
        skip_pmv[1][REFP_1][MV_X] = map_mv[neb_addr[last_bi_idx]][REFP_1][MV_X];
        skip_pmv[1][REFP_1][MV_Y] = map_mv[neb_addr[last_bi_idx]][REFP_1][MV_Y];
        skip_refi[1][REFP_1] = map_refi[neb_addr[last_bi_idx]][REFP_1];
        assert(REFI_IS_VALID(map_refi[neb_addr[last_bi_idx]][REFP_1]));
    }

    assert(REFI_IS_VALID(skip_refi[2][REFP_0]) && (!REFI_IS_VALID(skip_refi[2][REFP_1])));
    assert((!REFI_IS_VALID(skip_refi[1][REFP_0])) && REFI_IS_VALID(skip_refi[1][REFP_1]));
    assert(REFI_IS_VALID(skip_refi[0][REFP_0]) && REFI_IS_VALID(skip_refi[0][REFP_1]));
}

#if AWP
#if !AWP_UNIMV_SIMP
BOOL com_check_same_uni_motion(s16 uni_cands0[REFP_NUM][MV_D], s8 uni_ref0[REFP_NUM], s16 uni_cands1[REFP_NUM][MV_D], s8 uni_ref1[REFP_NUM], s8 RefList)
{
    if (uni_ref0[RefList] != uni_ref1[RefList] || uni_cands0[RefList][MV_X] != uni_cands1[RefList][MV_X] || uni_cands0[RefList][MV_Y] != uni_cands1[RefList][MV_Y])
    {
        return FALSE;
    }
    return TRUE;
}
#endif

void com_set_uni_cand(s16 candidate[REFP_NUM][MV_D], s8 candRefIdx[REFP_NUM], s16 awp_uni_cands[REFP_NUM][MV_D], s8 awp_uni_refi[REFP_NUM], s8 RefList)
{
    if (RefList == REFP_0)
    {
        awp_uni_cands[REFP_0][MV_X] = candidate[REFP_0][MV_X];
        awp_uni_cands[REFP_0][MV_Y] = candidate[REFP_0][MV_Y];
        awp_uni_refi[REFP_0] = candRefIdx[REFP_0];
        awp_uni_cands[REFP_1][MV_X] = 0;
        awp_uni_cands[REFP_1][MV_Y] = 0;
        awp_uni_refi[REFP_1] = REFI_INVALID;
    }
    else
    {
        awp_uni_cands[REFP_0][MV_X] = 0;
        awp_uni_cands[REFP_0][MV_Y] = 0;
        awp_uni_refi[REFP_0] = REFI_INVALID;
        awp_uni_cands[REFP_1][MV_X] = candidate[REFP_1][MV_X];
        awp_uni_cands[REFP_1][MV_Y] = candidate[REFP_1][MV_Y];
        awp_uni_refi[REFP_1] = candRefIdx[REFP_1];
    }
    return;
}

void com_set_awp_mv_para(COM_MODE* mod_info_curr, s16 awp_uni_cands[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM])
{
    //set 1st uni cand
    mod_info_curr->awp_mv0[REFP_0][MV_X] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_X];
    mod_info_curr->awp_mv0[REFP_0][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_0][MV_Y];
    mod_info_curr->awp_refi0[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_0];
    mod_info_curr->awp_mv0[REFP_1][MV_X] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_X];
    mod_info_curr->awp_mv0[REFP_1][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx0][REFP_1][MV_Y];
    mod_info_curr->awp_refi0[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx0][REFP_1];
    //set 2nd uni cand
    mod_info_curr->awp_mv1[REFP_0][MV_X] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_X];
    mod_info_curr->awp_mv1[REFP_0][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_0][MV_Y];
    mod_info_curr->awp_refi1[REFP_0] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_0];
    mod_info_curr->awp_mv1[REFP_1][MV_X] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_X];
    mod_info_curr->awp_mv1[REFP_1][MV_Y] = awp_uni_cands[mod_info_curr->awp_idx1][REFP_1][MV_Y];
    mod_info_curr->awp_refi1[REFP_1] = awp_uni_refi[mod_info_curr->awp_idx1][REFP_1];
    return;
}

#if AWP_UNIMV_SIMP
u8 com_derive_awp_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM],
                               s16 awp_uni_cand[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM], int num_refp[REFP_NUM])
#else
u8 com_derive_awp_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM],
                               s16 awp_uni_cand[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM])
#endif
{
    int scup = mod_info_curr->scup;
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;
    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    int cu_height_in_scu = cu_height >> MIN_CU_LOG2;
    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    u32* map_scu = pic_map->map_scu;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;
    int i, j;
    u8  cnt = 0;
#if AWP_UNIMV_SIMP
    int neb_addr[5];
    int valid_flag[5] = { 0, 0, 0, 0, 0 };
#else
    int neb_addr[6];
    int valid_flag[6] = { 0, 0, 0, 0, 0, 0 };
    u8  Checked[6] = { 0, 0, 0, 0, 0, 0 };
#endif

    for (i = 0; i < AWP_MV_LIST_LENGTH; i++)
    {
        for (j = 0; j < REFP_NUM; j++)
        {
            awp_uni_refi[i][j] = REFI_INVALID;
        }
    }

    BOOL sameL0flag = FALSE;
    BOOL sameL1flag = FALSE;
    BOOL sameflag = FALSE;

    //! F: left-below neighbor (inside)
    neb_addr[0] = scup + (cu_height_in_scu - 1) * pic_width_in_scu - 1;
    valid_flag[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]);

    //! G: above-right neighbor (inside)
    neb_addr[1] = scup - pic_width_in_scu + cu_width_in_scu - 1;
    valid_flag[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]);

    //! C: above-right neighbor (outside)
    neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu;
    valid_flag[2] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]);

    //! A: left neighbor
    neb_addr[3] = scup - 1;
    valid_flag[3] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[3]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[3]]);

#if AWP_UNIMV_SIMP
    //! D: above-left neighbor
    neb_addr[4] = scup - pic_width_in_scu - 1;
    valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[4]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[4]]);
#else
    //! B: above neighbor
    neb_addr[4] = scup - pic_width_in_scu;
    valid_flag[4] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[4]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[4]]);

    //! D: above-left neighbor
    neb_addr[5] = scup - pic_width_in_scu - 1;
    valid_flag[5] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[5]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[5]]);
#endif

#if USE_IBC
    valid_flag[0] = valid_flag[0] && !MCU_GET_IBC(map_scu[neb_addr[0]]);
    valid_flag[1] = valid_flag[1] && !MCU_GET_IBC(map_scu[neb_addr[1]]);
    valid_flag[2] = valid_flag[2] && !MCU_GET_IBC(map_scu[neb_addr[2]]);
    valid_flag[3] = valid_flag[3] && !MCU_GET_IBC(map_scu[neb_addr[3]]);
    valid_flag[4] = valid_flag[4] && !MCU_GET_IBC(map_scu[neb_addr[4]]);
#if !AWP_UNIMV_SIMP
    valid_flag[5] = valid_flag[5] && !MCU_GET_IBC(map_scu[neb_addr[5]]);
#endif
#endif

#if AWP_UNIMV_SIMP 
    COM_MOTION m0, m1, m2;

    //G
    if (valid_flag[0] && valid_flag[1])
    {
        create_motion(&m0, map_mv[neb_addr[1]], map_refi[neb_addr[1]]);
        create_motion(&m1, map_mv[neb_addr[0]], map_refi[neb_addr[0]]);
        valid_flag[1] = !same_motion(m0, m1);
    }

    //C
    if (valid_flag[1] && valid_flag[2])
    {
        create_motion(&m0, map_mv[neb_addr[2]], map_refi[neb_addr[2]]);
        create_motion(&m1, map_mv[neb_addr[1]], map_refi[neb_addr[1]]);
        valid_flag[2] = !same_motion(m0, m1);
    }

    //A
    if (valid_flag[0] && valid_flag[3])
    {
        create_motion(&m0, map_mv[neb_addr[3]], map_refi[neb_addr[3]]);
        create_motion(&m1, map_mv[neb_addr[0]], map_refi[neb_addr[0]]);
        valid_flag[3] = !same_motion(m0, m1);
    }

    //D
    if (valid_flag[4])
    {
        create_motion(&m0, map_mv[neb_addr[4]], map_refi[neb_addr[4]]);
        if (valid_flag[3])
            create_motion(&m1, map_mv[neb_addr[3]], map_refi[neb_addr[3]]);
        else
            init_motion(&m1);
        if (valid_flag[1])
            create_motion(&m2, map_mv[neb_addr[1]], map_refi[neb_addr[1]]);
        else
            init_motion(&m2);
        valid_flag[4] = (!valid_flag[3] || !same_motion(m0, m1)) && (!valid_flag[1] || !same_motion(m0, m2));
    }

    s16 valid_mv_cand[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D];
    s8  valid_refi_cand[AWP_MV_LIST_LENGTH][REFP_NUM];
    for (i = 0; i < 5; i++)
    {
        if (valid_flag[i])
        {
            valid_mv_cand[cnt][REFP_0][MV_X] = map_mv[neb_addr[i]][REFP_0][MV_X];
            valid_mv_cand[cnt][REFP_0][MV_Y] = map_mv[neb_addr[i]][REFP_0][MV_Y];
            valid_refi_cand[cnt][REFP_0] = map_refi[neb_addr[i]][REFP_0];
            valid_mv_cand[cnt][REFP_1][MV_X] = map_mv[neb_addr[i]][REFP_1][MV_X];
            valid_mv_cand[cnt][REFP_1][MV_Y] = map_mv[neb_addr[i]][REFP_1][MV_Y];
            valid_refi_cand[cnt][REFP_1] = map_refi[neb_addr[i]][REFP_1];
            cnt++;

            if (cnt == AWP_MV_LIST_LENGTH - 1)
            {
                break;
            }
        }
    }

    valid_mv_cand[cnt][REFP_0][MV_X] = t_mv[REFP_0][MV_X];
    valid_mv_cand[cnt][REFP_0][MV_Y] = t_mv[REFP_0][MV_Y];
    valid_refi_cand[cnt][REFP_0]     = t_refi[REFP_0];
    valid_mv_cand[cnt][REFP_1][MV_X] = t_mv[REFP_1][MV_X];
    valid_mv_cand[cnt][REFP_1][MV_Y] = t_mv[REFP_1][MV_Y];
    valid_refi_cand[cnt][REFP_1]     = t_refi[REFP_1];
    cnt++;

#if AWP_ADD_SCALE
    if (cnt < AWP_MV_LIST_LENGTH)
    {
        assert(cnt>0);
        int loop = 0;
        for (int loop = 0; loop < 4; loop++) {
            valid_mv_cand[cnt][REFP_0][MV_X] = valid_mv_cand[0][REFP_0][MV_X];
            valid_mv_cand[cnt][REFP_0][MV_Y] = valid_mv_cand[0][REFP_0][MV_Y];
            valid_refi_cand[cnt][REFP_0] = valid_refi_cand[0][REFP_0];
            valid_mv_cand[cnt][REFP_1][MV_X] = valid_mv_cand[0][REFP_1][MV_X];
            valid_mv_cand[cnt][REFP_1][MV_Y] = valid_mv_cand[0][REFP_1][MV_Y];
            valid_refi_cand[cnt][REFP_1] = valid_refi_cand[0][REFP_1];
            
            for (int refp = 0; refp < REFP_NUM; refp++) {
                if (!REFI_IS_VALID(valid_refi_cand[cnt][refp])) {
                    continue;
                }
                int temp_val = 0;
                int sign = 0;
                int axis = MV_X;
                int scale_factor = 1;

                switch (loop)
                {
                case 0:
                    axis = MV_X;
                    scale_factor = 1;
                    break;
                case 1:
                    axis = MV_Y;
                    scale_factor = 1;
                    break;
                case 2:
                    axis = MV_X;
                    scale_factor = -1;
                    break;
                case 3:
                    axis = MV_Y;
                    scale_factor = -1;
                    break;
                default:
                    printf("\nerror in add scale loop\n");
                    break;
                }

                temp_val = abs(valid_mv_cand[cnt][refp][axis]);
                sign = valid_mv_cand[cnt][refp][axis] >= 0 ? 0 : 1;
                valid_mv_cand[cnt][refp][axis] = temp_val < 8 ? 8 : (temp_val <= 64 ? ((temp_val * (4 + scale_factor) + 2) >> 2) : (temp_val <= 128 ? ((temp_val * (8 + scale_factor) + 4) >> 3) : ((temp_val * (32 + scale_factor) + 16) >> 5)));
                if (sign) {
                    valid_mv_cand[cnt][refp][axis] = -valid_mv_cand[cnt][refp][axis];
                }
                if (loop>1 && temp_val < 8) {
                    valid_mv_cand[cnt][refp][axis] = -valid_mv_cand[cnt][refp][axis];
                }
            }

            cnt++;
            if (cnt == AWP_MV_LIST_LENGTH)
            {
                break;
            }
        }
    }
#else // AWP_ADD_SCALE

    for (; cnt < AWP_MV_LIST_LENGTH; cnt++)
    {
        valid_mv_cand[cnt][REFP_0][MV_X] = 0;
        valid_mv_cand[cnt][REFP_0][MV_Y] = 0;
        valid_refi_cand[cnt][REFP_0] = 0;
        valid_mv_cand[cnt][REFP_1][MV_X] = 0;
        valid_mv_cand[cnt][REFP_1][MV_Y] = 0;
        valid_refi_cand[cnt][REFP_1] = -1;
    }
#endif // AWP_ADD_SCALE
    for (i = 0; i < cnt; i++)
    {
        int parity = (i & 0x01);
        if (REFI_IS_VALID(valid_refi_cand[i][parity]))
        {
            com_set_uni_cand(valid_mv_cand[i], valid_refi_cand[i], awp_uni_cand[i], awp_uni_refi[i], parity);
        }
        else if (REFI_IS_VALID(valid_refi_cand[i][1 - parity]))
        {
            com_set_uni_cand(valid_mv_cand[i], valid_refi_cand[i], awp_uni_cand[i], awp_uni_refi[i], 1 - parity);
        }
        else
        {
            assert(0);
        }
    }
#else
    /* Add uni */
    for (i = 0; i < 6; i++)
    {
        if (valid_flag[i] && !(REFI_IS_VALID(map_refi[neb_addr[i]][REFP_0]) && REFI_IS_VALID(map_refi[neb_addr[i]][REFP_1])))
        {
            // check same
            Checked[i] = 1;
            sameflag = FALSE;
            for (int j = 0; j < cnt; j++)
            {
                if (com_check_same_uni_motion(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[j], awp_uni_refi[j], REFP_0) &&
                    com_check_same_uni_motion(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[j], awp_uni_refi[j], REFP_1))
                {
                    sameflag = TRUE;
                    break;
                }
            }

            if (sameflag == FALSE)
            {
                // uni prediction
                awp_uni_cand[cnt][REFP_0][MV_X] = map_mv[neb_addr[i]][REFP_0][MV_X];
                awp_uni_cand[cnt][REFP_0][MV_Y] = map_mv[neb_addr[i]][REFP_0][MV_Y];
                awp_uni_refi[cnt][REFP_0] = map_refi[neb_addr[i]][REFP_0];
                awp_uni_cand[cnt][REFP_1][MV_X] = map_mv[neb_addr[i]][REFP_1][MV_X];
                awp_uni_cand[cnt][REFP_1][MV_Y] = map_mv[neb_addr[i]][REFP_1][MV_Y];
                awp_uni_refi[cnt][REFP_1] = map_refi[neb_addr[i]][REFP_1];
                cnt++;

                if (cnt == AWP_MV_LIST_LENGTH - 1)
                {
                    break;
                }
            }
        }
    }

    /* Add bi */
    if (cnt < AWP_MV_LIST_LENGTH - 1)
    {
        for (i = 0; i < 6; i++)
        {
            if (valid_flag[i] && Checked[i] == 0)
            {
                // bi prediction
                sameL0flag = FALSE;
                for (int j = 0; j < cnt; j++)
                {
                    if (com_check_same_uni_motion(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[j], awp_uni_refi[j], REFP_0))
                    {
                        sameL0flag = TRUE;
                        break;
                    }
                }
                if (sameL0flag == FALSE)
                {
                    com_set_uni_cand(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[cnt], awp_uni_refi[cnt], REFP_0);
                    cnt++;
                    if (cnt == AWP_MV_LIST_LENGTH - 1)
                    {
                        break;
                    }
                }

                sameL1flag = FALSE;
                for (int j = 0; j < cnt; j++)
                {
                    if (com_check_same_uni_motion(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[j], awp_uni_refi[j], REFP_1))
                    {
                        sameL1flag = TRUE;
                        break;
                    }
                }
                if (sameL1flag == FALSE)
                {
                    com_set_uni_cand(map_mv[neb_addr[i]], map_refi[neb_addr[i]], awp_uni_cand[cnt], awp_uni_refi[cnt], REFP_1);
                    cnt++;
                    if (cnt == AWP_MV_LIST_LENGTH - 1)
                    {
                        break;
                    }
                }
            }
        }
    }

    //Add TMVP
    sameL0flag = FALSE;
    for (int j = 0; j < cnt; j++)
    {
        if (com_check_same_uni_motion(t_mv, t_refi, awp_uni_cand[j], awp_uni_refi[j], REFP_0))
        {
            sameL0flag = TRUE;
            break;
        }
    }
    if (sameL0flag == FALSE)
    {
        com_set_uni_cand(t_mv, t_refi, awp_uni_cand[cnt], awp_uni_refi[cnt], REFP_0);
        cnt++;
    }
    if (cnt == AWP_MV_LIST_LENGTH)
    {
        return cnt;
    }

    sameL1flag = FALSE;
    for (int j = 0; j < cnt; j++)
    {
        if (com_check_same_uni_motion(t_mv, t_refi, awp_uni_cand[j], awp_uni_refi[j], REFP_1))
        {
            sameL1flag = TRUE;
            break;
        }
    }
    if (sameL1flag == FALSE)
    {
        com_set_uni_cand(t_mv, t_refi, awp_uni_cand[cnt], awp_uni_refi[cnt], REFP_1);
        cnt++;
    }
#if AWP_ADD_SCALE
    if (cnt < AWP_MV_LIST_LENGTH)
    {
        assert(cnt > 0);
        int loop = 0;
        for (int loop = 0; loop < 4; loop++) {
            awp_uni_cand[cnt][REFP_0][MV_X] = awp_uni_cand[0][REFP_0][MV_X];
            awp_uni_cand[cnt][REFP_0][MV_Y] = awp_uni_cand[0][REFP_0][MV_Y];
            awp_uni_refi[cnt][REFP_0]       = awp_uni_refi[0][REFP_0];
            awp_uni_cand[cnt][REFP_1][MV_X] = awp_uni_cand[0][REFP_1][MV_X];
            awp_uni_cand[cnt][REFP_1][MV_Y] = awp_uni_cand[0][REFP_1][MV_Y];
            awp_uni_refi[cnt][REFP_1]       = awp_uni_refi[0][REFP_1];

            for (int refp = 0; refp < REFP_NUM; refp++) {
                if (!REFI_IS_VALID(awp_uni_refi[cnt][refp])) {
                    continue;
                }
                int temp_val = 0;
                int sign = 0;
                int axis = MV_X;
                int scale_factor = 1;

                switch (loop)
                {
                case 0:
                    axis = MV_X;
                    scale_factor = 1;
                    break;
                case 1:
                    axis = MV_Y;
                    scale_factor = 1;
                    break;
                case 2:
                    axis = MV_X;
                    scale_factor = -1;
                    break;
                case 3:
                    axis = MV_Y;
                    scale_factor = -1;
                    break;
                default:
                    printf("\nerror in add scale loop\n");
                    break;
                }

                temp_val = abs(awp_uni_cand[cnt][refp][axis]);
                sign = awp_uni_cand[cnt][refp][axis] >= 0 ? 0 : 1;
                awp_uni_cand[cnt][refp][axis] = temp_val < 8 ? 8 : (temp_val <= 64 ? ((temp_val * (4 + scale_factor) + 2) >> 2) : (temp_val <= 128 ? ((temp_val * (8 + scale_factor) + 4) >> 3) : ((temp_val * (32 + scale_factor) + 16) >> 5)));
                if (sign) {
                    awp_uni_cand[cnt][refp][axis] = -awp_uni_cand[cnt][refp][axis];
                }
                if (loop > 1 && temp_val < 8) {
                    awp_uni_cand[cnt][refp][axis] = -awp_uni_cand[cnt][refp][axis];
                }
            }

            cnt++;
            if (cnt == AWP_MV_LIST_LENGTH)
            {
                break;
            }
        }
    }
#endif // AWP_ADD_SCALE
    /*fill list for conformance*/
    if (cnt < AWP_MV_LIST_LENGTH)
    {
        for (int i = cnt; i < AWP_MV_LIST_LENGTH; i++)
        {
            awp_uni_cand[i][REFP_0][MV_X] = awp_uni_cand[cnt - 1][REFP_0][MV_X];
            awp_uni_cand[i][REFP_0][MV_Y] = awp_uni_cand[cnt - 1][REFP_0][MV_Y];
            awp_uni_refi[i][REFP_0] = awp_uni_refi[cnt - 1][REFP_0];
            awp_uni_cand[i][REFP_1][MV_X] = awp_uni_cand[cnt - 1][REFP_1][MV_X];
            awp_uni_cand[i][REFP_1][MV_Y] = awp_uni_cand[cnt - 1][REFP_1][MV_Y];
            awp_uni_refi[i][REFP_1] = awp_uni_refi[cnt - 1][REFP_1];
        }
    }
#endif

    return cnt;
}
#endif

#if AWP_MVR
void com_set_awp_mvr_mv_para(COM_MODE* mod_info_curr, s16 awp_final_mv0[REFP_NUM][MV_D], s8 awp_final_refi0[REFP_NUM], s16 awp_final_mv1[REFP_NUM][MV_D], s8 awp_final_refi1[REFP_NUM])
{
    //set 1st uni cand
    mod_info_curr->awp_mv0[REFP_0][MV_X] = awp_final_mv0[REFP_0][MV_X];
    mod_info_curr->awp_mv0[REFP_0][MV_Y] = awp_final_mv0[REFP_0][MV_Y];
    mod_info_curr->awp_refi0[REFP_0] = awp_final_refi0[REFP_0];
    mod_info_curr->awp_mv0[REFP_1][MV_X] = awp_final_mv0[REFP_1][MV_X];
    mod_info_curr->awp_mv0[REFP_1][MV_Y] = awp_final_mv0[REFP_1][MV_Y];
    mod_info_curr->awp_refi0[REFP_1] = awp_final_refi0[REFP_1];
    //set 2nd uni cand
    mod_info_curr->awp_mv1[REFP_0][MV_X] = awp_final_mv1[REFP_0][MV_X];
    mod_info_curr->awp_mv1[REFP_0][MV_Y] = awp_final_mv1[REFP_0][MV_Y];
    mod_info_curr->awp_refi1[REFP_0] = awp_final_refi1[REFP_0];
    mod_info_curr->awp_mv1[REFP_1][MV_X] = awp_final_mv1[REFP_1][MV_X];
    mod_info_curr->awp_mv1[REFP_1][MV_Y] = awp_final_mv1[REFP_1][MV_Y];
    mod_info_curr->awp_refi1[REFP_1] = awp_final_refi1[REFP_1];
    return;
}
#endif

#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
void derive_umve_base_motions(COM_REFP(*refp)[REFP_NUM], COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM], s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM])
#else
void derive_umve_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM], s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM])
#endif
{
    int scup = mod_info_curr->scup;
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;
    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    int cu_height_in_scu = cu_height >> MIN_CU_LOG2;
    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    u32* map_scu = pic_map->map_scu;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;
    int i, j;
    int cnt = 0;
    int neb_addr[5];
    int valid_flag[5] = { 0, 0, 0, 0, 0 };

    for (i = 0; i < UMVE_BASE_NUM; i++)
    {
        for (j = 0; j < REFP_NUM; j++)
        {
            umve_base_refi[i][j] = REFI_INVALID;
        }
    }

    //F -> G -> C -> A -> D
    check_umve_motion_availability(info, mod_info_curr, pic_map, neb_addr, valid_flag);

    for (i = 0; i < 5; i++)
    {
        if (valid_flag[i])
        {
#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
            if (REFI_IS_VALID(map_refi[neb_addr[i]][REFP_0]) && REFI_IS_VALID(map_refi[neb_addr[i]][REFP_1]) && (refp[map_refi[neb_addr[i]][REFP_0]][REFP_0].is_library_picture || refp[map_refi[neb_addr[i]][REFP_1]][REFP_1].is_library_picture))
            {
                continue;
            }
#endif
            if (REFI_IS_VALID(map_refi[neb_addr[i]][REFP_0]))
            {
                umve_base_pmv[cnt][REFP_0][MV_X] = map_mv[neb_addr[i]][REFP_0][MV_X];
                umve_base_pmv[cnt][REFP_0][MV_Y] = map_mv[neb_addr[i]][REFP_0][MV_Y];
                umve_base_refi[cnt][REFP_0] = map_refi[neb_addr[i]][REFP_0];
            }
            if (REFI_IS_VALID(map_refi[neb_addr[i]][REFP_1]))
            {
                umve_base_pmv[cnt][REFP_1][MV_X] = map_mv[neb_addr[i]][REFP_1][MV_X];
                umve_base_pmv[cnt][REFP_1][MV_Y] = map_mv[neb_addr[i]][REFP_1][MV_Y];
                umve_base_refi[cnt][REFP_1] = map_refi[neb_addr[i]][REFP_1];
            }
            cnt++;
        }
        if (cnt == UMVE_BASE_NUM)
        {
            break;
        }
    }
    if (cnt < UMVE_BASE_NUM)
    {
        umve_base_pmv[cnt][REFP_0][MV_X] = t_mv[REFP_0][MV_X];
        umve_base_pmv[cnt][REFP_0][MV_Y] = t_mv[REFP_0][MV_Y];
        umve_base_refi[cnt][REFP_0] = t_refi[REFP_0];

        umve_base_pmv[cnt][REFP_1][MV_X] = t_mv[REFP_1][MV_X];
        umve_base_pmv[cnt][REFP_1][MV_Y] = t_mv[REFP_1][MV_Y];
        umve_base_refi[cnt][REFP_1] = t_refi[REFP_1];
        cnt++;
    }

    if (cnt < UMVE_BASE_NUM)
    {
        umve_base_pmv[cnt][REFP_0][MV_X] = 0;
        umve_base_pmv[cnt][REFP_0][MV_Y] = 0;
        umve_base_refi[cnt][REFP_0] = 0;

        umve_base_pmv[cnt][REFP_1][MV_X] = 0;
        umve_base_pmv[cnt][REFP_1][MV_Y] = 0;
        umve_base_refi[cnt][REFP_1] = -1;
        cnt++;
    }
}

#if UMVE_ENH 
void derive_umve_final_motions(int umve_idx, COM_REFP(*refp)[REFP_NUM], int cur_poc, s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM], s16 umve_final_pmv[UMVE_BASE_NUM * UMVE_MAX_REFINE_NUM_SEC_SET][REFP_NUM][MV_D], s8 umve_final_refi[UMVE_BASE_NUM * UMVE_MAX_REFINE_NUM_SEC_SET][REFP_NUM], BOOL isUMVESecSet)
#else
void derive_umve_final_motions(int umve_idx, COM_REFP(*refp)[REFP_NUM], int cur_poc, s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM], s16 umve_final_pmv[UMVE_BASE_NUM * UMVE_MAX_REFINE_NUM][REFP_NUM][MV_D], s8 umve_final_refi[UMVE_BASE_NUM * UMVE_MAX_REFINE_NUM][REFP_NUM])
#endif
{
#if UMVE_ENH
    int base_idx, refine_step, direction;
    if (!isUMVESecSet)
    {
        base_idx = umve_idx / UMVE_MAX_REFINE_NUM;
        refine_step = (umve_idx - (base_idx * UMVE_MAX_REFINE_NUM)) / 4;
        direction = umve_idx - base_idx * UMVE_MAX_REFINE_NUM - refine_step * 4;
    }
    else
    {
        base_idx = umve_idx / UMVE_MAX_REFINE_NUM_SEC_SET;
        refine_step = (umve_idx - (base_idx * UMVE_MAX_REFINE_NUM_SEC_SET)) / 4;
        direction = umve_idx - base_idx * UMVE_MAX_REFINE_NUM_SEC_SET - refine_step * 4;
    }
#else
    int base_idx = umve_idx / UMVE_MAX_REFINE_NUM;
    int refine_step = (umve_idx - (base_idx * UMVE_MAX_REFINE_NUM)) / 4;
    int direction = umve_idx - base_idx * UMVE_MAX_REFINE_NUM - refine_step * 4;
#endif
    s32 mv_offset[REFP_NUM][MV_D];
    int ref_mvd0, ref_mvd1;

    const int ref_mvd_cands[5] = { 1, 2, 4, 8, 16 };
#if UMVE_ENH
    const int ref_mvd_cands_sec_set[8] = { 1, 2, 4, 8, 16, 32, 64, 128 };
#endif
    const int refi0 = umve_base_refi[base_idx][REFP_0];
    const int refi1 = umve_base_refi[base_idx][REFP_1];

#if UMVE_ENH
    if (!isUMVESecSet)
    {
        ref_mvd0 = ref_mvd_cands[refine_step];
        ref_mvd1 = ref_mvd_cands[refine_step];
    }
    else
    {
        ref_mvd0 = ref_mvd_cands_sec_set[refine_step];
        ref_mvd1 = ref_mvd_cands_sec_set[refine_step];
    }
#else
    ref_mvd0 = ref_mvd_cands[refine_step];
    ref_mvd1 = ref_mvd_cands[refine_step];
#endif
    if (REFI_IS_VALID(refi0) && REFI_IS_VALID(refi1))
    {
        const int poc0 = refp[refi0][REFP_0].ptr * 2;
        const int poc1 = refp[refi1][REFP_1].ptr * 2;
        int list0_weight = 1 << MV_SCALE_PREC;
        int list1_weight = 1 << MV_SCALE_PREC;
        int list0_sign = 1;
        int list1_sign = 1;

        cur_poc *= 2;

        if (abs(poc1 - cur_poc) >= abs(poc0 - cur_poc))
        {
            list0_weight = (1 << MV_SCALE_PREC) / (abs(poc1 - cur_poc)) * abs(poc0 - cur_poc);
            if ((poc1 - cur_poc) * (poc0 - cur_poc) < 0)
            {
                list0_sign = -1;
            }
        }
        else
        {
            list1_weight = (1 << MV_SCALE_PREC) / (abs(poc0 - cur_poc)) * abs(poc1 - cur_poc);
            if ((poc1 - cur_poc) * (poc0 - cur_poc) < 0)
            {
                list1_sign = -1;
            }
        }

        ref_mvd0 = (list0_weight * ref_mvd0 + (1 << (MV_SCALE_PREC - 1))) >> MV_SCALE_PREC;
        ref_mvd1 = (list1_weight * ref_mvd1 + (1 << (MV_SCALE_PREC - 1))) >> MV_SCALE_PREC;

        ref_mvd0 = COM_CLIP3(-(1 << 15), (1 << 15) - 1, list0_sign * ref_mvd0);
        ref_mvd1 = COM_CLIP3(-(1 << 15), (1 << 15) - 1, list1_sign * ref_mvd1);

        if (direction == 0)
        {
            mv_offset[REFP_0][MV_X] = ref_mvd0;
            mv_offset[REFP_0][MV_Y] = 0;
            mv_offset[REFP_1][MV_X] = ref_mvd1;
            mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (direction == 1)
        {
            mv_offset[REFP_0][MV_X] = -ref_mvd0;
            mv_offset[REFP_0][MV_Y] = 0;
            mv_offset[REFP_1][MV_X] = -ref_mvd1;
            mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (direction == 2)
        {
            mv_offset[REFP_0][MV_X] = 0;
            mv_offset[REFP_0][MV_Y] = ref_mvd0;
            mv_offset[REFP_1][MV_X] = 0;
            mv_offset[REFP_1][MV_Y] = ref_mvd1;
        }
        else
        {
            mv_offset[REFP_0][MV_X] = 0;
            mv_offset[REFP_0][MV_Y] = -ref_mvd0;
            mv_offset[REFP_1][MV_X] = 0;
            mv_offset[REFP_1][MV_Y] = -ref_mvd1;
        }

        s32 mv_x = (s32)umve_base_pmv[base_idx][REFP_0][MV_X] + mv_offset[REFP_0][MV_X];
        s32 mv_y = (s32)umve_base_pmv[base_idx][REFP_0][MV_Y] + mv_offset[REFP_0][MV_Y];

        umve_final_pmv[umve_idx][REFP_0][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
        umve_final_pmv[umve_idx][REFP_0][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
        umve_final_refi[umve_idx][REFP_0] = umve_base_refi[base_idx][REFP_0];

        mv_x = (s32)umve_base_pmv[base_idx][REFP_1][MV_X] + mv_offset[REFP_1][MV_X];
        mv_y = (s32)umve_base_pmv[base_idx][REFP_1][MV_Y] + mv_offset[REFP_1][MV_Y];

        umve_final_pmv[umve_idx][REFP_1][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
        umve_final_pmv[umve_idx][REFP_1][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
        umve_final_refi[umve_idx][REFP_1] = umve_base_refi[base_idx][REFP_1];
    }
    else if (REFI_IS_VALID(refi0))
    {
        if (direction == 0)
        {
            mv_offset[REFP_0][MV_X] = ref_mvd0;
            mv_offset[REFP_0][MV_Y] = 0;
        }
        else if (direction == 1)
        {
            mv_offset[REFP_0][MV_X] = -ref_mvd0;
            mv_offset[REFP_0][MV_Y] = 0;
        }
        else if (direction == 2)
        {
            mv_offset[REFP_0][MV_X] = 0;
            mv_offset[REFP_0][MV_Y] = ref_mvd0;
        }
        else // (direction == 3)
        {
            mv_offset[REFP_0][MV_X] = 0;
            mv_offset[REFP_0][MV_Y] = -ref_mvd0;
        }

        s32 mv_x = (s32)umve_base_pmv[base_idx][REFP_0][MV_X] + mv_offset[REFP_0][MV_X];
        s32 mv_y = (s32)umve_base_pmv[base_idx][REFP_0][MV_Y] + mv_offset[REFP_0][MV_Y];

        umve_final_pmv[umve_idx][REFP_0][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
        umve_final_pmv[umve_idx][REFP_0][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
        umve_final_refi[umve_idx][REFP_0] = umve_base_refi[base_idx][REFP_0];

        umve_final_pmv[umve_idx][REFP_1][MV_X] = 0;
        umve_final_pmv[umve_idx][REFP_1][MV_Y] = 0;
        umve_final_refi[umve_idx][REFP_1] = REFI_INVALID;
    }
    else if (REFI_IS_VALID(refi1))
    {
        if (direction == 0)
        {
            mv_offset[REFP_1][MV_X] = ref_mvd1;
            mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (direction == 1)
        {
            mv_offset[REFP_1][MV_X] = -ref_mvd1;
            mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (direction == 2)
        {
            mv_offset[REFP_1][MV_X] = 0;
            mv_offset[REFP_1][MV_Y] = ref_mvd1;
        }
        else // (direction == 3)
        {
            mv_offset[REFP_1][MV_X] = 0;
            mv_offset[REFP_1][MV_Y] = -ref_mvd1;
        }
        s32 mv_x = (s32)umve_base_pmv[base_idx][REFP_1][MV_X] + mv_offset[REFP_1][MV_X];
        s32 mv_y = (s32)umve_base_pmv[base_idx][REFP_1][MV_Y] + mv_offset[REFP_1][MV_Y];

        umve_final_pmv[umve_idx][REFP_1][MV_X] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_x);
        umve_final_pmv[umve_idx][REFP_1][MV_Y] = (s16)COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_y);
        umve_final_refi[umve_idx][REFP_1] = umve_base_refi[base_idx][REFP_1];

        umve_final_pmv[umve_idx][REFP_0][MV_X] = 0;
        umve_final_pmv[umve_idx][REFP_0][MV_Y] = 0;
        umve_final_refi[umve_idx][REFP_0] = REFI_INVALID;
    }
    else
    {
        umve_final_pmv[umve_idx][REFP_0][MV_X] = 0;
        umve_final_pmv[umve_idx][REFP_0][MV_Y] = 0;
        umve_final_refi[umve_idx][REFP_0] = REFI_INVALID;

        umve_final_pmv[umve_idx][REFP_1][MV_X] = 0;
        umve_final_pmv[umve_idx][REFP_1][MV_Y] = 0;
        umve_final_refi[umve_idx][REFP_1] = REFI_INVALID;
    }
}

#if AFFINE_UMVE
void derive_affine_umve_final_motion(s8 refi[REFP_NUM], int affine_umve_idx, s32 affine_mv_offset[REFP_NUM][MV_D])
{
    if (affine_umve_idx == -1)
    {
        affine_mv_offset[REFP_0][MV_X] = 0;
        affine_mv_offset[REFP_0][MV_Y] = 0;
        affine_mv_offset[REFP_1][MV_X] = 0;
        affine_mv_offset[REFP_1][MV_Y] = 0;
        return;
    }

    int affine_refine_step = affine_umve_idx / 4;
    int affine_direction = affine_umve_idx - affine_refine_step * 4;
    int ref_mvd0, ref_mvd1;

    const int ref_mvd_cands[5] = { 1, 2, 4, 8, 16 };

    ref_mvd0 = ref_mvd_cands[affine_refine_step];
    ref_mvd1 = ref_mvd_cands[affine_refine_step];
    if (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
        if (affine_direction == 0)
        {
            affine_mv_offset[REFP_0][MV_X] = ref_mvd0;
            affine_mv_offset[REFP_0][MV_Y] = 0;
            affine_mv_offset[REFP_1][MV_X] = ref_mvd1;
            affine_mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (affine_direction == 1)
        {
            affine_mv_offset[REFP_0][MV_X] = -ref_mvd0;
            affine_mv_offset[REFP_0][MV_Y] = 0;
            affine_mv_offset[REFP_1][MV_X] = -ref_mvd1;
            affine_mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (affine_direction == 2)
        {
            affine_mv_offset[REFP_0][MV_X] = 0;
            affine_mv_offset[REFP_0][MV_Y] = ref_mvd0;
            affine_mv_offset[REFP_1][MV_X] = 0;
            affine_mv_offset[REFP_1][MV_Y] = ref_mvd1;
        }
        else
        {
            affine_mv_offset[REFP_0][MV_X] = 0;
            affine_mv_offset[REFP_0][MV_Y] = -ref_mvd0;
            affine_mv_offset[REFP_1][MV_X] = 0;
            affine_mv_offset[REFP_1][MV_Y] = -ref_mvd1;
        }
    }
    else if (REFI_IS_VALID(refi[REFP_0]))
    {
        if (affine_direction == 0)
        {
            affine_mv_offset[REFP_0][MV_X] = ref_mvd0;
            affine_mv_offset[REFP_0][MV_Y] = 0;
        }
        else if (affine_direction == 1)
        {
            affine_mv_offset[REFP_0][MV_X] = -ref_mvd0;
            affine_mv_offset[REFP_0][MV_Y] = 0;
        }
        else if (affine_direction == 2)
        {
            affine_mv_offset[REFP_0][MV_X] = 0;
            affine_mv_offset[REFP_0][MV_Y] = ref_mvd0;
        }
        else // (affine_direction == 3)
        {
            affine_mv_offset[REFP_0][MV_X] = 0;
            affine_mv_offset[REFP_0][MV_Y] = -ref_mvd0;
        }
    }
    else
    {
        if (affine_direction == 0)
        {
            affine_mv_offset[REFP_1][MV_X] = ref_mvd1;
            affine_mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (affine_direction == 1)
        {
            affine_mv_offset[REFP_1][MV_X] = -ref_mvd1;
            affine_mv_offset[REFP_1][MV_Y] = 0;
        }
        else if (affine_direction == 2)
        {
            affine_mv_offset[REFP_1][MV_X] = 0;
            affine_mv_offset[REFP_1][MV_Y] = ref_mvd1;
        }
        else // (affine_direction == 3)
        {
            affine_mv_offset[REFP_1][MV_X] = 0;
            affine_mv_offset[REFP_1][MV_Y] = -ref_mvd1;
        }
    }
    return;
}
#endif

#if AWP_MVR
void derive_awp_mvr_final_motion(int awp_mvr_idx, COM_REFP(*refp)[REFP_NUM], s8 refi[REFP_NUM], s32 awp_mvr_offset[REFP_NUM][MV_D])
{
    int listCur = -1;
    if (REFI_IS_VALID(refi[0]) && !REFI_IS_VALID(refi[1]))
    {
        listCur = 0;
    }
    else if (!REFI_IS_VALID(refi[0]) && REFI_IS_VALID(refi[1]))
    {
        listCur = 1;
    }

    const int ref_mvd_cands[5] = { 1, 2, 4, 8, 16 };
    int refine_step_idx = awp_mvr_idx / AWP_MVR_DIR;
    int direction = awp_mvr_idx - refine_step_idx * 4;
    int ref_mvd = ref_mvd_cands[refine_step_idx];

    if (direction == 0)
    {
        awp_mvr_offset[listCur][MV_X] = ref_mvd;
        awp_mvr_offset[listCur][MV_Y] = 0;
        awp_mvr_offset[1 - listCur][MV_X] = 0;
        awp_mvr_offset[1 - listCur][MV_Y] = 0;
    }
    else if (direction == 1)
    {
        awp_mvr_offset[listCur][MV_X] = -ref_mvd;
        awp_mvr_offset[listCur][MV_Y] = 0;
        awp_mvr_offset[1 - listCur][MV_X] = 0;
        awp_mvr_offset[1 - listCur][MV_Y] = 0;
    }
    else if (direction == 2)
    {
        awp_mvr_offset[listCur][MV_X] = 0;
        awp_mvr_offset[listCur][MV_Y] = ref_mvd;
        awp_mvr_offset[1 - listCur][MV_X] = 0;
        awp_mvr_offset[1 - listCur][MV_Y] = 0;
    }
    else
    {
        awp_mvr_offset[listCur][MV_X] = 0;
        awp_mvr_offset[listCur][MV_Y] = -ref_mvd;
        awp_mvr_offset[1 - listCur][MV_X] = 0;
        awp_mvr_offset[1 - listCur][MV_Y] = 0;
    }
}
#endif

void print_motion(COM_MOTION motion)
{
    printf(" %d %d %d", motion.mv[PRED_L0][MV_X], motion.mv[PRED_L0][MV_Y], motion.ref_idx[PRED_L0]);
    printf(" %d %d %d", motion.mv[PRED_L1][MV_X], motion.mv[PRED_L1][MV_Y], motion.ref_idx[PRED_L1]);
    printf("\n");
}

void update_skip_candidates(COM_MOTION motion_cands[ALLOWED_HMVP_NUM], s8 *num_cands, const int max_hmvp_num, s16 mv_new[REFP_NUM][MV_D], s8 refi_new[REFP_NUM])
{
    int i, equal_idx = -1;
    COM_MOTION motion_curr;
    if (!REFI_IS_VALID(refi_new[REFP_0]) && !REFI_IS_VALID(refi_new[REFP_1]))
    {
        return;
    }
    create_motion(&motion_curr, mv_new, refi_new);
    assert(*num_cands <= max_hmvp_num);
    for (i = *num_cands - 1; i >= 0; i--)
    {
        if (same_motion(motion_cands[i], motion_curr))
        {
            equal_idx = i;
            break;
        }
    }
    if(equal_idx == -1) // can be simplified
    {
        if (*num_cands < max_hmvp_num)
        {
            (*num_cands)++;
        }
        else
        {
            int valid_pos = min(max_hmvp_num - 1, *num_cands);
            for (i = 1; i <= valid_pos; i++)
            {
                copy_motion(&motion_cands[i - 1], motion_cands[i]);
            }
        }
        
        copy_motion(&motion_cands[*num_cands - 1], motion_curr);
    }
    else
    {
        for (i = equal_idx; i < *num_cands - 1; i++)
        {
            copy_motion(&motion_cands[i], motion_cands[i + 1]);
        }
        copy_motion(&motion_cands[*num_cands - 1], motion_curr);
    }
    
#if PRINT_HMVP_FIFO
    static int hmvp_trace_cnt = 0;
    printf("\nhmvp table cnt %d\n", hmvp_trace_cnt++);
    for (int j = 0; j < *num_cands; j++)
    {
        printf("motion %d", j);
        print_motion(motion_cands[*num_cands - j - 1]);
    }
#endif
}

void fill_skip_candidates(COM_MOTION motion_cands[ALLOWED_HMVP_NUM], s8 *num_cands, const int num_hmvp_cands, 
#if MVAP
    const int num_mvap_cands,
#endif
    s16 mv_new[REFP_NUM][MV_D], s8 refi_new[REFP_NUM], int bRemDuplicate)
{
#if MVAP
    int maxNumSkipCand = TRADITIONAL_SKIP_NUM + max(num_hmvp_cands, num_mvap_cands);
#else
    int maxNumSkipCand = TRADITIONAL_SKIP_NUM + num_hmvp_cands;
#endif
    int i;
    COM_MOTION motion_curr;
    assert(REFI_IS_VALID(refi_new[REFP_0]) || REFI_IS_VALID(refi_new[REFP_1]));

    create_motion(&motion_curr, mv_new, refi_new);
    assert(*num_cands <= maxNumSkipCand);
    if (bRemDuplicate) //! No duplicate motion information exists among HMVP candidates, so duplicate removal is not needed for HMVP itself
    {
#if MVAP
        for (i = 0; i < TRADITIONAL_SKIP_NUM; i++)
#else
        for (i = 0; i < *num_cands; i++)
#endif
        {
            if (same_motion(motion_cands[i], motion_curr))
            {
                return;
            }
        }
    }
    if (*num_cands < maxNumSkipCand)
    {
        copy_motion(&motion_cands[*num_cands], motion_curr);
        (*num_cands)++;
    }
}

#if IBC_BVP
void update_ibc_skip_candidates(COM_BLOCK_MOTION block_motion_cands[ALLOWED_HBVP_NUM], s8 *num_cands, const int max_hbvp_num, s16 mv_new[MV_D], int x, int y, int w, int h
#if SP_PRED
    , int len
#endif
)
{
    int i, equal_idx = -1;
    COM_BLOCK_MOTION block_motion_curr;
    create_block_motion(&block_motion_curr, mv_new, x, y, w, h, 0
#if SP_PRED
        , len
#endif
    );
    assert(*num_cands <= max_hbvp_num);
    for (i = *num_cands - 1; i >= 0; i--)
    {
        if (same_block_motion(block_motion_cands[i], block_motion_curr))
        {
            equal_idx = i;
            block_motion_curr.cnt = block_motion_cands[i].cnt + 1;
#if SP_PRED
            if (block_motion_curr.len < block_motion_cands[i].len)
#else
            if (block_motion_curr.w * block_motion_curr.h < block_motion_cands[i].w * block_motion_cands[i].h)
#endif
            {
                block_motion_curr.w = block_motion_cands[i].w;
                block_motion_curr.h = block_motion_cands[i].h;
#if SP_PRED
                block_motion_curr.len = block_motion_cands[i].len;
#endif
            }
            break;
        }
    }
    if (equal_idx == -1)
    {
        if (*num_cands < max_hbvp_num)
        {
            (*num_cands)++;
        }
        else
        {
            int valid_pos = min(max_hbvp_num - 1, *num_cands);
            for (i = 1; i <= valid_pos; i++)
            {
                copy_block_motion(&block_motion_cands[i - 1], block_motion_cands[i]);
            }
        }
        copy_block_motion(&block_motion_cands[*num_cands - 1], block_motion_curr);
    }
    else
    {
        for (i = equal_idx; i < *num_cands - 1; i++)
        {
            copy_block_motion(&block_motion_cands[i], block_motion_cands[i + 1]);
        }
        copy_block_motion(&block_motion_cands[*num_cands - 1], block_motion_curr);
    }
}

void fill_bvp_list(s16(*bv_list)[MV_D], u8 *num_cands, const s16 bv_new[MV_D], int num_check)
{
    int i;
    int maxNumCand = MAX_NUM_BVP;
    assert(*num_cands <= maxNumCand);
    for (i = 0; i < num_check; i++)
    {
        if (SAME_MV(bv_list[i], bv_new))
        {
            return;
        }
    }

    if (*num_cands < maxNumCand)
    {
        copy_mv(bv_list[*num_cands], bv_new);
        (*num_cands)++;
    }
}
#endif
#if USE_SP
int check_sp_offset(s16 offset, s16 offset_y, s8 n_recent_num, COM_MOTION n_recent_offset[SP_RECENT_CANDS])
{
    int i = 0;
    for (i = 0; i < n_recent_num; i++)
    {
        if (offset == n_recent_offset[i].mv[0][MV_X] && offset_y == n_recent_offset[i].mv[0][MV_Y])
        {
            return i;
        }
    }
    return n_recent_num;
}

void com_derive_sp_offset(s8 cnt_sp_recent_cands, COM_MOTION *motion_cands, int sp_n_idx, s16 mvp[MV_D])
{
    if (cnt_sp_recent_cands == 0)
    {
        mvp[MV_X] = 0;
        mvp[MV_Y] = 0;
    }
    else if (cnt_sp_recent_cands < sp_n_idx + 1)
    {
        COM_MOTION motion = motion_cands[cnt_sp_recent_cands - 1];
        mvp[MV_X] = motion.mv[0][MV_X];
        mvp[MV_Y] = motion.mv[0][MV_Y];
    }
    else
    {
        COM_MOTION motion = motion_cands[cnt_sp_recent_cands - 1 - sp_n_idx];
        mvp[MV_X] = motion.mv[0][MV_X];
        mvp[MV_Y] = motion.mv[0][MV_Y];
    }
}

void update_sp_recent_cands(COM_MOTION *sps_cands, s8 *num_cands, COM_SP_INFO *p_SPinfo, u16 str_num, u8 dir)
{
    int i, equal_idx = -1;
    s16 offset_curr[MV_D] = { 0 };
    for (int j = 0; j < str_num; j++)
    {
        offset_curr[MV_X] = p_SPinfo[j].offset_x;
        offset_curr[MV_Y] = p_SPinfo[j].offset_y;
        if (p_SPinfo[j].is_matched == 0)
        {
            continue;
        }
        if (offset_curr[MV_X] == 0 && offset_curr[MV_Y] == 1 && dir == TRUE)//HORIZONTAL
        {
            continue;
        }
        if (offset_curr[MV_X] == 1 && offset_curr[MV_Y] == 0 && dir == FALSE)//VERTICAL
        {
            continue;
        }
        assert(*num_cands <= SP_RECENT_CANDS);
        for (i = *num_cands - 1; i >= 0; i--)
        {
            if (SAME_MV(offset_curr, sps_cands[i].mv[0]))
            {
                equal_idx = i;
                break;
            }
        }
        if (equal_idx == -1) 
        {
            if (*num_cands < SP_RECENT_CANDS)
            {
                (*num_cands)++;
            }
            else
            {
                int valid_pos = min(SP_RECENT_CANDS - 1, *num_cands);
                for (i = 1; i <= valid_pos; i++)
                {
                    sps_cands[i - 1].mv[0][MV_X] = sps_cands[i].mv[0][MV_X];
                    sps_cands[i - 1].mv[0][MV_Y] = sps_cands[i].mv[0][MV_Y];
                }
            }
            sps_cands[*num_cands - 1].mv[0][MV_X] = offset_curr[MV_X];
            sps_cands[*num_cands - 1].mv[0][MV_Y] = offset_curr[MV_Y];
        }
        else
        {
            for (i = equal_idx; i < *num_cands - 1; i++)
            {
                sps_cands[i].mv[0][MV_X] = sps_cands[i + 1].mv[0][MV_X];
                sps_cands[i].mv[0][MV_Y] = sps_cands[i + 1].mv[0][MV_Y];
            }
            sps_cands[*num_cands - 1].mv[0][MV_X] = offset_curr[MV_X];
            sps_cands[*num_cands - 1].mv[0][MV_Y] = offset_curr[MV_Y];
        }
    }
}
#endif

#if MVAP
void fill_neighbor_motions(u8 ibc_flag, int scup, int cu_width, int cu_height, int pic_width_in_scu, int h_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4])
{
    s32 i                    = 0;
    s32 j                    = 0;
    s32 x_scu                = scup % pic_width_in_scu;
    s32 y_scu                = scup / pic_width_in_scu;
    s32 cu_width_in_scu      = cu_width >> MIN_CU_LOG2;
    s32 cu_height_in_scu     = cu_height >> MIN_CU_LOG2;
    s32 neb_addr             = 0;
    s32 valid_flag           = 0;
    s32 first_avaiable_index = -1;
    s32 first_avaiable       = 1;

    if (x_scu)
    {
        for (i = 0, j = cu_width_in_scu + cu_height_in_scu - 1; i < (cu_width_in_scu + cu_height_in_scu); i++, j--)
        {
            neb_addr   = scup - 1 + j * pic_width_in_scu;
            valid_flag = (y_scu + cu_height_in_scu + cu_width_in_scu - 1 - i < h_scu) && MCU_GET_CODED_FLAG(map_scu[neb_addr]) && (!MCU_GET_INTRA_FLAG(map_scu[neb_addr]));
#if USE_IBC
            if (ibc_flag)
            {
                valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
            }
#endif
            if (valid_flag)
            {
                if (first_avaiable)
                {
                    first_avaiable_index = i;
                    first_avaiable       = 0;
                }
            }
            else if (first_avaiable_index != -1)
            {
                neighbor_motions[i].mv[REFP_0][MV_X] = neighbor_motions[i - 1].mv[REFP_0][MV_X];
                neighbor_motions[i].mv[REFP_0][MV_Y] = neighbor_motions[i - 1].mv[REFP_0][MV_Y];
                neighbor_motions[i].ref_idx[REFP_0]  = neighbor_motions[i - 1].ref_idx[REFP_0];

                neighbor_motions[i].mv[REFP_1][MV_X] = neighbor_motions[i - 1].mv[REFP_1][MV_X];
                neighbor_motions[i].mv[REFP_1][MV_Y] = neighbor_motions[i - 1].mv[REFP_1][MV_Y];
                neighbor_motions[i].ref_idx[REFP_1]  = neighbor_motions[i - 1].ref_idx[REFP_1];
            }
            else if (first_avaiable_index == -1)
            {
                neighbor_motions[i].mv[REFP_0][MV_X] = 0;
                neighbor_motions[i].mv[REFP_0][MV_Y] = 0;
                neighbor_motions[i].ref_idx[REFP_0]  = 0;

                neighbor_motions[i].mv[REFP_1][MV_X] = 0;
                neighbor_motions[i].mv[REFP_1][MV_Y] = 0;
                neighbor_motions[i].ref_idx[REFP_1]  = -1;

                first_avaiable_index = i;
            }
        }
    }

    if (x_scu && y_scu)
    {
        neb_addr   = scup - pic_width_in_scu - 1;
        valid_flag = MCU_GET_CODED_FLAG(map_scu[neb_addr]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr]);
#if USE_IBC
        if (ibc_flag)
        {
            valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
        }
#endif
        if (valid_flag)
        {
            if (first_avaiable)
            {
                first_avaiable_index = cu_width_in_scu + cu_height_in_scu;
                first_avaiable       = 0;
            }
        }
        else if (first_avaiable_index != -1)
        {
            neighbor_motions[cu_width_in_scu + cu_height_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].mv[REFP_0][MV_X];
            neighbor_motions[cu_width_in_scu + cu_height_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].mv[REFP_0][MV_Y];
            neighbor_motions[cu_width_in_scu + cu_height_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].ref_idx[REFP_0];

            neighbor_motions[cu_width_in_scu + cu_height_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].mv[REFP_1][MV_X];
            neighbor_motions[cu_width_in_scu + cu_height_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].mv[REFP_1][MV_Y];
            neighbor_motions[cu_width_in_scu + cu_height_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1].ref_idx[REFP_1];
        }
        else if (first_avaiable_index == -1)
        {
            neighbor_motions[i].mv[REFP_0][MV_X] = 0;
            neighbor_motions[i].mv[REFP_0][MV_Y] = 0;
            neighbor_motions[i].ref_idx[REFP_0]  = 0;

            neighbor_motions[i].mv[REFP_1][MV_X] = 0;
            neighbor_motions[i].mv[REFP_1][MV_Y] = 0;
            neighbor_motions[i].ref_idx[REFP_1]  = -1;

            first_avaiable_index = i;
        }
    }

    if (y_scu)
    {
        for (i = cu_width_in_scu + cu_height_in_scu + 1, j = 0; i < (2 * (cu_width_in_scu + cu_height_in_scu) + 1); i++, j++)
        {
            neb_addr   = scup - pic_width_in_scu + j;
            valid_flag = x_scu + j < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr]);
#if USE_IBC
            if (ibc_flag)
            {
                valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
            }
#endif
            if (valid_flag)
            {
                if (first_avaiable)
                {
                    first_avaiable_index = i;
                    first_avaiable       = 0;
                }
            }
            else if (first_avaiable_index != -1)
            {
                neighbor_motions[i].mv[REFP_0][MV_X] = neighbor_motions[i - 1].mv[REFP_0][MV_X];
                neighbor_motions[i].mv[REFP_0][MV_Y] = neighbor_motions[i - 1].mv[REFP_0][MV_Y];
                neighbor_motions[i].ref_idx[REFP_0]  = neighbor_motions[i - 1].ref_idx[REFP_0];

                neighbor_motions[i].mv[REFP_1][MV_X] = neighbor_motions[i - 1].mv[REFP_1][MV_X];
                neighbor_motions[i].mv[REFP_1][MV_Y] = neighbor_motions[i - 1].mv[REFP_1][MV_Y];
                neighbor_motions[i].ref_idx[REFP_1]  = neighbor_motions[i - 1].ref_idx[REFP_1];
            }
            else if (first_avaiable_index == -1)
            {
                neighbor_motions[i].mv[REFP_0][MV_X] = 0;
                neighbor_motions[i].mv[REFP_0][MV_Y] = 0;
                neighbor_motions[i].ref_idx[REFP_0]  = 0;

                neighbor_motions[i].mv[REFP_1][MV_X] = 0;
                neighbor_motions[i].mv[REFP_1][MV_Y] = 0;
                neighbor_motions[i].ref_idx[REFP_1]  = -1;

                first_avaiable_index = i;
            }
        }
    }
}

void copy_available_motions(u8 ibc_flag, int scup, int cu_width, int cu_height, int pic_width_in_scu, int h_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4])
{
    s32 i                = 0;
    s32 j                = 0;
    s32 x_scu            = scup % pic_width_in_scu;
    s32 y_scu            = scup / pic_width_in_scu;
    s32 cu_width_in_scu  = cu_width >> MIN_CU_LOG2;
    s32 cu_height_in_scu = cu_height >> MIN_CU_LOG2;
    s32 neb_addr         = 0;
    s32 valid_flag       = 0;

    for (i = 0; i < (cu_width_in_scu * 2 + cu_height_in_scu * 2 + 1); i++)
    {
        neighbor_motions[i].mv[REFP_0][MV_X] =
        neighbor_motions[i].mv[REFP_0][MV_Y] =
        neighbor_motions[i].mv[REFP_1][MV_X] =
        neighbor_motions[i].mv[REFP_1][MV_Y] = 0;
        neighbor_motions[i].ref_idx[REFP_0]  =
        neighbor_motions[i].ref_idx[REFP_1]  = REFI_INVALID;
    }

    if (x_scu)
    {
        for (i = 0, j = cu_width_in_scu + cu_height_in_scu - 1; i < (cu_width_in_scu + cu_height_in_scu); i++, j--)
        {
            neb_addr   = scup - 1 + j * pic_width_in_scu;
            valid_flag = (y_scu + cu_height_in_scu + cu_width_in_scu - 1 - i < h_scu) && MCU_GET_CODED_FLAG(map_scu[neb_addr]) && (!MCU_GET_INTRA_FLAG(map_scu[neb_addr]));
#if USE_IBC
            if (ibc_flag)
            {
                valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
            }
#endif
            if (valid_flag)
            {
                copy_neighbor_motion(i, neb_addr, map_mv, map_refi, neighbor_motions);
            }
        }
    }

    if (x_scu && y_scu)
    {
        neb_addr   = scup - pic_width_in_scu - 1;
        valid_flag = MCU_GET_CODED_FLAG(map_scu[neb_addr]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr]);
#if USE_IBC
        if (ibc_flag)
        {
            valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
        }
#endif
        if (valid_flag)
        {
            copy_neighbor_motion(cu_width_in_scu + cu_height_in_scu, neb_addr, map_mv, map_refi, neighbor_motions);
        }
    }

    if (y_scu)
    {
        for (i = cu_width_in_scu + cu_height_in_scu + 1, j = 0; i < (2 * (cu_width_in_scu + cu_height_in_scu) + 1); i++, j++)
        {
            neb_addr   = scup - pic_width_in_scu + j;
            valid_flag = x_scu + j < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr]);
#if USE_IBC
            if (ibc_flag)
            {
                valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
            }
#endif
            if (valid_flag)
            {
                copy_neighbor_motion(i, neb_addr, map_mv, map_refi, neighbor_motions);
            }
        }
    }
}

int same_neighbor_motion(COM_MOTION motion1, COM_MOTION motion2)
{
    if ((!REFI_IS_VALID(motion1.ref_idx[PRED_L0])) && (!REFI_IS_VALID(motion1.ref_idx[PRED_L1])))
    {
        return 1;
    }

    if ((!REFI_IS_VALID(motion2.ref_idx[PRED_L0])) && (!REFI_IS_VALID(motion2.ref_idx[PRED_L1])))
    {
        return 1;
    }

    if (motion1.ref_idx[PRED_L0] != motion2.ref_idx[PRED_L0])
    {
        return 0;
    }
    if (REFI_IS_VALID(motion1.ref_idx[PRED_L0]) && !SAME_MV(motion1.mv[PRED_L0], motion2.mv[PRED_L0]))
    {
        return 0;
    }

    if (motion1.ref_idx[PRED_L1] != motion2.ref_idx[PRED_L1])
    {
        return 0;
    }
    if (REFI_IS_VALID(motion1.ref_idx[PRED_L1]) && !SAME_MV(motion1.mv[PRED_L1], motion2.mv[PRED_L1]))
    {
        return 0;
    }
    return 1;
}

void copy_neighbor_motion(s32 neighbor_idx, s32 neb_addr, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4])
{
    if (REFI_IS_VALID(map_refi[neb_addr][REFP_0]))
    {
        neighbor_motions[neighbor_idx].mv[REFP_0][MV_X] = map_mv[neb_addr][REFP_0][MV_X];
        neighbor_motions[neighbor_idx].mv[REFP_0][MV_Y] = map_mv[neb_addr][REFP_0][MV_Y];
        neighbor_motions[neighbor_idx].ref_idx[REFP_0]  = map_refi[neb_addr][REFP_0];
    }

    if (REFI_IS_VALID(map_refi[neb_addr][REFP_1]))
    {
        neighbor_motions[neighbor_idx].mv[REFP_1][MV_X] = map_mv[neb_addr][REFP_1][MV_X];
        neighbor_motions[neighbor_idx].mv[REFP_1][MV_Y] = map_mv[neb_addr][REFP_1][MV_Y];
        neighbor_motions[neighbor_idx].ref_idx[REFP_1]  = map_refi[neb_addr][REFP_1];
    }
}

void get_valid_mvap_mode(s32 num_cands, s32 scup, s32 pic_width_in_scu, s32 cu_width, s32 cu_height, s32 *valid_mvap_num, s32 *valid_mvap_index, COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], s8 refi_cands[MAX_SKIP_NUM][REFP_NUM])
{
    s32 cu_width_in_scu                   = cu_width >> MIN_CU_LOG2;
    s32 cu_height_in_scu                  = cu_height >> MIN_CU_LOG2;
    s32 x_scu                             = scup % pic_width_in_scu;
    s32 y_scu                             = scup / pic_width_in_scu;
    s32 idx                               = 0;
    s32 diff_flag                         = 0;
    s32 i                                 = 0;
    s32 stride                            = 0;
    s32 left_scu_num                      = 0;
    s32 mvap_type[ALLOWED_MVAP_NUM]       = { 0 };
    s32 prunning_stride[ALLOWED_MVAP_NUM] = { 0 };

    mvap_type[HORIZONTAL]            = mvap_type[HORIZONTAL_DOWN] = x_scu ? 1 : 0;
    mvap_type[VERTICAL]              = mvap_type[VERTICAL_RIGHT]  = y_scu ? 1 : 0;
    mvap_type[HORIZONTAL_UP]         = (x_scu && y_scu) ? 1 : 0;
    prunning_stride[VERTICAL]        =
    prunning_stride[HORIZONTAL_DOWN] = (cu_width >> 3);
    prunning_stride[HORIZONTAL]      =
    prunning_stride[VERTICAL_RIGHT]  = (cu_height >> 3);

    *valid_mvap_num = 0;
    stride = prunning_stride[HORIZONTAL];
    left_scu_num = (cu_height_in_scu / prunning_stride[HORIZONTAL]) + (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]);

    if (mvap_type[HORIZONTAL])
    {
        for (idx = (cu_width_in_scu + cu_height_in_scu - 1), i = 0; idx >= prunning_stride[HORIZONTAL_DOWN]; idx -= stride, i++)
        {
            if (idx < cu_width_in_scu)
            {
                stride = prunning_stride[HORIZONTAL_DOWN];
            }

            if (!same_neighbor_motion(neighbor_motions[idx], neighbor_motions[idx - stride]))
            {
                diff_flag |= (1 << (left_scu_num - 2 - i));
            }
        }
    }

    stride = prunning_stride[VERTICAL];
    if (mvap_type[VERTICAL])
    {
        for (idx = (cu_width_in_scu + cu_height_in_scu + 1), i = 0; idx <= (2 * (cu_width_in_scu + cu_height_in_scu) - prunning_stride[VERTICAL_RIGHT]); idx += stride, i++)
        {
            if (idx > (2 * cu_width_in_scu + cu_height_in_scu))
            {
                stride = prunning_stride[VERTICAL_RIGHT];
            }

            if (!same_neighbor_motion(neighbor_motions[idx], neighbor_motions[idx + stride]))
            {
                diff_flag |= (1 << (left_scu_num + 1 + i));
            }
        }
    }

    if (mvap_type[HORIZONTAL_UP])
    {
        if (!same_neighbor_motion(neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1], neighbor_motions[cu_width_in_scu + cu_height_in_scu]))
        {
            diff_flag |= (1 << (left_scu_num - 1));
        }

        if (!same_neighbor_motion(neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1], neighbor_motions[cu_width_in_scu + cu_height_in_scu]))
        {
            diff_flag |= (1 << left_scu_num);
        }
    }

    for (idx = 0; idx < ALLOWED_MVAP_NUM; idx++)
    {
        if (mvap_type[idx])
        {
            if (have_diff_motion(cu_width_in_scu, cu_height_in_scu, idx, diff_flag, prunning_stride))
            {
                mvap_type[idx] = 2;
            }
        }
    }

    for (idx = 0; idx < ALLOWED_MVAP_NUM; idx++)
    {
        if (2 == mvap_type[idx])
        {
            valid_mvap_index[(*valid_mvap_num)++] = idx;
            refi_cands[num_cands][REFP_0]         = 0;
            refi_cands[num_cands][REFP_1]         = -1;
            num_cands++;
        }
    }
}

int have_diff_motion(s32 cu_width_in_scu, s32 cu_height_in_scu, s32  list_idx, s32 diff_flag, s32 prunning_stride[ALLOWED_MVAP_NUM])
{
    s32 left_bit = 0;
    s32 right_bit = 0;

    switch (list_idx)
    {
    case HORIZONTAL:
    {
        right_bit = (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]);
        left_bit  = 32 - (cu_height_in_scu / prunning_stride[HORIZONTAL] - 1);
        break;
    }
    case VERTICAL:
    {
        right_bit = (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]) + (cu_height_in_scu / prunning_stride[HORIZONTAL]) + 1;
        left_bit  = 32 - (cu_width_in_scu / prunning_stride[VERTICAL] - 1);
        break;
    }
    case HORIZONTAL_UP:
    {
        right_bit = (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]) + 1;
        left_bit  = 32 - ((cu_height_in_scu / prunning_stride[HORIZONTAL]) + (cu_width_in_scu / prunning_stride[VERTICAL]) - 2);
        break;
    }
    case HORIZONTAL_DOWN:
    {
        right_bit = 0;
        left_bit  = 32 - ((cu_height_in_scu / prunning_stride[HORIZONTAL]) + (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]) - 2);
        break;
    }
    case VERTICAL_RIGHT:
    {
        right_bit = (cu_width_in_scu / prunning_stride[HORIZONTAL_DOWN]) + (cu_height_in_scu / prunning_stride[HORIZONTAL]) + 2;
        left_bit  = 32 - ((cu_width_in_scu / prunning_stride[VERTICAL]) + (cu_height_in_scu / prunning_stride[VERTICAL_RIGHT]) - 2);
        break;
    }
    }

    s32 temp_flag = (diff_flag >> right_bit) << left_bit;
    if (temp_flag)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void set_mvap_mvfield(s32 cu_width_in_scu, s32 cu_height_in_scu, s32 mvap_idx, COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], COM_MOTION tmp_cu_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)])
{
    s32 h = 0;
    s32 w = 0;

    switch (mvap_idx)
    {
    case HORIZONTAL:
    {
        for (h = 0; h < cu_height_in_scu; h++)
        {
            for (w = 0; w < cu_width_in_scu; w++)
            {
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].mv[REFP_0][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].mv[REFP_0][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].mv[REFP_1][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].mv[REFP_1][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].ref_idx[REFP_0];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 1 - 2 * (h >> 1)].ref_idx[REFP_1];
            }
        }

        break;
    }

    case VERTICAL:
    {
        for (h = 0; h < cu_height_in_scu; h++)
        {
            for (w = 0; w < cu_width_in_scu; w++)
            {
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].mv[REFP_0][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].mv[REFP_0][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].mv[REFP_1][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].mv[REFP_1][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].ref_idx[REFP_0];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 1 + 2 * (w >> 1)].ref_idx[REFP_1];
            }
        }
        break;
    }

    case HORIZONTAL_UP:
    {
        for (h = 0; h < cu_height_in_scu; h++)
        {
            for (w = 0; w < cu_width_in_scu; w++)
            {
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].mv[REFP_0][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].mv[REFP_0][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].mv[REFP_1][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].mv[REFP_1][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].ref_idx[REFP_0];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) + 2 * (w >> 1)].ref_idx[REFP_1];
            }
        }
        break;
    }

    case HORIZONTAL_DOWN:
    {
        for (h = 0; h < cu_height_in_scu; h++)
        {
            for (w = 0; w < cu_width_in_scu; w++)
            {
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].mv[REFP_0][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].mv[REFP_0][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].mv[REFP_1][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].mv[REFP_1][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].ref_idx[REFP_0];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu - 2 * (h >> 1) - 2 * (w >> 1) - 3].ref_idx[REFP_1];
            }
        }
        break;
    }

    case VERTICAL_RIGHT:
    {
        for (h = 0; h < cu_height_in_scu; h++)
        {
            for (w = 0; w < cu_width_in_scu; w++)
            {
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].mv[REFP_0][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].mv[REFP_0][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].mv[REFP_1][MV_X];
                tmp_cu_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].mv[REFP_1][MV_Y];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].ref_idx[REFP_0];
                tmp_cu_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1]  = neighbor_motions[cu_width_in_scu + cu_height_in_scu + 2 * (h >> 1) + 2 * (w >> 1) + 3].ref_idx[REFP_1];
            }
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

void derive_mvap_motions(u8 ibc_flag, s32 num_cands, s32 scup, s32 cu_width, s32 cu_height, s32 pic_width_in_scu, s32 pic_height_in_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], s32 *valid_mvap_num, s32 *valid_mvap_index, s8 refi_cands[MAX_SKIP_NUM][REFP_NUM])
{
    if ((cu_width < MIN_SUB_BLOCK_SIZE) || (cu_height < MIN_SUB_BLOCK_SIZE) || (cu_width == MIN_SUB_BLOCK_SIZE && cu_height == MIN_SUB_BLOCK_SIZE))
    {
        *valid_mvap_num = 0;
        return;
    }

    copy_available_motions(ibc_flag, scup, cu_width, cu_height, pic_width_in_scu, pic_height_in_scu, map_scu, map_mv, map_refi, neighbor_motions);
    get_valid_mvap_mode(num_cands, scup, pic_width_in_scu, cu_width, cu_height, valid_mvap_num, valid_mvap_index, neighbor_motions, refi_cands);
    fill_neighbor_motions(ibc_flag, scup, cu_width, cu_height, pic_width_in_scu, pic_height_in_scu, map_scu, map_mv, map_refi, neighbor_motions);
}
#endif

void get_hmvp_skip_cands(const COM_MOTION motion_cands[ALLOWED_HMVP_NUM], const u8 num_cands, s16(*skip_mvs)[REFP_NUM][MV_D], s8(*skip_refi)[REFP_NUM])
{
    int i, dir;
    for (i = 0; i < num_cands; i++)
    {
        for (dir = 0; dir < PRED_BI; dir++)
        {
            copy_mv(skip_mvs[i][dir], motion_cands[i].mv[dir]);
            skip_refi[i][dir] = motion_cands[i].ref_idx[dir];
        }
    }
}

void get_col_mv_from_list0(COM_REFP refp[REFP_NUM], u32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D])
{
    s16 mvc[MV_D];
    assert(REFI_IS_VALID(refp[REFP_0].map_refi[scup_co][REFP_0]));

    mvc[MV_X] = refp[REFP_0].map_mv[scup_co][0][MV_X];
    mvc[MV_Y] = refp[REFP_0].map_mv[scup_co][0][MV_Y];
    int ptr_col = refp[REFP_0].ptr;
#if ETMVP || SUB_TMVP
    int ptr_col_ref = refp[REFP_0].list_ptr[REFP_0][refp[REFP_0].map_refi[scup_co][REFP_0]];
#else
    int ptr_col_ref = refp[REFP_0].list_ptr[refp[REFP_0].map_refi[scup_co][REFP_0]];
#endif
    scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
}

// text 9.5.8.4.3
void get_col_mv(COM_REFP refp[REFP_NUM], u32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D])
{
    s16 mvc[MV_D];
    assert(REFI_IS_VALID(refp[REFP_1].map_refi[scup_co][REFP_0]));

    mvc[MV_X] = refp[REFP_1].map_mv[scup_co][0][MV_X];
    mvc[MV_Y] = refp[REFP_1].map_mv[scup_co][0][MV_Y];
    int ptr_col = refp[REFP_1].ptr;
#if ETMVP || SUB_TMVP
    int ptr_col_ref = refp[REFP_1].list_ptr[REFP_0][refp[REFP_1].map_refi[scup_co][REFP_0]];
#else
    int ptr_col_ref = refp[REFP_1].list_ptr[refp[REFP_1].map_refi[scup_co][REFP_0]];
#endif
    scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
    scaling_mv1(ptr_curr, refp[REFP_1].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_1]);
}
#if SUB_TMVP
void get_col_mv_ext(COM_REFP refp[REFP_NUM], s32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D], s8 refi[REFP_NUM])
{
    s16 mvc[MV_D];
    
    int ptr_col;
    int ptr_col_ref;
    ptr_col = refp[REFP_1].ptr;
    refi[REFP_0] = REFI_INVALID;
    refi[REFP_1] = REFI_INVALID;
    assert(REFI_IS_VALID(refp[REFP_1].map_refi[scup_co][REFP_0]) || REFI_IS_VALID(refp[REFP_1].map_refi[scup_co][REFP_1]));
    if (REFI_IS_VALID(refp[REFP_1].map_refi[scup_co][REFP_0]))
    {
        mvc[MV_X] = refp[REFP_1].map_mv[scup_co][REFP_0][MV_X];
        mvc[MV_Y] = refp[REFP_1].map_mv[scup_co][REFP_0][MV_Y];
        
        ptr_col_ref = refp[REFP_1].list_ptr[REFP_0][refp[REFP_1].map_refi[scup_co][REFP_0]];
        scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
        scaling_mv1(ptr_curr, refp[REFP_1].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_1]);
        refi[REFP_0] = 0;
        refi[REFP_1] = 0;
    }
    else if (REFI_IS_VALID(refp[REFP_1].map_refi[scup_co][REFP_1]))
    {
        mvc[MV_X] = refp[REFP_1].map_mv[scup_co][REFP_1][MV_X];
        mvc[MV_Y] = refp[REFP_1].map_mv[scup_co][REFP_1][MV_Y];
        
        ptr_col_ref = refp[REFP_1].list_ptr[REFP_1][refp[REFP_1].map_refi[scup_co][REFP_1]];
        scaling_mv1(ptr_curr, refp[REFP_1].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_1]);
        scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
        refi[REFP_1] = 0;
        refi[REFP_0] = 0;
    }
}
void get_col_mv_from_list0_ext(COM_REFP refp[REFP_NUM], s32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D], s8 refi[REFP_NUM])
{
    s16 mvc[MV_D];
    
    int ptr_col;
    int ptr_col_ref;
    ptr_col = refp[REFP_0].ptr;
    assert(REFI_IS_VALID(refp[REFP_0].map_refi[scup_co][REFP_0]) || REFI_IS_VALID(refp[REFP_0].map_refi[scup_co][REFP_1]));
    
    if (REFI_IS_VALID(refp[REFP_0].map_refi[scup_co][REFP_0]))
    {
        mvc[MV_X] = refp[REFP_0].map_mv[scup_co][REFP_0][MV_X];
        mvc[MV_Y] = refp[REFP_0].map_mv[scup_co][REFP_0][MV_Y];
        
        ptr_col_ref = refp[REFP_0].list_ptr[REFP_0][refp[REFP_0].map_refi[scup_co][REFP_0]];
        scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
        refi[REFP_0] = 0;
    }
    else if (REFI_IS_VALID(refp[REFP_0].map_refi[scup_co][REFP_1]))
    {
        mvc[MV_X] = refp[REFP_0].map_mv[scup_co][REFP_1][MV_X];
        mvc[MV_Y] = refp[REFP_0].map_mv[scup_co][REFP_1][MV_Y];
        
        ptr_col_ref = refp[REFP_0].list_ptr[REFP_1][refp[REFP_0].map_refi[scup_co][REFP_1]];
        scaling_mv1(ptr_curr, refp[REFP_0].ptr, ptr_col, ptr_col_ref, mvc, mvp[REFP_0]);
        refi[REFP_0] = 0;
    }
}
#endif
u16 com_get_avail_intra(int x_scu, int y_scu, int pic_width_in_scu, int scup, u32 * map_scu)
{
    u16 avail = 0;
    if (x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[scup - 1]))//左侧可用？
    {
        SET_AVAIL(avail, AVAIL_LE);
    }
    if (y_scu > 0)
    {
        if (MCU_GET_CODED_FLAG(map_scu[scup - pic_width_in_scu]))//上方可用？
        {
            SET_AVAIL(avail, AVAIL_UP);
        }
        if (x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[scup - pic_width_in_scu - 1]))//上方的左边一个scu可用？
        {
            SET_AVAIL(avail, AVAIL_UP_LE);
        }
    }
    return avail;//返回的avail每一位（其实只有后三位有效）代表一个位置的scu是否可以
}

int com_picbuf_signature(COM_PIC *pic, u8 *signature)
{
    return com_md5_imgb(pic->imgb, signature);
}

/* MD5 functions */
#define MD5FUNC(f, w, x, y, z, msg1, s,msg2 ) \
     ( w += f(x, y, z) + msg1 + msg2,  w = w<<s | w>>(32-s),  w += x )
#define FF(x, y, z) (z ^ (x & (y ^ z)))
#define GG(x, y, z) (y ^ (z & (x ^ y)))
#define HH(x, y, z) (x ^ y ^ z)
#define II(x, y, z) (y ^ (x | ~z))

static void com_md5_trans(u32 *buf, u32 *msg)
{
    register u32 a, b, c, d;
    a = buf[0];
    b = buf[1];
    c = buf[2];
    d = buf[3];
    MD5FUNC(FF, a, b, c, d, msg[ 0],  7, 0xd76aa478); /* 1 */
    MD5FUNC(FF, d, a, b, c, msg[ 1], 12, 0xe8c7b756); /* 2 */
    MD5FUNC(FF, c, d, a, b, msg[ 2], 17, 0x242070db); /* 3 */
    MD5FUNC(FF, b, c, d, a, msg[ 3], 22, 0xc1bdceee); /* 4 */
    MD5FUNC(FF, a, b, c, d, msg[ 4],  7, 0xf57c0faf); /* 5 */
    MD5FUNC(FF, d, a, b, c, msg[ 5], 12, 0x4787c62a); /* 6 */
    MD5FUNC(FF, c, d, a, b, msg[ 6], 17, 0xa8304613); /* 7 */
    MD5FUNC(FF, b, c, d, a, msg[ 7], 22, 0xfd469501); /* 8 */
    MD5FUNC(FF, a, b, c, d, msg[ 8],  7, 0x698098d8); /* 9 */
    MD5FUNC(FF, d, a, b, c, msg[ 9], 12, 0x8b44f7af); /* 10 */
    MD5FUNC(FF, c, d, a, b, msg[10], 17, 0xffff5bb1); /* 11 */
    MD5FUNC(FF, b, c, d, a, msg[11], 22, 0x895cd7be); /* 12 */
    MD5FUNC(FF, a, b, c, d, msg[12],  7, 0x6b901122); /* 13 */
    MD5FUNC(FF, d, a, b, c, msg[13], 12, 0xfd987193); /* 14 */
    MD5FUNC(FF, c, d, a, b, msg[14], 17, 0xa679438e); /* 15 */
    MD5FUNC(FF, b, c, d, a, msg[15], 22, 0x49b40821); /* 16 */
    /* Round 2 */
    MD5FUNC(GG, a, b, c, d, msg[ 1],  5, 0xf61e2562); /* 17 */
    MD5FUNC(GG, d, a, b, c, msg[ 6],  9, 0xc040b340); /* 18 */
    MD5FUNC(GG, c, d, a, b, msg[11], 14, 0x265e5a51); /* 19 */
    MD5FUNC(GG, b, c, d, a, msg[ 0], 20, 0xe9b6c7aa); /* 20 */
    MD5FUNC(GG, a, b, c, d, msg[ 5],  5, 0xd62f105d); /* 21 */
    MD5FUNC(GG, d, a, b, c, msg[10],  9,  0x2441453); /* 22 */
    MD5FUNC(GG, c, d, a, b, msg[15], 14, 0xd8a1e681); /* 23 */
    MD5FUNC(GG, b, c, d, a, msg[ 4], 20, 0xe7d3fbc8); /* 24 */
    MD5FUNC(GG, a, b, c, d, msg[ 9],  5, 0x21e1cde6); /* 25 */
    MD5FUNC(GG, d, a, b, c, msg[14],  9, 0xc33707d6); /* 26 */
    MD5FUNC(GG, c, d, a, b, msg[ 3], 14, 0xf4d50d87); /* 27 */
    MD5FUNC(GG, b, c, d, a, msg[ 8], 20, 0x455a14ed); /* 28 */
    MD5FUNC(GG, a, b, c, d, msg[13],  5, 0xa9e3e905); /* 29 */
    MD5FUNC(GG, d, a, b, c, msg[ 2],  9, 0xfcefa3f8); /* 30 */
    MD5FUNC(GG, c, d, a, b, msg[ 7], 14, 0x676f02d9); /* 31 */
    MD5FUNC(GG, b, c, d, a, msg[12], 20, 0x8d2a4c8a); /* 32 */
    /* Round 3 */
    MD5FUNC(HH, a, b, c, d, msg[ 5],  4, 0xfffa3942); /* 33 */
    MD5FUNC(HH, d, a, b, c, msg[ 8], 11, 0x8771f681); /* 34 */
    MD5FUNC(HH, c, d, a, b, msg[11], 16, 0x6d9d6122); /* 35 */
    MD5FUNC(HH, b, c, d, a, msg[14], 23, 0xfde5380c); /* 36 */
    MD5FUNC(HH, a, b, c, d, msg[ 1],  4, 0xa4beea44); /* 37 */
    MD5FUNC(HH, d, a, b, c, msg[ 4], 11, 0x4bdecfa9); /* 38 */
    MD5FUNC(HH, c, d, a, b, msg[ 7], 16, 0xf6bb4b60); /* 39 */
    MD5FUNC(HH, b, c, d, a, msg[10], 23, 0xbebfbc70); /* 40 */
    MD5FUNC(HH, a, b, c, d, msg[13],  4, 0x289b7ec6); /* 41 */
    MD5FUNC(HH, d, a, b, c, msg[ 0], 11, 0xeaa127fa); /* 42 */
    MD5FUNC(HH, c, d, a, b, msg[ 3], 16, 0xd4ef3085); /* 43 */
    MD5FUNC(HH, b, c, d, a, msg[ 6], 23,  0x4881d05); /* 44 */
    MD5FUNC(HH, a, b, c, d, msg[ 9],  4, 0xd9d4d039); /* 45 */
    MD5FUNC(HH, d, a, b, c, msg[12], 11, 0xe6db99e5); /* 46 */
    MD5FUNC(HH, c, d, a, b, msg[15], 16, 0x1fa27cf8); /* 47 */
    MD5FUNC(HH, b, c, d, a, msg[ 2], 23, 0xc4ac5665); /* 48 */
    /* Round 4 */
    MD5FUNC(II, a, b, c, d, msg[ 0],  6, 0xf4292244); /* 49 */
    MD5FUNC(II, d, a, b, c, msg[ 7], 10, 0x432aff97); /* 50 */
    MD5FUNC(II, c, d, a, b, msg[14], 15, 0xab9423a7); /* 51 */
    MD5FUNC(II, b, c, d, a, msg[ 5], 21, 0xfc93a039); /* 52 */
    MD5FUNC(II, a, b, c, d, msg[12],  6, 0x655b59c3); /* 53 */
    MD5FUNC(II, d, a, b, c, msg[ 3], 10, 0x8f0ccc92); /* 54 */
    MD5FUNC(II, c, d, a, b, msg[10], 15, 0xffeff47d); /* 55 */
    MD5FUNC(II, b, c, d, a, msg[ 1], 21, 0x85845dd1); /* 56 */
    MD5FUNC(II, a, b, c, d, msg[ 8],  6, 0x6fa87e4f); /* 57 */
    MD5FUNC(II, d, a, b, c, msg[15], 10, 0xfe2ce6e0); /* 58 */
    MD5FUNC(II, c, d, a, b, msg[ 6], 15, 0xa3014314); /* 59 */
    MD5FUNC(II, b, c, d, a, msg[13], 21, 0x4e0811a1); /* 60 */
    MD5FUNC(II, a, b, c, d, msg[ 4],  6, 0xf7537e82); /* 61 */
    MD5FUNC(II, d, a, b, c, msg[11], 10, 0xbd3af235); /* 62 */
    MD5FUNC(II, c, d, a, b, msg[ 2], 15, 0x2ad7d2bb); /* 63 */
    MD5FUNC(II, b, c, d, a, msg[ 9], 21, 0xeb86d391); /* 64 */
    buf[0] += a;
    buf[1] += b;
    buf[2] += c;
    buf[3] += d;
}

void com_md5_init(COM_MD5 *md5)
{
    md5->h[0] = 0x67452301;
    md5->h[1] = 0xefcdab89;
    md5->h[2] = 0x98badcfe;
    md5->h[3] = 0x10325476;
    md5->bits[0] = 0;
    md5->bits[1] = 0;
}

void com_md5_update(COM_MD5 *md5, void *buf_t, u32 len)
{
    u8 *buf;
    u32 i, idx, part_len;
    buf = (u8*)buf_t;
    idx = (u32)((md5->bits[0] >> 3) & 0x3f);
    md5->bits[0] += (len << 3);
    if(md5->bits[0] < (len << 3))
    {
        (md5->bits[1])++;
    }
    md5->bits[1] += (len >> 29);
    part_len = 64 - idx;
    if(len >= part_len)
    {
        com_mcpy(md5->msg + idx, buf, part_len);
        com_md5_trans(md5->h, (u32 *)md5->msg);
        for(i = part_len; i + 63 < len; i += 64)
        {
            com_md5_trans(md5->h, (u32 *)(buf + i));
        }
        idx = 0;
    }
    else
    {
        i = 0;
    }
    if(len - i > 0)
    {
        com_mcpy(md5->msg + idx, buf + i, len - i);
    }
}

void com_md5_update_16(COM_MD5 *md5, void *buf_t, u32 len)
{
    u16 *buf;
    u32 i, idx, part_len, j;
    u8 t[512];
    buf = (u16 *)buf_t;
    idx = (u32)((md5->bits[0] >> 3) & 0x3f);
    len = len * 2;
    for(j = 0; j < len; j += 2)
    {
        t[j] = (u8)(*(buf));
        t[j + 1] = *(buf) >> 8;
        buf++;
    }
    md5->bits[0] += (len << 3);
    if(md5->bits[0] < (len << 3))
    {
        (md5->bits[1])++;
    }
    md5->bits[1] += (len >> 29);
    part_len = 64 - idx;
    if(len >= part_len)
    {
        com_mcpy(md5->msg + idx, t, part_len);
        com_md5_trans(md5->h, (u32 *)md5->msg);
        for(i = part_len; i + 63 < len; i += 64)
        {
            com_md5_trans(md5->h, (u32 *)(t + i));
        }
        idx = 0;
    }
    else
    {
        i = 0;
    }
    if(len - i > 0)
    {
        com_mcpy(md5->msg + idx, t + i, len - i);
    }
}

void com_md5_finish(COM_MD5 *md5, u8 digest[16])
{
    u8 *pos;
    int cnt;
    cnt = (md5->bits[0] >> 3) & 0x3F;
    pos = md5->msg + cnt;
    *pos++ = 0x80;
    cnt = 64 - 1 - cnt;
    if(cnt < 8)
    {
        com_mset(pos, 0, cnt);
        com_md5_trans(md5->h, (u32 *)md5->msg);
        com_mset(md5->msg, 0, 56);
    }
    else
    {
        com_mset(pos, 0, cnt - 8);
    }
    com_mcpy((md5->msg + 14 * sizeof(u32)), &md5->bits[0], sizeof(u32));
    com_mcpy((md5->msg + 15 * sizeof(u32)), &md5->bits[1], sizeof(u32));
    com_md5_trans(md5->h, (u32 *)md5->msg);
    com_mcpy(digest, md5->h, 16);
    com_mset(md5, 0, sizeof(COM_MD5));
}

int com_md5_imgb(COM_IMGB *imgb, u8 digest[16])
{
    COM_MD5 md5;
    int i, j;
    com_md5_init(&md5);

    for (i = 0; i < imgb->np; i++)
    {
        for (j = 0; j < imgb->height[i]; j++)
        {
            com_md5_update(&md5, ((u8 *)imgb->addr_plane[i]) + j * imgb->stride[i], imgb->width[i] * 2);
        }
    }

    com_md5_finish(&md5, digest);
    return COM_OK;
}

void init_scan(u16 *scan, int size_x, int size_y, int scan_type)
{
    int x, y, l, pos, num_line;
    pos = 0;
    num_line = size_x + size_y - 1;
    if(scan_type == COEF_SCAN_ZIGZAG)
    {
        /* starting point */
        scan[pos] = 0;
        pos++;
        /* loop */
        for(l = 1; l < num_line; l++)
        {
            if(l % 2) /* decreasing loop */
            {
                x = COM_MIN(l, size_x - 1);
                y = COM_MAX(0, l - (size_x - 1));
                while(x >= 0 && y < size_y)
                {
                    scan[pos] = (u16)(y * size_x + x);
                    pos++;
                    x--;
                    y++;
                }
            }
            else /* increasing loop */
            {
                y = COM_MIN(l, size_y - 1);
                x = COM_MAX(0, l - (size_y - 1));
                while(y >= 0 && x < size_x)
                {
                    scan[pos] = (u16)(y * size_x + x);
                    pos++;
                    x++;
                    y--;
                }
            }
        }
    }
}

int com_scan_tbl_init()
{
    int x, y, scan_type;
    int size_y, size_x;
    for(scan_type = 0; scan_type < COEF_SCAN_TYPE_NUM; scan_type++)
    {
        for (y = 0; y < MAX_CU_LOG2; y++)
        {
            size_y = 1 << (y + 1);
            for (x = 0; x < MAX_CU_LOG2; x++)
            {
                size_x = 1 << (x + 1);
                com_scan_tbl[scan_type][x][y] = (u16*)com_malloc_fast(size_y * size_x * sizeof(u16));
                init_scan(com_scan_tbl[scan_type][x][y], size_x, size_y, scan_type);
            }
        }
    }
    return COM_OK;
}

int com_scan_tbl_delete()
{
    int x, y, scan_type;
    for(scan_type = 0; scan_type < COEF_SCAN_TYPE_NUM; scan_type++)
    {
        for(y = 0; y < MAX_CU_LOG2 - 1; y++)
        {
            for(x = 0; x < MAX_CU_LOG2 - 1; x++)
            {
                if(com_scan_tbl[scan_type][x][y] != NULL)
                {
                    free(com_scan_tbl[scan_type][x][y]);
                }
            }
        }
    }
    return COM_OK;
}

#if MODE_CONS
u8 com_get_cons_pred_mode(int cud, int cup, int cu_width, int cu_height, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int pos = cup + (((cu_height >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cu_width >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cu_width) - CONV_LOG2(cu_height));
    int cons_pred_mode = NO_MODE_CONS;
    assert( shape >= 0 && shape < NUM_BLOCK_SHAPE );
    if (cu_width < 8 && cu_height < 8)
    {
        assert(0);
    }
    cons_pred_mode = (split_mode_buf[cud][shape][pos] >> 3) & 0x03;
    assert(cons_pred_mode == ONLY_INTRA || cons_pred_mode == ONLY_INTER);
    return cons_pred_mode;
}

void com_set_cons_pred_mode(u8 cons_pred_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int pos = cup + (((cu_height >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cu_width >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cu_width) - CONV_LOG2(cu_height));
    assert( shape >= 0 && shape < NUM_BLOCK_SHAPE );
    if (cu_width < 8 && cu_height < 8)
    {
        assert(0);
    }
    split_mode_buf[cud][shape][pos] = (split_mode_buf[cud][shape][pos] & 0x67) + (cons_pred_mode << 3); //01100111 = 0x67
}
#endif

int com_get_split_mode(s8 *split_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s

                       , s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]

                      )
{
    int ret = COM_OK;
    int pos = cup + (((cu_height >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cu_width >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cu_width) - CONV_LOG2(cu_height));
    assert( shape >= 0 && shape < NUM_BLOCK_SHAPE );
    if(cu_width < 8 && cu_height < 8)
    {
        *split_mode = NO_SPLIT;
        return ret;
    }
    *split_mode = split_mode_buf[cud][shape][pos] & 0x07;
    return ret;
}

int com_set_split_mode(s8 split_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s

                       , s8 (*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]

                      )
{
    int ret = COM_OK;
    int pos = cup + (((cu_height >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cu_width >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cu_width) - CONV_LOG2(cu_height));
    assert( shape >= 0 && shape < NUM_BLOCK_SHAPE );
    if(cu_width < 8 && cu_height < 8)
    {
        return ret;
    }
    split_mode_buf[cud][shape][pos] = split_mode;
    return ret;
}

void com_check_split_mode(COM_SQH* sqh, int *split_allow, int cu_width_log2, int cu_height_log2, int boundary, int boundary_b, int boundary_r, int log2_max_cuwh, int id
                          , const int parent_split, int qt_depth, int bet_depth, int slice_type)
{
    //constraints parameters
    const int min_cu_size      = sqh->min_cu_size;
    const int min_bt_size      = sqh->min_cu_size;
    const int min_eqt_size     = sqh->min_cu_size;
    const int max_split_depth  = sqh->max_split_times;
    const int max_aspect_ratio = sqh->max_part_ratio;
    const int min_qt_size      = sqh->min_qt_size;
    const int max_bt_size      = sqh->max_bt_size;
    const int max_eqt_size     = sqh->max_eqt_size;
    int max_aspect_ratio_eqt = max_aspect_ratio >> 1;
    int cu_w = 1 << cu_width_log2;
    int cu_h = 1 << cu_height_log2;
    int i;

    for (i = NO_SPLIT; i <= SPLIT_QUAD; i++)
        split_allow[i] = 0;

    if (boundary)
    {
        // VPDU previous than boundary
        if ((cu_w == 64 && cu_h == 128) || (cu_h == 64 && cu_w == 128))
        {
            split_allow[SPLIT_BI_HOR] = 1;
            split_allow[SPLIT_BI_VER] = 1;
        }
        // large block previous than boundary
        else if (slice_type == SLICE_I && cu_w == 128 && cu_h == 128)
        {
            split_allow[SPLIT_QUAD] = 1;
            split_allow[NO_SPLIT] = 1;
        }
        else if (!boundary_r && !boundary_b)
        {
            split_allow[SPLIT_QUAD] = 1;
        }
        else if (boundary_r)
        {
            split_allow[SPLIT_BI_VER] = 1;
        }
        else if (boundary_b)
        {
            split_allow[SPLIT_BI_HOR] = 1;
        }
        assert(qt_depth + bet_depth < max_split_depth);
    }
    else
    {
        // VPDU
        if ((cu_w == 64 && cu_h == 128) || (cu_h == 64 && cu_w == 128))
        {
            split_allow[SPLIT_BI_HOR] = 1;
            split_allow[SPLIT_BI_VER] = 1;
            split_allow[NO_SPLIT] = 1;
        }
        //max qt-bt depth constraint
        else if (qt_depth + bet_depth >= max_split_depth)
        {
            split_allow[NO_SPLIT]   = 1; //no further split allowed
        }
        else if (slice_type == SLICE_I && cu_w == 128 && cu_h == 128)
        {
            split_allow[SPLIT_QUAD] = 1;
            split_allow[NO_SPLIT] = 1;
        }
        else
        {
            //not split
            if (cu_w <= cu_h * max_aspect_ratio && cu_h <= cu_w * max_aspect_ratio)
                split_allow[NO_SPLIT] = 1;

            //qt
            if (cu_w > min_qt_size && bet_depth == 0)
                split_allow[SPLIT_QUAD] = 1;

            //hbt
            if ((cu_w <= max_bt_size && cu_h <= max_bt_size) && cu_h > min_bt_size && cu_w < cu_h * max_aspect_ratio)
                split_allow[SPLIT_BI_HOR] = 1;

            //vbt
            if ((cu_w <= max_bt_size && cu_h <= max_bt_size) && cu_w > min_bt_size && cu_h < cu_w * max_aspect_ratio)
                split_allow[SPLIT_BI_VER] = 1;

            //heqt
            if ((cu_w <= max_eqt_size && cu_h <= max_eqt_size) && cu_h > min_eqt_size * 2 && cu_w > min_eqt_size && cu_w < cu_h * max_aspect_ratio_eqt)
                split_allow[SPLIT_EQT_HOR] = 1;

            //veqt
            if ((cu_w <= max_eqt_size && cu_h <= max_eqt_size) && cu_w > min_eqt_size * 2 && cu_h > min_eqt_size && cu_h < cu_w * max_aspect_ratio_eqt)
                split_allow[SPLIT_EQT_VER] = 1;
        }
    }

#if SPLIT_DEBUG
    int num_allowed = 0;
    for (i = NO_SPLIT; i <= SPLIT_QUAD; i++)
        num_allowed += split_allow[i] == 1;
    assert(num_allowed);
#endif

    COM_TRACE_STR("allow split mode: ");
    COM_TRACE_INT(split_allow[NO_SPLIT]);
    COM_TRACE_INT(split_allow[SPLIT_BI_VER]);
    COM_TRACE_INT(split_allow[SPLIT_BI_HOR]);
    COM_TRACE_INT(split_allow[SPLIT_EQT_VER]);
    COM_TRACE_INT(split_allow[SPLIT_EQT_HOR]);
    COM_TRACE_INT(split_allow[SPLIT_QUAD]);
    COM_TRACE_STR("\n");
}

void com_mv_rounding_s32(s32 hor, int ver, s32 * rounded_hor, s32 * rounded_ver, s32 right_shift, int left_shift)
{
    int add = (right_shift > 0) ? (1 << (right_shift - 1)) : 0;
    *rounded_hor = (hor >= 0) ? (((hor + add) >> right_shift) << left_shift) : -(((-hor + add) >> right_shift) << left_shift);
    *rounded_ver = (ver >= 0) ? (((ver + add) >> right_shift) << left_shift) : -(((-ver + add) >> right_shift) << left_shift);
}

void com_mv_rounding_s16(s32 hor, s32 ver, s16 * rounded_hor, s16 * rounded_ver, int right_shift, int left_shift)
{
    int add = (right_shift > 0) ? (1 << (right_shift - 1)) : 0;
    *rounded_hor = (hor >= 0) ? (((hor + add) >> right_shift) << left_shift) : -(((-hor + add) >> right_shift) << left_shift);
    *rounded_ver = (ver >= 0) ? (((ver + add) >> right_shift) << left_shift) : -(((-ver + add) >> right_shift) << left_shift);
}


/*******************************************/
/* Neighbor location: Graphical indication */
/*                                         */
/*           D B_____________G C           */
/*           A|               |            */
/*            |               |            */
/*            |               |            */
/*            |      cu    cuh|            */
/*            |               |            */
/*            |               |            */
/*           F|_____cuw______H|            */
/*                                         */
/*                                         */
/*******************************************/

void com_set_affine_mvf(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map)
{
    int scup = mod_info_curr->scup;
    int cu_w = mod_info_curr->cu_width;
    int cu_h = mod_info_curr->cu_height;
    int cu_w_in_scu = PEL2SCU(cu_w);
    int cu_h_in_scu = PEL2SCU(cu_h);
    int log2_cuw = mod_info_curr->cu_width_log2;
    int log2_cuh = mod_info_curr->cu_height_log2;

    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;

    
    int   w_sch, h_sch, x, y;
    int   lidx;
    int   cp_num = mod_info_curr->affine_flag + 1;
    int   aff_scup[VER_NUM];
    int   sub_w = 4;
    int   sub_h = 4;


    if (sh->affine_subblock_size_idx == 1)
    {
        sub_w = 8;
        sub_h = 8;
    }
    if (REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && REFI_IS_VALID(mod_info_curr->refi[REFP_1]))
    {
        sub_w = 8;
        sub_h = 8;
    }
    int   sub_w_in_scu = PEL2SCU(sub_w);
    int   sub_h_in_scu = PEL2SCU(sub_h);
    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;
    int   addr_in_scu;



    addr_in_scu = scup;
    for ( h_sch = 0; h_sch < cu_h_in_scu; h_sch++ )
    {
        for ( w_sch = 0; w_sch < cu_w_in_scu; w_sch++ )
        {
            pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->refi[REFP_0];
            pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->refi[REFP_1];
            pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = 0;
            pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = 0;
            pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = 0;
            pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = 0;
        }
        addr_in_scu += pic_width_in_scu;
    }

    aff_scup[0] = 0;
    aff_scup[1] = (cu_w_in_scu - 1);
    aff_scup[2] = (cu_h_in_scu - 1) * pic_width_in_scu;
    aff_scup[3] = (cu_w_in_scu - 1) + (cu_h_in_scu - 1) * pic_width_in_scu;
    for ( lidx = 0; lidx < REFP_NUM; lidx++ )
    {
        if ( mod_info_curr->refi[lidx] >= 0 )
        {
            CPMV (*ac_mv)[MV_D] = mod_info_curr->affine_mv[lidx];
            s32 dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
            s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
            s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
            s32 mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> log2_cuw;     // deltaMvHor
            dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> log2_cuw;
            if (cp_num == 3)
            {
                dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> log2_cuh; // deltaMvVer
                dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> log2_cuh;
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                                     // deltaMvVer
                dmv_ver_y = dmv_hor_x;
            }

            for (h_sch = 0; h_sch < cu_h_in_scu; h_sch += sub_h_in_scu)
            {
                for (w_sch = 0; w_sch < cu_w_in_scu; w_sch += sub_w_in_scu)
                {
                    int pos_x = (w_sch << MIN_CU_LOG2) + half_w;
                    int pos_y = (h_sch << MIN_CU_LOG2) + half_h;
                    if (w_sch == 0 && h_sch == 0)
                    {
                        pos_x = 0;
                        pos_y = 0;
                    }
                    else if (w_sch + sub_w_in_scu == cu_w_in_scu && h_sch == 0)
                    {
                        pos_x = cu_w_in_scu << MIN_CU_LOG2;
                        pos_y = 0;
                    }
                    else if (w_sch == 0 && h_sch + sub_h_in_scu == cu_h_in_scu && cp_num == 3)
                    {
                        pos_x = 0;
                        pos_y = cu_h_in_scu << MIN_CU_LOG2;
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

                    // 1/4 precision, 16 bits, for mv storage
                    com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 2, 0);
                    mv_scale_tmp_hor = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_scale_tmp_ver);

                    // save MV for each 4x4 block
                    for (y = h_sch; y < h_sch +sub_h_in_scu; y++)
                    {
                        for (x = w_sch; x < w_sch + sub_w_in_scu; x++)
                        {
                            addr_in_scu = scup + x + y * pic_width_in_scu;
                            pic_map->map_mv[addr_in_scu][lidx][MV_X] = (s16)mv_scale_tmp_hor;
                            pic_map->map_mv[addr_in_scu][lidx][MV_Y] = (s16)mv_scale_tmp_ver;
                        }
                    }
                }
            }
        }
    }
}

#if AWP
void com_set_awp_mvf(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map)
{
    int scup = mod_info_curr->scup;
    int cu_w = mod_info_curr->cu_width;
    int cu_h = mod_info_curr->cu_height;
    int cu_w_in_scu = PEL2SCU(cu_w);
    int cu_h_in_scu = PEL2SCU(cu_h);
    int pic_width_in_scu = info->pic_width_in_scu;
    int   w_sch, h_sch/*, x, y*/;
    int   addr_in_scu = scup;
    int stepIdx, angleIdx, subangleIdx;

    com_calculate_awp_para(mod_info_curr->skip_idx, cu_w, cu_h, &stepIdx, &angleIdx, &subangleIdx);

    int FirstPos = 0;
    int DeltaPos_W = 0;
    int DeltaPos_H = 0;

    //Set half pixel length
    int ValidLength_W = (cu_w + (cu_h >> angleIdx)) << 1;
    int ValidLength_H = (cu_h + (cu_w >> angleIdx)) << 1;
    int temp_W = ((cu_h << 1) >> angleIdx);
    int temp_H = ((cu_w << 1) >> angleIdx);
    DeltaPos_W = (ValidLength_W >> 3) - 1;
    DeltaPos_H = (ValidLength_H >> 3) - 1;
    DeltaPos_W = DeltaPos_W == 0 ? 1 : DeltaPos_W;
    DeltaPos_H = DeltaPos_H == 0 ? 1 : DeltaPos_H;
    DeltaPos_W = stepIdx * DeltaPos_W;
    DeltaPos_H = stepIdx * DeltaPos_H;

    switch (subangleIdx)
    {
    case 0:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (info->pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) + 1 + DeltaPos_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) - 2 + DeltaPos_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) - 2 + DeltaPos_H;
#endif
        for (h_sch = 0; h_sch < cu_h_in_scu; h_sch++)
        {
            for (w_sch = 0; w_sch < cu_w_in_scu; w_sch++)
            {
                int pos_x = (w_sch << MIN_CU_LOG2) + 2;
                int pos_y = (h_sch << MIN_CU_LOG2) + 2;
                if (((pos_y << 1) + ((pos_x << 1) >> angleIdx)) >= FirstPos)
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi0[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi0[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi1[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi1[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_scu += pic_width_in_scu;
        }
        break;
    case 1:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (info->pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) + 3 + DeltaPos_H - temp_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) + DeltaPos_H - temp_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) + DeltaPos_H - temp_H;
#endif
        for (h_sch = 0; h_sch < cu_h_in_scu; h_sch++)
        {
            for (w_sch = 0; w_sch < cu_w_in_scu; w_sch++)
            {
                int pos_x = (w_sch << MIN_CU_LOG2) + 2;
                int pos_y = (h_sch << MIN_CU_LOG2) + 2;
                if (((pos_y << 1) - ((pos_x << 1) >> angleIdx)) >= FirstPos)
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi0[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi0[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi1[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi1[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_scu += pic_width_in_scu;
        }
        break;
    case 2:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (info->pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) + 3 + DeltaPos_W - temp_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) + DeltaPos_W - temp_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) + DeltaPos_W - temp_W;
#endif
        for (h_sch = 0; h_sch < cu_h_in_scu; h_sch++)
        {
            for (w_sch = 0; w_sch < cu_w_in_scu; w_sch++)
            {
                int pos_x = (w_sch << MIN_CU_LOG2) + 2;
                int pos_y = (h_sch << MIN_CU_LOG2) + 2;
                if (((pos_x << 1) - ((pos_y << 1) >> angleIdx)) >= FirstPos)
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi0[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi0[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi1[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi1[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_scu += pic_width_in_scu;
        }
        break;
    case 3:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (info->pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) + 1 + DeltaPos_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) - 2 + DeltaPos_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) - 2 + DeltaPos_W;
#endif
        for (h_sch = 0; h_sch < cu_h_in_scu; h_sch++)
        {
            for (w_sch = 0; w_sch < cu_w_in_scu; w_sch++)
            {
                int pos_x = (w_sch << MIN_CU_LOG2) + 2;
                int pos_y = (h_sch << MIN_CU_LOG2) + 2;
                if (((pos_x << 1) + ((pos_y << 1) >> angleIdx)) >= FirstPos)
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi0[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi0[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_0] = mod_info_curr->awp_refi1[REFP_0];
                    pic_map->map_refi[addr_in_scu + w_sch][REFP_1] = mod_info_curr->awp_refi1[REFP_1];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
                    pic_map->map_mv[addr_in_scu + w_sch][REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_scu += pic_width_in_scu;
        }
        break;
    default:
        printf("\nError: awp parameter not expected\n");
        assert(0);
    }
}
#endif

int com_get_affine_memory_access(CPMV mv[VER_NUM][MV_D], int cu_width, int cu_height)
{
#if BD_AFFINE_AMVR
    s16 mv_tmp[VER_NUM][MV_D];
    for (int i = 0; i < VER_NUM; i++)
    {
        mv_tmp[i][MV_X] = mv[i][MV_X] >> 2;
        mv_tmp[i][MV_Y] = mv[i][MV_Y] >> 2;
    }
    int max_x = max(mv_tmp[0][MV_X], max(mv_tmp[1][MV_X] + cu_width, max(mv_tmp[2][MV_X], mv_tmp[3][MV_X] + cu_width))) >> 2;
    int min_x = min(mv_tmp[0][MV_X], min(mv_tmp[1][MV_X] + cu_width, min(mv_tmp[2][MV_X], mv_tmp[3][MV_X] + cu_width))) >> 2;
    int max_y = max(mv_tmp[0][MV_Y], max(mv_tmp[1][MV_Y], max(mv_tmp[2][MV_Y] + cu_height, mv_tmp[3][MV_Y] + cu_height))) >> 2;
    int min_y = min(mv_tmp[0][MV_Y], min(mv_tmp[1][MV_Y], min(mv_tmp[2][MV_Y] + cu_height, mv_tmp[3][MV_Y] + cu_height))) >> 2;
#else
    int max_x = max(mv[0][MV_X], max(mv[1][MV_X] + cu_width, max(mv[2][MV_X], mv[3][MV_X] + cu_width))) >> 2;
    int min_x = min(mv[0][MV_X], min(mv[1][MV_X] + cu_width, min(mv[2][MV_X], mv[3][MV_X] + cu_width))) >> 2;
    int max_y = max(mv[0][MV_Y], max(mv[1][MV_Y], max(mv[2][MV_Y] + cu_height, mv[3][MV_Y] + cu_height))) >> 2;
    int min_y = min(mv[0][MV_Y], min(mv[1][MV_Y], min(mv[2][MV_Y] + cu_height, mv[3][MV_Y] + cu_height))) >> 2;
#endif
    return (abs(max_x - min_x) + 7) *  (abs(max_y - min_y) + 7);
}

void com_derive_affine_model_mv( int scup, int scun, int lidx, s16 ( *map_mv )[REFP_NUM][MV_D], int cu_width, int cu_height, int pic_width_in_scu, int pic_height_in_scu, CPMV mvp[VER_NUM][MV_D], u32 *map_cu_mode, int * vertex_num, int log2_max_cuwh )
{
    s16 neb_mv[VER_NUM][MV_D] = {{0, }, };
    int i;
    int neb_addr[VER_NUM];
    int neb_log_w = MCU_GET_LOGW(map_cu_mode[scun]);
    int neb_log_h = MCU_GET_LOGH(map_cu_mode[scun]);
    int neb_w = 1 << neb_log_w;
    int neb_h = 1 << neb_log_h;
    int neb_x, neb_y;
    int cur_x, cur_y;
    int diff_w = 7 - neb_log_w;
    int diff_h = 7 - neb_log_h;
    s32 dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, hor_base, ver_base;
    s32 tmp_hor, tmp_ver;
    neb_addr[0] = scun - MCU_GET_X_SCU_OFF(map_cu_mode[scun]) - pic_width_in_scu * MCU_GET_Y_SCU_OFF(map_cu_mode[scun]);
    neb_addr[1] = neb_addr[0] + ((neb_w >> MIN_CU_LOG2) - 1);
    neb_addr[2] = neb_addr[0] + ((neb_h >> MIN_CU_LOG2) - 1) * pic_width_in_scu;
    neb_addr[3] = neb_addr[2] + ((neb_w >> MIN_CU_LOG2) - 1);

    neb_x = (neb_addr[0] % pic_width_in_scu) << MIN_CU_LOG2;
    neb_y = (neb_addr[0] / pic_width_in_scu) << MIN_CU_LOG2;
    cur_x = (scup % pic_width_in_scu) << MIN_CU_LOG2;
    cur_y = (scup / pic_width_in_scu) << MIN_CU_LOG2;

    for (i = 0; i < VER_NUM; i++)
    {
        neb_mv[i][MV_X] = map_mv[neb_addr[i]][lidx][MV_X];
        neb_mv[i][MV_Y] = map_mv[neb_addr[i]][lidx][MV_Y];
    }

    int is_top_ctu_boundary = FALSE;
    if ((neb_y + neb_h) % (1 << log2_max_cuwh) == 0 && (neb_y + neb_h) == cur_y)
    {
        is_top_ctu_boundary = TRUE;
        neb_y += neb_h;

        neb_mv[0][MV_X] = neb_mv[2][MV_X];
        neb_mv[0][MV_Y] = neb_mv[2][MV_Y];
        neb_mv[1][MV_X] = neb_mv[3][MV_X];
        neb_mv[1][MV_Y] = neb_mv[3][MV_Y];
    }

    dmv_hor_x = (s32)(neb_mv[1][MV_X] - neb_mv[0][MV_X]) << diff_w;      // deltaMvHor
    dmv_hor_y = (s32)(neb_mv[1][MV_Y] - neb_mv[0][MV_Y]) << diff_w;
    if (*vertex_num == 3 && !is_top_ctu_boundary)
    {
        dmv_ver_x = (s32)(neb_mv[2][MV_X] - neb_mv[0][MV_X]) << diff_h;  // deltaMvVer
        dmv_ver_y = (s32)(neb_mv[2][MV_Y] - neb_mv[0][MV_Y]) << diff_h;
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                          // deltaMvVer
        dmv_ver_y = dmv_hor_x;
        *vertex_num = 2;
    }
    hor_base = (s32)neb_mv[0][MV_X] << 7;
    ver_base = (s32)neb_mv[0][MV_Y] << 7;

    // derive CPMV 0
    tmp_hor = dmv_hor_x * (cur_x - neb_x) + dmv_ver_x * (cur_y - neb_y) + hor_base;
    tmp_ver = dmv_hor_y * (cur_x - neb_x) + dmv_ver_y * (cur_y - neb_y) + ver_base;
    com_mv_rounding_s32(tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, 7, 0);
#if BD_AFFINE_AMVR
    tmp_hor <<= 2;
    tmp_ver <<= 2;
#endif
    mvp[0][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_hor);
    mvp[0][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_ver);

    // derive CPMV 1
    tmp_hor = dmv_hor_x * (cur_x - neb_x + cu_width) + dmv_ver_x * (cur_y - neb_y) + hor_base;
    tmp_ver = dmv_hor_y * (cur_x - neb_x + cu_width) + dmv_ver_y * (cur_y - neb_y) + ver_base;
    com_mv_rounding_s32(tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, 7, 0);
#if BD_AFFINE_AMVR
    tmp_hor <<= 2;
    tmp_ver <<= 2;
#endif
    mvp[1][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_hor);
    mvp[1][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_ver);

    // derive CPMV 2
    if (*vertex_num == 3)
    {
        tmp_hor = dmv_hor_x * (cur_x - neb_x) + dmv_ver_x * (cur_y - neb_y + cu_height) + hor_base;
        tmp_ver = dmv_hor_y * (cur_x - neb_x) + dmv_ver_y * (cur_y - neb_y + cu_height) + ver_base;
        com_mv_rounding_s32(tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, 7, 0);
#if BD_AFFINE_AMVR
        tmp_hor <<= 2;
        tmp_ver <<= 2;
#endif
        mvp[2][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_hor);
        mvp[2][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, tmp_ver);
    }
}

int com_derive_affine_constructed_candidate( int cu_width, int cu_height, int cp_valid[VER_NUM], s16 cp_mv[REFP_NUM][VER_NUM][MV_D], int cp_refi[REFP_NUM][VER_NUM], int cp_idx[VER_NUM], int model_idx, int ver_num, CPMV mrg_list_cp_mv[AFF_MAX_NUM_MRG][REFP_NUM][VER_NUM][MV_D], s8 mrg_list_refi[AFF_MAX_NUM_MRG][REFP_NUM], int mrg_list_cp_num[AFF_MAX_NUM_MRG], int *mrg_idx )
{
    assert(com_tbl_log2[cu_width] >= 4);
    assert(com_tbl_log2[cu_height] >= 4);

    int lidx, i;
    int valid_model[2] = { 0, 0 };
    s32 cpmv_tmp[REFP_NUM][VER_NUM][MV_D];
    int tmp_hor, tmp_ver;
    int shiftHtoW = 7 + com_tbl_log2[cu_width] - com_tbl_log2[cu_height]; // x * cuWidth / cuHeight

    // early terminate
    if ( *mrg_idx >= AFF_MAX_NUM_MRG )
    {
        return 0;
    }

    // check valid model and decide ref idx
    if ( ver_num == 2 )
    {
        int idx0 = cp_idx[0], idx1 = cp_idx[1];
        if ( !cp_valid[idx0] || !cp_valid[idx1] )
        {
            return 0;
        }
        for ( lidx = 0; lidx < REFP_NUM; lidx++ )
        {
            if ( REFI_IS_VALID( cp_refi[lidx][idx0] ) && REFI_IS_VALID( cp_refi[lidx][idx1] ) && cp_refi[lidx][idx0] == cp_refi[lidx][idx1] )
            {
                valid_model[lidx] = 1;
            }
        }
    }
    else if ( ver_num == 3 )
    {
        int idx0 = cp_idx[0], idx1 = cp_idx[1], idx2 = cp_idx[2];
        if ( !cp_valid[idx0] || !cp_valid[idx1] || !cp_valid[idx2] )
        {
            return 0;
        }

        for ( lidx = 0; lidx < REFP_NUM; lidx++ )
        {
            if ( REFI_IS_VALID( cp_refi[lidx][idx0] ) && REFI_IS_VALID( cp_refi[lidx][idx1] ) && REFI_IS_VALID( cp_refi[lidx][idx2] ) && cp_refi[lidx][idx0] == cp_refi[lidx][idx1] && cp_refi[lidx][idx0] == cp_refi[lidx][idx2] )
            {
                valid_model[lidx] = 1;
            }
        }
    }

    // set merge index and vertex num for valid model
    if ( valid_model[0] || valid_model[1] )
    {
        mrg_list_cp_num[*mrg_idx] = ver_num;
    }
    else
    {
        return 0;
    }

    for ( lidx = 0; lidx < REFP_NUM; lidx++ )
    {
        if ( valid_model[lidx] )
        {
            for (i = 0; i < ver_num; i++)
            {
                cpmv_tmp[lidx][cp_idx[i]][MV_X] = (s32)cp_mv[lidx][cp_idx[i]][MV_X];
                cpmv_tmp[lidx][cp_idx[i]][MV_Y] = (s32)cp_mv[lidx][cp_idx[i]][MV_Y];
            }

            // convert to LT, RT[, [LB], [RB]]
            switch ( model_idx )
            {
            case 0: // 0 : LT, RT, LB
                break;
            case 1: // 1 : LT, RT, RB
                cpmv_tmp[lidx][2][MV_X] = cpmv_tmp[lidx][3][MV_X] + cpmv_tmp[lidx][0][MV_X] - cpmv_tmp[lidx][1][MV_X];
                cpmv_tmp[lidx][2][MV_Y] = cpmv_tmp[lidx][3][MV_Y] + cpmv_tmp[lidx][0][MV_Y] - cpmv_tmp[lidx][1][MV_Y];
                break;
            case 2: // 2 : LT, LB, RB
                cpmv_tmp[lidx][1][MV_X] = cpmv_tmp[lidx][3][MV_X] + cpmv_tmp[lidx][0][MV_X] - cpmv_tmp[lidx][2][MV_X];
                cpmv_tmp[lidx][1][MV_Y] = cpmv_tmp[lidx][3][MV_Y] + cpmv_tmp[lidx][0][MV_Y] - cpmv_tmp[lidx][2][MV_Y];
                break;
            case 3: // 3 : RT, LB, RB
                cpmv_tmp[lidx][0][MV_X] = cpmv_tmp[lidx][1][MV_X] + cpmv_tmp[lidx][2][MV_X] - cpmv_tmp[lidx][3][MV_X];
                cpmv_tmp[lidx][0][MV_Y] = cpmv_tmp[lidx][1][MV_Y] + cpmv_tmp[lidx][2][MV_Y] - cpmv_tmp[lidx][3][MV_Y];
                break;
            case 4: // 4 : LT, RT
                break;
            case 5: // 5 : LT, LB
                tmp_hor = +((cpmv_tmp[lidx][2][MV_Y] - cpmv_tmp[lidx][0][MV_Y]) << shiftHtoW) + (cpmv_tmp[lidx][0][MV_X] << 7);
                tmp_ver = -((cpmv_tmp[lidx][2][MV_X] - cpmv_tmp[lidx][0][MV_X]) << shiftHtoW) + (cpmv_tmp[lidx][0][MV_Y] << 7);
                com_mv_rounding_s32(tmp_hor, tmp_ver, &cpmv_tmp[lidx][1][MV_X], &cpmv_tmp[lidx][1][MV_Y], 7, 0);
                break;
            default:
                com_assert( 0 );
            }

            mrg_list_refi[*mrg_idx][lidx] = cp_refi[lidx][cp_idx[0]];
            for (i = 0; i < ver_num; i++)
            {
#if BD_AFFINE_AMVR // convert to 1/16 precision
                cpmv_tmp[lidx][i][MV_X] <<= 2;
                cpmv_tmp[lidx][i][MV_Y] <<= 2;
#endif
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, cpmv_tmp[lidx][i][MV_X]);
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, cpmv_tmp[lidx][i][MV_Y]);
            }
        }
        else
        {
            mrg_list_refi[*mrg_idx][lidx] = -1;
            for ( i = 0; i < ver_num; i++ )
            {
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_X] = 0;
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_Y] = 0;
            }
        }
    }

    (*mrg_idx)++;

    return 1;
}

/* merge affine mode */
int com_get_affine_merge_candidate(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map,
    s8 mrg_list_refi[AFF_MAX_NUM_MRG][REFP_NUM], CPMV mrg_list_cpmv[AFF_MAX_NUM_MRG][REFP_NUM][VER_NUM][MV_D], int mrg_list_cp_num[AFF_MAX_NUM_MRG], int ptr)
{
    int scup = mod_info_curr->scup;
    int x_scu = mod_info_curr->x_scu;
    int y_scu = mod_info_curr->y_scu;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int cu_width_in_scu = PEL2SCU(cu_width);
    int cu_height_in_scu = PEL2SCU(cu_height);

    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    int pic_height_in_scu = info->pic_height_in_scu;
    int log2_max_cuwh = info->log2_max_cuwh;
    int slice_type = info->pic_header.slice_type;

    s8(*map_refi)[REFP_NUM] = pic_map->map_refi;
    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    u32* map_scu = pic_map->map_scu;
    u32* map_cu_mode = pic_map->map_cu_mode;

    int lidx, i, k;
    int cnt = 0;

    //-------------------  Model based affine MVP  -------------------//
    {
        int neb_addr[5];
        int valid_flag[5];
        int top_left[5];

        neb_addr[0] = scup + pic_width_in_scu * (cu_height_in_scu - 1) - 1; // F
        neb_addr[1] = scup - pic_width_in_scu + cu_width_in_scu - 1;        // G
        neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu;            // C
        neb_addr[3] = scup - 1;                                             // A
        neb_addr[4] = scup - pic_width_in_scu - 1;                          // D

        valid_flag[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]) && MCU_GET_AFF(map_scu[neb_addr[2]]);
        valid_flag[3] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[3]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[3]]) && MCU_GET_AFF(map_scu[neb_addr[3]]);
        valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[4]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr[4]]) && MCU_GET_AFF(map_scu[neb_addr[4]]);

        for (k = 0; k < 5; k++)
        {
            if (valid_flag[k])
            {
                top_left[k] = neb_addr[k] - MCU_GET_X_SCU_OFF(map_cu_mode[neb_addr[k]]) - pic_width_in_scu * MCU_GET_Y_SCU_OFF(map_cu_mode[neb_addr[k]]);
            }
        }

        if (valid_flag[2] && valid_flag[1] && top_left[1] == top_left[2]) // exclude same CU cases
        {
            valid_flag[2] = 0;
        }

        int valid_flag_a = valid_flag[3];
        if (valid_flag[3] && valid_flag[0] && top_left[0] == top_left[3])
        {
            valid_flag[3] = 0;
        }
        if ((valid_flag[4] && valid_flag_a && top_left[4] == top_left[3]) || (valid_flag[4] && valid_flag[1] && top_left[4] == top_left[1]))
        {
            valid_flag[4] = 0;
        }

        for (k = 0; k < 5; k++)
        {
            if (valid_flag[k])
            {
                // set vertex number: affine flag == 1, set to 2 vertex, otherwise, set to 3 vertex
                mrg_list_cp_num[cnt] = (MCU_GET_AFF(map_scu[neb_addr[k]]) == 1) ? 2 : 3;

                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    if (REFI_IS_VALID(map_refi[neb_addr[k]][lidx]))
                    {
                        mrg_list_refi[cnt][lidx] = map_refi[neb_addr[k]][lidx];
                        com_derive_affine_model_mv(scup, neb_addr[k], lidx, map_mv, cu_width, cu_height, pic_width_in_scu, pic_height_in_scu, mrg_list_cpmv[cnt][lidx], map_cu_mode, &mrg_list_cp_num[cnt], log2_max_cuwh);
                    }
                    else // set to default value
                    {
                        mrg_list_refi[cnt][lidx] = -1;
                        for (i = 0; i < VER_NUM; i++)
                        {
                            mrg_list_cpmv[cnt][lidx][i][MV_X] = 0;
                            mrg_list_cpmv[cnt][lidx][i][MV_Y] = 0;
                        }
                    }
                }
                cnt++;
            }

            if (cnt >= AFF_MODEL_CAND)
            {
                break;
            }
        }
    }

    //-------------------  control point based affine MVP  -------------------//
    {
        s16 cp_mv[REFP_NUM][VER_NUM][MV_D];
        int cp_refi[REFP_NUM][VER_NUM];
        int cp_valid[VER_NUM];

        int neb_addr_lt[AFFINE_MAX_NUM_LT];
        int neb_addr_rt[AFFINE_MAX_NUM_RT];
        int neb_addr_lb;

        int valid_flag_lt[AFFINE_MAX_NUM_LT];
        int valid_flag_rt[AFFINE_MAX_NUM_RT];
        int valid_flag_lb;

        //------------------  INIT  ------------------//
        for (i = 0; i < VER_NUM; i++)
        {
            for (lidx = 0; lidx < REFP_NUM; lidx++)
            {
                cp_mv[lidx][i][MV_X] = 0;
                cp_mv[lidx][i][MV_Y] = 0;
                cp_refi[lidx][i] = -1;
            }
            cp_valid[i] = 0;
        }

        //-------------------  LT  -------------------//
        neb_addr_lt[0] = scup - 1;                     // A
        neb_addr_lt[1] = scup - pic_width_in_scu;      // B
        neb_addr_lt[2] = scup - pic_width_in_scu - 1;  // D

        valid_flag_lt[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[0]]);
        valid_flag_lt[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[1]]);
        valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[2]]);
#if IBC_CHECK_BUGFIX
        valid_flag_lt[0] = valid_flag_lt[0] && !MCU_GET_IBC(map_scu[neb_addr_lt[0]]);
        valid_flag_lt[1] = valid_flag_lt[1] && !MCU_GET_IBC(map_scu[neb_addr_lt[1]]);
        valid_flag_lt[2] = valid_flag_lt[2] && !MCU_GET_IBC(map_scu[neb_addr_lt[2]]);
#endif

        for (k = 0; k < AFFINE_MAX_NUM_LT; k++)
        {
            if (valid_flag_lt[k])
            {
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][0] = map_refi[neb_addr_lt[k]][lidx];
                    cp_mv[lidx][0][MV_X] = map_mv[neb_addr_lt[k]][lidx][MV_X];
                    cp_mv[lidx][0][MV_Y] = map_mv[neb_addr_lt[k]][lidx][MV_Y];
                }
                cp_valid[0] = 1;
                break;
            }
        }

        //-------------------  RT  -------------------//
        neb_addr_rt[0] = scup - pic_width_in_scu + cu_width_in_scu - 1;     // G
        neb_addr_rt[1] = scup - pic_width_in_scu + cu_width_in_scu;         // C
        valid_flag_rt[0] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[0]]);
        valid_flag_rt[1] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[1]]);
#if IBC_CHECK_BUGFIX
        valid_flag_rt[0] = valid_flag_rt[0] && !MCU_GET_IBC(map_scu[neb_addr_rt[0]]);
        valid_flag_rt[1] = valid_flag_rt[1] && !MCU_GET_IBC(map_scu[neb_addr_rt[1]]);
#endif

        for (k = 0; k < AFFINE_MAX_NUM_RT; k++)
        {
            if (valid_flag_rt[k])
            {
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][1] = map_refi[neb_addr_rt[k]][lidx];
                    cp_mv[lidx][1][MV_X] = map_mv[neb_addr_rt[k]][lidx][MV_X];
                    cp_mv[lidx][1][MV_Y] = map_mv[neb_addr_rt[k]][lidx][MV_Y];
                }
                cp_valid[1] = 1;
                break;
            }
        }

        //-------------------  LB  -------------------//
        neb_addr_lb = scup + pic_width_in_scu * (cu_height_in_scu - 1) - 1;  // F
        valid_flag_lb = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lb]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lb]);
#if IBC_CHECK_BUGFIX
        valid_flag_lb = valid_flag_lb && !MCU_GET_IBC(map_scu[neb_addr_lb]);
#endif

        if (valid_flag_lb)
        {
            for (lidx = 0; lidx < REFP_NUM; lidx++)
            {
                cp_refi[lidx][2] = map_refi[neb_addr_lb][lidx];
                cp_mv[lidx][2][MV_X] = map_mv[neb_addr_lb][lidx][MV_X];
                cp_mv[lidx][2][MV_Y] = map_mv[neb_addr_lb][lidx][MV_Y];
            }
            cp_valid[2] = 1;
        }

        //-------------------  RB  -------------------//
        s16 mv_tmp[REFP_NUM][MV_D];
        int neb_addr_rb = scup + pic_width_in_scu * (cu_height_in_scu - 1) + (cu_width_in_scu - 1);     // Col
        int scup_co = get_colocal_scup(neb_addr_rb, pic_width_in_scu, pic_height_in_scu);

        if (slice_type == SLICE_B)
        {
#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
            if (!REFI_IS_VALID(refp[0][REFP_1].map_refi[scup_co][REFP_0]) || refp[0][REFP_1].list_is_library_pic[refp[0][REFP_1].map_refi[scup_co][REFP_0]] || refp[0][REFP_1].is_library_picture || refp[0][REFP_0].is_library_picture)
#else
            if (!REFI_IS_VALID(refp[0][REFP_1].map_refi[scup_co][REFP_0]))
#endif
            {
                cp_valid[3] = 0;
            }
            else
            {
                get_col_mv(refp[0], ptr, scup_co, mv_tmp);
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][3] = 0; // ref idx
                    cp_mv[lidx][3][MV_X] = mv_tmp[lidx][MV_X];
                    cp_mv[lidx][3][MV_Y] = mv_tmp[lidx][MV_Y];
                }
                cp_valid[3] = 1;
            }
        }
        else
        {
            if (REFI_IS_VALID(refp[0][REFP_0].map_refi[scup_co][REFP_0]))
            {
                get_col_mv_from_list0(refp[0], ptr, scup_co, mv_tmp);
                cp_refi[0][3] = 0; // ref idx
                cp_mv[0][3][MV_X] = mv_tmp[0][MV_X];
                cp_mv[0][3][MV_Y] = mv_tmp[0][MV_Y];

                cp_refi[1][3] = REFI_INVALID;
                cp_mv[1][3][MV_X] = 0;
                cp_mv[1][3][MV_Y] = 0;
                cp_valid[3] = 1;
            }
            else
            {
                cp_valid[3] = 0;
            }
        }

        //-------------------  insert model  -------------------//
        int const_order[6] = { 0, 1, 2, 3, 4, 5 };
        int const_num = 6;

        int idx = 0;
        int const_model[6][VER_NUM] =
        {
            { 0, 1, 2 },          // 0: LT, RT, LB
            { 0, 1, 3 },          // 1: LT, RT, RB
            { 0, 2, 3 },          // 2: LT, LB, RB
            { 1, 2, 3 },          // 3: RT, LB, RB
            { 0, 1 },             // 4: LT, RT
            { 0, 2 },             // 5: LT, LB
        };

        int cp_num[6] = { 3, 3, 3, 3, 2, 2 };
        for (idx = 0; idx < const_num; idx++)
        {
            int const_idx = const_order[idx];
            com_derive_affine_constructed_candidate(cu_width, cu_height, cp_valid, cp_mv, cp_refi, const_model[const_idx], const_idx, cp_num[const_idx], mrg_list_cpmv, mrg_list_refi, mrg_list_cp_num, &cnt);
        }
    }

    // Zero padding
    {
        int pad, cp_idx;
        for (pad = cnt; pad < AFF_MAX_NUM_MRG; pad++)
        {
            mrg_list_cp_num[pad] = 2;
            for (lidx = 0; lidx < REFP_NUM; lidx++)
            {
                for (cp_idx = 0; cp_idx < 2; cp_idx++)
                {
                    mrg_list_cpmv[pad][lidx][cp_idx][MV_X] = 0;
                    mrg_list_cpmv[pad][lidx][cp_idx][MV_Y] = 0;
                }
            }
            mrg_list_refi[pad][REFP_0] = 0;
            mrg_list_refi[pad][REFP_1] = REFI_INVALID;
        }
    }

    return cnt;
}

/* inter affine mode */
void com_get_affine_mvp_scaling(COM_INFO *info, COM_MODE * mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int ptr, int lidx, \
                                CPMV mvp[VER_NUM][MV_D], int vertex_num
#if BD_AFFINE_AMVR
                                , u8 curr_mvr
#endif
                               )
{
    int scup = mod_info_curr->scup;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

    int pic_width_in_scu = info->pic_width_in_scu;
    int h_scu = info->pic_height_in_scu;

    s8 cur_refi = mod_info_curr->refi[lidx];

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;
    s8(*map_refi)[REFP_NUM] = pic_map->map_refi; 
    u32* map_scu = pic_map->map_scu; 
    u32* map_cu_mode = pic_map->map_cu_mode;

    int ptr_cur_ref;
    int x_scu = scup % pic_width_in_scu;
    int y_scu = scup / pic_width_in_scu;
    int cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    int cnt = 0, cnt_lt = 0, cnt_rt = 0;
    int i, j, k;
    int refi_tmp;

    s16 mvp_lt[MV_D], mvp_rt[MV_D];
    int neb_addr_lt[AFFINE_MAX_NUM_LT];
    int valid_flag_lt[AFFINE_MAX_NUM_LT];
    int neb_addr_rt[AFFINE_MAX_NUM_RT];
    int valid_flag_rt[AFFINE_MAX_NUM_RT];

    ptr_cur_ref = refp[cur_refi][lidx].ptr;
    for (j = 0; j < VER_NUM; j++)
    {
        mvp[j][MV_X] = 0;
        mvp[j][MV_Y] = 0;
    }

    //-------------------  LT  -------------------//
    neb_addr_lt[0] = scup - 1;                     // A
    neb_addr_lt[1] = scup - pic_width_in_scu;      // B
    neb_addr_lt[2] = scup - pic_width_in_scu - 1;  // D
#if IBC_CHECK_BUGFIX
    valid_flag_lt[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[0]]) && REFI_IS_VALID(map_refi[neb_addr_lt[0]][lidx]);
    valid_flag_lt[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[1]]) && REFI_IS_VALID(map_refi[neb_addr_lt[1]][lidx]);
    valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[2]]) && REFI_IS_VALID(map_refi[neb_addr_lt[2]][lidx]);
#else
    valid_flag_lt[0] = x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[0]]);
    valid_flag_lt[1] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[1]]);
    valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_lt[2]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_lt[2]]);
#endif

    for (k = 0; k < AFFINE_MAX_NUM_LT; k++)
    {
#if IBC_CHECK_BUGFIX
        if (valid_flag_lt[k])
#else
        if (valid_flag_lt[k] && REFI_IS_VALID(map_refi[neb_addr_lt[k]][lidx]))
#endif
        {
            refi_tmp = map_refi[neb_addr_lt[k]][lidx];
#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
            if (refp[refi_tmp][lidx].is_library_picture || refp[cur_refi][lidx].is_library_picture)
            {
                continue;
            }
            else
#endif
            {
                int ptr_neb_ref = refp[refi_tmp][lidx].ptr;
                scaling_mv1(ptr, ptr_cur_ref, ptr, ptr_neb_ref, map_mv[neb_addr_lt[k]][lidx], mvp_lt);
            }
            cnt_lt++;
            break;
        }
    }

    if (cnt_lt == 0)
    {
        mvp_lt[MV_X] = 0;
        mvp_lt[MV_Y] = 0;
        cnt_lt++;
    }

    //-------------------  RT  -------------------//
    neb_addr_rt[0] = scup - pic_width_in_scu + cu_width_in_scu - 1;     // G
    neb_addr_rt[1] = scup - pic_width_in_scu + cu_width_in_scu;         // C
#if IBC_CHECK_BUGFIX
    valid_flag_rt[0] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[0]]) && REFI_IS_VALID(map_refi[neb_addr_rt[0]][lidx]);
    valid_flag_rt[1] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[1]]) && REFI_IS_VALID(map_refi[neb_addr_rt[1]][lidx]);
#else
    valid_flag_rt[0] = y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[0]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[0]]);
    valid_flag_rt[1] = y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr_rt[1]]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr_rt[1]]);
#endif

    for (k = 0; k < AFFINE_MAX_NUM_RT; k++)
    {
#if IBC_CHECK_BUGFIX
        if (valid_flag_rt[k])
#else
        if (valid_flag_rt[k] && REFI_IS_VALID(map_refi[neb_addr_rt[k]][lidx]))
#endif
        {
            refi_tmp = map_refi[neb_addr_rt[k]][lidx];
#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
            if (refp[refi_tmp][lidx].is_library_picture || refp[cur_refi][lidx].is_library_picture)
            {
                continue;
            }
            else
#endif
            {
                int ptr_neb_ref = refp[refi_tmp][lidx].ptr;
                scaling_mv1(ptr, ptr_cur_ref, ptr, ptr_neb_ref, map_mv[neb_addr_rt[k]][lidx], mvp_rt);
            }
            cnt_rt++;
            break;
        }
    }

    if (cnt_rt == 0)
    {
        mvp_rt[MV_X] = 0;
        mvp_rt[MV_Y] = 0;
        cnt_rt++;
    }

    mvp[0][MV_X] = mvp_lt[MV_X];
    mvp[0][MV_Y] = mvp_lt[MV_Y];
    mvp[1][MV_X] = mvp_rt[MV_X];
    mvp[1][MV_Y] = mvp_rt[MV_Y];

#if BD_AFFINE_AMVR
    for (i = 0; i < 2; i++)
    {
        // convert to 1/16 precision
        s32 mvp_x = (s32)mvp[i][MV_X] << 2;
        s32 mvp_y = (s32)mvp[i][MV_Y] << 2;

        // rounding
        int amvr_shift = Tab_Affine_AMVR(curr_mvr);
        com_mv_rounding_s32(mvp_x, mvp_y, &mvp_x, &mvp_y, amvr_shift, amvr_shift);

        // clipping
        mvp[i][MV_X] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mvp_x);
        mvp[i][MV_Y] = (CPMV)COM_CLIP3(COM_CPMV_MIN, COM_CPMV_MAX, mvp_y);
    }
#endif
}

#if ETMVP
void derive_first_stage_motion(u8 ibc_flag, int scup, int cu_width, int cu_height, int pic_width_in_scu, u32 *map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION *first_stage_motion)
{
    s32 i = 0;
    s32 j = 0;
    s32 x_scu = scup % pic_width_in_scu;
    s32 y_scu = scup / pic_width_in_scu;
    s32 cu_width_in_scu = cu_width >> MIN_CU_LOG2;
    s32 cu_height_in_scu = cu_height >> MIN_CU_LOG2;
    s32 neb_addr = 0;
    s32 valid_flag = 0;

    first_stage_motion->mv[REFP_0][MV_X] = 0;
    first_stage_motion->mv[REFP_0][MV_Y] = 0;
    first_stage_motion->mv[REFP_1][MV_X] = 0;
    first_stage_motion->mv[REFP_1][MV_Y] = 0;
    first_stage_motion->ref_idx[REFP_0] = REFI_INVALID;
    first_stage_motion->ref_idx[REFP_1] = REFI_INVALID;

    if (x_scu)
    {
        neb_addr = scup + (cu_height_in_scu - 1) * pic_width_in_scu - 1;
        valid_flag = MCU_GET_CODED_FLAG(map_scu[neb_addr]) && !MCU_GET_INTRA_FLAG(map_scu[neb_addr]);
#if USE_IBC
        if (ibc_flag)
        {
            valid_flag = valid_flag && (!MCU_GET_IBC(map_scu[neb_addr]));
        }
#endif
        if (valid_flag)
        {
            if (REFI_IS_VALID(map_refi[neb_addr][REFP_0]))
            {
                first_stage_motion->mv[REFP_0][MV_X] = map_mv[neb_addr][REFP_0][MV_X];
                first_stage_motion->mv[REFP_0][MV_Y] = map_mv[neb_addr][REFP_0][MV_Y];
                first_stage_motion->ref_idx[REFP_0] = map_refi[neb_addr][REFP_0];
            }
            if (REFI_IS_VALID(map_refi[neb_addr][REFP_1]))
            {
                first_stage_motion->mv[REFP_1][MV_X] = map_mv[neb_addr][REFP_1][MV_X];
                first_stage_motion->mv[REFP_1][MV_Y] = map_mv[neb_addr][REFP_1][MV_Y];
                first_stage_motion->ref_idx[REFP_1] = map_refi[neb_addr][REFP_1];
            }
        }
    }
}

void adjust_first_stage_motion(s32 frameType, int scup, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], COM_MOTION *first_stage_motion)
{
    s8 neighbor_ref_idx = 0;
    s32 ptr_neighbor_ref = 0;
    s16 mv[MV_D] = { 0 };
    s32 pred_dir = REFP_1;
    s32 ref_dir = first_stage_motion->ref_idx[pred_dir] >= 0 ? pred_dir : (first_stage_motion->ref_idx[1 - pred_dir] >= 0 ? 1 - pred_dir : -1);

    if (frameType == SLICE_P)
    {
        pred_dir = REFP_0;
        ref_dir = first_stage_motion->ref_idx[pred_dir] >= 0 ? pred_dir : -1;
    }

    if ((ref_dir != -1) && (first_stage_motion->ref_idx[pred_dir] != 0))
    {
        neighbor_ref_idx = first_stage_motion->ref_idx[ref_dir];
        ptr_neighbor_ref = refp[neighbor_ref_idx][ref_dir].ptr;
        mv[MV_X] = first_stage_motion->mv[ref_dir][MV_X];
        mv[MV_Y] = first_stage_motion->mv[ref_dir][MV_Y];

        scaling_mv1(cur_ptr, refp[0][pred_dir].ptr, cur_ptr, ptr_neighbor_ref, mv, first_stage_motion->mv[pred_dir]);
    }
    first_stage_motion->ref_idx[pred_dir] = 0;
}

void derive_ref_block_position(s32 frameType, int pic_width_in_scu, int pic_height_in_scu, s32 pic_width, s32 pic_height, int cu_width, int cu_height, s32 x_ctb_pos, s32 y_ctb_pos, s32 x_pos, s32 y_pos, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], s32 *ref_block_x, s32 *ref_block_y, COM_MOTION *first_stage_motion)
{
    s16 mv[MV_D] = { 0 };
    s32 tmp_ref_x = 0;
    s32 tmp_ref_y = 0;
    int scup = 0;
    s32 cu_width_in_scu = (cu_width >> 2);
    s32 cu_height_in_scu = (cu_height >> 2);
    s32 ref_dir = REFP_1;

    if (frameType == SLICE_P)
    {
        ref_dir = REFP_0;
    }

    first_stage_motion->mv[ref_dir][MV_X] = 0;
    first_stage_motion->mv[ref_dir][MV_Y] = 0;

    mv[MV_X] = (first_stage_motion->mv[ref_dir][MV_X] >> 2) + 4;
    mv[MV_Y] = (first_stage_motion->mv[ref_dir][MV_Y] >> 2) + 4;

    tmp_ref_x = x_pos + mv[MV_X];
    tmp_ref_y = y_pos + mv[MV_Y];
    *ref_block_x = ((tmp_ref_x >> 3) << 3);
    *ref_block_y = ((tmp_ref_y >> 3) << 3);

    *ref_block_x = (COM_CLIP(*ref_block_x, x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - cu_width, pic_width - cu_width)));
    *ref_block_y = (COM_CLIP(*ref_block_y, y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - cu_height, pic_height - cu_height)));
}

void set_etmvp_mvfield(s32 frameType, s32 cur_ptr, s32 ref_x, s32 ref_y, s32 cu_width, s32 cu_height, s32 pic_width, s32 pic_height, s32 x_ctb_pos, s32 y_ctb_pos, int pic_width_in_scu, int pic_height_in_scu, COM_REFP(*refp)[REFP_NUM], COM_MOTION *etmvp_mvfield, COM_MOTION first_stage_motion)
{
    s32 cu_width_in_scu = (cu_width >> 2);
    s32 cu_height_in_scu = (cu_height >> 2);
    s32 scup = 0;
    s32 ref_dir = REFP_1;

    if (frameType == SLICE_P)
    {
        ref_dir = REFP_0;
    }

    s32 tmp_ref_x = ref_x;
    s32 tmp_ref_y = ref_y;
    COM_MOTION tmporal_motion;

    assert(((ref_x % 8) == 0) && ((ref_y % 8) == 0));

    for (int h = 0; h < cu_height_in_scu; h++)
    {
        for (int w = 0; w < cu_width_in_scu; w++)
        {
            ref_x = tmp_ref_x + w * 4;
            ref_y = tmp_ref_y + h * 4;

            ref_x = (COM_CLIP(ref_x, x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - 1, pic_width - 1)) >> 2);
            ref_y = (COM_CLIP(ref_y, y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - 1, pic_height - 1)) >> 2);

            scup = ref_y * pic_width_in_scu + ref_x;
            scup = get_colocal_scup(scup, pic_width_in_scu, pic_height_in_scu);
            drive_scaled_tmporal_motion(scup, ref_dir, refp, cur_ptr, &tmporal_motion);

            if (!(REFI_IS_VALID(tmporal_motion.ref_idx[REFP_0]) || REFI_IS_VALID(tmporal_motion.ref_idx[REFP_1])))
            {
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
                etmvp_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];

                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
                etmvp_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            }
            else
            {
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_X] = tmporal_motion.mv[REFP_0][MV_X];
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_0][MV_Y] = tmporal_motion.mv[REFP_0][MV_Y];
                etmvp_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_0] = tmporal_motion.ref_idx[REFP_0];

                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_X] = tmporal_motion.mv[REFP_1][MV_X];
                etmvp_mvfield[w + h * cu_width_in_scu].mv[REFP_1][MV_Y] = tmporal_motion.mv[REFP_1][MV_Y];
                etmvp_mvfield[w + h * cu_width_in_scu].ref_idx[REFP_1] = tmporal_motion.ref_idx[REFP_1];
            }
        }
    }
}

void drive_scaled_tmporal_motion(int scup, s32 ref_dir, COM_REFP(*refp)[REFP_NUM], s32 cur_ptr, COM_MOTION *tmporal_motion)
{
    s32 ptr_tmporal_ref = 0;
    s16 mv[MV_D] = { 0 };

    if (REFI_IS_VALID(refp[0][ref_dir].map_refi[scup][REFP_0]))
    {
        ptr_tmporal_ref = refp[0][ref_dir].list_ptr[REFP_0][refp[0][ref_dir].map_refi[scup][REFP_0]];
        scaling_mv1(cur_ptr, refp[0][REFP_0].ptr, refp[0][ref_dir].ptr, ptr_tmporal_ref, refp[0][ref_dir].map_mv[scup][REFP_0], mv);

        tmporal_motion->mv[REFP_0][MV_X] = mv[MV_X];
        tmporal_motion->mv[REFP_0][MV_Y] = mv[MV_Y];
        tmporal_motion->ref_idx[REFP_0] = 0;
    }
    else
    {
        tmporal_motion->mv[REFP_0][MV_X] = 0;
        tmporal_motion->mv[REFP_0][MV_Y] = 0;
        tmporal_motion->ref_idx[REFP_0] = REFI_INVALID;
    }


    if (REFI_IS_VALID(refp[0][ref_dir].map_refi[scup][REFP_1]))
    {
        ptr_tmporal_ref = refp[0][ref_dir].list_ptr[REFP_1][refp[0][ref_dir].map_refi[scup][REFP_1]];
        scaling_mv1(cur_ptr, refp[0][REFP_1].ptr, refp[0][ref_dir].ptr, ptr_tmporal_ref, refp[0][ref_dir].map_mv[scup][REFP_1], mv);

        tmporal_motion->mv[REFP_1][MV_X] = mv[MV_X];
        tmporal_motion->mv[REFP_1][MV_Y] = mv[MV_Y];
        tmporal_motion->ref_idx[REFP_1] = 0;
    }
    else
    {
        tmporal_motion->mv[REFP_1][MV_X] = 0;
        tmporal_motion->mv[REFP_1][MV_Y] = 0;
        tmporal_motion->ref_idx[REFP_1] = REFI_INVALID;
    }

}

int get_valid_etmvp_motion(s32 frameType, s32 cur_ptr, s32 cu_width, s32 cu_height, s32 x_ctb_pos, s32 y_ctb_pos, s32 pic_width, s32 pic_height, s32 pic_width_in_scu, s32 pic_height_in_scu, s32 ref_x, s32 ref_y, COM_REFP(*refp)[REFP_NUM], s32 *valid_etmvp_offset, COM_MOTION first_stage_motion)
{
    s32 cu_width_in_scu = (cu_width >> 2);
    s32 cu_height_in_scu = (cu_height >> 2);
    s32 scup = 0;
    s32 tmp_ref_x = ref_x;
    s32 tmp_ref_y = ref_y;
    s32 valid_etmvp_num = 1;
    COM_MOTION tmporal_motion[8];
    s32 ref_dir = REFP_1;

    if (frameType == SLICE_P)
    {
        ref_dir = REFP_0;
    }

    if (((tmp_ref_y + cu_height + MIN_ETMVP_SIZE) <= (y_ctb_pos + MAX_CU_SIZE)) && ((tmp_ref_y + cu_height + MIN_ETMVP_SIZE) <= (pic_height)))
    {
        //垂直偏移模式查重
        for (s32 i = 0; i < 4; i++)
        {
            ref_x = (COM_CLIP(tmp_ref_x + ((i >> 1) % 2) * (cu_width - 1), x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - 1, pic_width - 1)) >> 2);
            ref_y = (COM_CLIP(tmp_ref_y + (i % 2) * cu_height - (i >> 2), y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - 1, pic_height - 1)) >> 2);
            scup = ref_y * pic_width_in_scu + ref_x;
            scup = get_colocal_scup(scup, pic_width_in_scu, pic_height_in_scu);
            drive_scaled_tmporal_motion(scup, ref_dir, refp, cur_ptr, &tmporal_motion[i]);

            if (!(REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_0]) || REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_1])))
            {
                tmporal_motion[i].mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
                tmporal_motion[i].mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
                tmporal_motion[i].ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];

                tmporal_motion[i].mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
                tmporal_motion[i].mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
                tmporal_motion[i].ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            }
        }
        if ((!same_motion(tmporal_motion[0], tmporal_motion[1])) || (!same_motion(tmporal_motion[2], tmporal_motion[3])))
        {
            valid_etmvp_offset[valid_etmvp_num++] = 3;
        }
    }

    if (((tmp_ref_x + cu_width + MIN_ETMVP_SIZE) <= (x_ctb_pos + MAX_CU_SIZE)) && ((tmp_ref_x + cu_width + MIN_ETMVP_SIZE) <= (pic_width)))
    {
        //水平偏移模式查重
        for (s32 i = 0; i < 4; i++)
        {
            ref_x = (COM_CLIP(tmp_ref_x + (i % 2) * cu_width - (i >> 2), x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - 1, pic_width - 1)) >> 2);
            ref_y = (COM_CLIP(tmp_ref_y + ((i >> 1) % 2) * (cu_height - 1), y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - 1, pic_height - 1)) >> 2);
            scup = ref_y * pic_width_in_scu + ref_x;
            scup = get_colocal_scup(scup, pic_width_in_scu, pic_height_in_scu);
            drive_scaled_tmporal_motion(scup, ref_dir, refp, cur_ptr, &tmporal_motion[i]);

            if (!(REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_0]) || REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_1])))
            {
                tmporal_motion[i].mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
                tmporal_motion[i].mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
                tmporal_motion[i].ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];

                tmporal_motion[i].mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
                tmporal_motion[i].mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
                tmporal_motion[i].ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            }
        }

        if ((!same_motion(tmporal_motion[0], tmporal_motion[1])) || (!same_motion(tmporal_motion[2], tmporal_motion[3])))
        {
            valid_etmvp_offset[valid_etmvp_num++] = 1;
        }
    }

    if ((tmp_ref_y - MIN_ETMVP_SIZE) >= y_ctb_pos)
    {
        //垂直偏移模式查重
        for (s32 i = 4; i < 8; i++)
        {
            ref_x = (COM_CLIP(tmp_ref_x + ((i >> 1) % 2) * (cu_width - 1), x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - 1, pic_width - 1)) >> 2);
            ref_y = (COM_CLIP(tmp_ref_y + (i % 2) * cu_height - (i >> 2), y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - 1, pic_height - 1)) >> 2);
            scup = ref_y * pic_width_in_scu + ref_x;
            scup = get_colocal_scup(scup, pic_width_in_scu, pic_height_in_scu);
            drive_scaled_tmporal_motion(scup, ref_dir, refp, cur_ptr, &tmporal_motion[i]);

            if (!(REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_0]) || REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_1])))
            {
                tmporal_motion[i].mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
                tmporal_motion[i].mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
                tmporal_motion[i].ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];

                tmporal_motion[i].mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
                tmporal_motion[i].mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
                tmporal_motion[i].ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            }
        }

        if ((!same_motion(tmporal_motion[4], tmporal_motion[5])) || (!same_motion(tmporal_motion[6], tmporal_motion[7])))
        {
            valid_etmvp_offset[valid_etmvp_num++] = 4;
        }
    }

    if ((tmp_ref_x - MIN_ETMVP_SIZE) >= x_ctb_pos)
    {
        //水平偏移模式查重
        for (s32 i = 4; i < 8; i++)
        {
            ref_x = (COM_CLIP(tmp_ref_x + (i % 2) * cu_width - (i >> 2), x_ctb_pos, min(x_ctb_pos + MAX_CU_SIZE - 1, pic_width - 1)) >> 2);
            ref_y = (COM_CLIP(tmp_ref_y + ((i >> 1) % 2) * (cu_height - 1), y_ctb_pos, min(y_ctb_pos + MAX_CU_SIZE - 1, pic_height - 1)) >> 2);
            scup = ref_y * pic_width_in_scu + ref_x;
            scup = get_colocal_scup(scup, pic_width_in_scu, pic_height_in_scu);
            drive_scaled_tmporal_motion(scup, ref_dir, refp, cur_ptr, &tmporal_motion[i]);

            if (!(REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_0]) || REFI_IS_VALID(tmporal_motion[i].ref_idx[REFP_1])))
            {
                tmporal_motion[i].mv[REFP_0][MV_X] = first_stage_motion.mv[REFP_0][MV_X];
                tmporal_motion[i].mv[REFP_0][MV_Y] = first_stage_motion.mv[REFP_0][MV_Y];
                tmporal_motion[i].ref_idx[REFP_0] = first_stage_motion.ref_idx[REFP_0];

                tmporal_motion[i].mv[REFP_1][MV_X] = first_stage_motion.mv[REFP_1][MV_X];
                tmporal_motion[i].mv[REFP_1][MV_Y] = first_stage_motion.mv[REFP_1][MV_Y];
                tmporal_motion[i].ref_idx[REFP_1] = first_stage_motion.ref_idx[REFP_1];
            }
        }
        if ((!same_motion(tmporal_motion[4], tmporal_motion[5])) || (!same_motion(tmporal_motion[6], tmporal_motion[7])))
        {
            valid_etmvp_offset[valid_etmvp_num++] = 2;
        }
    }

    return valid_etmvp_num;
}

void derive_scaled_base_motion(s32 frameType, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], COM_MOTION *base_motion, COM_MOTION *first_stage_motion)
{
    s16 mv[MV_D] = { 0 };
    s8 neighbor_ref_idx = 0;
    s32 ptr_neighbor_ref = 0;

    if (base_motion->ref_idx[REFP_0] > 0)
    {
        neighbor_ref_idx = base_motion->ref_idx[REFP_0];
        ptr_neighbor_ref = refp[neighbor_ref_idx][REFP_0].ptr;
        mv[MV_X] = base_motion->mv[REFP_0][MV_X];
        mv[MV_Y] = base_motion->mv[REFP_0][MV_Y];

        scaling_mv1(cur_ptr, refp[0][REFP_0].ptr, cur_ptr, ptr_neighbor_ref, mv, base_motion->mv[REFP_0]);

        base_motion->ref_idx[REFP_0] = 0;
    }

    if (base_motion->ref_idx[REFP_1] > 0)
    {
        neighbor_ref_idx = base_motion->ref_idx[REFP_1];
        ptr_neighbor_ref = refp[neighbor_ref_idx][REFP_1].ptr;
        mv[MV_X] = base_motion->mv[REFP_1][MV_X];
        mv[MV_Y] = base_motion->mv[REFP_1][MV_Y];

        scaling_mv1(cur_ptr, refp[0][REFP_1].ptr, cur_ptr, ptr_neighbor_ref, mv, base_motion->mv[REFP_1]);

        base_motion->ref_idx[REFP_1] = 0;
    }

    if (frameType == SLICE_B && !(REFI_IS_VALID(base_motion->ref_idx[REFP_0]) || REFI_IS_VALID(base_motion->ref_idx[REFP_1])))
    {
        base_motion->ref_idx[REFP_0] =
            base_motion->ref_idx[REFP_1] = 0;
    }

    if (frameType == SLICE_P && !REFI_IS_VALID(base_motion->ref_idx[REFP_0]))
    {
        base_motion->ref_idx[REFP_0] = 0;
        base_motion->ref_idx[REFP_1] = REFI_INVALID;
    }

    first_stage_motion->mv[REFP_0][MV_X] = base_motion->mv[REFP_0][MV_X];
    first_stage_motion->mv[REFP_0][MV_Y] = base_motion->mv[REFP_0][MV_Y];
    first_stage_motion->ref_idx[REFP_0] = base_motion->ref_idx[REFP_0];

    first_stage_motion->mv[REFP_1][MV_X] = base_motion->mv[REFP_1][MV_X];
    first_stage_motion->mv[REFP_1][MV_Y] = base_motion->mv[REFP_1][MV_Y];
    first_stage_motion->ref_idx[REFP_1] = base_motion->ref_idx[REFP_1];
}
#endif

void com_sbac_ctx_init(COM_SBAC_CTX *sbac_ctx)
{
    int i, num;
    SBAC_CTX_MODEL *p;
    com_mset(sbac_ctx, 0x00, sizeof(*sbac_ctx));

    /* Initialization of the context models */
    num = sizeof(COM_SBAC_CTX) / sizeof(SBAC_CTX_MODEL);
    p = (SBAC_CTX_MODEL*)sbac_ctx;

    for (i = 0; i < num; i++)
    {
        p[i] = PROB_INIT;
    }
}


int com_split_part_count(int split_mode)
{
    switch (split_mode)
    {
    case SPLIT_BI_VER:
    case SPLIT_BI_HOR:
        return 2;
#if EQT
    case SPLIT_EQT_VER:
    case SPLIT_EQT_HOR:
        return 4;
#endif
    case SPLIT_QUAD:
        return 4;
    default:
        // NO_SPLIT
        return 0;
    }
}

int com_split_get_part_size(int split_mode, int part_num, int length)
{
    int ans = length;
    switch (split_mode)
    {
    case SPLIT_QUAD:
    case SPLIT_BI_HOR:
    case SPLIT_BI_VER:
        ans = length >> 1;
        break;
#if EQT
    case SPLIT_EQT_HOR:
    case SPLIT_EQT_VER:
        if ((part_num == 1)||(part_num == 2))
            ans = length >> 1;
        else
            ans = length >> 2;
        break;
#endif
    }
    return ans;
}

int com_split_get_part_size_idx(int split_mode, int part_num, int length_idx)
{
    int ans = length_idx;
    switch (split_mode)
    {
    case SPLIT_QUAD:
    case SPLIT_BI_HOR:
    case SPLIT_BI_VER:
        ans = length_idx - 1;
        break;
#if EQT
    case SPLIT_EQT_HOR:
    case SPLIT_EQT_VER:
        if ((part_num == 1) || (part_num == 2))
            ans = length_idx - 1;
        else
            ans = length_idx - 2;
        break;
#endif
    }
    return ans;
}

void com_split_get_part_structure(int split_mode, int x0, int y0, int cu_width, int cu_height, int cup, int cud, int log2_culine, COM_SPLIT_STRUCT* split_struct)
{
    int i;
    int log_cuw, log_cuh;
    int cup_w, cup_h;
    split_struct->part_count = com_split_part_count(split_mode);
    log_cuw = CONV_LOG2(cu_width);
    log_cuh = CONV_LOG2(cu_height);
    split_struct->x_pos[0] = x0;
    split_struct->y_pos[0] = y0;
    split_struct->cup[0] = cup;
    switch (split_mode)
    {
    case NO_SPLIT:
    {
        split_struct->width[0] = cu_width;
        split_struct->height[0] = cu_height;
        split_struct->log_cuw[0] = log_cuw;
        split_struct->log_cuh[0] = log_cuh;
    }
    break;
    case SPLIT_QUAD:
    {
        split_struct->width[0] = cu_width >> 1;
        split_struct->height[0] = cu_height >> 1;
        split_struct->log_cuw[0] = log_cuw - 1;
        split_struct->log_cuh[0] = log_cuh - 1;
        for (i = 1; i < split_struct->part_count; ++i)
        {
            split_struct->width[i] = split_struct->width[0];
            split_struct->height[i] = split_struct->height[0];
            split_struct->log_cuw[i] = split_struct->log_cuw[0];
            split_struct->log_cuh[i] = split_struct->log_cuh[0];
        }
        split_struct->x_pos[1] = x0 + split_struct->width[0];
        split_struct->y_pos[1] = y0;
        split_struct->x_pos[2] = x0;
        split_struct->y_pos[2] = y0 + split_struct->height[0];
        split_struct->x_pos[3] = split_struct->x_pos[1];
        split_struct->y_pos[3] = split_struct->y_pos[2];
        cup_w = (split_struct->width[0] >> MIN_CU_LOG2);
        cup_h = ((split_struct->height[0] >> MIN_CU_LOG2) << log2_culine);
        split_struct->cup[1] = cup + cup_w;
        split_struct->cup[2] = cup + cup_h;
        split_struct->cup[3] = split_struct->cup[1] + cup_h;
        split_struct->cud = cud + 1;
    }
    break;
    default:
    {
        if (com_split_is_vertical(split_mode))
        {
            for (i = 0; i < split_struct->part_count; ++i)
            {
                split_struct->width[i] = com_split_get_part_size(split_mode, i, cu_width);
                split_struct->log_cuw[i] = com_split_get_part_size_idx(split_mode, i, log_cuw);
#if EQT
                if (split_mode == SPLIT_EQT_VER)
                {
                    if (i == 0 || i == 3)
                    {
                        split_struct->height[i] = cu_height;
                        split_struct->log_cuh[i] = log_cuh;
                    }
                    else
                    {
                        split_struct->height[i] = cu_height >> 1;
                        split_struct->log_cuh[i] = log_cuh - 1;
                    }
                }
                else
                {
                    split_struct->height[i] = cu_height;
                    split_struct->log_cuh[i] = log_cuh;
                    if (i)
                    {
                        split_struct->x_pos[i] = split_struct->x_pos[i - 1] + split_struct->width[i - 1];
                        split_struct->y_pos[i] = split_struct->y_pos[i - 1];
                        split_struct->cup[i] = split_struct->cup[i - 1] + (split_struct->width[i - 1] >> MIN_CU_LOG2);
                    }
                }
#else
                split_struct->height[i] = cu_height;
                split_struct->log_cuh[i] = log_cuh;
                if (i)
                {
                    split_struct->x_pos[i] = split_struct->x_pos[i - 1] + split_struct->width[i - 1];
                    split_struct->y_pos[i] = split_struct->y_pos[i - 1];
                    split_struct->cup[i] = split_struct->cup[i - 1] + (split_struct->width[i - 1] >> MIN_CU_LOG2);
                }
#endif
            }
#if EQT
            if (split_mode == SPLIT_EQT_VER)
            {
                split_struct->x_pos[1] = split_struct->x_pos[0] + split_struct->width[0];
                split_struct->y_pos[1] = split_struct->y_pos[0];
                split_struct->cup[1] = split_struct->cup[0] + (split_struct->width[0] >> MIN_CU_LOG2);
                cup_h = ((split_struct->height[1] >> MIN_CU_LOG2) << log2_culine);
                split_struct->x_pos[2] = split_struct->x_pos[1];
                split_struct->y_pos[2] = split_struct->y_pos[1] + split_struct->height[1];
                split_struct->cup[2] = split_struct->cup[1] + cup_h;
                split_struct->x_pos[3] = split_struct->x_pos[1] + split_struct->width[1];
                split_struct->y_pos[3] = split_struct->y_pos[1];
                split_struct->cup[3] = split_struct->cup[1] + (split_struct->width[1] >> MIN_CU_LOG2);
            }
#endif
        }
        else
        {
            for (i = 0; i < split_struct->part_count; ++i)
            {
#if EQT
                if (split_mode == SPLIT_EQT_HOR)
                {
                    if (i == 0 || i == 3)
                    {
                        split_struct->width[i] = cu_width;
                        split_struct->log_cuw[i] = log_cuw;
                    }
                    else
                    {
                        split_struct->width[i] = cu_width >> 1;
                        split_struct->log_cuw[i] = log_cuw - 1;
                    }
                }
                else
                {
                    split_struct->width[i] = cu_width;
                    split_struct->log_cuw[i] = log_cuw;
                    if (i)
                    {
                        split_struct->y_pos[i] = split_struct->y_pos[i - 1] + split_struct->height[i - 1];
                        split_struct->x_pos[i] = split_struct->x_pos[i - 1];
                        split_struct->cup[i] = split_struct->cup[i - 1] + ((split_struct->height[i - 1] >> MIN_CU_LOG2) << log2_culine);
                    }
                }
#else
                split_struct->width[i] = cu_width;
                split_struct->log_cuw[i] = log_cuw;
                if (i)
                {
                    split_struct->y_pos[i] = split_struct->y_pos[i - 1] + split_struct->height[i - 1];
                    split_struct->x_pos[i] = split_struct->x_pos[i - 1];
                    split_struct->cup[i] = split_struct->cup[i - 1] + ((split_struct->height[i - 1] >> MIN_CU_LOG2) << log2_culine);
                }
#endif
                split_struct->height[i] = com_split_get_part_size(split_mode, i, cu_height);
                split_struct->log_cuh[i] = com_split_get_part_size_idx(split_mode, i, log_cuh);
            }
#if EQT
            if (split_mode == SPLIT_EQT_HOR)
            {
                split_struct->y_pos[1] = split_struct->y_pos[0] + split_struct->height[0];
                split_struct->x_pos[1] = split_struct->x_pos[0];
                split_struct->cup[1] = split_struct->cup[0] + ((split_struct->height[0] >> MIN_CU_LOG2) << log2_culine);
                split_struct->y_pos[2] = split_struct->y_pos[1];
                split_struct->x_pos[2] = split_struct->x_pos[1] + split_struct->width[1];
                split_struct->cup[2] = split_struct->cup[1] + (split_struct->width[1] >> MIN_CU_LOG2);
                split_struct->y_pos[3] = split_struct->y_pos[1] + split_struct->height[1];
                split_struct->x_pos[3] = split_struct->x_pos[1];
                split_struct->cup[3] = split_struct->cup[1] + ((split_struct->height[1] >> MIN_CU_LOG2) << log2_culine);
            }
#endif
        }
        switch (split_mode)
        {
        case SPLIT_BI_VER:
            split_struct->cud = cud + ((cu_width == cu_height || cu_width < cu_height) ? 0 : 1);
            break;
        case SPLIT_BI_HOR:
            split_struct->cud = cud + ((cu_width == cu_height || cu_width > cu_height) ? 0 : 1);
            break;
        default:
            // Triple tree case
            split_struct->cud = cud + (cu_width == cu_height ? 0 : 1);
            break;
        }
    }
    break;
    }
}

void com_split_get_split_rdo_order(int cu_width, int cu_height, SPLIT_MODE splits[MAX_SPLIT_NUM])
{
    splits[0] = NO_SPLIT;
    //qt must be tried first; otherwise, due to the split save & load fast algorithm, qt will be never tried in RDO (previous split decision is made base on bt/eqt)
    splits[1] = SPLIT_QUAD;
    splits[2] = cu_width < cu_height ? SPLIT_BI_HOR : SPLIT_BI_VER;
    splits[3] = cu_width < cu_height ? SPLIT_BI_VER : SPLIT_BI_HOR;
#if EQT
    splits[4] = cu_width < cu_height ? SPLIT_EQT_HOR : SPLIT_EQT_VER;
    splits[5] = cu_width < cu_height ? SPLIT_EQT_VER : SPLIT_EQT_HOR;
#endif
}

SPLIT_DIR com_split_get_direction(SPLIT_MODE mode)
{
    switch (mode)
    {
    case SPLIT_BI_HOR:
#if EQT
    case SPLIT_EQT_HOR:
#endif
        return SPLIT_HOR;
    case SPLIT_BI_VER:
#if EQT
    case SPLIT_EQT_VER:
        return SPLIT_VER;
#endif
    default:
        return SPLIT_QT;
    }
}

int com_split_is_vertical(SPLIT_MODE mode)
{
    return com_split_get_direction(mode) == SPLIT_VER ? 1 : 0;
}

int com_split_is_horizontal(SPLIT_MODE mode)
{
    return com_split_get_direction(mode) == SPLIT_HOR ? 1 : 0;
}

#if EQT
int  com_split_is_EQT(SPLIT_MODE mode)
{
    return (mode == SPLIT_EQT_HOR) || (mode == SPLIT_EQT_VER) ? 1 : 0;
}
#endif
int  com_split_is_BT(SPLIT_MODE mode)
{
    return (mode == SPLIT_BI_HOR) || (mode == SPLIT_BI_VER) ? 1 : 0;
}

#if DT_SYNTAX
/**确定当前CU是否满足DT的条件
*/
int com_dt_allow(int cu_w, int cu_h, int pred_mode, int max_dt_size)
{
    //only allow intra DT
    if (pred_mode != MODE_INTRA)
        return 0;

    int max_size = max_dt_size;
    int min_size = 16;
    int hori_allow = cu_h >= min_size && (cu_w <= max_size && cu_h <= max_size) && cu_w < cu_h * 4;//水平DT允许:宽高比小于4，CU不超过最大DTSize，高大于最小DTSize
    int vert_allow = cu_w >= min_size && (cu_w <= max_size && cu_h <= max_size) && cu_h < cu_w * 4;//垂直DT允许:高宽比小于4，CU不超过最大DTSize，宽大于最小DTSize

    return hori_allow + (vert_allow << 1);
}
#endif


#if TB_SPLIT_EXT
void init_tb_part(COM_MODE *mod_info_curr)
{
    mod_info_curr->tb_part = SIZE_2Nx2N;
}

void init_pb_part(COM_MODE *mod_info_curr)
{
    mod_info_curr->pb_part = SIZE_2Nx2N;
    cu_nz_cln(mod_info_curr->num_nz);
}

void set_pb_part(COM_MODE *mod_info_curr, int part_size)
{
    mod_info_curr->pb_part = part_size;
}

void set_tb_part(COM_MODE *mod_info_curr, int part_size)
{
    mod_info_curr->tb_part = part_size;
}

/**/
void get_part_info(int pic_width_in_scu, int x, int y, int w, int h, int part_size, COM_PART_INFO* sub_info)
{
    int i;
    int qw = w >> 2;//转化成scu计数
    int qh = h >> 2;//转化成scu计数
    int x_scu, y_scu;
    memset(sub_info, 0, sizeof(COM_PART_INFO));//初始化子块信息

    //derive sub_part x, y, w, h
    switch (part_size)
    {
    case SIZE_2Nx2N:
        sub_info->num_sub_part = 1;
        sub_info->sub_x[0] = x;
        sub_info->sub_y[0] = y;
        sub_info->sub_w[0] = w;
        sub_info->sub_h[0] = h;
        break;
    case SIZE_2NxhN:
        sub_info->num_sub_part = 4;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = x;
            sub_info->sub_y[i] = qh * i + y;
            sub_info->sub_w[i] = w;
            sub_info->sub_h[i] = qh;
        }
        break;
    case SIZE_2NxnU:
        sub_info->num_sub_part = 2;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = x;
            sub_info->sub_y[i] = qh * (i == 0 ? 0 : 1) + y;
            sub_info->sub_w[i] = w;
            sub_info->sub_h[i] = qh * (i == 0 ? 1 : 3);
        }
        break;
    case SIZE_2NxnD:
        sub_info->num_sub_part = 2;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = x;
            sub_info->sub_y[i] = qh * (i == 0 ? 0 : 3) + y;
            sub_info->sub_w[i] = w;
            sub_info->sub_h[i] = qh * (i == 0 ? 3 : 1);
        }
        break;
    case SIZE_hNx2N:
        sub_info->num_sub_part = 4;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = qw * i + x;
            sub_info->sub_y[i] = y;
            sub_info->sub_w[i] = qw;
            sub_info->sub_h[i] = h;
        }
        break;
    case SIZE_nLx2N:
        sub_info->num_sub_part = 2;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = qw * (i == 0 ? 0 : 1) + x;
            sub_info->sub_y[i] = y;
            sub_info->sub_w[i] = qw * (i == 0 ? 1 : 3);
            sub_info->sub_h[i] = h;
        }
        break;
    case SIZE_nRx2N:
        sub_info->num_sub_part = 2;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = qw * (i == 0 ? 0 : 3) + x;
            sub_info->sub_y[i] = y;
            sub_info->sub_w[i] = qw * (i == 0 ? 3 : 1);
            sub_info->sub_h[i] = h;
        }
        break;
    case SIZE_NxN:
        sub_info->num_sub_part = 4;
        for (i = 0; i < sub_info->num_sub_part; i++)
        {
            sub_info->sub_x[i] = qw * (i == 0 || i == 2 ? 0 : 2) + x;
            sub_info->sub_y[i] = qh * (i == 0 || i == 1 ? 0 : 2) + y;
            sub_info->sub_w[i] = qw * 2;
            sub_info->sub_h[i] = qh * 2;
        }
        break;
    default:
        assert(0);
    }

    //derive sub_part scup（scu_position）
    for (i = 0; i < sub_info->num_sub_part; i++)
    {
        x_scu = PEL2SCU(sub_info->sub_x[i]);
        y_scu = PEL2SCU(sub_info->sub_y[i]);
        sub_info->sub_scup[i] = x_scu + y_scu * pic_width_in_scu;
    }
}

int  get_part_idx(PART_SIZE part_size, int x, int y, int w, int h)
{
    int idx = 0;
    int hw = w >> 1; //half width
    int hh = h >> 1; //half height
    int qw = w >> 2; //quarter width
    int qh = h >> 2; //quarter height

    assert(x < w);
    assert(y < h);
    assert(w >= 4);
    assert(h >= 4);

    if (part_size == SIZE_2Nx2N)
    {
        idx = 0;
    }
    else if (part_size == SIZE_NxN)
    {
        if (x < hw && y < hh)
            idx = 0;
        else if (x >= hw && y < hh)
            idx = 1;
        else if (x < hw && y >= hh)
            idx = 2;
        else
            idx = 3;
    }
    else if (part_size == SIZE_2NxhN)
    {
        if (y < qh)           // 1/4
            idx = 0;
        else if (y < hh)      // 2/4
            idx = 1;
        else if (y < qh + hh) // 3/4
            idx = 2;
        else
            idx = 3;
    }
    else if (part_size == SIZE_hNx2N)
    {
        if (x < qw)           // 1/4
            idx = 0;
        else if (x < hw)      // 2/4
            idx = 1;
        else if (x < qw + hw) // 3/4
            idx = 2;
        else
            idx = 3;
    }
    else if (part_size == SIZE_2NxnU)
    {
        if (y < qh)
            idx = 0;
        else
            idx = 1;
    }
    else if (part_size == SIZE_2NxnD)
    {
        if (y < qh + hh)
            idx = 0;
        else
            idx = 1;
    }
    else if (part_size == SIZE_nLx2N)
    {
        if (x < qw)
            idx = 0;
        else
            idx = 1;
    }
    else if (part_size == SIZE_nRx2N)
    {
        if (x < qw + hw)
            idx = 0;
        else
            idx = 1;
    }
    else
    {
        printf("\nError: part_size not expected");
        assert(0);
    }
    return idx;
}

void update_intra_info_map_scu(u32 *map_scu, s8 *map_ipm, int tb_x, int tb_y, int tb_w, int tb_h, int pic_width_in_scu, int ipm)
{
    int scu_x = PEL2SCU(tb_x);
    int scu_y = PEL2SCU(tb_y);
    int scu_w = PEL2SCU(tb_w);
    int scu_h = PEL2SCU(tb_h);

    map_scu = map_scu + scu_y * pic_width_in_scu + scu_x;
    map_ipm = map_ipm + scu_y * pic_width_in_scu + scu_x;

    for (int j = 0; j < scu_h; j++)
    {
        for (int i = 0; i < scu_w; i++)
        {
            MCU_SET_CODED_FLAG(map_scu[i]);
            MCU_SET_INTRA_FLAG(map_scu[i]);
            map_ipm[i] = ipm;
        }
        map_scu += pic_width_in_scu;
        map_ipm += pic_width_in_scu;
    }
}
#endif

int get_part_num(PART_SIZE size)
{
    switch (size)
    {
    case SIZE_2Nx2N:
        return 1;
    case SIZE_NxN:
        return 4;
    case SIZE_2NxnU:
        return 2;
    case SIZE_2NxnD:
        return 2;
    case SIZE_2NxhN:
        return 4;
    case SIZE_nLx2N:
        return 2;
    case SIZE_nRx2N:
        return 2;
    case SIZE_hNx2N:
        return 4;
    default:
        assert(0);
        return -1;
    }
}

int get_part_num_tb_in_pb(PART_SIZE pb_part_size, int pb_part_idx)
{
    switch (pb_part_size)
    {
    case SIZE_2NxnU:
    case SIZE_nLx2N:
        return pb_part_idx == 0 ? 1 : 3;
    case SIZE_2NxnD:
    case SIZE_nRx2N:
        return pb_part_idx == 0 ? 3 : 1;
    case SIZE_2NxhN:
    case SIZE_hNx2N:
    case SIZE_2Nx2N:
        return 1;
    default:
        assert(0);
        return -1;
    }
}
/*有点不太懂*/
int get_tb_idx_offset(PART_SIZE pb_part_size, int pb_part_idx)
{
    switch (pb_part_size)
    {
    case SIZE_2NxnU:
    case SIZE_nLx2N:
        assert(pb_part_idx <= 1);
        return pb_part_idx == 0 ? 0 : 1;
    case SIZE_2NxnD:
    case SIZE_nRx2N:
        assert(pb_part_idx <= 1);
        return pb_part_idx == 0 ? 0 : 3;
    case SIZE_2NxhN:
    case SIZE_hNx2N:
        assert(pb_part_idx <= 3);
        return pb_part_idx;
    case SIZE_2Nx2N:
        assert(pb_part_idx == 0);
        return 0;
    default:
        assert(0);
        return -1;
    }
}


//note: this function only works for DT intra
/**根据PU调整TU的尺寸*/
void get_tb_width_height_in_pb(int pb_w, int pb_h, PART_SIZE pb_part_size, int pb_part_idx, int *tb_w, int *tb_h)
{
    switch (pb_part_size)
    {
    case SIZE_2NxnU:
        *tb_w = pb_w;
        *tb_h = pb_part_idx == 0 ? pb_h : pb_h / 3;
        break;
    case SIZE_2NxnD:
        *tb_w = pb_w;
        *tb_h = pb_part_idx == 1 ? pb_h : pb_h / 3;
        break;
    case SIZE_nLx2N:
        *tb_w = pb_part_idx == 0 ? pb_w : pb_w / 3;
        *tb_h = pb_h;
        break;
    case SIZE_nRx2N:
        *tb_w = pb_part_idx == 1 ? pb_w : pb_w / 3;
        *tb_h = pb_h;
        break;
    case SIZE_2NxhN:
    case SIZE_hNx2N:
    case SIZE_2Nx2N:
        *tb_w = pb_w;
        *tb_h = pb_h;
        break;
    default:
        assert(0);
        break;
    }
}

//note: this function only works for DT intra
void get_tb_pos_in_pb(int pb_x, int pb_y, PART_SIZE pb_part_size, int tb_w, int tb_h, int tb_part_idx, int *tb_x, int *tb_y)
{
    switch (pb_part_size)
    {
    case SIZE_2NxnU:
    case SIZE_2NxnD:
    case SIZE_2NxhN:
        *tb_x = pb_x;
        *tb_y = pb_y + tb_part_idx * tb_h;
        break;
    case SIZE_nLx2N:
    case SIZE_nRx2N:
    case SIZE_hNx2N:
        *tb_x = pb_x + tb_part_idx * tb_w;
        *tb_y = pb_y;
        break;
    case SIZE_2Nx2N:
        *tb_x = pb_x;
        *tb_y = pb_y;
        break;
    default:
        assert(0);
        break;
    }
}
/* DT下，TB 的划分是每1/4_条_ 当作一个part划分的*/
PART_SIZE get_tb_part_size_by_pb(PART_SIZE pb_part, int pred_mode)
{
    PART_SIZE tb_part = 0;

    switch (pb_part)
    {
    case SIZE_2Nx2N:
        tb_part = pred_mode == MODE_INTRA ? SIZE_2Nx2N : SIZE_NxN;
        break;
    case SIZE_2NxnU:
    case SIZE_2NxnD:
    case SIZE_2NxhN:
        tb_part = SIZE_2NxhN;
        break;
    case SIZE_nLx2N:
    case SIZE_nRx2N:
    case SIZE_hNx2N:
        tb_part = SIZE_hNx2N;
        break;
    case SIZE_NxN:
        assert(0);
        tb_part = SIZE_NxN;
        break;
    default:
        assert(0);
        break;
    }

    return tb_part;
}

void get_tb_width_height_log2(int log2_w, int log2_h, PART_SIZE part, int *log2_tb_w, int *log2_tb_h)
{
    switch (part)
    {
    case SIZE_2Nx2N:
        break;
    case SIZE_NxN:
        log2_w--;
        log2_h--;
        break;
    case SIZE_2NxhN:
        log2_h -= 2;
        break;
    case SIZE_hNx2N:
        log2_w -= 2;
        break;
    default:
        assert(0);
        break;
    }

    *log2_tb_w = log2_w;
    *log2_tb_h = log2_h;
}

void get_tb_width_height(int w, int h, PART_SIZE part, int *tb_w, int *tb_h)
{
    switch (part)
    {
    case SIZE_2Nx2N:
        break;
    case SIZE_NxN:
        w >>= 1;
        h >>= 1;
        break;
    case SIZE_2NxhN:
        h >>= 2;
        break;
    case SIZE_hNx2N:
        w >>= 2;
        break;
    default:
        assert(0);
        break;
    }

    *tb_w = w;
    *tb_h = h;
}

void get_tb_start_pos(int w, int h, PART_SIZE part, int idx, int *pos_x, int *pos_y)
{
    int x = 0, y = 0;

    switch (part)
    {
    case SIZE_2Nx2N:
        x = y = 0;
        break;
    case SIZE_NxN:
        y = (idx / 2) * h / 2;
        x = (idx % 2) * w / 2;
        break;
    case SIZE_2NxhN:
        x = 0;
        y = idx * (h / 4);
        break;
    case SIZE_hNx2N:
        y = 0;
        x = idx * (w / 4);
        break;
    default:
        assert(0);
        break;
    }

    *pos_x = x;
    *pos_y = y;
}

int get_coef_offset_tb(int cu_x, int cu_y, int tb_x, int tb_y, int cu_w, int cu_h, int tb_part_size)
{
    int offset;
    switch (tb_part_size)
    {
    case SIZE_2Nx2N:
        offset = 0;
        break;
    case SIZE_NxN:
        if (tb_x == cu_x && tb_y == cu_y)
            offset = 0;
        else if (tb_x > cu_x && tb_y == cu_y)
            offset = (cu_w * cu_h) / 4;
        else if (tb_x == cu_x && tb_y > cu_y)
            offset = (cu_w * cu_h) / 2;
        else
            offset = (cu_w * cu_h * 3) / 4;
        break;
    case SIZE_2NxhN:
        offset = (tb_y - cu_y) * cu_w;
        break;
    case SIZE_hNx2N:
        offset = (tb_x - cu_x) * cu_h;
        break;
    default:
        assert(0);
        break;
    }

    return offset;
}

int is_tb_avaliable( COM_INFO info, COM_MODE *mod_info_curr )
{
    int log2_w = mod_info_curr->cu_width_log2;
    int log2_h = mod_info_curr->cu_height_log2; 
    PART_SIZE pb_part_size = mod_info_curr->pb_part;
    int pred_mode = mod_info_curr->cu_mode;

    //common intra: always infer TB part
    if (pred_mode == MODE_INTRA)
        return 0;

#if INTERPF
    if( mod_info_curr->inter_filter_flag )
    {
        return 0;
    }
#endif

    //inter or IBC: signal for DT and PBT cases
    int avaliable = 0;
    if (info.sqh.position_based_transform_enable_flag && (pb_part_size == SIZE_2Nx2N && abs(log2_w - log2_h) <= 1 && log2_w <= 5 && log2_w >= 3 && log2_h <= 5 && log2_h >= 3))
    {
        avaliable = 1;
    }
    return avaliable;
}

int is_cu_nz(int nz[MAX_NUM_TB][N_C])
{
    int cu_nz = 0;
    int i, j;

    for (i = 0; i < MAX_NUM_TB; i++)
    {
        for (j = 0; j < N_C; j++)
        {
            cu_nz |= nz[i][j];
        }
    }
    return cu_nz ? 1 : 0;
}

int is_cu_plane_nz(int nz[MAX_NUM_TB][N_C], int plane)
{
    int cu_nz = 0;
    int i;

    for (i = 0; i < MAX_NUM_TB; i++)
    {
        cu_nz |= nz[i][plane];
    }
    return cu_nz ? 1 : 0;
}

void cu_plane_nz_cpy(int dst[MAX_NUM_TB][N_C], int src[MAX_NUM_TB][N_C], int plane)
{
    int i;

    for (i = 0; i < MAX_NUM_TB; i++)
    {
        dst[i][plane] = src[i][plane];
    }
}

void cu_plane_nz_cln(int dst[MAX_NUM_TB][N_C], int plane)
{
    int i;

    for (i = 0; i < MAX_NUM_TB; i++)
    {
        dst[i][plane] = 0;
    }
}

int is_cu_nz_equ(int dst[MAX_NUM_TB][N_C], int src[MAX_NUM_TB][N_C])
{
    int i, j;
    int equ = 1;

    for (i = 0; i < N_C; i++)
    {
        for (j = 0; j < MAX_NUM_TB; j++)
        {
            if (dst[j][i] != src[j][i])
            {
                equ = 0;
                break;
            }
        }
    }
    return equ;
}

/*清楚内存内容（设0）*/
void cu_nz_cln(int dst[MAX_NUM_TB][N_C])
{
    memset(dst, 0, sizeof(int) * MAX_NUM_TB * N_C);
}

void check_set_tb_part(COM_MODE *mode)
{
    if (!is_cu_plane_nz(mode->num_nz, Y_C))
    {
        mode->tb_part = SIZE_2Nx2N;
    }
}

void check_tb_part(COM_MODE *mode)
{
    if (!is_cu_plane_nz(mode->num_nz, Y_C) && mode->tb_part != SIZE_2Nx2N)
    {
        com_assert(0);
    }
}

void copy_rec_y_to_pic(pel* src, int x, int y, int w, int h, int stride, COM_PIC *pic)
{
    pel* dst;
    int  j, s_pic;

    s_pic = pic->stride_luma;
    dst = pic->y + x + y * s_pic;
    for (j = 0; j < h; j++)
    {
        com_mcpy(dst, src, sizeof(pel) * w);
        src += stride;
        dst += s_pic;
    }
}

#if MODE_CONS
u8 com_constrain_pred_mode(int w, int h, SPLIT_MODE split, u8 slice_type)
{
    if (slice_type == SLICE_I)
    {
        return 0;
    }
    else
    {
        int s = w * h;
        if ((com_split_is_EQT(split) && s == 128) || ((com_split_is_BT(split) || split == SPLIT_QUAD) && s == 64))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}
#endif

#if CHROMA_NOT_SPLIT
u8 com_tree_split(int w, int h, SPLIT_MODE split, u8 slice_type)
{
    if (split == SPLIT_QUAD)
    {
        if (w == 8)
            return 1;
        else
            return 0;
    }
    else if (split == SPLIT_EQT_HOR)
    {
        if (h == 16 || w == 8)
            return 1;
        else
            return 0;
    }
    else if (split == SPLIT_EQT_VER)
    {
        if (w == 16 || h == 8)
            return 1;
        else
            return 0;
    }
    else if (split == SPLIT_BI_HOR)
    {
        if (h == 8)
            return 1;
        else
            return 0;
    }
    else if (split == SPLIT_BI_VER)
    {
        if (w == 8)
            return 1;
        else
            return 0;
    }
    else
    {
        return 0;
    }
}
#endif

#if LIBVC_ON
void init_libvcdata(LibVCData *libvc_data)
{
    libvc_data->bits_dependencyFile = 0;
    libvc_data->bits_libpic = 0;

    libvc_data->library_picture_enable_flag = 0;
#if IPPPCRR
#if LIB_PIC_UPDATE
    libvc_data->lib_pic_update = 0;
    libvc_data->update = 0;
    libvc_data->countRL = 0;
    libvc_data->encode_skip = 0;
    libvc_data->end_of_intra_period = 0;
#else
    libvc_data->first_pic_as_libpic = 0;
#endif
#endif
    libvc_data->is_libpic_processing = 0;
    libvc_data->is_libpic_prepared = 0;

    libvc_data->num_candidate_pic = 0;
    libvc_data->num_lib_pic = 0;
    libvc_data->num_RLpic = 0;

    libvc_data->num_libpic_outside = 0;

    for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
    {
        libvc_data->list_poc_of_candidate_pic[i] = -1;
        libvc_data->list_candidate_pic[i] = NULL;

        libvc_data->list_hist_feature_of_candidate_pic[i].num_component = 0;
        libvc_data->list_hist_feature_of_candidate_pic[i].num_of_hist_interval = 0;
        libvc_data->list_hist_feature_of_candidate_pic[i].length_of_interval = 0;
        for (int j = 0; j < MAX_NUM_COMPONENT; j++)
        {
            libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j] = NULL;
        }

        libvc_data->list_poc_of_RLpic[i] = -1;
        libvc_data->list_libidx_for_RLpic[i] = -1;

    }
    for (int i = 0; i < MAX_NUM_LIBPIC; i++)
    {
        libvc_data->list_poc_of_libpic[i] = -1;
        libvc_data->list_libpic_outside[i] = NULL;
        libvc_data->list_library_index_outside[i] = -1;
    }
}

void delete_libvcdata(LibVCData *libvc_data)
{
    /* remove allocated picture and picture store buffer */
    for (int i = 0; i < MAX_NUM_LIBPIC; i++)
    {
        if (libvc_data->list_libpic_outside[i])
        {
            com_picbuf_free(libvc_data->list_libpic_outside[i]);
            libvc_data->list_libpic_outside[i] = NULL;
        }
    }

}

int get_libidx(LibVCData *libvc_data, int cur_ptr)
{
    int lib_idx;
    lib_idx = -1;
    for (int j = 0; j < libvc_data->num_RLpic; j++)
    {
        if (cur_ptr == libvc_data->list_poc_of_RLpic[j])
        {
            lib_idx = libvc_data->list_libidx_for_RLpic[j];
            break;
        }
    }

    return lib_idx;
}
#endif

#if SBT
u8 com_sbt_allow( COM_MODE* mod_info_curr, int tool_sbt, int tree_status )
{
    int cuw = 1 << mod_info_curr->cu_width_log2;
    int cuh = 1 << mod_info_curr->cu_height_log2;
    int pred_mode = mod_info_curr->cu_mode;
    int min_size = 8;
    int max_size = 1 << MAX_TR_LOG2;
    u8  mode_hori, mode_vert, mode_hori_quad, mode_vert_quad;

    if( !tool_sbt || (pred_mode == MODE_INTRA || pred_mode == MODE_IBC) || (cuw > max_size || cuh > max_size) || tree_status == TREE_C )
    {
        mode_hori = mode_vert = mode_hori_quad = mode_vert_quad = 0;
    }
#if INTERPF
    else if( mod_info_curr->inter_filter_flag )
    {
        mode_hori = mode_vert = mode_hori_quad = mode_vert_quad = 0;
    }
#endif
    else
    {
        mode_vert = cuw >= min_size ? 1 : 0;
        mode_vert_quad = cuw >= min_size * 2 ? 1 : 0;
        mode_hori = cuh >= min_size ? 1 : 0;
        mode_hori_quad = cuh >= min_size * 2 ? 1 : 0;
    }
    return (mode_vert << 0) + (mode_hori << 1) + (mode_vert_quad << 2) + (mode_hori_quad << 3);
}

void get_sbt_tb_size( u8 sbt_info, int comp, int log2_cuw, int log2_cuh, int* log2_tuw, int* log2_tuh )
{
    u8 sbt_idx = get_sbt_idx( sbt_info );
    if( sbt_info == 0 || comp != Y_C )
    {
        *log2_tuw = log2_cuw;
        *log2_tuh = log2_cuh;
        return;
    }
    assert( sbt_idx <= 4 );
    if( is_sbt_horizontal( sbt_idx ) )
    {
        *log2_tuw = log2_cuw;
        *log2_tuh = is_sbt_quad_size( sbt_idx ) ? log2_cuh - 2 : log2_cuh - 1;
    }
    else
    {
        *log2_tuw = is_sbt_quad_size( sbt_idx ) ? log2_cuw - 2 : log2_cuw - 1;
        *log2_tuh = log2_cuh;
    }
}

void get_sbt_tb_pos_offset( u8 sbt_info, int comp, int log2_cuw, int log2_cuh, int* x_offset, int* y_offset )
{
    u8 sbt_idx = get_sbt_idx( sbt_info );
    u8 sbt_pos = get_sbt_pos( sbt_info );
    int cuw = 1 << log2_cuw;
    int cuh = 1 << log2_cuh;

    if( sbt_idx == 0 || comp != Y_C )
    {
        *x_offset = 0;
        *y_offset = 0;
        return;
    }

    if( is_sbt_horizontal( sbt_idx ) )
    {
        *x_offset = 0;
        *y_offset = sbt_pos == 0 ? 0 : cuh - (is_sbt_quad_size( sbt_idx ) ? cuh / 4 : cuh / 2);
    }
    else
    {
        *x_offset = sbt_pos == 0 ? 0 : cuw - (is_sbt_quad_size( sbt_idx ) ? cuw / 4 : cuw / 2);
        *y_offset = 0;
    }
}
#if SBT
void get_sbt_tr( u8 sbt_info, int log2_cuw, int log2_cuh, int* hor_trans, int* ver_trans )
{
    if( sbt_info == 0 || log2_cuw > 5 || log2_cuh > 5 )
    {
        *hor_trans = DCT2;
        *ver_trans = DCT2;
    }
    else
    {
        u8 sbt_idx = get_sbt_idx( sbt_info );
        u8 sbt_pos = get_sbt_pos( sbt_info );
        if( is_sbt_horizontal( sbt_idx ) )
        {
            *hor_trans = DST7;
            *ver_trans = sbt_pos == 0 ? DCT8 : DST7;
        }
        else
        {
            *ver_trans = DST7;
            *hor_trans = sbt_pos == 0 ? DCT8 : DST7;
        }
        //not use 32-point DST7/DCT8
        if( log2_cuw > 4 )
        {
          *hor_trans = DCT2;
        }
        if( log2_cuh > 4 )
        {
          *ver_trans = DCT2;
        }
    }
}
#endif
#endif

#if SRCC
void com_get_ctx_srxy_para(int ch_type, int width, int height, int *result_offset_x, int *result_offset_y, int *result_shift_x, int *result_shift_y)
{
    const int g_prefix_ctx[8] = { 0, 0, 0, 3, 6, 10, 15, 21 }; //for 1, 2, 4, 8, 16, 32,64,128 side length
    const int log2_w = CONV_LOG2(width);
    const int log2_h = CONV_LOG2(height);

    *result_offset_x = (ch_type != Y_C) ? 0 : g_prefix_ctx[log2_w];
    *result_offset_y = (ch_type != Y_C) ? 0 : g_prefix_ctx[log2_h];
    *result_shift_x  = (ch_type != Y_C) ? COM_CLIP3(0, 2, (width >> 3))  : ((log2_w + 1) >> 2);
    *result_shift_y  = (ch_type != Y_C) ? COM_CLIP3(0, 2, (height >> 3)) : ((log2_h + 1) >> 2);
}

int com_get_ctx_gt0_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info
)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int diag = pos_x + pos_y;
    int num_gt0 = 0;

    for (int i = 0; i < NUM_PREV_0VAL; i++)
    {
        if (prev_value[i] != 0)
        {
            num_gt0++;
        }
    }
    num_gt0 = COM_MIN(num_gt0, 3) + 1;
    if (ch_type == Y_C)
    {
        num_gt0 += com_get_ctx_offset(pos_info, is_intra, pos_x, pos_y, sr_x, sr_y);
    }
    else
    {
        num_gt0 += (pos_x == 0 && pos_y == 0) ? 0 : 4;
    }
    return num_gt0;
}

int com_get_ctx_gt1_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info 
)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int num_gt1 = 0;

    for (int i = 0; i < NUM_PREV_12VAL; i++)
    {
        if (COM_ABS16(prev_value[i]) > 1)
        {
            num_gt1++;
        }
    }
    num_gt1 = COM_MIN(num_gt1, 3) + 1;
    if (ch_type == Y_C)
    {
        num_gt1 += com_get_ctx_offset(pos_info, is_intra, pos_x, pos_y, sr_x, sr_y);
    }
    return num_gt1;
}

int com_get_ctx_gt2_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info
)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int num_gt2 = 0;

    for (int i = 0; i < NUM_PREV_12VAL; i++)
    {
        if (COM_ABS16(prev_value[i]) > 2)
        {
            num_gt2++;
        }
    }
    num_gt2 = COM_MIN(num_gt2, 3) + 1;
    if (ch_type == Y_C)
    {
        num_gt2 += com_get_ctx_offset(pos_info, is_intra, pos_x, pos_y, sr_x, sr_y);
    }
    return num_gt2;
}

void com_init_scan_sr(int *scan, int size_x, int size_y, int width, int scan_type)
{
    int x, y, l, pos, num_line;

    pos = 0;
    num_line = size_x + size_y - 1;
    if (scan_type == COEF_SCAN_ZIGZAG)
    {
        /* starting point */
        scan[pos] = 0;
        pos++;

        /* loop */
        for (l = 1; l < num_line; l++)
        {
            if (l % 2) /* decreasing loop */
            {
                x = COM_MIN(l, size_x - 1);
                y = COM_MAX(0, l - (size_x - 1));

                while (x >= 0 && y < size_y)
                {
                    scan[pos] = y * width + x;
                    pos++;
                    x--;
                    y++;
                }
            }
            else /* increasing loop */
            {
                y = COM_MIN(l, size_y - 1);
                x = COM_MAX(0, l - (size_y - 1));
                while (y >= 0 && x < size_x)
                {
                    scan[pos] = y * width + x;
                    pos++;
                    x++;
                    y--;
                }
            }
        }
    }
}

void com_init_pos_info(COM_POS_INFO *pos_info, int sr_x, int sr_y)
{
    COM_POS_INFO * pos = pos_info;
    int ax, bx, cx, ay, by, cy;

    ax = sr_x; bx = ax << 1; cx = ax + bx;
    ay = sr_y; by = ay << 1; cy = ay + by;
    pos->x.a = ax >> 2; pos->x.b = bx >> 2; pos->x.c = cx >> 2;
    pos->y.a = ay >> 2; pos->y.b = by >> 2; pos->y.c = cy >> 2;
}

int com_get_ctx_offset(COM_POS_INFO *pos_info, u8 is_intra, int pos_x, int pos_y, int sr_x, int sr_y)
{
    int offset = 0;

    if (!(pos_x == 0 && pos_y == 0))
    {
        if (is_intra)
        {
            int i = (pos_x <= pos_info->x.b) ? (pos_x <= pos_info->x.a ? 0 : 1) : (pos_x <= pos_info->x.c ? 2 : 3);
            int j = (pos_y <= pos_info->y.b) ? (pos_y <= pos_info->y.a ? 0 : 1) : (pos_y <= pos_info->y.c ? 2 : 3);
            int s = i + j;

            if (s <= 1)
            {
                offset = 4;
            }
            else if (s > 2)
            {
                offset = 8;
            }
            else
            {
                offset = 12;
            }
        }
        else
        {
            if (pos_x <= sr_x / 2 && pos_y <= sr_y / 2)
            {
                offset = 4;
            }
            else
            {
                offset = 8;
            }
        }
    }

    return offset;
}
#endif


#if BIO
s32  divide_tbl(s32 dividend, s32 divisor)
{
    u8 sign_dividend = dividend < 0;
    u8 sign_divisor = divisor < 0;
    s32 quotient = 0;
    s32 pos = -1;
    u32 divisor_32b;
    u32 dividend_32b;

    dividend = (sign_dividend) ? -dividend : dividend;
    divisor = (sign_divisor) ? -divisor : divisor;

    divisor_32b = divisor;
    dividend_32b = dividend;

    while (divisor_32b < dividend_32b)
    {
        divisor_32b <<= 1;
        pos++;
    }

    divisor_32b >>= 1;

    while (pos > -1)
    {
        if (dividend_32b >= divisor_32b)
        {
            quotient += (1 << pos);
            dividend_32b -= divisor_32b;
        }

        divisor_32b >>= 1;
        pos -= 1;
    }

    return (sign_dividend + sign_divisor == 1) ? -quotient : quotient;
}
#endif

#if FIMC
void com_cntmpm_update(COM_CNTMPM* cntMpm, const s8 currMode)
{
    cntMpm->freqT[currMode] += CNTMPM_BASE_VAL;

    u32 currCost  = cntMpm->freqT[currMode];
    u8  mode0 = cntMpm->modeT[0];
    u8  mode1 = cntMpm->modeT[1];
    u32 cost0 = cntMpm->freqT[mode0];
    u32 cost1 = cntMpm->freqT[mode1];

    if (currCost >= cost0)
    {
        cntMpm->modeT[0] = currMode;
        cntMpm->modeT[1] = mode0 != currMode ? mode0 : mode1;
    }
    else if (currCost >= cost1)
    {
        cntMpm->modeT[1] = currMode;
    }
}

void com_cntmpm_reset(COM_CNTMPM* cntMpm)
{
    memcpy(cntMpm, &g_cntMpmInitTable, sizeof(COM_CNTMPM));
}

void com_cntmpm_init(COM_CNTMPM* cntMpm)
{
    u8 initMode[CNTMPM_INIT_NUM] = { IPD_VER, IPD_HOR };
    u8 currOrder = 0;
    memset(cntMpm, 0, sizeof(COM_CNTMPM));

    for (int idx = 0; idx < CNTMPM_INIT_NUM; idx++)
    {
        u8 currMode = initMode[idx];
        cntMpm->freqT [currMode] = IPD_CNT + (CNTMPM_INIT_NUM - idx);
        cntMpm->modeT [currOrder] = currMode;
        currOrder++;
    }
    for (u8 currMode = 0; currMode < CNTMPM_TABLE_LENGTH; currMode++)
    {
        if (cntMpm->freqT[currMode] == 0)
        {
            cntMpm->freqT [currMode] = IPD_CNT - currMode;
        }
    }
    assert(currOrder == CNTMPM_INIT_NUM);
    assert(cntMpm->modeT[0] != cntMpm->modeT[1]);
    assert(cntMpm->freqT[cntMpm->modeT[0]] >= cntMpm->freqT[cntMpm->modeT[1]]);
}

void com_cntmpm_copy(COM_CNTMPM* cntMpm_dst, COM_CNTMPM* cntMpm_src)
{
    memcpy(cntMpm_dst, cntMpm_src, sizeof(COM_CNTMPM));
}
#endif

#if USE_SP
int get_sp_trav_index(const int cu_width_log2, const int cu_height_log2, int* p_trav_buff, int* p_raster_buff, int is_hor_scan)
{
    int blk_width = 1 << cu_width_log2;
    int blk_height = 1 << cu_height_log2;
    int total_values = 1 << (cu_width_log2 + cu_height_log2);
    int m_column = 0, m_line = 0;
    int rtn = 0;
    //advance line and column to the next position
    if (!is_hor_scan)
    {
        for (int scan_position = 0; scan_position < total_values; scan_position++)
        {
            rtn = (m_line * blk_width) + m_column;
            p_trav_buff[scan_position] = rtn;
            p_raster_buff[rtn] = scan_position;
            if ((m_column & 0x1) == 0)
            {
                if (m_line == (blk_height - 1))
                {
                    m_column++;
                    m_line = blk_height - 1;
                }
                else
                {
                    m_line++;
                }
            }
            else
            {
                if (m_line == 0)
                {
                    m_column++;
                    m_line = 0;
                }
                else
                {
                    m_line--;
                }
            }
        }
    }// VerScan
    else 
    {
        for (int scanPosition = 0; scanPosition < total_values; scanPosition++)
        {
            rtn = (m_line * blk_width) + m_column;
            p_trav_buff[scanPosition] = rtn;
            p_raster_buff[rtn] = scanPosition;
            if ((m_line & 0x1) == 0)
            {
                if (m_column == (blk_width - 1))
                {
                    m_line++;
                    m_column = blk_width - 1;
                }
                else
                {
                    m_column++;
                }
            }
            else
            {
                if (m_column == 0)
                {
                    m_line++;
                    m_column = 0;
                }
                else
                {
                    m_column--;
                }
            }
        }
    }// HorScan
    return 0;
}

static void init_msb_p1_idx_lut()
{
    g_msb_p1_idx[0] = 0; g_msb_p1_idx[1] = 1;
    int val = 2;
    for (int idx = 2; idx <= 8; idx++)
    {
        for (int i = val - 1; i >= 0; i--)
        {
            g_msb_p1_idx[val++] = idx;
        }
    }
}

unsigned char get_msb_p1_idx(int uiVal)
{
    unsigned char idx = 0;
    while (uiVal > 255)
    {
        uiVal >>= 8;
        idx += 8;
    }
    return idx + g_msb_p1_idx[uiVal];
}

int com_sp_init()
{
    // initialise scan orders
    for (int cu_height_log2 = MIN_CU_LOG2; cu_height_log2 < MAX_CU_LOG2 + 1; cu_height_log2++)
    {
        for (int cu_width_log2 = MIN_CU_LOG2; cu_width_log2 < MAX_CU_LOG2 + 1; cu_width_log2++)
        {
            const int cu_width = 1 << cu_width_log2;
            const int cu_height = 1 << cu_height_log2;
            const int cu_pix_num = cu_width * cu_height;
            for (int is_hor_scan = 0; is_hor_scan <= 1; is_hor_scan++)
            {
                com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2] = com_malloc(cu_pix_num * sizeof(int));
                com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2] = com_malloc(cu_pix_num * sizeof(int));
                int* p_trav_buff = com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
                int* p_raster_buff = com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
                get_sp_trav_index(cu_width_log2, cu_height_log2, p_trav_buff, p_raster_buff, is_hor_scan);
            }
        }
    }
    init_msb_p1_idx_lut();
    return COM_OK;
}

int com_sp_delete()
{
    for (int cu_width_log2 = MIN_CU_LOG2; cu_width_log2 < MAX_CU_LOG2 + 1; cu_width_log2++)
    {
        for (int cu_height_log2 = MIN_CU_LOG2; cu_height_log2 < MAX_CU_LOG2 + 1; cu_height_log2++)
        {
            for (int is_hor_scan = 0; is_hor_scan <= 1; is_hor_scan++) 
            {
                com_mfree(com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2]);
                com_mfree(com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2]);
            }
        }
    }
    return COM_OK;
}
#endif

#if PMC
int com_is_mcpm(s8 ipm_c)
{
    int bMcpm = ipm_c == IPD_MCPM_C || ipm_c == IPD_MCPM_L_C || ipm_c == IPD_MCPM_T_C;
    return bMcpm;
}
#endif