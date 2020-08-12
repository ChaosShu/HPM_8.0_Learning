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

#ifndef _COM_MC_H_
#define _COM_MC_H_

#ifdef __cplusplus

extern "C"
{
#endif

#if DMVR
typedef void (*COM_MC_L) (pel* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel* pred, int w, int h, int bit_depth, int bHpFilter, int bDMVR);
#else
typedef void (*COM_MC_L) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter);
#endif

typedef void(*COM_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter
#if USE_IBC
    , int bIBC
#endif
#if DMVR
    , int bDMVR
#endif
    );

extern COM_MC_L com_tbl_mc_l[2][2];
extern COM_MC_C com_tbl_mc_c[2][2];

#if DMVR
typedef void(*COM_DMVR_MC_L) (pel* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel* pred, int w, int h, int bit_depth, int bHpFilter, int bDMVR);
typedef void(*COM_DMVR_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int bHpFilter);
    
extern COM_DMVR_MC_L ifvc_tbl_dmvr_mc_l[2][2];
extern COM_DMVR_MC_C ifvc_tbl_dmvr_mc_c[2][2];
#endif

#if DMVR
#define com_mc_l_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0xF)?1:0][(ori_mv_y & 0xF)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)
#define com_mc_l(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0x3)?1:0][(ori_mv_y & 0x3)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#else
#define com_mc_l_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0xF)?1:0][(ori_mv_y & 0xF)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1)
#define com_mc_l(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0x3)?1:0][(ori_mv_y & 0x3)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0)
#endif

#if USE_IBC
#if DMVR
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0, 0)

#define com_mc_c_ibc(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(gmv_x & 0x7)?1:0][(gmv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1, 0)

#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0, 0)
#else
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)

#define com_mc_c_ibc(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(gmv_x & 0x7)?1:0][(gmv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)

#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#endif
#else
#if DMVR
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)
#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#else
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1)
#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0)
#endif
#endif

#if DMVR
#define com_dmvr_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[((gmv_x) | ((gmv_x)>>1)) & 0x1])\
        [((gmv_y) | ((gmv_y)>>1)) & 0x1]\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)

#if USE_IBC
#define com_dmvr_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2)) & 0x1]\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0, 1)
#else
#define com_dmvr_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_dmvr_mc_c[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2)) & 0x1]\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)
#endif
#endif

/*****************************************************************************
* mc DMVR structure
*****************************************************************************/
#if DMVR
typedef struct _COM_DMVR
{
    int poc_c;
    pel *dmvr_current_template; 
    pel (*dmvr_ref_pred_interpolated)[(MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))];
    BOOL apply_DMVR;
    pel (*dmvr_padding_buf)[N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE];
} COM_DMVR;
#endif

void com_mc(int x, int y, int w, int h, int pred_stride, pel pred_buf[N_C][MAX_CU_DIM], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], CHANNEL_TYPE channel, int bit_depth
#if DMVR
            , COM_DMVR * dmvr
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
);

#if AWP
#if AWP_SCC
#if REDUCE_AWP_BUFFER
void com_calculate_awp_weight(pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx, int refine_flag);
#else
void com_calculate_awp_weight(pel weight0[N_C][MAX_CU_DIM], pel weight1[N_C][MAX_CU_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx, int refine_flag);
#endif
#else
void com_calculate_awp_weight(pel weight0[N_C][MAX_CU_DIM], pel weight1[N_C][MAX_CU_DIM], int compIdx, int nCbW, int nCbH, int modeIdx, int angleIdx, int subangleIdx);
#endif
void com_calculate_awp_para  (int AwpModeIdx, int blkWidth, int blkHeight, int *stepIdx, int *angleIdx, int *subangleIdx);
#if REDUCE_AWP_BUFFER
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM]);
void com_derive_awp_pred(COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM]);
#else
void com_derive_awp_weight   (COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_CU_DIM],   pel weight1[N_C][MAX_CU_DIM]);
void com_derive_awp_pred     (COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_CU_DIM], pel weight1[MAX_CU_DIM]);
#endif
#endif

#if MVAP
void com_mvap_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if SUB_TMVP 
void com_sbTmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, s32 sub_blk_width, s32 sub_blk_height, COM_MOTION* sbTmvp, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if ETMVP
void com_etmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if USE_IBC
void com_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], COM_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM], CHANNEL_TYPE channel, int bit_depth);
#endif

void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h,
             s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16(*mv_t)[MV_D]);

#if AWP
void com_awp_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth);
#endif
void com_affine_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int bit_depth);
void com_affine_mc_l(int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth);
void com_affine_mc_lc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, pel pred[N_C][MAX_CU_DIM], int sub_w, int sub_h, int lidx, int bit_depth);

#if SIMD_MC
#if AWP
void weight_average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, s16 *weight0, s16 *weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht);
#endif
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _COM_MC_H_ */
