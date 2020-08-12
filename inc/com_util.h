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

#ifndef _COM_UTIL_H_
#define _COM_UTIL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "com_def.h"
#include <stdlib.h>

/*! macro to determine maximum */
#define COM_MAX(a,b)                   (((a) > (b)) ? (a) : (b))

/*! macro to determine minimum */
#define COM_MIN(a,b)                   (((a) < (b)) ? (a) : (b))

/*! macro to absolute a value */
#define COM_ABS(a)                     abs(a)

/*! macro to absolute a 64-bit value */
#define COM_ABS64(a)                   (((a)^((a)>>63)) - ((a)>>63))

/*! macro to absolute a 32-bit value */
#define COM_ABS32(a)                   (((a)^((a)>>31)) - ((a)>>31))

/*! macro to absolute a 16-bit value */
#define COM_ABS16(a)                   (((a)^((a)>>15)) - ((a)>>15))

/*! macro to clipping within min and max */
#define COM_CLIP3(min_x, max_x, value)   COM_MAX((min_x), COM_MIN((max_x), (value)))

/*! macro to clipping within min and max */
#define COM_CLIP(n,min,max)            (((n)>(max))? (max) : (((n)<(min))? (min) : (n)))

#define COM_SIGN(x)                    (((x) < 0) ? -1 : 1)

/*! macro to get a sign from a 16-bit value.\n
operation: if(val < 0) return 1, else return 0 */
#define COM_SIGN_GET(val)              ((val<0)? 1: 0)

/*! macro to set sign into a value.\n
operation: if(sign == 0) return val, else if(sign == 1) return -val */
#define COM_SIGN_SET(val, sign)        ((sign)? -val : val)

/*! macro to get a sign from a 16-bit value.\n
operation: if(val < 0) return 1, else return 0 */
#define COM_SIGN_GET16(val)            (((val)>>15) & 1)

/*! macro to set sign into a 16-bit value.\n
operation: if(sign == 0) return val, else if(sign == 1) return -val */
#define COM_SIGN_SET16(val, sign)      (((val) ^ ((s16)((sign)<<15)>>15)) + (sign))

#define COM_ALIGN(val, align)          ((((val) + (align) - 1) / (align)) * (align))

#define CONV_LOG2(v)                    (com_tbl_log2[v])

#if !MOTION_DEF
typedef struct _COM_MOTION
{
    s16 mv[REFP_NUM][MV_D];
    s8 ref_idx[REFP_NUM];
} COM_MOTION;
#endif

#if IBC_BVP
typedef struct _COM_BLOCK_MOTION
{
    s16 mv[MV_D];
    int x, y, w, h, cnt;
#if SP_PRED
    int len;
#endif
} COM_BLOCK_MOTION;
#endif

#define SAME_MV(MV0, MV1) ((MV0[MV_X] == MV1[MV_X]) && (MV0[MV_Y] == MV1[MV_Y]))
void copy_mv(s16 dst[MV_D], const s16 src[MV_D]);
void copy_motion_table(COM_MOTION *motion_dst, s8 *cnt_cands_dst, const COM_MOTION *motion_src, const s8 cnt_cands_src);
#if EXT_AMVR_HMVP
int same_motion(COM_MOTION motion1, COM_MOTION motion2);
#endif

u16 com_get_avail_intra(int x_scu, int y_scu, int pic_width_in_scu, int scup, u32 *map_scu);

COM_PIC * com_pic_alloc(PICBUF_ALLOCATOR * pa, int * ret);
void com_pic_free(PICBUF_ALLOCATOR *pa, COM_PIC *pic);
COM_PIC* com_picbuf_alloc(int w, int h, int pad_l, int pad_c, int *err);
void com_picbuf_free(COM_PIC *pic);
void com_picbuf_expand(COM_PIC *pic, int exp_l, int exp_c);

void check_mvp_motion_availability(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, int neb_addr[NUM_AVS2_SPATIAL_MV], int valid_flag[NUM_AVS2_SPATIAL_MV], int lidx);
void check_umve_motion_availability(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, int neb_addr[NUM_AVS2_SPATIAL_MV], int valid_flag[NUM_AVS2_SPATIAL_MV]);

void update_skip_candidates(COM_MOTION motion_cands[ALLOWED_HMVP_NUM], s8 *cands_num, const int max_hmvp_num, s16 mv_new[REFP_NUM][MV_D], s8 refi_new[REFP_NUM]);
void fill_skip_candidates(COM_MOTION motion_cands[ALLOWED_HMVP_NUM], s8 *num_cands, const int num_hmvp_cands, 
#if MVAP
    const int num_mvap_cands,
#endif
    s16 mv_new[REFP_NUM][MV_D], s8 refi_new[REFP_NUM], int bRemDuplicate);
void get_hmvp_skip_cands(const COM_MOTION motion_cands[ALLOWED_HMVP_NUM], const u8 num_cands, s16(*skip_mvs)[REFP_NUM][MV_D], s8(*skip_refi)[REFP_NUM]);

void derive_MHBskip_spatial_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 skip_pmv[PRED_DIR_NUM][REFP_NUM][MV_D], s8 skip_refi[PRED_DIR_NUM][REFP_NUM]);
#if AWP
#if !AWP_UNIMV_SIMP
BOOL com_check_same_uni_motion  (s16 uni_cands0[REFP_NUM][MV_D], s8 uni_ref0[REFP_NUM], s16 uni_cands1[REFP_NUM][MV_D], s8 uni_ref1[REFP_NUM], s8 RefList);
#endif
void com_set_uni_cand           (s16 candidate[REFP_NUM][MV_D], s8 candRefIdx[REFP_NUM], s16 awp_uni_cands[REFP_NUM][MV_D], s8 awp_uni_refi[REFP_NUM], s8 RefList);
void com_set_awp_mv_para        (COM_MODE* mod_info_curr, s16 awp_uni_cands[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM]);
#if AWP_UNIMV_SIMP
u8   com_derive_awp_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM],
                                 s16 awp_uni_cand[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM], int num_refp[REFP_NUM]);
#else
u8   com_derive_awp_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM],
                                 s16 awp_uni_cand[AWP_MV_LIST_LENGTH][REFP_NUM][MV_D], s8 awp_uni_refi[AWP_MV_LIST_LENGTH][REFP_NUM]);
#endif
#endif
#if AWP_MVR
void com_set_awp_mvr_mv_para(COM_MODE* mod_info_curr, s16 awp_final_mv0[REFP_NUM][MV_D], s8 awp_final_refi0[REFP_NUM], s16 awp_final_mv1[REFP_NUM][MV_D], s8 awp_final_refi1[REFP_NUM]);
#endif
#if MVAP
int same_neighbor_motion(COM_MOTION motion1, COM_MOTION motion2);
int have_diff_motion(s32 cu_width_in_scu, s32 cu_height_in_scu, s32  list_idx, s32 diff_flag, s32 prunning_stride[ALLOWED_MVAP_NUM]);
void fill_neighbor_motions(u8 ibc_flag, s32 scup, s32 cu_width, s32 cu_height, s32 pic_width_in_scu, s32 h_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4]);
void copy_neighbor_motion(s32 neighbor_idx, s32 neb_addr, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4]);
void get_valid_mvap_mode(s32 num_cands, s32 scup, s32 pic_width_in_scu, s32 cu_width, s32 cu_height, s32 *valid_mvap_num, s32 *valid_mvap_index, COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], s8 refi_cands[MAX_SKIP_NUM][REFP_NUM]);
void set_mvap_mvfield(s32 cu_width_in_scu, s32 cu_height_in_scu, s32 mvap_idx, COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], COM_MOTION tmp_cu_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)]);
void derive_mvap_motions(u8 ibc_flag, s32 num_cands, s32 scup, s32 cu_width, s32 cu_height, s32 pic_width_in_scu, s32 h_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4], s32 *valid_mvap_num, s32 *valid_mvap_index, s8 refi_cands[MAX_SKIP_NUM][REFP_NUM]);
void copy_available_motions(u8 ibc_flag, int scup, int cu_width, int cu_height, int pic_width_in_scu, int h_scu, u32* map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION neighbor_motions[MAX_CU_SIZE + 4]);
#endif

#if !LIBVC_BLOCKDISTANCE_BY_LIBPTR
void derive_umve_base_motions(COM_REFP(*refp)[REFP_NUM], COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM], s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM]);
#else
void derive_umve_base_motions(COM_INFO *info, COM_MODE* mod_info_curr, COM_MAP *pic_map, s16 t_mv[REFP_NUM][MV_D], s8 t_refi[REFP_NUM], s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM]);
#endif
#if UMVE_ENH 
void derive_umve_final_motions(int umve_idx, COM_REFP(*refp)[REFP_NUM], int cur_poc, s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM], s16 umve_final_pmv[UMVE_BASE_NUM*UMVE_MAX_REFINE_NUM_SEC_SET][REFP_NUM][MV_D], s8 umve_final_refi[UMVE_BASE_NUM*UMVE_MAX_REFINE_NUM_SEC_SET][REFP_NUM], BOOL isUMVESecSet);
#else
void derive_umve_final_motions(int umve_idx, COM_REFP(*refp)[REFP_NUM], int cur_poc, s16 umve_base_pmv[UMVE_BASE_NUM][REFP_NUM][MV_D], s8 umve_base_refi[UMVE_BASE_NUM][REFP_NUM], s16 umve_final_pmv[UMVE_BASE_NUM*UMVE_MAX_REFINE_NUM][REFP_NUM][MV_D], s8 umve_final_refi[UMVE_BASE_NUM*UMVE_MAX_REFINE_NUM][REFP_NUM]);
#endif
#if AFFINE_UMVE
void derive_affine_umve_final_motion(s8 refi[REFP_NUM], int affine_umve_idx, s32 affine_mv_offset[REFP_NUM][MV_D]);
#endif
#if AWP_MVR
void derive_awp_mvr_final_motion(int awp_mvr_idx, COM_REFP(*refp)[REFP_NUM], s8 refi[REFP_NUM], s32 awp_mvr_offset[REFP_NUM][MV_D]);
#endif
void com_get_mvp_default(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int ptr_cur, int lidx, s8 cur_refi, u8 amvr_idx, s16 mvp[MV_D]);
void com_derive_mvp(COM_INFO info, COM_MODE *mod_info_curr, int ptr, int ref_list, int ref_idx, int cnt_hmvp_cands, COM_MOTION *motion_cands, COM_MAP map, COM_REFP(*refp)[REFP_NUM], int mvr_idx, s16 mvp[MV_D]);

#if IBC_BVP
int same_block_motion(COM_BLOCK_MOTION motion1, COM_BLOCK_MOTION motion2);
void copy_block_motion_table(COM_BLOCK_MOTION *motion_dst, s8 *cnt_cands_dst, const COM_BLOCK_MOTION *motion_src, const s8 cnt_cands_src);
void update_ibc_skip_candidates(COM_BLOCK_MOTION block_motion_cands[ALLOWED_HBVP_NUM], s8 *num_cands, const int max_hbvp_num, s16 mv_new[MV_D], int x, int y, int w, int h
#if SP_PRED
    , int len
#endif
);
void com_derive_bvp_list(u8 allowed_hbvp_num, COM_MODE *mod_info_curr, COM_BLOCK_MOTION* block_motion_cands, s8 cnt_hbvp_cands, s16(*bvp_cands)[MV_D], int *num_cands_all);
#if IBC_BVP_CHECK
void fill_bvp_list(s16(*bv_list)[MV_D], u8 *num_cands, const s16 bv_new[MV_D], int num_check);
#else
void fill_bvp_list(s16(*bv_list)[MV_D], u8 *num_cands, const s16 bv_new[MV_D], int bRemDuplicate);
#endif
#endif
#if USE_SP
int check_sp_offset(s16 offset, s16 offset_y, s8 n_recent_num, COM_MOTION n_recent_offset[SP_RECENT_CANDS]);
void com_derive_sp_offset(s8 cnt_sp_recent_cands, COM_MOTION *motion_cands, int sp_n_idx, s16 mvp[MV_D]);
void update_sp_recent_cands(COM_MOTION *sps_cands, s8 *num_cands, COM_SP_INFO *p_SPinfo, u16 str_num, u8 dir);
#endif

enum
{
    SPLIT_MAX_PART_COUNT = 4
};

typedef struct _COM_SPLIT_STRUCT
{
    int       part_count;
    int       cud;
    int       width[SPLIT_MAX_PART_COUNT];
    int       height[SPLIT_MAX_PART_COUNT];
    int       log_cuw[SPLIT_MAX_PART_COUNT];
    int       log_cuh[SPLIT_MAX_PART_COUNT];
    int       x_pos[SPLIT_MAX_PART_COUNT];
    int       y_pos[SPLIT_MAX_PART_COUNT];
    int       cup[SPLIT_MAX_PART_COUNT];
} COM_SPLIT_STRUCT;

//! Count of partitions, correspond to split_mode
int com_split_part_count(int split_mode);
//! Get partition size
int com_split_get_part_size(int split_mode, int part_num, int length);
//! Get partition size log
int com_split_get_part_size_idx(int split_mode, int part_num, int length_idx);
//! Get partition split structure
void com_split_get_part_structure(int split_mode, int x0, int y0, int cu_width, int cu_height, int cup, int cud, int log2_culine, COM_SPLIT_STRUCT* split_struct);
//! Get array of split modes tried sequentially in RDO
void com_split_get_split_rdo_order(int cu_width, int cu_height, SPLIT_MODE splits[MAX_SPLIT_NUM]);
//! Get split direction. Quad will return vertical direction.
SPLIT_DIR com_split_get_direction(SPLIT_MODE mode);
#if EQT
//! Is mode triple tree?
int  com_split_is_EQT(SPLIT_MODE mode);
#endif
//! Is mode BT?
int  com_split_is_BT(SPLIT_MODE mode);
//! Check that mode is vertical
int com_split_is_vertical(SPLIT_MODE mode);
//! Check that mode is horizontal
int com_split_is_horizontal(SPLIT_MODE mode);

int get_colocal_scup(int scup, int pic_width_in_scu, int pic_height_in_scu);

void get_col_mv(COM_REFP refp[REFP_NUM], u32 ptr, int scup, s16 mvp[REFP_NUM][MV_D]);
void get_col_mv_from_list0(COM_REFP refp[REFP_NUM], u32 ptr, int scup, s16 mvp[REFP_NUM][MV_D]);
#if SUB_TMVP
void get_col_mv_ext(COM_REFP refp[REFP_NUM], s32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D], s8 refi[REFP_NUM]);
void get_col_mv_from_list0_ext(COM_REFP refp[REFP_NUM], s32 ptr_curr, int scup_co, s16 mvp[REFP_NUM][MV_D], s8 refi[REFP_NUM]);
#endif
int com_scan_tbl_init();
int com_scan_tbl_delete();
int com_get_split_mode(s8* split_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s
                       , s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
int com_set_split_mode(s8  split_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s
                       , s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);

#if MODE_CONS
u8   com_get_cons_pred_mode(int cud, int cup, int cu_width, int cu_height, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
void com_set_cons_pred_mode(u8 cons_pred_mode, int cud, int cup, int cu_width, int cu_height, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
#endif

void com_mv_rounding_s32(s32 hor, s32 ver, s32 * rounded_hor, s32 * rounded_ver, int right_shift, int left_shift);
void com_mv_rounding_s16(s32 hor, s32 ver, s16 * rounded_hor, s16 * rounded_ver, int right_shift, int left_shift);

void com_get_affine_mvp_scaling(COM_INFO *info, COM_MODE * mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int ptr, int lidx, \
    CPMV mvp[VER_NUM][MV_D], int vertex_num
#if BD_AFFINE_AMVR
    , u8 curr_mvr
#endif
);

int com_get_affine_memory_access(CPMV mv[VER_NUM][MV_D], int cu_width, int cu_height);

void com_set_affine_mvf(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map);
#if AWP
void com_set_awp_mvf(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map);
#endif
int com_get_affine_merge_candidate(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map,
    s8 mrg_list_refi[AFF_MAX_NUM_MRG][REFP_NUM], CPMV mrg_list_cpmv[AFF_MAX_NUM_MRG][REFP_NUM][VER_NUM][MV_D], int mrg_list_cp_num[AFF_MAX_NUM_MRG], int ptr);
#if ETMVP
void derive_first_stage_motion(u8 ibc_flag, int scup, int cu_width, int cu_height, int pic_width_in_scu, u32 *map_scu, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], COM_MOTION *first_stage_motion);
void adjust_first_stage_motion(s32 frameType, int scup, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], COM_MOTION *first_stage_motion);
void derive_ref_block_position(s32 frameType, int pic_width_in_scu, int pic_height_in_scu, s32 pic_width, s32 pic_height, int cu_width, int cu_height, s32 x_ctb_pos, s32 y_ctb_pos, s32 x_pos, s32 y_pos, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], s32 *ref_block_x, s32 *ref_block_y, COM_MOTION *first_stage_motion);
void set_etmvp_mvfield(s32 frameType, s32 cur_ptr, s32 ref_x, s32 ref_y, s32 cu_width, s32 cu_height, s32 pic_width, s32 pic_height, s32 x_ctb_pos, s32 y_ctb_pos, int pic_width_in_scu, int pic_height_in_scu, COM_REFP(*refp)[REFP_NUM], COM_MOTION *etmvp_mvfield, COM_MOTION first_stage_motion);
void drive_scaled_tmporal_motion(int scup, s32 ref_dir, COM_REFP(*refp)[REFP_NUM], s32 cur_ptr, COM_MOTION *tmporal_motion);
int get_valid_etmvp_motion(s32 frameType, s32 cur_ptr, s32 cu_width, s32 cu_height, s32 x_ctb_pos, s32 y_ctb_pos, s32 pic_width, s32 pic_height, s32 pic_width_in_scu, s32 pic_height_in_scu, s32 ref_x, s32 ref_y, COM_REFP(*refp)[REFP_NUM], s32 *valid_etmvp_offset, COM_MOTION first_stage_motion);
void derive_scaled_base_motion(s32 frameType, s32 cur_ptr, COM_REFP(*refp)[REFP_NUM], COM_MOTION *base_motion, COM_MOTION *first_stage_motion);
#endif

/* MD5 structure */
typedef struct _COM_MD5
{
    u32     h[4]; /* hash state ABCD */
    u8      msg[64]; /*input buffer (chunk message) */
    u32     bits[2]; /* number of bits, modulo 2^64 (lsb first)*/
} COM_MD5;

/* MD5 Functions */
void com_md5_init(COM_MD5 * md5);
void com_md5_update(COM_MD5 * md5, void * buf, u32 len);
void com_md5_update_16(COM_MD5 * md5, void * buf, u32 len);
void com_md5_finish(COM_MD5 * md5, u8 digest[16]);
int com_md5_imgb(COM_IMGB * imgb, u8 digest[16]);

int com_picbuf_signature(COM_PIC * pic, u8 * md5_out);

int com_atomic_inc(volatile int * pcnt);
int com_atomic_dec(volatile int * pcnt);

void com_check_split_mode(COM_SQH* sqh, int *split_allow, int cu_width_log2, int cu_height_log2, int boundary, int boundary_b, int boundary_r, int log2_max_cuwh, int id
                          , const int parent_split, int qt_depth, int bet_depth, int slice_type);

#ifdef __cplusplus
}
#endif

void com_sbac_ctx_init(COM_SBAC_CTX *sbac_ctx);

#if DT_SYNTAX
int  com_dt_allow(int cu_w, int cu_h, int pred_mode, int max_dt_size);
#endif


#if TB_SPLIT_EXT
void init_tb_part(COM_MODE *mod_info_curr);
void init_pb_part(COM_MODE *mod_info_curr);
void set_pb_part(COM_MODE *mod_info_curr, int part_size);
void set_tb_part(COM_MODE *mod_info_curr, int part_size);
void get_part_info(int pic_width_in_scu, int x, int y, int w, int h, int part_size, COM_PART_INFO* sub_info);
int  get_part_idx(PART_SIZE part_size, int x, int y, int w, int h);
void update_intra_info_map_scu(u32 *map_scu, s8 *map_ipm, int tb_x, int tb_y, int tb_w, int tb_h, int pic_width_in_scu, int ipm);
#endif

int  com_ctx_tb_split(int pb_part_size);
int  get_part_num(PART_SIZE size);
int  get_part_num_tb_in_pb(PART_SIZE pb_part_size, int pb_part_idx);
int  get_tb_idx_offset(PART_SIZE pb_part_size, int pb_part_idx);
void get_tb_width_height_in_pb(int pb_w, int pb_h, PART_SIZE pb_part_size, int pb_part_idx, int *tb_w, int *tb_h);
void get_tb_pos_in_pb(int pb_x, int pb_y, PART_SIZE pb_part_size, int tb_w, int tb_h, int tb_part_idx, int *tb_x, int *tb_y);
int get_coef_offset_tb(int cu_x, int cu_y, int tb_x, int tb_y, int cu_w, int cu_h, int tb_part_size);
PART_SIZE get_tb_part_size_by_pb(PART_SIZE pb_part, int pred_mode);
void get_tb_width_height_log2(int log2_w, int log2_h, PART_SIZE part, int *log2_tb_w, int *log2_tb_h);
void get_tb_width_height(int w, int h, PART_SIZE part, int *tb_w, int *tb_h);
void get_tb_start_pos(int w, int h, PART_SIZE part, int idx, int *pos_x, int *pos_y);
int  is_tb_avaliable(COM_INFO info, COM_MODE *mod_info_curr);
int  is_cu_nz(int nz[MAX_NUM_TB][N_C]);
int  is_cu_plane_nz(int nz[MAX_NUM_TB][N_C], int plane);
void cu_plane_nz_cpy(int dst[MAX_NUM_TB][N_C], int src[MAX_NUM_TB][N_C], int plane);
void cu_plane_nz_cln(int dst[MAX_NUM_TB][N_C], int plane);
int is_cu_nz_equ(int dst[MAX_NUM_TB][N_C], int src[MAX_NUM_TB][N_C]);
void cu_nz_cln(int dst[MAX_NUM_TB][N_C]);
void check_set_tb_part(COM_MODE *mode);
void check_tb_part(COM_MODE *mode);
void copy_rec_y_to_pic(pel* src, int x, int y, int w, int h, int stride, COM_PIC *pic);

#if MODE_CONS
u8 com_constrain_pred_mode(int w, int h, SPLIT_MODE split, u8 slice_type);
#endif
#if CHROMA_NOT_SPLIT
u8 com_tree_split(int w, int h, SPLIT_MODE split, u8 slice_type);
#endif

#if BIO
s32  divide_tbl(s32 dividend, s32 divisor);
#endif

#if LIBVC_ON
void init_libvcdata(LibVCData *libvc_data);
void delete_libvcdata(LibVCData *libvc_data);
int  get_libidx(LibVCData *libvc_data, int cur_ptr);
#endif

#if SBT
u8   com_sbt_allow( COM_MODE* mod_info_curr, int tool_sbt, int tree_status );
void get_sbt_tb_size( u8 sbt_info, int comp, int log2_cuw, int log2_cuh, int* log2_tuw, int* log2_tuh );
void get_sbt_tb_pos_offset( u8 sbt_info, int comp, int log2_cuw, int log2_cuh, int* x_offset, int* y_offset );
void get_sbt_tr( u8 sbt_info, int log2_cuw, int log2_cuh, int* hor_trans, int* ver_trans );
#endif

#if SRCC
void com_init_scan_sr(int *scan, int size_x, int size_y, int width, int scan_type);
void com_init_pos_info(COM_POS_INFO *pos_info, int sr_x, int sr_y);
int  com_get_ctx_offset(COM_POS_INFO *pos_info, u8 is_intra, int pos_x, int pos_y, int sr_x, int sr_y);
void com_get_ctx_srxy_para(int ch_type, int width, int height, int *result_offset_x, int *result_offset_y, int *result_shift_x, int *result_shift_y);
int com_get_ctx_gt0_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info
);
int com_get_ctx_gt1_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info
);
int com_get_ctx_gt2_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y, int *prev_value
    , u8 is_intra, COM_POS_INFO *pos_info
);
#endif

#if FIMC
void com_cntmpm_update(COM_CNTMPM *cntMpm, const s8 currMode);
void com_cntmpm_reset (COM_CNTMPM* cntMpm);
void com_cntmpm_init  (COM_CNTMPM* cntMpm);
void com_cntmpm_copy  (COM_CNTMPM* cntMpm_dst, COM_CNTMPM* cntMpm_src);
#endif

#if PMC
int com_is_mcpm(s8 ipm_c);
#endif

#if USE_SP
int get_sp_trav_index(const int cu_width_log2, const int cu_height_log2, int* p_trav_buff, int* p_raster_buff, int is_hor_scan);
int com_sp_init();
int com_sp_delete();
unsigned char get_msb_p1_idx(int uiVal);
#endif

#endif /* _COM_UTIL_H_ */
