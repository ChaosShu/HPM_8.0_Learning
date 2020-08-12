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

#ifndef _ENC_ECO_H_
#define _ENC_ECO_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "enc_def.h"

#define GET_SBAC_ENC(bs)   ((ENC_SBAC *)(bs)->pdata[1])

int enc_eco_cnkh(COM_BSW * bs, COM_CNKH * cnkh);
int enc_eco_sqh(ENC_CTX *ctx, COM_BSW * bs, COM_SQH * sqh);
int enc_eco_pic_header(COM_BSW * bs, COM_PIC_HEADER * sh, COM_SQH * sqh);
int enc_eco_patch_header(COM_BSW * bs, COM_SQH *sqh, COM_PIC_HEADER *ph, COM_SH_EXT * sh, u8 patch_idx, PATCH_INFO* patch);
#if PATCH
int enc_eco_send(COM_BSW * bs);
#endif
int enc_eco_udata(ENC_CTX * ctx, COM_BSW * bs);

int encode_pred_mode(COM_BSW * bs, u8 pred_mode, ENC_CTX * ctx);
int encode_ipf_flag(COM_BSW *bs, u8 ipf_flag);

int encode_mvd(COM_BSW * bs, s16 mvd[MV_D]);
void enc_sbac_init(COM_BSW * bs);

void enc_sbac_finish(COM_BSW *bs, int is_ipcm);
void enc_sbac_encode_bin(u32 bin, ENC_SBAC *sbac, SBAC_CTX_MODEL *ctx_model, COM_BSW *bs);
void enc_sbac_encode_bin_trm(u32 bin, ENC_SBAC *sbac, COM_BSW *bs);
int encode_coef(COM_BSW * bs, s16 coef[N_C][MAX_CU_DIM], int cu_width_log2, int cu_height_log2, u8 pred_mode, COM_MODE *mi, u8 tree_status, ENC_CTX * ctx);

int enc_eco_unit(ENC_CTX * ctx, ENC_CORE * core, int x, int y, int cup, int cu_width, int cu_height);
#if EST
void enc_eco_est_flag(COM_BSW * bs, int flag);
#endif
#if CHROMA_NOT_SPLIT
int enc_eco_unit_chroma(ENC_CTX * ctx, ENC_CORE * core, int x, int y, int cup, int cu_width, int cu_height);
#endif
int enc_eco_split_mode(COM_BSW *bs, ENC_CTX *c, ENC_CORE *core, int cud, int cup, int cu_width, int cu_height, int lcu_s
                       , const int parent_split, int qt_depth, int bet_depth, int x, int y);
#if MODE_CONS
void enc_eco_cons_pred_mode_child(COM_BSW * bs, u8 cons_pred_mode_child);
#endif
int enc_extension_and_user_data(ENC_CTX* ctx, COM_BSW * bs, int i, u8 isExtension, COM_SQH* sqh, COM_PIC_HEADER* pic_header);

void enc_eco_lcu_delta_qp(COM_BSW *bs, int val, int last_dqp);
void enc_eco_slice_end_flag(COM_BSW * bs, int flag);
int encode_mvd(COM_BSW *bs, s16 mvd[MV_D]);
int encode_refidx(COM_BSW * bs, int num_refp, int refi);
void encode_inter_dir(COM_BSW * bs, s8 refi[REFP_NUM], int part_size, ENC_CTX * ctx);
void encode_skip_flag(COM_BSW * bs, ENC_SBAC *sbac, int flag, ENC_CTX * ctx);
#if INTERPF
void encode_inter_filter_flag(COM_BSW * bs, int flag);
#endif
#if BGC
void encode_bgc_flag(COM_BSW * bs, int flag, int idx);
#endif
#if AWP
void encode_umve_awp_flag(COM_BSW *bs, int flag);
#else
void encode_umve_flag(COM_BSW *bs, int flag);
#endif
void encode_umve_idx(COM_BSW *bs, int umve_idx);
#if UMVE_ENH 
void encode_umve_idx_sec_set(COM_BSW *bs, int umve_idx);
#endif
#if AFFINE_UMVE
void encode_affine_umve_flag(COM_BSW *bs, int flag, ENC_CTX * ctx);
void encode_affine_umve_idx(COM_BSW * bs, int affine_umve_idx);
#endif
void enc_eco_split_flag(ENC_CTX *c, int cu_width, int cu_height, int x, int y, COM_BSW * bs, ENC_SBAC *sbac, int flag);
void encode_skip_idx(COM_BSW *bs, int skip_idx, int num_hmvp_cands, 
#if MVAP
    int num_mvap_cands,
#endif
    ENC_CTX * ctx);
void encode_direct_flag(COM_BSW *bs, int t_direct_flag, ENC_CTX * ctx);
#if IBC_BVP
void encode_ibc_bvp_flag(COM_BSW * bs, ENC_SBAC *sbac, int flag, ENC_CTX * ctx);
#endif
//! \todo Change list of arguments
void enc_eco_xcoef(ENC_CTX *ctx, COM_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type);
//! \todo Change list of arguments
int encode_intra_dir(COM_BSW *bs, u8 ipm,
#if EIPM
    u8 eipm_flag,
#endif
    u8 mpm[2]);
#if TSCPM
int encode_intra_dir_c(COM_BSW *bs, u8 ipm, u8 ipm_l, u8 tscpm_enable_flag
#if ENHANCE_TSPCM
    , u8 enhance_tscpm_enable_flag
#endif
#if PMC
    , u8 pmc_enable_flag
#endif
);
#else
int encode_intra_dir_c(COM_BSW *bs, u8 ipm, u8 ipm_l
#if PMC
    , u8 pmc_enable_flag
#endif
);
#endif
#if IPCM
void encode_ipcm(ENC_SBAC *sbac, COM_BSW *bs, s16 pcm[MAX_CU_DIM], int tb_width, int tb_height, int cu_width, int bit_depth, int ch_type);
#endif
int encode_mvr_idx(COM_BSW *bs, u8 mvr_idx, BOOL is_affine_mode);
#if IBC_ABVR
int encode_bvr_idx(COM_BSW * bs, u8 bvr_idx);
#endif

#if EXT_AMVR_HMVP
void encode_extend_amvr_flag(COM_BSW *bs, u8 mvp_from_hmvp_flag);
#endif

void encode_affine_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_affine_mrg_idx( COM_BSW * bs, s16 affine_mrg_idx, ENC_CTX * ctx);

#if ETMVP
void encode_etmvp_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_etmvp_idx(COM_BSW *bs, int etmvp_idx);
#endif

#if AWP
void encode_awp_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_awp_mode(COM_BSW * bs, int awp_partition_idx, int awp_cand_idx0, int awp_cand_idx1, ENC_CTX * ctx);
#endif

#if AWP_MVR
void encode_awp_idx(COM_BSW * bs, int awp_cand_idx);
void encode_awp_mode1(COM_BSW * bs, int awp_partition_idx, int awp_cand_idx0, int awp_cand_idx1, ENC_CTX * ctx);
void encode_awp_mvr_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_awp_mvr_idx(COM_BSW * bs, int awp_mvr_idx);
#endif

#if SMVD
void encode_smvd_flag( COM_BSW * bs, int flag );
#endif

#if USE_IBC
int enc_eco_ibc(COM_BSW * bs, u8 pred_mode_ibc_flag, ENC_CTX *ctx);
#endif

#if USE_SP
void encode_sp_or_ibc_cu_flag(COM_BSW *bs, int cu_width, int cu_height, ENC_CU_DATA* cu_data, int cup, ENC_CTX *ctx);
void enc_eco_sp(ENC_CTX* ctx, ENC_CORE * core, COM_MODE* mod_info_curr, COM_BSW *bs, ENC_CU_DATA* cu_data, int x, int y, int cup);
int  enc_eco_sp_or_ibc_flag(COM_BSW * bs, u8 sp_or_ibc_flag);
int  enc_eco_sp_flag(COM_BSW * bs, u8 sp_flag);
int  enc_eco_sp_copy_dir_flag(COM_BSW * bs, u8 sp_copy_direct_flag);
int  enc_eco_sp_special_len_flag(COM_BSW * bs, u8 sp_special_len_flag);
int  enc_eco_above_offset(COM_BSW * bs, u8 sp_above_offset);
int  enc_eco_offset_zero(COM_BSW * bs, u8 offset_zero, u8 is_offset_x);
void enc_eco_sp_mvd_hor(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag);
void enc_eco_sp_mvd_ver(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag);
void enc_eco_sp_mvd(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, u8 sp_copy_dir, int cu_width, int cu_height, int cur_pixel, s16 mvd[MV_D]);
int  enc_eco_sp_n_recent_flag(COM_BSW * bs, u8 sp_n_recent_flag);
void enc_eco_sp_n_recent_index(COM_BSW * bs, ENC_SBAC *sbac, s8 flag);
int  enc_eco_sp_is_matched_flag(COM_BSW * bs, u8 sp_is_matched_flag);
void enc_eco_len_in_suffix(COM_BSW * bs, unsigned int b, unsigned int temp, int n, unsigned int max_val_infix);
int  enc_eco_sp_string_length(COM_BSW * bs, u16 value, u16 max_value);
int  enc_eco_pixel_y(pel pixel[N_C], int bit_depth, COM_BSW *bs);
int  enc_eco_pixel_uv(pel pixel[N_C], int bit_depth, COM_BSW *bs);
#endif

#if DT_SYNTAX
void encode_part_size(ENC_CTX *ctx, COM_BSW * bs, int part_size, int cu_w, int cu_h, int pred_mode);
#endif

#if SBT
int enc_eco_sbt_info( COM_BSW * bs, int log2_cuw, int log2_cuh, int sbt_info, u8 sbt_avail );
#endif

int enc_eco_DB_param(COM_BSW * bs, COM_PIC_HEADER *sh
#if DBK_SCC
    , COM_SQH *sqh
#endif
);

void  enc_eco_sao_mrg_flag(ENC_SBAC *sbac, COM_BSW *bs, int mergeleft_avail, int mergeup_avail, SAOBlkParam *saoBlkParam);
void  enc_eco_sao_mode(ENC_SBAC *sbac, COM_BSW *bs, SAOBlkParam *saoBlkParam);
void  enc_eco_sao_offset(ENC_SBAC *sbac, COM_BSW *bs, SAOBlkParam *saoBlkParam);
void  enc_eco_sao_type(ENC_SBAC *sbac, COM_BSW *bs, SAOBlkParam *saoBlkParam);
#if ESAO
void enc_eco_esao_pic_header(COM_BSW * bs, COM_PIC_HEADER *sh);
#endif
int enc_eco_ALF_param(COM_BSW * bs, COM_PIC_HEADER *sh);
void enc_eco_AlfLCUCtrl(ENC_SBAC *sbac, COM_BSW *bs, int iflag);
void enc_eco_AlfCoeff(COM_BSW *bs, ALFParam *Alfp);

void Demulate(COM_BSW * bs);

#ifdef __cplusplus
}
#endif
#endif /* _ENC_ECO_H_ */
