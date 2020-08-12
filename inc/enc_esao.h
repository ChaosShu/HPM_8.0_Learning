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

#ifndef ENC_ESAO_H
#define ENC_ESAO_H
#include "com_def.h"
#include  "enc_def.h"
#if ESAO
void get_multi_classes_statistics_for_esao(COM_PIC *pic_org,COM_PIC  *pic_lic, ESAOStatData *esao_state_data, int bit_depth, int comp_idx, int smb_pix_height, int smb_pix_width,
    int label_index,int lcu_pos,int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon);

void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAOStatData **esao_luma_state_data, ESAOStatData **esao_chroma_state_data);

void get_lcu_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAOStatData *lcu_sata_data, int comp_idx, int lcu_pos, int label_index, int is_template_one);

int uv_not_equal_count(ESAOBlkParam *esao_cur_param,int *begin, int mode_index);

void eco_esao_chroma_band_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag);

void eco_esao_chroma_len(int length, int start_band, int mode_index, ENC_SBAC *sbac, COM_BSW *bs);

long long int  distortion_cal_esao(long long int count, int offset, long long int diff);

long long int get_distortion_esao(int comp_idx, ESAOStatData esao_state_data, ESAOBlkParam *sao_cur_param, int mode_index, int types);

void enc_eco_esao_lcu_control_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag);

int offset_esao_estimation(double lambda, int offset_ori, int count, long long int diff,double *best_cost);

void find_esao_offset(int comp_idx, ESAOStatData esao_state_date, ESAOBlkParam *esao_blk_param, double lambda, int mode_index, int types);

void eco_esao_offset_AEC(int value1, ENC_SBAC *sbac, COM_BSW *bs);

void enc_eco_esao_param(ENC_CTX *ctx, COM_BSW * bs, COM_PIC_HEADER *sh);

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, ESAOBlkParam *rec_esao_cur_param, int mode_index,
    double *cost_count, int *types, int comp_idx, int uv_offset_count[2], int start_band[2], int esao_chroma_band_flag[2]);

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *min_cost);

void esao_rdcost_for_mode_lcu(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *cost_best);

void get_frame_param_for_esao(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp);

void enc_esao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs);

void enc_esao_init(ENC_CTX *ctx);

int enc_esao(ENC_CTX *ctx, ENC_CORE *core);
#endif 
#endif
