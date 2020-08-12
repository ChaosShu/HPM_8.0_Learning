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

#ifndef COM_ESAO_H
#define COM_ESAO_H
#include "com_def.h"
#if ESAO
void esao_on_block_for_luma_avx(pel* p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type);

void esao_on_block_for_luma_sse(pel* p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type);

void esao_on_block_for_luma_without_simd(pel* p_dst, int  i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type);

void esao_on_block_for_chroma_avx(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int  bit_depth, const int *esao_offset);

void esao_on_block_for_chroma_sse(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset);

void esao_on_block_for_chroma_without_simd(pel * p_dst, int i_dst, pel * p_src, int  i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset);

void decide_esao_filter_func_pointer(ESAOFuncPointer *func_esao_filter);

int com_malloc_2d_esao_statdate(ESAOStatData *** array2D, int num_SMB, int num_comp);

void com_free_2d_esao_statdate(ESAOStatData **array2D, int num_SMB);

void copy_esao_param_for_one_component(ESAOBlkParam *esao_para_dst, ESAOBlkParam *esao_para_src);

void copy_frame_for_esao(COM_PIC * pic_dst, COM_PIC * pic_src);

void check_boundary_available_for_esao(COM_INFO *info, COM_MAP *map, int pix_y, int pix_x, int smb_pix_height, int smb_pix_width, int comp, int *smb_process_left, 
    int *smb_process_right, int *smb_process_up, int *smb_process_down, int *smb_process_upleft, int *smb_process_upright, int *smb_process_leftdown, int *smb_process_rightdwon, int filter_on);

void esao_on_smb(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOFuncPointer *func_esao_filter, int pix_y, int pix_x, int lcu_pix_width, int lcu_pix_height, ESAOBlkParam *esao_blk_param, int sample_bit_depth, int lcu_pos);

void esao_on_block(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOBlkParam *esao_blk_param, ESAOFuncPointer *func_esao_filter, int comp_idx, int pix_y, int pix_x, int lcu_pix_height, int lcu_pix_width, int sample_bit_depth, int lcu_pos, int lcu_open_flag,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon);

void esao_on_frame(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAOBlkParam *rec_esao_params, ESAOFuncPointer *func_esao_filter);
#endif
#endif
