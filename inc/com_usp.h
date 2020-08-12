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

#ifndef COM_USP_H
#define COM_USP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "com_def.h"
#if USE_SP
typedef enum 
{
    NONE_TYPE = 0,
    LAST_STR = 1,
    NUM_SP_LEN_TYPE = 2
}SP_LENGTH_TYPE;

enum SP_HASH_STATUS_IDX
{
    HS_CURR_BEST,
    HS_NEXT_BEST,
    HS_TEMP_BEST,
    HS_CTX_NUM,
};

typedef struct _COM_SP_PIX
{
    pel Y;
    pel U;
    pel V;
}COM_SP_PIX;

typedef struct _COM_SP_POS
{
    s16 x;
    s16 y;
}COM_SP_POS;

typedef struct _COM_SP_INPUT
{
    int sample_bit_depth;
    int chroma_format;
    int img_width;
    int img_height;
    int max_cu_width;
    int max_cu_height;
    int y_stride;
    int recy_stride;
    int c_stride;
    int recc_stride;
}COM_SP_INPUT;

typedef struct _COM_SP_CODING_UNIT
{
    int         cu_pix_x;
    int         cu_pix_y;
    int         cu_width_log2;
    int         cu_height_log2;
    int         pic_width_in_scu;
    int         pic_height_in_scu;
    int         qp;
    pel         rec[N_C][MAX_CU_DIM];
    u32         *map_scu;
    int         scup;
    u8          tree_status;
    double      lamda;
    double      chroma_weight[2];
    double      cur_bst_rdcost;
    u8          string_prediction_mode_flag;
    u8          string_copy_direction;  //TRUE:Horizontal FALSE:Vertical
    int         sub_string_no;
    COM_SP_INFO p_string_copy_info[SP_STRING_INFO_NO];
    int         max_str_cnt;            //max string num =width * height / 4
    int         p_x, p_y;               //x_pos,y_pos of parent cu
    int         p_width, p_height;      //width,height of parent cu
    COM_MOTION  *p_cand;
    u16         p_cand_num;
    COM_MOTION  *b_cand;
    u16         b_cand_num;
    COM_MOTION  *n_cand;
    s8          n_cand_num;
    u8          is_sp_pix_completed;
    u8          is_sp_skip_non_scc;
} COM_SP_CODING_UNIT;
#endif
#ifdef __cplusplus
}
#endif

#endif 