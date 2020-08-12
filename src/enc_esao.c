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
#include "enc_mode.h"
#include "com_tbl.h"
#include "com_df.h"
#include <math.h>
#include "com_util.h"
#if ESAO
#include "enc_esao.h"
#include "com_esao.h"
#endif

#if ESAO
void get_multi_classes_statistics_for_esao(COM_PIC *pic_org, COM_PIC  *pic_esao, ESAOStatData *esao_state_data, int bit_depth, int comp_idx, int lcu_height, int lcu_width,
    int label_index, int lcu_pos, int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int x, y;
    pel *rec_pic, *org_pic;
    rec_pic = NULL;
    org_pic = NULL;
    int rec_stride, org_stride;
    int shift_count = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        rec_stride = pic_esao->stride_luma;
        org_stride = pic_org->stride_luma;
        rec_pic = pic_esao->y;
        org_pic = pic_org->y;
        break;
    case U_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->u;
        org_pic = pic_org->u;
        break;
    case V_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->v;
        org_pic = pic_org->v;
        break;
    default:
        rec_stride = 0;
        org_stride = 0;
        rec_pic = NULL;
        org_pic = NULL;
        assert(0);
    }
    int  start_x_r, end_x_r, start_y, end_y;
    rec_pic += pix_y * rec_stride + pix_x;
    org_pic += pix_y * org_stride + pix_x;
    if (comp_idx != 0)
    {
        start_x_r = 0;
        start_y = 0;
        end_y = lcu_height;
        end_x_r = lcu_width;
    }
    else
    {
        start_x_r = (lcu_available_left) ? 0 : 1;
        start_y = (lcu_available_up) ? 0 : 1;
        end_y = (lcu_available_down) ? lcu_height : lcu_height - 1;
        end_x_r = (lcu_available_right) ? lcu_width : lcu_width - 1;
    }
    for (y = start_y; y < end_y; y++)
    {
        for (x = start_x_r; x < end_x_r; x++)
        {
            if (comp_idx == 0)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                int diff21 = 0, diff22 = 0, diff23 = 0, diff24 = 0;
                int diff25 = 0, diff26 = 0, diff27 = 0, diff28 = 0;
                //up
                if (rec_pic[(y - 1) * rec_stride + x] > rec_pic[y * rec_stride + x])
                {
                    diff2 = 1;
                    diff22 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x] < rec_pic[y * rec_stride + x])
                {
                    diff2 = -1;
                }
                //upleft
                if (rec_pic[(y - 1) * rec_stride + x - 1] > rec_pic[y * rec_stride + x])
                {
                    diff1 = 1;
                    diff21 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x - 1] < rec_pic[y * rec_stride + x])
                {
                    diff1 = -1;
                }
                //upright
                if (rec_pic[(y - 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff3 = 1;
                    diff23 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff3 = -1;
                }
                //left
                if (rec_pic[(y)* rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff4 = 1;
                    diff24 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff4 = -1;
                }
                //right
                if (rec_pic[(y)* rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff5 = 1;
                    diff25 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff5 = -1;
                }
                //down
                if (rec_pic[(y + 1) * rec_stride + x] > rec_pic[(y)* rec_stride + x])
                {
                    diff7 = 1;
                    diff27 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x] < rec_pic[(y)* rec_stride + x])
                {
                    diff7 = -1;
                }
                //leftdown
                if (rec_pic[(y + 1) * rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff6 = 1;
                    diff26 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff6 = -1;
                }
                //rightdown
                if (rec_pic[(y + 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff8 = 1;
                    diff28 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff8 = -1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                int band_type = (rec_pic[y  * rec_stride + x] * shift_count) >> bit_depth;
                int true_Index = band_type * NUM_ESAO_LUMA_TYPE0 + diff_count;
                esao_state_data[0].diff[true_Index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                esao_state_data[0].count[true_Index] ++;
                if (ESAO_LUMA_TYPES > 1)
                {
                    diff_count = diff21 + diff22 + diff23 + diff24 + diff25 + diff26 + diff27 + diff28;
                    true_Index = band_type * NUM_ESAO_LUMA_TYPE1 + diff_count;
                    esao_state_data[1].diff[true_Index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                    esao_state_data[1].count[true_Index] ++;
                }
            }
            else
            {
                assert(comp_idx > 0 && comp_idx < 3);
                int index_shift = tab_esao_chroma_class[label_index];
                int band_type = (rec_pic[y  * rec_stride + x] * index_shift) >> bit_depth;
                esao_state_data[comp_idx - 1].diff[band_type] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                esao_state_data[comp_idx - 1].count[band_type] ++;
            }
        }
    }
}

void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map,COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAOStatData **esao_luma_state_data, ESAOStatData **esao_chroma_state_data)
{
    int bit_depth = info->bit_depth_internal;
    int lcu_pix_width ;
    int lcu_pix_height ;
    int comp_idx;
    int lcu_pix_height_t, lcu_pix_width_t;
    int is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_max_size_in_bit = info->log2_max_cuwh;
    int pix_y, pix_x, pix_x_t, pix_y_t;
    for (pix_y = 0; pix_y < pic_pix_height; pix_y += lcu_pix_height)
    {
        lcu_pix_height = min(1 << (input_max_size_in_bit), (pic_pix_height - pix_y));
        for (pix_x = 0; pix_x < pic_pix_width; pix_x += lcu_pix_width)
        {
            int x_in_lcu = pix_x >> info->log2_max_cuwh;
            int y_in_lcu = pix_y >> info->log2_max_cuwh;
            int lcu_pos = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            lcu_pix_width = min(1 << (input_max_size_in_bit), (pic_pix_width - pix_x));
            for (comp_idx = Y_C; comp_idx < N_C; comp_idx++)
            {
                lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1)) : (lcu_pix_width);
                lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1)) : (lcu_pix_height);
                pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
                pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
                check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
                      &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
                int label_count = (comp_idx == Y_C) ? ESAO_LABEL_NUM_Y : ((comp_idx == U_C) ? ESAO_LABEL_NUM_U : ESAO_LABEL_NUM_V);
                for (int label_index = 0; label_index < label_count; label_index++)
                {
                    get_multi_classes_statistics_for_esao(pic_org, pic_esao, ((comp_idx == Y_C) ? esao_luma_state_data[label_index] : esao_chroma_state_data[label_index]), bit_depth, comp_idx, lcu_pix_height_t, lcu_pix_width_t,
                        label_index, lcu_pos, pix_y_t, pix_x_t, is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
                }
            }
        }
    }
}

void get_statistics_for_esao_one_LCU(COM_PIC *pic_org, COM_PIC  *pic_esao, ESAOStatData *lcu_state_data, int bit_depth, int comp_idx, int lcu_height, int lcu_width,
    int label_index, int is_template_one, int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int x, y;
    pel *rec_pic, *org_pic;
    rec_pic = NULL;
    int rec_stride, org_stride;
    int shift_count = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        rec_stride = pic_esao->stride_luma;
        org_stride = pic_org->stride_luma;
        rec_pic = pic_esao->y;
        org_pic = pic_org->y;
        break;
    case U_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->u;
        org_pic = pic_org->u;
        break;
    case V_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->v;
        org_pic = pic_org->v;
        break;
    default:
        rec_stride = 0;
        org_stride = 0;
        rec_pic = NULL;
        org_pic = NULL;
        assert(0);
    }
    int  start_x_r, end_x_r, start_y, end_y;
    rec_pic += pix_y * rec_stride + pix_x;
    org_pic += pix_y * org_stride + pix_x;
    if (comp_idx != 0)
    {
        start_x_r = 0;
        start_y = 0;
        end_y = lcu_height;
        end_x_r = lcu_width;
    }
    else 
    {
        start_x_r = (lcu_available_left) ? 0 : 1;
        start_y = (lcu_available_up) ? 0 : 1;
        end_y = (lcu_available_down) ? lcu_height : lcu_height - 1;
        end_x_r = (lcu_available_right) ? lcu_width : lcu_width - 1;
    }
    for (y = start_y; y < end_y; y++)
    {
        for (x = start_x_r; x < end_x_r; x++)
        {
            if (comp_idx == 0)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                int diff21 = 0, diff22 = 0, diff23 = 0, diff24 = 0;
                int diff25 = 0, diff26 = 0, diff27 = 0, diff28 = 0;
                //up
                if (rec_pic[(y - 1) * rec_stride + x] > rec_pic[y * rec_stride + x]) //
                {
                    diff2 = 1;
                    diff22 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x] < rec_pic[y * rec_stride + x])
                {
                    diff2 = -1;
                }
                //upleft
                if (rec_pic[(y - 1) * rec_stride + x - 1] > rec_pic[y * rec_stride + x])
                {
                    diff1 = 1;
                    diff21 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x - 1] < rec_pic[y * rec_stride + x])
                {
                    diff1 = -1;
                }
                //upright
                if (rec_pic[(y - 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff3 = 1;
                    diff23 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff3 = -1;
                }
                //left
                if (rec_pic[(y)* rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff4 = 1;
                    diff24 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff4 = -1;
                }
                //right
                if (rec_pic[(y)* rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff5 = 1;
                    diff25 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff5 = -1;
                }
                //down
                if (rec_pic[(y + 1) * rec_stride + x] > rec_pic[(y)* rec_stride + x])
                {
                    diff7 = 1;
                    diff27 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x] < rec_pic[(y)* rec_stride + x])
                {
                    diff7 = -1;
                }
                //downleft
                if (rec_pic[(y + 1) * rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff6 = 1;
                    diff26 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff6 = -1;
                }
                //downright
                if (rec_pic[(y + 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff8 = 1;
                    diff28 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff8 = -1;
                }
                int diff_count, num;
                int band_type = (rec_pic[y  * rec_stride + x] * shift_count) >> 10;
                if (is_template_one)
                {
                    diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                    num = NUM_ESAO_LUMA_TYPE0;
                }
                else 
                {
                    diff_count = diff21 + diff22 + diff23 + diff24 + diff25 + diff26 + diff27 + diff28;
                    num = NUM_ESAO_LUMA_TYPE1;
                }
                int true_index = band_type * num + diff_count;
                lcu_state_data->diff[true_index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                lcu_state_data->count[true_index] ++;
            }
            else
            {
                int index_shift = tab_esao_chroma_class[label_index];
                int band_type = (rec_pic[y  * rec_stride + x] * index_shift) >> bit_depth;
                lcu_state_data->diff[band_type] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                lcu_state_data->count[band_type] ++;
            }
        }
    }
}

void get_lcu_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAOStatData *lcu_state_data, int comp_idx, int lcu_pos, int label_index, int is_template_one)
{
    int bit_depth = info->bit_depth_internal;
    int lcu_pix_width;
    int lcu_pix_height;
    int lcu_pix_height_t, lcu_pix_width_t;
    int  lcu_height = 1 << info->log2_max_cuwh;
    int lcu_width = lcu_height;
    int  is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail,
        is_below_right_avail;
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_max_size_in_bit = info->log2_max_cuwh;
    int pix_y, pix_x, pix_x_t, pix_y_t;
    //1.calculate the position according to lcu_pos
    pix_y = (lcu_pos / info->pic_width_in_lcu)*lcu_height;
    pix_x = (lcu_pos %info->pic_width_in_lcu)*lcu_width;
    lcu_pix_width = min(1 << (input_max_size_in_bit), (pic_pix_width - pix_x));
    lcu_pix_height = min(1 << (input_max_size_in_bit), (pic_pix_height - pix_y));
    lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1)) : (lcu_pix_width);
    lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1)) : (lcu_pix_height);
    pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
    pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
    check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
        &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
    get_statistics_for_esao_one_LCU(pic_org, pic_esao, lcu_state_data, bit_depth, comp_idx, lcu_pix_height_t, lcu_pix_width_t, label_index, is_template_one, pix_y_t, pix_x_t,
        is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
}

int uv_not_equal_count(ESAOBlkParam *esao_cur_param,int *begins, int mode_index)
{
    int count = 0;
    int class_idc;
    int class_index = tab_esao_chroma_class[mode_index];
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        if (esao_cur_param->offset[class_idc] != 0)
        {
                begins[0] = class_idc; //start the band not equal zero
                break;
        }
    }
    for (class_idc = class_index-1; class_idc >=0; class_idc--)
    {
        if (esao_cur_param->offset[class_idc] != 0)
        {
            begins[1] = class_idc; //end the band not equal zero
            break;
        }
    }
    count = begins[1] - begins[0] + 1;
    return count;
}

long long int  distortion_cal_esao(long long int count, int offset, long long int diff)
{
    return (count * (long long int)offset * (long long int)offset - diff * offset * 2);
}

long long int get_distortion_esao(int comp_idx,ESAOStatData esao_state_data, ESAOBlkParam *esao_cur_param, int mode_index,int types)
{
    int class_idc;
    long long int dist = 0;
    int class_index;
    if (comp_idx == Y_C)
    {
        int num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        class_index = (mode_index + 1) * num;
    }
    else
    {
        class_index = tab_esao_chroma_class[mode_index];
    }
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        dist += distortion_cal_esao(esao_state_data.count[class_idc], esao_cur_param->offset[class_idc],
            esao_state_data.diff[class_idc]);
    }
    return dist;
}

int offset_esao_estimation(double lambda, int offset_ori, int count, long long int diff, double *best_cost)
{
    int cur_offset = offset_ori;
    int offset_best = 0;
    int lower_bd, upper_bd, Th;
    int temp_offset, start_offset, end_offset;
    int temp_rate;
    long long int temp_dist;
    double temp_cost, min_cost;
    int offset_type;
    int offset_step;
    offset_type = 0;
    lower_bd = esao_clip[offset_type][0];
    upper_bd = esao_clip[offset_type][1];
    Th = esao_clip[offset_type][2];
    offset_step = 1;
    cur_offset = COM_CLIP3(lower_bd, upper_bd, cur_offset);
    start_offset = cur_offset >= 0 ? 0 : cur_offset;
    end_offset = cur_offset >= 0 ? cur_offset : 0;
    min_cost = MAX_COST;
    for (temp_offset = start_offset; temp_offset <= end_offset; temp_offset += offset_step)
    {
        int offset = temp_offset;
        temp_rate = abs(offset);
        temp_rate = temp_rate ? (temp_rate + 1) : 0;
        temp_rate = (temp_rate == Th) ? temp_rate : (temp_rate + 1);
        temp_dist = distortion_cal_esao(count, temp_offset, diff);
        temp_cost = (double)temp_dist + lambda * (double)temp_rate;
        if (temp_cost < min_cost)
        {
            min_cost = temp_cost;
            offset_best = temp_offset;
            *best_cost = temp_cost;
        }
    }
    return offset_best;
}

void find_esao_offset(int comp_idx, ESAOStatData esao_state_date, ESAOBlkParam *esao_blk_param, double lambda, int mode_index, int types)
{
    int class_i;
    double class_cost[ESAO_LABEL_CLASSES_MAX];
    double offth;
    int num_class ,num;
    if (comp_idx == Y_C)
    {
        num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        num_class = (mode_index + 1) * num;
    }
    else
    {
        num_class = tab_esao_chroma_class[mode_index];
    }
    for (int i = 0; i < ESAO_LABEL_CLASSES_MAX; i++)
    {
        esao_blk_param->offset[i] = 0;
    }
    for (class_i = 0; class_i < num_class; class_i++)
    {
        if (esao_state_date.count[class_i] == 0)
        {
            esao_blk_param->offset[class_i] = 0;
            continue;
        }
        offth = esao_state_date.diff[class_i] > 0 ? 0.5 : (esao_state_date.diff[class_i] < 0 ?
            -0.5 : 0);
        esao_blk_param->offset[class_i] = (int)((double)esao_state_date.diff[class_i] /
            (double)esao_state_date.count[class_i] + offth);
    }
    for (class_i = 0; class_i < num_class; class_i++)
    {
        if (esao_state_date.count[class_i] == 0)
        {
            esao_blk_param->offset[class_i] = 0;
            continue;
        }
        esao_blk_param->offset[class_i] = offset_esao_estimation(lambda, esao_blk_param->offset[class_i],
            esao_state_date.count[class_i], esao_state_date.diff[class_i], &(class_cost[class_i]));
    }
}

unsigned int esao_uvlcBitrate_estimate(int val)
{
    unsigned int length = 1;
    val++;
    assert(val);
    while (1 != val)
    {
        val >>= 1;
        length += 2;
    }
    return ((length >> 1) + ((length + 1) >> 1));
}

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, ESAOBlkParam *rec_esao_cur_param, int mode_index,
    double *cost_count, int *types, int comp_idx, int uv_offset_count[2], int start_band[2], int esao_chroma_band_flag[2])
{
    ENC_SBAC *esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_type), (*esao_sbac));
    SBAC_STORE((core->s_esao_cur_best), (*esao_sbac));
    long long int cur_dist;
    int  cur_rate; 
    int remeFlag = 0;
    double cur_cost, min_cost;
    int class_count;
    int type_num, loops, num;
    ESAOBlkParam temp_esao_param[ESAO_LUMA_TYPES];
    type_num = (comp_idx == Y_C) ? ESAO_LUMA_TYPES : 1;
    min_cost = 0;
    for (loops = 0; loops < type_num; loops++)
    {
        SBAC_LOAD((*esao_sbac), (core->s_esao_cur_type));
        if (comp_idx == Y_C)
        {
            num = (loops == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
            class_count = (mode_index + 1) * num;
        }
        else 
        {
            class_count = tab_esao_chroma_class[mode_index];
        }
        if (comp_idx == Y_C)
        {
            find_esao_offset(comp_idx, ctx->esao_luma_data[mode_index][loops], &temp_esao_param[loops], esao_lambda[comp_idx], mode_index, loops);
            cur_dist = get_distortion_esao(comp_idx, ctx->esao_luma_data[mode_index][loops], &temp_esao_param[loops], mode_index, loops);
        }
        else
        {
            find_esao_offset(comp_idx, ctx->esao_chroma_data[mode_index][comp_idx - 1], &temp_esao_param[loops], esao_lambda[comp_idx], mode_index, loops);
            cur_dist = get_distortion_esao(comp_idx, ctx->esao_chroma_data[mode_index][comp_idx - 1], &temp_esao_param[loops], mode_index, loops);
        }
        int count_not_zero=0;
        int class_count_ori = class_count;
        int begins[2] = { 0,0 };
        int uvop = 1;
        if (comp_idx != Y_C)
        {
            count_not_zero = uv_not_equal_count(&temp_esao_param[loops], begins, mode_index);
            class_count = count_not_zero;
        }
        if (comp_idx != Y_C)
        {
            //try to optimize the chroma filter coeff
            cur_rate = enc_get_bit_number(esao_sbac);
            eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 1);
            eco_esao_chroma_len(class_count, begins[0], mode_index, esao_sbac, esao_bs_temp);
            for (int i = 0; i < class_count; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i + begins[0]], esao_sbac, esao_bs_temp);
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
            SBAC_STORE((core->s_esao_uvop), (*esao_sbac));
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_type));
            eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 0);
            int off_rate = enc_get_bit_number(esao_sbac);
            for (int i = 0; i < class_count_ori; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i], esao_sbac, esao_bs_temp);
            }
            off_rate = enc_get_bit_number(esao_sbac) - off_rate;
            if (off_rate < cur_rate)
            {
                cur_rate = off_rate;
                uvop = 0;
            }
            else
            {
                uvop = 1;
                SBAC_LOAD((*esao_sbac), (core->s_esao_uvop));
            }
        }
        else
        {
            cur_rate = enc_get_bit_number(esao_sbac);
            for (int i = 0; i < class_count_ori; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i], esao_sbac, esao_bs_temp);
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
        }
        cur_rate += esao_uvlcBitrate_estimate(mode_index);
        cur_cost = (double)(cur_dist)+RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], cur_rate);
        if (cur_cost < min_cost)
        {
            min_cost = cur_cost;
            cost_count[comp_idx] = cur_cost;
            remeFlag = 1;
            if (comp_idx == Y_C)
            {
                (*types) = loops;
            }
            SBAC_STORE((core->s_esao_cur_best), (*esao_sbac));
            copy_esao_param_for_one_component(rec_esao_cur_param, &(temp_esao_param[loops]));
            if (comp_idx != Y_C)
            {
                esao_chroma_band_flag[comp_idx - 1] = uvop;
                uv_offset_count[comp_idx - 1] = count_not_zero;
                start_band[comp_idx - 1] = begins[0];
            }
        }
    }
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_best));
}

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *min_cost)
{
    ESAOBlkParam rec_esao_cur_param;
    double cost_yuv[N_C] = { MAX_COST ,MAX_COST ,MAX_COST };
    int label_yuv[N_C] = { ESAO_LABEL_NUM_Y,ESAO_LABEL_NUM_U,ESAO_LABEL_NUM_V };
    ENC_SBAC * esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_new), (*esao_sbac));
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
        SBAC_STORE((core->s_esao_next_type), (*esao_sbac));
        for (int mode = 0; mode < label_yuv[comp_idx]; mode++)
        {
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
            cost_yuv[comp_idx] = MAX_COST;
            int types;
            int uv_offset_count[2];
            int start_band[2];
            int esao_chroma_band_flag[2];
            esao_rdcost_for_mode_new_yuv_each(ctx, core, esao_bs_temp, esao_lambda, &rec_esao_cur_param, mode, cost_yuv, &types, comp_idx , 
                uv_offset_count, start_band, esao_chroma_band_flag);
            if (cost_yuv[comp_idx] < min_cost[comp_idx])
            {
                min_cost[comp_idx] = cost_yuv[comp_idx];
                ctx->info.pic_header.esao_adaptive_param[comp_idx] = mode;
                ctx->info.pic_header.pic_esao_on[comp_idx] = 1;
                if (comp_idx == Y_C)
                {
                    ctx->info.pic_header.esao_luma_type = types;
                    assert(ctx->info.pic_header.esao_luma_type == 0 || ctx->info.pic_header.esao_luma_type == 1);
                }
                if (comp_idx != Y_C )
                {
                    if (esao_chroma_band_flag[comp_idx - 1])
                    {
                        ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1] = 1;
                        ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1] = uv_offset_count[comp_idx - 1];
                        ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1] = start_band[comp_idx - 1];
                    }
                    else
                    {
                        ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1] = 0;
                    }
                }
                copy_esao_param_for_one_component(&(ctx->pic_esao_params[comp_idx]), &rec_esao_cur_param);
                SBAC_STORE((core->s_esao_next_type), (*esao_sbac));
            }
        }
        SBAC_LOAD((*esao_sbac), (core->s_esao_next_type));
    }
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_new));
}

void esao_rdcost_for_mode_lcu(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *cost_best)
{
    ENC_SBAC * esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
    int comp_idx;
    int  cur_rate;
    int  modex_index;
    int *lcu_open_control_flag;
    ESAOStatData lcu_stata_data;
    int class_count;
    int types, num;
    int is_template_one=0;
    int lcu_count = ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0));
    lcu_open_control_flag = (int *)malloc(sizeof(int)*lcu_count);
    for (comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        if (ctx->info.pic_header.pic_esao_on[comp_idx])
        {
            types = (comp_idx == Y_C) ? ctx->info.pic_header.esao_luma_type : 0;
            modex_index = ctx->info.pic_header.esao_adaptive_param[comp_idx]; // class
            if (comp_idx == Y_C)
            {
                is_template_one = (types == 0) ? 1 : 0;
                num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
                class_count = (modex_index + 1) * num;
            }
            else 
            {
                class_count = tab_esao_chroma_class[modex_index];
            }
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
            cur_rate = enc_get_bit_number(esao_sbac);
            if (comp_idx != Y_C && ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1])
            {
                eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 1);
                class_count = ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1];
                eco_esao_chroma_len(class_count, ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1], modex_index, esao_sbac, esao_bs_temp);
                for (int i = 0; i < class_count; i++)
                {
                    eco_esao_offset_AEC(ctx->pic_esao_params[comp_idx].offset[i + ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1]], esao_sbac, esao_bs_temp);
                }
            }
            else
            {
                if (comp_idx != Y_C)
                {
                    eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 0);
                }
                for (int i = 0; i < class_count; i++)
                {
                    eco_esao_offset_AEC(ctx->pic_esao_params[comp_idx].offset[i], esao_sbac, esao_bs_temp);
                }
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
            cur_rate += esao_uvlcBitrate_estimate(modex_index);
            double one_lcu_open_cost, one_lcu_close_cost;
            int one_lcu_rate;
            double cost_lcu_total = 0;
            SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac));
            for (int lcu_index = 0; lcu_index < lcu_count; lcu_index++)
            {
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                memset(lcu_stata_data.diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
                memset(lcu_stata_data.count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
                get_lcu_statistics_for_esao(&ctx->info, &ctx->map, PIC_ORG(ctx), PIC_REC(ctx), &lcu_stata_data, comp_idx, lcu_index, modex_index, is_template_one);
                one_lcu_open_cost = (double)get_distortion_esao(comp_idx, lcu_stata_data, &ctx->pic_esao_params[comp_idx], modex_index, types);
                //open
                one_lcu_rate = enc_get_bit_number(esao_sbac);
                enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, 1);
                one_lcu_rate = enc_get_bit_number(esao_sbac) - one_lcu_rate;
                one_lcu_open_cost += RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], one_lcu_rate);
                SBAC_STORE((core->s_esao_lcu_open), (*esao_sbac));
                //close
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                one_lcu_rate = enc_get_bit_number(esao_sbac);
                enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, 0);
                one_lcu_rate = enc_get_bit_number(esao_sbac) - one_lcu_rate;
                one_lcu_close_cost = RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], one_lcu_rate);
                SBAC_STORE((core->s_esao_lcu_close), (*esao_sbac)); 
                if (one_lcu_open_cost < one_lcu_close_cost) 
                {
                    cost_lcu_total += one_lcu_open_cost;
                    lcu_open_control_flag[lcu_index] = 1;
                    SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_open));
                    SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac)); 
                }
                else
                {
                    cost_lcu_total += one_lcu_close_cost;
                    lcu_open_control_flag[lcu_index] = 0;
                    SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_close));
                    SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac)); 
                }
            }
            cost_lcu_total += RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], cur_rate);
            if (cost_lcu_total < cost_best[comp_idx])
            {
                ctx->info.pic_header.esao_lcu_enable[comp_idx] = 1;
                cost_best[comp_idx] = cost_lcu_total;
                memcpy(ctx->pic_esao_params[comp_idx].lcu_flag, lcu_open_control_flag, sizeof(int)*lcu_count);
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
            }
            else
            {
                ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
            }
        }
        else
        {
            ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
        }
    }
}

void get_frame_param_for_esao(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp)
{
    double esao_lambda[N_C];
    double min_cost[N_C] = { 0,0,0 };
    int scale_lambda = (ctx->info.bit_depth_internal == 10) ? ctx->info.qp_offset_bit_depth : 1;
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        esao_lambda[comp_idx] = ctx->lambda[0] * scale_lambda;
    }
    
    esao_rdcost_for_mode_new(ctx, core, esao_bs_temp, esao_lambda, min_cost);
    
    esao_rdcost_for_mode_lcu(ctx, core, esao_bs_temp, esao_lambda, min_cost);
}

void enc_esao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs)
{
    get_frame_statistics_for_esao(&ctx->info, &ctx->map, PIC_ORG(ctx), ctx->pic_esao, ctx->esao_luma_data, ctx->esao_chroma_data);
    get_frame_param_for_esao(ctx, core, esao_bs);
}

void enc_esao_init(ENC_CTX *ctx)
{
    copy_frame_for_esao(ctx->pic_esao, PIC_REC(ctx));
    ctx->info.pic_header.esao_luma_type = -1;
    int lcu_count = ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0));
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        ctx->info.pic_header.esao_adaptive_param[comp_idx] = -1;
        ctx->info.pic_header.pic_esao_on[comp_idx] = 0;
        ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
        memset(ctx->pic_esao_params[comp_idx].lcu_flag, 0, sizeof(int)*lcu_count);
    }
    for (int i = 0; i < ESAO_LABEL_NUM_Y; i++)
    {
        for (int j = 0; j < ESAO_LUMA_TYPES; j++)
        {
            memset(ctx->esao_luma_data[i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
            memset(ctx->esao_luma_data[i][j].diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
        }
    }
    for (int i = 0; i < ESAO_LABEL_NUM_MAX; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            memset(ctx->esao_chroma_data[i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
            memset(ctx->esao_chroma_data[i][j].diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
        }
    }
}

int enc_esao(ENC_CTX *ctx, ENC_CORE *core)
{
    enc_esao_init(ctx);
    enc_sbac_init(&(core->bs_temp));
    enc_esao_rdo(ctx, core, &(core->bs_temp));
    copy_frame_for_esao(ctx->pic_esao, PIC_REC(ctx));
    esao_on_frame(&ctx->info, &ctx->map, PIC_REC(ctx), ctx->pic_esao, ctx->pic_esao_params, &ctx->func_esao_block_filter);
    return COM_OK;
}
#endif 