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

#include "enc_sp.h"
#if USE_SP
#include <string.h>
#include <math.h>
#include <vector>
#include <limits> 
using namespace std;
#include <cassert>

string_prediction::string_prediction(COM_SP_INPUT* input, u8 is_encoding)
{    
    m_max_cu_width            = input->max_cu_width;
    m_max_cu_height           = input->max_cu_height;
    m_pic_width               = input->img_width;
    m_pic_height              = input->img_height;
    m_is_encoding              = is_encoding;
    this->input               = &iptemp;
    this->input->max_cu_width = input->max_cu_width;
    this->input->max_cu_height = input->max_cu_height;
    this->input->img_width    = input->img_width;
    this->input->img_height   = input->img_height;
    this->input->sample_bit_depth = input->sample_bit_depth;
    this->input->chroma_format = input->chroma_format;    
    this->input->y_stride     = input->y_stride;
    this->input->c_stride     = input->c_stride;
    this->input->recy_stride  = input->recy_stride;    
    this->input->recc_stride  = input->recc_stride;
    if ( m_is_encoding )
    {
        //memory allocation
        m_dict = new COM_SP_POS*[m_pic_height];
        for (int i = 0; i < m_pic_height; i++)
        {
            m_dict[i] = new COM_SP_POS[m_pic_width];
        }
        m_ppp_dict_status = new SP_HASH_STATUS***[MAX_CU_DEPTH];
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            m_ppp_dict_status[i] = new SP_HASH_STATUS**[MAX_CU_DEPTH];
            for (int j = 0; j < MAX_CU_DEPTH; j++) 
            {
                m_ppp_dict_status[i][j] = new SP_HASH_STATUS*[HS_CTX_NUM];
                for (int k = 0; k < HS_CTX_NUM; k++)
                {
                    m_ppp_dict_status[i][j][k] = new SP_HASH_STATUS;
                }
            }
        }
        //initialize
        for (int i = 0; i < SP_HASH_SIZE; i++)
        {
            m_hash_table[i].x = -1;
            m_hash_table[i].y = -1;
        }
        for (int i = 0; i < m_pic_height; i++)
        {
            for (int j = 0; j < m_pic_width; j++)
            {
                m_dict[i][j].x = -1;
                m_dict[i][j].y = -1;
            }
        }
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            for (int j = 0; j < MAX_CU_DEPTH; j++) 
            {
                for (int k = 0; k < HS_CTX_NUM; k++)
                {
                    for (int l = 0; l < SP_HASH_SIZE; l++)
                    {
                        m_ppp_dict_status[i][j][k]->m_hash_table[l].x = -1;
                        m_ppp_dict_status[i][j][k]->m_hash_table[l].y = -1;
                    }
                    for (int h = 0; h < MAX_CU_SIZE; h++)
                    {
                        for (int w = 0; w < MAX_CU_SIZE; w++)
                        {
                            m_ppp_dict_status[i][j][k]->m_dict[h][w].x = -1;
                            m_ppp_dict_status[i][j][k]->m_dict[h][w].y = -1;
                        }
                    }
                }
            }
        }
        get_sp_hashvalue = &string_prediction::get_sp_hash_value_lossy;
    }
    else
    {
        m_dict = NULL;
        m_ppp_dict_status = NULL;
    }
    
    for ( int i=0; i<3; i++ )
    {
        m_p_pel_org[i] = NULL;
        m_p_pel_rec[i] = NULL;
    }    
}

void* get_sp_instance(COM_SP_INPUT* input, u8 isEncoding)
{
    return new string_prediction(input, isEncoding);
}

void release_sp_instance(void* sp_encoder)
{
    if (sp_encoder)
    {
        delete ((string_prediction*)sp_encoder);
        sp_encoder = NULL;
    }
}

void sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec)
{
    if (sp_encoder)
    {
        ((string_prediction*)sp_encoder)->reset(imgY_org, imgU_org, imgV_org, imgY_rec, imgU_rec, imgV_rec);
    }
}

u8 sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double* min_rdcost, double* distorion)
{
    if (sp_encoder == NULL)
    {
        return 0;
    }
    return ((string_prediction*)sp_encoder)->rdcost_sp_mode(cur_sp_info, min_rdcost, distorion);
}

void sp_lcu_hash_save(void* sp_encoder, int x_lcu_start, int y_lcu_start)
{
    if (sp_encoder == NULL)
    {
        return;
    }
    for (int i = 0; i < MAX_CU_DEPTH; i++)
    {
        for (int j = 0; j < MAX_CU_DEPTH; j++) 
        {
            ((string_prediction*)sp_encoder)->save(((string_prediction*)sp_encoder)->m_ppp_dict_status[i][j][HS_CURR_BEST], x_lcu_start, y_lcu_start, MAX_CU_SIZE, MAX_CU_SIZE); 
        }
    }
}

void sp_lcu_hash_restore(void* sp_encoder, int x_lcu_start, int y_lcu_start)
{
    ((string_prediction*)sp_encoder)->restore(((string_prediction*)sp_encoder)->m_ppp_dict_status[MAX_CU_LOG2 - 2][MAX_CU_LOG2 - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start, MAX_CU_SIZE, MAX_CU_SIZE);
}

void sp_cu_hash_copy(void* sp_encoder, int dst_w, int dst_h, int dst_status, int src_w, int src_h, int src_status)
{
    ((string_prediction*)sp_encoder)->m_ppp_dict_status[src_w][src_h][src_status]->save(((string_prediction*)sp_encoder)->m_ppp_dict_status[dst_w][dst_h][dst_status]);
}

u8 sp_cu_hash_update(void* sp_encoder, int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start)
{
    if (sp_encoder == NULL)
    {
        return FALSE;
    }
    return ((string_prediction*)sp_encoder)->sp_hash_update(cu_width_log2, cu_height_log2, x_cu_start, y_cu_start);
}

u8 sp_is_hash_updated(u8 *map_usp, int x, int y, int cu_width, int cu_height, int pic_width_in_scu)
{
    u8 is_sp_pix_completed = TRUE;
    u8 *map = map_usp;
    int idx = 0;
    for (int j = 0; j < cu_height >> MIN_CU_LOG2; j++)
    {
        for (int i = 0; i < cu_width >> MIN_CU_LOG2; i++)
        {
            is_sp_pix_completed &= map[idx + i];
            if (is_sp_pix_completed == FALSE)
            {
                return is_sp_pix_completed;
            }
        }
        idx += cu_width >> MIN_CU_LOG2;
    }
    return is_sp_pix_completed;
}

u8 get_adaptive_sp_flag(void* sp_encoder)
{
    if (sp_encoder == NULL)
    {
        return FALSE;
    }
    return (u8)((string_prediction*)sp_encoder)->sp_hash_sp_judge(0, 0);
}

string_prediction::~string_prediction()
{
    if (m_is_encoding)
    {
        if (m_dict != NULL)
        {
            for (int i = 0; i < m_pic_height; i++)
            {
                if (m_dict[i] != NULL)
                {
                    delete[] m_dict[i];
                    m_dict[i] = NULL;
                }
            }
            delete[] m_dict;
            m_dict = NULL;
        }
    } //m_is_encoding
    if (m_ppp_dict_status != NULL)
    {
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            if (m_ppp_dict_status[i] != NULL)
            {
                for (int j = 0; j < HS_CTX_NUM; j++)
                {
                    if (m_ppp_dict_status[i][j] != NULL)
                    {
                        delete m_ppp_dict_status[i][j];
                        m_ppp_dict_status[i][j] = NULL;
                    }
                }
                delete[] m_ppp_dict_status[i];
                m_ppp_dict_status[i] = NULL;
            }
        }
        delete[] m_ppp_dict_status;
        m_ppp_dict_status = NULL;
    }
}

void string_prediction::reset(pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec,pel *imgV_rec)
{
    if ( imgY_org != NULL )
    {
        m_p_pel_org[0] = imgY_org;
        m_p_pel_org[1] = imgU_org;
        m_p_pel_org[2] = imgV_org;
        m_stride_org[0] = input->y_stride;
        m_stride_org[1] = input->c_stride;
        m_stride_org[2] = input->c_stride;
    }
    if( imgY_rec!=NULL )
    {
        m_p_pel_rec[0] = imgY_rec;
        m_p_pel_rec[1] = imgU_rec;
        m_p_pel_rec[2] = imgV_rec;
        m_stride_rec[0] = input->recy_stride;
        m_stride_rec[1] = input->recc_stride;
        m_stride_rec[2] = input->recc_stride;
    }
}

u16 string_prediction::get_sp_hash_value_lossy(COM_SP_PIX pixelCur)
{
    u8 CRCCalculationBuffer[3];
    u8* p = CRCCalculationBuffer;
    p[0] = static_cast<u8>(pixelCur.Y >> 2);
    p[1] = static_cast<u8>(pixelCur.U >> 2);
    p[2] = static_cast<u8>(pixelCur.V >> 2);
    u16 crc = ((p[0] & 0xff) << 4) | ((p[1] & 0xc0) >> 4) | ((p[2] & 0xc0) >> 6); //822
    return crc;
}

void string_prediction::add_to_hash_table(int x_start_in_pic, int y_start_in_pic, int width, int height, u8 scale_x, u8 scale_y)
{
    COM_SP_PIX cur_pixel;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int current_x = x_start_in_pic + j;
            int current_y = y_start_in_pic + i;
            int lengthMinus1 = 0;
            int startPositionX = current_x - lengthMinus1;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos]; 
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            u16 hashValue = (this->*get_sp_hashvalue)(cur_pixel);
            m_dict[current_y][current_x].x = m_hash_table[hashValue].x;
            m_dict[current_y][current_x].y = m_hash_table[hashValue].y;
            m_hash_table[hashValue].x = startPositionX;
            m_hash_table[hashValue].y = startPositionY;
        }
    }
}

void string_prediction::save(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start, int width, int height)
{
    memcpy(pDictStatus->m_hash_table, m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    int stride = (x_lcu_start + MAX_CU_SIZE) > m_pic_width ? (m_pic_width - x_lcu_start) : MAX_CU_SIZE;
    for (int j = 0; j < MAX_CU_SIZE; j++)
    {
        if ((y_lcu_start + j) < m_pic_height)
        {
            memcpy(pDictStatus->m_dict[j], &m_dict[y_lcu_start + j][x_lcu_start], sizeof(COM_SP_POS)*stride);
        }
    }
}

void string_prediction::restore(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start, int width, int height)
{
    memcpy(m_hash_table, pDictStatus->m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    int stride = (x_lcu_start + MAX_CU_SIZE) > m_pic_width ? (m_pic_width - x_lcu_start) : MAX_CU_SIZE;
    for (int j = 0; j < MAX_CU_SIZE; j++)
    {
        if ((y_lcu_start + j) < m_pic_height)
        {
            memcpy(&m_dict[y_lcu_start + j][x_lcu_start], pDictStatus->m_dict[j], sizeof(COM_SP_POS)* stride);
        }
    }
}

u8 string_prediction::sp_hash_update(int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start)
{
    int width = 1 << cu_width_log2;
    int height = 1 << cu_height_log2;
    int x_lcu_start = (x_cu_start >> MAX_CU_LOG2) << MAX_CU_LOG2;
    int y_lcu_start = (y_cu_start >> MAX_CU_LOG2) << MAX_CU_LOG2;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    restore(m_ppp_dict_status[cu_width_log2 - 2][cu_height_log2 - 2][HS_CURR_BEST], x_lcu_start, y_lcu_start, width, height);
    add_to_hash_table(x_cu_start, y_cu_start, width, height, scale_x, scale_y);
    save(m_ppp_dict_status[cu_width_log2 - 2][cu_height_log2 - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start, width, height); 
    return TRUE;
}

void SP_HASH_STATUS::save(SP_HASH_STATUS* p)
{
    memcpy(p->m_hash_table, m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    for (int j = 0; j < MAX_CU_SIZE; j++)
    {
        memcpy(p->m_dict[j], m_dict[j], sizeof(COM_SP_POS)*MAX_CU_SIZE);//ZF
    }
}

u8 string_prediction::sp_hash_sp_judge(int x_start_in_pic, int y_start_in_pic)
{
    int hitratio = 0;
    int non_replic_pix = 0, all_pix = 0, diff_pix = 0;
    COM_SP_PIX cur_pixel;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    int hash_count[SP_HASH_SIZE] = { 0 };
    int long_chain = 0, short_chain = 0;
    for (int i = 0; i < m_pic_height; i ++)
    {
        for (int j = 0; j < m_pic_width; j ++)
        {
            int current_x = x_start_in_pic + j;
            int current_y = y_start_in_pic + i;
            int startPositionX = current_x;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos];
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            u8 CRCCalculationBuffer[3];
            u8 * p = CRCCalculationBuffer;
            p[0] = static_cast<u8>(cur_pixel.Y);
            p[1] = static_cast<u8>(cur_pixel.U);
            p[2] = static_cast<u8>(cur_pixel.V);
            u16 hashValue = ((p[0] & 0xf0) << 4) | ((p[1] & 0xf0)) | ((p[2] & 0xf0) >> 4);
            hash_count[hashValue]++;
            if (hash_count[hashValue]==1)
            {
                non_replic_pix++;
            }
        }
    }
    for (int k = 0; k < SP_HASH_SIZE; k++)
    {
        if (hash_count[k] >= (m_pic_width*m_pic_width >> 11))
        {
            all_pix++;
        }
    }
    for (int k = 0; k < SP_HASH_SIZE-1; k++)
    {
        long_chain = (hash_count[k] >= hash_count[k + 1]) ? hash_count[k] : hash_count[k + 1];
        short_chain = (hash_count[k] < hash_count[k + 1]) ? hash_count[k] : hash_count[k + 1];
        if (((short_chain + 1) << 7) < long_chain)
        {
            diff_pix++;
        }
    }
    if (diff_pix * 1000 / (all_pix+1) > 200)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int string_prediction::get_sp_search_area_width(int x_start_in_pic, int uiMaxSearchWidthToLeftInCTUs)
{
    const int lcu_width = MAX_CU_SIZE;
    const int max_width = uiMaxSearchWidthToLeftInCTUs * lcu_width;
    int width = 0;
    while (width < max_width)
    {
        if (x_start_in_pic / lcu_width == 0)
        {
            break;
        }
        else
        {
            x_start_in_pic -= lcu_width;
            width += lcu_width;
        }
    }
    return (width < max_width) ? width : max_width;
}

u8 string_prediction::rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion)
{
    u8 isFound = check_rdcost_sp_mode(p_cur_sp_info, min_rdcost, distorion);
    return isFound;
}
u8 string_prediction::check_rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion)
{
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int x_cu_start = p_cur_sp_info->cu_pix_x;
    int y_cu_start = p_cur_sp_info->cu_pix_y;
    u8 result = 0;    
    // do rdo here
    int x_lcu_start = (x_cu_start >> MAX_CU_LOG2) << MAX_CU_LOG2;
    int y_lcu_start = (y_cu_start >> MAX_CU_LOG2) << MAX_CU_LOG2;
    restore(m_ppp_dict_status[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2][HS_CURR_BEST], x_lcu_start, y_lcu_start, width, height);
    if (p_cur_sp_info->string_prediction_mode_flag == TRUE) 
    {
        vector<COM_SP_INFO> vec_dict_info;
        set_org_block(width, height, x_cu_start, y_cu_start);
        int cu_size_str_found = FALSE;
        result = encode_cu_size_str(p_cur_sp_info, x_cu_start, y_cu_start, vec_dict_info, min_rdcost, distorion);
        if (result == TRUE)
        {
            cu_size_str_found = TRUE;
        }
        if (cu_size_str_found == FALSE)
        {
            result = encode_general_str(p_cur_sp_info, x_cu_start, y_cu_start, vec_dict_info, min_rdcost, distorion);
        }
        if (result == TRUE) 
        {
            p_cur_sp_info->sub_string_no = (int)vec_dict_info.size();
            assert(p_cur_sp_info->sub_string_no <= p_cur_sp_info->max_str_cnt);
            for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
            {
                p_cur_sp_info->p_string_copy_info[i] = vec_dict_info.at(i);
            }
            if (p_cur_sp_info->is_sp_pix_completed)
            {
                save(m_ppp_dict_status[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start, x_cu_start, y_cu_start);
            }
        }
        return result;
    }
    return 0;
}

void string_prediction::add_pixel_to_hash_table(int count, u8 includeHash, int x_start_in_pic, int y_start_in_pic, int cu_width_log2, int cu_height_log2, int iStartIdx, u8 bHorizontal)
{
    int width = 1 << cu_width_log2;
    int height = 1 << cu_height_log2;
    int* p_trav_scan_order = com_tbl_raster2trav[bHorizontal][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
    if (includeHash)
    {
        for (int pixel_count = 0; pixel_count < count; pixel_count++)
        {
            int current_x;
            int current_y;
            int  uiIdx = iStartIdx + pixel_count;
            int  trav_order_index = p_trav_scan_order[uiIdx];
            current_x = x_start_in_pic + GET_TRAV_X(trav_order_index, width);
            current_y = y_start_in_pic + GET_TRAV_Y(trav_order_index, cu_width_log2);
            int lengthMinus1 = 0;
            int startPositionX = current_x - lengthMinus1;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            u16 hashValue = (this->*get_sp_hashvalue)(m_p_org_pixel_buffer[trav_order_index]);
            m_dict[current_y][current_x].x = m_hash_table[hashValue].x;
            m_dict[current_y][current_x].y = m_hash_table[hashValue].y;
            m_hash_table[hashValue].x = startPositionX;
            m_hash_table[hashValue].y = startPositionY;
        }
    }
}

int string_prediction::get_tb_bits(int ui_symbol, int uiMaxSymbol)
{
    int bins = 0;
    int uiThresh;
    if (uiMaxSymbol > 256)
    {
        int uiThreshVal = 1 << 8;
        uiThresh = 8;
        while (uiThreshVal <= uiMaxSymbol)
        {
            uiThresh++;
            uiThreshVal <<= 1;
        }
        uiThresh--;
    }
    else
    {
        uiThresh = g_sp_tb_tbl[uiMaxSymbol];
    }
    int uiVal = 1 << uiThresh;
    assert(uiVal <= uiMaxSymbol);
    assert((uiVal << 1) > uiMaxSymbol);
    assert(ui_symbol < (int)uiMaxSymbol);
    int b = uiMaxSymbol - uiVal;
    assert(b < uiVal);
    if (ui_symbol < (int)(uiVal - b))
    {
        bins = uiThresh;
    }
    else
    {
        bins = uiThresh + 1;
    }
    return bins;
}

int string_prediction::get_len_bits_cnt(int len, int totalLeftPixel)
{
    int bins = 0;
    assert(len <= totalLeftPixel);
    if (len < 4) // 2 bits
    {
        if (totalLeftPixel >= 4)
        {
            bins++;
        }
        if (totalLeftPixel < 3)
            bins += get_tb_bits(len, totalLeftPixel + 1);
        else
            bins += 2;
    }
    else if (len - 4 < 16) // 4 bits
    {
        bins += 1;
        if (totalLeftPixel - 4 >= 16)
        {
            bins += 1;
        }
        if (totalLeftPixel < 19)
            bins += get_tb_bits(len - 4, totalLeftPixel - 3);
        else
            bins += 4;
    }
    else if (len - 4 - 16 < 256) // 8 bits
    {
        bins += 1;
        bins += 1;
        if (totalLeftPixel - 4 - 16 >= 256)
        {
            bins += 1;
        }
        if (totalLeftPixel < 275)
            bins += get_tb_bits(len - 20, totalLeftPixel - 19);
        else
            bins += 8;
    }
    else if (len - 4 - 16 - 256 < 65536) // 12 bits, maximum number 4096
    {
        bins += 3;
        if (totalLeftPixel < 4096)
            bins += get_tb_bits(len - 276, totalLeftPixel - 275);
        else
            bins += 12;
    }
    return bins;
}

int string_prediction::get_offset_bits(int ui_symbol, int uiCount)
{
    int bins = 0;
    int numBins = 0;
    while (ui_symbol >= (int)(1 << uiCount))
    {
        bins = 2 * bins + 1;
        numBins++;
        ui_symbol -= 1 << uiCount;
        uiCount++;
    }
    bins = 2 * bins + 0;
    numBins++;
    bins = (bins << uiCount) | ui_symbol;
    numBins += uiCount;
    return numBins;
}

int string_prediction::check_str_len_bit(int len, int w, int h, int total_left_pixel_minus1, u8 dir, u8 *len_type)
{
    int cur_bitcount;
    u8 cur_lentype = NONE_TYPE;
    cur_bitcount = 1;
    int next_remianing_pixel_in_cu = total_left_pixel_minus1 + 1 - len;
    if (next_remianing_pixel_in_cu == 0)
    {
        cur_lentype = LAST_STR;
        cur_bitcount++;
    }
    else
    {
        assert(next_remianing_pixel_in_cu > 0);
        cur_lentype = NONE_TYPE;
        cur_bitcount += get_len_bits_cnt(next_remianing_pixel_in_cu - 1, total_left_pixel_minus1);
    }
    *len_type = cur_lentype;
    return cur_bitcount;
}

double string_prediction::get_hor_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
    , int match_length,    int offset_sign, int left_pix_in_cu_minus1, int *bit_cnt    
    , u8 *len_type, COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag)
{
    u32 sad = distortion;
    int num_of_bits = 0;
    int mappedOffset = offset_in_cu;
    // a bit for sign
#if SP_ALIGN_SIGN
    if (offset_x == 0 && offset_y == -1)
#else
    if (offset_x == 0 && offset_y == 1)
#endif
    {
        num_of_bits++;
    }
    else
    {
        int index = check_sp_offset(offset_x, offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
        if (index != p_cur_sp_info->n_cand_num)
        {
            num_of_bits++;
            num_of_bits += get_tb_bits(p_cur_sp_info->n_cand_num - 1 - index, p_cur_sp_info->n_cand_num);
            *n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
            *n_recent_flag = 1;
        }
        else
        {
            *n_recent_flag = 0;
#if SP_ALIGN_SIGN
            if (offset_x == 0 && offset_y < -1)
                offset_y++;
#else
            if (offset_x == 0 && offset_y > 1)
                offset_y--;
#endif

            num_of_bits++;
            num_of_bits++;
            if (offset_y != 0)
            {
                if (!is_boundary_cu) num_of_bits++;
                num_of_bits += get_offset_bits(abs(offset_y) - 1, 2);
#if SP_ALIGN_SIGN
                if (offset_y > 0)
#else
                if (offset_y < 0)
#endif
                {
                    num_of_bits += get_offset_bits(abs(offset_x) - mappedOffset - 1, 2);
                }
                else
                {
                    num_of_bits++;
                    if (offset_x != 0)
                    {
                        num_of_bits++;
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
                    }
                }
            }
            else
            {
                if (offset_sign == 1)
                    num_of_bits++;
                num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
            }
        }
    }
    if (match_length != 0)
    {
        num_of_bits += check_str_len_bit(match_length, 1 << p_cur_sp_info->cu_width_log2, 1 << p_cur_sp_info->cu_height_log2, left_pix_in_cu_minus1, TRUE, len_type);
    }
    *bit_cnt = num_of_bits;
    return sad + num_of_bits * p_cur_sp_info->lamda;
}

double string_prediction::get_ver_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
    , int match_length, int offset_sign, int left_pix_in_cu_minus1, int *bit_cnt
    , u8 *len_type, COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag)
{
    u32 sad = distortion;
    int num_of_bits = 0;
    // a bit for sign
#if SP_ALIGN_SIGN
    if (offset_x == -1 && offset_y == 0)
#else
    if (offset_x == 1 && offset_y == 0)
#endif
    {
        num_of_bits++;
    }
    else
    {
        int index = check_sp_offset(offset_x, offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
        if (index != p_cur_sp_info->n_cand_num)
        {
            num_of_bits++;
            num_of_bits += get_tb_bits(p_cur_sp_info->n_cand_num - 1 - index, p_cur_sp_info->n_cand_num);
            *n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
            *n_recent_flag = 1;
        }
        else
        {
            *n_recent_flag = 0;
#if SP_ALIGN_SIGN
            if (offset_y == 0 && offset_x < -1)
                offset_x--;
#else
            if (offset_y == 0 && offset_x > 1)
                offset_x--;
#endif
            num_of_bits++;
            //
            num_of_bits++;
            if (offset_y != 0)
            {
                num_of_bits++;
                num_of_bits += get_offset_bits(abs(offset_y) - 1, 2);
#if SP_ALIGN_SIGN
                if (offset_y > 0)
#else
                if (offset_y < 0)
#endif
                {
                    if (offset_sign == 1)
                    {
                        num_of_bits += get_offset_bits(abs(offset_x) + 1, 2);
                    }
                    else
                    {
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
                    }
                }
                else
                {
                    num_of_bits++;
                    if (offset_x != 0)
                    {
                        num_of_bits++;
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
                    }
                }
            }
            else
            {
                num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
            }
        }
    }
    if (match_length != 0)
    {
        num_of_bits += check_str_len_bit(match_length, 1 << p_cur_sp_info->cu_width_log2, 1 << p_cur_sp_info->cu_height_log2, left_pix_in_cu_minus1, FALSE, len_type);
    }
    *bit_cnt = num_of_bits;
    return sad + num_of_bits * p_cur_sp_info->lamda;
}

void string_prediction::get_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length,int processed_count,u8 scale_x, u8 scale_y,int uiShiftY, int uiShiftC,u32 *str_dist_y, u32 *str_dist_uv)
{
    u8 tree_status = p_cur_sp_info->tree_status;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int* p_trav_scan_order = com_tbl_raster2trav[p_cur_sp_info->string_copy_direction][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    int trav_order_index = p_trav_scan_order[processed_count];
    int trav_x, trav_y;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
#if SP_ALIGN_SIGN
    int mvX = offset_x;
    int mvY = offset_y;
#else
    int mvX = -offset_x;
    int mvY = -offset_y;
#endif
    u32 ui_sum;
    s32 i_temp;
    for (int pixel_count = 0; pixel_count < match_length; pixel_count++)
    {
        trav_order_index = p_trav_scan_order[processed_count + pixel_count];
        trav_x = GET_TRAV_X(trav_order_index, width);
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
        int cur_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x;
        int cur_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y;
        int ref_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x + mvX;
        int ref_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y + mvY;
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_C)
        {
            cur_pos = cur_y_in_pic * m_stride_rec[0] + cur_x_in_pic;
            ref_pos = ref_y_in_pic * m_stride_rec[0] + ref_x_in_pic;
            i_temp = m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_rec[0][ref_pos]; 
            ui_sum += ((i_temp * i_temp) >> uiShiftY);
            *str_dist_y += ui_sum;            
            m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
        }
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            cur_pos_c = (cur_y_in_pic >> scale_y) * m_stride_rec[1] + (cur_x_in_pic >> scale_x);
            ref_pos_c = (ref_y_in_pic >> scale_y) * m_stride_rec[1] + (ref_x_in_pic >> scale_x);
            i_temp = m_p_org_pixel_buffer[trav_order_index].U - m_p_pel_rec[1][ref_pos_c]; 
            ui_sum += ((i_temp * i_temp) >> uiShiftC);
            i_temp = m_p_org_pixel_buffer[trav_order_index].V - m_p_pel_rec[2][ref_pos_c]; 
            ui_sum += ((i_temp * i_temp) >> uiShiftC);
            *str_dist_uv += ui_sum;
            if (!((cur_x_in_pic & scale_x) || (cur_y_in_pic & scale_y)))
            {
                m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
            }
        }
    }
}

void string_prediction::set_org_block(int uiWidth, int uiHeight, int x_start_in_pic, int y_start_in_pic)
{
    COM_SP_PIX cur_pixel;
    int ui_pos;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    //gen major flag;  
    ui_pos = 0;
    for (int uiY = 0; uiY < uiHeight; uiY++)
    {
        for (int uiX = 0; uiX < uiWidth; uiX++)
        {
            int current_x = x_start_in_pic + uiX;
            int current_y = y_start_in_pic + uiY;
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos];
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            //save org pixel
            m_p_org_pixel_buffer[ui_pos].Y = m_p_pel_org[0][cur_pos];
            m_p_org_pixel_buffer[ui_pos].U = m_p_pel_org[1][cur_pos_c];
            m_p_org_pixel_buffer[ui_pos].V = m_p_pel_org[2][cur_pos_c];
            ui_pos++;
        }
    }
}

void string_prediction::init_sps_cands(COM_MOTION *p_cand, int p_cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num)
{
    COM_MOTION tmp;
    int i = 0, j = 0;
    if (dir == FALSE) //ver
    {
#if SP_ALIGN_SIGN
        tmp.mv[0][MV_X] = -1;
        tmp.mv[0][MV_Y] = 0;
#else
        tmp.mv[0][MV_X] = 1;
        tmp.mv[0][MV_Y] = 0;
#endif
        sps_cands.push_back(tmp);
#if SP_ALIGN_SIGN
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = -1;
#else
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = 1;
#endif
        sps_cands.push_back(tmp);
    }
    else //hor
    {
#if SP_ALIGN_SIGN
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = -1;
#else
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = 1;
#endif
        sps_cands.push_back(tmp);
#if SP_ALIGN_SIGN
        tmp.mv[0][MV_X] = -1;
        tmp.mv[0][MV_Y] = 0;
#else
        tmp.mv[0][MV_X] = 1;
        tmp.mv[0][MV_Y] = 0;
#endif
        sps_cands.push_back(tmp);
    }
    for (i = 0; i < p_cand_num; i++)
    {
        for (j = 0; j < sps_cands.size(); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], p_cand[i].mv[0]))
            {
                break;
            }
        }
        if (j == sps_cands.size())
        {
            sps_cands.push_back(p_cand[i]);
        }
    }
    for (i = 0; i < b_cand_num; i++)
    {
        for (j = 0; j < sps_cands.size(); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], b_cand[i].mv[0]))
            {
                break;
            }
        }
        if (j == sps_cands.size())
        {
            sps_cands.push_back(b_cand[i]);
        }
    }
}

void string_prediction::refine_sps_cands(vector<COM_MOTION>& org_sps_cands, vector<COM_MOTION>& refined_sps_cands, int current_x, int current_y)
{
    refined_sps_cands.clear();
    s16 mv_x = 0, mv_y = 0;
    for (int i = 0; i < org_sps_cands.size(); i++)
    {
        mv_x = org_sps_cands.at(i).mv[0][MV_X];
        mv_y = org_sps_cands.at(i).mv[0][MV_Y];
#if SP_ALIGN_SIGN
        if (current_x + mv_x >= 0 && current_y + mv_y >= 0)
#else
        if (current_x - mv_x >= 0 && current_y - mv_y >= 0)
#endif
        {
            refined_sps_cands.push_back(org_sps_cands.at(i));
        }
    }
}

u8 string_prediction::encode_general_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion)
{
    if (p_cur_sp_info->cu_width_log2 == 5 && p_cur_sp_info->cu_height_log2 == 5)
    {
        p_cur_sp_info->is_sp_skip_non_scc = 0;
    }
    if (p_cur_sp_info->is_sp_skip_non_scc)
    {
        return FALSE;
    }
    int replicate_num = 0;
    int same_num = 0;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int count = width * height;
    int processed_count = 0;
    int left_search_range = get_sp_search_area_width(x_start_in_pic, 1);
    const int  lcu_width = m_max_cu_width;
    const int  lcu_height = m_max_cu_height;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1;
    int bit_depth = input->sample_bit_depth;
    u8 is_hor_scan = p_cur_sp_info->string_copy_direction; //true=horizontal; false=vertical
    int cu_height_log2 = p_cur_sp_info->cu_height_log2;
    int cu_width_log2 = p_cur_sp_info->cu_width_log2;
    int  trav_x;
    int  trav_y;
    int  error_limit = 1 + (sp_max_error_quant[(int(p_cur_sp_info->qp) >> 1) + 10] << 2);
    double dict_rdcost = 0;
    u32   dict_dist_y = 0, dict_dist_uv = 0;
    double tmp_dict_rdcost = 0;
    u32 tmp_dict_dist_y = 0, tmp_dict_dist_uv = 0;
    int* p_trav_scan_order = com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];  // processed_count to TravOrder
    int* p_raster_scan_order = com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];  // TravOrder to processed_count
    int trav_order_index;
    int ui_shift_y = (bit_depth == 8) ? 0 : (bit_depth - 8) << 1;
    int ui_shift_c = (bit_depth == 8) ? 0 : (bit_depth - 8) << 1;
    vector<COM_MOTION> sps_cand;
    init_sps_cands(p_cur_sp_info->p_cand, p_cur_sp_info->p_cand_num, p_cur_sp_info->b_cand, p_cur_sp_info->b_cand_num, sps_cand, p_cur_sp_info->string_copy_direction,p_cur_sp_info->n_cand, p_cur_sp_info->n_cand_num);
    vector<COM_MOTION> refined_sps_cand, *p_sps_cand;
    p_sps_cand = &sps_cand;

    while (processed_count < count)
    {
        u8 string_found = FALSE;
        const int max_test_cases = MAX_SM_TEST_CASE;
        int attemp = 0;
        int current_x;
        int current_y;
        trav_order_index = p_trav_scan_order[processed_count];
        trav_x = GET_TRAV_X(trav_order_index, width);
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
        current_x = x_start_in_pic + trav_x;
        current_y = y_start_in_pic + trav_y;
        int cur_offset = current_y * m_stride_org[0] + current_x;
        int max_match_length = 0;
        u8  best_length_type = NONE_TYPE;
        s8  best_n_recent_idx = SP_RECENT_CANDS;
        u8  best_n_recent_flag = 0;
        int best_match_start_x = -1;
        int best_match_start_y = -1;
        u32 str_dist_y = 0, str_dist_uv = 0;
        int str_bits = 0; 
        int best_str_bits = 0;
        u32 best_str_dist_y = 0;
        u32 best_str_dist_uv = 0;
        double cur_rdcost = 0;
        double min_rdcost = SP_MAX_COST;
        {
            u16 hash_value = (this->*get_sp_hashvalue)(m_p_org_pixel_buffer[trav_order_index]);
            int curr_match_start_x = m_hash_table[hash_value].x;
            int curr_match_start_y = m_hash_table[hash_value].y;
            int after_sps_start_x = curr_match_start_x;
            int after_sps_start_y = curr_match_start_y;
            int sps_attemp = 0;
            refine_sps_cands(sps_cand, refined_sps_cand, current_x, current_y);
            p_sps_cand = &refined_sps_cand;
            if ((*p_sps_cand).size() > 0)
            {
#if SP_ALIGN_SIGN
                curr_match_start_x = current_x + (*p_sps_cand).at(sps_attemp).mv[0][MV_X];
                curr_match_start_y = current_y + (*p_sps_cand).at(sps_attemp).mv[0][MV_Y];
#else
                curr_match_start_x = current_x - (*p_sps_cand).at(sps_attemp).mv[0][MV_X];
                curr_match_start_y = current_y - (*p_sps_cand).at(sps_attemp).mv[0][MV_Y];
#endif
            }
            for (; attemp < max_test_cases && curr_match_start_x >= 0 && curr_match_start_y >= 0; attemp++)
            {
                int curr_match_length = 0;
                int next_match_start_x;
                int next_match_start_y;
                if (sps_attemp < (*p_sps_cand).size())
                {
                    if (sps_attemp + 1 < (*p_sps_cand).size())
                    {
#if SP_ALIGN_SIGN
                        next_match_start_x = current_x + (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_X];
                        next_match_start_y = current_y + (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_Y];
#else
                        next_match_start_x = current_x - (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_X];
                        next_match_start_y = current_y - (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_Y];
#endif
                    }
                    else
                    {
                        assert(sps_attemp + 1 == (*p_sps_cand).size());
                        next_match_start_x = after_sps_start_x;
                        next_match_start_y = after_sps_start_y;
                    }
                    sps_attemp++;
                    
                    if (curr_match_start_y < 0 || curr_match_start_y >= m_pic_height || curr_match_start_x < 0 || curr_match_start_x >= m_pic_width)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                }
                else
                {
                    assert(!(curr_match_start_y < 0 || curr_match_start_y >= m_pic_height || curr_match_start_x < 0 || curr_match_start_x >= m_pic_width));
                    next_match_start_x = m_dict[curr_match_start_y][curr_match_start_x].x;
                    next_match_start_y = m_dict[curr_match_start_y][curr_match_start_x].y;
                }
                s8 curr_n_recent_idx = SP_RECENT_CANDS;
                u8 curr_n_recent_flag = 0;
                u8 curr_length_type = NONE_TYPE;
                int ref_offset = 0;
                int ref_offset_c = 0;
                int mv_x = curr_match_start_x - current_x;
                int mv_y = curr_match_start_y - current_y;
                if (best_match_start_x == curr_match_start_x && best_match_start_y == curr_match_start_y)
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                if ((mv_y > 0 && mv_x > 0) || (mv_x == 0 && mv_y == 0)) 
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                int i_ref_x = (current_x & (lcu_width - 1)) + mv_x;
                int i_ref_y = (current_y & (lcu_height - 1)) + mv_y;
                //Test if the reference pixel is within Search Range
                if (i_ref_x < -left_search_range || i_ref_x >= (int)lcu_width || i_ref_y >= (int)lcu_height || i_ref_y < 0)
                {
                    //out of the valid research range
                    break;
                }
                ref_offset = curr_match_start_y * m_stride_org[0] + curr_match_start_x;
                ref_offset_c = (curr_match_start_y >> scale_y) * m_stride_org[1] + (curr_match_start_x >> scale_x);
                if (abs(m_p_pel_org[0][cur_offset] - m_p_pel_org[0][ref_offset]) >= error_limit)
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                int max_len_minus1_trav_idx = 0;
                int cur_minus1_x, cur_minus1_y;
                int ref_x_in_pic_n;
                int ref_y_in_pic_n;
                if (max_match_length > 2)
                {
                    max_len_minus1_trav_idx = p_trav_scan_order[processed_count + max_match_length - 2]; 
                    cur_minus1_x = GET_TRAV_X(max_len_minus1_trav_idx, width);
                    cur_minus1_y = GET_TRAV_Y(max_len_minus1_trav_idx, p_cur_sp_info->cu_width_log2);
                    ref_x_in_pic_n = x_start_in_pic + cur_minus1_x + mv_x;
                    ref_y_in_pic_n = y_start_in_pic + cur_minus1_y + mv_y;
                    if (ref_y_in_pic_n < 0 || ref_y_in_pic_n >= m_pic_height || ref_x_in_pic_n < 0 || ref_x_in_pic_n >= m_pic_width)
                        break;
                    int cur_n_offset = (y_start_in_pic + cur_minus1_y) * m_stride_org[0] + x_start_in_pic + cur_minus1_x;
                    int ref_n_offset = ref_y_in_pic_n * m_stride_org[0] + ref_x_in_pic_n;
                    if (abs(m_p_pel_org[0][cur_n_offset] - m_p_pel_org[0][ref_n_offset]) >= error_limit)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                    int i_ref_x_n = ((x_start_in_pic + cur_minus1_x) & (lcu_width - 1)) + mv_x;
                    int i_ref_y_n = ((y_start_in_pic + cur_minus1_y) & (lcu_height - 1)) + mv_y;
                    //Test if the reference pixel is within Search Range
                    if (i_ref_x_n < -left_search_range || ref_x_in_pic_n >= m_pic_width || i_ref_x_n >= (int)lcu_width || ref_y_in_pic_n >= m_pic_height || i_ref_y_n >= (int)lcu_height || i_ref_y_n < 0)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                    assert(i_ref_x >= -128 && i_ref_x < 128 && i_ref_y >= 0 && i_ref_y < 128);
                    if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, ref_x_in_pic_n, ref_y_in_pic_n, processed_count + max_match_length - 2, curr_match_start_x, curr_match_start_y)) 
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                }
                //HASH search
                {
                    int pc = processed_count;
                    int c_match_length = 0;
                    int ref_pos = ref_offset;
                    int ref_pos_c = ref_offset_c;
                    trav_order_index = p_trav_scan_order[processed_count];
                    trav_x = GET_TRAV_X(trav_order_index, width);
                    trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                    if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, x_start_in_pic + trav_x + mv_x, y_start_in_pic + trav_y + mv_y, processed_count, curr_match_start_x, curr_match_start_y)) 
                    {
                        assert(curr_match_start_x == x_start_in_pic + trav_x + mv_x);
                        assert(curr_match_start_y == y_start_in_pic + trav_y + mv_y);
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                    if (is_pix_in_errorlimit(trav_order_index, ref_pos, ref_pos_c, error_limit, p_cur_sp_info->tree_status))
                    {
                        c_match_length++;
                        pc++;
                        while ((int)pc < count)
                        {
                            int i_cur_x, i_cur_y;
                            trav_order_index = p_trav_scan_order[pc];
                            i_cur_x = GET_TRAV_X(trav_order_index, width);
                            i_cur_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            i_ref_x = ((x_start_in_pic + i_cur_x) & (lcu_width - 1)) + mv_x;
                            i_ref_y = ((y_start_in_pic + i_cur_y) & (lcu_height - 1)) + mv_y;
                            int iRefXInPic = x_start_in_pic + i_cur_x + mv_x;
                            int iRefYInPic = y_start_in_pic + i_cur_y + mv_y;
                            //Test if the reference pixel is within Search Range
                            if (i_ref_x < -left_search_range || iRefXInPic >= m_pic_width || i_ref_x >= (int)lcu_width || iRefYInPic >= m_pic_height || i_ref_y >= (int)lcu_height || i_ref_y < 0)
                            {
                                break; //out of the valid research range
                            }
#if SP_BASE
                            if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, iRefXInPic, iRefYInPic, processed_count, curr_match_start_x, curr_match_start_y))
#else
                            if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, iRefXInPic, iRefYInPic, pc, curr_match_start_x, curr_match_start_y)) 
#endif
                            {
                                break;
                            }
                            ref_pos = iRefYInPic * m_stride_org[0] + iRefXInPic;
                            ref_pos_c = (iRefYInPic >> scale_y) * m_stride_org[1] + (iRefXInPic >> scale_x);
                            if (is_pix_in_errorlimit(trav_order_index, ref_pos, ref_pos_c, error_limit, p_cur_sp_info->tree_status))
                            {
                                c_match_length++;
                                pc++;
                            }
                            else
                                break;
                        }
                    }
                    curr_match_length = c_match_length;
                }
                {
                    u8 isBCU = FALSE;
                    str_dist_y = 0; str_dist_uv = 0;
                    if (is_hor_scan)
                    {
                        int offset_in_cu = 0;
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 1;
                        if (curr_match_length != 0 && curr_match_length >= mxLen)
                        {
                            trav_order_index = p_trav_scan_order[processed_count];
                            int offset_sign_flag = 0;
                            int offset_x_in_cu = GET_TRAV_X(trav_order_index, width);
                            int offset_y_in_cu = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            if (1 == (offset_y_in_cu & 0x1))
                            {
                                offset_sign_flag = 1;
                            }
                            if (error_limit != 1)
                            {
#if SP_ALIGN_SIGN
                                get_string_dist(p_cur_sp_info, mv_x, mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
#else
                                get_string_dist(p_cur_sp_info, -mv_x, -mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
#endif
                            }
#if SP_ALIGN_SIGN
                            cur_rdcost = get_hor_sp_cost(curr_match_start_x - current_x, curr_match_start_y - current_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, &curr_length_type, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
#else
                            cur_rdcost = get_hor_sp_cost(current_x - curr_match_start_x, current_y - curr_match_start_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, &curr_length_type, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
#endif
                        }
                        else
                            cur_rdcost = SP_MAX_COST;
                    }
                    else
                    {
                        int offset_in_cu = 0;
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 1;
                        if (curr_match_length != 0 && (int)curr_match_length >= mxLen)
                        {
                            trav_order_index = p_trav_scan_order[processed_count];
                            int offset_sign_flag = 0;
                            int offset_x_in_cu = GET_TRAV_X(trav_order_index, width);
                            int offset_y_in_cu = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            if (1 == (offset_x_in_cu & 0x1))
                            {
                                offset_sign_flag = 1;
                            }
                            if (error_limit != 1)
                            {
#if SP_ALIGN_SIGN
                                get_string_dist(p_cur_sp_info, mv_x, mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
#else
                                get_string_dist(p_cur_sp_info, -mv_x, -mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
#endif
                            }
#if SP_ALIGN_SIGN
                            cur_rdcost = get_ver_sp_cost(curr_match_start_x - current_x, curr_match_start_y - current_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, &curr_length_type, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
#else
                            cur_rdcost = get_ver_sp_cost(current_x - curr_match_start_x, current_y - curr_match_start_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]), 
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, &curr_length_type, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
#endif
                        }
                        else
                            cur_rdcost = SP_MAX_COST;
                    }
                }
                if (curr_match_length > max_match_length
                    || ((curr_match_length == max_match_length) && curr_match_length != 0 && cur_rdcost < min_rdcost)
                    || ((curr_match_length == max_match_length - 1) && curr_match_length != 0 && (cur_rdcost*max_match_length) < (min_rdcost*curr_match_length))
                    )    
                {
                    string_found = TRUE;
                    max_match_length = curr_match_length;
                    best_length_type = curr_length_type;
                    best_n_recent_flag = curr_n_recent_flag;
                    best_n_recent_idx = curr_n_recent_idx;
                    best_match_start_x = curr_match_start_x;
                    best_match_start_y = curr_match_start_y;
                    min_rdcost = cur_rdcost;
                    best_str_bits = str_bits;
                    best_str_dist_y = str_dist_y;
                    best_str_dist_uv = str_dist_uv;
                }
                curr_match_start_x = next_match_start_x;
                curr_match_start_y = next_match_start_y;
#if !SP_ENC_OPT
                if (processed_count + curr_match_length == count)
                {
                    break;
                }
#endif
                if (str_dist_y + str_dist_uv ==0 && (int)curr_match_length >= (width * height >> 2))
                {
                    break;
                }
            }
        } // end hash search
        if (string_found)
        {
            tmp_dict_rdcost = min_rdcost; //choose Hash RDCost
            tmp_dict_dist_y = best_str_dist_y;
            tmp_dict_dist_uv = best_str_dist_uv;
            COM_SP_INFO dict_info;
            dict_info.is_matched = TRUE;
#if SP_ALIGN_SIGN
            dict_info.offset_x = best_match_start_x - current_x;
            dict_info.offset_y = best_match_start_y - current_y;
#else
            dict_info.offset_x = current_x - best_match_start_x;
            dict_info.offset_y = current_y - best_match_start_y;
#endif
            dict_info.length = max_match_length;
            dict_info.sp_length_type = best_length_type;
            dict_info.n_recent_flag = best_n_recent_flag;
            dict_info.n_recent_idx = best_n_recent_idx;
            pel former_pix[N_C] = { 0 };
            pel replic_pix[N_C] = { 0 };
            //reconstruction
            for (int pixelCount = 0; pixelCount < dict_info.length; pixelCount++)
            {
                int cur_pos_x;
                int cur_pos_y;
                int cur_pixel_pos = p_trav_scan_order[processed_count + pixelCount];
                trav_x = GET_TRAV_X(cur_pixel_pos, width);
                trav_y = GET_TRAV_Y(cur_pixel_pos, p_cur_sp_info->cu_width_log2);
                cur_pos_x = x_start_in_pic + trav_x;
                cur_pos_y = y_start_in_pic + trav_y;
                int cur_pos = cur_pos_y * m_stride_rec[0] + cur_pos_x;
                int cur_pos_c = (cur_pos_y >> scale_y) * m_stride_rec[1] + (cur_pos_x >> scale_x);
#if SP_ALIGN_SIGN
                int ref_pos = (cur_pos_y + dict_info.offset_y) * m_stride_rec[0] + (cur_pos_x + dict_info.offset_x);
                int ref_pos_c = ((cur_pos_y + dict_info.offset_y) >> scale_y) * m_stride_rec[1] + ((cur_pos_x + dict_info.offset_x) >> scale_x);
#else
                int ref_pos = (cur_pos_y - dict_info.offset_y) * m_stride_rec[0] + (cur_pos_x - dict_info.offset_x);
                int ref_pos_c = ((cur_pos_y - dict_info.offset_y) >> scale_y) * m_stride_rec[1] + ((cur_pos_x - dict_info.offset_x) >> scale_x);
#endif
                int cur_blk_pos = trav_y * width + trav_x;
                int cur_blk_pos_c = (trav_y >> scale_y)*(width >> scale_x) + (trav_x >> scale_x);
                p_cur_sp_info->rec[0][cur_blk_pos] = m_p_pel_rec[0][ref_pos];
                m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
                if (!((cur_pos_x & scale_x) || (cur_pos_y & scale_y)))
                {
                    m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                    m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
                    p_cur_sp_info->rec[1][cur_blk_pos_c] = m_p_pel_rec[1][ref_pos_c];
                    p_cur_sp_info->rec[2][cur_blk_pos_c] = m_p_pel_rec[2][ref_pos_c];
                }
                int curPosOrg = cur_pos_y * m_stride_org[0] + cur_pos_x;
                int curPosCOrg = (cur_pos_y >> scale_y) * m_stride_org[1] + (cur_pos_x >> scale_x);
                int diff[3] = { 0 };
                diff[0] = abs(m_p_pel_org[0][curPosOrg] - former_pix[0]);
                diff[1] = abs(m_p_pel_org[1][curPosCOrg] - former_pix[1]);
                diff[2] = abs(m_p_pel_org[2][curPosCOrg] - former_pix[2]);
                replicate_num = (diff[0] + diff[1] + diff[2] <= 8) ? replicate_num + 1 : replicate_num;
                same_num = (m_p_pel_org[0][curPosOrg] == former_pix[0]) ? same_num + 1 : same_num;
                former_pix[0] = m_p_pel_org[0][curPosOrg];
                former_pix[1] = m_p_pel_org[1][curPosCOrg];
                former_pix[2] = m_p_pel_org[2][curPosCOrg];
            }
            //add to hash 
            add_pixel_to_hash_table(max_match_length, TRUE, x_start_in_pic, y_start_in_pic, cu_width_log2, cu_height_log2, processed_count, is_hor_scan);
            vecDictInfo.push_back(dict_info);
            processed_count += max_match_length;
        }
        else
        {
            COM_SP_INFO dict_info;
            tmp_dict_rdcost = 0;
            if (p_cur_sp_info->tree_status != TREE_C)
            {
                tmp_dict_rdcost += bit_depth * (p_cur_sp_info->lamda);
            }
            if (p_cur_sp_info->tree_status != TREE_L)
            {
                if (input->chroma_format <= 1 && !(current_x & 0x1 || current_y & 0x1))
                {
                    tmp_dict_rdcost += 2 * bit_depth * (p_cur_sp_info->lamda);
                }    
            }
            tmp_dict_dist_y = 0;
            tmp_dict_dist_uv = 0;
            dict_info.is_matched = FALSE;
            dict_info.length = 1;
            dict_info.pixel[Y_C] = m_p_pel_org[0][current_y*m_stride_org[0] + current_x];
            dict_info.pixel[U_C] = m_p_pel_org[1][(current_y >> scale_y)*m_stride_org[1] + (current_x >> scale_x)];
            dict_info.pixel[V_C] = m_p_pel_org[2][(current_y >> scale_y)*m_stride_org[2] + (current_x >> scale_x)];
            if (dict_info.length == -1)
                dict_info.length = 1;
            // reconstruction          
            pel ydata = dict_info.pixel[Y_C];
            pel udata = dict_info.pixel[U_C];
            pel vdata = dict_info.pixel[V_C];
            for (int pixel_count = 0; pixel_count < dict_info.length; pixel_count++)
            {
                int cur_pos_x;
                int cur_pos_y;
                int cur_pixel_pos = p_trav_scan_order[processed_count + pixel_count];
                trav_x = GET_TRAV_X(cur_pixel_pos, width);
                trav_y = GET_TRAV_Y(cur_pixel_pos, p_cur_sp_info->cu_width_log2);
                cur_pos_x = x_start_in_pic + trav_x;
                cur_pos_y = y_start_in_pic + trav_y;
                int cur_pos = cur_pos_y * m_stride_rec[0] + cur_pos_x;
                int cur_pos_c = (cur_pos_y >> scale_y) * m_stride_rec[1] + (cur_pos_x >> scale_x);
                m_p_pel_rec[0][cur_pos] = ydata;
                int cur_blk_pos = trav_y * width + trav_x;
                int cur_blk_pos_c = (trav_y >> scale_y)*(width >> scale_x) + (trav_x >> scale_x);
                p_cur_sp_info->rec[0][cur_blk_pos] = ydata;
                if (!((cur_pos_x & scale_x) || (cur_pos_y & scale_y)))
                {
                    m_p_pel_rec[1][cur_pos_c] = udata;
                    m_p_pel_rec[2][cur_pos_c] = vdata;
                    p_cur_sp_info->rec[1][cur_blk_pos_c] = udata;
                    p_cur_sp_info->rec[2][cur_blk_pos_c] = vdata;
                }
            }
            //add to hash
            add_pixel_to_hash_table(dict_info.length, TRUE, x_start_in_pic, y_start_in_pic, cu_width_log2, cu_height_log2, processed_count, is_hor_scan);
            vecDictInfo.push_back(dict_info);
            processed_count += dict_info.length;
        }
        dict_dist_uv += tmp_dict_dist_uv;
        dict_dist_y += tmp_dict_dist_y;
        dict_rdcost += tmp_dict_rdcost;
        if (replicate_num < processed_count >> 2 && processed_count > 256 && width >= 32 && height >= 32)
        {
            if (same_num < (processed_count >> 3))
            {
                p_cur_sp_info->is_sp_skip_non_scc = 1;
            }
            return FALSE;
        }
        if (dict_rdcost > *curBestRDCost)
        {
            return FALSE;
        }
        if (vecDictInfo.size() > p_cur_sp_info->max_str_cnt)
            return FALSE;
    } //endwhile
    assert(processed_count == count);
    if (processed_count == count)
    {
        p_cur_sp_info->is_sp_pix_completed = TRUE;
    }
    if (p_cur_sp_info->tree_status == TREE_LC)
    {
        *distortion = dict_dist_y + dict_dist_uv * p_cur_sp_info->chroma_weight[0];
        *curBestRDCost = dict_rdcost;
    }
    else
    {
        assert(p_cur_sp_info->tree_status == TREE_L);
        *distortion = dict_dist_y;
        *curBestRDCost = dict_rdcost;
    }
    return TRUE;
}

u8 string_prediction::encode_cu_size_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* cur_best_rdcost, double* distortion)
{
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    COM_SP_POS best_sp[2];
    COM_SP_POS temp_sp;
    u32 ui_temp_distortion = 0;
    u32 ui_temp_distortion_uv = 0;
    int cu_pel_x, cu_pel_y;
    pel *org_pic;
    pel *ref_pic;
    int sp_roi_width, sp_roi_height;
    int bitDepth = input->sample_bit_depth;
    u8 shift = (bitDepth == 8) ? 0 : (bitDepth - 8) << 1;
    sp_roi_width = width; 
    sp_roi_height = height;     
    int numOfStrings = 1; //only one cu_size_string is searched
    for (int nos = 0; nos < numOfStrings; nos++)
    {
        cu_pel_x = x_start_in_pic;
        cu_pel_y = y_start_in_pic;
        cu_size_str_search(p_cur_sp_info, x_start_in_pic, y_start_in_pic, temp_sp);
        best_sp[nos].x = temp_sp.x;
        best_sp[nos].y = temp_sp.y;
        if (best_sp[nos].x == 0 && best_sp[nos].y == 0)
            return 0;
        else
        {
            for (u8 ch = 0; ch < 3; ch++)
            {
                org_pic = m_p_pel_org[ch];
                ref_pic = m_p_pel_rec[ch];
                u8 scale_x = (ch == 0 ? 0 : 1);
                u8 scale_y = (ch == 0 ? 0 : 1);
                int org_pos = (cu_pel_y >> scale_y) * m_stride_org[ch] + (cu_pel_x >> scale_x);
#if SP_ALIGN_SIGN
                int ref_pos = ((cu_pel_y + best_sp[nos].y) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x + best_sp[nos].x) >> scale_x);
#else
                int ref_pos = ((cu_pel_y - best_sp[nos].y) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x - best_sp[nos].x) >> scale_x);
#endif
                int recStride = 0;
                for (int row = 0; row < (sp_roi_height >> scale_y); row++)
                {
                    for (int col = 0; col < (sp_roi_width >> scale_x); col++)
                    {
                        int cu_ref_pos = ref_pos + col;
                        int cu_org_pos = org_pos + col;
                        if (ch == 0)
                        {
                            ui_temp_distortion += ((ref_pic[cu_ref_pos] - org_pic[cu_org_pos])*(ref_pic[cu_ref_pos] - org_pic[cu_org_pos]));
                        }
                        else
                        {
                            ui_temp_distortion_uv += ((ref_pic[cu_ref_pos] - org_pic[cu_org_pos])*(ref_pic[cu_ref_pos] - org_pic[cu_org_pos]));
                        }
                        p_cur_sp_info->rec[ch][recStride + col] = m_p_pel_rec[ch][cu_ref_pos];
                    }
                    org_pos += m_stride_org[ch];
                    ref_pos += m_stride_rec[ch];
                    recStride += (sp_roi_width >> scale_x);
                }
            }
        }
    } //endfor nos
    if ((best_sp[0].x == 0) && (best_sp[0].y == 0))
    {
        return 0;
    }
    double cu_size_str_rdcost, cu_size_str_distortion;
    if (p_cur_sp_info->tree_status == TREE_LC)
    {
        cu_size_str_distortion = (ui_temp_distortion >> shift) + (ui_temp_distortion_uv >> shift) * p_cur_sp_info->chroma_weight[0];
        cu_size_str_rdcost = get_sp_offset_bit_cnt(best_sp[0].x, best_sp[0].y)*p_cur_sp_info->lamda + cu_size_str_distortion;
    }
    else
    {
        assert(p_cur_sp_info->tree_status == TREE_L);
        cu_size_str_distortion = ui_temp_distortion;
        cu_size_str_rdcost = get_sp_offset_bit_cnt(best_sp[0].x, best_sp[0].y)*p_cur_sp_info->lamda + cu_size_str_distortion;
    }
    if (cu_size_str_rdcost > *cur_best_rdcost)
    {
        return FALSE;
    }
    else
    {
        *distortion = cu_size_str_distortion;
        *cur_best_rdcost = cu_size_str_rdcost;
    }
    COM_SP_INFO dict_info;
    dict_info.is_matched = TRUE;
    dict_info.offset_x = best_sp[0].x;
    dict_info.offset_y = best_sp[0].y;
    dict_info.length = (u16)width * height;
    dict_info.sp_length_type = LAST_STR;
    dict_info.n_recent_flag = 0;
#if SP_ENC_OPT
    int index = check_sp_offset(dict_info.offset_x, dict_info.offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
    if (index != p_cur_sp_info->n_cand_num)
    {
        dict_info.n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
        dict_info.n_recent_flag = 1;
    }
#endif
    vecDictInfo.push_back(dict_info);
    p_cur_sp_info->is_sp_pix_completed = 1;
    return 1;
}

void string_prediction::cu_size_str_search(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp)
{
    //set search range
    int srLeft, srRight, srTop, srBottom;
    int cu_pel_x, cu_pel_y;
    int sp_roi_width, sp_roi_height;
    SP_CU_SIZE_STRING  best_coding_info;
    best_coding_info.sm.x = 0;
    best_coding_info.sm.y = 0;
    COM_SP_POS best_sp;
    const int lcu_width = m_max_cu_width;
    const int lcu_height = m_max_cu_height;
    const int i_pic_width = m_pic_width;
    const int i_pic_height = m_pic_height;
    pel* ref_pic = m_p_pel_rec[0];
    pel* org_pic = m_p_pel_org[0];
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int i_best_cand_idx = 0;
    sp_roi_width = width;
    sp_roi_height = height;
    cu_pel_x = x_start_in_pic;
    cu_pel_y = y_start_in_pic;
    org_pic = m_p_pel_org[0];
    ref_pic = m_p_pel_rec[0];
    const int iRelCUPelX = cu_pel_x & (lcu_width - 1);
    const int iRelCUPelY = cu_pel_y & (lcu_height - 1);
    int maxXsr = iRelCUPelX + get_sp_search_area_width(x_start_in_pic, 1);
    int maxYsr = iRelCUPelY;
    if ((input->chroma_format == 0) || (input->chroma_format == 2)) maxXsr &= ~0x4;
    if ((input->chroma_format == 0)) maxYsr &= ~0x4;
    srLeft = -maxXsr;
    srTop = -maxYsr;
    srRight = lcu_width - iRelCUPelX - sp_roi_width;
    srBottom = lcu_height - iRelCUPelY - sp_roi_height;
    if ((int)cu_pel_x + srRight + sp_roi_width > i_pic_width)
    {
        srRight = i_pic_width % lcu_width - iRelCUPelX - sp_roi_width;
    }
    if ((int)cu_pel_y + srBottom + sp_roi_height > i_pic_height)
    {
        srBottom = i_pic_height % lcu_height - iRelCUPelY - sp_roi_height;
    }
    //end search range
    //Do integer search    
    u32  uiSad;
    u32  uiSadBestCand[SP_CHROMA_REFINE_CANDS];
    SP_CU_SIZE_STRING      cSMPositionCand[SP_CHROMA_REFINE_CANDS];
    u32  uiSadBest = 0xFFFFFFFFU;
    u32  uiTempSadBest = 0;
    const int boundY = (0 - sp_roi_height);
    for (int iCand = 0; iCand < SP_CHROMA_REFINE_CANDS; iCand++)
    {
        uiSadBestCand[iCand] = 0xFFFFFFFFU;
        cSMPositionCand[iCand].sm.x = 0;
        cSMPositionCand[iCand].sm.y = 0;
    }
    int lowY;
#if SP_ENC_OPT
    // svp search
    for (int i = 0; i < p_cur_sp_info->n_cand_num;i++)
    {
#if SP_ALIGN_SIGN
        int x = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_X];
        int y = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_Y];
#else
        int x = -p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_X];
        int y = -p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_Y];
#endif
        if (cu_pel_y + y < 0 || cu_pel_y + sp_roi_height + y >= m_pic_height || cu_pel_x + x < 0 || cu_pel_x + sp_roi_width + x >= m_pic_width)
        {
            continue;
        }
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
        uiSad = (u32)get_tb_bits(p_cur_sp_info->n_cand_num - 1 - i, p_cur_sp_info->n_cand_num);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (cu_pel_y + y + r) * m_stride_rec[0] + cu_pel_x + x;
            int cu_org_pos = (cu_pel_y + r) * m_stride_org[0] + cu_pel_x;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
#if SP_ALIGN_SIGN
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
#else
        sp_postion_cand_update(uiSad, -x, -y, uiSadBestCand, cSMPositionCand);
#endif
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
#endif
    // ver 1D search
    lowY = (srTop >= (int)(0 - cu_pel_y)) ? srTop : (0 - cu_pel_y);
    for (int y = boundY; y >= lowY; y--)
    {
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
        uiSad = (u32)get_sp_offset_bit_cnt(0, -y);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (y + r + cu_pel_y) * m_stride_rec[0] + cu_pel_x;
            int cu_org_pos = cu_pel_x + (r + cu_pel_y) * m_stride_org[0];
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
#if SP_ALIGN_SIGN
        sp_postion_cand_update(uiSad, 0, y, uiSadBestCand, cSMPositionCand);
#else
        sp_postion_cand_update(uiSad, 0, -y, uiSadBestCand, cSMPositionCand);
#endif
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
    // hor 1D search
    {
        const int boundX = (srLeft >= (int)(0 - cu_pel_x)) ? srLeft : (0 - cu_pel_x);
        for (int x = 0 - sp_roi_width; x >= boundX; --x)
        {
            if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
            {
                continue;
            }
            uiSad = (u32)get_sp_offset_bit_cnt(-x, 0);
            for (int r = 0; r < sp_roi_height; )
            {
                int cu_ref_pos = (r + cu_pel_y)* m_stride_rec[0] + x + cu_pel_x;
                int cu_org_pos = (r + cu_pel_y)* m_stride_org[0] + cu_pel_x;
                for (int j = 0; j < 4; j++)
                {
                    for (int i = 0; i < sp_roi_width; i++)
                    {
                        uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                    }
                    cu_ref_pos += m_stride_rec[0];
                    cu_org_pos += m_stride_org[0];
                }
                if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
                {
                    break;
                }
                r += 4;
            }
#if SP_ALIGN_SIGN
            sp_postion_cand_update(uiSad, x, 0, uiSadBestCand, cSMPositionCand);
#else
            sp_postion_cand_update(uiSad, -x, 0, uiSadBestCand, cSMPositionCand);
#endif
            uiTempSadBest = uiSadBestCand[0];
            if (uiSadBestCand[0] <= 3)
            {
                copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
                uiSadBest = uiSadBestCand[0];
                goto rec;
            }
        }
    }
    copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
    best_sp.x = best_coding_info.sm.x;
    best_sp.y = best_coding_info.sm.y;
    uiSadBest = uiSadBestCand[0];
    i_best_cand_idx = 0;
    if ((!best_sp.x && !best_sp.y) || (uiSadBest - (u32)get_sp_offset_bit_cnt(best_sp.x, best_sp.y) <= 32))
    {
        //chroma refine    
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            i_best_cand_idx = sp_postion_chroma_refine(sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y, uiSadBestCand, cSMPositionCand);
        }
        else
        {
            assert(p_cur_sp_info->tree_status == TREE_L);
        }
        copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[i_best_cand_idx]));
        uiSadBest = uiSadBestCand[i_best_cand_idx];
        goto rec;
    }
rec:
    sp.x = best_coding_info.sm.x;
    sp.y = best_coding_info.sm.y;
}

void string_prediction::sp_postion_cand_update(u32  uiSad, int x, int y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand)
{
    int j = SP_CHROMA_REFINE_CANDS - 1;
    if (uiSad < uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
    {
        for (int t = SP_CHROMA_REFINE_CANDS - 1; t >= 0; t--)
        {
            if (uiSad < uiSadBestCand[t])
                j = t;
        }
        for (int k = SP_CHROMA_REFINE_CANDS - 1; k > j; k--)
        {
            uiSadBestCand[k] = uiSadBestCand[k - 1];
            cSMPositionCand[k].sm.x = cSMPositionCand[k - 1].sm.x;
            cSMPositionCand[k].sm.y = cSMPositionCand[k - 1].sm.y;
        }
        uiSadBestCand[j] = uiSad;
        cSMPositionCand[j].sm.x = x;
        cSMPositionCand[j].sm.y = y;
    }
}

int string_prediction::sp_postion_chroma_refine(
    int         sp_roi_width,
    int         sp_roi_height,
    int         cu_pel_x,
    int         cu_pel_y,
    u32* uiSadBestCand,
    SP_CU_SIZE_STRING*     cSMPositionCand
)
{
    int iBestCandIdx = 0;
    u32 uiSadBest = 0xFFFFFFFFU;
    pel* ref_pic = m_p_pel_rec[0];
    pel* org_pic = m_p_pel_org[0];
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1;
    for (int iCand = 0; iCand < SP_CHROMA_REFINE_CANDS; iCand++)
    {
        u32 uiTempSad = uiSadBestCand[iCand];
        SP_CU_SIZE_STRING uiTempSMPosition = cSMPositionCand[iCand];
        for (u8 ch = 1; ch < 2; ch++)
        {
            int orgPos = (cu_pel_y >> scale_y) * m_stride_org[ch] + (cu_pel_x >> scale_x);
#if SP_ALIGN_SIGN
            int ref_pos = ((cu_pel_y + (uiTempSMPosition.sm.y)) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x + (uiTempSMPosition.sm.x)) >> scale_x);
#else
            int ref_pos = ((cu_pel_y - (uiTempSMPosition.sm.y)) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x - (uiTempSMPosition.sm.x)) >> scale_x);
#endif
            org_pic = m_p_pel_org[ch];
            ref_pic = m_p_pel_rec[ch];
            for (int row = 0; row < (sp_roi_height >> scale_y); row++)
            {
                for (int col = 0; col < (sp_roi_width >> scale_x); col++)
                {
                    uiTempSad += abs(ref_pic[ref_pos + col] - org_pic[orgPos + col]);
                }
                orgPos += m_stride_org[ch];
                ref_pos += m_stride_rec[ch];
            }
        }
        if (uiTempSad < uiSadBest)
        {
            uiSadBest = uiTempSad;
            iBestCandIdx = iCand;
        }
    }
    return iBestCandIdx;
}
#endif
