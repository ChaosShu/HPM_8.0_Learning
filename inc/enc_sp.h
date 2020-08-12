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

#ifndef ENC_SM_H
#define ENC_SM_H

#include "com_usp.h"
#if USE_SP
#undef max
#undef min
#include <vector>
#include <math.h>

using namespace std;

#define SP_MAX_COST                       1.7e+308
#define SP_CHROMA_REFINE_CANDS            8
#define MAX_SM_TEST_CASE                  1000
#define SP_HASH_SIZE                      (1<<12)

extern "C" void* get_sp_instance(COM_SP_INPUT* input,  u8 isEncoding);
extern "C" void  release_sp_instance(void* sp_encoder);
extern "C" void  sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
extern "C" u8    sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double*  min_rdcost, double*  distorion);
extern "C" void  sp_lcu_hash_save(void* sp_encoder, int x_lcu_start, int y_lcu_start);
extern "C" void  sp_lcu_hash_restore(void* sp_encoder, int x_lcu_start, int y_lcu_start);
extern "C" void  sp_cu_hash_copy(void* sp_encoder, int dst_w, int dst_h, int dst_status, int src_w, int src_h, int src_status);
extern "C" u8    sp_cu_hash_update(void* sp_encoder, int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start);
extern "C" u8    sp_is_hash_updated(u8 *map_usp, int x, int y, int cu_width, int cu_height, int pic_width_in_scu);
extern "C" u8    get_adaptive_sp_flag(void* sp_encoder);

struct SP_HASH_STATUS
{
    COM_SP_POS    m_hash_table[SP_HASH_SIZE];
    COM_SP_POS    m_dict[MAX_CU_SIZE][MAX_CU_SIZE];
    void          save(SP_HASH_STATUS* p);
};

typedef struct SP_CU_SIZE_STRING
{
    COM_SP_POS sm;
}SP_CU_SIZE_STRING;

class string_prediction
{
public:
    string_prediction(COM_SP_INPUT* input, u8 isEncoding);
    ~string_prediction();

private:
    string_prediction( const string_prediction& com_dict );
public:
    u8     rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion);
    u8     check_rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion);
    void   reset(pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
    int    get_sp_search_area_width(int x_start_in_pic, int uiMaxSearchWidthToLeftInCTUs);
    void   sp_postion_cand_update(u32  uiSad, int x, int y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand);
    int    sp_postion_chroma_refine(int sp_roi_width, int sp_roi_height, int cu_pel_x, int cu_pel_y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand);
    void   add_to_hash_table(int x_start_in_pic, int y_start_in_pic, int width, int height, u8 scale_x, u8 scale_y);
    u16    get_sp_hash_value_lossy(COM_SP_PIX pixelCur);
    u16(string_prediction::*get_sp_hashvalue)(COM_SP_PIX);
    void   save(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start, int width, int height);
    void   restore(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start, int width, int height);
    u8     sp_hash_update(int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start);
    u8     sp_hash_sp_judge(int x_start_in_pic, int y_start_in_pic);
    void   cu_size_str_search(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp);
    u8     encode_cu_size_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion);
    void   add_pixel_to_hash_table(int count, u8 includeHash, int x_start_in_pic, int y_start_in_pic, int cu_width_log2, int cu_height_log2, int iStartIdx, u8 bHorizontal = TRUE);
    int    get_tb_bits(int ui_symbol, int uiMaxSymbol);
    int    get_len_bits_cnt(int len, int totalPixel);
    int    get_offset_bits(int ui_symbol, int uiCount);
    int    check_str_len_bit(int len, int w, int h, int totalLeftPixel, u8 dir, u8 *len_type);
    double get_hor_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
        , int match_length, int offset_sign, int leftPixelInCU, int *bit_cnt
        , u8 *len_type, COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag);
    double get_ver_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
        , int match_length, int offset_sign, int leftPixelInCU, int *bit_cnt
        , u8 *len_type, COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag);
    void   get_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length, int processed_count, u8 scale_x, u8 scale_y, int uiShiftY, int uiShiftC, u32 *prbDistY, u32 *prbDistUV);
    void   set_org_block(int uiWidth, int uiHeight, int x_start_in_pic, int y_start_in_pic);
    void   init_sps_cands(COM_MOTION *cand, int cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num);
    void   refine_sps_cands(vector<COM_MOTION>& org_sps_cands, vector<COM_MOTION>& refined_sps_cands, int current_x, int current_y);
    u8     encode_general_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion);

    __inline void copy_cu_size_str_info(SP_CU_SIZE_STRING *dst, SP_CU_SIZE_STRING *src) 
    {
        memcpy(dst, src, sizeof(SP_CU_SIZE_STRING));
    }

    __inline int get_eg_bits(int act_sym, int exp_golomb_order)
    {
        int uiLength = 0;
        while (1)
        {
            if (act_sym >= (1 << exp_golomb_order))
            {
                uiLength++;
                act_sym = act_sym - (1 << exp_golomb_order);
                exp_golomb_order++;
            }
            else
            {
                uiLength++;
                while (exp_golomb_order--)
                {
                    uiLength++;
                }
                break;
            }
        }
        return uiLength;
    }

    __inline int get_sp_offset_bit_cnt(int offset_x, int offset_y)
    {
        int bits = 0;
        int uiHorAbs = 0 > offset_x ? -offset_x : offset_x;
        int uiVerAbs = 0 > offset_y ? -offset_y : offset_y;
        bits += get_eg_bits(uiHorAbs - 1, 0);
        bits += get_eg_bits(uiVerAbs - 1, 0);
        return bits;
    }

    __inline u8 is_ref_region_coded(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int sp_roi_width, int sp_roi_height) 
    {
        int pic_width_in_scu = p_cur_sp_info->pic_width_in_scu;      
        u32 *map_scu = p_cur_sp_info->map_scu;
        int upLeftOffset = (ref_Y_LT >> MIN_CU_LOG2)*pic_width_in_scu + (ref_X_LT >> MIN_CU_LOG2);
        int bottomRightOffset = ((ref_Y_LT + sp_roi_height - 1) >> MIN_CU_LOG2)*pic_width_in_scu + ((ref_X_LT + sp_roi_width - 1) >> MIN_CU_LOG2);
        if (MCU_GET_CODED_FLAG(map_scu[bottomRightOffset]) && MCU_GET_CODED_FLAG(map_scu[upLeftOffset])) 
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }

    __inline u8 is_ref_pix_coded(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT) 
    {
        int pic_width_in_scu = p_cur_sp_info->pic_width_in_scu;
        u32 *map_scu = p_cur_sp_info->map_scu;
        int upLeftOffset = (ref_Y_LT >> MIN_CU_LOG2) * pic_width_in_scu + (ref_X_LT >> MIN_CU_LOG2);
        if (MCU_GET_CODED_FLAG(map_scu[upLeftOffset]))
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }

    //One-CTU mem reuse
    __inline u8 is_ref_region_valid(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int sp_roi_width, int sp_roi_height,int cu_pel_x, int cu_pel_y) 
    {
        int offset64x = ((ref_X_LT + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1)) //in the left LCU but already updated
        {
            if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) // already coded or just current cu
            {
                return 0;
            }
        }
        if (!is_ref_region_coded(p_cur_sp_info, ref_X_LT, ref_Y_LT, sp_roi_width, sp_roi_height)) 
        { 
            return 0;
        }

        int offset64x_br = ((ref_X_LT + sp_roi_width - 1 + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        int offset64y_br = ((ref_Y_LT + sp_roi_height - 1) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (offset64x != offset64x_br || offset64y != offset64y_br)
        {
            return 0;
        }
        return 1;
    }

    __inline u8 is_ref_pix_valid(int* puiReverseOrder, COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int processed_count) 
    {
        int cu_width_log2 = p_cur_sp_info->cu_width_log2;
        int cu_height_log2 = p_cur_sp_info->cu_height_log2;
        if (is_ref_pix_coded(p_cur_sp_info,ref_X_LT,ref_Y_LT))  // in the coded cu
        {
            return 1;
        }
        else if ((ref_X_LT >= p_cur_sp_info->cu_pix_x &&ref_X_LT < p_cur_sp_info->cu_pix_x + (1 << cu_width_log2)) && (ref_Y_LT >= p_cur_sp_info->cu_pix_y &&ref_Y_LT < p_cur_sp_info->cu_pix_y + (1 << cu_height_log2))) //in the same cu
        {
            int trav_x = ref_X_LT - p_cur_sp_info->cu_pix_x;
            int trav_y = ref_Y_LT - p_cur_sp_info->cu_pix_y;
            int RefPointOrder = puiReverseOrder[trav_y * (1 << cu_width_log2) + trav_x];
            if (RefPointOrder < processed_count)
                return 1;
            else
                return 0;
        }
        else 
        {
            return 0;
        }
    }

    __inline u8 is_ref_pix_in_one_ctu(int* puiReverseOrder, COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int processed_count,int fir_refx, int fir_refy)
    {
        //judge if SP reference point is in One-CTU
        int cu_pel_x = p_cur_sp_info->cu_pix_x;
        int cu_pel_y = p_cur_sp_info->cu_pix_y;
        int sp_roi_width = 1 << p_cur_sp_info->cu_width_log2;
        int sp_roi_height = 1 << p_cur_sp_info->cu_height_log2;
        int offset64x = ((ref_X_LT + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1)) //in the left LCU but already updated
        {
            if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) //not in the mem reuse area
            {
                return 0;
            }
        }
        if (!is_ref_pix_valid(puiReverseOrder, p_cur_sp_info, ref_X_LT, ref_Y_LT, processed_count))
        {
            return 0;
        }
        int fir_ref_64x = ((fir_refx + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        int fir_ref_64y = (fir_refy >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (offset64x != fir_ref_64x || offset64y != fir_ref_64y)
        {
            return 0;
        }
        return 1;
    }

    __inline u8 is_pix_in_errorlimit(int trav_order_index, int ref_pos, int ref_pos_c, int errorLimit, u8 tree_status)
    {
        if (tree_status == TREE_LC)
        {
            if (abs(m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_org[0][ref_pos]) < errorLimit &&
                abs(m_p_org_pixel_buffer[trav_order_index].U - m_p_pel_org[1][ref_pos_c]) < errorLimit &&
                abs(m_p_org_pixel_buffer[trav_order_index].V - m_p_pel_org[2][ref_pos_c]) < errorLimit)
            {
                return TRUE;
            }
            else
            {
                return FALSE;
            }
        }
        else 
        {
            if (abs(m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_org[0][ref_pos]) < errorLimit)
            {
                return TRUE;
            }
            else
            {
                return FALSE;
            }
        }
    }
public:
    COM_SP_INPUT       iptemp;
    COM_SP_INPUT      *input;
    SP_HASH_STATUS ****m_ppp_dict_status;
    int                m_pic_width;
    int                m_pic_height;
private:
    int                m_max_cu_width;
    int                m_max_cu_height;
    u8                 m_is_encoding;
    pel               *m_p_pel_org[3];
    pel               *m_p_pel_rec[3];
    int                m_stride_org[3];
    int                m_stride_rec[3];
    COM_SP_PIX         m_p_org_pixel_buffer[MAX_CU_DIM];
    COM_SP_POS         m_hash_table[SP_HASH_SIZE];
    COM_SP_POS       **m_dict;
};

u8 sp_max_error_quant[60] = { 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 21, 22, 24, 23, 25, 26, 28, 29, 31, 32, 34, 36, 37, 39, 41, 42, 45, 46, 47, 48, 49, 50, 51, 52, 53 };
#endif
#endif //_ENC_SM_H