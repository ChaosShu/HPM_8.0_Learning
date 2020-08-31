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
#if USE_SP    
#include "com_usp.h"
extern void* get_sp_instance(COM_SP_INPUT* input, u8 isEncoding);
extern void  sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
extern u8    sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double*  min_rdcost, double*  distorion);
#endif
#include <math.h>

int pintra_init_frame(ENC_CTX * ctx)
{
    ENC_PINTRA * pi;
    COM_PIC     * pic;
    pi = &ctx->pintra;
    pic = PIC_ORG(ctx);
    pi->addr_org[Y_C] = pic->y;
    pi->addr_org[U_C] = pic->u;
    pi->addr_org[V_C] = pic->v;
    pi->stride_org[Y_C] = pic->stride_luma;
    pi->stride_org[U_C] = pic->stride_chroma;
    pi->stride_org[V_C] = pic->stride_chroma;

    pic = PIC_REC(ctx);
    pi->addr_rec_pic[Y_C] = pic->y;
    pi->addr_rec_pic[U_C] = pic->u;
    pi->addr_rec_pic[V_C] = pic->v;
    pi->stride_rec[Y_C] = pic->stride_luma;
    pi->stride_rec[U_C] = pic->stride_chroma;
    pi->stride_rec[V_C] = pic->stride_chroma;
    pi->slice_type = ctx->slice_type;
    pi->bit_depth = ctx->info.bit_depth_internal;

#if USE_SP 
    COM_SP_INPUT sp_input_pmt;
    sp_input_pmt.chroma_format = (&ctx->param)->chroma_format;
    sp_input_pmt.img_height    = (&ctx->param)->pic_height;
    sp_input_pmt.img_width     = (&ctx->param)->pic_width;
    sp_input_pmt.sample_bit_depth = ctx->info.bit_depth_internal;
    sp_input_pmt.max_cu_height = MAX_CU_SIZE;
    sp_input_pmt.max_cu_width  = MAX_CU_SIZE;
    sp_input_pmt.y_stride      = pi->stride_org[Y_C];
    sp_input_pmt.recy_stride   = pi->stride_rec[Y_C];
    sp_input_pmt.c_stride      = pi->stride_org[U_C];
    sp_input_pmt.recc_stride   = pi->stride_rec[U_C];
    ctx->sp_encoder = get_sp_instance(&sp_input_pmt, TRUE);
    sm_pic_reset(ctx->sp_encoder, pi->addr_org[Y_C], pi->addr_org[U_C], pi->addr_org[V_C], pi->addr_rec_pic[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C]);
#endif
    return COM_OK;
}

int pintra_init_lcu(ENC_CTX * ctx, ENC_CORE * core)
{
    return COM_OK;
}

//Note: this is PB-based RDO
static double pintra_residue_rdo(ENC_CTX *ctx, ENC_CORE *core, pel *org_luma, pel *org_cb, pel *org_cr, int s_org, int s_org_c, int cu_width_log2, int cu_height_log2,
                                 s32 *dist, int bChroma, int pb_idx, int x, int y)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_PINTRA *pi = &ctx->pintra;
    int bit_depth = ctx->info.bit_depth_internal;
    s16 coef_tmp[N_C][MAX_CU_DIM];
    s16 resi[MAX_CU_DIM];
    int cu_width, cu_height, bit_cnt;
    double cost = 0;
    u16 avail_tb;
    int s_mod = pi->stride_rec[Y_C];
    pel* mod;
    int num_nz_temp[MAX_NUM_TB][N_C];
#if IST
    int coef_restrict = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif

    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
    if (!bChroma)
    {
        int pb_part_size = mod_info_curr->pb_part;
        int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_idx);
        int pb_w = mod_info_curr->pb_info.sub_w[pb_idx];
        int pb_h = mod_info_curr->pb_info.sub_h[pb_idx];
        int pb_x = mod_info_curr->pb_info.sub_x[pb_idx];
        int pb_y = mod_info_curr->pb_info.sub_y[pb_idx];
        int tb_w, tb_h, tb_x, tb_y, tb_scup, tb_x_scu, tb_y_scu, coef_offset_tb;
        pel* pred_tb;
        cu_plane_nz_cln(mod_info_curr->num_nz, Y_C);

        get_tb_width_height_in_pb(pb_w, pb_h, pb_part_size, pb_idx, &tb_w, &tb_h);//根据PU确定当前TU宽高
        for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)//check每个TU
        {
            //derive tu basic info
            get_tb_pos_in_pb(pb_x, pb_y, pb_part_size, tb_w, tb_h, tb_idx, &tb_x, &tb_y);//根据PU确定TU起始位置
            coef_offset_tb = tb_idx * tb_w * tb_h;
            tb_x_scu = PEL2SCU(tb_x);
            tb_y_scu = PEL2SCU(tb_y);
            tb_scup = tb_x_scu + (tb_y_scu * ctx->info.pic_width_in_scu);
            //get start of tb residual
            pel* resi_tb = resi + coef_offset_tb; //residual is stored sequentially, not in raster, like coef
            pel* rec_tb = pi->rec[Y_C] + (tb_y - mod_info_curr->y_pos) * cu_width + (tb_x - mod_info_curr->x_pos);//rec中tb的起始位置

            avail_tb = com_get_avail_intra(tb_x_scu, tb_y_scu, ctx->info.pic_width_in_scu, tb_scup, ctx->map.map_scu);

            s8 intra_mode = mod_info_curr->ipm[pb_idx][0];
            int tb_width_log2 = com_tbl_log2[tb_w];
            int tb_height_log2 = com_tbl_log2[tb_h];
            assert(tb_width_log2 > 0 && tb_height_log2 > 0);
#if EST
            int use_secTrans = (ctx->info.sqh.est_enable_flag ? mod_info_curr->est_flag : ctx->info.sqh.secondary_transform_enable_flag) &&
                               (tb_width_log2 > 2 || tb_height_log2 > 2);
            int use_alt4x4Trans = (ctx->info.sqh.est_enable_flag ? mod_info_curr->est_flag : ctx->info.sqh.secondary_transform_enable_flag);
#else
            int use_secTrans = ctx->info.sqh.secondary_transform_enable_flag && (tb_width_log2 > 2 || tb_height_log2 > 2);
            int use_alt4x4Trans = ctx->info.sqh.secondary_transform_enable_flag;
#endif
            int secT_Ver_Hor = 0;
            if (use_secTrans)
            {
                int vt, ht;
                int block_available_up = IS_AVAIL(avail_tb, AVAIL_UP);
                int block_available_left = IS_AVAIL(avail_tb, AVAIL_LE);
#if EIPM
                vt = (intra_mode < IPD_HOR) || (intra_mode >= IPD_DIA_L_EXT && intra_mode < IPD_HOR_EXT);
                ht = (intra_mode > IPD_VER && intra_mode < IPD_IPCM) || (intra_mode > IPD_VER_EXT && intra_mode < IPD_CNT) || (intra_mode <= IPD_BI);
#else
                vt = intra_mode < IPD_HOR;
                ht = (intra_mode > IPD_VER && intra_mode < IPD_CNT) || intra_mode <= IPD_BI;
#endif
                vt = vt && block_available_up;
                ht = ht && block_available_left;
                secT_Ver_Hor = (vt << 1) | ht;
#if EST
                if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0)
                {
                    secT_Ver_Hor = 3;
                }
#endif
            }

            //prediction according to the input ipm
            if (num_tb_in_pb == 1)
                pred_tb = pi->pred_cache[intra_mode];//PU内只有一个TU时，直接等于
            else
            {//有DT时
                mod = pi->addr_rec_pic[Y_C] + (tb_y * s_mod) + tb_x;//s_mod CTU级stride?
                pred_tb = mod_info_curr->pred[Y_C]; // pred is temp memory
                com_get_nbr(tb_x, tb_y, tb_w, tb_h, mod, s_mod, avail_tb, core->nb, tb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
                com_ipred(core->nb[0][0] + 3, core->nb[0][1] + 3, pred_tb, intra_mode, tb_w, tb_h, bit_depth, avail_tb, mod_info_curr->ipf_flag
#if MIPF
                          , ctx->info.sqh.mipf_enable_flag
#endif
                );//最终  full_RDO    时的预测，预测数据写入pred_tb中
            }

            pel* org_luma_tb = org_luma + (tb_x - pb_x) + (tb_y - pb_y) * s_org;
            //trans & inverse trans
            enc_diff_16b(tb_width_log2, tb_height_log2, org_luma_tb, pred_tb, s_org, tb_w, tb_w, coef_tmp[Y_C]);//enc_sad.c  790行，调用了指令集优化
            mod_info_curr->num_nz[tb_idx][Y_C] = enc_tq_nnz(ctx, mod_info_curr, Y_C, 0, core->qp_y, ctx->lambda[0], coef_tmp[Y_C], coef_tmp[Y_C], tb_width_log2, tb_height_log2, pi->slice_type, Y_C, 1, secT_Ver_Hor, use_alt4x4Trans);
#if IST
            if (ctx->info.sqh.ist_enable_flag && mod_info_curr->num_nz[tb_idx][Y_C] > 0)
            {
                int skip = 0;
                if (!mod_info_curr->ist_tu_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
                {
                    for (int y = 0; y < cu_height; y++)
                    {
                        for (int x = 0; x < cu_width; x++)
                        {
                            if ((x >= IST_MAX_COEF_SIZE || y >= IST_MAX_COEF_SIZE) && coef_tmp[Y_C][y * cu_width + x])
                            {
                                skip = 1;
                                break;
                            }
                        }
                        if (skip)
                            break;
                    }
                }
                //coefficients refine
                if (mod_info_curr->tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6 && !skip &&
                    ((mod_info_curr->num_nz[tb_idx][Y_C] % 2 && !mod_info_curr->ist_tu_flag) || (!(mod_info_curr->num_nz[tb_idx][Y_C] % 2) && mod_info_curr->ist_tu_flag)))
                {
                    int scan_pos, num_coeff, nz_cnt;
                    const u16     *scanp;
                    scanp = com_scan_tbl[COEF_SCAN_ZIGZAG][tb_width_log2 - 1][tb_height_log2 - 1];
                    num_coeff = 1 << (tb_width_log2 + tb_height_log2);
                    nz_cnt = 0;

                    for (scan_pos = 0; scan_pos <= num_coeff; scan_pos++)
                    {
                        if (nz_cnt < mod_info_curr->num_nz[tb_idx][Y_C])
                        {
                            nz_cnt = coef_tmp[Y_C][scanp[scan_pos]] ? nz_cnt + 1 : nz_cnt;
                        }
                        else
                        {
                            if (coef_tmp[Y_C][scanp[scan_pos - 1]])
                            {
                                if (scan_pos - 1)
                                {
                                    mod_info_curr->num_nz[tb_idx][Y_C]--;
                                    coef_tmp[Y_C][scanp[scan_pos - 1]] = 0;
                                }
                                else
                                {
                                    mod_info_curr->num_nz[tb_idx][Y_C]++;
                                    coef_tmp[Y_C][scanp[scan_pos]] = 1;
                                }

                                break;
                            }
                        }
                    }
                }
#if ISTS
                if (mod_info_curr->tb_part == 0 && mod_info_curr->ist_tu_flag && !ctx->info.pic_header.ph_ists_enable_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
#else
                if (mod_info_curr->tb_part == 0 && mod_info_curr->ist_tu_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
#endif
                {
                    for (int y = 0; y < cu_height; y ++)
                    {
                        for (int x = 0; x < cu_width; x++)
                        {
                            if ((x >= IST_MAX_COEF_SIZE || y >= IST_MAX_COEF_SIZE) && coef_tmp[Y_C][y * cu_width + x])
                            {
                                coef_restrict = 1;
                            }
                        }
                    }
                }
            }
#endif
            s16* coef_tb = mod_info_curr->coef[Y_C] + coef_offset_tb;
            com_mcpy(coef_tb, coef_tmp[Y_C], sizeof(s16) * (tb_w * tb_h));
            if (mod_info_curr->num_nz[tb_idx][Y_C])
            {
                com_itdq(mod_info_curr, Y_C, 0, coef_tmp[Y_C], resi_tb, ctx->wq, tb_width_log2, tb_height_log2, core->qp_y, bit_depth, secT_Ver_Hor, use_alt4x4Trans);
            }

            //to fit the interface of com_recon()
            for (int comp = 0; comp < N_C; comp++)
                num_nz_temp[TB0][comp] = mod_info_curr->num_nz[tb_idx][comp];
            com_recon(SIZE_2Nx2N, resi_tb, pred_tb, num_nz_temp, Y_C, tb_w, tb_h, cu_width, rec_tb, bit_depth
#if SBT
                , 0
#endif
            );

            //dist calc
            cost += enc_ssd_16b(tb_width_log2, tb_height_log2, rec_tb, org_luma_tb, cu_width, s_org, bit_depth); // stride for rec_tb is cu_width
            if (mod_info_curr->pb_part == SIZE_2Nx2N)
            {
                calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, mod_info_curr->num_nz[TB0][Y_C] != 0, NULL, NULL, 0);
                cost += ctx->delta_dist;
            }

            //update map tb
            update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, intra_mode);//更新map内信息
            copy_rec_y_to_pic(rec_tb, tb_x, tb_y, tb_w, tb_h, cu_width, PIC_REC(ctx));
        }

        *dist = (s32)cost;
        if (pb_idx == 0)
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        else
            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_best);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        enc_bit_est_pb_intra_luma(ctx, core, ctx->info.pic_header.slice_type, mod_info_curr->coef, pb_idx);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
#if IST
        if (coef_restrict)
        {
            cost = MAX_COST;
        }
#endif
    }
    else
    {
#if TSCPM
        int strideY = pi->stride_rec[Y_C];
        pel *piRecoY = pi->addr_rec_pic[Y_C] + (y * strideY) + x;  //x y 即亮度块在pic中的坐标
#endif

        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu);
        com_ipred_uv(core->nb[1][0] + 3, core->nb[1][1] + 3, pi->pred[U_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                     , U_C, piRecoY, strideY, core->nb
#endif
#if MIPF
                     , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC
                     , NULL, -1
#endif
#if IPF_CHROMA
                     , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
                    );
        enc_diff_16b(cu_width_log2 - 1, cu_height_log2 - 1, org_cb, pi->pred[U_C], s_org_c, cu_width >> 1, cu_width >> 1, coef_tmp[U_C]);
        mod_info_curr->num_nz[TB0][U_C] = enc_tq_nnz(ctx, mod_info_curr, U_C, 0, core->qp_u, ctx->lambda[1], coef_tmp[U_C], coef_tmp[U_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, U_C, 1, 0, 0);
        com_mcpy(mod_info_curr->coef[U_C], coef_tmp[U_C], sizeof(u16) * (cu_width * cu_height));

#if PMC
        s8 ipm_c = mod_info_curr->ipm[PB0][1];
        int bMcpm = com_is_mcpm(ipm_c);
        if (bMcpm)
        {
            if (!IS_RIGHT_CBF_U(mod_info_curr->num_nz[TB0][U_C]))
            {
                return MAX_COST;
            }
        }
#endif

        if (mod_info_curr->num_nz[TB0][U_C])
        {
            com_itdq(mod_info_curr, U_C, 0, coef_tmp[U_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_u, bit_depth, 0, 0);
        }

        com_recon(SIZE_2Nx2N, resi, pi->pred[U_C], mod_info_curr->num_nz, U_C, cu_width >> 1, cu_height >> 1, cu_width >> 1, pi->rec[U_C], bit_depth
#if SBT
            , 0
#endif
        );
        com_ipred_uv(core->nb[2][0] + 3, core->nb[2][1] + 3, pi->pred[V_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                     , V_C, piRecoY, strideY, core->nb
#endif
#if MIPF
                     , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC
                     , pi->rec[U_C], cu_width >> 1
#endif
#if IPF_CHROMA
                     , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
                    );
        enc_diff_16b(cu_width_log2 - 1, cu_height_log2 - 1, org_cr, pi->pred[V_C], s_org_c, cu_width >> 1, cu_width >> 1, coef_tmp[V_C]);
#if PMC
        int qp_v = core->qp_v;
        double lambda_v = ctx->lambda[2];
        if (com_is_mcpm(ipm_c))
        {
            qp_v = core->qp_v_pmc;
            lambda_v = ctx->lambda_v_pmc;
        }
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0,       qp_v,       lambda_v, coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, 0, 0);
#else
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0, core->qp_v, ctx->lambda[2], coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, 0, 0);
#endif
        com_mcpy(mod_info_curr->coef[V_C], coef_tmp[V_C], sizeof(u16) * (cu_width * cu_height));
        SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_best);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        enc_bit_est_intra_chroma(ctx, core, mod_info_curr->coef);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += ctx->dist_chroma_weight[0] * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[U_C], org_cb, cu_width >> 1, s_org_c, bit_depth);
        if (mod_info_curr->num_nz[TB0][V_C])
        {
#if PMC
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1,       qp_v, bit_depth, 0, 0);
#else
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_v, bit_depth, 0, 0);
#endif
        }

        com_recon(SIZE_2Nx2N, resi, pi->pred[V_C], mod_info_curr->num_nz, V_C, cu_width >> 1, cu_height >> 1, cu_width >> 1, pi->rec[V_C], bit_depth
#if SBT
            , 0
#endif
        );
#if PMC
        double dist_weight_v = ctx->dist_chroma_weight[1];
        if (com_is_mcpm(ipm_c))
        {
            dist_weight_v = ctx->dist_chroma_weight_v_pmc;
        }
        cost += dist_weight_v              * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[V_C], org_cr, cu_width >> 1, s_org_c, bit_depth);
#else
        cost += ctx->dist_chroma_weight[1] * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[V_C], org_cr, cu_width >> 1, s_org_c, bit_depth);
#endif
        *dist = (s32)cost;
        cost += enc_ssd_16b(cu_width_log2, cu_height_log2, pi->rec[Y_C], org_luma, cu_width, s_org, bit_depth);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    }
    return cost;
}

#define NUM_IPM_CAND 3
#define PUT_IPM2LIST(list, cnt, ipm)\
{\
    int idx_list, is_check = 0;\
    for(idx_list = 0; idx_list < (cnt); idx_list++)\
        if((ipm) == (list)[idx_list]) is_check = 1;\
    if(is_check == 0)\
    {\
        (list)[(cnt)] = (ipm); (cnt)++;\
    }\
}

static u32 calc_satd_16b(int pu_w, int pu_h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
{
    u32 cost = 0;
    int num_seg_in_pu_w = 1, num_seg_in_pu_h = 1;
    int seg_w_log2 = com_tbl_log2[pu_w];
    int seg_h_log2 = com_tbl_log2[pu_h];
    s16 *src1_seg, *src2_seg;
    s16* s1 = (s16 *)src1;
    s16* s2 = (s16 *)src2;

    if (seg_w_log2 == -1)
    {
        num_seg_in_pu_w = 3;
        seg_w_log2 = (pu_w == 48) ? 4 : (pu_w == 24 ? 3 : 2);
    }

    if (seg_h_log2 == -1)
    {
        num_seg_in_pu_h = 3;
        seg_h_log2 = (pu_h == 48) ? 4 : (pu_h == 24 ? 3 : 2);
    }

    if (num_seg_in_pu_w == 1 && num_seg_in_pu_h == 1)
    {
        cost += enc_satd_16b(seg_w_log2, seg_h_log2, s1, s2, s_src1, s_src2, bit_depth);
        return cost;
    }

    for (int j = 0; j < num_seg_in_pu_h; j++)
    {
        for (int i = 0; i < num_seg_in_pu_w; i++)
        {
            src1_seg = s1 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src1;
            src2_seg = s2 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src2;
            cost += enc_satd_16b(seg_w_log2, seg_h_log2, src1_seg, src2_seg, s_src1, s_src2, bit_depth);
        }
    }
    return cost;
}

void com_update_cand_list(const u8 ipm, const double cost, const int ipd_num, int *ipred_list, double *cand_cost)
{/*     典型的排序算法，根据cost对ipred-list和cand-cost排序   */
    int shift = 0;
    while (shift < ipd_num && cost < cand_cost[ipd_num - 1 - shift])//计算cand_cost前ipd_num个模式种 代价大于cost的模式数量（ipred_list和cand_list是按从小到大顺序排好的）
    {
        shift++;
    }
    if (shift != 0)
    {
        for (int j = 1; j < shift; j++)//把代价比cost大的 模式及其代价整体往后移动（把cost及其对应模式插入到这些模式的前面）
        {
            ipred_list[ipd_num - j] = ipred_list[ipd_num - 1 - j];
            cand_cost[ipd_num - j] = cand_cost[ipd_num - 1 - j];
        }
        ipred_list[ipd_num - shift] = ipm;
        cand_cost[ipd_num - shift] = cost;
    }
}

/*check一种帧内模式，并存储其SATD代价相关信息*/
void check_one_intra_pred_mode(ENC_CTX *ctx, ENC_CORE *core, pel *org, int s_org, u8 ipm, int ipred_list_len, int *ipred_list, double *cand_cost, int part_idx, int pb_w, int pb_h, u16 avail_cu)
{
    ENC_PINTRA *pi = &ctx->pintra;
    int bit_cnt;
    double cost;
    int bit_depth = ctx->info.bit_depth_internal;
    int cu_width_log2 = (&core->mod_info_curr)->cu_width_log2;
    int cu_height_log2 = (&core->mod_info_curr)->cu_height_log2;

    /*获得当前mode下的预测图像*/
    pel *pred_buf = pi->pred_cache[ipm];//预测图像缓冲区
    com_ipred(core->nb[0][0] + 3, core->nb[0][1] + 3, pred_buf, ipm, pb_w, pb_h, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
              , ctx->info.sqh.mipf_enable_flag
#endif
    );
    cost = (double)calc_satd_16b(pb_w, pb_h, org, pred_buf, s_org, pb_w, bit_depth);//计算SATD失真代价
    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);//LOAD熵编码（未编码当前角度模式）
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    encode_intra_dir(&core->bs_temp, ipm, ctx->info.sqh.eipm_enable_flag, core->mod_info_curr.mpm[part_idx]);//编码帧内角度模式
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;//得到编码帧内角度模式所需比特数
    cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);//得到总哈达玛代价
    com_update_cand_list(ipm, cost, ipred_list_len, ipred_list, cand_cost);
}

/*生成粗选得到的帧内预测模式列表，并返回模式数量*/
static int make_ipred_list_fast(ENC_CTX *ctx, ENC_CORE *core, int width, int height, int cu_width_log2, int cu_height_log2, pel *org, int s_org, int *ipred_list, int part_idx, u16 avail_cu
    , int skip_ipd
    , u8 *mpm
)
{
    static  const int AngMap[IPD_CNT] =//帧内预测模式，按角度递增排列
    {
        0,1,2,3,34,4,35,5,36,6,37,7,
        38,8,39,9,40,10,41,11,42,43,
        12,44,45,13,46,14,47,15,48,
        16,49,17,50,18,51,19,52,20,// _39
        53,21,54,22,55,23,56,57,24,// _48
        58,59,25,60,26,61,27,62,28,// _57
        63,29,64,30,65,31,32,33

    };

    static const int ReverseAng[IPD_CNT] =//把模式序号，映射到该模式在AVS3中按角度递增排列的序号
    {
        0,1,2,3,5,7,9,11,13,15,
        17,19,22,25,27,29,31,33,
        35,37,39,41,43,45,48,51,
        53,55,57,59,61,63,64,65,
        4,6,8,10,12,14,16,18,20,
        21,23,24,26,28,30,32,34,
        36,38,40,42,44,46,47,49,
        50,52,54,56,58,60,62
    };
    /****************************                递进的帧内预测模式快速粗选算法                 *************************/
    static const u8 rmd_search_step_4[18] = { 0,  1,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32 };//RMD搜索，特殊模式+4倍角

    ENC_PINTRA *pi = &ctx->pintra;
    int cu_width, cu_height, i;
    int ipd_rdo_cnt = IPD_RDO_CNT;//率失真次数？
#if FIMC
    ipd_rdo_cnt -= NUM_MPM;//减去两个计数
    assert(ipd_rdo_cnt >= 0);
    int check_mpm_flag[NUM_MPM] = {0, 0};
#endif

    com_assert(ipd_rdo_cnt <= IPD_RDO_CNT);
    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
    for (i = 0; i < ipd_rdo_cnt; i++) //初始化ipred_list
    {
        ipred_list[i] = IPD_DC;
    }

    int num_cand_step_4_in = 18;//第一步输入18种模式
    int num_cand_step_4_out = 10;//第一步输出10种模式（第一步中最优）
    double rmd_cand_cost[18];
    int rmd_ipred_list[18];
    for (i = 0; i < num_cand_step_4_in; i++) //初始化粗选列表
    {
        rmd_ipred_list[i] = IPD_DC;
        rmd_cand_cost[i] = MAX_COST;
    }

    u8 rmd_search_step_2[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // at most 2*num_cand_step_4_out
    int rmd_search_step_2_in = 0;
    int rmd_search_step_2_out = 6;//第二步输出6种模式（第二步中最优）

    u8 rmd_search_step_1[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // at most rmd_search_step_2_out * 2 + 4
    int rmd_search_step_1_in = 0;

    for (i = 0; i < num_cand_step_4_in; i++) //遍历第一步中的每种模式，并存储代价 相关信息，更新rmd_ipred_list及rmd_cand_cost
    {
#if FIMC
        /*如果4倍角中有mpm的模式，视为MPM已被check*/
        if (rmd_search_step_4[i] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_4[i] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        if (skip_ipd == 1 && (rmd_search_step_4[i] == IPD_PLN || rmd_search_step_4[i] == IPD_BI || rmd_search_step_4[i] == IPD_DC))
        {
            continue;
        }

        com_assert(rmd_search_step_4[i] != 33);//模式33不存在rmd_ipred_list
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_4[i], COM_MAX(num_cand_step_4_out, ipd_rdo_cnt), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);//perform一遍帧内预测并更新rmd_ipred_list和rmd_cand_cost
    }

    if (rmd_ipred_list[0] < 2 && rmd_ipred_list[1] < 2) //粗模式表里的前两个模式是DC或Plane，提前终止 快速帧内角度模式决策
    {
        for (i = 0; i < 3; i++) 
        {
            ipred_list[i] = rmd_ipred_list[i];//将第一步的模式列表代价最小的前三个读入 ipred_list 中
        }
#if FIMC
        for( i = 0; i < NUM_MPM; i++ )
        {
            if( check_mpm_flag[i] == 0 && mpm[i] != IPD_IPCM )
            {//mpm没被check过则check一边mpm并更新rmd_ipred_list
                check_one_intra_pred_mode( ctx, core, org, s_org, mpm[i], COM_MIN( ipd_rdo_cnt, IPD_RDO_CNT ), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu );
            }
        }
#endif
        return 3;
    }

    /*对于输出的10个最优4倍角，获取其相邻二倍角*/
    for (int rmd_idx = 0; rmd_idx < num_cand_step_4_out; rmd_idx++) 
    {
        int imode4step = ReverseAng[rmd_ipred_list[rmd_idx]];
        int imode4step_sub_2 = AngMap[imode4step - 2];//左邻二倍角
        int imode4step_add_2 = AngMap[imode4step + 2];//右邻二倍角
        if (imode4step == 22 || imode4step == 48) //垂直模式或水平模式
        {
            imode4step_sub_2 = AngMap[imode4step - 3];
            imode4step_add_2 = AngMap[imode4step + 3];
        }       
        if (imode4step >= 5 && (imode4step != 64)) //AngMap[imode2step] 对应Mode= 53 和 mode = 7(左边相邻的二倍角是四倍角)
        {
            for (i = 0; i < rmd_search_step_2_in; i++) //check rmd_search_step_2的输入中是否含有模式imode4step_sub_2，（有则重复了，不放入rmd_search_step_2中）
            {
                if (imode4step_sub_2 == rmd_search_step_2[i]) 
                {
                  break;
                }
            }
            //若有重复模式，break，即  i != rmd_search_step_2_in.
            if (i == rmd_search_step_2_in && imode4step_sub_2 >= 3) //若无重复, i必 == rmd_search_step_2_in,  再确认左邻二倍角是角度模式
            {
                rmd_search_step_2[rmd_search_step_2_in++] = imode4step_sub_2;//添加‘左’邻二倍角
            }
            for (i = 0; i < rmd_search_step_2_in; i++) 
            {
                if (imode4step_add_2 == rmd_search_step_2[i]) 
                {
                    break;
                }
            }
            if (i == rmd_search_step_2_in && imode4step_add_2 != 33) //若无重复, 再确认右邻二倍角不是PCM模式（AngMap中模式33在AngMap[65]，因此只有add可能会出现IPM=33的情况，sub不会）
            {
                rmd_search_step_2[rmd_search_step_2_in++] = imode4step_add_2;//添加‘右’邻二倍角
            }
        }
    }
    /*check 第二阶段所有输入，并选出SATD代价最低的6种角度（与4倍角也进行比较）*/
    for (int rmd_idx = 0; rmd_idx < rmd_search_step_2_in; rmd_idx++) 
    {
        com_assert(rmd_search_step_2[rmd_idx] != 33);
#if FIMC
        /* 如果2倍角中有mpm的模式，视为MPM已被check */
        if (rmd_search_step_2[rmd_idx] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_2[rmd_idx] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_2[rmd_idx], COM_MAX(rmd_search_step_2_out, ipd_rdo_cnt), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);
    }
    /*check第二阶段输出最优的6种模式，获取其 相邻单倍角*/
    for (int rmd_idx = 0, j = 0; rmd_idx < rmd_search_step_2_out;)
    {
        int imode2step = ReverseAng[rmd_ipred_list[j++]];
        int imode2step_sub_1 = AngMap[imode2step - 1];//左侧单倍角
        int imode2step_add_1 = AngMap[imode2step + 1];//右侧单倍角

        if (j >= num_cand_step_4_in)
        {
            break;
        }
        if (imode2step >= 4 && imode2step != 65)//AngMap[imode2step]是单倍角且不是PCM模式
        {
            rmd_idx++;
            if (rmd_search_step_1_in == 0)//单倍角列表为空（第一次写入模式）
            {
                rmd_search_step_1[rmd_search_step_1_in++] = imode2step_sub_1;
                if (imode2step_add_1 != 33 && imode2step != 63)//mode ==33 PCM模式；mode==31已经没有‘右’邻单倍角模式了
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_add_1;//添加‘右’邻单倍角
                }
                if (imode2step == 22 || imode2step == 48)//水平和垂直模式用42和45
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step - 2];
                    rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step + 2];
                }
            } 
            else//同第二步
            {
                for (i = 0; i < rmd_search_step_1_in; i++)
                {
                    if (imode2step_sub_1 == rmd_search_step_1[i])
                    {
                        break;
                    }
                }
                if (i == rmd_search_step_1_in && (imode2step_sub_1 >= 3))
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_sub_1;
                }
                for (i = 0; i < rmd_search_step_1_in; i++)
                {
                    if (imode2step_add_1 == rmd_search_step_1[i] || (imode2step == 63))
                    {
                        break;
                    }
                }
                if (i == rmd_search_step_1_in && (imode2step_add_1 != 33))
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_add_1;
                }
                if (imode2step == 22 || imode2step == 48)
                {
                    for (i = 0; i < rmd_search_step_1_in; i++)
                    {
                        if ((imode2step - 2) == ReverseAng[rmd_search_step_1[i]])
                        {
                            break;
                        }
                    }
                    if (i == rmd_search_step_1_in)
                    {
                        rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step - 2];
                    }
                    for (i = 0; i < rmd_search_step_1_in; i++)
                    {
                        if ((imode2step + 2) == ReverseAng[rmd_search_step_1[i]])
                        {
                            break;
                        }
                    }
                    if (i == rmd_search_step_1_in)
                    {
                        rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step + 2];
                    }
                }
            }
        }
    }
    /*check第三阶段选处的单倍角模式, 并选出SATD代价最低的5或2(取最小值，FIMC时为2)种角度（与4倍角和2倍角也进行比较）*/
    for (int rmd_idx = 0; rmd_idx < rmd_search_step_1_in; rmd_idx++) 
    {
#if FIMC
        if (rmd_search_step_1[rmd_idx] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_1[rmd_idx] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        com_assert(rmd_search_step_1[rmd_idx] != 33);
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_1[rmd_idx], COM_MIN(ipd_rdo_cnt, IPD_RDO_CNT), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);
    }
#if FIMC
    /*最后检查MPM里模式是否均被check过，否则，check mpm模式并据此更新rmd_ipred_list */
    for( i = 0; i < NUM_MPM; i++ )
    {
        if( check_mpm_flag[i] == 0 && mpm[i] != IPD_IPCM )
        {
            check_one_intra_pred_mode( ctx, core, org, s_org, mpm[i], COM_MIN( ipd_rdo_cnt, IPD_RDO_CNT ), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu );
        }
    }
#endif
    int pred_cnt = COM_MIN(ipd_rdo_cnt, IPD_RDO_CNT);//不启用FIMC是5， 启用FIMC是3
    for (i = 0; i < pred_cnt; i++) 
    {//更新ipred_list， 取rmd_ipred_list里代价最小的前pred_cnt个模式
        ipred_list[i] = rmd_ipred_list[i];//更新帧内预测模式列表并返回
    }

    for (i = pred_cnt - 1; i >= 0; i--)/*？？？？*/
    {
        if (rmd_cand_cost[i] > core->inter_satd * (1.2))//比帧间SATD大1.2倍
        {
            pred_cnt--;//减少帧内预测数目
        }
        else
        {
            break;
        }
    }
    return pred_cnt;
}

static int make_ipred_list(ENC_CTX * ctx, ENC_CORE * core, int width, int height, int cu_width_log2, int cu_height_log2, pel * org, int s_org, int * ipred_list, int part_idx, u16 avail_cu
                           , int skip_ipd
                          )
{
    ENC_PINTRA * pi = &ctx->pintra;
    int bit_depth = ctx->info.bit_depth_internal;
    int pred_cnt, i, j;
    double cost, cand_cost[IPD_RDO_CNT];
    u32 cand_satd_cost[IPD_RDO_CNT];
    u32 cost_satd;
    int ipd_rdo_cnt = (width >= height * 4 || height >= width * 4) ? IPD_RDO_CNT - 1 : IPD_RDO_CNT;
#if FIMC
    ipd_rdo_cnt -= NUM_MPM;
    assert(ipd_rdo_cnt >= 0);
#endif
    for (i = 0; i < ipd_rdo_cnt; i++)
    {
        ipred_list[i] = IPD_DC;
        cand_cost[i] = MAX_COST;
        cand_satd_cost[i] = COM_UINT32_MAX;
    }
    pred_cnt = IPD_CNT;
#if EIPM
    int num_intra;
    if (ctx->info.sqh.eipm_enable_flag) 
    {
        num_intra = IPD_CNT;
    } 
    else 
    {
        num_intra = IPD_IPCM;
    }
    for (i = 0; i < num_intra; i++)
#else
    for (i = 0; i < IPD_CNT; i++)
#endif
    {
        if (skip_ipd == 1 && (i == IPD_PLN || i == IPD_BI || i == IPD_DC))
        {
            continue;
        }

#if EIPM
        if (i == IPD_IPCM)
        {
            continue;
        }
#endif
        int bit_cnt, shift = 0;
        pel * pred_buf = NULL;
        pred_buf = pi->pred_cache[i];
        com_ipred(core->nb[0][0] + 3, core->nb[0][1] + 3, pred_buf, i, width, height, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
                  , ctx->info.sqh.mipf_enable_flag
#endif
        );
        cost_satd = calc_satd_16b(width, height, org, pred_buf, s_org, width, bit_depth);
        cost = (double)cost_satd;
        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        encode_intra_dir(&core->bs_temp, (u8)i,
#if EIPM
            ctx->info.sqh.eipm_enable_flag,
#endif
            core->mod_info_curr.mpm[part_idx]);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);
        while (shift < ipd_rdo_cnt && cost < cand_cost[ipd_rdo_cnt - 1 - shift])
        {
            shift++;
        }
        if (shift != 0)
        {
            for (j = 1; j < shift; j++)
            {
                ipred_list[ipd_rdo_cnt - j] = ipred_list[ipd_rdo_cnt - 1 - j];
                cand_cost[ipd_rdo_cnt - j] = cand_cost[ipd_rdo_cnt - 1 - j];
                cand_satd_cost[ipd_rdo_cnt - j] = cand_satd_cost[ipd_rdo_cnt - 1 - j];
            }
            ipred_list[ipd_rdo_cnt - shift] = i;
            cand_cost[ipd_rdo_cnt - shift] = cost;
            cand_satd_cost[ipd_rdo_cnt - shift] = cost_satd;
        }
    }
    pred_cnt = ipd_rdo_cnt;
    for (i = ipd_rdo_cnt - 1; i >= 0; i--)
    {
        if (cand_satd_cost[i] > core->inter_satd * (1.1))
        {
            pred_cnt--;
        }
        else
        {
            break;
        }
    }
    return COM_MIN(pred_cnt, ipd_rdo_cnt);
}

double analyze_intra_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PINTRA *pi = &ctx->pintra;
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j, s_org, s_org_c, s_mod, s_mod_c;
    int best_ipd[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_pb_part[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_c = IPD_INVALID;//色度最优帧内预测模式
    s32 best_dist_y = 0, best_dist_c = 0;
    s32 best_dist_y_pb_part[MAX_NUM_TB] = { 0, 0, 0, 0 };
    u8  best_mpm_pb_part[MAX_NUM_TB][2];
    u8  best_mpm[MAX_NUM_TB][2];
    s16  coef_y_pb_part[MAX_CU_DIM];
    pel  rec_y_pb_part[MAX_CU_DIM];
    int  num_nz_y_pb_part[MAX_NUM_TB];
    int ipm_l2c = 0;
    int chk_bypass = 0;
    int bit_cnt = 0;
    int ipred_list[IPD_CNT];
    int pred_cnt = IPD_CNT;
    pel *org, *mod;
    pel *org_cb, *org_cr;
    pel *mod_cb, *mod_cr;
    double cost_temp, cost_best = MAX_COST;
    double cost_pb_temp, cost_pb_best;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int cu_x = mod_info_curr->x_pos;
    int cu_y = mod_info_curr->y_pos;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if EST
    double costdct2[5] = { MAX_COST, MAX_COST, MAX_COST, MAX_COST, MAX_COST };
    int cbfdct2[5] = { 1, 1, 1, 1, 1 };
    int cbfdst7[5] = { 1, 1, 1, 1, 1 };
    int bst_est_tu_flag = 0;
    bst_info->est_flag = 0;
    mod_info_curr->est_flag = 0;
#endif
#if USE_IBC
    mod_info_curr->cu_mode = MODE_INTRA;
    mod_info_curr->ibc_flag = 0;
#endif
#if USE_SP
    mod_info_curr->sp_flag = 0;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
    int ipd_buf[4] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };//4条带ipm缓冲器，存储四条带划分下每个PU内的ipm
    /*Ipd_add含义:    ipd_add表示当前CU若采用DT四条带划分，条带的模式重复情况
       ipd_add[0-1]表示水平四条带划分
            其中0：表示后三个PU的模式相同，[0,1,2]分别表示相同的模式为PLN,BI,DC，其值表示是否成立
            其中1：表示前三个PU的模式相同，[0,1,2]分别表示相同的模式为PLN,BI,DC，其值表示是否成立
       ipd_add[2-3]表示垂直四条带划分
            其中2：表示后三个PU的模式相同，[0,1,2]分别表示相同的模式为PLN,BI,DC，其值表示是否成立
            其中3：表示前三个PU的模式相同，[0,1,2]分别表示相同的模式为PLN,BI,DC，其值表示是否成立
    */
    int ipd_add[4][3] = { { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID },
        { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID }
    };
#if DT_SAVE_LOAD
    ENC_BEF_DATA* pData = &core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup];
#endif
#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    //                                             x_scup*4 = x_Pelp         y_scup*4 = y_Pelp
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
    /*stride of reconstruction picture buffer*/
    s_mod = pi->stride_rec[Y_C]; 
    s_org = pi->stride_org[Y_C];
    s_mod_c = pi->stride_rec[U_C];
    s_org_c = pi->stride_org[U_C];
    mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
    org = pi->addr_org[Y_C] + (y * s_org) + x;
    int part_size_idx, pb_part_idx;
    core->best_pb_part_intra = SIZE_2Nx2N;/*mark___________________________初始化帧内的PU最好的划分是2N*2N？*/

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)//亮度Tree__DualTree结构？
    {
        int allowed_part_size[7] = { SIZE_2Nx2N };
        int num_allowed_part_size = 1;
        int dt_allow = ctx->info.sqh.dt_intra_enable_flag ? com_dt_allow(mod_info_curr->cu_width, mod_info_curr->cu_height, MODE_INTRA, ctx->info.sqh.max_dt_size) : 0;
#if DT_INTRA_BOUNDARY_FILTER_OFF
        if (core->mod_info_curr.ipf_flag)//使用IPF时禁止DT
            dt_allow = 0;
#endif
        //prepare allowed PU partition，先check DT四条带划分    方向已经通过com_dt_allow里的return 值确定了
        if (dt_allow & 0x1) // horizontal
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxhN;
        }
        if (dt_allow & 0x2) // vertical
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_hNx2N;
        }
        if (dt_allow & 0x1) // horizontal
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxnU;
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxnD;
        }
        if (dt_allow & 0x2) // vertical
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_nLx2N;
            allowed_part_size[num_allowed_part_size++] = SIZE_nRx2N;
        }

        cost_best = MAX_COST;
#if DT_INTRA_FAST_BY_RD
        u8 try_non_2NxhN = 1, try_non_hNx2N = 1;
        double cost_2Nx2N = MAX_COST, cost_hNx2N = MAX_COST, cost_2NxhN = MAX_COST;
#endif

#if FIMC
        // copy table for dt, last, best, core
        COM_CNTMPM cntmpm_cands_curr;
        COM_CNTMPM cntmpm_cands_last;
        if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
        {
            com_cntmpm_copy(&cntmpm_cands_last, &core->cntmpm_cands);
            com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
        }
#endif

        for (part_size_idx = 0; part_size_idx < num_allowed_part_size; part_size_idx++)
        {
            //check所有可能的DT划分，选出能产生最小代价的DT划分及帧内模式；DT不被允许则PUSize=CUSize，即此for循环只执行一次
            /********************************           关于TU，PU的相关设置        ***************************************/
            PART_SIZE pb_part_size = allowed_part_size[part_size_idx];
            PART_SIZE tb_part_size = get_tb_part_size_by_pb(pb_part_size, MODE_INTRA);//TBSize根据PB的划分，只存在三种情况：1.PU为2N*2N时，2N*2N（帧内）或N*N（帧间）、2.PU水平划分时，水平四个条带状 3.PU垂直划分时，垂直四个条带状
            set_pb_part(mod_info_curr, pb_part_size);
            set_tb_part(mod_info_curr, tb_part_size);
            get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, pb_part_size, &mod_info_curr->pb_info);//获取PB划分信息
            get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, tb_part_size, &mod_info_curr->tb_info);//获取TB划分信息
            assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
            cost_temp = 0;
            memset(num_nz_y_pb_part, 0, MAX_NUM_TB * sizeof(int));

            //DT fast algorithm here
#if DT_INTRA_FAST_BY_RD
            if (((pb_part_size == SIZE_2NxnU || pb_part_size == SIZE_2NxnD) && !try_non_2NxhN)
                    || ((pb_part_size == SIZE_nLx2N || pb_part_size == SIZE_nRx2N) && !try_non_hNx2N))
            {
                continue;
            }
#endif
#if DT_SAVE_LOAD
            if (pb_part_size != SIZE_2Nx2N && pData->num_intra_history > 1 && pData->best_part_size_intra[0] == SIZE_2Nx2N && pData->best_part_size_intra[1] == SIZE_2Nx2N)
                break;
#endif
            //end of DT fast algorithm

#if FIMC
            // copy table for dt
            if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
            {
                com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_last);
            }
#endif

            // pb-based intra mode candidate selection
            for (pb_part_idx = 0; pb_part_idx < mod_info_curr->pb_info.num_sub_part; pb_part_idx++)//check每种DT下的每个PB，计算出其最低的代价
            {
                /***************************获取每个sub块的信息************************/
                int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
                int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
                int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
                int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
                int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
                int pb_x_scu = PEL2SCU(pb_x);
                int pb_y_scu = PEL2SCU(pb_y);
                int pb_coef_offset = get_coef_offset_tb(mod_info_curr->x_pos, mod_info_curr->y_pos, pb_x, pb_y, cu_width, cu_height, tb_part_size);
                int tb_idx_offset = get_tb_idx_offset(pb_part_size, pb_part_idx);//tb的索引补偿。非DT下正常操作，DT下，合并TB边界，并更新TB索引，若水平DT下，有4个TB，但若PB为2N*nU的话，下方的三个TB都被划作第二个TB（与PB对应）
                int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_part_idx);//获取PB 内 TB的个数
                int skip_ipd = 0;
                if (((pb_part_size == SIZE_2NxnU || pb_part_size == SIZE_nLx2N) && pb_part_idx == 1) ||
                    ((pb_part_size == SIZE_2NxnD || pb_part_size == SIZE_nRx2N) && pb_part_idx == 0))
                {
                    skip_ipd = 1;
                }

                cost_pb_best = MAX_COST;
                cu_nz_cln(mod_info_curr->num_nz);

                mod = pi->addr_rec_pic[Y_C] + (pb_y * s_mod) + pb_x;//重构图像中，当前PU对应的起始位置
                org = pi->addr_org[Y_C] + (pb_y * s_org) + pb_x;//原始图像中，当前PU对应的起始位置

                u16 avail_cu = com_get_avail_intra(pb_x_scu, pb_y_scu, ctx->info.pic_width_in_scu, pb_scup, ctx->map.map_scu);
                com_get_nbr(pb_x, pb_y, pb_w, pb_h, mod, s_mod, avail_cu, core->nb, pb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);///@UpdatedBy:Chaos ： 获取参考像素

#if FIMC
                if (ctx->info.sqh.fimc_enable_flag && ctx->info.pic_header.fimc_pic_flag)
                {
                    com_get_cntmpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx], &core->cntmpm_cands);
                }
                else
                {
#endif
                    com_get_mpm( pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx] );//获取mpm模式，源于左侧和上方的scu的帧内角度模式
#if FIMC
                }
#endif

#if EIPM
                if (ctx->info.sqh.eipm_enable_flag)//扩展到帧内预测模式，从33种帧内预测模式到65种预测模式，但与VVC的角度似乎不一样     
                {
                    pred_cnt = make_ipred_list_fast(ctx, core, pb_w, pb_h, cu_width_log2, cu_height_log2, org, s_org, ipred_list, pb_part_idx, avail_cu
                        , skip_ipd, mod_info_curr->mpm[pb_part_idx]
                    );
                }
                else
                {
#endif
                    pred_cnt = make_ipred_list(ctx, core, pb_w, pb_h, cu_width_log2, cu_height_log2, org, s_org, ipred_list, pb_part_idx, avail_cu
                        , skip_ipd
                    );
#if EIPM
                }
#endif

                if (skip_ipd == 1)/*触发条件：1/3等分的down或right  或  3/1等分的up或left*/
                {
                    if (pb_part_size == SIZE_2NxnU)
                    {
                        if (ipd_add[0][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[0][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[0][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_2NxnD)
                    {
                        if (ipd_add[1][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[1][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[1][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_nLx2N)
                    {
                        if (ipd_add[2][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[2][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[2][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_nRx2N)
                    {
                        if (ipd_add[3][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[3][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[3][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                }

#if FIMC
                /*  强行把FIMC获得的MPM放入pred_list进行RDO  */
                if ((ctx->info.sqh.ibc_flag && pb_w * pb_h <= 32) || !ctx->info.sqh.ibc_flag)//不启用IBC 或 启用IBC且PU的像素数不超过32个
                {
                    u8 mode_in_list[CNTMPM_MAX_NUM];
                    memset(mode_in_list, 0, sizeof(u8)* CNTMPM_MAX_NUM);
                    for (int mIdx = 0; mIdx < pred_cnt; mIdx++)
                    {
                        mode_in_list[ipred_list[mIdx]] = 1;//pred_list中现存的模式
                    }
                    u8 *curr_mpm = mod_info_curr->mpm[pb_part_idx];
                    const u8 extra_rdo_num = NUM_MPM;
                    for (int mIdx = 0; mIdx < extra_rdo_num; mIdx++)
                    {
                        u8 curr_mode = curr_mpm[mIdx];
                        if (mode_in_list[curr_mode] == 0 && (curr_mode != IPD_IPCM))//当前模式不是PCM模式且不在pred_list中（代价比较高）
                        {
                            assert(curr_mode >= 0 && curr_mode < IPD_CNT);
                            mode_in_list[curr_mode] = 1;
                            ipred_list[pred_cnt++] = curr_mode;
                        }
                    }
                }
#endif
                if (pred_cnt == 0)
                {
                    return MAX_COST;
                }
#if EST
                int st_all = (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0) ? 2 : 1;//先看1
                for (int use_st = 0; use_st < st_all; use_st++)
                {
                    mod_info_curr->est_flag = use_st;
                    if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part != 0)
                        mod_info_curr->est_flag = 1;
#endif
                    /*比较一个PU内不同角度预测模式的full_RDO代价，并选出最优的模式，更新相关信息*/
                    for (j = 0; j < pred_cnt; j++) /* Y */
                    {
                        s32 dist_t = 0;
                        i = ipred_list[j];
                        mod_info_curr->ipm[pb_part_idx][0] = (s8)i;
                        mod_info_curr->ipm[pb_part_idx][1] = IPD_INVALID;
#if IST
                        int ist_all = (ctx->info.sqh.ist_enable_flag &&
#if EST
                            (ctx->info.sqh.est_enable_flag ? !mod_info_curr->est_flag : !mod_info_curr->tb_part) &&
#else
                            mod_info_curr->tb_part == 0 &&
#endif
                            cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
                        for (int use_ist = 0; use_ist < ist_all; use_ist++)
                        {
                            mod_info_curr->ist_tu_flag = use_ist;
#endif
#if EST
                            if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && mod_info_curr->est_flag)
                            {
                                if (cost_pb_best * 1.4 < costdct2[j])
                                    continue;
                                if (!cbfdct2[j] && !cbfdst7[j])
                                    continue;
                            }
#endif
                            cost_pb_temp = pintra_residue_rdo(ctx, core, org, NULL, NULL, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 0, pb_part_idx, x, y);
#if EST
                            if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && !mod_info_curr->est_flag)
                            {
                                if (mod_info_curr->ist_tu_flag)
                                {
                                    cbfdst7[j] = mod_info_curr->num_nz[0][0];
                                }
                                else
                                {
                                    costdct2[j] = cost_pb_temp;
                                    cbfdct2[j] = mod_info_curr->num_nz[0][0];
                                }
                            }
#endif
#if PRINT_CU_LEVEL_2
                            printf("\nluma pred mode %2d cost_pb_temp %10.1f", i, cost_pb_temp);
                            double val = 2815.9;
                            if (cost_pb_temp - val < 0.1 && cost_pb_temp - val > -0.1)
                            {
                                int a = 0;
                            }
#endif
                            if (cost_pb_temp < cost_pb_best)//比较当前角度模式下PB内 的代价，更新相关信息
                            {
                                cost_pb_best = cost_pb_temp;
                                best_dist_y_pb_part[pb_part_idx] = dist_t;
                                best_ipd_pb_part[pb_part_idx] = i;//更新PB内更优ipm模式
                                best_mpm_pb_part[pb_part_idx][0] = mod_info_curr->mpm[pb_part_idx][0];//更新PB内更优mpm模式
                                best_mpm_pb_part[pb_part_idx][1] = mod_info_curr->mpm[pb_part_idx][1];//更新PB内更优mpm模式

#if IST
                                bst_ist_tu_flag = mod_info_curr->ist_tu_flag;
#endif
#if EST
                                bst_est_tu_flag = mod_info_curr->est_flag;
#endif
                                com_mcpy(coef_y_pb_part + pb_coef_offset, mod_info_curr->coef[Y_C], pb_w * pb_h * sizeof(s16));
                                for (int j = 0; j < pb_h; j++)
                                {
                                    int rec_offset = ((pb_y - cu_y) + j) * cu_width + (pb_x - cu_x);
                                    com_mcpy(rec_y_pb_part + rec_offset, pi->rec[Y_C] + rec_offset, pb_w * sizeof(pel));
                                }

                                for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
                                {
                                    num_nz_y_pb_part[tb_idx + tb_idx_offset] = mod_info_curr->num_nz[tb_idx][Y_C];
                                }
#if EST
                                if (ctx->info.sqh.est_enable_flag)
                                {
                                    SBAC_STORE(core->s_temp_prev_comp_run, core->s_temp_run);
                                }
                                else
                                {
                                    SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                                }
#else
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
#endif
                            }
#if IST
                        }
#endif
                    }
#if EST
                }
                if (ctx->info.sqh.est_enable_flag)
                {
                    SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_prev_comp_run);
                }
#endif
                cost_temp += cost_pb_best;//将每个PB的best_cost相加，以获得当前CU划分下的总代价
                if (pb_part_size == SIZE_2NxhN || pb_part_size == SIZE_hNx2N)
                {
                    ipd_buf[pb_part_idx] = best_ipd_pb_part[pb_part_idx];//更新 4条带 ipm缓冲器
                }

                //update map - pb
                update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, best_ipd_pb_part[pb_part_idx]);//更新pb附近的map信息
                copy_rec_y_to_pic(rec_y_pb_part + (pb_y - cu_y) * cu_width + (pb_x - cu_x), pb_x, pb_y, pb_w, pb_h, cu_width, PIC_REC(ctx));// 把重构信息写入rec_pic中

#if FIMC
                /*统计当前PU的IPM，并存入频数表*/
                if (ctx->info.sqh.fimc_enable_flag) //  && pb_part_idx == PB0
                {
                    com_cntmpm_update(&core->cntmpm_cands, best_ipd_pb_part[pb_part_idx]);
                }
#endif
            }

            if (pb_part_size == SIZE_2NxhN || pb_part_size == SIZE_hNx2N)//DT下4条带的划分
            {
                int mem_offset = pb_part_size == SIZE_hNx2N ? 2 : 0;//垂直4条带：2，水平4条带0
                /*若后三个PU的模式相同，均为特殊模式*/
                if (ipd_buf[1] == IPD_PLN || ipd_buf[2] == IPD_PLN || ipd_buf[3] == IPD_PLN)
                {
                    ipd_add[mem_offset + 0][0] = 1;
                }
                if (ipd_buf[1] == IPD_BI || ipd_buf[2] == IPD_BI || ipd_buf[3] == IPD_BI)
                {
                    ipd_add[mem_offset + 0][1] = 1;
                }
                if (ipd_buf[1] == IPD_DC || ipd_buf[2] == IPD_DC || ipd_buf[3] == IPD_DC)
                {
                    ipd_add[mem_offset + 0][2] = 1;
                }
                /*若前三个PU的模式相同，均为特殊模式*/
                if (ipd_buf[0] == IPD_PLN || ipd_buf[1] == IPD_PLN || ipd_buf[2] == IPD_PLN)
                {
                    ipd_add[mem_offset + 1][0] = 1;
                }
                if (ipd_buf[0] == IPD_BI || ipd_buf[1] == IPD_BI || ipd_buf[2] == IPD_BI)
                {
                    ipd_add[mem_offset + 1][1] = 1;
                }
                if (ipd_buf[0] == IPD_DC || ipd_buf[1] == IPD_DC || ipd_buf[2] == IPD_DC)
                {
                    ipd_add[mem_offset + 1][2] = 1;
                }
            }

            com_mcpy(pi->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));
#if RDO_DBK
            if (mod_info_curr->pb_part != SIZE_2Nx2N)//如果CU进行了DT划分的话，需要进行去块滤波
            {
                int cbf_y = num_nz_y_pb_part[0] + num_nz_y_pb_part[1] + num_nz_y_pb_part[2] + num_nz_y_pb_part[3];
                calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, cbf_y, NULL, NULL, 0);
                cost_temp += ctx->delta_dist;//将去块滤波的失真，也计算进代价中
                best_dist_y_pb_part[PB0] += (s32)ctx->delta_dist; //add delta SSD to the first PB
            }
#endif
            /*基于RD代价的帧内DT快速算法TODO*/
#if DT_INTRA_FAST_BY_RD
            if (pb_part_size == SIZE_2Nx2N)
                cost_2Nx2N = cost_temp;
            else if (pb_part_size == SIZE_2NxhN)
            {
                cost_2NxhN = cost_temp;
                assert(cost_2Nx2N != MAX_COST);
                if (cost_2NxhN > cost_2Nx2N * 1.05)
                    try_non_2NxhN = 0;
            }
            else if (pb_part_size == SIZE_hNx2N)
            {
                cost_hNx2N = cost_temp;
                assert(cost_2Nx2N != MAX_COST);
                if (cost_hNx2N > cost_2Nx2N * 1.05)
                    try_non_hNx2N = 0;
            }

            if (cost_hNx2N != MAX_COST && cost_2NxhN != MAX_COST)
            {
                if (cost_hNx2N > cost_2NxhN * 1.1)
                    try_non_hNx2N = 0;
                else if (cost_2NxhN > cost_hNx2N * 1.1)
                    try_non_2NxhN = 0;
            }
#endif
            //save luma cb decision for each pb_part_size，选出最优的PB划分模式（DT），并更新相关信息
            if (cost_temp < cost_best)
            {
                cost_best = cost_temp;
                best_dist_y = 0;
                for (int pb_idx = 0; pb_idx < mod_info_curr->pb_info.num_sub_part; pb_idx++)
                {
                    best_dist_y += best_dist_y_pb_part[pb_idx];
                    best_ipd[pb_idx] = best_ipd_pb_part[pb_idx];// 更新每个PB最优预测模式
                    best_mpm[pb_idx][0] = best_mpm_pb_part[pb_idx][0];
                    best_mpm[pb_idx][1] = best_mpm_pb_part[pb_idx][1];
                }
#if IST
                bst_info->ist_tu_flag = bst_ist_tu_flag;
#endif
#if EST
                bst_info->est_flag = bst_est_tu_flag;
#endif
                com_mcpy(bst_info->coef[Y_C], coef_y_pb_part, cu_width * cu_height * sizeof(s16));
                com_mcpy(bst_info->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));
                assert(mod_info_curr->pb_info.num_sub_part <= mod_info_curr->tb_info.num_sub_part);
                for (int tb_idx = 0; tb_idx < mod_info_curr->tb_info.num_sub_part; tb_idx++)
                {
                    bst_info->num_nz[tb_idx][Y_C] = num_nz_y_pb_part[tb_idx];
                }
#if TB_SPLIT_EXT
                core->best_pb_part_intra = mod_info_curr->pb_part;
                core->best_tb_part_intra = mod_info_curr->tb_part;
#endif
                SBAC_STORE(core->s_temp_pb_part_best, core->s_temp_prev_comp_best);//熵编码

#if FIMC
                // copy table for dt，记录更优DT划分的频数表
                if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
                {
                    com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
                }
#endif
            }
        }
#if FIMC
        // copy table for dt， 记录最优DT划分的频数表
        if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
        {
            com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_curr);
        }
#endif
        pel *pBestSrc = bst_info->rec[Y_C];
        pel *pModDst = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
        for (int h = 0; h < cu_height; h++)
        {
            com_mcpy(pModDst, pBestSrc, cu_width * sizeof(pel));
            pModDst += s_mod;
            pBestSrc += cu_width;
        }
    }

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_C)//    色度Tree或者亮色共用树
    {
        //chroma RDO
        SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_pb_part_best);
        org = pi->addr_org[Y_C] + (y * s_org) + x;
#if TSCPM
        mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
#endif
        mod_cb = pi->addr_rec_pic[U_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        mod_cr = pi->addr_rec_pic[V_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        org_cb = pi->addr_org[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
        org_cr = pi->addr_org[V_C] + ((y >> 1) * s_org_c) + (x >> 1);

        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu);
#if TSCPM
        com_get_nbr(x,      y,      cu_width,      cu_height,      mod,    s_mod,   avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
#endif
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cb, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, U_C);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cr, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, V_C);

        cost_best = MAX_COST;

#if CHROMA_NOT_SPLIT
        //get luma pred mode
        if (ctx->tree_status == TREE_C)
        {
            assert(cu_width >= 8 && cu_height >= 8);
            int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;//CU右下角scu的位置
            best_ipd[PB0] = ctx->map.map_ipm[luma_scup];
#if IPCM
            assert((best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT) || best_ipd[PB0] == IPD_IPCM);
#else
            assert(best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT);
#endif
#if USE_IBC
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]) || MCU_GET_IBC(ctx->map.map_scu[luma_scup]));
#else
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]));
#endif
        }
#endif
        ipm_l2c = best_ipd[PB0];//获取CU的0号PU的帧内模式
        mod_info_curr->ipm[PB0][0] = (s8)best_ipd[PB0];
        COM_IPRED_CONV_L2C_CHK(ipm_l2c, chk_bypass);
#if TSCPM
#if ENHANCE_TSPCM
        /* add ehance_tscpm, 2 more mode */
#if PMC
        for (i = 0; i < IPD_CHROMA_CNT + 6; i++) /* UV，check所有可能的色度预测模式，选出最好的一种 */
#else
        for (i = 0; i < IPD_CHROMA_CNT + 3; i++) /* UV */
#endif
#else
#if PMC
        for (i = 0; i < IPD_CHROMA_CNT + 6; i++) /* UV */
#else
        for (i = 0; i < IPD_CHROMA_CNT + 1; i++) /* UV */
#endif
#endif
#else
        for (i = 0; i < IPD_CHROMA_CNT; i++) /* UV */
#endif
        {
            s32 dist_t = 0;
            mod_info_curr->ipm[PB0][1] = (s8)i;
            if (i != IPD_DM_C && chk_bypass && i == ipm_l2c)
            {
                continue;
            }
#if IPCM
            if (i == IPD_DM_C && best_ipd[PB0] == IPD_IPCM)
            {
                continue;
            }
#endif
#if TSCPM
            if (!ctx->info.sqh.tscpm_enable_flag && i == IPD_TSCPM_C)
            {
                continue;
            }
#if ENHANCE_TSPCM
            if (!ctx->info.sqh.enhance_tscpm_enable_flag && (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)) 
            {
                continue;
            }
            if (IS_AVAIL(avail_cu, AVAIL_UP) && !IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_T_C) 
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_L_C) 
            {
                continue;
            }
#else
            if (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)
            {
                continue;
            }
#endif
#endif
#if PMC
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_MCPM_C || i == IPD_MCPM_T_C || i == IPD_MCPM_L_C))
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_LE) && i == IPD_MCPM_T_C)
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && i == IPD_MCPM_L_C)
            {
                continue;
            }
#endif

            cost_temp = pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 1, 0, x, y);
#if PRINT_CU_LEVEL_2
            printf("\nchro pred mode %2d cost_temp %10.1f", i, cost_temp);
            double val = 2815.9;
            if (cost_temp - val < 0.1 && cost_temp - val > -0.1)
            {
                int a = 0;
            }
#endif
            if (cost_temp < cost_best)
            {
                cost_best = cost_temp;
                best_dist_c = dist_t;
                best_ipd_c = i;
                for (j = U_C; j < N_C; j++)
                {
                    int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
                    com_mcpy(bst_info->coef[j], mod_info_curr->coef[j], size_tmp * sizeof(s16));
                    com_mcpy(bst_info->rec[j], pi->rec[j], size_tmp * sizeof(pel));
                    bst_info->num_nz[TB0][j] = mod_info_curr->num_nz[TB0][j];
                }
            }
        }
    }

    int pb_part_size_best = core->best_pb_part_intra;
    int tb_part_size_best = get_tb_part_size_by_pb(pb_part_size_best, MODE_INTRA);
    int num_pb_best = get_part_num(pb_part_size_best);
    int num_tb_best = get_part_num(tb_part_size_best);
    //some check
    if (ctx->tree_status == TREE_LC)
    {
        assert(best_ipd_c != IPD_INVALID);
    }
    else if (ctx->tree_status == TREE_L)
    {
        assert(bst_info->num_nz[TBUV0][U_C] == 0);
        assert(bst_info->num_nz[TBUV0][V_C] == 0);
        assert(best_dist_c == 0);
    }
    else if (ctx->tree_status == TREE_C)
    {
        assert(best_ipd_c != IPD_INVALID);
        assert(bst_info->num_nz[TBUV0][Y_C] == 0);
        assert(num_pb_best == 1 && num_tb_best == 1);
        assert(best_dist_y == 0);
    }
    else
        assert(0);
    //end of checks

    /*******************************    更新当前CU最优划分信息及划分下相关参数信息    *******************************/

    for (pb_part_idx = 0; pb_part_idx < num_pb_best; pb_part_idx++)//对于 bestCU 模式下每个PU
    {
        core->mod_info_best.mpm[pb_part_idx][0] = best_mpm[pb_part_idx][0];//更新mpm信息
        core->mod_info_best.mpm[pb_part_idx][1] = best_mpm[pb_part_idx][1];
        core->mod_info_best.ipm[pb_part_idx][0] = (s8)best_ipd[pb_part_idx];//更新最优IPM（亮度）
        core->mod_info_best.ipm[pb_part_idx][1] = (s8)best_ipd_c;//更新最优IPM（色度）
        mod_info_curr->ipm[pb_part_idx][0] = core->mod_info_best.ipm[pb_part_idx][0];/*置当前模式为 最优的模式*/
        mod_info_curr->ipm[pb_part_idx][1] = core->mod_info_best.ipm[pb_part_idx][1];
    }
    for (int tb_part_idx = 0; tb_part_idx < num_tb_best; tb_part_idx++)
    {
        for (j = 0; j < N_C; j++)
        {
            mod_info_curr->num_nz[tb_part_idx][j] = bst_info->num_nz[tb_part_idx][j];
        }
    }
#if IST
    mod_info_curr->ist_tu_flag = bst_info->ist_tu_flag;
#endif
#if EST
    mod_info_curr->est_flag = bst_info->est_flag;
#endif
#if TB_SPLIT_EXT
    mod_info_curr->pb_part = core->best_pb_part_intra;
    mod_info_curr->tb_part = core->best_tb_part_intra;
#endif
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    //enc_sbac_bit_reset(&core->s_temp_run);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_intra(ctx, core, ctx->info.pic_header.slice_type, bst_info->coef);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;
#if DT_SAVE_LOAD
    if (pData->num_intra_history < 2 && core->mod_info_curr.ipf_flag == 0 && ctx->tree_status != TREE_C)
    {
        pData->best_part_size_intra[pData->num_intra_history++] = core->best_pb_part_intra;
    }
#endif
    return cost_best;
}

#if USE_SP
static void init_sm_codingunit(ENC_CTX *ctx, ENC_CORE *core, COM_SP_CODING_UNIT* cur_sp_info)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    cur_sp_info->cu_pix_x          = mod_info_curr->x_pos;
    cur_sp_info->cu_pix_y          = mod_info_curr->y_pos;
    cur_sp_info->qp                = core->qp_y;
    cur_sp_info->lamda             = ctx->lambda[0];
    cur_sp_info->cu_width_log2     = mod_info_curr->cu_width_log2;
    cur_sp_info->cu_height_log2    = mod_info_curr->cu_height_log2;
    cur_sp_info->map_scu           = ctx->map.map_scu;
    cur_sp_info->pic_width_in_scu  = ctx->info.pic_width_in_scu;
    cur_sp_info->pic_height_in_scu = ctx->info.pic_height_in_scu;
    cur_sp_info->scup              = mod_info_curr->scup;
    cur_sp_info->tree_status       = ctx->tree_status;
    cur_sp_info->string_prediction_mode_flag = FALSE;
    cur_sp_info->max_str_cnt       = 1 << (cur_sp_info->cu_width_log2 + cur_sp_info->cu_height_log2 - 2);
    cur_sp_info->cur_bst_rdcost    = core->mod_info_best.cur_bst_rdcost;
    ENC_PARENT_INFO *parent_info = &core->cu_parent_info;
    cur_sp_info->p_x               = parent_info->p_x;
    cur_sp_info->p_y               = parent_info->p_y;
    cur_sp_info->p_width           = 1 << parent_info->p_width_log2;
    cur_sp_info->p_height          = 1 << parent_info->p_height_log2;
    cur_sp_info->is_sp_pix_completed  = FALSE;
    parent_info->p_sp_dir = core->cu_data_best[parent_info->p_width_log2 - 2][parent_info->p_height_log2 - 2].
        sp_copy_direction[((cur_sp_info->cu_pix_y - cur_sp_info->p_y) >> MIN_CU_LOG2) * (cur_sp_info->p_width >> MIN_CU_LOG2) + ((cur_sp_info->cu_pix_x - cur_sp_info->p_x) >> MIN_CU_LOG2)];
    cur_sp_info->chroma_weight[0]  = ctx->dist_chroma_weight[0];
    cur_sp_info->chroma_weight[1]  = ctx->dist_chroma_weight[1];
    cur_sp_info->p_cand            = core->parent_offset;
    cur_sp_info->p_cand_num        = core->p_offset_num;
    cur_sp_info->b_cand            = core->brother_offset;
    cur_sp_info->b_cand_num        = core->b_offset_num;
    cur_sp_info->n_cand            = core->n_recent_offset;
    cur_sp_info->n_cand_num        = core->n_offset_num;
    cur_sp_info->is_sp_skip_non_scc    = core->sp_skip_non_scc;
}

static u8 is_dir_skip(COM_MODE *bst_info, double distortion,u8 skipNonSCC)
{
    int w = bst_info->cu_width;
    int h = bst_info->cu_height;
    if (bst_info->sp_flag)
    {
        if (bst_info->sub_string_no == 1 && bst_info->string_copy_info[0].length == w * h)
        {
            return 1;
        }
    }
    if (skipNonSCC)
    {
        return 1;
    }
    return 0;
}

static void copy_to_bst_mode(COM_MODE *bst_info, COM_SP_CODING_UNIT* cur_sp_info)
{
    int width = 1 << cur_sp_info->cu_width_log2;
    int height = 1 << cur_sp_info->cu_height_log2;
    int size;
#if IST
    bst_info->ist_tu_flag = 0;
#endif
#if INTERPF
    bst_info->inter_filter_flag = 0;
#endif
    bst_info->skip_idx = 0;
    bst_info->ibc_flag = 0;
#if SBT
    bst_info->sbt_info = 0;
#endif
    bst_info->umve_flag = 0;
    bst_info->mvr_idx = 0;
#if IBC_ABVR
    bst_info->bvr_idx = 0;
#endif
#if EXT_AMVR_HMVP
    bst_info->mvp_from_hmvp_flag = 0;
#endif
    for (int lidx = 0; lidx < REFP_NUM; lidx++)
    {
        bst_info->mv[lidx][MV_X] = 0;
        bst_info->mv[lidx][MV_Y] = 0;
        bst_info->mvd[lidx][MV_X] = 0;
        bst_info->mvd[lidx][MV_Y] = 0;
    }
#if SMVD
    bst_info->smvd_flag = 0;
#endif
    bst_info->affine_flag = 0;
    if (cur_sp_info->string_prediction_mode_flag) 
    {
        bst_info->sub_string_no = (u16)cur_sp_info->sub_string_no;
        bst_info->sp_copy_direction = cur_sp_info->string_copy_direction;
        COM_SP_INFO *p_str_info = cur_sp_info->p_string_copy_info;
        memcpy(bst_info->string_copy_info, p_str_info, bst_info->sub_string_no * sizeof(COM_SP_INFO));
        bst_info->sp_flag = TRUE;
        bst_info->num_nz[0][Y_C] = 0;
        bst_info->num_nz[0][U_C] = 0;
        bst_info->num_nz[0][V_C] = 0;
        SET_REFI(bst_info->refi, REFI_INVALID, REFI_INVALID);
    }
    bst_info->is_sp_pix_completed = cur_sp_info->is_sp_pix_completed;
    size = width * height * sizeof(pel);
    memcpy(bst_info->rec[Y_C], cur_sp_info->rec[Y_C], size);
    size = (width * height * sizeof(pel)) >> 2;
    memcpy(bst_info->rec[U_C], cur_sp_info->rec[U_C], size);
    memcpy(bst_info->rec[V_C], cur_sp_info->rec[V_C], size);
#if PLATFORM_GENERAL_DEBUG
    com_mset(bst_info->coef[Y_C], 0, sizeof(s16) * width * height);
    com_mset(bst_info->coef[U_C], 0, sizeof(s16) * width * height >> 2);
    com_mset(bst_info->coef[V_C], 0, sizeof(s16) * width * height >> 2);
#endif
}

double analyze_sm_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PARENT_INFO *parent_info = &core->cu_parent_info;
    if (parent_info->c_sumRDCost >= parent_info->p_RDCost)
    {
        return MAX_COST;
    }
    int bit_cnt = 0;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->ist_tu_flag = 0;
#endif
#if INTERPF
    mod_info_curr->inter_filter_flag = 0;
#endif
#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, 1 << cu_width_log2, 1 << cu_height_log2, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, 1 << cu_width_log2, 1 << cu_height_log2, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
    mod_info_curr->cu_mode = MODE_IBC;
    int scu_x = 1 << (cu_width_log2 - 2);
    int scu_y = 1 << (cu_height_log2 - 2);
    int pic_width_in_scu = ctx->info.pic_width_in_scu;
    u32 *scu = ctx->map.map_scu + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++) 
    {
        for (int i = 0; i < scu_x; i++) 
        {
            MCU_CLR_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
    /*send the parameters of the sm mode*/
    COM_SP_CODING_UNIT cu;
    COM_SP_CODING_UNIT* cur_sp_info = &cu;
    init_sm_codingunit(ctx, core, cur_sp_info);
    double min_sp_rdcost = core->cost_best;
    double cur_sp_rdcost = MAX_COST;
    double cur_sp_dist;
    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        cur_sp_rdcost = cur_sp_info->cur_bst_rdcost;
    }
    cur_sp_info->string_prediction_mode_flag = TRUE;
    if (0 == IS_VALID_SP_CU_SIZE((1 << cu_width_log2), (1 << cu_height_log2))) 
    {
        cur_sp_info->string_prediction_mode_flag = FALSE;
    }
    // doing SP search
    if (cur_sp_info->string_prediction_mode_flag == TRUE) 
    {
        u8 skip_non_scc = FALSE;
        u8 curr_skip_non_scc = cur_sp_info->is_sp_skip_non_scc;
        //first direction
#if SP_BASE
        cur_sp_info->string_copy_direction = 1;
#else
        cur_sp_info->string_copy_direction = core->cu_parent_info.p_sp_dir;
#endif
        assert(core->cu_parent_info.p_sp_dir == 0 || core->cu_parent_info.p_sp_dir == 1);
        if (sm_mode_rdcost(ctx->sp_encoder, cur_sp_info, &cur_sp_rdcost, &cur_sp_dist))
        {
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_sp(ctx, core, cur_sp_info->p_string_copy_info, cur_sp_info->string_copy_direction, 1);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cur_sp_rdcost = (double)cur_sp_dist + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cur_sp_rdcost < min_sp_rdcost) 
            {
                min_sp_rdcost = cur_sp_rdcost;
                bst_info->slice_type = ctx->slice_type;
                copy_to_bst_mode(bst_info, cur_sp_info); 
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
            }
        }
        skip_non_scc = skip_non_scc || cur_sp_info->is_sp_skip_non_scc;
        cur_sp_info->is_sp_skip_non_scc = curr_skip_non_scc;
#if !SP_BASE
        //sp second direction
        cur_sp_info->string_copy_direction = !cur_sp_info->string_copy_direction;
        if (!is_dir_skip(bst_info, cur_sp_dist, skip_non_scc))
        {
            if (sm_mode_rdcost(ctx->sp_encoder, cur_sp_info, &cur_sp_rdcost, &cur_sp_dist))
            {
                SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                bit_cnt = enc_get_bit_number(&core->s_temp_run);
                enc_bit_est_sp(ctx, core, cur_sp_info->p_string_copy_info, cur_sp_info->string_copy_direction, 1);
                bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                cur_sp_rdcost = (double)cur_sp_dist + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                if (cur_sp_rdcost < min_sp_rdcost) 
                {
                    min_sp_rdcost = cur_sp_rdcost;
                    bst_info->slice_type = ctx->slice_type;
                    copy_to_bst_mode(bst_info, cur_sp_info);
                    SBAC_STORE(core->s_temp_best, core->s_temp_run);
                }
            }
        }
        skip_non_scc = skip_non_scc || cur_sp_info->is_sp_skip_non_scc;
        core->sp_skip_non_scc = skip_non_scc;
        //second dir end
#endif
        cur_sp_info->string_prediction_mode_flag = FALSE;
    }
    return min_sp_rdcost;
}
#endif

#if IPCM
double analyze_ipcm_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PINTRA *pi = &ctx->pintra;
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int i, j, s_org, s_org_c;

    int best_ipd[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_c = IPD_INVALID;
    s32 best_dist_y = 0, best_dist_c = 0;
    int bit_cnt = 0;
    pel *org;
    pel *org_cb, *org_cr;
    double cost_best = MAX_COST;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if SBT
    mod_info_curr->sbt_info = 0;
#endif

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        s_org = pi->stride_org[Y_C];
        org = pi->addr_org[Y_C] + (y * s_org) + x;
        int pb_part_idx = 0;
        core->best_pb_part_intra = SIZE_2Nx2N;

        for (i = 0; i < cu_height; i++)
        {
            for (j = 0; j < cu_width; j++)
            {
                bst_info->rec[Y_C][i * cu_width + j] = org[i * s_org + j];
                bst_info->coef[Y_C][i * cu_width + j] = org[i * s_org + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
            }
        }

        int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
        int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
        int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
        int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
        int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
        int pb_x_scu = PEL2SCU(pb_x);
        int pb_y_scu = PEL2SCU(pb_y);

#if FIMC
        if (ctx->info.sqh.fimc_enable_flag && ctx->info.pic_header.fimc_pic_flag)
        {
            com_get_cntmpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx], &core->cntmpm_cands);
        }
        else
        {
#endif
            com_get_mpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx]);
#if FIMC
        }
#endif
        update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, IPD_IPCM);

        best_ipd[PB0] = IPD_IPCM;

        mod_info_curr->num_nz[0][0] = bst_info->num_nz[0][0] = 0;
        copy_rec_y_to_pic(bst_info->rec[Y_C] + (pb_y - y) * cu_width + (pb_x - x), pb_x, pb_y, pb_w, pb_h, cu_width, PIC_REC(ctx));
        com_mcpy(pi->rec[Y_C], bst_info->rec[Y_C], cu_width * cu_height * sizeof(pel));
#if RDO_DBK
        calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, 0, NULL, NULL, 0);
        best_dist_y += (s32)ctx->delta_dist; //add delta SSD to the first PB
#endif
    }

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_C)
    {
        s_org_c = pi->stride_org[U_C];
        org_cb = pi->addr_org[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
        org_cr = pi->addr_org[V_C] + ((y >> 1) * s_org_c) + (x >> 1);
        core->best_pb_part_intra = SIZE_2Nx2N;

#if CHROMA_NOT_SPLIT
        //get luma pred mode
        if (ctx->tree_status == TREE_C)
        {
            assert(cu_width >= 8 && cu_height >= 8);
            int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
            best_ipd[PB0] = ctx->map.map_ipm[luma_scup];
            assert((best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT) || best_ipd[PB0] == IPD_IPCM);
#if USE_IBC
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]) || MCU_GET_IBC(ctx->map.map_scu[luma_scup]));
#else
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]));
#endif
        }
#endif
        if (best_ipd[PB0] == IPD_IPCM)
        {
            best_ipd_c = IPD_DM_C;

            for (i = 0; i < cu_height / 2; i++)
            {
                for (j = 0; j < cu_width / 2; j++)
                {
                    bst_info->rec[U_C][i * cu_width / 2 + j] = org_cb[i * s_org_c + j];
                    bst_info->coef[U_C][i * cu_width / 2 + j] = org_cb[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    bst_info->rec[V_C][i * cu_width / 2 + j] = org_cr[i * s_org_c + j];
                    bst_info->coef[V_C][i * cu_width / 2 + j] = org_cr[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                }
            }
        }
        else
        {
            return MAX_COST;
        }

        for (j = 1; j < N_C; j++)
        {
            mod_info_curr->num_nz[0][j] = bst_info->num_nz[0][j] = 0;
        }
    }

    int pb_part_idx = 0;

    core->mod_info_best.mpm[pb_part_idx][0] = mod_info_curr->mpm[pb_part_idx][0];
    core->mod_info_best.mpm[pb_part_idx][1] = mod_info_curr->mpm[pb_part_idx][1];
    core->mod_info_best.ipm[pb_part_idx][0] = best_ipd[PB0];
    core->mod_info_best.ipm[pb_part_idx][1] = best_ipd_c;
    mod_info_curr->ipm[pb_part_idx][0] = core->mod_info_best.ipm[pb_part_idx][0];
    mod_info_curr->ipm[pb_part_idx][1] = core->mod_info_best.ipm[pb_part_idx][1];

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_intra(ctx, core, ctx->info.pic_header.slice_type, bst_info->coef);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;

    return cost_best;
}
#endif

int pintra_set_complexity(ENC_CTX * ctx, int complexity)
{
    ENC_PINTRA * pi;
    pi = &ctx->pintra;
    pi->complexity = complexity;
    return COM_OK;
}

