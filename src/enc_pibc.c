/* ====================================================================================================================

The copyright in this software is being made available under the License included below.
This software may be subject to other third party and contributor rights, including patent rights, and no such
rights are granted under this license.

Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.
Copyright (c) 2019, TENCENT CO., LTD. All rights reserved.
Copyright (c) 2019, WUHAN UNIVERSITY. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted only for
the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
promoting such standards. The following conditions are required to be met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and
the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
the following disclaimer in the documentation and/or other materials provided with the distribution.
* The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. or TENCENT CO., LTD. may not be used to endorse or promote products derived from
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

#include <math.h>

#include "enc_def.h"
#include "enc_ibc_hash_wrapper.h"

#if USE_IBC
#define SWAP(a, b, t) { (t) = (a); (a) = (b); (b) = (t); }

#define CHROMA_REFINEMENT_CANDIDATES         8  /* 8 candidates BV to choose from */
#define MV_FRACTIONAL_BITS_INTERNAL          0

static s16    resi_t[N_C][MAX_CU_DIM];

#if TB_SPLIT_EXT
static u32 calc_sad_16b(int pu_w, int pu_h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
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
        cost += enc_sad_16b(seg_w_log2, seg_h_log2, s1, s2, s_src1, s_src2, bit_depth);
        return cost;
    }

    for (int j = 0; j < num_seg_in_pu_h; j++)
    {
        for (int i = 0; i < num_seg_in_pu_w; i++)
        {
            src1_seg = s1 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src1;
            src2_seg = s2 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src2;
            cost += enc_sad_16b(seg_w_log2, seg_h_log2, src1_seg, src2_seg, s_src1, s_src2, bit_depth);
        }
    }
    return cost;
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
#endif

void check_best_ibc_mode(ENC_CORE *core, ENC_PIBC *pi, const double cost_curr, double *cost_best)
{
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *cur_info = &core->mod_info_curr;

    if (cost_curr < *cost_best)
    {
        int j, lidx;
        int cu_width_log2 = cur_info->cu_width_log2;
        int cu_height_log2 = cur_info->cu_height_log2;
        int cu_width = 1 << cu_width_log2;
        int cu_height = 1 << cu_height_log2;
        bst_info->cu_mode = cur_info->cu_mode;
        bst_info->skip_idx = cur_info->skip_idx;
        bst_info->ibc_flag = 1;
#if IST
#if ISTS
        bst_info->ist_tu_flag = cur_info->ist_tu_flag;
#else
        bst_info->ist_tu_flag = 0;
#endif
#endif
#if INTERPF
        bst_info->inter_filter_flag = 0;
#endif

#if TB_SPLIT_EXT
        check_tb_part(cur_info);
        bst_info->pb_part = cur_info->pb_part;
        bst_info->tb_part = cur_info->tb_part;
        memcpy(&bst_info->pb_info, &cur_info->pb_info, sizeof(COM_PART_INFO));
        memcpy(&bst_info->tb_info, &cur_info->tb_info, sizeof(COM_PART_INFO));
#endif
#if SBT
        assert( cur_info->sbt_info == 0 );
        bst_info->sbt_info = cur_info->sbt_info;
#endif
        bst_info->umve_flag = 0;

        *cost_best = cost_curr;
        core->cost_best = cost_curr;

        SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
        
#if EXT_AMVR_HMVP
        bst_info->mvp_from_hmvp_flag = 0;
#endif
#if IBC_ABVR
        bst_info->bvr_idx = pi->curr_bvr;
#endif
        bst_info->mvr_idx = 0;
#if IBC_BVP
        bst_info->cbvp_idx = cur_info->cbvp_idx;
#endif

        bst_info->refi[REFP_0] = cur_info->refi[REFP_0];
        bst_info->refi[REFP_1] = cur_info->refi[REFP_1];
        for (lidx = 0; lidx < REFP_NUM; lidx++)
        {
            bst_info->mv[lidx][MV_X] = cur_info->mv[lidx][MV_X];
            bst_info->mv[lidx][MV_Y] = cur_info->mv[lidx][MV_Y];
            bst_info->mvd[lidx][MV_X] = cur_info->mvd[lidx][MV_X];
            bst_info->mvd[lidx][MV_Y] = cur_info->mvd[lidx][MV_Y];
        }
#if SMVD
        bst_info->smvd_flag = 0;
#endif

        bst_info->affine_flag = 0;
#if AFFINE_UMVE
        bst_info->affine_umve_flag = 0;
#endif

#if AWP
        bst_info->awp_flag = 0;
#endif

#if AWP_MVR
        bst_info->awp_mvr_flag0 = 0;
        bst_info->awp_mvr_flag1 = 0;
#endif

        for (j = 0; j < N_C; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            cu_plane_nz_cpy(bst_info->num_nz, cur_info->num_nz, j);
            com_mcpy(bst_info->pred[j], cur_info->pred[j], size_tmp * sizeof(pel));
            com_mcpy(bst_info->coef[j], cur_info->coef[j], size_tmp * sizeof(s16));
        }
    }
}

static double pibc_residue_rdo(ENC_CTX *ctx, ENC_CORE *core, int bForceAllZero)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s16(*mv)[MV_D] = mod_info_curr->mv;
    s8 *refi = mod_info_curr->refi;
    ENC_PIBC *pi = &ctx->pibc;
    pel(*pred)[MAX_CU_DIM] = mod_info_curr->pred;
    int bit_depth = ctx->info.bit_depth_internal;
    int(*num_nz_coef)[N_C], tnnz, width[N_C], height[N_C], log2_w[N_C], log2_h[N_C];
    pel(*rec)[MAX_CU_DIM];
    s64    dist[2][N_C], dist_pred[N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_best[N_C], nnz_store[MAX_NUM_TB][N_C], tb_part_store;
    int    bit_cnt;
    int    i, j, cbf_y, cbf_u, cbf_v;
    pel   *org[N_C];
    double cost_comp_best = MAX_COST;
    int    cbf_comps[N_C] = { 0, };
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
    int use_alt4x4Trans = 0;
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif

#if RDO_DBK
    u8  is_from_mv_field = 0;
#endif

#if !BD_AFFINE_AMVR
    if (mod_info_curr->affine_flag)
    {
        pi->curr_mvr = 0;
    }
#endif

    rec = mod_info_curr->rec;
    num_nz_coef = mod_info_curr->num_nz;
    width[Y_C] = 1 << cu_width_log2;
    height[Y_C] = 1 << cu_height_log2;
    width[U_C] = width[V_C] = 1 << (cu_width_log2 - 1);
    height[U_C] = height[V_C] = 1 << (cu_height_log2 - 1);
    log2_w[Y_C] = cu_width_log2;
    log2_h[Y_C] = cu_height_log2;
    log2_w[U_C] = log2_w[V_C] = cu_width_log2 - 1;
    log2_h[U_C] = log2_h[V_C] = cu_height_log2 - 1;
    org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
    org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
    org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
    com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv[0], pi->pic_unfiltered_rec, pred, ctx->tree_status, bit_depth);
    /* get residual */
    enc_diff_pred(x, y, cu_width_log2, cu_height_log2, pi->pic_org, pred, resi_t);

    for (i = 0; i < N_C; i++)
    {
        dist[0][i] = dist_pred[i] = enc_ssd_16b(log2_w[i], log2_h[i], pred[i], org[i], width[i], pi->stride_org[i], bit_depth);
    }
#if RDO_DBK
    calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pred, cu_width, x, y, 0, 0, refi, mv, is_from_mv_field);
    dist[0][Y_C] += ctx->delta_dist;
#endif

    /* test all zero case */
    memset(cbf_best, 0, sizeof(int) * N_C);
    memset(num_nz_coef, 0, sizeof(int) * MAX_NUM_TB * N_C);
    mod_info_curr->tb_part = SIZE_2Nx2N;
    if (ctx->tree_status == TREE_LC)
    {
        cost_best = (double)dist[0][Y_C] + ((dist[0][U_C] + dist[0][V_C]) * ctx->dist_chroma_weight[0]);
    }
    else
    {
        assert(ctx->tree_status == TREE_L);
        cost_best = (double)dist[0][Y_C];
    }

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    

    /* transform and quantization */
    bForceAllZero |= 0;

    if (!bForceAllZero)
    {
#if TR_EARLY_TERMINATE
        core->dist_pred_luma = dist_pred[Y_C];
#endif

#if USE_IBC //support N*N TB
        core->best_tb_part_hist = 255; //enable this line to bypass the save load mechanism
#endif

        tnnz = enc_tq_yuv_nnz(ctx, core, mod_info_curr, coef, resi_t, pi->slice_type, 0, use_secTrans, use_alt4x4Trans, refi, mv, is_from_mv_field);

        if (tnnz)
        {
            com_itdq_yuv(mod_info_curr, coef, resi_t, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans);
            for (i = 0; i < N_C; i++)
            {
                com_recon(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, resi_t[i], pred[i], num_nz_coef, i, width[i], height[i], width[i], rec[i], bit_depth
#if SBT
                    , 0
#endif
                );

                if (is_cu_plane_nz(num_nz_coef, i))
                {
                    dist[1][i] = enc_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], width[i], pi->stride_org[i], bit_depth);
                }
                else
                {
                    dist[1][i] = dist_pred[i];
                }
            }

#if RDO_DBK
            //filter rec and calculate ssd
            calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, rec, cu_width, x, y, 0, 1, refi, mv, is_from_mv_field);
            dist[1][Y_C] += ctx->delta_dist;
#endif

            cbf_y = is_cu_plane_nz(num_nz_coef, Y_C) > 0 ? 1 : 0;
            cbf_u = is_cu_plane_nz(num_nz_coef, U_C) > 0 ? 1 : 0;
            cbf_v = is_cu_plane_nz(num_nz_coef, V_C) > 0 ? 1 : 0;

            if (ctx->tree_status == TREE_LC)
            {
                cost = (double)dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);
            }
            else
            {
                assert(ctx->tree_status == TREE_L);
                cost = (double)dist[cbf_y][Y_C];
            }

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            //enc_sbac_bit_reset(&core->s_temp_run);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cost < cost_best)
            {
                cost_best = cost;
                cbf_best[Y_C] = cbf_y;
                cbf_best[U_C] = cbf_u;
                cbf_best[V_C] = cbf_v;
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
            }
            SBAC_LOAD(core->s_temp_prev_comp_best, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);

            //    /* cbf test for each component */
            com_mcpy(nnz_store, num_nz_coef, sizeof(int) * MAX_NUM_TB * N_C);
            tb_part_store = mod_info_curr->tb_part;
            if (ctx->tree_status == TREE_LC) // do not need to test for luma only
            {
                for (i = 0; i < N_C; i++)
                {
                    if (is_cu_plane_nz(nnz_store, i) > 0)
                    {
                        cost_comp_best = MAX_COST;
                        SBAC_LOAD(core->s_temp_prev_comp_run, core->s_temp_prev_comp_best);
                        for (j = 0; j < 2; j++)
                        {
                            cost = dist[j][i] * (i == 0 ? 1 : ctx->dist_chroma_weight[i - 1]);
                            if (j)
                            {
                                cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = tb_part_store;
                            }
                            else
                            {
                                cu_plane_nz_cln(num_nz_coef, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = SIZE_2Nx2N;
                            }

                            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_run);
                            //enc_sbac_bit_reset(&core->s_temp_run);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run);
                            enc_bit_est_inter_comp(ctx, core, coef[i], i);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                            cost += RATE_TO_COST_LAMBDA(ctx->lambda[i], bit_cnt);
                            if (cost < cost_comp_best)
                            {
                                cost_comp_best = cost;
                                cbf_comps[i] = j;
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                            }
                        }
                    }
                    else
                    {
                        cbf_comps[i] = 0;
                    }
                }

                // do not set if all zero, because the case of all zero has been tested
                if (cbf_comps[Y_C] != 0 || cbf_comps[U_C] != 0 || cbf_comps[V_C] != 0)
                {
                    for (i = 0; i < N_C; i++)
                    {
                        if (cbf_comps[i])
                        {
                            cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                            if (i == 0)
                                mod_info_curr->tb_part = tb_part_store;
                        }
                        else
                        {
                            cu_plane_nz_cln(num_nz_coef, i);
                            if (i == 0)
                                mod_info_curr->tb_part = SIZE_2Nx2N;
                        }
                    }

                    // if the best num_nz_coef is changed
                    if (!is_cu_nz_equ(num_nz_coef, nnz_store))
                    {
                        cbf_y = cbf_comps[Y_C];
                        cbf_u = cbf_comps[U_C];
                        cbf_v = cbf_comps[V_C];
                        cost = dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);

                        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                        //enc_sbac_bit_reset(&core->s_temp_run);
                        bit_cnt = enc_get_bit_number(&core->s_temp_run);
                        enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
                        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                        if (cost < cost_best)
                        {
                            cost_best = cost;
                            cbf_best[Y_C] = cbf_y;
                            cbf_best[U_C] = cbf_u;
                            cbf_best[V_C] = cbf_v;
                            SBAC_STORE(core->s_temp_best, core->s_temp_run);
                        }
                    }
                }
            }
            for (i = 0; i < N_C; i++)
            {
                if (cbf_best[i])
                {
                    cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                    if (i == 0)
                        mod_info_curr->tb_part = tb_part_store;
                }
                else
                {
                    cu_plane_nz_cln(num_nz_coef, i);
                    if (i == 0)
                        mod_info_curr->tb_part = SIZE_2Nx2N;
                }

                if (is_cu_plane_nz(num_nz_coef, i) == 0 && is_cu_plane_nz(nnz_store, i) != 0)
                {
                    com_mset(coef[i], 0, sizeof(s16) * ((cu_width * cu_height) >> (i == 0 ? 0 : 2)));
                }
            }
        }
    }

    if (!is_cu_plane_nz(num_nz_coef, Y_C))
        mod_info_curr->tb_part = SIZE_2Nx2N; // reset best tb_part if no residual Y

    return cost_best;
}

static void clip_ibc_mv(int rc_mv[2], int pic_width, int pic_height, int lcu_width, int lcu_height, int cu_pos_x, int cu_pos_y)
{
    int mv_shift = MV_FRACTIONAL_BITS_INTERNAL;
    int offset = 8;
    int hor_max = (pic_width + offset - cu_pos_x - 1) << mv_shift;
    int hor_min = (-lcu_width - offset - cu_pos_x + 1) << mv_shift;

    int ver_max = (pic_height + offset - cu_pos_y - 1) << mv_shift;
    int ver_min = (-lcu_height - offset - cu_pos_y + 1) << mv_shift;

    rc_mv[0] = COM_MIN(hor_max, COM_MAX(hor_min, rc_mv[0]));
    rc_mv[1] = COM_MIN(ver_max, COM_MAX(ver_min, rc_mv[1]));
}

static void ibc_set_search_range(ENC_CTX *ctx, ENC_CORE *core, int cu_pel_x, int cu_pel_y, int log2_cuw, int log2_cuh,
    const int local_search_range_x, const int local_search_range_y, int mv_search_range_left[2], int mv_search_range_right[2])
{
    int search_left = 0;
    int search_right = 0;
    int search_top = 0;
    int search_bottom = 0;

    const int roi_width = (1 << log2_cuw);
    const int roi_height = (1 << log2_cuh);

    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;

    search_left = -COM_MIN(cu_pel_x, local_search_range_x);
    search_top = -COM_MIN(cu_pel_y, local_search_range_y);

    search_right = COM_MIN(pic_width - cu_pel_x - roi_width, local_search_range_x);
    search_bottom = COM_MIN(pic_height - cu_pel_y - roi_height, local_search_range_y);

    mv_search_range_left[0] = search_left;
    mv_search_range_left[1] = search_top;
    mv_search_range_right[0] = search_right;
    mv_search_range_right[1] = search_bottom;

    clip_ibc_mv(mv_search_range_left, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
    clip_ibc_mv(mv_search_range_right, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
}

static void init_ibc_data(ENC_PIBC *pi, ENC_CORE *core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    mod_info_curr->skip_idx = 0;
#if SMVD
    mod_info_curr->smvd_flag = 0;
#endif
    get_part_info(pi->pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);

    com_mset(mod_info_curr->mv, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(mod_info_curr->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(mod_info_curr->refi, 0, sizeof(s8)  * REFP_NUM);

    com_mset(mod_info_curr->num_nz, 0, sizeof(int)*N_C*MAX_NUM_TB);
    com_mset(mod_info_curr->coef, 0, sizeof(s16) * N_C * MAX_CU_DIM);

    com_mset(mod_info_curr->affine_mv, 0, sizeof(CPMV) * REFP_NUM * VER_NUM * MV_D);
    com_mset(mod_info_curr->affine_mvd, 0, sizeof(s16) * REFP_NUM * VER_NUM * MV_D);
}

static void init_LOG_LUT(ENC_PIBC *pi)
{
    int size = sizeof(s8) * (MAX_CU_SIZE + 1);
    memset(pi->ctu_log2_tbl, 0, size);
    // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
    int c = 0;
    for (int i = 0, n = 0; i <= MAX_CU_SIZE; i++)
    {
        if (i == (1 << n))
        {
            c = n;
            n++;
        }

        pi->ctu_log2_tbl[i] = c;
    }
}

static void update_ibc_mv_cand(u32 sad, int x, int y, u32 *sad_best_cand, s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D])
{
    int j = CHROMA_REFINEMENT_CANDIDATES - 1;

    if (sad < sad_best_cand[CHROMA_REFINEMENT_CANDIDATES - 1])
    {
        for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
        {
            if (sad < sad_best_cand[t])
                j = t;
        }

        for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
        {
            sad_best_cand[k] = sad_best_cand[k - 1];

            mv_cand[k][0] = mv_cand[k - 1][0];
            mv_cand[k][1] = mv_cand[k - 1][1];
        }
        sad_best_cand[j] = sad;
        mv_cand[j][0] = x;
        mv_cand[j][1] = y;
    }
}

#if USE_IBC
static u32 getIComponentBits(int val)
{
    if (!val)
    {
        return 1;
    }

    u32 length = 1;
    u32 temp = (val <= 0) ? (-val << 1) + 1 : (val << 1);

    while (1 != temp)
    {
        temp >>= 1;
        length += 2;
    }

    return length;
}

u32 get_bv_cost_bits(int mv_x, int mv_y)
{
    return getIComponentBits(mv_x) + getIComponentBits(mv_y);
}
#else
__inline static u32 get_exp_golomb_bits(u32 abs_mvd)
{
  int bits = 0;
  int len_i, len_c, nn;
  /* abs(mvd) */
  nn = ((abs_mvd + 1) >> 1);
  for (len_i = 0; len_i < 16 && nn != 0; len_i++)
  {
    nn >>= 1;
  }
  len_c = (len_i << 1) + 1;
  bits += len_c;
  /* sign */
  if (abs_mvd)
  {
    bits++;
  }
  return bits;
}

int get_ibc_mv_bits(int mvd_x, int mvd_y)
{
    int bits = 0;
    bits = (mvd_x > 2048 || mvd_x <= -2048) ? get_exp_golomb_bits(COM_ABS(mvd_x)) : enc_tbl_mv_bits[mvd_x];
    bits += (mvd_y > 2048 || mvd_y <= -2048) ? get_exp_golomb_bits(COM_ABS(mvd_y)) : enc_tbl_mv_bits[mvd_y];
    return bits;
}
#endif

#if IBC_BVP
u32 get_bv_cost_bits_ibc_hash(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx)
{
    int idx;
    u32  curr_bits = COM_UINT32_MAX;
    u32  best_bits = COM_UINT32_MAX;

    if (core->cnt_class_cands == 0)
    {
        *bvp_idx = 0;
        best_bits = enc_bvd_bits[mv_x + 512] + enc_bvd_bits[mv_y + 512] + enc_bvp_idx_bits[*bvp_idx];
    }
    else
    {
        for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
        {
            curr_bits = enc_bvd_bits[mv_x - core->bvp_cands[idx][MV_X] + 512] + enc_bvd_bits[mv_y - core->bvp_cands[idx][MV_Y] + 512] + enc_bvp_idx_bits[idx];

            if (curr_bits <= best_bits)
            {
                *bvp_idx = idx;
                best_bits = curr_bits;

                if (best_bits < 5)
                {
                    goto end;
                }
            }
        }
    }
end:
    return best_bits;
}

static int ibc_hash_found_check_bvp(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 mv[MV_D], u8 *bvp_idx)
{
    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    u32 rui_cost = COM_UINT32_MAX;
    int mv_bits = 0, best_mv_bits = 0;
    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    u32 sad_bvp = COM_UINT32_MAX, sad_hash = COM_UINT32_MAX, sad_temp = COM_UINT32_MAX;
    u8 idx_best = 0;
    int x, y;
    x = mv[MV_X];
    y = mv[MV_Y];

    mv_bits = enc_bvd_bits[x - (core->bvp_cands[*bvp_idx][MV_X]) + 512] + enc_bvd_bits[y - (core->bvp_cands[*bvp_idx][MV_Y]) + 512] + enc_bvp_idx_bits[*bvp_idx];

    sad_hash = GET_MV_COST(ctx, mv_bits);
    ref = rec + ref_pic->stride_luma * y + x;
    sad_hash += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
    rui_cost = sad_hash;

    for (int idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
    {
        x = core->bvp_cands[idx][MV_X];
        y = core->bvp_cands[idx][MV_Y];

        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
            continue;
        }
        mv_bits = 2 + enc_bvp_idx_bits[idx];
        sad_temp = GET_MV_COST(ctx, mv_bits);
        ref = rec + ref_pic->stride_luma * y + x;
        sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
        if (sad_temp < sad_bvp)
        {
            sad_bvp = sad_temp;
            idx_best = idx;
        }
    }
    if (sad_bvp <= sad_hash)
    {
        *bvp_idx = idx_best;
        mv[MV_X] = core->bvp_cands[idx_best][MV_X];
        mv[MV_Y] = core->bvp_cands[idx_best][MV_Y];
        rui_cost = sad_bvp;
    }
    return rui_cost;
}

static void update_ibc_mv_cand_one(u32 sad, int x, int y, u32 sad_best_cand[1], s16 mv_cand[1][MV_D], u8 mvp_idx, u8  bvp_idx_cand[1], int mv_bits, int best_bv_bits[1])
{
    if (sad < sad_best_cand[0])
    {
        sad_best_cand[0] = sad;
        mv_cand[0][0] = x;
        mv_cand[0][1] = y;
        bvp_idx_cand[0] = mvp_idx;
        best_bv_bits[0] = mv_bits;
    }
    else if (sad == sad_best_cand[0])
    {
        if (mv_bits < best_bv_bits[0])
        {
            sad_best_cand[0] = sad;
            mv_cand[0][0] = x;
            mv_cand[0][1] = y;
            bvp_idx_cand[0] = mvp_idx;
            best_bv_bits[0] = mv_bits;
        }
    }
}

void get_bvd_bits_one_direction(ENC_CORE *core, u32 bvd_bits_table[MAX_NUM_BVP], int dir, int dir_value)
{
    int idx;
    for (idx = 0; idx < core->cnt_class_cands; idx++)
    {
        bvd_bits_table[idx] = enc_bvd_bits[dir_value - core->bvp_cands[idx][dir] + 512];
    }

}

u32 get_bv_cost_bits_ibc_local(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx, u32 bvd_bits_table[MAX_NUM_BVP], int dir)
{
    int  idx, mv_dir;
    u32  curr_bits = COM_UINT32_MAX;
    u32  best_bits = COM_UINT32_MAX;

    if (core->cnt_class_cands == 0)
    {
        *bvp_idx = 0;
        best_bits = enc_bvd_bits[mv_x + 512] + enc_bvd_bits[mv_y + 512] + enc_bvp_idx_bits[*bvp_idx];
    }
    else
    {
        if (dir == 1)
        {
            mv_dir = mv_x;
        }
        else
        {
            mv_dir = mv_y;
        }
        for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
        {
            curr_bits = enc_bvd_bits[mv_dir - core->bvp_cands[idx][1 - dir] + 512] + bvd_bits_table[idx] + enc_bvp_idx_bits[idx];

            if (curr_bits <= best_bits)
            {
                *bvp_idx = idx;
                best_bits = curr_bits;
                if (best_bits < 5)
                {
                    goto end;
                }
            }
        }
    }
end:
    return best_bits;
}

static void ibc_set_starting_point(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 starting_point[MV_D], u8* bvpasbv_idx)
{
    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    int mv_bits = 0, best_mv_bits = 0;
    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    u32 sad_bvp = COM_UINT32_MAX, sad_temp = COM_UINT32_MAX;
    u8 idx_best = 0;
    int x, y;
    int idx;


    for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
    {
        x = core->bvp_cands[idx][MV_X];
        y = core->bvp_cands[idx][MV_Y];

        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
            continue;
        }
        mv_bits = 2 + enc_bvp_idx_bits[idx];
        sad_temp = GET_MV_COST(ctx, mv_bits);
        ref = rec + ref_pic->stride_luma * y + x;
        sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
        if (sad_temp < sad_bvp)
        {
            sad_bvp = sad_temp;
            idx_best = idx;
        }
    }

    *bvpasbv_idx = idx_best;

    if (sad_bvp < 1000)
    {
        starting_point[MV_X] = core->bvp_cands[idx_best][MV_X];
        starting_point[MV_Y] = core->bvp_cands[idx_best][MV_Y];
    }
}
#endif

#if IBC_BVP
static int pibc_search_estimation(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D], u8 *bvp_idx, s16 starting_point[MV_D], u8 *bvpasbv_idx)
#else
static int pibc_search_estimation(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D])
#endif
{
    int mv_search_range_left[2] = { 0 };
    int mv_search_range_right[2] = { 0 };

    int srch_rng_hor_left = 0;
    int srch_rng_hor_right = 0;
    int srch_rng_ver_top = 0;
    int srch_rng_ver_bottom = 0;

    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int pu_pel_offset_x = 0;
    const int pu_pel_offset_y = 0;

    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

#if IBC_BVP
    const int bvp_x = starting_point[MV_X];
    const int bvp_y = starting_point[MV_Y];
    const int bvp_as_bv_idx = *bvpasbv_idx;
    const int cu_pel_x_new = starting_point[MV_X] + cu_x;
    const int cu_pel_y_new = starting_point[MV_Y] + cu_y;
#endif

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);

    u32 sad = 0;
    u32 sad_best = COM_UINT32_MAX;
    u32 rui_cost = COM_UINT32_MAX;
    int bestX = 0;
    int bestY = 0;
    int mv_bits = 0, best_mv_bits = 0;

    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    int best_cand_idx = 0;

#if IBC_BVP
    u32 sad_best_cand[1];
    s16 mv_cand[1][MV_D];
    u8 bvp_idx_cand[1];
    int best_bv_bits[1];
    u8 bvp_idx_temp;

    u32 bvd_bits_table[MAX_NUM_BVP] = { 0 };
    u8 bestIDX;

    ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right);
#else
    u32 sad_best_cand[CHROMA_REFINEMENT_CANDIDATES];
    s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D];

    ibc_set_search_range(ctx, core, cu_x, cu_y, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right);
#endif

    srch_rng_hor_left = mv_search_range_left[0];
    srch_rng_hor_right = mv_search_range_right[0];
    srch_rng_ver_top = mv_search_range_left[1];
    srch_rng_ver_bottom = mv_search_range_right[1];

#if !IBC_BVP
    mvp[MV_X] = 0;
    mvp[MV_Y] = 0;
#endif

#if IBC_BVP
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;

    sad_best_cand[0] = COM_UINT32_MAX;
    mv_cand[0][0] = 0;
    mv_cand[0][1] = 0;
    bvp_idx_cand[0] = 0;
    best_bv_bits[0] = COM_INT16_MAX;

    bvp_idx_temp = 0;
    bestIDX = 0;
    int bv_x_temp, bv_y_temp;

    if (bvp_as_bv_idx < core->cnt_class_cands)
    {
        u32 sad_temp = COM_UINT32_MAX;
        int x, y;
        x = core->bvp_cands[bvp_as_bv_idx][MV_X];
        y = core->bvp_cands[bvp_as_bv_idx][MV_Y];

        if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
            mv_bits = 2 + enc_bvp_idx_bits[bvp_as_bv_idx];
            sad_temp = GET_MV_COST(ctx, mv_bits);
            ref = rec + ref_pic->stride_luma * y + x;
            sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

            update_ibc_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, bvp_as_bv_idx, bvp_idx_cand, mv_bits, best_bv_bits);
        }
    }

#else
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
        sad_best_cand[cand] = COM_UINT32_MAX;
        mv_cand[cand][0] = 0;
        mv_cand[cand][1] = 0;
    }

    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
#endif

    {
        u32 tempSadBest = 0;

        int srLeft = srch_rng_hor_left, srRight = srch_rng_hor_right, srTop = srch_rng_ver_top, srBottom = srch_rng_ver_bottom;

#if ESTIMATION_IMPROVE
        int boundY = 0;
        if (starting_point[0] == 0 && starting_point[1] == 0)
            boundY = (0 - roi_height - pu_pel_offset_y);
        else
            boundY = (srch_rng_ver_bottom - pu_pel_offset_y);
#else
        const int boundY = (0 - roi_height - pu_pel_offset_y);
#endif
#if IBC_BVP
        bv_x_temp = 0 + bvp_x;
        get_bvd_bits_one_direction(core, bvd_bits_table, 0, bv_x_temp);

        for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y_new); y <= boundY; ++y)
#else
        for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y); y <= boundY; ++y)
#endif
        {
#if IBC_BVP
            bv_y_temp = y + bvp_y;

            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, 0, y, lcu_width))
#endif
            {
                continue;
            }

#if IBC_BVP
            mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 0);
#else
            mv_bits = get_bv_cost_bits(0, y);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
#if IBC_BVP
            ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
            ref = rec + ref_pic->stride_luma * y;
#endif
            sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
            update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
            update_ibc_mv_cand(sad, 0, y, sad_best_cand, mv_cand);
#endif
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }
        }

#if IBC_BVP
        const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x_new);
        bv_y_temp = 0 + bvp_y;
        get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
#else
        const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x);
#endif


#if ESTIMATION_IMPROVE
        int temp_x = 0;
        if (starting_point[0] == 0 && starting_point[1] == 0)
            temp_x = 0 - roi_width - pu_pel_offset_x;
        else
            temp_x = srch_rng_hor_right - pu_pel_offset_x;
        for (int x = temp_x; x >= boundX; --x)
#else
        for (int x = 0 - roi_width - pu_pel_offset_x; x >= boundX; --x)
#endif
        {
#if IBC_BVP
            bv_x_temp = x + bvp_x;
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, 0, lcu_width))
#endif
            {
                continue;
            }

#if IBC_BVP
            mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
            mv_bits = get_bv_cost_bits(x, 0);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
#if IBC_BVP
            ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
            ref = rec + x;
#endif
            sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
            update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
            update_ibc_mv_cand(sad, x, 0, sad_best_cand, mv_cand);
#endif
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }
        }

        bestX = mv_cand[0][0];
        bestY = mv_cand[0][1];
        sad_best = sad_best_cand[0];

#if IBC_BVP
        bestIDX = bvp_idx_cand[0];
        if (core->cnt_class_cands == 0)
        {
            mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512] + enc_bvp_idx_bits[bestIDX];
        }
        else
        {
            mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
        }
#endif

        sad = GET_MV_COST(ctx, mv_bits);
        if ((!bestX && !bestY) || (sad_best - sad <= 32))
        {
            best_cand_idx = 0;
            bestX = mv_cand[best_cand_idx][0];
            bestY = mv_cand[best_cand_idx][1];
            sad_best = sad_best_cand[best_cand_idx];
            mv[0] = bestX;
            mv[1] = bestY;
            rui_cost = sad_best;
#if IBC_BVP
            bestIDX = bvp_idx_cand[0];
            *bvp_idx = bestIDX;
#endif
            goto end;
        }

        if ((1 << log2_cuw) < 16 && (1 << log2_cuh) < 16)
        {
#if IBC_BVP
            for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y_new); y <= srch_rng_ver_bottom; y += 2)
#else
            for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y); y <= srch_rng_ver_bottom; y += 2)
#endif
            {
#if IBC_BVP
                if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                    continue;

#if IBC_BVP
                bv_y_temp = y + bvp_y;
                get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x++)
#else
                for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x++)
#endif
                {
#if IBC_BVP
                    if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                        continue;

#if IBC_BVP
                    bv_x_temp = x + bvp_x;
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                    {
                        continue;
                    }

#if IBC_BVP
                    mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                    mv_bits = get_bv_cost_bits(x, y);
#endif
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
#if IBC_BVP
                    ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                    ref = rec + y * ref_pic->stride_luma + x;
#endif
                    sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                    update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                }
            }

            bestX = mv_cand[0][0];
            bestY = mv_cand[0][1];
            sad_best = sad_best_cand[0];

#if IBC_BVP
            bestIDX = bvp_idx_cand[0];
            if (core->cnt_class_cands == 0)
            {
                mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512] + enc_bvp_idx_bits[bestIDX];
            }
            else
            {
                mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
            }
#else
            mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            if (sad_best - sad <= 16)
            {
                best_cand_idx = 0;
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[best_cand_idx];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }

#if IBC_BVP
            for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y_new) + 1); y <= srch_rng_ver_bottom; y += 2)
#else
            for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
#endif
            {
#if IBC_BVP
                if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                    continue;

#if IBC_BVP
                bv_y_temp = y + bvp_y;
                get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x += 2)
#else
                for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x += 2)
#endif
                {
#if IBC_BVP
                    if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                        continue;

#if IBC_BVP
                    bv_x_temp = x + bvp_x;
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                        pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                    {
                        continue;
                    }

#if IBC_BVP
                    mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                    mv_bits = get_bv_cost_bits(x, y);
#endif
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
#if IBC_BVP
                    ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                    ref = rec + y * ref_pic->stride_luma + x;
#endif
                    sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                    update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 5)
                    {
                        best_cand_idx = 0;
                        bestX = mv_cand[best_cand_idx][0];
                        bestY = mv_cand[best_cand_idx][1];
                        sad_best = sad_best_cand[best_cand_idx];
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
#if IBC_BVP
                        bestIDX = bvp_idx_cand[0];
                        *bvp_idx = bestIDX;
#endif
                        goto end;
                    }
                }
            }

            bestX = mv_cand[0][0];
            bestY = mv_cand[0][1];
            sad_best = sad_best_cand[0];

#if IBC_BVP
            bestIDX = bvp_idx_cand[0];
            if (core->cnt_class_cands == 0)
            {
                mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512] + enc_bvp_idx_bits[bestIDX];
            }
            else
            {
                mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
            }
#else
            mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            if ((sad_best >= tempSadBest) || ((sad_best - sad) <= 32))
            {
                best_cand_idx = 0;
                bestX = mv_cand[best_cand_idx][0];
                bestY = mv_cand[best_cand_idx][1];
                sad_best = sad_best_cand[best_cand_idx];
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }

            tempSadBest = sad_best_cand[0];

#if IBC_BVP
            for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y_new) + 1); y <= srch_rng_ver_bottom; y += 2)
#else
            for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
#endif
            {
#if IBC_BVP
                if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                    continue;

#if IBC_BVP
                bv_y_temp = y + bvp_y;
                get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                for (int x = (COM_MAX(srch_rng_hor_left, -cu_pel_x_new) + 1); x <= srch_rng_hor_right; x += 2)
#else
                for (int x = (COM_MAX(srch_rng_hor_left, -cu_pel_x) + 1); x <= srch_rng_hor_right; x += 2)
#endif
                {

#if IBC_BVP
                    if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                        continue;

#if IBC_BVP
                    bv_x_temp = x + bvp_x;
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                        pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                    {
                        continue;
                    }

#if IBC_BVP
                    mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                    mv_bits = get_bv_cost_bits(x, y);
#endif
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
#if IBC_BVP
                    ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                    ref = rec + y * ref_pic->stride_luma + x;
#endif
                    sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                    update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 5)
                    {
                        best_cand_idx = 0;
                        bestX = mv_cand[best_cand_idx][0];
                        bestY = mv_cand[best_cand_idx][1];
                        sad_best = sad_best_cand[best_cand_idx];
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
#if IBC_BVP
                        bestIDX = bvp_idx_cand[0];
                        *bvp_idx = bestIDX;
#endif
                        goto end;
                    }
                }
            }
        }
    }

    best_cand_idx = 0;
    bestX = mv_cand[best_cand_idx][0];
    bestY = mv_cand[best_cand_idx][1];
    sad_best = sad_best_cand[best_cand_idx];
    mv[0] = bestX;
    mv[1] = bestY;
    rui_cost = sad_best;
#if IBC_BVP
    bestIDX = bvp_idx_cand[0];
    *bvp_idx = bestIDX;
#endif

end:
    //if (roi_width + roi_height > 8)
    //{
    //    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, mv_cand, CHROMA_REFINEMENT_CANDIDATES);

    //    if (roi_width + roi_height == 32)
    //    {
    //        m_numBV16s = m_numBVs;
    //    }
    //}

    return rui_cost;
}

static u32 pibc_me_search(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int x, int y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D]
#if IBC_BVP
    , u8 *bvp_idx
#endif
)
{
    u32 cost = 0;
    s16 mv_temp[MV_D] = { 0, 0 };

#if IBC_BVP
    if (ctx->param.ibc_hash_search_flag)
    {
        cost = search_ibc_hash_match(core, ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx);

        if ((!(mv_temp[0] == 0 && mv_temp[1] == 0)) && core->cnt_class_cands > 0)
        {
            cost = ibc_hash_found_check_bvp(ctx, core, pi, x, y, log2_cuw, log2_cuh, mv_temp, bvp_idx);
        }
    }

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        // if hash search does not work or is not enabled
        s16 starting_point[MV_D] = { 0,0 };
        u8  bvpasbv_idx = 0;
        ibc_set_starting_point(ctx, core, pi, x, y, log2_cuw, log2_cuh, starting_point, &bvpasbv_idx);
        cost = pibc_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx, starting_point, &bvpasbv_idx);
    }
#else
    if (ctx->param.ibc_hash_search_flag)
    {
        cost = search_ibc_hash_match(ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        // if hash search does not work or is not enabled
        cost = pibc_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }
#endif

    mv[0] = mv_temp[0];
    mv[1] = mv_temp[1];

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        return COM_UINT32_MAX;
    }

    return cost;
}

double analyze_ibc_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi = &ctx->pibc;
    COM_MODE *cur_info = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j;

    s16  coef_blk[N_C][MAX_CU_DIM];
    s16  resi[N_C][MAX_CU_DIM];

    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    cur_info->sbt_info = 0;
#endif
#if ISTS
    int istsflag = 0;
    cur_info->ist_tu_flag = 0;
    cur_info->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif

    int num_amvr = MAX_NUM_MVR;

    {
        num_amvr = 1; /* only allow 1/4 pel of resolution */
    }
    //pi->curr_mvr = 0;

#if IBC_ABVR
    int num_abvr = MAX_NUM_BVR;

    if (ctx->info.sqh.abvr_enable_flag)
    {
        num_abvr = MAX_NUM_BVR;
    }
    else
    {
        num_abvr = 1;
    }
#endif

    int allow_affine = 0;// disable affine for IBC

#if IBC_ABVR
    u8 found_available_ibc = 0;
    double cost_best = core->cost_best;
    u8 skip_me = 0;
    u32 mecost_fast;
    s16 mv_fast[MV_D];
#else
    u32 mecost, best_mecost;
    s16*mvp, *mv, *mvd;
    u8 mvp_idx = 0;
    double cost, cost_best = core->cost_best;
    u8 found_available_ibc = 0;
#endif

    u8 bvp_idx = 0;
    u8 bvp_idx_fast = 0;
#if IBC_ABVR
    mecost_fast = COM_UINT32_MAX;
#if IBC_BVP
    int bvr_idx = 0;
    for (bvr_idx = 0; bvr_idx < num_abvr; bvr_idx++)
#else
    pi->curr_bvr = 0;
#endif
    {
        u32 mecost, best_mecost;
        s16*mvp, *mv, *mvd;
        u8 mvp_idx = 0;
        double cost;
        found_available_ibc = 0;
#endif
        cur_info->cu_mode = MODE_IBC;
        cur_info->ibc_flag = 1;

        init_ibc_data(pi, core);

        mv = cur_info->mv[0];
        mvd = cur_info->mvd[0];
        best_mecost = COM_UINT32_MAX;

        mvp = pi->mvp;

        mvp[MV_X] = 0;
        mvp[MV_Y] = 0;

#if IBC_BVP
        // initialize pmv and refi
        for (i = 0;i < MAX_NUM_BVP;i++)
        {
            core->bvp_cands[i][MV_X] = 0;
            core->bvp_cands[i][MV_Y] = 0;
        }
        com_derive_bvp_list(ctx->info.sqh.num_of_hbvp_cand, &core->mod_info_curr, core->block_motion_cands, core->cnt_hbvp_cands, core->bvp_cands, &core->cnt_class_cands);
#endif

        /* motion search ********************/
#if !IBC_ABVR
        u8 skip_me = 0;
#endif

        //WYQ: Add the code to check if this block is already encoded, if yes set skip_me to be true

        if (skip_me)
        {
#if IBC_ABVR
            mecost = mecost_fast;
            mv[MV_X] = mv_fast[MV_X];
            mv[MV_Y] = mv_fast[MV_Y];
#endif
#if IBC_BVP
            bvp_idx = bvp_idx_fast;
#endif
        }
        else
        {
#if IBC_BVP
            mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv, &bvp_idx);
            bvp_idx_fast = bvp_idx;
#else
            mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv);
#endif
#if IBC_ABVR
            mecost_fast = mecost;
            mv_fast[MV_X] = mv[MV_X];
            mv_fast[MV_Y] = mv[MV_Y];
#endif
        }

        if (mv[MV_X] != 0 || mv[MV_Y] != 0)
        {
            found_available_ibc = 1;
#if IBC_ABVR
            skip_me = 1;
#endif
            pi->mv[MV_X] = mv[MV_X];
            pi->mv[MV_Y] = mv[MV_Y];
            if (mecost < best_mecost)
            {
                best_mecost = mecost;
            }

            mv[MV_X] = mv[MV_X] << 2;
            mv[MV_Y] = mv[MV_Y] << 2;
            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);

            SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
#if IBC_BVP
            pi->curr_bvr = bvr_idx == 0 ? 0 : 2;
#else
            if (ctx->info.sqh.abvr_enable_flag && (mv[MV_X] % 16 == 0) && (mv[MV_Y] % 16 == 0))
            {
                pi->curr_bvr = 2;
            }
#endif

#if IBC_ABVR
            mv[MV_X] = (mv[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
            mv[MV_Y] = (mv[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
#if IBC_BVP
            if (core->cnt_class_cands == 0)
            {
                bvp_idx = 0;
                mvp[MV_X] = 0;
                mvp[MV_Y] = 0;
                mvd[MV_X] = mv[MV_X];
                mvd[MV_Y] = mv[MV_Y];
            }
            else
            {
                mvp[MV_X] = core->bvp_cands[bvp_idx][MV_X] << 2;
                mvp[MV_Y] = core->bvp_cands[bvp_idx][MV_Y] << 2;
#if IBC_ABVR
                mvp[MV_X] = (mvp[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
                mvp[MV_Y] = (mvp[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
                mvd[MV_X] = mv[MV_X] - mvp[MV_X];
                mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];
            }

            cur_info->cbvp_idx = bvp_idx;
#else
            mvd[MV_X] = mv[MV_X];
            mvd[MV_Y] = mv[MV_Y];
#endif
#if IBC_ABVR
            pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> (pi->curr_bvr + 2), mvd[MV_Y] >> (pi->curr_bvr + 2));
#else
            pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> 2, mvd[MV_Y] >> 2);
#endif

#if IBC_BVP
            pi->mvp_idx = bvp_idx;
#else
            pi->mvp_idx = mvp_idx;
#endif

#if IST && !ISTS
            core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
            core->mod_info_curr.inter_filter_flag = 0;
#endif
#if ISTS
            int ists_all = (ctx->info.sqh.ists_enable_flag && cur_info->ph_ists_enable_flag && core->mod_info_curr.tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
            for (istsflag = 0; istsflag < ists_all; istsflag++)
            {
                cur_info->ist_tu_flag = istsflag;
                cost = pibc_residue_rdo(ctx, core, 0);
                check_best_ibc_mode(core, pi, cost, &cost_best);
            }
#else
            //cost = cost_ibc = pibc_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred, pi->coef, mvp_idx, pi->mv[1]);
            cost = pibc_residue_rdo(ctx, core, 0);

            check_best_ibc_mode(core, pi, cost, &cost_best);
#endif
        }
#if IBC_ABVR
    }
#endif
    if (found_available_ibc)
    {
        /* reconstruct */
        int start_comp = (ctx->tree_status == TREE_L || ctx->tree_status == TREE_LC) ? Y_C : U_C;
        int num_comp = ctx->tree_status == TREE_LC ? 3 : (ctx->tree_status == TREE_L ? 1 : 2);
        for (j = start_comp; j < start_comp + num_comp; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            com_mcpy(coef_blk[j], bst_info->coef[j], sizeof(s16) * size_tmp);
        }
        int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 };
#if IST
        core->mod_info_best.slice_type = ctx->slice_type;
#endif
#if ISTS
        core->mod_info_best.ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if AWP_SCC
        core->mod_info_best.ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, 0);
        for (i = start_comp; i < start_comp + num_comp; i++)
        {
            int stride = (i == 0 ? cu_width : cu_width >> 1);
            com_recon(i == Y_C ? core->mod_info_best.tb_part : SIZE_2Nx2N, resi[i], bst_info->pred[i], core->mod_info_best.num_nz, i, stride, (i == 0 ? cu_height : cu_height >> 1), stride, bst_info->rec[i], bit_depth
#if SBT
                , bst_info->sbt_info
#endif
            );
        }

        return cost_best;
    }
    else
    {
        return MAX_COST;
    }
}

int pibc_init_frame(ENC_CTX *ctx)
{
    ENC_PIBC *pi;
    COM_PIC     *pic;
    pi = &ctx->pibc;
    pic = pi->pic_org = PIC_ORG(ctx);
    pi->Yuv_org[Y_C] = pic->y;
    pi->Yuv_org[U_C] = pic->u;
    pi->Yuv_org[V_C] = pic->v;
    pi->stride_org[Y_C] = pic->stride_luma;
    pi->stride_org[U_C] = pic->stride_chroma;
    pi->stride_org[V_C] = pic->stride_chroma;

    pic = pi->pic_unfiltered_rec = ctx->ibc_unfiltered_rec_pic;
    pi->unfiltered_rec[Y_C] = pic->y;
    pi->unfiltered_rec[U_C] = pic->u;
    pi->unfiltered_rec[V_C] = pic->v;

    pi->s_unfiltered_rec[Y_C] = pic->stride_luma;
    pi->s_unfiltered_rec[U_C] = pic->stride_chroma;
    pi->s_unfiltered_rec[V_C] = pic->stride_chroma;

    pi->slice_type = ctx->slice_type;

    pi->refp = ctx->refp;

    pi->bit_depth = ctx->info.bit_depth_internal;
    //pi->map_mv = ctx->map.map_mv;
    pi->pic_width_in_scu = ctx->info.pic_width_in_scu;
    //size = sizeof(pel) * MAX_CU_DIM;
    //com_mset(pi->pred_buf, 0, size);
    //size = sizeof(pel) * N_C * MAX_CU_DIM;
    //com_mset(pi->unfiltered_rec_buf, 0, size);
    //com_mset(pi->filtered_rec_buf, 0, size);

    init_LOG_LUT(pi);
    return COM_OK;
}

void reset_ibc_search_range(ENC_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
{
    int hashHitRatio = 0;
    ctx->pibc.search_range_x = ctx->param.ibc_search_range_x;
    ctx->pibc.search_range_y = ctx->param.ibc_search_range_y;

    hashHitRatio = get_hash_hit_ratio(ctx, ctx->ibc_hash_handle, cu_x, cu_y, log2_cuw, log2_cuh); // in percent

    if (hashHitRatio < 5) // 5%
    {
        ctx->pibc.search_range_x >>= 1;
        ctx->pibc.search_range_y >>= 1;
    }
}

int pibc_init_lcu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi;

    pi = &ctx->pibc;
    pi->lambda_mv = (u32)floor(65536.0 * ctx->sqrt_lambda[0]);
    pi->qp_y = core->qp_y;
    pi->qp_u = core->qp_u;
    pi->qp_v = core->qp_v;

    return COM_OK;
}

int pibc_set_complexity(ENC_CTX *ctx, int complexity)
{
    ENC_PIBC *pi;

    pi = &ctx->pibc;

    /* default values *************************************************/
    pi->search_range_x = ctx->param.ibc_search_range_x;
    pi->search_range_y = ctx->param.ibc_search_range_y;

    pi->complexity = complexity;
    return COM_OK;
}

double pibc_residue_rdo_chroma(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi = &ctx->pibc;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s16(*mv)[MV_D] = mod_info_curr->mv;
    s8 *refi = mod_info_curr->refi;
    pel(*pred)[MAX_CU_DIM] = mod_info_curr->pred;
    int bit_depth = ctx->info.bit_depth_internal;
    int(*num_nz_coef)[N_C], tnnz, width[N_C], height[N_C], log2_w[N_C], log2_h[N_C];;
    pel(*rec)[MAX_CU_DIM];
    s64    dist[2][N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_best[N_C];
    int    bit_cnt;
    int    i, cbf_y, cbf_u, cbf_v;
    pel   *org[N_C];
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for inter coding
    int use_alt4x4Trans = 0;
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif

    rec = mod_info_curr->rec;
    num_nz_coef = mod_info_curr->num_nz;
    width[Y_C] = 1 << cu_width_log2;
    height[Y_C] = 1 << cu_height_log2;
    width[U_C] = width[V_C] = 1 << (cu_width_log2 - 1);
    height[U_C] = height[V_C] = 1 << (cu_height_log2 - 1);
    log2_w[Y_C] = cu_width_log2;
    log2_h[Y_C] = cu_height_log2;
    log2_w[U_C] = log2_w[V_C] = cu_width_log2 - 1;
    log2_h[U_C] = log2_h[V_C] = cu_height_log2 - 1;
    org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
    org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
    org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif

    /* prepare MV */
    int luma_scup = mod_info_curr->scup + PEL2SCU(mod_info_curr->cu_width - 1) + PEL2SCU(mod_info_curr->cu_height - 1) * ctx->info.pic_width_in_scu;
    if (MCU_GET_IBC(ctx->map.map_scu[luma_scup]) == 0)
    {
        return MAX_COST;
    }
    for (i = 0; i < 1; i++)
    {
        refi[i] = ctx->map.map_refi[luma_scup][i];
        mv[i][MV_X] = ctx->map.map_mv[luma_scup][i][MV_X];
        mv[i][MV_Y] = ctx->map.map_mv[luma_scup][i][MV_Y];
    }
    if (mv[0][MV_X] == 0 && mv[0][MV_Y] == 0)
    {
        return MAX_COST;
    }

    /* chroma MC */
    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
    com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv[0], pi->pic_unfiltered_rec, pred, ctx->tree_status, bit_depth);

    /* get residual */
    enc_diff_pred(x, y, cu_width_log2, cu_height_log2, pi->pic_org, pred, resi_t);
    memset(dist, 0, sizeof(s64) * 2 * N_C);
    for (i = 1; i < N_C; i++)
    {
        dist[0][i] = enc_ssd_16b(log2_w[i], log2_h[i], pred[i], org[i], width[i], pi->stride_org[i], bit_depth);
    }

    /* test all zero case */
    memset(cbf_best, 0, sizeof(int) * N_C);
    memset(num_nz_coef, 0, sizeof(int) * MAX_NUM_TB * N_C);
    assert(mod_info_curr->tb_part == SIZE_2Nx2N);
    cost_best = (double)((dist[0][U_C] + dist[0][V_C]) * ctx->dist_chroma_weight[0]);

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx); // only count coeff bits for chroma tree
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);

    /* transform and quantization */
    tnnz = enc_tq_yuv_nnz(ctx, core, mod_info_curr, coef, resi_t, pi->slice_type, 0, use_secTrans, use_alt4x4Trans, refi, mv, 0);

    if (tnnz)
    {
        com_itdq_yuv(mod_info_curr, coef, resi_t, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans);
        for (i = 1; i < N_C; i++)
        {
            com_recon(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, resi_t[i], pred[i], num_nz_coef, i, width[i], height[i], width[i], rec[i], bit_depth
#if SBT
                , 0
#endif
            );

            if (is_cu_plane_nz(num_nz_coef, i))
            {
                dist[1][i] = enc_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], width[i], pi->stride_org[i], bit_depth);
            }
            else
            {
                dist[1][i] = dist[0][i];
            }
        }

        cbf_y = is_cu_plane_nz(num_nz_coef, Y_C) > 0 ? 1 : 0;
        cbf_u = is_cu_plane_nz(num_nz_coef, U_C) > 0 ? 1 : 0;
        cbf_v = is_cu_plane_nz(num_nz_coef, V_C) > 0 ? 1 : 0;

        cost = (double)((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx); // only count coeff bits for chroma tree
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        if (cost < cost_best)
        {
            cost_best = cost;
            cbf_best[Y_C] = cbf_y;
            cbf_best[U_C] = cbf_u;
            cbf_best[V_C] = cbf_v;
            SBAC_STORE(core->s_temp_best, core->s_temp_run);
        }

    }

    /* save */
    for (i = 0; i < N_C; i++)
    {
        int size_tmp = (cu_width * cu_height) >> (i == 0 ? 0 : 2);
        if (cbf_best[i] == 0)
        {
            cu_plane_nz_cln(core->mod_info_best.num_nz, i);
            com_mset(core->mod_info_best.coef[i], 0, sizeof(s16) * size_tmp);
            com_mcpy(core->mod_info_best.rec[i], pred[i], sizeof(s16) * size_tmp);
        }
        else
        {
            cu_plane_nz_cpy(core->mod_info_best.num_nz, num_nz_coef, i);
            com_mcpy(core->mod_info_best.coef[i], coef[i], sizeof(s16) * size_tmp);
            com_mcpy(core->mod_info_best.rec[i], rec[i], sizeof(s16) * size_tmp);
        }
    }

    return cost_best;
}
#endif