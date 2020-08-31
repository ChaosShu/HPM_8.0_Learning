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
#include "com_ipred.h"
#include "com_tbl.h"
#include "com_df.h"
#include <math.h>
#include "com_util.h"
#include "com_sao.h"
#if USE_IBC
#include "enc_ibc_hash_wrapper.h"
#endif
#if USE_SP
#include "com_usp.h"
extern void sp_lcu_hash_restore(void* sp_encoder, int x_lcu_start, int y_lcu_start);
extern void sp_cu_hash_copy(void* sp_encoder, int dst_w, int dst_h, int dst_status, int src_w, int src_h, int src_status);
extern u8 sp_cu_hash_update(void* sp_encoder, int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start);
extern u8 sp_is_hash_updated(u8 *map_usp, int x, int y, int cu_width, int cu_height, int pic_width_in_scu);
extern u8 get_adaptive_sp_flag(void* sp_encoder);
#endif

#define ENTROPY_BITS_TABLE_BiTS          10
#define ENTROPY_BITS_TABLE_BiTS_SHIFP   (PROB_BITS-ENTROPY_BITS_TABLE_BiTS)
#define ENTROPY_BITS_TABLE_SIZE         (1<<ENTROPY_BITS_TABLE_BiTS)

static s32 entropy_bits[ENTROPY_BITS_TABLE_SIZE];

//void enc_sbac_bit_reset(ENC_SBAC * sbac)
//{
//    sbac->left_bits = 23;
//    sbac->pending_byte = 0;
//    sbac->is_pending_byte = 0;
//    sbac->stacked_ff = 0;
//    sbac->bitcounter = 0;
//}

/**从熵编码结果获取比特数
*/
u32 enc_get_bit_number(ENC_SBAC *sbac)/*/获取比特数*/
{
    return sbac->bitcounter + 8 * (sbac->stacked_ff) + 8 * (sbac->is_pending_byte ? 1 : 0) + 23 - sbac->left_bits;
}

void enc_bit_est_affine_mvp(ENC_CTX * ctx, ENC_CORE * core, s32 slice_type, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][VER_NUM][MV_D], int vertex_num)
{
    int refi0, refi1;
    int vertex;
    refi0 = refi[REFP_0];
    refi1 = refi[REFP_1];

    if (IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
    {
        for (vertex = 0; vertex < vertex_num; vertex++)
        {
#if BD_AFFINE_AMVR
            s16 mvd_tmp[MV_D];
            u8 AMVR_Shift = Tab_Affine_AMVR(ctx->pinter.curr_mvr);
            mvd_tmp[MV_X] = mvd[REFP_0][vertex][MV_X] >> AMVR_Shift;
            mvd_tmp[MV_Y] = mvd[REFP_0][vertex][MV_Y] >> AMVR_Shift;
            encode_mvd(&core->bs_temp, mvd_tmp);
#else
            encode_mvd(&core->bs_temp, mvd[REFP_0][vertex]);
#endif
        }
    }
    if (slice_type == SLICE_B && REFI_IS_VALID(refi1))
    {
        for (vertex = 0; vertex < vertex_num; vertex++)
        {
#if BD_AFFINE_AMVR
            s16 mvd_tmp[MV_D];
            u8 AMVR_Shift = Tab_Affine_AMVR(ctx->pinter.curr_mvr);
            mvd_tmp[MV_X] = mvd[REFP_1][vertex][MV_X] >> AMVR_Shift;
            mvd_tmp[MV_Y] = mvd[REFP_1][vertex][MV_Y] >> AMVR_Shift;
            encode_mvd(&core->bs_temp, mvd_tmp);
#else
            encode_mvd(&core->bs_temp, mvd[REFP_1][vertex]);
#endif
        }
    }
}


void enc_bit_est_tb_intra_luma(ENC_CTX *ctx, ENC_CORE *core, s32 slice_type, int tb_idx, int cbf_y)
{
    ENC_SBAC *sbac = &core->s_temp_run;
    int* num_nz_coef;
#if DT_SYNTAX
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int log2_tb_w, log2_tb_h, tb_size;
    get_tb_width_height_log2(mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, core->mod_info_curr.tb_part, &log2_tb_w, &log2_tb_h);
    tb_size = 1 << (log2_tb_w + log2_tb_h);
#endif
    assert(ctx->cons_pred_mode != ONLY_INTER);
    enc_sbac_encode_bin(cbf_y, sbac, sbac->ctx.cbf + 0, &core->bs_temp);
    if (cbf_y)
    {
        s16* coef = core->mod_info_curr.coef[Y_C] + tb_idx * tb_size;
        num_nz_coef = core->mod_info_curr.num_nz[tb_idx];
        assert(num_nz_coef[Y_C] > 0);
        enc_eco_xcoef(ctx, &core->bs_temp, coef, log2_tb_w, log2_tb_h, num_nz_coef[Y_C], Y_C);
#if EST
        if (ctx->info.sqh.est_enable_flag && core->mod_info_curr.cu_mode == MODE_INTRA &&
            core->mod_info_curr.tb_part == SIZE_2Nx2N && core->mod_info_curr.ist_tu_flag == 0)
        {
            int log2_wh = (log2_tb_w + log2_tb_h) / 2 - 2;
            enc_eco_est_flag(&core->bs_temp, core->mod_info_curr.est_flag);
        }
#endif
    }
}

//note: this include bits for one pb; the cu bits above part_size is calculated at the first PB in CU, and the intra pred mode is calculated at the first TB in PB
void enc_bit_est_pb_intra_luma(ENC_CTX *ctx, ENC_CORE *core, s32 slice_type, s16 coef[N_C][MAX_CU_DIM], int pb_part_idx)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_SBAC *sbac = &core->s_temp_run;
    int cbf_y;
    int* num_nz_coef;
    assert(ctx->cons_pred_mode != ONLY_INTER);

    if (pb_part_idx == 0)
    {
        if (slice_type != SLICE_I)
        {
            if (ctx->cons_pred_mode != ONLY_INTRA)
            {
                encode_skip_flag(&core->bs_temp, sbac, 0, ctx);
                encode_direct_flag(&core->bs_temp, 0, ctx);
            }

            if (ctx->cons_pred_mode == NO_MODE_CONS)
            {
                encode_pred_mode(&core->bs_temp, MODE_INTRA, ctx);
            }
        }
#if DT_SYNTAX
        encode_part_size(ctx, &core->bs_temp, core->mod_info_curr.pb_part, mod_info_curr->cu_width, mod_info_curr->cu_height, MODE_INTRA);
#endif
    }

#if DT_SYNTAX
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    assert(core->mod_info_curr.pb_info.sub_scup[0] == mod_info_curr->scup);
#endif
    encode_intra_dir(&core->bs_temp, core->mod_info_curr.ipm[pb_part_idx][0],
#if EIPM
        ctx->info.sqh.eipm_enable_flag,
#endif
        core->mod_info_curr.mpm[pb_part_idx]);

#if DT_SYNTAX
    int log2_tb_w, log2_tb_h, tb_size;
    int num_tb_in_pb = get_part_num_tb_in_pb(core->mod_info_curr.pb_part, pb_part_idx);

    get_tb_width_height_log2(mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, core->mod_info_curr.tb_part, &log2_tb_w, &log2_tb_h);
    tb_size = 1 << (log2_tb_w + log2_tb_h);
    for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
    {
#endif
        num_nz_coef = core->mod_info_curr.num_nz[tb_idx];
        assert(num_nz_coef[Y_C] >= 0);
        cbf_y = (num_nz_coef[Y_C] > 0) ? 1 : 0;
        enc_sbac_encode_bin(cbf_y, sbac, sbac->ctx.cbf + 0, &core->bs_temp);
        if (cbf_y)
        {
            s16* coef_tb = coef[Y_C] + tb_size * tb_idx;
            enc_eco_xcoef(ctx, &core->bs_temp, coef_tb, log2_tb_w, log2_tb_h, num_nz_coef[Y_C], Y_C);
#if EST
            if (ctx->info.sqh.est_enable_flag && core->mod_info_curr.cu_mode == MODE_INTRA &&
                core->mod_info_curr.tb_part == SIZE_2Nx2N && core->mod_info_curr.ist_tu_flag == 0)
            {
                int log2_wh = (log2_tb_w + log2_tb_h) / 2 - 2;
                enc_eco_est_flag(&core->bs_temp, core->mod_info_curr.est_flag);
            }
#endif
        }
#if DT_SYNTAX
    }
#endif

#if IPF_BUGFIX
#if DT_INTRA_BOUNDARY_FILTER_OFF
    if (ctx->info.sqh.ipf_enable_flag && (log2_tb_w < MAX_CU_LOG2) && (log2_tb_h < MAX_CU_LOG2) && core->mod_info_curr.pb_part == SIZE_2Nx2N && ctx->tree_status != TREE_C)
#else
    if (ctx->info.sqh.ipf_enable_flag && (log2_tb_w < MAX_CU_LOG2) && (log2_tb_h < MAX_CU_LOG2) && ctx->tree_status != TREE_C)
#endif
        encode_ipf_flag(&core->bs_temp, core->mod_info_curr.ipf_flag);
#endif
}


void enc_bit_est_intra_chroma(ENC_CTX *ctx, ENC_CORE *core, s16 coef[N_C][MAX_CU_DIM])
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_SBAC *sbac = &core->s_temp_run;
    int cbf_u, cbf_v;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int *num_nz_coef = core->mod_info_curr.num_nz[TB0];
#if TSCPM
    encode_intra_dir_c(&core->bs_temp, core->mod_info_curr.ipm[PB0][1], core->mod_info_curr.ipm[PB0][0], ctx->info.sqh.tscpm_enable_flag
#if ENHANCE_TSPCM
        , ctx->info.sqh.enhance_tscpm_enable_flag
#endif
#if PMC
        , ctx->info.sqh.pmc_enable_flag
#endif
    );
#else
    encode_intra_dir_c(&core->bs_temp, core->mod_info_curr.ipm[PB0][1], core->mod_info_curr.ipm[PB0][0]
#if PMC
        , ctx->info.sqh.pmc_enable_flag
#endif
    );
#endif
    cbf_u = (num_nz_coef[U_C] > 0) ? 1 : 0;
    cbf_v = (num_nz_coef[V_C] > 0) ? 1 : 0;
#if PMC
    s8 ipm_c = core->mod_info_curr.ipm[PB0][1];
    int bMcpm = com_is_mcpm(ipm_c);
    if (bMcpm)
    {
        assert(IS_RIGHT_CBF_U(cbf_u));
    }
    else
    {
#endif
        enc_sbac_encode_bin(cbf_u, sbac, sbac->ctx.cbf + 1, &core->bs_temp);
#if PMC
    }
#endif
    enc_sbac_encode_bin(cbf_v, sbac, sbac->ctx.cbf + 2, &core->bs_temp);
    cu_width_log2--;
    cu_height_log2--;
    if (cbf_u)
    {
        enc_eco_xcoef(ctx, &core->bs_temp, coef[U_C], cu_width_log2, cu_height_log2, num_nz_coef[U_C], U_C);
    }
    if (cbf_v)
    {
        enc_eco_xcoef(ctx, &core->bs_temp, coef[V_C], cu_width_log2, cu_height_log2, num_nz_coef[V_C], V_C);
    }
}


void enc_bit_est_intra(ENC_CTX * ctx, ENC_CORE * core, s32 slice_type, s16 coef[N_C][MAX_CU_DIM])
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_SBAC *sbac = &core->s_temp_run;
    int cbf_y, cbf_u, cbf_v;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int* num_nz_coef;
#if SBT
    assert( mod_info_curr->sbt_info == 0 );
#endif

    if (slice_type != SLICE_I && ctx->tree_status != TREE_C)
    {
        if (ctx->cons_pred_mode != ONLY_INTRA)
        {
            encode_skip_flag(&core->bs_temp, sbac, 0, ctx);
            encode_direct_flag(&core->bs_temp, 0, ctx);
        }
        if (ctx->cons_pred_mode == NO_MODE_CONS)
        {
            encode_pred_mode(&core->bs_temp, MODE_INTRA, ctx);
        }
    }

    if (ctx->tree_status != TREE_C)
    {
#if DT_SYNTAX
        encode_part_size(ctx, &core->bs_temp, core->mod_info_curr.pb_part, mod_info_curr->cu_width, mod_info_curr->cu_height, MODE_INTRA);
        get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
        assert(core->mod_info_curr.pb_info.sub_scup[0] == mod_info_curr->scup);

        //pred mode y, pb-based
        for (int pb_idx = 0; pb_idx < core->mod_info_curr.pb_info.num_sub_part; pb_idx++)
        {
#endif
            encode_intra_dir(&core->bs_temp, core->mod_info_curr.ipm[pb_idx][0],
#if EIPM
                ctx->info.sqh.eipm_enable_flag,
#endif
                core->mod_info_curr.mpm[pb_idx]);
#if DT_SYNTAX
        }
#endif
    }

    //pred_mode uv
    if (ctx->tree_status != TREE_L)
    {
#if TSCPM
        encode_intra_dir_c(&core->bs_temp, core->mod_info_curr.ipm[PB0][1], core->mod_info_curr.ipm[PB0][0], ctx->info.sqh.tscpm_enable_flag
#if ENHANCE_TSPCM
            , ctx->info.sqh.enhance_tscpm_enable_flag
#endif
#if PMC
            , ctx->info.sqh.pmc_enable_flag
#endif
        );
#else
        encode_intra_dir_c(&core->bs_temp, core->mod_info_curr.ipm[PB0][1], core->mod_info_curr.ipm[PB0][0]
#if PMC
            , ctx->info.sqh.pmc_enable_flag
#endif
        );
#endif
    }

#if IPCM
    if (!(core->mod_info_curr.ipm[PB0][0] == IPD_IPCM && core->mod_info_curr.ipm[PB0][1] == IPD_DM_C))
    {
#endif
#if DT_INTRA_BOUNDARY_FILTER_OFF
        if (ctx->info.sqh.ipf_enable_flag && (cu_width_log2 < MAX_CU_LOG2) && (cu_height_log2 < MAX_CU_LOG2) && core->mod_info_curr.pb_part == SIZE_2Nx2N && ctx->tree_status != TREE_C)
#else
        if (ctx->info.sqh.ipf_enable_flag && (cu_width_log2 < MAX_CU_LOG2) && (cu_height_log2 < MAX_CU_LOG2) && ctx->tree_status != TREE_C)
#endif
            encode_ipf_flag(&core->bs_temp, core->mod_info_curr.ipf_flag);

#if DT_INTRA_BOUNDARY_FILTER_OFF
        if (core->mod_info_curr.pb_part != SIZE_2Nx2N)
            assert(core->mod_info_curr.ipf_flag == 0);
#endif

        if (ctx->tree_status != TREE_C)
        {
            //cbf y
            for (int tb_idx = 0; tb_idx < core->mod_info_curr.tb_info.num_sub_part; tb_idx++)
            {
                num_nz_coef = core->mod_info_curr.num_nz[tb_idx];
                cbf_y = (num_nz_coef[Y_C] > 0) ? 1 : 0;
                enc_sbac_encode_bin(cbf_y, sbac, sbac->ctx.cbf + 0, &core->bs_temp);
            }
        }
        else
        {
            assert(core->mod_info_curr.num_nz[TB0][Y_C] == 0);
            assert(core->mod_info_curr.pb_part == SIZE_2Nx2N);
        }

        //cbf uv
        num_nz_coef = core->mod_info_curr.num_nz[TB0];
        cbf_u = (num_nz_coef[U_C] > 0) ? 1 : 0;
        cbf_v = (num_nz_coef[V_C] > 0) ? 1 : 0;
        if (ctx->tree_status != TREE_L)
        {
#if PMC
            s8 ipm_c = core->mod_info_curr.ipm[PB0][1];
            int bMcpm = com_is_mcpm(ipm_c);
            if (bMcpm)
            {
                assert(IS_RIGHT_CBF_U(cbf_u));
            }
            else
            {
#endif
                enc_sbac_encode_bin(cbf_u, sbac, sbac->ctx.cbf + 1, &core->bs_temp);
#if PMC
            }
#endif
            enc_sbac_encode_bin(cbf_v, sbac, sbac->ctx.cbf + 2, &core->bs_temp);
        }
        else
        {
            assert(cbf_u == 0);
            assert(cbf_v == 0);
        }
#if IPCM
    }
#endif

    //coef y
#if IPCM
    if (core->mod_info_curr.ipm[PB0][0] == IPD_IPCM && ctx->tree_status != TREE_C)
    {
        int tb_w = cu_width_log2 > 5 ? 32 : (1 << cu_width_log2);
        int tb_h = cu_height_log2 > 5 ? 32 : (1 << cu_height_log2);
        int num_tb_w = cu_width_log2 > 5 ? 1 << (cu_width_log2 - 5) : 1;
        int num_tb_h = cu_height_log2 > 5 ? 1 << (cu_height_log2 - 5) : 1;
        for (int h = 0; h < num_tb_h; h++)
        {
            for (int w = 0; w < num_tb_w; w++)
            {
                s16* coef_tb = coef[Y_C] + (1 << cu_width_log2) * h * tb_h + w * tb_w;
                encode_ipcm(sbac, &core->bs_temp, coef_tb, tb_w, tb_h, 1 << cu_width_log2, ctx->info.bit_depth_input, Y_C);
            }
        }
    }
    else
    {
#endif
        assert(core->mod_info_curr.tb_info.num_sub_part == get_part_num(core->mod_info_curr.tb_part));
        int log2_tb_w, log2_tb_h, tb_size;
        get_tb_width_height_log2(mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, core->mod_info_curr.tb_part, &log2_tb_w, &log2_tb_h);
        tb_size = 1 << (log2_tb_w + log2_tb_h);
        for (int tb_idx = 0; tb_idx < core->mod_info_curr.tb_info.num_sub_part; tb_idx++)
        {
            num_nz_coef = core->mod_info_curr.num_nz[tb_idx];
            assert(num_nz_coef[Y_C] >= 0);
            cbf_y = (num_nz_coef[Y_C] > 0) ? 1 : 0;
            if (cbf_y)
            {
                s16* coef_tb = coef[Y_C] + tb_size * tb_idx;
                enc_eco_xcoef(ctx, &core->bs_temp, coef_tb, log2_tb_w, log2_tb_h, num_nz_coef[Y_C], Y_C);
#if EST
                if (ctx->info.sqh.est_enable_flag && core->mod_info_curr.cu_mode == MODE_INTRA &&
                    core->mod_info_curr.tb_part == SIZE_2Nx2N && core->mod_info_curr.ist_tu_flag == 0)
                {
                    int log2_wh = (log2_tb_w + log2_tb_h) / 2 - 2;
                    enc_eco_est_flag(&core->bs_temp, core->mod_info_curr.est_flag);
                }
#endif
            }
        }
#if IPCM
    }
#endif

    //coef uv
#if IPCM
    if (core->mod_info_curr.ipm[PB0][0] == IPD_IPCM && core->mod_info_curr.ipm[PB0][1] == IPD_DM_C && ctx->tree_status != TREE_L)
    {
        cu_width_log2--;
        cu_height_log2--;
        int tb_w = cu_width_log2 > 5 ? 32 : (1 << cu_width_log2);
        int tb_h = cu_height_log2 > 5 ? 32 : (1 << cu_height_log2);
        int num_tb_w = cu_width_log2 > 5 ? 1 << (cu_width_log2 - 5) : 1;
        int num_tb_h = cu_height_log2 > 5 ? 1 << (cu_height_log2 - 5) : 1;
        for (int h = 0; h < num_tb_h; h++)
        {
            for (int w = 0; w < num_tb_w; w++)
            {
                s16* coef_tb_u = coef[U_C] + (1 << cu_width_log2) * h * tb_h + w * tb_w;
                encode_ipcm(sbac, &core->bs_temp, coef_tb_u, tb_w, tb_h, 1 << cu_width_log2, ctx->info.bit_depth_input, U_C);
                s16* coef_tb_v = coef[V_C] + (1 << cu_width_log2) * h * tb_h + w * tb_w;
                encode_ipcm(sbac, &core->bs_temp, coef_tb_v, tb_w, tb_h, 1 << cu_width_log2, ctx->info.bit_depth_input, V_C);
            }
        }
    }
    else
    {
#endif
        cu_width_log2--;
        cu_height_log2--;
        num_nz_coef = core->mod_info_curr.num_nz[TB0];
        cbf_u = (num_nz_coef[U_C] > 0) ? 1 : 0;
        cbf_v = (num_nz_coef[V_C] > 0) ? 1 : 0;
        if (cbf_u)
        {
            enc_eco_xcoef(ctx, &core->bs_temp, coef[U_C], cu_width_log2, cu_height_log2, num_nz_coef[U_C], U_C);
        }
        if (cbf_v)
        {
            enc_eco_xcoef(ctx, &core->bs_temp, coef[V_C], cu_width_log2, cu_height_log2, num_nz_coef[V_C], V_C);
        }
#if IPCM
    }
#endif
}

void enc_bit_est_inter_comp(ENC_CTX *ctx, ENC_CORE * core, s16 coef[MAX_CU_DIM], int ch_type)
{
    int (*num_nz_coef)[N_C] = core->mod_info_curr.num_nz;
    ENC_SBAC* sbac = &core->s_temp_run;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int i, part_num;
    int log2_tb_w, log2_tb_h, tb_size;

    if (ch_type != Y_C)
    {
        cu_width_log2--;
        cu_height_log2--;
    }

    if (ch_type == Y_C)
    {
        int tb_avaliable = is_tb_avaliable(ctx->info, mod_info_curr);
        if (tb_avaliable)
        {
            enc_sbac_encode_bin(core->mod_info_curr.tb_part != SIZE_2Nx2N, sbac, sbac->ctx.tb_split, &core->bs_temp);
        }

#if SBT //in RDO
        int sbt_avail = com_sbt_allow( mod_info_curr, ctx->info.sqh.sbt_enable_flag, ctx->tree_status );
        if( sbt_avail && mod_info_curr->tb_part == SIZE_2Nx2N && num_nz_coef[TB0][Y_C] )
        {
            enc_eco_sbt_info( &core->bs_temp, cu_width_log2, cu_height_log2, mod_info_curr->sbt_info, sbt_avail );
        }
#endif
    }

    part_num = get_part_num(ch_type == Y_C ? core->mod_info_curr.tb_part : SIZE_2Nx2N);
    get_tb_width_height_log2(cu_width_log2, cu_height_log2, ch_type == Y_C ? core->mod_info_curr.tb_part : SIZE_2Nx2N, &log2_tb_w, &log2_tb_h);
#if SBT
    get_sbt_tb_size( mod_info_curr->sbt_info, ch_type, log2_tb_w, log2_tb_h, &log2_tb_w, &log2_tb_h );
#endif
    tb_size = 1 << (log2_tb_w + log2_tb_h);

    for (i = 0; i < part_num; i++)
    {
        int cbf = (num_nz_coef[i][ch_type] > 0) ? 1 : 0;
        enc_sbac_encode_bin(cbf, sbac, sbac->ctx.cbf + ch_type, &core->bs_temp);

        if (cbf)
        {
            enc_eco_xcoef(ctx, &core->bs_temp, coef + i * tb_size, log2_tb_w, log2_tb_h, num_nz_coef[i][ch_type], ch_type);
        }
    }
}

#if SBT_FAST
void enc_bit_est_inter(ENC_CTX * ctx, ENC_CORE * core, s32 slice_type, u8 include_coef_bits )
#else
void enc_bit_est_inter(ENC_CTX * ctx, ENC_CORE * core, s32 slice_type)
#endif
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s8 *refi = mod_info_curr->refi;
    s32 cup = mod_info_curr->scup;
    int refi0, refi1;

    int vertex = 0;
    int vertex_num = mod_info_curr->affine_flag + 1;

    int mvr_idx = ctx->pinter.curr_mvr;

    assert(slice_type != SLICE_I);
    assert(ctx->cons_pred_mode != ONLY_INTRA);
    int skip_flag = (mod_info_curr->cu_mode == MODE_SKIP);
    int dir_flag = (mod_info_curr->cu_mode == MODE_DIR);
    if (ctx->cons_pred_mode != ONLY_INTRA)
    {
        encode_skip_flag(&core->bs_temp, &core->s_temp_run, skip_flag, ctx);
        if (!skip_flag)
        {
            encode_direct_flag(&core->bs_temp, dir_flag, ctx);
        }
    }

    if (skip_flag || dir_flag)
    {
#if AWP
        int UmveAwpFlag = mod_info_curr->umve_flag || mod_info_curr->awp_flag
#if ETMVP
            || mod_info_curr->etmvp_flag
#endif
            ;
        if (ctx->info.sqh.umve_enable_flag || (ctx->info.sqh.awp_enable_flag && slice_type == SLICE_B && mod_info_curr->cu_width >= MIN_AWP_SIZE
            && mod_info_curr->cu_height >= MIN_AWP_SIZE && mod_info_curr->cu_width <= MAX_AWP_SIZE && mod_info_curr->cu_height <= MAX_AWP_SIZE)
#if ETMVP
            || (ctx->info.sqh.etmvp_enable_flag && mod_info_curr->cu_height >= MIN_ETMVP_SIZE && mod_info_curr->cu_width >= MIN_ETMVP_SIZE)
#endif
            )
        {
            encode_umve_awp_flag(&core->bs_temp, UmveAwpFlag);
        }

#if ETMVP
        if (UmveAwpFlag && ctx->info.sqh.etmvp_enable_flag)
        {
            encode_etmvp_flag(&core->bs_temp, mod_info_curr->etmvp_flag != 0, ctx); /* skip/direct etmvp_flag */
        }

        if (!mod_info_curr->etmvp_flag)
        {
#endif
            if (UmveAwpFlag && ctx->info.sqh.umve_enable_flag && ctx->info.sqh.awp_enable_flag && slice_type == SLICE_B && mod_info_curr->cu_width >= MIN_AWP_SIZE
                && mod_info_curr->cu_height >= MIN_AWP_SIZE && mod_info_curr->cu_width <= MAX_AWP_SIZE && mod_info_curr->cu_height <= MAX_AWP_SIZE)
            {
                encode_awp_flag(&core->bs_temp, mod_info_curr->awp_flag, ctx);
                assert(mod_info_curr->awp_flag != mod_info_curr->umve_flag);
            }
#if ETMVP
        }
#endif

        if (mod_info_curr->awp_flag)
        {
#if AWP_MVR
            if (ctx->info.sqh.awp_mvr_enable_flag)
            {
                encode_awp_mvr_flag(&core->bs_temp, mod_info_curr->awp_mvr_flag0, ctx);
                if (mod_info_curr->awp_mvr_flag0)
                {
                    encode_awp_mvr_idx(&core->bs_temp, mod_info_curr->awp_mvr_idx0);
                }
                encode_awp_mvr_flag(&core->bs_temp, mod_info_curr->awp_mvr_flag1, ctx);
                if (mod_info_curr->awp_mvr_flag1)
                {
                    encode_awp_mvr_idx(&core->bs_temp, mod_info_curr->awp_mvr_idx1);
                }

                if (!mod_info_curr->awp_mvr_flag0 && !mod_info_curr->awp_mvr_flag1)
                {
                    encode_awp_mode(&core->bs_temp, mod_info_curr->skip_idx, mod_info_curr->awp_idx0, mod_info_curr->awp_idx1, ctx);
                }
                else if (mod_info_curr->awp_mvr_flag0 && mod_info_curr->awp_mvr_flag1)
                {
                    if (mod_info_curr->awp_mvr_idx0 == mod_info_curr->awp_mvr_idx1)
                    {
                        encode_awp_mode(&core->bs_temp, mod_info_curr->skip_idx, mod_info_curr->awp_idx0, mod_info_curr->awp_idx1, ctx);
                    }
                    else
                    {
                        encode_awp_mode1(&core->bs_temp, mod_info_curr->skip_idx, mod_info_curr->awp_idx0, mod_info_curr->awp_idx1, ctx);
                    }
                }
                else
                {
                    encode_awp_mode1(&core->bs_temp, mod_info_curr->skip_idx, mod_info_curr->awp_idx0, mod_info_curr->awp_idx1, ctx);
                }
            }
            else
#endif
            encode_awp_mode(&core->bs_temp, mod_info_curr->skip_idx, mod_info_curr->awp_idx0, mod_info_curr->awp_idx1, ctx);
        }
        else if (mod_info_curr->umve_flag)
        {
#if UMVE_ENH
            if (ctx->info.sqh.umve_enh_enable_flag && ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64) && dir_flag)
            {
                encode_inter_filter_flag(&core->bs_temp, mod_info_curr->inter_filter_flag);
            }
            if (ctx->info.pic_header.umve_set_flag)
            {
                encode_umve_idx_sec_set(&core->bs_temp, mod_info_curr->umve_idx);
            }
            else
#endif
            encode_umve_idx(&core->bs_temp, mod_info_curr->umve_idx);
        }
#if ETMVP
        else if (mod_info_curr->etmvp_flag)
        {
            encode_etmvp_idx(&core->bs_temp, mod_info_curr->skip_idx);
        }
#endif
#else
        if (ctx->info.sqh.umve_enable_flag)
#if ETMVP
            encode_umve_flag(&core->bs_temp, mod_info_curr->umve_flag || mod_info_curr->etmvp_flag);
        if (mod_info_curr->umve_flag || mod_info_curr->etmvp_flag)
#else
            encode_umve_flag(&core->bs_temp, mod_info_curr->umve_flag);
        if (mod_info_curr->umve_flag)
#endif
        {
#if ETMVP
            encode_etmvp_flag(&core->bs_temp, mod_info_curr->etmvp_flag != 0, ctx); /* skip/direct etmvp_flag */
            if (mod_info_curr->etmvp_flag)
            {
                encode_etmvp_idx(&core->bs_temp, mod_info_curr->skip_idx);
            }
            else
            {
#endif
#if UMVE_ENH
                if (ctx->info.sqh.umve_enh_enable_flag && ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                    && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64) && dir_flag)
                {
                    encode_inter_filter_flag(&core->bs_temp, mod_info_curr->inter_filter_flag);
                }
                if (ctx->info.pic_header.umve_set_flag)
                {
                    encode_umve_idx_sec_set(&core->bs_temp, mod_info_curr->umve_idx);
                }
                else
#endif
                encode_umve_idx(&core->bs_temp, mod_info_curr->umve_idx);
#if ETMVP
            }
#endif
        }
#endif
        else
        {
            encode_affine_flag(&core->bs_temp, mod_info_curr->affine_flag != 0, ctx); /* skip/direct affine_flag */
            if (mod_info_curr->affine_flag)
            {
#if AFFINE_UMVE
                if (ctx->info.sqh.affine_umve_enable_flag) 
                {
                    encode_affine_umve_flag(&core->bs_temp, mod_info_curr->affine_umve_flag != 0, ctx); /* skip/direct affine_umve_flag */
                }
#endif
                encode_affine_mrg_idx(&core->bs_temp, mod_info_curr->skip_idx, ctx);
#if AFFINE_UMVE
                if (core->mod_info_curr.affine_umve_flag)
                {
                    encode_affine_umve_idx(&core->bs_temp, mod_info_curr->affine_umve_idx[0]);
                    encode_affine_umve_idx(&core->bs_temp, mod_info_curr->affine_umve_idx[1]);
                }
#endif
            }
            else
            {
#if INTERPF
                if( ctx->info.sqh.interpf_enable_flag && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 64)
                    && (mod_info_curr->cu_width <= 64) && (mod_info_curr->cu_height <= 64) && dir_flag )
                {
                    encode_inter_filter_flag( &core->bs_temp, mod_info_curr->inter_filter_flag );
                }
#endif
                encode_skip_idx(&core->bs_temp, mod_info_curr->skip_idx, ctx->info.sqh.num_of_hmvp_cand, 
#if MVAP
                    ctx->info.sqh.num_of_mvap_cand,
#endif
                    ctx);
            }
        }
    }
    else
    {
        if (ctx->cons_pred_mode == NO_MODE_CONS)
        {
            encode_pred_mode(&core->bs_temp, MODE_INTER, ctx);
        }

        // encode affine flag before amvr index
        encode_affine_flag(&core->bs_temp, mod_info_curr->affine_flag != 0, ctx);

        if (ctx->info.sqh.amvr_enable_flag)
        {
#if EXT_AMVR_HMVP
            if (ctx->info.sqh.emvr_enable_flag && !mod_info_curr->affine_flag) // also imply ctx->info.sqh.num_of_hmvp_cand is not zero
            {
                encode_extend_amvr_flag(&core->bs_temp, mod_info_curr->mvp_from_hmvp_flag);
            }
#endif
            encode_mvr_idx(&core->bs_temp, mvr_idx, mod_info_curr->affine_flag);
        }

        {
            if (slice_type == SLICE_B)
            {
                encode_inter_dir(&core->bs_temp, refi, mod_info_curr->pb_part, ctx);
            }

            refi0 = refi[REFP_0];
            refi1 = refi[REFP_1];
#if SMVD
            if ( ctx->info.sqh.smvd_enable_flag && REFI_IS_VALID( refi0 ) && REFI_IS_VALID( refi1 )
                && (ctx->ptr - ctx->refp[0][REFP_0].ptr == ctx->refp[0][REFP_1].ptr - ctx->ptr) && mod_info_curr->affine_flag == 0
                && !mod_info_curr->mvp_from_hmvp_flag
               )
            {
                encode_smvd_flag( &core->bs_temp, mod_info_curr->smvd_flag);
            }
#endif
            // forward
            if (IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
            {
#if SMVD
                if ( mod_info_curr->smvd_flag == 0 )
#endif
                    encode_refidx(&core->bs_temp, ctx->rpm.num_refp[REFP_0], refi0);

                if (mod_info_curr->affine_flag)
                {
                    for (vertex = 0; vertex < vertex_num; vertex++)
                    {
#if BD_AFFINE_AMVR
                        s16 affine_mvd_real[MV_D];
                        u8 amvr_shift = Tab_Affine_AMVR(mvr_idx);
                        affine_mvd_real[MV_X] = mod_info_curr->affine_mvd[REFP_0][vertex][MV_X] >> amvr_shift;
                        affine_mvd_real[MV_Y] = mod_info_curr->affine_mvd[REFP_0][vertex][MV_Y] >> amvr_shift;
                        encode_mvd(&core->bs_temp, affine_mvd_real);
#else
                        encode_mvd(&core->bs_temp, mod_info_curr->affine_mvd[REFP_0][vertex]);
#endif
                    }
                }
                else
                {
                    s16 mvd[MV_D];
                    mvd[MV_X] = mod_info_curr->mvd[REFP_0][MV_X] >> mvr_idx;
                    mvd[MV_Y] = mod_info_curr->mvd[REFP_0][MV_Y] >> mvr_idx;
                    encode_mvd(&core->bs_temp, mvd);
                }
            }

            // backward
            if (slice_type == SLICE_B && REFI_IS_VALID(refi1))
            {
#if SMVD
                if ( mod_info_curr->smvd_flag == 0 )
#endif
                    encode_refidx(&core->bs_temp, ctx->rpm.num_refp[REFP_1], refi1);

                if (mod_info_curr->affine_flag)
                {
                    for (vertex = 0; vertex < vertex_num; vertex++)
                    {
#if BD_AFFINE_AMVR
                        s16 affine_mvd_real[MV_D];
                        u8 amvr_shift = Tab_Affine_AMVR(mvr_idx);
                        affine_mvd_real[MV_X] = mod_info_curr->affine_mvd[REFP_1][vertex][MV_X] >> amvr_shift;
                        affine_mvd_real[MV_Y] = mod_info_curr->affine_mvd[REFP_1][vertex][MV_Y] >> amvr_shift;
                        encode_mvd(&core->bs_temp, affine_mvd_real);
#else
                        encode_mvd(&core->bs_temp, mod_info_curr->affine_mvd[REFP_1][vertex]);
#endif
                    }
                }
                else
                {
#if SMVD
                    if (mod_info_curr->smvd_flag == 0)
#endif
                    {
                        s16 mvd[MV_D];
                        mvd[MV_X] = mod_info_curr->mvd[REFP_1][MV_X] >> mvr_idx;
                        mvd[MV_Y] = mod_info_curr->mvd[REFP_1][MV_Y] >> mvr_idx;
                        encode_mvd(&core->bs_temp, mvd);
                    }
                }
            }
#if BGC
            if (ctx->info.sqh.bgc_enable_flag && slice_type == SLICE_B && REFI_IS_VALID(refi0) && REFI_IS_VALID(refi1) && (mod_info_curr->cu_width * mod_info_curr->cu_height >= 256))
            {
                ENC_PINTER *pi = &ctx->pinter;
                int poc0 = pi->refp[refi0][REFP_0].pic->ptr;
                int poc1 = pi->refp[refi1][REFP_1].pic->ptr;
                if ((poc0 - ctx->ptr) * (ctx->ptr - poc1) > 0)
                {
                    encode_bgc_flag(&core->bs_temp, mod_info_curr->bgc_flag, mod_info_curr->bgc_idx);
                }
            }
#endif
        }
    }

#if SBT_FAST
    if( !skip_flag && include_coef_bits )
#else
    if (!skip_flag)
#endif
    {
        assert(ctx->tree_status != TREE_C);
        encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx);
    }
}
#if USE_SP
void enc_bit_est_sp(ENC_CTX * ctx, ENC_CORE * core, COM_SP_INFO * sp_info, u8 dir, u8 sp_flag)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    enc_eco_sp_flag(&core->bs_temp, sp_flag);
    int bit_depth = ctx->info.bit_depth_internal;
    int width = 1 << mod_info_curr->cu_width_log2;
    int height = 1 << mod_info_curr->cu_height_log2;
    int total_pixel = width * height;
    int cur_pixel = 0;
    s16 sp_mv[MV_D];
    COM_SP_INFO *p_sp_info = sp_info;
    u8 is_sp_full_len = FALSE;
    /*string copy direction*/
    enc_eco_sp_copy_dir_flag(&core->bs_temp, dir);
    while (cur_pixel < total_pixel)
    {
        enc_eco_sp_is_matched_flag(&core->bs_temp, p_sp_info->is_matched);
        if (p_sp_info->is_matched)
        {
            u8 str_len_mode = p_sp_info->sp_length_type;
            if (str_len_mode != NONE_TYPE)
            {
                enc_eco_sp_special_len_flag(&core->bs_temp, 1);
            }
            else
            {
                enc_eco_sp_special_len_flag(&core->bs_temp, 0);
                int next_remianing_pixel_in_cu = total_pixel - cur_pixel - p_sp_info->length;
                assert(next_remianing_pixel_in_cu > 0);
                enc_eco_sp_string_length(&core->bs_temp, next_remianing_pixel_in_cu - 1, total_pixel - cur_pixel - 1);
            }
            sp_mv[MV_X] = p_sp_info->offset_x;
            sp_mv[MV_Y] = p_sp_info->offset_y;
            enc_eco_sp_mvd(&core->bs_temp,core, p_sp_info, dir, width, height, cur_pixel, sp_mv);
            cur_pixel += p_sp_info->length;
        }
        else 
        {
            int* p_trav_scan_order = com_tbl_raster2trav[dir][mod_info_curr->cu_width_log2 - MIN_CU_LOG2][mod_info_curr->cu_height_log2 - MIN_CU_LOG2];  // processed_count to TravOrder
            int trav_order_index = p_trav_scan_order[cur_pixel];
            int trav_x = GET_TRAV_X(trav_order_index, 1 << mod_info_curr->cu_width_log2);
            int trav_y = GET_TRAV_Y(trav_order_index, mod_info_curr->cu_width_log2);
            enc_eco_pixel_y(p_sp_info->pixel, bit_depth, &core->bs_temp);
            if (ctx->tree_status != TREE_L)
            {
                if (!((&ctx->param)->chroma_format <= 1 && (trav_x & 0x1 || trav_y & 0x1))) 
                {
                    enc_eco_pixel_uv(p_sp_info->pixel, bit_depth, &core->bs_temp);
                }
            }
            cur_pixel++;
        }
        p_sp_info++;
    }
}
#endif
#if USE_IBC
void enc_bit_est_ibc(ENC_CTX * ctx, ENC_CORE * core, s32 slice_type, u8 ibc_flag)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    int skip_flag = 0;
    int dir_flag = 0;
#if IBC_ABVR
    int bvr_idx = ctx->pibc.curr_bvr;
#endif

    if (skip_flag != 1 && dir_flag != 1 && (ctx->cons_pred_mode == NO_MODE_CONS || ctx->cons_pred_mode == ONLY_INTRA))
    {
        enc_eco_ibc(&core->bs_temp, ibc_flag, ctx);
    }

#if IBC_ABVR
    if (ctx->info.sqh.abvr_enable_flag)
    {
        encode_bvr_idx(&core->bs_temp, bvr_idx);
    }
#endif

    if (!skip_flag)
    {

#if IBC_BVP
        if (ctx->info.sqh.num_of_hbvp_cand > 0)
        {
            encode_ibc_bvp_flag(&core->bs_temp, &core->s_temp_run, mod_info_curr->cbvp_idx, ctx);
        }
#endif
#if IBC_ABVR
        mod_info_curr->mvd[0][MV_X] >>= (bvr_idx + 2);
        mod_info_curr->mvd[0][MV_Y] >>= (bvr_idx + 2);

        encode_mvd(&core->bs_temp, mod_info_curr->mvd[0]);

        mod_info_curr->mvd[0][MV_X] <<= (bvr_idx + 2);
        mod_info_curr->mvd[0][MV_Y] <<= (bvr_idx + 2);
#else
        mod_info_curr->mvd[0][MV_X] >>= 2;
        mod_info_curr->mvd[0][MV_Y] >>= 2;

        encode_mvd(&core->bs_temp, mod_info_curr->mvd[0]);

        mod_info_curr->mvd[0][MV_X] <<= 2;
        mod_info_curr->mvd[0][MV_Y] <<= 2;
#endif
        assert(ctx->tree_status != TREE_C);
        encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx);
    }
}
#endif

void enc_init_bits_est()
{
    int i = 0;
    double p;
    for (i = 0; i < ENTROPY_BITS_TABLE_SIZE; i++)
    {
        p = (MAX_PROB*(i + 0.5)) / ENTROPY_BITS_TABLE_SIZE;
        entropy_bits[i] = (s32)(-32000 * (log(p) / log(2.0) - PROB_BITS));
    }
}

static s32 biari_no_bits(int symbol, SBAC_CTX_MODEL* cm)
{
    s32 est_bits;
    u8 cmps;
    u16 prob_lps;
#if CABAC_MULTI_PROB
    u16 p0, p1;
    p0 = ((*cm) >> PROB_BITS)& MCABAC_PROB_MASK;
    p1 = ((*cm) >> 1) & MCABAC_PROB_MASK;
    prob_lps = (u16)(p0 + p1 + 1) >> 1;
    prob_lps = prob_lps < 6 ? 6 : prob_lps;
    cmps = (*cm) & 1;
    symbol = (u8)(symbol != 0);
#else
    cmps = (*cm) & 1;
    symbol = (u8)(symbol != 0);
    prob_lps = ((*cm) & PROB_MASK) >> 1;
#endif
    prob_lps = (symbol != cmps) ? prob_lps : (MAX_PROB - prob_lps);
    /* (s32)(-32000*(log(p)/log(2.0)-MCABAC_PROB_BITS)); */
    est_bits = entropy_bits[prob_lps >> ENTROPY_BITS_TABLE_BiTS_SHIFP];
    return est_bits;
}

static s32 biari_no_bitsW(int symbol, SBAC_CTX_MODEL* cm1, SBAC_CTX_MODEL* cm2)
{
    s32 est_bits;
    u16 prob_lps;
#if CABAC_MULTI_PROB
    u16 p1_0 = ((*cm1) >> PROB_BITS)& MCABAC_PROB_MASK;
    u16 p1_1 = ((*cm1) >> 1) & MCABAC_PROB_MASK;
    u16 prob_lps1 = (u16)(p1_0 + p1_1 + 1) >> 1;
    prob_lps1 = prob_lps1 < 6 ? 6 : prob_lps1;
    u16 p2_0 = ((*cm2) >> PROB_BITS)& MCABAC_PROB_MASK;
    u16 p2_1 = ((*cm2) >> 1) & MCABAC_PROB_MASK;
    u16 prob_lps2 = (u16)(p2_0 + p2_1 + 1) >> 1;
    prob_lps2 = prob_lps2 < 6 ? 6 : prob_lps2;
    u16 cmps;
    u16 cmps1 = (*cm1) & 1;
    u16 cmps2 = (*cm2) & 1;
#else
    u16 prob_lps1 = ((*cm1) & PROB_MASK) >> 1;
    u16 prob_lps2 = ((*cm2) & PROB_MASK) >> 1;
    u16 cmps;
    u16 cmps1 = (*cm1) & 1;
    u16 cmps2 = (*cm2) & 1;
#endif
    if (cmps1 == cmps2)
    {
        cmps = cmps1;
        prob_lps = (prob_lps1 + prob_lps2) >> 1;
    }
    else
    {
        if (prob_lps1 < prob_lps2)
        {
            cmps = cmps1;
            prob_lps = (256 << LG_PMPS_SHIFTNO) - 1 - ((prob_lps2 - prob_lps1) >> 1);
        }
        else
        {
            cmps = cmps2;
            prob_lps = (256 << LG_PMPS_SHIFTNO) - 1 - ((prob_lps1 - prob_lps2) >> 1);
        }
    }

    symbol = (u8)(symbol != 0);

    prob_lps = (symbol != cmps) ? prob_lps : (MAX_PROB - prob_lps);
    /* (s32)(-32000*(log(p)/log(2.0)-MCABAC_PROB_BITS)); */
    est_bits = entropy_bits[prob_lps >> ENTROPY_BITS_TABLE_BiTS_SHIFP];
    return est_bits;
}

static void enc_rdoq_bit_est(ENC_SBAC * sbac)
{
    int bin, ctx;
    for (bin = 0; bin < 2; bin++)
    {
        for (ctx = 0; ctx < NUM_QT_CBF_CTX; ctx++)
        {
            rdoq_est_cbf[ctx][bin] = biari_no_bits(bin, sbac->ctx.cbf + ctx);
        }
        rdoq_est_ctp_zero_flag[bin] = biari_no_bits(bin, sbac->ctx.ctp_zero_flag);
    }
#if SRCC
    for (ctx = 0; ctx < NUM_CTX_GT0; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            rdoq_est_gt0[ctx][bin] = biari_no_bits(bin, sbac->ctx.cc_gt0 + ctx);
        }
    }

    for (ctx = 0; ctx < NUM_CTX_GT1; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            rdoq_est_gt1[ctx][bin] = biari_no_bits(bin, sbac->ctx.cc_gt1 + ctx);
        }
    }

    for (ctx = 0; ctx < NUM_CTX_SCANR; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            rdoq_est_scanr_x[ctx][bin] = biari_no_bits(bin, sbac->ctx.cc_scanr_x + ctx);
            rdoq_est_scanr_y[ctx][bin] = biari_no_bits(bin, sbac->ctx.cc_scanr_y + ctx);
        }
    }
#endif
    for (ctx = 0; ctx < NUM_SBAC_CTX_RUN_RDOQ; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            rdoq_est_run[ctx][bin] = biari_no_bits(bin, sbac->ctx.run_rdoq + ctx);
        }
    }
    for (ctx = 0; ctx < NUM_SBAC_CTX_LEVEL; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            rdoq_est_level[ctx][bin] = biari_no_bits(bin, sbac->ctx.level + ctx);
        }
    }

    for (ctx = 0; ctx < 2; ctx++) // luma / chroma
    {
        int i, j;
        int chroma_offset1 = ctx * NUM_SBAC_CTX_LAST1;
        int chroma_offset2 = ctx * NUM_SBAC_CTX_LAST2;

        for (i = 0; i < NUM_SBAC_CTX_LAST1; i++)
        {
            for (j = 0; j < NUM_SBAC_CTX_LAST2; j++)
            {
                for (bin = 0; bin < 2; bin++)
                {
                    rdoq_est_last[ctx][i][j][bin] = biari_no_bitsW(bin, sbac->ctx.last1 + i + chroma_offset1, sbac->ctx.last2 + j + chroma_offset2);
                }
            }
        }
    }
}

static int init_cu_data(ENC_CU_DATA *cu_data, int cu_width_log2, int cu_height_log2)
{
    int i, j;
    int cuw_scu, cuh_scu;
    cuw_scu = 1 << (cu_width_log2 - MIN_CU_LOG2);
    cuh_scu = 1 << (cu_height_log2 - MIN_CU_LOG2);
    for (i = 0; i < MAX_CU_DEPTH; i++)
    {
        for (j = 0; j < NUM_BLOCK_SHAPE; j++)
        {
            com_mset(cu_data->split_mode[i][j], 0, cuw_scu * cuh_scu * sizeof(s8));
        }
    }
    com_mset(cu_data->mpm[0], 0, cuw_scu * cuh_scu * sizeof(u8));
    com_mset(cu_data->mpm[1], 0, cuw_scu * cuh_scu * sizeof(u8));
    com_mset(cu_data->ipm[0], 0, cuw_scu * cuh_scu * sizeof(s8));
    com_mset(cu_data->ipm[1], 0, cuw_scu * cuh_scu * sizeof(s8));
    com_mset(cu_data->ipf_flag, 0, cuw_scu * cuh_scu * sizeof(u8));
#if SBT //not necessary
    com_mset( cu_data->map_pb_tb_part, 0, cuw_scu * cuh_scu * sizeof( u32 ) );
#endif

#if IST
    com_mset(cu_data->ist_tu_flag, 0, cuw_scu * cuh_scu * sizeof(u8));
#endif
#if EST
    com_mset(cu_data->est_tu_flag, 0, cuw_scu * cuh_scu * sizeof(u8));
#endif
#if USE_SP
    com_mset(cu_data->sp_flag, 0, cuw_scu * cuh_scu * sizeof(u8));
#endif
    for (i = 0; i < 8; i++)
    {
        com_mset(cu_data->mpm_ext[i], 0, cuw_scu * cuh_scu * sizeof(u8));
    }
    return COM_OK;
}

static int copy_cu_data(ENC_CU_DATA *dst, ENC_CU_DATA *src, int x, int y, int cu_width_log2, int cu_height_log2, int log2_cus, int cud, u8 tree_status)
{
    int i, j, k;
    int cu_width, cu_height, cus;
    int cuw_scu, cuh_scu, cus_scu;
    int cx, cy;
    int size, idx_dst, idx_src;
    cx = x >> MIN_CU_LOG2;
    cy = y >> MIN_CU_LOG2;
    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
    cus = 1 << log2_cus;
    cuw_scu = 1 << (cu_width_log2 - MIN_CU_LOG2);
    cuh_scu = 1 << (cu_height_log2 - MIN_CU_LOG2);
    cus_scu = 1 << (log2_cus - MIN_CU_LOG2);
    assert(tree_status != TREE_C);
    if (tree_status == TREE_C)
    {
        for (j = 0; j < cuh_scu; j++)
        {
            idx_dst = (cy + j) * cus_scu + cx;
            idx_src = j * cuw_scu;
            size = cuw_scu * sizeof(s8);
            com_mcpy(dst->ipm[1] + idx_dst, src->ipm[1] + idx_src, size);

            size = cuw_scu * sizeof(int);
            assert(*(src->num_nz_coef[Y_C] + idx_src) == 0);
            for (k = U_C; k < N_C; k++)
            {
                com_mcpy(dst->num_nz_coef[k] + idx_dst, src->num_nz_coef[k] + idx_src, size);
            }
        }

        for (j = 0; j < cu_height >> 1; j++)
        {
            idx_dst = ((y >> 1) + j) * (cus >> 1) + (x >> 1);
            idx_src = j * (cu_width >> 1);
            size = (cu_width >> 1) * sizeof(s16);
            com_mcpy(dst->coef[U_C] + idx_dst, src->coef[U_C] + idx_src, size);
            com_mcpy(dst->coef[V_C] + idx_dst, src->coef[V_C] + idx_src, size);
            size = (cu_width >> 1) * sizeof(pel);
            com_mcpy(dst->reco[U_C] + idx_dst, src->reco[U_C] + idx_src, size);
            com_mcpy(dst->reco[V_C] + idx_dst, src->reco[V_C] + idx_src, size);
        }
        return COM_OK;
    }

    for (j = 0; j < cuh_scu; j++)
    {
        idx_dst = (cy + j) * cus_scu + cx;
        idx_src = j * cuw_scu;
        size = cuw_scu * sizeof(s8);
        for (k = cud; k < MAX_CU_DEPTH; k++)
        {
            for (i = 0; i < NUM_BLOCK_SHAPE; i++)
            {
                com_mcpy(dst->split_mode[k][i] + idx_dst, src->split_mode[k][i] + idx_src, size);
            }
        }
        com_mcpy(dst->pred_mode + idx_dst, src->pred_mode + idx_src, size);
        com_mcpy(dst->mpm[0] + idx_dst, src->mpm[0] + idx_src, size);
        com_mcpy(dst->mpm[1] + idx_dst, src->mpm[1] + idx_src, size);
        com_mcpy(dst->ipm[0] + idx_dst, src->ipm[0] + idx_src, size);
        com_mcpy(dst->ipm[1] + idx_dst, src->ipm[1] + idx_src, size);
        for (i = 0; i < 8; i++)
        {
            com_mcpy(dst->mpm_ext[i] + idx_dst, src->mpm_ext[i] + idx_src, size);
        }
        com_mcpy(dst->affine_flag + idx_dst, src->affine_flag + idx_src, size);
#if ETMVP
        com_mcpy(dst->etmvp_flag + idx_dst, src->etmvp_flag + idx_src, size);
#endif
#if SMVD
        com_mcpy( dst->smvd_flag + idx_dst, src->smvd_flag + idx_src, size );
#endif
        com_mcpy(dst->depth + idx_dst, src->depth + idx_src, size);
        size = cuw_scu * sizeof(u32);
        com_mcpy(dst->map_scu + idx_dst, src->map_scu + idx_src, size);
        com_mcpy(dst->map_cu_mode + idx_dst, src->map_cu_mode + idx_src, size);
#if TB_SPLIT_EXT
        com_mcpy(dst->map_pb_tb_part + idx_dst, src->map_pb_tb_part + idx_src, size);
#endif
#if USE_SP
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->map_usp + idx_dst, src->map_usp + idx_src, size);
        com_mcpy(dst->sp_flag + idx_dst, src->sp_flag + idx_src, size);
        size = cuw_scu * sizeof(s16);
        com_mcpy(dst->sub_string_no + idx_dst, src->sub_string_no + idx_src, size);
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->sp_copy_direction + idx_dst, src->sp_copy_direction + idx_src, size);
        com_mcpy(dst->is_sp_pix_completed + idx_dst, src->is_sp_pix_completed + idx_src, size);
#endif
        size = cuw_scu * sizeof(u8) * REFP_NUM;
        com_mcpy(*(dst->refi + idx_dst), *(src->refi + idx_src), size);
#if INTERPF
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->inter_filter_flag + idx_dst, src->inter_filter_flag + idx_src, size);
#endif
#if BGC
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->bgc_flag + idx_dst, src->bgc_flag + idx_src, size);
        com_mcpy(dst->bgc_idx + idx_dst, src->bgc_idx + idx_src, size);
#endif
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->umve_flag + idx_dst, src->umve_flag + idx_src, size);
        com_mcpy(dst->umve_idx + idx_dst, src->umve_idx + idx_src, size);
#if AFFINE_UMVE
        com_mcpy(dst->affine_umve_flag + idx_dst, src->affine_umve_flag + idx_src, size);
        for (k = 0; k < VER_NUM; k++)
        {
            com_mcpy(dst->affine_umve_idx[k] + idx_dst, src->affine_umve_idx[k] + idx_src, size);
        }
#endif
        com_mcpy(dst->skip_idx + idx_dst, src->skip_idx + idx_src, size);
#if AWP
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->awp_flag + idx_dst, src->awp_flag + idx_src, size);
        com_mcpy(dst->awp_idx0 + idx_dst, src->awp_idx0 + idx_src, size);
        com_mcpy(dst->awp_idx1 + idx_dst, src->awp_idx1 + idx_src, size);
#endif
#if AWP_MVR
        com_mcpy(dst->awp_mvr_flag0 + idx_dst, src->awp_mvr_flag0 + idx_src, size);
        com_mcpy(dst->awp_mvr_idx0 + idx_dst, src->awp_mvr_idx0 + idx_src, size);
        com_mcpy(dst->awp_mvr_flag1 + idx_dst, src->awp_mvr_flag1 + idx_src, size);
        com_mcpy(dst->awp_mvr_idx1 + idx_dst, src->awp_mvr_idx1 + idx_src, size);
#endif
#if USE_IBC
        com_mcpy(dst->ibc_flag + idx_dst, src->ibc_flag + idx_src, size);
#endif
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->mvr_idx + idx_dst, src->mvr_idx + idx_src, size);
#if IBC_ABVR
        com_mcpy(dst->bvr_idx + idx_dst, src->bvr_idx + idx_src, size);
#endif
#if EXT_AMVR_HMVP
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->mvp_from_hmvp_flag + idx_dst, src->mvp_from_hmvp_flag + idx_src, size);
#endif
#if IBC_BVP
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->cbvp_idx + idx_dst, src->cbvp_idx + idx_src, size);
#endif
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->ipf_flag + idx_dst, src->ipf_flag + idx_src, size);
        size = cuw_scu * sizeof(s16) * REFP_NUM * MV_D;
        com_mcpy(dst->mv + idx_dst, src->mv + idx_src, size);
        com_mcpy(dst->mvd + idx_dst, src->mvd + idx_src, size);
#if IST
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->ist_tu_flag + idx_dst, src->ist_tu_flag + idx_src, size);
#endif
#if EST
        size = cuw_scu * sizeof(u8);
        com_mcpy(dst->est_tu_flag + idx_dst, src->est_tu_flag + idx_src, size);
#endif
        size = cuw_scu * sizeof(int);
        for (k = 0; k < N_C; k++)
        {
            com_mcpy(dst->num_nz_coef[k] + idx_dst, src->num_nz_coef[k] + idx_src, size);
        }
#if TB_SPLIT_EXT
        com_mcpy(dst->pb_part + idx_dst, src->pb_part + idx_src, size);
        com_mcpy(dst->tb_part + idx_dst, src->tb_part + idx_src, size);
#endif
    }
    for (j = 0; j < cu_height; j++)
    {
        idx_dst = (y + j) * cus + x;
        idx_src = j * cu_width;
        size = cu_width * sizeof(s16);
        com_mcpy(dst->coef[Y_C] + idx_dst, src->coef[Y_C] + idx_src, size);
        size = cu_width * sizeof(pel);
        com_mcpy(dst->reco[Y_C] + idx_dst, src->reco[Y_C] + idx_src, size);
    }
    for (j = 0; j < cu_height >> 1; j++)
    {
        idx_dst = ((y >> 1) + j) * (cus >> 1) + (x >> 1);
        idx_src = j * (cu_width >> 1);
        size = (cu_width >> 1) * sizeof(s16);
        com_mcpy(dst->coef[U_C] + idx_dst, src->coef[U_C] + idx_src, size);
        com_mcpy(dst->coef[V_C] + idx_dst, src->coef[V_C] + idx_src, size);
        size = (cu_width >> 1) * sizeof(pel);
        com_mcpy(dst->reco[U_C] + idx_dst, src->reco[U_C] + idx_src, size);
        com_mcpy(dst->reco[V_C] + idx_dst, src->reco[V_C] + idx_src, size);
#if USE_SP
        size = (cu_width >> 1) * sizeof(COM_SP_INFO);
        com_mcpy(dst->sp_strInfo + idx_dst, src->sp_strInfo + idx_src, size);
#endif
    }
    return COM_OK;
}

static int mode_cu_init(ENC_CTX * ctx, ENC_CORE * core, int x, int y, int cu_width_log2, int cu_height_log2, int cud)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    bst_info->x_pos = mod_info_curr->x_pos = x;
    bst_info->y_pos = mod_info_curr->y_pos = y;
    bst_info->cu_width = mod_info_curr->cu_width = 1 << cu_width_log2;
    bst_info->cu_height = mod_info_curr->cu_height = 1 << cu_height_log2;
    bst_info->cu_width_log2 = mod_info_curr->cu_width_log2 = cu_width_log2;
    bst_info->cu_height_log2 = mod_info_curr->cu_height_log2 = cu_height_log2;
    bst_info->x_scu = mod_info_curr->x_scu = PEL2SCU(x);
    bst_info->x_scu = mod_info_curr->y_scu = PEL2SCU(y);
    bst_info->scup = mod_info_curr->scup = ((u32)mod_info_curr->y_scu * ctx->info.pic_width_in_scu) + mod_info_curr->x_scu;
    bst_info->cud = mod_info_curr->cud = cud;
    cu_nz_cln(core->mod_info_curr.num_nz);
    cu_nz_cln(core->mod_info_best.num_nz);
#if PLATFORM_GENERAL_DEBUG
    memset(bst_info->coef[Y_C], 0, sizeof(s16)*mod_info_curr->cu_width*mod_info_curr->cu_height);
    memset(bst_info->coef[U_C], 0, sizeof(s16)*mod_info_curr->cu_width*mod_info_curr->cu_height / 4);
    memset(bst_info->coef[V_C], 0, sizeof(s16)*mod_info_curr->cu_width*mod_info_curr->cu_height / 4);
#endif

#if CHROMA_NOT_SPLIT //maybe not necessary
    if (ctx->tree_status == TREE_C)
    {
        return COM_OK;
    }
#endif

    bst_info->cu_mode = MODE_INTRA;
#if TB_SPLIT_EXT
    //init the best cu info
    init_pb_part(bst_info);
    init_tb_part(bst_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, bst_info->pb_part, &bst_info->pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, bst_info->tb_part, &bst_info->tb_info);
    assert(bst_info->pb_info.sub_scup[0] == mod_info_curr->scup);
#endif
    //com_mset(bst_info, 0, sizeof(ENC_MODE));
    com_mset(bst_info->mv, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(bst_info->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(bst_info->refi, 0, sizeof(s16) * REFP_NUM);
    bst_info->mvr_idx = 0;
#if IBC_ABVR
    bst_info->bvr_idx = 0;
#endif
    bst_info->skip_idx = 0;
#if INTERPF
    bst_info->inter_filter_flag = 0;
#endif
#if BGC
    bst_info->bgc_flag = 0;
    bst_info->bgc_idx = 0;
#endif
    bst_info->umve_flag = 0;
    bst_info->umve_idx = -1;
#if AFFINE_UMVE
    core->mod_info_curr.affine_umve_flag = 0;
    bst_info->affine_umve_flag = 0;
    for (int i = 0; i < VER_NUM; i++)
    {
        bst_info->affine_umve_idx[i] = -1;
    }
#endif
#if EXT_AMVR_HMVP
    bst_info->mvp_from_hmvp_flag = 0;
    core->skip_mvps_check = 1;
#endif
    bst_info->ipf_flag = 0;
#if USE_IBC
    bst_info->ibc_flag = 0;
#endif
#if IBC_BVP
    bst_info->cbvp_idx = 0;
#endif
#if AWP
    bst_info->awp_flag = 0;
    bst_info->awp_idx0 = 0;
    bst_info->awp_idx1 = 0;
#endif
#if AWP_MVR
    core->mod_info_curr.awp_mvr_flag0 = 0;
    core->mod_info_curr.awp_mvr_idx0 = 0;
    core->mod_info_curr.awp_mvr_flag1 = 0;
    core->mod_info_curr.awp_mvr_idx1 = 0;
    bst_info->awp_mvr_flag0 = 0;
    bst_info->awp_mvr_idx0 = 0;
    bst_info->awp_mvr_flag1 = 0;
    bst_info->awp_mvr_idx1 = 0;
#endif

    core->mod_info_curr.affine_flag = 0;
    bst_info->affine_flag = 0;
    com_mset(bst_info->affine_mv, 0, sizeof(CPMV) * REFP_NUM * VER_NUM * MV_D);
    com_mset(bst_info->affine_mvd, 0, sizeof(s16) * REFP_NUM * VER_NUM * MV_D);

#if USE_SP
    bst_info->sp_flag = 0;
    bst_info->sub_string_no = 0;
    bst_info->sp_copy_direction = 0;
    com_mset(bst_info->string_copy_info, 0, sizeof(COM_SP_INFO) * SP_STRING_INFO_NO);
    bst_info->is_sp_pix_completed = FALSE;
#endif

#if SMVD
    core->mod_info_curr.smvd_flag = 0;
    bst_info->smvd_flag = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = bst_info->sbt_info = 0;
#endif
#if IST
    bst_info->ist_tu_flag = 0;
#endif
#if EST
    bst_info->est_flag = 0;
#endif
    enc_rdoq_bit_est(&core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    return COM_OK;
}

#if USE_IBC
static void mode_cpy_rec_to_unfilterd_ref(ENC_CORE *core, COM_MODE *mi, int x, int y, int w, int h, COM_PIC *pic, u8 tree_status)
{
    pel *src = NULL, *dst = NULL;
    int j, s_pic, off, size;
    int log2_w, log2_h;
    int stride;
    log2_w = CONV_LOG2(w);
    log2_h = CONV_LOG2(h);
    s_pic = pic->stride_luma;
    stride = w;
    if (x + w > pic->width_luma)
    {
        w = pic->width_luma - x;
    }
    if (y + h > pic->height_luma)
    {
        h = pic->height_luma - y;
    }

    assert(tree_status != TREE_C);

    /* luma */
    if (tree_status != TREE_C)
    {
        src = mi->rec[Y_C];
        dst = pic->y + x + y * s_pic;
        size = sizeof(pel) * w;
        for (j = 0; j < h; j++)
        {
            com_mcpy(dst, src, size);
            src += stride;
            dst += s_pic;
        }
    }

    /* chroma */
    if (tree_status != TREE_L)
    {
        s_pic = pic->stride_chroma;
        off = (x >> 1) + (y >> 1) * s_pic;
        size = (sizeof(pel) * w) >> 1;
        src = mi->rec[U_C];
        dst = pic->u + off;
        for (j = 0; j < (h >> 1); j++)
        {
            com_mcpy(dst, src, size);
            src += (stride >> 1);
            dst += s_pic;
        }
        src = mi->rec[V_C];
        dst = pic->v + off;
        for (j = 0; j < (h >> 1); j++)
        {
            com_mcpy(dst, src, size);
            src += (stride >> 1);
            dst += s_pic;
        }
    }
}
#endif

static void mode_cpy_rec_to_ref(ENC_CORE *core, int x, int y, int w, int h, COM_PIC *pic, u8 tree_status)
{
    ENC_CU_DATA *cu_data;
    pel           *src, *dst;
    int            j, s_pic, off, size;
    int            log2_w, log2_h;
    int            stride;
    log2_w = CONV_LOG2(w);
    log2_h = CONV_LOG2(h);
    cu_data = &core->cu_data_best[log2_w - 2][log2_h - 2];
    s_pic = pic->stride_luma;
    stride = w;
    if (x + w > pic->width_luma)
    {
        w = pic->width_luma - x;
    }
    if (y + h > pic->height_luma)
    {
        h = pic->height_luma - y;
    }

    assert(tree_status != TREE_C);

    /* luma */
    if (tree_status != TREE_C)
    {
        src = cu_data->reco[Y_C];
        dst = pic->y + x + y * s_pic;
        size = sizeof(pel) * w;
        for (j = 0; j < h; j++)
        {
            com_mcpy(dst, src, size);
            src += stride;
            dst += s_pic;
        }

    }

    /* chroma */
    if (tree_status != TREE_L)
    {
        s_pic = pic->stride_chroma;
        off = (x >> 1) + (y >> 1) * s_pic;
        size = (sizeof(pel) * w) >> 1;
        src = cu_data->reco[U_C];
        dst = pic->u + off;
        for (j = 0; j < (h >> 1); j++)
        {
            com_mcpy(dst, src, size);
            src += (stride >> 1);
            dst += s_pic;
        }
        src = cu_data->reco[V_C];
        dst = pic->v + off;
        for (j = 0; j < (h >> 1); j++)
        {
            com_mcpy(dst, src, size);
            src += (stride >> 1);
            dst += s_pic;
        }
    }
}


void enc_set_affine_mvf(ENC_CTX *ctx, ENC_CORE *core, COM_MODE *mi)
{
    ENC_CU_DATA *cu_data;
    int   cu_width_log2, cu_height_log2;
    int   cu_w_in_scu, cu_h_in_scu;
    int   sub_w_in_scu, sub_h_in_scu;
    int   w, h, x, y;
    int   lidx;
    int   idx;
    int   cp_num = mi->affine_flag + 1;
    int   aff_scup[VER_NUM];
    int   sub_w = 4;
    int   sub_h = 4;

    if (ctx->info.pic_header.affine_subblock_size_idx > 0)
    {
        sub_w = 8;
        sub_h = 8;
    }
    if (REFI_IS_VALID(mi->refi[REFP_0]) && REFI_IS_VALID(mi->refi[REFP_1]))
    {
        sub_w = 8;
        sub_h = 8;
    }

    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;

    sub_w_in_scu = sub_w >> MIN_CU_LOG2;
    sub_h_in_scu = sub_h >> MIN_CU_LOG2;

    cu_width_log2 = CONV_LOG2(mi->cu_width);
    cu_height_log2 = CONV_LOG2(mi->cu_height);
    cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
    cu_w_in_scu = mi->cu_width >> MIN_CU_LOG2;
    cu_h_in_scu = mi->cu_height >> MIN_CU_LOG2;
    aff_scup[0] = 0;
    aff_scup[1] = (cu_w_in_scu - 1);
    aff_scup[2] = (cu_h_in_scu - 1) * cu_w_in_scu;
    aff_scup[3] = (cu_w_in_scu - 1) + (cu_h_in_scu - 1) * cu_w_in_scu;
    for (lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if (mi->refi[lidx] >= 0)
        {
            CPMV (*ac_mv)[MV_D] = mi->affine_mv[lidx];
            s32 dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
            s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
            s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
            s32 mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> mi->cu_width_log2;      // deltaMvHor
            dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> mi->cu_width_log2;
            if (cp_num == 3)
            {
                dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> mi->cu_height_log2; // deltaMvVer
                dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> mi->cu_height_log2;
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                                                 // deltaMvVer
                dmv_ver_y = dmv_hor_x;
            }

            idx = 0;
            for (h = 0; h < cu_h_in_scu; h += sub_h_in_scu)
            {
                for (w = 0; w < cu_w_in_scu; w += sub_w_in_scu)
                {
                    int pos_x = (w << MIN_CU_LOG2) + half_w;
                    int pos_y = (h << MIN_CU_LOG2) + half_h;
                    if (w == 0 && h == 0)
                    {
                        pos_x = 0;
                        pos_y = 0;
                    }
                    else if (w + sub_w_in_scu == cu_w_in_scu && h == 0)
                    {
                        pos_x = cu_w_in_scu << MIN_CU_LOG2;
                        pos_y = 0;
                    }
                    else if (w == 0 && h + sub_h_in_scu == cu_h_in_scu && cp_num == 3)
                    {
                        pos_x = 0;
                        pos_y = cu_h_in_scu << MIN_CU_LOG2;
                    }

                    mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                    mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

                    // 1/16 precision, 18 bits, for MC
#if BD_AFFINE_AMVR
                    com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
#else
                    com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#endif
                    mv_scale_tmp_hor = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_ver);

                    // 1/4 precision, 16 bits, for mv storage
                    com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 2, 0);
                    mv_scale_tmp_hor = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv_scale_tmp_ver);

                    // save MV for each 4x4 block
                    for (y = h; y < h + sub_h_in_scu; y++)
                    {
                        for (x = w; x < w + sub_w_in_scu; x++)
                        {
                            idx = x + y * cu_w_in_scu;
                            cu_data->mv[idx][lidx][MV_X] = (s16)mv_scale_tmp_hor;
                            cu_data->mv[idx][lidx][MV_Y] = (s16)mv_scale_tmp_ver;
                        }
                    }
                }
            }

            // save mvd for encoding, and reset vertex mv
            for (h = 0; h < cp_num; h++)
            {
                cu_data->mvd[aff_scup[h]][lidx][MV_X] = mi->affine_mvd[lidx][h][MV_X];
                cu_data->mvd[aff_scup[h]][lidx][MV_Y] = mi->affine_mvd[lidx][h][MV_Y];
            }
        }
    }
}

#if AWP
void enc_set_awp_mvf(ENC_CTX *ctx, ENC_CORE *core, COM_MODE *mi)
{
    int          cu_w = mi->cu_width;
    int          cu_h = mi->cu_height;
    int          cu_w_in_scu = PEL2SCU(cu_w);
    int          cu_h_in_scu = PEL2SCU(cu_h);
    int          w, h/*, x, y*/;
    int          addr_in_cu = 0;
    ENC_CU_DATA *cu_data = &core->cu_data_temp[CONV_LOG2(cu_w) - 2][CONV_LOG2(cu_h) - 2];
    int stepIdx, angleIdx, subangleIdx;

    com_calculate_awp_para(mi->skip_idx, cu_w, cu_h, &stepIdx, &angleIdx, &subangleIdx);

    int FirstPos = 0;
    int DeltaPos_W = 0;
    int DeltaPos_H = 0;

    //Set half pixel length
    int ValidLength_W = (cu_w + (cu_h >> angleIdx)) << 1;
    int ValidLength_H = (cu_h + (cu_w >> angleIdx)) << 1;
    int temp_W = ((cu_h << 1) >> angleIdx);
    int temp_H = ((cu_w << 1) >> angleIdx);
    DeltaPos_W = (ValidLength_W >> 3) - 1;
    DeltaPos_H = (ValidLength_H >> 3) - 1;
    DeltaPos_W = DeltaPos_W == 0 ? 1 : DeltaPos_W;
    DeltaPos_H = DeltaPos_H == 0 ? 1 : DeltaPos_H;
    DeltaPos_W = stepIdx * DeltaPos_W;
    DeltaPos_H = stepIdx * DeltaPos_H;

    switch (subangleIdx)
    {
    case 0:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (ctx->info.pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) + 1 + DeltaPos_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) - 2 + DeltaPos_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) - 2 + DeltaPos_H;
#endif
        for (h = 0; h < cu_h_in_scu; h++)
        {
            for (w = 0; w < cu_w_in_scu; w++)
            {
                int pos_x = (w << MIN_CU_LOG2) + 2;
                int pos_y = (h << MIN_CU_LOG2) + 2;
                if (((pos_y << 1) + ((pos_x << 1) >> angleIdx)) >= FirstPos)
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi0[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi0[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv0[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv0[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv0[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi1[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi1[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv1[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv1[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv1[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_cu += cu_w_in_scu;
        }
        break;
    case 1:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (ctx->info.pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_H >> 1) + 3 + DeltaPos_H - temp_H;
        }
        else
        {
            FirstPos = (ValidLength_H >> 1) + DeltaPos_H - temp_H;
        }
#else
        FirstPos = (ValidLength_H >> 1) + DeltaPos_H - temp_H;
#endif
        for (h = 0; h < cu_h_in_scu; h++)
        {
            for (w = 0; w < cu_w_in_scu; w++)
            {
                int pos_x = (w << MIN_CU_LOG2) + 2;
                int pos_y = (h << MIN_CU_LOG2) + 2;
                if (((pos_y << 1) - ((pos_x << 1) >> angleIdx)) >= FirstPos)
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi0[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi0[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv0[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv0[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv0[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi1[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi1[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv1[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv1[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv1[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_cu += cu_w_in_scu;
        }
        break;
    case 2:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (ctx->info.pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) + 3 + DeltaPos_W - temp_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) + DeltaPos_W - temp_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) + DeltaPos_W - temp_W;
#endif
        for (h = 0; h < cu_h_in_scu; h++)
        {
            for (w = 0; w < cu_w_in_scu; w++)
            {
                int pos_x = (w << MIN_CU_LOG2) + 2;
                int pos_y = (h << MIN_CU_LOG2) + 2;
                if (((pos_x << 1) - ((pos_y << 1) >> angleIdx)) >= FirstPos)
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi0[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi0[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv0[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv0[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv0[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi1[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi1[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv1[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv1[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv1[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_cu += cu_w_in_scu;
        }
        break;
    case 3:
        //Calculate FirstPos & reference weights [per block]
#if AWP_SCC
        if (ctx->info.pic_header.ph_awp_refine_flag)
        {
            FirstPos = (ValidLength_W >> 1) + 1 + DeltaPos_W;
        }
        else
        {
            FirstPos = (ValidLength_W >> 1) - 2 + DeltaPos_W;
        }
#else
        FirstPos = (ValidLength_W >> 1) - 2 + DeltaPos_W;
#endif
        for (h = 0; h < cu_h_in_scu; h++)
        {
            for (w = 0; w < cu_w_in_scu; w++)
            {
                int pos_x = (w << MIN_CU_LOG2) + 2;
                int pos_y = (h << MIN_CU_LOG2) + 2;
                if (((pos_x << 1) + ((pos_y << 1) >> angleIdx)) >= FirstPos)
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi0[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi0[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv0[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv0[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv0[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv0[REFP_1][MV_Y];
                }
                else
                {
                    cu_data->refi[addr_in_cu + w][REFP_0] = mi->awp_refi1[REFP_0];
                    cu_data->refi[addr_in_cu + w][REFP_1] = mi->awp_refi1[REFP_1];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_X] = mi->awp_mv1[REFP_0][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_0][MV_Y] = mi->awp_mv1[REFP_0][MV_Y];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_X] = mi->awp_mv1[REFP_1][MV_X];
                    cu_data->mv[addr_in_cu + w][REFP_1][MV_Y] = mi->awp_mv1[REFP_1][MV_Y];
                }
            }
            addr_in_cu += cu_w_in_scu;
        }
        break;
    default:
        printf("\nError: awp parameter not expected\n");
        assert(0);
    }
}
#endif

static void copy_to_cu_data(ENC_CTX *ctx, ENC_CORE *core, COM_MODE *mi)
{
    ENC_CU_DATA *cu_data;
    int i, j, k, idx, size;
    int cu_width_log2, cu_height_log2;
    cu_width_log2 = CONV_LOG2(mi->cu_width);
    cu_height_log2 = CONV_LOG2(mi->cu_height);
    cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
    if (ctx->tree_status != TREE_C)
    {
        /* copy coef */
        size = mi->cu_width * mi->cu_height * sizeof(s16);
        com_mcpy(cu_data->coef[Y_C], mi->coef[Y_C], size);
        /* copy reco */
        size = mi->cu_width * mi->cu_height * sizeof(pel);
        com_mcpy(cu_data->reco[Y_C], mi->rec[Y_C], size);
#if USE_SP
        /* copy sp string copy info*/
        if (mi->sp_flag == TRUE) 
        {
            size = (mi->cu_width*mi->cu_height >> 2) * sizeof(COM_SP_INFO);
            com_mcpy(cu_data->sp_strInfo, mi->string_copy_info, size);
        }
#endif
#if PLATFORM_GENERAL_DEBUG
        int num_coef_y = 0;
        int num_nnz_sum = 0;
        int num_luma_tb = get_part_num(mi->tb_part);
        for (int i = 0; i < mi->cu_width * mi->cu_height; i++)
        {
            num_coef_y += mi->coef[Y_C][i] != 0 ? 1 : 0;
        }
        for (int i = 0; i < num_luma_tb; i++)
        {
            num_nnz_sum += mi->num_nz[i][Y_C];
        }
#if IPCM
        if (core->mod_info_best.cu_mode == MODE_INTRA && mi->ipm[PB0][0] == IPD_IPCM)
        {
            num_coef_y = 0;
        }
#endif
        assert(num_coef_y == num_nnz_sum);
#endif
    }
    if (ctx->tree_status != TREE_L)
    {
        /* copy coef */
        size = (mi->cu_width * mi->cu_height * sizeof(s16)) >> 2;
        com_mcpy(cu_data->coef[U_C], mi->coef[U_C], size);
        com_mcpy(cu_data->coef[V_C], mi->coef[V_C], size);
        /* copy reco */
        size = (mi->cu_width * mi->cu_height * sizeof(pel)) >> 2;
        com_mcpy(cu_data->reco[U_C], mi->rec[U_C], size);
        com_mcpy(cu_data->reco[V_C], mi->rec[V_C], size);

#if PLATFORM_GENERAL_DEBUG
        int num_coef_u = 0, num_coef_v = 0;
        for (int i = 0; i < mi->cu_width * mi->cu_height / 4; i++)
        {
            num_coef_u += mi->coef[U_C][i] != 0 ? 1 : 0;
            num_coef_v += mi->coef[V_C][i] != 0 ? 1 : 0;
        }
#if IPCM
        if (core->mod_info_best.cu_mode == MODE_INTRA && mi->ipm[PB0][0] == IPD_IPCM && mi->ipm[PB0][1] == IPD_DM_C)
        {
            num_coef_u = 0;
            num_coef_v = 0;
        }
#endif
        assert(num_coef_u == mi->num_nz[TB0][U_C]);
        assert(num_coef_v == mi->num_nz[TB0][V_C]);
#endif
    }
    /* copy mode info */
    if (ctx->tree_status == TREE_C)
    {
        idx = 0;
        for (j = 0; j < mi->cu_height >> MIN_CU_LOG2; j++)
        {
            for (i = 0; i < mi->cu_width >> MIN_CU_LOG2; i++)
            {
                //intra chroma mode
                if (core->mod_info_best.cu_mode == MODE_INTRA)
                {
                    cu_data->ipm[1][idx + i] = mi->ipm[PB0][1];
                }

                // residual and partition
                for (k = U_C; k < N_C; k++)
                {
                    cu_data->num_nz_coef[k][idx + i] = core->mod_info_best.num_nz[TBUV0][k];
                }
            }
            idx += mi->cu_width >> MIN_CU_LOG2;
        }
    }
    else
    {
        int cu_cbf_flag = 0;
        if (ctx->tree_status == TREE_LC)
        {
            cu_cbf_flag = is_cu_nz(mi->num_nz);
        }
        else if (ctx->tree_status == TREE_L)
        {
            cu_cbf_flag = is_cu_plane_nz(mi->num_nz, Y_C);
        }
        idx = 0;
        for (j = 0; j < mi->cu_height >> MIN_CU_LOG2; j++)
        {
            for (i = 0; i < mi->cu_width >> MIN_CU_LOG2; i++)
            {
                int pb_idx_y = get_part_idx(mi->pb_part, i << 2, j << 2, mi->cu_width, mi->cu_height);
                int tb_idx_y = get_part_idx(mi->tb_part, i << 2, j << 2, mi->cu_width, mi->cu_height);
                int pb_idx_u = PB0;
                int tb_idx_u = TBUV0;
                cu_data->pred_mode[idx + i] = (u8)core->mod_info_best.cu_mode;
#if INTERPF
                cu_data->inter_filter_flag[idx + i] = core->mod_info_best.inter_filter_flag;
#endif
#if USE_IBC
#if USE_SP
                cu_data->ibc_flag[idx + i] = (core->mod_info_best.cu_mode == MODE_IBC && mi->sp_flag == FALSE);
                cu_data->sp_flag[idx + i] = (core->mod_info_best.cu_mode == MODE_IBC && mi->sp_flag == TRUE);
                if (core->mod_info_best.sp_flag == TRUE)
                {
                    assert(core->mod_info_best.cu_mode == MODE_IBC);
                    assert(core->mod_info_best.ibc_flag == FALSE);
                }
                if (core->mod_info_best.ibc_flag == TRUE)
                {
                    assert(core->mod_info_best.cu_mode == MODE_IBC);
                    assert(core->mod_info_best.sp_flag == FALSE);
                }
#else
                cu_data->ibc_flag[idx + i] = core->mod_info_best.cu_mode == MODE_IBC;
#endif
#endif
                cu_data->umve_flag[idx + i] = core->mod_info_best.umve_flag;
                cu_data->umve_idx[idx + i] = core->mod_info_best.umve_idx;
#if AFFINE_UMVE
                cu_data->affine_umve_flag[idx + i] = core->mod_info_best.affine_umve_flag;
                for (k = 0; k < VER_NUM; k++)
                {
                    cu_data->affine_umve_idx[k][idx + i] = core->mod_info_best.affine_umve_idx[k];
                }
#endif
#if TB_SPLIT_EXT
                cu_data->pb_part[idx + i] = mi->pb_part;
                cu_data->tb_part[idx + i] = mi->tb_part;
#endif
                for (k = 0; k < N_C; k++)
                {
                    cu_data->num_nz_coef[k][idx + i] = core->mod_info_best.num_nz[k == Y_C ? tb_idx_y : tb_idx_u][k];
                }
                if (cu_cbf_flag)
                {
                    MCU_SET_CBF(cu_data->map_scu[idx + i]);
                }
                else
                {
                    MCU_CLR_CBF(cu_data->map_scu[idx + i]);
                }
                MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->mod_info_best.cu_mode == MODE_INTRA, ctx->slice_num, core->qp_y);
                if (core->mod_info_best.cu_mode == MODE_SKIP)
                {
                    MCU_SET_SF(cu_data->map_scu[idx + i]);
                }
                else
                {
                    MCU_CLR_SF(cu_data->map_scu[idx + i]);
                }
#if USE_IBC
                if (ctx->info.pic_header.ibc_flag)
                {
                    if (cu_data->ibc_flag[idx + i])
                    {
                        MCU_SET_IBC(cu_data->map_scu[idx + i]);
#if USE_SP
                        MSP_CLR_SP_INFO(cu_data->map_usp[idx + i]);
                    }
                    else if (cu_data->sp_flag[idx + i])
                    {
                        MCU_SET_IBC(cu_data->map_scu[idx + i]);
                        MSP_SET_SP_INFO(cu_data->map_usp[idx + i]);
#endif
                    }
                    else
                    {
#if USE_SP
                        MSP_CLR_SP_INFO(cu_data->map_usp[idx + i]);
#endif
                        MCU_CLR_IBC(cu_data->map_scu[idx + i]);
                    }
                }
#endif
                cu_data->depth[idx + i] = (s8)core->mod_info_best.cud;

                cu_data->affine_flag[idx + i] = mi->affine_flag;
#if ETMVP
                cu_data->etmvp_flag[idx + i] = mi->etmvp_flag;
#endif
                if (mi->affine_flag)
                {
                    MCU_SET_AFF(cu_data->map_scu[idx + i], mi->affine_flag);
                }
                else
                {
                    MCU_CLR_AFF(cu_data->map_scu[idx + i]);
                }

#if AWP
                cu_data->awp_flag[idx + i] = mi->awp_flag;
                cu_data->awp_idx0[idx + i] = mi->awp_idx0;
                cu_data->awp_idx1[idx + i] = mi->awp_idx1;
#endif

#if AWP_MVR
                cu_data->awp_mvr_flag0[idx + i] = mi->awp_mvr_flag0;
                cu_data->awp_mvr_idx0[idx + i]  = mi->awp_mvr_idx0;
                cu_data->awp_mvr_flag1[idx + i] = mi->awp_mvr_flag1;
                cu_data->awp_mvr_idx1[idx + i]  = mi->awp_mvr_idx1;
#endif

                MCU_SET_X_SCU_OFF(cu_data->map_cu_mode[idx + i], i);
                MCU_SET_Y_SCU_OFF(cu_data->map_cu_mode[idx + i], j);
                MCU_SET_LOGW(cu_data->map_cu_mode[idx + i], cu_width_log2);
                MCU_SET_LOGH(cu_data->map_cu_mode[idx + i], cu_height_log2);
#if TB_SPLIT_EXT
                MCU_SET_TB_PART_LUMA(cu_data->map_pb_tb_part[idx + i], mi->tb_part);
#endif
#if SBT
                MCU_SET_SBT_INFO( cu_data->map_pb_tb_part[idx + i], mi->sbt_info );
#endif
                if (core->mod_info_best.cu_mode == MODE_INTRA)
                {
                    cu_data->mpm[0][idx + i] = mi->mpm[pb_idx_y][0];
                    cu_data->mpm[1][idx + i] = mi->mpm[pb_idx_y][1];
                    cu_data->ipm[0][idx + i] = mi->ipm[pb_idx_y][0];
                    cu_data->ipm[1][idx + i] = mi->ipm[pb_idx_u][1];
                    cu_data->ipf_flag[idx + i] = mi->ipf_flag;

                    cu_data->mv[idx + i][REFP_0][MV_X] = 0;
                    cu_data->mv[idx + i][REFP_0][MV_Y] = 0;
                    cu_data->mv[idx + i][REFP_1][MV_X] = 0;
                    cu_data->mv[idx + i][REFP_1][MV_Y] = 0;
                    cu_data->refi[idx + i][REFP_0] = -1;
                    cu_data->refi[idx + i][REFP_1] = -1;
#if BGC
                    cu_data->bgc_flag[idx + i] = 0;
                    cu_data->bgc_idx[idx + i] = 0;
#endif
#if IST
                    cu_data->ist_tu_flag[idx + i] = mi->ist_tu_flag;
#endif
#if EST
                    cu_data->est_tu_flag[idx + i] = mi->est_flag;
#endif
                }
#if USE_IBC
                else if (core->mod_info_best.cu_mode == MODE_IBC)
                {
#if USE_SP
                    cu_data->sub_string_no[idx + i] = core->mod_info_best.sub_string_no;
                    cu_data->sp_copy_direction[idx + i] = core->mod_info_best.sp_copy_direction;
                    cu_data->is_sp_pix_completed[idx + i] = core->mod_info_best.is_sp_pix_completed;
#endif
                    cu_data->mvr_idx[idx + i] = mi->mvr_idx;
#if IBC_ABVR
                    cu_data->bvr_idx[idx + i] = mi->bvr_idx;
#endif
#if EXT_AMVR_HMVP
                    cu_data->mvp_from_hmvp_flag[idx + i] = mi->mvp_from_hmvp_flag;
#endif
#if IBC_BVP
                    cu_data->cbvp_idx[idx + i] = core->mod_info_best.cbvp_idx;
#endif
#if SMVD
                    cu_data->smvd_flag[idx + i] = mi->smvd_flag;
#endif
                    cu_data->skip_idx[idx + i] = mi->skip_idx;
                    cu_data->refi[idx + i][REFP_0] = mi->refi[REFP_0]; // -1
                    cu_data->refi[idx + i][REFP_1] = mi->refi[REFP_1]; // -1
                    cu_data->mv[idx + i][REFP_0][MV_X] = mi->mv[REFP_0][MV_X];
                    cu_data->mv[idx + i][REFP_0][MV_Y] = mi->mv[REFP_0][MV_Y];
                    cu_data->mv[idx + i][REFP_1][MV_X] = 0;
                    cu_data->mv[idx + i][REFP_1][MV_Y] = 0;
                    cu_data->mvd[idx + i][REFP_0][MV_X] = mi->mvd[REFP_0][MV_X];
                    cu_data->mvd[idx + i][REFP_0][MV_Y] = mi->mvd[REFP_0][MV_Y];
                    cu_data->mvd[idx + i][REFP_1][MV_X] = 0;
                    cu_data->mvd[idx + i][REFP_1][MV_Y] = 0;
#if BGC
                    cu_data->bgc_flag[idx + i] = 0;
                    cu_data->bgc_idx[idx + i] = 0;
#endif

                    cu_data->ipm[0][idx + i] = 0; //DC mode
                    cu_data->ipm[1][idx + i] = 0;
                    cu_data->ipf_flag[idx + i] = 0;
                    cu_data->mpm[0][idx + i] = 0;
                    cu_data->mpm[1][idx + i] = 0;
                }
#endif
                else
                {
#if SUB_TMVP 
                  if (core->best_sbTmvp_flag)
                  {

                      int blk = ((i >= (mi->cu_width >> MIN_CU_LOG2) / SBTMVP_NUM_1D) ? 1 : 0) + ((j >= (mi->cu_height >> MIN_CU_LOG2) / SBTMVP_NUM_1D) ? SBTMVP_NUM_1D : 0);
                      cu_data->refi[idx + i][REFP_0] = core->best_sbTmvp[blk].ref_idx[REFP_0];
                      cu_data->refi[idx + i][REFP_1] = core->best_sbTmvp[blk].ref_idx[REFP_1];
                  }
                  else
                  {
#endif
#if MVAP
                      if (core->best_mvap_flag)
                      {
                          cu_data->refi[idx + i][REFP_0] = core->best_cu_mvfield[idx + i].ref_idx[REFP_0];
                          cu_data->refi[idx + i][REFP_1] = core->best_cu_mvfield[idx + i].ref_idx[REFP_1];
                      }
                      else
                      {
#endif
                          cu_data->refi[idx + i][REFP_0] = mi->refi[REFP_0];
                          cu_data->refi[idx + i][REFP_1] = mi->refi[REFP_1];
#if MVAP
                      }
#endif
#if SUB_TMVP
                  }
#endif
#if ETMVP
                    if (mi->etmvp_flag)
                    {
                        cu_data->refi[idx + i][REFP_0] = core->best_etmvp_mvfield[idx + i].ref_idx[REFP_0];
                        cu_data->refi[idx + i][REFP_1] = core->best_etmvp_mvfield[idx + i].ref_idx[REFP_1];
                    }
#endif
                    cu_data->mvr_idx[idx + i] = mi->mvr_idx;
#if EXT_AMVR_HMVP
                    cu_data->mvp_from_hmvp_flag[idx + i] = mi->mvp_from_hmvp_flag;
#endif
#if SMVD
                    cu_data->smvd_flag[idx + i] = mi->smvd_flag;
#endif
                    cu_data->skip_idx[idx + i] = mi->skip_idx;
#if BGC
                    cu_data->bgc_flag[idx + i] = mi->bgc_flag;
                    cu_data->bgc_idx[idx + i] = mi->bgc_idx;
#endif
#if SUB_TMVP
                    if (core->best_sbTmvp_flag)
                    {

                        int blk = ((i >= (mi->cu_width >> MIN_CU_LOG2) / SBTMVP_NUM_1D) ? 1 : 0) + ((j >= (mi->cu_height >> MIN_CU_LOG2) / SBTMVP_NUM_1D) ? SBTMVP_NUM_1D : 0);
                        cu_data->mv[idx + i][REFP_0][MV_X] = core->best_sbTmvp[blk].mv[REFP_0][MV_X];
                        cu_data->mv[idx + i][REFP_0][MV_Y] = core->best_sbTmvp[blk].mv[REFP_0][MV_Y];
                        cu_data->mv[idx + i][REFP_1][MV_X] = core->best_sbTmvp[blk].mv[REFP_1][MV_X];
                        cu_data->mv[idx + i][REFP_1][MV_Y] = core->best_sbTmvp[blk].mv[REFP_1][MV_Y];
                        
                        cu_data->mvd[idx + i][REFP_0][MV_X] = 0;
                        cu_data->mvd[idx + i][REFP_0][MV_Y] = 0;
                        cu_data->mvd[idx + i][REFP_1][MV_X] = 0;
                        cu_data->mvd[idx + i][REFP_1][MV_Y] = 0;
                    }
                    else
                    {
#endif
#if MVAP
                        if (core->best_mvap_flag)
                        {
                            cu_data->mv[idx + i][REFP_0][MV_X] = core->best_cu_mvfield[idx + i].mv[REFP_0][MV_X];
                            cu_data->mv[idx + i][REFP_0][MV_Y] = core->best_cu_mvfield[idx + i].mv[REFP_0][MV_Y];
                            cu_data->mv[idx + i][REFP_1][MV_X] = core->best_cu_mvfield[idx + i].mv[REFP_1][MV_X];
                            cu_data->mv[idx + i][REFP_1][MV_Y] = core->best_cu_mvfield[idx + i].mv[REFP_1][MV_Y];
                            cu_data->mvd[idx + i][REFP_0][MV_X] = 0;
                            cu_data->mvd[idx + i][REFP_0][MV_Y] = 0;
                            cu_data->mvd[idx + i][REFP_1][MV_X] = 0;
                            cu_data->mvd[idx + i][REFP_1][MV_Y] = 0;
                        }
                        else
                        {
#endif
                            cu_data->mv[idx + i][REFP_0][MV_X] = mi->mv[REFP_0][MV_X];
                            cu_data->mv[idx + i][REFP_0][MV_Y] = mi->mv[REFP_0][MV_Y];
                            cu_data->mv[idx + i][REFP_1][MV_X] = mi->mv[REFP_1][MV_X];
                            cu_data->mv[idx + i][REFP_1][MV_Y] = mi->mv[REFP_1][MV_Y];
                            cu_data->mvd[idx + i][REFP_0][MV_X] = mi->mvd[REFP_0][MV_X];
                            cu_data->mvd[idx + i][REFP_0][MV_Y] = mi->mvd[REFP_0][MV_Y];
                            cu_data->mvd[idx + i][REFP_1][MV_X] = mi->mvd[REFP_1][MV_X];
                            cu_data->mvd[idx + i][REFP_1][MV_Y] = mi->mvd[REFP_1][MV_Y];
#if MVAP
                        }
#endif
#if SUB_TMVP
                    }
#endif
#if ETMVP
                    if (mi->etmvp_flag)
                    {
                        cu_data->mv[idx + i][REFP_0][MV_X] = core->best_etmvp_mvfield[idx + i].mv[REFP_0][MV_X];
                        cu_data->mv[idx + i][REFP_0][MV_Y] = core->best_etmvp_mvfield[idx + i].mv[REFP_0][MV_Y];
                        cu_data->mv[idx + i][REFP_1][MV_X] = core->best_etmvp_mvfield[idx + i].mv[REFP_1][MV_X];
                        cu_data->mv[idx + i][REFP_1][MV_Y] = core->best_etmvp_mvfield[idx + i].mv[REFP_1][MV_Y];
                        cu_data->mvd[idx + i][REFP_0][MV_X] = 0;
                        cu_data->mvd[idx + i][REFP_0][MV_Y] = 0;
                        cu_data->mvd[idx + i][REFP_1][MV_X] = 0;
                        cu_data->mvd[idx + i][REFP_1][MV_Y] = 0;
                    }
#endif
                }
            }
            idx += mi->cu_width >> MIN_CU_LOG2;
        }

        if (mi->affine_flag)
        {
            enc_set_affine_mvf(ctx, core, mi);
        }
#if AWP
        if (mi->awp_flag)
        {
            enc_set_awp_mvf(ctx, core, mi);
        }
#endif
    }
}

static void update_map_scu(ENC_CTX *ctx, ENC_CORE *core, int x, int y, int src_cuw, int src_cuh, u8 tree_status)
{
    u32  *map_scu = 0, *src_map_scu = 0;
    s8   *map_ipm = 0, *src_map_ipm = 0;
#if TB_SPLIT_EXT
    u32  *map_pb_tb_part = 0, *src_map_pb_tb_part = 0;
#endif
#if USE_SP
    u8   *map_usp = 0, *src_map_usp = 0;
    int  size_usp;
#endif
    s16(*map_mv)[REFP_NUM][MV_D] = 0, (*src_map_mv)[REFP_NUM][MV_D] = 0;
    s8(*map_refi)[REFP_NUM] = 0;
    s8 **src_map_refi = NULL;
    s8   *map_depth = 0, *src_depth = 0;
    int   size_depth;
    int   w, h, i, size, size_ipm, size_mv, size_refi;
    int   log2_src_cuw, log2_src_cuh;
    int   scu_x, scu_y;
    u32  *map_cu_mode = 0, *src_map_cu_mode = 0;
    assert(ctx->tree_status != TREE_C);

    scu_x = x >> MIN_CU_LOG2;
    scu_y = y >> MIN_CU_LOG2;
    log2_src_cuw = CONV_LOG2(src_cuw);
    log2_src_cuh = CONV_LOG2(src_cuh);
    map_scu = ctx->map.map_scu + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_scu = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_scu;
    map_ipm = ctx->map.map_ipm + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_ipm = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].ipm[0];
#if TB_SPLIT_EXT
    map_pb_tb_part = ctx->map.map_pb_tb_part + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_pb_tb_part = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_pb_tb_part;
#endif
#if USE_SP
    map_usp = ctx->map.map_usp + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_usp = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_usp;
#endif
    map_mv = ctx->map.map_mv + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_mv = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].mv;
    map_refi = ctx->map.map_refi + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_refi = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].refi;
    map_depth = ctx->map.map_depth + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_depth = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].depth;
    map_cu_mode = ctx->map.map_cu_mode + scu_y * ctx->info.pic_width_in_scu + scu_x;
    src_map_cu_mode = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_cu_mode;
    if (x + src_cuw > ctx->info.pic_width)
    {
        w = (ctx->info.pic_width - x) >> MIN_CU_LOG2;
    }
    else
    {
        w = (src_cuw >> MIN_CU_LOG2);
    }
    if (y + src_cuh > ctx->info.pic_height)
    {
        h = (ctx->info.pic_height - y) >> MIN_CU_LOG2;
    }
    else
    {
        h = (src_cuh >> MIN_CU_LOG2);
    }
    size = sizeof(u32) * w;
    size_ipm = sizeof(u8) * w;
    size_mv = sizeof(s16) * w * REFP_NUM * MV_D;
    size_refi = sizeof(s8) * w * REFP_NUM;
    size_depth = sizeof(s8) * w;
#if USE_SP
    size_usp = sizeof(u8) * w;
#endif
    for (i = 0; i < h; i++)
    {
        com_mcpy(map_scu, src_map_scu, size);
        map_scu += ctx->info.pic_width_in_scu;
        src_map_scu += (src_cuw >> MIN_CU_LOG2);
        com_mcpy(map_cu_mode, src_map_cu_mode, size);
        map_cu_mode += ctx->info.pic_width_in_scu;
        src_map_cu_mode += (src_cuw >> MIN_CU_LOG2);
#if TB_SPLIT_EXT
        com_mcpy(map_pb_tb_part, src_map_pb_tb_part, size);
        map_pb_tb_part += ctx->info.pic_width_in_scu;
        src_map_pb_tb_part += (src_cuw >> MIN_CU_LOG2);
#endif
        com_mcpy(map_ipm, src_map_ipm, size_ipm);
        map_ipm += ctx->info.pic_width_in_scu;
        src_map_ipm += (src_cuw >> MIN_CU_LOG2);
        com_mcpy(map_mv, src_map_mv, size_mv);
        map_mv += ctx->info.pic_width_in_scu;
        src_map_mv += (src_cuw >> MIN_CU_LOG2);
        com_mcpy(map_refi, *(src_map_refi), size_refi);
        map_refi += ctx->info.pic_width_in_scu;
        src_map_refi += (src_cuw >> MIN_CU_LOG2);
        com_mcpy(map_depth, src_depth, size_depth);
        map_depth += ctx->info.pic_width_in_scu;
        src_depth += (src_cuw >> MIN_CU_LOG2);
#if USE_SP
        com_mcpy(map_usp, src_map_usp, size_usp);
        map_usp += ctx->info.pic_width_in_scu;
        src_map_usp += (src_cuw >> MIN_CU_LOG2);
#endif
    }
}

static void clear_map_scu(ENC_CTX *ctx, ENC_CORE *core, int x, int y, int cu_width, int cu_height)
{
    u32 *map_scu;
    int w, h, i, size;
    u32 *map_cu_mode = ctx->map.map_cu_mode + (y >> MIN_CU_LOG2) * ctx->info.pic_width_in_scu + (x >> MIN_CU_LOG2);
    map_scu = ctx->map.map_scu + (y >> MIN_CU_LOG2) * ctx->info.pic_width_in_scu + (x >> MIN_CU_LOG2);
#if USE_SP
    u8 *map_usp = ctx->map.map_usp + (y >> MIN_CU_LOG2) * ctx->info.pic_width_in_scu + (x >> MIN_CU_LOG2);
#endif
    if (x + cu_width > ctx->info.pic_width)
    {
        cu_width = ctx->info.pic_width - x;
    }
    if (y + cu_height > ctx->info.pic_height)
    {
        cu_height = ctx->info.pic_height - y;
    }
    w = (cu_width >> MIN_CU_LOG2);
    h = (cu_height >> MIN_CU_LOG2);
    size = sizeof(u32) * w;
#if USE_SP
    int size_usp = sizeof(u8) * w;
#endif
    for (i = 0; i < h; i++)
    {
        com_mset(map_scu, 0, size);
        map_scu += ctx->info.pic_width_in_scu;
        com_mset(map_cu_mode, 0, size);
        map_cu_mode += ctx->info.pic_width_in_scu;
#if USE_SP
        com_mset(map_usp, 0, size_usp);
        map_usp += ctx->info.pic_width_in_scu;
#endif
    }
}

void enc_init_bef_data(ENC_CORE* core, ENC_CTX* ctx)
{
    int stride = 1 << ctx->log2_culine;
    int max_size = stride * stride; //size of a CTU
    int boundary_CTU = 0;
    int ctu_size = 1 << (ctx->info.log2_max_cuwh);
    int x0 = core->x_pel;
    int y0 = core->y_pel;
    if ((x0 / ctu_size + 1) * ctu_size > ctx->info.pic_width || (y0 / ctu_size + 1) * ctu_size > ctx->info.pic_height)
        boundary_CTU = 1;
    for (int m1 = 0; m1 < MAX_CU_DEPTH; m1++)
    {
        for (int m2 = 0; m2 < MAX_CU_DEPTH; m2++)
        {
            com_mset(&core->bef_data[m1][m2], 0, sizeof(ENC_BEF_DATA) * max_size);
        }
    }
}



static double mode_coding_unit(ENC_CTX *ctx, ENC_CORE *core, int x, int y, int cu_width_log2, int cu_height_log2, int cud)
{
    double  cost_best, cost;
    COM_MODE *mi = &core->mod_info_best;
    int bit_depth = ctx->info.bit_depth_internal;
    u8 cons_pred_mode = NO_MODE_CONS;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;

    s8 ipf_flag;
    s8 ipf_passes_num = (ctx->info.sqh.ipf_enable_flag && (cu_width < MAX_CU_SIZE) && (cu_height < MAX_CU_SIZE)) ? 2 : 1;

#if SPLIT_DEBUG
    if (cu_width > cu_height * ctx->info.sqh.max_part_ratio || cu_height > cu_width * ctx->info.sqh.max_part_ratio)
        assert(0);
    if ((cu_width > 64 && cu_height < 64) || (cu_height > 64 && cu_width < 64))
        assert(0);
#endif
    mode_cu_init(ctx, core, x, y, cu_width_log2, cu_height_log2, cud);
#if MODE_CONS
    cons_pred_mode = ctx->cons_pred_mode;
#endif
#if CHROMA_NOT_SPLIT //derive corresponding luma mode
    if (ctx->tree_status == TREE_C)
    {
        ENC_CU_DATA* cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
        int cu_w_scu = PEL2SCU(cu_width);
        int cu_h_scu = PEL2SCU(cu_height);
        int luma_pos_scu = (cu_w_scu - 1) + (cu_h_scu - 1) * cu_w_scu; // bottom-right
        u8  luma_pred_mode = cu_data->pred_mode[luma_pos_scu];
        cons_pred_mode = (luma_pred_mode == MODE_INTRA || luma_pred_mode == MODE_IBC) ? ONLY_INTRA : ONLY_INTER;

#if USE_IBC
        if (luma_pred_mode != MODE_INTRA && luma_pred_mode != MODE_IBC)
#else
        if (luma_pred_mode != MODE_INTRA)
#endif
        {
            cost_best = pinter_residue_rdo_chroma(ctx, core
#if DMVR
                                                  , 0
#endif
            );
            copy_to_cu_data(ctx, core, mi);
            return cost_best;
        }
        else
        {
            ipf_passes_num = 1;
        }
    }
#endif
    /* inter *************************************************************/
    cost_best = MAX_COST;
    core->cost_best = MAX_COST;
#if SUB_TMVP
    core->sbTmvp_flag = 0;
    core->best_sbTmvp_flag = 0;
#endif
#if MVAP
    core->mvap_flag      = 0;
    core->best_mvap_flag = 0;
#endif
#if ETMVP
    core->mod_info_curr.etmvp_flag = 0;
#endif
#if AWP
    core->mod_info_curr.awp_flag = 0;
#endif
#if USE_IBC
    if (ctx->slice_type != SLICE_I && cons_pred_mode != ONLY_INTRA)
#endif
    {
        cost = analyze_inter_cu(ctx, core);
        if (cu_width > 64 || cu_height > 64)
        {
            assert(!is_cu_nz(core->mod_info_best.num_nz));
        }
#if PRINT_CU
#if FIXED_SPLIT
        printf("\n[inter] ctu_idx_seq %5d x %4d y %4d w %3d h %3d tree %d cost %12.1f", ctx->ctu_idx_in_sequence, x, y, cu_width, cu_height, ctx->tree_status, cost);
#else
        printf("\n[inter] x %4d y %4d w %3d h %3d tree %d cost %12.1f", x, y, cu_width, cu_height, ctx->tree_status, cost);
#endif
        double val = 2404.8;
        if (cost - val > -0.1 && cost - val < 0.1)
        {
            int a = 0;
        }
#endif
        if (cost < cost_best)
        {
            cost_best = cost;
            copy_to_cu_data(ctx, core, mi);
        }
    }
#if USE_IBC
    ///* IBC *************************************************************/
    if( (is_cu_nz( core->mod_info_best.num_nz ) || cost_best == MAX_COST) && ctx->tree_status != TREE_C
        && ctx->info.pic_header.ibc_flag == 1
#if USE_SP&& !SP_IBC1ST
        && ctx->info.pic_header.sp_pic_flag == 0
#endif
        && cons_pred_mode != ONLY_INTER
        )
    {
        if ((1 << cu_width_log2) <= IBC_MAX_CAND_SIZE && (1 << cu_height_log2) <= IBC_MAX_CAND_SIZE)
        {
            if (ctx->tree_status == TREE_C)
            { 
                assert(0);
#if IST
                core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
                core->mod_info_curr.inter_filter_flag = 0;
#endif
                cost = pibc_residue_rdo_chroma(ctx, core);

                check_best_ibc_mode(core, &ctx->pibc, cost, &cost_best);
            }
            else
            {
                cost = analyze_ibc_cu(ctx, core);
            }

            if (cost < cost_best)
            {
#if USE_SP
                core->mod_info_best.sp_flag = 0; 
#endif
                cost_best = cost;
                copy_to_cu_data(ctx, core, mi);
            }
        }
    }
#endif


#if FIMC
    // copy table for dt and ipf, last, best, core
    COM_CNTMPM cntmpm_cands_curr;
    COM_CNTMPM cntmpm_cands_last;
    if (ctx->info.sqh.fimc_enable_flag)
    {
        com_cntmpm_copy(&cntmpm_cands_last, &core->cntmpm_cands);
        com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
    }
#endif

#if USE_IBC
    if ((ctx->slice_type == SLICE_I || is_cu_nz(core->mod_info_best.num_nz) || cost_best == MAX_COST) && cons_pred_mode != ONLY_INTER)
#else
    /* intra *************************************************************/
    if ((ctx->slice_type == SLICE_I || is_cu_nz(core->mod_info_best.num_nz) || cost_best == MAX_COST) && cons_pred_mode != ONLY_INTER)
#endif
    {
        if (cu_width <= 64 && cu_height <= 64)
        {
            assert(core->cost_best == cost_best);
            core->dist_cu_best = COM_INT32_MAX;
            if (core->cost_best != MAX_COST)
            {
                ENC_PINTRA *pi = &ctx->pintra;
                core->inter_satd = enc_satd_16b(cu_width_log2, cu_height_log2, pi->addr_org[Y_C] + (y * pi->stride_org[Y_C]) + x, mi->pred[0], pi->stride_org[Y_C], 1 << cu_width_log2, bit_depth);
            }
            else
            {
                core->inter_satd = COM_UINT32_MAX;
            }

            for (ipf_flag = 0; ipf_flag < ipf_passes_num; ++ipf_flag)//根据IPF_FLAG，执行被允许的IPF情况（若允许IPF，执行两次）
            {
#if FIMC
                // copy table for dt and ipf
                if (ctx->info.sqh.fimc_enable_flag && ipf_flag > 0)//默认会先执行不做ipf的情况，若开了ipf第二次执行时会进入if中
                {
                    com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_last);
                }
#endif
                core->mod_info_curr.ipf_flag = ipf_flag;
                cost = analyze_intra_cu(ctx, core);//递归最底层的cost
#if PRINT_CU
#if FIXED_SPLIT
                printf("\n[intra] ctu_idx_seq %5d x %4d y %4d w %3d h %3d tree %d cost %12.1f", ctx->ctu_idx_in_sequence, x, y, cu_width, cu_height, ctx->tree_status, cost == MAX_COST ? 9999999 : cost);
#else
                printf("\n[intra] x %4d y %4d w %3d h %3d tree %d cost %12.1f", x, y, cu_width, cu_height, ctx->tree_status, cost == MAX_COST ? 9999999 : cost);
#endif
                double val = 2185.5;
                if (cost - val > -0.1 && cost - val < 0.1)
                {
                    int a = 0;
                }
#endif
                if (cost < cost_best)// 为什么会乱码啊，选出当前CU是否采用IPF 
                {
                    cost_best = cost;
                    core->mod_info_best.cu_mode = MODE_INTRA;
#if USE_IBC
                    core->mod_info_best.ibc_flag = 0;
#endif
#if USE_SP
                    core->mod_info_best.sp_flag = 0;
                    core->cost_best = cost_best;
#endif
                    core->mod_info_best.ipf_flag = ipf_flag;

                    SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
                    core->dist_cu_best = core->dist_cu;
#if TB_SPLIT_EXT
                    mi->pb_part = core->best_pb_part_intra;
                    mi->tb_part = core->best_tb_part_intra;
#endif
#if SBT
                    mi->sbt_info = 0;
#endif
                    mi->affine_flag = 0;
#if AFFINE_UMVE
                    mi->affine_umve_flag = 0;
#endif
#if INTERPF
                    mi->inter_filter_flag = 0;
#endif
#if ETMVP
                    mi->etmvp_flag = 0;
#endif
#if AWP
                    mi->awp_flag = 0;
#endif
#if AWP_MVR
                    mi->awp_mvr_flag0 = 0;
                    mi->awp_mvr_flag1 = 0;
#endif
#if FIMC
                    // copy table for dt and ipf
                    if (ctx->info.sqh.fimc_enable_flag)
                    {
                        com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
                    }
#endif
                    copy_to_cu_data(ctx, core, mi);
                }
            }
        }
    }

    /* sm *************************************************************/
#if USE_IBC && USE_SP
    if ((is_cu_nz(core->mod_info_best.num_nz) || cost_best == MAX_COST) && ctx->tree_status != TREE_C
        && ctx->info.pic_header.ibc_flag == 1
        && cons_pred_mode != ONLY_INTER
#if USE_SP
        && ctx->info.pic_header.sp_pic_flag==1
#endif
        )
    {
        //=============================
        /* do String Prediction mode in CU level *************************************************************/
        if (IS_VALID_SP_CU_SIZE(cu_width, cu_height)) 
        {
            assert(core->cost_best == cost_best);
            if (ctx->tree_status == TREE_C) 
            {
                assert(0);
            }
            else 
            {
                core->mod_info_best.cur_bst_rdcost = cost_best;
                cost = analyze_sm_cu(ctx, core);
            }
            if (cost < cost_best)
            {
                cost_best = cost;
                core->cost_best = cost;
                core->mod_info_best.cu_mode = MODE_IBC;
#if USE_IBC
                core->mod_info_best.ibc_flag = 0;
#endif
                core->mod_info_best.ipf_flag = 0;

                SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
                
#if TB_SPLIT_EXT
                mi->pb_part = SIZE_2Nx2N;
                mi->tb_part = SIZE_2Nx2N;
#endif
#if SBT
                mi->sbt_info = 0;
#endif
                mi->affine_flag = 0;
#if INTERPF
                mi->inter_filter_flag = 0;
#endif
#if AWP
                mi->awp_flag = 0;
#endif
                copy_to_cu_data(ctx, core, mi);
            }
            else
            {
                if (ctx->tree_status != TREE_C)
                {
                    core->mod_info_best.sp_flag = 0;
                    copy_spinfo_to_cudata(&core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], &core->mod_info_best);
                }
            }
        }
    }
#endif

#if USE_IBC && USE_SP &&!SP_IBC1ST
    ///* IBC *************************************************************/
    if ((is_cu_nz(core->mod_info_best.num_nz) || cost_best == MAX_COST) && ctx->tree_status != TREE_C
        && ctx->info.pic_header.ibc_flag == 1
#if USE_SP
        && ctx->info.pic_header.sp_pic_flag == 1
#endif
        && cons_pred_mode != ONLY_INTER)
    {
        if ((1 << cu_width_log2) <= IBC_MAX_CAND_SIZE && (1 << cu_height_log2) <= IBC_MAX_CAND_SIZE)
        {
            if (ctx->tree_status == TREE_C)
            {
                assert(0);
#if IST
                core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
                core->mod_info_curr.inter_filter_flag = 0;
#endif
                cost = pibc_residue_rdo_chroma(ctx, core);

                check_best_ibc_mode(core, &ctx->pibc, cost, &cost_best);
            }
            else
            {
                cost = analyze_ibc_cu(ctx, core);
            }
            if (cost < cost_best)
            {
#if USE_SP
                core->mod_info_best.sp_flag = 0; 
#endif
                cost_best = cost;
                copy_to_cu_data(ctx, core, mi);
            }
        }
    }
#endif

#if IPCM
    /* ipcm *************************************************************/
#if USE_IBC
    if (ctx->info.sqh.ipcm_enable_flag && (ctx->slice_type == SLICE_I || is_cu_nz(core->mod_info_best.num_nz) || cost_best == MAX_COST) && cons_pred_mode != ONLY_INTER)
#endif
    {
#if FIMC
        // copy table for dt and ipf
        if (ctx->info.sqh.fimc_enable_flag)
        {
            com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_last);
        }
#endif
        cost = analyze_ipcm_cu(ctx, core);
        if (cost < cost_best)
        {
            cost_best = cost;
            core->mod_info_best.cu_mode = MODE_INTRA;
#if USE_IBC
            core->mod_info_best.ibc_flag = 0;
#endif
#if USE_SP
            core->mod_info_best.sp_flag = 0;
#endif
            core->mod_info_best.ipf_flag = 0;
            SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
            core->dist_cu_best = core->dist_cu;
#if TB_SPLIT_EXT
            mi->pb_part = SIZE_2Nx2N;
            mi->tb_part = SIZE_2Nx2N;
#endif
            mi->affine_flag = 0;
#if AFFINE_UMVE
            mi->affine_umve_flag = 0;
#endif
#if AWP_MVR
            mi->awp_flag = 0;
            mi->awp_mvr_flag0 = 0;
            mi->awp_mvr_flag1 = 0;
#endif
#if INTERPF
            mi->inter_filter_flag = 0;
#endif
#if SBT
            mi->sbt_info = 0;
#endif

#if FIMC
            // copy table for dt and ipf
            if (ctx->info.sqh.fimc_enable_flag)
            {
                // update table for ipcm
                com_cntmpm_update(&core->cntmpm_cands, core->mod_info_best.ipm[PB0][0]);
                com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
            }
#endif
            copy_to_cu_data(ctx, core, mi);
        }
    }
#endif

#if FIMC
    // copy table for dt and ipf
    if (ctx->info.sqh.fimc_enable_flag)
    {
        com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_curr);
    }
#endif

    return cost_best;
}

#if FIXED_SPLIT
static void cln_rest_split(SPLIT_MODE target_split_mode, int* split_allow)
{
    for (SPLIT_MODE i = NO_SPLIT; i <= SPLIT_QUAD; i++)
    {
        if (i != target_split_mode)
            split_allow[i] = 0;
    }
}
#endif

#if FS_SAME_SIZE_PER_X_CTU
static void get_curr_split_for_target_size(ENC_CTX *ctx, int cu_w, int cu_h, int target_w, int target_h, int* split_allow)
{
    if (cu_w < target_w || cu_h < target_h)
    {
        if (split_allow[NO_SPLIT])
        {
            cln_rest_split(NO_SPLIT, split_allow);
        }
        else
        {
            assert(ctx->slice_type == SLICE_I && cu_w == 128 && cu_h == 128);
        }
    }
    else if (cu_w == target_w && cu_h == target_h)
    {
        if (split_allow[NO_SPLIT])
        {
            cln_rest_split(NO_SPLIT, split_allow);
        }
        else
        {
            assert(ctx->slice_type == SLICE_I && cu_w == 128 && cu_h == 128);
        }
    }
    else if (cu_w > target_w && cu_h > target_h)
    {
        if (cu_w >= target_w * 4 && cu_h >= target_h * 2 && split_allow[SPLIT_EQT_VER] && target_w >= 8 && target_h >= 8)
        {
            cln_rest_split(SPLIT_EQT_VER, split_allow);
        }
        else if (cu_w >= target_w * 2 && cu_h >= target_h * 4 && split_allow[SPLIT_EQT_HOR] && target_w >= 8 && target_h >= 8)
        {
            cln_rest_split(SPLIT_EQT_HOR, split_allow);
        }
        else if (split_allow[SPLIT_QUAD])
        {
            cln_rest_split(SPLIT_QUAD, split_allow);
        }
        else if (split_allow[SPLIT_BI_HOR])
        {
            cln_rest_split(SPLIT_BI_HOR, split_allow);
        }
        else if (split_allow[SPLIT_BI_VER])
        {
            cln_rest_split(SPLIT_BI_VER, split_allow);
        }
        else
        {
            assert(0);
        }
    }
    else if (cu_w == target_w)
    {
        assert(cu_h > target_h);
        if (split_allow[SPLIT_BI_HOR])
        {
            cln_rest_split(SPLIT_BI_HOR, split_allow);
        }
        else
        {
            assert(ctx->slice_type == SLICE_I && cu_w == 128 && cu_h == 128);
        }
    }
    else if (cu_h == target_h)
    {
        assert(cu_w > target_w);
        if (split_allow[SPLIT_BI_VER])
        {
            cln_rest_split(SPLIT_BI_VER, split_allow);
        }
        else
        {
            assert(ctx->slice_type == SLICE_I && cu_w == 128 && cu_h == 128);
        }
    }
    else
    {
        assert(0);
    }
}

static void get_target_size_CTU(ENC_CTX *ctx, int* target_size)
{
    assert(FS_SAME_SIZE_X_VAL >= 1);
    int size_idx = ctx->ctu_idx_in_sequence / FS_SAME_SIZE_X_VAL;
    if (tbl_target_size_list[size_idx][0] == -1)
    {
        ctx->ctu_idx_in_sequence = 0;
        size_idx = 0;
    }
    target_size[0] = tbl_target_size_list[size_idx][0];
    target_size[1] = tbl_target_size_list[size_idx][1];
}

static void fixed_split_for_target_size(ENC_CTX *ctx, ENC_CORE *core, int cu_width_log2, int cu_height_log2, int qt_depth, int bet_depth, int* split_allow)
{
    if ((core->x_lcu + 1) * ctx->info.max_cuwh > ctx->info.pic_width || (core->y_lcu + 1) * ctx->info.max_cuwh > ctx->info.pic_height)
        return;

    int cu_w = 1 << cu_width_log2;
    int cu_h = 1 << cu_height_log2;
    int target_size[2];

    get_target_size_CTU(ctx, target_size);
    get_curr_split_for_target_size(ctx, cu_w, cu_h, target_size[0], target_size[1], split_allow);
    int num_split_mode = 0;
    for (int i = 0; i <= SPLIT_QUAD; i++)
        num_split_mode += split_allow[i];
    assert(num_split_mode == 1);
}
#endif

#if FS_ALL_COMBINATION
static void fixed_split_for_target_split_combination(ENC_CTX *ctx, ENC_CORE *core, int cu_width_log2, int cu_height_log2, int qt_depth, int bet_depth, int* split_allow)
{
    COM_MODE* mod_info_curr = &core->mod_info_curr;
    if ((core->x_lcu + 1) * ctx->info.max_cuwh > ctx->info.pic_width || (core->y_lcu + 1) * ctx->info.max_cuwh > ctx->info.pic_height)
        return;

    int cu_w = 1 << cu_width_log2;
    int cu_h = 1 << cu_height_log2;
    int split_depth = qt_depth + bet_depth;
    SPLIT_MODE target_split = split_depth == 6 ? NO_SPLIT : ctx->split_combination[split_depth];

    if (split_allow[target_split])
    {
        cln_rest_split(target_split, split_allow);
    }
    else
    {
        int idx = (mod_info_curr->x_pos + mod_info_curr->y_pos + cu_w + cu_h) % 3;
        int allow_mode_idx = 0;
        SPLIT_MODE allow_mode;
        for (SPLIT_MODE i = NO_SPLIT; i <= SPLIT_QUAD; i++)
        {
            if (split_allow[i] && i != target_split)
            {
                allow_mode = i;
                allow_mode_idx++;
                if (allow_mode_idx == idx + 1)
                    break;
            }
        }
        assert(allow_mode_idx);
        cln_rest_split(allow_mode, split_allow);
    }

    int num_split_mode = 0;
    for (int i = 0; i <= SPLIT_QUAD; i++)
        num_split_mode += split_allow[i];
    assert(num_split_mode == 1);
}
#endif

static void check_run_split(ENC_CORE *core, int cu_width_log2, int cu_height_log2, int cup, int next_split, int do_curr, int do_split, int* split_allow, int boundary)
{
    int i;
    double min_cost = MAX_COST;
    int run_list[MAX_SPLIT_NUM]; //a smaller set of allowed split modes based on a save & load technique
#if SPLIT_DEBUG
    int split_allow_save[MAX_SPLIT_NUM];
    memcpy(split_allow_save, split_allow, sizeof(int)*MAX_SPLIT_NUM);
#endif
    if (!next_split)
    {
        assert(split_allow[0] == 1);
        for (i = 1; i < MAX_SPLIT_NUM; i++)
        {
            split_allow[i] = 0;
        }
        return;
    }

    if (core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_visit)
    {
        if ((core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].nosplit < 1
                && core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split >= 1))
        {
            run_list[0] = 0;
            for (i = 1; i < MAX_SPLIT_NUM; i++)
            {
                if (core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[i] < min_cost && split_allow[i])
                {
                    min_cost = core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[i];
                }
            }
            if (min_cost == MAX_COST)
            {
                run_list[0] = 1;
                for (i = 1; i < MAX_SPLIT_NUM; i++)
                {
                    run_list[i] = 0;
                }
            }
            else
            {
                for (i = 1; i < MAX_SPLIT_NUM; i++)
                {
#if RDO_DBK //harmonize with cu split fast algorithm
                    double th = (min_cost < 0) ? 0.99 : 1.01;
                    if (core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[i] <= th * min_cost)
#else
                    if (core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[i] <= (1.01) * min_cost)
#endif
                    {
                        run_list[i] = 1;
                    }
                    else
                    {
                        run_list[i] = 0;
                    }
                }
            }
        }
        else if (core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].nosplit == 0
                 && core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split == 0)
        {
            run_list[0] = 1;
            for (i = 1; i < MAX_SPLIT_NUM; i++)
            {
                run_list[i] = 0;
            }
        }
        else
        {
            run_list[0] = 1;
            for (i = 1; i < MAX_SPLIT_NUM; i++)
            {
                run_list[i] = 0;
            }
        }
    }
    else
    {
        for (i = 0; i < MAX_SPLIT_NUM; i++)
        {
            run_list[i] = 1;
            core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[i] = MAX_COST;
        }
        run_list[0] &= do_curr;
        for (i = 1; i < MAX_SPLIT_NUM; i++)
        {
            run_list[i] &= do_split;
        }
    }

    //modified split_allow by the save & load decision
    int num_run = 0;
    for (i = 1; i < MAX_SPLIT_NUM; i++)
    {
        split_allow[i] = run_list[i] && split_allow[i];
        num_run += split_allow[i];
    }
    //if all further splitting modes are not tried, at least we need try NO_SPLIT
    if (num_run == 0)
    {
        assert(split_allow[0] == 1);
    }
    else
    {
        split_allow[0] = run_list[0] && split_allow[0];
    }

#if SPLIT_DEBUG
    //debug
    for (i = 0; i < MAX_SPLIT_NUM; i++)
    {
        if (split_allow[i])
            assert(split_allow_save[i]);
    }
#endif
}

#if RDO_DBK
void calc_delta_dist_filter_boundary(ENC_CTX* ctx, ENC_CORE *core, COM_PIC *pic_rec, COM_PIC *pic_org, int cu_width, int cu_height, pel(*src)[MAX_CU_DIM],
                                     int s_src, int x, int y, u8 intra_flag, u8 cu_cbf, s8* refi, s16(*mv)[MV_D], u8 is_mv_from_mvf)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j;
    int cu_width_log2 = CONV_LOG2(cu_width);
    int cu_height_log2 = CONV_LOG2(cu_height);
    assert(cu_width_log2 >= 2 && cu_height_log2 >= 2);
    int x_offset = 8; //for preparing deblocking filter taps
    int y_offset = 8;
    int x_tm = 4; //for calculating template dist
    int y_tm = 4; //must be the same as x_tm
    int log2_x_tm = CONV_LOG2(x_tm);
    int log2_y_tm = CONV_LOG2(y_tm);
    COM_PIC *pic_dbk = ctx->pic_dbk;
    int s_l_dbk = pic_dbk->stride_luma;
    int s_c_dbk = pic_dbk->stride_chroma;
    int s_l_org = pic_org->stride_luma;
    pel* dst_y = pic_dbk->y + y * s_l_dbk + x;
    pel* org_y = pic_org->y + y * s_l_org + x;
    int x_scu = x >> MIN_CU_LOG2;
    int y_scu = y >> MIN_CU_LOG2;
    int t = x_scu + y_scu * ctx->info.pic_width_in_scu;
    //cu info to save
    u8 intra_flag_save, cu_cbf_save;
    u8 do_filter = 0;
    if (ctx->info.pic_header.loop_filter_disable_flag == 0)
    {
        do_filter = 1;
    }
    if (do_filter == 0)
    {
        ctx->delta_dist = 0;
        return; //if no filter is applied, just return delta_dist as 0
    }
    //reset
    ctx->dist_filter = ctx->dist_nofilt = 0;
    /********************** prepare pred/rec pixels (not filtered) ****************************/
    //fill src to dst
    for (i = 0; i < cu_height; i++)
        com_mcpy(dst_y + i * s_l_dbk, src[Y_C] + i * s_src, cu_width * sizeof(pel));
    //fill top
    if (y != 0) // note: may not correct in patch
    {
        for (i = 0; i < y_offset; i++)
            com_mcpy(dst_y + (-y_offset + i)*s_l_dbk, pic_rec->y + (y - y_offset + i)*s_l_dbk + x, cu_width * sizeof(pel));
    }
    //fill left
    if (x != 0) // note: may not correct in patch
    {
        for (i = 0; i < cu_height; i++)
            com_mcpy(dst_y + i * s_l_dbk - x_offset, pic_rec->y + (y + i)*s_l_dbk + (x - x_offset), x_offset * sizeof(pel));
    }

    //add distortion of current
    ctx->dist_nofilt += enc_ssd_16b(cu_width_log2, cu_height_log2, dst_y, org_y, s_l_dbk, s_l_org, bit_depth);
    //add distortion of top
    if (y != 0)
    {
        ctx->dist_nofilt += enc_ssd_16b(cu_width_log2, log2_y_tm, dst_y - y_tm * s_l_dbk, org_y - y_tm * s_l_org, s_l_dbk, s_l_org, bit_depth);
    }
    //add distortion of left
    if (x != 0)
    {
        ctx->dist_nofilt += enc_ssd_16b(log2_x_tm, cu_height_log2, dst_y - x_tm, org_y - x_tm, s_l_dbk, s_l_org, bit_depth);
    }

    /********************************* filter the pred/rec **************************************/
    if (do_filter)
    {
        int w_scu = cu_width >> MIN_CU_LOG2;
        int h_scu = cu_height >> MIN_CU_LOG2;
        int ind, k;
        //save current best cu info
        intra_flag_save = MCU_GET_INTRA_FLAG(ctx->map.map_scu[t]);
        cu_cbf_save = MCU_GET_CBF(ctx->map.map_scu[t]);
        //set map info of current cu to current mode
        for (j = 0; j < h_scu; j++)
        {
            ind = (y_scu + j) * ctx->info.pic_width_in_scu + x_scu;
            for (i = 0; i < w_scu; i++)
            {
                int pb_idx_y = get_part_idx(mod_info_curr->pb_part, i << 2, j << 2, mod_info_curr->cu_width, mod_info_curr->cu_height);
                k = ind + i;
                if (intra_flag)
                    MCU_SET_INTRA_FLAG(ctx->map.map_scu[k]);
                else
                    MCU_CLEAR_INTRA_FLAG(ctx->map.map_scu[k]);

                if (cu_cbf)
                    MCU_SET_CBF(ctx->map.map_scu[k]);
                else
                    MCU_CLR_CBF(ctx->map.map_scu[k]);

                MCU_SET_TB_PART_LUMA(ctx->map.map_pb_tb_part[k], mod_info_curr->tb_part);
#if SBT
                MCU_SET_SBT_INFO( ctx->map.map_pb_tb_part[k], mod_info_curr->sbt_info );
#endif

                if (refi != NULL && !is_mv_from_mvf)
                {
#if SUB_TMVP
                    if (core->sbTmvp_flag)
                    {

                        int blk = ((i >= w_scu / SBTMVP_NUM_1D) ? 1 : 0) + ((j >= h_scu / SBTMVP_NUM_1D) ? SBTMVP_NUM_1D : 0);
                        assert(core->mvap_flag == 0);
                        ctx->map.map_mv[k][REFP_0][MV_X] = core->sbTmvp[blk].mv[REFP_0][MV_X];
                        ctx->map.map_mv[k][REFP_0][MV_Y] = core->sbTmvp[blk].mv[REFP_0][MV_Y];
                        ctx->map.map_mv[k][REFP_1][MV_X] = core->sbTmvp[blk].mv[REFP_1][MV_X];
                        ctx->map.map_mv[k][REFP_1][MV_Y] = core->sbTmvp[blk].mv[REFP_1][MV_Y];
                        ctx->map.map_refi[k][REFP_0] = core->sbTmvp[blk].ref_idx[REFP_0];
                        ctx->map.map_refi[k][REFP_1] = core->sbTmvp[blk].ref_idx[REFP_1];

                    }
                    else
                    {
#endif
#if MVAP
                        if (core->mvap_flag)
                        {
                            ctx->map.map_mv[k][REFP_0][MV_X] = core->tmp_cu_mvfield[i + j * w_scu].mv[REFP_0][MV_X];
                            ctx->map.map_mv[k][REFP_0][MV_Y] = core->tmp_cu_mvfield[i + j * w_scu].mv[REFP_0][MV_Y];
                            ctx->map.map_mv[k][REFP_1][MV_X] = core->tmp_cu_mvfield[i + j * w_scu].mv[REFP_1][MV_X];
                            ctx->map.map_mv[k][REFP_1][MV_Y] = core->tmp_cu_mvfield[i + j * w_scu].mv[REFP_1][MV_Y];
                            ctx->map.map_refi[k][REFP_0] = core->tmp_cu_mvfield[i + j * w_scu].ref_idx[REFP_0];
                            ctx->map.map_refi[k][REFP_1] = core->tmp_cu_mvfield[i + j * w_scu].ref_idx[REFP_1];
                        }
                        else
                        {
#endif
#if ETMVP
                            if (mod_info_curr->etmvp_flag)
                            {
                                ctx->map.map_mv[k][REFP_0][MV_X] = core->tmp_etmvp_mvfield[i + j * w_scu].mv[REFP_0][MV_X];
                                ctx->map.map_mv[k][REFP_0][MV_Y] = core->tmp_etmvp_mvfield[i + j * w_scu].mv[REFP_0][MV_Y];
                                ctx->map.map_mv[k][REFP_1][MV_X] = core->tmp_etmvp_mvfield[i + j * w_scu].mv[REFP_1][MV_X];
                                ctx->map.map_mv[k][REFP_1][MV_Y] = core->tmp_etmvp_mvfield[i + j * w_scu].mv[REFP_1][MV_Y];
                                ctx->map.map_refi[k][REFP_0] = core->tmp_etmvp_mvfield[i + j * w_scu].ref_idx[REFP_0];
                                ctx->map.map_refi[k][REFP_1] = core->tmp_etmvp_mvfield[i + j * w_scu].ref_idx[REFP_1];
                            }
                            else
                            {
#endif
                                ctx->map.map_refi[k][REFP_0] = refi[REFP_0];
                                ctx->map.map_refi[k][REFP_1] = refi[REFP_1];
                                ctx->map.map_mv[k][REFP_0][MV_X] = mv[REFP_0][MV_X];
                                ctx->map.map_mv[k][REFP_0][MV_Y] = mv[REFP_0][MV_Y];
                                ctx->map.map_mv[k][REFP_1][MV_X] = mv[REFP_1][MV_X];
                                ctx->map.map_mv[k][REFP_1][MV_Y] = mv[REFP_1][MV_Y];
#if !USE_IBC
                                if (!REFI_IS_VALID(refi[REFP_0]))
                                {
                                    assert(mv[REFP_0][MV_X] == 0 && mv[REFP_0][MV_Y] == 0);
                                }
#endif
                                if (!REFI_IS_VALID(refi[REFP_1]))
                                {
                                    assert(mv[REFP_1][MV_X] == 0 && mv[REFP_1][MV_Y] == 0);
                                }
#if ETMVP
                            }
#endif
#if MVAP
                        }
#endif
#if SUB_TMVP
                    }
#endif
                }
                else if (intra_flag)
                {
                    ctx->map.map_refi[k][REFP_0] = -1;
                    ctx->map.map_refi[k][REFP_1] = -1;
                    ctx->map.map_mv[k][REFP_0][MV_X] = 0;
                    ctx->map.map_mv[k][REFP_0][MV_Y] = 0;
                    ctx->map.map_mv[k][REFP_1][MV_X] = 0;
                    ctx->map.map_mv[k][REFP_1][MV_Y] = 0;
                }

                MCU_SET_QP(ctx->map.map_scu[k], ctx->core->qp_y);
                //clear coded (necessary)
                MCU_CLR_CODED_FLAG(ctx->map.map_scu[k]);
            }
        }
        if (ctx->info.pic_header.loop_filter_disable_flag == 0)
        {
            BOOL b_recurse = 0;
            ClearEdgeFilter_avs2(x, y, cu_width, cu_height, ctx->ppbEdgeFilter);
            xSetEdgeFilter_One_SCU_avs2(&ctx->info, &ctx->map, pic_dbk, ctx->ppbEdgeFilter, x, y, cu_width, cu_height, 0, 0, b_recurse);
            DeblockBlk_avs2(&ctx->info, &ctx->map, pic_dbk, ctx->refp, ctx->ppbEdgeFilter, x, y, cu_width, cu_height);
        }

        //recover best cu info
        for (j = 0; j < h_scu; j++)
        {
            ind = (y_scu + j) * ctx->info.pic_width_in_scu + x_scu;
            for (i = 0; i < w_scu; i++)
            {
                k = ind + i;
                if (intra_flag_save)
                    MCU_SET_INTRA_FLAG(ctx->map.map_scu[k]);
                else
                    MCU_CLEAR_INTRA_FLAG(ctx->map.map_scu[k]);

                if (cu_cbf_save)
                    MCU_SET_CBF(ctx->map.map_scu[k]);
                else
                    MCU_CLR_CBF(ctx->map.map_scu[k]);
                MCU_CLR_CODED_FLAG(ctx->map.map_scu[k]);
            }
        }
    }
    /*********************** calc dist of filtered pixels *******************************/
    //add current
    ctx->dist_filter += enc_ssd_16b(cu_width_log2, cu_height_log2, dst_y, org_y, s_l_dbk, s_l_org, bit_depth);
    //add top
    if (y != 0)
    {
        ctx->dist_filter += enc_ssd_16b(cu_width_log2, log2_y_tm, dst_y - y_tm * s_l_dbk, org_y - y_tm * s_l_org, s_l_dbk, s_l_org, bit_depth);
    }
    //add left
    if (x != 0)
    {
        ctx->dist_filter += enc_ssd_16b(log2_x_tm, cu_height_log2, dst_y - x_tm, org_y - x_tm, s_l_dbk, s_l_org, bit_depth);
    }

    /******************************* derive delta dist ********************************/
    ctx->delta_dist = ctx->dist_filter - ctx->dist_nofilt;
}

#endif


double SAORDCost_for_mode_merge(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs_temp, int lcu_pos, int pix_y, int pix_x,
                                double* sao_labmda, SAOStatData **saostatData, SAOBlkParam **rec_saoBlkParam,
                                SAOBlkParam *sao_cur_param, SAOBlkParam *rec_sao_cur_param, int *MergeLeftAvail, int *MergeUpAvail)
{
    COM_SH_EXT *sh = &(ctx->info.shext);
    ENC_SBAC *sao_sbac = GET_SBAC_ENC(sao_bs_temp);
    SAOBlkParam merge_candidate[NUM_SAO_MERGE_TYPES][N_C];
    SAOBlkParam temp_sao_param[N_C];
    int mb_x = pix_x >> MIN_CU_LOG2;
    int mb_y = pix_y >> MIN_CU_LOG2;
    int merge_avail[NUM_SAO_MERGE_TYPES];
    int mergeIdx, compIdx;
    int type, mode;
    double curdist = 0;
    int currate;
    double curcost, mincost;
    mincost = MAX_COST;
    getSaoMergeNeighbor(&ctx->info, ctx->map.map_patch_idx, ctx->info.pic_width_in_scu, ctx->info.pic_width_in_lcu, lcu_pos, mb_y, mb_x,
                        rec_saoBlkParam, merge_avail, merge_candidate);
    *MergeLeftAvail = merge_avail[SAO_MERGE_LEFT];
    *MergeUpAvail = merge_avail[SAO_MERGE_ABOVE];
    SBAC_STORE((core->s_sao_cur_mergetype), (*sao_sbac));
    for (mergeIdx = 0; mergeIdx < NUM_SAO_MERGE_TYPES; mergeIdx++)
    {
        if (merge_avail[mergeIdx])
        {
            for (compIdx = Y_C; compIdx < N_C; compIdx++)
            {
                if (!sh->slice_sao_enable[compIdx])
                {
                    merge_candidate[mergeIdx][compIdx].modeIdc = SAO_MODE_OFF;
                }
            }

            curdist = 0;
            copySAOParam_for_blk(temp_sao_param, merge_candidate[mergeIdx]);
            for (compIdx = Y_C; compIdx < N_C; compIdx++)
            {
                type = merge_candidate[mergeIdx][compIdx].typeIdc;
                mode = merge_candidate[mergeIdx][compIdx].modeIdc;
                temp_sao_param[compIdx].typeIdc = mergeIdx;
                temp_sao_param[compIdx].modeIdc = SAO_MODE_MERGE;
                if (mode != SAO_MODE_OFF)
                {
                    curdist += (get_distortion(compIdx, type, saostatData, temp_sao_param)) / sao_labmda[compIdx];
                }
            }
            //enc_sbac_bit_reset(sao_sbac);
            currate = enc_get_bit_number(sao_sbac);
            enc_eco_sao_mrg_flag(sao_sbac, sao_bs_temp, *MergeLeftAvail, *MergeUpAvail, temp_sao_param);
            currate = enc_get_bit_number(sao_sbac) - currate;
            curcost = currate + curdist;
            if (curcost < mincost)
            {
                mincost = curcost;
                copySAOParam_for_blk(sao_cur_param, temp_sao_param);
                copySAOParam_for_blk(rec_sao_cur_param, merge_candidate[mergeIdx]);
                SBAC_STORE((core->s_sao_next_mergetype), (*sao_sbac));
            }
        }
        SBAC_LOAD((*sao_sbac), (core->s_sao_cur_mergetype));
    }
    SBAC_LOAD((*sao_sbac), (core->s_sao_next_mergetype));
    return mincost;
}

double SAORDCost_for_mode_new_YUV_sep(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs_temp, int MergeLeftAvail, int MergeUpAvail, double* sao_lambda, SAOStatData **saostatData, SAOBlkParam *sao_cur_param, SAOBlkParam *rec_sao_cur_param)
{
    COM_SH_EXT *sh = &(ctx->info.shext);
    ENC_SBAC *sao_sbac = GET_SBAC_ENC(sao_bs_temp);
    int compIdx;
    int type;
    long long int mindist[N_C], curdist[N_C];
    int  minrate[N_C], currate[N_C];
    double curcost, mincost;
    double normmodecost;
    SAOBlkParam temp_sao_param[N_C];
    int mergeflag_rate;
    mergeflag_rate = 0;
    sao_cur_param[Y_C].modeIdc = SAO_MODE_OFF;
    rec_sao_cur_param[Y_C].modeIdc = SAO_MODE_OFF;

    //////////////////////////////////////////coding MERGE FLAG///////////////////////////////////////////////////////////
    if (MergeLeftAvail + MergeUpAvail)
    {
        //enc_sbac_bit_reset(sao_sbac);
        mergeflag_rate = enc_get_bit_number(sao_sbac);
        enc_eco_sao_mrg_flag(sao_sbac, sao_bs_temp, MergeLeftAvail, MergeUpAvail, &(sao_cur_param[Y_C]));
        mergeflag_rate = enc_get_bit_number(sao_sbac) - mergeflag_rate;
    }
    ///////////////////////////////////////////////SAO NEW MODE///////////////////////////////////////////////////////////
    for (compIdx = Y_C; compIdx < N_C; compIdx++)
    {
        SBAC_STORE((core->s_sao_cur_type), (*sao_sbac));
        /////SAO OFF MODE////
        //enc_sbac_bit_reset(sao_sbac);
        minrate[compIdx] = enc_get_bit_number(sao_sbac);
        sao_cur_param[compIdx].modeIdc = SAO_MODE_OFF;
        rec_sao_cur_param[compIdx].modeIdc = SAO_MODE_OFF;
        enc_eco_sao_mode(sao_sbac, sao_bs_temp, &(sao_cur_param[compIdx]));
        minrate[compIdx] = enc_get_bit_number(sao_sbac) - minrate[compIdx];
        mindist[compIdx] = 0;
        mincost = RATE_TO_COST_LAMBDA(sao_lambda[compIdx], minrate[compIdx]);
        SBAC_STORE((core->s_sao_next_type), (*sao_sbac));
        SBAC_LOAD((*sao_sbac), (core->s_sao_cur_type));
        /////SAO NEW MODE/////
        if (sh->slice_sao_enable[compIdx])
        {
            for (type = 0; type < NUM_SAO_NEW_TYPES; type++)
            {
                temp_sao_param[compIdx].modeIdc = SAO_MODE_NEW;
                temp_sao_param[compIdx].typeIdc = type;
                find_offset(compIdx, type, saostatData, temp_sao_param, sao_lambda[compIdx]);
                curdist[compIdx] = get_distortion(compIdx, type, saostatData, temp_sao_param);
                // assert(curdist[compIdx]<=0);
                //enc_sbac_bit_reset(sao_sbac);
                currate[compIdx] = enc_get_bit_number(sao_sbac);
                enc_eco_sao_mode(sao_sbac, sao_bs_temp, &(temp_sao_param[compIdx]));
                enc_eco_sao_offset(sao_sbac, sao_bs_temp, &(temp_sao_param[compIdx]));
                enc_eco_sao_type(sao_sbac, sao_bs_temp, &(temp_sao_param[compIdx]));
                currate[compIdx] = enc_get_bit_number(sao_sbac) - currate[compIdx];
                curcost = (double)(curdist[compIdx]) + RATE_TO_COST_LAMBDA(sao_lambda[compIdx], currate[compIdx]);
                if (curcost < mincost)
                {
                    mincost = curcost;
                    minrate[compIdx] = currate[compIdx];
                    mindist[compIdx] = curdist[compIdx];
                    copySAOParam_for_blk_onecomponent(&sao_cur_param[compIdx], &temp_sao_param[compIdx]);
                    copySAOParam_for_blk_onecomponent(&rec_sao_cur_param[compIdx], &temp_sao_param[compIdx]);
                    SBAC_STORE((core->s_sao_next_type), (*sao_sbac));
                }
                SBAC_LOAD((*sao_sbac), (core->s_sao_cur_type));
            }
            SBAC_LOAD((*sao_sbac), (core->s_sao_next_type));
        }
        else
        {
            mindist[compIdx] = 0;
            minrate[compIdx] = 0;
        }
    }
    normmodecost = (double)(mindist[Y_C] + mindist[U_C] + mindist[V_C]) / sao_lambda[Y_C];
    normmodecost += minrate[Y_C] + minrate[U_C] + minrate[V_C] + mergeflag_rate;
    return normmodecost;
}

void getParaSAO_one_LCU(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs_temp, int lcu_pos, int mb_y, int mb_x,
                        SAOStatData **saostatData, SAOBlkParam *saoBlkParam, SAOBlkParam **rec_saoBlkParam, double* sao_labmda)
{
    COM_SH_EXT *sh = &(ctx->info.shext);
    ENC_SBAC *sao_sbac = GET_SBAC_ENC(sao_bs_temp);
    int pix_x = mb_x << MIN_CU_LOG2;
    int pix_y = mb_y << MIN_CU_LOG2;

    int MergeLeftAvail = 0;
    int MergeUpAvail = 0;
    double mcost, mincost;
    int mode;
    SAOBlkParam sao_cur_param[N_C];
    SAOBlkParam rec_sao_cur_param[N_C];
    if (!sh->slice_sao_enable[Y_C] && !sh->slice_sao_enable[U_C] && !sh->slice_sao_enable[V_C])
    {
        off_sao(saoBlkParam);
        off_sao(rec_saoBlkParam[lcu_pos]);
        return;
    }
    SBAC_STORE((core->s_sao_cur_blk), (*sao_sbac));
    mincost = MAX_COST;
    for (mode = 0; mode < NUM_SAO_MODES; mode++)
    {
        switch (mode)
        {
        case SAO_MODE_OFF:
        {
            continue;
        }
        break;
        case SAO_MODE_MERGE:
        {
            mcost = SAORDCost_for_mode_merge(ctx, core, sao_bs_temp, lcu_pos, pix_y, pix_x, sao_labmda, saostatData, rec_saoBlkParam, sao_cur_param, rec_sao_cur_param, &MergeLeftAvail, &MergeUpAvail);
        }
        break;
        case SAO_MODE_NEW:
        {
            mcost = SAORDCost_for_mode_new_YUV_sep(ctx, core, sao_bs_temp, MergeLeftAvail, MergeUpAvail, sao_labmda, saostatData, sao_cur_param, rec_sao_cur_param);
        }
        break;
        default:
        {
            printf("Not a supported SAO mode\n");
            assert(0);
            exit(-1);
        }
        }
        if (mcost < mincost)
        {
            mincost = mcost;
            copySAOParam_for_blk(saoBlkParam, sao_cur_param);
            copySAOParam_for_blk(rec_saoBlkParam[lcu_pos], rec_sao_cur_param);
            SBAC_STORE((core->s_sao_next_blk), (*sao_sbac));
        }
        SBAC_LOAD((*sao_sbac), (core->s_sao_cur_blk));
    }
    SBAC_LOAD((*sao_sbac), (core->s_sao_cur_blk));

}



void getStatblk(COM_PIC  *pic_org, COM_PIC  *pic_sao, SAOStatData *saostatsData, int bit_depth, int compIdx, int pix_y, int pix_x, int lcu_pix_height,
                int lcu_pix_width, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
                int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int type;
    int start_x, end_x, start_y, end_y;
    int start_x_r0, end_x_r0, start_x_r, end_x_r, start_x_rn, end_x_rn;
    int x, y;
    pel *Rec, *Org;
    char leftsign, rightsign, upsign, downsign;
    long diff;
    SAOStatData *statsDate;
    char *signupline, *signupline1;
    int reg = 0;
    int edgetype, bandtype;

    Rec = Org = NULL;
    int SrcStride, OrgStride;

    if (lcu_pix_height <= 0 || lcu_pix_width <= 0)
    {
        return;
    }

    switch (compIdx)
    {
    case Y_C:
        SrcStride = pic_sao->stride_luma;
        OrgStride = pic_org->stride_luma;
        Rec = pic_sao->y;
        Org = pic_org->y;
        break;
    case U_C:
        SrcStride = pic_sao->stride_chroma;
        OrgStride = pic_org->stride_chroma;
        Rec = pic_sao->u;
        Org = pic_org->u;
        break;
    case V_C:
        SrcStride = pic_sao->stride_chroma;
        OrgStride = pic_org->stride_chroma;
        Rec = pic_sao->v;
        Org = pic_org->v;
        break;
    default:
        SrcStride = 0;
        OrgStride = 0;
        Rec = NULL;
        Org = NULL;
        assert(0);
    }
    signupline = (char *)malloc((lcu_pix_width + 1) * sizeof(char));
    for (type = 0; type < NUM_SAO_NEW_TYPES; type++)
    {
        statsDate = &(saostatsData[type]);
        switch (type)
        {
        case SAO_TYPE_EO_0:
        {
            start_y = 0;
            end_y = lcu_pix_height;
            start_x = lcu_available_left ? 0 : 1;
            end_x = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
            for (y = start_y; y < end_y; y++)
            {
                diff = Rec[(pix_y + y) * SrcStride + pix_x + start_x] - Rec[(pix_y + y) * SrcStride + pix_x + start_x - 1];
                leftsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                for (x = start_x; x < end_x; x++)
                {
                    diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x + 1];
                    rightsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    edgetype = leftsign + rightsign;
                    leftsign = -rightsign;
                    statsDate->diff[edgetype + 2] += (Org[(pix_y + y) * OrgStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x]);
                    statsDate->count[edgetype + 2]++;
                }
            }
        }
        break;
        case SAO_TYPE_EO_90:
        {
            start_x = 0;
            end_x = lcu_pix_width;
            start_y = lcu_available_up ? 0 : 1;
            end_y = lcu_available_down ? lcu_pix_height : (lcu_pix_height - 1);
            for (x = start_x; x < end_x; x++)
            {
                diff = Rec[(pix_y + start_y) * SrcStride + pix_x + x] - Rec[(pix_y + start_y - 1) * SrcStride + pix_x + x];
                upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                for (y = start_y; y < end_y; y++)
                {
                    diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y + 1) * SrcStride + pix_x + x];
                    downsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    edgetype = downsign + upsign;
                    upsign = -downsign;
                    statsDate->diff[edgetype + 2] += (Org[(pix_y + y) * OrgStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x]);
                    statsDate->count[edgetype + 2]++;
                }
            }
        }
        break;
        case SAO_TYPE_EO_135:
        {
            start_x_r0 = lcu_available_upleft ? 0 : 1;
            end_x_r0 = lcu_available_up ? (lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1)) : 1;
            start_x_r = lcu_available_left ? 0 : 1;
            end_x_r = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
            start_x_rn = lcu_available_down ? (lcu_available_left ? 0 : 1) : (lcu_pix_width - 1);
            end_x_rn = lcu_available_rightdwon ? lcu_pix_width : (lcu_pix_width - 1);
            for (x = start_x_r + 1; x < end_x_r + 1; x++)
            {
                diff = Rec[(pix_y + 1) * SrcStride + pix_x + x] - Rec[pix_y * SrcStride + pix_x + x - 1];
                upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                signupline[x] = upsign;
            }
            //first row
            for (x = start_x_r0; x < end_x_r0; x++)
            {
                diff = Rec[pix_y * SrcStride + pix_x + x] - Rec[(pix_y - 1) * SrcStride + pix_x + x - 1];
                upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edgetype = upsign - signupline[x + 1];
                statsDate->diff[edgetype + 2] += (Org[pix_y * OrgStride + pix_x + x] - Rec[pix_y * SrcStride + pix_x + x]);
                statsDate->count[edgetype + 2]++;
            }
            //middle rows
            for (y = 1; y < lcu_pix_height - 1; y++)
            {
                for (x = start_x_r; x < end_x_r; x++)
                {
                    if (x == start_x_r)
                    {
                        diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y - 1) * SrcStride + pix_x + x - 1];
                        upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                        signupline[x] = upsign;
                    }
                    diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y + 1) * SrcStride + pix_x + x + 1];
                    downsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    edgetype = downsign + signupline[x];
                    statsDate->diff[edgetype + 2] += (Org[(pix_y + y) * OrgStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x]);
                    statsDate->count[edgetype + 2]++;
                    signupline[x] = (char)reg;
                    reg = -downsign;
                }
            }
            //last row
            for (x = start_x_rn; x < end_x_rn; x++)
            {
                if (x == start_x_r)
                {
                    diff = Rec[(pix_y + lcu_pix_height - 1) * SrcStride + pix_x + x] - Rec[(pix_y + lcu_pix_height - 2) * SrcStride + pix_x + x - 1];
                    upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    signupline[x] = upsign;
                }
                diff = Rec[(pix_y + lcu_pix_height - 1) * SrcStride + pix_x + x] - Rec[(pix_y + lcu_pix_height) * SrcStride + pix_x + x + 1];
                downsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edgetype = downsign + signupline[x];
                statsDate->diff[edgetype + 2] += (Org[(pix_y + lcu_pix_height - 1) * OrgStride + pix_x + x] - Rec[(pix_y + lcu_pix_height - 1) * SrcStride + pix_x + x]);
                statsDate->count[edgetype + 2]++;
            }
        }
        break;
        case SAO_TYPE_EO_45:
        {
            start_x_r0 = lcu_available_up ? (lcu_available_left ? 0 : 1) : (lcu_pix_width - 1);
            end_x_r0 = lcu_available_upright ? lcu_pix_width : (lcu_pix_width - 1);
            start_x_r = lcu_available_left ? 0 : 1;
            end_x_r = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
            start_x_rn = lcu_available_leftdown ? 0 : 1;
            end_x_rn = lcu_available_down ? (lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1)) : 1;
            signupline1 = signupline + 1;
            for (x = start_x_r - 1; x < max(end_x_r - 1, end_x_r0 - 1); x++)
            {
                diff = Rec[(pix_y + 1) * SrcStride + pix_x + x] - Rec[pix_y * SrcStride + pix_x + x + 1];
                upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                signupline1[x] = upsign;
            }
            //first row
            for (x = start_x_r0; x < end_x_r0; x++)
            {
                diff = Rec[pix_y * SrcStride + pix_x + x] - Rec[(pix_y - 1) * SrcStride + pix_x + x + 1];
                upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edgetype = upsign - signupline1[x - 1];
                statsDate->diff[edgetype + 2] += (Org[pix_y * OrgStride + pix_x + x] - Rec[pix_y * SrcStride + pix_x + x]);
                statsDate->count[edgetype + 2]++;
            }
            //middle rows
            for (y = 1; y < lcu_pix_height - 1; y++)
            {
                for (x = start_x_r; x < end_x_r; x++)
                {
                    if (x == end_x_r - 1)
                    {
                        diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y - 1) * SrcStride + pix_x + x + 1];
                        upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                        signupline1[x] = upsign;
                    }
                    diff = Rec[(pix_y + y) * SrcStride + pix_x + x] - Rec[(pix_y + y + 1) * SrcStride + pix_x + x - 1];
                    downsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    edgetype = downsign + signupline1[x];
                    statsDate->diff[edgetype + 2] += (Org[(pix_y + y) * OrgStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x]);
                    statsDate->count[edgetype + 2]++;
                    signupline1[x - 1] = -downsign;
                }
            }
            for (x = start_x_rn; x < end_x_rn; x++)
            {
                if (x == end_x_r - 1)
                {
                    diff = Rec[(pix_y + lcu_pix_height - 1) * SrcStride + pix_x + x] - Rec[(pix_y + lcu_pix_height - 2) * SrcStride + pix_x
                            + x + 1];
                    upsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    signupline1[x] = upsign;
                }
                diff = Rec[(pix_y + lcu_pix_height - 1) * SrcStride + pix_x + x] - Rec[(pix_y + lcu_pix_height) * SrcStride + pix_x + x
                        - 1];
                downsign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edgetype = downsign + signupline1[x];
                statsDate->diff[edgetype + 2] += (Org[(pix_y + lcu_pix_height - 1) * OrgStride + pix_x + x] - Rec[(pix_y +
                                                  lcu_pix_height - 1) * SrcStride + pix_x +
                                                  x]);
                statsDate->count[edgetype + 2]++;
            }
        }
        break;
        case SAO_TYPE_BO:
        {
            start_x = 0;
            end_x = lcu_pix_width;
            start_y = 0;
            end_y = lcu_pix_height;
            for (x = start_x; x < end_x; x++)
            {
                for (y = start_y; y < end_y; y++)
                {
                    bandtype = Rec[(pix_y + y) * SrcStride + pix_x + x] >> (bit_depth - NUM_SAO_BO_CLASSES_IN_BIT);
                    statsDate->diff[bandtype] += (Org[(pix_y + y) * OrgStride + pix_x + x] - Rec[(pix_y + y) * SrcStride + pix_x + x]);
                    statsDate->count[bandtype]++;
                }
            }
        }
        break;
        default:
        {
            printf("Not a supported SAO types\n");
            assert(0);
            exit(-1);
        }
        }
    }
    free(signupline);
}

void enc_SAO_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs)
{
    COM_INFO *info = &ctx->info;
    COM_MAP *map = &ctx->map;
    int bit_depth = ctx->info.bit_depth_internal;

    int lcu_x = core->x_pel;
    int lcu_y = core->y_pel;
    int mb_x = PEL2SCU(lcu_x);
    int mb_y = PEL2SCU(lcu_y);
    int lcuw = min(1 << info->log2_max_cuwh, info->pic_width - lcu_x);
    int lcuh = min(1 << info->log2_max_cuwh, info->pic_height - lcu_y);
    int lcu_mb_width = lcuw >> MIN_CU_LOG2;
    int lcu_mb_height = lcuh >> MIN_CU_LOG2;
    int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
    Copy_LCU_for_SAO(ctx->pic_sao, PIC_REC(ctx), lcu_y, lcu_x, lcuh, lcuw);

    if (ctx->info.pic_header.loop_filter_disable_flag == 0)
    {
        //deblock one LCU for SAO
        BOOL b_recurse = 1;
        ClearEdgeFilter_avs2(lcu_x, lcu_y, lcuw, lcuh, ctx->ppbEdgeFilter);
        xSetEdgeFilter_One_SCU_avs2(info, map, ctx->pic_sao, ctx->ppbEdgeFilter, lcu_x, lcu_y, info->max_cuwh, info->max_cuwh, 0, 0, b_recurse);
        DeblockBlk_avs2(info, map, ctx->pic_sao, ctx->refp, ctx->ppbEdgeFilter, lcu_x, lcu_y, lcuw, lcuh);
    }

    getStatisticsSAO_one_LCU(info, map, PIC_ORG(ctx), ctx->pic_sao, mb_y, mb_x, lcu_mb_height, lcu_mb_width,
                             ctx->saostatData[lcu_pos]);
    double sao_lambda[N_C];
    int scale_lambda = (bit_depth == 10) ? ctx->info.qp_offset_bit_depth : 1;
    for (int compIdx = Y_C; compIdx < N_C; compIdx++)
    {
        sao_lambda[compIdx] = ctx->lambda[0] * scale_lambda;
    }
    getParaSAO_one_LCU(ctx, core, sao_bs, lcu_pos, mb_y, mb_x,
                       ctx->saostatData[lcu_pos], ctx->saoBlkParams[lcu_pos],
                       ctx->rec_saoBlkParams, sao_lambda);

}

void getStatisticsSAO_one_LCU(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_sao, int mb_y, int mb_x, int lcu_mb_height, int lcu_mb_width,
                              SAOStatData **saostatData)
{
    COM_SH_EXT *sh = &(info->shext);
    int bit_depth = info->bit_depth_internal;
    int pix_x = mb_x << MIN_CU_LOG2;
    int pix_y = mb_y << MIN_CU_LOG2;
    int lcu_pix_width = lcu_mb_width << MIN_CU_LOG2;
    int lcu_pix_height = lcu_mb_height << MIN_CU_LOG2;
    int compIdx;
    int lcu_pix_height_t, lcu_pix_width_t, pix_x_t, pix_y_t;
    SAOStatData *statsDate;
    int isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
        isBelowRightAvail;
    int type;
    int  isLeftProc, isRightProc, isAboveProc, isBelowProc, isAboveLeftProc, isAboveRightProc, isBelowLeftProc,
         isBelowRightProc;
    checkBoundaryPara(info, map, mb_y, mb_x, lcu_pix_height, lcu_pix_width, &isLeftAvail, &isRightAvail,
                      &isAboveAvail, &isBelowAvail, &isAboveLeftAvail, &isAboveRightAvail, &isBelowLeftAvail, &isBelowRightAvail, 0);
    for (compIdx = Y_C; compIdx < N_C; compIdx++)
    {
        if (!sh->slice_sao_enable[compIdx])
        {
            continue;
        }

        for (type = 0; type < NUM_SAO_NEW_TYPES; type++)
        {
            statsDate = &(saostatData[compIdx][type]);
            init_StateDate(statsDate);
        }
        statsDate = saostatData[compIdx];
        lcu_pix_width_t = compIdx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
        lcu_pix_height_t = compIdx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
        pix_x_t = compIdx ? (pix_x >> 1) : pix_x;
        pix_y_t = compIdx ? (pix_y >> 1) : pix_y;
        checkBoundaryProc(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                          &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
        getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                    isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                    isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, isBelowRightProc/*BelowRight*/);
        //
        if (isAboveLeftAvail)
        {
            lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
            lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
            pix_x_t = compIdx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
            pix_y_t = compIdx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
            assert(isAboveAvail && isLeftAvail);
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                        isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, isBelowRightProc/*BelowRight*/);
        }
        if (isLeftAvail)
        {
            lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
            lcu_pix_height_t = compIdx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
            pix_x_t = compIdx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
            pix_y_t = compIdx ? (pix_y >> 1) : pix_y;
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                        isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, isBelowRightProc/*BelowRight*/);
        }
        if (isAboveAvail)
        {
            lcu_pix_width_t = compIdx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
            lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
            pix_x_t = compIdx ? (pix_x >> 1) : pix_x;
            pix_y_t = compIdx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                        isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, isBelowRightProc/*BelowRight*/);
        }
        if (!isRightAvail)
        {
            if (isAboveAvail)
            {
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = compIdx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                          SAO_SHIFT_PIX_NUM);
                pix_y_t = compIdx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
                checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                                  &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
                getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                            isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                            isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, 0/*BelowRight is forced to 0*/);
            }
            lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
            lcu_pix_height_t = compIdx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
            pix_x_t = compIdx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                      SAO_SHIFT_PIX_NUM);
            pix_y_t = compIdx ? (pix_y >> 1) : pix_y;
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        0/*Right is forced to 0*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                        isAboveRightProc/*AboveRight*/, isBelowLeftProc/*BelowLeft*/, 0/*BelowRight is forced to 0*/);
        }
        if (!isBelowAvail)
        {
            if (isLeftAvail)
            {
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = compIdx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
                pix_y_t = compIdx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                          SAO_SHIFT_PIX_NUM);
                checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                                  &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
                getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                            isRightProc/*Right*/, isAboveProc/*Above*/, 0/*Below is forced to 0*/, isAboveLeftProc/*AboveLeft*/,
                            isAboveRightProc/*AboveRight*/, 0/*BelowLeft is forced to 0*/, 0/*BelowRight is forced to 0*/);
            }
            lcu_pix_width_t = compIdx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
            lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
            pix_x_t = compIdx ? (pix_x >> 1) : pix_x;
            pix_y_t = compIdx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                      SAO_SHIFT_PIX_NUM);
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 1);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        isRightProc/*Right*/, isAboveProc/*Above*/, 0/*Below is forced to 0*/, isAboveLeftProc/*AboveLeft*/,
                        isAboveRightProc/*AboveRight*/, 0/*BelowLeft is forced to 0*/, 0/*BelowRight is forced to 0*/);
        }
        if (!isBelowRightAvail && !isRightAvail && !isBelowAvail)
        {
            lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
            lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
            pix_x_t = compIdx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                      SAO_SHIFT_PIX_NUM);
            pix_y_t = compIdx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                      SAO_SHIFT_PIX_NUM);
            checkBoundaryProc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, compIdx, &isLeftProc,
                              &isRightProc, &isAboveProc, &isBelowProc, &isAboveLeftProc, &isAboveRightProc, &isBelowLeftProc, &isBelowRightProc, 0);
            getStatblk( pic_org, pic_sao, statsDate, bit_depth, compIdx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, isLeftProc /*Left*/,
                        isRightProc/*Right*/, isAboveProc/*Above*/, isBelowProc/*Below*/, isAboveLeftProc/*AboveLeft*/,
                        0/*AboveRight is forced to 0*/, isBelowLeftProc/*BelowLeft*/, isBelowRightProc/*BelowRight*/);
        }
    }
}

void find_offset(int compIdx, int typeIdc, SAOStatData **saostatData, SAOBlkParam *saoBlkParam, double lambda)
{
    int class_i;
    double classcost[MAX_NUM_SAO_CLASSES];
    double offth;
    int num_class = (typeIdc == SAO_TYPE_BO) ? NUM_SAO_BO_CLASSES : NUM_SAO_EO_CLASSES;
    double mincost_bandsum, cost_bandsum;
    int start_band1, start_band2, delta_band12;
    int sb_temp[2];
    int db_temp;
    for (class_i = 0; class_i < num_class; class_i++)
    {
        //EO - PLAIN
        if ((typeIdc != SAO_TYPE_BO) && (class_i == SAO_CLASS_EO_PLAIN))
        {
            saoBlkParam[compIdx].offset[class_i] = 0;
            continue;
        }
        //count == 0
        if (saostatData[compIdx][typeIdc].count[class_i] == 0)
        {
            saoBlkParam[compIdx].offset[class_i] = 0; //offset will be zero
            continue;
        }
        offth = saostatData[compIdx][typeIdc].diff[class_i] > 0 ? 0.5 : (saostatData[compIdx][typeIdc].diff[class_i] < 0 ?
                -0.5 : 0);
        saoBlkParam[compIdx].offset[class_i] = (int)((double)saostatData[compIdx][typeIdc].diff[class_i] /
                                               (double)saostatData[compIdx][typeIdc].count[class_i] + offth);
    }
    if (typeIdc == SAO_TYPE_BO)
    {
        for (class_i = 0; class_i < num_class; class_i++)
        {
            saoBlkParam[compIdx].offset[class_i] = offset_estimation(typeIdc, class_i, lambda, saoBlkParam[compIdx].offset[class_i],
                                                   saostatData[compIdx][typeIdc].count[class_i], saostatData[compIdx][typeIdc].diff[class_i], &(classcost[class_i]));
        }
        mincost_bandsum = 1.7e+308;
        for (start_band1 = 0; start_band1 < (NUM_SAO_BO_CLASSES - 1); start_band1++)
        {
            for (start_band2 = start_band1 + 2; start_band2 < (NUM_SAO_BO_CLASSES - 1); start_band2++)
            {
                cost_bandsum = classcost[start_band1] + classcost[start_band1 + 1] + classcost[start_band2] + classcost[start_band2 +
                               1];
                delta_band12 = (start_band2 - start_band1) > (NUM_SAO_BO_CLASSES >> 1) ? (32 - start_band2 + start_band1) :
                               (start_band2 - start_band1);
                assert(delta_band12 >= 0 && delta_band12 <= (NUM_SAO_BO_CLASSES >> 1));
                cost_bandsum += lambda * (double)deltaband_cost[delta_band12];
                if (cost_bandsum < mincost_bandsum)
                {
                    mincost_bandsum = cost_bandsum;
                    saoBlkParam[compIdx].startBand = start_band1;
                    saoBlkParam[compIdx].startBand2 = start_band2;
                }
            }
        }
        for (class_i = 0; class_i < num_class; class_i++)
        {
            if ((class_i >= saoBlkParam[compIdx].startBand && class_i <= saoBlkParam[compIdx].startBand + 1) ||
                    (class_i >= saoBlkParam[compIdx].startBand2 && class_i <= saoBlkParam[compIdx].startBand2 + 1))
            {
                continue;
            }
            saoBlkParam[compIdx].offset[class_i] = 0;
        }
        sb_temp[0] = min(saoBlkParam[compIdx].startBand, saoBlkParam[compIdx].startBand2);
        sb_temp[1] = max(saoBlkParam[compIdx].startBand, saoBlkParam[compIdx].startBand2);
        db_temp = (sb_temp[1] - sb_temp[0]);
        if (db_temp > (NUM_SAO_BO_CLASSES >> 1))
        {
            saoBlkParam[compIdx].deltaband = 32 - db_temp;
            saoBlkParam[compIdx].startBand = sb_temp[1];
            saoBlkParam[compIdx].startBand2 = sb_temp[0];
        }
        else
        {
            saoBlkParam[compIdx].deltaband = db_temp;
            saoBlkParam[compIdx].startBand = sb_temp[0];
            saoBlkParam[compIdx].startBand2 = sb_temp[1];
        }
    }
    else
    {
        assert(typeIdc >= SAO_TYPE_EO_0 && typeIdc <= SAO_TYPE_EO_45);
        // LOOP EO 5 types (find best offset for each type)
        for (class_i = 0; class_i < num_class; class_i++)
        {
            if (class_i == SAO_CLASS_EO_PLAIN)
            {
                saoBlkParam[compIdx].offset[class_i] = 0;
                classcost[class_i] = 0;
            }
            else
            {
                saoBlkParam[compIdx].offset[class_i] = offset_estimation(typeIdc, class_i, lambda, saoBlkParam[compIdx].offset[class_i],
                                                       saostatData[compIdx][typeIdc].count[class_i], saostatData[compIdx][typeIdc].diff[class_i], &(classcost[class_i]));
            }
        }
        saoBlkParam[compIdx].startBand = 0;
    }
}
int offset_estimation(int typeIdx, int classIdx, double lambda, int offset_ori, int count, long long int diff,
                      double *bestCost)
{
    int cur_offset = offset_ori;
    int offset_best = 0;
    int lower_bd, upper_bd, Th;
    int temp_offset, start_offset, end_offset;
    int temprate;
    long long int tempdist;
    double tempcost, mincost;
    int *eo_offset_bins = &(EO_OFFSET_MAP[1]);
    int offset_type;
    int offset_step;
    if (typeIdx == SAO_TYPE_BO)
    {
        offset_type = SAO_CLASS_BO;
    }
    else
    {
        offset_type = classIdx;
    }
    lower_bd = saoclip[offset_type][0];
    upper_bd = saoclip[offset_type][1];
    Th = saoclip[offset_type][2];
    offset_step = 1;

    cur_offset = COM_CLIP3(lower_bd, upper_bd, cur_offset);


    if (typeIdx == SAO_TYPE_BO)
    {
        start_offset = cur_offset >= 0 ? 0 : cur_offset;
        end_offset = cur_offset >= 0 ? cur_offset : 0;
    }
    else
    {
        assert(typeIdx >= SAO_TYPE_EO_0 && typeIdx <= SAO_TYPE_EO_45);
        switch (classIdx)
        {
        case SAO_CLASS_EO_FULL_VALLEY:
            start_offset = -1;
            end_offset = max(cur_offset, 1);
            break;
        case SAO_CLASS_EO_HALF_VALLEY:
            start_offset = 0;
            end_offset = 1;
            break;
        case SAO_CLASS_EO_HALF_PEAK:
            start_offset = -1;
            end_offset = 0;
            break;
        case SAO_CLASS_EO_FULL_PEAK:
            start_offset = min(cur_offset, -1);
            end_offset = 1;
            break;
        default:
            printf("Not a supported SAO mode\n");
            assert(0);
            exit(-1);
        }
    }
    mincost = MAX_COST;
    for (temp_offset = start_offset; temp_offset <= end_offset; temp_offset += offset_step)
    {
        int offset = temp_offset;
        assert(offset >= -7 && offset <= 7);
        if (typeIdx == SAO_TYPE_BO)
        {
            assert(offset_type == SAO_CLASS_BO);
            temprate = abs(offset);
            temprate = temprate ? (temprate + 1) : 0;
        }
        else if (classIdx == SAO_CLASS_EO_HALF_VALLEY || classIdx == SAO_CLASS_EO_HALF_PEAK)
        {
            temprate = abs(offset);
        }
        else
        {
            temprate = eo_offset_bins[classIdx == SAO_CLASS_EO_FULL_VALLEY ? offset : -offset];
        }
        temprate = (temprate == Th) ? temprate : (temprate + 1);

        tempdist = distortion_cal(count, temp_offset, diff);
        tempcost = (double)tempdist + lambda * (double)temprate;
        if (tempcost < mincost)
        {
            mincost = tempcost;
            offset_best = temp_offset;
            *bestCost = tempcost;
        }
    }
    return offset_best;
}
void writeParaSAO_one_LCU(ENC_CTX *ctx, int y_pel, int x_pel,
                          SAOBlkParam *saoBlkParam)
{
    COM_SH_EXT  *sh = &(ctx->info.shext);
    int lcuw = 1 << ctx->info.log2_max_cuwh;
    int lcuh = 1 << ctx->info.log2_max_cuwh;

    int x_in_lcu = x_pel >> ctx->info.log2_max_cuwh;
    int y_in_lcu = y_pel >> ctx->info.log2_max_cuwh;
    int lcu_pos = x_in_lcu + y_in_lcu * ctx->info.pic_width_in_lcu;
    if (!sh->slice_sao_enable[Y_C] && !sh->slice_sao_enable[U_C] && !sh->slice_sao_enable[V_C])
    {
        return;
    }
    write_sao_lcu(ctx, lcu_pos, y_pel, x_pel, saoBlkParam);
}

void write_sao_lcu(ENC_CTX *ctx, int lcu_pos, int pix_y, int pix_x, SAOBlkParam *sao_cur_param)
{
    COM_SH_EXT  *sh = &(ctx->info.shext);
    int mb_x = pix_x >> MIN_CU_LOG2;
    int mb_y = pix_y >> MIN_CU_LOG2;
    SAOBlkParam merge_candidate[NUM_SAO_MERGE_TYPES][N_C];
    int merge_avail[NUM_SAO_MERGE_TYPES];
    int MergeLeftAvail, MergeUpAvail;

    getSaoMergeNeighbor(&ctx->info, ctx->map.map_patch_idx, ctx->info.pic_width_in_scu, ctx->info.pic_width_in_lcu, lcu_pos, mb_y, mb_x, ctx->saoBlkParams, merge_avail, merge_candidate);
    MergeLeftAvail = merge_avail[SAO_MERGE_LEFT];
    MergeUpAvail = merge_avail[SAO_MERGE_ABOVE];
    COM_BSW *bs = &ctx->bs;
    ENC_SBAC * sbac = GET_SBAC_ENC(bs);
    if (MergeLeftAvail + MergeUpAvail)
    {
        enc_eco_sao_mrg_flag(sbac, bs, MergeLeftAvail, MergeUpAvail, &(sao_cur_param[Y_C]));
    }
    if (sao_cur_param[Y_C].modeIdc != SAO_MODE_MERGE)
    {
        //luma
        if (sh->slice_sao_enable[Y_C] == 1)
        {
            enc_eco_sao_mode(sbac, bs, &(sao_cur_param[Y_C]));
            if (sao_cur_param[Y_C].modeIdc == SAO_MODE_NEW)
            {
                enc_eco_sao_offset(sbac, bs, &(sao_cur_param[Y_C]));
                enc_eco_sao_type(sbac, bs, &(sao_cur_param[Y_C]));
            }
        }
        for (int compIdx = U_C; compIdx < N_C; compIdx++)
        {
            if (sh->slice_sao_enable[compIdx] == 1)
            {
                enc_eco_sao_mode(sbac, bs, &(sao_cur_param[compIdx]));
                if (sao_cur_param[compIdx].modeIdc == SAO_MODE_NEW)
                {
                    enc_eco_sao_offset(sbac, bs, &(sao_cur_param[compIdx]));
                    enc_eco_sao_type(sbac, bs, &(sao_cur_param[compIdx]));
                }
            }
        }
    }
}

int enc_sao_avs2(ENC_CTX * ctx, COM_PIC * pic)
{
    Copy_frame_for_SAO(ctx->pic_sao, pic);
    SAOFrame(&ctx->info, &ctx->map, pic, ctx->pic_sao, ctx->rec_saoBlkParams);
    return COM_OK;
}

#if USE_SP
void init_cu_parent(int x, int y, int width_log2, int height_log2, double parent_RDCost, double cur_sumRDCost, ENC_PARENT_INFO* parent)
{
    parent->p_RDCost = parent_RDCost;
    parent->c_sumRDCost = cur_sumRDCost;
    parent->p_x = x;
    parent->p_y = y;
    parent->p_width_log2 = width_log2;
    parent->p_height_log2 = height_log2;
}

void copy_spinfo_to_cudata(ENC_CU_DATA *cu_data, COM_MODE *mi)
{
    int size = 0;
    int idx = 0;
    cu_data->sp_copy_direction[0] = mi->sp_copy_direction;
    cu_data->sub_string_no[0] = mi->sub_string_no;
    size = (mi->cu_width * mi->cu_height >> 2) * sizeof(COM_SP_INFO);
    com_mcpy(cu_data->sp_strInfo, mi->string_copy_info, size);
    for (int j = 0; j < mi->cu_height >> MIN_CU_LOG2; j++)
    {
        for (int i = 0; i < mi->cu_width >> MIN_CU_LOG2; i++)
        {
            cu_data->sp_copy_direction[idx + i] = mi->sp_copy_direction;
            cu_data->is_sp_pix_completed[idx + i] = mi->is_sp_pix_completed;
        }
        idx += mi->cu_width >> MIN_CU_LOG2;
    }
}

void update_sps_candidates(COM_MOTION *sps_cands, u16 *num_cands, COM_SP_INFO *p_GSinfo, u16 str_num)
{
    s16 tmp[MV_D];
    int i, j;
    for (i = 0; i < str_num; i++)
    {
        if ((*num_cands) >= SP_MAX_SPS_CANDS)
        {
            break;
        }
        if (p_GSinfo[i].is_matched == 0)
        {
            continue;
        }
        assert(*num_cands < SP_MAX_SPS_CANDS);
        tmp[MV_X] = p_GSinfo[i].offset_x;
        tmp[MV_Y] = p_GSinfo[i].offset_y;
        for (j = 0; j < (*num_cands); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], tmp))
            {
                break;
            }
        }
        if (j == (*num_cands))
        {
            sps_cands[*num_cands].mv[0][MV_X] = p_GSinfo[i].offset_x;
            sps_cands[*num_cands].mv[0][MV_Y] = p_GSinfo[i].offset_y;
            (*num_cands)++;
        }
    }
}

void copy_sps_motion(COM_MOTION *motion_dst, u16 *cnt_cands_dst, const COM_MOTION *motion_src, const u16 cnt_cands_src)
{
    *cnt_cands_dst = cnt_cands_src;
    memcpy(motion_dst, motion_src, sizeof(COM_MOTION) * cnt_cands_src);
}
#endif

static double mode_coding_tree(ENC_CTX *ctx, ENC_CORE *core, int x0, int y0, int cup, int cu_width_log2, int cu_height_log2, int cud
                               , const int parent_split, int qt_depth, int bet_depth, u8 cons_pred_mode, u8 tree_status)
{

    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    s8 best_split_mode = NO_SPLIT;
    u8 best_cons_pred_mode = NO_MODE_CONS;
    int bit_cnt;
    double cost_best = MAX_COST;
    double cost_temp = MAX_COST;
    ENC_SBAC s_temp_depth = { 0 };
    int boundary = !(x0 + cu_width <= ctx->info.pic_width && y0 + cu_height <= ctx->info.pic_height);
    int boundary_r = 0, boundary_b = 0;
    int split_allow[SPLIT_QUAD + 1]; //allowed split by normative and non-normative selection
    SPLIT_MODE split_mode = NO_SPLIT;
    double best_split_cost = MAX_COST;
    double best_curr_cost = MAX_COST;
    int num_split_tried = 0;
    int num_split_to_try = 0;
    int next_split = 1; //early termination on split by setting this to 0

    COM_MOTION motion_cands_curr[ALLOWED_HMVP_NUM];
    s8 cnt_hmvp_cands_curr = 0;
    COM_MOTION motion_cands_last[ALLOWED_HMVP_NUM];
    s8 cnt_hmvp_cands_last = 0;
    if (ctx->info.sqh.num_of_hmvp_cand)
    {
        copy_motion_table(motion_cands_last, &cnt_hmvp_cands_last, core->motion_cands, core->cnt_hmvp_cands);
    }
#if IBC_BVP
    COM_BLOCK_MOTION block_motion_cands_curr[ALLOWED_HBVP_NUM];
    s8 cnt_hbvp_cands_curr = 0;
    COM_BLOCK_MOTION block_motion_cands_last[ALLOWED_HBVP_NUM];
    s8 cnt_hbvp_cands_last = 0;
    if (ctx->info.sqh.num_of_hbvp_cand)
    {
        copy_block_motion_table(block_motion_cands_last, &cnt_hbvp_cands_last, core->block_motion_cands, core->cnt_hbvp_cands);
    }
#endif
#if USE_SP
    COM_MOTION p_offset_curr[SP_MAX_SPS_CANDS]; 
    u16 p_offset_num_curr = 0;
    COM_MOTION b_offset_curr[SP_MAX_SPS_CANDS]; 
    u16 b_offset_num_curr = 0;
    COM_MOTION n_cands_curr[SP_RECENT_CANDS];
    s8 cnt_n_cands_curr = 0;
    COM_MOTION n_cands_last[SP_RECENT_CANDS];
    s8 cnt_n_cands_last = 0;
    if (ctx->info.pic_header.sp_pic_flag)
    {
        copy_motion_table(n_cands_last, &cnt_n_cands_last, core->n_recent_offset, core->n_offset_num);
    }
    u8 sp_skip_non_scc = FALSE;
#endif
#if FIMC
    if (ctx->info.sqh.fimc_enable_flag)
    {
        int cu_x = x0;
        int cu_y = y0;
        int rem_x_128 = cu_x % 128;
        int rem_y_64 = cu_y % 64;

        if (rem_x_128 == 0 && rem_y_64 == 0)
        {
            com_cntmpm_reset(&core->cntmpm_cands);
        }
    }

    // copy table
    COM_CNTMPM cntmpm_cands_curr;
    COM_CNTMPM cntmpm_cands_last;
    if (ctx->info.sqh.fimc_enable_flag)
    {
        com_cntmpm_copy(&cntmpm_cands_last, &core->cntmpm_cands);
        com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
    }
#endif

#if CHROMA_NOT_SPLIT
    ctx->tree_status = tree_status;
#endif
#if MODE_CONS
    ctx->cons_pred_mode = cons_pred_mode;
#endif

    SBAC_LOAD(core->s_curr_before_split[cu_width_log2 - 2][cu_height_log2 - 2], core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    //decide allowed split modes for the current node
    if (cu_width > MIN_CU_SIZE || cu_height > MIN_CU_SIZE)
    {
        /***************************** Step 1: decide normatively allowed split modes ********************************/
        boundary_b = boundary && (y0 + cu_height > ctx->info.pic_height) && !(x0 + cu_width > ctx->info.pic_width);
        boundary_r = boundary && (x0 + cu_width > ctx->info.pic_width) && !(y0 + cu_height > ctx->info.pic_height);
        com_check_split_mode(&ctx->info.sqh, split_allow, cu_width_log2, cu_height_log2, boundary, boundary_b, boundary_r, ctx->info.log2_max_cuwh, ctx->info.pic_header.temporal_id
                             , parent_split, qt_depth, bet_depth, ctx->info.pic_header.slice_type);
        for (int i = 1; i < MAX_SPLIT_NUM; i++)
            num_split_to_try += split_allow[i];
        /***************************** Step 2: apply encoder split constraints *************************************/
        if (cu_width == 64 && cu_height == 128)
        {
            split_allow[SPLIT_BI_VER] = 0;
        }
        if (cu_width == 128 && cu_height == 64)
        {
            split_allow[SPLIT_BI_HOR] = 0;
        }
        if (ctx->slice_type == SLICE_I && cu_width == 128 && cu_height == 128)
        {
            split_allow[NO_SPLIT] = 0;
        }
        /***************************** Step 3: reduce split modes by fast algorithm ********************************/
#if FS_SAME_SIZE_PER_X_CTU
        assert(FS_ALL_COMBINATION == 0);
        fixed_split_for_target_size(ctx, core, cu_width_log2, cu_height_log2, qt_depth, bet_depth, split_allow);
#elif FS_ALL_COMBINATION
        assert(FS_SAME_SIZE_PER_X_CTU == 0);
        fixed_split_for_target_split_combination(ctx, core, cu_width_log2, cu_height_log2, qt_depth, bet_depth, split_allow);
#else
        int do_split = 1;
        int do_curr  = 1;
        check_run_split(core, cu_width_log2, cu_height_log2, cup, next_split, do_curr, do_split, split_allow, boundary);
#endif
    }
    else
    {
        split_allow[0] = 1;
        for (int i = 1; i < MAX_SPLIT_NUM; i++)
        {
            split_allow[i] = 0;
        }
    }

    if (!boundary  && cu_width <= ctx->info.max_cuwh && cu_height <= ctx->info.max_cuwh)
    {
        cost_temp = 0.0;
        init_cu_data(&core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], cu_width_log2, cu_height_log2);
        split_mode = NO_SPLIT;
        if (split_allow[split_mode])
        {
            if (cu_width > MIN_CU_SIZE || cu_height > MIN_CU_SIZE)
            {
                /* consider CU split mode */
                SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                //enc_sbac_bit_reset(&core->s_temp_run);
                bit_cnt = enc_get_bit_number(&core->s_temp_run);
                com_set_split_mode(NO_SPLIT, cud, 0, cu_width, cu_height, cu_width, core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2].split_mode
                                  );
                enc_eco_split_mode(&core->bs_temp, ctx, core, cud, 0, cu_width, cu_height, cu_width
                                   , parent_split, qt_depth, bet_depth, x0, y0);
                bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                cost_temp += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                SBAC_STORE(core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_run);
            }
            core->cup = cup;
            clear_map_scu(ctx, core, x0, y0, cu_width, cu_height);

            if (ctx->info.sqh.num_of_hmvp_cand && ctx->info.pic_header.slice_type != SLICE_I)
            {
                copy_motion_table(motion_cands_curr, &cnt_hmvp_cands_curr, motion_cands_last, cnt_hmvp_cands_last);
            }
#if IBC_BVP
            if (ctx->info.sqh.num_of_hbvp_cand)
            {
                copy_block_motion_table(block_motion_cands_curr, &cnt_hbvp_cands_curr, block_motion_cands_last, cnt_hbvp_cands_last);
            }
#endif
#if USE_SP
            if (ctx->info.pic_header.sp_pic_flag)
            {
                copy_sps_motion(p_offset_curr, &p_offset_num_curr, core->parent_offset, core->p_offset_num);
                copy_sps_motion(b_offset_curr, &b_offset_num_curr, core->brother_offset, core->b_offset_num);
                copy_motion_table(n_cands_curr, &cnt_n_cands_curr, n_cands_last, cnt_n_cands_last);
                sp_skip_non_scc = core->sp_skip_non_scc;
            }
#endif
#if CHROMA_NOT_SPLIT
            ctx->tree_status = tree_status;
#endif
#if MODE_CONS
            ctx->cons_pred_mode = cons_pred_mode;
#endif
            cost_temp += mode_coding_unit(ctx, core, x0, y0, cu_width_log2, cu_height_log2, cud);
        }
        else
        {
            cost_temp = MAX_COST;
        }
        if (!core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_visit)
        {
            core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[split_mode] = cost_temp;
            best_curr_cost = cost_temp;
        }
        best_curr_cost = cost_temp;

        if (cost_best > cost_temp)
        {
#if USE_SP
            if (ctx->info.pic_header.sp_pic_flag)
            {
                if (!sp_is_hash_updated(core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2].is_sp_pix_completed, x0, y0, cu_width, cu_height, ctx->info.pic_width_in_scu))
                {
                    sp_cu_hash_update(ctx->sp_encoder, cu_width_log2, cu_height_log2, x0, y0);
                }
                sp_cu_hash_copy(ctx->sp_encoder, cu_width_log2 - 2, cu_height_log2 - 2, HS_TEMP_BEST, cu_width_log2 - 2, cu_height_log2 - 2, HS_NEXT_BEST);
            }
#endif
            /* backup the current best data */
            copy_cu_data(&core->cu_data_best[cu_width_log2 - 2][cu_height_log2 - 2], &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], 0, 0, cu_width_log2, cu_height_log2, cu_width_log2, cud, tree_status);
            // update the HMVP list from larger CU
#if AWP
            if (ctx->info.sqh.num_of_hmvp_cand && core->mod_info_best.cu_mode != MODE_INTRA && !core->mod_info_best.affine_flag && !core->mod_info_best.awp_flag)
#else
            if (ctx->info.sqh.num_of_hmvp_cand && core->mod_info_best.cu_mode != MODE_INTRA && !core->mod_info_best.affine_flag)
#endif
            {
                ENC_CU_DATA *cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
                update_skip_candidates(motion_cands_curr, &cnt_hmvp_cands_curr, ctx->info.sqh.num_of_hmvp_cand, cu_data->mv[0], cu_data->refi[0]);
                assert(ctx->tree_status != TREE_C);
            }

#if USE_SP
            if (ctx->info.pic_header.sp_pic_flag)
            {
                ENC_CU_DATA *cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
                update_sps_candidates(p_offset_curr, &p_offset_num_curr, cu_data->sp_strInfo, cu_data->sub_string_no[0]);
                update_sps_candidates(b_offset_curr, &b_offset_num_curr, cu_data->sp_strInfo, cu_data->sub_string_no[0]);
#if !SP_PRED
                if (core->mod_info_best.cu_mode == MODE_IBC && !core->mod_info_best.affine_flag && core->mod_info_best.sp_flag)
                {
                    update_sp_recent_cands(n_cands_curr, &cnt_n_cands_curr, cu_data->sp_strInfo, cu_data->sub_string_no[0], cu_data->sp_copy_direction[0]);
                }
#endif
                sp_skip_non_scc = core->sp_skip_non_scc;
            }
#endif
#if IBC_BVP && !SP_PRED
            // update the HBVP list from larger CU
            if (ctx->info.sqh.num_of_hbvp_cand && core->mod_info_best.cu_mode == MODE_IBC && !core->mod_info_best.affine_flag
#if USE_SP
                &&!core->mod_info_best.sp_flag
#endif
                )
            {
                ENC_CU_DATA *cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
                update_ibc_skip_candidates(block_motion_cands_curr, &cnt_hbvp_cands_curr, ctx->info.sqh.num_of_hbvp_cand, cu_data->mv[0][0], x0, y0, cu_width, cu_height);
                assert(ctx->tree_status != TREE_C);
            }
#endif

#if SP_PRED
            if (ctx->info.sqh.num_of_hbvp_cand && core->mod_info_best.cu_mode == MODE_IBC && !core->mod_info_best.affine_flag)
            {
                ENC_CU_DATA *cu_data = &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2];
                if (!core->mod_info_best.sp_flag)
                {
                    update_ibc_skip_candidates(block_motion_cands_curr, &cnt_hbvp_cands_curr, ctx->info.sqh.num_of_hbvp_cand, cu_data->mv[0][0], x0, y0, cu_width, cu_height, cu_width * cu_height);

                    COM_SP_INFO tmp_info;
                    tmp_info.is_matched = TRUE;
#if SP_ALIGN_SIGN
                    tmp_info.offset_x = cu_data->mv[0][0][MV_X] >> 2;
                    tmp_info.offset_y = cu_data->mv[0][0][MV_Y] >> 2;
#else
                    tmp_info.offset_x = -(cu_data->mv[0][0][MV_X] >> 2);
                    tmp_info.offset_y = -(cu_data->mv[0][0][MV_Y] >> 2);
#endif
                    update_sps_candidates(p_offset_curr, &p_offset_num_curr, &tmp_info, 1);
                    update_sps_candidates(b_offset_curr, &b_offset_num_curr, &tmp_info, 1);
                    assert(ctx->tree_status != TREE_C);
                }
                else
                {
                    s16 offset_curr[MV_D] = { 0 };
                    int cur_pixel = 0;
                    int* p_trav_scan_order = com_tbl_raster2trav[cu_data->sp_copy_direction[0]][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
                    int trav_x, trav_y, trav_order_index;

                    for (int j = 0; j < cu_data->sub_string_no[0]; j++)
                    {
                        if (cu_data->sp_strInfo[j].is_matched)
                        {
                            trav_order_index = p_trav_scan_order[cur_pixel];
                            trav_x = x0 + GET_TRAV_X(trav_order_index, 1 << cu_width_log2);
                            trav_y = y0 + GET_TRAV_Y(trav_order_index, cu_width_log2);
                            offset_curr[MV_X] = cu_data->sp_strInfo[j].offset_x;
                            offset_curr[MV_Y] = cu_data->sp_strInfo[j].offset_y;

#if SP_ALIGN_SIGN
                            if (!((cu_data->sp_strInfo[j].is_matched == 0)
                                || (offset_curr[MV_X] == 0 && offset_curr[MV_Y] == -1 && cu_data->sp_copy_direction[0] == TRUE)
                                || (offset_curr[MV_X] == -1 && offset_curr[MV_Y] == 0 && cu_data->sp_copy_direction[0] == FALSE)))
                            {

                                offset_curr[MV_X] = offset_curr[MV_X] << 2;
                                offset_curr[MV_Y] = offset_curr[MV_Y] << 2;
#else
                            if (!((cu_data->sp_strInfo[j].is_matched == 0)
                                || (offset_curr[MV_X] == 0 && offset_curr[MV_Y] == 1 && cu_data->sp_copy_direction[0] == TRUE)
                                || (offset_curr[MV_X] == 1 && offset_curr[MV_Y] == 0 && cu_data->sp_copy_direction[0] == FALSE)))
                            {
                                offset_curr[MV_X] = (-offset_curr[MV_X]) << 2;
                                offset_curr[MV_Y] = (-offset_curr[MV_Y]) << 2;
#endif
                                update_ibc_skip_candidates(block_motion_cands_curr, &cnt_hbvp_cands_curr, ctx->info.sqh.num_of_hbvp_cand, offset_curr, trav_x, trav_y, cu_width, cu_height, cu_data->sp_strInfo[j].length);
                            }
                            cur_pixel += cu_data->sp_strInfo[j].length;
                        }
                        else
                        {
                            cur_pixel++;
                        }
                    }
                }

                // copy to sp_recent_cand
                cnt_n_cands_curr = cnt_hbvp_cands_curr;
                for (int j = 0;j < cnt_hbvp_cands_curr;j++)
                {
#if SP_ALIGN_SIGN
                    n_cands_curr[j].mv[0][MV_X] = block_motion_cands_curr[j].mv[MV_X];
                    n_cands_curr[j].mv[0][MV_Y] = block_motion_cands_curr[j].mv[MV_Y];
#else
                    n_cands_curr[j].mv[0][MV_X] = -(block_motion_cands_curr[j].mv[MV_X]);
                    n_cands_curr[j].mv[0][MV_Y] = -(block_motion_cands_curr[j].mv[MV_Y]);
#endif
                }
            }
#endif

#if FIMC
            if (ctx->info.sqh.fimc_enable_flag && core->mod_info_best.cu_mode == MODE_INTRA)
            {
                assert(!core->mod_info_best.ibc_flag);
                assert(ctx->tree_status != TREE_C);
                com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
            }
#endif

            cost_best = cost_temp;
            best_split_mode = NO_SPLIT;
#if MODE_CONS
            best_cons_pred_mode = cons_pred_mode;
#endif
            SBAC_STORE(s_temp_depth, core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            mode_cpy_rec_to_ref(core, x0, y0, cu_width, cu_height, PIC_REC(ctx), tree_status);
#if USE_IBC
            if (ctx->info.pic_header.ibc_flag)
            {
                mode_cpy_rec_to_ref(core, x0, y0, cu_width, cu_height, ctx->ibc_unfiltered_rec_pic, tree_status);
            }
#endif
        }
#if SPLIT_SPEED_UP_BUGFIX
        if (split_allow[split_mode] != 0 && cons_pred_mode != ONLY_INTRA)
#else
        if (split_allow[split_mode] != 0)
#endif
        {
            core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].visit = 1;
        }
    }
#if ENC_ECU_ADAPTIVE
    if (cost_best != MAX_COST && cud >= (ctx->ptr % 2 ? ENC_ECU_DEPTH - 1 : ENC_ECU_DEPTH)
#else
    if (cost_best != MAX_COST && cud >= ENC_ECU_DEPTH
#endif
            && core->mod_info_best.cu_mode == MODE_SKIP
       )
    {
        next_split = 0;
    }
    if (cost_best != MAX_COST && ctx->info.pic_header.slice_type == SLICE_I)
    {
        int dist_cu = core->dist_cu_best;
        int dist_cu_th = 1 << (cu_width_log2 + cu_height_log2 + 7);
#if IPCM
        if (core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2].ipm[PB0][0] != IPD_IPCM && dist_cu < dist_cu_th)
#else
        if (dist_cu < dist_cu_th)
#endif
        {
            u8 bits_inc_by_split = 0;
            bits_inc_by_split += (cu_width_log2 + cu_height_log2 >= 6) ? 2 : 0; //two split flags
            bits_inc_by_split += 8; //one more (intra dir + cbf + edi_flag + mtr info) + 1-bit penalty, approximately 8 bits
            if (dist_cu < ctx->lambda[0] * bits_inc_by_split)
                next_split = 0;
        }
    }
    if ((cu_width > MIN_CU_SIZE || cu_height > MIN_CU_SIZE) && next_split)
    {
        SPLIT_MODE split_mode_order[MAX_SPLIT_NUM];
        int split_mode_num = 0;
        com_split_get_split_rdo_order(cu_width, cu_height, split_mode_order);
        for (split_mode_num = 1; split_mode_num < MAX_SPLIT_NUM; ++split_mode_num)
        {
            split_mode = split_mode_order[split_mode_num];
#if EQT
            int is_mode_EQT = com_split_is_EQT(split_mode);
            int EQT_not_skiped = is_mode_EQT ? (best_split_mode != NO_SPLIT || cost_best == MAX_COST) : 1;
#else
            int is_mode_EQT = 0;
            int EQT_not_skiped = 1;
#endif

            if (split_allow[split_mode] && EQT_not_skiped)
            {
                double cost_mode_best = MAX_COST;
                COM_SPLIT_STRUCT split_struct;
                if (split_allow[split_mode])
                    num_split_tried++;
                com_split_get_part_structure(split_mode, x0, y0, cu_width, cu_height, cup, cud, ctx->log2_culine, &split_struct);
                if (split_allow[split_mode])
                {
                    int prev_log2_sub_cuw = split_struct.log_cuw[0];
                    int prev_log2_sub_cuh = split_struct.log_cuh[0];
                    u8 tree_status_child = TREE_LC;
                    u8 num_cons_pred_mode_loop = 1;
                    u8 cons_pred_mode_child = NO_MODE_CONS;
#if CHROMA_NOT_SPLIT
                    tree_status_child = (tree_status == TREE_LC && com_tree_split(cu_width, cu_height, split_mode, ctx->slice_type)) ? TREE_L : tree_status;
#endif
#if MODE_CONS
                    num_cons_pred_mode_loop = (cons_pred_mode == NO_MODE_CONS && com_constrain_pred_mode(cu_width, cu_height, split_mode, ctx->slice_type)) ? 2 : 1;
#endif
                    for (int loop_idx = 0; loop_idx < num_cons_pred_mode_loop; loop_idx++)
                    {
                        if (ctx->info.sqh.num_of_hmvp_cand && ctx->info.pic_header.slice_type != SLICE_I)
                        {
                            copy_motion_table(core->motion_cands, &core->cnt_hmvp_cands, motion_cands_last, cnt_hmvp_cands_last);
                        }
#if USE_SP
                        if (ctx->info.pic_header.sp_pic_flag)
                        {
                            copy_sps_motion(core->parent_offset, &core->p_offset_num, p_offset_curr, p_offset_num_curr);
                            copy_motion_table(core->n_recent_offset, &core->n_offset_num, n_cands_last, cnt_n_cands_last);
                        }
#endif
#if IBC_BVP
                        if (ctx->info.sqh.num_of_hbvp_cand)
                        {
                            copy_block_motion_table(core->block_motion_cands, &core->cnt_hbvp_cands, block_motion_cands_last, cnt_hbvp_cands_last);
                        }
#endif
#if FIMC
                        // copy table
                        if (ctx->info.sqh.fimc_enable_flag)
                        {
                            com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_last);
                        }
#endif
#if MODE_CONS
                        if (cons_pred_mode == NO_MODE_CONS)
                        {
                            if (num_cons_pred_mode_loop == 1)
                                cons_pred_mode_child = NO_MODE_CONS;
                            else
                                cons_pred_mode_child = loop_idx == 0 ? ONLY_INTER : ONLY_INTRA;
                        }
                        else
                        {
                            cons_pred_mode_child = cons_pred_mode;
                            assert(loop_idx == 0 && num_cons_pred_mode_loop == 1);
                        }
#endif
                        init_cu_data(&core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], cu_width_log2, cu_height_log2);
                        clear_map_scu(ctx, core, x0, y0, cu_width, cu_height);
                        int part_num = 0;
                        cost_temp = 0.0;
                        if (x0 + cu_width <= ctx->info.pic_width && y0 + cu_height <= ctx->info.pic_height)
                        {
                            /* consider CU split flag */
                            SBAC_LOAD(core->s_temp_run, core->s_curr_before_split[cu_width_log2 - 2][cu_height_log2 - 2]);
                            //enc_sbac_bit_reset(&core->s_temp_run);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run);
                            com_set_split_mode(split_mode, cud, 0, cu_width, cu_height, cu_width, core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2].split_mode
                                              );
                            enc_eco_split_mode(&core->bs_temp, ctx, core, cud, 0, cu_width, cu_height, cu_width
                                               , parent_split, qt_depth, bet_depth, x0, y0);
#if MODE_CONS

                            if (cons_pred_mode == NO_MODE_CONS && com_constrain_pred_mode(cu_width, cu_height, split_mode, ctx->slice_type))
                                enc_eco_cons_pred_mode_child(&core->bs_temp, cons_pred_mode_child);
                            else
                                assert(cons_pred_mode_child == cons_pred_mode);
#endif
                            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                            cost_temp += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                            SBAC_STORE(core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_run);

                            if( best_curr_cost != MAX_COST )
                            {
                                if( best_curr_cost*0.9 + cost_temp + RATE_TO_COST_LAMBDA( ctx->lambda[0], 1 ) > best_curr_cost )
                                {
                                    continue;
                                }
                            }
                        }
                        for (part_num = 0; part_num < split_struct.part_count; ++part_num)
                        {
                            int cur_part_num = part_num;
                            int log2_sub_cuw = split_struct.log_cuw[cur_part_num];
                            int log2_sub_cuh = split_struct.log_cuh[cur_part_num];
                            int x_pos = split_struct.x_pos[cur_part_num];
                            int y_pos = split_struct.y_pos[cur_part_num];
                            int cur_cuw = split_struct.width[cur_part_num];
                            int cur_cuh = split_struct.height[cur_part_num];

                            if ((x_pos < ctx->info.pic_width) && (y_pos < ctx->info.pic_height))
                            {
                                if (part_num == 0)
                                    SBAC_LOAD(core->s_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                                else
                                    SBAC_LOAD(core->s_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->s_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
#if USE_SP
                                if (ctx->info.pic_header.sp_pic_flag)
                                {
                                    if (part_num == 0)
                                    {
                                        sp_cu_hash_copy(ctx->sp_encoder, log2_sub_cuw - 2, log2_sub_cuh - 2, HS_CURR_BEST, cu_width_log2 - 2, cu_height_log2 - 2, HS_CURR_BEST);
                                    }
                                    else
                                    {
                                        sp_cu_hash_copy(ctx->sp_encoder, log2_sub_cuw - 2, log2_sub_cuh - 2, HS_CURR_BEST, prev_log2_sub_cuw - 2, prev_log2_sub_cuh - 2, HS_NEXT_BEST);
                                    }
                                    if (part_num == 0 && cu_width == 64 && cu_height == 64)
                                    {
                                        core->b_offset_num = 0;
                                        com_mset_x64a(core->brother_offset, 0, sizeof(COM_MOTION) * SP_MAX_SPS_CANDS); 
                                    }
                                    init_cu_parent(x0, y0, cu_width_log2, cu_height_log2, cost_best, cost_temp, &core->cu_parent_info);
                                    core->sp_skip_non_scc = sp_skip_non_scc;
                                }
#endif
                                cost_temp += mode_coding_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], log2_sub_cuw, log2_sub_cuh, split_struct.cud
                                                              , split_mode, INC_QT_DEPTH(qt_depth, split_mode), INC_BET_DEPTH(bet_depth, split_mode), cons_pred_mode_child, tree_status_child);
                                copy_cu_data(&core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], &core->cu_data_best[log2_sub_cuw - 2][log2_sub_cuh - 2], x_pos - split_struct.x_pos[0], y_pos - split_struct.y_pos[0], log2_sub_cuw, log2_sub_cuh, cu_width_log2, cud, tree_status_child);
                                update_map_scu(ctx, core, x_pos, y_pos, cur_cuw, cur_cuh, tree_status_child);
                                prev_log2_sub_cuw = log2_sub_cuw;
                                prev_log2_sub_cuh = log2_sub_cuh;
                            }
                        }
#if CHROMA_NOT_SPLIT
                        if (tree_status_child == TREE_L && tree_status == TREE_LC)
                        {

                            ctx->tree_status = TREE_C;
                            ctx->cons_pred_mode = NO_MODE_CONS;
                            cost_temp += mode_coding_unit(ctx, core, x0, y0, cu_width_log2, cu_height_log2, cud);
                            ctx->tree_status = TREE_LC;
                        }
#endif

                        if (cost_mode_best - 0.0001 > cost_temp)
                        {
                            cost_mode_best = cost_temp;
                        }
                        if (cost_best - 0.0001 > cost_temp)
                        {
#if USE_SP
                            if (ctx->info.pic_header.sp_pic_flag)
                            {
                                sp_cu_hash_copy(ctx->sp_encoder, cu_width_log2 - 2, cu_height_log2 - 2, HS_NEXT_BEST, prev_log2_sub_cuw - 2, prev_log2_sub_cuh - 2, HS_NEXT_BEST);
                                sp_cu_hash_copy(ctx->sp_encoder, cu_width_log2 - 2, cu_height_log2 - 2, HS_TEMP_BEST, cu_width_log2 - 2, cu_height_log2 - 2, HS_NEXT_BEST);
                            }
#endif
                            /* backup the current best data */
                            copy_cu_data(&core->cu_data_best[cu_width_log2 - 2][cu_height_log2 - 2], &core->cu_data_temp[cu_width_log2 - 2][cu_height_log2 - 2], 0, 0, cu_width_log2, cu_height_log2, cu_width_log2, cud, tree_status);
                            cost_best = cost_temp;
                            SBAC_STORE(s_temp_depth, core->s_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
                            best_split_mode = split_mode;
#if MODE_CONS
                            best_cons_pred_mode = cons_pred_mode_child;
#endif
                            if (ctx->info.sqh.num_of_hmvp_cand && ctx->info.pic_header.slice_type != SLICE_I)
                            {
                                copy_motion_table(motion_cands_curr, &cnt_hmvp_cands_curr, core->motion_cands, core->cnt_hmvp_cands);
                            }
#if IBC_BVP
                            if (ctx->info.sqh.num_of_hbvp_cand)
                            {
                                copy_block_motion_table(block_motion_cands_curr, &cnt_hbvp_cands_curr, core->block_motion_cands, core->cnt_hbvp_cands);
                            }
#endif
#if USE_SP
                            if (ctx->info.pic_header.sp_pic_flag)
                            {
                                copy_motion_table(n_cands_curr, &cnt_n_cands_curr, core->n_recent_offset, core->n_offset_num);
                            }
#endif
#if FIMC
                            // copy table
                            if (ctx->info.sqh.fimc_enable_flag)
                            {
                                com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
                            }
#endif
                        }
                    }
                }
                if (split_mode != NO_SPLIT && cost_mode_best < best_split_cost)
                    best_split_cost = cost_mode_best;
                if (!core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_visit)
                {
                    core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_cost[split_mode] = cost_mode_best;
                }
            }
            if (!core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_visit && num_split_tried > 0)
            {
                if ((best_curr_cost * (1.10)) < best_split_cost)
                {
                    break;
                }
            }
        }
    }
    mode_cpy_rec_to_ref(core, x0, y0, cu_width, cu_height, PIC_REC(ctx), tree_status);
#if USE_IBC
    if (ctx->info.pic_header.ibc_flag)
    {
        mode_cpy_rec_to_ref(core, x0, y0, cu_width, cu_height, ctx->ibc_unfiltered_rec_pic, tree_status);
    }
#endif
    /* restore best data */
    com_set_split_mode(best_split_mode, cud, 0, cu_width, cu_height, cu_width, core->cu_data_best[cu_width_log2 - 2][cu_height_log2 - 2].split_mode);
#if MODE_CONS
    if (cons_pred_mode == NO_MODE_CONS && com_constrain_pred_mode(cu_width, cu_height, best_split_mode, ctx->slice_type))
    {
        com_set_cons_pred_mode(best_cons_pred_mode, cud, 0, cu_width, cu_height, cu_width, core->cu_data_best[cu_width_log2 - 2][cu_height_log2 - 2].split_mode);
    }
#endif

    if (ctx->info.sqh.num_of_hmvp_cand && ctx->info.pic_header.slice_type != SLICE_I)
    {
        copy_motion_table(core->motion_cands, &core->cnt_hmvp_cands, motion_cands_curr, cnt_hmvp_cands_curr);
    }
#if IBC_BVP
    if (ctx->info.sqh.num_of_hbvp_cand)
    {
        copy_block_motion_table(core->block_motion_cands, &core->cnt_hbvp_cands, block_motion_cands_curr, cnt_hbvp_cands_curr);
    }
#endif
#if USE_SP
    if (ctx->info.pic_header.sp_pic_flag)
    {
        copy_motion_table(core->n_recent_offset, &core->n_offset_num, n_cands_curr, cnt_n_cands_curr);
        copy_sps_motion(core->brother_offset, &core->b_offset_num, b_offset_curr, b_offset_num_curr);
        sp_cu_hash_copy(ctx->sp_encoder, cu_width_log2 - 2, cu_height_log2 - 2, HS_NEXT_BEST, cu_width_log2 - 2, cu_height_log2 - 2, HS_TEMP_BEST);
        if (cu_width_log2 == MAX_CU_LOG2 && cu_height_log2 == MAX_CU_LOG2) 
        {
            sp_lcu_hash_restore(ctx->sp_encoder, core->x_pel, core->y_pel); 
        }
    }
#endif
#if FIMC
    // copy table
    if (ctx->info.sqh.fimc_enable_flag)
    {
        com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_curr);
    }
#endif
    SBAC_LOAD(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], s_temp_depth);

    if (num_split_to_try > 0)
    {
        {
            if (best_split_mode == NO_SPLIT)
            {
                core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].nosplit += 1;
            }
            else
            {
                core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split += 1;
            }
        }
        core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][cup].split_visit = 1;
    }
    com_assert(cost_best != MAX_COST);
    return (cost_best > MAX_COST) ? MAX_COST : cost_best;
}

int enc_mode_init_frame(ENC_CTX *ctx)
{
    int ret;
    /* initialize pintra */
    ret = pintra_init_frame(ctx);
    com_assert_rv(ret == COM_OK, ret);
    /* initialize pinter */
    ret = pinter_init_frame(ctx);
    com_assert_rv(ret == COM_OK, ret);

#if FIMC
    int reuse_hash_result = 0;
#endif

#if USE_IBC
    if (ctx->param.use_ibc_flag)
    {
        ret = pibc_init_frame(ctx);
        com_assert_rv(ret == COM_OK, ret);
        rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORG(ctx));
#if FIMC || ISTS
        reuse_hash_result = 1;
#endif
        if (ctx->param.use_ibc_flag == 2)
        {
#if FIMC || ISTS
            reuse_hash_result = 2;
#endif
            ctx->info.pic_header.ibc_flag = getHashHitRatio( ctx, ctx->ibc_hash_handle );
        }
        else
        {
            ctx->info.pic_header.ibc_flag = 1;
        }
#if USE_SP
        if (ctx->param.sp_enable_flag)
        {
#if SP_PIC
            ctx->info.pic_header.sp_pic_flag = get_adaptive_sp_flag(ctx->sp_encoder);
#else
            ctx->info.pic_header.sp_pic_flag = 1;
#endif
        }
        else
        {
            ctx->info.pic_header.sp_pic_flag = 0;
        }
#endif
    }
    else
    {
        ctx->info.pic_header.ibc_flag = 0;
    }
#endif

#if FIMC
    if (ctx->param.fimc_enable_flag == 1)
    {
        // frame adaptive 
        if (!reuse_hash_result)
        {
            ret = pibc_init_frame(ctx);
            com_assert_rv(ret == COM_OK, ret);
            rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORG(ctx));
        }
        if (reuse_hash_result == 2)
        {
            ctx->info.pic_header.fimc_pic_flag = ctx->info.pic_header.ibc_flag;
        }
        else
        {
            ctx->info.pic_header.fimc_pic_flag = getHashHitRatio(ctx, ctx->ibc_hash_handle);
        }
    }
    else if (ctx->param.fimc_enable_flag == 2)
    {
        ctx->info.pic_header.fimc_pic_flag = 1;
    }
    else
    {
        ctx->info.pic_header.fimc_pic_flag = 0;
    }
#endif

#if ISTS
    if (ctx->param.ists_enable_flag == 1)
    {
        // frame adaptive 
        if (!reuse_hash_result)
        {
            ret = pibc_init_frame(ctx);
            com_assert_rv(ret == COM_OK, ret);
            rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORG(ctx));
        }
        if (reuse_hash_result == 2)
        {
            ctx->info.pic_header.ph_ists_enable_flag = ctx->info.pic_header.ibc_flag;
        }
        else
        {
            ctx->info.pic_header.ph_ists_enable_flag = getHashHitRatio(ctx, ctx->ibc_hash_handle);
        }
    }
    else if (ctx->param.ists_enable_flag == 2)
    {
        ctx->info.pic_header.ph_ists_enable_flag = 1;
    }
    else
    {
        ctx->info.pic_header.ph_ists_enable_flag = 0;
    }
#endif

#if AWP_SCC
    if (ctx->param.awp_enable_flag == 2)
    {
        // frame adaptive 
        if (!reuse_hash_result)
        {
            ret = pibc_init_frame(ctx);
            com_assert_rv(ret == COM_OK, ret);
            rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORG(ctx));
        }
        if (reuse_hash_result == 2)
        {
            ctx->info.pic_header.ph_awp_refine_flag = ctx->info.pic_header.ibc_flag;
        }
        else
        {
            ctx->info.pic_header.ph_awp_refine_flag = getHashHitRatio(ctx, ctx->ibc_hash_handle);
        }
    }
    else if (ctx->param.awp_enable_flag == 3)
    {
        ctx->info.pic_header.ph_awp_refine_flag = 1;
    }
    else
    {
        ctx->info.pic_header.ph_awp_refine_flag = 0;
    }
#endif

#if DBK_SCC
    if (ctx->param.loop_filter_type_enable_flag == 1)
    {
        // frame adaptive 
        if (!reuse_hash_result)
        {
            ret = pibc_init_frame(ctx);
            com_assert_rv(ret == COM_OK, ret);
            rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORG(ctx));
        }
        if (reuse_hash_result == 2)
        {
            ctx->info.pic_header.loop_fitler_type = ctx->info.pic_header.ibc_flag;
        }
        else
        {
            ctx->info.pic_header.loop_fitler_type = getHashHitRatio(ctx, ctx->ibc_hash_handle);
        }
    }
    else
    {
        ctx->info.pic_header.loop_fitler_type = 0;
    }
#endif

#if UMVE_ENH
    ctx->info.pic_header.umve_set_flag = 0;
    if(ctx->needReset && (ctx->info.pic_header.poc > ctx->lastIPicPOC))
    {
      ctx->needReset = FALSE;
      ctx->umveAveOffset = -1;
    }
    if (ctx->dataCol && ctx->umveAveOffset > 1.1)
    {
        ctx->info.pic_header.umve_set_flag = 1;
    }
#endif

#if FIXED_SPLIT
    if (ctx->ptr == 0)
    {
        ctx->ctu_idx_in_sequence = 0;
        ctx->ctu_idx_in_seq_I_slice_ctu = 0;
        ctx->ctu_idx_in_seq_B_slice_ctu = 0;
    }
    ctx->ctu_idx_in_picture = 0;
#endif
    return COM_OK;
}

int enc_mode_init_lcu(ENC_CTX *ctx, ENC_CORE *core)
{
    int ret;
    /* initialize pintra */
    ret = pintra_init_lcu(ctx, core);
    com_assert_rv(ret == COM_OK, ret);
    /* initialize pinter */
    ret = pinter_init_lcu(ctx, core);
    com_assert_rv(ret == COM_OK, ret);
#if USE_IBC
    if (ctx->info.pic_header.ibc_flag)
    {
        ret = pibc_init_lcu(ctx, core);
        com_assert_rv(ret == COM_OK, ret);
    }
#endif
    return COM_OK;
}


static void update_to_ctx_map(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_CU_DATA *cu_data;
    int  cu_width, cu_height, i, j, w, h;
    int  x, y;
    int  core_idx, ctx_idx;
    s8(*map_refi)[REFP_NUM];
    s16(*map_mv)[REFP_NUM][MV_D];
    s8   *map_ipm;
    s8(*map_split)[MAX_CU_DEPTH][NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU];
    cu_data = &core->cu_data_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2];
    cu_width = ctx->info.max_cuwh;
    cu_height = ctx->info.max_cuwh;
    x = core->x_pel;
    y = core->y_pel;
    ctx->tree_status = TREE_LC;
    if (x + cu_width > ctx->info.pic_width)
    {
        cu_width = ctx->info.pic_width - x;
    }
    if (y + cu_height > ctx->info.pic_height)
    {
        cu_height = ctx->info.pic_height - y;
    }
    w = cu_width >> MIN_CU_LOG2;
    h = cu_height >> MIN_CU_LOG2;
    /* copy mode info */
    core_idx = 0;
    ctx_idx = (y >> MIN_CU_LOG2) * ctx->info.pic_width_in_scu + (x >> MIN_CU_LOG2);
    map_ipm = ctx->map.map_ipm;
    map_refi = ctx->map.map_refi;
    map_mv = ctx->map.map_mv;
    map_split = ctx->map.map_split;
    for (int k1 = 0; k1 < MAX_CU_DEPTH; k1++)
        for (int k2 = 0; k2 < NUM_BLOCK_SHAPE; k2++)
            for (int k3 = 0; k3 < MAX_CU_CNT_IN_LCU; k3++)
                map_split[core->lcu_num][k1][k2][k3] = cu_data->split_mode[k1][k2][k3];
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            if (cu_data->pred_mode[core_idx + j] == MODE_INTRA)
            {
                map_ipm[ctx_idx + j] = cu_data->ipm[0][core_idx + j];
                map_refi[ctx_idx + j][REFP_0] = REFI_INVALID;
                map_refi[ctx_idx + j][REFP_1] = REFI_INVALID;
                map_mv[ctx_idx + j][REFP_0][MV_X] = 0;
                map_mv[ctx_idx + j][REFP_0][MV_Y] = 0;
                map_mv[ctx_idx + j][REFP_1][MV_X] = 0;
                map_mv[ctx_idx + j][REFP_1][MV_Y] = 0;
            }
            else
            {
                map_refi[ctx_idx + j][REFP_0] = cu_data->refi[core_idx + j][REFP_0];
                map_refi[ctx_idx + j][REFP_1] = cu_data->refi[core_idx + j][REFP_1];
                map_mv[ctx_idx + j][REFP_0][MV_X] = cu_data->mv[core_idx + j][REFP_0][MV_X];
                map_mv[ctx_idx + j][REFP_0][MV_Y] = cu_data->mv[core_idx + j][REFP_0][MV_Y];
                map_mv[ctx_idx + j][REFP_1][MV_X] = cu_data->mv[core_idx + j][REFP_1][MV_X];
                map_mv[ctx_idx + j][REFP_1][MV_Y] = cu_data->mv[core_idx + j][REFP_1][MV_Y];
            }
        }
        ctx_idx += ctx->info.pic_width_in_scu;
        core_idx += (ctx->info.max_cuwh >> MIN_CU_LOG2);
    }
    update_map_scu(ctx, core, core->x_pel, core->y_pel, ctx->info.max_cuwh, ctx->info.max_cuwh, TREE_LC);
}

#if FS_ALL_COMBINATION
void derive_split_combination_CTU(ENC_CTX *ctx, ENC_CORE *core)
{
    int start_depth = START_QT_DEPTH;
    int* ctu_idx_slice_type = ctx->slice_type == SLICE_I ? &ctx->ctu_idx_in_seq_I_slice_ctu : &ctx->ctu_idx_in_seq_B_slice_ctu;
    SPLIT_MODE split_mode_start = (SPLIT_MODE)START_SPLIT_MODE;
    for (int i = 0; i < 6; i++)
    {
        ctx->split_combination[i] = NUM_SPLIT_MODE;
    }

    for (int i = 0; i < start_depth; i++)
    {
        ctx->split_combination[i] = SPLIT_QUAD;
    }

    if (split_mode_start != 0)
    {
        ctx->split_combination[start_depth] = split_mode_start;
        start_depth++;
    }

    int fac = (split_mode_start == SPLIT_QUAD || split_mode_start == NO_SPLIT) ? 6 : 5;
#if FS_SIMPLE_ORDER
    int split_mode_order[3][6] = { NO_SPLIT, SPLIT_BI_VER, SPLIT_BI_HOR, SPLIT_EQT_VER, SPLIT_EQT_HOR, SPLIT_QUAD,
                                   NO_SPLIT, SPLIT_BI_VER, SPLIT_BI_HOR, SPLIT_EQT_VER, SPLIT_EQT_HOR, SPLIT_QUAD,
                                   NO_SPLIT, SPLIT_BI_VER, SPLIT_BI_HOR, SPLIT_EQT_VER, SPLIT_EQT_HOR, SPLIT_QUAD
                                 };
#else
    //make complex combination between layers (may be can trigger problem earlier)
    int split_mode_order[3][6] = { SPLIT_BI_VER, SPLIT_BI_HOR, SPLIT_EQT_VER, SPLIT_EQT_HOR, NO_SPLIT, SPLIT_QUAD,
                                   NO_SPLIT, SPLIT_BI_HOR, SPLIT_BI_VER, SPLIT_EQT_VER, SPLIT_EQT_HOR, SPLIT_QUAD,
                                   SPLIT_EQT_VER, SPLIT_EQT_HOR, NO_SPLIT, SPLIT_BI_HOR, SPLIT_BI_VER, SPLIT_QUAD
                                 };
#endif
    //reset after trying every split combination
    if (*ctu_idx_slice_type == pow(fac, 6 - start_depth))
    {
        *ctu_idx_slice_type = 0;
        printf("\nReset %s ctu_idx to 0", ctx->slice_type == SLICE_I ? "SLICE_I" : "SLICE_B");
    }

    //derive the split combination of the current CTU based on ctu_idx
    int ctu_idx = *ctu_idx_slice_type;
    for (int i = start_depth; i < 6; i++)
    {
        ctx->split_combination[i] = split_mode_order[i % 3][ctu_idx % fac];
        ctu_idx = ctu_idx / fac;;
    }

    //debug
    for (int i = 0; i < 6; i++)
    {
        assert(ctx->split_combination[i] != NUM_SPLIT_MODE);
    }
    //NOTE: the derived combination may not be normatively allowed.
}
#endif

int enc_mode_analyze_lcu(ENC_CTX *ctx, ENC_CORE *core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    u32 *map_scu;
    int i, j, w, h;
#if IPPPCRR && LIB_PIC_UPDATE
    int x=0, y=0, pic_stride=0, wid=0, hei=0;
    double avr_self = 0, variance_self = 0, avr_residual = 0, variance_residual = 0;
    static double sum_self = 0, sum_residual = 0;
    if (ctx->rpm.libvc_data->lib_pic_update && ctx->rpm.libvc_data->end_of_intra_period)
    {
        if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
        {
            COM_PIC *picc = &ctx->pico->pic;
            COM_PIC *cz = NULL;
            x = core->x_pel; y = core->y_pel; pic_stride = picc->stride_luma; hei = wid = 1 << ctx->info.log2_max_cuwh;
            if (x + wid > ctx->info.pic_width)
            {
                wid = ctx->info.pic_width - x;
            }
            if (y + hei > ctx->info.pic_height)
            {
                hei = ctx->info.pic_height - y;
            }
            if (core->x_pel == 0 && core->y_pel == 0)
            {
                sum_self = 0; sum_residual = 0;
            }
            avr_self = 0; variance_self = 0; avr_residual = 0; variance_residual = 0;

            for (int i = 0; i < hei; i++)
            {
                for (int j = 0; j < wid; j++)
                {
                    avr_self += picc->y[(y + i)*pic_stride + x + j] / (double)(wid);
                }
            }
            avr_self /= hei;
            for (int i = 0; i < hei; i++)
            {
                for (int j = 0; j < wid; j++)
                {
                    variance_self += (picc->y[(y + i)*pic_stride + x + j] - avr_self)*(picc->y[(y + i)*pic_stride + x + j] - avr_self) / (double)(wid);
                }
            }
            variance_self /= hei;
            sum_self += variance_self;
            if (   (ctx->refp[1][0].is_library_picture == 1 && ctx->refp[1][0].pic != NULL)
                || (ctx->refp[0][0].is_library_picture == 1 && ctx->refp[0][0].pic != NULL))
            {
                if (ctx->refp[1][0].is_library_picture == 1)
                {
                    cz = ctx->refp[1][0].pic;
                }
                else
                {
                    cz = ctx->refp[0][0].pic;
                }
                for (int i = 0; i < hei; i++)
                {
                    for (int j = 0; j < wid; j++)
                    {
                        avr_residual += abs( cz->y[(y + i)*cz->stride_luma + x + j] - picc->y[(y + i)*pic_stride + x + j] ) / (double)(wid);
                    }
                }
                avr_residual /= hei;
                for (int i = 0; i < hei; i++)
                {
                    for (int j = 0; j < wid; j++)
                    {
                        variance_residual += (abs( cz->y[(y + i)*cz->stride_luma + x + j] - picc->y[(y + i)*pic_stride + x + j] ) - avr_residual) *
                               (abs( cz->y[(y + i)*cz->stride_luma + x + j] - picc->y[(y + i)*pic_stride + x + j] ) - avr_residual) / (double)(wid);
                    }
                }
                variance_residual /= hei;
                sum_residual += variance_residual;
            }
            if (sum_self * 0.7 < sum_residual)
            {
                ctx->rpm.libvc_data->update = 1;
            }
            else
            {
                ctx->rpm.libvc_data->update = 0;
            }
        }
    }
    if (ctx->rpm.libvc_data->encode_skip == 1)
    {
        return COM_OK_SKIP;
    }
#endif
    /* initialize cu data */
    init_cu_data(&core->cu_data_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2], ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh);
    init_cu_data(&core->cu_data_temp[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2], ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh);

    /* decide mode */
    mode_coding_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh, 0
                     , NO_SPLIT, 0, 0, NO_MODE_CONS, TREE_LC);
    update_to_ctx_map(ctx, core);
    copy_cu_data(&ctx->map_cu_data[core->lcu_num], &core->cu_data_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2],
                 0, 0, ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh, 0, TREE_LC);

    if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
    {
        COM_BSW *sao_bs = &(core->bs_temp);
        ENC_SBAC *sao_sbac = GET_SBAC_ENC(sao_bs);
        SBAC_LOAD((*sao_sbac), (core->s_sao_init));
        enc_SAO_rdo(ctx, core, sao_bs);
    }

    /* Reset all coded flag for the current lcu */
    mod_info_curr->x_scu = PEL2SCU(core->x_pel);
    mod_info_curr->y_scu = PEL2SCU(core->y_pel);
    map_scu = ctx->map.map_scu + ((u32)mod_info_curr->y_scu * ctx->info.pic_width_in_scu) + mod_info_curr->x_scu;
    w = COM_MIN(1 << (ctx->info.log2_max_cuwh - MIN_CU_LOG2), ctx->info.pic_width_in_scu - mod_info_curr->x_scu);
    h = COM_MIN(1 << (ctx->info.log2_max_cuwh - MIN_CU_LOG2), ctx->info.pic_height_in_scu - mod_info_curr->y_scu);
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            MCU_CLR_CODED_FLAG(map_scu[j]);
        }
        map_scu += ctx->info.pic_width_in_scu;
    }
    return COM_OK;
}
