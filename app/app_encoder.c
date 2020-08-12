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

#include <com_typedef.h>
#include "app_util.h"
#include "app_args.h"
#include "enc_def.h"
#include <math.h>

#include <time.h>

#if LINUX
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>


void handler(int sig)
{
    void *array[10];
    size_t size;
    // get void*'s for all entries on the stack
    size = backtrace(array, 10);
    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}
#endif

int g_CountDOICyCleTime;                    // number to count the DOI cycle time.
int g_DOIPrev;                              // the doi of previous frm.

#define VERBOSE_FRAME              VERBOSE_1
#define MAX_BUMP_FRM_CNT           (8 <<1)

#define MAX_BS_BUF                 (32*1024*1024)
#define PRECISE_BS_SIZE            1

typedef enum _STATES
{
    STATE_ENCODING,
    STATE_BUMPING,
    STATE_SKIPPING
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    ,
    STATE_LIB_SKIPPING
#endif
} STATES;

typedef struct _IMGB_LIST
{
    COM_IMGB    * imgb;
    int            used;
    COM_MTIME     ts;
} IMGB_LIST;

#define CALC_SSIM           1    //calculate Multi-scale SSIM (ms_ssim)
#if CALC_SSIM
#define     CALC_SSIM_UV    1    //calculate Multi-scale SSIM (ms_ssim) of U/V 
#define     MAX_SCALE       5    //number of scales
#define     WIN_SIZE        11   //window size for SSIM calculation
static double exponent[MAX_SCALE] = { 0.0448, 0.2856, 0.3001, 0.2363, 0.1333 };  //weights for different scales

enum STRUCTURE
{
    LUMA_COMPONENT,
    SIMILARITY_COMPONENT,
    SSIM,
};
#endif

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
typedef struct _ENC_LIBPIC
{
    int                 i, ret, size;
    int                 libpic_icnt;
    double              bits_libpic;
    STATES              state_lib;
    unsigned char      *bs_buf_lib;
    unsigned char      *bs_buf_lib2;
    FILE               *fp_inp_lib;
    ENC                 id_lib;
    COM_BITB            bitb_lib;
    COM_IMGB           *imgb_enc_lib;
    COM_IMGB           *imgb_rec_lib;
    ENC_STAT            stat_lib;
    int                 udata_size_lib;
    COM_CLK             clk_beg_lib, clk_end_lib, clk_tot_lib;
    COM_MTIME           pic_icnt_lib, pic_ocnt_lib, pic_skip_lib;
    int                 num_encoded_frames;
    double              bitrate_lib;
    double              psnr_lib[3];
    double              psnr_avg_lib[3];
#if CALC_SSIM
    double              ms_ssim_lib[3];
    double              ms_ssim_avg_lib[3];
#endif
    IMGB_LIST           ilist_org_lib[MAX_BUMP_FRM_CNT];
    IMGB_LIST           ilist_rec_lib[MAX_BUMP_FRM_CNT];
    IMGB_LIST          *ilist_t_lib;
    int                 is_first_enc_lib;
#if !CALC_SSIM
    double              seq_header_bit = 0;
#endif
} ENC_LIBPIC;
#endif
//functionality
static char op_fname_cfg[256] = "\0"; /* config file path name */
static char op_fname_inp[256] = "\0"; /* input original video */
static char op_fname_out[256] = "\0"; /* output bitstream */
static char op_fname_rec[256] = "\0"; /* reconstructed video */
static int  op_max_frm_num = 0;
static int  op_use_pic_signature = 0;
static int  op_w = 0;
static int  op_h = 0;
static int  op_qp = 0;
static int  op_fps = 0;
static int  op_i_period = 0;
static int  op_max_b_frames= 0;
static int  op_disable_hgop = 0;
static int  op_bit_depth_input = 8;
static int  op_skip_frames = 0;
static int  op_bit_depth_internal = 0;
static int  op_frame_qp_add = 0;
static int  op_pb_frame_qp_offset = 0;
static int  op_qp_offset_cb = 1;
static int  op_qp_offset_cr = 1;
static int  op_delta_qp_flag = 0;//mark
static int  op_qp_offset_adp = 1;
static int  op_subSampleRatio = 1;
#if CALC_SSIM_UV
static int  op_msssim_uv = 0;
#endif

//scc tools
#if USE_IBC
static int  op_enable_ibc = 0;
static int  op_ibc_search_range_x = IBC_SEARCH_RANGE;
static int  op_ibc_search_range_y = IBC_SEARCH_RANGE;
static int  op_ibc_hash_search_flag = 1;
static int  op_ibc_hash_search_max_cand = IBC_NUM_CANDIDATES;
static int  op_ibc_hash_search_range_4smallblk = IBC_SEARCH_RANGE;
static int  op_ibc_fast_method = IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE;
#if IBC_ABVR
static int  op_tool_abvr  = 1;
#endif
#if IBC_BVP
static int  op_tool_hbvp  = 12;
#endif
#endif
#if FIMC
static int  op_tool_fimc  = 0;
#endif
#if ISTS
static int  op_tool_ists  = 0;
#endif
#if USE_SP
static int  op_tool_sp    = 0;
#endif

//regular tools (phase 1 tools first in each category)
//intra 
#if IPCM
static int  op_tool_ipcm          = 1;
#endif
static int  op_tool_ipf           = 1;
#if DT_PARTITION
static int  op_tool_dt_intra      = 1;
#endif
static int  op_tool_tscpm         = 1;
#if EIPM
static int  op_tool_eipm          = 1;
#endif
#if MIPF
static int  op_tool_mipf          = 1;
#endif
#if ENHANCE_TSPCM
static int  op_tool_enhance_tscpm = 1;
#endif
#if PMC
static int  op_tool_pmc           = 1;
#endif
#if IPF_CHROMA
static int  op_tool_chroma_ipf    = 1;
#endif

//inter
static int  op_tool_amvr          = 1;
static int  op_tool_affine        = 1;
static int  op_tool_hmvp          = 8;
static int  op_tool_umve          = 1;
static int  op_tool_emvr          = 1;
static int  op_tool_smvd          = 1;
#if BIO
static int  op_tool_bio           = 1;
#endif
#if BGC
static int  op_tool_bgc           = 1;
#endif
#if DMVR
static int  op_tool_dmvr          = 1;
#endif
#if INTERPF
static int  op_tool_interpf       = 1;
#endif
#if MVAP
static int  op_tool_mvap          = 1;
#endif
#if AFFINE_UMVE
static int  op_tool_affine_umve   = 1;
#endif
#if AWP
static int  op_tool_awp           = 1;
#endif
#if AWP_MVR
static int  op_tool_awp_mvr       = 1;
#endif
#if ETMVP
static int  op_tool_etmvp         = 1;
#endif
#if SUB_TMVP
static int  op_tool_sbtmvp        = 1;
#endif
#if UMVE_ENH
static int  op_tool_umve_enh      = 1;
#endif

//transform & quantization
static int  op_tool_pbt           = 0; //off in CTC decided at 71 meeting
static int  op_tool_secTrans      = 1;
#if IST
static int  op_tool_ist           = 1;
#endif
#if SBT
static int  op_tool_sbt           = 1;
#endif
#if EST
static int  op_tool_est           = 1;
#endif

//residual coding & CABAC
#if SRCC
static int  op_tool_srcc          = 1;
#endif
#if CABAC_MULTI_PROB
static int  op_tool_mcabac        = 1;
#endif

//loop filter
static int  op_tool_deblock       = 1;
#if DBK_SCC
static int  op_tool_deblock_type  = 0;
#endif
static int  op_tool_sao           = 1;
#if ESAO
static int  op_tool_esao          = 1;
#endif 
static int  op_tool_alf           = 1;

//high level syntax
#if PHASE_2_PROFILE
static int op_profile             = PROFILE_ID;
#endif
#if PATCH
static int op_patch_stable        = 1;
static int op_cross_patch_loop_filter = 1;
static int op_patch_ref_colocated = 0;
static int op_patch_uniform       = 1;
static int op_patch_width_in_lcu  = 0;
static int op_patch_height_in_lcu = 0;
#endif
#if HLS_RPL
static char  op_rpl0[MAX_NUM_RPLS][256];
static char  op_rpl1[MAX_NUM_RPLS][256];
#endif
#if LIBVC_ON
static int  op_tool_libpic = 0;
static int  op_qp_offset_libpic = -6;
static int  op_qp_offset_rlpic = 0;
static char op_fname_libout[256] = "libpic.bin"; /* output libpic bitstream */
static char op_fname_librec[256] = "\0"; /* reconstructed libpic */
static char op_fname_libdependency[256] = "\0"; /* the file storing library picture information */
static int  op_skip_frames_when_extract_libpic = 0;
static int  op_max_frm_num_when_extract_libpic = 0;
#if IPPPCRR
#if LIB_PIC_UPDATE
static int  op_lib_pic_update = 0;
#else
static int  op_first_pic_as_libpic = 0;
#endif
#endif
#endif

//partition parameters
static int  op_ctu_size        = 128;
static int  op_min_cu_size     = 4;
static int  op_max_part_ratio  = 8;
static int  op_max_split_times = 6;
static int  op_min_qt_size     = 8;
static int  op_max_bt_size     = 128;
static int  op_max_eqt_size    = 64;
static int  op_max_dt_size     = 64;

//Quantization matrix
static int  op_wq_enable = 0;
static int  op_seq_wq_mode = 0;
static char op_seq_wq_user[2048] = "\0";
static int  op_pic_wq_data_idx = 0;
static char op_pic_wq_user[2048] = "\0";
static int  op_wq_param = 0;
static int  op_wq_model = 0;
static char op_wq_param_detailed  [256] = "[64,49,53,58,58,64]";
static char op_wq_param_undetailed[256] = "[67,71,71,80,80,106]";


typedef enum _OP_FLAGS
{
    OP_FLAG_FNAME_CFG,
    OP_FLAG_FNAME_INP,
    OP_FLAG_FNAME_OUT,
    OP_FLAG_FNAME_REC,
    OP_FLAG_WIDTH_INP,
    OP_FLAG_HEIGHT_INP,
    OP_FLAG_QP,
    OP_FLAG_FPS,
    OP_FLAG_IPERIOD,
    OP_FLAG_MAX_FRM_NUM,
    OP_FLAG_USE_PIC_SIGN,
    OP_FLAG_VERBOSE,
    OP_FLAG_MAX_B_FRAMES,
#if USE_IBC
    OP_FLAG_IBC,
#if IBC_ABVR
    OP_FLAG_IBC_ABVR,
#endif
    OP_IBC_SEARCH_RNG_X,
    OP_IBC_SEARCH_RND_Y,
    OP_IBC_HASH_FLAG,
    OP_IBC_HASH_SEARCH_MAX_CAND,
    OP_IBC_HASH_SEARCH_RANGE_4SMALLBLK,
    OP_IBC_FAST_METHOD,
#endif
    OP_FLAG_DISABLE_HGOP,
    OP_FLAG_OUT_BIT_DEPTH,
    OP_FLAG_IN_BIT_DEPTH,
    OP_FLAG_SKIP_FRAMES,
    OP_FLAG_ADD_QP_FRAME,
    OP_FLAG_PB_FRAME_QP_OFFSET,
    OP_FLAG_QP_OFFSET_CB,
    OP_FLAG_QP_OFFSET_CR,
    OP_FLAG_QP_OFFSET_ADP,
    OP_FLAG_DELTA_QP,
#if IPCM
    OP_TOOL_IPCM,
#endif
#if BIO
    OP_TOOL_BIO,
#endif
#if BGC
    OP_TOOL_BGC,
#endif
#if DMVR
    OP_TOOL_DMVR,
#endif
#if INTERPF
    OP_TOOL_INTERPF,
#endif
    OP_TOOL_AMVR,
#if MIPF
    OP_TOOL_MIPF,
#endif
    OP_TOOL_IPF,
    OP_TOOL_AFFINE,
    OP_TOOL_SMVD,
    OP_TOOL_DT_INTRA,
    OP_TOOL_PBT,
#if SBT
    OP_TOOL_SBT,
#endif
#if IST
    OP_TOOL_IST,
#endif
#if ISTS
    OP_TOOL_ISTS,
#endif
#if EST
    OP_TOOL_EST,
#endif
#if SRCC
    OP_TOOL_SRCC,
#endif
#if CABAC_MULTI_PROB
    OP_TOOL_MCABAC,
#endif
#if EIPM
    OP_TOOL_EIPM,
#endif
#if MVAP
    OP_TOOL_MVAP,
#endif
    OP_TOOL_DEBLOCK,
#if DBK_SCC
    OP_TOOL_DEBLOCK_TYPE,
#endif
    OP_TOOL_SAO,
    OP_TOOL_ALF,
    OP_WQ_ENABLE,
    OP_SEQ_WQ_MODE,
    OP_SEQ_WQ_FILE,
    OP_PIC_WQ_DATA_IDX,
    OP_PIC_WQ_FILE,
    OP_WQ_PARAM,
    OP_WQ_PARAM_MODEL,
    OP_WQ_PARAM_DETAILED,
    OP_WQ_PARAM_UNDETAILED,

#if ESAO
    OP_TOOL_ESAO,
#endif 
    OP_TOOL_HMVP,
#if IBC_BVP
    OP_TOOL_HBVP,
#endif
    OP_TOOL_TSCPM,
#if PMC
    OP_TOOL_PMC,
#endif
#if FIMC
    OP_TOOL_FIMC,
#endif
#if USE_SP
    OP_TOOL_SP,
#endif
#if ENHANCE_TSPCM
    OP_TOOL_ENHANCE_TSCPM,
#endif
#if IPF_CHROMA
    OP_TOOL_CHROMA_IPF,
#endif
#if AFFINE_UMVE
    OP_TOOL_AFFINE_UMVE,
#endif
    OP_TOOL_UMVE,
#if UMVE_ENH
    OP_TOOL_UMVE_ENH,
#endif
#if AWP
    OP_TOOL_AWP,
#endif
#if AWP_MVR
    OP_TOOL_AWP_MVR,
#endif
#if ETMVP
    OP_TOOL_ETMVP,
#endif
#if SUB_TMVP
    OP_TOOL_SBTMVP,
#endif
    OP_TOOL_EMVR,
#if PHASE_2_PROFILE
    OP_PROFILE,
#endif
#if PATCH
    OP_PATCH_STABLE,
    OP_CROSS_PATCH_LOOP_FILTER,
    OP_PATCH_REF_COLOCATED,
    OP_PATCH_UNIFORM,
    OP_PATCH_WIDTH,
    OP_PATCH_HEIGHT,
    OP_PATCH_COLUMNS,
    OP_PATCH_ROWS,
    OP_COLUMN_WIDTH,
    OP_ROW_HEIGHT,
#endif
#if HLS_RPL
    OP_FLAG_RPL0_0,
    OP_FLAG_RPL0_1,
    OP_FLAG_RPL0_2,
    OP_FLAG_RPL0_3,
    OP_FLAG_RPL0_4,
    OP_FLAG_RPL0_5,
    OP_FLAG_RPL0_6,
    OP_FLAG_RPL0_7,
    OP_FLAG_RPL0_8,
    OP_FLAG_RPL0_9,
    OP_FLAG_RPL0_10,
    OP_FLAG_RPL0_11,
    OP_FLAG_RPL0_12,
    OP_FLAG_RPL0_13,
    OP_FLAG_RPL0_14,
    OP_FLAG_RPL0_15,
    OP_FLAG_RPL0_16,
    OP_FLAG_RPL0_17,
    OP_FLAG_RPL0_18,
    OP_FLAG_RPL0_19,
    OP_FLAG_RPL0_20,
    OP_FLAG_RPL0_21,
    OP_FLAG_RPL0_22,
    OP_FLAG_RPL0_23,
    OP_FLAG_RPL0_24,  OP_FLAG_RPL0_25, OP_FLAG_RPL0_26, OP_FLAG_RPL0_27, OP_FLAG_RPL0_28, OP_FLAG_RPL0_29, OP_FLAG_RPL0_30,
    OP_FLAG_RPL0_31,

    OP_FLAG_RPL1_0,
    OP_FLAG_RPL1_1,
    OP_FLAG_RPL1_2,
    OP_FLAG_RPL1_3,
    OP_FLAG_RPL1_4,
    OP_FLAG_RPL1_5,
    OP_FLAG_RPL1_6,
    OP_FLAG_RPL1_7,
    OP_FLAG_RPL1_8,
    OP_FLAG_RPL1_9,
    OP_FLAG_RPL1_10,
    OP_FLAG_RPL1_11,
    OP_FLAG_RPL1_12,
    OP_FLAG_RPL1_13,
    OP_FLAG_RPL1_14,
    OP_FLAG_RPL1_15,
    OP_FLAG_RPL1_16,
    OP_FLAG_RPL1_17,
    OP_FLAG_RPL1_18,
    OP_FLAG_RPL1_19,
    OP_FLAG_RPL1_20,
    OP_FLAG_RPL1_21,
    OP_FLAG_RPL1_22,
    OP_FLAG_RPL1_23,
    OP_FLAG_RPL1_24, OP_FLAG_RPL1_25, OP_FLAG_RPL1_26, OP_FLAG_RPL1_27, OP_FLAG_RPL1_28, OP_FLAG_RPL1_29, OP_FLAG_RPL1_30,
    //...
    OP_FLAG_RPL1_31,
#endif
#if LIBVC_ON
    OP_LIBPIC_ENABLE,
    OP_FLAG_QP_OFFSET_LIBPIC,
    OP_FLAG_QP_OFFSET_RLPIC,
    OP_FLAG_FNAME_LIBOUT,
    OP_FLAG_FNAME_LIBREC,
    OP_FLAG_FNAME_LIBDATA,
    OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC,
    OP_FLAG_MAX_FRM_NUM_WHEN_EXTRACT_LIBPIC,
#if IPPPCRR
#if LIB_PIC_UPDATE
    OP_LIBPIC_ADAPTIVE_UPDATE,
#else
    OP_FIRST_PIC_AS_LIBPIC,
#endif
#endif
#endif
    OP_TEMP_SUB_RATIO,
    OP_CTU_SZE,
    OP_MIN_CU_SIZE,
    OP_MAX_PART_RATIO,
    OP_MAX_SPLIT_TIMES,
    OP_MIN_QT_SIZE,
    OP_MAX_BT_SIZE,
    OP_MAX_EQT_SIZE,
    OP_MAX_DT_SIZE,
#if CALC_SSIM_UV
    OP_MSSSIM_UV,
#endif
    OP_FLAG_MAX
} OP_FLAGS;

static int op_flag[OP_FLAG_MAX] = {0};

static COM_ARGS_OPTION options[] =
{
    {
        COM_ARGS_NO_KEY, "config", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_CFG], op_fname_cfg,
        "file name of configuration"
    },
    {
        'i', "input", ARGS_TYPE_STRING|ARGS_TYPE_MANDATORY,
        &op_flag[OP_FLAG_FNAME_INP], op_fname_inp,
        "file name of input video"
    },
    {
        'o', "output", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_OUT], op_fname_out,
        "file name of output bitstream"
    },
    {
        'r', "recon", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_REC], op_fname_rec,
        "file name of reconstructed video"
    },
    {
        'w',  "width", ARGS_TYPE_INTEGER|ARGS_TYPE_MANDATORY,
        &op_flag[OP_FLAG_WIDTH_INP], &op_w,
        "pixel width of input video"
    },
    {
        'h',  "height", ARGS_TYPE_INTEGER|ARGS_TYPE_MANDATORY,
        &op_flag[OP_FLAG_HEIGHT_INP], &op_h,
        "pixel height of input video"
    },
    {
        'q',  "op_qp", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP], &op_qp,
        "QP value (-16~63 for 8bit input, 0~79 for 10bit input)"
    },
    {
        'z',  "frame_rate", ARGS_TYPE_INTEGER|ARGS_TYPE_MANDATORY,
        &op_flag[OP_FLAG_FPS], &op_fps,
        "frame rate (Hz)"
    },
    {
        'p',  "i_period", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_IPERIOD], &op_i_period,
        "I-picture period"
    },
    {
        'g',  "max_b_frames", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_B_FRAMES], &op_max_b_frames,
        "Number of maximum B frames (1,3,7,15)\n"
    },
    {
        'f',  "frames", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM], &op_max_frm_num,
        "maximum number of frames to be encoded"
    },
    {
        's',  "signature", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_USE_PIC_SIGN], &op_use_pic_signature,
        "embed picture signature (HASH) for conformance checking in decoding"
        "\t 0: disable\n"
        "\t 1: enable\n"
    },
    {
        'v',  "verbose", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_VERBOSE], &op_verbose,
        "verbose level\n"
        "\t 0: no message\n"
        "\t 1: frame-level messages (default)\n"
        "\t 2: all messages\n"
    },
    {
        'd',  "input_bit_depth", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_IN_BIT_DEPTH], &op_bit_depth_input,
        "input bitdepth (8(default), 10) "
    },
    {
        COM_ARGS_NO_KEY,  "internal_bit_depth", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_bit_depth_internal,
        "output bitdepth (8, 10)(default: same as input bitdpeth) "
    },
#if USE_IBC
    {
        COM_ARGS_NO_KEY,  "ibc", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_IBC], &op_enable_ibc,
        "use IBC feature. if not set, IBC feature is disabled"
    },
#if IBC_ABVR
    {
        COM_ARGS_NO_KEY,  "abvr", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_IBC_ABVR], &op_tool_abvr,
        "abvr on/off flag"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "ibc_search_range_x", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_SEARCH_RNG_X], &op_ibc_search_range_x,
        "set ibc search range in horizontal direction (default 64)"
    },

    {
        COM_ARGS_NO_KEY,  "ibc_search_range_y", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_SEARCH_RND_Y], &op_ibc_search_range_y,
        "set ibc search range in vertical direction (default 64)"
    },

    {
        COM_ARGS_NO_KEY,  "ibc_hash_search_flag", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_HASH_FLAG], &op_ibc_hash_search_flag,
        "use IBC hash based block matching search feature. if not set, it is disable"
    },

    {
        COM_ARGS_NO_KEY,  "ibc_hash_search_max_cand", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_HASH_SEARCH_MAX_CAND], &op_ibc_hash_search_max_cand,
        "Max candidates for hash based IBC search (default 64)"
    },

    {
        COM_ARGS_NO_KEY,  "ibc_hash_search_range_4smallblk", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_HASH_SEARCH_RANGE_4SMALLBLK], &op_ibc_hash_search_range_4smallblk,
        "Small block search range in IBC based search. (default 64)"
    },

    {
        COM_ARGS_NO_KEY,  "ibc_fast_method", ARGS_TYPE_INTEGER,
        &op_flag[OP_IBC_FAST_METHOD], &op_ibc_fast_method,
        "Fast methods for IBC\n"
        "\t 1: Buffer IBC block vector (current not support)\n"
        "\t 2: Adaptive search range (default)\n"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "disable_hgop", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_DISABLE_HGOP], &op_disable_hgop,
        "disable hierarchical GOP. if not set, hierarchical GOP is used"
    },
    {
        COM_ARGS_NO_KEY,  "skip_frames", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_SKIP_FRAMES], &op_skip_frames,
        "number of skipped frames before encoding. default 0"
    },
    {
        COM_ARGS_NO_KEY,  "qp_add_frm", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_ADD_QP_FRAME], &op_frame_qp_add,
        "one more qp are added after this number of frames, disable:0 (default)"
    },
    {
        COM_ARGS_NO_KEY, "pb_frame_qp_offset", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_PB_FRAME_QP_OFFSET], &op_pb_frame_qp_offset,
        "add fixed delta qp to p/b frames, for IPPP structure, default 0"
    },
    {
        COM_ARGS_NO_KEY,  "qp_offset_cb", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_CB], &op_qp_offset_cb,
        "qp offset for cb, disable:0 (default)"
    },
    {
        COM_ARGS_NO_KEY,  "qp_offset_cr", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_CR], &op_qp_offset_cr,
        "qp offset for cr, disable:0 (default)"
    },
    {
        COM_ARGS_NO_KEY,  "qp_offset_adp", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_ADP], &op_qp_offset_adp,
        "adaptive frame-level qp offset for cb and cr, disable:0, enable: 1(default)"
    },
    {
        COM_ARGS_NO_KEY,  "lcu_delta_qp", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_DELTA_QP], &op_delta_qp_flag,
        "Random qp for lcu, on/off flag"
    },
#if IPCM
    {
        COM_ARGS_NO_KEY,  "ipcm", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_IPCM], &op_tool_ipcm,
        "ipcm on/off flag"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "amvr", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_AMVR], &op_tool_amvr,
        "amvr on/off flag"
    },
#if MIPF
    {
        COM_ARGS_NO_KEY,  "mipf", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_MIPF], &op_tool_mipf,
        "Multiple intra prediction filter on/off flag"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "ipf", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_IPF], &op_tool_ipf,
        "Intra prediction filter on/off flag"
    },

    {
        COM_ARGS_NO_KEY, "wq_enable", ARGS_TYPE_INTEGER,
        &op_flag[OP_WQ_ENABLE], &op_wq_enable,
        "Weight Quant on/off, disable: 0(default), enable: 1"
    },
    {
        COM_ARGS_NO_KEY, "seq_wq_mode", ARGS_TYPE_INTEGER,
        &op_flag[OP_SEQ_WQ_MODE], &op_seq_wq_mode,
        "Seq Weight Quant (0: default, 1: User Define)"
    },
    {
        COM_ARGS_NO_KEY, "seq_wq_user", ARGS_TYPE_STRING,
        &op_flag[OP_SEQ_WQ_FILE], &op_seq_wq_user,
        "User Defined Seq WQ Matrix"
    },
    {
        COM_ARGS_NO_KEY, "pic_wq_data_idx", ARGS_TYPE_INTEGER,
        &op_flag[OP_PIC_WQ_DATA_IDX], &op_pic_wq_data_idx,
        "Pic level WQ data index, 0(default):refer to seq_header,  1:derived by WQ parameter,  2:load from pic_header"
    },
    {
        COM_ARGS_NO_KEY, "pic_wq_user", ARGS_TYPE_STRING,
        &op_flag[OP_PIC_WQ_FILE], &op_pic_wq_user,
        "User Defined Pic WQ Matrix"
    },
    {
        COM_ARGS_NO_KEY, "wq_param", ARGS_TYPE_INTEGER,
        &op_flag[OP_WQ_PARAM], &op_wq_param,
        "WQ Param (0=Default, 1=UnDetailed, 2=Detailed)"
    },
    {
        COM_ARGS_NO_KEY, "wq_model", ARGS_TYPE_INTEGER,
        &op_flag[OP_WQ_PARAM_MODEL], &op_wq_model,
        "WQ Model (0~2, default:0)"
    },
    {
        COM_ARGS_NO_KEY, "wq_param_detailed", ARGS_TYPE_STRING,
        &op_flag[OP_WQ_PARAM_DETAILED], &op_wq_param_detailed,
        "default:[64,49,53,58,58,64]"
    },
    {
        COM_ARGS_NO_KEY, "wq_param_undetailed", ARGS_TYPE_STRING,
        &op_flag[OP_WQ_PARAM_UNDETAILED], &op_wq_param_undetailed,
        "default:[67,71,71,80,80,106]"
    },
    {
        COM_ARGS_NO_KEY,  "hmvp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_HMVP], &op_tool_hmvp,
        "number of hmvp skip candidates (default: 8, disable: 0)"
    },
#if IBC_BVP
    {
        COM_ARGS_NO_KEY,  "hbvp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_HBVP], &op_tool_hbvp,
        "number of hbvp candidates (default: 12, disable: 0)"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "tscpm", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_TSCPM], &op_tool_tscpm,
        "tscpm on/off flag"
    },
#if PMC
    {
        COM_ARGS_NO_KEY,  "pmc", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_PMC], &op_tool_pmc,
        "pmc on/off flag"
    },
#endif
#if FIMC
    {
        COM_ARGS_NO_KEY,  "fimc", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_FIMC], &op_tool_fimc,
        "fimc on/off flag"
    },
#endif
#if USE_SP
    {
        COM_ARGS_NO_KEY,  "sp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SP], &op_tool_sp,
        "sp on/off flag"
    },
#endif
#if ENHANCE_TSPCM
    {
        COM_ARGS_NO_KEY,  "enhance_tscpm", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_ENHANCE_TSCPM], &op_tool_enhance_tscpm,
        "enhance_tscpm on/off flag"
    },
#endif
#if IPF_CHROMA
    {
        COM_ARGS_NO_KEY, "ipf_chroma", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_CHROMA_IPF], &op_tool_chroma_ipf,
    },
#endif
    {
        COM_ARGS_NO_KEY,  "umve", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_UMVE], &op_tool_umve,
        "umve on/off flag"
    },
#if UMVE_ENH
    {
        COM_ARGS_NO_KEY,  "umve_enh", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_UMVE_ENH], &op_tool_umve_enh,
        "umve_enh on/off flag"
    },
#endif
#if AWP
    {
        COM_ARGS_NO_KEY,  "awp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_AWP], &op_tool_awp,
        "awp on/off flag"
    },
#endif
#if AWP_MVR
    {
        COM_ARGS_NO_KEY,  "awp_mvr", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_AWP_MVR], &op_tool_awp_mvr,
        "awp mvr on/off flag"
    },
#endif
#if ETMVP
    {
        COM_ARGS_NO_KEY,  "etmvp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_ETMVP], &op_tool_etmvp,
        "etmvp on/off flag"
    },
#endif
#if SUB_TMVP
    {
        COM_ARGS_NO_KEY,  "sbtmvp", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SBTMVP], &op_tool_sbtmvp,
        "sbtmvp on/off flag"
    },
#endif
#if AFFINE_UMVE
    {
        COM_ARGS_NO_KEY,  "affine_umve", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_AFFINE_UMVE], &op_tool_affine_umve,
        "affine_umve on/off flag"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "emvr", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_EMVR], &op_tool_emvr,
        "emvr on/off flag"
    },
#if BIO
    {
        COM_ARGS_NO_KEY, "bio", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_BIO], &op_tool_bio,
        "bio on/off flag"
    },
#endif
#if BGC
    {
        COM_ARGS_NO_KEY, "bgc", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_BGC], &op_tool_bgc,
        "bgc on/off flag"
    },
#endif
#if DMVR
    {
        COM_ARGS_NO_KEY,  "dmvr", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_DMVR], &op_tool_dmvr,
        "dmvr on/off flag"
    },
#endif
#if INTERPF
    {
        COM_ARGS_NO_KEY,  "interpf", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_INTERPF], &op_tool_interpf,
        "interpf on/off flag"
    },
#endif
    {
        COM_ARGS_NO_KEY,  "affine", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_AFFINE], &op_tool_affine,
        "affine on/off flag"
    },
    {
        COM_ARGS_NO_KEY,  "smvd", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SMVD], &op_tool_smvd,
        "smvd on/off flag (on: 1, off: 0, default: 1)"
    },
    {
        COM_ARGS_NO_KEY,  "dt_intra", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_DT_INTRA], &op_tool_dt_intra,
        "dt_intra on/off flag (on: 1, off: 0, default: 1)"
    },
    {
        COM_ARGS_NO_KEY,  "pbt", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_PBT], &op_tool_pbt,
        "pbt on/off flag (on: 1, off: 0, default: 1)"
    },
#if SBT
    {
        COM_ARGS_NO_KEY, "sbt", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SBT], &op_tool_sbt,
        "sbt on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#if IST
    {
        COM_ARGS_NO_KEY, "ist", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_IST], &op_tool_ist,
        "ist on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#if ISTS
    {
        COM_ARGS_NO_KEY, "ists", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_ISTS], &op_tool_ists,
        "ists on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#if EST
    {
        COM_ARGS_NO_KEY, "est", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_EST], &op_tool_est,
        "est on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#if SRCC
    {
        COM_ARGS_NO_KEY, "srcc", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SRCC], &op_tool_srcc,
        "srcc on/off flag (on: 1, off: 0, default: 1)"
    },
#if EIPM
    {
        COM_ARGS_NO_KEY, "eipm", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_EIPM],&op_tool_eipm,
        "eipm on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#endif
#if MVAP
    {
        COM_ARGS_NO_KEY, "mvap", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_MVAP], &op_tool_mvap,
        "mvap on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
#if CABAC_MULTI_PROB
    {
        COM_ARGS_NO_KEY, "mcabac", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_MCABAC], &op_tool_mcabac,
        "mcabac on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
    {
        COM_ARGS_NO_KEY, "deblock", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_DEBLOCK], &op_tool_deblock,
        "deblock on/off flag (on: 1, off: 0, default: 1)"
    },
#if DBK_SCC
    {
        COM_ARGS_NO_KEY, "deblock_type", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_DEBLOCK_TYPE], &op_tool_deblock_type,
        "adaptive control of deblock type (on: 1, off: 0, default: 0)"
    },
#endif
    {
        COM_ARGS_NO_KEY, "sao", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_SAO], &op_tool_sao,
        "sao on/off flag (on: 1, off: 0, default: 1)"
    },
#if ESAO
    {
        COM_ARGS_NO_KEY, "esao", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_ESAO], &op_tool_esao,
        "esao on/off flag (on: 1, off: 0, default: 1)"
    },
#endif
    {
        COM_ARGS_NO_KEY, "alf", ARGS_TYPE_INTEGER,
        &op_flag[OP_TOOL_ALF], &op_tool_alf,
        "alf on/off flag (on: 1, off: 0, default: 1)"
    },
#if PHASE_2_PROFILE
    {
        COM_ARGS_NO_KEY,  "profile", ARGS_TYPE_INTEGER,
        &op_flag[OP_PROFILE], &op_profile,
        "profile value (phase1: 0x20/0x22, phase2: 0x30/0x32, default: 0x32)"
    },
#endif
#if PATCH
    {
        COM_ARGS_NO_KEY, "patch_stable", ARGS_TYPE_INTEGER,
        &op_flag[OP_PATCH_STABLE], &op_patch_stable,
        "stable_patch_flag(0:all the pic as the same patch size;1:each pic as the different patch size)"
    },
    {
        COM_ARGS_NO_KEY, "cross_patch_loopfilter", ARGS_TYPE_INTEGER,
        &op_flag[OP_CROSS_PATCH_LOOP_FILTER], &op_cross_patch_loop_filter,
        "loop_filter_across_patch_flag(1:cross;0:non cross)"
    },
    {
        COM_ARGS_NO_KEY, "patch_ref_colocated", ARGS_TYPE_INTEGER,
        &op_flag[OP_PATCH_REF_COLOCATED], &op_patch_ref_colocated,
        "if the MV out of the patch boundary,0:non use colocated;1:use colocated"
    },
    {
        COM_ARGS_NO_KEY, "patch_uniform", ARGS_TYPE_INTEGER,
        &op_flag[OP_PATCH_UNIFORM], &op_patch_uniform,
        "0:all the patch are in the same size;1:each patch in the different size"
    },
    {
        COM_ARGS_NO_KEY, "patch_width_in_lcu", ARGS_TYPE_INTEGER,
        &op_flag[OP_PATCH_WIDTH], &op_patch_width_in_lcu,
        "when uniform is 1,the nember of LCU in horizon in a patch"
    },
    {
        COM_ARGS_NO_KEY, "patch_height_in_lcu", ARGS_TYPE_INTEGER,
        &op_flag[OP_PATCH_HEIGHT], &op_patch_height_in_lcu,
        "when uniform is 1,the nember of LCU in vertical in a patch"
    },
#endif
#if HLS_RPL
    {
        COM_ARGS_NO_KEY,  "RPL0_0", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_0], &op_rpl0[0],
        "RPL0_0"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_1", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_1], &op_rpl0[1],
        "RPL0_1"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_2", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_2], &op_rpl0[2],
        "RPL0_2"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_3", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_3], &op_rpl0[3],
        "RPL0_3"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_4", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_4], &op_rpl0[4],
        "RPL0_4"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_5", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_5], &op_rpl0[5],
        "RPL0_5"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_6", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_6], &op_rpl0[6],
        "RPL0_6"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_7", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_7], &op_rpl0[7],
        "RPL0_7"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_8", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_8], &op_rpl0[8],
        "RPL0_8"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_9", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_9], &op_rpl0[9],
        "RPL0_9"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_10", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_10], &op_rpl0[10],
        "RPL0_10"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_11", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_11], &op_rpl0[11],
        "RPL0_11"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_12", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_12], &op_rpl0[12],
        "RPL0_12"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_13", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_13], &op_rpl0[13],
        "RPL0_13"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_14", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_14], &op_rpl0[14],
        "RPL0_14"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_15", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_15], &op_rpl0[15],
        "RPL0_15"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_16", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_16], &op_rpl0[16],
        "RPL0_16"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_17", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_17], &op_rpl0[17],
        "RPL0_17"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_18", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_18], &op_rpl0[18],
        "RPL0_18"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_19", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_19], &op_rpl0[19],
        "RPL0_19"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_20", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_20], &op_rpl0[20],
        "RPL0_20"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_21", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_21], &op_rpl0[21],
        "RPL0_21"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_22", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_22], &op_rpl0[22],
        "RPL0_22"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_23", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_23], &op_rpl0[23],
        "RPL0_23"
    },
    {
        COM_ARGS_NO_KEY,  "RPL0_24", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_24], &op_rpl0[24],
        "RPL0_24"
    },
            /*BUG_Fix*/                 ///@UpdatedBy:Chaos 
            {
        COM_ARGS_NO_KEY,  "RPL0_25", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_25], &op_rpl0[25],
        "RPL0_25"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_26", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_26], &op_rpl0[26],
        "RPL0_26"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_27", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_27], &op_rpl0[27],
        "RPL0_27"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_28", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_28], &op_rpl0[28],
        "RPL0_28"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_29", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_29], &op_rpl0[29],
        "RPL0_29"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_30", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_30], &op_rpl0[30],
        "RPL0_30"
            }, {
        COM_ARGS_NO_KEY,  "RPL0_31", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_31], &op_rpl0[31],
        "RPL0_31"
            },
    {
        COM_ARGS_NO_KEY,  "RPL1_0", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_0], &op_rpl1[0],
        "RPL1_0"
    },

    {
        COM_ARGS_NO_KEY,  "RPL1_1", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_1], &op_rpl1[1],
        "RPL1_1"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_2", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_2], &op_rpl1[2],
        "RPL1_2"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_3", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_3], &op_rpl1[3],
        "RPL1_3"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_4", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_4], &op_rpl1[4],
        "RPL1_4"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_5", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_5], &op_rpl1[5],
        "RPL1_5"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_6", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_6], &op_rpl1[6],
        "RPL1_6"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_7", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_7], &op_rpl1[7],
        "RPL1_7"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_8", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_8], &op_rpl1[8],
        "RPL1_8"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_9", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_9], &op_rpl1[9],
        "RPL1_9"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_10", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_10], &op_rpl1[10],
        "RPL1_10"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_11", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_11], &op_rpl1[11],
        "RPL1_11"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_12", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_12], &op_rpl1[12],
        "RPL1_12"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_13", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_13], &op_rpl1[13],
        "RPL1_13"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_14", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_14], &op_rpl1[14],
        "RPL1_14"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_15", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_15], &op_rpl1[15],
        "RPL1_15"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_16", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_16], &op_rpl1[16],
        "RPL1_16"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_17", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_17], &op_rpl1[17],
        "RPL1_17"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_18", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_18], &op_rpl1[18],
        "RPL1_18"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_19", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_19], &op_rpl1[19],
        "RPL1_19"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_20", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_20], &op_rpl1[20],
        "RPL1_20"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_21", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_21], &op_rpl1[21],
        "RPL1_21"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_22", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_22], &op_rpl1[22],
        "RPL1_22"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_23", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_23], &op_rpl1[23],
        "RPL1_23"
    },
    {
        COM_ARGS_NO_KEY,  "RPL1_24", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_24], &op_rpl1[24],
        "RPL1_24"
    }, 
            /*BUG_Fix*/   ///@UpdatedBy:Chaos 
        {
        COM_ARGS_NO_KEY,  "RPL1_25", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_25], &op_rpl1[25],
        "RPL1_25"
        },
            {
        COM_ARGS_NO_KEY,  "RPL1_26", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_26], &op_rpl1[26],
        "RPL1_26"
            },
            {
        COM_ARGS_NO_KEY,  "RPL1_27", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_27], &op_rpl1[27],
        "RPL1_27"
            },
            {
        COM_ARGS_NO_KEY,  "RPL1_28", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_28], &op_rpl1[28],
        "RPL1_28"
            },
            {
        COM_ARGS_NO_KEY,  "RPL1_29", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_29], &op_rpl1[29],
        "RPL1_29"
            },
            {
        COM_ARGS_NO_KEY,  "RPL1_30", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_30], &op_rpl1[30],
        "RPL1_30"
            },
            {
        COM_ARGS_NO_KEY,  "RPL1_31", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_31], &op_rpl1[31],
        "RPL1_31"
            },
#endif
#if LIBVC_ON
    {
        COM_ARGS_NO_KEY,  "libpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_LIBPIC_ENABLE], &op_tool_libpic,
        "libpic on/off flag"
    },
    {
        COM_ARGS_NO_KEY,  "qp_offset_libpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_LIBPIC], &op_qp_offset_libpic,
        "qp offset for libpic, default:-6"
    },
    {
        COM_ARGS_NO_KEY,  "qp_offset_rlpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_RLPIC], &op_qp_offset_rlpic,
        "qp offset for rlpic, default:0"
    },
    {
        COM_ARGS_NO_KEY,  "libout", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_LIBOUT], op_fname_libout,
        "file name of output libpic bitstream"
    },

    {
        COM_ARGS_NO_KEY,  "librec", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_LIBREC], op_fname_librec,
        "file name of reconstructed libpic"
    },
    {
        COM_ARGS_NO_KEY,  "libdependency", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_LIBDATA], op_fname_libdependency,
        "file name of library picture dependency information"
    },
    {
        COM_ARGS_NO_KEY,  "skip_frames_when_extract_libpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC], &op_skip_frames_when_extract_libpic,
        "number of skipped frames before extract libpic. default: the same as the number of skipped frames before encoding"
    },
    {
        COM_ARGS_NO_KEY,  "frames_when_extract_libpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM_WHEN_EXTRACT_LIBPIC], &op_max_frm_num_when_extract_libpic,
        "maximum number of frames when extract libpic. default: the same as the nmaximum number of frames to be encoded"
    },
#if IPPPCRR
#if LIB_PIC_UPDATE
    {
        COM_ARGS_NO_KEY,  "lib_pic_update", ARGS_TYPE_INTEGER,
        &op_flag[OP_LIBPIC_ADAPTIVE_UPDATE], &op_lib_pic_update,
        "adaptive library picture update"
    },
#else
    {
        COM_ARGS_NO_KEY,  "first_pic_as_libpic", ARGS_TYPE_INTEGER,
        &op_flag[OP_FIRST_PIC_AS_LIBPIC], &op_first_pic_as_libpic,
        "first_pic_as_libpic"
    },
#endif
#endif
#endif
    {
        COM_ARGS_NO_KEY, "TemporalSubsampleRatio", ARGS_TYPE_INTEGER,
        &op_flag[OP_TEMP_SUB_RATIO], &op_subSampleRatio,
        "Subsampling Ratio (default: 8 for AI, 1 for RA and LD)"
    },
    {
        COM_ARGS_NO_KEY, "ctu_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_CTU_SZE], &op_ctu_size,
        "ctu_size (default: 128; allowed values: 32, 64, 128)"
    },
    {
        COM_ARGS_NO_KEY, "min_cu_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_MIN_CU_SIZE], &op_min_cu_size,
        "min_cu_size (default: 4; allowed values: 4)"
    },
    {
        COM_ARGS_NO_KEY, "max_part_ratio", ARGS_TYPE_INTEGER,
        &op_flag[OP_MAX_PART_RATIO], &op_max_part_ratio,
        "max_part_ratio (default:8; allowed values: 8)"
    },
    {
        COM_ARGS_NO_KEY, "max_split_times", ARGS_TYPE_INTEGER,
        &op_flag[OP_MAX_SPLIT_TIMES], &op_max_split_times,
        "max_split_times (default: 6, allowed values: 6)"
    },
    {
        COM_ARGS_NO_KEY, "min_qt_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_MIN_QT_SIZE], &op_min_qt_size,
        "min_qt_size (default: 8, allowed values: 4, 8, 16, 32, 64, 128)"
    },
    {
        COM_ARGS_NO_KEY, "max_bt_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_MAX_BT_SIZE], &op_max_bt_size,
        "max_bt_size (default: 128, allowed values: 64, 128)"
    },
    {
        COM_ARGS_NO_KEY, "max_eqt_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_MAX_EQT_SIZE], &op_max_eqt_size,
        "max_eqt_size (default: 64, allowed values: 8, 16, 32, 64)"
    },
    {
        COM_ARGS_NO_KEY, "max_dt_size", ARGS_TYPE_INTEGER,
        &op_flag[OP_MAX_DT_SIZE], &op_max_dt_size,
        "max_dt_size (default: 64, allowed values: 16, 32, 64)"
    },
#if CALC_SSIM_UV
    {
        COM_ARGS_NO_KEY, "msssim_uv", ARGS_TYPE_INTEGER,
        &op_flag[OP_MSSSIM_UV], &op_msssim_uv,
        "msssim_uv (default: 0, allowed values: 0, 1)"
    },
#endif
    {0, "", COM_ARGS_VAL_TYPE_NONE, NULL, NULL, ""} /* termination */
};

#define NUM_ARG_OPTION   ((int)(sizeof(options)/sizeof(options[0]))-1)
static void print_usage(void)
{
    int i;
    char str[1024];
    v0print("< Usage >\n");
    for(i=0; i<NUM_ARG_OPTION; i++)
    {
        if(com_args_get_help(options, i, str) < 0) return;
        v0print("%s\n", str);
    }
}


/*
*����cfg�в������β���
*/
static int get_conf(ENC_PARAM * param)
{
    memset(param, 0, sizeof(ENC_PARAM));
#if PHASE_2_PROFILE
    param->profile = op_profile;
    if (param->profile != 0x22 && param->profile != 0x20
     && param->profile != 0x32 && param->profile != 0x30
        )
    {
        printf("unknow profile id: 0x%x\n", param->profile);
        assert(0);
    }
#endif
    param->horizontal_size = op_w;
    param->vertical_size   = op_h;
    param->pic_width  = (param->horizontal_size + MINI_SIZE - 1) / MINI_SIZE * MINI_SIZE;
    param->pic_height = (param->vertical_size   + MINI_SIZE - 1) / MINI_SIZE * MINI_SIZE;
    param->qp = op_qp;
    param->fps = op_fps;
    param->i_period = op_i_period;
    param->max_b_frames = op_max_b_frames;
    param->frame_qp_add = op_frame_qp_add;
    param->pb_frame_qp_offset = op_pb_frame_qp_offset;
    if( param->horizontal_size % MINI_SIZE != 0 )
    {
        printf( "Note: picture width in encoding process is padded to %d (input value %d)\n", param->pic_width, param->horizontal_size );
    }
    if( param->vertical_size % MINI_SIZE != 0 )
    {
        printf( "Note: picture height in encoding process is padded to %d (input value %d)\n", param->pic_height, param->vertical_size );
    }
    if(op_disable_hgop)
    {
        param->disable_hgop = 1;
    }
    param->bit_depth_input = op_bit_depth_input;
    if (op_bit_depth_internal == 0)
    {
        op_bit_depth_internal = op_bit_depth_input;
    }
    param->bit_depth_internal = op_bit_depth_internal;

    if (param->bit_depth_input > param->bit_depth_internal)
    {
        printf("Warning: Data precision may lose because input bit depth is higher than internal one !\n");
    }

#if PHASE_2_PROFILE
    if (param->profile == 0x20
     || param->profile == 0x30
        )
    {
        assert(param->bit_depth_internal == 8);
    }
    if (param->profile == 0x22
        || param->profile == 0x32
        )
    {
        assert(param->bit_depth_internal == 10);
    }
#endif
    assert(param->bit_depth_input == 8 || param->bit_depth_input == 10);
    assert(param->bit_depth_internal == 8 || param->bit_depth_internal == 10);
#if IPCM
    param->ipcm_enable_flag = op_tool_ipcm;
#endif
    param->amvr_enable_flag = op_tool_amvr;
    param->affine_enable_flag = op_tool_affine;
    param->smvd_enable_flag = op_tool_smvd;
    param->use_deblock = op_tool_deblock;
    param->num_of_hmvp_cand = op_tool_hmvp;
    param->ipf_flag = op_tool_ipf;
#if TSCPM
    param->tscpm_enable_flag = op_tool_tscpm;
#endif
    param->umve_enable_flag = op_tool_umve;
#if EXT_AMVR_HMVP
    if (param->amvr_enable_flag && param->num_of_hmvp_cand)
    {
        param->emvr_enable_flag = op_tool_emvr;
    }
    else
    {
        param->emvr_enable_flag = 0;
    }
#endif
#if DT_PARTITION
    param->dt_intra_enable_flag = op_tool_dt_intra;
#endif
    param->position_based_transform_enable_flag = op_tool_pbt;

    param->wq_enable = op_wq_enable;
    param->seq_wq_mode = op_seq_wq_mode;
    strcpy(param->seq_wq_user, op_seq_wq_user);

    param->pic_wq_data_idx = op_pic_wq_data_idx;
    strcpy(param->pic_wq_user, op_pic_wq_user);

    param->wq_param = op_wq_param;
    param->wq_model = op_wq_model;
    strcpy(param->wq_param_detailed, op_wq_param_detailed);
    strcpy(param->wq_param_undetailed, op_wq_param_undetailed);

#if PATCH
    param->patch_stable = op_patch_stable;
    param->cross_patch_loop_filter = op_cross_patch_loop_filter;
    param->patch_ref_colocated = op_patch_ref_colocated;

    if (param->patch_stable)
    {
        param->patch_uniform = op_patch_uniform;
        if (param->patch_uniform)
        {
            param->patch_width_in_lcu = op_patch_width_in_lcu;
            param->patch_height_in_lcu = op_patch_height_in_lcu;
        }
        else
        {
            printf("please set patch_uniform = 1\n");
            exit(-1);
        }
    }
#endif
    param->sample_adaptive_offset_enable_flag = op_tool_sao;
    param->adaptive_leveling_filter_enable_flag = op_tool_alf;
    param->secondary_transform_enable_flag = op_tool_secTrans;
#if LIBVC_ON
    param->library_picture_enable_flag = op_tool_libpic;
#else
    param->library_picture_enable_flag = 0;
#endif
    param->chroma_format = 1; //must be 1 in base profile 
    param->encoding_precision = (param->bit_depth_internal == 8) ? 1 : 2;
#if HLS_RPL
    for (int i = 0; i < MAX_NUM_RPLS && op_rpl0[i][0] != 0; ++i)
    {
        char* slice_type = strtok(op_rpl0[i], "|");
        if (strcmp(slice_type, "B") == 0)
        {
            param->rpls_l0[i].slice_type = SLICE_B;
        }
        else if (strcmp(slice_type, "P") == 0)
        {
            param->rpls_l0[i].slice_type = SLICE_P;
        }
        else if (strcmp(slice_type, "I") == 0)
        {
            param->rpls_l0[i].slice_type = SLICE_I;
        }
        else
        {
            printf("\n wrong slice type\n");
            assert(0);
        }
        param->rpls_l0[i].poc = atoi(strtok(NULL, "|"));
        param->rpls_l0[i].tid = atoi(strtok(NULL, "|"));
        param->rpls_l0[i].ref_pic_active_num = atoi(strtok(NULL, "|"));
#if LIBVC_ON
        param->rpls_l0[i].reference_to_library_enable_flag = 0;
#endif
        int j = 0;
        int k = 0;
        int ddoi = 0;
        do
        {
            char* val = strtok(NULL, "|");
            if (!val)
                break;
#if LIBVC_ON
            if (atoi(val) == 9999)
            {
                ddoi = 1;
                continue;
            }
            if (ddoi == 1)
            {
                param->rpls_l0[i].ref_pics_ddoi[k++] = atoi(val);
            }
            else
            {
                param->rpls_l0[i].ref_pics[j] = atoi(val);
                param->rpls_l0[i].library_index_flag[j] = 0;
                j++;
            }
#else
            if (atoi(val) == 9999)
            {
                ddoi = 1;
                continue;
            }
            if (ddoi == 1)
            {
                param->rpls_l0[i].ref_pics_ddoi[k++] = atoi(val);
            }
            else
            {
                param->rpls_l0[i].ref_pics[j++] = atoi(val);
            }
#endif
        }
        while (1);

        param->rpls_l0[i].ref_pic_num = j;
        ++param->rpls_l0_cfg_num;
    }

    for (int i = 0; i < MAX_NUM_RPLS && op_rpl1[i][0] != 0; ++i)
    {
        char* slice_type = strtok(op_rpl1[i], "|");
        if (strcmp(slice_type, "B") == 0)
        {
            param->rpls_l1[i].slice_type = SLICE_B;
        }
        else if (strcmp(slice_type, "P") == 0)
        {
            param->rpls_l1[i].slice_type = SLICE_P;
        }
        else if (strcmp(slice_type, "I") == 0)
        {
            param->rpls_l1[i].slice_type = SLICE_I;
        }
        else
        {
            printf("\n wrong slice type\n");
            assert(0);
        }
        param->rpls_l1[i].poc = atoi(strtok(NULL, "|"));
        param->rpls_l1[i].tid = atoi(strtok(NULL, "|"));
        param->rpls_l1[i].ref_pic_active_num = atoi(strtok(NULL, "|"));
#if LIBVC_ON
        param->rpls_l1[i].reference_to_library_enable_flag = 0;
#endif
        int j = 0;
        int k = 0;
        int ddoi = 0;
        do
        {
            char* val = strtok(NULL, "|");
            if (!val)
                break;
#if LIBVC_ON
            if (atoi(val) == 9999)
            {
                ddoi = 1;
                continue;
            }
            if (ddoi == 1)
            {
                param->rpls_l1[i].ref_pics_ddoi[k++] = atoi(val);
            }
            else
            {
                param->rpls_l1[i].ref_pics[j] = atoi(val);
                param->rpls_l1[i].library_index_flag[j] = 0;
                j++;
            }
#else
            if (atoi(val) == 9999)
            {
                ddoi = 1;
                continue;
            }
            if (ddoi == 1)
            {
                param->rpls_l1[i].ref_pics_ddoi[k++] = atoi(val);
            }
            else
            {
                param->rpls_l1[i].ref_pics[j++] = atoi(val);
            }
#endif

        }
        while (1);

        param->rpls_l1[i].ref_pic_num = j;
        ++param->rpls_l1_cfg_num;
    }
#endif
#if LIBVC_ON
    param->qp_offset_libpic = op_qp_offset_libpic;
    param->qp_offset_rlpic = op_qp_offset_rlpic;
#endif
    param->sub_sample_ratio = op_subSampleRatio;
    param->frames_to_be_encoded = (op_max_frm_num + op_subSampleRatio - 1) / op_subSampleRatio;
    param->use_pic_sign = op_use_pic_signature;
    param->ctu_size = op_ctu_size;
    param->min_cu_size = op_min_cu_size;
    param->max_part_ratio = op_max_part_ratio;
    param->max_split_times = op_max_split_times;
    param->min_qt_size = op_min_qt_size;
    param->max_bt_size = op_max_bt_size;
    param->max_eqt_size = op_max_eqt_size;
    param->max_dt_size = op_max_dt_size;
    //verify parameters allowed in Profile
    assert(param->ctu_size == 32 || param->ctu_size == 64 || param->ctu_size == 128);
    assert(param->min_cu_size == 4);
    assert(param->max_part_ratio == 8);
    assert(param->max_split_times == 6);
    assert(param->min_qt_size == 4 || param->min_qt_size == 8 || param->min_qt_size == 16 || param->min_qt_size == 32 || param->min_qt_size == 64 || param->min_qt_size == 128);
    assert(param->max_bt_size == 64 || param->max_bt_size == 128);
    assert(param->max_eqt_size == 8 || param->max_eqt_size == 16 || param->max_eqt_size == 32 || param->max_eqt_size == 64);
    assert(param->max_dt_size == 16 || param->max_dt_size == 32 || param->max_dt_size == 64);

    param->qp_offset_cb = op_qp_offset_cb;
    param->qp_offset_cr = op_qp_offset_cr;

    param->delta_qp_flag = op_delta_qp_flag;
    param->qp_offset_adp = op_qp_offset_adp;

#if PHASE_2_PROFILE
    if (param->profile == 0x30 || param->profile == 0x32)
    {
#endif
#if USE_IBC
        if (op_enable_ibc)
        {
            param->use_ibc_flag = op_enable_ibc;
        }
        if (param->use_ibc_flag)
        {
            param->ibc_search_range_x = op_ibc_search_range_x;
            param->ibc_search_range_y = op_ibc_search_range_y;
            param->ibc_hash_search_flag = op_ibc_hash_search_flag;
            param->ibc_hash_search_max_cand = op_ibc_hash_search_max_cand;
            param->ibc_hash_search_range_4smallblk = op_ibc_hash_search_range_4smallblk;
            param->ibc_fast_method = op_ibc_fast_method;
        }
#endif
#if IBC_ABVR
        param->abvr_enable_flag = op_tool_abvr;
#endif
#if BIO
        param->bio_enable_flag = op_tool_bio;
#endif
#if BGC
        param->bgc_enable_flag = op_tool_bgc;
#endif
#if DMVR
        param->dmvr_enable_flag = op_tool_dmvr;
#endif
#if INTERPF
        param->interpf_enable_flag = op_tool_interpf;
#endif
#if IBC_BVP
        param->num_of_hbvp_cand = op_tool_hbvp;
#endif
#if MIPF
        param->mipf_flag = op_tool_mipf;
#endif
#if TSCPM && ENHANCE_TSPCM
        param->enhance_tscpm_enable_flag = param->tscpm_enable_flag && op_tool_enhance_tscpm;
#endif
#if IPF_CHROMA
        param->chroma_ipf_enable_flag = param->ipf_flag && op_tool_chroma_ipf;
#endif
#if PMC
        param->pmc_enable_flag = op_tool_pmc;
#endif
#if FIMC
        param->fimc_enable_flag = op_tool_fimc;
#endif
#if AWP
        param->awp_enable_flag = op_tool_awp;
#endif
#if AWP_MVR
        param->awp_mvr_enable_flag = param->awp_enable_flag && op_tool_awp_mvr;
#endif
#if ETMVP
        param->etmvp_enable_flag = op_tool_etmvp;
#endif
#if SUB_TMVP
        param->sbtmvp_enable_flag = op_tool_sbtmvp;
#endif
#if AFFINE_UMVE
        param->affine_umve_enable_flag = op_tool_affine_umve;
#endif
#if UMVE_ENH
        param->umve_enh_enable_flag = param->umve_enable_flag && op_tool_umve_enh;
#endif
#if IST
        param->ist_enable_flag = op_tool_ist;
#endif
#if ISTS
        param->ists_enable_flag = op_tool_ists;
#endif
#if SRCC
        param->srcc_enable_flag = op_tool_srcc;
#endif
#if CABAC_MULTI_PROB
        param->mcabac_enable_flag = op_tool_mcabac;
#endif
#if EIPM
        param->eipm_enable_flag = op_tool_eipm;
#endif
#if MVAP
        param->mvap_enable_flag = op_tool_mvap;
#endif
#if SBT
        param->sbt_enable_flag = op_tool_sbt;
#endif
#if EST
        param->est_enable_flag = param->secondary_transform_enable_flag ? op_tool_est : 0;
#endif
#if DBK_SCC
        param->loop_filter_type_enable_flag = param->use_deblock && op_tool_deblock_type;
#endif
#if USE_SP
        param->sp_enable_flag = op_tool_sp && op_enable_ibc;
#endif
#if ESAO
        param->esao_enable_flag = op_tool_esao;
        if (param->esao_enable_flag)
        {
            param->sample_adaptive_offset_enable_flag = 0;
        }
#endif 
#if PHASE_2_PROFILE
    }
#endif // end of PHASE_2_PROFILE

    return 0;
}

static int set_extra_config(ENC id)
{
    ENC_CTX *ctx = (ENC_CTX *)id;
    ctx->param.use_pic_sign = op_use_pic_signature;
    return 0;
}
static void print_stat_init(void)
{
    if(op_verbose < VERBOSE_FRAME) return;
    print("-------------------------------------------------------------------------------\n");
    print("  Input YUV file           : %s \n", op_fname_inp);
    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        print("  Output bitstream         : %s \n", op_fname_out);
    }
    if(op_flag[OP_FLAG_FNAME_REC])
    {
        print("  Output YUV file          : %s \n", op_fname_rec);
    }
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        print("------------------------------------------------------------------------------------------------------------------------\n");
        print(" POC       QP   PSNR-Y    PSNR-U    PSNR-V    Bits      EncT(ms)   MS-SSIM-Y   MS-SSIM-U   MS-SSIM-V   Ref. List\n");
        print("------------------------------------------------------------------------------------------------------------------------\n");
        print_flush(stdout);
    }
    else
    {
#endif
        print("--------------------------------------------------------------------------------------\n");
        print(" POC       QP   PSNR-Y    PSNR-U    PSNR-V    Bits      EncT(ms)   MS-SSIM   Ref. List\n");
        print("--------------------------------------------------------------------------------------\n");
        print_flush(stdout);
#if CALC_SSIM_UV
    }
#endif
}

static void print_config(ENC id, ENC_PARAM param)
{
    print("HPM version : %s -------------------------------------------------------------------------------\n", HPM_VERSION);
    print("< Configurations >\n");
    if (op_flag[OP_FLAG_FNAME_CFG])
    {
        print("\tconfig file name     : %s\n", op_fname_cfg);
    }
#if PHASE_2_PROFILE
    print("\tprofile                  : 0x%x\n", param.profile);
#endif
    print("\twidth                    : %d\n", param.pic_width);
    print("\theight                   : %d\n", param.pic_height);
    print("\tFPS                      : %d\n", param.fps);
    print("\tintra picture period     : %d\n", param.i_period);
    print("\tinput bit depth          : %d\n", param.bit_depth_input);
    print("\tinternal bit depth       : %d\n", param.bit_depth_internal);
    print("\tQP                       : %d\n", param.qp);
    print("\tframes                   : %d\n", op_max_frm_num);
    print("\thierarchical GOP         : %s\n", !param.disable_hgop ? "enabled" : "disabled");
    print("\tPatch flag               : %d\n", param.patch_width_in_lcu | param.patch_height_in_lcu);
    if (param.patch_width_in_lcu | param.patch_height_in_lcu)
    {
        if (param.patch_width_in_lcu)
        {
            print("\tPatch width in lcu       : %d\n", param.patch_width_in_lcu);
        }
        else
        {
            print("\tPatch width in lcu       : %s\n", "same as picture");
        }

        if (param.patch_height_in_lcu)
        {
            print("\tPatch height in lcu      : %d\n", param.patch_height_in_lcu);
        }
        else
        {
            print("\tPatch height in lcu      : %s\n", "same as picture");
        }
    }

    print("\tsub_sample_ratio         : %d\n", param.sub_sample_ratio);
    print("\tdelta_qp flag            : %d\n", param.delta_qp_flag);
    print("\tqp_offset_cb             : %d\n", param.qp_offset_cb);
    print("\tqp_offset_cr             : %d\n", param.qp_offset_cr);
    print("\tqp_offset_adp            : %d\n", param.qp_offset_adp);
    print("\tqp_offset_libpic         : %d\n", param.qp_offset_libpic);
    print("\tseq_ref2_lib_enable      : %d\n", param.library_picture_enable_flag);

    //printf split configure
    printf("\nCU split CFG: ");
    printf("\n\tctu_size:        %d", param.ctu_size);
    printf("\n\tmin_cu_size:     %d", param.min_cu_size);
    printf("\n\tmax_part_ratio:  %d", param.max_part_ratio);
    printf("\n\tmax_split_times: %d", param.max_split_times);
    printf("\n\tmin_qt_size:     %d", param.min_qt_size);
    printf("\n\tmax_bt_size:     %d", param.max_bt_size);
    printf("\n\tmax_eqt_size:    %d", param.max_eqt_size);
    printf("\n\tmax_dt_size:     %d", param.max_dt_size);
    printf("\n");

    // printf tool configurations
    printf("\nTool CFG:\n");
    //loop filter
    printf("DEBLOCK: %d, ", param.use_deblock);
#if DBK_SCC
    printf("DEBLOCK_TYPE: %d, ", param.loop_filter_type_enable_flag);
#endif
    printf("CROSS_PATCH_LP: %d, ", param.cross_patch_loop_filter);
    printf("SAO: %d, ", param.sample_adaptive_offset_enable_flag);
    printf("ALF: %d, ", param.adaptive_leveling_filter_enable_flag);
#if ESAO
    printf("ESAO: %d, ", param.esao_enable_flag);
#endif
    printf("\n");

    //inter
    printf("AMVR: %d, ", param.amvr_enable_flag);
    printf("HMVP_NUM: %d, ", param.num_of_hmvp_cand);
#if IBC_BVP
    printf("HBVP_NUM: %d, ", param.num_of_hbvp_cand);
#endif
    printf("AFFINE: %d, ", param.affine_enable_flag);
#if SMVD
    printf("SMVD: %d, ", param.smvd_enable_flag);
#endif
    printf("UMVE: %d, ", param.umve_enable_flag);
#if UMVE_ENH
    printf("UMVE_ENH: %d, ", param.umve_enh_enable_flag);
#endif
#if AWP
    printf("AWP: %d, ", param.awp_enable_flag);
#endif
#if AWP_MVR
    printf("AWP MVR: %d, ", param.awp_mvr_enable_flag);
#endif
#if ETMVP
    printf("ETMVP: %d, ", param.etmvp_enable_flag);
#endif
#if SUB_TMVP
    printf("SBTMVP: %d, ", param.sbtmvp_enable_flag);
#endif
#if EXT_AMVR_HMVP
    printf("EMVR: %d, ", param.emvr_enable_flag);
#endif
#if BIO
    printf("BIO: %d, ", param.bio_enable_flag);
#endif
#if BGC
    printf("BGC: %d, ", param.bgc_enable_flag);
#endif
#if MVAP
    printf("MVAP: %d, ", param.mvap_enable_flag);
#endif
#if AFFINE_UMVE
    printf("AFFINE_UMVE: %d, ", param.affine_umve_enable_flag);
#endif
    printf("\n");

    //intra
#if IPCM
    printf("IPCM: %d, ", param.ipcm_enable_flag);
#endif
#if TSCPM
    printf("TSCPM: %d, ", param.tscpm_enable_flag);
#if ENHANCE_TSPCM
    printf("Enhance_TSCPM: %d, ", param.enhance_tscpm_enable_flag);
#endif
#endif
#if IPF_CHROMA
    printf("IPF_CHROMA: %d, ", param.chroma_ipf_enable_flag);
#endif
#if PMC
    printf("PMC: %d, ", param.pmc_enable_flag);
#endif
#if FIMC
    printf("FIMC: %d, ", param.fimc_enable_flag);
#endif
#if MIPF
    printf("MIPF: %d, ", param.mipf_flag);
#endif
    printf("IPF: %d, ", param.ipf_flag);
#if USE_IBC
    printf("IBC: %d, ", param.use_ibc_flag);
#endif
#if IBC_ABVR
    printf("ABVR: %d, ", param.abvr_enable_flag);
#endif
#if USE_SP
    printf("SP: %d, ", param.sp_enable_flag);
#endif
    printf("\n");

    //transform & quant
    printf("PBT: %d, ", param.position_based_transform_enable_flag);
    printf("SecondaryTr: %d, ", param.secondary_transform_enable_flag);
#if SBT
    printf( "SBT: %d, ", param.sbt_enable_flag );
#endif
    //IST
#if IST
    printf("IST: %d, ", param.ist_enable_flag);
#endif
    //ISTS
#if ISTS
    printf("ISTS: %d, ", param.ists_enable_flag);
#endif
    //EST
#if EST
    printf("EST: %d, ", param.est_enable_flag);
#endif
    printf("WeightedQuant: %d, ", param.wq_enable);
    printf("\n");

    //DT
#if DT_PARTITION
    printf("INTRA_DT: %d, ", param.dt_intra_enable_flag);
    printf("MaxDTSize: %d, ", param.max_dt_size);
#endif
    printf("\n");

#if DMVR
    printf("DMVR: %d, ", param.dmvr_enable_flag );
#endif
#if INTERPF
    printf("INTERPF: %d, ", param.interpf_enable_flag );
#endif
#if SRCC
    //coefficient coding
    printf("SRCC: %d, ", param.srcc_enable_flag);
    printf("\n");
#endif
#if EIPM
    printf("EIPM: %d, ", param.eipm_enable_flag);
    printf("\n");
#endif
#if CABAC_MULTI_PROB
    printf("MULTI_CABAC: %d, ", param.mcabac_enable_flag);
    printf("\n");
#endif
    print_flush(stdout);
}

static void calc_psnr(COM_IMGB * org, COM_IMGB * rec, double psnr[3], int bit_depth)
{
    double sum[3], mse[3];
    short *o, *r;
    int i, j, k;
#if PSNR_1020
    int peak_val = (bit_depth == 8) ? 255 : 1020;
#else
    int peak_val = (bit_depth == 8) ? 255 : 1023;
#endif
    for(i=0; i<org->np; i++)
    {
        o       = (short*)org->addr_plane[i];
        r       = (short*)rec->addr_plane[i];
        sum[i] = 0;
        for(j=0; j<org->height[i]; j++)
        {
            for(k=0; k<org->width[i]; k++)
            {
                sum[i] += (o[k] - r[k]) * (o[k] - r[k]);
            }
            o = (short*)((unsigned char *)o + org->stride[i]);
            r = (short*)((unsigned char *)r + rec->stride[i]);
        }
        mse[i] = sum[i] / (org->width[i] * org->height[i]);
        psnr[i] = (mse[i]==0.0) ? 100. : fabs( 10*log10(((peak_val *peak_val)/mse[i])) );
    }
}

#if CALC_SSIM
const double gaussian_filter[11][11] =
{
    {0.000001,0.000008,0.000037,0.000112,0.000219,0.000274,0.000219,0.000112,0.000037,0.000008,0.000001},
    {0.000008,0.000058,0.000274,0.000831,0.001619,0.002021,0.001619,0.000831,0.000274,0.000058,0.000008},
    {0.000037,0.000274,0.001296,0.003937,0.007668,0.009577,0.007668,0.003937,0.001296,0.000274,0.000037},
    {0.000112,0.000831,0.003937,0.011960,0.023294,0.029091,0.023294,0.011960,0.003937,0.000831,0.000112},
    {0.000219,0.001619,0.007668,0.023294,0.045371,0.056662,0.045371,0.023294,0.007668,0.001619,0.000219},
    {0.000274,0.002021,0.009577,0.029091,0.056662,0.070762,0.056662,0.029091,0.009577,0.002021,0.000274},
    {0.000219,0.001619,0.007668,0.023294,0.045371,0.056662,0.045371,0.023294,0.007668,0.001619,0.000219},
    {0.000112,0.000831,0.003937,0.011960,0.023294,0.029091,0.023294,0.011960,0.003937,0.000831,0.000112},
    {0.000037,0.000274,0.001296,0.003937,0.007668,0.009577,0.007668,0.003937,0.001296,0.000274,0.000037},
    {0.000008,0.000058,0.000274,0.000831,0.001619,0.002021,0.001619,0.000831,0.000274,0.000058,0.000008},
    {0.000001,0.000008,0.000037,0.000112,0.000219,0.000274,0.000219,0.000112,0.000037,0.000008,0.000001}
};

void cal_ssim( int factor, double rad_struct[], int width_s1, int height_s1, short* org_curr_scale, short* rec_curr_scale, int bit_depth )
{
    int width, height, stride;
    const double K1 = 0.01;
    const double K2 = 0.03;
    const int    peak = (1<<bit_depth) - 1; //255 for 8-bit, 1023 for 10-bit
    const double C1 = K1 * K1 * peak * peak;
    const double C2 = K2 * K2 * peak * peak;
    double tmp_luma, tmp_simi, loc_mean_ref, loc_mean_rec, loc_var_ref,
           loc_var_rec, loc_covar, num1, num2, den1, den2;
    int num_win;
    int win_width, win_height;
    short *org, *rec, *org_pel, *rec_pel;
    int i, j, x, y;
    width          = stride = width_s1 / factor;
    height         = height_s1 / factor;
    win_width       = WIN_SIZE<width?  WIN_SIZE:width;
    win_height      = WIN_SIZE<height? WIN_SIZE:height;
    org            = org_curr_scale;
    rec            = rec_curr_scale;
    org_pel         = org;
    rec_pel         = rec;
    num_win         = (height - win_height + 1)*(width - win_width + 1);
    for ( j=0; j<=height-win_height; j++ )
    {
        for ( i=0; i<=width-win_width; i++ )
        {
            loc_mean_ref = 0;
            loc_mean_rec = 0;
            loc_var_ref  = 0;
            loc_var_rec  = 0;
            loc_covar   = 0;
            org_pel     = org + i + width*j;
            rec_pel     = rec + i + width*j;
            for ( y=0; y<win_height; y++ )
            {
                for ( x=0; x<win_width; x++ )
                {
                    loc_mean_ref    += org_pel[x]*gaussian_filter[y][x];
                    loc_mean_rec    += rec_pel[x]*gaussian_filter[y][x];
                    loc_var_ref     += org_pel[x]*gaussian_filter[y][x] * org_pel[x];
                    loc_var_rec     += rec_pel[x]*gaussian_filter[y][x] * rec_pel[x];
                    loc_covar      += org_pel[x]*gaussian_filter[y][x] * rec_pel[x];
                }
                org_pel += width;
                rec_pel += width;
            }
            loc_var_ref  =  (loc_var_ref  -  loc_mean_ref * loc_mean_ref);
            loc_var_rec  =  (loc_var_rec  -  loc_mean_rec * loc_mean_rec) ;
            loc_covar   =  (loc_covar   -  loc_mean_ref * loc_mean_rec) ;
            num1        =  2.0 * loc_mean_ref * loc_mean_rec + C1;
            num2        =  2.0 * loc_covar                 + C2;
            den1        =  loc_mean_ref * loc_mean_ref + loc_mean_rec * loc_mean_rec + C1;
            den2        =  loc_var_ref                + loc_var_rec                + C2;
            tmp_luma                        =  num1 / den1;
            tmp_simi                        =  num2 / den2;
            rad_struct[LUMA_COMPONENT]       += tmp_luma;
            rad_struct[SIMILARITY_COMPONENT] += tmp_simi;
            rad_struct[SSIM]                 += tmp_luma * tmp_simi;
        }
    }
    rad_struct[LUMA_COMPONENT]           /= (double) num_win;
    rad_struct[SIMILARITY_COMPONENT]     /= (double) num_win;
    rad_struct[SSIM]                     /= (double) num_win;
}

void calc_ssim_scale( double* ms_ssim, int scale, int width_s1, int height_s1, short* org_last_scale, short* rec_last_scale, int bit_depth) //calc ssim of each scale
{
    int width, height;
    int factor = 1 << ( scale-1 );
    int pos[4];
    int x, y;
    short* org_curr_scale = NULL;
    short* rec_curr_scale = NULL;
    short *o, *r;  //pointers to org and rec
    short *od,*rd; //pointers downsampled org and rec
    double struct_y[3] = {0.0, 0.0, 0.0};
    width  = width_s1 >> (scale-1);
    height = height_s1 >> (scale-1);
    if ( scale == 1 )
    {
        org_curr_scale = org_last_scale;
        rec_curr_scale = rec_last_scale;
    }
    else
    {
        org_curr_scale = (short*)malloc(width*height*sizeof(short));
        rec_curr_scale = (short*)malloc(width*height*sizeof(short));
        o = org_last_scale;
        r = rec_last_scale;
        od= org_curr_scale;
        rd= rec_curr_scale;
        /* Downsample */
        // 2x2 low-pass filter in ms-ssim Matlab code by Wang Zhou.
        for ( y=0; y<height; ++y )
        {
            int offset_last = (y<<1)*width_s1;
            int offset     =  y    *width;
            for ( x=0; x<width; ++x )
            {
                pos[0] = offset_last + (x<<1);
                pos[1] = pos[0] + 1;
                pos[2] = pos[0] + width_s1;
                pos[3] = pos[2] + 1;
                od[offset+x] = (o[pos[0]] + o[pos[1]] + o[pos[2]] + o[pos[3]] + 2) >> 2;
                rd[offset+x] = (r[pos[0]] + r[pos[1]] + r[pos[2]] + r[pos[3]] + 2) >> 2;
            }
        }
        //replace PicLastScale with down-sampled version
        memset(org_last_scale, 0, width_s1*height_s1*sizeof(short));
        memset(rec_last_scale, 0, width_s1*height_s1*sizeof(short));
        for( y=0; y<height; ++y )
        {
            int offset_pic_last_scale = y*width_s1;
            int offset_pic_down      = y*width;
            memcpy(org_last_scale + offset_pic_last_scale, od + offset_pic_down, width*sizeof(short));
            memcpy(rec_last_scale + offset_pic_last_scale, rd + offset_pic_down, width*sizeof(short));
        }
    }
    /* Y */
    cal_ssim( factor, struct_y, width_s1, height_s1, org_curr_scale, rec_curr_scale, bit_depth );
    if ( scale < MAX_SCALE )
        *ms_ssim = struct_y[SIMILARITY_COMPONENT];
    else
        *ms_ssim = struct_y[LUMA_COMPONENT]*struct_y[SIMILARITY_COMPONENT];
    if(scale>1)
    {
        if(org_curr_scale)
            free(org_curr_scale);
        if(rec_curr_scale)
            free(rec_curr_scale);
    }
}

static void find_ms_ssim(COM_IMGB *org, COM_IMGB *rec, double ms_ssim[3], int bit_depth)
{
    int width, height;
    short* org_last_scale[3];
    short* rec_last_scale[3];
    int i, j, k;
    short* o = NULL;
    short* r = NULL;
    short* o_dst = NULL;
    short* r_dst = NULL;
    int channel = 1;
    for (i = 0; i<3; i++)
    {
        ms_ssim[i] = 1.0;
    }
#if CALC_SSIM_UV
    if( op_msssim_uv )
    {
      channel = 3;
    }
#endif 
    for (k = 0; k < channel; k++)
    {
        width = org->width[k];
        height = org->height[k];
        org_last_scale[k] = (short*)malloc(width*height * sizeof(short));
        rec_last_scale[k] = (short*)malloc(width*height * sizeof(short));
        o = (short*)org->addr_plane[k];
        r = (short*)rec->addr_plane[k];
        o_dst = org_last_scale[k];
        r_dst = rec_last_scale[k];
        for (j = 0; j < height; j++)
        {
            memcpy(o_dst, o, width * sizeof(short));
            memcpy(r_dst, r, width * sizeof(short));
            o += org->stride[k] >> 1; //because s[0] is in unit of byte and 1 short = 2 bytes
            r += rec->stride[k] >> 1;
            o_dst += width;
            r_dst += width;
        }

        for (i = 1; i <= MAX_SCALE; i++)
        {
            double tmp_ms_ssim;
            calc_ssim_scale(&tmp_ms_ssim, i, width, height, org_last_scale[k], rec_last_scale[k], bit_depth);
            ms_ssim[k] *= pow(tmp_ms_ssim, (double)exponent[i - 1]);
        }
    }

    for (k = 0; k < channel; k++)
    {
        if (org_last_scale[k])
            free(org_last_scale[k]);
        if (rec_last_scale[k])
            free(rec_last_scale[k]);
    }
}
#endif

static int imgb_list_alloc(IMGB_LIST *list, int width, int height, int horizontal_size, int vertical_size, int bit_depth)
{
    int i;
    memset(list, 0, sizeof(IMGB_LIST)*MAX_BUMP_FRM_CNT);
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        list[i].imgb =  imgb_alloc(width, height, COM_COLORSPACE_YUV420, bit_depth);
        if(list[i].imgb == NULL) goto ERR;
        list[i].imgb->horizontal_size = horizontal_size;
        list[i].imgb->vertical_size   = vertical_size;
        if( horizontal_size % 2 == 1 || vertical_size % 2 == 1 )
        {
            printf( "\n[Error] width and height of a YUV4:2:0 input video shall be at least a multiple of 2" );
            goto ERR;
        }
    }
    return 0;
ERR:
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].imgb)
        {
            imgb_free(list[i].imgb);
            list[i].imgb = NULL;
        }
    }
    return -1;
}

static void imgb_list_free(IMGB_LIST *list)
{
    int i;
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].imgb)
        {
            imgb_free(list[i].imgb);
            list[i].imgb = NULL;
        }
    }
}

static IMGB_LIST *store_rec_img(IMGB_LIST *list, COM_IMGB *imgb, COM_MTIME ts, int bit_depth)
{
    int i;

    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].used == 0)
        {
            assert(list[i].imgb->cs == imgb->cs);
            imgb_cpy_conv_rec(list[i].imgb, imgb, bit_depth);
            list[i].used = 1;
            list[i].ts = ts;
            return &list[i];
        }
    }
    return NULL;
}

static IMGB_LIST *imgb_list_get_empty(IMGB_LIST *list)
{
    int i;
    /* store original imgb for PSNR */
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].used == 0)
        {
            return &list[i];
        }
    }
    return NULL;
}

static void imgb_list_make_used(IMGB_LIST *list, COM_MTIME ts)
{
    list->used = 1;
    list->ts = list->imgb->ts[0] = ts;
}

static int cal_psnr(IMGB_LIST * imgblist_inp, COM_IMGB * imgb_rec, COM_MTIME ts, double psnr[3]
#if CALC_SSIM
                    , double ms_ssim[3]
#endif
                   )
{
    int            i;
    COM_IMGB     *imgb_t = NULL;
    /* calculate PSNR */
    psnr[0] = psnr[1] = psnr[2] = 0;
    for(i = 0; i < MAX_BUMP_FRM_CNT; i++)
    {
        if(imgblist_inp[i].ts == ts && imgblist_inp[i].used == 1)
        {
            calc_psnr(imgblist_inp[i].imgb, imgb_rec, psnr, op_bit_depth_internal);
#if CALC_SSIM
            find_ms_ssim(imgblist_inp[i].imgb, imgb_rec, ms_ssim, op_bit_depth_internal);
#endif
            imgblist_inp[i].used = 0;
            return 0;
        }
    }
    return -1;
}

static int write_rec(IMGB_LIST *list, COM_MTIME *ts, int bit_depth_write, int write_rec_flag, char *filename)
{
    int i;
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].ts == (*ts) && list[i].used == 1)
        {
            if (write_rec_flag)
            {
                if (imgb_write(filename, list[i].imgb, bit_depth_write))
                {
                    v0print("cannot write reconstruction image\n");
                    return -1;
                }
            }

            list[i].used = 0;
            (*ts)++;
            break;
        }
    }

    return 0;
}

#if CALC_SSIM
void print_psnr(ENC_STAT * stat, double * psnr, double ms_ssim[3], int bitrate, COM_CLK clk_end)
#else
void print_psnr(ENC_STAT * stat, double * psnr, int bitrate, COM_CLK clk_end)
#endif
{
#if LIBVC_ON
    char  *stype;
#else
    char  stype;
#endif
    int i, j;
#if LIBVC_ON
    if (op_tool_libpic && stat->is_RLpic_flag)
    {
        stype = "RL";
    }
    else
    {
        switch (stat->stype)
        {
        case COM_ST_I:
            stype = "I";
            break;
        case COM_ST_P:
            stype = "P";
            break;
        case COM_ST_B:
            stype = "B";
            break;
        case COM_ST_UNKNOWN:
        default:
            stype = "U";
            break;
        }
    }
#else
    switch (stat->stype)
    {
    case COM_ST_I:
        stype = 'I';
        break;
    case COM_ST_P:
        stype = 'P';
        break;
    case COM_ST_B:
        stype = 'B';
        break;
    case COM_ST_UNKNOWN:
    default:
        stype = 'U';
        break;
    }
#endif
#if CALC_SSIM
#if LIBVC_ON
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        v1print("%-7d(%2s) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f%-12.7f%-12.7f", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end), ms_ssim[0], ms_ssim[1], ms_ssim[2]);
    }
    else
    {
#endif
        v1print("%-7d(%2s) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end), ms_ssim[0]);
#if CALC_SSIM_UV
    }
#endif
#else
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        v1print("%-7d(%c) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f%-12.7f%-12.7f", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end), ms_ssim[0], ms_ssim[1], ms_ssim[2]);
    }
    else 
    {
#endif
        v1print("%-7d(%c) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end), ms_ssim);
#if CALC_SSIM_UV
    }
#endif

#endif
#else
#if LIBVC_ON
    v1print("%-7d(%2s) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end));
#else
    v1print("%-7d(%c) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d", \
            stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
            bitrate, com_clk_msec(clk_end));
#endif
#endif
    for (i = 0; i < 2; i++)
    {
#if LIBVC_ON
        if (op_tool_libpic && stat->is_RLpic_flag)
        {
            v1print("[Lib%d ", i);
        }
        else
#endif
        {
            v1print("[L%d ", i);
        }
#if IPPPCRR
        for (j = 0; j < stat->refpic_num[i]; j++)
        {
            if (stat->reflib[i][j] == 1) v1print("LIB");
            v1print("%d ", stat->refpic[i][j]);
        }
#else
        for (j = 0; j < stat->refpic_num[i]; j++) v1print("%d ", stat->refpic[i][j]);
#endif
        v1print("] ");
    }
    v1print("\n");
    print_flush(stdout);
}

int setup_bumping(ENC id)
{
    ENC_CTX* ctx = (ENC_CTX *)id;
    v1print("Entering bumping process... \n");
    ctx->param.force_output = 1;
    ctx->pic_ticnt = ctx->pic_icnt;

    return 0;
}

#if LIBVC_ON
void print_libenc_data(LibVCData libvc_data)
{
    v1print("  \n\nTotal bits of libpic dependency file(bits) : %-.0f\n", libvc_data.bits_dependencyFile);
#if IPPPCRR&&LIB_PIC_UPDATE
    if (op_lib_pic_update)
        v1print("  Total libpic frames : %d ,\tOne libpic bits(bits) : %-.0f\n", libvc_data.num_lib_pic, libvc_data.bits_libpic)
    else
    {
#endif
        v1print("  Total libpic frames : %d ,\tTotal libpic bits(bits) : %-.0f\n", libvc_data.num_lib_pic, libvc_data.bits_libpic);
#if IPPPCRR&&LIB_PIC_UPDATE
    }
#endif
}

void set_livcdata_enc(ENC id_lib, LibVCData *libvc_data)
{
    ENC_CTX      * tmp_ctx = (ENC_CTX *)id_lib;
    tmp_ctx->rpm.libvc_data = libvc_data;
}

double cnt_libpic_dependency_info(char* fname, int FileOpenFlag, LibVCData libvc_data)
{
    // library_id
    int size = libvc_data.num_lib_pic;
    unsigned char ucIdLib, ucIdHighSystem, ucIdLowSystm;

    //write file
    if (size > 0 && FileOpenFlag)
    {
        /* libic information file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_libdependency, "wb");
        if (fp == NULL)
        {
            v0print("cannot open libpic information file (%s)\n", op_fname_libdependency);
            return -1;
        }
        fclose(fp);
        for (short i = 0; i < size; i++)
        {
            ucIdLib = (unsigned char)i & 0x00FF;
            ucIdHighSystem = (unsigned char)(i >> 8);
            ucIdLowSystm = (unsigned char)(i & 0x00FF);

            write_data(fname, &ucIdLib, 1, NOT_END_OF_VIDEO_SEQUENCE);
            write_data(fname, &ucIdHighSystem, 1, NOT_END_OF_VIDEO_SEQUENCE);
            write_data(fname, &ucIdLowSystm, 1, NOT_END_OF_VIDEO_SEQUENCE);

        }
    }

    //count the file bits
    double bits_dependencyfile = (double)(size * (double)(sizeof(ucIdLib)) * 3 * 8);

    return bits_dependencyfile;
}

int run_extract_feature(LibVCData *libvc_data, int candpic_idx, COM_IMGB *imgb_tmp)
{
    //// === extract feature from library picture candidates === //

    // set histogram information.
    PIC_HIST_FEATURE *feature_of_candidate_pic = &libvc_data->list_hist_feature_of_candidate_pic[candpic_idx];
    feature_of_candidate_pic->num_component = 3;
    feature_of_candidate_pic->num_of_hist_interval = 1 << op_bit_depth_input;
    feature_of_candidate_pic->length_of_interval = 1;

    // create histogram and entropy.

    for (int i = 0; i < feature_of_candidate_pic->num_component; i++)
    {
        if (feature_of_candidate_pic->list_hist_feature[i] == NULL)
        {
            feature_of_candidate_pic->list_hist_feature[i] = (int *)malloc(feature_of_candidate_pic->num_of_hist_interval * sizeof(int));
            if (feature_of_candidate_pic->list_hist_feature[i] == NULL)
            {
                v0print("cannot create feature buffer\n");
                return -1;
            }
            memset(feature_of_candidate_pic->list_hist_feature[i], 0, feature_of_candidate_pic->num_of_hist_interval * sizeof(int));
        }
    }

    for (int i = 0; i < feature_of_candidate_pic->num_component; i++)
    {
        unsigned short* pixel;

        int height = imgb_tmp->height[i];
        int width = imgb_tmp->width[i];
        int idx_interval = -1;

        int length_of_interval = feature_of_candidate_pic->length_of_interval;
        int num_of_hist_interval = feature_of_candidate_pic->num_of_hist_interval;

        // reset histogram to 0.
        memset(feature_of_candidate_pic->list_hist_feature[i], 0, feature_of_candidate_pic->num_of_hist_interval * sizeof(int));


        pixel = imgb_tmp->addr_plane[i];

        for (int ih = 0; ih < height; ih++)
        {
            for (int iw = 0; iw < width; iw++)
            {
                idx_interval = pixel[iw] / length_of_interval;

                if (idx_interval < 0 || idx_interval > num_of_hist_interval)
                {
                    return -1;
                }
                feature_of_candidate_pic->list_hist_feature[i][idx_interval]++;
            }
            pixel += width;
        }
    }

    return 0;
}

int run_decide_libpic_candidate_set_and_extract_feature(ENC_PARAM param_in, LibVCData *libvc_data)
{
    FILE            *fp_inp_tmp = NULL;
    STATES          state_tmp = STATE_ENCODING;
    int             pic_skip_tmp, pic_icnt_tmp;
    int             candpic_cnt;
    COM_IMGB        *imgb_tmp = NULL;
    int             pic_width;
    int             pic_height;
    int             ret_tmp;

    fp_inp_tmp = fopen(op_fname_inp, "rb");
    if (fp_inp_tmp == NULL)
    {
        v0print("cannot open original file (%s)\n", op_fname_inp);
        print_usage();
        return -1;
    }

    pic_width = param_in.pic_width;
    pic_height = param_in.pic_height;
    imgb_tmp = imgb_alloc(pic_width, pic_height, COM_COLORSPACE_YUV420, 10);
    imgb_tmp->horizontal_size = param_in.horizontal_size;
    imgb_tmp->vertical_size = param_in.vertical_size;

    if (imgb_tmp == NULL)
    {
        v0print("cannot allocate image \n");
        return -1;
    }

    pic_skip_tmp = 0;
    pic_icnt_tmp = 0;
    candpic_cnt = 0;
    if ((op_flag[OP_FLAG_SKIP_FRAMES] && op_skip_frames > 0) || (op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC] && op_skip_frames_when_extract_libpic > 0))
    {
        state_tmp = STATE_SKIPPING;
    }

    while (1)
    {
        if (state_tmp == STATE_SKIPPING)
        {
            if (pic_skip_tmp < op_skip_frames_when_extract_libpic)
            {
                if (imgb_read_conv(fp_inp_tmp, imgb_tmp, param_in.bit_depth_input, param_in.bit_depth_input))
                {
                    v2print("reached end of original file (or reading error) when skip frames\n");
                    return -1;
                }
            }
            else
            {
                state_tmp = STATE_ENCODING;
            }
            pic_skip_tmp++;
            continue;
        }
        if (state_tmp == STATE_ENCODING)
        {
            /* read original image */
            if (pic_icnt_tmp == op_max_frm_num_when_extract_libpic || imgb_read_conv(fp_inp_tmp, imgb_tmp, param_in.bit_depth_input, param_in.bit_depth_input))
            {
                v2print("reached end of original file (or reading error)\n");
                break;
            }
            else if (pic_icnt_tmp != candpic_cnt * param_in.i_period)
            {
                pic_icnt_tmp++;
                continue;
            }

            //store library picture candidate set
            libvc_data->num_candidate_pic++;
            libvc_data->list_poc_of_candidate_pic[candpic_cnt] = candpic_cnt * param_in.i_period;
            libvc_data->list_candidate_pic[candpic_cnt] = imgb_tmp;

            //extract feature from library picture candidates
            ret_tmp = run_extract_feature(libvc_data, candpic_cnt, imgb_tmp);
            if (ret_tmp)
            {
                v0print("error when extract feature\n");
                return -1;
            }

            pic_icnt_tmp++;
            candpic_cnt++;

            imgb_tmp = NULL;
            imgb_tmp = imgb_alloc(pic_width, pic_height, COM_COLORSPACE_YUV420, 10);
            imgb_tmp->horizontal_size = param_in.horizontal_size;
            imgb_tmp->vertical_size = param_in.vertical_size;

            if (imgb_tmp == NULL)
            {
                v0print("cannot allocate image \n");
                return -1;
            }
            
        }
    }

    if (imgb_tmp)
    {
        imgb_free(imgb_tmp);
        imgb_tmp = NULL;
    }
    if (fp_inp_tmp) fclose(fp_inp_tmp);

    return 0;
}

double get_mse_of_hist_feature(PIC_HIST_FEATURE *x, PIC_HIST_FEATURE *y)
{
    int num_component = x->num_component;
    int num_of_hist_interval = x->num_of_hist_interval;


    double sum = 0;
    for (int i = 0; i < num_component; i++)
    {
        for (int j = 0; j < num_of_hist_interval; j++)
        {
            sum += (x->list_hist_feature[i][j] - y->list_hist_feature[i][j])*(x->list_hist_feature[i][j] - y->list_hist_feature[i][j]);
        }
    }

    return sqrt(sum);
}

double get_sad_of_hist_feature(PIC_HIST_FEATURE *x, PIC_HIST_FEATURE *y)
{
    int num_component = x->num_component;
    int num_of_hist_interval = x->num_of_hist_interval;

    double sum = 0;
    for (int i = 0; i < num_component; i++)
    {
        for (int j = 0; j < num_of_hist_interval; j++)
        {
            sum += abs(x->list_hist_feature[i][j] - y->list_hist_feature[i][j]);
        }
    }

    return sqrt(sum);
}

int run_classify(LibVCData *libvc_data, int num_cluster, int *list_cluster_center_idx, int cur_candpic_idx)
{
    PIC_HIST_FEATURE *cur_feature = NULL;
    PIC_HIST_FEATURE *center_feature = NULL;
    double mse;
    double min_mse;
    int cluster_idx;
    int center_idx;

    cur_feature = &libvc_data->list_hist_feature_of_candidate_pic[cur_candpic_idx];

    cluster_idx = 0;
    center_idx = list_cluster_center_idx[cluster_idx];
    center_feature = &libvc_data->list_hist_feature_of_candidate_pic[center_idx];
    min_mse = get_mse_of_hist_feature(cur_feature, center_feature);

    for (int i = 1; i < num_cluster; i++)
    {
        center_idx = list_cluster_center_idx[i];
        center_feature = &libvc_data->list_hist_feature_of_candidate_pic[center_idx];
        mse = get_mse_of_hist_feature(cur_feature, center_feature);
        if (mse < min_mse)
        {
            min_mse = mse;
            cluster_idx = i;
        }
    }

    cur_feature = NULL;
    center_feature = NULL;

    return cluster_idx;
}

double get_total_mse(LibVCData *libvc_data, int *list_cluster_center_idx, int *list_candpic_cluster_idx)
{
    double total_mse = 0;
    int cluster_idx, center_idx;
    PIC_HIST_FEATURE *cur_feature = NULL;
    PIC_HIST_FEATURE *center_feature = NULL;

    for (int i = 0; i < libvc_data->num_candidate_pic; i++)
    {
        cur_feature = &libvc_data->list_hist_feature_of_candidate_pic[i];
        cluster_idx = list_candpic_cluster_idx[i];
        center_idx = list_cluster_center_idx[cluster_idx];
        center_feature = &libvc_data->list_hist_feature_of_candidate_pic[center_idx];

        total_mse += get_mse_of_hist_feature(cur_feature, center_feature);
    }
    cur_feature = NULL;
    center_feature = NULL;

    return total_mse;
}

int update_center(LibVCData *libvc_data, int cluster_member_num, int *cluster_member_idx)
{
    double cluster_mse = MAX_COST;
    double min_cluster_mse = MAX_COST;
    int center_idx = 0;
    int tmp_center_idx, tmp_cur_idx;
    PIC_HIST_FEATURE *cur_feature = NULL;
    PIC_HIST_FEATURE *center_feature = NULL;

    for (int i = 0; i < cluster_member_num; i++)
    {
        tmp_center_idx = cluster_member_idx[i];
        center_feature = &libvc_data->list_hist_feature_of_candidate_pic[tmp_center_idx];

        cluster_mse = 0;
        for (int j = 0; j < cluster_member_num; j++)
        {
            tmp_cur_idx = cluster_member_idx[j];
            cur_feature = &libvc_data->list_hist_feature_of_candidate_pic[tmp_cur_idx];
            cluster_mse += get_mse_of_hist_feature(cur_feature, center_feature);
        }
        if (i == 0)
        {
            min_cluster_mse = cluster_mse;
            center_idx = tmp_center_idx;
        }
        else
        {
            if (cluster_mse < min_cluster_mse)
            {
                min_cluster_mse = cluster_mse;
                center_idx = tmp_center_idx;
            }
        }
    }

    cur_feature = NULL;
    center_feature = NULL;

    return center_idx;

}

int get_a_new_center(LibVCData *libvc_data, int num_cluster, int *list_cluster_center_idx, int *list_candpic_cluster_idx)
{
    double max_mse = -1;
    double mse = 0;
    int new_center_idx = -1;
    int cluster_idx, center_idx;
    PIC_HIST_FEATURE *cur_feature = NULL;
    PIC_HIST_FEATURE *center_feature = NULL;

    for (int i = 0; i < libvc_data->num_candidate_pic; i++)
    {
        cur_feature = &libvc_data->list_hist_feature_of_candidate_pic[i];
        cluster_idx = list_candpic_cluster_idx[i];
        center_idx = list_cluster_center_idx[cluster_idx];
        center_feature = &libvc_data->list_hist_feature_of_candidate_pic[center_idx];

        mse = get_mse_of_hist_feature(cur_feature, center_feature);
        if (mse > max_mse)
        {
            max_mse = mse;
            new_center_idx = i;
        }

    }

    cur_feature = NULL;
    center_feature = NULL;

    return new_center_idx;
}

double get_cluster_cost(LibVCData *libvc_data, int num_cluster, int *list_cluster_center_idx, int *list_cluster_member_num, int list_cluster_member_idx[MAX_CANDIDATE_PIC][MAX_CANDIDATE_PIC])
{
    double normalize;
    double center_cost, inter_cost, delta_inter_cost;
    double cluster_cost;
    double list_center_cost[MAX_CANDIDATE_PIC];
    double list_inter_cost[MAX_CANDIDATE_PIC];
    double *joint_distribution = NULL;
    int center_idx, member_idx;
    COM_IMGB * center_pic = NULL;
    COM_IMGB * member_pic = NULL;
    unsigned short *center_pixel_org = NULL;
    unsigned short *center_pixel_y = NULL;
    unsigned short *member_pixel_y = NULL;
    PIC_HIST_FEATURE *center_feature = NULL;

    int width = libvc_data->list_candidate_pic[0]->width[0];
    int height = libvc_data->list_candidate_pic[0]->height[0];
    int num_of_hist_interval = libvc_data->list_hist_feature_of_candidate_pic[0].num_of_hist_interval;


    for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
    {
        list_center_cost[i] = 0;
        list_inter_cost[i] = 0;
    }

    for (int i = 0; i < num_cluster; i++)
    {
        center_cost = 0;
        center_idx = list_cluster_center_idx[i];
        center_feature = &libvc_data->list_hist_feature_of_candidate_pic[center_idx];
        for (int n = 0; n < num_of_hist_interval; n++)
        {
            normalize = (double)center_feature->list_hist_feature[0][n] / (double)(width * height);
            center_cost += normalize ? normalize * (log(normalize) / log(2.0)) : 0;
        }
        center_cost = -center_cost;
        list_center_cost[i] = center_cost;
        center_feature = NULL;
    }


    for (int i = 0; i < num_cluster; i++)
    {
        inter_cost = 0;
        center_idx = list_cluster_center_idx[i];
        center_pic = libvc_data->list_candidate_pic[center_idx];
        center_pixel_org = center_pic->addr_plane[0];

        joint_distribution = (double *)malloc(sizeof(double) * num_of_hist_interval * num_of_hist_interval);
        for (int j = 0; j < list_cluster_member_num[i]; j++)
        {
            delta_inter_cost = 0;
            center_pixel_y = center_pixel_org;

            // initialize
            for (int k = 0; k < num_of_hist_interval; k++)
            {
                for (int t = 0; t < num_of_hist_interval; t++)
                {
                    joint_distribution[k * num_of_hist_interval + t] = 0;
                }
            }

            member_idx = list_cluster_member_idx[i][j];
            member_pic = libvc_data->list_candidate_pic[member_idx];
            member_pixel_y = member_pic->addr_plane[0];

            for (int k = 0; k < height; k++)
            {
                for (int t = 0; t < width; t++)
                {
                    joint_distribution[center_pixel_y[t] * num_of_hist_interval + member_pixel_y[t]]++;
                }

                center_pixel_y += width;
                member_pixel_y += width;

            }
            for (int k = 0; k < num_of_hist_interval; k++)
            {
                for (int t = 0; t < num_of_hist_interval; t++)
                {
                    normalize = joint_distribution[k * num_of_hist_interval + t] / (width * height);
                    delta_inter_cost += normalize ? normalize * (log(normalize) / log(2.0)) : 0;
                }
            }
            inter_cost += -delta_inter_cost - list_center_cost[i];
        }
        list_inter_cost[i] = inter_cost;
        free(joint_distribution);
        joint_distribution = NULL;
        center_pixel_org = NULL;
        center_pixel_y = NULL;
        member_pixel_y = NULL;
        center_pic = NULL;
        member_pic = NULL;

    }

    cluster_cost = 0;
    for (int i = 0; i < num_cluster; i++)
    {

        if (list_cluster_member_num[i] == 1)
        {
            cluster_cost += list_center_cost[i];
        }
        else
        {
            cluster_cost += list_center_cost[i];
            cluster_cost += list_inter_cost[i];
        }
    }
    return cluster_cost;
}

double k_means(LibVCData *libvc_data, int num_cluster, int *list_init_cluster_center_idx, int *list_candpic_cluster_center_idx, int *list_final_cluster_member_num)
{
    double total_mse_old, total_mse_new;
    int cluster_idx = 0;
    int list_cluster_center_idx[MAX_CANDIDATE_PIC];
    int list_cluster_member_num[MAX_CANDIDATE_PIC];
    int list_cluster_member_idx[MAX_CANDIDATE_PIC][MAX_CANDIDATE_PIC];
    int list_candpic_cluster_idx[MAX_CANDIDATE_PIC];
    double cluster_cost;

    for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
    {
        list_candpic_cluster_idx[i] = -1;
        list_cluster_center_idx[i] = -1;
        list_cluster_member_num[i] = 0;
        for (int j = 0; j < MAX_CANDIDATE_PIC; j++)
        {
            list_cluster_member_idx[i][j] = -1;
        }
    }

    // set original cluster centers
    for (int i = 0; i < num_cluster; i++)
    {
        list_cluster_center_idx[i] = list_init_cluster_center_idx[i];
    }
    // classify each candpic according to the original cluster centers
    for (int candpic_idx = 0; candpic_idx < libvc_data->num_candidate_pic; candpic_idx++)
    {
        cluster_idx = run_classify(libvc_data, num_cluster, list_cluster_center_idx, candpic_idx);
        list_candpic_cluster_idx[candpic_idx] = cluster_idx;
        list_cluster_member_idx[cluster_idx][list_cluster_member_num[cluster_idx]] = candpic_idx;
        list_cluster_member_num[cluster_idx]++;
    }
    total_mse_old = -1;
    total_mse_new = get_total_mse(libvc_data, list_cluster_center_idx, list_candpic_cluster_idx);


    while (fabs(total_mse_new - total_mse_old) >= 1)
    {
        total_mse_old = total_mse_new;
        // update cluster means
        for (int i = 0; i < num_cluster; i++)
        {
            list_cluster_center_idx[i] = update_center(libvc_data, list_cluster_member_num[i], list_cluster_member_idx[i]);
            // clear
            for (int j = 0; j < list_cluster_member_num[i]; j++)
            {
                list_cluster_member_idx[i][j] = -1;
            }
            list_cluster_member_num[i] = 0;
        }
        total_mse_new = get_total_mse(libvc_data, list_cluster_center_idx, list_candpic_cluster_idx);

        // classify each candpic according to the original cluster means
        for (int candpic_idx = 0; candpic_idx < libvc_data->num_candidate_pic; candpic_idx++)
        {
            cluster_idx = run_classify(libvc_data, num_cluster, list_cluster_center_idx, candpic_idx);
            list_candpic_cluster_idx[candpic_idx] = cluster_idx;
            list_cluster_member_idx[cluster_idx][list_cluster_member_num[cluster_idx]] = candpic_idx;
            list_cluster_member_num[cluster_idx]++;
        }
        //total_mse_new = get_total_mse(libvc_data, list_cluster_center_idx, list_candpic_cluster_idx);
    }


    for (int i = 0; i < libvc_data->num_candidate_pic; i++)
    {
        cluster_idx = list_candpic_cluster_idx[i];
        list_candpic_cluster_center_idx[i] = list_cluster_center_idx[cluster_idx];
    }
    for (int i = 0; i < num_cluster; i++)
    {
        list_final_cluster_member_num[i] = list_cluster_member_num[i];
    }

    // get the initial point of the next new cluster
    for (int i = 0; i < num_cluster; i++)
    {
        list_init_cluster_center_idx[i] = list_cluster_center_idx[i];
    }
    list_init_cluster_center_idx[num_cluster] = get_a_new_center(libvc_data, num_cluster, list_cluster_center_idx, list_candpic_cluster_idx);

    // get the final cluster cost
    cluster_cost = get_cluster_cost(libvc_data, num_cluster, list_cluster_center_idx, list_cluster_member_num, list_cluster_member_idx);


    return cluster_cost;
}

void run_generate_libpic(LibVCData* libvc_data, int *list_libpic_idx)
{
    // === process kmeans to generate library picture === //

    double tmp_cluster_cost, best_cluster_cost;
    int tmp_num_cluster, best_num_cluster;
    int tmp_list_candpic_cluster_center_idx[MAX_CANDIDATE_PIC];
    int list_final_candpic_cluster_center_idx[MAX_CANDIDATE_PIC];
    int list_init_cluster_center_idx[MAX_NUM_LIBPIC+1];
    int list_final_cluster_center_idx[MAX_NUM_LIBPIC];
    int tmp_list_final_cluster_member_num[MAX_NUM_LIBPIC];
    int list_final_cluster_member_num[MAX_NUM_LIBPIC];

    int num_libpic;
    int list_poc_of_libpic[MAX_NUM_LIBPIC];
    int max_num_cluster;

    // initialize all list.
    for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
    {
        tmp_list_candpic_cluster_center_idx[i] = -1;
        list_final_candpic_cluster_center_idx[i] = -1;
    }
    for (int i = 0; i < MAX_NUM_LIBPIC; i++)
    {
        list_poc_of_libpic[i] = -1;
        list_init_cluster_center_idx[i] = -1;
        list_final_cluster_center_idx[i] = -1;
        tmp_list_final_cluster_member_num[i] = 0;
        list_final_cluster_member_num[i] = 0;
    }
    list_init_cluster_center_idx[MAX_NUM_LIBPIC] = -1;

    // cluster candidate pic by using k_means && choose the best cluster result
    // initialize k_means with num_cluster = 1;
    tmp_num_cluster = 1;
    //list_init_cluster_center_idx[0] = rand() % libvc_data->num_candidate_pic;
    list_init_cluster_center_idx[0] = 0;
    tmp_cluster_cost = k_means(libvc_data, tmp_num_cluster, list_init_cluster_center_idx, tmp_list_candpic_cluster_center_idx, tmp_list_final_cluster_member_num);
    best_num_cluster = tmp_num_cluster;
    best_cluster_cost = tmp_cluster_cost;
    for (int i = 0; i < libvc_data->num_candidate_pic; i++)
    {
        list_final_candpic_cluster_center_idx[i] = tmp_list_candpic_cluster_center_idx[i];
    }
    for (int i = 0; i < best_num_cluster; i++)
    {
        list_final_cluster_center_idx[i] = list_init_cluster_center_idx[i];
        list_final_cluster_member_num[i] = tmp_list_final_cluster_member_num[i];
    }

    max_num_cluster = min(libvc_data->num_candidate_pic, MAX_NUM_LIBPIC);
    for (tmp_num_cluster = 2; tmp_num_cluster <= max_num_cluster; tmp_num_cluster++)
    {
        int is_break = 0;
        for (int i = 0; i < tmp_num_cluster - 1; i++)
        {
            // the last one center is the new added on.
            if (list_init_cluster_center_idx[tmp_num_cluster - 1] == list_init_cluster_center_idx[i])
            {
                is_break = 1;
                break;
            }
        }
        if (is_break)
        {
            break;
        }

        tmp_cluster_cost = k_means(libvc_data, tmp_num_cluster, list_init_cluster_center_idx, tmp_list_candpic_cluster_center_idx, tmp_list_final_cluster_member_num);
        if (tmp_cluster_cost < best_cluster_cost)
        {
            best_cluster_cost = tmp_cluster_cost;
            best_num_cluster = tmp_num_cluster;
            for (int t = 0; t < libvc_data->num_candidate_pic; t++)
            {
                list_final_candpic_cluster_center_idx[t] = tmp_list_candpic_cluster_center_idx[t];
            }
            for (int k = 0; k < best_num_cluster; k++)
            {
                list_final_cluster_center_idx[k] = list_init_cluster_center_idx[k];
                list_final_cluster_member_num[k] = tmp_list_final_cluster_member_num[k];
            }
        }
    }

    // avoid the cluster with only one library picture.
    num_libpic = 0;
    for (int i = 0; i < best_num_cluster; i++)
    {
        if (list_final_cluster_member_num[i] > 1)
        {
            int center_idx = list_final_cluster_center_idx[i];
            list_libpic_idx[num_libpic] = center_idx;
            list_poc_of_libpic[num_libpic] = libvc_data->list_poc_of_candidate_pic[center_idx];
            num_libpic++;
        }
    }
    num_libpic = min(MAX_NUM_LIBPIC, num_libpic);
    //reorder the library frame according to POC value
    for (int i = 0; i < num_libpic; i++)
    {
        for (int j = i + 1; j < num_libpic; j++)
        {
            if (list_poc_of_libpic[i] > list_poc_of_libpic[j])
            {
                int tmp = list_poc_of_libpic[i];
                list_poc_of_libpic[i] = list_poc_of_libpic[j];
                list_poc_of_libpic[j] = tmp;
                tmp = list_libpic_idx[i];
                list_libpic_idx[i] = list_libpic_idx[j];
                list_libpic_idx[j] = tmp;
            }
        }
    }

    // store the libpic infomation
    libvc_data->num_lib_pic = num_libpic;
    for (int i = 0; i < libvc_data->num_lib_pic; i++)
    {
        libvc_data->list_poc_of_libpic[i] = list_poc_of_libpic[i];
    }
}

void run_decide_pic_referencing_libpic(LibVCData* libvc_data, int *list_libpic_idx)
{
    // decide sequence pictures that reference library picture

    PIC_HIST_FEATURE *cur_feature = NULL;
    PIC_HIST_FEATURE *lib_feature = NULL;
    int list_candpic_libidx[MAX_CANDIDATE_PIC];

    double min_diff = MAX_DIFFERENCE_OF_RLPIC_AND_LIBPIC;
    double diff;
    int lib_idx;
    int num_RLpic;


    for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
    {
        list_candpic_libidx[i] = -1;
    }

    for (int candpic_idx = 0; candpic_idx < libvc_data->num_candidate_pic; candpic_idx++)
    {
        cur_feature = &libvc_data->list_hist_feature_of_candidate_pic[candpic_idx];

        for (int libpid = 0; libpid < libvc_data->num_lib_pic; libpid++)
        {
            lib_idx = list_libpic_idx[libpid];
            lib_feature = &libvc_data->list_hist_feature_of_candidate_pic[lib_idx];
            diff = get_sad_of_hist_feature(cur_feature, lib_feature);
            if (diff < min_diff)
            {
                min_diff = diff;
                list_candpic_libidx[candpic_idx] = libpid;
            }
        }
        min_diff = MAX_DIFFERENCE_OF_RLPIC_AND_LIBPIC;
    }
    cur_feature = NULL;
    lib_feature = NULL;

    // store the RLpic infomation
    num_RLpic = 0;
    for (int candpic_idx = 0; candpic_idx < libvc_data->num_candidate_pic; candpic_idx++)
    {
        if (list_candpic_libidx[candpic_idx] >= 0)
        {
            libvc_data->list_poc_of_RLpic[num_RLpic] = libvc_data->list_poc_of_candidate_pic[candpic_idx];
            libvc_data->list_libidx_for_RLpic[num_RLpic] = list_candpic_libidx[candpic_idx];
            num_RLpic++;
        }
    }
    libvc_data->num_RLpic = num_RLpic;


    //clear buffer
    for (int i = 0; i < libvc_data->num_candidate_pic; i++)
    {
        if (libvc_data->list_candidate_pic[i])
        {
            imgb_free(libvc_data->list_candidate_pic[i]);
            libvc_data->list_candidate_pic[i] = NULL;
        }
        for (int j = 0; j < MAX_NUM_COMPONENT; j++)
        {
            if (libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j])
            {
                free(libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j]);
                libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j] = NULL;
            }
        }
    }

}

int compute_LibVCData(ENC_PARAM param_in, LibVCData* libvc_data)
{
    int ret = 0;
    int list_libpic_idx[MAX_NUM_LIBPIC];
    for (int i = 0; i < MAX_NUM_LIBPIC; i++)
    {
        list_libpic_idx[i] = -1;
    }

    /*
    for (int i = 0; i * param_in.i_period < param_in.frames_to_be_encoded; i++)
    {
        libvc_data->list_poc_of_RLpic[i] = i * param_in.i_period;
        libvc_data->num_RLpic++;
    }

    libvc_data->num_lib_pic = 3;
    for (int i = 0; i < libvc_data->num_lib_pic; i++)
    {
        libvc_data->list_poc_of_libpic[i] = libvc_data->list_poc_of_RLpic[i];
    }
    for (int i = 0; i < libvc_data->num_RLpic / 3; i++)
    {
        libvc_data->list_libidx_for_RLpic[i] = 0;
    }
    for (int i = libvc_data->num_RLpic / 3; i < libvc_data->num_RLpic * 2 / 3; i++)
    {
        libvc_data->list_libidx_for_RLpic[i] = 1;
    }
    for (int i = libvc_data->num_RLpic * 2 / 3; i < libvc_data->num_RLpic; i++)
    {
        libvc_data->list_libidx_for_RLpic[i] = 2;
    }
    */

    // step 1
#if IPPPCRR
#if LIB_PIC_UPDATE
    libvc_data->lib_pic_update = op_lib_pic_update;
    if (op_lib_pic_update == 1)
#else
    libvc_data->first_pic_as_libpic = op_first_pic_as_libpic;
    if (op_first_pic_as_libpic == 1)
#endif
    {
        if (param_in.i_period != 0)
        {
            libvc_data->num_candidate_pic = (op_max_frm_num_when_extract_libpic - 1) / param_in.i_period + 1;
        }
        else
        {
            libvc_data->num_candidate_pic = 0;
        }
        for (int i = 0; i < libvc_data->num_candidate_pic; i++)
        {
            libvc_data->list_poc_of_candidate_pic[i] = i * param_in.i_period;
        }
#if !LIB_PIC_UPDATE
        libvc_data->num_lib_pic = 1;
        list_libpic_idx[0] = 0;
        libvc_data->list_poc_of_libpic[0] = 0;
#endif
        for (int candpic_idx = 0; candpic_idx < libvc_data->num_candidate_pic; candpic_idx++)
        {
            libvc_data->list_poc_of_RLpic[candpic_idx] = libvc_data->list_poc_of_candidate_pic[candpic_idx];
            libvc_data->list_libidx_for_RLpic[candpic_idx] = 0;
        }
        libvc_data->num_RLpic = libvc_data->num_candidate_pic;
    }
    else
    {
#endif
        ret = run_decide_libpic_candidate_set_and_extract_feature(param_in, libvc_data);
        if (ret)
        {
            // error when decide libpic candidate set and extract feature, clear buffer
            for (int i = 0; i < MAX_CANDIDATE_PIC; i++)
            {
                if (libvc_data->list_candidate_pic[i])
                {
                    imgb_free(libvc_data->list_candidate_pic[i]);
                    libvc_data->list_candidate_pic[i] = NULL;
                }
                for (int j = 0; j < MAX_NUM_COMPONENT; j++)
                {
                    if (libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j])
                    {
                        free(libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j]);
                        libvc_data->list_hist_feature_of_candidate_pic[i].list_hist_feature[j] = NULL;
                    }
                }
            }

            return -1;
        }

        // step 2
        run_generate_libpic(libvc_data, list_libpic_idx);

        // step 3
        run_decide_pic_referencing_libpic( libvc_data, list_libpic_idx );
#if IPPPCRR
    }
#endif
    // stat libdependency info
    libvc_data->bits_dependencyFile = cnt_libpic_dependency_info(op_fname_libdependency, op_flag[OP_FLAG_FNAME_LIBDATA], *libvc_data);


    return 0;
}

#if IPPPCRR && LIB_PIC_UPDATE
int enc_free(ENC_CTX * ctx)
{
    ctx->inbuf[0] = NULL;
    return COM_OK;
}
int  free_enc_libpic(ENC_LIBPIC *enc_libpic)
{
    write_data(op_fname_libout, NULL, 0, END_OF_VIDEO_SEQUENCE);
    enc_delete(enc_libpic->id_lib, 2);
    imgb_list_free(enc_libpic->ilist_org_lib);
    imgb_list_free(enc_libpic->ilist_rec_lib);
    if (enc_libpic->fp_inp_lib)  fclose(enc_libpic->fp_inp_lib);
    if (enc_libpic->bs_buf_lib)  free(enc_libpic->bs_buf_lib); /* release bitstream buffer */
    if (enc_libpic->bs_buf_lib2) free(enc_libpic->bs_buf_lib2); /* release bitstream buffer */
    return 0;
}
int enc_prepare_LibVC(ENC_PARAM param_in, LibVCData* libvc_data, ENC_LIBPIC * enc_libpic)
{
    enc_libpic->state_lib = STATE_ENCODING;
    enc_libpic->bs_buf_lib = NULL;
    enc_libpic->bs_buf_lib2 = NULL;
    enc_libpic->fp_inp_lib = NULL;
    enc_libpic->imgb_enc_lib = NULL;
    enc_libpic->imgb_rec_lib = NULL;
    enc_libpic->udata_size_lib = 0;
    enc_libpic->num_encoded_frames = 0;
    for (int i = 0; i < 3; i++)
    {
        enc_libpic->psnr_lib[i] = 0;
        enc_libpic->psnr_avg_lib[i] = 0;
#if CALC_SSIM
        enc_libpic->ms_ssim_lib[i] = 0;
        enc_libpic->ms_ssim_avg_lib[i] = 0;
#endif
    }
    enc_libpic->ilist_t_lib = NULL;
    enc_libpic->is_first_enc_lib = 1;
#if !CALC_SSIM
    enc_libpic->seq_header_bit = 0;
#endif
#if LINUX
    signal(SIGSEGV, handler);   // install our handler
#endif
    if (op_flag[OP_FLAG_FNAME_OUT])
    {
        /* libic bitstream file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_libout, "wb");
        if (fp == NULL)
        {
            v0print("cannot open libpic bitstream file (%s)\n", op_fname_libout);
            return -1;
        }
        fclose(fp);
    }
    if (op_flag[OP_FLAG_FNAME_LIBREC])
    {
        /* reconstruction libic file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_librec, "ab");
        if (fp == NULL)
        {
            v0print("cannot open reconstruction libpic file (%s)\n", op_fname_librec);
            return -1;
        }
        fclose(fp);
    }

    /* open original file */
    enc_libpic->fp_inp_lib = fopen(op_fname_inp, "rb");
    if (enc_libpic->fp_inp_lib == NULL)
    {
        v0print("cannot open original file (%s)\n", op_fname_inp);
        print_usage();
        return -1;
    }
    /* allocate bitstream buffer */
    enc_libpic->bs_buf_lib = (unsigned char*)malloc(MAX_BS_BUF);
    if (enc_libpic->bs_buf_lib == NULL)
    {
        v0print("cannot allocate bitstream buffer, size=%d", MAX_BS_BUF);
        return -1;
    }
    enc_libpic->bs_buf_lib2 = (unsigned char*)malloc(MAX_BS_BUF);
    if (enc_libpic->bs_buf_lib2 == NULL)
    {
        v0print("ERROR: cannot allocate bit buffer, size2=%d\n", MAX_BS_BUF);
        return -1;
    }
    /* create encoder */
    enc_libpic->id_lib = enc_create(&param_in, NULL);
    if (enc_libpic->id_lib == NULL)
    {
        v0print("cannot create encoder\n");
        return -1;
    }
    set_livcdata_enc(enc_libpic->id_lib, libvc_data);

    if (set_extra_config(enc_libpic->id_lib))
    {
        v0print("cannot set extra configurations\n");
        return -1;
    }
    /* create image lists */
    if (imgb_list_alloc(enc_libpic->ilist_org_lib, param_in.pic_width, param_in.pic_height, param_in.horizontal_size, param_in.vertical_size, 10))
    {
        v0print("cannot allocate image list for original image\n");
        return -1;
    }
    if (imgb_list_alloc(enc_libpic->ilist_rec_lib, param_in.pic_width, param_in.pic_height, param_in.horizontal_size, param_in.vertical_size, param_in.bit_depth_internal))
    {
        v0print("cannot allocate image list for reconstructed image\n");
        return -1;
    }

    enc_libpic->bits_libpic = 0;
    enc_libpic->bitrate_lib = 0;
    enc_libpic->bitb_lib.addr = enc_libpic->bs_buf_lib;
    enc_libpic->bitb_lib.addr2 = enc_libpic->bs_buf_lib2;
    enc_libpic->bitb_lib.bsize = MAX_BS_BUF;
#if WRITE_MD5_IN_USER_DATA //discount user data bits in per-frame bits
    enc_libpic->udata_size_lib = (op_use_pic_signature) ? 23 : 0;
#endif
#if !PRECISE_BS_SIZE
    enc_libpic->udata_size_lib += 4; /* 4-byte prefix (length field of chunk) */
#endif
    /* encode Sequence Header if needed **************************************/
    enc_libpic->bitb_lib.err = 0; // update BSB
#if REPEAT_SEQ_HEADER
    enc_libpic->ret = init_seq_header((ENC_CTX *)enc_libpic->id_lib, &enc_libpic->bitb_lib);
#else
    ret = enc_seq_header((ENC_CTX *)enc_libpic->id_lib, &enc_libpic->bitb_lib, &enc_libpic->stat_lib);
    if (COM_FAILED(ret))
    {
        v0print("cannot encode header \n");
        return -1;
    }
    if (op_flag[OP_FLAG_FNAME_OUT])
    {
        /* write Sequence Header bitstream to file */
        if (write_data(op_fname_libout, enc_libpic->bs_buf_lib, enc_libpic->stat_lib.write, NOT_END_OF_VIDEO_SEQUENCE))
        {
            v0print("Cannot write header information (SQH)\n");
            return -1;
        }
#if PRECISE_BS_SIZE
        enc_libpic->bitrate_lib += enc_libpic->stat_lib.write;
#if !CALC_SSIM
        enc_libpic->seq_header_bit = enc_libpic->stat_lib.write;
#endif
#else
        enc_libpic->bitrate_lib += (enc_libpic->stat_lib.write - 4)/* 4-byte prefix (length field of chunk) */;
#if !CALC_SSIM
        enc_libpic->seq_header_bit = (enc_libpic->stat_lib.write - 4)/* 4-byte prefix (length field of chunk) */;
#endif
#endif
    }
#endif
    if ((op_flag[OP_FLAG_SKIP_FRAMES] && op_skip_frames > 0) || (op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC] && op_skip_frames_when_extract_libpic > 0))
    {
        enc_libpic->state_lib = STATE_SKIPPING;
    }
    enc_libpic->clk_tot_lib = 0;
    enc_libpic->pic_icnt_lib = 0;
    enc_libpic->pic_ocnt_lib = 0;
    enc_libpic->pic_skip_lib = 0;

    enc_libpic->libpic_icnt = 0;
    /* encode pictures *******************************************************/
    while (1)
    {
        if (enc_libpic->state_lib == STATE_SKIPPING)
        {
            if (enc_libpic->pic_skip_lib < op_skip_frames_when_extract_libpic)
            {
                enc_libpic->ilist_t_lib = imgb_list_get_empty(enc_libpic->ilist_org_lib);
                if (enc_libpic->ilist_t_lib == NULL)
                {
                    v0print("cannot get empty orignal buffer\n");
                    free_enc_libpic(enc_libpic);
                    return -1;
                }

                if (enc_libpic->pic_icnt_lib == op_max_frm_num_when_extract_libpic || imgb_read_conv(enc_libpic->fp_inp_lib, enc_libpic->ilist_t_lib->imgb, param_in.bit_depth_input, param_in.bit_depth_internal))
                {
                    v2print("reached end of original file (or reading error)\n");
                    free_enc_libpic(enc_libpic);
                    return -1;
                }
            }
            else
            {
                enc_libpic->state_lib = STATE_ENCODING;
                return 0;
            }
            enc_libpic->pic_skip_lib++;
            continue;
        }
        else break;
    }
    return 0;
}

int encode_one_libpic(ENC_PARAM param_in, LibVCData* libvc_data, ENC_LIBPIC * enc_libpic)
{
    /* encode pictures *******************************************************/
    //enc_libpic->id_lib = enc_create(&param_in, NULL);
    //set_livcdata_enc(enc_libpic->id_lib, libvc_data);
    //init_seq_header((ENC_CTX *)enc_libpic->id_lib, &enc_libpic->bitb_lib);
    while (1)
    {
        if (enc_libpic->state_lib == STATE_ENCODING)
        {
            enc_libpic->ilist_t_lib = imgb_list_get_empty(enc_libpic->ilist_org_lib);
            if (enc_libpic->ilist_t_lib == NULL)
            {
                v0print("cannot get empty orignal buffer\n");
                return -1;
            }
            if (enc_libpic->pic_icnt_lib == op_max_frm_num_when_extract_libpic || imgb_read_conv(enc_libpic->fp_inp_lib, enc_libpic->ilist_t_lib->imgb, param_in.bit_depth_input, param_in.bit_depth_internal) || enc_libpic->libpic_icnt == libvc_data->num_lib_pic)
            {
                v2print("reached end of original file (or reading error)\n");
                enc_libpic->state_lib = STATE_BUMPING;
                setup_bumping(enc_libpic->id_lib);
                continue;
            }
            else if (enc_libpic->pic_icnt_lib != libvc_data->list_poc_of_libpic[enc_libpic->libpic_icnt]) {
                enc_libpic->pic_icnt_lib++;
                continue;
            }
            imgb_list_make_used(enc_libpic->ilist_t_lib, enc_libpic->libpic_icnt);
            enc_libpic->libpic_icnt++;

            /* get encodng buffer */
            //com_assert_rv((&imgb_enc) != NULL, COM_ERR_INVALID_ARGUMENT);
            if (COM_OK != enc_picbuf_get_inbuf((ENC_CTX *)enc_libpic->id_lib, &enc_libpic->imgb_enc_lib))
            {
                v0print("Cannot get original image buffer\n");
                return -1;
            }
            /* copy original image to encoding buffer */
            imgb_cpy_internal(enc_libpic->imgb_enc_lib, enc_libpic->ilist_t_lib->imgb);
            /* push image to encoder */
            enc_libpic->ret = enc_push_frm((ENC_CTX *)enc_libpic->id_lib, enc_libpic->imgb_enc_lib);
            if (COM_FAILED(enc_libpic->ret))
            {
                v0print("enc_push() failed\n");
                return -1;
            }
            /* release encoding buffer */
            enc_libpic->imgb_enc_lib->release(enc_libpic->imgb_enc_lib);
            enc_libpic->pic_icnt_lib++;
        }
        /* encoding */
        enc_libpic->clk_beg_lib = com_clk_get();
        enc_libpic->ret = enc_encode(enc_libpic->id_lib, &(enc_libpic->bitb_lib), &(enc_libpic->stat_lib));
        enc_libpic->num_encoded_frames += enc_libpic->ret == COM_OK;
        if (COM_FAILED(enc_libpic->ret))
        {
            v0print("enc_encode() failed\n");
            return -1;
        }
        enc_libpic->clk_end_lib = com_clk_from(enc_libpic->clk_beg_lib);
        enc_libpic->clk_tot_lib += enc_libpic->clk_end_lib;
        /* store bitstream */
        if (enc_libpic->ret == COM_OK_OUT_NOT_AVAILABLE)
        {
            v1print("--> RETURN OK BUT PICTURE IS NOT AVAILABLE YET\n");
            continue;
        }
        else if (enc_libpic->ret == COM_OK)
        {
            if (op_flag[OP_FLAG_FNAME_OUT] && enc_libpic->stat_lib.write > 0)
            {
                int end_of_seq = 0;
                if (write_data(op_fname_libout, enc_libpic->bs_buf_lib, enc_libpic->stat_lib.write, end_of_seq))
                {
                    v0print("cannot write bitstream\n");
                    return -1;
                }
            }
            /* get reconstructed image */
            enc_libpic->size = sizeof(COM_IMGB**);
            enc_libpic->imgb_rec_lib = PIC_REC((ENC_CTX *)enc_libpic->id_lib)->imgb;
            enc_libpic->imgb_rec_lib->addref(enc_libpic->imgb_rec_lib);
            if (COM_FAILED(enc_libpic->ret))
            {
                v0print("failed to get reconstruction image\n");
                return -1;
            }

            /* calculate PSNR */
#if CALC_SSIM
            if (cal_psnr(enc_libpic->ilist_org_lib, enc_libpic->imgb_rec_lib, enc_libpic->imgb_rec_lib->ts[0], enc_libpic->psnr_lib, enc_libpic->ms_ssim_lib))
#else
            if (cal_psnr(ilist_org_lib, imgb_rec_lib, imgb_rec_lib->ts[0], psnr_lib))
#endif
            {
                v0print("cannot calculate PSNR\n");
                return -1;
            }

            /* store reconstructed image to list only for writing out */
            enc_libpic->ilist_t_lib = store_rec_img(enc_libpic->ilist_rec_lib, enc_libpic->imgb_rec_lib, enc_libpic->imgb_rec_lib->ts[0], param_in.bit_depth_internal);
            if (enc_libpic->ilist_t_lib == NULL)
            {
                v0print("cannot put reconstructed image to list\n");
                return -1;
            }

            if (write_rec(enc_libpic->ilist_rec_lib, &(enc_libpic->pic_ocnt_lib), param_in.bit_depth_internal, op_flag[OP_FLAG_FNAME_LIBREC], op_fname_librec))
            {
                v0print("cannot write reconstruction image\n");
                return -1;
            }

            if (enc_libpic->is_first_enc_lib)
            {
                enc_libpic->is_first_enc_lib = 0;
            }

            enc_libpic->bitrate_lib += (enc_libpic->stat_lib.write - enc_libpic->udata_size_lib);
            for (int i = 0; i < 3; i++) enc_libpic->psnr_avg_lib[i] += enc_libpic->psnr_lib[i];
#if CALC_SSIM
            enc_libpic->ms_ssim_avg_lib[0] += enc_libpic->ms_ssim_lib[0];
#if CALC_SSIM_UV
            if (op_msssim_uv)
            {
                enc_libpic->ms_ssim_avg_lib[1] += enc_libpic->ms_ssim_lib[1];
                enc_libpic->ms_ssim_avg_lib[2] += enc_libpic->ms_ssim_lib[2];
            }
#endif
#endif
            /* release recon buffer */
            if (enc_libpic->imgb_rec_lib) enc_libpic->imgb_rec_lib->release(enc_libpic->imgb_rec_lib);
        }
        else if (enc_libpic->ret == COM_OK_NO_MORE_FRM)
        {
            break;
        }
        else
        {
            v2print("invaild return value (%d)\n", enc_libpic->ret);
            return -1;
        }
        if ((op_flag[OP_FLAG_MAX_FRM_NUM] || op_flag[OP_FLAG_MAX_FRM_NUM_WHEN_EXTRACT_LIBPIC]) && enc_libpic->pic_icnt_lib >= op_max_frm_num_when_extract_libpic
            && enc_libpic->state_lib == STATE_ENCODING)
        {
            enc_libpic->state_lib = STATE_BUMPING;
            setup_bumping(enc_libpic->id_lib);
        }
    }
    /* store remained reconstructed pictures in output list */
    while (enc_libpic->libpic_icnt - enc_libpic->pic_ocnt_lib > 0)
    {
        write_rec(enc_libpic->ilist_rec_lib, &enc_libpic->pic_ocnt_lib, param_in.bit_depth_internal, op_flag[OP_FLAG_FNAME_LIBREC], op_fname_librec);
    }
    if (enc_libpic->libpic_icnt != enc_libpic->pic_ocnt_lib)
    {
        v2print("number of input(=%d) and output(=%d) is not matched\n",
            (int)enc_libpic->libpic_icnt, (int)enc_libpic->pic_ocnt_lib);
    }

    enc_libpic->psnr_avg_lib[0] /= enc_libpic->pic_ocnt_lib;
    enc_libpic->psnr_avg_lib[1] /= enc_libpic->pic_ocnt_lib;
    enc_libpic->psnr_avg_lib[2] /= enc_libpic->pic_ocnt_lib;

#if CALC_SSIM
    enc_libpic->ms_ssim_avg_lib[0] /= enc_libpic->pic_ocnt_lib;
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        enc_libpic->ms_ssim_avg_lib[1] /= enc_libpic->pic_ocnt_lib;
        enc_libpic->ms_ssim_avg_lib[2] /= enc_libpic->pic_ocnt_lib;
    }
#endif
#endif
    enc_libpic->bits_libpic = enc_libpic->bitrate_lib * 8;
    libvc_data->bits_libpic = enc_libpic->bits_libpic;

    enc_libpic->bitrate_lib *= (param_in.fps * 8);
    enc_libpic->bitrate_lib /= enc_libpic->pic_ocnt_lib;
    enc_libpic->bitrate_lib /= 1000;
    return 0;
}
#endif
int encode_libpics(ENC_PARAM param_in, LibVCData* libvc_data)
{
    int                 libpic_icnt;
    double              bits_libpic;
    STATES              state_lib = STATE_ENCODING;
    unsigned char      *bs_buf_lib = NULL;
    unsigned char      *bs_buf_lib2 = NULL;
    FILE               *fp_inp_lib = NULL;
    ENC                 id_lib;
    COM_BITB            bitb_lib;
    COM_IMGB           *imgb_enc_lib = NULL;
    COM_IMGB           *imgb_rec_lib = NULL;
    ENC_STAT            stat_lib;
    int                 udata_size_lib = 0;
    int                 i, ret, size;
    COM_CLK             clk_beg_lib, clk_end_lib, clk_tot_lib;
    COM_MTIME           pic_icnt_lib, pic_ocnt_lib, pic_skip_lib;
    int                 num_encoded_frames = 0;
    double              bitrate_lib;
    double              psnr_lib[3] = { 0, };
    double              psnr_avg_lib[3] = { 0, };
#if CALC_SSIM
    double              ms_ssim_lib[3] = { 0, 0, 0 };
    double              ms_ssim_avg_lib[3] = { 0, 0, 0 };
#endif
    IMGB_LIST           ilist_org_lib[MAX_BUMP_FRM_CNT];
    IMGB_LIST           ilist_rec_lib[MAX_BUMP_FRM_CNT];
    IMGB_LIST          *ilist_t_lib = NULL;
    static int          is_first_enc_lib = 1;
#if !CALC_SSIM
    double              seq_header_bit = 0;
#endif
#if LINUX
    signal(SIGSEGV, handler);   // install our handler
#endif
    if (op_flag[OP_FLAG_FNAME_OUT])
    {
        /* libic bitstream file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_libout, "wb");
        if (fp == NULL)
        {
            v0print("cannot open libpic bitstream file (%s)\n", op_fname_libout);
            return -1;
        }
        fclose(fp);
    }
    if (op_flag[OP_FLAG_FNAME_LIBREC])
    {
        /* reconstruction libic file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_librec, "wb");
        if (fp == NULL)
        {
            v0print("cannot open reconstruction libpic file (%s)\n", op_fname_librec);
            return -1;
        }
        fclose(fp);
    }

    /* open original file */
    fp_inp_lib = fopen(op_fname_inp, "rb");
    if (fp_inp_lib == NULL)
    {
        v0print("cannot open original file (%s)\n", op_fname_inp);
        print_usage();
        return -1;
    }
    /* allocate bitstream buffer */
    bs_buf_lib = (unsigned char*)malloc(MAX_BS_BUF);
    if (bs_buf_lib == NULL)
    {
        v0print("cannot allocate bitstream buffer, size=%d", MAX_BS_BUF);
        return -1;
    }
    bs_buf_lib2 = (unsigned char*)malloc(MAX_BS_BUF);
    if (bs_buf_lib2 == NULL)
    {
        v0print("ERROR: cannot allocate bit buffer, size2=%d\n", MAX_BS_BUF);
        return -1;
    }
    /* create encoder */
    id_lib = enc_create(&param_in, NULL);
    if (id_lib == NULL)
    {
        v0print("cannot create encoder\n");
        return -1;
    }
    set_livcdata_enc(id_lib, libvc_data);

    if (set_extra_config(id_lib))
    {
        v0print("cannot set extra configurations\n");
        return -1;
    }
    /* create image lists */
    if (imgb_list_alloc(ilist_org_lib, param_in.pic_width, param_in.pic_height, param_in.horizontal_size, param_in.vertical_size, 10))
    {
        v0print("cannot allocate image list for original image\n");
        return -1;
    }
    if (imgb_list_alloc(ilist_rec_lib, param_in.pic_width, param_in.pic_height, param_in.horizontal_size, param_in.vertical_size, param_in.bit_depth_internal))
    {
        v0print("cannot allocate image list for reconstructed image\n");
        return -1;
    }

    bits_libpic = 0;
    bitrate_lib = 0;
    bitb_lib.addr  = bs_buf_lib;
    bitb_lib.addr2 = bs_buf_lib2;
    bitb_lib.bsize = MAX_BS_BUF;
#if WRITE_MD5_IN_USER_DATA //discount user data bits in per-frame bits
    udata_size_lib = (op_use_pic_signature) ? 23 : 0;
#endif
#if !PRECISE_BS_SIZE
    udata_size_lib += 4; /* 4-byte prefix (length field of chunk) */
#endif
    /* encode Sequence Header if needed **************************************/
    bitb_lib.err = 0; // update BSB
#if REPEAT_SEQ_HEADER
    ret = init_seq_header((ENC_CTX *)id_lib, &bitb_lib);
#else
    ret = enc_seq_header((ENC_CTX *)id_lib, &bitb_lib, &stat_lib);
    if (COM_FAILED(ret))
    {
        v0print("cannot encode header \n");
        return -1;
    }
    if (op_flag[OP_FLAG_FNAME_OUT])
    {
        /* write Sequence Header bitstream to file */
        if (write_data(op_fname_libout, bs_buf_lib, stat_lib.write, NOT_END_OF_VIDEO_SEQUENCE))
        {
            v0print("Cannot write header information (SQH)\n");
            return -1;
        }
#if PRECISE_BS_SIZE
        bitrate_lib += stat_lib.write;
#if !CALC_SSIM
        seq_header_bit = stat_lib.write;
#endif
#else
        bitrate_lib += (stat_lib.write - 4)/* 4-byte prefix (length field of chunk) */;
#if !CALC_SSIM
        seq_header_bit = (stat_lib.write - 4)/* 4-byte prefix (length field of chunk) */;
#endif
#endif
    }
#endif

    if ((op_flag[OP_FLAG_SKIP_FRAMES] && op_skip_frames > 0) || (op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC] && op_skip_frames_when_extract_libpic > 0))
    {
        state_lib = STATE_SKIPPING;
    }
    clk_tot_lib = 0;
    pic_icnt_lib = 0;
    pic_ocnt_lib = 0;
    pic_skip_lib = 0;

    libpic_icnt = 0;
    /* encode pictures *******************************************************/
    while (1)
    {
        if (state_lib == STATE_SKIPPING)
        {
            if (pic_skip_lib < op_skip_frames_when_extract_libpic)
            {
                ilist_t_lib = imgb_list_get_empty(ilist_org_lib);
                if (ilist_t_lib == NULL)
                {
                    v0print("cannot get empty orignal buffer\n");
                    goto ERR;
                }

                if (pic_icnt_lib == op_max_frm_num_when_extract_libpic || imgb_read_conv(fp_inp_lib, ilist_t_lib->imgb, param_in.bit_depth_input, param_in.bit_depth_internal))
                {
                    v2print("reached end of original file (or reading error)\n");
                    goto ERR;
                }
            }
            else
            {
                state_lib = STATE_ENCODING;
            }
            pic_skip_lib++;
            continue;
        }
        if (state_lib == STATE_ENCODING)
        {
            ilist_t_lib = imgb_list_get_empty(ilist_org_lib);
            if (ilist_t_lib == NULL)
            {
                v0print("cannot get empty orignal buffer\n");
                return -1;
            }
            /* read original image */
            if (pic_icnt_lib == op_max_frm_num_when_extract_libpic || imgb_read_conv(fp_inp_lib, ilist_t_lib->imgb, param_in.bit_depth_input, param_in.bit_depth_internal) || libpic_icnt == libvc_data->num_lib_pic)
            {
                v2print("reached end of original file (or reading error)\n");
                state_lib = STATE_BUMPING;
                setup_bumping(id_lib);
                continue;
            }
            else if (pic_icnt_lib != libvc_data->list_poc_of_libpic[libpic_icnt])
            {
                pic_icnt_lib++;
                continue;
            }

            imgb_list_make_used(ilist_t_lib, libpic_icnt);
            libpic_icnt++;

            /* get encodng buffer */
            //com_assert_rv((&imgb_enc) != NULL, COM_ERR_INVALID_ARGUMENT);
            if (COM_OK != enc_picbuf_get_inbuf((ENC_CTX *)id_lib, &imgb_enc_lib))
            {
                v0print("Cannot get original image buffer\n");
                return -1;
            }
            /* copy original image to encoding buffer */
            imgb_cpy_internal(imgb_enc_lib, ilist_t_lib->imgb);
            /* push image to encoder */
            ret = enc_push_frm((ENC_CTX *)id_lib, imgb_enc_lib);
            if (COM_FAILED(ret))
            {
                v0print("enc_push() failed\n");
                return -1;
            }
            /* release encoding buffer */
            imgb_enc_lib->release(imgb_enc_lib);
            pic_icnt_lib++;
        }
        /* encoding */
        clk_beg_lib = com_clk_get();
        ret = enc_encode(id_lib, &bitb_lib, &stat_lib);
        num_encoded_frames += ret == COM_OK;
        if (COM_FAILED(ret))
        {
            v0print("enc_encode() failed\n");
            return -1;
        }
        clk_end_lib = com_clk_from(clk_beg_lib);
        clk_tot_lib += clk_end_lib;
        /* store bitstream */
        if (ret == COM_OK_OUT_NOT_AVAILABLE)
        {
            v1print("--> RETURN OK BUT PICTURE IS NOT AVAILABLE YET\n");
            continue;
        }
        else if (ret == COM_OK)
        {
            if (op_flag[OP_FLAG_FNAME_OUT] && stat_lib.write > 0)
            {
                int end_of_seq = libpic_icnt == num_encoded_frames ? END_OF_VIDEO_SEQUENCE : NOT_END_OF_VIDEO_SEQUENCE;
                if (write_data(op_fname_libout, bs_buf_lib, stat_lib.write, end_of_seq))
                {
                    v0print("cannot write bitstream\n");
                    return -1;
                }
            }
            /* get reconstructed image */
            size = sizeof(COM_IMGB**);
            imgb_rec_lib = PIC_REC((ENC_CTX *)id_lib)->imgb;
            imgb_rec_lib->addref(imgb_rec_lib);
            if (COM_FAILED(ret))
            {
                v0print("failed to get reconstruction image\n");
                return -1;
            }

            /* calculate PSNR */
#if CALC_SSIM
            if (cal_psnr(ilist_org_lib, imgb_rec_lib, imgb_rec_lib->ts[0], psnr_lib, ms_ssim_lib))
#else
            if (cal_psnr(ilist_org_lib, imgb_rec_lib, imgb_rec_lib->ts[0], psnr_lib))
#endif
            {
                v0print("cannot calculate PSNR\n");
                return -1;
            }

            /* store reconstructed image to list only for writing out */
            ilist_t_lib = store_rec_img(ilist_rec_lib, imgb_rec_lib, imgb_rec_lib->ts[0], param_in.bit_depth_internal);
            if (ilist_t_lib == NULL)
            {
                v0print("cannot put reconstructed image to list\n");
                return -1;
            }

            if (write_rec(ilist_rec_lib, &pic_ocnt_lib, param_in.bit_depth_internal, op_flag[OP_FLAG_FNAME_LIBREC], op_fname_librec))
            {
                v0print("cannot write reconstruction image\n");
                return -1;
            }

            if (is_first_enc_lib)
            {
                is_first_enc_lib = 0;
            }

            bitrate_lib += (stat_lib.write - udata_size_lib);
            for (i = 0; i < 3; i++) psnr_avg_lib[i] += psnr_lib[i];
#if CALC_SSIM
            ms_ssim_avg_lib[0] += ms_ssim_lib[0];
#if CALC_SSIM_UV
            if (op_msssim_uv)
            {
                ms_ssim_avg_lib[1] += ms_ssim_lib[1];
                ms_ssim_avg_lib[2] += ms_ssim_lib[2];
            }
#endif
#endif
            /* release recon buffer */
            if (imgb_rec_lib) imgb_rec_lib->release(imgb_rec_lib);
        }
        else if (ret == COM_OK_NO_MORE_FRM)
        {
            break;
        }
        else
        {
            v2print("invaild return value (%d)\n", ret);
            return -1;
        }
        if ((op_flag[OP_FLAG_MAX_FRM_NUM] || op_flag[OP_FLAG_MAX_FRM_NUM_WHEN_EXTRACT_LIBPIC]) && pic_icnt_lib >= op_max_frm_num_when_extract_libpic
                && state_lib == STATE_ENCODING)
        {
            state_lib = STATE_BUMPING;
            setup_bumping(id_lib);
        }
    }
    /* store remained reconstructed pictures in output list */
    while (libpic_icnt - pic_ocnt_lib > 0)
    {
        write_rec(ilist_rec_lib, &pic_ocnt_lib, param_in.bit_depth_internal, op_flag[OP_FLAG_FNAME_LIBREC], op_fname_librec);
    }
    if (libpic_icnt != pic_ocnt_lib)
    {
        v2print("number of input(=%d) and output(=%d) is not matched\n",
                (int)libpic_icnt, (int)pic_ocnt_lib);
    }

    psnr_avg_lib[0] /= pic_ocnt_lib;
    psnr_avg_lib[1] /= pic_ocnt_lib;
    psnr_avg_lib[2] /= pic_ocnt_lib;

#if CALC_SSIM
    ms_ssim_avg_lib[0] /= pic_ocnt_lib;
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        ms_ssim_avg_lib[1] /= pic_ocnt_lib;
        ms_ssim_avg_lib[2] /= pic_ocnt_lib;
    }
#endif
#endif
    bits_libpic = bitrate_lib * 8;
    libvc_data->bits_libpic = bits_libpic;

    bitrate_lib *= (param_in.fps * 8);
    bitrate_lib /= pic_ocnt_lib;
    bitrate_lib /= 1000;

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    enc_delete(id_lib,1);
#else
    enc_delete(id_lib);
#endif
    imgb_list_free(ilist_org_lib);
    imgb_list_free(ilist_rec_lib);
    if (fp_inp_lib)  fclose(fp_inp_lib);
    if (bs_buf_lib)  free(bs_buf_lib); /* release bitstream buffer */
    if (bs_buf_lib2) free(bs_buf_lib2); /* release bitstream buffer */
    return 0;
ERR:
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    enc_delete(id_lib, 1);
#else
    enc_delete(id_lib);
#endif
    imgb_list_free(ilist_org_lib);
    imgb_list_free(ilist_rec_lib);
    if (fp_inp_lib)  fclose(fp_inp_lib);
    if (bs_buf_lib)  free(bs_buf_lib); /* release bitstream buffer */
    if (bs_buf_lib2) free(bs_buf_lib2); /* release bitstream buffer */
    return -1;
}
#endif

int main(int argc, const char **argv)
{
    STATES              state = STATE_ENCODING;
    unsigned char      *bs_buf = NULL;
    unsigned char      *bs_buf2 = NULL;
    FILE               *fp_inp = NULL;
    ENC                id;
    ENC_PARAM          param_input;
    COM_BITB           bitb;
    COM_IMGB          *imgb_enc = NULL; // used in the real encoding process (always 16-bit)
    COM_IMGB          *imgb_rec = NULL; // point to the real reconstructed picture (always 16-bit)
    ENC_STAT           stat;
    int                udata_size = 0;
    int                i, ret, size;
    COM_CLK            clk_beg, clk_end, clk_tot;
    COM_MTIME          pic_icnt, pic_ocnt, pic_skip;
#if IPPPCRR && LIB_PIC_UPDATE
    COM_MTIME          pic_lib_skip;
#endif
    int                num_encoded_frames = 0;
    double             bitrate;
    double             psnr[3] = {0, 0, 0};
    double             psnr_avg[3] = {0, 0 ,0};
#if CALC_SSIM
    double             ms_ssim[3] = { 0, 0, 0 };
    double             ms_ssim_avg[3] = { 0, 0, 0 };
#endif
    IMGB_LIST          ilist_org[MAX_BUMP_FRM_CNT]; // always 10-bit depth
    IMGB_LIST          ilist_rec[MAX_BUMP_FRM_CNT]; // which is only used for storing the reconstructed pictures to be written out, so the output bit depth is used for it
    IMGB_LIST         *ilist_t = NULL;
    static int         is_first_enc = 1;
#if !CALC_SSIM
    double             seq_header_bit = 0;
#endif
#if LINUX
    signal(SIGSEGV, handler);   // install our handler
#endif
#if ENC_DEC_TRACE
    fp_trace = NULL;
#endif

    srand((unsigned int)(time(NULL)));

    /* parse options */
    ret = com_args_parse_all(argc, argv, options);

    if(ret != 0)
    {
        if(ret > 0) v0print("-%c argument should be set\n", ret);
        if(ret < 0) v0print("Configuration error, please refer to the usage.\n");
        print_usage();
        return -1;
    }
    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        /* bitstream file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_out, "wb");
        if(fp == NULL)
        {
            v0print("cannot open bitstream file (%s)\n", op_fname_out);
            return -1;
        }
        fclose(fp);
    }
    if(op_flag[OP_FLAG_FNAME_REC])
    {
        /* reconstruction file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_rec, "wb");
        if(fp == NULL)
        {
            v0print("cannot open reconstruction file (%s)\n", op_fname_rec);
            return -1;
        }
        fclose(fp);
    }
    /* open original file */
    fp_inp = fopen(op_fname_inp, "rb");
    if(fp_inp == NULL)
    {
        v0print("cannot open original file (%s)\n", op_fname_inp);
        print_usage();
        return -1;
    }
    /* allocate bitstream buffer */
    bs_buf = (unsigned char*)malloc(MAX_BS_BUF);
    if(bs_buf == NULL)
    {
        v0print("cannot allocate bitstream buffer, size=%d", MAX_BS_BUF);
        return -1;
    }
    bs_buf2 = (unsigned char*)malloc(MAX_BS_BUF);
    if (bs_buf2 == NULL)
    {
        v0print("cannot allocate bitstream buffer, size=%d", MAX_BS_BUF);
        return -1;
    }
    /* read configurations and set values for create descriptor */
    if(get_conf(&param_input))
    {
        print_usage();
        return -1;
    }
#if LIBVC_ON
    int ori_flag = 0;
    if (!op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC])
    {
        op_skip_frames_when_extract_libpic = op_skip_frames;
    }
    if (!op_flag[OP_FLAG_MAX_FRM_NUM_WHEN_EXTRACT_LIBPIC])
    {
        op_max_frm_num_when_extract_libpic = param_input.frames_to_be_encoded;
    }

    LibVCData libvc_data;
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    ENC_LIBPIC enc_libpic;
    int rl_k = 0;
#endif
    init_libvcdata(&libvc_data);
    if (op_tool_libpic)
    {
        ori_flag = param_input.library_picture_enable_flag;
        libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag;

        ret = compute_LibVCData(param_input, &libvc_data);
        if (ret)
        {
            v0print("Error when compute LibVCData!");
            return -1;
        }
#if IPPPCRR && LIB_PIC_UPDATE
        if (op_lib_pic_update)
        {
            libvc_data.update = 1;
            int ori_iperiod = param_input.i_period;
            int ori_max_b_frames = param_input.max_b_frames;
            int ori_qp = param_input.qp;
            param_input.i_period = 1;
            param_input.max_b_frames = 0;
            param_input.qp = ori_qp + param_input.qp_offset_libpic;
            libvc_data.is_libpic_processing = 1;
            libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = 0;
            enc_prepare_LibVC(param_input, &libvc_data, &enc_libpic);
            // for slice parallel coding
            if (op_flag[OP_FLAG_SKIP_FRAMES] || op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC])
            {
                int k=0;
                for (k = libvc_data.num_RLpic - 1; k > 0; k--)
                {
                    if (libvc_data.list_poc_of_RLpic[k] > op_max_frm_num + op_skip_frames - op_skip_frames_when_extract_libpic - 1)
                    {
                        libvc_data.list_poc_of_RLpic[k] = -1;
                    }
                    else
                    {
                        break;
                    }
                }
                libvc_data.num_RLpic = k + 1;
            }
            param_input.i_period = ori_iperiod;
            param_input.max_b_frames = ori_max_b_frames;
            param_input.qp = ori_qp;
            libvc_data.is_libpic_processing = 0;
            libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = ori_flag;
            libvc_data.is_libpic_prepared = 1;
        }
        else
        {
#endif
            if (libvc_data.num_lib_pic > 0)
            {
                //lib pic is encoded as AllIntra
                int ori_iperiod = param_input.i_period;
                int ori_max_b_frames = param_input.max_b_frames;
                int ori_qp = param_input.qp;
                param_input.i_period = 1;
                param_input.max_b_frames = 0;
                param_input.qp = ori_qp + param_input.qp_offset_libpic;

                libvc_data.is_libpic_processing = 1;
                libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = 0;
                int err = encode_libpics(param_input, &libvc_data);
                if (err)
                {
                    v0print("Error when encode lib pic!");
                    return -1;
                }

                // for slice parallel coding
                if (op_flag[OP_FLAG_SKIP_FRAMES] || op_flag[OP_FLAG_SKIP_FRAMES_WHEN_EXTRACT_LIBPIC])
                {
                    for (int k = 0; k < libvc_data.num_RLpic; k++)
                    {

                        int RLpoc_in_seq = libvc_data.list_poc_of_RLpic[k] + op_skip_frames_when_extract_libpic - op_skip_frames;
                        if (RLpoc_in_seq<0 || RLpoc_in_seq>op_max_frm_num - 1)
                        {
                            libvc_data.list_poc_of_RLpic[k] = -1;
                        }
                        else
                        {
                            libvc_data.list_poc_of_RLpic[k] = RLpoc_in_seq;
                        }
                    }
                }

                // restore the following param for sequence pic encoding
                param_input.i_period = ori_iperiod;
                param_input.max_b_frames = ori_max_b_frames;
                param_input.qp = ori_qp;

                libvc_data.is_libpic_processing = 0;
                libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = ori_flag;
                libvc_data.is_libpic_prepared = 1;
            }
            else
            {
                //no need to use libpic
                v2print("\nWarning: There is no library picture extracted, so no need to use libpic! Probably because there are too few I frames!");
                libvc_data.bits_dependencyFile = 0;
                libvc_data.bits_libpic = 0;
                libvc_data.is_libpic_processing = 0;
                libvc_data.is_libpic_prepared = 0;
            }
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
        }
#endif
    }
#endif
    /* create encoder */
    id = enc_create(&param_input, NULL);
    if (id == NULL)
    {
        v0print("cannot create encoder\n");
        return -1;
    }
#if LIBVC_ON
    set_livcdata_enc(id, &libvc_data);
#endif
    if (set_extra_config(id))
    {
        v0print("cannot set extra configurations\n");
        return -1;
    }
    /* create image lists */
    if(imgb_list_alloc(ilist_org, param_input.pic_width, param_input.pic_height, param_input.horizontal_size, param_input.vertical_size, 10))
    {
        v0print("cannot allocate image list for original image\n");
        return -1;
    }
    if(imgb_list_alloc(ilist_rec, param_input.pic_width, param_input.pic_height, param_input.horizontal_size, param_input.vertical_size, param_input.bit_depth_internal))
    {
        v0print("cannot allocate image list for reconstructed image\n");
        return -1;
    }
    print_config(id, param_input);
    print_stat_init();
#if LIBVC_ON
#if IPPPCRR && LIB_PIC_UPDATE
    if (!op_lib_pic_update)
#endif
    if (op_tool_libpic)
    {
        print_libenc_data(libvc_data);
    }
#endif

    bitrate = 0;
    bitb.addr  = bs_buf;
    bitb.addr2 = bs_buf2;
    bitb.bsize = MAX_BS_BUF;
#if WRITE_MD5_IN_USER_DATA //discount user data bits in per-frame bits
    udata_size = (op_use_pic_signature) ? 23 : 0;
#endif
#if !PRECISE_BS_SIZE
    udata_size += 4; /* 4-byte prefix (length field of chunk) */
#endif
    /* encode Sequence Header if needed **************************************/
    bitb.err = 0; // update BSB
#if REPEAT_SEQ_HEADER
    ret = init_seq_header( (ENC_CTX *)id, &bitb);
#else
    ret = enc_seq_header((ENC_CTX *)id, &bitb, &stat);
    if(COM_FAILED(ret))
    {
        v0print("cannot encode header \n");
        return -1;
    }
    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        /* write Sequence Header bitstream to file */

        if (write_data(op_fname_out, bs_buf,  stat.write, NOT_END_OF_VIDEO_SEQUENCE))
        {
            v0print("Cannot write header information (SQH)\n");
            return -1;
        }
#if PRECISE_BS_SIZE
        bitrate += stat.write;
#if !CALC_SSIM
        seq_header_bit = stat.write;
#endif
#else
        bitrate += (stat.write - 4)/* 4-byte prefix (length field of chunk) */;
#if !CALC_SSIM
        seq_header_bit = (stat.write - 4)/* 4-byte prefix (length field of chunk) */;
#endif
#endif
    }
#endif
    if(op_flag[OP_FLAG_SKIP_FRAMES] && op_skip_frames > 0)
    {
        state = STATE_SKIPPING;
    }
    clk_tot = 0;
    pic_icnt = 0;
    pic_ocnt = 0;
    pic_skip = 0;
    g_DOIPrev = 0;
#if IPPPCRR && LIB_PIC_UPDATE
    pic_lib_skip = 0;
#endif

    /* encode pictures *******************************************************/
    while(1)
    {
        print_flush(stdout);
        if(state == STATE_SKIPPING)
        {
#if IPPPCRR && LIB_PIC_UPDATE
            if ((op_lib_pic_update&&pic_skip+1 < op_skip_frames_when_extract_libpic)
                || (!op_lib_pic_update&&pic_skip < op_skip_frames))
            {
#else
            if(pic_skip < op_skip_frames)
            {
#endif
                ilist_t = imgb_list_get_empty(ilist_org);
                if(ilist_t == NULL)
                {
                    v0print("cannot get empty orignal buffer\n");
                    goto ERR;
                }

                if (pic_icnt == param_input.frames_to_be_encoded || imgb_read_conv(fp_inp, ilist_t->imgb, param_input.bit_depth_input, param_input.bit_depth_internal))
                {
                    v2print("reached end of original file (or reading error)\n");
                    goto ERR;
                }
            }
            else
            {

#if IPPPCRR && LIB_PIC_UPDATE
                rl_k = 0;
                if (op_lib_pic_update&&pic_skip < op_skip_frames)
                {
                    state = STATE_LIB_SKIPPING;
                }
                else
                    state = STATE_ENCODING;
#else
                state = STATE_ENCODING;
#endif
            }
            pic_skip++;
            continue;
        }
        if (state == STATE_ENCODING)
        {
#if IPPPCRR && LIB_PIC_UPDATE
            libvc_data.encode_skip = 0;
#endif
            ilist_t = imgb_list_get_empty(ilist_org);
            if (ilist_t == NULL)
            {
                v0print("cannot get empty orignal buffer\n");
                return -1;
            }
            /* read original image */
            if (pic_icnt == param_input.frames_to_be_encoded || imgb_read_conv(fp_inp, ilist_t->imgb, param_input.bit_depth_input, param_input.bit_depth_internal))
            {
                v2print("reached end of original file (or reading error)\n");
                state = STATE_BUMPING;
                setup_bumping(id);
                continue;
            }
            skip_frames(fp_inp, ilist_t->imgb, param_input.sub_sample_ratio - 1, param_input.bit_depth_input);
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
            if (op_lib_pic_update)
            {
            if (pic_lib_skip + pic_icnt + 1 >= libvc_data.list_poc_of_RLpic[rl_k] && libvc_data.list_poc_of_RLpic[rl_k] >= 0)
            {
                if (pic_icnt + pic_lib_skip > 0 || (pic_lib_skip + pic_icnt + 1 >= libvc_data.list_poc_of_RLpic[rl_k + 1]&& libvc_data.list_poc_of_RLpic[rl_k+1] >= 0))
                    libvc_data.end_of_intra_period = 1;
                rl_k++;
            }
            else  libvc_data.end_of_intra_period = 0;
        }
#endif
            imgb_list_make_used(ilist_t, pic_icnt);

            /* get encoding buffer */
            if(COM_OK != enc_picbuf_get_inbuf((ENC_CTX *)id, &imgb_enc))
            {
                v0print("Cannot get original image buffer\n");
                return -1;
            }
            /* copy original image to encoding buffer */
            imgb_cpy_internal(imgb_enc, ilist_t->imgb);
            /* push image to encoder */
            ret = enc_push_frm((ENC_CTX *)id, imgb_enc);
            if(COM_FAILED(ret))
            {
                v0print("enc_push() failed\n");
                return -1;
            }
            /* release encoding buffer */
            imgb_enc->release(imgb_enc);
            pic_icnt++;
        }
        /* encoding */
        clk_beg = com_clk_get();

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE

        if (state == STATE_LIB_SKIPPING)
        {
            if (pic_skip < op_skip_frames)
            {
                ilist_t = imgb_list_get_empty(ilist_org);
                if (ilist_t == NULL)
                {
                    v0print("cannot get empty orignal buffer\n");
                    goto ERR;
                }

                if (pic_icnt == param_input.frames_to_be_encoded || imgb_read_conv(fp_inp, ilist_t->imgb, param_input.bit_depth_input, param_input.bit_depth_internal))
                {
                    v2print("reached end of original file (or reading error)\n");
                    goto ERR;
                }            
                    skip_frames(fp_inp, ilist_t->imgb, param_input.sub_sample_ratio - 1, param_input.bit_depth_input);
                    if (pic_lib_skip+1 >= libvc_data.list_poc_of_RLpic[rl_k] && libvc_data.list_poc_of_RLpic[rl_k] >= 0)
                    {
                        if (pic_lib_skip > 0 || (pic_lib_skip + 1 >= libvc_data.list_poc_of_RLpic[rl_k + 1]))
                        libvc_data.end_of_intra_period = 1;
                        rl_k++;
                    }
                    else  libvc_data.end_of_intra_period = 0;
                /* get encoding buffer */
                if (COM_OK != enc_picbuf_get_inbuf((ENC_CTX *)id, &imgb_enc))
                {
                    v0print("Cannot get original image buffer\n");
                    return -1;
                }
                /* copy original image to encoding buffer */
                imgb_cpy_internal(imgb_enc, ilist_t->imgb);
                enc_free((ENC_CTX *)id);
                /* push image to encoder */
                ret = enc_push_frm((ENC_CTX *)id, imgb_enc);
                if (COM_FAILED(ret))
                {
                    v0print("enc_push() failed\n");
                    return -1;
                }
                /* release encoding buffer */
                imgb_enc->release(imgb_enc);
                pic_lib_skip++;
                pic_skip += param_input.sub_sample_ratio;
                libvc_data.encode_skip = 1;
            }
            else
            {
                state = STATE_ENCODING;
                continue;
            }

        }
        ret = enc_encode(id, &bitb, &stat);
        if (ret == RL_UPDATE_TO_LIBPIC)
        {
            {
                libvc_data.num_lib_pic++;
                libvc_data.list_poc_of_libpic[libvc_data.num_lib_pic - 1] = libvc_data.list_poc_of_RLpic[rl_k-1];
                for (int k = 0; k < libvc_data.num_RLpic; k++)
                {
                    libvc_data.list_libidx_for_RLpic[k] = libvc_data.num_lib_pic - 1;
                }
                //lib pic is encoded as AllIntra
                int ori_iperiod = param_input.i_period;
                int ori_max_b_frames = param_input.max_b_frames;
                int ori_qp = param_input.qp;
                int lib_skip = libvc_data.encode_skip;
                param_input.i_period = 1;
                param_input.max_b_frames = 0;
                param_input.qp = ori_qp + param_input.qp_offset_libpic;

                libvc_data.encode_skip = 0;
                libvc_data.is_libpic_processing = 1;
                libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = 0;
                enc_libpic.state_lib = STATE_ENCODING;
                int err = encode_one_libpic(param_input, &libvc_data, &enc_libpic);
    
                if (err)
                {
                    v0print("Error when encode lib pic!");
                    return -1;
                }
                libvc_data.encode_skip = lib_skip;
                // restore the following param for sequence pic encoding
                param_input.i_period = ori_iperiod;
                param_input.max_b_frames = ori_max_b_frames;
                param_input.qp = ori_qp;

                libvc_data.is_libpic_processing = 0;
                libvc_data.library_picture_enable_flag = param_input.library_picture_enable_flag = ori_flag;
                libvc_data.is_libpic_prepared = 1;
                print_libenc_data(libvc_data);
            }
            libvc_data.update = 0;
            ret = enc_encode(id, &bitb, &stat);
        }
        if (ret == COM_OK_SKIP)
            continue;
#else
        ret = enc_encode(id, &bitb, &stat);
#endif
        num_encoded_frames += ret == COM_OK;
        if(COM_FAILED(ret))
        {
            v0print("enc_encode() failed\n");
            return -1;
        }
        clk_end = com_clk_from(clk_beg);
        clk_tot += clk_end;
        /* store bitstream */
        if (ret == COM_OK_OUT_NOT_AVAILABLE)
        {
            v1print("--> RETURN OK BUT PICTURE IS NOT AVAILABLE YET\n");
            continue;
        }
        else if (ret == COM_OK)
        {
            if (op_flag[OP_FLAG_FNAME_OUT] && stat.write > 0)
            {
                int end_of_seq = (param_input.frames_to_be_encoded == num_encoded_frames) ? END_OF_VIDEO_SEQUENCE : NOT_END_OF_VIDEO_SEQUENCE;
                if (write_data(op_fname_out, bs_buf, stat.write, end_of_seq))
                {
                    v0print("cannot write bitstream\n");
                    return -1;
                }
            }
            /* get reconstructed image */
            size = sizeof(COM_IMGB**);
            imgb_rec = PIC_REC((ENC_CTX *)id)->imgb;
            imgb_rec->addref(imgb_rec);
            if (COM_FAILED(ret))
            {
                v0print("failed to get reconstruction image\n");
                return -1;
            }

            /* calculate PSNR */
#if CALC_SSIM
            if (cal_psnr(ilist_org, imgb_rec, imgb_rec->ts[0], psnr, ms_ssim))
#else
            if(cal_psnr(ilist_org, imgb_rec, imgb_rec->ts[0], psnr))
#endif
            {
                v0print("cannot calculate PSNR\n");
                return -1;
            }

            /* store reconstructed image to list only for writing out */
            ilist_t = store_rec_img(ilist_rec, imgb_rec, imgb_rec->ts[0], param_input.bit_depth_internal);
            if (ilist_t == NULL)
            {
                v0print("cannot put reconstructed image to list\n");
                return -1;
            }

            if (write_rec(ilist_rec, &pic_ocnt, param_input.bit_depth_internal, op_flag[OP_FLAG_FNAME_REC], op_fname_rec))
            {
                v0print("cannot write reconstruction image\n");
                return -1;
            }

            if(is_first_enc)
            {
#if CALC_SSIM
                print_psnr(&stat, psnr, ms_ssim, (stat.write - udata_size + (int)bitrate) << 3, clk_end);
#else
                print_psnr(&stat, psnr, (stat.write - udata_size + (int)seq_header_bit) << 3, clk_end);
#endif
                is_first_enc = 0;
            }
            else
            {
#if CALC_SSIM
                print_psnr(&stat, psnr, ms_ssim, (stat.write - udata_size) << 3, clk_end);
#else
                print_psnr(&stat, psnr, (stat.write - udata_size) << 3, clk_end);
#endif
            }
            bitrate += (stat.write - udata_size);
            for(i=0; i<3; i++) psnr_avg[i] += psnr[i];
#if CALC_SSIM
            ms_ssim_avg[0] += ms_ssim[0];
#if CALC_SSIM_UV
            if( op_msssim_uv )
            {
                ms_ssim_avg[1] += ms_ssim[1];
                ms_ssim_avg[2] += ms_ssim[2];
            }
#endif
#endif
            /* release recon buffer */
            if(imgb_rec) imgb_rec->release(imgb_rec);
        }
        else if (ret == COM_OK_NO_MORE_FRM)
        {
            break;
        }
        else
        {
            v2print("invaild return value (%d)\n", ret);
            return -1;
        }
        if(op_flag[OP_FLAG_MAX_FRM_NUM] && pic_icnt >= param_input.frames_to_be_encoded
                && state == STATE_ENCODING)
        {
            state = STATE_BUMPING;
            setup_bumping(id);
        }
    }
    /* store remained reconstructed pictures in output list */
    while(pic_icnt - pic_ocnt > 0)
    {
        write_rec(ilist_rec, &pic_ocnt, param_input.bit_depth_internal, op_flag[OP_FLAG_FNAME_REC], op_fname_rec);
    }

    if(pic_icnt != pic_ocnt)
    {
        v2print("number of input(=%d) and output(=%d) is not matched\n",
                (int)pic_icnt, (int)pic_ocnt);
    }
    v1print("===============================================================================\n");
    psnr_avg[0] /= max( 1, pic_ocnt );
    psnr_avg[1] /= max( 1, pic_ocnt );
    psnr_avg[2] /= max( 1, pic_ocnt );
#if CALC_SSIM
    ms_ssim_avg[0] /= max( 1, pic_ocnt );
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        ms_ssim_avg[1] /= max(1, pic_ocnt);
        ms_ssim_avg[2] /= max(1, pic_ocnt);
    }
#endif
#endif
    v1print("  PSNR Y(dB)       : %-5.4f\n", psnr_avg[0]);
    v1print("  PSNR U(dB)       : %-5.4f\n", psnr_avg[1]);
    v1print("  PSNR V(dB)       : %-5.4f\n", psnr_avg[2]);
#if CALC_SSIM
    v1print("  MsSSIM_Y         : %-8.7f\n", ms_ssim_avg[0]);
#if CALC_SSIM_UV
    if (op_msssim_uv)
    {
        v1print("  MsSSIM_U         : %-8.7f\n", ms_ssim_avg[1]);
        v1print("  MsSSIM_V         : %-8.7f\n", ms_ssim_avg[2]);
    }
#endif
#endif

#if LIBVC_ON
    if (op_tool_libpic)
    {
        bitrate = bitrate * 8 + libvc_data.bits_dependencyFile + libvc_data.bits_libpic;
    }
    else
    {
        bitrate = bitrate * 8;
    }
    v1print("  Total bits(bits) : %-.0f\n", bitrate);
    bitrate *= param_input.fps;
#else
    v1print("  Total bits(bits) : %-.0f\n", bitrate * 8);
    bitrate *= (param_input.fps * 8);
#endif
    bitrate /= max( 1, pic_ocnt );
    bitrate /= 1000;
    v1print("  bitrate(kbps)    : %-5.4f\n", bitrate);
    v1print("===============================================================================\n");
    v1print("Encoded frame count               = %d\n", (int)pic_ocnt);
    v1print("Total encoding time               = %.3f msec,",
            (float)com_clk_msec(clk_tot));
    v1print(" %.3f sec\n", (float)(com_clk_msec(clk_tot)/1000.0));
    v1print("Average encoding time for a frame = %.3f msec\n",
            (float)com_clk_msec(clk_tot)/ max( 1, pic_ocnt ) );
    v1print("Average encoding speed            = %.3f frames/sec\n",
            ((float)pic_ocnt*1000)/((float)com_clk_msec(max(1,clk_tot))));
    v1print("===============================================================================\n");
    print_flush(stdout);
ERR:
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    enc_delete(id,1);
    if (op_lib_pic_update)
    {
        free_enc_libpic(&enc_libpic);
    }
#else
    enc_delete(id);
#endif
    imgb_list_free(ilist_org);
    imgb_list_free(ilist_rec);
    if(fp_inp) fclose(fp_inp);
    if(bs_buf) free(bs_buf); /* release bitstream buffer */
#if LIBVC_ON
    delete_libvcdata(&libvc_data);
#endif
#if ENC_DEC_TRACE
    if( fp_trace )
    {
        fclose( fp_trace );
        fp_trace = NULL;
    }
#endif
    return 0;
}
