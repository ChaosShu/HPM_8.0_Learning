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

#ifndef __ENC_IBC_HASH_WRAPPER_H
#define __ENC_IBC_HASH_WRAPPER_H

#include <stdint.h>
#include "enc_def.h"

#if USE_IBC

#ifdef __cplusplus
extern "C" {
#endif

typedef void *ibc_hash_handle;

ibc_hash_handle* create_enc_IBC(int picWidth, int picHeight);
void destroy_enc_IBC(ibc_hash_handle* p);

void rebuild_hashmap(ibc_hash_handle* p, const COM_PIC* pic);

#if IBC_BVP
u32 search_ibc_hash_match(ENC_CORE *core, ENC_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
    int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D], u8 *bvp_idx);
#else
u32 search_ibc_hash_match(ENC_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
    int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D]);
#endif

int get_hash_hit_ratio(ENC_CTX* ctx, ibc_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh);
#if USE_IBC
u8  getHashHitRatio(ENC_CTX* ctx, ibc_hash_handle* p);
#endif
#ifdef __cplusplus
}
#endif
#endif

#endif /* end of __ENC_IBC_HASH_WRAPPER_H */
