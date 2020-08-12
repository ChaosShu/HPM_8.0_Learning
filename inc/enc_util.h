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

#ifndef _ENC_UTIL_H_
#define _ENC_UTIL_H_

#define PIC_ORG(ctx)             ((ctx)->pic[PIC_IDX_ORG])
#define PIC_REC(ctx)             ((ctx)->pic[PIC_IDX_REC])

void enc_picbuf_expand(ENC_CTX *ctx, COM_PIC *pic);

COM_PIC * com_pic_alloc(PICBUF_ALLOCATOR *pa, int *ret);
void com_pic_free(PICBUF_ALLOCATOR *pa, COM_PIC *pic);

void enc_bsw_skip_slice_size(COM_BSW *bs);
void enc_bsw_write_slice_size(COM_BSW *bs);

void enc_diff_pred(int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC *org, pel pred[N_C][MAX_CU_DIM], s16 diff[N_C][MAX_CU_DIM]);


#define SBAC_STORE(dst, src) com_mcpy(&dst, &src, sizeof(ENC_SBAC))
#define SBAC_LOAD(dst, src)  com_mcpy(&dst, &src, sizeof(ENC_SBAC))

int enc_create_cu_data(ENC_CU_DATA *cu_data, int cu_width_log2, int cu_height_log2);
int enc_delete_cu_data(ENC_CU_DATA *cu_data);

#endif /* _ENC_UTIL_H_ */
