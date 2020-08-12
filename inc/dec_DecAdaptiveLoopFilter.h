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

#ifndef DEC_DECADAPTIVELOOPFILTER_H
#define DEC_DECADAPTIVELOOPFILTER_H

#include "com_def.h"
#include "com_ComAdaptiveLoopFilter.h"
#include  "dec_def.h"
#include <math.h>

void CreateAlfGlobalBuffer(DEC_CTX *ctx);
void ReleaseAlfGlobalBuffer(DEC_CTX *ctx);

void ALFProcess_dec(DEC_CTX* ctx, ALFParam **alfParam, COM_PIC* pic_Rec, COM_PIC* pic_alf_Dec);
void filterOneCTB(DecALFVar * Dec_ALF, pel *pRest, pel *pDec, int stride, int compIdx, int bit_depth, ALFParam *alfParam, int ctuYPos, int ctuHeight,
                  int ctuXPos, int ctuWidth,
                  BOOL isAboveAvail, BOOL isBelowAvail, BOOL isLeftAvail, BOOL isRightAvail, BOOL isAboveLeftAvail,
                  BOOL isAboveRightAvail, int sample_bit_depth);
void deriveLoopFilterBoundaryAvailibility(DEC_CTX *ctx, int numLCUInPicWidth, int numLCUInPicHeight, int ctu,
        BOOL *isLeftAvail, BOOL *isRightAvail, BOOL *isAboveAvail, BOOL *isBelowAvail);

void deriveBoundaryAvail(DEC_CTX *ctx, int numLCUInPicWidth, int numLCUInPicHeight, int ctu, BOOL *isLeftAvail,
                         BOOL *isRightAvail, BOOL *isAboveAvail, BOOL *isBelowAvail, BOOL *isAboveLeftAvail, BOOL *isAboveRightAvail);

void AllocateAlfPar(ALFParam **alf_par, int cID);
void freeAlfPar(ALFParam *alf_par, int cID);
void allocateAlfAPS(ALF_APS *pAPS);
void freeAlfAPS(ALF_APS *pAPS);

#endif