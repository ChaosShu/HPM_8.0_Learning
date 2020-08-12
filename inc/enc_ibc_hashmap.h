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

/** \file     enc_ibc_hashmap.h
    \brief    IBC hash map encoder class (header)
*/

#ifndef __ENC_IBC_HASHMAP__
#define __ENC_IBC_HASHMAP__

#include "enc_def.h"

#if  USE_IBC
#undef max
#undef min
#include <unordered_map>
#include <vector>

// ====================================================================================================================
// Class definition
// ====================================================================================================================
typedef std::pair<int, int> Position;

class ENC_IBC_HashMap
{
private:
    int     m_picWidth;
    int     m_picHeight;

    unsigned int xxCalcBlockHash(const pel* pel, const int stride, const int width, const int height, unsigned int crc);

    void    xxBuildPicHashMap(const COM_PIC* pic);

    static  unsigned int xxComputeCrc32c16bit(unsigned int crc, const pel pel);

public:
    unsigned int**  m_pos2Hash;
    std::unordered_map<unsigned int, std::vector<Position>> m_hash2Pos;

public:
    unsigned int(*m_computeCrc32c) (unsigned int crc, const pel pel);

    ENC_IBC_HashMap();
    virtual ~ENC_IBC_HashMap();

    void    init(const int picWidth, const int picHeight);
    void    destroy();
    void    rebuildPicHashMap(const COM_PIC* pic);
    bool    ibcHashMatch(ENC_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
        std::vector<Position>& cand, const int maxCand, const int searchRange4SmallBlk);
};

#endif
#endif // __ENC_IBC_HASHMAP__
