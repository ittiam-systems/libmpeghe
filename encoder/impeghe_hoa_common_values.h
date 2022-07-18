/* 	Copyright (c) [2022] Ittiam Systems Pvt. Ltd.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted (subject to the limitations in the
   disclaimer below) provided that the following conditions are met:
   •	Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   •	Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------
*/

#ifndef IMPEGHE_HOA_COMMON_VALUES_H
#define IMPEGHE_HOA_COMMON_VALUES_H
#define HOA_ORDER (6)
#define MAX_NUM_PERC_CODERS (24)
#define MAX_MATRIX_SIZE (10000)
#define MAX_NUM_SIG_INDICES (24)
#define MAX_NUM_DIR_SIGNAL (24)
#define MAX_NUM_VECTOR_SIG_INDICES (24)
#define MAX_NUM_DIRS (900)
#define MAX_SET_SIZE (100)
#define MAX_NUM_PAR_SUBBANDS (64)
#define MAX_NUMBER_CHANNELS (24)
#define MAX_NUM_HOA_DIR_SIGNALS (16)

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define MAX_NUM_HOA_COEFFS (HOA_ORDER + 1) * (HOA_ORDER + 1)
#define MAX_FRAME_LEN (1024)
#define MAX_NUM_AMB_COEFF_INDICES (24)
#define MAX_HOA_ORDER_FOR_AMB (3)
#define MAX_NUM_HOA_COEFFS_FOR_AMB (MAX_HOA_ORDER_FOR_AMB + 1) * (MAX_HOA_ORDER_FOR_AMB + 1)
#define MAX_HOA_DUMMY_FRAMES (6)

#define HOA_DIR_CHANNEL (0)
#define HOA_VEC_CHANNEL (1)
#define HOA_ADD_HOA_CHANNEL (2)
#define HOA_EMPTY_CHANNEL (3)

#define HOA_EMPTY_DECISION (2)

#define MAX_VEC_CHANNELS (8)
#define MAX_HOA_INPUT_RANGE (3.00f)
#define HOA_MAX_BS_TRANS_FRAME_OFFSET (3)
#define MAX_HOA_MATRIX_LEN (2 * ((1 << 8) - 1) + ((1 << 12) - 1))
#define MAX_NUM_RENDERER_MATRIX (6)

#endif /*IMPEGHE_HOA_COMMON_VALUES_H*/
