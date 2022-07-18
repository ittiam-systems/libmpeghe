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

#ifndef IMPEGHE_AVQ_ENC_H
#define IMPEGHE_AVQ_ENC_H

#define NB_SPHERE 32
#define LEN_ABS_LEADER 37
#define LEN_SIGN_LEADER 226

VOID impeghe_find_nearest_neighbor(FLOAT32 *bk, WORD32 *ck);
VOID impeghe_apply_voronoi_ext(WORD32 *x, WORD32 *n, WORD32 *idx, WORD32 *k);
VOID impeghe_alg_vec_quant(FLOAT32 *ptr_input, WORD32 *ptr_out, WORD32 *ptr_lpc_idx);

extern const WORD32 impeghe_pow2_table[8];
extern const WORD32 impeghe_factorial_table[8];
extern const WORD32 impeghe_iso_code_index_table[LEN_ABS_LEADER];
extern const UWORD8 impeghe_iso_code_data_table[LEN_SIGN_LEADER];
extern const UWORD32 impeghe_signed_leader_is[LEN_SIGN_LEADER];
extern const WORD32 impeghe_da_nq[], impeghe_da_pos[], impeghe_da_num_bits[];
extern const UWORD32 impeghe_da_id[];
extern const FLOAT32 impeghe_wlsf_factor_table[4];
extern const FLOAT32 impeghe_dico_lsf_abs_8b_flt[16 * 256];

#endif /*IMPEGHE_AVQ_ENC_H*/
