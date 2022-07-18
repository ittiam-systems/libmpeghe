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

#ifndef IMPEGHE_ARITH_ENC_H
#define IMPEGHE_ARITH_ENC_H

typedef struct
{
  WORD32 low;
  WORD32 high;
  WORD32 value;
} impeghe_state_arith;

WORD32 impeghe_arith_enc_spec(ia_bit_buf_struct *it_bit_buf, WORD32 window_sequence,
                              WORD32 *ptr_x_ac_enc, WORD32 max_spec_coefficients,
                              WORD32 *ptr_c_pres, WORD32 *ptr_c_prev, WORD32 *ptr_size_prev,
                              WORD32 arith_reset_flag, impeghe_scratch_mem *str_scratch);
WORD32 impeghe_tcx_coding(ia_bit_buf_struct *it_bit_buff, WORD32 tcx_size, WORD32 max_tcx_size,
                          WORD32 *ptr_quant, WORD32 *c_pres, WORD32 *c_prev,
                          impeghe_scratch_mem *pstr_scratch);
WORD32 impeghe_arith_done(ia_bit_buf_struct *it_bit_buff, WORD32 bit_pos, impeghe_state_arith *s);

WORD32 impeghe_arith_encode(ia_bit_buf_struct *it_bit_buff, WORD32 bit_pos,
                            impeghe_state_arith *state, WORD32 symbol, UWORD16 const *cum_freq);

#endif /* IMPEGHE_ARITH_ENC_H */
