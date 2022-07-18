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

#ifndef IMPEGHE_HOA_ASSIGNMENT_INFO_H
#define IMPEGHE_HOA_ASSIGNMENT_INFO_H

typedef struct
{
  WORD32 amb_coeff_idx_changed;
  UWORD32 amb_coeff_idx_transition_state;
  UWORD32 amb_coeff_idx;
  UWORD32 active_dir_ids;
} impeghe_hoa_info_ch_str;

typedef struct
{
  UWORD16 us_bits_q;
  WORD32 p_flag;
  WORD32 cb_flag;
  WORD32 same_header_prev_frame;
  WORD32 new_ch_type_one;
  UWORD32 indices_code_book_idx;
  UWORD32 max_v_vec_dir;
  UWORD32 v_vec_dir;
  UWORD32 weighting_code_book_idx;
  UWORD32 ui_8bit_coded_v_element[MAX_NUM_HOA_COEFFS];
} impeghe_hoa_vector_based_info_ch_str;

typedef struct
{
  WORD32 is_available;
  UWORD32 bit_rate;
  WORD32 ch_type;
  impeghe_hoa_vector_based_info_ch_str vec_based_ch_info;
  impeghe_hoa_info_ch_str ch_info;
} ia_spatial_enc_assignment_info_str;

VOID impeghe_hoa_assignment_info_init(ia_spatial_enc_assignment_info_str *pstr_hoa);

#endif /*IMPEGHE_HOA_ASSIGNMENT_INFO_H*/
