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

#ifndef IMPEGHE_DRC_TABLES_H
#define IMPEGHE_DRC_TABLES_H

extern const FLOAT32 impeghe_downmix_coeff[16];
extern const FLOAT32 impeghe_downmix_coeff_lfe[16];
extern const FLOAT32 impeghe_channel_weight[16];
extern const FLOAT32 impeghe_downmix_coeff_v1[32];
extern const FLOAT32 impeghe_eq_slope_table[16];
extern const FLOAT32 impeghe_eq_gain_delta_table[32];
extern const FLOAT32 impeghe_zero_pole_radius_table[128];
extern const FLOAT32 impeghe_zero_pole_angle_table[128];

typedef struct
{
  WORD32 size;
  WORD32 code;
  WORD32 value;
} ia_drc_delta_time_code_table_entry_struct;

typedef struct
{
  WORD32 size;
  WORD32 code;
  FLOAT32 value;
  WORD32 index;
} ia_drc_slope_code_table_entry_struct;

typedef struct
{
  WORD32 size;
  WORD32 code;
  FLOAT32 value;
} ia_drc_delta_gain_code_entry_struct;

VOID impeghe_drc_generate_delta_time_code_table(
    const WORD32 num_gain_values_max,
    ia_drc_delta_time_code_table_entry_struct *delta_time_code_table_item);

VOID impeghe_drc_get_delta_gain_code_table(
    const WORD32 gain_coding_profile,
    ia_drc_delta_gain_code_entry_struct const **pstr_delta_gain_code_table, WORD32 *num_entries);

const ia_drc_slope_code_table_entry_struct *impeghe_drc_get_slope_code_table_by_value(VOID);

FLOAT32 impeghe_drc_decode_slope_idx_value(const WORD32 slope_code_index);

FLOAT32 impeghe_drc_decode_slope_idx_magnitude(const WORD32 slope_code_index);

#endif /*IMPEGHE_DRC_TABLES_H*/
