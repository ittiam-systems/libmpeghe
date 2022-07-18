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

#ifndef IMPEGHE_PSY_UTILS_H
#define IMPEGHE_PSY_UTILS_H

typedef struct
{
  WORD32 sample_rate;
  WORD32 num_sfb_long;
  WORD32 num_sfb_short;
  const WORD16 *cb_offset_long;
  const WORD16 *cb_offset_short;
  WORD32 sfb_width_long[MAX_NUM_SFB_LONG];
  WORD32 sfb_width_short[MAX_NUM_SFB_SHORT];

} ia_sfb_info_struct;

VOID impeghe_psy_long_config_init(WORD32 bit_rate, WORD32 sample_rate, WORD32 band_width,
                                  ia_psy_mod_long_config_struct *pstr_psy_config);
VOID impeghe_psy_short_config_init(WORD32 bit_rate, WORD32 sample_rate, WORD32 band_width,
                                   ia_psy_mod_short_config_struct *pstr_psy_config);
VOID impeghe_calc_band_energy(const FLOAT64 *ptr_spec_coeffs, const WORD32 *band_offset,
                              const WORD32 num_bands, FLOAT32 *ptr_band_energy, WORD32 sfb_count);
VOID impeghe_find_max_spreading(const WORD32 sfb_count, const FLOAT32 *ptr_mask_low_fac,
                                const FLOAT32 *ptr_mask_high_fac, FLOAT32 *ptr_spreaded_enegry);
VOID impeghe_pre_echo_control(FLOAT32 *ptr_thr_nm1, WORD32 sfb_count, FLOAT32 max_allowed_inc_fac,
                              FLOAT32 min_remaining_thr_fac, FLOAT32 *ptr_threshold);

VOID impeghe_sfb_params_init(WORD32 sample_rate, WORD32 *ptr_sfb_width, WORD32 *num_sfb,
                             WORD32 win_seq);
#endif /* IMPEGHE_PSY_UTILS_H */
