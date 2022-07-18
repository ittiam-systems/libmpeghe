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

#ifndef IMPEGHE_MS_H
#define IMPEGHE_MS_H

#ifndef NULL
#define NULL 0
#endif
typedef struct
{
  WORD32 ms_mask;
  WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
} ia_ms_info_struct;

VOID impeghe_ms_apply(ia_psy_mod_data_struct *pstr_psy_data, FLOAT64 *ptr_spec_left,
                      FLOAT64 *ptr_spec_right, WORD32 *ms_select,
                      WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG], const WORD32 sfb_count,
                      const WORD32 sfb_per_group, const WORD32 max_sfb_per_grp,
                      const WORD32 *ptr_sfb_offsets, WORD32 chn, FLOAT64 *ptr_ms_spec);

VOID impeghe_calc_ms_band_energy(const FLOAT64 *ptr_spec_left, const FLOAT64 *ptr_spec_right,
                                 const WORD32 *ptr_band_offset, const WORD32 num_bands,
                                 FLOAT32 *ptr_band_energy_mid, FLOAT32 *ptr_band_energy_side);
#endif /* IMPEGHE_MS_H */
