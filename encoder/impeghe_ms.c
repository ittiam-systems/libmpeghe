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

#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"

/**
 *  impeghe_ms_apply
 *
 *  \brief Mid/Side stereo processing
 *
 *  \param [in] pstr_psy_data		Pointer to psychoacoustic data structure data structure
 *  \param [in/out] ptr_spec_left	Pointer to left MDCT spectrum
 *  \param [in/out] ptr_spec_right	Pointer to right MDCT spectrum
 *  \param [out] ms_select		    MS mask value
 *  \param [in] sfb_count			Number of scalefactor bands
 *  \param [in] sfb_per_group		Number of scalefactor bands per group
 *  \param [in] max_sfb_per_grp		Max scalefactor bands per group
 *  \param [in] ptr_sfb_offsets		Offset table for scalefactor band
 *  \param [in] chn					Channel offset
 *  \param [in] ptr_ms_spec			Pointer to mid/side MDCT spectrum
 *
 *  \return VOID
 */
VOID impeghe_ms_apply(ia_psy_mod_data_struct *pstr_psy_data, FLOAT64 *ptr_spec_left,
                      FLOAT64 *ptr_spec_right, WORD32 *ms_select,
                      WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG], const WORD32 sfb_count,
                      const WORD32 sfb_per_group, const WORD32 max_sfb_per_grp,
                      const WORD32 *ptr_sfb_offsets, WORD32 chn, FLOAT64 *ptr_ms_spec)
{
  FLOAT32 *ptr_sfb_enegry_left = pstr_psy_data[chn].ptr_sfb_energy_long;
  FLOAT32 *ptr_sfb_energy_right = pstr_psy_data[chn + 1].ptr_sfb_energy_long;
  const FLOAT32 *ptr_sfb_energy_mid = pstr_psy_data[chn].ptr_sfb_energy_long_ms;
  const FLOAT32 *ptr_sfb_energy_side = pstr_psy_data[chn + 1].ptr_sfb_energy_long_ms;
  FLOAT32 *ptr_sfb_thr_left = pstr_psy_data[chn].ptr_sfb_thr_long;
  FLOAT32 *ptr_sfb_thr_right = pstr_psy_data[chn + 1].ptr_sfb_thr_long;
  FLOAT32 *ptr_sfb_spread_energy_left = pstr_psy_data[chn].ptr_sfb_spreaded_energy_long;
  FLOAT32 *ptr_sfb_spread_energy_right = pstr_psy_data[chn + 1].ptr_sfb_spreaded_energy_long;
  WORD32 sfb, sfb_offsets, j;
  WORD32 grp = 0;
  WORD32 ms_counter = 0;
  WORD32 lr_counter = 0;

  *ms_select = 0;

  for (sfb = 0; sfb < sfb_count; sfb += sfb_per_group, grp++)
  {
    for (sfb_offsets = 0; sfb_offsets < max_sfb_per_grp; sfb_offsets++)
    {
      FLOAT32 left_right, mid_side, min_thr;
      WORD32 use_ms;
      ms_used[grp][sfb_offsets] = 0;

      min_thr = min(ptr_sfb_thr_left[sfb + sfb_offsets], ptr_sfb_thr_right[sfb + sfb_offsets]);

      left_right =
          (ptr_sfb_thr_left[sfb + sfb_offsets] /
           max(ptr_sfb_enegry_left[sfb + sfb_offsets], ptr_sfb_thr_left[sfb + sfb_offsets])) *
          (ptr_sfb_thr_right[sfb + sfb_offsets] /
           max(ptr_sfb_energy_right[sfb + sfb_offsets], ptr_sfb_thr_right[sfb + sfb_offsets]));

      mid_side = (min_thr / max(ptr_sfb_energy_mid[sfb + sfb_offsets], min_thr)) *
                 (min_thr / max(ptr_sfb_energy_side[sfb + sfb_offsets], min_thr));

      use_ms = (mid_side >= left_right);

      if (use_ms)
      {

        ms_used[grp][sfb_offsets] = 1;

        for (j = ptr_sfb_offsets[sfb + sfb_offsets]; j < ptr_sfb_offsets[sfb + sfb_offsets + 1];
             j++)
        {
          if (ptr_ms_spec != NULL)
          {
            ptr_spec_left[j] = ptr_ms_spec[j];
            ptr_spec_right[j] = ptr_ms_spec[FRAME_LEN_LONG + j];
          }
          else
          {
            FLOAT64 tmp = ptr_spec_left[j];

            ptr_spec_left[j] = 0.5f * (ptr_spec_left[j] + ptr_spec_right[j]);

            ptr_spec_right[j] = 0.5f * (tmp - ptr_spec_right[j]);
          }
        }

        ptr_sfb_thr_left[sfb + sfb_offsets] = ptr_sfb_thr_right[sfb + sfb_offsets] = min_thr;

        ptr_sfb_enegry_left[sfb + sfb_offsets] = ptr_sfb_energy_mid[sfb + sfb_offsets];
        ptr_sfb_energy_right[sfb + sfb_offsets] = ptr_sfb_energy_side[sfb + sfb_offsets];

        ptr_sfb_spread_energy_left[sfb + sfb_offsets] =
            ptr_sfb_spread_energy_right[sfb + sfb_offsets] =
                min(ptr_sfb_spread_energy_left[sfb + sfb_offsets],
                    ptr_sfb_spread_energy_right[sfb + sfb_offsets]) *
                0.5f;

        ms_counter++;
      }
      else
      {

        ms_used[grp][sfb_offsets] = 0;
        lr_counter++;
      }
    }
  }

  if (ms_counter == 0)
  {
    *ms_select = 0;
  }
  else
  {
    if (lr_counter != 0)
    {
      *ms_select = 1;
    }
    else
    {
      *ms_select = 2;
    }
  }
  return;
}

/**
 *  impeghe_calc_ms_band_energy
 *
 *  \brief Calculates mid and side energy per scalefactor band
 *
 *  \param [in] ptr_spec_left			Pointer to mid MDCT spectrum
 *  \param [in] ptr_spec_right			Pointer to side MDCT spectrum
 *  \param [in] ptr_band_offset			Pointer to scalefactor band offsets
 *  \param [in] num_bands				Number of scalefactor bands
 *  \param [out] ptr_band_energy_mid	Pointer to bandwise mid energies
 *  \param [out] ptr_band_energy_side	Pointer to bandwise side energies
 *
 *  \return VOID
 */
VOID impeghe_calc_ms_band_energy(const FLOAT64 *ptr_spec_left, const FLOAT64 *ptr_spec_right,
                                 const WORD32 *ptr_band_offset, const WORD32 num_bands,
                                 FLOAT32 *ptr_band_energy_mid, FLOAT32 *ptr_band_energy_side)
{

  WORD32 i, j;

  j = 0;
  for (i = 0; i < num_bands; i++)
  {
    ptr_band_energy_mid[i] = 0.0f;
    ptr_band_energy_side[i] = 0.0f;

    while (j < ptr_band_offset[i + 1])
    {
      FLOAT32 specm, specs;

      specm = (FLOAT32)(0.5f * (ptr_spec_left[j] + ptr_spec_right[j]));
      specs = (FLOAT32)(0.5f * (ptr_spec_left[j] - ptr_spec_right[j]));

      ptr_band_energy_mid[i] += specm * specm;
      ptr_band_energy_side[i] += specs * specs;

      j++;
    }
  }

  return;
}
