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

#include <string.h>
#include "math.h"
#include "impeghe_cnst.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_cnst.h"
#include "impeghe_fd_quant.h"

#include "impeghe_nf.h"

/**
 *  impeghe_noise_filling_limiter
 *
 *  \brief Remove highest tone components
 *
 *  \param [in,out] energy			Spectrum energy
 *  \param [in] ptr_spec		Pointer to spectrum
 *  \param [in] ptr_quant_spec	Pointer to quantized spectrum
 *  \param [in] n0_by_4			N0/4
 *  \param [in] sfb_offset		offset table for scalefactor band
 *  \param [in] sb				Band index
 *  \param [in] cntr			Index counter
 *  \param [in] highest_tone	Highest tones
 *
 *  \return VOID
 */
static VOID impeghe_noise_filling_limiter(FLOAT64 *energy, FLOAT64 *ptr_spec,
                                          WORD32 *ptr_quant_spec, WORD32 n0_by_4,
                                          WORD32 *ptr_sfb_offset, WORD32 sb, WORD32 cntr,
                                          FLOAT64 *ptr_highest_tone)
{
  WORD32 n, i;
  FLOAT64 tone_energy;

  if (!n0_by_4)
    return;
  if (cntr <= n0_by_4)
    return;

  memset(ptr_highest_tone, 0, n0_by_4 * sizeof(*ptr_highest_tone));

  /* finds the n0_by_4 strongest bins */
  for (i = ptr_sfb_offset[sb]; i < ptr_sfb_offset[sb + 1]; i++)
  {
    if (!ptr_quant_spec[i])
    {
      tone_energy = ptr_spec[i] * ptr_spec[i];

      for (n = 0; n < n0_by_4; n++)
      {
        if (tone_energy > ptr_highest_tone[n])
        {
          memmove(ptr_highest_tone + 1 + n, ptr_highest_tone + n,
                  (n0_by_4 - n - 1) * sizeof(*ptr_highest_tone));
          ptr_highest_tone[n] = tone_energy;
          break;
        }
      }
    }
  }
  /* remove the contribution of the highest_tone components */
  for (n = 0; n < n0_by_4; n++)
    *energy -= ptr_highest_tone[n];

  /* add the average component energy */
  *energy += n0_by_4 * (*energy) / (cntr - n0_by_4);
  return;
}

/**
 *  impeghe_noise_filling
 *
 *  \brief Noise filling process
 *
 *  \param [out] noise_level					Noise level
 *  \param [out] noise_offset					Noise offset
 *  \param [in] ptr_quant_spec					Pointer to quantized spectrum
 *  \param [in] pstr_quant_info					Pointer to quant structure
 *  \param [in] ptr_sfb_offset						offset table for
 * scalefactor
 * band
 *  \param [in] max_sfb							Max sfb value
 *  \param [in] window_size_samples				block length
 *  \param [in] num_window_groups				Number of window groups
 *  \param [in] ptr_window_group_length				Window group length
 *  \param [in] noise_filling_start_offset		start offset of noise filling
 *  \param [in] ptr_scratch_buf					Pointer to scratch
 *
 *  \return VOID
 */
VOID impeghe_noise_filling(WORD32 *noise_level, WORD32 *noise_offset, FLOAT64 *ptr_quant_spec,
                           ia_usac_quant_info_struct *pstr_quant_info, WORD32 *ptr_sfb_offset,
                           WORD32 max_sfb, WORD32 window_size_samples, WORD32 num_window_groups,
                           const WORD32 *ptr_window_group_length,
                           WORD32 noise_filling_start_offset, FLOAT64 *ptr_scratch_buf)
{
  FLOAT64 energy;
  FLOAT64 noise_level_temp;
  FLOAT64 noise_offset_temp;

  FLOAT64 sum_sfb_on, sum_sfb_off;
  FLOAT64 e_sfb_on, e_sfb_off;

  WORD32 n0;
  WORD32 start_sfb, sfb, i;
  WORD32 band_quantized_to_zero;

  FLOAT64 alpha = 0.15; /* prudence factor */
  WORD32 grp = 0;

  e_sfb_on = 1e-6;
  e_sfb_off = 1e-6;

  sum_sfb_on = 1e-6;
  sum_sfb_off = 1e-6;

  *noise_offset = 0;
  *noise_level = 0;

  for (sfb = 0; sfb < max_sfb; sfb++)
  {
    if (ptr_sfb_offset[sfb + 1] > noise_filling_start_offset)
      break;
  }
  start_sfb = sfb;
  for (grp = 0; grp < num_window_groups; grp++)
  {
    WORD32 grp_win = 0;
    for (sfb = start_sfb; sfb < max_sfb; sfb++)
    {
      band_quantized_to_zero = 1;
      for (grp_win = 0; grp_win < ptr_window_group_length[grp]; grp_win++)
      {
        WORD32 offset = grp_win * window_size_samples;
        energy = 0;
        n0 = 0;
        for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 1]; i++)
        {
          /* calculate energy if the quantized value is non zero */
          if (!pstr_quant_info->quant_degroup[offset + i])
          {
            energy += ptr_quant_spec[offset + i] * ptr_quant_spec[offset + i];
            n0++;
          }
          else
          {
            /* All quantized values are not zero */
            band_quantized_to_zero = 0;
          }
        }

        /* Remove highest (tonal) contributions */
        impeghe_noise_filling_limiter(&energy, &ptr_quant_spec[offset],
                                      pstr_quant_info->quant_degroup, n0 / 4, ptr_sfb_offset, sfb,
                                      n0, ptr_scratch_buf);

        if (band_quantized_to_zero == 0)
        {
          e_sfb_on += energy;
          sum_sfb_on += pow(2., 0.5 * pstr_quant_info->scale_factor[sfb] - 50) * n0;
        }
        else
        /* subband is completely zeroed  */
        {
          e_sfb_off += energy;
          sum_sfb_off += pow(2., 0.5 * pstr_quant_info->scale_factor[sfb] - 58) *
                         (ptr_sfb_offset[sfb + 1] - ptr_sfb_offset[sfb]);
        }
      }
    }
  }

  if (num_window_groups > 1)
    alpha = alpha * 0.15;

  if (sum_sfb_on)
  {
    noise_level_temp = 1.5 * (log(alpha * e_sfb_on) - log(sum_sfb_on)) / log(2.0) + 14.0;

    /* quantize to nearest integer */
    *noise_level = (WORD32)(noise_level_temp + 0.5);

    /* noise level limited to quantization range [0,7] */
    *noise_level = max(*noise_level, 0);
    *noise_level = min(*noise_level, 7);

    if (*noise_level != 0)
    {
      noise_offset_temp =
          2. * log(alpha * e_sfb_off * sum_sfb_on / sum_sfb_off / e_sfb_on) / log(2.);

      /* quantize to nearest integer */
      *noise_offset = (WORD32)(noise_offset_temp + 0.5);

      /* noise offset limited to quantization range [0,31] */
      *noise_level = *noise_offset <= 0 ? 0 : *noise_level;
      *noise_offset = min(*noise_offset, 31);
      *noise_offset = max(*noise_offset, 0);
    }
  }
  return;
}
