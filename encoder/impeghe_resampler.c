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
#include <stdio.h>
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_resampler.h"
#include "impeghe_resampler_rom.h"

/**
 * @defgroup ResampProc Resampler Processing
 * @ingroup  ResampProc
 * @brief Resampler Processing
 *
 * @{
 */

/**
 *  impeghe_resample
 *
 *  \brief According to Sec 4.8.2.2 of ISO_IEC_23008-3(2019 version)
 *         The LPD mode shall only be employed at 3D audio core coder
 *         sampling rates ≤ 32000 Hz. If ptr_inp_l signal is > 32000 Hz the
 *         encoder should resample ptr_inp_l signal to ≤ 32000 Hz
 *
 *  \param [in ] pstr_resampler Pointer to structure holding delayed ptr_inp_l signal
 *  \param [in ] ptr_input      Input PCM samples
 *  \param [out] ptr_output     Output PCM samples
 *  \param [in ] ptr_scratch    Pointer to scratch buffer
 *
 *  \return
 *
 */

VOID impeghe_resample(ia_resampler_struct *pstr_resampler, FLOAT32 *ptr_input,
                      FLOAT32 *ptr_output, FLOAT32 *ptr_scratch)
{
  WORD32 i = 0;
  FLOAT32 *ptr_resamp_mem = &pstr_resampler->delay_samples[0];
  WORD32 fac_down = pstr_resampler->fac_down;
  WORD32 fac_up = pstr_resampler->fac_up;
  const FLOAT32 *ptr_resamp_filt_tbl = NULL;
  FLOAT32 *ptr_inp_l = ptr_input, *ptr_out_l = ptr_output;
  if (fac_down == 2)
  {
    ptr_resamp_filt_tbl = &impeghe_resampler_filter[0];
  }
  else // fac_down == 3
  {
    ptr_resamp_filt_tbl = &impeghe_resampler_filter_3[0];
  }
  if (fac_up != 1)
  {
    ptr_out_l = ptr_scratch;
    ptr_resamp_mem = &pstr_resampler->delay_samples_upsamp[0];
    for (i = 0; i < (pstr_resampler->input_length * fac_up); i++)
    {
      FLOAT32 mac = 0;
      FLOAT32 *ptr_temp = NULL;
      WORD32 j = 0;
      const FLOAT32 *ptr_filt_coef = impeghe_resampler_filter;
      FLOAT32 *ptr_delay = &pstr_resampler->delay_samples_upsamp[0];
      ptr_delay += ((i + 1) / fac_up);

      if (((i % fac_up) == 0) && (i < RESAMPLE_DELAY))
      {
        mac += (*ptr_delay) * (*ptr_filt_coef);
        ptr_delay++;
        ptr_filt_coef++;
        j = 1;
      }
      for (; (i + j) < RESAMPLE_DELAY; j += fac_up)
      {
        mac += (*ptr_delay) * (*ptr_filt_coef);
        ptr_filt_coef++;
        mac += (*ptr_delay) * (*ptr_filt_coef);
        ptr_delay++;
        ptr_filt_coef++;
      }

      if (i < (RESAMPLE_FILTER_LEN + 1))
      {
        ptr_temp = ptr_inp_l;
      }
      else
      {
        ptr_temp = ptr_inp_l + ((i + 1) / fac_up - 30);
      }
      if ((i > ((RESAMPLE_UPSAMPLE_DELAY - 1) * fac_up)) && ((i % fac_up) == 0) &&
          j < RESAMPLE_FILTER_LEN)
      {
        mac += (*ptr_temp) * (*ptr_filt_coef);
        ptr_filt_coef++;
        ptr_temp++;
        j++;
      }

      for (; j < RESAMPLE_FILTER_LEN; j += fac_up)
      {
        mac += (*ptr_temp) * (*ptr_filt_coef);
        ptr_filt_coef++;
        if (j + 1 < RESAMPLE_FILTER_LEN)
        {
          mac += (*ptr_temp) * (*ptr_filt_coef);
          ptr_filt_coef++;
        }
        ptr_temp++;
      }
      if ((i % fac_up) == 0 && (j < RESAMPLE_FILTER_LEN))
      {
        mac += (*ptr_temp) * (*ptr_filt_coef);
        ptr_temp++;
        ptr_filt_coef++;
        j++;
      }
      *ptr_out_l = mac;
      ptr_out_l++;
    }

    memcpy(&pstr_resampler->delay_samples_upsamp[0],
           ptr_input + (pstr_resampler->input_length) - RESAMPLE_UPSAMPLE_DELAY,
           RESAMPLE_UPSAMPLE_DELAY * sizeof(FLOAT32));
    ptr_inp_l = ptr_scratch;
    ptr_out_l = ptr_output;
    ptr_resamp_mem = &pstr_resampler->delay_samples[0];
  }

  for (i = 0; i < (RESAMPLER_OUT_FRAME_LEN * fac_down); i += fac_down)
  {
    FLOAT32 mac = 0;
    FLOAT32 *ptr_temp;
    WORD32 j = 0;
    const FLOAT32 *ptr_filt_coef = ptr_resamp_filt_tbl;
    FLOAT32 *ptr_delay = &ptr_resamp_mem[i];

    for (j = 0; i + j < RESAMPLE_DELAY; j++)
    {
      mac += (*ptr_delay) * (*ptr_filt_coef);
      ptr_delay++;
      ptr_filt_coef++;
    }

    ptr_temp = ptr_inp_l + i + j - RESAMPLE_DELAY;
    for (; j < RESAMPLE_FILTER_LEN; j++)
    {
      mac += (*ptr_temp) * (*ptr_filt_coef);
      ptr_temp++;
      ptr_filt_coef++;
    }
    *ptr_out_l = mac;
    ptr_out_l++;
  }

  /* Store ptr_delay samples for next frame */
  memcpy(&ptr_resamp_mem[0], ptr_inp_l + (RESAMPLER_OUT_FRAME_LEN * fac_down) - RESAMPLE_DELAY,
         RESAMPLE_DELAY * sizeof(FLOAT32));
}
/** @} */ /* End of ResampProc */
