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
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"

#include "impeghe_fd_quant.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_fft.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"

/**
 *  impeghe_calc_pds
 *
 *  \brief Normalize input
 *
 *  \param [in,out] ptr_input Input data
 *
 *  \return VOID
 */
static VOID impeghe_calc_pds(FLOAT32 *ptr_input)
{
  WORD32 i;
  FLOAT64 max_pow, delta;

  max_pow = max(
      10 * (log10(ptr_input[0] * ptr_input[0] + ptr_input[1] * ptr_input[1]) - LOG_1024_BASE_10) +
          10e-15,
      MIN_POW);

  for (i = 1; i < FRAME_LEN_LONG / 2; i++)
  {
    /* removed the sqrt along with clubbing the for loops */
    ptr_input[2 * i] = (FLOAT32)max(10 * (log10(ptr_input[2 * i] * ptr_input[2 * i] +
                                                ptr_input[2 * i + 1] * ptr_input[2 * i + 1]) -
                                          LOG_1024_BASE_10) +
                                        10e-15,
                                    MIN_POW);
    max_pow = max(max_pow, ptr_input[2 * i]);
  }

  /* Normalized to reference sound pressure level 96 dB */
  delta = 96 - max_pow;

  for (i = 0; i < FRAME_LEN_LONG / 2; i++)
  {
    ptr_input[2 * i] = ptr_input[2 * i] + (FLOAT32)delta;
  }
  return;
}

/**
 *  impeghe_find_tonal
 *
 *  \brief Find tonal components
 *
 *  \param [in] ptr_input			Pointer to input
 *  \param [in] ptr_tonal_flag			Tonal flag
 *  \param [in] ptr_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
static VOID impeghe_find_tonal(FLOAT32 *ptr_input, WORD32 *ptr_tonal_flag, FLOAT32 *ptr_scratch)
{
  WORD32 i, j;
  WORD32 is_tonal;
  FLOAT64 tonal_spl;
  FLOAT64 absolute_threshold_xm;

  for (i = 0; i < FRAME_LEN_LONG / 2; i++)
  {
    ptr_scratch[i] = ptr_input[2 * i];
  }

  /* initialize ptr_tonal_flag[] */
  for (i = 0; i <= 511; i++)
  {
    ptr_tonal_flag[i] = 0;
  }

  for (i = 2; i < 500; i++)
  {
    if (ptr_scratch[i] > ptr_scratch[i - 1] && ptr_scratch[i] >= ptr_scratch[i + 1])
    {
      is_tonal = 1;

      /* Verify it meets the condition: ptr_scratch[i]-ptr_scratch[i+j]>=7 */

      if (1 < i && i < 62)
      {
        for (j = -2; j <= -2; j++)
        {
          is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
          if (is_tonal == 0)
            break;
        }
        if (is_tonal == 1)
        {
          for (j = 2; j <= 2; j++)
          {
            is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
            if (is_tonal == 0)
              break;
          }
        }

        if (is_tonal == 1)
        {
          ptr_tonal_flag[i] = 1;
        }
      }

      else if (62 <= i && i < 126)
      {
        for (j = -3; j <= -2; j++)
        {
          is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
          if (is_tonal == 0)
            break;
        }
        if (is_tonal == 1)
        {
          for (j = 2; j <= 3; j++)
          {
            is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
            if (is_tonal == 0)
              break;
          }
        }

        if (is_tonal == 1)
        {
          ptr_tonal_flag[i] = 1;
        }
      }

      else if (126 <= i && i < 254)
      {
        for (j = -6; j <= -2; j++)
        {
          is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
          if (is_tonal == 0)
            break;
        }
        if (is_tonal == 1)
        {
          for (j = 2; j <= 6; j++)
          {
            is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
            if (is_tonal == 0)
              break;
          }
        }

        if (is_tonal == 1)
        {
          ptr_tonal_flag[i] = 1;
        }
      }

      else if (254 <= i && i < 500)
      {
        for (j = -12; j <= -2; j++)
        {
          is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
          if (is_tonal == 0)
            break;
        }
        if (is_tonal == 1)
        {
          for (j = 2; j <= 12; j++)
          {
            is_tonal = is_tonal && ptr_scratch[i] - ptr_scratch[i + j] >= 7;
            if (is_tonal == 0)
              break;
          }
        }

        if (is_tonal == 1)
        {
          ptr_tonal_flag[i] = 1;
        }
      }
      else
      {
        is_tonal = 0;
      }
    }
  }

  for (i = 0; i <= 511; i++)
  {
    if (ptr_tonal_flag[i] == 1)
    {
      /* compute the SPL of tonal */
      tonal_spl = 10 * log10(pow(10, (ptr_scratch[i - 1] / 10)) + pow(10, (ptr_scratch[i] / 10)) +
                             pow(10, (ptr_scratch[i + 1] / 10)));

      if (i >= 324)
      {
        absolute_threshold_xm = impeghe_classify_arrays.absolute_threshold[i] + 20;
      }
      else
      {
        absolute_threshold_xm = impeghe_classify_arrays.absolute_threshold[i];
      }

      if (tonal_spl < absolute_threshold_xm)
      {
        ptr_tonal_flag[i] = 0;
      }
    }
  }
  return;
}

/**
 *  impeghe_tonal_analysis
 *
 *  \brief Tonal components analysis
 *
 *  \param [in,out] pstr_ton_params	Pointer to tonal params structure
 *  \param [in] pstr_scratch		Pointer to scratch
 *
 *  \return VOID
 */
static VOID impeghe_tonal_analysis(ia_tonal_params_struct *pstr_ton_params,
                                   impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *ptr_complex_fft = pstr_scratch->p_complex_fft;
  WORD32 *ptr_tonal_flag = pstr_scratch->p_tonal_flag;
  FLOAT32 *ptr_time_sig = pstr_ton_params->time_signal;
  WORD32 framecnt_xm = pstr_ton_params->framecnt_xm;
  WORD32 *ptr_n_tonal = pstr_ton_params->n_tonal;
  WORD32 *ptr_n_tonal_low_frequency = pstr_ton_params->n_tonal_low_frequency;
  FLOAT32 *ptr_n_tonal_low_frequency_ratio = pstr_ton_params->n_tonal_low_frequency_ratio;
  FLOAT32 *ave_n_tonal = pstr_ton_params->ave_n_tonal;
  FLOAT32 *ave_n_tonal_short = pstr_ton_params->ave_n_tonal_short;
  WORD32 i;
  WORD32 fft_size = 1024;

  WORD32 frame_length;
  WORD32 n_tonal_total, n_tonal_low_frequency_total;

  for (i = 0; i < FRAME_LEN_LONG; i++)
  {

    ptr_complex_fft[2 * i] =
        (FLOAT32)(ptr_time_sig[i] * impeghe_classify_arrays.hanning_window[i]);
    ptr_complex_fft[2 * i + 1] = 0;
  }

  impeghe_complex_fft(ptr_complex_fft, fft_size, pstr_scratch);

  /* compute power density spectrum */
  /* re_fft contains the resulting pds */
  impeghe_calc_pds(ptr_complex_fft);

  /* detect tonal */
  impeghe_find_tonal(ptr_complex_fft, ptr_tonal_flag, pstr_scratch->p_pow_spec);

  /* update n_tonal, n_tonal_low_frequency */
  for (i = 0; i < 99; i++)
  {
    ptr_n_tonal[i] = ptr_n_tonal[i + 1];
    ptr_n_tonal_low_frequency[i] = ptr_n_tonal_low_frequency[i + 1];
  }
  ptr_n_tonal[99] = 0;
  for (i = 0; i < FRAME_LEN_LONG / 2; i++)
  {
    ptr_n_tonal[99] += ptr_tonal_flag[i];
  }
  ptr_n_tonal_low_frequency[99] = 0;
  for (i = 0; i < INDEXOFLOWFREQUENCY; i++)
  {
    ptr_n_tonal_low_frequency[99] += ptr_tonal_flag[i];
  }

  /* compute long-term AVE and the ratio of distribution in low-frequency domain */
  if (framecnt_xm < AVE_TONAL_LENGTH)
  {
    frame_length = framecnt_xm;
  }
  else
  {
    frame_length = AVE_TONAL_LENGTH;
  }

  n_tonal_total = 0;
  n_tonal_low_frequency_total = 0;
  for (i = 0; i < frame_length; i++)
  {
    n_tonal_total += ptr_n_tonal[99 - i];
    n_tonal_low_frequency_total += ptr_n_tonal_low_frequency[99 - i];
  }

  *ave_n_tonal = (FLOAT32)n_tonal_total / frame_length;

  if (n_tonal_total == 0)
  {
    *ptr_n_tonal_low_frequency_ratio = 1;
  }
  else
  {
    *ptr_n_tonal_low_frequency_ratio = (FLOAT32)n_tonal_low_frequency_total / n_tonal_total;
  }

  /* compute the short-term AVE */
  if (framecnt_xm < AVE_TONAL_LENGTH_SHORT)
  {
    frame_length = framecnt_xm;
  }
  else
  {
    frame_length = AVE_TONAL_LENGTH_SHORT;
  }

  n_tonal_total = 0;
  for (i = 0; i < frame_length; i++)
  {
    n_tonal_total += ptr_n_tonal[99 - i];
  }

  *ave_n_tonal_short = (FLOAT32)n_tonal_total / frame_length;
  return;
}

/**
 *  impeghe_spectral_tilt_analysis
 *
 *  \brief Spectral tilt analysis
 *
 *  \param [in,out] ptr_spec_params	Pointer to spec tilt params structure
 *
 *  \return VOID
 */
static VOID impeghe_spectral_tilt_analysis(ia_spec_tilt_params_struct *ptr_spec_params)
{
  FLOAT32 *ptr_time_signal = ptr_spec_params->time_signal;
  WORD32 framecnt_xm = ptr_spec_params->framecnt_xm;
  FLOAT32 *ptr_spec_tilt_buf = ptr_spec_params->spec_tilt_buf;
  FLOAT32 *ptr_msd_spec_tilt = ptr_spec_params->msd_spec_tilt;
  FLOAT32 *ptr_msd_spec_tilt_short = ptr_spec_params->msd_spec_tilt_short;
  WORD32 i;
  WORD32 frame_length;

  FLOAT32 r0, r1;
  FLOAT32 spec_tilt;
  FLOAT32 ave_spec_tilt;

  /* compute spectral tilt */
  r0 = 0;
  r1 = 0;
  for (i = 0; i < FRAME_LEN_LONG - 1; i++)
  {
    r0 += ptr_time_signal[i] * ptr_time_signal[i];
    r1 += ptr_time_signal[i] * ptr_time_signal[i + 1];
  }
  r0 += ptr_time_signal[i] * ptr_time_signal[i];

  if (r0 == 0)
  {
    spec_tilt = 1.0f;
  }
  else
  {
    spec_tilt = r1 / r0;
  }

  /* update spec_tilt_buf */
  for (i = 0; i < 100 - 1; i++)
  {
    ptr_spec_tilt_buf[i] = ptr_spec_tilt_buf[i + 1];
  }
  ptr_spec_tilt_buf[99] = spec_tilt;

  /* compute the long-term mean square deviation of the spectral tilt */
  if (framecnt_xm < SPECTRAL_TILT_LENGTH)
  {
    frame_length = framecnt_xm;
  }
  else
  {
    frame_length = SPECTRAL_TILT_LENGTH;
  }

  ave_spec_tilt = 0;
  for (i = 0; i < frame_length; i++)
  {
    ave_spec_tilt += ptr_spec_tilt_buf[99 - i];
  }
  ave_spec_tilt /= frame_length;

  *ptr_msd_spec_tilt = 0;
  for (i = 0; i < frame_length; i++)
  {
    *ptr_msd_spec_tilt +=
        (ptr_spec_tilt_buf[99 - i] - ave_spec_tilt) * (ptr_spec_tilt_buf[99 - i] - ave_spec_tilt);
  }
  *ptr_msd_spec_tilt /= frame_length;

  /* compute the short-term mean square deviation of the spectral tilt */
  if (framecnt_xm < SPECTRAL_TILT_LENGTH_SHORT)
  {
    frame_length = framecnt_xm;
  }
  else
  {
    frame_length = SPECTRAL_TILT_LENGTH_SHORT;
  }

  ave_spec_tilt = 0;
  for (i = 0; i < frame_length; i++)
  {
    ave_spec_tilt += ptr_spec_tilt_buf[99 - i];
  }
  ave_spec_tilt /= frame_length;

  *ptr_msd_spec_tilt_short = 0;
  for (i = 0; i < frame_length; i++)
  {
    *ptr_msd_spec_tilt_short +=
        (ptr_spec_tilt_buf[99 - i] - ave_spec_tilt) * (ptr_spec_tilt_buf[99 - i] - ave_spec_tilt);
  }
  *ptr_msd_spec_tilt_short /= frame_length;

  /* compute the energy of current frame */
  if (r0 <= 1)
  {
    ptr_spec_params->frame_energy = 0;
  }
  else
  {
    ptr_spec_params->frame_energy = (FLOAT32)(10 * log(r0) / log(10));
  }
  return;
}

/**
 *  impeghe_init_mode_decision
 *
 *  \brief Initialize mode decision params
 *
 *  \param [in,out] pstr_mode_params Pointer to mode decision params
 *
 *  \return WORD32		Mode decision result
 */
static WORD32 impeghe_init_mode_decision(ia_mode_params_struct *pstr_mode_params)
{
  WORD32 i;
  WORD32 framecnt = pstr_mode_params->framecnt;
  WORD32 *framecnt_xm = pstr_mode_params->framecnt_xm;
  WORD32 *flag_border = pstr_mode_params->flag_border;
  FLOAT32 ave_n_tonal_short = pstr_mode_params->ave_n_tonal_short;
  FLOAT32 ave_n_tonal = pstr_mode_params->ave_n_tonal;
  FLOAT32 *ave_n_tonal_short_buf = pstr_mode_params->ave_n_tonal_short_buf;
  FLOAT32 *ave_n_tonal_buf = pstr_mode_params->ave_n_tonal_buf;
  FLOAT32 msd_spec_tilt = pstr_mode_params->msd_spec_tilt;
  FLOAT32 msd_spec_tilt_short = pstr_mode_params->msd_spec_tilt_short;
  FLOAT32 *msd_spec_tilt_buf = pstr_mode_params->msd_spec_tilt_buf;
  FLOAT32 *msd_spec_tilt_short_buf = pstr_mode_params->msd_spec_tilt_short_buf;
  FLOAT32 n_tonal_low_frequency_ratio = pstr_mode_params->n_tonal_low_frequency_ratio;
  FLOAT32 frame_energy = pstr_mode_params->frame_energy;
  WORD32 init_mode_decision_result = TBD;
  WORD32 count_msd_st_monchhichi = 0;
  WORD32 count_msd_st_speech_music = 0, count_msd_st_music_speech = 0;
  WORD32 flag_ave_music_speech = 0;
  WORD32 count_msd_st_music = 0;
  WORD32 border_state = 0;
  WORD32 count_quiet_mode = 0;

  *flag_border = NO_BORDER;

  /* border decision according to spectral tilt */

  /* update msd_spec_tilt_buf, msd_spec_tilt_short_buf */
  for (i = 0; i < 5 - 1; i++)
  {
    msd_spec_tilt_buf[i] = msd_spec_tilt_buf[i + 1];
    msd_spec_tilt_short_buf[i] = msd_spec_tilt_short_buf[i + 1];
  }
  msd_spec_tilt_buf[4] = msd_spec_tilt;
  msd_spec_tilt_short_buf[4] = msd_spec_tilt_short;

  /* speech->music find strict border of speech->music */
  if ((msd_spec_tilt >= 0.014) && (msd_spec_tilt_short <= 0.000005))
  {
    count_msd_st_monchhichi++;
  }
  else
  {
    count_msd_st_monchhichi = 0;
  }
  if (((*flag_border != BORDER_SPEECH_MUSIC_DEFINITE) &&
       (*flag_border != BORDER_MUSIC_SPEECH_DEFINITE)) &&
      (border_state != BORDER_SPEECH_MUSIC_DEFINITE) && (count_msd_st_monchhichi >= 15) &&
      (*framecnt_xm >= 300))
  {
    count_msd_st_monchhichi = 0;
    *framecnt_xm = 10;
    *flag_border = BORDER_SPEECH_MUSIC;
  }

  /* find the relative loose border of speech->music */
  if ((msd_spec_tilt >= 0.0025) && (msd_spec_tilt_short <= 0.000003))
  {
    count_msd_st_speech_music++;
  }
  else
  {
    count_msd_st_speech_music = 0;
  }
  if (((*flag_border != BORDER_SPEECH_MUSIC_DEFINITE) &&
       (*flag_border != BORDER_MUSIC_SPEECH_DEFINITE)) &&
      (border_state != BORDER_SPEECH_MUSIC_DEFINITE) && (count_msd_st_speech_music >= 15) &&
      (*framecnt_xm >= 300))
  {
    count_msd_st_speech_music = 0;
    *framecnt_xm = 10;
    *flag_border = BORDER_SPEECH_MUSIC;
  }

  /* music->speech */
  if ((msd_spec_tilt_buf[0] <= 0.0003) && (msd_spec_tilt_short_buf[0] <= 0.0002))
  {
    count_msd_st_music_speech++;
  }
  if (((*flag_border != BORDER_SPEECH_MUSIC_DEFINITE) &&
       (*flag_border != BORDER_MUSIC_SPEECH_DEFINITE)) &&
      (border_state != BORDER_MUSIC_SPEECH_DEFINITE) && (count_msd_st_music_speech >= 100) &&
      (msd_spec_tilt >= 0.0008) && (msd_spec_tilt_short >= 0.0025) && (*framecnt_xm >= 20))
  {
    count_msd_st_music_speech = 0;
    *framecnt_xm = 10;
    *flag_border = BORDER_MUSIC_SPEECH;
  }

  /* border decision according to tonal
   *  update ave_n_tonal_short_buf, ave_n_tonal_buf */
  for (i = 0; i < 5 - 1; i++)
  {
    ave_n_tonal_short_buf[i] = ave_n_tonal_short_buf[i + 1];
    ave_n_tonal_buf[i] = ave_n_tonal_buf[i + 1];
  }
  ave_n_tonal_short_buf[4] = ave_n_tonal_short;
  ave_n_tonal_buf[4] = ave_n_tonal;

  /* music->speech */
  if ((ave_n_tonal_buf[0] >= 12) && (ave_n_tonal_buf[0] < 15) &&
      (ave_n_tonal_buf[0] - ave_n_tonal_short_buf[0] >= 5) && (*framecnt_xm >= 20) &&
      (ave_n_tonal_short - ave_n_tonal_short_buf[0] < 5))
  {
    *framecnt_xm = 10;
    flag_ave_music_speech = 1;
    *flag_border = BORDER_MUSIC_SPEECH_DEFINITE;
  }

  /* update border decision according to energy */
  if (frame_energy <= 60)
  {
    count_quiet_mode = 0;
  }
  else
  {
    count_quiet_mode++;
  }

  if ((*flag_border == BORDER_MUSIC_SPEECH) && (count_quiet_mode <= 5))
  {
    *flag_border = BORDER_MUSIC_SPEECH_DEFINITE;
    *framecnt_xm = 10;
  }

  /* MUSIC_DEFINITE and SPEECH_DEFINITE mode decision according to short-term characters */

  /* ave_n_tonal_short */
  if ((init_mode_decision_result == TBD) && (ave_n_tonal_short >= 19))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }
  if ((init_mode_decision_result == TBD) && (ave_n_tonal_short <= 1.5))
  {
    init_mode_decision_result = SPEECH_DEFINITE;
  }

  /* msd_spec_tilt_short */
  if (msd_spec_tilt_short >= 0.02)
  {
    init_mode_decision_result = SPEECH_DEFINITE;
  }
  if ((init_mode_decision_result == TBD) && (msd_spec_tilt_short <= 0.00000025) &&
      (framecnt >= 10))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }

  /* SPEECH mode decision */

  /* flag_ave_music_speech??ave_n_tonal_short */
  if ((init_mode_decision_result == TBD) && (flag_ave_music_speech == 1))
  {
    if ((ave_n_tonal_short <= 12) && (*framecnt_xm <= 150))
    {
      init_mode_decision_result = SPEECH;
    }
    else
    {
      flag_ave_music_speech = 0;
    }
  }

  /* MUSIC_DEFINITE and SPEECH_DEFINITE mode decision */

  /* ave_n_tonal */
  if ((init_mode_decision_result == TBD) && (ave_n_tonal <= 3))
  {
    init_mode_decision_result = SPEECH_DEFINITE;
  }
  if ((init_mode_decision_result == TBD) && (ave_n_tonal >= 15))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }

  /** ave_n_tonal_short
   */
  if ((init_mode_decision_result == TBD) && (ave_n_tonal_short >= 17))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }

  /** msd_spec_tilt
   */
  if ((init_mode_decision_result == TBD) && (msd_spec_tilt >= 0.01))
  {
    init_mode_decision_result = SPEECH_DEFINITE;
  }
  if ((init_mode_decision_result == TBD) && (framecnt >= 10) && (msd_spec_tilt <= 0.00004))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }

  /** n_tonal_low_frequency_ratio
   */
  if ((init_mode_decision_result == TBD) && (n_tonal_low_frequency_ratio <= 0.91))
  {
    init_mode_decision_result = MUSIC_DEFINITE;
  }

  /** MUSIC and SPEECH mode decision
   */

  /** msd_spec_tilt
   */
  if ((init_mode_decision_result == TBD) && (msd_spec_tilt <= 0.0002) && (*framecnt_xm >= 15))
  {
    init_mode_decision_result = MUSIC;
  }

  /** n_tonal_low_frequency_ratio
   */
  if ((init_mode_decision_result == TBD) && (n_tonal_low_frequency_ratio >= 0.95))
  {
    init_mode_decision_result = SPEECH;
  }
  if ((init_mode_decision_result == TBD) && (n_tonal_low_frequency_ratio <= 0.935))
  {
    init_mode_decision_result = MUSIC;
  }

  /** the rest of the frame to SPEECH
   */
  if (init_mode_decision_result == TBD)
  {
    init_mode_decision_result = SPEECH;
  }

  /** MUSIC mode decision according to changes of the MSD of the spectral tilt
   */

  /** compute the changes of the MSD of the spectral tilt
   */
  if ((msd_spec_tilt <= 0.007) && (init_mode_decision_result != SPEECH_DEFINITE))
  {
    if (init_mode_decision_result != SPEECH)
    {
      count_msd_st_music++;
    }
  }
  else
  {
    count_msd_st_music = 0;
  }

  if ((init_mode_decision_result != SPEECH_DEFINITE) && (count_msd_st_music >= 400) &&
      (border_state != BORDER_MUSIC_SPEECH_DEFINITE))
  {
    init_mode_decision_result = MUSIC;
  }

  /** update border flag
   */

  if (*flag_border != NO_BORDER)
  {
    border_state = *flag_border;
  }

  /** update BORDER_SPEECH_MUSIC_DEFINITE
   */
  if (((border_state == BORDER_MUSIC_SPEECH) || (border_state == BORDER_MUSIC_SPEECH_DEFINITE)) &&
      (init_mode_decision_result == MUSIC_DEFINITE) && (*framecnt_xm >= 20))
  {
    *flag_border = BORDER_SPEECH_MUSIC_DEFINITE;
    *framecnt_xm = 10;
    border_state = *flag_border;
  }

  /** update BORDER_MUSIC_SPEECH_DEFINITE
   */
  if (((border_state == BORDER_SPEECH_MUSIC) || (border_state == BORDER_SPEECH_MUSIC_DEFINITE)) &&
      (init_mode_decision_result == SPEECH_DEFINITE) && (*framecnt_xm >= 20))
  {
    *flag_border = BORDER_MUSIC_SPEECH_DEFINITE;
    *framecnt_xm = 10;
    border_state = *flag_border;
  }

  return init_mode_decision_result;
}

/**
 *  impeghe_smoothing_mode_decision
 *
 *  \brief Smoothing mode decision
 *
 *  \param [in] pstr_smooth_param	Pointer to smoothing params
 *
 *  \return WORD32				Mode decision result
 */
static WORD32 impeghe_smoothing_mode_decision(ia_smooth_params_struct *pstr_smooth_param)
{
  WORD32 *ptr_init_result_ahead = pstr_smooth_param->init_result_ahead;
  WORD32 flag_border = pstr_smooth_param->flag_border;
  WORD32 *ptr_flag_border_buf_behind = pstr_smooth_param->flag_border_buf_behind;
  WORD32 *ptr_flag_border_buf_ahead = pstr_smooth_param->flag_border_buf_ahead;
  FLOAT32 frame_energy = pstr_smooth_param->frame_energy;
  FLOAT32 *ptr_frame_energy_buf_behind = pstr_smooth_param->frame_energy_buf_behind;
  FLOAT32 *ptr_frame_energy_buf_ahead = pstr_smooth_param->frame_energy_buf_ahead;
  WORD32 *ptr_smoothing_result_buf = pstr_smooth_param->smoothing_result_buf;
  WORD32 *ptr_init_result_behind = pstr_smooth_param->init_result_behind;
  WORD32 init_mode_decision_result = pstr_smooth_param->init_mode_decision_result;
  WORD32 i;

  WORD32 mode_decision_result;

  WORD32 num_music, num_speech;

  /** update data array
   */

  /** update init_result_behind, init_result_ahead
   */
  for (i = 0; i < 99; i++)
  {
    ptr_init_result_behind[i] = ptr_init_result_behind[i + 1];
  }
  ptr_init_result_behind[99] = ptr_init_result_ahead[0];

  ptr_init_result_ahead[NUM_AHEAD_FRAMES - 1] = init_mode_decision_result;

  /** update flag_border_buf_behind, flag_border_buf_ahead
   * update frame_energy_buf_behind, frame_energy_buf_ahead
   */

  for (i = 0; i < 9; i++) /**< Clubbed the for loops into one*/
  {
    ptr_flag_border_buf_behind[i] = ptr_flag_border_buf_behind[i + 1];
    ptr_frame_energy_buf_behind[i] = ptr_frame_energy_buf_behind[i + 1];
  }
  ptr_flag_border_buf_behind[9] = ptr_flag_border_buf_ahead[0];
  ptr_frame_energy_buf_behind[9] = ptr_frame_energy_buf_ahead[0];

  ptr_flag_border_buf_ahead[NUM_AHEAD_FRAMES - 1] = flag_border;

  ptr_frame_energy_buf_ahead[NUM_AHEAD_FRAMES - 1] = frame_energy;

  /** smoothing according to past results
   */

  mode_decision_result = ptr_init_result_behind[99];

  /** update smoothing_result_buf
   */
  if (ptr_flag_border_buf_behind[9] == NO_BORDER)
  {
    for (i = 0; i < 99; i++)
    {
      ptr_smoothing_result_buf[i] = ptr_smoothing_result_buf[i + 1];
    }
    pstr_smooth_param->num_smoothing++;
  }
  else
  {
    for (i = 0; i < 99; i++)
    {
      ptr_smoothing_result_buf[i] = TBD;
    }
    pstr_smooth_param->num_smoothing = 1;
  }
  ptr_smoothing_result_buf[99] = ptr_init_result_behind[99];

  if (pstr_smooth_param->num_smoothing >= SMOOTHING_LENGTH)
  {
    num_music = 0;
    num_speech = 0;

    /** smoothed result count
     */
    for (i = 0; i < SMOOTHING_LENGTH; i++)
    {
      if ((ptr_smoothing_result_buf[100 - i] == SPEECH) ||
          (ptr_smoothing_result_buf[100 - i] == SPEECH_DEFINITE))
      {
        num_speech++;
      }
      else
      {
        num_music++;
      }
    }

    /** smoothing
     */
    if ((num_speech > num_music) && (init_mode_decision_result != MUSIC_DEFINITE))
    {
      mode_decision_result = SPEECH;
    }
    if ((num_music > num_speech) && (init_mode_decision_result != SPEECH_DEFINITE))
    {
      mode_decision_result = MUSIC;
    }
  }

  /** correct according to energies and ahead mode decision results
   */

  if ((mode_decision_result == MUSIC) && (ptr_frame_energy_buf_behind[9] <= 60))
  {
    for (i = 0; i < NUM_AHEAD_FRAMES; i++)
    {
      if ((ptr_init_result_ahead[i] == SPEECH_DEFINITE) || (ptr_init_result_ahead[i] == SPEECH))
      {
        pstr_smooth_param->flag_speech_definite = 1;
      }
    }
  }
  if ((pstr_smooth_param->flag_speech_definite == 1) && (mode_decision_result == MUSIC))
  {
    mode_decision_result = SPEECH;
  }
  else
  {
    pstr_smooth_param->flag_speech_definite = 0;
  }

  /** correct MUSIC mode
   */

  if (ptr_frame_energy_buf_behind[9] <= 65)
  {
    pstr_smooth_param->count_small_energy = 0;
  }
  else
  {
    pstr_smooth_param->count_small_energy++;
  }
  if (((ptr_flag_border_buf_ahead[NUM_AHEAD_FRAMES - 1] == BORDER_SPEECH_MUSIC) ||
       (ptr_flag_border_buf_ahead[NUM_AHEAD_FRAMES - 1] == BORDER_SPEECH_MUSIC_DEFINITE)) &&
      (pstr_smooth_param->count_small_energy <= 30))
  {
    pstr_smooth_param->flag_music_definite = 1;
  }
  if ((pstr_smooth_param->flag_music_definite == 1) &&
      ((mode_decision_result == SPEECH) || (mode_decision_result == SPEECH_DEFINITE)))
  {
    mode_decision_result = MUSIC;
  }
  else
  {
    pstr_smooth_param->flag_music_definite = 0;
  }

  return mode_decision_result;
}

/**
 *  impeghe_classification_1024
 *
 *  \brief signal classification
 *
 *  \param [in] pstr_sig_class	Pointer signal classification params struct
 *  \param [in] ptr_time_signal		Input data
 *  \param [in] pstr_scratch	Pointer to scratch memory
 *
 *  \return WORD32				Core mode
 */
static WORD32 impeghe_classification_1024(ia_classification_struct *pstr_sig_class,
                                          FLOAT32 *ptr_time_signal,
                                          impeghe_scratch_mem *pstr_scratch)
{
  WORD32 i;
  ia_tonal_params_struct pstr_ton_params;
  ia_smooth_params_struct smooth_param;
  ia_mode_params_struct pstr_mode_params;
  ia_spec_tilt_params_struct ptr_spec_params;

  ia_classification_buf_struct *pstr_buffers = &(pstr_sig_class->buffers);
  pFLOAT32 spec_tilt_buf = pstr_sig_class->spec_tilt_buf;
  pWORD32 n_tonal = pstr_sig_class->n_tonal;
  pWORD32 n_tonal_low_frequency = pstr_sig_class->n_tonal_low_frequency;
  pWORD32 framecnt_xm = &(pstr_sig_class->framecnt_xm);
  pWORD32 framecnt = &(pstr_sig_class->framecnt);
  pFLOAT32 ave_n_tonal_short_buf = pstr_sig_class->ave_n_tonal_short_buf;
  pFLOAT32 ave_n_tonal_buf = pstr_sig_class->ave_n_tonal_buf;
  pFLOAT32 msd_spec_tilt_buf = pstr_sig_class->msd_spec_tilt_buf;
  pFLOAT32 msd_spec_tilt_short_buf = pstr_sig_class->msd_spec_tilt_short_buf;

  FLOAT32 n_tonal_low_frequency_ratio;    /* the ratio of distribution of the numbers */
                                          /* of tonal in the low frequency domain     */
  FLOAT32 ave_n_tonal, ave_n_tonal_short; /**< the number of tonal */
  FLOAT32 msd_spec_tilt;                  /* the long-term MSD of spectral tilt */
  FLOAT32 msd_spec_tilt_short;            /* the short-term MSD of spectral tilt */

  WORD32 init_mode_decision_result; /* the initial mode decision */
  WORD32 flag_border = NO_BORDER;   /* flag of current border */

  WORD32 mode_decision_result; /* final mode decision result */

  if (pstr_sig_class->init_flag == 0)
  {
    /* initialize */
    pstr_sig_class->init_flag = 1;

    for (i = 0; i < 5; i++)
    {
      n_tonal[i] = 0;
      n_tonal_low_frequency[i] = 0;
      spec_tilt_buf[i] = 0;
      pstr_buffers->init_result_behind[i] = TBD;
      pstr_buffers->smoothing_result_buf[i] = TBD;

      ave_n_tonal_short_buf[i] = 0;
      ave_n_tonal_buf[i] = 0;
      msd_spec_tilt_buf[i] = 0;
      msd_spec_tilt_short_buf[i] = 0;

      pstr_buffers->frame_energy_buf_behind[i] = 0;
      pstr_buffers->flag_border_buf_behind[i] = NO_BORDER;
    }
    for (; i < 10; i++)
    {
      n_tonal[i] = 0;
      n_tonal_low_frequency[i] = 0;
      spec_tilt_buf[i] = 0;
      pstr_buffers->init_result_behind[i] = TBD;
      pstr_buffers->smoothing_result_buf[i] = TBD;

      pstr_buffers->frame_energy_buf_behind[i] = 0;
      pstr_buffers->flag_border_buf_behind[i] = NO_BORDER;
    }

    for (; i < 100; i++)
    {
      n_tonal[i] = 0;
      n_tonal_low_frequency[i] = 0;
      spec_tilt_buf[i] = 0;
      pstr_buffers->init_result_behind[i] = TBD;
      pstr_buffers->smoothing_result_buf[i] = TBD;
    }
    for (i = 0; i < NUM_AHEAD_FRAMES; i++)
    {
      pstr_buffers->frame_energy_buf_ahead[i] = 0;
      pstr_buffers->flag_border_buf_ahead[i] = NO_BORDER;
      pstr_buffers->init_result_ahead[i] = TBD;
    }
  }

  *framecnt += 1;
  *framecnt_xm += 1;

  pstr_ton_params.time_signal = (FLOAT32 *)ptr_time_signal;
  pstr_ton_params.framecnt_xm = *framecnt_xm;
  pstr_ton_params.n_tonal = n_tonal;
  pstr_ton_params.n_tonal_low_frequency = n_tonal_low_frequency;
  pstr_ton_params.n_tonal_low_frequency_ratio = &n_tonal_low_frequency_ratio;
  pstr_ton_params.ave_n_tonal = &ave_n_tonal;
  pstr_ton_params.ave_n_tonal_short = &ave_n_tonal_short;
  /** analysis tonal
   */
  impeghe_tonal_analysis(&pstr_ton_params, pstr_scratch);

  ptr_spec_params.time_signal = ptr_time_signal;
  ptr_spec_params.framecnt_xm = *framecnt_xm;
  ptr_spec_params.spec_tilt_buf = spec_tilt_buf;
  ptr_spec_params.msd_spec_tilt = &msd_spec_tilt;
  ptr_spec_params.msd_spec_tilt_short = &msd_spec_tilt_short;
  /** analysis spectral tilt
   */
  impeghe_spectral_tilt_analysis(&ptr_spec_params);

  pstr_mode_params.framecnt = *framecnt;
  pstr_mode_params.framecnt_xm = framecnt_xm;
  pstr_mode_params.flag_border = &flag_border;
  pstr_mode_params.ave_n_tonal_short = ave_n_tonal_short;
  pstr_mode_params.ave_n_tonal = ave_n_tonal;
  pstr_mode_params.ave_n_tonal_short_buf = ave_n_tonal_short_buf;
  pstr_mode_params.ave_n_tonal_buf = ave_n_tonal_buf;
  pstr_mode_params.msd_spec_tilt = msd_spec_tilt;
  pstr_mode_params.msd_spec_tilt_short = msd_spec_tilt_short;
  pstr_mode_params.msd_spec_tilt_buf = msd_spec_tilt_buf;
  pstr_mode_params.msd_spec_tilt_short_buf = msd_spec_tilt_short_buf;
  pstr_mode_params.n_tonal_low_frequency_ratio = n_tonal_low_frequency_ratio;
  pstr_mode_params.frame_energy = ptr_spec_params.frame_energy;
  /** initial mode decision and boundary decisions
   */
  init_mode_decision_result = impeghe_init_mode_decision(&pstr_mode_params);

  smooth_param.flag_border_buf_behind = pstr_buffers->flag_border_buf_behind;
  smooth_param.flag_border_buf_ahead = pstr_buffers->flag_border_buf_ahead;
  smooth_param.frame_energy = ptr_spec_params.frame_energy;
  smooth_param.frame_energy_buf_behind = pstr_buffers->frame_energy_buf_behind;
  smooth_param.frame_energy_buf_ahead = pstr_buffers->frame_energy_buf_ahead;
  smooth_param.smoothing_result_buf = pstr_buffers->smoothing_result_buf;
  smooth_param.init_result_ahead = pstr_buffers->init_result_ahead;
  smooth_param.flag_border = flag_border;
  smooth_param.init_result_behind = pstr_buffers->init_result_behind;
  smooth_param.init_mode_decision_result = init_mode_decision_result;
  smooth_param.flag_speech_definite = 0;
  smooth_param.count_small_energy = 0;
  smooth_param.flag_music_definite = 0;
  smooth_param.num_smoothing = 0;
  /* smoothing */
  mode_decision_result = impeghe_smoothing_mode_decision(&smooth_param);

  return mode_decision_result;
}

/**
 *  impeghe_classification
 *
 *  \brief signal classification wrapper function
 *
 *  \param [in] pstr_sig_class	Pointer signal classification params struct
 *  \param [in] pstr_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
VOID impeghe_classification(ia_classification_struct *pstr_sig_class,
                            impeghe_scratch_mem *pstr_scratch)
{
  WORD32 n_frames, n_class, avg_cls, nf;
  WORD32 i;
  FLOAT32 *ptr_time_signal = pstr_scratch->p_time_signal;
  WORD32 mode_decision_result;

  n_frames = pstr_sig_class->n_buffer_samples / FRAME_LEN_LONG;
  for (nf = 0; nf < n_frames; nf++)
  {
    for (i = 0; i < FRAME_LEN_LONG; i++)
    {
      ptr_time_signal[i] = pstr_sig_class->input_samples[FRAME_LEN_LONG * nf + i];
    }

    /* classification of 1024-frame */
    mode_decision_result =
        impeghe_classification_1024(pstr_sig_class, ptr_time_signal, pstr_scratch);

    /* coding mode decision of 1024-frame */
    if ((mode_decision_result == MUSIC) || (mode_decision_result == MUSIC_DEFINITE))
    {
      pstr_sig_class->coding_mode = FD_MODE;
    }
    else if ((mode_decision_result == SPEECH) || (mode_decision_result == SPEECH_DEFINITE))
    {
      pstr_sig_class->coding_mode = TD_MODE;
    }

    pstr_sig_class->class_buf[pstr_sig_class->n_buf_class + nf] = pstr_sig_class->coding_mode;
    pstr_sig_class->pre_mode = pstr_sig_class->coding_mode;
  }

  /* merge 1024-frame results */
  pstr_sig_class->n_buf_class += n_frames;
  n_class = (pstr_sig_class->n_class_frames > pstr_sig_class->n_buf_class)
                ? pstr_sig_class->n_buf_class
                : pstr_sig_class->n_class_frames;
  {
    WORD32 min_cls, max_cls;

    avg_cls = 0;
    min_cls = max_cls = pstr_sig_class->class_buf[0];
    for (i = 1; i < n_class; i++)
    {
      if (pstr_sig_class->class_buf[i] > max_cls)
      {
        max_cls = pstr_sig_class->class_buf[i];
      }
      else if (pstr_sig_class->class_buf[i] < min_cls)
      {
        min_cls = pstr_sig_class->class_buf[i];
      }
    }

    avg_cls = 0;
    for (i = 0; i < n_class; i++)
    {
      if (pstr_sig_class->class_buf[i] == max_cls)
      {
        avg_cls += 1;
      }
      if (pstr_sig_class->class_buf[i] == min_cls)
      {
        avg_cls += -1;
      }
    }

    if (avg_cls > 0)
    {
      pstr_sig_class->coding_mode = max_cls;
    }
    else
    {
      pstr_sig_class->coding_mode = min_cls;
    }
  }

  /* shift, save pre_mode and unused class */
  if (n_class > 0)
  {
    pstr_sig_class->pre_mode = pstr_sig_class->class_buf[n_class - 1];
  }
  pstr_sig_class->n_buf_class -= n_class;
  pstr_sig_class->n_buffer_samples -= FRAME_LEN_LONG * n_frames;

  WORD32 minimum = min(pstr_sig_class->n_buf_class, pstr_sig_class->n_buffer_samples);
  if (minimum == pstr_sig_class->n_buf_class)
  {
    for (i = 0; i < minimum; i++)
    {
      pstr_sig_class->class_buf[i] = pstr_sig_class->class_buf[i + n_class];
      pstr_sig_class->input_samples[i] =
          pstr_sig_class->input_samples[i + FRAME_LEN_LONG * n_frames];
    }

    /* shift, save unused samples */
    for (; i < pstr_sig_class->n_buffer_samples; i++)
    {
      pstr_sig_class->input_samples[i] =
          pstr_sig_class->input_samples[i + FRAME_LEN_LONG * n_frames];
    }
  }
  else
  {
    for (i = 0; i < minimum; i++)
    {
      pstr_sig_class->class_buf[i] = pstr_sig_class->class_buf[i + n_class];
      pstr_sig_class->input_samples[i] =
          pstr_sig_class->input_samples[i + FRAME_LEN_LONG * n_frames];
    }

    /* shift, save unused samples */
    for (; i < pstr_sig_class->n_buf_class; i++)
    {
      pstr_sig_class->class_buf[i] = pstr_sig_class->class_buf[i + n_class];
    }
  }
}

/**
 *  impeghe_init_classification
 *
 *  \brief initialize signal classification params
 *
 *  \param [in,out] pstr_sig_class	Pointer signal classification params struct
 *
 *  \return VOID
 */
VOID impeghe_init_classification(ia_classification_struct *pstr_sig_class)
{
  pstr_sig_class->pre_mode = FD_MODE;

  pstr_sig_class->n_buffer_samples = 0;
  memset(pstr_sig_class->input_samples, 0, 3840 * 2 * sizeof(FLOAT32));
  pstr_sig_class->n_class_frames = 2;
  pstr_sig_class->n_buf_class = 0;

  pstr_sig_class->is_switch_mode = 1;

  pstr_sig_class->framecnt = 0;
  pstr_sig_class->init_flag = 0;
  pstr_sig_class->framecnt_xm = 0;

  memset(&pstr_sig_class->buffers, 0, sizeof(ia_classification_buf_struct));
  memset(pstr_sig_class->spec_tilt_buf, 0, sizeof(FLOAT32) * 100);
  memset(pstr_sig_class->n_tonal, 0, sizeof(WORD32) * 100);
  memset(pstr_sig_class->n_tonal_low_frequency, 0, sizeof(WORD32) * 100);
  memset(pstr_sig_class->msd_spec_tilt_buf, 0, sizeof(FLOAT32) * 5);
  memset(pstr_sig_class->msd_spec_tilt_short_buf, 0, sizeof(FLOAT32) * 5);
  memset(pstr_sig_class->ave_n_tonal_short_buf, 0, sizeof(FLOAT32) * 5);
  memset(pstr_sig_class->ave_n_tonal_buf, 0, sizeof(FLOAT32) * 5);
  return;
}
