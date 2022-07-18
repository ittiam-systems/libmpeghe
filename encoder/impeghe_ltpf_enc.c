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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_cnst.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"
#include "impeghe_psy_utils.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"
#include "impeghe_ltpf_enc.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_ltpf_init
 *
 *  \brief LTPF data initialization function.
 *
 *  \param [in,out] ltpf_data Pointer to LTPF data structure.
 *  \param [in    ] input_sr  Input sampling frequency value.
 *
 *  \return WORD32 Error code incase of unsupported sampling frequency.
 */
WORD32 impeghe_ltpf_init(ia_usac_ltpf_data_struct *ltpf_data, WORD32 input_sr)
{

  ltpf_data->ltpf_pitch_min = (WORD32)(34.f * ((FLOAT32)input_sr / 2.f) / 12800.f + 0.5f) * 2;
  ltpf_data->ltpf_pitch_fr2 = 324 - ltpf_data->ltpf_pitch_min;
  ltpf_data->ltpf_pitch_fr1 = 320;
  ltpf_data->ltpf_pitch_max = 54 + 6 * ltpf_data->ltpf_pitch_min;
  ltpf_data->ltpf_pitch_res = 2;
  ltpf_data->ltpf_pitch_res_by_2 = ltpf_data->ltpf_pitch_res >> 1;
  ltpf_data->ltpf_mem_in_len = ltpf_data->ltpf_pitch_max + 15;
  if (input_sr == 48000)
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[0];
  }
  else if (input_sr == 24000)
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[1];
  }
  else if (input_sr == 44100)
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[5];
  }
  else if ((input_sr == 32000) || (input_sr == 29400))
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[6];
  }
  else if (input_sr == 22050)
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[7];
  }
  else if ((input_sr == 16000) || (input_sr == 14700))
  {
    ltpf_data->fir_lpf_coefs = impeghe_ltpf_fir_inter4_1[8];
  }
  else
  {
    return -1;
  }

  ltpf_data->pitch_search_delta = (WORD32)((4.0f / 12800) * input_sr + 0.5);
  ltpf_data->pitch_search_l_interpol = ltpf_data->pitch_search_delta;
  return 0;
}

/**
 *  impeghe_search_max_indice
 *
 *  \brief Searches for the maximum value in a buffer and returns the index of same.
 *
 *  \param [in] in  Pointer to input buffer.
 *  \param [in] len Length of input buffer.
 *
 *  \return WORD32 Index of maximum value in input buffer.
 */
WORD32 impeghe_search_max_indice(FLOAT32 *in, WORD32 len)
{
  WORD32 max_i = 0, i = 0;
  FLOAT32 max = 0;

  if (len <= 0)
  {
    return -128;
  }

  for (i = 0; i < len; i++)
  {
    if (in[i] > max)
    {
      max = in[i];
      max_i = i;
    }
  }

  return max_i;
}

/**
 *  impeghe_ol_pitch_estim
 *
 *  \brief Implements open loop pitch estimation.
 *
 *  \param [in ] x            Pointer to input buffer.
 *  \param [in ] ltpf_data    Pointer to LTPF data structure.
 *  \param [out] ol_pitch     Pointer to open loop method estimated pitch values.
 *  \param [out] normcorr_out Normalized correlation values holding buffer.
 *  \param [in ] x_length     Length of input buffer.
 *  \param [in ] pstr_scratch Pointer to scratch buffer.
 *
 *  \return VOID
 */
VOID impeghe_ol_pitch_estim(FLOAT32 *x, ia_usac_ltpf_data_struct *ltpf_data, WORD32 *ol_pitch,
                            FLOAT32 *normcorr_out, WORD32 x_length,
                            impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *prev_x = ltpf_data->olpa_mem_x;
  WORD32 *prev_T = &ltpf_data->olpa_mem_pitch;
  FLOAT32 *buf_tmp = pstr_scratch->p_ol_pitch_buf_tmp;
  FLOAT32 *speech_buf = pstr_scratch->p_ol_pitch_speech_buf;
  FLOAT32 *w_table = pstr_scratch->p_ol_pitch_w_table;
  FLOAT32 *R = pstr_scratch->p_ol_pitch_R;
  FLOAT32 *R0 = pstr_scratch->p_ol_pitch_R0;

  FLOAT32 norm_corr = 0, sum = 0, sum_sq1 = 0, sum_sq2 = 0, sum_tmp = 0, norm_corr2 = 0;
  WORD32 i = 0, j = 0, T1 = 0, T2 = 0, k_min2 = 0, k_max2 = 0, L = 0;
  WORD32 kmin = ltpf_data->ltpf_pitch_min / 2;
  WORD32 kmax = ltpf_data->ltpf_pitch_max / 2;
  WORD32 len_corr = kmax - kmin + 1;

  memcpy(speech_buf, prev_x, kmax * sizeof(FLOAT32));
  memcpy(&speech_buf[kmax], x, x_length * sizeof(FLOAT32));
  memcpy(prev_x, &speech_buf[x_length], kmax * sizeof(FLOAT32));

  for (i = kmin; i <= kmax; i++)
  {
    sum = 0;
    memcpy(buf_tmp, &speech_buf[kmax - i], x_length * sizeof(FLOAT32));

    for (j = 0; j < x_length; j++)
    {
      sum += x[j] * buf_tmp[j];
    }
    R0[i - kmin] = sum;
  }

  memmove(R, R0, len_corr * sizeof(FLOAT32));

  for (i = kmin; i < kmax; i++)
  {
    w_table[i - kmin] = (FLOAT32)(1 - 0.5 * ((FLOAT32)(i - kmin) / (kmax - kmin)));
  }

  for (i = 0; i < len_corr; i++)
  {
    R0[i] = R0[i] * w_table[i];
  }

  L = impeghe_search_max_indice(R0, len_corr);
  T1 = L + kmin;

  memcpy(buf_tmp, &speech_buf[kmax - T1], x_length * sizeof(FLOAT32));

  sum_tmp = 0;
  sum_sq1 = 0;
  sum_sq2 = 0;

  for (i = 0; i < x_length; i++)
  {
    sum_tmp += x[i] * buf_tmp[i];
    sum_sq1 += x[i] * x[i];
    sum_sq2 += buf_tmp[i] * buf_tmp[i];
  }

  sum_sq1 = sum_sq1 * sum_sq2;
  sum_sq1 = sqrtf(sum_sq1) + POW_10_MIN_5;

  norm_corr = sum_tmp / sum_sq1;
  norm_corr = MAX(0, norm_corr);

  k_min2 = MAX(kmin, *prev_T - 15);
  k_max2 = MIN(kmax, *prev_T + 15);

  /* Distance can be negative here */
  if (((k_max2 - kmin + 1) - (k_min2 - kmin)) > 0)
  {
    memcpy(buf_tmp, &R[k_min2 - kmin], ((k_max2 - kmin + 1) - (k_min2 - kmin)) * sizeof(FLOAT32));
  }

  L = impeghe_search_max_indice(buf_tmp, (k_max2 - kmin + 1) - (k_min2 - kmin));

  if (L == -128)
  {
    T2 = -128;
  }
  else
  {
    T2 = L + k_min2;
  }

  if ((T2 != T1) && (T2 != -128))
  {
    memcpy(buf_tmp, &speech_buf[kmax - T2], x_length * sizeof(FLOAT32));

    sum_tmp = 0;
    sum_sq1 = 0;
    sum_sq2 = 0;
    for (i = 0; i < x_length; i++)
    {
      sum_tmp += x[i] * buf_tmp[i];
      sum_sq1 += x[i] * x[i];
      sum_sq2 += buf_tmp[i] * buf_tmp[i];
    }

    sum_sq1 = sum_sq1 * sum_sq2;
    sum_sq1 = sqrtf(sum_sq1) + powf(10.0, -5.0);

    norm_corr2 = sum_tmp / sum_sq1;

    norm_corr2 = MAX(0, norm_corr2);

    if (norm_corr2 > (norm_corr * 0.85))
    {
      T1 = T2;
      norm_corr = norm_corr2;
    }
  }

  *prev_T = T1;
  T1 = T1 * 2;
  *ol_pitch = T1;
  *normcorr_out = norm_corr;
}

/**
 *  impeghe_get_gain
 *
 *  \brief Calclulates gain value of a buffer using auto and
 *         cross correlations.
 *
 *  \param [in] x      Pointer to buffer x.
 *  \param [in] y      Pointer to buffer y.
 *  \param [in] length Length of the buffers.
 *
 *  \return Returns computed gain(scaling factor) value.
 */
FLOAT32 impeghe_get_gain(const FLOAT32 *x, FLOAT32 *y, WORD32 length)
{
  FLOAT32 corr = 0.0f, ener = 1e-6f;
  WORD16 i;

  for (i = 0; i < length; i++)
  {
    corr += x[i] * y[i];
    ener += y[i] * y[i];
  }

  return (corr / ener);
}

/**
 *  impeghe_get_ltpf_gain_idx
 *
 *  \brief Calculates LTPF gain index value.
 *
 *  \param [in ] speech      Pointer to input speech buffer.
 *  \param [in ] pred_speech Pointer to predicted speech buffer.
 *  \param [in ] frame_len   Frame length value.
 *  \param [out] gain        Pointer to gain value.
 *  \param [out] gain_index  Pointer to gain index value.
 *
 *  \return VOID
 */
VOID impeghe_get_ltpf_gain_idx(const FLOAT32 *speech, FLOAT32 *pred_speech, WORD32 frame_len,
                               FLOAT32 *gain, WORD32 *gain_index)
{
  *gain = impeghe_get_gain(speech, pred_speech, frame_len);

  /* Quantize gain */
  if (*gain >= 0.875f)
  {
    *gain_index = 3; /* 1.00/2 */
  }
  else if (*gain >= 0.625f)
  {
    *gain_index = 2; /* 0.75/2 */
  }
  else if (*gain >= 0.375f)
  {
    *gain_index = 1; /* 0.50/2 */
  }
  else if (*gain >= 0.125f)
  {
    *gain_index = 0; /* 0.25/2 */
  }
  else
  {
    *gain_index = -1; /* escape */
  }

  /* Dequantize gain */
  *gain = (FLOAT32)(*gain_index + 1) * 0.0625f;
}

/**
 *  impeghe_ltpf_predict_signal
 *
 *  \brief Computes LTPF prediction signal.
 *
 *  \param [in ] input_exc   Pointer to input excitation signal.
 *  \param [out] pred_exc    Pointer to predicted excitation signal.
 *  \param [in ] pitch_int   Integer pitch value.
 *  \param [in ] pitch_frac  Fractional pitch value.
 *  \param [in ] frac_max    Maximum fractional pitch.
 *  \param [in ] len_subfrm  Length of subframe.
 *
 *  \return VOID
 */
VOID impeghe_ltpf_predict_signal(const FLOAT32 *input_exc, FLOAT32 *pred_exc,
                                 const WORD16 pitch_int, WORD16 pitch_frac, const WORD16 frac_max,
                                 const WORD16 len_subfrm)
{
  WORD16 j;
  FLOAT32 s;
  const FLOAT32 *x0, *win;

  x0 = &input_exc[-pitch_int - 1];
  pitch_frac = -pitch_frac;

  if (pitch_frac < 0)
  {
    pitch_frac += frac_max;
    x0--;
  }

  if (frac_max == 6)
  {
    win = &inter6_2tcx2[pitch_frac][0];
  }
  else
  {
    win = &inter4_2tcx2[pitch_frac][0];
  }

  for (j = 0; j < len_subfrm; j++)
  {
    s = win[1] * x0[1] + win[2] * x0[2];
    pred_exc[j] = s + win[0] * x0[0] + win[3] * x0[3];
    x0++;
  }

  return;
}

/**
 *  impeghe_ltpf_encode
 *
 *  \brief LTPF encoding function.
 *
 *  \param [in] ltpf_data          Pointer to LTPF data structure.
 *  \param [in] x                  Pointer to input buffer.
 *  \param [in] x_length           Length of input buffer.
 *  \param [in] pitch_ol           Open loop pitch value.
 *  \param [in] pitch_ol_norm_corr Open loop pitch value - normalized.
 *  \param [in] pstr_scratch       Pointer to scratch buffer.
 *
 *  \return Error code if any.
 */
WORD32 impeghe_ltpf_encode(ia_usac_ltpf_data_struct *ltpf_data, const FLOAT32 *x, WORD32 x_length,
                           WORD32 pitch_ol, FLOAT32 pitch_ol_norm_corr,
                           impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *prev_x = ltpf_data->ltpf_mem_in;
  FLOAT32 *speech_buffer = pstr_scratch->p_ltpf_encode_speech_buffer;
  FLOAT32 *buf_tmp = pstr_scratch->p_ltpf_encode_buf_tmp;
  FLOAT32 *cor = pstr_scratch->p_ltpf_encode_cor;
  FLOAT32 *cor_int = pstr_scratch->p_ltpf_encode_cor_int;
  FLOAT32 *curr_frame = cor;
  FLOAT32 *pred_frame = cor_int;
  FLOAT32 *pred_speech = speech_buffer;
  WORD32 i = 0, j = 0, k = 0, len_prev_x = 0, t0_min = 0, t0_max = 0, t_min = 0, t_max = 0, L = 0,
         T1 = 0, pitch_int = 0, pitch_fr = 0, midpoint = 0, delta_up = 0, delta_down = 0,
         pitch = 0;
  FLOAT32 gain = 0, norm_corr = 0, sum = 0, sum1 = 0, sum2 = 0, sum3 = 0;

  /* Signal Buffer */
  len_prev_x = ltpf_data->ltpf_mem_in_len + 1;

  memcpy(speech_buffer, prev_x, len_prev_x * sizeof(FLOAT32));
  memcpy(&speech_buffer[len_prev_x], x, x_length * sizeof(FLOAT32));
  memcpy(prev_x, &speech_buffer[x_length], len_prev_x * sizeof(FLOAT32));

  ltpf_data->ltpf_active = 0;
  norm_corr = 0;

  if (pitch_ol_norm_corr > 0.6f)
  {
    /* Search Bounds */
    t0_min = pitch_ol - ltpf_data->pitch_search_delta;
    t0_max = pitch_ol + ltpf_data->pitch_search_delta;
    t0_min = MAX(t0_min, ltpf_data->ltpf_pitch_min);
    t0_max = MIN(t0_max, ltpf_data->ltpf_pitch_max);

    /* Cross-Correlation Bounds */
    t_min = t0_min - ltpf_data->pitch_search_l_interpol;
    t_max = t0_max + ltpf_data->pitch_search_l_interpol;

    /* Compute Cross-Correlation */
    for (i = t_min; i <= t_max; i++)
    {
      sum = 0;
      for (j = len_prev_x; j < (x_length + len_prev_x); j++)
      {
        sum += speech_buffer[j] * speech_buffer[j - i];
      }
      cor[i - t_min] = sum; // R(k)
    }

    /* Find Integer Pitch-Lag */
    L = impeghe_search_max_indice(&cor[ltpf_data->pitch_search_l_interpol],
                                  t_max - t_min - 2 * ltpf_data->pitch_search_l_interpol + 1);

    T1 = L + t0_min;
    if (!(T1 >= t0_min && T1 <= t0_max))
      return -1;

    /* Find Fractional Pitch-Lag */
    if ((T1 <= ltpf_data->ltpf_pitch_max && T1 >= ltpf_data->ltpf_pitch_fr1) ||
        (T1 < ltpf_data->ltpf_pitch_fr1 && T1 >= ltpf_data->ltpf_pitch_fr2))
    {
      pitch_int = T1;
      pitch_fr = 0;
    }
    else if (T1 < ltpf_data->ltpf_pitch_fr2 && T1 >= ltpf_data->ltpf_pitch_min)
    {
      j = 0;
      for (i = 0; i < (t0_max - t0_min + 1); i++)
      {
        sum = 0;
        k = 0;
        for (j = i; j < i + 32; j++)
        {
          sum += cor[j] * ltpf_data->fir_lpf_coefs[k];
          k++;
        }
        cor_int[i] = sum;
      }

      midpoint = (T1 - t0_min) + 1;
      delta_up = 1;

      if (T1 == t0_min)
      {
        delta_down = 0;
      }
      else
      {
        delta_down = 1;
      }

      j = 0;
      for (i = midpoint - delta_down; i < midpoint + delta_up; i++)
      {
        buf_tmp[j] = cor_int[i];
        j++;
      }

      pitch_fr =
          impeghe_search_max_indice(buf_tmp, ((midpoint + delta_up) - (midpoint - delta_down)));

      if (pitch_fr >= 0)
      {
        pitch_int = T1;
      }
      else
      {
        pitch_int = T1 - 1;
      }
    }

    if (!(pitch_fr >= 0 && pitch_fr <= 1))
    {
      return -1;
    }

    if (pitch_int < ltpf_data->ltpf_pitch_fr2)
    { // pi < 260
      ltpf_data->ltpf_pitch_index = pitch_int * ltpf_data->ltpf_pitch_res + pitch_fr -
                                    (ltpf_data->ltpf_pitch_min * ltpf_data->ltpf_pitch_res);
    }
    else if (pitch_int < ltpf_data->ltpf_pitch_fr1 && pitch_int >= ltpf_data->ltpf_pitch_fr2)
    { // 320 > pi > 260
      ltpf_data->ltpf_pitch_index =
          pitch_int * ltpf_data->ltpf_pitch_res_by_2 -
          (ltpf_data->ltpf_pitch_fr2 * ltpf_data->ltpf_pitch_res_by_2) +
          ((ltpf_data->ltpf_pitch_fr2 - ltpf_data->ltpf_pitch_min) * ltpf_data->ltpf_pitch_res);
    }
    else
    { // pi > 320
      ltpf_data->ltpf_pitch_index =
          ((pitch_int - ltpf_data->ltpf_pitch_fr1) / ltpf_data->ltpf_pitch_res) +
          ltpf_data->ltpf_pitch_res * (ltpf_data->ltpf_pitch_fr2 - ltpf_data->ltpf_pitch_min) +
          (ltpf_data->ltpf_pitch_fr1 - ltpf_data->ltpf_pitch_fr2);
    }

    if (!(ltpf_data->ltpf_pitch_index >= 0 && ltpf_data->ltpf_pitch_index < 512))
      return -1;

    pitch = pitch_int + pitch_fr / ltpf_data->ltpf_pitch_res;

    {
      for (k = 0; k < x_length; k++)
      {
        sum = 0;
        j = 0;

        for (i = len_prev_x + k + LTPF_ENC_INTER_LEN - 1;
             i >= len_prev_x + k - LTPF_ENC_INTER_LEN; i--)
        {
          sum += speech_buffer[i] * impeghe_ltpf_inter_filter[0][j];
          j++;
        }
        curr_frame[k] = sum;

        sum = 0;
        j = 0;

        for (i = len_prev_x + k - pitch_int + LTPF_ENC_INTER_LEN - 1;
             i >= len_prev_x + k - pitch_int - LTPF_ENC_INTER_LEN; i--)
        {
          sum += speech_buffer[i] * impeghe_ltpf_inter_filter[pitch_fr][j];
          j++;
        }
        pred_frame[k] = sum;
      }
    }

    /* Normalized Correlation */
    for (i = 0; i < x_length; i++)
    {
      sum1 += curr_frame[i] * pred_frame[i];
    }

    for (i = 0; i < x_length; i++)
    {
      sum2 += curr_frame[i] * curr_frame[i];
    }

    for (i = 0; i < x_length; i++)
    {
      sum3 += pred_frame[i] * pred_frame[i];
    }

    sum2 = sqrtf(sum2 * sum3) + POW_10_MIN_5;
    norm_corr = sum1 / sum2;

    if (norm_corr >= -1.00001 && norm_corr <= 1.00001)
    {
      norm_corr = MIN(1, MAX(-1, norm_corr));
      if (norm_corr < 0)
      {
        norm_corr = 0;
      }
    }
    else
    {
      return -1;
    }

    /* Decision if ltpf active */
    if ((ltpf_data->ltpf_mem_ltpf_on == 0 && ltpf_data->ltpf_mem_normcorr > 0.94 &&
         norm_corr > 0.94) ||
        (ltpf_data->ltpf_mem_ltpf_on == 1 && norm_corr > 0.9) ||
        (ltpf_data->ltpf_mem_ltpf_on == 1 && abs(pitch - ltpf_data->ltpf_mem_pitch) < 2 &&
         (norm_corr - ltpf_data->ltpf_mem_normcorr) > -0.1 && norm_corr > 0.84))
    {
      ltpf_data->ltpf_active = 1;
    }

    if (ltpf_data->ltpf_active)
    {
      impeghe_ltpf_predict_signal(&speech_buffer[len_prev_x], pred_speech, pitch_int, pitch_fr,
                                  ltpf_data->ltpf_pitch_res, x_length);

      impeghe_get_ltpf_gain_idx(x, pred_speech, x_length, &gain, &ltpf_data->ltpf_gain_index);

      if (gain <= 0)
      {
        ltpf_data->ltpf_gain_index = 0;
      }
    }
  }
  else
  {
    gain = 0;
    norm_corr = pitch_ol_norm_corr;
    pitch = 0;
  }

  if (gain < 0)
  {
    ltpf_data->ltpf_pitch_index = 0;
    ltpf_data->ltpf_active = 0;
    ltpf_data->ltpf_gain_index = 0;
  }

  ltpf_data->ltpf_mem_normcorr = norm_corr;
  ltpf_data->ltpf_mem_ltpf_on = ltpf_data->ltpf_active;
  ltpf_data->ltpf_mem_pitch = pitch;

  return 0;
}
/** @} */ /* End of CoreEncProc */
