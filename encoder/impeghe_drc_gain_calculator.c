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

#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_cnst.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_rom.h"
#include "impeghe_fft.h"
#include "impeghe_drc_tables.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_drc_enc.h"
#include "impeghe_drc_tables.h"
#include "impeghe_drc_mux.h"

/**
 *  impeghe_drc_compand_update_volume
 *
 *  \brief Updates volume data
 *
 *  \param [in,out] pstr_channel_param  Pointer to DRC compand channel params structure
 *  \param [in] in_value                Input value
 *
 *  \return VOID
 */
static VOID
impeghe_drc_compand_update_volume(ia_drc_compand_chan_param_struct *pstr_channel_param,
                                  FLOAT64 in_value)
{
  FLOAT64 delta = in_value - pstr_channel_param->volume;

  if (delta <= 0.0)
  {
    pstr_channel_param->volume += delta * pstr_channel_param->decay;
  }
  else
  {
    pstr_channel_param->volume += delta * pstr_channel_param->attack;
  }
}

/**
 *  impeghe_drc_compand_get_volume
 *
 *  \brief Gets volume data
 *
 *  \param [in] pstr_drc_compand		Pointer to DRC compand structure
 *  \param [in] in_lin							Input volume
 *
 *  \return FLOAT64  Gain value
 */
static FLOAT64 impeghe_drc_compand_get_volume(ia_drc_compand_struct *pstr_drc_compand,
                                              FLOAT64 in_lin)
{
  ULOOPIDX idx;
  FLOAT64 in_log, out_log;
  ia_drc_compand_segment_struct *pstr_compand_segment;

  if (in_lin < pstr_drc_compand->in_min_lin)
  {
    return pstr_drc_compand->out_min_lin;
  }

  in_log = log(in_lin);

  for (idx = 1; idx < pstr_drc_compand->nb_segments; idx++)
  {
    if (in_log <= pstr_drc_compand->str_segment[idx].x)
    {
      break;
    }
  }

  pstr_compand_segment = &pstr_drc_compand->str_segment[idx - 1];
  in_log -= pstr_compand_segment->x;
  out_log = pstr_compand_segment->y +
            in_log * (pstr_compand_segment->a * in_log + pstr_compand_segment->b);

  return exp(out_log);
}

/**
 *  impeghe_td_drc_gain_calc_process
 *
 *  \brief Calculates DRC gain value in time domain
 *
 *  \param [in,out] pstr_drc_gain_enc         Pointer to DRC gain encode structure
 *  \param [in] drc_coefficients_uni_drc_idx  Index of DRC coefficient
 *  \param [in] gain_set_idx                  Index of gain set
 *  \param [in] num_samples                   Frame size
 *  \param [in] in_buff                       Pointer to input buffer
 *  \param [out] out_buff                     Pointer to output buffer
 *
 *  \return VOID
 */
VOID impeghe_td_drc_gain_calc_process(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                      WORD32 drc_coefficients_uni_drc_idx, WORD32 gain_set_idx,
                                      WORD32 num_samples, FLOAT32 *in_buff, FLOAT32 *out_buff)
{
  LOOPIDX idx;
  FLOAT64 gain;
  ia_drc_compand_chan_param_struct *pstr_channel_param;
  ia_drc_compand_struct *pstr_drc_compand =
      &pstr_drc_gain_enc->str_drc_compand[drc_coefficients_uni_drc_idx][gain_set_idx];

  pstr_channel_param = &pstr_drc_compand->str_channel_param;

  for (idx = 0; idx < num_samples; idx++)
  {
    impeghe_drc_compand_update_volume(pstr_channel_param, fabs((FLOAT64)in_buff[idx] / 32768.0));

    gain = impeghe_drc_compand_get_volume(pstr_drc_compand, pstr_channel_param->volume);
    out_buff[idx] = (FLOAT32)(20.0 * log10(gain));
  }
}

/**
 *  impeghe_td_drc_gain_calc_init
 *
 *  \brief Initializes DRC gain calculator in time domain
 *
 *  \param [in,out] pstr_drc_gain_enc         Pointer to DRC gain encode structure
 *  \param [in] drc_coefficients_uni_drc_idx  Index of DRC coefficient
 *  \param [in] gain_set_idx                  Index of gain set
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_td_drc_gain_calc_init(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                           WORD32 drc_coefficients_uni_drc_idx,
                                           WORD32 gain_set_idx)
{
  ULOOPIDX i, j;
  UWORD32 num_points;
  FLOAT64 g1, g2;
  FLOAT64 x, y, cx, cy, r;
  FLOAT64 inp_1, inp_2, out_1, out_2, theta, length, radius;
  ia_drc_compand_struct *pstr_drc_compand;
  ia_drc_compand_chan_param_struct *pstr_chan_param;

  if ((drc_coefficients_uni_drc_idx >= MAX_DRC_COEFF_COUNT) ||
      (gain_set_idx >= GAIN_SET_COUNT_MAX))
  {
    return IMPEGHE_CONFIG_FATAL_DRC_COMPAND_FAIL;
  }

  pstr_drc_compand =
      &pstr_drc_gain_enc->str_drc_compand[drc_coefficients_uni_drc_idx][gain_set_idx];

  pstr_drc_compand->nb_segments = (pstr_drc_compand->nb_points + 4) * 2;

  for (i = 0; i < pstr_drc_compand->nb_points; i++)
  {
    if (i && pstr_drc_compand->str_segment[2 * ((i - 1) + 1)].x >
                 pstr_drc_compand->str_segment[2 * ((i) + 1)].x)
    {
      return IMPEGHE_CONFIG_FATAL_DRC_COMPAND_FAIL;
    }
    pstr_drc_compand->str_segment[2 * (i + 1)].y -= pstr_drc_compand->str_segment[2 * (i + 1)].x;
  }
  num_points = pstr_drc_compand->nb_points;

  if (num_points == 0 || pstr_drc_compand->str_segment[2 * ((num_points - 1) + 1)].x)
  {
    num_points++;
  }

  pstr_drc_compand->str_segment[0].x =
      pstr_drc_compand->str_segment[2].x - 2 * pstr_drc_compand->width_db;
  pstr_drc_compand->str_segment[0].y = pstr_drc_compand->str_segment[2].y;
  num_points++;

  radius = pstr_drc_compand->width_db * M_LN10_DIV_20;

  for (i = 2; i < num_points; i++)
  {
    g1 = (pstr_drc_compand->str_segment[2 * (i - 1)].y -
          pstr_drc_compand->str_segment[2 * (i - 2)].y) *
         (pstr_drc_compand->str_segment[2 * i].x - pstr_drc_compand->str_segment[2 * (i - 1)].x);
    g2 = (pstr_drc_compand->str_segment[2 * i].y - pstr_drc_compand->str_segment[2 * (i - 1)].y) *
         (pstr_drc_compand->str_segment[2 * (i - 1)].x -
          pstr_drc_compand->str_segment[2 * (i - 2)].x);

    if (fabs(g1 - g2))
    {
      continue;
    }
    num_points--;

    for (j = --i; j < num_points; j++)
    {
      pstr_drc_compand->str_segment[2 * j] = pstr_drc_compand->str_segment[2 * (j + 1)];
    }
  }

  for (i = 0; i < pstr_drc_compand->nb_segments; i += 2)
  {
    pstr_drc_compand->str_segment[i].y += pstr_drc_compand->gain_db;
    pstr_drc_compand->str_segment[i].x *= M_LN10_DIV_20;
    pstr_drc_compand->str_segment[i].y *= M_LN10_DIV_20;
  }

  for (i = 4; i < pstr_drc_compand->nb_segments; i += 2)
  {
    pstr_drc_compand->str_segment[i - 4].a = 0;
    pstr_drc_compand->str_segment[i - 4].b =
        (pstr_drc_compand->str_segment[i - 2].y - pstr_drc_compand->str_segment[i - 4].y) /
        (pstr_drc_compand->str_segment[i - 2].x - pstr_drc_compand->str_segment[i - 4].x);

    pstr_drc_compand->str_segment[i - 2].a = 0;
    pstr_drc_compand->str_segment[i - 2].b =
        (pstr_drc_compand->str_segment[i].y - pstr_drc_compand->str_segment[i - 2].y) /
        (pstr_drc_compand->str_segment[i].x - pstr_drc_compand->str_segment[i - 2].x);

    theta =
        atan2(pstr_drc_compand->str_segment[i - 2].y - pstr_drc_compand->str_segment[i - 4].y,
              pstr_drc_compand->str_segment[i - 2].x - pstr_drc_compand->str_segment[i - 4].x);
    length =
        hypot(pstr_drc_compand->str_segment[i - 2].x - pstr_drc_compand->str_segment[i - 4].x,
              pstr_drc_compand->str_segment[i - 2].y - pstr_drc_compand->str_segment[i - 4].y);

    r = MIN(radius, length);
    pstr_drc_compand->str_segment[i - 3].x =
        pstr_drc_compand->str_segment[i - 2].x - r * cos(theta);
    pstr_drc_compand->str_segment[i - 3].y =
        pstr_drc_compand->str_segment[i - 2].y - r * sin(theta);

    theta =
        atan2(pstr_drc_compand->str_segment[i].y - pstr_drc_compand->str_segment[i - 2].y,
              pstr_drc_compand->str_segment[i - 0].x - pstr_drc_compand->str_segment[i - 2].x);
    length = hypot(pstr_drc_compand->str_segment[i].x - pstr_drc_compand->str_segment[i - 2].x,
                   pstr_drc_compand->str_segment[i].y - pstr_drc_compand->str_segment[i - 2].y);

    r = MIN(radius, length / 2);
    x = pstr_drc_compand->str_segment[i - 2].x + r * cos(theta);
    y = pstr_drc_compand->str_segment[i - 2].y + r * sin(theta);

    cx =
        (pstr_drc_compand->str_segment[i - 3].x + pstr_drc_compand->str_segment[i - 2].x + x) / 3;
    cy =
        (pstr_drc_compand->str_segment[i - 3].y + pstr_drc_compand->str_segment[i - 2].y + y) / 3;

    pstr_drc_compand->str_segment[i - 2].x = x;
    pstr_drc_compand->str_segment[i - 2].y = y;

    inp_1 = cx - pstr_drc_compand->str_segment[i - 3].x;
    out_1 = cy - pstr_drc_compand->str_segment[i - 3].y;
    inp_2 = pstr_drc_compand->str_segment[i - 2].x - pstr_drc_compand->str_segment[i - 3].x;
    out_2 = pstr_drc_compand->str_segment[i - 2].y - pstr_drc_compand->str_segment[i - 3].y;
    pstr_drc_compand->str_segment[i - 3].a = (out_2 / inp_2 - out_1 / inp_1) / (inp_2 - inp_1);
    pstr_drc_compand->str_segment[i - 3].b =
        out_1 / inp_1 - pstr_drc_compand->str_segment[i - 3].a * inp_1;
  }
  pstr_drc_compand->str_segment[i - 3].x = 0;
  pstr_drc_compand->str_segment[i - 3].y = pstr_drc_compand->str_segment[i - 3].y;

  pstr_drc_compand->in_min_lin = exp(pstr_drc_compand->str_segment[1].x);
  pstr_drc_compand->out_min_lin = exp(pstr_drc_compand->str_segment[1].y);

  pstr_chan_param = &pstr_drc_compand->str_channel_param;

  if (pstr_chan_param->attack < 1.0 / pstr_drc_gain_enc->sample_rate)
  {
    pstr_chan_param->attack = 1.0;
  }
  else
  {
    pstr_chan_param->attack =
        1.0 - exp(-1.0 / (pstr_drc_gain_enc->sample_rate * pstr_chan_param->attack));
  }

  if (pstr_chan_param->decay < 1.0 / pstr_drc_gain_enc->sample_rate)
  {
    pstr_chan_param->decay = 1.0;
  }
  else
  {
    pstr_chan_param->decay =
        1.0 - exp(-1.0 / (pstr_drc_gain_enc->sample_rate * pstr_chan_param->decay));
  }
  pstr_chan_param->volume = EXP10(pstr_drc_compand->initial_volume / 20);

  return IA_NO_ERROR;
}

/**
 *  impeghe_stft_drc_compand_get_volume
 *
 *  \brief Gets volume data in frequency domain
 *
 *  \param [in] pstr_drc_stft_gain_handle  Pointer to STFT DRC gain calculation structure
 *  \param [in] in_db                      Input in decibel
 *
 *  \return FLOAT32  Volume data
 */
static FLOAT32
impeghe_stft_drc_compand_get_volume(ia_drc_stft_gain_calc_struct *pstr_drc_stft_gain_handle,
                                    FLOAT32 in_db)
{
  ULOOPIDX idx;
  FLOAT32 in_log, out_log;
  ia_drc_compand_segment_struct *pstr_compand_segment;

  if (in_db < pstr_drc_stft_gain_handle->in_min_db)
  {
    return pstr_drc_stft_gain_handle->out_min_db;
  }

  in_log = (FLOAT32)(in_db * M_LN10_DIV_20);

  for (idx = 1; idx < pstr_drc_stft_gain_handle->nb_segments; idx++)
  {
    if (in_log <= pstr_drc_stft_gain_handle->str_segment[idx].x)
    {
      break;
    }
  }

  pstr_compand_segment = &pstr_drc_stft_gain_handle->str_segment[idx - 1];
  in_log -= (FLOAT32)(pstr_compand_segment->x);
  out_log = (FLOAT32)(pstr_compand_segment->y +
                      in_log * (pstr_compand_segment->a * in_log + pstr_compand_segment->b));

  return (FLOAT32)(out_log * M_LOG10_E * 20.0f);
}

/**
 *  impeghe_stft_drc_gain_calc_process
 *
 *  \brief Calculates DRC gain value in frequency domain
 *
 *  \param [in,out] pstr_drc_gain_enc         Pointer to DRC gain encode structure
 *  \param [in] drc_coefficients_uni_drc_idx  Index of DRC coefficient
 *  \param [in] gain_set_idx                  Index of gain set
 *  \param [in] band_idx                      Index of band
 *  \param [in] start_sub_band_index          Start index of subband
 *  \param [in] stop_sub_band_index           Stop index of subband
 *  \param [in] num_frames                    Number of frames
 *  \param [in] in_buff                       Pointer to input buffer
 *  \param [in,out] gain_values               Pointer to gain buffer
 *
 *  \return VOID
 */
VOID impeghe_stft_drc_gain_calc_process(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                        WORD32 drc_coefficients_uni_drc_idx, WORD32 gain_set_idx,
                                        WORD32 band_idx, WORD32 start_sub_band_index,
                                        WORD32 stop_sub_band_index, UWORD32 num_frames,
                                        FLOAT32 *in_buff, FLOAT32 *gain_values)
{
  ULOOPIDX idx;
  LOOPIDX band;
  FLOAT32 xg, xl, yl, cdb;
  FLOAT32 in_real, in_imag;
  FLOAT32 abs_val_sqr;
  UWORD32 num_time_slot = num_frames / STFT256_HOP_SIZE;
  WORD32 start_out_idx = stop_sub_band_index - start_sub_band_index + 1;

  ia_drc_stft_gain_calc_struct *pstr_drc_stft_gain_handle =
      &pstr_drc_gain_enc
           ->str_drc_stft_gain_handle[drc_coefficients_uni_drc_idx][gain_set_idx][band_idx];

  if (start_out_idx & 1)
  {
    start_out_idx = (STFT256_HOP_SIZE - start_out_idx - 1) / 2;
  }
  else
  {
    start_out_idx = (STFT256_HOP_SIZE - start_out_idx) / 2;
  }

  for (idx = 0; idx < num_time_slot; idx++)
  {
    abs_val_sqr = 0.0f;
    for (band = start_sub_band_index; band <= stop_sub_band_index; band++)
    {
      in_imag = in_buff[((idx * STFT256_HOP_SIZE + band) << 1) + 1];
      in_real = in_buff[(idx * STFT256_HOP_SIZE + band) << 1];

      abs_val_sqr += sqrtf(powf(in_real, 2.0f) + powf(in_imag, 2.0f));
    }

    abs_val_sqr /= (FLOAT32)((stop_sub_band_index - start_sub_band_index + 1) << 4);

    abs_val_sqr = powf(abs_val_sqr, 2.0f);
    xg = 10.0f * log10f((abs_val_sqr) + 2e-13f);

    xl = -impeghe_stft_drc_compand_get_volume(pstr_drc_stft_gain_handle, xg);

    if (xl > pstr_drc_stft_gain_handle->yl_z1[band])
    {
      yl = (pstr_drc_stft_gain_handle->alpha_a * pstr_drc_stft_gain_handle->yl_z1[band]) +
           ((1.0f - pstr_drc_stft_gain_handle->alpha_a) * xl);
    }
    else
    {
      yl = (pstr_drc_stft_gain_handle->alpha_r * pstr_drc_stft_gain_handle->yl_z1[band]) +
           ((1.0f - pstr_drc_stft_gain_handle->alpha_r) * xl);
    }

    pstr_drc_stft_gain_handle->yl_z1[band] = yl;
    cdb = -yl;
    cdb = MAX(IMPEGHE_DRC_SPECTRAL_FLOOR, (powf(10.0f, cdb / 20.0f)));
    cdb = 20.0f * log10f(cdb);

    for (band = 0; band < STFT256_HOP_SIZE; band++)
    {
      gain_values[idx * STFT256_HOP_SIZE + band] = cdb;
    }
  }
}

/**
 *  impeghe_stft_drc_gain_calc_init
 *
 *  \brief Initializes DRC gain calculator in frequency domain
 *
 *  \param [in,out] pstr_drc_gain_enc         Pointer to DRC gain encode structure
 *  \param [in] drc_coefficients_uni_drc_idx  Index of DRC coefficient
 *  \param [in] gain_set_idx                  Index of gain set
 *  \param [in] band_idx                      Index of band
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_stft_drc_gain_calc_init(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                             WORD32 drc_coefficients_uni_drc_idx,
                                             WORD32 gain_set_idx, WORD32 band_idx)
{
  ULOOPIDX i, j;
  UWORD32 num_points;
  FLOAT32 width_e;
  FLOAT64 g1, g2;
  FLOAT64 x, y, cx, cy, r;
  FLOAT64 inp_1, inp_2, out_1, out_2, theta, len;
  ia_drc_compand_chan_param_struct *pstr_chan_param;
  ia_drc_stft_gain_calc_struct *pstr_drc_stft_gain_handle;

  if ((drc_coefficients_uni_drc_idx >= MAX_DRC_COEFF_COUNT) ||
      (gain_set_idx >= GAIN_SET_COUNT_MAX) || (band_idx >= MAX_BAND_COUNT))
  {
    return IMPEGHE_CONFIG_FATAL_DRC_COMPAND_FAIL;
  }

  pstr_drc_stft_gain_handle =
      &pstr_drc_gain_enc
           ->str_drc_stft_gain_handle[drc_coefficients_uni_drc_idx][gain_set_idx][band_idx];

  width_e = (FLOAT32)(pstr_drc_stft_gain_handle->width_db * M_LN10_DIV_20);

  pstr_drc_stft_gain_handle->nb_segments = (pstr_drc_stft_gain_handle->nb_points + 4) * 2;

  for (i = 0; i < pstr_drc_stft_gain_handle->nb_points; i++)
  {
    if (i && pstr_drc_stft_gain_handle->str_segment[2 * ((i - 1) + 1)].x >
                 pstr_drc_stft_gain_handle->str_segment[2 * (i + 1)].x)
    {
      return IMPEGHE_CONFIG_FATAL_DRC_COMPAND_FAIL;
    }
    pstr_drc_stft_gain_handle->str_segment[2 * (i + 1)].y -=
        pstr_drc_stft_gain_handle->str_segment[2 * (i + 1)].x;
  }
  num_points = pstr_drc_stft_gain_handle->nb_points;

  if (num_points == 0 || pstr_drc_stft_gain_handle->str_segment[2 * ((num_points - 1) + 1)].x)
  {
    num_points++;
  }

  pstr_drc_stft_gain_handle->str_segment[0].x =
      pstr_drc_stft_gain_handle->str_segment[2].x - pstr_drc_stft_gain_handle->width_db;
  pstr_drc_stft_gain_handle->str_segment[0].y = pstr_drc_stft_gain_handle->str_segment[2].y;
  num_points++;

  for (i = 2; i < num_points; i++)
  {
    g1 = (pstr_drc_stft_gain_handle->str_segment[2 * (i - 1)].y -
          pstr_drc_stft_gain_handle->str_segment[2 * (i - 2)].y) *
         (pstr_drc_stft_gain_handle->str_segment[2 * i].x -
          pstr_drc_stft_gain_handle->str_segment[2 * (i - 1)].x);
    g2 = (pstr_drc_stft_gain_handle->str_segment[2 * i].y -
          pstr_drc_stft_gain_handle->str_segment[2 * (i - 1)].y) *
         (pstr_drc_stft_gain_handle->str_segment[2 * (i - 1)].x -
          pstr_drc_stft_gain_handle->str_segment[2 * (i - 2)].x);

    if (fabs(g1 - g2))
    {
      continue;
    }
    num_points--;

    for (j = --i; j < num_points; j++)
    {
      pstr_drc_stft_gain_handle->str_segment[2 * j] =
          pstr_drc_stft_gain_handle->str_segment[2 * (j + 1)];
    }
  }

  for (i = 0; i < pstr_drc_stft_gain_handle->nb_segments; i += 2)
  {
    pstr_drc_stft_gain_handle->str_segment[i].y += pstr_drc_stft_gain_handle->gain_db;
    pstr_drc_stft_gain_handle->str_segment[i].x *= M_LN10_DIV_20;
    pstr_drc_stft_gain_handle->str_segment[i].y *= M_LN10_DIV_20;
  }

  for (i = 4; i < pstr_drc_stft_gain_handle->nb_segments; i += 2)
  {
    pstr_drc_stft_gain_handle->str_segment[i - 4].a = 0;
    pstr_drc_stft_gain_handle->str_segment[i - 4].b =
        (pstr_drc_stft_gain_handle->str_segment[i - 2].y -
         pstr_drc_stft_gain_handle->str_segment[i - 4].y) /
        (pstr_drc_stft_gain_handle->str_segment[i - 2].x -
         pstr_drc_stft_gain_handle->str_segment[i - 4].x);

    pstr_drc_stft_gain_handle->str_segment[i - 2].a = 0;
    pstr_drc_stft_gain_handle->str_segment[i - 2].b =
        (pstr_drc_stft_gain_handle->str_segment[i].y -
         pstr_drc_stft_gain_handle->str_segment[i - 2].y) /
        (pstr_drc_stft_gain_handle->str_segment[i].x -
         pstr_drc_stft_gain_handle->str_segment[i - 2].x);

    theta = atan2(pstr_drc_stft_gain_handle->str_segment[i - 2].y -
                      pstr_drc_stft_gain_handle->str_segment[i - 4].y,
                  pstr_drc_stft_gain_handle->str_segment[i - 2].x -
                      pstr_drc_stft_gain_handle->str_segment[i - 4].x);
    len = hypot(pstr_drc_stft_gain_handle->str_segment[i - 2].x -
                    pstr_drc_stft_gain_handle->str_segment[i - 4].x,
                pstr_drc_stft_gain_handle->str_segment[i - 2].y -
                    pstr_drc_stft_gain_handle->str_segment[i - 4].y);
    r = MIN(width_e / (2.0f * cos(theta)), len);
    pstr_drc_stft_gain_handle->str_segment[i - 3].x =
        pstr_drc_stft_gain_handle->str_segment[i - 2].x - r * cos(theta);
    pstr_drc_stft_gain_handle->str_segment[i - 3].y =
        pstr_drc_stft_gain_handle->str_segment[i - 2].y - r * sin(theta);

    theta = atan2(pstr_drc_stft_gain_handle->str_segment[i].y -
                      pstr_drc_stft_gain_handle->str_segment[i - 2].y,
                  pstr_drc_stft_gain_handle->str_segment[i].x -
                      pstr_drc_stft_gain_handle->str_segment[i - 2].x);
    len = hypot(pstr_drc_stft_gain_handle->str_segment[i].x -
                    pstr_drc_stft_gain_handle->str_segment[i - 2].x,
                pstr_drc_stft_gain_handle->str_segment[i].y -
                    pstr_drc_stft_gain_handle->str_segment[i - 2].y);
    r = MIN(width_e / (2.0f * cos(theta)), len / 2);
    x = pstr_drc_stft_gain_handle->str_segment[i - 2].x + r * cos(theta);
    y = pstr_drc_stft_gain_handle->str_segment[i - 2].y + r * sin(theta);

    cx = (pstr_drc_stft_gain_handle->str_segment[i - 3].x +
          pstr_drc_stft_gain_handle->str_segment[i - 2].x + x) /
         3;
    cy = (pstr_drc_stft_gain_handle->str_segment[i - 3].y +
          pstr_drc_stft_gain_handle->str_segment[i - 2].y + y) /
         3;

    pstr_drc_stft_gain_handle->str_segment[i - 2].x = x;
    pstr_drc_stft_gain_handle->str_segment[i - 2].y = y;

    inp_1 = cx - pstr_drc_stft_gain_handle->str_segment[i - 3].x;
    out_1 = cy - pstr_drc_stft_gain_handle->str_segment[i - 3].y;
    inp_2 = pstr_drc_stft_gain_handle->str_segment[i - 2].x -
            pstr_drc_stft_gain_handle->str_segment[i - 3].x;
    out_2 = pstr_drc_stft_gain_handle->str_segment[i - 2].y -
            pstr_drc_stft_gain_handle->str_segment[i - 3].y;
    pstr_drc_stft_gain_handle->str_segment[i - 3].a =
        (out_2 / inp_2 - out_1 / inp_1) / (inp_2 - inp_1);
    pstr_drc_stft_gain_handle->str_segment[i - 3].b =
        out_1 / inp_1 - pstr_drc_stft_gain_handle->str_segment[i - 3].a * inp_1;
  }
  pstr_drc_stft_gain_handle->str_segment[i - 3].x = 0;
  pstr_drc_stft_gain_handle->str_segment[i - 3].y =
      pstr_drc_stft_gain_handle->str_segment[i - 2].y;

  pstr_drc_stft_gain_handle->in_min_db =
      (FLOAT32)(pstr_drc_stft_gain_handle->str_segment[1].x * M_LOG10_E * 20.0f);
  pstr_drc_stft_gain_handle->out_min_db =
      (FLOAT32)(pstr_drc_stft_gain_handle->str_segment[1].y * M_LOG10_E * 20.0f);

  pstr_chan_param = &pstr_drc_stft_gain_handle->str_channel_param;

  pstr_chan_param->volume = EXP10(pstr_drc_stft_gain_handle->initial_volume / 20.0f);

  for (i = 0; i < STFT256_HOP_SIZE; i++)
  {
    pstr_drc_stft_gain_handle->yl_z1[i] = 0.0f;
  }

  pstr_drc_stft_gain_handle->alpha_a =
      expf(-1.0f / ((pstr_drc_stft_gain_handle->attack_ms / (FLOAT32)STFT256_HOP_SIZE) *
                    (FLOAT32)pstr_drc_gain_enc->sample_rate * 0.001f));

  pstr_drc_stft_gain_handle->alpha_r =
      expf(-1.0f / ((pstr_drc_stft_gain_handle->release_ms / (FLOAT32)STFT256_HOP_SIZE) *
                    (FLOAT32)pstr_drc_gain_enc->sample_rate * 0.001f));

  return IA_NO_ERROR;
}

/**
 *  impeghe_stft_drc_convert_to_fd
 *
 *  \brief Converts data from time domain to frequency domain
 *
 *  \param [in,out] pstr_drc_gain_enc  Pointer to DRC gain encoder structure
 *  \param [in] ptr_input              Pointer to input buffer
 *  \param [in] ch_idx                 Channel index
 *  \param [in] frame_size             Frame size
 *  \param [out] ptr_output            Pointer to output buffer
 *  \param [in] pstr_scratch           Pointer to scratch memory
 *
 *  \return VOID
 */
VOID impeghe_stft_drc_convert_to_fd(ia_drc_gain_enc_struct *pstr_drc_gain_enc, FLOAT32 *ptr_input,
                                    UWORD32 ch_idx, UWORD32 frame_size, FLOAT32 *ptr_output,
                                    VOID *pstr_scratch)
{
  ULOOPIDX i, j;
  UWORD32 num_time_slot = frame_size / STFT256_HOP_SIZE;
  FLOAT32 time_sample_vector;
  impeghe_scratch_mem *ptr_scratch = (impeghe_scratch_mem *)(pstr_scratch);
  pFLOAT32 scratch_buff = ptr_scratch->ptr_drc_scratch_buf;

  for (i = 0; i < num_time_slot; i++)
  {
    for (j = 0; j < STFT256_HOP_SIZE; j++)
    {
      time_sample_vector = (FLOAT32)(ptr_input[i * STFT256_HOP_SIZE + j] / (32768.0));

      scratch_buff[(j << 1)] =
          (FLOAT32)(pstr_drc_gain_enc->stft_tmp_in_buf_time[ch_idx][j] * impeghe_sine_win_256[j]);
      scratch_buff[(j << 1) + 1] = 0.0f;

      scratch_buff[(STFT256_HOP_SIZE + j) << 1] =
          (FLOAT32)(impeghe_sine_win_256[STFT256_HOP_SIZE - 1 - j] * time_sample_vector);
      scratch_buff[((STFT256_HOP_SIZE + j) << 1) + 1] = 0.0f;

      pstr_drc_gain_enc->stft_tmp_in_buf_time[ch_idx][j] = time_sample_vector;
    }

    impeghe_complex_fft(scratch_buff, STFT256_HOP_SIZE << 1, ptr_scratch);

    ptr_output[(i * STFT256_HOP_SIZE) << 1] = scratch_buff[0];
    ptr_output[((i * STFT256_HOP_SIZE) << 1) + 1] = scratch_buff[STFT256_HOP_SIZE << 1];

    for (j = 1; j < STFT256_HOP_SIZE; j++)
    {
      ptr_output[(i * STFT256_HOP_SIZE + j) << 1] = scratch_buff[j << 1];
      ptr_output[((i * STFT256_HOP_SIZE + j) << 1) + 1] = scratch_buff[(j << 1) + 1];
    }
  }
}
