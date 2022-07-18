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
#include "impeghe_bitbuffer.h"
#include "impeghe_cnst.h"
#include "impeghe_igf_enc.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"

#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
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
#include "impeghe_lpd.h"
#include "impeghe_lpd_rom.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_write_bits2buf
 *
 *  \brief Writes the binary representation of a 32-bit value in a buffer
 *         upto a certian number of bits.
 *
 *  \param [in ] value      Value which has to be converted to binary form.
 *  \param [in ] no_of_bits Number of bits to be extracted.
 *  \param [out] bitstream  Pointer to the buffer holding the binary data.
 *
 *  \return VOID
 */
VOID impeghe_write_bits2buf(WORD32 value, WORD32 no_of_bits, WORD16 *bitstream)
{
  LOOPIDX idx;
  WORD16 *pt_bitstream;
  pt_bitstream = bitstream + no_of_bits;
  for (idx = 0; idx < no_of_bits; idx++)
  {
    *--pt_bitstream = (WORD16)(value & MASK);
    value >>= 1;
  }
  return;
}

/**
 *  impeghe_get_num_params
 *
 *  \brief Utility function to extract number of params.
 *
 *  \param [in] qn input value.
 *
 *  \return Number of params.
 */
WORD32 impeghe_get_num_params(WORD32 *qn)
{
  return 2 + ((qn[0] > 0) ? 9 : 0) + ((qn[1] > 0) ? 9 : 0);
}

/**
 *  impeghe_cal_segsnr
 *
 *  \brief Helper function to calculate segmental SNR.
 *
 *  \param [in] sig1 Pointer to signal1.
 *  \param [in] sig2 Pointer to signal2.
 *  \param [in] len  Length of signal.
 *  \param [in] nseg Number of segments.
 *
 *  \return Segmental SNR value.
 */
FLOAT32 impeghe_cal_segsnr(FLOAT32 *sig1, FLOAT32 *sig2, WORD16 len, WORD16 nseg)
{
  LOOPIDX i, j;
  FLOAT32 signal, noise, sig_error, fac, snr;

  snr = 0.0f;
  for (i = 0; i < len; i += nseg)
  {
    noise = 1e-6f;
    signal = 1e-6f;
    for (j = 0; j < nseg; j++)
    {
      signal += (*sig1) * (*sig1);
      sig_error = *sig1++ - *sig2++;
      noise += sig_error * sig_error;
    }
    snr += (FLOAT32)log10((FLOAT64)(signal / noise));
  }
  fac = ((FLOAT32)(10 * nseg)) / (FLOAT32)len;
  snr = fac * snr;
  if (snr < -99.0f)
  {
    snr = -99.0f;
  }
  return (snr);
}

/**
 *  impeghe_highpass_50hz_12k8
 *
 *  \brief Helper function for high pass filtering (50Hz).
 *
 *  \param [in,out] signal Pointer to the input singal.
 *  \param [in]     lg     Length of the input signal.
 *  \param [in,out] mem    Pointer to filter states.
 *  \param [in]     fscale Sampling frequency of input
 *
 *  \return VOID
 */
VOID impeghe_highpass_50hz_12k8(FLOAT32 *signal, WORD32 lg, FLOAT32 *mem, WORD32 fscale)
{
  LOOPIDX idx;
  FLOAT32 x0, x1, x2, y0, y1, y2;
  const FLOAT32 *a = NULL, *b = NULL;
  y1 = mem[0];
  y2 = mem[1];
  x0 = mem[2];
  x1 = mem[3];
  switch (fscale)
  {
  case 14700:
    a = &impeghe_hp20_filter_coeffs[0][0];
    b = &impeghe_hp20_filter_coeffs[0][2];
    break;
  case 16000:
    a = &impeghe_hp20_filter_coeffs[1][0];
    b = &impeghe_hp20_filter_coeffs[1][2];
    break;
  case 22050:
    a = &impeghe_hp20_filter_coeffs[2][0];
    b = &impeghe_hp20_filter_coeffs[2][2];
    break;
  case 24000:
    a = &impeghe_hp20_filter_coeffs[3][0];
    b = &impeghe_hp20_filter_coeffs[3][2];
    break;
  case 29400:
    a = &impeghe_hp20_filter_coeffs[4][0];
    b = &impeghe_hp20_filter_coeffs[4][2];
    break;
  default:
    a = &impeghe_hp20_filter_coeffs[5][0];
    b = &impeghe_hp20_filter_coeffs[5][2];
    break;
  }
  for (idx = 0; idx < lg; idx++)
  {
    x2 = x1;
    x1 = x0;
    x0 = signal[idx];
    y0 = (y1 * a[0]) + (y2 * a[1]) + (x0 * b[1]) + (x1 * b[0]) + (x2 * b[1]);
    signal[idx] = y0;
    y2 = y1;
    y1 = y0;
  }

  mem[3] = ((x1 > 1e-10) | (x1 < -1e-10)) ? x1 : 0;
  mem[2] = ((x0 > 1e-10) | (x0 < -1e-10)) ? x0 : 0;
  mem[1] = ((y2 > 1e-10) | (y2 < -1e-10)) ? y2 : 0;
  mem[0] = ((y1 > 1e-10) | (y1 < -1e-10)) ? y1 : 0;
}

/**
 *  impeghe_apply_preemph
 *
 *  \brief Pre-emphasis filtering
 *
 *  \param [in,out] signal Pointer to input signal.
 *  \param [in]     factor Pre-emphasis filter factor.
 *  \param [in]     length Length of input signal.
 *  \param [in,out] mem    Pointer to filter states.
 *
 *  \return VOID
 */
VOID impeghe_apply_preemph(FLOAT32 *signal, FLOAT32 factor, WORD32 length, FLOAT32 *mem)
{
  LOOPIDX idx;
  FLOAT32 val;
  val = signal[length - 1];
  for (idx = length - 1; idx > 0; idx--)
  {
    signal[idx] = signal[idx] - factor * signal[idx - 1];
  }
  signal[0] -= factor * (*mem);
  *mem = val;
}

/**
 *  impeghe_apply_deemph
 *
 *  \brief De-emphasis filtering
 *
 *  \param [in,out] signal Pointer to input signal.
 *  \param [in]     factor De-emphasis filter factor.
 *  \param [in]     length Length of input signal.
 *  \param [in,out] mem    Pointer to filter states.
 *
 *  \return WORD32
 */
VOID impeghe_apply_deemph(FLOAT32 *signal, FLOAT32 factor, WORD32 length, FLOAT32 *mem)
{
  LOOPIDX idx;
  signal[0] = signal[0] + factor * (*mem);
  for (idx = 1; idx < length; idx++)
  {
    signal[idx] = signal[idx] + factor * signal[idx - 1];
  }
  *mem = signal[length - 1];
  if ((*mem < 1e-10) & (*mem > -1e-10))
  {
    *mem = 0;
  }
}

/**
 *  impeghe_synthesis_tool_float
 *
 *  \brief Calculates residual error signal derived from original and synthesized.
 *
 *  \param [in ]    a                 Coefficients of synthesis filter
 *  \param [in ]    x                 Pointer to input signal.
 *  \param [out]    y                 Pointer to output signal (residue).
 *  \param [in ]    l                 Length of signal.
 *  \param [in,out] mem               Pointer to synthesis filter states.
 *  \param [in ]   scratch_synth_tool Pointer to scratch buffer.
 *
 *  \return VOID
 */
VOID impeghe_synthesis_tool_float(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l, FLOAT32 *mem,
                                  FLOAT32 *scratch_synth_tool)
{
  LOOPIDX i, j;
  FLOAT32 s;
  FLOAT32 *yy;

  memcpy(scratch_synth_tool, mem, ORDER * sizeof(FLOAT32));
  yy = &scratch_synth_tool[ORDER];
  for (i = 0; i < l; i++)
  {
    s = x[i];
    for (j = 1; j <= ORDER; j += 4)
    {
      s -= a[j] * yy[i - j];
      s -= a[j + 1] * yy[i - (j + 1)];
      s -= a[j + 2] * yy[i - (j + 2)];
      s -= a[j + 3] * yy[i - (j + 3)];
    }
    y[i] = s;
    yy[i] = s;
  }
}

/**
 *  impeghe_compute_lp_residual
 *
 *  \brief Calculates linear prediction residual signal
 *
 *  \param [in] a Pointer to Linear Prediction coefficients.
 *  \param [in] x Pointer to input signal.
 *  \param [in] y Pointer to output signal residual.
 *  \param [in] l Length of signal.
 *
 *  \return VOID
 */
VOID impeghe_compute_lp_residual(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l)
{
  LOOPIDX idx;
  FLOAT32 s;
  for (idx = 0; idx < l; idx++)
  {
    s = x[idx];
    s += a[1] * x[idx - 1];
    s += a[2] * x[idx - 2];
    s += a[3] * x[idx - 3];
    s += a[4] * x[idx - 4];
    s += a[5] * x[idx - 5];
    s += a[6] * x[idx - 6];
    s += a[7] * x[idx - 7];
    s += a[8] * x[idx - 8];
    s += a[9] * x[idx - 9];
    s += a[10] * x[idx - 10];
    s += a[11] * x[idx - 11];
    s += a[12] * x[idx - 12];
    s += a[13] * x[idx - 13];
    s += a[14] * x[idx - 14];
    s += a[15] * x[idx - 15];
    s += a[16] * x[idx - 16];
    y[idx] = s;
  }
}

/**
 *  impeghe_convolve
 *
 *  \brief Helper function that carries our convolution.
 *
 *  \param [in ] signal           Pointer to input signal.
 *  \param [in ] wsynth_filter_ir Pointer to impulse response of filter.
 *  \param [out] conv_out         Convolution output.
 *
 *  \return VOID
 */
VOID impeghe_convolve(FLOAT32 *signal, FLOAT32 *wsynth_filter_ir, FLOAT32 *conv_out)
{
  LOOPIDX i, n;
  FLOAT32 val;

  for (n = 0; n < LEN_SUBFR; n += 2)
  {
    val = 0.0f;
    for (i = 0; i <= n; i++)
    {
      val += signal[i] * wsynth_filter_ir[n - i];
    }
    conv_out[n] = val;
    val = 0.0f;
    for (i = 0; i <= (n + 1); i += 2)
    {
      val += signal[i] * wsynth_filter_ir[(n + 1) - i];
      val += signal[i + 1] * wsynth_filter_ir[n - i];
    }
    conv_out[n + 1] = val;
  }
}

/**
 *  impeghe_autocorr_plus
 *
 *  \brief Helper function that carries out windowing and autocorrelation.
 *
 *  \param [in ] speech           Pointer to input speech signal.
 *  \param [out] auto_corr_vector Pointer to auto-correlation result.
 *  \param [in ] window_len       Window length.
 *  \param [in ] lp_analysis_win  Linear prediction analysis window.
 *  \param [in ] temp_aut_corr    Scratch buffer used by the function.
 *
 *  \return VOID
 */
VOID impeghe_autocorr_plus(FLOAT32 *speech, FLOAT32 *auto_corr_vector, WORD32 window_len,
                           const FLOAT32 *lp_analysis_win, FLOAT32 *temp_aut_corr)
{
  LOOPIDX i, j;
  FLOAT32 val;

  for (i = 0; i < window_len; i++)
  {
    temp_aut_corr[i] = speech[i] * lp_analysis_win[i];
  }
  for (i = 0; i <= ORDER; i++)
  {
    val = 0.0f;
    for (j = 0; j < window_len - i; j++)
    {
      val += temp_aut_corr[j] * temp_aut_corr[j + i];
    }
    auto_corr_vector[i] = val;
  }
  if (auto_corr_vector[0] < 1.0)
  {
    auto_corr_vector[0] = 1.0;
  }
}

/**
 *  impeghe_get_norm_correlation
 *
 *  \brief Computes normalized correlation value of a signal.
 *         Normalization factor is related to signal energy.
 *
 *  \param [in ] exc          Pointer to input excitation signal.
 *  \param [in ] xn           Pointer to input signal.
 *  \param [in ] wsyn_filt_ir Pointer to synthesis filter impulse response.
 *  \param [in ] min_interval Minimum limit of the interval for correlation.
 *  \param [in ] max_interval Maximum limit of the interval for correlation.
 *  \param [out] norm_corr    Pointer to normalized correlation buffer.
 *
 *  \return VOID
 */
static VOID impeghe_get_norm_correlation(FLOAT32 *exc, FLOAT32 *xn, FLOAT32 *wsyn_filt_ir,
                                         WORD32 min_interval, WORD32 max_interval,
                                         FLOAT32 *norm_corr)
{
  LOOPIDX i, j;
  WORD32 min;
  FLOAT32 energy_filt_exc, corr, norm;
  FLOAT32 filt_prev_exc[LEN_SUBFR];
  min = -min_interval;

  impeghe_convolve(&exc[min], wsyn_filt_ir, filt_prev_exc);

  for (i = min_interval; i <= max_interval; i++)
  {
    corr = 0.0F;
    energy_filt_exc = 0.01F;
    for (j = 0; j < LEN_SUBFR; j++)
    {
      corr += xn[j] * filt_prev_exc[j];
      energy_filt_exc += filt_prev_exc[j] * filt_prev_exc[j];
    }

    norm = (FLOAT32)(1.0f / sqrt(energy_filt_exc));
    norm_corr[i - min_interval] = corr * norm;

    if (i != max_interval)
    {
      min--;
      for (j = LEN_SUBFR - 1; j > 0; j--)
      {
        filt_prev_exc[j] = filt_prev_exc[j - 1] + exc[min] * wsyn_filt_ir[j];
      }
      filt_prev_exc[0] = exc[min];
    }
  }
}

/**
 *  impeghe_corr_interpolate
 *
 *  \brief Carries interpolation on correlation data.
 *
 *  \param [in] x        Pointer to input data.
 *  \param [in] fraction Linear interpolation fraction value.
 *
 *  \return Interpolated output value.
 */
static FLOAT32 impeghe_corr_interpolate(FLOAT32 *x, WORD32 fraction)
{
  FLOAT32 interpol_value;
  FLOAT32 *x1, *x2;
  const FLOAT32 *p1_interp4_1_table, *p2_interp4_1_table;

  if (fraction < 0)
  {
    x--;
    fraction += 4;
  }
  p1_interp4_1_table = &impeghe_interp4_1[fraction];
  p2_interp4_1_table = &impeghe_interp4_1[4 - fraction];
  x1 = &x[0];
  x2 = &x[1];
  interpol_value = x1[0] * p1_interp4_1_table[0] + x2[0] * p2_interp4_1_table[0];
  interpol_value += x1[-1] * p1_interp4_1_table[4] + x2[1] * p2_interp4_1_table[4];
  interpol_value += x1[-2] * p1_interp4_1_table[8] + x2[2] * p2_interp4_1_table[8];
  interpol_value += x1[-3] * p1_interp4_1_table[12] + x2[3] * p2_interp4_1_table[12];

  return interpol_value;
}

/**
 *  impeghe_open_loop_search
 *
 *  \brief Open loop pitch search function.
 *
 *  \param [in ] wsp           Pointer to weighted speech buffer.
 *  \param [in ] min_pitch_lag Minimum pitch lag value.
 *  \param [in ] max_pitch_lag Maximum pitch lag value.
 *  \param [in ] num_frame     Number of frames.
 *  \param [out] ol_pitch_lag  Pointer to open loop pitch lag values.
 *  \param [in ] pstr_td_encoder            Pointer to USAC LPD state structure.
 *
 *  \return VOID
 */
VOID impeghe_open_loop_search(FLOAT32 *wsp, WORD32 min_pitch_lag, WORD32 max_pitch_lag,
                              WORD32 num_frame, WORD32 *ol_pitch_lag,
                              ia_usac_td_encoder_struct *pstr_td_encoder)
{
  LOOPIDX i, j;
  FLOAT32 r, corr, energy1, energy2, corr_max;
  FLOAT32 *data_a, *data_b, *hp_wsp, *p, *p1;
  const FLOAT32 *p1_ol_cw_table, *p2_ol_cw_table;

  corr_max = -1.0e23f;
  *ol_pitch_lag = 0;
  p1_ol_cw_table = &impeghe_ol_corr_weight[453];
  p2_ol_cw_table = &impeghe_ol_corr_weight[259 + max_pitch_lag - pstr_td_encoder->prev_pitch_med];
  for (i = max_pitch_lag; i > min_pitch_lag; i--)
  {
    corr = 0.0;
    p = &wsp[0];
    p1 = &wsp[-i];
    for (j = 0; j < num_frame; j += 2)
    {
      corr += p[j] * p1[j];
      corr += p[j + 1] * p1[j + 1];
    }
    corr *= *p1_ol_cw_table--;
    if ((pstr_td_encoder->prev_pitch_med > 0) && (pstr_td_encoder->ol_wght_flg == 1))
    {
      corr *= *p2_ol_cw_table--;
    }
    if (corr >= corr_max)
    {
      corr_max = corr;
      *ol_pitch_lag = i;
    }
  }
  hp_wsp = pstr_td_encoder->prev_hp_wsp + max_pitch_lag;
  data_a = pstr_td_encoder->hp_ol_ltp_mem;
  data_b = pstr_td_encoder->hp_ol_ltp_mem + HP_ORDER;
  for (i = 0; i < num_frame; i++)
  {
    data_b[0] = data_b[1];
    data_b[1] = data_b[2];
    data_b[2] = data_b[3];
    data_b[HP_ORDER] = wsp[i];
    r = data_b[0] * 0.83787057505665F;
    r += data_b[1] * -2.50975570071058F;
    r += data_b[2] * 2.50975570071058F;
    r += data_b[3] * -0.83787057505665F;
    r -= data_a[0] * -2.64436711600664F;
    r -= data_a[1] * 2.35087386625360F;
    r -= data_a[2] * -0.70001156927424F;
    data_a[2] = data_a[1];
    data_a[1] = data_a[0];
    data_a[0] = r;
    hp_wsp[i] = r;
  }
  p = &hp_wsp[0];
  p1 = &hp_wsp[-(*ol_pitch_lag)];
  corr = 0.0F;
  energy1 = 0.0F;
  energy2 = 0.0F;
  for (i = 0; i < num_frame; i++)
  {
    corr += p[i] * p1[i];
    energy1 += p1[i] * p1[i];
    energy2 += p[i] * p[i];
  }
  pstr_td_encoder->ol_gain = (FLOAT32)(corr / (sqrt(energy1 * energy2) + 1e-5));
  memmove(pstr_td_encoder->prev_hp_wsp, &pstr_td_encoder->prev_hp_wsp[num_frame],
          max_pitch_lag * sizeof(FLOAT32));
}

/**
 *  impeghe_get_ol_lag_median
 *
 *  \brief Calclutates median of open loop(OL) pitch lag values
 *
 *  \param [in] prev_ol_lag  Most recent OL pitch lag value.
 *  \param [in] prev_ol_lags Pointer to buffer of past OL pitch lag values.
 *
 *  \return Median of the open loop pitch lags.
 */
WORD32 impeghe_get_ol_lag_median(WORD32 prev_ol_lag, WORD32 *prev_ol_lags)
{
  LOOPIDX i, j;
  WORD32 idx, val, num_lags;
  WORD32 sorted_ol_lags_out[NUM_OPEN_LOOP_LAGS + 1] = {0};

  num_lags = NUM_OPEN_LOOP_LAGS;
  idx = (NUM_OPEN_LOOP_LAGS >> 1) + 1;
  for (i = NUM_OPEN_LOOP_LAGS - 1; i > 0; i--)
  {
    prev_ol_lags[i] = prev_ol_lags[i - 1];
  }
  prev_ol_lags[0] = prev_ol_lag;
  for (i = 0; i < NUM_OPEN_LOOP_LAGS; i++)
  {
    sorted_ol_lags_out[i + 1] = prev_ol_lags[i];
  }
  for (;;)
  {
    if (idx <= 1)
    {
      val = sorted_ol_lags_out[num_lags];
      sorted_ol_lags_out[num_lags] = sorted_ol_lags_out[1];
      if (--num_lags == 1)
      {
        sorted_ol_lags_out[1] = val;
        break;
      }
    }
    else
    {
      val = sorted_ol_lags_out[--idx];
    }
    i = idx;
    j = idx << 1;
    while (j <= num_lags)
    {
      if (j < num_lags && sorted_ol_lags_out[j] < sorted_ol_lags_out[j + 1])
      {
        ++j;
      }
      if (val >= sorted_ol_lags_out[j])
      {
        j = num_lags + 1;
      }
      else
      {
        sorted_ol_lags_out[i] = sorted_ol_lags_out[j];
        i = j;
        j *= 2;
      }
    }
    sorted_ol_lags_out[i] = val;
  }

  return sorted_ol_lags_out[OPEN_LOOP_LAG_MEDIAN];
}

/**
 *  impeghe_closed_loop_search
 *
 *  \brief Carries out closed loop pitch search
 *
 *  \param [in ] exc                  Pointer to excitation signal.
 *  \param [in ] xn                   Pointer to input signal.
 *  \param [in ] wsyn_filt_ir         Pointer to synthesis filter impulse resp.
 *  \param [in ] search_range_min     Minimum value of search range.
 *  \param [in ] search_range_max     Maximum value of search range.
 *  \param [in ] pit_frac             Fractional pitch value.
 *  \param [in ] is_first_subfrm      Flag indicating first sub frame.
 *  \param [in ] min_pitch_lag_res1_2 Minimum pitch lag resolution value 2.
 *  \param [in ] min_pitch_lag_res_1  Minimum pitch lag resolution value 1.
 *  \param [out] pitch_lag_out        Pointer to output pitch lag buffer.
 *
 *  \return VOID
 */
VOID impeghe_closed_loop_search(FLOAT32 *exc, FLOAT32 *xn, FLOAT32 *wsyn_filt_ir,
                                WORD32 search_range_min, WORD32 search_range_max,
                                WORD32 *pit_frac, WORD32 is_first_subfrm,
                                WORD32 min_pitch_lag_res1_2, WORD32 min_pitch_lag_res_1,
                                WORD32 *pitch_lag_out)
{
  LOOPIDX idx;
  WORD32 fraction, step;
  WORD32 min_interval, max_interval;
  FLOAT32 corr_max, temp;
  FLOAT32 corr_vector[15 + 2 * LEN_INTERPOL1 + 1];
  FLOAT32 *p_norm_corr_vector;

  min_interval = search_range_min - LEN_INTERPOL1;
  max_interval = search_range_max + LEN_INTERPOL1;
  p_norm_corr_vector = &corr_vector[0];
  impeghe_get_norm_correlation(exc, xn, wsyn_filt_ir, min_interval, max_interval,
                               p_norm_corr_vector);

  corr_max = p_norm_corr_vector[LEN_INTERPOL1];
  *pitch_lag_out = search_range_min;
  for (idx = search_range_min + 1; idx <= search_range_max; idx++)
  {
    if (p_norm_corr_vector[idx - search_range_min + LEN_INTERPOL1] > corr_max)
    {
      *pitch_lag_out = idx;
      corr_max = p_norm_corr_vector[idx - search_range_min + LEN_INTERPOL1];
    }
  }
  if (!((is_first_subfrm == 0) && (*pitch_lag_out >= min_pitch_lag_res_1)))
  {
    step = 1;
    fraction = -3;
    if (((is_first_subfrm == 0) && (*pitch_lag_out >= min_pitch_lag_res1_2)) ||
        (min_pitch_lag_res1_2 == TMIN))
    {
      step = 2;
      fraction = -2;
    }
    if (*pitch_lag_out == search_range_min)
    {
      fraction = 0;
    }
    corr_max = impeghe_corr_interpolate(
        &p_norm_corr_vector[(*pitch_lag_out) - search_range_min + LEN_INTERPOL1], fraction);
    for (idx = (fraction + step); idx <= 3; idx += step)
    {
      ;
      temp = impeghe_corr_interpolate(
          &p_norm_corr_vector[(*pitch_lag_out) - search_range_min + LEN_INTERPOL1], idx);
      if (temp > corr_max)
      {
        corr_max = temp;
        fraction = idx;
      }
    }
    if (fraction < 0)
    {
      fraction += 4;
      (*pitch_lag_out) -= 1;
    }
    *pit_frac = fraction;
  }
  else
  {
    *pit_frac = 0;
  }
}

/**
 *  impeghe_decim2_fir_filter
 *
 *  \brief Carries out signal decimation by 2.
 *
 *  \param [in,out] signal          Pointer to input signal.
 *  \param [in] length              Length of the input signal.
 *  \param [in] mem                 Pointer to decimation filter states.
 *  \param [in] scratch_fir_sig_buf Pointer to scratch buffer.
 *
 *  \return VOID
 */
VOID impeghe_decim2_fir_filter(FLOAT32 *signal, WORD32 length, FLOAT32 *mem,
                               FLOAT32 *scratch_fir_sig_buf)
{
  LOOPIDX i, j;
  FLOAT32 temp;
  FLOAT32 *sig_buf = scratch_fir_sig_buf;

  memcpy(sig_buf, mem, DECIM2_FIR_FILT_MEM_SIZE * sizeof(FLOAT32));
  memcpy(&sig_buf[DECIM2_FIR_FILT_MEM_SIZE], signal, length * sizeof(FLOAT32));
  for (i = 0; i < DECIM2_FIR_FILT_MEM_SIZE; i++)
  {
    mem[i] = ((signal[length - DECIM2_FIR_FILT_MEM_SIZE + i] > 1e-10) ||
              (signal[length - DECIM2_FIR_FILT_MEM_SIZE + i] < -1e-10))
                 ? signal[length - DECIM2_FIR_FILT_MEM_SIZE + i]
                 : 0;
  }
  for (i = 0, j = 0; i < length; i += 2, j++)
  {
    temp = sig_buf[i + 4] * 0.13F;
    temp += sig_buf[i + 3] * 0.23F;
    temp += sig_buf[i + 2] * 0.28F;
    temp += sig_buf[i + 1] * 0.23F;
    temp += sig_buf[i] * 0.13F;
    signal[j] = temp;
  }
}

/**
 *  impeghe_calc_sq_gain
 *
 *  \brief Calculates gain value based on signal energy values.
 *
 *  \param [in] x                  Pointer to input signal.
 *  \param [in] num_bits           Number of bits values.
 *  \param [in] length             Processing length of input signal.
 *  \param [in] scratch_sq_gain_en Pointer to scratch buffer.
 *
 *  \return Gain value
 */
FLOAT32 impeghe_calc_sq_gain(FLOAT32 *x, WORD32 num_bits, WORD32 length,
                             FLOAT32 *scratch_sq_gain_en)
{
  LOOPIDX i, j;
  FLOAT32 gain, ener, temp, target, factor, offset;
  FLOAT32 *en = scratch_sq_gain_en;

  for (i = 0; i < length; i += 4)
  {
    ener = 0.01f;
    for (j = i; j < i + 4; j++)
    {
      ener += x[j] * x[j];
    }

    temp = (FLOAT32)log10(ener);
    en[i / 4] = 9.0f + 10.0f * temp;
  }

  target = (6.0f / 4.0f) * (FLOAT32)(num_bits - (length / 16));

  factor = 128.0f;
  offset = factor;

  for (i = 0; i < 10; i++)
  {
    factor *= 0.5f;
    offset -= factor;
    ener = 0.0f;
    for (j = 0; j < length / 4; j++)
    {
      temp = en[j] - offset;

      if (temp > 3.0f)
      {
        ener += temp;
      }
    }
    if (ener > target)
    {
      offset += factor;
    }
  }

  gain = (FLOAT32)pow(10.0f, offset / 20.0f);

  return (gain);
}

/**
 *  impeghe_lpc_coef_gen
 *
 *  \brief Generates LPC coefficients from LSP values.
 *
 *  \param [in ] lsf_old  Pointer to previous LSF values.
 *  \param [in ] lsf_new  Pointer to new LSF values.
 *  \param [out] a        Pointer to Linear prediction coefficients.
 *  \param [in ] nb_subfr Number of subframes.
 *  \param [in ] m        LPC order.
 *
 *  \return VOID
 */
VOID impeghe_lpc_coef_gen(FLOAT32 *lsf_old, FLOAT32 *lsf_new, FLOAT32 *a, WORD32 nb_subfr,
                          WORD32 m)
{
  LOOPIDX idx = 0;
  FLOAT32 inc, fnew, fold;
  FLOAT32 lsf[ORDER] = {0};
  FLOAT32 *ptr_a;

  inc = 1.0f / (FLOAT32)nb_subfr;
  fnew = 0.5f - (0.5f * inc);
  fold = 1.0f - fnew;
  ptr_a = a;
  for (idx = 0; idx < m; idx++)
  {
    lsf[idx] = (lsf_old[idx] * fold) + (lsf_new[idx] * fnew);
  }
  impeghe_lsp_to_lp_conversion(lsf, ptr_a);
  ptr_a += (m + 1);
  impeghe_lsp_to_lp_conversion(lsf_old, ptr_a);
  ptr_a += (m + 1);
  impeghe_lsp_to_lp_conversion(lsf_new, ptr_a);
  ptr_a += (m + 1);

  return;
}

/**
 *  impeghe_interpolation_lsp_params
 *
 *  \brief Interpolates LPC coefficients for all subframs from lsp data.
 *
 *  \param [in ] lsp_old        Pointer to previous LSP values.
 *  \param [in ] lsp_new        Pointer to new LSP values.
 *  \param [out] lp_flt_coff_a  Pointer to linear prediction coefficients.
 *  \param [in ] nb_subfr       Number of subframes.
 *
 *  \return VOID
 */
VOID impeghe_interpolation_lsp_params(FLOAT32 *lsp_old, FLOAT32 *lsp_new, FLOAT32 *lp_flt_coff_a,
                                      WORD32 nb_subfr)
{
  LOOPIDX i, j;
  FLOAT32 factor;
  FLOAT32 lsp[ORDER];
  FLOAT32 x_plus_y, x_minus_y;
  factor = 1.0f / (FLOAT32)nb_subfr;

  x_plus_y = 0.5f * factor;

  for (i = 0; i < nb_subfr; i++)
  {
    x_minus_y = 1.0f - x_plus_y;
    for (j = 0; j < ORDER; j++)
    {
      lsp[j] = (lsp_old[j] * x_minus_y) + (lsp_new[j] * x_plus_y);
    }
    x_plus_y += factor;

    impeghe_lsp_to_lp_conversion(lsp, lp_flt_coff_a);

    lp_flt_coff_a += (ORDER + 1);
  }

  impeghe_lsp_to_lp_conversion(lsp_new, lp_flt_coff_a);

  return;
}
/** @} */ /* End of CoreEncProc */
