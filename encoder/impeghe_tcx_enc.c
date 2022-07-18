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
#include "impeghe_fft.h"
#include "impeghe_tcx_mdct.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_enc_mct.h"
#include "impeghe_resampler.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_main.h"

#include "impeghe_lpd_rom.h"
#include "impeghe_lpd.h"
#include "impeghe_avq_enc.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

#define ia_mul_flt(a, b) (a * b)
#define ia_mac_flt(x, a, b) (x + a * b)
#define ia_min_int(a, b) (min(a, b))
#define ERROR_THRESHOLD                                                                          \
  (0.03f) // threshold to determine if fdp processed values should be considered or ignored
//(For certain files where monotonous frequencies are present, some differences can be made out
// between fdp and no-fdp)

const UWORD32 impeghe_pow10_gain_div28[128] = {
    1024,     1112,     1207,     1311,     1423,     1545,     1677,     1821,     1977,
    2146,     2330,     2530,     2747,     2983,     3238,     3516,     3817,     4144,
    4499,     4885,     5304,     5758,     6252,     6788,     7370,     8001,     8687,
    9432,     10240,    11118,    12071,    13105,    14228,    15448,    16772,    18210,
    19770,    21465,    23305,    25302,    27471,    29825,    32382,    35157,    38171,
    41442,    44994,    48851,    53038,    57584,    62519,    67878,    73696,    80012,
    86870,    94316,    102400,   111177,   120706,   131052,   142284,   154480,   167720,
    182096,   197703,   214649,   233047,   253021,   274708,   298254,   323817,   351572,
    381706,   414422,   449943,   488508,   530378,   575838,   625193,   678779,   736958,
    800124,   868703,   943161,   1024000,  1111768,  1207059,  1310517,  1422843,  1544797,
    1677203,  1820958,  1977034,  2146488,  2330466,  2530213,  2747080,  2982536,  3238172,
    3515720,  3817056,  4144220,  4499426,  4885077,  5303782,  5758375,  6251932,  6787792,
    7369581,  8001236,  8687031,  9431606,  10240000, 11117682, 12070591, 13105175, 14228434,
    15447969, 16772032, 18209581, 19770345, 21464883, 23304662, 25302131, 27470805, 29825358,
    32381723, 35157197};

static const FLOAT64 impeghe_lpc_pre_twid_cos[ORDER + 1] = {1.0,
                                                            0.99969881867944277,
                                                            0.99879545613814691,
                                                            0.99729045666498317,
                                                            0.99518472640441780,
                                                            0.99247953486470630,
                                                            0.98917650991010153,
                                                            0.98527764316379052,
                                                            0.98078527933727178,
                                                            0.97570212991605565,
                                                            0.97003125425052761,
                                                            0.96377606826277584,
                                                            0.95694033551585822,
                                                            0.94952817722361749,
                                                            0.94154406823678738,
                                                            0.93299279849944938,
                                                            0.92387952832938047};

static const FLOAT64 impeghe_lpc_pre_twid_sin[ORDER + 1] = {0,
                                                            0.024541229205697054,
                                                            0.049067675691753569,
                                                            0.073564563785488826,
                                                            0.098017143048367339,
                                                            0.12241067304257494,
                                                            0.14673047482398086,
                                                            0.17096188429473480,
                                                            0.19509032737506427,
                                                            0.21910124070226658,
                                                            0.24298017568754085,
                                                            0.26671274855909161,
                                                            0.29028467796767482,
                                                            0.31368175059826858,
                                                            0.33688984485751389,
                                                            0.35989503740419343,
                                                            0.38268344246110436};

/**
 *  impeghe_lpc_mdct
 *
 *  \brief Carries out MDCT on the linear prediction coefficients.
 *
 *  \param [in ] ptr_lpc_coeffs Pointer to LPC coefficients.
 *  \param [out] ptr_mdct_gains     Pointer to MDCT gains.
 *  \param [in ] length         Length of the input data.
 *  \param [in ] pstr_scratch   Pointer to scratch structure.
 *
 *  \return VOID
 */
static VOID impeghe_lpc_mdct(FLOAT32 *ptr_lpc_coeffs, FLOAT32 *ptr_mdct_gains, WORD32 length,
                             impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *in_out = pstr_scratch->p_in_out_tcx;
  WORD32 i;

  for (i = 0; i < ORDER + 1; i++)
  {
    in_out[2 * i] = (FLOAT32)(ptr_lpc_coeffs[i] * impeghe_lpc_pre_twid_cos[i]);
    in_out[2 * i + 1] = (FLOAT32)(-ptr_lpc_coeffs[i] * impeghe_lpc_pre_twid_sin[i]);
  }
  for (; i < length; i++)
  {
    in_out[2 * i] = 0.f;
    in_out[2 * i + 1] = 0.f;
  }

  impeghe_complex_fft(in_out, length, pstr_scratch);

  for (i = 0; i<length>> 1; i++)
  {
    ptr_mdct_gains[i] = (FLOAT32)(
        1.0f / sqrt(in_out[2 * i] * in_out[2 * i] + in_out[2 * i + 1] * in_out[2 * i + 1]));
  }

  return;
}

/**
 *  impeghe_rounded_sqrt
 *
 *  \brief Calculates rounded sqrt and returns the value
 *
 *  \param [in]	pos_num	Positive number
 *
 *  \return Square root value
 *
 */
UWORD32 impeghe_rounded_sqrt(UWORD32 pos_num)
{
  UWORD32 num = pos_num;
  UWORD32 value = 0;
  UWORD32 bit_set = 1 << 30;

  while (bit_set > num)
  {
    bit_set >>= 2;
  }
  while (bit_set)
  {
    if (num >= value + bit_set)
    {
      num -= value + bit_set;
      value += bit_set << 1;
    }
    value >>= 1;
    bit_set >>= 2;
  }
  num = value + 1;
  if (num * num - pos_num < pos_num - value * value)
  {
    return num;
  }
  return value;
}

/**
 *  impeghe_noise_shaping
 *
 *  \brief Implements noise shaping in the TCX path.
 *
 *  \param [in,out] ptr_rr    Pointer to input buffer.
 *  \param [out]    lg    Length of input buffer.
 *  \param [in ]    ptr_gain1 Pointer to gain buffer1.
 *  \param [in ]    ptr_gain2 Pointer to gain buffer2.
 *
 *  \return VOID
 */
static VOID impeghe_noise_shaping(FLOAT32 *ptr_rr, WORD32 lg, FLOAT32 *ptr_gain1,
                                  FLOAT32 *ptr_gain2)
{
  WORD32 i, k;
  FLOAT32 r, r_prev, g1, g2, a = 0, b = 0;

  k = lg / FDNS_RESOLUTION;

  r_prev = 0;
  for (i = 0; i < lg; i++)
  {
    if ((i % k) == 0)
    {
      g1 = ptr_gain1[i / k];
      g2 = ptr_gain2[i / k];

      a = 2.0f * g1 * g2 / (g1 + g2);
      b = (g2 - g1) / (g1 + g2);
    }

    r = a * ptr_rr[i] + b * r_prev;

    ptr_rr[i] = r;
    r_prev = r;
  }

  return;
}

/**
 *  impeghe_pre_shaping
 *
 *  \brief Preprocessing before apply noise shaping
 *
 *  \param [in,out] ptr_rr    Pointer to input buffer.
 *  \param [out]    lg    Length of input buffer.
 *  \param [in ]    ptr_gain1 Pointer to gain buffer1.
 *  \param [in ]    ptr_gain2 Pointer to gain buffer2.
 *
 *  \return VOID
 */
static VOID impeghe_pre_shaping(FLOAT32 *ptr_rr, WORD32 lg, FLOAT32 *ptr_gain1,
                                FLOAT32 *ptr_gain2)
{
  WORD32 i, k;
  FLOAT32 r, r_prev, g1, g2, a = 0, b = 0;

  k = lg / FDNS_RESOLUTION;

  r_prev = 0;
  for (i = 0; i < lg; i++)
  {
    if ((i % k) == 0)
    {
      g1 = ptr_gain1[i / k];
      g2 = ptr_gain2[i / k];

      a = (g1 + g2) / (2.0f * g1 * g2);
      b = (g1 - g2) / (2.0f * g1 * g2);
    }

    r = a * ptr_rr[i] + b * r_prev;

    r_prev = ptr_rr[i];
    ptr_rr[i] = r;
  }

  return;
}

/**
 *  impeghe_adapt_lo_freq_emph
 *
 *  \brief Carries out adaptive low frequency emphasis.
 *
 *  \param [in,out] signal Pointer to input signal.
 *  \param [in]     length Input data length.
 *
 *  \return VOID
 */
static VOID impeghe_adapt_lo_freq_emph(FLOAT32 *ptr_signal, WORD32 length)
{
  WORD32 i, j, i_max;
  FLOAT32 max_energy, factor, temp;

  i_max = length >> 2;

  max_energy = 0.01f;
  for (i = 0; i < i_max; i += 8)
  {
    temp = 0.01f;
    for (j = i; j < i + 8; j++)
    {
      temp += ptr_signal[j] * ptr_signal[j];
    }
    if (temp > max_energy)
    {
      max_energy = temp;
    }
  }

  factor = 10.0f;
  for (i = 0; i < i_max; i += 8)
  {
    temp = 0.01f;
    for (j = i; j < i + 8; j++)
    {
      temp += ptr_signal[j] * ptr_signal[j];
    }
    temp = (FLOAT32)sqrt(sqrt(max_energy / temp));
    if (temp < factor)
    {
      factor = temp;
    }
    for (j = i; j < i + 8; j++)
    {
      ptr_signal[j] *= factor;
    }
  }
  return;
}

/**
 *  impeghe_adapt_lo_freq_deemph
 *
 *  \brief Carries out adaptive low frequency de-emphasis.
 *
 *  \param [in ] ptr_signal Pointer to input signal.
 *  \param [in ] length Input data length.
 *  \param [in ] ptr_gains  Pointer to gains buffer.
 *
 *  \return VOID
 */
static VOID impeghe_adapt_lo_freq_deemph(FLOAT32 *ptr_signal, WORD32 length, FLOAT32 *ptr_gains)
{
  WORD32 i, j, i_max;
  FLOAT32 max_energy, factor, energy, rm;

  i_max = length >> 2;

  max_energy = 0.01f;
  for (i = 0; i < i_max; i += 8)
  {
    energy = 0.01f;
    for (j = i; j < i + 8; j++)
    {
      energy += ptr_signal[j] * ptr_signal[j];
    }
    if (energy > max_energy)
    {
      max_energy = energy;
    }
  }

  factor = 0.1f;
  for (i = 0; i < i_max; i += 8)
  {
    energy = 0.01f;
    for (j = i; j < i + 8; j++)
    {
      energy += ptr_signal[j] * ptr_signal[j];
    }

    rm = (FLOAT32)sqrt(energy / max_energy);
    if (rm > factor)
    {
      factor = rm;
    }
    for (j = i; j < i + 8; j++)
    {
      ptr_signal[j] *= factor;
    }
    ptr_gains[i / 8] = factor;
  }

  return;
}

/**
 *  impeghe_igf_get_swb_offset
 *
 *  \brief Get scalefactor window band offset.
 *
 *  \param [in ] igf_win_type   Intelligent Gap Filling window type.
 *  \param [in ] ptr_swb_offset     Pointer to scalefactor window band offset.
 *  \param [in ] ptr_swb_lpd_offset Pointer to scalefactor window band offset value.
 *
 *  \return VOID
 *
 */
static VOID impeghe_igf_get_swb_offset(WORD32 igf_win_type, WORD32 *ptr_swb_offset,
                                       WORD32 *ptr_swb_lpd_offset)
{
  WORD32 sfb;
  WORD32 swb_shift = 0;

  switch (igf_win_type)
  {
  case 0:
  case 1:
  case 4:
    swb_shift = 0;
    break;
  case 2:
  case 3:
    swb_shift = (WORD32)igf_win_type;
    break;
  }

  for (sfb = 0; sfb <= 15; sfb++)
  {
    ptr_swb_lpd_offset[sfb] = ptr_swb_offset[sfb] << swb_shift;
  }
  return;
}

/**
 *  impeghe_tcx_fac_encode
 *
 *  \brief TCX Forward Aliasing Cancellation encoding function.
 *
 *  \param [in ] pstr_usac_data              Pointer to USAC data structure.
 *  \param [in ] pstr_usac_config            Pointer to USAC config structure.
 *  \param [out] ptr_lpc_coeffs             Pointer to LPC data.
 *  \param [out] ptr_lpc_coeffs_quant       Pointer to quantized LPC data.
 *  \param [in ] ptr_speech                 Pointer to input ptr_speech signal.
 *  \param [in ] frame_len              Processing frame length.
 *  \param [in ] num_bits_per_supfrm    Number of encoded bits per super frame value.
 *  \param [in ] lpd_state              Pointer to LPD state.
 *  \param [out] params                 Pointer to LPD params.
 *  \param [out] n_param                Pointer to number of TCX Params.
 *  \param [in ] ptr_sfb_offset         Pointer to scalefactor band offset table
 *  \param [in ] usac_independency_flag USAC independency flag.
 *  \param [in ] num_sfb                Number of scalefactor bands.
 *  \param [in ] ch_idx                 Channel index value.
 *  \param [in ] k_idx                  Sub-frame index.
 *  \param [in ] ele_id                 Syntactic element id.
 *
 *  \return VOID
 */
VOID impeghe_tcx_fac_encode(ia_usac_data_struct *pstr_usac_data,
                            ia_usac_encoder_config_struct *pstr_usac_config,
                            FLOAT32 *ptr_lpc_coeffs, FLOAT32 *ptr_lpc_coeffs_quant,
                            FLOAT32 *ptr_speech, WORD32 frame_len, WORD32 num_bits_per_supfrm,
                            ia_usac_lpd_state_struct *lpd_state, WORD32 *params, WORD32 *n_param,
                            WORD32 *ptr_sfb_offset, WORD32 usac_independency_flag, WORD32 num_sfb,
                            WORD32 ch_idx, WORD32 chn, WORD32 k_idx, WORD32 ele_id)
{
  ia_usac_td_encoder_struct *st = pstr_usac_data->td_encoder[ch_idx];
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  FLOAT32 *weighted_sig = &pstr_scratch->p_wsig_buf[k_idx * st->len_subfrm];
  FLOAT32 *wsynth = pstr_scratch->p_wsyn_tcx_buf;
  FLOAT32 *synth = pstr_scratch->p_synth_tcx_buf;
  WORD32 tns_active = (ch_idx != -1) ? pstr_usac_config->tns_select : 0;
  ia_tns_info *pstr_tns_info = pstr_usac_data->pstr_tns_info[0];
  WORD32 enhanced_noise_filling = pstr_usac_data->str_igf_config[ele_id].igf_active;
  ia_igf_data_struct *pstr_igf_data = &pstr_usac_data->str_igf_data[ch_idx];
  ia_igf_config_struct *pstr_igf_config = &pstr_usac_data->str_igf_config[ele_id];
  WORD32 i, k, n, mode, i_subfr, lg, lext, index, target_bits;
  FLOAT32 tmp, gain, fac_ns, energy, gain_tcx, nsfill_en_thres;
  FLOAT32 *ptr_lp_flt_coeffs, lp_flt_coeffs[ORDER + 1];
  const FLOAT32 *sine_window_prev, *sine_window;
  FLOAT32 mem_tcx_q;
  FLOAT32 *xn;
  FLOAT32 *xn1 = pstr_scratch->p_xn1_tcx;
  FLOAT32 *xn_buf = pstr_scratch->p_xn_buf_tcx;
  FLOAT32 *x = pstr_scratch->p_x_tcx;
  FLOAT32 *x_tmp = pstr_scratch->p_x_tmp_tcx;
  FLOAT32 *en = pstr_scratch->p_en_tcx;
  FLOAT32 sq_gain;
  FLOAT32 gain_prev, gain_next;
  FLOAT32 *alfd_gains = pstr_scratch->p_alfd_gains_tcx;
  FLOAT32 *sq_enc = pstr_scratch->p_sq_enc_tcx;
  WORD32 *sq_quant = pstr_scratch->p_sq_quant_tcx;
  FLOAT32 sq_err_energy;
  WORD32 max_k;
  FLOAT32 *gain1 = pstr_scratch->p_gain1_tcx;
  FLOAT32 *gain2 = pstr_scratch->p_gain2_tcx;
  FLOAT32 *facelp = pstr_scratch->p_facelp_tcx;
  FLOAT32 *xn2 = pstr_scratch->p_xn2_tcx;
  FLOAT32 *fac_window = pstr_scratch->p_fac_window_tcx;
  FLOAT32 *x1 = pstr_scratch->p_x1_tcx;
  FLOAT32 *x2 = pstr_scratch->p_x2_tcx;
  WORD32 *y = pstr_scratch->p_y_tcx;

  WORD32 TTT;
  FLOAT32 corr = 0;
  WORD32 len_subfrm = st->len_subfrm;
  WORD32 fac_length = len_subfrm >> 1;
  // FLOAT64 tns_pred_gain = 0;
  WORD32 fac_len_prev, fac_len;

  if (frame_len == 4 * st->len_subfrm)
  {
    if (st->last_was_short)
    {
      fac_len_prev = (st->len_frame) / 16;
    }
    else
    {
      fac_len_prev = st->len_subfrm / 2;
    }
    if (st->next_is_short)
    {
      fac_len = (st->len_frame) / 16;
    }
    else
    {
      fac_len = st->len_subfrm / 2;
    }
  }
  else if (frame_len == 2 * st->len_subfrm)
  {
    if (k_idx == 0 && st->last_was_short)
    {
      fac_len_prev = (st->len_frame) / 16;
    }
    else
    {
      fac_len_prev = st->len_subfrm / 2;
    }
    if (k_idx == 2 && st->next_is_short)
    {
      fac_len = (st->len_frame) / 16;
    }
    else
    {
      fac_len = st->len_subfrm / 2;
    }
  }
  else
  {
    if (k_idx == 0 && st->last_was_short)
    {
      fac_len_prev = (st->len_frame) / 16;
    }
    else
    {
      fac_len_prev = st->len_subfrm / 2;
    }
    if (k_idx == 3 && st->next_is_short)
    {
      fac_len = (st->len_frame) / 16;
    }
    else
    {
      fac_len = st->len_subfrm / 2;
    }
  }

  memset(xn_buf, 0, (128 + frame_len + 128) * sizeof(FLOAT32));

  mode = frame_len / len_subfrm;
  if (!pstr_usac_config->full_band_lpd)
  {
    if (mode > 2)
    {
      mode = 3;
    }
  }
  else
  {
    if (mode > 2)
    {
      mode = 2;
    }
    else
    {
      if (mode == 2)
      {
        mode = 1;
      }
      else
      {
        mode = 2;
      }
    }
  }

  if (lpd_state->mode == 0)
  {
    params += fac_len_prev;
  }
  switch (fac_len_prev)
  {
  case 64:
    sine_window_prev = impeghe_sin_window_128;
    break;
  default:
    sine_window_prev = impeghe_sin_window_256;
    break;
  }
  switch (fac_len)
  {
  case 64:
    sine_window = impeghe_sin_window_128;
    break;
  default:
    sine_window = impeghe_sin_window_256;
    break;
  }

  lg = frame_len;
  lext = fac_length;
  xn = xn_buf + fac_length;

  *n_param = lg;

  target_bits = num_bits_per_supfrm - 10;

  for (i = 0; i < fac_length; i++)
  {
    xn_buf[i] = lpd_state->tcx_mem[i + 128 - fac_length];
  }

  memcpy(xn, ptr_speech, (frame_len + fac_length) * sizeof(FLOAT32));

  tmp = xn[-1];

  impeghe_apply_deemph(xn, TILT_FAC, frame_len, &tmp);

  memcpy(lpd_state->tcx_mem, &xn[frame_len - 128], 128 * sizeof(FLOAT32));

  memcpy(&xn[frame_len], &ptr_speech[frame_len], lext * sizeof(FLOAT32));
  impeghe_apply_deemph(&xn[frame_len], TILT_FAC, lext, &tmp);

  for (i = 0; i < ORDER + fac_len_prev; i++)
  {
    xn1[i] = xn_buf[fac_length - ORDER + i];
  }
  for (i = 0; i < ORDER + fac_len; i++)
  {
    xn2[i] = xn_buf[frame_len - ORDER + i];
  }

  if (lpd_state->mode >= -1)
  {

    for (i = 0; i < fac_length - fac_len_prev; i++)
    {
      xn_buf[i] = 0.0f;
    }
    for (i = fac_length - fac_len_prev; i < (fac_length + fac_len_prev); i++)
    {
      xn_buf[i] *= sine_window_prev[i - fac_length + fac_len_prev];
    }
    for (i = 0; i < (2 * fac_len); i++)
    {
      xn_buf[frame_len + fac_length - fac_len + i] *= sine_window[(2 * fac_len) - 1 - i];
    }
    for (i = 0; i < fac_length - fac_len; i++)
    {
      xn_buf[frame_len + fac_length + fac_len + i] = 0.0f;
    }
  }

  impeghe_tcx_mdct_main(xn_buf, x, (2 * fac_length), frame_len - (2 * fac_length),
                        (2 * fac_length), pstr_scratch);

  if (pstr_usac_config->full_band_lpd == 1 && frame_len == 1024)
  {
    FLOAT64 *temp_spec = (FLOAT64 *)pstr_scratch->ptr_tcx_scratch;

    WORD32 swb_offset[16];

    for (i = 0; i < frame_len; i++)
    {
      temp_spec[i] = x[i];
    }
    impeghe_igf_get_swb_offset(3, ptr_sfb_offset, swb_offset);

    if (enhanced_noise_filling == 1)
    {
      pstr_igf_config->is_short_block = (1 == mode) ? 1 : 0;
      pstr_igf_config->m_igf_start_sfb = pstr_igf_config->igf_start_sfb_sb;
      pstr_igf_config->m_igf_stop_sfb = pstr_igf_config->igf_stop_sfb_sb;
    }

    if (enhanced_noise_filling == 1 && pstr_igf_config->igf_after_tns_synth == 0)
    {
      impeghe_igf(temp_spec, swb_offset, pstr_igf_data, pstr_igf_config, 0, 0, 1, 0, 0, 40, 1,
                  pstr_scratch->ptr_igf_scratch);
    }

    if (tns_active != 0)
    {
      WORD32 i, j;
      FLOAT32 ptr_band_energy[15] = {0};
      pstr_tns_info->number_of_bands = pstr_tns_info->max_sfb_short;
      pstr_tns_info->block_type = EIGHT_SHORT_SEQUENCE;
      pstr_tns_info->spec = &temp_spec[0];
      pstr_tns_info->sfb_offset_table_short_tcx = swb_offset;
      j = 0;
      for (i = 0; i < pstr_tns_info->number_of_bands; i++)
      {
        while (j < swb_offset[i + 1])
        {
          ptr_band_energy[i] += (FLOAT32)(x[j] * x[j]);
          j++;
        }
      }

      impeghe_tns_encode(pstr_tns_info, pstr_tns_info, ptr_band_energy, 0, chn, 1024,
                         pstr_igf_config, pstr_scratch->p_tns_filter, 1,
                         pstr_scratch->ptr_tns_scratch);
    }

    if (enhanced_noise_filling == 1 && pstr_igf_config->igf_after_tns_synth == 1)
    {
      impeghe_igf(temp_spec, swb_offset, pstr_igf_data, pstr_igf_config, 0,
                  pstr_tns_info->window_data[0].tns_pred_gain, 1, 1,
                  pstr_igf_config->igf_after_tns_synth, 40, 1, pstr_scratch->ptr_igf_scratch);
    }

    for (i = 0; i < frame_len; i++)
    {
      temp_spec[i] = x[i];
    }
  }

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (ORDER + 1), lp_flt_coeffs);
  impeghe_lpc_mdct(lp_flt_coeffs, gain1, ((FDNS_RESOLUTION * len_subfrm) / LEN_FRAME) << 1,
                   pstr_scratch);

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (2 * (ORDER + 1)), lp_flt_coeffs);
  impeghe_lpc_mdct(lp_flt_coeffs, gain2, ((FDNS_RESOLUTION * len_subfrm) / LEN_FRAME) << 1,
                   pstr_scratch);

  impeghe_pre_shaping(x, lg, gain1, gain2);

  for (i = 0; i < lg; i++)
  {
    x_tmp[i] = x[i];
  }

  impeghe_adapt_lo_freq_emph(x, lg);

  sq_gain = impeghe_calc_sq_gain(x, target_bits, lg, pstr_scratch->p_sq_gain_en);

  for (i = 0; i < lg; i++)
  {
    sq_enc[i] = x[i] / sq_gain;

    if (sq_enc[i] > 0.f)
      sq_quant[i] = ((WORD32)(0.5f + sq_enc[i]));
    else
      sq_quant[i] = ((WORD32)(-0.5f + sq_enc[i]));
  }

  for (i = 0; i < lg; i++)
  {
    params[i + 2] = sq_quant[i];
    x[i] = (FLOAT32)sq_quant[i];
  }

  for (i = 0; i < lg; i++)
  {
    en[i] = x[i] * x[i];
  }

  if (pstr_usac_config->full_band_lpd)
  {
    if (mode == 4)
    {
      tmp = 0.9441f;
    }
    else if (mode == 2)
    {
      tmp = 0.8913f;
    }
  }
  else
  {
    if (mode == 3)
    {
      tmp = 0.9441f;
    }
    else if (mode == 2)
    {
      tmp = 0.8913f;
    }
    else
    {
      tmp = 0.7943f;
    }
  }

  energy = 0.0f;
  for (i = 0; i < lg; i++)
  {
    if (en[i] > energy)
    {
      energy = en[i];
    }
    en[i] = energy;
    energy *= tmp;
  }
  energy = 0.0f;
  for (i = lg - 1; i >= 0; i--)
  {
    if (en[i] > energy)
    {
      energy = en[i];
    }
    en[i] = energy;
    energy *= tmp;
  }

  nsfill_en_thres = 0.707f;

  tmp = 0.0625f;
  k = 1;
  for (i = 0; i < lg; i++)
  {
    if (en[i] <= nsfill_en_thres)
    {
      tmp += sq_enc[i] * sq_enc[i];
      k++;
    }
  }
  fac_ns = (FLOAT32)sqrt(tmp / (FLOAT32)k);

  impeghe_adapt_lo_freq_deemph(x, lg, alfd_gains);

  energy = 1e-6f;
  for (i = 0; i < lg; i++)
  {
    corr += x_tmp[i] * x[i];
    energy += x[i] * x[i];
  }
  gain_tcx = (corr / energy);

  if (gain_tcx == 0.0f)
  {
    gain_tcx = sq_gain;
  }

  energy = 0.0001f;
  for (i = 0; i < lg; i++)
  {
    tmp = x_tmp[i] - gain_tcx * x[i];
    energy += tmp * tmp;
  }

  tmp = (FLOAT32)sqrt((energy * (2.0f / (FLOAT32)lg)) / (FLOAT32)lg);

  for (i = 0; i < frame_len; i++)
  {
    wsynth[i] = weighted_sig[i] + tmp;
  }

  energy = 0.01f;
  for (i = 0; i < lg; i++)
  {
    energy += x[i] * x[i];
  }

  tmp = (FLOAT32)(2.0f * sqrt(energy) / (FLOAT32)lg);
  gain = gain_tcx * tmp;

  index = (WORD32)floor(0.5f + (28.0f * (FLOAT32)log10(gain)));
  if (index < 0)
  {
    index = 0;
  }
  if (index > 127)
  {
    index = 127;
  }
  params[1] = index;

  gain_tcx = (FLOAT32)pow(10.0f, ((FLOAT32)index) / 28.0f) / tmp;
  st->gain_tcx = gain_tcx;

  FLOAT32 i_gain;
  FLOAT32 *out_spec_curr = (FLOAT32 *)pstr_scratch->ptr_tcx_scratch;
  FLOAT32 *tcx_quant = x;
  WORD32 quant_gain_curr = 0, pred_bw, global_gain;

  global_gain = index;
  pred_bw = len_subfrm == 192 ? 120 : 160;

  FLOAT32 sq_gain_fdp = 0, max_cur_frame = 0;
  FLOAT32 *sq_enc_fdp = pstr_scratch->ptr_tcx_scratch + 1024;
  WORD32 *sq_quant_fdp = (WORD32 *)pstr_scratch->ptr_tcx_scratch + 2048;
  memset(&sq_quant_fdp[0], 0, sizeof(WORD32) * 1024);
  memset(&sq_enc_fdp[0], 0, sizeof(FLOAT32) * 1024);

  if (pstr_usac_config->fdp_enable == 1 && frame_len == 1024)
  {
    if (mode == (pstr_usac_config->full_band_lpd ? 2 : 3) && pstr_usac_config->fdp_enable == 1)
    {
      impeghe_fdp_encode(pstr_usac_data, pstr_usac_config, ch_idx, 0, CORE_MODE_TD);

      for (i = 0; i < 1024; i++)
      {
        out_spec_curr[i] = (FLOAT32)tcx_quant[i];
        if ((fabs(out_spec_curr[i])) > max_cur_frame)
        {
          max_cur_frame = (FLOAT32)fabs(out_spec_curr[i]);
        }
      }

      // i_gain
      i_gain = 64.0f / st->gain_tcx;

      // quant_gain_curr
      for (i = 0; i < lg; i++)
      {
        const int xInt = (int)ia_mul_flt(out_spec_curr[i], 16.0f);
        quant_gain_curr += xInt * xInt;
      }

      quant_gain_curr = impeghe_rounded_sqrt(41 + 16 * (unsigned int)quant_gain_curr);
      if (pstr_usac_config->full_band_lpd)
      {
        quant_gain_curr = (int)(ia_mac_flt(0.5f, (float)quant_gain_curr, 1024.0f) / (float)lg);
      }

      quant_gain_curr =
          (impeghe_pow10_gain_div28[global_gain] + quant_gain_curr) / (2 * quant_gain_curr);

      if (usac_independency_flag || (mode != (pstr_usac_config->full_band_lpd ? 2 : 3)))
      {
        for (i = 0; i < 172; i++)
        {
          pstr_usac_data->fdp_data.fdp_quant_spec_prev_tcx[ch_idx][0][i] = 0;
          pstr_usac_data->fdp_data.fdp_quant_spec_prev_tcx[ch_idx][1][i] = 0;
        }
      }

      impeghe_tcx_fdp_encode(&pstr_usac_data->fdp_data, out_spec_curr, tcx_quant, quant_gain_curr,
                             i_gain, 1024, pred_bw, pred_bw, ch_idx, pstr_scratch->ptr_fdp_int);

      if (pstr_usac_data->fdp_data.fdp_active[ch_idx])
      {
        impeghe_adapt_lo_freq_emph(out_spec_curr, lg);
        sq_gain_fdp =
            impeghe_calc_sq_gain(out_spec_curr, target_bits, lg, pstr_scratch->p_sq_gain_en);

        for (i = 0; i < lg; i++)
        {
          sq_enc_fdp[i] = out_spec_curr[i] / sq_gain_fdp;

          if (sq_enc_fdp[i] > 0.f)
            sq_quant_fdp[i] = ((WORD32)(0.5f + sq_enc_fdp[i]));
          else
            sq_quant_fdp[i] = ((WORD32)(-0.5f + sq_enc_fdp[i]));
        }

        for (i = 0; i < lg; i++)
        {
          FLOAT32 error_rel_abs = (FLOAT32)abs(params[i + 2] - sq_quant_fdp[i]) / max_cur_frame;
          if (error_rel_abs < ERROR_THRESHOLD)
          {
            params[i + 2] = sq_quant_fdp[i];
          }
        }
      }

      max_cur_frame = 0;
    }
  }

  sq_err_energy = 0.f;
  n = 0;
  for (k = lg / 2; k < lg;)
  {
    tmp = 0.f;

    max_k = MIN(lg, k + 8);
    for (i = k; i < max_k; i++)
    {
      tmp += sq_quant[i] * sq_quant[i];
    }
    if (tmp == 0.f)
    {
      tmp = 0.f;
      for (i = k; i < max_k; i++)
      {
        tmp += sq_enc[i] * sq_enc[i];
      }

      sq_err_energy += (FLOAT32)log10((tmp / (FLOAT64)8) + 0.000000001);
      n += 1;
    }
    k = max_k;
  }
  if (n > 0)
  {
    fac_ns = (FLOAT32)pow(10., sq_err_energy / (FLOAT64)(2 * n));
  }
  else
  {
    fac_ns = 0.f;
  }

  tmp = 8.0f - (16.0f * fac_ns);

  index = (WORD32)floor(tmp + 0.5);
  if (index < 0)
  {
    index = 0;
  }
  if (index > 7)
  {
    index = 7;
  }

  params[0] = index;

  fac_ns = 0.0625f * (8.0f - ((FLOAT32)index));

  impeghe_noise_shaping(x, lg, gain1, gain2);

  impeghe_tcx_imdct(x, xn_buf, (2 * fac_length), frame_len - (2 * fac_length), (2 * fac_length),
                    pstr_scratch);
  for (i = 0; i < frame_len + (2 * fac_length); i++)
  {
    xn_buf[i] = xn_buf[i] * (2.0f / lg);
  }

  for (i = 0; i < fac_len_prev; i++)
  {
    fac_window[i] = sine_window_prev[i] * sine_window_prev[(2 * fac_len_prev) - 1 - i];
    fac_window[fac_len_prev + i] =
        1.0f - (sine_window_prev[fac_len_prev + i] * sine_window_prev[fac_len_prev + i]);
  }

  for (i = 0; i < fac_len_prev; i++)
  {
    xn1[ORDER + i] -= sq_gain * xn_buf[fac_length + i] * sine_window_prev[fac_len_prev + i];
  }
  for (i = 0; i < fac_len; i++)
  {
    xn2[ORDER + i] -= sq_gain * xn_buf[i + frame_len] * sine_window[(2 * fac_len) - 1 - i];
  }

  for (i = 0; i < ORDER; i++)
  {
    xn1[i] -= lpd_state->tcx_quant[1 + 128 - ORDER + i];
    xn2[i] -= sq_gain * xn_buf[frame_len - ORDER + i];
  }

  for (i = 0; i < fac_len_prev; i++)
  {
    facelp[i] = lpd_state->tcx_quant[1 + 128 + i] * fac_window[fac_len_prev + i] +
                lpd_state->tcx_quant[1 + 128 - 1 - i] * fac_window[fac_len_prev - 1 - i];
  }

  energy = 0.0f;
  for (i = 0; i < fac_len_prev; i++)
    energy += xn1[ORDER + i] * xn1[ORDER + i];
  energy *= 2.0f;
  tmp = 0.0f;
  for (i = 0; i < fac_len_prev; i++)
    tmp += facelp[i] * facelp[i];
  if (tmp > energy)
    gain = (FLOAT32)sqrt(energy / tmp);
  else
    gain = 1.0f;

  for (i = 0; i < fac_len_prev; i++)
  {
    xn1[ORDER + i] -= gain * facelp[i];
  }

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (ORDER + 1), lp_flt_coeffs);
  impeghe_compute_lp_residual(lp_flt_coeffs, xn1 + ORDER, x1, fac_len_prev);

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (2 * (ORDER + 1)), lp_flt_coeffs);
  impeghe_compute_lp_residual(lp_flt_coeffs, xn2 + ORDER, x2, fac_len);

  impeghe_tcx_mdct(x1, x1, fac_len_prev, pstr_scratch);
  impeghe_tcx_mdct(x2, x2, fac_len, pstr_scratch);

  gain_prev = (FLOAT32)(sq_gain * 0.5f * sqrt(((FLOAT32)fac_len_prev) / (FLOAT32)frame_len));
  gain_next = (FLOAT32)(sq_gain * 0.5f * sqrt(((FLOAT32)fac_len) / (FLOAT32)frame_len));

  for (i = 0; i < fac_len_prev; i++)
  {
    x1[i] /= gain_prev;
  }
  for (i = 0; i < fac_len; i++)
  {
    x2[i] /= gain_next;
  }
  for (i = 0; i < fac_len_prev / 4; i++)
  {
    k = i * lg / (8 * fac_len_prev);
    x1[i] /= alfd_gains[k];
  }
  for (i = 0; i < fac_len / 4; i++)
  {
    k = i * lg / (8 * fac_len);
    x2[i] /= alfd_gains[k];
  }

  for (i = 0; i < fac_len; i += 8)
  {
    impeghe_find_nearest_neighbor(&x2[i], &y[i]);
  }
  for (i = 0; i < fac_len; i++)
  {
    lpd_state->avq_params[i] = y[i];
    x2[i] = (FLOAT32)y[i];
  }

  for (i = 0; i < fac_len_prev; i += 8)
  {
    impeghe_find_nearest_neighbor(&x1[i], &y[i]);
  }

  for (i = 0; i < fac_len_prev; i++)
  {
    x1[i] = (FLOAT32)y[i];
  }

  gain_prev = (FLOAT32)(gain_tcx * 0.5f * sqrt(((FLOAT32)fac_len_prev) / (FLOAT32)frame_len));
  gain_next = (FLOAT32)(gain_tcx * 0.5f * sqrt(((FLOAT32)fac_len) / (FLOAT32)frame_len));

  for (i = 0; i < fac_len_prev; i++)
  {
    x1[i] *= gain_prev;
  }
  for (i = 0; i < fac_len; i++)
  {
    x2[i] *= gain_next;
  }
  for (i = 0; i<fac_len_prev>> 2; i++)
  {
    k = i * lg / (fac_len_prev << 3);
    x1[i] *= alfd_gains[k];
  }
  for (i = 0; i<fac_len>> 2; i++)
  {
    k = i * lg / (fac_len << 3);
    x2[i] *= alfd_gains[k];
  }
  impeghe_tcx_mdct(x1, xn1, fac_len_prev, pstr_scratch);
  impeghe_tcx_mdct(x2, xn2, fac_len, pstr_scratch);

  FLOAT32 coeff1 = (2.0f / (FLOAT32)fac_len_prev), coeff2 = (2.0f / (FLOAT32)fac_len);

  for (i = 0; i < fac_len_prev; i++)
  {
    xn1[i] = xn1[i] * coeff1;
  }

  for (i = 0; i < fac_len; i++)
  {
    xn2[i] = xn2[i] * coeff2;
  }

  memset(xn1 + fac_len_prev, 0, fac_len_prev * sizeof(FLOAT32));
  memset(xn2 + fac_len, 0, fac_len * sizeof(FLOAT32));

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (ORDER + 1), lp_flt_coeffs);
  impeghe_synthesis_tool_float(lp_flt_coeffs, xn1, xn1, 2 * fac_len_prev, xn1 + fac_len_prev,
                               pstr_scratch->p_buf_synthesis_tool);

  impeghe_get_weighted_lpc(ptr_lpc_coeffs_quant + (2 * (ORDER + 1)), lp_flt_coeffs);
  impeghe_synthesis_tool_float(lp_flt_coeffs, xn2, xn2, fac_len, xn2 + fac_len,
                               pstr_scratch->p_buf_synthesis_tool);

  for (i = 0; i < fac_len_prev; i++)
  {
    xn1[i] += facelp[i];
  }

  for (i = 0; i < frame_len + (fac_length << 1); i++)
  {
    xn_buf[i] *= gain_tcx;
  }

  if (lpd_state->mode >= -1)
  {
    for (i = 0; i < (2 * fac_len_prev); i++)
    {
      xn_buf[i + fac_length - fac_len_prev] *= sine_window_prev[i];
    }
    for (i = 0; i < fac_length - fac_len_prev; i++)
    {
      xn_buf[i] = 0.0f;
    }
  }
  for (i = 0; i < (2 * fac_len); i++)
  {
    xn_buf[i + frame_len + fac_length - fac_len] *= sine_window[(2 * fac_len) - 1 - i];
  }
  for (i = 0; i < fac_length - fac_len; i++)
  {
    xn_buf[i + frame_len + fac_length + fac_len] = 0.0f;
  }

  if (lpd_state->mode != 0)
  {
    for (i = 0; i < (2 * fac_length); i++)
    {
      xn_buf[i] += lpd_state->tcx_quant[1 + 128 - fac_length + i];
    }

    mem_tcx_q = lpd_state->tcx_quant[128 - fac_length];
  }
  else
  {
    for (i = 0; i < fac_len_prev; i++)
    {
      params[i - fac_len_prev] = y[i];
    }

    for (i = 0; i < (2 * fac_len_prev); i++)
    {
      xn_buf[i + fac_length] += xn1[i];
    }
    mem_tcx_q = lpd_state->tcx_quant[128];
  }

  memcpy(lpd_state->tcx_quant, xn_buf + frame_len + fac_length - 128 - 1,
         (1 + 256) * sizeof(FLOAT32));

  for (i = 0; i < fac_len; i++)
  {
    xn_buf[i + frame_len + (fac_length - fac_len)] += xn2[i];
  }

  if (lpd_state->mode > 0)
  {
    impeghe_apply_preemph(xn_buf, TILT_FAC, fac_length, &mem_tcx_q);

    ptr_lp_flt_coeffs = lpd_state->lpc_coeffs_quant;

    TTT = fac_length % LEN_SUBFR;
    if (TTT != 0)
    {
      memcpy(&(lpd_state->synth[ORDER + 128 - fac_length]), &xn_buf[0], TTT * sizeof(FLOAT32));
      impeghe_compute_lp_residual(ptr_lp_flt_coeffs,
                                  &(lpd_state->synth[ORDER + 128 - fac_length]),
                                  &(lpd_state->acelp_exc[(2 * len_subfrm) - fac_length]), TTT);

      ptr_lp_flt_coeffs += (ORDER + 1);
    }

    for (i_subfr = TTT; i_subfr < fac_length; i_subfr += LEN_SUBFR)
    {
      memcpy(&(lpd_state->synth[ORDER + 128 - fac_length + i_subfr]), &xn_buf[i_subfr],
             LEN_SUBFR * sizeof(FLOAT32));
      impeghe_compute_lp_residual(
          ptr_lp_flt_coeffs, &(lpd_state->synth[ORDER + 128 - fac_length + i_subfr]),
          &(lpd_state->acelp_exc[(2 * len_subfrm) - fac_length + i_subfr]), LEN_SUBFR);
      ptr_lp_flt_coeffs += (ORDER + 1);
    }

    ptr_lp_flt_coeffs = lpd_state->lpc_coeffs;
    for (i_subfr = 0; i_subfr < fac_length; i_subfr += LEN_SUBFR)
    {
      impeghe_get_weighted_lpc(ptr_lp_flt_coeffs, lp_flt_coeffs);
      impeghe_compute_lp_residual(
          lp_flt_coeffs, &(lpd_state->synth[ORDER + 128 - fac_length + i_subfr]),
          &(lpd_state->wsynth[1 + 128 - fac_length + i_subfr]), LEN_SUBFR);
      ptr_lp_flt_coeffs += (ORDER + 1);
    }
    tmp = lpd_state->wsynth[0 + 128 - fac_length];
    impeghe_apply_deemph(&(lpd_state->wsynth[1 + 128 - fac_length]), TILT_FAC, fac_length, &tmp);
  }

  k = ((frame_len / LEN_SUBFR) - 2) * (ORDER + 1);
  memcpy(lpd_state->lpc_coeffs, ptr_lpc_coeffs + k, 2 * (ORDER + 1) * sizeof(FLOAT32));

  memcpy(lpd_state->lpc_coeffs_quant, ptr_lpc_coeffs_quant + (2 * (ORDER + 1)),
         (ORDER + 1) * sizeof(FLOAT32));
  memcpy(lpd_state->lpc_coeffs_quant + (ORDER + 1), lpd_state->lpc_coeffs_quant,
         (ORDER + 1) * sizeof(FLOAT32));

  memcpy(synth - 128, &(lpd_state->synth[ORDER]), 128 * sizeof(FLOAT32));
  lpd_state->tcx_fac = xn[frame_len - 1];

  impeghe_apply_preemph(xn, TILT_FAC, frame_len, &mem_tcx_q);
  ptr_lp_flt_coeffs = ptr_lpc_coeffs_quant;
  for (i_subfr = 0; i_subfr < frame_len; i_subfr += LEN_SUBFR)
  {
    memcpy(&synth[i_subfr], &xn[i_subfr], LEN_SUBFR * sizeof(FLOAT32));
    impeghe_compute_lp_residual(ptr_lpc_coeffs_quant + (2 * (ORDER + 1)), &synth[i_subfr],
                                &xn[i_subfr], LEN_SUBFR);
  }
  memcpy(lpd_state->synth, synth + frame_len - (ORDER + 128), (ORDER + 128) * sizeof(FLOAT32));

  if (frame_len == len_subfrm)
  {
    memcpy(x, lpd_state->acelp_exc + len_subfrm, len_subfrm * sizeof(FLOAT32));
    memcpy(lpd_state->acelp_exc, x, len_subfrm * sizeof(FLOAT32));
    memcpy(lpd_state->acelp_exc + len_subfrm, xn, len_subfrm * sizeof(FLOAT32));
  }
  else
  {
    memcpy(lpd_state->acelp_exc, xn + frame_len - (2 * len_subfrm),
           2 * len_subfrm * sizeof(FLOAT32));
  }

  memcpy(wsynth - 128, &(lpd_state->wsynth[1]), 128 * sizeof(FLOAT32));

  ptr_lp_flt_coeffs = ptr_lpc_coeffs;
  for (i_subfr = 0; i_subfr < frame_len; i_subfr += LEN_SUBFR)
  {
    impeghe_get_weighted_lpc(ptr_lp_flt_coeffs, lp_flt_coeffs);
    impeghe_compute_lp_residual(lp_flt_coeffs, &synth[i_subfr], &wsynth[i_subfr], LEN_SUBFR);
    ptr_lp_flt_coeffs += (ORDER + 1);
  }
  tmp = wsynth[-1];
  impeghe_apply_deemph(wsynth, TILT_FAC, frame_len, &tmp);

  memcpy(lpd_state->wsynth, wsynth + frame_len - (1 + 128), (1 + 128) * sizeof(FLOAT32));

  lpd_state->mode = mode;

  lpd_state->num_bits = 10 + target_bits;

  return;
}
/** @} */ /* End of CoreEncProc */
