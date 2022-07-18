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
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_api.h"
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
#include "impeghe_lpd_rom.h"
#include "impeghe_lpd.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_acelp_encode
 *
 *  \brief ACELP mode encoding main processing function.
 *
 *  \param [out] lp_filt_coeff       Pointer to buffer containing linear prediction coefficients.
 *  \param [out] quant_lp_filt_coeff Pointer to buffer containing quantized lpc.
 *  \param [in ] speech_in           Pointer to input speech buffer.
 *  \param [in ] wsig_in             Pointer to weighted signal input buffer.
 *  \param [out] synth_out           Pointer to synthesized speech buffer.
 *  \param [out] wsynth_out          Pointer to weighted synthesized speech buffer.
 *  \param [in ] acelp_core_mode     ACELP core mode.
 *  \param [in ] pstr_lpd_state           Pointer to LPD state structure.
 *  \param [in ] len_subfrm          Length of subframe.
 *  \param [in ] norm_corr           Normalized correlation value.
 *  \param [in ] norm_corr2          Normalized correlation value.
 *  \param [in ] ol_pitch_lag1       Open-loop pitch lag value 1.
 *  \param [in ] ol_pitch_lag2       Open-loop pitch lag value 2.
 *  \param [in ] pit_adj             Pitch adjustment value.
 *  \param [out] acelp_params        Pointer to buffer carrying ACELP bitstream parameters.
 *  \param [in ] pstr_scratch        Pointer to scratch buffer.
 *
 *  \return VOID
 */
VOID impeghe_acelp_encode(FLOAT32 *lp_filt_coeff, FLOAT32 *quant_lp_filt_coeff,
                          FLOAT32 *speech_in, FLOAT32 *wsig_in, FLOAT32 *synth_out,
                          FLOAT32 *wsynth_out, WORD16 acelp_core_mode,
                          ia_usac_lpd_state_struct *pstr_lpd_state, WORD32 len_subfrm,
                          FLOAT32 norm_corr, FLOAT32 norm_corr2, WORD32 ol_pitch_lag1,
                          WORD32 ol_pitch_lag2, WORD32 pit_adj, WORD32 *acelp_params,
                          impeghe_scratch_mem *pstr_scratch)
{
  LOOPIDX i, j, idx;
  WORD32 i_subfr, num_bits, t;
  WORD32 t0, t0_min, t0_max, index, subfrm_flag;
  WORD32 t0_frac;
  WORD32 fac_length;
  WORD32 min_pitch_lag_res1_4;
  WORD32 min_pitch_lag_res1_2;
  WORD32 min_pitch_lag_res1;
  WORD32 max_pitch_lag;
  WORD16 cb_exc[LEN_SUBFR];

  FLOAT32 temp, energy, max_ener, mean_ener_code;
  FLOAT32 pitch_gain, code_gain, gain1, gain2;
  FLOAT32 sum;
  FLOAT32 mem_txn, mem_txnq;
  FLOAT32 tgt_cb_corr[5], tgt_cb_corr2[2];
  UWORD8 *ptr_scratch;
  FLOAT32 *p_lp_filt_coeff, *p_quant_lp_filt_coeff, weighted_lpc[ORDER + 1];
  FLOAT32 *imp_res, *code, *error, *cn, *xn, *xn2, *dn;
  FLOAT32 *y0, *y1, *y2;
  FLOAT32 *exc_buf, *exc;
  ptr_scratch = (UWORD8 *)pstr_scratch->ptr_acelp_scratch;
  imp_res = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(imp_res[0]);
  code = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(code[0]);
  error = (FLOAT32 *)ptr_scratch;
  ptr_scratch += (ORDER + LEN_SUBFR + 8) * sizeof(error[0]);
  cn = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(cn[0]);
  xn = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(xn[0]);
  xn2 = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(xn2[0]);
  dn = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(dn[0]);
  y0 = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(y0[0]);
  y1 = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(y1[0]);
  y2 = (FLOAT32 *)ptr_scratch;
  ptr_scratch += LEN_SUBFR * sizeof(y2[0]);
  exc_buf = pstr_scratch->p_acelp_exc_buf;

  fac_length = len_subfrm / 2;
  if (pstr_lpd_state->mode > 0)
  {
    for (i = 0; i < fac_length; i++)
    {
      acelp_params[i] = pstr_lpd_state->avq_params[i];
    }
    acelp_params += fac_length;
  }

  if (pit_adj != SR_MAX)
  {
    exc = exc_buf + (2 * len_subfrm);
  }
  else
  {
    exc = exc_buf + (2 * len_subfrm) + 41;
  }

  memset(exc_buf, 0, (2 * len_subfrm) * sizeof(exc_buf[0]));
  memcpy(exc_buf, pstr_lpd_state->acelp_exc, 2 * len_subfrm * sizeof(FLOAT32));
  memcpy(wsynth_out - 128, &(pstr_lpd_state->wsynth[1]), 128 * sizeof(FLOAT32));
  memcpy(synth_out - 128, &(pstr_lpd_state->synth[ORDER]), 128 * sizeof(FLOAT32));

  num_bits = ((impeghe_acelp_core_numbits_1024[acelp_core_mode] - NBITS_MODE) / 4) - NBITS_LPC;

  if (pit_adj != 0)
  {
    i = (((pit_adj * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN;
    min_pitch_lag_res1_4 = TMIN + i;
    min_pitch_lag_res1_2 = TFR2 - i;
    min_pitch_lag_res1 = TFR1;
    max_pitch_lag = TMAX + (6 * i);
  }
  else
  {
    min_pitch_lag_res1_4 = TMIN;
    min_pitch_lag_res1_2 = TFR2;
    min_pitch_lag_res1 = TFR1;
    max_pitch_lag = TMAX;
  }

  ol_pitch_lag1 *= OPL_DECIM;
  ol_pitch_lag2 *= OPL_DECIM;

  t0_min = ol_pitch_lag1 - 8;

  t = min(ol_pitch_lag1, ol_pitch_lag2) - 4;
  if (t0_min < t)
    t0_min = t;

  if (t0_min < min_pitch_lag_res1_4)
  {
    t0_min = min_pitch_lag_res1_4;
  }
  t0_max = t0_min + 15;
  t = max(ol_pitch_lag1, ol_pitch_lag2) + 4;
  if (t0_max > t)
  {
    t0_max = t;
  }

  if (t0_max > max_pitch_lag)
  {
    t0_max = max_pitch_lag;
    t0_min = t0_max - 15;
  }

  max_ener = 0.0;
  mean_ener_code = 0.0;
  p_quant_lp_filt_coeff = quant_lp_filt_coeff;
  for (i_subfr = 0; i_subfr < len_subfrm; i_subfr += LEN_SUBFR)
  {
    impeghe_compute_lp_residual(p_quant_lp_filt_coeff, &speech_in[i_subfr], &exc[i_subfr],
                                LEN_SUBFR);
    energy = 0.01f;
    for (i = 0; i < LEN_SUBFR; i++)
    {
      energy += exc[i + i_subfr] * exc[i + i_subfr];
    }
    energy = 10.0f * (FLOAT32)log10(energy / ((FLOAT32)LEN_SUBFR));
    if (energy < 0.0)
    {
      energy = 0.0;
    }
    if (energy > max_ener)
    {
      max_ener = energy;
    }
    mean_ener_code += 0.25f * energy;
    p_quant_lp_filt_coeff += (ORDER + 1);
  }

  mean_ener_code -= 5.0f * norm_corr;
  mean_ener_code -= 5.0f * norm_corr2;

  temp = (mean_ener_code - 18.0f) / 12.0f;
  index = (WORD32)floor(temp + 0.5);
  if (index > 3)
  {
    index = 3;
  }
  if (index < 0)
  {
    index = 0;
  }

  mean_ener_code = (((FLOAT32)index) * 12.0f) + 18.0f;

  while ((mean_ener_code < (max_ener - 27.0)) && (index < 3))
  {
    index++;
    mean_ener_code += 12.0;
  }
  *acelp_params = index;
  acelp_params++;

  p_lp_filt_coeff = lp_filt_coeff;
  p_quant_lp_filt_coeff = quant_lp_filt_coeff;
  for (i_subfr = 0; i_subfr < len_subfrm; i_subfr += LEN_SUBFR)
  {
    subfrm_flag = i_subfr;
    if ((len_subfrm == 256) && (i_subfr == (2 * LEN_SUBFR)))
    {
      subfrm_flag = 0;

      t0_min = ol_pitch_lag2 - 8;

      t = MIN(ol_pitch_lag1, ol_pitch_lag2) - 4;
      if (t0_min < t)
        t0_min = t;

      if (t0_min < min_pitch_lag_res1_4)
      {
        t0_min = min_pitch_lag_res1_4;
      }
      t0_max = t0_min + 15;

      t = MAX(ol_pitch_lag1, ol_pitch_lag2) + 4;
      if (t0_max > t)
        t0_max = t;

      if (t0_max > max_pitch_lag)
      {
        t0_max = max_pitch_lag;
        t0_min = t0_max - 15;
      }
    }

    memcpy(xn, &wsig_in[i_subfr], LEN_SUBFR * sizeof(FLOAT32));

    memcpy(error, &synth_out[i_subfr - ORDER], ORDER * sizeof(FLOAT32));
    memset(error + ORDER, 0, LEN_SUBFR * sizeof(FLOAT32));
    impeghe_synthesis_tool_float(p_quant_lp_filt_coeff, error + ORDER, error + ORDER, LEN_SUBFR,
                                 error, pstr_scratch->p_buf_synthesis_tool);
    impeghe_get_weighted_lpc(p_lp_filt_coeff, weighted_lpc);
    impeghe_compute_lp_residual(weighted_lpc, error + ORDER, xn2, LEN_SUBFR);

    temp = wsynth_out[i_subfr - 1];
    impeghe_apply_deemph(xn2, TILT_FAC, LEN_SUBFR, &temp);
    memcpy(y0, xn2, LEN_SUBFR * sizeof(FLOAT32));

    for (i = 0; i < LEN_SUBFR; i++)
    {
      xn[i] -= xn2[i];
    }
    impeghe_compute_lp_residual(p_quant_lp_filt_coeff, &speech_in[i_subfr], &exc[i_subfr],
                                LEN_SUBFR);

    memset(code, 0, ORDER * sizeof(FLOAT32));
    memcpy(code + ORDER, xn, (LEN_SUBFR / 2) * sizeof(FLOAT32));
    temp = 0.0;
    impeghe_apply_preemph(code + ORDER, TILT_FAC, LEN_SUBFR / 2, &temp);
    impeghe_get_weighted_lpc(p_lp_filt_coeff, weighted_lpc);
    impeghe_synthesis_tool_float(weighted_lpc, code + ORDER, code + ORDER, LEN_SUBFR / 2, code,
                                 pstr_scratch->p_buf_synthesis_tool);
    impeghe_compute_lp_residual(p_quant_lp_filt_coeff, code + ORDER, cn, LEN_SUBFR / 2);
    memcpy(cn + (LEN_SUBFR / 2), &exc[i_subfr + (LEN_SUBFR / 2)],
           (LEN_SUBFR / 2) * sizeof(FLOAT32));

    impeghe_get_weighted_lpc(p_lp_filt_coeff, weighted_lpc);
    memset(imp_res, 0, LEN_SUBFR * sizeof(FLOAT32));
    memcpy(imp_res, weighted_lpc, (ORDER + 1) * sizeof(FLOAT32));
    impeghe_synthesis_tool_float(p_quant_lp_filt_coeff, imp_res, imp_res, LEN_SUBFR,
                                 &imp_res[ORDER + 1], pstr_scratch->p_buf_synthesis_tool);
    temp = 0.0;
    impeghe_apply_deemph(imp_res, TILT_FAC, LEN_SUBFR, &temp);

    impeghe_closed_loop_search(&exc[i_subfr], xn, imp_res, t0_min, t0_max, &t0_frac, subfrm_flag,
                               min_pitch_lag_res1_2, min_pitch_lag_res1, &t0);

    if (subfrm_flag != 0)
    {
      i = t0 - t0_min;
      index = i * 4 + t0_frac;
    }
    else
    {
      if (t0 < min_pitch_lag_res1_2)
      {
        index = t0 * 4 + t0_frac - (min_pitch_lag_res1_4 * 4);
      }
      else if (t0 < min_pitch_lag_res1)
      {
        index = t0 * 2 + (t0_frac >> 1) - (min_pitch_lag_res1_2 * 2) +
                ((min_pitch_lag_res1_2 - min_pitch_lag_res1_4) * 4);
      }
      else
      {
        index = t0 - min_pitch_lag_res1 + ((min_pitch_lag_res1_2 - min_pitch_lag_res1_4) * 4) +
                ((min_pitch_lag_res1 - min_pitch_lag_res1_2) * 2);
      }

      t0_min = t0 - 8;
      if (t0_min < min_pitch_lag_res1_4)
      {
        t0_min = min_pitch_lag_res1_4;
      }
      t0_max = t0_min + 15;
      if (t0_max > max_pitch_lag)
      {
        t0_max = max_pitch_lag;
        t0_min = t0_max - 15;
      }
    }
    *acelp_params = index;
    acelp_params++;

    impeghe_acelp_ltpred_cb_exc(&exc[i_subfr], t0, t0_frac, LEN_SUBFR + 1);
    impeghe_convolve(&exc[i_subfr], imp_res, y1);
    gain1 = impeghe_acelp_tgt_cb_corr2(xn, y1, tgt_cb_corr);

    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      xn2[idx] = xn[idx] - gain1 * y1[idx];
    }
    energy = 0.0;
    for (i = 0; i < LEN_SUBFR; i++)
    {
      energy += xn2[i] * xn2[i];
    }

    for (i = 0; i < LEN_SUBFR; i++)
    {
      code[i] = (FLOAT32)(0.18 * exc[i - 1 + i_subfr] + 0.64 * exc[i + i_subfr] +
                          0.18 * exc[i + 1 + i_subfr]);
    }
    impeghe_convolve(code, imp_res, y2);
    gain2 = impeghe_acelp_tgt_cb_corr2(xn, y2, tgt_cb_corr2);

    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      xn2[idx] = xn[idx] - gain2 * y2[idx];
    }
    temp = 0.0;
    for (i = 0; i < LEN_SUBFR; i++)
    {
      temp += xn2[i] * xn2[i];
    }

    if (temp >= energy)
    {
      *acelp_params = 1;
      pitch_gain = gain1;
    }
    else
    {
      *acelp_params = 0;
      memcpy(&exc[i_subfr], code, LEN_SUBFR * sizeof(FLOAT32));
      memcpy(y1, y2, LEN_SUBFR * sizeof(FLOAT32));
      pitch_gain = gain2;
      tgt_cb_corr[0] = tgt_cb_corr2[0];
      tgt_cb_corr[1] = tgt_cb_corr2[1];
    }
    acelp_params++;

    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      xn2[idx] = xn[idx] - pitch_gain * y1[idx];
      cn[idx] = cn[idx] - pitch_gain * exc[i_subfr + idx];
    }
    temp = 0.0;
    impeghe_apply_preemph(imp_res, TILT_CODE, LEN_SUBFR, &temp);
    if (t0_frac > 2)
    {
      t0++;
    }

    for (i = t0; i < LEN_SUBFR; i++)
    {
      imp_res[i] += imp_res[i - t0] * PIT_SHARP;
    }

    for (i = 0; i < LEN_SUBFR; i++)
    {
      sum = 0.0F;
      for (j = i; j < LEN_SUBFR; j++)
      {
        sum += xn2[j] * imp_res[j - i];
      }
      dn[i] = sum;
    }
    switch (acelp_core_mode)
    {
    case ACELP_CORE_MODE_9k6:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_20, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_11k2:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_28, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_12k8:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_36, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_14k4:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_44, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_16k:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_52, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_18k4:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_64, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 8;
      break;
    case ACELP_CORE_MODE_8k0:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_12, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    case ACELP_CORE_MODE_8k8:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_16, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 4;
      break;
    default:
      impeghe_acelp_cb_exc(dn, cn, imp_res, cb_exc, y2, ACELP_NUM_BITS_64, 0, acelp_params,
                           pstr_scratch->p_acelp_ir_buf, ptr_scratch);
      acelp_params += 8;
      break;
    }

    for (i = 0; i < LEN_SUBFR; i++)
    {
      code[i] = (FLOAT32)(cb_exc[i] / 512);
    }

    temp = 0.0;
    impeghe_apply_preemph(code, TILT_CODE, LEN_SUBFR, &temp);
    for (i = t0; i < LEN_SUBFR; i++)
    {
      code[i] += code[i - t0] * PIT_SHARP;
    }

    impeghe_acelp_tgt_cb_corr1(xn, y1, y2, tgt_cb_corr);
    impeghe_acelp_quant_gain(code, &pitch_gain, &code_gain, tgt_cb_corr, mean_ener_code,
                             acelp_params);
    acelp_params++;

    temp = 0.0;
    for (i = 0; i < LEN_SUBFR; i++)
    {
      temp += code[i] * code[i];
    }
    temp *= code_gain * code_gain;

    energy = 0.0;
    for (i = 0; i < LEN_SUBFR; i++)
    {
      energy += exc[i + i_subfr] * exc[i + i_subfr];
    }
    energy *= (pitch_gain * pitch_gain);

    for (i = 0; i < LEN_SUBFR; i++)
    {
      exc[i + i_subfr] = pitch_gain * exc[i + i_subfr] + code_gain * code[i];
    }

    for (i = 0; i < LEN_SUBFR; i++)
    {
      wsynth_out[i + i_subfr] = y0[i] + (pitch_gain * y1[i]) + (code_gain * y2[i]);
    }

    impeghe_synthesis_tool_float(p_quant_lp_filt_coeff, &exc[i_subfr], &synth_out[i_subfr],
                                 LEN_SUBFR, &synth_out[i_subfr - ORDER],
                                 pstr_scratch->p_buf_synthesis_tool);
    p_lp_filt_coeff += (ORDER + 1);
    p_quant_lp_filt_coeff += (ORDER + 1);
  }

  memcpy(pstr_lpd_state->synth, synth_out + len_subfrm - (ORDER + 128),
         (ORDER + 128) * sizeof(FLOAT32));
  memcpy(pstr_lpd_state->wsynth, wsynth_out + len_subfrm - (1 + 128),
         (1 + 128) * sizeof(FLOAT32));
  memcpy(pstr_lpd_state->lpc_coeffs_quant, p_quant_lp_filt_coeff - (2 * (ORDER + 1)),
         (2 * (ORDER + 1)) * sizeof(FLOAT32));
  memcpy(pstr_lpd_state->lpc_coeffs, p_lp_filt_coeff - (2 * (ORDER + 1)),
         (2 * (ORDER + 1)) * sizeof(FLOAT32));
  memcpy(pstr_lpd_state->acelp_exc, exc - len_subfrm, 2 * len_subfrm * sizeof(FLOAT32));

  mem_txnq = pstr_lpd_state->tcx_fac;
  mem_txn = pstr_lpd_state->tcx_mem[128 - 1];

  p_quant_lp_filt_coeff = quant_lp_filt_coeff;
  for (i_subfr = 0; i_subfr < (len_subfrm - 2 * LEN_SUBFR); i_subfr += LEN_SUBFR)
  {
    impeghe_get_weighted_lpc(p_quant_lp_filt_coeff, weighted_lpc);

    memcpy(error, &speech_in[i_subfr], LEN_SUBFR * sizeof(FLOAT32));
    impeghe_apply_deemph(error, TILT_FAC, LEN_SUBFR, &mem_txn);

    memcpy(error, &synth_out[i_subfr], LEN_SUBFR * sizeof(FLOAT32));
    impeghe_apply_deemph(error, TILT_FAC, LEN_SUBFR, &mem_txnq);

    p_quant_lp_filt_coeff += (ORDER + 1);
  }

  pstr_lpd_state->tcx_quant[0] = mem_txnq;
  for (i_subfr = 0; i_subfr < (2 * LEN_SUBFR); i_subfr += LEN_SUBFR)
  {
    impeghe_get_weighted_lpc(p_quant_lp_filt_coeff, weighted_lpc);

    memcpy(&(pstr_lpd_state->tcx_mem[i_subfr]),
           &speech_in[i_subfr + (len_subfrm - 2 * LEN_SUBFR)], LEN_SUBFR * sizeof(FLOAT32));
    impeghe_apply_deemph(&(pstr_lpd_state->tcx_mem[i_subfr]), TILT_FAC, LEN_SUBFR, &mem_txn);

    memcpy(&(pstr_lpd_state->tcx_quant[1 + i_subfr]),
           &synth_out[i_subfr + (len_subfrm - 2 * LEN_SUBFR)], LEN_SUBFR * sizeof(FLOAT32));
    impeghe_apply_deemph(&(pstr_lpd_state->tcx_quant[1 + i_subfr]), TILT_FAC, LEN_SUBFR,
                         &mem_txnq);
    p_quant_lp_filt_coeff += (ORDER + 1);
  }
  pstr_lpd_state->tcx_fac = mem_txnq;

  impeghe_get_weighted_lpc(p_quant_lp_filt_coeff, weighted_lpc);

  memcpy(error, &synth_out[len_subfrm - ORDER], ORDER * sizeof(FLOAT32));
  for (i_subfr = (2 * LEN_SUBFR); i_subfr < (4 * LEN_SUBFR); i_subfr += LEN_SUBFR)
  {
    memset(error + ORDER, 0, LEN_SUBFR * sizeof(FLOAT32));

    impeghe_synthesis_tool_float(p_quant_lp_filt_coeff, error + ORDER, error + ORDER, LEN_SUBFR,
                                 error, pstr_scratch->p_buf_synthesis_tool);
    memcpy(&(pstr_lpd_state->tcx_quant[1 + i_subfr]), error + ORDER, LEN_SUBFR * sizeof(FLOAT32));
    impeghe_apply_deemph(&(pstr_lpd_state->tcx_quant[1 + i_subfr]), TILT_FAC, LEN_SUBFR,
                         &mem_txnq);
    memcpy(error, error + LEN_SUBFR, ORDER * sizeof(FLOAT32));
  }

  pstr_lpd_state->mode = 0;

  pstr_lpd_state->num_bits = num_bits;
}
/** @} */ /* End of CoreEncProc */
