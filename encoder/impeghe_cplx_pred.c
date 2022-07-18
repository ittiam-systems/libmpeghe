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
#include <string.h>

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
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_signal_classifier.h"
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
#include "impeghe_rom.h"

/**
 *  impeghe_compute_pred_coef
 *
 *  \brief Calculates complex stereo prediction coefficients
 *
 *  \param [in]		num_lines				Number of MDCT coefficients per
 * prediction
 * band
 *  \param [in] 	complex_coef			Flag denoting real or complex stereo
 * prediction
 *  \param [in] 	ptr_spec_mdct_dmx		Pointer to MDCT downmix spectrum
 *  \param [in] 	ptr_spec_mdst_dmx		Pointer to MDST downmix spectrum
 *  \param [in] 	ptr_spec_mdct_mid_side	Pointer to MDCT side spectrum
 *  \param [out] 	pred_coef_re		    Prediction coefficient real part
 *  \param [out] 	pred_coef_im		    Prediction coefficient imaginary part
 *  \param [out] 	pred_coef_q_re		    Prediction coefficient quantized real part
 *  \param [out] 	pred_coef_q_im		    Prediction coefficient quantized imaginary
 * part
 *  \param [out] 	pred_coef_q_int_re	    Prediction coefficient quantized real part as
 * integer
 *  \param [out] 	pred_coef_q_int_im	    Prediction coefficient quantized imaginary
 * part
 * as integer
 *
 *  \return VOID
 */
static VOID impeghe_compute_pred_coef(WORD32 num_lines, WORD32 complex_coef,
                                      FLOAT64 *ptr_spec_mdct_dmx, FLOAT64 *ptr_spec_mdst_dmx,
                                      FLOAT64 *ptr_spec_mdct_mid_side, FLOAT32 *pred_coef_re,
                                      FLOAT32 *pred_coef_im, FLOAT32 *pred_coef_q_re,
                                      FLOAT32 *pred_coef_q_im, WORD32 *pred_coef_q_int_re,
                                      WORD32 *pred_coef_q_int_im)
{
  LOOPIDX bin_idx;
  FLOAT32 iprod_re = 0.0f, iprod_im = 0.0f;
  FLOAT32 eps = 1.0e-6f;
  const FLOAT32 k_delta = 0.1f;
  const FLOAT32 k_max = 3.0f;
  WORD32 pred_coef_sign_re, pred_coef_sign_im;
  FLOAT32 pred_coef_abs_re, pred_coef_abs_im;
  FLOAT32 abs_dmx = 0.0f;

  for (bin_idx = 0; bin_idx < num_lines; bin_idx++)
  {
    /* D = Dr + jDi */
    /* D*S = (Dr + jDi)*S = Dr*S + j(Di*S) */
    /* alpha = alpha_r - jalpha_i */
    if (complex_coef == 1)
    {
      iprod_re += (FLOAT32)(ptr_spec_mdct_dmx[bin_idx] * ptr_spec_mdct_mid_side[bin_idx]);
      iprod_im += (FLOAT32)(ptr_spec_mdst_dmx[bin_idx] * ptr_spec_mdct_mid_side[bin_idx]);
      abs_dmx += (FLOAT32)(ptr_spec_mdct_dmx[bin_idx] * ptr_spec_mdct_dmx[bin_idx] +
                           ptr_spec_mdst_dmx[bin_idx] * ptr_spec_mdst_dmx[bin_idx]);
    }
    else
    {
      iprod_re += (FLOAT32)(ptr_spec_mdct_dmx[bin_idx] * ptr_spec_mdct_mid_side[bin_idx]);
      abs_dmx += (FLOAT32)(ptr_spec_mdct_dmx[bin_idx] * ptr_spec_mdct_dmx[bin_idx]);
    }
  }

  /* Compute real and imaginary parts of prediction coefficient  */
  *pred_coef_re = iprod_re / (abs_dmx + eps);
  pred_coef_sign_re = *pred_coef_re > 0 ? 1 : -1;
  pred_coef_abs_re = (FLOAT32)min(fabs(*pred_coef_re), k_max);
  *pred_coef_q_int_re = pred_coef_sign_re * (WORD32)(pred_coef_abs_re / k_delta + 0.5f);
  *pred_coef_q_re = *pred_coef_q_int_re * k_delta;

  if (complex_coef == 1)
  {
    *pred_coef_im = iprod_im / (abs_dmx + eps);
    pred_coef_sign_im = *pred_coef_im > 0 ? 1 : -1;
    pred_coef_abs_im = (FLOAT32)min(fabs(*pred_coef_im), k_max);
    *pred_coef_q_int_im = pred_coef_sign_im * (WORD32)(pred_coef_abs_im / k_delta + 0.5f);
    *pred_coef_q_im = *pred_coef_q_int_im * k_delta;
  }
}

/**
 *  impeghe_compute_res
 *
 *  \brief Calculates MDCT residual spectrum corresponding to complex stereo prediction
 *
 *  \param [in] 	num_lines			        Number of MDCT coefficients per
 * prediction
 * band
 *  \param [in] 	complex_coef		        Flag denoting real or complex stereo
 * prediction
 *  \param [in] 	pred_coef_q_re	            Prediction coefficient quantized real part
 *  \param [in] 	pred_coef_q_im	            Prediction coefficient quantized imaginary
 * part
 *  \param [in] 	ptr_spec_mdct_dmx		    Pointer to MDCT downmix spectrum
 *  \param [in] 	ptr_spec_mdst_dmx		    Pointer to MDST downmix spectrum
 *  \param [in] 	ptr_spec_mdct_mid_side		Pointer to MDCT side spectrum
 *  \param [out] 	ptr_spec_mdct_res			Pointer to MDCT residual spectrum
 *
 *  \return VOID
 */
static VOID impeghe_compute_res(WORD32 num_lines, WORD32 complex_coef, FLOAT32 pred_coef_q_re,
                                FLOAT32 pred_coef_q_im, FLOAT64 *ptr_spec_mdct_dmx,
                                FLOAT64 *ptr_spec_mdst_dmx, FLOAT64 *ptr_spec_mdct_mid_side,
                                FLOAT64 *ptr_spec_mdct_res)
{
  LOOPIDX i;

  for (i = 0; i < num_lines; i++)
  {
    /* DMX = M; E = S - alpha*DMX if pred_dir = 0 */
    /* DMX = S; E = M - alpha*DMX if pred_dir = 1 */
    if (complex_coef == 1)
    {
      ptr_spec_mdct_res[i] =
          (FLOAT32)(ptr_spec_mdct_mid_side[i] - pred_coef_q_re * ptr_spec_mdct_dmx[i] -
                    pred_coef_q_im * ptr_spec_mdst_dmx[i]);
    }
    else
    {
      ptr_spec_mdct_res[i] =
          (FLOAT32)(ptr_spec_mdct_mid_side[i] - pred_coef_q_re * ptr_spec_mdct_dmx[i]);
    }
  }
}

/**
 *  impeghe_filter_and_add
 *
 *  \brief Helper function to filter and add
 *
 *  \param [in]  ptr_in         Pointer to input buffer
 *  \param [in]  length         Length of the input data
 *  \param [in]  ptr_filter     Pointer to filter coefficients
 *  \param [out] ptr_out        Pointer to output buffer
 *  \param [in]  factor_even    Multiplication factor
 *
 *  \return VOID
 *
 */
static VOID impeghe_filter_and_add(const FLOAT64 *ptr_in, const WORD32 length,
                                   const FLOAT64 *ptr_filter, FLOAT64 *ptr_out,
                                   const WORD32 factor_even)
{
  LOOPIDX i;
  FLOAT64 s;

  i = 0;
  s = ptr_filter[6] * ptr_in[2] + ptr_filter[5] * ptr_in[1] + ptr_filter[4] * ptr_in[0] +
      ptr_filter[3] * ptr_in[0] + ptr_filter[2] * ptr_in[1] + ptr_filter[1] * ptr_in[2] +
      ptr_filter[0] * ptr_in[3];
  ptr_out[i] += s * factor_even;
  i = 1;
  s = ptr_filter[6] * ptr_in[1] + ptr_filter[5] * ptr_in[0] + ptr_filter[4] * ptr_in[0] +
      ptr_filter[3] * ptr_in[1] + ptr_filter[2] * ptr_in[2] + ptr_filter[1] * ptr_in[3] +
      ptr_filter[0] * ptr_in[4];
  ptr_out[i] += s;
  i = 2;
  s = ptr_filter[6] * ptr_in[0] + ptr_filter[5] * ptr_in[0] + ptr_filter[4] * ptr_in[1] +
      ptr_filter[3] * ptr_in[2] + ptr_filter[2] * ptr_in[3] + ptr_filter[1] * ptr_in[4] +
      ptr_filter[0] * ptr_in[5];
  ptr_out[i] += s * factor_even;

  for (i = 3; i < length - 4; i += 2)
  {
    s = ptr_filter[6] * ptr_in[i - 3] + ptr_filter[5] * ptr_in[i - 2] +
        ptr_filter[4] * ptr_in[i - 1] + ptr_filter[3] * ptr_in[i] +
        ptr_filter[2] * ptr_in[i + 1] + ptr_filter[1] * ptr_in[i + 2] +
        ptr_filter[0] * ptr_in[i + 3];
    ptr_out[i] += s;
    s = ptr_filter[6] * ptr_in[i - 2] + ptr_filter[5] * ptr_in[i - 1] +
        ptr_filter[4] * ptr_in[i] + ptr_filter[3] * ptr_in[i + 1] +
        ptr_filter[2] * ptr_in[i + 2] + ptr_filter[1] * ptr_in[i + 3] +
        ptr_filter[0] * ptr_in[i + 4];
    ptr_out[i + 1] += s * factor_even;
  }

  i = length - 3;
  s = ptr_filter[6] * ptr_in[i - 3] + ptr_filter[5] * ptr_in[i - 2] +
      ptr_filter[4] * ptr_in[i - 1] + ptr_filter[3] * ptr_in[i] + ptr_filter[2] * ptr_in[i + 1] +
      ptr_filter[1] * ptr_in[i + 2] + ptr_filter[0] * ptr_in[i + 2];
  ptr_out[i] += s;
  i = length - 2;
  s = ptr_filter[6] * ptr_in[i - 3] + ptr_filter[5] * ptr_in[i - 2] +
      ptr_filter[4] * ptr_in[i - 1] + ptr_filter[3] * ptr_in[i] + ptr_filter[2] * ptr_in[i + 1] +
      ptr_filter[1] * ptr_in[i + 1] + ptr_filter[0] * ptr_in[i];
  ptr_out[i] += s * factor_even;
  i = length - 1;
  s = ptr_filter[6] * ptr_in[i - 3] + ptr_filter[5] * ptr_in[i - 2] +
      ptr_filter[4] * ptr_in[i - 1] + ptr_filter[3] * ptr_in[i] + ptr_filter[2] * ptr_in[i] +
      ptr_filter[1] * ptr_in[i - 1] + ptr_filter[0] * ptr_in[i - 2];
  ptr_out[i] += s;
}

/**
 *  impeghe_estimate_dmx_im
 *
 *  \brief Computation of the downmix MDST estimate from downmix MDCT spectrum
 *
 *  \param [in]  ptr_dmx_re        Pointer to MDCT downmix spectrum of current frame
 *  \param [in]  ptr_dmx_re_prev   Pointer to MDCT downmix spectrum of previous frame
 *  \param [out] ptr_dmx_im        Pointer to MDST downmix spectrum of current frame
 *  \param [in]  window            Window type
 *  \param [in]  w_shape           Window shape
 *  \param [in]  prev_w_shape      Previous window shape
 *  \param [in]  num_sbk           Number of sub-blocks
 *  \param [in]  bins_per_sbk      Number of bins per sub-block
 *
 *  \return VOID
 *
 */
static VOID impeghe_estimate_dmx_im(const FLOAT64 *ptr_dmx_re, const FLOAT64 *ptr_dmx_re_prev,
                                    FLOAT64 *ptr_dmx_im, WORD32 window, const WORD32 w_shape,
                                    const WORD32 prev_w_shape, WORD32 num_sbk,
                                    WORD32 bins_per_sbk)
{
  LOOPIDX i;
  const FLOAT64 *ptr_mdst_fcoeff_curr, *ptr_mdst_fcoeff_prev;

  switch (window)
  {
  case ONLY_LONG_SEQUENCE:
  case EIGHT_SHORT_SEQUENCE:
    ptr_mdst_fcoeff_curr = impeghe_mdst_fcoeff_longshort_curr[prev_w_shape][w_shape];
    ptr_mdst_fcoeff_prev = impeghe_mdst_fcoeff_l_s_start_left_prev[prev_w_shape];
    break;
  case LONG_START_SEQUENCE:
    ptr_mdst_fcoeff_curr = impeghe_mdst_fcoeff_start_curr[prev_w_shape][w_shape];
    ptr_mdst_fcoeff_prev = impeghe_mdst_fcoeff_l_s_start_left_prev[prev_w_shape];
    break;
  case LONG_STOP_SEQUENCE:
    ptr_mdst_fcoeff_curr = impeghe_mdst_fcoeff_stop_cur[prev_w_shape][w_shape];
    ptr_mdst_fcoeff_prev = impeghe_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  case STOP_START_SEQUENCE:
    ptr_mdst_fcoeff_curr = impeghe_mdst_fcoeff_stopstart_cur[prev_w_shape][w_shape];
    ptr_mdst_fcoeff_prev = impeghe_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  default:
    ptr_mdst_fcoeff_curr = impeghe_mdst_fcoeff_stopstart_cur[prev_w_shape][w_shape];
    ptr_mdst_fcoeff_prev = impeghe_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  }

  for (i = 0; i < num_sbk; i++)
  {
    impeghe_filter_and_add(ptr_dmx_re, bins_per_sbk, ptr_mdst_fcoeff_curr, ptr_dmx_im, 1);

    if (ptr_dmx_re_prev)
    {
      impeghe_filter_and_add(ptr_dmx_re_prev, bins_per_sbk, ptr_mdst_fcoeff_prev, ptr_dmx_im, -1);
    }

    ptr_dmx_re_prev = ptr_dmx_re;
    ptr_dmx_re += bins_per_sbk;
    ptr_dmx_im += bins_per_sbk;
  }
  return;
}

/**
 *  impeghe_usac_cplx_save_prev
 *
 *  \brief Save current frame complex prediction data for use in next frame
 *
 *  \param [in]  ptr_mdct_spec          Pointer to current frame MDCT spectrum
 *  \param [out] ptr_mdct_spec_prev     Pointer to buffer to store current frame MDCT spectrum
 *  \param [in]  save_zeros             Flag to set previous frame data to zeros
 *  \param [in]  condition_2            Second condition to set previous data to zeros
 *  \param [in]  samp_per_bk            Number of samples per block
 *  \param [in]  bins_per_sbk           Number of bins per sub-block
 *
 *  \return VOID
 *
 */
static VOID impeghe_usac_cplx_save_prev(FLOAT64 *ptr_mdct_spec, FLOAT64 *ptr_mdct_spec_prev,
                                        WORD32 save_zeros, WORD32 condition_2, WORD32 samp_per_bk,
                                        WORD32 bins_per_sbk)
{
  WORD32 offset;

  offset = samp_per_bk - bins_per_sbk;

  if (save_zeros || condition_2)
  {
    memset(ptr_mdct_spec_prev + offset, 0, sizeof(FLOAT64) * bins_per_sbk);
  }
  else
  {
    memcpy(ptr_mdct_spec_prev + offset, ptr_mdct_spec + offset, sizeof(FLOAT64) * bins_per_sbk);
  }
  return;
}

/**
 *  impeghe_compute_ipd
 *
 *  \brief Calculate inter-channel phase difference
 *
 *  \param [in] ptr_spec_real1	Pointer to real part of channel 1 spectral coefficients
 *  \param [in] ptr_spec_imag1	Pointer to imaginary part of channel 1 spectral coefficients
 *  \param [in] ptr_spec_real2	Pointer to real part of channel 2 spectral coefficients
 *  \param [in] ptr_spec_imag2	Pointer to imaginary part of channel 2 spectral coefficients
 *  \param [in] num_lines       Number of frequency bins
 *
 *  \return FLOAT64         Interchannel phase difference
 */

static FLOAT64 impeghe_compute_ipd(FLOAT64 *ptr_spec_real1, FLOAT64 *ptr_spec_imag1,
                                   FLOAT64 *ptr_spec_real2, FLOAT64 *ptr_spec_imag2,
                                   WORD32 num_lines)
{
  LOOPIDX i;

  FLOAT64 ipd = 0.0f;
  FLOAT64 cross_corr_real = 0.0f, cross_corr_imag = 0.0f;

  for (i = 0; i < num_lines; i++)
  {
    cross_corr_real +=
        ptr_spec_real1[i] * ptr_spec_real2[i] + ptr_spec_imag1[i] * ptr_spec_imag2[i];
    cross_corr_imag +=
        -ptr_spec_imag2[i] * ptr_spec_real1[i] + ptr_spec_imag1[i] * ptr_spec_real2[i];
  }

  ipd = (FLOAT64)atan2(cross_corr_imag, cross_corr_real);
  ipd = ipd > 0 ? ipd : 2. * PI + ipd;

  return ipd;
}

/**
 *  impeghe_cplx_pred_main
 *
 *  \brief Performs actual complex prediction processing
 *
 *  \param [in] 	num_sfb						    Number of SFBs
 *  \param [in] 	num_window_groups			    Number of window groups
 *  \param [in,out] ptr_spec_mdct_mid		        Pointer to MDCT mid spectrum
 *  \param [in,out] ptr_spec_mdct_side		        Pointer to MDCT side spectrum
 *  \param [out] 	pred_coef_q_int_re			    Prediction coefficient
 * quantized
 * real
 * part as integer
 *  \param [out] 	pred_coef_q_int_im			    Prediction coefficient
 * quantized
 * imaginary part as integer
 *  \param [out] 	pred_dir			            Flag indicating direction of
 * prediction
 *  \param [in,out] pstr_usac_data				    Pointer to USAC data structure
 *  \param [in] 	pstr_sfb_prms				    Pointer to SFB parameter
 * structure
 *  \param [in] 	usac_independancy_flag		    Flag denoting USAC independent frame
 *  \param [in] 	pstr_usac_config			    Pointer to encoder
 * configuration
 * structure
 *  \param [in] 	ptr_scratch_cmpx_mdct_buf	    Pointer to Complex MDCT scratch buffer
 *  \param [in]		cplx_pred_used				    Flag denoting whether complex
 * prediction or L/R coding is used per window group and scale-factor band
 *  \param [in] 	chn							    Channel index
 *  \param [in] 	ptr_sfb_offsets				    Pointer to grouped SFB offsets
 *  \param [in]     nrg_mid                         Energy of mid channel per frame
 *  \param [in]     nrg_side                        Energy of side channel per frame
 *  \param [in]     ms_mask_flag                    Pointer to MS mask
 *  \return VOID
 */
static VOID impeghe_cplx_pred_main(
    WORD32 num_sfb, WORD32 num_window_groups, FLOAT64 *ptr_spec_mdct_mid,
    FLOAT64 *ptr_spec_mdct_side, WORD32 pred_coef_q_int_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
    WORD32 pred_coef_q_int_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG], WORD32 *pred_dir,
    ia_usac_data_struct *pstr_usac_data, ia_sfb_params_struct *pstr_sfb_prms,
    WORD32 usac_independancy_flag, ia_usac_encoder_config_struct *pstr_usac_config,
    FLOAT64 *ptr_scratch_cmpx_mdct_buf,
    WORD32 cplx_pred_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG], WORD32 chn,
    const WORD32 *ptr_sfb_offsets, FLOAT32 nrg_mid, FLOAT32 nrg_side, WORD32 *ms_mask_flag)
{
  LOOPIDX group, sfb, i;
  FLOAT32 pred_coef_re, pred_coef_im, pred_coef_q_re, pred_coef_q_im;
  const WORD32 sfb_per_pred_band = 2;
  WORD32 left = 0, right = 0, save_zeros = 0, condition_2 = 0, samp_per_bk = 0, bins_per_sbk = 0,
         num_sbk = 0;
  FLOAT64 *ptr_dmx_re_prev;
  FLOAT64 *ptr_spec_mdct_res = &ptr_scratch_cmpx_mdct_buf[0];
  const WORD32 sfb_count = num_window_groups * num_sfb;
  const WORD32 sfb_per_group = num_sfb;
  WORD32 sfb_offsets = 0, zero_flag, spec_start, spec_end;

  left = chn, right = chn + 1;

  /* Number of sub-blocks */
  if (pstr_usac_config->window_sequence[left] == EIGHT_SHORT_SEQUENCE)
  {
    num_sbk = MAX_SHORT_WINDOWS;
  }
  if (pstr_usac_config->window_sequence[left] == ONLY_LONG_SEQUENCE ||
      pstr_usac_config->window_sequence[left] == LONG_START_SEQUENCE ||
      pstr_usac_config->window_sequence[left] == LONG_STOP_SEQUENCE ||
      pstr_usac_config->window_sequence[left] == STOP_START_SEQUENCE)
  {
    num_sbk = 1;
  }

  samp_per_bk = pstr_usac_config->ccfl;
  bins_per_sbk = samp_per_bk / num_sbk;

  /* Compute prediction direction */
  if (nrg_mid >= nrg_side)
  {
    *pred_dir = 0;
  }
  else
  {
    *pred_dir = 1;
  }

  if (pstr_usac_data->complex_coef[chn] == 1)
  {

    save_zeros = ((pstr_usac_config->window_sequence[left] == EIGHT_SHORT_SEQUENCE &&
                   pstr_usac_config->window_sequence[right] != EIGHT_SHORT_SEQUENCE) ||
                  (pstr_usac_config->window_sequence[left] != EIGHT_SHORT_SEQUENCE &&
                   pstr_usac_config->window_sequence[right] == EIGHT_SHORT_SEQUENCE));

    condition_2 = (usac_independancy_flag || pstr_usac_data->core_mode_prev[left] ||
                   pstr_usac_data->core_mode_prev[right]);

    /* Compute current frame's MDST down-mix*/
    ptr_dmx_re_prev = !(usac_independancy_flag) ? pstr_usac_data->ptr_dmx_re_save[chn] : NULL;

    memset(pstr_usac_data->ptr_dmx_im[chn], 0, sizeof(FLOAT64) * FRAME_LEN_LONG);

    impeghe_estimate_dmx_im(*pred_dir == 0 ? ptr_spec_mdct_mid : ptr_spec_mdct_side,
                            ptr_dmx_re_prev, pstr_usac_data->ptr_dmx_im[chn],
                            pstr_usac_config->window_sequence[left],
                            pstr_sfb_prms->window_shape[left],
                            pstr_usac_config->window_shape_prev[left], num_sbk, bins_per_sbk);

    /* MCLT of downmix = dmx_re + j*dmx_im */
    /*  Save MDCT down-mix for use as previous frame MDCT down-mix in the next frame */
    impeghe_usac_cplx_save_prev(*pred_dir == 0 ? &ptr_spec_mdct_mid[0] : &ptr_spec_mdct_side[0],
                                pstr_usac_data->ptr_dmx_re_save[chn], save_zeros, condition_2,
                                samp_per_bk, bins_per_sbk);
  }

  /* Reset buffer to zero */
  for (group = 0; group < MAX_SHORT_WINDOWS; group++)
  {
    memset(pred_coef_q_int_re, 0, MAX_NUM_SFB_LONG * sizeof(WORD32));
    memset(pred_coef_q_int_im, 0, MAX_NUM_SFB_LONG * sizeof(WORD32));
  }

  group = 0;
  for (sfb = 0; sfb < sfb_count; sfb += sfb_per_group, group++)
  {
    for (sfb_offsets = 0; sfb_offsets < sfb_per_group; sfb_offsets += sfb_per_pred_band)
    {
      if (cplx_pred_used[group][sfb_offsets] == 1)
      {
        zero_flag = (ptr_sfb_offsets[sfb + sfb_offsets + 1] != FRAME_LEN_LONG);
        spec_start = ptr_sfb_offsets[sfb + sfb_offsets];
        spec_end = (zero_flag ? ptr_sfb_offsets[sfb + sfb_offsets + 2]
                              : ptr_sfb_offsets[sfb + sfb_offsets + 1]);

        /* Calculate prediction coefficients */
        impeghe_compute_pred_coef(
            spec_end - spec_start, pstr_usac_data->complex_coef[chn],
            *pred_dir == 0 ? &ptr_spec_mdct_mid[spec_start] : &ptr_spec_mdct_side[spec_start],
            pstr_usac_data->complex_coef[chn] == 1 ? &pstr_usac_data->ptr_dmx_im[chn][spec_start]
                                                   : NULL,
            *pred_dir == 0 ? &ptr_spec_mdct_side[spec_start] : &ptr_spec_mdct_mid[spec_start],
            &pred_coef_re, pstr_usac_data->complex_coef[chn] == 1 ? &pred_coef_im : NULL,
            &pred_coef_q_re, pstr_usac_data->complex_coef[chn] == 1 ? &pred_coef_q_im : NULL,
            &pred_coef_q_int_re[group][sfb_offsets],
            pstr_usac_data->complex_coef[chn] == 1 ? &pred_coef_q_int_im[group][sfb_offsets]
                                                   : NULL);

        /* Calculate residual */
        impeghe_compute_res(
            spec_end - spec_start, pstr_usac_data->complex_coef[chn], pred_coef_q_re,
            pstr_usac_data->complex_coef[chn] == 1 ? pred_coef_q_im : 0,
            *pred_dir == 0 ? &ptr_spec_mdct_mid[spec_start] : &ptr_spec_mdct_side[spec_start],
            pstr_usac_data->complex_coef[chn] == 1 ? &pstr_usac_data->ptr_dmx_im[chn][spec_start]
                                                   : NULL,
            *pred_dir == 0 ? &ptr_spec_mdct_side[spec_start] : &ptr_spec_mdct_mid[spec_start],
            &ptr_spec_mdct_res[spec_start]);
      }
    }
  }

  /* Compute the prediction gain */
  FLOAT32 pred_gain = 0.f, nrg_res = 0.f;
  for (i = 0; i < pstr_usac_config->ccfl; i++)
  {
    nrg_res += (FLOAT32)(ptr_spec_mdct_res[i] * ptr_spec_mdct_res[i]);
  }
  pred_gain =
      10.f * log10f((*pred_dir == 0 ? nrg_side : nrg_mid) / nrg_res); /* Prediction gain in dB */

  if (pred_gain > 20.f) /* Retain complex prediction */
  {
    if (*pred_dir == 1)
    {
      for (i = 0; i < pstr_usac_config->ccfl; i++)
      {
        ptr_spec_mdct_mid[i] = ptr_spec_mdct_side[i];
        ptr_spec_mdct_side[i] = ptr_spec_mdct_res[i];
      }
    }
    else
    {
      for (i = 0; i < pstr_usac_config->ccfl; i++)
      {
        ptr_spec_mdct_side[i] = ptr_spec_mdct_res[i];
      }
    }
  }
  else /* Use M/S */
  {
    *ms_mask_flag = 0;
    /* Revert spectra to L and R */
    for (i = 0; i < pstr_usac_config->ccfl; i++)
    {
      ptr_spec_mdct_mid[i] = pstr_usac_data->left_chan_save[chn][i];
      ptr_spec_mdct_side[i] = pstr_usac_data->right_chan_save[chn][i];
    }
  }

  return;
}

/**
 *  impeghe_cplx_pred_proc
 *
 *  \brief Complex prediction pre-processing
 *
 *  \param [in,out] pstr_usac_data				Pointer to USAC data structure
 *  \param [in] 	pstr_usac_config			Pointer to encoder configuration
 * structure
 *  \param [in] 	usac_independancy_flag		Flag denoting USAC independent frame
 *  \param [in] 	pstr_sfb_prms				Pointer to SFB parameter structure
 *  \param [in] 	chn							Channel index
 *  \param [in,out]	pstr_psy_data				Pointer to psychoacoustic data
 * structure
 *  \param [in] 	ptr_sfb_offsets				Pointer to SFB offsets
 *  \param [in] 	scratch_cmpx_mdct_temp_buf	Pointer to complex prediction scratch
 * buffer
 *  \param [in] 	ptr_ms_spec					Pointer to mid-side
 * spectral
 * coefficients
 *  \param [in] 	nrg_mid						Mid energy per frame
 *  \param [in] 	nrg_side					Side energy per frame
 *  \param [in] 	ele_id					    Element index
 *
 *  \return VOID
 */
VOID impeghe_cplx_pred_proc(ia_usac_data_struct *pstr_usac_data,
                            ia_usac_encoder_config_struct *pstr_usac_config,
                            WORD32 usac_independancy_flag, ia_sfb_params_struct *pstr_sfb_prms,
                            WORD32 chn, ia_psy_mod_data_struct *pstr_psy_data,
                            const WORD32 *ptr_sfb_offsets, FLOAT64 *ptr_scratch_cmpx_mdct_buf,
                            FLOAT64 *ptr_ms_spec, FLOAT32 nrg_mid, FLOAT32 nrg_side,
                            WORD32 ele_id)
{
  FLOAT32 *ptr_sfb_enegry_left = pstr_psy_data[chn].ptr_sfb_energy_long;
  FLOAT32 *ptr_sfb_energy_right = pstr_psy_data[chn + 1].ptr_sfb_energy_long;
  const FLOAT32 *ptr_sfb_energy_mid = pstr_psy_data[chn].ptr_sfb_energy_long_ms;
  const FLOAT32 *ptr_sfb_energy_side = pstr_psy_data[chn + 1].ptr_sfb_energy_long_ms;
  FLOAT32 *ptr_sfb_thr_left = pstr_psy_data[chn].ptr_sfb_thr_long;
  FLOAT32 *ptr_sfb_thr_right = pstr_psy_data[chn + 1].ptr_sfb_thr_long;
  FLOAT32 *ptr_sfb_spread_energy_left = pstr_psy_data[chn].ptr_sfb_spreaded_energy_long;
  FLOAT32 *ptr_sfb_spread_energy_right = pstr_psy_data[chn + 1].ptr_sfb_spreaded_energy_long;
  WORD32 sfb, sfb_offsets;
  WORD32 *ptr_num_sfb = pstr_sfb_prms->num_sfb;
  WORD32 *ptr_num_window_groups = pstr_sfb_prms->num_window_groups;
  const WORD32 sfb_count = ptr_num_window_groups[chn] * ptr_num_sfb[chn];
  const WORD32 sfb_per_group = ptr_num_sfb[chn];
  WORD32 grp = 0, i, zero_flag;
  const WORD32 sfb_per_pred_band = 2;
  FLOAT32 min_thr_1, min_thr_2;
  FLOAT32 temp_1 = 0, temp_2 = 0;
  ia_ms_info_struct *pstr_ms_info = &pstr_usac_data->str_ms_info[chn];

  if (pstr_usac_data->str_igf_config[ele_id].igf_use_enf == 1)
  {
    pstr_usac_data->complex_coef[chn] = 0;
  }
  else
  {
    FLOAT64 ipd;
    /* Compute IPD between L and R channels */
    ipd = impeghe_compute_ipd(&pstr_usac_data->spectral_line_vector[chn][0],
                              &pstr_usac_data->mdst_spectrum[chn][0],
                              &pstr_usac_data->spectral_line_vector[chn + 1][0],
                              &pstr_usac_data->mdst_spectrum[chn + 1][0], pstr_usac_config->ccfl);

    /* Decide value of complex_coef based on IPD */
    if ((ipd > (PI / 2 - 5 * PI / 180) && ipd < (PI / 2 + 5 * PI / 180)) ||
        (ipd > (3 * PI / 2 - 5 * PI / 180) && ipd < (3 * PI / 2 + 5 * PI / 180)))
    {
      pstr_usac_data->complex_coef[chn] = 1;
    }
    else
    {
      pstr_usac_data->complex_coef[chn] = 0;
    }
  }

  /* Compute M and S spectra */
  for (i = 0; i < pstr_usac_config->ccfl; i++)
  {
    pstr_usac_data->spectral_line_vector[chn][i] = ptr_ms_spec[i];
    pstr_usac_data->spectral_line_vector[chn + 1][i] = ptr_ms_spec[FRAME_LEN_LONG + i];
  }

  impeghe_cplx_pred_main(
      pstr_sfb_prms->num_sfb[chn], pstr_sfb_prms->num_window_groups[chn],
      pstr_usac_data->spectral_line_vector[chn], pstr_usac_data->spectral_line_vector[chn + 1],
      pstr_usac_data->pred_coef_re[chn], pstr_usac_data->pred_coef_im[chn],
      &pstr_usac_data->pred_dir_idx[chn], pstr_usac_data, pstr_sfb_prms, usac_independancy_flag,
      pstr_usac_config, ptr_scratch_cmpx_mdct_buf, pstr_usac_data->cplx_pred_used[chn], chn,
      ptr_sfb_offsets, nrg_mid, nrg_side, &pstr_ms_info->ms_mask);

  if (pstr_ms_info->ms_mask == 3)
  {
    /* Compute thresholds required for quantization (similar to that in MS coding) */
    for (sfb = 0; sfb < sfb_count; sfb += sfb_per_group, grp++)
    {
      for (sfb_offsets = 0; sfb_offsets < sfb_per_group; sfb_offsets += sfb_per_pred_band)
      {
        if (pstr_usac_data->cplx_pred_used[chn][grp][sfb_offsets] == 1)
        {
          zero_flag = (ptr_sfb_offsets[sfb + sfb_offsets + 1] != FRAME_LEN_LONG);
          min_thr_1 =
              min(ptr_sfb_thr_left[sfb + sfb_offsets], ptr_sfb_thr_right[sfb + sfb_offsets]);
          if (zero_flag)
          {
            min_thr_2 = min(ptr_sfb_thr_left[sfb + sfb_offsets + 1],
                            ptr_sfb_thr_right[sfb + sfb_offsets + 1]);
          }

          ptr_sfb_thr_left[sfb + sfb_offsets] = ptr_sfb_thr_right[sfb + sfb_offsets] = min_thr_1;
          ptr_sfb_enegry_left[sfb + sfb_offsets] = ptr_sfb_energy_mid[sfb + sfb_offsets];
          ptr_sfb_energy_right[sfb + sfb_offsets] = ptr_sfb_energy_side[sfb + sfb_offsets];
          if (zero_flag)
          {
            ptr_sfb_thr_left[sfb + sfb_offsets + 1] = ptr_sfb_thr_right[sfb + sfb_offsets + 1] =
                min_thr_2;
            ptr_sfb_enegry_left[sfb + sfb_offsets + 1] =
                ptr_sfb_energy_mid[sfb + sfb_offsets + 1];
            ptr_sfb_energy_right[sfb + sfb_offsets + 1] =
                ptr_sfb_energy_side[sfb + sfb_offsets + 1];
          }
          ptr_sfb_spread_energy_left[sfb + sfb_offsets] =
              ptr_sfb_spread_energy_right[sfb + sfb_offsets] =
                  min(ptr_sfb_spread_energy_left[sfb + sfb_offsets],
                      ptr_sfb_spread_energy_right[sfb + sfb_offsets]) *
                  0.5f;
          if (zero_flag)
          {
            ptr_sfb_spread_energy_left[sfb + sfb_offsets + 1] =
                ptr_sfb_spread_energy_right[sfb + sfb_offsets + 1] =
                    min(ptr_sfb_spread_energy_left[sfb + sfb_offsets + 1],
                        ptr_sfb_spread_energy_right[sfb + sfb_offsets + 1]) *
                    0.5f;
          }
        }
      }
    }

    if (pstr_usac_data->pred_dir_idx[chn] == 1)
    {
      grp = 0;

      for (sfb = 0; sfb < sfb_count; sfb += sfb_per_group, grp++)
      {
        for (sfb_offsets = 0; sfb_offsets < sfb_per_group; sfb_offsets += sfb_per_pred_band)
        {
          zero_flag = (ptr_sfb_offsets[sfb + sfb_offsets + 1] != FRAME_LEN_LONG);

          if (pstr_usac_data->cplx_pred_used[chn][grp][sfb_offsets] == 1)
          {
            temp_1 = ptr_sfb_enegry_left[sfb + sfb_offsets];
            ptr_sfb_enegry_left[sfb + sfb_offsets] = ptr_sfb_energy_right[sfb + sfb_offsets];
            ptr_sfb_energy_right[sfb + sfb_offsets] = temp_1;
            if (zero_flag)
            {
              temp_2 = ptr_sfb_enegry_left[sfb + sfb_offsets + 1];
              ptr_sfb_enegry_left[sfb + sfb_offsets + 1] =
                  ptr_sfb_energy_right[sfb + sfb_offsets + 1];
              ptr_sfb_energy_right[sfb + sfb_offsets + 1] = temp_2;
            }
          }
        }
      }
    }
  }
  return;
}
