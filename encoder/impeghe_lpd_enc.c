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

#include "impeghe_psy_utils.h"
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
 *  impeghe_reset_td_enc
 *
 *  \brief Resets LDP(TD) encoder state structure
 *
 *  \param [in] pstr_td_encoder Pointer to LPD encoder state structure
 *
 *  \return VOID
 */
static VOID impeghe_reset_td_enc(ia_usac_td_encoder_struct *pstr_td_encoder)
{
  WORD32 i;

  memset(pstr_td_encoder->old_speech_pe, 0, (ORDER + LEN_NEXT_HIGH_RATE) * sizeof(FLOAT32));
  memset(pstr_td_encoder->prev_exc, 0, (MAX_PITCH + LEN_INTERPOL) * sizeof(FLOAT32));
  memset(pstr_td_encoder->prev_wsp, 0, (MAX_PITCH / OPL_DECIM) * sizeof(FLOAT32));
  memset(pstr_td_encoder->mem_lp_decim2, 0, 3 * sizeof(FLOAT32));
  memset(pstr_td_encoder->weighted_sig, 0, 128 * sizeof(FLOAT32));

  pstr_td_encoder->lpd_state.mode = -1;
  pstr_td_encoder->lpd_state.num_bits = 0;
  memset(pstr_td_encoder->lpd_state.lpc_coeffs_quant, 0, (2 * (ORDER + 1)) * sizeof(FLOAT32));
  memset(pstr_td_encoder->lpd_state.lpc_coeffs, 0, (2 * (ORDER + 1)) * sizeof(FLOAT32));
  memset(pstr_td_encoder->lpd_state.synth, 0, (ORDER + 128) * sizeof(FLOAT32));
  memset(pstr_td_encoder->lpd_state.wsynth, 0, (1 + 128) * sizeof(FLOAT32));

  memset(pstr_td_encoder->lpd_state.acelp_exc, 0, (2 * LEN_FRAME) * sizeof(FLOAT32));

  memset(pstr_td_encoder->lpd_state.tcx_mem, 0, 128 * sizeof(FLOAT32));
  memset(pstr_td_encoder->lpd_state.tcx_quant, 0, (1 + 256) * sizeof(FLOAT32));
  pstr_td_encoder->lpd_state.tcx_fac = 0.0f;

  memset(pstr_td_encoder->prev_hp_wsp, 0,
         (FRAME_LEN_LONG / OPL_DECIM + (MAX_PITCH / OPL_DECIM)) * sizeof(FLOAT32));

  memset(pstr_td_encoder->hp_ol_ltp_mem, 0, (3 * 2 + 1) * sizeof(FLOAT32));
  for (i = 0; i < 5; i++)
  {
    pstr_td_encoder->prev_ol_lags[i] = 40;
  }
  pstr_td_encoder->prev_wsyn_mem = 0.0;
  pstr_td_encoder->prev_wsp_mem = 0.0;
  pstr_td_encoder->prev_xnq_mem = 0.0;
  pstr_td_encoder->mem_wsp = 0.0;
  pstr_td_encoder->prev_ovlp_size = 0;
  pstr_td_encoder->prev_pitch_med = 40;
  pstr_td_encoder->ol_wght_flg = 0;
  pstr_td_encoder->ada_w = 0.0;

  memcpy(pstr_td_encoder->isf_old, impeghe_lsf_init, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isp_old, impeghe_ispold_init, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isp_old_q, pstr_td_encoder->isp_old, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isf_flpd_old, impeghe_lsf_init, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isp_flpd_old, impeghe_ispold_init, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isp_flpd_old_q, pstr_td_encoder->isp_flpd_old, ORDER * sizeof(FLOAT32));

  pstr_td_encoder->mem_preemph = 0.0;
  memset(pstr_td_encoder->mem_sig_in, 0, 4 * sizeof(FLOAT32));
  memset(pstr_td_encoder->xn_buffer, 0, 128 * sizeof(FLOAT32));

  return;
}

/**
 *  impeghe_highpass_prev_wsp
 *
 *  \brief Brief description
 *
 *  \param [in,out] pstr_td_encoder     Pointer to LPD state structure.
 *  \param [in] decim_sig  Pointer to decimated signal data.
 *  \param [in] pitch_max  Max pitch value.
 *
 *  \return VOID
 */
static VOID impeghe_highpass_prev_wsp(ia_usac_td_encoder_struct *pstr_td_encoder,
                                      FLOAT32 *decim_sig, WORD32 pitch_max)
{
  LOOPIDX i, j;
  WORD32 wsp_offset, num_frames;
  FLOAT32 val;
  FLOAT32 *hp_wsp_mem, *prev_hp_wsp, *wsp;
  FLOAT32 *data_a, *data_b, *hp_wsp;
  wsp_offset = pitch_max / OPL_DECIM;
  num_frames = (2 * LEN_SUBFR) / OPL_DECIM;
  hp_wsp_mem = pstr_td_encoder->hp_ol_ltp_mem;
  prev_hp_wsp = pstr_td_encoder->prev_hp_wsp;
  for (i = 0; i < 2 * pstr_td_encoder->len_subfrm / OPL_DECIM; i += num_frames)
  {
    wsp = decim_sig + ORDER + i;
    data_a = hp_wsp_mem;
    data_b = hp_wsp_mem + HP_ORDER;
    hp_wsp = prev_hp_wsp + wsp_offset;
    for (j = 0; j < num_frames; j++)
    {
      data_b[0] = data_b[1];
      data_b[1] = data_b[2];
      data_b[2] = data_b[3];
      data_b[HP_ORDER] = wsp[j];
      val = data_b[0] * 0.83787057505665F;
      val += data_b[1] * -2.50975570071058F;
      val += data_b[2] * 2.50975570071058F;
      val += data_b[3] * -0.83787057505665F;
      val -= data_a[0] * -2.64436711600664F;
      val -= data_a[1] * 2.35087386625360F;
      val -= data_a[2] * -0.70001156927424F;
      data_a[2] = data_a[1];
      data_a[1] = data_a[0];
      data_a[0] = val;
      hp_wsp[j] = val;
    }
    memmove(prev_hp_wsp, &prev_hp_wsp[num_frames], wsp_offset * sizeof(FLOAT32));
  }
}

/**
 *  impeghe_find_weighted_speech
 *
 *  \brief Calculates weighted speech data.
 *
 *  \param [in ] filter_coef  Pointer to linear prediction filter coeffs.
 *  \param [in ] speech       Pointer to input speech data.
 *  \param [out] wsp          Pointer to output wighted speech data.
 *  \param [in,out] mem_wsp   Pointer to weighted speech filter states.
 *  \param [in ] length       Length of input data.
 *
 *  \return VOID
 */
static VOID impeghe_find_weighted_speech(FLOAT32 *filter_coef, FLOAT32 *speech, FLOAT32 *wsp,
                                         FLOAT32 *mem_wsp, WORD32 length)
{
  LOOPIDX i_subfr;
  FLOAT32 weighted_lpc[ORDER + 1];
  for (i_subfr = 0; i_subfr < length; i_subfr += LEN_SUBFR)
  {
    impeghe_get_weighted_lpc(filter_coef, weighted_lpc);
    impeghe_compute_lp_residual(weighted_lpc, &speech[i_subfr], &wsp[i_subfr], LEN_SUBFR);
    filter_coef += (ORDER + 1);
  }
  impeghe_apply_deemph(wsp, TILT_FAC, length, mem_wsp);
  return;
}

/**
 *  impeghe_get_interpolated_lpc
 *
 *  \brief Interpolates LSP values for the intermediate sub frames.
 *         Calculates LPC values from interpolated LSPs for all sub frames.
 *
 *  \param [in ] lsp_old    Pointer to previous LSP values.
 *  \param [in ] lsp_new    Pointer to new LSP values.
 *  \param [out] lpc        Pointer to LPC values.
 *  \param [in ] num_subfrm Number of subframes.
 *
 *  \return VOID
 */
static VOID impeghe_get_interpolated_lpc(FLOAT32 *lsp_old, FLOAT32 *lsp_new, FLOAT32 *lpc,
                                         WORD32 num_subfrm)
{
  LOOPIDX i, j;
  FLOAT32 inc, fnew, fold;
  FLOAT32 lsp[ORDER];
  FLOAT32 *p_lpc;

  inc = 1.0f / (FLOAT32)num_subfrm;
  fnew = 0.0f;
  p_lpc = lpc;

  for (i = 0; i < num_subfrm; i++)
  {
    fold = 1.0f - fnew;
    for (j = 0; j < ORDER; j++)
    {
      lsp[j] = (FLOAT32)(lsp_old[j] * fold + lsp_new[j] * fnew);
    }
    fnew += inc;
    impeghe_lsp_to_lp_conversion(lsp, p_lpc);
    p_lpc += (ORDER + 1);
  }

  impeghe_lsp_to_lp_conversion(lsp_new, p_lpc);
}

/**
 *  impeghe_core_lpd_encode
 *
 *  \brief Core LPD encoder function.
 *
 *  \param [in,out] pstr_usac_data          Pointer to USAC state structure.
 *  \param [in,out] pstr_usac_config        Pointer to USAC config structure.
 *  \param [in]     speech             Pointer to input speech data buffer.
 *  \param [in,out] mode               Pointer to encoding mode variable.
 *  \param [in,out] num_tcx_param      Pointer to number of TCX BS params.
 *  \param [in] ptr_sfb_offset         Pointer to Scalefactor band offsets.
 *  \param [in] usac_independence_flag USAC independency flag value.
 *  \param [in] num_sfb                Number of scale factor bands value.
 *  \param [in] ch_idx                 Channel index.
 *  \param [in] chn                    Channel number.
 *  \param [in] ele_id                 Syntactic element ID.
 *
 *  \return WORD32
 */
static VOID impeghe_core_lpd_encode(ia_usac_data_struct *pstr_usac_data,
                                    ia_usac_encoder_config_struct *pstr_usac_config,
                                    FLOAT32 *speech, WORD32 *mode, WORD32 *num_tcx_param,
                                    WORD32 *ptr_sfb_offset, WORD32 usac_independence_flag,
                                    WORD32 num_sfb, WORD32 ch_idx, WORD32 chn, WORD32 ele_id)
{
  LOOPIDX i, j, k, i1, i2;
  WORD32 first_lpd_flag, pit_adj;
  WORD32 num_indices, num_bits;
  WORD32 num_params, pitch_min, pitch_max;
  WORD32 num_bits_acelp, num_bits_tcx;
  WORD32 range_pitch_search;
  WORD32 len_subfrm, num_subfrm, num_acelp_frames;
  WORD32 num_sbfrm_per_supfrm, window_len, len;
  WORD32 num_indices_flpd, num_bits_flpd;
  WORD32 ol_pitch_lag[2 * NUM_FRAMES] = {0};
  WORD32 *p_params;
  WORD32 *num_fac_bits;
  WORD32 *lpc_params;
  WORD32 *lpc_params_flpd;
  WORD32 *acelp_tcx_params_flpd;
  WORD32 *acelp_tcx_params;
  WORD32 *prm_tcx;
  WORD16 *serial_fac_out;
  WORD16 *codec_mode;
  pUWORD8 ptr_lpd_scratch;
  FLOAT32 mem_wsyn;
  FLOAT32 ssnr_256, ssnr_512, ssnr_1024, tmp_ssnr = 0.0f;
  FLOAT32 energy, max_corr, t0;
  FLOAT32 auto_corr_vector[ORDER + 1] = {0};
  FLOAT32 auto_corr_lp_filter_coeff[ORDER + 1] = {0};
  FLOAT32 isp_new[ORDER] = {0};
  FLOAT32 norm_corr[2 * NUM_FRAMES] = {0};
  FLOAT32 *p, *p1;
  FLOAT32 *flpd_wsp_prev_buf;
  FLOAT32 *lp_filter_coeff;
  FLOAT32 *lp_filter_coeff_q;
  FLOAT32 *lp_filter_coeff_flpd;
  FLOAT32 *lp_filter_coeff_flpd_q;
  FLOAT32 *isp_curr;
  FLOAT32 *isp_curr_q;
  FLOAT32 *isf_curr;
  FLOAT32 *isf_curr_q;
  FLOAT32 *p_wsp_prev_buf;
  FLOAT32 *p_flpd_wsp_prev_buf = NULL;
  FLOAT32 *wsp_prev_buf;
  FLOAT32 *isp_curr_flpd;
  FLOAT32 *isp_curr_flpd_q;
  FLOAT32 *isf_curr_flpd;
  FLOAT32 *isf_curr_flpd_q;
  FLOAT32 *ptr_speech_buf;
  const FLOAT32 *lp_analysis_window;

  ia_usac_lpd_scratch *pstr_lpd_scratch;
  ia_usac_lpd_state_struct lpd_state_temp = {0};
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  ia_usac_td_encoder_struct *pstr_td_encoder = pstr_usac_data->td_encoder[ch_idx];

  lp_filter_coeff = pstr_scratch->p_lp_filter_coeff;
  lp_filter_coeff_q = pstr_scratch->p_lp_filter_coeff_q;
  prm_tcx = pstr_scratch->p_prm_tcx;
  wsp_prev_buf = pstr_scratch->p_wsp_prev_buf;
  ptr_lpd_scratch = pstr_scratch->ptr_lpd_scratch;
  lp_filter_coeff_flpd = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1) * sizeof(lp_filter_coeff_flpd[0]);
  lp_filter_coeff_flpd_q = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1) * sizeof(lp_filter_coeff_flpd_q[0]);
  isp_curr = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_FRAMES + 1) * ORDER * sizeof(isp_curr[0]);
  isp_curr_q = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_FRAMES + 1) * ORDER * sizeof(isp_curr_q[0]);
  isf_curr = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_FRAMES + 1) * ORDER * sizeof(isf_curr[0]);
  isf_curr_q = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += (NUM_FRAMES + 1) * ORDER * sizeof(isf_curr_q[0]);
  isp_curr_flpd = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isp_curr_flpd[0]);
  isp_curr_flpd_q = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isp_curr_flpd_q[0]);
  isf_curr_flpd = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isf_curr_flpd[0]);
  isf_curr_flpd_q = (FLOAT32 *)ptr_lpd_scratch;
  ptr_lpd_scratch += ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isf_curr_flpd_q[0]);
  pstr_lpd_scratch = (ia_usac_lpd_scratch *)ptr_lpd_scratch;
  ptr_lpd_scratch += 6 * sizeof(ia_usac_lpd_scratch);

  num_bits_acelp = 0;
  num_bits_tcx = 0;
  range_pitch_search = 0;
  len_subfrm = 0;
  num_subfrm = 0;
  num_sbfrm_per_supfrm = 0;
  window_len = 0;
  num_indices_flpd = 0;
  num_bits_flpd = 0;
  num_indices = 0;
  num_bits = 0;
  ssnr_256 = 0.0f;
  ssnr_512 = 0.0f;
  ssnr_1024 = 0.0f;
  tmp_ssnr = 0.0f;
  energy = 0;
  max_corr = 0;
  t0 = 0;
  num_acelp_frames = NUM_FRAMES;
  len = (MAX_PITCH / OPL_DECIM);
  first_lpd_flag = (pstr_usac_data->core_mode_prev[ch_idx] == CORE_MODE_FD);
  pit_adj = pstr_usac_data->td_encoder[ch_idx]->fscale;
  len_subfrm = pstr_td_encoder->len_subfrm;
  num_subfrm = pstr_td_encoder->num_subfrm;
  num_sbfrm_per_supfrm = NUM_FRAMES * num_subfrm;

  num_fac_bits = &pstr_usac_data->num_td_fac_bits[ch_idx];
  lpc_params = pstr_usac_data->param_buf + (NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV);
  lpc_params_flpd = pstr_usac_data->param_buf_flpd + (NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV);
  acelp_tcx_params_flpd = pstr_usac_data->param_buf_flpd;
  acelp_tcx_params = pstr_usac_data->param_buf;
  serial_fac_out = pstr_usac_data->fac_out_stream[ch_idx];
  flpd_wsp_prev_buf = pstr_usac_data->p_flpd_wsp_prev_buf;
  lp_analysis_window = pstr_td_encoder->lp_analysis_window;
  codec_mode = &pstr_td_encoder->acelp_core_mode;

  memset(isp_curr, 0, (NUM_FRAMES + 1) * ORDER * sizeof(isp_curr[0]));
  memset(isp_curr_q, 0, (NUM_FRAMES + 1) * ORDER * sizeof(isp_curr_q[0]));
  memset(isf_curr, 0, (NUM_FRAMES + 1) * ORDER * sizeof(isf_curr[0]));
  memset(isf_curr_q, 0, (NUM_FRAMES + 1) * ORDER * sizeof(isf_curr_q[0]));
  memset(lp_filter_coeff, 0, ((NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1)) * sizeof(FLOAT32));
  memset(lp_filter_coeff_q, 0, ((NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1)) * sizeof(FLOAT32));
  memset(wsp_prev_buf, 0, ((MAX_PITCH1 / OPL_DECIM) + LEN_FRAME) * sizeof(FLOAT32));
  memset(isp_curr_flpd, 0, ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isp_curr_flpd[0]));
  memset(isp_curr_flpd_q, 0, ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isp_curr_flpd_q[0]));
  memset(isf_curr_flpd, 0, ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isf_curr_flpd[0]));
  memset(isf_curr_flpd_q, 0, ((NUM_FRAMES >> 1) + 1) * ORDER * sizeof(isf_curr_flpd_q[0]));
  memset(lp_filter_coeff_flpd, 0,
         (NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1) * sizeof(lp_filter_coeff_flpd[0]));
  memset(lp_filter_coeff_flpd_q, 0,
         (NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1) * sizeof(lp_filter_coeff_flpd_q[0]));
  memset(pstr_lpd_scratch, 0, 6 * sizeof(ia_usac_lpd_scratch));

  num_bits_acelp = (impeghe_acelp_core_numbits_1024[*codec_mode] - NBITS_MODE) >> 2;
  num_bits_tcx = (WORD32)(0.85f * num_bits_acelp - NBITS_LPC);
  if (pit_adj > FSCALE_DENOM)
  {
    window_len = (LEN_LP_WINDOW_HIGH_RATE * len_subfrm) / LEN_FRAME;
  }
  else
  {
    window_len = (LEN_LP_WINDOW * len_subfrm) / LEN_FRAME;
  }

  if (pit_adj != 0)
  {
    if (pstr_usac_config->full_band_lpd)
    {
      i = ((((pit_adj / 2) * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN;
    }
    else
    {
      i = (((pit_adj * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN;
    }
    pitch_min = TMIN + i;
    pitch_max = TMAX + (6 * i);
  }
  else
  {
    pitch_min = TMIN;
    pitch_max = TMAX;
  }

  memcpy(pstr_scratch->p_wsig_buf, pstr_td_encoder->weighted_sig, 128 * sizeof(FLOAT32));
  p_wsp_prev_buf = wsp_prev_buf + MAX_PITCH1 / OPL_DECIM;
  memcpy(wsp_prev_buf, pstr_td_encoder->prev_wsp,
         (WORD32)((MAX_PITCH / OPL_DECIM) * sizeof(FLOAT32)));

  if (pstr_usac_config->full_band_lpd)
  {
    p_flpd_wsp_prev_buf = flpd_wsp_prev_buf + ((MAX_PITCH1 >> 1) / OPL_DECIM);
    memcpy(flpd_wsp_prev_buf, pstr_td_encoder->prev_flpd_wsp,
           (WORD32)(((MAX_PITCH1 >> 1) / OPL_DECIM) * sizeof(FLOAT32)));
    window_len = window_len >> 1;
  }

  if (pstr_usac_config->full_band_lpd)
  {
    num_acelp_frames = num_acelp_frames >> 1;
    impeghe_tbe_resample_input(&pstr_td_encoder->flpd_speech_buf[0], &speech[0],
                               &pstr_td_encoder->flpd_resamp_filt_mem[0],
                               &pstr_td_encoder->scratch_mem[0], FRAME_LEN_LONG);
  }

  if (first_lpd_flag)
  {
    memcpy(pstr_td_encoder->isp_old, impeghe_ispold_init, ORDER * sizeof(FLOAT32));

    impeghe_autocorr_plus(&speech[-(window_len / 2)], auto_corr_vector, window_len,
                          (FLOAT32 *)lp_analysis_window, pstr_scratch->p_buf_aut_corr);

    for (j = 0; j <= ORDER; j++)
    {
      auto_corr_vector[j] *= (FLOAT32)impeghe_lag_window[j];
    }

    impeghe_levinson_durbin_algo(auto_corr_vector, auto_corr_lp_filter_coeff);
    impeghe_lpc_2_lsp_conversion(auto_corr_lp_filter_coeff, isp_new, pstr_td_encoder->isp_old);
    memcpy(pstr_td_encoder->isp_old, isp_new, ORDER * sizeof(FLOAT32));
    impeghe_lsp_2_lsf_conversion(isp_new, isf_curr);
    memcpy(pstr_td_encoder->isf_old, isf_curr, ORDER * sizeof(FLOAT32));
  }

  memcpy(isp_curr, pstr_td_encoder->isp_old, ORDER * sizeof(FLOAT32));

  if (pstr_usac_config->full_band_lpd)
  {
    memcpy(isp_curr_flpd, pstr_td_encoder->isp_flpd_old, ORDER * sizeof(FLOAT32));
  }

  for (i = 0; i < num_subfrm; i++)
  {
    impeghe_autocorr_plus(&speech[((i + 1) * len_subfrm) - (window_len / 2)], auto_corr_vector,
                          window_len, (FLOAT32 *)lp_analysis_window,
                          pstr_scratch->p_buf_aut_corr);

    for (j = 0; j <= ORDER; j++)
    {
      auto_corr_vector[j] *= (FLOAT32)impeghe_lag_window[j];
    }

    impeghe_levinson_durbin_algo(auto_corr_vector, auto_corr_lp_filter_coeff);
    impeghe_lpc_2_lsp_conversion(auto_corr_lp_filter_coeff, isp_new, pstr_td_encoder->isp_old);
    memcpy(&isp_curr[(i + 1) * ORDER], isp_new, ORDER * sizeof(FLOAT32));
    impeghe_interpolation_lsp_params(pstr_td_encoder->isp_old, isp_new,
                                     &lp_filter_coeff[i * num_subfrm * (ORDER + 1)], num_subfrm);
    impeghe_lsp_2_lsf_conversion(&isp_curr[(i + 1) * ORDER], &isf_curr[(i + 1) * ORDER]);
    memcpy(pstr_td_encoder->isp_old, isp_new, ORDER * sizeof(FLOAT32));
  }

  if (pstr_usac_config->full_band_lpd)
  {
    ptr_speech_buf = &pstr_td_encoder->flpd_speech_buf[0];
    for (i = 0; i < num_acelp_frames; i++)
    {
      impeghe_autocorr_plus(&ptr_speech_buf[((i + 1) * len_subfrm) - (window_len / 2)],
                            auto_corr_vector, window_len, (FLOAT32 *)lp_analysis_window,
                            pstr_scratch->p_buf_aut_corr);

      for (j = 0; j <= ORDER; j++)
      {
        auto_corr_vector[j] *= (FLOAT32)impeghe_lag_window[j];
      }

      impeghe_levinson_durbin_algo(auto_corr_vector, auto_corr_lp_filter_coeff);
      impeghe_lpc_2_lsp_conversion(auto_corr_lp_filter_coeff, isp_new,
                                   pstr_td_encoder->isp_flpd_old);
      memcpy(&isp_curr_flpd[(i + 1) * ORDER], isp_new, ORDER * sizeof(FLOAT32));
      impeghe_interpolation_lsp_params(pstr_td_encoder->isp_flpd_old, isp_new,
                                       &lp_filter_coeff_flpd[i * num_subfrm * (ORDER + 1)],
                                       num_subfrm);
      impeghe_lsp_2_lsf_conversion(&isp_curr_flpd[(i + 1) * ORDER],
                                   &isf_curr_flpd[(i + 1) * ORDER]);
      memcpy(pstr_td_encoder->isp_flpd_old, isp_new, ORDER * sizeof(FLOAT32));
    }
  }

  memcpy(isf_curr, pstr_td_encoder->isf_old, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isf_old, &isf_curr[num_subfrm * ORDER], ORDER * sizeof(FLOAT32));

  if (pstr_usac_config->full_band_lpd)
  {
    memcpy(isf_curr_flpd, pstr_td_encoder->isf_flpd_old, ORDER * sizeof(FLOAT32));
    memcpy(pstr_td_encoder->isf_flpd_old, &isf_curr_flpd[num_acelp_frames * ORDER],
           ORDER * sizeof(FLOAT32));
  }

  if (!first_lpd_flag)
  {
    impeghe_lsp_2_lsf_conversion(pstr_td_encoder->isp_old_q, isf_curr_q);
    if (pstr_usac_config->full_band_lpd)
    {
      impeghe_lsp_2_lsf_conversion(pstr_td_encoder->isp_flpd_old_q, isf_curr_flpd_q);
    }
  }

  impeghe_quantize_lpc_avq(&isf_curr[ORDER], &isf_curr_q[ORDER], first_lpd_flag, &lpc_params[0],
                           &num_indices, &num_bits, ptr_lpd_scratch);

  for (i = 0; i < num_subfrm; i++)
  {
    impeghe_lsf_2_lsp_conversion(&isf_curr_q[(i + 1) * ORDER], &isp_curr_q[(i + 1) * ORDER]);
  }

  if (pstr_usac_config->full_band_lpd)
  {
    impeghe_quantize_flpd_lpc_avq(&isf_curr_flpd[ORDER], &isf_curr_flpd_q[ORDER], first_lpd_flag,
                                  &lpc_params_flpd[0], &num_indices_flpd, &num_bits_flpd,
                                  ptr_lpd_scratch);
    for (i = 0; i < num_acelp_frames; i++)
    {
      impeghe_lsf_2_lsp_conversion(&isf_curr_flpd_q[(i + 1) * ORDER],
                                   &isp_curr_flpd_q[(i + 1) * ORDER]);
    }
  }

  *num_fac_bits = 0;
  if (first_lpd_flag)
  {
    WORD32 fac_length;
    WORD32 num_bits_fac = (WORD32)((FLOAT32)num_bits_tcx / 2.f);
    FLOAT32 premph_mem = 0.0f;
    FLOAT32 lpc[9 * (ORDER + 1)];
    FLOAT32 hp_mem[4];
    FLOAT32 *temp_speech = pstr_scratch->p_buf_speech;
    FLOAT32 *temp_res = pstr_scratch->p_buf_res;
    FLOAT32 *temp_signal = pstr_scratch->p_buf_signal;

    if (pstr_usac_config->window_sequence_prev[ch_idx] == EIGHT_SHORT_SEQUENCE)
    {
      fac_length = pstr_td_encoder->len_frame >> 4;
    }
    else
    {
      fac_length = pstr_td_encoder->len_frame >> 3;
    }

    impeghe_lsf_2_lsp_conversion(isf_curr_q, isp_curr_q);
    memcpy(pstr_td_encoder->isp_old_q, isp_curr_q, ORDER * sizeof(FLOAT32));
    if (pstr_usac_config->full_band_lpd)
    {
      impeghe_lsf_2_lsp_conversion(isf_curr_flpd_q, isp_curr_flpd_q);
      memcpy(pstr_td_encoder->isp_flpd_old_q, isp_curr_flpd_q, ORDER * sizeof(FLOAT32));
    }

    impeghe_get_interpolated_lpc(pstr_td_encoder->isp_old_q, pstr_td_encoder->isp_old_q, lpc,
                                 (2 * len_subfrm) / LEN_SUBFR);

    memset(temp_speech, 0, (ORDER + 2 * len_subfrm) * sizeof(FLOAT32));
    memset(temp_res, 0, (2 * len_subfrm) * sizeof(FLOAT32));

    impeghe_fac_apply(&pstr_td_encoder->fd_orig[1 + ORDER], len_subfrm, fac_length,
                      pstr_td_encoder->low_pass_line, num_bits_fac,
                      &pstr_td_encoder->fd_synth[1 + ORDER], lpc, serial_fac_out, num_fac_bits,
                      pstr_scratch);
    memset(hp_mem, 0, 4 * sizeof(FLOAT32));
    impeghe_highpass_50hz_12k8(pstr_td_encoder->fd_orig, 2 * len_subfrm + 1 + ORDER, hp_mem,
                               pit_adj);
    premph_mem = 0.0f;
    impeghe_apply_preemph(pstr_td_encoder->fd_orig, PREEMPH_FILT_FAC, 2 * len_subfrm + 1 + ORDER,
                          &premph_mem);

    memcpy(temp_signal, pstr_td_encoder->fd_orig + len_subfrm + 1,
           (len_subfrm + ORDER) * sizeof(FLOAT32));
    premph_mem = temp_signal[0];
    impeghe_apply_deemph(temp_signal, PREEMPH_FILT_FAC, len_subfrm + ORDER, &premph_mem);
    memcpy(pstr_td_encoder->lpd_state.tcx_mem, &temp_signal[len_subfrm + ORDER - 128],
           128 * sizeof(FLOAT32));

    premph_mem = 0.0f;
    impeghe_apply_preemph(pstr_td_encoder->fd_synth, PREEMPH_FILT_FAC, 2 * len_subfrm + 1 + ORDER,
                          &premph_mem);
    memcpy(pstr_td_encoder->lpd_state.synth,
           pstr_td_encoder->fd_synth + 2 * len_subfrm - ORDER - 128 + 1 + ORDER,
           (ORDER + 128) * sizeof(FLOAT32));
    memcpy(temp_speech + ORDER, pstr_td_encoder->fd_synth + 1 + ORDER,
           2 * len_subfrm * sizeof(FLOAT32));

    premph_mem = 0.0f;
    impeghe_find_weighted_speech(lpc, temp_speech + ORDER, temp_res, &premph_mem, 2 * len_subfrm);
    pstr_td_encoder->prev_wsyn_mem = premph_mem;
    memcpy(pstr_td_encoder->lpd_state.wsynth, temp_res + 2 * len_subfrm - ORDER - 128,
           (ORDER + 128) * sizeof(FLOAT32));
    memcpy(temp_speech + ORDER, pstr_td_encoder->fd_synth + 1 + ORDER,
           2 * len_subfrm * sizeof(FLOAT32));
    memset(temp_res, 0, 2 * len_subfrm * sizeof(FLOAT32));
    for (i = 0; i < 2 * len_subfrm; i += LEN_SUBFR)
    {
      impeghe_compute_lp_residual(lpc, &temp_speech[ORDER + i], &temp_res[i], LEN_SUBFR);
    }
    memcpy(pstr_td_encoder->lpd_state.acelp_exc, temp_res, 2 * len_subfrm * sizeof(FLOAT32));
    premph_mem = 0.0f;
    impeghe_find_weighted_speech(lp_filter_coeff, pstr_td_encoder->fd_orig + 1 + ORDER,
                                 temp_speech + ORDER, &(pstr_td_encoder->mem_wsp),
                                 2 * len_subfrm);
    memcpy(pstr_td_encoder->weighted_sig, temp_speech + ORDER + 2 * len_subfrm - 128,
           128 * sizeof(FLOAT32));
    memcpy(pstr_scratch->p_wsig_buf, pstr_td_encoder->weighted_sig, 128 * sizeof(FLOAT32));
    for (i = 0; i < 2 * len_subfrm; i += len_subfrm)
    {
      impeghe_decim2_fir_filter(&temp_speech[i + ORDER], len_subfrm,
                                pstr_td_encoder->mem_lp_decim2, pstr_scratch->p_fir_sig_buf);
      memcpy(temp_speech + ORDER + i / OPL_DECIM, temp_speech + ORDER + i,
             (len_subfrm / OPL_DECIM) * sizeof(FLOAT32));
    }
    memcpy(wsp_prev_buf, temp_speech + ORDER + 2 * len_subfrm / OPL_DECIM - MAX_PITCH / OPL_DECIM,
           (WORD32)((MAX_PITCH / OPL_DECIM) * sizeof(FLOAT32)));
    impeghe_highpass_prev_wsp(pstr_td_encoder, temp_speech, pitch_max);
  }
  memcpy(isp_curr_q, pstr_td_encoder->isp_old_q, ORDER * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->isp_old_q, &isp_curr_q[num_subfrm * ORDER], ORDER * sizeof(FLOAT32));

  if (pstr_usac_config->full_band_lpd)
  {
    memcpy(isp_curr_flpd_q, pstr_td_encoder->isp_flpd_old_q, ORDER * sizeof(FLOAT32));
    memcpy(pstr_td_encoder->isp_flpd_old_q, &isp_curr_flpd_q[num_acelp_frames * ORDER],
           ORDER * sizeof(FLOAT32));
  }

  for (i = 0; i < num_subfrm; i++)
  {
    impeghe_find_weighted_speech(
        &lp_filter_coeff[i * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)], &speech[i * len_subfrm],
        &pstr_scratch->p_wsig_buf[i * len_subfrm], &(pstr_td_encoder->mem_wsp), len_subfrm);
    memcpy(p_wsp_prev_buf, &pstr_scratch->p_wsig_buf[i * len_subfrm],
           len_subfrm * sizeof(FLOAT32));

    impeghe_decim2_fir_filter(p_wsp_prev_buf, len_subfrm, pstr_td_encoder->mem_lp_decim2,
                              pstr_scratch->p_fir_sig_buf);
    range_pitch_search = 2 * LEN_SUBFR;
    if (num_subfrm < 4 && !pstr_usac_config->full_band_lpd)
    {
      range_pitch_search = 3 * LEN_SUBFR;
    }

    impeghe_open_loop_search(p_wsp_prev_buf, (pitch_min / OPL_DECIM) + 1, pitch_max / OPL_DECIM,
                             range_pitch_search / OPL_DECIM, &ol_pitch_lag[i * 2],
                             pstr_td_encoder);

    if (pstr_td_encoder->ol_gain <= 0.6)
    {
      pstr_td_encoder->ada_w = pstr_td_encoder->ada_w * 0.9f;
    }
    else
    {
      pstr_td_encoder->prev_pitch_med =
          impeghe_get_ol_lag_median(ol_pitch_lag[i * 2], pstr_td_encoder->prev_ol_lags);
      pstr_td_encoder->ada_w = 1.0;
    }
    if (pstr_td_encoder->ada_w >= 0.8)
    {
      pstr_td_encoder->ol_wght_flg = 1;
    }
    else
    {
      pstr_td_encoder->ol_wght_flg = 0;
    }

    max_corr = 0.0f;
    p = &p_wsp_prev_buf[0];
    p1 = p_wsp_prev_buf - ol_pitch_lag[i * 2];
    for (j = 0; j < range_pitch_search / OPL_DECIM; j++)
    {
      max_corr += *p++ * *p1++;
    }

    t0 = 0.01f;
    p = p_wsp_prev_buf - ol_pitch_lag[i * 2];
    for (j = 0; j < range_pitch_search / OPL_DECIM; j++, p++)
    {
      t0 += *p * *p;
    }
    t0 = (FLOAT32)(1.0 / sqrt(t0));
    norm_corr[i * 2] = max_corr * t0;

    energy = 0.01f;
    for (j = 0; j < range_pitch_search / OPL_DECIM; j++)
    {
      energy += p_wsp_prev_buf[j] * p_wsp_prev_buf[j];
    }
    energy = (FLOAT32)(1.0 / sqrt(energy));
    norm_corr[i * 2] *= energy;

    if (num_subfrm < 4 && !pstr_usac_config->full_band_lpd)
    {
      ol_pitch_lag[(i * 2) + 1] = ol_pitch_lag[i * 2];
      norm_corr[(i * 2) + 1] = norm_corr[i * 2];
    }
    else
    {

      impeghe_open_loop_search(p_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM),
                               (pitch_min / OPL_DECIM) + 1, pitch_max / OPL_DECIM,
                               (2 * LEN_SUBFR) / OPL_DECIM, &ol_pitch_lag[(i * 2) + 1],
                               pstr_td_encoder);

      if (pstr_td_encoder->ol_gain <= 0.6)
      {
        pstr_td_encoder->ada_w = pstr_td_encoder->ada_w * 0.9f;
      }
      else
      {
        pstr_td_encoder->prev_pitch_med =
            impeghe_get_ol_lag_median(ol_pitch_lag[(i * 2) + 1], pstr_td_encoder->prev_ol_lags);
        pstr_td_encoder->ada_w = 1.0;
      }
      if (pstr_td_encoder->ada_w >= 0.8)
      {
        pstr_td_encoder->ol_wght_flg = 1;
      }
      else
      {
        pstr_td_encoder->ol_wght_flg = 0;
      }
      max_corr = 0.0f;
      p = p_wsp_prev_buf + (2 * LEN_SUBFR) / OPL_DECIM;
      p1 = p_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM) - ol_pitch_lag[(i * 2) + 1];
      for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++)
      {
        max_corr += *p++ * *p1++;
      }

      t0 = 0.01f;
      p = p_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM) - ol_pitch_lag[(i * 2) + 1];
      for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++, p++)
      {
        t0 += *p * *p;
      }
      t0 = (FLOAT32)(1.0 / sqrt(t0));
      norm_corr[(i * 2) + 1] = max_corr * t0;

      energy = 0.01f;
      for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++)
      {
        energy += p_wsp_prev_buf[((2 * LEN_SUBFR) / OPL_DECIM) + j] *
                  p_wsp_prev_buf[((2 * LEN_SUBFR) / OPL_DECIM) + j];
      }
      energy = (FLOAT32)(1.0 / sqrt(energy));
      norm_corr[(i * 2) + 1] *= energy;
    }

    memmove(wsp_prev_buf, &wsp_prev_buf[len_subfrm / OPL_DECIM],
            (WORD32)((MAX_PITCH / OPL_DECIM) * sizeof(FLOAT32)));
  }
  if (pstr_usac_config->full_band_lpd)
  {
    ptr_speech_buf = &pstr_td_encoder->flpd_speech_buf[0];
    for (i = 0; i < num_acelp_frames; i++)
    {
      impeghe_find_weighted_speech(
          &lp_filter_coeff_flpd[i * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
          &ptr_speech_buf[i * len_subfrm], &pstr_usac_data->p_flpd_wsig_buf[i * len_subfrm],
          &(pstr_td_encoder->mem_wsp), len_subfrm);
      memcpy(p_flpd_wsp_prev_buf, &pstr_usac_data->p_flpd_wsig_buf[i * len_subfrm],
             len_subfrm * sizeof(FLOAT32));

      impeghe_decim2_fir_filter(p_flpd_wsp_prev_buf, len_subfrm, pstr_td_encoder->mem_lp_decim2,
                                pstr_scratch->p_fir_sig_buf);
      range_pitch_search = 2 * LEN_SUBFR;

      impeghe_open_loop_search(p_wsp_prev_buf, (pitch_min / OPL_DECIM) + 1, pitch_max / OPL_DECIM,
                               range_pitch_search / OPL_DECIM, &ol_pitch_lag[i * 2],
                               pstr_td_encoder);

      if (pstr_td_encoder->ol_gain <= 0.6)
      {
        pstr_td_encoder->ada_w = pstr_td_encoder->ada_w * 0.9f;
      }
      else
      {
        pstr_td_encoder->prev_pitch_med =
            impeghe_get_ol_lag_median(ol_pitch_lag[i * 2], pstr_td_encoder->prev_ol_lags);
        pstr_td_encoder->ada_w = 1.0;
      }
      if (pstr_td_encoder->ada_w >= 0.8)
      {
        pstr_td_encoder->ol_wght_flg = 1;
      }
      else
      {
        pstr_td_encoder->ol_wght_flg = 0;
      }

      max_corr = 0.0f;
      p = &p_flpd_wsp_prev_buf[0];
      p1 = p_flpd_wsp_prev_buf - ol_pitch_lag[i * 2];
      for (j = 0; j < range_pitch_search / OPL_DECIM; j++)
      {
        max_corr += *p++ * *p1++;
      }

      t0 = 0.01f;
      p = p_flpd_wsp_prev_buf - ol_pitch_lag[i * 2];
      for (j = 0; j < range_pitch_search / OPL_DECIM; j++, p++)
      {
        t0 += *p * *p;
      }
      t0 = (FLOAT32)(1.0 / sqrt(t0));
      norm_corr[i * 2] = max_corr * t0;

      energy = 0.01f;
      for (j = 0; j < range_pitch_search / OPL_DECIM; j++)
      {
        energy += p_flpd_wsp_prev_buf[j] * p_flpd_wsp_prev_buf[j];
      }
      energy = (FLOAT32)(1.0 / sqrt(energy));
      norm_corr[i * 2] *= energy;
      {
        impeghe_open_loop_search(p_flpd_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM),
                                 (pitch_min / OPL_DECIM) + 1, pitch_max / OPL_DECIM,
                                 (2 * LEN_SUBFR) / OPL_DECIM, &ol_pitch_lag[(i * 2) + 1],
                                 pstr_td_encoder);

        if (pstr_td_encoder->ol_gain <= 0.6)
        {
          pstr_td_encoder->ada_w = pstr_td_encoder->ada_w * 0.9f;
        }
        else
        {
          pstr_td_encoder->prev_pitch_med =
              impeghe_get_ol_lag_median(ol_pitch_lag[(i * 2) + 1], pstr_td_encoder->prev_ol_lags);
          pstr_td_encoder->ada_w = 1.0;
        }
        if (pstr_td_encoder->ada_w >= 0.8)
        {
          pstr_td_encoder->ol_wght_flg = 1;
        }
        else
        {
          pstr_td_encoder->ol_wght_flg = 0;
        }
        max_corr = 0.0f;
        p = p_flpd_wsp_prev_buf + (2 * LEN_SUBFR) / OPL_DECIM;
        p1 = p_flpd_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM) - ol_pitch_lag[(i * 2) + 1];
        for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++)
        {
          max_corr += *p++ * *p1++;
        }

        t0 = 0.01f;
        p = p_flpd_wsp_prev_buf + ((2 * LEN_SUBFR) / OPL_DECIM) - ol_pitch_lag[(i * 2) + 1];
        for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++, p++)
        {
          t0 += *p * *p;
        }
        t0 = (FLOAT32)(1.0 / sqrt(t0));
        norm_corr[(i * 2) + 1] = max_corr * t0;

        energy = 0.01f;
        for (j = 0; j < (2 * LEN_SUBFR) / OPL_DECIM; j++)
        {
          energy += p_flpd_wsp_prev_buf[((2 * LEN_SUBFR) / OPL_DECIM) + j] *
                    p_flpd_wsp_prev_buf[((2 * LEN_SUBFR) / OPL_DECIM) + j];
        }
        energy = (FLOAT32)(1.0 / sqrt(energy));
        norm_corr[(i * 2) + 1] *= energy;
      }

      memmove(flpd_wsp_prev_buf, &flpd_wsp_prev_buf[len_subfrm / OPL_DECIM],
              (WORD32)((MAX_PITCH / OPL_DECIM) * sizeof(FLOAT32)));
    }
  }

  pstr_lpd_scratch->lpd_state[0] = pstr_td_encoder->lpd_state;

  ssnr_1024 = 0;

  for (i1 = 0; i1 < 2; i1++)
  {
    ssnr_512 = 0;
    for (i2 = 0; i2 < 2; i2++)
    {
      k = (i1 * 2) + i2;
      p_params = acelp_tcx_params + (k * MAX_NUM_TCX_PRM_PER_DIV);

      impeghe_interpolation_lsp_params(&isp_curr_q[k * ORDER], &isp_curr_q[(k + 1) * ORDER],
                                       lp_filter_coeff_q, pstr_td_encoder->num_subfrm);

      pstr_lpd_scratch->lpd_state[k + 1] = pstr_lpd_scratch->lpd_state[k];

      impeghe_acelp_encode(&lp_filter_coeff[k * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
                           lp_filter_coeff_q, &speech[k * pstr_td_encoder->len_subfrm],
                           &pstr_scratch->p_wsig_buf[k * pstr_td_encoder->len_subfrm],
                           &pstr_scratch->p_synth_buf[k * pstr_td_encoder->len_subfrm],
                           &pstr_scratch->p_wsyn_buf[k * pstr_td_encoder->len_subfrm],
                           *codec_mode, &pstr_lpd_scratch->lpd_state[k + 1],
                           pstr_td_encoder->len_subfrm, norm_corr[k * 2], norm_corr[(k * 2) + 1],
                           ol_pitch_lag[k * 2], ol_pitch_lag[(k * 2) + 1], pit_adj, p_params,
                           pstr_scratch);

      mem_wsyn = pstr_lpd_scratch->lpd_state[k].mem_wsyn;
      impeghe_find_weighted_speech(&lp_filter_coeff[k * (NUM_SUBFR_SUPERFRAME / 4) * (ORDER + 1)],
                                   &pstr_scratch->p_synth_buf[k * LEN_FRAME],
                                   pstr_scratch->p_temp_wsyn_buf, &mem_wsyn, LEN_FRAME);
      pstr_lpd_scratch->lpd_state[k + 1].mem_wsyn = mem_wsyn;
      ssnr_256 = impeghe_cal_segsnr(&pstr_scratch->p_wsig_buf[k * LEN_FRAME],
                                    pstr_scratch->p_temp_wsyn_buf, LEN_FRAME, LEN_SUBFR);

      mode[k] = 0;
      num_tcx_param[k] = 0;

      impeghe_lpc_coef_gen(&isp_curr_q[k * ORDER], &isp_curr_q[(k + 1) * ORDER],
                           lp_filter_coeff_q, pstr_td_encoder->num_subfrm, ORDER);
      lpd_state_temp = pstr_lpd_scratch->lpd_state[k];

      impeghe_tcx_fac_encode(pstr_usac_data, pstr_usac_config,
                             &lp_filter_coeff[k * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
                             lp_filter_coeff_q, &speech[k * pstr_td_encoder->len_subfrm],
                             pstr_td_encoder->len_subfrm, num_bits_tcx, &lpd_state_temp, prm_tcx,
                             &num_params, ptr_sfb_offset, usac_independence_flag, num_sfb, ch_idx,
                             chn, k, ele_id);

      mem_wsyn = pstr_lpd_scratch->lpd_state[k].mem_wsyn;
      impeghe_find_weighted_speech(&lp_filter_coeff[k * (NUM_SUBFR_SUPERFRAME / 4) * (ORDER + 1)],
                                   pstr_scratch->p_synth_tcx_buf, pstr_scratch->p_temp_wsyn_buf,
                                   &mem_wsyn, LEN_FRAME);
      lpd_state_temp.mem_wsyn = mem_wsyn;
      tmp_ssnr = impeghe_cal_segsnr(&pstr_scratch->p_wsig_buf[k * LEN_FRAME],
                                    pstr_scratch->p_temp_wsyn_buf, LEN_FRAME, LEN_SUBFR);
      if (tmp_ssnr > ssnr_256)
      {
        ssnr_256 = tmp_ssnr;
        mode[k] = 1;
        num_tcx_param[k] = num_params;

        pstr_lpd_scratch->lpd_state[k + 1] = lpd_state_temp;

        memcpy(&pstr_scratch->p_synth_buf[(k * pstr_td_encoder->len_subfrm) - 128],
               pstr_scratch->p_synth_tcx_buf - 128,
               (pstr_td_encoder->len_subfrm + 128) * sizeof(FLOAT32));

        memcpy(&pstr_scratch->p_wsyn_buf[(k * pstr_td_encoder->len_subfrm) - 128],
               pstr_scratch->p_wsyn_tcx_buf - 128,
               (pstr_td_encoder->len_subfrm + 128) * sizeof(FLOAT32));

        memcpy(p_params, prm_tcx, NUM_TCX20_PRM * sizeof(WORD32));
      }
      ssnr_512 += 0.50f * ssnr_256;
    }

    k = i1 * 2;

    p_params = acelp_tcx_params + (k * MAX_NUM_TCX_PRM_PER_DIV);

    if (pstr_usac_config->full_band_lpd)
    {
      WORD32 *p_params_flpd = acelp_tcx_params_flpd + (i1 * MAX_NUM_TCX_PRM_PER_DIV);
      ptr_speech_buf = &pstr_td_encoder->flpd_speech_buf[0];
      impeghe_interpolation_lsp_params(&isp_curr_flpd_q[i1 * ORDER],
                                       &isp_curr_flpd_q[(i1 + 1) * ORDER], lp_filter_coeff_flpd_q,
                                       pstr_td_encoder->num_subfrm);

      pstr_lpd_scratch->flpd_state[i1 + 1] = pstr_lpd_scratch->flpd_state[i1];

      impeghe_acelp_encode(
          &lp_filter_coeff_flpd[i1 * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
          lp_filter_coeff_flpd_q, &ptr_speech_buf[i1 * pstr_td_encoder->len_subfrm],
          &pstr_usac_data->p_flpd_wsig_buf[i1 * pstr_td_encoder->len_subfrm],
          &pstr_usac_data->p_flpd_synth_buf[i1 * pstr_td_encoder->len_subfrm],
          &pstr_usac_data->p_flpd_wsyn_buf[i1 * pstr_td_encoder->len_subfrm], *codec_mode,
          &pstr_lpd_scratch->lpd_state[i1 + 1], pstr_td_encoder->len_subfrm, norm_corr[i1 * 2],
          norm_corr[(i1 * 2) + 1], ol_pitch_lag[i1 * 2], ol_pitch_lag[(i1 * 2) + 1], pit_adj,
          p_params_flpd, pstr_scratch);

      mem_wsyn = pstr_lpd_scratch->flpd_state[i1].mem_wsyn;
      impeghe_find_weighted_speech(
          &lp_filter_coeff_flpd[i1 * (NUM_SUBFR_SUPERFRAME / 4) * (ORDER + 1)],
          &pstr_usac_data->p_flpd_synth_buf[i1 * LEN_FRAME], pstr_usac_data->p_flpd_temp_wsyn_buf,
          &mem_wsyn, LEN_FRAME);
      pstr_lpd_scratch->flpd_state[i1 + 1].mem_wsyn = mem_wsyn;
      ssnr_256 = impeghe_cal_segsnr(&pstr_usac_data->p_flpd_wsig_buf[i1 * LEN_FRAME],
                                    pstr_usac_data->p_flpd_temp_wsyn_buf, LEN_FRAME, LEN_SUBFR);

      mode[i1] = 0;
      num_tcx_param[i1] = 0;

      ssnr_512 = ssnr_256;
      k = i1;
      p_params = acelp_tcx_params_flpd + (i1 * MAX_NUM_TCX_PRM_PER_DIV);
    }
    impeghe_lpc_coef_gen(&isp_curr_q[2 * i1 * ORDER], &isp_curr_q[(2 * i1 + 2) * ORDER],
                         lp_filter_coeff_q, (num_sbfrm_per_supfrm / 2), ORDER);
    lpd_state_temp = pstr_lpd_scratch->lpd_state[2 * i1];

    impeghe_tcx_fac_encode(pstr_usac_data, pstr_usac_config,
                           &lp_filter_coeff[2 * i1 * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
                           lp_filter_coeff_q, &speech[2 * i1 * pstr_td_encoder->len_subfrm],
                           2 * pstr_td_encoder->len_subfrm, 2 * num_bits_tcx, &lpd_state_temp,
                           prm_tcx, &num_params, ptr_sfb_offset, usac_independence_flag, num_sfb,
                           ch_idx, chn, 2 * i1, ele_id);

    mem_wsyn = pstr_lpd_scratch->lpd_state[2 * i1].mem_wsyn;
    impeghe_find_weighted_speech(
        &lp_filter_coeff[2 * i1 * (NUM_SUBFR_SUPERFRAME / 4) * (ORDER + 1)],
        pstr_scratch->p_synth_tcx_buf, pstr_scratch->p_temp_wsyn_buf, &mem_wsyn, LEN_FRAME * 2);
    lpd_state_temp.mem_wsyn = mem_wsyn;
    tmp_ssnr = impeghe_cal_segsnr(&pstr_scratch->p_wsig_buf[2 * i1 * LEN_FRAME],
                                  pstr_scratch->p_temp_wsyn_buf, LEN_FRAME * 2, LEN_SUBFR);

    if (tmp_ssnr > ssnr_512)
    {
      ssnr_512 = tmp_ssnr;
      if (!(pstr_usac_config->full_band_lpd))
      {
        for (i = 0; i < 2; i++)
        {
          mode[k + i] = 2;
          num_tcx_param[k + i] = num_params;
        }
        pstr_lpd_scratch->lpd_state[k + 2] = lpd_state_temp;
      }
      else
      {
        mode[k] = 1;
        num_tcx_param[k] = num_params;
        mode[k + 1] = 1;
        num_tcx_param[k + 1] = num_params;
        pstr_lpd_scratch->lpd_state[2 * i1 + 2] = lpd_state_temp;
      }

      memcpy(&pstr_scratch->p_synth_buf[(2 * i1 * pstr_td_encoder->len_subfrm) - 128],
             pstr_scratch->p_synth_tcx_buf - 128,
             ((2 * pstr_td_encoder->len_subfrm) + 128) * sizeof(FLOAT32));
      memcpy(&pstr_scratch->p_wsyn_buf[(2 * i1 * pstr_td_encoder->len_subfrm) - 128],
             pstr_scratch->p_wsyn_tcx_buf - 128,
             ((2 * pstr_td_encoder->len_subfrm) + 128) * sizeof(FLOAT32));
      memcpy(p_params, prm_tcx, NUM_TCX40_PRM * sizeof(WORD32));
    }
    ssnr_1024 += 0.50f * ssnr_512;
  }

  k = 0;

  if (!(pstr_usac_config->full_band_lpd))
  {
    p_params = acelp_tcx_params + (k * MAX_NUM_TCX_PRM_PER_DIV);
  }
  else
  {
    p_params = acelp_tcx_params_flpd + (k * MAX_NUM_TCX_PRM_PER_DIV);
  }

  impeghe_lpc_coef_gen(&isp_curr_q[k * ORDER], &isp_curr_q[(k + 4) * ORDER], lp_filter_coeff_q,
                       num_sbfrm_per_supfrm, ORDER);
  lpd_state_temp = pstr_lpd_scratch->lpd_state[k];

  impeghe_tcx_fac_encode(pstr_usac_data, pstr_usac_config,
                         &lp_filter_coeff[k * (num_sbfrm_per_supfrm / 4) * (ORDER + 1)],
                         lp_filter_coeff_q, &speech[k * pstr_td_encoder->len_subfrm],
                         4 * pstr_td_encoder->len_subfrm, 4 * num_bits_tcx, &lpd_state_temp,
                         prm_tcx, &num_params, ptr_sfb_offset, usac_independence_flag, num_sfb,
                         ch_idx, chn, k, ele_id);

  mem_wsyn = pstr_lpd_scratch->lpd_state[k].mem_wsyn;
  impeghe_find_weighted_speech(&lp_filter_coeff[k * (NUM_SUBFR_SUPERFRAME / 4) * (ORDER + 1)],
                               pstr_scratch->p_synth_tcx_buf, pstr_scratch->p_temp_wsyn_buf,
                               &mem_wsyn, LEN_FRAME * 4);
  lpd_state_temp.mem_wsyn = mem_wsyn;
  tmp_ssnr = impeghe_cal_segsnr(&pstr_scratch->p_wsig_buf[k * LEN_FRAME],
                                pstr_scratch->p_temp_wsyn_buf, LEN_FRAME * 4, LEN_SUBFR);

  if (pstr_usac_config->full_band_lpd)
  {
    tmp_ssnr = ssnr_1024 + 1;
  }
  if (tmp_ssnr > ssnr_1024)
  {
    ssnr_1024 = tmp_ssnr;
    if (!(pstr_usac_config->full_band_lpd))
    {
      for (i = 0; i < 4; i++)
      {
        mode[k + i] = 3;
        num_tcx_param[k + i] = num_params;
      }
    }
    else
    {
      for (i = 0; i < 2; i++)
      {
        mode[k + i] = 2;
        num_tcx_param[k + i] = num_params;
      }
    }

    pstr_lpd_scratch->lpd_state[k + 4] = lpd_state_temp;

    memcpy(&pstr_scratch->p_synth_buf[(k * pstr_td_encoder->len_subfrm) - 128],
           pstr_scratch->p_synth_tcx_buf - 128,
           ((4 * pstr_td_encoder->len_subfrm) + 128) * sizeof(FLOAT32));
    memcpy(&pstr_scratch->p_wsyn_buf[(k * pstr_td_encoder->len_subfrm) - 128],
           pstr_scratch->p_wsyn_tcx_buf - 128,
           ((4 * pstr_td_encoder->len_subfrm) + 128) * sizeof(FLOAT32));
    memcpy(p_params, prm_tcx, NUM_TCX80_PRM * sizeof(WORD32));
  }

  pstr_td_encoder->lpd_state = pstr_lpd_scratch->lpd_state[4];

  memcpy(pstr_td_encoder->weighted_sig, pstr_scratch->p_wsig_buf + (pstr_td_encoder->len_frame),
         128 * sizeof(FLOAT32));
  memcpy(pstr_td_encoder->prev_wsp, wsp_prev_buf, (len * sizeof(FLOAT32)));

  return;
}

/**
 *  impeghe_lpd_encode
 *
 *  \brief Main interface function for LPD encoding.
 *
 *  \param [in,out] pstr_usac_data         Pointer to USAC state structure.
 *  \param [in,out] mod_out           Pointer to encoding mode variable.
 *  \param [in] usac_independency_flg USAC independency flag value.
 *  \param [in,out] pstr_usac_config       Pointer to USAC config structure.
 *  \param [in] len_frame             Processing frame length.
 *  \param [in] i_ch                  Channel index.
 *  \param [in] chn                   Channel number.
 *  \param [in] pstr_it_bit_buff           Pointer to bitstream handler.
 *  \param [in] ele_id                Syntactic element id.
 *
 *  \return WORD32
 */
VOID impeghe_lpd_encode(ia_usac_data_struct *pstr_usac_data, WORD32 *mod_out,
                        WORD32 const usac_independency_flg,
                        ia_usac_encoder_config_struct *pstr_usac_config, WORD32 len_frame,
                        WORD32 i_ch, WORD32 chn, ia_bit_buf_struct *pstr_it_bit_buff,
                        WORD32 ele_id)
{
  LOOPIDX idx;
  WORD32 len_next_high_rate;
  WORD32 len_lpc0;
  WORD32 len_subfrm, k;
  WORD32 fscale;
  WORD32 first_lpd_flag;
  WORD32 num_sfb, ptr_sfb_width[MAX_NUM_SFB_LONG], sfb_offset[MAX_NUM_SFB_LONG + 1] = {0};
  WORD32 num_tcx_params[NUM_FRAMES] = {0};
  WORD32 mode_buf[1 + NUM_FRAMES] = {0};
  WORD32 *mode;
  FLOAT32 *speech, *new_speech;
  FLOAT32 *input_data;
  FLOAT32 *speech_buf;
  FLOAT32 *ptr_scratch_buf;
  ia_usac_td_encoder_struct *pstr_td_encoder;

  len_next_high_rate = (LEN_NEXT_HIGH_RATE * len_frame) / FRAME_LEN_LONG;
  len_lpc0 = (LEN_LPC0 * len_frame) / FRAME_LEN_LONG;
  fscale = pstr_usac_data->td_encoder[i_ch]->fscale;
  first_lpd_flag = (pstr_usac_data->core_mode_prev[i_ch] == CORE_MODE_FD);

  input_data = &pstr_usac_data->td_in_buf[i_ch][len_next_high_rate];
  speech_buf = pstr_usac_data->speech_buf;
  ptr_scratch_buf = pstr_usac_data->str_scratch.p_lpd_frm_enc_scratch;

  pstr_td_encoder = pstr_usac_data->td_encoder[i_ch];

  if (pstr_usac_config->full_band_lpd != 1)
  {
    impeghe_sfb_params_init(pstr_usac_config->sampling_rate, ptr_sfb_width, &num_sfb,
                            ONLY_LONG_SEQUENCE);
  }
  else
  {
    impeghe_sfb_params_init(pstr_usac_config->sampling_rate, ptr_sfb_width, &num_sfb,
                            EIGHT_SHORT_SEQUENCE);
  }

  k = 0;
  for (idx = 0; idx < num_sfb; idx++)
  {
    sfb_offset[idx] = k;
    k += ptr_sfb_width[idx];
  }
  sfb_offset[idx] = k;

  len_subfrm = pstr_td_encoder->len_subfrm;

  if (pstr_usac_data->core_mode_prev[i_ch] == CORE_MODE_FD)
  {
    WORD32 length;
    FLOAT32 *in_data;
    FLOAT32 *speech;
    ia_usac_td_encoder_struct *pstr_td_encoder_st;

    length = len_next_high_rate + len_lpc0;

    impeghe_reset_td_enc(pstr_usac_data->td_encoder[i_ch]);

    in_data = pstr_usac_data->td_in_prev_buf[i_ch];
    speech = pstr_usac_data->speech_buf;
    pstr_td_encoder_st = pstr_usac_data->td_encoder[i_ch];

    memcpy(speech, in_data, length * sizeof(FLOAT32));

    impeghe_highpass_50hz_12k8(speech, length, pstr_td_encoder_st->mem_sig_in,
                               pstr_td_encoder_st->fscale);
    impeghe_apply_preemph(speech, PREEMPH_FILT_FAC, length, &(pstr_td_encoder_st->mem_preemph));

    memcpy(pstr_td_encoder_st->old_speech_pe + ORDER, speech, length * sizeof(FLOAT32));
  }

  if (first_lpd_flag)
  {
    pstr_td_encoder->prev_mode = -1;
  }

  fscale = (fscale * len_subfrm) / LEN_FRAME;
  mode = mode_buf + 1;
  mode[-1] = pstr_td_encoder->prev_mode;
  new_speech = speech_buf + ORDER + (LEN_NEXT_HIGH_RATE * len_subfrm) / LEN_FRAME;
  speech = speech_buf + ORDER;
  if (first_lpd_flag)
  {
    new_speech += (LEN_LPC0 * len_subfrm) / LEN_FRAME;
    speech += (LEN_LPC0 * len_subfrm) / LEN_FRAME;
  }
  memcpy(new_speech, input_data, pstr_td_encoder->len_frame * sizeof(FLOAT32));

  impeghe_highpass_50hz_12k8(new_speech, pstr_td_encoder->len_frame, pstr_td_encoder->mem_sig_in,
                             fscale);
  impeghe_apply_preemph(new_speech, PREEMPH_FILT_FAC, pstr_td_encoder->len_frame,
                        &(pstr_td_encoder->mem_preemph));

  if (!(first_lpd_flag))
  {
    memcpy(speech_buf, pstr_td_encoder->old_speech_pe,
           ((ORDER + ((LEN_NEXT_HIGH_RATE * len_subfrm) / LEN_FRAME))) * sizeof(FLOAT32));
  }
  else
  {
    memcpy(speech_buf, pstr_td_encoder->old_speech_pe,
           ((ORDER + (((LEN_NEXT_HIGH_RATE + LEN_LPC0) * len_subfrm) / LEN_FRAME))) *
               sizeof(FLOAT32));
    for (idx = 0; idx < (len_subfrm + 1); idx++)
    {
      ptr_scratch_buf[idx] = speech[-len_subfrm - 1 + idx];
    }
    impeghe_apply_deemph(ptr_scratch_buf, PREEMPH_FILT_FAC, len_subfrm + 1, &ptr_scratch_buf[0]);
    memcpy(pstr_td_encoder->lpd_state.tcx_mem, &ptr_scratch_buf[len_subfrm - 128 + 1],
           128 * sizeof(FLOAT32));
  }

  if (pstr_usac_config->stereo_lpd)
  {
    WORD32 past = 0;
    if (first_lpd_flag)
    {
      // Copy appropriate left and right buffers data here.
    }
    if (!pstr_usac_data->str_slpd_data[ele_id].init_flag)
    {
      impeghe_slpd_init(&pstr_usac_data->str_slpd_data[ele_id], pstr_usac_config->sampling_rate,
                        pstr_usac_config->full_band_lpd, pstr_usac_config->ccfl);
    }
    past = pstr_usac_data->str_slpd_data[ele_id].lpd_stereo_config.dft_size - LEN_FRAME;
    if (past > LEN_FRAME)
    {
      past -= LEN_FRAME;
    }
    if (i_ch != 0)
    {
      memcpy(&pstr_usac_data->str_slpd_scratch.time_buff_right[0],
             &pstr_usac_data->str_slpd_data[ele_id].prev_right[0],
             sizeof(pstr_usac_data->str_slpd_scratch.time_buff_right[0]) * (past));
      memcpy(&pstr_usac_data->str_slpd_scratch.time_buff_right[past], input_data,
             sizeof(pstr_usac_data->str_slpd_scratch.time_buff_right[0]) *
                 (pstr_td_encoder->len_frame));
      memcpy(&pstr_usac_data->str_slpd_data[ele_id].prev_right[0],
             &input_data[pstr_td_encoder->len_frame - past],
             sizeof(pstr_usac_data->str_slpd_data[ele_id].prev_right[0]) * (past));
      impeghe_enc_stereo_lpd(&pstr_usac_data->str_slpd_data[ele_id],
                             &pstr_usac_data->str_slpd_scratch, &pstr_usac_data->str_scratch);
    }
    else
    {
      memcpy(&pstr_usac_data->str_slpd_scratch.time_buff_left[0],
             &pstr_usac_data->str_slpd_data[ele_id].prev_left[0],
             sizeof(pstr_usac_data->str_slpd_scratch.time_buff_left[0]) * (past));
      memcpy(&pstr_usac_data->str_slpd_scratch.time_buff_left[past], input_data,
             sizeof(pstr_usac_data->str_slpd_scratch.time_buff_left[0]) *
                 (pstr_td_encoder->len_frame));
      memcpy(&pstr_usac_data->str_slpd_data[ele_id].prev_left[0],
             &input_data[pstr_td_encoder->len_frame - past],
             sizeof(pstr_usac_data->str_slpd_data[ele_id].prev_left[0]) * (past));
      impeghe_core_lpd_encode(pstr_usac_data, pstr_usac_config, speech, mode, num_tcx_params,
                              sfb_offset, usac_independency_flg, num_sfb, i_ch, chn, ele_id);
    }
  }
  else
  {
    impeghe_core_lpd_encode(pstr_usac_data, pstr_usac_config, speech, mode, num_tcx_params,
                            sfb_offset, usac_independency_flg, num_sfb, i_ch, chn, ele_id);
  }

  if (!(first_lpd_flag))
  {
    memcpy(pstr_td_encoder->old_speech_pe, &speech_buf[(pstr_td_encoder->len_frame)],
           (ORDER + ((LEN_NEXT_HIGH_RATE * len_subfrm) / LEN_FRAME)) * sizeof(FLOAT32));
  }
  else
  {
    memcpy(pstr_td_encoder->old_speech_pe,
           &speech_buf[(pstr_td_encoder->len_frame) + (LEN_LPC0 * len_subfrm) / LEN_FRAME],
           (ORDER + ((LEN_NEXT_HIGH_RATE * len_subfrm) / LEN_FRAME)) * sizeof(FLOAT32));
  }

  if (!(i_ch == 1 && pstr_usac_config->stereo_lpd))
  {
    impeghe_encode_fac_params(mode, num_tcx_params, pstr_usac_data, pstr_usac_config,
                              usac_independency_flg, pstr_it_bit_buff, i_ch, ele_id);
  }
  else
  {
    impeghe_enc_slpd_params(&pstr_usac_data->str_slpd_data[ele_id],
                            &pstr_usac_data->str_slpd_scratch, pstr_it_bit_buff, first_lpd_flag);
  }
  if (!(pstr_usac_config->full_band_lpd))
  {
    pstr_td_encoder->prev_mode = mode[3];
  }
  else
  {
    pstr_td_encoder->prev_mode = mode[1];
  }

  memcpy(mod_out, mode, 4 * sizeof(WORD32));
  return;
}

/**
 *  impeghe_init_td_data
 *
 *  \brief Initializes LPD(TD) data structure.
 *
 *  \param [in] pstr_td_encoder             Pointer to USAC LPD encoder state structure.
 *  \param [in] len_frame      Processing frame length.
 *  \param [in] full_band_lpd  Fullband LPD configuration flag.
 *
 *  \return VOID
 */
VOID impeghe_init_td_data(ia_usac_td_encoder_struct *pstr_td_encoder, WORD32 len_frame,
                          WORD32 full_band_lpd)
{
  WORD32 len_window;
  WORD32 num_frames = NUM_FRAMES;

  if (full_band_lpd == 1)
  {
    num_frames = NUM_FRAMES;
  }

  pstr_td_encoder->len_subfrm = len_frame / num_frames;
  pstr_td_encoder->len_frame = len_frame;
  pstr_td_encoder->num_subfrm = (MAX_NUM_SUBFR * len_frame) / FRAME_LEN_LONG;

  impeghe_reset_td_enc(pstr_td_encoder);
  pstr_td_encoder->prev_mode = -1;
  pstr_td_encoder->arith_reset_flag = 1;

  if (pstr_td_encoder->fscale > FSCALE_DENOM)
  {
    len_window = (LEN_LP_WINDOW_HIGH_RATE * len_frame) / FRAME_LEN_LONG;
  }
  else
  {
    len_window = (LEN_LP_WINDOW * len_frame) / FRAME_LEN_LONG;
  }

  switch (len_window)
  {
  case 448:
    pstr_td_encoder->lp_analysis_window = impeghe_cos_window_448;
    break;
  case 512:
    pstr_td_encoder->lp_analysis_window = impeghe_cos_window_512;
    break;
  default:
    pstr_td_encoder->lp_analysis_window = impeghe_cos_window_512;
    break;
  }
  return;
}

/**
 *  impeghe_config_acelp_core_mode
 *
 *  \brief  Finds ACELP mode of encoding based on input configuration values.
 *
 *  \param [in,out] pstr_td_encoder            Pointer to USAC LPD encoder state structure.
 *  \param [in]     sampling_rate Sampling rate value.
 *  \param [in]     bitrate       Bitrate Value.
 *
 *  \return VOID
 */
VOID impeghe_config_acelp_core_mode(ia_usac_td_encoder_struct *pstr_td_encoder,
                                    WORD32 sampling_rate, WORD32 bitrate)
{
  WORD32 max_bits, coder_bits;
  const WORD32 *p_acelp_core_numbits_table;

  max_bits = (WORD32)((FLOAT32)(bitrate * FRAME_LEN_LONG) / (FLOAT32)sampling_rate);
  p_acelp_core_numbits_table = (WORD32 *)impeghe_acelp_core_numbits_1024;

  for (pstr_td_encoder->acelp_core_mode = 5; pstr_td_encoder->acelp_core_mode >= 0;
       pstr_td_encoder->acelp_core_mode--)
  {
    coder_bits = p_acelp_core_numbits_table[pstr_td_encoder->acelp_core_mode];
    if (coder_bits <= max_bits)
    {
      return;
    }
  }

  for (pstr_td_encoder->acelp_core_mode = 7; pstr_td_encoder->acelp_core_mode >= 6;
       pstr_td_encoder->acelp_core_mode--)
  {
    coder_bits = p_acelp_core_numbits_table[pstr_td_encoder->acelp_core_mode];
    if (coder_bits <= max_bits)
    {
      return;
    }
  }

  pstr_td_encoder->acelp_core_mode = 6;

  return;
}

/** @} */ /* End of CoreEncProc */
