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
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_hoa_frame.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_hoa_encoder.h"

/**
 *  impeghe_hoa_get_core_coder_delay
 *
 *  \brief Fetch core coder delay
 *
 *  \param [in] bitrate input bitrate
 *  \param [out] delay Core coder delay
 *
 *  \return VOID
 */
static VOID impeghe_hoa_get_core_coder_delay(UWORD32 bitrate, pUWORD32 delay)
{
  (void)bitrate;
  // Current core coder delay. Same for any number of HOA transport channels
  *delay = 1600;
}

/**
 *  impeghe_hoa_calc_num_coders
 *
 *  \brief Calculate number of HOA transport channels/coders
 *
 *  \param [in] total_bit_rate Input bitrate
 *  \param [in] use_vec_est Direction of Arrival of Sound estimation method
 *
 *  \return WORD32 number of HOA transport channels/coders
 */
WORD32 impeghe_hoa_calc_num_coders(UWORD32 total_bit_rate, UWORD32 use_vec_est)
{
  /*During create call, let the allocation happen for max channels. The br opt can be done from
  init() stage.*/
  WORD32 tot_num_coders = 0;
  if ((total_bit_rate >= 1200000))
  {
    tot_num_coders = 17;
  }
  else if ((total_bit_rate < 1200000) & (total_bit_rate >= 768000))
  {
    tot_num_coders = 24;
    if (use_vec_est)
    {
      tot_num_coders = 20;
    }
  }
  else if ((total_bit_rate < 768000) & (total_bit_rate >= 512000))
  {
    tot_num_coders = 16;
    if (use_vec_est)
    {
      tot_num_coders = 12;
    }
  }
  else if ((total_bit_rate < 512000) & (total_bit_rate >= 384000))
  {
    tot_num_coders = 12;
    if (use_vec_est)
    {
      tot_num_coders = 8;
    }
  }
  else if ((total_bit_rate < 384000) & (total_bit_rate >= 256000))
  {
    tot_num_coders = 8;
    if (use_vec_est)
    {
      tot_num_coders = 4;
    }
  }
  else if ((total_bit_rate < 256000))
  {
    UWORD32 temp = (UWORD32)(total_bit_rate) / (UWORD32)(32000);
    tot_num_coders = MAX(1, temp);
  }
  return tot_num_coders;
}

/**
 *  impeghe_hoa_config_init
 *
 *  \brief HOA configuration parameters initialization
 *
 *  \param [in,out] pstr_hoa_cfg HOA configuration parameter structure
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_config_init(ia_hoa_config_struct *pstr_hoa_cfg)
{

  // 3 Bit
  pstr_hoa_cfg->order = 0;

  // 1 Bit
  pstr_hoa_cfg->is_screen_relative = 0;

  // 1 Bit
  pstr_hoa_cfg->uses_nfc = 0;

  // 32 bit
  pstr_hoa_cfg->nfc_ref_distance = 0.0;

  // 3Bit
  pstr_hoa_cfg->min_amb_order = 0;

  pstr_hoa_cfg->total_num_coders = 0;

  // 4Bit
  pstr_hoa_cfg->num_addnl_coders = 0;

  // 1Bit
  pstr_hoa_cfg->use_phase_shift_decorr = 0;

  // 2Bit
  pstr_hoa_cfg->max_no_of_dir_sigs_for_prediction = 0;

  // 4Bit
  pstr_hoa_cfg->no_of_bits_per_scale_factor = 8;

  // 3Bit
  pstr_hoa_cfg->spat_interpolation_time = 256;

  // 1Bit
  pstr_hoa_cfg->spat_interpolation_method = 0;

  // 2Bit
  pstr_hoa_cfg->coded_v_vec_length = 1;

  pstr_hoa_cfg->max_v_vec_dir = 8;

  // 3Bit
  pstr_hoa_cfg->max_gain_corr_amp_exp = 4;

  // 1Bit
  pstr_hoa_cfg->is_single_layer = 1;

  // 2Bit
  pstr_hoa_cfg->frame_length_indicator = 0;
  pstr_hoa_cfg->frame_length = 1024;

  // 3Bit
  pstr_hoa_cfg->max_num_of_pred_dirs = 0;

  // escaped value (3,2,5)
  pstr_hoa_cfg->max_num_of_pred_dirs_per_band = 0;

  // escaped value (2,5,0)
  pstr_hoa_cfg->max_order_to_be_transmitted = 0;

  // 2Bit
  pstr_hoa_cfg->dir_grid_table_idx = 0;

  // 4 Bit
  pstr_hoa_cfg->first_sbr_subband_idx = 0;

  // 2 bit
  pstr_hoa_cfg->pred_subbands_idx = 0;

  // 2Bit
  pstr_hoa_cfg->par_subband_table_idx = 0;

  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_encoder_init
 *
 *  \brief HOA encoder initialization
 *
 *  \param [in] pstr_hoa_enc HOA encoder handle
 *  \param [in] it_bit_buff HOA bitstream bit buffer
 *  \param [in] total_bit_rate Input bit-rate
 *  \param [in] pstr_hoa_cfg_prm HOA configuration parameter structure
 *  \param [in] res_bitrate residual bitrate for encoding HOA transport channel
 *  \param [in] ptr_scratch Scratch memory
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_hoa_encoder_init(ia_hoa_enc_str *pstr_hoa_enc,
                                      ia_bit_buf_struct *it_bit_buff,
                                      const UWORD32 total_bit_rate,
                                      impeghe_hoa_config_str *pstr_hoa_cfg_prm,
                                      WORD32 *res_bitrate, pVOID ptr_scratch)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  UWORD32 core_coder_delay;
  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  ia_hoa_frame_struct *pstr_hoa_frame = pstr_hoa_enc->hoa_frm;

  const UWORD32 hoa_order = pstr_hoa_cfg_prm->hoa_order;
  const UWORD32 uses_nfc = pstr_hoa_cfg_prm->uses_nfc;
  const FLOAT32 nfc_distance = pstr_hoa_cfg_prm->nfc_distance;
  UWORD32 core_frame_len = MAX_FRAME_LEN;
  UWORD32 hoa_frame_len = MAX_FRAME_LEN;
  pUWORD32 tot_num_coders = &(pstr_hoa_cfg->total_num_coders);

  LOOPIDX i, ch;
  UWORD32 max_dir_sigs;
  WORD32 amb_hoa_min_order;
  UWORD32 max_pred_dir_sigs, bits_per_sf;
  UWORD32 max_gain_corr_amp_exp, interpolation_samples, coded_v_vec_len;
  UWORD32 interp_method, vec_ele_num_bits;

  UWORD32 dir_grid_table_idx, max_num_pred_dirs_per_band;
  UWORD32 max_num_pred_dirs_for_subband_pred, first_sbr_subband_idx;
  WORD32 max_hoa_order_to_be_transmitted = 1;
  UWORD32 subband_config_for_dir_pred_table_idx;

  UWORD32 num_pred_subbands, subband_config_for_dir_par_table_idx;
  UWORD32 num_par_subbands, last_first_order_subband_idx;
  UWORD8 *pscratch = ptr_scratch;
  UWORD32 *par_upmix_hoa_order = (UWORD32 *)pscratch;
  pscratch += MAX_NUM_PAR_SUBBANDS * sizeof(par_upmix_hoa_order[0]);
  WORD32 *use_real_coeffs = (WORD32 *)pscratch;
  pscratch += MAX_NUM_PAR_SUBBANDS * sizeof(use_real_coeffs[0]);
  UWORD32 *bit_rates_per_coder = (UWORD32 *)pscratch;

  UWORD32 max_trans_coeffs, bit_idx;

  memset(pstr_hoa_cfg, 0, sizeof(ia_hoa_config_struct));
  memset(pstr_hoa_frame, 0, sizeof(ia_hoa_frame_struct));
  memset(bit_rates_per_coder, 0, MAX_NUM_PERC_CODERS * sizeof(UWORD32));

  pstr_hoa_enc->ptr_scratch = ptr_scratch;
  pstr_hoa_enc->scratch_used_size = 0;

  pstr_hoa_enc->frm_bs_cnt = 0;
  pstr_hoa_enc->cfg_bs_bits = 0;
  pstr_hoa_enc->hoa_frame_length = hoa_frame_len;

  impeghe_hoa_get_core_coder_delay(total_bit_rate, &core_coder_delay);

  pstr_hoa_enc->num_dummy_frm = core_coder_delay / hoa_frame_len;

  if (core_coder_delay % hoa_frame_len)
    pstr_hoa_enc->num_dummy_frm++;

  pstr_hoa_enc->add_samp_dly = pstr_hoa_enc->num_dummy_frm * hoa_frame_len - core_coder_delay;

  max_dir_sigs = 4;
  // the folllowing parameters are included in config
  amb_hoa_min_order = 1;
  max_pred_dir_sigs = 2;
  bits_per_sf = 8;
  max_gain_corr_amp_exp = 4;
  interpolation_samples = 256;
  interp_method = 0;
  vec_ele_num_bits = 8;
  coded_v_vec_len = 0;

  dir_grid_table_idx = 0;
  max_num_pred_dirs_per_band = 0;
  max_num_pred_dirs_for_subband_pred = 0;
  first_sbr_subband_idx = 0;
  max_hoa_order_to_be_transmitted = 1;
  subband_config_for_dir_pred_table_idx = 1;

  if (total_bit_rate > 256000)
  {
    subband_config_for_dir_pred_table_idx = 0;
  }
  pstr_hoa_enc->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);
  impeghe_hoa_config_init(pstr_hoa_cfg);
  if (total_bit_rate > 256000)
  {
    pstr_hoa_cfg->par_subband_table_idx = 0;
  }
  pstr_hoa_cfg->core_coder_frame_length = core_frame_len;
  pstr_hoa_cfg->order = hoa_order;
  pstr_hoa_cfg->uses_nfc = uses_nfc;
  if (uses_nfc)
  {
    pstr_hoa_cfg->nfc_ref_distance = (FLOAT32)nfc_distance;
  }

  if (total_bit_rate >= 256000)
    max_hoa_order_to_be_transmitted = (WORD32)(hoa_order);
  else
    max_hoa_order_to_be_transmitted = 1;
  *tot_num_coders = impeghe_hoa_calc_num_coders(total_bit_rate, pstr_hoa_cfg_prm->use_vec_est);

  if (*tot_num_coders > pstr_hoa_enc->num_hoa_coeffs)
  {
    *tot_num_coders = pstr_hoa_enc->num_hoa_coeffs;
  }

  if ((total_bit_rate < 256000))
  {
    max_dir_sigs = 0;
  }

  num_pred_subbands = 0;
  switch (subband_config_for_dir_pred_table_idx)
  {
  case 0:
    num_pred_subbands = 1;
    break;
  case 1:
    num_pred_subbands = 10;
    break;
  case 2:
    num_pred_subbands = 20;
    break;
  }

  subband_config_for_dir_par_table_idx = 0;
  num_par_subbands = 0;
  switch (subband_config_for_dir_par_table_idx)
  {
  case 0:
    num_par_subbands = 1;
    break;
  case 1:
    num_par_subbands = 4;
    break;
  case 2:
    num_par_subbands = 8;
    break;
  }

  last_first_order_subband_idx = 0;

  for (i = 0; i < (LOOPIDX)num_par_subbands; i++)
  {
    par_upmix_hoa_order[i] = 2;
  }

  if (num_par_subbands > 0)
  {
    par_upmix_hoa_order[0] = 1;
  }

  for (i = 0; i < (LOOPIDX)num_par_subbands; i++)
  {
    use_real_coeffs[i] = 0;
  }
  if (num_par_subbands > 0)
  {
    use_real_coeffs[num_par_subbands - 1] = 1;
  }

  max_trans_coeffs =
      (WORD32)((max_hoa_order_to_be_transmitted + 1) * (max_hoa_order_to_be_transmitted + 1));

  if (*tot_num_coders < (UWORD32)((amb_hoa_min_order + 1) * (amb_hoa_min_order + 1)))
  {
    amb_hoa_min_order = (WORD32)(floor(sqrt((FLOAT64)(*tot_num_coders)) - 1.0));
  }

  amb_hoa_min_order = -1;
  pstr_hoa_cfg->min_amb_order = amb_hoa_min_order;
  pstr_hoa_cfg->max_no_of_dir_sigs_for_prediction = max_pred_dir_sigs;
  pstr_hoa_cfg->no_of_bits_per_scale_factor = bits_per_sf;
  pstr_hoa_cfg->spat_interpolation_time = interpolation_samples;
  pstr_hoa_cfg->spat_interpolation_method = interp_method;
  pstr_hoa_cfg->coded_v_vec_length = coded_v_vec_len;
  pstr_hoa_cfg->max_gain_corr_amp_exp = max_gain_corr_amp_exp;

  pstr_hoa_cfg->min_coeffs_for_amb = (amb_hoa_min_order + 1) * (amb_hoa_min_order + 1);
  pstr_hoa_cfg->num_addnl_coders = *tot_num_coders - pstr_hoa_cfg->min_coeffs_for_amb;
  if ((pstr_hoa_cfg->num_addnl_coders > MAX_VEC_CHANNELS) && (pstr_hoa_cfg_prm->use_vec_est))
  {
    WORD32 diff = pstr_hoa_cfg->num_addnl_coders - MAX_VEC_CHANNELS;
    pstr_hoa_cfg->min_amb_order = (WORD32)(floor(sqrt((FLOAT64)(diff)) - 1.0)) + 1;
    pstr_hoa_cfg->min_coeffs_for_amb =
        (pstr_hoa_cfg->min_amb_order + 1) * (pstr_hoa_cfg->min_amb_order + 1);

    pstr_hoa_cfg->num_addnl_coders = *tot_num_coders - pstr_hoa_cfg->min_coeffs_for_amb;
  }

  // Estimate the number of bits taken by HOA bitstream per second
  if (pstr_hoa_cfg_prm->use_vec_est)
  {
    FLOAT32 num_bits = (FLOAT32)(3 + 4 * pstr_hoa_cfg->num_addnl_coders);
    num_bits += 8 * pstr_hoa_cfg->num_addnl_coders * pstr_hoa_enc->num_hoa_coeffs;
    num_bits *= 46.875;
    if ((total_bit_rate < 1200000) & (total_bit_rate >= 256000))
    {
      *res_bitrate = total_bit_rate - (WORD32)floor(num_bits);
    }
    else
      *res_bitrate = total_bit_rate;
  }
  else
    *res_bitrate = total_bit_rate;

  pstr_hoa_cfg->frame_length = hoa_frame_len;
  pstr_hoa_cfg->max_num_of_pred_dirs = max_num_pred_dirs_for_subband_pred;
  pstr_hoa_cfg->max_num_of_pred_dirs_per_band = max_num_pred_dirs_per_band;
  pstr_hoa_cfg->max_order_to_be_transmitted = max_hoa_order_to_be_transmitted;
  pstr_hoa_cfg->dir_grid_table_idx = dir_grid_table_idx;
  pstr_hoa_cfg->first_sbr_subband_idx = first_sbr_subband_idx;
  pstr_hoa_cfg->subband_config_idx = subband_config_for_dir_pred_table_idx;
  pstr_hoa_cfg->num_of_pred_subbands = num_pred_subbands;

  pstr_hoa_cfg->par_subband_table_idx = subband_config_for_dir_par_table_idx;
  pstr_hoa_cfg->par_subband_widths_sz = num_par_subbands;
  pstr_hoa_cfg->last_first_order_subband_idx = last_first_order_subband_idx;
  pstr_hoa_cfg->upmix_order_per_par_subband_sz = num_par_subbands;
  for (i = 0; i < pstr_hoa_cfg->par_subband_widths_sz; i++)
  {
    pstr_hoa_cfg->upmix_order_per_par_subband[i] = par_upmix_hoa_order[i];
  }

  pstr_hoa_cfg->use_real_coeffs_per_par_subband_sz = num_par_subbands;
  for (i = 0; i < pstr_hoa_cfg->use_real_coeffs_per_par_subband_sz; i++)
  {
    pstr_hoa_cfg->use_real_coeffs_per_par_subband[i] = use_real_coeffs[i];
  }

  pstr_hoa_enc->tot_coders = *tot_num_coders;

  pstr_hoa_frame->ptr_config_data = pstr_hoa_cfg;

  for (ch = 0; ch < MAX_NUMBER_CHANNELS; ch++)
  {
    pstr_hoa_frame->coded_gain_correction_exp_sz[ch] = 0;
    for (bit_idx = 0; bit_idx < 128; bit_idx++)
    {
      pstr_hoa_frame->coded_gain_correction_exp[ch][bit_idx] = -1;
    }
  }

  for (i = 0; i < (LOOPIDX)(*tot_num_coders); i++)
  {
    pstr_hoa_frame->coded_gain_correction_exp[i][0] = 1;
    pstr_hoa_frame->coded_gain_correction_exp_sz[i] = 1;
    pstr_hoa_frame->gain_correction_exception[i] = 0;
  }

  for (i = 0; i < (LOOPIDX)(*tot_num_coders); i++)
  {
    bit_rates_per_coder[i] = total_bit_rate / *tot_num_coders;
  }
  pstr_hoa_enc->cfg_bs_bits = impeghe_fill_config(it_bit_buff, pstr_hoa_cfg);

  pstr_hoa_enc->cfg_bs_bits += impeghe_byte_align_buffer(it_bit_buff);

  pstr_hoa_enc->samp_read = 0;
  pstr_hoa_enc->enc_dly_samp = 2048 + pstr_hoa_enc->add_samp_dly;
  pstr_hoa_enc->samp_written = -pstr_hoa_enc->enc_dly_samp;

  memset(pstr_hoa_enc->trans_ch_dly_buf, 0,
         sizeof(FLOAT64) * pstr_hoa_enc->num_hoa_coeffs * pstr_hoa_enc->add_samp_dly);

  pstr_hoa_enc->op_samp_per_ch = hoa_frame_len;

  pstr_hoa_enc->hoa_si_bits = 0;
  pstr_hoa_enc->hoa_si_cnt = 0;
  pstr_hoa_enc->ip_ended = 0;
  pstr_hoa_enc->is_ip_ended = 0;
  pstr_hoa_enc->hoa_si_per_frame = core_frame_len / hoa_frame_len;

  pstr_hoa_frame->ch_side_info_sz = (WORD32)(
      *tot_num_coders - ((pstr_hoa_cfg->min_amb_order + 1) * (pstr_hoa_cfg->min_amb_order + 1)));

  for (i = 0; i < (LOOPIDX)pstr_hoa_frame->ch_side_info_sz; i++)
  {
    pstr_hoa_frame->channel_type[i] = HOA_EMPTY_CHANNEL;
  }

  if (0 == vec_ele_num_bits)
  {
    return IMPEGHE_INIT_FATAL_INVALID_QUANT;
  }
  err_code = impeghe_hoa_spatial_enc_init(pstr_hoa_enc, max_dir_sigs, bit_rates_per_coder[0],
                                          vec_ele_num_bits, max_trans_coeffs,
                                          pstr_hoa_cfg_prm->use_vec_est);
  if (err_code)
  {
    return err_code;
  }

  pstr_hoa_frame->hoa_independency_flag = 1;

  for (i = 0; i < (LOOPIDX)(*tot_num_coders); i++)
  {
    pstr_hoa_frame->coded_gain_correction_exp[i][0] = 1;
    pstr_hoa_frame->coded_gain_correction_exp_sz[i] = 1;
    pstr_hoa_frame->gain_correction_exception[i] = 0;
  }
  pstr_hoa_cfg->num_transport_ch = *tot_num_coders;

  pstr_hoa_frame->hoa_independency_flag = 1;
  pstr_hoa_frame->ps_prediction_active = 0;
  pstr_hoa_frame->use_dir_pred = 0;
  pstr_hoa_frame->perform_par = 0;

  // write dummy frames for synchronization between HOAFrames and decoded transport channels
  for (i = 1; i < pstr_hoa_enc->num_dummy_frm + 1; i++)
  {
    memcpy(&pstr_hoa_enc->hoa_frm[i], pstr_hoa_frame, sizeof(ia_hoa_frame_struct));
  }

  pstr_hoa_enc->ip_samp_per_ch = 0;

  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_encoder_process
 *
 *  \brief HOA encoder processing
 *
 *  \param [in] pstr_hoa_enc HOA encoder handle
 *  \param [in] pptr_hoa_signals Input HOA coefficient signals
 *  \param [in] in_sample_per_ch Samples per HOA coefficient signals
 *  \param [in] it_bit_buff HOA bitstream bit buffer
 *  \param [in] output_size Total number of output transport channel samples
 *  \param [in] pptr_hoa_trans_ch HOA transport channels
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_hoa_encoder_process(ia_hoa_enc_str *pstr_hoa_enc, FLOAT32 **pptr_hoa_signals,
                                         pUWORD32 in_sample_per_ch, ia_bit_buf_struct *it_bit_buf,
                                         pUWORD32 output_size, FLOAT32 **pptr_hoa_trans_ch)
{
  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  IA_ERRORCODE err_code = IA_NO_ERROR;

  ULOOPIDX i, j, k, n, n_ch;
  ia_spatial_enc_str *pstr_hoa_sp_enc = &(pstr_hoa_enc->spat_enc_hdl);
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_hoa_sp_enc->decmp_hdl);
  pFlOAT64 ptr_in_hoa_samples = pstr_decomp_handle->ip_hoa_frame[pstr_decomp_handle->ip_idx];

  UWORD32 tot_coders = pstr_hoa_enc->tot_coders;
  UWORD32 frm_len = pstr_hoa_enc->hoa_frame_length;
  UWORD32 hoa_coeffs = pstr_hoa_enc->num_hoa_coeffs;
  UWORD32 add_samp_dly = pstr_hoa_enc->add_samp_dly;
  pUWORD32 ip_samp_per_ch = &(pstr_hoa_enc->ip_samp_per_ch);
  pUWORD32 op_samp_per_ch = &(pstr_hoa_enc->op_samp_per_ch);
  pWORD64 samp_read = &(pstr_hoa_enc->samp_read);
  pWORD64 samp_written = &(pstr_hoa_enc->samp_written);
  pFlOAT64 ptr_trans_delay_buf = pstr_hoa_enc->trans_ch_dly_buf;

  UWORD32 taken_samples;
  pFlOAT64 ptr_output = NULL;
  pFlOAT64 ptr_temp_buf = pstr_hoa_enc->ptr_scratch;
  pstr_hoa_enc->scratch_used_size = sizeof(FLOAT64) * (MAX_FRAME_LEN);

  pstr_hoa_enc->frm_bs_cnt = 1;
  pstr_hoa_enc->frm_bs_bits[0] = 0;
  pstr_hoa_enc->cfg_bs_bits = 0;
  *output_size = 0;

  if ((*ip_samp_per_ch < frm_len) & !pstr_hoa_enc->ip_ended)
  {
    UWORD32 ip_buf_samples = *ip_samp_per_ch;
    UWORD32 samples_missing = frm_len - ip_buf_samples;
    UWORD32 samples_available = *in_sample_per_ch;
    UWORD32 samples_to_copy = MIN(samples_missing, samples_available);
    UWORD32 new_samples_per_ch = *in_sample_per_ch;

    *samp_read += samples_to_copy;

    for (n_ch = 0; n_ch < hoa_coeffs; ++n_ch)
    {
      *ip_samp_per_ch = samples_to_copy + ip_buf_samples;
      if (ip_buf_samples)
      {
        memset(ptr_in_hoa_samples + (n_ch * frm_len), 0, sizeof(FLOAT64) * ip_buf_samples);
      }

      for (i = 0; i < samples_to_copy; i++)
      {
        ptr_in_hoa_samples[(n_ch * frm_len) + ip_buf_samples + i] =
            (FLOAT64)pptr_hoa_signals[n_ch][i];
      }

      // copy remaining samples of input buffer to the beginning of the buffer
      new_samples_per_ch = samples_available - samples_to_copy;
      if (samples_to_copy < samples_available)
      {
        for (i = samples_to_copy; i < samples_available; i++)
        {
          pptr_hoa_signals[n_ch][i - samples_to_copy] = pptr_hoa_signals[n_ch][i];
        }
      }
    }
    *in_sample_per_ch = new_samples_per_ch;
  }

  if ((*ip_samp_per_ch < frm_len) & !pstr_hoa_enc->ip_ended)
  {
    return err_code;
  }
  else if ((*ip_samp_per_ch > 0) & (*ip_samp_per_ch < frm_len) & pstr_hoa_enc->ip_ended)
  {
    for (n_ch = 0; n_ch < hoa_coeffs; ++n_ch)
    {
      memset(&(ptr_in_hoa_samples[n_ch * frm_len + *ip_samp_per_ch]), 0,
             sizeof(FLOAT64) * (frm_len - *ip_samp_per_ch));
    }
    *ip_samp_per_ch = frm_len;
  }

  if (pstr_hoa_enc->ip_ended & (*ip_samp_per_ch == 0))
  {
    pstr_hoa_sp_enc->ip_ended = 1;
  }

  if (!(pstr_hoa_sp_enc->op_ended))
  {
    err_code = impeghe_hoa_spatial_enc_process(pstr_hoa_enc);
    if (IA_NO_ERROR != err_code)
    {
      return err_code;
    }
    ptr_output = pstr_hoa_sp_enc->trans_ch[pstr_hoa_sp_enc->curr_op_idx];

    *op_samp_per_ch = frm_len;

    *ip_samp_per_ch = 0;

    // write oldest stored frame
    pstr_hoa_enc->frm_bs_bits[0] =
        impeghe_hoa_frame(it_bit_buf, &(pstr_hoa_enc->hoa_frm[pstr_hoa_enc->num_dummy_frm]));
    // Move stored HOA frame
    for (i = pstr_hoa_enc->num_dummy_frm; i > 0; i--)
    {
      memcpy(&pstr_hoa_enc->hoa_frm[i], &pstr_hoa_enc->hoa_frm[i - 1],
             sizeof(ia_hoa_frame_struct));
    }

    pstr_hoa_enc->hoa_si_cnt++;
    if (pstr_hoa_enc->hoa_si_cnt == pstr_hoa_enc->hoa_si_per_frame)
    {
      // byte align frame buffer
      pstr_hoa_enc->frm_bs_bits[0] += impeghe_byte_align_buffer(it_bit_buf);

      pstr_hoa_enc->hoa_si_cnt = 0;
    }

    // Add additional delay to the transport channels for synchronization
    // of HOA frames with the decoded transport channels

    taken_samples = frm_len - add_samp_dly;

    for (i = 0; i < pstr_hoa_cfg->total_num_coders; i++)
    {
      memcpy(ptr_temp_buf, &(ptr_output[(i * frm_len) + taken_samples]),
             sizeof(FLOAT64) * (add_samp_dly));

      for (j = 0; j < taken_samples; j++)
      {
        ptr_output[(i * frm_len) + (frm_len - 1) - j] =
            ptr_output[(i * frm_len) + (taken_samples - 1) - j];
      }

      memcpy(&ptr_output[(i * frm_len)], &ptr_trans_delay_buf[i * add_samp_dly],
             sizeof(FLOAT64) * add_samp_dly);

      memcpy(&ptr_trans_delay_buf[i * add_samp_dly], ptr_temp_buf,
             sizeof(FLOAT64) * add_samp_dly);
    }

    *samp_written += (WORD64)frm_len;
    if (*samp_written > *samp_read)
    {
      WORD32 samples_to_write =
          (WORD32)MAX((WORD64)0, *samp_read - *samp_written + (WORD64)frm_len);

      *samp_written = *samp_written - (WORD64)frm_len + samples_to_write;

      if (samples_to_write <= (WORD32)*op_samp_per_ch)
      {
        k = 0;
        for (n = 0; n < tot_coders; ++n)
        {
          for (i = 0; i < (ULOOPIDX)samples_to_write; i++)
          {
            ptr_output[k] = ptr_output[n * frm_len + i];
            k++;
          }
        }
      }
      *op_samp_per_ch = samples_to_write;
    }

    *output_size = tot_coders * *op_samp_per_ch;
  }
  else
  {
    if (*samp_written < *samp_read)
    {
      // Add additional delay to the transport channels for synchronization
      // of HOA frames with the decoded transport channels
      UWORD32 samples_to_copy_from_delay_buffer;
      UWORD32 missing_samples = (UWORD32)(*samp_read - *samp_written);
      *samp_written += missing_samples;
      if (missing_samples > frm_len)
      {
        missing_samples = frm_len;
      }
      samples_to_copy_from_delay_buffer = MIN(missing_samples, (UWORD32)(add_samp_dly));
      if (ptr_output)
      {
        for (i = 0; i < pstr_hoa_cfg->total_num_coders; i++)
        {
          memset(&ptr_output[(i * frm_len)], 0, sizeof(FLOAT64) * missing_samples);
          memcpy(&ptr_output[(i * frm_len)], &ptr_trans_delay_buf[i * add_samp_dly],
                 sizeof(FLOAT64) * samples_to_copy_from_delay_buffer);
        }
      }

      *output_size = tot_coders * *op_samp_per_ch;
    }
    else
    {
      pstr_hoa_enc->is_ip_ended = 1;
    }
  }
  k = 0;
  if (ptr_output)
  {
    for (n = 0; n < (WORD32)tot_coders; ++n)
    {
      for (i = 0; i < *op_samp_per_ch; i++)
      {
        pptr_hoa_trans_ch[n][i] = (FLOAT32)(ptr_output[k] * 32768);
        k++;
      }
    }
  }

  return err_code;
}
