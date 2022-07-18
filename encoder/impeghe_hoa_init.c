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
#include <stdio.h>
#include <string.h>
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_hoa_common_functions.h"

/**
 *  ia_hoa_dec_frame_param_init
 *
 *  \brief Intialization of HOA bit-stream parameters
 *
 *  \param [out] pstr_se_handle Spatial encoder handle
 *
 *  \return VOID
 */
static VOID ia_hoa_dec_frame_param_init(ia_spatial_enc_str *pstr_se_handle)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  UWORD32 tot_per_coders = pstr_se_handle->tot_perc_coders;
  UWORD32 num_coeffs = pstr_se_handle->num_hoa_coeffs;

  UWORD32 no_of_hoa_coeffs = num_coeffs;

  ULOOPIDX i;

  for (i = 0; i < tot_per_coders; i++)
  {
    pstr_frm_prm->is_exception[i] = 0;
    pstr_frm_prm->exponents[i] = 0;
  }
  for (i = 0; i < MAX_NUMBER_CHANNELS; i++)
  {
    pstr_frm_prm->active_n_grid_dir_idx[i].ch_idx = (UWORD32)-1;
    pstr_frm_prm->vectors[i].index = (UWORD32)-1;
    memset(pstr_frm_prm->vectors[i].sig_id, 0, sizeof(FLOAT64) * MAX_NUM_HOA_COEFFS);
  }
  pstr_frm_prm->vec_sz = 0;

  for (i = 0; i < no_of_hoa_coeffs; i++)
  {
    pstr_frm_prm->pred_typ_vec[i] = 0;
  }

  memset(pstr_frm_prm->pred_idx_mat, 0,
         sizeof(UWORD32) * MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS);
  memset(pstr_frm_prm->quant_pred_fac_mat, 0,
         sizeof(WORD32) * MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS);

  for (i = 0; i < MAX_SET_SIZE; i++)
  {
    pstr_frm_prm->amb_coeff_idx_to_enb[i] = (UWORD32)-1;
    pstr_frm_prm->amb_coeff_idx_to_disable[i] = (UWORD32)-1;
    pstr_frm_prm->non_en_dis_able_act_idx[i] = (UWORD32)-1;
    pstr_frm_prm->active_n_grid_dir_idx_for_subband_pred[i] = (UWORD32)-1;
  }

  return;
}

/**
 *  impeghe_hoa_amb_comp_mod_init
 *
 *  \brief Initialize ambient component modification module
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *  \param [in] amb_hoa_min_order Minimum ambient HOA order
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_amb_comp_mod_init(ia_spatial_enc_str *pstr_se_handle,
                                                  UWORD32 amb_hoa_min_order)
{
  ia_spatial_enc_amb_comp_mod_str *pstr_amb_handle = &(pstr_se_handle->amb_comp_hdl);
  UWORD32 frame_size = pstr_se_handle->frame_size;
  UWORD32 hoa_order = pstr_se_handle->hoa_order;
  UWORD32 tot_per_coders = pstr_se_handle->tot_perc_coders;
  UWORD32 num_var_ch = pstr_se_handle->num_add_perc_coders;
  UWORD32 bitrate = pstr_se_handle->br_per_perc_coders;

  ULOOPIDX i, j, ch;

  pstr_amb_handle->frame_size = frame_size;
  pstr_amb_handle->hoa_order = hoa_order;
  pstr_amb_handle->amb_hoa_order = amb_hoa_min_order;
  pstr_amb_handle->tot_perc_coders = tot_per_coders;
  pstr_amb_handle->num_var_ch = num_var_ch;
  pstr_amb_handle->br_per_coder = bitrate;

  pstr_amb_handle->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);
  pstr_amb_handle->num_amb_hoa_coeffs = (amb_hoa_min_order + 1) * (amb_hoa_min_order + 1);

  for (i = 1; i <= pstr_amb_handle->num_hoa_coeffs; i++)
  {
    pstr_amb_handle->pot_amb_hoa_coeff_idx[i - 1] = i;
  }

  for (j = 0; j < MAX_NUM_AMB_COEFF_INDICES; j++)
  {
    pstr_amb_handle->act_amb_hoa_coeffs_idx[j] = (UWORD32)-1;
    pstr_amb_handle->amb_hoa_coeffs_idx_to_enable[j] = (UWORD32)-1;
    pstr_amb_handle->amb_hoa_coeffs_idx_to_disable[j] = (UWORD32)-1;
  }

  if ((WORD32)amb_hoa_min_order > (WORD32)-1)
    pstr_amb_handle->inv_low_order_mode_mat = (pFlOAT64)impeghe_inv_mode_mat[amb_hoa_min_order];
  else
    pstr_amb_handle->inv_low_order_mode_mat = NULL;

  for (ch = 0; ch < pstr_amb_handle->num_var_ch; ch++)
  {
    impeghe_hoa_assignment_info_init(&(pstr_amb_handle->prev_tgt_assgn_vec[ch]));
    impeghe_hoa_assignment_info_init(&(pstr_amb_handle->assgn_vec[ch]));
  }

  for (ch = pstr_amb_handle->num_var_ch; ch < pstr_amb_handle->tot_perc_coders; ch++)
  {
    pstr_amb_handle->act_amb_hoa_coeffs_idx[ch - pstr_amb_handle->num_var_ch] =
        (ch - pstr_amb_handle->num_var_ch + 1);
  }

  if ((pstr_amb_handle->num_var_ch > pstr_amb_handle->tot_perc_coders) ||
      ((pstr_amb_handle->num_amb_hoa_coeffs + pstr_amb_handle->num_var_ch) >
       pstr_amb_handle->tot_perc_coders))
  {
    return IMPEGHE_INIT_FATAL_INVALID_AMB_INIT;
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_ch_assignment_init
 *
 *  \brief Initialize channel assignement module
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *
 *  \return VOID
 */
static VOID impeghe_hoa_ch_assignment_init(ia_spatial_enc_str *pstr_se_handle)
{
  ia_spatial_enc_ch_assignment_str *pstr_ch_asgmt = &(pstr_se_handle->ch_asgnmnt_hdl);

  pstr_ch_asgmt->frame_size = pstr_se_handle->frame_size;
  pstr_ch_asgmt->hoa_order = pstr_se_handle->hoa_order;
  pstr_ch_asgmt->tot_perc_coders = pstr_se_handle->tot_perc_coders;
  pstr_ch_asgmt->num_var_ch = pstr_se_handle->num_add_perc_coders;
  return;
}

/**
 *  impeghe_hoa_dir_vec_estimation_init
 *
 *  \brief Initialize dircetion/vector estimation logic
 *
 *  \param [in] pstr_se_handle Spatial encoder handle
 *  \param [in] coded_vec_len coded vector length
 *  \param [in] use_vec_est Flag to indicate to perform direction estimation or vector estimation
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_dir_vec_estimation_init(ia_spatial_enc_str *pstr_se_handle,
                                                        UWORD32 coded_vec_len,
                                                        UWORD32 use_vec_est)
{
  ia_spatial_enc_dir_vec_est_str *pstr_vec_est = &(pstr_se_handle->dir_vec_est_hdl);
  UWORD32 hoa_order = pstr_se_handle->hoa_order;
  UWORD32 min_amb_hoa_coeff = pstr_se_handle->min_amb_hoa_coeff;
  WORD32 i;

  pstr_vec_est->frame_size = pstr_se_handle->frame_size;
  pstr_vec_est->hoa_order = hoa_order;
  pstr_vec_est->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);
  pstr_vec_est->min_amb_hoa_coeff = min_amb_hoa_coeff;
  pstr_vec_est->vec_ele_num_bits = pstr_se_handle->vec_ele_num_bits;
  pstr_vec_est->use_vec_est = use_vec_est;
  if (coded_vec_len)
  {
    pstr_vec_est->num_vec_elem = pstr_vec_est->num_hoa_coeffs - pstr_vec_est->min_amb_hoa_coeff;
  }
  else
  {
    pstr_vec_est->num_vec_elem = pstr_vec_est->num_hoa_coeffs;
  }
  pstr_se_handle->frm_prm_hdl.vec_sz = pstr_vec_est->num_vec_elem;

  if (0 == pstr_se_handle->frm_prm_hdl.vec_sz)
  {
    return IMPEGHE_INIT_FATAL_INVALID_VEC_ELE;
  }

  pstr_vec_est->frame_counter = 0;
  pstr_vec_est->is_dir1_active = 0;
  pstr_vec_est->is_dir2_active = 0;
  for (i = 0; i < MAX_NUMBER_CHANNELS; i++)
  {
    pstr_vec_est->max_energy_dir[i] = -1;
  }
  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_dyn_correction_init
 *
 *  \brief Intialization of dynamic correction module
 *
 *  \param [out] pstr_se_handle spatial encoder handle
 *  \param [in] idx channel index
 *  \param [in] max_amp_exp maximum exponent
 *
 *  \return VOID
 */
static VOID impeghe_hoa_dyn_correction_init(ia_spatial_enc_str *pstr_se_handle, UWORD32 idx,
                                            UWORD32 max_amp_exp)
{
  ia_spatial_enc_dyn_correction_str *pstr_dyn_cor = &(pstr_se_handle->dyn_corrctn_hdl[idx]);
  UWORD32 frame_size = pstr_se_handle->frame_size;
  UWORD32 hoa_order = pstr_se_handle->hoa_order;
  UWORD32 hoa_coeffs;

  pstr_dyn_cor->frame_size = frame_size;
  pstr_dyn_cor->max_amp_exp = max_amp_exp;
  pstr_dyn_cor->win_func = (pFlOAT64)impeghe_window_function;
  pstr_dyn_cor->last_exp_base2 = 0;
  pstr_dyn_cor->prev_exp_base2 = 0;

  pstr_dyn_cor->last_gain = (FLOAT64)(1.0);
  pstr_dyn_cor->last_max_abs_value = (FLOAT64)(0.0);
  pstr_dyn_cor->smallest_delta = (FLOAT64)(pow(2.0, -31));
  pstr_dyn_cor->max_val_absorption = (FLOAT64)(0.1);

  hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);

  pstr_dyn_cor->max_attn_exp =
      pstr_dyn_cor->max_amp_exp + (UWORD32)(ceil(log((FLOAT64)(hoa_coeffs)) / log(2.0)));

  return;
}

/**
 *  impeghe_hoa_dir_based_pre_dom_sound_syn_init
 *
 *  \brief Direction based sound synthesis initialisation
 *
 *  \param [in,out] pstr_hoa_enc Spatial encoder handle
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_dir_based_pre_dom_sound_syn_init(ia_hoa_enc_str *pstr_hoa_enc)
{
  UWORD32 i;
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_hoa_enc->spat_enc_hdl.decmp_hdl);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *pstr_dir_pre_dom_syn =
      &(pstr_decomp_handle->pre_dom_syn_handle.dir_predom_syn_handle);
  UWORD32 hoa_order = pstr_decomp_handle->hoa_order;

  pstr_dir_pre_dom_syn->num_var_ch = pstr_decomp_handle->num_var_ch;
  pstr_dir_pre_dom_syn->hoa_order = hoa_order;
  pstr_dir_pre_dom_syn->frame_size = pstr_decomp_handle->frame_size;
  pstr_dir_pre_dom_syn->min_amb_hoa_coeff = pstr_decomp_handle->min_amb_hoa_coeff;
  pstr_dir_pre_dom_syn->min_pred_dir_sig = pstr_decomp_handle->max_pred_dir_sigs;
  pstr_dir_pre_dom_syn->scale_fac_num_bits = pstr_decomp_handle->bits_per_sf;
  pstr_dir_pre_dom_syn->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);

  switch (hoa_order)
  {
  case 0:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_0_coarse_grid_mode_mat;
    break;
  case 1:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_1_coarse_grid_mode_mat;
    break;
  case 2:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_2_coarse_grid_mode_mat;
    break;
  case 3:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_3_coarse_grid_mode_mat;
    break;
  case 4:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_4_coarse_grid_mode_mat;
    break;
  case 5:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_5_coarse_grid_mode_mat;
    break;
  case 6:
    pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt =
        (pFlOAT64)impeghe_hoa_order_6_coarse_grid_mode_mat;
    break;
  default:
    return IMPEGHE_INIT_FATAL_INVALID_HOA_ORDER;
  }

  pstr_dir_pre_dom_syn->transp_mode_fine_grid_pt = pstr_decomp_handle->fine_grid_transp_mode_mat;

  if (NULL == pstr_dir_pre_dom_syn->transp_mode_fine_grid_pt)
  {
    return IMPEGHE_INIT_FATAL_INVALID_MATRIX;
  }
  memset(pstr_dir_pre_dom_syn->pred_grid_dir_digs_smt_frm, 0,
         sizeof(FLOAT64) * MAX_FRAME_LEN * MAX_NUM_HOA_COEFFS);
  memset(pstr_dir_pre_dom_syn->pred_grid_dir_sigs_lst_frm, 0,
         sizeof(FLOAT64) * MAX_FRAME_LEN * MAX_NUM_HOA_COEFFS);
  memset(pstr_dir_pre_dom_syn->pred_grid_dir_sigs_cur_frm, 0,
         sizeof(FLOAT64) * MAX_FRAME_LEN * MAX_NUM_HOA_COEFFS);

  memset(pstr_dir_pre_dom_syn->lst_pred_idx_mat, 0,
         sizeof(UWORD32) * MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS);
  memset(pstr_dir_pre_dom_syn->lst_pred_fac_mat, 0,
         sizeof(UWORD32) * MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS);
  memset(pstr_dir_pre_dom_syn->pred_fac_mat, 0,
         sizeof(UWORD32) * MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS);

  memset(pstr_dir_pre_dom_syn->lst_pred_typ_vec, 0, sizeof(UWORD32) * MAX_NUM_HOA_COEFFS);

  for (i = 0; i < 2 * MAX_FRAME_LEN; i++)
  {
    pstr_dir_pre_dom_syn->ones_vector[i] = 1.0;
  }

  for (i = 0; i < MAX_NUM_HOA_COEFFS; i++)
  {
    pstr_dir_pre_dom_syn->lst_act_n_grid_dir_idx[i].ch_idx = (UWORD32)-1;
    pstr_dir_pre_dom_syn->old_ps_sig_idx[i] = (UWORD32)-1;
  }
  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_vector_based_predom_sound_syn_init
 *
 *  \brief Vector based sound synthesis initialisation
 *
 *  \param [in,out] pstr_hoa_enc Spatial encoder handle
 *
 *  \return VOID
 */
static VOID impeghe_hoa_vector_based_predom_sound_syn_init(ia_hoa_enc_str *pstr_hoa_enc)
{
  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_hoa_enc->spat_enc_hdl.decmp_hdl);
  ia_spatial_dec_vector_based_predom_sound_syn_str *pstr_vec_pre_dom_syn =
      &(pstr_decomp_handle->pre_dom_syn_handle.vec_predom_syn_handle);
  UWORD32 hoa_order = pstr_decomp_handle->hoa_order;

  ULOOPIDX j;

  pstr_vec_pre_dom_syn->max_dir_sigs = pstr_decomp_handle->num_var_ch;
  pstr_vec_pre_dom_syn->hoa_order = hoa_order;
  pstr_vec_pre_dom_syn->frame_size = pstr_decomp_handle->frame_size;
  pstr_vec_pre_dom_syn->num_hoa_coeff = (hoa_order + 1) * (hoa_order + 1);
  pstr_vec_pre_dom_syn->num_amb_ch = pstr_decomp_handle->min_amb_hoa_coeff;
  pstr_vec_pre_dom_syn->vector_start_ch = pstr_decomp_handle->vec_start_ch;
  pstr_vec_pre_dom_syn->coded_vec_len = pstr_hoa_cfg->coded_v_vec_length;
  pstr_vec_pre_dom_syn->interp_samples = pstr_hoa_cfg->spat_interpolation_time;

  for (j = 0; j < MAX_NUM_SIG_INDICES; j++)
  {
    pstr_vec_pre_dom_syn->prev_vec[j].index = (UWORD32)-1;
  }
  for (j = 0; j < MAX_NUM_HOA_DIR_SIGNALS; j++)
  {
    pstr_vec_pre_dom_syn->prev_ps_sig_idx[j] = (UWORD32)-1;
  }
  return;
}

/**
 *  impeghe_hoa_pre_dom_sound_syn_init
 *
 *  \brief Predominant sound synthesis initialisation
 *
 *  \param [in,out] pstr_hoa_enc Spatial encoder handle
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_pre_dom_sound_syn_init(ia_hoa_enc_str *pstr_hoa_enc)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_hoa_enc->spat_enc_hdl.decmp_hdl);
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_pre_dom_syn =
      &(pstr_decomp_handle->pre_dom_syn_handle);
  UWORD32 hoa_order = pstr_decomp_handle->hoa_order;
  UWORD32 frm_sz = pstr_decomp_handle->frame_size;
  UWORD32 interp_samples = pstr_hoa_cfg->spat_interpolation_time;
  UWORD32 interp_method = pstr_hoa_cfg->spat_interpolation_method;
  pFlOAT64 dir_sigs_cross_fade_win = (pFlOAT64)impeghe_fade_win_for_dir_based_syn;
  pstr_pre_dom_syn->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);
  pstr_pre_dom_syn->frame_size = frm_sz;
  pstr_pre_dom_syn->dir_sig_fade_in_win = dir_sigs_cross_fade_win;
  pstr_pre_dom_syn->dir_sig_fade_out_win = dir_sigs_cross_fade_win + frm_sz;

  impeghe_hoa_compute_fade_win_for_vec_based_syn(pstr_pre_dom_syn->vec_sig_fade_win, frm_sz,
                                                 interp_samples, interp_method);

  err_code = impeghe_hoa_dir_based_pre_dom_sound_syn_init(pstr_hoa_enc);
  if (err_code)
  {
    return err_code;
  }

  impeghe_hoa_vector_based_predom_sound_syn_init(pstr_hoa_enc);

  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_decomposition_init
 *
 *  \brief HOA signal decomposition module initialisation
 *
 *  \param [in,out] pstr_hoa_enc Spatial encoder handle
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_decomposition_init(ia_hoa_enc_str *pstr_hoa_enc)
{
  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  ia_spatial_enc_str *pstr_hoa_sp_enc = &(pstr_hoa_enc->spat_enc_hdl);
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_hoa_sp_enc->decmp_hdl);
  ULOOPIDX i;
  UWORD32 hoa_order = pstr_hoa_sp_enc->hoa_order;
  UWORD32 frame_size = pstr_hoa_sp_enc->frame_size;
  UWORD32 max_dir_sigs = pstr_hoa_sp_enc->max_dir_sigs;
  UWORD32 min_amb_hoa_coeff = pstr_hoa_sp_enc->min_amb_hoa_coeff;
  UWORD32 num_var_ch = pstr_hoa_sp_enc->num_add_perc_coders;
  UWORD32 coded_vec_length = pstr_hoa_cfg->coded_v_vec_length;
  UWORD32 interp_samples = pstr_hoa_cfg->spat_interpolation_time;
  UWORD32 interp_method = pstr_hoa_cfg->spat_interpolation_method;

  pstr_decomp_handle->ptr_scratch = pstr_hoa_enc->ptr_scratch;
  pstr_decomp_handle->hoa_order = hoa_order;
  pstr_decomp_handle->frame_size = frame_size;
  pstr_decomp_handle->max_dir_sigs = max_dir_sigs;
  pstr_decomp_handle->min_amb_hoa_coeff = min_amb_hoa_coeff;
  pstr_decomp_handle->max_pred_dir_sigs = pstr_hoa_cfg->max_no_of_dir_sigs_for_prediction;
  pstr_decomp_handle->bits_per_sf = pstr_hoa_cfg->no_of_bits_per_scale_factor;
  pstr_decomp_handle->num_var_ch = num_var_ch;
  pstr_decomp_handle->br_per_coder = pstr_hoa_sp_enc->br_per_perc_coders;
  ;
  pstr_decomp_handle->vec_ele_num_bits = pstr_hoa_sp_enc->vec_ele_num_bits;

  pstr_decomp_handle->num_hoa_coeffs = (hoa_order + 1) * (hoa_order + 1);
  pstr_decomp_handle->interp_samples = interp_samples;

  switch (coded_vec_length)
  {
  case 0:
    pstr_decomp_handle->vec_factor = 1.0;
    pstr_decomp_handle->vec_start_ch = 0;
    break;
  case 1:
    pstr_decomp_handle->vec_factor = 0.0;
    pstr_decomp_handle->vec_start_ch = pstr_decomp_handle->min_amb_hoa_coeff;
    break;
  case 2:
    pstr_decomp_handle->vec_factor = 1.0;
    pstr_decomp_handle->vec_start_ch = pstr_decomp_handle->min_amb_hoa_coeff;
    break;
  }

  pstr_decomp_handle->fine_grid_transp_mode_mat_sz = pstr_decomp_handle->num_hoa_coeffs;
  switch (hoa_order)
  {
  case 0:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_0_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_0_fine_grid_mode_mat_inv;
    break;
  case 1:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_1_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_1_fine_grid_mode_mat_inv;
    break;
  case 2:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_2_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_2_fine_grid_mode_mat_inv;
    break;
  case 3:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_3_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_3_fine_grid_mode_mat_inv;
    break;
  case 4:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_4_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_4_fine_grid_mode_mat_inv;
    break;
  case 5:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_5_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_5_fine_grid_mode_mat_inv;
    break;
  case 6:
    pstr_decomp_handle->fine_grid_transp_mode_mat =
        (pFlOAT64)impeghe_hoa_order_6_fine_grid_mode_mat;
    pstr_decomp_handle->fine_grid_transp_mode_mat_inv =
        (pFlOAT64)impeghe_hoa_order_6_fine_grid_mode_mat_inv;
    break;
  default:
    return IMPEGHE_INIT_FATAL_INVALID_HOA_ORDER;
  }

  pstr_decomp_handle->dir_sigs_cross_fade_win = (pFlOAT64)impeghe_fade_win_for_dir_based_syn;

  if (interp_samples > frame_size)
  {
    return IMPEGHE_INIT_FATAL_INVALID_INTERP_SAMPLE_SIZE;
  }

  impeghe_hoa_compute_fade_win_for_vec_based_syn(pstr_decomp_handle->vec_sigs_cross_fade_win,
                                                 frame_size, interp_samples, interp_method);

  memset(pstr_decomp_handle->ip_hoa_frame[0], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  memset(pstr_decomp_handle->ip_hoa_frame[1], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  memset(pstr_decomp_handle->ip_hoa_frame[2], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  pstr_decomp_handle->ip_idx = 0;
  pstr_decomp_handle->lst_ip_idx = 1;
  pstr_decomp_handle->penult_ip_idx = 2;

  memset(pstr_decomp_handle->dir_smtd_frame[0], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  memset(pstr_decomp_handle->dir_smtd_frame[1], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  pstr_decomp_handle->dir_smtd_frame_idx = 0;
  pstr_decomp_handle->dir_smtd_lst_frame_idx = 1;

  memset(pstr_decomp_handle->vec_smtd_frame[0], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  memset(pstr_decomp_handle->vec_smtd_frame[1], 0,
         sizeof(FLOAT64) * pstr_decomp_handle->frame_size * pstr_decomp_handle->num_hoa_coeffs);
  pstr_decomp_handle->vec_smtd_frame_idx = 0;
  pstr_decomp_handle->vec_smtd_lst_frame_idx = 1;

  pstr_decomp_handle->ps_sigs_smtd_lst_frame = pstr_hoa_enc->spat_enc_hdl.ps_sigs_smtd_lst_frame;

  for (i = 0; i < MAX_NUMBER_CHANNELS; i++)
  {
    pstr_decomp_handle->lst_sig_idx_map[i].ch_idx = -1;
    pstr_decomp_handle->lst_vec[i].index = (UWORD32)-1;
  }

  impeghe_hoa_pre_dom_sound_syn_init(pstr_hoa_enc);

  return IA_NO_ERROR;
}

/**
 *  impeghe_hoa_spatial_enc_init
 *
 *  \brief Spatial Encoder Intialization
 *
 *  \param [in,out] pstr_hoa_enc HOA encoder handle
 *  \param [in] max_dir_sigs Maximum directonal signals
 *  \param [in] br_per_perc_coders bitrate per channel
 *  \param [in] vec_ele_num_bits Number of bits for vector quantization
 *  \param [in] max_num_transmitted_hoa_coeffs Maximum number of HOA coefficients
 *  \param [in] use_vec_est Flag to indicate Vector estimation/Direction estimation
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_hoa_spatial_enc_init(ia_hoa_enc_str *pstr_hoa_enc, UWORD32 max_dir_sigs,
                                          UWORD32 br_per_perc_coders, UWORD32 vec_ele_num_bits,
                                          UWORD32 max_num_transmitted_hoa_coeffs,
                                          UWORD32 use_vec_est)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_spatial_enc_str *pstr_se_handle = &(pstr_hoa_enc->spat_enc_hdl);

  ia_hoa_config_struct *pstr_hoa_cfg = &pstr_hoa_enc->hoa_cfg;
  ia_hoa_frame_struct *pstr_hoa_frame = pstr_hoa_enc->hoa_frm;
  UWORD32 frame_size = pstr_hoa_enc->hoa_frame_length;
  UWORD32 hoa_order = pstr_hoa_cfg->order;
  UWORD32 min_amb_order = pstr_hoa_cfg->min_amb_order;
  UWORD32 tot_coders = pstr_hoa_cfg->total_num_coders;
  UWORD32 max_amp_exp = pstr_hoa_cfg->max_gain_corr_amp_exp;
  UWORD32 coded_v_vec_len = pstr_hoa_cfg->coded_v_vec_length;

  UWORD32 single_pred_idx_num_bits, indices_num_bits;
  WORD32 prelim_num_add_perc_coders;
  ULOOPIDX j, i;

  pstr_se_handle->send_ind_flag = 1;

  pstr_se_handle->frame_size = frame_size;
  pstr_se_handle->hoa_order = hoa_order;
  pstr_se_handle->max_dir_sigs = max_dir_sigs;
  pstr_se_handle->amb_hoa_min_order = pstr_hoa_cfg->min_amb_order;
  pstr_se_handle->tot_perc_coders = tot_coders;
  pstr_se_handle->br_per_perc_coders = br_per_perc_coders;
  pstr_se_handle->max_pred_dir_sigs = pstr_hoa_cfg->max_no_of_dir_sigs_for_prediction;
  pstr_se_handle->vec_ele_num_bits = vec_ele_num_bits;

  pstr_se_handle->max_num_hoa_coeffs = max_num_transmitted_hoa_coeffs;
  pstr_se_handle->coded_v_vec_len = coded_v_vec_len;
  pstr_se_handle->ip_ended = 0;
  pstr_se_handle->op_ended = 0;
  pstr_se_handle->delay_frm_cnt = 0;
  pstr_se_handle->req_delay_frm = 4;

  pstr_se_handle->min_amb_hoa_coeff = (min_amb_order + 1) * (min_amb_order + 1);
  pstr_se_handle->num_hoa_coeffs =
      (pstr_se_handle->hoa_order + 1) * (pstr_se_handle->hoa_order + 1);

  single_pred_idx_num_bits = impeghe_hoa_get_ceil_log2(pstr_se_handle->num_hoa_coeffs);

  indices_num_bits = single_pred_idx_num_bits;
  pstr_se_handle->explct_pred_idx_to_code = 1;

  while (indices_num_bits < pstr_se_handle->num_hoa_coeffs)
  {
    pstr_se_handle->explct_pred_idx_to_code++;
    indices_num_bits = pstr_se_handle->explct_pred_idx_to_code * single_pred_idx_num_bits +
                       impeghe_hoa_get_ceil_log2(pstr_se_handle->explct_pred_idx_to_code);
  }

  pstr_se_handle->explct_pred_idx_to_code--;

  prelim_num_add_perc_coders = (WORD32)(tot_coders - pstr_se_handle->min_amb_hoa_coeff);

  if (prelim_num_add_perc_coders < 0)
  {
    pstr_se_handle->amb_hoa_min_order =
        (UWORD32)(floor(sqrt(MAX(1.0, (FLOAT64)(pstr_se_handle->tot_perc_coders)))) - 1);
    pstr_se_handle->min_amb_hoa_coeff =
        (pstr_se_handle->amb_hoa_min_order + 1) * (pstr_se_handle->amb_hoa_min_order + 1);

    prelim_num_add_perc_coders = (WORD32)(tot_coders - pstr_se_handle->min_amb_hoa_coeff);
  }

  pstr_se_handle->num_add_perc_coders = prelim_num_add_perc_coders;

  if (pstr_se_handle->max_dir_sigs > pstr_se_handle->num_add_perc_coders)
  {
    pstr_se_handle->max_dir_sigs = pstr_se_handle->num_add_perc_coders;
  }

  ia_hoa_dec_frame_param_init(pstr_se_handle);

  err_code = impeghe_hoa_dir_vec_estimation_init(pstr_se_handle, coded_v_vec_len, use_vec_est);
  if (err_code)
  {
    return err_code;
  }

  err_code = impeghe_hoa_decomposition_init(pstr_hoa_enc);
  if (err_code)
  {
    return err_code;
  }

  pstr_hoa_cfg->num_transport_ch = pstr_se_handle->num_add_perc_coders;
  memcpy(&pstr_se_handle->penult_hoa_frm, pstr_hoa_frame, sizeof(ia_hoa_frame_struct));
  memcpy(&pstr_se_handle->thrd_lst_hoa_frm, pstr_hoa_frame, sizeof(ia_hoa_frame_struct));

  err_code = impeghe_hoa_amb_comp_mod_init(pstr_se_handle, min_amb_order);
  if (err_code)
  {
    return err_code;
  }

  impeghe_hoa_ch_assignment_init(pstr_se_handle);

  for (j = 0; j < pstr_se_handle->tot_perc_coders; j++)
  {
    impeghe_hoa_dyn_correction_init(pstr_se_handle, j, max_amp_exp);
  }

  memset(pstr_se_handle->ps_sigs_smtd_lst_frame, 0,
         sizeof(FLOAT64) * pstr_se_handle->max_dir_sigs * pstr_se_handle->frame_size);

  for (i = 0; i < MAX_SET_SIZE; i++)
  {
    pstr_se_handle->act_hoa_coeff_idx[i] = (UWORD32)-1;
  }

  for (i = 0; i < MAX_SET_SIZE; i++)
  {
    pstr_se_handle->enble_amb_coeff_idx_thrd_lst_frm[i] = (UWORD32)-1;
    pstr_se_handle->disble_amb_coeff_idx_thrd_lst_frm[i] = (UWORD32)-1;
    pstr_se_handle->no_en_dis_able_amb_coeff_idx_thrd_lst_frm[i] = (UWORD32)-1;
  }

  memset(pstr_se_handle->trans_ch[0], 0,
         sizeof(FLOAT64) * pstr_se_handle->tot_perc_coders * pstr_se_handle->frame_size);
  memset(pstr_se_handle->trans_ch[1], 0,
         sizeof(FLOAT64) * pstr_se_handle->tot_perc_coders * pstr_se_handle->frame_size);
  memset(pstr_se_handle->trans_ch[2], 0,
         sizeof(FLOAT64) * pstr_se_handle->tot_perc_coders * pstr_se_handle->frame_size);
  pstr_se_handle->curr_op_idx = 0;
  pstr_se_handle->scd_lst_ip_idx = 1;
  pstr_se_handle->thrd_lst_ip_idx = 2;
  return IA_NO_ERROR;
}
