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
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_pre_dom_sound_syn.h"

/**
 *  impeghe_hoa_compute_hoa_representation_of_spat_pred
 *
 *  \brief Compute HOA representation of spatially predicted signals
 *
 *  \param [in,out] pstr_se_handle Spatial Encoder handle
 *  \param [in] pred_typ_vec Prediction type
 *  \param [in] pred_idx_mat Prediction index matrix
 *  \param [in] pred_fac_mat prediction factor matrix
 *  \param [in] p_fade_win fade window
 *  \param [in] ptr_pred_grid_dir_sig_frm predicted directional frame
 *  \param [in] ptr_spatial_pred_in_curr_frame predicted spatial frame
 *
 *  \return VOID
 */
static VOID impeghe_hoa_compute_hoa_representation_of_spat_pred(
    ia_spatial_enc_str *pstr_se_handle, const pUWORD32 pred_typ_vec, const pUWORD32 pred_idx_mat,
    const pFlOAT64 pred_fac_mat, const pFlOAT64 p_fade_win, pFlOAT64 ptr_pred_grid_dir_sig_frm,
    pWORD32 ptr_spatial_pred_in_curr_frame)
{
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_predom_syn =
      &(pstr_se_handle->decmp_hdl.pre_dom_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *pstr_dir_pre_dom_syn =
      &(pstr_predom_syn->dir_predom_syn_handle);
  pVOID ptr_scratch = ((pUWORD8)pstr_se_handle->decmp_hdl.ptr_scratch +
                       pstr_se_handle->decmp_hdl.scratch_used_size);

  UWORD32 frm_sz = pstr_dir_pre_dom_syn->frame_size;
  UWORD32 hoa_coeffs = pstr_dir_pre_dom_syn->num_hoa_coeffs;
  UWORD32 pred_max_dir_sigs = pstr_dir_pre_dom_syn->min_pred_dir_sig;
  UWORD32 pred_vec_sz = pstr_predom_syn->num_hoa_coeffs;
  const pFlOAT64 ptr_in_ps_sigs_frm = pstr_se_handle->ps_sigs_smtd_lst_frame;
  const pFlOAT64 ptr_mode_mat_grid = pstr_dir_pre_dom_syn->mode_mat_coarse_grid_pt;
  ULOOPIDX k, s, i, j;
  pFlOAT64 ptr_tmp_faded_pred_grid_dir_sig, ptr_pred_grid_dir_sig_frm_temp;

  memset(ptr_pred_grid_dir_sig_frm, 0, sizeof(FLOAT64) * frm_sz * hoa_coeffs);

  ptr_tmp_faded_pred_grid_dir_sig = (pFlOAT64)ptr_scratch;

  *ptr_spatial_pred_in_curr_frame = 0;

  for (i = 0; i < pred_vec_sz; i++)
  {
    if (pred_typ_vec[i] != 0)
    {
      UWORD32 num_valid_sigs_for_pred;
      pUWORD32 ptr_pred_idx_mat;

      *ptr_spatial_pred_in_curr_frame = 1;

      memset(ptr_tmp_faded_pred_grid_dir_sig, 0, sizeof(FLOAT64) * MAX_FRAME_LEN);

      num_valid_sigs_for_pred = 0;
      ptr_pred_idx_mat = (UWORD32 *)pred_idx_mat + i;

      for (j = 0; j < pred_max_dir_sigs; j++)
      {
        if (*ptr_pred_idx_mat != 0)
        {
          num_valid_sigs_for_pred++;
        }
        else
        {
          break;
        }
        ptr_pred_idx_mat += pred_vec_sz;
      }

      for (s = 0; s < frm_sz; s++)
      {
        pFlOAT64 ptr_pred_factors_mat = (pFlOAT64)pred_fac_mat + i;
        ptr_pred_idx_mat = (UWORD32 *)pred_idx_mat + i;

        for (j = 0; j < num_valid_sigs_for_pred; j++)
        {
          ptr_tmp_faded_pred_grid_dir_sig[s] +=
              *ptr_pred_factors_mat *
              *(ptr_in_ps_sigs_frm + (*ptr_pred_idx_mat - 1) * frm_sz + s);

          ptr_pred_factors_mat += pred_vec_sz;
          ptr_pred_idx_mat += pred_vec_sz;
        }
        ptr_tmp_faded_pred_grid_dir_sig[s] *= p_fade_win[s];
      }

      ptr_pred_grid_dir_sig_frm_temp = ptr_pred_grid_dir_sig_frm;
      for (k = 0; k < hoa_coeffs; k++)
      {
        pFlOAT64 ptr_mode_mat_all_coarse_grid_points =
            (pFlOAT64)ptr_mode_mat_grid + k * hoa_coeffs + i;

        for (s = 0; s < frm_sz; s++)
        {
          *ptr_pred_grid_dir_sig_frm_temp +=
              *ptr_mode_mat_all_coarse_grid_points * ptr_tmp_faded_pred_grid_dir_sig[s];
          ptr_pred_grid_dir_sig_frm_temp++;
        }
      }
    }
  }

  return;
}

/**
 *  impeghe_hoa_compute_hoa_repr_of_dir_sigs
 *
 *  \brief Compute HOA representation of directional signals
 *
 *  \param [in,out] pstr_se_handle Spatial Encoder handle
 *  \param [in] ptr_set_of_ps_sig_indices set of ps signal indices
 *
 *  \return VOID
 */
static VOID impeghe_hoa_compute_hoa_repr_of_dir_sigs(ia_spatial_enc_str *pstr_se_handle,
                                                     const pUWORD32 ptr_set_of_ps_sig_indices)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_predom_syn =
      &(pstr_se_handle->decmp_hdl.pre_dom_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *pstr_dir_pre_dom_syn =
      &(pstr_predom_syn->dir_predom_syn_handle);

  UWORD32 frm_sz = pstr_dir_pre_dom_syn->frame_size;
  pFlOAT64 ptr_in_ps_sigs_frame = pstr_se_handle->ps_sigs_smtd_lst_frame;
  const ia_hoa_dir_id_str *ptr_active_dir_and_grid_indices = pstr_frm_prm->active_n_grid_dir_idx;
  const pFlOAT64 ptr_vec_sig_fade_out_win = &(pstr_predom_syn->vec_sig_fade_win[frm_sz]);
  UWORD32 num_coeffs = pstr_dir_pre_dom_syn->num_hoa_coeffs;
  pFlOAT64 ptr_out_smt_dir_sigs_frm =
      pstr_se_handle->decmp_hdl.dir_smtd_frame[pstr_se_handle->decmp_hdl.dir_smtd_frame_idx];
  ia_hoa_dir_id_str *pstr_lst_act_dir_idx = pstr_dir_pre_dom_syn->lst_act_n_grid_dir_idx;
  pFlOAT64 ptr_op_smt_dir_sigs_frm_temp;
  ULOOPIDX i, j, k, s;

  memset(ptr_out_smt_dir_sigs_frm, 0, sizeof(FLOAT64) * num_coeffs * frm_sz);

  for (i = 0; pstr_lst_act_dir_idx[i].ch_idx != -1; i++)
  {
    UWORD32 curr_dir_idx, curr_grid_dir_idx;

    if (i == MAX_NUM_HOA_COEFFS)
      break;

    curr_dir_idx = pstr_lst_act_dir_idx[i].ch_idx;
    curr_grid_dir_idx = pstr_lst_act_dir_idx[i].dir_id;

    if (curr_grid_dir_idx != 0)
    {
      pFlOAT64 curr_fade_out_win;

      for (j = 0; j < MAX_NUM_HOA_COEFFS; j++)
      {
        if ((UWORD32)ptr_active_dir_and_grid_indices[j].ch_idx == curr_dir_idx)
          break;
      }

      curr_fade_out_win = pstr_dir_pre_dom_syn->ones_vector;

      if (j != MAX_NUM_HOA_COEFFS)
      {
        if (ptr_active_dir_and_grid_indices[j].dir_id != 0)
        {
          curr_fade_out_win = &(pstr_predom_syn->dir_sig_fade_out_win[0]);
        }
      }
      else
      {
        for (j = 0; j < MAX_NUM_HOA_DIR_SIGNALS; j++)
        {
          if (ptr_set_of_ps_sig_indices[j] == curr_dir_idx)
            break;
        }

        if (j != MAX_NUM_HOA_COEFFS)
        {
          curr_fade_out_win = &(ptr_vec_sig_fade_out_win[0]);
        }
      }

      ptr_op_smt_dir_sigs_frm_temp = ptr_out_smt_dir_sigs_frm;
      for (k = 0; k < num_coeffs; k++)
      {
        pFlOAT64 ptr_in_ps_sigs_frame_temp = ptr_in_ps_sigs_frame + (curr_dir_idx - 1) * frm_sz;
        pFlOAT64 ptr_transpose_mode_mat_all_fine_grid_points =
            pstr_dir_pre_dom_syn->transp_mode_fine_grid_pt +
            (curr_grid_dir_idx - 1) * num_coeffs + k;

        for (s = 0; s < frm_sz; s++)
        {
          FLOAT64 curr_hoa_contribution =
              (*ptr_transpose_mode_mat_all_fine_grid_points) * (*ptr_in_ps_sigs_frame_temp++);

          curr_hoa_contribution *= curr_fade_out_win[s];

          *ptr_op_smt_dir_sigs_frm_temp += curr_hoa_contribution;
          ptr_op_smt_dir_sigs_frm_temp++;
        }
      }
    }
  }

  for (i = 0; ptr_active_dir_and_grid_indices[i].ch_idx != -1; i++)
  {
    UWORD32 curr_dir_idx, curr_grid_dir_idx;

    if (i == MAX_NUM_HOA_COEFFS)
      break;

    curr_dir_idx = ptr_active_dir_and_grid_indices[i].ch_idx;
    curr_grid_dir_idx = ptr_active_dir_and_grid_indices[i].dir_id;
    if (curr_grid_dir_idx != 0)
    {
      pFlOAT64 curr_fade_in_win = pstr_dir_pre_dom_syn->ones_vector;

      for (j = 0; j < MAX_NUM_HOA_COEFFS; j++)
      {
        if ((UWORD32)pstr_lst_act_dir_idx[j].ch_idx == curr_dir_idx)
          break;
      }
      if (j != MAX_NUM_HOA_COEFFS)
      {
        if (pstr_lst_act_dir_idx[j].dir_id != 0)
        {
          curr_fade_in_win = &(pstr_predom_syn->dir_sig_fade_in_win[0]);
        }
      }
      else
      {
        pUWORD32 old_ps_sig_idx = pstr_dir_pre_dom_syn->old_ps_sig_idx;
        for (j = 0; j < MAX_NUM_HOA_COEFFS; j++)
        {
          if (old_ps_sig_idx[j] == curr_dir_idx)
            break;
        }
        if (j != MAX_NUM_HOA_COEFFS)
        {
          curr_fade_in_win = &(pstr_predom_syn->dir_sig_fade_in_win[0]);
        }
      }

      ptr_op_smt_dir_sigs_frm_temp = ptr_out_smt_dir_sigs_frm;
      for (k = 0; k < num_coeffs; k++)
      {
        pFlOAT64 input_ps_sigs_frame_ptr = ptr_in_ps_sigs_frame + (curr_dir_idx - 1) * frm_sz;
        pFlOAT64 transpose_mode_mat_all_fine_grid_points_ptr =
            pstr_dir_pre_dom_syn->transp_mode_fine_grid_pt +
            (curr_grid_dir_idx - 1) * num_coeffs + k;

        for (s = 0; s < frm_sz; s++)
        {
          FLOAT64 curr_hoa_contribution =
              *(transpose_mode_mat_all_fine_grid_points_ptr) * (*input_ps_sigs_frame_ptr++);
          curr_hoa_contribution *= curr_fade_in_win[s];
          *ptr_op_smt_dir_sigs_frm_temp += curr_hoa_contribution;
          ptr_op_smt_dir_sigs_frm_temp++;
        }
      }
    }
  }

  for (i = 0; ptr_active_dir_and_grid_indices[i].ch_idx != -1; i++)
  {
    if (i == MAX_NUM_HOA_COEFFS)
      break;

    pstr_lst_act_dir_idx[i].ch_idx = ptr_active_dir_and_grid_indices[i].ch_idx;
    pstr_lst_act_dir_idx[i].dir_id = ptr_active_dir_and_grid_indices[i].dir_id;
  }
  for (; i < MAX_NUM_HOA_COEFFS; i++)
  {
    pstr_lst_act_dir_idx[i].ch_idx = (UWORD32)-1;
    pstr_lst_act_dir_idx[i].dir_id = (UWORD32)-1;
  }

  for (i = 0; (WORD32)ptr_set_of_ps_sig_indices[i] != -1; i++)
  {
    if (i == MAX_NUM_HOA_DIR_SIGNALS)
      break;

    pstr_dir_pre_dom_syn->old_ps_sig_idx[i] = ptr_set_of_ps_sig_indices[i];
  }
  for (; i < MAX_NUM_HOA_COEFFS; i++)
    pstr_dir_pre_dom_syn->old_ps_sig_idx[i] = (UWORD32)-1;

  return;
}
/**
 *  impeghe_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs
 *
 *  \brief Compute smoothened HOA representation of spatially predicted signals
 *
 *  \param [in,out] pstr_se_handle Spatial Encoder handle
 *  \param [in] ptr_amb_coeff_indices_enabled Ambient coefficients to be enabled
 *  \param [in] ptr_amb_coeff_indices_disabled Ambient coefficients to be disabled
 *  \param [in] ptr_hoa_coeff_indices Active HOA coefficients that are not
 * enabled/disabled
 *
 *  \return VOID
 */
static VOID impeghe_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs(
    ia_spatial_enc_str *pstr_se_handle, const pUWORD32 ptr_amb_coeff_indices_enabled,
    const pUWORD32 ptr_amb_coeff_indices_disabled, const pUWORD32 ptr_hoa_coeff_indices)
{
  UWORD32 i;
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_predom_syn =
      &(pstr_se_handle->decmp_hdl.pre_dom_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *pstr_dir_pre_dom_syn =
      &(pstr_predom_syn->dir_predom_syn_handle);

  UWORD32 ip_ps_sigs_frm_sz = pstr_se_handle->decmp_hdl.num_var_ch;

  const pUWORD32 pred_typ_vec = pstr_frm_prm->pred_typ_vec;
  const pUWORD32 pred_idx_mat = pstr_frm_prm->pred_idx_mat;
  const pFlOAT64 ptr_dir_fade_in_win = pstr_predom_syn->dir_sig_fade_in_win;
  const pFlOAT64 ptr_dir_fade_out_win = pstr_predom_syn->dir_sig_fade_out_win;
  pFlOAT64 ptr_out_pred_grid_dir_smth_frm = pstr_dir_pre_dom_syn->pred_grid_dir_digs_smt_frm;

  UWORD32 num_coeffs = pstr_dir_pre_dom_syn->num_hoa_coeffs;
  UWORD32 frm_sz = pstr_dir_pre_dom_syn->frame_size;
  UWORD32 min_pred_dir_sig = pstr_dir_pre_dom_syn->min_pred_dir_sig;

  ULOOPIDX l, j, k, s;
  WORD32 is_spat_pred_lst_frm, is_spat_pred_cur_frm;

  pUWORD32 ptr_lst_pred_idx_mat, ptr_pred_idx_mat;
  pFlOAT64 ptr_lst_pred_fac_mat;

  pWORD32 quant_pred_fac_mat_ptr = pstr_frm_prm->quant_pred_fac_mat;
  pFlOAT64 pred_fac_mat_ptr = pstr_dir_pre_dom_syn->pred_fac_mat;
  for (l = 0; l < min_pred_dir_sig; l++)
  {
    for (j = 0; j < num_coeffs; j++)
    {
      FLOAT64 dequantized_factor =
          (FLOAT64)(((FLOAT64)(*quant_pred_fac_mat_ptr++) + 0.5) *
                    pow(2.0, -(WORD32)(pstr_dir_pre_dom_syn->scale_fac_num_bits) + 1));
      *pred_fac_mat_ptr++ = dequantized_factor;
    }
  }

  is_spat_pred_lst_frm = 0;
  is_spat_pred_cur_frm = 0;

  if (ip_ps_sigs_frm_sz == 0)
  {
    memset(pstr_dir_pre_dom_syn->pred_grid_dir_sigs_lst_frm, 0,
           sizeof(FLOAT64) * MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN);
    memset(pstr_dir_pre_dom_syn->pred_grid_dir_sigs_cur_frm, 0,
           sizeof(FLOAT64) * MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN);
  }
  else
  {
    impeghe_hoa_compute_hoa_representation_of_spat_pred(
        pstr_se_handle, pstr_dir_pre_dom_syn->lst_pred_typ_vec,
        pstr_dir_pre_dom_syn->lst_pred_idx_mat, pstr_dir_pre_dom_syn->lst_pred_fac_mat,
        ptr_dir_fade_out_win, pstr_dir_pre_dom_syn->pred_grid_dir_sigs_lst_frm,
        &is_spat_pred_lst_frm);

    impeghe_hoa_compute_hoa_representation_of_spat_pred(
        pstr_se_handle, pred_typ_vec, pred_idx_mat, pstr_dir_pre_dom_syn->pred_fac_mat,
        ptr_dir_fade_in_win, pstr_dir_pre_dom_syn->pred_grid_dir_sigs_cur_frm,
        &is_spat_pred_cur_frm);
  }

  for (k = 0; k < num_coeffs; k++)
  {

    pFlOAT64 ptr_out_pred_grid_dir_smth_frm_temp, pred_grid_dir_sig_lst_frm_ptr;
    pFlOAT64 pred_grid_dir_sig_cur_frm_ptr;

    for (i = 0; i < MAX_NUM_HOA_COEFFS; i++)
    {
      if (ptr_hoa_coeff_indices[i] == (k + 1))
        break;
    }

    ptr_out_pred_grid_dir_smth_frm_temp = ptr_out_pred_grid_dir_smth_frm + k * frm_sz;
    pred_grid_dir_sig_lst_frm_ptr = pstr_dir_pre_dom_syn->pred_grid_dir_sigs_lst_frm + k * frm_sz;
    pred_grid_dir_sig_cur_frm_ptr = pstr_dir_pre_dom_syn->pred_grid_dir_sigs_cur_frm + k * frm_sz;
    if (i == MAX_NUM_HOA_COEFFS)
    {
      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_pred_grid_dir_smth_frm_temp++ =
            (*pred_grid_dir_sig_lst_frm_ptr++) + (*pred_grid_dir_sig_cur_frm_ptr++);
      }
    }
    else
    {
      memset(ptr_out_pred_grid_dir_smth_frm_temp, 0, sizeof(FLOAT64) * frm_sz);
    }
  }

  if (is_spat_pred_cur_frm)
  {
    for (i = 0; (WORD32)ptr_amb_coeff_indices_enabled[i] != -1; i++)
    {
      UWORD32 coeff_idx_for_pred_hoa_to_be_faded_out;
      pFlOAT64 ptr_out_pred_grid_dir_smth_frm_temp;

      if (i == MAX_NUM_HOA_COEFFS)
        break;

      coeff_idx_for_pred_hoa_to_be_faded_out = ptr_amb_coeff_indices_enabled[i] - 1;
      ptr_out_pred_grid_dir_smth_frm_temp =
          ptr_out_pred_grid_dir_smth_frm + coeff_idx_for_pred_hoa_to_be_faded_out * frm_sz;

      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_pred_grid_dir_smth_frm_temp *= ptr_dir_fade_out_win[s];
        ptr_out_pred_grid_dir_smth_frm_temp++;
      }
    }
  }

  if (is_spat_pred_lst_frm)
  {
    for (i = 0; (WORD32)ptr_amb_coeff_indices_disabled[i] != -1; i++)
    {
      UWORD32 coeff_idx_for_pred_hoa_to_be_faded_in;
      pFlOAT64 ptr_out_pred_grid_dir_smth_frm_temp;

      if (i == MAX_NUM_HOA_COEFFS)
        break;

      coeff_idx_for_pred_hoa_to_be_faded_in = ptr_amb_coeff_indices_disabled[i] - 1;
      ptr_out_pred_grid_dir_smth_frm_temp =
          ptr_out_pred_grid_dir_smth_frm + coeff_idx_for_pred_hoa_to_be_faded_in * frm_sz;

      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_pred_grid_dir_smth_frm_temp *= ptr_dir_fade_in_win[s];
        ptr_out_pred_grid_dir_smth_frm_temp++;
      }
    }
  }

  for (i = 0; i < pstr_predom_syn->num_hoa_coeffs; i++)
  {
    pstr_dir_pre_dom_syn->lst_pred_typ_vec[i] = pred_typ_vec[i];
  }

  ptr_lst_pred_idx_mat = pstr_dir_pre_dom_syn->lst_pred_idx_mat;
  ptr_lst_pred_fac_mat = pstr_dir_pre_dom_syn->lst_pred_fac_mat;
  pred_fac_mat_ptr = pstr_dir_pre_dom_syn->pred_fac_mat;
  ptr_pred_idx_mat = pred_idx_mat;

  for (l = 0; l < min_pred_dir_sig; l++)
  {
    for (i = 0; i < num_coeffs; i++)
    {
      *ptr_lst_pred_idx_mat++ = *ptr_pred_idx_mat++;
      *ptr_lst_pred_fac_mat++ = *pred_fac_mat_ptr++;
    }
  }
  return;
}
/**
 *  impeghe_hoa_dir_based_pre_dom_sound_syn_process
 *
 *  \brief Direction based predominant sound synthesis processing
 *
 *  \param [in,out] pstr_se_handle Spatial Encoder handle
 *  \param [in] ptr_amb_coeff_indices_enabled Ambient coefficients to be enabled
 *  \param [in] ptr_amb_coeff_indices_disabled Ambient coefficients to be disabled
 *  \param [in] ptr_hoa_coeff_indices Active HOA coefficients that are not
 * enabled/disabled
 *  \param [in] ptr_ps_sig_indices set of ps signal indices
 *
 *  \return VOID
 */
VOID impeghe_hoa_dir_based_pre_dom_sound_syn_process(
    ia_spatial_enc_str *pstr_se_handle, const pUWORD32 ptr_amb_coeff_indices_enabled,
    const pUWORD32 ptr_amb_coeff_indices_disabled, const pUWORD32 ptr_hoa_coeff_indices,
    const pUWORD32 ptr_ps_sig_indices)
{
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_predom_syn =
      &(pstr_se_handle->decmp_hdl.pre_dom_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *pstr_dir_pre_dom_syn =
      &(pstr_predom_syn->dir_predom_syn_handle);
  pFlOAT64 ptr_out_dir_predom_frm = pstr_se_handle->penult_smtd_pre_dom_frm;
  pFlOAT64 ptr_out_smt_dir_sigs_frm =
      pstr_se_handle->decmp_hdl.dir_smtd_frame[pstr_se_handle->decmp_hdl.dir_smtd_frame_idx];
  UWORD32 num_coeffs = pstr_dir_pre_dom_syn->num_hoa_coeffs;
  UWORD32 frm_sz = pstr_dir_pre_dom_syn->frame_size;
  pFlOAT64 ptr_pred_grid_dir_digs_smt_frm = pstr_dir_pre_dom_syn->pred_grid_dir_digs_smt_frm;
  ULOOPIDX i, k, s;

  impeghe_hoa_compute_hoa_repr_of_dir_sigs(pstr_se_handle, ptr_ps_sig_indices);

  impeghe_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs(
      pstr_se_handle, ptr_amb_coeff_indices_enabled, ptr_amb_coeff_indices_disabled,
      ptr_hoa_coeff_indices);

  memcpy(ptr_out_dir_predom_frm, ptr_out_smt_dir_sigs_frm,
         sizeof(FLOAT64) * (num_coeffs) * (frm_sz));

  i = 0;
  for (k = 0; k < num_coeffs; k++)
  {
    for (s = 0; s < frm_sz; s++)
    {
      FLOAT64 temp = ptr_out_dir_predom_frm[i] + ptr_pred_grid_dir_digs_smt_frm[i];
      ptr_out_dir_predom_frm[i] = temp;
      i++;
    }
  }

  return;
}
