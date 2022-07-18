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
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_hoa_pre_dom_sound_syn.h"

/**
 *  impeghe_hoa_decomposition_process
 *
 *  \brief HOA signal decompostion
 *
 *  \param [in] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_ip_hoa_frame Input HOA signals
 *
 *  \return VOID
 */
static const UWORD32 u8_quantizer_word = 5;
VOID impeghe_hoa_decomposition_process(ia_spatial_enc_str *pstr_se_handle,
                                       const pFlOAT64 ptr_ip_hoa_frame)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_se_handle->decmp_hdl);
  const ia_hoa_dir_id_str *ptr_ip_sig_n_dir_id_map = pstr_frm_prm->active_n_grid_dir_idx;
  const ia_hoa_vec_sig_str *ptr_in_vec_map = pstr_frm_prm->vectors;
  const WORD32 ip_vec_sz = pstr_frm_prm->vec_sz;
  WORD32 fine_grid_mat_sz = pstr_decomp_handle->fine_grid_transp_mode_mat_sz;
  pUWORD32 op_pred_idx_mat = pstr_frm_prm->pred_idx_mat;
  pWORD32 op_quant_pred_fac_mat = pstr_frm_prm->quant_pred_fac_mat;
  ia_spatial_enc_assignment_info_str *pstr_target_asgmt_vec = pstr_se_handle->target_assgn_vec;

  WORD32 num_coeffs = pstr_decomp_handle->num_hoa_coeffs;
  WORD32 frm_sz = pstr_decomp_handle->frame_size;
  WORD32 interp_samples = pstr_decomp_handle->interp_samples;
  WORD32 var_ch = pstr_decomp_handle->num_var_ch;
  WORD32 dir_max_pred = pstr_decomp_handle->max_pred_dir_sigs;
  WORD32 vec_start_ch = pstr_decomp_handle->vec_start_ch;
  ia_hoa_vec_sig_str *pstr_lst_vec = pstr_decomp_handle->lst_vec;
  ia_hoa_dir_id_str *pstr_lst_sig_map = pstr_decomp_handle->lst_sig_idx_map;
  pUWORD32 ip_idx = &(pstr_decomp_handle->ip_idx);
  pUWORD32 penult_ip_idx = &(pstr_decomp_handle->penult_ip_idx);
  pUWORD32 lst_ip_idx = &(pstr_decomp_handle->lst_ip_idx);
  pUWORD32 dir_smtd_frame_idx = &(pstr_decomp_handle->dir_smtd_frame_idx);
  pUWORD32 dir_smtd_lst_frm_idx = &(pstr_decomp_handle->dir_smtd_lst_frame_idx);
  pUWORD32 vec_smtd_frm_idx = &(pstr_decomp_handle->vec_smtd_frame_idx);
  pUWORD32 vec_smtd_lst_frm_idx = &(pstr_decomp_handle->vec_smtd_lst_frame_idx);
  WORD32 id, ch, s, j, k, i;
  pFlOAT64 ptr_smth_dir_sigs_lst_frm_ptr;
  pFlOAT64 ptr_smth_dir_sigs_frm, ptr_smth_vec_sigs_lst_frm;
  pFlOAT64 ptr_smth_vec_sigs_frm, ptr_out_penult_amb_hoa_frm;
  pFlOAT64 ptr_out_pred_lst_amb_hoa_frm, ptr_last_hoa_frame_temp;
  pFlOAT64 ptr_penult_frm;
  pUWORD32 ptr_out_pred_idx_mat;
  pWORD32 ptr_out_quant_pred_fac_mat;
  pFlOAT64 ptr_last_hoa_frame = pstr_decomp_handle->ip_hoa_frame[*lst_ip_idx];
  pFlOAT64 ptr_penult_hoa_frm = pstr_decomp_handle->ip_hoa_frame[*penult_ip_idx];

  pstr_decomp_handle->scratch_used_size = pstr_se_handle->scratch_used_size;
  pstr_decomp_handle->ptr_scratch = pstr_se_handle->ptr_scratch;
  memcpy(pstr_se_handle->trans_ch[pstr_se_handle->scd_lst_ip_idx],
         pstr_decomp_handle->ps_sigs_smtd_lst_frame, sizeof(FLOAT64) * (var_ch) * (frm_sz));

  memset(pstr_decomp_handle->ps_sigs_smtd_lst_frame, 0, sizeof(FLOAT64) * (var_ch) * (frm_sz));

  for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
  {
    if (-1 != pstr_lst_sig_map[id].ch_idx)
    {
      UWORD32 curr_vec_idx;
      pFlOAT64 curr_face_win;
      UWORD32 curr_dir_idx = pstr_lst_sig_map[id].ch_idx - 1;
      UWORD32 curr_grid_dir_idx = pstr_lst_sig_map[id].dir_id - 1;

      pFlOAT64 pseudo_inv_mode_vec =
          (pFlOAT64)(&pstr_decomp_handle
                          ->fine_grid_transp_mode_mat_inv[curr_grid_dir_idx * fine_grid_mat_sz]);
      curr_face_win = pstr_decomp_handle->dir_sigs_cross_fade_win;
      for (curr_vec_idx = 0; curr_vec_idx < MAX_NUMBER_CHANNELS; curr_vec_idx++)
      {
        if ((curr_dir_idx + 1) == ptr_in_vec_map[curr_vec_idx].index)
          break;
      }

      if (MAX_NUMBER_CHANNELS != curr_vec_idx)
      {
        curr_face_win = pstr_decomp_handle->vec_sigs_cross_fade_win;
      }

      for (s = 0; s < frm_sz; s++)
      {
        FLOAT64 curr_contribution = (FLOAT64)(0.0);

        for (j = 0; j < num_coeffs; j++)
        {
          curr_contribution += pseudo_inv_mode_vec[j] * ptr_last_hoa_frame[(j * frm_sz) + s];
        }

        curr_contribution *= curr_face_win[frm_sz + s];

        pstr_decomp_handle->ps_sigs_smtd_lst_frame[(curr_dir_idx * frm_sz) + s] +=
            curr_contribution;
      }
    }
    else
      break;
  }

  for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
  {
    if (-1 != pstr_lst_vec[id].index)
    {
      pFlOAT64 curr_face_win;
      UWORD32 ui_curr_vec_idx = pstr_lst_vec[id].index - 1;
      const pFlOAT64 curr_vec = pstr_lst_vec[id].sig_id;

      pFlOAT64 pseudo_inv_mode_vec = (pFlOAT64)((pUWORD8)pstr_decomp_handle->ptr_scratch +
                                                pstr_decomp_handle->scratch_used_size);

      impeghe_hoa_compute_pseudo_inverse_vec(curr_vec, ip_vec_sz, pseudo_inv_mode_vec);
      curr_face_win = pstr_decomp_handle->vec_sigs_cross_fade_win;
      for (s = 0; s < interp_samples; s++)
      {
        FLOAT64 curr_contribution = (FLOAT64)(0.0);

        for (j = vec_start_ch; j < num_coeffs; j++)
        {
          curr_contribution +=
              pseudo_inv_mode_vec[j - vec_start_ch] * ptr_last_hoa_frame[(j * frm_sz) + s];
        }

        curr_contribution *= curr_face_win[frm_sz + s];

        pstr_decomp_handle->ps_sigs_smtd_lst_frame[(ui_curr_vec_idx * frm_sz) + s] +=
            curr_contribution;
      }
    }
    else
      break;
  }

  for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
  {
    if (-1 != ptr_ip_sig_n_dir_id_map[id].ch_idx)
    {
      pFlOAT64 curr_face_win;
      UWORD32 curr_dir_idx = ptr_ip_sig_n_dir_id_map[id].ch_idx - 1;
      UWORD32 curr_grid_dir_idx = ptr_ip_sig_n_dir_id_map[id].dir_id - 1;

      pFlOAT64 pseudo_inv_mode_vec =
          (pFlOAT64)(&pstr_decomp_handle
                          ->fine_grid_transp_mode_mat_inv[curr_grid_dir_idx * fine_grid_mat_sz]);

      curr_face_win = pstr_decomp_handle->dir_sigs_cross_fade_win;

      for (s = 0; s < frm_sz; s++)
      {
        FLOAT64 curr_contribution = (FLOAT64)(0.0);

        for (j = 0; j < num_coeffs; j++)
        {
          curr_contribution += pseudo_inv_mode_vec[j] * ptr_last_hoa_frame[(j * frm_sz) + s];
        }

        curr_contribution *= curr_face_win[s];

        pstr_decomp_handle->ps_sigs_smtd_lst_frame[(curr_dir_idx * frm_sz) + s] +=
            curr_contribution;
      }
    }
    else
      break;
  }

  for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
  {
    if (-1 != ptr_in_vec_map[id].index)
    {
      pFlOAT64 curr_face_win;
      UWORD32 ui_curr_vec_idx = ptr_in_vec_map[id].index - 1;
      pFlOAT64 curr_vec = (pFlOAT64)ptr_in_vec_map[id].sig_id;

      pFlOAT64 pseudo_inv_mode_vec = (pFlOAT64)((pUWORD8)pstr_decomp_handle->ptr_scratch +
                                                pstr_decomp_handle->scratch_used_size);

      impeghe_hoa_compute_pseudo_inverse_vec(curr_vec, ip_vec_sz, pseudo_inv_mode_vec);

      curr_face_win = pstr_decomp_handle->vec_sigs_cross_fade_win;

      for (s = 0; s < frm_sz; s++)
      {
        FLOAT64 curr_contribution = (FLOAT64)(0.0);

        for (j = vec_start_ch; j < num_coeffs; j++)
        {
          curr_contribution +=
              pseudo_inv_mode_vec[j - vec_start_ch] * ptr_last_hoa_frame[(j * frm_sz) + s];
        }

        curr_contribution *= curr_face_win[s];

        pstr_decomp_handle->ps_sigs_smtd_lst_frame[(ui_curr_vec_idx * frm_sz) + s] +=
            curr_contribution;
      }
    }
    else
      break;
  }

  ch = *dir_smtd_lst_frm_idx;
  *dir_smtd_lst_frm_idx = *dir_smtd_frame_idx;
  *dir_smtd_frame_idx = ch;

  ch = *vec_smtd_lst_frm_idx;
  *vec_smtd_lst_frm_idx = *vec_smtd_frm_idx;
  *vec_smtd_frm_idx = ch;

  for (j = 0; j < num_coeffs; j++)
  {
    pstr_frm_prm->pred_typ_vec[j] = 0;
  }

  ptr_out_pred_idx_mat = op_pred_idx_mat;
  for (k = 0; k < dir_max_pred; k++)
  {
    for (j = 0; j < num_coeffs; j++)
    {
      *ptr_out_pred_idx_mat++ = 0;
    }
  }

  ptr_out_quant_pred_fac_mat = op_quant_pred_fac_mat;
  for (k = 0; k < dir_max_pred; k++)
  {
    for (j = 0; j < num_coeffs; j++)
    {
      *ptr_out_quant_pred_fac_mat++ = 0;
    }
  }

  impeghe_hoa_pre_dom_sound_syn_process(pstr_se_handle, pstr_frm_prm->amb_coeff_idx_to_enb,
                                        pstr_frm_prm->amb_coeff_idx_to_disable,
                                        pstr_frm_prm->non_en_dis_able_act_idx);

  if (pstr_decomp_handle->vec_factor)
  {
    WORD32 min_coeffs_for_amb_hoa = pstr_se_handle->amb_comp_hdl.num_amb_hoa_coeffs;

    ptr_out_penult_amb_hoa_frm = pstr_se_handle->penult_amb_hoa_frm;
    ptr_penult_frm = ptr_penult_hoa_frm;
    ptr_smth_dir_sigs_lst_frm_ptr = pstr_decomp_handle->dir_smtd_frame[*dir_smtd_lst_frm_idx];
    ptr_smth_vec_sigs_lst_frm = pstr_decomp_handle->vec_smtd_frame[*vec_smtd_lst_frm_idx];

    ptr_out_pred_lst_amb_hoa_frm = pstr_se_handle->pred_lst_amb_hoa_frm;
    ptr_last_hoa_frame_temp = ptr_last_hoa_frame;
    ptr_smth_dir_sigs_frm = pstr_decomp_handle->dir_smtd_frame[*dir_smtd_frame_idx];
    ptr_smth_vec_sigs_frm = pstr_decomp_handle->vec_smtd_frame[*vec_smtd_frm_idx];

    for (j = 0; j < min_coeffs_for_amb_hoa; j++)
    {
      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_penult_amb_hoa_frm++ =
            *ptr_penult_frm++ - *ptr_smth_dir_sigs_lst_frm_ptr++ - *ptr_smth_vec_sigs_lst_frm++;
        *ptr_out_pred_lst_amb_hoa_frm++ =
            *ptr_last_hoa_frame_temp++ - *ptr_smth_dir_sigs_frm++ - *ptr_smth_vec_sigs_frm++;
      }
    }
    ptr_out_pred_lst_amb_hoa_frm =
        pstr_se_handle->modfd_pred_lst_amb_hoa_frm + min_coeffs_for_amb_hoa * (frm_sz);
    ptr_out_penult_amb_hoa_frm =
        pstr_se_handle->modfd_penult_amb_hoa_frm + min_coeffs_for_amb_hoa * (frm_sz);

    for (; j < num_coeffs; j++)
    {
      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_penult_amb_hoa_frm++ =
            *ptr_penult_frm++ - *ptr_smth_dir_sigs_lst_frm_ptr++ - *ptr_smth_vec_sigs_lst_frm++;
        *ptr_out_pred_lst_amb_hoa_frm++ =
            *ptr_last_hoa_frame_temp++ - *ptr_smth_dir_sigs_frm++ - *ptr_smth_vec_sigs_frm++;
      }
    }
  }
  else
  {
    WORD32 min_coeffs_for_amb_hoa = pstr_se_handle->amb_comp_hdl.num_amb_hoa_coeffs;
    ptr_out_penult_amb_hoa_frm = pstr_se_handle->penult_amb_hoa_frm;
    ptr_penult_frm = ptr_penult_hoa_frm;
    ptr_smth_dir_sigs_lst_frm_ptr = pstr_decomp_handle->dir_smtd_frame[*dir_smtd_lst_frm_idx];

    ptr_out_pred_lst_amb_hoa_frm = pstr_se_handle->pred_lst_amb_hoa_frm;
    ptr_last_hoa_frame_temp = ptr_last_hoa_frame;
    ptr_smth_dir_sigs_frm = pstr_decomp_handle->dir_smtd_frame[*dir_smtd_frame_idx];

    for (j = 0; j < min_coeffs_for_amb_hoa; j++)
    {
      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_penult_amb_hoa_frm++ = *ptr_penult_frm++ - *ptr_smth_dir_sigs_lst_frm_ptr++;
        *ptr_out_pred_lst_amb_hoa_frm++ = *ptr_last_hoa_frame_temp++ - *ptr_smth_dir_sigs_frm++;
      }
    }
    ptr_out_pred_lst_amb_hoa_frm =
        pstr_se_handle->modfd_pred_lst_amb_hoa_frm + min_coeffs_for_amb_hoa * (frm_sz);
    ptr_out_penult_amb_hoa_frm =
        pstr_se_handle->modfd_penult_amb_hoa_frm + min_coeffs_for_amb_hoa * (frm_sz);

    for (; j < num_coeffs; j++)
    {
      for (s = 0; s < frm_sz; s++)
      {
        *ptr_out_penult_amb_hoa_frm++ = *ptr_penult_frm++ - *ptr_smth_dir_sigs_lst_frm_ptr++;
        *ptr_out_pred_lst_amb_hoa_frm++ = *ptr_last_hoa_frame_temp++ - *ptr_smth_dir_sigs_frm++;
      }
    }
  }

  for (ch = 0; ch < var_ch; ch++)
  {
    UWORD32 inactive_dir_ids = 0;
    UWORD32 input_vectors_id = 0;
    UWORD32 last_vectors_id = 0;
    WORD32 is_dir_currently_active = 0;
    WORD32 was_dir_active_before = 0;
    WORD32 is_vec_currently_active = 0;
    WORD32 was_vec_active_before = 0;

    for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
    {
      if ((ch + 1) == (UWORD32)ptr_ip_sig_n_dir_id_map[id].ch_idx)
        break;
    }
    if (MAX_NUMBER_CHANNELS != id)
    {
      is_dir_currently_active = 1;
      inactive_dir_ids = id;
    }

    for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
    {
      if ((ch + 1) == (UWORD32)pstr_lst_sig_map[id].ch_idx)
        break;
    }
    if (MAX_NUMBER_CHANNELS != id)
    {
      was_dir_active_before = 1;
    }

    for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
    {
      if ((ch + 1) == ptr_in_vec_map[id].index)
        break;
    }
    if (MAX_NUMBER_CHANNELS != id)
    {
      input_vectors_id = id;
      is_vec_currently_active = 1;
    }

    for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
    {
      if ((ch + 1) == pstr_lst_vec[id].index)
        break;
    }
    if (MAX_NUMBER_CHANNELS != id)
    {
      last_vectors_id = id;
      was_vec_active_before = 1;
    }

    if (is_dir_currently_active || was_dir_active_before || is_vec_currently_active ||
        was_vec_active_before)
    {
      pstr_target_asgmt_vec[ch].is_available = 1;
      pstr_target_asgmt_vec[ch].bit_rate = pstr_decomp_handle->br_per_coder;

      if (is_dir_currently_active ||
          (was_dir_active_before && !is_dir_currently_active && !is_vec_currently_active))
      {
        impeghe_hoa_info_ch_str *p_dir_c_info = &(pstr_target_asgmt_vec[ch].ch_info);
        pstr_target_asgmt_vec[ch].ch_type = HOA_DIR_CHANNEL;

        if (is_dir_currently_active)
        {
          p_dir_c_info->active_dir_ids = ptr_ip_sig_n_dir_id_map[inactive_dir_ids].dir_id;
        }
        else
        {
          p_dir_c_info->active_dir_ids = 0;
        }
      }

      if (is_vec_currently_active ||
          (was_vec_active_before && !is_dir_currently_active && !is_vec_currently_active))
      {
        pFlOAT64 ptr_curr_vec;
        impeghe_hoa_vector_based_info_ch_str *pstr_vec_ch_info =
            &(pstr_target_asgmt_vec[ch].vec_based_ch_info);
        pstr_target_asgmt_vec[ch].ch_type = HOA_VEC_CHANNEL;

        if (u8_quantizer_word == pstr_vec_ch_info->us_bits_q)
        {
          pstr_vec_ch_info->same_header_prev_frame = 1;
        }
        else
        {
          pstr_vec_ch_info->same_header_prev_frame = 0;
          pstr_vec_ch_info->us_bits_q = (UWORD16)u8_quantizer_word;
        }

        if (is_vec_currently_active)
        {
          ptr_curr_vec = (pFlOAT64)ptr_in_vec_map[input_vectors_id].sig_id;
        }
        else
        {
          ptr_curr_vec = pstr_lst_vec[last_vectors_id].sig_id;
        }

        for (i = 0; i < num_coeffs - vec_start_ch; i++)
        {
          FLOAT64 curr_scaled_elem = ptr_curr_vec[i];
          curr_scaled_elem /= (FLOAT64)(pstr_decomp_handle->hoa_order + 1);
          pstr_vec_ch_info->ui_8bit_coded_v_element[i] = impeghe_hoa_quantize_uniform(
              curr_scaled_elem, pstr_decomp_handle->vec_ele_num_bits);
        }
      }
    }
    else
    {
      impeghe_hoa_assignment_info_init(&(pstr_target_asgmt_vec[ch]));
    }
  }

  for (id = 0; id < MAX_NUMBER_CHANNELS; id++)
  {
    pstr_lst_sig_map[id].ch_idx = ptr_ip_sig_n_dir_id_map[id].ch_idx;
    pstr_lst_sig_map[id].dir_id = ptr_ip_sig_n_dir_id_map[id].dir_id;

    pstr_lst_vec[id].index = ptr_in_vec_map[id].index;
    for (i = 0; i < ip_vec_sz; i++)
    {
      pstr_lst_vec[id].sig_id[i] = ptr_in_vec_map[id].sig_id[i];
    }
  }

  if (ptr_ip_hoa_frame)
  {
    ch = *penult_ip_idx;
    *penult_ip_idx = *lst_ip_idx;
    *lst_ip_idx = *ip_idx;
    *ip_idx = ch;
  }
  else
  {
    *penult_ip_idx = *lst_ip_idx;
    *lst_ip_idx = *ip_idx;
    memset(pstr_decomp_handle->ip_hoa_frame[*lst_ip_idx], 0,
           sizeof(FLOAT64) * frm_sz * num_coeffs);
  }
  return;
}
