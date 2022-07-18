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

/**
 *  impeghe_hoa_vector_based_predom_sound_syn_process
 *
 *  \brief Vector-based predominant sound sythesis
 *
 *  \param [in] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_set_of_ps_sig_indices Set of indices corresponding to ps signals
 *  \param [in] ptr_amb_coef_indices_enable Ambient coefficient indices to enable
 *  \param [in] ptr_amb_coef_indices_disable Ambient coefficient indices to disable
 *
 *  \return VOID
 */
VOID impeghe_hoa_vector_based_predom_sound_syn_process(
    ia_spatial_enc_str *pstr_se_handle, const pUWORD32 ptr_set_of_ps_sig_indices,
    const pUWORD32 ptr_amb_coef_indices_enable, const pUWORD32 ptr_amb_coef_indices_disable)
{
  UWORD32 interp_samples;
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  impeghe_hoa_decomposition_str *pstr_decomp_handle = &(pstr_se_handle->decmp_hdl);
  ia_spatial_dec_pre_dom_sound_syn_str *pstr_predom_syn =
      &(pstr_decomp_handle->pre_dom_syn_handle);
  ia_spatial_dec_vector_based_predom_sound_syn_str *pstr_vec_pre_dom_syn =
      &(pstr_predom_syn->vec_predom_syn_handle);

  const pFlOAT64 ptr_in_dir_sig_frame = pstr_se_handle->ps_sigs_smtd_lst_frame;
  UWORD32 frm_sz = pstr_vec_pre_dom_syn->frame_size;
  pFlOAT64 ptr_out_vec_predom_frm =
      pstr_decomp_handle->vec_smtd_frame[pstr_decomp_handle->vec_smtd_frame_idx];
  const ia_hoa_vec_sig_str *pstr_vectors = pstr_frm_prm->vectors;
  const pFlOAT64 ptr_dir_sig_fade_in_win = pstr_predom_syn->dir_sig_fade_in_win;
  const pFlOAT64 ptr_dir_sig_fade_out_win = pstr_predom_syn->dir_sig_fade_out_win;
  const pFlOAT64 ptr_vec_sig_fade_in_win = pstr_predom_syn->vec_sig_fade_win;
  const pFlOAT64 ptr_vec_sig_fade_out_win = &(pstr_predom_syn->vec_sig_fade_win[frm_sz]);
  ia_hoa_vec_sig_str *pstr_prev_vec = pstr_vec_pre_dom_syn->prev_vec;
  UWORD32 num_coeff = pstr_vec_pre_dom_syn->num_hoa_coeff;
  UWORD32 vec_st_ch = pstr_vec_pre_dom_syn->vector_start_ch;
  pUWORD32 prev_ps_sig_idx = pstr_vec_pre_dom_syn->prev_ps_sig_idx;
  pFlOAT64 ptr_out_vec_predom_frm_temp;
  ULOOPIDX i, j, k, l;
  ULOOPIDX map_it, set_it;

  memset(ptr_out_vec_predom_frm, 0, sizeof(FLOAT64) * frm_sz * num_coeff);

  for (i = 0; (WORD32)pstr_prev_vec[i].index != -1; i++)
  {
    UWORD32 vec_sig_idx, it;
    pFlOAT64 ptr_curr_vec;

    if (i == MAX_FRAME_LEN)
      break;

    vec_sig_idx = pstr_prev_vec[i].index;
    ptr_curr_vec = pstr_prev_vec[i].sig_id;

    for (it = 0; it < MAX_NUM_HOA_DIR_SIGNALS; it++)
    {
      if (ptr_set_of_ps_sig_indices[it] == vec_sig_idx)
        break;
    }

    if (it != MAX_NUM_SIG_INDICES)
    {
      pFlOAT64 ptr_curr_interp_win = &(ptr_dir_sig_fade_out_win[0]);
      interp_samples = frm_sz;
      for (map_it = 0; map_it < MAX_NUM_VECTOR_SIG_INDICES; map_it++)
      {
        if (pstr_vectors[map_it].index == vec_sig_idx)
          break;
      }

      if (map_it != MAX_NUM_VECTOR_SIG_INDICES)
      {
        ptr_curr_interp_win = &(ptr_vec_sig_fade_out_win[0]);
        interp_samples = pstr_vec_pre_dom_syn->interp_samples;
      }

      for (k = vec_st_ch; k < num_coeff; k++)
      {
        pFlOAT64 ptr_in_dir_sig_frame_temp =
            (pFlOAT64)(ptr_in_dir_sig_frame + (vec_sig_idx - 1) * frm_sz);
        ptr_out_vec_predom_frm_temp = ptr_out_vec_predom_frm + (k) * (frm_sz);
        for (l = 0; l < interp_samples; l++)
        {
          FLOAT64 interpolated_vec = ptr_curr_vec[k - vec_st_ch] * ptr_curr_interp_win[l];
          FLOAT64 curr_hoa_contribution = interpolated_vec * (*ptr_in_dir_sig_frame_temp++);
          ptr_out_vec_predom_frm_temp[l] += curr_hoa_contribution;
        }
      }
    }
  }

  for (map_it = 0; (WORD32)pstr_vectors[map_it].index != -1; map_it++)
  {
    UWORD32 vec_sig_idx;
    pFlOAT64 ptr_curr_vec, ptr_curr_interp_win;

    if (map_it == MAX_NUM_VECTOR_SIG_INDICES)
      break;

    vec_sig_idx = pstr_vectors[map_it].index;
    ptr_curr_vec = (pFlOAT64)pstr_vectors[map_it].sig_id;

    ptr_curr_interp_win = NULL;
    for (set_it = 0; set_it < MAX_NUM_HOA_DIR_SIGNALS; set_it++)
    {
      if (pstr_vec_pre_dom_syn->prev_ps_sig_idx[set_it] == vec_sig_idx)
        break;
    }

    if (set_it != MAX_NUM_SIG_INDICES)
    {
      ptr_curr_interp_win = &(ptr_vec_sig_fade_in_win[0]);
    }

    ptr_out_vec_predom_frm_temp = ptr_out_vec_predom_frm + (vec_st_ch) * (frm_sz);
    if (NULL == ptr_curr_interp_win)
    {
      for (k = vec_st_ch; k < num_coeff; k++)
      {
        pFlOAT64 ptr_in_dir_sig_frame_temp =
            (pFlOAT64)(ptr_in_dir_sig_frame + (vec_sig_idx - 1) * frm_sz);
        FLOAT64 interpolated_vec = ptr_curr_vec[k - vec_st_ch];
        for (l = 0; l < frm_sz; l++)
        {
          FLOAT64 curr_hoa_contribution = interpolated_vec * (*ptr_in_dir_sig_frame_temp++);
          *ptr_out_vec_predom_frm_temp++ += curr_hoa_contribution;
        }
      }
    }
    else
    {
      for (k = vec_st_ch; k < num_coeff; k++)
      {
        pFlOAT64 ptr_in_dir_sig_frame_temp =
            (pFlOAT64)(ptr_in_dir_sig_frame + (vec_sig_idx - 1) * frm_sz);
        for (l = 0; l < frm_sz; l++)
        {
          FLOAT64 interpolated_vec = ptr_curr_vec[k - vec_st_ch] * ptr_curr_interp_win[l];
          FLOAT64 curr_hoa_contribution = interpolated_vec * (*ptr_in_dir_sig_frame_temp++);
          *ptr_out_vec_predom_frm_temp++ += curr_hoa_contribution;
        }
      }
    }
  }

  if (pstr_vec_pre_dom_syn->coded_vec_len == 1)
  {

    for (set_it = 0; (WORD32)ptr_amb_coef_indices_enable[set_it] != -1; set_it++)
    {
      UWORD32 coeff_idx_to_be_faded_out;

      if (set_it == MAX_NUM_AMB_COEFF_INDICES)
        break;
      coeff_idx_to_be_faded_out = ptr_amb_coef_indices_enable[set_it] - 1;
      ptr_out_vec_predom_frm_temp = ptr_out_vec_predom_frm + coeff_idx_to_be_faded_out * (frm_sz);

      for (l = 0; l < frm_sz; l++)
      {
        *ptr_out_vec_predom_frm_temp++ *= ptr_dir_sig_fade_out_win[l];
      }
    }

    for (set_it = 0; (WORD32)ptr_amb_coef_indices_disable[set_it] != -1; set_it++)
    {
      UWORD32 coeff_idx_to_be_faded_in;

      if (set_it == MAX_NUM_AMB_COEFF_INDICES)
        break;

      coeff_idx_to_be_faded_in = ptr_amb_coef_indices_disable[set_it] - 1;
      ptr_out_vec_predom_frm_temp = ptr_out_vec_predom_frm + coeff_idx_to_be_faded_in * (frm_sz);

      for (l = 0; l < frm_sz; l++)
      {
        *ptr_out_vec_predom_frm_temp++ *= ptr_dir_sig_fade_in_win[l];
      }
    }

    for (j = 0; j < MAX_NUM_VECTOR_SIG_INDICES; j++)
    {
      pstr_prev_vec[j].index = (UWORD32)-1;
    }
    for (map_it = 0; (WORD32)pstr_vectors[map_it].index != -1; map_it++)
    {
      UWORD32 vec_sig_idx;
      pFlOAT64 ptr_curr_vec;

      if (map_it == MAX_NUM_VECTOR_SIG_INDICES)
        break;

      vec_sig_idx = pstr_vectors[map_it].index;
      ptr_curr_vec = (pFlOAT64)pstr_vectors[map_it].sig_id;

      pstr_prev_vec[map_it].index = vec_sig_idx;

      for (j = 0; j < MAX_FRAME_LEN; j++)
        pstr_prev_vec[map_it].sig_id[j] = ptr_curr_vec[j];

      for (j = 0; (WORD32)ptr_amb_coef_indices_enable[j] != -1; j++)
      {
        UWORD32 coeff_idx_to_be_faded_out;

        if (j == MAX_NUM_AMB_COEFF_INDICES)
          break;

        coeff_idx_to_be_faded_out = ptr_amb_coef_indices_enable[j] - 1;
        pstr_prev_vec[map_it].sig_id[coeff_idx_to_be_faded_out - vec_st_ch] = 0;
      }
    }
    for (; map_it < MAX_NUM_VECTOR_SIG_INDICES; map_it++)
    {
      pstr_prev_vec[map_it].index = (UWORD32)-1;
    }
  }
  else
  {
    for (map_it = 0; (WORD32)pstr_vectors[map_it].index != -1; map_it++)
    {
      if (map_it == MAX_NUM_VECTOR_SIG_INDICES)
        break;

      pstr_prev_vec[map_it].index = pstr_vectors[map_it].index;

      for (j = 0; j < MAX_FRAME_LEN; j++)
        pstr_prev_vec[map_it].sig_id[j] = pstr_vectors[map_it].sig_id[j];
    }
    for (; map_it < MAX_NUM_VECTOR_SIG_INDICES; map_it++)
    {
      pstr_prev_vec[map_it].index = (UWORD32)-1;
    }
  }

  for (set_it = 0; set_it < MAX_NUM_HOA_DIR_SIGNALS; set_it++)
    prev_ps_sig_idx[set_it] = ptr_set_of_ps_sig_indices[set_it];
  return;
}
