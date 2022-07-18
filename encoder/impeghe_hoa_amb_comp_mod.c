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
#include "impeghe_hoa_struct.h"

/**
 *  impeghe_hoa_sort_array_ascending
 *
 *  \brief Arranges a given array in ascending order of value
 *
 *  \param [in] ptr_src unsorted array
 *  \param [out] ptr_dst output
 *
 *  \return VOID
 */
static VOID impeghe_hoa_sort_array_ascending(pUWORD32 ptr_src, pUWORD32 ptr_dst)
{
  UWORD32 set_size;
  ULOOPIDX i, j;
  for (i = 0; (WORD32)ptr_src[i] != -1; i++)
  {
    if (MAX_NUM_AMB_COEFF_INDICES == i)
      break;
  }
  set_size = i;
  if (set_size)
  {
    UWORD32 next = 0xFFFF; // Large number
    UWORD32 next_id = (UWORD32)-1;
    for (j = 0; j < set_size; j++)
    {
      next = 0xFFFF;
      for (i = 0; i < set_size; i++)
      {
        if ((next > ptr_src[i]) && (ptr_src[i] != -1))
        {
          next = ptr_src[i];
          next_id = i;
        }
        else if (next == ptr_src[i])
        {
          ptr_src[i] = (UWORD32)-1;
        }
      }
      if (next == 0xFFFF)
        break;

      ptr_dst[j] = next;
      ptr_src[next_id] = (UWORD32)-1;
    }
    for (; j < MAX_NUM_AMB_COEFF_INDICES; j++)
    {
      ptr_dst[j] = (UWORD32)-1;
    }
  }
  else
  {
    for (j = 0; j < MAX_NUM_AMB_COEFF_INDICES; j++)
    {
      ptr_dst[j] = (UWORD32)-1;
    }
  }
  return;
}

/**
 *  impeghe_hoa_set_diff_uint
 *
 *  \brief isolates elements that are unique in given set
 *
 *  \param [out] ptr_out array of unique elements
 *  \param [in] ptr_first First set
 *  \param [in] ptr_second Second set
 *
 *  \return VOID
 */
static VOID impeghe_hoa_set_diff_uint(pUWORD32 ptr_out, pUWORD32 ptr_first, pUWORD32 ptr_second)
{
  UWORD32 i, j;
  UWORD32 idx = 0;
  for (i = 0; i < MAX_NUM_AMB_COEFF_INDICES; i++)
  {
    if (-1 != ptr_first[i])
    {
      for (j = 0; j < MAX_NUM_AMB_COEFF_INDICES; j++)
      {
        if (ptr_first[i] == ptr_second[j])
          break;
      }
      if (MAX_NUM_AMB_COEFF_INDICES == j)
      {
        ptr_out[idx] = ptr_first[i];
        idx++;
      }
    }
    else
      break;
  }
  for (; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    ptr_out[idx] = (UWORD32)-1;
  }
  return;
}

/**
 *  impeghe_hoa_amb_comp_mod_process
 *
 *  \brief Ambient modification processing
 *
 *  \param [in,out] pstr_se_handle spatial encoder handle
 *
 *  \return VOID
 */
VOID impeghe_hoa_amb_comp_mod_process(ia_spatial_enc_str *pstr_se_handle)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_enc_amb_comp_mod_str *pstr_amb_handle = &(pstr_se_handle->amb_comp_hdl);
  const pFlOAT64 ptr_ip_amb_hoa_frame = pstr_se_handle->penult_amb_hoa_frm;
  const pFlOAT64 ptr_ip_pred_nxt_amb_hoa_frame = pstr_se_handle->pred_lst_amb_hoa_frm;
  const ia_spatial_enc_assignment_info_str *pstr_target_asgmt_vec =
      pstr_se_handle->target_assgn_vec;
  pFlOAT64 ptr_op_mod_amb_hoa_frame = pstr_se_handle->modfd_penult_amb_hoa_frm;
  pFlOAT64 ptr_op_mod_pred_nxt_amb_hoa_frame = pstr_se_handle->modfd_pred_lst_amb_hoa_frm;
  ia_spatial_enc_assignment_info_str *pstr_final_asgmt_vec = pstr_se_handle->assgn_vec;
  pUWORD32 ptr_act_amb_hoa_coeffs_idx = pstr_se_handle->act_hoa_coeff_idx;
  pUWORD32 ptr_amb_coeffs_idx_to_enable = pstr_frm_prm->amb_coeff_idx_to_enb;
  pUWORD32 ptr_amb_coeffs_idx_to_disable = pstr_frm_prm->amb_coeff_idx_to_disable;

  pFlOAT64 ptr_ip_pred_nxt_amb_hoa_frm_ptr;
  ULOOPIDX id, idx, ch, s, ele_id;
  UWORD32 next_target_ch_type, curr_target_ch_type;
  UWORD32 act_amb_hoa_coeff_idx[MAX_NUM_AMB_COEFF_INDICES];
  UWORD32 amb_hoa_coeff_possibly_to_activate[MAX_NUM_AMB_COEFF_INDICES];
  UWORD32 combined_indices[MAX_NUM_AMB_COEFF_INDICES];
  UWORD32 temp_data[MAX_NUM_AMB_COEFF_INDICES];
  UWORD32 frm_sz = pstr_amb_handle->frame_size;
  UWORD32 num_var_ch = pstr_amb_handle->num_var_ch;
  UWORD32 tot_perc_coders = pstr_amb_handle->tot_perc_coders;
  UWORD32 num_amb_hoa_coeffs = pstr_amb_handle->num_amb_hoa_coeffs;
  UWORD32 br_per_coder = pstr_amb_handle->br_per_coder;
  UWORD32 hoa_coeffs = pstr_amb_handle->num_hoa_coeffs;
  pUWORD32 ptr_act_coeffs_idx = pstr_amb_handle->act_amb_hoa_coeffs_idx;
  pUWORD32 ptr_enb_amb_coeffs_idx = pstr_amb_handle->amb_hoa_coeffs_idx_to_enable;
  pUWORD32 ptr_disb_amb_coeffs_idx = pstr_amb_handle->amb_hoa_coeffs_idx_to_disable;
  ia_spatial_enc_assignment_info_str *pstr_assgn_vec = pstr_amb_handle->assgn_vec;
  ia_spatial_enc_assignment_info_str *pstr_prev_assgn_vec = pstr_amb_handle->prev_tgt_assgn_vec;

  UWORD32 curr_hoa_coeffs_idx = 0;
  UWORD32 amb_hoa_coeffs_to_enable_id = 0;
  UWORD32 amb_hoa_coeffs_to_disable_id = 0;
  UWORD32 active_amb_hoa_coeffs_id = 0;

  for (id = 0; id < MAX_NUM_AMB_COEFF_INDICES; id++)
  {
    act_amb_hoa_coeff_idx[id] = (UWORD32)-1;
    amb_hoa_coeff_possibly_to_activate[id] = (UWORD32)-1;
  }

  impeghe_hoa_set_diff_uint(act_amb_hoa_coeff_idx, ptr_act_coeffs_idx, ptr_disb_amb_coeffs_idx);
  impeghe_hoa_set_diff_uint(amb_hoa_coeff_possibly_to_activate,
                            pstr_amb_handle->pot_amb_hoa_coeff_idx, act_amb_hoa_coeff_idx);
  for (idx = 0; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    ptr_act_coeffs_idx[idx] = (UWORD32)-1;
    ptr_enb_amb_coeffs_idx[idx] = (UWORD32)-1;
    ptr_disb_amb_coeffs_idx[idx] = (UWORD32)-1;
  }

  for (ch = 0; ch < num_var_ch; ch++)
  {
    UWORD32 old_ch_type;
    if (!(pstr_assgn_vec[ch].is_available))
    {
      impeghe_hoa_assignment_info_init(&(pstr_assgn_vec[ch]));
    }
    old_ch_type = pstr_assgn_vec[ch].ch_type;

    if (!(pstr_prev_assgn_vec[ch].is_available))
    {
      impeghe_hoa_assignment_info_init(&(pstr_prev_assgn_vec[ch]));
    }
    curr_target_ch_type = pstr_prev_assgn_vec[ch].ch_type;

    if (!(pstr_target_asgmt_vec[ch].is_available))
    {
      next_target_ch_type = HOA_EMPTY_CHANNEL;
    }
    else
    {
      next_target_ch_type = pstr_target_asgmt_vec[ch].ch_type;
    }

    switch (old_ch_type)
    {
    case HOA_DIR_CHANNEL:
    case HOA_VEC_CHANNEL:
    case HOA_EMPTY_CHANNEL:
    {
      if ((curr_target_ch_type == HOA_EMPTY_CHANNEL) &&
          (next_target_ch_type == HOA_EMPTY_CHANNEL))
      {
        if (amb_hoa_coeff_possibly_to_activate[curr_hoa_coeffs_idx] != -1)
        {
          UWORD32 amb_coeff_idx;
          impeghe_hoa_info_ch_str *pstr_ch_info;

          pstr_assgn_vec[ch].is_available = 1;
          pstr_assgn_vec[ch].bit_rate = br_per_coder;
          pstr_assgn_vec[ch].ch_type = HOA_ADD_HOA_CHANNEL;

          amb_coeff_idx = amb_hoa_coeff_possibly_to_activate[curr_hoa_coeffs_idx];
          pstr_ch_info = &(pstr_assgn_vec[ch].ch_info);

          pstr_ch_info->amb_coeff_idx = amb_coeff_idx;
          pstr_ch_info->amb_coeff_idx_changed = 1;
          pstr_ch_info->amb_coeff_idx_transition_state = 1;

          ptr_enb_amb_coeffs_idx[amb_hoa_coeffs_to_enable_id] = amb_coeff_idx;
          ptr_act_coeffs_idx[active_amb_hoa_coeffs_id] = amb_coeff_idx;

          amb_hoa_coeffs_to_enable_id++;
          active_amb_hoa_coeffs_id++;
          curr_hoa_coeffs_idx++;
        }
      }
      else
      {
        pstr_assgn_vec[ch] = pstr_prev_assgn_vec[ch];
      }
      break;
    }
    case HOA_ADD_HOA_CHANNEL:
    {
      if (curr_target_ch_type == HOA_EMPTY_CHANNEL)
      {
        UWORD32 amb_coeff_idx;
        impeghe_hoa_info_ch_str *pstr_ch_info;

        pstr_assgn_vec[ch].bit_rate = br_per_coder;
        pstr_assgn_vec[ch].ch_type = HOA_ADD_HOA_CHANNEL;

        pstr_ch_info = &(pstr_assgn_vec[ch].ch_info);
        amb_coeff_idx = pstr_ch_info->amb_coeff_idx;

        ptr_act_coeffs_idx[active_amb_hoa_coeffs_id] = amb_coeff_idx;
        active_amb_hoa_coeffs_id++;

        if (next_target_ch_type != HOA_EMPTY_CHANNEL)
        {
          ptr_disb_amb_coeffs_idx[amb_hoa_coeffs_to_disable_id] = amb_coeff_idx;
          amb_hoa_coeffs_to_disable_id++;

          pstr_ch_info->amb_coeff_idx_changed = 1;
          pstr_ch_info->amb_coeff_idx_transition_state = 2;
        }
        else
        {
          pstr_ch_info->amb_coeff_idx_changed = 0;
          pstr_ch_info->amb_coeff_idx_transition_state = 0;
        }
      }
      else
      {
        pstr_assgn_vec[ch] = pstr_prev_assgn_vec[ch];
      }
    }
    }
  }

  for (ch = num_var_ch; ch < tot_perc_coders; ch++)
  {
    ptr_act_coeffs_idx[active_amb_hoa_coeffs_id] = ch - num_var_ch + 1;
    active_amb_hoa_coeffs_id++;
  }

  for (idx = 0; (WORD32)ptr_act_coeffs_idx[idx] != -1; idx++)
  {
    if (MAX_NUM_AMB_COEFF_INDICES == idx)
      break;
    ptr_act_amb_hoa_coeffs_idx[idx] = ptr_act_coeffs_idx[idx];
  }
  for (; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    ptr_act_amb_hoa_coeffs_idx[idx] = (UWORD32)-1;
  }

  for (idx = 0; (WORD32)ptr_enb_amb_coeffs_idx[idx] != -1; idx++)
  {
    if (MAX_NUM_AMB_COEFF_INDICES == idx)
      break;
    ptr_amb_coeffs_idx_to_enable[idx] = ptr_enb_amb_coeffs_idx[idx];
  }
  for (; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    ptr_amb_coeffs_idx_to_enable[idx] = (UWORD32)-1;
  }

  for (idx = 0; (WORD32)ptr_disb_amb_coeffs_idx[idx] != -1; idx++)
  {
    if (MAX_NUM_AMB_COEFF_INDICES == idx)
      break;
    ptr_amb_coeffs_idx_to_disable[idx] = ptr_disb_amb_coeffs_idx[idx];
  }
  for (; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    ptr_amb_coeffs_idx_to_disable[idx] = (UWORD32)-1;
  }

  idx = 0;
  for (idx = 0; (-1 != (WORD32)ptr_enb_amb_coeffs_idx[idx]); idx++)
  {
    if (MAX_NUM_AMB_COEFF_INDICES == idx)
      break;
    combined_indices[idx] = ptr_enb_amb_coeffs_idx[idx];
  }

  for (UWORD32 index = 0; (-1 != (WORD32)ptr_disb_amb_coeffs_idx[index]); index++)
  {
    if ((MAX_NUM_AMB_COEFF_INDICES == idx) || (MAX_NUM_AMB_COEFF_INDICES == index))
      break;
    combined_indices[idx++] = ptr_disb_amb_coeffs_idx[index];
  }
  for (; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    combined_indices[idx] = (UWORD32)-1;
  }

  impeghe_hoa_sort_array_ascending(combined_indices, temp_data);
  impeghe_hoa_set_diff_uint(pstr_frm_prm->non_en_dis_able_act_idx, ptr_act_coeffs_idx, temp_data);

  for (ch = 0; ch < num_var_ch; ch++)
  {
    pstr_final_asgmt_vec[ch] = pstr_assgn_vec[ch];
  }

  for (id = 0; id < num_amb_hoa_coeffs; id++)
  {
    pFlOAT64 ptr_inv_low_order_mode_mat_ptr =
        pstr_amb_handle->inv_low_order_mode_mat + (id * num_amb_hoa_coeffs);
    for (s = 0; s < frm_sz; s++)
    {
      pFlOAT64 ptr_ip_amb_hoa_frame_ptr = ptr_ip_amb_hoa_frame + s;
      *ptr_op_mod_amb_hoa_frame = 0.0;
      for (ele_id = 0; ele_id < num_amb_hoa_coeffs; ele_id++)
      {
        *ptr_op_mod_amb_hoa_frame +=
            ptr_inv_low_order_mode_mat_ptr[ele_id] * (*ptr_ip_amb_hoa_frame_ptr);
        ptr_ip_amb_hoa_frame_ptr += frm_sz;
      }
      ptr_op_mod_amb_hoa_frame++;
    }
  }

  for (id = num_amb_hoa_coeffs; id < hoa_coeffs; id++)
  {
    UWORD32 index;
    for (index = 0; index < MAX_NUM_AMB_COEFF_INDICES; index++)
    {
      if ((id + 1) == ptr_act_coeffs_idx[index])
        break;
    }
    if (index == MAX_NUM_AMB_COEFF_INDICES)
    {
      memset(ptr_op_mod_amb_hoa_frame, 0, sizeof(FLOAT64) * frm_sz);
    }
    ptr_op_mod_amb_hoa_frame += frm_sz;
  }

  for (id = 0; id < num_amb_hoa_coeffs; id++)
  {
    pFlOAT64 ptr_inv_low_order_mode_mat_ptr =
        pstr_amb_handle->inv_low_order_mode_mat + id * (num_amb_hoa_coeffs);
    for (s = 0; s < frm_sz; s++)
    {
      ptr_ip_pred_nxt_amb_hoa_frm_ptr = ptr_ip_pred_nxt_amb_hoa_frame + s;
      *ptr_op_mod_pred_nxt_amb_hoa_frame = 0.0;
      for (ele_id = 0; ele_id < num_amb_hoa_coeffs; ele_id++)
      {
        *ptr_op_mod_pred_nxt_amb_hoa_frame +=
            ptr_inv_low_order_mode_mat_ptr[ele_id] * (*ptr_ip_pred_nxt_amb_hoa_frm_ptr);
        ptr_ip_pred_nxt_amb_hoa_frm_ptr += frm_sz;
      }
      ptr_op_mod_pred_nxt_amb_hoa_frame++;
    }
  }

  for (idx = 0; idx < MAX_NUM_AMB_COEFF_INDICES; idx++)
  {
    pstr_prev_assgn_vec[idx] = pstr_target_asgmt_vec[idx];
  }
  return;
}
