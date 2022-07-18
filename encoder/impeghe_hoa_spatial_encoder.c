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
#include "impeghe_error_standards.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_dir_vec_estimation.h"

/**
 *  impeghe_encode_frame_side_info
 *
 *  \brief Generate HOA bit-stream
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *
 *  \return VOID
 */
static VOID impeghe_encode_frame_side_info(ia_spatial_enc_str *pstr_se_handle)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_hoa_frame_struct *pstr_frame = &(pstr_se_handle->penult_hoa_frm);

  ULOOPIDX i, j, k;
  UWORD32 num_act_pred;
  UWORD32 num_coeffs = pstr_se_handle->num_hoa_coeffs;
  UWORD32 tot_perc_cod = pstr_se_handle->tot_perc_coders;
  UWORD32 add_perc_cod = pstr_se_handle->num_add_perc_coders;
  UWORD32 max_pred_dir_sigs = pstr_se_handle->max_pred_dir_sigs;
  ia_spatial_enc_assignment_info_str *pstr_assgn_vec = pstr_se_handle->assgn_vec;
  ia_spatial_enc_dyn_correction_str *pstr_dyn_cor_hdl = pstr_se_handle->dyn_corrctn_hdl;
  UWORD32 curr_pred_dir_idx = 0;
  UWORD32 curr_pred_scale_idx = 0;
  FLAG flag;

  for (j = 0; j < tot_perc_cod; j++)
  {
    WORD32 curr_exponent = pstr_frm_prm->exponents[j];
    WORD32 core_code_len;

    switch (curr_exponent)
    {
    case 0:
    {
      core_code_len = 1;
      break;
    }
    case -1:
    {
      core_code_len = 2;
      break;
    }
    default:
    {
      core_code_len = curr_exponent + 2;
    }
    }

    for (i = 0; i < (ULOOPIDX)core_code_len - 1; i++)
    {
      pstr_frame->coded_gain_correction_exp[j][i] = 0;
    }

    pstr_frame->coded_gain_correction_exp[j][i] = 1;
    i++;
    pstr_frame->coded_gain_correction_exp_sz[j] = i;

    for (; i < 128; i++)
    {
      pstr_frame->coded_gain_correction_exp[j][i] = -1;
    }

    pstr_frame->gain_correction_exception[j] = pstr_frm_prm->is_exception[j];

    pstr_frame->gain_corr_prev_amp_exp[j] = pstr_dyn_cor_hdl[j].prev_exp_base2;
  }

  for (; j < MAX_NUMBER_CHANNELS; j++)
  {
    pstr_frame->coded_gain_correction_exp[j][0] = -1;
    pstr_frame->gain_corr_prev_amp_exp[j] = -1;
    pstr_frame->gain_correction_exception[j] = -1;
  }
  pstr_frame->ch_side_info_sz = add_perc_cod;

  if (pstr_se_handle->send_ind_flag)
  {
    pstr_frame->hoa_independency_flag = 1;
    pstr_se_handle->send_ind_flag = 0;
  }
  else
  {
    pstr_frame->hoa_independency_flag = 0;
  }

  for (j = 0; j < add_perc_cod; j++)
  {
    pstr_frame->channel_type[j] = pstr_assgn_vec[j].ch_type;

    switch (pstr_frame->channel_type[j])
    {
    case HOA_DIR_CHANNEL:
      pstr_frame->active_dirs_ids[j] = pstr_assgn_vec[j].ch_info.active_dir_ids;
      break;
    case HOA_VEC_CHANNEL:
      pstr_frame->new_channel_type_one[j] = pstr_assgn_vec[j].vec_based_ch_info.new_ch_type_one;
      pstr_frame->n_bits_q[j] = pstr_assgn_vec[j].vec_based_ch_info.us_bits_q;
      pstr_frame->codebk_idx[j] = pstr_assgn_vec[j].vec_based_ch_info.indices_code_book_idx;
      pstr_frame->num_vvec_indices[j] = pstr_assgn_vec[j].vec_based_ch_info.v_vec_dir;
      pstr_frame->cb_flag[j] = pstr_assgn_vec[j].vec_based_ch_info.cb_flag;
      pstr_frame->p_flag[j] = pstr_assgn_vec[j].vec_based_ch_info.p_flag;
      pstr_frame->same_header_prev_frame[j] =
          pstr_assgn_vec[j].vec_based_ch_info.same_header_prev_frame;
      pstr_frame->weight_idx[j] = pstr_assgn_vec[j].vec_based_ch_info.weighting_code_book_idx;
      memcpy(pstr_frame->c_8bit_quantizer_word[j],
             pstr_assgn_vec[j].vec_based_ch_info.ui_8bit_coded_v_element,
             MAX_NUM_HOA_COEFFS * sizeof(UWORD32));
      break;
    case HOA_ADD_HOA_CHANNEL:
      pstr_frame->amb_coeff_transition_state[j] =
          pstr_assgn_vec[j].ch_info.amb_coeff_idx_transition_state;
      pstr_frame->amb_coeff_idx[j] = pstr_assgn_vec[j].ch_info.amb_coeff_idx;
      pstr_frame->amb_coeff_idx_transition[j] = pstr_assgn_vec[j].ch_info.amb_coeff_idx_changed;
      break;
    }
  }

  num_act_pred = 0;

  for (k = 0; k < num_coeffs; k++)
  {
    if (pstr_frm_prm->pred_typ_vec[k] > 0)
    {
      num_act_pred++;
    }
  }

  if (num_act_pred == 0)
  {
    pstr_frame->ps_prediction_active = 0;
  }
  else
  {
    pstr_frame->ps_prediction_active = 1;

    if (num_act_pred <= pstr_se_handle->explct_pred_idx_to_code)
    {
      pstr_frame->kind_of_coded_pred_ids = 1;
    }
    else
    {
      pstr_frame->kind_of_coded_pred_ids = 0;
    }

    for (i = 0; i < MAX_NUM_HOA_COEFFS; i++)
    {
      pstr_frame->active_pred[i] = -1;
    }
    for (i = 0; i < 128; i++)
    {
      pstr_frame->pred_ids[i] = -1;
      pstr_frame->pred_gains[i] = -1;
    }

    for (k = 0; k < num_coeffs; k++)
    {
      UWORD32 curr_prediction_type = pstr_frm_prm->pred_typ_vec[k];

      pstr_frame->active_pred[k] = (curr_prediction_type > 0);

      if (curr_prediction_type > 0)
      {
        pUWORD32 prediction_indices_mat_ptr;
        prediction_indices_mat_ptr = pstr_frm_prm->pred_idx_mat + k;

        for (i = 0; i < max_pred_dir_sigs; i++)
        {
          pstr_frame->pred_ids[curr_pred_dir_idx] = *prediction_indices_mat_ptr;
          curr_pred_dir_idx++;

          if (0 != *prediction_indices_mat_ptr)
          {
            pstr_frame->pred_gains[curr_pred_scale_idx] =
                *(pstr_frm_prm->quant_pred_fac_mat + i * num_coeffs + k);
            curr_pred_scale_idx++;
          }
          prediction_indices_mat_ptr += num_coeffs;
        }
      }
    }
  }

  flag = 0;
  if (-1 != pstr_frm_prm->active_n_grid_dir_idx_for_subband_pred[0])
  {
    flag = 1;
  }

  pstr_frame->use_dir_pred = flag;
  return;
}

/**
 *  impeghe_hoa_spatial_enc_process
 *
 *  \brief Spatial Encoder Processing
 *
 *  \param [in,out] pstr_hoa_enc Spatial encoder handle
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_hoa_spatial_enc_process(ia_hoa_enc_str *pstr_hoa_enc)
{
  ia_hoa_frame_struct *pstr_hoa_frame = pstr_hoa_enc->hoa_frm;
  ia_spatial_enc_str *pstr_se_handle = &(pstr_hoa_enc->spat_enc_hdl);
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  IA_ERRORCODE err_code = IA_NO_ERROR;

  ULOOPIDX i, ch_idx;
  pFlOAT64 act_ip_frm_ptr = NULL;

  pFlOAT64 available_memory;
  UWORD32 used_memory = 0;
  pstr_se_handle->ptr_scratch = pstr_hoa_enc->ptr_scratch;
  pstr_se_handle->scratch_used_size = pstr_hoa_enc->scratch_used_size;
  available_memory =
      (pFlOAT64)((pUWORD8)pstr_se_handle->ptr_scratch + pstr_se_handle->scratch_used_size);
  pstr_se_handle->modfd_pred_lst_amb_hoa_frm = available_memory;
  used_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;
  available_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;

  pstr_se_handle->modfd_penult_amb_hoa_frm = available_memory;
  used_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;
  available_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;

  pstr_se_handle->penult_smtd_pre_dom_frm = available_memory;
  used_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;
  available_memory += MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN;

  pstr_se_handle->pred_lst_amb_hoa_frm = available_memory;
  used_memory += MAX_NUM_HOA_COEFFS_FOR_AMB * MAX_FRAME_LEN;
  available_memory += MAX_NUM_HOA_COEFFS_FOR_AMB * MAX_FRAME_LEN;

  pstr_se_handle->penult_amb_hoa_frm = available_memory;
  used_memory += MAX_NUM_HOA_COEFFS_FOR_AMB * MAX_FRAME_LEN;
  available_memory += MAX_NUM_HOA_COEFFS_FOR_AMB * MAX_FRAME_LEN;

  pstr_se_handle->scratch_used_size += (used_memory * sizeof(FLOAT64));

  if (!pstr_se_handle->ip_ended)
  {
    impeghe_hoa_decomposition_str *decomp_handle = &(pstr_hoa_enc->spat_enc_hdl.decmp_hdl);
    act_ip_frm_ptr = decomp_handle->ip_hoa_frame[decomp_handle->ip_idx];
  }
  else
  {
    pstr_se_handle->delay_frm_cnt++;
  }

  if (pstr_se_handle->max_dir_sigs > 0)
  {
    err_code = impeghe_hoa_dir_vec_estimation_process(pstr_se_handle, act_ip_frm_ptr);
    if (IA_NO_ERROR != err_code)
    {
      return err_code;
    }
  }
  else
  {
    for (i = 0; i < MAX_NUMBER_CHANNELS; i++)
    {
      pstr_frm_prm->active_n_grid_dir_idx[i].ch_idx = (UWORD32)-1;
      pstr_frm_prm->vectors[i].index = (UWORD32)-1;
    }
  }

  ch_idx = pstr_se_handle->curr_op_idx;
  pstr_se_handle->curr_op_idx = pstr_se_handle->thrd_lst_ip_idx;
  pstr_se_handle->thrd_lst_ip_idx = pstr_se_handle->scd_lst_ip_idx;
  pstr_se_handle->scd_lst_ip_idx = ch_idx;

  impeghe_hoa_decomposition_process(pstr_se_handle, act_ip_frm_ptr);

  impeghe_hoa_amb_comp_mod_process(pstr_se_handle);

  impeghe_hoa_ch_assignment_process(pstr_se_handle);

  for (ch_idx = 0; ch_idx < pstr_se_handle->tot_perc_coders; ch_idx++)
  {
    impeghe_hoa_dyn_correction_process(pstr_se_handle, ch_idx);
  }

  memcpy(pstr_hoa_frame, &(pstr_se_handle->thrd_lst_hoa_frm), sizeof(ia_hoa_frame_struct));

  memcpy(&(pstr_se_handle->thrd_lst_hoa_frm), &(pstr_se_handle->penult_hoa_frm),
         sizeof(ia_hoa_frame_struct));

  impeghe_encode_frame_side_info(pstr_se_handle);

  for (i = 0; i < MAX_SET_SIZE; i++)
  {
    pstr_se_handle->enble_amb_coeff_idx_thrd_lst_frm[i] = pstr_frm_prm->amb_coeff_idx_to_enb[i];
    pstr_se_handle->disble_amb_coeff_idx_thrd_lst_frm[i] =
        pstr_frm_prm->amb_coeff_idx_to_disable[i];
    pstr_se_handle->no_en_dis_able_amb_coeff_idx_thrd_lst_frm[i] =
        pstr_frm_prm->non_en_dis_able_act_idx[i];
  }

  if (pstr_se_handle->delay_frm_cnt >= pstr_se_handle->req_delay_frm)
  {
    pstr_se_handle->op_ended = 1;
  }

  return err_code;
}
