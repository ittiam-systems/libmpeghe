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
 *  impeghe_hoa_ch_assignment_process
 *
 *  \brief channel assignment processing
 *
 *  \param [in,out] pstr_se_handle spatial encoder handle
 *
 *  \return VOID
 */
VOID impeghe_hoa_ch_assignment_process(ia_spatial_enc_str *pstr_se_handle)
{
  ia_spatial_enc_ch_assignment_str *pstr_ch_asgmt = &(pstr_se_handle->ch_asgnmnt_hdl);
  const ia_spatial_enc_assignment_info_str *pstr_ip_curr_asgmt_vec = pstr_se_handle->assgn_vec;
  const ia_spatial_enc_assignment_info_str *pstr_ip_pred_asgmt_vec =
      pstr_se_handle->target_assgn_vec;
  pFlOAT64 ptr_op_pred_all_sigs_code_nxt_frm_ptr = pstr_se_handle->penult_smtd_pre_dom_frm;
  pFlOAT64 ptr_op_all_sigs_code_frm_ptr =
      pstr_se_handle->trans_ch[pstr_se_handle->scd_lst_ip_idx];

  UWORD32 num_var_ch = pstr_ch_asgmt->num_var_ch;
  UWORD32 frm_sz = pstr_ch_asgmt->frame_size;
  UWORD32 tot_coders = pstr_ch_asgmt->tot_perc_coders;
  ULOOPIDX ch;
  UWORD32 pred_ch_type;

  for (ch = 0; ch < num_var_ch; ch++)
  {
    UWORD32 curr_ch_type;

    if (!pstr_ip_curr_asgmt_vec[ch].is_available)
    {
      curr_ch_type = HOA_EMPTY_CHANNEL;
    }
    else
    {
      curr_ch_type = pstr_ip_curr_asgmt_vec[ch].ch_type;
    }

    switch (curr_ch_type)
    {
    case HOA_DIR_CHANNEL:
    case HOA_VEC_CHANNEL:
    {
      break;
    }
    case HOA_ADD_HOA_CHANNEL:
    {
      const impeghe_hoa_info_ch_str *pstr_ch_info = &(pstr_ip_curr_asgmt_vec[ch].ch_info);
      UWORD32 amb_coeff_idx = pstr_ch_info->amb_coeff_idx;
      pFlOAT64 ptr_in_mod_amb_hoa_frm =
          pstr_se_handle->modfd_penult_amb_hoa_frm + (amb_coeff_idx - 1) * (frm_sz);

      memcpy(ptr_op_all_sigs_code_frm_ptr, ptr_in_mod_amb_hoa_frm, sizeof(FLOAT64) * (frm_sz));
      break;
    }
    case HOA_EMPTY_CHANNEL:
    {
      memset(ptr_op_all_sigs_code_frm_ptr, 0, sizeof(FLOAT64) * (frm_sz));
      break;
    }
    }
    ptr_op_all_sigs_code_frm_ptr += frm_sz;

    if (!pstr_ip_pred_asgmt_vec[ch].is_available)
    {
      pred_ch_type = HOA_EMPTY_CHANNEL;
    }
    else
    {
      pred_ch_type = pstr_ip_pred_asgmt_vec[ch].ch_type;
    }

    switch (pred_ch_type)
    {
    case HOA_DIR_CHANNEL:
    case HOA_VEC_CHANNEL:
    {
      pFlOAT64 ptr_in_smt_ps_sigs_nxt_frm =
          pstr_se_handle->ps_sigs_smtd_lst_frame + (ch) * (frm_sz);
      memcpy(ptr_op_pred_all_sigs_code_nxt_frm_ptr, ptr_in_smt_ps_sigs_nxt_frm,
             sizeof(FLOAT64) * (frm_sz));
      break;
    }
    case HOA_EMPTY_CHANNEL:
    {
      if (curr_ch_type != HOA_ADD_HOA_CHANNEL)
      {
        memset(ptr_op_pred_all_sigs_code_nxt_frm_ptr, 0, sizeof(FLOAT64) * (frm_sz));
      }
      else
      {
        const impeghe_hoa_info_ch_str *pstr_ch_info = &(pstr_ip_curr_asgmt_vec[ch].ch_info);
        UWORD32 amb_coeff_idx = pstr_ch_info->amb_coeff_idx;
        pFlOAT64 ptr_in_mod_pred_nxt_amb_hoa_frm =
            pstr_se_handle->modfd_pred_lst_amb_hoa_frm + (amb_coeff_idx - 1) * (frm_sz);

        memcpy(ptr_op_pred_all_sigs_code_nxt_frm_ptr, ptr_in_mod_pred_nxt_amb_hoa_frm,
               sizeof(FLOAT64) * (frm_sz));
      }
      break;
    }
    }
    ptr_op_pred_all_sigs_code_nxt_frm_ptr += frm_sz;
  }

  for (ch = num_var_ch; ch < tot_coders; ch++)
  {
    UWORD32 amb_coeff_idx = ch - num_var_ch + 1;
    pFlOAT64 ptr_ip_mod_amb_hoa_frm_ptr =
        pstr_se_handle->modfd_penult_amb_hoa_frm + (amb_coeff_idx - 1) * (frm_sz);
    pFlOAT64 ptr_ip_mod_pred_nxt_amb_hoa_frm_ptr =
        pstr_se_handle->modfd_pred_lst_amb_hoa_frm + (amb_coeff_idx - 1) * (frm_sz);

    memcpy(ptr_op_all_sigs_code_frm_ptr, ptr_ip_mod_amb_hoa_frm_ptr, sizeof(FLOAT64) * (frm_sz));
    memcpy(ptr_op_pred_all_sigs_code_nxt_frm_ptr, ptr_ip_mod_pred_nxt_amb_hoa_frm_ptr,
           sizeof(FLOAT64) * (frm_sz));
    ptr_op_all_sigs_code_frm_ptr += pstr_se_handle->frame_size;
    ptr_op_pred_all_sigs_code_nxt_frm_ptr += pstr_se_handle->frame_size;
  }
  return;
}
