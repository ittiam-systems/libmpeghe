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

#include "impeghe_error_standards.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_pre_dom_sound_syn.h"

/**
 *  impeghe_hoa_pre_dom_sound_syn_process
 *
 *  \brief Pre-dominant sound synthesis processing
 *
 *  \param [in] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_set_amb_coeff_indices_enabled Ambient coefficients to be enabled
 *  \param [in] ptr_set_amb_coeff_indices_disabled Ambient coefficients to be disabled
 *  \param [in] ptr_set_act_hoa_coeff_indices Set of active hoa coefficients that are
 * neither enabled/disabled
 *
 *  \return VOID
 */
VOID impeghe_hoa_pre_dom_sound_syn_process(ia_spatial_enc_str *pstr_se_handle,
                                           const pUWORD32 ptr_set_amb_coeff_indices_enabled,
                                           const pUWORD32 ptr_set_amb_coeff_indices_disabled,
                                           const pUWORD32 ptr_set_act_hoa_coeff_indices)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  const ia_hoa_dir_id_str *pstr_active_dir_n_idx = pstr_frm_prm->active_n_grid_dir_idx;
  const ia_hoa_vec_sig_str *pstr_vectors = pstr_frm_prm->vectors;

  UWORD32 ps_sig_idx[MAX_NUM_HOA_DIR_SIGNALS];
  ULOOPIDX i;

  for (i = 0; pstr_active_dir_n_idx[i].ch_idx != -1; i++)
  {
    if (MAX_NUM_HOA_DIR_SIGNALS == i)
      break;

    ps_sig_idx[i] = pstr_active_dir_n_idx[i].ch_idx;
  }
  for (; i < MAX_NUM_HOA_DIR_SIGNALS; i++)
  {
    ps_sig_idx[i] = (UWORD32)-1;
  }

  for (i = 0; (WORD32)pstr_vectors[i].index != -1; i++)
  {
    if (MAX_NUM_HOA_DIR_SIGNALS == i)
      break;

    ps_sig_idx[i] = pstr_vectors[i].index;
  }
  for (; i < MAX_NUM_HOA_DIR_SIGNALS; i++)
  {
    ps_sig_idx[i] = (UWORD32)-1;
  }

  impeghe_hoa_dir_based_pre_dom_sound_syn_process(
      pstr_se_handle, ptr_set_amb_coeff_indices_enabled, ptr_set_amb_coeff_indices_disabled,
      ptr_set_act_hoa_coeff_indices, ps_sig_idx);

  impeghe_hoa_vector_based_predom_sound_syn_process(pstr_se_handle, ps_sig_idx,
                                                    ptr_set_amb_coeff_indices_enabled,
                                                    ptr_set_amb_coeff_indices_disabled);

  return;
}
