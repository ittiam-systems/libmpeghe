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

#ifndef IMPEGHE_HOA_FRAME_STRUCT_H
#define IMPEGHE_HOA_FRAME_STRUCT_H

#include "impeghe_hoa_common_values.h"
#include "impeghe_hoa_assignment_info.h"

typedef struct
{

  ia_hoa_config_struct *ptr_config_data;

  WORD32 num_of_dir_sigs;
  UWORD32 num_of_vec_sigs;

  WORD32 hoa_independency_flag;

  WORD32 dir_sig_channel_ids[MAX_NUMBER_CHANNELS];
  WORD32 vec_sig_channel_ids[MAX_NUMBER_CHANNELS];

  WORD32 channel_type[MAX_NUMBER_CHANNELS];
  UWORD32 active_dirs_ids[MAX_NUMBER_CHANNELS];
  WORD32 new_channel_type_one[MAX_NUMBER_CHANNELS];
  UWORD32 n_bits_q[MAX_NUMBER_CHANNELS];
  WORD32 codebk_idx[MAX_NUMBER_CHANNELS];
  UWORD32 num_vvec_indices[MAX_NUMBER_CHANNELS];
  WORD32 p_flag[MAX_NUMBER_CHANNELS];
  WORD32 cb_flag[MAX_NUMBER_CHANNELS];

  UWORD32 amb_coeff_transition_state[MAX_NUMBER_CHANNELS];
  WORD32 amb_coeff_idx[MAX_NUMBER_CHANNELS];
  WORD32 amb_coeff_idx_transition[MAX_NUMBER_CHANNELS];

  WORD32 gain_corr_prev_amp_exp[MAX_NUMBER_CHANNELS];
  WORD32 coded_gain_correction_exp[MAX_NUMBER_CHANNELS][128];
  WORD32 coded_gain_correction_exp_sz[MAX_NUMBER_CHANNELS];
  WORD32 gain_correction_exception[MAX_NUMBER_CHANNELS];
  WORD32 perform_par;

  WORD32 ps_prediction_active;
  WORD32 kind_of_coded_pred_ids;

  WORD32 pred_ids[128];
  WORD32 active_pred[MAX_NUM_HOA_COEFFS];
  WORD32 pred_dir_sig_ids[128];
  WORD32 pred_gains[128];
  WORD32 num_active_pred;
  WORD32 num_of_gains;
  WORD32 use_dir_pred;

  UWORD32 vvec_idx[128];
  WORD32 n_bit_idx;
  UWORD8 sgn_val[MAX_NUMBER_CHANNELS][128];
  WORD32 sgn_val_size[128];
  WORD32 prev_sgn_val_size[128];
  WORD32 weight_val[128];
  WORD32 weight_idx[128];
  WORD32 c_8bit_quantizer_word[MAX_NUMBER_CHANNELS][128];

  WORD32 v_vec_length_used;

  WORD32 same_header_prev_frame[MAX_NUMBER_CHANNELS];
  WORD32 prev_channel_type[MAX_NUMBER_CHANNELS];

  UWORD32 additional_value[MAX_NUMBER_CHANNELS][128];

  UWORD32 non_transitional_add_hoa_channels[128];
  UWORD32 fade_in_add_hoa_channels[128];
  UWORD32 new_vec_channels[MAX_NUMBER_CHANNELS];

  UWORD32 ch_side_info_sz;

} ia_hoa_frame_struct;

#endif /*IMPEGHE_HOA_FRAME_STRUCT_H*/
