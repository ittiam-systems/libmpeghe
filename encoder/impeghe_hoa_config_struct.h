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

#ifndef IMPEGH_HOA_CONFIG_STRUCT_H
#define IMPEGH_HOA_CONFIG_STRUCT_H

#define MAXIMUM_MAP_ARRAY_SIZE (1024)

typedef struct
{
  UWORD32 num_transport_ch;
  WORD32 core_coder_frame_length;
  WORD32 order;
  UWORD32 num_coeffs;
  WORD32 is_screen_relative;
  WORD32 uses_nfc;
  FLOAT32 nfc_ref_distance;
  WORD32 min_amb_order;
  WORD32 min_coeffs_for_amb;
  UWORD32 total_num_coders;
  UWORD32 num_addnl_coders;
  WORD32 is_single_layer;
  WORD32 coded_spat_interpolation_time;
  WORD32 spat_interpolation_method;
  WORD32 coded_v_vec_length;
  WORD32 max_gain_corr_amp_exp;
  WORD32 frame_length_indicator;
  WORD32 max_order_to_be_transmitted;
  WORD32 diff_order_bits;
  WORD32 diff_order;
  WORD32 max_num_of_coeffs_to_be_transmitted;
  WORD32 max_num_add_active_amb_coeffs;
  WORD32 vq_conf_bits;
  WORD32 num_v_vec_vq_elements_bits;
  WORD32 use_phase_shift_decorr;
  UWORD32 max_no_of_dir_sigs_for_prediction;
  WORD32 no_of_bits_per_scale_factor;
  WORD32 pred_subbands_idx;
  WORD32 num_of_pred_subbands;
  WORD32 pred_subband_widths[20];
  WORD32 subband_config_idx;
  WORD32 first_sbr_subband_idx;
  WORD32 max_num_of_pred_dirs;
  WORD32 max_num_of_pred_dirs_per_band;
  WORD32 dir_grid_table_idx;
  WORD32 par_subband_table_idx;
  WORD32 num_of_par_subbands;
  WORD32 par_subband_widths_sz;
  WORD32 par_subband_widths[20];
  WORD32 last_first_order_subband_idx;
  WORD32 use_real_coeffs_per_par_subband_sz;
  WORD32 use_real_coeffs_per_par_subband[128];
  UWORD32 frame_length;
  WORD32 spat_interpolation_time;
  WORD32 amb_asign_m_bits;
  WORD32 active_pred_ids_bits;
  WORD32 num_active_pred_ids_bits;
  WORD32 gain_corr_prev_amp_exp_bits;
  WORD32 upmix_order_per_par_subband_sz;
  WORD32 upmix_order_per_par_subband[128];
  WORD32 num_ch_is_lay[128];
  UWORD32 idx_offset;
  WORD32 v_vec_length_used;
  WORD32 index_length;
  UWORD32 max_v_vec_dir;

} ia_hoa_config_struct;

typedef struct
{
  WORD32 ch_idx;
  WORD32 dir_id;
} ia_hoa_dir_id_str;

typedef struct
{
  UWORD32 index;
  FLOAT64 sig_id[MAXIMUM_MAP_ARRAY_SIZE];
} ia_hoa_vec_sig_str;

#endif /*IMPEGH_HOA_CONFIG_STRUCT_H*/
