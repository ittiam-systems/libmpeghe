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

#ifndef IMPEGHE_HOA_STRUCT_H
#define IMPEGHE_HOA_STRUCT_H

#include "impeghe_hoa_common_values.h"
#include "impeghe_type_def.h"
#include "impeghe_hoa_assignment_info.h"
#include "impeghe_hoa_config_struct.h"
#include "impeghe_hoa_frame_struct.h"

typedef struct
{
  WORD32 hoa_rend_id;
  WORD32 hoa_cicp;
  WORD32 hoa_matrix_in_dim;
  WORD32 hoa_matrix_out_dim;
  FLOAT64 hoa_matrix[MAX_NUM_HOA_COEFFS * MAX_NUM_PERC_CODERS];
} impeghe_hoa_matrix_cfg_str;

typedef struct
{
  WORD32 hoa_order;
  WORD32 uses_nfc;
  FLOAT32 nfc_distance;
  WORD32 num_hoa_coeffs;
  UWORD32 use_vec_est;
  WORD32 num_hoa_matrix;
  impeghe_hoa_matrix_cfg_str hoa_mat_cfg[MAX_NUM_RENDERER_MATRIX];
  WORD32 hoa_mtx_status;
} impeghe_hoa_config_str;

typedef struct
{
  UWORD32 frame_size;
  UWORD32 hoa_order;
  UWORD32 num_hoa_coeffs;
  UWORD32 amb_hoa_order;
  UWORD32 num_amb_hoa_coeffs;
  UWORD32 tot_perc_coders;
  UWORD32 num_var_ch;
  UWORD32 br_per_coder;

  UWORD32 act_amb_hoa_coeffs_idx[MAX_NUM_PERC_CODERS];
  UWORD32 pot_amb_hoa_coeff_idx[MAX_NUM_HOA_COEFFS];

  UWORD32 amb_hoa_coeffs_idx_to_enable[MAX_NUM_AMB_COEFF_INDICES];
  UWORD32 amb_hoa_coeffs_idx_to_disable[MAX_NUM_AMB_COEFF_INDICES];

  pFlOAT64 inv_low_order_mode_mat;

  ia_spatial_enc_assignment_info_str prev_tgt_assgn_vec[MAX_NUMBER_CHANNELS];
  ia_spatial_enc_assignment_info_str assgn_vec[MAX_NUMBER_CHANNELS];

} ia_spatial_enc_amb_comp_mod_str;

typedef struct
{
  UWORD32 frame_size;
  UWORD32 hoa_order;
  UWORD32 tot_perc_coders;
  UWORD32 num_var_ch;
} ia_spatial_enc_ch_assignment_str;

typedef struct
{
  UWORD32 frame_size;
  UWORD32 hoa_order;
  UWORD32 num_hoa_coeffs;
  UWORD32 min_amb_hoa_coeff;
  UWORD32 vec_ele_num_bits;
  UWORD32 num_vec_elem;
  WORD32 is_dir1_active;
  WORD32 is_dir2_active;
  UWORD32 frame_counter;
  WORD32 max_energy_dir[MAX_NUMBER_CHANNELS];
  FLOAT64 svd_input[2 * MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  UWORD32 use_vec_est;
} ia_spatial_enc_dir_vec_est_str;

typedef struct
{
  FLOAT64 last_gain;
  FLOAT64 last_max_abs_value;
  FLOAT64 smallest_delta;
  FLOAT64 max_val_absorption;
  WORD32 last_exp_base2;
  WORD32 prev_exp_base2;
  UWORD32 frame_size;
  UWORD32 max_amp_exp;
  UWORD32 max_attn_exp;
  pFlOAT64 win_func;
} ia_spatial_enc_dyn_correction_str;

typedef struct
{
  WORD32 is_exception[MAX_NUM_PERC_CODERS];
  WORD32 exponents[MAX_NUM_PERC_CODERS];
  ia_hoa_dir_id_str active_n_grid_dir_idx[MAX_NUMBER_CHANNELS];
  UWORD32 active_n_grid_dir_idx_for_subband_pred[MAX_SET_SIZE];
  ia_hoa_vec_sig_str vectors[MAX_NUMBER_CHANNELS];
  UWORD32 vec_sz;
  UWORD32 pred_typ_vec[MAX_NUM_HOA_COEFFS];
  UWORD32 pred_idx_mat[MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS];
  WORD32 quant_pred_fac_mat[MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS];
  UWORD32 amb_coeff_idx_to_enb[MAX_SET_SIZE];
  UWORD32 amb_coeff_idx_to_disable[MAX_SET_SIZE];
  UWORD32 non_en_dis_able_act_idx[MAX_SET_SIZE];
} ia_hoa_dec_frame_param_str;

typedef struct
{
  UWORD32 max_dir_sigs;
  UWORD32 hoa_order;
  UWORD32 frame_size;
  UWORD32 num_hoa_coeff;
  UWORD32 num_amb_ch;
  UWORD32 vector_start_ch;
  UWORD32 coded_vec_len;
  UWORD32 interp_samples;
  ia_hoa_vec_sig_str prev_vec[MAX_NUM_VECTOR_SIG_INDICES];
  UWORD32 prev_ps_sig_idx[MAX_NUM_HOA_DIR_SIGNALS];
} ia_spatial_dec_vector_based_predom_sound_syn_str;

typedef struct
{
  UWORD32 num_var_ch;
  UWORD32 hoa_order;
  UWORD32 frame_size;
  UWORD32 num_hoa_coeffs;
  UWORD32 min_amb_hoa_coeff;
  UWORD32 min_pred_dir_sig;
  UWORD32 scale_fac_num_bits;
  UWORD32 filter_delay;
  pFlOAT64 mode_mat_coarse_grid_pt;

  pFlOAT64 transp_mode_fine_grid_pt;
  FLOAT64 ones_vector[2 * MAX_FRAME_LEN];
  FLOAT64 pred_grid_dir_sigs_lst_frm[MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  FLOAT64 pred_grid_dir_sigs_cur_frm[MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  FLOAT64 pred_grid_dir_digs_smt_frm[MAX_FRAME_LEN * MAX_NUM_HOA_COEFFS];
  ia_hoa_dir_id_str lst_act_n_grid_dir_idx[MAX_NUM_HOA_COEFFS];
  UWORD32 old_ps_sig_idx[MAX_NUM_HOA_COEFFS];
  UWORD32 lst_pred_typ_vec[MAX_NUM_HOA_COEFFS];
  UWORD32 lst_pred_idx_mat[MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS];
  FLOAT64 lst_pred_fac_mat[MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS];
  FLOAT64 pred_fac_mat[MAX_NUM_HOA_COEFFS * MAX_NUM_HOA_COEFFS];
} ia_spatial_dec_dir_based_pre_dom_sound_syn_str;

typedef struct
{
  UWORD32 num_hoa_coeffs;
  UWORD32 frame_size;
  pFlOAT64 dir_sig_fade_in_win;
  pFlOAT64 dir_sig_fade_out_win;
  FLOAT64 vec_sig_fade_win[2 * MAX_FRAME_LEN];
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str dir_predom_syn_handle;
  ia_spatial_dec_vector_based_predom_sound_syn_str vec_predom_syn_handle;
} ia_spatial_dec_pre_dom_sound_syn_str;

typedef struct
{
  ia_hoa_config_struct *hoa_cfg;

  UWORD32 hoa_order;
  UWORD32 num_hoa_coeffs;
  UWORD32 frame_size;
  UWORD32 interp_samples;
  UWORD32 max_dir_sigs;
  UWORD32 min_amb_hoa_coeff;
  UWORD32 max_pred_dir_sigs;
  UWORD32 bits_per_sf;
  UWORD32 num_var_ch;
  UWORD32 br_per_coder;
  UWORD32 fine_grid_transp_mode_mat_sz;

  UWORD32 vec_start_ch;
  FLOAT64 vec_factor;
  UWORD32 vec_ele_num_bits;

  pVOID ptr_scratch;
  UWORD32 scratch_used_size;
  pFlOAT64 fine_grid_transp_mode_mat;
  pFlOAT64 fine_grid_transp_mode_mat_inv;

  pFlOAT64 dir_sigs_cross_fade_win;
  FLOAT64 vec_sigs_cross_fade_win[2 * MAX_FRAME_LEN];
  FLOAT64 ip_hoa_frame[3][MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  UWORD32 ip_idx;
  UWORD32 lst_ip_idx;
  UWORD32 penult_ip_idx;

  FLOAT64 vec_smtd_frame[2][MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  UWORD32 vec_smtd_frame_idx;
  UWORD32 vec_smtd_lst_frame_idx;

  FLOAT64 dir_smtd_frame[2][MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  UWORD32 dir_smtd_frame_idx;
  UWORD32 dir_smtd_lst_frame_idx;

  pFlOAT64 ps_sigs_smtd_lst_frame;
  ia_hoa_dir_id_str lst_sig_idx_map[MAX_NUMBER_CHANNELS];
  ia_hoa_vec_sig_str lst_vec[MAX_NUMBER_CHANNELS];

  ia_spatial_dec_pre_dom_sound_syn_str pre_dom_syn_handle;
} impeghe_hoa_decomposition_str;

typedef struct
{
  WORD32 ip_ended;
  WORD32 op_ended;

  UWORD32 delay_frm_cnt;
  UWORD32 req_delay_frm;

  UWORD32 frame_size;
  UWORD32 hoa_order;
  UWORD32 max_dir_sigs;
  UWORD32 amb_hoa_min_order;
  UWORD32 tot_perc_coders;
  UWORD32 br_per_perc_coders;
  UWORD32 num_add_perc_coders;
  UWORD32 max_pred_dir_sigs;
  UWORD32 min_amb_hoa_coeff;
  UWORD32 num_hoa_coeffs;
  UWORD32 explct_pred_idx_to_code;

  UWORD32 vec_ele_num_bits;

  UWORD32 max_num_hoa_coeffs; // not used
  UWORD32 coded_v_vec_len;
  UWORD32 send_ind_flag;

  ia_spatial_enc_assignment_info_str target_assgn_vec[MAX_NUM_PERC_CODERS];
  ia_spatial_enc_assignment_info_str assgn_vec[MAX_NUM_PERC_CODERS];

  FLOAT64 ps_sigs_smtd_lst_frame[MAX_NUM_DIR_SIGNAL * MAX_FRAME_LEN];

  pFlOAT64 penult_smtd_pre_dom_frm;
  pFlOAT64 penult_amb_hoa_frm;
  pFlOAT64 pred_lst_amb_hoa_frm;
  pFlOAT64 modfd_penult_amb_hoa_frm;
  pFlOAT64 modfd_pred_lst_amb_hoa_frm;

  UWORD32 act_hoa_coeff_idx[MAX_SET_SIZE];
  UWORD32 enble_amb_coeff_idx_thrd_lst_frm[MAX_SET_SIZE];
  UWORD32 disble_amb_coeff_idx_thrd_lst_frm[MAX_SET_SIZE];
  UWORD32 no_en_dis_able_amb_coeff_idx_thrd_lst_frm[MAX_SET_SIZE];

  FLOAT64 trans_ch[3][MAX_NUM_PERC_CODERS * MAX_FRAME_LEN];
  UWORD32 curr_op_idx;
  UWORD32 scd_lst_ip_idx;
  UWORD32 thrd_lst_ip_idx;

  ia_hoa_dec_frame_param_str frm_prm_hdl;
  impeghe_hoa_decomposition_str decmp_hdl;
  ia_spatial_enc_amb_comp_mod_str amb_comp_hdl;
  ia_spatial_enc_ch_assignment_str ch_asgnmnt_hdl;
  ia_spatial_enc_dir_vec_est_str dir_vec_est_hdl;
  ia_spatial_enc_dyn_correction_str dyn_corrctn_hdl[MAX_NUM_PERC_CODERS];

  ia_hoa_frame_struct penult_hoa_frm;
  ia_hoa_frame_struct thrd_lst_hoa_frm;

  pVOID ptr_scratch;
  UWORD32 scratch_used_size;
} ia_spatial_enc_str;

typedef struct
{
  ia_spatial_enc_str spat_enc_hdl;
  FLOAT64 trans_ch_dly_buf[MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN];
  UWORD32 num_hoa_coeffs;
  UWORD32 ip_samp_per_ch;
  UWORD32 tot_coders;
  UWORD32 op_samp_per_ch;
  UWORD32 hoa_frame_length;
  WORD32 ip_ended;
  WORD32 is_ip_ended;
  UWORD32
  hoa_si_per_frame;
  UWORD32 hoa_si_cnt;
  UWORD32 hoa_si_bits;
  WORD64 samp_read;
  WORD64 samp_written;
  WORD32 enc_dly_samp;
  UWORD32 add_samp_dly;
  WORD32 num_dummy_frm;
  ia_hoa_frame_struct dummy_hoa_frame;
  WORD32 frm_bs_bits[MAX_HOA_DUMMY_FRAMES];
  WORD32 prev_frm_bs_bits[MAX_HOA_DUMMY_FRAMES];
  WORD32 cfg_bs_bits;
  WORD32 frm_bs_cnt;
  ia_hoa_config_struct hoa_cfg;
  ia_hoa_frame_struct hoa_frm[HOA_MAX_BS_TRANS_FRAME_OFFSET + 1];
  pVOID ptr_scratch;
  UWORD32 scratch_used_size;
} ia_hoa_enc_str;

#endif /*IMPEGHE_HOA_STRUCT_H*/
