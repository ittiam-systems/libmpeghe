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

#ifndef IMPEGHE_CONFIG_H
#define IMPEGHE_CONFIG_H

#define USAC_MAX_ELEMENTS (32)
#define USAC_MAX_CONFIG_EXTENSIONS (16)

#define ID_USAC_SCE 0
#define ID_USAC_CPE 1
#define ID_USAC_LFE 2
#define ID_USAC_EXT 3

#define ID_EXT_ELE_FILL 0
#define ID_EXT_ELE_MPEGS 1
#define ID_EXT_ELE_SAOC 2
#define ID_EXT_ELE_AUDIOPREROLL 3
#define ID_EXT_ELE_UNI_DRC 4
#define ID_EXT_ELE_OAM 5
#define ID_EXT_ELE_HOA 7
#define ID_EXT_ELE_FMT_CNVRTR 8
#define ID_EXT_ELE_MCT 9
#define ID_EXT_ELE_ENHANCED_OBJ_METADATA 13

#define ID_CONFIG_EXT_FILL 0
#define ID_CONFIG_EXT_DOWNMIX (1)
#define ID_CONFIG_EXT_LOUDNESS_INFO (2)
#define ID_CONFIG_EXT_AUDIOSCENE_INFO (3)
#define ID_CONFIG_EXT_SIG_GROUP_INFO (6)
#define ID_CONFIG_EXT_OAM (3)
#define ID_CONFIG_EXT_HOA_MATRIX (4)
#define EXT_ELE_CFG_BIT_BUF_LEN_BYTE (4096)

typedef struct
{
  WORD32 dmx_id;
  WORD32 dmx_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 downmix_mtx_count;
  WORD32 num_assigned_group_ids[IMPEGHE_MAX_DMX_MATRICES_PER_ID];
  WORD32 signal_group_id[IMPEGHE_MAX_DMX_MATRICES_PER_ID][IMPEGHE_MAX_DMX_MAX_GROUPS_ASSIGNED];
  ia_dmx_matrix_enc_cfg_struct str_dmx_mtx_cfg;
} ia_mpeghe_ext_cfg_dmx_mtx_struct;

typedef struct
{
  UWORD32 dmx_config_type;
  UWORD32 passive_dmx_flag;
  UWORD32 phase_align_strength;
  UWORD32 immersive_downmix_flag;
  UWORD32 downmix_id_count;
  ia_mpeghe_ext_cfg_dmx_mtx_struct str_dmx_matrix[IMPEGHE_MAX_DMX_MATRIX_ELEMENTS];
  UWORD8 dmx_matrix_enc_mem[EXT_ELE_CFG_BIT_BUF_LEN_BYTE];
} ia_mpeghe_ext_cfg_downmix_struct;

typedef union _ia_mpeghe_ext_element_config_union_ {
  ia_mpeghe_ext_cfg_downmix_struct str_ext_cfg_downmix;
} ia_mpeghe_ext_element_config_union;

typedef struct
{
  UWORD32 usac_ext_ele_type;
  UWORD32 usac_ext_ele_cfg_len;
  UWORD32 usac_ext_ele_dflt_len_present;
  UWORD32 usac_ext_ele_dflt_len;
  UWORD32 usac_ext_ele_payload_present;
  UWORD32 stereo_config_index;
  UWORD32 tw_mdct;
  UWORD32 noise_filling;
  UWORD32 full_band_lpd;
  UWORD32 enhanced_noise_filling;
  UWORD32 igf_use_enf;
  UWORD32 igf_use_high_res;
  UWORD32 igf_use_whitening;
  UWORD32 igf_after_tns_synth;
  UWORD32 igf_start_index;
  UWORD32 igf_stop_index;
  UWORD32 igf_independent_tiling;
  UWORD8 usac_ext_ele_cfg_payload[MAX_CHANNEL_BITS / MAX_SHORT_WINDOWS];

  FLAG oam_has_core_length;
  WORD32 oam_block_size;
  FLAG oam_has_scrn_rel_objs;
  WORD32 oam_num_objects;
  FLAG oam_is_scrn_rel_obj[OAM_MAX_NUM_OBJECTS];
  FLAG oam_has_dyn_obj_priority;
  FLAG oam_has_uniform_spread;

  pUWORD8 hoa_config_bs;
  UWORD32 hoa_config_len;
  WORD32 use_hoa;
  WORD32 use_hoa_matrix;
  WORD32 hoa_num_trans_ch;
  WORD32 lpd_stereo_idx;
  UWORD8 *drc_config_data;
} ia_usac_enc_element_config_struct;
typedef struct
{
  WORD8 num_elements;
  WORD8 num_ext_elements;
  UWORD32 usac_element_type[USAC_MAX_ELEMENTS];
  UWORD32 usac_cfg_ext_present;
  UWORD32 num_config_extensions;
  UWORD32 usac_config_ext_type[USAC_MAX_CONFIG_EXTENSIONS];
  UWORD32 usac_config_ext_len[USAC_MAX_CONFIG_EXTENSIONS];
  UWORD8 usac_cfg_ext_info_buf[USAC_MAX_CONFIG_EXTENSIONS][MAX_CHANNEL_BITS / MAX_SHORT_WINDOWS];
  WORD32 num_out_channels;
  WORD32 num_signal_grp;
  ia_usac_enc_element_config_struct str_usac_element_config[USAC_MAX_ELEMENTS];
  ia_mpeghe_ext_element_config_union str_extn_element_config[USAC_MAX_CONFIG_EXTENSIONS];

  ia_mpeghe_ext_cfg_downmix_struct str_ext_cfg_downmix[USAC_MAX_CONFIG_EXTENSIONS];

} ia_usac_config_struct;

typedef struct
{
  UWORD32 samp_frequency_index;
  UWORD32 sampling_frequency;
  UWORD32 channel_configuration;
  UWORD32 ext_audio_object_type;
  UWORD32 ext_samp_frequency_index;
  UWORD32 ext_sampling_frequency;
  UWORD32 ext_sync_word;
  ia_usac_config_struct str_usac_config;
  ia_mae_audio_scene_info str_asi_info;
  UWORD32 num_audio_objs;
  UWORD32 num_audio_chs;
  UWORD32 num_audio_channels;
  WORD32 num_ch_sig_groups;
  WORD32 num_sig_grps;
  WORD32 num_ch_per_sig_group[16];
  WORD32 num_ch_idx_per_grp[MAX_TIME_CHANNELS];
  WORD32 num_obj_sig_groups;
  WORD32 num_objs_per_sig_group[16];
  WORD32 num_hoa_sig_groups;
  WORD32 num_hoas_per_sig_group[16];
  UWORD32 num_hoa_transport_channels;
  UWORD32 profile_info;
  WORD32 flex_spk_azi[MAX_NUM_OF_SPEAKERS];
  WORD32 flex_spk_ele[MAX_NUM_OF_SPEAKERS];
  WORD8 flex_spk_islfe[MAX_NUM_OF_SPEAKERS];
  WORD32 num_spk;
  WORD8 flex_spk_enable;
} ia_usac_audio_specific_config_struct;

typedef struct
{
  FLOAT64 *p_fd_mdct_windowed_long_buf;
  FLOAT64 *p_fd_mdct_windowed_short_buf;
  FLOAT32 *p_fft_mdct_buf;
  FLOAT64 *p_sort_grouping_scratch;
  WORD32 *p_degroup_scratch;
  WORD32 *p_arith_map_prev_scratch;
  WORD32 *p_arith_map_pres_scratch;
  FLOAT64 *p_noise_filling_highest_tone;
  FLOAT32 *p_lpd_frm_enc_scratch;
  FLOAT64 *p_quant_spectrum_spec_scratch;
  FLOAT64 *p_fdp_href;
  FLOAT64 *p_fdp_mdct_out;
  UWORD8 *ptr_scratch_buf;
  FLOAT32 *p_synth_tcx_buf;
  FLOAT32 *p_synth_buf;
  FLOAT32 *p_wsig_buf;
  FLOAT32 *p_wsyn_buf;
  FLOAT32 *p_wsyn_tcx_buf;
  FLOAT32 *p_temp_wsyn_buf;
  FLOAT32 *p_buf_aut_corr;
  FLOAT32 *p_buf_synthesis_tool;
  FLOAT32 *p_buf_speech;
  FLOAT32 *p_buf_res;
  FLOAT32 *p_buf_signal;
  FLOAT32 *p_lp_filter_coeff;
  FLOAT32 *p_lp_filter_coeff_q;
  WORD32 *p_prm_tcx;
  FLOAT32 *p_wsp_prev_buf;
  FLOAT32 *p_xn2;
  FLOAT32 *p_fac_dec;
  FLOAT32 *p_right_fac_spec;
  FLOAT32 *p_x2;
  WORD32 *p_param;
  FLOAT32 *p_x;
  FLOAT32 *p_xn_2;
  FLOAT32 *p_fac_window;
  FLOAT32 *p_temp_mdct;
  WORD16 *p_fac_bits_word;
  FLOAT64 *p_left_fac_time_data;
  FLOAT32 *p_left_fac_timedata_flt;
  FLOAT32 *p_left_fac_spec;
  FLOAT64 *p_fac_win;
  WORD32 *p_fac_prm;
  FLOAT32 *p_acelp_folded_scratch;
  FLOAT32 *p_xn1_tcx;
  FLOAT32 *p_xn_buf_tcx;
  FLOAT32 *p_x_tcx;
  FLOAT32 *p_x_tmp_tcx;
  FLOAT32 *p_en_tcx;
  FLOAT32 *p_alfd_gains_tcx;
  FLOAT32 *p_sq_enc_tcx;
  WORD32 *p_sq_quant_tcx;
  FLOAT32 *p_gain1_tcx;
  FLOAT32 *p_gain2_tcx;
  FLOAT32 *p_facelp_tcx;
  FLOAT32 *p_xn2_tcx;
  FLOAT32 *p_fac_window_tcx;
  FLOAT32 *p_x1_tcx;
  FLOAT32 *p_x2_tcx;
  WORD32 *p_y_tcx;
  FLOAT32 *p_in_out_tcx;
  FLOAT32 *p_time_signal;
  FLOAT32 *p_complex_fft;
  WORD32 *p_tonal_flag;
  FLOAT32 *p_pow_spec;
  FLOAT64 *p_tns_filter;
  FLOAT32 *p_exp_spec;
  FLOAT32 *p_mdct_spec_float;
  FLOAT32 *p_fir_sig_buf;
  FLOAT32 *p_sq_gain_en;
  FLOAT32 *p_acelp_ir_buf;
  FLOAT32 *p_acelp_exc_buf;
  FLOAT32 *p_adjthr_ptr_exp_spec;
  FLOAT32 *p_adjthr_mdct_spec_float;
  WORD16 *p_adjthr_quant_spec_temp;
  FLOAT64 *p_cmpx_mdct_temp_buf;
  FLOAT32 *p_fft_p2_y;
  FLOAT32 *p_fft_p3_data_3;
  FLOAT32 *p_fft_p3_y;
  FLOAT32 *p_tcx_input;
  FLOAT32 *p_tcx_output;
  FLOAT64 *p_reconstructed_time_signal[2];
  FLOAT32 *p_ltpf_scratch;
  FLOAT32 *p_ol_pitch_buf_tmp;
  FLOAT32 *p_ol_pitch_speech_buf;
  FLOAT32 *p_ol_pitch_w_table;
  FLOAT32 *p_ol_pitch_R;
  FLOAT32 *p_ol_pitch_R0;
  FLOAT32 *p_ltpf_encode_speech_buffer;
  FLOAT32 *p_ltpf_encode_buf_tmp;
  FLOAT32 *p_ltpf_encode_cor;
  FLOAT32 *p_ltpf_encode_cor_int;
  WORD32 *p_c_prev; // size 516
  WORD32 *p_c_pres; // size 516
  VOID *pstr_dmx_scratch;
  VOID *ptr_drc_scratch_buf;
  WORD32 *ptr_num_fac_bits;
  FLOAT32 *ptr_tmp_lp_res;

  FLOAT32 *ptr_sfb_form_fac[2];
  FLOAT32 *ptr_sfb_num_relevant_lines[2];
  FLOAT32 *ptr_sfb_ld_energy[2];
  pUWORD8 ptr_fd_scratch;
  pUWORD8 ptr_lpd_scratch;
  FLOAT32 *ptr_tcx_scratch;
  FLOAT64 *ptr_tns_scratch;
  WORD32 *ptr_fdp_int;
  FLOAT64 *ptr_igf_scratch;
  FLOAT32 *ptr_acelp_scratch;
  FLOAT32 mixed_rad_fft[2 * FRAME_LEN_LONG];
} impeghe_scratch_mem;

WORD32 impeghe_get_audiospecific_config_bytes(
    ia_bit_buf_struct *it_bit_buff, impeghe_scratch_mem *pstr_scratch,
    ia_usac_audio_specific_config_struct *pstr_audio_specific_config);
#endif /* IMPEGHE_CONFIG_H */
