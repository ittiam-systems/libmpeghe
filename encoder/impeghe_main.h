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

#ifndef IMPEGHE_MAIN_H
#define IMPEGHE_MAIN_H

#include "impeghe_type_def.h"
#include "impeghe_hoa_config_struct.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_encoder.h"

#define MINIMUM_BITRATE_PER_CHANNEL (32000)

typedef struct
{
  WORD32 ltpf_active;
  WORD32 ltpf_pitch_index;
  WORD32 ltpf_gain_index;
  const FLOAT32 *fir_lpf_coefs;
  FLOAT32 olpa_mem_x[KMAX_MAX];
  WORD32 olpa_mem_pitch;
  FLOAT32 ltpf_mem_in[1030];
  WORD32 ltpf_mem_in_len;
  FLOAT32 ltpf_mem_normcorr;
  WORD32 ltpf_mem_ltpf_on;
  WORD32 ltpf_mem_pitch;
  WORD32 ltpf_pitch_min;
  WORD32 ltpf_pitch_fr2;
  WORD32 ltpf_pitch_fr1;
  WORD32 ltpf_pitch_max;
  WORD32 ltpf_pitch_res;
  WORD32 ltpf_pitch_res_by_2;
  WORD32 pitch_search_delta;
  WORD32 pitch_search_l_interpol;
} ia_usac_ltpf_data_struct;

typedef struct
{
  WORD32 fdp_active[MAX_TIME_CHANNELS];
  WORD32 fdp_spacing_idx[MAX_TIME_CHANNELS];
  WORD32 fdp_prev_s0[MAX_TIME_CHANNELS];
  WORD16 fdp_quant_spec_prev[MAX_TIME_CHANNELS][2 * 172];
  WORD32 fdp_quant_spec_prev_tcx[MAX_TIME_CHANNELS][2][172]; // td
} ia_usac_fdp_data_struct;

typedef struct
{
  WORD32 mode;
  WORD32 num_bits;
  FLOAT32 lpc_coeffs_quant[2 * (ORDER + 1)];
  FLOAT32 lpc_coeffs[2 * (ORDER + 1)];
  FLOAT32 synth[ORDER + 128];
  FLOAT32 wsynth[1 + 128];
  FLOAT32 acelp_exc[2 * LEN_FRAME];
  WORD32 avq_params[FAC_LENGTH];
  FLOAT32 tcx_mem[128];
  FLOAT32 tcx_quant[1 + (2 * 128)];
  FLOAT32 tcx_fac;
  FLOAT32 mem_wsyn;
} ia_usac_lpd_state_struct;
typedef struct
{
  ia_usac_lpd_state_struct lpd_state[6];
  ia_usac_lpd_state_struct flpd_state[6];

} ia_usac_lpd_scratch;
typedef struct
{
  WORD32 len_frame;
  WORD32 len_subfrm;
  WORD32 num_subfrm;
  WORD16 acelp_core_mode;
  WORD32 fscale;
  FLOAT32 mem_lp_decim2[3];
  WORD32 decim_frac;
  FLOAT32 mem_sig_in[4];
  FLOAT32 mem_preemph;
  FLOAT32 old_speech_pe[L_OLD_SPEECH_HIGH_RATE + LEN_LPC0];
  FLOAT32 weighted_sig[128];
  ia_usac_lpd_state_struct lpd_state;
  FLOAT32 prev_wsp[MAX_PITCH / OPL_DECIM];
  FLOAT32 prev_exc[MAX_PITCH + LEN_INTERPOL];
  FLOAT32 prev_wsyn_mem;
  FLOAT32 prev_wsp_mem;
  FLOAT32 prev_xnq_mem;
  WORD32 prev_ovlp_size;
  FLOAT32 isf_old[ORDER];
  FLOAT32 isp_old[ORDER];
  FLOAT32 isp_old_q[ORDER];
  FLOAT32 isf_flpd_old[ORDER];
  FLOAT32 isp_flpd_old[ORDER];
  FLOAT32 isp_flpd_old_q[ORDER];
  FLOAT32 mem_wsp;
  FLOAT32 ada_w;
  FLOAT32 ol_gain;
  WORD16 ol_wght_flg;
  WORD32 prev_ol_lags[5];
  WORD32 prev_pitch_med;
  FLOAT32 prev_hp_wsp[FRAME_LEN_LONG / OPL_DECIM + (MAX_PITCH / OPL_DECIM)];
  FLOAT32 hp_ol_ltp_mem[3 * 2 + 1];
  const FLOAT32 *lp_analysis_window;
  FLOAT32 xn_buffer[128];
  WORD32 c_prev[(FRAME_LEN_LONG / 2) + 4];
  WORD32 c_pres[(FRAME_LEN_LONG / 2) + 4];
  WORD32 arith_reset_flag;
  WORD16 prev_mode;
  WORD32 num_bits_per_supfrm;
  FLOAT32 fd_synth[2 * LEN_FRAME + 1 + ORDER];
  FLOAT32 fd_orig[2 * LEN_FRAME + 1 + ORDER];
  WORD32 low_pass_line;
  WORD32 last_was_short;
  WORD32 next_is_short;
  FLOAT32 gain_tcx;
  FLOAT32 flpd_speech_buf[((LEN_TOTAL_HIGH_RATE) >> 1) + LEN_LPC0];
  FLOAT32 flpd_resamp_filt_mem[DOWNSAMP_FILT_MEM_SIZE];
  FLOAT32 scratch_mem[DOWNSAMP_FILT_MEM_SIZE + LEN_TOTAL_HIGH_RATE + LEN_LPC0];
  FLOAT32 prev_flpd_wsp[MAX_PITCH / OPL_DECIM];
  WORD32 max_sfb_short;
} ia_usac_td_encoder_struct;

typedef struct
{
  WORD8 ec_present;
  WORD32 ec_end_frame;
  WORD32 num_earcons;
  WORD8 ec_active;
  WORD32 n_channels;
  WORD32 sample_rate;
  WORD32 pcm_sz;
  WORD32 i_channel_mask;
  WORD32 length;
  UWORD8 *ptr_ec_buff;
  UWORD32 is_independent;
  UWORD32 has_text_label;
  UWORD32 type;
  UWORD32 active;
  UWORD32 position_type;
  UWORD32 cicp_idx;
  UWORD32 has_gain;
  WORD32 azimuth;
  WORD32 elevation;
  WORD32 distance;
  UWORD32 gain;
  WORD8 not_first_frame;
} ia_ec_config_struct;
typedef struct
{
  WORD32 pcm_packet_type_present;
  WORD32 bs_num_pcm_signals;
  WORD32 pcm_align_audio_flag;
  WORD32 pcm_sampling_rate_idx;
  WORD32 pcm_sampling_rate;
  WORD32 pcm_bits_per_sample_idx;
  WORD32 pcm_frame_size_idx;
  WORD32 pcm_fix_frame_size;
  WORD32 pcm_signal_id[128];
  WORD32 bs_pcm_loudness_value;
  WORD32 pcm_has_attenuation_gain;
  WORD32 bs_pcm_attenuation_gain;
  WORD32 pcm_bits_per_sample;
  WORD32 pcm_packet_data_present;
  WORD32 bsnum_pcm_signals_in_frame;
  WORD32 pcm_var_frame_size;
  WORD32 *pcm_sample;

  WORD32 num_bytes_written;

} ia_pcm_data_config;
typedef struct
{
  WORD32 iframes_interval;
  WORD32 ccfl;
  WORD32 bit_rate;
  WORD32 channels;
  WORD32 bw_limit[USAC_MAX_ELEMENTS];
  WORD32 use_fill_element;
  WORD32 prof_level;
  WORD32 use_oam_element;
  WORD32 use_hoa;
  WORD32 use_hoa_matrix;
  WORD32 cicp_index;
  WORD32 codec_mode;
  WORD32 flag_noiseFilling;
  WORD32 enhanced_noise_filling;
  WORD32 igf_start_freq;
  WORD32 igf_stop_freq;
  WORD32 ltpf_enable;
  WORD32 igf_after_tns_synth;
  WORD32 full_band_lpd;
  WORD32 stereo_lpd;
  WORD32 mhas_pkt;
  WORD32 crc16;
  WORD32 crc32;
  WORD32 global_crc16;
  WORD32 global_crc32;
  WORD32 aud_ch;
  WORD32 num_trans_ch;
  WORD32 user_specified;
  WORD32 num_oam_ch;
  WORD32 oam_bitrate;
  WORD32 basic_bitrate;
  WORD32 hoa_bitrate;
  ia_bit_buf_struct mhas_bit_buf;
  ia_bit_buf_struct *ptr_mhas_bit_buf;
  UWORD8 mhas_buf[15];
  WORD32 prev_aliasing_symmetry;
  WORD32 curr_aliasing_symmetry;
  WORD32 tns_select;
  WORD32 window_shape_prev[MAX_TIME_CHANNELS];
  WORD32 window_shape_prev_copy[MAX_TIME_CHANNELS];
  WORD32 window_sequence[MAX_TIME_CHANNELS];
  WORD32 window_sequence_prev[MAX_TIME_CHANNELS];
  WORD32 window_sequence_prev_copy[MAX_TIME_CHANNELS];
  WORD32 cmplx_pred_flag;
  WORD32 sampling_rate;
  WORD32 native_sampling_rate;
  WORD32 ui_pcm_wd_sz;
  ia_usac_audio_specific_config_struct audio_specific_config;
  WORD32 mct_mode;
  WORD16 oam_version;
  WORD16 has_dyn_obj_priority;
  WORD16 has_uniform_spread;
  WORD32 num_objects;
  WORD32 num_channels;
  WORD32 num_elements;
  WORD8 num_aud_elements;
  WORD8 num_hoa_elements;
  WORD8 num_oam_elements;
  WORD8 num_ext_elements;
  WORD32 extra_objects;
  FLAG oam_high_rate;
  WORD32 oam_replace_radius;
  FLAG oam_fixed_values[6];
  FLAG oam_has_core_length;
  FLAG oam_has_scrn_rel_objs;
  FLAG oam_is_scrn_rel_obj[OAM_MAX_NUM_OBJECTS];
  void *oam_data_hndl;
  WORD32 (*oam_read_data)(void *oam_data_hndl, UWORD8 *buff, WORD32 bytes_to_read);
  WORD32 (*oam_skip_data)(void *oam_data_hndl, WORD32 bytes_to_skip);
  WORD32 in_frame_length;
  WORD32 cc_resamp_fac_down;
  WORD32 cc_resamp_fac_up;
  impeghe_hoa_config_str hoa_config;
  ia_ec_config_struct str_ec_config_struct;
  ia_pcm_data_config str_pcm_data_config;
  // DRC Params
  FLAG use_drc_element;
  ia_drc_input_config str_drc_cfg;

  // Downmix extention config
  FLAG use_downmix_ext_config;
  ia_mpeghe_ext_cfg_downmix_input_struct str_ext_cfg_downmix_input;
  ia_sfb_params_struct str_sfb_prms;
  WORD32 fdp_enable;
} ia_usac_encoder_config_struct;

typedef struct
{
  WORD32 usac_independency_flag_interval;
  WORD32 usac_independency_flag_count;
  WORD32 frame_count;
  WORD32 core_mode[MAX_TIME_CHANNELS];
  WORD32 core_mode_prev[MAX_TIME_CHANNELS];
  WORD32 core_mode_prev_copy[MAX_TIME_CHANNELS];
  WORD32 core_mode_next[MAX_TIME_CHANNELS];
  WORD32 core_mode_copy[MAX_TIME_CHANNELS];
  ia_block_switch_ctrl block_switch_ctrl[MAX_TIME_CHANNELS];
  ia_classification_struct str_sig_class_data;
  FLOAT32 td_in_buf[MAX_TIME_CHANNELS][FRAME_LEN_LONG + LEN_NEXT_HIGH_RATE];
  FLOAT32 td_in_prev_buf[MAX_TIME_CHANNELS][FRAME_LEN_LONG + LEN_NEXT_HIGH_RATE + LEN_LPC0];
  FLOAT32 speech_buf[LEN_TOTAL_HIGH_RATE + LEN_LPC0];
  FLOAT32 synth_buf[ORDER + FRAME_LEN_LONG];
  WORD32 param_buf[(NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV) + NUM_LPC_PRM];
  WORD32 param_buf_flpd[(NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV) + NUM_LPC_PRM];
  ia_usac_td_encoder_struct *td_encoder[MAX_TIME_CHANNELS];
  WORD32 total_nbbits[MAX_TIME_CHANNELS];
  WORD32 FD_nbbits_fac[MAX_TIME_CHANNELS];
  WORD32 num_td_fac_bits[MAX_TIME_CHANNELS];
  WORD32 td_bitrate[MAX_TIME_CHANNELS];
  WORD32 acelp_core_mode[MAX_TIME_CHANNELS];
  WORD32 max_bitreservoir_bits;
  WORD32 available_bitreservoir_bits;
  ia_ms_info_struct str_ms_info[MAX_TIME_CHANNELS];
  WORD32 pred_coef_re[MAX_TIME_CHANNELS][MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 pred_coef_im[MAX_TIME_CHANNELS][MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 pred_coef_re_prev[MAX_TIME_CHANNELS][MAX_NUM_SFB_LONG];
  WORD32 pred_coef_im_prev[MAX_TIME_CHANNELS][MAX_NUM_SFB_LONG];
  /* Temporary buffers for bitstream writing function when computing static bits */
  WORD32 temp_pred_coef_re_prev[MAX_TIME_CHANNELS][MAX_NUM_SFB_LONG];
  WORD32 temp_pred_coef_im_prev[MAX_TIME_CHANNELS][MAX_NUM_SFB_LONG];
  ia_tns_info *pstr_tns_info[MAX_TIME_CHANNELS];
  WORD32 noise_offset[2];
  WORD32 noise_level[2];
  ia_usac_quant_info_struct str_quant_info[2];
  ia_igf_data_struct str_igf_data[MAX_TIME_CHANNELS];
  ia_igf_config_struct str_igf_config[MAX_TIME_CHANNELS];
  WORD32 prev_aliasing_symmetry[MAX_TIME_CHANNELS];
  WORD32 curr_aliasing_symmetry[MAX_TIME_CHANNELS];
  WORD32 noise_filling[MAX_TIME_CHANNELS];
  ia_psy_mod_struct str_psy_mod;
  ia_qc_main_struct str_qc_main;
  FLOAT64 *ptr_time_data[MAX_TIME_CHANNELS];
  FLOAT64 *ptr_look_ahead_time_data[MAX_TIME_CHANNELS];
  FLOAT64 *spectral_line_vector[MAX_TIME_CHANNELS];
  FLOAT64 mdst_spectrum[MAX_TIME_CHANNELS][FRAME_LEN_LONG];
  FLOAT64 *ptr_2frame_time_data[MAX_TIME_CHANNELS];
  WORD16 td_serial_out[MAX_TIME_CHANNELS][NBITS_MAX];
  WORD16 fac_out_stream[MAX_TIME_CHANNELS][NBITS_MAX];
  FLOAT64 overlap_buf[MAX_TIME_CHANNELS][2048];
  impeghe_scratch_mem str_scratch;
  ia_usac_ltpf_data_struct ltpf_data[MAX_TIME_CHANNELS];
  ia_resampler_struct resampler[MAX_TIME_CHANNELS];
  ia_usac_fdp_data_struct fdp_data;
  ia_usac_slpd_enc_data_str str_slpd_data[MAX_TIME_CHANNELS >> 1];
  ia_slpd_scratch_str str_slpd_scratch;
  ia_usac_tbe_data_str str_tbe_data;
  WORD32 channel_elem_type[MAX_TIME_CHANNELS];
  WORD32 channel_elem_idx[MAX_TIME_CHANNELS];
  WORD32 num_ext_elements;
  WORD32 ext_type[MAX_EXTENSION_PAYLOADS];
  UWORD8 ext_elem_config_payload[MAX_EXTENSION_PAYLOADS][MAX_EXTENSION_PAYLOAD_LEN];
  UWORD32 ext_elem_config_len[MAX_EXTENSION_PAYLOADS];
  mct_data_t mct_data[MAX_CH_TYPE_SIG_GROUP];
  UWORD32 mct_data_bit_cnt;
  ia_oam_enc_state_struct str_oam_state;
  UWORD32 oam_data_bit_cnt;

  ia_hoa_enc_str str_hoa_state;
  ia_drc_enc_state str_drc_state;
  UWORD32 drc_data_bit_cnt;
  WORD32 pred_dir_idx[MAX_TIME_CHANNELS];
  WORD32 cplx_pred_all[MAX_TIME_CHANNELS];
  WORD32 cplx_pred_used[MAX_TIME_CHANNELS][MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 delta_code_time[MAX_TIME_CHANNELS];
  WORD32 complex_coef[MAX_TIME_CHANNELS];
  FLOAT64 *ptr_dmx_re_save[MAX_TIME_CHANNELS]; /*For saving previous frame MDCT down-mix */
  FLOAT64 *ptr_dmx_im[MAX_TIME_CHANNELS];
  FLOAT64 arr_dmx_im[MAX_TIME_CHANNELS][(FRAME_LEN_LONG + FRAME_LEN_LONG / 8)];
  FLOAT64 arr_dmx_save_float[MAX_TIME_CHANNELS][(FRAME_LEN_LONG + FRAME_LEN_LONG / 8)];
  FLOAT64 left_chan_save[MAX_TIME_CHANNELS][(FRAME_LEN_LONG + FRAME_LEN_LONG / 8)];
  FLOAT64 right_chan_save[MAX_TIME_CHANNELS][(FRAME_LEN_LONG + FRAME_LEN_LONG / 8)];

  FLOAT32 p_flpd_synth_buf[128 + LEN_FRAME];
  FLOAT32 p_flpd_wsig_buf[LEN_FRAME];
  FLOAT32 p_flpd_wsyn_buf[128 + LEN_FRAME];
  FLOAT32 p_flpd_temp_wsyn_buf[LEN_FRAME];
  FLOAT32 p_flpd_wsp_prev_buf[((MAX_PITCH1 / OPL_DECIM) + LEN_FRAME)];
} ia_usac_data_struct;

typedef struct
{
  ia_usac_encoder_config_struct *p_config;
  WORD32 i_out_bytes;
  UWORD32 ui_in_bytes;
  UWORD32 ui_input_over;
  UWORD32 ui_init_done;
  UWORD32 ui_bytes_consumed;
  ia_bit_buf_struct str_bit_buf;
  FLOAT32 **ptr_in_buf;
  ia_usac_data_struct str_usac_enc_data;
  ia_bit_buf_struct str_hoa_bit_buf;
  ia_bit_buf_struct str_hoa_matrix_bit_buf;
  UWORD8 hoa_bit_stream[MAX_FRAME_LEN];
  UWORD8 prev_hoa_bit_stream[MAX_FRAME_LEN];
  UWORD8 hoa_matrix_bit_stream[1024];
  pVOID hoa_scratch;
  pVOID drc_scratch;
  UWORD32 ui_num_inp_received;
  jmp_buf *impegh_jmp_buf;

} ia_usac_enc_state_struct;

typedef struct ia_mpeghe_api_struct
{
  ia_usac_enc_state_struct *p_state_mpeghe;
  ia_usac_encoder_config_struct config;
  ia_mem_info_struct *p_mem_info_mpeghe;
  pVOID *pp_mem;
} ia_mpeghe_api_struct;

IA_ERRORCODE impeghe_fd_encode(ia_sfb_params_struct *pstr_sfb_prms, WORD32 usac_independancy_flag,
                               ia_usac_data_struct *pstr_usac_data,
                               ia_usac_encoder_config_struct *pstr_usac_config,
                               ia_bit_buf_struct *it_bit_buff, WORD32 nr_core_coder_ch,
                               WORD32 chn, WORD32 ele_id, WORD32 *bit_written);

IA_ERRORCODE impeghe_enc_init(ia_usac_encoder_config_struct *ptr_usac_config,
                              ia_usac_enc_state_struct *p_state);

IA_ERRORCODE impeghe_core_coder_process(FLOAT32 **ptr_input,
                                        ia_usac_encoder_config_struct *ptr_usac_config,
                                        ia_usac_enc_state_struct *p_state,
                                        ia_bit_buf_struct *it_bit_buff);

IA_ERRORCODE impeghe_quantize_spec(ia_sfb_params_struct *pstr_sfb_prms,
                                   WORD32 usac_independancy_flag, WORD32 num_chans,
                                   ia_usac_data_struct *ptr_usac_data,
                                   ia_usac_encoder_config_struct *ptr_usac_config, WORD32 chn,
                                   WORD32 ele_id);

IA_ERRORCODE impeghe_grouping(ia_sfb_params_struct *pstr_sfb_prms, WORD32 num_chans,
                              ia_usac_data_struct *ptr_usac_data,
                              ia_usac_encoder_config_struct *ptr_usac_config, WORD32 chn,
                              WORD32 ele_id);

VOID impeghe_stereo_proc(ia_sfb_params_struct *pstr_sfb_prms, WORD32 usac_independancy_flag,
                         ia_usac_data_struct *ptr_usac_data,
                         ia_usac_encoder_config_struct *ptr_usac_config, WORD32 chn,
                         WORD32 ele_id);

VOID impeghe_fdp_encode(ia_usac_data_struct *pstr_usac_data,
                        ia_usac_encoder_config_struct *pstr_usac_config, WORD32 i_ch,
                        WORD32 max_sfb, WORD32 core_mode);

VOID impeghe_fd_chn_fdp_decode_update(ia_usac_data_struct *pstr_usac_data, WORD32 max_sfb,
                                      WORD32 chn, WORD32 is_long, WORD32 *ptr_fdp_int,
                                      WORD32 idx);

VOID impeghe_tcx_fdp_encode(ia_usac_fdp_data_struct *pstr_fdp_data, FLOAT32 *ptr_out_spec_curr,
                            FLOAT32 *ptr_tcx_quant, WORD32 quant_gain_curr, FLOAT32 i_gain,
                            WORD32 lg, WORD32 max_lines, WORD32 pred_bw, WORD32 chn,
                            WORD32 *ptr_fdp_int);

VOID impeghe_tcx_fdp_encode_update(ia_usac_fdp_data_struct *st, FLOAT32 *out_spec_curr,
                                   FLOAT32 *tcx_quant, WORD32 quant_gain_curr, FLOAT32 i_gain,
                                   WORD32 lg, WORD32 max_lines, WORD32 pred_bw, WORD32 chn);

VOID impeghe_classification(ia_classification_struct *pstr_sig_class,
                            impeghe_scratch_mem *pstr_scratch);

WORD32 impeghe_write_pcm_data_pkt(ia_bit_buf_struct *it_bit_buff,
                                  ia_pcm_data_config *ptr_pcm_data_config);
WORD32 impeghe_write_pcm_config_pkt(ia_bit_buf_struct *it_bit_buff,
                                    ia_pcm_data_config *ptr_pcm_data_config);
WORD32 impeghe_write_ec_pkt(ia_bit_buf_struct *it_bit_buff,
                            ia_ec_config_struct *ptr_ec_config_struct);
WORD32 impeghe_mhas_write_earcon_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                        UWORD32 num_bits);

WORD32 impeghe_mhas_write_crc_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                     WORD32 crc_len, UWORD32 crc_val);
WORD32 impeghe_mhas_write_global_crc_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                            WORD32 crc_len, UWORD32 crc_val,
                                            UWORD32 global_crc_type,
                                            UWORD32 num_protected_packets);

WORD32 impeghe_mhas_write_sync_header(ia_bit_buf_struct *it_bit_buff);
WORD32 impeghe_mhas_write_cfg_only_header(ia_bit_buf_struct *it_bit_buff, UWORD32 num_bits);

WORD32 impeghe_mhas_write_frame_header(ia_bit_buf_struct *it_bit_buff, UWORD32 num_bits);

#endif /* IMPEGHE_MAIN_H */
