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

#ifndef IMPEGHE_LPD_H
#define IMPEGHE_LPD_H

VOID impeghe_autocorr_plus(FLOAT32 *speech, FLOAT32 *auto_corr_vector, WORD32 window_len,
                           const FLOAT32 *lp_analysis_win, FLOAT32 *temp_aut_corr);
VOID impeghe_compute_lp_residual(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l);
VOID impeghe_convolve(FLOAT32 *signal, FLOAT32 *wsynth_filter_ir, FLOAT32 *conv_out);
VOID impeghe_synthesis_tool_float(FLOAT32 *a, FLOAT32 *x, FLOAT32 *y, WORD32 l, FLOAT32 *mem,
                                  FLOAT32 *buf_synth_tool);
VOID impeghe_apply_preemph(FLOAT32 *signal, FLOAT32 factor, WORD32 length, FLOAT32 *mem);
VOID impeghe_apply_deemph(FLOAT32 *signal, FLOAT32 factor, WORD32 length, FLOAT32 *mem);
VOID impeghe_lpc_2_lsp_conversion(FLOAT32 *lpc, FLOAT32 *lsp, FLOAT32 *prev_lsp);
VOID impeghe_levinson_durbin_algo(FLOAT32 *auto_corr_input, FLOAT32 *lpc);
VOID impeghe_get_weighted_lpc(FLOAT32 *lpc, FLOAT32 *weighted_lpc);
VOID impeghe_lsp_2_lsf_conversion(FLOAT32 *lsp, FLOAT32 *lsf);
VOID impeghe_lsf_2_lsp_conversion(FLOAT32 *lsf, FLOAT32 *lsp);
VOID impeghe_open_loop_search(FLOAT32 *wsp, WORD32 min_pitch_lag, WORD32 max_pitch_lag,
                              WORD32 num_frame, WORD32 *ol_pitch_lag,
                              ia_usac_td_encoder_struct *st);
WORD32 impeghe_get_ol_lag_median(WORD32 prev_ol_lag, WORD32 *prev_ol_lags);
VOID impeghe_closed_loop_search(FLOAT32 *exc, FLOAT32 *xn, FLOAT32 *wsyn_filt_ir,
                                WORD32 search_range_min, WORD32 search_range_max,
                                WORD32 *pit_frac, WORD32 is_first_subfrm,
                                WORD32 min_pitch_lag_res1_2, WORD32 min_pitch_lag_res_1,
                                WORD32 *pitch_lag_out);
VOID impeghe_decim2_fir_filter(FLOAT32 *signal, WORD32 length, FLOAT32 *mem,
                               FLOAT32 *scratch_fir_sig_buf);
FLOAT32 impeghe_calc_sq_gain(FLOAT32 *x, WORD32 num_bits, WORD32 length,
                             FLOAT32 *scratch_sq_gain_en);
VOID impeghe_lpc_coef_gen(FLOAT32 *lsf_old, FLOAT32 *lsf_new, FLOAT32 *a, WORD32 nb_subfr,
                          WORD32 m);
VOID impeghe_interpolation_lsp_params(FLOAT32 *lsp_old, FLOAT32 *lsp_new, FLOAT32 *lp_flt_coff_a,
                                      WORD32 nb_subfr);

VOID impeghe_acelp_tgt_cb_corr1(FLOAT32 *xn, FLOAT32 *y1, FLOAT32 *y2, FLOAT32 *corr_out);

FLOAT32 impeghe_acelp_tgt_cb_corr2(FLOAT32 *xn, FLOAT32 *y1, FLOAT32 *corr_out);

VOID impeghe_acelp_cb_exc(FLOAT32 *corr_input, FLOAT32 *lp_residual, FLOAT32 *ir_wsyn,
                          WORD16 *alg_cb_exc_out, FLOAT32 *filt_cb_exc, WORD32 num_bits_cb,
                          WORD16 speech_mode, WORD32 *acelp_param_out,
                          FLOAT32 *scratch_acelp_ir_buf, UWORD8 *ptr_scratch);

VOID impeghe_acelp_ltpred_cb_exc(FLOAT32 *exc, WORD32 t0, WORD32 t0_frac, WORD32 len_subfrm);

VOID impeghe_acelp_quant_gain(FLOAT32 *code, FLOAT32 *pitch_gain, FLOAT32 *code_gain,
                              FLOAT32 *tgt_cb_corr_data, FLOAT32 mean_energy, WORD32 *qunt_idx);

VOID impeghe_write_bits2buf(WORD32 value, WORD32 no_of_bits, WORD16 *bitstream);

WORD32 impeghe_get_num_params(WORD32 *qn);

VOID impeghe_highpass_50hz_12k8(FLOAT32 *signal, WORD32 lg, FLOAT32 *mem, WORD32 fscale);
VOID impeghe_lpd_encode(ia_usac_data_struct *ptr_usac_data, WORD32 *mod_out,
                        WORD32 const usac_independency_flg,
                        ia_usac_encoder_config_struct *ptr_usac_config, WORD32 len_frame,
                        WORD32 i_ch, WORD32 chn, ia_bit_buf_struct *it_bit_buff, WORD32 ele_id);

VOID impeghe_init_td_data(ia_usac_td_encoder_struct *st, WORD32 len_frame, WORD32 full_band_lpd);

VOID impeghe_config_acelp_core_mode(ia_usac_td_encoder_struct *st, WORD32 sampling_rate,
                                    WORD32 bitrate);

VOID impeghe_encode_fac_params(WORD32 *mod, WORD32 *n_param_tcx,
                               ia_usac_data_struct *ptr_usac_data,
                               ia_usac_encoder_config_struct *ptr_usac_config,
                               WORD32 const usac_independency_flag,
                               ia_bit_buf_struct *it_bit_buff, WORD32 ch_idx, WORD32 ele_id);

VOID impeghe_acelp_encode(FLOAT32 *lp_filt_coeff, FLOAT32 *quant_lp_filt_coeff,
                          FLOAT32 *speech_in, FLOAT32 *wsig_in, FLOAT32 *synth_out,
                          FLOAT32 *wsynth_out, WORD16 acelp_core_mode,
                          ia_usac_lpd_state_struct *lpd_state, WORD32 len_subfrm,
                          FLOAT32 norm_corr, FLOAT32 norm_corr2, WORD32 ol_pitch_lag1,
                          WORD32 ol_pitch_lag2, WORD32 pit_adj, WORD32 *acelp_params,
                          impeghe_scratch_mem *pstr_scratch);

VOID impeghe_tcx_fac_encode(ia_usac_data_struct *ptr_usac_data,
                            ia_usac_encoder_config_struct *ptr_usac_config, FLOAT32 *lpc_coeffs,
                            FLOAT32 *lpc_coeffs_quant, FLOAT32 *speech, WORD32 frame_len,
                            WORD32 num_bits_per_supfrm, ia_usac_lpd_state_struct *lpd_state,
                            WORD32 *params, WORD32 *n_param, WORD32 *ptr_sfb_offset,
                            WORD32 usac_independency_flag, WORD32 num_sfb, WORD32 ch_idx,
                            WORD32 chn, WORD32 k_idx, WORD32 ele_id);

VOID impeghe_fac_apply(FLOAT32 *orig, WORD32 len_subfrm, WORD32 fac_len, WORD32 low_pass_line,
                       WORD32 target_br, FLOAT32 *synth, FLOAT32 *ptr_lpc_coeffs,
                       WORD16 *fac_bits_word, WORD32 *num_fac_bits,
                       impeghe_scratch_mem *pstr_scratch);

VOID impeghe_quantize_lpc_avq(FLOAT32 *ptr_lsf, FLOAT32 *ptr_lsfq, WORD32 lpc0,
                              WORD32 *ptr_lpc_idx, WORD32 *nb_indices, WORD32 *nbbits,
                              pUWORD8 ptr_lpd_scratch);

VOID impeghe_quantize_flpd_lpc_avq(FLOAT32 *ptr_lsf, FLOAT32 *ptr_lsfq, WORD32 lpc0,
                                   WORD32 *ptr_lpc_idx, WORD32 *nb_indices, WORD32 *nbbits,
                                   pUWORD8 ptr_lpd_scratch);

VOID impeghe_lsp_2_lsf_conversion(FLOAT32 *lsp, FLOAT32 *lsf);
VOID impeghe_lsp_to_lp_conversion(FLOAT32 *lsp, FLOAT32 *lp_flt_coff_a);

IA_ERRORCODE impeghe_fd_fac(WORD32 *sfb_offsets, WORD32 sfb_active, FLOAT64 *orig_sig_dbl,
                            WORD32 window_sequence, FLOAT64 *synth_time,
                            ia_usac_td_encoder_struct *pstr_acelp, WORD32 last_subfr_was_acelp,
                            WORD32 next_frm_lpd, WORD16 *fac_prm_out, WORD32 *num_fac_bits,
                            impeghe_scratch_mem *pstr_scratch);

FLOAT32 impeghe_cal_segsnr(FLOAT32 *sig1, FLOAT32 *sig2, WORD16 len, WORD16 nseg);

WORD32 impeghe_fd_encode_fac(WORD32 *prm, WORD16 *ptr_bit_buf, WORD32 fac_length);

WORD32 impeghe_ltpf_encode(ia_usac_ltpf_data_struct *ltpf_data, const FLOAT32 *x, WORD32 x_length,
                           WORD32 pitch_ol, FLOAT32 pitch_ol_norm_corr,
                           impeghe_scratch_mem *pstr_scratch);
#endif /* IMPEGHE_ENC_LPD_H */
