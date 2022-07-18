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

#ifndef IMPEGHE_WRITE_BITSTREAM_H
#define IMPEGHE_WRITE_BITSTREAM_H

WORD32 impeghe_write_ics_info(ia_bit_buf_struct *it_bit_buf, ia_sfb_params_struct *pstr_sfb_prms,
                              WORD32 ch);

WORD32 impeghe_write_mpegh3da_ext_ele(ia_bit_buf_struct *it_bit_buf, WORD32 ext_data_present,
                                      WORD32 ext_data_size,
                                      UWORD8 ext_data[MAX_EXTENSION_PAYLOAD_LEN]);
WORD32 impeghe_write_cpe(ia_sfb_params_struct *pstr_sfb_prms, ia_bit_buf_struct *it_bit_buf,
                         WORD32 *tns_data_present, WORD32 const usac_independency_flg,
                         ia_usac_encoder_config_struct *ptr_usac_config,
                         ia_usac_data_struct *ptr_usac_data, WORD32 ch, WORD32 ele_id);

WORD32 impeghe_write_fd_data(ia_bit_buf_struct *it_bit_buf, ia_sfb_params_struct *pstr_sfb_prms,
                             WORD32 num_fac_bits, WORD32 usac_independency_flg,
                             ia_usac_data_struct *ptr_usac_data,
                             ia_usac_encoder_config_struct *ptr_usac_config, WORD32 ch_idx,
                             WORD32 ele_id, WORD32 idx);

WORD32 impeghe_count_fd_bits(ia_sfb_params_struct *pstr_sfb_prms,
                             ia_usac_data_struct *ptr_usac_data, WORD32 usac_independency_flg,
                             ia_usac_encoder_config_struct *ptr_usac_config, WORD32 ch_idx,
                             WORD32 idx);

WORD32 impeghe_write_fill_ele(ia_bit_buf_struct *it_bit_buf, WORD32 num_bits);

WORD32 impeghe_write_tns_data(ia_bit_buf_struct *it_bit_buf, ia_tns_info *pstr_tns_info,
                              WORD32 window_sequence, WORD32 core_mode);
WORD32 impeghe_write_cplx_pred_data(ia_bit_buf_struct *it_bit_buf, WORD32 num_win_grps,
                                    WORD32 num_sfb, WORD32 complex_coef,
                                    WORD32 pred_coeffs_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    WORD32 pred_coeffs_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2],
                                    WORD32 const usac_independency_flg, WORD32 pred_dir,
                                    WORD32 cplx_pred_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    WORD32 cplx_pred_all, WORD32 *ptr_prev_alpha_coeff_re,
                                    WORD32 *ptr_prev_alpha_coeff_im, WORD32 *delta_code_time);

WORD32 impeghe_write_ms_data(ia_bit_buf_struct *it_bit_buf, WORD32 ms_mask,
                             WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                             WORD32 num_win_grps, WORD32 nr_of_sfb);

WORD32 impeghe_write_scf_data(ia_bit_buf_struct *it_bit_buf, WORD32 max_sfb, WORD32 num_sfb,
                              const WORD32 *scale_factors, WORD32 num_win_grps,
                              WORD32 global_gain, const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2]);

#endif /* IMPEGHE_WRITE_BITSTREAM_H */
