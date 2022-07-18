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

#ifndef IMPEGHE_IGF_ENC_H
#define IMPEGHE_IGF_ENC_H

#include "impeghe_type_def.h"

#define MAX_IGF_TILES (4)
#define IGF_START_SR48_BR32 (8000)
#define IGF_START_SR48_BR16 (7000)
#define IGF_START_SR32_BR32 (6000)
#define IGF_START_SR32_BR16 (5000)
#define IGF_START_SR16_BR32 (4000)
#define IGF_START_SR16_BR16 (3000)

#define IGF_STOP_SR48 (20000)
#define IGF_STOP_SR32 (16000)
#define IGF_STOP_SR16 (8000)

typedef struct ia_igf_data_struct
{
  WORD32 igf_level[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 igf_level_prev[MAX_NUM_SFB_LONG];
  WORD32 igf_prev_d;
  WORD32 igf_src_tile_idx[MAX_IGF_TILES];
  WORD32 igf_curr_tile_width_long[MAX_IGF_TILES];
  WORD32 igf_curr_tile_width_short[MAX_IGF_TILES];
  WORD32 igf_whitening_level[MAX_IGF_TILES];
  WORD32 igf_prev_whitening_level[MAX_IGF_TILES];
  WORD32 igf_n_tiles_long;
  WORD32 igf_n_tiles_short;
  WORD32 prev_num_windows;
  WORD32 lpd_mode;
  WORD32 last_lpd_mode;
  WORD32 code_mode_prev;
  WORD32 igf_arith_t;
  WORD32 igf_all_zero;

} ia_igf_data_struct;

typedef struct ia_igf_config_struct
{
  WORD32 m_igf_start_sfb;
  WORD32 m_igf_stop_sfb;
  WORD32 is_short_block;
  WORD32 igf_use_whitening;
  WORD32 igf_use_enf;
  WORD32 igf_apply_tnf;
  WORD32 igf_use_high_res;
  WORD32 igf_start_freq;
  WORD32 igf_stop_freq;
  WORD32 igf_after_tns_synth;
  WORD32 igf_start_sfb_sb;
  WORD32 igf_start_sfb_lb;
  WORD32 igf_stop_sfb_sb;
  WORD32 igf_stop_sfb_lb;
  WORD32 igf_start_index;
  WORD32 igf_stop_index;
  WORD32 igf_independent_tiling;
  WORD32 igf_active;
  WORD32 igf_ms_mask;
  WORD32 igf_stereo_pred_all;
  WORD32 igf_min_l;
  WORD32 igf_min_s;
  WORD32 igf_min_tcx;
} ia_igf_config_struct;

WORD32 impeghe_write_igf_data(ia_bit_buf_struct *it_bit_buf, ia_igf_data_struct *pstr_igf_data,
                              ia_igf_config_struct *pstr_igf_config,
                              WORD32 usac_independency_flag, WORD32 core_mode);

WORD32 impeghe_write_igf_levels(ia_bit_buf_struct *it_bit_buf, ia_igf_data_struct *pstr_igf_data,
                                ia_igf_config_struct *pstr_igf_config,
                                WORD32 usac_independency_flg, WORD32 core_mode,
                                WORD32 num_window_groups);

VOID impeghe_igf(FLOAT64 *ptr_in_spec, WORD32 *ptr_sfb_offset, ia_igf_data_struct *pstr_igf_data,
                 ia_igf_config_struct *pstr_igf_config, WORD32 group_id, FLOAT64 tns_pred_gain,
                 WORD32 win_group_length, WORD32 tns_data_present, WORD32 igf_after_tns_synth,
                 WORD32 igf_emphasis, WORD32 core_mode, FLOAT64 *ptr_src_energy_scratch);

VOID impeghe_igf_init(ia_igf_config_struct *pstr_igf_config, WORD32 sample_rate,
                      WORD32 *ptr_sfb_offset_long, WORD32 *ptr_sfb_offset_short, WORD32 num_sfb,
                      WORD32 igf_after_tns_synth, WORD32 sfb_active);

VOID impeghe_get_tile_info(WORD32 sfb_bgn, WORD32 sfb_end, WORD32 buh, WORD32 igf_min,
                           WORD32 *ptr_sfb_offset, ia_igf_data_struct *pstr_igf_data,
                           WORD32 is_short_block);

VOID impeghe_get_tile_info_tcx(WORD32 sfb_bgn, WORD32 sfb_end, WORD32 buh, WORD32 igf_min,
                               WORD32 *ptr_sfb_offset, WORD32 *ptr_tile_width,
                               WORD32 *igf_n_tiles);

WORD32 impeghe_write_igf_mask_data(ia_bit_buf_struct *it_bit_buf, WORD32 num_groups,
                                   ia_igf_data_struct *pstr_igf_data,
                                   ia_igf_config_struct *pstr_igf_config,
                                   WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG]);

WORD32 impeghe_write_igf_pred_data(
    ia_bit_buf_struct *it_bit_buf, WORD32 num_groups, ia_igf_config_struct *pstr_igf_config,
    WORD32 usac_independency_flag, const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2],
    WORD32 igf_pred_dir, WORD32 pred_coeffs_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
    WORD32 *igf_delta_code_time, WORD32 cplx_pred_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG]);

VOID impeghe_igf_ms(FLOAT64 *ptr_in_spec, WORD32 *ptr_sfb_offset,
                    ia_igf_data_struct *pstr_igf_data, ia_igf_config_struct *pstr_igf_config,
                    WORD32 sfb_per_group, WORD32 num_sfb);
#endif /*IMPEGHE_IGF_ENC_H*/