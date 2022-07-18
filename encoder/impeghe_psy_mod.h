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

#ifndef IMPEGHE_PSY_MOD_H
#define IMPEGHE_PSY_MOD_H

#define MAX_NUM_GROUPED_SFB 60
#define MAX_BARC_VALUE (24.0f)
#define MASK_LOW_FAC (3.0f)
#define MASK_HIGH_FAC (1.5f)
#define MASK_LOW_SP_ENERGY_L (3.0f)
#define MASK_HIGH_SP_ENERGY_L (2.0f)
#define MASK_HIGH_SP_ENERGY_L_LBR (1.5f)
#define MASK_LOW_SP_ENERGY_S (2.0f)
#define MASK_HIGH_SP_ENERGY_S (1.5f)
#define C_RATIO (0.001258925f)

#define TRUE 1
#define FALSE 0

typedef struct
{
  WORD32 sfb_count;
  WORD32 sfb_active;
  WORD32 sfb_offset[MAX_NUM_SFB_LONG + 1];
  FLOAT32 sfb_thr_quiet[MAX_NUM_SFB_LONG];
  FLOAT32 max_allowed_inc_fac;
  FLOAT32 min_remaining_thr_fac;
  WORD32 low_pass_line;
  FLOAT32 clip_energy;
  FLOAT32 ratio;
  FLOAT32 sfb_mask_low_fac[MAX_NUM_SFB_LONG];
  FLOAT32 sfb_mask_high_fac[MAX_NUM_SFB_LONG];
  FLOAT32 sfb_mask_low_fac_spr_ener[MAX_NUM_SFB_LONG];
  FLOAT32 sfb_mask_high_fac_spr_ener[MAX_NUM_SFB_LONG];
  FLOAT32 sfb_min_snr[MAX_NUM_SFB_LONG];
} ia_psy_mod_long_config_struct;

typedef struct
{
  WORD32 sfb_count;
  WORD32 sfb_active;
  WORD32 sfb_offset[MAX_NUM_SFB_SHORT + 1];
  FLOAT32 sfb_thr_quiet[MAX_NUM_SFB_SHORT];
  FLOAT32 max_allowed_inc_fac;
  FLOAT32 min_remaining_thr_fac;
  WORD32 low_pass_line;
  FLOAT32 clip_energy;
  FLOAT32 ratio;
  FLOAT32 sfb_mask_low_fac[MAX_NUM_SFB_SHORT];
  FLOAT32 sfb_mask_high_fac[MAX_NUM_SFB_SHORT];
  FLOAT32 sfb_mask_low_fac_spr_ener[MAX_NUM_SFB_SHORT];
  FLOAT32 sfb_mask_high_fac_spr_ener[MAX_NUM_SFB_SHORT];
  FLOAT32 sfb_min_snr[MAX_NUM_SFB_SHORT];
} ia_psy_mod_short_config_struct;

typedef struct
{
  WORD32 sfb_count;
  WORD32 max_sfb_per_grp;
  WORD32 sfb_per_group;
  WORD32 window_sequence;
  WORD32 window_shape;
  WORD32 sfb_offsets[100];
  FLOAT32 *ptr_sfb_energy;
  FLOAT32 *ptr_sfb_spread_energy;
  FLOAT32 *ptr_sfb_thr;
  FLOAT64 *ptr_spec_coeffs;
  FLOAT32 pe;
  FLOAT32 sfb_min_snr[MAX_NUM_GROUPED_SFB];
  WORD32 ms_used[MAX_NUM_GROUPED_SFB];
} ia_psy_mod_out_data_struct;

typedef struct
{
  WORD32 window_sequence;
  FLOAT32 sfb_thr_nm1[MAX_NUM_GROUPED_SFB];
  FLOAT32 *ptr_sfb_thr_long;
  FLOAT32 sfb_thr_short[MAX_SHORT_WINDOWS][MAX_NUM_SFB_SHORT];
  FLOAT32 *ptr_sfb_energy_long;
  FLOAT32 ptr_sfb_energy_long_ms[MAX_NUM_GROUPED_SFB];
  FLOAT32 ptr_sfb_energy_short_ms[MAX_SHORT_WINDOWS][MAX_NUM_SFB_SHORT];
  FLOAT32 sfb_energy_short[MAX_SHORT_WINDOWS][MAX_NUM_SFB_SHORT];
  FLOAT32 *ptr_sfb_spreaded_energy_long;
  FLOAT32 sfb_spreaded_energy_short[MAX_SHORT_WINDOWS][MAX_NUM_SFB_SHORT];
} ia_psy_mod_data_struct;

typedef struct ia_psy_mod_struct
{
  ia_psy_mod_long_config_struct str_psy_long_config[MAX_TIME_CHANNELS];
  ia_psy_mod_short_config_struct str_psy_short_config[MAX_TIME_CHANNELS];
  ia_psy_mod_data_struct str_psy_data[MAX_TIME_CHANNELS];
  ia_psy_mod_out_data_struct str_psy_out_data[MAX_TIME_CHANNELS];
} ia_psy_mod_struct;

typedef struct
{
  WORD32 sample_rate;
  const UWORD8 params_long[MAX_NUM_SFB_LONG];
  const UWORD8 params_short[MAX_NUM_SFB_SHORT];
} ia_sfb_info_tables;

typedef struct ia_sfb_params_struct
{
  WORD32 num_sfb[MAX_TIME_CHANNELS];
  WORD32 max_sfb[MAX_TIME_CHANNELS];
  WORD32 max_sfb_ste;
  WORD32 sfb_width_table[MAX_TIME_CHANNELS][MAX_NUM_SFB_LONG];
  WORD32 grouped_sfb_offset[MAX_TIME_CHANNELS][MAX_SF_BANDS + 1];
  WORD32 sfb_offset[MAX_TIME_CHANNELS][MAX_SF_BANDS + 1];
  WORD32 num_window_groups[MAX_TIME_CHANNELS];
  WORD32 window_group_length[MAX_TIME_CHANNELS][MAX_SHORT_WINDOWS];
  WORD32 window_shape[MAX_TIME_CHANNELS];
  WORD32 window_sequence[MAX_TIME_CHANNELS];
  WORD32 common_win[MAX_TIME_CHANNELS];

} ia_sfb_params_struct;

VOID impeghe_psy_mod_init(ia_psy_mod_struct *pstr_psy_mod, WORD32 sample_rate, WORD32 bit_rate,
                          WORD32 band_width, WORD32 num_channels, WORD32 ch, WORD32 ele_id);
VOID impeghe_psy_mod_sb(ia_psy_mod_struct *pstr_psy_mod, ia_igf_data_struct *pstr_igf_data,
                        ia_sfb_params_struct *pstr_sfb_prms, FLOAT64 *ptr_spec_in,
                        ia_tns_info *ptr_tns_info[MAX_TIME_CHANNELS], WORD32 tns_select,
                        WORD32 i_ch, WORD32 chn, WORD32 igf_after_tns_synth,
                        ia_igf_config_struct *pstr_igf_config, WORD32 channel_type,
                        FLOAT64 *scratch_tns_filter, WORD32 elem_idx, FLOAT64 *ptr_igf_scratch,
                        FLOAT64 *ptr_tns_scratch);
VOID impeghe_psy_mod_lb(ia_psy_mod_struct *pstr_psy_mod, ia_igf_data_struct *pstr_igf_data,
                        ia_sfb_params_struct *pstr_sfb_prms, FLOAT64 *ptr_spec_in,
                        ia_tns_info *ptr_tns_info[MAX_TIME_CHANNELS], WORD32 tns_select,
                        WORD32 i_ch, WORD32 chn, WORD32 igf_after_tns_synth,
                        ia_igf_config_struct *pstr_igf_config, WORD32 channel_type,
                        FLOAT64 *scratch_tns_filter, WORD32 elem_idx, FLOAT64 *ptr_igf_scratch,
                        FLOAT64 *ptr_tns_scratch);

#endif /* IMPEGHE_PSY_MOD_H */
