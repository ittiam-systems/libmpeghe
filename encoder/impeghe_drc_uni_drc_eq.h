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

#ifndef IMPEGHE_DRC_UNI_DRC_EQ_H
#define IMPEGHE_DRC_UNI_DRC_EQ_H

#define MAX_EQ_CHANNEL_COUNT (8)
#define MAX_EQ_AUDIO_DELAY (1024)
#define MAX_EQ_FIR_FILTER_SIZE (128)
#define MAX_EQ_SUBBAND_COUNT (256)
#define MAX_EQ_INTERMEDIATE_2ND_ORDER_PARAMS_COUNT (32)
#define MAX_EQ_FILTER_SECTION_COUNT (8)
#define MAX_EQ_FILTER_ELEMENT_COUNT (4)
#define MAX_MATCHING_PHASE_FILTER_COUNT (32)

#define EQ_FILTER_DOMAIN_NONE 0
#define EQ_FILTER_DOMAIN_TIME (1)
#define EQ_FILTER_DOMAIN_SUBBAND (2)

#define CONFIG_REAL_POLE (0)
#define CONFIG_COMPLEX_POLE (1)
#define CONFIG_REAL_ZERO_RADIUS_ONE (2)
#define CONFIG_REAL_ZERO (3)
#define CONFIG_GENERIC_ZERO (4)

#define STEP_RATIO_F_LOW (20.0f)
#define STEP_RATIO_F_HIGH (24000.0f)
#define MAX_STEP_RATIO_EQ_NODE_COUNT (33)

#define FILTER_ELEMENT_FORMAT_POLE_ZERO (0)
#define FILTER_ELEMENT_FORMAT_FIR (1)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define STEP_RATIO_COMPUTED (0.0739601776f)

typedef struct
{
  WORD32 delay;
  FLOAT32 state[MAX_EQ_CHANNEL_COUNT][MAX_EQ_AUDIO_DELAY];
} ia_drc_audio_delay_struct;

typedef struct
{
  FLOAT32 radius;
  FLOAT32 coeff[2];
} ia_drc_second_order_filter_params_struct;

typedef struct
{
  WORD32 coeff_count;
  FLOAT32 coeff[MAX_EQ_FIR_FILTER_SIZE];
  FLOAT32 state[MAX_EQ_CHANNEL_COUNT][MAX_EQ_FIR_FILTER_SIZE];
} ia_drc_fir_filter_struct;

typedef struct
{
  WORD32 eq_frame_size_subband;
  WORD32 coeff_count;
  FLOAT32 subband_coeff[MAX_EQ_SUBBAND_COUNT];
} ia_drc_subband_filter_struct;

typedef struct
{
  WORD32 filter_format;
  WORD32 filter_param_count_for_zeros;
  ia_drc_second_order_filter_params_struct
      str_second_order_filter_params_for_zeros[MAX_EQ_INTERMEDIATE_2ND_ORDER_PARAMS_COUNT];
  WORD32 filter_param_count_for_poles;
  ia_drc_second_order_filter_params_struct
      str_second_order_filter_params_for_poles[MAX_EQ_INTERMEDIATE_2ND_ORDER_PARAMS_COUNT];
  ia_drc_fir_filter_struct str_fir_filter;
} ia_drc_intermediate_filter_params_struct;

typedef struct
{
  FLOAT32 state_in_1;
  FLOAT32 state_in_2;
  FLOAT32 state_out_1;
  FLOAT32 state_out_2;
} ia_drc_filter_section_state_struct;

typedef struct
{
  FLOAT32 var_a1;
  FLOAT32 var_a2;
  FLOAT32 var_b1;
  FLOAT32 var_b2;
  ia_drc_filter_section_state_struct str_filter_section_state[MAX_EQ_CHANNEL_COUNT];
} ia_drc_filter_section_struct;

typedef struct
{
  WORD32 member_count;
  WORD32 member_index[EQ_MAX_CHANNEL_GROUP_COUNT];
} ia_drc_cascade_alignment_group_struct;

typedef struct
{
  WORD32 is_valid;
  WORD32 matches_filter_count;
  WORD32 matches_filter[MAX_EQ_FILTER_SECTION_COUNT];
  FLOAT32 gain;
  WORD32 section_count;
  ia_drc_filter_section_struct str_filter_section[MAX_EQ_FILTER_SECTION_COUNT];
  ia_drc_audio_delay_struct str_audio_delay;
} ia_drc_phase_alignment_filter_struct;

typedef ia_drc_phase_alignment_filter_struct ia_drc_matching_phase_filter_struct;

typedef struct
{
  WORD32 matches_cascade_index;
  WORD32 allpass_count;
  ia_drc_matching_phase_filter_struct str_matching_phase_filter[MAX_MATCHING_PHASE_FILTER_COUNT];
} ia_drc_allpass_chain_struct;

typedef struct
{
  WORD32 section_count;
  ia_drc_filter_section_struct str_filter_section[MAX_EQ_FILTER_SECTION_COUNT];
  WORD32 fir_coeffs_present;
  ia_drc_fir_filter_struct str_fir_filter;
  ia_drc_audio_delay_struct str_audio_delay;
} ia_drc_pole_zero_filter_struct;

typedef struct
{
  FLOAT32 element_gain_linear;
  WORD32 format;
  ia_drc_pole_zero_filter_struct str_pole_zero_filter;
  ia_drc_fir_filter_struct str_fir_filter;
  WORD32 phase_alignment_filter_count;
  ia_drc_phase_alignment_filter_struct str_phase_alignment_filter[MAX_EQ_FILTER_ELEMENT_COUNT];
} ia_drc_eq_filter_element_struct;

typedef struct
{
  WORD32 element_count;
  ia_drc_eq_filter_element_struct str_eq_filter_element[MAX_EQ_FILTER_ELEMENT_COUNT];
  ia_drc_matching_phase_filter_struct str_matching_phase_filter_element_0;
} ia_drc_eq_filter_block_struct;

typedef struct
{
  FLOAT32 cascade_gain_linear;
  WORD32 block_count;
  ia_drc_eq_filter_block_struct str_eq_filter_block[EQ_FILTER_BLOCK_COUNT_MAX];
  WORD32 phase_alignment_filter_count;
  ia_drc_phase_alignment_filter_struct
      str_phase_alignment_filter[EQ_FILTER_BLOCK_COUNT_MAX * EQ_FILTER_BLOCK_COUNT_MAX];
} ia_drc_filter_cascade_t_domain_struct;

typedef struct
{
  WORD32 domain;
  WORD32 audio_channel_count;
  WORD32 eq_channel_group_count;
  WORD32 eq_channel_group_for_channel[MAX_EQ_CHANNEL_COUNT];
  ia_drc_filter_cascade_t_domain_struct str_filter_cascade_t_domain[EQ_MAX_CHANNEL_GROUP_COUNT];
  ia_drc_subband_filter_struct str_subband_filter[EQ_MAX_CHANNEL_GROUP_COUNT];
} ia_drc_eq_set_struct;

IA_ERRORCODE impeghe_drc_derive_eq_set(ia_drc_eq_coefficients_struct *pstr_eq_coefficients,
                                       ia_drc_eq_instructions_struct *pstr_eq_instructions,
                                       const FLOAT32 audio_sample_rate,
                                       const WORD32 drc_frame_size,
                                       const WORD32 sub_band_domain_mode,
                                       ia_drc_eq_set_struct *pstr_eq_set, VOID *ptr_scratch,
                                       WORD32 *scratch_used);

IA_ERRORCODE impeghe_drc_get_eq_complexity(ia_drc_eq_set_struct *pstr_eq_set,
                                           WORD32 *eq_complexity_level);

#endif /* IMPEGHE_DRC_UNI_DRC_EQ_H */
