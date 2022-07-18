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

#ifndef IMPEGHE_DRC_GAIN_ENC_H
#define IMPEGHE_DRC_GAIN_ENC_H

#include "impeghe_drc_tables.h"

#define N_UNIDRC_GAIN_MAX (MAX_NODE_COUNT)

#ifndef M_LN10
#define M_LN10 (2.30258509299404568402) /* log_e 10 */
#endif
#ifndef M_LOG2_10
#define M_LOG2_10 (3.32192809488736234787) /* log_2 10 */
#endif

#ifndef M_LOG10_E
#define M_LOG10_E (0.4342944819) /* log_10 e */
#endif

#define M_LN10_DIV_20 (0.115129254649702284201) /* (log_e 10  / 20) */

#define EXP10(x) (exp2(M_LOG2_10 * x))

#define MAX_NUM_CHANNELS (32)
#define STFT256_HOP_SIZE (256)
#define IMPEGHE_DRC_SPECTRAL_FLOOR (0.02818383f)

#define IMPEGHE_DRC_MAX_NSEQ (40)
#define IMPEGHE_DRC_MAX_FRAMESIZE (4096)
#define IMPEGHE_DRC_COMPAND_MAX_NB_POINTS (MAX_NUM_CHANNELS << 3)
#define IMPEGHE_DRC_COMPAND_MAX_NUM_SEGMENTS ((IMPEGHE_DRC_COMPAND_MAX_NB_POINTS + 4) << 1)

#define MAX_TIME_DEVIATION_FACTOR (0.25f)
#define SLOPE_CHANGE_THR (3.0f)
#define SLOPE_QUANT_THR (8.0f)

#define GAIN_QUANT_STEP_SIZE (0.125f)
#define GAIN_QUANT_STEP_SIZE_INV (8.0f)

#define MAX_DRC_GAIN_DELTA_BEFORE_QUANT (1.0f + 0.5f * GAIN_QUANT_STEP_SIZE)

#define SCALE_APPROXIMATE_DB                                                                     \
  (0.99657842f) /* factor for converting dB to approximate dB: log2(10)*6/20 */

typedef struct ia_drc_compand_chan_param_struct
{
  FLOAT64 attack;
  FLOAT64 decay;
  FLOAT64 volume;
} ia_drc_compand_chan_param_struct;

typedef struct ia_drc_compand_segment_struct
{
  FLOAT64 x, y;
  FLOAT64 a, b;
} ia_drc_compand_segment_struct;

typedef struct ia_drc_compand_struct
{
  UWORD8 is_valid;
  UWORD32 ch_idx;
  UWORD32 nb_points;
  UWORD32 nb_segments;
  ia_drc_compand_segment_struct str_segment[IMPEGHE_DRC_COMPAND_MAX_NUM_SEGMENTS];
  ia_drc_compand_chan_param_struct str_channel_param;
  FLOAT64 in_min_lin;
  FLOAT64 out_min_lin;
  FLOAT64 width_db;
  FLOAT64 gain_db;
  FLOAT64 initial_volume;
} ia_drc_compand_struct;

typedef struct ia_drc_stft_gain_calc_struct
{
  UWORD8 is_valid;
  UWORD32 ch_idx;
  FLOAT32 theshold;
  FLOAT32 ratio;
  FLOAT32 attack_ms;
  FLOAT32 release_ms;
  FLOAT32 alpha_a;
  FLOAT32 alpha_r;
  FLOAT32 yl_z1[STFT256_HOP_SIZE + 1];

  UWORD32 nb_points;
  UWORD32 nb_segments;
  ia_drc_compand_segment_struct str_segment[IMPEGHE_DRC_COMPAND_MAX_NUM_SEGMENTS];
  ia_drc_compand_chan_param_struct str_channel_param;
  FLOAT32 in_min_db;
  FLOAT32 out_min_db;
  FLOAT32 width_db;
  FLOAT32 gain_db;
  FLOAT32 initial_volume;
} ia_drc_stft_gain_calc_struct;

typedef struct
{
  WORD32 n_gain_values;

  FLOAT32 drc_gain_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 gain_code[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 gain_code_length[IMPEGHE_DRC_MAX_FRAMESIZE];
  FLOAT32 slope_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 slope_code_index[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 ts_gain_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 time_delta_quant[IMPEGHE_DRC_MAX_FRAMESIZE];

  FLOAT32 drc_gain[IMPEGHE_DRC_MAX_FRAMESIZE];
  FLOAT32 slope[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 ts_gain[IMPEGHE_DRC_MAX_FRAMESIZE];

  FLOAT32 gain_prev_node;
  FLOAT32 drc_gain_quant_prev;

} ia_drc_group_struct;

typedef struct
{
  WORD32 n_gain_values;

  FLOAT32 drc_gain_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 gain_code[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 gain_code_length[IMPEGHE_DRC_MAX_FRAMESIZE];
  FLOAT32 slope_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 slope_code_index[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 ts_gain_quant[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 time_delta_quant[IMPEGHE_DRC_MAX_FRAMESIZE];

  WORD32 time_delta_code[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 time_delta_code_index[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 time_delta_code_size[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 slope_code[IMPEGHE_DRC_MAX_FRAMESIZE];
  WORD32 slope_code_size[IMPEGHE_DRC_MAX_FRAMESIZE];

  FLOAT32 drc_gain_quant_prev;
  FLOAT32 drc_gain_quant_next;
  WORD32 time_quant_next;
  WORD32 time_quant_prev;
  WORD32 slope_code_index_next;
  WORD32 slope_code_index_prev;

  WORD32 coding_mode;
} ia_drc_group_for_output_struct;

typedef struct
{
  ia_drc_gain_set_params_struct str_gain_set_params;
  ia_drc_group_struct str_drc_group;
  ia_drc_group_for_output_struct str_drc_group_for_output;
} ia_drc_gain_seq_buf_struct;

typedef struct
{
  WORD32 n_sequences;
  WORD32 delta_tmin;
  WORD32 delta_tmin_default;
  WORD32 drc_frame_size;
  WORD32 sample_rate;
  WORD32 delay_mode;
  WORD32 domain;
  WORD32 base_ch_count;
  ia_drc_uni_drc_config_struct str_uni_drc_config;
  ia_drc_loudness_info_set_struct str_loudness_info_set;
  FLOAT32 drc_gain_per_sample_with_prev_frame[IMPEGHE_DRC_MAX_NSEQ]
                                             [3 * IMPEGHE_DRC_MAX_FRAMESIZE];
  ia_drc_gain_seq_buf_struct str_drc_gain_seq_buf[IMPEGHE_DRC_MAX_NSEQ];
  ia_drc_delta_time_code_table_entry_struct
      str_delta_time_code_table[2 * IMPEGHE_DRC_MAX_FRAMESIZE + 1];
  WORD32 delta_time_quant_table[IMPEGHE_DRC_MAX_FRAMESIZE];

  ia_drc_eq_set_struct str_eq_set;
  ia_drc_filter_banks_struct str_filter_banks;
  ia_drc_compand_struct str_drc_compand[MAX_DRC_COEFF_COUNT][GAIN_SET_COUNT_MAX];
  ia_drc_stft_gain_calc_struct str_drc_stft_gain_handle[MAX_DRC_COEFF_COUNT][GAIN_SET_COUNT_MAX]
                                                       [MAX_BAND_COUNT];
  FLOAT32 stft_tmp_in_buf_time[MAX_NUM_CHANNELS][STFT256_HOP_SIZE];
  FLOAT32 complex_fft_ptr[MAX_NUM_CHANNELS][IMPEGHE_DRC_MAX_FRAMESIZE << 1];
} ia_drc_gain_enc_struct;

VOID impeghe_drc_quantize_and_encode_drc_gain(
    ia_drc_gain_enc_struct *pstr_gain_enc, const FLOAT32 *ptr_drc_gain_per_sample,
    FLOAT32 *ptr_drc_gain_per_sample_with_prev_frame,
    ia_drc_delta_time_code_table_entry_struct *pstr_delta_time_code_table,
    ia_drc_gain_seq_buf_struct *pstr_drc_gain_seq_buf, VOID *pstr_scratch);

IA_ERRORCODE impeghe_td_drc_gain_calc_init(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                           WORD32 drc_coefficients_uni_drc_idx,
                                           WORD32 gain_set_idx);

IA_ERRORCODE impeghe_stft_drc_gain_calc_init(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                             WORD32 drc_coefficients_uni_drc_idx,
                                             WORD32 gain_set_idx, WORD32 band_idx);

VOID impeghe_td_drc_gain_calc_process(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                      WORD32 drc_coefficients_uni_drc_idx, WORD32 gain_set_idx,
                                      WORD32 num_samples, FLOAT32 *in_buff, FLOAT32 *out_buff);

VOID impeghe_stft_drc_gain_calc_process(ia_drc_gain_enc_struct *pstr_drc_gain_enc,
                                        WORD32 drc_coefficients_uni_drc_idx, WORD32 gain_set_idx,
                                        WORD32 band_idx, WORD32 start_sub_band_index,
                                        WORD32 stop_sub_band_index, UWORD32 num_frames,
                                        FLOAT32 *in_buff, FLOAT32 *gain_values);

VOID impeghe_stft_drc_convert_to_fd(ia_drc_gain_enc_struct *pstr_drc_gain_enc, FLOAT32 *ptr_input,
                                    UWORD32 ch_idx, UWORD32 frame_size, FLOAT32 *ptr_output,
                                    VOID *pstr_scratch);
#endif /*IMPEGHE_ERC_GAIN_ENC_H*/
