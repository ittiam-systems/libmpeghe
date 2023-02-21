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

#ifndef IMPEGHE_DMX_MATRIX_COMMON_H
#define IMPEGHE_DMX_MATRIX_COMMON_H

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define IMPEGHE_DMX_MAX_NUM_EQ_PARAM (32)
#define IMPEGHE_MAX_NUM_PEAK_FILTER (32)

#define SP_PAIR_CENTER (0)
#define SP_PAIR_SYMMETRIC (1)
#define SP_PAIR_SINGLE (2)
#define SP_PAIR_NONE (3)

#define IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT (32)
#define IMPEGHE_DMX_MATRIX_GAIN_ZERO (-256.0f)
#define IMPEGHE_DMX_MATRIX_CODER_STATE_COUNT_MAX (512)
#define IMPEGHE_DMX_MATRIX_GAIN_TABLE_SIZE_MAX ((22 - (-46)) * (1 << 2) + 2)

typedef struct ia_dmx_speaker_information_str
{
  WORD16 elevation;
  WORD16 azimuth;
  WORD16 is_lfe;

  WORD16 original_position;
  WORD16 is_already_used;
  struct ia_dmx_speaker_information_str *pstr_symmetric_pair;
  WORD32 pair_type;
} ia_dmx_speaker_information_struct;

typedef struct
{
  WORD32 min_gain;
  WORD32 max_gain;
  WORD32 precision_level;
  WORD32 raw_coding_nonzeros;
  WORD32 gain_l_g_r_param;
  FLOAT32 history[IMPEGHE_DMX_MATRIX_CODER_STATE_COUNT_MAX];
  WORD32 history_count;
  FLOAT32 gain_table[IMPEGHE_DMX_MATRIX_GAIN_TABLE_SIZE_MAX];
  WORD32 gain_table_size;
} ia_dmx_mtx_coder_state_struct;

typedef struct
{
  FLOAT32 peak_freq_hz;
  FLOAT32 peak_q_factor;
  FLOAT32 peak_gain_db;
} ia_dmx_pk_filter_params_struct;

typedef struct
{
  WORD32 num_pk_filter;
  FLOAT32 global_gain_db;
  ia_dmx_pk_filter_params_struct pk_filter_params[IMPEGHE_MAX_NUM_PEAK_FILTER];
} ia_dmx_eq_params_struct;

typedef struct
{
  WORD32 num_eq;
  ia_dmx_eq_params_struct eq_params[IMPEGHE_DMX_MAX_NUM_EQ_PARAM];
  WORD32 eq_map[IMPEGHE_DMX_MAX_NUM_EQ_PARAM];
} ia_dmx_eq_config_struct;

typedef struct
{
  /*scratch*/
  ia_dmx_speaker_information_struct input_config[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  ia_dmx_speaker_information_struct output_config[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  ia_dmx_channel_geometry_struct input_geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  ia_dmx_channel_geometry_struct output_geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  WORD8 compact_downmix_matrix[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT]
                              [IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  WORD8 compact_template_buffer[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT *
                                IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
} ia_dmx_sratch;

typedef struct
{
  WORD32 output_config_index;
  WORD32 input_config_index;
  FLOAT32 flat_downmix_matrix[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT *
                              IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  ia_dmx_eq_config_struct pstr_eq_config;

} ia_dmx_matrix_enc_cfg_struct;

IA_ERRORCODE
impeghe_dmx_coder_state_generate_gain_table(ia_dmx_mtx_coder_state_struct *coder_state);

VOID impeghe_dmx_convert_to_compact_config(
    WORD32 input_count, ia_dmx_speaker_information_struct *pstr_input_config,
    WORD32 *ptr_compact_input_count,
    ia_dmx_speaker_information_struct *pstr_compact_input_config[]);

pWORD8 impeghe_dmx_find_compact_template(WORD32 input_index, WORD32 output_index,
                                         WORD32 *err_code);

#endif /* IMPEGHE_DMX_MATRIX_COMMON_H */
