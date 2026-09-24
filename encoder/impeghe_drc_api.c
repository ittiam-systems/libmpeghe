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

#include <string.h>
#include "impeghe_error_standards.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_error_codes.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_tables.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_drc_enc.h"
#include "impeghe_drc_tables.h"
#include "impeghe_drc_gain_enc.h"
#define IMPD_DRC_BOUND_CHECK(var, lower_bound, upper_bound) \
  {                                                         \
    var = MIN(var, upper_bound);                            \
    var = MAX(var, lower_bound);                            \
  }

/**
 *  impeghe_drc_validate_config_params
 *
 *  \brief Validates and bounds DRC information
 *
 *  \param [in,out] pstr_inp_config         Pointer to DRC input config
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_drc_validate_config_params(ia_drc_input_config *pstr_inp_config) {
  LOOPIDX i, j, k;
  WORD32 curr_start_subband_idx, next_start_subband_idx;
  ia_drc_uni_drc_config_struct *pstr_uni_drc_config = &pstr_inp_config->str_uni_drc_config;
  ia_drc_loudness_info_set_struct *pstr_enc_loudness_info_set =
    &pstr_inp_config->str_enc_loudness_info_set;

  IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->drc_instructions_uni_drc_count, 0,
    MAX_DRC_INSTRUCTIONS_COUNT);
  for (i = 0; i < pstr_uni_drc_config->drc_instructions_uni_drc_count; i++) {
    IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i].drc_set_id, 0,
      MAX_DRC_SET_ID);
    IMPD_DRC_BOUND_CHECK(
      pstr_uni_drc_config->str_drc_instructions_uni_drc[i].additional_downmix_id_count, 0,
      ADDITIONAL_DOWNMIX_ID_COUNT_MAX);
    IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i].drc_location, 0,
      MAX_DRC_LOCATION);
    IMPD_DRC_BOUND_CHECK(
      pstr_uni_drc_config->str_drc_instructions_uni_drc[i].drc_set_target_loudness_value_upper,
      MIN_DRC_TARGET_LOUDNESS, 0);
    IMPD_DRC_BOUND_CHECK(
      pstr_uni_drc_config->str_drc_instructions_uni_drc[i].drc_set_target_loudness_value_lower,
      MIN_DRC_TARGET_LOUDNESS, 0);
    for (j = 0; j < MAX_CHANNEL_COUNT; j++) {
      IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i].gain_set_index[j],
        0, GAIN_SET_COUNT_MAX - 1);
    }
    IMPD_DRC_BOUND_CHECK(
      pstr_uni_drc_config->str_drc_instructions_uni_drc[i].num_drc_channel_groups, 0,
      MAX_CHANNEL_GROUP_COUNT);
    for (j = 0; j < pstr_uni_drc_config->str_drc_instructions_uni_drc[i].num_drc_channel_groups;
      j++) {
      IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i]
        .str_gain_modifiers[j]
        .attenuation_scaling[0],
        0, MAX_ATTENUATION_SCALING);
      IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i]
        .str_gain_modifiers[j]
        .amplification_scaling[0],
        0, MAX_AMPLIFICATION_SCALING);
      IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i]
        .str_gain_modifiers[j]
        .gain_offset[0],
        MIN_DRC_GAIN_OFFSET, MAX_DRC_GAIN_OFFSET);
    }
    IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_instructions_uni_drc[i].limiter_peak_target,
      MIN_LIMITER_PEAK_TARGET, 0.0f);
  }
  IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->drc_coefficients_uni_drc_count, 0,
    MAX_DRC_COEFF_COUNT);
  for (i = 0; i < pstr_uni_drc_config->drc_coefficients_uni_drc_count; i++) {
    IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].drc_location, 0,
      MAX_DRC_LOCATION);
    IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].gain_set_count, 0,
      MAX_CHANNEL_GROUP_COUNT);
    for (j = 0; j < pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].gain_set_count; j++) {
      IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
        .str_gain_set_params[j]
        .gain_coding_profile,
        0, MAX_GAIN_CODING_PROFILE);
      IMPD_DRC_BOUND_CHECK(
        pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].band_count,
        0, MAX_BAND_COUNT);
      for (k = 0;
        k <
        pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].band_count;
        k++) {
        IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
          .str_gain_set_params[j]
          .gain_params[k]
          .nb_points,
          0, MAX_GAIN_POINTS);
        IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
          .str_gain_set_params[j]
          .gain_params[k]
          .drc_characteristic,
          0, MAX_DRC_CHARACTERISTIC_VALUE);
        IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
          .str_gain_set_params[j]
          .gain_params[k]
          .crossover_freq_index,
          0, MAX_CROSSOVER_FREQ_INDEX);
        IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
          .str_gain_set_params[j]
          .gain_params[k]
          .start_sub_band_index,
          0, STFT256_HOP_SIZE - 1);
        IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].gain_params[k].width, -MAX_FLT_VAL_DB, MAX_FLT_VAL_DB);
        for (WORD32 m = 0; m < pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].gain_params[k].nb_points; m++) {
          IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].gain_params[k].gain_points[m].x,
            -MAX_FLT_VAL_DB, MAX_FLT_VAL_DB);
          IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
            .str_gain_set_params[j].gain_params[k].gain_points[m].y,
            -MAX_FLT_VAL_DB, MAX_FLT_VAL_DB);
        }
      }
      for (k = 0; k <
        pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].str_gain_set_params[j].band_count
        - 1; k++) {
        curr_start_subband_idx = pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].
          str_gain_set_params[j].gain_params[k].start_sub_band_index;
        next_start_subband_idx = pstr_uni_drc_config->str_drc_coefficients_uni_drc[i].
          str_gain_set_params[j].gain_params[k + 1].start_sub_band_index;
        /* It is assumed that the start index of a subband is greater than the start index of its previous subbands for a multiband */
        if (next_start_subband_idx <= curr_start_subband_idx) {
          return IMPEGHE_NONFATAL_USAC_INVALID_SUBBAND_INDEX;
        }
      }
    }
  }
  IMPD_DRC_BOUND_CHECK(pstr_uni_drc_config->downmix_instructions_count, 0,
    MAX_DOWNMIX_INSTRUCTION_COUNT);

  IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->loudness_info_count, 0,
    MAX_LOUDNESS_INFO_COUNT);
  for (i = 0; i < pstr_enc_loudness_info_set->loudness_info_count; i++) {
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info[i].sample_peak_level,
      MIN_SAMPLE_PEAK_LEVEL, MAX_SAMPLE_PEAK_LEVEL);
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info[i].true_peak_level,
      MIN_TRUE_PEAK_LEVEL, MAX_TRUE_PEAK_LEVEL);
    IMPD_DRC_BOUND_CHECK(
      pstr_enc_loudness_info_set->str_loudness_info[i].true_peak_level_measurement_system, 0,
      MAX_MEASUREMENT_SYSTEM_TYPE);
    IMPD_DRC_BOUND_CHECK(
      pstr_enc_loudness_info_set->str_loudness_info[i].true_peak_level_reliability, 0,
      MAX_RELIABILITY_TYPE);
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info[i].measurement_count, 0,
      MAX_MEASUREMENT_COUNT);
    for (j = 0; j < pstr_enc_loudness_info_set->str_loudness_info[i].measurement_count; j++) {
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info[i]
        .str_loudness_measure[j]
        .method_definition,
        0, MAX_METHOD_DEFINITION_TYPE);
      IMPD_DRC_BOUND_CHECK(
        pstr_enc_loudness_info_set->str_loudness_info[i].str_loudness_measure[j].method_value,
        MIN_METHOD_VALUE, MAX_METHOD_VALUE);
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info[i]
        .str_loudness_measure[j]
        .measurement_system,
        0, MAX_MEASUREMENT_SYSTEM_TYPE);
      IMPD_DRC_BOUND_CHECK(
        pstr_enc_loudness_info_set->str_loudness_info[i].str_loudness_measure[j].reliability, 0,
        MAX_RELIABILITY_TYPE);
    }
  }
  IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->loudness_info_album_count, 0,
    MAX_LOUDNESS_INFO_COUNT);
  for (i = 0; i < pstr_enc_loudness_info_set->loudness_info_album_count; i++) {
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i].sample_peak_level,
      MIN_SAMPLE_PEAK_LEVEL, MAX_SAMPLE_PEAK_LEVEL);
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i].true_peak_level,
      MIN_TRUE_PEAK_LEVEL, MAX_TRUE_PEAK_LEVEL);
    IMPD_DRC_BOUND_CHECK(
      pstr_enc_loudness_info_set->str_loudness_info_album[i].true_peak_level_measurement_system,
      0, MAX_MEASUREMENT_SYSTEM_TYPE);
    IMPD_DRC_BOUND_CHECK(
      pstr_enc_loudness_info_set->str_loudness_info_album[i].true_peak_level_reliability, 0,
      MAX_RELIABILITY_TYPE);
    IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i].measurement_count,
      0, MAX_MEASUREMENT_COUNT);
    for (j = 0; j < pstr_enc_loudness_info_set->str_loudness_info_album[i].measurement_count;
      j++) {
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i]
        .str_loudness_measure[j]
        .method_definition,
        0, MAX_METHOD_DEFINITION_TYPE);
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i]
        .str_loudness_measure[j]
        .method_value,
        MIN_METHOD_VALUE, MAX_METHOD_VALUE);
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i]
        .str_loudness_measure[j]
        .measurement_system,
        0, MAX_MEASUREMENT_SYSTEM_TYPE);
      IMPD_DRC_BOUND_CHECK(pstr_enc_loudness_info_set->str_loudness_info_album[i]
        .str_loudness_measure[j]
        .reliability,
        0, MAX_RELIABILITY_TYPE);
    }
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_drc_validate_drc_instructions
 *
 *  \brief Validates DRC instructions
 *
 *  \param [in] pstr_uni_drc_config  Pointer to DRC config structure
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE
impeghe_drc_validate_drc_instructions(ia_drc_uni_drc_config_struct *pstr_uni_drc_config)
{
  LOOPIDX i, j;
  WORD32 profile_found = FALSE;

  for (j = 0; j < pstr_uni_drc_config->drc_coefficients_uni_drc_count; j++) {
    if (pstr_uni_drc_config->str_drc_coefficients_uni_drc[j].drc_location == 1) {
      profile_found = TRUE;
      break;
    }
  }
  if (pstr_uni_drc_config->uni_drc_config_ext_present &&
    pstr_uni_drc_config->str_uni_drc_config_ext.parametric_drc_present &&
    pstr_uni_drc_config->str_uni_drc_config_ext.str_drc_coeff_parametric_drc.drc_location ==
    1) {
    profile_found = TRUE;
  }
  if ((pstr_uni_drc_config->uni_drc_config_ext_present == 0) && profile_found == FALSE) {
    return IMPEGHE_CONFIG_FATAL_DRC_INVALID_CONFIG;
  }

  if (pstr_uni_drc_config->uni_drc_config_ext_present) {
    ia_drc_uni_drc_config_ext_struct *pstr_uni_drc_config_ext =
      &pstr_uni_drc_config->str_uni_drc_config_ext;
    profile_found = FALSE;
    if (pstr_uni_drc_config->str_uni_drc_config_ext.parametric_drc_present &&
      pstr_uni_drc_config->str_uni_drc_config_ext.str_drc_coeff_parametric_drc.drc_location ==
      1) {
      profile_found = TRUE;
    }
    if (profile_found == FALSE)
    {
      return IMPEGHE_CONFIG_FATAL_DRC_INVALID_CONFIG;
    }
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_drc_enc_init
 *
 *  \brief Initialize DRC encoder
 *
 *  \param [out] pstr_drc_state  Pointer to DRC encoder state structure
 *  \param [in] ptr_drc_scratch  Pointer to DRC scratch memory
 *  \param [in] pstr_inp_config  Pointer to DRC input config structure
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_drc_enc_init(VOID *pstr_drc_state, VOID *ptr_drc_scratch,
                                  ia_drc_input_config *pstr_inp_config)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  WORD32 bit_count = 0;
  ia_drc_enc_state *pstr_drc_state_local = pstr_drc_state;

  pstr_drc_state_local->drc_scratch_mem = ptr_drc_scratch;
  pstr_drc_state_local->drc_scratch_used = 0;

  impeghe_create_bit_buffer(&pstr_drc_state_local->str_bit_buf_cfg,
                            pstr_drc_state_local->bit_buf_base_cfg,
                            sizeof(pstr_drc_state_local->bit_buf_base_cfg));

  impeghe_create_bit_buffer(&pstr_drc_state_local->str_bit_buf_cfg_ext,
                            pstr_drc_state_local->bit_buf_base_cfg_ext,
                            sizeof(pstr_drc_state_local->bit_buf_base_cfg_ext));

  impeghe_create_bit_buffer(&pstr_drc_state_local->str_bit_buf_cfg_tmp,
                            pstr_drc_state_local->bit_buf_base_cfg_tmp,
                            sizeof(pstr_drc_state_local->bit_buf_base_cfg_tmp));

  impeghe_create_bit_buffer(&pstr_drc_state_local->str_bit_buf_out,
                            pstr_drc_state_local->bit_buf_base_out,
                            sizeof(pstr_drc_state_local->bit_buf_base_out));


  err_code = impeghe_drc_gain_enc_init(
      &pstr_drc_state_local->str_gain_enc, &pstr_inp_config->str_uni_drc_config,
      &pstr_inp_config->str_enc_loudness_info_set, pstr_inp_config->str_enc_params.frame_size,
      pstr_inp_config->str_enc_params.sample_rate, pstr_inp_config->str_enc_params.delay_mode,
      pstr_inp_config->str_enc_params.domain);
  if (err_code) {
    return err_code;
  }

  err_code = impeghe_drc_validate_drc_instructions(&pstr_inp_config->str_uni_drc_config);

  if (err_code & IA_FATAL_ERROR) {
    return IMPEGHE_CONFIG_FATAL_DRC_INVALID_CONFIG;
  }

  pstr_drc_state_local->str_enc_params = pstr_inp_config->str_enc_params;
  pstr_drc_state_local->str_uni_drc_config = pstr_inp_config->str_uni_drc_config;
  pstr_drc_state_local->str_enc_gain_extension = pstr_inp_config->str_enc_gain_extension;
  pstr_drc_state_local->str_gain_enc.str_uni_drc_config = pstr_inp_config->str_uni_drc_config;
  pstr_drc_state_local->str_enc_params.gain_sequence_present = FALSE;
  for (WORD16 k = 0; k < pstr_drc_state_local->str_uni_drc_config.drc_coefficients_uni_drc_count;
    k++) {
    if ((pstr_drc_state_local->str_uni_drc_config.str_drc_coefficients_uni_drc[k].drc_location ==
      1) &&
      (pstr_drc_state_local->str_uni_drc_config.str_drc_coefficients_uni_drc[k].gain_set_count >
        0)) {
      pstr_drc_state_local->str_enc_params.gain_sequence_present = TRUE;
      break;
    }
  }

  err_code = impeghe_drc_write_uni_drc_config(pstr_drc_state_local, &bit_count, 1);
  if (err_code & IA_FATAL_ERROR)
  {
    return err_code;
  }

  pstr_drc_state_local->drc_config_data_size_bit = bit_count;

  // Loudness info set
  if (pstr_drc_state_local->str_gain_enc.str_uni_drc_config.loudness_info_set_present == 1) {
    bit_count = 0;
    impeghe_reset_bit_buffer(&pstr_drc_state_local->str_bit_buf_cfg_ext);
    memset(pstr_drc_state_local->bit_buf_base_cfg_ext, 0,
      sizeof(MAX_DRC_PAYLOAD_BYTES * sizeof(pstr_drc_state_local->bit_buf_base_cfg_ext[0])));
    err_code = impeghe_drc_write_loudness_info_set(
      pstr_drc_state, &pstr_drc_state_local->str_bit_buf_cfg_ext, &bit_count, 1);
    if (err_code & IA_FATAL_ERROR) {
      return (err_code);
    }
    pstr_drc_state_local->drc_config_ext_data_size_bit = bit_count;
  }

  return err_code;
}

/**
 *  impeghe_loudness_info_init
 *
 *  \brief Initialize loudness for encoder
 *
 *  \param [out] pstr_drc_state  Pointer to DRC encoder state structure
 *  \param [in] ptr_drc_scratch  Pointer to DRC scratch memory
 *  \param [in] pstr_inp_config  Pointer to DRC input config structure
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_loudness_info_init(VOID *pstr_drc_state, VOID *ptr_drc_scratch,
  ia_drc_input_config *pstr_inp_config)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_drc_enc_state *pstr_drc_state_local = pstr_drc_state;

  jmp_buf drc_enc_init_jmp_buf;
  err_code = setjmp(drc_enc_init_jmp_buf);
  if (err_code != IA_NO_ERROR)
  {
    return IMPEGHE_INIT_FATAL_INSUFFICIENT_DRC_WRITE_BUFFER_SIZE;
  }

  impeghe_create_bit_buffer(&pstr_drc_state_local->str_bit_buf_cfg_ext,
    pstr_drc_state_local->bit_buf_base_cfg_ext,
    sizeof(pstr_drc_state_local->bit_buf_base_cfg_ext));

  memcpy(&pstr_drc_state_local->str_gain_enc.str_loudness_info_set,
    &pstr_inp_config->str_enc_loudness_info_set,
    sizeof(ia_drc_loudness_info_set_struct));

  err_code = impeghe_drc_write_measured_loudness_info(pstr_drc_state_local);
  return err_code;
}

/**
 *  impeghe_drc_enc
 *
 *  \brief DRC encoding
 *
 *  \param [in,out] pstr_drc_state  Pointer to DRC encoder state structure
 *  \param [in] pptr_input          Pointer to input data
 *  \param [in] inp_offset          Input buffer offset
 *  \param [out] ptr_bits_written	  Pointer to bit count
 *  \param [in] pstr_scratch		    Pointer to scratch buffer
 *
 *  \return VOID
 */
IA_ERRORCODE impeghe_drc_enc(VOID *pstr_drc_state, FLOAT32 **pptr_input, UWORD32 inp_offset,
                     WORD32 *ptr_bits_written, VOID *pstr_scratch)
{
  LOOPIDX i, j, k;
  WORD32 band_count = 0;
  WORD32 stop_sub_band_index;
  WORD32 num_bits_payload = 0;
  UWORD8 is_fft_ready[MAX_NUM_CHANNELS] = {0};
  ia_drc_enc_state *pstr_drc_state_local = pstr_drc_state;
  ia_drc_gain_enc_struct *pstr_gain_enc = &pstr_drc_state_local->str_gain_enc;
  ia_drc_uni_drc_config_struct *pstr_uni_drc_config = &pstr_drc_state_local->str_uni_drc_config;
  ia_drc_compand_struct *pstr_drc_compand;
  ia_drc_stft_gain_calc_struct *pstr_drc_stft_gain_calc;

  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_drc_coefficients_uni_drc_struct *pstr_drc_coefficients_uni_drc = &pstr_uni_drc_config->str_drc_coefficients_uni_drc[0];
  if (pstr_drc_state_local->str_enc_params.gain_sequence_present)
  {
    for (i = 0; i < MAX_DRC_COEFF_COUNT; i++)
    {
      for (j = 0; j < GAIN_SET_COUNT_MAX; j++)
      {
        pstr_drc_stft_gain_calc = &pstr_gain_enc->str_drc_stft_gain_handle[i][j][0];
        pstr_drc_compand = &pstr_gain_enc->str_drc_compand[i][j];
        if ((pstr_drc_compand->is_valid == 0) && (pstr_drc_stft_gain_calc->is_valid == 0))
        {
          break;
        }

        if (pstr_drc_compand->is_valid == 0)
        {
          if (is_fft_ready[pstr_drc_stft_gain_calc->ch_idx] == 0)
          {
            impeghe_stft_drc_convert_to_fd(
                pstr_gain_enc, &pptr_input[pstr_drc_stft_gain_calc->ch_idx][inp_offset],
                pstr_drc_stft_gain_calc->ch_idx, pstr_drc_state_local->str_enc_params.frame_size,
                pstr_gain_enc->complex_fft_ptr[pstr_drc_stft_gain_calc->ch_idx], pstr_scratch);
            is_fft_ready[pstr_drc_stft_gain_calc->ch_idx] = 1;
          }

          for (k = 0; k < pstr_uni_drc_config->str_drc_coefficients_uni_drc[i]
                              .str_gain_set_params[j]
                              .band_count;
               k++)
          {
            if (k == pstr_drc_coefficients_uni_drc[i].str_gain_set_params[j].band_count - 1) {
              stop_sub_band_index = STFT256_HOP_SIZE - 1;
            }
            else {
              stop_sub_band_index = pstr_drc_coefficients_uni_drc[i]
                                        .str_gain_set_params[j]
                                        .gain_params[k + 1]
                                        .start_sub_band_index -
                                    1;
            }

            impeghe_stft_drc_gain_calc_process(
                pstr_gain_enc, i, j, k,
                pstr_drc_coefficients_uni_drc[i]
                    .str_gain_set_params[j]
                    .gain_params[k]
                    .start_sub_band_index,
                stop_sub_band_index, pstr_drc_state_local->str_enc_params.frame_size,
                pstr_gain_enc->complex_fft_ptr[pstr_drc_stft_gain_calc->ch_idx],
                pstr_drc_state_local->gain_buffer[band_count + k]);
          }
        }
        else
        {
          impeghe_td_drc_gain_calc_process(pstr_gain_enc, i, j,
                                           pstr_drc_state_local->str_enc_params.frame_size,
                                           &pptr_input[pstr_drc_compand->ch_idx][inp_offset],
                                           pstr_drc_state_local->gain_buffer[band_count]);
        }

        band_count += pstr_drc_coefficients_uni_drc[i].str_gain_set_params[j].band_count;
      }
    }
  }
  err_code = impeghe_drc_encode_uni_drc_gain(pstr_gain_enc, pstr_drc_state_local->gain_buffer[0],
                                  pstr_scratch);
  if (err_code) {
    return err_code;
  }

  if (pstr_drc_state_local->is_first_drc_process_complete == 1)
  {
    impeghe_drc_write_uni_drc_gain(pstr_drc_state_local, &num_bits_payload);
  }

  *ptr_bits_written = num_bits_payload;
  return err_code;
}
