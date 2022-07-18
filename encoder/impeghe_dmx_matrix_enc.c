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
#include <math.h>
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_dmx_matrix_enc_api.h"

static const FLOAT32 impeghe_eq_precisions[4] = {1.0f, 0.5f, 0.25f, 0.1f};

static const FLOAT32 impeghe_eq_min_ranges[2][4] = {{-8.0f, -8.0f, -8.0f, -6.4f},
                                                    {-16.0f, -16.0f, -16.0f, -12.8f}};

static const FLOAT32 impeghe_eq_max_ranges[2][4] = {{7.0f, 7.5f, 7.75f, 6.3f},
                                                    {15.0f, 15.5f, 15.75f, 12.7f}};

/**
 *  impeghe_dmx_write_range
 *
 *  \brief Writes range into bit-buffer
 *
 *  \param [in,out] it_bit_buf  Pointer to bit-buffer
 *  \param [in] 	value          Value to be written into bit-buffer
 *  \param [in] 	alphabet_size  Alphabet size
 *  \param [out] 	ptr_bit_cnt    Pointer to bit count
 *
 *  \return VOID
 */
static VOID impeghe_dmx_write_range(ia_bit_buf_struct *it_bit_buf, UWORD32 value,
                                    UWORD32 alphabet_size, WORD32 *ptr_bit_cnt)
{
  WORD32 ilog2 = 0;
  WORD32 num_bits;
  UWORD32 num_unused;
  UWORD32 size = alphabet_size;

  while (size > 1)
  {
    size >>= 1;
    ++ilog2;
  }
  num_bits = ilog2;
  num_unused = (1U << (num_bits + 1)) - alphabet_size;

  if (value >= num_unused)
  {
    value += num_unused;
    *ptr_bit_cnt += impeghe_write_bits_buf(it_bit_buf, value >> 1, num_bits);
    *ptr_bit_cnt += impeghe_write_bits_buf(it_bit_buf, value & 1, 1);
  }
  else
  {
    *ptr_bit_cnt += impeghe_write_bits_buf(it_bit_buf, value, num_bits);
  }
}

/**
 *  impeghe_dmx_coder_state_compute_best_param
 *
 *  \brief Computes the best parameter
 *
 *  \param [in,out] coder_state Pointer to downmix matrix coder state structure
 *
 *  \return WORD32	Best length
 */
static WORD32
impeghe_dmx_coder_state_compute_best_param(ia_dmx_mtx_coder_state_struct *coder_state)
{
  LOOPIDX i, k;
  WORD32 test_param;
  WORD32 test_length;
  WORD32 best_param = -1;
  WORD32 best_length = 1 << 30;

  for (test_param = 0; test_param < (1 << 3); test_param++)
  {
    WORD32 max_head = (coder_state->gain_table_size - 1) >> test_param;
    test_length = 0;

    for (i = 0; i < coder_state->history_count; i++)
    {
      WORD32 gain_index = -1;
      FLOAT32 gain_db = coder_state->history[i];

      for (k = 0; k < coder_state->gain_table_size; k++)
      {
        if (gain_db == coder_state->gain_table[k])
        {
          gain_index = k;
          break;
        }
      }
      if (-1 == gain_index)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }

      test_length += MIN((gain_index >> test_param) + 1, max_head) + test_param;
    }

    if (test_length < best_length)
    {
      best_length = test_length;
      best_param = test_param;
    }
  }

  coder_state->gain_l_g_r_param = best_param;

  return best_length;
}

/**
 *  impeghe_dmx_encode_gain_value
 *
 *  \brief Encodes gain value
 *
 *  \param [in,out] it_bit_buf  Pointer to bit-buffer
 *  \param [in,out] coder_state  Pointer to downmix matrix coder state structure
 *  \param [in] 	gain_db        Gain value in db
 *  \param [out] 	ptr_bit_cnt    Pointer to bit count
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_dmx_encode_gain_value(ia_bit_buf_struct *it_bit_buf,
                                                  ia_dmx_mtx_coder_state_struct *coder_state,
                                                  FLOAT32 gain_db, WORD32 *ptr_bit_cnt)
{
  WORD32 bit_cnt_local = 0;
  WORD32 gain_index = -1;
  WORD32 max_gain = coder_state->max_gain;
  WORD32 min_gain = coder_state->min_gain;
  WORD32 precision_level = coder_state->precision_level;

  if ((coder_state->gain_l_g_r_param == -1) && !coder_state->raw_coding_nonzeros)
  {
    coder_state->history[coder_state->history_count++] = gain_db;
    return IA_NO_ERROR;
  }

  if (coder_state->raw_coding_nonzeros)
  {
    WORD32 num_values = ((max_gain - min_gain) << precision_level) + 2;

    if (gain_db != IMPEGHE_DMX_MATRIX_GAIN_ZERO)
    {
      gain_index = (WORD32)((max_gain - gain_db) * (FLOAT32)(1 << precision_level));
    }
    else
    {
      gain_index = num_values - 1;
    }

    impeghe_dmx_write_range(it_bit_buf, gain_index, num_values, &bit_cnt_local);
  }
  else
  {
    LOOPIDX idx;
    WORD32 head;
    WORD32 max_head = (coder_state->gain_table_size - 1) >> coder_state->gain_l_g_r_param;

    for (idx = 0; idx < coder_state->gain_table_size; idx++)
    {
      if (gain_db == coder_state->gain_table[idx])
      {
        gain_index = idx;
        break;
      }
    }
    if (-1 == gain_index)
    {
      return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    }

    head = gain_index >> coder_state->gain_l_g_r_param;
    for (idx = head - 1; idx >= 0; idx--)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 0, 1);
    }

    if (head < max_head)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 1, 1);
    }
    bit_cnt_local += impeghe_write_bits_buf(
        it_bit_buf, gain_index & ((1 << coder_state->gain_l_g_r_param) - 1),
        coder_state->gain_l_g_r_param);
  }

  *ptr_bit_cnt += bit_cnt_local;

  return IA_NO_ERROR;
}

/**
 *  impeghe_dmx_encode_flat_compact_matrix
 *
 *  \brief Encodes flat compact matrix
 *
 *  \param [in,out] it_bit_buf        Pointer to bit-buffer
 *  \param [out]  flat_compact_matrix  Pointer to flat compact matrix
 *  \param [in]   total_count          Matrix size
 *  \param [out]  ptr_bit_cnt          Pointer to bit count
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_dmx_encode_flat_compact_matrix(ia_bit_buf_struct *it_bit_buf,
                                                           WORD8 *flat_compact_matrix,
                                                           WORD32 total_count,
                                                           WORD32 *ptr_bit_cnt)
{
  LOOPIDX idx;
  WORD32 max_head, count;
  WORD32 num = 3;
  WORD32 test_param, test_length;
  WORD32 best_param = -1;
  WORD32 best_length = 1 << 30;
  WORD32 bit_count = 0;
  WORD32 bit_cnt_local = 0;

  if (flat_compact_matrix[total_count - 1] == 0)
  {
    flat_compact_matrix[total_count] = 1;
  }
  if (total_count >= 256)
  {
    num = 4;
  }

  for (test_param = 0; test_param < (1 << num); test_param++)
  {
    idx = 0;
    test_length = num;
    max_head = total_count >> test_param;
    do
    {
      count = 0;
      while (flat_compact_matrix[idx++] == 0)
      {
        count++;
      }
      test_length += MIN((count >> test_param) + 1, max_head) + test_param;
    } while (idx < total_count);

    if (test_length < best_length)
    {
      best_length = test_length;
      best_param = test_param;
    }
  }

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, best_param, num);
  idx = 0;
  max_head = total_count >> best_param;
  do
  {
    LOOPIDX t;
    WORD32 head;

    count = 0;
    while (flat_compact_matrix[idx++] == 0)
    {
      count++;
    }
    head = count >> best_param;
    for (t = head - 1; t >= 0; t--)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 0, 1);
    }
    if (head < max_head)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 1, 1);
    }
    bit_cnt_local +=
        impeghe_write_bits_buf(it_bit_buf, count & ((1 << best_param) - 1), best_param);
  } while (idx < total_count);

  bit_count = bit_cnt_local;
  if (bit_count != best_length)
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }

  *ptr_bit_cnt += bit_cnt_local;

  return IA_NO_ERROR;
}

/**
 *  impeghe_dmx_encode_eq_config
 *
 *  \brief Encodes equalizer config data
 *
 *  \param [in,out] it_bit_buf     Pointer to bit-buffer
 *  \param [in] input_count         Number of input channels
 *  \param [in] eq_precision_level  Equalizer precision level
 *  \param [in] pstr_eq_config      Pointer to downmix equalizer config structure
 *  \param [out] ptr_bit_cnt        Pointer to bit count
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_dmx_encode_eq_config(ia_bit_buf_struct *it_bit_buf,
                                                 WORD32 input_num_channels,
                                                 WORD32 eq_precision_level,
                                                 ia_dmx_eq_config_struct *pstr_eq_config,
                                                 WORD32 *ptr_bit_cnt)
{
  LOOPIDX i, j, k;
  WORD32 sg_precision_level = 0;
  WORD32 eq_extended_range = 0;
  WORD32 bit_cnt_local = 0;

  bit_cnt_local += impeghe_write_escape_value(it_bit_buf, pstr_eq_config->num_eq - 1, 3, 5, 0);
  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, eq_precision_level, 2);

  sg_precision_level = MIN(eq_precision_level + 1, 3);
  for (i = pstr_eq_config->num_eq - 1; i >= 0; i--)
  {
    if ((pstr_eq_config->eq_params[i].global_gain_db >
         impeghe_eq_max_ranges[0][sg_precision_level]) ||
        (pstr_eq_config->eq_params[i].global_gain_db <
         impeghe_eq_min_ranges[0][sg_precision_level]))
    {
      eq_extended_range = 1;
      break;
    }
    for (j = pstr_eq_config->eq_params[i].num_pk_filter - 1; j >= 0; j--)
    {
      if ((pstr_eq_config->eq_params[i].pk_filter_params[j].peak_gain_db >
           impeghe_eq_max_ranges[0][eq_precision_level]) ||
          (pstr_eq_config->eq_params[i].pk_filter_params[j].peak_gain_db <
           impeghe_eq_min_ranges[0][eq_precision_level]))
      {
        eq_extended_range = 1;
        break;
      }
    }
  }
  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, eq_extended_range, 1);

  for (i = 0; i < pstr_eq_config->num_eq; i++)
  {
    WORD32 last_center_freq_p10 = 0;
    WORD32 last_center_freq_ld2 = 10;
    WORD32 max_center_freq_ld2 = 99;
    WORD32 scaling_gain_index;
    FLOAT32 scaling_gain;

    bit_cnt_local += impeghe_write_escape_value(
        it_bit_buf, pstr_eq_config->eq_params[i].num_pk_filter - 1, 2, 4, 0);

    for (j = 0; j < pstr_eq_config->eq_params[i].num_pk_filter - 1; j++)
    {
      ia_dmx_pk_filter_params_struct *filter_param_j =
          &(pstr_eq_config->eq_params[i].pk_filter_params[j]);
      for (k = j + 1; k < pstr_eq_config->eq_params[i].num_pk_filter; k++)
      {
        ia_dmx_pk_filter_params_struct *filter_param_k =
            &(pstr_eq_config->eq_params[i].pk_filter_params[k]);
        if (filter_param_j->peak_freq_hz > filter_param_k->peak_freq_hz)
        {
          FLOAT32 temp;
          temp = filter_param_j->peak_freq_hz;
          filter_param_j->peak_freq_hz = filter_param_k->peak_freq_hz;
          filter_param_k->peak_freq_hz = temp;
          temp = filter_param_j->peak_q_factor;
          filter_param_j->peak_q_factor = filter_param_k->peak_q_factor;
          filter_param_k->peak_q_factor = temp;
          temp = filter_param_j->peak_gain_db;
          filter_param_j->peak_gain_db = filter_param_k->peak_gain_db;
          filter_param_k->peak_gain_db = temp;
        }
      }
    }

    for (j = 0; j < pstr_eq_config->eq_params[i].num_pk_filter; j++)
    {
      WORD32 center_freq;
      WORD32 center_freq_p10;
      WORD32 center_freq_ld2;
      WORD32 q_factor_index;
      WORD32 q_factor_extra;
      WORD32 center_gain_index;
      FLOAT32 quality_factor;
      FLOAT32 center_gain;

      center_freq =
          (WORD32)(pstr_eq_config->eq_params[i].pk_filter_params[j].peak_freq_hz + 0.5f);
      if ((center_freq > 24000) || (center_freq < 10))
      {
        center_freq = MAX(10, MIN(center_freq, 24000));
      }
      if (center_freq >= 9950)
      {
        center_freq_ld2 = (center_freq + 500) / 1000;
        center_freq_ld2 = MIN(center_freq_ld2, 24);
        center_freq_p10 = 3;
      }
      else if (center_freq >= 995)
      {
        center_freq_ld2 = (center_freq + 50) / 100;
        center_freq_p10 = 2;
      }
      else if (center_freq >= 100)
      {
        center_freq_ld2 = (center_freq + 5) / 10;
        center_freq_p10 = 1;
      }
      else
      {
        center_freq_ld2 = center_freq;
        center_freq_p10 = 0;
      }
      if (center_freq_p10 < last_center_freq_p10)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }

      impeghe_dmx_write_range(it_bit_buf, center_freq_p10 - last_center_freq_p10,
                              4 - last_center_freq_p10, &bit_cnt_local);

      if (center_freq_p10 == 3)
      {
        max_center_freq_ld2 = 24;
      }
      if (center_freq_p10 > last_center_freq_p10)
      {
        last_center_freq_ld2 = 10;
      }
      if (center_freq_ld2 < last_center_freq_ld2)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
      impeghe_dmx_write_range(it_bit_buf, center_freq_ld2 - last_center_freq_ld2,
                              1 + max_center_freq_ld2 - last_center_freq_ld2, &bit_cnt_local);

      last_center_freq_ld2 = center_freq_ld2;
      last_center_freq_p10 = center_freq_p10;

      quality_factor = pstr_eq_config->eq_params[i].pk_filter_params[j].peak_q_factor;
      if ((quality_factor > 10.6f) || (quality_factor < 0.05f))
      {
        quality_factor = MAX(0.05f, MIN(quality_factor, 10.6f));
      }
      if (quality_factor >= 1.05f)
      {
        WORD32 q_factor = (WORD32)((quality_factor - 1.0f) * 10.0f + 0.5f) - 1;
        q_factor = MIN(q_factor, (31 - 20) * 8 + 7);
        q_factor_index = (q_factor >> 3) + 20;
        q_factor_extra = q_factor & 7;
        bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, q_factor_index, 5);
        bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, q_factor_extra, 3);
      }
      else
      {
        q_factor_index = (WORD32)((quality_factor - 0.05f) * 20.0f + 0.5f);
        q_factor_index = MIN(q_factor_index, 19);
        bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, q_factor_index, 5);
      }

      center_gain = pstr_eq_config->eq_params[i].pk_filter_params[j].peak_gain_db;
      if ((center_gain > impeghe_eq_max_ranges[eq_extended_range][eq_precision_level]) ||
          (center_gain < impeghe_eq_min_ranges[eq_extended_range][eq_precision_level]))
      {
        if (eq_extended_range == 0)
        {
          return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
        }
        center_gain =
            MAX(impeghe_eq_min_ranges[eq_extended_range][eq_precision_level],
                MIN(center_gain, impeghe_eq_max_ranges[eq_extended_range][eq_precision_level]));
      }
      center_gain_index =
          (WORD32)((center_gain - impeghe_eq_min_ranges[eq_extended_range][eq_precision_level]) /
                       impeghe_eq_precisions[eq_precision_level] +
                   0.5f);
      if (center_gain_index >= (1 << (4 + eq_extended_range + eq_precision_level)))
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, center_gain_index,
                                              4 + eq_extended_range + eq_precision_level);
    }

    scaling_gain = pstr_eq_config->eq_params[i].global_gain_db;
    if ((scaling_gain > impeghe_eq_max_ranges[eq_extended_range][sg_precision_level]) ||
        (scaling_gain < impeghe_eq_min_ranges[eq_extended_range][sg_precision_level]))
    {
      if (eq_extended_range == 0)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
      scaling_gain =
          MAX(impeghe_eq_min_ranges[eq_extended_range][sg_precision_level],
              MIN(scaling_gain, impeghe_eq_max_ranges[eq_extended_range][sg_precision_level]));
    }
    scaling_gain_index =
        (WORD32)((scaling_gain - impeghe_eq_min_ranges[eq_extended_range][sg_precision_level]) /
                     impeghe_eq_precisions[sg_precision_level] +
                 0.5f);
    if (scaling_gain_index >= (1 << (4 + eq_extended_range + sg_precision_level)))
    {
      return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    }
    bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, scaling_gain_index,
                                            4 + eq_extended_range + sg_precision_level);
  }

  for (i = 0; i < input_num_channels; i++)
  {
    if (pstr_eq_config->eq_map[i] == 0)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 0, 1);
    }
    else
    {
      if (pstr_eq_config->eq_map[i] > pstr_eq_config->num_eq)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 1, 1);
      impeghe_dmx_write_range(it_bit_buf, pstr_eq_config->eq_map[i] - 1, pstr_eq_config->num_eq,
                              &bit_cnt_local);
    }
  }

  *ptr_bit_cnt += bit_cnt_local;
  return IA_NO_ERROR;
}

/**
 *  impeghe_dmx_encode_downmix_matrix
 *
 *  \brief Encodes downmix matrix set
 *
 *  \param [in] input_index             Input CICP Index
 *  \param [in] input_num_channels      Input number of channels
 *  \param [in,out] pstr_input_config   Pointer to input downmix speaker information structure
 *  \param [in] output_index            Output CICP Index
 *  \param [in] output_num_channels     Output number of channels
 *  \param [in,out] pstr_output_config  Pointer to output downmix speaker information structure
 *  \param [in] precision_level         Precision level
 *  \param [in,out] it_bit_buf         Pointer to bit-buffer
 *  \param [in,out] ptr_downmix_matrix  Pointer to flat downmix matrix
 *  \param [in] eq_precision_level      Equalizer precision level
 *  \param [in] pstr_eq_config          Pointer to downmix equalizer config structure
 *  \param [out] pstr_dmx_scratch       Pointer to scratch memory
 *  \param [out] ptr_bit_cnt            Pointer to bit count
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_dmx_encode_downmix_matrix(
    WORD32 input_index, WORD32 input_num_channels,
    ia_dmx_speaker_information_struct *pstr_input_config, WORD32 output_index,
    WORD32 output_num_channels, ia_dmx_speaker_information_struct *pstr_output_config,
    WORD32 precision_level, ia_bit_buf_struct *it_bit_buf, FLOAT32 *ptr_downmix_matrix,
    WORD32 eq_precision_level, ia_dmx_eq_config_struct *pstr_eq_config,
    ia_dmx_sratch *pstr_dmx_scratch, WORD32 *ptr_bit_cnt)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  LOOPIDX i, j;
  WORD32 compact_input_count = 0;
  WORD32 compact_output_count = 0;
  WORD32 is_separable[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 is_symmetric[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 is_all_separable = 1;
  WORD32 is_all_symmetric = 1;
  WORD32 config_full_for_asymmetric_inputs = 0;
  WORD32 config_mix_lfe_only_to_lfe = 1;
  WORD32 config_raw_coding_compact_matrix = 0;
  WORD32 config_use_compact_template = 1;
  WORD32 config_raw_coding_non_zeros = 0;
  WORD32 use_compact_template = config_use_compact_template;
  WORD32 mix_lfe_only_to_lfe = config_mix_lfe_only_to_lfe;
  WORD32 raw_coding_compact_matrix = config_raw_coding_compact_matrix;
  WORD32 gains_best_length = 0;
  WORD32 step;
  WORD32 bit_count_save = 0;
  WORD32 bit_count = 0;
  WORD32 bit_cnt_local = 0;
  FLOAT32 multiplier = (FLOAT32)(1 << precision_level);
  FLOAT32 inv_multiplier = (FLOAT32)(1.0f / multiplier);
  FLOAT32 min_gain = 96.0f;
  FLOAT32 max_gain = -96.0f;
  WORD8 *compact_template = NULL;
  ia_dmx_speaker_information_struct
      *pstr_compact_input_config[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  ia_dmx_speaker_information_struct
      *pstr_compact_output_config[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT];
  ia_dmx_mtx_coder_state_struct coder_state;

  if (pstr_eq_config->num_eq == 0)
  {
    bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 0, 1);
  }
  else
  {
    bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, 1, 1);
    err_code = impeghe_dmx_encode_eq_config(it_bit_buf, input_num_channels, eq_precision_level,
                                            pstr_eq_config, &bit_cnt_local);
    if (err_code)
    {
      return err_code;
    }
  }

  impeghe_dmx_convert_to_compact_config(input_num_channels, pstr_input_config,
                                        &compact_input_count, pstr_compact_input_config);
  impeghe_dmx_convert_to_compact_config(output_num_channels, pstr_output_config,
                                        &compact_output_count, pstr_compact_output_config);

  if (precision_level > 2)
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }
  for (i = 0; i < input_num_channels; i++)
  {
    for (j = 0; j < output_num_channels; j++)
    {
      FLOAT32 value = ptr_downmix_matrix[i * output_num_channels + j];
      if (value == 0.0f)
      {
        ptr_downmix_matrix[i * output_num_channels + j] = IMPEGHE_DMX_MATRIX_GAIN_ZERO;
      }
      else
      {
        FLOAT32 gain_db = (FLOAT32)(20.0f * log10(value));
        gain_db = (FLOAT32)(floor(gain_db * multiplier + 0.5) * inv_multiplier);
        gain_db = (FLOAT32)MIN(gain_db, 22.0f);
        gain_db = (FLOAT32)MAX(gain_db, -47.0f);
        ptr_downmix_matrix[i * output_num_channels + j] = gain_db;
        max_gain = (FLOAT32)MAX(max_gain, ceil(gain_db));
        min_gain = (FLOAT32)MIN(min_gain, floor(gain_db));
      }
    }
  }
  if (min_gain == 96.0f)
  {
    max_gain = 0.0f;
    min_gain = -1.0f;
  }
  if (max_gain <= -1.0f)
  {
    max_gain = 0.0f;
  }
  if (min_gain >= 0.0f)
  {
    min_gain = -1.0f;
  }

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, precision_level, 2);
  bit_cnt_local += impeghe_write_escape_value(it_bit_buf, (UWORD32)max_gain, 3, 4, 0);
  bit_cnt_local += impeghe_write_escape_value(it_bit_buf, (UWORD32)(-min_gain - 1), 4, 5, 0);

  for (j = compact_output_count - 1; j >= 0; j--)
  {
    is_symmetric[j] = 1;
    is_separable[j] = 1;
  }

  for (i = 0; i < compact_input_count; i++)
  {
    if (pstr_compact_input_config[i]->pair_type == SP_PAIR_NONE)
    {
      return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    }

    for (j = 0; j < compact_output_count; j++)
    {
      FLOAT32 mat[2][2] = {{0}};
      WORD32 ci = 1 + (pstr_compact_input_config[i]->pstr_symmetric_pair != NULL);
      WORD32 co = 1 + (pstr_compact_output_config[j]->pstr_symmetric_pair != NULL);

      if (pstr_compact_output_config[j]->pair_type == SP_PAIR_NONE)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }

      mat[0][0] = ptr_downmix_matrix[(pstr_compact_input_config[i]->original_position *
                                      output_num_channels) +
                                     pstr_compact_output_config[j]->original_position];
      if (co == 2)
      {
        mat[0][1] = ptr_downmix_matrix
            [(pstr_compact_input_config[i]->original_position * output_num_channels) +
             pstr_compact_output_config[j]->pstr_symmetric_pair->original_position];
        if (ci == 2)
        {
          mat[1][1] = ptr_downmix_matrix
              [(pstr_compact_input_config[i]->pstr_symmetric_pair->original_position *
                output_num_channels) +
               pstr_compact_output_config[j]->pstr_symmetric_pair->original_position];
        }
      }
      if (ci == 2)
      {
        mat[1][0] = ptr_downmix_matrix[(pstr_compact_input_config[i]
                                            ->pstr_symmetric_pair->original_position *
                                        output_num_channels) +
                                       pstr_compact_output_config[j]->original_position];
      }

      if ((ci == 1) && (co == 2))
      {
        if (config_full_for_asymmetric_inputs)
        {
          if ((pstr_compact_input_config[i]->pair_type != SP_PAIR_SINGLE) &&
              (mat[0][0] != mat[0][1]))
          {
            is_symmetric[j] = 0;
          }
        }
        else
        {
          if (mat[0][0] != mat[0][1])
          {
            is_symmetric[j] = 0;
          }
        }
        pstr_dmx_scratch->compact_downmix_matrix[i][j] =
            ((mat[0][1] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
             (mat[0][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO));
      }
      else if ((ci == 2) && (co == 1))
      {
        if (mat[0][0] != mat[1][0])
        {
          is_symmetric[j] = 0;
        }
        pstr_dmx_scratch->compact_downmix_matrix[i][j] =
            ((mat[1][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
             (mat[0][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO));
      }
      else if ((ci == 1) && (co == 1))
      {
        pstr_dmx_scratch->compact_downmix_matrix[i][j] =
            (mat[0][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO);
      }
      else
      {
        if ((mat[1][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
            (mat[0][1] != IMPEGHE_DMX_MATRIX_GAIN_ZERO))
        {
          is_separable[j] = 0;
        }
        if ((mat[0][0] != mat[1][1]) || (mat[0][1] != mat[1][0]))
        {
          is_symmetric[j] = 0;
        }

        pstr_dmx_scratch->compact_downmix_matrix[i][j] =
            ((mat[0][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
             (mat[1][0] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
             (mat[0][1] != IMPEGHE_DMX_MATRIX_GAIN_ZERO) ||
             (mat[1][1] != IMPEGHE_DMX_MATRIX_GAIN_ZERO));
      }

      is_all_symmetric &= is_symmetric[j];
      is_all_separable &= is_separable[j];
      if (pstr_dmx_scratch->compact_downmix_matrix[i][j])
      {
        mix_lfe_only_to_lfe &=
            (pstr_compact_input_config[i]->is_lfe == pstr_compact_output_config[j]->is_lfe);
      }
    }
  }

  coder_state.history_count = 0;
  coder_state.raw_coding_nonzeros = config_raw_coding_non_zeros;
  coder_state.gain_l_g_r_param = -1;
  coder_state.min_gain = (WORD32)min_gain;
  coder_state.max_gain = (WORD32)max_gain;
  coder_state.precision_level = precision_level;
  coder_state.gain_table_size = 0;

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, is_all_separable, 1);
  if (!is_all_separable)
  {
    for (j = 0; j < compact_output_count; j++)
    {
      if (pstr_compact_output_config[j]->pair_type == SP_PAIR_SYMMETRIC)
      {
        bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, is_separable[j], 1);
      }
    }
  }

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, is_all_symmetric, 1);
  if (!is_all_symmetric)
  {
    for (j = 0; j < compact_output_count; j++)
    {
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, is_symmetric[j], 1);
    }
  }

  if ((input_index == -1) || (output_index == -1))
  {
    use_compact_template = 0;
  }

  if (compact_input_count * compact_output_count <= 8)
  {
    raw_coding_compact_matrix = 1;
  }

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, mix_lfe_only_to_lfe, 1);
  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, raw_coding_compact_matrix, 1);

  if (!raw_coding_compact_matrix)
  {
    WORD8 flat_compact_matrix[IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT *
                                  IMPEGHE_DMX_MATRIX_MAX_SPEAKER_COUNT +
                              1];
    WORD32 count = 0;
    WORD32 total_count = 0;

    if (!use_compact_template)
    {
      for (i = 0; i < compact_input_count * compact_output_count; i++)
      {
        pstr_dmx_scratch->compact_template_buffer[i] = 0;
      }
      compact_template = (WORD8 *)pstr_dmx_scratch->compact_template_buffer;
    }
    else
    {
      compact_template = impeghe_dmx_find_compact_template(input_index, output_index, &err_code);
      if (err_code)
      {
        return err_code;
      }
      if (compact_template == NULL)
      {
        use_compact_template = 0;
        for (i = 0; i < compact_input_count * compact_output_count; i++)
        {
          pstr_dmx_scratch->compact_template_buffer[i] = 0;
        }
        compact_template = (WORD8 *)pstr_dmx_scratch->compact_template_buffer;
      }
    }
    bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, use_compact_template, 1);

    if (!mix_lfe_only_to_lfe)
    {
      total_count = compact_input_count * compact_output_count;
    }
    else
    {
      WORD32 compact_input_lfe_count = 0;
      WORD32 compact_output_lfe_count = 0;

      for (i = 0; i < compact_input_count; i++)
      {
        if (pstr_compact_input_config[i]->is_lfe)
        {
          compact_input_lfe_count++;
        }
      }
      for (i = 0; i < compact_output_count; i++)
      {
        if (pstr_compact_output_config[i]->is_lfe)
        {
          compact_output_lfe_count++;
        }
      }
      total_count = (compact_input_count - compact_input_lfe_count) *
                    (compact_output_count - compact_output_lfe_count);
    }

    for (i = 0; i < compact_input_count; i++)
    {
      for (j = 0; j < compact_output_count; j++)
      {
        if (!mix_lfe_only_to_lfe ||
            (!pstr_compact_input_config[i]->is_lfe && !pstr_compact_output_config[j]->is_lfe))
        {
          flat_compact_matrix[count++] = pstr_dmx_scratch->compact_downmix_matrix[i][j] ^
                                         compact_template[i * compact_output_count + j];
        }
      }
    }
    if (count != total_count)
    {
      return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    }

    err_code = impeghe_dmx_encode_flat_compact_matrix(it_bit_buf, flat_compact_matrix,
                                                      total_count, &bit_cnt_local);
    if (err_code)
    {
      return err_code;
    }

    count = 0;
    for (i = 0; i < compact_input_count; i++)
    {
      for (j = 0; j < compact_output_count; j++)
      {
        if (mix_lfe_only_to_lfe && pstr_compact_input_config[i]->is_lfe &&
            pstr_compact_output_config[j]->is_lfe)
        {
          bit_cnt_local += impeghe_write_bits_buf(
              it_bit_buf, pstr_dmx_scratch->compact_downmix_matrix[i][j], 1);
        }
        else if (mix_lfe_only_to_lfe &&
                 (pstr_compact_input_config[i]->is_lfe ^ pstr_compact_output_config[j]->is_lfe))
        {
          if (pstr_dmx_scratch->compact_downmix_matrix[i][j] != 0)
          {
            return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
          }
        }
        else
        {
          count++;
        }
      }
    }
    if (count != total_count)
    {
      return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    }
  }
  else
  {
    for (i = 0; i < compact_input_count; i++)
    {
      for (j = 0; j < compact_output_count; j++)
      {
        if (!mix_lfe_only_to_lfe ||
            (pstr_compact_input_config[i]->is_lfe == pstr_compact_output_config[j]->is_lfe))
        {
          bit_cnt_local += impeghe_write_bits_buf(
              it_bit_buf, pstr_dmx_scratch->compact_downmix_matrix[i][j], 1);
        }
        else
        {
          if (pstr_dmx_scratch->compact_downmix_matrix[i][j] != 0)
          {
            return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
          }
        }
      }
    }
  }

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, config_full_for_asymmetric_inputs, 1);
  bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, config_raw_coding_non_zeros, 1);

  for (step = 0; step <= !config_raw_coding_non_zeros; step++)
  {
    if (!config_raw_coding_non_zeros && (step == 0))
    {
      err_code = impeghe_dmx_coder_state_generate_gain_table(&coder_state);
      if (err_code)
      {
        return err_code;
      }
    }
    if (!config_raw_coding_non_zeros && (step == 1))
    {
      gains_best_length = impeghe_dmx_coder_state_compute_best_param(&coder_state);
      bit_cnt_local += impeghe_write_bits_buf(it_bit_buf, coder_state.gain_l_g_r_param, 3);

      bit_count_save = bit_cnt_local;
    }

    for (i = 0; i < compact_input_count; i++)
    {
      if (pstr_compact_input_config[i]->pair_type == SP_PAIR_NONE)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }

      for (j = 0; j < compact_output_count; j++)
      {
        FLOAT32 mat[2][2] = {{0}};
        WORD32 ci = 1 + (pstr_compact_input_config[i]->pstr_symmetric_pair != NULL);
        WORD32 co = 1 + (pstr_compact_output_config[j]->pstr_symmetric_pair != NULL);

        if (pstr_compact_output_config[j]->pair_type == SP_PAIR_NONE)
        {
          return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
        }

        mat[0][0] = ptr_downmix_matrix[(pstr_compact_input_config[i]->original_position *
                                        output_num_channels) +
                                       pstr_compact_output_config[j]->original_position];

        if (co == 2)
        {
          mat[0][1] = ptr_downmix_matrix
              [(pstr_compact_input_config[i]->original_position * output_num_channels) +
               pstr_compact_output_config[j]->pstr_symmetric_pair->original_position];
          if (ci == 2)
          {
            mat[1][1] = ptr_downmix_matrix
                [(pstr_compact_input_config[i]->pstr_symmetric_pair->original_position *
                  output_num_channels) +
                 pstr_compact_output_config[j]->pstr_symmetric_pair->original_position];
          }
        }
        if (ci == 2)
        {
          mat[1][0] = ptr_downmix_matrix[(pstr_compact_input_config[i]
                                              ->pstr_symmetric_pair->original_position *
                                          output_num_channels) +
                                         pstr_compact_output_config[j]->original_position];
        }

        if (pstr_dmx_scratch->compact_downmix_matrix[i][j] != 0)
        {
          if ((ci == 1) && (co == 2))
          {
            WORD32 is_use_full = ((pstr_compact_input_config[i]->pair_type == SP_PAIR_SINGLE) &&
                                  config_full_for_asymmetric_inputs);
            err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][0],
                                                     &bit_cnt_local);
            if (err_code)
            {
              return err_code;
            }

            if (!is_use_full && is_symmetric[j])
            {
              if (mat[0][0] != mat[0][1])
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
            }
            else
            {
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][1],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
            }
          }
          else if ((ci == 2) && (co == 1))
          {
            err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][0],
                                                     &bit_cnt_local);
            if (err_code)
            {
              return err_code;
            }
            if (is_symmetric[j])
            {
              if (mat[0][0] != mat[1][0])
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
            }
            else
            {
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[1][0],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
            }
          }
          else if ((ci == 1) && (co == 1))
          {
            err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][0],
                                                     &bit_cnt_local);
            if (err_code)
            {
              return err_code;
            }
          }
          else
          {
            err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][0],
                                                     &bit_cnt_local);
            if (err_code)
            {
              return err_code;
            }

            if (!is_separable[j] && is_symmetric[j])
            {
              if (!((mat[0][1] == mat[1][0]) && (mat[0][0] == mat[1][1])))
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][1],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
            }
            else if (is_separable[j] && !is_symmetric[j])
            {
              if (!((mat[0][1] == IMPEGHE_DMX_MATRIX_GAIN_ZERO) &&
                    (mat[1][0] == IMPEGHE_DMX_MATRIX_GAIN_ZERO)))
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[1][1],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
            }
            else if (is_separable[j] && is_symmetric[j])
            {
              if (!((mat[0][1] == IMPEGHE_DMX_MATRIX_GAIN_ZERO) &&
                    (mat[1][0] == IMPEGHE_DMX_MATRIX_GAIN_ZERO)))
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
              if (!((mat[0][1] == mat[1][0]) && (mat[0][0] == mat[1][1])))
              {
                return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
              }
            }
            else
            {
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[0][1],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[1][0],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
              err_code = impeghe_dmx_encode_gain_value(it_bit_buf, &coder_state, mat[1][1],
                                                       &bit_cnt_local);
              if (err_code)
              {
                return err_code;
              }
            }
          }
        }
      }
    }

    if (!config_raw_coding_non_zeros && (step == 1))
    {
      bit_count = bit_cnt_local;
      if (bit_count - bit_count_save != gains_best_length)
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
    }
  }

  *ptr_bit_cnt += bit_cnt_local;

  return err_code;
}
