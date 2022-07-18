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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_error_standards.h"
#include "impeghe_hoa_common_values.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_hoa_matrix.h"

/**impeghe_hoa_mtrx_state_init
 *
 *  \brief Matrix pstr_hoa_state initialization
 *
 *  \param [in] pstr_hoa_state state handle
 *  \param [in] max_gain maximum gain
 *
 *  \return VOID
 */
static VOID impeghe_hoa_mtrx_state_init(impeghe_hoa_matrix_state_str *pstr_hoa_state,
                                        FLOAT32 max_gain)
{
  WORD32 i;
  for (i = 0; i < (HOA_ORDER + 1); ++i)
  {
    pstr_hoa_state->min_gain[i] = -1;
    pstr_hoa_state->max_gain[i] = (WORD32)max_gain;
  }
  pstr_hoa_state->prec_lvl = 0;
  pstr_hoa_state->non_zero_coding = 1;
  return;
}

/**impeghe_hoa_mtrx_find_sym_spks
 *
 *  \brief Find symmetric speakers
 *
 *  \param [in] output_cnt Output number of channels
 *  \param [out] out_cfg output speaker configuration
 *  \param [in] has_lfe_rendering flag that indicates if LFE rendering is present
 *
 *  \return WORD32 Number of symmetric speakers
 */
static WORD32 impeghe_hoa_mtrx_find_sym_spks(WORD32 output_cnt,
                                             ia_dmx_speaker_information_struct *out_cfg,
                                             WORD32 has_lfe_rendering)
{
  WORD32 i, j;
  WORD32 num_pairs = 0;
  for (i = 0; i < output_cnt; ++i)
  {
    out_cfg[i].original_position = (WORD16)i;
    out_cfg[i].is_already_used = 0;
    out_cfg[i].pstr_symmetric_pair = NULL;
  }

  for (i = 0; i < output_cnt; ++i)
  {
    if (out_cfg[i].is_already_used)
      continue;

    if ((out_cfg[i].azimuth == 0) || (abs(out_cfg[i].azimuth) == 180))
    {
      out_cfg[i].pstr_symmetric_pair = NULL;
      out_cfg[i].pair_type = SP_PAIR_CENTER;

      out_cfg[i].is_already_used = 1;
    }
    else
    {
      for (j = i + 1; j < output_cnt; ++j)
      {
        if (out_cfg[j].is_already_used)
          continue;

        if ((out_cfg[i].is_lfe == out_cfg[j].is_lfe) &&
            (out_cfg[i].elevation == out_cfg[j].elevation) &&
            (out_cfg[i].azimuth == -out_cfg[j].azimuth))
        {

          out_cfg[i].pstr_symmetric_pair = &(out_cfg[j]);
          out_cfg[i].pair_type = SP_PAIR_SYMMETRIC;
          if (0 == (out_cfg[i].is_lfe & !has_lfe_rendering))
          {
            num_pairs++;
          }
          out_cfg[j].pstr_symmetric_pair = NULL;
          out_cfg[j].pair_type = SP_PAIR_NONE;

          out_cfg[i].is_already_used = 1;
          out_cfg[j].is_already_used = 1;
          break;
        }
      }

      if (!out_cfg[i].is_already_used)
      {
        out_cfg[i].pstr_symmetric_pair = NULL;
        out_cfg[i].pair_type = SP_PAIR_SINGLE;

        out_cfg[i].is_already_used = 1;
      }
    }
  }

  for (i = 0; i < output_cnt; ++i)
  {
    out_cfg[i].is_already_used = 0;
  }
  return num_pairs;
}

/**impeghe_hoa_mtrx_create_sym_signs
 *
 *  \brief Create symmetric signs
 *
 *  \param [out] ptr_sym_sign array of symmetric signs
 *  \param [in] hoa_order HOA order
 *
 *  \return VOID
 */
static VOID impeghe_hoa_mtrx_create_sym_signs(WORD32 *ptr_sym_sign, WORD32 hoa_order)
{
  WORD32 n, m, k = 0;
  for (n = 0; n <= hoa_order; ++n)
  {
    for (m = -n; m <= n; ++m)
      ptr_sym_sign[k++] = ((m >= 0) * 2) - 1;
  }
  return;
}

/**impeghe_hoa_mtrx_create_2d_bit_mask
 *
 *  \brief Create 2 dimensional bit-mask matrix
 *
 *  \param [out] ptr_bitmask bit-mask matrix
 *  \param [in] hoa_order HOA order
 *
 *  \return VOID
 */
static VOID impeghe_hoa_mtrx_create_2d_bit_mask(WORD32 *ptr_bitmask, WORD32 hoa_order)
{
  WORD32 n, m, k = 0;
  ptr_bitmask[k++] = 0;
  for (n = 1; n <= hoa_order; ++n)
  {
    for (m = -n; m <= n; ++m)
      ptr_bitmask[k++] = abs(m) != n;
  }
  return;
}

/**impeghe_hoa_mtrx_integer_log2
 *
 *  \brief Compute log 2 of integer value
 *
 *  \param [in] n input value
 *
 *  \return WORD32 log 2 of input value
 */
static WORD32 impeghe_hoa_mtrx_integer_log2(UWORD32 n)
{
  WORD32 result = 0;

  while (n > 1)
  {
    n >>= 1;
    ++result;
  }

  return result;
}

/**impeghe_hoa_mtrx_write_range
 *
 *  \brief Determine range for a given alphabet
 *
 *  \param [out] bit_buffer bit buffer
 *  \param [in] value input value
 *  \param [in] alphabet_sz alphabet size
 *  \param [out] bit_cnt number of bits written to bit buffer
 *
 *  \return VOID
 */
static VOID impeghe_hoa_mtrx_write_range(ia_bit_buf_struct *it_bit_buf, UWORD32 value,
                                         UWORD32 alphabet_sz, WORD32 *bit_cnt)
{

  WORD32 bits = impeghe_hoa_mtrx_integer_log2(alphabet_sz);
  UWORD32 unused = (1 << (bits + 1)) - alphabet_sz;
  if (value < unused)
  {
    *bit_cnt += impeghe_write_bits_buf(it_bit_buf, value, bits);
  }
  else
  {
    value += unused;
    *bit_cnt += impeghe_write_bits_buf(it_bit_buf, value >> 1, bits);
    *bit_cnt += impeghe_write_bits_buf(it_bit_buf, value & 1, 1);
  }
}

/**impeghe_hoa_mtrx_encode_gain_val
 *
 *  \brief Encode gain value
 *
 *  \param [out] it_bit_buff bit buffer
 *  \param [in] pstr_hoa_state HOA matrix state
 *  \param [in] gain gain value
 *  \param [in] order HOA order
 *  \param [out] bit_cnt number of bits written to bit buffer
 *
 *  \return VOID
 */
static VOID impeghe_hoa_mtrx_encode_gain_val(ia_bit_buf_struct *it_bit_buff,
                                             impeghe_hoa_matrix_state_str *pstr_hoa_state,
                                             FLOAT32 gain, WORD32 order, WORD32 *bit_cnt)
{
  WORD32 num_val = ((pstr_hoa_state->max_gain[order] - pstr_hoa_state->min_gain[order])
                    << pstr_hoa_state->prec_lvl) +
                   2;
  WORD32 gain_idx;

  if (gain == HOA_MATRIX_GAIN_ZERO)
  {
    gain_idx = num_val - 1; /* the largest gain index indicates HOA_MATRIX_GAIN_ZERO */
  }
  else
  {
    gain_idx = (WORD32)((pstr_hoa_state->max_gain[order] - gain) *
                        (FLOAT32)(1 << pstr_hoa_state->prec_lvl));
  }

  impeghe_hoa_mtrx_write_range(it_bit_buff, gain_idx, num_val, bit_cnt);
}

/**impeghe_hoa_matrix_encode
 *
 *  \brief Encode HOA matrix
 *
 *  \param [in] input_cnt Input number of channels for HOA matrix
 *  \param [in] output_cnt output number of channels for HOA matrix
 *  \param [in] pstr_out_cfg Output speaker configuration
 *  \param [in] prec_lvl Precision level
 *  \param [out] it_bit_buff bit buffer
 *  \param [in] hoa_matrix HOA rendering matrix
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_hoa_matrix_encode(WORD32 input_cnt, WORD32 output_cnt,
                                       ia_dmx_speaker_information_struct *pstr_out_cfg,
                                       WORD32 prec_lvl, ia_bit_buf_struct *it_bit_buff,
                                       pFlOAT64 hoa_matrix, pVOID scratch)
{
  WORD32 max_hoa_order = (WORD32)sqrt((FLOAT32)input_cnt) - 1;
  WORD32 gain_limit_per_order = 1;
  WORD32 is_full_mtrx = 1;
  WORD32 first_sparse_order = max_hoa_order;
  WORD32 is_normalized = 0;
  WORD32 i, j, k, sign1, sign2;
  FLOAT64 val1, val2;
  WORD32 val_symmetric, sign_symmetric;
  WORD32 bits_hoa_order = impeghe_hoa_get_ceil_log2(max_hoa_order + 1);
  WORD32 curr_hoa_order = 0;
  FLOAT64 curr_scalar;
  UWORD8 *ptr_scratch = scratch;
  WORD32 *is_hoa_coef_sparse = (WORD32 *)ptr_scratch;
  ptr_scratch += HOA_MATRIX_MAX_HOA_COEF_COUNT * sizeof(is_hoa_coef_sparse[0]);
  WORD32 has_value = 1;
  WORD32 lfe_exist = 0;
  WORD32 has_lfe_rendering = 0;
  WORD32 is_zero_order_positive = 1;
  WORD32 num_pairs = 0;
  WORD32 is_sign_symmetric[HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 is_value_symmetric[HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 value_symmetric_pairs[HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 sign_symmetric_pairs[HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 pair_idx = 0;
  WORD32 *sign_matrix = (WORD32 *)ptr_scratch;
  ptr_scratch +=
      (HOA_MATRIX_MAX_HOA_COEF_COUNT * HOA_MATRIX_MAX_SPEAKER_COUNT) * sizeof(sign_matrix[0]);
  WORD32 is_any_sign_symmetric = 0;
  WORD32 is_all_sign_symmetric = 0;
  WORD32 is_any_value_symmetric = 0;
  WORD32 is_all_value_symmetric = 0;
  impeghe_hoa_matrix_state_str state;
  WORD32 error = IA_NO_ERROR;

  FLOAT32 multiplier = (FLOAT32)(1 << prec_lvl);
  FLOAT64 inv_multiplier = (1.0 / multiplier);
  FLOAT64 gain;
  FLOAT64 min_gain = 0.0;
  FLOAT64 max_gain = -60.0;
  WORD32 bit_cnt = 0;
  WORD32 num_val_sym_pairs = 0;
  WORD32 num_sign_sym_pairs = 0;
  WORD32 has_vertical_coef = 0;
  WORD32 *ptr_sym_sign = (WORD32 *)ptr_scratch;
  ptr_scratch += HOA_MATRIX_MAX_HOA_COEF_COUNT * sizeof(ptr_sym_sign[0]);
  WORD32 *ptr_vert_bitmask = (WORD32 *)ptr_scratch;
  ptr_scratch += HOA_MATRIX_MAX_HOA_COEF_COUNT * sizeof(ptr_vert_bitmask[0]);
  impeghe_hoa_mtrx_create_sym_signs(&ptr_sym_sign[0], max_hoa_order);
  impeghe_hoa_mtrx_create_2d_bit_mask(&ptr_vert_bitmask[0], max_hoa_order);

  if (input_cnt > output_cnt)
  {
    WORD32 eq_order = (WORD32)ceil(sqrt((FLOAT32)output_cnt));
    if (max_hoa_order >= eq_order)
    {
      is_full_mtrx = 0;
      first_sparse_order = eq_order;
    }
  }

  curr_scalar = 0.0;
  for (i = 0; i < input_cnt; ++i)
  {
    for (j = 0; j < output_cnt; ++j)
    {
      if (!pstr_out_cfg[j].is_lfe)
        curr_scalar += hoa_matrix[i * output_cnt + j] * hoa_matrix[i * output_cnt + j];
    }
  }
  curr_scalar = fabs(1.0 - sqrt(curr_scalar));
  if (curr_scalar < (1e-6))
  {
    is_normalized = 1;
  }

  impeghe_hoa_mtrx_state_init(&state, (FLOAT32)max_gain + 1.0f);

  for (i = 0; i < input_cnt; ++i)
  {
    curr_hoa_order = (WORD32)ceil(sqrt(i + 1.0f) - 1);
    curr_scalar = sqrt(2 * curr_hoa_order + 1.0);
    for (j = 0; j < output_cnt; ++j)
    {
      FLOAT64 value = hoa_matrix[i * output_cnt + j];
      value *= curr_scalar;
      if (curr_hoa_order == 0)
      {
        if (value < 0.0)
          is_zero_order_positive = 0;
      }
      if (value != 0.0)
      {
        gain = (20.0 * log10(fabs(value)));
        sign_matrix[i * output_cnt + j] = (value > 0.0);
        gain = (floor(gain * multiplier + 0.5) * inv_multiplier);
        gain = MIN(gain, min_gain);
        if (gain >= max_gain)
        {
          hoa_matrix[i * output_cnt + j] = gain;
        }
        else
        {
          hoa_matrix[i * output_cnt + j] = HOA_MATRIX_GAIN_ZERO;
        }
        if (ptr_vert_bitmask[i] && (hoa_matrix[i * output_cnt + j] != HOA_MATRIX_GAIN_ZERO))
        {
          has_vertical_coef = 1;
        }

        if ((gain >= max_gain) && (has_vertical_coef || !ptr_vert_bitmask[i]))
        {
          if (gain_limit_per_order)
          {
            state.min_gain[curr_hoa_order] =
                MIN(state.min_gain[curr_hoa_order], (WORD32)floor(gain));
            state.max_gain[curr_hoa_order] =
                MAX(state.max_gain[curr_hoa_order], (WORD32)ceil(gain));
          }
          else
          {
            state.min_gain[0] = MIN(state.min_gain[0], (WORD32)floor(gain));
            state.max_gain[0] = MAX(state.max_gain[0], (WORD32)ceil(gain));
          }
        }
      }
      else
      {
        hoa_matrix[i * output_cnt + j] = HOA_MATRIX_GAIN_ZERO;
        sign_matrix[i * output_cnt + j] = 1;
      }
      if (pstr_out_cfg[j].is_lfe)
      {
        lfe_exist = 1;
        if (hoa_matrix[i * output_cnt + j] != HOA_MATRIX_GAIN_ZERO)
        {
          has_lfe_rendering = 1;
        }
      }
    }
  }

  for (i = curr_hoa_order; i > -1; --i)
  {
    if (state.min_gain[i] > state.max_gain[i])
    {
      first_sparse_order = MIN(first_sparse_order, i);
      state.max_gain[i] = 0;
      state.min_gain[i] = -1;
    }
    else if (state.min_gain[i] == state.max_gain[i])
    {
      state.min_gain[i] -= 1;
    }
  }

  if (!gain_limit_per_order)
  {
    for (i = 1; i < (curr_hoa_order + 1); ++i)
    {
      state.min_gain[i] = state.min_gain[0];
      state.max_gain[i] = state.max_gain[0];
    }
  }

  num_pairs = impeghe_hoa_mtrx_find_sym_spks(output_cnt, pstr_out_cfg, has_lfe_rendering);

  for (k = 0; k < output_cnt; ++k)
  {
    if (pstr_out_cfg[k].is_already_used)
      continue;
    if ((!has_lfe_rendering && pstr_out_cfg[k].is_lfe) ||
        (pstr_out_cfg[k].pstr_symmetric_pair == NULL))
    {
      is_sign_symmetric[k] = 0;
      is_value_symmetric[k] = 0;
      continue;
    }
    val_symmetric = 1;
    sign_symmetric = 1;

    j = pstr_out_cfg[k].pstr_symmetric_pair->original_position;
    for (i = 0; i < input_cnt; ++i)
    {
      val1 = hoa_matrix[i * output_cnt + k];
      val2 = hoa_matrix[i * output_cnt + j];
      sign1 = (2 * sign_matrix[i * output_cnt + k]) - 1;
      sign2 = (2 * sign_matrix[i * output_cnt + j]) - 1;

      if (0 == ((val1 == HOA_MATRIX_GAIN_ZERO) && (val2 == HOA_MATRIX_GAIN_ZERO)))
      {
        if (sign1 * val1 != (sign2 * ptr_sym_sign[i] * val2))
        {
          val_symmetric = 0;
        }
        if (sign1 != (ptr_sym_sign[i] * sign2))
        {
          sign_symmetric = 0;
        }
      }
    }
    is_sign_symmetric[j] = 0;
    is_sign_symmetric[k] = sign_symmetric;
    sign_symmetric_pairs[pair_idx] = sign_symmetric;
    num_sign_sym_pairs += sign_symmetric;
    is_value_symmetric[j] = 0;
    is_value_symmetric[k] = val_symmetric;
    value_symmetric_pairs[pair_idx] = val_symmetric;
    num_val_sym_pairs += val_symmetric;
    pair_idx++;
    pstr_out_cfg[j].is_already_used = 1;
  }

  if (num_val_sym_pairs == num_pairs)
  {
    is_all_value_symmetric = 1;
    is_all_sign_symmetric = 0;
    is_any_sign_symmetric = 0;
    num_sign_sym_pairs = 0;
  }
  else if (num_val_sym_pairs)
  {
    is_any_value_symmetric = 1;
    for (i = 0; i < num_pairs; ++i)
    {
      if (sign_symmetric_pairs[i] && value_symmetric_pairs[i])
      {
        sign_symmetric_pairs[i] = 0;
        num_sign_sym_pairs--;
      }
    }
  }
  else
  {
    is_any_value_symmetric = 0;
  }

  is_all_sign_symmetric = (num_sign_sym_pairs == num_pairs);
  is_any_sign_symmetric = num_sign_sym_pairs > 0;

  if (0 == is_full_mtrx)
  {
    for (i = 0; i < (first_sparse_order * first_sparse_order); ++i)
    {
      is_hoa_coef_sparse[i] = 0;
    }
    for (; i < input_cnt; ++i)
    {
      is_hoa_coef_sparse[i] = 1;
    }
  }
  else
  {
    for (i = 0; i < input_cnt; ++i)
    {
      is_hoa_coef_sparse[i] = 0;
    }
  }

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, prec_lvl, 2);
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_normalized, 1);
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, gain_limit_per_order, 1);
  if (1 == gain_limit_per_order)
  {
    for (i = 0; i < (1 + curr_hoa_order); ++i)
    {
      bit_cnt += impeghe_write_escape_value(it_bit_buff, (UWORD32)-state.max_gain[i], 3, 5, 6);
      bit_cnt += impeghe_write_escape_value(
          it_bit_buff, (UWORD32)(-state.min_gain[i] - 1 + state.max_gain[i]), 4, 5, 6);
    }
  }
  else
  {
    bit_cnt += impeghe_write_escape_value(it_bit_buff, (UWORD32)-state.max_gain[0], 3, 5, 6);
    bit_cnt += impeghe_write_escape_value(
        it_bit_buff, (UWORD32)(-state.min_gain[0] - 1 + state.max_gain[0]), 4, 5, 6);
  }
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_full_mtrx, 1);
  if (0 == is_full_mtrx)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, first_sparse_order, bits_hoa_order);
  }
  if (lfe_exist)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, has_lfe_rendering, 1);
  }
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_zero_order_positive, 1);
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_all_value_symmetric, 1);
  if (0 == is_all_value_symmetric)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_any_value_symmetric, 1);
    if (is_any_value_symmetric)
    {
      for (i = 0; i < num_pairs; ++i)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, value_symmetric_pairs[i], 1);
      }
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_any_sign_symmetric, 1);
      if (is_any_sign_symmetric)
      {
        if (is_any_value_symmetric)
        {
          for (i = 0; i < num_pairs; ++i)
          {
            if (0 == value_symmetric_pairs[i])
            {
              bit_cnt += impeghe_write_bits_buf(it_bit_buff, sign_symmetric_pairs[i], 1);
            }
          }
        }
        else
        {
          for (i = 0; i < num_pairs; ++i)
          {
            bit_cnt += impeghe_write_bits_buf(it_bit_buff, sign_symmetric_pairs[i], 1);
          }
        }
      }
    }
    else
    {
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_all_sign_symmetric, 1);
      if (0 == is_all_sign_symmetric)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, is_any_sign_symmetric, 1);
        if (is_any_sign_symmetric)
        {
          for (i = 0; i < num_pairs; ++i)
          {
            bit_cnt += impeghe_write_bits_buf(it_bit_buff, sign_symmetric_pairs[i], 1);
          }
        }
      }
    }
  }

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, has_vertical_coef, 1);
  state.prec_lvl = prec_lvl;

  for (i = 0; i < input_cnt; ++i)
  {
    if ((ptr_vert_bitmask[i] && has_vertical_coef) || !ptr_vert_bitmask[i])
    {
      curr_hoa_order = (WORD32)ceil(sqrt(i + 1.0f) - 1);
      for (j = output_cnt - 1; j >= 0; --j)
      {
        FLOAT32 value = (FLOAT32)hoa_matrix[i * output_cnt + j];
        has_value = 1;
        if ((has_lfe_rendering && pstr_out_cfg[j].is_lfe) || (!pstr_out_cfg[j].is_lfe))
        {

          if (0 == is_value_symmetric[j])
          {
            if (is_hoa_coef_sparse[i])
            {
              has_value = (value != HOA_MATRIX_GAIN_ZERO);
              bit_cnt += impeghe_write_bits_buf(it_bit_buff, has_value, 1);
            }
          }
          else
          {
            has_value = 0;
          }
          if (has_value)
          {
            impeghe_hoa_mtrx_encode_gain_val(it_bit_buff, &state, value, curr_hoa_order,
                                             &bit_cnt);
            if (0 == is_sign_symmetric[j])
            {
              if (value != HOA_MATRIX_GAIN_ZERO)
              {
                if (curr_hoa_order || !is_zero_order_positive)
                {
                  bit_cnt +=
                      impeghe_write_bits_buf(it_bit_buff, sign_matrix[i * output_cnt + j], 1);
                }
              }
            }
          }
        }
      }
    }
  }

  return error;
}
