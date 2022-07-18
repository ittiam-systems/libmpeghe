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

#include <math.h>
#include <string.h>
#include "impeghe_type_def.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_hoa_common_values.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_hoa_assignment_info.h"

/**
 *  impeghe_hoa_compute_max_abs_val
 *
 *  \brief Find the maximum absolute value in an array
 *
 *  \param [in] ptr_input Input array
 *  \param [in] vec_len Array size
 *  \param [out] max_abs_val Maximum absolute value
 *  \param [Out] max_abs_index
 *
 *  \return VOID
 */
VOID impeghe_hoa_compute_max_abs_val(const pFlOAT64 ptr_input, const UWORD32 vec_len,
                                     pFlOAT64 max_abs_val, pUWORD32 max_abs_index)
{
  *max_abs_index = 0;
  *max_abs_val = ABS(ptr_input[*max_abs_index]);

  for (UWORD32 elem_idx = 1; elem_idx < vec_len; elem_idx++)
  {
    FLOAT64 curr_abs_val = ABS(ptr_input[elem_idx]);

    if (curr_abs_val > *max_abs_val)
    {
      *max_abs_index = elem_idx;
      *max_abs_val = curr_abs_val;
    }
  }
  return;
}

/**
 *  impeghe_hoa_compute_fade_win_for_vec_based_syn
 *
 *  \brief Compute fading window for vector based sount synthesis
 *
 *  \param [out] ptr_out output window
 *  \param [in] frame_sz frame size
 *  \param [in] interp_samples Number of interpolation samples
 *  \param [in] interp_method Interpolation method
 *
 *  \return VOID
 */
VOID impeghe_hoa_compute_fade_win_for_vec_based_syn(pFlOAT64 ptr_out, UWORD32 frame_sz,
                                                    UWORD32 interp_samples, UWORD32 interp_method)
{
  UWORD32 sample;

  memset(ptr_out, 0, sizeof(FLOAT64) * (2 * frame_sz));

  if (0 == interp_samples)
  {
    for (sample = 0; sample < frame_sz; sample++)
    {
      ptr_out[sample] = (FLOAT64)(1.0);
    }
  }
  else
  {
    for (sample = interp_samples; sample < frame_sz; sample++)
    {
      ptr_out[sample] = (FLOAT64)(1.0);
    }
    if (0 == interp_method)
    {
      for (sample = 0; sample < interp_samples; sample++)
      {
        ptr_out[sample] = (FLOAT64)(sample) / (FLOAT64)(interp_samples - 1);
        ptr_out[frame_sz + sample] = (FLOAT64)(1.0) - ptr_out[sample];
      }
    }
    else if (1 == interp_method)
    {
      for (sample = 0; sample < interp_samples; sample++)
      {
        ptr_out[sample] = (FLOAT64)(
            0.5 * (1 - cos(__M_PI__ * ((FLOAT64)(sample)) / ((FLOAT64)(interp_samples)))));
        ptr_out[frame_sz + sample] = (FLOAT64)(
            0.5 * (1 + cos(__M_PI__ * ((FLOAT64)(sample)) / ((FLOAT64)(interp_samples)))));
      }
    }
  }
  return;
}

/**
 *  impeghe_hoa_quantize_uniform
 *
 *  \brief Uniformly quantize input
 *
 *  \param [in] input_val Input value
 *  \param [in] num_bits Number of bits available for quantization
 *
 *  \return UWORD32 Quantized value
 */
UWORD32 impeghe_hoa_quantize_uniform(FLOAT64 input_val, UWORD32 num_bits)
{
  UWORD32 out_quant_val =
      (UWORD32)(MIN((UWORD32)(floor(0.5 + ((FLOAT64)(input_val) + 1.0) *
                                              (FLOAT64)(impeghe_hoa_get_pow2(num_bits - 1)))),
                    (UWORD32)(impeghe_hoa_get_pow2(num_bits) - 1)));
  return out_quant_val;
}

/**
 *  impeghe_hoa_get_pow2
 *
 *  \brief Compute power of 2 for positive exponents
 *
 *  \param [in] exponent value of exponent
 *
 *  \return UWORD32 2^exp
 */
UWORD32 impeghe_hoa_get_pow2(const UWORD32 exponent)
{
  UWORD32 pow_result = 1;

  for (UWORD32 exp_idx = 0; exp_idx < exponent; exp_idx++)
  {
    pow_result <<= 1;
  }

  return pow_result;
}

/**
 *  impeghe_hoa_get_ceil_log2
 *
 *  \brief Compute log 2 of a given value
 *
 *  \param [in] x Input value
 *
 *  \return UWORD32 log 2 of a given value
 */
UWORD32 impeghe_hoa_get_ceil_log2(const UWORD32 x)
{
  UWORD32 tmp = 1;
  UWORD32 n = 0;
  if (x == 0)
    return 0;
  while ((tmp << (n)) < x)
  {
    n++;
  }
  return n;
};

/**
 *  impeghe_hoa_assignment_info_init
 *
 *  \brief Initialize HOA assignment information structure
 *
 *  \param [in,out] pstr_hoa assignment information handle
 *
 *  \return VOID
 */
VOID impeghe_hoa_assignment_info_init(ia_spatial_enc_assignment_info_str *pstr_hoa)
{
  pstr_hoa->is_available = 1;
  pstr_hoa->bit_rate = 0;
  pstr_hoa->ch_type = HOA_EMPTY_CHANNEL;

  pstr_hoa->vec_based_ch_info.us_bits_q = 99;
  pstr_hoa->vec_based_ch_info.p_flag = 0;
  pstr_hoa->vec_based_ch_info.cb_flag = 0;
  pstr_hoa->vec_based_ch_info.same_header_prev_frame = 0;
  pstr_hoa->vec_based_ch_info.new_ch_type_one = 0;
  pstr_hoa->vec_based_ch_info.v_vec_dir = 8;
  pstr_hoa->vec_based_ch_info.max_v_vec_dir = 8;
  pstr_hoa->vec_based_ch_info.indices_code_book_idx = 0;
  pstr_hoa->vec_based_ch_info.weighting_code_book_idx = 0;

  pstr_hoa->ch_info.amb_coeff_idx_changed = 1;
  pstr_hoa->ch_info.amb_coeff_idx_transition_state = 2;
  pstr_hoa->ch_info.amb_coeff_idx = 0;
  return;
}

/**
 *  impeghe_hoa_compute_pseudo_inverse_vec
 *
 *  \brief Compute pseudo-inverse of a vector
 *
 *  \param [in] ptr_inp Input vector
 *  \param [in] input_vec_size Size of input vector
 *  \param [in] ptr_out_pseudo_inv_vec pseudo-inverse of input vector
 *
 *  \return VOID
 */
VOID impeghe_hoa_compute_pseudo_inverse_vec(const pFlOAT64 ptr_inp, const UWORD32 input_vec_size,
                                            pFlOAT64 ptr_out_pseudo_inv_vec)
{
  ULOOPIDX i;
  FLOAT64 tmp_power = (FLOAT64)(0.0);

  for (i = 0; i < input_vec_size; i++)
  {
    tmp_power += ptr_inp[i] * ptr_inp[i];
  }

  for (i = 0; i < input_vec_size; i++)
  {
    ptr_out_pseudo_inv_vec[i] = ptr_inp[i] / tmp_power;
  }
  return;
}
