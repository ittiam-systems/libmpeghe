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
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_cnst.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_psy_mod.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_rom.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_lpc_eval_chebyshev_polyn
 *
 *  \brief Interpolates value using chebyshev polynomial.
 *
 *  \param [in] x     Pointer to input data.
 *  \param [in] coefs Pointer to coefficient data.
 *  \param [in] order Order of coefficients.
 *
 *  \return Interpolated value.
 */
static FLOAT32 impeghe_lpc_eval_chebyshev_polyn(FLOAT32 x, FLOAT32 *coefs, WORD32 order)
{
  LOOPIDX i;
  FLOAT32 b0, b1, b2, x2;
  x2 = 2.0f * x;
  b1 = x2 + coefs[1];
  b2 = 1.0f;
  for (i = 2; i < order; i++)
  {
    b0 = x2 * b1 - b2 + coefs[i];
    b2 = b1;
    b1 = b0;
  }
  return (x * b1 - b2 + 0.5f * coefs[order]);
}

/**
 *  impeghe_lpc_2_lsp_conversion
 *
 *  \brief Converts linear pred. coeffs to line spectral pairs.
 *
 *  \param [in ] lpc         Pointer to LPC data.
 *  \param [out] lsp         Pointer to LSP data.
 *  \param [in,out] prev_lsp Pointer to previous LSP data.
 *
 *  \return VOID
 */
VOID impeghe_lpc_2_lsp_conversion(FLOAT32 *lpc, FLOAT32 *lsp, FLOAT32 *prev_lsp)
{
  LOOPIDX i;
  WORD32 point, num_found_freeq, is_first_polyn;
  FLOAT32 x_low, y_low, x_high, y_high, x_mid, y_mid, x_lin_interp;
  FLOAT32 sum_polyn[(ORDER_BY_2) + 1], diff_polyn[(ORDER_BY_2) + 1];
  FLOAT32 *p1_lpc, *p2_lpc, *p_sum_polyn, *p_diff_polyn;

  p_sum_polyn = sum_polyn;
  p_diff_polyn = diff_polyn;
  point = 0;
  num_found_freeq = 0;
  is_first_polyn = 0;
  *p_sum_polyn++ = 1.0f;
  *p_diff_polyn++ = 1.0f;
  sum_polyn[0] = 1.0f;
  diff_polyn[0] = 1.0f;

  p1_lpc = lpc + 1;
  p2_lpc = lpc + ORDER;
  for (i = 0; i <= ORDER_BY_2 - 1; i++)
  {
    *p_sum_polyn = *p1_lpc + *p2_lpc - *(p_sum_polyn - 1);
    p_sum_polyn++;
    *p_diff_polyn = *p1_lpc++ - *p2_lpc-- + *(p_diff_polyn - 1);
    p_diff_polyn++;
  }
  x_low = impeghe_chebyshev_polyn_grid[0];
  p_sum_polyn = sum_polyn;
  y_low = impeghe_lpc_eval_chebyshev_polyn(x_low, p_sum_polyn, ORDER_BY_2);

  while ((num_found_freeq < ORDER) && (point < CHEBYSHEV_NUM_POINTS))
  {
    point++;
    x_high = x_low;
    y_high = y_low;
    x_low = impeghe_chebyshev_polyn_grid[point];
    y_low = impeghe_lpc_eval_chebyshev_polyn(x_low, p_sum_polyn, ORDER_BY_2);

    if (y_low * y_high <= 0.0) /* if sign change new root exists */
    {
      point--;
      for (i = 0; i < CHEBYSHEV_NUM_ITER; i++)
      {
        x_mid = 0.5f * (x_low + x_high);
        y_mid = impeghe_lpc_eval_chebyshev_polyn(x_mid, p_sum_polyn, ORDER_BY_2);
        if (y_low * y_mid > 0.0)
        {
          y_low = y_mid;
          x_low = x_mid;
        }
        else
        {
          y_high = y_mid;
          x_high = x_mid;
        }
      }

      /* linear interpolation for evaluating the root */
      x_lin_interp = x_low - y_low * (x_high - x_low) / (y_high - y_low);

      lsp[num_found_freeq] = x_lin_interp;
      num_found_freeq++;

      is_first_polyn = 1 - is_first_polyn;
      p_sum_polyn = is_first_polyn ? diff_polyn : sum_polyn;

      x_low = x_lin_interp;
      y_low = impeghe_lpc_eval_chebyshev_polyn(x_low, p_sum_polyn, ORDER_BY_2);
    }
  }

  /* Check if ORDER roots found */
  /* if not use the LSPs from previous frame */
  if (num_found_freeq < ORDER)
  {
    for (i = 0; i < ORDER; i++)
    {
      lsp[i] = prev_lsp[i];
    }
  }
}

/**
 *  impeghe_compute_coeff_poly_f
 *
 *  \brief Computes line spectral pairs polynomials.
 *
 *  \param [in ] lsp   Pointer to line spectral pairs.
 *  \param [out] poly1 Pointer to polynomial 1.
 *  \param [out] poly2 Pointer to polynomial 2.
 *
 *  \return VOID
 */
static VOID impeghe_compute_coeff_poly_f(FLOAT32 *lsp, FLOAT32 *poly1, FLOAT32 *poly2)
{
  LOOPIDX i, j;
  FLOAT32 b1, b2;
  FLOAT32 *ptr_lsp;

  poly1[0] = poly2[0] = 1.0f;
  ptr_lsp = lsp;

  for (i = 1; i <= ORDER_BY_2; i++)
  {
    b1 = -2.0f * (*ptr_lsp++);
    b2 = -2.0f * (*ptr_lsp++);
    poly1[i] = (b1 * poly1[i - 1]) + (2.0f * poly1[i - 2]);
    poly2[i] = (b2 * poly2[i - 1]) + (2.0f * poly2[i - 2]);
    for (j = i - 1; j > 0; j--)
    {
      poly1[j] += (b1 * poly1[j - 1]) + poly1[j - 2];
      poly2[j] += (b2 * poly2[j - 1]) + poly2[j - 2];
    }
  }
}

/**
 *  impeghe_lsp_to_lp_conversion
 *
 *  \brief Computes linear prediction coeffs from line spectral pairs.
 *
 *  \param [in ] lsp           Pointer to line spectral pairs data.
 *  \param [out] lp_flt_coff_a Pointer to linear prediction coefficients.
 *
 *  \return VOID
 */
VOID impeghe_lsp_to_lp_conversion(FLOAT32 *lsp, FLOAT32 *lp_flt_coff_a)
{
  LOOPIDX i;
  FLOAT32 *ppoly_f1, *ppoly_f2;
  FLOAT32 *plp_flt_coff_a_bott, *plp_flt_coff_a_top;
  FLOAT32 poly1[ORDER_BY_2 + 2], poly2[ORDER_BY_2 + 2];

  poly1[0] = 0.0f;
  poly2[0] = 0.0f;

  impeghe_compute_coeff_poly_f(lsp, &poly1[1], &poly2[1]);

  ppoly_f1 = poly1 + ORDER_BY_2 + 1;
  ppoly_f2 = poly2 + ORDER_BY_2 + 1;

  for (i = 0; i < ORDER_BY_2; i++)
  {
    ppoly_f1[0] += ppoly_f1[-1];
    ppoly_f2[0] -= ppoly_f2[-1];
    ppoly_f1--;
    ppoly_f2--;
  }

  plp_flt_coff_a_bott = lp_flt_coff_a;
  *plp_flt_coff_a_bott++ = 1.0f;
  plp_flt_coff_a_top = lp_flt_coff_a + ORDER;
  ppoly_f1 = poly1 + 2;
  ppoly_f2 = poly2 + 2;
  for (i = 0; i < ORDER_BY_2; i++)
  {
    *plp_flt_coff_a_bott++ = 0.5f * (*ppoly_f1 + *ppoly_f2);
    *plp_flt_coff_a_top-- = 0.5f * (*ppoly_f1++ - *ppoly_f2++);
  }
}

/**
 *  impeghe_levinson_durbin_algo
 *
 *  \brief Levinson durbin implementation for finding LPC.
 *
 *  \param [in ] auto_corr_input Pointer to auto correlation data.
 *  \param [out] lpc             Pointer to computed LPC data.
 *
 *  \return VOID
 */
VOID impeghe_levinson_durbin_algo(FLOAT32 *auto_corr_input, FLOAT32 *lpc)
{
  LOOPIDX i, j;
  FLOAT32 lpc_val, sum, sigma;
  FLOAT32 reflection_coeffs[LEV_DUR_MAX_ORDER];

  lpc[0] = 1.0f;

  reflection_coeffs[0] = -auto_corr_input[1] / auto_corr_input[0];
  lpc[1] = reflection_coeffs[0];
  sigma = auto_corr_input[0] + auto_corr_input[1] * reflection_coeffs[0];

  for (i = 2; i <= ORDER; i++)
  {
    sum = 0.0f;
    for (j = 0; j < i; j++)
      sum += auto_corr_input[i - j] * lpc[j];
    reflection_coeffs[i - 1] = -sum / sigma;

    sigma = sigma * (1.0f - reflection_coeffs[i - 1] * reflection_coeffs[i - 1]);

    if (sigma <= 1.0E-09f)
    {
      sigma = 1.0E-09f;
      for (j = i; j <= ORDER; j++)
      {
        reflection_coeffs[j - 1] = 0.0f;
        lpc[j] = 0.0f;
      }
      break;
    }

    for (j = 1; j <= (i / 2); j++)
    {
      lpc_val = lpc[j] + reflection_coeffs[i - 1] * lpc[i - j];
      lpc[i - j] += reflection_coeffs[i - 1] * lpc[j];
      lpc[j] = lpc_val;
    }

    lpc[i] = reflection_coeffs[i - 1];
  }
}

/**
 *  impeghe_get_weighted_lpc
 *
 *  \brief Multiplies LPC data with weights provided.
 *
 *  \param [in ] lpc          Pointer to input LPC data.
 *  \param [out] weighted_lpc Pointer to weighted LPC data.
 *
 *  \return VOID
 */

VOID impeghe_get_weighted_lpc(FLOAT32 *lpc, FLOAT32 *weighted_lpc)
{
  LOOPIDX idx;
  for (idx = 0; idx <= ORDER; idx++)
  {
    weighted_lpc[idx] = impeghe_gamma_table[idx] * lpc[idx];
  }
}

/**
 *  impeghe_lsp_2_lsf_conversion
 *
 *  \brief Converts line spectral frequencies to line spectral pairs.
 *
 *  \param [in ] lsp Pointer to LSP data.
 *  \param [out] lsf Pointer to LSF data.
 *
 *  \return VOID
 */
VOID impeghe_lsp_2_lsf_conversion(FLOAT32 *lsp, FLOAT32 *lsf)
{
  LOOPIDX idx;
  for (idx = 0; idx < ORDER; idx++)
  {
    lsf[idx] = (FLOAT32)(acos(lsp[idx]) * LSP_2_LSF_SCALE);
  }
}

/**
 *  impeghe_lsf_2_lsp_conversion
 *
 *  \brief Converts line spectral frequencies to line spectral pairs.
 *
 *  \param [in ] lsf Pointer to LSF data.
 *  \param [out] lsp Pointer to LSP data.
 *
 *  \return VOID
 */
VOID impeghe_lsf_2_lsp_conversion(FLOAT32 *lsf, FLOAT32 *lsp)
{
  LOOPIDX idx;
  for (idx = 0; idx < ORDER; idx++)
  {
    lsp[idx] = (FLOAT32)cos((FLOAT64)lsf[idx] * (FLOAT64)PI_BY_6400);
  }
}
/** @} */ /* End of CoreEncProc */