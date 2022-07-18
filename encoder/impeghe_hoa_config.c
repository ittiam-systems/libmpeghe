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

#include <impeghe_type_def.h>
#include "impeghe_hoa_common_values.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_hoa_config_struct.h"

/**
 *  impeghe_hoa_v_vec_len_and_coeff_calc
 *
 *  \brief Compute HOA vector length
 *
 *  \param [in,out] pstr_config_data HOA configuration structure
 *
 *  \return VOID
 */

static VOID impeghe_hoa_v_vec_len_and_coeff_calc(ia_hoa_config_struct *pstr_config_data)
{
  switch (pstr_config_data->coded_v_vec_length)
  {
  case 1:
  case 2:
    pstr_config_data->idx_offset =
        (pstr_config_data->min_amb_order + 1) * (pstr_config_data->min_amb_order + 1) + 1;
    break;
  case 0:
  default:
    pstr_config_data->idx_offset = 1;
  }
  pstr_config_data->v_vec_length_used =
      pstr_config_data->num_coeffs - pstr_config_data->idx_offset + 1;
  pstr_config_data->index_length = impeghe_hoa_get_ceil_log2(pstr_config_data->num_coeffs);
  return;
}
/**
 *  impeghe_fill_config
 *
 *  \brief Create HOA configuration bit-stream
 *
 *  \param [out] it_bit_buf Pointer to bit buffer
 *  \param [in] pstr_config_data HOA configuration structure
 *
 *  \return WORD32 Number of bits written
 */
WORD32 impeghe_fill_config(ia_bit_buf_struct *it_bit_buf, ia_hoa_config_struct *pstr_config_data)
{
  WORD32 val, i, index;
  WORD32 num_bits = 0;

  /* HOA order*/
  if (pstr_config_data->order > 7)
  {
    num_bits += impeghe_write_bits_buf(it_bit_buf, 7, 3);
    num_bits += impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->order - 7), 5);
  }
  else
    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->order, 3);

  pstr_config_data->num_coeffs = (pstr_config_data->order + 1) * (pstr_config_data->order + 1);

  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->is_screen_relative, 1);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->uses_nfc, 1);
  if (pstr_config_data->uses_nfc)
  {
    pWORD8 ptr = (pWORD8)(&pstr_config_data->nfc_ref_distance);
    num_bits += impeghe_write_bits_buf(it_bit_buf, ptr[0], 8);
    num_bits += impeghe_write_bits_buf(it_bit_buf, ptr[1], 8);
    num_bits += impeghe_write_bits_buf(it_bit_buf, ptr[2], 8);
    num_bits += impeghe_write_bits_buf(it_bit_buf, ptr[3], 8);
  }

  if ((pstr_config_data->min_amb_order + 1) > 7)
  {
    num_bits += impeghe_write_bits_buf(it_bit_buf, 7, 3);
    num_bits += impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->min_amb_order + 1 - 7), 5);
  }
  else
    num_bits += impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->min_amb_order + 1), 3);

  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->is_single_layer, 1);

  index = 0;
  switch (pstr_config_data->frame_length)
  {
  case 768:
    index = 0;
    break;
  case 1024:
    index = 1;
    break;
  case 2048:
    index = 2;
    break;
  case 4096:
    index = 3;
    break;
  }
  for (i = 0; i < 8; i++)
  {
    if (impeghe_spat_interp_time_code_table[index][i] ==
        pstr_config_data->spat_interpolation_time)
    {
      pstr_config_data->coded_spat_interpolation_time = i;
      break;
    }
  }

  num_bits +=
      impeghe_write_bits_buf(it_bit_buf, pstr_config_data->coded_spat_interpolation_time, 3);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->spat_interpolation_method, 1);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->coded_v_vec_length, 2);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->max_gain_corr_amp_exp, 3);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->frame_length_indicator, 2);

  if (pstr_config_data->min_amb_order < pstr_config_data->order)
  {
    pstr_config_data->diff_order_bits = impeghe_hoa_get_ceil_log2(
        (UWORD32)(pstr_config_data->order - pstr_config_data->min_amb_order + 1));

    pstr_config_data->diff_order =
        pstr_config_data->max_order_to_be_transmitted - pstr_config_data->min_amb_order;
    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->diff_order,
                                       (UWORD8)(pstr_config_data->diff_order_bits));
  }
  else
  {
    pstr_config_data->max_order_to_be_transmitted = pstr_config_data->order;
  }

  pstr_config_data->max_num_of_coeffs_to_be_transmitted =
      (pstr_config_data->max_order_to_be_transmitted + 1) *
      (pstr_config_data->max_order_to_be_transmitted + 1);
  pstr_config_data->max_num_add_active_amb_coeffs =
      (pstr_config_data->max_num_of_coeffs_to_be_transmitted -
       pstr_config_data->min_coeffs_for_amb);

  pstr_config_data->vq_conf_bits = impeghe_hoa_get_ceil_log2(
      impeghe_hoa_get_ceil_log2((UWORD32)(pstr_config_data->num_coeffs + 1)));

  pstr_config_data->num_v_vec_vq_elements_bits =
      impeghe_hoa_get_ceil_log2(pstr_config_data->max_v_vec_dir);
  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->num_v_vec_vq_elements_bits,
                                     (UWORD8)pstr_config_data->vq_conf_bits);
  if (1 == pstr_config_data->min_amb_order)
  {
    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->use_phase_shift_decorr, 1);
  }

  num_bits += impeghe_write_bits_buf(
      it_bit_buf, (pstr_config_data->max_no_of_dir_sigs_for_prediction - 1), 2);

  num_bits +=
      impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->no_of_bits_per_scale_factor - 1), 4);

  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->pred_subbands_idx, 2);

  if (pstr_config_data->pred_subbands_idx < 3)
  {
    pstr_config_data->num_of_pred_subbands =
        impeghe_num_of_pred_subbands_table[pstr_config_data->pred_subbands_idx];
  }
  else
  {
    num_bits +=
        impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->num_of_pred_subbands - 1), 5);
  }

  if (pstr_config_data->num_of_pred_subbands > 0)
  {
    WORD32 first_sbr_subbband_idx_bits =
        impeghe_hoa_get_ceil_log2((UWORD32)(pstr_config_data->num_of_pred_subbands + 1));

    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->first_sbr_subband_idx,
                                       (UWORD8)(first_sbr_subbband_idx_bits));

    val = impeghe_hoa_get_ceil_log2(pstr_config_data->max_num_of_pred_dirs);
    num_bits += impeghe_write_bits_buf(it_bit_buf, val, 3);

    val = pstr_config_data->max_num_of_pred_dirs_per_band - 1;
    if ((val) > 10)
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, 7, 3);
      num_bits += impeghe_write_bits_buf(it_bit_buf, 3, 2);
      num_bits += impeghe_write_bits_buf(it_bit_buf, val - 10, 5);
    }
    else if (val > 7)
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, 7, 3);
      num_bits += impeghe_write_bits_buf(it_bit_buf, val - 7, 2);
    }
    else
      num_bits += impeghe_write_bits_buf(it_bit_buf, val, 3);

    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->dir_grid_table_idx, 2);
  }

  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->par_subband_table_idx, 2);
  if (pstr_config_data->par_subband_table_idx < 3)
  {
    pstr_config_data->num_of_par_subbands =
        impeghe_num_of_par_subbands_table[pstr_config_data->par_subband_table_idx];
  }
  else
  {
    num_bits +=
        impeghe_write_bits_buf(it_bit_buf, (pstr_config_data->num_of_par_subbands - 1), 5);
  }
  if (pstr_config_data->num_of_par_subbands > 0)
  {
    WORD32 last_first_order_subband_idx_bits =
        impeghe_hoa_get_ceil_log2((UWORD32)(pstr_config_data->num_of_par_subbands + 1));

    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_config_data->last_first_order_subband_idx,
                                       (UWORD8)(last_first_order_subband_idx_bits));

    pstr_config_data->upmix_order_per_par_subband_sz = pstr_config_data->num_of_par_subbands;
    for (WORD32 idx = 0; idx < pstr_config_data->num_of_par_subbands; idx++)
    {
      num_bits += impeghe_write_bits_buf(
          it_bit_buf, pstr_config_data->use_real_coeffs_per_par_subband[idx], 1);
    }
  }

  pstr_config_data->amb_asign_m_bits =
      impeghe_hoa_get_ceil_log2(pstr_config_data->max_num_add_active_amb_coeffs);
  pstr_config_data->active_pred_ids_bits =
      impeghe_hoa_get_ceil_log2(pstr_config_data->num_coeffs);

  WORD32 mx;
  i = 1;

  while ((WORD32)(i * pstr_config_data->active_pred_ids_bits + impeghe_hoa_get_ceil_log2(i)) <
         pstr_config_data->num_coeffs)
  {
    i++;
  }
  mx = (1 > (i - 1)) ? 1 : (i - 1);
  pstr_config_data->num_active_pred_ids_bits = impeghe_hoa_get_ceil_log2(mx);

  pstr_config_data->gain_corr_prev_amp_exp_bits = impeghe_hoa_get_ceil_log2(
      impeghe_hoa_get_ceil_log2((UWORD32)(1.5 * pstr_config_data->num_coeffs)) +
      pstr_config_data->max_gain_corr_amp_exp + 1);

  impeghe_hoa_v_vec_len_and_coeff_calc(pstr_config_data);
  return num_bits;
}
