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
#include <impeghe_type_def.h>
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_hoa_config_struct.h"
#include "impeghe_hoa_frame_struct.h"
#include "impeghe_hoa_frame.h"

/**
 *  impeghe_hoa_get_sub_idx
 *
 *  \brief Convert HOA coeff index to subtract index
 *
 *  \param [in] hoa_coeff_idx HOA coefficient index
 *
 *  \return WORD32 Index value
 *
 */
static WORD32 impeghe_hoa_get_sub_idx(WORD32 hoa_coeff_idx)
{
  if (hoa_coeff_idx >= 1 && hoa_coeff_idx <= 4)
  {
    return (1);
  }
  else if (hoa_coeff_idx >= 5 && hoa_coeff_idx <= 9)
  {
    return (2);
  }
  else if (hoa_coeff_idx >= 10 && hoa_coeff_idx <= 49)
  {
    return (3);
  }
  else
  {
    return (0);
  }
}
/**
 *  impeghe_hoa_frame_v_vector_data
 *
 *  \brief Fetch HOA frame vector data
 *
 *  \param [in]     it_bit_buf HOA Bit-stream buffer
 *  \param [in,out] pstr_frame_data HOA frame parameters
 *  \param [in]     i vector signal channel id
 *  \param [in]     idx index value
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32 impeghe_hoa_frame_v_vector_data(ia_bit_buf_struct *it_bit_buf,
                                              ia_hoa_frame_struct *pstr_frame_data, WORD32 i,
                                              WORD32 idx)
{
  UWORD32 hoa_coeff_idx = pstr_frame_data->ptr_config_data->idx_offset;
  WORD32 num_bits = 0;

  i--;

  pstr_frame_data->v_vec_length_used = pstr_frame_data->ptr_config_data->v_vec_length_used;
  if (pstr_frame_data->n_bits_q[i] == 4)
  {
    WORD32 j;

    for (j = pstr_frame_data->sgn_val_size[idx]; j < (WORD32)pstr_frame_data->num_vvec_indices[i];
         j++)
    {
      pstr_frame_data->sgn_val[idx][j] = 0;
    }
    pstr_frame_data->sgn_val_size[idx] = pstr_frame_data->num_vvec_indices[i];
    switch (pstr_frame_data->codebk_idx[i])
    {
    case 0:
      pstr_frame_data->n_bit_idx = (10);
      break;
    case 1:
      pstr_frame_data->n_bit_idx = (6);
      break;
    case 2:
      pstr_frame_data->n_bit_idx = (6);
      break;
    case 3:
      pstr_frame_data->n_bit_idx = (6);
      break;
    case 4: // reserved
    case 5: // reserved
    case 6: // reserved
    case 7:
      pstr_frame_data->n_bit_idx =
          impeghe_hoa_get_ceil_log2(pstr_frame_data->ptr_config_data->num_coeffs);
    }
    if (pstr_frame_data->num_vvec_indices[i] == 1)
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->vvec_idx[0] - 1,
                                         (UWORD8)(pstr_frame_data->n_bit_idx));
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->sgn_val[idx][0], 1);
    }
    else
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->weight_idx[i], 8);
      for (j = 0; j < (WORD32)pstr_frame_data->num_vvec_indices[i]; j++)
      {
        num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->vvec_idx[j] - 1,
                                           (UWORD8)(pstr_frame_data->n_bit_idx));

        if (j < 8)
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->sgn_val[idx][j], 1);
          pstr_frame_data->weight_val[j] =
              ((pstr_frame_data->sgn_val[idx][j] * 2) - 1) * pstr_frame_data->weight_val[j];
        }
        else
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->sgn_val[idx][j], 1);
          pstr_frame_data->weight_val[j] = ((pstr_frame_data->sgn_val[idx][j] * 2) - 1) *
                                           pstr_frame_data->weight_val[6 + j % 2];
        }
      }
    }
  }
  else if (pstr_frame_data->n_bits_q[i] == 5)
  {
    WORD32 m;
    for (m = 0; m < pstr_frame_data->v_vec_length_used; m++, hoa_coeff_idx++)
    {
      if ((pstr_frame_data->non_transitional_add_hoa_channels[hoa_coeff_idx] == 0) &&
          !((pstr_frame_data->new_vec_channels[idx] == 1) &&
            (pstr_frame_data->fade_in_add_hoa_channels[hoa_coeff_idx] == 1)))
      {
        num_bits +=
            impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->c_8bit_quantizer_word[idx][m], 8);
      }
    }
  }
  else if (pstr_frame_data->n_bits_q[i] >= 6)
  {
    WORD32 m = 0, huff_idx;
    UWORD16 cid = 0;

    for (m = pstr_frame_data->sgn_val_size[idx]; m < pstr_frame_data->v_vec_length_used; m++)
    {
      pstr_frame_data->sgn_val[idx][m] = 1;
    }

    for (m = 0; m < pstr_frame_data->v_vec_length_used; m++, hoa_coeff_idx++)
    {
      if ((pstr_frame_data->non_transitional_add_hoa_channels[hoa_coeff_idx] == 0) &&
          !((pstr_frame_data->new_vec_channels[idx] == 1) &&
            (pstr_frame_data->fade_in_add_hoa_channels[hoa_coeff_idx] == 1)))
      {
        {
          huff_idx = 5;
          if (pstr_frame_data->cb_flag[i] == 1)
          {
            huff_idx = impeghe_hoa_get_sub_idx(hoa_coeff_idx);
          }
          else if (pstr_frame_data->p_flag[i] == 1)
          {
            huff_idx = 4;
          }
        }
        // Spec section C.5.4.8
        if (pstr_frame_data->c_8bit_quantizer_word[idx][m])
        {
          if (pstr_frame_data->c_8bit_quantizer_word[idx][m] < 0)
          {
            pstr_frame_data->sgn_val[idx][m] = 0;
          }
          else
          {
            pstr_frame_data->sgn_val[idx][m] = 1;
          }
          cid = (UWORD16)impeghe_hoa_get_ceil_log2(
              ABS(pstr_frame_data->c_8bit_quantizer_word[idx][m]));
        }
        else
          cid = 0;

        const impeghe_word_struct *table = &(
            impeghe_hoa_frame_huffman_table[pstr_frame_data->n_bits_q[i] - 6][huff_idx - 1][cid]);
        num_bits += impeghe_write_bits_buf(it_bit_buf, table->code_word, (UWORD8)(table->len));

        pstr_frame_data->additional_value[i][m] =
            ABS(pstr_frame_data->c_8bit_quantizer_word[idx][m]) - (1 << (cid - 1));

        if (cid > 0)
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->sgn_val[idx][m], 1);
          if (cid > 1)
          {
            num_bits += impeghe_write_bits_buf(
                it_bit_buf, pstr_frame_data->additional_value[i][m], (UWORD8)(cid - 1));
          }
        }
      }
      else
      {
        pstr_frame_data->additional_value[i][m] = 0;
      }
    }
    pstr_frame_data->sgn_val_size[idx] = pstr_frame_data->v_vec_length_used;
  }
  return num_bits;
}
/**
 *  impeghe_hoa_frame
 *
 *  \brief Fetch HOA frame data
 *
 *  \param [in]     it_bit_buf  HOA Bit-stream buffer
 *  \param [in,out] pstr_frame_data HOA frame parameters
 *
 *  \return WORD32 Number of bits written
 *
 */
WORD32 impeghe_hoa_frame(ia_bit_buf_struct *it_bit_buf, ia_hoa_frame_struct *pstr_frame_data)
{
  WORD32 i;

  WORD32 num_bits = 0;

  pstr_frame_data->num_of_dir_sigs = 0;
  pstr_frame_data->num_of_vec_sigs = 0;
  memset(pstr_frame_data->non_transitional_add_hoa_channels, 0,
         sizeof(pstr_frame_data->non_transitional_add_hoa_channels));
  memset(pstr_frame_data->fade_in_add_hoa_channels, 0,
         sizeof(pstr_frame_data->fade_in_add_hoa_channels));
  memset(pstr_frame_data->new_vec_channels, 0, sizeof(pstr_frame_data->new_vec_channels));

  num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->hoa_independency_flag, 1);

  for (i = 0; i < (WORD32)pstr_frame_data->ptr_config_data->num_addnl_coders; i++)
  {
    num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->channel_type[i], 2);

    switch (pstr_frame_data->channel_type[i])
    {
    case HOA_DIR_CHANNEL:
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->active_dirs_ids[i], 10);
      break;
    case HOA_VEC_CHANNEL:
      if (pstr_frame_data->hoa_independency_flag)
      {
        if (pstr_frame_data->ptr_config_data->coded_v_vec_length == 1)
        {
          num_bits +=
              impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->new_channel_type_one[i], 1);
          if (1 == pstr_frame_data->new_channel_type_one[i])
          {
            pstr_frame_data->new_vec_channels[i] = 1;
          }
        }
        num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->n_bits_q[i], 4);

        if (pstr_frame_data->n_bits_q[i] == 4)
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->codebk_idx[i], 3);
          num_bits += impeghe_write_bits_buf(
              it_bit_buf, (pstr_frame_data->num_vvec_indices[i] - 1),
              (UWORD8)(pstr_frame_data->ptr_config_data->num_v_vec_vq_elements_bits));
        }
        else if (pstr_frame_data->n_bits_q[i] >= 6)
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->cb_flag[i], 1);
        }
      }
      else
      {
        if (pstr_frame_data->ptr_config_data->coded_v_vec_length == 1 &&
            pstr_frame_data->prev_channel_type[i] != HOA_VEC_CHANNEL)
        {
          pstr_frame_data->new_vec_channels[i] = 1;
          pstr_frame_data->new_channel_type_one[i] = pstr_frame_data->prev_channel_type[i];
        }
        if (pstr_frame_data->same_header_prev_frame[i])
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, 0, 1);
          num_bits += impeghe_write_bits_buf(it_bit_buf, 0, 1);
        }
        else
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->n_bits_q[i], 4);

          if (pstr_frame_data->n_bits_q[i] == 4)
          {
            num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->codebk_idx[i], 3);
            num_bits += impeghe_write_bits_buf(
                it_bit_buf, (pstr_frame_data->num_vvec_indices[i] - 1),
                (UWORD8)(pstr_frame_data->ptr_config_data->num_v_vec_vq_elements_bits));
          }
          else if (pstr_frame_data->n_bits_q[i] > 5)
          {
            num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->p_flag[i], 1);
            num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->cb_flag[i], 1);
          }
        }
      }
      break;
    case HOA_ADD_HOA_CHANNEL:
    {
      if (pstr_frame_data->hoa_independency_flag)
      {
        num_bits +=
            impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->amb_coeff_transition_state[i], 2);
        num_bits +=
            impeghe_write_bits_buf(it_bit_buf,
                                   (pstr_frame_data->amb_coeff_idx[i] - 1 -
                                    pstr_frame_data->ptr_config_data->min_coeffs_for_amb),
                                   (UWORD8)(pstr_frame_data->ptr_config_data->amb_asign_m_bits));
      }
      else
      {
        num_bits +=
            impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->amb_coeff_idx_transition[i], 1);
        if (pstr_frame_data->amb_coeff_idx_transition[i] == 1)
        {
          if (pstr_frame_data->amb_coeff_transition_state[i] == 1)
          {
            pstr_frame_data->amb_coeff_transition_state[i] = 1;
            num_bits += impeghe_write_bits_buf(
                it_bit_buf,
                pstr_frame_data->amb_coeff_idx[i] - 1 -
                    pstr_frame_data->ptr_config_data->min_coeffs_for_amb,
                (UWORD8)(pstr_frame_data->ptr_config_data->amb_asign_m_bits));
          }
        }
      }

      if (1 == pstr_frame_data->ptr_config_data->coded_v_vec_length)
      {
        switch (pstr_frame_data->amb_coeff_transition_state[i])
        {
        case 0:
          pstr_frame_data->non_transitional_add_hoa_channels[pstr_frame_data->amb_coeff_idx[i]] =
              1;
          break;
        case 1:
          pstr_frame_data->fade_in_add_hoa_channels[pstr_frame_data->amb_coeff_idx[i]] = 1;
          break;
        }
      }
    }
    break;
    }
    if (pstr_frame_data->hoa_independency_flag)
    {
      UWORD32 temp = pstr_frame_data->gain_corr_prev_amp_exp[i] +
                     impeghe_hoa_get_ceil_log2(
                         (UWORD32)(1.5f * (FLOAT32)pstr_frame_data->ptr_config_data->num_coeffs));

      num_bits += impeghe_write_bits_buf(
          it_bit_buf, temp,
          (UWORD8)(pstr_frame_data->ptr_config_data->gain_corr_prev_amp_exp_bits));
    }

    WORD32 n = 0;
    WORD32 size = pstr_frame_data->coded_gain_correction_exp_sz[i];

    while (size > 0)
    {
      num_bits +=
          impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->coded_gain_correction_exp[i][n], 1);
      size--;
      n++;
    }

    num_bits +=
        impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->gain_correction_exception[i], 1);

    switch (pstr_frame_data->channel_type[i])
    {
    case HOA_DIR_CHANNEL:
    {
      pstr_frame_data->dir_sig_channel_ids[pstr_frame_data->num_of_dir_sigs] = i + 1;
      pstr_frame_data->num_of_dir_sigs++;
      break;
    }
    case HOA_VEC_CHANNEL:
    {
      pstr_frame_data->vec_sig_channel_ids[pstr_frame_data->num_of_vec_sigs] = i + 1;
      pstr_frame_data->num_of_vec_sigs++;
      break;
    }
    }
  }
  for (i = pstr_frame_data->ptr_config_data->num_addnl_coders;
       i < (WORD32)pstr_frame_data->ptr_config_data->num_transport_ch; ++i)
  {
    {
      WORD32 size, n;
      if (pstr_frame_data->hoa_independency_flag)
      {
        UWORD32 temp = pstr_frame_data->gain_corr_prev_amp_exp[i] +
                       impeghe_hoa_get_ceil_log2((UWORD32)(
                           1.5f * (FLOAT32)pstr_frame_data->ptr_config_data->num_coeffs));

        num_bits += impeghe_write_bits_buf(
            it_bit_buf, temp,
            (UWORD8)(pstr_frame_data->ptr_config_data->gain_corr_prev_amp_exp_bits));
      }
      size = pstr_frame_data->coded_gain_correction_exp_sz[i];
      n = 0;
      while (size > 0)
      {
        num_bits += impeghe_write_bits_buf(it_bit_buf,
                                           pstr_frame_data->coded_gain_correction_exp[i][n], 1);
        size--;
        n++;
      }
      num_bits +=
          impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->gain_correction_exception[i], 1);
    }
  }

  {
    for (i = 0; i < (WORD32)pstr_frame_data->num_of_vec_sigs; i++)
    {
      pstr_frame_data->sgn_val_size[i] = pstr_frame_data->prev_sgn_val_size[i];
    }

    for (i = 0; i < (WORD32)pstr_frame_data->num_of_vec_sigs; i++)
    {
      num_bits += impeghe_hoa_frame_v_vector_data(it_bit_buf, pstr_frame_data,
                                                  pstr_frame_data->vec_sig_channel_ids[i], i);
    }

    for (i = 0; i < (WORD32)pstr_frame_data->num_of_vec_sigs; i++)
    {
      pstr_frame_data->prev_sgn_val_size[i] = pstr_frame_data->sgn_val_size[i];
    }
  }

  if (pstr_frame_data->ptr_config_data->is_single_layer == 1)
  {
    if (pstr_frame_data->num_of_dir_sigs > 0)
    {
      WORD32 pred_ids_bits, loop;
      pred_ids_bits = impeghe_hoa_get_ceil_log2(pstr_frame_data->num_of_dir_sigs + 1);
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->ps_prediction_active, 1);

      if (pstr_frame_data->ps_prediction_active)
      {
        num_bits +=
            impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->kind_of_coded_pred_ids, 1);

        if (pstr_frame_data->kind_of_coded_pred_ids)
        {

          UWORD32 num_active_pred = 0;
          UWORD32 n;
          for (n = 0; n < pstr_frame_data->ptr_config_data->num_coeffs; n++)
          {
            if (pstr_frame_data->active_pred[n] > 0)
            {
              num_active_pred++;
            }
          }

          num_bits += impeghe_write_bits_buf(
              it_bit_buf, (num_active_pred - 1),
              (UWORD8)(pstr_frame_data->ptr_config_data->num_active_pred_ids_bits));

          i = 0;
          while (i < pstr_frame_data->num_active_pred)
          {
            num_bits += impeghe_write_bits_buf(
                it_bit_buf, pstr_frame_data->pred_ids[i],
                (UWORD8)(pstr_frame_data->ptr_config_data->active_pred_ids_bits));
            i++;
          }
        }
        else
        {
          loop = (pstr_frame_data->ptr_config_data->num_coeffs);
          for (i = 0; i < loop; i++)
          {
            num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->active_pred[i], 1);
            if (pstr_frame_data->active_pred[i] > 0)
            {
              pstr_frame_data->num_active_pred++;
            }
          }
        }
        pstr_frame_data->num_of_gains = 0;
        loop = pstr_frame_data->num_active_pred *
               pstr_frame_data->ptr_config_data->max_no_of_dir_sigs_for_prediction;
        for (i = 0; i < loop; i++)
        {
          num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->pred_dir_sig_ids[i],
                                             (UWORD8)(pred_ids_bits));
          if (pstr_frame_data->pred_dir_sig_ids[i] > 0)
          {
            pstr_frame_data->pred_dir_sig_ids[i] =
                pstr_frame_data->dir_sig_channel_ids[pstr_frame_data->pred_dir_sig_ids[i] - 1];
            pstr_frame_data->num_of_gains++;
          }
        }
        for (i = 0; i < pstr_frame_data->num_of_gains; i++)
        {
          if (pstr_frame_data->pred_dir_sig_ids[i] > 0)
          {
            num_bits += impeghe_write_bits_buf(
                it_bit_buf, pstr_frame_data->pred_gains[i],
                (UWORD8)(pstr_frame_data->ptr_config_data->no_of_bits_per_scale_factor));
          }
        }
      }
    }
    if (pstr_frame_data->ptr_config_data->num_of_pred_subbands > 0)
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->use_dir_pred, 1);
    }
    if (pstr_frame_data->ptr_config_data->num_of_par_subbands)
    {
      num_bits += impeghe_write_bits_buf(it_bit_buf, pstr_frame_data->perform_par, 1);
    }
  }

  for (i = 0; i < (WORD32)pstr_frame_data->ptr_config_data->num_addnl_coders; i++)
  {
    pstr_frame_data->prev_channel_type[i] = pstr_frame_data->channel_type[i];
  }
  return num_bits;
}
