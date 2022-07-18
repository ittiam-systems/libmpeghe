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
#include "impeghe_error_standards.h"
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_common_functions.h"
#include "impeghe_hoa_rom.h"
#include "impeghe_hoa_dir_vec_estimation.h"
#include "impeghe_simple_matrix.h"

/**
 *  impeghe_hoa_dequantize_uniform
 *
 *  \brief Perform uniformde-quantization
 *
 *  \param [in] in_quant_val Quantized value
 *  \param [in] num_bits Number of bits for quantization
 *
 *  \return FLOAT64 Output value
 */
static FLOAT64 impeghe_hoa_dequantize_uniform(UWORD32 in_quant_val, UWORD32 num_bits)
{
  FLOAT64 out_val =
      (FLOAT64)((FLOAT64)(in_quant_val) / (FLOAT64)(impeghe_hoa_get_pow2(num_bits - 1)) - 1.0);
  return out_val;
}

/**
 *  impeghe_hoa_vector_estimation
 *
 *  \brief Vector estimation algorithm
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_in_hoa_frame Input HOA signals
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_hoa_vector_estimation(ia_spatial_enc_str *pstr_se_handle,
                                                  pFlOAT64 ptr_in_hoa_frame)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_enc_dir_vec_est_str *pstr_vec_est = &(pstr_se_handle->dir_vec_est_hdl);
  ia_hoa_vec_sig_str *pstr_vectors = pstr_frm_prm->vectors;
  WORD32 num_var_ch = pstr_se_handle->num_add_perc_coders;
  IA_ERRORCODE err_code = IA_NO_ERROR;

  WORD32 num_vec_elem = pstr_vec_est->num_vec_elem;
  WORD32 frm_sz = pstr_vec_est->frame_size;
  LOOPIDX i, j, idx;
  pUWORD8 ptr_scratch = (pUWORD8)pstr_se_handle->ptr_scratch + pstr_se_handle->scratch_used_size;
  pFlOAT64 ptr_vec[MAX_NUMBER_CHANNELS];
  UWORD32 tmp_for_quant;
  FLOAT64 *ptr_norm_vec = (FLOAT64 *)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (num_var_ch);
  for (i = 0; i < num_var_ch; i++)
  {
    ptr_vec[i] = pstr_vectors[i].sig_id;
    ptr_norm_vec[i] = 0.0;
  }

  if (num_var_ch > (WORD32)pstr_vec_est->num_hoa_coeffs)
  {
    num_var_ch = pstr_vec_est->num_hoa_coeffs;
  }
  idx = frm_sz * num_vec_elem;
  for (i = 0; i < frm_sz; i++)
  {
    pFlOAT64 ptr_ip_hoa_frame = ptr_in_hoa_frame + i;
    for (j = 0; j < num_vec_elem; j++)
    {
      pstr_vec_est->svd_input[idx] = *ptr_ip_hoa_frame;
      idx++;
      ptr_ip_hoa_frame += frm_sz;
    }
  }
  ia_render_hoa_simple_mtrx_str pstr_input;
  ia_render_hoa_simple_mtrx_svd_str pstr_svd;

  pstr_svd.u.mtrx = (pFlOAT64)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (2 * frm_sz) * (num_vec_elem);
  pstr_svd.v.mtrx = (pFlOAT64)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (num_vec_elem) * (num_vec_elem);
  pstr_svd.s.mtrx = (pFlOAT64)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (num_vec_elem) * (num_vec_elem);

  impeghe_hoa_ren_simple_mtrx_init_with_data_ptr(&pstr_input, 2 * frm_sz, num_vec_elem,
                                                 pstr_vec_est->svd_input);
  err_code = impeghe_hoa_ren_simple_mtrx_svd_init(&pstr_svd, &pstr_input, ptr_scratch);
  if (err_code)
  {
    return err_code;
  }

  for (i = 0; i < num_var_ch; i++)
  {
    for (j = 0; j < num_vec_elem; j++)
    {
      UWORD32 offset = j * num_vec_elem;
      ptr_vec[i][j] = pstr_svd.v.mtrx[offset + i];
      ptr_norm_vec[i] += ptr_vec[i][j] * ptr_vec[i][j];
    }
    ptr_norm_vec[i] = sqrt(ptr_norm_vec[i]);
  }

  for (i = 0; i < num_var_ch; i++)
  {
    for (j = 0; j < num_vec_elem; j++)
    {
      ptr_vec[i][j] /= ptr_norm_vec[i];

      tmp_for_quant = impeghe_hoa_quantize_uniform(ptr_vec[i][j], pstr_vec_est->vec_ele_num_bits);
      ptr_vec[i][j] =
          impeghe_hoa_dequantize_uniform(tmp_for_quant, pstr_vec_est->vec_ele_num_bits);

      ptr_vec[i][j] *= (FLOAT64)(pstr_vec_est->hoa_order + 1);
    }
    pstr_vectors[i].index = i + 1;
  }

  return err_code;
}

/**
 *  impeghe_hoa_sort_energy_indices
 *
 *  \brief Arrange the energies in descending order
 *
 *  \param [in] ptr_energy Input energies
 *  \param [in] length Size of energy array
 *  \param [out] ptr_sorted_indices Output array of indices with energy arranged in descending
 * order
 *
 *  \return VOID
 */
static VOID impeghe_hoa_sort_energy_indices(pFlOAT64 ptr_energy, WORD32 length,
                                            pWORD32 ptr_sorted_indices)
{
  WORD32 i, j, temp_id;
  FLOAT64 temp;

  for (i = 0; i < length; i++)
  {
    ptr_sorted_indices[i] = i;
    ptr_energy[i] = fabs(ptr_energy[i]);
  }

  for (i = 0; i < length - 1; i++)
  {
    for (j = 0; j < length - i - 1; j++)
    {
      if (ptr_energy[j] < ptr_energy[j + 1])
      {
        temp = ptr_energy[j];
        ptr_energy[j] = ptr_energy[j + 1];
        ptr_energy[j + 1] = temp;

        temp_id = ptr_sorted_indices[j];
        ptr_sorted_indices[j] = ptr_sorted_indices[j + 1];
        ptr_sorted_indices[j + 1] = temp_id;
      }
    }
  }
  return;
}

/**
 *  impeghe_hoa_is_diff_source
 *
 *  \brief Identify is the sound source is same or different
 *
 *  \param [in] azi_diff Difference in azimuth
 *  \param [in] ele_diff Difference in elevation
 *  \param [in] range Tolerance range
 *
 *  \return WORD32 Flag indicating if sound source is same or different
 */
static WORD32 impeghe_hoa_is_diff_source(FLOAT64 azi_diff, FLOAT64 ele_diff, FLOAT64 range)
{
  azi_diff *= (180 / PI);
  ele_diff *= (180 / PI);

  if (azi_diff < 0)
  {
    azi_diff += 360;
  }
  ele_diff = fabs(ele_diff);

  azi_diff = MIN(azi_diff, (360 - azi_diff));

  if ((azi_diff < range) && (ele_diff < range))
  {
    return 0;
  }
  else
    return 1;
}

/**
 *  impeghe_hoa_direction_estimation_beam_former
 *
 *  \brief direction estimation algorithm
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_in_hoa_frame Input HOA signals
 *
 *  \return VOID
 */
static VOID impeghe_hoa_direction_estimation_beam_former(ia_spatial_enc_str *pstr_se_handle,
                                                         pFlOAT64 ptr_in_hoa_frame)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_enc_dir_vec_est_str *pstr_vec_est = &(pstr_se_handle->dir_vec_est_hdl);
  ia_hoa_dir_id_str *ptr_out_sig_n_dir_idx = pstr_frm_prm->active_n_grid_dir_idx;
  pWORD32 ptr_max_energy_dir = pstr_vec_est->max_energy_dir;
  pFlOAT64 ptr_transpose_mode_mat_fine_grid_tab =
      pstr_se_handle->decmp_hdl.fine_grid_transp_mode_mat;
  WORD32 num_var_ch = pstr_se_handle->num_add_perc_coders;
  WORD32 frm_sz = pstr_vec_est->frame_size;
  WORD32 num_hoa_coeffs = pstr_vec_est->num_hoa_coeffs;
  LOOPIDX x, y;
  WORD32 first_sound_source_dir = -1;
  WORD32 sec_sound_source_dir = -1;

  pUWORD8 ptr_scratch = (pUWORD8)pstr_se_handle->ptr_scratch + pstr_se_handle->scratch_used_size;
  pFlOAT64 ptr_transpose_mode_mat_fine_grid;

  pFlOAT64 ptr_beam_former = (pFlOAT64)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (MAX_NUM_DIRS);

  pFlOAT64 ptr_acc_input = (pFlOAT64)ptr_scratch;
  ptr_scratch += sizeof(FLOAT64) * (num_hoa_coeffs);

  pWORD32 ptr_sorted_indices = (pWORD32)ptr_scratch;
  ptr_scratch += sizeof(WORD32) * (MAX_NUM_DIRS);
  pFLOAT32 ptr_fft_input = (pFLOAT32)ptr_scratch;
  ptr_scratch += sizeof(FLOAT32) * (frm_sz);
  pFLOAT32 ptr_fft_output = (pFLOAT32)ptr_scratch;
  ptr_scratch += sizeof(FLOAT32) * (2) * (frm_sz);

  for (x = 0; x < num_hoa_coeffs; x++)
  {
    for (y = 0; y < frm_sz; y++)
    {
      ptr_fft_input[y] = (FLOAT32)(*ptr_in_hoa_frame++);
    }

    impeghe_complex_fft_re_input(ptr_fft_input, frm_sz, ptr_fft_output);

    ptr_acc_input[x] = 0;
    for (y = 0; y < frm_sz; y++)
    {
      ptr_acc_input[x] += ptr_fft_output[2 * y];
    }
  }

  for (x = 0; x < MAX_NUM_DIRS; x++)
  {
    ptr_transpose_mode_mat_fine_grid = ptr_transpose_mode_mat_fine_grid_tab + x * num_hoa_coeffs;
    ptr_beam_former[x] = 0;
    for (y = 0; y < num_hoa_coeffs; y++)
    {
      ptr_beam_former[x] += ptr_acc_input[y] * (*ptr_transpose_mode_mat_fine_grid++);
    }
  }
  impeghe_hoa_sort_energy_indices(ptr_beam_former, MAX_NUM_DIRS, ptr_sorted_indices);

  if (-1 == ptr_max_energy_dir[0])
  {
    first_sound_source_dir = ptr_sorted_indices[0];
  }
  else
  {
    x = 0;
    do
    {
      FLOAT64 azi_diff = ABS_RAD(impeghe_hoa_azimuth[ptr_max_energy_dir[0]]) -
                         ABS_RAD(impeghe_hoa_azimuth[ptr_sorted_indices[x]]);
      FLOAT64 ele_diff = impeghe_hoa_elevation[ptr_max_energy_dir[0]] -
                         impeghe_hoa_elevation[ptr_sorted_indices[x]];
      if (!impeghe_hoa_is_diff_source(azi_diff, ele_diff, IMPEGHE_DIR_EST_CLUSTER_THR_DEG))
      {
        first_sound_source_dir = ptr_sorted_indices[x];
        ptr_sorted_indices[x] = -1;
        break;
      }
      x++;
    } while (x < IMPEGHE_HOA_NUM_DIR_FOR_SCAN);
  }

  x = 0;
  do
  {
    FLOAT64 azi_diff = 0;
    FLOAT64 ele_diff = 0;
    if ((-1 == ptr_sorted_indices[x]))
    {
      x++;
      continue;
    }
    if (first_sound_source_dir != -1)
    {
      azi_diff = ABS_RAD(impeghe_hoa_azimuth[first_sound_source_dir]) -
                 ABS_RAD(impeghe_hoa_azimuth[ptr_sorted_indices[x]]);
      ele_diff = impeghe_hoa_elevation[first_sound_source_dir] -
                 impeghe_hoa_elevation[ptr_sorted_indices[x]];
    }
    else
    {
      azi_diff = IMPEGHE_DIR_EST_CLUSTER_THR_RAD + 1;
    }

    if (impeghe_hoa_is_diff_source(azi_diff, ele_diff, IMPEGHE_DIR_EST_CLUSTER_THR_DEG))
    {
      sec_sound_source_dir = ptr_sorted_indices[x];
      if (-1 != ptr_max_energy_dir[1])
      {
        azi_diff = ABS_RAD(impeghe_hoa_azimuth[ptr_max_energy_dir[1]]) -
                   ABS_RAD(impeghe_hoa_azimuth[sec_sound_source_dir]);
        ele_diff = impeghe_hoa_elevation[ptr_max_energy_dir[1]] -
                   impeghe_hoa_elevation[sec_sound_source_dir];
        if (impeghe_hoa_is_diff_source(azi_diff, ele_diff, IMPEGHE_DIR_EST_CLUSTER_THR_DEG))
        {
          sec_sound_source_dir = -1;
          x++;
          continue;
        }
        else
        {
          ptr_sorted_indices[x] = -1;
          break;
        }
      }
      else
      {
        ptr_sorted_indices[x] = -1;
        break;
      }
    }
    x++;
  } while (x < IMPEGHE_HOA_NUM_DIR_FOR_SCAN);

  if (num_var_ch > 0)
  {
    if (-1 != first_sound_source_dir)
    {
      ptr_out_sig_n_dir_idx[0].ch_idx = 1;
      ptr_out_sig_n_dir_idx[0].dir_id = first_sound_source_dir + 1;
    }
    ptr_max_energy_dir[0] = first_sound_source_dir;

    if ((num_var_ch > 1) && (sec_sound_source_dir != -1))
    {
      ptr_out_sig_n_dir_idx[1].ch_idx = 2;
      ptr_out_sig_n_dir_idx[1].dir_id = sec_sound_source_dir + 1;
    }

    ptr_max_energy_dir[1] = sec_sound_source_dir;
  }

  return;
}

/**
 *  impeghe_hoa_dir_vec_estimation_process
 *
 *  \brief Background signal processing
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *  \param [in] ptr_in_hoa_frame Input HOA signals
 *
 *  \return IA_ERRORCODE Error Code
 */
IA_ERRORCODE impeghe_hoa_dir_vec_estimation_process(ia_spatial_enc_str *pstr_se_handle,
                                                    pFlOAT64 ptr_in_hoa_frame)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_enc_dir_vec_est_str *pstr_vec_est = &(pstr_se_handle->dir_vec_est_hdl);
  ia_hoa_dir_id_str *ptr_out_sig_n_dir_idx = pstr_frm_prm->active_n_grid_dir_idx;
  ia_hoa_vec_sig_str *pstr_vectors = pstr_frm_prm->vectors;
  WORD32 decision = HOA_EMPTY_DECISION;
  WORD32 num_vec_elem = pstr_vec_est->num_vec_elem;
  WORD32 frm_sz = pstr_vec_est->frame_size;
  LOOPIDX i, j, idx;
  IA_ERRORCODE err_code = IA_NO_ERROR;

  pstr_vec_est->frame_counter++;
  decision = pstr_vec_est->use_vec_est;

  for (i = 0; i < MAX_NUMBER_CHANNELS; i++)
  {
    ptr_out_sig_n_dir_idx[i].ch_idx = (UWORD32)-1;
    pstr_vectors[i].index = (UWORD32)-1;
  }

  if ((decision == HOA_DIR_CHANNEL) && (ptr_in_hoa_frame))
  {
    impeghe_hoa_direction_estimation_beam_former(pstr_se_handle, ptr_in_hoa_frame);
  }

  if ((decision == HOA_VEC_CHANNEL) && (1 < pstr_vec_est->frame_counter) && (ptr_in_hoa_frame))
  {
    err_code = impeghe_hoa_vector_estimation(pstr_se_handle, ptr_in_hoa_frame);
    if (err_code)
    {
      return err_code;
    }
  }

  if (ptr_in_hoa_frame)
  {
    idx = 0;

    for (i = 0; i < frm_sz; i++)
    {
      pFlOAT64 ptr_ip_hoa_frame = ptr_in_hoa_frame + i;
      for (j = 0; j < num_vec_elem; j++)
      {
        pstr_vec_est->svd_input[idx] = *ptr_ip_hoa_frame;
        idx++;
        ptr_ip_hoa_frame += frm_sz;
      }
    }
  }

  return err_code;
}
