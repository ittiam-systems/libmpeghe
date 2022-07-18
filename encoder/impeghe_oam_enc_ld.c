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

#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_cnst.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_oam_enc.h"
#include "impeghe_oam_enc_ld.h"
#include "impeghe_oam_enc_utils.h"

/**
 *  impeghe_obj_md_round_multidata_frame_ld
 *
 *  \brief Rounds off the OAM parameters.
 *
 *  \param [in,out]	pstr_oam_data	Pointer to OAM multidata structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_obj_md_round_multidata_frame_ld(ia_oam_enc_multidata_struct *pstr_oam_data)
{
  WORD32 n = 0;

  for (n = 0; n < pstr_oam_data->num_elements; n++)
  {
    pstr_oam_data->azimuth[n] = (FLOAT32)ROUND(pstr_oam_data->azimuth[n]);
    pstr_oam_data->elevation[n] = (FLOAT32)ROUND(pstr_oam_data->elevation[n]);
    pstr_oam_data->radius[n] = (FLOAT32)ROUND(pstr_oam_data->radius[n]);
    pstr_oam_data->gain[n] = (FLOAT32)ROUND(pstr_oam_data->gain[n]);
    pstr_oam_data->spread[n] = (FLOAT32)ROUND(pstr_oam_data->spread[n]);
    pstr_oam_data->spread_height[n] = (FLOAT32)ROUND(pstr_oam_data->spread_height[n]);
    pstr_oam_data->spread_depth[n] = (FLOAT32)ROUND(pstr_oam_data->spread_depth[n]);
    pstr_oam_data->dyn_obj_priority[n] = (FLOAT32)ROUND(pstr_oam_data->dyn_obj_priority[n]);
  }
  return;
}

/**
 *  impeghe_obj_md_intracoded_object_data_ld
 *
 *  \brief Encodes OAM data in intracoded style.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32 impeghe_obj_md_intracoded_object_data_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                                       ia_bit_buf_struct *it_bit_buf)
{
  WORD32 loop_idx;
  ia_oam_enc_multidata_struct *pstr_oam_data = &ptr_oam_enc_state->str_oam_data_inp;
  WORD32 bits_written_local = 0;

  WORD32 common_val[6] = {1, 1, 1, 1, 1, 1};

  for (loop_idx = 1; loop_idx < pstr_oam_data->num_elements; loop_idx++)
  {
    if (common_val[OAM_LD_AZIMUTH_IDX] &&
        (pstr_oam_data->azimuth[loop_idx] != pstr_oam_data->azimuth[0]))
    {
      common_val[OAM_LD_AZIMUTH_IDX] = 0;
    }
    if (common_val[OAM_LD_ELEVATION_IDX] &&
        (pstr_oam_data->elevation[loop_idx] != pstr_oam_data->elevation[0]))
    {
      common_val[OAM_LD_ELEVATION_IDX] = 0;
    }
    if (common_val[OAM_LD_RADIUS_IDX] &&
        (pstr_oam_data->radius[loop_idx] != pstr_oam_data->radius[0]))
    {
      common_val[OAM_LD_RADIUS_IDX] = 0;
    }
    if (common_val[OAM_LD_GAIN_IDX] && (pstr_oam_data->gain[loop_idx] != pstr_oam_data->gain[0]))
    {
      common_val[OAM_LD_GAIN_IDX] = 0;
    }
    if (common_val[OAM_LD_SPREAD_IDX])
    {
      if (ptr_oam_enc_state->str_config.has_uniform_spread == 1)
      {
        if (pstr_oam_data->spread[loop_idx] != pstr_oam_data->spread[0])
        {
          common_val[OAM_LD_SPREAD_IDX] = 0;
        }
      }
      else
      {
        if (pstr_oam_data->spread[loop_idx] != pstr_oam_data->spread[0] ||
            pstr_oam_data->spread_height[loop_idx] != pstr_oam_data->spread_height[0] ||
            pstr_oam_data->spread_depth[loop_idx] != pstr_oam_data->spread_depth[0])
        {
          common_val[OAM_LD_SPREAD_IDX] = 0;
        }
      }
    }

    if (ptr_oam_enc_state->str_config.has_dyn_obj_priority == 1)
    {
      if (common_val[OAM_LD_DYN_OBJ_PRI_IDX] &&
          (pstr_oam_data->dyn_obj_priority[loop_idx] != pstr_oam_data->dyn_obj_priority[0]))
      {
        common_val[OAM_LD_DYN_OBJ_PRI_IDX] = 0;
      }
    }
  }

  if (pstr_oam_data->num_elements > 1)
  {
    bits_written_local += impeghe_write_bits_buf(
        it_bit_buf, ROUND(ptr_oam_enc_state->str_config.fixed_values[OAM_LD_AZIMUTH_IDX]), 1);
    if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_AZIMUTH_IDX])
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->azimuth[0]), NO_BITS_AZI_OAM);
    }
    else
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(common_val[OAM_LD_AZIMUTH_IDX]), 1);
      if (common_val[OAM_LD_AZIMUTH_IDX])
      {
        bits_written_local +=
            impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->azimuth[0]), NO_BITS_AZI_OAM);
      }
      else
      {
        for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->azimuth[loop_idx]), NO_BITS_AZI_OAM);
        }
      }
    }

    bits_written_local += impeghe_write_bits_buf(
        it_bit_buf, ptr_oam_enc_state->str_config.fixed_values[OAM_LD_ELEVATION_IDX], 1);
    if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_ELEVATION_IDX])
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->elevation[0]), NO_BITS_ELE_OAM);
    }
    else
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, common_val[OAM_LD_ELEVATION_IDX], 1);
      if (common_val[OAM_LD_ELEVATION_IDX])
      {
        bits_written_local += impeghe_write_bits_buf(
            it_bit_buf, ROUND(pstr_oam_data->elevation[0]), NO_BITS_ELE_OAM);
      }
      else
      {
        for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->elevation[loop_idx]), NO_BITS_ELE_OAM);
        }
      }
    }

    bits_written_local += impeghe_write_bits_buf(
        it_bit_buf, ptr_oam_enc_state->str_config.fixed_values[OAM_LD_RADIUS_IDX], 1);
    if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_RADIUS_IDX])
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->radius[0]), NO_BITS_RAD_OAM);
    }
    else
    {
      bits_written_local += impeghe_write_bits_buf(it_bit_buf, common_val[OAM_LD_RADIUS_IDX], 1);
      if (common_val[OAM_LD_RADIUS_IDX])
      {
        bits_written_local +=
            impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->radius[0]), NO_BITS_RAD_OAM);
      }
      else
      {
        for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->radius[loop_idx]), NO_BITS_RAD_OAM);
        }
      }
    }

    bits_written_local += impeghe_write_bits_buf(
        it_bit_buf, ptr_oam_enc_state->str_config.fixed_values[OAM_LD_GAIN_IDX], 1);
    if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_GAIN_IDX])
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->gain[0]), NO_BITS_GAIN_OAM);
    }
    else
    {
      bits_written_local += impeghe_write_bits_buf(it_bit_buf, common_val[OAM_LD_GAIN_IDX], 1);
      if (common_val[OAM_LD_GAIN_IDX])
      {
        bits_written_local +=
            impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->gain[0]), NO_BITS_GAIN_OAM);
      }
      else
      {
        for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->gain[loop_idx]), NO_BITS_GAIN_OAM);
        }
      }
    }

    bits_written_local += impeghe_write_bits_buf(
        it_bit_buf, ptr_oam_enc_state->str_config.fixed_values[OAM_LD_SPREAD_IDX], 1);
    if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_SPREAD_IDX])
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->spread[0]), NO_BITS_SPREAD_OAM);
      if (ptr_oam_enc_state->str_config.has_uniform_spread != 1)
      {
        bits_written_local += impeghe_write_bits_buf(
            it_bit_buf, ROUND(pstr_oam_data->spread_height[0]), NO_BITS_SPREAD_HEIGHT_OAM);
        bits_written_local += impeghe_write_bits_buf(
            it_bit_buf, ROUND(pstr_oam_data->spread_depth[0]), NO_BITS_SPREAD_DEPTH_OAM);
      }
    }
    else
    {
      bits_written_local += impeghe_write_bits_buf(it_bit_buf, common_val[OAM_LD_SPREAD_IDX], 1);
      if (common_val[OAM_LD_SPREAD_IDX])
      {
        bits_written_local += impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->spread[0]),
                                                     NO_BITS_SPREAD_OAM);
        if (ptr_oam_enc_state->str_config.has_uniform_spread != 1)
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->spread_height[0]), NO_BITS_SPREAD_HEIGHT_OAM);
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->spread_depth[0]), NO_BITS_SPREAD_DEPTH_OAM);
        }
      }
      else
      {
        if (ptr_oam_enc_state->str_config.has_uniform_spread != 1)
        {
          for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
          {
            bits_written_local += impeghe_write_bits_buf(
                it_bit_buf, ROUND(pstr_oam_data->spread[loop_idx]), NO_BITS_SPREAD_OAM);

            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->spread_height[loop_idx]),
                                       NO_BITS_SPREAD_HEIGHT_OAM);
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->spread_depth[loop_idx]),
                                       NO_BITS_SPREAD_DEPTH_OAM);
          }
        }
        else
        {
          for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
          {
            bits_written_local += impeghe_write_bits_buf(
                it_bit_buf, ROUND(pstr_oam_data->spread[loop_idx]), NO_BITS_SPREAD_OAM);
          }
        }
      }
    }

    if (ptr_oam_enc_state->str_config.has_dyn_obj_priority == 1)
    {
      bits_written_local += impeghe_write_bits_buf(
          it_bit_buf, ptr_oam_enc_state->str_config.fixed_values[OAM_LD_DYN_OBJ_PRI_IDX], 1);
      if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_DYN_OBJ_PRI_IDX])
      {
        bits_written_local += impeghe_write_bits_buf(
            it_bit_buf, ROUND(pstr_oam_data->dyn_obj_priority[0]), NO_BITS_PRIORITY_OAM);
      }
      else
      {
        bits_written_local +=
            impeghe_write_bits_buf(it_bit_buf, common_val[OAM_LD_DYN_OBJ_PRI_IDX], 1);
        if (common_val[OAM_LD_DYN_OBJ_PRI_IDX])
        {
          bits_written_local += impeghe_write_bits_buf(
              it_bit_buf, ROUND(pstr_oam_data->dyn_obj_priority[0]), NO_BITS_PRIORITY_OAM);
        }
        else
        {
          for (loop_idx = 0; loop_idx < pstr_oam_data->num_elements; loop_idx++)
          {
            bits_written_local += impeghe_write_bits_buf(
                it_bit_buf, ROUND(pstr_oam_data->dyn_obj_priority[loop_idx]),
                NO_BITS_PRIORITY_OAM);
          }
        }
      }
    }
  }
  else
  {
    bits_written_local +=
        impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->azimuth[0]), NO_BITS_AZI_OAM);
    bits_written_local +=
        impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->elevation[0]), NO_BITS_ELE_OAM);
    bits_written_local +=
        impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->radius[0]), NO_BITS_RAD_OAM);
    bits_written_local +=
        impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->gain[0]), NO_BITS_GAIN_OAM);
    bits_written_local +=
        impeghe_write_bits_buf(it_bit_buf, ROUND(pstr_oam_data->spread[0]), NO_BITS_SPREAD_OAM);
    if (ptr_oam_enc_state->str_config.has_uniform_spread != 1)
    {
      bits_written_local += impeghe_write_bits_buf(
          it_bit_buf, ROUND(pstr_oam_data->spread_height[0]), NO_BITS_SPREAD_HEIGHT_OAM);
      bits_written_local += impeghe_write_bits_buf(
          it_bit_buf, ROUND(pstr_oam_data->spread_depth[0]), NO_BITS_SPREAD_DEPTH_OAM);
    }
    if (ptr_oam_enc_state->str_config.has_dyn_obj_priority == 1)
    {
      bits_written_local += impeghe_write_bits_buf(
          it_bit_buf, ROUND(pstr_oam_data->dyn_obj_priority[0]), NO_BITS_PRIORITY_OAM);
    }
  }

  return bits_written_local;
}

/**
 *  impeghe_obj_md_calc_dpcm_frame_ld
 *
 *  \brief Calculates differential frame.
 *
 *  \param [in,out]	ptr_oam_enc_state	Pointer to OAM state structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_obj_md_calc_dpcm_frame_ld(ia_oam_enc_state_struct *ptr_oam_enc_state)
{
  WORD32 loop_idx;
  ia_oam_enc_multidata_struct *pstr_oam_data_inp = &ptr_oam_enc_state->str_oam_data_inp;
  ia_oam_enc_multidata_struct *pstr_oam_data_last = &ptr_oam_enc_state->str_oam_data_last;
  ia_oam_enc_multidata_struct *pstr_oam_data_diff = &ptr_oam_enc_state->str_oam_data_diff;

  for (loop_idx = 0; loop_idx < pstr_oam_data_inp->num_elements; loop_idx++)
  {
    pstr_oam_data_diff->azimuth[loop_idx] =
        pstr_oam_data_inp->azimuth[loop_idx] - pstr_oam_data_last->azimuth[loop_idx];
    pstr_oam_data_diff->elevation[loop_idx] =
        pstr_oam_data_inp->elevation[loop_idx] - pstr_oam_data_last->elevation[loop_idx];
    pstr_oam_data_diff->radius[loop_idx] =
        pstr_oam_data_inp->radius[loop_idx] - pstr_oam_data_last->radius[loop_idx];
    pstr_oam_data_diff->gain[loop_idx] =
        pstr_oam_data_inp->gain[loop_idx] - pstr_oam_data_last->gain[loop_idx];
    pstr_oam_data_diff->spread[loop_idx] =
        pstr_oam_data_inp->spread[loop_idx] - pstr_oam_data_last->spread[loop_idx];
    pstr_oam_data_diff->spread_height[loop_idx] =
        pstr_oam_data_inp->spread_height[loop_idx] - pstr_oam_data_last->spread_height[loop_idx];
    pstr_oam_data_diff->spread_depth[loop_idx] =
        pstr_oam_data_inp->spread_depth[loop_idx] - pstr_oam_data_last->spread_depth[loop_idx];
    pstr_oam_data_diff->dyn_obj_priority[loop_idx] =
        pstr_oam_data_inp->dyn_obj_priority[loop_idx] -
        pstr_oam_data_last->dyn_obj_priority[loop_idx];
  }
  return;
}

/**
 *  signed_bits_required
 *
 *  \brief Calculates bits required to store a value.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in]		ptr_data_row		Pointer to data row
 *  \param [out]	ptr_bits			Pointer to number of bits needed to store
 * the
 * value
 *
 *  \return VOID
 *
 */
static VOID signed_bits_required(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                 const WORD32 *ptr_data_row, WORD32 *ptr_bits)
{
  WORD32 x_min, x_max;
  WORD32 min_val, max_val;
  WORD32 val;
  WORD32 loop_idx, kmax;

  val = ptr_data_row[0];
  x_min = val;
  x_max = val;
  kmax = OAM_LD_NUM_COMPONENTS + ptr_oam_enc_state->str_config.has_dyn_obj_priority;
  for (loop_idx = 1; loop_idx < kmax; loop_idx++)
  {
    val = ptr_data_row[loop_idx];
    x_min = MIN(x_min, val);
    x_max = MAX(x_max, val);
  }

  if (!ptr_oam_enc_state->str_config.has_uniform_spread)
  {
    for (loop_idx = OAM_LD_SPREAD_HEIGHT_IDX; loop_idx <= OAM_LD_SPREAD_DEPTH_IDX; loop_idx++)
    {
      val = ptr_data_row[loop_idx];
      x_min = MIN(x_min, val);
      x_max = MAX(x_max, val);
    }
  }

  *ptr_bits = 2;
  min_val = OAM_BIT_SIGN_MIN_VALUE(*ptr_bits);
  max_val = OAM_BIT_SIGN_MAX_VALUE(*ptr_bits);
  while (x_min < min_val || x_max > max_val)
  {
    (*ptr_bits)++;
    min_val = OAM_BIT_SIGN_MIN_VALUE(*ptr_bits);
    max_val = OAM_BIT_SIGN_MAX_VALUE(*ptr_bits);
  }

  return;
}

/**
 *  impeghe_obj_md_single_dynamic_object_data_ld
 *
 *  \brief Encodes object in dynamic format.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *  \param [in]		ptr_single_data		Pointer to single object data buffer
 *  \param [in]		flag_absolute		Flag
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32
impeghe_obj_md_single_dynamic_object_data_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                             ia_bit_buf_struct *it_bit_buf,
                                             WORD32 *ptr_single_data, WORD32 flag_absolute)
{
  WORD32 num_bits;
  WORD32 loop_idx;
  WORD32 bits_written_local = 0;
  WORD32 flag[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const WORD32 word_size_array[6] = {NO_BITS_AZI_OAM,  NO_BITS_ELE_OAM,    NO_BITS_RAD_OAM,
                                     NO_BITS_GAIN_OAM, NO_BITS_SPREAD_OAM, NO_BITS_PRIORITY_OAM};

  if (flag_absolute)
  {
    for (loop_idx = 0;
         loop_idx < OAM_LD_NUM_COMPONENTS + ptr_oam_enc_state->str_config.has_dyn_obj_priority;
         loop_idx++)
    {
      if (!ptr_oam_enc_state->str_config.fixed_values[loop_idx])
      {
        bits_written_local += impeghe_write_bits_buf(it_bit_buf, ptr_single_data[loop_idx],
                                                     word_size_array[loop_idx]);
      }
    }

    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_SPREAD_IDX] &&
        !ptr_oam_enc_state->str_config.has_uniform_spread)
    {
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ptr_single_data[OAM_LD_SPREAD_HEIGHT_IDX],
                                 word_size_array[OAM_LD_SPREAD_IDX]);
      bits_written_local +=
          impeghe_write_bits_buf(it_bit_buf, ptr_single_data[OAM_LD_SPREAD_DEPTH_IDX],
                                 word_size_array[OAM_LD_SPREAD_IDX]);
    }
  }
  else
  {
    signed_bits_required(ptr_oam_enc_state, ptr_single_data, &num_bits);
    bits_written_local += impeghe_write_bits_buf(it_bit_buf, (num_bits - 2), 3);

    for (loop_idx = 0;
         loop_idx < OAM_LD_NUM_COMPONENTS + ptr_oam_enc_state->str_config.has_dyn_obj_priority;
         loop_idx++)
    {
      if (!ptr_oam_enc_state->str_config.fixed_values[loop_idx])
      {
        if (ptr_single_data[loop_idx])
        {
          flag[loop_idx] = 1;
        }
        bits_written_local += impeghe_write_bits_buf(it_bit_buf, flag[loop_idx], 1);
        if (flag[loop_idx])
        {
          if (loop_idx == 0)
          {
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ptr_single_data[loop_idx], num_bits);
          }
          else
          {
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ptr_single_data[loop_idx],
                                       MIN(num_bits, (word_size_array[loop_idx] + 1)));
          }
        }

        if (loop_idx == OAM_LD_SPREAD_IDX && !ptr_oam_enc_state->str_config.has_uniform_spread)
        {
          if (ptr_single_data[OAM_LD_SPREAD_HEIGHT_IDX])
          {
            flag[OAM_LD_SPREAD_HEIGHT_IDX] = 1;

            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, flag[OAM_LD_SPREAD_HEIGHT_IDX], 1);
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ptr_single_data[OAM_LD_SPREAD_HEIGHT_IDX],
                                       MIN(num_bits, (NO_BITS_SPREAD_HEIGHT_OAM + 1)));
          }
          else
          {
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, flag[OAM_LD_SPREAD_HEIGHT_IDX], 1);
          }

          if (ptr_single_data[OAM_LD_SPREAD_DEPTH_IDX])
          {
            flag[OAM_LD_SPREAD_DEPTH_IDX] = 1;

            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, flag[OAM_LD_SPREAD_DEPTH_IDX], 1);
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, ptr_single_data[OAM_LD_SPREAD_DEPTH_IDX],
                                       MIN(num_bits, (NO_BITS_SPREAD_DEPTH_OAM + 1)));
          }
          else
          {
            bits_written_local +=
                impeghe_write_bits_buf(it_bit_buf, flag[OAM_LD_SPREAD_DEPTH_IDX], 1);
          }
        }
      }
    }
  }

  return bits_written_local;
}

/**
 *  impeghe_obj_md_dynamic_object_data_ld
 *
 *  \brief Encodes object in dynamic format.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *
 *  \return WORD32 NUmber of bits written
 *
 */
static WORD32 impeghe_obj_md_dynamic_object_data_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                                    ia_bit_buf_struct *it_bit_buf)
{
  WORD32 loop_idx;
  WORD32 num_objects = ptr_oam_enc_state->str_config.num_objects;
  WORD32 word_size = 0;
  WORD32 flag_absolute = 0;
  WORD32 bits_written_diff = 0;
  WORD32 data_row[8];
  WORD32 size_absolute;
  WORD32 sum, bits_written = 0;
  WORD32 word_size_array[6] = {NO_BITS_AZI_OAM,  NO_BITS_ELE_OAM,    NO_BITS_RAD_OAM,
                               NO_BITS_GAIN_OAM, NO_BITS_SPREAD_OAM, NO_BITS_PRIORITY_OAM};

  WORD32 needs_transmission[OAM_MAX_NUM_OBJECTS] = {0};

  for (loop_idx = 0;
       loop_idx < OAM_LD_NUM_COMPONENTS + ptr_oam_enc_state->str_config.has_dyn_obj_priority;
       loop_idx++)
  {
    if (ptr_oam_enc_state->str_config.fixed_values[loop_idx] == 0)
    {
      word_size += word_size_array[loop_idx];
    }
  }
  if (ptr_oam_enc_state->str_config.fixed_values[OAM_LD_SPREAD_IDX] == 0 &&
      ptr_oam_enc_state->str_config.has_uniform_spread == 0)
  {
    word_size += NO_BITS_SPREAD_HEIGHT_OAM;
    word_size += NO_BITS_SPREAD_DEPTH_OAM;
  }

  for (loop_idx = 0; loop_idx < ptr_oam_enc_state->str_oam_data_inp.num_elements; loop_idx++)
  {
    sum = 0;

    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_AZIMUTH_IDX])
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.azimuth[loop_idx]));
    }
    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_ELEVATION_IDX])
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.elevation[loop_idx]));
    }
    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_RADIUS_IDX])
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.radius[loop_idx]));
    }
    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_GAIN_IDX])
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.gain[loop_idx]));
    }
    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_SPREAD_IDX])
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.spread[loop_idx]));
      if (!ptr_oam_enc_state->str_config.has_uniform_spread)
      {
        sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.spread_height[loop_idx]));
        sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.spread_depth[loop_idx]));
      }
    }
    if (!ptr_oam_enc_state->str_config.fixed_values[OAM_LD_DYN_OBJ_PRI_IDX] &&
        ptr_oam_enc_state->str_config.has_dyn_obj_priority)
    {
      sum += ABS(ROUND(ptr_oam_enc_state->str_oam_data_diff.dyn_obj_priority[loop_idx]));
    }

    if (sum > 0)
    {
      needs_transmission[loop_idx] = 1;
    }
    else
    {
      needs_transmission[loop_idx] = 0;
    }
  }

  impeghe_reset_bit_buffer(&ptr_oam_enc_state->str_bit_buf_diff);

  bits_written_diff +=
      impeghe_write_bits_buf(&ptr_oam_enc_state->str_bit_buf_diff, flag_absolute, 1);

  for (loop_idx = 0; loop_idx < ptr_oam_enc_state->str_oam_data_inp.num_elements; loop_idx++)
  {
    bits_written_diff += impeghe_write_bits_buf(&ptr_oam_enc_state->str_bit_buf_diff,
                                                needs_transmission[loop_idx], 1);

    if (needs_transmission[loop_idx] == 1)
    {
      data_row[OAM_LD_AZIMUTH_IDX] =
          ROUND(ptr_oam_enc_state->str_oam_data_diff.azimuth[loop_idx]);
      data_row[OAM_LD_ELEVATION_IDX] =
          ROUND(ptr_oam_enc_state->str_oam_data_diff.elevation[loop_idx]);
      data_row[OAM_LD_RADIUS_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_diff.radius[loop_idx]);
      data_row[OAM_LD_GAIN_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_diff.gain[loop_idx]);
      data_row[OAM_LD_SPREAD_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_diff.spread[loop_idx]);
      if (!ptr_oam_enc_state->str_config.has_uniform_spread)
      {
        data_row[OAM_LD_SPREAD_HEIGHT_IDX] =
            ROUND(ptr_oam_enc_state->str_oam_data_diff.spread_height[loop_idx]);
        data_row[OAM_LD_SPREAD_DEPTH_IDX] =
            ROUND(ptr_oam_enc_state->str_oam_data_diff.spread_depth[loop_idx]);
      }
      if (ptr_oam_enc_state->str_config.has_dyn_obj_priority)
      {
        data_row[OAM_LD_DYN_OBJ_PRI_IDX] =
            ROUND(ptr_oam_enc_state->str_oam_data_diff.dyn_obj_priority[loop_idx]);
      }

      bits_written_diff += impeghe_obj_md_single_dynamic_object_data_ld(
          ptr_oam_enc_state, &ptr_oam_enc_state->str_bit_buf_diff, data_row, flag_absolute);
    }
  }

  sum = 0;
  for (loop_idx = 0; loop_idx < ptr_oam_enc_state->str_oam_data_inp.num_elements; loop_idx++)
  {
    sum += needs_transmission[loop_idx];
  }

  size_absolute = 1 + ptr_oam_enc_state->str_oam_data_inp.num_elements + sum * word_size;

  if (size_absolute <= bits_written_diff)
  {
    flag_absolute = 1;

    bits_written += impeghe_write_bits_buf(it_bit_buf, flag_absolute, 1);

    for (loop_idx = 0; loop_idx < num_objects; loop_idx++)
    {
      bits_written += impeghe_write_bits_buf(it_bit_buf, needs_transmission[loop_idx], 1);

      if (needs_transmission[loop_idx] == 1)
      {
        data_row[OAM_LD_AZIMUTH_IDX] =
            ROUND(ptr_oam_enc_state->str_oam_data_inp.azimuth[loop_idx]);
        data_row[OAM_LD_ELEVATION_IDX] =
            ROUND(ptr_oam_enc_state->str_oam_data_inp.elevation[loop_idx]);
        data_row[OAM_LD_RADIUS_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_inp.radius[loop_idx]);
        data_row[OAM_LD_GAIN_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_inp.gain[loop_idx]);
        data_row[OAM_LD_SPREAD_IDX] = ROUND(ptr_oam_enc_state->str_oam_data_inp.spread[loop_idx]);
        if (!ptr_oam_enc_state->str_config.has_uniform_spread)
        {
          data_row[OAM_LD_SPREAD_HEIGHT_IDX] =
              ROUND(ptr_oam_enc_state->str_oam_data_inp.spread_height[loop_idx]);
          data_row[OAM_LD_SPREAD_DEPTH_IDX] =
              ROUND(ptr_oam_enc_state->str_oam_data_inp.spread_depth[loop_idx]);
        }
        if (ptr_oam_enc_state->str_config.has_dyn_obj_priority)
        {
          data_row[OAM_LD_DYN_OBJ_PRI_IDX] =
              ROUND(ptr_oam_enc_state->str_oam_data_inp.dyn_obj_priority[loop_idx]);
        }

        bits_written += impeghe_obj_md_single_dynamic_object_data_ld(
            ptr_oam_enc_state, it_bit_buf, data_row, flag_absolute);
      }
    }
  }
  else
  {
    bits_written += impeghe_obj_md_bitbuf_write(it_bit_buf, ptr_oam_enc_state->bit_buf_base_diff,
                                                bits_written_diff);
  }
  return bits_written;
}

/**
 *  impeghe_obj_md_update_last_sample_ld
 *
 *  \brief Updated pstr_oam_data_last sample in hanlde.
 *
 *  \param [in,out]	ptr_oam_enc_state	Pointer to OAM state structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_obj_md_update_last_sample_ld(ia_oam_enc_state_struct *ptr_oam_enc_state)
{
  WORD32 loop_idx;

  for (loop_idx = 0; loop_idx < ptr_oam_enc_state->str_oam_data_inp.num_elements; loop_idx++)
  {
    ptr_oam_enc_state->str_oam_data_last.azimuth[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.azimuth[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.elevation[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.elevation[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.radius[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.radius[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.gain[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.gain[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.spread[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.spread[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.spread_height[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.spread_height[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.spread_depth[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.spread_depth[loop_idx];
    ptr_oam_enc_state->str_oam_data_last.dyn_obj_priority[loop_idx] =
        ptr_oam_enc_state->str_oam_data_inp.dyn_obj_priority[loop_idx];
  }
  return;
}

/**
 *  impeghe_obj_md_compress_oam_data_ld
 *
 *  \brief Encodes OAM data.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in]		is_iframe			Flag to indicate i-frame
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32 impeghe_obj_md_compress_oam_data_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                                  WORD32 is_iframe, ia_bit_buf_struct *it_bit_buf)
{
  WORD32 bits_written_intra = 0;
  WORD32 bits_written_dynm = 0, bits_written = 0;

  if (is_iframe)
  {
    bits_written += impeghe_write_bits_buf(it_bit_buf, 1, 1);
    bits_written += impeghe_obj_md_intracoded_object_data_ld(ptr_oam_enc_state, it_bit_buf);
  }
  else
  {
    impeghe_reset_bit_buffer(&ptr_oam_enc_state->str_bit_buf_intra);

    bits_written_intra = impeghe_obj_md_intracoded_object_data_ld(
        ptr_oam_enc_state, &ptr_oam_enc_state->str_bit_buf_intra);

    impeghe_obj_md_calc_dpcm_frame_ld(ptr_oam_enc_state);

    impeghe_reset_bit_buffer(&ptr_oam_enc_state->str_bit_buf_dynm);

    bits_written_dynm = impeghe_obj_md_dynamic_object_data_ld(
        ptr_oam_enc_state, &ptr_oam_enc_state->str_bit_buf_dynm);

    if (bits_written_dynm < bits_written_intra)
    {
      if (bits_written_dynm > 0)
      {
        bits_written += impeghe_write_bits_buf(it_bit_buf, 0, 1);
        bits_written += impeghe_obj_md_bitbuf_write(
            it_bit_buf, ptr_oam_enc_state->bit_buf_base_dynm, bits_written_dynm);
      }
    }
    else
    {
      bits_written += impeghe_write_bits_buf(it_bit_buf, 1, 1);
      bits_written += impeghe_obj_md_bitbuf_write(
          it_bit_buf, ptr_oam_enc_state->bit_buf_base_intra, bits_written_intra);
    }
  }

  impeghe_obj_md_update_last_sample_ld(ptr_oam_enc_state);
  return bits_written;
}

/**
 *  impeghe_obj_md_encode_frame_ld
 *
 *  \brief Ecnodes one oam frame in low delay mode.
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in]		is_iframe			Flag to indicate i-frame
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32 impeghe_obj_md_encode_frame_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                             WORD32 is_iframe, ia_bit_buf_struct *it_bit_buf)
{
  WORD32 bits_written = 0;
  impeghe_obj_md_round_multidata_frame_ld(&ptr_oam_enc_state->str_oam_data_inp);

  bits_written += impeghe_obj_md_compress_oam_data_ld(ptr_oam_enc_state, is_iframe, it_bit_buf);

  return bits_written;
}

/**
 *  impeghe_obj_md_low_delay_enc
 *
 *  \brief Object Metadata bit stream encoder for low delay mode
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in]		is_iframe			Flag to indicate i-frame
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *  \param [out]	bits_written	Pointer to bit-count
 *
 *  \return WORD32
 *
 */
WORD32 impeghe_obj_md_low_delay_enc(ia_oam_enc_state_struct *ptr_oam_enc_state, WORD32 is_iframe,
                                    ia_bit_buf_struct *it_bit_buf, WORD32 *bits_written)
{
  WORD32 err_code = 0;
  LOOPIDX i;
  WORD32 object_metadata_present = 0;
  WORD32 bits_written_local = 0;

  for (i = 0; i < ptr_oam_enc_state->frame_factor; i++)
  {
    if (ptr_oam_enc_state->is_elem_read == 0)
    {
      err_code = impeghe_obj_md_read_frame(ptr_oam_enc_state);
      if (err_code & IA_FATAL_ERROR)
      {
        if (err_code == IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED)
        {
          return IMPEGHE_CONFIG_NONFATAL_OAM_NOT_AVAILABLE;
        }
        else
        {
          return err_code;
        }
      }
    }

    ptr_oam_enc_state->is_elem_read = 0;

    err_code = impeghe_obj_md_get_scaled_chunk_ld(ptr_oam_enc_state,
                                                  &ptr_oam_enc_state->str_oam_data_inp, 1);
    if (err_code & IA_FATAL_ERROR)
    {
      return err_code;
    }

    if (ptr_oam_enc_state->frame_factor > 1)
    {
      bits_written_local = 0;
      object_metadata_present = 0;
      impeghe_reset_bit_buffer(&ptr_oam_enc_state->str_bit_buf_ld);

      bits_written_local += impeghe_obj_md_encode_frame_ld(ptr_oam_enc_state, is_iframe,
                                                           &ptr_oam_enc_state->str_bit_buf_ld);

      if (bits_written_local > 0)
      {
        object_metadata_present = 1;
      }
      *bits_written += impeghe_write_bits_buf(it_bit_buf, object_metadata_present, 1);
      *bits_written += impeghe_obj_md_bitbuf_write(it_bit_buf, ptr_oam_enc_state->bit_buf_base_ld,
                                                   bits_written_local);
    }
    else
    {
      *bits_written += impeghe_obj_md_encode_frame_ld(ptr_oam_enc_state, is_iframe, it_bit_buf);

      if (ptr_oam_enc_state->sub_sample > 1)
      {
        ptr_oam_enc_state->str_config.skip_data(ptr_oam_enc_state->str_config.data_hndl,
                                                ptr_oam_enc_state->bytes_per_element *
                                                    ptr_oam_enc_state->str_config.num_objects *
                                                    (ptr_oam_enc_state->sub_sample - 1));
      }
    }
  }

  return err_code;
}
