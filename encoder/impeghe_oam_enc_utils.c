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
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_cnst.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_oam_enc.h"
#include "impeghe_oam_enc_ld.h"
#include "impeghe_oam_enc_utils.h"

/**
 *  impeghe_obj_md_bitbuf_write
 *
 *  \brief Writes buffer data to bit buffer
 *
 *  \param [in,out]	it_bit_buf	Pointer to bit-buffer
 *  \param [in]		ptr_buf		Pointer to source buffer
 *  \param [in]		num_bits	Number of bits to be written
 *
 *  \return Number of bits written
 *
 */
WORD32 impeghe_obj_md_bitbuf_write(ia_bit_buf_struct *it_bit_buf, UWORD8 *ptr_buf,
                                   UWORD32 num_bits)
{
  WORD32 num_bits_written = 0;
  WORD32 num_full_bytes = num_bits >> 3;
  LOOPIDX idx = 0;

  if (num_bits <= 0)
  {
    return 0;
  }

  for (idx = 0; idx < num_full_bytes; idx++)
  {
    num_bits_written += impeghe_write_bits_buf(it_bit_buf, ptr_buf[idx], 8);
  }

  if (num_bits & 7)
  {
    num_bits_written +=
        impeghe_write_bits_buf(it_bit_buf, ptr_buf[idx] >> (8 - (num_bits & 7)), (num_bits & 7));
  }

  return num_bits_written;
}

/**
 *  impeghe_obj_md_scale_multidata_ld
 *
 *  \brief Scales OAM data
 *
 *  \param [in,out]	pstr_oam_data				Pointer to OAM multidata structure
 *  \param [in]		has_uniform_spread	flag to indicate uniform spread
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE impeghe_obj_md_scale_multidata_ld(ia_oam_enc_multidata_struct *pstr_oam_data,
                                                      WORD16 has_uniform_spread)
{
  WORD32 n;
  IA_ERRORCODE error_code = 0;

  for (n = 0; n < pstr_oam_data->num_elements; n++)
  {
    if (pstr_oam_data->azimuth[n] < AZIMUTH_MIN_DEGREE)
    {
      pstr_oam_data->azimuth[n] = AZIMUTH_MIN_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->azimuth[n] > AZIMUTH_MAX_DEGREE)
    {
      pstr_oam_data->azimuth[n] = AZIMUTH_MAX_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->azimuth[n] =
        OAM_BIT_LIMIT_SIGN_VALUE(SCALE_AZIMUTH(pstr_oam_data->azimuth[n]),
                                 OAM_BIT_LIMIT_SIGN_AZIMUTH_MIN, OAM_BIT_LIMIT_SIGN_AZIMUTH_MAX);

    if (pstr_oam_data->elevation[n] < ELEVATION_MIN_DEGREE)
    {
      pstr_oam_data->elevation[n] = ELEVATION_MIN_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->elevation[n] > ELEVATION_MAX_DEGREE)
    {
      pstr_oam_data->elevation[n] = ELEVATION_MAX_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->elevation[n] = OAM_BIT_LIMIT_SIGN_VALUE(
        SCALE_ELEVATION(pstr_oam_data->elevation[n]), OAM_BIT_LIMIT_SIGN_ELEVATION_MIN,
        OAM_BIT_LIMIT_SIGN_ELEVATION_MAX);

    if (pstr_oam_data->radius[n] < RADIUS_MIN_METER)
    {
      pstr_oam_data->radius[n] = RADIUS_MIN_METER;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->radius[n] > RADIUS_MAX_METER)
    {
      pstr_oam_data->radius[n] = RADIUS_MAX_METER;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->radius[n] = OAM_BIT_LIMIT_UNSIGN_VALUE(SCALE_RADIUS(pstr_oam_data->radius[n]),
                                                          OAM_BIT_LIMIT_UNSIGN_RADIUS_MAX);

    if (pstr_oam_data->gain[n] < GAIN_MIN_VALUE)
    {
      pstr_oam_data->gain[n] = GAIN_MIN_VALUE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->gain[n] > GAIN_MAX_VALUE)
    {
      pstr_oam_data->gain[n] = GAIN_MAX_VALUE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->gain[n] =
        OAM_BIT_LIMIT_SIGN_VALUE(SCALE_GAIN(pstr_oam_data->gain[n]), OAM_BIT_LIMIT_SIGN_GAIN_MIN,
                                 OAM_BIT_LIMIT_SIGN_GAIN_MAX);

    if (pstr_oam_data->spread[n] < SPREAD_MIN_DEGREE)
    {
      pstr_oam_data->spread[n] = SPREAD_MIN_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->spread[n] > SPREAD_MAX_DEGREE)
    {
      pstr_oam_data->spread[n] = SPREAD_MAX_DEGREE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->spread[n] = OAM_BIT_LIMIT_UNSIGN_VALUE(SCALE_SPREAD(pstr_oam_data->spread[n]),
                                                          OAM_BIT_LIMIT_UNSIGN_SPREAD_MAX);

    if (pstr_oam_data->dyn_obj_priority[n] < DYN_OBJ_PRIORITY_MIN_VALUE)
    {
      pstr_oam_data->dyn_obj_priority[n] = DYN_OBJ_PRIORITY_MIN_VALUE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    else if (pstr_oam_data->spread[n] > DYN_OBJ_PRIORITY_MAX_VALUE)
    {
      pstr_oam_data->dyn_obj_priority[n] = DYN_OBJ_PRIORITY_MAX_VALUE;
      error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
    }
    pstr_oam_data->dyn_obj_priority[n] = OAM_BIT_LIMIT_UNSIGN_VALUE(
        pstr_oam_data->dyn_obj_priority[n], OAM_BIT_LIMIT_UNSIGN_PRIORITY_MAX);
  }

  if (has_uniform_spread == 0)
  {
    for (n = 0; n < pstr_oam_data->num_elements; n++)
    {
      if (pstr_oam_data->spread_height[n] < SPREAD_HEIGHT_MIN_DEGREE)
      {
        pstr_oam_data->spread_height[n] = SPREAD_HEIGHT_MIN_DEGREE;
        error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
      }
      else if (pstr_oam_data->spread_height[n] > SPREAD_HEIGHT_MAX_DEGREE)
      {
        pstr_oam_data->spread_height[n] = SPREAD_HEIGHT_MAX_DEGREE;
        error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
      }
      pstr_oam_data->spread_height[n] =
          OAM_BIT_LIMIT_UNSIGN_VALUE(SCALE_SPREAD_HEIGHT(pstr_oam_data->spread_height[n]),
                                     OAM_BIT_LIMIT_UNSIGN_SPREAD_HEIGHT_MAX);

      if (pstr_oam_data->spread_depth[n] < SPREAD_DEPTH_MIN_METER)
      {
        pstr_oam_data->spread_depth[n] = SPREAD_DEPTH_MIN_METER;
        error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
      }
      else if (pstr_oam_data->spread_depth[n] > SPREAD_DEPTH_MAX_METER)
      {
        pstr_oam_data->spread_depth[n] = SPREAD_DEPTH_MAX_METER;
        error_code = IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE;
      }
      pstr_oam_data->spread_depth[n] =
          OAM_BIT_LIMIT_UNSIGN_VALUE(SCALE_SPREAD_DEPTH(pstr_oam_data->spread_depth[n]),
                                     OAM_BIT_LIMIT_UNSIGN_SPREAD_DEPTH_MAX);
    }
  }

  return error_code;
}

/**
 *  impeghe_obj_md_get_scaled_chunk_ld
 *
 *  \brief Replaces default values if required and scales OAM data
 *
 *  \param [in]		ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [out]	pstr_oam_data				Pointer to OAM multidata structure
 *  \param [unused]	num_samples
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_obj_md_get_scaled_chunk_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                                ia_oam_enc_multidata_struct *pstr_oam_data,
                                                WORD32 num_samples)
{
  FLOAT32 default_radius = 100.0f;
  WORD32 n;
  IA_ERRORCODE error_code;

  if (ptr_oam_enc_state->str_config.replace_radius)
  {
    for (n = 0; n < ptr_oam_enc_state->str_config.num_objects; n++)
    {
      pstr_oam_data->radius[n] = default_radius;
    }
  }

  error_code = impeghe_obj_md_scale_multidata_ld(
      pstr_oam_data, ptr_oam_enc_state->str_config.has_uniform_spread);

  return error_code;
}
