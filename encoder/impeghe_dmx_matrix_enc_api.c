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
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_dmx_matrix_enc_api.h"

/**
 *  impeghe_dmx_matrix_enc
 *
 *  \brief Encode downmix matrix
 *
 *  \param [in,out] it_bit_buf    Pointer to bit-buffer
 *  \param [in] pstr_enc_config	  Pointer to downmix matrix encoder config structure
 *  \param [out] pstr_dmx_scratch Pointer to scratch memory
 *  \param [out] ptr_bit_cnt      Pointer to bit count
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_dmx_matrix_enc(ia_bit_buf_struct *it_bit_buf,
                                    ia_dmx_matrix_enc_cfg_struct *pstr_enc_config,
                                    ia_dmx_sratch *pstr_dmx_scratch, WORD32 *ptr_bit_cnt)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  LOOPIDX i, j;
  WORD32 input_index = -1;
  WORD32 output_index = -1;
  WORD32 input_num_channels;
  WORD32 output_num_channels;
  WORD32 precision_level = 1;
  WORD32 eq_precision_level = 1;
  WORD32 num_lfe = 0;
  WORD32 bit_cnt_local = 0;
  FLOAT32 *matrix = NULL;

  input_index = pstr_enc_config->input_config_index;
  output_index = pstr_enc_config->output_config_index;

  if (pstr_enc_config->input_config_index <= 0 || pstr_enc_config->input_config_index >= 64)
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }
  else
  {
    err_code = impeghe_dmx_get_geometry_from_cicp(pstr_enc_config->input_config_index,
                                                  pstr_dmx_scratch->input_geometry,
                                                  &input_num_channels, &num_lfe);
    if (err_code)
    {
      return err_code;
    }
    input_num_channels += num_lfe;
  }
  for (i = 0; i < input_num_channels; i++)
  {
    pstr_dmx_scratch->input_config[i].azimuth = pstr_dmx_scratch->input_geometry[i].azimuth;
    pstr_dmx_scratch->input_config[i].elevation = pstr_dmx_scratch->input_geometry[i].elevation;
    pstr_dmx_scratch->input_config[i].is_lfe = pstr_dmx_scratch->input_geometry[i].is_lfe;
  }
  for (i = 0; i < input_num_channels - 1; i++)
  {
    ia_dmx_speaker_information_struct speaker_info = pstr_dmx_scratch->input_config[i];
    for (j = i + 1; j < input_num_channels; j++)
    {
      if ((speaker_info.azimuth == pstr_dmx_scratch->input_config[j].azimuth) &&
          (speaker_info.elevation == pstr_dmx_scratch->input_config[j].elevation) &&
          (speaker_info.is_lfe == pstr_dmx_scratch->input_config[j].is_lfe))
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
    }
  }

  if (pstr_enc_config->output_config_index <= 0 || pstr_enc_config->output_config_index >= 64)
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }
  else
  {
    err_code = impeghe_dmx_get_geometry_from_cicp(pstr_enc_config->output_config_index,
                                                  pstr_dmx_scratch->output_geometry,
                                                  &output_num_channels, &num_lfe);
    if (err_code)
    {
      return err_code;
    }
    output_num_channels += num_lfe;
  }
  for (i = 0; i < output_num_channels; i++)
  {
    pstr_dmx_scratch->output_config[i].azimuth = pstr_dmx_scratch->output_geometry[i].azimuth;
    pstr_dmx_scratch->output_config[i].elevation = pstr_dmx_scratch->output_geometry[i].elevation;
    pstr_dmx_scratch->output_config[i].is_lfe = pstr_dmx_scratch->output_geometry[i].is_lfe;
  }
  for (i = 0; i < output_num_channels - 1; i++)
  {
    ia_dmx_speaker_information_struct speaker_info = pstr_dmx_scratch->output_config[i];
    for (j = i + 1; j < output_num_channels; j++)
    {
      if ((speaker_info.azimuth == pstr_dmx_scratch->output_config[j].azimuth) &&
          (speaker_info.elevation == pstr_dmx_scratch->output_config[j].elevation) &&
          (speaker_info.is_lfe == pstr_dmx_scratch->output_config[j].is_lfe))
      {
        return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
      }
    }
  }

  matrix = (FLOAT32 *)pstr_enc_config->flat_downmix_matrix;

  err_code = impeghe_dmx_encode_downmix_matrix(
      input_index, input_num_channels, pstr_dmx_scratch->input_config, output_index,
      output_num_channels, pstr_dmx_scratch->output_config, precision_level, it_bit_buf, matrix,
      eq_precision_level, &pstr_enc_config->pstr_eq_config, pstr_dmx_scratch, &bit_cnt_local);

  if (err_code)
  {
    return err_code;
  }

  *ptr_bit_cnt += bit_cnt_local;

  return err_code;
}
