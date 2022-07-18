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
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"

const WORD8 impeghe_compact_template_cicp13_to_cicp6[15 * 4] = {
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0};

const WORD8 impeghe_compact_template_cicp14_to_cicp6[5 * 4] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                               1, 0, 0, 0, 0, 1, 1, 0, 0, 0};

const WORD8 impeghe_compact_template_cicp12_to_cicp6[5 * 4] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                               1, 0, 0, 0, 0, 1, 0, 0, 0, 1};

const WORD8 impeghe_compact_template_cicp7_to_cicp6[5 * 4] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                              1, 0, 0, 0, 0, 1, 1, 0, 0, 1};

const WORD8 impeghe_compact_template_cicp13_to_cicp14[15 * 5] = {
    1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0};

const WORD8 impeghe_compact_template_cicp13_to_cicp12[15 * 5] = {
    1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1,
    0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0};

const WORD8 impeghe_compact_template_cicp13_to_cicp7[15 * 5] = {
    0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1,
    0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0};

const WORD8 impeghe_compact_template_cicp13_to_cicp2[15 * 1] = {1, 1, 1, 1, 1, 1, 1, 1,
                                                                1, 1, 1, 1, 1, 1, 1};

const WORD16 impeghe_compact_templates_input_index[] = {13, 13, 13, 13, 14, 12, 7, 13};

const WORD16 impeghe_compact_templates_output_index[] = {6, 14, 12, 7, 6, 6, 6, 2};

const pWORD8 impeghe_compact_templates_data[] = {
    (const pWORD8)&impeghe_compact_template_cicp13_to_cicp6,
    (const pWORD8)&impeghe_compact_template_cicp13_to_cicp14,
    (const pWORD8)&impeghe_compact_template_cicp13_to_cicp12,
    (const pWORD8)&impeghe_compact_template_cicp13_to_cicp7,
    (const pWORD8)&impeghe_compact_template_cicp14_to_cicp6,
    (const pWORD8)&impeghe_compact_template_cicp12_to_cicp6,
    (const pWORD8)&impeghe_compact_template_cicp7_to_cicp6,
    (const pWORD8)&impeghe_compact_template_cicp13_to_cicp2};

/**
 *  impeghe_dmx_coder_state_generate_gain_table
 *
 *  \brief Generates downmix coder state gain table
 *
 *  \param [in,out] coder_state  Pointer to downmix matrix coder state structure
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE
impeghe_dmx_coder_state_generate_gain_table(ia_dmx_mtx_coder_state_struct *coder_state)
{
  LOOPIDX idx, p_idx;
  WORD32 index = 0;
  WORD32 max_gain = coder_state->max_gain;
  WORD32 min_gain = coder_state->min_gain;
  WORD32 precision_level = coder_state->precision_level;
  FLOAT32 last_step, step;
  FLOAT64 f_idx;

  if (!(precision_level <= 2))
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }

  for (idx = 0; idx >= min_gain; idx -= 3)
  {
    coder_state->gain_table[index++] = (FLOAT32)idx;
  }
  for (idx = 3; idx <= max_gain; idx += 3)
  {
    coder_state->gain_table[index++] = (FLOAT32)idx;
  }

  for (idx = 0; idx >= min_gain; idx--)
  {
    if ((idx % 3) != 0)
    {
      coder_state->gain_table[index++] = (FLOAT32)idx;
    }
  }

  for (idx = 1; idx <= max_gain; idx++)
  {
    if ((idx % 3) != 0)
    {
      coder_state->gain_table[index++] = (FLOAT32)idx;
    }
  }

  last_step = 1.0f;
  for (p_idx = 1; p_idx <= precision_level; p_idx++)
  {
    step = 1.0f / (FLOAT32)(1 << p_idx);

    for (f_idx = 0; f_idx >= min_gain; f_idx -= step)
    {
      if (fmod(f_idx, last_step) != 0.0f)
      {
        coder_state->gain_table[index++] = (FLOAT32)f_idx;
      }
    }
    for (f_idx = step; f_idx <= max_gain; f_idx += step)
    {
      if (fmod(f_idx, last_step) != 0.0f)
      {
        coder_state->gain_table[index++] = (FLOAT32)f_idx;
      }
    }
    last_step = step;
  }

  coder_state->gain_table[index++] = IMPEGHE_DMX_MATRIX_GAIN_ZERO;
  coder_state->gain_table_size = index;

  if (!(coder_state->gain_table_size == ((max_gain - min_gain) << precision_level) + 2))
  {
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_dmx_find_compact_template
 *
 *  \brief Find downmix compact template
 *
 *  \param [in] input_index		Input index value
 *  \param [in] output_index	Output index value
 *  \param [out] err_code			Pointer to error code
 *
 *  \return pWORD8  Pointer to compact template data
 */
pWORD8 impeghe_dmx_find_compact_template(WORD32 input_index, WORD32 output_index,
                                         WORD32 *err_code)
{
  LOOPIDX idx;
  WORD32 template_size;

  if (!((input_index != -1) && (output_index != -1)))
  {
    *err_code = IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
    return NULL;
  }

  template_size = (WORD32)(sizeof(impeghe_compact_templates_data) /
                           sizeof(impeghe_compact_templates_data[0]));

  for (idx = 0; idx < template_size; idx++)
  {
    if ((impeghe_compact_templates_input_index[idx] != input_index) ||
        (impeghe_compact_templates_output_index[idx] != output_index))
    {
      continue;
    }
    return (pWORD8)impeghe_compact_templates_data[idx];
  }

  return NULL;
}

/**
 *  impeghe_dmx_convert_to_compact_config
 *
 *  \brief Convert downmix speaker configuration to compact speaker configuration
 *
 *  \param [in] input_count            Number of speakers
 *  \param [in,out] pstr_input_config  Pointer to input downmix speaker information structure
 *  \param [out] ptr_compact_input_count  Pointer to number of speakers after compact conversion
 *  \param [out] pstr_compact_input_config	Pointer to compact input downmix speaker
 * information structure
 *
 *  \return VOID
 */
VOID impeghe_dmx_convert_to_compact_config(
    WORD32 input_count, ia_dmx_speaker_information_struct *pstr_input_config,
    WORD32 *ptr_compact_input_count,
    ia_dmx_speaker_information_struct *pstr_compact_input_config[])
{
  LOOPIDX i, j;

  for (i = input_count - 1; i >= 0; i--)
  {
    pstr_input_config[i].is_already_used = 0;
    pstr_input_config[i].original_position = (WORD16)i;
    pstr_input_config[i].pstr_symmetric_pair = NULL;
  }

  *ptr_compact_input_count = 0;
  for (i = 0; i < input_count; i++)
  {
    if (pstr_input_config[i].is_already_used)
    {
      continue;
    }
    if ((pstr_input_config[i].azimuth != 0) && (abs(pstr_input_config[i].azimuth) != 180))
    {
      for (j = i + 1; j < input_count; j++)
      {
        if (pstr_input_config[j].is_already_used)
        {
          continue;
        }
        if ((pstr_input_config[i].is_lfe == pstr_input_config[j].is_lfe) &&
            (pstr_input_config[i].elevation == pstr_input_config[j].elevation) &&
            (pstr_input_config[i].azimuth == -pstr_input_config[j].azimuth))
        {

          if (pstr_input_config[i].azimuth <= 0)
          {
            pstr_compact_input_config[(*ptr_compact_input_count)++] = &(pstr_input_config[j]);
            pstr_input_config[j].pstr_symmetric_pair = &(pstr_input_config[i]);
            pstr_input_config[j].pair_type = SP_PAIR_SYMMETRIC;
            pstr_input_config[i].pstr_symmetric_pair = NULL;
            pstr_input_config[i].pair_type = SP_PAIR_NONE;
          }
          else
          {
            pstr_compact_input_config[(*ptr_compact_input_count)++] = &(pstr_input_config[i]);
            pstr_input_config[i].pstr_symmetric_pair = &(pstr_input_config[j]);
            pstr_input_config[i].pair_type = SP_PAIR_SYMMETRIC;
            pstr_input_config[j].pstr_symmetric_pair = NULL;
            pstr_input_config[j].pair_type = SP_PAIR_NONE;
          }

          pstr_input_config[j].is_already_used = 1;
          pstr_input_config[i].is_already_used = 1;
          break;
        }
      }

      if (!pstr_input_config[i].is_already_used)
      {
        pstr_compact_input_config[(*ptr_compact_input_count)++] = &(pstr_input_config[i]);
        pstr_input_config[i].pstr_symmetric_pair = NULL;
        pstr_input_config[i].pair_type = SP_PAIR_SINGLE;
        pstr_input_config[i].is_already_used = 1;
      }
    }
    else
    {
      pstr_compact_input_config[(*ptr_compact_input_count)++] = &(pstr_input_config[i]);
      pstr_input_config[i].pstr_symmetric_pair = NULL;
      pstr_input_config[i].pair_type = SP_PAIR_CENTER;
      pstr_input_config[i].is_already_used = 1;
    }
  }
}
