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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_tables.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_drc_enc.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_drc_user_config.h"

/**
 *  impeghe_drc_get_float_value
 *
 *  \brief Gets floating point value from the DRC config parameters file
 *
 *  \param [in,out] fp     File pointer
 *
 *  \return FLAOT32    floating point value read from file
 */
static FLOAT32 impeghe_drc_get_float_value(FILE *fp)
{
  WORD32 i = 0;
  FLOAT32 result = 0.0f;
  CHAR8 line[1024];
  pCHAR8 retval;
  retval = fgets(line, sizeof(line), fp);
  if (retval)
  {
    pCHAR8 c = line;
    while ((line[0] == '#' || line[0] == '\n') && (retval != NULL))
    {
      retval = fgets(line, sizeof(line), fp);
    }
    while (line[i] != ':')
    {
      c++;
      i++;
    }
    c++;
    result = (FLOAT32)atof(c);
  }
  return result;
}

/**
 *  impeghe_drc_get_integer_value
 *
 *  \brief Gets floating point value from the DRC config parameters file
 *
 *  \param [in,out] fp     File pointer
 *
 *  \return WORD32    integer value read from file
 */
static WORD32 impeghe_drc_get_integer_value(FILE *fp)
{
  WORD32 i = 0;
  WORD32 result = 0;
  CHAR8 line[1024];
  pCHAR8 retval;
  retval = fgets(line, sizeof(line), fp);
  if (retval)
  {
    pCHAR8 c = line;
    while ((line[0] == '#' || line[0] == '\n') && (retval != NULL))
    {
      retval = fgets(line, sizeof(line), fp);
    }
    while (line[i] != ':')
    {
      c++;
      i++;
    }
    c++;
    if (c[0] == '0' && c[1] == 'x')
    {
      result = (WORD32)strtol(c, NULL, 16);
    }
    else
    {
      result = atoi(c);
    }
  }
  return result;
}

/**
 *  impeghe_read_drc_config_params
 *
 *  \brief Reads DRC parameters from DRC config params text file
 *
 *  \param [in,out] fp                           File pointer
 *  \param [in,out] pstr_enc_params              Pointer to DRC encoder parameters structure
 *  \param [in,out] pstr_uni_drc_config          Pointer to DRC config information structure
 *  \param [in,out] pstr_enc_loudness_info_set   Pointer to DRC loudness information structure
 *  \param [in,out] pstr_enc_gain_extension      Pointer to DRC gain extension structure
 *  \param [in] pstr_ext_cfg_downmix_input       Pointer to downmix input extension config
 * structure
 *
 *  \return VOID
 */
VOID impeghe_read_drc_config_params(
    FILE *fp, ia_drc_enc_params_struct *pstr_enc_params,
    ia_drc_uni_drc_config_struct *pstr_uni_drc_config,
    ia_drc_loudness_info_set_struct *pstr_enc_loudness_info_set,
    ia_drc_uni_drc_gain_ext_struct *pstr_enc_gain_extension,
    ia_mpeghe_ext_cfg_downmix_input_struct *pstr_ext_cfg_downmix_input)
{
  WORD32 n, g, s, m, ch, p;
  WORD32 gain_set_channels;

  pstr_enc_params->frame_size = 1024;
  pstr_enc_params->delay_mode = DELAY_MODE_REGULAR_DELAY;

  pstr_uni_drc_config->str_drc_coefficients_uni_drc->drc_frame_size = pstr_enc_params->frame_size;
  pstr_uni_drc_config->sample_rate_present = 1;
  pstr_uni_drc_config->str_drc_coefficients_uni_drc->drc_frame_size_present = 0;
  pstr_uni_drc_config->loudness_info_set_present = 0;

  /***********  str_drc_instructions_uni_drc  *************/

  pstr_uni_drc_config->drc_instructions_uni_drc_count = impeghe_drc_get_integer_value(fp);
  for (n = 0; n < pstr_uni_drc_config->drc_instructions_uni_drc_count; n++)
  {
    ia_drc_instructions_uni_drc *pstr_drc_instructions_uni_drc =
        &pstr_uni_drc_config->str_drc_instructions_uni_drc[n];
    pstr_drc_instructions_uni_drc->drc_set_id = n + 1;
    pstr_drc_instructions_uni_drc->downmix_id = impeghe_drc_get_integer_value(fp);
    pstr_drc_instructions_uni_drc->additional_downmix_id_present = 0;
    pstr_drc_instructions_uni_drc->additional_downmix_id_count = 0;
    pstr_drc_instructions_uni_drc->drc_location = 1;
    pstr_drc_instructions_uni_drc->depends_on_drc_set_present = 0;
    pstr_drc_instructions_uni_drc->depends_on_drc_set = 0;
    pstr_drc_instructions_uni_drc->no_independent_use = 0;
    pstr_drc_instructions_uni_drc->drc_set_effect = impeghe_drc_get_integer_value(fp);
    pstr_drc_instructions_uni_drc->drc_set_target_loudness_present = 0;
    pstr_drc_instructions_uni_drc->drc_set_target_loudness_value_upper = 0;
    pstr_drc_instructions_uni_drc->drc_set_target_loudness_value_lower_present = 0;
    pstr_drc_instructions_uni_drc->drc_set_target_loudness_value_lower = 0;

    gain_set_channels = impeghe_drc_get_integer_value(fp);
    for (ch = 0; ch < gain_set_channels; ch++)
    {
      pstr_drc_instructions_uni_drc->gain_set_index[ch] = impeghe_drc_get_integer_value(fp);
    }
    for (; ch < MAX_CHANNEL_COUNT; ch++)
    {
      pstr_drc_instructions_uni_drc->gain_set_index[ch] =
          pstr_drc_instructions_uni_drc->gain_set_index[gain_set_channels - 1];
    }

    pstr_drc_instructions_uni_drc->num_drc_channel_groups = impeghe_drc_get_integer_value(fp);

    for (g = 0; g < pstr_drc_instructions_uni_drc->num_drc_channel_groups; g++)
    {
      pstr_drc_instructions_uni_drc->str_gain_modifiers[g].gain_scaling_present[0] = 0;
      pstr_drc_instructions_uni_drc->str_gain_modifiers[g].attenuation_scaling[0] = 1.5f;
      pstr_drc_instructions_uni_drc->str_gain_modifiers[g].amplification_scaling[0] = 1.5f;
      pstr_drc_instructions_uni_drc->str_gain_modifiers[g].gain_offset_present[0] = 0;
      pstr_drc_instructions_uni_drc->str_gain_modifiers[g].gain_offset[0] = 16.0f;
    }

    pstr_drc_instructions_uni_drc->limiter_peak_target_present = 0;
    pstr_drc_instructions_uni_drc->limiter_peak_target = 0.0f;
    pstr_drc_instructions_uni_drc->drc_instructions_type = 0;
    pstr_drc_instructions_uni_drc->mae_group_id = 0;
    pstr_drc_instructions_uni_drc->mae_group_preset_id = 0;
  }

  /***********  str_drc_coefficients_uni_drc  *************/

  pstr_uni_drc_config->drc_coefficients_uni_drc_count = impeghe_drc_get_integer_value(fp);
  for (n = 0; n < pstr_uni_drc_config->drc_coefficients_uni_drc_count; n++)
  {
    ia_drc_coefficients_uni_drc_struct *pstr_drc_coefficients_uni_drc =
        &pstr_uni_drc_config->str_drc_coefficients_uni_drc[n];
    pstr_drc_coefficients_uni_drc->drc_location = 1;
    pstr_drc_coefficients_uni_drc->gain_set_count = impeghe_drc_get_integer_value(fp);

    for (s = 0; s < pstr_drc_coefficients_uni_drc->gain_set_count; s++)
    {
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_coding_profile = 0;
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_interpolation_type = 1;
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].full_frame = 0;
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].time_alignment = 0;
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].time_delta_min_present = 0;
      pstr_drc_coefficients_uni_drc->str_gain_set_params[s].band_count =
          impeghe_drc_get_integer_value(fp);
      if (pstr_drc_coefficients_uni_drc->str_gain_set_params[s].band_count == 1)
      {
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].nb_points =
            impeghe_drc_get_integer_value(fp);
        for (p = 0;
             p < pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].nb_points;
             p++)
        {
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].gain_points[p].x =
              impeghe_drc_get_float_value(fp);
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].gain_points[p].y =
              impeghe_drc_get_float_value(fp);
        }
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].width =
            impeghe_drc_get_float_value(fp);
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].attack =
            impeghe_drc_get_float_value(fp);
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].decay =
            impeghe_drc_get_float_value(fp);
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[0].drc_characteristic =
            0;
        pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
            .gain_params[0]
            .crossover_freq_index = 0;
      }
      else
      {
        for (m = 0; m < pstr_drc_coefficients_uni_drc->str_gain_set_params[s].band_count; m++)
        {
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[m].nb_points =
              impeghe_drc_get_integer_value(fp);
          for (p = 0;
               p < pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[m].nb_points;
               p++)
          {
            pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
                .gain_params[m]
                .gain_points[p]
                .x = impeghe_drc_get_float_value(fp);
            pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
                .gain_params[m]
                .gain_points[p]
                .y = impeghe_drc_get_float_value(fp);
          }
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[m].width =
              impeghe_drc_get_float_value(fp);
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[m].attack =
              impeghe_drc_get_float_value(fp);
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].gain_params[m].decay =
              impeghe_drc_get_float_value(fp);
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s].drc_band_type = 0;
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
              .gain_params[m]
              .start_sub_band_index = impeghe_drc_get_integer_value(fp);
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
              .gain_params[m]
              .drc_characteristic = 0;
          pstr_drc_coefficients_uni_drc->str_gain_set_params[s]
              .gain_params[m]
              .crossover_freq_index = 0;
        }
      }
    }
  }

  /***********  str_channel_layout  *************/

  pstr_uni_drc_config->str_channel_layout.layout_signaling_present = 0;
  pstr_uni_drc_config->str_channel_layout.defined_layout = 0;
  pstr_uni_drc_config->str_channel_layout.speaker_position[0] = 0;

  /***********  str_downmix_instructions  *************/

  pstr_uni_drc_config->downmix_instructions_count = pstr_ext_cfg_downmix_input->downmix_id_count;
  for (n = 0; n < pstr_uni_drc_config->downmix_instructions_count; n++)
  {
    pstr_uni_drc_config->str_downmix_instructions[n].target_layout =
        impeghe_drc_get_integer_value(fp);
    pstr_uni_drc_config->str_downmix_instructions[n].downmix_coefficients_present =
        impeghe_drc_get_integer_value(fp);
  }

  pstr_uni_drc_config->drc_description_basic_present = 0;
  pstr_uni_drc_config->uni_drc_config_ext_present = 0;
  pstr_enc_loudness_info_set->loudness_info_set_ext_present = 0;
  pstr_enc_gain_extension->uni_drc_gain_ext_present = 0;
}
