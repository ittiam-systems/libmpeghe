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
#include <stdio.h>
#include <stdlib.h>

#include "impeghe_type_def.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_config_defines.h"
/**
 *  impeghe_read_asi
 *
 *  \brief Read audio scene information elements
 *
 *  \param [out]    pstr_asi_config     pointer to asi config structure
 *  \param [in]     file                pointer to asi input text file
 *
 *  \return WORD32       error code
 *
 */

WORD32 impeghe_read_asi(ia_asi_config *pstr_asi_config, FILE *file)
{
  WORD32 num_positions = 0;
  WORD32 i, j, k;
  WORD8 ch;
  while ((ch = fgetc(file)) != EOF)
  {
    if (ch == '\n')
      num_positions++;
  }
  fseek(file, 0, SEEK_SET);
  WORD8 line[MAX_MAE_CONFIG_LINE_LEN];
  WORD32 ii = 0;
  while (ii < num_positions)
  {
    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
    {
      ii++;
    }
    if (strncmp((pCHAR8)line, STRING_MAIN_STREAM_FLAG, strlen(STRING_MAIN_STREAM_FLAG)) == 0)
    {
      pstr_asi_config->main_stream_flag =
          (atoi((const pCHAR8)&line[strlen(STRING_MAIN_STREAM_FLAG)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
    }
    if (strncmp((pCHAR8)line, STRING_MAE_ID_OFFSET, strlen(STRING_MAE_ID_OFFSET)) == 0)
    {
      pstr_asi_config->mae_id_offset = (atoi((const pCHAR8)&line[strlen(STRING_MAE_ID_OFFSET)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
    }
    if (strncmp((pCHAR8)line, STRING_MAE_ID_MAX_AVAIL, strlen(STRING_MAE_ID_MAX_AVAIL)) == 0)
    {
      pstr_asi_config->mae_id_offset =
          (atoi((const pCHAR8)&line[strlen(STRING_MAE_ID_MAX_AVAIL)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
    }
    if (strncmp((pCHAR8)line, STRING_NUM_GROUPS, strlen(STRING_NUM_GROUPS)) == 0)
    {
      pstr_asi_config->num_groups = (atoi((const pCHAR8)&line[strlen(STRING_NUM_GROUPS)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }

      for (i = 0; i < pstr_asi_config->num_groups; i++)
      {
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_GROUP_ID, strlen(STRING_GROUP_DFN_GROUP_ID)) ==
            0)
        {
          pstr_asi_config->grp_def_grp_id[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_GROUP_ID)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_ON_OFF,
                    strlen(STRING_GROUP_DFN_ALLOW_ON_OFF)) == 0)
        {
          pstr_asi_config->grp_def_allow_on_off[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_ALLOW_ON_OFF)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_DEFAULT_ON_OFF,
                    strlen(STRING_GROUP_DFN_DEFAULT_ON_OFF)) == 0)
        {
          pstr_asi_config->grp_def_default_on_off[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_DEFAULT_ON_OFF)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_POS_INTERACT,
                    strlen(STRING_GROUP_DFN_ALLOW_POS_INTERACT)) == 0)
        {
          pstr_asi_config->grp_def_allow_pos_interact[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_ALLOW_POS_INTERACT)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_AZ_OFFSET,
                    strlen(STRING_GROUP_DFN_MIN_AZ_OFFSET)) == 0)
        {
          pstr_asi_config->grp_def_min_az_offset[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MIN_AZ_OFFSET)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_AZ_OFFSET,
                    strlen(STRING_GROUP_DFN_MAX_AZ_OFFSET)) == 0)
        {
          pstr_asi_config->grp_def_max_az_offset[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MAX_AZ_OFFSET)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_EL_OFFSET,
                    strlen(STRING_GROUP_DFN_MIN_EL_OFFSET)) == 0)
        {
          pstr_asi_config->grp_def_min_el_offset[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MIN_EL_OFFSET)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_EL_OFFSET,
                    strlen(STRING_GROUP_DFN_MAX_EL_OFFSET)) == 0)
        {
          pstr_asi_config->grp_def_max_el_offset[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MAX_EL_OFFSET)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_DIST_FACTOR,
                    strlen(STRING_GROUP_DFN_MIN_DIST_FACTOR)) == 0)
        {
          pstr_asi_config->grp_def_min_dist_factor[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MIN_DIST_FACTOR)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_DIST_FACTOR,
                    strlen(STRING_GROUP_DFN_MAX_DIST_FACTOR)) == 0)
        {
          pstr_asi_config->grp_def_max_dist_factor[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MAX_DIST_FACTOR)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_GAIN_FACTOR,
                    strlen(STRING_GROUP_DFN_ALLOW_GAIN_FACTOR)) == 0)
        {
          pstr_asi_config->grp_def_allow_gain_interact[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_ALLOW_GAIN_FACTOR)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_GAIN, strlen(STRING_GROUP_DFN_MIN_GAIN)) ==
            0)
        {
          pstr_asi_config->grp_def_min_gain[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MIN_GAIN)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_GAIN, strlen(STRING_GROUP_DFN_MAX_GAIN)) ==
            0)
        {
          pstr_asi_config->grp_def_max_gain[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_MAX_GAIN)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_HAS_CONJUNCT_MEM,
                    strlen(STRING_GROUP_DFN_HAS_CONJUNCT_MEM)) == 0)
        {
          pstr_asi_config->grp_def_start_id[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_HAS_CONJUNCT_MEM)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
          if (strncmp((pCHAR8)line, STRING_GROUP_DFN_START_ID,
                      strlen(STRING_GROUP_DFN_START_ID)) == 0)
          {
            pstr_asi_config->grp_def_start_id[i] =
                (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_START_ID)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_GRP_NUM, strlen(STRING_GROUP_DFN_GRP_NUM)) ==
            0)
        {
          pstr_asi_config->grp_def_group_num[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_GRP_NUM)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
          for (j = 0; j < pstr_asi_config->grp_def_group_num[i]; j++)
          {
            if (strncmp((pCHAR8)line, STRING_GROUP_DFN_META_ELE_ID,
                        strlen(STRING_GROUP_DFN_META_ELE_ID)) == 0)
            {
              pstr_asi_config->grp_def_metadata_ele_id[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DFN_META_ELE_ID)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
          }
        }
      }
    }
    if (strncmp((pCHAR8)line, STRING_NUM_SWITCH_GRP, strlen(STRING_NUM_SWITCH_GRP)) == 0)
    {
      pstr_asi_config->num_switch_groups =
          (atoi((const pCHAR8)&line[strlen(STRING_NUM_SWITCH_GRP)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
      for (i = 0; i < pstr_asi_config->num_switch_groups; i++)
      {
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_GRP_ID,
                    strlen(STRING_SWITCH_GRP_DEFN_GRP_ID)) == 0)
        {
          pstr_asi_config->switch_grp_def_grp_id[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_GRP_ID)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF,
                    strlen(STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF)) == 0)
        {
          pstr_asi_config->switch_grp_def_allow_on_off[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF,
                    strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF)) == 0)
        {
          pstr_asi_config->switch_grp_def_default_on_off[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM,
                    strlen(STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM)) == 0)
        {
          pstr_asi_config->switch_grp_def_grp_num_member[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
          for (j = 0; j < pstr_asi_config->switch_grp_def_grp_num_member[i]; j++)
          {
            if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_MEM_ID,
                        strlen(STRING_SWITCH_GRP_DEFN_MEM_ID)) == 0)
            {
              pstr_asi_config->switch_grp_def_grp_member_id[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_MEM_ID)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
          }
        }

        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID,
                    strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID)) == 0)
        {
          pstr_asi_config->switch_grp_def_default_group_id[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
      }
    }
    if (strncmp((pCHAR8)line, STRING_NUM_GRP_PRESETS, strlen(STRING_NUM_GRP_PRESETS)) == 0)
    {
      pstr_asi_config->num_group_presets =
          (atoi((const pCHAR8)&line[strlen(STRING_NUM_GRP_PRESETS)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
      for (i = 0; i < pstr_asi_config->num_group_presets; i++)
      {
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GRP_ID,
                    strlen(STRING_GRP_PRESET_DEFN_GRP_ID)) == 0)
        {
          pstr_asi_config->grp_preset_def_grp_id[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GRP_ID)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_PRESET_KIND,
                    strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)) == 0)
        {
          pstr_asi_config->grp_preset_def_preset_kind[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_NUM_CONDITION,
                    strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)) == 0)
        {
          pstr_asi_config->grp_preset_def_num_conditions[i] =
              (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
          for (j = 0; j < pstr_asi_config->grp_preset_def_num_conditions[i]; j++)
          {
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_REF_ID,
                        strlen(STRING_GRP_PRESET_DEFN_REF_ID)) == 0)
            {
              pstr_asi_config->grp_preset_def_reference_id[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_REF_ID)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_CON_ON_OFF,
                        strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)) == 0)
            {
              pstr_asi_config->grp_preset_def_cond_on_off[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN_FLAG,
                        strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)) == 0)
            {
              pstr_asi_config->grp_preset_def_gain_flag[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN,
                        strlen(STRING_GRP_PRESET_DEFN_GAIN)) == 0)
            {
              pstr_asi_config->grp_preset_def_gain[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GAIN)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)) == 0)
            {
              pstr_asi_config->grp_preset_def_disable_gain_interact[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)) == 0)
            {
              pstr_asi_config->grp_preset_def_disable_position_interact[i][j] = (atoi(
                  (const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)) == 0)
            {
              pstr_asi_config->grp_preset_def_position_interact[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_AZ_OFFSET,
                        strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)) == 0)
            {
              pstr_asi_config->grp_preset_def_azimuth_offset[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_EL_OFFSET,
                        strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)) == 0)
            {
              pstr_asi_config->grp_preset_def_elevation_offset[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DIST_FACTOR,
                        strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)) == 0)
            {
              pstr_asi_config->grp_preset_def_dist_factor[i][j] =
                  (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
          }
        }
      }
    }

    if (strncmp((pCHAR8)line, STRING_NUM_DATA_SETS, strlen(STRING_NUM_DATA_SETS)) == 0)
    {
      pstr_asi_config->num_data_sets = (atoi((const pCHAR8)&line[strlen(STRING_NUM_DATA_SETS)]));
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
      {
        ii++;
      }
      for (i = 0; i < pstr_asi_config->num_data_sets; i++)
      {
        if (strncmp((pCHAR8)line, STRING_DATA_TYPE, strlen(STRING_DATA_TYPE)) == 0)
        {
          pstr_asi_config->data_type[i] = (atoi((const pCHAR8)&line[strlen(STRING_DATA_TYPE)]));
          memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
          if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
          {
            ii++;
          }
        }
        if (pstr_asi_config->data_type[i] == 0)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS,
                      strlen(STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS)) == 0)
          {
            pstr_asi_config->num_grp_def_decription_blocks =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < pstr_asi_config->num_grp_def_decription_blocks; j++)
            {
              if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_GRP_ID,
                          strlen(STRING_GROUP_DEF_DSCRPTN_GRP_ID)) == 0)
              {
                pstr_asi_config->grp_def_decription_grp_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DEF_DSCRPTN_GRP_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                pstr_asi_config->num_grp_def_decription_languages[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
                for (WORD32 k = 0; k < pstr_asi_config->num_grp_def_decription_languages[j]; k++)
                {
                  if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_LANGUAGES,
                              strlen(STRING_GROUP_DEF_DSCRPTN_LANGUAGES)) == 0)
                  {
                    pstr_asi_config->grp_def_decription_languages[j][k] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DEF_DSCRPTN_LANGUAGES)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                    pstr_asi_config->grp_def_decription_data_length[j][k] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  for (WORD32 l = 0; l < pstr_asi_config->grp_def_decription_data_length[j][k];
                       l++)
                  {
                    if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_DATA,
                                strlen(STRING_GROUP_DEF_DSCRPTN_DATA)) == 0)
                    {
                      pstr_asi_config->grp_def_decription_data[j][k][l] =
                          (atoi((const pCHAR8)&line[strlen(STRING_GROUP_DEF_DSCRPTN_DATA)]));
                      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                      {
                        ii++;
                      }
                    }
                  }
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 1)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_SWITCH_GRP_DSCRPTN_BLOCKS,
                      strlen(STRING_NUM_SWITCH_GRP_DSCRPTN_BLOCKS)) == 0)
          {
            pstr_asi_config->num_switch_grp_decription_blocks =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_SWITCH_GRP_DSCRPTN_BLOCKS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < pstr_asi_config->num_switch_grp_decription_blocks; j++)
            {
              if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_GRP_ID,
                          strlen(STRING_SWITCH_GRP_DSCRPTN_GRP_ID)) == 0)
              {
                pstr_asi_config->switch_grp_decription_grp_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DSCRPTN_GRP_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                pstr_asi_config->switch_grp_num_decription_languages[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
                for (WORD32 k = 0; k < pstr_asi_config->switch_grp_num_decription_languages[j];
                     k++)
                {
                  if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_LANGUAGES,
                              strlen(STRING_SWITCH_GRP_DSCRPTN_LANGUAGES)) == 0)
                  {
                    pstr_asi_config->switch_grp_decription_languages[j][k] =
                        (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DSCRPTN_LANGUAGES)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                    pstr_asi_config->switch_grp_decription_data_length[j][k] = (atoi(
                        (const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  for (WORD32 l = 0; l < pstr_asi_config->switch_grp_decription_data_length[j][k];
                       l++)
                  {
                    if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_DATA,
                                strlen(STRING_SWITCH_GRP_DSCRPTN_DATA)) == 0)
                    {
                      pstr_asi_config->switch_grp_decription_data[j][k][l] =
                          (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_DSCRPTN_DATA)]));
                      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                      {
                        ii++;
                      }
                    }
                  }
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 5)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_PRESET_DSCRPTN_BLOCKS,
                      strlen(STRING_NUM_PRESET_DSCRPTN_BLOCKS)) == 0)
          {
            pstr_asi_config->num_preset_decription_blocks =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_PRESET_DSCRPTN_BLOCKS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < pstr_asi_config->num_preset_decription_blocks; j++)
            {
              if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_GRP_ID,
                          strlen(STRING_PRESET_DSCRPTN_GRP_ID)) == 0)
              {
                pstr_asi_config->grp_preset_def_grp_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_PRESET_DSCRPTN_GRP_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_PRESET_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_PRESET_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                pstr_asi_config->preset_num_decription_languages[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_PRESET_NUM_DSCRPTN_LANGUAGES)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
                for (WORD32 k = 0; k < pstr_asi_config->preset_num_decription_languages[j]; k++)
                {
                  if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_LANGUAGES,
                              strlen(STRING_PRESET_DSCRPTN_LANGUAGES)) == 0)
                  {
                    pstr_asi_config->preset_decription_languages[j][k] =
                        (atoi((const pCHAR8)&line[strlen(STRING_PRESET_DSCRPTN_LANGUAGES)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_PRESET_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                    pstr_asi_config->preset_decription_data_length[j][k] =
                        (atoi((const pCHAR8)&line[strlen(STRING_PRESET_DSCRPTN_DATA_LENGTH)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  for (WORD32 l = 0; l < pstr_asi_config->preset_decription_data_length[j][k];
                       l++)
                  {
                    if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_DATA,
                                strlen(STRING_PRESET_DSCRPTN_DATA)) == 0)
                    {
                      pstr_asi_config->preset_decription_data[j][k][l] =
                          (atoi((const pCHAR8)&line[strlen(STRING_PRESET_DSCRPTN_DATA)]));
                      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                      {
                        ii++;
                      }
                    }
                  }
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 2)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_CONTENT_DATA_BLOCKS,
                      strlen(STRING_NUM_CONTENT_DATA_BLOCKS)) == 0)
          {
            pstr_asi_config->num_content_data_blocks =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_CONTENT_DATA_BLOCKS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < pstr_asi_config->num_content_data_blocks; j++)
            {
              if (strncmp((pCHAR8)line, STRING_CONTENT_GRP_ID, strlen(STRING_CONTENT_GRP_ID)) ==
                  0)
              {
                pstr_asi_config->content_group_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_CONTENT_GRP_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_CONTENT_KIND, strlen(STRING_CONTENT_KIND)) == 0)
              {
                pstr_asi_config->content_kind[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_CONTENT_KIND)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_HAS_CONTENT_LANGUAGE,
                          strlen(STRING_HAS_CONTENT_LANGUAGE)) == 0)
              {
                pstr_asi_config->has_content_language[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_HAS_CONTENT_LANGUAGE)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_CONTENT_LANGUAGE,
                          strlen(STRING_CONTENT_LANGUAGE)) == 0)
              {
                pstr_asi_config->content_language[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_CONTENT_LANGUAGE)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 3)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_COMP_PAIRS, strlen(STRING_NUM_COMP_PAIRS)) == 0)
          {
            pstr_asi_config->num_comp_pairs =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_COMP_PAIRS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < 2 * pstr_asi_config->num_comp_pairs; j++)
            {
              if (strncmp((pCHAR8)line, STRING_ELEMENT_ID, strlen(STRING_ELEMENT_ID)) == 0)
              {
                pstr_asi_config->element_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_ELEMENT_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 4)
        {
          if (strncmp((pCHAR8)line, STRING_HAS_NON_STD_SCRN_SIZE,
                      strlen(STRING_HAS_NON_STD_SCRN_SIZE)) == 0)
          {
            pstr_asi_config->has_non_std_screen_size[i] =
                (atoi((const pCHAR8)&line[strlen(STRING_HAS_NON_STD_SCRN_SIZE)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_AZ, strlen(STRING_SCRN_SIZE_AZ)) == 0)
          {
            pstr_asi_config->screen_size_az =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_AZ)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_EL, strlen(STRING_SCRN_SIZE_EL)) == 0)
          {
            pstr_asi_config->screen_size_el =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_EL)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_BOT_EL, strlen(STRING_SCRN_SIZE_BOT_EL)) ==
              0)
          {
            pstr_asi_config->screen_size_bot_el[i] =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_BOT_EL)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 7)
        {

          if (strncmp((pCHAR8)line, STRING_OVERWRITE_PRO_SCRN_SIZE_DATA,
                      strlen(STRING_OVERWRITE_PRO_SCRN_SIZE_DATA)) == 0)
          {
            pstr_asi_config->overwrite_prod_screen_size_data =
                ((int)line[strlen(STRING_OVERWRITE_PRO_SCRN_SIZE_DATA)] - '0');
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_SCRN_SZ_LEFT_AZ,
                      strlen(STRING_DFLT_SCRN_SZ_LEFT_AZ)) == 0)
          {
            pstr_asi_config->default_screen_sz_left_az =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_SCRN_SZ_LEFT_AZ)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_SCRN_SZ_RIGHT_AZ,
                      strlen(STRING_DFLT_SCRN_SZ_RIGHT_AZ)) == 0)
          {
            pstr_asi_config->default_screen_sz_right_az =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_SCRN_SZ_RIGHT_AZ)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (strncmp((pCHAR8)line, STRING_NUM_PRESET_PROD_SCRNS,
                      strlen(STRING_NUM_PRESET_PROD_SCRNS)) == 0)
          {
            pstr_asi_config->num_preset_prod_screens =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_PRESET_PROD_SCRNS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (j = 0; j < pstr_asi_config->num_preset_prod_screens; j++)
            {
              if (strncmp((pCHAR8)line, STRING_SCRN_GRP_PRESET_ID,
                          strlen(STRING_SCRN_GRP_PRESET_ID)) == 0)
              {
                pstr_asi_config->screen_grp_preset_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_GRP_PRESET_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_CENTERED_IN_AZ, strlen(STRING_CENTERED_IN_AZ)) ==
                  0)
              {
                pstr_asi_config->centered_in_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_CENTERED_IN_AZ)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_LEFT_AZ, strlen(STRING_SCRN_SZ_LEFT_AZ)) ==
                  0)
              {
                pstr_asi_config->screen_sz_left_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_LEFT_AZ)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_RIGHT_AZ,
                          strlen(STRING_SCRN_SZ_RIGHT_AZ)) == 0)
              {
                pstr_asi_config->screen_sz_right_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_RIGHT_AZ)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_TOP_EL, strlen(STRING_SCRN_SZ_TOP_EL)) ==
                  0)
              {
                pstr_asi_config->screen_sz_top_el[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_TOP_EL)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 8)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_GRP_PRESETS, strlen(STRING_NUM_GRP_PRESETS)) == 0)
          {
            pstr_asi_config->num_group_presets =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_GRP_PRESETS)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
            for (i = 0; i < pstr_asi_config->num_group_presets; i++)
            {
              if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GRP_ID,
                          strlen(STRING_GRP_PRESET_DEFN_GRP_ID)) == 0)
              {
                pstr_asi_config->grp_preset_def_grp_id[i] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GRP_ID)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_PRESET_KIND,
                          strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)) == 0)
              {
                pstr_asi_config->grp_preset_def_preset_kind[i] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_NUM_CONDITION,
                          strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)) == 0)
              {
                pstr_asi_config->grp_preset_def_num_conditions[i] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
                for (j = 0; j < pstr_asi_config->grp_preset_def_num_conditions[i]; j++)
                {
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_REF_ID,
                              strlen(STRING_GRP_PRESET_DEFN_REF_ID)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_reference_id[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_REF_ID)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_CON_ON_OFF,
                              strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_cond_on_off[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN_FLAG,
                              strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_gain_flag[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN,
                              strlen(STRING_GRP_PRESET_DEFN_GAIN)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_gain[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_GAIN)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT,
                              strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_disable_gain_interact[i][j] = (atoi(
                        (const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT,
                              strlen(STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_disable_position_interact[i][j] =
                        (atoi((const pCHAR8)&line[strlen(
                            STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT,
                              strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_position_interact[i][j] = (atoi(
                        (const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_AZ_OFFSET,
                              strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_azimuth_offset[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_EL_OFFSET,
                              strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_elevation_offset[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                  if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DIST_FACTOR,
                              strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)) == 0)
                  {
                    pstr_asi_config->grp_preset_def_dist_factor[i][j] =
                        (atoi((const pCHAR8)&line[strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)]));
                    memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                    if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                    {
                      ii++;
                    }
                  }
                }
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 9)
        {
          if (pstr_asi_config->num_group_presets)
          {
            for (k = 0; k < pstr_asi_config->num_groups; k++)
            {
              if (strncmp((pCHAR8)line, STRING_GRP_LOUDNESS, strlen(STRING_GRP_LOUDNESS)) == 0)
              {
                pstr_asi_config->group_loudness[k] =
                    (atoi((const pCHAR8)&line[strlen(STRING_GRP_LOUDNESS)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_PARAM_PRESENT,
                      strlen(STRING_DFLT_PARAM_PRESENT)) == 0)
          {
            pstr_asi_config->default_params_present =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_PARAM_PRESENT)]));
            memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
            if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
            {
              ii++;
            }
          }
          if (pstr_asi_config->default_params_present)
          {
            for (k = 0; k < pstr_asi_config->num_groups; k++)
            {

              if (strncmp((pCHAR8)line, STRING_DFLT_INCL_GRP, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_include_group[k] =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_INCL_GRP)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
            if (strncmp((pCHAR8)line, STRING_DFLT_MIN_MAX_GAIN_PRESENT,
                        strlen(STRING_DFLT_INCL_GRP)) == 0)
            {
              pstr_asi_config->default_min_max_gain_present =
                  (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MIN_MAX_GAIN_PRESENT)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }

            if (pstr_asi_config->default_min_max_gain_present)
            {
              if (strncmp((pCHAR8)line, STRING_DFLT_MIN_GAIN, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_min_gain =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MIN_GAIN)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (strncmp((pCHAR8)line, STRING_DFLT_MAX_GAIN, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_max_gain =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MAX_GAIN)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
            }
          }
          for (k = 0; k < pstr_asi_config->num_group_presets; k++)
          {
            if (strncmp((pCHAR8)line, STRING_PRESET_PARAM_PRESENT,
                        strlen(STRING_DFLT_INCL_GRP)) == 0)
            {
              pstr_asi_config->preset_params_present[k] =
                  (atoi((const pCHAR8)&line[strlen(STRING_PRESET_PARAM_PRESENT)]));
              memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
              if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
              {
                ii++;
              }
            }
            if (pstr_asi_config->preset_params_present[k])
            {
              for (j = 0; j < pstr_asi_config->num_groups; j++)
              {
                if (strncmp((pCHAR8)line, STRING_PRESET_INCLUDE_GRP,
                            strlen(STRING_DFLT_INCL_GRP)) == 0)
                {
                  pstr_asi_config->preset_include_group[k][j] =
                      (atoi((const pCHAR8)&line[strlen(STRING_PRESET_INCLUDE_GRP)]));
                  memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                  if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                  {
                    ii++;
                  }
                }
              }
              if (strncmp((pCHAR8)line, STRING_PRESET_MIN_MAX_PRESENT,
                          strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->preset_min_max_gain_present[k] =
                    (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MIN_MAX_PRESENT)]));
                memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                {
                  ii++;
                }
              }
              if (pstr_asi_config->preset_min_max_gain_present[k])
              {
                if (strncmp((pCHAR8)line, STRING_PRESET_MIN_GAIN, strlen(STRING_DFLT_INCL_GRP)) ==
                    0)
                {
                  pstr_asi_config->preset_min_gain[k] =
                      (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MIN_GAIN)]));
                  memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                  if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                  {
                    ii++;
                  }
                }
                if (strncmp((pCHAR8)line, STRING_PRESET_MAX_GAIN, strlen(STRING_DFLT_INCL_GRP)) ==
                    0)
                {
                  pstr_asi_config->preset_max_gain[k] =
                      (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MAX_GAIN)]));
                  memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);
                  if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)
                  {
                    ii++;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  fclose(file);
  return 0;
}