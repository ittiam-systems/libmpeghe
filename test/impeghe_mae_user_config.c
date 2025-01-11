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

#include <ctype.h>
#include <math.h>
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



#define READ_NEXT_LINE()                                                           \
  memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);                                        \
  if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)                  \
  {                                                                                \
    ii++;                                                                          \
  }                                                                                \
  do                                                                               \
  {                                                                                \
    if (line[0] == '#' || (line[0] == '/' && line[1] == '/') || isspace(line[0]))  \
    {                                                                              \
      memset(line, 0, MAX_MAE_CONFIG_LINE_LEN);                                    \
      if (fgets((pCHAR8)line, MAX_MAE_CONFIG_LINE_LEN, file) != NULL)              \
      {                                                                            \
        ii++;                                                                      \
      }                                                                            \
    }                                                                              \
    else                                                                           \
    {                                                                              \
      break;                                                                       \
    }                                                                              \
  } while (1);



/**
 *  impeghe_mae_read_csv_descr_data
 *
 *  \brief Read comma separated values (csv) from a string
 *
 *  \param [out]    ptr_out     pointer to asi config structure.
 *  \param [in]     ptr_string  pointer to string.
 *  \param [in]     max_len     maximum lenght of the buffer holding description data.
 *  \param [in]     ptr_len     pointer to buffer with actual length of description data.
 *  \param [in]     len         number of values to be extracted.
 *
 *  \return VOID       error code
 *
 */

static VOID impeghe_mae_read_csv_descr_data( pWORD8 ptr_out, pCHAR8 ptr_string, WORD32 max_len, WORD32 *ptr_len, WORD32 num_values)
{
  pCHAR8 p_separator = ",";
  pCHAR8 ptr_val = strtok(ptr_string, p_separator);
  for (WORD8 i = 0; i < num_values; i++)
  {
    for (WORD8 j = 0; j < ptr_len[i]; j++)
    {
      ptr_out[j] = ptr_val[j];
    }
    ptr_out += max_len;
    ptr_val = strtok(NULL, p_separator);
    if (ptr_val == NULL)
    {
      break;
    }
  }
  return ;
}

/**
 *  impeghe_mae_read_csv_char
 *
 *  \brief Read comma separated values (csv) from a string
 *
 *  \param [out]    ptr_out     pointer to asi config structure.
 *  \param [in]     ptr_string  pointer to string.
 *  \param [in]     len         number of values to be extracted.
 *
 *  \return VOID       error code
 *
 */

static VOID impeghe_mae_read_csv_lan( pWORD8 ptr_out, pCHAR8 ptr_string, WORD32 num_values)
{
  pCHAR8 p_separator = ",";
  pCHAR8 ptr_val = strtok(ptr_string, p_separator);
  for (WORD8 i = 0; i < num_values; i++)
  {
    ptr_out[0] = ptr_val[0];
    ptr_out[1] = ptr_val[1];
    ptr_out[2] = ptr_val[2];
    ptr_out += 3;
    ptr_val = strtok(NULL, p_separator);
    if (ptr_val == NULL)
    {
      break;
    }
  }
  return ;
}

/**
 *  impeghe_mae_read_csv_char
 *
 *  \brief Read comma separated values (csv) from a string
 *
 *  \param [out]    ptr_out     pointer to asi config structure.
 *  \param [in]     ptr_string  pointer to string.
 *  \param [in]     len         number of values to be extracted.
 *
 *  \return VOID
 *
 */

static VOID impeghe_mae_read_csv_float( pWORD32 ptr_out, pCHAR8 ptr_string, WORD32 num_values, FLOAT32 quant_fac, WORD32 offset)
{
  pCHAR8 p_separator = ",";
  pCHAR8 ptr_val = strtok(ptr_string, p_separator);
  for (WORD8 i = 0; i < num_values; i++)
  {
    FLOAT32 tmp = (FLOAT32)(atof(ptr_val) / (quant_fac)) + offset;
    ptr_out[i] = (WORD32) floor(tmp);
    ptr_val = strtok(NULL, p_separator);
    if (ptr_val == NULL)
    {
      break;
    }
  }
  return ;
}

/**
 *  impeghe_mae_read_csv_char
 *
 *  \brief Read comma separated values (csv) from a string
 *
 *  \param [out]    ptr_out     pointer to asi config structure.
 *  \param [in]     ptr_string  pointer to string.
 *  \param [in]     len         number of values to be extracted.
 *
 *  \return VOID
 *
 */

static VOID impeghe_mae_read_csv_char( pWORD32 ptr_out, pCHAR8 ptr_string, WORD32 num_values)
{
  pCHAR8 p_separator = ",";
  pCHAR8 ptr_val = strtok(ptr_string, p_separator);
  for (WORD8 i = 0; i < num_values; i++)
  {
    ptr_out[i] = atoi(ptr_val);
    ptr_val = strtok(NULL, p_separator);
    if (ptr_val == NULL)
    {
      break;
    }
  }
  return ;
}

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
  CHAR8 line[MAX_MAE_CONFIG_LINE_LEN];
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
      READ_NEXT_LINE()
    }
    if (strncmp((pCHAR8)line, STRING_MAE_ID_MAX_AVAIL, strlen(STRING_MAE_ID_MAX_AVAIL)) == 0)
    {
      pstr_asi_config->mae_id_max_avail =
          (atoi((const pCHAR8)&line[strlen(STRING_MAE_ID_MAX_AVAIL)]));
      READ_NEXT_LINE()
    }
    if (strncmp((pCHAR8)line, STRING_NUM_GROUPS, strlen(STRING_NUM_GROUPS)) == 0)
    {
      pstr_asi_config->num_groups = (atoi((const pCHAR8)&line[strlen(STRING_NUM_GROUPS)]));
      READ_NEXT_LINE()

        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_GROUP_ID, strlen(STRING_GROUP_DFN_GROUP_ID)) ==
            0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_GROUP_ID)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_grp_id[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_ON_OFF,
                    strlen(STRING_GROUP_DFN_ALLOW_ON_OFF)) == 0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_ALLOW_ON_OFF)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_allow_on_off[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_DEFAULT_ON_OFF,
                    strlen(STRING_GROUP_DFN_DEFAULT_ON_OFF)) == 0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_DEFAULT_ON_OFF)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_default_on_off[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_POS_INTERACT,
                    strlen(STRING_GROUP_DFN_ALLOW_POS_INTERACT)) == 0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_ALLOW_POS_INTERACT)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_allow_pos_interact[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_AZ_OFFSET,
                    strlen(STRING_GROUP_DFN_MIN_AZ_OFFSET)) == 0)
        {
          FLOAT32 quant = -1.5f;
          WORD32 offset = 0;
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MIN_AZ_OFFSET)];
          impeghe_mae_read_csv_float(&pstr_asi_config->grp_def_min_az_offset[0], ptr_val_string, pstr_asi_config->num_groups, quant, offset);
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_AZ_OFFSET,
                    strlen(STRING_GROUP_DFN_MAX_AZ_OFFSET)) == 0)
        {
          FLOAT32 quant = 1.5f;
          WORD32 offset = 0;
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MAX_AZ_OFFSET)];
          impeghe_mae_read_csv_float(&pstr_asi_config->grp_def_max_az_offset[0], ptr_val_string, pstr_asi_config->num_groups, quant, offset);
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_EL_OFFSET,
                    strlen(STRING_GROUP_DFN_MIN_EL_OFFSET)) == 0)
        {
          FLOAT32 quant = -3.0f;
          WORD32 offset = 0;
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MIN_EL_OFFSET)];
          impeghe_mae_read_csv_float(&pstr_asi_config->grp_def_min_el_offset[0], ptr_val_string, pstr_asi_config->num_groups, quant, offset);
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_EL_OFFSET,
                    strlen(STRING_GROUP_DFN_MAX_EL_OFFSET)) == 0)
        {
          FLOAT32 quant = 3.0f;
          WORD32 offset = 0;
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MIN_EL_OFFSET)];
          impeghe_mae_read_csv_float(&pstr_asi_config->grp_def_max_el_offset[0], ptr_val_string, pstr_asi_config->num_groups, quant, offset);
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_DIST_FACTOR,
                    strlen(STRING_GROUP_DFN_MIN_DIST_FACTOR)) == 0)
        {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MIN_DIST_FACTOR)];
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_min_dist_factor[0], ptr_val_string, pstr_asi_config->num_groups);
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_DIST_FACTOR,
                    strlen(STRING_GROUP_DFN_MAX_DIST_FACTOR)) == 0)
        {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MAX_DIST_FACTOR)];
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_max_dist_factor[0], ptr_val_string, pstr_asi_config->num_groups);
          READ_NEXT_LINE();
          }
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_ALLOW_GAIN_FACTOR,
                    strlen(STRING_GROUP_DFN_ALLOW_GAIN_FACTOR)) == 0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_ALLOW_GAIN_FACTOR)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_allow_gain_interact[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();

        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MIN_GAIN, strlen(STRING_GROUP_DFN_MIN_GAIN)) ==
            0)
        {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MIN_GAIN)];
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_min_gain[0], ptr_val_string, pstr_asi_config->num_groups);
          for (WORD32 idx = 0; idx < pstr_asi_config->num_groups; idx++)
          {
            pstr_asi_config->grp_def_min_gain[idx] += 63;
          }
          READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_MAX_GAIN, strlen(STRING_GROUP_DFN_MAX_GAIN)) ==
            0)
        {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_MAX_GAIN)];
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_max_gain[0], ptr_val_string, pstr_asi_config->num_groups);
          READ_NEXT_LINE();
        }
      }

      if (strncmp((pCHAR8)line, STRING_GROUP_DFN_GRP_NUM, strlen(STRING_GROUP_DFN_GRP_NUM)) ==
          0)
          {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_GRP_NUM)];
        impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_group_num[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();
        }
        if (strncmp((pCHAR8)line, STRING_GROUP_DFN_HAS_CONJUNCT_MEM,
                    strlen(STRING_GROUP_DFN_HAS_CONJUNCT_MEM)) == 0)
        {
        pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_HAS_CONJUNCT_MEM)];
        impeghe_mae_read_csv_char(&pstr_asi_config->has_conjunct_members[0], ptr_val_string, pstr_asi_config->num_groups);
        READ_NEXT_LINE();

          if (strncmp((pCHAR8)line, STRING_GROUP_DFN_START_ID,
                      strlen(STRING_GROUP_DFN_START_ID)) == 0)
          {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_START_ID)];
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_start_id[0], ptr_val_string, pstr_asi_config->num_groups);
          READ_NEXT_LINE();
          }
            if (strncmp((pCHAR8)line, STRING_GROUP_DFN_META_ELE_ID,
                        strlen(STRING_GROUP_DFN_META_ELE_ID)) == 0)
            {
          pCHAR8 ptr_val_string = &line[strlen(STRING_GROUP_DFN_GRP_NUM)];
          for (i = 0; i < pstr_asi_config->num_groups; i++)
              {
            impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_metadata_ele_id[i][0], ptr_val_string, pstr_asi_config->grp_def_group_num[i]);
          }
          READ_NEXT_LINE()
        }
      }
    }
    if (strncmp((pCHAR8)line, STRING_NUM_SWITCH_GRP, strlen(STRING_NUM_SWITCH_GRP)) == 0)
    {
      pstr_asi_config->num_switch_groups =
          (atoi((const pCHAR8)&line[strlen(STRING_NUM_SWITCH_GRP)]));
      READ_NEXT_LINE()
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_GRP_ID,
                    strlen(STRING_SWITCH_GRP_DEFN_GRP_ID)) == 0)
        {
        impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_grp_id[0],
            &line[strlen(STRING_SWITCH_GRP_DEFN_GRP_ID)],
            pstr_asi_config->num_switch_groups);
        READ_NEXT_LINE()
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF,
                    strlen(STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF)) == 0)
        {
        impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_allow_on_off[0],
            &line[strlen(STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF)],
            pstr_asi_config->num_switch_groups);
        READ_NEXT_LINE()
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF,
                    strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF)) == 0)
        {
        impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_default_on_off[0],
            &line[strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF)],
            pstr_asi_config->num_switch_groups);
        READ_NEXT_LINE()
        }
        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM,
                    strlen(STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM)) == 0)
        {
        impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_grp_num_member[0],
            &line[strlen(STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM)],
            pstr_asi_config->num_switch_groups);
        READ_NEXT_LINE()
        for (j = 0; j < pstr_asi_config->num_switch_groups; j++)
          {
            if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_MEM_ID,
                        strlen(STRING_SWITCH_GRP_DEFN_MEM_ID)) == 0)
            {
            impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_grp_member_id[j][0],
                &line[strlen(STRING_SWITCH_GRP_DEFN_MEM_ID)],
                pstr_asi_config->switch_grp_def_grp_num_member[j]);
            READ_NEXT_LINE()
            }
          }
        }

        if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID,
                    strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID)) == 0)
        {
        impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_def_default_group_id[0],
            &line[strlen(STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID)],
            pstr_asi_config->num_switch_groups);
        READ_NEXT_LINE()
      }
    }
    if (strncmp((pCHAR8)line, STRING_NUM_GRP_PRESETS, strlen(STRING_NUM_GRP_PRESETS)) == 0)
    {
      pstr_asi_config->num_group_presets =
          (atoi((const pCHAR8)&line[strlen(STRING_NUM_GRP_PRESETS)]));
      READ_NEXT_LINE()
      //for (i = 0; i < pstr_asi_config->num_group_presets; i++)
      {
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GRP_ID,
                    strlen(STRING_GRP_PRESET_DEFN_GRP_ID)) == 0)
        {
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_grp_id[0],
              &line[strlen(STRING_GRP_PRESET_DEFN_GRP_ID)],
              pstr_asi_config->num_group_presets);
          READ_NEXT_LINE()
        }
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_PRESET_KIND,
                    strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)) == 0)
        {
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_preset_kind[0],
              &line[strlen(STRING_GRP_PRESET_DEFN_PRESET_KIND)],
              pstr_asi_config->num_group_presets);
          READ_NEXT_LINE()
        }
        if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_NUM_CONDITION,
                    strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)) == 0)
        {
          impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_num_conditions[0],
              &line[strlen(STRING_GRP_PRESET_DEFN_NUM_CONDITION)],
              pstr_asi_config->num_group_presets);
          READ_NEXT_LINE()
          for (j = 0; j < pstr_asi_config->num_group_presets; j++)
          {
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_REF_ID,
                        strlen(STRING_GRP_PRESET_DEFN_REF_ID)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_reference_id[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_REF_ID)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_CON_ON_OFF,
                        strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_cond_on_off[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_CON_ON_OFF)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN_FLAG,
                        strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_gain_flag[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_GAIN_FLAG)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_GAIN,
                        strlen(STRING_GRP_PRESET_DEFN_GAIN)) == 0)
            {
              FLOAT32 quant_fac = 0.5f;
              WORD32 offset = 191;
              impeghe_mae_read_csv_float(&pstr_asi_config->grp_preset_def_gain[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_GAIN)],
                  pstr_asi_config->grp_preset_def_num_conditions[j],
                  quant_fac, offset);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_disable_gain_interact[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_disable_position_interact[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT,
                        strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_position_interact[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_AZ_OFFSET,
                        strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)) == 0)
            {
              FLOAT32 quant_fac = 1.5f;
              WORD32 offset = 127;
              impeghe_mae_read_csv_float(&pstr_asi_config->grp_preset_def_azimuth_offset[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_AZ_OFFSET)],
                  pstr_asi_config->grp_preset_def_num_conditions[j],
                  quant_fac,
                  offset);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_EL_OFFSET,
                        strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)) == 0)
            {
              FLOAT32 quant_fac = 3.0f;
              WORD32 offset = 31;
              impeghe_mae_read_csv_float(&pstr_asi_config->grp_preset_def_elevation_offset[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_EL_OFFSET)],
                  pstr_asi_config->grp_preset_def_num_conditions[j],
                  quant_fac,
                  offset);
              READ_NEXT_LINE()
            }
            if (strncmp((pCHAR8)line, STRING_GRP_PRESET_DEFN_DIST_FACTOR,
                        strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)) == 0)
            {
              impeghe_mae_read_csv_char(&pstr_asi_config->grp_preset_def_dist_factor[j][0],
                  &line[strlen(STRING_GRP_PRESET_DEFN_DIST_FACTOR)],
                  pstr_asi_config->grp_preset_def_num_conditions[j]);
              READ_NEXT_LINE()
            }
          }
        }
      }
    }

    if (strncmp((pCHAR8)line, STRING_NUM_DATA_SETS, strlen(STRING_NUM_DATA_SETS)) == 0)
    {
      pstr_asi_config->num_data_sets = (atoi((const pCHAR8)&line[strlen(STRING_NUM_DATA_SETS)]));
      READ_NEXT_LINE()
      if (strncmp((pCHAR8)line, STRING_DATA_TYPE, strlen(STRING_DATA_TYPE)) == 0)
      {
        impeghe_mae_read_csv_char(&pstr_asi_config->data_type[0],
            &line[strlen(STRING_DATA_TYPE)],
            pstr_asi_config->num_data_sets);
        READ_NEXT_LINE()
      }
      for (i = 0; i < pstr_asi_config->num_data_sets; i++)
      {
        if (pstr_asi_config->data_type[i] == 0)
        {
          if (strncmp((pCHAR8)line, STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS,
                      strlen(STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS)) == 0)
          {
            pstr_asi_config->num_grp_def_decription_blocks =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS)]));
            READ_NEXT_LINE()
            for (int block_idx = 0; block_idx < pstr_asi_config->num_grp_def_decription_blocks; block_idx++)
            {
              if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_GRP_ID,
                          strlen(STRING_GROUP_DEF_DSCRPTN_GRP_ID)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_decription_grp_id[block_idx],
                  &line[strlen(STRING_GROUP_DEF_DSCRPTN_GRP_ID)],
                  1);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->num_grp_def_decription_languages[block_idx],
                  &line[strlen(STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES)],
                  1);
                READ_NEXT_LINE()

                  if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_LANGUAGES,
                              strlen(STRING_GROUP_DEF_DSCRPTN_LANGUAGES)) == 0)
                  {
                    impeghe_mae_read_csv_lan(&pstr_asi_config->grp_def_decription_languages[block_idx][0][0],
                      &line[strlen(STRING_GROUP_DEF_DSCRPTN_LANGUAGES)],
                      pstr_asi_config->num_grp_def_decription_languages[block_idx]);
                    READ_NEXT_LINE()
                  }
                  if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                  impeghe_mae_read_csv_char(&pstr_asi_config->grp_def_decription_data_length[block_idx][0],
                    &line[strlen(STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH)],
                    pstr_asi_config->num_grp_def_decription_languages[block_idx]);
                  READ_NEXT_LINE()
                    }

                    if (strncmp((pCHAR8)line, STRING_GROUP_DEF_DSCRPTN_DATA,
                                strlen(STRING_GROUP_DEF_DSCRPTN_DATA)) == 0)
                    {
                  impeghe_mae_read_csv_descr_data(&pstr_asi_config->grp_def_decription_data[block_idx][0][0],
                    &line[strlen(STRING_GROUP_DEF_DSCRPTN_DATA)],
                    MAX_DESCRIPTON_DATA_LEN,
                    &pstr_asi_config->grp_def_decription_data_length[block_idx][0],
                    pstr_asi_config->num_grp_def_decription_languages[block_idx]);
                  READ_NEXT_LINE()
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
            READ_NEXT_LINE()
            for (int block_idx = 0; block_idx < pstr_asi_config->num_switch_grp_decription_blocks; block_idx++)
            {
              if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_GRP_ID,
                          strlen(STRING_SWITCH_GRP_DSCRPTN_GRP_ID)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_decription_grp_id[block_idx],
                  &line[strlen(STRING_SWITCH_GRP_DSCRPTN_GRP_ID)],
                  pstr_asi_config->num_switch_grp_decription_blocks);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                pstr_asi_config->switch_grp_num_decription_languages[block_idx] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES)]));
                READ_NEXT_LINE()
                  if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_LANGUAGES,
                              strlen(STRING_SWITCH_GRP_DSCRPTN_LANGUAGES)) == 0)
                  {
                    impeghe_mae_read_csv_lan(&pstr_asi_config->switch_grp_decription_languages[block_idx][0][0],
                      &line[strlen(STRING_SWITCH_GRP_DSCRPTN_LANGUAGES)],
                      pstr_asi_config->switch_grp_num_decription_languages[block_idx]);
                    READ_NEXT_LINE()
                  }
                  if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                  impeghe_mae_read_csv_char(&pstr_asi_config->switch_grp_decription_data_length[block_idx][0],
                    &line[strlen(STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH)],
                    pstr_asi_config->switch_grp_num_decription_languages[block_idx]);
                  READ_NEXT_LINE()
                    }
                    if (strncmp((pCHAR8)line, STRING_SWITCH_GRP_DSCRPTN_DATA,
                                strlen(STRING_SWITCH_GRP_DSCRPTN_DATA)) == 0)
                    {
                  impeghe_mae_read_csv_descr_data(&pstr_asi_config->switch_grp_decription_data[block_idx][0][0],
                    &line[strlen(STRING_SWITCH_GRP_DSCRPTN_DATA)],
                    MAX_DESCRIPTON_DATA_LEN,
                    &pstr_asi_config->switch_grp_decription_data_length[block_idx][0],
                    pstr_asi_config->switch_grp_num_decription_languages[block_idx]);
                  READ_NEXT_LINE()
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
            READ_NEXT_LINE()
            for (int block_idx = 0; block_idx < pstr_asi_config->num_preset_decription_blocks; block_idx++)
            {
              if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_GRP_ID,
                          strlen(STRING_PRESET_DSCRPTN_GRP_ID)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->preset_decription_grp_id[block_idx],
                  &line[strlen(STRING_PRESET_DSCRPTN_GRP_ID)],
                  1);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_PRESET_NUM_DSCRPTN_LANGUAGES,
                          strlen(STRING_PRESET_NUM_DSCRPTN_LANGUAGES)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->preset_num_decription_languages[block_idx],
                    &line[strlen(STRING_PRESET_NUM_DSCRPTN_LANGUAGES)],
                    1);
                READ_NEXT_LINE()
                  if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_LANGUAGES,
                              strlen(STRING_PRESET_DSCRPTN_LANGUAGES)) == 0)
                  {
                    impeghe_mae_read_csv_lan(&pstr_asi_config->preset_decription_languages[block_idx][0][0],
                      &line[strlen(STRING_PRESET_DSCRPTN_LANGUAGES)],
                      pstr_asi_config->preset_num_decription_languages[block_idx]);
                    READ_NEXT_LINE()
                  }
                  if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_DATA_LENGTH,
                              strlen(STRING_PRESET_DSCRPTN_DATA_LENGTH)) == 0)
                  {
                  impeghe_mae_read_csv_char(&pstr_asi_config->preset_decription_data_length[block_idx][0],
                    &line[strlen(STRING_PRESET_DSCRPTN_DATA_LENGTH)],
                    pstr_asi_config->preset_num_decription_languages[block_idx]);
                  READ_NEXT_LINE()
                  }
                    if (strncmp((pCHAR8)line, STRING_PRESET_DSCRPTN_DATA,
                                strlen(STRING_PRESET_DSCRPTN_DATA)) == 0)
                    {
                  impeghe_mae_read_csv_descr_data(&pstr_asi_config->preset_decription_data[block_idx][0][0],
                    &line[strlen(STRING_PRESET_DSCRPTN_DATA)],
                    MAX_DESCRIPTON_DATA_LEN,
                    &pstr_asi_config->preset_decription_data_length[block_idx][0],
                    pstr_asi_config->preset_num_decription_languages[0]);
                  READ_NEXT_LINE()
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
            READ_NEXT_LINE()
            //for (j = 0; j < pstr_asi_config->num_content_data_blocks; j++)
            {
              if (strncmp((pCHAR8)line, STRING_CONTENT_GRP_ID, strlen(STRING_CONTENT_GRP_ID)) ==
                  0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->content_group_id[0],
                    &line[strlen(STRING_CONTENT_GRP_ID)],
                    pstr_asi_config->num_content_data_blocks);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_CONTENT_KIND, strlen(STRING_CONTENT_KIND)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->content_kind[0],
                    &line[strlen(STRING_CONTENT_KIND)],
                    pstr_asi_config->num_content_data_blocks);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_HAS_CONTENT_LANGUAGE,
                          strlen(STRING_HAS_CONTENT_LANGUAGE)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->has_content_language[0],
                    &line[strlen(STRING_HAS_CONTENT_LANGUAGE)],
                    pstr_asi_config->num_content_data_blocks);
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_CONTENT_LANGUAGE,
                          strlen(STRING_CONTENT_LANGUAGE)) == 0)
              {
                impeghe_mae_read_csv_lan(&pstr_asi_config->content_language[0][0],
                    &line[strlen(STRING_CONTENT_LANGUAGE)],
                    pstr_asi_config->num_content_data_blocks);
                READ_NEXT_LINE()
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
            READ_NEXT_LINE()
            for (j = 0; j < 2 * pstr_asi_config->num_comp_pairs; j++)
            {
              if (strncmp((pCHAR8)line, STRING_ELEMENT_ID, strlen(STRING_ELEMENT_ID)) == 0)
              {
                pstr_asi_config->element_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_ELEMENT_ID)]));
                READ_NEXT_LINE()
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
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_AZ, strlen(STRING_SCRN_SIZE_AZ)) == 0)
          {
            pstr_asi_config->screen_size_az =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_AZ)]));
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_EL, strlen(STRING_SCRN_SIZE_EL)) == 0)
          {
            pstr_asi_config->screen_size_el =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_EL)]));
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_SCRN_SIZE_BOT_EL, strlen(STRING_SCRN_SIZE_BOT_EL)) ==
              0)
          {
            pstr_asi_config->screen_size_bot_el[i] =
                (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SIZE_BOT_EL)]));
            READ_NEXT_LINE()
          }
        }
        if (pstr_asi_config->data_type[i] == 7)
        {

          if (strncmp((pCHAR8)line, STRING_OVERWRITE_PRO_SCRN_SIZE_DATA,
                      strlen(STRING_OVERWRITE_PRO_SCRN_SIZE_DATA)) == 0)
          {
            pstr_asi_config->overwrite_prod_screen_size_data =
                ((int)line[strlen(STRING_OVERWRITE_PRO_SCRN_SIZE_DATA)] - '0');
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_SCRN_SZ_LEFT_AZ,
                      strlen(STRING_DFLT_SCRN_SZ_LEFT_AZ)) == 0)
          {
            pstr_asi_config->default_screen_sz_left_az =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_SCRN_SZ_LEFT_AZ)]));
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_SCRN_SZ_RIGHT_AZ,
                      strlen(STRING_DFLT_SCRN_SZ_RIGHT_AZ)) == 0)
          {
            pstr_asi_config->default_screen_sz_right_az =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_SCRN_SZ_RIGHT_AZ)]));
            READ_NEXT_LINE()
          }
          if (strncmp((pCHAR8)line, STRING_NUM_PRESET_PROD_SCRNS,
                      strlen(STRING_NUM_PRESET_PROD_SCRNS)) == 0)
          {
            pstr_asi_config->num_preset_prod_screens =
                (atoi((const pCHAR8)&line[strlen(STRING_NUM_PRESET_PROD_SCRNS)]));
            READ_NEXT_LINE()
            for (j = 0; j < pstr_asi_config->num_preset_prod_screens; j++)
            {
              if (strncmp((pCHAR8)line, STRING_SCRN_GRP_PRESET_ID,
                          strlen(STRING_SCRN_GRP_PRESET_ID)) == 0)
              {
                pstr_asi_config->screen_grp_preset_id[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_GRP_PRESET_ID)]));
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_CENTERED_IN_AZ, strlen(STRING_CENTERED_IN_AZ)) ==
                  0)
              {
                pstr_asi_config->centered_in_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_CENTERED_IN_AZ)]));
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_LEFT_AZ, strlen(STRING_SCRN_SZ_LEFT_AZ)) ==
                  0)
              {
                pstr_asi_config->screen_sz_left_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_LEFT_AZ)]));
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_RIGHT_AZ,
                          strlen(STRING_SCRN_SZ_RIGHT_AZ)) == 0)
              {
                pstr_asi_config->screen_sz_right_az[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_RIGHT_AZ)]));
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_SCRN_SZ_TOP_EL, strlen(STRING_SCRN_SZ_TOP_EL)) ==
                  0)
              {
                pstr_asi_config->screen_sz_top_el[j] =
                    (atoi((const pCHAR8)&line[strlen(STRING_SCRN_SZ_TOP_EL)]));
                READ_NEXT_LINE()
              }
            }
          }
        }
        if (pstr_asi_config->data_type[i] == 8)
        {
          WORD32 pr;
          if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_HAS_SWITCH_COND, strlen(STRING_GRP_PR_EXT_HAS_SWITCH_COND)) == 0)
            {
            impeghe_mae_read_csv_char(&pstr_asi_config->has_switch_group_conditions[0],
                    &line[strlen(STRING_GRP_PR_EXT_HAS_SWITCH_COND)],
                    pstr_asi_config->num_group_presets);
            READ_NEXT_LINE();
            }
          for (pr = 0; pr < pstr_asi_config->num_group_presets; pr++)
            {
            if (pstr_asi_config->has_switch_group_conditions[pr])
              {
              if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_IS_SWITCH_COND_PRESET_DEF, strlen(STRING_GRP_PR_EXT_IS_SWITCH_COND_PRESET_DEF)) == 0)
                {
                impeghe_mae_read_csv_char(&pstr_asi_config->is_switch_group_condition_preset_definition[pr][0],
                        &line[strlen(STRING_GRP_PR_EXT_IS_SWITCH_COND_PRESET_DEF)],
                        pstr_asi_config->grp_preset_def_num_conditions[pr]);
                READ_NEXT_LINE();
              }
                }
              }
          if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_HAS_DOWNMIX_ID, strlen(STRING_GRP_PR_EXT_HAS_DOWNMIX_ID)) == 0)
                {
            impeghe_mae_read_csv_char(&pstr_asi_config->has_downmix_id_group_preset_extensions[0],
                    &line[strlen(STRING_GRP_PR_EXT_HAS_DOWNMIX_ID)],
                    pstr_asi_config->num_group_presets);
            READ_NEXT_LINE();
                }
          for (pr = 0; pr < pstr_asi_config->num_group_presets; pr++)
                {
            if (pstr_asi_config->has_downmix_id_group_preset_extensions[pr])
                  {
              if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_NUM_DOWNMIX_ID, strlen(STRING_GRP_PR_EXT_NUM_DOWNMIX_ID)) == 0)
                    {
                impeghe_mae_read_csv_char(&pstr_asi_config->num_dmx_id_group_preset_ext[pr],
                        &line[strlen(STRING_GRP_PR_EXT_NUM_DOWNMIX_ID)],
                        1);
                READ_NEXT_LINE();
                    }
              if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_DOWNMIX_ID, strlen(STRING_GRP_PR_EXT_DOWNMIX_ID)) == 0)
              {
                impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_downmix_id[pr][0],
                        &line[strlen(STRING_GRP_PR_EXT_DOWNMIX_ID)],
                        pstr_asi_config->num_dmx_id_group_preset_ext[pr]);
                READ_NEXT_LINE();
                  }
              if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_NUM_COND, strlen(STRING_GRP_PR_EXT_NUM_COND)) == 0)
                  {
                impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_num_conditions[pr][1],
                        &line[strlen(STRING_GRP_PR_EXT_NUM_COND)],
                        pstr_asi_config->num_dmx_id_group_preset_ext[pr]);
                READ_NEXT_LINE();
                  }
              for (j = 0; j < pstr_asi_config->num_dmx_id_group_preset_ext[pr]; j++)
                  {
                if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_IS_SWITCH_COND, strlen(STRING_GRP_PR_EXT_IS_SWITCH_COND)) == 0)
                    {
                  impeghe_mae_read_csv_char(&pstr_asi_config->is_switch_group_condition[pr][j + 1][0],
                          &line[strlen(STRING_GRP_PR_EXT_IS_SWITCH_COND)],
                          pstr_asi_config->group_preset_num_conditions[pr][j + 1]);
                  READ_NEXT_LINE();
                    }
                if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_SW_GRP_GRP_ID, strlen(STRING_GRP_PR_EXT_SW_GRP_GRP_ID)) == 0)
                {
                  impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_group_id[pr][j + 1][0],
                          &line[strlen(STRING_GRP_PR_EXT_SW_GRP_GRP_ID)],
                          pstr_asi_config->group_preset_num_conditions[pr][j + 1]);
                  READ_NEXT_LINE();
                  }
                if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_COND_ON_OFF, strlen(STRING_GRP_PR_EXT_COND_ON_OFF)) == 0)
                  {
                  impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_condition_on_off[pr][j + 1][0],
                          &line[strlen(STRING_GRP_PR_EXT_COND_ON_OFF)],
                          pstr_asi_config->group_preset_num_conditions[pr][j + 1]);
                  READ_NEXT_LINE();
                }
                for (k = 0; k < pstr_asi_config->group_preset_num_conditions[pr][j + 1]; k++)
                    {
                  if (pstr_asi_config->group_preset_condition_on_off[pr][j + 1][k])
                  {
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_DISABLE_GAIN_INTERACT, strlen(STRING_GRP_PR_EXT_DISABLE_GAIN_INTERACT)) == 0)
                    {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_disable_gain_interactivity[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_DISABLE_GAIN_INTERACT)],
                              1);
                      READ_NEXT_LINE();
                    }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_GAIN_FLAG, strlen(STRING_GRP_PR_EXT_GAIN_FLAG)) == 0)
                    {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_gain_flag[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_GAIN_FLAG)],
                              1);
                      READ_NEXT_LINE();
                  }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_GAIN, strlen(STRING_GRP_PR_EXT_GAIN)) == 0)
                  {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_gain[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_GAIN)],
                              1);
                      READ_NEXT_LINE();
                    }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_DISABLE_POS_INTERACT, strlen(STRING_GRP_PR_EXT_DISABLE_POS_INTERACT)) == 0)
                    {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_disable_position_interactivity[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_DISABLE_POS_INTERACT)],
                              1);
                      READ_NEXT_LINE();
                  }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_POS_FLAG, strlen(STRING_GRP_PR_EXT_POS_FLAG)) == 0)
                  {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_position_flag[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_POS_FLAG)],
                              1);
                      READ_NEXT_LINE();
                    }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_AZ_OFFSET, strlen(STRING_GRP_PR_EXT_AZ_OFFSET)) == 0)
                  {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_az_offset[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_AZ_OFFSET)],
                              1);
                      READ_NEXT_LINE();
                    }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_EL_OFFSET, strlen(STRING_GRP_PR_EXT_EL_OFFSET)) == 0)
                    {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_el_offset[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_EL_OFFSET)],
                              1);
                      READ_NEXT_LINE();
                  }
                    if (strncmp((pCHAR8)line, STRING_GRP_PR_EXT_DIST_FACT, strlen(STRING_GRP_PR_EXT_DIST_FACT)) == 0)
                  {
                      impeghe_mae_read_csv_char(&pstr_asi_config->group_preset_dist_factor[pr][j + 1][k],
                              &line[strlen(STRING_GRP_PR_EXT_DIST_FACT)],
                              1);
                      READ_NEXT_LINE();
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
                READ_NEXT_LINE()
              }
            }
          }
          if (strncmp((pCHAR8)line, STRING_DFLT_PARAM_PRESENT,
                      strlen(STRING_DFLT_PARAM_PRESENT)) == 0)
          {
            pstr_asi_config->default_params_present =
                (atoi((const pCHAR8)&line[strlen(STRING_DFLT_PARAM_PRESENT)]));
            READ_NEXT_LINE()
          }
          if (pstr_asi_config->default_params_present)
          {
            for (k = 0; k < pstr_asi_config->num_groups; k++)
            {

              if (strncmp((pCHAR8)line, STRING_DFLT_INCL_GRP, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_include_group[k] =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_INCL_GRP)]));
                READ_NEXT_LINE()
              }
            }
            if (strncmp((pCHAR8)line, STRING_DFLT_MIN_MAX_GAIN_PRESENT,
                        strlen(STRING_DFLT_INCL_GRP)) == 0)
            {
              pstr_asi_config->default_min_max_gain_present =
                  (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MIN_MAX_GAIN_PRESENT)]));
              READ_NEXT_LINE()
            }

            if (pstr_asi_config->default_min_max_gain_present)
            {
              if (strncmp((pCHAR8)line, STRING_DFLT_MIN_GAIN, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_min_gain =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MIN_GAIN)]));
                READ_NEXT_LINE()
              }
              if (strncmp((pCHAR8)line, STRING_DFLT_MAX_GAIN, strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->default_max_gain =
                    (atoi((const pCHAR8)&line[strlen(STRING_DFLT_MAX_GAIN)]));
                READ_NEXT_LINE()
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
              READ_NEXT_LINE()
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
                  READ_NEXT_LINE()
                }
              }
              if (strncmp((pCHAR8)line, STRING_PRESET_MIN_MAX_PRESENT,
                          strlen(STRING_DFLT_INCL_GRP)) == 0)
              {
                pstr_asi_config->preset_min_max_gain_present[k] =
                    (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MIN_MAX_PRESENT)]));
                READ_NEXT_LINE()
              }
              if (pstr_asi_config->preset_min_max_gain_present[k])
              {
                if (strncmp((pCHAR8)line, STRING_PRESET_MIN_GAIN, strlen(STRING_DFLT_INCL_GRP)) ==
                    0)
                {
                  pstr_asi_config->preset_min_gain[k] =
                      (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MIN_GAIN)]));
                  READ_NEXT_LINE()
                }
                if (strncmp((pCHAR8)line, STRING_PRESET_MAX_GAIN, strlen(STRING_DFLT_INCL_GRP)) ==
                    0)
                {
                  pstr_asi_config->preset_max_gain[k] =
                      (atoi((const pCHAR8)&line[strlen(STRING_PRESET_MAX_GAIN)]));
                  READ_NEXT_LINE()
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

