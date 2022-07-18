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
#include "impeghe_type_def.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_uni_drc_filter_bank.h"

/**
 *  impeghe_drc_filter_bank_complexity
 *
 *  \brief Gets DRC filter bank complexity value
 *
 *  \param [in] num_bands              Number of bands
 *  \param [out] pstr_drc_filter_bank  Pointer to DRC filter bank structure
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE
impeghe_drc_filter_bank_complexity(const WORD32 num_bands,
                                   ia_drc_filter_bank_struct *pstr_drc_filter_bank)
{
  pstr_drc_filter_bank->complexity = 0;
  pstr_drc_filter_bank->num_bands = num_bands;
  switch (num_bands)
  {
  case 1:
    break;
  case 2:
    pstr_drc_filter_bank->complexity = 8;
    break;
  case 3:
    pstr_drc_filter_bank->complexity = 18;
    break;
  case 4:
    pstr_drc_filter_bank->complexity = 28;
    break;
  default:
    return IMPEGHE_CONFIG_FATAL_DRC_PARAM_OUT_OF_RANGE;
    break;
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_drc_init_all_filter_banks
 *
 *  \brief Initialize all filter banks
 *
 *  \param [in] pstr_drc_coefficients_uni_drc Pointer to DRC coefficients structure
 *  \param [in] pstr_drc_instructions_uni_drc Pointer to DRC instructions structure
 *  \param [out] pstr_filter_banks            Pointer to DRC filter banks structure
 *  \param [in] ptr_scratch                   Pointer to scratch memory
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_drc_init_all_filter_banks(
    const ia_drc_coefficients_uni_drc_struct *pstr_drc_coefficients_uni_drc,
    const ia_drc_instructions_uni_drc *pstr_drc_instructions_uni_drc,
    ia_drc_filter_banks_struct *pstr_filter_banks, VOID *ptr_scratch)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  LOOPIDX band_idx, group_idx, i, j, k;
  WORD32 crossover_freq_index, num_ch_in_groups, num_phase_alignment_ch_groups;
  WORD32 index_found = FALSE;
  WORD32 group_count[MAX_CHANNEL_GROUP_COUNT + 1] = {0};
  WORD32 *ptr_cascade_crossover_indices[MAX_CHANNEL_GROUP_COUNT + 1];

  for (i = 0; i < MAX_CHANNEL_GROUP_COUNT + 1; i++)
  {
    ptr_cascade_crossover_indices[i] = (WORD32 *)(ptr_scratch);
    ptr_scratch = (UWORD8 *)ptr_scratch + (MAX_CHANNEL_GROUP_COUNT * 3) * sizeof(WORD32);
  }

  num_ch_in_groups = 0;
  for (group_idx = 0; group_idx < pstr_drc_instructions_uni_drc->num_drc_channel_groups;
       group_idx++)
  {
    num_ch_in_groups += pstr_drc_instructions_uni_drc->num_channels_per_channel_group[group_idx];
  }
  num_phase_alignment_ch_groups = pstr_drc_instructions_uni_drc->num_drc_channel_groups;
  if (num_ch_in_groups < pstr_drc_instructions_uni_drc->drc_channel_count)
  {
    num_phase_alignment_ch_groups++;
  }
  if (num_phase_alignment_ch_groups > IMPEGHE_DRC_MAX_PHASE_ALIGN_CH_GROUP)
  {
    return IMPEGHE_CONFIG_FATAL_DRC_PARAM_OUT_OF_RANGE;
  }

  memset(pstr_filter_banks->str_drc_filter_bank, 0,
         sizeof(pstr_filter_banks->str_drc_filter_bank));
  pstr_filter_banks->num_phase_alignment_ch_groups = num_phase_alignment_ch_groups;
  pstr_filter_banks->num_filter_banks = pstr_drc_instructions_uni_drc->num_drc_channel_groups;
  if (pstr_drc_coefficients_uni_drc != NULL)
  {
    for (group_idx = 0; group_idx < pstr_drc_instructions_uni_drc->num_drc_channel_groups;
         group_idx++)
    {
      err_code = impeghe_drc_filter_bank_complexity(
          pstr_drc_coefficients_uni_drc
              ->str_gain_set_params[pstr_drc_instructions_uni_drc
                                        ->gain_set_index_for_channel_group[group_idx]]
              .band_count,
          &(pstr_filter_banks->str_drc_filter_bank[group_idx]));
      if (err_code)
      {
        return err_code;
      }
    }
  }
  else
  {
    pstr_filter_banks->str_drc_filter_bank->num_bands = 1;
  }

  if (pstr_drc_coefficients_uni_drc != NULL)
  {
    for (group_idx = 0; group_idx < pstr_drc_instructions_uni_drc->num_drc_channel_groups;
         group_idx++)
    {
      for (band_idx = 1;
           band_idx < pstr_drc_coefficients_uni_drc
                          ->str_gain_set_params[pstr_drc_instructions_uni_drc
                                                    ->gain_set_index_for_channel_group[group_idx]]
                          .band_count;
           band_idx++)
      {
        crossover_freq_index =
            pstr_drc_coefficients_uni_drc
                ->str_gain_set_params[pstr_drc_instructions_uni_drc
                                          ->gain_set_index_for_channel_group[group_idx]]
                .gain_params[band_idx]
                .crossover_freq_index;

        for (j = 0; j < num_phase_alignment_ch_groups; j++)
        {
          if (j != group_idx)
          {
            ptr_cascade_crossover_indices[j][group_count[j]] = crossover_freq_index;
            group_count[j]++;
            if (group_count[j] > MAX_CHANNEL_GROUP_COUNT * 3)
            {
              return IMPEGHE_CONFIG_FATAL_DRC_PARAM_OUT_OF_RANGE;
            }
          }
        }
      }
    }
  }

  i = 0;
  while (i < group_count[0])
  {
    crossover_freq_index = ptr_cascade_crossover_indices[0][i];
    index_found = FALSE;
    for (group_idx = 1; group_idx < num_phase_alignment_ch_groups; group_idx++)
    {
      index_found = FALSE;
      for (j = 0; j < group_count[group_idx]; j++)
      {
        if (ptr_cascade_crossover_indices[group_idx][j] == crossover_freq_index)
        {
          index_found = TRUE;
          break;
        }
      }
      if (index_found == FALSE)
      {
        break;
      }
    }
    if (index_found == FALSE)
    {
      i++;
    }
    else
    {
      for (group_idx = 0; group_idx < num_phase_alignment_ch_groups; group_idx++)
      {
        for (j = 0; j < group_count[group_idx]; j++)
        {
          if (ptr_cascade_crossover_indices[group_idx][j] == crossover_freq_index)
          {
            for (k = j + 1; k < group_count[group_idx]; k++)
            {
              ptr_cascade_crossover_indices[group_idx][k - 1] =
                  ptr_cascade_crossover_indices[group_idx][k];
            }
            group_count[group_idx]--;
            break;
          }
        }
      }
      i = 0;
    }
  }
  for (group_idx = 0; group_idx < num_phase_alignment_ch_groups; group_idx++)
  {
    if (group_count[group_idx] > 0)
    {
      pstr_filter_banks->str_drc_filter_bank[group_idx].complexity +=
          (group_count[group_idx] << 1);
    }
  }
  pstr_filter_banks->complexity = 0;
  for (group_idx = 0; group_idx < pstr_drc_instructions_uni_drc->num_drc_channel_groups;
       group_idx++)
  {
    pstr_filter_banks->complexity +=
        pstr_drc_instructions_uni_drc->num_channels_per_channel_group[group_idx] *
        pstr_filter_banks->str_drc_filter_bank[group_idx].complexity;
  }

  return err_code;
}
