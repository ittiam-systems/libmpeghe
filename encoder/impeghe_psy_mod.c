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
#include "impeghe_cnst.h"
#include "impeghe_type_def.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_psy_mod.h"
#include "impeghe_psy_utils.h"

#include "impeghe_error_standards.h"
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"

#include "impeghe_ms.h"

#include "impeghe_psy_utils.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_oam_enc.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"

/**
 *  impeghe_psy_mod_init
 *
 *  \brief Initialization of psychoacoustic module
 *
 *  \param [in,out] pstr_psy_mod	Pointer to psy module structure
 *  \param [in]     sample_rate		Sample rate
 *  \param [in]     bit_rate		Bit rate
 *  \param [in]     band_width		Band width limit
 *  \param [in]     num_channels	Number of chnanels
 *  \param [in]     ch				Channel index
 *  \param [in]     ele_id			Element index
 *
 *  \return VOID
 */
VOID impeghe_psy_mod_init(ia_psy_mod_struct *pstr_psy_mod, WORD32 sample_rate, WORD32 bit_rate,
                          WORD32 band_width, WORD32 num_channels, WORD32 ch, WORD32 ele_id)
{
  WORD32 i;

  for (i = ch; i < ch + num_channels; i++)
  {
    impeghe_psy_long_config_init(bit_rate / num_channels, sample_rate, band_width,
                                 &(pstr_psy_mod->str_psy_long_config[ele_id]));

    impeghe_psy_short_config_init(bit_rate / num_channels, sample_rate, band_width,
                                  &(pstr_psy_mod->str_psy_short_config[ele_id]));

    pstr_psy_mod->str_psy_data[i].ptr_sfb_thr_long =
        (FLOAT32 *)pstr_psy_mod->str_psy_data[i].sfb_thr_short;
    pstr_psy_mod->str_psy_data[i].ptr_sfb_energy_long =
        (FLOAT32 *)pstr_psy_mod->str_psy_data[i].sfb_energy_short;
    pstr_psy_mod->str_psy_data[i].ptr_sfb_spreaded_energy_long =
        (FLOAT32 *)pstr_psy_mod->str_psy_data[i].sfb_spreaded_energy_short;

    memcpy(pstr_psy_mod->str_psy_data[i].sfb_thr_nm1,
           pstr_psy_mod->str_psy_long_config[ele_id].sfb_thr_quiet,
           pstr_psy_mod->str_psy_long_config[ele_id].sfb_count * sizeof(FLOAT32));
  }

  return;
}

/**
 *  impeghe_psy_mod_lb
 *
 *  \brief Compute psychoacoustic energies and thresholds for long block
 *
 *  \param [in,out] pstr_psy_mod    	Pointer to psychoacoustic model structure
 *  \param [in,out] pstr_igf_data   	Pointer to IGF data structure
 *  \param [in]     pstr_sfb_prms       Pointer to scalefactor band parameter structure
 *  \param [in]     ptr_spec_in         Pointer to input spectral coefficients
 *  \param [in,out] pstr_tns_info    	Pointer to TNS info structure
 *  \param [out]    tns_select         	Flag indicating if TNS is to be used
 *  \param [in]     i_ch                Channel offset
 *  \param [in]     chn                 Channel index
 *  \param [in]     igf_after_tns_synth IGF after TNS flag
 *  \param [in,out] pstr_igf_config 	Pointer to IGF configuration structure
 *  \param [in]     channel_type        Channel element type
 *  \param [in]     ptr_scratch_tns_filter  Pointer to scratch buffer for TNS filter
 *  \param [in]     elem_idx            Element index
 *  \param [in]     ptr_igf_scratch     Pointer to IGF scratch buffer
 *  \param [in]     ptr_tns_scratch     Pointer to TNS scratch buffer
 *
 *  \return VOID
 */
VOID impeghe_psy_mod_lb(ia_psy_mod_struct *pstr_psy_mod, ia_igf_data_struct *pstr_igf_data,
                        ia_sfb_params_struct *pstr_sfb_prms, FLOAT64 *ptr_spec_in,
                        ia_tns_info *pstr_tns_info[MAX_TIME_CHANNELS], WORD32 tns_select,
                        WORD32 i_ch, WORD32 chn, WORD32 igf_after_tns_synth,
                        ia_igf_config_struct *pstr_igf_config, WORD32 channel_type,
                        FLOAT64 *ptr_scratch_tns_filter, WORD32 elem_idx,
                        FLOAT64 *ptr_igf_scratch, FLOAT64 *ptr_tns_scratch)
{
  ia_psy_mod_data_struct *pstr_psy_data = &(pstr_psy_mod->str_psy_data[i_ch]);
  ia_psy_mod_long_config_struct *pstr_psy_config = &(pstr_psy_mod->str_psy_long_config[elem_idx]);
  WORD32 window_sequence = pstr_sfb_prms->window_sequence[i_ch];
  WORD32 num_sfb = pstr_sfb_prms->num_sfb[i_ch];
  ia_tns_info *ptr_tns_info = pstr_tns_info[i_ch];
  WORD32 sfb, line;
  WORD32 i;
  FLOAT32 energy_shift = 0.25f;
  FLOAT32 clip_energy = pstr_psy_config->clip_energy * energy_shift;

  pstr_psy_data->window_sequence = window_sequence;
  memset(&ptr_spec_in[pstr_psy_config->low_pass_line], 0,
         (FRAME_LEN_LONG - pstr_psy_config->low_pass_line) * sizeof(FLOAT64));

  impeghe_calc_band_energy(ptr_spec_in, pstr_psy_config->sfb_offset, pstr_psy_config->sfb_active,
                           pstr_psy_data->ptr_sfb_energy_long, pstr_psy_config->sfb_count);

  if (channel_type == ID_USAC_LFE)
  {
    tns_select = 0;
  }

  if (pstr_igf_config->igf_active == 1)
  {
    pstr_igf_config->is_short_block = 0;
    pstr_igf_config->m_igf_start_sfb = pstr_igf_config->igf_start_sfb_lb;
    pstr_igf_config->m_igf_stop_sfb = pstr_igf_config->igf_stop_sfb_lb;
  }

  if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active == 1 &&
      igf_after_tns_synth == 0)
  {
    impeghe_igf(ptr_spec_in, pstr_sfb_prms->sfb_offset[i_ch], pstr_igf_data, pstr_igf_config, 0,
                0, 1, 0, igf_after_tns_synth, 0, 0, ptr_igf_scratch);
  }

  if (tns_select != 0)
  {

    if (channel_type != ID_USAC_LFE)
    {
      ia_tns_info *ptr_tns_info_ch2 = pstr_tns_info[i_ch - chn];
      ptr_tns_info->number_of_bands = num_sfb;
      ptr_tns_info->block_type = window_sequence;
      ptr_tns_info->spec = ptr_spec_in;
      impeghe_tns_encode(ptr_tns_info_ch2, ptr_tns_info, pstr_psy_data->ptr_sfb_energy_long, 0,
                         chn, pstr_psy_config->low_pass_line, pstr_igf_config,
                         ptr_scratch_tns_filter, 0, ptr_tns_scratch);
    }
  }

  if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active == 1 &&
      igf_after_tns_synth == 1)
  {
    impeghe_igf(ptr_spec_in, pstr_sfb_prms->sfb_offset[i_ch], pstr_igf_data, pstr_igf_config, 0,
                ptr_tns_info->window_data[0].tns_pred_gain, 1, ptr_tns_info->tns_data_present,
                igf_after_tns_synth, 0, 0, ptr_igf_scratch);
  }

  for (i = 0; i < pstr_psy_config->sfb_count; i++)
  {
    pstr_psy_data->ptr_sfb_thr_long[i] =
        pstr_psy_data->ptr_sfb_energy_long[i] * pstr_psy_config->ratio;
    pstr_psy_data->ptr_sfb_thr_long[i] = min(pstr_psy_data->ptr_sfb_thr_long[i], clip_energy);
  }

  if (tns_select != 0)
  {
    if (ptr_tns_info->tns_data_present == 1)
    {
      impeghe_calc_band_energy(ptr_spec_in, pstr_psy_config->sfb_offset,
                               pstr_psy_config->sfb_active, pstr_psy_data->ptr_sfb_energy_long,
                               pstr_psy_config->sfb_count);
    }
  }

  if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active != 0)
  {
    impeghe_calc_band_energy(ptr_spec_in, pstr_psy_config->sfb_offset,
                             pstr_psy_config->sfb_active, pstr_psy_data->ptr_sfb_energy_long,
                             pstr_psy_config->sfb_count);
  }

  impeghe_find_max_spreading(pstr_psy_config->sfb_count, pstr_psy_config->sfb_mask_low_fac,
                             pstr_psy_config->sfb_mask_high_fac, pstr_psy_data->ptr_sfb_thr_long);

  for (i = 0; i < pstr_psy_config->sfb_count; i++)
  {
    pstr_psy_data->ptr_sfb_thr_long[i] = max(pstr_psy_data->ptr_sfb_thr_long[i],
                                             (pstr_psy_config->sfb_thr_quiet[i] * energy_shift));
  }

  if (pstr_psy_data->window_sequence == LONG_STOP_SEQUENCE)
  {
    for (i = 0; i < pstr_psy_config->sfb_count; i++)
    {
      pstr_psy_data->sfb_thr_nm1[i] = 1.0e20f;
    }
  }

  impeghe_pre_echo_control(pstr_psy_data->sfb_thr_nm1, pstr_psy_config->sfb_count,
                           pstr_psy_config->max_allowed_inc_fac,
                           pstr_psy_config->min_remaining_thr_fac,
                           pstr_psy_data->ptr_sfb_thr_long);

  if (pstr_psy_data->window_sequence == LONG_START_SEQUENCE)
  {
    for (i = 0; i < pstr_psy_config->sfb_count; i++)
    {
      pstr_psy_data->sfb_thr_nm1[i] = 1.0e20f;
    }
  }

  for (i = 0; i < pstr_psy_config->sfb_count; i++)
  {
    pstr_psy_data->ptr_sfb_spreaded_energy_long[i] = pstr_psy_data->ptr_sfb_energy_long[i];
  }
  impeghe_find_max_spreading(
      pstr_psy_config->sfb_count, pstr_psy_config->sfb_mask_low_fac_spr_ener,
      pstr_psy_config->sfb_mask_high_fac_spr_ener, pstr_psy_data->ptr_sfb_spreaded_energy_long);

  for (sfb = pstr_psy_config->sfb_count - 1; sfb >= 0; sfb--)
  {
    for (line = pstr_psy_config->sfb_offset[sfb + 1] - 1;
         line >= pstr_psy_config->sfb_offset[sfb]; line--)
    {
      if (ptr_spec_in[line] != 0)
        break;
    }
    if (line >= pstr_psy_config->sfb_offset[sfb])
      break;
  }

  pstr_psy_mod->str_psy_out_data[i_ch].max_sfb_per_grp = sfb + 1;

  return;
}

/**
 *  impeghe_psy_mod_sb
 *
 *  \brief Compute psychoacoustic energies and thresholds for short block
 *
 *  \param [in,out] pstr_psy_mod    	Pointer to psychoacoustic model structure
 *  \param [in,out] pstr_igf_data   	Pointer to IGF data structure
 *  \param [in]     pstr_sfb_prms       Pointer to scalefactor band parameter structure
 *  \param [in]     ptr_spec_in         Pointer to input spectral coefficients
 *  \param [in,out] pstr_tns_info    	Pointer to TNS info structure
 *  \param [out]    tns_select         	Flag indicating if TNS is to be used
 *  \param [in]     i_ch                Channel offset
 *  \param [in]     chn                 Channel index
 *  \param [in]     igf_after_tns_synth IGF after TNS flag
 *  \param [in,out] pstr_igf_config 	Pointer to IGF configuration structure
 *  \param [in]     channel_type        Channel element type
 *  \param [in]     ptr_scratch_tns_filter  Pointer to scratch buffer for TNS filter
 *  \param [in]     elem_idx            Element index
 *  \param [in]     ptr_igf_scratch     Pointer to IGF scratch buffer
 *  \param [in]     ptr_tns_scratch     Pointer to TNS scratch buffer

 *
 *  \return VOID
 */
VOID impeghe_psy_mod_sb(ia_psy_mod_struct *pstr_psy_mod, ia_igf_data_struct *pstr_igf_data,
                        ia_sfb_params_struct *pstr_sfb_prms, FLOAT64 *ptr_spec_in,
                        ia_tns_info *pstr_tns_info[MAX_TIME_CHANNELS], WORD32 tns_select,
                        WORD32 i_ch, WORD32 chn, WORD32 igf_after_tns_synth,
                        ia_igf_config_struct *pstr_igf_config, WORD32 channel_type,
                        FLOAT64 *ptr_scratch_tns_filter, WORD32 elem_idx,
                        FLOAT64 *ptr_igf_scratch, FLOAT64 *ptr_tns_scratch)
{
  ia_psy_mod_data_struct *pstr_psy_data = &(pstr_psy_mod->str_psy_data[i_ch]);
  ia_psy_mod_short_config_struct *pstr_psy_config =
      &(pstr_psy_mod->str_psy_short_config[elem_idx]);
  WORD32 max_sfb = 0, sfb, line;
  WORD32 window_sequence = pstr_sfb_prms->window_sequence[i_ch];
  WORD32 num_sfb = pstr_sfb_prms->num_sfb[i_ch];
  ia_tns_info *ptr_tns_info = pstr_tns_info[i_ch];
  WORD32 i, w;
  FLOAT32 energy_shift = 0.25f;
  FLOAT32 clip_energy = pstr_psy_config->clip_energy * energy_shift;
  WORD32 igf_offset = 0;
  WORD32 igf_grp = 0, group_offset = 0;

  pstr_psy_data->window_sequence = window_sequence;

  for (w = 0; w < MAX_SHORT_WINDOWS; w++)
  {
    WORD32 w_offset = w * FRAME_LEN_SHORT;
    WORD32 offset;
    FLOAT64 *pmdct_double = &ptr_spec_in[pstr_psy_config->low_pass_line + w_offset];

    offset = FRAME_LEN_SHORT - pstr_psy_config->low_pass_line;

    memset(pmdct_double, 0, sizeof(FLOAT64) * offset);

    impeghe_calc_band_energy(ptr_spec_in + w_offset, pstr_psy_config->sfb_offset,
                             pstr_psy_config->sfb_active, pstr_psy_data->sfb_energy_short[w],
                             pstr_psy_config->sfb_count);

    if (channel_type == ID_USAC_LFE)
    {
      tns_select = 0;
    }

    if (pstr_igf_config->igf_active == 1)
    {
      pstr_igf_config->is_short_block = 1;
      pstr_igf_config->m_igf_start_sfb = pstr_igf_config->igf_start_sfb_sb;
      pstr_igf_config->m_igf_stop_sfb = pstr_igf_config->igf_stop_sfb_sb;
    }

    if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active == 1 &&
        igf_after_tns_synth == 0 &&
        ((group_offset + pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT ==
              w_offset &&
          w != 0) ||
         w == 0))
    {
      impeghe_igf(ptr_spec_in + igf_offset, pstr_sfb_prms->sfb_offset[i_ch], pstr_igf_data,
                  pstr_igf_config, igf_grp, 0, pstr_sfb_prms->window_group_length[i_ch][igf_grp],
                  0, igf_after_tns_synth, 0, 0, ptr_igf_scratch);
      if (w != 0)
      {
        group_offset += pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT;
      }
      igf_offset += (pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT);
      igf_grp++;
    }

    if (tns_select != 0)
    {
      ia_tns_info *ptr_tns_info_ch2 = pstr_tns_info[i_ch - chn];
      ptr_tns_info->number_of_bands = num_sfb;
      ptr_tns_info->block_type = window_sequence;
      ptr_tns_info->spec = ptr_spec_in + w_offset;
      impeghe_tns_encode(ptr_tns_info_ch2, ptr_tns_info, pstr_psy_data->sfb_energy_short[w], w,
                         chn, pstr_psy_config->low_pass_line, pstr_igf_config,
                         ptr_scratch_tns_filter, 0, ptr_tns_scratch);
    }

    if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active == 1 &&
        igf_after_tns_synth == 1 &&
        ((group_offset + pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT ==
              w_offset &&
          w != 0) ||
         w == 0))
    {
      impeghe_igf(ptr_spec_in + igf_offset, pstr_sfb_prms->sfb_offset[i_ch], pstr_igf_data,
                  pstr_igf_config, igf_grp, ptr_tns_info->window_data[w].tns_pred_gain,
                  pstr_sfb_prms->window_group_length[i_ch][igf_grp], 0, igf_after_tns_synth, 0, 0,
                  ptr_igf_scratch);
      if (w != 0)
      {
        group_offset += pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT;
      }
      igf_offset += (pstr_sfb_prms->window_group_length[i_ch][igf_grp] * FRAME_LEN_SHORT);
      igf_grp++;
    }

    for (i = 0; i < pstr_psy_config->sfb_count; i++)
    {
      pstr_psy_data->sfb_thr_short[w][i] =
          pstr_psy_data->sfb_energy_short[w][i] * pstr_psy_config->ratio;
      pstr_psy_data->sfb_thr_short[w][i] = min(pstr_psy_data->sfb_thr_short[w][i], clip_energy);
    }

    if (tns_select != 0)
    {
      if (ptr_tns_info->tns_data_present == 1)
      {
        impeghe_calc_band_energy(ptr_spec_in + w_offset, pstr_psy_config->sfb_offset,
                                 pstr_psy_config->sfb_active, pstr_psy_data->sfb_energy_short[w],
                                 pstr_psy_config->sfb_count);
      }
    }
    if (pstr_igf_config->igf_independent_tiling == 1 && pstr_igf_config->igf_active != 0)
    {
      impeghe_calc_band_energy(ptr_spec_in, pstr_psy_config->sfb_offset,
                               pstr_psy_config->sfb_active, pstr_psy_data->ptr_sfb_energy_long,
                               pstr_psy_config->sfb_count);
    }

    impeghe_find_max_spreading(pstr_psy_config->sfb_count, pstr_psy_config->sfb_mask_low_fac,
                               pstr_psy_config->sfb_mask_high_fac,
                               pstr_psy_data->sfb_thr_short[w]);

    for (i = 0; i < pstr_psy_config->sfb_count; i++)
    {
      pstr_psy_data->sfb_thr_short[w][i] =
          max(pstr_psy_data->sfb_thr_short[w][i], (pstr_psy_config->sfb_thr_quiet[i] * 0.25f));
    }

    impeghe_pre_echo_control(pstr_psy_data->sfb_thr_nm1, pstr_psy_config->sfb_count,
                             pstr_psy_config->max_allowed_inc_fac,
                             pstr_psy_config->min_remaining_thr_fac,
                             pstr_psy_data->sfb_thr_short[w]);

    for (i = 0; i < pstr_psy_config->sfb_count; i++)
    {
      pstr_psy_data->sfb_spreaded_energy_short[w][i] = pstr_psy_data->sfb_energy_short[w][i];
    }
    impeghe_find_max_spreading(
        pstr_psy_config->sfb_count, pstr_psy_config->sfb_mask_low_fac_spr_ener,
        pstr_psy_config->sfb_mask_high_fac_spr_ener, pstr_psy_data->sfb_spreaded_energy_short[w]);
  }

  for (WORD32 wnd = 0; wnd < MAX_SHORT_WINDOWS; wnd++)
  {
    for (sfb = pstr_psy_config->sfb_count - 1; sfb >= max_sfb; sfb--)
    {
      for (line = pstr_psy_config->sfb_offset[sfb + 1] - 1;
           line >= pstr_psy_config->sfb_offset[sfb]; line--)
      {
        if (ptr_spec_in[wnd * 128 + line] != 0.0)
          break;
      }
      if (line >= pstr_psy_config->sfb_offset[sfb])
        break;
    }
    max_sfb = max(max_sfb, sfb);
  }
  max_sfb = max_sfb > 0 ? max_sfb : 0;

  pstr_psy_mod->str_psy_out_data[i_ch].max_sfb_per_grp = max_sfb + 1;

  return;
}
