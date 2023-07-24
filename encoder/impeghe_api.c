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
#include "impeghe_type_def.h"
#include "impeghe_api.h"
#include "impeghe_cnst.h"
#include "impeghe_error_standards.h"
#include "impeghe_apicmd_standards.h"
#include "impeghe_error_codes.h"

#include "impeghe_block_switch_const.h"
#include "impeghe_bitbuffer.h"

#include "impeghe_igf_enc.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_drc_enc.h"
#include "impeghe_tns_usac.h"

#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_memory_standards.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"
#include "impeghe_mhas_write.h"
#include "impeghe_write_bitstream.h"
#include "impeghe_config.h"
#include "impeghe_version_number.h"
#include "impeghe_rom.h"
#include "impeghe_crc.h"
#include <setjmp.h>

#define IMPEGHE_NUM_MEMTABS 5
#define LIB_NAME "IMPEGH_LC_ENC"

#define IMPEGHE_FLOAT32_SCRATCH_SIZE 47704
#define IMPEGHE_FLOAT64_SCRATCH_SIZE                                                             \
  (2048 + 1024) /* Increased by 1024 for complex prediction \                                    \
                 */

IA_ERRORCODE impeghe_enc_fill_mem_tables(ia_mpeghe_api_struct *p_obj_mpeghe);
IA_ERRORCODE impeghe_initialize(ia_mpeghe_api_struct *p_obj_mpeghe);
IA_ERRORCODE impeghe_process(ia_mpeghe_api_struct *p_obj_mpeghe, WORD32 *header_bytes);

/**
 *  impeghe_check_config_params
 *
 *  \brief Check config paramters and set them to default if any of them is invalid
 *
 *  \param [in,out] pstr_input_config  Pointer to input config
 *
 *  \return VOID
 *
 */
static VOID impeghe_check_config_params(ia_input_config *pstr_input_config)
{
  if (pstr_input_config->aud_ch_pcm_cfg.n_channels > 0)
  {
    if (pstr_input_config->bitrate < 32000 && pstr_input_config->aud_ch_pcm_cfg.n_channels == 1)
    {
      pstr_input_config->bitrate = 32000;
    }
    else if (pstr_input_config->bitrate < 64000 &&
             pstr_input_config->aud_ch_pcm_cfg.n_channels == 2)
    {
      pstr_input_config->bitrate = 64000;
    }
    else if (pstr_input_config->bitrate < 192000 &&
             pstr_input_config->aud_ch_pcm_cfg.n_channels == 6)
    {
      pstr_input_config->bitrate = 192000;
    }
    else if (pstr_input_config->bitrate < 256000 &&
             pstr_input_config->aud_ch_pcm_cfg.n_channels == 8)
    {
      pstr_input_config->bitrate = 256000;
    }
    else if (pstr_input_config->bitrate < 320000 &&
             pstr_input_config->aud_ch_pcm_cfg.n_channels == 10)
    {
      pstr_input_config->bitrate = 320000;
    }
  }
  else
  {
    if (pstr_input_config->bitrate < 32000)
    {
      pstr_input_config->bitrate = 32000;
    }
  }
  if (pstr_input_config->tns_enable != 0 && pstr_input_config->tns_enable != 1)
  {
    pstr_input_config->tns_enable = 1;
  }
  if (pstr_input_config->codec_mode != USAC_SWITCHED &&
      pstr_input_config->codec_mode != USAC_ONLY_FD &&
      pstr_input_config->codec_mode != USAC_ONLY_TD)
  {
    pstr_input_config->codec_mode = USAC_ONLY_FD;
  }
  if (pstr_input_config->noise_filling != 0 && pstr_input_config->noise_filling != 1)
  {
    pstr_input_config->noise_filling = 0;
  }
  if (pstr_input_config->fill_elem != 0 && pstr_input_config->fill_elem != 1)
  {
    pstr_input_config->fill_elem = 1;
  }
  if (pstr_input_config->prof_level != PROFILE_LC_LVL1 &&
      pstr_input_config->prof_level != PROFILE_LC_LVL2 &&
      pstr_input_config->prof_level != PROFILE_LC_LVL3 &&
      pstr_input_config->prof_level != PROFILE_LC_LVL4 &&
      pstr_input_config->prof_level != PROFILE_BL_LVL1 &&
      pstr_input_config->prof_level != PROFILE_BL_LVL2 &&
      pstr_input_config->prof_level != PROFILE_BL_LVL3)
  {
    pstr_input_config->prof_level = PROFILE_LC_LVL1;
  }
  if (pstr_input_config->cplx_pred != 0 && pstr_input_config->cplx_pred != 1)
  {
    pstr_input_config->cplx_pred = 0;
  }
  if (pstr_input_config->out_fmt != 1 && pstr_input_config->out_fmt != 2 &&
      pstr_input_config->out_fmt != 3)
  {
    pstr_input_config->out_fmt = 1;
  }
  if (pstr_input_config->cicp_index < 0 || pstr_input_config->cicp_index > IMPEGHE_NUM_LS_CFGS ||
      pstr_input_config->cicp_index == 8)
  {
    pstr_input_config->cicp_index = 0;
  }
  if (pstr_input_config->use_oam_element)
  {
    if (pstr_input_config->oam_high_rate != 0 && pstr_input_config->oam_high_rate != 1)
    {
      pstr_input_config->oam_high_rate = 1;
    }
  }
  if (pstr_input_config->use_hoa_element)
  {
    if (pstr_input_config->use_vec_est != 0 && pstr_input_config->use_vec_est != 1)
    {
      pstr_input_config->use_vec_est = 0;
    }
  }
  if (pstr_input_config->use_drc_element != 0 && pstr_input_config->use_drc_element != 1)
  {
    pstr_input_config->use_drc_element = 0;
  }
  if (pstr_input_config->crc16 != 0 && pstr_input_config->crc16 != 1)
  {
    pstr_input_config->crc16 = 0;
  }
  if (pstr_input_config->crc32 != 0 && pstr_input_config->crc32 != 1)
  {
    pstr_input_config->crc32 = 0;
  }
  if (pstr_input_config->global_crc16 != 0 && pstr_input_config->global_crc16 != 1)
  {
    pstr_input_config->global_crc16 = 0;
  }
  if (pstr_input_config->global_crc32 != 0 && pstr_input_config->global_crc32 != 1)
  {
    pstr_input_config->global_crc32 = 0;
  }
  if (pstr_input_config->enhanced_noise_filling != 0 &&
      pstr_input_config->enhanced_noise_filling != 1)
  {
    pstr_input_config->enhanced_noise_filling = 1;
  }
  if (pstr_input_config->igf_after_tns_synth != 0 && pstr_input_config->igf_after_tns_synth != 1)
  {
    pstr_input_config->igf_after_tns_synth = 1;
  }
  if (pstr_input_config->igf_start_freq < 0 || pstr_input_config->igf_start_freq > 20000)
  {
    pstr_input_config->igf_start_freq = IGF_START_SR48_BR32;
    pstr_input_config->igf_start_freq_flag = 1;
  }
  if (pstr_input_config->igf_stop_freq < 0 || pstr_input_config->igf_stop_freq > 20000)
  {
    pstr_input_config->igf_stop_freq = IGF_STOP_SR48;
    pstr_input_config->igf_stop_freq_flag = 1;
  }
  if (pstr_input_config->ltpf_enable != 0 && pstr_input_config->ltpf_enable != 1)
  {
    pstr_input_config->ltpf_enable = 0;
  }
  if (pstr_input_config->fdp_enable != 0 && pstr_input_config->fdp_enable != 1)
  {
    pstr_input_config->fdp_enable = 0;
  }
  if (pstr_input_config->kernel < 0 || pstr_input_config->kernel > 3)
  {
    pstr_input_config->kernel = 0;
    pstr_input_config->prev_aliasing_symmetry = 0;
    pstr_input_config->curr_aliasing_symmetry = 0;
  }
  if (pstr_input_config->full_band_lpd != 0 && pstr_input_config->full_band_lpd != 1)
  {
    pstr_input_config->full_band_lpd = 0;
  }
  if (pstr_input_config->stereo_lpd != 0 && pstr_input_config->stereo_lpd != 1)
  {
    pstr_input_config->stereo_lpd = 0;
  }
  if (pstr_input_config->mct_mode < -1 || pstr_input_config->mct_mode > 3)
  {
    pstr_input_config->mct_mode = -1;
  }
  if (pstr_input_config->str_ec_info_struct.ec_present)
  {
    if (pstr_input_config->str_ec_info_struct.ec_start_frame < 0)
    {
      pstr_input_config->str_ec_info_struct.ec_start_frame = 0;
    }
    if (pstr_input_config->str_ec_info_struct.ec_frame_cnt < 0)
    {
      pstr_input_config->str_ec_info_struct.ec_frame_cnt = 0;
    }
    if (pstr_input_config->str_ec_info_struct.ec_count < 0)
    {
      pstr_input_config->str_ec_info_struct.ec_count = 0;
    }
  }
}

/**
 *  impeghe_set_config_params
 *
 *  \brief Set the configurations as per application's settings
 *
 *  \param [out]	p_obj_mpeghe		Pointer to API structure
 *  \param [in]		pstr_input_config	Pointer to received input configuration structure
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE impeghe_set_config_params(ia_mpeghe_api_struct *p_obj_mpeghe,
                                              ia_input_config *pstr_input_config)
{
  IA_ERRORCODE error = IA_NO_ERROR;
  LOOPIDX idx;
  LOOPIDX i, j;
  ia_usac_audio_specific_config_struct *pstr_asc = &p_obj_mpeghe->config.audio_specific_config;

  impeghe_check_config_params(pstr_input_config);

  if((pstr_input_config->aud_ch_pcm_cfg.n_channels > 24) ||
     (pstr_input_config->aud_ch_pcm_cfg.n_channels < 0))
  {
    return IMPEGHE_CONFIG_FATAL_NUM_CHANNELS_UNSUPPORTED;
  }

  if (pstr_input_config->aud_ch_pcm_cfg.n_channels > 0)
  {
    pstr_input_config->sample_rate = pstr_input_config->aud_ch_pcm_cfg.sample_rate;
  }
  else if (pstr_input_config->aud_obj_pcm_cfg.n_channels > 0)
  {
    pstr_input_config->sample_rate = pstr_input_config->aud_obj_pcm_cfg.sample_rate;
  }
  else if (pstr_input_config->hoa_pcm_cfg.n_channels > 0)
  {
    pstr_input_config->sample_rate = pstr_input_config->hoa_pcm_cfg.sample_rate;
  }

  if ((pstr_input_config->aud_obj_pcm_cfg.n_channels > 0) &&
      (pstr_input_config->hoa_pcm_cfg.n_channels > 0))
  {
    // HOA and OAM of different sample rate not supported
    if (pstr_input_config->aud_obj_pcm_cfg.sample_rate !=
        pstr_input_config->hoa_pcm_cfg.sample_rate)
    {
      return (IMPEGHE_CONFIG_FATAL_SAMP_FREQ);
    }
  }

  for (idx = 0;
       idx < sizeof(impeghe_sampl_freq_idx_table) / sizeof(impeghe_sampl_freq_idx_table[0]);
       idx++)
  {
    if (pstr_input_config->sample_rate == impeghe_sampl_freq_idx_table[idx])
      break;
  }

  if ((pstr_input_config->sample_rate != 29400) && (pstr_input_config->sample_rate != 14700))
  {
    if (idx < 3 || idx > 8)
    {
      return (IMPEGHE_CONFIG_FATAL_SAMP_FREQ);
    }
  }

  p_obj_mpeghe->config.num_objects = pstr_input_config->num_objects;
  p_obj_mpeghe->config.num_channels = pstr_input_config->num_channels;
  p_obj_mpeghe->config.extra_objects = pstr_input_config->extra_objects;

  if (OAM_MAX_NUM_OBJECTS < p_obj_mpeghe->config.num_objects)
  {
    p_obj_mpeghe->config.extra_objects = p_obj_mpeghe->config.num_objects - OAM_MAX_NUM_OBJECTS;
    p_obj_mpeghe->config.num_objects = OAM_MAX_NUM_OBJECTS;
    error = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
  }

  if (OAM_MAX_NUM_OBJECTS <
      (p_obj_mpeghe->config.num_objects + p_obj_mpeghe->config.num_channels))
  {
    p_obj_mpeghe->config.extra_objects = p_obj_mpeghe->config.num_objects +
                                         p_obj_mpeghe->config.num_channels - OAM_MAX_NUM_OBJECTS;
    p_obj_mpeghe->config.num_objects = OAM_MAX_NUM_OBJECTS - p_obj_mpeghe->config.num_channels;
    error = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
  }

  p_obj_mpeghe->config.num_oam_ch =
      p_obj_mpeghe->config.num_channels + p_obj_mpeghe->config.num_objects;

  p_obj_mpeghe->config.channels = pstr_input_config->aud_ch_pcm_cfg.n_channels +
                                  pstr_input_config->num_trans_ch +
                                  p_obj_mpeghe->config.num_oam_ch;

  if ((pstr_input_config->bitrate >
       (((MAX_CHANNEL_BITS / FRAME_LEN_LONG) * pstr_input_config->sample_rate *
         p_obj_mpeghe->config.channels))))
  {
    return (IMPEGHE_CONFIG_FATAL_BITRATE);
  }
  for (j = 0; j < MAX_TIME_CHANNELS; j++)
  {
    p_obj_mpeghe->config.audio_specific_config.num_ch_idx_per_grp[j] = j;
  }
  WORD32 ch_offset_idx = 0;
  pstr_asc->num_ch_sig_groups = pstr_input_config->num_ch_sig_groups;
  for (i = 0; i < pstr_input_config->num_ch_sig_groups; i++)
  {
    pstr_asc->num_ch_per_sig_group[i] = pstr_input_config->num_ch_per_sig_grp[i];
    for (j = 0; j < pstr_asc->num_ch_per_sig_group[i]; j++)
    {
      pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
          pstr_input_config->num_ch_idx_per_grp[ch_offset_idx];
      ch_offset_idx++;
    }
  }
  pstr_asc->num_obj_sig_groups = pstr_input_config->num_obj_sig_groups;
  for (i = 0; i < pstr_input_config->num_obj_sig_groups; i++)
  {
    pstr_asc->num_objs_per_sig_group[i] = pstr_input_config->num_objs_per_sig_grp[i];
    for (j = 0; j < pstr_asc->num_objs_per_sig_group[i]; j++)
    {
      pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
          pstr_input_config->num_ch_idx_per_grp[ch_offset_idx];
      ch_offset_idx++;
    }
  }

  pstr_asc->num_hoa_sig_groups = pstr_input_config->num_hoa_sig_groups;
  for (i = 0; i < pstr_input_config->num_hoa_sig_groups; i++)
  {
    pstr_asc->num_hoas_per_sig_group[i] = pstr_input_config->num_hoas_per_sig_grp[i];
    for (j = 0; j < pstr_asc->num_hoas_per_sig_group[i]; j++)
    {
      pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
          pstr_input_config->num_ch_idx_per_grp[ch_offset_idx];
      ch_offset_idx++;
    }
  }
  p_obj_mpeghe->config.bit_rate = pstr_input_config->bitrate;
  p_obj_mpeghe->config.enhanced_noise_filling = pstr_input_config->enhanced_noise_filling;
  p_obj_mpeghe->config.igf_start_freq = pstr_input_config->igf_start_freq;
  p_obj_mpeghe->config.igf_stop_freq = pstr_input_config->igf_stop_freq;
  p_obj_mpeghe->config.igf_after_tns_synth = pstr_input_config->igf_after_tns_synth;
  p_obj_mpeghe->config.ltpf_enable = pstr_input_config->ltpf_enable;
  WORD16 sr_idx = min(4, (pstr_input_config->sample_rate / 8000) - 1);
  if (p_obj_mpeghe->config.ltpf_enable == 1)
  {
    if ((pstr_input_config->bitrate / p_obj_mpeghe->config.channels) <
        impeghe_ltpf_max_bit_rate[sr_idx])
    {
      p_obj_mpeghe->config.ltpf_enable = 1;
    }
    else
    {
      p_obj_mpeghe->config.ltpf_enable = 0;
    }
  }
  p_obj_mpeghe->config.full_band_lpd = pstr_input_config->full_band_lpd;
  p_obj_mpeghe->config.stereo_lpd = pstr_input_config->stereo_lpd;
  p_obj_mpeghe->config.fdp_enable = pstr_input_config->fdp_enable;
  p_obj_mpeghe->config.flag_noiseFilling = pstr_input_config->noise_filling;
  p_obj_mpeghe->config.tns_select = pstr_input_config->tns_enable;

  if ((p_obj_mpeghe->config.igf_after_tns_synth == 1) &&
      (p_obj_mpeghe->config.enhanced_noise_filling == 1))
  {
    p_obj_mpeghe->config.tns_select = 1;
  }
  p_obj_mpeghe->config.sampling_rate = pstr_input_config->sample_rate;
  p_obj_mpeghe->config.native_sampling_rate = pstr_input_config->sample_rate;
  p_obj_mpeghe->config.aud_ch = pstr_input_config->aud_ch_pcm_cfg.n_channels;
  p_obj_mpeghe->config.num_trans_ch = pstr_input_config->num_trans_ch;

  // Bitrate dirtribution among all element types
  // used for converting to 16 bit pcm format. So, aud_ch_pcm_cfg.n_channels = 0 when OAM is
  // present and vice versa
  // HOA does not need this conversion as hoa enc input is float
  if (pstr_input_config->aud_ch_pcm_cfg.n_channels > 0)
  {
    p_obj_mpeghe->config.ui_pcm_wd_sz = pstr_input_config->aud_ch_pcm_cfg.pcm_sz;
  }
  else if (pstr_input_config->aud_obj_pcm_cfg.n_channels > 0)
  {
    p_obj_mpeghe->config.ui_pcm_wd_sz = pstr_input_config->aud_obj_pcm_cfg.pcm_sz;
  }
  p_obj_mpeghe->config.codec_mode = pstr_input_config->codec_mode;

  if (p_obj_mpeghe->config.full_band_lpd == 0 &&
      (p_obj_mpeghe->config.codec_mode != USAC_ONLY_FD))
  {
    p_obj_mpeghe->config.enhanced_noise_filling = 0;
    p_obj_mpeghe->config.igf_after_tns_synth = 0;
    p_obj_mpeghe->config.tns_select = 0;
  }
  p_obj_mpeghe->config.cmplx_pred_flag = pstr_input_config->cplx_pred;
  p_obj_mpeghe->config.mhas_pkt = pstr_input_config->mhas_pkt;
  p_obj_mpeghe->config.crc16 = pstr_input_config->crc16;
  p_obj_mpeghe->config.crc32 = pstr_input_config->crc32;
  p_obj_mpeghe->config.global_crc16 = pstr_input_config->global_crc16;
  p_obj_mpeghe->config.global_crc32 = pstr_input_config->global_crc32;
  p_obj_mpeghe->config.prev_aliasing_symmetry = pstr_input_config->prev_aliasing_symmetry;
  p_obj_mpeghe->config.curr_aliasing_symmetry = pstr_input_config->curr_aliasing_symmetry;
  p_obj_mpeghe->config.use_fill_element = pstr_input_config->fill_elem;
  p_obj_mpeghe->config.prof_level = pstr_input_config->prof_level;
  if (pstr_input_config->str_ec_info_struct.ec_present)
  {
    if (pstr_input_config->str_ec_info_struct.ec_start_frame == 0)
    {
      p_obj_mpeghe->config.str_ec_config_struct.ec_active = 1;
    }
    else
    {
      p_obj_mpeghe->config.str_ec_config_struct.ec_active = 0;
    }
    p_obj_mpeghe->config.str_pcm_data_config.bsnum_pcm_signals_in_frame = 0;
    p_obj_mpeghe->config.str_ec_config_struct.sample_rate =
        pstr_input_config->str_ec_info_struct.str_pcm_config.sample_rate;
    p_obj_mpeghe->config.str_ec_config_struct.cicp_idx = pstr_input_config->cicp_index;
    p_obj_mpeghe->config.str_ec_config_struct.pcm_sz =
        pstr_input_config->str_ec_info_struct.str_pcm_config.pcm_sz;
    p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample =
        pstr_input_config->str_ec_info_struct.str_pcm_config.pcm_sz;
    p_obj_mpeghe->config.str_ec_config_struct.num_earcons =
        pstr_input_config->str_ec_info_struct.ec_count;
    if (p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample == 16)
      p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample_idx = 0;
    else if (p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample == 24)
      p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample_idx = 1;
    else
      p_obj_mpeghe->config.str_pcm_data_config.pcm_bits_per_sample_idx = 2;
    p_obj_mpeghe->config.str_pcm_data_config.pcm_frame_size_idx = 2;
    p_obj_mpeghe->config.str_ec_config_struct.i_channel_mask =
        pstr_input_config->str_ec_info_struct.str_pcm_config.i_channel_mask;
    p_obj_mpeghe->config.str_ec_config_struct.n_channels =
        pstr_input_config->str_ec_info_struct.str_pcm_config.n_channels;
    p_obj_mpeghe->config.str_pcm_data_config.pcm_sample =
        pstr_input_config->str_ec_info_struct.ptr_ec_buff;
    p_obj_mpeghe->config.str_pcm_data_config.pcm_sampling_rate =
        pstr_input_config->str_ec_info_struct.str_pcm_config.sample_rate;
  }
  if ((pstr_input_config->mct_mode < 0) || (pstr_input_config->mct_mode > 3) ||
      (p_obj_mpeghe->config.codec_mode != USAC_ONLY_FD) ||
      ((pstr_input_config->aud_ch_pcm_cfg.n_channels < 3) &&
       (p_obj_mpeghe->config.num_oam_ch < 3)))
  {
    p_obj_mpeghe->config.mct_mode = -1;
  }
  else
  {
    p_obj_mpeghe->config.mct_mode = pstr_input_config->mct_mode;
  }
  if (p_obj_mpeghe->config.mct_mode > 1)
  {
    p_obj_mpeghe->config.flag_noiseFilling = 1;
    pstr_input_config->noise_filling = 1;
  }
  p_obj_mpeghe->config.user_specified = pstr_input_config->user_specified_sig_grp;
  // OAM Params
  p_obj_mpeghe->config.use_oam_element = pstr_input_config->use_oam_element;
  p_obj_mpeghe->config.cicp_index = pstr_input_config->cicp_index;
  if (pstr_input_config->use_oam_element)
  {
    // If OAM is in use, then ifile option is not valid
    if (pstr_input_config->use_hoa_element)
    {
      p_obj_mpeghe->config.oam_bitrate =
          p_obj_mpeghe->config.num_oam_ch * MINIMUM_BITRATE_PER_CHANNEL;
      if (p_obj_mpeghe->config.oam_bitrate > p_obj_mpeghe->config.bit_rate)
      {
        return IMPEGHE_CONFIG_FATAL_BITRATE;
      }
    }
    else
    {
      p_obj_mpeghe->config.oam_bitrate = p_obj_mpeghe->config.bit_rate;
    }
  }

  p_obj_mpeghe->config.audio_specific_config.flex_spk_enable =
      pstr_input_config->flexi_spk_enable;
  if (p_obj_mpeghe->config.audio_specific_config.flex_spk_enable)
  {
    for (WORD32 i = 0; i < pstr_input_config->str_flexi_spk_config.num_speaker; i++)
    {
      p_obj_mpeghe->config.audio_specific_config.flex_spk_azi[i] =
          pstr_input_config->str_flexi_spk_config.flex_spk_azi[i];
      p_obj_mpeghe->config.audio_specific_config.flex_spk_ele[i] =
          pstr_input_config->str_flexi_spk_config.flex_spk_ele[i];
      p_obj_mpeghe->config.audio_specific_config.flex_spk_islfe[i] =
          pstr_input_config->str_flexi_spk_config.flex_spk_islfe[i];
    }
    p_obj_mpeghe->config.audio_specific_config.num_spk =
        pstr_input_config->str_flexi_spk_config.num_speaker;
  }
  if (pstr_input_config->asi_enable)
  {
    ia_mae_audio_scene_info *pstr_mae_info =
        (ia_mae_audio_scene_info *)&p_obj_mpeghe->config.audio_specific_config.str_asi_info;
    pstr_mae_info->asi_present = pstr_input_config->asi_enable;
    pstr_mae_info->main_stream_flag = pstr_input_config->str_asi_config.main_stream_flag;
    pstr_mae_info->mae_id_offset = pstr_input_config->str_asi_config.mae_id_offset;
    pstr_mae_info->mae_id_max_avail = pstr_input_config->str_asi_config.mae_id_max_avail;
    pstr_mae_info->mae_data.num_data_sets = pstr_input_config->str_asi_config.num_data_sets;
    pstr_mae_info->num_groups = pstr_input_config->str_asi_config.num_groups;
    pstr_mae_info->num_group_presets = pstr_input_config->str_asi_config.num_group_presets;
    pstr_mae_info->num_switch_groups = pstr_input_config->str_asi_config.num_switch_groups;
    if (pstr_mae_info->num_groups)
    {
      for (idx = 0; idx < pstr_mae_info->num_groups; idx++)
      {
        pstr_mae_info->group_definition[idx].default_on_off =
            pstr_input_config->str_asi_config.grp_def_default_on_off[idx];
        pstr_mae_info->group_definition[idx].group_id =
            pstr_input_config->str_asi_config.grp_def_grp_id[idx];
        pstr_mae_info->group_definition[idx].allow_pos_interact =
            pstr_input_config->str_asi_config.grp_def_allow_pos_interact[idx];
        pstr_mae_info->group_definition[idx].allow_on_off =
            pstr_input_config->str_asi_config.grp_def_allow_on_off[idx];
        pstr_mae_info->group_definition[idx].allow_pos_interact =
            pstr_input_config->str_asi_config.grp_def_allow_pos_interact[idx];

        pstr_mae_info->group_definition[idx].min_az_offset =
            pstr_input_config->str_asi_config.grp_def_min_az_offset[idx];

        pstr_mae_info->group_definition[idx].max_az_offset =
            pstr_input_config->str_asi_config.grp_def_max_az_offset[idx];

        pstr_mae_info->group_definition[idx].min_el_offset =
            pstr_input_config->str_asi_config.grp_def_min_el_offset[idx];

        pstr_mae_info->group_definition[idx].max_el_offset =
            pstr_input_config->str_asi_config.grp_def_max_el_offset[idx];
        pstr_mae_info->group_definition[idx].min_dist_factor =
            pstr_input_config->str_asi_config.grp_def_min_dist_factor[idx];

        pstr_mae_info->group_definition[idx].max_dist_factor =
            pstr_input_config->str_asi_config.grp_def_max_dist_factor[idx];

        pstr_mae_info->group_definition[idx].allow_gain_interact =
            pstr_input_config->str_asi_config.grp_def_allow_gain_interact[idx];

        pstr_mae_info->group_definition[idx].min_gain =
            pstr_input_config->str_asi_config.grp_def_min_gain[idx];

        pstr_mae_info->group_definition[idx].max_gain =
            pstr_input_config->str_asi_config.grp_def_max_gain[idx];

        pstr_mae_info->group_definition[idx].group_num_members =
            pstr_input_config->str_asi_config.grp_def_group_num[idx];
        if (pstr_mae_info->group_definition[idx].group_num_members < 1)
        {
          return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
        }
        pstr_mae_info->group_definition[idx].has_conjunct_members =
            pstr_input_config->str_asi_config.has_conjunct_members[idx];

        pstr_mae_info->group_definition[idx].start_id =
            pstr_input_config->str_asi_config.grp_def_start_id[idx];

        for (WORD32 j = 0; j < pstr_mae_info->group_definition[idx].group_num_members; j++)
        {
          pstr_mae_info->group_definition[idx].metadata_ele_id[j] =
              pstr_input_config->str_asi_config.grp_def_metadata_ele_id[idx][j];
        }
      }
    }
    if (pstr_mae_info->num_switch_groups)
    {
      for (idx = 0; idx < pstr_mae_info->num_switch_groups; idx++)
      {
        pstr_mae_info->switch_group_definition[idx].default_on_off =
            pstr_input_config->str_asi_config.switch_grp_def_default_on_off[idx];
        pstr_mae_info->switch_group_definition[idx].group_id =
            pstr_input_config->str_asi_config.switch_grp_def_grp_id[idx];
        pstr_mae_info->switch_group_definition[idx].allow_on_off =
            pstr_input_config->str_asi_config.switch_grp_def_allow_on_off[idx];

        pstr_mae_info->switch_group_definition[idx].group_num_members =
            pstr_input_config->str_asi_config.switch_grp_def_grp_num_member[idx];
        if (pstr_mae_info->switch_group_definition[idx].group_num_members < 1)
        {
          return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
        }
        for (WORD32 j = 0; j < pstr_mae_info->switch_group_definition[idx].group_num_members; j++)
        {
          pstr_mae_info->switch_group_definition[idx].member_id[j] =
              pstr_input_config->str_asi_config.switch_grp_def_grp_member_id[idx][j];
        }
        pstr_mae_info->switch_group_definition[idx].default_group_id =
            pstr_input_config->str_asi_config.switch_grp_def_default_group_id[idx];
      }
    }
    if (pstr_mae_info->num_group_presets)
    {
      for (idx = 0; idx < pstr_mae_info->num_group_presets; idx++)
      {
        pstr_mae_info->group_presets_definition[idx].group_id =
            pstr_input_config->str_asi_config.grp_preset_def_grp_id[idx];
        pstr_mae_info->group_presets_definition[idx].preset_kind =
            pstr_input_config->str_asi_config.grp_preset_def_preset_kind[idx];
        pstr_mae_info->group_presets_definition[idx].num_conditions[0] =
            pstr_input_config->str_asi_config.grp_preset_def_num_conditions[idx];
        if (pstr_mae_info->group_presets_definition[idx].num_conditions[0] < 1)
        {
          return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
        }
        for (WORD32 i = 0; i < pstr_mae_info->group_presets_definition[idx].num_conditions[0];
             i++)
        {
          pstr_mae_info->group_presets_definition[idx].reference_id[i] =
              pstr_input_config->str_asi_config.grp_preset_def_reference_id[idx][i];
          pstr_mae_info->group_presets_definition[idx].cond_on_off[i] =
              pstr_input_config->str_asi_config.grp_preset_def_cond_on_off[idx][i];

          pstr_mae_info->group_presets_definition[idx].disable_gain_interact[i] =
              pstr_input_config->str_asi_config.grp_preset_def_disable_gain_interact[idx][i];

          pstr_mae_info->group_presets_definition[idx].gain_flag[i] =
              pstr_input_config->str_asi_config.grp_preset_def_gain_flag[idx][i];

          pstr_mae_info->group_presets_definition[idx].gain[i] =
              pstr_input_config->str_asi_config.grp_preset_def_gain[idx][i];

          pstr_mae_info->group_presets_definition[idx].disable_pos_interact[i] =
              pstr_input_config->str_asi_config.grp_preset_def_disable_position_interact[idx][i];

          pstr_mae_info->group_presets_definition[idx].position_interact[i] =
              pstr_input_config->str_asi_config.grp_preset_def_position_interact[idx][i];

          pstr_mae_info->group_presets_definition[idx].azimuth_offset[i] =
              pstr_input_config->str_asi_config.grp_preset_def_azimuth_offset[idx][i];

          pstr_mae_info->group_presets_definition[idx].elevation_offset[i] =
              pstr_input_config->str_asi_config.grp_preset_def_elevation_offset[idx][i];

          pstr_mae_info->group_presets_definition[idx].dist_factor[i] =
              pstr_input_config->str_asi_config.grp_preset_def_dist_factor[idx][i];
        }
      }
    }
    if (pstr_mae_info->mae_data.num_data_sets)
    {
      for (idx = 0; idx < pstr_mae_info->mae_data.num_data_sets; idx++)
      {
        WORD32 n = 0;
        ia_description_data *ptr_description_data = &pstr_mae_info->group_desc_data;
        pstr_mae_info->mae_data.data_type[idx] = pstr_input_config->str_asi_config.data_type[idx];
        if (pstr_mae_info->mae_data.data_type[idx] == 0)
        {
          ptr_description_data->num_desc_blocks =
              pstr_input_config->str_asi_config.num_grp_def_decription_blocks;
          if (ptr_description_data->num_desc_blocks < 1)
          {
            return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
          }
          for (n = 0; n < pstr_mae_info->group_desc_data.num_desc_blocks; n++)
          {
            ptr_description_data->group_id[n] =
                pstr_input_config->str_asi_config.grp_def_decription_grp_id[n];

            ptr_description_data->num_descr_languages[n] =
                pstr_input_config->str_asi_config.num_grp_def_decription_languages[n];
            for (WORD32 i = 0; i < ptr_description_data->num_descr_languages[n]; i++)
            {
              ptr_description_data->descr_language[n][i] =
                  pstr_input_config->str_asi_config.grp_def_decription_languages[n][i];
              ptr_description_data->descr_data_length[n][i] =
                  pstr_input_config->str_asi_config.grp_def_decription_data_length[n][i];
              for (WORD32 c = 0; c < ptr_description_data->descr_data_length[n][i]; c++)
              {
                ptr_description_data->descr_data[n][i][c] =
                    pstr_input_config->str_asi_config.grp_def_decription_data[n][i][c];
              }
            }
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 1)
        {
          ptr_description_data = &pstr_mae_info->switch_group_desc_data;

          ptr_description_data->num_desc_blocks =
              pstr_input_config->str_asi_config.num_switch_grp_decription_blocks;
          if (ptr_description_data->num_desc_blocks < 1)
          {
            return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
          }
          for (n = 0; n < ptr_description_data->num_desc_blocks; n++)
          {
            ptr_description_data->group_id[n] =
                pstr_input_config->str_asi_config.switch_grp_decription_grp_id[n];

            ptr_description_data->num_descr_languages[n] =
                pstr_input_config->str_asi_config.switch_grp_num_decription_languages[n];
            for (WORD32 i = 0; i < ptr_description_data->num_descr_languages[n]; i++)
            {
              ptr_description_data->descr_language[n][i] =
                  pstr_input_config->str_asi_config.switch_grp_decription_languages[n][i];
              ptr_description_data->descr_data_length[n][i] =
                  pstr_input_config->str_asi_config.switch_grp_decription_data_length[n][i];
              for (WORD32 c = 0; c < ptr_description_data->descr_data_length[n][i]; c++)
              {
                ptr_description_data->descr_data[n][i][c] =
                    pstr_input_config->str_asi_config.switch_grp_decription_data[n][i][c];
              }
            }
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 5)
        {
          ptr_description_data = &pstr_mae_info->preset_desc_data;

          ptr_description_data->num_desc_blocks =
              pstr_input_config->str_asi_config.num_preset_decription_blocks;
          if (ptr_description_data->num_desc_blocks < 1)
          {
            return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
          }
          for (n = 0; n < pstr_mae_info->group_desc_data.num_desc_blocks; n++)
          {
            ptr_description_data->group_id[n] =
                pstr_input_config->str_asi_config.preset_decription_grp_id[n];

            ptr_description_data->num_descr_languages[n] =
                pstr_input_config->str_asi_config.preset_num_decription_languages[n];
            for (WORD32 i = 0; i < ptr_description_data->num_descr_languages[n]; i++)
            {
              ptr_description_data->descr_language[n][i] =
                  pstr_input_config->str_asi_config.preset_decription_languages[n][i];
              ptr_description_data->descr_data_length[n][i] =
                  pstr_input_config->str_asi_config.preset_decription_data_length[n][i];
              for (WORD32 c = 0; c < ptr_description_data->descr_data_length[n][i]; c++)
              {
                ptr_description_data->descr_data[n][i][c] =
                    pstr_input_config->str_asi_config.preset_decription_data[n][i][c];
              }
            }
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 2)
        {
          pstr_mae_info->content_data.num_data_blocks =
              pstr_input_config->str_asi_config.num_content_data_blocks;
          if (pstr_mae_info->content_data.num_data_blocks < 1)
          {
            return IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG;
          }
          for (n = 0; n < pstr_mae_info->content_data.num_data_blocks; n++)
          {
            pstr_mae_info->content_data.data_group_id[n] =
                pstr_input_config->str_asi_config.content_group_id[n];

            pstr_mae_info->content_data.content_kind[n] =
                pstr_input_config->str_asi_config.content_kind[n];

            pstr_mae_info->content_data.has_content_language[n] =
                pstr_input_config->str_asi_config.has_content_language[n];

            pstr_mae_info->content_data.content_language[n] =
                pstr_input_config->str_asi_config.content_language[n];
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 3)
        {
          pstr_mae_info->composite_pair_data.num_pairs =
              pstr_input_config->str_asi_config.num_comp_pairs;
          for (WORD32 i = 0; i < pstr_mae_info->composite_pair_data.num_pairs; i++)
          {
            pstr_mae_info->composite_pair_data.ele_id[i][0] =
                pstr_input_config->str_asi_config.element_id[2 * i];
            pstr_mae_info->composite_pair_data.ele_id[i][0] =
                pstr_input_config->str_asi_config.element_id[2 * i + 1];
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 4)
        {

          pstr_mae_info->screen_size_data.has_non_std_screen_size =
              pstr_input_config->str_asi_config.has_non_std_screen_size[0];

          pstr_mae_info->screen_size_data.screen_size_az =
              pstr_input_config->str_asi_config.screen_size_az;

          pstr_mae_info->screen_size_data.screen_size_el =
              pstr_input_config->str_asi_config.screen_size_el;

          pstr_mae_info->screen_size_data.screen_size_bot_el =
              pstr_input_config->str_asi_config.screen_size_bot_el[0];
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 7)
        {
          pstr_mae_info->screen_size_ext_data.overwrite_prod_screen_size_data =
              pstr_input_config->str_asi_config.overwrite_prod_screen_size_data;
          pstr_mae_info->screen_size_ext_data.default_screen_sz_left_az =

              pstr_input_config->str_asi_config.default_screen_sz_left_az;
          pstr_mae_info->screen_size_ext_data.default_screen_sz_right_az =
              pstr_input_config->str_asi_config.default_screen_sz_right_az;
          pstr_mae_info->screen_size_ext_data.num_preset_prod_screens =
              pstr_input_config->str_asi_config.num_preset_prod_screens;
          for (WORD32 i = 0; i < pstr_mae_info->screen_size_ext_data.num_preset_prod_screens; i++)
          {
            pstr_mae_info->screen_size_ext_data.screen_grp_preset_id[i] =
                pstr_input_config->str_asi_config.screen_grp_preset_id[i];

            pstr_mae_info->screen_size_ext_data.has_non_std_screen_sz[i] =
                pstr_input_config->str_asi_config.has_non_std_screen_size[i];

            pstr_mae_info->screen_size_ext_data.centered_in_az[i] =
                pstr_input_config->str_asi_config.centered_in_az[i];

            pstr_mae_info->screen_size_ext_data.screen_sz_left_az[i] =
                pstr_input_config->str_asi_config.screen_sz_left_az[i];

            pstr_mae_info->screen_size_ext_data.screen_sz_right_az[i] =
                pstr_input_config->str_asi_config.screen_sz_right_az[i];

            pstr_mae_info->screen_size_ext_data.screen_sz_top_el[i] =
                pstr_input_config->str_asi_config.screen_sz_top_el[i];

            pstr_mae_info->screen_size_ext_data.screen_sz_bot_el[i] =
                pstr_input_config->str_asi_config.screen_size_bot_el[i];
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 8)
        {
          for (n = 0; n < pstr_mae_info->num_group_presets; n++)
          {
            pstr_mae_info->group_presets_definition[n].group_id =
                pstr_input_config->str_asi_config.grp_preset_def_grp_id[n];
            pstr_mae_info->group_presets_definition[n].preset_kind =
                pstr_input_config->str_asi_config.grp_preset_def_preset_kind[n];
            pstr_mae_info->group_presets_definition[n].num_conditions[0] =
                pstr_input_config->str_asi_config.grp_preset_def_num_conditions[n];
            for (WORD32 i = 0; i < pstr_mae_info->group_presets_definition[n].num_conditions[0];
                 i++)
            {
              pstr_mae_info->group_presets_definition[n].reference_id[i] =
                  pstr_input_config->str_asi_config.grp_preset_def_reference_id[n][i];
            }
          }
        }
        if (pstr_mae_info->mae_data.data_type[idx] == 9)
        {
          if (pstr_mae_info->num_group_presets)
          {
            for (i = 0; i < pstr_mae_info->num_groups; i++)
            {
              pstr_mae_info->loud_comp_data.group_loudness[i] =
                  pstr_input_config->str_asi_config.group_loudness[i];
            }
          }
          pstr_mae_info->loud_comp_data.default_params_present =
              pstr_input_config->str_asi_config.default_params_present;
          if (pstr_mae_info->loud_comp_data.default_params_present)
          {
            for (i = 0; i < pstr_mae_info->num_groups; i++)
            {
              pstr_mae_info->loud_comp_data.default_include_group[i] =
                  pstr_input_config->str_asi_config.default_include_group[i];
            }
            pstr_mae_info->loud_comp_data.default_min_max_gain_present =
                pstr_input_config->str_asi_config.default_min_max_gain_present;
            if (pstr_mae_info->loud_comp_data.default_min_max_gain_present)
            {
              pstr_mae_info->loud_comp_data.default_min_gain =
                  pstr_input_config->str_asi_config.default_min_gain;

              pstr_mae_info->loud_comp_data.default_max_gain =
                  pstr_input_config->str_asi_config.default_max_gain;
            }
          }
          for (i = 0; i < pstr_mae_info->num_group_presets; i++)
          {
            pstr_mae_info->loud_comp_data.preset_params_present[i] =
                pstr_input_config->str_asi_config.preset_params_present[i];

            if (pstr_mae_info->loud_comp_data.preset_params_present[i])
            {
              WORD32 j;
              for (j = 0; j < pstr_mae_info->num_groups; j++)
              {
                pstr_mae_info->loud_comp_data.preset_include_group[i][j] =
                    pstr_input_config->str_asi_config.preset_include_group[i][j];
              }
              pstr_mae_info->loud_comp_data.preset_min_max_gain_present[i] =
                  pstr_input_config->str_asi_config.preset_min_max_gain_present[i];

              if (pstr_mae_info->loud_comp_data.preset_min_max_gain_present[i])
              {
                pstr_mae_info->loud_comp_data.preset_min_gain[i] =
                    pstr_input_config->str_asi_config.preset_min_gain[i];

                pstr_mae_info->loud_comp_data.preset_max_gain[i] =
                    pstr_input_config->str_asi_config.preset_max_gain[i];
              }
            }
          }
        }
      }
    }
  }
  p_obj_mpeghe->config.oam_high_rate = pstr_input_config->oam_high_rate;
  p_obj_mpeghe->config.oam_replace_radius = pstr_input_config->oam_replace_radius;
  for (idx = 0; idx < 6; idx++)
  {
    p_obj_mpeghe->config.oam_fixed_values[idx] = pstr_input_config->oam_fixed_values[idx];
  }
  p_obj_mpeghe->config.oam_has_core_length = pstr_input_config->oam_has_core_length;
  p_obj_mpeghe->config.oam_has_scrn_rel_objs = pstr_input_config->oam_has_scrn_rel_objs;
  for (idx = 0; idx < OAM_MAX_NUM_OBJECTS; idx++)
  {
    p_obj_mpeghe->config.oam_is_scrn_rel_obj[idx] = pstr_input_config->oam_is_scrn_rel_obj[idx];
  }
  p_obj_mpeghe->config.oam_data_hndl = pstr_input_config->oam_data_hndl;
  p_obj_mpeghe->config.oam_read_data = pstr_input_config->oam_read_data;
  p_obj_mpeghe->config.oam_skip_data = pstr_input_config->oam_skip_data;
  p_obj_mpeghe->config.oam_version = pstr_input_config->oam_version;
  p_obj_mpeghe->config.has_dyn_obj_priority = pstr_input_config->has_dyn_obj_priority;
  p_obj_mpeghe->config.has_uniform_spread = pstr_input_config->has_uniform_spread;

  if (pstr_input_config->use_drc_element)
  {
    pstr_input_config->str_drc_cfg.str_uni_drc_config.str_channel_layout.base_ch_count =
        pstr_input_config->aud_ch_pcm_cfg.n_channels;
    pstr_input_config->str_drc_cfg.str_enc_params.sample_rate =
        pstr_input_config->aud_ch_pcm_cfg.sample_rate;
    if (pstr_input_config->use_oam_element)
    {
      pstr_input_config->str_drc_cfg.str_uni_drc_config.str_channel_layout.base_ch_count +=
          pstr_input_config->num_oam_ch;
      pstr_input_config->str_drc_cfg.str_enc_params.sample_rate =
          pstr_input_config->aud_obj_pcm_cfg.sample_rate;
    }
    if (pstr_input_config->use_hoa_element)
    {
      pstr_input_config->str_drc_cfg.str_uni_drc_config.str_channel_layout.base_ch_count +=
          pstr_input_config->num_trans_ch;
      pstr_input_config->str_drc_cfg.str_enc_params.sample_rate =
          pstr_input_config->hoa_pcm_cfg.sample_rate;
    }
    pstr_input_config->str_drc_cfg.str_uni_drc_config.sample_rate =
        pstr_input_config->str_drc_cfg.str_enc_params.sample_rate;
    for (i = 0;
         i < pstr_input_config->str_drc_cfg.str_uni_drc_config.drc_coefficients_uni_drc_count;
         i++)
    {
      for (j = 0;
           j < pstr_input_config->str_drc_cfg.str_uni_drc_config.str_drc_coefficients_uni_drc[i]
                   .gain_set_count;
           j++)
      {
        pstr_input_config->str_drc_cfg.str_uni_drc_config.str_drc_coefficients_uni_drc[i]
            .str_gain_set_params[j]
            .delta_tmin = impeghe_drc_get_delta_t_min(
            pstr_input_config->str_drc_cfg.str_uni_drc_config.sample_rate);
      }
    }

    if (pstr_input_config->use_downmix_ext_config)
    {
      ia_drc_uni_drc_config_struct *pstr_uni_drc_config =
          &pstr_input_config->str_drc_cfg.str_uni_drc_config;
      ia_mpeghe_ext_cfg_downmix_input_struct *pstr_ext_cfg_downmix_input =
          &pstr_input_config->str_ext_cfg_downmix_input;
      pstr_uni_drc_config->downmix_instructions_count =
          pstr_ext_cfg_downmix_input->downmix_id_count;
      for (i = 0; i < pstr_uni_drc_config->downmix_instructions_count; i++)
      {
        pstr_uni_drc_config->str_downmix_instructions[i].downmix_id =
            pstr_ext_cfg_downmix_input->str_dmx_matrix[i].dmx_id;
        pstr_uni_drc_config->str_downmix_instructions[i].target_ch_count =
            pstr_ext_cfg_downmix_input->str_dmx_matrix[i].cicp_spk_layout_idx;
      }
    }
  }

  // DRC Params
  p_obj_mpeghe->config.use_drc_element = pstr_input_config->use_drc_element;
  if (p_obj_mpeghe->config.use_drc_element != 0)
  {
    p_obj_mpeghe->config.str_drc_cfg = pstr_input_config->str_drc_cfg;
  }

  // Downmix Params
  p_obj_mpeghe->config.use_downmix_ext_config = pstr_input_config->use_downmix_ext_config;
  if (p_obj_mpeghe->config.use_downmix_ext_config != 0)
  {
    p_obj_mpeghe->config.str_ext_cfg_downmix_input = pstr_input_config->str_ext_cfg_downmix_input;
  }

  if ((pstr_input_config->use_oam_element) || (pstr_input_config->use_hoa_element))
  {
    p_obj_mpeghe->config.basic_bitrate =
        p_obj_mpeghe->config.aud_ch * MINIMUM_BITRATE_PER_CHANNEL;
    if ((p_obj_mpeghe->config.oam_bitrate + p_obj_mpeghe->config.basic_bitrate) >
        p_obj_mpeghe->config.bit_rate)
    {
      return IMPEGHE_CONFIG_FATAL_BITRATE;
    }
  }
  else
  {
    p_obj_mpeghe->config.basic_bitrate = p_obj_mpeghe->config.bit_rate;
  }

  p_obj_mpeghe->config.use_hoa = pstr_input_config->use_hoa_element;
  p_obj_mpeghe->config.hoa_config.hoa_order = pstr_input_config->hoa_order;
  p_obj_mpeghe->config.hoa_config.uses_nfc = pstr_input_config->uses_nfc;
  p_obj_mpeghe->config.hoa_config.nfc_distance = pstr_input_config->nfc_distance;
  p_obj_mpeghe->config.hoa_config.num_hoa_coeffs = pstr_input_config->num_hoa_coeffs;
  p_obj_mpeghe->config.hoa_config.use_vec_est = pstr_input_config->use_vec_est;

  if (1 == p_obj_mpeghe->config.use_hoa)
  {
    p_obj_mpeghe->config.hoa_bitrate = p_obj_mpeghe->config.bit_rate -
                                       p_obj_mpeghe->config.oam_bitrate -
                                       p_obj_mpeghe->config.basic_bitrate;
    if (p_obj_mpeghe->config.hoa_bitrate < MINIMUM_BITRATE_PER_CHANNEL)
    {
      return IMPEGHE_CONFIG_FATAL_BITRATE;
    }

    // HOA matrix related
    p_obj_mpeghe->config.use_hoa_matrix = pstr_input_config->use_hoa_matrix;
    if (p_obj_mpeghe->config.use_hoa_matrix)
    {
      p_obj_mpeghe->config.hoa_config.num_hoa_matrix = pstr_input_config->num_hoa_matrix;
      for (idx = 0; idx < p_obj_mpeghe->config.hoa_config.num_hoa_matrix; idx++)
      {
        p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_rend_id =
            pstr_input_config->hoa_rend_id[idx];
        p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_cicp =
            pstr_input_config->hoa_cicp[idx];
        p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix_in_dim =
            pstr_input_config->hoa_matrix_in_dim[idx];
        p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix_out_dim =
            pstr_input_config->hoa_matrix_out_dim[idx];
        memcpy(p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix,
               pstr_input_config->hoa_matrix[idx],
               sizeof(FLOAT64) * pstr_input_config->hoa_matrix_in_dim[idx] *
                   pstr_input_config->hoa_matrix_out_dim[idx]);
      }
    }
  }
  // If invalid value is set, default to direction estimation
  if (p_obj_mpeghe->config.hoa_config.use_vec_est > 1)
    p_obj_mpeghe->config.hoa_config.use_vec_est = 0;

  /* Set Profile and level based on HOA order */
  if ((p_obj_mpeghe->config.hoa_config.hoa_order > 2) &&
      p_obj_mpeghe->config.prof_level == PROFILE_LC_LVL1)
    p_obj_mpeghe->config.prof_level = PROFILE_LC_LVL2;
  if ((p_obj_mpeghe->config.hoa_config.hoa_order > 4) &&
      p_obj_mpeghe->config.prof_level == PROFILE_LC_LVL2)
    p_obj_mpeghe->config.prof_level = PROFILE_LC_LVL3;

  return error;
}

/**
 *  impeghe_set_default_config
 *
 *  \brief Set default configurations
 *
 *  \param [in,out]	p_obj_mpeghe	Pointer to API structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_set_default_config(ia_mpeghe_api_struct *p_obj_mpeghe)
{
  LOOPIDX idx;

  p_obj_mpeghe->config.ui_pcm_wd_sz = 16;
  memset(&p_obj_mpeghe->config.audio_specific_config, 0,
         sizeof(p_obj_mpeghe->config.audio_specific_config));

  memset(&p_obj_mpeghe->config, 0, sizeof(ia_usac_encoder_config_struct));
  p_obj_mpeghe->config.sampling_rate = 44100;
  p_obj_mpeghe->config.native_sampling_rate = 44100;
  p_obj_mpeghe->config.codec_mode = USAC_ONLY_FD;
  p_obj_mpeghe->config.bit_rate = 32000;
  for (idx = 0; idx < USAC_MAX_ELEMENTS; idx++)
  {
    p_obj_mpeghe->config.bw_limit[idx] = 0;
  }
  p_obj_mpeghe->config.tns_select = 1;
  p_obj_mpeghe->config.flag_noiseFilling = 0;
  p_obj_mpeghe->config.cmplx_pred_flag = 0;
  p_obj_mpeghe->config.enhanced_noise_filling = 1;
  p_obj_mpeghe->config.igf_after_tns_synth = 1;
  p_obj_mpeghe->config.prev_aliasing_symmetry = 0;
  p_obj_mpeghe->config.curr_aliasing_symmetry = 0;
  p_obj_mpeghe->config.window_shape_prev[0] = WIN_SEL_1;
  p_obj_mpeghe->config.use_fill_element = 1;
  p_obj_mpeghe->config.prof_level = PROFILE_LC_LVL1;
  p_obj_mpeghe->config.crc16 = 0;
  p_obj_mpeghe->config.crc32 = 0;
  p_obj_mpeghe->config.global_crc16 = 0;
  p_obj_mpeghe->config.global_crc32 = 0;
  p_obj_mpeghe->config.mct_mode = -1;

  // OAM Params
  p_obj_mpeghe->config.cicp_index = 0;
  p_obj_mpeghe->config.use_oam_element = 0;
  p_obj_mpeghe->config.oam_high_rate = 1;
  p_obj_mpeghe->config.oam_replace_radius = 0;
  for (idx = 0; idx < 6; idx++)
  {
    p_obj_mpeghe->config.oam_fixed_values[idx] = 0;
  }
  p_obj_mpeghe->config.oam_has_core_length = 0;
  p_obj_mpeghe->config.oam_has_scrn_rel_objs = 0;
  for (idx = 0; idx < OAM_MAX_NUM_OBJECTS; idx++)
  {
    p_obj_mpeghe->config.oam_is_scrn_rel_obj[idx] = 0;
  }
  p_obj_mpeghe->config.oam_data_hndl = 0;
  p_obj_mpeghe->config.oam_read_data = 0;
  p_obj_mpeghe->config.oam_skip_data = 0;

  // DRC Params
  p_obj_mpeghe->config.use_drc_element = 0;
  memset(&p_obj_mpeghe->config.str_drc_cfg, 0, sizeof(ia_drc_input_config));

  // Downmix Params
  p_obj_mpeghe->config.use_downmix_ext_config = 0;
  memset(&p_obj_mpeghe->config.str_ext_cfg_downmix_input, 0,
         sizeof(p_obj_mpeghe->config.str_ext_cfg_downmix_input));

  p_obj_mpeghe->config.hoa_config.hoa_order = 0;
  p_obj_mpeghe->config.hoa_config.uses_nfc = 0;
  p_obj_mpeghe->config.hoa_config.nfc_distance = 0;
  p_obj_mpeghe->config.hoa_config.use_vec_est = 0;
  p_obj_mpeghe->config.hoa_config.hoa_mtx_status = 0;

  // HOA matrix related
  p_obj_mpeghe->config.hoa_config.num_hoa_matrix = 0;
  for (idx = 0; idx < MAX_NUM_RENDERER_MATRIX; idx++)
  {
    p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_rend_id = -1;
    p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_cicp = -1;
    p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix_in_dim = 0;
    p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix_out_dim = 0;
    memset(p_obj_mpeghe->config.hoa_config.hoa_mat_cfg[idx].hoa_matrix, 0,
           sizeof(FLOAT64) * (MAX_NUM_HOA_COEFFS * MAX_NUM_PERC_CODERS));
  }

  p_obj_mpeghe->config.in_frame_length = FRAME_LEN_LONG;
  p_obj_mpeghe->config.cc_resamp_fac_down = 1;
  p_obj_mpeghe->config.cc_resamp_fac_up = 1;
  return;
}

/**
 *  impeghe_alloc_and_assign_mem
 *
 *  \brief Allocates the memory using callback function and assigns corresponding pointers
 *
 *  \param [in]		p_obj_mpeghe	Pointer to API structure
 *  \param [out]	ptr_out_cfg		Pointer to structure that maintains output
 * configurations
 *
 *  \return VOID
 *
 */
static VOID impeghe_alloc_and_assign_mem(ia_mpeghe_api_struct *p_obj_mpeghe,
                                         ia_output_config *ptr_out_cfg)
{
  UWORD32 i_idx;
  pVOID pv_value;
  ptr_out_cfg->in_frame_length = FRAME_LEN_LONG;

  for (i_idx = 0; i_idx < IMPEGHE_NUM_MEMTABS; i_idx++)
  {
    UWORD32 *meminfo = (UWORD32 *)(p_obj_mpeghe->p_mem_info_mpeghe + i_idx);

    ptr_out_cfg->mem_info_table[i_idx].ui_size =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_SIZE - IA_API_CMD_GET_MEM_INFO_SIZE));
    ptr_out_cfg->mem_info_table[i_idx].ui_alignment =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_ALIGNMENT - IA_API_CMD_GET_MEM_INFO_SIZE));
    ptr_out_cfg->mem_info_table[i_idx].ui_type =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_TYPE - IA_API_CMD_GET_MEM_INFO_SIZE));
    if ((p_obj_mpeghe->config.codec_mode != USAC_ONLY_FD) && (i_idx == IA_MEMTYPE_INPUT))
    {
      switch (p_obj_mpeghe->config.sampling_rate)
      {
      case 44100:
        ptr_out_cfg->mem_info_table[i_idx].ui_size *= RESAMPLE_RATIO_3;
        ptr_out_cfg->mem_info_table[i_idx].ui_size =
            (ptr_out_cfg->mem_info_table[i_idx].ui_size >> 1);
        p_obj_mpeghe->config.sampling_rate = 29400;
        ptr_out_cfg->in_frame_length = (FRAME_LEN_LONG * RESAMPLE_RATIO_3) >> 1;
        p_obj_mpeghe->config.cc_resamp_fac_down = RESAMPLE_RATIO_3;
        p_obj_mpeghe->config.cc_resamp_fac_up = RESAMPLE_RATIO_2;
        break;
      case 48000:
        ptr_out_cfg->mem_info_table[i_idx].ui_size *= RESAMPLE_RATIO_3;
        ptr_out_cfg->mem_info_table[i_idx].ui_size =
            (ptr_out_cfg->mem_info_table[i_idx].ui_size >> 1);
        p_obj_mpeghe->config.sampling_rate = 32000;
        ptr_out_cfg->in_frame_length = (FRAME_LEN_LONG * RESAMPLE_RATIO_3) >> 1;
        p_obj_mpeghe->config.cc_resamp_fac_down = RESAMPLE_RATIO_3;
        p_obj_mpeghe->config.cc_resamp_fac_up = RESAMPLE_RATIO_2;
        break;
      }
    }

    ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] =
        ptr_out_cfg->malloc_xaac(ptr_out_cfg->mem_info_table[i_idx].ui_size,
                                 ptr_out_cfg->mem_info_table[i_idx].ui_alignment);

    ptr_out_cfg->ui_rem =
        (UWORD32)((SIZE_T)ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] %
                  ptr_out_cfg->mem_info_table[i_idx].ui_alignment);

    pv_value = (pVOID)((WORD8 *)ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] + 4 -
                       ptr_out_cfg->ui_rem);

    pv_value = ptr_out_cfg->mem_info_table[i_idx].mem_ptr =
        (pVOID)((WORD8 *)ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] +
                ptr_out_cfg->mem_info_table[i_idx].ui_alignment - ptr_out_cfg->ui_rem);

    p_obj_mpeghe->pp_mem[i_idx] = ptr_out_cfg->mem_info_table[i_idx].mem_ptr;
    memset(p_obj_mpeghe->pp_mem[i_idx], 0, p_obj_mpeghe->p_mem_info_mpeghe[i_idx].ui_size);

    if (i_idx == IA_MEMTYPE_PERSIST)
    {
      WORD32 offset_size = sizeof(ia_usac_enc_state_struct), i;
      WORD8 *p_offset = NULL, *p_temp;

      p_obj_mpeghe->p_state_mpeghe = (ia_usac_enc_state_struct *)pv_value;
      memset(p_obj_mpeghe->p_state_mpeghe, 0, sizeof(ia_usac_enc_state_struct));
      ia_usac_encoder_config_struct *pstr_usac_config = &p_obj_mpeghe->config;

      ia_usac_enc_state_struct *p_state = p_obj_mpeghe->p_state_mpeghe;
      ia_usac_data_struct *str_usac_enc_data = &(p_state->str_usac_enc_data);

      p_state->ptr_in_buf = (FLOAT32 **)((WORD8 *)p_state + offset_size);
      ;

      p_offset = (WORD8 *)p_state->ptr_in_buf + (pstr_usac_config->channels * sizeof(FLOAT32 *));
      p_temp = p_offset;

      for (i = 0; i < p_obj_mpeghe->config.channels; i++)
      {
        p_state->ptr_in_buf[i] = (FLOAT32 *)p_offset;
        p_offset += pstr_usac_config->ccfl * sizeof(FLOAT32) * 2;
      }
      memset(p_temp, 0, (p_offset - p_temp));

      p_temp = p_offset;
      for (i = 0; i < pstr_usac_config->channels; i++)
      {
        str_usac_enc_data->ptr_time_data[i] = (FLOAT64 *)p_offset;
        p_offset += 2 * (pstr_usac_config->ccfl) * sizeof(FLOAT64);
      }

      for (i = 0; i < pstr_usac_config->channels; i++)
      {
        str_usac_enc_data->ptr_look_ahead_time_data[i] = (FLOAT64 *)p_offset;
        p_offset += pstr_usac_config->ccfl * sizeof(FLOAT64);
      }

      for (i = 0; i < pstr_usac_config->channels; i++)
      {
        str_usac_enc_data->spectral_line_vector[i] = (FLOAT64 *)p_offset;
        p_offset += ((pstr_usac_config->ccfl + 128) * 2) * sizeof(FLOAT64);
      }

      for (i = 0; i < pstr_usac_config->channels; i++)
      {
        str_usac_enc_data->ptr_2frame_time_data[i] = (FLOAT64 *)p_offset;
        p_offset += 3 * pstr_usac_config->ccfl * sizeof(FLOAT64);
      }

      memset(p_temp, 0, p_offset - p_temp);

      if (pstr_usac_config->tns_select != 0)
      {
        p_temp = p_offset;
        for (i = 0; i < pstr_usac_config->channels; i++)
        {
          str_usac_enc_data->pstr_tns_info[i] = (ia_tns_info *)p_offset;
          p_offset += sizeof(ia_tns_info);
        }
        memset(p_temp, 0, p_offset - p_temp);
      }

      p_temp = p_offset;
      for (i = 0; i < pstr_usac_config->channels; i++)
      {
        str_usac_enc_data->td_encoder[i] = (ia_usac_td_encoder_struct *)p_offset;
        p_offset += sizeof(ia_usac_td_encoder_struct);
      }
      memset(p_temp, 0, p_offset - p_temp);
    }

    if (i_idx == IA_MEMTYPE_SCRATCH)
    {
      p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_scratch.ptr_scratch_buf =
          (UWORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_SCRATCH];
    }

    if (i_idx == IA_MEMTYPE_INPUT)
    {
      ptr_out_cfg->ui_inp_buf_size = ptr_out_cfg->mem_info_table[i_idx].ui_size;
    }
    ptr_out_cfg->malloc_count++;
  }
  p_obj_mpeghe->config.in_frame_length = ptr_out_cfg->in_frame_length;
}

/**
 *  impeghe_get_lib_id_strings
 *
 *  \brief Get library identification strings
 *
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return VOID
 *
 */
VOID impeghe_get_lib_id_strings(pVOID pv_output)
{
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;

  /*Update library name and version number*/
  pstr_output_config->p_lib_name = (WORD8 *)LIB_NAME;
  pstr_output_config->p_version_num = (WORD8 *)MPEGH_LC_ENC_VER;

  return;
}

/**
 *  impeghe_create
 *
 *  \brief Create encoder object
 *
 *  \details
 *  Following are details about input config structure
 *  elements, default value and valid range
 *  cplx_pred: Flag that controls usage of complex prediction in encoding.
 *             Valid values: 0, 1. 0 – Disable. 1 – Enable. Default value: 0
 *  coding_mode: //Not used
 *  fill_elem: Flag that controls usage of fill elements in encoding.
 *             Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 1
 *  iframe_interval: //Not used
 *  sample_rate: Sampling frequency of the input.
 *               Valid values: 14700, 16000, 22050, 24000, 29400, 32000, 44100 and 48000
 *  aud_ch_pcm_cfg: Structure containing information about audio channel PCM configuration like
 *                  channel-mask, length, number of channels, sample rate and pcm-size
 *  aud_obj_pcm_cfg: Structure containing information about audio object PCM configuration like
 *                  channel-mask, length, number of channels, sample rate and pcm-size
 *  hoa_pcm_cfg: Structure containing information about HOA PCM configuration like channel-mask,
 *               length, number of channels, sample rate and pcm-size
 *  num_trans_ch: Number of HOA transport channels. //Not used
 *  num_oam_ch: Sum of number of channels and number of objects being given as input. This
 *              value would be read from input .OAM file
 *  codec_mode: Codec mode to encode. Valid value range: 0 - 2. 0 – Switched Mode. 1 – FD mode.
 *              2 – TD mode. Default value: 1 (FD mode)
 *  out_fmt: Output format of encoded stream.
 *           Valid value range: 1 – 3. 1 – Raw MHAS, 2 – MP4(MHA1), 3 – MP4 (MHM1)
 *           Default value: 1
 *  mhas_pkt: Flag indicating if the encoded output stream is of MHAS packet type.
 *            Valid values: 0, 1. Default value: 1 if out_fmt is Raw MHAS or MP4 (MHM1), 0 if
 *            out_fmt is MP4 (MHA1)
 *  crc16: Flag that controls the addition of CRC16 packet in encoding.
 *         Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  crc32: Flag that controls the addition of CRC32 packet in encoding. Valid values: 0, 1.
 *         0 – Disable, 1 – Enable. Default value: 0
 *  global_crc16: Flag that controls the addition of global CRC16 packet in encoding.
 *                Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  global_crc32: Flag that controls the addition of global CRC32 packet in encoding.
 *                Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  Bitrate: Bitrate set in bits per second.
 *           Range: 32000 to (6 * Sample rate * Number of channels).
 *           Default value: 32000 for mono (1-channel), 64000 for stereo (2-channel),
 *           192000 for 6-channel, 256000 for 8-channel, 320000 for 10-channel
 *  ltpf_enable: Flag that controls the usage of LTPF in encoding. Valid values: 0, 1.
 *  0 – Disable, 1 – Enable. Default value: 0
 *  num_ch_sig_groups: Number of channel signal groups. //Not used
 *  num_ch_per_sig_grp[16]: Number of channels per signal group. //Not used
 *  num_ch_idx_per_grp[56]: Number of channels per group. //Not used
 *  num_obj_sig_groups: Number of object signal groups. //Not used
 *  num_objs_per_sig_grp[16]: Number of objects per signal group.. //Not used
 *  num_hoa_sig_groups: Number of HOA signal groups. //Not used
 *  num_hoas_per_sig_grp[16]: Number of HOA per signal group. //Not used
 *  enhanced_noise_filling: Flag that controls usage of IGF in encoding.
 *                          Valid values: 0, 1. 0 – Disable, 1 – Enable.
 *                          Default value: 1
 *  igf_start_freq: IGF start frequency. Valid range: 0 to 20000. Default value: Based on sample
 *                  rate and bit rate, library assigns default value
 *  igf_stop_freq: IGF stop frequency. Valid range: 0 to 20000. Default value: Based on sample
 *                 rate and bit rate, library assigns default value
 *  igf_start_freq_flag: Flag for IGF start frequency set when out of range value is provided.
 *                      //Not used
 *  igf_stop_freq_flag: Flag for IGF stop frequency set when out of range values is provided.
 *                      //Not used
 *  igf_after_tns_synth: Flag that controls the usage of IGF after TNS in encoding.
 *                       Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 1
 *  fdp_enable: Flag that controls the usage of FDP in encoding. Valid values: 0, 1.
 *              0 – Disable, 1 – Enable. Default value: 0
 *  noise_filling: Flag that controls the usage of noise filling in encoding.
 *                 Valid values: 0, 1. 0 – Disable, 1 – Enable, Default value: 0
 *  tns_enable: Flag that controls the usage of TNS in encoding. Valid values: 0, 1.
 *              0 – Disable, 1 – Enable. Default value: 1
 *  prof_level: Defines the profile and level in encoding. Range: 0 – 6.
 *              0 – Low Complexity Level 1, 1 - Low Complexity Level 2,
 *              2 - Low Complexity Level 3, 3 - Low Complexity Level 4,
 *              4 - Baseline Level 1, 5 - Baseline Level 2 , 6 - Baseline Level 3.
 *              Default value: 0. Value gets updated based on input properties
 *  prof_level_flag: Flag that is set when profile and level data is provided. //Not used
 *  kernel: Controls aliasing symmetry values. Valid value range: 0-3. Default value: 0
 *  prev_aliasing_symmetry: Previous aliasing symmetry value. Valid value range: 0-3.
 *                          Default value: 0
 *  curr_aliasing_symmetry: Current aliasing symmetry values. Valid value range: 0-3.
 *                          Default value: 0
 *  full_band_lpd: Flag that controls usage of fullband LPD in encoding. Valid values: 0, 1.
 *                 0 – Disable, 1 – Enable. Default value: 0
 *  stereo_lpd: Flag that controls usage of stereo LPD in encoding. Valid values: 0, 1.
 *              0 – Disable, 1 – Enable. Default value: 0
 *  mct_mode: Multichannel coding tool mode. Valid values: -1, 0, 1, 2, 3.
 *           -1 – Disable, 0 – Prediction, 1 – Rotation, 2 – Prediction + Stereo Filling,
 *           3 – Rotation + Stereo Filling. Default value: -1
 *  num_objects: Number of OAM objects. Valid value range: 0 - 24
 *  extra_objects: Number of extra OAM objects, if OAM objects exceeds limit of 24.
 *  num_channels: Number of OAM channels. Valid value range: 0 - 24
 *  oam_version: OAM version number. Vaid value range: 1 - 4
 *  has_dyn_obj_priority: Flag that is set if OAM dynamic object priority is present.
 *                        Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  has_uniform_spread: Flag that is set if OAM uniform spread is present. Valid values: 0, 1.
 *  0 – Disable, 1 – Enable. Default value: 1
 *  use_oam_element: Flag that is set when OAM channels and/or objects are present.
 *                   Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  user_specified_sig_grp: //Not used
 *  oam_high_rate: Flag that is set when OAM rate is set high. Valid values: 0, 1.
 *                 0 – Disable, 1 – Enable. Default value: 1
 *  cicp_index: Channel configuration index. Valid range value – 1 - 20 except 8.
 *              Default: Default value is assigned based on number of input channels
 *  oam_replace_radius: OAM replace radius. Default value: 0
 *  oam_fixed_values[6]: Array containing OAM fixed values. Default value: 0
 *  oam_has_core_length: Flag that is set when OAM has core length. Valid values: 0, 1.
 *                       0 – Disable, 1 – Enable. Default value: 0
 *  oam_has_scrn_rel_objs: Flag that is set when OAM has screen related objects.
 *                         Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  oam_is_scrn_rel_obj[24]: Array containing OAM screen related objects. Default value: 0
 *  oam_data_hndl: Pointer to OAM data handle. Default value: NULL
 *  oam_read_data: Pointer to read OAM data. Default value: NULL
 *  oam_skip_data: Pointer to skip OAM data. Default value: NULL
 *  item_prefix[64]: Array containing OAM item prefix values. //Not used
 *  err_code: Error code. Default: 0
 *  asi_enable: Flag that controls the usage of ASI in encoding. //Not used
 *  str_asi_config: Structure containing information about ASI configuration. //Not used
 *  flexi_spk_enable: Flag that controls the usage of flexible speaker configuration in encoding.
 *                    //Not used
 *  str_flexi_spk_config: Structure containing information about flexible speaker configuration.
 *                        //Not used
 *  use_hoa_element: Flag that is set when HOA data is present. Valid values: 0, 1.
 *                   0 – Disable, 1 – Enable. Default value: 0
 *  hoa_order: HOA order. Valid value range: 1 - 6
 *  uses_nfc: Flag that is set when HOA NFC data is present. //Not used
 *  nfc_distance: HOA NFC distance. Default value: 0
 *  num_hoa_coeffs: Number of HOA coefficients. Default value: No default value, depends on input
 *  use_vec_est: Flag that controls the usage of HOA vector estimation in encoding.
 *               Valid values: 0, 1. 0 – Disable, 1 – Enable. Default value: 0
 *  use_hoa_matrix: Flag that controls the usage of HOA matrix in encoding. //Not used
 *  num_hoa_matrix: Number of HOA matrices. //Not used
 *  hoa_rend_id[6]: Array containing different HOA renderer IDs. //Not used
 *  hoa_cicp[6]: Array containing different HOA channel configuration indices. //Not used
 *  hoa_matrix_in_dim[6]: Array containing HOA matrix input dimensions. //Not used
 *  hoa_matrix_out_dim[6]: Array containing HOA matrix output dimensions. //Not used
 *  hoa_matrix[6][49*24]: Two dimensional HOA matrix. //Not used
 *  use_drc_element: Flag that is set when DRC is enabled. Valid values: 0, 1.
 *  0 – Disable, 1 – Enable. Default value: 0
 *  str_drc_cfg Structure: containing information about DRC input configuration. //Not used
 *  str_drc_user_options: Structure containing information about DRC user options. //Not used
 *  use_downmix_ext_config: Flag that is set when downmix configuration is present. //Not used
 *  str_ext_cfg_downmix_input: Structure containing information about downmix extension
 *                             configuration. //Not used
 *  str_ec_info_struct: Structure containing information about Earcon data. //Not used
 *
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Error code
 *
 */
IA_ERRORCODE impeghe_create(pVOID pv_input, pVOID pv_output)
{
  pVOID pv_value;
  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;
  WORD32 ui_api_size = sizeof(ia_mpeghe_api_struct);

  pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] =
      pstr_output_config->malloc_xaac(ui_api_size, 4);

  /*Update library name and version number*/
  pstr_output_config->p_lib_name = (WORD8 *)LIB_NAME;
  pstr_output_config->p_version_num = (WORD8 *)MPEGH_LC_ENC_VER;
  // HOA the number of channels if input is HOA. HOA input files are mono, but the output number
  // of channels depend on bitrate.
  // The output of HOA serves as input to core coder and this value is needed for memory
  // allocation.

  if (1 == pstr_input_config->use_hoa_element)
  {
    pstr_input_config->num_trans_ch = impeghe_hoa_calc_num_coders(pstr_input_config->bitrate, 0);

    if (pstr_input_config->num_trans_ch >
        ((pstr_input_config->hoa_order + 1) * (pstr_input_config->hoa_order + 1)))
      pstr_input_config->num_trans_ch =
          (pstr_input_config->hoa_order + 1) * (pstr_input_config->hoa_order + 1);

    // HOA is always in FD_MODE. Need to check if this is needed.
    pstr_input_config->codec_mode = USAC_ONLY_FD;
  }

  if (1 == pstr_input_config->use_oam_element)
  {
    // OAM is always in FD mode.
    pstr_input_config->codec_mode = USAC_ONLY_FD;
  }

  /*Error*/
  pstr_output_config->ui_rem = (UWORD32)(
      (SIZE_T)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] & 3);

  pstr_output_config->pv_ia_process_api_obj =
      (pVOID)((WORD8 *)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] +
              4 - pstr_output_config->ui_rem);
  pstr_output_config->malloc_count++;

  ia_mpeghe_api_struct *p_obj_mpeghe =
      (ia_mpeghe_api_struct *)pstr_output_config->pv_ia_process_api_obj;

  memset(p_obj_mpeghe, 0, sizeof(*p_obj_mpeghe));

  impeghe_set_default_config(p_obj_mpeghe);

  err_code = impeghe_set_config_params(p_obj_mpeghe, pstr_input_config);
  if (((UWORD32)err_code & 0x8000) >> 15)
    return err_code;

  pstr_output_config->i_num_chan = p_obj_mpeghe->config.channels;
  pstr_output_config->ui_proc_mem_tabs_size =
      (sizeof(ia_mem_info_struct) + sizeof(pVOID *)) * (IMPEGHE_NUM_MEMTABS);

  pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] =
      pstr_output_config->malloc_xaac(pstr_output_config->ui_proc_mem_tabs_size, 4);

  pstr_output_config->ui_rem = (UWORD32)(
      (SIZE_T)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] & 3);

  pv_value =
      (pVOID)((WORD8 *)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] +
              4 - pstr_output_config->ui_rem);

  if (pv_value == NULL)
    return IMPEGHE_API_FATAL_MEM_ALLOC;
  memset(pv_value, 0, (sizeof(ia_mem_info_struct) + sizeof(pVOID *)) * (IMPEGHE_NUM_MEMTABS));

  p_obj_mpeghe->p_mem_info_mpeghe = pv_value;
  p_obj_mpeghe->pp_mem =
      (pVOID *)((WORD8 *)pv_value + sizeof(ia_mem_info_struct) * IMPEGHE_NUM_MEMTABS);

  pstr_output_config->malloc_count++;

  p_obj_mpeghe->config.ccfl = 1024;
  impeghe_enc_fill_mem_tables(p_obj_mpeghe);
  impeghe_alloc_and_assign_mem(p_obj_mpeghe, pstr_output_config);
  if ((pstr_input_config->bitrate >=
       (((MAX_CHANNEL_BITS / FRAME_LEN_LONG) * p_obj_mpeghe->config.sampling_rate *
         p_obj_mpeghe->config.channels))))
  {
    return (IMPEGHE_CONFIG_FATAL_BITRATE);
  }
  if (!(pstr_input_config->aud_ch_pcm_cfg.pcm_sz == 16 ||
        pstr_input_config->aud_ch_pcm_cfg.pcm_sz == 24 ||
        pstr_input_config->aud_ch_pcm_cfg.pcm_sz == 32))
  {
    return IMPEGHE_CONFIG_FATAL_PCM_SIZE;
  }
  return err_code;
}

/**
 *  impeghe_init
 *
 *  \brief Initialize the encoder object
 *
 *  \param [in,out] p_ia_mpeghe_obj  Pointer to the encoder object
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Error code
 *
 */
IA_ERRORCODE impeghe_init(pVOID p_ia_mpeghe_obj, pVOID pv_input, pVOID pv_output)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  ia_mpeghe_api_struct *p_obj_mpeghe = (ia_mpeghe_api_struct *)p_ia_mpeghe_obj;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;

  jmp_buf api_init_jmp_buf;
  err_code = setjmp(api_init_jmp_buf);
  if (err_code != IA_NO_ERROR)
  {
    return IMPEGHE_INIT_FATAL_INSUFFICIENT_WRITE_BUFFER_SIZE;
  }
  p_obj_mpeghe->p_state_mpeghe->impegh_jmp_buf = &api_init_jmp_buf;

  /* Set config pointer in api obj */
  p_obj_mpeghe->p_state_mpeghe->p_config = &p_obj_mpeghe->config;
  err_code = impeghe_initialize(p_obj_mpeghe);
  if (err_code)
    return err_code;

  if ((p_obj_mpeghe->config.use_hoa) && (p_obj_mpeghe->config.use_hoa_matrix))
  {
    // Update hoa matrix status
    pstr_output_config->hoa_mtx_status = p_obj_mpeghe->config.hoa_config.hoa_mtx_status;
  }

  pstr_input_config->igf_start_freq = p_obj_mpeghe->config.igf_start_freq;
  pstr_input_config->igf_stop_freq = p_obj_mpeghe->config.igf_stop_freq;
  pstr_input_config->enhanced_noise_filling = p_obj_mpeghe->config.enhanced_noise_filling;
  pstr_input_config->prof_level = p_obj_mpeghe->config.prof_level;

  p_obj_mpeghe->p_state_mpeghe->ui_bytes_consumed = 0;

  p_obj_mpeghe->p_state_mpeghe->ui_init_done = 1;

  if (p_obj_mpeghe->config.mhas_pkt == 1)
  {
    p_obj_mpeghe->config.ptr_mhas_bit_buf = &p_obj_mpeghe->config.mhas_bit_buf;
    p_obj_mpeghe->config.ptr_mhas_bit_buf =
        impeghe_create_bit_buffer(p_obj_mpeghe->config.ptr_mhas_bit_buf,
                                  &p_obj_mpeghe->config.mhas_buf[0], MAX_MHAS_PKT_HDR_SZ);
    p_obj_mpeghe->config.ptr_mhas_bit_buf->impeghe_jmp_buf = &api_init_jmp_buf;
  }

  pstr_output_config->profile_info = p_obj_mpeghe->config.audio_specific_config.profile_info;

  UWORD32 num_bits = 0;
  ia_bit_buf_struct *ptr_asc_bit_buf;
  ptr_asc_bit_buf = impeghe_create_bit_buffer(
      &(p_obj_mpeghe->p_state_mpeghe->str_bit_buf), p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT],
      p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT].ui_size);
  ptr_asc_bit_buf->impeghe_jmp_buf = &api_init_jmp_buf;

  if (p_obj_mpeghe->config.mhas_pkt == 1)
  {
    UWORD32 crc_val;
    WORD32 crc_len = 16;
    ia_bit_buf_struct str_temp_bit_buf;
    str_temp_bit_buf.ptr_bit_buf_base = (UWORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT];
    str_temp_bit_buf.ptr_bit_buf_end =
        str_temp_bit_buf.ptr_bit_buf_base +
        p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT].ui_size - 1;
    str_temp_bit_buf.size = (p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT].ui_size << 3);
    str_temp_bit_buf.cnt_bits = 0;
    impeghe_reset_bit_buffer(&str_temp_bit_buf);
    str_temp_bit_buf.impeghe_jmp_buf = &api_init_jmp_buf;
    num_bits = impeghe_get_audiospecific_config_bytes(
        &str_temp_bit_buf, &p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_scratch,
        &p_obj_mpeghe->config.audio_specific_config);

    impeghe_mhas_write_sync_header(ptr_asc_bit_buf);

    if (p_obj_mpeghe->config.crc16 == 1 || p_obj_mpeghe->config.crc32 == 1)
    {
      if (p_obj_mpeghe->config.crc16 == 1)
      {
        crc_len = 16;
      }
      else if (p_obj_mpeghe->config.crc32 == 1)
      {
        crc_len = 32;
      }
      WORD32 out_buf[MAXCRC];
      impeghe_enc_crc((WORD32 *)str_temp_bit_buf.ptr_bit_buf_base, out_buf, ((num_bits + 7) >> 3),
                      crc_len);
      crc_val = 0;
      for (WORD32 loop = 0; loop < crc_len; loop++)
      {
        crc_val = crc_val << 1;
        crc_val += out_buf[loop];
      }

      if (p_obj_mpeghe->config.crc16 == 1)
      {
        impeghe_mhas_write_crc_header(ptr_asc_bit_buf, MHAS_PAC_TYP_CRC16, crc_len, crc_val);
      }
      else
      {
        impeghe_mhas_write_crc_header(ptr_asc_bit_buf, MHAS_PAC_TYP_CRC32, crc_len, crc_val);
      }
    }
    else if (p_obj_mpeghe->config.global_crc16 == 1 || p_obj_mpeghe->config.global_crc32 == 1)
    {
      if (p_obj_mpeghe->config.global_crc16 == 1)
      {
        crc_len = 16;
      }
      else
      {
        crc_len = 32;
      }
      WORD32 out_buf[MAXCRC];
      impeghe_enc_crc((WORD32 *)str_temp_bit_buf.ptr_bit_buf_base, out_buf, ((num_bits + 7) >> 3),
                      crc_len);
      crc_val = 0;
      for (WORD32 loop = 0; loop < crc_len; loop++)
      {
        crc_val = crc_val << 1;
        crc_val += out_buf[loop];
      }

      if (p_obj_mpeghe->config.global_crc16 == 1)
      {
        impeghe_mhas_write_global_crc_header(ptr_asc_bit_buf, MHAS_PAC_TYP_GLOBAL_CRC16, crc_len,
                                             crc_val, 0, 1);
      }
      else
      {
        impeghe_mhas_write_global_crc_header(ptr_asc_bit_buf, MHAS_PAC_TYP_GLOBAL_CRC32, crc_len,
                                             crc_val, 0, 1);
      }
    }
    impeghe_mhas_write_cfg_only_header(ptr_asc_bit_buf, num_bits);
  }

  impeghe_get_audiospecific_config_bytes(
      ptr_asc_bit_buf, &p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_scratch,
      &p_obj_mpeghe->config.audio_specific_config);
  pstr_output_config->i_dec_len = (ptr_asc_bit_buf->cnt_bits + 7) / 8;

  return err_code;
}

/**
 *  impeghe_execute
 *
 *  \brief Execute encoding preocess
 *
 *  \param [in,out] p_ia_mpeghe_obj  Pointer to the encoder object
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Error code
 *
 */
IA_ERRORCODE impeghe_execute(pVOID p_ia_mpeghe_obj, pVOID pv_input, pVOID pv_output)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  WORD32 header_bytes = 0;
  ia_mpeghe_api_struct *p_obj_mpeghe = (ia_mpeghe_api_struct *)p_ia_mpeghe_obj;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;

  if (pstr_input_config->str_ec_info_struct.ec_active)
  {
    p_obj_mpeghe->config.str_ec_config_struct.ec_active = 1;
  }
  else
  {
    p_obj_mpeghe->config.str_ec_config_struct.ec_active = 0;
  }
  err_code = impeghe_process(p_obj_mpeghe, &header_bytes);
  if (err_code)
  {
    pstr_output_config->i_out_bytes = 0;
    return err_code;
  }

  p_obj_mpeghe->p_state_mpeghe->ui_bytes_consumed = 0;
  pstr_output_config->i_out_bytes = p_obj_mpeghe->p_state_mpeghe->i_out_bytes;
  return err_code;
}

/**
 *  impeghe_delete
 *
 *  \brief Deletes the encoder object
 *
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return VOID
 *
 */
VOID impeghe_delete(pVOID pv_output)
{
  LOOPIDX idx;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;
  for (idx = 0; idx < (WORD32)pstr_output_config->malloc_count; idx++)
  {
    if (pstr_output_config->arr_alloc_memory[idx])
      free(pstr_output_config->arr_alloc_memory[idx]);
  }

  return;
}

/**
 *  impeghe_scratch_size
 *
 *  \brief Calculates the scratch memory size
 *
 *  \param [in]	pstr_usac_config	Pointer to encoder configurations structure
 *
 *  \return WORD32 Scratch size
 *
 */
WORD32 impeghe_scratch_size(ia_usac_encoder_config_struct *pstr_usac_config)
{
  WORD32 scr_size = 0;

  /* Memory allocation for handles */
  scr_size += sizeof(ia_bit_buf_struct *);

  /* Memory allocation for actual stream struct */
  scr_size += sizeof(ia_bit_buf_struct);

  scr_size += IMPEGHE_FLOAT64_SCRATCH_SIZE * sizeof(FLOAT64);
  scr_size += IMPEGHE_FLOAT32_SCRATCH_SIZE * sizeof(FLOAT32);
  scr_size += 4096 * sizeof(FLOAT64) * pstr_usac_config->channels;
  scr_size += 5 * 1024 * 1024;
  return scr_size;
}

/**
 *  impeghe_calc_pers_buf_sizes
 *
 *  \brief Calculates the persistent buffer sizes
 *
 *  \param [in]	p_obj_mpeghe	Pointer to API structure
 *
 *  \return WORD32 Persistent memory size
 *
 */
WORD32 impeghe_calc_pers_buf_sizes(ia_mpeghe_api_struct *p_obj_mpeghe)
{
  WORD32 pers_size = 0;
  ia_usac_encoder_config_struct *pstr_usac_config = &p_obj_mpeghe->config;

  pers_size += pstr_usac_config->channels * sizeof(FLOAT32 *);
  pers_size += pstr_usac_config->channels * sizeof(FLOAT32 *);
  pers_size += pstr_usac_config->channels * sizeof(FLOAT32 *);

  pers_size += ((2 * pstr_usac_config->ccfl) * sizeof(FLOAT32) * pstr_usac_config->channels);

  pers_size += pstr_usac_config->ccfl * sizeof(FLOAT32) * pstr_usac_config->channels;
  pers_size += pstr_usac_config->ccfl * sizeof(FLOAT32) * pstr_usac_config->channels * 2;

  pers_size += 2 * pstr_usac_config->ccfl * sizeof(FLOAT64) * pstr_usac_config->channels;

  pers_size += pstr_usac_config->ccfl * sizeof(FLOAT64) * pstr_usac_config->channels;

  pers_size += (pstr_usac_config->ccfl + 128) * sizeof(FLOAT64) * pstr_usac_config->channels;

  pers_size += (pstr_usac_config->ccfl + 128) * sizeof(FLOAT64) * pstr_usac_config->channels;

  pers_size += 3 * pstr_usac_config->ccfl * sizeof(FLOAT64) * pstr_usac_config->channels;

  pers_size += pstr_usac_config->ccfl * 4 * sizeof(FLOAT64) * pstr_usac_config->channels;

  if (pstr_usac_config->tns_select != 0)
    pers_size += sizeof(ia_tns_info) * pstr_usac_config->channels;

  pers_size += sizeof(ia_usac_td_encoder_struct) * pstr_usac_config->channels;
  return pers_size;
}

/**
 *  impeghe_enc_fill_mem_tables
 *
 *  \brief Fills memory tables with size, alignment and priority details
 *
 *  \param [in,out]	p_obj_mpeghe	Pointer to API structure
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_enc_fill_mem_tables(ia_mpeghe_api_struct *p_obj_mpeghe)
{
  ia_mem_info_struct *ptr_mem_info;
  WORD32 num_aac_chan;

  /* persistent */
  {
    ptr_mem_info = &p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_PERSIST];
    ptr_mem_info->ui_size = sizeof(ia_usac_enc_state_struct);

    /* function to calculate all persistent buffer sizes */
    ptr_mem_info->ui_size += impeghe_calc_pers_buf_sizes(p_obj_mpeghe);
    ptr_mem_info->ui_alignment = 8;
    ptr_mem_info->ui_type = IA_MEMTYPE_PERSIST;
    ptr_mem_info->ui_priority = IA_MEMPRIORITY_ANYWHERE;
    ptr_mem_info->ui_placed[0] = 0;
    ptr_mem_info->ui_placed[1] = 0;
  }
  /* scratch */
  {
    ptr_mem_info = &p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_SCRATCH];
    ptr_mem_info->ui_size = 32;
    ptr_mem_info->ui_size += impeghe_scratch_size(&p_obj_mpeghe->config);

    ptr_mem_info->ui_alignment = 8;
    ptr_mem_info->ui_type = IA_MEMTYPE_SCRATCH;
    ptr_mem_info->ui_priority = IA_MEMPRIORITY_ANYWHERE;
    ptr_mem_info->ui_placed[0] = 0;
    ptr_mem_info->ui_placed[1] = 0;
  }
  /* input */
  {
    ptr_mem_info = &p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_INPUT];
    num_aac_chan = p_obj_mpeghe->config.aud_ch + p_obj_mpeghe->config.num_oam_ch;
    ptr_mem_info->ui_size =
        FRAME_LEN_LONG * num_aac_chan * (p_obj_mpeghe->config.ui_pcm_wd_sz >> 3);

    ptr_mem_info->ui_alignment = 4;
    ptr_mem_info->ui_type = IA_MEMTYPE_INPUT;
    ptr_mem_info->ui_priority = IA_MEMPRIORITY_ANYWHERE;
    ptr_mem_info->ui_placed[0] = 0;
    ptr_mem_info->ui_placed[1] = 0;
  }

  /* HOA input */
  {
    ptr_mem_info = &p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_INPUT_HOA];
    ptr_mem_info->ui_size = MAX_NUM_HOA_COEFFS * MAX_FRAME_LEN * sizeof(float);
    ptr_mem_info->ui_alignment = 4;
    ptr_mem_info->ui_type = IA_MEMTYPE_INPUT_HOA;
    ptr_mem_info->ui_priority = IA_MEMPRIORITY_ANYWHERE;
    ptr_mem_info->ui_placed[0] = 0;
    ptr_mem_info->ui_placed[1] = 0;
  }

  /* output */
  {
    ptr_mem_info = &p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT];
    ptr_mem_info->ui_size = (MAX_CHANNEL_BITS / 8) * p_obj_mpeghe->config.channels;
    ptr_mem_info->ui_alignment = 1;
    ptr_mem_info->ui_type = IA_MEMTYPE_OUTPUT;
    ptr_mem_info->ui_priority = IA_MEMPRIORITY_ANYWHERE;
    ptr_mem_info->ui_placed[0] = 0;
    ptr_mem_info->ui_placed[1] = 0;
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_initialize
 *
 *  \brief Initializes the encoder
 *
 *  \param [in,out]	p_obj_mpeghe	Pointer to API structure
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_initialize(ia_mpeghe_api_struct *p_obj_mpeghe)
{
  LOOPIDX i = 0;
  IA_ERRORCODE error = IA_NO_ERROR;

  ia_usac_encoder_config_struct *pstr_usac_config = &p_obj_mpeghe->config;
  ia_usac_data_struct *pstr_usac_data = &p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data;

  if (pstr_usac_config->codec_mode == USAC_ONLY_TD)
  {
    for (i = 0; i < p_obj_mpeghe->config.channels; i++)
    {
      pstr_usac_data->core_mode_prev[i] = CORE_MODE_TD;
      pstr_usac_data->core_mode[i] = CORE_MODE_TD;
    }
  }
  else
  {
    for (i = 0; i < p_obj_mpeghe->config.channels; i++)
    {
      pstr_usac_data->core_mode_prev[i] = CORE_MODE_FD;
      pstr_usac_data->core_mode[i] = CORE_MODE_FD;
    }
  }
  /* Assigning scracth pointer that is needed for HOA init */
  p_obj_mpeghe->p_state_mpeghe->hoa_scratch = p_obj_mpeghe->pp_mem[IA_MEMTYPE_SCRATCH];
  error = impeghe_enc_init(pstr_usac_config, p_obj_mpeghe->p_state_mpeghe);

  if (0 != error)
  {
    return error;
  }
  p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.frame_count = 0;

  return IA_NO_ERROR;
}

/**
 *  impeghe_process
 *
 *  \brief Process the raw data to generate encoded frame
 *
 *  \param [in,out]	p_obj_mpeghe	Pointer to API structure
 *  \param [unused]	header_bytes	Pointer to header bytes
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_process(ia_mpeghe_api_struct *p_obj_mpeghe, WORD32 *header_bytes)
{
  LOOPIDX i;
  IA_ERRORCODE err = IA_NO_ERROR;
  jmp_buf api_execute_jmp_buf;
  err = setjmp(api_execute_jmp_buf);
  if (err != IA_NO_ERROR)
  {
    return IMPEGHE_EXE_NONFATAL_INSUFFICIENT_WRITE_BUFFER_SIZE;
  }
  WORD16 *ps_inp_buf = (WORD16 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_INPUT];
  WORD8 *pi1_inp_buf = (WORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_INPUT];
  WORD32 *pi4_inp_buf = (WORD32 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_INPUT];
  WORD8 *ps_out_buf = (WORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT];
  ia_usac_enc_state_struct *p_state = p_obj_mpeghe->p_state_mpeghe;
  ia_bit_buf_struct *it_bit_buff = &p_state->str_bit_buf;
  ia_usac_encoder_config_struct *pstr_usac_config = &p_obj_mpeghe->config;
  ia_usac_data_struct *pstr_usac_data = &p_state->str_usac_enc_data;
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  ia_classification_struct *pstr_sig_class_data = &p_state->str_usac_enc_data.str_sig_class_data;
  WORD32 num_samples;
  UWORD32 padding_bits = 0;
  WORD32 short_count = (pstr_usac_config->in_frame_length *
                        (pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch));

  WORD32 i4_inp_data;
  WORD32 ptr_inp_buf_offset = 0;

  ia_resampler_struct *pstr_resampler = &p_state->str_usac_enc_data.resampler[0];
  FLOAT32 *ptr_inp_buf[MAX_TIME_CHANNELS];
  WORD32 resample_input = ((USAC_ONLY_FD != pstr_usac_config->codec_mode) &&
                           (32000 < pstr_usac_config->native_sampling_rate));
  FLOAT32 *ptr_scratch = p_obj_mpeghe->pp_mem[IA_MEMTYPE_SCRATCH];
  if (p_obj_mpeghe->config.str_ec_config_struct.ec_active)
  {
    UWORD8 *ptr_out = (UWORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT];
    WORD32 bit_cnt = 0;
    WORD32 packet_bytes = 0, packet_bytes1 = 0;
    p_obj_mpeghe->p_state_mpeghe->i_out_bytes = 0;
    it_bit_buff =
        impeghe_create_bit_buffer(it_bit_buff, p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT],
                                  p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT].ui_size);
    it_bit_buff->impeghe_jmp_buf = &api_execute_jmp_buf;
    if (!p_obj_mpeghe->config.str_ec_config_struct.not_first_frame)
    {
      /*earcon packet*/
      impeghe_write_ec_pkt(it_bit_buff, &p_obj_mpeghe->config.str_ec_config_struct);
      padding_bits = 8 - (it_bit_buff->cnt_bits & 7);
      if (padding_bits > 0 && padding_bits < 8)
      {
        ps_out_buf[it_bit_buff->cnt_bits >> 3] =
            (WORD8)((UWORD32)ps_out_buf[it_bit_buff->cnt_bits >> 3]) & (0xFF << padding_bits);
      }
      p_obj_mpeghe->p_state_mpeghe->i_out_bytes =
          (padding_bits > 0 && padding_bits < 8) ? (it_bit_buff->cnt_bits + padding_bits) >> 3
                                                 : it_bit_buff->cnt_bits >> 3;
      impeghe_reset_bit_buffer(p_obj_mpeghe->config.ptr_mhas_bit_buf);

      /* write earcon header */
      bit_cnt = impeghe_mhas_write_earcon_header(p_obj_mpeghe->config.ptr_mhas_bit_buf,
                                                 MHAS_PAC_TYP_EARCON,
                                                 p_obj_mpeghe->p_state_mpeghe->i_out_bytes * 8);
      memmove((ptr_out + (bit_cnt >> 3)), ptr_out, p_obj_mpeghe->p_state_mpeghe->i_out_bytes);
      memcpy(ptr_out, p_obj_mpeghe->config.ptr_mhas_bit_buf->ptr_bit_buf_base, (bit_cnt >> 3));
      p_obj_mpeghe->p_state_mpeghe->i_out_bytes += (bit_cnt) >> 3;
      it_bit_buff->cnt_bits += bit_cnt + padding_bits;
      bit_cnt = it_bit_buff->cnt_bits;
      packet_bytes = p_obj_mpeghe->p_state_mpeghe->i_out_bytes;
      packet_bytes1 = p_obj_mpeghe->p_state_mpeghe->i_out_bytes;
      it_bit_buff->ptr_write_next += packet_bytes;
      ptr_out += p_obj_mpeghe->p_state_mpeghe->i_out_bytes;
      memset(ptr_out, 0, 1024);
      it_bit_buff->ptr_write_next = ptr_out;
      it_bit_buff->write_position = 7;

      /*pcm config packet */
      impeghe_write_pcm_config_pkt(it_bit_buff, &p_obj_mpeghe->config.str_pcm_data_config);
      padding_bits = 8 - ((it_bit_buff->cnt_bits - bit_cnt) & 7);
      if (padding_bits > 0)
      {
        ps_out_buf[it_bit_buff->cnt_bits >> 3] =
            (WORD8)((UWORD32)ps_out_buf[it_bit_buff->cnt_bits >> 3]) & (0xFF << padding_bits);
      }
      p_obj_mpeghe->p_state_mpeghe->i_out_bytes = (it_bit_buff->cnt_bits + padding_bits) >> 3;

      impeghe_reset_bit_buffer(p_obj_mpeghe->config.ptr_mhas_bit_buf);
      packet_bytes = p_obj_mpeghe->p_state_mpeghe->i_out_bytes - packet_bytes;

      /* write pcm config header */
      bit_cnt = impeghe_mhas_write_earcon_header(p_obj_mpeghe->config.ptr_mhas_bit_buf,
                                                 MHAS_PAC_TYP_PCM_CONFIG, packet_bytes * 8);
      memmove((ptr_out + (bit_cnt >> 3)), ptr_out, packet_bytes);
      memcpy(ptr_out, p_obj_mpeghe->config.ptr_mhas_bit_buf->ptr_bit_buf_base, (bit_cnt >> 3));
      p_obj_mpeghe->p_state_mpeghe->i_out_bytes += (bit_cnt) >> 3;
      it_bit_buff->cnt_bits += bit_cnt + padding_bits;
      bit_cnt = it_bit_buff->cnt_bits;
      p_obj_mpeghe->config.str_ec_config_struct.not_first_frame = 1;
      ptr_out += p_obj_mpeghe->p_state_mpeghe->i_out_bytes - packet_bytes1;
      it_bit_buff->ptr_write_next = ptr_out;
      packet_bytes = p_obj_mpeghe->p_state_mpeghe->i_out_bytes;
      it_bit_buff->write_position = 7;
    }

    /*pcm_payload data packet */
    impeghe_write_pcm_data_pkt(it_bit_buff, &p_obj_mpeghe->config.str_pcm_data_config);
    padding_bits = 8 - ((it_bit_buff->cnt_bits - bit_cnt) & 7);
    if (padding_bits > 0)
    {
      ps_out_buf[it_bit_buff->cnt_bits >> 3] =
          (WORD8)((UWORD32)ps_out_buf[it_bit_buff->cnt_bits >> 3]) & (0xFF << padding_bits);
    }
    p_obj_mpeghe->p_state_mpeghe->i_out_bytes = (it_bit_buff->cnt_bits + padding_bits) >> 3;

    packet_bytes = p_obj_mpeghe->p_state_mpeghe->i_out_bytes - packet_bytes;
    impeghe_reset_bit_buffer(p_obj_mpeghe->config.ptr_mhas_bit_buf);

    /*pcm payload packet header*/
    bit_cnt = impeghe_mhas_write_earcon_header(p_obj_mpeghe->config.ptr_mhas_bit_buf,
                                               MHAS_PAC_TYP_PCM_DATA, packet_bytes * 8);
    memmove((ptr_out + (bit_cnt >> 3)), ptr_out, packet_bytes);
    memcpy(ptr_out, p_obj_mpeghe->config.ptr_mhas_bit_buf->ptr_bit_buf_base, (bit_cnt >> 3));
    p_obj_mpeghe->p_state_mpeghe->i_out_bytes += (bit_cnt + 7) >> 3;
    return IA_NO_ERROR;
  }
  if (resample_input)
  {
    for (i = 0; i < pstr_usac_config->channels; i++)
    {
      ptr_inp_buf[i] = &ptr_scratch[i * pstr_usac_config->in_frame_length];
    }
  }
  else
  {
    if (pstr_usac_config->use_drc_element)
    {
      WORD32 num_ch = (pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch);
      for (i = 0; i < short_count; i++)
      {
        p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[i % num_ch][i / num_ch] =
            p_obj_mpeghe->p_state_mpeghe
                ->ptr_in_buf[i % num_ch][i / num_ch + pstr_usac_config->ccfl];
      }
      ptr_inp_buf_offset = pstr_usac_config->ccfl;
    }

    for (i = 0; i < pstr_usac_config->channels; i++)
    {
      ptr_inp_buf[i] = p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[i];
    }
  }

  // Assumption:HOA input is always float PCM
  if ((pstr_usac_config->aud_ch != 0) || (pstr_usac_config->num_oam_ch != 0))
  {
    WORD32 num_ch = pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch;

    if (16 == pstr_usac_config->ui_pcm_wd_sz)
    {
      for (i = 0; i < short_count; i++)
      {

        ptr_inp_buf[i % num_ch][i / num_ch + ptr_inp_buf_offset] = ps_inp_buf[i];
      }
    }
    else if (24 == pstr_usac_config->ui_pcm_wd_sz)
    {
      for (i = 0; i < short_count; i++)
      {
        i4_inp_data = ((WORD32)(*pi1_inp_buf)) & 0xFF;
        pi1_inp_buf++;
        i4_inp_data += ((WORD32)(*pi1_inp_buf) << 8) & 0xFFFF;
        pi1_inp_buf++;
        i4_inp_data += ((WORD32)(*pi1_inp_buf) << 16) & 0xFFFFFF; // Extract only 24 bits
        pi1_inp_buf++;
        i4_inp_data = i4_inp_data - (i4_inp_data >> 23 << 24); // To take care of sign bit
        ptr_inp_buf[i % num_ch][i / num_ch + ptr_inp_buf_offset] = i4_inp_data / 256.0f;
      }
    }
    else if (32 == pstr_usac_config->ui_pcm_wd_sz)
    {
      pi4_inp_buf = (WORD32 *)pi1_inp_buf;
      for (i = 0; i < short_count; i++)
      {
        i4_inp_data = *pi4_inp_buf++;
        ptr_inp_buf[i % num_ch][i / num_ch + ptr_inp_buf_offset] = i4_inp_data / 65536.0f;
      }
    }
  }
  if (resample_input)
  {
    WORD32 i_ch;
    if (!(((pstr_usac_config->cc_resamp_fac_down == 3) &&
           (pstr_usac_config->cc_resamp_fac_up == 2)) ||
          ((pstr_usac_config->cc_resamp_fac_down == 2) &&
           (pstr_usac_config->cc_resamp_fac_up == 1))))
    {
      return -1;
    }
    if (1 == pstr_usac_config->use_drc_element)
    {
      WORD32 num_ch = (pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch);
      for (i = 0; i < short_count; i++)
      {
        p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[i % num_ch][i / num_ch] =
            p_obj_mpeghe->p_state_mpeghe
                ->ptr_in_buf[i % num_ch][i / num_ch + pstr_usac_config->ccfl];
      }
    }
    for (i_ch = 0; i_ch < pstr_usac_config->channels; i_ch++)
    {
      pstr_resampler[i_ch].fac_down = pstr_usac_config->cc_resamp_fac_down;
      pstr_resampler[i_ch].fac_up = pstr_usac_config->cc_resamp_fac_up;
      pstr_resampler[i_ch].input_length = pstr_usac_config->in_frame_length;

      if (1 == pstr_usac_config->use_drc_element)
      {
        impeghe_resample(
            &pstr_resampler[i_ch], ptr_inp_buf[i_ch],
            &(p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[i_ch][pstr_usac_config->ccfl]),
            &ptr_scratch[short_count]);
      }
      else
      {
        impeghe_resample(&pstr_resampler[i_ch], ptr_inp_buf[i_ch],
                         p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[i_ch],
                         &ptr_scratch[short_count]);
      }
    }
  }

  if ((1 > p_state->ui_num_inp_received) && (pstr_usac_config->use_drc_element))
  {
    if (pstr_usac_config->use_hoa)
    {
      /* Perform HOA Encoding */
      WORD32 num_hoa_coeff =
          p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.num_hoa_coeffs;
      UWORD32 output_size = 0;
      UWORD32 in_samples_per_ch = pstr_usac_config->ccfl;
      FLOAT32 *f_transport_channel[MAX_NUM_PERC_CODERS];
      FLOAT32 *hoa_input[MAX_NUM_HOA_COEFFS];
      WORD32 out_ch_offset = pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch;
      WORD32 input_ptr_offset = 0;
      WORD32 max_out_ch_offset =
          pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch +
          p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.tot_coders;

      if (1 == pstr_usac_config->use_drc_element)
      {
        input_ptr_offset = pstr_usac_config->ccfl;
      }
      for (i = out_ch_offset; i < max_out_ch_offset; i++)
      {
        f_transport_channel[i - out_ch_offset] = &ptr_inp_buf[i][input_ptr_offset];
      }
      FLOAT32 *input_ptr = (FLOAT32 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_INPUT_HOA];

      for (i = 0; i < (WORD32)(num_hoa_coeff * in_samples_per_ch); i++)
      {
        if ((fabs(input_ptr[i])) > MAX_HOA_INPUT_RANGE)
        {
          p_obj_mpeghe->p_state_mpeghe->ui_bytes_consumed = in_samples_per_ch * sizeof(FLOAT32);
          p_obj_mpeghe->p_state_mpeghe->i_out_bytes = 0;
          return IMPEGHE_CONFIG_NONFATAL_INPUT_OUT_OF_RANGE;
        }
      }

      for (i = 0; i < num_hoa_coeff; i++)
      {
        hoa_input[i] = input_ptr + (i * in_samples_per_ch);
      }

      impeghe_reset_bit_buffer(&(p_obj_mpeghe->p_state_mpeghe->str_hoa_bit_buf));

      err = impeghe_hoa_encoder_process(
          &p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state, hoa_input,
          &in_samples_per_ch, &(p_obj_mpeghe->p_state_mpeghe->str_hoa_bit_buf), &output_size,
          f_transport_channel);

      if (0 != err)
        return -1;

      memset(p_obj_mpeghe->p_state_mpeghe->prev_hoa_bit_stream, 0,
             sizeof(p_obj_mpeghe->p_state_mpeghe->prev_hoa_bit_stream));
      memset(
          p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.prev_frm_bs_bits, 0,
          sizeof(p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.prev_frm_bs_bits));
    }
    p_state->ui_num_inp_received++;
    return IA_NO_ERROR;
  }

  num_samples = p_state->ui_input_over ? 0 : pstr_usac_config->ccfl;

  if (pstr_sig_class_data->is_switch_mode)
  {
    for (i = 0; i < num_samples; i++)
    {
      pstr_sig_class_data->input_samples[pstr_sig_class_data->n_buffer_samples + i] =
          p_obj_mpeghe->p_state_mpeghe->ptr_in_buf[0][i];
    }
    pstr_sig_class_data->n_buffer_samples += num_samples;
    impeghe_classification(pstr_sig_class_data, pstr_scratch);
  }

  it_bit_buff =
      impeghe_create_bit_buffer(it_bit_buff, p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT],
                                p_obj_mpeghe->p_mem_info_mpeghe[IA_MEMTYPE_OUTPUT].ui_size);
  if (it_bit_buff == NULL)
  {
    return -1;
  }
  it_bit_buff->impeghe_jmp_buf = &api_execute_jmp_buf;
  if (pstr_usac_config->use_hoa)
  {
    /* Perform HOA Encoding */
    WORD32 num_hoa_coeff =
        p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.num_hoa_coeffs;
    UWORD32 output_size = 0;
    UWORD32 in_samples_per_ch = pstr_usac_config->ccfl;
    FLOAT32 *ptr_f_transport_channel[MAX_NUM_PERC_CODERS];
    FLOAT32 *ptr_hoa_input[MAX_NUM_HOA_COEFFS];
    WORD32 out_ch_offset = pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch;
    WORD32 input_ptr_offset = 0;
    WORD32 max_out_ch_offset =
        pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch +
        p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.tot_coders;

    if (1 == pstr_usac_config->use_drc_element)
    {
      for (i = out_ch_offset; i < max_out_ch_offset; i++)
      {
        for (int j = 0; j < MAX_FRAME_LEN; j++)
        {
          ptr_inp_buf[i][j] = ptr_inp_buf[i][j + pstr_usac_config->ccfl];
        }
      }

      memcpy(p_obj_mpeghe->p_state_mpeghe->prev_hoa_bit_stream,
             p_obj_mpeghe->p_state_mpeghe->hoa_bit_stream,
             sizeof(p_obj_mpeghe->p_state_mpeghe->prev_hoa_bit_stream));
      memcpy(p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.prev_frm_bs_bits,
             p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.frm_bs_bits,
             sizeof(p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state.frm_bs_bits));

      input_ptr_offset = pstr_usac_config->ccfl;
    }
    for (i = out_ch_offset; i < max_out_ch_offset; i++)
    {
      ptr_f_transport_channel[i - out_ch_offset] = &ptr_inp_buf[i][input_ptr_offset];
    }
    FLOAT32 *ptr_input = (FLOAT32 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_INPUT_HOA];

    for (i = 0; i < (WORD32)(num_hoa_coeff * in_samples_per_ch); i++)
    {
      if ((fabs(ptr_input[i])) > MAX_HOA_INPUT_RANGE)
      {
        p_obj_mpeghe->p_state_mpeghe->ui_bytes_consumed = in_samples_per_ch * sizeof(FLOAT32);
        p_obj_mpeghe->p_state_mpeghe->i_out_bytes = 0;
        return IMPEGHE_CONFIG_NONFATAL_INPUT_OUT_OF_RANGE;
      }
    }

    for (i = 0; i < num_hoa_coeff; i++)
    {
      ptr_hoa_input[i] = ptr_input + (i * in_samples_per_ch);
    }

    impeghe_reset_bit_buffer(&(p_obj_mpeghe->p_state_mpeghe->str_hoa_bit_buf));

    err = impeghe_hoa_encoder_process(
        &p_obj_mpeghe->p_state_mpeghe->str_usac_enc_data.str_hoa_state, ptr_hoa_input,
        &in_samples_per_ch, &(p_obj_mpeghe->p_state_mpeghe->str_hoa_bit_buf), &output_size,
        ptr_f_transport_channel);

    if (0 != err)
      return -1;
  }

  err = impeghe_core_coder_process(p_obj_mpeghe->p_state_mpeghe->ptr_in_buf, pstr_usac_config,
                                   p_obj_mpeghe->p_state_mpeghe, it_bit_buff);
  if (err)
    return err;

  padding_bits = 8 - (it_bit_buff->cnt_bits & 7);
  if (padding_bits > 0 && padding_bits < 8)
  {
    ps_out_buf[it_bit_buff->cnt_bits >> 3] =
        (WORD8)((UWORD32)ps_out_buf[it_bit_buff->cnt_bits >> 3]) & (0xFF << padding_bits);
  }
  p_obj_mpeghe->p_state_mpeghe->i_out_bytes = (padding_bits > 0 && padding_bits < 8)
                                                  ? (it_bit_buff->cnt_bits + padding_bits) >> 3
                                                  : it_bit_buff->cnt_bits >> 3;
  if (p_obj_mpeghe->config.mhas_pkt == 1)
  {
    UWORD32 crc_val = 0;
    WORD32 crc_len = 0;
    if (p_obj_mpeghe->config.crc16 == 1 || p_obj_mpeghe->config.crc32 == 1)
    {
      WORD32 out_buf[MAXCRC];
      if (p_obj_mpeghe->config.crc16 == 1)
      {
        crc_len = 16;
      }
      else
      {
        crc_len = 32;
      }
      impeghe_enc_crc((WORD32 *)it_bit_buff->ptr_bit_buf_base, out_buf,
                      p_obj_mpeghe->p_state_mpeghe->i_out_bytes, crc_len);
      crc_val = 0;
      for (WORD32 loop = 0; loop < crc_len; loop++)
      {
        crc_val = crc_val << 1;
        crc_val += out_buf[loop];
      }
    }
    WORD32 bit_cnt;
    UWORD8 *ptr_out = (UWORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT];

    impeghe_reset_bit_buffer(p_obj_mpeghe->config.ptr_mhas_bit_buf);

    /* write mhas frame */
    bit_cnt = impeghe_mhas_write_frame_header(p_obj_mpeghe->config.ptr_mhas_bit_buf,
                                              p_obj_mpeghe->p_state_mpeghe->i_out_bytes * 8);

    memmove((ptr_out + (bit_cnt >> 3)), ptr_out, p_obj_mpeghe->p_state_mpeghe->i_out_bytes);

    memcpy(ptr_out, p_obj_mpeghe->config.ptr_mhas_bit_buf->ptr_bit_buf_base, (bit_cnt >> 3));

    p_obj_mpeghe->p_state_mpeghe->i_out_bytes += (bit_cnt + 7) >> 3;
    if (p_obj_mpeghe->config.crc16 == 1 || p_obj_mpeghe->config.crc32 == 1)
    {
      UWORD8 *ptr_out = (UWORD8 *)p_obj_mpeghe->pp_mem[IA_MEMTYPE_OUTPUT];
      WORD32 bit_cnt = 0;
      WORD32 bytes_count;
      if (p_obj_mpeghe->config.crc16 == 1)
      {
        UWORD8 ia_mhas_crc16_buf[CRC16_LENGTH];
        ia_bit_buf_struct ia_crc_bit_buf_str;
        ia_crc_bit_buf_str.ptr_bit_buf_base = &ia_mhas_crc16_buf[0];
        ia_crc_bit_buf_str.ptr_bit_buf_end = &ia_mhas_crc16_buf[CRC16_LENGTH];
        ia_crc_bit_buf_str.size = (CRC16_LENGTH << 3);
        ia_crc_bit_buf_str.cnt_bits = 0;
        impeghe_reset_bit_buffer(&ia_crc_bit_buf_str);
        bit_cnt = impeghe_mhas_write_crc_header(&ia_crc_bit_buf_str, MHAS_PAC_TYP_CRC16, crc_len,
                                                crc_val);
        bytes_count = (bit_cnt + 7) >> 3;
        memmove((ptr_out + bytes_count), ptr_out, p_obj_mpeghe->p_state_mpeghe->i_out_bytes);
        memcpy(ptr_out, ia_crc_bit_buf_str.ptr_bit_buf_base, (bytes_count));
        p_obj_mpeghe->p_state_mpeghe->i_out_bytes += bytes_count;
      }
      else
      {
        UWORD8 mhas_crc32_buf[CRC32_LENGTH];
        ia_bit_buf_struct ia_crc_bit_buf_str;
        ia_crc_bit_buf_str.ptr_bit_buf_base = &mhas_crc32_buf[0];
        ia_crc_bit_buf_str.ptr_bit_buf_end = &mhas_crc32_buf[CRC32_LENGTH];
        ia_crc_bit_buf_str.size = (CRC32_LENGTH << 3);
        ia_crc_bit_buf_str.cnt_bits = 0;
        impeghe_reset_bit_buffer(&ia_crc_bit_buf_str);
        bit_cnt = impeghe_mhas_write_crc_header(&ia_crc_bit_buf_str, MHAS_PAC_TYP_CRC32, crc_len,
                                                crc_val);
        bytes_count = (bit_cnt + 7) >> 3;
        memmove((ptr_out + bytes_count), ptr_out, p_obj_mpeghe->p_state_mpeghe->i_out_bytes);
        memcpy(ptr_out, ia_crc_bit_buf_str.ptr_bit_buf_base, (bytes_count));
        p_obj_mpeghe->p_state_mpeghe->i_out_bytes += bytes_count;
      }
    }
  }

  p_state->str_usac_enc_data.frame_count++;

  return IA_NO_ERROR;
}
