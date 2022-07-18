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
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_dmx_matrix_enc_api.h"
#include "impeghe_memory_standards.h"
#include "impeghe_cnst.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_rom.h"

/**
 *  impeghe_downmix_config
 *
 *  \brief Writes downmix configuration
 *
 *  \param [in,out] it_bit_buff       Pointer to bit-buffer
 *  \param [in] pstr_ext_cfg_downmix  Pointer to Downmix configuration structure
 *  \param [in,out] pstr_dmx_scratch  Pointer to scratch memory
 *  \param [in,out] bit_cnt       Pointer to bit-counter
 *
 *  \return IA_ERRORCODE Error code
 */
static IA_ERRORCODE impeghe_downmix_config(ia_bit_buf_struct *it_bit_buff,
                                           ia_mpeghe_ext_cfg_downmix_struct *pstr_ext_cfg_downmix,
                                           ia_dmx_sratch *pstr_dmx_scratch, WORD32 *bit_cnt)
{
  LOOPIDX i, k, l, m;
  WORD32 bit_cnt_local = 0;
  WORD32 bit_cnt_mtx_local = 0;
  WORD32 byte_cnt_mtx_local = 0;
  WORD32 err_code = 0;
  ia_bit_buf_struct it_bit_buf_mtx;

  bit_cnt_local += impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->dmx_config_type, 2);

  if ((pstr_ext_cfg_downmix->dmx_config_type == 0) ||
      (pstr_ext_cfg_downmix->dmx_config_type == 2))
  {
    bit_cnt_local +=
        impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->passive_dmx_flag, 1);
    if (pstr_ext_cfg_downmix->passive_dmx_flag == 0)
    {
      bit_cnt_local +=
          impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->phase_align_strength, 3);
    }
    bit_cnt_local +=
        impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->immersive_downmix_flag, 1);
  }

  if ((pstr_ext_cfg_downmix->dmx_config_type == 1) ||
      (pstr_ext_cfg_downmix->dmx_config_type == 2))
  {

    impeghe_create_bit_buffer(&it_bit_buf_mtx, pstr_ext_cfg_downmix->dmx_matrix_enc_mem,
                              sizeof(pstr_ext_cfg_downmix->dmx_matrix_enc_mem));
    it_bit_buf_mtx.impeghe_jmp_buf = it_bit_buff->impeghe_jmp_buf;
    bit_cnt_local +=
        impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->downmix_id_count, 5);

    for (k = 0; k < (WORD32)pstr_ext_cfg_downmix->downmix_id_count; k++)
    {
      bit_cnt_local +=
          impeghe_write_bits_buf(it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].dmx_id, 7);

      bit_cnt_local += impeghe_write_bits_buf(
          it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].dmx_type, 2);

      if (pstr_ext_cfg_downmix->str_dmx_matrix[k].dmx_type == 0)
      {
        bit_cnt_local += impeghe_write_bits_buf(
            it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].cicp_spk_layout_idx, 6);
      }
      else if (pstr_ext_cfg_downmix->str_dmx_matrix[k].dmx_type == 1)
      {
        bit_cnt_local += impeghe_write_bits_buf(
            it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].cicp_spk_layout_idx, 6);

        bit_cnt_local += impeghe_write_escape_value(
            it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].downmix_mtx_count - 1, 1, 3, 0);

        for (l = 0; l < pstr_ext_cfg_downmix->str_dmx_matrix[k].downmix_mtx_count; l++)
        {
          bit_cnt_local += impeghe_write_escape_value(
              it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].num_assigned_group_ids[l] - 1,
              1, 4, 4);
          for (m = 0; m < pstr_ext_cfg_downmix->str_dmx_matrix[k].num_assigned_group_ids[l]; m++)
          {
            bit_cnt_local += impeghe_write_bits_buf(
                it_bit_buff, pstr_ext_cfg_downmix->str_dmx_matrix[k].signal_group_id[l][m], 5);
          }

          bit_cnt_mtx_local = 0;
          impeghe_reset_bit_buffer(&it_bit_buf_mtx);
          err_code = impeghe_dmx_matrix_enc(
              &it_bit_buf_mtx, &pstr_ext_cfg_downmix->str_dmx_matrix[k].str_dmx_mtx_cfg,
              pstr_dmx_scratch, &bit_cnt_mtx_local);
          if (err_code)
          {
            return err_code;
          }

          bit_cnt_local += impeghe_write_escape_value(it_bit_buff, bit_cnt_mtx_local, 8, 8, 12);

          byte_cnt_mtx_local = bit_cnt_mtx_local >> 3;
          for (i = 0; i < byte_cnt_mtx_local; i++)
          {
            bit_cnt_local += impeghe_write_bits_buf(
                it_bit_buff, pstr_ext_cfg_downmix->dmx_matrix_enc_mem[i], 8);
          }

          if (bit_cnt_mtx_local & 7)
          {
            bit_cnt_local += impeghe_write_bits_buf(it_bit_buff,
                                                    pstr_ext_cfg_downmix->dmx_matrix_enc_mem[i] >>
                                                        (8 - (bit_cnt_mtx_local & 7)),
                                                    bit_cnt_mtx_local & 7);
          }
        }
      }
    }
  }

  *bit_cnt += bit_cnt_local;

  return err_code;
}

/**
 *  impeghe_config_extension
 *
 *  \brief Writes extension configurations
 *
 *  \param [in,out]	it_bit_buff			Pointer to bit-buffer
 *  \param [in,out]	pstr_usac_config	Pointer to config structure
 *  \param [in,out]	pstr_scratch		Pointer to scratch memory
 *  \param [in,out]	pstr_mae_asi		Pointer to MAE ASI Structure
 *
 *  \return WORD32	Number of bits written
 *
 */
static WORD32 impeghe_config_extension(ia_bit_buf_struct *it_bit_buff,
                                       ia_usac_config_struct *pstr_usac_config,
                                       impeghe_scratch_mem *pstr_scratch,
                                       ia_mae_audio_scene_info *pstr_mae_asi)
{
  WORD32 bit_cnt = 0;
  ULOOPIDX i, j;
  WORD32 bit_cnt_ext;
  ia_bit_buf_struct it_bit_buf_local;
  ia_signal_grp_info str_signal_grp;
  IA_ERRORCODE err_code = 0;
  UWORD32 loudness_byte_val = 0x00;

  UWORD32 fill_byte_val = 0xa5;

  bit_cnt += impeghe_write_escape_value(it_bit_buff, pstr_usac_config->num_config_extensions - 1,
                                        2, 4, 8);

  for (j = 0; j < pstr_usac_config->num_config_extensions; j++)
  {
    switch (pstr_usac_config->usac_config_ext_type[j])
    {
    case ID_CONFIG_EXT_DOWNMIX:
    {
      ia_mpeghe_ext_cfg_downmix_struct *pstr_ext_cfg_downmix =
          &pstr_usac_config->str_ext_cfg_downmix[j];
      bit_cnt_ext = 0;
      memcpy(pstr_ext_cfg_downmix,
             &(pstr_usac_config->str_extn_element_config[j].str_ext_cfg_downmix),
             sizeof(pstr_usac_config->str_extn_element_config[j].str_ext_cfg_downmix));
      impeghe_create_bit_buffer(&it_bit_buf_local,
                                &(pstr_usac_config->usac_cfg_ext_info_buf[j][0]),
                                sizeof(pstr_usac_config->usac_cfg_ext_info_buf[j]));
      it_bit_buf_local.impeghe_jmp_buf = it_bit_buff->impeghe_jmp_buf;
      err_code = impeghe_downmix_config(&it_bit_buf_local, pstr_ext_cfg_downmix,
                                        pstr_scratch->pstr_dmx_scratch, &bit_cnt_ext);
      if (err_code)
      {
        bit_cnt_ext = 0;
      }

      pstr_usac_config->usac_config_ext_len[j] = (bit_cnt_ext + 7) >> 3;
    }
    break;
      // case ID_CONFIG_EXT_LOUDNESS_INFO:
      //{
      //} break;

    case ID_CONFIG_EXT_AUDIOSCENE_INFO:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local,
                                &(pstr_usac_config->usac_cfg_ext_info_buf[j][0]),
                                sizeof(pstr_usac_config->usac_cfg_ext_info_buf[j]));
      err_code = impeghe_mae_asi_write(&it_bit_buf_local, pstr_mae_asi, &bit_cnt_ext);
      if (err_code & IA_FATAL_ERROR)
      {
        bit_cnt_ext = 0;
      }

      pstr_usac_config->usac_config_ext_len[j] = ((bit_cnt_ext + 7) >> 3);
      break;
    }
    case ID_CONFIG_EXT_SIG_GROUP_INFO:
    {

      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local,
                                &(pstr_usac_config->usac_cfg_ext_info_buf[j][0]),
                                sizeof(pstr_usac_config->usac_cfg_ext_info_buf[j]));
      err_code = impeghe_signal_group_info(&it_bit_buf_local, &str_signal_grp,
                                           pstr_usac_config->num_signal_grp, &bit_cnt_ext);
      if (err_code & IA_FATAL_ERROR)
      {
        bit_cnt_ext = 0;
      }

      pstr_usac_config->usac_config_ext_len[j] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    default:
      break;
    }

    bit_cnt += impeghe_write_escape_value(it_bit_buff, pstr_usac_config->usac_config_ext_type[j],
                                          4, 8, 16);

    bit_cnt += impeghe_write_escape_value(it_bit_buff, pstr_usac_config->usac_config_ext_len[j],
                                          4, 8, 16);

    switch (pstr_usac_config->usac_config_ext_type[j])
    {
    case ID_CONFIG_EXT_FILL:
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, fill_byte_val, 8);
      }
      break;

    case ID_CONFIG_EXT_LOUDNESS_INFO:
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, loudness_byte_val, 8);
      }
      break;
    case ID_CONFIG_EXT_AUDIOSCENE_INFO:
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt += impeghe_write_bits_buf(
            it_bit_buff, (UWORD32)pstr_usac_config->usac_cfg_ext_info_buf[j][i], 8);
      }
      break;
    case ID_CONFIG_EXT_SIG_GROUP_INFO:
    {
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt += impeghe_write_bits_buf(
            it_bit_buff, (UWORD32)pstr_usac_config->usac_cfg_ext_info_buf[j][i], 8);
      }
      break;
    }
    case ID_CONFIG_EXT_HOA_MATRIX:
    {
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt +=
            impeghe_write_bits_buf(it_bit_buff, pstr_usac_config->usac_cfg_ext_info_buf[j][i], 8);
      }
      break;
    }
    default:
      for (i = 0; i < pstr_usac_config->usac_config_ext_len[j]; i++)
      {
        bit_cnt += impeghe_write_bits_buf(
            it_bit_buff, (UWORD32)pstr_usac_config->usac_cfg_ext_info_buf[j][i], 8);
      }
      break;
    }
  }

  return bit_cnt;
}

/**
 *  impeghe_ext_element_config_oam
 *
 *  \brief Writes OAM extension configuration element
 *
 *  \param [in,out]	it_bit_buff			Pointer to bit-buffer
 *  \param [in]		pstr_usac_enc_conf	Pointer to encoder element configuration structure
 *
 *  \return WORD32	Number of bits written
 *
 */
static WORD32
impeghe_ext_element_config_oam(ia_bit_buf_struct *it_bit_buff,
                               ia_usac_enc_element_config_struct *pstr_usac_enc_conf)
{
  WORD32 bit_cnt = 0;
  LOOPIDX idx;

  if (pstr_usac_enc_conf->usac_ext_ele_type == ID_EXT_ELE_OAM)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 1); // low delay
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->oam_has_core_length,
                                      1); // has core length

    if (!pstr_usac_enc_conf->oam_has_core_length)
    {
      bit_cnt += impeghe_write_bits_buf(
          it_bit_buff, (pstr_usac_enc_conf->oam_block_size >> 6) - 1, 6); // frame length
    }

    bit_cnt += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->oam_has_scrn_rel_objs,
                                      1); // screen relative objects

    if (pstr_usac_enc_conf->oam_has_scrn_rel_objs)
    {
      for (idx = 0; idx < pstr_usac_enc_conf->oam_num_objects; idx++)
      {
        bit_cnt +=
            impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->oam_is_scrn_rel_obj[idx],
                                   1); // screen relative objects
      }
    }

    bit_cnt += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->oam_has_dyn_obj_priority,
                                      1); // dyn obj priority

    bit_cnt += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->oam_has_uniform_spread,
                                      1); // uniform spread
  }

  return bit_cnt;
}

/**
 *  impeghe_ext_element_config
 *
 *  \brief Writes external element configurations
 *
 *  \param [in,out]	it_bit_buff			Pointer to bit-buffer
 *  \param [in,out]	pstr_usac_enc_conf	Pointer to encoder element configuration structure
 *
 *  \return WORD32	Number of bits written
 *
 */
static WORD32 impeghe_ext_element_config(ia_bit_buf_struct *it_bit_buff,
                                         ia_usac_enc_element_config_struct *pstr_usac_enc_conf)
{
  WORD32 bit_count = 0;
  UWORD8 bit_buf_base[EXT_ELE_CFG_BIT_BUF_LEN_BYTE] = {0};
  WORD32 cfg_bit_cnt = 0;

  switch (pstr_usac_enc_conf->usac_ext_ele_type)
  {
  case ID_EXT_ELE_OAM:
  {
    ia_bit_buf_struct bit_buf;
    impeghe_create_bit_buffer(&bit_buf, bit_buf_base, sizeof(bit_buf_base));
    bit_buf.impeghe_jmp_buf = it_bit_buff->impeghe_jmp_buf;
    cfg_bit_cnt = impeghe_ext_element_config_oam(&bit_buf, pstr_usac_enc_conf);
    pstr_usac_enc_conf->usac_ext_ele_cfg_len = ((cfg_bit_cnt + 7) >> 3);
  }
  break;
  case ID_EXT_ELE_HOA:
  {
    pstr_usac_enc_conf->usac_ext_ele_cfg_len = ((pstr_usac_enc_conf->hoa_config_len + 7) >> 3);
  }
  break;
  default:
  {
  }
  break;
  }

  bit_count +=
      impeghe_write_escape_value(it_bit_buff, pstr_usac_enc_conf->usac_ext_ele_type, 4, 8, 16);
  bit_count +=
      impeghe_write_escape_value(it_bit_buff, pstr_usac_enc_conf->usac_ext_ele_cfg_len, 4, 8, 16);
  bit_count +=
      impeghe_write_bits_buf(it_bit_buff, (pstr_usac_enc_conf->usac_ext_ele_dflt_len_present), 1);

  if (pstr_usac_enc_conf->usac_ext_ele_dflt_len_present)
  {
    bit_count += impeghe_write_escape_value(
        it_bit_buff, pstr_usac_enc_conf->usac_ext_ele_dflt_len - 1, 8, 16, 0);
  }
  bit_count +=
      impeghe_write_bits_buf(it_bit_buff, (pstr_usac_enc_conf->usac_ext_ele_payload_present), 1);

  switch (pstr_usac_enc_conf->usac_ext_ele_type)
  {
  case ID_EXT_ELE_FILL:
    /* no configuration payload */
    break;
  case ID_EXT_ELE_UNI_DRC:
  {
    UWORD32 i;
    for (i = 0; i < pstr_usac_enc_conf->usac_ext_ele_cfg_len; i++)
    {
      bit_count += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->drc_config_data[i], 8);
    }
  }
  break;
  case ID_EXT_ELE_OAM:
  {
    UWORD32 i;
    for (i = 0; i < pstr_usac_enc_conf->usac_ext_ele_cfg_len; i++)
    {
      bit_count += impeghe_write_bits_buf(it_bit_buff, bit_buf_base[i], 8);
    }
  }
  break;
  case ID_EXT_ELE_HOA:
  {
    UWORD32 i;
    for (i = 0; i < pstr_usac_enc_conf->usac_ext_ele_cfg_len; i++)
    {
      bit_count += impeghe_write_bits_buf(it_bit_buff, pstr_usac_enc_conf->hoa_config_bs[i], 8);
    }
  }
  break;
  default:
  {
    UWORD32 i;
    for (i = 0; i < pstr_usac_enc_conf->usac_ext_ele_cfg_len; i++)
    {
      UWORD32 data = pstr_usac_enc_conf->usac_ext_ele_cfg_payload[i];
      bit_count += impeghe_write_bits_buf(it_bit_buff, data, 8);
      pstr_usac_enc_conf->usac_ext_ele_cfg_payload[i] = 0;
    }
  }
  break;
  }

  return bit_count;
}

/**
 *  impeghe_flex_spk_config
 *
 *  \brief Writes flexible speaker configuration
 *
 *  \param [in,out] it_bit_buff					Pointer to bit-buffer
 *  \param [in] 	pstr_audio_specific_config	Pointer to audio specific configuration
 * structure
 *
 *  \return WORD32	Number of bits written
 */
static WORD32
impeghe_flex_spk_config(ia_bit_buf_struct *it_bit_buff,
                        ia_usac_audio_specific_config_struct *pstr_audio_specific_config)
{
  WORD32 bit_cnt = 0;
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 1); // angular precision is set to 1

  for (WORD32 i = 0; i < pstr_audio_specific_config->num_spk; i++)
  {

    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1); // cicpidx is set to 0
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 3, 2); // elevation calss set as 3
    bit_cnt +=
        impeghe_write_bits_buf(it_bit_buff, ABS(pstr_audio_specific_config->flex_spk_ele[i]), 7);
    if (pstr_audio_specific_config->flex_spk_ele[i] > 0)
    {
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    else if (pstr_audio_specific_config->flex_spk_ele[i] < 0)
    {
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 1);
    }
    bit_cnt +=
        impeghe_write_bits_buf(it_bit_buff, ABS(pstr_audio_specific_config->flex_spk_azi[i]), 8);
    if (pstr_audio_specific_config->flex_spk_azi[i] != 0 &&
        ABS(pstr_audio_specific_config->flex_spk_azi[i]) != 180)
    {
      if (pstr_audio_specific_config->flex_spk_azi[i] > 0)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);
      }
      else if (pstr_audio_specific_config->flex_spk_azi[i] < 0)
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 1);
      }
    }
    bit_cnt +=
        impeghe_write_bits_buf(it_bit_buff, pstr_audio_specific_config->flex_spk_islfe[i], 1);
    if (pstr_audio_specific_config->flex_spk_azi[i] != 0 &&
        ABS(pstr_audio_specific_config->flex_spk_azi[i]) != 180)
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);
  }

  return bit_cnt;
}
/**
 *  impeghe_encoder_config
 *
 *  \brief Writes encoder configuration bits to bit-buffer
 *
 *  \param [in,out]	it_bit_buff		Pointer to bit-buffer
 *  \param [in,out]	pstr_usac_config	Pointer to configuration structure
 *
 *  \return WORD32	Number of bits written
 *
 */
static WORD32 impeghe_encoder_config(ia_bit_buf_struct *it_bit_buff,
                                     ia_usac_config_struct *pstr_usac_config)
{
  WORD32 bit_cnt = 0;
  WORD8 elem_idx = 0;

  bit_cnt +=
      impeghe_write_escape_value(it_bit_buff, pstr_usac_config->num_elements - 1, 4, 8, 16);

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);

  for (elem_idx = 0; elem_idx < pstr_usac_config->num_elements; elem_idx++)
  {
    WORD32 temp;
    ia_usac_enc_element_config_struct str_usac_enc_conf =
        pstr_usac_config->str_usac_element_config[elem_idx];

    unsigned long tmp = pstr_usac_config->usac_element_type[elem_idx];
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, tmp, 2);

    switch (pstr_usac_config->usac_element_type[elem_idx])
    {
    case ID_USAC_SCE:
    case ID_USAC_CPE:
      str_usac_enc_conf =
          pstr_usac_config
              ->str_usac_element_config[elem_idx - pstr_usac_config->num_ext_elements];

      temp = str_usac_enc_conf.tw_mdct;
      temp = (temp << 1) | str_usac_enc_conf.full_band_lpd;
      temp = (temp << 1) | str_usac_enc_conf.noise_filling;
      temp = (temp << 1) | str_usac_enc_conf.enhanced_noise_filling;
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, temp, 4);
      if (str_usac_enc_conf.enhanced_noise_filling == 1)
      {
        temp = str_usac_enc_conf.igf_use_enf;
        temp = (temp << 1) | str_usac_enc_conf.igf_use_high_res;
        temp = (temp << 1) | str_usac_enc_conf.igf_use_whitening;
        temp = (temp << 1) | str_usac_enc_conf.igf_after_tns_synth;

        bit_cnt += impeghe_write_bits_buf(it_bit_buff, temp, 4);
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, (str_usac_enc_conf.igf_start_index), 5);
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, (str_usac_enc_conf.igf_stop_index), 4);

        if (ID_USAC_CPE == pstr_usac_config->usac_element_type[elem_idx])
        {
          bit_cnt +=
              impeghe_write_bits_buf(it_bit_buff, (str_usac_enc_conf.igf_independent_tiling), 1);
        }
      }
      if (ID_USAC_CPE == pstr_usac_config->usac_element_type[elem_idx])
      {
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 3);
        bit_cnt += impeghe_write_bits_buf(it_bit_buff, str_usac_enc_conf.lpd_stereo_idx, 1);
      }
      break;
    case ID_USAC_EXT:
      bit_cnt += impeghe_ext_element_config(it_bit_buff, &(str_usac_enc_conf));
      break;
    case ID_USAC_LFE:
      /* Do nothing */
      break;
    default:
      return -1;
      break;
    }
  }

  return bit_cnt;
}

/**
 *  impeghe_get_audiospecific_config_bytes
 *
 *  \brief Writes audio specific configuration bytes
 *
 *  \param [in,out]	it_bit_buff					Pointer to bit-buffer
 *  \param [in,out]	pstr_scratch				Pointer to scratch memory
 *  \param [in,out]	pstr_audio_specific_config	Pointer to audio specific configuration
 * structure
 *
 *  \return WORD32	Number of bits written
 *
 */
WORD32 impeghe_get_audiospecific_config_bytes(
    ia_bit_buf_struct *it_bit_buff, impeghe_scratch_mem *pstr_scratch,
    ia_usac_audio_specific_config_struct *pstr_audio_specific_config)
{
  WORD32 bit_cnt = 0, i;
  ia_usac_config_struct *ptr_usac_config = &(pstr_audio_specific_config->str_usac_config);
  WORD32 num_signal_groups = pstr_audio_specific_config->num_sig_grps;

  WORD32 group_type[16] = {0}, idx = 0, hoa_idx = 0, obj_idx = 0;
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, pstr_audio_specific_config->profile_info, 8);

  bit_cnt +=
      impeghe_write_bits_buf(it_bit_buff, (pstr_audio_specific_config->samp_frequency_index), 5);

  if (pstr_audio_specific_config->samp_frequency_index == 0x1f)
    bit_cnt +=
        impeghe_write_bits_buf(it_bit_buff, (pstr_audio_specific_config->sampling_frequency), 24);
  else
  {
    pstr_audio_specific_config->sampling_frequency =
        impeghe_sampl_freq_table[pstr_audio_specific_config->samp_frequency_index];
  }

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 3); // core sbr frame length

  for (i = 0; i < pstr_audio_specific_config->num_ch_sig_groups; i++)
  {
    if (pstr_audio_specific_config->num_ch_per_sig_group[i] > 0)
    {
      group_type[idx++] = 0;
    }
  }
  for (i = 0; i < pstr_audio_specific_config->num_obj_sig_groups; i++)
  {
    if (pstr_audio_specific_config->num_objs_per_sig_group[i] > 0)
    {
      group_type[idx++] = 1;
    }
  }
  for (i = 0; i < pstr_audio_specific_config->num_hoa_sig_groups; i++)
  {
    if (pstr_audio_specific_config->num_hoas_per_sig_group[i] > 0)
    {
      group_type[idx++] = 3;
    }
  }

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);

  if (num_signal_groups > 1)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 1, 1);
  }
  else
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);
  }

  if (!pstr_audio_specific_config->flex_spk_enable)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 2);

    bit_cnt +=
        impeghe_write_bits_buf(it_bit_buff, pstr_audio_specific_config->channel_configuration, 6);
  }
  /*speakerLayout type when 2*/
  else
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, 2, 2);
    bit_cnt += impeghe_write_escape_value(it_bit_buff, pstr_audio_specific_config->num_spk - 1, 5,
                                          8, 16);
    bit_cnt += impeghe_flex_spk_config(it_bit_buff, pstr_audio_specific_config);
  }
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, num_signal_groups - 1, 5);
  idx = 0;
  for (i = 0; i < num_signal_groups; i++)
  {
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, group_type[i], 3);

    switch (group_type[i])
    {
    case 0:
      bit_cnt += impeghe_write_escape_value(
          it_bit_buff, pstr_audio_specific_config->num_ch_per_sig_group[idx++] - 1, 5, 8, 16);
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, 0, 1);

      break;
    case 1:
      bit_cnt += impeghe_write_escape_value(
          it_bit_buff, pstr_audio_specific_config->num_objs_per_sig_group[obj_idx++] - 1, 5, 8,
          16);

      break;
    case 2:
      break;
    case 3:
      if (0 < pstr_audio_specific_config->num_hoa_transport_channels)
      {
        bit_cnt += impeghe_write_escape_value(
            it_bit_buff, pstr_audio_specific_config->num_hoas_per_sig_group[hoa_idx++] - 1, 5, 8,
            16);
      }
      break;
    }
  }
  if (pstr_audio_specific_config->flex_spk_enable)
  {
    ptr_usac_config->usac_cfg_ext_present = 1;
    ptr_usac_config->num_config_extensions = 1;
  }
  bit_cnt += impeghe_encoder_config(it_bit_buff, ptr_usac_config);

  bit_cnt += impeghe_write_bits_buf(it_bit_buff, (ptr_usac_config->usac_cfg_ext_present), 1);

  if (ptr_usac_config->usac_cfg_ext_present)
  {
    ptr_usac_config->num_signal_grp = num_signal_groups;
    bit_cnt += impeghe_config_extension(it_bit_buff, ptr_usac_config, pstr_scratch,
                                        &pstr_audio_specific_config->str_asi_info);
  }

  pstr_audio_specific_config->ext_audio_object_type = 0;

  return bit_cnt;
}
