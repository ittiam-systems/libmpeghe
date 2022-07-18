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

#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_error_standards.h"
#include "impeghe_apicmd_standards.h"
#include "impeghe_error_codes.h"

#include "impeghe_block_switch_const.h"

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
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_mhas_write.h"

/**
 *  impeghe_write_mhas_pkt_header
 *
 *  \brief Writes MHAS packet header
 *
 *  \param [in]  pkt_info		Pointer to MHAS packet information structure
 *  \param [out] it_bit_buff	Pointer to bit-buffer structure
 *
 *  \return WORD32	Number of bits written
 */
static WORD32 impeghe_write_mhas_pkt_header(ia_mhas_pac_info *pkt_info,
                                            ia_bit_buf_struct *it_bit_buff)
{
  WORD32 bit_cnt = 0;

  bit_cnt = impeghe_write_escape_value(it_bit_buff, pkt_info->packet_type, 3, 8, 8);
  bit_cnt += impeghe_write_escape_value(it_bit_buff, pkt_info->packet_lbl, 2, 8, 32);
  bit_cnt += impeghe_write_escape_value(it_bit_buff, pkt_info->packet_length, 11, 24, 24);

  return bit_cnt;
}

/**
 *  impeghe_write_mhas_pkt
 *
 *  \brief Writes MHAS packet data
 *
 *  \param [out] it_bit_buff	Pointer to bit-buffer structure
 *  \param [in]  pkt_info		Pointer to MHAS packet information structure
 *  \param [in]  data			Pointer to packet data
 *  \param [in]  num_bits		Number of bit to be written
 *
 *  \return WORD32	Number of bits written
 */
static WORD32 impeghe_write_mhas_pkt(ia_bit_buf_struct *it_bit_buff, ia_mhas_pac_info *pkt_info,
                                     UWORD8 *data, UWORD32 num_bits)
{
  UWORD8 val;
  WORD32 bit_cnt, i;

  pkt_info->packet_length = (num_bits + 7) >> 3;

  bit_cnt = impeghe_write_mhas_pkt_header(pkt_info, it_bit_buff);

  if (data != NULL)
  {
    for (i = 0; i < pkt_info->packet_length; i++)
    {
      val = data[i];
      impeghe_write_bits_buf(it_bit_buff, val, 8);
      bit_cnt += 8;
    }
  }

  return bit_cnt;
}

/**
 *  impeghe_write_crc_pkt
 *
 *  \brief Write CRC packet in bit stream
 *
 *  \param [out]    it_bit_buff   Pointer to bit-buffer structure
 *  \param [in]     pkt_info      Pointer to MHAS Packet information structure
 *  \param [in]     data          CRC value
 *  \param [in]     num_bits      Num of bits for CRC Value
 *
 *  \return WORD32  Number of bits written
 *
 */
static WORD32 impeghe_write_crc_pkt(ia_bit_buf_struct *it_bit_buff, ia_mhas_pac_info *pkt_info,
                                    UWORD32 data, UWORD32 num_bits)
{
  WORD32 bit_cnt;

  pkt_info->packet_length = (num_bits + 7) >> 3;

  bit_cnt = impeghe_write_mhas_pkt_header(pkt_info, it_bit_buff);

  if (num_bits == 16)
  {
    UWORD16 val = (UWORD16)(data);
    impeghe_write_bits_buf(it_bit_buff, val, 16);
    bit_cnt += 16;
  }
  else if (num_bits == 32)
  {
    UWORD32 val = (UWORD32)(data);
    impeghe_write_bits_buf(it_bit_buff, val, 32);
    bit_cnt += 32;
  }
  return bit_cnt;
}

/**
 *  impeghe_write_pcm_data_pkt
 *
 *  \brief Writes PCM data
 *
 *  \param [out] it_bit_buff			Pointer to bit-buffer structure
 *  \param [in]  ptr_pcm_data_config	Pointer to PCM data configuration structure
 *
 *  \return WORD32
 */
WORD32 impeghe_write_pcm_data_pkt(ia_bit_buf_struct *it_bit_buff,
                                  ia_pcm_data_config *ptr_pcm_data_config)
{

  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->bsnum_pcm_signals_in_frame, 7);
  for (WORD32 i = 0; i < ptr_pcm_data_config->bsnum_pcm_signals_in_frame + 1; i++)
  {
    impeghe_write_bits_buf(it_bit_buff, 0, 7);
  }
  for (WORD32 i = 0; i < 1024; i++)
  {
    impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_sample[i],
                           ptr_pcm_data_config->pcm_bits_per_sample);
  }
  return 0;
}

/**
 *  impeghe_write_pcm_config_pkt
 *
 *  \brief Writes PCM config information
 *
 *  \param [out] it_bit_buff			Pointer to bit-buffer structure
 *  \param [in]  ptr_pcm_data_config	Pointer to PCM data configuration structure
 *
 *  \return WORD32
 */
WORD32 impeghe_write_pcm_config_pkt(ia_bit_buf_struct *it_bit_buff,
                                    ia_pcm_data_config *ptr_pcm_data_config)
{

  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->bs_num_pcm_signals, 7);
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_align_audio_flag, 1);
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_sampling_rate_idx, 5);
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_bits_per_sample_idx, 3);
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_frame_size_idx, 3);
  if (ptr_pcm_data_config->pcm_frame_size_idx == 5)
  {
    impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_fix_frame_size, 16);
  }
  for (WORD32 i = 0; i < ptr_pcm_data_config->bs_num_pcm_signals + 1; i++)
  {
    impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_signal_id[i], 7);
  }
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->bs_pcm_loudness_value, 8);
  impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->pcm_has_attenuation_gain, 2);
  if (ptr_pcm_data_config->pcm_has_attenuation_gain)
  {
    impeghe_write_bits_buf(it_bit_buff, ptr_pcm_data_config->bs_pcm_attenuation_gain, 8);
  }
  return 0;
}

/**
 *  impeghe_write_ec_pkt
 *
 *  \brief Writes EC packet
 *
 *  \param [out] 	 it_bit_buff			Pointer to bit-buffer structure
 *  \param [in,out]  ptr_ec_config_struct	Pointer to EC configuration structure
 *
 *  \return WORD32
 */
WORD32 impeghe_write_ec_pkt(ia_bit_buf_struct *it_bit_buff,
                            ia_ec_config_struct *ptr_ec_config_struct)
{
  ptr_ec_config_struct->position_type = 0;
  ptr_ec_config_struct->elevation = 0;
  ptr_ec_config_struct->azimuth = 0;
  impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->num_earcons, 7); // no:of earcons
  for (WORD32 i = 0; i < ptr_ec_config_struct->num_earcons + 1; i++)
  {
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->is_independent, 1);
    impeghe_write_bits_buf(it_bit_buff, 0, 7); // ear con id need to add to structure
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->type, 4);
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->active, 1);
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->position_type, 2);
    if (ptr_ec_config_struct->position_type == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->cicp_idx, 7);
    }
    else if (ptr_ec_config_struct->position_type == 1)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->azimuth, 8);
      impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->elevation, 6);
      impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->distance, 9);
    }
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->has_gain, 1);
    if (ptr_ec_config_struct->has_gain)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->gain, 7);
    }
    impeghe_write_bits_buf(it_bit_buff, ptr_ec_config_struct->has_text_label, 1);
  }
  return 0;
}

/**
 *  impeghe_mhas_write_earcon_header
 *
 *  \brief Writes EARCON header
 *
 *  \param [out] it_bit_buff	Pointer to bit-buffer structure
 *  \param [in]  pkt_type		Packet type
 *  \param [in]  num_bits		Number of bits
 *
 *  \return WORD32	Number of bits written
 */
WORD32 impeghe_mhas_write_earcon_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                        UWORD32 num_bits)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;
  UWORD8 mhas_sync_word = 0xA5;
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_SYNC;
  pkt_info_tmp.packet_lbl = 0;
  bit_cnt = impeghe_write_mhas_pkt(it_bit_buff, &pkt_info_tmp, &mhas_sync_word, 8);
  /* write MHAS_PAC_TYP_EARCON */
  pkt_info_tmp.packet_type = pkt_type;
  pkt_info_tmp.packet_lbl = 0;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;

  bit_cnt += impeghe_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}

/**
 *  impeghe_mhas_write_crc_header
 *
 *  \brief Write CRC Header in bit stream
 *
 *  \param [out]    it_bit_buff   Pointer to bit-buffer structure
 *  \param [in]     pkt_type      CRC MHAS Packet Type
 *  \param [in]     crc_len       Num of bits for CRC Value
 *  \param [in]     crc_val       CRC value
 *
 *  \return WORD32  Number of bits written
 *
 */
WORD32 impeghe_mhas_write_crc_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                     WORD32 crc_len, UWORD32 crc_val)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;
  pkt_info_tmp.packet_type = pkt_type;
  pkt_info_tmp.packet_lbl = 1;
  bit_cnt = impeghe_write_crc_pkt(it_bit_buff, &pkt_info_tmp, (crc_val), crc_len);
  return bit_cnt;
}

/**
 *  impeghe_mhas_write_global_crc_header
 *
 *  \brief Write Global CRC Header in bit stream
 *
 *  \param [out]    it_bit_buff             Pointer to bit-buffer structure
 *  \param [in]     pkt_type                CRC MHAS Packet Type
 *  \param [in]     crc_len                 Num of bits for CRC Value
 *  \param [in]     crc_val                 CRC value
 *  \param [in]     global_crc_type         Global CRC Type
 *  \param [in]     num_protected_packets   Number of Protected Packets by Global CRC
 *
 *  \return WORD32  Number of bits written
 *
 */
WORD32 impeghe_mhas_write_global_crc_header(ia_bit_buf_struct *it_bit_buff, WORD32 pkt_type,
                                            WORD32 crc_len, UWORD32 crc_val,
                                            UWORD32 global_crc_type,
                                            UWORD32 num_protected_packets)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;
  pkt_info_tmp.packet_type = pkt_type;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length =
      (crc_len + 8 + 7) >> 3; // 8 bits for global_crc_type and num_protected_packets

  bit_cnt = impeghe_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);
  impeghe_write_bits_buf(it_bit_buff, global_crc_type, 2);
  impeghe_write_bits_buf(it_bit_buff, num_protected_packets, 6);
  bit_cnt += 8;
  if (crc_len == 16)
  {
    UWORD16 val = (UWORD16)(crc_val);
    impeghe_write_bits_buf(it_bit_buff, val, 16);
    bit_cnt += 16;
  }
  else if (crc_len == 32)
  {
    UWORD32 val = (UWORD32)(crc_val);
    impeghe_write_bits_buf(it_bit_buff, val, 32);
    bit_cnt += 32;
  }
  return bit_cnt;
}

/**
 *  impeghe_mhas_write_sync_header
 *
 *  \brief Write MHAS Sync Header
 *
 *  \param [out]    it_bit_buff   Pointer to bit-buffer structure
 *
 *  \return WORD32  Number of bits written
 *
 */
WORD32 impeghe_mhas_write_sync_header(ia_bit_buf_struct *it_bit_buff)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_SYNC PKT */
  UWORD8 mhas_sync_word = 0xA5;
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_SYNC;
  pkt_info_tmp.packet_lbl = 0;
  pkt_info_tmp.packet_length = 1;

  bit_cnt = impeghe_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);
  impeghe_write_bits_buf(it_bit_buff, mhas_sync_word, 8);
  bit_cnt += 8;

  return bit_cnt;
}

/**
 *  impeghe_mhas_write_cfg_only_header
 *
 *  \brief Write only MHAS Config Header
 *
 *  \param [out]    it_bit_buff   Pointer to bit-buffer structure
 *  \param [in]     num_bits      Number of bits of config data for packet length
 *
 *  \return WORD32  Number of bits written
 *
 */
WORD32 impeghe_mhas_write_cfg_only_header(ia_bit_buf_struct *it_bit_buff, UWORD32 num_bits)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_MPEGH3DACFG */
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_MPEGH3DACFG;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;

  bit_cnt = impeghe_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}

/**
 *  impeghe_mhas_write_frame_header
 *
 *  \brief Writes MHAS frame header
 *
 *  \param [out] it_bit_buff Pointer to bit-buffer structure
 *  \param [in]  num_bits	 Number of bits to be written
 *
 *  \return WORD32 Number of bits written
 */
WORD32 impeghe_mhas_write_frame_header(ia_bit_buf_struct *it_bit_buff, UWORD32 num_bits)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_MPEGH3DAFRAME */
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_MPEGH3DAFRAME;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;
  bit_cnt += impeghe_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}
