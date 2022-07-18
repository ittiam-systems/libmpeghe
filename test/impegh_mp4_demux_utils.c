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
#include "impegh_mp4_demux_utils.h"
#include "impegh_demux_error.h"
#include "impegh_demux_define.h"

/**impegh_create_bit_buffer
 *
 *  \brief Creates and initializes the bit-buffer using given base pointer, size and init flag
 *
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer structure
 *  \param [in]		ptr_bit_buf_base	Pointer to bit-buffer base
 *  \param [in]		bit_buffer_size		Size of bit buffer to be created
 *  \param [in]		init				Initialization flag
 *
 *  \return ia_bit_buf_struct
 */
ia_bit_buf_struct_d *impegh_create_bit_buffer(ia_bit_buf_struct_d *it_bit_buf,
                                              UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size,
                                              WORD32 init)
{
  it_bit_buf->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buf->ptr_bit_buf_end = ptr_bit_buf_base + bit_buffer_size - 1;
  it_bit_buf->ptr_read_next = ptr_bit_buf_base;
  it_bit_buf->ptr_write_next = ptr_bit_buf_base;

  if (init)
  {
    it_bit_buf->write_position = 7;
    it_bit_buf->read_position = 7;
    it_bit_buf->cnt_bits = 0;
    it_bit_buf->size = bit_buffer_size * 8;
  }

  return (it_bit_buf);
}
/**impegh_create_read_buf
 *
 *
 *  \brief Initialize bit buffer structure entries.
 *
 *  \param [i/o] it_bit_buff      Pointer to bit buffer structure.
 *  \param [in]  ptr_bit_buf_base Pointer to bit buffer base.
 *  \param [in]  bit_buf_size     Pointer to bit buffer size.
 *
 *  \return ia_bit_buf_struct * Pointer to bit buffer structure

 *
 */
ia_bit_buf_struct_e *impegh_create_read_buf(ia_bit_buf_struct_e *it_bit_buff,
                                            UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size)
{
  it_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buff->ptr_bit_buf_end = ptr_bit_buf_base + bit_buf_size - 1;
  it_bit_buff->ptr_read_next = ptr_bit_buf_base;
  it_bit_buff->bit_pos = 7;

  it_bit_buff->cnt_bits = (bit_buf_size << 3);
  it_bit_buff->size = bit_buf_size << 3;

  it_bit_buff->error = 0;
  it_bit_buff->max_size = it_bit_buff->size;

  return it_bit_buff;
}

/**impegh_write_bits_buf
 *
 *  \brief Writes bits into bit-buffer
 *
 *  \param [in,out]	it_bit_buf	Pointer to bit-buffer structure
 *  \param [in]		write_val	Value to be written into bit-buffer
 *  \param [in]		num_bits	Number of bits to be written
 *
 *  \return UWORD8
 */
UWORD8 impegh_write_bits_buf(ia_bit_buf_struct_d *it_bit_buf, UWORD32 write_val, UWORD8 num_bits)
{
  WORD32 bits_to_write;
  WORD32 write_position;
  UWORD8 *ptr_write_next;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_bit_buf_base;

  UWORD8 bits_written = num_bits;
  if (it_bit_buf)
  {
    it_bit_buf->cnt_bits += num_bits;

    write_position = it_bit_buf->write_position;
    ptr_write_next = it_bit_buf->ptr_write_next;
    ptr_bit_buf_end = it_bit_buf->ptr_bit_buf_end;
    ptr_bit_buf_base = it_bit_buf->ptr_bit_buf_base;

    while (num_bits)
    {
      UWORD8 tmp, msk;

      bits_to_write = MIN(write_position + 1, num_bits);

      tmp = (UWORD8)(write_val << (32 - num_bits) >> (32 - bits_to_write)
                                                         << (write_position + 1 - bits_to_write));

      msk = ~(((1 << bits_to_write) - 1) << (write_position + 1 - bits_to_write));

      *ptr_write_next &= msk;
      *ptr_write_next |= tmp;

      write_position -= bits_to_write;

      num_bits -= bits_to_write;

      if (write_position < 0)
      {

        write_position += 8;
        ptr_write_next++;

        if (ptr_write_next > ptr_bit_buf_end)
        {

          ptr_write_next = ptr_bit_buf_base;
        }
      }
    }

    it_bit_buf->write_position = write_position;
    it_bit_buf->ptr_write_next = ptr_write_next;
  }

  return (bits_written);
}
/**impegh_mhas_write_maei_header
 *
 *
 *  \brief Writes the maei header to the mhas packet
 *
 *  \param [in,out] it_bit_buf Pointer to the bit buffer structure
 *  \param data [out] Pointer to the data to be written
 *  \param [in] num_bits Number of bits to be written
 *
 *  \return WORD32 return the number of bits written
 *
 */
WORD32 impegh_mhas_write_maei_header(ia_bit_buf_struct_d *it_bit_buff, UWORD8 *data,
                                     UWORD32 num_bits)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_SYNC PKT */
  UWORD8 mhas_sync_word = 0xA5;
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_SYNC;
  pkt_info_tmp.packet_lbl = 0;
  bit_cnt = impegh_write_mhas_pkt(it_bit_buff, &pkt_info_tmp, &mhas_sync_word, 8);

  /* write MHAS_PAC_TYP_AUDIOSCENEINFO */
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_AUDIOSCENEINFO;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;

  bit_cnt += impegh_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}

/**impegh_mhas_write_cfg_header
 *
 *  \brief Write mhas config header
 *
 *  \param [in,out] it_bit_buff	Pointer to bit-buffer structure
 *  \param [in] data	Data to write
 *  \param [in] num_bits	Number of bits to be written
 *
 *  \return WORD32
 */
WORD32 impegh_mhas_write_cfg_header(ia_bit_buf_struct_d *it_bit_buff, UWORD8 *data,
                                    UWORD32 num_bits)

{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_SYNC PKT */
  UWORD8 mhas_sync_word = 0xA5;
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_SYNC;
  pkt_info_tmp.packet_lbl = 0;
  bit_cnt = impegh_write_mhas_pkt(it_bit_buff, &pkt_info_tmp, &mhas_sync_word, 8);

  /* write MHAS_PAC_TYP_MPEGH3DACFG */
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_MPEGH3DACFG;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;

  bit_cnt += impegh_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}

/**impegh_mhas_write_frame_header
 *
 *  \brief Write mhas frame header
 *
 *  \param [in,out] it_bit_buff	Pointer to ia_bit_buf_struct_d structure
 *  \param [in] num_bits	Number of bits written
 *
 *  \return WORD32
 */
WORD32 impegh_mhas_write_frame_header(ia_bit_buf_struct_d *it_bit_buff, UWORD32 num_bits)
{
  WORD32 bit_cnt = 0;
  ia_mhas_pac_info pkt_info_tmp;

  /* write MHAS_PAC_TYP_MPEGH3DAFRAME */
  pkt_info_tmp.packet_type = MHAS_PAC_TYP_MPEGH3DAFRAME;
  pkt_info_tmp.packet_lbl = 1;
  pkt_info_tmp.packet_length = (num_bits + 7) >> 3;
  bit_cnt += impegh_write_mhas_pkt_header(&pkt_info_tmp, it_bit_buff);

  return bit_cnt;
}

/**impegh_mp4_find_stsz
 *
 *  \brief Function to find stsz
 *
 *  \param [in] itf	Pointer to it_avi_file_ctxt structure
 *  \param [in] offset	pointer to offset
 *  \param [in] stsz_size Pointer to stsz size
 *
 *  \return IA_ERRORCODE
 */

IA_ERRORCODE impegh_mp4_find_stsz(it_avi_file_ctxt *itf, WORD32 *offset, WORD32 *stsz_size)
{
  WORD8 buf_size[4];
  WORD8 buf_test;
  WORD32 bytes_test;
  UWORD32 *data_size = (UWORD32 *)&buf_size[0];
  *stsz_size = 0;
  while (!feof(itf->fp))
  {
    bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
    if (buf_test == 's')
    {
      bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
      if (buf_test == 't')
      {
        bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 's')
          bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 'z')
        {
          impegh_mp4_fseek(itf, -8, SEEK_CUR);
          bytes_test = impegh_mp4_fread(buf_size, 1, 4, itf);
          if (bytes_test < 4)
          {
            *stsz_size = -1;
            return IMPEGH_DEMUX_STSZ_FIND;
          }

          *stsz_size = impegh_mp4_rev32(*data_size);
          bytes_test = impegh_mp4_fread(buf_size, 1, 4, itf);
          *offset = ftell(itf->fp);
          *stsz_size = *stsz_size - 8;
          return IA_NO_ERROR;
        }
      }
      if (buf_test == 'm')
        impegh_mp4_fseek(itf, -1, SEEK_CUR);
    }
  }
  return IA_NO_ERROR;
}

/**impegh_write_mhas_pkt_header
 *
 *  \brief Function to write mhas packet header
 *
 *  \param [in] pkt_info Pointer to ia_mhas_pac_info structure
 *  \param [out] it_bit_buff	Pointer to ia_bit_buf_struct_d structure
 *
 *  \return WORD32
 */
WORD32 impegh_write_mhas_pkt_header(ia_mhas_pac_info *pkt_info, ia_bit_buf_struct_d *it_bit_buff)
{
  WORD32 bit_cnt = 0;

  bit_cnt = impegh_write_escape_value(it_bit_buff, pkt_info->packet_type, 3, 8, 8);
  bit_cnt += impegh_write_escape_value(it_bit_buff, pkt_info->packet_lbl, 2, 8, 32);
  bit_cnt += impegh_write_escape_value(it_bit_buff, pkt_info->packet_length, 11, 24, 24);

  return bit_cnt;
}

/**impegh_write_mhas_pkt
 *
 *  \brief Function to write mhas packet
 *
 *  \param [in,out] it_bit_buff	Pointer to ia_bit_buf_struct_d structure
 *  \param [in,out] pkt_info	Pointer tp ia_mhas_pac_info structure
 *  \param [in] data	Pointer to data to be written
 *  \param [in] num_bits Number of bits for packet lengeth
 *
 *  \return WORD32
 */
WORD32 impegh_write_mhas_pkt(ia_bit_buf_struct_d *it_bit_buff, ia_mhas_pac_info *pkt_info,
                             UWORD8 *data, UWORD32 num_bits)
{
  UWORD8 val;
  WORD32 bit_cnt, i;

  pkt_info->packet_length = (num_bits + 7) >> 3;

  bit_cnt = impegh_write_mhas_pkt_header(pkt_info, it_bit_buff);

  if (data != NULL)
  {
    for (i = 0; i < pkt_info->packet_length; i++)
    {
      val = data[i];
      impegh_write_bits_buf(it_bit_buff, val, 8);
      bit_cnt += 8;
    }
  }

  return bit_cnt;
}
/**impegh_read_bits_buf
 *
 *
 *  \brief Helper function to read bits.
 *
 *  \param [i/o] it_bit_buff Pointer to bit buffer structure.
 *  \param [in]  no_of_bits  Pointer to no of bits to be read.
 *
 *  \return WORD32 Value read from bit stream.
 *
 */

WORD32 impegh_read_bits_buf(ia_bit_buf_struct_e *it_bit_buff, WORD no_of_bits)
{
  UWORD32 ret_val;
  UWORD8 *ptr_read_next = it_bit_buff->ptr_read_next;
  WORD bit_pos = it_bit_buff->bit_pos; // 7

  if (no_of_bits == 0)
  {
    return 0;
  }

  it_bit_buff->cnt_bits -= no_of_bits;
  ret_val = (UWORD32)*ptr_read_next;

  bit_pos -= no_of_bits;
  while (bit_pos < -1)
  {
    bit_pos += 8;
    ptr_read_next++;

    ret_val <<= 8;

    ret_val |= (UWORD32)*ptr_read_next;
  }

  if (bit_pos == -1)
  {
    bit_pos += 8;
    ptr_read_next++;
    ret_val <<= 8;
  }

  ret_val = ret_val << ((31 - no_of_bits) - bit_pos) >> (32 - no_of_bits);
  it_bit_buff->ptr_read_next = ptr_read_next;
  it_bit_buff->bit_pos = (WORD16)bit_pos;
  return ret_val;
}
/**impegh_write_escape_value
 *
 *  \brief Function to write escape value
 *
 *  \param [in,out] it_bit_buff Pointer to ia_bit_buf_struct_d structure
 *  \param [in] value value to calculate escape value
 *  \param [in] no_bits1	Number of bits - first level
 *  \param [in] no_bits2	Number of bits - second level
 *  \param [in] no_bits3	Number of bits - third level
 *
 *  \return WORD32
 */
WORD32 impegh_write_escape_value(ia_bit_buf_struct_d *it_bit_buff, UWORD32 value,
                                 UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3)
{

  WORD32 bit_cnt = 0;
  UWORD32 esc_val = 0;
  UWORD32 max_val1 = ((UWORD32)1 << no_bits1) - 1;
  UWORD32 max_val2 = ((UWORD32)1 << no_bits2) - 1;
  UWORD32 max_val3 = ((UWORD32)1 << no_bits3) - 1;

  esc_val = MIN(value, max_val1);
  bit_cnt += impegh_write_bits_buf(it_bit_buff, esc_val, no_bits1);

  if (esc_val == max_val1)
  {
    value = value - esc_val;

    esc_val = MIN(value, max_val2);
    bit_cnt += impegh_write_bits_buf(it_bit_buff, esc_val, no_bits2);

    if (esc_val == max_val2)
    {
      value = value - esc_val;

      esc_val = MIN(value, max_val3);
      bit_cnt += impegh_write_bits_buf(it_bit_buff, esc_val, no_bits3);
    }
  }

  return bit_cnt;
}
/**impegh_mae_write_description_data
 *
 *
 *  \brief Writes the group decription data elements
 *
 *  \param [in,out] ptr_bit_buf Pointer to the bit buffer handle
 *  \param [in,out] ptr_description_data Pointer to the description data structure
 *  \param [in] group_id_bits Number of group id bits
 *
 *  \return VOID
 *
 */
static VOID impegh_mae_write_description_data(ia_bit_buf_struct_d *ptr_bit_buf,
                                              ia_description_data *ptr_description_data,
                                              WORD32 group_id_bits, WORD32 *ptr_bit_cnt)
{
  WORD32 n, num_descr_blocks, tmp, i, c, bits_written = 0;
  num_descr_blocks = ptr_description_data->num_desc_blocks;
  bits_written += impegh_write_bits_buf(ptr_bit_buf, num_descr_blocks, 7);

  for (n = 0; n < num_descr_blocks + 1; n++)
  {
    tmp = ptr_description_data->group_id[n];
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, group_id_bits);

    tmp = ptr_description_data->num_descr_languages[n];
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 4);

    for (i = 0; i < ptr_description_data->num_descr_languages[n] + 1; i++)
    {
      tmp = ptr_description_data->descr_language[n][i];
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 24);

      tmp = ptr_description_data->descr_data_length[n][i];
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 8);

      for (c = 0; c < ptr_description_data->descr_data_length[n][i] + 1; c++)
      {
        tmp = ptr_description_data->descr_data[n][i][c];
        bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 8);
      }
    }
  }
  *ptr_bit_cnt = bits_written;
}

/**impegh_mp4_write_mael_buf
 *
 *
 *  \brief Writes the mael elements from the buffer
 *
 *  \param [in,out] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in,out] ptr_mae_audio_scene_info Pointer to the mae audio scene structure
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_write_mael_buf(ia_bit_buf_struct_d *ptr_bit_buf,
                                 ia_mae_audio_scene_info *ptr_mae_asi)
{
  WORD32 i = 0, data_type, j, bits_written = 0;
  WORD32 num_data_sets = 0;
  WORD32 bit_cnt_ext;
  ia_bit_buf_struct_d it_bit_buf_local;
  ia_mae_data *ptr_mae_data = &ptr_mae_asi->mae_data;
  num_data_sets = ptr_mae_data->num_data_sets;
  bits_written += impegh_write_bits_buf(ptr_bit_buf, num_data_sets, 4);
  for (i = 0; i < num_data_sets; i++)
  {
    data_type = ptr_mae_data->data_type[i];
    switch (data_type)
    {
    case ID_MAE_GROUP_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impegh_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                               sizeof(ptr_mae_asi->mae_info_buf[i]), 1);
      impegh_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->group_desc_data, 7,
                                        &bit_cnt_ext);

      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_SWITCHGROUP_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impegh_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                               sizeof(ptr_mae_asi->mae_info_buf[i]), 1);
      impegh_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->switch_group_desc_data,
                                        5, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_GROUP_PRESET_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impegh_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                               sizeof(ptr_mae_asi->mae_info_buf[i]), 1);
      impegh_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->preset_desc_data, 5,
                                        &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    default:
      break;
    }
    bits_written += impegh_write_bits_buf(ptr_bit_buf, data_type, 4);
    bits_written += impegh_write_bits_buf(ptr_bit_buf, ptr_mae_data->data_length[i], 16);
    for (j = 0; j < ptr_mae_data->data_length[i]; j++)
    {
      bits_written +=
          impegh_write_bits_buf(ptr_bit_buf, (UWORD32)ptr_mae_asi->mae_info_buf[i][j], 8);
    }
  }
  return bits_written;
}
/**impegh_mp4_write_maep_buf
 *
 *
 *  \brief writes the maep elements from the buffer
 *
 *  \param [in,out] ptr_bit_buf Poiter to the bit buffer structure
 *  \param [in,out] ptr_group_presets_definition Pointer to the group preset definition structure
 *  \param [in] num_groups Number of preset groups
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_write_maep_buf(ia_bit_buf_struct_d *ptr_bit_buf,
                                 ia_mae_group_presets_def *ptr_group_presets_definition,
                                 WORD32 num_group_presets)
{
  WORD32 i, tmp, j, num_conditions, bits_written = 0;
  for (i = 0; i < num_group_presets; i++)
  {
    tmp = ptr_group_presets_definition[i].group_id;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

    tmp = ptr_group_presets_definition[i].preset_kind;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

    num_conditions = ptr_group_presets_definition[i].num_conditions;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, num_conditions, 4);

    for (j = 0; j < num_conditions + 1; j++)
    {
      tmp = ptr_group_presets_definition[i].reference_id[j];
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);

      tmp = ptr_group_presets_definition[i].cond_on_off[j];
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

      if (tmp)
      {
        tmp = ptr_group_presets_definition[i].disable_gain_interact[j];
        bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

        tmp = ptr_group_presets_definition[i].gain_flag[j];
        bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);
        if (tmp)
        {
          tmp = ptr_group_presets_definition[i].gain[j];
          bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 8);
        }
        tmp = ptr_group_presets_definition[i].disable_pos_interact[j];
        bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

        tmp = ptr_group_presets_definition[i].position_interact[j];
        bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

        if (tmp)
        {
          tmp = ptr_group_presets_definition[i].azimuth_offset[j];
          bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 8);

          tmp = ptr_group_presets_definition[i].elevation_offset[j];
          bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 6);

          tmp = ptr_group_presets_definition[i].dist_factor[j];
          bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 4);
        }
      }
    }
  }
  return bits_written;
}
/**impegh_mp4_write_maes_buf
 *
 *
 *  \brief Writes the maes elements from the buffer
 *
 *  \param [in,out] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in,out]  ptr_switch_group_definition Pointer to switch group definition structure
 *  \param [in] num_groups Number of switch groups
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_write_maes_buf(ia_bit_buf_struct_d *ptr_bit_buf,
                                 ia_mae_switch_group_def *ptr_switch_group_definition,
                                 WORD32 num_switch_groups)
{
  WORD32 i, j, tmp, bits_written = 0;
  for (i = 0; i < num_switch_groups; i++)
  {
    tmp = ptr_switch_group_definition[i].group_id;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

    tmp = ptr_switch_group_definition[i].allow_on_off;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_switch_group_definition[i].default_on_off;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);
    }

    tmp = ptr_switch_group_definition[i].group_num_members;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

    for (j = 0; j < ptr_switch_group_definition[i].group_num_members + 1; j++)
    {
      tmp = ptr_switch_group_definition[i].member_id[j];
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);
    }
    tmp = ptr_switch_group_definition[i].default_group_id;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);
  }
  return bits_written;
}
/**impegh_mp4_write_maeg_buf
 *
 *
 *  \brief Writes the maeg elements from the buffer
 *
 *  \param [in,out] ptr_bit_buf Pointer to bit buffer handle
 *  \param [in,out] ptr_group_definition Pointer to group definition structure
 *  \param [in] num_groups Number of groups
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_write_maeg_buf(ia_bit_buf_struct_d *ptr_bit_buf,
                                 ia_mae_group_def *ptr_group_definition, WORD32 num_groups)
{
  WORD32 i, tmp, bits_written = 0;
  for (i = 0; i < num_groups; i++)
  {
    tmp = ptr_group_definition[i].group_id;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);

    tmp = ptr_group_definition[i].allow_on_off;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    tmp = ptr_group_definition[i].default_on_off;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    tmp = ptr_group_definition[i].allow_pos_interact;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].min_az_offset;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);

      tmp = ptr_group_definition[i].max_az_offset;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);

      tmp = ptr_group_definition[i].min_el_offset;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

      tmp = ptr_group_definition[i].max_el_offset;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);

      tmp = ptr_group_definition[i].min_dist_factor;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 4);

      tmp = ptr_group_definition[i].max_dist_factor;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 4);
    }
    tmp = ptr_group_definition[i].allow_gain_interact;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].min_gain;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 6);

      tmp = ptr_group_definition[i].max_gain;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 5);
    }

    tmp = ptr_group_definition[i].group_num_members;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);

    tmp = ptr_group_definition[i].has_conjunct_members;
    bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].start_id;
      bits_written += impegh_write_bits_buf(ptr_bit_buf, tmp, 7);
    }
    else
    {
      WORD32 j = 0;
      for (j = 0; j < ptr_group_definition[i].group_num_members + 1; j++)
      {
        ptr_group_definition[i].metadata_ele_id[j] = 0;
        bits_written +=
            impegh_write_bits_buf(ptr_bit_buf, ptr_group_definition[i].metadata_ele_id[j], 7);
      }
    }
  }
  return bits_written;
}

/**impegh_mp4_get_mael
 *
 *
 *  \brief Reads the mael elements to the buffer
 *
 *  \param [in,out] ptr_read_buff Pointer to the read buffer handle
 *  \param [in,out] ptr_write_buff Pointer to the write buffer handle
 *  \param [in] read_size Total bytes read
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_get_mael(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size)
{
  WORD32 k, j;

  WORD32 num_bits_written;
  ia_content_data ptr_content;
  ia_mae_audio_scene_info *ptr_mae_audio_scene_info =
      (ia_mae_audio_scene_info *)ptr_content.array;

  ptr_mae_audio_scene_info->mae_data.num_data_sets = 3;
  ptr_mae_audio_scene_info->mae_data.data_type[0] = 0;
  ptr_mae_audio_scene_info->mae_data.data_type[1] = 1;
  ptr_mae_audio_scene_info->mae_data.data_type[2] = 5;
  impegh_read_bits_buf(ptr_read_buff, 4); // reserved bits
  ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[0] =
      impegh_read_bits_buf(ptr_read_buff, 4); // description languages
  ptr_mae_audio_scene_info->group_desc_data.descr_language[0][0] = 0;
  impegh_read_bits_buf(ptr_read_buff, 24);
  impegh_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info->group_desc_data.num_desc_blocks =
      impegh_read_bits_buf(ptr_read_buff, 7); // reserved bits
  for (j = 0; j < ptr_mae_audio_scene_info->group_desc_data.num_desc_blocks; j++)
  {

    ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[j] = 0;
    ptr_mae_audio_scene_info->group_desc_data.descr_language[j][0] = 0;
    impegh_read_bits_buf(ptr_read_buff, 1); // reserved bits
    ptr_mae_audio_scene_info->group_desc_data.group_id[j] =
        impegh_read_bits_buf(ptr_read_buff, 7);
    ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] = 0;
    impegh_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
      impegh_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
  }

  ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[j] = 0;
  ptr_mae_audio_scene_info->group_desc_data.descr_language[j][0] = 0;

  ptr_mae_audio_scene_info->group_desc_data.group_id[j] = 0;
  ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] = 0;

  for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
  {
    ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;

  impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
  ptr_mae_audio_scene_info->switch_group_desc_data.num_desc_blocks =
      impegh_read_bits_buf(ptr_read_buff, 5);
  for (j = 0; j < ptr_mae_audio_scene_info->switch_group_desc_data.num_desc_blocks; j++)
  {
    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_languages[j] = 0;
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_language[j][0] = 0;
    ptr_mae_audio_scene_info->switch_group_desc_data.group_id[j] =
        impegh_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data_length[j][0] = 0;
    impegh_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] =
          impegh_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] = 0;
  }

  ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_languages[j] = 0;
  ptr_mae_audio_scene_info->switch_group_desc_data.descr_language[j][0] = 0;
  ptr_mae_audio_scene_info->switch_group_desc_data.group_id[j] = 0;

  ptr_mae_audio_scene_info->switch_group_desc_data.descr_data_length[j][0] = 0;

  for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
  {
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] = 0;
  impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
  ptr_mae_audio_scene_info->preset_desc_data.num_desc_blocks =
      impegh_read_bits_buf(ptr_read_buff, 5);
  for (j = 0; j < ptr_mae_audio_scene_info->preset_desc_data.num_desc_blocks; j++)
  {
    ptr_mae_audio_scene_info->preset_desc_data.num_descr_languages[j] = 0;
    ptr_mae_audio_scene_info->preset_desc_data.descr_language[j][0] = 0;
    ptr_mae_audio_scene_info->preset_desc_data.group_id[j] =
        impegh_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0] = 0;
    impegh_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
      impegh_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->preset_desc_data.num_descr_languages[j] = 0;
  ptr_mae_audio_scene_info->preset_desc_data.descr_language[j][0] = 0;
  ptr_mae_audio_scene_info->preset_desc_data.group_id[j] = 0;

  ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0] = 0;

  for (k = 0; k < ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0]; k++)
  {
    ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;

  num_bits_written = impegh_mp4_write_mael_buf(ptr_write_buff, ptr_mae_audio_scene_info);
  num_bits_written += impegh_write_bits_buf(ptr_write_buff, 0, 7); // mae_id_max_avail
  return num_bits_written;
}
/**impegh_mp4_get_maep
 *
 *
 *  \brief Reads the maep elements to the read buffer
 *
 *  \param [in,out] ptr_read_buff Pointer to read buffer handle
 *  \param [in,out] ptr_write_buff Pointer to the write buffer handle
 *  \param [in] read_size Total bytes read
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_get_maep(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size)
{
  WORD32 i, j, num_conditions, num_group_presets, num_bits_written = 0;
  ia_mae_audio_scene_info ptr_mae_audio_scene_info;
  impegh_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info.num_group_presets =
      impegh_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_group_presets = ptr_mae_audio_scene_info.num_group_presets;
  for (i = 0; i < num_group_presets; i++)
  {
    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info.group_presets_definition[i].group_id =
        impegh_read_bits_buf(ptr_read_buff, 5); // preset id

    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info.group_presets_definition[i].preset_kind =
        impegh_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info.group_presets_definition[i].num_conditions =
        impegh_read_bits_buf(ptr_read_buff, 8) - 1;
    num_conditions = ptr_mae_audio_scene_info.group_presets_definition[i].num_conditions;
    for (j = 0; j < num_conditions + 1; j++)
    {
      ptr_mae_audio_scene_info.group_presets_definition[i].reference_id[j] =
          impegh_read_bits_buf(ptr_read_buff, 7); // preset group id

      ptr_mae_audio_scene_info.group_presets_definition[i].cond_on_off[j] =
          impegh_read_bits_buf(ptr_read_buff, 1);

      if (ptr_mae_audio_scene_info.group_presets_definition[i].cond_on_off[j])
      {
        impegh_read_bits_buf(ptr_read_buff, 4); // reserved bits
        ptr_mae_audio_scene_info.group_presets_definition[i].disable_gain_interact[j] =
            impegh_read_bits_buf(ptr_read_buff, 1); // group disable gain Interactvity

        ptr_mae_audio_scene_info.group_presets_definition[i].gain_flag[j] =
            impegh_read_bits_buf(ptr_read_buff, 1);

        ptr_mae_audio_scene_info.group_presets_definition[i].disable_pos_interact[j] =
            impegh_read_bits_buf(ptr_read_buff, 1); // disable position interactivity
        ptr_mae_audio_scene_info.group_presets_definition[i].position_interact[j] =
            impegh_read_bits_buf(ptr_read_buff, 1); // position flag

        if (ptr_mae_audio_scene_info.group_presets_definition[i].gain_flag[j])
        {
          ptr_mae_audio_scene_info.group_presets_definition[i].gain[j] =
              impegh_read_bits_buf(ptr_read_buff, 8);
        }
        if (ptr_mae_audio_scene_info.group_presets_definition[i].position_interact[j])
        {
          ptr_mae_audio_scene_info.group_presets_definition[i].azimuth_offset[j] =
              impegh_read_bits_buf(ptr_read_buff, 8); // azimuth offset

          ptr_mae_audio_scene_info.group_presets_definition[i].elevation_offset[j] =
              impegh_read_bits_buf(ptr_read_buff, 8); // elevation offset

          ptr_mae_audio_scene_info.group_presets_definition[i].dist_factor[j] =
              impegh_read_bits_buf(ptr_read_buff, 8); // dist factor
        }
      }
    }
  }
  num_bits_written += impegh_write_bits_buf(ptr_write_buff, num_group_presets, 5);
  num_bits_written += impegh_mp4_write_maep_buf(
      ptr_write_buff, &ptr_mae_audio_scene_info.group_presets_definition[0], num_group_presets);
  return num_bits_written;
}
/**impegh_mp4_get_maes
 *
 *
 *  \brief Reads the maes elements to the read buffer
 *
 *  \param [in,out] ptr_read_buff Pointer to read buffer handle
 *  \param [in,out] ptr_write_buff Pointer to write buffer handle
 *  \param [in] read_size Total bytes read
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_get_maes(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size)
{
  WORD32 num_bits_written = 0;
  WORD32 num_switch_groups, i, j;
  ia_mae_audio_scene_info ptr_mae_audio_scene_info;

  impegh_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info.num_switch_groups =
      impegh_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_switch_groups = ptr_mae_audio_scene_info.num_switch_groups;
  for (i = 0; i < num_switch_groups; i++)
  {
    ptr_mae_audio_scene_info.switch_group_definition[i].allow_on_off = 0;
    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info.switch_group_definition[i].group_id =
        impegh_read_bits_buf(ptr_read_buff, 5); // reserved bits + switch group id

    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info.switch_group_definition[i].group_num_members =
        impegh_read_bits_buf(ptr_read_buff, 5) - 1;

    for (j = 0; j < ptr_mae_audio_scene_info.switch_group_definition[i].group_num_members + 1;
         j++)
    {

      impegh_read_bits_buf(ptr_read_buff, 1); // reserve bit
      ptr_mae_audio_scene_info.switch_group_definition[i].member_id[j] =
          impegh_read_bits_buf(ptr_read_buff, 7); // switch group member id
    }

    impegh_read_bits_buf(ptr_read_buff, 1); // reserved
    ptr_mae_audio_scene_info.switch_group_definition[i].default_group_id =
        impegh_read_bits_buf(ptr_read_buff, 7); // default id
  }
  num_bits_written += impegh_write_bits_buf(ptr_write_buff, num_switch_groups, 5);
  num_bits_written += impegh_mp4_write_maes_buf(
      ptr_write_buff, &ptr_mae_audio_scene_info.switch_group_definition[0], num_switch_groups);
  return num_bits_written;
}
/**impegh_mp4_get_maeg
 *
 *
 *  \brief Reads the maeg elements to the read buffer
 *
 *  \param [in,out] ptr_read_buff Pointer to the read buffer
 *  \param [in,out] ptr_write_buff Pointer to the write buffer
 *  \param [in]  read_size Total bytes read
 *
 *  \return WORD32
 *
 */
WORD32 impegh_mp4_get_maeg(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size)
{
  WORD32 num_bits_written = 0;
  ia_mae_audio_scene_info ptr_mae_audio_scene_info;
  WORD32 num_grps;
  ptr_mae_audio_scene_info.main_stream_flag = 1;
  ptr_mae_audio_scene_info.asi_id_present = 1;
  ptr_mae_audio_scene_info.asi_id = impegh_read_bits_buf(ptr_read_buff, 8);     // writing asi_id
  impegh_read_bits_buf(ptr_read_buff, 1);                                       // reserved bits
  ptr_mae_audio_scene_info.num_groups = impegh_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_grps = ptr_mae_audio_scene_info.num_groups;
  for (WORD32 grp = 0; grp < num_grps; grp++)
  {
    impegh_read_bits_buf(ptr_read_buff, 1); // reserved bits
    ptr_mae_audio_scene_info.group_definition[grp].group_id =
        (WORD8)impegh_read_bits_buf(ptr_read_buff, 7);
    impegh_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info.group_definition[grp].allow_on_off =
        (WORD8)impegh_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info.group_definition[grp].default_on_off =
        (WORD8)impegh_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info.group_definition[grp].allow_pos_interact =
        (WORD8)impegh_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info.group_definition[grp].allow_gain_interact =
        (WORD8)impegh_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info.group_definition[grp].has_contentLanguage =
        impegh_read_bits_buf(ptr_read_buff, 1); // content language
    impegh_read_bits_buf(ptr_read_buff, 8);     // reserved + content kind
    if (ptr_mae_audio_scene_info.group_definition[grp].allow_pos_interact)
    {
      impegh_read_bits_buf(ptr_read_buff, 1); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].min_az_offset =
          impegh_read_bits_buf(ptr_read_buff, 7);
      impegh_read_bits_buf(ptr_read_buff, 1); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].max_az_offset =
          impegh_read_bits_buf(ptr_read_buff, 7);
      impegh_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].min_el_offset =
          impegh_read_bits_buf(ptr_read_buff, 5);
      impegh_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].max_el_offset =
          impegh_read_bits_buf(ptr_read_buff, 5);
      ptr_mae_audio_scene_info.group_definition[grp].min_dist_factor =
          impegh_read_bits_buf(ptr_read_buff, 4);
      ptr_mae_audio_scene_info.group_definition[grp].max_dist_factor =
          impegh_read_bits_buf(ptr_read_buff, 4);
    }
    ptr_mae_audio_scene_info.group_definition[grp].group_num_members = 1;

    ptr_mae_audio_scene_info.group_definition[grp].has_conjunct_members = 1;

    if (ptr_mae_audio_scene_info.group_definition[grp].allow_gain_interact)
    {
      impegh_read_bits_buf(ptr_read_buff, 2); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].min_gain =
          impegh_read_bits_buf(ptr_read_buff, 6);
      impegh_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info.group_definition[grp].max_gain =
          impegh_read_bits_buf(ptr_read_buff, 5);
    }
    if (ptr_mae_audio_scene_info.group_definition[grp].has_contentLanguage)
    {
      impegh_read_bits_buf(ptr_read_buff, 8); // content language
    }
  }
  ptr_mae_audio_scene_info.group_definition[0].start_id = 0;
  ptr_mae_audio_scene_info.group_definition[1].start_id = 2;
  num_bits_written +=
      impegh_write_bits_buf(ptr_write_buff, ptr_mae_audio_scene_info.main_stream_flag, 1);
  num_bits_written +=
      impegh_write_bits_buf(ptr_write_buff, ptr_mae_audio_scene_info.asi_id_present, 1);
  num_bits_written += impegh_write_bits_buf(ptr_write_buff, ptr_mae_audio_scene_info.asi_id, 8);
  num_bits_written += impegh_write_bits_buf(ptr_write_buff, num_grps, 7);
  num_bits_written += impegh_mp4_write_maeg_buf(
      ptr_write_buff, &ptr_mae_audio_scene_info.group_definition[0], num_grps);
  return num_bits_written;
}