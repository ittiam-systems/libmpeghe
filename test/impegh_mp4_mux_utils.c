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
#include <assert.h>
#include "impegh_mp4_mux_utils.h"
#include "impegh_error_codes_mux.h"
#include "impegh_mux_rom.h"

/**
 *  impegh_write_bits_buf
 *
 *  \brief Writes bits into bit-buffer
 *
 *  \param [in,out]	it_bit_buf	Pointer to bit-buffer
 *  \param [in]		write_val	Value to be written into bit-buffer
 *  \param [in]		num_bits	Number of bits to be written
 *
 *  \return UWORD8
 *
 */
UWORD8 impegh_write_bits_buf(ia_bit_buf_struct *it_bit_buf, UWORD32 write_val, UWORD8 num_bits)
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
/**
 *  impegh_create_bit_buffer
 *
 *  \brief Creates and initializes the bit-buffer using given base pointer, size and init flag
 *
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *  \param [in]		ptr_bit_buf_base	Pointer to bit-buffer base
 *  \param [in]		bit_buffer_size		Size of bit buffer to be created
 *  \param [in]		init				Initialization flag
 *
 *  \return ia_bit_buf_struct
 *
 */
ia_bit_buf_struct *impegh_create_mp4_buffer(ia_bit_buf_struct *it_bit_buf,
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

/**impegh_read_bits_buf
 *
 *  \brief Helper function to read bits.
 *
 *  \param [in,out] it_bit_buff Pointer to bit buffer structure.
 *  \param [in]  no_of_bits     Pointer to no of bits to be read.
 *
 *  \return WORD32              Value read from bit stream.
 *
 */

WORD32 impegh_read_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits)
{
  UWORD32 ret_val;
  UWORD8 *ptr_read_next = it_bit_buff->ptr_read_next;
  WORD bit_pos = it_bit_buff->bit_pos; // 7

  if (no_of_bits == 0)
  {
    return 0;
  }
  if (it_bit_buff->cnt_bits < no_of_bits || it_bit_buff->cnt_bits < 0)
  {
    longjmp(*(it_bit_buff->impeghe_jmp_buf), 1);
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

UWORD32 impegh_read_escape_value(ia_bit_buf_struct *it_bit_buff,
  UWORD32 no_bits1,
  UWORD32 no_bits2, UWORD32 no_bits3, WORD32 *bits_read) {
  UWORD32 value = 0;
  UWORD32 val_add = 0;
  UWORD32 max_val1 = (1 << no_bits1) - 1;
  UWORD32 max_val2 = (1 << no_bits2) - 1;
  *bits_read = 0;
  value = impegh_read_bits_buf(it_bit_buff, no_bits1);
  *bits_read = no_bits1;
  if (value == max_val1) {
    val_add = impegh_read_bits_buf(it_bit_buff, no_bits2);

    value += val_add;
    *bits_read += no_bits2;
    if (val_add == max_val2) {
      val_add = impegh_read_bits_buf(it_bit_buff, no_bits3);

      value += val_add;
      *bits_read += no_bits3;
    }
  }

  return value;
}

/**impegh_skip_bits_buf
 *
 *  \brief Helper function to skip bits in bit buffer.
 *
 *  \param [in,out] it_bit_buff Pointer to bit buffer structure.
 *  \param [in]     no_of_bits  No. of bits to skip.
 *
 *  \return IA_ERRORCODE     error
 *
 */
IA_ERRORCODE impegh_skip_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits)
{
  UWORD8 *ptr_read_next = it_bit_buff->ptr_read_next;
  WORD bit_pos = it_bit_buff->bit_pos;

  it_bit_buff->cnt_bits -= no_of_bits;

  ptr_read_next += (no_of_bits >> 3);
  bit_pos -= (no_of_bits % 8);
  if (bit_pos < 0)
  {
    bit_pos += 8;
    ptr_read_next++;
  }
  if (bit_pos > 7)
  {
    return IMPEGHE_MUX_NON_FATAL_INVALID_BIT_POS;
  }
  it_bit_buff->ptr_read_next = ptr_read_next;
  it_bit_buff->bit_pos = (WORD16)bit_pos;
  return IA_NO_ERROR;
}

/**impegh_create_bit_buf
 *
 *  \brief Initialize bit buffer structure entries.
 *
 *  \param [in,out] it_bit_buff      Pointer to bit buffer structure.
 *  \param [in]     ptr_bit_buf_base Pointer to bit buffer base.
 *  \param [in]     bit_buf_size     Pointer to bit buffer size.
 *
 *  \return ia_bit_buf_struct * Pointer to bit buffer structure
 *
 */
ia_bit_buf_struct *impegh_create_bit_buf(ia_bit_buf_struct *it_bit_buff,
                                           UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size)
{
  it_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buff->ptr_bit_buf_end = ptr_bit_buf_base + bit_buf_size - 1;

  it_bit_buff->ptr_read_next = ptr_bit_buf_base;
  it_bit_buff->bit_pos = 7;

  it_bit_buff->cnt_bits = 0;
  it_bit_buff->size = bit_buf_size << 3;

  return it_bit_buff;
}
/**
 *  impegh_mae_parse_description_data
 *
 *  \brief Parse the mae description data elements
 *
 *  \param [in]     ptr_bit_buf          Pointer to the bit buffer handle
 *  \param [in,out] ptr_description_data Pointer to descrition data handle
 *  \param [in]     group_id_bits        Number of group id bits
 *
 *  \return VOID
 */
static VOID impegh_mae_parse_description_data(ia_bit_buf_struct *ptr_bit_buf,
                                              ia_description_data_struct *description_data,
                                              WORD32 group_id_bits)
{
  WORD32 n, num_descr_blocks, tmp, i, c;
  num_descr_blocks = impegh_read_bits_buf(ptr_bit_buf, 7);
  description_data->mae_bsNumDescriptionBlocks = num_descr_blocks + 1;
  for (n = 0; n < num_descr_blocks + 1; n++)
  {
    tmp = impegh_read_bits_buf(ptr_bit_buf, group_id_bits);
    description_data->mae_descriptionID[n] = tmp;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 4);
    description_data->mae_bsNumDescLanguages[n] = tmp + 1;
    for (i = 0; i < description_data->mae_bsNumDescLanguages[n]; i++)
    {
      tmp = impegh_read_bits_buf(ptr_bit_buf, 24);
      description_data->mae_bsDescriptionLanguage[n][i] = tmp;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      description_data->mae_bsDescriptionDataLength[n][i] = tmp + 1;
      for (c = 0; c < description_data->mae_bsDescriptionDataLength[n][i]; c++)
      {
        tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
        description_data->mae_descriptionData[n][i][c] = tmp;
      }
    }
  }
}


/**
 *  impegh_mae_parse_desimpegh_mae_parse_content_datacription_data
 *
 *  \brief Parse the mae description data elements
 *
 *  \param [in]     ptr_bit_buf          Pointer to the bit buffer handle
 *  \param [in,out] ptr_content_data Pointer to descrition data handle
 *  \param [in]     group_id_bits        Number of group id bits
 *
 *  \return VOID
 */
static VOID impegh_mae_parse_content_data(ia_bit_buf_struct *ptr_bit_buf, ia_content_data_structure *content)
{
  WORD32 n, num_content_blocks;
  num_content_blocks = impegh_read_bits_buf(ptr_bit_buf, 7);
  content->mae_bsNumContentDataBlocks = num_content_blocks + 1;
  for (n = 0; n < num_content_blocks + 1; n++)
  {
    content->mae_ContentDataGroupID[n] = impegh_read_bits_buf(ptr_bit_buf, 7);
    content->mae_contentKind[n] = impegh_read_bits_buf(ptr_bit_buf, 4);
    content->mae_hasContentLanguage[n] = impegh_read_bits_buf(ptr_bit_buf, 1);//mae_hasContentLanguage;
    if (content->mae_hasContentLanguage[n])
    {
      content->mae_contentLanguage[n] = impegh_read_bits_buf(ptr_bit_buf, 24);
    }
  }
}

static WORD32 is_lang_present(WORD32 arr[], WORD32 size, WORD32 val) {
    for (WORD32 i = 0; i < size; i++) {
        if (arr[i] == val) {
            return 1;
        }
    }
    return 0;
}

WORD32 impegh_find_unique_language(ia_mae_data_struct *mae_data, WORD32 *descr_language)
{
  WORD32 group_desc_data_language_cnt = 0;
  WORD32 switch_group_desc_data_language_cnt = 0;
  WORD32 group_preset_desc_data_language_cnt = 0;

  WORD32 unique_group_desc_data_language_cnt = 0;
  WORD32 unique_switch_group_desc_data_language_cnt = 0;
  WORD32 unique_group_preset_desc_data_language_cnt = 0;

  WORD32 num_descr_languages = 0;
  WORD32 desc_data_language[16] = { 0 };
  WORD32 group_desc_data_language[16] = { 0 };
  WORD32 switch_group_desc_data_languag[16] = {0};
  WORD32 group_preset_desc_data_language[16] = { 0 };
  for (int n = 0; n < mae_data->group_desc_data.mae_bsNumDescriptionBlocks + 1; n++)
  {
    for (int i = 0; i < mae_data->group_desc_data.mae_bsNumDescLanguages[n]; i++)
    {
      if (unique_group_desc_data_language_cnt == 0)
      {
        group_desc_data_language[unique_group_desc_data_language_cnt] = mae_data->group_desc_data.mae_bsDescriptionLanguage[n][i];
        unique_group_desc_data_language_cnt++;
      }
      else
      {
        for (int idx = 0; idx < unique_group_desc_data_language_cnt; idx++)
        {
          if (group_desc_data_language[idx] == mae_data->group_desc_data.mae_bsDescriptionLanguage[n][i])
          {
            continue;
          }
          else
          {
            group_desc_data_language[unique_group_desc_data_language_cnt] = mae_data->group_desc_data.mae_bsDescriptionLanguage[n][i];
            unique_group_desc_data_language_cnt++;
          }
        }

      }
    }
  }

  for (int n = 0; n < mae_data->switch_desc_data.mae_bsNumDescriptionBlocks + 1; n++)
  {
    for (int i = 0; i < mae_data->switch_desc_data.mae_bsNumDescLanguages[n]; i++)
    {
      if (unique_switch_group_desc_data_language_cnt == 0)
      {
        switch_group_desc_data_languag[unique_switch_group_desc_data_language_cnt] = mae_data->switch_desc_data.mae_bsDescriptionLanguage[n][i];
        unique_switch_group_desc_data_language_cnt++;
      }
      else
      {
        for (int idx = 0; idx < unique_switch_group_desc_data_language_cnt; idx++)
        {
          if (switch_group_desc_data_languag[idx] == mae_data->switch_desc_data.mae_bsDescriptionLanguage[n][i])
          {
            continue;
          }
          else
          {
            switch_group_desc_data_languag[unique_switch_group_desc_data_language_cnt] = mae_data->switch_desc_data.mae_bsDescriptionLanguage[n][i];
            unique_switch_group_desc_data_language_cnt++;
          }
        }

      }
    }
  }

  for (int n = 0; n < mae_data->group_preset_desc_data.mae_bsNumDescriptionBlocks + 1; n++)
  {
    for (int i = 0; i < mae_data->group_preset_desc_data.mae_bsNumDescLanguages[n]; i++)
    {
      if (unique_group_preset_desc_data_language_cnt == 0)
      {
        group_preset_desc_data_language[unique_group_preset_desc_data_language_cnt] = mae_data->group_preset_desc_data.mae_bsDescriptionLanguage[n][i];
        unique_group_preset_desc_data_language_cnt++;
      }
      else
      {
        for (int idx = 0; idx < unique_group_preset_desc_data_language_cnt; idx++)
        {
          if (group_preset_desc_data_language[idx] == mae_data->group_preset_desc_data.mae_bsDescriptionLanguage[n][i])
          {
            continue;
          }
          else
          {
            group_preset_desc_data_language[unique_group_preset_desc_data_language_cnt] = mae_data->group_preset_desc_data.mae_bsDescriptionLanguage[n][i];
            unique_group_preset_desc_data_language_cnt++;
          }
        }

      }
    }
  }
  if ((unique_group_preset_desc_data_language_cnt == unique_switch_group_desc_data_language_cnt) && (unique_group_desc_data_language_cnt == unique_switch_group_desc_data_language_cnt))
  {
    for (int idx = 0; idx < unique_group_preset_desc_data_language_cnt; idx++)
    {
      if ((group_preset_desc_data_language[idx] == switch_group_desc_data_languag[idx]) && (switch_group_desc_data_languag[idx] == group_desc_data_language[idx]))
      {
        descr_language[idx] = group_desc_data_language[idx];
        num_descr_languages++;
      }
      else
      {
        assert(0);
        //need to design this logic
        //condition where switch_group_desc_data_languag, group_preset_desc_data_language, group_desc_data_language order and value is not same, 
        //here we need to make zero entrie to structure to keep all language
      }
    }
  }
  else
  {
    for (int i = 0; i < unique_group_desc_data_language_cnt; i++) {
        if (!is_lang_present(descr_language, num_descr_languages, group_desc_data_language[i])) {
            descr_language[num_descr_languages++] = group_desc_data_language[i];
        }
    }
    for (int i = 0; i < unique_switch_group_desc_data_language_cnt; i++) {
        if (!is_lang_present(descr_language, num_descr_languages, group_desc_data_language[i])) {
            descr_language[num_descr_languages++] = switch_group_desc_data_languag[i];
        }
    }
    for (int i = 0; i < unique_group_preset_desc_data_language_cnt; i++) {
        if (!is_lang_present(descr_language, num_descr_languages, group_preset_desc_data_language[i])) {
            descr_language[num_descr_languages++] = group_preset_desc_data_language[i];
        }
    }
  }
  return num_descr_languages;

}
/**
 *  impegh_mae_asi_data_parse
 *
 *  \brief Parse mae data elements
 *
 *  \param [in]     ptr_bit_buf  Pointer to bit buffer handle
 *  \param [in,out] ptr_mael_buf Pointer to bit buffer handle of mael box
 *
 *  \return VOID
 */
VOID impegh_mae_asi_data_parse(ia_audio_scene_data *audio_scene_data, ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i = 0, data_type, data_length;
  WORD32 num_data_sets = 0;
  num_data_sets = impegh_read_bits_buf(ptr_bit_buf, 4);
  audio_scene_data->mae_data.mae_numDataSets = num_data_sets;
  for (i = 0; i < num_data_sets; i++)
  {
    WORD32 cnt_bits = 0;
    data_type = impegh_read_bits_buf(ptr_bit_buf, 4);
    audio_scene_data->mae_data.mae_dataType[i] = data_type;
    data_length = impegh_read_bits_buf(ptr_bit_buf, 16);
    audio_scene_data->mae_data.mae_dataLength[i] = data_length;
    cnt_bits = ptr_bit_buf->cnt_bits;
    switch (data_type)
    {
    case ID_MAE_GROUP_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &audio_scene_data->mae_data.group_desc_data, 7);
      break;
    case ID_MAE_SWITCHGROUP_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &audio_scene_data->mae_data.switch_desc_data, 5);
      break;
    case ID_MAE_GROUP_PRESET_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &audio_scene_data->mae_data.group_preset_desc_data, 5);
      break;
    case ID_MAE_GROUP_CONTENT:
      impegh_mae_parse_content_data(ptr_bit_buf, &audio_scene_data->mae_data.content_data);
      break;
    case ID_MAE_GROUP_COMPOSITE:
    case ID_MAE_SCREEN_SIZE:
    case ID_MAE_DRC_UI_INFO:
    case ID_MAE_SCREEN_SIZE_EXTENSION:
    case ID_MAE_GROUP_PRESET_EXTENSION:
    case ID_MAE_LOUDNESS_COMPENSATION:
      impegh_read_bits_buf(ptr_bit_buf, data_length<<3);
      break;
    default:
      while (data_length > 0)
      {
        impegh_read_bits_buf(ptr_bit_buf, 8);
        data_length--;
      }
      break;
    }
    cnt_bits = cnt_bits - ptr_bit_buf->cnt_bits;
    data_length = audio_scene_data->mae_data.mae_dataLength[i];
    if (cnt_bits < (data_length << 3))
    {
      WORD32 skip_bits = (data_length << 3) - cnt_bits;
      impegh_read_bits_buf(ptr_bit_buf, skip_bits);
    }
  }
}

VOID impegh_mae_asi_data_write(ia_audio_scene_data *audio_scene_data, ia_bit_buf_struct *ptr_mael_buf)
{
  WORD32 i = 0;
  WORD32 descr_language[16];
  WORD8 num_descr_languages = 0;
  num_descr_languages = impegh_find_unique_language(&audio_scene_data->mae_data, descr_language);

  impegh_write_bits_buf(ptr_mael_buf, 0, 4); // reserved bits
  impegh_write_bits_buf(ptr_mael_buf, num_descr_languages,
    4); // description languages
  for (i = 0; i < num_descr_languages; i++)
  {
    impegh_write_bits_buf(ptr_mael_buf, descr_language[i], 24);
    impegh_write_bits_buf(ptr_mael_buf, 0, 1); // reserved bits

    impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_desc_data.mae_bsNumDescriptionBlocks, 7);
    for (WORD32 j = 0; j < audio_scene_data->mae_data.group_desc_data.mae_bsNumDescriptionBlocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 1); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_desc_data.mae_descriptionID[j],
        7); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_desc_data.mae_bsDescriptionDataLength[j][i],
        8);
      for (WORD32 k = 0; k < audio_scene_data->mae_data.group_desc_data.mae_bsDescriptionDataLength[j][i]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_desc_data.mae_descriptionData[j][i][k], 8);
      }
    }
    impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
    impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.switch_desc_data.mae_bsNumDescriptionBlocks, 5);
    for (WORD32 j = 0; j < audio_scene_data->mae_data.switch_desc_data.mae_bsNumDescriptionBlocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.switch_desc_data.mae_descriptionID[j], 5);
      impegh_write_bits_buf(ptr_mael_buf,
        audio_scene_data->mae_data.switch_desc_data.mae_bsDescriptionDataLength[j][i], 8);
      for (WORD32 k = 0; k < audio_scene_data->mae_data.switch_desc_data.mae_bsDescriptionDataLength[j][i]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf,
          audio_scene_data->mae_data.switch_desc_data.mae_descriptionData[j][i][k], 8);
      }
    }
    impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
    impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_preset_desc_data.mae_bsNumDescriptionBlocks, 5);
    for (WORD32 j = 0; j < audio_scene_data->mae_data.group_preset_desc_data.mae_bsNumDescriptionBlocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_preset_desc_data.mae_descriptionID[j], 5);
      impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_preset_desc_data.mae_bsDescriptionDataLength[j][i],
        8);
      for (WORD32 k = 0; k < audio_scene_data->mae_data.group_preset_desc_data.mae_bsDescriptionDataLength[j][i]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf, audio_scene_data->mae_data.group_preset_desc_data.mae_descriptionData[j][i][k],
          8);
      }
    }
  }
}

/**
 *  impegh_mae_asi_group_presets_def
 *
 *  \brief Parse and write mae group preset elements to an intermediate buffer
 *
 *  \param [in,out]     ia_audio_scene_data Pointer to audio_scene_data handle
 *  \param [in]     ptr_bit_buf       Pointer to bit-buffer handle
 *  \param [in,out] ptr_maep_buf      Pointer to bit buffer handle of maep box
 *  \param [in]     num_group_presets Number of group presets
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_group_presets_def_parse(ia_audio_scene_data *audio_scene_data, ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i, j;
  for (i = 0; i < audio_scene_data->mae_numGroupPresets; i++)
  {
    audio_scene_data->maeP_data[i].mae_groupPresetID = impegh_read_bits_buf(ptr_bit_buf, 5);
    audio_scene_data->maeP_data[i].mae_groupPresetKind = impegh_read_bits_buf(ptr_bit_buf, 5);

    audio_scene_data->maeP_data[i].mae_bsGroupPresetNumConditions = impegh_read_bits_buf(ptr_bit_buf, 4) +1 ;
    for (j = 0; j < audio_scene_data->maeP_data[i].mae_bsGroupPresetNumConditions; j++)
    {
      audio_scene_data->maeP_data[i].mae_groupPresetReferenceID[j] = impegh_read_bits_buf(ptr_bit_buf, 7); // preset group id

      audio_scene_data->maeP_data[i].mae_groupPresetConditionOnOff[j] = impegh_read_bits_buf(ptr_bit_buf, 1);

      if (audio_scene_data->maeP_data[i].mae_groupPresetConditionOnOff[j])
      {
        audio_scene_data->maeP_data[i].mae_groupPresetDisableGainInteractivity[j] = impegh_read_bits_buf(ptr_bit_buf, 1);

        audio_scene_data->maeP_data[i].mae_groupPresetGainFlag[j] = impegh_read_bits_buf(ptr_bit_buf, 1);

        if (audio_scene_data->maeP_data[i].mae_groupPresetGainFlag[j])
        {
          audio_scene_data->maeP_data[i].mae_groupPresetGain[j] = impegh_read_bits_buf(ptr_bit_buf, 8);
        }
        audio_scene_data->maeP_data[i].mae_groupPresetDisablePositionInteractivity[j] = impegh_read_bits_buf(ptr_bit_buf, 1);

        audio_scene_data->maeP_data[i].mae_groupPresetPositionFlag[j] = impegh_read_bits_buf(ptr_bit_buf, 1);  // position flag

        if (audio_scene_data->maeP_data[i].mae_groupPresetPositionFlag[j])
        {
          audio_scene_data->maeP_data[i].mae_groupPresetAzOffset[j] = impegh_read_bits_buf(ptr_bit_buf, 8); // azimuth offset

          audio_scene_data->maeP_data[i].mae_groupPresetElOffset[j] = impegh_read_bits_buf(ptr_bit_buf, 6); // elevation offset

          audio_scene_data->maeP_data[i].mae_groupPresetDistFactor[j] = impegh_read_bits_buf(ptr_bit_buf, 4); // dist factor
        }
      }
    }
  }
}

static VOID impegh_mae_asi_group_presets_def_write(ia_audio_scene_data *audio_scene_data,
  ia_bit_buf_struct *ptr_maep_buf)
{
  WORD32 i, j;
  for (i = 0; i < audio_scene_data->mae_numGroupPresets; i++)
  {
    impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetID, 8); // preset id +reserved bits
    impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetKind, 8); // mae_groupPresetKind +reserved bits
    impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_bsGroupPresetNumConditions, 8);
    for (j = 0; j < audio_scene_data->maeP_data[i].mae_bsGroupPresetNumConditions; j++)
    {
      impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetReferenceID[j], 7);
      impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetConditionOnOff[j], 1); // preset condition on off

      if (audio_scene_data->maeP_data[i].mae_groupPresetConditionOnOff[j])
      {
        impegh_write_bits_buf(ptr_maep_buf, 0, 4); // reserved bits

        impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetDisableGainInteractivity[j], 1); // group disable gain Interactvity

        impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetGainFlag[j], 1); // group preset gain flag

        impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetDisablePositionInteractivity[j], 1); // disable position interactivity

        impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetPositionFlag[j], 1);
        if (audio_scene_data->maeP_data[i].mae_groupPresetGainFlag[j])
        {
          impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetGain[j], 8);
        }
        if (audio_scene_data->maeP_data[i].mae_groupPresetPositionFlag[j])
        {
          impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetAzOffset[j], 8);

          impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetElOffset[j], 8);

          impegh_write_bits_buf(ptr_maep_buf, audio_scene_data->maeP_data[i].mae_groupPresetDistFactor[j], 8);
        }
      }
    }
  }
}

/**
 *  impegh_mae_asi_group_def
 *
 *  \brief Parse and write asi group definition elements to an intermediate buffer
 *
 *  \param [in,out]     ia_audio_scene_data Pointer to audio_scene_data handle
 *  \param [in]     ptr_bit_buf    Pointer to bit buffer handle
 *  \param [in,out] ptr_asi_buffer Pointer to asi buffer handle
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_group_def_parse(ia_audio_scene_data *audio_scene_data, ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i;
  for (i = 0; i < audio_scene_data->mae_numGroups; i++)
  {
    audio_scene_data->maeG_data[i].mae_groupID = impegh_read_bits_buf(ptr_bit_buf, 7);  // mae_grp id

    audio_scene_data->maeG_data[i].mae_allowOnOff = impegh_read_bits_buf(ptr_bit_buf, 1); // allow_on_off

    audio_scene_data->maeG_data[i].mae_defaultOnOff = impegh_read_bits_buf(ptr_bit_buf, 1); // default_on_off

    audio_scene_data->maeG_data[i].mae_allowPositionInteractivity = impegh_read_bits_buf(ptr_bit_buf, 1); // position interactivity

    if (audio_scene_data->maeG_data[i].mae_allowPositionInteractivity)
    {
      audio_scene_data->maeG_data[i].mae_interactivityMinAzOffset = impegh_read_bits_buf(ptr_bit_buf, 7);
      audio_scene_data->maeG_data[i].mae_interactivityMaxAzOffset = impegh_read_bits_buf(ptr_bit_buf, 7);
      audio_scene_data->maeG_data[i].mae_interactivityMinElOffset = impegh_read_bits_buf(ptr_bit_buf, 5);
      audio_scene_data->maeG_data[i].mae_interactivityMaxElOffset = impegh_read_bits_buf(ptr_bit_buf, 5);
      audio_scene_data->maeG_data[i].mae_interactivityMinDistFactor = impegh_read_bits_buf(ptr_bit_buf, 4);
      audio_scene_data->maeG_data[i].mae_interactivityMaxDistFactor = impegh_read_bits_buf(ptr_bit_buf, 4);
    }
    audio_scene_data->maeG_data[i].mae_allowGainInteractivity = impegh_read_bits_buf(ptr_bit_buf, 1); // gain interaction

    if (audio_scene_data->maeG_data[i].mae_allowGainInteractivity)
    {
      audio_scene_data->maeG_data[i].mae_interactivityMinGain = impegh_read_bits_buf(ptr_bit_buf, 6);
      audio_scene_data->maeG_data[i].mae_interactivityMaxGain = impegh_read_bits_buf(ptr_bit_buf, 5);
    }

    audio_scene_data->maeG_data[i].mae_bsGroupNumMembers  = impegh_read_bits_buf(ptr_bit_buf, 7) + 1; // group_num_members skip them

    audio_scene_data->maeG_data[i].mae_hasConjunctMembers = impegh_read_bits_buf(ptr_bit_buf, 1);               // has conjucnt_members skip them

    if (audio_scene_data->maeG_data[i].mae_hasConjunctMembers)
    {
      audio_scene_data->maeG_data[i].mae_startID = impegh_read_bits_buf(ptr_bit_buf, 7); // start id skip them
    }
    else
    {
      WORD32 j = 0;
      for (j = 0; j < audio_scene_data->maeG_data[i].mae_bsGroupNumMembers; j++)
      {
        audio_scene_data->maeG_data[i].mae_metaDataElementID[j] = impegh_read_bits_buf(ptr_bit_buf, 7); // meta data ele id skip them
      }
    }
  }
}

static VOID impegh_mae_asi_group_def_write(ia_audio_scene_data *audio_scene_data,
  ia_bit_buf_struct *ptr_asi_buffer)
{
  WORD32 i;
  for (i = 0; i < audio_scene_data->mae_numGroups; i++)
  {
    impegh_write_bits_buf(ptr_asi_buffer, 0, 1); // reserved bits

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_groupID, 7);

    impegh_write_bits_buf(ptr_asi_buffer, 0, 3); // reserved bits

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_allowOnOff, 1);

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_defaultOnOff, 1);

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_allowPositionInteractivity, 1);

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_allowGainInteractivity, 1);

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->mae_data.content_data.mae_hasContentLanguage[i], 1);

    impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->mae_data.content_data.mae_contentKind[i], 8);

    if (audio_scene_data->maeG_data[i].mae_allowPositionInteractivity)
    {

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMinAzOffset, 8); // reserved+min_az_offest bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMaxAzOffset, 8); // reserved+max_az_offset bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMinElOffset, 8); // reserved + min_el_offest bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMaxElOffset, 8); // reserved + max_el_offest bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMinDistFactor, 4); // MinDistFactor bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMaxDistFactor, 4); // MaxDistFactor bits
    }
    if (audio_scene_data->maeG_data[i].mae_allowGainInteractivity)
    {
      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMinGain, 8); // reserved + min_gain bits

      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->maeG_data[i].mae_interactivityMaxGain, 8); // reserved +max_gain bits
    }

    if (audio_scene_data->mae_data.content_data.mae_hasContentLanguage[i] == 1)
    {
      impegh_write_bits_buf(ptr_asi_buffer, audio_scene_data->mae_data.content_data.mae_contentLanguage[i], 24);
    }
  }
}

/**
 *  impegh_mae_asi_switch_group_def
 *
 *  \brief Parse and write the elements of switch group to an intermediate buffer
 *
 *  \param [in,out]     ia_audio_scene_data Pointer to audio_scene_data handle
 *  \param [in]     ptr_bit_buf Pointer to bit buffer handle
 *  \param [in,out] ptr_maes_buf Pointer to switch group bit buffer handle
 *  \param [in]     num_switch_groups Number of switch groups
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_switch_group_def_parse(ia_audio_scene_data *audio_scene_data, ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i, j;
  for (i = 0; i < audio_scene_data->mae_numSwitchGroups; i++)
  {
    audio_scene_data->maeS_data[i].mae_switchGroupID = impegh_read_bits_buf(ptr_bit_buf, 5);

    audio_scene_data->maeS_data[i].mae_switchGroupAllowOnOff = impegh_read_bits_buf(ptr_bit_buf, 1);  // allow on off
    if (audio_scene_data->maeS_data[i].mae_switchGroupAllowOnOff)
    {
      audio_scene_data->maeS_data[i].mae_switchGroupDefaultOnOff = impegh_read_bits_buf(ptr_bit_buf, 1);
    }
    audio_scene_data->maeS_data[i].mae_bsSwitchGroupNumMembers = impegh_read_bits_buf(ptr_bit_buf, 5) + 1;

    for (j = 0; j < audio_scene_data->maeS_data[i].mae_bsSwitchGroupNumMembers; j++)
    {
      audio_scene_data->maeS_data[i].mae_switchGroupMemberID[j] = impegh_read_bits_buf(ptr_bit_buf, 7);  // member id
    }
    audio_scene_data->maeS_data[i].mae_switchGroupDefaultGroupID = impegh_read_bits_buf(ptr_bit_buf, 7);  // default id
  }
}

static VOID impegh_mae_asi_switch_group_def_write(ia_audio_scene_data *audio_scene_data,
  ia_bit_buf_struct *ptr_maes_buf)
{
  WORD32 i, j;
  for (i = 0; i < audio_scene_data->mae_numSwitchGroups; i++)
  {
    impegh_write_bits_buf(ptr_maes_buf, audio_scene_data->maeS_data[i].mae_switchGroupID, 8); // reserved bits + switch group id
    impegh_write_bits_buf(ptr_maes_buf, audio_scene_data->maeS_data[i].mae_bsSwitchGroupNumMembers, 8); // reserved bits+  num_switch group memer
    for (j = 0; j < audio_scene_data->maeS_data[i].mae_bsSwitchGroupNumMembers; j++)
    {
      impegh_write_bits_buf(ptr_maes_buf, audio_scene_data->maeS_data[i].mae_switchGroupMemberID[j], 8); // reserved bits + switch group member id
    }
    impegh_write_bits_buf(ptr_maes_buf, audio_scene_data->maeS_data[i].mae_switchGroupDefaultGroupID, 8); // reserved bits + switch group default id
  }
}

static VOID EnhancedObjectMetadataConfig(ia_mpegh3daExtElementConfig *extElementConfig, ia_bit_buf_struct *ptr_bit_buf, int num_objects)
{
  ia_EnhancedObjectMetadataConfig *pstr_eomConfig = &extElementConfig->EnhancedObjectMetadataConfig_data;
  pstr_eomConfig->hasDiffuseness = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (pstr_eomConfig->hasDiffuseness)
  {
    pstr_eomConfig->hasCommonGroupDiffuseness = impegh_read_bits_buf(ptr_bit_buf, 1);
  }
  pstr_eomConfig->hasExcludedSectors = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (pstr_eomConfig->hasExcludedSectors)
  {
    pstr_eomConfig->hasCommonGroupExcludedSectors = impegh_read_bits_buf(ptr_bit_buf, 1);
    if (pstr_eomConfig->hasCommonGroupExcludedSectors)
    {
        memset(pstr_eomConfig->useOnlyPredefinedSectors, (WORD8)impegh_read_bits_buf(ptr_bit_buf, 1), sizeof(pstr_eomConfig->useOnlyPredefinedSectors));
    }
  } else {
      pstr_eomConfig->hasCommonGroupExcludedSectors = 0;
  }
  pstr_eomConfig->hasClosestSpeakerCondition = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (pstr_eomConfig->hasClosestSpeakerCondition)
  {
      pstr_eomConfig->closestSpeakerThresholdAngle = impegh_read_bits_buf(ptr_bit_buf, 7);
  }
  for (int o = 0; o < num_objects; o++)
  {
      pstr_eomConfig->hasDivergence[o] = impegh_read_bits_buf(ptr_bit_buf, 1);
      if (pstr_eomConfig->hasDivergence[o])
      {
          pstr_eomConfig->divergenceAzimuthRange[o] = impegh_read_bits_buf(ptr_bit_buf, 6);
      }
      if (pstr_eomConfig->hasCommonGroupExcludedSectors == 0)
      {
          pstr_eomConfig->useOnlyPredefinedSectors[o] = impegh_read_bits_buf(ptr_bit_buf, 1);
      }
  }
}

static VOID drcCoefficientsUniDrc(ia_drcCoefficientsUniDrc *drcCoefficientsUniDrc_data, ia_bit_buf_struct *ptr_bit_buf)
{
    drcCoefficientsUniDrc_data->gainSequenceCount = 0;
    drcCoefficientsUniDrc_data->drcLocation = impegh_read_bits_buf(ptr_bit_buf, 4);
    drcCoefficientsUniDrc_data->drcFrameSizePresent = impegh_read_bits_buf(ptr_bit_buf, 1);
    if (drcCoefficientsUniDrc_data->drcFrameSizePresent)
    {
        drcCoefficientsUniDrc_data->bsDrcFrameSize = impegh_read_bits_buf(ptr_bit_buf, 15);
    }
    drcCoefficientsUniDrc_data->gainSetCount = impegh_read_bits_buf(ptr_bit_buf, 6);
    for (int i = 0; i < drcCoefficientsUniDrc_data->gainSetCount; i++)
    {
        drcCoefficientsUniDrc_data->gainSetIndex[i] = 1;
        drcCoefficientsUniDrc_data->gainCodingProfile[i] = impegh_read_bits_buf(ptr_bit_buf, 2);
        drcCoefficientsUniDrc_data->gainInterpolationType[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
        drcCoefficientsUniDrc_data->fullFrame[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
        drcCoefficientsUniDrc_data->timeAlignment[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
        drcCoefficientsUniDrc_data->timeDeltaMinPresent[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
        if (drcCoefficientsUniDrc_data->timeDeltaMinPresent[i] == 1)
        {
            drcCoefficientsUniDrc_data->bsTimeDeltaMin[i] = impegh_read_bits_buf(ptr_bit_buf, 11);
        }
        if (drcCoefficientsUniDrc_data->gainCodingProfile[i] == 3)
        {
            drcCoefficientsUniDrc_data->bandCount[i] = 1;
        }
        else
        {
            drcCoefficientsUniDrc_data->bandCount[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
            if (drcCoefficientsUniDrc_data->bandCount[i] > 1)
            {
                drcCoefficientsUniDrc_data->drcBandType[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
            }
            for (int j = 0; j < drcCoefficientsUniDrc_data->bandCount[i]; j++)
            {
                drcCoefficientsUniDrc_data->drcCharacteristic[i][j] = impegh_read_bits_buf(ptr_bit_buf, 7);
            }
            for (int j = 1; j < drcCoefficientsUniDrc_data->bandCount[i]; j++)
            {
                if (drcCoefficientsUniDrc_data->drcBandType[i] == 1)
                {
                    drcCoefficientsUniDrc_data->crossoverFreqIndex[i][j] = impegh_read_bits_buf(ptr_bit_buf, 4);
                }
                else
                {
                    drcCoefficientsUniDrc_data->startSubBandIndex[i][j] = impegh_read_bits_buf(ptr_bit_buf, 10);
                }
            }
        }
        drcCoefficientsUniDrc_data->gainSequenceCount += drcCoefficientsUniDrc_data->bandCount[i];
    }
}

static ia_downmixInstruction* selectDownmixInstructions(ia_mpegh3daUniDrcConfig *mpegh3daUniDrcConfig_data, WORD32 dmix_id)
{
  for (int i = 0; i < mpegh3daUniDrcConfig_data->downmixInstructionsCount; i++)
  {
    if (mpegh3daUniDrcConfig_data->downmixInstructions[i].downmixId == dmix_id)
    {
      return &(mpegh3daUniDrcConfig_data->downmixInstructions[i]);
    }
  }
  return NULL;
}

static VOID drcInstructionsUniDrc(ia_mpegh3daUniDrcConfig *mpegh3daUniDrcConfig_data, WORD32 idx, ia_bit_buf_struct *ptr_bit_buf, WORD32 baseChannelCount)
{
  WORD32 channel_count = 0;
  ia_drcInstructionsUniDrc *drcInstructionsUniDrc_data = &mpegh3daUniDrcConfig_data->drcInstructionsUniDrc_data[idx];
  drcInstructionsUniDrc_data->drcSetId = impegh_read_bits_buf(ptr_bit_buf, 6);
  drcInstructionsUniDrc_data->drcLocation = impegh_read_bits_buf(ptr_bit_buf, 4);
  drcInstructionsUniDrc_data->downmixId = impegh_read_bits_buf(ptr_bit_buf, 7);
  drcInstructionsUniDrc_data->additionalDownmixIdPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (drcInstructionsUniDrc_data->additionalDownmixIdPresent == 1)
  {
      drcInstructionsUniDrc_data->additionalDownmixIdCount = impegh_read_bits_buf(ptr_bit_buf, 3);
      for (int j = 0; j < drcInstructionsUniDrc_data->additionalDownmixIdCount; j++)
      {
          drcInstructionsUniDrc_data->additionalDownmixId[j] = impegh_read_bits_buf(ptr_bit_buf, 7);
      }
  }
  else
  {
      drcInstructionsUniDrc_data->additionalDownmixIdCount = 0;
  }
  drcInstructionsUniDrc_data->drcSetEffect = impegh_read_bits_buf(ptr_bit_buf, 16);
  if ((drcInstructionsUniDrc_data->drcSetEffect & (3 << 10)) == 0)
  {
      drcInstructionsUniDrc_data->limiterPeakTargetPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
      if (drcInstructionsUniDrc_data->limiterPeakTargetPresent == 1)
      {
          drcInstructionsUniDrc_data->bsLimiterPeakTarget = impegh_read_bits_buf(ptr_bit_buf, 8);
      }
  }

  drcInstructionsUniDrc_data->drcSetTargetLoudnessPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (drcInstructionsUniDrc_data->drcSetTargetLoudnessPresent == 1)
  {
      drcInstructionsUniDrc_data->bsDrcSetTargetLoudnessValueUpper = impegh_read_bits_buf(ptr_bit_buf, 6);
      drcInstructionsUniDrc_data->drcSetTargetLoudnessValueLowerPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
      if (drcInstructionsUniDrc_data->drcSetTargetLoudnessValueLowerPresent == 1)
      {
          drcInstructionsUniDrc_data->bsDrcSetTargetLoudnessValueLower = impegh_read_bits_buf(ptr_bit_buf, 6);
      }
  }
  drcInstructionsUniDrc_data->dependsOnDrcSetPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (drcInstructionsUniDrc_data->dependsOnDrcSetPresent == 1)
  {
      drcInstructionsUniDrc_data->dependsOnDrcSet = impegh_read_bits_buf(ptr_bit_buf, 6);
  }
  else
  {
      drcInstructionsUniDrc_data->noIndependentUse = impegh_read_bits_buf(ptr_bit_buf, 1);
  }
  channel_count = baseChannelCount;
  if ((drcInstructionsUniDrc_data->drcSetEffect & (3 << 10)) != 0)
  {
      for (int i = 0; i < channel_count; i++)
      {
          drcInstructionsUniDrc_data->bsGainSetIndex[i] = impegh_read_bits_buf(ptr_bit_buf, 6);
          drcInstructionsUniDrc_data->duckingScalingPresent[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          if (drcInstructionsUniDrc_data->duckingScalingPresent[i] == 1)
          {
              drcInstructionsUniDrc_data->bsDuckingScaling[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
          }
          drcInstructionsUniDrc_data->repeatParameters[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          if (drcInstructionsUniDrc_data->repeatParameters[i])
          {
              drcInstructionsUniDrc_data->bsRepeatParametersCount[i] = impegh_read_bits_buf(ptr_bit_buf, 5);
              i = i + drcInstructionsUniDrc_data->bsRepeatParametersCount[i] + 1;
          }
      }
  }
  else
  {
      WORD32 dmix_id = drcInstructionsUniDrc_data->downmixId;
      WORD32 additional_dmix_id_cnt = drcInstructionsUniDrc_data->additionalDownmixIdCount;
      if (dmix_id != 0 && dmix_id != 0x7F && additional_dmix_id_cnt == 0)
      {
          ia_downmixInstruction *pDmix = selectDownmixInstructions(mpegh3daUniDrcConfig_data, dmix_id);
          channel_count = pDmix->targetChannelCount;
      }
      else if (dmix_id != 0x7F || additional_dmix_id_cnt != 0)
      {
          channel_count = 1;
      }
      for (int i = 0; i < channel_count; i++)
      {
          drcInstructionsUniDrc_data->bsGainSetIndex[i] = impegh_read_bits_buf(ptr_bit_buf, 6);
          drcInstructionsUniDrc_data->repeatGainSetIndex[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          if (drcInstructionsUniDrc_data->repeatGainSetIndex[i])
          {
              drcInstructionsUniDrc_data->bsRepeatGainSetIndexCount[i] = impegh_read_bits_buf(ptr_bit_buf, 5);
              i = i + drcInstructionsUniDrc_data->bsRepeatGainSetIndexCount[i] + 1;
          }
      }
      WORD32 nDrcChannelGroups = 0;  // calculateNumDrcChannelGroups (Derivation of drcChannelGroups from gainSetIndices)
      for (int i = 0; i < nDrcChannelGroups; i++)
      {
          drcInstructionsUniDrc_data->gainScalingPresent[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          if (drcInstructionsUniDrc_data->gainScalingPresent[i] == 1)
          {
              drcInstructionsUniDrc_data->bsAttenuationScaling[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
              drcInstructionsUniDrc_data->bsAmplificationScaling[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
          }
          drcInstructionsUniDrc_data->gainOffsetPresent[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          if (drcInstructionsUniDrc_data->gainOffsetPresent[i] == 1)
          {
              drcInstructionsUniDrc_data->bsGainOffset[i] = impegh_read_bits_buf(ptr_bit_buf, 6);
          }
      }
  }
}

static VOID uniDrcConfigExtension(ia_uniDrcConfigExtension *uniDrcConfigExtension_data, ia_bit_buf_struct *ptr_bit_buf)
{
  WORD8 extSizeBits;
  WORD32 extBitSize;
  uniDrcConfigExtension_data->uniDrcConfigExtType = impegh_read_bits_buf(ptr_bit_buf, 4);
  while (uniDrcConfigExtension_data->uniDrcConfigExtType != UNIDRCCONFEXT_TERM)
  {
    uniDrcConfigExtension_data->bitSizeLen = impegh_read_bits_buf(ptr_bit_buf, 4);
    extSizeBits = uniDrcConfigExtension_data->bitSizeLen + 4;
    uniDrcConfigExtension_data->bitSize = impegh_read_bits_buf(ptr_bit_buf, extSizeBits);
    extBitSize = uniDrcConfigExtension_data->bitSize + 1;
    switch (uniDrcConfigExtension_data->uniDrcConfigExtType)
    {
    /* skipping bits for UNIDRCCONFEXT_PARAM_DRC and UNIDRCCONFEXT_V1*/
    case UNIDRCCONFEXT_PARAM_DRC:
    case UNIDRCCONFEXT_V1:
    default:
      for (int i = 0; i < extBitSize; i++)
      {
        uniDrcConfigExtension_data->otherBit[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
      }
    }
    uniDrcConfigExtension_data->uniDrcConfigExtType = impegh_read_bits_buf(ptr_bit_buf, 4);
  }
}

static VOID loudnessInfo(ia_bit_buf_struct *ptr_bit_buf, ia_loudnessInfo *loudnessInfo, WORD8 version)
{
  loudnessInfo->version = version;
  loudnessInfo->drcSetId = impegh_read_bits_buf(ptr_bit_buf, 6);
  if (version == 1)
  {
    loudnessInfo->eqSetId = impegh_read_bits_buf(ptr_bit_buf, 6);
  }
  loudnessInfo->downmixId = impegh_read_bits_buf(ptr_bit_buf, 7);
  loudnessInfo->samplePeakLevelPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (loudnessInfo->samplePeakLevelPresent)
  {
    loudnessInfo->bsSamplePeakLevel = impegh_read_bits_buf(ptr_bit_buf, 12);
  }
  loudnessInfo->truePeakLevelPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (loudnessInfo->truePeakLevelPresent)
  {
    loudnessInfo->bsTruePeakLevel = impegh_read_bits_buf(ptr_bit_buf, 12);
    loudnessInfo->measurementSystem = impegh_read_bits_buf(ptr_bit_buf, 4);
    loudnessInfo->reliability = impegh_read_bits_buf(ptr_bit_buf, 2);
  }
  loudnessInfo->measurementCount = impegh_read_bits_buf(ptr_bit_buf, 4);
  for (int i = 0; i < loudnessInfo->measurementCount; i++)
  {
    WORD32 esc_bits_read;
    loudnessInfo->methodDefinition[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
    loudnessInfo->methodValue[i] = impegh_read_escape_value(ptr_bit_buf, 2, 6, 0, &esc_bits_read);
    loudnessInfo->measurementSystemArr[i] = impegh_read_bits_buf(ptr_bit_buf, 4);
    loudnessInfo->reliabilityArr[i] = impegh_read_bits_buf(ptr_bit_buf, 2);
  }
}

static VOID loudnessInfoSetExtension(ia_bit_buf_struct *ptr_bit_buf, ia_loudnessInfoSetExtension *loudnessInfoSetExtension)
{
  WORD8 extSizeBits;
  WORD32 extBitSize;
  loudnessInfoSetExtension->loudnessInfoSetExtType = impegh_read_bits_buf(ptr_bit_buf, 4);
  while (loudnessInfoSetExtension->loudnessInfoSetExtType != UNIDRCLOUDEXT_TERM)
  {
    loudnessInfoSetExtension->bitSizeLen = impegh_read_bits_buf(ptr_bit_buf, 4);
    extSizeBits = loudnessInfoSetExtension->bitSizeLen + 4;
    loudnessInfoSetExtension->bitSize = impegh_read_bits_buf(ptr_bit_buf, extSizeBits);
    extBitSize = loudnessInfoSetExtension->bitSize + 1;
    switch (loudnessInfoSetExtension->loudnessInfoSetExtType)
    {
    case UNIDRCLOUDEXT_EQ:
      loudnessInfoSetExtension->loudnessInfoV1AlbumCount = impegh_read_bits_buf(ptr_bit_buf, 6);
      loudnessInfoSetExtension->loudnessInfoV1Count = impegh_read_bits_buf(ptr_bit_buf, 6);
      for (int i = 0; i < loudnessInfoSetExtension->loudnessInfoV1AlbumCount; i++)
      {
        loudnessInfo(ptr_bit_buf, &loudnessInfoSetExtension->loudnessInfoV1Album[i], 1);
      }
      for (int i = 0; i < loudnessInfoSetExtension->loudnessInfoV1Count; i++)
      {
        loudnessInfo(ptr_bit_buf, &loudnessInfoSetExtension->loudnessInfoV1[i], 1);
      }
      break;
    /* add future extensions here */
    default:
      for (int i = 0; i < extBitSize; i++)
      {
        loudnessInfoSetExtension->otherBit[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
      }
    }
    loudnessInfoSetExtension->loudnessInfoSetExtType = impegh_read_bits_buf(ptr_bit_buf, 4);
  }
}

static VOID mpegh3daLoudnessInfoSet(ia_mpegh3daLoudnessInfoSet *uniDrcConfigExtension_data, ia_bit_buf_struct *ptr_bit_buf)
{
  uniDrcConfigExtension_data->loudnessInfoCount = impegh_read_bits_buf(ptr_bit_buf, 6);
  for (int i = 0; i < uniDrcConfigExtension_data->loudnessInfoCount; i++)
  {
    uniDrcConfigExtension_data->loudnessInfoType[i] = impegh_read_bits_buf(ptr_bit_buf, 2);
    if (uniDrcConfigExtension_data->loudnessInfoType[i] == 1 || uniDrcConfigExtension_data->loudnessInfoType[i] == 2)
    {
      uniDrcConfigExtension_data->mae_groupID[i] = impegh_read_bits_buf(ptr_bit_buf, 7);
    }
    else
    {
      uniDrcConfigExtension_data->mae_groupPresetID[i] = impegh_read_bits_buf(ptr_bit_buf, 5);
    }
    loudnessInfo(ptr_bit_buf, &uniDrcConfigExtension_data->loudnessInfo[i], 0);
  }
  uniDrcConfigExtension_data->loudnessInfoAlbumPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (uniDrcConfigExtension_data->loudnessInfoAlbumPresent)
  {
    uniDrcConfigExtension_data->loudnessInfoAlbumCount = impegh_read_bits_buf(ptr_bit_buf, 6);
    for (int i = 0; i < uniDrcConfigExtension_data->loudnessInfoAlbumCount; i++)
    {
      uniDrcConfigExtension_data->loudnessInfoType[i] = 0;
      loudnessInfo(ptr_bit_buf, &uniDrcConfigExtension_data->loudnessInfoAlbum[i], 0);
    }
  }
  uniDrcConfigExtension_data->loudnessInfoSetExtensionPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (uniDrcConfigExtension_data->loudnessInfoSetExtensionPresent)
  {
    loudnessInfoSetExtension(ptr_bit_buf, &uniDrcConfigExtension_data->loudnessInfoSetExtension);
  }
}

static VOID mpegh3daUniDrcConfig(ia_mpegh3daExtElementConfig *extElementConfig, ia_bit_buf_struct *ptr_bit_buf)
{
  ia_mpegh3daUniDrcConfig* mpegh3daUniDrcConfig_data = &extElementConfig->mpegh3daUniDrcConfig_data;
  WORD32 baseChannelCount;
  mpegh3daUniDrcConfig_data->drcCoefficientsUniDrcCount = impegh_read_bits_buf(ptr_bit_buf, 3);
  mpegh3daUniDrcConfig_data->drcInstructionsUniDrcCount = impegh_read_bits_buf(ptr_bit_buf, 6);
  mpegh3daUniDrcConfig_data->mpegh3daUniDrcChannelLayout_data.baseChannelCount = impegh_read_bits_buf(ptr_bit_buf, 7);
  baseChannelCount = mpegh3daUniDrcConfig_data->mpegh3daUniDrcChannelLayout_data.baseChannelCount;
  for (int i = 0; i < mpegh3daUniDrcConfig_data->drcCoefficientsUniDrcCount; i++)
  {
      drcCoefficientsUniDrc(&mpegh3daUniDrcConfig_data->drcCoefficientsUniDrc_data[i], ptr_bit_buf);
  }
  for (int i = 0; i < mpegh3daUniDrcConfig_data->drcInstructionsUniDrcCount; i++)
  {
      mpegh3daUniDrcConfig_data->drcInstructionsType[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
      if (mpegh3daUniDrcConfig_data->drcInstructionsType[i] == 0)
      {
          mpegh3daUniDrcConfig_data->mae_groupID[i] = -1;
          mpegh3daUniDrcConfig_data->mae_groupPresetID[i] = -1;
      }
      else
      {
          mpegh3daUniDrcConfig_data->drcInstructionsType[i] = impegh_read_bits_buf(ptr_bit_buf, 1);
          mpegh3daUniDrcConfig_data->drcInstructionsType[i] =
              mpegh3daUniDrcConfig_data->drcInstructionsType[i] | (1 << 1);
          if (mpegh3daUniDrcConfig_data->drcInstructionsType[i] == 3)
          {
              mpegh3daUniDrcConfig_data->mae_groupPresetID[i] = impegh_read_bits_buf(ptr_bit_buf, 5);
          }
          else if (mpegh3daUniDrcConfig_data->drcInstructionsType[i] == 2)
          {
              mpegh3daUniDrcConfig_data->mae_groupID[i] = impegh_read_bits_buf(ptr_bit_buf, 7);
          }
          drcInstructionsUniDrc(mpegh3daUniDrcConfig_data, i, ptr_bit_buf, baseChannelCount);
      }
  }
  mpegh3daUniDrcConfig_data->uniDrcConfigExtPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (mpegh3daUniDrcConfig_data->uniDrcConfigExtPresent == 1)
  {
      uniDrcConfigExtension(&mpegh3daUniDrcConfig_data->uniDrcConfigExtension_data, ptr_bit_buf);
  }
  mpegh3daUniDrcConfig_data->loudnessInfoSetPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (mpegh3daUniDrcConfig_data->loudnessInfoSetPresent == 1)
  {
      mpegh3daLoudnessInfoSet(&mpegh3daUniDrcConfig_data->mpegh3daLoudnessInfoSet_data, ptr_bit_buf);
  }

}

/**impegh_create_init_bit_buf
 *
 *  \brief Create bit buffer reading structure.
 *
 *  \param [in,out] it_bit_buff      Pointer to bit buffer handle.
 *  \param [in]     ptr_bit_buf_base Pointer to bit buffer base.
 *  \param [in]     bit_buf_size     Bit buffer size.
 *
 *  \return VOID
 *
 */
VOID impegh_create_init_bit_buf(ia_bit_buf_struct *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                WORD32 bit_buf_size)
{
  impegh_create_bit_buf(it_bit_buff, ptr_bit_buf_base, bit_buf_size);
  it_bit_buff->cnt_bits = (bit_buf_size << 3);
  return;
}
/**
 *  impegh_audio_scene_info_process_parse
 *
 *  \brief Parse and write the asi elements to intermediate buffers
 *
 *  \param [in] ptr_bit_buf Pointer to bit buffer handle
 *  \param [in,out] header_info Pointer to header info structure
 *
 *  \return IA_ERRORCODE
 */

IA_ERRORCODE impegh_audio_scene_info_process(ia_bit_buf_struct *ptr_bit_buf, packet_info *header_info)
{
  WORD32 tmp = 0, tmp_value = 0;
  ia_bit_buf_struct ptr_maeg_buf;
  ia_bit_buf_struct ptr_maes_buf;
  ia_bit_buf_struct ptr_maep_buf;
  ia_bit_buf_struct ptr_mael_buf;
  ia_audio_scene_data audio_scene_data = { 0 };



  //////////Parsing///////////////

  audio_scene_data.mae_isMainStream = impegh_read_bits_buf(ptr_bit_buf, 1);;
  if (audio_scene_data.mae_isMainStream) // main_stream_flag
  {
    audio_scene_data.mae_audioSceneInfoIDPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
    if (audio_scene_data.mae_audioSceneInfoIDPresent)
    {
      audio_scene_data.mae_audioSceneInfoID = impegh_read_bits_buf(ptr_bit_buf, 8); // asi_id
    }


    /* ASI - Group Definition*/
    audio_scene_data.mae_numGroups = impegh_read_bits_buf(ptr_bit_buf, 7); // no:of groups
    if (audio_scene_data.mae_numGroups)
    {
      header_info->maei_present = 1;
    }
    else
    {
      header_info->maei_present = 0;
      return 0;
    }
    if (audio_scene_data.mae_numGroups > MAX_NUM_GROUPS)
    {
      return -1;
    }
    impegh_mae_asi_group_def_parse(&audio_scene_data,ptr_bit_buf);


    /* ASI - Switch Group Definition*/
    audio_scene_data.mae_numSwitchGroups = impegh_read_bits_buf(ptr_bit_buf, 5);
    if (audio_scene_data.mae_numSwitchGroups > MAX_NUM_SWITCH_GROUPS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    impegh_mae_asi_switch_group_def_parse(&audio_scene_data, ptr_bit_buf);


    /* ASI - Group presets Definition*/
    audio_scene_data.mae_numGroupPresets = impegh_read_bits_buf(ptr_bit_buf, 5);
    if (audio_scene_data.mae_numGroupPresets > MAX_NUM_GROUPS_PRESETS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    impegh_mae_asi_group_presets_def_parse(&audio_scene_data, ptr_bit_buf);


    /* ASI - MAE Data*/
    ia_bit_buf_struct ptr_mael_buf;
    impegh_create_mp4_buffer(&ptr_mael_buf, &(header_info->mael_buff[0]),
                             sizeof(header_info->mael_buff), 1);
    impegh_mae_asi_data_parse(&audio_scene_data, ptr_bit_buf);
    audio_scene_data.mae_metaDataElementIDoffset = 0;
    audio_scene_data.mae_metaDataElementIDmaxAvail = impegh_read_bits_buf(ptr_bit_buf, 7); // id max avail
  }
  else
  {
    audio_scene_data.mae_metaDataElementIDoffset = impegh_read_bits_buf(ptr_bit_buf, 7) + 1;
    audio_scene_data.mae_metaDataElementIDmaxAvail = impegh_read_bits_buf(ptr_bit_buf, 7);
  }



  ////Writing part for box data////
  impegh_create_mp4_buffer(&ptr_maeg_buf, &(header_info->maeg_buff[0]),
    sizeof(header_info->maeg_buff), 1);
  impegh_create_mp4_buffer(&ptr_maes_buf, &(header_info->maes_buff[0]),
    sizeof(header_info->maes_buff), 1);
  impegh_create_mp4_buffer(&ptr_maep_buf, &(header_info->maep_buff[0]),
    sizeof(header_info->maep_buff), 1);
  impegh_create_mp4_buffer(&ptr_mael_buf, &(header_info->mael_buff[0]),
    sizeof(header_info->mael_buff), 1);

  if (header_info->maei_present) // main_stream_flag
  {
    //maeG box
    if (audio_scene_data.mae_numGroups > MAX_NUM_GROUPS)
    {
      return -1;
    }
    impegh_write_bits_buf(&ptr_maeg_buf, audio_scene_data.mae_audioSceneInfoID, 8); // writing asi_id
    impegh_write_bits_buf(&ptr_maeg_buf, audio_scene_data.mae_numGroups,8); //reserve group
    impegh_mae_asi_group_def_write(&audio_scene_data, &ptr_maeg_buf);
    header_info->maeg_bits = ptr_maeg_buf.cnt_bits;

    //maeS box
    if (audio_scene_data.mae_numSwitchGroups > MAX_NUM_SWITCH_GROUPS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    impegh_write_bits_buf(&ptr_maes_buf, audio_scene_data.mae_numSwitchGroups, 8); // 3 reserved bits + 5 number of groups
    impegh_mae_asi_switch_group_def_write(&audio_scene_data, &ptr_maes_buf);
    header_info->maes_bits = ptr_maes_buf.cnt_bits;

    //maeP box
    if (audio_scene_data.mae_numGroupPresets > MAX_NUM_GROUPS_PRESETS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    impegh_write_bits_buf(&ptr_maep_buf, audio_scene_data.mae_numGroupPresets, 8); // 3 reserved bits + 5 number of groups
    impegh_mae_asi_group_presets_def_write(&audio_scene_data, &ptr_maep_buf);
    header_info->maep_bits = ptr_maep_buf.cnt_bits;

    //maeL box
    impegh_mae_asi_data_write(&audio_scene_data, &ptr_mael_buf);
    header_info->mael_bits = ptr_mael_buf.cnt_bits;

  }
  return IA_NO_ERROR;
}

WORD32 mpegh3daFlexibleSpeakerConfig(ia_bit_buf_struct *ptr_bit_buf,ia_SpeakerConfig3d *SpeakerConfig3d_data)
{
  WORD32 num_speaker;
  WORD32 angularPrecision;
  ia_mpegh3daSpeakerDescription *pstr_mpegh3daSpeakerDescription_data;
  SpeakerConfig3d_data->mpegh3daFlexibleSpeakerConfig_data.angularPrecision = impegh_read_bits_buf(ptr_bit_buf, 1);//angularPrecision
  angularPrecision = SpeakerConfig3d_data->mpegh3daFlexibleSpeakerConfig_data.angularPrecision;
  num_speaker = SpeakerConfig3d_data->numSpeakers;
  for (int i = 0; i < num_speaker; i++)
  {
    pstr_mpegh3daSpeakerDescription_data = &SpeakerConfig3d_data->mpegh3daFlexibleSpeakerConfig_data.mpegh3daSpeakerDescription_data[i];
    pstr_mpegh3daSpeakerDescription_data->isCICPspeakerIdx = impegh_read_bits_buf(ptr_bit_buf, 1);//isCICPspeakerIdx
    if (pstr_mpegh3daSpeakerDescription_data->isCICPspeakerIdx)
    {
      SpeakerConfig3d_data->CICPspeakerIdx[i] = impegh_read_bits_buf(ptr_bit_buf, 7);//CICPspeakerIdx
    }
    else
    {
      pstr_mpegh3daSpeakerDescription_data->ElevationClass = impegh_read_bits_buf(ptr_bit_buf, 2);//ElevationClass
      if (pstr_mpegh3daSpeakerDescription_data->ElevationClass == 3)
      {
        pstr_mpegh3daSpeakerDescription_data->ElevationAngleIdx = impegh_read_bits_buf(ptr_bit_buf, angularPrecision ? 7 : 5);
        if (pstr_mpegh3daSpeakerDescription_data->ElevationAngleIdx != 0 /* Table 44 */) {
          pstr_mpegh3daSpeakerDescription_data->ElevationDirection = impegh_read_bits_buf(ptr_bit_buf, 1); /* ElevationDirection */
          if (pstr_mpegh3daSpeakerDescription_data->ElevationDirection) {
            pstr_mpegh3daSpeakerDescription_data->ElevationAngleIdx = -pstr_mpegh3daSpeakerDescription_data->ElevationAngleIdx;
          }
        }
        pstr_mpegh3daSpeakerDescription_data->el = pstr_mpegh3daSpeakerDescription_data->ElevationAngleIdx * (angularPrecision ? 1 : 5);

        pstr_mpegh3daSpeakerDescription_data->AzimuthAngleIdx = impegh_read_bits_buf(ptr_bit_buf, angularPrecision ? 8 : 6);
        pstr_mpegh3daSpeakerDescription_data->az = pstr_mpegh3daSpeakerDescription_data->AzimuthAngleIdx * (angularPrecision ? 1 : 5);
        if ((pstr_mpegh3daSpeakerDescription_data->az != 0) && (pstr_mpegh3daSpeakerDescription_data->az != 180))
        {
          pstr_mpegh3daSpeakerDescription_data->AzimuthDirection = impegh_read_bits_buf(ptr_bit_buf, 1); /* AzimuthDirection */
          if (pstr_mpegh3daSpeakerDescription_data->AzimuthDirection) {
            pstr_mpegh3daSpeakerDescription_data->az = -pstr_mpegh3daSpeakerDescription_data->az;
          }
        }

        pstr_mpegh3daSpeakerDescription_data->isLFE = impegh_read_bits_buf(ptr_bit_buf, 1); /* isLFE; */
        if ((pstr_mpegh3daSpeakerDescription_data->az != 0) && (pstr_mpegh3daSpeakerDescription_data->az != 180))
        {
          SpeakerConfig3d_data->mpegh3daFlexibleSpeakerConfig_data.alsoAddSymmetricPair[i] = impegh_read_bits_buf(ptr_bit_buf, 1);  /* AzimuthDirection */
        }
      }
    }
  }
  return 0;
}

WORD32 SpeakerConfig3d(ia_bit_buf_struct *ptr_bit_buf, ia_SpeakerConfig3d *SpeakerConfig3d_data)
{
  SpeakerConfig3d_data->speakerLayoutType = impegh_read_bits_buf(ptr_bit_buf, 2); // speakerLayoutType
  SpeakerConfig3d_data->CICPspeakerLayoutIdx = 0;
  if (SpeakerConfig3d_data->speakerLayoutType == 0)
  {
    SpeakerConfig3d_data->CICPspeakerLayoutIdx = impegh_read_bits_buf(ptr_bit_buf, 6); // CICPspeakerLayoutIdx
  }
  else
  {
    WORD32 escap_buffer_bist;
    SpeakerConfig3d_data->numSpeakers = impegh_read_escape_value(ptr_bit_buf, 5, 8, 16, &escap_buffer_bist) + 1; //num_speaker
    if (SpeakerConfig3d_data->speakerLayoutType == 1)
    {
      for (int i = 0; i < SpeakerConfig3d_data->numSpeakers; i++)
      {
        SpeakerConfig3d_data->CICPspeakerIdx[i] = impegh_read_bits_buf(ptr_bit_buf, 7); // CICPspeakerIdx 
      }
    }
    else if (SpeakerConfig3d_data->speakerLayoutType == 2)
    {
        mpegh3daFlexibleSpeakerConfig(ptr_bit_buf, SpeakerConfig3d_data);
    }
    else if (SpeakerConfig3d_data->speakerLayoutType == 3)
    {
      return 0;
      /* speakerLayoutType == 3 is Contribution Mode */
    }
  }
  return SpeakerConfig3d_data->CICPspeakerLayoutIdx;
}

WORD32 FrameworkConfig3d(ia_bit_buf_struct *ptr_bit_buf, ia_3d_audio_cnfg_data *audio_config_data)
{
  ia_FrameworkConfig3d *pstr_FrameworkConfig3d_data;
  pstr_FrameworkConfig3d_data = &audio_config_data->FrameworkConfig3d_data;
  //Signals3d()
  {
    int grp, signalGroupType_grp_prev = 0;
    WORD32 escap_buffer_bist;
    int sigIdx = 0;
    pstr_FrameworkConfig3d_data->numAudioChannels = 0;
    pstr_FrameworkConfig3d_data->numAudioObjects = 0;
    pstr_FrameworkConfig3d_data->numSAOCTransportChannels = 0;
    pstr_FrameworkConfig3d_data->numHOATransportChannels = 0;
    pstr_FrameworkConfig3d_data->bsNumSignalGroups = impegh_read_bits_buf(ptr_bit_buf, 5) + 1;
    if (pstr_FrameworkConfig3d_data->bsNumSignalGroups > TP_MPEGH_MAX_SIGNAL_GROUPS) {
      return -1;
    }
    for (grp = 0; grp < pstr_FrameworkConfig3d_data->bsNumSignalGroups; grp++)
    {
      pstr_FrameworkConfig3d_data->signal_groupID[grp] = grp;
      pstr_FrameworkConfig3d_data->differsFromReferenceLayout[grp] = 0;
      pstr_FrameworkConfig3d_data->signalGroupType[grp] = impegh_read_bits_buf(ptr_bit_buf, 3);
      pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp] = impegh_read_escape_value(ptr_bit_buf, 5, 8, 16, &escap_buffer_bist) + 1;
      if ((sigIdx + pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp]) > TP_MPEGH_MAX_SIGNAL_GROUPS)
      {
        return -1;
      }
      if (pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp] > TP_MAX_CHANNELS_PER_SIGNAL_GROUP) {
        return -1;
      }
      switch (pstr_FrameworkConfig3d_data->signalGroupType[grp])
      {
      case 0:/* SignalGroupTypeChannels */
        pstr_FrameworkConfig3d_data->numAudioChannels += pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp];
        pstr_FrameworkConfig3d_data->differsFromReferenceLayout[grp] = impegh_read_bits_buf(ptr_bit_buf, 1);
        if (pstr_FrameworkConfig3d_data->differsFromReferenceLayout[grp])
        {
          pstr_FrameworkConfig3d_data->audioChannelLayout[grp] = SpeakerConfig3d(ptr_bit_buf, &pstr_FrameworkConfig3d_data->SpeakerConfig3d_data[grp]);
        }
        else
        {
          pstr_FrameworkConfig3d_data->audioChannelLayout[grp] = audio_config_data->referenceLayout;
        }
        break;
      case 1: /* SignalGroupTypeObject */
        pstr_FrameworkConfig3d_data->numAudioObjects += pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp];
        break;
      case 2:/* SignalGroupTypeSAOC */
        pstr_FrameworkConfig3d_data->numSAOCTransportChannels += pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp];
        pstr_FrameworkConfig3d_data->saocDmxLayoutPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
        if (pstr_FrameworkConfig3d_data->saocDmxLayoutPresent)
        {
          pstr_FrameworkConfig3d_data->saocDmxChannelLayout[grp] = SpeakerConfig3d(ptr_bit_buf, &pstr_FrameworkConfig3d_data->SpeakerConfig3d_data[grp]);
        }
        assert(0);//unsupport: to be teseted
        break;
      case 3: /* SignalGroupTypeHOA */
        pstr_FrameworkConfig3d_data->numHOATransportChannels += pstr_FrameworkConfig3d_data->bsNumberOfSignals[grp];
        break;
       default:
         assert(0);//unsupport 
      }
    }
  }
  audio_config_data->mpegh3daDecoderConfig_data.numAudioChannels = pstr_FrameworkConfig3d_data->numAudioChannels;
  audio_config_data->mpegh3daDecoderConfig_data.numAudioObjects = pstr_FrameworkConfig3d_data->numAudioObjects;
  audio_config_data->mpegh3daDecoderConfig_data.numSAOCTransportChannels = pstr_FrameworkConfig3d_data->numSAOCTransportChannels;
  audio_config_data->mpegh3daDecoderConfig_data.numHOATransportChannels = pstr_FrameworkConfig3d_data->numHOATransportChannels;
  return 0;
}

WORD32 mpegh3daCoreConfig(ia_bit_buf_struct *ptr_bit_buf, ia_mpegh3daCoreConfig *mpegh3daCoreConfig_data)
{
  mpegh3daCoreConfig_data->tw_mdct = impegh_read_bits_buf(ptr_bit_buf, 1);
  mpegh3daCoreConfig_data->fullbandLpd = impegh_read_bits_buf(ptr_bit_buf, 1);
  mpegh3daCoreConfig_data->noiseFilling = impegh_read_bits_buf(ptr_bit_buf, 1);
  mpegh3daCoreConfig_data->enhancedNoiseFilling = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (mpegh3daCoreConfig_data->enhancedNoiseFilling)
  {
    mpegh3daCoreConfig_data->igfUseEnf = impegh_read_bits_buf(ptr_bit_buf, 1);
    mpegh3daCoreConfig_data->igfUseHighRes = impegh_read_bits_buf(ptr_bit_buf, 1);
    mpegh3daCoreConfig_data->igfUseWhitening = impegh_read_bits_buf(ptr_bit_buf, 1);
    mpegh3daCoreConfig_data->igfAfterTnsSynth = impegh_read_bits_buf(ptr_bit_buf, 1);
    mpegh3daCoreConfig_data->igfStartIndex = impegh_read_bits_buf(ptr_bit_buf, 5);
    mpegh3daCoreConfig_data->igfStopIndex = impegh_read_bits_buf(ptr_bit_buf, 4);
  }

  return 0;
}

WORD32 mpegh3daSingleChannelElementConfig(ia_bit_buf_struct *ptr_bit_buf, ia_mpegh3daCoreConfig *mpegh3daCoreConfig_data)
{
  mpegh3daCoreConfig(ptr_bit_buf, mpegh3daCoreConfig_data);

  //add this SBR related block here
  //if (sbrRatioIndex > 0) {
  //  SbrConfig();
  //}
  return 0;
}

WORD32 mpegh3daChannelPairElementConfig(ia_bit_buf_struct *ptr_bit_buf, ia_mpegh3daCoreConfig *mpegh3daCoreConfig_data, WORD32 total_channels)
{
  WORD32 stereoConfigIndex;
  WORD32 nBits;
  WORD32 val;
  switch (total_channels - 1)
  {
  case 15:
  case 14:
  case 13:
  case 12:
  case 11:
  case 10:
  case 9:
  case 8:
    val = 3;
    break;
  case 7:
  case 6:
  case 5:
  case 4:
    val = 2;
    break;
  case 3:
  case 2:
    val = 1;
    break;
  case 1:
    val = 0;
    break;
  }
  nBits = (val + 1);

  mpegh3daCoreConfig(ptr_bit_buf, mpegh3daCoreConfig_data);
  if (mpegh3daCoreConfig_data->enhancedNoiseFilling)
  {
    mpegh3daCoreConfig_data->igfIndependentTiling = impegh_read_bits_buf(ptr_bit_buf, 1);
  }
  //add this block here
  if (0)//(sbrRatioIndex > 0) 
  {
    //SbrConfig();
  }
  else
  {
    stereoConfigIndex = 0;
  }
  //if (stereoConfigIndex > 0) 
  //{
  //  Mps212Config(stereoConfigIndex);
  //}

  mpegh3daCoreConfig_data->qceIndex = impegh_read_bits_buf(ptr_bit_buf, 2);
  if (mpegh3daCoreConfig_data->qceIndex > 0)
  {
    //Need to test this /* QCE not supported by MPEG-H LC Profile */ 
    mpegh3daCoreConfig_data->shiftIndex0 = impegh_read_bits_buf(ptr_bit_buf, 1);
    if (mpegh3daCoreConfig_data->shiftIndex0 > 0)
    {
      mpegh3daCoreConfig_data->shiftChannel0 = impegh_read_bits_buf(ptr_bit_buf, nBits);
    }
  }
  mpegh3daCoreConfig_data->shiftIndex1 = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (mpegh3daCoreConfig_data->shiftIndex1 > 0)
  {
    mpegh3daCoreConfig_data->shiftChannel1 = impegh_read_bits_buf(ptr_bit_buf, nBits);
  }
  //if (sbrRatioIndex == 0 && qceIndex == 0)
  if(1)
  {
    //check if sbr_ratio is added as a support
    mpegh3daCoreConfig_data->lpdStereoIndex = impegh_read_bits_buf(ptr_bit_buf, 1);
  }
  else
  {
    mpegh3daCoreConfig_data->lpdStereoIndex = 0;
  }
  return 0;
}

WORD32 mpegh3daLfeElementConfig(ia_mpegh3daCoreConfig *mpegh3daCoreConfig_data)
{
  mpegh3daCoreConfig_data->tw_mdct = 0;
  mpegh3daCoreConfig_data->fullbandLpd = 0;
  mpegh3daCoreConfig_data->noiseFilling = 0;
  mpegh3daCoreConfig_data->enhancedNoiseFilling = 0;
  return 0;
}

WORD32 mpegh3daExtElementConfig(ia_bit_buf_struct *ptr_bit_buf, ia_mpegh3daCoreConfig *mpegh3daCoreConfig_data, int numAudioObjects, WORD8 *uniDrcConfigPresent)
{
  ia_mpegh3daExtElementConfig *pstr_mpegh3daExtElementConfig_data;
  WORD32 bits_used;
  WORD32 temp_bits_to_skip;
  WORD32 cnt_bits;
  pstr_mpegh3daExtElementConfig_data = &mpegh3daCoreConfig_data->mpegh3daExtElementConfig_data;
  pstr_mpegh3daExtElementConfig_data->usacExtElementType = impegh_read_escape_value(ptr_bit_buf, 4, 8, 16, &bits_used);
  pstr_mpegh3daExtElementConfig_data->usacExtElementConfigLength = impegh_read_escape_value(ptr_bit_buf, 4, 8, 16, &bits_used);
  pstr_mpegh3daExtElementConfig_data->usacExtElementDefaultLengthPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (pstr_mpegh3daExtElementConfig_data->usacExtElementDefaultLengthPresent)
  {
    pstr_mpegh3daExtElementConfig_data->usacExtElementDefaultLength = impegh_read_escape_value(ptr_bit_buf, 8, 16, 0, &bits_used) + 1;
  }
  pstr_mpegh3daExtElementConfig_data->usacExtElementPayloadFrag = impegh_read_bits_buf(ptr_bit_buf, 1);

  //Need to parse entire extension temp skipping the bits
  temp_bits_to_skip = pstr_mpegh3daExtElementConfig_data->usacExtElementConfigLength << 3;
  switch (pstr_mpegh3daExtElementConfig_data->usacExtElementType)
  {
  case ID_EXT_ELE_UNKNOWN:
  case ID_EXT_ELE_FILL:
    break;
  case ID_EXT_ELE_MPEGS:
  case ID_EXT_ELE_SAOC:
  case ID_EXT_ELE_SAOC_3D:
  case ID_EXT_ELE_HOA_ENH_LAYER:
  case ID_EXT_ELE_HREP:
    assert(0);
  case ID_EXT_ELE_TCC:
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    break;
  case ID_EXT_ELE_ENHANCED_OBJ_METADATA:
    cnt_bits = ptr_bit_buf->cnt_bits;
    EnhancedObjectMetadataConfig(pstr_mpegh3daExtElementConfig_data, ptr_bit_buf, numAudioObjects);
    cnt_bits = cnt_bits - ptr_bit_buf->cnt_bits;
    assert(cnt_bits < temp_bits_to_skip);
    temp_bits_to_skip = temp_bits_to_skip - cnt_bits;
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    break;
  case ID_EXT_ELE_AUDIOPREROLL:
    /* No configuration element */
    break;
  case ID_EXT_ELE_FMT_CNVRTR:
    /* No configuration element */
    break;
  case ID_EXT_ELE_OBJ_METADATA:
  {
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    //OAM data
    //Supported
  }
    /* parsing of ObjectMetadataConfig() */
    break;
  case ID_EXT_ELE_MCT:
  {
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    /* MCTConfig() */
    //Supported
  }
    break;
  case ID_EXT_ELE_HOA: 
  {
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    //HOA data
    //supported
  } 
  break;
  case ID_EXT_ELE_UNI_DRC: 
  {
    //mhaD box data
    cnt_bits = ptr_bit_buf->cnt_bits;
    *uniDrcConfigPresent = 1;
    mpegh3daUniDrcConfig(pstr_mpegh3daExtElementConfig_data, ptr_bit_buf);
    cnt_bits = cnt_bits - ptr_bit_buf->cnt_bits;
    assert(cnt_bits < temp_bits_to_skip);
    temp_bits_to_skip = temp_bits_to_skip - cnt_bits;
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    //DRC data
    //Supported
  } 
  break;
  case ID_EXT_ELE_PROD_METADATA:
    break;
  default:
    impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
    break;
  }
  return 0;
}

WORD32 mpegh3daDecoderConfig(ia_bit_buf_struct *ptr_bit_buf, ia_3d_audio_cnfg_data *audio_config_data)
{
  WORD32 bits_used;
  WORD8 uniDrcConfigPresent = 0;
  ia_mpegh3daDecoderConfig *pstr_mpegh3daDecoderConfig;
  pstr_mpegh3daDecoderConfig = &audio_config_data->mpegh3daDecoderConfig_data;
  pstr_mpegh3daDecoderConfig->numElements = impegh_read_escape_value(ptr_bit_buf, 4, 8, 16, &bits_used) + 1;
  pstr_mpegh3daDecoderConfig->elementLengthPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  WORD32 total_channels = pstr_mpegh3daDecoderConfig->numAudioChannels + pstr_mpegh3daDecoderConfig->numAudioObjects
    + pstr_mpegh3daDecoderConfig->numSAOCTransportChannels + pstr_mpegh3daDecoderConfig->numHOATransportChannels;
  for (int elemIdx = 0; elemIdx < pstr_mpegh3daDecoderConfig->numElements; elemIdx++)
  {
    pstr_mpegh3daDecoderConfig->usacElementType[elemIdx] = (MP4_ELEMENT_ID)(impegh_read_bits_buf(ptr_bit_buf, 2) |USAC_ID_BIT);
    switch (pstr_mpegh3daDecoderConfig->usacElementType[elemIdx])
    {
    case ID_USAC_SCE:
      mpegh3daSingleChannelElementConfig(ptr_bit_buf, &pstr_mpegh3daDecoderConfig->mpegh3daCoreConfig_data[elemIdx]);
      break;
    case ID_USAC_CPE:
      mpegh3daChannelPairElementConfig(ptr_bit_buf, &pstr_mpegh3daDecoderConfig->mpegh3daCoreConfig_data[elemIdx], total_channels);
      break;
    case ID_USAC_LFE:
      mpegh3daLfeElementConfig( &pstr_mpegh3daDecoderConfig->mpegh3daCoreConfig_data[elemIdx]);
      break;
    case ID_USAC_EXT:
      mpegh3daExtElementConfig(ptr_bit_buf, &pstr_mpegh3daDecoderConfig->mpegh3daCoreConfig_data[elemIdx], audio_config_data->mpegh3daDecoderConfig_data.numAudioObjects, &uniDrcConfigPresent);
      pstr_mpegh3daDecoderConfig->mpegh3daUniDrcConfigPresent = uniDrcConfigPresent;
      break;
    default:
      assert(0);//unknown
      break;
    }
  }
  return 0;
}
WORD32 CompatibleProfileLevelSet(ia_bit_buf_struct *ptr_bit_buf, ia_3d_audio_cnfg_data *audio_config_data)
{
  ia_CompatibleProfileLevelSet *pstr_CompatibleProfileLevelSet = &audio_config_data->mpegh3daConfigExtension_data.CompatibleProfileLevelSet_data;
  pstr_CompatibleProfileLevelSet->CompatibleProfileLevelSet_data_present = 1;//indication of mhaP box
  pstr_CompatibleProfileLevelSet->bsNumCompatibleSets = impegh_read_bits_buf(ptr_bit_buf, 4);
  pstr_CompatibleProfileLevelSet->numCompatibleSets = pstr_CompatibleProfileLevelSet->bsNumCompatibleSets + 1;
  impegh_read_bits_buf(ptr_bit_buf, 4);
  for (int idx = 0; idx < pstr_CompatibleProfileLevelSet->numCompatibleSets; idx++)
  {
    pstr_CompatibleProfileLevelSet->CompatibleSetIndication[idx] = impegh_read_bits_buf(ptr_bit_buf, 8);
  }
  return 0;
}

VOID downmixMatrixSet(ia_bit_buf_struct *ptr_bit_buf, ia_downmixMatrixSet *downmixMatrixSet)
{
  downmixMatrixSet->downmixIdCount = impegh_read_bits_buf(ptr_bit_buf, 5);
  for (int k = 0; k < downmixMatrixSet->downmixIdCount; k++)
  {
    downmixMatrixSet->downmixId[k] = impegh_read_bits_buf(ptr_bit_buf, 7);
    downmixMatrixSet->downmixType[k] = impegh_read_bits_buf(ptr_bit_buf, 2);
    if (downmixMatrixSet->downmixType[k] == 0)
    {
      downmixMatrixSet->CICPspeakerLayoutIdx[k] = impegh_read_bits_buf(ptr_bit_buf, 6);
    }
    else if (downmixMatrixSet->downmixType[k] == 1)
    {
      WORD32 esc_bits_read;
      downmixMatrixSet->CICPspeakerLayoutIdx[k] = impegh_read_bits_buf(ptr_bit_buf, 6);
      downmixMatrixSet->bsDownmixMatrixCount[k] = impegh_read_escape_value(ptr_bit_buf, 1, 3, 0, &esc_bits_read);
      for (int l = 0; l < downmixMatrixSet->bsDownmixMatrixCount[k]; l++)
      {
        downmixMatrixSet->bsNumAssignedGroupIDs[k][l] = impegh_read_escape_value(ptr_bit_buf, 1, 4, 4, &esc_bits_read);
        for (int m = 0; m < downmixMatrixSet->bsNumAssignedGroupIDs[k][l] + 1; m++) {
          downmixMatrixSet->signal_groupID[k][l][m] = impegh_read_bits_buf(ptr_bit_buf, 5);
        }
        downmixMatrixSet->dmxMatrixLenBits[k][l] = impegh_read_escape_value(ptr_bit_buf, 8, 8, 12, &esc_bits_read);
        // downmixMatrix(ptr_bit_buf);
        impegh_read_bits_buf(ptr_bit_buf, downmixMatrixSet->dmxMatrixLenBits[k][l]);  // skip bits for DownmixMatrix subroutine
      }
    }
  }
}

VOID downmixConfig(ia_bit_buf_struct *ptr_bit_buf, ia_downmixConfig *downmixConfig)
{
  downmixConfig->downmixConfigType = impegh_read_bits_buf(ptr_bit_buf, 2);
  if (downmixConfig->downmixConfigType == 0 || downmixConfig->downmixConfigType == 2)
  {
    downmixConfig->passiveDownmixFlag = impegh_read_bits_buf(ptr_bit_buf, 1);
    if (downmixConfig->passiveDownmixFlag == 0)
    {
      downmixConfig->phaseAlignStrength = impegh_read_bits_buf(ptr_bit_buf, 3);
    }
  }
  if (downmixConfig->downmixConfigType == 1 || downmixConfig->downmixConfigType == 2)
  {
    downmixMatrixSet(ptr_bit_buf, &downmixConfig->dowmixMatrixSet);
  }
}

WORD32 mpegh3daConfigExtension(ia_bit_buf_struct *ptr_bit_buf, ia_3d_audio_cnfg_data *audio_config_data)
{
  WORD32 bits_used, cnt_bits;
  WORD32 temp_bits_to_skip;
  ia_mpegh3daConfigExtension *pstr_mpegh3daConfigExtension_data = &audio_config_data->mpegh3daConfigExtension_data;
  pstr_mpegh3daConfigExtension_data->numConfigExtensions = impegh_read_escape_value(ptr_bit_buf, 2, 4, 8, &bits_used) +1 ;
  for (int confExtIdx = 0; confExtIdx < pstr_mpegh3daConfigExtension_data->numConfigExtensions; confExtIdx++)
  {
    pstr_mpegh3daConfigExtension_data->usacConfigExtType[confExtIdx] = impegh_read_escape_value(ptr_bit_buf, 4, 8, 16, &bits_used) ;
    pstr_mpegh3daConfigExtension_data->usacConfigExtLength[confExtIdx] = impegh_read_escape_value(ptr_bit_buf, 4, 8, 16, &bits_used) ;
    temp_bits_to_skip = pstr_mpegh3daConfigExtension_data->usacConfigExtLength[confExtIdx] << 3;
    switch (pstr_mpegh3daConfigExtension_data->usacConfigExtType[confExtIdx])
    {
    case ID_CONFIG_EXT_FILL:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_DOWNMIX:
      cnt_bits = ptr_bit_buf->cnt_bits;
      downmixConfig(ptr_bit_buf, &pstr_mpegh3daConfigExtension_data->downmixConfig);
      cnt_bits = cnt_bits - ptr_bit_buf->cnt_bits;
      assert(cnt_bits < temp_bits_to_skip);
      temp_bits_to_skip = temp_bits_to_skip - cnt_bits;
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_LOUDNESS_INFO:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_AUDIOSCENE_INFO:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_HOA_MATRIX:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_ICG:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_SIG_GROUP_INFO:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;
    case ID_CONFIG_EXT_COMPATIBLE_PROFILELVL_SET:
      CompatibleProfileLevelSet(ptr_bit_buf, audio_config_data);
      break;
    default:
      impegh_read_bits_buf(ptr_bit_buf, temp_bits_to_skip);
      break;

    }
  }
  return 0;
}

VOID ia_write_mhaP_dat(ia_bit_buf_struct *ptr_bit_buf, ia_CompatibleProfileLevelSet *pstr_CompatibleProfileLevelSet_data)
{
  impegh_write_bits_buf(ptr_bit_buf, pstr_CompatibleProfileLevelSet_data->numCompatibleSets, 8); // writing asi_id

  for (int i = 0; i < pstr_CompatibleProfileLevelSet_data->numCompatibleSets; i++) {
    impegh_write_bits_buf(ptr_bit_buf, pstr_CompatibleProfileLevelSet_data->CompatibleSetIndication[i], 8); //reserve group
  }
}

static VOID write_LoudnessBaseBox(ia_bit_buf_struct *ptr_bit_buf, ia_loudnessInfo *loudnessInfo)
{
  /* This is implemented as per definition of 'LoudnessBaseBox' defined in ISO/IEC 14496-12:2022(E) */
  WORD8 version = loudnessInfo->version;
  if (version >= 1)
  {
    impegh_write_bits_buf(ptr_bit_buf, 0, 2); // reserved
    impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->eqSetId, 6);
  }
  impegh_write_bits_buf(ptr_bit_buf, 0, 3); // reserved
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->downmixId, 7);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->drcSetId, 6);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->bsSamplePeakLevel, 12);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->bsTruePeakLevel, 12);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->measurementSystem, 4);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->reliability, 4);
  impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->measurementCount, 8);
  for (int i = 0; i < loudnessInfo->measurementCount; i++)
  {
    impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->methodDefinition[i], 8);
    impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->methodValue[i], 8);
    impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->measurementSystemArr[i], 4);
    impegh_write_bits_buf(ptr_bit_buf, loudnessInfo->reliabilityArr[i], 4);
  }
}

VOID ia_write_mhaD_dat(ia_bit_buf_struct *ptr_bit_buf, ia_3d_audio_cnfg_data *audio_config_data)
{
  /* Searching for USAC element index that corresponds to DRC config info. */
  WORD8 elem_idx_with_uni_drc_cnfg = -1;
  for (int elemIdx = 0; elemIdx < audio_config_data->mpegh3daDecoderConfig_data.numElements; elemIdx++)
  {
    if (audio_config_data->mpegh3daDecoderConfig_data.mpegh3daCoreConfig_data[elemIdx].mpegh3daExtElementConfig_data.usacExtElementType == ID_EXT_ELE_UNI_DRC)
    {
      elem_idx_with_uni_drc_cnfg = elemIdx;
      break;
    }
  }
  ia_mpegh3daUniDrcConfig *drcCfg = &audio_config_data->mpegh3daDecoderConfig_data
    .mpegh3daCoreConfig_data[elem_idx_with_uni_drc_cnfg].mpegh3daExtElementConfig_data.mpegh3daUniDrcConfig_data;
  WORD8 downmixIdCount = audio_config_data->mpegh3daConfigExtension_data.downmixConfig.dowmixMatrixSet.downmixIdCount;

  impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
  impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrcCount, 6);
  impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
  impegh_write_bits_buf(ptr_bit_buf, drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoCount, 6);
  impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
  impegh_write_bits_buf(ptr_bit_buf, drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoAlbumCount, 6);
  impegh_write_bits_buf(ptr_bit_buf, 0, 3);   // reserved
  impegh_write_bits_buf(ptr_bit_buf, downmixIdCount, 5);

  for (int i = 0; i < drcCfg->drcInstructionsUniDrcCount; i++)
  {
    impegh_write_bits_buf(ptr_bit_buf, 0, 6);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsType[i], 2);
    if (drcCfg->drcInstructionsType[i] == 2)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->mae_groupID[i], 7);
    }
    if (drcCfg->drcInstructionsType[i] == 3)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 3);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->mae_groupPresetID[i], 5);
    }
    impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].drcSetId, 6);
    impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].downmixId, 7);
    impegh_write_bits_buf(ptr_bit_buf, 0, 5);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].additionalDownmixIdCount, 3);
    for (int j = 0; j < drcCfg->drcInstructionsUniDrc_data[i].additionalDownmixIdCount; j++)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].additionalDownmixId[j], 7);
    }
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].drcSetEffect, 16);
    impegh_write_bits_buf(ptr_bit_buf, 0, 7);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].limiterPeakTargetPresent, 1);
    if (drcCfg->drcInstructionsUniDrc_data[i].limiterPeakTargetPresent == 1)
    {
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].bsLimiterPeakTarget, 8);
    }
    impegh_write_bits_buf(ptr_bit_buf, 0, 7);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].drcSetTargetLoudnessPresent, 1);
    if (drcCfg->drcInstructionsUniDrc_data[i].drcSetTargetLoudnessPresent == 1)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].bsDrcSetTargetLoudnessValueUpper, 6);
      impegh_write_bits_buf(ptr_bit_buf, 0, 2);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].bsDrcSetTargetLoudnessValueLower, 6);
    }
    impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].dependsOnDrcSet, 6);
    if (drcCfg->drcInstructionsUniDrc_data[i].dependsOnDrcSet)
    {
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->drcInstructionsUniDrc_data[i].noIndependentUse, 1);
    }
    else
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
    }
  }

  for (int i = 0; i < drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoCount; i++)
  {
    impegh_write_bits_buf(ptr_bit_buf, 0, 6);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoType[i], 2);
    if (drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoType[i] == 1 || drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoType[i] == 2)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->mpegh3daLoudnessInfoSet_data.mae_groupID[i], 7);
    }
    else if (drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoType[i] == 3)
    {
      impegh_write_bits_buf(ptr_bit_buf, 0, 3);   // reserved
      impegh_write_bits_buf(ptr_bit_buf, drcCfg->mpegh3daLoudnessInfoSet_data.mae_groupPresetID[i], 5);
    }
    write_LoudnessBaseBox(ptr_bit_buf, &drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfo[i]);
  }
  for (int i = 0; i < drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoAlbumCount; i++)
  {
    write_LoudnessBaseBox(ptr_bit_buf, &drcCfg->mpegh3daLoudnessInfoSet_data.loudnessInfoAlbum[i]);
  }
  for (int i = 0; i < downmixIdCount; i++)
  {
    impegh_write_bits_buf(ptr_bit_buf, 0, 1);   // reserved
    impegh_write_bits_buf(ptr_bit_buf, audio_config_data->mpegh3daConfigExtension_data.downmixConfig.dowmixMatrixSet.downmixId[i], 7);
    impegh_write_bits_buf(ptr_bit_buf, audio_config_data->mpegh3daConfigExtension_data.downmixConfig.dowmixMatrixSet.downmixType[i], 2);
    impegh_write_bits_buf(ptr_bit_buf, audio_config_data->mpegh3daConfigExtension_data.downmixConfig.dowmixMatrixSet.CICPspeakerLayoutIdx[i], 6);
  }
}

IA_ERRORCODE impegh_3d_audio_config_data_process(ia_bit_buf_struct *ptr_bit_buf, packet_info *header_info)
{
  WORD32 tmp = 0, tmp_value = 0;
  ia_bit_buf_struct ptr_mhaD_buf;
  ia_bit_buf_struct ptr_mhaP_buf;
  ia_3d_audio_cnfg_data* audio_config_data = NULL;

  audio_config_data = (ia_3d_audio_cnfg_data *)calloc(1, sizeof(ia_3d_audio_cnfg_data));

  //////////Parsing///////////////
  header_info->profile_info =
    impegh_read_bits_buf(ptr_bit_buf, 8); // read mpegh_3da_profile_lvl_indication
  audio_config_data->usacSamplingFrequencyIndex = impegh_read_bits_buf(ptr_bit_buf, 5);
  if (audio_config_data->usacSamplingFrequencyIndex == 0x1f)
  {
    audio_config_data->usacSamplingFrequency = impegh_read_bits_buf(ptr_bit_buf, 24);
    if (audio_config_data->usacSamplingFrequency > MAX_SAMPLE_RATE)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_SAMPLING_RATE;
    }
  }
  else
  {
    audio_config_data->usacSamplingFrequency =
      ia_sampling_rate_tbl[audio_config_data->usacSamplingFrequencyIndex]; /* Extract sampling rate from the
                                                         config packet */
  }
  audio_config_data->coreSbrFrameLengthIndex = impegh_read_bits_buf(
    ptr_bit_buf, 3); // coreSbrFrameLengthIndex

  audio_config_data->cfg_reserved = impegh_read_bits_buf(
    ptr_bit_buf, 1); // cfg_reserved
  audio_config_data->receiverDelayCompensation = impegh_read_bits_buf(
    ptr_bit_buf, 1); // cfg_reserved
  audio_config_data->referenceLayout = SpeakerConfig3d(ptr_bit_buf, &audio_config_data->SpeakerConfig3d_data);
  header_info->sampling_freq = audio_config_data->usacSamplingFrequency;
  header_info->spaker_layout = audio_config_data->referenceLayout;
  FrameworkConfig3d(ptr_bit_buf, audio_config_data);
  mpegh3daDecoderConfig(ptr_bit_buf, audio_config_data);
  audio_config_data->usacConfigExtensionPresent = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (audio_config_data->usacConfigExtensionPresent)
  {
    mpegh3daConfigExtension(ptr_bit_buf, audio_config_data);
  }


  ////Writing part for box data////
  impegh_create_mp4_buffer(&ptr_mhaD_buf, &(header_info->mhaD_buff[0]),
    sizeof(header_info->mhaD_buff), 1);
  impegh_create_mp4_buffer(&ptr_mhaP_buf, &(header_info->mhaP_buff[0]),
    sizeof(header_info->mhaP_buff), 1);

  //mhaP box data
  ia_CompatibleProfileLevelSet *pstr_CompatibleProfileLevelSet_data = &audio_config_data->mpegh3daConfigExtension_data.CompatibleProfileLevelSet_data;
  header_info->mhaP_data_present = pstr_CompatibleProfileLevelSet_data->CompatibleProfileLevelSet_data_present;
  if (pstr_CompatibleProfileLevelSet_data->CompatibleProfileLevelSet_data_present)
  {
    ia_write_mhaP_dat(&ptr_mhaP_buf, pstr_CompatibleProfileLevelSet_data);
  }
  header_info->mhaP_bits = ptr_mhaP_buf.cnt_bits;

  //mhaD box data
  header_info->mhaD_data_present = audio_config_data->mpegh3daDecoderConfig_data.mpegh3daUniDrcConfigPresent;
  if (header_info->mhaD_data_present)
  {
    ia_write_mhaD_dat(&ptr_mhaD_buf, audio_config_data);
  }
  header_info->mhaD_bits = ptr_mhaD_buf.cnt_bits;

  if (audio_config_data != NULL)
      free(audio_config_data);

  return IA_NO_ERROR;
}

/**impegh_mhas_parse
 *
 *  \brief Parse MPEG-H 3D audio stream
 *
 *  \param [in]		ptr_bit_buf		Bit buffer handle
 *  \param [out]	ptr_pac_info	MHAS packet info handle
 *  \param [out]	ptr_mae_asi		Metadata audio element asi handle
 *
 *  \param [out]	header_info		Packet info
 *  \param [out]	header_info_sync_cur		Packet info
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impegh_mhas_parse(ia_bit_buf_struct *ptr_bit_buf, ia_mhas_pac_info *ptr_pac_info,
                               packet_info *header_info)
{
  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 packet_type, packet_lbl, packet_length, tmp;
  WORD32 packet_info_bits = 0;
  WORD32 bits_used;
  do
  {
    packet_type = impegh_read_escape_value(ptr_bit_buf, 3, 8, 8, &bits_used);
    packet_info_bits = bits_used;
    packet_lbl = impegh_read_escape_value(ptr_bit_buf, 2, 8, 32, &bits_used);
    packet_info_bits += bits_used;
    packet_length = impegh_read_escape_value(ptr_bit_buf, 11, 24, 24, &bits_used);
    packet_info_bits += bits_used;

    if (MHAS_PAC_TYP_SYNC == packet_type)
    {
      header_info->sync_packet_length = packet_length;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      if (tmp != MHAS_SYNC_BYTE || packet_length != 1)
      {
        return IMPEGHE_MUX_NON_FATAL_MHAS_SYNCWORD_MISMATCH;
      }
      header_info->sync_packet_bits = packet_info_bits + (packet_length << 3);
      if(header_info->config_packet_found == 0)
      {
        header_info->config_packet_start_position += ((packet_info_bits + 7) >> 3) + packet_length;
      }
    }
    else if (MHAS_PAC_TYP_AUDIOSCENEINFO == packet_type)
    {

      WORD32 asi_packet_bits = 0, final_bits = 0;
      asi_packet_bits = ptr_bit_buf->cnt_bits;

      error = impegh_audio_scene_info_process(ptr_bit_buf, header_info);
      if (error != IA_NO_ERROR)
      {
        return error;
      }

      final_bits = asi_packet_bits - ptr_bit_buf->cnt_bits;
      if (final_bits < (packet_length << 3))
      {
        impegh_read_bits_buf(ptr_bit_buf, ((packet_length << 3) - final_bits)); // just skip  them if bits left out
      }

      header_info->asi_packet_length = packet_length;
      header_info->asi_packet_bits = packet_info_bits + (packet_length << 3);
      if (header_info->config_packet_found == 0)
      {
        header_info->config_packet_start_position += ((packet_info_bits + 7) >> 3) + packet_length;
      }
    }
    else if (MHAS_PAC_TYP_MPEGH3DACFG == packet_type)
    {

      WORD32 config_packet_bits = 0, final_bits = 0;
      config_packet_bits = ptr_bit_buf->cnt_bits;


      error = impegh_3d_audio_config_data_process(ptr_bit_buf, header_info);
      if (error != IA_NO_ERROR)
      {
        return error;
      }

      final_bits = config_packet_bits - ptr_bit_buf->cnt_bits;
      if (final_bits < (packet_length << 3))
      {
        impegh_read_bits_buf(ptr_bit_buf, ((packet_length << 3) - final_bits)); // just skip  them if bits left out
      }


      header_info->config_packet_length = packet_length;
      header_info->config_packet_bits = packet_info_bits + (packet_length << 3);
      if (header_info->config_packet_found == 0)
      {
        header_info->config_packet_start_position += ((packet_info_bits + 7) >> 3);
        header_info->mhac_content_size = packet_length;
      }
      header_info->config_packet_found = 1;
    }
    else if (MHAS_PAC_TYP_MPEGH3DAFRAME == packet_type)
    {
      tmp = packet_length << 3;
      error = impegh_skip_bits_buf(ptr_bit_buf, tmp);
      if (error)
      {
        return error;
      }
      header_info->frame_packet_bits = packet_info_bits + (packet_length << 3);
    }
    if ((MHAS_PAC_TYP_MPEGH3DACFG != packet_type) &&
        (MHAS_PAC_TYP_MPEGH3DAFRAME != packet_type) && (MHAS_PAC_TYP_SYNC != packet_type) &&
        (MHAS_PAC_TYP_AUDIOSCENEINFO != packet_type))
    {
      tmp = packet_length << 3;
      impegh_read_bits_buf(ptr_bit_buf, tmp);
      header_info->other_packet_bits += packet_info_bits + (packet_length << 3);
      if (header_info->config_packet_found == 0)
      {
        header_info->config_packet_start_position += ((packet_info_bits + 7) >> 3) + packet_length;
      }
    }
  } while ((MHAS_PAC_TYP_MPEGH3DAFRAME != packet_type));

  ptr_pac_info->packet_type = packet_type;
  ptr_pac_info->packet_lbl = packet_lbl;
  ptr_pac_info->packet_length = packet_length;

  return IA_NO_ERROR;
}


IA_ERRORCODE impegh_file_parse(ia_bit_buf_struct *ptr_bit_buf)
{
  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 packet_type, packet_lbl, packet_length, tmp;
  WORD32 bit_cnt_temp = 0;
  WORD32 bits_used;
  do
  {
    bit_cnt_temp = ptr_bit_buf->cnt_bits;
    packet_type = impegh_read_escape_value(ptr_bit_buf, 3, 8, 8, &bits_used);
    packet_lbl = impegh_read_escape_value(ptr_bit_buf, 2, 8, 32, &bits_used);
    packet_length = impegh_read_escape_value(ptr_bit_buf, 11, 24, 24, &bits_used);

    tmp = packet_length << 3;
    tmp = impegh_read_bits_buf(ptr_bit_buf, tmp);

  } while(MHAS_PAC_TYP_MPEGH3DAFRAME != packet_type);

  return IA_NO_ERROR;
}
