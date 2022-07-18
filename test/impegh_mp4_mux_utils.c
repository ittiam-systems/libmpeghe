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

WORD32 impegh_read_bits_buf(ia_bit_buf_struct_e *it_bit_buff, WORD no_of_bits)
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
    longjmp(*(it_bit_buff->impegh_jmp_buf), 1);
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
IA_ERRORCODE impegh_skip_bits_buf(ia_bit_buf_struct_e *it_bit_buff, WORD no_of_bits)
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
ia_bit_buf_struct_e *impegh_create_bit_buf(ia_bit_buf_struct_e *it_bit_buff,
                                           UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size)
{
  it_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buff->ptr_bit_buf_end = ptr_bit_buf_base + bit_buf_size - 1;

  it_bit_buff->ptr_read_next = ptr_bit_buf_base;
  it_bit_buff->bit_pos = 7;

  it_bit_buff->cnt_bits = 0;
  it_bit_buff->size = bit_buf_size << 3;

  it_bit_buff->error = 0;
  it_bit_buff->max_size = it_bit_buff->size;

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
static VOID impegh_mae_parse_description_data(ia_bit_buf_struct_e *ptr_bit_buf,
                                              ia_mp4description_data *ptr_description_data,
                                              WORD32 group_id_bits)
{
  WORD32 n, num_descr_blocks, tmp, i, c;
  num_descr_blocks = impegh_read_bits_buf(ptr_bit_buf, 7);
  ptr_description_data->num_desc_blocks = num_descr_blocks + 1;
  for (n = 0; n < num_descr_blocks + 1; n++)
  {
    tmp = impegh_read_bits_buf(ptr_bit_buf, group_id_bits);
    ptr_description_data->group_id[n] = tmp;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 4);
    ptr_description_data->num_descr_languages = tmp + 1;
    for (i = 0; i < ptr_description_data->num_descr_languages; i++)
    {
      tmp = impegh_read_bits_buf(ptr_bit_buf, 24);
      ptr_description_data->descr_language[i] = tmp;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      ptr_description_data->descr_data_length[n][i] = tmp + 1;
      for (c = 0; c < ptr_description_data->descr_data_length[n][i]; c++)
      {
        tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
        ptr_description_data->descr_data[n][i][c] = tmp;
      }
    }
  }
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
VOID impegh_mae_asi_data_parse(ia_bit_buf_struct_e *ptr_bit_buf, ia_bit_buf_struct *ptr_mael_buf)
{
  WORD32 i = 0, data_type, data_length;
  WORD32 num_data_sets = 0;
  ia_mp4mae_data *ptr_mae_data = (ia_mp4mae_data *)malloc(sizeof(*ptr_mae_data));
  memset(ptr_mae_data, 0, sizeof(*ptr_mae_data));
  num_data_sets = impegh_read_bits_buf(ptr_bit_buf, 4);

  for (i = 0; i < num_data_sets; i++)
  {
    WORD32 cnt_bits = 0;
    data_type = impegh_read_bits_buf(ptr_bit_buf, 4);
    data_length = impegh_read_bits_buf(ptr_bit_buf, 16);
    ptr_mae_data->data_length[i] = data_length;
    cnt_bits = ptr_bit_buf->cnt_bits;
    switch (data_type)
    {
    case ID_MAE_GROUP_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &ptr_mae_data->group_desc_data, 7);
      break;
    case ID_MAE_SWITCHGROUP_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &ptr_mae_data->switch_group_desc_data, 5);
      break;
    case ID_MAE_GROUP_PRESET_DESCRIPTION:
      impegh_mae_parse_description_data(ptr_bit_buf, &ptr_mae_data->preset_desc_data, 5);
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
    data_length = ptr_mae_data->data_length[i];
    if (cnt_bits < (data_length << 3))
    {
      WORD32 skip_bits = (data_length << 3) - cnt_bits;
      impegh_read_bits_buf(ptr_bit_buf, skip_bits);
    }
  }
  impegh_write_bits_buf(ptr_mael_buf, 0, 4); // reserved bits
  impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.num_descr_languages,
                        4); // description languages
  for (i = 0; i < ptr_mae_data->group_desc_data.num_descr_languages; i++)
  {
    impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.descr_language[i], 24);
    impegh_write_bits_buf(ptr_mael_buf, 0, 1); // reserved bits
    impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.num_desc_blocks, 7);
    for (WORD32 j = 0; j < ptr_mae_data->group_desc_data.num_desc_blocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 1); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.group_id[j],
                            7); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.descr_data_length[i][j],
                            8);
      for (WORD32 k = 0; k < ptr_mae_data->group_desc_data.descr_data_length[i][j]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->group_desc_data.descr_data[i][j][k], 1);
      }
    }
    impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
    impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->switch_group_desc_data.num_desc_blocks, 5);
    for (WORD32 j = 0; j < ptr_mae_data->switch_group_desc_data.num_desc_blocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->switch_group_desc_data.group_id[j], 5);
      impegh_write_bits_buf(ptr_mael_buf,
                            ptr_mae_data->switch_group_desc_data.descr_data_length[i][j], 8);
      for (WORD32 k = 0; k < ptr_mae_data->switch_group_desc_data.descr_data_length[i][j]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf,
                              ptr_mae_data->switch_group_desc_data.descr_data[i][j][k], 1);
      }
    }
    impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
    impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->preset_desc_data.num_desc_blocks, 5);
    for (WORD32 j = 0; j < ptr_mae_data->preset_desc_data.num_desc_blocks; j++)
    {
      impegh_write_bits_buf(ptr_mael_buf, 0, 3); // reserved bits
      impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->preset_desc_data.group_id[j], 5);
      impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->preset_desc_data.descr_data_length[i][j],
                            8);
      for (WORD32 k = 0; k < ptr_mae_data->preset_desc_data.descr_data_length[i][j]; k++)
      {
        impegh_write_bits_buf(ptr_mael_buf, ptr_mae_data->preset_desc_data.descr_data[i][j][k],
                              1);
      }
    }
  }
  if (ptr_mae_data)
  {
    free(ptr_mae_data);
  }
}

/**
 *  impegh_mae_asi_group_presets_def
 *
 *  \brief Parse and write mae group preset elements to an intermediate buffer
 *
 *  \param [in]     ptr_bit_buf       Pointer to bit-buffer handle
 *  \param [in,out] ptr_maep_buf      Pointer to bit buffer handle of maep box
 *  \param [in]     num_group_presets Number of group presets
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_group_presets_def(ia_bit_buf_struct_e *ptr_bit_buf,
                                             ia_bit_buf_struct *ptr_maep_buf,
                                             WORD32 num_group_presets)
{
  WORD32 i, tmp, j, num_conditions, gain = 0, preset_gain = 0;
  for (i = 0; i < num_group_presets; i++)
  {
    impegh_write_bits_buf(ptr_maep_buf, 0, 3); // reserved bits
    tmp = 0;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    impegh_write_bits_buf(ptr_maep_buf, tmp, 5); // preset id
    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    impegh_write_bits_buf(ptr_maep_buf, 0, 3); // reserved bits
    impegh_write_bits_buf(ptr_maep_buf, tmp, 5);
    num_conditions = impegh_read_bits_buf(ptr_bit_buf, 4) + 1;
    impegh_write_bits_buf(ptr_maep_buf, num_conditions, 8);
    for (j = 0; j < num_conditions; j++)
    {
      tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // preset group id
      impegh_write_bits_buf(ptr_maep_buf, tmp, 7);

      tmp = impegh_read_bits_buf(ptr_bit_buf, 1);
      impegh_write_bits_buf(ptr_maep_buf, tmp, 1); // preset condition on off

      if (tmp)
      {
        impegh_write_bits_buf(ptr_maep_buf, 0, 4); // reserved bits
        tmp = impegh_read_bits_buf(ptr_bit_buf, 1);
        impegh_write_bits_buf(ptr_maep_buf, tmp, 1); // group disable gain Interactvity

        preset_gain = impegh_read_bits_buf(ptr_bit_buf, 1);
        impegh_write_bits_buf(ptr_maep_buf, preset_gain, 1); // group preset gain flag
        if (preset_gain)
        {
          gain = impegh_read_bits_buf(ptr_bit_buf, 8);
        }
        tmp = impegh_read_bits_buf(ptr_bit_buf, 1);
        impegh_write_bits_buf(ptr_maep_buf, tmp, 1); // disable position interactivity
        tmp = impegh_read_bits_buf(ptr_bit_buf, 1);  // position flag
        impegh_write_bits_buf(ptr_maep_buf, tmp, 1);
        if (preset_gain)
        {
          impegh_write_bits_buf(ptr_maep_buf, gain, 8);
        }
        if (tmp)
        {
          tmp = impegh_read_bits_buf(ptr_bit_buf, 8); // azimuth offset
          impegh_write_bits_buf(ptr_maep_buf, tmp, 8);

          tmp = impegh_read_bits_buf(ptr_bit_buf, 6); // elevation offset
          impegh_write_bits_buf(ptr_maep_buf, tmp, 8);

          tmp = impegh_read_bits_buf(ptr_bit_buf, 4); // dist factor
          impegh_write_bits_buf(ptr_maep_buf, tmp, 8);
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
 *  \param [in]     ptr_bit_buf    Pointer to bit buffer handle
 *  \param [in,out] ptr_asi_buffer Pointer to asi buffer handle
 *  \param [in]     num_groups     Number of groups
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_group_def(ia_bit_buf_struct_e *ptr_bit_buf,
                                     ia_bit_buf_struct *ptr_asi_buffer, WORD32 num_groups)
{
  WORD32 i, tmp, allow_pos_interact = 0;
  WORD32 min_az_offset = 0, max_az_offset = 0, min_gain = 0, max_gain = 0, min_el_offset = 0,
         remain_bits = 0, allow_gain_interact = 0, group_num_members;
  for (i = 0; i < num_groups; i++)
  {
    impegh_write_bits_buf(ptr_asi_buffer, 0, 1); // reserved bits
    tmp = impegh_read_bits_buf(ptr_bit_buf, 7);  // mae_grp id
    impegh_write_bits_buf(ptr_asi_buffer, tmp, 7);
    impegh_write_bits_buf(ptr_asi_buffer, 0, 3); // reserved bits

    tmp = impegh_read_bits_buf(ptr_bit_buf, 1); // allow_on_off
    impegh_write_bits_buf(ptr_asi_buffer, tmp, 1);

    tmp = impegh_read_bits_buf(ptr_bit_buf, 1); // default_on_off
    impegh_write_bits_buf(ptr_asi_buffer, tmp, 1);

    allow_pos_interact = impegh_read_bits_buf(ptr_bit_buf, 1); // position interactivity
    impegh_write_bits_buf(ptr_asi_buffer, allow_pos_interact, 1);

    if (allow_pos_interact)
    {

      min_az_offset = impegh_read_bits_buf(ptr_bit_buf, 7);

      max_az_offset = impegh_read_bits_buf(ptr_bit_buf, 7);

      min_el_offset = impegh_read_bits_buf(ptr_bit_buf, 5);

      remain_bits = impegh_read_bits_buf(ptr_bit_buf, 13);
    }
    allow_gain_interact = impegh_read_bits_buf(ptr_bit_buf, 1); // gain interaction
    impegh_write_bits_buf(ptr_asi_buffer, allow_gain_interact, 1);
    impegh_write_bits_buf(ptr_asi_buffer, 0,
                          9); // 1bit content_language + 4 reserved+4 content_kind
    if (allow_gain_interact)
    {
      min_gain = impegh_read_bits_buf(ptr_bit_buf, 6);

      max_gain = impegh_read_bits_buf(ptr_bit_buf, 5);
    }
    if (allow_pos_interact)
    {

      impegh_write_bits_buf(ptr_asi_buffer, min_az_offset, 8); // reserved+min_az_offest bits

      impegh_write_bits_buf(ptr_asi_buffer, max_az_offset, 8); // reserved+max_az_offset bits

      impegh_write_bits_buf(ptr_asi_buffer, min_el_offset, 8); // reserved + min_el_offest bits

      impegh_write_bits_buf(ptr_asi_buffer, remain_bits, 13);
    }
    if (allow_gain_interact)
    {
      impegh_write_bits_buf(ptr_asi_buffer, min_gain, 8); // reserved + min_gain bits

      impegh_write_bits_buf(ptr_asi_buffer, max_gain, 8); // reserved +max_gain bits
    }
    group_num_members = impegh_read_bits_buf(ptr_bit_buf, 7); // group_num_members skip them
    tmp = impegh_read_bits_buf(ptr_bit_buf, 1);               // has conjucnt_members skip them
    if (tmp)
    {
      tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // start id skip them
    }
    else
    {
      WORD32 j = 0;
      for (j = 0; j < group_num_members; j++)
      {
        tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // meta data ele id skip them
      }
    }
  }
}

/**
 *  impegh_mae_asi_switch_group_def
 *
 *  \brief Parse and write the elements of switch group to an intermediate buffer
 *
 *  \param [in]     ptr_bit_buf Pointer to bit buffer handle
 *  \param [in,out] ptr_maes_buf Pointer to switch group bit buffer handle
 *  \param [in]     num_switch_groups Number of switch groups
 *
 *  \return VOID
 */
static VOID impegh_mae_asi_switch_group_def(ia_bit_buf_struct_e *ptr_bit_buf,
                                            ia_bit_buf_struct *ptr_maes_buf,
                                            WORD32 num_switch_groups)
{
  WORD32 i, j, tmp = 0, group_num_members = 0;
  for (i = 0; i < num_switch_groups; i++)
  {
    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    impegh_write_bits_buf(ptr_maes_buf, tmp, 8); // reserved bits + switch group id
    tmp = impegh_read_bits_buf(ptr_bit_buf, 1);  // allow on off

    if (tmp)
    {
      tmp = impegh_read_bits_buf(ptr_bit_buf, 1);
    }

    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    group_num_members = tmp + 1;
    impegh_write_bits_buf(ptr_maes_buf, 0, 3);                 // reserved bits
    impegh_write_bits_buf(ptr_maes_buf, group_num_members, 5); // num_switch group memer
    for (j = 0; j < group_num_members; j++)
    {
      tmp = 0;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 7);  // member id
      impegh_write_bits_buf(ptr_maes_buf, tmp, 8); // reserved bits + switch group member id
    }
    tmp = 0;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 7);  // default id
    impegh_write_bits_buf(ptr_maes_buf, tmp, 8); // reserved bits + switch group default id
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
VOID impegh_create_init_bit_buf(ia_bit_buf_struct_e *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                WORD32 bit_buf_size)
{
  impegh_create_bit_buf(it_bit_buff, ptr_bit_buf_base, bit_buf_size);
  it_bit_buff->cnt_bits = (bit_buf_size << 3);
  return;
}
/**
 *  impegh_mae_asi_mp4_parse
 *
 *  \brief Parse and write the asi elements to intermediate buffers
 *
 *  \param [in] ptr_bit_buf Pointer to bit buffer handle
 *  \param [in,out] header_info Pointer to header info structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mae_asi_mp4_parse(ia_bit_buf_struct_e *ptr_bit_buf, packet_info *header_info)
{
  WORD32 tmp = 0, tmp_value = 0;
  ia_bit_buf_struct ptr_maeg_buf;
  impegh_create_mp4_buffer(&ptr_maeg_buf, &(header_info->maeg_buff[0]),
                           sizeof(header_info->maeg_buff), 1);
  tmp = impegh_read_bits_buf(ptr_bit_buf, 1);
  if (tmp) // main_stream_flag
  {
    tmp = impegh_read_bits_buf(ptr_bit_buf, 1); // asi_id_present
    if (tmp)
    {
      tmp_value = impegh_read_bits_buf(ptr_bit_buf, 8); // asi_id
    }
    impegh_write_bits_buf(&ptr_maeg_buf, tmp_value, 8); // writing asi_id
    impegh_write_bits_buf(&ptr_maeg_buf, 0, 1);         // reserved bits
    /* ASI - Group Definition*/
    tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // no:of groups

    if (tmp)
    {
      header_info->maei_present = 1;
    }
    else
    {
      header_info->maei_present = 0;
      return 0;
    }
    impegh_write_bits_buf(&ptr_maeg_buf, tmp, 7);
    if (tmp > MAX_NUM_GROUPS)
    {
      return -1;
    }
    impegh_mae_asi_group_def(ptr_bit_buf, &ptr_maeg_buf, tmp);
    header_info->maeg_bits = ptr_maeg_buf.cnt_bits;
    /* ASI - Switch Group Definition*/
    ia_bit_buf_struct ptr_maes_buf;
    impegh_create_mp4_buffer(&ptr_maes_buf, &(header_info->maes_buff[0]),
                             sizeof(header_info->maes_buff), 1);
    tmp = 0;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    impegh_write_bits_buf(&ptr_maes_buf, tmp, 8); // 3 reserved bits + 5 number of groups
    if (tmp > MAX_NUM_SWITCH_GROUPS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    impegh_mae_asi_switch_group_def(ptr_bit_buf, &ptr_maes_buf, tmp);
    header_info->maes_bits = ptr_maes_buf.cnt_bits;
    /* ASI - Group presets Definition*/
    tmp = 0;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 5);
    if (tmp > MAX_NUM_GROUPS_PRESETS)
    {
      return IMPEGHE_MUX_NON_FATAL_INVALID_ASI_VALUE;
    }
    ia_bit_buf_struct ptr_maep_buf;
    impegh_create_mp4_buffer(&ptr_maep_buf, &(header_info->maep_buff[0]),
                             sizeof(header_info->maep_buff), 1);
    impegh_write_bits_buf(&ptr_maep_buf, tmp, 8); // 3 reserved bits + 5 number of groups
    impegh_mae_asi_group_presets_def(ptr_bit_buf, &ptr_maep_buf, tmp);
    header_info->maep_bits = ptr_maep_buf.cnt_bits;
    /* ASI - MAE Data*/
    ia_bit_buf_struct ptr_mael_buf;
    impegh_create_mp4_buffer(&ptr_mael_buf, &(header_info->mael_buff[0]),
                             sizeof(header_info->mael_buff), 1);
    impegh_mae_asi_data_parse(ptr_bit_buf, &ptr_mael_buf);
    header_info->mael_bits = ptr_mael_buf.cnt_bits;

    tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // id max avail
  }
  else
  {
    tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // id offset

    tmp = impegh_read_bits_buf(ptr_bit_buf, 7); // id max avail
  }
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
IA_ERRORCODE impegh_mhas_parse(ia_bit_buf_struct_e *ptr_bit_buf, ia_mhas_pac_info *ptr_pac_info,
                               ia_mae_audio_scene_info *ptr_mae_asi, packet_info *header_info,
                               packet_info *header_info_sync_cur)
{
  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 packet_type, packet_lbl, packet_length, tmp;
  WORD32 bit_cnt_temp = 0;
  do
  {
    bit_cnt_temp = ptr_bit_buf->cnt_bits;
    tmp = impegh_read_bits_buf(ptr_bit_buf, 3);
    if (tmp == 7)
    {
      packet_type = tmp;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      if (tmp == 255)
      {
        packet_type = packet_type + tmp;
        tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
        packet_type = packet_type + tmp;
      }
      else
      {
        packet_type += tmp;
      }
    }
    else
    {
      packet_type = tmp;
    }
    tmp = impegh_read_bits_buf(ptr_bit_buf, 2);
    if (tmp == 3)
    {
      packet_lbl = tmp;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      if (tmp == 255)
      {
        packet_lbl = packet_lbl + tmp;
        tmp = impegh_read_bits_buf(ptr_bit_buf, 32);
        packet_lbl = packet_lbl + tmp;
      }
      else
      {
        packet_lbl += tmp;
      }
    }
    else
    {
      packet_lbl = tmp;
    }
    tmp = impegh_read_bits_buf(ptr_bit_buf, 11);
    if (tmp == 2047)
    {
      packet_length = tmp;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 24);
      if (tmp == (WORD32)((1 << 24) - 1))
      {
        packet_length = packet_length + tmp;
        tmp = impegh_read_bits_buf(ptr_bit_buf, 24);
        packet_length = packet_length + tmp;
      }
      else
      {
        packet_length += tmp;
      }
    }
    else
    {
      packet_length = tmp;
    }
    if (MHAS_PAC_TYP_SYNC == packet_type)
    {
      header_info->sync_packet_length = packet_length;
      tmp = impegh_read_bits_buf(ptr_bit_buf, 8);
      if (tmp != MHAS_SYNC_BYTE || packet_length != 1)
      {
        return IMPEGHE_MUX_NON_FATAL_MHAS_SYNCWORD_MISMATCH;
      }
      header_info->sync_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
      header_info_sync_cur->sync_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
    }
    else if (MHAS_PAC_TYP_AUDIOSCENEINFO == packet_type)
    {
      header_info->asi_packet_length = packet_length;
      WORD32 asi_packet_bits = 0, final_bits = 0;
      if (header_info->maei_parse)
      {
        asi_packet_bits = ptr_bit_buf->cnt_bits;
        error = impegh_mae_asi_mp4_parse(ptr_bit_buf, header_info);
        if (error != IA_NO_ERROR)
        {
          return error;
        }
        final_bits = asi_packet_bits - ptr_bit_buf->cnt_bits;
        if (final_bits < (packet_length << 3))
        {
          impegh_read_bits_buf(ptr_bit_buf, ((packet_length << 3) - final_bits)); // just skip
                                                                                  // them
        }
        header_info_sync_cur->asi_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
        header_info_sync_cur->other_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
        header_info->maei_parse = 0;
      }
      else
      {
        impegh_read_bits_buf(ptr_bit_buf, (packet_length << 3)); // just skip them the first time
        header_info_sync_cur->asi_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
        header_info_sync_cur->other_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
        header_info->maei_parse = 1;
      }
    }
    else if (MHAS_PAC_TYP_MPEGH3DACFG == packet_type)
    {
      header_info->config_packet_length = packet_length;
      tmp = packet_length << 3;
      WORD32 bits_to_skip = tmp;

      header_info->profile_info =
          impegh_read_bits_buf(ptr_bit_buf, 8); // read mpegh_3da_profile_lvl_indication

      WORD32 sampling_frequency_index = impegh_read_bits_buf(ptr_bit_buf, 5);
      if (sampling_frequency_index == 0x1f)
      {
        sampling_frequency_index = impegh_read_bits_buf(ptr_bit_buf, 24);
        if (sampling_frequency_index > MAX_SAMPLE_RATE)
        {
          return IMPEGHE_MUX_NON_FATAL_INVALID_SAMPLING_RATE;
        }
      }
      else
      {
        if (sampling_frequency_index > 0x08)
        {
          return IMPEGHE_MUX_NON_FATAL_INVALID_SAMPLING_RATE;
        }
        impegh_read_bits_buf(ptr_bit_buf, 24);
        sampling_frequency_index =
            ia_sampling_rate_tbl[sampling_frequency_index]; /* Extract sampling rate from the
                                                               config packet */
      }
      bits_to_skip -= (8 + 5 + 24); /* No of bits from the packet remaining to be skipped */
      impegh_read_bits_buf(
          ptr_bit_buf, bits_to_skip); /* Dummay read to skip the remaining bits from the packet */

      header_info->sampling_freq = sampling_frequency_index;
      header_info->config_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
      header_info_sync_cur->config_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
    }
    else if (MHAS_PAC_TYP_MPEGH3DAFRAME == packet_type)
    {
      tmp = packet_length << 3;
      error = impegh_skip_bits_buf(ptr_bit_buf, tmp);
      if (error)
      {
        return error;
      }
      header_info->frame_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
    }
    if ((MHAS_PAC_TYP_MPEGH3DACFG != packet_type) &&
        (MHAS_PAC_TYP_MPEGH3DAFRAME != packet_type) && (MHAS_PAC_TYP_SYNC != packet_type) &&
        (MHAS_PAC_TYP_AUDIOSCENEINFO != packet_type))
    {
      tmp = packet_length << 3;
      impegh_read_bits_buf(ptr_bit_buf, tmp);
      header_info_sync_cur->other_packet_bits = bit_cnt_temp - ptr_bit_buf->cnt_bits;
    }
  } while ((MHAS_PAC_TYP_MPEGH3DACFG != packet_type) &&
           (MHAS_PAC_TYP_MPEGH3DAFRAME != packet_type));

  ptr_pac_info->packet_type = packet_type;
  ptr_pac_info->packet_lbl = packet_lbl;
  ptr_pac_info->packet_length = packet_length;

  return IA_NO_ERROR;
}
