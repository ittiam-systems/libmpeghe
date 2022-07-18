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
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_cnst.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_oam_enc.h"
#include "impeghe_oam_enc_ld.h"
#include "impeghe_oam_enc_utils.h"

/**
 *  impeghe_obj_md_read_frame
 *
 *  \brief Reads OAM frame by making callback to application
 *
 *  \param [in,out]	ptr_oam_enc_state	Pointer to OAM state structure
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_obj_md_read_frame(ia_oam_enc_state_struct *ptr_oam_enc_state)
{
  FLOAT32 *ptr_file_data;
  UWORD16 obj_id;
  WORD32 bytes_read;
  LOOPIDX idx;
  UWORD32 read_buff_offset;
  UWORD64 *ptr_read_buff_64;
  UWORD16 *ptr_read_buff_16;
  ia_oam_enc_multidata_struct *pstr_oam_data = &ptr_oam_enc_state->str_oam_data_inp;

  for (idx = 0; idx < pstr_oam_data->num_elements; idx++)
  {
    read_buff_offset = 0;
    bytes_read = ptr_oam_enc_state->str_config.read_data(ptr_oam_enc_state->str_config.data_hndl,
                                                         ptr_oam_enc_state->read_buff,
                                                         ptr_oam_enc_state->bytes_per_element);
    if (bytes_read != ptr_oam_enc_state->bytes_per_element)
    {
      return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
    }

    ptr_read_buff_64 = ((UWORD64 *)&ptr_oam_enc_state->read_buff[read_buff_offset]);
    pstr_oam_data->sample[idx] = *ptr_read_buff_64;
    read_buff_offset += sizeof(UWORD64);

    ptr_read_buff_16 = ((UWORD16 *)&ptr_oam_enc_state->read_buff[read_buff_offset]);
    obj_id = *ptr_read_buff_16;
    read_buff_offset += sizeof(UWORD16);
    if (obj_id != (idx % ptr_oam_enc_state->str_config.num_objects))
    {
      return IMPEGHE_CONFIG_FATAL_OAM_INVALID_FRAME;
    }

    ptr_file_data = (FLOAT32 *)(&ptr_oam_enc_state->read_buff[read_buff_offset]);

    pstr_oam_data->azimuth[idx] = ptr_file_data[0];
    pstr_oam_data->elevation[idx] = ptr_file_data[1];
    pstr_oam_data->radius[idx] = ptr_file_data[2];
    pstr_oam_data->gain[idx] = ptr_file_data[3];

    if (ptr_oam_enc_state->str_config.oam_version > 1)
    {
      pstr_oam_data->spread[idx] = ptr_file_data[4];
    }
    else
    {
      pstr_oam_data->spread[idx] = 0.0f;
    }

    if (ptr_oam_enc_state->str_config.oam_version > 2 &&
        ptr_oam_enc_state->str_config.has_dyn_obj_priority == 1)
    {
      pstr_oam_data->dyn_obj_priority[idx] = ptr_file_data[5];
    }
    else
    {
      pstr_oam_data->dyn_obj_priority[idx] = 7.0f;
    }

    if (ptr_oam_enc_state->str_config.oam_version > 3)
    {
      if (ptr_oam_enc_state->num_components == 7)
      {
        pstr_oam_data->spread_height[idx] = ptr_file_data[5];
        pstr_oam_data->spread_depth[idx] = ptr_file_data[6];
      }
      else if (ptr_oam_enc_state->num_components == 8)
      {
        pstr_oam_data->spread_height[idx] = ptr_file_data[5];
        pstr_oam_data->spread_depth[idx] = ptr_file_data[6];
        pstr_oam_data->dyn_obj_priority[idx] = ptr_file_data[7];
      }
      else
      {
        pstr_oam_data->spread_height[idx] = 0.0f;
        pstr_oam_data->spread_depth[idx] = 0.0f;
      }
    }
  }

  for (idx = 0; idx < ptr_oam_enc_state->str_config.extra_objects; idx++)
  {
    read_buff_offset = 0;
    bytes_read = ptr_oam_enc_state->str_config.read_data(ptr_oam_enc_state->str_config.data_hndl,
                                                         ptr_oam_enc_state->read_buff,
                                                         ptr_oam_enc_state->bytes_per_element);
    if (bytes_read != ptr_oam_enc_state->bytes_per_element)
    {
      return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
    }
  }

  return IA_NO_ERROR;
}

/**
 *  impeghe_obj_md_enc_read_header
 *
 *  \brief Reads OAM header by making callback to application
 *
 *  \param [in,out]	ptr_oam_enc_state	Pointer to OAM state structure
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE impeghe_obj_md_enc_read_header(ia_oam_enc_state_struct *ptr_oam_enc_state)
{
  switch (ptr_oam_enc_state->str_config.oam_version)
  {
  case 4:
    if (!ptr_oam_enc_state->str_config.has_dyn_obj_priority &&
        ptr_oam_enc_state->str_config.has_uniform_spread)
    {
      ptr_oam_enc_state->num_components = 5;
    }
    else if (ptr_oam_enc_state->str_config.has_dyn_obj_priority &&
             ptr_oam_enc_state->str_config.has_uniform_spread)
    {
      ptr_oam_enc_state->num_components = 6;
    }
    else if (!ptr_oam_enc_state->str_config.has_dyn_obj_priority &&
             !ptr_oam_enc_state->str_config.has_uniform_spread)
    {
      ptr_oam_enc_state->num_components = 7;
    }
    else
    {
      ptr_oam_enc_state->num_components = 8;
    }
    break;
  case 3:
    if (ptr_oam_enc_state->str_config.has_dyn_obj_priority != 1)
    {
      ptr_oam_enc_state->num_components = 5;
    }
    else
    {
      ptr_oam_enc_state->num_components = 6;
    }
    break;
  case 2:
    ptr_oam_enc_state->num_components = 5;
    break;
  case 1:
    ptr_oam_enc_state->num_components = 4;
    break;
  default:
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_HEADER;
  }

  ptr_oam_enc_state->bytes_per_element =
      OAM_SAMPLE_NUM_SIZE_BYTES + OAM_OBJ_ID_SIZE_BYTES +
      OAM_COMPONENT_DATA_SIZE_BYTES * ptr_oam_enc_state->num_components;

  return IA_NO_ERROR;
}

/**
 *  impeghe_obj_md_enc_init
 *
 *  \brief OAM object encoder initialization
 *
 *  \param [out] 	ptr_oam_enc_state	Pointer to OAM state structure
 *  \param [in] 	pstr_oam_config				Pointer to OAM configuration
 * structure
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_obj_md_enc_init(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                     ia_oam_enc_config_struct *pstr_oam_config)
{
  IA_ERRORCODE err_code = 0;
  jmp_buf obj_md_enc_init_jmp_buf;
  err_code = setjmp(obj_md_enc_init_jmp_buf);
  if (err_code != IA_NO_ERROR)
  {
    return IMPEGHE_INIT_FATAL_INSUFFICIENT_OAM_WRITE_BUFFER_SIZE;
  }
  if (pstr_oam_config->core_block_size != 1024 && pstr_oam_config->core_block_size != 2048 &&
      pstr_oam_config->core_block_size != 4096)
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }

  if ((pstr_oam_config->read_data == 0) || (pstr_oam_config->skip_data == 0))
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }

  memset(ptr_oam_enc_state, 0x00, sizeof(ia_oam_enc_state_struct));

  ptr_oam_enc_state->str_config = *pstr_oam_config;

  err_code = impeghe_obj_md_enc_read_header(ptr_oam_enc_state);
  if (err_code & IA_FATAL_ERROR)
  {
    return err_code;
  }

  ptr_oam_enc_state->str_oam_data_inp.num_elements = ptr_oam_enc_state->str_config.num_objects;
  if (ptr_oam_enc_state->str_config.num_objects != 0)
  {
    do
    {
      err_code = impeghe_obj_md_read_frame(ptr_oam_enc_state);
      if (err_code & IA_FATAL_ERROR)
      {
        return err_code;
      }

      if (ptr_oam_enc_state->oam_block_size == 0)
      {
        ptr_oam_enc_state->oam_block_size = (WORD32)ptr_oam_enc_state->str_oam_data_inp.sample[0];
      }

      if ((WORD32)ptr_oam_enc_state->str_oam_data_inp.sample[0] >=
          (ptr_oam_enc_state->str_config.core_block_size - 1))
      {
        break;
      }

    } while (1);
  }
  else
  {
    ptr_oam_enc_state->oam_block_size = pstr_oam_config->core_block_size;
  }

  ptr_oam_enc_state->is_elem_read = 1;

  if (ptr_oam_enc_state->oam_block_size > pstr_oam_config->core_block_size ||
      ((ptr_oam_enc_state->oam_block_size % 64) != 0))
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }
  if (ptr_oam_enc_state->oam_block_size < 1)
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }
  if (pstr_oam_config->core_block_size % ptr_oam_enc_state->oam_block_size)
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }

  if (pstr_oam_config->high_rate == 1)
  {
    ptr_oam_enc_state->sub_sample = 1;
    ptr_oam_enc_state->frame_factor =
        (pstr_oam_config->core_block_size / ptr_oam_enc_state->oam_block_size);
  }
  else
  {
    ptr_oam_enc_state->frame_factor = 1;
    ptr_oam_enc_state->sub_sample =
        (pstr_oam_config->core_block_size / ptr_oam_enc_state->oam_block_size);
  }

  impeghe_create_bit_buffer(&ptr_oam_enc_state->str_bit_buf_out,
                            ptr_oam_enc_state->bit_buf_base_out,
                            sizeof(ptr_oam_enc_state->bit_buf_base_out));

  impeghe_create_bit_buffer(&ptr_oam_enc_state->str_bit_buf_ld,
                            ptr_oam_enc_state->bit_buf_base_ld,
                            sizeof(ptr_oam_enc_state->bit_buf_base_ld));

  impeghe_create_bit_buffer(&ptr_oam_enc_state->str_bit_buf_dynm,
                            ptr_oam_enc_state->bit_buf_base_dynm,
                            sizeof(ptr_oam_enc_state->bit_buf_base_dynm));

  impeghe_create_bit_buffer(&ptr_oam_enc_state->str_bit_buf_intra,
                            ptr_oam_enc_state->bit_buf_base_intra,
                            sizeof(ptr_oam_enc_state->bit_buf_base_intra));

  impeghe_create_bit_buffer(&ptr_oam_enc_state->str_bit_buf_diff,
                            ptr_oam_enc_state->bit_buf_base_diff,
                            sizeof(ptr_oam_enc_state->bit_buf_base_diff));
  ptr_oam_enc_state->str_bit_buf_out.impeghe_jmp_buf = &obj_md_enc_init_jmp_buf;
  ptr_oam_enc_state->str_bit_buf_ld.impeghe_jmp_buf = &obj_md_enc_init_jmp_buf;
  ptr_oam_enc_state->str_bit_buf_dynm.impeghe_jmp_buf = &obj_md_enc_init_jmp_buf;
  ptr_oam_enc_state->str_bit_buf_intra.impeghe_jmp_buf = &obj_md_enc_init_jmp_buf;
  ptr_oam_enc_state->str_bit_buf_diff.impeghe_jmp_buf = &obj_md_enc_init_jmp_buf;
  if (1 == ptr_oam_enc_state->str_config.low_delay)
  {
    ptr_oam_enc_state->str_oam_data_last.num_elements = ptr_oam_enc_state->str_config.num_objects;
    ptr_oam_enc_state->str_oam_data_diff.num_elements = ptr_oam_enc_state->str_config.num_objects;
  }
  else
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }
  return IA_NO_ERROR;
}

/**
 *  impeghe_obj_md_enc
 *
 *  \brief OAM object encoder
 *
 *  \param [in] 	ptr_oam_enc_state		Pointer to OAM state structure
 *  \param [in] 	usac_independency_flg	Independency flag
 *  \param [in,out]	it_bit_buf				Pointer to bit-buffer
 *  \param [out] 	bits_written		Pointer to bit-count
 *
 *  \return WORD32
 */
WORD32 impeghe_obj_md_enc(ia_oam_enc_state_struct *ptr_oam_enc_state,
                          WORD32 usac_independency_flg, ia_bit_buf_struct *it_bit_buf,
                          WORD32 *bits_written)
{
  WORD32 err_code = 0;

  if (ptr_oam_enc_state->str_config.low_delay == 0)
  {
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG;
  }
  else
  {
    err_code = impeghe_obj_md_low_delay_enc(ptr_oam_enc_state, usac_independency_flg, it_bit_buf,
                                            bits_written);
  }

  return err_code;
}
