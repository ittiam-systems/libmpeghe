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

#ifndef IMPEGHE_OAM_ENC_STRUCT_DEF_H
#define IMPEGHE_OAM_ENC_STRUCT_DEF_H

#define OAM_HEADER_SIZE_BYTES (4)
#define OAM_VERSION_SIZE_BYTES (2)
#define OAM_DESCRIPTION_SIZE_BYTES (32)
#define OAM_CH_FILE_NAME_SIZE_BYTES (64)
#define OAM_OBJ_DESCRIPTION_SIZE_BYTES (64)
#define OAM_SAMPLE_NUM_SIZE_BYTES (8)
#define OAM_OBJ_ID_SIZE_BYTES (2)
#define OAM_COMPONENT_DATA_SIZE_BYTES (sizeof(FLOAT32))
#define OAM_READ_BUFFER_SIZE_BYTES (2048)
#define OAM_BITBUFFER_SIZE (2048)

typedef struct
{
  WORD16 oam_version;
  WORD16 has_dyn_obj_priority;
  WORD16 has_uniform_spread;
  WORD32 num_objects;
  WORD32 extra_objects;
  WORD32 num_channels;
  WORD32 core_block_size;
  FLAG high_rate;
  WORD32 replace_radius;
  FLAG low_delay;
  FLAG fixed_values[6];
  VOID *data_hndl;
  WORD32 (*read_data)(VOID *read_data_hndl, UWORD8 *buff, WORD32 bytes_to_read);
  WORD32 (*skip_data)(VOID *data_hndl, WORD32 bytes_to_skip);
} ia_oam_enc_config_struct;

typedef struct
{
  UWORD64 sample[OAM_MAX_NUM_OBJECTS];
  FLOAT32 azimuth[OAM_MAX_NUM_OBJECTS];
  FLOAT32 elevation[OAM_MAX_NUM_OBJECTS];
  FLOAT32 radius[OAM_MAX_NUM_OBJECTS];
  FLOAT32 gain[OAM_MAX_NUM_OBJECTS];
  FLOAT32 spread[OAM_MAX_NUM_OBJECTS];
  FLOAT32 spread_height[OAM_MAX_NUM_OBJECTS];
  FLOAT32 spread_depth[OAM_MAX_NUM_OBJECTS];
  FLOAT32 dyn_obj_priority[OAM_MAX_NUM_OBJECTS];
  WORD32 num_elements;
} ia_oam_enc_multidata_struct;

typedef struct
{
  ia_oam_enc_config_struct str_config;
  WORD32 sub_sample;
  WORD32 frame_factor;
  WORD32 is_elem_read;
  WORD16 num_components;
  UWORD8 read_buff[OAM_READ_BUFFER_SIZE_BYTES];
  UWORD32 bytes_per_element;
  ia_oam_enc_multidata_struct str_oam_data_inp;
  ia_oam_enc_multidata_struct str_oam_data_last;
  ia_oam_enc_multidata_struct str_oam_data_diff;
  UWORD8 bit_buf_base_out[OAM_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_out;
  UWORD8 bit_buf_base_ld[OAM_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_ld;
  UWORD8 bit_buf_base_dynm[OAM_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_dynm;
  UWORD8 bit_buf_base_intra[OAM_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_intra;
  UWORD8 bit_buf_base_diff[OAM_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_diff;
  WORD32 oam_block_size;
} ia_oam_enc_state_struct;

#endif /* IMPEGHE_OAM_ENC_STRUCT_DEF_H */
