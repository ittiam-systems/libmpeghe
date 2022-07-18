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

#ifndef IMPEGH_MP4_MUX_UTILS_H
#define IMPEGH_MP4_MUX_UTILS_H
#include <setjmp.h>
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_mhas_write.h"
#include "impeghe_mae_write.h"
#include "impegh_error_codes_mux.h"
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX_SAMPLE_RATE (96000)
#define MAX_MAE_BUFFER_SIZE (100)
// structures
typedef struct ia_bit_buf_struct_e
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;

  WORD32 bit_pos;
  WORD32 cnt_bits;
  WORD32 size;
  WORD32 error;
  WORD32 max_size;
  jmp_buf *impegh_jmp_buf;

} ia_bit_buf_struct_e;

typedef struct packet_info
{
  WORD32 sync_packet_bits;
  WORD32 sync_packet_length;
  WORD32 asi_packet_length;
  WORD32 asi_packet_bits;
  UWORD8 maeg_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 maes_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 maep_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 mael_buff[MAX_MAE_BUFFER_SIZE];
  WORD32 maeg_bits;
  WORD32 maes_bits;
  WORD32 maep_bits;
  WORD32 mael_bits;
  WORD32 maei_present;
  WORD8 maei_parse;
  WORD32 group_definition_length;
  WORD32 switch_group_length;
  WORD32 group_preset_length;
  WORD32 config_packet_bits;
  WORD32 config_packet_length;
  WORD32 other_packet_bits;
  WORD32 other_packet_length;
  WORD32 frame_packet_bits;
  WORD32 sampling_freq;
  UWORD32 profile_info;
} packet_info;

typedef struct
{
  WORD32 num_desc_blocks;
  WORD32 group_id[MAX_NUM_DESCRIPTOIN_BLOCKS];
  WORD32 num_descr_languages;
  WORD32 descr_language[MAX_NUM_DESCR_LANGUAGES];
  WORD32 descr_data_length[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD32 descr_data[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES][MAX_DESCRIPTON_DATA_LEN];
} ia_mp4description_data;

typedef struct
{
  WORD32 num_data_sets[MAX_MAE_NUM_DATASETS];
  WORD32 data_type[MAX_MAE_NUM_DATASETS];
  WORD32 data_length[MAX_MAE_NUM_DATASETS];
  ia_mp4description_data group_desc_data;
  ia_mp4description_data switch_group_desc_data;
  ia_mp4description_data preset_desc_data;
} ia_mp4mae_data;
WORD32 impegh_read_bits_buf(ia_bit_buf_struct_e *it_bit_buff, WORD no_of_bits);

WORD32 impegh_skip_bits_buf(ia_bit_buf_struct_e *it_bit_buff, WORD no_of_bits);

ia_bit_buf_struct_e *impegh_create_bit_buf(ia_bit_buf_struct_e *it_bit_buff,
                                           UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size);

VOID impegh_create_init_bit_buf(ia_bit_buf_struct_e *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                WORD32 bit_buf_size);

WORD32 impegh_mhas_parse(ia_bit_buf_struct_e *ptr_bit_buf, ia_mhas_pac_info *ptr_pac_info,
                         ia_mae_audio_scene_info *ptr_mae_asi, packet_info *header_info,
                         packet_info *header_info_sync_cur);

#endif
