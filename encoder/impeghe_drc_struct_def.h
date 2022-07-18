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

#ifndef IMPEGHE_DRC_STRUCT_DEF_H
#define IMPEGHE_DRC_STRUCT_DEF_H

#define MAX_DRC_GAIN_BAND_COUNT (50)
#define MAX_DRC_FRAME_SIZE (4096)
#define DRC_OUT_BITBUFFER_SIZE (4096)

typedef struct
{
  ia_drc_enc_params_struct str_enc_params;
  ia_drc_uni_drc_config_struct str_uni_drc_config;
  ia_drc_loudness_info_set_struct str_enc_loudness_info_set;
  ia_drc_uni_drc_gain_ext_struct str_enc_gain_extension;

  ia_drc_gain_enc_struct str_gain_enc;
  UWORD8 bit_buf_base_cfg[MAX_DRC_PAYLOAD_BYTES];
  ia_bit_buf_struct str_bit_buf_cfg;
  WORD32 drc_config_data_size_bit;
  UWORD8 bit_buf_base_cfg_ext[MAX_DRC_PAYLOAD_BYTES];
  ia_bit_buf_struct str_bit_buf_cfg_ext;
  UWORD8 bit_buf_base_cfg_tmp[MAX_DRC_PAYLOAD_BYTES];
  ia_bit_buf_struct str_bit_buf_cfg_tmp;

  UWORD8 drc_payload_data[MAX_DRC_PAYLOAD_BYTES];
  FLOAT32 gain_buffer[MAX_DRC_GAIN_BAND_COUNT][MAX_DRC_FRAME_SIZE];

  UWORD8 bit_buf_base_out[DRC_OUT_BITBUFFER_SIZE];
  ia_bit_buf_struct str_bit_buf_out;
  UWORD8 is_first_drc_process_complete;

  VOID *drc_scratch_mem;
  WORD32 drc_scratch_used;
} ia_drc_enc_state;

#endif /*IMPEGHE_DRC_STRUCT_DEF_H*/
