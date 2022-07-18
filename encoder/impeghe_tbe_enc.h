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

#ifndef IMPEGHE_TBE_ENC_H
#define IMPEGHE_TBE_ENC_H

typedef struct
{
  WORD32 harm_ext_mode;
  WORD32 frame_gain_idx;
  WORD32 sub_gain_idx;
  WORD32 lsf_idx[2];
  WORD32 idx_mix_config;
  WORD32 idx_shb_fr_gain;
  WORD32 idx_res_sub_gains;
  WORD32 idx_shb_exc_resp[2];
  WORD32 hr_config;
  WORD32 nl_config;
} ia_usac_tbe_bitstream_str;

typedef struct
{
  FLOAT32 resampler_mem[MAX_TIME_CHANNELS][DOWNSAMP_FILT_MEM_SIZE];
  ia_usac_tbe_bitstream_str bit_stream[2];
} ia_usac_tbe_data_str;

VOID impeghe_write_tbe_data(ia_usac_tbe_data_str *ptr_tbe_dec_data,
                            ia_bit_buf_struct *it_bit_buff, WORD32 tbe_frame_class);

VOID impeghe_tbe_resample_input(FLOAT32 *ptr_output_signal, FLOAT32 *ptr_input_signal,
                                FLOAT32 *ptr_filt_mem, FLOAT32 *ptr_scratch_mem,
                                WORD32 num_samples_in);

#endif /* IMPEGHE_TBE_ENC_H */
