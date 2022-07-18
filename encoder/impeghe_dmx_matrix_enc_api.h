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

#ifndef IMPEGHE_DMX_MATRIX_ENC_API_H
#define IMPEGHE_DMX_MATRIX_ENC_API_H

IA_ERRORCODE impeghe_dmx_matrix_enc(ia_bit_buf_struct *it_bit_buf,
                                    ia_dmx_matrix_enc_cfg_struct *pstr_enc_config,
                                    ia_dmx_sratch *pstr_dmx_scratch, WORD32 *ptr_bit_cnt);

IA_ERRORCODE impeghe_dmx_encode_downmix_matrix(
    WORD32 input_index, WORD32 input_num_channels,
    ia_dmx_speaker_information_struct *pstr_input_config, WORD32 output_index,
    WORD32 output_num_channels, ia_dmx_speaker_information_struct *pstr_output_config,
    WORD32 precision_level, ia_bit_buf_struct *it_bit_buf, FLOAT32 *ptr_downmix_matrix,
    WORD32 eq_precision_level, ia_dmx_eq_config_struct *pstr_eq_config,
    ia_dmx_sratch *pstr_dmx_scratch, WORD32 *ptr_bit_cnt);

#endif /* IMPEGHE_DMX_MATRIX_ENC_API_H */
