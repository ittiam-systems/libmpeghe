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

#ifndef IMPEGHE_DRC_ENC_H
#define IMPEGHE_DRC_ENC_H

IA_ERRORCODE impeghe_drc_gain_enc_init(ia_drc_gain_enc_struct *pstr_gain_enc,
                                       ia_drc_uni_drc_config_struct *pstr_uni_drc_config,
                                       ia_drc_loudness_info_set_struct *pstr_loudness_info_set,
                                       const WORD32 frame_size, const WORD32 sample_rate,
                                       const WORD32 delay_mode, const WORD32 domain);

WORD32 impeghe_drc_get_delta_t_min(const WORD32 sample_rate);

VOID impeghe_drc_encode_uni_drc_gain(ia_drc_gain_enc_struct *pstr_gain_enc,
                                     FLOAT32 *ptr_gain_buffer, VOID *pstr_scratch);

IA_ERRORCODE impeghe_drc_write_uni_drc_config(ia_drc_enc_state *pstr_drc_state,
                                              WORD32 *ptr_bit_cnt);

VOID impeghe_drc_write_uni_drc_gain(ia_drc_enc_state *pstr_drc_state, WORD32 *ptr_bit_cnt);

#endif /*IMPEGHE_DRC_ENC_H*/
