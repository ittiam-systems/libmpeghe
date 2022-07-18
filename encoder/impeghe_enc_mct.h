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

#ifndef IMPEGHE_ENC_MCT_H
#define IMPEGHE_ENC_MCT_H

#include "impeghe_cnst.h"
#define MAX_NUM_MCT_PAIRS (28)
#define MAX_NUM_MC_BANDS (64)

#define DEFAULT_ALPHA (0)
#define DEFAULT_BETA (48) /*equals 45 degrees */

typedef struct
{
  WORD32 mct_signalling_type;
  WORD32 num_channels;
  WORD32 num_ch_to_apply;
  WORD32 num_pairs;
  WORD32 prev_num_pairs;
  WORD32 ext_elem_idx;
  WORD32 channel_pair[MAX_NUM_MCT_PAIRS][2];
  WORD32 prev_channel_pair[MAX_NUM_MCT_PAIRS][2];
  WORD32 pred_dir[MAX_NUM_MCT_PAIRS];
  WORD32 pair_alpha[MAX_NUM_MCT_PAIRS][MAX_NUM_MC_BANDS];
  WORD32 pair_mct_mask[MAX_NUM_MCT_PAIRS][MAX_NUM_MC_BANDS];
  WORD32 num_mask_bands[MAX_NUM_MCT_PAIRS];
  WORD32 mct_ch_mask[MAX_TIME_CHANNELS];
  WORD32 is_mct_mask_present[MAX_NUM_MCT_PAIRS];
  UWORD8 mct_ext_data[MAX_EXTENSION_PAYLOAD_LEN];
  WORD32 mct_ext_data_size;
  WORD32 mct_ext_data_present;
} mct_data_t;

VOID impeghe_encode_mct(FLOAT64 *ptr_spec[MAX_TIME_CHANNELS], WORD32 *ptr_win_seq,
                        const WORD32 *ptr_sfb_offset, WORD32 sfb_end, mct_data_t *pstr_mct_data,
                        WORD32 ch_offset);

IA_ERRORCODE impeghe_write_mct_data(mct_data_t *pstr_mct_data, WORD32 indep_flag,
                                    UWORD8 *ptr_payload_extn, WORD32 *size_payload_extn,
                                    WORD32 *payload_extn_present, UWORD8 *ptr_scratch);
#endif /*IMPEGHE_ENC_MCT_H*/