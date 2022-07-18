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

#ifndef IMPEGHE_DRC_UNI_DRC_FILTER_BANK_H
#define IMPEGHE_DRC_UNI_DRC_FILTER_BANK_H

#define FILTER_BANK_PARAMETER_COUNT (16)
#define IMPEGHE_DRC_MAX_PHASE_ALIGN_CH_GROUP (32)

typedef struct
{
  WORD32 num_bands;
  WORD32 complexity;
} ia_drc_filter_bank_struct;

typedef struct
{
  WORD32 num_filter_banks;
  WORD32 num_phase_alignment_ch_groups;
  WORD32 complexity;
  ia_drc_filter_bank_struct str_drc_filter_bank[IMPEGHE_DRC_MAX_PHASE_ALIGN_CH_GROUP];
} ia_drc_filter_banks_struct;

IA_ERRORCODE impeghe_drc_init_all_filter_banks(
    const ia_drc_coefficients_uni_drc_struct *pstr_drc_coefficients_uni_drc,
    const ia_drc_instructions_uni_drc *pstr_drc_instructions_uni_drc,
    ia_drc_filter_banks_struct *pstr_filter_banks, VOID *ptr_scratch);

#endif /* IMPEGHE_DRC_UNI_DRC_FILTER_BANK_H */
