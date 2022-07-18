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

#ifndef IMPEGHE_FD_QC_UTIL_H
#define IMPEGHE_FD_QC_UTIL_H

typedef struct
{
  UWORD8 modify_min_snr;
  WORD32 start_sfb_long;
  WORD32 start_sfb_short;
} ia_ah_param_struct;

typedef struct
{
  FLOAT32 max_red;
  FLOAT32 start_ratio;
  FLOAT32 max_ratio;
  FLOAT32 red_ratio_fac;
  FLOAT32 red_offs;
} ia_min_snr_adapt_param_struct;

typedef struct
{
  FLOAT32 pe_min;
  FLOAT32 pe_max;
  FLOAT32 pe_offset;
  ia_ah_param_struct str_ah_param;
  ia_min_snr_adapt_param_struct str_min_snr_adapt_params;
  FLOAT32 pe_last;
  WORD32 dyn_bits_last;
  FLOAT32 pe_correction_fac;
} ia_adj_thr_elem_struct;

typedef struct
{
  WORD32 ch_bitrate;
  WORD32 avg_bits;
  WORD32 max_bits;
  WORD32 bit_res_lvl;
  WORD32 max_bitres_bits;
  WORD32 static_bits;
  FLOAT32 max_bit_fac;
  WORD32 tot_avg_bits;
  WORD32 padding;
  ia_adj_thr_elem_struct str_adj_thr_ele;
  WORD8 num_ch;
} ia_qc_data_struct;

typedef struct
{
  WORD16 quant_spec[FRAME_LEN_LONG];
  WORD16 scalefactor[FRAME_LEN_LONG];
  WORD32 global_gain;
} ia_qc_out_chan_struct;

typedef struct
{

  ia_qc_out_chan_struct str_qc_out_chan[2];
  WORD32 static_bits;
  WORD32 dyn_bits;
  WORD32 fill_bits;
  FLOAT32 pe;
} ia_qc_out_data_struct;

typedef struct
{
  ia_qc_data_struct str_qc_data[MAX_TIME_CHANNELS];
  ia_qc_out_data_struct str_qc_out;
} ia_qc_main_struct;

VOID impeghe_qc_create(ia_qc_main_struct *pstr_qc_data);

VOID impeghe_qc_init(ia_qc_data_struct *pstr_qc_data, const WORD32 max_bits, WORD32 sample_rate,
                     WORD32 bw_limit, WORD32 channels);

VOID impeghe_adj_bitrate(ia_qc_data_struct *pstr_qc_data, WORD32 bit_rate, WORD32 sample_rate);

WORD32 impeghe_calc_max_val_in_sfb(WORD32 sfb_count, WORD32 max_sfb_per_grp, WORD32 sfb_per_group,
                                   WORD32 *ptr_sfb_offset, WORD16 *ptr_quant_spec);

#endif /* IMPEGHE_FD_QC_UTIL_H */
