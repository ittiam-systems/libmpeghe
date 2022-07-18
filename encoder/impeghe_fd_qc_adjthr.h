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

#ifndef IMPEGHE_FD_QC_ADJTHR_H
#define IMPEGHE_FD_QC_ADJTHR_H

#define RED_EXP_VAL 0.25f
#define INV_RED_EXP_VAL (1.0f / RED_EXP_VAL)
#define MIN_SNR_LIMIT 0.8f

#define MAX_SCF_DELTA 60

#define LOG2_1 1.442695041f
#define C1_SF -69.33295f /* -16/3*log(MAX_QUANT+0.5-logCon)/log(2) */
#define C2_SF 5.77078f   /* 4/log(2) */

#define PE_C1 3.0f       /* log(8.0)/log(2) */
#define PE_C2 1.3219281f /* log(2.5)/log(2) */
#define PE_C3 0.5593573f /* 1-C2/C1 */

#define NO_AH (0)
#define AH_INACTIVE (1)
#define AH_ACTIVE (2)

#define NUM_MS_MASK_BITS (2)
#define NUM_ICS_INFO_BITS_LONG (9)
#define NUM_ICS_INFO_BITS_SHORT (14)

typedef struct
{
  FLOAT32 *sfb_ld_energy;
  FLOAT32 *sfb_lines;
  FLOAT32 sfb_pe[MAX_NUM_GROUPED_SFB];
  FLOAT32 sfb_const_part[MAX_NUM_GROUPED_SFB];
  FLOAT32 num_sfb_active_lines[MAX_NUM_GROUPED_SFB];
  FLOAT32 pe;
  FLOAT32 const_part;
  FLOAT32 num_active_lines;
} ia_qc_pe_chan_data_struct;

typedef struct
{
  ia_qc_pe_chan_data_struct pe_ch_data[2];
  FLOAT32 pe;
  FLOAT32 const_part;
  FLOAT32 num_active_lines;
  FLOAT32 offset;
} ia_qc_pe_data_struct;

FLOAT32 impeghe_bits_to_pe(const FLOAT32 bits);

VOID impeghe_adj_thr_init(ia_adj_thr_elem_struct *pstr_adj_thr_state, const FLOAT32 mean_pe,
                          WORD32 ch_bitrate);

VOID impeghe_adj_thr(ia_adj_thr_elem_struct *pstr_adj_thr_elem,
                     ia_psy_mod_out_data_struct *pstr_psy_out, FLOAT32 *ch_bit_dist,
                     ia_qc_out_data_struct *pstr_qc_out, const WORD32 avg_bits,
                     const WORD32 bitres_bits, const WORD32 max_bitres_bits,
                     const WORD32 side_info_bits, FLOAT32 *max_bit_fac, WORD32 num_chans,
                     WORD32 chn, impeghe_scratch_mem *pstr_scratch);

VOID impeghe_calc_form_fac_per_chan(ia_psy_mod_out_data_struct *pstr_psy_out_chan,
                                    impeghe_scratch_mem *pstr_scratch, WORD32 i_ch);

VOID impeghe_estimate_scfs_chan(ia_psy_mod_out_data_struct *pstr_psy_out,
                                ia_qc_out_chan_struct *str_qc_out_chan, WORD32 num_channels,
                                WORD32 chn, impeghe_scratch_mem *pstr_scratch);

VOID impeghe_quantize_lines(const WORD32 gain, const WORD32 num_lines, FLOAT32 *mdct_spectrum,
                            WORD16 *quant_spectrum, FLOAT32 *mdct_float);

#endif /* IMPEGHE_FD_QC_ADJTHR_H */
