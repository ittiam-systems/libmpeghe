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

#ifndef IMPEGHE_LTPF_ENC_H
#define IMPEGHE_LTPF_ENC_H
#define LTPF_ENC_INTER_LEN 2
#define LP_FILT_LEN 240
#define POW_10_MIN_5 powf(10.0, -5.0)

VOID impeghe_ol_pitch_estim(FLOAT32 *x, ia_usac_ltpf_data_struct *ltpf_data, WORD32 *ol_pitch,
                            FLOAT32 *normcorr_out, WORD32 x_length,
                            impeghe_scratch_mem *pstr_scratch);

WORD32 impeghe_ltpf_encode(ia_usac_ltpf_data_struct *ltpf_data, const FLOAT32 *in_data,
                           WORD32 in_len, WORD32 pitch_ol, FLOAT32 pitch_ol_norm_corr,
                           impeghe_scratch_mem *pstr_scratch);
WORD32 impeghe_ltpf_init(ia_usac_ltpf_data_struct *ltpf_data, WORD32 input_sr);

extern const FLOAT32 impeghe_olpa_down2_fir_coeffs[5];
extern const FLOAT32 impeghe_olpa_weighted_auto_corr_table[98];

extern const FLOAT32 impeghe_ltpf_fir_inter4_1[12][33];
extern const FLOAT32 impeghe_ltpf_inter_filter[4][4];
extern const FLOAT32 inter4_2tcx2[4][4];
extern const FLOAT32 inter6_2tcx2[6][4];

#endif /* IMPEGHE_LTPF_ENC_H */
