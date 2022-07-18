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

#ifndef IMPEGHE_LPD_ROM_H
#define IMPEGHE_LPD_ROM_H

extern const FLOAT32 impeghe_lsf_init[ORDER];
extern const FLOAT32 impeghe_sin_window_128[128];
extern const FLOAT32 impeghe_sin_window_256[256];
extern const FLOAT32 impeghe_res_interp_filter1_4[INTER_LP_FIL_LEN + 4];
extern const FLOAT32 impeghe_lag_window[17];
extern const FLOAT32 impeghe_lsf_init[ORDER];
extern const FLOAT32 impeghe_ispold_init[ORDER];
extern const FLOAT32 impeghe_cos_window_512[512];
extern const FLOAT32 impeghe_cos_window_448[448];
extern const WORD32 impeghe_acelp_core_numbits_1024[NUM_ACELP_CORE_MODES];
extern const FLOAT32 impeghe_acelp_quant_gain_table[];
extern const UWORD8 impeghe_acelp_ipos[36];
extern const FLOAT32 impeghe_hp20_filter_coeffs[6][4];
extern const FLOAT32 impeghe_ol_corr_weight[518];
extern const FLOAT32 impeghe_interp4_1[17];

#endif /*IMPEGHE_LPD_ROM_H*/
