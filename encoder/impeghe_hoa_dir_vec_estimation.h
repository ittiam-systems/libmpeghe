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

#ifndef IMPEGHE_HOA_DIR_VEC_ESTIMATION_H
#define IMPEGHE_HOA_DIR_VEC_ESTIMATION_H

#define IMPEGHE_HOA_NUM_DIR_FOR_SCAN (200)
#define IMPEGHE_DIR_EST_CLUSTER_THR_DEG (20)
#define IMPEGHE_DIR_EST_CLUSTER_THR_RAD (IMPEGHE_DIR_EST_CLUSTER_THR_DEG * PI / 180)

#define ABS_RAD(azimuth) (azimuth < 0 ? 2 * PI + azimuth : azimuth)

IA_ERRORCODE impeghe_hoa_dir_vec_estimation_process(ia_spatial_enc_str *pstr_se_handle,
                                                    pFlOAT64 ip_hoa_frame);

VOID impeghe_hoa_amb_comp_mod_process(ia_spatial_enc_str *pstr_se_handle);

VOID impeghe_hoa_ch_assignment_process(ia_spatial_enc_str *pstr_se_handle);

VOID impeghe_hoa_decomposition_process(ia_spatial_enc_str *pstr_se_handle,
                                       const pFlOAT64 input_hoa_frame);

VOID impeghe_hoa_dyn_correction_process(ia_spatial_enc_str *pstr_se_handle, UWORD32 ch_idx);

VOID impeghe_complex_fft_re_input(FLOAT32 *ptr_x, WORD32 nlength, FLOAT32 *y);

#endif /*IMPEGHE_HOA_DIR_VEC_ESTIMATION_H*/
