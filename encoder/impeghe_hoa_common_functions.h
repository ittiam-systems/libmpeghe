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

#ifndef IMPEGHE_HOA_COMMON_FUNCTIONS_H
#define IMPEGHE_HOA_COMMON_FUNCTIONS_H

#define __M_PI__ ((FLOAT32)(3.14159265358979323846f))

#ifndef ABS
#define ABS(X) (X < 0) ? -X : X
#endif

#ifndef MAX
#define MAX(X, Y) (X > Y) ? X : Y
#endif

#ifndef MIN
#define MIN(X, Y) (X < Y) ? X : Y
#endif

VOID impeghe_hoa_compute_max_abs_val(const pFlOAT64 ptr_input, const UWORD32 vec_len,
                                     pFlOAT64 max_abs_val, pUWORD32 max_abs_index);

VOID impeghe_hoa_compute_fade_win_for_vec_based_syn(pFlOAT64 out, UWORD32 frame_sz,
                                                    UWORD32 interp_samples,
                                                    UWORD32 interp_method);

UWORD32 impeghe_hoa_quantize_uniform(FLOAT64 input_val, UWORD32 num_bits);

UWORD32 impeghe_hoa_get_pow2(const UWORD32 exponent);

UWORD32 impeghe_hoa_get_ceil_log2(const UWORD32 x);

VOID impeghe_hoa_compute_pseudo_inverse_vec(const pFlOAT64 input_vec,
                                            const UWORD32 input_vec_size,
                                            pFlOAT64 output_pseudo_inv_vec);

#endif /*IMPEGHE_HOA_COMMON_FUNCTIONS_H*/
