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

#ifndef IMPEGHE_HOA_ROM_H
#define IMPEGHE_HOA_ROM_H

#include "impeghe_type_def.h"

typedef struct
{
  WORD8 len;
  WORD16 code_word;
} impeghe_word_struct;

extern const FLOAT64 impeghe_hoa_pow_2_table[64];
extern const FLOAT64 impeghe_hoa_pow_2_inv_table[64];
extern const WORD32 impeghe_num_of_par_subbands_table[3];
extern const WORD32 impeghe_num_of_pred_subbands_table[3];
extern const WORD32 impeghe_par_subband_width_table[3][20];
extern const WORD32 impeghe_spat_interp_time_code_table[4][8];
extern const impeghe_word_struct impeghe_hoa_frame_huffman_table[10][5][15];
extern const FLOAT64 impeghe_window_function[1024];
extern const FLOAT64 impeghe_inv_mode_mat[7][49 * 49];
extern const FLOAT64 impeghe_fade_win_for_dir_based_syn[2048];
extern const FLOAT64 impeghe_hoa_azimuth[900];
extern const FLOAT64 impeghe_hoa_elevation[900];

extern const FLOAT64 impeghe_hoa_order_0_fine_grid_mode_mat[900];
extern const FLOAT64 impeghe_hoa_order_1_fine_grid_mode_mat[4 * 900];
extern const FLOAT64 impeghe_hoa_order_2_fine_grid_mode_mat[9 * 900];
extern const FLOAT64 impeghe_hoa_order_3_fine_grid_mode_mat[16 * 900];
extern const FLOAT64 impeghe_hoa_order_4_fine_grid_mode_mat[25 * 900];
extern const FLOAT64 impeghe_hoa_order_5_fine_grid_mode_mat[36 * 900];
extern const FLOAT64 impeghe_hoa_order_6_fine_grid_mode_mat[49 * 900];
extern const FLOAT64 impeghe_hoa_order_0_fine_grid_mode_mat_inv[900];
extern const FLOAT64 impeghe_hoa_order_1_fine_grid_mode_mat_inv[4 * 900];
extern const FLOAT64 impeghe_hoa_order_2_fine_grid_mode_mat_inv[9 * 900];
extern const FLOAT64 impeghe_hoa_order_3_fine_grid_mode_mat_inv[16 * 900];
extern const FLOAT64 impeghe_hoa_order_4_fine_grid_mode_mat_inv[25 * 900];
extern const FLOAT64 impeghe_hoa_order_5_fine_grid_mode_mat_inv[36 * 900];
extern const FLOAT64 impeghe_hoa_order_6_fine_grid_mode_mat_inv[49 * 900];
extern const FLOAT64 impeghe_hoa_order_0_coarse_grid_mode_mat[1];
extern const FLOAT64 impeghe_hoa_order_1_coarse_grid_mode_mat[16];
extern const FLOAT64 impeghe_hoa_order_2_coarse_grid_mode_mat[81];
extern const FLOAT64 impeghe_hoa_order_3_coarse_grid_mode_mat[256];
extern const FLOAT64 impeghe_hoa_order_4_coarse_grid_mode_mat[625];
extern const FLOAT64 impeghe_hoa_order_5_coarse_grid_mode_mat[1296];
extern const FLOAT64 impeghe_hoa_order_6_coarse_grid_mode_mat[2401];

extern const FLOAT64 impeghe_twiddle_table_fft_32x32[514];
extern const FLOAT64 impeghe_twiddle_sin_2048[1024];
extern const FLOAT64 impeghe_twiddle_cos_2048[1024];

#endif /*IMPEGHE_HOA_ROM_H*/
