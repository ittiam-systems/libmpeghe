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

#ifndef IMPEGHE_ROM_H
#define IMPEGHE_ROM_H

#define WIN_LEN_1024 1024
#define WIN_LEN_128 128
#define WIN_LEN_256 256

extern const FLOAT32 impeghe_iir_hipass_coeffs[BLK_SWITCH_FILT_LEN];
extern const WORD32 impeghe_suggested_grouping_table[MAX_SHORT_WINDOWS][MAX_NUM_WIN_GROUPS];
extern const WORD32 impeghe_synchronized_block_types[4][4];

extern const UWORD32 impeghe_sampl_freq_table[];

extern const FLOAT32 impeghe_gamma_table[ORDER + 1];
extern const FLOAT32 impeghe_chebyshev_polyn_grid[101];

extern const FLOAT32 impeghe_fd_quant_table_q[16];
extern const FLOAT32 impeghe_fd_quant_table_e[40];
extern const FLOAT32 impeghe_fd_inv_quant_table_q[16];
extern const FLOAT32 impeghe_fd_inv_quant_table_e[17];
extern const FLOAT32 impeghe_pow4_3_table[64];

extern const UWORD32 impeghe_sampl_freq_idx_table[32];

extern const WORD32 impeghe_huffman_code_table[121][2];

extern const FLOAT64 impeghe_twiddle_table_fft_32x32[514];
extern const FLOAT64 impeghe_twiddle_table_3pr[1155];
extern const FLOAT64 impeghe_twiddle_table_3pi[1155];
extern const FLOAT64 impeghe_pre_post_twid_cos_2048[512];
extern const FLOAT64 impeghe_pre_post_twid_sin_2048[512];
extern const FLOAT64 impeghe_pre_post_twid_cos_256[64];
extern const FLOAT64 impeghe_pre_post_twid_sin_256[64];
extern const FLOAT64 impeghe_pre_twid_type2_dct_dst_cos_256[128];
extern const FLOAT64 impeghe_pre_twid_type2_dct_dst_cos_2048[1024];
extern const FLOAT64 impeghe_twiddle_cos_2048[1024];
extern const FLOAT64 impeghe_twiddle_sin_2048[1024];
extern const FLOAT64 impeghe_kbd_win1024[1024];
extern const FLOAT64 impeghe_kbd_win256[256];
extern const FLOAT64 impeghe_kbd_win128[128];

extern const FLOAT64 impeghe_sine_win_1024[1024];
extern const FLOAT64 impeghe_sine_win_256[256];
extern const FLOAT64 impeghe_sine_win_128[128];

extern const UWORD16 impeghe_ari_cf_r[3][4];
extern const UWORD16 impeghe_ari_lookup_m[742];
extern const UWORD32 impeghe_ari_hash_m[742];
extern const UWORD8 impeghe_ari_hash_m_lsb[742];
extern const UWORD16 impeghe_ari_cf_m[64][17];

extern const FLOAT32 impeghe_pre_post_twid_cos_sin_512[4][512];
extern const FLOAT32 impeghe_pre_post_twid_cos_sin_256[4][256];
extern const FLOAT32 impeghe_pre_post_twid_cos_sin_128[4][128];
extern const FLOAT32 impeghe_pre_post_twid_cos_sin_64[4][64];
extern const FLOAT32 impeghe_pre_post_twid_cos_sin_32[4][32];

extern const FLOAT64 impeghe_pow_table[9000];

extern const FLOAT64 impeghe_mdst_fcoeff_long_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_long_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_long_sin_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_long_kbd_sin[];
extern const FLOAT64 *impeghe_mdst_fcoeff_longshort_curr[2][2];

extern const FLOAT64 impeghe_mdst_fcoeff_start_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_start_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_start_sin_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_start_kbd_sin[];

extern const FLOAT64 *impeghe_mdst_fcoeff_start_curr[2][2];

extern const FLOAT64 impeghe_mdst_fcoeff_stop_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_stop_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_stop_sin_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_stop_kbd_sin[];

extern const FLOAT64 *impeghe_mdst_fcoeff_stop_cur[2][2];

extern const FLOAT64 impeghe_mdst_fcoeff_stopstart_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_stopstart_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_stopstart_sin_kbd[];
extern const FLOAT64 impeghe_mdst_fcoeff_stopstart_kbd_sin[];

extern const FLOAT64 *impeghe_mdst_fcoeff_stopstart_cur[2][2];

extern const FLOAT64 impeghe_mdst_fcoeff_l_s_start_left_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_l_s_start_left_kbd[];

extern const FLOAT64 impeghe_mdst_fcoeff_stop_stopstart_left_sin[];
extern const FLOAT64 impeghe_mdst_fcoeff_stop_stopstart_left_kbd[];

extern const FLOAT64 *impeghe_mdst_fcoeff_l_s_start_left_prev[2];
extern const FLOAT64 *impeghe_mdst_fcoeff_stop_stopstart_left_prev[2];

extern const WORD32 impeghe_ltpf_max_bit_rate[5];

extern const FLOAT32 impeghe_rad_3_fft_twiddle_re[1155];
extern const FLOAT32 impeghe_rad_3_fft_twiddle_im[1155];
extern const FLOAT32 impeghe_fft_mix_rad_twid_tbl_336[564];
extern const FLOAT32 impeghe_fft_mix_rad_twid_tbl_168[276];

extern const UWORD16 impeghe_cf_se_01[27];
extern const UWORD16 impeghe_cf_se_10[27];
extern const UWORD16 impeghe_cf_se_02[7][27];
extern const UWORD16 impeghe_cf_se_20[7][27];
extern const UWORD16 impeghe_cf_se_11[7][7][27];
extern const WORD16 impeghe_cf_off_se_02[7];
extern const WORD16 impeghe_cf_off_se_20[7];
extern const WORD16 impeghe_cf_off_se_11[7][7];
extern const UWORD16 impeghe_cf_for_bit[2];
extern const FLOAT32 impeghe_tnf_acf_win[8];

#endif /*IMPEGHE_ROM_H*/
