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

#ifndef IMPEGHE_STEREO_LPD_H
#define IMPEGHE_STEREO_LPD_H

typedef struct
{
  WORD32 fs;
  WORD32 ccfl;
  WORD32 dft_size;
  WORD32 frame_size;
  WORD32 overlap_size;
  WORD32 num_subframes;
} ia_usac_slpd_config_str;

typedef struct
{
  WORD32 res_mode;
  WORD32 q_mode;
  WORD32 ipd_mode;
  WORD32 pred_mode;
  WORD32 cod_mode;
  WORD32 ild_idx[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 ipd_idx[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 pred_gain_idx[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  WORD32 cod_gain_idx[STEREO_LPD_DFT_NB];
} ia_usac_slpd_bitstream_str;

typedef struct
{
  WORD32 params[NUM_LPC_PRM + 1];
  FLOAT32 ild[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 ipd[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 pred_gain[STEREO_LPD_DFT_NB][STEREO_LPD_BAND_MAX];
  FLOAT32 cod_gain[STEREO_LPD_DFT_NB];
  FLOAT32 res_sig[STEREO_LPD_DFT_NB][STEREO_LPD_DFT_SIZE];
} ia_usac_slpd_param_str;

typedef struct
{
  WORD32 band_limits[STEREO_LPD_BAND_MAX + 1];
  WORD32 num_bands;
  WORD32 ipd_band_max;
  WORD32 cod_band_max;
  WORD32 side_dft_lines;
} ia_usac_slpd_band_info_str;

typedef struct
{
  WORD32 init_flag;
  WORD32 fac_fb;
  FLOAT32 dft_prev_dmx[STEREO_LPD_DFT_SIZE];
  FLOAT32 time_buff_dmx[FRAME_LEN_LONG / 2 + FRAME_LEN_LONG];
  FLOAT32 prev_left[2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FAC_LENGTH];
  FLOAT32 prev_right[2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FAC_LENGTH];
  FLOAT32 prev_mid[2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FAC_LENGTH];
  FLOAT32 prev_side[2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FAC_LENGTH];
  const FLOAT32 *p_slpd_sin_table;
  const FLOAT32 *win;

  ia_usac_slpd_config_str lpd_stereo_config;
  ia_usac_slpd_bitstream_str lpd_stereo_bitstream;
  ia_usac_slpd_param_str lpd_stereo_parameter;
  ia_usac_slpd_band_info_str lpd_stereo_bandinfo;
} ia_usac_slpd_enc_data_str;

typedef struct
{
  FLOAT32 time_buff_left[(2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FRAME_LEN_LONG + FAC_LENGTH)];
  FLOAT32 time_buff_right[(2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FRAME_LEN_LONG + FAC_LENGTH)];
  FLOAT32 time_buff_mid[(2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FRAME_LEN_LONG + FAC_LENGTH)];
  FLOAT32 time_buff_side[(2 * MAX_PITCH + FRAME_LEN_LONG / 2 + FRAME_LEN_LONG + FAC_LENGTH)];
  FLOAT32 dft_left[2 * STEREO_LPD_DFT_SIZE];
  FLOAT32 dft_rght[2 * STEREO_LPD_DFT_SIZE];
  FLOAT32 dft_dmix[2 * STEREO_LPD_DFT_SIZE];
  FLOAT32 dft_side[STEREO_LPD_DFT_NB][2 * STEREO_LPD_DFT_SIZE];
  FLOAT32 left_chn_nrgs[STEREO_LPD_BAND_MAX];
  FLOAT32 rght_chn_nrgs[STEREO_LPD_BAND_MAX];
  FLOAT32 side_chn_nrgs[STEREO_LPD_BAND_MAX];
} ia_slpd_scratch_str;

VOID impeghe_slpd_init(ia_usac_slpd_enc_data_str *slpd_enc_data, WORD32 fs, WORD32 full_band_lpd,
                       WORD32 ccfl);
IA_ERRORCODE impeghe_enc_slpd_params(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data,
                                     ia_slpd_scratch_str *ptr_slpd_scratch,
                                     ia_bit_buf_struct *it_bit_buff, WORD32 td_start_flag);
VOID impeghe_enc_stereo_lpd(ia_usac_slpd_enc_data_str *slpd_enc_data,
                            ia_slpd_scratch_str *pstr_slpd_scratch,
                            impeghe_scratch_mem *pstr_scratch_mem);

#endif /* IMPEGHE_STEREO_LPD_H */
