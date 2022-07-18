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

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "impeghe_error_standards.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_cnst.h"
#include "impeghe_igf_enc.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"

#include "impeghe_psy_utils.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_oam_enc.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"
#include "impeghe_write_bitstream.h"
#include "impeghe_windowing.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h" //DRC
#include "impeghe_config.h"
#include "impeghe_tcx_mdct.h"
#include "impeghe_lpd.h"
#include "impeghe_block_switch.h"
#include "impeghe_rom.h"
#include "impeghe_error_codes.h"
#include "impeghe_ltpf_enc.h"
#include "impeghe_hoa_matrix.h"
#include "impeghe_fd_qc_adjthr.h"

static WORD32 elem_map_5_1[4] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE, ID_USAC_CPE};

static WORD32 elem_map_9_0[5] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_CPE, ID_USAC_CPE, ID_USAC_CPE};

static WORD32 elem_map_7_1[5] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE, ID_USAC_CPE, ID_USAC_CPE};

static WORD32 elem_map_9_1[6] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE,
                                 ID_USAC_CPE, ID_USAC_CPE, ID_USAC_CPE};

static WORD32 elem_map_11_1[7] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE, ID_USAC_CPE,
                                  ID_USAC_CPE, ID_USAC_CPE, ID_USAC_CPE};

static WORD32 elem_map_22_2[16] = {ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE, ID_USAC_CPE,
                                   ID_USAC_CPE, ID_USAC_SCE, ID_USAC_LFE, ID_USAC_CPE,
                                   ID_USAC_CPE, ID_USAC_SCE, ID_USAC_SCE, ID_USAC_CPE,
                                   ID_USAC_CPE, ID_USAC_SCE, ID_USAC_SCE, ID_USAC_CPE};

/**
 *  impeghe_get_num_elements
 *
 *  \brief Maps number of channels to number of elements
 *
 *  \param [in] num_channels  Number of channels
 *
 *  \return WORD32            Number of elements
 */
static WORD32 impeghe_get_num_elements(WORD32 num_channels)
{
  WORD32 num_of_elements = 0;

  switch (num_channels)
  {
  case 1:
  case 2:
    num_of_elements = 1;
    break;
  case 6:
    num_of_elements = 4;
    break;
  case 8:
  case 9:
    num_of_elements = 5;
    break;
  case 10:
    num_of_elements = 6;
    break;
  case 12:
    num_of_elements = 7;
    break;
  case 24:
    num_of_elements = 16;
    break;
  default:
    num_of_elements = num_channels;
    break;
  }

  return num_of_elements;
}

/**
 *  impeghe_get_element_type
 *
 *  \brief Maps to element type
 *
 *  \param [in] elem_idx       Element index
 *  \param [in] num_channels   Number of channels
 *
 *  \return UWORD32             Element type
 */
static UWORD32 impeghe_get_element_type(WORD32 elem_idx, WORD32 num_channels)
{
  UWORD32 elem_type = -1;

  switch (num_channels)
  {
  case 1:
    elem_type = ID_USAC_SCE;
    break;
  case 2:
    elem_type = ID_USAC_CPE;
    break;
  case 6:
    elem_type = elem_map_5_1[elem_idx];
    break;
  case 8:
    elem_type = elem_map_7_1[elem_idx];
    break;
  case 9:
    elem_type = elem_map_9_0[elem_idx];
    break;
  case 10:
    elem_type = elem_map_9_1[elem_idx];
    break;
  case 12:
    elem_type = elem_map_11_1[elem_idx];
    break;
  case 24:
    elem_type = elem_map_22_2[elem_idx];
    break;
  default:
    elem_type = ID_USAC_SCE;
    break;
  }

  return elem_type;
}

/**
 *  impeghe_bw_init
 *
 *  \brief Initailize bandwidth limit
 *
 *  \param [in] pstr_usac_config  Pointer to encoder config structure
 *  \param [in] ele_idx          Element index
 *
 *  \return VOID
 */
static VOID impeghe_bw_init(ia_usac_encoder_config_struct *pstr_usac_config, WORD32 ele_idx)
{
  pstr_usac_config->bw_limit[ele_idx] = 20000;

  pstr_usac_config->bw_limit[ele_idx] =
      min(pstr_usac_config->bw_limit[ele_idx], pstr_usac_config->sampling_rate / 2);

  if (ID_USAC_LFE == pstr_usac_config->audio_specific_config.str_usac_config
                         .usac_element_type[ele_idx + pstr_usac_config->num_ext_elements])
  {
    pstr_usac_config->bw_limit[ele_idx] = 280;
  }

  return;
}

/**
 *  impeghe_scratch_mem_init
 *
 *  \brief Scratch memory initialization
 *
 *  \param [in] pstr_usac_data  Pointer to encoder data structure
 *  \param [in] total_ch   Total number of channels
 *  \param [in] sr         Sample rate
 *
 *  \return VOID
 */
VOID impeghe_scratch_mem_init(ia_usac_data_struct *pstr_usac_data, WORD32 total_ch, WORD32 sr)
{
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  UWORD8 *temp_ptr = pstr_scratch->ptr_scratch_buf;

  pstr_scratch->p_fd_mdct_windowed_long_buf = (FLOAT64 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT64);

  pstr_scratch->p_fd_mdct_windowed_short_buf = (FLOAT64 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT64);
  temp_ptr = (UWORD8 *)pstr_scratch->p_fd_mdct_windowed_short_buf; // reused

  pstr_scratch->p_fdp_href = (FLOAT64 *)temp_ptr;
  temp_ptr += 512 * sizeof(FLOAT64);

  pstr_scratch->p_fdp_mdct_out = (FLOAT64 *)temp_ptr;
  temp_ptr += 1024 * sizeof(FLOAT64);
  temp_ptr = (UWORD8 *)pstr_scratch->p_fdp_href; // reused
  pstr_scratch->p_tns_filter = (FLOAT64 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT64);

  pstr_scratch->ptr_tns_scratch = (FLOAT64 *)(temp_ptr);
  temp_ptr += (MAX_SHIFT_LEN_LONG + (TNS_MAX_ORDER + 1) * 2) *
              sizeof(pstr_scratch->ptr_tns_scratch[0]); // newly added

  pstr_scratch->ptr_igf_scratch = (FLOAT64 *)(temp_ptr);
  temp_ptr += (MAX_NUM_SFB_LONG)*2 * sizeof(pstr_scratch->ptr_igf_scratch[0]);
  temp_ptr = (UWORD8 *)pstr_scratch->p_tns_filter; // reused

  pstr_scratch->p_left_fac_time_data = (FLOAT64 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH + ORDER) * sizeof(FLOAT64);

  pstr_scratch->p_fac_win = (FLOAT64 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT64);
  temp_ptr = (UWORD8 *)pstr_scratch->p_left_fac_time_data; // reuse

  pstr_scratch->p_sort_grouping_scratch = (FLOAT64 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT64);
  temp_ptr = (UWORD8 *)pstr_scratch->p_sort_grouping_scratch; // reused

  pstr_scratch->p_noise_filling_highest_tone = (FLOAT64 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT64);

  pstr_scratch->p_quant_spectrum_spec_scratch = (FLOAT64 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT64);

  pstr_scratch->p_cmpx_mdct_temp_buf = (FLOAT64 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT64);
  temp_ptr = (UWORD8 *)pstr_scratch->p_noise_filling_highest_tone; // reused

  for (WORD32 i = 0; i < 2; i++)
  {
    pstr_scratch->p_reconstructed_time_signal[i] = (FLOAT64 *)(temp_ptr);
    temp_ptr += (4 * FRAME_LEN_LONG) * sizeof(FLOAT64);
  }

  pstr_scratch->p_fft_p2_y = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_fft_p3_data_3 = (FLOAT32 *)(temp_ptr);
  temp_ptr += (800) * sizeof(FLOAT32);

  pstr_scratch->p_fft_p3_y = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_time_signal = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_complex_fft = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_tonal_flag = (WORD32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG / 2) * sizeof(WORD32);

  pstr_scratch->p_pow_spec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG / 2) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_time_signal; // reused

  pstr_scratch->p_temp_mdct = (FLOAT32 *)(temp_ptr);
  temp_ptr += (1024) * sizeof(FLOAT32);

  pstr_scratch->p_buf_synthesis_tool = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_FRAME_16K + ORDER_LP_FILT_16K) * sizeof(FLOAT32);

  pstr_scratch->p_mdct_spec_float = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_sq_gain_en = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG / 4) * sizeof(FLOAT32);

  pstr_scratch->p_fft_mdct_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr += (4 * FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_arith_map_prev_scratch = (WORD32 *)(temp_ptr);
  temp_ptr += (516) * sizeof(WORD32);

  pstr_scratch->p_arith_map_pres_scratch = (WORD32 *)(temp_ptr);
  temp_ptr += (516) * sizeof(WORD32);

  pstr_scratch->p_ol_pitch_buf_tmp = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_ol_pitch_speech_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG + LAG_MAX) * sizeof(FLOAT32);

  pstr_scratch->p_ol_pitch_w_table = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_CORR_R) * sizeof(FLOAT32);

  pstr_scratch->p_ol_pitch_R = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_CORR_R) * sizeof(FLOAT32);

  WORD32 R0_size = (54 + 6 * ((WORD32)(34.f * ((FLOAT32)sr / 2.f) / 12800.f + 0.5f) * 2)) / 2;
  pstr_scratch->p_ol_pitch_R0 = (FLOAT32 *)(temp_ptr);
  temp_ptr += (R0_size) * sizeof(FLOAT32);

  pstr_scratch->p_ltpf_encode_speech_buffer = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG + 1030) * sizeof(FLOAT32);

  pstr_scratch->p_ltpf_encode_buf_tmp = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_ltpf_encode_cor = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_ltpf_encode_cor_int = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_ltpf_encode_speech_buffer; // reused

  pstr_scratch->p_lpd_frm_enc_scratch = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_FRAME + 1) * sizeof(FLOAT32);

  pstr_scratch->p_wsig_buf = (FLOAT32 *)(temp_ptr + 128 * sizeof(FLOAT32));
  temp_ptr += (128 + FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_wsyn_tcx_buf = (FLOAT32 *)(temp_ptr + 128 * sizeof(FLOAT32));
  temp_ptr += (128 + FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_synth_tcx_buf = (FLOAT32 *)(temp_ptr + 128 * sizeof(FLOAT32));
  temp_ptr += (128 + FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_wsyn_buf = (FLOAT32 *)(temp_ptr + 128 * sizeof(FLOAT32));
  temp_ptr += (128 + FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_synth_buf = (FLOAT32 *)(temp_ptr + 128 * sizeof(FLOAT32));
  temp_ptr += (128 + FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_temp_wsyn_buf = (FLOAT32 *)temp_ptr;
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_lp_filter_coeff = (FLOAT32 *)(temp_ptr);
  temp_ptr += ((NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1)) * sizeof(FLOAT32);

  pstr_scratch->p_lp_filter_coeff_q = (FLOAT32 *)(temp_ptr);
  temp_ptr += ((NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1)) * sizeof(FLOAT32);

  pstr_scratch->p_wsp_prev_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr += ((MAX_PITCH1 / OPL_DECIM) + LEN_FRAME) * sizeof(FLOAT32);

  pstr_scratch->ptr_lpd_scratch = (UWORD8 *)temp_ptr;
  temp_ptr += ((2 * (NUM_SUBFR_SUPERFRAME + 1) * (ORDER + 1)) + (4 * (NUM_FRAMES + 1) * ORDER) +
               (((NUM_FRAMES >> 1) + 1) * ORDER) * 4) *
                  sizeof(FLOAT32) +
              100 * sizeof(WORD32) + 6 * sizeof(ia_usac_lpd_scratch); // newly added

  pstr_scratch->p_prm_tcx = (WORD32 *)(temp_ptr);
  temp_ptr += (NUM_TCX80_PRM) * sizeof(WORD32);

  pstr_scratch->p_buf_speech = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * LEN_FRAME + ORDER) * sizeof(FLOAT32);

  pstr_scratch->p_buf_res = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * LEN_FRAME) * sizeof(FLOAT32);

  pstr_scratch->p_buf_signal = (FLOAT32 *)(temp_ptr);
  temp_ptr += (ORDER + LEN_FRAME) * sizeof(FLOAT32);

  pstr_scratch->p_xn1_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_xn_buf_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (128 + FRAME_LEN_LONG + 128) * sizeof(FLOAT32);

  pstr_scratch->p_x_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_x_tmp_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_en_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_alfd_gains_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG / (4 * 8)) * sizeof(FLOAT32);

  pstr_scratch->p_sq_enc_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_sq_quant_tcx = (WORD32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(WORD32);

  pstr_scratch->p_gain1_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_gain2_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_facelp_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_xn2_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_fac_window_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_x1_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_x2_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_y_tcx = (WORD32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(WORD32);

  pstr_scratch->p_in_out_tcx = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG * 2 * 2) * sizeof(FLOAT32);

  pstr_scratch->p_tcx_input = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->ptr_tcx_scratch = (FLOAT32 *)(temp_ptr);
  temp_ptr += 3 * (FRAME_LEN_LONG) * sizeof(pstr_scratch->ptr_tcx_scratch[0]); // newly added

  pstr_scratch->ptr_fdp_int = (WORD32 *)(temp_ptr);
  temp_ptr += (172) * sizeof(pstr_scratch->ptr_fdp_int[0]);

  pstr_scratch->p_tcx_output = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_buf_aut_corr = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_WIN_PLUS) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_buf_aut_corr; // reused

  pstr_scratch->p_xn2 = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH + ORDER) * sizeof(FLOAT32);

  pstr_scratch->p_fac_dec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_right_fac_spec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_x2 = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_param = (WORD32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH + 1) * sizeof(WORD32);

  pstr_scratch->p_x = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_xn_2 = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH + ORDER) * sizeof(FLOAT32);

  pstr_scratch->p_fac_window = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_fir_sig_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr += (LEN_FRAME + 3) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_fir_sig_buf; // reused

  pstr_scratch->p_acelp_ir_buf = (FLOAT32 *)(temp_ptr);

  temp_ptr += (4 * LEN_SUBFR) * sizeof(FLOAT32);

  pstr_scratch->ptr_acelp_scratch = (FLOAT32 *)(temp_ptr); // newly added
  temp_ptr += ((11 * LEN_SUBFR) + (ORDER + LEN_SUBFR + 8) + 1024) *
              sizeof(pstr_scratch->ptr_acelp_scratch[0]);

  pstr_scratch->p_acelp_exc_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr += ((3 * LEN_FRAME) + 1 + 41) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_lpd_frm_enc_scratch; // reused

  pstr_scratch->p_fac_bits_word = (WORD16 *)(temp_ptr);
  temp_ptr += (5000) * sizeof(WORD16);

  pstr_scratch->p_left_fac_timedata_flt = (FLOAT32 *)(temp_ptr);
  temp_ptr += (2 * FAC_LENGTH + ORDER) * sizeof(FLOAT32);

  pstr_scratch->p_left_fac_spec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);

  pstr_scratch->p_fac_prm = (WORD32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH + 1) * sizeof(WORD32);

  pstr_scratch->p_acelp_folded_scratch = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FAC_LENGTH) * sizeof(FLOAT32);
  temp_ptr = (UWORD8 *)pstr_scratch->p_fac_bits_word; // reused

  pstr_scratch->p_exp_spec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_adjthr_ptr_exp_spec = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_adjthr_mdct_spec_float = (FLOAT32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(FLOAT32);

  pstr_scratch->p_adjthr_quant_spec_temp = (WORD16 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(WORD16);

  pstr_scratch->p_degroup_scratch = (WORD32 *)(temp_ptr);
  temp_ptr += (FRAME_LEN_LONG) * sizeof(WORD32);

  /*Newly added*/

  pstr_scratch->ptr_drc_scratch_buf = (FLOAT32 *)(temp_ptr);
  temp_ptr = (UWORD8 *)pstr_scratch->ptr_drc_scratch_buf; // reused
  pstr_scratch->pstr_dmx_scratch = (ia_dmx_sratch *)(temp_ptr);
  temp_ptr = (UWORD8 *)pstr_scratch->pstr_dmx_scratch; // reused

  pstr_scratch->p_c_prev = (WORD32 *)(temp_ptr);
  temp_ptr += (516) * sizeof(WORD32);

  pstr_scratch->p_c_pres = (WORD32 *)(temp_ptr);

  temp_ptr = (UWORD8 *)pstr_scratch->p_c_prev; // reused

  pstr_scratch->ptr_num_fac_bits = (WORD32 *)temp_ptr;
  temp_ptr += MAX_TIME_CHANNELS * sizeof(pstr_scratch->ptr_num_fac_bits[0]);

  pstr_scratch->ptr_tmp_lp_res = (FLOAT32 *)temp_ptr;
  temp_ptr += FAC_LENGTH * sizeof(pstr_scratch->ptr_tmp_lp_res[0]);

  for (WORD32 i = 0; i < 2; i++)
  {
    pstr_scratch->ptr_sfb_form_fac[i] = (FLOAT32 *)temp_ptr;
    temp_ptr += (MAX_NUM_GROUPED_SFB) * sizeof(FLOAT32);
  }
  for (WORD32 i = 0; i < 2; i++)
  {
    pstr_scratch->ptr_sfb_num_relevant_lines[i] = (FLOAT32 *)temp_ptr;
    temp_ptr += (MAX_NUM_GROUPED_SFB) * sizeof(FLOAT32);
  }
  for (WORD32 i = 0; i < 2; i++)
  {
    pstr_scratch->ptr_sfb_ld_energy[i] = (FLOAT32 *)temp_ptr;
    temp_ptr += (MAX_NUM_GROUPED_SFB) * sizeof(FLOAT32);
  }

  pstr_scratch->ptr_fd_scratch = (UWORD8 *)temp_ptr;
  temp_ptr += sizeof(ia_qc_pe_data_struct) +
              (MAX_TIME_CHANNELS * MAX_NUM_GROUPED_SFB * 3) * sizeof(WORD32);
  return;
}

/**
 *  impeghe_limitbitrate
 *
 *  \brief Limit bitrate for element type
 *
 *  \param [in] core_sample_rate  Sample rate
 *  \param [in] num_ch            Number of channels
 *  \param [in] bit_rate          Bitrate
 *
 *  \return WORD32                Bitrate
 */
static WORD32 impeghe_limitbitrate(WORD32 core_sample_rate, WORD32 num_ch, WORD32 bit_rate)
{
  WORD32 transport_bits, prev_bit_rate, shift = 0, iter = 0;
  WORD32 frame_len = FRAME_LEN_LONG >> 1;

  while ((frame_len & ~((1 << (shift + 1)) - 1)) == frame_len &&
         (core_sample_rate & ~((1 << (shift + 1)) - 1)) == core_sample_rate)
  {
    shift++;
  }

  do
  {
    prev_bit_rate = bit_rate;
    /* Assume some worst case */
    transport_bits = 208;

    bit_rate =
        max(bit_rate, ((((40 * num_ch) + transport_bits) * (core_sample_rate)) / frame_len));
    bit_rate =
        min(bit_rate, ((num_ch * 6144) * (core_sample_rate >> shift)) / (frame_len >> shift));

  } while (prev_bit_rate != bit_rate && iter++ < 3);

  return bit_rate;
}

/**
 *  impeghe_enc_init
 *
 *  \brief Enocder initialization
 *
 *  \param [in] pstr_usac_config  Pointer to encoder config structure
 *  \param [in] pstr_usac_state          Poniter to encoder handle structure
 *
 *  \return IA_ERRORCODE         Error code
 */
IA_ERRORCODE impeghe_enc_init(ia_usac_encoder_config_struct *pstr_usac_config,
                              ia_usac_enc_state_struct *pstr_usac_state)
{
  WORD32 err_code = 0;
  LOOPIDX i, j, k, idx, i_ch;
  WORD8 elem_idx = 0;
  ia_usac_data_struct *pstr_usac_data = (&pstr_usac_state->str_usac_enc_data);
  ia_usac_audio_specific_config_struct *pstr_asc = &pstr_usac_config->audio_specific_config;
  ia_usac_config_struct *pstr_asc_usac_config = &(pstr_asc->str_usac_config);
  ia_usac_ltpf_data_struct *pstr_ltpf_data = &pstr_usac_data->ltpf_data[0];
  WORD32 nbuff = FRAME_LEN_LONG << 1;
  WORD32 num_sfb, ptr_sfb_width[MAX_NUM_SFB_LONG], sfb_offset_long[MAX_NUM_SFB_LONG + 1],
      sfb_offset_short[MAX_NUM_SFB_LONG + 1];
  pstr_usac_data->usac_independency_flag_count = 0;
  pstr_usac_data->usac_independency_flag_interval = 25;
  for (j = 0; j < MAX_TIME_CHANNELS; j++)
  {
    memset(pstr_usac_data->overlap_buf[j], 0, nbuff * sizeof(FLOAT64));

    pstr_usac_data->str_ms_info[j].ms_mask = 0;
    for (i = 0; i < MAX_SHORT_WINDOWS; i++)
    {
      for (k = 0; k < MAX_NUM_SFB_LONG; k++)
      {
        pstr_usac_data->str_ms_info[j].ms_used[i][k] = 0;
      }
    }
  }

  impeghe_scratch_mem_init(pstr_usac_data, pstr_usac_config->channels,
                           pstr_usac_config->sampling_rate);
  pstr_usac_state->drc_scratch = pstr_usac_state->hoa_scratch;
  for (i = 0; i < MAX_TIME_CHANNELS; i++)
  {
    if (pstr_usac_config->cmplx_pred_flag)
    {
      pstr_usac_data->str_ms_info[i].ms_mask = 3;
    }
    pstr_usac_data->ptr_dmx_re_save[i] = &pstr_usac_data->arr_dmx_save_float[i][0];
    pstr_usac_data->ptr_dmx_im[i] = &pstr_usac_data->arr_dmx_im[i][0];
  }

  pstr_asc_usac_config->num_elements = 0;
  pstr_asc_usac_config->usac_cfg_ext_present = 0;
  pstr_asc_usac_config->num_config_extensions = 0;

  if (pstr_usac_config->use_oam_element == 1)
  {
    ia_oam_enc_config_struct pstr_oam_config;
    memset(&pstr_oam_config, 0x00, sizeof(pstr_oam_config));

    pstr_oam_config.core_block_size = pstr_usac_config->ccfl;
    pstr_oam_config.high_rate = pstr_usac_config->oam_high_rate;
    pstr_oam_config.replace_radius = pstr_usac_config->oam_replace_radius;
    pstr_oam_config.low_delay = 1;
    for (idx = 0; idx < 6; idx++)
    {
      pstr_oam_config.fixed_values[idx] = pstr_usac_config->oam_fixed_values[idx];
    }
    pstr_oam_config.data_hndl = pstr_usac_config->oam_data_hndl;
    pstr_oam_config.read_data = pstr_usac_config->oam_read_data;
    pstr_oam_config.skip_data = pstr_usac_config->oam_skip_data;
    pstr_oam_config.oam_version = pstr_usac_config->oam_version;
    pstr_oam_config.has_dyn_obj_priority = pstr_usac_config->has_dyn_obj_priority;
    pstr_oam_config.has_uniform_spread = pstr_usac_config->has_uniform_spread;
    pstr_oam_config.num_objects = pstr_usac_config->num_objects;
    pstr_oam_config.num_channels = pstr_usac_config->num_channels;
    pstr_oam_config.extra_objects = pstr_usac_config->extra_objects;

    err_code = impeghe_obj_md_enc_init(&pstr_usac_data->str_oam_state, &pstr_oam_config);
    if (err_code & IA_FATAL_ERROR)
    {
      return err_code;
    }

    ia_usac_enc_element_config_struct *pstr_usac_elem_config =
        &(pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]);
    pstr_asc_usac_config->usac_element_type[pstr_asc_usac_config->num_elements] = ID_USAC_EXT;
    pstr_usac_elem_config->usac_ext_ele_type = ID_EXT_ELE_OAM;
    pstr_usac_elem_config->usac_ext_ele_cfg_len = 0;
    pstr_usac_elem_config->usac_ext_ele_dflt_len_present = 0;
    pstr_usac_elem_config->usac_ext_ele_payload_present = 0;

    pstr_usac_elem_config->oam_has_core_length = pstr_usac_config->oam_has_core_length;
    if (pstr_usac_config->oam_high_rate != 0)
    {
      pstr_usac_elem_config->oam_block_size = pstr_usac_data->str_oam_state.oam_block_size;
    }
    else
    {
      pstr_usac_elem_config->oam_block_size = pstr_usac_config->ccfl;
    }

    pstr_usac_elem_config->oam_has_scrn_rel_objs = pstr_usac_config->oam_has_scrn_rel_objs;
    pstr_usac_elem_config->oam_num_objects = pstr_usac_data->str_oam_state.str_config.num_objects;
    for (idx = 0; idx < pstr_usac_elem_config->oam_num_objects; idx++)
    {
      pstr_usac_elem_config->oam_is_scrn_rel_obj[idx] =
          pstr_usac_config->oam_is_scrn_rel_obj[idx];
    }
    pstr_usac_elem_config->oam_has_dyn_obj_priority =
        pstr_usac_data->str_oam_state.str_config.has_dyn_obj_priority;
    pstr_usac_elem_config->oam_has_uniform_spread =
        pstr_usac_data->str_oam_state.str_config.has_uniform_spread;

    pstr_asc->num_audio_objs = pstr_usac_data->str_oam_state.str_config.num_objects;
    pstr_asc->num_audio_chs = pstr_usac_data->str_oam_state.str_config.num_channels;

    pstr_asc_usac_config->num_elements++;
  }

  if (pstr_usac_config->use_hoa)
  {
    WORD32 res_bitrate;
    memset(&pstr_usac_state->str_hoa_bit_buf, 0, sizeof(ia_bit_buf_struct));
    memset(pstr_usac_state->hoa_bit_stream, 0, sizeof(pstr_usac_state->hoa_bit_stream));

    impeghe_create_bit_buffer(&pstr_usac_state->str_hoa_bit_buf, pstr_usac_state->hoa_bit_stream,
                              sizeof(pstr_usac_state->hoa_bit_stream));
    pstr_usac_state->str_hoa_bit_buf.impeghe_jmp_buf = pstr_usac_state->impegh_jmp_buf;
    err_code = impeghe_hoa_encoder_init(
        &pstr_usac_data->str_hoa_state, &pstr_usac_state->str_hoa_bit_buf,
        pstr_usac_config->hoa_bitrate, &pstr_usac_config->hoa_config, &res_bitrate,
        pstr_usac_state->hoa_scratch);
    if (err_code & IA_FATAL_ERROR)
    {
      return err_code;
    }
    // Bit rate available used as overall bitrate by core coder
    pstr_usac_config->hoa_bitrate = res_bitrate;

    /* the number of input channels read from HOA input will always be 1. Setting number of
           channels after HOA encoder initialization */
    pstr_usac_config->num_trans_ch = pstr_usac_data->str_hoa_state.tot_coders;
    pstr_usac_config->channels =
        pstr_usac_config->num_trans_ch + pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch;

    ia_usac_enc_element_config_struct *pstr_usac_elem_config =
        &(pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]);
    pstr_asc_usac_config->usac_element_type[pstr_asc_usac_config->num_elements] = ID_USAC_EXT;
    pstr_usac_elem_config->usac_ext_ele_type = ID_EXT_ELE_HOA;
    pstr_usac_elem_config->hoa_config_bs = pstr_usac_state->hoa_bit_stream;
    pstr_usac_elem_config->hoa_config_len = pstr_usac_data->str_hoa_state.cfg_bs_bits;
    // Delay frames
    for (i = 0; i < pstr_usac_data->str_hoa_state.frm_bs_cnt; i++)
    {
      pstr_usac_elem_config->hoa_config_len += pstr_usac_data->str_hoa_state.frm_bs_bits[i];
    }
    pstr_asc->num_hoa_transport_channels = pstr_usac_config->num_trans_ch;
    pstr_asc_usac_config->num_elements++;
    pstr_usac_elem_config->usac_ext_ele_cfg_len = 0;
    pstr_usac_elem_config->usac_ext_ele_dflt_len_present = 0;
    pstr_usac_elem_config->usac_ext_ele_payload_present = 0;

    // Send HOA matrix if applicable
    if (pstr_usac_config->use_hoa_matrix)
    {
      // send only when HOA is enabled
      WORD32 bit_cnt = 0;
      WORD32 k;
      ia_bit_buf_struct final_hoa_bs;

      ia_dmx_channel_geometry_struct str_out_geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
      ia_dmx_speaker_information_struct str_out_cfg[HOA_MATRIX_MAX_SPEAKER_COUNT];
      WORD32 num_ren_mat = pstr_usac_config->hoa_config.num_hoa_matrix;

      // Set hoa matrix status to success
      pstr_usac_config->hoa_config.hoa_mtx_status = 0;

      // generate bit_stream
      bit_cnt = 0;
      impeghe_create_bit_buffer(
          &final_hoa_bs,
          pstr_asc_usac_config
              ->usac_cfg_ext_info_buf[pstr_asc_usac_config->num_config_extensions],
          sizeof(pstr_asc_usac_config->usac_cfg_ext_info_buf[0]) /
              sizeof(pstr_asc_usac_config->usac_cfg_ext_info_buf[0][0]));
      final_hoa_bs.impeghe_jmp_buf = pstr_usac_state->impegh_jmp_buf;
      // numOfHoaRenderingMatrices
      bit_cnt += impeghe_write_bits_buf(&final_hoa_bs, num_ren_mat, 5);

      for (k = 0; k < num_ren_mat; k++)
      {
        WORD32 out_dim = pstr_usac_config->hoa_config.hoa_mat_cfg[k].hoa_matrix_out_dim;
        WORD32 input_dim = pstr_usac_config->hoa_config.hoa_mat_cfg[k].hoa_matrix_in_dim;
        WORD32 out_id = pstr_usac_config->hoa_config.hoa_mat_cfg[k].hoa_cicp;
        WORD32 hoa_ren_mat_id = pstr_usac_config->hoa_config.hoa_mat_cfg[k].hoa_rend_id;
        WORD32 num_lfe = 0;
        WORD32 out_cnt = 0;
        WORD32 hoa_matrix_bits;
        /*precision_lvl - Smallest quantization step size[dB]
                0 - 1.0
                1 - 0.5
                2 - 0.25
                3 - 0.125
                Set precision to max and find one that can fit within MAX_HOA_MATRIX_LEN
           */
        WORD32 precision_lvl = 1;

        // Sanity check
        if ((input_dim == pstr_usac_config->hoa_config.num_hoa_coeffs) &&
            ((out_id > 0) && (out_id < 21)))
        {
          /* prepare output config */
          if (impeghe_dmx_get_geometry_from_cicp(out_id, str_out_geometry, &out_cnt, &num_lfe))
          {
            return -1;
          }
          out_cnt += num_lfe;

          if (out_cnt == out_dim)
          {
            /* copy the output geometry to out_cfg */
            for (i = 0; i < out_cnt; ++i)
            {
              str_out_cfg[i].azimuth = str_out_geometry[i].azimuth;
              str_out_cfg[i].elevation = str_out_geometry[i].elevation;
              str_out_cfg[i].is_lfe = str_out_geometry[i].is_lfe;
            }
            /* consistency check: no two identical speakers allowed */
            for (i = 0; i < out_cnt - 1; ++i)
            {
              ia_dmx_speaker_information_struct str_si = str_out_cfg[i];
              for (j = i + 1; j < out_cnt; ++j)
              {
                if ((str_si.azimuth == str_out_cfg[j].azimuth) &&
                    (str_si.elevation == str_out_cfg[j].elevation) &&
                    (str_si.is_lfe == str_out_cfg[j].is_lfe))
                {
                  pstr_usac_config->hoa_config.hoa_mtx_status =
                      IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT;
                  break;
                }
              }
            }

            if (pstr_usac_config->hoa_config.hoa_mtx_status)
            {
              break;
            }
            // HoaRenderingMatrixId
            bit_cnt += impeghe_write_bits_buf(&final_hoa_bs, hoa_ren_mat_id, 7);
            // CICPspeakerLayoutIdx
            bit_cnt += impeghe_write_bits_buf(&final_hoa_bs, out_id, 6);

            do
            {
              precision_lvl--;

              memset(&pstr_usac_state->str_hoa_matrix_bit_buf, 0, sizeof(ia_bit_buf_struct));
              memset(pstr_usac_state->hoa_matrix_bit_stream, 0,
                     sizeof(pstr_usac_state->hoa_matrix_bit_stream));

              // bit buffer init
              impeghe_create_bit_buffer(&pstr_usac_state->str_hoa_matrix_bit_buf,
                                        pstr_usac_state->hoa_matrix_bit_stream,
                                        sizeof(pstr_usac_state->hoa_matrix_bit_stream));
              pstr_usac_state->str_hoa_matrix_bit_buf.impeghe_jmp_buf =
                  pstr_usac_state->impegh_jmp_buf;
              // Encode HOA Matrix call edits the hoa matrix values. So, store initial values,
              // just in case we need to reiterate
              if (impeghe_hoa_matrix_encode(
                      input_dim, out_cnt, str_out_cfg, precision_lvl,
                      &pstr_usac_state->str_hoa_matrix_bit_buf,
                      pstr_usac_config->hoa_config.hoa_mat_cfg[k].hoa_matrix,
                      pstr_usac_state->hoa_scratch))
              {
                pstr_usac_config->hoa_config.hoa_mtx_status =
                    IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT;
                break;
              }
              // HoaMatrixLenBits
              hoa_matrix_bits = pstr_usac_state->str_hoa_matrix_bit_buf.cnt_bits;

            } while ((hoa_matrix_bits > MAX_HOA_MATRIX_LEN) && (precision_lvl > 0));

            if ((hoa_matrix_bits > MAX_HOA_MATRIX_LEN) ||
                (pstr_usac_config->hoa_config.hoa_mtx_status))
            {
              pstr_usac_config->hoa_config.hoa_mtx_status =
                  IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT;
              break;
            }

            bit_cnt += impeghe_write_escape_value(&final_hoa_bs, hoa_matrix_bits, 8, 8, 12);
            // Copy HOA matrix
            for (i = 0; i < (hoa_matrix_bits >> 3); i++)
            {
              bit_cnt += impeghe_write_bits_buf(&final_hoa_bs,
                                                pstr_usac_state->hoa_matrix_bit_stream[i], 8);
            }
            if (hoa_matrix_bits & 0x7)
            {
              bit_cnt +=
                  impeghe_write_bits_buf(&final_hoa_bs, pstr_usac_state->hoa_matrix_bit_stream[i],
                                         (hoa_matrix_bits & 0x7));
            }
          }
          else
          {
            pstr_usac_config->hoa_config.hoa_mtx_status =
                IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT;
            break;
          }
        }
        else
        {
          pstr_usac_config->hoa_config.hoa_mtx_status =
              IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT;
          break;
        }
      }

      if (0 == pstr_usac_config->hoa_config.hoa_mtx_status)
      {
        // Align the size
        impeghe_byte_align_buffer(&final_hoa_bs);

        // Set ext element
        pstr_asc_usac_config->usac_config_ext_type[pstr_asc_usac_config->num_config_extensions] =
            ID_CONFIG_EXT_HOA_MATRIX;
        pstr_asc_usac_config->usac_config_ext_len[pstr_asc_usac_config->num_config_extensions] =
            final_hoa_bs.cnt_bits / 8;
        pstr_asc_usac_config->num_config_extensions++;
        pstr_asc_usac_config->usac_cfg_ext_present = 1;
      }
    }
  }

  pstr_asc->num_sig_grps =
      pstr_asc->num_ch_sig_groups + pstr_asc->num_obj_sig_groups + pstr_asc->num_hoa_sig_groups;
  if (pstr_asc->num_sig_grps == 0)
  {
    if ((pstr_usac_config->aud_ch > 0) || (pstr_usac_config->num_channels > 0))
    {
      pstr_asc->num_ch_per_sig_group[0] =
          pstr_usac_config->aud_ch + pstr_usac_config->num_channels;
      pstr_asc->num_ch_sig_groups = 1;
      pstr_asc->num_sig_grps += 1;
    }
    if (pstr_usac_config->num_objects > 0)
    {
      pstr_asc->num_objs_per_sig_group[0] = pstr_usac_config->num_objects;
      pstr_asc->num_obj_sig_groups = 1;
      pstr_asc->num_sig_grps += 1;
    }
    if (pstr_usac_config->num_trans_ch > 0)
    {
      pstr_asc->num_hoas_per_sig_group[0] = pstr_usac_config->num_trans_ch;
      pstr_asc->num_hoa_sig_groups = 1;
      pstr_asc->num_sig_grps += 1;
    }
  }

  if (pstr_usac_config->mct_mode > -1)
  {
    UWORD32 config_len = 0;
    pstr_usac_data->ext_type[pstr_usac_data->num_ext_elements] = ID_EXT_ELE_MCT;
    for (WORD32 grp = 0; grp < pstr_asc->num_sig_grps; grp++)
    {
      UWORD8 mct_ch_mask[8] = {0};
      UWORD8 *ptr_mct_ch_mask = mct_ch_mask;
      if (grp == pstr_asc->num_ch_sig_groups)
      {
        pstr_usac_data->mct_data[grp].num_channels = pstr_asc->num_objs_per_sig_group[grp];
      }
      else if (grp == pstr_asc->num_ch_sig_groups + pstr_asc->num_obj_sig_groups)
      {
        pstr_usac_data->mct_data[grp].num_channels = pstr_asc->num_hoas_per_sig_group[grp];
      }
      else
      {
        pstr_usac_data->mct_data[grp].num_channels = pstr_asc->num_ch_per_sig_group[grp];
      }
      if (pstr_asc->num_ch_per_sig_group[grp] > 1)
      {
        for (i = 0; i < pstr_usac_data->mct_data[grp].num_channels; i++)
        {
          if (i % 8 != 0 && i != 0)
          {
            ptr_mct_ch_mask[i / 8] <<= 1;
          }
          if (i != 3) // Assuming 4th channel is LFE
          {
            ptr_mct_ch_mask[i / 8] |= 1;
            pstr_usac_data->mct_data[grp].mct_ch_mask[i] = 1;
            pstr_usac_data->mct_data[grp].num_ch_to_apply++;
          }
          else // LFE channel
          {
            ptr_mct_ch_mask[i / 8] |= 0;
            pstr_usac_data->mct_data[grp].mct_ch_mask[i] = 0;
          }
        }
        ptr_mct_ch_mask[pstr_usac_data->mct_data[grp].num_channels / 8] <<=
            ((8 - pstr_usac_data->mct_data[grp].num_channels % 8) % 8);
        config_len = (pstr_usac_data->mct_data[grp].num_channels % 8)
                         ? pstr_usac_data->mct_data[grp].num_channels / 8 + 1
                         : pstr_usac_data->mct_data[grp].num_channels / 8;

        /* Read config into buffer */
        if (config_len > 0)
        {
          pstr_usac_data->ext_elem_config_len[pstr_usac_data->num_ext_elements] = config_len;
          memcpy(pstr_usac_data->ext_elem_config_payload[pstr_usac_data->num_ext_elements],
                 &mct_ch_mask,
                 pstr_usac_data->ext_elem_config_len[pstr_usac_data->num_ext_elements]);
        }
        else
        {
          return -1;
        }

        pstr_asc_usac_config->usac_element_type[pstr_asc_usac_config->num_elements] = ID_USAC_EXT;

        pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
            .usac_ext_ele_type = ID_EXT_ELE_MCT;

        pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
            .usac_ext_ele_dflt_len_present = 0;
        pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
            .usac_ext_ele_dflt_len = 0;
        pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
            .usac_ext_ele_payload_present = 0;
        pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
            .usac_ext_ele_cfg_len =
            pstr_usac_data->ext_elem_config_len[pstr_usac_data->num_ext_elements];
        memcpy(pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]
                   .usac_ext_ele_cfg_payload,
               pstr_usac_data->ext_elem_config_payload[pstr_usac_data->num_ext_elements],
               pstr_usac_data->ext_elem_config_len[pstr_usac_data->num_ext_elements]);

        pstr_usac_data->num_ext_elements++;
        pstr_asc_usac_config->num_elements++;
      }
      pstr_usac_data->mct_data[grp].mct_signalling_type = pstr_usac_config->mct_mode;
    }
  }

  if (pstr_usac_config->use_oam_element)
  {
    if (pstr_usac_config->cicp_index == 0)
    {
      if (pstr_usac_config->num_channels > 0)
      {
        if (pstr_usac_config->num_channels < 7)
        {
          pstr_asc->channel_configuration = pstr_usac_config->num_channels;
        }
        else if (pstr_usac_config->num_channels == 7)
        {
          pstr_asc->channel_configuration = 11; // 7 ls
        }
        else if (pstr_usac_config->num_channels == 8)
        {
          pstr_asc->channel_configuration = 7; // 8 ls
        }
        else if (pstr_usac_config->num_channels == 10)
        {
          pstr_asc->channel_configuration = 16; // 10 ls
        }
        else if (pstr_usac_config->num_channels == 12)
        {
          pstr_asc->channel_configuration = 15; // 12 ls
        }
        else if (pstr_usac_config->num_channels == 14)
        {
          pstr_asc->channel_configuration = 18; // 14 ls
        }
        else if (pstr_usac_config->num_channels == 24)
        {
          pstr_asc->channel_configuration = 13; // 24 ls
        }
        else
        {
          // Use flexible speaker configuration
          pstr_asc->channel_configuration = 6;
        }
      }
      else
      {
        pstr_asc->channel_configuration = 6; // Default value
      }
    }
  }
  else

  {
    pstr_asc->num_audio_chs =
        /*pstr_usac_config->num_trans_ch + */ pstr_usac_config->aud_ch;
    if (pstr_usac_config->cicp_index == 0)
    {
      if (pstr_asc->num_audio_chs < 7)
      {
        pstr_asc->channel_configuration = pstr_asc->num_audio_chs;
      }
      else if (pstr_asc->num_audio_chs == 7)
      {
        pstr_asc->channel_configuration = 11; // 7 ls
      }
      else if (pstr_asc->num_audio_chs < 10)
      {
        pstr_asc->channel_configuration = 7; // 8 ls
      }
      else if (pstr_asc->num_audio_chs < 12)
      {
        pstr_asc->channel_configuration = 16; // 10 ls
      }
      else
      {
        pstr_asc->channel_configuration = 15;
      }
    }
  }

  pstr_usac_config->channels =
      pstr_usac_config->num_trans_ch + pstr_usac_config->aud_ch + pstr_usac_config->num_oam_ch;
  if ((1 == pstr_usac_config->use_hoa) && (0 == pstr_usac_config->use_oam_element))
  {
    if (0 == pstr_asc->num_audio_chs)
    {
      pstr_asc->channel_configuration = 6; // Default
    }
    pstr_asc->num_audio_chs = pstr_usac_config->aud_ch;
  }

  // DRC Config
  if (pstr_usac_config->use_drc_element)
  {
    pstr_usac_config->str_drc_cfg.str_uni_drc_config.str_channel_layout.base_ch_count =
        pstr_usac_config->channels;

    memset(&pstr_usac_data->str_drc_state, 0, sizeof(ia_drc_enc_state));

    err_code = impeghe_drc_enc_init(&pstr_usac_data->str_drc_state, pstr_usac_state->drc_scratch,
                                    &pstr_usac_config->str_drc_cfg);
    if (err_code & IA_FATAL_ERROR)
    {
      return err_code;
    }

    ia_usac_enc_element_config_struct *pstr_usac_elem_config =
        &(pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]);
    pstr_asc_usac_config->usac_element_type[pstr_asc_usac_config->num_elements] = ID_USAC_EXT;
    pstr_usac_elem_config->usac_ext_ele_type = ID_EXT_ELE_UNI_DRC;
    pstr_usac_elem_config->usac_ext_ele_dflt_len_present = 0;
    pstr_usac_elem_config->usac_ext_ele_payload_present = 0;
    pstr_usac_elem_config->drc_config_data = pstr_usac_data->str_drc_state.bit_buf_base_cfg;
    pstr_usac_elem_config->usac_ext_ele_cfg_len =
        (pstr_usac_data->str_drc_state.drc_config_data_size_bit + 7) >> 3;

    pstr_asc_usac_config->num_elements++;
  }

  if (pstr_usac_config->use_downmix_ext_config)
  {
    ia_mpeghe_ext_cfg_downmix_struct *pstr_ext_cfg_downmix = &(
        pstr_asc_usac_config->str_extn_element_config[pstr_asc_usac_config->num_config_extensions]
            .str_ext_cfg_downmix);
    UWORD32 i = 0;

    pstr_ext_cfg_downmix->dmx_config_type =
        pstr_usac_config->str_ext_cfg_downmix_input.dmx_config_type;
    pstr_ext_cfg_downmix->passive_dmx_flag =
        pstr_usac_config->str_ext_cfg_downmix_input.passive_dmx_flag;
    pstr_ext_cfg_downmix->phase_align_strength =
        pstr_usac_config->str_ext_cfg_downmix_input.phase_align_strength;
    pstr_ext_cfg_downmix->immersive_downmix_flag =
        pstr_usac_config->str_ext_cfg_downmix_input.immersive_downmix_flag;
    pstr_ext_cfg_downmix->downmix_id_count =
        pstr_usac_config->str_ext_cfg_downmix_input.downmix_id_count;

    for (i = 0; i < pstr_ext_cfg_downmix->downmix_id_count; i++)
    {
      pstr_ext_cfg_downmix->str_dmx_matrix[i].dmx_id =
          pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].dmx_id;

      pstr_ext_cfg_downmix->str_dmx_matrix[i].dmx_type =
          pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].dmx_type;

      pstr_ext_cfg_downmix->str_dmx_matrix[i].cicp_spk_layout_idx =
          pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].cicp_spk_layout_idx;

      pstr_ext_cfg_downmix->str_dmx_matrix[i].downmix_mtx_count =
          pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].downmix_mtx_count;

      memcpy(pstr_ext_cfg_downmix->str_dmx_matrix[i].num_assigned_group_ids,
             pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].num_assigned_group_ids,
             sizeof(pstr_ext_cfg_downmix->str_dmx_matrix[i].num_assigned_group_ids));

      memcpy(pstr_ext_cfg_downmix->str_dmx_matrix[i].signal_group_id,
             pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].signal_group_id,
             sizeof(pstr_ext_cfg_downmix->str_dmx_matrix[i].signal_group_id));

      pstr_ext_cfg_downmix->str_dmx_matrix[i].str_dmx_mtx_cfg =
          pstr_usac_config->str_ext_cfg_downmix_input.str_dmx_matrix[i].str_dmx_mtx_cfg;
    }

    pstr_asc_usac_config->usac_config_ext_type[pstr_asc_usac_config->num_config_extensions] =
        ID_CONFIG_EXT_DOWNMIX;
    pstr_asc_usac_config->num_config_extensions++;
    pstr_asc_usac_config->usac_cfg_ext_present = 1;
  } // TBD
  if (pstr_asc->str_asi_info.asi_present)
  {
    pstr_asc_usac_config->usac_config_ext_type[pstr_asc_usac_config->num_config_extensions] =
        ID_CONFIG_EXT_AUDIOSCENE_INFO;
    pstr_asc_usac_config->usac_config_ext_len[pstr_asc_usac_config->num_config_extensions] = 1;
    pstr_asc_usac_config->num_config_extensions++;
    pstr_asc_usac_config->usac_cfg_ext_present = 1;
  }
  if (pstr_usac_config->use_drc_element) // For Loudness
  {
    pstr_asc_usac_config->usac_config_ext_type[pstr_asc_usac_config->num_config_extensions] =
        ID_CONFIG_EXT_LOUDNESS_INFO;
    pstr_asc_usac_config->usac_config_ext_len[pstr_asc_usac_config->num_config_extensions] = 1;
    pstr_asc_usac_config->num_config_extensions++;
    pstr_asc_usac_config->usac_cfg_ext_present = 1;
  }

  if (pstr_usac_config->cicp_index != 0)
  {
    pstr_asc->channel_configuration = pstr_usac_config->cicp_index;
  }

  pstr_asc->sampling_frequency = pstr_usac_config->sampling_rate;
  for (idx = 0;
       idx < sizeof(impeghe_sampl_freq_idx_table) / sizeof(impeghe_sampl_freq_idx_table[0]);
       idx++)
  {
    if (pstr_asc->sampling_frequency == impeghe_sampl_freq_idx_table[idx])
      break;
  }

  if (pstr_usac_config->sampling_rate == 29400 || pstr_usac_config->sampling_rate == 14700)
  {
    idx = 0x1f;
  }

  pstr_asc->samp_frequency_index = idx;
  pstr_asc->num_audio_channels = pstr_usac_config->channels;

  elem_idx = pstr_asc_usac_config->num_elements;
  pstr_usac_config->num_ext_elements = elem_idx;
  pstr_asc_usac_config->num_ext_elements = elem_idx;
  i = elem_idx;
  j = 0;
  k = 0;
  WORD32 ch_offset_idx = 0;

  if (pstr_usac_config->user_specified > 0)
  {
    for (j = 0; j < pstr_asc->num_ch_sig_groups; j++)
    {
      for (k = 0; k < pstr_asc->num_ch_per_sig_group[j]; k++)
      {
        if ((abs(pstr_asc->num_ch_idx_per_grp[ch_offset_idx] -
                 pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]) == 1) &&
            (k < pstr_asc->num_ch_per_sig_group[j] - 1))
        {
          pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
              min(pstr_asc->num_ch_idx_per_grp[ch_offset_idx],
                  pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]);
          pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1] =
              pstr_asc->num_ch_idx_per_grp[ch_offset_idx] + 1;
          pstr_asc_usac_config->usac_element_type[i] = ID_USAC_CPE;
          k++;
          ch_offset_idx++;
        }
        else
        {
          pstr_asc_usac_config->usac_element_type[i] = ID_USAC_SCE;
        }
        pstr_asc_usac_config->num_elements += 1;
        pstr_usac_config->num_aud_elements += 1;
        i++;
        ch_offset_idx++;
      }
    }
  }
  else
  {

    if ((pstr_usac_config->aud_ch != 0) ||
        ((pstr_usac_config->num_channels != 0) && (pstr_usac_config->num_objects != 0)))
    {
      pstr_usac_config->num_aud_elements =
          impeghe_get_num_elements(pstr_usac_config->aud_ch + pstr_usac_config->num_channels);
      pstr_asc_usac_config->num_elements += pstr_usac_config->num_aud_elements;
      for (; i < pstr_asc_usac_config->num_elements; i++)
      {
        pstr_asc_usac_config->usac_element_type[i] =
            impeghe_get_element_type((i - pstr_usac_config->num_ext_elements),
                                     (pstr_usac_config->aud_ch + pstr_usac_config->num_channels));
      }
    }
  }

  if (pstr_usac_config->use_oam_element == 1)
  {
    if (pstr_asc->num_obj_sig_groups > 0)
    {
      for (j = 0; j < pstr_asc->num_obj_sig_groups; j++)
      {
        for (k = 0; k < pstr_asc->num_objs_per_sig_group[j]; k++)
        {
          if ((abs(pstr_asc->num_ch_idx_per_grp[ch_offset_idx] -
                   pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]) == 1) &&
              (k < pstr_asc->num_objs_per_sig_group[j] - 1))
          {
            pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
                min(pstr_asc->num_ch_idx_per_grp[ch_offset_idx],
                    pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]);
            pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1] =
                pstr_asc->num_ch_idx_per_grp[ch_offset_idx] + 1;

            pstr_asc_usac_config->usac_element_type[i] = ID_USAC_CPE;
            k++;
            ch_offset_idx++;
          }
          else
          {
            pstr_asc_usac_config->usac_element_type[i] = ID_USAC_SCE;
          }
          pstr_asc_usac_config->num_elements += 1;
          pstr_usac_config->num_oam_elements += 1;
          i++;
          ch_offset_idx++;
        }
      }
    }
    else
    {
      pstr_usac_config->num_oam_elements = (pstr_usac_config->num_oam_ch / 2);
      pstr_asc_usac_config->num_elements += pstr_usac_config->num_oam_elements;
      for (; i < pstr_asc_usac_config->num_elements; i++)
      {
        pstr_asc_usac_config->usac_element_type[i] = ID_USAC_CPE;
      }
      if (pstr_usac_config->num_oam_ch & 0x01)
      {
        pstr_asc_usac_config->usac_element_type[i] = ID_USAC_SCE;
        pstr_asc_usac_config->num_elements += 1;
        pstr_usac_config->num_oam_elements += 1;
        i++;
      }
    }
  }

  // If input is HOA , treat all hoa transport channel as CPE if channel count is even and last as
  // SCE if odd
  if (1 == pstr_usac_config->use_hoa)
  {
    if (pstr_asc->num_hoa_sig_groups > 0)
    {
      for (j = 0; j < pstr_asc->num_hoa_sig_groups; j++)
      {
        for (k = 0; k < pstr_asc->num_hoas_per_sig_group[j]; k++)
        {
          if ((abs(pstr_asc->num_ch_idx_per_grp[ch_offset_idx] -
                   pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]) == 1) &&
              (k < pstr_asc->num_hoas_per_sig_group[j] - 1))
          {
            pstr_asc->num_ch_idx_per_grp[ch_offset_idx] =
                min(pstr_asc->num_ch_idx_per_grp[ch_offset_idx],
                    pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1]);
            pstr_asc->num_ch_idx_per_grp[ch_offset_idx + 1] =
                pstr_asc->num_ch_idx_per_grp[ch_offset_idx] + 1;
            pstr_asc_usac_config->usac_element_type[i] = ID_USAC_CPE;
            k++;
            ch_offset_idx++;
          }
          else
          {
            pstr_asc_usac_config->usac_element_type[i] = ID_USAC_SCE;
          }
          pstr_asc_usac_config->num_elements += 1;
          pstr_usac_config->num_hoa_elements += 1;
          i++;
          ch_offset_idx++;
        }
      }
    }
    else
    {
      pstr_asc_usac_config->num_elements += (pstr_usac_config->num_trans_ch / 2);
      pstr_usac_config->num_hoa_elements = (pstr_usac_config->num_trans_ch / 2);
      for (; i < pstr_asc_usac_config->num_elements; i++)
      {
        pstr_asc_usac_config->usac_element_type[i] = ID_USAC_CPE;
      }
      /*
      If there are odd number of HOA transport channels, last channel will be SCE.
      */
      if (pstr_usac_config->num_trans_ch & 0x01)
      {
        pstr_asc_usac_config->usac_element_type[i] = ID_USAC_SCE;
        pstr_asc_usac_config->num_elements += 1;
        pstr_usac_config->num_hoa_elements += 1;
        i++;
      }
    }
  }

  pstr_usac_config->num_elements = pstr_asc_usac_config->num_elements;

  impeghe_qc_create(&pstr_usac_data->str_qc_main);

  i = elem_idx;
  WORD32 count = pstr_usac_config->num_aud_elements;
  if (pstr_usac_config->num_aud_elements > 2)
  {
    WORD32 num_mono = 0, num_stereo = 0, num_lfe = 0;
    for (WORD8 ch_idx = 0; ch_idx < count; ch_idx++)
    {
      switch (
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
      case ID_USAC_SCE:
        num_mono++;
        break;
      case ID_USAC_CPE:
        num_stereo++;
        break;
      case ID_USAC_LFE:
        num_lfe++;
        break;
      case ID_USAC_EXT:
        break;
      default:
        return -1;
      }
    }

    WORD32 bitrate_per_stereo = (WORD32)((pstr_usac_config->basic_bitrate - (num_lfe)*8000) /
                                         (num_mono * 0.625 + num_stereo));
    WORD32 bitrate_per_mono = (WORD32)(0.625 * bitrate_per_stereo);

    for (WORD8 ch_idx = 0; ch_idx < count; ch_idx++)
    {
      switch (
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
      case ID_USAC_SCE:
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate = bitrate_per_mono;
        break;
      case ID_USAC_CPE:
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate = bitrate_per_stereo;
        break;
      case ID_USAC_LFE:
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate = 8000;
        break;
      case ID_USAC_EXT:
        break;
      default:
        return -1;
      }

      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 1;
      if (ID_USAC_CPE ==
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 2;
      }

      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
          min(360000 * pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch,
              pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate);
      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
          max(8000 * pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch,
              pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate);

      if (ID_USAC_LFE !=
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
            impeghe_limitbitrate(pstr_usac_config->sampling_rate,
                                 pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch,
                                 pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate);
      }

      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].avg_bits =
          (pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate * FRAME_LEN_LONG) /
          pstr_usac_config->sampling_rate;
    }
  }
  else
  {
    for (WORD8 ch_idx = 0; ch_idx < count; ch_idx++)
    {
      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch =
          pstr_usac_config->aud_ch + pstr_usac_config->num_channels;
      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
          pstr_usac_config->basic_bitrate;
      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].avg_bits =
          (pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate * FRAME_LEN_LONG) /
          pstr_usac_config->sampling_rate;
    }
  }

  if (1 == pstr_usac_config->use_oam_element)
  {
    WORD8 ch_idx = pstr_usac_config->num_aud_elements;
    WORD32 count = ch_idx + pstr_usac_config->num_oam_elements;
    for (; ch_idx < count; ch_idx++)
    {
      if (ID_USAC_SCE ==
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 1;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
            pstr_usac_config->oam_bitrate / pstr_usac_config->num_oam_ch;
      }
      else if (ID_USAC_CPE ==
               pstr_asc_usac_config
                   ->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 2;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
            2 * pstr_usac_config->oam_bitrate / pstr_usac_config->num_oam_ch;
      }
      pstr_usac_data->str_qc_main.str_qc_data[ch_idx].avg_bits =
          (pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate * FRAME_LEN_LONG) /
          pstr_usac_config->sampling_rate;
    }
  }

  if (1 == pstr_usac_config->use_hoa)
  {
    WORD8 ch_idx = pstr_usac_config->num_aud_elements + pstr_usac_config->num_oam_elements;
    WORD32 count = ch_idx + pstr_usac_config->num_hoa_elements;
    for (; ch_idx < count; ch_idx++)
    {
      if (ID_USAC_SCE ==
          pstr_asc_usac_config->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 1;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
            pstr_usac_config->hoa_bitrate / pstr_usac_config->num_trans_ch;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].avg_bits =
            (pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate * FRAME_LEN_LONG) /
            pstr_usac_config->sampling_rate;
      }
      else if (ID_USAC_CPE ==
               pstr_asc_usac_config
                   ->usac_element_type[ch_idx + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch = 2;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate =
            2 * pstr_usac_config->hoa_bitrate / pstr_usac_config->num_trans_ch;
        pstr_usac_data->str_qc_main.str_qc_data[ch_idx].avg_bits =
            (pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate * FRAME_LEN_LONG) /
            pstr_usac_config->sampling_rate;
      }
    }
  }

  for (i_ch = 0; i_ch < pstr_usac_config->num_elements - pstr_usac_config->num_ext_elements;
       i_ch++)
  {
    impeghe_bw_init(pstr_usac_config, i_ch);

    pstr_usac_data->prev_aliasing_symmetry[i_ch] = pstr_usac_config->prev_aliasing_symmetry;
    pstr_usac_data->noise_filling[i_ch] = pstr_usac_config->flag_noiseFilling;
    pstr_usac_data->curr_aliasing_symmetry[i_ch] = pstr_usac_config->curr_aliasing_symmetry;
    pstr_usac_data->str_igf_config[i_ch].igf_active = pstr_usac_config->enhanced_noise_filling;
  }

  memset(&pstr_usac_data->str_psy_mod.str_psy_out_data, 0,
         sizeof(ia_psy_mod_out_data_struct) * MAX_TIME_CHANNELS);

  i_ch = 0;
  for (WORD8 ch_idx = 0;
       ch_idx < pstr_asc_usac_config->num_elements - pstr_usac_config->num_ext_elements; ch_idx++)
  {
    impeghe_psy_mod_init(&pstr_usac_data->str_psy_mod, (pstr_usac_config->sampling_rate),
                         pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate,
                         pstr_usac_config->bw_limit[ch_idx],
                         pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch, i_ch, ch_idx);
    i_ch += pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch;
  }

  if (pstr_usac_config->enhanced_noise_filling)
  {
    WORD32 chn = 0;
    for (i_ch = 0; i_ch < pstr_usac_config->num_elements - pstr_usac_config->num_ext_elements;
         i_ch++)
    {
      WORD32 bl = pstr_usac_config->ccfl;
      WORD32 sb;
      WORD8 num_channels = 1;
      WORD32 ch_br = pstr_usac_data->str_qc_main.str_qc_data[i_ch].ch_bitrate;
      pstr_usac_data->str_igf_config[i_ch].igf_independent_tiling = 1;
      if (ID_USAC_CPE ==
          pstr_asc_usac_config->usac_element_type[i_ch + pstr_usac_config->num_ext_elements])
      {
        pstr_usac_data->str_igf_config[i_ch].igf_independent_tiling = 1;
        num_channels = 2;
        ch_br = pstr_usac_data->str_qc_main.str_qc_data[i_ch].ch_bitrate / num_channels;
      }
      if (ch_br > 32000)
      {
        pstr_usac_data->str_igf_config[i_ch].igf_active = 0;
        chn++;
      }
      else
      {
        if (ch_br < 32000)
        {
          pstr_usac_data->str_igf_config[i_ch].igf_use_high_res = 0;
        }
        else
        {
          pstr_usac_data->str_igf_config[i_ch].igf_use_high_res = 1;
        }

        if (pstr_usac_config->igf_start_freq == 0)
        {
          if (ch_br < 32000)
          {
            switch (pstr_usac_config->sampling_rate)
            {
            case 48000:
            case 44100:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR48_BR16;
              break;
            case 32000:
            case 29400:
            case 24000:
            case 22050:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR32_BR16;
              break;
            case 16000:
            case 14700:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR16_BR16;
              break;
            }
          }
          else
          {
            switch (pstr_usac_config->sampling_rate)
            {
            case 48000:
            case 44100:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR48_BR32;
              break;
            case 32000:
            case 29400:
            case 24000:
            case 22050:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR32_BR32;
              break;
            case 16000:
            case 14700:
              pstr_usac_data->str_igf_config[i_ch].igf_start_freq = IGF_START_SR16_BR32;
              break;
            }
          }
        }
        else
        {
          pstr_usac_data->str_igf_config[i_ch].igf_start_freq = pstr_usac_config->igf_start_freq;
        }
        if (pstr_usac_config->igf_stop_freq == 0)
        {
          switch (pstr_usac_config->sampling_rate)
          {
          case 48000:
          case 44100:
            pstr_usac_data->str_igf_config[i_ch].igf_stop_freq = IGF_STOP_SR48;
            break;
          default:
            pstr_usac_data->str_igf_config[i_ch].igf_stop_freq =
                pstr_usac_config->sampling_rate >> 1;
            break;
          }
        }
        else
        {
          pstr_usac_data->str_igf_config[i_ch].igf_stop_freq = pstr_usac_config->igf_stop_freq;
        }

        impeghe_sfb_params_init(pstr_usac_config->sampling_rate, ptr_sfb_width, &num_sfb,
                                ONLY_LONG_SEQUENCE);

        sfb_offset_long[0] = 0;
        k = 0;
        for (i = 0; i < num_sfb; i++)
        {
          sfb_offset_long[i] = k;
          k += ptr_sfb_width[i];
        }
        sfb_offset_long[i] = k;

        pstr_usac_data->str_igf_config[i_ch].is_short_block = 0;
        sb = (WORD32)(1125 * bl * (2 / (FLOAT32)pstr_usac_config->sampling_rate));
        pstr_usac_data->str_igf_config[i_ch].igf_min_l = sb + (sb % 2);
        impeghe_igf_init(&pstr_usac_data->str_igf_config[i_ch], pstr_usac_config->sampling_rate,
                         sfb_offset_long, sfb_offset_short, num_sfb,
                         pstr_usac_config->igf_after_tns_synth,
                         pstr_usac_data->str_psy_mod.str_psy_long_config[i_ch].sfb_active);

        for (i = chn; i < chn + num_channels; i++)
        {
          impeghe_get_tile_info(pstr_usac_data->str_igf_config[i_ch].igf_start_sfb_lb,
                                pstr_usac_data->str_igf_config[i_ch].igf_stop_sfb_lb,
                                pstr_usac_data->str_igf_config[i_ch].igf_use_high_res,
                                pstr_usac_data->str_igf_config[i_ch].igf_min_l, sfb_offset_long,
                                &pstr_usac_data->str_igf_data[i], 0);
        }

        sb = (WORD32)(1125 * 512 * (2 / (FLOAT32)pstr_usac_config->sampling_rate));
        pstr_usac_data->str_igf_config[i_ch].igf_min_tcx = sb + (sb % 2);

        impeghe_sfb_params_init(pstr_usac_config->sampling_rate, ptr_sfb_width, &num_sfb,
                                EIGHT_SHORT_SEQUENCE);

        sfb_offset_short[0] = 0;
        k = 0;
        for (i = 0; i < num_sfb; i++)
        {
          sfb_offset_short[i] = k;
          k += ptr_sfb_width[i];
        }
        sfb_offset_short[i] = k;

        pstr_usac_data->str_igf_config[i_ch].is_short_block = 1;
        bl = pstr_usac_config->ccfl >> 3;
        sb = (WORD32)(1125 * bl * (2 / (FLOAT32)pstr_usac_config->sampling_rate));
        pstr_usac_data->str_igf_config[i_ch].igf_min_s = sb + (sb % 2);
        impeghe_igf_init(&pstr_usac_data->str_igf_config[i_ch], pstr_usac_config->sampling_rate,
                         sfb_offset_long, sfb_offset_short, num_sfb,
                         pstr_usac_config->igf_after_tns_synth,
                         pstr_usac_data->str_psy_mod.str_psy_short_config[i_ch].sfb_active);

        for (i = chn; i < chn + num_channels; i++)
        {
          impeghe_get_tile_info(pstr_usac_data->str_igf_config[i_ch].igf_start_sfb_sb,
                                pstr_usac_data->str_igf_config[i_ch].igf_stop_sfb_sb,
                                pstr_usac_data->str_igf_config[i_ch].igf_use_high_res,
                                pstr_usac_data->str_igf_config[i_ch].igf_min_s, sfb_offset_short,
                                &pstr_usac_data->str_igf_data[i], 1);
        }
        chn++;
        if (num_channels == 2)
        {
          chn++;
        }
      }
    }

    WORD32 all_enf_zero = 0;
    for (i_ch = 0; i_ch < pstr_usac_config->num_elements - pstr_usac_config->num_ext_elements;
         i_ch++)
    {
      if (pstr_usac_data->str_igf_config[i_ch].igf_active == 0)
      {
        all_enf_zero = 1;
      }
      else
      {
        all_enf_zero = 0;
      }
    }
    if (all_enf_zero == 1)
    {
      pstr_usac_config->enhanced_noise_filling = 0;
    }
    else
    {
      pstr_usac_config->igf_stop_freq = pstr_usac_data->str_igf_config[0].igf_stop_freq;
      pstr_usac_config->igf_start_freq = pstr_usac_data->str_igf_config[0].igf_start_freq;
    }
  }

  if (pstr_usac_config->ltpf_enable)
  {
    for (i_ch = 0; i_ch < pstr_usac_config->channels; i_ch++)
    {
      WORD32 err = impeghe_ltpf_init(&pstr_ltpf_data[i_ch], pstr_usac_config->sampling_rate);
      if (err)
      {
        return err;
      }
    }
  }

  pstr_usac_data->fdp_data.fdp_active[0] = 0;
  pstr_usac_data->fdp_data.fdp_active[1] = 0;
  pstr_usac_data->fdp_data.fdp_spacing_idx[0] = -1;
  pstr_usac_data->fdp_data.fdp_spacing_idx[1] = -1;

  for (; elem_idx < pstr_asc_usac_config->num_elements; elem_idx++)
  {
    idx = elem_idx - pstr_asc_usac_config->num_ext_elements;
    pstr_asc_usac_config->str_usac_element_config[idx].noise_filling =
        pstr_usac_data->noise_filling[idx];
    pstr_asc_usac_config->str_usac_element_config[idx].enhanced_noise_filling =
        pstr_usac_data->str_igf_config[idx].igf_active;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_use_enf =
        pstr_usac_data->str_igf_config[idx].igf_use_enf;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_use_high_res =
        pstr_usac_data->str_igf_config[idx].igf_use_high_res;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_use_whitening =
        pstr_usac_data->str_igf_config[idx].igf_use_whitening;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_after_tns_synth =
        pstr_usac_config->igf_after_tns_synth;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_start_index =
        pstr_usac_data->str_igf_config[idx].igf_start_index;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_stop_index =
        pstr_usac_data->str_igf_config[idx].igf_stop_index;
    pstr_asc_usac_config->str_usac_element_config[idx].igf_independent_tiling =
        pstr_usac_data->str_igf_config[idx].igf_independent_tiling;
    pstr_asc_usac_config->str_usac_element_config[idx].full_band_lpd =
        pstr_usac_config->full_band_lpd;
    pstr_asc_usac_config->str_usac_element_config[idx].lpd_stereo_idx =
        pstr_usac_config->stereo_lpd;
    pstr_usac_data->channel_elem_type[idx] = pstr_asc_usac_config->usac_element_type[elem_idx];
  }

  if (pstr_usac_config->use_fill_element)
  {
    ia_usac_enc_element_config_struct *pstr_usac_elem_config =
        &(pstr_asc_usac_config->str_usac_element_config[pstr_asc_usac_config->num_elements]);
    pstr_asc_usac_config->usac_element_type[pstr_asc_usac_config->num_elements] = ID_USAC_EXT;
    pstr_usac_elem_config->usac_ext_ele_type = ID_EXT_ELE_FILL;
    pstr_usac_elem_config->usac_ext_ele_cfg_len = 0;
    pstr_usac_elem_config->usac_ext_ele_dflt_len_present = 0;
    pstr_usac_elem_config->usac_ext_ele_payload_present = 0;
    pstr_asc_usac_config->num_elements++;
  }
  // pstr_asc_usac_config->usac_cfg_ext_present = 0;
  if (pstr_usac_config->codec_mode == USAC_SWITCHED)
  {
    impeghe_init_classification(&pstr_usac_data->str_sig_class_data);
  }

  i_ch = 0;
  for (WORD8 ch_idx = 0;
       ch_idx < pstr_asc_usac_config->num_elements - pstr_usac_config->num_ext_elements; ch_idx++)
  {
    for (idx = 0; idx < pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch; idx++, i_ch++)
    {
      impeghe_init_block_switching(&pstr_usac_data->block_switch_ctrl[i_ch],
                                   pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate,
                                   pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch);
    }
  }

  pstr_asc_usac_config->str_usac_element_config[elem_idx].stereo_config_index = 0;

  for (i_ch = 0; i_ch < pstr_usac_config->channels; i_ch++)
  {
    pstr_usac_config->window_sequence[i_ch] = ONLY_LONG_SEQUENCE;
    pstr_usac_config->window_shape_prev[i_ch] = WIN_SEL_0;
  }

  for (i_ch = 0; i_ch < pstr_usac_config->channels; i_ch++)
  {
    memset(pstr_usac_data->td_in_buf[i_ch], 0,
           (FRAME_LEN_LONG + LEN_NEXT_HIGH_RATE) * sizeof(pstr_usac_data->td_in_buf[i_ch][0]));
  }

  pstr_usac_data->max_bitreservoir_bits = MAX_CHANNEL_BITS * pstr_usac_config->channels;
  pstr_usac_data->available_bitreservoir_bits = pstr_usac_data->max_bitreservoir_bits;
  pstr_usac_data->available_bitreservoir_bits -=
      (pstr_usac_config->bit_rate * pstr_usac_config->ccfl) / pstr_usac_config->sampling_rate;

  i_ch = 0;
  for (WORD8 ch_idx = 0;
       ch_idx < pstr_asc_usac_config->num_elements - pstr_usac_config->num_ext_elements; ch_idx++)
  {
    for (idx = 0; idx < pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch; idx++, i_ch++)
    {
      pstr_usac_data->td_encoder[i_ch]->max_sfb_short =
          pstr_usac_data->str_psy_mod.str_psy_short_config[ch_idx].sfb_count;
      if (pstr_usac_config->tns_select == 0)
      {
        pstr_usac_data->pstr_tns_info[i_ch] = NULL;
      }
      else
      {
        pstr_usac_data->pstr_tns_info[i_ch]->sfb_offset_table_short =
            pstr_usac_data->str_psy_mod.str_psy_short_config[ch_idx].sfb_offset;
        pstr_usac_data->pstr_tns_info[i_ch]->sfb_offset_table_long =
            pstr_usac_data->str_psy_mod.str_psy_long_config[ch_idx].sfb_offset;
        pstr_usac_data->pstr_tns_info[i_ch]->max_sfb_short =
            pstr_usac_data->str_psy_mod.str_psy_short_config[ch_idx].sfb_count;
        pstr_usac_data->pstr_tns_info[i_ch]->max_sfb_long =
            pstr_usac_data->str_psy_mod.str_psy_long_config[ch_idx].sfb_count;
        if (impeghe_tns_init(pstr_usac_config->sampling_rate,
                             pstr_usac_data->str_qc_main.str_qc_data[ch_idx].ch_bitrate /
                                 pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch,
                             pstr_usac_data->pstr_tns_info[i_ch],
                             pstr_usac_data->str_qc_main.str_qc_data[ch_idx].num_ch))
          return -1;
      }
    }
  }

  /* Profile and level updation based on number of channels */
  if ((pstr_usac_config->channels > 5) && (pstr_usac_config->prof_level == PROFILE_LC_LVL1))
    pstr_usac_config->prof_level = PROFILE_LC_LVL2;
  if ((pstr_usac_config->channels > 9) && (pstr_usac_config->prof_level == PROFILE_LC_LVL2))
    pstr_usac_config->prof_level = PROFILE_LC_LVL3;
  if ((pstr_usac_config->channels > 16) && (pstr_usac_config->prof_level == PROFILE_LC_LVL3))
    pstr_usac_config->prof_level = PROFILE_LC_LVL4;

  if ((pstr_usac_config->channels > 5) && (pstr_usac_config->prof_level == PROFILE_BL_LVL1))
    pstr_usac_config->prof_level = PROFILE_BL_LVL2;
  if ((pstr_usac_config->channels > 9) && (pstr_usac_config->prof_level == PROFILE_BL_LVL2))
    pstr_usac_config->prof_level = PROFILE_BL_LVL3;

  /* Profile and level updation based on maximum number of loud speakers */
  if (((pstr_asc->channel_configuration != 0) && (pstr_asc->channel_configuration != 1) &&
       (pstr_asc->channel_configuration != 2)) &&
      (pstr_usac_config->prof_level == PROFILE_LC_LVL1))
  {
    pstr_usac_config->prof_level = PROFILE_LC_LVL2;
  }

  if (((pstr_asc->channel_configuration != 0) && (pstr_asc->channel_configuration != 1) &&
       (pstr_asc->channel_configuration != 2) && (pstr_asc->channel_configuration != 3) &&
       (pstr_asc->channel_configuration != 4) && (pstr_asc->channel_configuration != 5) &&
       (pstr_asc->channel_configuration != 6) && (pstr_asc->channel_configuration != 7) &&
       (pstr_asc->channel_configuration != 9) && (pstr_asc->channel_configuration != 10) &&
       (pstr_asc->channel_configuration != 11) && (pstr_asc->channel_configuration != 12) &&
       (pstr_asc->channel_configuration != 14)) &&
      (pstr_usac_config->prof_level == PROFILE_LC_LVL2))
  {
    pstr_usac_config->prof_level = PROFILE_LC_LVL3;
  }

  if (((pstr_asc->channel_configuration != 0) && (pstr_asc->channel_configuration != 1) &&
       (pstr_asc->channel_configuration != 2) && (pstr_asc->channel_configuration != 3) &&
       (pstr_asc->channel_configuration != 4) && (pstr_asc->channel_configuration != 5) &&
       (pstr_asc->channel_configuration != 6) && (pstr_asc->channel_configuration != 7) &&
       (pstr_asc->channel_configuration != 9) && (pstr_asc->channel_configuration != 10) &&
       (pstr_asc->channel_configuration != 11) && (pstr_asc->channel_configuration != 12) &&
       (pstr_asc->channel_configuration != 14) && (pstr_asc->channel_configuration != 15) &&
       (pstr_asc->channel_configuration != 16) && (pstr_asc->channel_configuration != 17) &&
       (pstr_asc->channel_configuration != 19)) &&
      (pstr_usac_config->prof_level == PROFILE_LC_LVL3))
  {
    pstr_usac_config->prof_level = PROFILE_LC_LVL4;
  }

  switch (pstr_usac_config->prof_level)
  {
  case PROFILE_LC_LVL2:
    pstr_asc->profile_info = CONFIG_PROFILE_LC_LVL2;
    break;
  case PROFILE_LC_LVL3:
    pstr_asc->profile_info = CONFIG_PROFILE_LC_LVL3;
    break;
  case PROFILE_LC_LVL4:
    pstr_asc->profile_info = CONFIG_PROFILE_LC_LVL4;
    break;
  case PROFILE_BL_LVL1:
    pstr_asc->profile_info = CONFIG_PROFILE_BL_LVL1;
    break;
  case PROFILE_BL_LVL2:
    pstr_asc->profile_info = CONFIG_PROFILE_BL_LVL2;
    break;
  case PROFILE_BL_LVL3:
    pstr_asc->profile_info = CONFIG_PROFILE_BL_LVL3;
    break;
  default:
    pstr_asc->profile_info = CONFIG_PROFILE_LC_LVL1;
    break;
  }

  for (i = 0; i < 2; i++)
    pstr_usac_data->str_quant_info[i].reset = 1;

  if (pstr_usac_config->codec_mode == USAC_SWITCHED ||
      pstr_usac_config->codec_mode == USAC_ONLY_TD)
  {
    for (i_ch = 0; i_ch < pstr_usac_config->channels; i_ch++)
    {

      if ((pstr_usac_config->sampling_rate) < SR_MIN ||
          (pstr_usac_config->sampling_rate) > SR_MAX)
      {
        return -1;
      }
      else
      {
        pstr_usac_data->td_encoder[i_ch]->fscale = pstr_usac_config->sampling_rate;

        impeghe_init_td_data(pstr_usac_data->td_encoder[i_ch], pstr_usac_config->ccfl,
                             pstr_usac_config->full_band_lpd);
      }

      pstr_usac_data->td_bitrate[i_ch] = pstr_usac_config->bit_rate;
      pstr_usac_data->td_bitrate[i_ch] /= pstr_usac_config->channels;
      impeghe_config_acelp_core_mode(pstr_usac_data->td_encoder[i_ch],
                                     pstr_usac_config->sampling_rate,
                                     pstr_usac_data->td_bitrate[i_ch]);

      pstr_usac_data->acelp_core_mode[i_ch] = (pstr_usac_data->td_encoder[i_ch])->acelp_core_mode;
    }
  }
  else
  {
    pstr_usac_data->acelp_core_mode[0] = 0;
  }

  for (WORD8 ch = 0; ch < pstr_asc_usac_config->num_elements - pstr_usac_config->num_ext_elements;
       ch++)
  {
    impeghe_qc_init(&pstr_usac_data->str_qc_main.str_qc_data[ch], MAX_CHANNEL_BITS,
                    pstr_usac_config->sampling_rate, pstr_usac_config->bw_limit[ch],
                    pstr_usac_data->str_qc_main.str_qc_data[ch].num_ch);
  }

  return err_code;
}

/**
 *  impeghe_enc_ext_elemts
 *
 *  \brief Encodes extension elements data
 *
 *  \param [in] usac_ext_ele_type         USAC extension element type
 *  \param [in] pstr_usac_config           Pointer to USAC encoder config structure
 *  \param [in,out] pstr_usac_state               Pointer to USAC encoder state structure
 *  \param [in] usac_independency_flg     USAC Independency flag
 *  \param [in] pptr_input                 Pointer to input data
 *  \param [in,out] it_bit_buff           Pointer to bit-buffer
 *  \param [in,out] num_bits_written      Pointer to number of bits written into bit-buffer
 *
 *  \return IA_ERRORCODE                  Error code
 */
static IA_ERRORCODE impeghe_enc_ext_elemts(UWORD32 usac_ext_ele_type,
                                           ia_usac_encoder_config_struct *pstr_usac_config,
                                           ia_usac_enc_state_struct *pstr_usac_state,
                                           WORD32 usac_independency_flg, FLOAT32 **pptr_input,
                                           ia_bit_buf_struct *it_bit_buff,
                                           WORD32 *num_bits_written)
{
  WORD8 idx = 0;
  LOOPIDX idx_2 = 0;
  WORD32 num_bits_payload = 0;
  WORD32 num_byts_payload = 0;
  ia_usac_data_struct *pstr_usac_data = (&pstr_usac_state->str_usac_enc_data);
  ia_usac_audio_specific_config_struct *pstr_asc = &pstr_usac_config->audio_specific_config;
  ia_usac_config_struct *pstr_asc_usac_config = &(pstr_asc->str_usac_config);
  ia_oam_enc_state_struct *pstr_oam_state = &pstr_usac_data->str_oam_state;
  ia_hoa_enc_str *pstr_hoa_state = &pstr_usac_data->str_hoa_state;
  VOID *pstr_scratch = &pstr_usac_data->str_scratch;
  IA_ERRORCODE err_code = IA_NO_ERROR;

  for (idx = 0; idx < pstr_asc_usac_config->num_elements; idx++)
  {
    if (ID_USAC_EXT != pstr_asc_usac_config->usac_element_type[idx])
    {
      continue;
    }

    ia_usac_enc_element_config_struct *pstr_usac_elem_config =
        &(pstr_asc_usac_config->str_usac_element_config[idx]);

    if (usac_ext_ele_type != pstr_usac_elem_config->usac_ext_ele_type)
    {
      continue;
    }

    switch (pstr_usac_elem_config->usac_ext_ele_type)
    {
    case ID_EXT_ELE_UNI_DRC:
    {

      if (pstr_usac_data->str_drc_state.is_first_drc_process_complete == 0)
      {
        impeghe_reset_bit_buffer(&pstr_usac_data->str_drc_state.str_bit_buf_out);
        impeghe_drc_enc(&pstr_usac_data->str_drc_state, pptr_input, 0, &num_bits_payload,
                        pstr_scratch);

        pstr_usac_data->str_drc_state.is_first_drc_process_complete = 1;
        num_bits_payload = 0;
      }

      impeghe_reset_bit_buffer(&pstr_usac_data->str_drc_state.str_bit_buf_out);
      impeghe_drc_enc(&pstr_usac_data->str_drc_state, pptr_input, pstr_usac_state->p_config->ccfl,
                      &num_bits_payload, pstr_scratch);

      num_byts_payload = (num_bits_payload + 7) >> 3;
    }
    break;

    case ID_EXT_ELE_OAM:
    {
      impeghe_reset_bit_buffer(&pstr_oam_state->str_bit_buf_out);

      err_code = impeghe_obj_md_enc(pstr_oam_state, usac_independency_flg,
                                    &pstr_oam_state->str_bit_buf_out, &num_bits_payload);
      if (err_code & IA_FATAL_ERROR)
      {
        return err_code;
      }

      num_byts_payload = (num_bits_payload + 7) >> 3;
    }
    break;
    case ID_EXT_ELE_HOA:
    {
      if (pstr_usac_state->p_config->use_drc_element)
      {
        num_bits_payload = pstr_hoa_state->prev_frm_bs_bits[0];
      }
      else
      {
        num_bits_payload = pstr_hoa_state->frm_bs_bits[0];
      }
      num_byts_payload = (num_bits_payload + 7) >> 3;
    }
    break;
    default:
    {
    }
    break;
    }

    if (num_byts_payload <= 0)
    {
      *num_bits_written += impeghe_write_bits_buf(it_bit_buff, 0, 1); // usacExtElementPresent
    }
    else
    {
      *num_bits_written += impeghe_write_bits_buf(it_bit_buff, 1, 1); // usacExtElementPresent

      *num_bits_written +=
          impeghe_write_bits_buf(it_bit_buff, 0, 1); // usacExtElementUseDefaultLength

      if (num_byts_payload >= 255)
      {
        *num_bits_written +=
            impeghe_write_bits_buf(it_bit_buff, 255, 8); // usacExtElementPayloadLength

        UWORD16 value_add = num_byts_payload - 255 + 2;
        *num_bits_written += impeghe_write_bits_buf(it_bit_buff, value_add, 16); // valueAdd
      }
      else
      {
        *num_bits_written += impeghe_write_bits_buf(it_bit_buff, num_byts_payload,
                                                    8); // usacExtElementPayloadLength
      }

      switch (pstr_usac_elem_config->usac_ext_ele_type)
      {
      case ID_EXT_ELE_UNI_DRC:
      {
        for (idx_2 = 0; idx_2 < num_byts_payload; idx_2++)
        {
          *num_bits_written += impeghe_write_bits_buf(
              it_bit_buff, pstr_usac_data->str_drc_state.bit_buf_base_out[idx_2], 8);
        }
      }
      break;
      case ID_EXT_ELE_OAM:
      {
        for (idx_2 = 0; idx_2 < num_byts_payload; idx_2++)
        {
          *num_bits_written +=
              impeghe_write_bits_buf(it_bit_buff, pstr_oam_state->bit_buf_base_out[idx_2], 8);
        }
      }
      break;
      case ID_EXT_ELE_HOA:
      {
        if (pstr_usac_state->p_config->use_drc_element)
        {
          for (idx_2 = 0; idx_2 < num_byts_payload; idx_2++)
          {
            *num_bits_written += impeghe_write_bits_buf(
                it_bit_buff, pstr_usac_state->prev_hoa_bit_stream[idx_2], 8);
          }
        }
        else
        {
          for (idx_2 = 0; idx_2 < num_byts_payload; idx_2++)
          {
            *num_bits_written +=
                impeghe_write_bits_buf(it_bit_buff, pstr_usac_state->hoa_bit_stream[idx_2], 8);
          }
        }
      }
      break;
      default:
      {
      }
      break;
      }
    }
  }

  return err_code;
}

/**
 *  impeghe_core_coder_process
 *
 *  \brief Main processing function
 *
 *  \param [in] pptr_input        Pointer to input data
 *  \param [in] pstr_usac_config  Pointer to encoder config structure
 *  \param [in] pstr_usac_state          Pointer to encoder handle structure
 *  \param [in] it_bit_buff      Bit buffer
 *
 *  \return IA_ERRORCODE         Error code
 */
IA_ERRORCODE impeghe_core_coder_process(FLOAT32 **pptr_input,
                                        ia_usac_encoder_config_struct *pstr_usac_config,
                                        ia_usac_enc_state_struct *pstr_usac_state,
                                        ia_bit_buf_struct *it_bit_buff)
{
  IA_ERRORCODE err = 0;
  LOOPIDX i_ch, i, k;
  ia_usac_data_struct *pstr_usac_data = &pstr_usac_state->str_usac_enc_data;
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  ia_usac_ltpf_data_struct *pstr_ltpf_data = &pstr_usac_data->ltpf_data[0];
  ia_usac_audio_specific_config_struct *pstr_asc = &pstr_usac_config->audio_specific_config;
  WORD32 ol_pitch = 0;
  FLOAT32 norm_corr = 0;
  WORD32 bits_written;

  WORD32 next_window_sequence[2] = {0};
  WORD32 new_win_seq[2] = {0};
  ia_sfb_params_struct *pstr_sfb_prms = &pstr_usac_config->str_sfb_prms;
  memset(pstr_sfb_prms, 0, sizeof(ia_sfb_params_struct));
  WORD32 *ptr_num_window_groups = pstr_sfb_prms->num_window_groups;
  WORD32 average_bits_total;
  WORD32 min_bits_needed;
  WORD32 num_bits;
  WORD32 num_bits_ext_elem;
  WORD32 num_bits_oam;
  WORD32 padding_bits;
  WORD32 *ptr_common_win = pstr_sfb_prms->common_win;
  WORD32 usac_independency_flg;
  WORD32 mod[NUM_FRAMES] = {0};
  WORD32 len_frame;
  WORD32 len_lpc0;
  WORD32 len_next_high_rate;
  WORD8 elem_idx, nr_core_coder_channels = 0, chn = 0;
  WORD32 ch_offset = pstr_asc->num_ch_idx_per_grp[0], ch_offset_idx = 0;
  WORD32 elem_idx_max = pstr_usac_config->num_elements;
  WORD32 used_bits = 0;
  usac_independency_flg = !(pstr_usac_data->usac_independency_flag_count %
                            pstr_usac_data->usac_independency_flag_interval);
  len_frame = pstr_usac_config->ccfl;
  len_lpc0 = (LEN_LPC0 * len_frame) / FRAME_LEN_LONG;
  len_next_high_rate = (LEN_NEXT_HIGH_RATE * len_frame) / FRAME_LEN_LONG;

  average_bits_total =
      (pstr_usac_config->bit_rate * pstr_usac_config->ccfl) / pstr_usac_config->sampling_rate;

  if (pstr_usac_config->mct_mode > -1)
  {
    for (WORD32 grp = 0; grp < pstr_asc->num_sig_grps; grp++)
    {
      if ((pstr_asc->num_ch_per_sig_group[grp] + pstr_asc->num_objs_per_sig_group[grp] +
           pstr_asc->num_hoas_per_sig_group[grp]) > 1)
      {
        pstr_usac_data->mct_data[grp].mct_ext_data[0] = '\0';
        pstr_usac_data->mct_data[grp].mct_ext_data_size = 0;
        pstr_usac_data->mct_data[grp].mct_ext_data_present = 1;
        pstr_usac_data->mct_data[grp].ext_elem_idx = 0;
        average_bits_total -= pstr_usac_data->mct_data[grp].mct_ext_data_size * 8;

        used_bits += pstr_usac_data->mct_data[grp].mct_ext_data_size * 8;
        used_bits += 10;

        if (pstr_usac_data->mct_data[grp].mct_ext_data_size >= 255)
          used_bits += 16;

        elem_idx_max -= 1;
      }
    }
  }

  min_bits_needed = (long)(pstr_usac_data->available_bitreservoir_bits + 2 * average_bits_total -
                           pstr_usac_data->max_bitreservoir_bits);
  if (min_bits_needed < 0)
  {
    min_bits_needed = 0;
  }

  num_bits = 0;

  impeghe_write_bits_buf(it_bit_buff, usac_independency_flg, 1);
  num_bits++;

  if (pstr_usac_config->use_oam_element == 1)
  {
    elem_idx_max -= 1;
  }

  if (pstr_usac_config->use_drc_element == 1)
  {
    elem_idx_max -= 1;
  }

  if (pstr_usac_config->use_hoa)
  {
    elem_idx_max -= 1;
  }
  if (pstr_usac_data->core_mode[0] == CORE_MODE_FD)
  {
    for (elem_idx = 0; elem_idx < elem_idx_max; elem_idx++)
    {
      if (pstr_usac_data->channel_elem_type[elem_idx] != ID_USAC_LFE)
      {
        nr_core_coder_channels =
            (pstr_usac_data->channel_elem_type[elem_idx] == ID_USAC_CPE) ? 2 : 1;

        for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
        {
          impeghe_block_switching(&pstr_usac_data->block_switch_ctrl[i_ch], pptr_input[i_ch]);
        }
      }
      else
      {
        pstr_usac_data->block_switch_ctrl[ch_offset].window_seq = ONLY_LONG_SEQUENCE;
      }
      ch_offset_idx += nr_core_coder_channels;
      ch_offset = pstr_asc->num_ch_idx_per_grp[ch_offset_idx];
    }
  }
  ch_offset_idx = 0;
  ch_offset = pstr_asc->num_ch_idx_per_grp[0];

  for (elem_idx = 0; elem_idx < elem_idx_max; elem_idx++)
  {
    switch (pstr_usac_data->channel_elem_type[elem_idx])
    {
    case ID_USAC_SCE:
      nr_core_coder_channels = 1;
      break;
    case ID_USAC_CPE:
      nr_core_coder_channels = 2;
      break;
    case ID_USAC_LFE:
      nr_core_coder_channels = 1;
      break;
    }

    if (pstr_usac_data->channel_elem_type[elem_idx] == ID_USAC_LFE)
    {
      for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
      {
        pstr_usac_data->core_mode[i_ch] = 0;
      }
    }

    i_ch = ch_offset;
    if (nr_core_coder_channels == 2)
    {
      impeghe_sync_block_switching(&pstr_usac_data->block_switch_ctrl[i_ch],
                                   &pstr_usac_data->block_switch_ctrl[i_ch + 1]);
    }

    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      switch (pstr_usac_config->codec_mode)
      {
      case USAC_SWITCHED:
        if (pstr_usac_data->str_sig_class_data.coding_mode == 2)
        {
          pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_FD;
        }
        else
        {
          pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_TD;
        }
        break;
      case USAC_ONLY_FD:
        pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_FD;
        break;
      case USAC_ONLY_TD:
        pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_TD;
        break;
      default:
        return (-1);
      }
      if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_TD)
      {
        for (i = 0; i < pstr_usac_config->ccfl; i++)
        {
          pstr_usac_data->ptr_2frame_time_data[i_ch][i] = pstr_usac_data->ptr_time_data[i_ch][i];
          pstr_usac_data->ptr_2frame_time_data[i_ch][pstr_usac_config->ccfl + i] =
              pstr_usac_data->ptr_look_ahead_time_data[i_ch][i];
          pstr_usac_data->ptr_time_data[i_ch][i] =
              pstr_usac_data->ptr_look_ahead_time_data[i_ch][i];
          pstr_usac_data->ptr_look_ahead_time_data[i_ch][i] = (FLOAT64)pptr_input[i_ch][i];
        }
      }
      else
      {
        for (i = 0; i < pstr_usac_config->ccfl; i++)
        {
          pstr_usac_data->ptr_2frame_time_data[i_ch][i] = pstr_usac_data->ptr_time_data[i_ch][i];
          pstr_usac_data->ptr_2frame_time_data[i_ch][pstr_usac_config->ccfl + i] =
              pstr_usac_data->ptr_look_ahead_time_data[i_ch][i];
          pstr_usac_data->ptr_time_data[i_ch][i] = pptr_input[i_ch][i];
          pstr_usac_data->ptr_look_ahead_time_data[i_ch][i] = (FLOAT64)pptr_input[i_ch][i];
        }
      }

      for (i = 0; i < len_frame + len_next_high_rate; i++)
      {
        pstr_usac_data->td_in_buf[i_ch][i] =
            (FLOAT32)(pstr_usac_data->ptr_2frame_time_data[i_ch][i + TD_BUFFER_OFFSET]);
      }
      for (i = 0; i < len_frame + len_next_high_rate + len_lpc0; i++)
      {
        pstr_usac_data->td_in_prev_buf[i_ch][i] = (FLOAT32)(
            pstr_usac_data->ptr_2frame_time_data[i_ch][i + TD_BUFFER_OFFSET - len_lpc0]);
      }

      if (pstr_usac_config->ltpf_enable)
      {
        impeghe_ol_pitch_estim(pptr_input[i_ch], &pstr_ltpf_data[i_ch], &ol_pitch, &norm_corr,
                               pstr_usac_config->ccfl, pstr_scratch);

        err = impeghe_ltpf_encode(&pstr_ltpf_data[i_ch], pptr_input[i_ch], pstr_usac_config->ccfl,
                                  ol_pitch, norm_corr, pstr_scratch);
        if (err)
          return -1;
      }

      if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_FD)
      {
        pstr_sfb_prms->window_sequence[i_ch] = pstr_usac_data->block_switch_ctrl[i_ch].window_seq;
        pstr_usac_config->window_sequence[i_ch] = pstr_sfb_prms->window_sequence[i_ch];
        new_win_seq[chn] = pstr_usac_data->block_switch_ctrl[i_ch].next_win_seq;
      }

      impeghe_sfb_params_init(pstr_usac_config->sampling_rate,
                              pstr_sfb_prms->sfb_width_table[i_ch], &pstr_sfb_prms->num_sfb[i_ch],
                              pstr_sfb_prms->window_sequence[i_ch]);

      pstr_sfb_prms->sfb_offset[i_ch][0] = 0;
      k = 0;
      for (i = 0; i < pstr_sfb_prms->num_sfb[i_ch]; i++)
      {
        pstr_sfb_prms->sfb_offset[i_ch][i] = k;
        k += pstr_sfb_prms->sfb_width_table[i_ch][i];
      }
      pstr_sfb_prms->sfb_offset[i_ch][i] = k;

      if (pstr_usac_data->core_mode[i_ch] != CORE_MODE_TD)
      {
        next_window_sequence[chn] = new_win_seq[chn];
        if (pstr_usac_data->core_mode_next[i_ch] == CORE_MODE_TD)
        {
          next_window_sequence[chn] = EIGHT_SHORT_SEQUENCE;
        }

        if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_TD &&
            pstr_usac_data->core_mode_next[i_ch] != CORE_MODE_TD)
        {
          next_window_sequence[chn] = LONG_STOP_SEQUENCE;
        }

        if (next_window_sequence[chn] == EIGHT_SHORT_SEQUENCE)
        {
          if (pstr_sfb_prms->window_sequence[i_ch] == ONLY_LONG_SEQUENCE)
          {
            pstr_sfb_prms->window_sequence[i_ch] = LONG_START_SEQUENCE;
          }
          if (pstr_sfb_prms->window_sequence[i_ch] == LONG_STOP_SEQUENCE)
          {
            pstr_sfb_prms->window_sequence[i_ch] = STOP_START_SEQUENCE;
          }
        }

        if (next_window_sequence[chn] == ONLY_LONG_SEQUENCE)
        {
          if (pstr_sfb_prms->window_sequence[i_ch] == EIGHT_SHORT_SEQUENCE)
          {
            next_window_sequence[chn] = LONG_STOP_SEQUENCE;
          }
        }

        if (pstr_sfb_prms->window_sequence[i_ch] == EIGHT_SHORT_SEQUENCE)
        {
          ptr_num_window_groups[i_ch] = pstr_usac_data->block_switch_ctrl[i_ch].tot_grps_cnt;
          for (i = 0; i < 8; i++)
          {
            pstr_sfb_prms->window_group_length[i_ch][i] =
                pstr_usac_data->block_switch_ctrl[i_ch].group_len[i];
          }
        }
        else
        {
          ptr_num_window_groups[i_ch] = 1;
          pstr_sfb_prms->window_group_length[i_ch][0] = 1;
        }

        pstr_sfb_prms->window_shape[i_ch] = pstr_usac_config->window_shape_prev[i_ch];

        if (pstr_usac_data->channel_elem_type[elem_idx] == ID_USAC_LFE)
        {
          pstr_usac_data->prev_aliasing_symmetry[i_ch] =
              pstr_usac_data->curr_aliasing_symmetry[i_ch] = 0;
          pstr_usac_data->noise_filling[elem_idx] = 0;
          pstr_usac_data->str_igf_config[elem_idx].igf_active = 0;
        }

        err = impeghe_fd_mdct(pstr_usac_data, pstr_usac_config, i_ch);
        if (err)
          return err;

        if (pstr_sfb_prms->window_sequence[i_ch] != EIGHT_SHORT_SEQUENCE)
        {
          impeghe_psy_mod_lb(
              &pstr_usac_data->str_psy_mod, &pstr_usac_data->str_igf_data[i_ch], pstr_sfb_prms,
              pstr_usac_data->spectral_line_vector[i_ch], pstr_usac_data->pstr_tns_info,
              pstr_usac_config->tns_select, i_ch, chn, pstr_usac_config->igf_after_tns_synth,
              &pstr_usac_data->str_igf_config[elem_idx],
              pstr_usac_data->channel_elem_type[elem_idx], pstr_scratch->p_tns_filter, elem_idx,
              pstr_scratch->ptr_igf_scratch, pstr_scratch->ptr_tns_scratch);
        }
        else
        {
          impeghe_psy_mod_sb(
              &(pstr_usac_data->str_psy_mod), &pstr_usac_data->str_igf_data[i_ch], pstr_sfb_prms,
              pstr_usac_data->spectral_line_vector[i_ch], pstr_usac_data->pstr_tns_info,
              pstr_usac_config->tns_select, i_ch, chn, pstr_usac_config->igf_after_tns_synth,
              &pstr_usac_data->str_igf_config[elem_idx],
              pstr_usac_data->channel_elem_type[elem_idx], pstr_scratch->p_tns_filter, elem_idx,
              pstr_scratch->ptr_igf_scratch, pstr_scratch->ptr_tns_scratch);
        }

        if (pstr_usac_data->str_igf_config[elem_idx].igf_active == 1)
        {
          pstr_usac_data->str_psy_mod.str_psy_out_data[i_ch].max_sfb_per_grp =
              pstr_usac_data->str_igf_config[elem_idx].m_igf_start_sfb;
        }

        pstr_sfb_prms->max_sfb[i_ch] =
            pstr_usac_data->str_psy_mod.str_psy_out_data[i_ch].max_sfb_per_grp;
      }
    }
    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      if (nr_core_coder_channels == 2)
      {
        if ((pstr_sfb_prms->window_shape[i_ch] == pstr_sfb_prms->window_shape[i_ch + 1]) &&
            (pstr_sfb_prms->window_sequence[i_ch] == pstr_sfb_prms->window_sequence[i_ch + 1]) &&
            (pstr_usac_data->core_mode[i_ch] == pstr_usac_data->core_mode[i_ch + 1]))
        {
          ptr_common_win[i_ch] = ptr_common_win[i_ch + 1] = 1;
        }
        else
        {
          ptr_common_win[i_ch] = 0;
        }
        chn++;
      }
      else
      {
        ptr_common_win[i_ch] = 0;
      }
    }

    if (nr_core_coder_channels == 2)
    {
      if (i_ch == (ch_offset + 1))
      {
        if (pstr_sfb_prms->window_sequence[i_ch] != EIGHT_SHORT_SEQUENCE)
        {
          impeghe_calc_ms_band_energy(
              pstr_usac_data->spectral_line_vector[ch_offset],
              pstr_usac_data->spectral_line_vector[ch_offset + 1],
              pstr_usac_data->str_psy_mod.str_psy_long_config[elem_idx].sfb_offset,
              pstr_usac_data->str_psy_mod.str_psy_long_config[elem_idx].sfb_active,
              pstr_usac_data->str_psy_mod.str_psy_data[ch_offset].ptr_sfb_energy_long_ms,
              pstr_usac_data->str_psy_mod.str_psy_data[ch_offset + 1].ptr_sfb_energy_long_ms);
        }
        else
        {
          for (WORD32 w = 0; w < MAX_SHORT_WINDOWS; w++)
          {
            WORD32 w_offset = w * FRAME_LEN_SHORT;

            impeghe_calc_ms_band_energy(
                pstr_usac_data->spectral_line_vector[ch_offset] + w_offset,
                pstr_usac_data->spectral_line_vector[ch_offset + 1] + w_offset,
                pstr_usac_data->str_psy_mod.str_psy_short_config[elem_idx].sfb_offset,
                pstr_usac_data->str_psy_mod.str_psy_short_config[elem_idx].sfb_active,
                pstr_usac_data->str_psy_mod.str_psy_data[ch_offset].ptr_sfb_energy_short_ms[w],
                pstr_usac_data->str_psy_mod.str_psy_data[ch_offset + 1]
                    .ptr_sfb_energy_short_ms[w]);
          }
        }
      }
    }
    if ((nr_core_coder_channels == 2)
            ? ((pstr_usac_data->core_mode[ch_offset] == CORE_MODE_FD) &&
               (pstr_usac_data->core_mode[ch_offset + 1] == CORE_MODE_FD))
            : ((pstr_usac_data->core_mode[ch_offset] == CORE_MODE_FD)))
    {
      impeghe_grouping(pstr_sfb_prms, nr_core_coder_channels, pstr_usac_data, pstr_usac_config,
                       ch_offset, elem_idx);

      if (nr_core_coder_channels == 2)
      {
        impeghe_stereo_proc(pstr_sfb_prms, usac_independency_flg, pstr_usac_data,
                            pstr_usac_config, ch_offset, elem_idx);
      }
    }
    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      pstr_usac_config->window_shape_prev_copy[i_ch] = pstr_usac_config->window_shape_prev[i_ch];
      pstr_usac_config->window_shape_prev[i_ch] = pstr_sfb_prms->window_shape[i_ch];
      pstr_usac_config->window_sequence_prev_copy[i_ch] =
          pstr_usac_config->window_sequence_prev[i_ch];
      pstr_usac_config->window_sequence_prev[i_ch] =
          pstr_usac_config->window_sequence[i_ch]; /* Newly inserted for complex prediction */
      pstr_usac_config->window_sequence[i_ch] = next_window_sequence[chn];
      pstr_usac_data->core_mode_prev_copy[i_ch] = pstr_usac_data->core_mode_prev[i_ch];
      pstr_usac_data->core_mode_copy[i_ch] = pstr_usac_data->core_mode[i_ch];
      pstr_usac_data->core_mode_prev[i_ch] = pstr_usac_data->core_mode[i_ch];
      pstr_usac_data->core_mode[i_ch] = pstr_usac_data->core_mode_next[i_ch];
    }
    ch_offset_idx += nr_core_coder_channels;
    ch_offset = pstr_asc->num_ch_idx_per_grp[ch_offset_idx];
  }

  //*********************************************************************************************

  if (pstr_usac_config->use_oam_element == 1)
  {
    num_bits_oam = 0;
    err = impeghe_enc_ext_elemts(ID_EXT_ELE_OAM, pstr_usac_config, pstr_usac_state,
                                 usac_independency_flg, pptr_input, it_bit_buff, &num_bits_oam);
    if (err & IA_FATAL_ERROR)
    {
      return err;
    }
    num_bits += num_bits_oam;
    pstr_usac_data->oam_data_bit_cnt = num_bits_oam;
  }

  if (pstr_usac_config->use_hoa)
  {
    WORD32 num_bits_hoa = 0;
    err = impeghe_enc_ext_elemts(ID_EXT_ELE_HOA, pstr_usac_config, pstr_usac_state,
                                 usac_independency_flg, pptr_input, it_bit_buff, &num_bits_hoa);
    if (err & IA_FATAL_ERROR)
    {
      return err;
    }
    num_bits += num_bits_hoa;
  }

  WORD32 ch_mct = 0;
  for (i = 0; i < (WORD32)pstr_asc->num_sig_grps; i++)
  {
    if ((pstr_asc->num_ch_per_sig_group[i] + pstr_asc->num_objs_per_sig_group[i] +
         pstr_asc->num_hoas_per_sig_group[i]) > 1)
    {
      if (pstr_usac_config->mct_mode > -1)
      {
        impeghe_encode_mct(pstr_usac_data->spectral_line_vector, pstr_sfb_prms->window_sequence,
                           pstr_sfb_prms->sfb_offset[0], pstr_sfb_prms->num_sfb[0],
                           &pstr_usac_data->mct_data[i], ch_mct);
        err = impeghe_write_mct_data(&pstr_usac_data->mct_data[i], usac_independency_flg,
                                     pstr_usac_data->mct_data[i].mct_ext_data,
                                     &pstr_usac_data->mct_data[i].mct_ext_data_size,
                                     &pstr_usac_data->mct_data[i].mct_ext_data_present,
                                     pstr_usac_data->str_scratch.ptr_scratch_buf);
        if (err)
        {
          return err;
        }
        used_bits += pstr_usac_data->mct_data[i].mct_ext_data_size * 8;
        pstr_usac_data->mct_data_bit_cnt = used_bits;

        /* Extension data - mpegh3daExtElement */
        num_bits += impeghe_write_mpegh3da_ext_ele(
            it_bit_buff, pstr_usac_data->mct_data[i].mct_ext_data_present,
            pstr_usac_data->mct_data[i].mct_ext_data_size,
            pstr_usac_data->mct_data[i].mct_ext_data);
        pstr_usac_data->mct_data[i].ext_elem_idx++;
      }
    }
    ch_mct += pstr_asc->num_ch_per_sig_group[i];
  }

  if (pstr_usac_config->use_drc_element)
  {
    num_bits_ext_elem = 0;
    err = impeghe_enc_ext_elemts(ID_EXT_ELE_UNI_DRC, pstr_usac_config, pstr_usac_state,
                                 usac_independency_flg, pptr_input, it_bit_buff,
                                 &num_bits_ext_elem);
    if (err & IA_FATAL_ERROR)
    {
      return err;
    }
    num_bits += num_bits_ext_elem;
  }
  ch_offset_idx = 0;
  ch_offset = pstr_asc->num_ch_idx_per_grp[0];
  for (elem_idx = 0; elem_idx < elem_idx_max; elem_idx++)
  {
    switch (pstr_usac_data->channel_elem_type[elem_idx])
    {
    case ID_USAC_SCE:
      nr_core_coder_channels = 1;
      break;
    case ID_USAC_CPE:
      nr_core_coder_channels = 2;
      break;
    case ID_USAC_LFE:
      nr_core_coder_channels = 1;
      break;
    }

    /* Rewriting the original code_mode value */
    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      pstr_usac_data->core_mode[i_ch] = pstr_usac_data->core_mode_copy[i_ch];
      pstr_usac_data->core_mode_prev[i_ch] = pstr_usac_data->core_mode_prev_copy[i_ch];
      pstr_usac_config->window_shape_prev[i_ch] = pstr_usac_config->window_shape_prev_copy[i_ch];
      pstr_usac_config->window_sequence_prev[i_ch] =
          pstr_usac_config->window_sequence_prev_copy[i_ch];
    }

    if (pstr_usac_data->channel_elem_type[elem_idx] != ID_USAC_LFE)
    {
      for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
      {
        impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->core_mode[i_ch], 1);
        num_bits++;
      }
    }
    else
    {
      for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
      {
        pstr_usac_data->core_mode[i_ch] = 0;
      }
    }

    /* Separate Mid / Side signals in SLPD */
    if ((pstr_usac_data->core_mode[i_ch] == CORE_MODE_TD) && (pstr_usac_config->stereo_lpd) &&
        (ID_USAC_CPE == pstr_usac_data->channel_elem_type[elem_idx]))
    {
      FLOAT32 m, s;
      for (i = 0; i < (FRAME_LEN_LONG + LEN_NEXT_HIGH_RATE); i++)
      {
        m = pstr_usac_data->td_in_buf[ch_offset + 0][i];
        s = pstr_usac_data->td_in_buf[ch_offset + 1][i];
        pstr_usac_data->td_in_buf[ch_offset + 0][i] = (m + s) * 0.5f;
        pstr_usac_data->td_in_buf[ch_offset + 1][i] = (m - s) * 0.5f;
      }
    }

    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      if (pstr_usac_data->channel_elem_type[elem_idx] != ID_USAC_LFE)
      {
        switch (pstr_usac_config->codec_mode)
        {
        case USAC_SWITCHED:
          if (pstr_usac_data->str_sig_class_data.coding_mode == 2)
          {
            pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_FD;
          }
          else
          {
            pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_TD;
          }
          break;
        case USAC_ONLY_FD:
          pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_FD;
          break;
        case USAC_ONLY_TD:
          pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_TD;
          break;
        default:
          return (-1);
        }
      }
      else
      {
        pstr_usac_data->core_mode_next[i_ch] = CORE_MODE_FD;
        pstr_usac_data->core_mode[i_ch] = CORE_MODE_FD;
      }

      if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_FD)
      {
        pstr_sfb_prms->window_sequence[i_ch] = pstr_usac_data->block_switch_ctrl[i_ch].window_seq;
        pstr_usac_config->window_sequence[i_ch] = pstr_sfb_prms->window_sequence[i_ch];
        new_win_seq[chn] = pstr_usac_data->block_switch_ctrl[i_ch].next_win_seq;
      }

      if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_TD)
      {
        impeghe_lpd_encode(pstr_usac_data, mod, usac_independency_flg, pstr_usac_config,
                           len_frame, i_ch, chn, it_bit_buff, elem_idx);

        num_bits = it_bit_buff->cnt_bits;

        if ((pstr_usac_data->core_mode_prev[i_ch] == CORE_MODE_FD) && (mod[0] == 0))
        {
          for (i = 0; i < pstr_usac_data->num_td_fac_bits[i_ch]; i++)
          {
            impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->fac_out_stream[i_ch][i], 1);
            num_bits++;
          }
        }
      }
      else
      {
        next_window_sequence[chn] = new_win_seq[chn];
        if (pstr_usac_data->core_mode_next[i_ch] == CORE_MODE_TD)
        {
          next_window_sequence[chn] = EIGHT_SHORT_SEQUENCE;
        }

        if (pstr_usac_data->core_mode[i_ch] == CORE_MODE_TD &&
            pstr_usac_data->core_mode_next[i_ch] != CORE_MODE_TD)
        {
          next_window_sequence[chn] = LONG_STOP_SEQUENCE;
        }

        if (next_window_sequence[chn] == EIGHT_SHORT_SEQUENCE)
        {
          if (pstr_sfb_prms->window_sequence[i_ch] == ONLY_LONG_SEQUENCE)
          {
            pstr_sfb_prms->window_sequence[i_ch] = LONG_START_SEQUENCE;
          }
          if (pstr_sfb_prms->window_sequence[i_ch] == LONG_STOP_SEQUENCE)
          {
            pstr_sfb_prms->window_sequence[i_ch] = STOP_START_SEQUENCE;
          }
        }

        if (next_window_sequence[chn] == ONLY_LONG_SEQUENCE)
        {
          if (pstr_sfb_prms->window_sequence[i_ch] == EIGHT_SHORT_SEQUENCE)
          {
            next_window_sequence[chn] = LONG_STOP_SEQUENCE;
          }
        }

        if (pstr_sfb_prms->window_sequence[i_ch] == EIGHT_SHORT_SEQUENCE)
        {
          for (i = 0; i < MAX_SHORT_WINDOWS; i++)
          {
            pstr_sfb_prms->window_group_length[i_ch][i] =
                pstr_usac_data->block_switch_ctrl[i_ch].group_len[i];
          }
        }
        else
        {
          pstr_sfb_prms->window_group_length[i_ch][0] = 1;
        }

        pstr_sfb_prms->window_shape[i_ch] = pstr_usac_config->window_shape_prev[i_ch];
      }
    }
    if ((nr_core_coder_channels == 2)
            ? ((pstr_usac_data->core_mode[ch_offset] == CORE_MODE_FD) &&
               (pstr_usac_data->core_mode[ch_offset + 1] == CORE_MODE_FD))
            : ((pstr_usac_data->core_mode[ch_offset] == CORE_MODE_FD)))
    {
      err = impeghe_fd_encode(pstr_sfb_prms, usac_independency_flg, pstr_usac_data,
                              pstr_usac_config, it_bit_buff, nr_core_coder_channels, ch_offset,
                              elem_idx, &bits_written);
      if (err)
      {
        return err;
      }

      num_bits += bits_written;
    }
    WORD32 idx = 0;
    for (chn = 0, i_ch = ch_offset; chn < nr_core_coder_channels; chn++, i_ch++)
    {
      if (pstr_usac_config->fdp_enable == 1)
        impeghe_fd_chn_fdp_decode_update(pstr_usac_data, pstr_sfb_prms->max_sfb[i_ch], i_ch,
                                         pstr_sfb_prms->window_sequence[i_ch] !=
                                             EIGHT_SHORT_SEQUENCE,
                                         pstr_scratch->ptr_fdp_int, idx);
      pstr_usac_config->window_shape_prev[i_ch] = pstr_sfb_prms->window_shape[i_ch];
      pstr_usac_config->window_sequence_prev[i_ch] =
          pstr_usac_config->window_sequence[i_ch]; /* Newly inserted for complex prediction */
      pstr_usac_config->window_sequence[i_ch] = next_window_sequence[chn];
      pstr_usac_data->core_mode_prev[i_ch] = pstr_usac_data->core_mode[i_ch];
      pstr_usac_data->core_mode[i_ch] = pstr_usac_data->core_mode_next[i_ch];
      idx++;
    }
    ch_offset_idx += nr_core_coder_channels;
    ch_offset = pstr_asc->num_ch_idx_per_grp[ch_offset_idx];
  }
  //*********************************************************************************************

  if (pstr_usac_config->use_fill_element)
  {
    WORD32 full_elem_num_bits = 0;
    padding_bits = min_bits_needed - num_bits;
    full_elem_num_bits = impeghe_write_fill_ele(it_bit_buff, padding_bits);
    num_bits += full_elem_num_bits;
  }

  pstr_usac_data->available_bitreservoir_bits -= num_bits;

  if (num_bits % 8)
  {
    pstr_usac_data->available_bitreservoir_bits -= 8 - (num_bits % 8);
  }
  pstr_usac_data->available_bitreservoir_bits += average_bits_total;

  if (pstr_usac_data->available_bitreservoir_bits > pstr_usac_data->max_bitreservoir_bits)
  {
    pstr_usac_data->available_bitreservoir_bits = pstr_usac_data->max_bitreservoir_bits;
  }

  pstr_usac_data->usac_independency_flag_count =
      (pstr_usac_data->usac_independency_flag_count + 1) %
      pstr_usac_data->usac_independency_flag_interval;

  return 0;
}
