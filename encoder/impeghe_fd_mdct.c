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
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
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

#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_fft.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"
#include "impeghe_windowing.h"

/**
 *  impeghe_fd_mdct_short
 *
 *  \brief Perform MDCT on short block
 *
 *  \param [in] pstr_usac_data         Pointer to encoder data structure
 *  \param [in] pstr_usac_config       Pointer to encoder config structure
 *  \param [in] ch_idx                 Channel index
 *  \param [in] transform_kernel_type  transform kernel type
 *
 *  \return IA_ERRORCODE               Error code
 */
static IA_ERRORCODE impeghe_fd_mdct_short(ia_usac_data_struct *pstr_usac_data,
                                          ia_usac_encoder_config_struct *pstr_usac_config,
                                          WORD32 ch_idx, WORD32 transform_kernel_type)
{
  IA_ERRORCODE err_code = 0;
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  IA_ERRORCODE err_code_2 = 0;
  FLOAT64 *ptr_windowed_buf = pstr_scratch->p_fd_mdct_windowed_short_buf;
  WORD32 n_long = pstr_usac_config->ccfl;
  WORD32 n_short = pstr_usac_config->ccfl >> 3;
  FLOAT64 *ptr_in_data = pstr_usac_data->ptr_time_data[ch_idx];
  FLOAT64 *ptr_out_mdct = pstr_usac_data->spectral_line_vector[ch_idx];
  FLOAT64 *ptr_out_mdst = pstr_usac_data->mdst_spectrum[ch_idx];
  WORD32 window_shape = pstr_usac_config->window_shape_prev[ch_idx];
  FLOAT64 *ptr_win_gen_medium = NULL, *ptr_win_gen_short = NULL;
  FLOAT64 *ptr_overlap = pstr_usac_data->overlap_buf[ch_idx];
  WORD32 nflat_ls;
  LOOPIDX i, k;

  memset(ptr_windowed_buf, 0, 2 * n_short * sizeof(FLOAT64));
  nflat_ls = (n_long - n_short) >> 1;
  err_code = impeghe_calc_window(&ptr_win_gen_short, n_short, window_shape);
  if (err_code)
    return err_code;
  err_code = impeghe_calc_window(&ptr_win_gen_medium, n_short, 0);
  if (err_code)
    return err_code;
  ptr_overlap += nflat_ls;

  for (k = MAX_SHORT_WINDOWS - 1; k-- >= 0;)
  {
    for (i = 0; i < n_short; i++)
    {
      ptr_windowed_buf[i] = ptr_win_gen_short[i] * ptr_overlap[i];
    }
    for (i = 0; i < n_short; i++)
    {
      ptr_windowed_buf[i + n_short] =
          ptr_win_gen_medium[n_short - 1 - i] * ptr_overlap[i + n_short];
    }
    ptr_win_gen_medium = ptr_win_gen_short;

    err_code = impeghe_fft_based_mdct(ptr_windowed_buf, ptr_out_mdct, n_short,
                                      transform_kernel_type, pstr_scratch);
    err_code_2 = impeghe_fft_based_mdct(ptr_windowed_buf, ptr_out_mdst, n_short, 3, pstr_scratch);
    if (err_code)
      return err_code;
    if (err_code_2)
      return err_code_2;
    ptr_out_mdct += n_short;
    ptr_out_mdst += n_short;
    ptr_overlap += n_short;
  }

  ptr_overlap = pstr_usac_data->overlap_buf[ch_idx];
  memcpy(ptr_overlap, ptr_overlap + n_long, 576 * sizeof(*ptr_overlap));
  memcpy(ptr_overlap + 576, ptr_in_data, n_long * sizeof(*ptr_overlap));

  return err_code;
}

/**
 *  impeghe_fd_mdct_long
 *
 *  \brief Perform MDCT on long block
 *
 *  \param [in] pstr_usac_data         Pointer to encoder data structure
 *  \param [in] pstr_usac_config       Pointer to encoder config structure
 *  \param [in] ch_idx                 Channel index
 *  \param [in] window_sequence        Window sequence
 *  \param [in] transform_kernel_type  transform kernel type
 *
 *  \return IA_ERRORCODE               Error code
 */
static IA_ERRORCODE impeghe_fd_mdct_long(ia_usac_data_struct *pstr_usac_data,
                                         ia_usac_encoder_config_struct *pstr_usac_config,
                                         WORD32 ch_idx, WORD32 window_sequence,
                                         WORD32 transform_kernel_type)
{
  IA_ERRORCODE err_code = 0;
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  IA_ERRORCODE err_code_2 = 0;
  FLOAT64 *ptr_windowed_buf = pstr_scratch->p_fd_mdct_windowed_long_buf;
  WORD32 n_long = pstr_usac_config->ccfl;
  WORD32 n_short = pstr_usac_config->ccfl >> 3;
  WORD32 prev_mode = (pstr_usac_data->core_mode_prev[ch_idx] == CORE_MODE_TD);
  WORD32 next_mode = (pstr_usac_data->core_mode_next[ch_idx] == CORE_MODE_TD);
  FLOAT64 *ptr_in_data = pstr_usac_data->ptr_time_data[ch_idx];
  FLOAT64 *ptr_out_mdct = pstr_usac_data->spectral_line_vector[ch_idx];
  FLOAT64 *ptr_out_mdst = pstr_usac_data->mdst_spectrum[ch_idx];
  WORD32 window_shape = pstr_usac_config->window_shape_prev[ch_idx];
  FLOAT64 *ptr_win_long = NULL, *ptr_win_med = NULL;
  WORD32 win_len;
  FLOAT64 *ptr_overlap = pstr_usac_data->overlap_buf[ch_idx];

  WORD32 nflat_ls;

  memset(ptr_windowed_buf, 0, 2 * n_long * sizeof(FLOAT64));

  switch (window_sequence)
  {
  case ONLY_LONG_SEQUENCE:
    err_code = impeghe_calc_window(&ptr_win_long, n_long, window_shape);
    if (err_code)
      return err_code;
    impeghe_windowing_long(ptr_overlap, ptr_win_long, ptr_windowed_buf, ptr_in_data, n_long);
    break;

  case LONG_START_SEQUENCE:
    win_len = n_short << next_mode;
    nflat_ls = (n_long - win_len) >> 1;
    err_code = impeghe_calc_window(&ptr_win_long, n_long, window_shape);
    if (err_code)
      return err_code;
    err_code = impeghe_calc_window(&ptr_win_med, win_len, 0);
    if (err_code)
      return err_code;

    impeghe_windowing_long_start(ptr_overlap, ptr_win_long, ptr_windowed_buf, ptr_in_data, n_long,
                                 nflat_ls, ptr_win_med, win_len);
    break;

  case LONG_STOP_SEQUENCE:
    win_len = n_short << prev_mode;
    nflat_ls = (n_long - win_len) >> 1;
    err_code = impeghe_calc_window(&ptr_win_long, n_long, window_shape);
    if (err_code)
      return err_code;
    err_code = impeghe_calc_window(&ptr_win_med, win_len, 1);
    if (err_code)
      return err_code;

    impeghe_windowing_long_stop(ptr_overlap, ptr_win_long, ptr_windowed_buf, ptr_in_data, n_long,
                                nflat_ls, ptr_win_med, win_len);
    break;

  case STOP_START_SEQUENCE:
    win_len = n_short << (prev_mode | next_mode);
    err_code = impeghe_calc_window(&ptr_win_med, win_len, window_shape);
    if (err_code)
      return err_code;

    impeghe_windowing_stop_start(ptr_overlap, ptr_windowed_buf, ptr_win_med, win_len, n_long);
    break;
  }

  err_code = impeghe_fft_based_mdct(ptr_windowed_buf, ptr_out_mdct, n_long, transform_kernel_type,
                                    pstr_scratch);
  err_code_2 = impeghe_fft_based_mdct(ptr_windowed_buf, ptr_out_mdst, n_long, 3, pstr_scratch);
  if (err_code)
    return err_code;
  if (err_code_2)
    return err_code_2;
  return 0;
}

/**
 *  impeghe_fd_mdct
 *
 *  \brief Perform MDCT on long/short block
 *
 *  \param [in] pstr_usac_data    Pointer to encoder data structure
 *  \param [in] pstr_usac_config  Pointer to encoder config structure
 *  \param [in] ch_idx            Channel index
 *
 *  \return IA_ERRORCODE          Error code
 */
WORD32 impeghe_fd_mdct(ia_usac_data_struct *pstr_usac_data,
                       ia_usac_encoder_config_struct *pstr_usac_config, WORD32 ch_idx)
{
  IA_ERRORCODE err_code = 0;
  WORD32 window_sequence = pstr_usac_config->window_sequence[ch_idx];
  WORD32 transform_kernel_type = (pstr_usac_data->prev_aliasing_symmetry[ch_idx] << 1) |
                                 pstr_usac_data->curr_aliasing_symmetry[ch_idx];

  if (window_sequence != EIGHT_SHORT_SEQUENCE)
  {
    err_code = impeghe_fd_mdct_long(pstr_usac_data, pstr_usac_config, ch_idx, window_sequence,
                                    transform_kernel_type);
  }
  else
  {
    err_code =
        impeghe_fd_mdct_short(pstr_usac_data, pstr_usac_config, ch_idx, transform_kernel_type);
  }

  return err_code;
}
