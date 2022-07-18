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
#include "impeghe_bitbuffer.h"
#include "impeghe_cnst.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_ms.h"
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
#include "impeghe_write_bitstream.h"
#include "impeghe_lpd.h"
#include "impeghe_cplx_pred.h"

/**
 *  impeghe_fd_encode
 *
 *  \brief Write encoded bit stream
 *
 *  \param [in] pstr_sfb_prms           Pointer to sfb params
 *  \param [in] usac_independancy_flag  USAC independency flag
 *  \param [in] pstr_usac_data               Pointer to encoder data structure
 *  \param [in] pstr_usac_config             Pointer to encoder config structure
 *  \param [in] it_bit_buff             Bit buffer
 *  \param [in] nr_core_coder_ch        Number of channels in element
 *  \param [in] chn                     Channel offset
 *  \param [in] ele_id                  Element index
 *  \param [out] bit_written            Bits written
 *
 *  \return IA_ERRORCODE                Error code
 */
IA_ERRORCODE impeghe_fd_encode(ia_sfb_params_struct *pstr_sfb_prms, WORD32 usac_independancy_flag,
                               ia_usac_data_struct *pstr_usac_data,
                               ia_usac_encoder_config_struct *pstr_usac_config,
                               ia_bit_buf_struct *it_bit_buff, WORD32 nr_core_coder_ch,
                               WORD32 chn, WORD32 ele_id, WORD32 *bit_written)
{

  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  IA_ERRORCODE err_code = 0;
  LOOPIDX i_ch, idx = 0;
  WORD32 *ptr_num_fac_bits = pstr_scratch->ptr_num_fac_bits;
  WORD32 tns_data_present[2];
  WORD32 *ptr_core_mode_next = pstr_usac_data->core_mode_next;
  WORD32 *ptr_core_mode_prev = pstr_usac_data->core_mode_prev;
  *bit_written = 0;
  memset(pstr_scratch->ptr_num_fac_bits, 0,
         MAX_TIME_CHANNELS * sizeof(pstr_scratch->ptr_num_fac_bits[0]));
  for (i_ch = chn; i_ch < chn + nr_core_coder_ch; i_ch++)
  {
    tns_data_present[idx] = pstr_usac_data->pstr_tns_info[i_ch] != NULL;

    if (tns_data_present[idx])
    {
      tns_data_present[idx] = pstr_usac_data->pstr_tns_info[i_ch]->tns_data_present;
    }
    idx++;
  }

  idx = 0;
  for (i_ch = chn; i_ch < chn + nr_core_coder_ch; i_ch++)
  {
    memset(pstr_scratch->p_reconstructed_time_signal[idx], 0, 4096 * sizeof(FLOAT64));
    err_code = impeghe_fd_fac(
        pstr_sfb_prms->grouped_sfb_offset[i_ch], pstr_sfb_prms->max_sfb[i_ch],
        pstr_usac_data->ptr_2frame_time_data[i_ch], pstr_sfb_prms->window_sequence[i_ch],
        pstr_scratch->p_reconstructed_time_signal[idx], pstr_usac_data->td_encoder[i_ch],
        ((pstr_usac_data->td_encoder[i_ch]->prev_mode == 0) && ptr_core_mode_prev[i_ch]) ==
            CORE_MODE_TD,
        ptr_core_mode_next[i_ch] == CORE_MODE_TD, pstr_usac_data->fac_out_stream[i_ch],
        &ptr_num_fac_bits[i_ch], pstr_scratch);
    if (err_code)
    {
      return err_code;
    }
    idx++;
  }

  err_code = impeghe_quantize_spec(pstr_sfb_prms, usac_independancy_flag, nr_core_coder_ch,
                                   pstr_usac_data, pstr_usac_config, chn, ele_id);
  if (err_code)
    return err_code;

  for (i_ch = chn; i_ch < chn + nr_core_coder_ch; i_ch++)
  {
    pstr_sfb_prms->window_shape[i_ch] =
        pstr_usac_data->str_psy_mod.str_psy_out_data[i_ch].window_shape;
  }

  if (nr_core_coder_ch == 1 && pstr_usac_data->channel_elem_type[ele_id] != ID_USAC_LFE)
  {
    impeghe_write_bits_buf(it_bit_buff, tns_data_present[0], 1);
    *bit_written = *bit_written + 1;
  }
  if (nr_core_coder_ch == 2)
  {
    *bit_written = *bit_written + impeghe_write_cpe(pstr_sfb_prms, it_bit_buff, tns_data_present,
                                                    usac_independancy_flag, pstr_usac_config,
                                                    pstr_usac_data, chn, ele_id);
  }
  idx = 0;
  for (i_ch = chn; i_ch < chn + nr_core_coder_ch; i_ch++)
  {
    *bit_written =
        *bit_written + impeghe_write_fd_data(it_bit_buff, pstr_sfb_prms, ptr_num_fac_bits[i_ch],
                                             usac_independancy_flag, pstr_usac_data,
                                             pstr_usac_config, i_ch, ele_id, idx);
    idx++;
  }

  return err_code;
}
