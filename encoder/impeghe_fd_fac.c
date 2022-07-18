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
#include <math.h>
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
#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_tcx_mdct.h"
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
#include "impeghe_lpd_rom.h"
#include "impeghe_lpd.h"
#include "impeghe_avq_enc.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_decode_fd_fac
 *
 *  \brief Utility function that decodes FD FAC parameters.
 *
 *  \param [in ] ptr_fac_prms   Pointer to FAC parameters buffer.
 *  \param [in ] len_subfrm     Length of sub frame.
 *  \param [in ] fac_len        FAC length
 *  \param [in ] ptr_lpc_coeffs Pointer to LPC coefficients.
 *  \param [in ] zir_sig        Pointer to zero input response signal.
 *  \param [out] ptr_fac_dec    Pointer to FAC decoder buffer.
 *  \param [in ] pstr_scratch   Pointer to scratch buffer.
 *
 *  \return VOID
 */
static VOID impeghe_decode_fd_fac(WORD32 *ptr_fac_prms, WORD32 len_subfrm, WORD32 fac_len,
                                  FLOAT32 *ptr_lpc_coeffs, FLOAT32 *zir_sig, FLOAT32 *ptr_fac_dec,
                                  impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *x = pstr_scratch->p_x;
  FLOAT32 *xn2 = pstr_scratch->p_xn_2;
  FLOAT32 fac_gain;
  LOOPIDX i;
  const FLOAT32 *sin_window;
  FLOAT32 *fac_window = pstr_scratch->p_fac_window;
  FLOAT32 Ap[ORDER + 1];

  if (fac_len == 64)
  {
    sin_window = impeghe_sin_window_128;
  }
  else
  {
    sin_window = impeghe_sin_window_256;
  }

  if (ptr_lpc_coeffs != NULL && ptr_fac_dec != NULL)
  {
    fac_gain = (FLOAT32)pow(10.0f, ((FLOAT32)ptr_fac_prms[0]) / 28.0f);
    for (i = 0; i < fac_len; i++)
    {
      x[i] = (FLOAT32)ptr_fac_prms[i + 1] * fac_gain;
    }

    impeghe_tcx_mdct(x, xn2, fac_len, pstr_scratch);

    impeghe_get_weighted_lpc(ptr_lpc_coeffs, Ap);

    memset(xn2 + fac_len, 0, fac_len * sizeof(FLOAT32));
    impeghe_synthesis_tool_float(Ap, xn2, ptr_fac_dec, 2 * fac_len, xn2 + fac_len,
                                 pstr_scratch->p_buf_synthesis_tool);

    if (zir_sig != NULL)
    {
      for (i = 0; i < fac_len; i++)
      {
        fac_window[i] = sin_window[i] * sin_window[(2 * fac_len) - 1 - i];
        fac_window[fac_len + i] = 1.0f - (sin_window[fac_len + i] * sin_window[fac_len + i]);
      }
      for (i = 0; i < fac_len; i++)
      {
        ptr_fac_dec[i] += zir_sig[1 + (len_subfrm / 2) + i] * fac_window[fac_len + i] +
                          zir_sig[1 + (len_subfrm / 2) - 1 - i] * fac_window[fac_len - 1 - i];
      }
    }
  }

  return;
}

/**
 *  impeghe_fac_apply
 *
 *  \brief Applies forward aliasing cancellation.
 *
 *  \param [in ] orig           Pointer to original signal.
 *  \param [in ] len_subfrm     Length of sub frame.
 *  \param [in ] fac_len        FAC data length.
 *  \param [in ] low_pass_line  Low pass spectral line value.
 *  \param [in ] target_br      Target bitrate value.
 *  \param [in ] synth          Pointer to synthesis buffer.
 *  \param [in ] ptr_lpc_coeffs Pointer to LPC coefficients buffer
 *  \param [in ] fac_bits_word  Pointer to FAC bits words
 *  \param [in ] num_fac_bits   Pointer to number of FAC bits.
 *  \param [in ] pstr_scratch   Pointer to scratch memory.
 *
 *  \return VOID
 */
VOID impeghe_fac_apply(FLOAT32 *orig, WORD32 len_subfrm, WORD32 fac_len, WORD32 low_pass_line,
                       WORD32 target_br, FLOAT32 *synth, FLOAT32 *ptr_lpc_coeffs,
                       WORD16 *fac_bits_word, WORD32 *num_fac_bits,
                       impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *xn2 = pstr_scratch->p_xn2;
  FLOAT32 *fac_dec = pstr_scratch->p_fac_dec;
  FLOAT32 *right_fac_spec = pstr_scratch->p_right_fac_spec;
  FLOAT32 *x2 = pstr_scratch->p_x2;
  WORD32 *param = pstr_scratch->p_param;
  FLOAT32 Ap[ORDER + 1];
  FLOAT32 fac_gain;
  LOOPIDX i;
  WORD32 index;
  WORD32 num_enc_bits = 0;
  WORD32 start_right = 2 * len_subfrm - fac_len;

  *num_fac_bits = 0;

  memset(xn2, 0, (FAC_LENGTH + ORDER) * sizeof(FLOAT32)); // need to check further the change

  memcpy(xn2 + ORDER, &orig[start_right], fac_len * sizeof(FLOAT32));
  for (i = 0; i < fac_len; i++)
  {
    xn2[ORDER + i] -= synth[start_right + i];
  }

  impeghe_get_weighted_lpc(ptr_lpc_coeffs, Ap);
  impeghe_compute_lp_residual(Ap, xn2 + ORDER, x2, fac_len);
  for (i = 0; i < fac_len; i++)
  {
    x2[i] = x2[i] * (2.0f / (FLOAT32)fac_len);
  }

  impeghe_tcx_mdct(x2, right_fac_spec, fac_len, pstr_scratch);

  memset(&right_fac_spec[low_pass_line], 0, (fac_len - low_pass_line) * sizeof(FLOAT32));

  fac_gain = impeghe_calc_sq_gain(right_fac_spec, target_br, fac_len, pstr_scratch->p_sq_gain_en);
  index = (WORD32)floor(0.5f + (28.0f * (FLOAT32)log10(fac_gain)));
  if (index < 0)
    index = 0;
  if (index > 127)
    index = 127;
  param[0] = index;
  fac_gain = (FLOAT32)pow(10.0f, ((FLOAT32)index) / 28.0f);
  for (i = 0; i < fac_len; i++)
    right_fac_spec[i] /= fac_gain;

  for (i = 0; i < fac_len; i += 8)
  {
    impeghe_find_nearest_neighbor(&right_fac_spec[i], &param[i + 1]);
  }

  impeghe_write_bits2buf(index, 7, fac_bits_word);
  num_enc_bits += 7;
  num_enc_bits += impeghe_fd_encode_fac(&param[1], &fac_bits_word[7], fac_len);
  impeghe_decode_fd_fac(&param[0], len_subfrm, fac_len, ptr_lpc_coeffs, NULL, fac_dec,
                        pstr_scratch);
  *num_fac_bits = num_enc_bits;

  for (i = 0; i < fac_len; i++)
  {
    synth[start_right + i] += fac_dec[i];
  }
  return;
}

/**
 *  impeghe_fd_fac
 *
 *  \brief FD FAC interface function
 *
 *  \param [in ] sfb_offsets          Pointer to SFB offsets table.
 *  \param [in ] sfb_active           SFB active flag.
 *  \param [in ] orig_sig_dbl         Pointer to original signal data.
 *  \param [in ] window_sequence      Window sequence value.
 *  \param [in ] synth_time           Pointer to time domain synthesis buffer.
 *  \param [in ] pstr_acelp           Pointer to ACELP data structure.
 *  \param [in ] last_subfr_was_acelp Flag indicating if the last subframe was ACELP.
 *  \param [in ] next_frm_lpd         Flag indicating if next frame is LPD.
 *  \param [out] fac_prm_out          Pointer to FAC parameters output buffer
 *  \param [in ] num_fac_bits         Pointer to number of FAC bits.
 *  \param [out] pstr_scratch         Pointer to scratch buffer.
 *
 *  \return IA_ERRORCODE              Error code
 */
IA_ERRORCODE impeghe_fd_fac(WORD32 *sfb_offsets, WORD32 sfb_active, FLOAT64 *orig_sig_dbl,
                            WORD32 window_sequence, FLOAT64 *synth_time,
                            ia_usac_td_encoder_struct *pstr_acelp, WORD32 last_subfr_was_acelp,
                            WORD32 next_frm_lpd, WORD16 *fac_prm_out, WORD32 *num_fac_bits,
                            impeghe_scratch_mem *pstr_scratch)
{
  const FLOAT32 *sin_window = NULL;
  LOOPIDX i;
  FLOAT32 *zir_sig = NULL;
  FLOAT32 *lpc_coeffs_q = NULL;
  WORD32 index;
  WORD32 low_pass_line;
  WORD32 fac_len;
  FLOAT64 *left_fac_time_data = pstr_scratch->p_left_fac_time_data;
  FLOAT32 *left_fac_timedata_flt = pstr_scratch->p_left_fac_timedata_flt;
  FLOAT32 *left_fac_spec = pstr_scratch->p_left_fac_spec;
  FLOAT64 *fac_win = pstr_scratch->p_fac_win;
  WORD32 *fac_prm = pstr_scratch->p_fac_prm;
  WORD16 *fac_bits_word = pstr_scratch->p_fac_bits_word;
  FLOAT32 *acelp_folded = pstr_scratch->p_acelp_folded_scratch;

  *num_fac_bits = 0;

  if (window_sequence == EIGHT_SHORT_SEQUENCE)
    fac_len = (pstr_acelp->len_frame / 16);
  else
    fac_len = (pstr_acelp->len_frame / 8);

  low_pass_line = (WORD32)((FLOAT32)sfb_offsets[sfb_active] * (FLOAT32)fac_len /
                           (FLOAT32)pstr_acelp->len_frame);
  if (last_subfr_was_acelp)
  {
    FLOAT32 *tmp_lp_res = pstr_scratch->ptr_tmp_lp_res;
    FLOAT32 lpc_coeffs[ORDER + 1];
    FLOAT32 ener, fac_gain;
    WORD32 leftStart;

    switch (fac_len)
    {
    case 64:
      sin_window = impeghe_sin_window_128;
      break;
    case 128:
      sin_window = impeghe_sin_window_256;
      break;
    default:
      return IMPEGHE_EXE_FATAL_INVALID_FAC_LEN;
    }

    for (i = 0; i < fac_len; i++)
    {
      fac_win[i] = sin_window[i] * sin_window[(2 * fac_len) - 1 - i];
      fac_win[fac_len + i] = 1.0f - (sin_window[fac_len + i] * sin_window[fac_len + i]);
    }

    leftStart = (pstr_acelp->len_frame / 2) - fac_len - ORDER;

    for (i = 0; i < 2 * fac_len + ORDER; i++)
    {
      left_fac_time_data[i] = orig_sig_dbl[leftStart + i];
    }

    for (i = 0; i < fac_len; i++)
    {
      left_fac_time_data[fac_len + ORDER + i] =
          left_fac_time_data[fac_len + ORDER + i] - synth_time[leftStart + fac_len + ORDER + i];
    }

    zir_sig = pstr_acelp->lpd_state.tcx_quant;

    for (i = 0; i < ORDER; i++)
    {
      left_fac_time_data[fac_len + i] =
          left_fac_time_data[fac_len + i] - zir_sig[1 + 128 - ORDER + i];
    }

    for (i = 0; i < fac_len; i++)
    {
      acelp_folded[i] = zir_sig[1 + 128 + i] * (FLOAT32)fac_win[fac_len + i] +
                        zir_sig[1 + 128 - 1 - i] * (FLOAT32)fac_win[fac_len - 1 - i];
    }

    {
      FLOAT32 ener_tmp;
      ener = 0.0f;
      ener_tmp = 0.0f;

      for (i = 0; i < fac_len; i++)
      {
        ener += (FLOAT32)(left_fac_time_data[i + ORDER + fac_len] *
                          left_fac_time_data[i + ORDER + fac_len]);
      }
      ener *= 2.0f;

      for (i = 0; i < fac_len; i++)
      {
        ener_tmp += acelp_folded[i] * acelp_folded[i];
      }

      if (ener_tmp > ener)
        fac_gain = (FLOAT32)sqrt(ener / ener_tmp);
      else
        fac_gain = 1.0f;

      for (i = 0; i < fac_len; i++)
      {
        left_fac_time_data[i + ORDER + fac_len] -= fac_gain * acelp_folded[i];
      }
    }

    for (i = 0; i < 2 * fac_len + ORDER; i++)
    {
      left_fac_timedata_flt[i] = (FLOAT32)left_fac_time_data[i];
    }

    lpc_coeffs_q = pstr_acelp->lpd_state.lpc_coeffs_quant;
    lpc_coeffs_q += ORDER + 1;
    impeghe_get_weighted_lpc(lpc_coeffs_q, lpc_coeffs);
    impeghe_compute_lp_residual(lpc_coeffs, left_fac_timedata_flt + ORDER + fac_len, tmp_lp_res,
                                fac_len);
    FLOAT32 coeff = (2.0f / (FLOAT32)fac_len);
    for (i = 0; i < fac_len; i++)
    {
      tmp_lp_res[i] = tmp_lp_res[i] * coeff;
    }

    impeghe_tcx_mdct(tmp_lp_res, left_fac_spec, fac_len, pstr_scratch);
    memset(&left_fac_spec[low_pass_line], 0, (fac_len - low_pass_line) * sizeof(FLOAT32));

    fac_gain = impeghe_calc_sq_gain(left_fac_spec, 240, fac_len, pstr_scratch->p_sq_gain_en);

    index = (WORD32)floor(0.5f + (28.0f * (FLOAT32)log10(fac_gain)));
    if (index < 0)
      index = 0;
    if (index > 127)
      index = 127;
    impeghe_write_bits2buf(index, 7, fac_bits_word);
    *num_fac_bits += 7;
    fac_gain = (FLOAT32)pow(10.0f, ((FLOAT32)index) / 28.0f);

    for (i = 0; i < fac_len; i++)
    {
      left_fac_spec[i] /= fac_gain;
    }

    for (i = 0; i < fac_len; i += 8)
    {
      impeghe_find_nearest_neighbor(&left_fac_spec[i], &fac_prm[i]);
    }

    *num_fac_bits += impeghe_fd_encode_fac(fac_prm, &fac_bits_word[7], fac_len);

    for (i = 0; i < (*num_fac_bits + 7) / 8; i++)
    {
      fac_prm_out[i] =
          (WORD16)((fac_bits_word[8 * i + 0] & 0x1) << 7 | (fac_bits_word[8 * i + 1] & 0x1) << 6 |
                   (fac_bits_word[8 * i + 2] & 0x1) << 5 | (fac_bits_word[8 * i + 3] & 0x1) << 4 |
                   (fac_bits_word[8 * i + 4] & 0x1) << 3 | (fac_bits_word[8 * i + 5] & 0x1) << 2 |
                   (fac_bits_word[8 * i + 6] & 0x1) << 1 | (fac_bits_word[8 * i + 7] & 0x1) << 0);
    }
  }
  else
  {
    *num_fac_bits = 0;
  }

  if (next_frm_lpd)
  {
    for (i = 0; i < 1024 / 2 + 1 + ORDER; i++)
    {
      pstr_acelp->fd_synth[i] = (FLOAT32)synth_time[pstr_acelp->len_frame - 1 + i - ORDER];
      pstr_acelp->fd_orig[i] = (FLOAT32)orig_sig_dbl[pstr_acelp->len_frame + i - ORDER];
    }

    pstr_acelp->low_pass_line = low_pass_line;
  }

  return IA_NO_ERROR;
}
/** @} */ /* End of CoreEncProc */
