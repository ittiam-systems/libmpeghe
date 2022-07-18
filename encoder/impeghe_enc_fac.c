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
#include "impeghe_write_bitstream.h"
#include "impeghe_lpd.h"
#include "impeghe_avq_enc.h"
#include "impeghe_lpd_rom.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_unary_code
 *
 *  \brief Utility function to implement unary code encoding.
 *
 *  \param [in] idx         Number of 1s to be inserted.
 *  \param [in] ptr_bit_buf Pointer to bit buffer handle.
 *
 *  \return WORD32 Number of bits value.
 */
static WORD32 impeghe_unary_code(WORD32 idx, WORD16 *ptr_bit_buf)
{
  WORD32 num_bits;

  num_bits = 1;

  idx -= 1;
  while (idx-- > 0)
  {
    *ptr_bit_buf++ = 1;
    num_bits++;
  }

  *ptr_bit_buf = 0;

  return (num_bits);
}

/**
 *  impeghe_get_nk_mode
 *
 *  \brief Utility function to get corresponding codebook numbers nk.
 *
 *  \param [in ] mode_lpc      LPC mode value.
 *  \param [out] it_bit_buff   Pointer to bit buffer handle.
 *  \param [out] nk_mode       Pointer to lpc data set mode.
 *  \param [in ] lpc_set       LPC set value.
 *  \param [in ] full_band_lpd Fullband LPD flag.
 *
 *  \return VOID
 */
static VOID impeghe_get_nk_mode(WORD32 mode_lpc, ia_bit_buf_struct *it_bit_buff, WORD32 *nk_mode,
                                WORD32 lpc_set, WORD32 full_band_lpd)
{
  switch (lpc_set)
  {
  case 4:
    break;
  case 0:
  case 2:
    if (full_band_lpd)
    {
      if (mode_lpc)
      {
        *nk_mode = 3;
      }
      else
      {
        *nk_mode = 0;
      }
    }
    else
    {
      *nk_mode = 3;
    }
    impeghe_write_bits_buf(it_bit_buff, mode_lpc, 1);
    break;
  case 1:
    *nk_mode = mode_lpc;
    if (mode_lpc == 2)
    {
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
      if (full_band_lpd)
      {
        *nk_mode = 3;
      }
    }
    else if (mode_lpc == 1)
    {
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
    }
    else if (mode_lpc == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    break;
  case 3:
    if (mode_lpc == 0)
    {
      *nk_mode = 0;
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    else if (mode_lpc == 1)
    {
      *nk_mode = 1;
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    else if (mode_lpc == 2)
    {
      *nk_mode = 2;
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    else
    {
      *nk_mode = 2;
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
    }
    break;
  }
  return;
}

/**
 *  impeghe_write_qn_data
 *
 *  \brief Helper function for creating binary code with the corresponding codebook numbers nk.
 *
 *  \param [out] qn          Pointer to binary code data.
 *  \param [in ] it_bit_buff Pointer to bit buffer handle.
 *  \param [in ] nk_mode     nk mode value.
 *  \param [in ] num_frames  Number of frames value.
 *
 *  \return VOID
 */
static VOID impeghe_write_qn_data(WORD32 *qn, ia_bit_buf_struct *it_bit_buff, WORD32 nk_mode,
                                  WORD32 num_frames)
{
  LOOPIDX k, i;
  switch (nk_mode)
  {
  case 1:
    for (k = 0; k < 2; k++)
    {
      for (i = 0; i < qn[k] - 1; i++)
      {
        impeghe_write_bits_buf(it_bit_buff, 1, 1);
      }
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
    }
    break;
  case 0:
  case 2:
  case 3:
    for (k = 0; k < 2; k++)
    {
      WORD32 qn1 = qn[k] - 2;
      if (qn1 < 0 || qn1 > 3)
      {
        qn1 = 3;
      }
      impeghe_write_bits_buf(it_bit_buff, qn1, 2);
    }
    if ((nk_mode == 2) && num_frames != 2) // Change as per MPEG-H LC decoder implementation
    {
      for (k = 0; k < 2; k++)
      {
        if (qn[k] > 4)
        {
          for (i = 0; i < qn[k] - 4; i++)
          {
            impeghe_write_bits_buf(it_bit_buff, 1, 1);
          }
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
        }
        if (qn[k] == 0) // check not found in dec
        {
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
        }
      }
    }
    else
    {
      for (k = 0; k < 2; k++)
      {
        if (qn[k] == 5)
        {
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
        }
        else if (qn[k] == 6)
        {
          impeghe_write_bits_buf(it_bit_buff, 1, 1);
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
        }
        else if (qn[k] == 0)
        {
          impeghe_write_bits_buf(it_bit_buff, 1, 1);
          impeghe_write_bits_buf(it_bit_buff, 1, 1);
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
        }
        else
        {
          WORD32 i, qn_ext = qn[k] - 4;
          if (qn_ext > 0)
          {
            for (i = 0; i < qn_ext; i++)
            {
              impeghe_write_bits_buf(it_bit_buff, 1, 1);
            }
            impeghe_write_bits_buf(it_bit_buff, 0, 1);
          }
        }
      }
    }
    break;
  }
  return;
}

/**
 *  impeghe_write_cb_indices
 *
 *  \brief Utilit function to write codebook indices value.
 *
 *  \param [in ] qn          Pointer to binary code data.
 *  \param [in ] ptr_params  Pointer to ACELP params.
 *  \param [out] idx         Pointer to index value buffers.
 *  \param [out] it_bit_buff Pointer to bit buffer handle.
 *  \param [in ] nk_mode     nk mode value
 *  \param [in ] num_frames  Number of frames
 *
 *  \return VOID
 */
static VOID impeghe_write_cb_indices(WORD32 *qn, WORD32 *ptr_params, WORD32 *idx,
                                     ia_bit_buf_struct *it_bit_buff, WORD32 nk_mode,
                                     WORD32 num_frames)
{
  LOOPIDX k;
  WORD32 j = *idx;

  impeghe_write_qn_data(qn, it_bit_buff, nk_mode, num_frames);

  for (k = 0; k < 2; k++)
  {
    if (qn[k] > 0)
    {
      WORD32 n, nk, i;
      if (qn[k] > 4)
      {
        nk = (qn[k] - 3) >> 1;
        n = qn[k] - nk * 2;
      }
      else
      {
        nk = 0;
        n = qn[k];
      }

      impeghe_write_bits_buf(it_bit_buff, ptr_params[j++], 4 * n);

      for (i = 0; i < 8; i++)
      {
        impeghe_write_bits_buf(it_bit_buff, ptr_params[j++], nk);
      }
    }
  }

  *idx = j;

  return;
}

/**
 *  impeghe_write_lpc_data
 *
 *  \brief Utility function to write LPC data.
 *
 *  \param [in] it_bit_buff    Pointer to bit buffer handle.
 *  \param [in] ptr_param_lpc      Pointer to LPC parameters buffer.
 *  \param [in] first_lpd_flag First LPD flag value.
 *  \param [in] ptr_mod            Pointer to LPC mode value.
 *  \param [in] num_frames     Number of frames value
 *  \param [in] full_band_lpd  Fullband LPD flag value.
 *
 *  \return VOID
 */
static VOID impeghe_write_lpc_data(ia_bit_buf_struct *it_bit_buff, WORD32 *ptr_param_lpc,
                                   WORD32 first_lpd_flag, WORD32 *ptr_mod, WORD32 num_frames,
                                   WORD32 full_band_lpd)
{
  WORD32 nk_mode = 0;
  WORD32 j = 0;
  LOOPIDX k;
  WORD32 mode_lpc = 0;
  WORD32 qn[2] = {0};

  impeghe_get_nk_mode(mode_lpc, it_bit_buff, &nk_mode, 4, full_band_lpd);

  impeghe_write_bits_buf(it_bit_buff, ptr_param_lpc[j++], 8);

  for (k = 0; k < 2; k++)
  {
    qn[k] = ptr_param_lpc[j++];
  }

  impeghe_write_cb_indices(qn, ptr_param_lpc, &j, it_bit_buff, nk_mode, num_frames);

  if (first_lpd_flag)
  {
    mode_lpc = ptr_param_lpc[j++];
    impeghe_get_nk_mode(mode_lpc, it_bit_buff, &nk_mode, 0, full_band_lpd);

    if (mode_lpc == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_param_lpc[j++], 8);
    }

    for (k = 0; k < 2; k++)
    {
      qn[k] = ptr_param_lpc[j++];
    }

    impeghe_write_cb_indices(qn, ptr_param_lpc, &j, it_bit_buff, nk_mode, num_frames);
  }

  if (!full_band_lpd)
  {
    mode_lpc = ptr_param_lpc[j++];
  }
  if (num_frames == 4 && ptr_mod[0] < 3)
  {
    impeghe_get_nk_mode(mode_lpc, it_bit_buff, &nk_mode, 2, full_band_lpd);

    if (mode_lpc == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_param_lpc[j++], 8);
    }

    for (k = 0; k < 2; k++)
    {
      qn[k] = ptr_param_lpc[j++];
    }

    impeghe_write_cb_indices(qn, ptr_param_lpc, &j, it_bit_buff, nk_mode, num_frames);
  }
  mode_lpc = ptr_param_lpc[j++];
  if (ptr_mod[0] < 2)
  {
    impeghe_get_nk_mode(mode_lpc, it_bit_buff, &nk_mode, 1, full_band_lpd);

    if (mode_lpc != 1)
    {
      if (mode_lpc == 0)
      {
        impeghe_write_bits_buf(it_bit_buff, ptr_param_lpc[j++], 8);
      }

      for (k = 0; k < 2; k++)
      {
        qn[k] = ptr_param_lpc[j++];
      }

      impeghe_write_cb_indices(qn, ptr_param_lpc, &j, it_bit_buff, nk_mode, num_frames);
    }
  }
  else if (mode_lpc != 1) // to be checked
  {
    if (mode_lpc == 0)
    {
      j++;
    }
    for (k = 0; k < 2; k++)
    {
      qn[k] = ptr_param_lpc[j++];
    }
    j += ((qn[0] > 0) ? 9 : 0) + ((qn[1] > 0) ? 9 : 0);
  }

  mode_lpc = ptr_param_lpc[j++];
  if (num_frames != 2 && ptr_mod[2] < 2)
  {
    impeghe_get_nk_mode(mode_lpc, it_bit_buff, &nk_mode, 3, full_band_lpd);
    if (mode_lpc == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, ptr_param_lpc[j++], 8);
    }

    for (k = 0; k < 2; k++)
    {
      qn[k] = ptr_param_lpc[j++];
    }
    impeghe_write_cb_indices(qn, ptr_param_lpc, &j, it_bit_buff, nk_mode, num_frames);
  }
  return;
}
/**
 *  impeghe_encode_fac_params
 *
 *  \brief Function to encode FAC params.
 *
 *  \param [in ] ptr_mod                    Pointer to LPC mode value buffer.
 *  \param [in ] ptr_param_tcx            Pointer to TCX parameters buffer.
 *  \param [in ] pstr_usac_data              Pointer to USAC data structure.
 *  \param [in ] pstr_usac_config            Pointer to configuration structure.
 *  \param [in ] usac_independency_flag USAC independency flag.
 *  \param [out] it_bit_buff            Pointer to bit buffer handle.
 *  \param [in ] ch_idx                 Channel index value.
 *  \param [in ] ele_id                 Element ID value.
 *
 *  \return VOID
 */
VOID impeghe_encode_fac_params(WORD32 *ptr_mod, WORD32 *ptr_param_tcx,
                               ia_usac_data_struct *pstr_usac_data,
                               ia_usac_encoder_config_struct *pstr_usac_config,
                               WORD32 const usac_independency_flag,
                               ia_bit_buf_struct *it_bit_buff, WORD32 ch_idx, WORD32 ele_id)
{
  WORD32 *total_nbbits = &pstr_usac_data->total_nbbits[ch_idx];
  ia_usac_td_encoder_struct *pstr_td = pstr_usac_data->td_encoder[ch_idx];
  WORD32 codec_mode = pstr_td->acelp_core_mode;
  WORD16 *bit_buf = pstr_usac_data->td_serial_out[ch_idx];
  WORD32 is_bass_post_filter = 1;
  WORD32 first_lpd_flag = (pstr_usac_data->core_mode_prev[ch_idx] == CORE_MODE_FD);
  WORD32 *param_lpc = pstr_usac_data->param_buf + (NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV);
  WORD32 *param_lpc_flpd =
      pstr_usac_data->param_buf_flpd + (NUM_FRAMES * MAX_NUM_TCX_PRM_PER_DIV);
  WORD32 *param = pstr_usac_data->param_buf;
  ia_tns_info *pstr_tns_info = pstr_usac_data->pstr_tns_info[0];
  ia_igf_data_struct *pstr_igf_data = &pstr_usac_data->str_igf_data[ch_idx];
  ia_igf_config_struct *pstr_igf_config = &pstr_usac_data->str_igf_config[ele_id];
  WORD32 j, k, n, sfr, lpd_mode, num_bits, sqBits, *prm;
  WORD16 first_tcx_flag = 1;
  WORD32 nbits_fac, nb_bits_lpc;
  WORD32 core_mode_last = (first_lpd_flag) ? 0 : 1;
  WORD32 fac_data_present;
  WORD32 num_frames = NUM_FRAMES;
  WORD32 full_band_lpd = pstr_usac_config->full_band_lpd;
  WORD32 enhanced_noise_filling = pstr_usac_data->str_igf_config[ele_id].igf_active;
  WORD32 tns_data_present = 0;
  WORD16 *ptr_bit_buf = bit_buf;

  if (pstr_usac_config->tns_select == 1)
  {
    tns_data_present = pstr_tns_info->tns_data_present;
  }

  ptr_bit_buf = bit_buf;
  pstr_td->num_bits_per_supfrm = 0;
  *total_nbbits = 0;

  if (pstr_usac_config->full_band_lpd == 1)
  {
    if (pstr_usac_data->pstr_tns_info[0] != NULL)
    {
      impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->pstr_tns_info[0]->tns_data_present, 1);
      *total_nbbits = 1;
    }
    else
    {
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
      *total_nbbits = 1;
    }
    if ((pstr_usac_data->pstr_tns_info[0] != NULL &&
         pstr_usac_data->pstr_tns_info[0]->tns_data_present == 1) ||
        pstr_usac_data->noise_filling[ele_id] == 1)
    {
      impeghe_write_bits_buf(it_bit_buff, WIN_SEL_0, 1);
      impeghe_write_bits_buf(it_bit_buff, pstr_td->max_sfb_short, 4);
      *total_nbbits += 5;
    }
  }

  impeghe_write_bits_buf(it_bit_buff, pstr_td->acelp_core_mode, 3);

  if (pstr_usac_config->full_band_lpd)
  {
    if (ptr_mod[0] == 2)
    {
      lpd_mode = 4;
    }
    else
    {
      lpd_mode = (ptr_mod[1] << 1) + ptr_mod[0];
    }
    param = pstr_usac_data->param_buf_flpd;
  }
  else
  {
    if (ptr_mod[0] == 3)
    {
      lpd_mode = 25;
    }
    else if ((ptr_mod[0] == 2) && (ptr_mod[2] == 2))
    {
      lpd_mode = 24;
    }
    else
    {
      if (ptr_mod[0] == 2)
      {
        lpd_mode = 16 + ptr_mod[2] + 2 * ptr_mod[3];
      }
      else if (ptr_mod[2] == 2)
      {
        lpd_mode = 20 + ptr_mod[0] + 2 * ptr_mod[1];
      }
      else
      {
        lpd_mode = ptr_mod[0] + 2 * ptr_mod[1] + 4 * ptr_mod[2] + 8 * ptr_mod[3];
      }
    }
  }

  if (full_band_lpd == 1)
  {
    num_frames >>= 1;
    impeghe_write_bits_buf(it_bit_buff, lpd_mode, 3);
    pstr_td->num_bits_per_supfrm = 3;
    *total_nbbits += 3;
  }
  else
  {
    impeghe_write_bits_buf(it_bit_buff, lpd_mode, 5);
    pstr_td->num_bits_per_supfrm = 5;
    *total_nbbits += 5;

    impeghe_write_bits_buf(it_bit_buff, is_bass_post_filter, 1);
    *total_nbbits += 1;
  }

  impeghe_write_bits_buf(it_bit_buff, core_mode_last, 1);
  *total_nbbits += 1;

  if (((ptr_mod[0] == 0) && (ptr_mod[-1] != 0)) || ((ptr_mod[0] > 0) && (ptr_mod[-1] == 0)))
  {
    fac_data_present = 1;
  }
  else
  {
    fac_data_present = 0;
  }

  impeghe_write_bits_buf(it_bit_buff, fac_data_present, 1);
  *total_nbbits += 1;

  num_bits = (impeghe_acelp_core_numbits_1024[codec_mode] / 4) - 2;

  k = 0;
  while (k < num_frames)
  {
    lpd_mode = ptr_mod[k];
    prm = param + (k * MAX_NUM_TCX_PRM_PER_DIV);
    j = 0;

    if (((ptr_mod[k - 1] == 0) && (ptr_mod[k] > 0)) ||
        ((ptr_mod[k - 1] > 0) && (ptr_mod[k] == 0)))
    {
      nbits_fac = impeghe_fd_encode_fac(&prm[j], ptr_bit_buf, (pstr_td->len_subfrm) / 2);
      j += (pstr_td->len_subfrm) / 2;
      *total_nbbits += nbits_fac;
      for (WORD32 i = 0; i < nbits_fac; i++)
      {
        impeghe_write_bits_buf(it_bit_buff, ptr_bit_buf[i], 1);
      }
    }

    if (lpd_mode == 0)
    {
      impeghe_write_bits_buf(it_bit_buff, prm[j++], 2);

      for (sfr = 0; sfr < (pstr_td->num_subfrm); sfr++)
      {
        n = 6;
        if ((sfr == 0) || (((pstr_td->len_subfrm) == 256) && (sfr == 2)))
          n = 9;
        impeghe_write_bits_buf(it_bit_buff, prm[j], n);
        j++;
        impeghe_write_bits_buf(it_bit_buff, prm[j], 1);
        j++;
        if (codec_mode == ACELP_CORE_MODE_8k0)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 1);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 1);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_8k8)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 1);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_9k6)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_11k2)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 5);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_12k8)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_14k4)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 9);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_16k)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 13);
          j++;
        }
        else if (codec_mode == ACELP_CORE_MODE_18k4)
        {
          impeghe_write_bits_buf(it_bit_buff, prm[j], 2);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 2);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 2);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 2);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 14);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 14);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 14);
          j++;
          impeghe_write_bits_buf(it_bit_buff, prm[j], 14);
          j++;
        }
        impeghe_write_bits_buf(it_bit_buff, prm[j], 7);
        j++;
      }
      if (pstr_usac_config->full_band_lpd == 1)
      {
        impeghe_write_tbe_data(&pstr_usac_data->str_tbe_data, it_bit_buff, 0 /*To be adjusted*/);
      }
      *total_nbbits += (num_bits - NBITS_LPC);
      pstr_td->num_bits_per_supfrm += (num_bits - NBITS_LPC);
      k++;
    }
    else
    {
      if (!pstr_usac_data->noise_filling[ele_id])
      {
        j++;
      }
      else
      {
        impeghe_write_bits_buf(it_bit_buff, prm[j++], 3);
        *total_nbbits += 3;
        pstr_td->num_bits_per_supfrm += 3;
      }
      impeghe_write_bits_buf(it_bit_buff, prm[j++], 7);
      *total_nbbits += 7;
      pstr_td->num_bits_per_supfrm += 7;

      if ((lpd_mode == 3) || (lpd_mode == 2 && pstr_usac_config->full_band_lpd == 1))
      {
        if (pstr_usac_config->ltpf_enable && pstr_usac_data->ltpf_data[ch_idx].ltpf_active)
        {
          impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->ltpf_data[ch_idx].ltpf_active, 1);
          impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->ltpf_data[ch_idx].ltpf_pitch_index,
                                 9);
          impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->ltpf_data[ch_idx].ltpf_gain_index,
                                 2);
          *total_nbbits += 12;
          pstr_td->num_bits_per_supfrm += 12;
        }
        else
        {
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
          *total_nbbits += 1;
          pstr_td->num_bits_per_supfrm += 1;
        }
        /* FDP */
        if (!usac_independency_flag)
        {
          impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->fdp_data.fdp_active[ch_idx], 1);

          if (pstr_usac_data->fdp_data.fdp_active[ch_idx])
            impeghe_write_bits_buf(it_bit_buff, pstr_usac_data->fdp_data.fdp_spacing_idx[ch_idx],
                                   8);

          *total_nbbits += 9;
          pstr_td->num_bits_per_supfrm += 9;
        }
      }

      if (enhanced_noise_filling == 1)
      {
        WORD32 num_windows = 1;
        WORD32 bits_written = 0;
        pstr_igf_config->m_igf_start_sfb = pstr_igf_config->igf_start_sfb_sb;
        pstr_igf_config->m_igf_stop_sfb = pstr_igf_config->igf_stop_sfb_sb;

        bits_written = impeghe_write_igf_levels(it_bit_buff, pstr_igf_data, pstr_igf_config,
                                                usac_independency_flag, 1, num_windows);

        *total_nbbits += bits_written;

        pstr_td->num_bits_per_supfrm += bits_written;

        if (pstr_igf_data->igf_all_zero == 0)
        {
          bits_written = impeghe_write_igf_data(it_bit_buff, pstr_igf_data, pstr_igf_config,
                                                usac_independency_flag, 1);
          *total_nbbits += bits_written;
          pstr_td->num_bits_per_supfrm += bits_written;
        }
      }

      if (tns_data_present == 1)
      {
        WORD32 bits_written = 0;
        bits_written =
            impeghe_write_tns_data(it_bit_buff, pstr_tns_info, EIGHT_SHORT_SEQUENCE, 1);
        *total_nbbits += bits_written;
        pstr_td->num_bits_per_supfrm += bits_written;
      }
      if (first_tcx_flag)
      {
        first_tcx_flag = 0;
        if (usac_independency_flag)
        {
          pstr_td->arith_reset_flag = 1;
          memset(pstr_td->c_pres, 0, 516 * sizeof(WORD32));
          memset(pstr_td->c_prev, 0, 516 * sizeof(WORD32));
        }
        else
        {
          if (pstr_td->arith_reset_flag)
          {
            memset(pstr_td->c_pres, 0, 516 * sizeof(WORD32));
            memset(pstr_td->c_prev, 0, 516 * sizeof(WORD32));
          }
          impeghe_write_bits_buf(it_bit_buff, pstr_td->arith_reset_flag, 1);
          *total_nbbits += 1;
          pstr_td->num_bits_per_supfrm += 1;
        }
      }

      sqBits =
          impeghe_tcx_coding(it_bit_buff, ptr_param_tcx[k], pstr_td->len_frame, prm + j,
                             pstr_td->c_pres, pstr_td->c_prev, &(pstr_usac_data->str_scratch));

      *total_nbbits += sqBits;
      pstr_td->num_bits_per_supfrm += sqBits;

      if (pstr_usac_config->full_band_lpd)
      {
        k += (lpd_mode);
      }
      else
      {
        k += (1 << (lpd_mode - 1));
      }
    }
  }

  nb_bits_lpc = *total_nbbits;
  nb_bits_lpc = it_bit_buff->cnt_bits;

  if (pstr_usac_config->full_band_lpd)
  {
    impeghe_write_lpc_data(it_bit_buff, param_lpc_flpd, first_lpd_flag, ptr_mod, num_frames,
                           pstr_usac_config->full_band_lpd);
  }
  else
  {
    impeghe_write_lpc_data(it_bit_buff, param_lpc, first_lpd_flag, ptr_mod, num_frames,
                           pstr_usac_config->full_band_lpd);
  }

  nb_bits_lpc = it_bit_buff->cnt_bits - nb_bits_lpc;
  *total_nbbits += nb_bits_lpc;
  pstr_td->num_bits_per_supfrm += nb_bits_lpc;

  if ((core_mode_last == 0) && (fac_data_present == 1))
  {
    WORD32 short_fac_flag;
    if (pstr_usac_config->window_sequence_prev[ch_idx] == EIGHT_SHORT_SEQUENCE)
    {
      short_fac_flag = 1;
    }
    else
    {
      short_fac_flag = 0;
    }
    impeghe_write_bits_buf(it_bit_buff, short_fac_flag, 1);
    *total_nbbits += 1;
  }

  return;
}

/**
 *  impeghe_fd_encode_fac
 *
 *  \brief Utility function to encode FAC params
 *
 *  \param [in] ptr_prm         Pointer to params buffer.
 *  \param [in] ptr_bit_buf Pointer to bit buffer handle.
 *  \param [in] fac_length  FAC params length.
 *
 *  \return WORD32 error code if any.
 */
WORD32 impeghe_fd_encode_fac(WORD32 *ptr_prm, WORD16 *ptr_bit_buf, WORD32 fac_length)
{
  LOOPIDX i;
  WORD32 j, n, nb, qn, kv[8], nk, fac_bits;
  WORD32 I;

  fac_bits = 0;

  for (i = 0; i < fac_length; i += 8)
  {
    impeghe_apply_voronoi_ext(&ptr_prm[i], &qn, &I, kv);

    nb = impeghe_unary_code(qn, ptr_bit_buf);
    ptr_bit_buf += nb;

    fac_bits += nb;

    nk = 0;
    n = qn;
    if (qn > 4)
    {
      nk = (qn - 3) >> 1;
      n = qn - nk * 2;
    }

    impeghe_write_bits2buf(I, 4 * n, ptr_bit_buf);
    ptr_bit_buf += 4 * n;
    for (j = 0; j < 8; j++)
    {
      impeghe_write_bits2buf(kv[j], nk, ptr_bit_buf);
      ptr_bit_buf += nk;
    }

    fac_bits += 4 * qn;
  }

  return fac_bits;
}

/** @} */ /* End of CoreEncProc */
