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
#include "impeghe_avq_enc.h"
#include "impeghe_lpd.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_lsf_weight
 *
 *  \brief Computes LSF weights values.
 *
 *  \param [in ] lsf   Pointer to LSF data.
 *  \param [out] ptr_w Pointer to computed weights data.
 *
 *  \return VOID
 */
static VOID impeghe_lsf_weight(FLOAT32 *lsf, FLOAT32 *ptr_w)
{
  LOOPIDX i;
  FLOAT32 d[ORDER + 1];

  d[ORDER] = FREQ_MAX - lsf[ORDER - 1];
  d[0] = lsf[0];

  for (i = 1; i < ORDER; i++)
  {
    d[i] = lsf[i] - lsf[i - 1];
  }

  for (i = 0; i < ORDER; i++)
  {
    ptr_w[i] = (1.0f / d[i]) + (1.0f / d[i + 1]);
  }

  return;
}

/**
 *  impeghe_lsf_weight_2st_flt
 *
 *  \brief Computes LSF weights values
 *
 *  \param [in] lsfq Pointer to LSF quantization data.
 *  \param [in] w    Pointer to weights buffer.
 *  \param [in] mode Mode value.
 *
 *  \return VOID
 */
static VOID impeghe_lsf_weight_2st_flt(FLOAT32 *lsfq, FLOAT32 *w, WORD32 mode)
{
  LOOPIDX i;
  FLOAT32 d[ORDER + 1];
  d[ORDER] = FREQ_MAX - lsfq[ORDER - 1];
  d[0] = lsfq[0];
  for (i = 1; i < ORDER; i++)
  {
    d[i] = lsfq[i] - lsfq[i - 1];
  }

  for (i = 0; i < ORDER; i++)
  {
    w[i] = (FLOAT32)(impeghe_wlsf_factor_table[mode] / (FREQ_DIV / sqrt(d[i] * d[i + 1])));
  }
}

/**
 *  impeghe_avq_first_approx_abs
 *
 *  \brief First stage approximation - absolute in AVQ process.
 *
 *  \param [in] lsf  Pointer to LSF data.
 *  \param [in] lsfq Pointer to quantized LSF data.
 *
 *  \return Index value
 */
static WORD32 impeghe_avq_first_approx_abs(FLOAT32 *lsf, FLOAT32 *lsfq)
{
  LOOPIDX i, j;
  WORD32 index;
  FLOAT32 dist_min, dist, temp;
  const FLOAT32 *p_dico;
  FLOAT32 w[ORDER];

  index = 0;
  dist_min = 1.0e30f;
  p_dico = impeghe_dico_lsf_abs_8b_flt;

  impeghe_lsf_weight(lsf, w);

  for (i = 0; i < 256; i++)
  {
    dist = 0.0;
    for (j = 0; j < ORDER; j++)
    {
      temp = lsf[j] - *p_dico++;
      dist += w[j] * temp * temp;
    }
    if (dist < dist_min)
    {
      index = i;
      dist_min = dist;
    }
  }

  for (j = 0; j < ORDER; j++)
  {
    lsfq[j] = impeghe_dico_lsf_abs_8b_flt[index * ORDER + j];
  }

  return index;
}

/**
 *  impeghe_avq_first_approx_rel
 *
 *  \brief First stage approximation - relative in AVQ process.
 *
 *  \param [in] ptr_lsf  Pointer to LSF data.
 *  \param [in] ptr_lsfq Pointer to quantized LSF data.
 *  \param [in] idx      Pointer to index value.
 *  \param [in] mode     Mode value.
 *
 *  \return Number bits needed for encoding.
 */
static WORD32 impeghe_avq_first_approx_rel(FLOAT32 *ptr_lsf, FLOAT32 *ptr_lsfq, WORD32 *idx,
                                           WORD32 mode)
{
  LOOPIDX i;
  WORD32 nq, num_bits;
  WORD32 avq[ORDER];
  FLOAT32 lsf_min, temp;
  FLOAT32 w[ORDER], x[ORDER];

  temp = 0.0f;

  impeghe_lsf_weight_2st_flt(ptr_lsf, w, 1);

  for (i = 0; i < ORDER; i++)
  {
    x[i] = (ptr_lsf[i] - ptr_lsfq[i]) / w[i];
    temp += x[i] * x[i];
  }

  if (temp < 8.0f)
  {
    idx[0] = 0;
    idx[1] = 0;

    if (mode == 1)
    {
      return (2);
    }
    else if ((mode == 0) || (mode == 3))
    {
      return (10);
    }
    else
    {
      return (6);
    }
  }

  impeghe_lsf_weight_2st_flt(ptr_lsfq, w, mode);

  for (i = 0; i < ORDER; i++)
  {
    x[i] = (ptr_lsf[i] - ptr_lsfq[i]) / w[i];
  }

  impeghe_alg_vec_quant(x, avq, idx);

  for (i = 0; i < ORDER; i++)
  {
    ptr_lsfq[i] += (w[i] * (FLOAT32)avq[i]);
  }

  num_bits = 0;
  for (i = 0; i < 2; i++)
  {
    nq = idx[i];

    if (mode == 1)
    {
      num_bits += nq * 5;
      if (nq == 0)
      {
        num_bits += 1;
      }
    }
    else if ((mode == 0) || (mode == 3))
    {
      num_bits += (2 + (nq * 4));

      if (nq > 6)
      {
        num_bits += nq - 3;
      }
      else if (nq > 4)
      {
        num_bits += nq - 4;
      }
      else if (nq == 0)
      {
        num_bits += 3;
      }
    }
    else
    {
      num_bits += (2 + (nq * 4));
      if (nq > 4)
      {
        num_bits += nq - 3;
      }
      else if (nq == 0)
      {
        num_bits += 1;
      }
    }
  }

  lsf_min = LSF_GAP;
  for (i = 0; i < ORDER; i++)
  {
    if (ptr_lsfq[i] < lsf_min)
    {
      ptr_lsfq[i] = lsf_min;
    }

    lsf_min = ptr_lsfq[i] + LSF_GAP;
  }

  lsf_min = FREQ_MAX - LSF_GAP;
  for (i = ORDER - 1; i >= 0; i--)
  {
    if (ptr_lsfq[i] > lsf_min)
    {
      ptr_lsfq[i] = lsf_min;
    }

    lsf_min = ptr_lsfq[i] - LSF_GAP;
  }

  return (num_bits);
}

/**
 *  impeghe_quantize_flpd_lpc_avq
 *
 *  \brief Helper function to quantize LPC sets of FLPD mode.
 *
 *  \param [in ] ptr_lsf     Pointer to LSF data.
 *  \param [out] ptr_lsfq    Pointer to quantized LSF data.
 *  \param [in ] lpc0        LPC set 0 flag.
 *  \param [in ] ptr_lpc_idx Pointer to LPC index.
 *  \param [in ] nb_indices  Pointer to number of indices value.
 *  \param [in ] nbbits      Pointer to number of bits value.
 *
 *  \return VOID
 */
VOID impeghe_quantize_flpd_lpc_avq(FLOAT32 *ptr_lsf, FLOAT32 *ptr_lsfq, WORD32 lpc0,
                                   WORD32 *ptr_lpc_idx, WORD32 *nb_indices, WORD32 *nbbits,
                                   pUWORD8 ptr_lpd_scratch)
{
  LOOPIDX i;
  WORD32 num_bits, nbt, nit;
  WORD32 *ptr_index, *indxt;
  FLOAT32 lsfq[ORDER];

  *nb_indices = 0;
  *nbbits = 0;
  indxt = (WORD32 *)ptr_lpd_scratch;
  ptr_index = &ptr_lpc_idx[0];

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[ORDER], &ptr_lsfq[ORDER]);

  nbt = impeghe_avq_first_approx_rel(&ptr_lsf[ORDER], &ptr_lsfq[ORDER], &ptr_index[1], 0);
  nit = 1 + impeghe_get_num_params(&ptr_lpc_idx[1]);

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += 8 + nbt;

  if (lpc0)
  {
    *ptr_index = 0;
    ptr_index++;
    *nb_indices += 1;
    *nbbits += 1;

    ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[-ORDER], &ptr_lsfq[-ORDER]);

    num_bits =
        impeghe_avq_first_approx_rel(&ptr_lsf[-ORDER], &ptr_lsfq[-ORDER], &ptr_index[1], 0);
    nbt = 8 + num_bits;
    nit = 1 + impeghe_get_num_params(&ptr_index[1]);

    for (i = 0; i < ORDER; i++)
    {
      lsfq[i] = ptr_lsfq[ORDER + i];
    }

    num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[-ORDER], &lsfq[0], indxt, 3);

    if (num_bits < nbt)
    {
      nbt = num_bits;
      nit = impeghe_get_num_params(&indxt[0]);
      ptr_index[-1] = 1;
      for (i = 0; i < ORDER; i++)
      {
        ptr_lsfq[-ORDER + i] = lsfq[i];
      }
      for (i = 0; i < nit; i++)
      {
        ptr_index[i] = indxt[i];
      }
    }

    ptr_index += nit;
    *nb_indices += nit;
    *nbbits += nbt;
  }

  *ptr_index = 0;
  ptr_index++;
  *nb_indices += 1;
  *nbbits += 1;

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[0], &ptr_lsfq[0]);

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], &ptr_lsfq[0], &ptr_index[1], 0);
  nbt = 2 + 8 + num_bits;
  nit = 1 + impeghe_get_num_params(&ptr_index[1]);

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = 0.5f * (ptr_lsfq[-ORDER + i] + ptr_lsfq[ORDER + i]);
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], lsfq, indxt, 1);

  if (num_bits < 10)
  {
    nit = 0;
    nbt = 2;
    ptr_index[-1] = 1;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[i] = lsfq[i];
    }
  }

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = ptr_lsfq[ORDER + i];
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], lsfq, indxt, 3);
  num_bits += 1;

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 2;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += nbt;

  return;
}

/**
 *  impeghe_quantize_lpc_avq
 *
 *  \brief Carries out quantization of LPC data sets using AVQ
 *
 *  \param [in] ptr_lsf     Pointer to LSF data.
 *  \param [in] ptr_lsfq    Pointer to quantized LSF data.
 *  \param [in] lpc0        LPC set 0 data flag.
 *  \param [in] ptr_lpc_idx Pointer to LPC index.
 *  \param [in] nb_indices  Pointer to number indices value.
 *  \param [in] nbbits      Pointer to number of bits value
 *
 *  \return VOID
 */
VOID impeghe_quantize_lpc_avq(FLOAT32 *ptr_lsf, FLOAT32 *ptr_lsfq, WORD32 lpc0,
                              WORD32 *ptr_lpc_idx, WORD32 *nb_indices, WORD32 *nbbits,
                              pUWORD8 ptr_lpd_scratch)
{
  LOOPIDX i;
  WORD32 num_bits, nbt, nit;
  WORD32 *ptr_index, *indxt;
  FLOAT32 lsfq[ORDER];

  *nb_indices = 0;
  *nbbits = 0;
  indxt = (WORD32 *)ptr_lpd_scratch;
  ptr_index = &ptr_lpc_idx[0];

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[3 * ORDER], &ptr_lsfq[3 * ORDER]);

  nbt = impeghe_avq_first_approx_rel(&ptr_lsf[3 * ORDER], &ptr_lsfq[3 * ORDER], &ptr_index[1], 0);
  nit = 1 + impeghe_get_num_params(&ptr_lpc_idx[1]);

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += 8 + nbt;

  if (lpc0)
  {
    *nb_indices += 1;
    *nbbits += 1;
    *ptr_index = 0;
    ptr_index++;

    ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[-ORDER], &ptr_lsfq[-ORDER]);

    num_bits =
        impeghe_avq_first_approx_rel(&ptr_lsf[-ORDER], &ptr_lsfq[-ORDER], &ptr_index[1], 0);
    nbt = 8 + num_bits;
    nit = 1 + impeghe_get_num_params(&ptr_index[1]);

    for (i = 0; i < ORDER; i++)
    {
      lsfq[i] = ptr_lsfq[3 * ORDER + i];
    }

    num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[-ORDER], &lsfq[0], indxt, 3);

    if (num_bits < nbt)
    {
      nbt = num_bits;
      nit = impeghe_get_num_params(&indxt[0]);
      ptr_index[-1] = 1;
      for (i = 0; i < ORDER; i++)
      {
        ptr_lsfq[-ORDER + i] = lsfq[i];
      }
      for (i = 0; i < nit; i++)
      {
        ptr_index[i] = indxt[i];
      }
    }

    ptr_index += nit;
    *nb_indices += nit;
    *nbbits += nbt;
  }

  *ptr_index = 0;
  ptr_index++;
  *nb_indices += 1;
  *nbbits += 1;

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[ORDER], &ptr_lsfq[ORDER]);

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[ORDER], &ptr_lsfq[ORDER], &ptr_index[1], 0);
  nbt = 8 + num_bits;
  nit = 1 + impeghe_get_num_params(&ptr_index[1]);

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = ptr_lsfq[3 * ORDER + i];
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[ORDER], &lsfq[0], indxt, 3);

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 1;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[ORDER + i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += nbt;

  *ptr_index = 0;
  ptr_index++;
  *nb_indices += 1;

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[0], &ptr_lsfq[0]);

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], &ptr_lsfq[0], &ptr_index[1], 0);
  nbt = 2 + 8 + num_bits;
  nit = 1 + impeghe_get_num_params(&ptr_index[1]);

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = 0.5f * (ptr_lsfq[-ORDER + i] + ptr_lsfq[ORDER + i]);
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], lsfq, indxt, 1);

  if (num_bits < 10)
  {
    nbt = 2;
    nit = 0;
    ptr_index[-1] = 1;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[i] = lsfq[i];
    }
  }

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = ptr_lsfq[ORDER + i];
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[0], lsfq, indxt, 2);
  num_bits += 1;

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 2;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += nbt;

  *ptr_index = 0;
  ptr_index++;
  *nb_indices += 1;

  ptr_index[0] = impeghe_avq_first_approx_abs(&ptr_lsf[2 * ORDER], &ptr_lsfq[2 * ORDER]);

  num_bits =
      impeghe_avq_first_approx_rel(&ptr_lsf[2 * ORDER], &ptr_lsfq[2 * ORDER], &ptr_index[1], 0);
  nbt = 2 + 8 + num_bits;
  nit = 1 + impeghe_get_num_params(&ptr_index[1]);

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = 0.5f * (ptr_lsfq[ORDER + i] + ptr_lsfq[3 * ORDER + i]);
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[2 * ORDER], lsfq, indxt, 1);
  num_bits += 1;

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 1;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[2 * ORDER + i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = ptr_lsfq[ORDER + i];
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[2 * ORDER], lsfq, indxt, 2);
  num_bits += 3;

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 2;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[2 * ORDER + i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  for (i = 0; i < ORDER; i++)
  {
    lsfq[i] = ptr_lsfq[3 * ORDER + i];
  }

  num_bits = impeghe_avq_first_approx_rel(&ptr_lsf[2 * ORDER], lsfq, indxt, 2);
  num_bits += 3;

  if (num_bits < nbt)
  {
    nbt = num_bits;
    nit = impeghe_get_num_params(&indxt[0]);
    ptr_index[-1] = 3;
    for (i = 0; i < ORDER; i++)
    {
      ptr_lsfq[2 * ORDER + i] = lsfq[i];
    }
    for (i = 0; i < nit; i++)
    {
      ptr_index[i] = indxt[i];
    }
  }

  ptr_index += nit;
  *nb_indices += nit;
  *nbbits += nbt;

  return;
}

/** @} */ /* End of CoreEncProc */
