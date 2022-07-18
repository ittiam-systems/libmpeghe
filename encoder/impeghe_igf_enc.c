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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_psy_mod.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_qc_util.h"
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
#include "impeghe_cplx_pred.h"
#include "impeghe_rom.h"

#define CF_OFFSET_SE_01 (+2)
#define CF_OFFSET_SE_10 (-4)

/**
 *  impeghe_quant_ctx
 *
 *  \brief Limits values to +-3
 *
 *  \param [in] input value
 *
 *  \return WORD32	output value
 */
static WORD32 impeghe_quant_ctx(WORD32 x)
{
  if (abs(x) <= 3)
  {
    return x;
  }
  else
  {
    if (x < 0)
    {
      return -3;
    }
    else
    {
      return 3;
    }
  }
}

/**
 *  impeghe_arith_encode_bits
 *
 *  \brief Arithmetic encoding.
 *
 *  \param [out]    it_bit_buff  Bit Buffer
 *  \param [in,out]    bit_pos           bit count
 *  \param [in,out]    pstr_state         Pointer to arithmetic coding state structure.
 *  \param [in]     value        Value to encode
 *  \param [in]     n_bits       Number of bits to encode
 *
 *  \return VOID
 *
 */
static VOID impeghe_arith_encode_bits(ia_bit_buf_struct *it_bit_buf, WORD32 *bit_pos,
                                      impeghe_state_arith *pstr_state, WORD32 value,
                                      WORD32 n_bits)
{
  WORD32 i;
  for (i = n_bits - 1; i >= 0; --i)
  {
    WORD32 bit = (value >> i) & 1;
    (*bit_pos) = impeghe_arith_encode(it_bit_buf, *bit_pos, pstr_state, bit, impeghe_cf_for_bit);
  }
  return;
}

/**
 *  impeghe_arith_encode_residual
 *
 *  \brief Arithmetic encoding of residual value.
 *
 *  \param [in]   it_bit_buf    Bit buffer
 *  \param [in,out]  bit_pos            Bit count
 *  \param [in,out]  pstr_state          Pointer to arithmetic coding state structure.
 *  \param [in]   value         Value to encode
 *  \param [in]   ptr_cf_table  Cumulative frequency table
 *  \param [in]   table_offset  Offset to frequency	table
 *
 *  \return VOID
 *
 */
static VOID impeghe_arith_encode_residual(ia_bit_buf_struct *it_bit_buf, WORD32 *bit_pos,
                                          impeghe_state_arith *pstr_state, WORD32 value,
                                          const UWORD16 *ptr_cf_table, WORD32 table_offset)
{
  value += table_offset;

  if (value >= -12 && value <= 12)
  {
    *bit_pos = impeghe_arith_encode(it_bit_buf, *bit_pos, pstr_state, value + 13, ptr_cf_table);
  }
  else if (value < -12)
  {
    WORD32 extra = -value - 13;
    *bit_pos = impeghe_arith_encode(it_bit_buf, *bit_pos, pstr_state, 0, ptr_cf_table);
    if (extra < 15)
    {
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, extra, 4);
    }
    else
    {
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, 15, 4);
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, extra - 15, 7);
    }
  }
  else
  {
    WORD32 extra = value - 13;
    *bit_pos = impeghe_arith_encode(it_bit_buf, *bit_pos, pstr_state, 26, ptr_cf_table);
    if (extra < 15)
    {
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, extra, 4);
    }
    else
    {
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, 15, 4);
      impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, extra - 15, 7);
    }
  }
  return;
}

/**
 *  impeghe_igf_arith_encode
 *
 *  \brief Arithmetic encoding of IGF levels
 *
 *  \param [in] it_bit_buf       Bit buffer
 *  \param [in,out] bit_pos              Bit count
 *  \param [in] pstr_igf_data    Pointer to IGF data
 *  \param [in] pstr_igf_config  Pointer to IGF config
 *  \param [in] ptr_igf_curr     IGF current levels
 *  \param [in] t                time index	since the last reset
 *  \param [in,out] pstr_state           Pointer to arithmetic coding state structure.
 *
 *  \return VOID
 *
 */
static VOID impeghe_igf_arith_encode(ia_bit_buf_struct *it_bit_buf, WORD32 *bit_pos,
                                     ia_igf_data_struct *pstr_igf_data,
                                     ia_igf_config_struct *pstr_igf_config, WORD32 *ptr_igf_curr,
                                     WORD32 t, impeghe_state_arith *pstr_state, WORD32 core_mode)
{
  WORD32 m_igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 m_igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;
  WORD32 igf_p = pstr_igf_config->igf_use_high_res ? 1 : 2;
  WORD32 igf_inc = igf_p;
  WORD32 f, z;
  WORD32 pred, ctx, ctx_f, ctx_t;
  WORD32 *ptr_igf_prev = pstr_igf_data->igf_level_prev + m_igf_start_sfb;
  ptr_igf_curr = ptr_igf_curr + m_igf_start_sfb;
  WORD32 prev_d = pstr_igf_data->igf_prev_d;

  if (pstr_igf_config->is_short_block || core_mode == 1)
  {
    igf_inc = 1;
  }

  for (f = 0; f < m_igf_stop_sfb - m_igf_start_sfb; f += igf_inc)
  {
    if (t == 0)
    {
      if (f == 0)
      {
        impeghe_arith_encode_bits(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f], 7);
      }
      else if (f == igf_inc)
      {
        pred = ptr_igf_curr[f - igf_inc];
        impeghe_arith_encode_residual(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f] - pred,
                                      impeghe_cf_se_01, CF_OFFSET_SE_01);
      }
      else
      {
        pred = ptr_igf_curr[f - igf_inc];
        ctx = impeghe_quant_ctx(ptr_igf_curr[f - igf_inc] - ptr_igf_curr[f - 2 * igf_inc]);
        impeghe_arith_encode_residual(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f] - pred,
                                      impeghe_cf_se_02[3 + ctx], impeghe_cf_off_se_02[3 + ctx]);
      }
    }
    else if (f == 0)
    {
      if (t == 1)
      {
        pred = ptr_igf_prev[f];
        impeghe_arith_encode_residual(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f] - pred,
                                      impeghe_cf_se_10, CF_OFFSET_SE_10);
      }
      else
      {
        pred = ptr_igf_prev[f];
        ctx = impeghe_quant_ctx(ptr_igf_prev[f] - prev_d);
        impeghe_arith_encode_residual(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f] - pred,
                                      impeghe_cf_se_20[3 + ctx], impeghe_cf_off_se_20[3 + ctx]);
      }
    }
    else
    {
      pred = ptr_igf_prev[f] + ptr_igf_curr[f - igf_inc] - ptr_igf_prev[f - igf_inc];
      ctx_f = impeghe_quant_ctx(ptr_igf_prev[f] - ptr_igf_prev[f - igf_inc]);
      ctx_t = impeghe_quant_ctx(ptr_igf_curr[f - igf_inc] - ptr_igf_prev[f - igf_inc]);
      impeghe_arith_encode_residual(it_bit_buf, bit_pos, pstr_state, ptr_igf_curr[f] - pred,
                                    impeghe_cf_se_11[3 + ctx_t][3 + ctx_f],
                                    impeghe_cf_off_se_11[3 + ctx_t][3 + ctx_f]);
    }
    for (z = f + 1; z < min(f + igf_inc, m_igf_stop_sfb - m_igf_start_sfb); ++z)
    {
      ptr_igf_curr[z] = ptr_igf_curr[f];
    }
  }
  ptr_igf_curr -= m_igf_start_sfb;

  return;
}

/**
 *  impeghe_igf_tnf_norm
 *
 *  \brief Calculates norm values
 *
 *  \param [in]    ptr_spec    tnf spec coeff
 *  \param [in]    n    range
 *
 *  \return FLOAT64 Norm value
 *
 */
static FLOAT64 impeghe_igf_tnf_norm(const FLOAT64 *ptr_spec, const WORD32 n)
{
  WORD32 i;
  FLOAT64 acc = 0.0f;

  for (i = 0; i < n; i++)
  {
    acc += ptr_spec[i] * ptr_spec[i];
  }
  return acc;
}

/**
 *  impeghe_igf_tnf_convert_lpc
 *
 *  \brief Calculates LPC pred coeffs and LPC gain
 *
 *  \param [in]  ptr_input      acf Input
 *  \param [in]  order      order of filter
 *  \param [out] pred_gain  Prediction gain
 *  \param [out] ptr_igf_scratch  Scratch memory for IGF
 *
 *  \return VOID
 *
 */
static VOID impeghe_igf_tnf_convert_lpc(const FLOAT64 *ptr_input, WORD32 order,
                                        FLOAT64 *pred_gain, FLOAT64 *ptr_igf_scratch)
{
  WORD32 i;
  WORD32 j;
  FLOAT64 tmp;
  FLOAT64 tmp2;
  FLOAT64 *ptr_mem = ptr_igf_scratch;
  memset(ptr_mem, 0, 32 * sizeof(ptr_mem[0]));
  FLOAT64 *const ptr_mem2 = &ptr_mem[order];
  const FLOAT64 threshold = 0.0000152;

  for (i = 0; i < order; i++)
  {
    ptr_mem[i] = ptr_input[i];
    ptr_mem2[i] = ptr_input[i + 1];
  }

  for (i = 0; i < order; i++)
  {
    tmp = 0;
    if (ptr_mem[0] >= threshold)
    {
      tmp = -ptr_mem2[i] / ptr_mem[0];
    }

    tmp = min(0.999, max(-0.999, tmp));

    for (j = i; j < order; j++)
    {
      tmp2 = ptr_mem2[j] + tmp * ptr_mem[j - i];
      ptr_mem[j - i] += tmp * ptr_mem2[j];
      ptr_mem2[j] = tmp2;
    }
  }

  *pred_gain = ((ptr_input[0] + 1e-30) / (ptr_mem[0] + 1e-30));
  return;
}

/**
 *  impeghe_igf_tnf_autocorrelation
 *
 *  \brief Calculates tnf autocorrelation
 *
 *  \param [in] ptr_spec    tnf spectral coeff
 *  \param [in] n    tnf range
 *  \param [in] lag  lag
 *
 *  \return FLOAT64  Autocorrelation value
 *
 */
static FLOAT64 impeghe_igf_tnf_autocorrelation(const FLOAT64 *ptr_spec, const WORD32 n,
                                               const WORD32 lag)
{
  WORD32 i;
  FLOAT64 acc = 0.0f;

  if (n - lag)
  {
    acc = ptr_spec[0] * ptr_spec[lag];
  }
  for (i = 1; i < n - lag; i++)
  {
    acc += ptr_spec[i] * ptr_spec[i + lag];
  }
  return acc;
}
/**
 *  impeghe_igf_tnf_detect
 *
 *  \brief Detects igf tnf and estimates pred coeffs and gain
 *
 *  \param [in] ptr_spec     tnf spectral coeff
 *  \param [in] igf_start  igf start subband
 *  \param [in] igf_stop   igf stop subband
 *  \param [in] pred_gain  estimated prediction gain
 *  \param [in] igf_min    subband index which corresponding to min frequency
 *
 *  \return VOID
 *
 */
static VOID impeghe_igf_tnf_detect(const FLOAT64 *ptr_spec, const WORD32 igf_start,
                                   const WORD32 igf_stop, FLOAT64 *pred_gain, WORD32 igf_min,
                                   FLOAT64 *ptr_igf_scratch)
{
  WORD32 i;
  WORD32 lag;
  WORD32 start_line;
  WORD32 stop_line;
  WORD32 tnf_range;
  FLOAT64 fac;
  FLOAT64 norms[3] = {0.0f};
  FLOAT64 *ptr_acf_x = ptr_igf_scratch;
  memset(ptr_acf_x, 0, (16 + 1) * sizeof(ptr_acf_x[0]));
  ptr_igf_scratch += (16 + 1);
  const WORD32 n_div = 3;
  const WORD32 igf_range = igf_stop - igf_start;
  const FLOAT32 threshold = 0.0000000037f;

  for (i = 0; i < n_div; i++)
  {
    start_line = igf_start + (igf_stop - igf_start) * i / n_div;
    stop_line = igf_start + (igf_stop - igf_start) * (i + 1) / n_div;
    tnf_range = stop_line - start_line;

    norms[i] = impeghe_igf_tnf_norm(ptr_spec + start_line - igf_min, tnf_range);
  }

  for (i = 0; (i < n_div) && (norms[i] > threshold); i++)
  {
    fac = 1.0 / norms[i];
    start_line = igf_start + igf_range * i / n_div;
    stop_line = igf_start + igf_range * (i + 1) / n_div;
    tnf_range = stop_line - start_line;

    for (lag = 1; lag <= 8; lag++)
    {
      ptr_acf_x[lag] +=
          fac * impeghe_tnf_acf_win[lag - 1] *
          impeghe_igf_tnf_autocorrelation(ptr_spec + start_line - igf_min, tnf_range, lag);
    }
  }

  if (i == n_div)
  {
    ptr_acf_x[0] = (FLOAT32)n_div;
    impeghe_igf_tnf_convert_lpc(ptr_acf_x, min(8, igf_range >> 2), pred_gain, ptr_igf_scratch);
  }
  return;
}

/**
 *  impeghe_igf_get_sb
 *
 *  \brief Calculate igf source subbands
 *
 *  \param [in] igf_bgn    IGF start	subband
 *  \param [in] tb         index of target subband
 *  \param [in] tile       tile width
 *  \param [in] igf_min    subband index which corresponding to min frequency
 *  \param [in] sel_tile   Source tile index
 *
 *  \return WORD32   Source subband
 *
 */
static WORD32 impeghe_igf_get_sb(WORD32 igf_min, WORD32 igf_bgn, WORD32 tb, WORD32 tile,
                                 WORD8 sel_tile)
{
  WORD32 sbs = igf_bgn;
  WORD32 offset = 0;
  WORD32 sb = tb;
  WORD32 src = igf_bgn - igf_min;

  if (src > 0)
  {
    offset = (WORD32)(((-1 * sel_tile + 5) * tile) >> 1) + sbs - igf_bgn;
    sb = tb - offset;
    if (sb < igf_min)
    {
      sb = (igf_min + tb % src);
    }
  }
  return sb;
}

/**
 *  impeghe_find_src_tile
 *
 *  \brief Find source tile matching with target tile
 *
 *  \param [in] ptr_sfb_offset      offset table for scalefactor band
 *  \param [in] num_tiles           Number of source tiles
 *  \param [in] tile_width          Target tiles width
 *  \param [in] spec_start          spectrum start subband
 *  \param [in] igf_min             subband index which corresponding to min frequency
 *  \param [in] ptr_in_spec         Pointer to spectrum
 *  \param [in] tgt_tile_enegry     Target tile energey
 *  \param [out] ptr_src_tile_tonal  Pointer to store info if source tile is tonal
 *
 *  \return WORD32                  Source tile index
 *
 */
static WORD32 impeghe_find_src_tile(WORD32 *ptr_sfb_offset, WORD32 num_tiles, WORD32 tile_width,
                                    WORD32 spec_start, WORD32 igf_min, FLOAT64 *ptr_in_spec,
                                    FLOAT64 tgt_tile_enegry, WORD32 *ptr_src_tile_tonal,
                                    FLOAT64 *ptr_igf_scratch)
{
  WORD32 ii;
  WORD32 idx;
  WORD32 src_tile_idx = 3, max_ener_src_idx = 0;
  FLOAT64 src_tile_energy[MAX_IGF_TILES] = {0};
  FLOAT64 *ptr_src_energy = ptr_igf_scratch;
  memset(ptr_src_energy, 0, 51 * sizeof(ptr_src_energy[0]));
  FLOAT64 max_band_energy = 0;
  FLOAT64 max_corr = 0;

  for (ii = 0; ii < num_tiles; ii++)
  {
    WORD32 start = ptr_sfb_offset[spec_start];
    WORD32 spec_stop = start + tile_width;
    WORD32 band = spec_start;
    FLOAT64 corr[MAX_IGF_TILES] = {0};
    FLOAT64 auto_corr[MAX_IGF_TILES] = {0};
    for (; start < spec_stop; start++)
    {
      idx = impeghe_igf_get_sb(igf_min, ptr_sfb_offset[spec_start], start, tile_width, ii);

      ptr_src_energy[band] += ptr_in_spec[idx] * ptr_in_spec[idx];
      src_tile_energy[ii] += ptr_in_spec[idx] * ptr_in_spec[idx];

      if (start >= ptr_sfb_offset[band + 1])
      {
        if (max_band_energy < ptr_src_energy[band])
        {
          max_band_energy = ptr_src_energy[band];
          max_ener_src_idx = band;
        }
        band++;
      }

      corr[ii] += ptr_in_spec[idx] * ptr_in_spec[start];
      auto_corr[ii] += ptr_in_spec[idx] * ptr_in_spec[idx];
      corr[ii] = corr[ii] / auto_corr[ii];
    }
    if (max_corr < fabs(corr[ii]))
    {
      max_corr = fabs(corr[ii]);
      src_tile_idx = ii;
    }
    if (max_band_energy != 0)
    {
      if ((ptr_src_energy[max_ener_src_idx - 1] / ptr_src_energy[max_ener_src_idx] < 0.1) &&
          (ptr_src_energy[max_ener_src_idx + 1] / ptr_src_energy[max_ener_src_idx] < 0.1))
      {
        ptr_src_tile_tonal[ii] = 1;
      }
    }
  }
  return src_tile_idx;
}

/**
 *  impeghe_calc_whitening_lvl
 *
 *  \brief Calculate IGF whitening levels
 *
 *  \param [in] pstr_igf_data    Pointer to IGF data
 *  \param [in] pstr_igf_config  Pointer to IGF config
 *  \param [in] ptr_sfb_offset   offset table for scalefactor band
 *  \param [in] ptr_in_spec      Pointer to spectrum
 *  \param [in] igf_min          subband index which corresponding to min frequency
 *  \param [in] core_mode        Core coder mode
 *
 *  \return VOID
 *
 */
static VOID impeghe_calc_whitening_lvl(ia_igf_data_struct *pstr_igf_data,
                                       ia_igf_config_struct *pstr_igf_config,
                                       WORD32 *ptr_sfb_offset, FLOAT64 *ptr_in_spec,
                                       WORD32 igf_min, WORD32 core_mode, FLOAT64 *ptr_igf_scratch)
{
  WORD32 m_igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  FLOAT64 *ptr_src_energy = (FLOAT64 *)ptr_igf_scratch;
  memset(ptr_src_energy, 0, MAX_NUM_SFB_LONG * sizeof(ptr_src_energy[0]));
  ptr_igf_scratch += MAX_NUM_SFB_LONG;
  WORD32 i, tile_idx;
  WORD32 num_tiles = (core_mode == CORE_MODE_FD) ? pstr_igf_data->igf_n_tiles_long
                                                 : pstr_igf_data->igf_n_tiles_short;
  WORD32 k;
  WORD32 spec_start = m_igf_start_sfb;
  WORD32 n_st[MAX_IGF_TILES] = {0};
  WORD32 ptr_tile_width[MAX_IGF_TILES] = {0};

  for (i = 0; i < MAX_IGF_TILES; i++)
  {
    pstr_igf_data->igf_prev_whitening_level[i] = pstr_igf_data->igf_whitening_level[i];
    pstr_igf_data->igf_whitening_level[i] = 1;
    pstr_igf_data->igf_src_tile_idx[i] = 3;
    ptr_tile_width[i] = pstr_igf_data->igf_curr_tile_width_long[i];
  }

  if (core_mode == 1)
  {
    impeghe_get_tile_info_tcx(pstr_igf_config->igf_start_sfb_sb, pstr_igf_config->igf_stop_sfb_sb,
                              pstr_igf_config->igf_use_high_res, igf_min, ptr_sfb_offset,
                              ptr_tile_width, &num_tiles);
  }

  for (tile_idx = 0; tile_idx < num_tiles; tile_idx++)
  {
    FLOAT64 max_tgt_enegry = 0;
    WORD32 max_ener_tgt_idx = 0, tgt_is_tonal = 0;
    WORD32 spec_stop = ptr_sfb_offset[spec_start] + ptr_tile_width[tile_idx];
    WORD32 src_is_tonal[MAX_IGF_TILES] = {0};
    FLOAT64 tgt_tile_energy = 0;
    n_st[tile_idx] =
        max(1, min(MAX_IGF_TILES, (WORD32)ceil((ptr_sfb_offset[m_igf_start_sfb] - igf_min) /
                                               (float)(ptr_tile_width[tile_idx] / 2)) -
                                      1));

    for (k = spec_start; ptr_sfb_offset[k] < spec_stop; k++)
    {
      WORD32 width_wk = ptr_sfb_offset[k + 1] - ptr_sfb_offset[k];
      ptr_src_energy[k] = 0;
      for (i = 0; i < width_wk; i++)
      {
        ptr_src_energy[k] +=
            ptr_in_spec[ptr_sfb_offset[k] + i] * ptr_in_spec[ptr_sfb_offset[k] + i];
      }
      tgt_tile_energy += ptr_src_energy[k];
      if (max_tgt_enegry < ptr_src_energy[k])
      {
        max_tgt_enegry = ptr_src_energy[k];
        max_ener_tgt_idx = k;
      }
    }

    pstr_igf_data->igf_src_tile_idx[tile_idx] = impeghe_find_src_tile(
        ptr_sfb_offset, n_st[tile_idx], ptr_tile_width[tile_idx], m_igf_start_sfb, igf_min,
        ptr_in_spec, tgt_tile_energy, src_is_tonal, ptr_igf_scratch);

    if (max_tgt_enegry != 0)
    {
      if (max_ener_tgt_idx != spec_start && max_ener_tgt_idx != k - 1)
      {
        if ((ptr_src_energy[max_ener_tgt_idx - 1] / ptr_src_energy[max_ener_tgt_idx] < .5) &&
            (ptr_src_energy[max_ener_tgt_idx + 1] / ptr_src_energy[max_ener_tgt_idx] < 0.5))
        {
          tgt_is_tonal = 1;
        }
      }
    }

    if (0 == tgt_is_tonal && src_is_tonal[pstr_igf_data->igf_src_tile_idx[tile_idx]] == 1)
    {
      pstr_igf_data->igf_whitening_level[tile_idx] = 2;
    }

    if (1 == tgt_is_tonal && src_is_tonal[pstr_igf_data->igf_src_tile_idx[tile_idx]] == 1)
    {
      pstr_igf_data->igf_whitening_level[tile_idx] = 0;
    }

    spec_start = k;
  }
  spec_start = m_igf_start_sfb;

  for (tile_idx = 0; tile_idx < num_tiles; tile_idx++)
  {
    WORD32 spec_stop = ptr_sfb_offset[spec_start] + ptr_tile_width[tile_idx];
    for (k = spec_start; ptr_sfb_offset[k] < spec_stop; k++)
    {
      WORD32 width_wk = ptr_sfb_offset[k + 1] - ptr_sfb_offset[k];
      for (i = 0; i < width_wk; i++)
      {
        ptr_in_spec[ptr_sfb_offset[k] + i] = 0;
      }
    }
    spec_start = k;
  }
  return;
}

/**
 *  impeghe_igf
 *
 *  \brief Calculate IGF levels, and other features
 *
 *  \param [in] ptr_in_spec       Pointer to spectrum
 *  \param [in] ptr_sfb_offset    offset table for scalefactor band
 *  \param [in,out] pstr_igf_data     Pointer to IGF data
 *  \param [in,out] pstr_igf_config   Pointer to IGF config
 *  \param [in] group_id          Window group index
 *  \param [in] tns_pred_gain     TNS prediction gain
 *  \param [in] win_group_length  Window group length
 *  \param [in] tns_data_present  TNS present flag
 *  \param [in] igf_after_tns_synth  IGF after TNS flag
 *  \param [in] igf_emphasis         IGF emphasis
 *  \param [in] core_mode            Core coder mode
 *
 *  \return VOID
 *
 */
VOID impeghe_igf(FLOAT64 *ptr_in_spec, WORD32 *ptr_sfb_offset, ia_igf_data_struct *pstr_igf_data,
                 ia_igf_config_struct *pstr_igf_config, WORD32 group_id, FLOAT64 tns_pred_gain,
                 WORD32 win_group_length, WORD32 tns_data_present, WORD32 igf_after_tns_synth,
                 WORD32 igf_emphasis, WORD32 core_mode, FLOAT64 *ptr_igf_scratch)
{
  WORD32 k, i;
  WORD32 m_igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 m_igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;
  WORD32 *ptr_ek = pstr_igf_data->igf_level[group_id];
  FLOAT64 pred_gain = 0;
  /* calculate ek */

  pstr_igf_data->igf_all_zero = 1;
  pstr_igf_config->igf_apply_tnf = 0;

  memset(ptr_ek, 0, MAX_NUM_SFB_LONG * sizeof(WORD32));
  for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k++)
  {
    WORD32 width_wk = ptr_sfb_offset[k + 1] - ptr_sfb_offset[k];
    FLOAT64 temp = 0;
    for (i = 0; i < width_wk; i++)
    {
      temp += ptr_in_spec[ptr_sfb_offset[k] + i] * ptr_in_spec[ptr_sfb_offset[k] + i];
    }
    temp = sqrt(temp / width_wk);
    if (temp != 0)
    {
      ptr_ek[k] = (WORD32)round(4 * log2(temp)) + igf_emphasis;
      if (ptr_ek[k] < 0)
      {
        ptr_ek[k] = 0;
      }
    }
  }

  if (pstr_igf_config->igf_use_high_res == 0)
  {
    for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k++)
    {
      ptr_ek[k] >>= 1;
    }
  }

  /* To determine if IGF data is all zeros */
  for (WORD32 group_len = 0; group_len < win_group_length; group_len++)
  {
    WORD32 igf_p = pstr_igf_config->igf_use_high_res ? 1 : 2;
    for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k += igf_p)
    {
      if (pstr_igf_data->igf_level[group_id][k] != 0)
      {
        pstr_igf_data->igf_all_zero = 0;
        break;
      }
    }
    if (pstr_igf_data->igf_all_zero == 0)
      break;
  }
  if (pstr_igf_config->is_short_block == 0)
  {
    WORD32 igf_min = pstr_igf_config->igf_min_l;

    if (core_mode == 1)
    {
      igf_min = pstr_igf_config->igf_min_tcx;
    }

    if (tns_data_present == 1 && igf_after_tns_synth)
    {
      impeghe_igf_tnf_detect(ptr_in_spec, ptr_sfb_offset[m_igf_start_sfb],
                             ptr_sfb_offset[m_igf_stop_sfb], &pred_gain, igf_min,
                             ptr_igf_scratch);

      if (pred_gain < 1.15 && tns_pred_gain < 1.15)
      {
        pstr_igf_config->igf_apply_tnf = 1;
      }
    }
    impeghe_calc_whitening_lvl(pstr_igf_data, pstr_igf_config, ptr_sfb_offset, ptr_in_spec,
                               igf_min, core_mode, ptr_igf_scratch);
  }

  return;
}

/**
 *  impeghe_igf_ms
 *
 *  \brief Calculate IGF levels after M/S processing
 *
 *  \param [in]  ptr_in_spec       Pointer to spectrum
 *  \param [in]  ptr_sfb_offset    offset table for scalefactor band
 *  \param [in,out] pstr_igf_data     Pointer to IGF data
 *  \param [in]  pstr_igf_config   Pointer to IGF config
 *  \param [in]  sfb_per_group     Number of scalefactors per group
 *  \param [in]  num_sfb           Number of scalefactors
 *
 *  \return VOID
 *
 */
VOID impeghe_igf_ms(FLOAT64 *ptr_in_spec, WORD32 *ptr_sfb_offset,
                    ia_igf_data_struct *pstr_igf_data, ia_igf_config_struct *pstr_igf_config,
                    WORD32 sfb_per_group, WORD32 num_sfb)
{
  WORD32 k, i;
  WORD32 m_igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 m_igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;

  /* calculate ek */
  WORD32 grp = 0;

  pstr_igf_data->igf_all_zero = 1;
  pstr_igf_config->igf_apply_tnf = 0;

  for (WORD32 sfb = 0; sfb < num_sfb; sfb += sfb_per_group, grp++)
  {
    WORD32 *ptr_ek = pstr_igf_data->igf_level[grp];
    memset(ptr_ek, 0, MAX_NUM_SFB_LONG * sizeof(WORD32));
    for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k++)
    {
      WORD32 width_wk = ptr_sfb_offset[sfb + k + 1] - ptr_sfb_offset[sfb + k];
      FLOAT64 temp = 0;
      for (i = 0; i < width_wk; i++)
      {
        temp +=
            ptr_in_spec[ptr_sfb_offset[sfb + k] + i] * ptr_in_spec[ptr_sfb_offset[sfb + k] + i];
      }
      temp = sqrt(temp / width_wk);
      if (temp != 0)
      {
        ptr_ek[k] = (WORD32)round(4 * log2(temp));
        if (ptr_ek[k] < 0)
        {
          ptr_ek[k] = 0;
        }
      }
    }

    if (pstr_igf_config->igf_use_high_res == 0)
    {
      for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k++)
      {
        ptr_ek[k] >>= 1;
      }
    }
  }

  /* To determine if IGF data is all zeros */
  for (WORD32 g = 0; g < grp; g++)
  {
    for (k = m_igf_start_sfb; k < m_igf_stop_sfb; k++)
    {
      if (pstr_igf_data->igf_level[g][k] != 0)
      {
        pstr_igf_data->igf_all_zero = 0;
        break;
      }
    }
  }

  return;
}

/**
 *  impeghe_write_igf_data
 *
 *  \brief Write IGF data to bit buffer
 *
 *  \param [in,out] it_bit_buf              Bit buffer
 *  \param [in]  pstr_igf_data           Pointer to IGF data
 *  \param [in]  pstr_igf_config         Pointer to IGF config
 *  \param [in]  usac_independency_flag  USAC independency flag
 *  \param [in]  core_mode               Core coder mode
 *
 *  \return WORD32                       Number of bits written
 *
 */
WORD32 impeghe_write_igf_data(ia_bit_buf_struct *it_bit_buf, ia_igf_data_struct *pstr_igf_data,
                              ia_igf_config_struct *pstr_igf_config,
                              WORD32 usac_independency_flag, WORD32 core_mode)
{
  WORD32 bit_count = 0;
  WORD32 igf_n_tiles, i;
  WORD32 igf_enable_wht = 0;
  WORD32 use_prev_tile = 0; /* can be 0 or 1 */
  WORD32 igf_use_prev_whitening_level = 0;
  WORD32 remaining_tiles_different = 0;

  if (core_mode == 1)
  {
    igf_n_tiles = pstr_igf_data->igf_n_tiles_short;
  }
  else
  {
    if (pstr_igf_config->is_short_block == 1)
    {
      igf_n_tiles = pstr_igf_data->igf_n_tiles_short;
    }
    else
    {
      igf_n_tiles = pstr_igf_data->igf_n_tiles_long;
    }
  }

  if (usac_independency_flag == 0)
  {
    impeghe_write_bits_buf(it_bit_buf, use_prev_tile, 1);
    bit_count += 1;
  }
  else
  {
    use_prev_tile = 0;
  }
  if (use_prev_tile == 0)
  {
    for (i = 0; i < igf_n_tiles; i++)
    {
      impeghe_write_bits_buf(it_bit_buf, pstr_igf_data->igf_src_tile_idx[i], 2);
      bit_count += 2;
    }
  }
  if (core_mode == 0)
  {
    igf_enable_wht = (!pstr_igf_config->is_short_block);
  }
  else
  {
    igf_enable_wht = 1;
  }

  if (igf_enable_wht == 1 && pstr_igf_config->igf_use_whitening == 1)
  {
    if (usac_independency_flag == 0)
    {
      impeghe_write_bits_buf(it_bit_buf, igf_use_prev_whitening_level, 1);
      bit_count += 1;
    }
    else
    {
      igf_use_prev_whitening_level = 0;
    }

    if (igf_use_prev_whitening_level == 0)
    {
      if (pstr_igf_data->igf_whitening_level[0] != 0)
      {
        WORD32 temp = pstr_igf_data->igf_whitening_level[0] - 1;
        impeghe_write_bits_buf(it_bit_buf, 1, 1);
        impeghe_write_bits_buf(it_bit_buf, temp, 1);
        bit_count += 2;
      }
      else
      {
        impeghe_write_bits_buf(it_bit_buf, pstr_igf_data->igf_whitening_level[0], 1);
        bit_count += 1;
      }

      for (i = 1; i < igf_n_tiles; i++)
      {
        if (pstr_igf_data->igf_whitening_level[0] != pstr_igf_data->igf_whitening_level[i])
        {
          remaining_tiles_different = 1;
          break;
        }
      }
      impeghe_write_bits_buf(it_bit_buf, remaining_tiles_different, 1);
      bit_count += 1;
      if (remaining_tiles_different == 1)
      {
        for (i = 1; i < igf_n_tiles; i++)
        {
          if (pstr_igf_data->igf_whitening_level[i] != 0)
          {
            WORD32 temp = pstr_igf_data->igf_whitening_level[i] - 1;
            impeghe_write_bits_buf(it_bit_buf, 1, 1);
            impeghe_write_bits_buf(it_bit_buf, temp, 1);
            bit_count += 2;
          }
          else
          {
            impeghe_write_bits_buf(it_bit_buf, pstr_igf_data->igf_whitening_level[i], 1);
            bit_count += 1;
          }
        }
      }
    }
  }

  if (pstr_igf_config->igf_use_enf == 1)
  {
    if (pstr_igf_config->is_short_block == 0)
    {
      impeghe_write_bits_buf(it_bit_buf, pstr_igf_config->igf_apply_tnf, 1);
      bit_count += 1;
    }
  }

  return bit_count;
}

/**
 *  impeghe_write_igf_levels
 *
 *  \brief Write IGF levels to bit buffer
 *
 *  \param [in,out] it_bit_buf            Bit buffer
 *  \param [in] pstr_igf_data          Pointer to IGF data
 *  \param [in] pstr_igf_config        Pointer to IGF config
 *  \param [in] usac_independency_flg  USAC independency flag
 *  \param [in] core_mode              Core coder mode
 *  \param [in] num_window_groups      Number of window groups
 *
 *  \return WORD32                     Number of bits written
 *
 */
WORD32 impeghe_write_igf_levels(ia_bit_buf_struct *it_bit_buf, ia_igf_data_struct *pstr_igf_data,
                                ia_igf_config_struct *pstr_igf_config,
                                WORD32 usac_independency_flg, WORD32 core_mode,
                                WORD32 num_window_groups)
{
  WORD32 g, sfb;
  WORD32 bit_count = 0;
  WORD32 m_igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 m_igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;
  impeghe_state_arith str_state = {0};
  WORD32 curr_window_sequence = num_window_groups > 1 ? 0 : 1;
  WORD32 prev_window_sequence = pstr_igf_data->prev_num_windows > 1 ? 0 : 1;

  if ((pstr_igf_data->igf_all_zero != 0) || (usac_independency_flg == 1) ||
      ((curr_window_sequence != prev_window_sequence) && (core_mode == 0)) ||
      ((pstr_igf_data->lpd_mode != pstr_igf_data->last_lpd_mode) && (core_mode == 1)) ||
      (core_mode != pstr_igf_data->code_mode_prev))
  {
    memset(pstr_igf_data->igf_level_prev, 0, MAX_NUM_SFB_LONG * sizeof(WORD32));
    pstr_igf_data->igf_arith_t = 0;
    pstr_igf_data->igf_prev_d = 0;
  }

  pstr_igf_data->prev_num_windows = num_window_groups;
  pstr_igf_data->code_mode_prev = core_mode;

  bit_count += impeghe_write_bits_buf(it_bit_buf, pstr_igf_data->igf_all_zero, 1);

  str_state.low = 0;
  str_state.high = 65535;
  str_state.value = 0;

  if (pstr_igf_data->igf_all_zero == 0)
  {
    for (g = 0; g < num_window_groups; g++)
    {
      impeghe_igf_arith_encode(it_bit_buf, &bit_count, pstr_igf_data, pstr_igf_config,
                               pstr_igf_data->igf_level[g], pstr_igf_data->igf_arith_t,
                               &str_state, core_mode);
      pstr_igf_data->igf_prev_d = pstr_igf_data->igf_level_prev[m_igf_start_sfb];
      for (sfb = m_igf_start_sfb; sfb < m_igf_stop_sfb; sfb++)
      {
        pstr_igf_data->igf_level_prev[sfb] = pstr_igf_data->igf_level[g][sfb];
      }
      pstr_igf_data->igf_arith_t++;
    }
    bit_count = impeghe_arith_done(it_bit_buf, bit_count, &str_state);
  }

  return bit_count;
}

/**
 *  impeghe_get_tile_info
 *
 *  \brief Calculate number and width of tiles
 *
 *  \param [in] sfb_bgn         IGF start subband
 *  \param [in] sfb_end         IGF stop subband
 *  \param [in] buh             High resoultion flag
 *  \param [in] igf_min         subband index which corresponding to min frequency
 *  \param [in] ptr_sfb_offset  offset table for scalefactor band
 *  \param [in,out] pstr_igf_data  Pointer to IGF data
 *  \param [in] is_short_block  Flag to indication short/long block
 *
 *  \return VOID
 *
 */
VOID impeghe_get_tile_info(WORD32 sfb_bgn, WORD32 sfb_end, WORD32 buh, WORD32 igf_min,
                           WORD32 *ptr_sfb_offset, ia_igf_data_struct *pstr_igf_data,
                           WORD32 is_short_block)
{
  WORD32 i = 0, n;
  WORD32 igf_bgn = ptr_sfb_offset[sfb_bgn];
  WORD32 igf_end = ptr_sfb_offset[sfb_end];
  WORD32 mem = sfb_bgn;
  WORD32 sbs = igf_bgn;
  WORD32 rng = igf_end - igf_bgn;
  WORD32 *ptr_tile_width;
  WORD32 *igf_n_tiles;

  if (is_short_block == 1)
  {
    ptr_tile_width = pstr_igf_data->igf_curr_tile_width_short;
    igf_n_tiles = &pstr_igf_data->igf_n_tiles_short;
  }
  else
  {
    ptr_tile_width = pstr_igf_data->igf_curr_tile_width_long;
    igf_n_tiles = &pstr_igf_data->igf_n_tiles_long;
  }

  *igf_n_tiles = 0;

  for (n = 0; n < 4; n++)
  {
    ptr_tile_width[n] = (max(8, rng) >> 2);
  }

  if (igf_bgn > igf_min && igf_bgn < igf_end)
  {
    for (i = 0; i < 4; i++)
    {
      WORD32 k = 0;
      do
      {
        k++;
      } while (min(sbs + ptr_tile_width[i], igf_end) > ptr_sfb_offset[k]);
      if (buh == 0)
      {
        if (i < 3 && ((k - sfb_bgn) % 2) && (k - mem > 1))
        {
          k--;
        }
        else
        {
          if (i == 3)
          {
            k = sfb_end;
          }
        }
        mem = k;
      }
      ptr_tile_width[i] = max(2, min(ptr_sfb_offset[k] - sbs, igf_end - sbs));
      sbs += ptr_tile_width[i];
      *igf_n_tiles = *igf_n_tiles + 1;

      if (sbs == igf_end)
      {
        break;
      }
    }
  }

  for (i = 0; i < *igf_n_tiles; i++)
  {
    pstr_igf_data->igf_whitening_level[i] = 1;
  }

  return;
}

/**
 *  impeghe_get_tile_info
 *
 *  \brief Calculate number and width of tiles for TCX
 *
 *  \param [in] sfb_bgn         IGF start subband
 *  \param [in] sfb_end         IGF stop subband
 *  \param [in] buh             High resoultion flag
 *  \param [in] igf_min         subband index which corresponding to min frequency
 *  \param [in] ptr_sfb_offset  offset table for scalefactor band
 *  \param [out] ptr_tile_width  Pointer to tile width
 *  \param [out] igf_n_tiles      Pointer to number of tiles
 *
 *  \return VOID
 *
 */
VOID impeghe_get_tile_info_tcx(WORD32 sfb_bgn, WORD32 sfb_end, WORD32 buh, WORD32 igf_min,
                               WORD32 *ptr_sfb_offset, WORD32 *ptr_tile_width,
                               WORD32 *igf_n_tiles)
{
  WORD32 i = 0, n;
  WORD32 igf_bgn = ptr_sfb_offset[sfb_bgn];
  WORD32 igf_end = ptr_sfb_offset[sfb_end];
  WORD32 mem = sfb_bgn;
  WORD32 sbs = igf_bgn;
  WORD32 rng = igf_end - igf_bgn;

  *igf_n_tiles = 0;

  for (n = 0; n < 4; n++)
  {
    ptr_tile_width[n] = (max(8, rng) >> 2);
  }

  if (igf_bgn > igf_min && igf_bgn < igf_end)
  {
    for (i = 0; i < 4; i++)
    {
      WORD32 k = 0;
      do
      {
        k++;
      } while (min(sbs + ptr_tile_width[i], igf_end) > ptr_sfb_offset[k]);
      if (buh == 0)
      {
        if (i < 3 && ((k - sfb_bgn) % 2) && (k - mem > 1))
        {
          k--;
        }
        else
        {
          if (i == 3)
          {
            k = sfb_end;
          }
        }
        mem = k;
      }
      ptr_tile_width[i] = max(2, min(ptr_sfb_offset[k] - sbs, igf_end - sbs));
      sbs += ptr_tile_width[i];
      *igf_n_tiles = *igf_n_tiles + 1;

      if (sbs == igf_end)
      {
        break;
      }
    }
  }

  return;
}

/**
 *  impeghe_igf_init
 *
 *  \brief Initialize IGF parameters
 *
 *  \param [in,out] pstr_igf_config      Pointer to IGF config
 *  \param [in] sample_rate           Sample rate
 *  \param [in] ptr_sfb_offset_long   offset table for scalefactor band for long block
 *  \param [in] ptr_sfb_offset_short  offset table for scalefactor band for short block
 *  \param [in] num_sfb               Number of scalefactors
 *  \param [in] igf_after_tns_synth   IGF after TNS flag
 *  \param [in] sfb_active            Number of active scalefactors band
 *
 *  \return VOID
 *
 */
VOID impeghe_igf_init(ia_igf_config_struct *pstr_igf_config, WORD32 sample_rate,
                      WORD32 *ptr_sfb_offset_long, WORD32 *ptr_sfb_offset_short, WORD32 num_sfb,
                      WORD32 igf_after_tns_synth, WORD32 sfb_active)
{
  WORD32 i;
  WORD32 sfb_bgn, sfb_end;

  WORD32 igf_start_sb = (pstr_igf_config->igf_start_freq * FRAME_LEN_LONG) / (sample_rate >> 1);

  WORD32 igf_stop_sb = (pstr_igf_config->igf_stop_freq * FRAME_LEN_LONG) / (sample_rate >> 1);

  pstr_igf_config->igf_use_enf = 1;
  if (pstr_igf_config->igf_start_freq > 8000)
    pstr_igf_config->igf_use_enf = 0;
  pstr_igf_config->igf_use_whitening = 1;
  pstr_igf_config->igf_after_tns_synth = igf_after_tns_synth;

  sfb_bgn = -1;
  for (i = 0; i < num_sfb; i++)
  {
    if (igf_start_sb >= ptr_sfb_offset_long[i])
    {
      sfb_bgn = i;
    }
  }
  sfb_bgn = min(num_sfb - 5, sfb_bgn);
  if (sfb_bgn < 11)
    sfb_bgn = 12;

  sfb_end = -1;
  for (i = 0; i <= num_sfb; i++)
  {
    if (igf_stop_sb >= ptr_sfb_offset_long[i])
    {
      sfb_end = i;
    }
  }
  sfb_end = max(sfb_end, sfb_bgn + 2);

  if (pstr_igf_config->is_short_block == 1)
  {
    pstr_igf_config->igf_start_sfb_sb = -1;
    for (WORD32 sfb = 0; sfb < num_sfb; sfb++)
    {
      if (ptr_sfb_offset_short[sfb + 1] >=
          (ptr_sfb_offset_long[pstr_igf_config->igf_start_sfb_lb] >> 3))
      {
        if (pstr_igf_config->igf_start_sfb_sb < 0)
        {
          pstr_igf_config->igf_start_sfb_sb = sfb + 1;
        }
      }
    }

    pstr_igf_config->igf_stop_sfb_sb = -1;
    for (WORD32 sfb = 0; sfb < num_sfb; sfb++)
    {
      if (ptr_sfb_offset_short[sfb + 1] >=
          (ptr_sfb_offset_long[pstr_igf_config->igf_stop_sfb_lb] >> 3))
      {
        if (pstr_igf_config->igf_stop_sfb_sb < 0)
        {
          pstr_igf_config->igf_stop_sfb_sb = sfb + 1;
        }
      }
    }

    pstr_igf_config->igf_stop_sfb_sb = min(pstr_igf_config->igf_stop_sfb_sb, sfb_active);
  }
  else
  {
    pstr_igf_config->igf_start_sfb_lb = sfb_bgn;
    pstr_igf_config->igf_stop_sfb_lb = sfb_end;

    pstr_igf_config->igf_stop_sfb_lb = min(pstr_igf_config->igf_stop_sfb_lb, sfb_active);

    if (pstr_igf_config->igf_start_sfb_lb <= num_sfb - 5)
    {
      pstr_igf_config->igf_start_index = ((pstr_igf_config->igf_start_sfb_lb - 11));
    }

    if (pstr_igf_config->igf_stop_sfb_lb == num_sfb)
    {
      pstr_igf_config->igf_stop_index = 15;
    }
    else
    {
      pstr_igf_config->igf_stop_index = (WORD32)(
          (WORD32)floor(
              (((pstr_igf_config->igf_stop_sfb_lb - pstr_igf_config->igf_start_sfb_lb) << 4) /
               (FLOAT32)(num_sfb - (pstr_igf_config->igf_start_sfb_lb + 1)))) -
          2);
    }

    pstr_igf_config->igf_stop_index = max(0, pstr_igf_config->igf_stop_index);

    pstr_igf_config->igf_stop_index = min(15, pstr_igf_config->igf_stop_index);

    if (pstr_igf_config->igf_stop_index != 15)
    {
      pstr_igf_config->igf_stop_sfb_lb =
          min(num_sfb, max(pstr_igf_config->igf_start_sfb_lb +
                               (((num_sfb - (pstr_igf_config->igf_start_sfb_lb + 1)) *
                                 (pstr_igf_config->igf_stop_index + 2)) >>
                                4),
                           pstr_igf_config->igf_start_sfb_lb + 1));
    }
  }

  return;
}
