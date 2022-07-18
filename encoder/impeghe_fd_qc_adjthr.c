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
#include <limits.h>
#include "impeghe_type_def.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_cnst.h"
#include "impeghe_rom.h"
#include "impeghe_bitbuffer.h"
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
#include "impeghe_fd_qc_adjthr.h"

#define CLIP_SAVE_LO_LONG (0.2f)
#define CLIP_SAVE_HI_LONG (0.95f)
#define MIN_BITS_SAVE_LONG (-0.05f)
#define MAX_BITS_SAVE_LONG (0.3f)
#define CLIP_SPEND_LO_LONG (0.2f)
#define CLIP_SPEND_HI_LONG (0.95f)
#define MIN_BITS_SPEND_LONG (-0.10f)
#define MAX_BITS_SPEND_LONG (0.4f)
#define CLIP_SAVE_LO_SHORT (0.2f)
#define CLIP_SAVE_HI_SHORT (0.75f)
#define MIN_BITS_SAVE_SHORT (0.0f)
#define MAX_BITS_SAVE_SHORT (0.2f)
#define CLIP_SPEND_LO_SHORT (0.2f)
#define CLIP_SPEND_HI_SHORT (0.75f)
#define MIN_BITS_SPEND_SHORT (-0.05f)
#define MAX_BITS_SPEND_SHORT (0.5f)
#define CLIP_SAVE_LO_TO_HI_LONG (CLIP_SAVE_HI_LONG - CLIP_SAVE_LO_LONG)
#define CLIP_SAVE_LO_TO_HI_SHORT (CLIP_SAVE_HI_SHORT - CLIP_SAVE_LO_SHORT)
#define CLIP_SPEND_LO_TO_HI_LONG (CLIP_SPEND_HI_LONG - CLIP_SPEND_LO_LONG)
#define CLIP_SPEND_LO_TO_HI_SHORT (CLIP_SPEND_HI_SHORT - CLIP_SPEND_LO_SHORT)
#define MIN_TO_MAX_SAVE_BITS_LONG (MAX_BITS_SAVE_LONG - MIN_BITS_SAVE_LONG)
#define MIN_TO_MAX_SAVE_BITS_SHORT (MAX_BITS_SAVE_SHORT - MIN_BITS_SAVE_SHORT)
#define MIN_TO_MAX_SPEND_BITS_LONG (MAX_BITS_SPEND_LONG - MIN_BITS_SPEND_LONG)
#define MIN_TO_MAX_SPEND_BITS_SHORT (MAX_BITS_SPEND_SHORT - MIN_BITS_SPEND_SHORT)
#define BITS_SAVE_RATIO_LONG (MIN_TO_MAX_SAVE_BITS_LONG / CLIP_SAVE_LO_TO_HI_LONG)
#define BITS_SAVE_RATIO_SHORT (MIN_TO_MAX_SAVE_BITS_SHORT / CLIP_SAVE_LO_TO_HI_SHORT)
#define BITS_SPEND_RATIO_LONG (MIN_TO_MAX_SPEND_BITS_LONG / CLIP_SPEND_LO_TO_HI_LONG)
#define BITS_SPEND_RATIO_SHORT (MIN_TO_MAX_SPEND_BITS_SHORT / CLIP_SPEND_LO_TO_HI_SHORT)

/**
 *  impeghe_bits_to_pe
 *
 *  \brief Convert bits into perceptual entropy
 *
 *  \param [in] bits   Number of average bits per channel and transform block
 *
 *  \return FLOAT32    Perceptual entropy equivalent of the input bits
 */
FLOAT32 impeghe_bits_to_pe(const FLOAT32 bits) { return (bits * 1.18f); }

/**
 *  impeghe_adj_thr_init
 *
 *  \brief Initialization of threshold adjustment module
 *
 *  \param [out] pstr_adj_thr_ele		Pointer to adjust threshold handle
 *  \param [in]  mean_pe				Mean value of perceptual entropy
 *  \param [in]  ch_bitrate				Channel bitrate
 *
 *  \return VOID
 */
VOID impeghe_adj_thr_init(ia_adj_thr_elem_struct *pstr_adj_thr_ele, const FLOAT32 mean_pe,
                          WORD32 ch_bitrate)
{
  ia_min_snr_adapt_param_struct *pstr_min_snr_params =
      &pstr_adj_thr_ele->str_min_snr_adapt_params;

  pstr_adj_thr_ele->pe_min = (FLOAT32)0.8f * mean_pe;
  pstr_adj_thr_ele->pe_max = (FLOAT32)1.2f * mean_pe;
  pstr_adj_thr_ele->pe_offset = 0.0f;

  if (ch_bitrate < 32000)
  {
    pstr_adj_thr_ele->pe_offset =
        max((FLOAT32)50.0f, (FLOAT32)(100.0f) - (FLOAT32)(100.0f / 32000) * (FLOAT32)ch_bitrate);
  }

  if (ch_bitrate > 20000)
  {
    pstr_adj_thr_ele->str_ah_param.modify_min_snr = TRUE;
    pstr_adj_thr_ele->str_ah_param.start_sfb_long = 15;
    pstr_adj_thr_ele->str_ah_param.start_sfb_short = 3;
  }
  else
  {
    pstr_adj_thr_ele->str_ah_param.modify_min_snr = FALSE;
    pstr_adj_thr_ele->str_ah_param.start_sfb_long = 0;
    pstr_adj_thr_ele->str_ah_param.start_sfb_short = 0;
  }

  pstr_min_snr_params->max_red = (FLOAT32)0.25f;

  pstr_min_snr_params->start_ratio = (FLOAT32)10.0f;

  pstr_min_snr_params->max_ratio = (FLOAT32)1000.0f;

  pstr_min_snr_params->red_ratio_fac =
      (1.0f - pstr_min_snr_params->max_red) /
      (10.0f * (FLOAT32)log10(pstr_min_snr_params->start_ratio / pstr_min_snr_params->max_ratio));

  pstr_min_snr_params->red_offs = 1.0f - pstr_min_snr_params->red_ratio_fac * 10.0f *
                                             (FLOAT32)log10(pstr_min_snr_params->start_ratio);

  pstr_adj_thr_ele->pe_last = (FLOAT32)0.0f;
  pstr_adj_thr_ele->dyn_bits_last = 0;
  pstr_adj_thr_ele->pe_correction_fac = (FLOAT32)1.0f;
}

/**
 *  impeghe_calc_sfb_pe_data
 *
 *  \brief Preparatory steps of scale factor band perceptual entropy calculation
 *
 *  \param [in,out] pstr_qc_pe_data   Pointer to perceptual entropy based quantization data
 * structure
 *  \param [in,out] pstr_psy_out	  Pointer to psychoacoustic model output data structure
 *  \param [in]     num_channels      Number of channels
 *  \param [in]     chn               Channel index
 *
 *  \return VOID
 */
static VOID impeghe_calc_sfb_pe_data(ia_qc_pe_data_struct *pstr_qc_pe_data,
                                     ia_psy_mod_out_data_struct *pstr_psy_out,
                                     WORD32 num_channels, WORD32 chn)
{
  LOOPIDX ch, idx = 0;
  WORD32 scf_band_grp;
  FLOAT32 num_lines;
  FLOAT32 ld_thr, ld_ratio;
  LOOPIDX i = 0, scf;
  WORD32 sfb_count;
  WORD32 scf_band_per_grp;
  WORD32 max_sfb_per_grp;
  FLOAT32 *ptr_sfb_energy;
  FLOAT32 *ptr_sfb_thr;
  ia_qc_pe_chan_data_struct *str_qc_pe_chan_data;

  pstr_qc_pe_data->pe = pstr_qc_pe_data->offset;
  pstr_qc_pe_data->const_part = 0.0f;
  pstr_qc_pe_data->num_active_lines = 0.0f;

  for (ch = chn; ch < chn + num_channels; ch++)
  {
    sfb_count = pstr_psy_out[ch].sfb_count;
    scf_band_per_grp = pstr_psy_out[ch].sfb_per_group;
    max_sfb_per_grp = pstr_psy_out[ch].max_sfb_per_grp;
    ptr_sfb_energy = pstr_psy_out[ch].ptr_sfb_energy;
    ptr_sfb_thr = pstr_psy_out[ch].ptr_sfb_thr;
    str_qc_pe_chan_data = &pstr_qc_pe_data->pe_ch_data[idx];
    str_qc_pe_chan_data->pe = 0;
    str_qc_pe_chan_data->num_active_lines = 0;
    str_qc_pe_chan_data->const_part = 0;

    for (scf_band_grp = 0; scf_band_grp < sfb_count; scf_band_grp += scf_band_per_grp)
    {
      i = scf_band_grp;
      for (scf = max_sfb_per_grp - 1; scf >= 0; scf--, i++)
      {
        if (ptr_sfb_energy[i] > ptr_sfb_thr[i])
        {
          ld_thr = (FLOAT32)log(ptr_sfb_thr[i]) * LOG2_1;
          ld_ratio = str_qc_pe_chan_data->sfb_ld_energy[i] - ld_thr;
          num_lines = str_qc_pe_chan_data->sfb_lines[i];
          if (ld_ratio < PE_C1)
          {
            str_qc_pe_chan_data->sfb_pe[i] = num_lines * (PE_C2 + PE_C3 * ld_ratio);
            str_qc_pe_chan_data->sfb_const_part[i] =
                num_lines * (PE_C2 + PE_C3 * str_qc_pe_chan_data->sfb_ld_energy[i]);
            num_lines = num_lines * PE_C3;
          }
          else
          {
            str_qc_pe_chan_data->sfb_pe[i] = num_lines * ld_ratio;
            str_qc_pe_chan_data->sfb_const_part[i] =
                num_lines * str_qc_pe_chan_data->sfb_ld_energy[i];
          }
          str_qc_pe_chan_data->num_sfb_active_lines[i] = num_lines;
        }
        else
        {
          str_qc_pe_chan_data->sfb_pe[i] = 0.0f;
          str_qc_pe_chan_data->sfb_const_part[i] = 0.0f;
          str_qc_pe_chan_data->num_sfb_active_lines[i] = 0.0;
        }

        str_qc_pe_chan_data->pe += str_qc_pe_chan_data->sfb_pe[i];
        str_qc_pe_chan_data->const_part += str_qc_pe_chan_data->sfb_const_part[i];
        str_qc_pe_chan_data->num_active_lines += str_qc_pe_chan_data->num_sfb_active_lines[i];
      }
    }
    pstr_qc_pe_data->pe += str_qc_pe_chan_data->pe;
    pstr_qc_pe_data->const_part += str_qc_pe_chan_data->const_part;
    pstr_qc_pe_data->num_active_lines += str_qc_pe_chan_data->num_active_lines;
    pstr_psy_out[ch].pe = pstr_qc_pe_data->pe;
    idx++;
  }
  return;
}

/**
 *  impeghe_adj_pe_minmax
 *
 *  \brief Adjust min-max value of perceptual entropy
 *
 *  \param [in]     curr_pe		Current value
 *  \param [in,out] pe_min		Minumum value
 *  \param [in,out] pe_max		Maximum value
 *
 *  \return VOID
 */
static VOID impeghe_adj_pe_minmax(const FLOAT32 curr_pe, FLOAT32 *pe_min, FLOAT32 *pe_max)
{
  FLOAT32 min_hi_fac = 0.3f, max_hi_fac = 1.0f, min_low_fac = 0.14f, max_low_fac = 0.07f;
  FLOAT32 diff;
  FLOAT32 min_diff = curr_pe * (FLOAT32)0.1666666667f;

  if (curr_pe <= *pe_max)
  {
    if (curr_pe >= *pe_min)
    {
      *pe_min += (curr_pe - *pe_min) * min_hi_fac;
      *pe_max -= (*pe_max - curr_pe) * max_low_fac;
    }
    else
    {
      diff = (*pe_min - curr_pe);
      *pe_min -= diff * min_low_fac;
      *pe_max -= diff * max_low_fac;
    }
  }
  else
  {
    diff = (curr_pe - *pe_max);
    *pe_min += diff * min_hi_fac;
    *pe_max += diff * max_hi_fac;
  }

  if (min_diff > (*pe_max - *pe_min))
  {
    FLOAT32 low_part, high_part;
    low_part = max((FLOAT32)0.0f, curr_pe - *pe_min);
    high_part = max((FLOAT32)0.0f, *pe_max - curr_pe);
    *pe_max = curr_pe + high_part / (low_part + high_part) * min_diff;
    *pe_min = curr_pe - low_part / (low_part + high_part) * min_diff;
    *pe_min = max((FLOAT32)0.0f, *pe_min);
  }

  return;
}

/**
 *  impeghe_bitres_calc_bitfac
 *
 *  \brief Calculate bit-factor related to bit-reservoir
 *
 *  \param [in] bitres_bits         Number of bits in the bit-reservoir
 *  \param [in] max_bitres_bits     Maximum number of bits in the bit-reservoir
 *  \param [in] pe                  Perceptual entropy
 *  \param [in] win_seq             Window sequence of current frame
 *  \param [in] avg_bits            Average number of bits per frame matching the given constant
 * bitrate
 *  \param [in] max_bit_fac         Maximum bit-factor
 *  \param [in] pstr_adj_thr_elem   Pointer to threshold adjustment element structure
 *
 *  \return FLOAT32                 Bit-factor indicating whether bits need to be put or taken
 * from bit-reservoir
 */

static FLOAT32 impeghe_bitres_calc_bitfac(const WORD32 bitres_bits, const WORD32 max_bitres_bits,
                                          const FLOAT32 pe, const WORD32 win_seq,
                                          const WORD32 avg_bits, const FLOAT32 max_bit_fac,
                                          ia_adj_thr_elem_struct *pstr_adj_thr_elem)
{
  FLOAT32 pex;
  FLOAT32 fill_lvl;
  FLOAT32 bit_save, bit_spend, bitres_factor;

  fill_lvl = (FLOAT32)bitres_bits / (FLOAT32)max_bitres_bits;

  if (win_seq != EIGHT_SHORT_SEQUENCE)
  {
    fill_lvl = max(fill_lvl, CLIP_SAVE_LO_LONG);
    fill_lvl = min(fill_lvl, CLIP_SAVE_HI_LONG);
    bit_save = MAX_BITS_SAVE_LONG - (BITS_SAVE_RATIO_LONG * (fill_lvl - CLIP_SAVE_LO_LONG));
    bit_spend = MIN_BITS_SPEND_LONG + (BITS_SPEND_RATIO_LONG * (fill_lvl - CLIP_SPEND_LO_LONG));
  }
  else
  {
    fill_lvl = max(fill_lvl, CLIP_SPEND_LO_SHORT);
    fill_lvl = min(fill_lvl, CLIP_SPEND_HI_SHORT);
    bit_save = MAX_BITS_SAVE_SHORT - (BITS_SAVE_RATIO_SHORT * (fill_lvl - CLIP_SAVE_LO_SHORT));
    bit_spend =
        MIN_BITS_SPEND_SHORT + (BITS_SPEND_RATIO_SHORT * (fill_lvl - CLIP_SPEND_LO_SHORT));
  }

  pex = max(pe, pstr_adj_thr_elem->pe_min);
  pex = min(pex, pstr_adj_thr_elem->pe_max);

  bitres_factor =
      (FLOAT32)1.0f - bit_save +
      ((bit_spend + bit_save) / (pstr_adj_thr_elem->pe_max - pstr_adj_thr_elem->pe_min)) *
          (pex - pstr_adj_thr_elem->pe_min);
  bitres_factor = min(bitres_factor,
                      (FLOAT32)1.0f - (FLOAT32)0.3f + (FLOAT32)bitres_bits / (FLOAT32)avg_bits);

  bitres_factor = min(bitres_factor, max_bit_fac);
  impeghe_adj_pe_minmax(pe, &pstr_adj_thr_elem->pe_min, &pstr_adj_thr_elem->pe_max);

  return bitres_factor;
}

/**
 *  impeghe_calc_pe_correction
 *
 *  \brief Compensate for systematic differences by applying a correction factor to the actual
 * perceptual entropy
 *
 *  \param [in,out] correction_fac	 Pointer to correction factor for perceptual entropy
 *  \param [in]     pe_act               Actual perceptual entropy
 *  \param [in]     pe_last              Perceptual entropy of previous frame
 *  \param [in]     bits_prev            Dynamic bits of previous frame
 *
 *  \return VOID
 */
static VOID impeghe_calc_pe_correction(FLOAT32 *correction_fac, const FLOAT32 pe_act,
                                       const FLOAT32 pe_last, const WORD32 bits_prev)
{
  if ((bits_prev > 0) && (pe_act < (FLOAT32)1.5f * pe_last) &&
      (pe_act > (FLOAT32)0.7f * pe_last) &&
      ((FLOAT32)1.2f * impeghe_bits_to_pe((FLOAT32)bits_prev) > pe_last) &&
      ((FLOAT32)0.65f * impeghe_bits_to_pe((FLOAT32)bits_prev) < pe_last))
  {
    FLOAT32 new_fac = pe_last / impeghe_bits_to_pe((FLOAT32)bits_prev);

    if (new_fac >= (FLOAT32)1.0f)
    {
      new_fac = max((FLOAT32)0.9f * new_fac, (FLOAT32)1.0f);
      new_fac = min(new_fac, (FLOAT32)1.15f);
    }
    else
    {
      new_fac = min((FLOAT32)1.1f * new_fac, (FLOAT32)1.0f);
      new_fac = max(new_fac, (FLOAT32)0.85f);
    }
    if (((new_fac > (FLOAT32)1.0f) && (*correction_fac < (FLOAT32)1.0f)) ||
        ((new_fac < (FLOAT32)1.0f) && (*correction_fac > (FLOAT32)1.0f)))
    {
      *correction_fac = (FLOAT32)1.0f;
    }

    if ((*correction_fac < (FLOAT32)1.0f && new_fac < *correction_fac) ||
        (*correction_fac > (FLOAT32)1.0f && new_fac > *correction_fac))
      *correction_fac = (FLOAT32)0.85f * (*correction_fac) + (FLOAT32)0.15f * new_fac;
    else
      *correction_fac = (FLOAT32)0.7f * (*correction_fac) + (FLOAT32)0.3f * new_fac;

    *correction_fac = min(*correction_fac, (FLOAT32)1.15f);
    *correction_fac = max(*correction_fac, (FLOAT32)0.85f);
  }
  else
  {
    *correction_fac = (FLOAT32)1.0f;
  }

  return;
}

/**
 *  impeghe_calc_thr_exp
 *
 *  \brief Compute the 1/4th power of the input threshold values
 *
 *  \param [out] pptr_thr_exp       Output modified threshold values per scalefactor band
 *  \param [in]  pstr_psy_out  Pointer to the psychoacoustic model output data structure
 *  \param [in]  num_chans     Number of channels
 *  \param [in]  chn           Channel index
 *
 *  \return VOID
 */
static VOID impeghe_calc_thr_exp(FLOAT32 **pptr_thr_exp, ia_psy_mod_out_data_struct *pstr_psy_out,
                                 WORD32 num_chans, WORD32 chn)
{
  LOOPIDX sfb, ch, scf_band_grp, idx = 0;
  ia_psy_mod_out_data_struct *pstr_psy_chan_out;
  FLOAT32 *scf_band_thr;
  FLOAT32 *ptr_thr_exp;
  for (ch = chn; ch < chn + num_chans; ch++)
  {
    pstr_psy_chan_out = &pstr_psy_out[ch];
    scf_band_thr = pstr_psy_chan_out->ptr_sfb_thr;
    ptr_thr_exp = pptr_thr_exp[idx];
    for (scf_band_grp = 0; scf_band_grp < pstr_psy_chan_out->sfb_count;
         scf_band_grp += pstr_psy_chan_out->sfb_per_group)
    {
      FLOAT32 *thr_exp = &ptr_thr_exp[scf_band_grp];
      scf_band_thr = &pstr_psy_chan_out->ptr_sfb_thr[scf_band_grp];
      for (sfb = 0; sfb < pstr_psy_chan_out->max_sfb_per_grp; sfb++)
      {
        thr_exp[sfb] = (FLOAT32)pow(*scf_band_thr++, RED_EXP_VAL);
      }
    }
    idx++;
  }

  return;
}

/**
 *  impeghe_adapt_min_snr
 *
 *  \brief Perform signal dependent modifications of the minimum Signal to Noise Ratio
 *
 *  \param [in,out] pstr_psy_out         Pointer to psychoacoustic model output data structure
 *  \param [in]     pstr_min_snr_params  Pointer to minimum SNR adaptation parameter structure
 *  \param [in]     num_chans            Number of channels
 *  \param [in]     chn                  Channel index
 *
 *  \return VOID
 */
static VOID impeghe_adapt_min_snr(ia_psy_mod_out_data_struct *pstr_psy_out,
                                  ia_min_snr_adapt_param_struct *pstr_min_snr_params,
                                  WORD32 num_chans, WORD32 chn)
{
  WORD32 num_sfb = 0, scf_band_cnt;
  FLOAT32 avg_energy = 0.0f, db_ratio, min_snr_red;
  LOOPIDX i, ch, sfb_off, sfb;
  for (ch = chn; ch < chn + num_chans; ch++)
  {
    ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];

    num_sfb = 0;
    avg_energy = 0;
    scf_band_cnt = pstr_psy_chan_out->max_sfb_per_grp;

    for (sfb_off = 0; sfb_off < pstr_psy_chan_out->sfb_count;
         sfb_off += pstr_psy_chan_out->sfb_per_group)
    {

      FLOAT32 *sfb_energy = &pstr_psy_chan_out->ptr_sfb_energy[sfb_off];
      for (sfb = scf_band_cnt - 1; sfb >= 0; sfb--)
      {
        avg_energy += sfb_energy[sfb];
      }
      num_sfb += scf_band_cnt;
    }

    if (num_sfb > 0)
    {
      avg_energy /= num_sfb;
    }

    for (sfb_off = 0; sfb_off < pstr_psy_chan_out->sfb_count;
         sfb_off += pstr_psy_chan_out->sfb_per_group)
    {
      i = sfb_off;
      for (sfb = scf_band_cnt - 1; sfb >= 0; sfb--, i++)
      {
        if (pstr_min_snr_params->start_ratio * pstr_psy_chan_out->ptr_sfb_energy[i] < avg_energy)
        {
          db_ratio =
              (FLOAT32)(10.0 * log10((MIN_FLT_VAL + avg_energy) /
                                     (MIN_FLT_VAL + pstr_psy_chan_out->ptr_sfb_energy[i])));
          min_snr_red =
              pstr_min_snr_params->red_offs + pstr_min_snr_params->red_ratio_fac * db_ratio;
          min_snr_red = max(min_snr_red, pstr_min_snr_params->max_red);
          pstr_psy_chan_out->sfb_min_snr[i] =
              (FLOAT32)pow(pstr_psy_out[ch].sfb_min_snr[i], min_snr_red);
          pstr_psy_chan_out->sfb_min_snr[i] = min(MIN_SNR_LIMIT, pstr_psy_out[ch].sfb_min_snr[i]);
        }
      }
    }
  }

  return;
}

/**
 *  impeghe_init_avoid_hole_flag
 *
 *  \brief Decide which scalefactor bands need avoidance of spectral holes
 *
 *  \param [out] pptr_ah_flag         Flag per scalefactor band indicating whether spectral hole
 * avoidance algorithm is active or not
 *  \param [in]  pstr_psy_out     Pointer to psychoacoustic model output data structure
 *  \param [in]  pstr_ah_param    Pointer to avoidance of holes algorithm parameter structure
 *  \param [in]  num_chans        Number of channels
 *  \param [in]  chn              Channel index
 *
 *  \return VOID
 */
static VOID impeghe_init_avoid_hole_flag(WORD32 **pptr_ah_flag,
                                         ia_psy_mod_out_data_struct *pstr_psy_out,
                                         ia_ah_param_struct *pstr_ah_param, WORD32 num_chans,
                                         WORD32 chn)
{
  LOOPIDX ch, idx;
  FLOAT32 sfb_energy;
  FLOAT32 scale_spread_energy;
  WORD32 scf_band_grp, sfb, scf_band;
  FLOAT32 *ptr_scf_band_spread_energy, *ptr_scf_band_energy, *ptr_scf_band_min_snr;
  for (ch = chn; ch < chn + num_chans; ch++)
  {
    ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];

    if (pstr_psy_chan_out->window_sequence == EIGHT_SHORT_SEQUENCE)
    {
      scale_spread_energy = 0.63f;
    }
    else
    {
      scale_spread_energy = 0.5f;
    }

    for (scf_band_grp = 0; scf_band_grp < pstr_psy_chan_out->sfb_count;
         scf_band_grp += pstr_psy_chan_out->sfb_per_group)
    {
      ptr_scf_band_spread_energy = &pstr_psy_chan_out->ptr_sfb_spread_energy[scf_band_grp];
      ptr_scf_band_energy = &pstr_psy_chan_out->ptr_sfb_energy[scf_band_grp];
      sfb = pstr_psy_chan_out->max_sfb_per_grp;
      for (scf_band = sfb - 1; scf_band >= 0; scf_band--)
      {
        *ptr_scf_band_spread_energy = *ptr_scf_band_spread_energy * scale_spread_energy;
        ptr_scf_band_spread_energy++;
      }
    }
  }

  if (pstr_ah_param->modify_min_snr)
  {
    for (ch = chn; ch < chn + num_chans; ch++)
    {
      ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
      ptr_scf_band_spread_energy = pstr_psy_chan_out->ptr_sfb_spread_energy;
      ptr_scf_band_energy = pstr_psy_chan_out->ptr_sfb_energy;

      ptr_scf_band_min_snr = pstr_psy_chan_out->sfb_min_snr;

      for (scf_band_grp = 0; scf_band_grp < pstr_psy_chan_out->sfb_count;
           scf_band_grp += pstr_psy_chan_out->sfb_per_group)
      {

        for (scf_band = 0; scf_band < pstr_psy_chan_out->max_sfb_per_grp; scf_band++)
        {
          FLOAT32 sfb_en_m1, sfb_en_p1, avg_energy;
          if (scf_band > 0)
          {
            sfb_en_m1 = ptr_scf_band_energy[scf_band_grp + scf_band - 1];
          }
          else
          {
            sfb_en_m1 = ptr_scf_band_energy[scf_band_grp];
          }
          if (scf_band < pstr_psy_chan_out->max_sfb_per_grp - 1)
            sfb_en_p1 = ptr_scf_band_energy[scf_band_grp + scf_band + 1];
          else
            sfb_en_p1 = ptr_scf_band_energy[scf_band_grp + scf_band];

          avg_energy = (sfb_en_m1 + sfb_en_p1) / (FLOAT32)2.0f;
          sfb_energy = ptr_scf_band_energy[scf_band_grp + scf_band];

          if (sfb_energy > avg_energy)
          {
            FLOAT32 temp_min_snr = max((FLOAT32)0.8f * avg_energy / sfb_energy, (FLOAT32)0.316f);
            if (pstr_psy_chan_out->window_sequence != EIGHT_SHORT_SEQUENCE)
              temp_min_snr = max(temp_min_snr, (FLOAT32)0.316f);
            else
              temp_min_snr = max(temp_min_snr, (FLOAT32)0.5f);
            ptr_scf_band_min_snr[scf_band_grp + scf_band] =
                min(ptr_scf_band_min_snr[scf_band_grp + scf_band], temp_min_snr);
          }

          if (((FLOAT32)2.0f * sfb_energy < avg_energy) && (sfb_energy > (FLOAT32)0.0f))
          {
            FLOAT32 temp_min_snr = avg_energy / ((FLOAT32)2.0f * sfb_energy) *
                                   ptr_scf_band_min_snr[scf_band_grp + scf_band];
            temp_min_snr = min((FLOAT32)0.8f, temp_min_snr);
            ptr_scf_band_min_snr[scf_band_grp + scf_band] =
                min(temp_min_snr, ptr_scf_band_min_snr[scf_band_grp + scf_band] * (FLOAT32)3.16f);
          }
        }
      }
    }
  }

  if (num_chans == 2)
  {
    ia_psy_mod_out_data_struct *psy_out_mid = &pstr_psy_out[chn];
    ia_psy_mod_out_data_struct *psy_out_side = &pstr_psy_out[chn + 1];

    for (WORD32 sfb = 0; sfb < psy_out_mid->sfb_count; sfb++)
    {

      if (pstr_psy_out[chn].ms_used[sfb])
      {
        FLOAT32 sfb_en_mid = psy_out_mid->ptr_sfb_energy[sfb];
        FLOAT32 sfb_en_side = psy_out_side->ptr_sfb_energy[sfb];
        FLOAT32 max_sfb_en = max(sfb_en_mid, sfb_en_side);
        FLOAT32 max_thr = 0.25f * psy_out_mid->sfb_min_snr[sfb] * max_sfb_en;

        psy_out_mid->sfb_min_snr[sfb] = (FLOAT32)max(
            psy_out_mid->sfb_min_snr[sfb],
            min(MAX_FLT_VAL, ((double)max_thr / (MIN_FLT_VAL + (double)sfb_en_mid))));

        if (psy_out_mid->ptr_sfb_energy[sfb] <= 1.0f)
        {

          psy_out_mid->ptr_sfb_energy[sfb] = min(psy_out_mid->ptr_sfb_energy[sfb], 0.8f);
        }

        psy_out_side->sfb_min_snr[sfb] = (FLOAT32)max(
            psy_out_side->sfb_min_snr[sfb],
            min(MAX_FLT_VAL, ((double)max_thr / (MIN_FLT_VAL + (double)sfb_en_side))));

        if (psy_out_side->sfb_min_snr[sfb] <= 1.0f)
        {

          psy_out_side->sfb_min_snr[sfb] = min(psy_out_side->sfb_min_snr[sfb], 0.8f);
        }

        if (sfb_en_mid > psy_out_mid->ptr_sfb_spread_energy[sfb])
        {

          psy_out_side->ptr_sfb_spread_energy[sfb] = 0.9f * sfb_en_side;
        }

        if (sfb_en_side > psy_out_side->ptr_sfb_spread_energy[sfb])
        {

          psy_out_mid->ptr_sfb_spread_energy[sfb] = 0.9f * sfb_en_mid;
        }
      }
    }
  }
  idx = 0;
  for (ch = chn; ch < chn + num_chans; ch++)
  {
    ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
    for (scf_band_grp = 0; scf_band_grp < pstr_psy_chan_out->sfb_count;
         scf_band_grp += pstr_psy_chan_out->sfb_per_group)
    {

      for (scf_band = 0; scf_band < pstr_psy_chan_out->max_sfb_per_grp; scf_band++)
      {

        if (pstr_psy_chan_out->ptr_sfb_spread_energy[scf_band_grp + scf_band] >
                pstr_psy_chan_out->ptr_sfb_energy[scf_band_grp + scf_band] ||
            pstr_psy_chan_out->sfb_min_snr[scf_band_grp + scf_band] > (float)1.0)
        {

          pptr_ah_flag[idx][scf_band_grp + scf_band] = NO_AH;
        }
        else
        {

          pptr_ah_flag[idx][scf_band_grp + scf_band] = AH_INACTIVE;
        }
      }

      for (scf_band = pstr_psy_chan_out->max_sfb_per_grp;
           scf_band < pstr_psy_chan_out->sfb_per_group; scf_band++)
      {
        pptr_ah_flag[idx][scf_band_grp + scf_band] = NO_AH;
      }
    }
    idx++;
  }

  return;
}

/**
 *  impeghe_reduce_thr
 *
 *  \brief Reduce masking thresholds based on the reduction value and avoidance of spectral holes
 *
 *  \param [in,out] pstr_psy_out  Pointer to psychoacoustic model output data structure
 *  \param [in]     pptr_ah_flag       Flag per scalefactor band indicating whether spectral hole
 * avoidance algorithm is active or not
 *  \param [in]     pptr_thr_exp       Modified threshold values per scalefactor band
 *  \param [in]     red_value     Reduction value
 *  \param [in]     num_channels  Number of channels
 *  \param [in]     chn           Channel index
 *
 *  \return VOID
 */
static VOID impeghe_reduce_thr(ia_psy_mod_out_data_struct *pstr_psy_out, WORD32 **pptr_ah_flag,
                               FLOAT32 **pptr_thr_exp, const FLOAT32 red_value,
                               WORD32 num_channels, WORD32 chn)
{
  LOOPIDX ch, sfb_group, sfb;
  WORD32 idx = 0;
  FLOAT32 sfb_energy, sfb_threshold, sfb_thr_reduced;
  FLOAT32 *sfb_energy_fix, *sfb_threshold_fix, *sfb_min_snr_fix, *thr_exp_fix;
  for (ch = chn; ch < chn + num_channels; ch++)
  {
    ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
    sfb_energy_fix = pstr_psy_chan_out->ptr_sfb_energy;
    sfb_threshold_fix = pstr_psy_chan_out->ptr_sfb_thr;
    sfb_min_snr_fix = pstr_psy_chan_out->sfb_min_snr;
    thr_exp_fix = &pptr_thr_exp[idx][0];
    for (sfb_group = 0; sfb_group < pstr_psy_chan_out->sfb_count;
         sfb_group += pstr_psy_chan_out->sfb_per_group)
    {
      for (sfb = 0; sfb < pstr_psy_chan_out->max_sfb_per_grp; sfb++)
      {
        sfb_energy = sfb_energy_fix[sfb_group + sfb];
        sfb_threshold = sfb_threshold_fix[sfb_group + sfb];
        if (sfb_energy > sfb_threshold)
        {
          sfb_thr_reduced =
              (FLOAT32)pow((thr_exp_fix[sfb_group + sfb] + red_value), INV_RED_EXP_VAL);

          if ((sfb_thr_reduced > sfb_min_snr_fix[sfb_group + sfb] * sfb_energy) &&
              (pptr_ah_flag[idx][sfb_group + sfb] != NO_AH))
          {
            sfb_thr_reduced = max(sfb_min_snr_fix[sfb_group + sfb] * sfb_energy, sfb_threshold);
            pptr_ah_flag[idx][sfb_group + sfb] = AH_ACTIVE;
          }
          sfb_threshold_fix[sfb_group + sfb] = sfb_thr_reduced;
        }
      }
    }
    idx++;
  }

  return;
}

/**
 *  impeghe_calc_pe_no_active_holes
 *
 *  \brief Modification of perceptual entropy considering bands that do not avoid holes
 *
 *  \param [out] pe                Modified perceptual entropy
 *  \param [out] const_part        Constant part of the modified perceptual entropy
 *  \param [out] nactive_lines     Individual bands that actually do not avoid holes
 *  \param [in]  pstr_qs_pe_data   Pointer to perceptual entropy based quantization data structure
 *  \param [in]     pptr_ah_flag       Flag per scalefactor band indicating whether spectral hole
 * avoidance algorithm is active or not
 *  \param [in]  pstr_psy_out      Pointer to psychoacoustic model output data structure
 *  \param [in]  num_channels      Number of channels
 *  \param [in]  chn               Channel index
 *
 *  \return VOID
 */
static VOID impeghe_calc_pe_no_active_holes(FLOAT32 *pe, FLOAT32 *const_part,
                                            FLOAT32 *nactive_lines,
                                            ia_qc_pe_data_struct *pstr_qs_pe_data,
                                            WORD32 **pptr_ah_flag,
                                            ia_psy_mod_out_data_struct *pstr_psy_out,
                                            WORD32 num_channels, WORD32 chn)
{
  LOOPIDX ch, sfb_group, sfb;
  WORD32 idx = 0;
  *pe = 0.0f;
  *const_part = 0.0f;
  *nactive_lines = 0;
  for (ch = chn; ch < chn + num_channels; ch++)
  {
    ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
    ia_qc_pe_chan_data_struct *pe_channel_data = &pstr_qs_pe_data->pe_ch_data[idx];

    for (sfb_group = 0; sfb_group < pstr_psy_chan_out->sfb_count;
         sfb_group += pstr_psy_chan_out->sfb_per_group)
    {
      for (sfb = 0; sfb < pstr_psy_chan_out->max_sfb_per_grp; sfb++)
      {
        if (pptr_ah_flag[idx][sfb_group + sfb] < AH_ACTIVE)
        {
          *pe += pe_channel_data->sfb_pe[sfb_group + sfb];
          *const_part += pe_channel_data->sfb_const_part[sfb_group + sfb];
          *nactive_lines += pe_channel_data->num_sfb_active_lines[sfb_group + sfb];
        }
      }
    }
    idx++;
  }

  return;
}

/**
 *  impeghe_correct_thr
 *
 *  \brief Performs final threshold modification by linearization
 *
 *  \param [in,out] pstr_psy_out  	  Pointer to psychoacoustic model output data structure
 *  \param [in]     ah_flag           Flag per scalefactor band indicating whether spectral hole
 * avoidance algorithm is active or not
 *  \param [in]     pstr_qs_pe_data   Pointer to perceptual entropy based quantization data
 * structure
 *  \param [in]     thr_exp           Modified threshold values per scalefactor band
 *  \param [in]     red_value         Reduction value
 *  \param [in]     delta_pe          Difference in perceptual entropies
 *  \param [in]     num_channels      Number of channels
 *  \param [in]     chn               Channel index
 *  \param [in]     ptr_scratch       Pointer to scratch buffer
 *
 *  \return VOID
 */
static VOID impeghe_correct_thr(ia_psy_mod_out_data_struct *pstr_psy_out, WORD32 **ah_flag,
                                ia_qc_pe_data_struct *pstr_qs_pe_data, FLOAT32 **thr_exp,
                                const FLOAT32 red_value, const FLOAT32 delta_pe,
                                WORD32 num_channels, WORD32 chn, UWORD8 *ptr_scratch)
{
  WORD32 i, ch, sfb_group, sfb, idx = 0;
  FLOAT32 delta_sfb_pe;
  FLOAT32 thr_factor;
  FLOAT32 norm_factor[2] = {0};
  FLOAT32 *sfb_pe_factors[2];
  for (WORD32 i = 0; i < 2; i++)
  {
    sfb_pe_factors[i] = (FLOAT32 *)ptr_scratch;
    ptr_scratch += (MAX_NUM_GROUPED_SFB) * sizeof(sfb_pe_factors[0][0]);
  }
  FLOAT32 sfb_en, sfb_thr, sfb_thr_reduced;
  FLOAT32 *p_thr_exp;
  FLOAT32 *p_sfb_energy, *p_sfb_thr, *p_sfb_min_snr;
  ia_psy_mod_out_data_struct *pstr_psy_chan_out = NULL;
  ia_qc_pe_chan_data_struct *pe_channel_data = NULL;

  for (ch = chn; ch < chn + num_channels; ch++)
  {
    pstr_psy_chan_out = &pstr_psy_out[ch];
    pe_channel_data = &pstr_qs_pe_data->pe_ch_data[idx];
    norm_factor[idx] = MIN_FLT_VAL;
    p_thr_exp = thr_exp[idx];

    for (sfb_group = 0; sfb_group < pstr_psy_chan_out->sfb_count;
         sfb_group += pstr_psy_chan_out->sfb_per_group)
    {
      for (sfb = 0; sfb < pstr_psy_chan_out->max_sfb_per_grp; sfb++)
      {
        if ((ah_flag[idx][sfb_group + sfb] < AH_ACTIVE) || (delta_pe > 0))
        {
          sfb_pe_factors[idx][sfb_group + sfb] =
              pe_channel_data->num_sfb_active_lines[sfb_group + sfb] /
              (p_thr_exp[sfb_group + sfb] + red_value);
          norm_factor[idx] += sfb_pe_factors[idx][sfb_group + sfb];
        }
        else
        {
          sfb_pe_factors[idx][sfb_group + sfb] = 0.0f;
        }
      }
    }
    idx++;
  }
  if (num_channels > 1)
  {
    norm_factor[0] = norm_factor[0] + norm_factor[1];
  }
  norm_factor[0] = 1.0f / norm_factor[0];
  idx = 0;
  for (ch = chn; ch < chn + num_channels; ch++)
  {
    pstr_psy_chan_out = &pstr_psy_out[ch];
    pe_channel_data = &pstr_qs_pe_data->pe_ch_data[idx];
    p_sfb_energy = pstr_psy_chan_out->ptr_sfb_energy;
    p_sfb_thr = pstr_psy_chan_out->ptr_sfb_thr;
    p_sfb_min_snr = pstr_psy_chan_out->sfb_min_snr;

    for (sfb_group = 0; sfb_group < pstr_psy_chan_out->sfb_count;
         sfb_group += pstr_psy_chan_out->sfb_per_group)
    {
      i = sfb_group;
      for (sfb = pstr_psy_chan_out->max_sfb_per_grp - 1; sfb >= 0; sfb--, i++)
      {
        delta_sfb_pe = sfb_pe_factors[idx][i] * norm_factor[0] * delta_pe;
        if (pe_channel_data->num_sfb_active_lines[i] > (FLOAT32)0.5f)
        {
          sfb_en = p_sfb_energy[i];
          sfb_thr = p_sfb_thr[i];
          thr_factor = min(-delta_sfb_pe / pe_channel_data->num_sfb_active_lines[i], 20.f);
          thr_factor = (FLOAT32)pow(2.0f, thr_factor);
          sfb_thr_reduced = sfb_thr * thr_factor;

          if ((sfb_thr_reduced > p_sfb_min_snr[i] * sfb_en) && (ah_flag[idx][i] == AH_INACTIVE))
          {
            sfb_thr_reduced = max(p_sfb_min_snr[i] * sfb_en, sfb_thr);
            ah_flag[idx][i] = AH_ACTIVE;
          }
          p_sfb_thr[i] = sfb_thr_reduced;
        }
      }
    }
    idx++;
  }

  return;
}

/**
 *  impeghe_reduce_min_snr
 *
 *  \brief Adjust perceptual entropy so that it reaches desired entropy based on linearization of
 * algorithms
 *
 *  \param [in,out] pstr_psy_out      Pyschoacoustic model output data structure
 *  \param [in,out] pstr_qs_pe_data   Pointer to perceptual entropy based quantization data
 * structure
 *  \param [in] 	ah_flag           Flag per scalefactor band indicating whether spectral
 * hole
 * avoidance algorithm is active or not
 *  \param [in] 	desired_pe        Desired value of perceptual entropy
 *  \param [in] 	num_channels      Number of channels
 *  \param [in] 	chn               Channel index
 *
 *  \return VOID
 */
static VOID impeghe_reduce_min_snr(ia_psy_mod_out_data_struct *pstr_psy_out,
                                   ia_qc_pe_data_struct *pstr_qs_pe_data, WORD32 **ah_flag,
                                   const FLOAT32 desired_pe, WORD32 num_channels, WORD32 chn)
{
  WORD32 sfb, sfb_sub_win, ch, idx;
  FLOAT32 delta_pe;

  sfb_sub_win = pstr_psy_out[chn].max_sfb_per_grp;

  while (pstr_qs_pe_data->pe > desired_pe && sfb_sub_win > 0)
  {
    sfb_sub_win--;
    for (sfb = sfb_sub_win; sfb < pstr_psy_out[chn].sfb_count;
         sfb += pstr_psy_out[chn].sfb_per_group)
    {
      idx = 0;
      for (ch = chn; ch < chn + num_channels; ch++)
      {
        if (ah_flag[idx][sfb] != NO_AH && pstr_psy_out[ch].sfb_min_snr[sfb] < MIN_SNR_LIMIT)
        {
          pstr_psy_out[ch].sfb_min_snr[sfb] = MIN_SNR_LIMIT;
          pstr_psy_out[ch].ptr_sfb_thr[sfb] =
              pstr_psy_out[ch].ptr_sfb_energy[sfb] * pstr_psy_out[ch].sfb_min_snr[sfb];
          delta_pe = pstr_qs_pe_data->pe_ch_data[idx].sfb_lines[sfb] * 1.5f -
                     pstr_qs_pe_data->pe_ch_data[idx].sfb_pe[sfb];
          pstr_qs_pe_data->pe += delta_pe;
          pstr_qs_pe_data->pe_ch_data[idx].pe += delta_pe;
        }
        idx++;
      }
      if (pstr_qs_pe_data->pe <= desired_pe)
        break;
    }
  }

  return;
}

/**
 *  impeghe_allow_more_holes
 *
 *  \brief Relax constraints by perceptual energy reduction
 *
 *  \param  [in,out] pstr_psy_out      Pyschoacoustic model output data structure
 *  \param [in,out] pstr_qs_pe_data   Pointer to perceptual entropy based quantization data
 * structure
 *  \param [in,out] ah_flag           Flag per scalefactor band indicating whether spectral hole
 * avoidance algorithm is active or not
 *  \param [in]     str_ah_param      Pointer to avoidance of holes algorithm parameter structure
 *  \param [in]     desired_pe        Desired perceptual entropy
 *  \param [in]     num_channels      Number of channels
 *  \param [in]     chn               Channel index
 *  \param [in]     ptr_scratch       Pointer to scratch buffer
 *
 *  \return VOID
 */
static VOID impeghe_allow_more_holes(ia_psy_mod_out_data_struct *pstr_psy_out,
                                     ia_qc_pe_data_struct *pstr_qs_pe_data, WORD32 **ah_flag,
                                     const ia_ah_param_struct *str_ah_param,
                                     const FLOAT32 desired_pe, WORD32 num_channels, WORD32 chn,
                                     UWORD8 *ptr_scratch)
{
  WORD32 sfb, ch, idx;
  FLOAT32 act_pe = pstr_qs_pe_data->pe;

  if (num_channels == 2 &&
      pstr_psy_out[chn].window_sequence == pstr_psy_out[chn + 1].window_sequence)
  {
    ia_psy_mod_out_data_struct *psy_out_left = &pstr_psy_out[chn];
    ia_psy_mod_out_data_struct *psy_out_right = &pstr_psy_out[chn + 1];

    for (sfb = 0; sfb < psy_out_left->sfb_count; sfb++)
    {

      if (pstr_psy_out[chn].ms_used[sfb])
      {

        if (ah_flag[1][sfb] != NO_AH &&
            0.4f * psy_out_left->sfb_min_snr[sfb] * psy_out_left->ptr_sfb_energy[sfb] >
                psy_out_right->ptr_sfb_energy[sfb])
        {

          ah_flag[1][sfb] = NO_AH;

          psy_out_right->ptr_sfb_thr[sfb] = 2.0f * psy_out_right->ptr_sfb_energy[sfb];

          act_pe -= pstr_qs_pe_data->pe_ch_data[1].sfb_pe[sfb];
        }
        else
        {
          if (ah_flag[0][sfb] != NO_AH &&
              0.4f * psy_out_right->sfb_min_snr[sfb] * psy_out_right->ptr_sfb_energy[sfb] >
                  psy_out_left->ptr_sfb_energy[sfb])
          {

            ah_flag[0][sfb] = NO_AH;

            psy_out_left->ptr_sfb_thr[sfb] = 2.0f * psy_out_left->ptr_sfb_energy[sfb];

            act_pe -= pstr_qs_pe_data->pe_ch_data[0].sfb_pe[sfb];
          }
        }
        if (act_pe < desired_pe)
          break;
      }
    }
  }
  if (act_pe > desired_pe)
  {
    WORD32 *start_sfb = (WORD32 *)ptr_scratch;
    memset(start_sfb, 0, MAX_TIME_CHANNELS * sizeof(start_sfb[0]));
    FLOAT32 average_energy, min_energy;
    WORD32 ah_cnt;
    WORD32 en_idx;
    FLOAT32 energy[4];
    WORD32 min_sfb, max_sfb;
    WORD32 done;
    for (ch = chn; ch < chn + num_channels; ch++)
    {
      if (pstr_psy_out[ch].window_sequence != EIGHT_SHORT_SEQUENCE)
        start_sfb[ch] = str_ah_param->start_sfb_long;
      else
        start_sfb[ch] = str_ah_param->start_sfb_short;
    }

    average_energy = 0.0f;
    min_energy = MAX_FLT_VAL;
    ah_cnt = 0;
    idx = 0;
    for (ch = chn; ch < chn + num_channels; ch++)
    {
      ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
      for (sfb = start_sfb[ch]; sfb < pstr_psy_chan_out->sfb_count; sfb++)
      {
        if ((ah_flag[idx][sfb] != NO_AH) &&
            (pstr_psy_chan_out->ptr_sfb_energy[sfb] > pstr_psy_chan_out->ptr_sfb_thr[sfb]))
        {
          min_energy = min(min_energy, pstr_psy_chan_out->ptr_sfb_energy[sfb]);
          average_energy += pstr_psy_chan_out->ptr_sfb_energy[sfb];
          ah_cnt++;
        }
      }
      idx++;
    }

    average_energy = min(MAX_FLT_VAL, average_energy / (ah_cnt + MIN_FLT_VAL));

    for (en_idx = 0; en_idx < 4; en_idx++)
    {
      energy[en_idx] = min_energy * (FLOAT32)pow(average_energy / (min_energy + MIN_FLT_VAL),
                                                 (2 * en_idx + 1) / 7.0f);
    }
    max_sfb = pstr_psy_out[chn].sfb_count - 1;
    min_sfb = start_sfb[chn];

    if (num_channels == 2)
    {

      max_sfb = max(max_sfb, pstr_psy_out[chn + 1].sfb_count - 1);

      min_sfb = min(min_sfb, start_sfb[chn + 1]);
    }

    sfb = max_sfb;
    en_idx = 0;
    done = 0;
    while (!done)
    {
      idx = 0;
      for (ch = chn; ch < chn + num_channels; ch++)
      {
        ia_psy_mod_out_data_struct *pstr_psy_chan_out = &pstr_psy_out[ch];
        if (sfb >= start_sfb[ch] && sfb < pstr_psy_chan_out->sfb_count)
        {
          if (ah_flag[idx][sfb] != NO_AH &&
              pstr_psy_chan_out->ptr_sfb_energy[sfb] < energy[en_idx])
          {
            ah_flag[idx][sfb] = NO_AH;
            pstr_psy_chan_out->ptr_sfb_thr[sfb] = 2.0f * pstr_psy_chan_out->ptr_sfb_energy[sfb];
            act_pe -= pstr_qs_pe_data->pe_ch_data[idx].sfb_pe[sfb];
          }

          if (act_pe < desired_pe)
          {
            done = 1;
            break;
          }
        }
        idx++;
      }
      sfb--;
      if (sfb < min_sfb)
      {
        sfb = max_sfb;
        en_idx++;
        if (en_idx >= 4)
        {
          done = 1;
        }
      }
    }
  }

  return;
}

/**
 *  impeghe_adapt_thr_to_pe
 *
 *  \brief Perform threshold adapation based on the perceptual entropy
 *
 *  \param [in,out] pstr_psy_out        Pointer to psychoacoustic model output data structure
 *  \param [in,out] pstr_qs_pe_data     Pointer to perceptual entropy based quantization data
 * structure
 *  \param [in] 	desired_pe          Desired perceptual entropy
 *  \param [in] 	str_ah_param        Pointer to avoidance of holes algorithm parameter
 * structure
 *  \param [in] 	msa_param           Pointer to minimum SNR adaptation parameter structure
 *  \param [in] 	num_channels        Number of channels
 *  \param [in] 	chn                 Channel index
 *  \param [in] 	ptr_scratch         Pointer to scratch buffer

 *
 *  \return VOID
 */
static VOID impeghe_adapt_thr_to_pe(ia_psy_mod_out_data_struct *pstr_psy_out,
                                    ia_qc_pe_data_struct *pstr_qs_pe_data,
                                    const FLOAT32 desired_pe, ia_ah_param_struct *str_ah_param,
                                    ia_min_snr_adapt_param_struct *msa_param, WORD32 num_channels,
                                    WORD32 chn, UWORD8 *ptr_scratch)
{
  FLOAT32 no_red_pe, red_pe, red_pe_no_ah;
  FLOAT32 const_part, const_part_no_ah;
  FLOAT32 nactive_lines, nactive_lines_no_ah;
  FLOAT32 desired_pe_no_ah;
  FLOAT32 avg_thr_exp, redval;
  WORD32 *ah_flag[2];
  WORD32 iteration;
  for (WORD32 i = 0; i < 2; i++)
  {
    ah_flag[i] = (WORD32 *)ptr_scratch;
    ptr_scratch += (MAX_NUM_GROUPED_SFB) * sizeof(ah_flag[0][0]);
  }
  FLOAT32 *thr_exp[2];
  for (WORD32 i = 0; i < 2; i++)
  {
    thr_exp[i] = (FLOAT32 *)ptr_scratch;
    ptr_scratch += (MAX_NUM_GROUPED_SFB) * sizeof(thr_exp[0][0]);
  }

  impeghe_calc_thr_exp(thr_exp, pstr_psy_out, num_channels, chn);
  impeghe_adapt_min_snr(pstr_psy_out, msa_param, num_channels, chn);
  impeghe_init_avoid_hole_flag(ah_flag, pstr_psy_out, str_ah_param, num_channels, chn);

  no_red_pe = pstr_qs_pe_data->pe;
  const_part = pstr_qs_pe_data->const_part;
  nactive_lines = pstr_qs_pe_data->num_active_lines;
  avg_thr_exp = (FLOAT32)pow(2.0f, (const_part - no_red_pe) / (INV_RED_EXP_VAL * nactive_lines));
  redval = (FLOAT32)pow(2.0f, (const_part - desired_pe) / (INV_RED_EXP_VAL * nactive_lines)) -
           avg_thr_exp;
  redval = max(0.0f, redval);

  impeghe_reduce_thr(pstr_psy_out, ah_flag, thr_exp, redval, num_channels, chn);

  impeghe_calc_sfb_pe_data(pstr_qs_pe_data, pstr_psy_out, num_channels, chn);
  red_pe = pstr_qs_pe_data->pe;

  iteration = 0;
  do
  {
    impeghe_calc_pe_no_active_holes(&red_pe_no_ah, &const_part_no_ah, &nactive_lines_no_ah,
                                    pstr_qs_pe_data, ah_flag, pstr_psy_out, num_channels, chn);

    desired_pe_no_ah = max(desired_pe - (red_pe - red_pe_no_ah), 0);
    if (nactive_lines_no_ah > 0)
    {
      avg_thr_exp = (FLOAT32)pow(2.0f, (const_part_no_ah - red_pe_no_ah) /
                                           (INV_RED_EXP_VAL * nactive_lines_no_ah));
      redval += (FLOAT32)pow(2.0f, (const_part_no_ah - desired_pe_no_ah) /
                                       (INV_RED_EXP_VAL * nactive_lines_no_ah)) -
                avg_thr_exp;
      redval = max(0.0f, redval);
      impeghe_reduce_thr(pstr_psy_out, ah_flag, thr_exp, redval, num_channels, chn);
    }

    impeghe_calc_sfb_pe_data(pstr_qs_pe_data, pstr_psy_out, num_channels, chn);
    red_pe = pstr_qs_pe_data->pe;
    iteration++;
  } while ((fabs(red_pe - desired_pe) > (0.05f) * desired_pe) && (iteration < 2));
  if (red_pe >= 1.15f * desired_pe)
  {
    impeghe_reduce_min_snr(pstr_psy_out, pstr_qs_pe_data, ah_flag, 1.05f * desired_pe,
                           num_channels, chn);
    impeghe_allow_more_holes(pstr_psy_out, pstr_qs_pe_data, ah_flag, str_ah_param,
                             1.05f * desired_pe, num_channels, chn, ptr_scratch);
  }
  else
  {
    impeghe_correct_thr(pstr_psy_out, ah_flag, pstr_qs_pe_data, thr_exp, redval,
                        desired_pe - red_pe, num_channels, chn, ptr_scratch);
  }

  return;
}

/**
 *  impeghe_adj_thr
 *
 *  \brief Perform adjustment of quantization thresholds based on perceptual entropy
 *
 *  \param [in]     pstr_adj_thr_state        Pointer to threshold adjustment state structure
 *  \param [in,out] pstr_adj_thr_elem         Pointer to threshold adjustment element structure
 *  \param [in]     pstr_psy_out              Pointer to psychoacoustic model output data
 * structure
 *  \param [out]    ch_bit_dist               Distribution of bits between the channels
 *  \param [out]    pstr_qc_out               Pointer to quantization output data structure
 *  \param [in]     avg_bits                  Average number of bits per frame matching the given
 * constant bitrate
 *  \param [in] 	bitres_bits               Actual number of bits in the bitreservoir
 *  \param [in] 	max_bitres_bits           Maximum number of bits in the bitreservoir
 *  \param [in] 	side_info_bits            Bits for side information
 *  \param [in] 	max_bit_fac               Maximum value of bit-factor
 *  \param [in] 	num_channels              Number of channels
 *  \param [in] 	chn                       Channel index
 *  \param [out]	pstr_scratch		      Pointer to scratch memory structure
 *
 *  \return VOID
 */
VOID impeghe_adj_thr(ia_adj_thr_elem_struct *pstr_adj_thr_elem,
                     ia_psy_mod_out_data_struct *pstr_psy_out, FLOAT32 *ch_bit_dist,
                     ia_qc_out_data_struct *pstr_qc_out, const WORD32 avg_bits,
                     const WORD32 bitres_bits, const WORD32 max_bitres_bits,
                     const WORD32 side_info_bits, FLOAT32 *max_bit_fac, WORD32 num_channels,
                     WORD32 chn, impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 no_red_pe, granted_pe, granted_pe_corr;
  WORD32 curr_win_sequence;
  ia_qc_pe_data_struct *pstr_qc_pe_data = (ia_qc_pe_data_struct *)pstr_scratch->ptr_fd_scratch;
  pUWORD8 ptr_scratch = pstr_scratch->ptr_fd_scratch + sizeof(ia_qc_pe_data_struct);
  FLOAT32 bit_factor;
  WORD32 ch;

  pstr_qc_pe_data->pe_ch_data[0].sfb_lines = pstr_scratch->ptr_sfb_num_relevant_lines[0];
  pstr_qc_pe_data->pe_ch_data[0].sfb_ld_energy = pstr_scratch->ptr_sfb_ld_energy[0];
  if (num_channels == 2)
  {
    pstr_qc_pe_data->pe_ch_data[1].sfb_lines = pstr_scratch->ptr_sfb_num_relevant_lines[1];
    pstr_qc_pe_data->pe_ch_data[1].sfb_ld_energy = pstr_scratch->ptr_sfb_ld_energy[1];
  }
  pstr_qc_pe_data->offset = pstr_adj_thr_elem->pe_offset;

  impeghe_calc_sfb_pe_data(pstr_qc_pe_data, pstr_psy_out, num_channels, chn);
  no_red_pe = pstr_qc_pe_data->pe;

  curr_win_sequence = ONLY_LONG_SEQUENCE;
  if (num_channels == 2)
  {
    if ((pstr_psy_out[chn].window_sequence == EIGHT_SHORT_SEQUENCE) ||
        (pstr_psy_out[chn + 1].window_sequence == EIGHT_SHORT_SEQUENCE))
    {
      curr_win_sequence = EIGHT_SHORT_SEQUENCE;
    }
  }
  else
  {
    curr_win_sequence = pstr_psy_out[chn].window_sequence;
  }

  bit_factor =
      impeghe_bitres_calc_bitfac(bitres_bits, max_bitres_bits, no_red_pe + 5.0f * side_info_bits,
                                 curr_win_sequence, avg_bits, *max_bit_fac, pstr_adj_thr_elem);
  granted_pe = bit_factor * impeghe_bits_to_pe((FLOAT32)avg_bits);
  impeghe_calc_pe_correction(&(pstr_adj_thr_elem->pe_correction_fac), min(granted_pe, no_red_pe),
                             pstr_adj_thr_elem->pe_last, pstr_adj_thr_elem->dyn_bits_last);
  granted_pe_corr = granted_pe * pstr_adj_thr_elem->pe_correction_fac;

  if (granted_pe_corr < no_red_pe)
  {
    impeghe_adapt_thr_to_pe(
        pstr_psy_out, pstr_qc_pe_data, granted_pe_corr, &pstr_adj_thr_elem->str_ah_param,
        &pstr_adj_thr_elem->str_min_snr_adapt_params, num_channels, chn, ptr_scratch);
  }

  for (ch = 0; ch < num_channels; ch++)
  {
    FLOAT32 tmp_var, temp1;
    if (pstr_qc_pe_data->pe)
    {
      tmp_var = 1.0f - num_channels * 0.2f;
      temp1 = pstr_qc_pe_data->pe_ch_data[ch].pe / pstr_qc_pe_data->pe;
      temp1 = temp1 * tmp_var;
      ch_bit_dist[ch] = temp1 + 0.2f;
      if (ch_bit_dist[ch] < 0.2f)
        ch_bit_dist[ch] = 0.2f;
    }
    else
    {
      ch_bit_dist[ch] = 0.2f;
    }
  }

  pstr_qc_out->pe = no_red_pe;
  pstr_adj_thr_elem->pe_last = granted_pe;

  return;
}

/**
 *  impeghe_calc_form_fac_per_chan
 *
 *  \brief Calculate form factors per channel
 *
 *  \param [in] pstr_psy_out_chan   Pointer to psychoacoustic model output data structure
 *  \param [in,out] pstr_scratch    Pointer to scratch memory structure
 *  \param [in] i_ch				channel index
 *
 *  \return VOID
 */
VOID impeghe_calc_form_fac_per_chan(ia_psy_mod_out_data_struct *pstr_psy_out_chan,
                                    impeghe_scratch_mem *pstr_scratch, WORD32 i_ch)
{
  WORD32 i, j, sfb_offs;
  WORD32 sfb, sfb_width;
  FLOAT32 *ptr_sfb_form_factor = pstr_scratch->ptr_sfb_form_fac[i_ch];
  FLOAT32 *ptr_sfb_num_relevant_lines = pstr_scratch->ptr_sfb_num_relevant_lines[i_ch];
  FLOAT32 *ptr_sfb_ld_energy = pstr_scratch->ptr_sfb_ld_energy[i_ch];

  memset(ptr_sfb_num_relevant_lines, 0, sizeof(FLOAT32) * pstr_psy_out_chan->sfb_count);

  for (sfb_offs = 0; sfb_offs < pstr_psy_out_chan->sfb_count;
       sfb_offs += pstr_psy_out_chan->sfb_per_group)
  {
    i = sfb_offs;
    for (sfb = 0; sfb < pstr_psy_out_chan->max_sfb_per_grp; sfb++, i++)
    {
      ptr_sfb_form_factor[i] = MIN_FLT_VAL;
      if (pstr_psy_out_chan->ptr_sfb_energy[i] > pstr_psy_out_chan->ptr_sfb_thr[i])
      {
        FLOAT32 avg_form_factor;

        for (j = pstr_psy_out_chan->sfb_offsets[i]; j < pstr_psy_out_chan->sfb_offsets[i + 1];
             j++)
        {
          ptr_sfb_form_factor[i] += (FLOAT32)sqrt(fabs(pstr_psy_out_chan->ptr_spec_coeffs[j]));
        }

        sfb_width = pstr_psy_out_chan->sfb_offsets[i + 1] - pstr_psy_out_chan->sfb_offsets[i];
        avg_form_factor =
            (FLOAT32)pow(pstr_psy_out_chan->ptr_sfb_energy[i] / (FLOAT32)sfb_width, 0.25);
        ptr_sfb_num_relevant_lines[i] = ptr_sfb_form_factor[i] / avg_form_factor;
        ptr_sfb_ld_energy[i] = (FLOAT32)(log(pstr_psy_out_chan->ptr_sfb_energy[i]) * LOG2_1);
      }
    }
  }

  return;
}

/**
 *  impeghe_quantize_lines
 *
 *  \brief Perform quantization of spectral coefficients
 *
 *  \param [in]  gain                  Quantization gain
 *  \param [in]  num_lines             Number of spectral coefficients
 *  \param [out] ptr_exp_spectrum      Pointer to modified MDCT spectral coefficients
 *  \param [out] ptr_quant_spectrum    Pointer to quantized spectral coefficients
 *  \param [in]  ptr_mdct_spec         Pointer to original MDCT spectral coefficients
 *
 *  \return VOID
 */
VOID impeghe_quantize_lines(const WORD32 gain, const WORD32 num_lines, FLOAT32 *ptr_exp_spectrum,
                            WORD16 *ptr_quant_spectrum, FLOAT32 *ptr_mdct_spec)
{
  FLOAT32 quantizer;
  FLOAT32 k = 0.4054f;
  ;
  WORD32 line;

  quantizer = impeghe_fd_quant_table_e[(gain >> 4) + 8] * impeghe_fd_quant_table_q[gain & 15];
  for (line = 0; line < num_lines; line++)
  {
    FLOAT32 tmp = ptr_mdct_spec[line];
    if (tmp >= 0.0f)
    {
      ptr_exp_spectrum[line] = (FLOAT32)sqrt(tmp);
      ptr_exp_spectrum[line] *= (FLOAT32)sqrt(ptr_exp_spectrum[line]);
      ptr_quant_spectrum[line] = (WORD32)(k + quantizer * ptr_exp_spectrum[line]);
    }
    else
    {
      ptr_exp_spectrum[line] = (FLOAT32)sqrt(-tmp);
      ptr_exp_spectrum[line] *= (FLOAT32)sqrt(ptr_exp_spectrum[line]);
      ptr_quant_spectrum[line] = -(WORD32)(k + quantizer * ptr_exp_spectrum[line]);
    }
  }
  return;
}

/**
 *  impeghe_calculate_exp_spec
 *
 *  \brief Computes the 3/4th power of the input
 *
 *  \param [in]  num_lines          Number of spectral coefficients
 *  \param [out] ptr_exp_spectrum   Pointer to output modified spectral coefficients
 *  \param [in]  ptr_mdct_spec      Pointer to input MDCT spectrum
 *
 *  \return VOID
 */
VOID impeghe_calculate_exp_spec(const WORD32 num_lines, FLOAT32 *ptr_exp_spectrum,
                                FLOAT32 *ptr_mdct_spec)
{
  WORD32 line;

  for (line = 0; line < num_lines; line++)
  {
    ptr_exp_spectrum[line] = (FLOAT32)sqrt(fabs(ptr_mdct_spec[line]));
    ptr_exp_spectrum[line] *= (FLOAT32)sqrt(ptr_exp_spectrum[line]);
  }
  return;
}

/**
 *  impeghe_scf_delta_bit_count
 *
 *  \brief Compute bit-estimate based on differential Huffman coding
 *
 *  \param [in] delta     Difference value
 *
 *  \return WORD32        Bit-estimate for the difference value
 */

static WORD32 impeghe_scf_delta_bit_count(WORD32 delta)
{
  if (delta > 60)
  {
    return (impeghe_huffman_code_table[120][0]);
  }
  if (delta < -60)
  {
    return (impeghe_huffman_code_table[0][0]);
  }
  return (impeghe_huffman_code_table[delta + 60][0]);
}

/**
 *  impeghe_count_single_scf_bits
 *
 *  \brief Compute bit-estimate for current section
 *
 *  \param [in] scf         Scalefactor gain of current section
 *  \param [in] left_scf    Scalefactor gain of previous (left) section
 *  \param [in] right_scf   Scalefactor gain of next (right) section
 *
 *  \return WORD32   Estimate of scalefactor bits
 */
static WORD32 impeghe_count_single_scf_bits(WORD32 scf, WORD32 left_scf, WORD32 right_scf)
{
  WORD32 scf_bits;

  scf_bits =
      impeghe_scf_delta_bit_count(left_scf - scf) + impeghe_scf_delta_bit_count(scf - right_scf);

  return scf_bits;
}

/**
 *  impeghe_calc_single_spec_pe
 *
 *  \brief Compute perceptual entropy for noiseless coding for a single scalefactor band
 *
 *  \param [in] scf                 Actual scalefactor gain
 *  \param [in] sfb_const_pe_part   Constant part of perceptual entropy
 *  \param [in] num_lines           Number of spectral lines per scalefactor band
 *
 *  \return FLOAT32   Bit-estimate for current section
 */
static FLOAT32 impeghe_calc_single_spec_pe(WORD32 scf, FLOAT32 sfb_const_pe_part,
                                           FLOAT32 num_lines)
{
  FLOAT32 spec_pe;
  FLOAT32 ld_ratio;

  ld_ratio = sfb_const_pe_part - (FLOAT32)0.375f * (FLOAT32)scf;

  if (ld_ratio >= PE_C1)
  {

    spec_pe = (FLOAT32)0.7f * num_lines * ld_ratio;
  }
  else
  {

    spec_pe = (FLOAT32)0.7f * num_lines * (PE_C2 + PE_C3 * ld_ratio);
  }

  return spec_pe;
}

/**
 *  impeghe_count_scf_bits_diff
 *
 *  \brief Compute difference in bit-estimate
 *
 *  \param [in] ptr_sfb_prev   Pointer to previous scalefactor bands
 *  \param [in] ptr_sfb_new    Pointer to newly computed scalefactor bands
 *  \param [in] sfb_count      Total number of scalefactor bands
 *  \param [in] start_sfb      Start scalefactor band index
 *  \param [in] stop_sfb       Stop scalefactor band index
 *
 *  \return WORD32  Estimate of bit-difference
 */
static WORD32 impeghe_count_scf_bits_diff(WORD16 *ptr_sfb_prev, WORD16 *ptr_sfb_new,
                                          WORD32 sfb_count, WORD32 start_sfb, WORD32 stop_sfb)
{
  WORD32 scf_bits_diff = 0;
  WORD32 sfb = 0, sfb_last;
  WORD32 sfb_prev, sfb_next;

  sfb_last = start_sfb;

  while ((sfb_last < stop_sfb) && (ptr_sfb_prev[sfb_last] == SHRT_MIN))
  {

    sfb_last++;
  }

  sfb_prev = start_sfb - 1;

  while ((sfb_prev >= 0) && (ptr_sfb_prev[sfb_prev] == SHRT_MIN))
  {
    sfb_prev--;
  }

  if (sfb_prev >= 0)
  {
    scf_bits_diff += impeghe_scf_delta_bit_count(ptr_sfb_new[sfb_prev] - ptr_sfb_new[sfb_last]) -
                     impeghe_scf_delta_bit_count(ptr_sfb_prev[sfb_prev] - ptr_sfb_prev[sfb_last]);
  }

  for (sfb = sfb_last + 1; sfb < stop_sfb; sfb++)
  {

    if (ptr_sfb_prev[sfb] != SHRT_MIN)
    {

      scf_bits_diff += impeghe_scf_delta_bit_count(ptr_sfb_new[sfb_last] - ptr_sfb_new[sfb]) -
                       impeghe_scf_delta_bit_count(ptr_sfb_prev[sfb_last] - ptr_sfb_prev[sfb]);

      sfb_last = sfb;
    }
  }

  sfb_next = stop_sfb;

  while ((sfb_next < sfb_count) && (ptr_sfb_prev[sfb_next] == SHRT_MIN))
  {

    sfb_next++;
  }

  if (sfb_next < sfb_count)
  {

    scf_bits_diff += impeghe_scf_delta_bit_count(ptr_sfb_new[sfb_last] - ptr_sfb_new[sfb_next]) -
                     impeghe_scf_delta_bit_count(ptr_sfb_prev[sfb_last] - ptr_sfb_prev[sfb_next]);
  }

  return scf_bits_diff;
}

/**
 *  impeghe_calc_spec_pe_diff
 *
 *  \brief Compute difference in perceptual entropy
 *
 *  \param [in] pstr_psy_out                Pointer to psychoacoustic model output data structure
 *  \param [in] scf_prev                    Previous scalefactor gain
 *  \param [in] scf_new                     Newly compute scalefactor gain
 *  \param [in] ptr_sfb_const_pe_part       Pointer to constant part of perceptual entropy per
 * scalefactor band
 *  \param [in] ptr_sfb_form_fac            Pointer to form factor per scalefactor band
 *  \param [in] ptr_sfb_num_rel_lines       Pointer to number of spectral coefficients per scale
 * factor band
 *  \param [in] start_sfb                   Start index of scalefactor band
 *  \param [in] stop_sfb                    Stop index of scalefactor band
 *
 *  \return WORD32   Estimated difference in perceptual entropy
 */
static FLOAT32 impeghe_calc_spec_pe_diff(ia_psy_mod_out_data_struct *pstr_psy_out,
                                         WORD16 *scf_prev, WORD16 *scf_new,
                                         FLOAT32 *ptr_sfb_const_pe_part,
                                         FLOAT32 *ptr_sfb_form_fac,
                                         FLOAT32 *ptr_sfb_num_rel_lines, WORD32 start_sfb,
                                         WORD32 stop_sfb)
{
  FLOAT32 spec_pe_diff = 0.0f;
  WORD32 sfb;

  for (sfb = start_sfb; sfb < stop_sfb; sfb++)
  {

    if (scf_prev[sfb] != SHRT_MIN)
    {
      FLOAT32 ld_ratio_prev, ld_ratio_new, pe_prev, pe_new;

      if (ptr_sfb_const_pe_part[sfb] == MIN_FLT_VAL)
      {

        ptr_sfb_const_pe_part[sfb] = (FLOAT32)log(pstr_psy_out->ptr_sfb_energy[sfb] *
                                                  (FLOAT32)6.75f / ptr_sfb_form_fac[sfb]) *
                                     LOG2_1;
      }

      ld_ratio_prev = ptr_sfb_const_pe_part[sfb] - 0.375f * scf_prev[sfb];
      ld_ratio_new = ptr_sfb_const_pe_part[sfb] - 0.375f * scf_new[sfb];

      if (ld_ratio_prev < PE_C1)
      {
        pe_prev = PE_C2 + PE_C3 * ld_ratio_prev;
      }
      else
      {
        pe_prev = ld_ratio_prev;
      }

      if (ld_ratio_new < PE_C1)
      {
        pe_new = PE_C2 + PE_C3 * ld_ratio_new;
      }
      else
      {
        pe_new = ld_ratio_new;
      }

      spec_pe_diff += (FLOAT32)0.7f * ptr_sfb_num_rel_lines[sfb] * (pe_new - pe_prev);
    }
  }

  return spec_pe_diff;
}

/**
 *  impeghe_calc_sfb_dist
 *
 *  \brief Compute scalefactor distortion based on quantization and inverse quantization
 *
 *  \param [in] ptr_spec          Pointer to original spectrum
 *  \param [in] ptr_exp_spec      Pointer to expected spectrum
 *  \param [in] ptr_quant_spec    Pointer to quantized spectrum
 *  \param [in] sfb_width         Scalefactor band width
 *  \param [in] gain              Scalefactor gain
 *
 *  \return FLOAT32               Scalefactor distortion
 */
FLOAT32 impeghe_calc_sfb_dist(const FLOAT32 *ptr_spec, const FLOAT32 *ptr_exp_spec,
                              WORD16 *ptr_quant_spec, WORD32 sfb_width, WORD32 gain)
{
  WORD32 i;
  FLOAT32 dist = 0;
  FLOAT32 k = 0.4054f;
  FLOAT32 quantizer =
      impeghe_fd_quant_table_e[(gain >> 4) + 8] * impeghe_fd_quant_table_q[gain & 15];
  FLOAT32 inv_quantizer =
      impeghe_fd_inv_quant_table_e[(gain >> 4) + 8] * impeghe_fd_inv_quant_table_q[gain & 15];

  for (i = 0; i < sfb_width; i++)
  {
    FLOAT32 iq_val;
    FLOAT32 diff;

    ptr_quant_spec[i] = (WORD32)(k + quantizer * ptr_exp_spec[i]);

    if (ptr_quant_spec[i] < 64)
    {
      iq_val = impeghe_pow4_3_table[ptr_quant_spec[i]] * inv_quantizer;
    }
    else
    {
      iq_val = (FLOAT32)((pow((FLOAT32)abs(ptr_quant_spec[i]), 4.0f / 3.0f)) * inv_quantizer);
    }

    diff = (FLOAT32)fabs(ptr_spec[i]) - iq_val;

    dist += diff * diff;
  }

  return dist;
}

/**
 *  impeghe_improve_scf
 *
 *  \brief Scalefactor gain improvement by quantization
 *
 *  \param [in]  	 ptr_spec             Pointer to original spectral coefficients
 *  \param [in]  	 ptr_exp_spec         Pointer to modified spectral coefficients
 *  \param [out] 	 ptr_quant_spec       Pointer to final quantized spectral coefficients
 *  \param [out] 	 ptr_quant_spec_temp  Pointer to temporary buffer containing quantized
 * spectral coefficients
 *  \param [out] 	 sfb_width            Scalefactor band width
 *  \param [in]  	 threshold            Scalefactor band thresholds (in dB)
 *  \param [in,out]  scf                  Integer values of scalefactor gain
 *  \param [in]      min_scf              Minimum scalefactor gain value
 *  \param [out]     dist                 Pointer to scalefactor distortion
 *  \param [in]      ptr_min_calc_scf     Pointer to minimum calculated scalefactor gain
 *
 *  \return WORD16                    Best value of scalefactor gain that results in minimum
 * distortion
 */
static WORD16 impeghe_improve_scf(FLOAT32 *ptr_spec, FLOAT32 *ptr_exp_spec,
                                  WORD16 *ptr_quant_spec, WORD16 *ptr_quant_spec_temp,
                                  WORD32 sfb_width, FLOAT32 threshold, WORD16 scf, WORD16 min_scf,
                                  FLOAT32 *dist, WORD16 *ptr_min_calc_scf)
{
  FLOAT32 sfb_dist;
  WORD16 best_scf = scf;
  WORD32 j;

  sfb_dist = impeghe_calc_sfb_dist(ptr_spec, ptr_exp_spec, ptr_quant_spec, sfb_width, scf);

  *ptr_min_calc_scf = scf;

  if (sfb_dist <= (1.25 * threshold))
  {
    FLOAT32 best_sfb_dist = sfb_dist;
    FLOAT32 allowed_sfb_dist = min(sfb_dist * 1.25f, threshold);
    WORD32 count;

    for (count = 0; count < 3; count++)
    {

      scf++;

      sfb_dist =
          impeghe_calc_sfb_dist(ptr_spec, ptr_exp_spec, ptr_quant_spec_temp, sfb_width, scf);

      if (sfb_dist < allowed_sfb_dist)
      {

        *ptr_min_calc_scf = best_scf + 1;

        best_scf = scf;
        best_sfb_dist = sfb_dist;
        memcpy(ptr_quant_spec, ptr_quant_spec_temp, sfb_width * sizeof(WORD16));
      }
    }
    *dist = best_sfb_dist;
  }
  else
  {
    WORD16 estimated_scf = scf;
    FLOAT32 best_sfb_dist = sfb_dist;
    WORD32 count;

    count = 0;

    while ((sfb_dist > (1.25 * threshold)) && (count++ < 3))
    {

      scf++;

      sfb_dist =
          impeghe_calc_sfb_dist(ptr_spec, ptr_exp_spec, ptr_quant_spec_temp, sfb_width, scf);

      if (sfb_dist < best_sfb_dist)
      {

        best_scf = scf;
        best_sfb_dist = sfb_dist;

        memcpy(ptr_quant_spec, ptr_quant_spec_temp, sfb_width * sizeof(WORD16));
      }
    }

    count = 0;
    scf = estimated_scf;
    sfb_dist = best_sfb_dist;

    while ((sfb_dist > (1.25 * threshold)) && (count++ < 1) && (scf > min_scf))
    {
      scf--;

      sfb_dist =
          impeghe_calc_sfb_dist(ptr_spec, ptr_exp_spec, ptr_quant_spec_temp, sfb_width, scf);

      if (sfb_dist < best_sfb_dist)
      {

        best_scf = scf;
        best_sfb_dist = sfb_dist;

        memcpy(ptr_quant_spec, ptr_quant_spec_temp, sfb_width * sizeof(WORD16));
      }
      *ptr_min_calc_scf = scf;
    }
    *dist = best_sfb_dist;
  }

  for (j = 0; j < sfb_width; j++)
  {
    if (ptr_spec[j] < 0)
    {
      ptr_quant_spec[j] = -ptr_quant_spec[j];
    }
  }

  return best_scf;
}

/**
 *  impeghe_assimilate_single_scf
 *
 *  \brief Perform scalefactor difference reduction for single scalefactor bands
 *
 *  \param [in]     pstr_psy_out                   Pointer to psychoacoustic model output data
 * structure
 *  \param [in]     ptr_exp_spec                   Pointer to modified spectral coefficients
 *  \param [out]    ptr_quant_spec                 Pointer to final quantized spectral
 * coefficients
 *  \param [in]     ptr_quant_spec_temp            Pointer to temporary buffer containing
 * quantized spectral coefficients
 *  \param [in,out] scf                            Pointer to values of scalefactor gain
 *  \param [in]     ptr_min_scf                    Pointer to scalefactor gain value
 *  \param [in,out] ptr_sfb_dist                   Pointer to scalefactor distortion
 *  \param [in,out] ptr_sfb_const_pe_part          Pointer to constant part of perceptual entropy
 *  \param [in]     ptr_sfb_form_fac               Pointer to form factors
 *  \param [in]     ptr_sfb_num_lines              Pointer to number of spectral coefficients per
 * scale factor band
 *  \param [in,out] ptr_min_calc_scf               Pointer to minimum calculated scalefactor gain
 *  \param [in]     ptr_mdct_spec_float            Pointer to original MDCT spectral coefficients
 *
 *  \return VOID
 */
static VOID impeghe_assimilate_single_scf(ia_psy_mod_out_data_struct *pstr_psy_out,
                                          FLOAT32 *ptr_exp_spec, WORD16 *ptr_quant_spec,
                                          WORD16 *ptr_quant_spec_temp, WORD16 *scf,
                                          WORD16 *ptr_min_scf, FLOAT32 *ptr_sfb_dist,
                                          FLOAT32 *ptr_sfb_const_pe_part,
                                          FLOAT32 *ptr_sfb_form_fac, FLOAT32 *ptr_sfb_num_lines,
                                          WORD16 *ptr_min_calc_scf, FLOAT32 *ptr_mdct_spec_float)
{
  WORD32 sfb_prev, sfb_act, sfb_next;
  WORD16 scf_act, *scf_prev, *scf_next, min_scf, max_scf;
  WORD32 sfb_width, sfb_offs;
  FLOAT32 energy;
  FLOAT32 sfb_pe_prev, sfb_pe_new;
  FLOAT32 sfb_dist_new;
  WORD32 j;
  WORD32 success = 0;
  FLOAT32 delta_pe = 0.0f, delta_pe_new, delta_pe_temp;
  WORD16 prev_scf_last[MAX_NUM_GROUPED_SFB], prev_scf_next[MAX_NUM_GROUPED_SFB];
  FLOAT32 delta_pe_last[MAX_NUM_GROUPED_SFB];
  WORD32 update_min_scf;

  for (j = 0; j < pstr_psy_out->sfb_count; j++)
  {

    prev_scf_last[j] = SHRT_MAX;
    prev_scf_next[j] = SHRT_MAX;
    delta_pe_last[j] = MAX_FLT_VAL;
  }

  sfb_prev = -1;
  sfb_act = -1;
  sfb_next = -1;
  scf_prev = 0;
  scf_next = 0;
  min_scf = SHRT_MAX;
  max_scf = SHRT_MAX;

  do
  {
    sfb_next++;

    while ((sfb_next < pstr_psy_out->sfb_count) && (scf[sfb_next] == SHRT_MIN))
    {
      sfb_next++;
    }

    if ((sfb_prev >= 0) && (sfb_act >= 0) && (sfb_next < pstr_psy_out->sfb_count))
    {

      scf_act = scf[sfb_act];

      scf_prev = scf + sfb_prev;
      scf_next = scf + sfb_next;

      min_scf = min(*scf_prev, *scf_next);

      max_scf = max(*scf_prev, *scf_next);
    }
    else
    {

      if ((sfb_prev == -1) && (sfb_act >= 0) && (sfb_next < pstr_psy_out->sfb_count))
      {

        scf_act = scf[sfb_act];

        scf_prev = &scf_act;

        scf_next = scf + sfb_next;

        min_scf = *scf_next;

        max_scf = *scf_next;
      }
      else
      {

        if ((sfb_prev >= 0) && (sfb_act >= 0) && (sfb_next == pstr_psy_out->sfb_count))
        {

          scf_act = scf[sfb_act];

          scf_prev = scf + sfb_prev;

          scf_next = &scf_act;

          min_scf = *scf_prev;

          max_scf = *scf_prev;
        }
      }
    }

    if (sfb_act >= 0)
    {

      min_scf = max(min_scf, ptr_min_scf[sfb_act]);
    }

    if ((sfb_act >= 0) && (sfb_prev >= 0 || sfb_next < pstr_psy_out->sfb_count) &&
        (scf_act > min_scf) && (scf_act <= min_scf + MAX_SCF_DELTA) &&
        (scf_act >= max_scf - MAX_SCF_DELTA) &&
        (*scf_prev != prev_scf_last[sfb_act] || *scf_next != prev_scf_next[sfb_act] ||
         delta_pe < delta_pe_last[sfb_act]))
    {
      success = 0;

      sfb_width = pstr_psy_out->sfb_offsets[sfb_act + 1] - pstr_psy_out->sfb_offsets[sfb_act];

      sfb_offs = pstr_psy_out->sfb_offsets[sfb_act];

      energy = pstr_psy_out->ptr_sfb_energy[sfb_act];

      if (ptr_sfb_const_pe_part[sfb_act] == MIN_FLT_VAL)
      {

        ptr_sfb_const_pe_part[sfb_act] =
            (FLOAT32)log(energy * (FLOAT32)6.75f / ptr_sfb_form_fac[sfb_act]) * LOG2_1;
      }

      sfb_pe_prev = impeghe_calc_single_spec_pe(scf_act, ptr_sfb_const_pe_part[sfb_act],
                                                ptr_sfb_num_lines[sfb_act]) +
                    impeghe_count_single_scf_bits(scf_act, *scf_prev, *scf_next);

      delta_pe_new = delta_pe;
      update_min_scf = 1;

      do
      {
        scf_act--;

        if (scf_act < ptr_min_calc_scf[sfb_act] && scf_act >= max_scf - MAX_SCF_DELTA)
        {
          sfb_pe_new = impeghe_calc_single_spec_pe(scf_act, ptr_sfb_const_pe_part[sfb_act],
                                                   ptr_sfb_num_lines[sfb_act]) +
                       (FLOAT32)impeghe_count_single_scf_bits(scf_act, *scf_prev, *scf_next);

          delta_pe_temp = delta_pe + sfb_pe_new - sfb_pe_prev;

          if (delta_pe_temp < (FLOAT32)10.0f)
          {

            sfb_dist_new =
                impeghe_calc_sfb_dist(ptr_mdct_spec_float + sfb_offs, ptr_exp_spec + sfb_offs,
                                      ptr_quant_spec_temp + sfb_offs, sfb_width, scf_act);

            if (sfb_dist_new < ptr_sfb_dist[sfb_act])
            {

              scf[sfb_act] = scf_act;
              ptr_sfb_dist[sfb_act] = sfb_dist_new;

              for (j = sfb_offs; j < sfb_offs + sfb_width; j++)
              {

                ptr_quant_spec[j] = ptr_quant_spec_temp[j];

                if (ptr_mdct_spec_float[j] < 0.0f)
                {
                  ptr_quant_spec[j] = -ptr_quant_spec[j];
                }
              }
              delta_pe_new = delta_pe_temp;
              success = 1;
            }

            if (update_min_scf)
            {
              ptr_min_calc_scf[sfb_act] = scf_act;
            }
          }
          else
          {
            update_min_scf = 0;
          }
        }
      } while (scf_act > min_scf);

      delta_pe = delta_pe_new;

      prev_scf_last[sfb_act] = *scf_prev;
      prev_scf_next[sfb_act] = *scf_next;
      delta_pe_last[sfb_act] = delta_pe;
    }

    if (success)
    {

      sfb_prev = -1;
      sfb_act = -1;
      sfb_next = -1;
      scf_prev = 0;
      scf_next = 0;
      min_scf = SHRT_MAX;
      max_scf = SHRT_MAX;
      success = 0;
    }
    else
    {

      sfb_prev = sfb_act;
      sfb_act = sfb_next;
    }
  } while (sfb_next < pstr_psy_out->sfb_count);
  return;
}

/**
 *  impeghe_assimilate_multiple_scf
 *
 *  \brief Perform scalefactor difference reduction for a region of scalefactor bands
 *
 *  \param [in]     pstr_psy_out                    Pointer to psychoacoustic model output data
 * structure
 *  \param [in]     ptr_exp_spec                    Pointer to modified spectral coefficients
 *  \param [out]    ptr_quant_spec                  Pointer to final quantized spectral
 * coefficients
 *  \param [in]     ptr_quant_spec_temp             Pointer to temporary buffer containing
 * quantized spectral coefficients
 *  \param [in,out] ptr_scf                         Pointer to integer values of scalefactor gain
 *  \param [in]     ptr_min_scf                     Pointer to minimum scalefactor gain value
 *  \param [in,out] ptr_sfb_dist                    Pointer to scalefactor distortion
 *  \param [in,out] ptr_sfb_const_pe_part           Pointer to constant part of perceptual entropy
 *  \param [in]     ptr_sfb_form_fac                Pointer to form factors
 *  \param [in]     ptr_sfb_num_lines               Pointer to number of spectral coefficients per
 * scale factor band
 *  \param [in]     ptr_mdct_spec_float             Pointer to original MDCT spectral coefficients
 *
 *  \return VOID
 */
static VOID impeghe_assimilate_multiple_scf(ia_psy_mod_out_data_struct *pstr_psy_out,
                                            FLOAT32 *ptr_exp_spec, WORD16 *ptr_quant_spec,
                                            WORD16 *ptr_quant_spec_temp, WORD16 *ptr_scf,
                                            WORD16 *ptr_min_scf, FLOAT32 *ptr_sfb_dist,
                                            FLOAT32 *ptr_sfb_const_pe_part,
                                            FLOAT32 *ptr_sfb_form_fac, FLOAT32 *ptr_sfb_num_lines,
                                            FLOAT32 *ptr_mdct_spec_float, pUWORD8 pstr_scratch)
{
  WORD32 sfb, start_sfb, stop_sfb;
  WORD16 scf_temp[MAX_NUM_GROUPED_SFB], min_scf, max_scf, scf_act;
  WORD32 possible_region_found;
  WORD32 sfb_width, sfb_offs, j;
  FLOAT32 prev_dist_sum, new_dist_sum;
  WORD32 delta_scf_bits;
  FLOAT32 delta_spec_pe;
  FLOAT32 delta_pe = 0.0f, delta_pe_new;
  WORD32 sfb_count = pstr_psy_out->sfb_count;
  FLOAT32 *sfb_dist_new = (FLOAT32 *)pstr_scratch;
  min_scf = SHRT_MAX;
  max_scf = SHRT_MIN;

  for (sfb = 0; sfb < sfb_count; sfb++)
  {

    if (ptr_scf[sfb] != SHRT_MIN)
    {

      min_scf = min(min_scf, ptr_scf[sfb]);

      max_scf = max(max_scf, ptr_scf[sfb]);
    }
  }

  if (max_scf != SHRT_MIN && max_scf <= min_scf + MAX_SCF_DELTA)
  {

    scf_act = max_scf;

    do
    {
      scf_act--;

      memcpy(scf_temp, ptr_scf, MAX_NUM_GROUPED_SFB * sizeof(WORD16));

      stop_sfb = 0;

      do
      {

        sfb = stop_sfb;

        while (sfb < sfb_count && (ptr_scf[sfb] == SHRT_MIN || ptr_scf[sfb] <= scf_act))
        {

          sfb++;
        }

        start_sfb = sfb;

        sfb++;

        while (sfb < sfb_count && (ptr_scf[sfb] == SHRT_MIN || ptr_scf[sfb] > scf_act))
        {
          sfb++;
        }

        stop_sfb = sfb;

        possible_region_found = 0;

        if (start_sfb < sfb_count)
        {

          possible_region_found = 1;

          for (sfb = start_sfb; sfb < stop_sfb; sfb++)
          {

            if (ptr_scf[sfb] != SHRT_MIN)
            {

              if (scf_act < ptr_min_scf[sfb])
              {

                possible_region_found = 0;
                break;
              }
            }
          }
        }

        if (possible_region_found)
        {

          for (sfb = start_sfb; sfb < stop_sfb; sfb++)
          {

            if (scf_temp[sfb] != SHRT_MIN)
            {

              scf_temp[sfb] = scf_act;
            }
          }

          delta_scf_bits =
              impeghe_count_scf_bits_diff(ptr_scf, scf_temp, sfb_count, start_sfb, stop_sfb);

          delta_spec_pe =
              impeghe_calc_spec_pe_diff(pstr_psy_out, ptr_scf, scf_temp, ptr_sfb_const_pe_part,
                                        ptr_sfb_form_fac, ptr_sfb_num_lines, start_sfb, stop_sfb);

          delta_pe_new = delta_pe + (FLOAT32)delta_scf_bits + delta_spec_pe;

          if (delta_pe_new < (FLOAT32)10.0f)
          {

            prev_dist_sum = new_dist_sum = 0.0f;

            for (sfb = start_sfb; sfb < stop_sfb; sfb++)
            {

              if (scf_temp[sfb] != SHRT_MIN)
              {

                prev_dist_sum += ptr_sfb_dist[sfb];

                sfb_width = pstr_psy_out->sfb_offsets[sfb + 1] - pstr_psy_out->sfb_offsets[sfb];

                sfb_offs = pstr_psy_out->sfb_offsets[sfb];

                sfb_dist_new[sfb] =
                    impeghe_calc_sfb_dist(ptr_mdct_spec_float + sfb_offs, ptr_exp_spec + sfb_offs,
                                          ptr_quant_spec_temp + sfb_offs, sfb_width, scf_act);

                if (sfb_dist_new[sfb] > pstr_psy_out->ptr_sfb_thr[sfb])
                {

                  new_dist_sum = (FLOAT32)2.0f * prev_dist_sum;
                  break;
                }

                new_dist_sum += sfb_dist_new[sfb];
              }
            }

            if (new_dist_sum < prev_dist_sum)
            {

              delta_pe = delta_pe_new;

              for (sfb = start_sfb; sfb < stop_sfb; sfb++)
              {

                if (ptr_scf[sfb] != SHRT_MIN)
                {

                  sfb_width = pstr_psy_out->sfb_offsets[sfb + 1] - pstr_psy_out->sfb_offsets[sfb];

                  sfb_offs = pstr_psy_out->sfb_offsets[sfb];
                  ptr_scf[sfb] = scf_act;
                  ptr_sfb_dist[sfb] = sfb_dist_new[sfb];

                  for (j = sfb_offs; j < sfb_offs + sfb_width; j++)
                  {

                    ptr_quant_spec[j] = ptr_quant_spec_temp[j];

                    if (ptr_mdct_spec_float[j] < 0.0f)
                    {

                      ptr_quant_spec[j] = -ptr_quant_spec[j];
                    }
                  }
                }
              }
            }
          }
        }

      } while (stop_sfb <= sfb_count);

    } while (scf_act > min_scf);
  }
  return;
}

/**
 *  impeghe_estimate_scfs_chan
 *
 *  \brief Estimate scalefactors per channel
 *
 *  \param [in,out] pstr_psy_out             Pointer to psychoacoustic model output data structure
 *  \param [out]    str_qc_out_chan          Pointer to quantization output data structure
 *  \param [in]     num_channels             Number of channels
 *  \param [in]     chn                      Channel index
 *  \param [in]     pstr_scratch             Pointer to scratch memory structure
 *
 *  \return VOID
 */
VOID impeghe_estimate_scfs_chan(ia_psy_mod_out_data_struct *pstr_psy_out,
                                ia_qc_out_chan_struct *str_qc_out_chan, WORD32 num_channels,
                                WORD32 chn, impeghe_scratch_mem *pstr_scratch)
{
  WORD16 *ptr_scalefactor;
  WORD32 *global_gain;
  WORD16 *ptr_quant_spec;
  LOOPIDX i, ch, j, idx = 0;
  FLOAT32 thresh, energy, energy_part, thr_part;
  FLOAT32 scf_float;
  WORD16 scf_int = 0, min_scf = 0, max_scf = 0;
  FLOAT64 max_spec = 0.0f;
  WORD16 min_sf_max_quant[MAX_NUM_GROUPED_SFB] = {0};
  pUWORD8 ptr_scratch = pstr_scratch->ptr_fd_scratch;
  FLOAT32 *ptr_sfb_dist = (FLOAT32 *)ptr_scratch;
  ptr_scratch += MAX_NUM_GROUPED_SFB * sizeof(ptr_sfb_dist[0]);
  WORD16 min_calc_scf[MAX_NUM_GROUPED_SFB] = {0};

  WORD16 *ptr_quant_spec_temp = pstr_scratch->p_adjthr_quant_spec_temp;
  FLOAT32 *ptr_exp_spec = pstr_scratch->p_adjthr_ptr_exp_spec;
  FLOAT32 *ptr_mdct_spec_float = pstr_scratch->p_adjthr_mdct_spec_float;
  FLOAT32 *ptr_sfb_const_pe_part = (FLOAT32 *)ptr_scratch;

  FLOAT32 **ptr_sfb_form_factor = &pstr_scratch->ptr_sfb_form_fac[0];
  FLOAT32 **ptr_sfb_num_relevant_lines = &pstr_scratch->ptr_sfb_num_relevant_lines[0];

  ptr_scratch += MAX_NUM_GROUPED_SFB * sizeof(ptr_sfb_const_pe_part[0]);

  memset(ptr_quant_spec_temp, 0, FRAME_LEN_LONG * sizeof(WORD16));
  memset(ptr_mdct_spec_float, 0, FRAME_LEN_LONG * sizeof(FLOAT32));
  memset(ptr_exp_spec, 0, FRAME_LEN_LONG * sizeof(FLOAT32));
  memset(ptr_sfb_dist, 0, MAX_NUM_GROUPED_SFB * sizeof(FLOAT32));
  for (ch = chn; ch < chn + num_channels; ch++)
  {
    ia_psy_mod_out_data_struct *ptr_psy_out = &pstr_psy_out[ch];
    str_qc_out_chan[idx].global_gain = 0;

    memset(str_qc_out_chan[idx].scalefactor, 0, sizeof(WORD16) * pstr_psy_out[ch].sfb_count);
    memset(str_qc_out_chan[idx].quant_spec, 0, sizeof(WORD16) * FRAME_LEN_LONG);

    ptr_scalefactor = str_qc_out_chan[idx].scalefactor;
    global_gain = &str_qc_out_chan[idx].global_gain;
    ptr_quant_spec = str_qc_out_chan[idx].quant_spec;
    for (i = 0; i < ptr_psy_out->sfb_count; i++)
    {
      thresh = ptr_psy_out->ptr_sfb_thr[i];
      energy = ptr_psy_out->ptr_sfb_energy[i];
      max_spec = 0.0;

      for (j = ptr_psy_out->sfb_offsets[i]; j < ptr_psy_out->sfb_offsets[i + 1]; j++)
      {
        max_spec = max(max_spec, fabs(ptr_psy_out->ptr_spec_coeffs[j]));
      }

      ptr_scalefactor[i] = MIN_SHRT_VAL;
      min_sf_max_quant[i] = MIN_SHRT_VAL;

      if ((max_spec > 0.0) && (energy > thresh) && (ptr_sfb_form_factor[idx][i] != MIN_FLT_VAL))
      {
        WORD32 j;

        energy_part = (FLOAT32)log10(ptr_sfb_form_factor[idx][i]);

        thr_part = (FLOAT32)log10(6.75 * thresh + MIN_FLT_VAL);
        scf_float = 8.8585f * (thr_part - energy_part);
        scf_int = (WORD32)floor(scf_float);
        min_sf_max_quant[i] = (WORD32)ceil(C1_SF + C2_SF * log(max_spec));
        scf_int = max(scf_int, min_sf_max_quant[i]);

        for (j = 0; j < ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i]; j++)
        {
          ptr_exp_spec[ptr_psy_out->sfb_offsets[i] + j] =
              (FLOAT32)(ptr_psy_out->ptr_spec_coeffs[ptr_psy_out->sfb_offsets[i] + j]);
          ptr_mdct_spec_float[ptr_psy_out->sfb_offsets[i] + j] =
              (FLOAT32)(ptr_psy_out->ptr_spec_coeffs[ptr_psy_out->sfb_offsets[i] + j]);
        }

        impeghe_calculate_exp_spec(ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i],
                                   ptr_exp_spec + ptr_psy_out->sfb_offsets[i],
                                   ptr_mdct_spec_float + ptr_psy_out->sfb_offsets[i]);

        scf_int = impeghe_improve_scf(
            ptr_mdct_spec_float + ptr_psy_out->sfb_offsets[i],
            ptr_exp_spec + ptr_psy_out->sfb_offsets[i],
            ptr_quant_spec + ptr_psy_out->sfb_offsets[i],
            ptr_quant_spec_temp + ptr_psy_out->sfb_offsets[i],
            ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i], thresh, scf_int,
            min_sf_max_quant[i], &ptr_sfb_dist[i], &min_calc_scf[i]);

        ptr_scalefactor[i] = scf_int;
      }
    }

    for (i = 0; i < ptr_psy_out->sfb_count; i++)
    {
      ptr_sfb_const_pe_part[i] = MIN_FLT_VAL;
    }

    impeghe_assimilate_single_scf(
        ptr_psy_out, ptr_exp_spec, ptr_quant_spec, ptr_quant_spec_temp, ptr_scalefactor,
        min_sf_max_quant, ptr_sfb_dist, ptr_sfb_const_pe_part, ptr_sfb_form_factor[idx],
        ptr_sfb_num_relevant_lines[idx], min_calc_scf, ptr_mdct_spec_float);

    impeghe_assimilate_multiple_scf(
        ptr_psy_out, ptr_exp_spec, ptr_quant_spec, ptr_quant_spec_temp, ptr_scalefactor,
        min_sf_max_quant, ptr_sfb_dist, ptr_sfb_const_pe_part, ptr_sfb_form_factor[idx],
        ptr_sfb_num_relevant_lines[idx], ptr_mdct_spec_float, ptr_scratch);

    max_scf = MIN_SHRT_VAL;
    min_scf = MAX_SHRT_VAL;
    for (i = 0; i < ptr_psy_out->sfb_count; i++)
    {
      if (max_scf < ptr_scalefactor[i])
      {
        max_scf = ptr_scalefactor[i];
      }
      if ((ptr_scalefactor[i] != MIN_SHRT_VAL) && (min_scf > ptr_scalefactor[i]))
      {
        min_scf = ptr_scalefactor[i];
      }
    }

    for (i = 0; i < pstr_psy_out[ch].sfb_count; i++)
    {
      if ((ptr_scalefactor[i] != MIN_SHRT_VAL) && (min_scf + MAX_SCF_DELTA) < ptr_scalefactor[i])
      {
        ptr_scalefactor[i] = min_scf + MAX_SCF_DELTA;

        impeghe_calc_sfb_dist(ptr_mdct_spec_float + +ptr_psy_out->sfb_offsets[i],
                              ptr_exp_spec + ptr_psy_out->sfb_offsets[i],
                              ptr_quant_spec + ptr_psy_out->sfb_offsets[i],
                              ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i],
                              ptr_scalefactor[i]);
      }
    }

    max_scf = min((min_scf + MAX_SCF_DELTA), max_scf);

    if (max_scf > MIN_SHRT_VAL)
    {
      *global_gain = max_scf;
      for (i = 0; i < ptr_psy_out->sfb_count; i++)
      {
        if (ptr_scalefactor[i] == MIN_SHRT_VAL)
        {
          ptr_scalefactor[i] = 0;
          memset(&ptr_psy_out->ptr_spec_coeffs[ptr_psy_out->sfb_offsets[i]], 0,
                 (ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i]) *
                     sizeof(FLOAT64));
        }
        else
        {
          ptr_scalefactor[i] = max_scf - ptr_scalefactor[i];
        }
      }
    }
    else
    {
      *global_gain = 0;
      for (i = 0; i < ptr_psy_out->sfb_count; i++)
      {
        ptr_scalefactor[i] = 0;
        memset(&ptr_psy_out->ptr_spec_coeffs[ptr_psy_out->sfb_offsets[i]], 0,
               (ptr_psy_out->sfb_offsets[i + 1] - ptr_psy_out->sfb_offsets[i]) * sizeof(FLOAT64));
      }
    }
    idx++;
  }

  return;
}
