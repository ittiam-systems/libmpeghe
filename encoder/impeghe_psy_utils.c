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
#include "impeghe_block_switch_const.h"
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
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_psy_utils.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_fd_qc_adjthr.h"

extern ia_sfb_info_struct impeghe_sfb_info_1024[6];
extern const ia_sfb_info_tables impeghe_sfb_info_tables[6];

static const FLOAT32 impeghe_bark_quiet_thr_val[] = {
    15.0f, 10.0f, 7.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  0.0f,  0.0f, 0.0f,
    0.0f,  0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 5.0f, 10.0f, 20.0f, 30.0f};

/**
 *  impeghe_calc_band_energy
 *
 *  \brief Calculate scalefactor band energies
 *
 *  \param [in]  ptr_spec_coeffs		Pointer to input spectral coefficients
 *  \param [in]  ptr_band_offset			Pointer to scalefactor band offsets
 *  \param [in]  num_bands			Number of scalefactro bands
 *  \param [out] ptr_band_energy	Pointer to bandwise energies
 *  \param [in]  sfb_count			Number of active scalefactor bands
 *
 *  \return VOID
 */
VOID impeghe_calc_band_energy(const FLOAT64 *ptr_spec_coeffs, const WORD32 *ptr_band_offset,
                              const WORD32 num_bands, FLOAT32 *ptr_band_energy, WORD32 sfb_count)
{
  WORD32 i, j;

  j = 0;
  memset(ptr_band_energy, 0, sfb_count * sizeof(FLOAT32));
  for (i = 0; i < num_bands; i++)
  {
    while (j < ptr_band_offset[i + 1])
    {
      ptr_band_energy[i] += (FLOAT32)(ptr_spec_coeffs[j] * ptr_spec_coeffs[j]);
      j++;
    }
  }
  return;
}

/**
 *  impeghe_find_max_spreading
 *
 *  \brief Find spreaded energies per scalefactor band
 *
 *  \param [in]  sfb_count             Number of scalefactor bands
 *  \param [in]  ptr_mask_low_fac      Frequency dependent weighting factor for high frequencies
 *  \param [in]  ptr_mask_high_fac     Frequency dependent weighting factor for low frequencies
 *  \param [out] ptr_spreaded_enegry   Pointer to spreaded energies per scalefactor band
 *
 *  \return VOID
 */
VOID impeghe_find_max_spreading(const WORD32 sfb_count, const FLOAT32 *ptr_mask_low_fac,
                                const FLOAT32 *ptr_mask_high_fac, FLOAT32 *ptr_spreaded_enegry)
{
  WORD32 i;

  for (i = 1; i < sfb_count; i++)
  {
    ptr_spreaded_enegry[i] =
        max(ptr_spreaded_enegry[i], ptr_mask_high_fac[i] * ptr_spreaded_enegry[i - 1]);
  }

  for (i = sfb_count - 2; i >= 0; i--)
  {
    ptr_spreaded_enegry[i] =
        max(ptr_spreaded_enegry[i], ptr_mask_low_fac[i] * ptr_spreaded_enegry[i + 1]);
  }
  return;
}

/**
 *  impeghe_pre_echo_control
 *
 *  \brief Pre-echo control of the psychoacoustic model
 *
 *  \param [in]     ptr_thr_nm1              Previous threshold
 *  \param [in]     sfb_count                Number of scalefactor bands
 *  \param [in]     max_allowed_inc_fac      Parameter 1
 *  \param [in]     min_remaining_thr_fac    Parameter 2
 *  \param [in,out] ptr_threshold            Actual threshold
 *
 *  \return VOID
 */
VOID impeghe_pre_echo_control(FLOAT32 *ptr_thr_nm1, WORD32 sfb_count, FLOAT32 max_allowed_inc_fac,
                              FLOAT32 min_remaining_thr_fac, FLOAT32 *ptr_threshold)
{
  WORD32 i;
  FLOAT32 thr1, thr2;

  for (i = 0; i < sfb_count; i++)
  {
    thr1 = max_allowed_inc_fac * (ptr_thr_nm1[i]);
    thr2 = min_remaining_thr_fac * ptr_threshold[i];

    ptr_thr_nm1[i] = ptr_threshold[i];

    if (ptr_threshold[i] > thr1)
    {
      ptr_threshold[i] = thr1;
    }
    if (thr2 > ptr_threshold[i])
    {
      ptr_threshold[i] = thr2;
    }
  }
  return;
}

/**
 *  impeghe_sfb_init
 *
 *  \brief Initialization of sfb params
 *
 *  \param [in]  sample_rate		Sample rate
 *  \param [in]  block_type			Block type
 *  \param [out] ptr_sfb_offset		offset table for scalefactor band
 *  \param [out] ptr_sfb_count		Number of scalefactor bands
 *
 *  \return IA_ERRORCODE			Error code
 */
static VOID impeghe_sfb_init(WORD32 sample_rate, WORD32 block_type, WORD32 *ptr_sfb_offset,
                             WORD32 *ptr_sfb_count)
{
  const UWORD8 *ptr_sfb_params = 0;
  WORD32 start_offset, block_len = 0;

  if (block_type == ONLY_LONG_SEQUENCE)
  {
    block_len = FRAME_LEN_LONG;
    switch (sample_rate)
    {
    case 48000:
      ptr_sfb_params = impeghe_sfb_info_tables[5].params_long;
      break;
    case 44100:
      ptr_sfb_params = impeghe_sfb_info_tables[4].params_long;
      break;
    case 32000:
    case 29400:
      ptr_sfb_params = impeghe_sfb_info_tables[3].params_long;
      break;
    case 24000:
      ptr_sfb_params = impeghe_sfb_info_tables[2].params_long;
      break;
    case 22050:
      ptr_sfb_params = impeghe_sfb_info_tables[1].params_long;
      break;
    case 16000:
    case 14700:
      ptr_sfb_params = impeghe_sfb_info_tables[0].params_long;
      break;
    }
  }
  else
  {
    block_len = FRAME_LEN_SHORT;
    switch (sample_rate)
    {
    case 48000:
      ptr_sfb_params = impeghe_sfb_info_tables[5].params_short;
      break;
    case 44100:
      ptr_sfb_params = impeghe_sfb_info_tables[4].params_short;
      break;
    case 32000:
    case 29400:
      ptr_sfb_params = impeghe_sfb_info_tables[3].params_short;
      break;
    case 24000:
      ptr_sfb_params = impeghe_sfb_info_tables[2].params_short;
      break;
    case 22050:
      ptr_sfb_params = impeghe_sfb_info_tables[1].params_short;
      break;
    case 16000:
    case 14700:
      ptr_sfb_params = impeghe_sfb_info_tables[0].params_short;
      break;
    }
  }

  *ptr_sfb_count = 0;
  start_offset = 0;

  do
  {
    ptr_sfb_offset[*ptr_sfb_count] = start_offset;
    start_offset += ptr_sfb_params[*ptr_sfb_count];
    (*ptr_sfb_count)++;
  } while (start_offset < block_len);

  ptr_sfb_offset[*ptr_sfb_count] = start_offset;

  return;
}

/**
 *  impeghe_atan_approx
 *
 *  \brief Calculate approximate value of atan()
 *
 *  \param [in] val		input value
 *
 *  \return FLOAT32		output value
 */
static FLOAT32 impeghe_atan_approx(FLOAT32 val)
{
  if (val < (FLOAT32)1.0)
  {
    return (val / ((FLOAT32)1.0f + (FLOAT32)0.280872f * val * val));
  }
  else
  {
    return ((FLOAT32)1.57079633f - val / ((FLOAT32)0.280872f + val * val));
  }
}

/**
 *  impeghe_calc_barc_line_value
 *
 *  \brief Calculate bark value
 *
 *  \param [in] num_lines	Block frame length
 *  \param [in] fft_line	frame index
 *  \param [in] sample_rate	Sample rate
 *
 *  \return FLOAT32			Bark value
 */
static FLOAT32 impeghe_calc_barc_line_value(WORD32 num_lines, WORD32 fft_line, WORD32 sample_rate)
{

  FLOAT32 center_freq, temp, b_value;

  center_freq = (FLOAT32)fft_line * ((FLOAT32)sample_rate * (FLOAT32)0.5f) / (FLOAT32)num_lines;
  temp = (FLOAT32)impeghe_atan_approx((FLOAT32)1.3333333e-4f * center_freq);
  b_value = (FLOAT32)13.3f * impeghe_atan_approx((FLOAT32)0.00076f * center_freq) +
            (FLOAT32)3.5f * temp * temp;

  return (b_value);
}

/**
 *  impeghe_barc_values_init
 *
 *  \brief Initialization of bark values
 *
 *  \param [in]  sfb_count		Number of scalefactors band
 *  \param [in]  ptr_sfb_offset	offset table for scalefactor band
 *  \param [in]  num_lines		block frame length
 *  \param [in]  sample_rate	Sample rate
 *  \param [out] ptr_b_value	Pointer to bark values
 *
 *  \return VOID
 */
static VOID impeghe_barc_values_init(WORD32 sfb_count, WORD32 *ptr_sfb_offset, WORD32 num_lines,
                                     WORD32 sample_rate, FLOAT32 *ptr_b_value)
{
  WORD32 i;
  FLOAT32 b_val0, b_val1;
  b_val0 = 0.0f;

  for (i = 0; i < sfb_count; i++)
  {
    b_val1 = impeghe_calc_barc_line_value(num_lines, ptr_sfb_offset[i + 1], sample_rate);
    ptr_b_value[i] = (b_val0 + b_val1) * (FLOAT32)0.5f;
    b_val0 = b_val1;
  }
  return;
}

/**
 *  impeghe_thr_quiet_init
 *
 *  \brief Initialize threshold values in quiet
 *
 *  \param [in]  sfb_count		Number of scalefactors band
 *  \param [in]  ptr_sfb_offset	offset table for scalefactor band
 *  \param [in]  ptr_barc_val	Pointer to bark values
 *  \param [out] ptr_thr_quiet	Pointer to threshold values
 *
 *  \return VOID
 */
static VOID impeghe_thr_quiet_init(WORD32 sfb_count, WORD32 *ptr_sfb_offset,
                                   FLOAT32 *ptr_barc_val, FLOAT32 *ptr_thr_quiet)
{
  WORD32 i;
  FLOAT32 barc_thr_quiet;

  for (i = 0; i < sfb_count; i++)
  {
    WORD32 b_val1, b_val2;

    if (i > 0)
    {
      b_val1 = (WORD32)(ptr_barc_val[i] + ptr_barc_val[i - 1]) >> 1;
    }
    else
    {
      b_val1 = (WORD32)(ptr_barc_val[i]) >> 1;
    }

    if (i < sfb_count - 1)
    {
      b_val2 = (WORD32)(ptr_barc_val[i] + ptr_barc_val[i + 1]) >> 1;
    }
    else
    {
      b_val2 = (WORD32)(ptr_barc_val[i]);
    }
    b_val1 = min(b_val1, (WORD32)MAX_BARC_VALUE);
    b_val2 = min(b_val2, (WORD32)MAX_BARC_VALUE);
    barc_thr_quiet = min(impeghe_bark_quiet_thr_val[b_val1], impeghe_bark_quiet_thr_val[b_val2]);

    ptr_thr_quiet[i] = (FLOAT32)pow(10.0f, (barc_thr_quiet - 20.0f) * (FLOAT32)0.1f) * 16887.8f *
                       (FLOAT32)(ptr_sfb_offset[i + 1] - ptr_sfb_offset[i]);
  }
  return;
}

/**
 *  impeghe_spreading_init
 *
 *  \brief Initialization of spreading params
 *
 *  \param [in]  sfb_count						Number of scalefactors
 *												bands
 *  \param [in]  ptr_barc_val					Pointer to bark values
 *  \param [out] ptr_mask_low_fac				Frequency dependent weighting
 *												factor
 *for
 *low
 *frequencies
 *  \param [out] ptr_mask_high_fac				Frequency dependent weighting
 *												factor
 *for
 *high
 *frequencies
 *  \param [out] ptr_mask_low_fac_spr_energy	Pointer to spreaded energy for low frequencies
 *  \param [out] ptr_mask_high_fac_spr_energy	Pointer to spreaded energy for high frequencies
 *  \param [in]  bit_rate						Bit rate
 *  \param [in]  block_type						Block type
 *
 *  \return VOID
 */
static VOID impeghe_spreading_init(WORD32 sfb_count, FLOAT32 *ptr_barc_val,
                                   FLOAT32 *ptr_mask_low_fac, FLOAT32 *ptr_mask_high_fac,
                                   FLOAT32 *ptr_mask_low_fac_spr_energy,
                                   FLOAT32 *ptr_mask_high_fac_spr_energy, const WORD32 bit_rate,
                                   WORD32 block_type)
{
  WORD32 i;
  FLOAT32 mask_low_spr_energy, mask_high_spr_energy;

  if (block_type != EIGHT_SHORT_SEQUENCE)
  {
    mask_low_spr_energy = MASK_LOW_SP_ENERGY_L;
    mask_high_spr_energy = (bit_rate > 22000) ? MASK_HIGH_SP_ENERGY_L : MASK_HIGH_SP_ENERGY_L_LBR;
  }
  else
  {
    mask_low_spr_energy = MASK_LOW_SP_ENERGY_S;
    mask_high_spr_energy = MASK_HIGH_SP_ENERGY_S;
  }

  for (i = 0; i < sfb_count; i++)
  {
    if (i > 0)
    {
      FLOAT32 db_val;
      FLOAT32 diff_val = (ptr_barc_val[i] - ptr_barc_val[i - 1]);

      db_val = MASK_HIGH_FAC * diff_val;
      ptr_mask_high_fac[i] = (FLOAT32)pow(10.0f, -db_val);
      db_val = MASK_LOW_FAC * diff_val;
      ptr_mask_low_fac[i - 1] = (FLOAT32)pow(10.0f, -db_val);
      db_val = mask_high_spr_energy * diff_val;
      ptr_mask_high_fac_spr_energy[i] = (FLOAT32)pow(10.0f, -db_val);
      db_val = mask_low_spr_energy * diff_val;
      ptr_mask_low_fac_spr_energy[i - 1] = (FLOAT32)pow(10.0f, -db_val);
    }
    else
    {
      ptr_mask_high_fac[i] = 0.0f;
      ptr_mask_low_fac[sfb_count - 1] = 0.0f;
      ptr_mask_high_fac_spr_energy[i] = 0.0f;
      ptr_mask_low_fac_spr_energy[sfb_count - 1] = 0.0f;
    }
  }
  return;
}

/**
 *  impeghe_min_snr_init
 *
 *  \brief Initialize minimum SNR values
 *
 *  \param [in]  bit_rate			Bit rate
 *  \param [in]  sample_rate		Sample rate
 *  \param [in]  num_lines			block frame length
 *  \param [in]  ptr_sfb_offset		offset table for scalefactor band
 *  \param [in]  ptr_barc_value		Pointer to bark values
 *  \param [in]  sfb_active			Number of active sfb's
 *  \param [out] ptr_sfb_min_snr	Pointer to minimum SNR values
 *
 *  \return VOID
 */
static VOID impeghe_min_snr_init(const WORD32 bit_rate, const WORD32 sample_rate,
                                 const WORD32 num_lines, const WORD32 *ptr_sfb_offset,
                                 const FLOAT32 *ptr_barc_value, const WORD32 sfb_active,
                                 FLOAT32 *ptr_sfb_min_snr)
{
  WORD32 sfb;
  FLOAT32 barc_fac;
  FLOAT32 barc_width;
  FLOAT32 pe_per_window, pe_part;
  FLOAT32 snr;
  FLOAT32 b_val0, b_val1;

  barc_fac = (FLOAT32)1.0 / min(ptr_barc_value[sfb_active - 1] / MAX_BARC_VALUE, (FLOAT32)1.0);
  pe_per_window =
      impeghe_bits_to_pe((FLOAT32)bit_rate / (FLOAT32)sample_rate * (FLOAT32)num_lines);
  b_val0 = (FLOAT32)0.0f;

  for (sfb = 0; sfb < sfb_active; sfb++)
  {
    b_val1 = (FLOAT32)2.0 * ptr_barc_value[sfb] - b_val0;
    barc_width = b_val1 - b_val0;
    b_val0 = b_val1;

    pe_part = pe_per_window * (FLOAT32)0.024f * barc_fac;
    pe_part *= barc_width;
    pe_part /= (FLOAT32)(ptr_sfb_offset[sfb + 1] - ptr_sfb_offset[sfb]);
    snr = (FLOAT32)pow(2.0f, pe_part) - 1.5f;
    snr = 1.0f / max(snr, 1.0f);
    snr = min(snr, 0.8f);
    snr = max(snr, 0.003f);
    ptr_sfb_min_snr[sfb] = snr;
  }
  return;
}

/**
 *  impeghe_psy_long_config_init
 *
 *  \brief Initialization of psy params for long block
 *
 *  \param [in] 	bit_rate		Bit rate
 *  \param [in] 	sample_rate		Sample rate
 *  \param [in] 	band_width		Bandwidth limit
 *  \param [in,out] pstr_psy_config	Pointer to psy params for long block
 *
 *  \return VOID
 */
VOID impeghe_psy_long_config_init(WORD32 bit_rate, WORD32 sample_rate, WORD32 band_width,
                                  ia_psy_mod_long_config_struct *pstr_psy_config)
{
  WORD32 sfb;
  FLOAT32 sfb_barc_val[MAX_NUM_GROUPED_SFB];

  impeghe_sfb_init(sample_rate, ONLY_LONG_SEQUENCE, pstr_psy_config->sfb_offset,
                   &(pstr_psy_config->sfb_count));

  impeghe_barc_values_init(pstr_psy_config->sfb_count, pstr_psy_config->sfb_offset,
                           pstr_psy_config->sfb_offset[pstr_psy_config->sfb_count], sample_rate,
                           sfb_barc_val);

  impeghe_thr_quiet_init(pstr_psy_config->sfb_count, pstr_psy_config->sfb_offset, sfb_barc_val,
                         pstr_psy_config->sfb_thr_quiet);

  impeghe_spreading_init(
      pstr_psy_config->sfb_count, sfb_barc_val, pstr_psy_config->sfb_mask_low_fac,
      pstr_psy_config->sfb_mask_high_fac, pstr_psy_config->sfb_mask_low_fac_spr_ener,
      pstr_psy_config->sfb_mask_high_fac_spr_ener, bit_rate, ONLY_LONG_SEQUENCE);

  pstr_psy_config->ratio = C_RATIO;
  pstr_psy_config->max_allowed_inc_fac = 2.0f;
  pstr_psy_config->min_remaining_thr_fac = 0.01f;

  pstr_psy_config->clip_energy = 1.0e9f;
  pstr_psy_config->low_pass_line = (WORD32)((2 * band_width * FRAME_LEN_LONG) / sample_rate);

  for (sfb = 0; sfb < pstr_psy_config->sfb_count; sfb++)
  {
    if (pstr_psy_config->sfb_offset[sfb] >= pstr_psy_config->low_pass_line)
      break;
  }
  pstr_psy_config->sfb_active = sfb;

  impeghe_min_snr_init(bit_rate, sample_rate,
                       pstr_psy_config->sfb_offset[pstr_psy_config->sfb_count],
                       pstr_psy_config->sfb_offset, sfb_barc_val, pstr_psy_config->sfb_active,
                       pstr_psy_config->sfb_min_snr);

  return;
}

/**
 *  impeghe_psy_short_config_init
 *
 *  \brief Initialization of psy params for short block
 *
 *  \param [in] 	bit_rate			Bit rate
 *  \param [in]		sample_rate			Sample rate
 *  \param [in] 	band_width			Bandwidth limit
 *  \param [in,out] pstr_psy_config		Pointer to psy params for short block
 *
 *  \return VOID
 */
VOID impeghe_psy_short_config_init(WORD32 bit_rate, WORD32 sample_rate, WORD32 band_width,
                                   ia_psy_mod_short_config_struct *pstr_psy_config)
{
  WORD32 sfb;
  FLOAT32 sfb_barc_val[MAX_NUM_GROUPED_SFB];

  impeghe_sfb_init(sample_rate, EIGHT_SHORT_SEQUENCE, pstr_psy_config->sfb_offset,
                   &(pstr_psy_config->sfb_count));

  impeghe_barc_values_init(pstr_psy_config->sfb_count, pstr_psy_config->sfb_offset,
                           pstr_psy_config->sfb_offset[pstr_psy_config->sfb_count], sample_rate,
                           sfb_barc_val);

  impeghe_thr_quiet_init(pstr_psy_config->sfb_count, pstr_psy_config->sfb_offset, sfb_barc_val,
                         pstr_psy_config->sfb_thr_quiet);

  impeghe_spreading_init(
      pstr_psy_config->sfb_count, sfb_barc_val, pstr_psy_config->sfb_mask_low_fac,
      pstr_psy_config->sfb_mask_high_fac, pstr_psy_config->sfb_mask_low_fac_spr_ener,
      pstr_psy_config->sfb_mask_high_fac_spr_ener, bit_rate, EIGHT_SHORT_SEQUENCE);

  pstr_psy_config->ratio = C_RATIO;
  pstr_psy_config->max_allowed_inc_fac = 2.0f;
  pstr_psy_config->min_remaining_thr_fac = 0.01f;

  pstr_psy_config->clip_energy = 15625000;
  pstr_psy_config->low_pass_line = (WORD32)((2 * band_width * FRAME_LEN_SHORT) / sample_rate);

  for (sfb = 0; sfb < pstr_psy_config->sfb_count; sfb++)
  {
    if (pstr_psy_config->sfb_offset[sfb] >= pstr_psy_config->low_pass_line)
      break;
  }
  pstr_psy_config->sfb_active = sfb;

  impeghe_min_snr_init(bit_rate, sample_rate,
                       pstr_psy_config->sfb_offset[pstr_psy_config->sfb_count],
                       pstr_psy_config->sfb_offset, sfb_barc_val, pstr_psy_config->sfb_active,
                       pstr_psy_config->sfb_min_snr);

  return;
}

/**
 *  impeghe_sfb_params_init
 *
 *  \brief Initialization of sfb params
 *
 *  \param [in]  sample_rate	Sample rate
 *  \param [out] ptr_sfb_width	offset table for scalefactor band
 *  \param [out] num_sfb		Number of sfb's
 *  \param [in]  win_seq		Window sequence
 *
 *  \return VOID
 */
VOID impeghe_sfb_params_init(WORD32 sample_rate, WORD32 *ptr_sfb_width, WORD32 *num_sfb,
                             WORD32 win_seq)
{
  WORD32 i, j, k;
  ia_sfb_info_struct *ptr_sr_info = NULL;

  if (sample_rate == 14700)
  {
    sample_rate = 16000;
  }
  if (sample_rate == 29400)
  {
    sample_rate = 32000;
  }

  switch (sample_rate)
  {
  case 16000:
    ptr_sr_info = &impeghe_sfb_info_1024[0];
    break;
  case 22050:
    ptr_sr_info = &impeghe_sfb_info_1024[1];
    break;
  case 24000:
    ptr_sr_info = &impeghe_sfb_info_1024[2];
    break;
  case 32000:
    ptr_sr_info = &impeghe_sfb_info_1024[3];
    break;
  case 44100:
    ptr_sr_info = &impeghe_sfb_info_1024[4];
    break;
  case 48000:
    ptr_sr_info = &impeghe_sfb_info_1024[5];
    break;
  default:
    ptr_sr_info = &impeghe_sfb_info_1024[5];
    break;
  }

  j = 0;
  for (i = 0; i < ptr_sr_info->num_sfb_long; i++)
  {
    k = ptr_sr_info->cb_offset_long[i];
    ptr_sr_info->sfb_width_long[i] = k - j;
    j = k;
  }
  j = 0;
  for (i = 0; i < ptr_sr_info->num_sfb_short; i++)
  {
    k = ptr_sr_info->cb_offset_short[i];
    ptr_sr_info->sfb_width_short[i] = k - j;
    j = k;
  }

  if (win_seq == EIGHT_SHORT_SEQUENCE)
  {
    memcpy(ptr_sfb_width, ptr_sr_info->sfb_width_short, MAX_NUM_SFB_SHORT * sizeof(WORD32));
    *num_sfb = ptr_sr_info->num_sfb_short;
  }
  else
  {
    memcpy(ptr_sfb_width, ptr_sr_info->sfb_width_long, MAX_NUM_SFB_LONG * sizeof(WORD32));
    *num_sfb = ptr_sr_info->num_sfb_long;
  }

  return;
}
