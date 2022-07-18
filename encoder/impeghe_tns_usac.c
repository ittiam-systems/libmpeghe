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
#include <string.h>
#include "impeghe_cnst.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"

static const WORD32 impeghe_tns_supported_sampling_rates[13] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 0};

static const UWORD16 impeghe_tns_min_band_number_long[12] = {11, 12, 15, 16, 17, 20,
                                                             25, 26, 24, 28, 30, 31};

static const UWORD16 impeghe_tns_min_band_number_short[12] = {2, 2, 2, 3,  3,  4,
                                                              6, 6, 8, 10, 10, 12};

static const WORD32 impeghe_tns_max_bands_table[16][2] = {{31, 9},  /**< 96000 */
                                                          {31, 9},  /**< 88200 */
                                                          {34, 10}, /**< 64000 */
                                                          {40, 14}, /**< 48000 */
                                                          {42, 14}, /**< 44100 */
                                                          {51, 14}, /**< 32000 */
                                                          {47, 15}, /**< 24000 */
                                                          {47, 15}, /**< 22050 */
                                                          {43, 15}, /**< 16000 */
                                                          {43, 15}, /**< 12000 */
                                                          {43, 15}, /**< 11025 */
                                                          {40, 15}, /**< 8000  */
                                                          {40, 15}, /**< 7350  */
                                                          {0, 0},   {0, 0}, {0, 0}};

/**
 *  impeghe_freq_to_band_mapping
 *
 *  \brief Maps frequency to band index
 *
 *  \param [in] freq					Input frequency
 *  \param [in] sample_rate				Sampling frequency
 *  \param [in] num_bands				Number of bands
 *  \param [in] ptr_band_start_offset	offset table for scalefactor band
 *
 *  \return WORD32						Band index
 */
static WORD32 impeghe_freq_to_band_mapping(WORD32 freq, WORD32 sample_rate, WORD32 num_bands,
                                           const WORD32 *ptr_band_start_offset)
{
  WORD32 line_num, band;

  line_num = (freq * ptr_band_start_offset[num_bands] * 4 / sample_rate + 1) / 2;

  if (line_num >= ptr_band_start_offset[num_bands])
  {
    return num_bands;
  }

  for (band = 0; band < num_bands; band++)
  {
    if (ptr_band_start_offset[band + 1] > line_num)
      break;
  }

  if (line_num - ptr_band_start_offset[band] > ptr_band_start_offset[band + 1] - line_num)
  {
    band++;
  }

  return band;
};

/**
 *  impeghe_calc_gauss_win
 *
 *  \brief Calculate Gaussian window coeffs
 *
 *  \param [out] ptr_win			Pointer to window coeffs
 *  \param [in] length				Length of window
 *  \param [in] sample_rate			Sample rate
 *  \param [in] win_seq				Window sequence
 *  \param [in] time_resolution		Time resolution
 *
 *  \return VOID
 */
static VOID impeghe_calc_gauss_win(FLOAT64 *ptr_win, const WORD32 length,
                                   const WORD32 sample_rate, const WORD32 win_seq,
                                   const FLOAT32 time_resolution)
{
  WORD32 i;
  FLOAT32 gauss_exp = 3.14159265358979323f * sample_rate * 0.001f * (FLOAT32)time_resolution /
                      (win_seq != EIGHT_SHORT_SEQUENCE ? 1024.0f : 128.0f);

  gauss_exp = -0.5f * gauss_exp * gauss_exp;

  for (i = 0; i < length; i++)
  {

    ptr_win[i] = (FLOAT32)exp(gauss_exp * (i + 0.5) * (i + 0.5));
  }
  return;
}

/**
 *  impeghe_tns_init
 *
 *  \brief Initialize TNS params
 *
 *  \param [in] sampling_rate	Sample rate
 *  \param [in] bit_rate		Bit rate
 *  \param [out] pstr_tns_info		Pointer to TNS info structure
 *  \param [in] num_channels	Number of channels
 *
 *  \return IA_ERRORCODE		Error code
 */
IA_ERRORCODE impeghe_tns_init(WORD32 sampling_rate, WORD32 bit_rate, ia_tns_info *pstr_tns_info,
                              WORD32 num_channels)
{
  WORD32 fs_index = 0;
  WORD32 lpc_stop_freq = 16000;
  WORD32 lpc_start_freq_long = 2500, lpc_start_freq_short = 3750;
  pstr_tns_info->threshold = 1.41f;
  pstr_tns_info->tns_time_res_short = 0.6f;
  pstr_tns_info->tns_time_res_long = 0.6f;

  if (sampling_rate == 14700)
  {
    sampling_rate = 16000;
  }
  if (sampling_rate == 29400)
  {
    sampling_rate = 32000;
  }

  if (bit_rate < 32000)
  {
    if (num_channels == 1)
    {
      pstr_tns_info->threshold = 1.2f;
      lpc_start_freq_long = 2000;
    }
  }
  else if (bit_rate < 36000)
  {
    if (num_channels == 1)
    {
      pstr_tns_info->tns_time_res_long = 0.8f;
    }
    else
    {
      pstr_tns_info->tns_time_res_long = 0.5f;
    }
    pstr_tns_info->tns_time_res_short = 0.3f;
  }
  else
  {
    pstr_tns_info->tns_time_res_long = 0.5f;
    pstr_tns_info->tns_time_res_short = 0.3f;
  }

  /** Determine if sampling rate is supported
   */
  while (sampling_rate != impeghe_tns_supported_sampling_rates[fs_index])
  {
    if (!impeghe_tns_supported_sampling_rates[fs_index])
    {
      return -1;
    }
    fs_index++;
  }

  pstr_tns_info->tns_max_bands_long = impeghe_tns_max_bands_table[fs_index][0];
  pstr_tns_info->tns_max_bands_short = impeghe_tns_max_bands_table[fs_index][1];
  pstr_tns_info->tns_max_order_long = 15;
  pstr_tns_info->tns_max_order_short = 7;

  pstr_tns_info->tns_min_band_number_long = impeghe_tns_min_band_number_long[fs_index];
  pstr_tns_info->tns_min_band_number_short = impeghe_tns_min_band_number_short[fs_index];

  pstr_tns_info->lpc_start_band_long = impeghe_freq_to_band_mapping(
      lpc_start_freq_long, sampling_rate, pstr_tns_info->max_sfb_long,
      pstr_tns_info->sfb_offset_table_long);

  pstr_tns_info->lpc_start_band_short = impeghe_freq_to_band_mapping(
      lpc_start_freq_short, sampling_rate, pstr_tns_info->max_sfb_short,
      pstr_tns_info->sfb_offset_table_short);

  pstr_tns_info->lpc_stop_band_long =
      impeghe_freq_to_band_mapping(lpc_stop_freq, sampling_rate, pstr_tns_info->max_sfb_long,
                                   pstr_tns_info->sfb_offset_table_long);

  pstr_tns_info->lpc_stop_band_short =
      impeghe_freq_to_band_mapping(lpc_stop_freq, sampling_rate, pstr_tns_info->max_sfb_short,
                                   pstr_tns_info->sfb_offset_table_short);

  impeghe_calc_gauss_win(pstr_tns_info->win_long, pstr_tns_info->tns_max_order_long + 1,
                         sampling_rate, ONLY_LONG_SEQUENCE, pstr_tns_info->tns_time_res_long);

  impeghe_calc_gauss_win(pstr_tns_info->win_short, pstr_tns_info->tns_max_order_short + 1,
                         sampling_rate, EIGHT_SHORT_SEQUENCE, pstr_tns_info->tns_time_res_short);
  return 0;
}

/**
 *  impeghe_tns_filter
 *
 *  \brief TNS filter implementation
 *
 *  \param [in] length				Length of input
 *  \param [in,out] ptr_spec				Pointer to spectrum
 *  \param [in] filter				Pointer to filter structure
 *  \param [in] ptr_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
static VOID impeghe_tns_filter(WORD32 length, FLOAT64 *ptr_spec, ia_tns_filter_data *pstr_filter,
                               FLOAT64 *ptr_scratch)
{
  WORD32 i, j, k = 0;
  WORD32 order = pstr_filter->order;
  FLOAT64 *a = pstr_filter->a_coeffs;

  /** Determine loop parameters for given direction
   */
  if (pstr_filter->direction)
  {

    /** Startup, initial state is zero
     */
    ptr_scratch[length - 1] = ptr_spec[length - 1];
    for (i = length - 2; i > (length - 1 - order); i--)
    {
      ptr_scratch[i] = ptr_spec[i];
      k++;
      for (j = 1; j <= k; j++)
      {
        ptr_spec[i] += ptr_scratch[i + j] * a[j];
      }
    }

    /** Now filter the rest
     */
    for (i = length - 1 - order; i >= 0; i--)
    {
      ptr_scratch[i] = ptr_spec[i];
      for (j = 1; j <= order; j++)
      {
        ptr_spec[i] += ptr_scratch[i + j] * a[j];
      }
    }
  }
  else
  {

    /** Startup, initial state is zero
     */
    ptr_scratch[0] = ptr_spec[0];
    for (i = 1; i < order; i++)
    {
      ptr_scratch[i] = ptr_spec[i];
      for (j = 1; j <= i; j++)
      {
        ptr_spec[i] += ptr_scratch[i - j] * a[j];
      }
    }

    /** Now filter the rest
     */
    for (i = order; i < length; i++)
    {
      ptr_scratch[i] = ptr_spec[i];
      for (j = 1; j <= order; j++)
      {
        ptr_spec[i] += ptr_scratch[i - j] * a[j];
      }
    }
  }

  return;
}

/**
 *  impeghe_truncate_coeffs
 *
 *  \brief Truncate coeffs below threshold
 *
 *  \param [in] order		Filter order
 *  \param [in] threshold	Threshold value
 *  \param [in,out] ptr_k		Pointer to input array
 *
 *  \return WORD32			Truncated order value
 */
static WORD32 impeghe_truncate_coeffs(WORD32 order, FLOAT64 threshold, FLOAT64 *ptr_k)
{
  WORD32 i;
  for (i = order; i >= 0; i--)
  {
    ptr_k[i] = (fabs(ptr_k[i]) > threshold) ? ptr_k[i] : 0.0;
    if (ptr_k[i] != 0.0)
    {
      return i;
    }
  }
  return 0;
}

/**
 *  impeghe_quantize_reflection_coeffs
 *
 *  \brief Quanization of coeffs
 *
 *  \param [in] f_order			Filter order
 *  \param [in] coeff_res		coeffs resolution
 *  \param [in,out] ptr_k		Input array
 *  \param [out] ptr_index	Index array
 *
 *  \return VOID
 */
static VOID impeghe_quantize_reflection_coeffs(WORD32 f_order, WORD32 coeff_res, FLOAT64 *ptr_k,
                                               WORD32 *ptr_index)
{

  FLOAT64 iqfac, iqfac_m;
  WORD32 i;

  iqfac = (((WORD64)1 << (coeff_res - 1)) - 0.5) / (PI / 2);
  iqfac_m = (((WORD64)1 << (coeff_res - 1)) + 0.5) / (PI / 2);

  /* Quantize and inverse quantize */
  for (i = 1; i <= f_order; i++)
  {
    ptr_index[i] = (WORD32)(0.5 + (asin(ptr_k[i]) * ((ptr_k[i] >= 0) ? iqfac : iqfac_m)));
    ptr_k[i] = sin((FLOAT64)ptr_index[i] / ((ptr_index[i] >= 0) ? iqfac : iqfac_m));
  }
  return;
}

/**
 *  impeghe_tns_auto_corr
 *
 *  \brief Auto-correlation
 *
 *  \param [in] max_order	Maximum order
 *  \param [in] data_size	Data size
 *  \param [in] ptr_data		Input data
 *  \param [in] ptr_r		Auto correlation values
 *
 *  \return VOID
 */
static VOID impeghe_tns_auto_corr(WORD32 max_order, WORD32 data_size, FLOAT64 *ptr_data,
                                  FLOAT64 *ptr_r)
{

  WORD32 i, j;
  FLOAT64 tmp_var;
  for (i = 0; i < data_size; i += 2)
  {
    const FLOAT64 *input1 = &ptr_data[i];
    FLOAT64 temp1 = *input1;
    FLOAT64 temp2 = *(input1 + 1);
    FLOAT64 inp_tmp1 = *input1++;
    for (j = 0; j <= max_order; j++)
    {

      FLOAT64 inp_tmp2;
      tmp_var = temp1 * inp_tmp1;
      inp_tmp2 = *input1++;
      tmp_var += temp2 * inp_tmp2;
      ptr_r[j] += tmp_var;
      j++;
      tmp_var = temp1 * inp_tmp2;
      inp_tmp1 = *input1++;
      tmp_var += temp2 * inp_tmp1;
      ptr_r[j] += tmp_var;
    }
  }
  return;
}

/**
 *  impeghe_levinson_durbin
 *
 *  \brief Levinson durbin algo
 *
 *  \param [in] order		Filter order
 *  \param [in] data_size	Data size
 *  \param [in] ptr_data		Input data
 *  \param [in] ptr_k		Reflection coeffs
 *  \param [in] ptr_win		Pointer to window coeffs
 *  \param [out] ptr_scratch	Pointer to scratch memory
 *
 *  \return FLOAT64			TNS prediction gain
 */
static FLOAT64 impeghe_levinson_durbin(WORD32 order, WORD32 data_size, FLOAT64 *ptr_data,
                                       FLOAT64 *ptr_k, FLOAT64 *ptr_win, FLOAT64 *ptr_scratch)
{
  WORD32 i, j;
  FLOAT64 *ptr_work_buffer_temp;
  FLOAT64 *ptr_work_buffer = ptr_scratch;
  FLOAT64 *ptr_input = ptr_scratch + TNS_MAX_ORDER + 1;
  memset(ptr_input, 0, (TNS_MAX_ORDER + 1) * sizeof(ptr_input[0]));
  impeghe_tns_auto_corr(order, data_size, ptr_data, ptr_input);

  WORD32 num_of_coeff = order;
  FLOAT64 *ptr_refl_coeff = ptr_k;
  ptr_k[0] = 1.0;

  if (ptr_input[0] == 0)
  {
    return 0;
  }

  for (i = 0; i < num_of_coeff + 1; i++)
  {

    ptr_input[i] = ptr_input[i] * ptr_win[i];
  }

  FLOAT64 tmp_var;
  ptr_work_buffer[0] = ptr_input[0];

  for (i = 1; i < num_of_coeff; i++)
  {
    tmp_var = ptr_input[i];
    ptr_work_buffer[i] = tmp_var;
    ptr_work_buffer[i + num_of_coeff - 1] = tmp_var;
  }
  ptr_work_buffer[i + num_of_coeff - 1] = ptr_input[i];

  for (i = 0; i < num_of_coeff; i++)
  {
    FLOAT64 refc, tmp;
    tmp = ptr_work_buffer[num_of_coeff + i];
    if (tmp < 0)
    {
      tmp = -tmp;
    }
    else
    {
      if (ptr_work_buffer[0] < tmp)
      {
        break;
      }
    }
    if (ptr_work_buffer[0] == 0)
    {
      refc = 0;
    }
    else
    {
      refc = tmp / ptr_work_buffer[0];
    }

    if (ptr_work_buffer[num_of_coeff + i] > 0)
    {
      refc = -refc;
    }
    ptr_refl_coeff[i + 1] = refc;
    ptr_work_buffer_temp = &(ptr_work_buffer[num_of_coeff]);

    for (j = i; j < num_of_coeff; j++)
    {
      FLOAT64 accu1, accu2;
      accu1 = refc * ptr_work_buffer[j - i];
      accu1 += ptr_work_buffer_temp[j];
      accu2 = refc * ptr_work_buffer_temp[j];
      accu2 += ptr_work_buffer[j - i];
      ptr_work_buffer_temp[j] = accu1;
      ptr_work_buffer[j - i] = accu2;
    }
  }
  return (ptr_input[0] / ptr_work_buffer[0]);
}

/**
 *  impeghe_step_up
 *
 *  \brief Calculate prediction coeffs
 *
 *  \param [in] f_order		Order
 *  \param [in] ptr_k		Reflection coeffs
 *  \param [out] ptr_a	Prediction coeffs
 *  \param [out] ptr_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
static VOID impeghe_step_up(WORD32 f_order, FLOAT64 *ptr_k, FLOAT64 *ptr_a, FLOAT64 *ptr_scratch)
{
  FLOAT64 *ptr_a_temp = ptr_scratch;
  WORD32 i, order;

  ptr_a[0] = 1.0;
  ptr_a_temp[0] = 1.0;
  for (order = 1; order <= f_order; order++)
  {
    ptr_a[order] = 0.0;
    for (i = 1; i <= order; i++)
    {
      ptr_a_temp[i] = ptr_a[i] + ptr_k[order] * ptr_a[order - i];
    }
    for (i = 1; i <= order; i++)
    {
      ptr_a[i] = ptr_a_temp[i];
    }
  }
  return;
}

/**
 *  impeghe_calc_weighted_spec
 *
 *  \brief Calculate weighted spectrum
 *
 *  \param [in] ptr_spec		Pointer to spectrum
 *  \param [in] ptr_wgt_spec	Pointer to weighted spectrum
 *  \param [in] ptr_sfb_en		Pointer to sfb energy
 *  \param [in] ptr_sfb_offset	offset table for scalefactor band
 *  \param [in] lpc_start_band	LPC start band index
 *  \param [in] lpc_stop_band	LCP stop band index
 *  \param [out] ptr_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
static VOID impeghe_calc_weighted_spec(FLOAT64 *ptr_spec, FLOAT64 *ptr_wgt_spec,
                                       FLOAT32 *ptr_sfb_en, WORD32 *ptr_sfb_offset,
                                       WORD32 lpc_start_band, WORD32 lpc_stop_band,
                                       FLOAT64 *ptr_scratch)
{
  WORD32 i, sfb;
  FLOAT32 temp;
  FLOAT32 *ptr_tns_sfb_mean = (FLOAT32 *)ptr_scratch;
  memset(ptr_scratch, 0, MAX_NUM_GROUPED_SFB * sizeof(ptr_tns_sfb_mean[0]));
  WORD32 lpc_stop_line = ptr_sfb_offset[lpc_stop_band];
  WORD32 lpc_start_line = ptr_sfb_offset[lpc_start_band];

  for (sfb = lpc_start_band; sfb < lpc_stop_band; sfb++)
  {
    ptr_tns_sfb_mean[sfb] = (FLOAT32)(1.0 / sqrt(ptr_sfb_en[sfb] + 1e-30f));
  }

  sfb = lpc_start_band;
  temp = ptr_tns_sfb_mean[sfb];

  for (i = lpc_start_line; i < lpc_stop_line; i++)
  {
    if (ptr_sfb_offset[sfb + 1] == i)
    {

      sfb++;

      if (sfb + 1 < lpc_stop_band)
      {
        temp = ptr_tns_sfb_mean[sfb];
      }
    }
    ptr_wgt_spec[i] = temp;
  }

  for (i = lpc_stop_line - 2; i >= lpc_start_line; i--)
  {
    ptr_wgt_spec[i] = (ptr_wgt_spec[i] + ptr_wgt_spec[i + 1]) * 0.5f;
  }

  for (i = lpc_start_line + 1; i < lpc_stop_line; i++)
  {
    ptr_wgt_spec[i] = (ptr_wgt_spec[i] + ptr_wgt_spec[i - 1]) * 0.5f;
  }

  for (i = lpc_start_line; i < lpc_stop_line; i++)
  {
    ptr_wgt_spec[i] = ptr_wgt_spec[i] * ptr_spec[i];
  }
  return;
}

/**
 *  impeghe_tns_data_sync
 *
 *  \brief Sync TNS data between 2 channels
 *
 *  \param [out] ptr_tns_dest	Pointer to 2nd channel TNS structure
 *  \param [in] ptr_tns_src		Pointer to 1st channel TNS structure
 *  \param [in] w				Window index
 *  \param [in] order			Filter order
 *
 *  \return VOID
 */
static VOID impeghe_tns_data_sync(ia_tns_info *ptr_tns_dest, ia_tns_info *ptr_tns_src,
                                  const WORD32 w, WORD32 order)
{
  ia_tns_window_data *ptr_win_data_src = &ptr_tns_src->window_data[w];
  ia_tns_window_data *ptr_win_data_dest = &ptr_tns_dest->window_data[w];
  WORD32 i;
  if (fabs(ptr_win_data_dest->tns_pred_gain - ptr_win_data_src->tns_pred_gain) <
      ((FLOAT32)0.03f * ptr_win_data_dest->tns_pred_gain))
  {

    ptr_win_data_dest->tns_active = ptr_win_data_src->tns_active;

    for (i = 0; i < order; i++)
    {

      ptr_win_data_dest->tns_filter->k_coeffs[i] = ptr_win_data_src->tns_filter->k_coeffs[i];
    }
  }
  return;
}
/**
 *  impeghe_tns_encode
 *
 *  \brief TNS processing
 *
 *  \param [in,out] pstr_tns_info_ch2	Pointer to 2nd channel TNS structure
 *  \param [in,out] pstr_tns_info		Pointer to 1st channel TNS structure
 *  \param [in] psfb_energy			Pointer to sfb energy
 *  \param [in] w					Window index
 *  \param [in] i_ch				Channel index
 *  \param [in] low_pass_line		Low pass line index
 *  \param [in] pstr_igf_config		Pointer to IGF config structure
 *  \param [in] ptr_scratch_tns_filter	Pointer to filter scratch memory
 *  \param [in] core_mode	Core coding mode
 *  \param [in] ptr_tns_scratch	Pointer to scratch memory
 *
 *  \return VOID
 */
VOID impeghe_tns_encode(ia_tns_info *pstr_tns_info_ch2, ia_tns_info *pstr_tns_info,
                        FLOAT32 *ptr_sfb_energy, WORD32 w, WORD32 i_ch, WORD32 low_pass_line,
                        ia_igf_config_struct *pstr_igf_config, FLOAT64 *ptr_scratch_tns_filter,
                        WORD32 core_mode, FLOAT64 *ptr_tns_scratch)
{
  WORD32 number_of_bands = pstr_tns_info->number_of_bands;
  WORD32 block_type = pstr_tns_info->block_type;
  FLOAT64 *ptr_spec = pstr_tns_info->spec;
  WORD32 start_band, stop_band, order; /**< bands over which to apply TNS */
  WORD32 length_in_bands;              /**< Length to filter, in bands */
  WORD32 start_index, length;
  WORD32 nbands;
  WORD32 coeff_res;
  FLOAT64 *ptr_weighted_spec = ptr_tns_scratch;
  memset(ptr_weighted_spec, 0, 4096 * sizeof(ptr_weighted_spec[0]));
  FLOAT64 *ptr_scratch = ptr_tns_scratch + 4096;
  FLOAT64 *ptr_window = NULL;
  WORD32 lpc_start_band, lpc_stop_band;
  WORD32 *ptr_sfb_offset_table;

  switch (block_type)
  {
  case EIGHT_SHORT_SEQUENCE:
    start_band = pstr_tns_info->tns_min_band_number_short;
    stop_band = number_of_bands;
    length_in_bands = stop_band - start_band;
    order = pstr_tns_info->tns_max_order_short;
    start_band = min(start_band, pstr_tns_info->tns_max_bands_short);
    stop_band = min(stop_band, pstr_tns_info->tns_max_bands_short);
    coeff_res = 3;
    ptr_window = pstr_tns_info->win_short;
    nbands = pstr_tns_info->max_sfb_short;
    lpc_start_band = pstr_tns_info->lpc_start_band_short;
    lpc_stop_band = pstr_tns_info->lpc_stop_band_short;
    if (core_mode == CORE_MODE_FD)
    {
      ptr_sfb_offset_table = pstr_tns_info->sfb_offset_table_short;
    }
    else
    {
      ptr_sfb_offset_table = pstr_tns_info->sfb_offset_table_short_tcx;
    }
    break;

  default:
    start_band = pstr_tns_info->tns_min_band_number_long;
    stop_band = number_of_bands;
    length_in_bands = stop_band - start_band;
    order = pstr_tns_info->tns_max_order_long;
    start_band = min(start_band, pstr_tns_info->tns_max_bands_long);
    stop_band = min(stop_band, pstr_tns_info->tns_max_bands_long);
    coeff_res = 4;
    ptr_window = pstr_tns_info->win_long;
    nbands = pstr_tns_info->max_sfb_long;
    lpc_start_band = pstr_tns_info->lpc_start_band_long;
    lpc_stop_band = pstr_tns_info->lpc_stop_band_long;
    ptr_sfb_offset_table = pstr_tns_info->sfb_offset_table_long;
    break;
  }

  if (pstr_igf_config->igf_active == 1)
  {
    if (pstr_igf_config->igf_after_tns_synth == 0)
    {
      if (block_type != EIGHT_SHORT_SEQUENCE)
      {
        if (pstr_igf_config->igf_stop_sfb_lb > nbands)
        {
          nbands = pstr_igf_config->igf_stop_sfb_lb;
        }
      }
      else
      {
        if (pstr_igf_config->igf_stop_sfb_sb > nbands)
        {
          nbands = pstr_igf_config->igf_stop_sfb_sb;
        }
      }
    }
    else
    {
      if (block_type != EIGHT_SHORT_SEQUENCE)
      {
        if (pstr_igf_config->igf_start_sfb_lb < nbands)
        {
          nbands = pstr_igf_config->igf_start_sfb_lb;
        }
      }
      else
      {
        if (pstr_igf_config->igf_start_sfb_sb < nbands)
        {
          nbands = pstr_igf_config->igf_start_sfb_sb;
        }
      }
    }
  }

  /** Make sure that start and stop bands < max_sfb
   * Make sure that start and stop bands >= 0
   */
  start_band = min(start_band, nbands);
  stop_band = min(stop_band, nbands);
  start_band = max(start_band, 0);
  stop_band = max(stop_band, 0);

  pstr_tns_info->tns_data_present = 0; /**< default TNS not used */

  /** Perform analysis and filtering for each window
   */
  {

    ia_tns_window_data *window_data = &pstr_tns_info->window_data[w];
    ia_tns_filter_data *tns_filter = window_data->tns_filter;
    FLOAT64 *k = tns_filter->k_coeffs; /**< reflection coeffs */
    FLOAT64 *a = tns_filter->a_coeffs; /**< prediction coeffs */

    impeghe_calc_weighted_spec(ptr_spec, ptr_weighted_spec, ptr_sfb_energy, ptr_sfb_offset_table,
                               lpc_start_band, lpc_stop_band, ptr_scratch);

    window_data->n_filt = 0;
    window_data->coef_res = coeff_res;

    start_index = ptr_sfb_offset_table[lpc_start_band];
    length = ptr_sfb_offset_table[lpc_stop_band] -
             ptr_sfb_offset_table[lpc_start_band]; /**< The length of the spectral data to be
                                                      processed */

    window_data->tns_pred_gain = impeghe_levinson_durbin(
        order, length, &ptr_weighted_spec[start_index], k, ptr_window, ptr_scratch);

    window_data->tns_active = 0;
    if (window_data->tns_pred_gain > DEF_TNS_GAIN_THRESH)
    {
      window_data->tns_active = 1;
    }

    if (i_ch == 1)
    {
      impeghe_tns_data_sync(pstr_tns_info, pstr_tns_info_ch2, w, order);
    }

    if (window_data->tns_pred_gain > DEF_TNS_GAIN_THRESH)
    {
      /** Use TNS
       */
      WORD32 truncated_order;
      window_data->n_filt++;
      pstr_tns_info->tns_data_present = 1;
      tns_filter->direction = 0;
      tns_filter->coef_compress = 0;
      tns_filter->length = length_in_bands;
      impeghe_quantize_reflection_coeffs(order, coeff_res, k, tns_filter->index);
      truncated_order = impeghe_truncate_coeffs(order, DEF_TNS_COEFF_THRESH, k);
      tns_filter->order = truncated_order;
      impeghe_step_up(truncated_order, k, a, ptr_scratch); /**< Compute prediction coefficients */
      start_index = ptr_sfb_offset_table[start_band];
      length = min(ptr_sfb_offset_table[stop_band], low_pass_line) - start_index;
      if (block_type == EIGHT_SHORT_SEQUENCE)
      {
        length = ptr_sfb_offset_table[stop_band] - start_index;
      }
      impeghe_tns_filter(length, &ptr_spec[start_index], tns_filter,
                         ptr_scratch_tns_filter); /**< filter */
    }
  }
  return;
}
