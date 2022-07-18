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
#include "impeghe_cnst.h"

#include "impeghe_avq_enc.h"
#include "impeghe_fd_quant.h"
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
#include "impeghe_bitbuffer.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_stereo_lpd_rom.h"
#include "impeghe_fft.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */
#define SLPD_COD_VEC_LEN (8)
#define DELTA ((FLOAT32)PI / 4.0f)
#define COD_BAND_THRESHOLD (0.001f)

/**
 *  impeghe_sldp_calc_ipd_ild
 *
 *  \brief Calculate inter-channel phase difference
 *
 *  \param [in] ptr_cplx_in1	Pointer to channel 1 complex spectral coefficients
 *  \param [in] ptr_cplx_in2	Pointer to channel 2 complex spectral coefficients
 *  \param [in] sig_len		Processing length of the input
 *
 *  \return FLOAT32         Interchannel phase difference
 */

static FLOAT32 impeghe_sldp_calc_ipd(FLOAT32 *ptr_cplx_in1, FLOAT32 *ptr_cplx_in2, WORD32 sig_len)
{
  WORD32 i;

  FLOAT32 ipd = 0.0f;
  FLOAT64 xcor_real = 0.0f, xcor_imag = 0.0f;

  for (i = 0; i < sig_len; i++)
  {
    xcor_real += ptr_cplx_in1[2 * i + 0] * ptr_cplx_in2[2 * i + 0] +
                 ptr_cplx_in1[2 * i + 1] * ptr_cplx_in2[2 * i + 1];
    xcor_imag += -ptr_cplx_in2[2 * i + 1] * ptr_cplx_in1[2 * i + 0] +
                 ptr_cplx_in1[2 * i + 1] * ptr_cplx_in2[2 * i + 0];
  }

  ipd = (FLOAT32)atan2(xcor_imag, xcor_real);
  ipd = (FLOAT32)(ipd > 0 ? ipd : 2. * PI + ipd);

  return ipd;
}

/**
 *  impeghe_calc_real_sig_nrg
 *
 *  \brief Computes energy of a complex valued input signal
 *
 *  \param [in] ptr_in_real Pointer to input real signal.
 *  \param [in] sig_len     Input signal length.
 *
 *  \return FLOAT32 Energy value.
 */
static FLOAT32 impeghe_calc_real_sig_nrg(FLOAT32 *ptr_in_real, WORD32 sig_len)
{
  WORD32 i = 0;
  FLOAT32 energy = 0;
  for (i = 0; i < sig_len; i++)
  {
    energy += ptr_in_real[i] * ptr_in_real[i];
  }
  return energy;
}

/**
 *  impeghe_calc_cplx_sig_nrg
 *
 *  \brief Computes energy of a complex valued input signal
 *
 *  \param [in] ptr_in_cplx Pointer to input complex signal.
 *  \param [in] sig_len     Input signal length.
 *
 *  \return FLOAT32 Energy value.
 */
static FLOAT32 impeghe_calc_cplx_sig_nrg(FLOAT32 *ptr_in_cplx, WORD32 sig_len)
{
  WORD32 i = 0;
  FLOAT32 energy = 0;
  for (i = 0; i < sig_len; i++)
  {
    energy += ptr_in_cplx[2 * i + 0] * ptr_in_cplx[2 * i + 0];
    energy += ptr_in_cplx[2 * i + 1] * ptr_in_cplx[2 * i + 1];
  }
  return energy;
}

/**
 *  impeghe_calc_slpd_band_nrgs
 *
 *  \brief Helper function to calculate SLPD DFT band energies.
 *
 *  \param [out] ptr_bands_nrg_buf Pointer to slpd band energy output buffer.
 *  \param [in ] ptr_dft_buf       Pointer to DFT data buffer.
 *  \param [in ] ptr_bands_tbl     Pointer to SLPD bands table.
 *  \param [in ] num_bands         Number of bands.
 *
 *  \return VOID
 *
 */
VOID impeghe_calc_slpd_band_nrgs(FLOAT32 *ptr_bands_nrg_buf, FLOAT32 *ptr_dft_buf,
                                 WORD32 *ptr_bands_tbl, WORD32 num_bands, WORD32 cplx_flag)
{
  WORD32 i = 0;
  if (cplx_flag)
  {
    for (i = 0; i < num_bands; i++)
    {
      ptr_bands_nrg_buf[i] =
          impeghe_calc_cplx_sig_nrg(&ptr_dft_buf[ptr_bands_tbl[i]], ptr_bands_tbl[i + 1]);
    }
  }
  else
  {
    for (i = 0; i < num_bands; i++)
    {
      ptr_bands_nrg_buf[i] =
          impeghe_calc_real_sig_nrg(&ptr_dft_buf[ptr_bands_tbl[i]], ptr_bands_tbl[i + 1]);
    }
  }
}

/**
 *  impeghe_quantize_side_signal_avq
 *
 *  \brief Carries out quantization of side signal sectral values using AVQ.
 *
 *  \param [in ] ptr_lsf       Pointer to LSF data.
 *  \param [out] ptr_params    Pointer to parameter buffer.
 *  \param [in ] num_dft_lines Length of vector.
 *
 *  \return VOID
 */
VOID impeghe_quantize_side_signal_avq(FLOAT32 *ptr_lsf, WORD32 *ptr_params, WORD32 num_dft_lines)
{
  WORD32 i;
  for (i = 0; i < num_dft_lines; i += 8)
  {
    impeghe_find_nearest_neighbor(&ptr_lsf[i], &ptr_params[i]);
  }
  return;
}

/**
 *  impeghe_slpd_band_config
 *
 *  \brief Calculate Stereo LPD band limits
 *
 *  \param [in,out]		band_limit	Stereo band info bandlimit
 *  \param [in]			res_mode	Stereo LPD ERB mode
 *  \param [in]			N			DFT size
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_slpd_band_config(WORD32 *band_limits, WORD32 res_mode, WORD32 N)
{
  WORD32 num_bands = 0;

  band_limits[0] = 1;
  if (0 == res_mode)
  {
    while (band_limits[num_bands++] < (N / 2))
    {
      band_limits[num_bands] = (WORD32)impeghe_band_lim_erb2[num_bands];
    }
  }
  else
  {
    while (band_limits[num_bands++] < (N / 2))
    {
      band_limits[num_bands] = (WORD32)impeghe_band_lim_erb4[num_bands];
    }
  }

  num_bands--;
  band_limits[num_bands] = N / 2;

  return num_bands;
}

/**
 *  impeghe_unary_code
 *
 *  \brief Utility function to implement unary code encoding.
 *
 *  \param [in] idx         Number of 1s to be inserted.
 *  \param [in] it_bit_buf Pointer to bit buffer handle.
 *
 *  \return WORD32 Number of bits value.
 */
static VOID impeghe_unary_code(WORD32 idx, ia_bit_buf_struct *it_bit_buf)
{

  idx -= 1;
  while (idx-- > 0)
  {
    impeghe_write_bits_buf(it_bit_buf, 1, 1);
  }
  impeghe_write_bits_buf(it_bit_buf, 0, 1);
}

/**
 *  impeghe_slpd_code_book_indices
 *
 *  \brief Calculate Stereo LPD code book indices
 *
 *  \param [in] it_bit_buf   Pointer to bitstream handler.
 *  \param [in]	ptr_params    Pointer to params buffer.
 *  \param [in] num_dft_lines Number of DFT lines.
 *
 *  \return VOID
 *
 */
static VOID impeghe_slpd_code_book_indices(ia_bit_buf_struct *it_bit_buf, WORD32 *ptr_params,
                                           WORD32 num_dft_lines)
{
  WORD32 i, j, n, qn, kv[8], nk;
  WORD32 I;
  for (i = 0; i < num_dft_lines; i += 8)
  {
    impeghe_apply_voronoi_ext(&ptr_params[i], &qn, &I, kv);

    impeghe_unary_code(qn, it_bit_buf);

    nk = 0;
    n = qn;
    if (qn > 4)
    {
      nk = (qn - 3) >> 1;
      n = qn - nk * 2;
    }

    impeghe_write_bits_buf(it_bit_buf, I, 4 * n);
    for (j = 0; j < 8; j++)
    {
      impeghe_write_bits_buf(it_bit_buf, kv[j], nk);
    }
  }
  return;
}

/**
 *  impeghe_slpd_init
 *
 *  \brief Initialize Stereo LPD data
 *
 *  \param [in,out] pstr_slpd_enc_data Pointer to stereo LPD data.
 *  \param [in]     fs                 Sampling frequency.
 *  \param [in]     full_band_lpd      Flag to check full band LPD.
 *  \param [in]     ccfl               Frame length.
 *
 *  \return VOID
 *
 */
VOID impeghe_slpd_init(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data, WORD32 fs,
                       WORD32 full_band_lpd, WORD32 ccfl)
{
  ia_usac_slpd_bitstream_str *pstr_slpd_bs = &pstr_slpd_enc_data->lpd_stereo_bitstream;
  ia_usac_slpd_band_info_str *band_info = &pstr_slpd_enc_data->lpd_stereo_bandinfo;
  pstr_slpd_enc_data->lpd_stereo_config.fs = fs;
  pstr_slpd_enc_data->lpd_stereo_config.ccfl = ccfl;
  if (1 == full_band_lpd)
  {
    pstr_slpd_enc_data->fac_fb = 2;
    pstr_slpd_enc_data->win = impeghe_sin_window160;
    pstr_slpd_enc_data->lpd_stereo_config.dft_size = 672;
    pstr_slpd_enc_data->lpd_stereo_config.frame_size = 512;
    pstr_slpd_enc_data->lpd_stereo_config.overlap_size = 160;
  }
  else
  {
    pstr_slpd_enc_data->fac_fb = 1;
    pstr_slpd_enc_data->win = impeghe_sin_window80;
    pstr_slpd_enc_data->lpd_stereo_config.dft_size = 336;
    pstr_slpd_enc_data->lpd_stereo_config.frame_size = 256;
    pstr_slpd_enc_data->lpd_stereo_config.overlap_size = 80;
  }
  pstr_slpd_enc_data->lpd_stereo_config.num_subframes =
      ccfl / pstr_slpd_enc_data->lpd_stereo_config.frame_size;

  band_info->num_bands = impeghe_slpd_band_config(band_info->band_limits, pstr_slpd_bs->res_mode,
                                                  pstr_slpd_enc_data->lpd_stereo_config.dft_size);
  band_info->cod_band_max = impeghe_max_band[pstr_slpd_bs->res_mode][pstr_slpd_bs->cod_mode];
  band_info->ipd_band_max = impeghe_max_band[pstr_slpd_bs->res_mode][pstr_slpd_bs->ipd_mode];
  band_info->side_dft_lines = (band_info->band_limits[band_info->cod_band_max] - 1) << 1;

  switch (pstr_slpd_enc_data->lpd_stereo_config.dft_size)
  {
  case 672:
    pstr_slpd_enc_data->p_slpd_sin_table = impeghe_slpd_sintable_336;
    break;
  case 336:
    pstr_slpd_enc_data->p_slpd_sin_table = impeghe_slpd_sintable_168;
    break;
  }
  pstr_slpd_enc_data->init_flag = 1;
}

/**
 *  impeghe_find_slpd_param_idx
 *
 *  \brief Finds the index of a
 *
 *  \param [in]	ild_value	  Pointer to Stereo LPD structure.
 *  \param [in] num_entries Number of entries in the table.
 *  \param [in] pstr_table  Pointer to table.
 *
 *  \return WORD32 index value
 *
 */
static WORD32 impeghe_find_slpd_param_idx(FLOAT32 ild_value, WORD32 num_entries,
                                          const FLOAT32 *pstr_table)
{
  WORD32 idx = 0;
  for (idx = 0; idx < num_entries; idx++)
  {
    if (ild_value <= pstr_table[idx])
    {
      return idx;
    }
  }
  return (num_entries - 1);
}

/**
 *  impeghe_slpd_data_quant
 *
 *  \brief SLPD data quantization
 *
 *  \param [in,out] pstr_slpd_enc_data Pointer to Stereo LPD structure.
 *  \param [in ]    ptr_slpd_scratch   Pointer to Stereo LPD scratch structure.
 *
 *  \return VOID
 *
 */
static VOID impeghe_slpd_data_quant(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data,
                                    ia_slpd_scratch_str *ptr_slpd_scratch)
{
  WORD32 i, band;
  WORD32 ipd_band_max = 0;
  ia_usac_slpd_bitstream_str *lpd_stereo_bitstream = &pstr_slpd_enc_data->lpd_stereo_bitstream;
  ia_usac_slpd_param_str *lpd_stereo_parameter = &pstr_slpd_enc_data->lpd_stereo_parameter;
  ia_usac_slpd_band_info_str *lpd_stereo_bandinfo = &pstr_slpd_enc_data->lpd_stereo_bandinfo;
  ia_usac_slpd_config_str *lpd_stereo_config = &pstr_slpd_enc_data->lpd_stereo_config;

  WORD32 num_div = lpd_stereo_config->ccfl / lpd_stereo_config->frame_size;
  /* quantization */
  for (i = num_div - 1; i >= 0; i--)
  {
    if ((lpd_stereo_bitstream->q_mode == 0) || ((i & 1) == 1))
    {
      WORD32 num_entries = STEREO_LPD_MAX_ILD_IDX + 1;
      for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
      {
        lpd_stereo_bitstream->ild_idx[i][band] = impeghe_find_slpd_param_idx(
            lpd_stereo_parameter->ild[i][band], num_entries, &impeghe_ild_qtable[0]);
        if (lpd_stereo_bitstream->ild_idx[i][band] > STEREO_LPD_MAX_ILD_IDX)
        {
          lpd_stereo_bitstream->ild_idx[i][band] = STEREO_LPD_MAX_ILD_IDX;
        }
        // The addition with DELTA/2 is to take care of rounding.
        lpd_stereo_bitstream->ipd_idx[i][band] =
            (WORD32)((lpd_stereo_parameter->ipd[i][band] + (FLOAT32)PI + (DELTA / 2)) / DELTA);
        if ((lpd_stereo_bitstream->ipd_idx[i][band] != 4) && band > ipd_band_max)
        {
          ipd_band_max = band;
        }
      }

      if (lpd_stereo_bitstream->pred_mode)
      {
        WORD32 num_entries = STEREO_LPD_RES_PRED_GAIN_TBL_SZ;
        for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
        {
          lpd_stereo_bitstream->pred_gain_idx[i][band] =
              impeghe_find_slpd_param_idx(lpd_stereo_parameter->pred_gain[i][band], num_entries,
                                          &impeghe_res_pred_gain_qtable[0]);
        }
      }
    }
    impeghe_quantize_side_signal_avq(&ptr_slpd_scratch->dft_side[i][0],
                                     &lpd_stereo_parameter->params[0],
                                     lpd_stereo_bandinfo->side_dft_lines);
  }
  lpd_stereo_bitstream->cod_mode = 0;
  for (i = 0; i < SLPD_MAX_BAND_TBL_SZ; i++)
  {
    if (lpd_stereo_bandinfo->cod_band_max <= impeghe_max_band[lpd_stereo_bitstream->res_mode][i])
    {
      lpd_stereo_bandinfo->cod_band_max = impeghe_max_band[lpd_stereo_bitstream->res_mode][i];
      lpd_stereo_bitstream->cod_mode = i;
      break;
    }
  }
  if (i == SLPD_MAX_BAND_TBL_SZ)
  {
    lpd_stereo_bandinfo->cod_band_max = impeghe_max_band[lpd_stereo_bitstream->res_mode][i - 1];
    lpd_stereo_bitstream->cod_mode = i - 1;
  }
  lpd_stereo_bitstream->ipd_mode = 0;
  for (i = 0; i < SLPD_MAX_BAND_TBL_SZ; i++)
  {
    if (ipd_band_max <= impeghe_max_band[lpd_stereo_bitstream->res_mode][i])
    {
      lpd_stereo_bandinfo->ipd_band_max = impeghe_max_band[lpd_stereo_bitstream->res_mode][i];
      lpd_stereo_bitstream->ipd_mode = i;
      break;
    }
  }
  if (i == SLPD_MAX_BAND_TBL_SZ)
  {
    lpd_stereo_bandinfo->ipd_band_max = impeghe_max_band[lpd_stereo_bitstream->res_mode][i - 1];
    lpd_stereo_bitstream->ipd_mode = i - 1;
  }
}

/**
 *  impeghe_slpd_data_write
 *
 *  \brief Write SLPD data to bit stream
 *
 *  \param [in,out]	pstr_slpd_enc_data Pointer to stereo LPD data structure.
 *  \param [in ]    it_bit_buff		     Pointer to bit buffer handle.
 *
 *  \return IA_ERRORCODE Error code if any
 *
 */
IA_ERRORCODE impeghe_slpd_data_write(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data,
                                     ia_bit_buf_struct *it_bit_buff)
{
  WORD32 i, band;
  ia_usac_slpd_config_str *pstr_lpd_stereo_config = &pstr_slpd_enc_data->lpd_stereo_config;
  ia_usac_slpd_bitstream_str *lpd_stereo_bitstream = &pstr_slpd_enc_data->lpd_stereo_bitstream;
  ia_usac_slpd_param_str *lpd_stereo_parameter = &pstr_slpd_enc_data->lpd_stereo_parameter;
  ia_usac_slpd_band_info_str *lpd_stereo_bandinfo = &pstr_slpd_enc_data->lpd_stereo_bandinfo;
  WORD32 num_div = pstr_lpd_stereo_config->num_subframes;

  /* read lpd stream data */
  impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->res_mode, 1);
  impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->q_mode, 1);
  impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->ipd_mode, 2);
  impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->pred_mode, 1);
  impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->cod_mode, 2);

  for (i = num_div - 1; i >= 0; i--)
  {
    if ((lpd_stereo_bitstream->q_mode == 0) || (i % 2 == 1))
    {
      for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
      {
        impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->ild_idx[i][band], 5);
      }

      for (band = 0; band < lpd_stereo_bandinfo->ipd_band_max; band++)
      {
        impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->ipd_idx[i][band], 3);
      }

      if (1 == lpd_stereo_bitstream->pred_mode)
      {
        for (band = lpd_stereo_bandinfo->cod_band_max; band < lpd_stereo_bandinfo->num_bands;
             band++)
        {
          impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->pred_gain_idx[i][band], 3);
        }
      }
    }

    if (lpd_stereo_bitstream->cod_mode > 0)
    {
      impeghe_write_bits_buf(it_bit_buff, lpd_stereo_bitstream->cod_gain_idx[i], 7);

      impeghe_slpd_code_book_indices(it_bit_buff, &lpd_stereo_parameter->params[0],
                                     lpd_stereo_bandinfo->side_dft_lines);
    }
  }
  return -1;
}

/**
 *  impeghe_enc_slpd_params
 *
 *  \brief Encode SLPD data and write into bit stream
 *
 *  \param [in,out] pstr_slpd_enc_data Pointer to Stereo LPD structure.
 *  \param [in]     it_bit_buff        Bit buffer stream.
 *  \param [in]     td_start_flag      Flag to check if prev frame is TD.
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghe_enc_slpd_params(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data,
                                     ia_slpd_scratch_str *ptr_slpd_scratch,
                                     ia_bit_buf_struct *it_bit_buff, WORD32 td_start_flag)
{
  IA_ERRORCODE error_code = 0;

  /* Quantization */
  impeghe_slpd_data_quant(pstr_slpd_enc_data, ptr_slpd_scratch);

  /* read lpd stream data */
  error_code = impeghe_slpd_data_write(pstr_slpd_enc_data, it_bit_buff);

  return error_code;
}

/**
 *  impeghe_enc_stereo_lpd
 *
 *  \brief Apply stereo LPD and update stereo buffer
 *
 *  \param [in,out] pstr_slpd_enc_data Stereo lpd structure.
 *  \param [in]     pstr_slpd_scratch  Pointer to slpd scratch structure.
 *  \param [in]		  ptr_scratch_mem    Pointer to LPD scratch memory.
 *
 *  \return VOID
 *
 */

VOID impeghe_enc_stereo_lpd(ia_usac_slpd_enc_data_str *pstr_slpd_enc_data,
                            ia_slpd_scratch_str *pstr_slpd_scratch,
                            impeghe_scratch_mem *pstr_scratch_mem)
{
  ia_usac_slpd_config_str *pstr_slpd_cfg = &pstr_slpd_enc_data->lpd_stereo_config;
  WORD32 k, i;
  WORD32 num_subframes = pstr_slpd_cfg->num_subframes;
  WORD32 dft_size = pstr_slpd_cfg->dft_size;
  WORD32 past = pstr_slpd_cfg->dft_size - 256;
  WORD32 cod_band_max = 0;
  const FLOAT32 *ptr_sin_tbl = pstr_slpd_enc_data->p_slpd_sin_table;
  ia_usac_slpd_bitstream_str *pstr_slpd_bs = &pstr_slpd_enc_data->lpd_stereo_bitstream;
  ia_usac_slpd_band_info_str *pstr_slpd_band_info = &pstr_slpd_enc_data->lpd_stereo_bandinfo;
  if (past > 256)
  {
    past -= 256;
  }

  pstr_slpd_band_info->num_bands = impeghe_slpd_band_config(
      pstr_slpd_band_info->band_limits, pstr_slpd_bs->res_mode, pstr_slpd_cfg->dft_size);

  for (k = 0; k < num_subframes; k++)
  {
    FLOAT32 *ptr_dft_left = &pstr_slpd_scratch->dft_left[0];
    FLOAT32 *ptr_dft_rght = &pstr_slpd_scratch->dft_rght[0];
    FLOAT32 *ptr_dft_dmix = &pstr_slpd_scratch->dft_dmix[0];
    FLOAT32 *ptr_dft_side = &pstr_slpd_scratch->dft_side[k][0];
    FLOAT32 *ptr_time_buf_left = &pstr_slpd_scratch->time_buff_left[k * (dft_size - past)];
    FLOAT32 *ptr_time_buf_rght = &pstr_slpd_scratch->time_buff_right[k * (dft_size - past)];
    WORD32 idx = 0;
    for (i = 0; i < dft_size; i++)
    {
      ptr_dft_left[2 * i + 0] = ptr_time_buf_left[i];
      ptr_dft_left[2 * i + 1] = 0;
      ptr_dft_rght[2 * i + 0] = ptr_time_buf_rght[i];
      ptr_dft_rght[2 * i + 1] = 0;
    }

    impeghe_slpd_fft(&ptr_dft_left[0], ptr_sin_tbl, dft_size, pstr_scratch_mem);
    impeghe_slpd_fft(&ptr_dft_rght[0], ptr_sin_tbl, dft_size, pstr_scratch_mem);

    for (i = 0; i < dft_size; i++)
    {
      ptr_dft_dmix[2 * i + 0] = (ptr_dft_left[2 * i + 0] + ptr_dft_rght[2 * i + 0]) * 0.5f;
      ptr_dft_dmix[2 * i + 1] = (ptr_dft_left[2 * i + 1] + ptr_dft_rght[2 * i + 1]) * 0.5f;
      // Taking only the real part of the DFT.
      ptr_dft_side[i] = (ptr_dft_left[2 * i + 0] - ptr_dft_rght[2 * i + 0]) * 0.5f;
    }

    impeghe_calc_slpd_band_nrgs(&pstr_slpd_scratch->left_chn_nrgs[0], &ptr_dft_left[0],
                                &pstr_slpd_band_info->band_limits[0],
                                pstr_slpd_band_info->num_bands, 1);

    impeghe_calc_slpd_band_nrgs(&pstr_slpd_scratch->rght_chn_nrgs[0], &ptr_dft_rght[0],
                                &pstr_slpd_band_info->band_limits[0],
                                pstr_slpd_band_info->num_bands, 1);

    impeghe_calc_slpd_band_nrgs(&pstr_slpd_scratch->side_chn_nrgs[0], &ptr_dft_side[0],
                                &pstr_slpd_band_info->band_limits[0],
                                pstr_slpd_band_info->num_bands, 0);

    for (i = 0; i < pstr_slpd_band_info->num_bands; i++)
    {
      WORD32 num_bands = pstr_slpd_band_info->band_limits[i];
      pstr_slpd_enc_data->lpd_stereo_parameter.ild[k][i] =
          (FLOAT32)(10.0f * log10((pstr_slpd_scratch->left_chn_nrgs[i] + 1.0e-6f) /
                                  (pstr_slpd_scratch->rght_chn_nrgs[i] + 1.0e-6f)));
      pstr_slpd_enc_data->lpd_stereo_parameter.ipd[k][i] =
          impeghe_sldp_calc_ipd(&ptr_dft_left[idx], &ptr_dft_rght[idx], num_bands);
      idx = pstr_slpd_band_info->band_limits[i];
      if (pstr_slpd_scratch->side_chn_nrgs[i] > COD_BAND_THRESHOLD)
      {
        cod_band_max = i;
      }
    }
  }

  pstr_slpd_bs->q_mode = 0;
  pstr_slpd_bs->res_mode = 0;
  pstr_slpd_bs->pred_mode = 0;
  pstr_slpd_band_info->cod_band_max = cod_band_max;
  pstr_slpd_band_info->side_dft_lines = 2 * (pstr_slpd_band_info->band_limits[cod_band_max] - 1);
  if (pstr_slpd_band_info->side_dft_lines > (2 * (STEREO_LPD_MAX_SPEC_LINE - 1)))
  {
    pstr_slpd_band_info->side_dft_lines = (2 * (STEREO_LPD_MAX_SPEC_LINE - 1));
  }
  for (k = 0; k < num_subframes; k++)
  {
    FLOAT32 cod_gain_value = 0.0f;
    WORD32 cod_gain_idx = 0;
    for (i = 0; i < cod_band_max; i++)
    {
      if (cod_gain_value < (FLOAT32)fabs(pstr_slpd_scratch->dft_side[k][i]))
      {
        cod_gain_value = (FLOAT32)fabs(pstr_slpd_scratch->dft_side[k][i]);
      }
    }
    if (cod_gain_value > 6400.0f)
    {
      cod_gain_value = cod_gain_value / 6400.0f;
      for (i = 0; i < cod_band_max; i++)
      {
        pstr_slpd_scratch->dft_side[k][i] = pstr_slpd_scratch->dft_side[k][i] / cod_gain_value;
      }
    }
    else
    {
      cod_gain_value = 1.0f;
    }
    for (i = 0; i < 128; i++)
    {
      if (cod_gain_value <= ia_cod_gain_tbl[i])
      {
        cod_gain_idx = i;
        break;
      }
    }
    pstr_slpd_bs->cod_gain_idx[k] = cod_gain_idx;
  }
}
/** @} */ /* End of CoreDecProc */