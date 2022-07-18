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

#include "impeghe_fd_qc_util.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_fd_quant.h"
#include "impeghe_ms.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_enc_mct.h"
#include "impeghe_resampler.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_main.h"
#include "impeghe_rom.h"

/**
 *  impeghe_write_scf_data
 *
 *  \brief Write scalefactors to bit stream
 *
 *  \param [in,out] it_bit_buf     Bit buffer
 *  \param [in] max_sfb            Max number of scalefactor bands
 *  \param [in] num_sfb            Number of scalefactor bands
 *  \param [in] ptr_scale_factors  Pointer to scalefactors
 *  \param [in] num_win_grps       Number of window groups
 *  \param [in] global_gain        Global gain
 *  \param [in] huff_tab           Huffman code table
 *
 *  \return WORD32			       Bits consumed for writing scalefactor data
 *
 */
WORD32 impeghe_write_scf_data(ia_bit_buf_struct *it_bit_buf, WORD32 max_sfb, WORD32 num_sfb,
                              const WORD32 *ptr_scale_factors, WORD32 num_win_grps,
                              WORD32 global_gain, const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2])
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 i, j, bit_count = 0;
  WORD32 diff, length, codeword;
  WORD32 index = 0;
  WORD32 sf_out = 0;
  WORD32 sf_not_out = 0;
  WORD32 previous_scale_factor = global_gain;

  for (j = 0; j < num_win_grps; j++)
  {
    for (i = 0; i < max_sfb; i++)
    {
      if (!((i == 0) && (j == 0)))
      {
        diff = ptr_scale_factors[index] - previous_scale_factor;
        length = huff_tab[diff + 60][0];
        bit_count += length;
        previous_scale_factor = ptr_scale_factors[index];
        if (write_flag == 1)
        {
          codeword = huff_tab[diff + 60][1];
          impeghe_write_bits_buf(it_bit_buf, codeword, length);
          sf_out++;
        }
      }
      index++;
    }
    for (; i < num_sfb; i++)
    {
      sf_not_out++;
      index++;
    }
  }

  return (bit_count);
}

/**
 *  impeghe_write_ms_data
 *
 *  \brief Write M/S bits
 *
 *  \param [in,out] it_bit_buf  Bit buffer
 *  \param [in] ms_mask         MS mask value
 *  \param [in] ms_used         MS used per band
 *  \param [in] num_win_grps    Number of window groups
 *  \param [in] nr_of_sfb       Number of scalefactor bands
 *
 *  \return WORD32			    Bits consumed for writing M/S data
 *
 */
WORD32 impeghe_write_ms_data(ia_bit_buf_struct *it_bit_buf, WORD32 ms_mask,
                             WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                             WORD32 num_win_grps, WORD32 nr_of_sfb)
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 bit_count = 0;
  WORD32 i, j;
  WORD32 ms_mask_write = ms_mask;

  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, ms_mask_write, 2);
  bit_count += 2;

  if (ms_mask_write == 1)
  {
    for (i = 0; i < num_win_grps; i++)
    {
      for (j = 0; j < nr_of_sfb; j++)
      {
        if (write_flag)
          impeghe_write_bits_buf(it_bit_buf, ms_used[i][j], 1);
        bit_count += 1;
      }
    }
  }

  return bit_count;
}

/**
 *  impeghe_write_tns_data
 *
 *  \brief Write TNS bits
 *
 *  \param [in,out] it_bit_buf      Bit buffer
 *  \param [in] pstr_tns_info       Pointer to TNS info structure
 *  \param [in] window_sequence     Window sequence
 *  \param [in] core_mode           Core coding mode
 *
 *  \return WORD32				 Bits consumed for writing TNS data
 *
 */
WORD32 impeghe_write_tns_data(ia_bit_buf_struct *it_bit_buf, ia_tns_info *pstr_tns_info,
                              WORD32 window_sequence, WORD32 core_mode)
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 bit_count = 0;
  WORD32 num_windows;
  WORD32 len_tns_nfilt;
  WORD32 len_tns_length;
  WORD32 len_tns_order;
  WORD32 filt;
  WORD32 res_bits;
  UWORD32 coeff;
  WORD32 w;

  if (window_sequence == EIGHT_SHORT_SEQUENCE)
  {
    num_windows = MAX_SHORT_WINDOWS;
    len_tns_nfilt = 1;
    len_tns_length = 4;
    len_tns_order = 3;
  }
  else
  {
    num_windows = 1;
    len_tns_nfilt = 2;
    len_tns_length = 6;
    len_tns_order = 4;
  }
  if (core_mode == 1)
  {
    num_windows = 1;
  }

  for (w = 0; w < num_windows; w++)
  {
    ia_tns_window_data *ptr_win_data = &pstr_tns_info->window_data[w];
    WORD32 n_filt = ptr_win_data->n_filt;
    if (write_flag)
    {
      impeghe_write_bits_buf(it_bit_buf, n_filt, len_tns_nfilt);
    }
    bit_count += len_tns_nfilt;
    if (n_filt)
    {
      res_bits = ptr_win_data->coef_res;
      if (write_flag)
      {
        impeghe_write_bits_buf(it_bit_buf, res_bits - DEF_TNS_RES_OFFSET, 1);
      }
      bit_count += 1;
      for (filt = 0; filt < n_filt; filt++)
      {
        ia_tns_filter_data *ptr_tns_filt = &ptr_win_data->tns_filter[filt];
        WORD32 order = ptr_tns_filt->order;
        if (write_flag)
        {
          impeghe_write_bits_buf(it_bit_buf, ptr_tns_filt->length, len_tns_length);
          impeghe_write_bits_buf(it_bit_buf, order, len_tns_order);
        }
        bit_count += (len_tns_length + len_tns_order);
        if (order)
        {
          WORD32 i;
          if (write_flag)
          {
            impeghe_write_bits_buf(it_bit_buf, ptr_tns_filt->direction, 1);
            impeghe_write_bits_buf(it_bit_buf, ptr_tns_filt->coef_compress, 1);
          }
          bit_count += 2;
          for (i = 1; i <= order; i++)
          {
            if (write_flag)
            {
              coeff = (UWORD32)(ptr_tns_filt->index[i]) & ((1 << res_bits) - 1);
              impeghe_write_bits_buf(it_bit_buf, coeff, res_bits);
            }
            bit_count += res_bits;
          }
        }
      }
    }
  }

  return bit_count;
}

/**
 *  impeghe_calc_grouping_bits
 *
 *  \brief Calculate grouping bits
 *
 *  \param [in] ptr_win_grp_len  Pointer to length of window groups
 *  \param [in] num_win_grps     Number of window groups
 *
 *  \return WORD32			 Grouping bits
 *
 */
static WORD32 impeghe_calc_grouping_bits(const WORD32 *ptr_win_grp_len, WORD32 num_win_grps)
{

  WORD32 grouping_bits = 0;
  WORD32 tmp[8] = {0};
  WORD32 i, j;
  WORD32 index = 0;

  for (i = 0; i < num_win_grps; i++)
  {
    for (j = 0; j < ptr_win_grp_len[i]; j++)
    {
      tmp[index++] = i;
    }
  }

  for (i = 1; i < 8; i++)
  {
    grouping_bits = grouping_bits << 1;
    if (tmp[i] == tmp[i - 1])
    {
      grouping_bits++;
    }
  }

  return (grouping_bits);
}

/**
 *  impeghe_write_ics_info
 *
 *  \brief Write ICS bits
 *
 *  \param [in,out] it_bit_buf    Bit buffer
 *  \param [in] pstr_sfb_prms     Pointer to SFB parameters
 *  \param [in] ch                Channel index
 *
 *  \return WORD32			   Bits consumed for writing ICS data
 *
 */
WORD32 impeghe_write_ics_info(ia_bit_buf_struct *it_bit_buf, ia_sfb_params_struct *pstr_sfb_prms,
                              WORD32 ch)
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 bit_count = 0;
  WORD32 win_seq = 0;
  WORD32 grouping_bits = 0;
  WORD32 max_sfb = pstr_sfb_prms->max_sfb[ch];
  WORD32 window_sequence = pstr_sfb_prms->window_sequence[ch];
  WORD32 window_shape = pstr_sfb_prms->window_shape[ch];
  WORD32 num_win_grps = pstr_sfb_prms->num_window_groups[ch];

  switch (window_sequence)
  {
  case EIGHT_SHORT_SEQUENCE:
    win_seq = 2;
    break;
  case ONLY_LONG_SEQUENCE:
    win_seq = 0;
    break;
  case LONG_START_SEQUENCE:
  case STOP_START_SEQUENCE:
    win_seq = 1;
    break;
  case LONG_STOP_SEQUENCE:
    win_seq = 3;
    break;
  default:
    win_seq = 3;
    break;
  }
  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, win_seq, 2);
  bit_count += 2;

  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, window_shape, 1);
  bit_count += 1;

  if (window_sequence == EIGHT_SHORT_SEQUENCE)
  {
    if (write_flag)
      impeghe_write_bits_buf(it_bit_buf, max_sfb, 4);
    bit_count += 4;

    grouping_bits =
        impeghe_calc_grouping_bits(pstr_sfb_prms->window_group_length[ch], num_win_grps);
    if (write_flag)
      impeghe_write_bits_buf(it_bit_buf, grouping_bits, 7);
    bit_count += 7;
  }
  else
  {
    if (write_flag)
      impeghe_write_bits_buf(it_bit_buf, max_sfb, 6);
    bit_count += 6;
  }

  return (bit_count);
}

/**
 *  impeghe_write_cplx_pred_data
 *
 *  \brief Writes complex prediction data to the bitstream
 *
 *  \param [in,out] it_bit_buf				Pointer to bit-buffer
 *  \param [in]     num_win_grps			Number of window groups
 *  \param [in]     num_sfb					Number of scalefactor bands
 *  \param [in]     complex_coef			Flag denoting real or complex stereo
 * prediction
 *  \param [in]     pred_coeffs_re			Quantized real prediction coefficients
 *  \param [in]     pred_coeffs_im			Quantized imaginary prediction
 * coefficients
 *  \param [in]     huff_tab				Huffman table for encoding
 *  \param [in]     usac_independency_flg   Flag denoting USAC independent frame
 *  \param [in]     pred_dir                Flag denoting direction of prediction
 *  \param [in]     cplx_pred_used          Flag denoting whether complex prediction or L/R coding
 * is used per window group and scale-factor band
 *  \param [in]     cplx_pred_all           Flag denoting whether complex prediction is used for
 * all frequency bands
 *  \param [out]    ptr_prev_alpha_coef_re  Pointer to previous frame/band quantized real
 * prediction coefficient
 *  \param [out]    ptr_prev_alpha_coef_im  Pointer to previous frame/band quantized imaginary
 * prediction coefficient
 *  \param [out]    delta_code_time         Flag denoting time- or frequency-differential
 * Huffmancoding of prediction coefficents
 *
 *  \return WORD32 Bits consumed for writing complex prediction data
 *
 */
WORD32 impeghe_write_cplx_pred_data(ia_bit_buf_struct *it_bit_buf, WORD32 num_win_grps,
                                    WORD32 num_sfb, WORD32 complex_coef,
                                    WORD32 pred_coeffs_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    WORD32 pred_coeffs_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2],
                                    WORD32 const usac_independency_flg, WORD32 pred_dir,
                                    WORD32 cplx_pred_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
                                    WORD32 cplx_pred_all, WORD32 *ptr_prev_alpha_coeff_re,
                                    WORD32 *ptr_prev_alpha_coeff_im, WORD32 *delta_code_time)
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 bit_count = 0;
  WORD32 i, j;
  WORD32 g;
  WORD32 sfb;
  const WORD32 sfb_per_pred_band = 2;
  WORD32 length_temp1_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      length_temp2_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      length_temp1_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      length_temp2_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 code_word_temp1_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      code_word_temp2_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      code_word_temp1_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
      code_word_temp2_im[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG];
  WORD32 length_tot1 = 0, length_tot2 = 0;

  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, cplx_pred_all, 1);
  bit_count += 1;

  if (cplx_pred_all == 0)
  {
    for (g = 0; g < num_win_grps; g++)
    {
      for (sfb = 0; sfb < num_sfb; sfb += sfb_per_pred_band)
      {
        impeghe_write_bits_buf(it_bit_buf, cplx_pred_used[g][sfb], 1);
        bit_count += 1;
      }
    }
  }

  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, pred_dir, 1);
  bit_count += 1;

  if (write_flag)
    impeghe_write_bits_buf(it_bit_buf, complex_coef, 1);
  bit_count += 1;

  if (complex_coef)
  {
    if (!usac_independency_flg)
    {
      if (write_flag)
        impeghe_write_bits_buf(it_bit_buf, 1, 1); /* use_prev_frame */
      bit_count += 1;
    }
  }

  if (usac_independency_flg)
  {
    *delta_code_time = 0;
  }

  /* Switching mechanism for delta_code_time */
  WORD32 prev_pred_coeff_re_temp1 = 0, prev_pred_coeff_re_temp2 = 0;
  WORD32 diff_pred_coeff_re_temp1 = 0, diff_pred_coeff_re_temp2 = 0;
  WORD32 prev_pred_coeff_im_temp1 = 0, prev_pred_coeff_im_temp2 = 0;
  WORD32 diff_pred_coeff_im_temp1 = 0, diff_pred_coeff_im_temp2 = 0;

  for (i = 0; i < num_win_grps; i++)
  {
    /* delta_code_time = 0*/
    prev_pred_coeff_re_temp1 = 0;
    if (complex_coef == 1)
    {
      prev_pred_coeff_im_temp1 = 0;
    }

    for (j = 0; j < num_sfb; j += 2)
    {
      if (!usac_independency_flg)
      {
        /* delta_code_time = 1*/
        if (i > 0)
        {
          prev_pred_coeff_re_temp2 = pred_coeffs_re[i - 1][j];
          if (complex_coef == 1)
          {
            prev_pred_coeff_im_temp2 = pred_coeffs_im[i - 1][j];
          }
        }
        else
        {
          prev_pred_coeff_re_temp2 = ptr_prev_alpha_coeff_re[j];
          if (complex_coef == 1)
          {
            prev_pred_coeff_im_temp2 = ptr_prev_alpha_coeff_im[j];
          }
        }
      }

      if (cplx_pred_used[i][j] == 1)
      {
        /*Differential Huffman coding of real prediction coefficients*/
        diff_pred_coeff_re_temp1 =
            pred_coeffs_re[i][j] - prev_pred_coeff_re_temp1; /* delta_code_time = 0 */
        prev_pred_coeff_re_temp1 = pred_coeffs_re[i][j];     /* delta_code_time = 0 */
        if (!usac_independency_flg)
        {
          diff_pred_coeff_re_temp2 =
              pred_coeffs_re[i][j] - prev_pred_coeff_re_temp2; /* delta_code_time = 1 */
        }

        /* delta_code_time = 0 */
        length_temp1_re[i][j] = huff_tab[diff_pred_coeff_re_temp1 + 60][0];
        code_word_temp1_re[i][j] = huff_tab[diff_pred_coeff_re_temp1 + 60][1];

        length_tot1 += length_temp1_re[i][j];

        if (!usac_independency_flg)
        {
          /*delta_code_time = 1 */
          length_temp2_re[i][j] = huff_tab[diff_pred_coeff_re_temp2 + 60][0];
          code_word_temp2_re[i][j] = huff_tab[diff_pred_coeff_re_temp2 + 60][1];

          length_tot2 += length_temp2_re[i][j];
        }

        if (complex_coef == 1)
        {
          /*Differential Huffman coding of imaginary prediction coefficients*/
          diff_pred_coeff_im_temp1 =
              pred_coeffs_im[i][j] - prev_pred_coeff_im_temp1; /* delta_code_time = 0 */
          prev_pred_coeff_im_temp1 = pred_coeffs_im[i][j];     /* delta_code_time = 0*/

          if (!usac_independency_flg)
          {
            diff_pred_coeff_im_temp2 =
                pred_coeffs_im[i][j] - prev_pred_coeff_im_temp2; /* delta_code_time = 1 */
          }

          /*delta_code_time = 0*/
          length_temp1_im[i][j] = huff_tab[diff_pred_coeff_im_temp1 + 60][0];
          code_word_temp1_im[i][j] = huff_tab[diff_pred_coeff_im_temp1 + 60][1];

          length_tot1 += length_temp1_im[i][j];

          if (!usac_independency_flg)
          {
            /*delta_code_time = 1*/
            length_temp2_im[i][j] = huff_tab[diff_pred_coeff_im_temp2 + 60][0];
            code_word_temp2_im[i][j] = huff_tab[diff_pred_coeff_im_temp2 + 60][1];

            length_tot2 += length_temp2_im[i][j];
          }
        }
      }
      else
      {
        pred_coeffs_re[i][j] = 0;
        /*delta_code_time = 0*/
        prev_pred_coeff_re_temp1 = pred_coeffs_re[i][j];
        if (complex_coef == 1)
        {
          pred_coeffs_im[i][j] = 0;
          /*delta_code_time = 0*/
          prev_pred_coeff_im_temp1 = pred_coeffs_im[i][j];
        }
      }

      ptr_prev_alpha_coeff_re[j] = pred_coeffs_re[i][j];
      if (complex_coef == 1)
      {
        ptr_prev_alpha_coeff_im[j] = pred_coeffs_im[i][j];
      }
    }

    for (j = num_sfb; j < MAX_NUM_SFB_LONG; j++)
    {
      pred_coeffs_re[i][j] = 0;
      ptr_prev_alpha_coeff_re[j] = 0;
      if (complex_coef == 1)
      {
        pred_coeffs_im[i][j] = 0;
        ptr_prev_alpha_coeff_im[j] = 0;
      }
    }
  }

  /*Make a decison on the value of delta_code_time per frame */
  if (!usac_independency_flg)
  {
    // Compare the code-word lengths
    if (length_tot1 <= length_tot2)
    {
      *delta_code_time = 0;
    }
    else
    {
      *delta_code_time = 1;
    }

    /* Write the value of delta_code_time to bitstream */
    if (write_flag)
      impeghe_write_bits_buf(it_bit_buf, *delta_code_time, 1);
    bit_count += 1;
  }

  if (*delta_code_time == 0)
  {
    for (i = 0; i < num_win_grps; i++)
    {
      for (j = 0; j < num_sfb; j += 2)
      {
        if (cplx_pred_used[i][j] == 1)
        {
          if (write_flag)
            impeghe_write_bits_buf(it_bit_buf, code_word_temp1_re[i][j], length_temp1_re[i][j]);
          bit_count += length_temp1_re[i][j];

          if (complex_coef == 1)
          {
            if (write_flag)
              impeghe_write_bits_buf(it_bit_buf, code_word_temp1_im[i][j], length_temp1_im[i][j]);
            bit_count += length_temp1_im[i][j];
          }
        }
      }
    }
  }
  else
  {
    for (i = 0; i < num_win_grps; i++)
    {
      for (j = 0; j < num_sfb; j += 2)
      {
        if (cplx_pred_used[i][j] == 1)
        {
          if (write_flag)
            impeghe_write_bits_buf(it_bit_buf, code_word_temp2_re[i][j], length_temp2_re[i][j]);
          bit_count += length_temp2_re[i][j];

          if (complex_coef == 1)
          {
            if (write_flag)
              impeghe_write_bits_buf(it_bit_buf, code_word_temp2_im[i][j], length_temp2_im[i][j]);
            bit_count += length_temp2_im[i][j];
          }
        }
      }
    }
  }

  return bit_count;
}

/**
 *  impeghe_write_igf_mask_data
 *
 *  \brief Write IGF mask bits
 *
 *  \param [in,out] it_bit_buf   Bit buffer
 *  \param [in] num_groups       Number of window groups
 *  \param [in] pstr_igf_data    Pointer to IGF data structure
 *  \param [in] pstr_igf_config  Pointer to IGF config structure
 *  \param [in] ms_used          MS used per band
 *
 *  \return WORD32				 Bits consumed for writing IGF mask data
 *
 */
WORD32 impeghe_write_igf_mask_data(ia_bit_buf_struct *it_bit_buf, WORD32 num_groups,
                                   ia_igf_data_struct *pstr_igf_data,
                                   ia_igf_config_struct *pstr_igf_config,
                                   WORD32 ms_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG])
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 g, sfb;
  WORD32 igf_use_high_res = pstr_igf_config->igf_use_high_res;
  WORD32 bit_count = 0;
  WORD32 igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;
  WORD32 inc = (2 - igf_use_high_res);

  if (write_flag)
  {
    for (g = 0; g < num_groups; g++)
    {
      for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += inc)
      {
        impeghe_write_bits_buf(it_bit_buf, ms_used[g][sfb], 1);
      }
    }
  }
  bit_count += (num_groups * (WORD32)ceil((igf_stop_sfb - igf_start_sfb) / (float)inc));

  return bit_count;
}

/**
 *  impeghe_write_igf_pred_data
 *
 *  \brief Write IGF prediction bits
 *
 *  \param [in,out] it_bit_buf          Bit buffer
 *  \param [in] num_groups              Number of window groups
 *  \param [in] pstr_igf_config           Pointer to IGF config structure
 *  \param [in] usac_independency_flag  USAC independency flag
 *  \param [in] huff_tab                Huffman code table
 *  \param [in] igf_pred_dir            IGF prediction direction
 *  \param [in] pred_coeffs_re          Pointer to real prediction coefficients
 *  \param [in] igf_delta_code_time     IGF delta code time
 *  \param [in] cplx_pred_used          Flag signalling if complex prediction or L/R is used per
 * sfb
 *
 *  \return WORD32						Bits consumed for writing IGF
 * prediction
 * data
 *
 */
WORD32 impeghe_write_igf_pred_data(
    ia_bit_buf_struct *it_bit_buf, WORD32 num_groups, ia_igf_config_struct *pstr_igf_config,
    WORD32 usac_independency_flag, const WORD32 huff_tab[CODE_BOOK_ALPHA_LAV][2],
    WORD32 igf_pred_dir, WORD32 pred_coeffs_re[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG],
    WORD32 *igf_delta_code_time, WORD32 cplx_pred_used[MAX_SHORT_WINDOWS][MAX_NUM_SFB_LONG])
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 g, sfb, bit_count = 0;
  WORD32 igf_start_sfb = pstr_igf_config->m_igf_start_sfb;
  WORD32 igf_stop_sfb = pstr_igf_config->m_igf_stop_sfb;
  WORD32 igf_stereo_pred_all = pstr_igf_config->igf_stereo_pred_all;

  if (write_flag)
  {
    impeghe_write_bits_buf(it_bit_buf, igf_stereo_pred_all, 1);

    if (igf_stereo_pred_all == 0)
    {
      for (g = 0; g < num_groups; g++)
      {
        for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += 2)
        {
          impeghe_write_bits_buf(it_bit_buf, cplx_pred_used[g][sfb], 1);
        }
      }
    }
    impeghe_write_bits_buf(it_bit_buf, igf_pred_dir, 1);
  }
  bit_count += 2;
  if (igf_stereo_pred_all == 0)
  {
    bit_count += (num_groups * (WORD32)ceil((igf_stop_sfb - igf_start_sfb) / (float)2));
  }
  if (usac_independency_flag == 0)
  {
    impeghe_write_bits_buf(it_bit_buf, *igf_delta_code_time, 1);
    bit_count += 1;
  }

  for (g = 0; g < num_groups; g++)
  {
    WORD32 prev_pred_coeff_re = 0;
    WORD32 diff_pred_coeff_re;

    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += 2)
    {
      if (cplx_pred_used[g][sfb] == 1)
      {
        WORD32 length;
        WORD32 code_word;

        /*Differential Huffman coding of real prediction coefficients*/
        diff_pred_coeff_re = pred_coeffs_re[g][sfb] - prev_pred_coeff_re;
        prev_pred_coeff_re = pred_coeffs_re[g][sfb];

        length = huff_tab[diff_pred_coeff_re + 60][0];
        code_word = huff_tab[diff_pred_coeff_re + 60][1];

        if (write_flag)
          impeghe_write_bits_buf(it_bit_buf, code_word, length);
        bit_count += length;
      }
    }
  }
  return bit_count;
}

/**
 *  impeghe_write_cpe
 *
 *  \brief Write CPE bits
 *
 *  \param [in] pstr_sfb_prms           Pointer to sfb parameters
 *  \param [in,out] it_bit_buf          Bit buffer
 *  \param [in] tns_data_present        TNS present flag
 *  \param [in] usac_independency_flg   USAC independency flag
 *  \param [in] pstr_usac_config         Pointer to encoder config structure
 *  \param [in] pstr_usac_data           Pointer to encoder data structure
 *  \param [in] ch                      Channel index
 *  \param [in] ele_id                  Element index
 *
 *  \return WORD32						Bits consumed for writing CPE
 *
 */
WORD32 impeghe_write_cpe(ia_sfb_params_struct *pstr_sfb_prms, ia_bit_buf_struct *it_bit_buf,
                         WORD32 *tns_data_present, WORD32 const usac_independency_flg,
                         ia_usac_encoder_config_struct *pstr_usac_config,
                         ia_usac_data_struct *pstr_usac_data, WORD32 ch, WORD32 ele_id)
{
  WORD32 bit_count = 0;
  WORD32 enhanced_noise_filling = pstr_usac_data->str_igf_config[ele_id].igf_active;
  WORD32 igf_after_tns_synth = pstr_usac_config->igf_after_tns_synth;
  WORD32 igf_independent_tiling = pstr_usac_data->str_igf_config[ele_id].igf_independent_tiling;
  WORD32 ms_mask = pstr_usac_data->str_ms_info[ch].ms_mask;
  WORD32 common_max_sfb = 1;
  WORD32 tns_active = tns_data_present[0] || tns_data_present[1];

  impeghe_write_bits_buf(it_bit_buf, tns_active, 1);
  bit_count += 1;

  impeghe_write_bits_buf(it_bit_buf, pstr_sfb_prms->common_win[ch], 1);
  bit_count += 1;

  if (pstr_sfb_prms->max_sfb[ch] != pstr_sfb_prms->max_sfb[ch + 1])
  {
    common_max_sfb = 0;
  }

  if (pstr_sfb_prms->common_win[ch])
  {
    bit_count += impeghe_write_ics_info(it_bit_buf, pstr_sfb_prms, ch);

    impeghe_write_bits_buf(it_bit_buf, common_max_sfb, 1);
    bit_count += 1;

    if (common_max_sfb == 0)
    {
      if (pstr_sfb_prms->window_sequence[ch] != EIGHT_SHORT_SEQUENCE)
      {
        impeghe_write_bits_buf(it_bit_buf, pstr_sfb_prms->max_sfb[ch + 1], 6);
        bit_count += 6;
      }
      else
      {
        impeghe_write_bits_buf(it_bit_buf, pstr_sfb_prms->max_sfb[ch + 1], 4);
        bit_count += 4;
      }
    }

    pstr_sfb_prms->max_sfb_ste = max(pstr_sfb_prms->max_sfb[ch], pstr_sfb_prms->max_sfb[ch + 1]);

    if (enhanced_noise_filling && (igf_independent_tiling != 1))
    {
      if (pstr_sfb_prms->window_sequence[ch] != EIGHT_SHORT_SEQUENCE)
      {
        pstr_sfb_prms->max_sfb_ste = min(pstr_sfb_prms->max_sfb_ste,
                                         pstr_usac_data->str_igf_config[ele_id].igf_start_sfb_lb);
      }
      else
      {
        pstr_sfb_prms->max_sfb_ste = min(pstr_sfb_prms->max_sfb_ste,
                                         pstr_usac_data->str_igf_config[ele_id].igf_start_sfb_sb);
      }
    }

    bit_count +=
        impeghe_write_ms_data(it_bit_buf, ms_mask, pstr_usac_data->str_ms_info[ch].ms_used,
                              pstr_sfb_prms->num_window_groups[ch], pstr_sfb_prms->max_sfb_ste);

    {

      if (ms_mask == 3)
      {

        bit_count += impeghe_write_cplx_pred_data(
            it_bit_buf, pstr_sfb_prms->num_window_groups[ch], pstr_sfb_prms->max_sfb_ste,
            pstr_usac_data->complex_coef[ch], pstr_usac_data->pred_coef_re[ch],
            pstr_usac_data->pred_coef_im[ch], impeghe_huffman_code_table, usac_independency_flg,
            pstr_usac_data->pred_dir_idx[ch], pstr_usac_data->cplx_pred_used[ch],
            pstr_usac_data->cplx_pred_all[ch], pstr_usac_data->pred_coef_re_prev[ch],
            pstr_usac_data->pred_coef_im_prev[ch], &pstr_usac_data->delta_code_time[ch]);
      }
    }
    if (enhanced_noise_filling && (igf_independent_tiling != 1))
    {
      WORD32 igf_ms_mask_present = pstr_usac_data->str_igf_config[ele_id].igf_ms_mask;
      impeghe_write_bits_buf(it_bit_buf, igf_ms_mask_present, 2);
      bit_count += 2;

      if (igf_ms_mask_present == 1)
      {
        bit_count += impeghe_write_igf_mask_data(
            it_bit_buf, pstr_sfb_prms->num_window_groups[ch], &pstr_usac_data->str_igf_data[ch],
            &pstr_usac_data->str_igf_config[ele_id], pstr_usac_data->str_ms_info[ch].ms_used);
      }
      if (igf_ms_mask_present == 3)
      {
        bit_count += impeghe_write_igf_pred_data(
            it_bit_buf, pstr_sfb_prms->num_window_groups[ch],
            &pstr_usac_data->str_igf_config[ele_id], usac_independency_flg,
            impeghe_huffman_code_table, pstr_usac_data->pred_dir_idx[ch],
            pstr_usac_data->pred_coef_re[ch], &pstr_usac_data->delta_code_time[ch],
            pstr_usac_data->cplx_pred_used[ch]);
      }
    }
  }

  impeghe_write_bits_buf(it_bit_buf, 0, 1); // common ltpf
  bit_count += 1;

  if (tns_active)
  {
    WORD32 common_tns = 0;
    WORD32 tns_on_lr = 1;
    WORD32 tns_present_both = tns_data_present[0] && tns_data_present[1];
    WORD32 tns_data_present1 = tns_data_present[1];

    if (pstr_sfb_prms->common_win[ch])
    {
      impeghe_write_bits_buf(it_bit_buf, common_tns, 1);
      bit_count += 1;
    }

    if (enhanced_noise_filling == 0 || igf_after_tns_synth == 1)
    {
      impeghe_write_bits_buf(it_bit_buf, tns_on_lr, 1);
      bit_count += 1;
    }

    impeghe_write_bits_buf(it_bit_buf, tns_present_both, 1);
    bit_count += 1;

    if (!tns_present_both)
    {
      impeghe_write_bits_buf(it_bit_buf, tns_data_present1, 1);
      bit_count += 1;
    }
  }

  return (bit_count);
}

/**
 *  impeghe_write_fd_data
 *
 *  \brief Write FD mode bits
 *
 *  \param [in,out] it_bit_buf          Bit buffer
 *  \param [in] pstr_sfb_prms			Pointer to SFB parameters
 *  \param [in] num_fac_bits			Number of fac bits
 *  \param [in] usac_independency_flg	USAC independency flag
 *  \param [in] pstr_usac_data			Pointer to encoder data structure
 *  \param [in] pstr_usac_config			Pointer to encoder config structure
 *  \param [in] ch_idx					Channel index
 *  \param [in] ele_id					Element index
 *  \param [in] idx	     				Channel index 0 or 1
 *
 *  \return WORD32						Bits consumed for writing FD mode
 * data
 *
 */
WORD32 impeghe_write_fd_data(ia_bit_buf_struct *it_bit_buf, ia_sfb_params_struct *pstr_sfb_prms,
                             WORD32 num_fac_bits, WORD32 usac_independency_flg,
                             ia_usac_data_struct *pstr_usac_data,
                             ia_usac_encoder_config_struct *pstr_usac_config, WORD32 ch_idx,
                             WORD32 ele_id, WORD32 idx)
{
  WORD32 bit_count = 0;
  WORD32 fac_data_present = (num_fac_bits > 0) ? 1 : 0;
  WORD32 common_ltpf = 0;
  WORD32 enhanced_noise_filling = pstr_usac_data->str_igf_config[ele_id].igf_active;
  ia_igf_config_struct *pstr_igf_config = &pstr_usac_data->str_igf_config[ele_id];
  ia_igf_data_struct *pstr_igf_data = &pstr_usac_data->str_igf_data[ch_idx];
  WORD16 *ptr_fac_data = pstr_usac_data->fac_out_stream[ch_idx];

  WORD32 is_noise_filling = pstr_usac_data->noise_filling[ele_id];
  WORD32 common_window = pstr_sfb_prms->common_win[ch_idx];
  ia_usac_quant_info_struct *pstr_quant_info = &(pstr_usac_data->str_quant_info[idx]);
  ia_tns_info *pstr_tns_info = pstr_usac_data->pstr_tns_info[ch_idx];
  WORD32 global_gain = pstr_usac_data->str_quant_info[idx].scale_factor[0];

  if (pstr_usac_data->channel_elem_type[ele_id] == ID_USAC_LFE)
  {
    is_noise_filling = 0;
    enhanced_noise_filling = 0;
    pstr_usac_data->ltpf_data[ch_idx].ltpf_active = 0;
    if (pstr_tns_info != NULL)
    {
      pstr_tns_info->tns_data_present = 0;
    }
  }

  impeghe_write_bits_buf(it_bit_buf, global_gain, 8);
  bit_count += 8;

  if (is_noise_filling)
  {

    impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->noise_level[idx], 3);

    impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->noise_offset[idx], 5);
    bit_count += 8;
  }

  if (!common_window)
  {
    bit_count += impeghe_write_ics_info(it_bit_buf, pstr_sfb_prms, ch_idx);
  }
  if (!common_ltpf)
  {
    if (pstr_usac_config->ltpf_enable && pstr_usac_data->ltpf_data[ch_idx].ltpf_active)
    {
      impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->ltpf_data[ch_idx].ltpf_active, 1);
      impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->ltpf_data[ch_idx].ltpf_pitch_index, 9);
      impeghe_write_bits_buf(it_bit_buf, 0, 2); // 2 more bits for gain idx
      bit_count += 12;
    }
    else
    {
      impeghe_write_bits_buf(it_bit_buf, 0, 1);
      bit_count += 1;
    }
  }

  if (usac_independency_flg == 0 &&
      pstr_sfb_prms->window_sequence[ch_idx] != EIGHT_SHORT_SEQUENCE)
  {
    // fdp_data_present
    impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->fdp_data.fdp_active[ch_idx], 1);
    bit_count += 1;

    if (pstr_usac_data->fdp_data.fdp_active[ch_idx])
    {
      impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->fdp_data.fdp_spacing_idx[ch_idx], 8);
      bit_count += 8;
    }
  }

  if (usac_independency_flg == 1)
  {
    // prev_aliasing_symmetry
    impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->prev_aliasing_symmetry[ch_idx], 1);
    bit_count += 1;
  }
  else
  {
    pstr_usac_data->prev_aliasing_symmetry[ch_idx] =
        pstr_usac_data->curr_aliasing_symmetry[ch_idx];
  }
  // curr_aliasing_symmetry
  impeghe_write_bits_buf(it_bit_buf, pstr_usac_data->curr_aliasing_symmetry[ch_idx], 1);
  bit_count += 1;

  bit_count += impeghe_write_scf_data(
      it_bit_buf, pstr_sfb_prms->max_sfb[ch_idx], pstr_sfb_prms->num_sfb[ch_idx],
      pstr_quant_info->scale_factor, pstr_sfb_prms->num_window_groups[ch_idx], global_gain,
      impeghe_huffman_code_table);

  if (enhanced_noise_filling == 1)
  {
    bit_count += impeghe_write_igf_levels(it_bit_buf, pstr_igf_data, pstr_igf_config,
                                          usac_independency_flg, CORE_MODE_FD,
                                          pstr_sfb_prms->num_window_groups[ch_idx]);

    if (pstr_igf_data->igf_all_zero == 0)
    {
      bit_count += impeghe_write_igf_data(it_bit_buf, pstr_igf_data, pstr_igf_config,
                                          usac_independency_flg, 0);
    }
  }

  if (pstr_tns_info != NULL && pstr_tns_info->tns_data_present == 1)
  {
    bit_count += impeghe_write_tns_data(it_bit_buf, pstr_tns_info,
                                        pstr_sfb_prms->window_sequence[ch_idx], 0);
  }

  if (!usac_independency_flg)
  {
    impeghe_write_bits_buf(it_bit_buf, pstr_quant_info->reset, 1);
    bit_count += 1;
  }

  bit_count += impeghe_arith_enc_spec(
      it_bit_buf, pstr_sfb_prms->window_sequence[ch_idx], pstr_quant_info->quant_degroup,
      pstr_quant_info->max_spec_coeffs, pstr_quant_info->c_pres, pstr_quant_info->c_prev,
      &(pstr_quant_info->arith_size_prev), usac_independency_flg || pstr_quant_info->reset,
      &(pstr_usac_data->str_scratch));

  impeghe_write_bits_buf(it_bit_buf, fac_data_present, 1);
  bit_count += 1;

  if (fac_data_present)
  {
    WORD32 i;
    for (i = 0; i < num_fac_bits; i += 8)
    {
      WORD32 bits_to_write = min(8, num_fac_bits - i);
      impeghe_write_bits_buf(it_bit_buf, ptr_fac_data[i / 8] >> (8 - bits_to_write),
                             bits_to_write);
    }
    bit_count += num_fac_bits;
  }

  return (bit_count);
}

/**
 *  impeghe_count_fd_bits
 *
 *  \brief Count FD mode bits
 *
 *  \param [in] pstr_sfb_prms			Pointer to SFB parameters
 *  \param [in] pstr_usac_data			Pointer to encoder data structure
 *  \param [in] usac_independency_flg	USAC independency flag
 *  \param [in] pstr_usac_config			Pointer to encoder config structure
 *  \param [in] ch_idx					Channel index
 *  \param [in] idx					    Channel index 0 or 1.
 *
 *  \return WORD32					Bit count for FD mode
 *
 */
WORD32 impeghe_count_fd_bits(ia_sfb_params_struct *pstr_sfb_prms,
                             ia_usac_data_struct *pstr_usac_data, WORD32 usac_independency_flg,
                             ia_usac_encoder_config_struct *pstr_usac_config, WORD32 ch_idx,
                             WORD32 idx)
{
  WORD32 bit_count = 0;
  ia_usac_quant_info_struct *pstr_quant_info = &pstr_usac_data->str_quant_info[idx];
  WORD32 window_sequence = pstr_sfb_prms->window_sequence[ch_idx];
  WORD32 global_gain = pstr_quant_info->scale_factor[0];
  WORD32 max_sfb = pstr_sfb_prms->max_sfb[ch_idx];
  WORD32 num_sfb = pstr_sfb_prms->num_sfb[ch_idx];
  WORD32 num_win_grps = pstr_sfb_prms->num_window_groups[ch_idx];

  bit_count += impeghe_write_scf_data(NULL, max_sfb, num_sfb, pstr_quant_info->scale_factor,
                                      num_win_grps, global_gain, impeghe_huffman_code_table);

  WORD32 temp_c_pres[516], temp_c_prev[516], temp_size = pstr_quant_info->arith_size_prev;
  memcpy(temp_c_pres, pstr_quant_info->c_pres, 516 * sizeof(pstr_quant_info->c_pres[0]));
  memcpy(temp_c_prev, pstr_quant_info->c_prev, 516 * sizeof(pstr_quant_info->c_prev[0]));
  bit_count += impeghe_arith_enc_spec(
      NULL, window_sequence, pstr_quant_info->quant_degroup, pstr_quant_info->max_spec_coeffs,
      temp_c_pres, temp_c_prev, &(temp_size), usac_independency_flg || pstr_quant_info->reset,
      &(pstr_usac_data->str_scratch));

  return (bit_count);
}

/**
 *  impeghe_write_fill_ele
 *
 *  \brief Write fill bits
 *
 *  \param [in,out] it_bit_buf		Bit buffer
 *  \param [in] num_bits			Number of fill bits
 *
 *  \return WORD32					Number of written bits
 *
 */
WORD32 impeghe_write_fill_ele(ia_bit_buf_struct *it_bit_buf, WORD32 num_bits)
{
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 bit_count = 0;

  if (num_bits <= 8)
  {
    if (write_flag)
    {
      impeghe_write_bits_buf(it_bit_buf, 0, 1);
    }
    bit_count++;
    num_bits--;
  }
  else
  {
    if (write_flag)
    {
      impeghe_write_bits_buf(it_bit_buf, 1, 1);
    }
    bit_count++;
    num_bits--;

    if (num_bits <= 8)
    {
      if (write_flag)
      {
        impeghe_write_bits_buf(it_bit_buf, 1, 1);
      }
      bit_count++;
      num_bits--;
    }
    else
    {
      WORD32 bytes_to_write = 0;
      if (write_flag)
      {
        impeghe_write_bits_buf(it_bit_buf, 0, 1);
      }
      bit_count++;
      num_bits--;
      bytes_to_write = num_bits >> 3;

      if (bytes_to_write > 255)
      {
        bytes_to_write -= 3;
        if (write_flag)
        {
          impeghe_write_bits_buf(it_bit_buf, 255, 8);
        }
        if (write_flag)
        {
          impeghe_write_bits_buf(it_bit_buf, bytes_to_write - 253, 16);
        }
        bit_count += 24;
        num_bits -= 24;
      }
      else
      {
        bytes_to_write--;
        if (write_flag)
        {
          impeghe_write_bits_buf(it_bit_buf, bytes_to_write, 8);
        }
        bit_count += 8;
        num_bits -= 8;
      }

      while (bytes_to_write > 0)
      {
        if (write_flag)
        {
          impeghe_write_bits_buf(it_bit_buf, 0xA9, 8);
        }
        bit_count += 8;
        num_bits -= 8;
        bytes_to_write--;
      }
    }
  }
  return bit_count;
}

/**
 *  impeghe_write_mpegh3da_ext_ele
 *
 *  \brief Write extension element bits
 *
 *  \param [in,out] it_bit_buf		Bit buffer
 *  \param [in] ext_data_present		Ext. data present flag
 *  \param [in] ext_data_size		Ext. data size
 *  \param [in] ext_data				Ext. data payload
 *  \param [in] num_ext_elements		Number of ext elements
 *
 *  \return WORD32					Number of written bits
 *
 */
WORD32 impeghe_write_mpegh3da_ext_ele(ia_bit_buf_struct *it_bit_buf, WORD32 ext_data_present,
                                      WORD32 ext_data_size,
                                      UWORD8 ext_data[MAX_EXTENSION_PAYLOAD_LEN])

{
  WORD32 bit_count = 0;
  WORD32 j = 0;

  WORD32 ext_ele_def_len = 0;
  WORD32 ext_ele_present = ext_data_present;
  WORD32 ext_ele_payload_len = ext_data_size;

  if (1 == ext_ele_present && 0 == ext_ele_payload_len)
  {
    return -1;
  }

  impeghe_write_bits_buf(it_bit_buf, ext_ele_present, 1);
  bit_count++;

  if (1 == ext_ele_present)
  {
    impeghe_write_bits_buf(it_bit_buf, ext_ele_def_len, 1);
    bit_count++;

    if (ext_ele_payload_len >= 255)
    {
      WORD32 value_add = ext_ele_payload_len - 255 + 2;
      impeghe_write_bits_buf(it_bit_buf, 255, 8);
      impeghe_write_bits_buf(it_bit_buf, value_add, 16);
      bit_count += 24;
    }
    else
    {
      impeghe_write_bits_buf(it_bit_buf, ext_ele_payload_len, 8);
      bit_count += 8;
    }

    for (j = 0; j < ext_ele_payload_len; ++j)
    {
      impeghe_write_bits_buf(it_bit_buf, ext_data[j], 8);
      bit_count += 8;
    }
  }

  return bit_count;
}
