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

#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_enc_mct.h"
#include "impeghe_enc_mct_rom.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_cnst.h"
#include "impeghe_rom.h"
#include "impeghe_error_codes.h"
#include "impeghe_error_standards.h"

/**
 *  channel_pair_to_idx
 *
 *  \brief Calculate channel pair index
 *
 *  \param [in] ch_idx1         Channel index 1
 *  \param [in] ch_idx2         Channel index 2
 *  \param [in] num_channels    Number of channels
 *
 *  \return WORD32              Channel pair index value
 *
 */
static WORD32 channel_pair_to_idx(WORD32 ch_idx1, WORD32 ch_idx2, WORD32 num_channels)
{
  LOOPIDX ch1, ch2;
  WORD32 pair_idx = 0;

  for (ch2 = 1; ch2 < num_channels; ch2++)
  {
    for (ch1 = 0; ch1 < ch2; ch1++)
    {
      if ((ch1 == ch_idx1) && (ch2 == ch_idx2))
        return pair_idx;
      else
        pair_idx++;
    }
  }
  return -1;
}

/**
 *  impeghe_encode_mct
 *
 *  \brief Multi Channel Tool encoding
 *
 *  \param [in,out] ptr_spec    Pointer to spectral line vector
 *  \param [in]     ptr_win_seq         Window sequence array
 *  \param [in]     ptr_sfb_offset              SFB offset array
 *  \param [in]     sfb_end                 SFB end index
 *  \param [in]     ch_offset                 Channel offset
 *  \param [in,out] pstr_mct_data                Pointer to MCT data structure
 *
 *  \return VOID
 *
 */
VOID impeghe_encode_mct(FLOAT64 *ptr_spec[MAX_TIME_CHANNELS], WORD32 *ptr_win_seq,
                        const WORD32 *ptr_sfb_offset, WORD32 sfb_end, mct_data_t *pstr_mct_data,
                        WORD32 ch_offset)
{
  WORD32 ch1 = 0;
  WORD32 ch2 = 1;
  LOOPIDX pair, i, sfb;
  WORD32 ch = ch_offset;

  WORD32 max_pairs = (pstr_mct_data->num_ch_to_apply * (pstr_mct_data->num_ch_to_apply - 1)) / 2;

  max_pairs = min(MAX_NUM_MCT_PAIRS, max_pairs);

  pstr_mct_data->num_pairs = 0;

  for (pair = 0; pair < max_pairs; pair++)
  {
    FLOAT64 max_corr = 0.0;
    WORD32 paired = 0;
    WORD32 tmp_ch1, tmp_ch2;
    ch1 = 0;
    ch2 = 0;

    pstr_mct_data->channel_pair[pair][0] = 0;
    pstr_mct_data->channel_pair[pair][1] = 0;

    /* find highest correlation */
    for (tmp_ch1 = ch; tmp_ch1 < ch + pstr_mct_data->num_channels - 1; tmp_ch1++)
    {
      for (tmp_ch2 = tmp_ch1 + 1; tmp_ch2 < ch + pstr_mct_data->num_channels; tmp_ch2++)
      {
        FLOAT64 corr = 0.0, auto_corr = 0;
        if ((pstr_mct_data->mct_ch_mask[tmp_ch1] != 0 &&
             pstr_mct_data->mct_ch_mask[tmp_ch2] != 0) &&
            ptr_win_seq[tmp_ch1] == ptr_win_seq[tmp_ch2])
        {
          WORD32 pair_coding_done = 0;
          for (i = 0; i < pair; i++)
          {
            if ((pstr_mct_data->channel_pair[i][0] == tmp_ch1) ||
                (pstr_mct_data->channel_pair[i][1] == tmp_ch2))
            {
              pair_coding_done = 1;
            }
          }
          if (pair_coding_done)
            continue;
          if (ptr_win_seq[tmp_ch1] == EIGHT_SHORT_SEQUENCE)
            continue;
          if (ptr_win_seq[tmp_ch2] == EIGHT_SHORT_SEQUENCE)
            continue;
          for (i = 0; i < ptr_sfb_offset[sfb_end]; i++)
          {
            auto_corr += ptr_spec[tmp_ch2][i] * ptr_spec[tmp_ch2][i];
            corr += ptr_spec[tmp_ch1][i] * ptr_spec[tmp_ch1][i];
          }
          auto_corr = fabs(1.0 - auto_corr / corr);

          if (auto_corr < max_corr || max_corr == 0.0)
          {
            if (auto_corr < 0.20)
            {
              max_corr = fabs(auto_corr);
              ch1 = tmp_ch1;
              ch2 = tmp_ch2;
            }
          }
          if (auto_corr == 0)
          {
            paired = 1;
            break;
          }
        }
      }
      if (paired == 1)
      {
        break;
      }
    }

    if (ch1 == 0 && ch2 == 0)
    {
      break;
    }

    if (ch1 != 0 || ch2 != 0)
    {
      pstr_mct_data->channel_pair[pair][0] = ch1;
      pstr_mct_data->channel_pair[pair][1] = ch2;
      pstr_mct_data->num_pairs++;

      if (pstr_mct_data->num_pairs >= MAX_NUM_MCT_PAIRS)
      {
        break;
      }
    }
  }
  for (pair = 0; pair < pstr_mct_data->num_pairs; pair++)
  {
    FLOAT64 mid = 0, side = 0;
    ch1 = pstr_mct_data->channel_pair[pair][0];
    ch2 = pstr_mct_data->channel_pair[pair][1];
    for (i = 0; i < 1024; i++)
    {
      mid += ptr_spec[ch1][i] * ptr_spec[ch1][i];
      side += ptr_spec[ch2][i] * ptr_spec[ch2][i];
    }

    if (mid >= side)
    {
      pstr_mct_data->pred_dir[pair] = 0;
    }
    else
    {
      pstr_mct_data->pred_dir[pair] = 1;
    }

    pstr_mct_data->num_mask_bands[pair] = 0;
    for (sfb = 0; sfb < sfb_end; sfb += 2)
    {
      WORD32 alpha_quant = 0;
      FLOAT64 alpha = 0.0;

      /* prediction */
      if ((pstr_mct_data->mct_signalling_type == 0) || (pstr_mct_data->mct_signalling_type == 2))
      {
        FLOAT64 emid = 0.0, cov = 0.0, eside = 0.0;
        for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 2]; i++)
        {
          FLOAT64 mid = 0.5 * ptr_spec[ch1][i] + 0.5 * ptr_spec[ch2][i];
          FLOAT64 side = 0.5 * ptr_spec[ch1][i] - 0.5 * ptr_spec[ch2][i];
          emid += mid * mid;
          eside += side * side;
          cov += mid * side;
        }

        if (pstr_mct_data->pred_dir[pair] == 0)
        {
          alpha = cov / (emid + 1.0e-6f);
        }
        else
        {
          alpha = cov / (eside + 1.0e-6f);
        }

        alpha_quant = (alpha < 0.0 ? -1 : 1) * ((WORD32)(fabs(alpha) * 10.0 + 0.5));

        if (fabs(alpha) > 3.0)
        {
          pstr_mct_data->pair_alpha[pair][pstr_mct_data->num_mask_bands[pair]] = 0;
          pstr_mct_data->pair_mct_mask[pair][pstr_mct_data->num_mask_bands[pair]++] = 0;
        }
        else
        {
          alpha_quant = (alpha < 0.0 ? -1 : 1) * min((WORD32)(fabs(alpha) * 10.0 + 0.5), 30);
          FLOAT64 mid, side, res;

          for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 2]; i++)
          {
            mid = 0.5 * ptr_spec[ch1][i] + 0.5 * ptr_spec[ch2][i];
            side = 0.5 * ptr_spec[ch1][i] - 0.5 * ptr_spec[ch2][i];
            if (pstr_mct_data->pred_dir[pair] == 0)
            {
              res = side - (FLOAT64)alpha_quant * 0.1 * mid;
              ptr_spec[ch1][i] = mid;
            }
            else
            {
              res = mid - (FLOAT64)alpha_quant * 0.1 * side;
              ptr_spec[ch1][i] = side;
            }
            ptr_spec[ch2][i] = res;
          }
          pstr_mct_data->pair_alpha[pair][pstr_mct_data->num_mask_bands[pair]] = alpha_quant;
          pstr_mct_data->pair_mct_mask[pair][pstr_mct_data->num_mask_bands[pair]++] = 1;
        }
      }
      /* rotation */
      else if ((pstr_mct_data->mct_signalling_type == 1) ||
               (pstr_mct_data->mct_signalling_type == 3))
      {
        FLOAT64 energy1 = 0.0, energy2 = 0.0;

        for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 2]; i++)
        {
          energy1 += ptr_spec[ch1][i] * ptr_spec[ch1][i];
          energy2 += ptr_spec[ch2][i] * ptr_spec[ch2][i];
        }

        alpha = atan2(sqrt(energy2), sqrt(energy1));
        alpha_quant = 32 + (WORD32)((alpha < 0 ? -1 : 1) *
                                    (WORD32)(fabs((alpha / (1.5707963f) * 32) + 0.5f)));

        for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 2]; i++)
        {
          FLOAT64 dmx = ptr_spec[ch1][i] * impeghe_mc_index_to_cos_alpha[alpha_quant] +
                        ptr_spec[ch2][i] * impeghe_mc_index_to_sin_alpha[alpha_quant];
          FLOAT64 res = -ptr_spec[ch1][i] * impeghe_mc_index_to_sin_alpha[alpha_quant] +
                        ptr_spec[ch2][i] * impeghe_mc_index_to_cos_alpha[alpha_quant];
          ptr_spec[ch1][i] = dmx;
          ptr_spec[ch2][i] = res;
        }
        pstr_mct_data->pair_alpha[pair][pstr_mct_data->num_mask_bands[pair]] = alpha_quant;
        pstr_mct_data->pair_mct_mask[pair][pstr_mct_data->num_mask_bands[pair]++] = 1;
      }
    }

    for (i = 0; i < pstr_mct_data->num_mask_bands[pair]; i++)
    {
      if (pstr_mct_data->pair_mct_mask[pair][i] == 0)
      {
        pstr_mct_data->is_mct_mask_present[pair] = 1;
        break;
      }
    }
  }

  // Assuming 4th channel as LFE
  for (pair = 0; pair < pstr_mct_data->num_pairs; pair++)
  {
    pstr_mct_data->channel_pair[pair][0] = (pstr_mct_data->channel_pair[pair][0] > 3)
                                               ? pstr_mct_data->channel_pair[pair][0] - 1
                                               : pstr_mct_data->channel_pair[pair][0];
    pstr_mct_data->channel_pair[pair][1] = (pstr_mct_data->channel_pair[pair][1] > 3)
                                               ? pstr_mct_data->channel_pair[pair][1] - 1
                                               : pstr_mct_data->channel_pair[pair][1];
  }
  return;
}

/**
 *  impeghe_write_mct_data
 *
 *  \brief Write Multi-channel coded bit stream
 *
 *  \param [in]     pstr_mct_data                Pointer to MCT data structure
 *  \param [in]     indep_flag              Independecy flag
 *  \param [out]    ptr_payload_extn            Payload
 *  \param [out]    size_payload_extn       Size of payload
 *  \param [out]    payload_extn_present    Payload present flag
 *  \param [in]    ptr_scratch    Pointer to scratch memory
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_write_mct_data(mct_data_t *pstr_mct_data, WORD32 indep_flag,
                                    UWORD8 *ptr_payload_extn, WORD32 *size_payload_extn,
                                    WORD32 *payload_extn_present, UWORD8 *ptr_scratch)
{
  IA_ERRORCODE err_code = IA_NO_ERROR;
  LOOPIDX pair, band;
  WORD32 keep_tree = 0;
  WORD32 is_bandwise_coeff_present = 1;
  WORD32 is_mct_short = 0;
  WORD32 bit_cnt = 0;
  ia_bit_buf_struct it_bit_buff;
  ia_bit_buf_struct *ps_it_bit_buff;
  jmp_buf mct_bs_jmp_buf;
  err_code = setjmp(mct_bs_jmp_buf);
  if (err_code)
  {
    return IMPEGHE_EXE_NONFATAL_INSUFFICIENT_MCT_WRITE_BUFFER_SIZE;
  }
  WORD32 max_num_pair_idx =
      pstr_mct_data->num_ch_to_apply * (pstr_mct_data->num_ch_to_apply - 1) / 2 - 1;
  WORD32 num_of_pair_idx_bits = 1;

  ps_it_bit_buff = impeghe_create_bit_buffer(&it_bit_buff, ptr_scratch, 768);
  if (ps_it_bit_buff == NULL)
  {
    return -1;
  }
  ps_it_bit_buff->impeghe_jmp_buf = &mct_bs_jmp_buf;

  if (pstr_mct_data->prev_num_pairs == pstr_mct_data->num_pairs)
  {
    for (pair = 0; pair < pstr_mct_data->num_pairs; pair++)
    {
      if (pstr_mct_data->channel_pair[pair][0] == pstr_mct_data->prev_channel_pair[pair][0] ||
          pstr_mct_data->channel_pair[pair][1] == pstr_mct_data->prev_channel_pair[pair][1])
      {
        keep_tree = 1;
      }
      else
      {
        keep_tree = 0;
        break;
      }
    }
  }

  while (max_num_pair_idx >>= 1)
  {
    ++num_of_pair_idx_bits;
  }

  *size_payload_extn = 0;
  *payload_extn_present = 0;

  bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->mct_signalling_type, 2);

  if (!indep_flag)
  {
    bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, keep_tree, 1);
  }
  else
  {
    keep_tree = 0;
  }
  if (keep_tree == 0)
  {
    bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->num_pairs, 5);
  }

  for (pair = 0; pair < pstr_mct_data->num_pairs; pair++)
  {
    if (pstr_mct_data->mct_signalling_type > 1)
    {
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, 1 /* has stereo filling [pair]*/, 1);
    }

    if (keep_tree == 0)
    {
      WORD32 pair_idx = channel_pair_to_idx(pstr_mct_data->channel_pair[pair][0],
                                            pstr_mct_data->channel_pair[pair][1],
                                            pstr_mct_data->num_ch_to_apply);
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, pair_idx, num_of_pair_idx_bits);
    }

    bit_cnt +=
        impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->is_mct_mask_present[pair], 1);
    bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, is_bandwise_coeff_present, 1);

    if (pstr_mct_data->is_mct_mask_present[pair] || is_bandwise_coeff_present)
    {
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, is_mct_short, 1);
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->num_mask_bands[pair], 5);
    }
    else
    {
      pstr_mct_data->num_mask_bands[pair] = MAX_NUM_MC_BANDS;
    }

    if (pstr_mct_data->is_mct_mask_present[pair])
    {
      for (band = 0; band < pstr_mct_data->num_mask_bands[pair]; band++)
      {
        bit_cnt +=
            impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->pair_mct_mask[pair][band], 1);
      }
    }

    if ((pstr_mct_data->mct_signalling_type == 0) || (pstr_mct_data->mct_signalling_type == 2))
    {
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, pstr_mct_data->pred_dir[pair], 1);
    }
    if (!indep_flag)
    {
      bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, 0 /* mct_delta_time */, 1);
    }

    /* prediction */
    if ((pstr_mct_data->mct_signalling_type == 0) || (pstr_mct_data->mct_signalling_type == 2))
    {
      WORD32 prev_val = DEFAULT_ALPHA;
      if (is_bandwise_coeff_present)
      {
        for (band = 0; band < pstr_mct_data->num_mask_bands[pair]; band++)
        {
          if (pstr_mct_data->pair_mct_mask[pair][band] == 1)
          {
            WORD32 angle_bits = -(pstr_mct_data->pair_alpha[pair][band] - prev_val);
            angle_bits += CODE_BOOK_ALPHA_LAV / 2;
            prev_val = pstr_mct_data->pair_alpha[pair][band];
            bit_cnt +=
                impeghe_write_bits_buf(ps_it_bit_buff, impeghe_huffman_code_table[angle_bits][1],
                                       impeghe_huffman_code_table[angle_bits][0]);
          }
        }
      }
      else
      {
        WORD32 angle_bits = -(pstr_mct_data->pair_alpha[pair][0] - prev_val);
        angle_bits += CODE_BOOK_ALPHA_LAV / 2;
        bit_cnt +=
            impeghe_write_bits_buf(ps_it_bit_buff, impeghe_huffman_code_table[angle_bits][1],
                                   impeghe_huffman_code_table[angle_bits][0]);
      }
    }
    /* rotation */
    else if ((pstr_mct_data->mct_signalling_type == 1) ||
             (pstr_mct_data->mct_signalling_type == 3))
    {
      WORD32 prev_val = DEFAULT_BETA;
      if (is_bandwise_coeff_present)
      {
        for (band = 0; band < pstr_mct_data->num_mask_bands[pair]; band++)
        {
          if (pstr_mct_data->pair_mct_mask[pair][band] == 1)
          {
            WORD32 angle_bits = pstr_mct_data->pair_alpha[pair][band] - prev_val;
            if (angle_bits < 0)
              angle_bits += CODE_BOOK_BETA_LAV; /* wraparound */
            prev_val = pstr_mct_data->pair_alpha[pair][band];
            bit_cnt +=
                impeghe_write_bits_buf(ps_it_bit_buff, impeghe_mc_huff_code_tbl_angle[angle_bits],
                                       impeghe_mc_huff_len_tbl_angle[angle_bits]);
          }
        }
      }
      else
      {
        WORD32 angle_bits = pstr_mct_data->pair_alpha[pair][0] - prev_val;
        if (angle_bits < 0)
          angle_bits += CODE_BOOK_BETA_LAV; /* wraparound */
        bit_cnt +=
            impeghe_write_bits_buf(ps_it_bit_buff, impeghe_mc_huff_code_tbl_angle[angle_bits],
                                   impeghe_mc_huff_len_tbl_angle[angle_bits]);
      }
    }
  }

  if (bit_cnt % 8)
  {
    bit_cnt += impeghe_write_bits_buf(ps_it_bit_buff, 0, 8 - bit_cnt % 8);
  }

  /* copy payload to extension buffer */
  *size_payload_extn = bit_cnt / 8;

  if ((*size_payload_extn) > 0)
  {
    memcpy(ptr_payload_extn, ptr_scratch, *size_payload_extn);
    *payload_extn_present = 1;
  }

  for (pair = 0; pair < pstr_mct_data->num_pairs; pair++)
  {
    pstr_mct_data->prev_channel_pair[pair][0] = pstr_mct_data->channel_pair[pair][0];
    pstr_mct_data->prev_channel_pair[pair][1] = pstr_mct_data->channel_pair[pair][1];
  }
  pstr_mct_data->prev_num_pairs = pstr_mct_data->num_pairs;

  return IA_NO_ERROR;
}
