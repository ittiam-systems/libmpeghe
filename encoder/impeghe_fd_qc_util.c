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
#include "impeghe_fd_qc_util.h"
#include "impeghe_fd_qc_adjthr.h"

#define FRAME_LEN_BYTES_MODULO (1)
#define FRAME_LEN_BYTES_INT (2)

/**
 *  impeghe_qc_create
 *
 *  \brief Initialization of quantization spectrum and sfb's
 *
 *  \param [out] pstr_qc_main	Pointer to quantization structure
 *
 *  \return VOID
 */
VOID impeghe_qc_create(ia_qc_main_struct *pstr_qc_main)
{
  WORD32 i = 0;
  memset(pstr_qc_main, 0, sizeof(ia_qc_main_struct));

  for (i = 0; i < 2; i++)
  {
    memset(pstr_qc_main->str_qc_out.str_qc_out_chan[i].quant_spec, 0,
           sizeof(WORD16) * FRAME_LEN_LONG);
    memset(pstr_qc_main->str_qc_out.str_qc_out_chan[i].scalefactor, 0,
           sizeof(WORD16) * FRAME_LEN_LONG);
  }
  return;
}

/**
 *  impeghe_qc_init
 *
 *  \brief Initialization of quantization params
 *
 *  \param [in,out] pstr_qc_data		Pointer to quantization structure
 *  \param [in] 	max_bits			Maximum channel bits
 *  \param [in] 	sample_rate			Sample rate
 *  \param [in] 	bw_limit			Bandwidth limit
 *  \param [in] 	channels			Number of channels
 *
 *  \return VOID
 */
VOID impeghe_qc_init(ia_qc_data_struct *pstr_qc_data, const WORD32 max_bits, WORD32 sample_rate,
                     WORD32 bw_limit, WORD32 channels)
{
  FLOAT32 mean_pe;
  pstr_qc_data->tot_avg_bits = pstr_qc_data->avg_bits;
  pstr_qc_data->static_bits = 1;
  pstr_qc_data->avg_bits = (pstr_qc_data->avg_bits - pstr_qc_data->static_bits);
  pstr_qc_data->max_bits = channels * max_bits;
  pstr_qc_data->max_bitres_bits = channels * max_bits - pstr_qc_data->tot_avg_bits;
  pstr_qc_data->max_bitres_bits =
      pstr_qc_data->max_bitres_bits - (pstr_qc_data->max_bitres_bits % 8);
  pstr_qc_data->bit_res_lvl = pstr_qc_data->max_bitres_bits;
  pstr_qc_data->padding = sample_rate;

  pstr_qc_data->max_bit_fac =
      (FLOAT32)channels * (max_bits - 744) /
      (FLOAT32)(pstr_qc_data->tot_avg_bits ? pstr_qc_data->tot_avg_bits : 1);
  mean_pe = 10.0f * FRAME_LEN_LONG * bw_limit / (sample_rate / 2.0f);

  impeghe_adj_thr_init(&pstr_qc_data->str_adj_thr_ele, mean_pe,
                       (channels > 0) ? pstr_qc_data->ch_bitrate / channels : 0);

  return;
}

/**
 *  impeghe_calc_frame_len
 *
 *  \brief Calculate frame length
 *
 *  \param [in] bit_rate		Bit rate
 *  \param [in] sample_rate		Sample rate
 *  \param [in] mode			Mode in which frame length is expected
 *
 *  \return WORD32				Frame length
 */
static WORD32 impeghe_calc_frame_len(WORD32 bit_rate, WORD32 sample_rate, WORD32 mode)
{

  WORD32 result;

  result = ((FRAME_LEN_LONG) >> 3) * (bit_rate);
  switch (mode)
  {
  case FRAME_LEN_BYTES_MODULO:
    result %= sample_rate;
    break;
  case FRAME_LEN_BYTES_INT:
    result /= sample_rate;
    break;
  }

  return (result);
}

/**
 *  impeghe_get_frame_padding
 *
 *  \brief Evaluate if frame padding is needed
 *
 *  \param [in] bit_rate		Bit Rate
 *  \param [in] sample_rate		Sample rate
 *  \param [in] padding			Padding bits
 *
 *  \return WORD32				Padding flag
 */
static WORD32 impeghe_get_frame_padding(WORD32 bit_rate, WORD32 sample_rate, WORD32 *padding)
{
  WORD32 padding_on = 0;
  WORD32 difference;

  difference = impeghe_calc_frame_len(bit_rate, sample_rate, FRAME_LEN_BYTES_MODULO);

  *padding -= difference;

  if (*padding <= 0)
  {
    padding_on = 1;
    *padding += sample_rate;
  }

  return padding_on;
}

/**
 *  impeghe_adj_bitrate
 *
 *  \brief Adjustment of bit rate
 *
 *  \param [in] pstr_qc_data	Pointer to quantization struct
 *  \param [in] bit_rate		Bit rate
 *  \param [in] sample_rate		Sample rate
 *
 *  \return VOID
 */
VOID impeghe_adj_bitrate(ia_qc_data_struct *pstr_qc_data, WORD32 bit_rate, WORD32 sample_rate)
{
  WORD32 padding_on;
  WORD32 frame_len;
  WORD32 code_bits;
  WORD32 code_bits_prev;
  WORD32 total_bits = 0;

  padding_on = impeghe_get_frame_padding(bit_rate, sample_rate, &pstr_qc_data->padding);
  frame_len = padding_on + impeghe_calc_frame_len(bit_rate, sample_rate, FRAME_LEN_BYTES_INT);

  frame_len <<= 3;
  code_bits_prev = pstr_qc_data->tot_avg_bits - pstr_qc_data->static_bits;
  code_bits = frame_len - pstr_qc_data->static_bits;

  if (code_bits != code_bits_prev)
  {
    pstr_qc_data->avg_bits = (WORD32)code_bits;
    total_bits += pstr_qc_data->avg_bits;
    pstr_qc_data->avg_bits += code_bits - total_bits;
  }

  pstr_qc_data->tot_avg_bits = frame_len;

  return;
}

/**
 *  impeghe_calc_max_val_in_sfb
 *
 *  \brief Calculate max-value of quantized spectral coefficient
 *
 *  \param [in]  sfb_count                        Total number of scalefactor bands
 *  \param [in]  max_sfb_per_grp                  Maximum allowed scalefactor bands per window
 * group
 *  \param [in]  sfb_per_group        Total number of scalefactor bands per window
 * group
 *  \param [in]  ptr_sfb_offset                       Pointer to scalefactor band offsets
 *  \param [in]  ptr_quant_spec                       Pointer to quantized spectral coefficients
 *
 *  \return WORD32  Maximum value of quantized spectral coefficient
 */
WORD32 impeghe_calc_max_val_in_sfb(WORD32 sfb_count, WORD32 max_sfb_per_grp, WORD32 sfb_per_group,
                                   WORD32 *ptr_sfb_offset, WORD16 *ptr_quant_spec)
{
  WORD32 sfb;
  WORD32 max = 0;
  WORD32 sfb_offs;

  for (sfb_offs = 0; sfb_offs < sfb_count; sfb_offs += sfb_per_group)
  {
    for (sfb = 0; sfb < max_sfb_per_grp; sfb++)
    {
      WORD32 line;
      WORD32 local_max = 0;
      for (line = ptr_sfb_offset[sfb + sfb_offs]; line < ptr_sfb_offset[sfb + sfb_offs + 1];
           line++)
      {
        if (abs(ptr_quant_spec[line]) > local_max)
        {
          local_max = abs(ptr_quant_spec[line]);
        }
      }
      if (local_max > max)
      {
        max = local_max;
      }
    }
  }

  return max;
}
