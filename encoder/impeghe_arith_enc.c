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
#include "impeghe_cnst.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_arith_enc.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_rom.h"

#define ARITH_ESCAPE (16)

/**
 *  impeghe_arith_map_context
 *
 *  \brief Helper function with arithmetic encoding context mapping
 *
 *  \param [in]		pres_n				Index of the present code word
 *  \param [in]		prev_n				Index of the previous code word
 *  \param [in,out]	ptr_c_prev			Pointer to previous code word buffer
 *  \param [in,out]	ptr_c_pres			Pointer to code word buffer
 *  \param [in]		arith_reset_flag	State reset flag
 *  \param [unused]	pstr_scratch		Pointer to scratch structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_arith_map_context(WORD32 pres_n, WORD32 prev_n, WORD32 *ptr_c_prev,
                                      WORD32 *ptr_c_pres, WORD32 arith_reset_flag,
                                      impeghe_scratch_mem *pstr_scratch)
{
  LOOPIDX i, k;
  FLOAT32 ratio;
  if (arith_reset_flag)
  {
    memset(ptr_c_pres, 0, 516 * sizeof(WORD32));
    memset(ptr_c_prev, 0, 516 * sizeof(WORD32));
  }
  else
  {
    memcpy(&pstr_scratch->p_c_prev[2], &ptr_c_prev[2], (prev_n / 2 + 2) * sizeof(WORD32));
    memcpy(&pstr_scratch->p_c_pres[2], &ptr_c_pres[2], (prev_n / 2 + 2) * sizeof(WORD32));

    ratio = (FLOAT32)(prev_n) / (FLOAT32)(pres_n);
    for (i = 0; i < (pres_n / 2); i++)
    {
      k = (WORD32)((FLOAT32)(i)*ratio);
      ptr_c_pres[2 + i] = pstr_scratch->p_c_pres[2 + k];
      ptr_c_prev[2 + i] = pstr_scratch->p_c_prev[2 + k];
    }

    ptr_c_pres[(pres_n / 2) + 2] = pstr_scratch->p_c_pres[(prev_n / 2) + 2];
    ptr_c_pres[(pres_n / 2) + 3] = pstr_scratch->p_c_pres[(prev_n / 2) + 3];
    ptr_c_prev[(pres_n / 2) + 2] = pstr_scratch->p_c_prev[(prev_n / 2) + 2];
    ptr_c_prev[(pres_n / 2) + 3] = pstr_scratch->p_c_prev[(prev_n / 2) + 3];
  }
  return;
}

/**
 *  impeghe_arith_get_context
 *
 *  \brief Gets arithmetic encoding state
 *
 *  \param [in]	ptr_c_pres	Pointer to present code word buffer
 *  \param [in]	ptr_c_prev	Pointer to previous code word buffer
 *  \param [out]	ptr_s		Pointer to code word buffer
 *  \param [in]	idx		Index
 *
 *  \return WORD32 Coded value
 *
 */
static WORD32 impeghe_arith_get_context(WORD32 *ptr_c_pres, WORD32 *ptr_c_prev, WORD32 *ptr_s,
                                        WORD32 idx)
{
  WORD32 s_tmp = *ptr_s;

  s_tmp = s_tmp >> 4;
  s_tmp = s_tmp + (ptr_c_prev[idx + 1] << 12);
  s_tmp = (s_tmp & 0xFFF0) + ptr_c_pres[idx - 1];

  *ptr_s = s_tmp;

  if (idx > 3)
  {
    if ((ptr_c_pres[idx - 1] + ptr_c_pres[idx - 2] + ptr_c_pres[idx - 3]) < 5)
    {
      return (s_tmp + 0x10000);
    }
  }

  return (s_tmp);
}

/**
 *  impeghe_arith_get_pk
 *
 *  \brief Gets the peak value from look-up
 *
 *  \param [in]	c	code word param
 *
 *  \return UWORD16 cumulative frequency table index
 *
 */
static UWORD16 impeghe_arith_get_pk(WORD32 c)
{
  WORD32 j;
  WORD32 i, i_min, i_max;

  i_min = -1;
  i = i_min;
  i_max = (sizeof(impeghe_ari_lookup_m) / sizeof(impeghe_ari_lookup_m[0])) - 1;
  while ((i_max - i_min) > 1)
  {
    i = i_min + ((i_max - i_min) / 2);
    j = impeghe_ari_hash_m[i];
    if (c < j)
      i_max = i;
    else if (c > j)
      i_min = i;
    else
      return (impeghe_ari_hash_m_lsb[i]);
  }

  return (impeghe_ari_lookup_m[i_max]);
}

/**
 *  impeghe_copy_bit_buf
 *
 *  \brief Copies bit-buffer structure contents from source to destination
 *
 *  \param [out]	it_bit_buff_dest	Pointer to destination bit-buffer structure
 *  \param [in]	it_bit_buff_src		Pointer to source bit-buffer structure
 *
 *  \return VOID
 *
 */
static VOID impeghe_copy_bit_buf(ia_bit_buf_struct *it_bit_buff_dest,
                                 ia_bit_buf_struct *it_bit_buff_src)
{
  if (it_bit_buff_src != NULL && it_bit_buff_dest != NULL)
  {
    it_bit_buff_dest->cnt_bits = it_bit_buff_src->cnt_bits;
    it_bit_buff_dest->ptr_write_next = it_bit_buff_src->ptr_write_next;
    it_bit_buff_dest->write_position = it_bit_buff_src->write_position;
  }
  return;
}

/**
 *  impeghe_arith_encode_level2
 *
 *  \brief Second level arithmetic encoding function
 *
 *  \param [in,out]		it_bit_buff	Pointer to bit-buffer
 *  \param [in]			bit_pos			Bits counter
 *  \param [in,out]		ptr_c_pres	Pointer to present code word
 *  \param [in,out]		ptr_c_prev	Pointer to previous code word
 *  \param [in]			ptr_quant		Pointer to quantized coeffs
 *  \param [in]			n			Half the number of co-effs
 *  \param [in]			nt			Number of co-effs
 *  \param [in]			use_stop	Arithmetic encoding control value
 *
 *  \return WORD32 Number of bits written
 *
 */
static WORD32 impeghe_arith_encode_level2(ia_bit_buf_struct *it_bit_buff, WORD32 bit_pos,
                                          WORD32 *ptr_c_pres, WORD32 *ptr_c_prev,
                                          WORD32 *ptr_quant, WORD32 n, WORD32 nt, WORD32 use_stop)
{
  LOOPIDX i, l;
  WORD32 qs[32];
  impeghe_state_arith as, as_stop;

  WORD32 a, b, a1, b1, m;
  WORD32 s, t, lev, esc_nb;
  UWORD16 pki;
  WORD32 bp_start = bit_pos;
  WORD32 bp_stop = bit_pos;
  WORD32 stop = 0;
  WORD32 sopt;
  WORD32 a2, b2;
  ia_bit_buf_struct it_bit_buff_temp;
  memset(&it_bit_buff_temp, 0, sizeof(it_bit_buff_temp));
  impeghe_copy_bit_buf(&it_bit_buff_temp, it_bit_buff);

  as.low = 0;
  as.high = 65535;
  as.value = 0;

  sopt = ptr_c_prev[0] << 12;

  for (i = 0; i < n; i++)
  {

    if ((use_stop == 1 || use_stop == 2) && (stop == 0))
    {
      LOOPIDX j;

      stop = 1;
      for (j = i; j < n; j++)
      {
        if (ptr_quant[2 * j] != 0 || ptr_quant[2 * j + 1] != 0)
        {
          stop = 0;
          break;
        }
      }

      if (stop)
      {
        s = impeghe_arith_get_context(ptr_c_pres, ptr_c_prev, &sopt, i);
        t = s & 0xFFFFF;

        pki = impeghe_arith_get_pk(t);

        if (use_stop == 1)
        {
          bit_pos = impeghe_arith_encode(it_bit_buff, bit_pos, &as, ARITH_ESCAPE,
                                         impeghe_ari_cf_m[pki]);
          pki = impeghe_arith_get_pk(t + (1 << 17));
          bit_pos = impeghe_arith_encode(it_bit_buff, bit_pos, &as, 0, impeghe_ari_cf_m[pki]);

          break;
        }
        else
        {
          bp_stop = bit_pos;
          as_stop.low = as.low;
          as_stop.high = as.high;
          as_stop.value = as.value;

          bp_stop =
              impeghe_arith_encode(NULL, bp_stop, &as_stop, ARITH_ESCAPE, impeghe_ari_cf_m[pki]);

          pki = impeghe_arith_get_pk(t + (1 << 17));
          bp_stop = impeghe_arith_encode(NULL, bp_stop, &as_stop, (0), impeghe_ari_cf_m[pki]);
        }
      }
    }
    s = impeghe_arith_get_context(ptr_c_pres, ptr_c_prev, &sopt, i);
    t = s & 0xFFFFF;

    a = ptr_quant[2 * i];
    b = ptr_quant[2 * i + 1];
    a1 = abs(a);
    b1 = abs(b);

    ptr_c_pres[i] = a1 + b1 + 1;
    if (ptr_c_pres[i] > 0xF)
    {
      ptr_c_pres[i] = 0xF;
    }

    lev = 0;
    esc_nb = 0;

    while ((a1) > 3 || (b1) > 3)
    {
      pki = impeghe_arith_get_pk(t + (esc_nb << 17));

      bit_pos =
          impeghe_arith_encode(it_bit_buff, bit_pos, &as, ARITH_ESCAPE, impeghe_ari_cf_m[pki]);

      qs[lev++] = (a1 & 1) | ((b1 & 1) << 1);
      a1 >>= 1;
      b1 >>= 1;
      esc_nb++;

      if (esc_nb > 7)
      {
        esc_nb = 7;
      }
    }
    m = a1 + (b1 << 2);
    pki = impeghe_arith_get_pk(t + (esc_nb << 17));
    bit_pos = impeghe_arith_encode(it_bit_buff, bit_pos, &as, m, impeghe_ari_cf_m[pki]);

    a2 = a1;
    b2 = b1;

    for (l = lev - 1; l >= 0; l--)
    {

      WORD32 lsbidx = (a2 == 0) ? 1 : ((b2 == 0) ? 0 : 2);
      bit_pos = impeghe_arith_encode(it_bit_buff, bit_pos, &as, qs[l], impeghe_ari_cf_r[lsbidx]);

      a2 = (a2 << 1) | (qs[l] & 1);
      b2 = (b2 << 1) | ((qs[l] >> 1) & 1);
    }
  }

  if (use_stop == 2)
  {
    bit_pos = impeghe_arith_done(it_bit_buff, bit_pos, &as);
    if (stop)
    {
      bp_stop = impeghe_arith_done(NULL, bp_stop, &as_stop);

      if (bp_stop < bit_pos)
      {
        impeghe_copy_bit_buf(it_bit_buff, &it_bit_buff_temp);
        bit_pos = impeghe_arith_encode_level2(it_bit_buff, bp_start, ptr_c_pres, ptr_c_prev,
                                              ptr_quant, n, nt, 1);
      }
      else
      {
        impeghe_copy_bit_buf(it_bit_buff, &it_bit_buff_temp);
        bit_pos = impeghe_arith_encode_level2(it_bit_buff, bp_start, ptr_c_pres, ptr_c_prev,
                                              ptr_quant, n, nt, 0);
      }
    }
    else
    {
      impeghe_copy_bit_buf(it_bit_buff, &it_bit_buff_temp);
      bit_pos = impeghe_arith_encode_level2(it_bit_buff, bp_start, ptr_c_pres, ptr_c_prev,
                                            ptr_quant, n, nt, 0);
    }
  }
  else
  {
    bit_pos = impeghe_arith_done(it_bit_buff, bit_pos, &as);

    for (; i < nt; i++)
    {
      ptr_c_pres[i] = 1;
    }

    for (i = 0; i < n; i++)
    {

      if (ptr_quant[2 * i] != 0)
      {
        if (ptr_quant[2 * i] > 0)
        {
          impeghe_write_bits_buf(it_bit_buff, 1, 1);
          bit_pos++;
        }
        else
        {
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
          bit_pos++;
        }
      }

      if (ptr_quant[2 * i + 1] != 0)
      {
        if (ptr_quant[2 * i + 1] > 0)
        {
          impeghe_write_bits_buf(it_bit_buff, 1, 1);
          bit_pos++;
        }
        else
        {
          impeghe_write_bits_buf(it_bit_buff, 0, 1);
          bit_pos++;
        }
      }
    }

    for (i = 0; i < nt; i++)
    {
      ptr_c_prev[i] = ptr_c_pres[i];
      ptr_c_pres[i] = 1;
    }
  }

  return bit_pos;
}

/**
 *  impeghe_arith_enc_spec
 *
 *  \brief Arithmetic encodes the spectral co-efficients
 *
 *  \param [in,out]				it_bit_buf				Pointer to
 * bit-buffer
 *  \param [in]					window_sequence			Window sequence
 *  \param [in]					ptr_x_ac_enc			Pointer to
 * quantized
 * coeffs
 *  \param [in]					max_spec_coefficients	Maximum spectral coeffs
 *  \param [in,out]				ptr_c_pres				Pointer to
 * present
 * code
 * word
 *  \param [in,out]				ptr_c_prev				Pointer to
 * previous
 * code
 * word
 *  \param [in,out]				ptr_size_prev			Pointer to
 * previous
 * code
 * word
 * size
 *  \param [in]					arith_reset_flag		Reset flag
 *  \param [in]					pstr_scratch				Pointer to
 * Scratch
 * buffer
 *
 *  \return WORD32 Number of bits written
 *
 */
WORD32 impeghe_arith_enc_spec(ia_bit_buf_struct *it_bit_buf, WORD32 window_sequence,
                              WORD32 *ptr_x_ac_enc, WORD32 max_spec_coefficients,
                              WORD32 *ptr_c_pres, WORD32 *ptr_c_prev, WORD32 *ptr_size_prev,
                              WORD32 arith_reset_flag, impeghe_scratch_mem *pstr_scratch)
{
  LOOPIDX i;
  WORD32 write_flag = (it_bit_buf != NULL);
  WORD32 size;
  WORD32 num_wins = (window_sequence == EIGHT_SHORT_SEQUENCE) ? MAX_SHORT_WINDOWS : 1;
  WORD32 bits_data_written = 0;

  switch (window_sequence)
  {
  case ONLY_LONG_SEQUENCE:
  case LONG_START_SEQUENCE:
  case STOP_START_SEQUENCE:
  case LONG_STOP_SEQUENCE:
    size = FRAME_LEN_LONG;
    break;
  case EIGHT_SHORT_SEQUENCE:
    size = FRAME_LEN_SHORT;
    break;
  default:
    size = FRAME_LEN_SHORT;
    break;
  }

  impeghe_arith_map_context(size, *ptr_size_prev, ptr_c_pres, ptr_c_prev, arith_reset_flag,
                            pstr_scratch);

  if (max_spec_coefficients > 0)
  {
    for (i = 0; i < num_wins; i++)
    {
      bits_data_written = impeghe_arith_encode_level2(
          it_bit_buf, bits_data_written, ptr_c_pres + 2, ptr_c_prev + 2, &ptr_x_ac_enc[i * size],
          max_spec_coefficients / 2, size / 2, 2);
    }
  }

  if (write_flag)
  {
    *ptr_size_prev = size;
  }

  return bits_data_written;
}
/**
 *  impeghe_tcx_coding
 *
 *  \brief Transform coding excitation (TCX) coding
 *
 *  \param [in,out]		it_bit_buff		Pointer to bit-buffer
 *  \param [in]			tcx_size		TCX size
 *  \param [in]			max_tcx_size	Maximum TCX size
 *  \param [in]			ptr_quant		Pointer to quantized coeffs
 *  \param [in,out]		ptr_c_pres			Pointer to present code word
 *  \param [in,out]		ptr_c_prev			Pointer to present code word
 *  \param [in]			pstr_scratch		Pointer to scratch structure
 *
 *  \return WORD32 Number of bits written
 *
 */
WORD32 impeghe_tcx_coding(ia_bit_buf_struct *it_bit_buff, WORD32 tcx_size, WORD32 max_tcx_size,
                          WORD32 *ptr_quant, WORD32 *ptr_c_pres, WORD32 *ptr_c_prev,
                          impeghe_scratch_mem *pstr_scratch)
{
  WORD32 bits_written = 0;

  impeghe_arith_map_context(tcx_size, max_tcx_size, ptr_c_pres, ptr_c_prev, 0, pstr_scratch);

  bits_written =
      impeghe_arith_encode_level2(it_bit_buff, bits_written, ptr_c_pres + 2, ptr_c_prev + 2,
                                  ptr_quant, tcx_size / 2, tcx_size / 2, 2);

  impeghe_arith_map_context(max_tcx_size, tcx_size, ptr_c_pres, ptr_c_prev, 0, pstr_scratch);

  return bits_written;
}
/**
 *  impeghe_arith_done
 *
 *  \brief Updates bit buffers and state after arithmetic encode
 *
 *  \param [in,out]	it_bit_buff	Pointer to bit-buffer
 *  \param [in]		bit_pos			Bit counter
 *  \param [in,out]	pstr_state			Pointer to arithmetic encoding state
 * structure
 *
 *  \return WORD32 Number of bits written
 *
 */
WORD32 impeghe_arith_done(ia_bit_buf_struct *it_bit_buff, WORD32 bit_pos,
                          impeghe_state_arith *pstr_state)
{
  WORD32 low, high;
  WORD32 bits_to_follow;

  low = pstr_state->low;
  high = pstr_state->high;
  bits_to_follow = pstr_state->value + 1;

  if (low < 16384)
  {
    impeghe_write_bits_buf(it_bit_buff, 0, 1);
    bit_pos++;
    while (bits_to_follow)
    {
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      bit_pos++;
      bits_to_follow--;
    }
  }
  else
  {
    impeghe_write_bits_buf(it_bit_buff, 1, 1);
    bit_pos++;
    while (bits_to_follow)
    {
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
      bit_pos++;
      bits_to_follow--;
    }
  }

  pstr_state->low = low;
  pstr_state->high = high;
  pstr_state->value = bits_to_follow;

  return bit_pos;
}

/**
 *  impeghe_arith_encode
 *
 *  \brief Arithmetic encoding function
 *
 *  \param [in,out]	it_bit_buff		Pointer to bit-buffer
 *  \param [in]		bit_pos				Bit counter
 *  \param [in,out]	pstr_state		Pointer to arithmetic encoding state
 * structure
 *  \param [in]		symbol			Symbol to be encoded
 *  \param [in]		ptr_cum_freq		Pointer to cumulative frequency table
 *
 *  \return WORD32 Number of bits written
 *
 */
WORD32 impeghe_arith_encode(ia_bit_buf_struct *it_bit_buff, WORD32 bit_pos,
                            impeghe_state_arith *pstr_state, WORD32 symbol,
                            UWORD16 const *ptr_cum_freq)
{
  WORD32 low, high, range;
  WORD32 bits_to_follow;

  high = pstr_state->high;
  low = pstr_state->low;
  range = high - low + 1;

  if (symbol > 0)
  {
    high = low + ((range * ptr_cum_freq[symbol - 1]) >> 14) - 1;
  }

  low = low + ((range * ptr_cum_freq[symbol]) >> 14);

  bits_to_follow = pstr_state->value;

  for (;;)
  {
    if (high < 32768)
    {
      impeghe_write_bits_buf(it_bit_buff, 0, 1);
      bit_pos++;
      while (bits_to_follow)
      {
        impeghe_write_bits_buf(it_bit_buff, 1, 1);
        bit_pos++;
        bits_to_follow--;
      }
    }
    else if (low >= 32768)
    {
      impeghe_write_bits_buf(it_bit_buff, 1, 1);
      bit_pos++;
      while (bits_to_follow)
      {
        impeghe_write_bits_buf(it_bit_buff, 0, 1);
        bit_pos++;
        bits_to_follow--;
      }
      low -= 32768;
      high -= 32768;
    }
    else if (low >= 16384 && high < 49152)
    {
      bits_to_follow += 1;
      low -= 16384;
      high -= 16384;
    }
    else
      break;

    low += low;
    high += high + 1;
  }

  pstr_state->low = low;
  pstr_state->high = high;
  pstr_state->value = bits_to_follow;

  return bit_pos;
}
