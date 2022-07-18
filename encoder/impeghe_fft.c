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
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"

#include "impeghe_block_switch_const.h"
#include "impeghe_rom.h"

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
#include "impeghe_fft.h"

#include "impeghe_basic_ops_flt.h"
#ifndef PI
#define PI 3.141592653589795
#endif

#define DIG_REV(i, m, j)                                                                         \
  do                                                                                             \
  {                                                                                              \
    unsigned _ = (i);                                                                            \
    _ = ((_ & 0x33333333) << 2) | ((_ & ~0x33333333) >> 2);                                      \
    _ = ((_ & 0x0F0F0F0F) << 4) | ((_ & ~0x0F0F0F0F) >> 4);                                      \
    _ = ((_ & 0x00FF00FF) << 8) | ((_ & ~0x00FF00FF) >> 8);                                      \
    (j) = _ >> (m);                                                                              \
  } while (0)

#define ia_add_flt(a, b) ((a) + (b))
#define ia_sub_flt(a, b) ((a) - (b))
#define ia_mac_flt(x, a, b) ((x) + (a) * (b))
#define ia_msu_flt(x, a, b) ((x) - (a) * (b))
#define ia_mul_flt(a, b) ((a) * (b))

/**
 *  impeghe_calc_norm
 *
 *  \brief Normalize the input value
 *
 *  \param [in] a	Integer input
 *
 *  \return WORD32	Normalized value of input
 */
static PLATFORM_INLINE WORD8 impeghe_calc_norm(WORD32 a)
{
  WORD8 norm_val;

  if (a == 0)
  {
    norm_val = 31;
  }
  else
  {
    if (a == (WORD32)0xffffffffL)
    {
      norm_val = 31;
    }
    else
    {
      if (a < 0)
      {
        a = ~a;
      }
      for (norm_val = 0; a < (WORD32)0x40000000L; norm_val++)
      {
        a <<= 1;
      }
    }
  }

  return norm_val;
}

/**
 *  impeghe_complex_3point_fft
 *
 *  \brief 3 point FFT
 *
 *  \param [in]  ptr_in  Pointer to input data
 *  \param [out] ptr_out Pointer to output data
 *
 *  \return VOID
 */
static PLATFORM_INLINE VOID impeghe_complex_3point_fft(FLOAT32 *ptr_in, FLOAT32 *ptr_out)
{
  FLOAT32 add_r, sub_r;
  FLOAT32 add_i, sub_i;
  FLOAT32 x01r, x01i, temp;
  FLOAT32 p1, p2, p3, p4;
  FLOAT64 sinmu;

  sinmu = 0.866025403784439;

  x01r = ptr_in[0] + ptr_in[2];
  x01i = ptr_in[1] + ptr_in[3];

  add_r = ptr_in[2] + ptr_in[4];
  add_i = ptr_in[3] + ptr_in[5];

  sub_r = ptr_in[2] - ptr_in[4];
  sub_i = ptr_in[3] - ptr_in[5];

  p1 = add_r / (FLOAT32)2.0;
  p4 = add_i / (FLOAT32)2.0;
  p2 = (FLOAT32)((FLOAT64)sub_i * sinmu);
  p3 = (FLOAT32)((FLOAT64)sub_r * sinmu);

  temp = ptr_in[0] - p1;

  ptr_out[0] = x01r + ptr_in[4];
  ptr_out[1] = x01i + ptr_in[5];
  ptr_out[2] = temp + p2;
  ptr_out[3] = (ptr_in[1] - p3) - p4;
  ptr_out[4] = temp - p2;
  ptr_out[5] = (ptr_in[1] + p3) - p4;

  return;
}

/**
 *  impeghe_complex_fft_p2
 *
 *  \brief Radix 2 complex n-point FFT
 *
 *  \param [in,out] ptr_x				Pointer to data buffer
 *  \param [in]     nlength				n-point for FFT
 *  \param [in]     ptr_scratch_fft_p2_y	Pointer to scratch buffer
 *
 *  \return VOID
 */
static VOID impeghe_complex_fft_p2(FLOAT32 *ptr_x, WORD32 nlength, FLOAT32 *ptr_scratch_fft_p2_y)
{
  WORD32 i, j, k, n_stages, h2;
  FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
  WORD32 del, nodespacing, in_loop_cnt;
  WORD32 not_power_4;
  WORD32 dig_rev_shift;
  FLOAT32 *ptr_y = ptr_scratch_fft_p2_y;
  WORD32 mpass = nlength;
  WORD32 npoints = nlength;
  FLOAT32 *ptr_y2 = ptr_y;
  const FLOAT64 *ptr_w;

  dig_rev_shift = impeghe_calc_norm(mpass) + 1 - 16;
  n_stages = 30 - impeghe_calc_norm(mpass);
  not_power_4 = n_stages & 1;

  n_stages = n_stages >> 1;

  ptr_w = impeghe_twiddle_table_fft_32x32;

  for (i = 0; i < npoints; i += 4)
  {
    FLOAT32 *ptr_inp = ptr_x;
    FLOAT32 tmk;

    DIG_REV(i, dig_rev_shift, h2);
    if (not_power_4)
    {
      h2 += 1;
      h2 &= ~1;
    }
    ptr_inp += (h2);

    x0r = *ptr_inp;
    x0i = *(ptr_inp + 1);
    ptr_inp += (npoints >> 1);

    x1r = *ptr_inp;
    x1i = *(ptr_inp + 1);
    ptr_inp += (npoints >> 1);

    x2r = *ptr_inp;
    x2i = *(ptr_inp + 1);
    ptr_inp += (npoints >> 1);

    x3r = *ptr_inp;
    x3i = *(ptr_inp + 1);

    x0r = x0r + x2r;
    x0i = x0i + x2i;

    tmk = x0r - x2r;
    x2r = tmk - x2r;
    tmk = x0i - x2i;
    x2i = tmk - x2i;

    x1r = x1r + x3r;
    x1i = x1i + x3i;

    tmk = x1r - x3r;
    x3r = tmk - x3r;
    tmk = x1i - x3i;
    x3i = tmk - x3i;

    x0r = x0r + x1r;
    x0i = x0i + x1i;

    tmk = x0r - x1r;
    x1r = tmk - x1r;
    tmk = x0i - x1i;
    x1i = tmk - x1i;

    x2r = x2r + x3i;
    x2i = x2i - x3r;

    tmk = x2r - x3i;
    x3i = tmk - x3i;
    tmk = x2i + x3r;
    x3r = tmk + x3r;

    *ptr_y2++ = x0r;
    *ptr_y2++ = x0i;
    *ptr_y2++ = x2r;
    *ptr_y2++ = x2i;
    *ptr_y2++ = x1r;
    *ptr_y2++ = x1i;
    *ptr_y2++ = x3i;
    *ptr_y2++ = x3r;
  }
  ptr_y2 -= 2 * npoints;
  del = 4;
  nodespacing = 64;
  in_loop_cnt = npoints >> 4;
  for (i = n_stages - 1; i > 0; i--)
  {
    const FLOAT64 *ptr_twiddles = ptr_w;
    FLOAT32 *ptr_data = ptr_y2;
    FLOAT64 W1, W2, W3, W4, W5, W6;
    WORD32 sec_loop_cnt;

    for (k = in_loop_cnt; k != 0; k--)
    {
      x0r = (*ptr_data);
      x0i = (*(ptr_data + 1));
      ptr_data += (del << 1);

      x1r = (*ptr_data);
      x1i = (*(ptr_data + 1));
      ptr_data += (del << 1);

      x2r = (*ptr_data);
      x2i = (*(ptr_data + 1));
      ptr_data += (del << 1);

      x3r = (*ptr_data);
      x3i = (*(ptr_data + 1));
      ptr_data -= 3 * (del << 1);

      x0r = x0r + x2r;
      x0i = x0i + x2i;
      x2r = x0r - (x2r * 2);
      x2i = x0i - (x2i * 2);
      x1r = x1r + x3r;
      x1i = x1i + x3i;
      x3r = x1r - (x3r * 2);
      x3i = x1i - (x3i * 2);

      x0r = x0r + x1r;
      x0i = x0i + x1i;
      x1r = x0r - (x1r * 2);
      x1i = x0i - (x1i * 2);
      x2r = x2r + x3i;
      x2i = x2i - x3r;
      x3i = x2r - (x3i * 2);
      x3r = x2i + (x3r * 2);

      *ptr_data = x0r;
      *(ptr_data + 1) = x0i;
      ptr_data += (del << 1);

      *ptr_data = x2r;
      *(ptr_data + 1) = x2i;
      ptr_data += (del << 1);

      *ptr_data = x1r;
      *(ptr_data + 1) = x1i;
      ptr_data += (del << 1);

      *ptr_data = x3i;
      *(ptr_data + 1) = x3r;
      ptr_data += (del << 1);
    }
    ptr_data = ptr_y2 + 2;

    sec_loop_cnt = (nodespacing * del);
    sec_loop_cnt = (sec_loop_cnt / 4) + (sec_loop_cnt / 8) - (sec_loop_cnt / 16) +
                   (sec_loop_cnt / 32) - (sec_loop_cnt / 64) + (sec_loop_cnt / 128) -
                   (sec_loop_cnt / 256);
    j = nodespacing;

    for (j = nodespacing; j <= sec_loop_cnt; j += nodespacing)
    {
      W1 = *(ptr_twiddles + j);
      W4 = *(ptr_twiddles + j + 257);
      W2 = *(ptr_twiddles + (j << 1));
      W5 = *(ptr_twiddles + (j << 1) + 257);
      W3 = *(ptr_twiddles + j + (j << 1));
      W6 = *(ptr_twiddles + j + (j << 1) + 257);

      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

        ptr_data += (del << 1);

        x1r = *ptr_data;
        x1i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x2r = *ptr_data;
        x2i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x3r = *ptr_data;
        x3i = *(ptr_data + 1);
        ptr_data -= 3 * (del << 1);

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4));
        x1i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x1r, W4), (FLOAT64)x1i, W1);
        x1r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x2r, W2) - impeghe_dmult((FLOAT64)x2i, W5));
        x2i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x2r, W5), (FLOAT64)x2i, W2);
        x2r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x3r, W3) - impeghe_dmult((FLOAT64)x3i, W6));
        x3i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x3r, W6), (FLOAT64)x3i, W3);
        x3r = tmp;

        x0r = (*ptr_data);
        x0i = (*(ptr_data + 1));

        x0r = x0r + (x2r);
        x0i = x0i + (x2i);
        x2r = x0r - (x2r * 2);
        x2i = x0i - (x2i * 2);
        x1r = x1r + x3r;
        x1i = x1i + x3i;
        x3r = x1r - (x3r * 2);
        x3i = x1i - (x3i * 2);

        x0r = x0r + (x1r);
        x0i = x0i + (x1i);
        x1r = x0r - (x1r * 2);
        x1i = x0i - (x1i * 2);
        x2r = x2r + (x3i);
        x2i = x2i - (x3r);
        x3i = x2r - (x3i * 2);
        x3r = x2i + (x3r * 2);

        *ptr_data = x0r;
        *(ptr_data + 1) = x0i;
        ptr_data += (del << 1);

        *ptr_data = x2r;
        *(ptr_data + 1) = x2i;
        ptr_data += (del << 1);

        *ptr_data = x1r;
        *(ptr_data + 1) = x1i;
        ptr_data += (del << 1);

        *ptr_data = x3i;
        *(ptr_data + 1) = x3r;
        ptr_data += (del << 1);
      }
      ptr_data -= 2 * npoints;
      ptr_data += 2;
    }
    for (; j <= (nodespacing * del) >> 1; j += nodespacing)
    {
      W1 = *(ptr_twiddles + j);
      W4 = *(ptr_twiddles + j + 257);
      W2 = *(ptr_twiddles + (j << 1));
      W5 = *(ptr_twiddles + (j << 1) + 257);
      W3 = *(ptr_twiddles + j + (j << 1) - 256);
      W6 = *(ptr_twiddles + j + (j << 1) + 1);

      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

        ptr_data += (del << 1);

        x1r = *ptr_data;
        x1i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x2r = *ptr_data;
        x2i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x3r = *ptr_data;
        x3i = *(ptr_data + 1);
        ptr_data -= 3 * (del << 1);

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4));
        x1i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x1r, W4), (FLOAT64)x1i, W1);
        x1r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x2r, W2) - impeghe_dmult((FLOAT64)x2i, W5));
        x2i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x2r, W5), (FLOAT64)x2i, W2);
        x2r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x3r, W6) + impeghe_dmult((FLOAT64)x3i, W3));
        x3i = (FLOAT32)(-impeghe_dmult((FLOAT64)x3r, W3) + impeghe_dmult((FLOAT64)x3i, W6));
        x3r = tmp;

        x0r = (*ptr_data);
        x0i = (*(ptr_data + 1));

        x0r = x0r + (x2r);
        x0i = x0i + (x2i);
        x2r = x0r - (x2r * 2);
        x2i = x0i - (x2i * 2);
        x1r = x1r + x3r;
        x1i = x1i + x3i;
        x3r = x1r - (x3r * 2);
        x3i = x1i - (x3i * 2);

        x0r = x0r + (x1r);
        x0i = x0i + (x1i);
        x1r = x0r - (x1r * 2);
        x1i = x0i - (x1i * 2);
        x2r = x2r + (x3i);
        x2i = x2i - (x3r);
        x3i = x2r - (x3i * 2);
        x3r = x2i + (x3r * 2);

        *ptr_data = x0r;
        *(ptr_data + 1) = x0i;
        ptr_data += (del << 1);

        *ptr_data = x2r;
        *(ptr_data + 1) = x2i;
        ptr_data += (del << 1);

        *ptr_data = x1r;
        *(ptr_data + 1) = x1i;
        ptr_data += (del << 1);

        *ptr_data = x3i;
        *(ptr_data + 1) = x3r;
        ptr_data += (del << 1);
      }
      ptr_data -= 2 * npoints;
      ptr_data += 2;
    }
    for (; j <= sec_loop_cnt * 2; j += nodespacing)
    {
      W1 = *(ptr_twiddles + j);
      W4 = *(ptr_twiddles + j + 257);
      W2 = *(ptr_twiddles + (j << 1) - 256);
      W5 = *(ptr_twiddles + (j << 1) + 1);
      W3 = *(ptr_twiddles + j + (j << 1) - 256);
      W6 = *(ptr_twiddles + j + (j << 1) + 1);

      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

        ptr_data += (del << 1);

        x1r = *ptr_data;
        x1i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x2r = *ptr_data;
        x2i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x3r = *ptr_data;
        x3i = *(ptr_data + 1);
        ptr_data -= 3 * (del << 1);

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4));
        x1i = (FLOAT32)impeghe_dmac(impeghe_dmult(x1r, W4), x1i, W1);
        x1r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x2r, W5) + impeghe_dmult((FLOAT64)x2i, W2));
        x2i = (FLOAT32)(-impeghe_dmult(x2r, W2) + impeghe_dmult(x2i, W5));
        x2r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x3r, W6) + impeghe_dmult((FLOAT64)x3i, W3));
        x3i = (FLOAT32)(-impeghe_dmult((FLOAT64)x3r, W3) + impeghe_dmult((FLOAT64)x3i, W6));
        x3r = tmp;

        x0r = (*ptr_data);
        x0i = (*(ptr_data + 1));

        x0r = x0r + (x2r);
        x0i = x0i + (x2i);
        x2r = x0r - (x2r * 2);
        x2i = x0i - (x2i * 2);
        x1r = x1r + x3r;
        x1i = x1i + x3i;
        x3r = x1r - (x3r * 2);
        x3i = x1i - (x3i * 2);

        x0r = x0r + (x1r);
        x0i = x0i + (x1i);
        x1r = x0r - (x1r * 2);
        x1i = x0i - (x1i * 2);
        x2r = x2r + (x3i);
        x2i = x2i - (x3r);
        x3i = x2r - (x3i * 2);
        x3r = x2i + (x3r * 2);

        *ptr_data = x0r;
        *(ptr_data + 1) = x0i;
        ptr_data += (del << 1);

        *ptr_data = x2r;
        *(ptr_data + 1) = x2i;
        ptr_data += (del << 1);

        *ptr_data = x1r;
        *(ptr_data + 1) = x1i;
        ptr_data += (del << 1);

        *ptr_data = x3i;
        *(ptr_data + 1) = x3r;
        ptr_data += (del << 1);
      }
      ptr_data -= 2 * npoints;
      ptr_data += 2;
    }
    for (; j < nodespacing * del; j += nodespacing)
    {
      W1 = *(ptr_twiddles + j);
      W4 = *(ptr_twiddles + j + 257);
      W2 = *(ptr_twiddles + (j << 1) - 256);
      W5 = *(ptr_twiddles + (j << 1) + 1);
      W3 = *(ptr_twiddles + j + (j << 1) - 512);
      W6 = *(ptr_twiddles + j + (j << 1) - 512 + 257);

      for (k = in_loop_cnt; k != 0; k--)
      {
        FLOAT32 tmp;
        FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

        ptr_data += (del << 1);

        x1r = *ptr_data;
        x1i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x2r = *ptr_data;
        x2i = *(ptr_data + 1);
        ptr_data += (del << 1);

        x3r = *ptr_data;
        x3i = *(ptr_data + 1);
        ptr_data -= 3 * (del << 1);

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4));
        x1i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x1r, W4), (FLOAT64)x1i, W1);
        x1r = tmp;

        tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x2r, W5) + impeghe_dmult((FLOAT64)x2i, W2));
        x2i = (FLOAT32)(-impeghe_dmult((FLOAT64)x2r, W2) + impeghe_dmult((FLOAT64)x2i, W5));
        x2r = tmp;

        tmp = (FLOAT32)(-impeghe_dmult((FLOAT64)x3r, W3) + impeghe_dmult((FLOAT64)x3i, W6));
        x3i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x3r, W6), (FLOAT64)x3i, W3);
        x3r = tmp;

        x0r = (*ptr_data);
        x0i = (*(ptr_data + 1));

        x0r = x0r + (x2r);
        x0i = x0i + (x2i);
        x2r = x0r - (x2r * 2);
        x2i = x0i - (x2i * 2);
        x1r = x1r + x3r;
        x1i = x1i - x3i;
        x3r = x1r - (x3r * 2);
        x3i = x1i + (x3i * 2);

        x0r = x0r + (x1r);
        x0i = x0i + (x1i);
        x1r = x0r - (x1r * 2);
        x1i = x0i - (x1i * 2);
        x2r = x2r + (x3i);
        x2i = x2i - (x3r);
        x3i = x2r - (x3i * 2);
        x3r = x2i + (x3r * 2);

        *ptr_data = x0r;
        *(ptr_data + 1) = x0i;
        ptr_data += (del << 1);

        *ptr_data = x2r;
        *(ptr_data + 1) = x2i;
        ptr_data += (del << 1);

        *ptr_data = x1r;
        *(ptr_data + 1) = x1i;
        ptr_data += (del << 1);

        *ptr_data = x3i;
        *(ptr_data + 1) = x3r;
        ptr_data += (del << 1);
      }
      ptr_data -= 2 * npoints;
      ptr_data += 2;
    }
    nodespacing >>= 2;
    del <<= 2;
    in_loop_cnt >>= 2;
  }
  if (not_power_4)
  {
    const FLOAT64 *ptr_twiddles = ptr_w;
    nodespacing <<= 1;

    for (j = del / 2; j != 0; j--)
    {
      FLOAT64 W1 = *ptr_twiddles;
      FLOAT64 W4 = *(ptr_twiddles + 257);
      FLOAT32 tmp;
      ptr_twiddles += nodespacing;

      x0r = *ptr_y2;
      x0i = *(ptr_y2 + 1);
      ptr_y2 += (del << 1);

      x1r = *ptr_y2;
      x1i = *(ptr_y2 + 1);

      tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4));
      x1i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x1r, W4), (FLOAT64)x1i, W1);
      x1r = tmp;

      *ptr_y2 = (x0r) - (x1r);
      *(ptr_y2 + 1) = (x0i) - (x1i);
      ptr_y2 -= (del << 1);

      *ptr_y2 = (x0r) + (x1r);
      *(ptr_y2 + 1) = (x0i) + (x1i);
      ptr_y2 += 2;
    }
    ptr_twiddles = ptr_w;
    for (j = del / 2; j != 0; j--)
    {
      FLOAT64 W1 = *ptr_twiddles;
      FLOAT64 W4 = *(ptr_twiddles + 257);
      FLOAT32 tmp;
      ptr_twiddles += nodespacing;

      x0r = *ptr_y2;
      x0i = *(ptr_y2 + 1);
      ptr_y2 += (del << 1);

      x1r = *ptr_y2;
      x1i = *(ptr_y2 + 1);

      tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W4) + impeghe_dmult((FLOAT64)x1i, W1)) /*/2*/;
      x1i = (FLOAT32)(-impeghe_dmult((FLOAT64)x1r, W1) + impeghe_dmult((FLOAT64)x1i, W4)) /*/2*/;
      x1r = tmp;

      *ptr_y2 = (x0r) - (x1r);
      *(ptr_y2 + 1) = (x0i) - (x1i);
      ptr_y2 -= (del << 1);

      *ptr_y2 = (x0r) + (x1r);
      *(ptr_y2 + 1) = (x0i) + (x1i);
      ptr_y2 += 2;
    }
  }

  for (i = 0; i < nlength; i++)
  {
    *(ptr_x + 2 * i) = ptr_y[2 * i];
    *(ptr_x + 2 * i + 1) = ptr_y[2 * i + 1];
  }
  return;
}

/**
 *  impeghe_complex_fft_p3
 *
 *  \brief Radix 3 complex n-point FFT
 *
 *  \param [in,out] ptr_data		 Pointer to data buffer
 *  \param [in]     nlength		 n-point for FFT
 *  \param [in]     pstr_scratch Pointer to scratch buffer
 *
 *  \return VOID
 */
static VOID impeghe_complex_fft_p3(FLOAT32 *ptr_data, WORD32 nlength,
                                   impeghe_scratch_mem *pstr_scratch)
{
  WORD32 i, j;
  FLOAT32 *ptr_data_3 = pstr_scratch->p_fft_p3_data_3;
  FLOAT32 *ptr_y = pstr_scratch->p_fft_p3_y;
  WORD32 cnfac;
  WORD32 mpass = nlength;
  FLOAT32 *ptr_x = ptr_data;
  FLOAT32 *ptr_y2 = ptr_y;

  cnfac = 0;
  while (mpass % 3 == 0)
  {
    mpass /= 3;
    cnfac++;
  }

  for (i = 0; i < 3 * cnfac; i++)
  {
    for (j = 0; j < mpass; j++)
    {
      ptr_data_3[2 * j] = ptr_data[3 * (2 * j) + (2 * i)];
      ptr_data_3[2 * j + 1] = ptr_data[3 * (2 * j) + 1 + (2 * i)];
    }
    impeghe_complex_fft_p2(ptr_data_3, mpass, pstr_scratch->p_fft_p2_y);

    for (j = 0; j < mpass; j++)
    {
      ptr_data[3 * (2 * j) + (2 * i)] = ptr_data_3[2 * j];
      ptr_data[3 * (2 * j) + 1 + (2 * i)] = ptr_data_3[2 * j + 1];
    }
  }

  {
    const FLOAT64 *ptr_w1r, *ptr_w1i;
    FLOAT32 tmp;
    ptr_w1r = impeghe_twiddle_table_3pr;
    ptr_w1i = impeghe_twiddle_table_3pi;

    for (i = 0; i < nlength; i += 3)
    {
      tmp = (FLOAT32)((FLOAT64)ptr_data[2 * i] * (*ptr_w1r) -
                      (FLOAT64)ptr_data[2 * i + 1] * (*ptr_w1i));
      ptr_data[2 * i + 1] = (FLOAT32)((FLOAT64)ptr_data[2 * i] * (*ptr_w1i) +
                                      (FLOAT64)ptr_data[2 * i + 1] * (*ptr_w1r));
      ptr_data[2 * i] = tmp;

      ptr_w1r++;
      ptr_w1i++;

      tmp = (FLOAT32)((FLOAT64)ptr_data[2 * (i + 1)] * (*ptr_w1r) -
                      (FLOAT64)ptr_data[2 * (i + 1) + 1] * (*ptr_w1i));
      ptr_data[2 * (i + 1) + 1] = (FLOAT32)((FLOAT64)ptr_data[2 * (i + 1)] * (*ptr_w1i) +
                                            (FLOAT64)ptr_data[2 * (i + 1) + 1] * (*ptr_w1r));
      ptr_data[2 * (i + 1)] = tmp;

      ptr_w1r++;
      ptr_w1i++;

      tmp = (FLOAT32)((FLOAT64)ptr_data[2 * (i + 2)] * (*ptr_w1r) -
                      (FLOAT64)ptr_data[2 * (i + 2) + 1] * (*ptr_w1i));
      ptr_data[2 * (i + 2) + 1] = (FLOAT32)((FLOAT64)ptr_data[2 * (i + 2)] * (*ptr_w1i) +
                                            (FLOAT64)ptr_data[2 * (i + 2) + 1] * (*ptr_w1r));
      ptr_data[2 * (i + 2)] = tmp;

      ptr_w1r += 3 * (128 / mpass - 1) + 1;
      ptr_w1i += 3 * (128 / mpass - 1) + 1;
    }
  }

  for (i = 0; i < mpass; i++)
  {
    impeghe_complex_3point_fft(ptr_x, ptr_y2);

    ptr_x = ptr_x + 6;
    ptr_y2 = ptr_y2 + 6;
  }

  for (i = 0; i < mpass; i++)
  {
    ptr_data[2 * i] = ptr_y[6 * i];
    ptr_data[2 * i + 1] = ptr_y[6 * i + 1];
  }

  for (i = 0; i < mpass; i++)
  {
    ptr_data[2 * (i + mpass)] = ptr_y[6 * i + 2];
    ptr_data[2 * (i + mpass) + 1] = ptr_y[6 * i + 3];
  }

  for (i = 0; i < mpass; i++)
  {
    ptr_data[2 * (i + 2 * mpass)] = ptr_y[6 * i + 4];
    ptr_data[2 * (i + 2 * mpass) + 1] = ptr_y[6 * i + 5];
  }
  return;
}

/**
 *  impeghe_calc_pre_twid_enc
 *
 *  \brief Calculate pre-twiddle data
 *
 *  \param [in,out] ptr_in					Pointer to data buffer
 *  \param [out] 	ptr_fft					Pointer to FFT data buffer
 *  \param [in] 	npoints					n-point for FFT
 *  \param [in] 	ptr_cos					Pointer to twiddle cos table
 *  \param [in] 	ptr_sin					Pointer to twiddle sine table
 *  \param [in] 	transform_kernel_type	Transform kernel type
 *
 *  \return VOID
 */
static VOID impeghe_calc_pre_twid_enc(FLOAT64 *ptr_in, FLOAT32 *ptr_fft, WORD32 npoints,
                                      const FLOAT64 *ptr_cos, const FLOAT64 *ptr_sin,
                                      WORD32 transform_kernel_type)
{
  WORD32 i, n;
  WORD32 b = npoints >> 1;
  WORD32 a = npoints - b;
  WORD32 nlength = npoints >> 2;
  FLOAT64 tempr, tempi;

  if (transform_kernel_type == 3)
  {
    FLOAT64 norm;
    for (i = 0; i < b; i++)
    {
      norm = ptr_in[i]; /* reuse MDCT: spectrally reverse all bins */
      ptr_in[i] = ptr_in[npoints - 1 - i];
      ptr_in[npoints - 1 - i] = norm;
    }
  }
  for (i = 0; i < nlength; i++)
  {

    n = npoints / 2 - 1 - 2 * i;
    if (i < b / 4)
    {
      tempr = ptr_in[a / 2 + n] + ptr_in[npoints + a / 2 - 1 - n];
    }
    else
    {
      tempr = ptr_in[a / 2 + n] - ptr_in[a / 2 - 1 - n];
    }
    n = 2 * i;
    if (i < a / 4)
    {
      tempi = ptr_in[a / 2 + n] - ptr_in[a / 2 - 1 - n];
    }
    else
    {
      tempi = ptr_in[a / 2 + n] + ptr_in[npoints + a / 2 - 1 - n];
    }

    ptr_fft[2 * i] = (FLOAT32)(tempr * (*ptr_cos) + tempi * (*ptr_sin));
    ptr_fft[2 * i + 1] = (FLOAT32)(tempi * (*ptr_cos++) - tempr * (*ptr_sin++));
  }
  return;
}

/**
 *  impeghe_complex_fft
 *
 *  \brief Complex FFT
 *
 *  \param [in,out] ptr_data			Pointer to data buffer
 *  \param [in] 	nlength			n-point for FFT
 *  \param [in] 	pstr_scratch	Pointer to scratch buffer
 *
 *  \return VOID
 */
VOID impeghe_complex_fft(FLOAT32 *ptr_data, WORD32 nlength, impeghe_scratch_mem *pstr_scratch)
{
  if (nlength & (nlength - 1))
  {
    impeghe_complex_fft_p3(ptr_data, nlength, pstr_scratch);
  }
  else
  {
    impeghe_complex_fft_p2(ptr_data, nlength, pstr_scratch->p_fft_p2_y);
  }
  return;
}

/**
 *  impeghe_calc_post_twid_enc
 *
 *  \brief Calculate post twiddle data
 *
 *  \param [out] ptr_out				Pointer to output data buffer
 *  \param [in]  ptr_fft				Pointer to FFT data buffer
 *  \param [in]  npoints				n-point for FFT
 *  \param [in]  ptr_cos				Pointer to twiddle cos table
 *  \param [in]  ptr_sin				Pointer to twiddle sine table
 *  \param [in]  transform_kernel_type	Transform kernel type
 *
 *  \return VOID
 */
static VOID impeghe_calc_post_twid_enc(FLOAT64 *ptr_out, FLOAT32 *ptr_fft, WORD32 npoints,
                                       const FLOAT64 *ptr_cos, const FLOAT64 *ptr_sin,
                                       WORD32 transform_kernel_type)
{
  WORD32 i;
  WORD32 nlength = npoints >> 2;
  FLOAT64 tempr, tempi;

  /* post-twiddle FFT output and then get output data */
  for (i = 0; i < nlength; i++)
  {

    tempr =
        2 * ((FLOAT64)(ptr_fft[2 * i]) * (*ptr_cos) + (FLOAT64)(ptr_fft[2 * i + 1]) * (*ptr_sin));
    tempi = 2 * ((FLOAT64)(ptr_fft[2 * i + 1]) * (*ptr_cos++) -
                 (FLOAT64)(ptr_fft[2 * i]) * (*ptr_sin++));

    ptr_out[2 * i] = -tempr;
    ptr_out[npoints / 2 - 1 - 2 * i] = tempi;
    ptr_out[npoints / 2 + 2 * i] = -tempi;
    ptr_out[npoints - 1 - 2 * i] = tempr;
  }
  if (transform_kernel_type == 3)
  {
    for (i = 0; i < npoints; i += 2)
    {
      ptr_out[i] *= -1; /* reuse MDCT: flip signs at odd indices */
    }
  }
  return;
}

/**
 *  impeghe_fft_based_mdct
 *
 *  \brief Computes FFT based MDCT
 *
 *  \param [in,out] ptr_in					Pointer to data buffer
 *  \param [out] 	ptr_out					Pointer to output buffer
 *  \param [in] 	npoints					n-point for FFT
 *  \param [in] 	transform_kernel_type	Transform kernel type
 *  \param [in] 	pstr_scratch			Pointer to scratch buffer
 *
 *  \return IA_ERRORCODE	Error code
 */
IA_ERRORCODE impeghe_fft_based_mdct(FLOAT64 *ptr_in, FLOAT64 *ptr_out, WORD32 npoints,
                                    WORD32 transform_kernel_type,
                                    impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *ptr_scratch = pstr_scratch->p_fft_mdct_buf;
  FLOAT32 *ptr_scratch1 = (FLOAT32 *)ptr_scratch;
  const FLOAT64 *ptr_cos = NULL;
  const FLOAT64 *ptr_sin = NULL;
  WORD32 nlength = npoints >> 1;
  WORD32 n_total = npoints << 1;
  FLOAT32 *ptr_scratch2 = &ptr_scratch1[n_total];

  memset(ptr_scratch, 0, (n_total << 1) * sizeof(*ptr_scratch));

  if (transform_kernel_type == 2 || transform_kernel_type == 1)
  {
    switch (npoints)
    {
    case (FRAME_LEN_LONG):
      ptr_cos = impeghe_pre_twid_type2_dct_dst_cos_2048;
      break;
    case (FRAME_LEN_SHORT):
      ptr_cos = impeghe_pre_twid_type2_dct_dst_cos_256;
      break;
    default:
      return -1;
    }
  }
  else
  {
    switch (npoints)
    {
    case (FRAME_LEN_SHORT):
      ptr_cos = impeghe_pre_post_twid_cos_256;
      ptr_sin = impeghe_pre_post_twid_sin_256;
      break;
    case (FRAME_LEN_LONG):
      ptr_cos = impeghe_pre_post_twid_cos_2048;
      ptr_sin = impeghe_pre_post_twid_sin_2048;
      break;
    default:
      return -1;
    }
  }
  if (transform_kernel_type == 3 || transform_kernel_type == 0)
  {
    /* pre-twiddle */
    impeghe_calc_pre_twid_enc(ptr_in, ptr_scratch1, npoints << 1, ptr_cos, ptr_sin,
                              transform_kernel_type);

    /* complex FFT */
    impeghe_complex_fft(ptr_scratch1, nlength, pstr_scratch);

    /* post-twiddle */
    impeghe_calc_post_twid_enc(ptr_out, ptr_scratch1, npoints << 1, ptr_cos, ptr_sin,
                               transform_kernel_type);
  }
  else
  {
    WORD32 i;
    if (npoints == FRAME_LEN_LONG)
    {
      const FLOAT64 *ptr_cos_2048 = impeghe_twiddle_cos_2048;
      const FLOAT64 *ptr_sin_2048 = impeghe_twiddle_sin_2048;

      // To reuse DCT-II for DST-II, flip signs of alternate inputs
      if (transform_kernel_type == 1)
      {
        for (i = 0; i < npoints; i++)
        {
          ptr_in[2 * i + 1] = -ptr_in[2 * i + 1];
        }
      }
      for (i = 0; i < npoints; i++)
      {
        ptr_scratch1[2 * i] = (FLOAT32)ptr_in[2 * i];
      }

      /* Complex FFT of N/2 */
      impeghe_complex_fft(ptr_scratch1, npoints, pstr_scratch);

      for (i = 0; i < npoints; i++)
      {
        ptr_scratch2[2 * i] = (FLOAT32)ptr_in[2 * i + 1];
      }

      /* Complex FFT of N/2 */
      impeghe_complex_fft(ptr_scratch2, npoints, pstr_scratch);

      /* get 2048 point FFT output  */
      for (i = 0; i < npoints; i++)
      {
        ptr_out[2 * i] =
            ptr_scratch1[2 * i] + ((FLOAT64)(ptr_scratch2[2 * i]) * ptr_cos_2048[i] +
                                   (FLOAT64)(ptr_scratch2[2 * i + 1]) * (ptr_sin_2048[i]));

        ptr_out[2 * i + 1] =
            ptr_scratch1[2 * i + 1] + ((FLOAT64)(-ptr_scratch2[2 * i]) * (ptr_sin_2048[i]) +
                                       (FLOAT64)(ptr_scratch2[2 * i + 1]) * ptr_cos_2048[i]);
      }

      /* get post-twiddled FFT output  */
      for (i = 0; i < npoints; i++)
      {
        ptr_out[i] = 2 * ((FLOAT64)(ptr_out[2 * i]) * ptr_cos[i] +
                          (FLOAT64)(ptr_out[2 * i + 1]) * (ptr_cos[npoints - i]));
      }
    }
    else
    {
      for (i = 0; i < n_total; i++)
      {
        ptr_scratch1[2 * i] = (FLOAT32)ptr_in[i];
        ptr_out[i] = 0;
      }
      impeghe_complex_fft(ptr_scratch1, n_total, pstr_scratch);

      /* get post-twiddled FFT output  */
      for (i = 0; i < npoints; i++)
      {
        ptr_out[i] = 2 * ((FLOAT64)(ptr_scratch1[2 * i]) * ptr_cos[i] +
                          (FLOAT64)(ptr_scratch1[2 * i + 1]) * (ptr_cos[npoints - i]));
      }
    }

    // To reuse DCT-II for DST-II, reverse order of the outputs
    if (transform_kernel_type == 1)
    {
      for (i = 0; i<npoints>> 1; i++)
      {
        double temp = ptr_out[i];
        ptr_out[i] = ptr_out[npoints - i - 1];
        ptr_out[npoints - i - 1] = temp;
      }
    }
  }

  return 0;
}

/**
 *  impeghe_fft_7
 *
 *  \brief 7 point FFT
 *
 *  \param [in]  ptr_inp    Pointer to input data
 *  \param [out] ptr_out     Pointer to output data
 *
 *  \return VOID
 *
 */
static PLATFORM_INLINE VOID impeghe_fft_7(FLOAT32 *ptr_inp, FLOAT32 *ptr_out)
{
  FLOAT32 x0r, x1r, x2r, x3r, x4r, x5r, x6r, x7r, x8r;
  FLOAT32 x0i, x1i, x2i, x3i, x4i, x5i, x6i, x7i, x8i;
  FLOAT32 y0r, y1r, y2r, y3r, y4r, y5r, y6r, y7r, y8r;
  FLOAT32 y0i, y1i, y2i, y3i, y4i, y5i, y6i, y7i, y8i;

  /*
   * Node 1 of Winograd FFT for 7 point
   *
   * 1   0   0   0   0   0   0
   * 0   1   0   0   0   0   1
   * 0   1   0   0   0   0  -1
   * 0   0   1   0   0   1   0
   * 0   0   1   0   0  -1   0
   * 0   0   0   1   1   0   0
   * 0   0   0  -1   1   0   0
   *
   */

  x0r = ptr_inp[0];
  x0i = ptr_inp[1];
  x1r = ia_add_flt(ptr_inp[2], ptr_inp[12]);
  x1i = ia_add_flt(ptr_inp[3], ptr_inp[13]);
  x2r = ia_sub_flt(ptr_inp[2], ptr_inp[12]);
  x2i = ia_sub_flt(ptr_inp[3], ptr_inp[13]);
  x3r = ia_add_flt(ptr_inp[4], ptr_inp[10]);
  x3i = ia_add_flt(ptr_inp[5], ptr_inp[11]);
  x4r = ia_sub_flt(ptr_inp[4], ptr_inp[10]);
  x4i = ia_sub_flt(ptr_inp[5], ptr_inp[11]);
  x5r = ia_add_flt(ptr_inp[8], ptr_inp[6]);
  x5i = ia_add_flt(ptr_inp[9], ptr_inp[7]);
  x6r = ia_sub_flt(ptr_inp[8], ptr_inp[6]);
  x6i = ia_sub_flt(ptr_inp[9], ptr_inp[7]);

  /*
   * Node 2 of Winograd FFT for 7 point
   *
   * 1   0   0   0   0   0   0
   * 0   1   0   1   0   1   0
   * 0   1   0  -1   0   0   0
   * 0  -1   0   0   0   1   0
   * 0   0   0   1   0  -1   0
   * 0   0   1   0   1   0   1
   * 0   0   1   0  -1   0   0
   * 0   0  -1   0   0   0   1
   * 0   0   0   0   1   0  -1
   *
   */

  y0r = x0r;
  y0i = x0i;
  y1r = ia_add_flt(ia_add_flt(x1r, x3r), x5r);
  y1i = ia_add_flt(ia_add_flt(x1i, x3i), x5i);
  y2r = ia_sub_flt(x1r, x3r);
  y2i = ia_sub_flt(x1i, x3i);
  y3r = ia_sub_flt(x5r, x1r);
  y3i = ia_sub_flt(x5i, x1i);
  y4r = ia_sub_flt(x3r, x5r);
  y4i = ia_sub_flt(x3i, x5i);
  y5r = ia_add_flt(ia_add_flt(x2r, x4r), x6r);
  y5i = ia_add_flt(ia_add_flt(x2i, x4i), x6i);
  y6r = ia_sub_flt(x2r, x4r);
  y6i = ia_sub_flt(x2i, x4i);
  y7r = ia_sub_flt(x6r, x2r);
  y7i = ia_sub_flt(x6i, x2i);
  y8r = ia_sub_flt(x4r, x6r);
  y8i = ia_sub_flt(x4i, x6i);

  /*
   * Node 3 of Winograd FFT for 7 point
   *
   * 1    1    0    0    0     0     0     0     0
   * 1  c70    0    0    0     0     0     0     0
   * 0    0  c71    0    0     0     0     0     0
   * 0    0    0  c72    0     0     0     0     0
   * 0    0    0    0  c73     0     0     0     0
   * 0    0    0    0    0  jc74     0     0     0
   * 0    0    0    0    0     0  jc75     0     0
   * 0    0    0    0    0     0     0  jc76     0
   * 0    0    0    0    0     0     0     0  jc77
   *
   */
  x0r = ia_add_flt(y0r, y1r);
  x0i = ia_add_flt(y0i, y1i);
  x1r = ia_mac_flt(y0r, C70, y1r);
  x1i = ia_mac_flt(y0i, C70, y1i);
  x2r = ia_mul_flt(C71, y2r);
  x2i = ia_mul_flt(C71, y2i);
  x3r = ia_mul_flt(C72, y3r);
  x3i = ia_mul_flt(C72, y3i);
  x4r = ia_mul_flt(C73, y4r);
  x4i = ia_mul_flt(C73, y4i);
  x5r = ia_mul_flt(-C74, y5i);
  x5i = ia_mul_flt(C74, y5r);
  x6r = ia_mul_flt(-C75, y6i);
  x6i = ia_mul_flt(C75, y6r);
  x7r = ia_mul_flt(-C76, y7i);
  x7i = ia_mul_flt(C76, y7r);
  x8r = ia_mul_flt(-C77, y8i);
  x8i = ia_mul_flt(C77, y8r);

  /*
   * Node 4 of Winograd FFT for 7 point
   *
   * 1   0   0   0   0   0   0   0   0
   * 0   1   1   0   1   0   0   0   0
   * 0   1  -1  -1   0   0   0   0   0
   * 0   1   0   1  -1   0   0   0   0
   * 0   0   0   0   0   1   1   0   1
   * 0   0   0   0   0   1  -1  -1   0
   * 0   0   0   0   0   1   0   1  -1
   *
   */

  y0r = x0r;
  y0i = x0i;
  y1r = ia_add_flt(ia_add_flt(x1r, x2r), x4r);
  y1i = ia_add_flt(ia_add_flt(x1i, x2i), x4i);
  y2r = ia_sub_flt(ia_sub_flt(x1r, x2r), x3r);
  y2i = ia_sub_flt(ia_sub_flt(x1i, x2i), x3i);
  y3r = ia_sub_flt(ia_add_flt(x1r, x3r), x4r);
  y3i = ia_sub_flt(ia_add_flt(x1i, x3i), x4i);
  y4r = ia_add_flt(ia_add_flt(x5r, x6r), x8r);
  y4i = ia_add_flt(ia_add_flt(x5i, x6i), x8i);
  y5r = ia_sub_flt(ia_sub_flt(x5r, x6r), x7r);
  y5i = ia_sub_flt(ia_sub_flt(x5i, x6i), x7i);
  y6r = ia_sub_flt(ia_add_flt(x5r, x7r), x8r);
  y6i = ia_sub_flt(ia_add_flt(x5i, x7i), x8i);

  /*
   * Node 5 of Winograd FFT for 7 point
   *
   * 1   0   0   0   0   0   0
   * 0   1   0   0   1   0   0
   * 0   0   0   1   0   0   1
   * 0   0   1   0   0  -1   0
   * 0   0   1   0   0   1   0
   * 0   0   0   1   0   0  -1
   * 0   1   0   0  -1   0   0
   *
   */
  x0r = y0r;
  x0i = y0i;
  x1r = ia_add_flt(y1r, y4r);
  x1i = ia_add_flt(y1i, y4i);
  x2r = ia_add_flt(y3r, y6r);
  x2i = ia_add_flt(y3i, y6i);
  x3r = ia_sub_flt(y2r, y5r);
  x3i = ia_sub_flt(y2i, y5i);
  x4r = ia_add_flt(y2r, y5r);
  x4i = ia_add_flt(y2i, y5i);
  x5r = ia_sub_flt(y3r, y6r);
  x5i = ia_sub_flt(y3i, y6i);
  x6r = ia_sub_flt(y1r, y4r);
  x6i = ia_sub_flt(y1i, y4i);

  ptr_out[0] = x0r;
  ptr_out[1] = x0i;
  ptr_out[2] = x1r;
  ptr_out[3] = x1i;
  ptr_out[4] = x2r;
  ptr_out[5] = x2i;
  ptr_out[6] = x3r;
  ptr_out[7] = x3i;
  ptr_out[8] = x4r;
  ptr_out[9] = x4i;
  ptr_out[10] = x5r;
  ptr_out[11] = x5i;
  ptr_out[12] = x6r;
  ptr_out[13] = x6i;

  return;
}

/**
 *  impeghe_mix_rad_fft_3nx3
 *
 *  \brief Mixed radix FFT 3nx3
 *
 *  \param [in,out] ptr_real    Pointer to real part data buffer
 *  \param [in,out] ptr_imag    Pointer to imaginary part data buffer
 *  \param [in]     n_points    N-point for FFT
 *  \param [in]     ptr_scratch Pointer to scratch buffer
 *
 *  \return VOID
 *
 */
static VOID impeghe_mix_rad_fft_3nx3(FLOAT32 *ptr_real, FLOAT32 *ptr_imag, WORD32 n_points,
                                     FLOAT32 *ptr_scratch)
{
  WORD32 i, j;
  WORD32 cnfac;
  WORD32 m_points = n_points;
  FLOAT32 *ptr_x, *ptr_y, *ptr_temp;
  FLOAT32 *ptr_x2, *ptr_y2;
  ptr_x2 = ptr_x = ptr_scratch;
  ptr_scratch += 2 * n_points;
  ptr_y2 = ptr_y = ptr_scratch;
  ptr_scratch += 2 * n_points;
  ptr_temp = ptr_scratch;
  ptr_scratch += 2 * n_points;

  cnfac = 0;
  while (m_points % 3 == 0)
  {
    m_points /= 3;
    cnfac++;
  }

  for (i = 0; i < 3 * cnfac; i++)
  {
    for (j = 0; j < m_points; j++)
    {
      ptr_temp[2 * j + 0] = ptr_real[3 * j + i];
      ptr_temp[2 * j + 1] = ptr_imag[3 * j + i];
    }

    impeghe_complex_fft_p2(ptr_temp, m_points, ptr_scratch);

    for (j = 0; j < m_points; j++)
    {
      ptr_real[3 * j + i] = ptr_temp[2 * j + 0];
      ptr_imag[3 * j + i] = ptr_temp[2 * j + 1];
    }
  }

  {
    FLOAT32 *ptr_w1r, *ptr_w1i;
    FLOAT32 tmp;
    ptr_w1r = (FLOAT32 *)&impeghe_rad_3_fft_twiddle_re[0];
    ptr_w1i = (FLOAT32 *)&impeghe_rad_3_fft_twiddle_im[0];

    for (i = 0; i < n_points; i += 3)
    {
      tmp = ia_msu_flt(ia_mul_flt(ptr_real[i], (*ptr_w1r)), ptr_imag[i], (*ptr_w1i));
      ptr_imag[i] = ia_mac_flt(ia_mul_flt(ptr_real[i], (*ptr_w1i)), ptr_imag[i], (*ptr_w1r));
      ptr_real[i] = tmp;

      ptr_w1r++;
      ptr_w1i++;

      tmp = ia_msu_flt(ia_mul_flt(ptr_real[i + 1], (*ptr_w1r)), ptr_imag[i + 1], (*ptr_w1i));
      ptr_imag[i + 1] =
          ia_mac_flt(ia_mul_flt(ptr_real[i + 1], (*ptr_w1i)), ptr_imag[i + 1], (*ptr_w1r));
      ptr_real[i + 1] = tmp;

      ptr_w1r++;
      ptr_w1i++;

      tmp = ia_msu_flt(ia_mul_flt(ptr_real[i + 2], (*ptr_w1r)), ptr_imag[i + 2], (*ptr_w1i));
      ptr_imag[i + 2] =
          ia_mac_flt(ia_mul_flt(ptr_real[i + 2], (*ptr_w1i)), ptr_imag[i + 2], (*ptr_w1r));
      ptr_real[i + 2] = tmp;

      ptr_w1r += 3 * (128 / m_points - 1) + 1;
      ptr_w1i += 3 * (128 / m_points - 1) + 1;
    }
  }

  for (i = 0; i < n_points; i++)
  {
    ptr_x2[2 * i] = ptr_real[i];
    ptr_x2[2 * i + 1] = ptr_imag[i];
  }

  for (i = 0; i < m_points; i++)
  {
    impeghe_complex_3point_fft(ptr_x2, ptr_y2);

    ptr_x2 = ptr_x2 + 6;
    ptr_y2 = ptr_y2 + 6;
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[i] = ptr_y[6 * i];
    ptr_imag[i] = ptr_y[6 * i + 1];
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[m_points + i] = ptr_y[6 * i + 2];
    ptr_imag[m_points + i] = ptr_y[6 * i + 3];
  }

  for (i = 0; i < m_points; i++)
  {
    ptr_real[2 * m_points + i] = ptr_y[6 * i + 4];
    ptr_imag[2 * m_points + i] = ptr_y[6 * i + 5];
  }
  return;
}

/**
 *  impeghe_mix_rad_fft_tw_mult
 *
 *  \brief Mixed radix FFT and Twiddle multiply
 *
 *  \param [in]  ptr_inp    Pointer to input data
 *  \param [out] ptr_out     Pointer to output data
 *  \param [in]  dim1   Dimension 1
 *  \param [in]  dim2   Dimension 2
 *  \param [in]  ptr_tw     Pointer to twiddle data
 *
 *  \return VOID
 *
 */
static VOID impeghe_mix_rad_fft_tw_mult(FLOAT32 *ptr_inp, FLOAT32 *ptr_out, WORD32 dim1,
                                        WORD32 dim2, const FLOAT32 *ptr_tw)
{
  FLOAT32 accu1, accu2;
  WORD32 i, j;
  WORD32 step_val = (dim2 - 1) << 1;
  for (i = 0; i < (dim2); i++)
  {
    ptr_out[0] = ptr_inp[0];
    ptr_out[1] = ptr_inp[1];
    ptr_out += 2;
    ptr_inp += 2;
  }

  for (j = 0; j < (dim1 - 1); j++)
  {
    ptr_out[0] = ptr_inp[0];
    ptr_out[1] = ptr_inp[1];
    ptr_inp += 2;
    ptr_out += 2;
    for (i = 0; i < (dim2 - 1); i++)
    {
      CPLX_MPY_FFT(accu1, accu2, ptr_inp[2 * i + 0], ptr_inp[2 * i + 1], ptr_tw[2 * i + 1],
                   ptr_tw[2 * i]);
      ptr_out[2 * i + 0] = accu1;
      ptr_out[2 * i + 1] = accu2;
    }
    ptr_inp += step_val;
    ptr_out += step_val;
    ptr_tw += (dim2 - 1) * 2;
  }
  return;
}

/**
 *  impeghe_mix_rad_fft_3nx7
 *
 *  \brief Mixed radix FFT 3nx7
 *
 *  \param [in,out] ptr_inp         Pointer to input data
 *  \param [in]     len         Input length
 *  \param [in]     ptr_scratch Pointer to scratch buffer
 *
 *  \return VOID
 *
 */
static VOID impeghe_mix_rad_fft_3nx7(FLOAT32 *ptr_inp, WORD32 len, FLOAT32 *ptr_scratch)
{
  WORD32 i, j;
  WORD32 m_points = len / 7;
  FLOAT32 *ptr_real, *ptr_imag, *ptr_real_1, *ptr_imag_1, *ptr_scratch_1;
  ptr_real = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  ptr_imag = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  ptr_scratch_1 = ptr_real_1 = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;
  ptr_imag_1 = ptr_scratch;
  ptr_scratch += SLPD_MAX_FFT_SIZE;

  for (i = 0; i < len; i++)
  {
    ptr_real[i] = ptr_inp[2 * i];
    ptr_imag[i] = ptr_inp[2 * i + 1];
  }

  for (i = 0; i < m_points; i++)
  {
    for (j = 0; j < 7; j++)
    {
      ptr_real_1[2 * j + 0] = ptr_inp[m_points * 2 * j + 2 * i];
      ptr_real_1[2 * j + 1] = ptr_inp[m_points * 2 * j + 2 * i + 1];
    }

    impeghe_fft_7(ptr_real_1, ptr_scratch);

    for (j = 0; j < 7; j++)
    {
      ptr_inp[m_points * 2 * j + 2 * i + 0] = ptr_scratch[2 * j + 0];
      ptr_inp[m_points * 2 * j + 2 * i + 1] = ptr_scratch[2 * j + 1];
    }
  }

  if (m_points == 48)
    impeghe_mix_rad_fft_tw_mult(ptr_inp, ptr_scratch_1, 7, m_points,
                                impeghe_fft_mix_rad_twid_tbl_336);
  else
    impeghe_mix_rad_fft_tw_mult(ptr_inp, ptr_scratch_1, 7, m_points,
                                impeghe_fft_mix_rad_twid_tbl_168);

  for (i = 0; i < len; i++)
  {
    ptr_real[i] = ptr_scratch_1[2 * i];
    ptr_imag[i] = ptr_scratch_1[2 * i + 1];
  }

  for (i = 0; i < 7; i++)
  {
    impeghe_mix_rad_fft_3nx3(ptr_real, ptr_imag, m_points, ptr_scratch);
    ptr_real += (m_points);
    ptr_imag += (m_points);
  }
  ptr_real -= 7 * m_points;
  ptr_imag -= 7 * m_points;
  for (j = 0; j < 7; j++)
  {
    for (i = 0; i < m_points; i++)
    {
      ptr_real_1[7 * i + j] = ptr_real[m_points * j + i];
      ptr_imag_1[7 * i + j] = ptr_imag[m_points * j + i];
    }
  }

  for (i = 0; i < len; i++)
  {
    ptr_inp[2 * i] = ptr_real_1[i];
    ptr_inp[2 * i + 1] = ptr_imag_1[i];
  }

  return;
}

/**
 *  impeghe_slpd_fft
 *
 *  \brief Stereo LPD module's FFT
 *
 *  \param [in,out] ptr_fft_data    Pointer to FFT data
 *  \param [in]     ptr_sin   Pointer to Sine table
 *  \param [in]     len         Input length
 *  \param [in]     ptr_scratch Pointer to scratch buffer
 *
 *  \return VOID
 *
 */
VOID impeghe_slpd_fft(FLOAT32 *ptr_fft_data, const FLOAT32 *ptr_sin, WORD32 len,
                      impeghe_scratch_mem *ptr_scratch)
{
  WORD32 i;
  FLOAT32 tmp1, tmp2, tmp3, tmp4, s, c;
  FLOAT32 *ptr_scratch_flt = &ptr_scratch->mixed_rad_fft[0];
  ptr_scratch_flt = ptr_scratch_flt + 2 * SLPD_MAX_FFT_SIZE;

  /* Check that input vector has an even length */
  if (len == 672 || len == 336)
  {
    impeghe_mix_rad_fft_3nx7(ptr_fft_data, len / 2, ptr_scratch_flt);
  }

  tmp1 = ia_add_flt(ptr_fft_data[0], ptr_fft_data[1]);
  ptr_fft_data[1] = ia_sub_flt(ptr_fft_data[0], ptr_fft_data[1]);
  ptr_fft_data[0] = tmp1;

  for (i = 1; i <= (len / 4 + (len % 4) / 2); ++i)
  {
    tmp1 = ia_sub_flt(ptr_fft_data[2 * i], ptr_fft_data[len - 2 * i]);
    tmp2 = ia_add_flt(ptr_fft_data[2 * i + 1], ptr_fft_data[len - 2 * i + 1]);

    s = ptr_sin[i];           /* sin(pi*i/(len/2)) */
    c = ptr_sin[i + len / 4]; /* cos(pi*i/(len/2)) */

    tmp3 = ia_msu_flt(ia_mul_flt(s, tmp1), c, tmp2); /* real part of j*W(k,N)*[T(k) - T'(N-k)] */
    tmp4 = ia_mac_flt(ia_mul_flt(c, tmp1), s, tmp2); /* imag part of j*W(k,N)*[T(k) - T'(N-k)] */

    tmp1 = ia_add_flt(ptr_fft_data[2 * i], ptr_fft_data[len - 2 * i]);
    tmp2 = ia_sub_flt(ptr_fft_data[2 * i + 1], ptr_fft_data[len - 2 * i + 1]);

    ptr_fft_data[2 * i] = ia_mul_flt(0.5f, ia_sub_flt(tmp1, tmp3));
    ptr_fft_data[2 * i + 1] = ia_mul_flt(0.5f, ia_sub_flt(tmp2, tmp4));
    ptr_fft_data[len - 2 * i] = ia_mul_flt(0.5f, ia_add_flt(tmp1, tmp3));
    ptr_fft_data[len - 2 * i + 1] = ia_mul_flt(-0.5f, ia_add_flt(tmp2, tmp4));
  }
  return;
}