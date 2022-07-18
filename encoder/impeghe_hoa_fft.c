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

#include "impeghe_type_def.h"
#include "impeghe_hoa_common_values.h"
#include "impeghe_hoa_rom.h"

#include "impeghe_basic_ops_flt.h"

#define DIG_REV(i, m, j)                                                                         \
  do                                                                                             \
  {                                                                                              \
    unsigned _ = (i);                                                                            \
    _ = ((_ & 0x33333333) << 2) | ((_ & ~0x33333333) >> 2);                                      \
    _ = ((_ & 0x0F0F0F0F) << 4) | ((_ & ~0x0F0F0F0F) >> 4);                                      \
    _ = ((_ & 0x00FF00FF) << 8) | ((_ & ~0x00FF00FF) >> 8);                                      \
    (j) = _ >> (m);                                                                              \
  } while (0)

/**
 *  impeghe_calc_norm
 *
 *  \brief Calculate norm of a number
 *
 *  \param [in] a Input number
 *
 *  \return CHAR8 norm value
 */
static CHAR8 impeghe_calc_norm(WORD32 a)
{
  CHAR8 norm_val;

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
 *  impeghe_complex_fft_re_input
 *
 *  \brief calculate complex FFT of nlength with only real values
 *
 *  \param [in] ptr_x input data
 *  \param [in] nlength length of FFT
 *  \param [out] ptr_y fft output
 *
 *  \return VOID
 */
VOID impeghe_complex_fft_re_input(FLOAT32 *ptr_x, WORD32 nlength, FLOAT32 *ptr_y)
{
  WORD32 i, j, k, n_stages, h2;
  FLOAT32 x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;
  WORD32 del, nodespacing, in_loop_cnt;
  WORD32 not_power_4;
  WORD32 dig_rev_shift;
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
    ptr_inp += (h2 >> 1);

    x0r = *ptr_inp;
    x0i = 0;
    ptr_inp += (npoints >> 2);

    x1r = *ptr_inp;
    x1i = 0;
    ptr_inp += (npoints >> 2);

    x2r = *ptr_inp;
    x2i = 0;
    ptr_inp += (npoints >> 2);

    x3r = *ptr_inp;
    x3i = 0;

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

      tmp = (FLOAT32)(impeghe_dmult((FLOAT64)x1r, W1) - impeghe_dmult((FLOAT64)x1i, W4)) /*/2*/;
      x1i = (FLOAT32)impeghe_dmac(impeghe_dmult((FLOAT64)x1r, W4), (FLOAT64)x1i, W1) /*/2*/;
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
  return;
}
