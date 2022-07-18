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

/**
 *  impeghe_calc_window
 *
 *  \brief Map window shape to window coeffs
 *
 *  \param [out] pptr_win		Pointer to window coeffs
 *  \param [in] win_sz		Pointer to window size
 *  \param [in] win_sel		Pointer to window shape
 *
 *  \return IA_ERRORCODE	Error code
 *
 */
IA_ERRORCODE impeghe_calc_window(FLOAT64 **pptr_win, WORD32 win_sz, WORD32 win_sel)
{
  switch (win_sel)
  {
  case WIN_SEL_0:
    switch (win_sz)
    {
    case WIN_LEN_128:
      *pptr_win = (FLOAT64 *)impeghe_sine_win_128;
      break;
    case WIN_LEN_256:
      *pptr_win = (FLOAT64 *)impeghe_sine_win_256;
      break;
    case WIN_LEN_1024:
      *pptr_win = (FLOAT64 *)impeghe_sine_win_1024;
      break;
    default:
      return -1;
      break;
    }
    break;
  case WIN_SEL_1:
    switch (win_sz)
    {
    case WIN_LEN_128:
      *pptr_win = (FLOAT64 *)impeghe_sine_win_128;
      break;
    case WIN_LEN_256:
      *pptr_win = (FLOAT64 *)impeghe_kbd_win256;
      break;
    case WIN_LEN_1024:
      *pptr_win = (FLOAT64 *)impeghe_kbd_win1024;
      break;
    default:
      return -1;
      break;
    }
    break;

  default:
    return -1;
    break;
  }
  return 0;
}

/**
 *  impeghe_windowing_long
 *
 *  \brief Perform windowing on long sequence
 *
 *  \param [in] ptr_win_long		Pointer to window coeffs
 *  \param [in] ptr_overlap			Pointer to overlap buffer
 *  \param [out] ptr_win_buf		Pointer to output buffer
 *  \param [in] ptr_in_data			Pointer to input data
 *  \param [in] n_long				length
 *
 *  \return VOID
 *
 */
VOID impeghe_windowing_long(FLOAT64 *ptr_overlap, FLOAT64 *ptr_win_long, FLOAT64 *ptr_win_buf,
                            FLOAT64 *ptr_in_data, WORD32 n_long)
{
  WORD32 i;
  FLOAT64 *ptr_win = ptr_win_long + n_long - 1;

  for (i = 0; i < n_long; i++)
  {
    ptr_win_buf[i] = ptr_overlap[i] * ptr_win_long[i];
  }

  memcpy(ptr_overlap, ptr_overlap + n_long, 576 * sizeof(FLOAT64));
  memcpy(ptr_overlap + 576, ptr_in_data, n_long * sizeof(FLOAT64));

  for (i = 0; i < n_long; i++)
  {
    ptr_win_buf[i + n_long] = ptr_overlap[i] * (*ptr_win--);
  }

  return;
}

/**
 *  impeghe_windowing_long_start
 *
 *  \brief Perform windowing on long start sequence
 *
 *  \param [in] ptr_win_long	Pointer to window coeffs
 *  \param [out] ptr_win_buf	Pointer to output buffer
 *  \param [in] ptr_overlap		Pointer to overlap buffer
 *  \param [in] ptr_in_data		Pointer to input data
 *  \param [in] n_long			length
 *  \param [in] nflat_ls		length
 *  \param [in] ptr_win_med		Pointer to window coeffs
 *  \param [in] win_sz			Window size
 *
 *  \return VOID
 *
 */
VOID impeghe_windowing_long_start(FLOAT64 *ptr_overlap, FLOAT64 *ptr_win_long,
                                  FLOAT64 *ptr_win_buf, FLOAT64 *ptr_in_data, WORD32 n_long,
                                  WORD32 nflat_ls, FLOAT64 *ptr_win_med, WORD32 win_sz)
{
  WORD32 i;
  FLOAT64 *ptr_win = ptr_win_buf + 2 * n_long - 1;

  for (i = 0; i < n_long; i++)
  {
    ptr_win_buf[i] = ptr_overlap[i] * ptr_win_long[i];
  }

  memcpy(ptr_overlap, ptr_overlap + n_long, 576 * sizeof(FLOAT64));
  memcpy(ptr_overlap + 576, ptr_in_data, n_long * sizeof(FLOAT64));
  memcpy(ptr_win_buf + n_long, ptr_overlap, nflat_ls * sizeof(FLOAT64));

  ptr_win_med = ptr_win_med + win_sz - 1;
  win_sz = n_long - 2 * nflat_ls;

  for (i = 0; i < win_sz; i++)
  {
    ptr_win_buf[i + n_long + nflat_ls] = ptr_overlap[i + nflat_ls] * (*ptr_win_med--);
  }

  for (i = 0; i < nflat_ls; i++)
  {
    *ptr_win-- = 0;
  }

  return;
}

/**
 *  impeghe_windowing_long_stop
 *
 *  \brief Perform windowing on long stop sequence
 *
 *  \param [in] ptr_win_long	Pointer to window coeffs
 *  \param [out] ptr_win_buf	Pointer to output buffer
 *  \param [in] ptr_overlap		Pointer to overlap buffer
 *  \param [in] ptr_in_data		Pointer to input data
 *  \param [in] n_long			length
 *  \param [in] nflat_ls		length
 *  \param [in] ptr_win_med		Pointer to window coeffs
 *  \param [in] win_sz			Window size
 *
 *  \return VOID
 *
 */
VOID impeghe_windowing_long_stop(FLOAT64 *ptr_overlap, FLOAT64 *ptr_win_long,
                                 FLOAT64 *ptr_win_buf, FLOAT64 *ptr_in_data, WORD32 n_long,
                                 WORD32 nflat_ls, FLOAT64 *ptr_win_med, WORD32 win_sz)
{
  WORD32 i;
  FLOAT64 *ptr_win = ptr_win_long + n_long - 1;

  memset(ptr_win_buf, 0, nflat_ls * sizeof(FLOAT64));
  for (i = 0; i < win_sz; i++)
  {
    ptr_win_buf[i + nflat_ls] = ptr_overlap[i + nflat_ls] * ptr_win_med[i];
  }

  memcpy(ptr_win_buf + nflat_ls + win_sz, ptr_overlap + nflat_ls + win_sz,
         nflat_ls * sizeof(FLOAT64));
  memcpy(ptr_overlap, ptr_overlap + n_long, 576 * sizeof(FLOAT64));
  memcpy(ptr_overlap + 576, ptr_in_data, n_long * sizeof(FLOAT64));

  for (i = 0; i < n_long; i++)
  {
    ptr_win_buf[i + n_long] = ptr_overlap[i] * (*ptr_win--);
  }
  return;
}

/**
 *  impeghe_windowing_stop_start
 *
 *  \brief Perform windowing on stop start sequence
 *
 *  \param [in] ptr_win_med			Pointer to window coeffs
 *  \param [in] ptr_overlap			Pointer to overlap buffer
 *  \param [out] ptr_win_buf		Pointer to output buffer
 *  \param [in] win_sz				Window size
 *  \param [in] n_long				length
 *
 *  \return VOID
 *
 */
VOID impeghe_windowing_stop_start(FLOAT64 *ptr_overlap, FLOAT64 *ptr_win_buf,
                                  FLOAT64 *ptr_win_med, WORD32 win_sz, WORD32 n_long)
{
  WORD32 i;
  FLOAT64 *win_gen;
  WORD32 wsize = (n_long - win_sz) >> 1;
  win_gen = ptr_win_med;

  for (i = 0; i < win_sz; i++)
  {
    ptr_win_buf[wsize + i] = ptr_overlap[wsize + i] * (*win_gen++);
  }
  memcpy(ptr_win_buf + wsize, ptr_overlap + wsize, wsize * sizeof(FLOAT64));
  memcpy(ptr_win_buf + n_long, ptr_overlap + n_long, wsize * sizeof(FLOAT64));

  win_gen = ptr_win_med + win_sz - 1;
  win_sz = n_long - 2 * wsize;

  for (i = 0; i < win_sz; i++)
  {
    ptr_win_buf[n_long + wsize + i] = ptr_overlap[n_long + wsize + i] * (*win_gen--);
  }
  return;
}
