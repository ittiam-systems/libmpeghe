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
#include "impeghe_block_switch_const.h"
#include "impeghe_cnst.h"
#include "impeghe_rom.h"
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
#include "impeghe_tcx_mdct.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_tcx_mdct_main
 *
 *  \brief MDCT interface function for the TCX module. The input is
 *         re-arranged before calling the actual MDCT.
 *
 *  \param [in ] ptr_in       Pointer to input buffer to MDCT module.
 *  \param [out] ptr_out      Pointer to output buffer of MDCT.
 *  \param [in ] l            Value corresponding to input length.
 *  \param [in ] m            Value corresponding to input length.
 *  \param [in ] r            Value corresponding to input length.
 *  \param [in ] pstr_scratch Pointer to scratch structure.
 *
 *  \return VOID
 */
VOID impeghe_tcx_mdct_main(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 l, WORD32 m, WORD32 r,
                           impeghe_scratch_mem *pstr_scratch)
{

  WORD32 length = l / 2 + m + r / 2;
  FLOAT32 *ptr_input = pstr_scratch->p_tcx_input;
  WORD32 i;

  for (i = 0; i < m / 2; i++)
  {
    ptr_input[m / 2 + r / 2 + i] = -ptr_in[l + m / 2 - 1 - i];
  }
  for (i = 0; i < l / 2; i++)
  {
    ptr_input[m / 2 + r / 2 + m / 2 + i] = ptr_in[i] - ptr_in[l - 1 - i];
  }
  for (i = 0; i < m / 2; i++)
  {
    ptr_input[m / 2 + r / 2 - 1 - i] = -ptr_in[l + m / 2 + i];
  }
  for (i = 0; i < r / 2; i++)
  {
    ptr_input[m / 2 + r / 2 - 1 - m / 2 - i] = -ptr_in[l + m + i] - ptr_in[l + m + r - 1 - i];
  }

  impeghe_tcx_mdct(ptr_input, ptr_out, length, pstr_scratch);

  return;
}

/**
 *  impeghe_tcx_imdct
 *
 *  \brief IMDCT interface function for the TCX module. The input is
 *         re-arranged before calling the actual MDCT.
 *
 *  \param [in ] ptr_in       Pointer to input buffer to MDCT module.
 *  \param [out] ptr_out      Pointer to output buffer of MDCT.
 *  \param [in ] l            Value corresponding to input length.
 *  \param [in ] m            Value corresponding to input length.
 *  \param [in ] r            Value corresponding to input length.
 *  \param [in ] pstr_scratch Pointer to scratch structure.
 *
 *  \return VOID
 */
VOID impeghe_tcx_imdct(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 l, WORD32 m, WORD32 r,
                       impeghe_scratch_mem *pstr_scratch)
{

  WORD32 length = l / 2 + m + r / 2;
  FLOAT32 *put_output = pstr_scratch->p_tcx_output;
  impeghe_tcx_mdct(ptr_in, put_output, length, pstr_scratch);

  WORD32 i;

  for (i = 0; i < m / 2; i++)
  {
    ptr_out[l + m / 2 - 1 - i] = -1.0f * put_output[m / 2 + r / 2 + i];
  }
  for (i = 0; i < l / 2; i++)
  {
    ptr_out[i] = put_output[m / 2 + r / 2 + m / 2 + i];
    ptr_out[l - 1 - i] = -1.0f * put_output[m / 2 + r / 2 + m / 2 + i];
  }
  for (i = 0; i < m / 2; i++)
  {
    ptr_out[l + m / 2 + i] = -1.0f * put_output[m / 2 + r / 2 - 1 - i];
  }
  for (i = 0; i < r / 2; i++)
  {
    ptr_out[l + m + i] = -1.0f * put_output[m / 2 + r / 2 - 1 - m / 2 - i];
    ptr_out[l + m + r - 1 - i] = -1.0f * put_output[m / 2 + r / 2 - 1 - m / 2 - i];
  }
  return;
}

/**
 *  impeghe_get_pre_post_twid
 *
 *  \brief Initializes the twiddle tables corresponding to
 *         pre and post processing blocks inside MDCT implementation.
 *
 *  \param [out] pptr_pre_twid_re  Pointer to real part of pre-proc table.
 *  \param [out] pptr_pre_twid_im  Pointer to imag part of pre-proc table.
 *  \param [out] pptr_post_twid_re Pointer to real part of post-proc table.
 *  \param [out] pptr_post_twid_im Pointer to imag part of post-proc table.
 *  \param [in ] length           Value corresponding to MDCT length.
 *
 *  \return VOID
 */
static VOID impeghe_get_pre_post_twid(FLOAT32 **pptr_pre_twid_re, FLOAT32 **pptr_pre_twid_im,
                                      FLOAT32 **pptr_post_twid_re, FLOAT32 **pptr_post_twid_im,
                                      WORD32 length)
{
  switch (length)
  {
  case 512:
    *pptr_pre_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_256[0][0];
    *pptr_pre_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_256[1][0];
    *pptr_post_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_256[2][0];
    *pptr_post_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_256[3][0];
    break;
  case 256:
    *pptr_pre_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_128[0][0];
    *pptr_pre_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_128[1][0];
    *pptr_post_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_128[2][0];
    *pptr_post_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_128[3][0];
    break;
  case 128:
    *pptr_pre_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_64[0][0];
    *pptr_pre_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_64[1][0];
    *pptr_post_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_64[2][0];
    *pptr_post_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_64[3][0];
    break;
  case 64:
    *pptr_pre_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_32[0][0];
    *pptr_pre_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_32[1][0];
    *pptr_post_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_32[2][0];
    *pptr_post_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_32[3][0];
    break;
  default:
    *pptr_pre_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_512[0][0];
    *pptr_pre_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_512[1][0];
    *pptr_post_twid_re = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_512[2][0];
    *pptr_post_twid_im = (FLOAT32 *)&impeghe_pre_post_twid_cos_sin_512[3][0];
  }

  return;
}

/**
 *  impeghe_pre_twid
 *
 *  \brief Processes input before passing to FFT - first step in
 *         FFT based MDCT implementation.
 *
 *  \param [in ]    ptr_twid_re Pointer to twiddle table real part.
 *  \param [in ]    ptr_twid_im Pointer to twiddle table imag part.
 *  \param [in,out] ptr_in      Pointer to input data.
 *  \param [in]     length      Input data length.
 *
 *  \return VOID
 */
static VOID impeghe_pre_twid(FLOAT32 *ptr_twid_re, FLOAT32 *ptr_twid_im, FLOAT32 *ptr_in,
                             WORD32 length)
{
  WORD32 i;
  for (i = 0; i < length; i++)
  {
    FLOAT32 temp = ptr_in[2 * i] * ptr_twid_re[i] - ptr_in[2 * i + 1] * ptr_twid_im[i];

    ptr_in[2 * i + 1] = ptr_in[2 * i] * ptr_twid_im[i] + ptr_in[2 * i + 1] * ptr_twid_re[i];

    ptr_in[2 * i] = temp;
  }
  return;
}

/**
 *  impeghe_post_twid
 *
 *  \brief Processes output of the FFT block - last step in
 *         FFT based MDCT implementation.
 *
 *  \param [in ]    ptr_twid_re Pointer to twiddle table real part.
 *  \param [in ]    ptr_twid_im Pointer to twiddle table imag part.
 *  \param [in,out] ptr_in      Pointer to input data.
 *  \param [in]     length      Input data length.
 *
 *  \return VOID
 */
static VOID impeghe_post_twid(FLOAT32 *ptr_twid_re, FLOAT32 *ptr_twid_im, FLOAT32 *ptr_in,
                              WORD32 length)
{
  WORD32 i;
  for (i = 0; i < length; i++)
  {
    FLOAT32 temp = ptr_in[2 * i] * ptr_twid_re[i] - ptr_in[2 * i + 1] * ptr_twid_im[i];

    ptr_in[2 * i + 1] = -ptr_in[2 * i] * ptr_twid_im[i] - ptr_in[2 * i + 1] * ptr_twid_re[i];
    ptr_in[2 * i] = temp;
  }
  return;
}

/**
 *  impeghe_tcx_mdct
 *
 *  \brief Carries out FFT based MDCT.
 *
 *  \param [in ] ptr_in       Pointer to input buffer.
 *  \param [out] ptr_out      Pointer to output buffer.
 *  \param [in ] length       Length of input buffer.
 *  \param [in ] pstr_scratch Pointer to scratch structure.
 *
 *  \return VOID
 */
VOID impeghe_tcx_mdct(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 length,
                      impeghe_scratch_mem *pstr_scratch)
{
  FLOAT32 *ptr_pre_twid_re, *ptr_pre_twid_im;
  FLOAT32 *ptr_post_twid_re, *ptr_post_twid_im;
  WORD32 i;
  FLOAT32 *ptr_real = pstr_scratch->p_temp_mdct;
  WORD32 len_by_2 = length >> 1;

  impeghe_get_pre_post_twid(&ptr_pre_twid_re, &ptr_pre_twid_im, &ptr_post_twid_re,
                            &ptr_post_twid_im, length);

  for (i = 0; i < len_by_2; i++)
  {
    ptr_real[2 * i] = ptr_in[2 * i];
    ptr_real[2 * i + 1] = ptr_in[length - 1 - 2 * i];
  }

  /* pre twiddle */
  impeghe_pre_twid(ptr_pre_twid_re, ptr_pre_twid_im, ptr_real, len_by_2);

  impeghe_complex_fft(ptr_real, len_by_2, pstr_scratch);

  /* post twiddle */
  impeghe_post_twid(ptr_post_twid_re, ptr_post_twid_im, ptr_real, len_by_2);

  for (i = 0; i < len_by_2; i++)
  {
    ptr_out[2 * i] = ptr_real[2 * i];
    ptr_out[length - 1 - 2 * i] = ptr_real[2 * i + 1];
  }

  return;
}
/** @} */ /* End of CoreEncProc */
