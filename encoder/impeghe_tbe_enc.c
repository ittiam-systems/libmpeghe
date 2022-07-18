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

#include "string.h"
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_tbe_rom.h"

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/**
 *  impeghe_tbe_resample_input
 *
 *  \brief Resample ptr_input_signal data to half before checking.
 *
 *  \param [In]  ptr_input_signal   Pointer to input samples buffer.
 *  \param [Out] ptr_output_signal  Pointer to output samples buffer.
 *  \param [I/O] ptr_filt_mem       Pointer to filter memory states.
 *  \param [In]  ptr_scratch_mem    Pointer to scratch memory.
 *  \param [In]  num_samples_in     Number of input samples.
 *
 *  \return VOID
 *
 */
VOID impeghe_tbe_resample_input(FLOAT32 *ptr_output_signal, FLOAT32 *ptr_input_signal,
                                FLOAT32 *ptr_filt_mem, FLOAT32 *ptr_scratch_mem,
                                WORD32 num_samples_in)
{
  WORD32 i = 0;
  FLOAT32 *ptr_filt_in = ptr_scratch_mem;
  memcpy(ptr_filt_in, ptr_filt_mem, DOWNSAMP_FILT_MEM_SIZE * sizeof(ptr_filt_in[0]));
  memcpy(&ptr_filt_in[DOWNSAMP_FILT_MEM_SIZE], ptr_input_signal,
         num_samples_in * sizeof(ptr_filt_in[0]));

  for (i = 0; i < (num_samples_in >> 1); i++)
  {
    WORD32 j = 0;
    FLOAT32 *ptr_input;
    FLOAT32 temp = 0.0f;
    /*
     * The below operations work fine as long as the DOWNSAMP_FILT_LEN is even
     */
    ptr_input = &ptr_filt_in[2 * i + (DOWNSAMP_FILT_LEN >> 1)];
    temp = ptr_filt_in[2 * i + (DOWNSAMP_FILT_LEN >> 1)];
    for (j = 0; j < (DOWNSAMP_FILT_LEN >> 1); j++)
    {
      temp += (ptr_input[-j - 1] + ptr_input[j + 1]) * impeghe_tdr_filter_interp[j];
    }
    ptr_output_signal[i] = (temp * 0.5f);
  }
  memcpy(ptr_filt_mem, &ptr_filt_in[num_samples_in],
         DOWNSAMP_FILT_MEM_SIZE * sizeof(ptr_filt_mem[0]));

  return;
}

/**
 *  impeghe_write_tbe_data
 *
 *  \brief Write TD-bandwidth extenstion bit stream data.
 *
 *  \param pstr_tbe_enc_data Pointer to TBE data structure.
 *  \param it_bit_buff       Pointer to bit buffer structure.
 *  \param tbe_frame_class   TBE frame class type.
 *
 *  \return VOID
 *
 */
VOID impeghe_write_tbe_data(ia_usac_tbe_data_str *pstr_tbe_enc_data,
                            ia_bit_buf_struct *it_bit_buff, WORD32 tbe_frame_class)
{
  WORD32 sf_idx = (1 == tbe_frame_class) ? 1 : 0;
  WORD32 bits_written = 0;
  ia_usac_tbe_bitstream_str *pstr_tbe_bs = &pstr_tbe_enc_data->bit_stream[sf_idx];

  bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->harm_ext_mode, 1);
  bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->frame_gain_idx, 5);
  bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->sub_gain_idx, 5);
  bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->lsf_idx[0], 7);
  bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->lsf_idx[1], 7);

  if (pstr_tbe_bs->harm_ext_mode == 0)
  {
    bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->hr_config, 1);
    bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->nl_config, 1);
    bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->idx_mix_config, 2);

    if (pstr_tbe_bs->hr_config == 1)
    {
      bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->idx_shb_fr_gain, 6);
      bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->idx_res_sub_gains, 5);
    }
    else
    {
      bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->idx_shb_exc_resp[0], 7);
      bits_written += impeghe_write_bits_buf(it_bit_buff, pstr_tbe_bs->idx_shb_exc_resp[1], 4);
    }
  }
  return;
}
/** @} */ /* End of CoreEncProc */
