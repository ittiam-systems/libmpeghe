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
#include "impeghe_crc.h"

/**
 *  impeghe_enc_crc
 *
 *  \brief CRC output stream calculation and reverse
 *
 *  \param [out]    ptr_crc       CRC value stream
 *  \param [in]     ptr_inseq     Input Buffer
 *  \param [in]     n         length of inseq
 *  \param [in]     n_crc     length of crc required
 *
 *  \return VOID
 *
 */
VOID impeghe_enc_crc(WORD32 *ptr_inseq, WORD32 *ptr_crc, WORD32 n, WORD32 n_crc)
{
  LOOPIDX i;
  UWORD32 shift_reg, crcdata = 0;

  // CRC Calculation
  if (n_crc == 16)
  {
    crcdata = 98337; /* Polynomial for 16 bit: x16+x15+x5+1 */
  }
  else if (n_crc == 32)
  {
    crcdata =
        79764919; /* Plynomial for 32 bit: x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1 */
    /* Should be 4374732215 which would be casted to 79764919, whenever using WORD32. Ignoring the
     * MSB 1 as it does not fit into WORD32 data type*/
  }

  shift_reg = 0;

  if (n >= n_crc)
  {
    for (i = 0; i < n_crc; i++)
      shift_reg = (shift_reg << 1) | ptr_inseq[i];
    for (i = n_crc; i < n; i++)
      shift_reg =
          ((shift_reg << 1) | ptr_inseq[i]) ^ (((shift_reg >> (n_crc - 1)) & 0x1) * crcdata);
    for (i = n; i < n_crc + n; i++)
      shift_reg = (shift_reg << 1) ^ (((shift_reg >> (n_crc - 1)) & 0x1) * crcdata);
  }
  else
  {
    for (i = 0; i < n; i++)
      shift_reg = (shift_reg << 1) | ptr_inseq[i];
    for (i = n; i < n_crc; i++)
      shift_reg = (shift_reg << 1);
    for (i = n_crc; i < n_crc + n; i++)
      shift_reg = (shift_reg << 1) ^ (((shift_reg >> (n_crc - 1)) & 0x1) * crcdata);
  }
  for (i = 0; i < n_crc; i++)
  {
    ptr_crc[i] = shift_reg & 0x1;
    shift_reg >>= 1;
  }

  /* reverse bit */
  for (i = 0; i < n_crc; i++)
  {
    ptr_crc[i] = !ptr_crc[i];
  }
}
