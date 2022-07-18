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
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
/**
 *  impeghe_create_bit_buffer
 *
 *  \brief Creates and initializes the bit-buffer using given base pointer, size and init flag
 *
 *  \param [in,out]	it_bit_buf			Pointer to bit-buffer
 *  \param [in]		ptr_bit_buf_base	Pointer to bit-buffer base
 *  \param [in]		bit_buffer_size		Size of bit buffer to be created
 *
 *  \return ia_bit_buf_struct  Bit buffer
 *
 */
ia_bit_buf_struct *impeghe_create_bit_buffer(ia_bit_buf_struct *it_bit_buf,
                                             UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size)
{
  it_bit_buf->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buf->ptr_bit_buf_end = ptr_bit_buf_base + bit_buffer_size - 1;
  it_bit_buf->ptr_read_next = ptr_bit_buf_base;
  it_bit_buf->ptr_write_next = ptr_bit_buf_base;

  it_bit_buf->write_position = 7;
  it_bit_buf->read_position = 7;
  it_bit_buf->cnt_bits = 0;
  it_bit_buf->size = bit_buffer_size * 8;

  return (it_bit_buf);
}

/**
 *  impeghe_reset_bit_buffer
 *
 *  \brief Resets the bit-buffer
 *
 *  \param [in,out]	it_bit_buf	Pointer to bit-buffer
 *
 *  \return VOID
 *
 */
VOID impeghe_reset_bit_buffer(ia_bit_buf_struct *it_bit_buf)
{
  it_bit_buf->ptr_read_next = it_bit_buf->ptr_bit_buf_base;
  it_bit_buf->ptr_write_next = it_bit_buf->ptr_bit_buf_base;

  it_bit_buf->write_position = 7;
  it_bit_buf->read_position = 7;
  it_bit_buf->cnt_bits = 0;

  return;
}

/**
 *  impeghe_write_bits_buf
 *
 *  \brief Writes bits into bit-buffer
 *
 *  \param [in,out]	it_bit_buf	Pointer to bit-buffer
 *  \param [in]		write_val	Value to be written into bit-buffer
 *  \param [in]		num_bits	Number of bits to be written
 *
 *  \return UWORD8 Number of bits written
 *
 */
UWORD8 impeghe_write_bits_buf(ia_bit_buf_struct *it_bit_buf, UWORD32 write_val, UWORD8 num_bits)
{
  WORD32 bits_to_write;
  WORD32 write_position;
  UWORD8 *ptr_write_next;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 bits_written = num_bits;
  if (it_bit_buf)
  {
    it_bit_buf->cnt_bits += num_bits;

    write_position = it_bit_buf->write_position;
    ptr_write_next = it_bit_buf->ptr_write_next;
    ptr_bit_buf_end = it_bit_buf->ptr_bit_buf_end;
    while (num_bits)
    {
      UWORD8 tmp, msk;

      bits_to_write = min(write_position + 1, num_bits);

      tmp = (UWORD8)(write_val << (32 - num_bits) >> (32 - bits_to_write)
                                                         << (write_position + 1 - bits_to_write));

      msk = ~(((1 << bits_to_write) - 1) << (write_position + 1 - bits_to_write));

      *ptr_write_next &= msk;
      *ptr_write_next |= tmp;

      write_position -= bits_to_write;

      num_bits -= bits_to_write;

      if (write_position < 0)
      {

        write_position += 8;
        ptr_write_next++;

        if (ptr_write_next > ptr_bit_buf_end)
        {
          longjmp(*(it_bit_buf->impeghe_jmp_buf), 1);
        }
      }
    }

    it_bit_buf->write_position = write_position;
    it_bit_buf->ptr_write_next = ptr_write_next;
  }

  return (bits_written);
}

/**
 *  impeghe_write_escape_value
 *
 *  \brief Writes escape value into bit-buffer
 *
 *  \param [in,out]	it_bit_buff	Pointer to bit-buffer
 *  \param [in]		value		Escape value
 *  \param [in]		no_bits1	Number of bits - first level
 *  \param [in]		no_bits2	Number of bits - second level
 *  \param [in]		no_bits3	Number of bits - third level
 *
 *  \return WORD32	Number of bits written
 *
 */
WORD32 impeghe_write_escape_value(ia_bit_buf_struct *it_bit_buff, UWORD32 value, UWORD32 no_bits1,
                                  UWORD32 no_bits2, UWORD32 no_bits3)
{

  WORD32 bit_cnt = 0;
  UWORD32 esc_val = 0;
  UWORD32 max_val1 = ((UWORD32)1 << no_bits1) - 1;
  UWORD32 max_val2 = ((UWORD32)1 << no_bits2) - 1;
  UWORD32 max_val3 = ((UWORD32)1 << no_bits3) - 1;

  esc_val = min(value, max_val1);
  bit_cnt += impeghe_write_bits_buf(it_bit_buff, esc_val, no_bits1);

  if (esc_val == max_val1)
  {
    value = value - esc_val;

    esc_val = min(value, max_val2);
    bit_cnt += impeghe_write_bits_buf(it_bit_buff, esc_val, no_bits2);

    if (esc_val == max_val2)
    {
      value = value - esc_val;

      esc_val = min(value, max_val3);
      bit_cnt += impeghe_write_bits_buf(it_bit_buff, esc_val, no_bits3);
    }
  }

  return bit_cnt;
}

/**
 *  impeghe_byte_align_buffer
 *
 *  \brief Byte aligns the bit-buffer
 *
 *  \param [in] it_bit_buff	Pointer to bit-buffer
 *
 *  \return UWORD32 Alignment
 */
UWORD32 impeghe_byte_align_buffer(ia_bit_buf_struct *it_bit_buff)
{
  WORD alignment;
  alignment = (WORD)((it_bit_buff->cnt_bits) & 0x07);

  if (alignment)
  {
    impeghe_write_bits_buf(it_bit_buff, 0, (8 - alignment));
    return (8 - alignment);
  }
  return 0;
}
