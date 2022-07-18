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

#ifndef IMPEGHE_BITBUFFER_H
#define IMPEGHE_BITBUFFER_H
#include <setjmp.h>
typedef struct ia_bit_buf_struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;
  UWORD8 *ptr_write_next;

  WORD32 read_position;
  WORD32 write_position;
  WORD32 cnt_bits;
  WORD32 size;
  jmp_buf *impeghe_jmp_buf;
} ia_bit_buf_struct;

ia_bit_buf_struct *impeghe_create_bit_buffer(ia_bit_buf_struct *it_bit_buf,
                                             UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size);

VOID impeghe_reset_bit_buffer(ia_bit_buf_struct *it_bit_buf);

UWORD8 impeghe_write_bits_buf(ia_bit_buf_struct *it_bit_buf, UWORD32 write_val, UWORD8 num_bits);

WORD32 impeghe_write_escape_value(ia_bit_buf_struct *it_bit_buff, UWORD32 value, UWORD32 no_bits1,
                                  UWORD32 no_bits2, UWORD32 no_bits3);

UWORD32 impeghe_byte_align_buffer(ia_bit_buf_struct *it_bit_buff);

#endif /* IMPEGHE_BITBUFFER_H */
