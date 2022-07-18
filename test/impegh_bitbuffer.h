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

#ifndef IXHEAACD_BITBUFFER_H
#define IXHEAACD_BITBUFFER_H

#include <setjmp.h>

#define CRC_ADTS_HEADER_LEN 56
#define CRC_ADTS_RAW_DATA_BLK_LEN 192
#define CRC_ADTS_RAW_IIND_ICS 128
#define CRC_ADTS_LEN_ALL -1

#define MAX_REG_SIZE 192
#define MAX_CRC_REGS 7

struct ia_crc_bit_buf_struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;

  UWORD8 *ptr_read_next;
  WORD16 bit_pos;
  WORD32 cnt_bits;

  WORD32 size;
};

typedef struct
{
  UWORD8 active;
  WORD32 buf_size;
  WORD32 max_bits;
  UWORD32 bit_cnt;
  WORD32 bit_buf_cnt;
  struct ia_crc_bit_buf_struct str_bit_buf;
} ia_crc_reg_data_struct;

typedef struct
{
  UWORD8 crc_active;
  UWORD16 no_reg;
  UWORD16 file_value;
  UWORD16 crc_lookup[256];
  ia_crc_reg_data_struct str_crc_reg_data[MAX_CRC_REGS];
} ia_adts_crc_info_struct;

typedef struct ia_bit_buf_struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;

  UWORD8 *ptr_read_next;

  WORD32 bit_pos;
  WORD32 cnt_bits;

  WORD32 size;
#ifndef MPEG_H
  WORD32 adts_header_present;
  WORD32 crc_check;
  WORD8 protection_absent;
  WORD8 no_raw_data_blocks;
  ia_adts_crc_info_struct str_adts_crc_info;
  ia_adts_crc_info_struct *pstr_adts_crc_info;

  WORD32 initial_cnt_bits;
  WORD32 audio_mux_align;
#else
  WORD32 error;
#endif
  WORD32 bit_count;
  WORD32 valid_bits;
  UWORD8 byte;
  UWORD8 *byte_ptr;
  UWORD8 *ptr_start;
  WORD32 write_bit_count;
  WORD32 max_size;
  jmp_buf *xaac_jmp_buf;

} ia_bit_buf_struct;

typedef struct ia_bit_buf_struct *ia_handle_bit_buf_struct;

typedef struct ia_crc_bit_buf_struct *ia_crc_bit_buf_struct_handle;

VOID ixheaacd_byte_align(ia_bit_buf_struct *it_bit_buff, WORD32 *ptr_byte_align_bits);

ia_bit_buf_struct *ixheaacd_create_bit_buf(ia_bit_buf_struct *it_bit_buff,
                                           UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size);

VOID ixheaacd_create_init_bit_buf(ia_bit_buf_struct *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                  WORD32 bit_buf_size);

WORD32 ixheaacd_read_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

WORD32 ixheaacd_skip_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

WORD32 ixheaacd_show_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

VOID ixheaacd_read_bidirection(ia_bit_buf_struct *it_bit_buff, WORD32 ixheaacd_drc_offset);

UWORD32 ixheaacd_aac_showbits_32(UWORD8 *ptr_read_next, WORD32 cnt_bits, WORD32 *increment);

#ifndef MPEG_H
VOID ixheaacd_aac_read_byte(UWORD8 **ptr_read_next, WORD32 *bit_pos, WORD32 *readword);
#endif
VOID ixheaacd_aac_read_byte_corr(UWORD8 **ptr_read_next, WORD32 *ptr_bit_pos, WORD32 *readword,
                                 UWORD8 *p_bit_buf_end);

VOID ixheaacd_aac_read_byte_corr1(UWORD8 **ptr_read_next, WORD32 *ptr_bit_pos, WORD32 *readword,
                                  UWORD8 *p_bit_buf_end);

#define get_no_bits_available(it_bit_buff) ((it_bit_buff)->cnt_bits)
#define ixheaacd_no_bits_read(it_bit_buff) ((it_bit_buff)->size - (it_bit_buff)->cnt_bits)

WORD32 ixheaacd_aac_read_bit_rev(ia_bit_buf_struct *it_bit_buff);
WORD32 ixheaacd_aac_read_bit(ia_bit_buf_struct *it_bit_buff);

#ifdef MPEG_H
typedef struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;
  UWORD8 *ptr_write_next;

  WORD32 read_position;
  WORD32 write_position;
  WORD32 cnt_bits;
  WORD32 size;

} ia_write_bit_buf_struct;

WORD32 impeghd_create_write_bit_buffer(ia_write_bit_buf_struct *it_bit_buf,
                                       UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size,
                                       WORD32 init);
VOID impeghd_reset_write_bit_buffer(ia_write_bit_buf_struct *it_bit_buf);
UWORD8 impeghd_write_bits_buf(ia_write_bit_buf_struct *it_bit_buf, UWORD32 write_val,
                              UWORD8 num_bits);
WORD32 impeghd_write_escape_value(ia_write_bit_buf_struct *it_bit_buff, UWORD32 value,
                                  UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3);
#else
VOID ixheaacd_write_bit(ia_bit_buf_struct *it_bit_buff, WORD32 value, WORD32 no_of_bits);

WORD32 ixheaacd_read_bit(ia_bit_buf_struct *data, WORD32 no_of_bits);
#endif

#endif /* IXHEAACD_BITBUFFER_H */
