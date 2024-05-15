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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "impeghe_type_def.h"
#include "impeghe_mp4_writer.h"

/*
Ittiam comments:

1. 'mhaC' box is 4 byte-aligned now which is not necessary.

2. chunk offsets assume 'mdat' box comes right after 'stco' box.

3. 'mhm1' support to be added.

*/

/**impeghe_mp4_rev32
 *
 *  \brief Change big endian to small endian and vice-versa.
 *         Input is 4 bytes
 *
 *  \param [in]		l 4 bytes Input
 *
 *  \return UWORD32
 *
 */
UWORD32 impeghe_mp4_rev32(UWORD32 l)
{
  return (((l & 0xff) << 24) + ((l & 0xff00) << 8) + ((l & 0xff0000) >> 8) +
          ((l & 0xff000000) >> 24));
}

/**impeghe_mp4_write_ftyp
 *
 *  \brief Write MP4 ftype
 *
 *  \param [in,out] pstr_mp4_writer_io  Pointer to ia_mp4_writer_struct structure.
 *
 *  \return VOID
 *
 */
static VOID impeghe_mp4_write_ftyp(ia_mp4_writer_struct *pstr_mp4_writer_io)
{
  WORD32 box_size, box_type, buf;
  UWORD8 charbuf[10] = {'f', 't', 'y', 'p'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  box_size = impeghe_mp4_rev32(24);
  fwrite(&box_size, 4, 1, pstr_mp4_writer_io->fp_mp4); // Box size

  box_type = *data;
  fwrite(&box_type, 4, 1, pstr_mp4_writer_io->fp_mp4);
  charbuf[0] = 'm';
  charbuf[1] = 'p';
  charbuf[2] = '4';
  charbuf[3] = '2';

  buf = *data; // Major brand
  fwrite(&buf, 4, 1, pstr_mp4_writer_io->fp_mp4);

  buf = impeghe_mp4_rev32(0x1); // Minor version
  fwrite(&buf, 4, 1, pstr_mp4_writer_io->fp_mp4);
  charbuf[0] = 'm';
  charbuf[1] = 'p';
  charbuf[2] = '4';
  charbuf[3] = '2';
  buf = *data; // compatible brand 1

  fwrite(&buf, 4, 1, pstr_mp4_writer_io->fp_mp4);
  charbuf[0] = 'i';
  charbuf[1] = 's';
  charbuf[2] = 'o';
  charbuf[3] = 'm';
  buf = *data; // compatible brand 2

  fwrite(&buf, 4, 1, pstr_mp4_writer_io->fp_mp4);
}

/**impeghe_mp4_write_mdat
 *
 *  \brief Write MP4 mdat
 *
 *  \param [in,out] pstr_mp4_writer_io  Pointer to ia_mp4_writer_struct structure.
 *
 *  \return VOID
 *
 */
static VOID impeghe_mp4_write_mdat(ia_mp4_writer_struct *pstr_mp4_writer_io)
{
  WORD32 box_size, box_type;
  UWORD8 charbuf[10] = {'m', 'd', 'a', 't'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // Box size
  box_size = impeghe_mp4_rev32((WORD32)pstr_mp4_writer_io->mdat_size + 8);
  fwrite(&box_size, 4, 1, pstr_mp4_writer_io->fp_mp4);

  // Box type

  box_type = *data;
  fwrite(&box_type, 4, 1, pstr_mp4_writer_io->fp_mp4);

  // Box data written in main testbench
}

/**impeghe_mp4_write_mvhd
 *
 *  \brief Write MP4 mvhd data
 *
 *  \param [in,out] pstr_mp4_writer_io  Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf Pointer to buffer
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_mvhd(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'m', 'v', 'h', 'd'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  UWORD32 creation_time = 0x0;
  UWORD32 modification_time = 0x0;
  UWORD32 timescale = pstr_mp4_writer_io->meta_info.media_time_scale;
  UWORD32 duration = pstr_mp4_writer_io->meta_info.playTimeInSamples[0];
  UWORD32 rate = 0x00010000; // typically 1.0
  UWORD16 volume = 0x0100;   // typically, full volume
  WORD32 unitary_matrix[9] = {0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000};
  UWORD32 next_track_id = 0xFFFFFFFF;

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  box_type = 0;
  *ptr++ = box_type;
  bytes_used += 4;

  // creation time
  box_type = impeghe_mp4_rev32(creation_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // modification time
  box_type = impeghe_mp4_rev32(modification_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // timescale
  box_type = impeghe_mp4_rev32(timescale);
  *ptr++ = box_type;
  bytes_used += 4;

  // duration
  box_type = impeghe_mp4_rev32(duration);
  *ptr++ = box_type;
  bytes_used += 4;

  // rate
  box_type = impeghe_mp4_rev32(rate);
  *ptr++ = box_type;
  bytes_used += 4;

  // volume + 16 bits reserved
  box_type = impeghe_mp4_rev32((WORD32)volume);
  *ptr++ = box_type;
  bytes_used += 4;

  // reserved
  *ptr++ = 0;
  *ptr++ = 0;
  bytes_used += 8;

  // Unity matrix
  for (i = 0; i < 9; i++)
  {
    box_type = impeghe_mp4_rev32(unitary_matrix[i]);
    *ptr++ = box_type;
    bytes_used += 4;
  }

  // pre defined
  for (i = 0; i < 6; i++)
  {
    *ptr++ = 0;
    bytes_used += 4;
  }

  // next_track_id
  box_type = impeghe_mp4_rev32(next_track_id);
  *ptr++ = box_type;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mdhd
 *
 *  \brief Write MP4 mdhd data
 *
 *  \param [in,out] pstr_mp4_writer_io Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf Pointer to buffer
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_mdhd(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'m', 'd', 'h', 'd'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  UWORD32 creation_time = 0x0;
  UWORD32 modification_time = 0x0;
  UWORD32 timescale = pstr_mp4_writer_io->meta_info.media_time_scale;
  UWORD32 duration = pstr_mp4_writer_io->meta_info.playTimeInSamples[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  box_type = 0;
  *ptr++ = box_type;
  bytes_used += 4;

  // creation time
  box_type = impeghe_mp4_rev32(creation_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // modification time
  box_type = impeghe_mp4_rev32(modification_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // timescale
  box_type = impeghe_mp4_rev32(timescale);
  *ptr++ = box_type;
  bytes_used += 4;

  // duration
  box_type = impeghe_mp4_rev32(duration);
  *ptr++ = box_type;
  bytes_used += 4;

  // pad 1 bit + lang 5x3 bits + predefined 16 bits
  box_type = impeghe_mp4_rev32(0x55C40000); // undef
  *ptr++ = box_type;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_hdlr
 *
 *  \brief Write MP4 hdlr data
 *
 *  \param [in,out] pstr_mp4_writer_io Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_hdlr(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'h', 'd', 'l', 'r'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version 0
  *ptr++ = 0;
  bytes_used += 4;

  // pre defined = 0
  *ptr++ = 0;
  bytes_used += 4;

  // handler type
  charbuf[0] = 's';
  charbuf[1] = 'o';
  charbuf[2] = 'u';
  charbuf[3] = 'n';
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // reserved
  *ptr++ = 0;
  *ptr++ = 0;
  *ptr++ = 0;
  bytes_used += 12;

  // string name
  charbuf[0] = 'I';
  charbuf[1] = 'T';
  charbuf[2] = 'T';
  charbuf[3] = 'I';
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;
  charbuf[0] = 'A';
  charbuf[1] = 'M';
  charbuf[2] = ' ';
  charbuf[3] = 'M';
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;
  charbuf[0] = 'P';
  charbuf[1] = 'E';
  charbuf[2] = 'G';
  charbuf[3] = 'H';
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stco
 *
 *  \brief Write MP4 STCO data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stco(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'s', 't', 'c', 'o'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  UWORD32 *ia_mp4_stsz_size = pstr_mp4_writer_io->meta_info.ia_mp4_stsz_size;
  WORD32 entry_count = pstr_mp4_writer_io->meta_info.ia_mp4_stsz_entries;
  WORD32 chunk_offset = 0;

  // calculate inital chunck offset
  chunk_offset = (WORD32)(ptr_buf - pstr_mp4_writer_io->ptr_buffer);
  chunk_offset += 24;                                 // ftyp box size
  chunk_offset += 4 + 4;                              // moov box header
  chunk_offset += 4 + 4 + 4 + 4 + (entry_count << 2); // stco box size
  chunk_offset += 4 + 4;                              // mdat box header

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  *ptr++ = 0x0;
  bytes_used += 4;

  // entry_count
  box_type = impeghe_mp4_rev32(entry_count);
  *ptr++ = box_type;
  bytes_used += 4;

  for (i = 0; i < entry_count; i++)
  {
    // chunk offset
    box_type = impeghe_mp4_rev32(chunk_offset);
    *ptr++ = box_type;
    bytes_used += 4;

    chunk_offset += ia_mp4_stsz_size[i];
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stts
 *
 *  \brief Write MP4 stts data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stts(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'s', 't', 't', 's'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD32 entry_count = 1;

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  // entry_count
  box_type = impeghe_mp4_rev32(entry_count);
  *ptr++ = box_type;
  bytes_used += 4;

  for (i = 0; i < entry_count; i++)
  {
    // sample_count
    box_type = impeghe_mp4_rev32(pstr_mp4_writer_io->meta_info.ia_mp4_stsz_entries);
    *ptr++ = box_type;
    bytes_used += 4;

    // sample_delta = 1024 = frame size
    box_type = impeghe_mp4_rev32(1024);
    *ptr++ = box_type;
    bytes_used += 4;
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stsc
 *
 *  \brief Write MP4 stsc data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stsc(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'s', 't', 's', 'c'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD32 entry_count = 1;

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  *ptr++ = 0x0;
  bytes_used += 4;

  // entry_count
  box_type = impeghe_mp4_rev32(entry_count);
  *ptr++ = box_type;
  bytes_used += 4;

  for (i = 0; i < entry_count; i++)
  {
    // first_chunk
    box_type = impeghe_mp4_rev32(0x1);
    *ptr++ = box_type;
    bytes_used += 4;

    // samples_per_chunk
    box_type = impeghe_mp4_rev32(0x1);
    *ptr++ = box_type;
    bytes_used += 4;

    // sample_description_index
    box_type = impeghe_mp4_rev32(0x1);
    *ptr++ = box_type;
    bytes_used += 4;
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stsz
 *
 *  \brief Write MP4 stsz data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stsz(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'s', 't', 's', 'z'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD32 sample_count = pstr_mp4_writer_io->meta_info.ia_mp4_stsz_entries;
  UWORD32 *ia_mp4_stsz_size = pstr_mp4_writer_io->meta_info.ia_mp4_stsz_size;

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  *ptr++ = 0x0;
  bytes_used += 4;

  // sample_size = 0 (non uniform)
  *ptr++ = 0x0;
  bytes_used += 4;

  // sample_count
  box_type = impeghe_mp4_rev32(sample_count);
  *ptr++ = box_type;
  bytes_used += 4;

  // sample sizes (frame sizes)
  for (i = 0; i < sample_count; i++)
  {
    // update mdat size
    pstr_mp4_writer_io->mdat_size += ia_mp4_stsz_size[i];

    // write frame size to buffer
    box_type = impeghe_mp4_rev32(ia_mp4_stsz_size[i]);
    *ptr++ = box_type;
    bytes_used += 4;
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**
 *  impeghe_mp4_write_mael
 *
 *  \brief Brief description
 *
 *  \param pstr_mp4_writer_io
 *  \param ptr_buf
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_mael(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = {'m', 'a', 'e', 'L'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;

  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_mael, pstr_mp4_writer_io->meta_info.mael_length);
  bytes_used += pstr_mp4_writer_io->meta_info.mael_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.mael_length;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**
 *  impeghe_mp4_write_maeg
 *
 *  \brief Brief description
 *
 *  \param pstr_mp4_writer_io
 *  \param ptr_buf
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_maep(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = {'m', 'a', 'e', 'P'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;

  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_maep, pstr_mp4_writer_io->meta_info.maep_length);
  bytes_used += pstr_mp4_writer_io->meta_info.maep_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.maep_length;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**
 *  impeghe_mp4_write_maeS
 *
 *  \brief Brief description
 *
 *  \param pstr_mp4_writer_io
 *  \param ptr_buf
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_maes(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = {'m', 'a', 'e', 'S'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;

  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_maes, pstr_mp4_writer_io->meta_info.maes_length);
  bytes_used += pstr_mp4_writer_io->meta_info.maes_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.maes_length;


  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**
 *  impeghe_mp4_write_maeg
 *
 *  \brief Brief description
 *
 *  \param pstr_mp4_writer_io
 *  \param ptr_buf
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_maeg(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = {'m', 'a', 'e', 'G'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;


  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_maeg, pstr_mp4_writer_io->meta_info.maeg_length);
  bytes_used += pstr_mp4_writer_io->meta_info.maeg_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.maeg_length;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**
 *  impeghe_mp4_write_maeI
 *
 *  \brief Brief description
 *
 *  \param pstr_mp4_writer_io
 *  \param ptr_buf
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_mp4_write_maeI(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = {'m', 'a', 'e', 'I'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // maeI version
  //need to see if its really required
  //*ptr++ = 0x0;
  //bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  box_bytes = impeghe_mp4_write_maeg(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  box_bytes = impeghe_mp4_write_maes(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  box_bytes = impeghe_mp4_write_maep(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  box_bytes = impeghe_mp4_write_mael(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}
/**impeghe_mp4_write_mhaC
 *
 *  \brief Write MP4 mhaC data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */

static WORD32 impeghe_mp4_write_mhaC(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  WORD16 *word16_ptr;
  UWORD8 charbuf[10] = {'m', 'h', 'a', 'C'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD16 mpegh3daConfigLength = pstr_mp4_writer_io->meta_info.mhac_length;

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;

  // config version = 1
  *word8_ptr++ = 0x01;
  bytes_used += 1;

  // mpegh3daProfileLevelIndication
  *word8_ptr++ = pstr_mp4_writer_io->profile_info;
  bytes_used += 1;

  // referenceChannelLayout
  *word8_ptr++ = pstr_mp4_writer_io->spaker_layout;
  bytes_used += 1;

  word16_ptr = (WORD16 *)word8_ptr;

  // mpegh3daConfigLength
  *word16_ptr++ = BYTE_SWAP_UINT16(mpegh3daConfigLength);
  bytes_used += 2;

  word8_ptr = (WORD8 *)word16_ptr;

  // mpegh3daConfig
  // fread(word8_ptr, 1, mpegh3daConfigLength, pstr_mp4_writer_io->fp_raw);
  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr, mpegh3daConfigLength);
  bytes_used += mpegh3daConfigLength;
  word8_ptr += mpegh3daConfigLength;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mhaP
 *
 *  \brief Write MP4 mhaP data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_mhaP(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = { 'm', 'h', 'a', 'P' };
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;
  word8_ptr = (WORD8 *)ptr;

  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_mhaP, pstr_mp4_writer_io->meta_info.mhaP_length);
  bytes_used += pstr_mp4_writer_io->meta_info.mhaP_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.mhaP_length;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mhaD
 *
 *  \brief Write MP4 mhaD data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_mhaD(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr;
  UWORD8 charbuf[10] = { 'm', 'h', 'a', 'D' };
  UWORD32 *data = (UWORD32 *)&charbuf[0];

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;


  // version //flag
  *ptr++ = 0x0;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;

  memcpy(word8_ptr, pstr_mp4_writer_io->ptr_hdr_mhaD, pstr_mp4_writer_io->meta_info.mhaD_length);
  bytes_used += pstr_mp4_writer_io->meta_info.mhaD_length;
  word8_ptr += pstr_mp4_writer_io->meta_info.mhaD_length;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_btrt
 *
 *  \brief Write MP4 btrt data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_btrt(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'b', 't', 'r', 't'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD32 max_frame_data_size = pstr_mp4_writer_io->max_frame_data_size;
  WORD32 total_frame_data_size = pstr_mp4_writer_io->total_frame_data_size;
  WORD32 frame_count = pstr_mp4_writer_io->frame_count;
  WORD32 avg_frame_data_size = total_frame_data_size / frame_count;
  WORD32 sample_rate = pstr_mp4_writer_io->sampling_freq;
  WORD32 frame_lrngth = 1024;
  WORD32 max_bitrate = ((WORD32)(max_frame_data_size / (1024.0 / sample_rate))) << 3;
  WORD32 avg_bitrate = ((WORD32)(avg_frame_data_size / (1024.0 / sample_rate)) << 3);
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // bufferSizeDB
  *ptr++ = impeghe_mp4_rev32(max_frame_data_size);
  bytes_used += 4;

  // maxBitrate
  *ptr++ = impeghe_mp4_rev32(max_bitrate);
  bytes_used += 4;

  // avgBitrate
  *ptr++ = impeghe_mp4_rev32(avg_bitrate);
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_m4ds
 *
 *  \brief Write MP4 m4ds data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_m4ds(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'m', '4', 'd', 's'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // description
  charbuf[0] = 'N';
  charbuf[1] = 'U';
  charbuf[2] = 'L';
  charbuf[3] = 'L';
  *ptr++ = *data;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mha1
 *
 *  \brief Write MP4 mha1 data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_mha1(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'m', 'h', 'a', '1'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD32 sample_rate = pstr_mp4_writer_io->sampling_freq;
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // bytes copied from ref mp4 bitstream TBF
  *ptr++ = 0;//Reserved1
  *ptr++ = impeghe_mp4_rev32(0x1);// DataReferenceIndex
  //(48 bits Reserved1 + 16 bits DataReferenceIndex)

  *ptr++ = 0; //QtVersion(16),QtRevision(16)
  *ptr++ = 0; //QtVendor(32)
  *ptr++ = 0; //channelcount(16), SampleSize(16)
  *ptr++ = 0; //pre_defined(16), reserved(16)
  *ptr++ = impeghe_mp4_rev32(sample_rate << 16);
  bytes_used += 28;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'mhaC' */
  box_bytes = impeghe_mp4_write_mhaC(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'maeI' -- optional */
  if (pstr_mp4_writer_io->maei_present)
  {
    box_bytes = impeghe_mp4_write_maeI(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }

  /* Box type - 'mhaP' -- optional */
  if (pstr_mp4_writer_io->mhaP_data_present)
  {
    box_bytes = impeghe_mp4_write_mhaP(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }

  /* Box type - 'mhaD' -- optional */
  if (pstr_mp4_writer_io->mhaD_data_present)
  {
    box_bytes = impeghe_mp4_write_mhaD(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }

  /* Box type - 'btrt' -- optional */
  box_bytes = impeghe_mp4_write_btrt(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'm4ds' -- optional */
  box_bytes = impeghe_mp4_write_m4ds(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mhm1
 *
 *  \brief Write MP4 mhm1 data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_mhm1(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'m', 'h', 'm', '1'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  WORD8 *word8_ptr ;
  WORD32 sample_rate = pstr_mp4_writer_io->sampling_freq;
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // bytes copied from ref mp4 bitstream TBF
  *ptr++ = 0;//Reserved1
  *ptr++ = impeghe_mp4_rev32(0x1);// DataReferenceIndex
  //(48 bits Reserved1 + 16 bits DataReferenceIndex)

  *ptr++ = 0; //QtVersion(16),QtRevision(16)
  *ptr++ = 0; //QtVendor(32)
  *ptr++ = 0; //channelcount(16), SampleSize(16)
  *ptr++ = 0; //pre_defined(16), reserved(16)
  *ptr++ = impeghe_mp4_rev32(sample_rate << 16);
  bytes_used += 28;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'mhaC' -- optional */
  box_bytes = impeghe_mp4_write_mhaC(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;
  /* Box type - 'mhaeI' -- optional */
  if (pstr_mp4_writer_io->maei_present)
  {
    box_bytes = impeghe_mp4_write_maeI(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }



  /* Box type - 'mhaD' -- optional */
  if (pstr_mp4_writer_io->mhaD_data_present)
  {
    box_bytes = impeghe_mp4_write_mhaD(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }

  /* Box type - 'btrt' -- optional */
  box_bytes = impeghe_mp4_write_btrt(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'm4ds' -- optional */
  box_bytes = impeghe_mp4_write_m4ds(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'mhaP' -- optional */
  if (pstr_mp4_writer_io->mhaP_data_present)
  {
    box_bytes = impeghe_mp4_write_mhaP(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }
  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_smhd
 *
 *  \brief Write MP4 smhd data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_smhd(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'s', 'm', 'h', 'd'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version 0
  *ptr++ = 0;
  bytes_used += 4;

  // 16 bit balance and 16 bit reserved
  *ptr++ = 0;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_url
 *
 *  \brief Write MP4 url data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_url(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'u', 'r', 'l', ' '};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // taken from ref mp4 TBA
  box_type = impeghe_mp4_rev32(0x1);
  *ptr++ = box_type;
  bytes_used += 4;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_dref
 *
 *  \brief Write MP4 dref data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_dref(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'d', 'r', 'e', 'f'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  *ptr++ = 0x0;
  bytes_used += 4;

  // no of elements
  box_type = impeghe_mp4_rev32(0x1);
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'url ' */
  box_bytes = impeghe_mp4_write_url(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_tkhd
 *
 *  \brief Write MP4 tkhd data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_tkhd(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  UWORD8 charbuf[10] = {'t', 'k', 'h', 'd'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  UWORD32 creation_time = 0x0;
  UWORD32 modification_time = 0x0;
  UWORD32 track_id = 0x1;
  UWORD32 duration = pstr_mp4_writer_io->meta_info.playTimeInSamples[0];
  UWORD16 volume = 0x0100; // typically, full volume
  WORD32 unitary_matrix[9] = {0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000};

  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // flags - default values is 7
  box_type = impeghe_mp4_rev32(0x7);
  *ptr++ = box_type;
  bytes_used += 4;

  // creation time
  box_type = impeghe_mp4_rev32(creation_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // modification time
  box_type = impeghe_mp4_rev32(modification_time);
  *ptr++ = box_type;
  bytes_used += 4;

  // track_id
  box_type = impeghe_mp4_rev32(track_id);
  *ptr++ = box_type;
  bytes_used += 4;

  // reserved
  box_type = 0x0;
  *ptr++ = box_type;
  bytes_used += 4;

  // duration
  box_type = impeghe_mp4_rev32(duration);
  *ptr++ = box_type;
  bytes_used += 4;

  // reserved
  *ptr++ = 0;
  *ptr++ = 0;
  bytes_used += 8;

  // layer + alternate group
  *ptr++ = 0;
  bytes_used += 4;

  // volume + 16 bits reserved
  box_type = impeghe_mp4_rev32((WORD32)volume);
  *ptr++ = box_type;
  bytes_used += 4;

  // Unity matrix
  for (i = 0; i < 9; i++)
  {
    box_type = impeghe_mp4_rev32(unitary_matrix[i]);
    *ptr++ = box_type;
    bytes_used += 4;
  }

  // width & height
  *ptr++ = 0;
  *ptr++ = 0;
  bytes_used += 8;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_elst
 *
 *  \brief Write MP4 elst data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_elst(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, i;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD32 entry_count = 1;
  UWORD8 charbuf[10] = {'e', 'l', 's', 't'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // version
  *ptr++ = 0x0;
  bytes_used += 4;

  // entry_count
  box_type = impeghe_mp4_rev32(entry_count);
  *ptr++ = box_type;
  bytes_used += 4;

  for (i = 0; i < entry_count; i++)
  {
    // segement_duration
    box_type = impeghe_mp4_rev32(pstr_mp4_writer_io->meta_info.playTimeInSamples[i]);
    *ptr++ = box_type;
    bytes_used += 4;

    // media_time
    box_type = impeghe_mp4_rev32(pstr_mp4_writer_io->meta_info.startOffsetInSamples[i]);
    *ptr++ = box_type;
    bytes_used += 4;

    // media_rate
    *ptr++ = 0x0;
    bytes_used += 4;
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_edts
 *
 *  \brief Write MP4 edts data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_edts(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'e', 'd', 't', 's'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type
  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'elst' */
  box_bytes = impeghe_mp4_write_elst(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_dinf
 *
 *  \brief Write MP4 dinf data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_dinf(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'d', 'i', 'n', 'f'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'dref' */
  box_bytes = impeghe_mp4_write_dref(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stsd
 *
 *  \brief Write MP4 stsd data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stsd(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'s', 't', 's', 'd'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  // 8 bytes copied from ref mp4 bitstream TBF
  *ptr++ = 0;
  *ptr++ = impeghe_mp4_rev32(0x1);
  bytes_used += 8;

  word8_ptr = (WORD8 *)ptr;
  if (pstr_mp4_writer_io->is_mhm1)
  {
    /* Box type - 'mhm1' op_fmt 3 */
    box_bytes = impeghe_mp4_write_mhm1(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }
  else
  {
    /* Box type - 'mha1' op_fmt 2 */
    box_bytes = impeghe_mp4_write_mha1(pstr_mp4_writer_io, word8_ptr);
    word8_ptr += box_bytes;
    bytes_used += box_bytes;
  }

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_stbl
 *
 *  \brief Write MP4 stbl data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_stbl(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'s', 't', 'b', 'l'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;
  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'stts' */
  box_bytes = impeghe_mp4_write_stts(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'stsd' */
  box_bytes = impeghe_mp4_write_stsd(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'stsz' */
  box_bytes = impeghe_mp4_write_stsz(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'stsc' */
  box_bytes = impeghe_mp4_write_stsc(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'stco' */
  box_bytes = impeghe_mp4_write_stco(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;

  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_minf
 *
 *  \brief Write MP4 minf data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_minf(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'m', 'i', 'n', 'f'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'smhd' */
  box_bytes = impeghe_mp4_write_smhd(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'dinf' */
  box_bytes = impeghe_mp4_write_dinf(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'stbl' */
  box_bytes = impeghe_mp4_write_stbl(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_mdia
 *
 *  \brief Write MP4 mdia data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_mdia(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'m', 'd', 'i', 'a'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'mdhd' */
  box_bytes = impeghe_mp4_write_mdhd(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'hdlr' */
  box_bytes = impeghe_mp4_write_hdlr(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'minf' */
  box_bytes = impeghe_mp4_write_minf(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_trak
 *
 *  \brief Write MP4 trak data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in,out] ptr_buf	Pointer to buffer
 *
 *  \return WORD32
 */
static WORD32 impeghe_mp4_write_trak(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD8 *ptr_buf)
{
  WORD32 box_type, box_bytes;
  WORD32 bytes_used = 0;
  WORD32 *ptr = (WORD32 *)ptr_buf;
  WORD8 *word8_ptr ;
  UWORD8 charbuf[10] = {'t', 'r', 'a', 'k'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  // box size
  *ptr++ = 0x0;
  bytes_used += 4;

  // box type

  box_type = *data;
  *ptr++ = box_type;
  bytes_used += 4;

  word8_ptr = (WORD8 *)ptr;
  /* Box type - 'tkhd' */
  box_bytes = impeghe_mp4_write_tkhd(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'edts' */
  box_bytes = impeghe_mp4_write_edts(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  /* Box type - 'mdia' */
  box_bytes = impeghe_mp4_write_mdia(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += box_bytes;
  bytes_used += box_bytes;

  // update box size
  ptr = (WORD32 *)ptr_buf;
  box_type = impeghe_mp4_rev32(bytes_used);
  *ptr = box_type;

  return bytes_used;
}

/**impeghe_mp4_write_moov
 *
 *  \brief Write MP4 moov data
 *
 *  \param [in,out] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *
 *  \return VOID
 */
static VOID impeghe_mp4_write_moov(ia_mp4_writer_struct *pstr_mp4_writer_io)
{
  WORD32 box_size, box_type;
  WORD32 bytes_used = 0;
  UWORD8 charbuf[10] = {'m', 'o', 'o', 'v'};
  UWORD32 *data = (UWORD32 *)&charbuf[0];
  /* malloc for 'moov' box size plus 'stsz' and  'stco' size */
  WORD8 *word8_ptr = pstr_mp4_writer_io->ptr_buffer =
      malloc(MAX_MOOV_BOX_SIZE + ((4 + 4) * pstr_mp4_writer_io->meta_info.ia_mp4_stsz_entries));
  /* Box type - 'mvhd' */
  bytes_used += impeghe_mp4_write_mvhd(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += bytes_used;

  /* Box type - 'trak' */
  bytes_used += impeghe_mp4_write_trak(pstr_mp4_writer_io, word8_ptr);
  word8_ptr += bytes_used;
  // Box size
  box_size = impeghe_mp4_rev32(bytes_used + 8);
  fwrite(&box_size, 4, 1, pstr_mp4_writer_io->fp_mp4);

  // Box type
  box_type = *data;
  fwrite(&box_type, 4, 1, pstr_mp4_writer_io->fp_mp4);
  if (pstr_mp4_writer_io->ptr_buffer != NULL)
  {
    fwrite(pstr_mp4_writer_io->ptr_buffer, 1, bytes_used, pstr_mp4_writer_io->fp_mp4);
  }
  free(pstr_mp4_writer_io->ptr_buffer);
  pstr_mp4_writer_io->ptr_buffer = NULL;
}

/**impeghe_mp4_writer
 *
 *  \brief MP4 write main
 *
 *  \param [in] pstr_mp4_writer_io	Pointer to ia_mp4_writer_struct structure
 *  \param [in] dummy_write	Dummy write
 *
 *  \return WORD32
 */
WORD32 impeghe_mp4_writer(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD32 dummy_write)
{

  /* init */
  pstr_mp4_writer_io->mdat_size = 0;

  /* Box type - 'ftyp' */
  impeghe_mp4_write_ftyp(pstr_mp4_writer_io);

  /* Box type - 'moov' */
  impeghe_mp4_write_moov(pstr_mp4_writer_io);

  /* Box type - 'mdat' - only box header */
  impeghe_mp4_write_mdat(pstr_mp4_writer_io);

  return 0;
}
