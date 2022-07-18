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

#ifndef IMPEGHE_MP4_WRITER_H
#define IMPEGHE_MP4_WRITER_H

#define BYTE_SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))
#define BYTE_SWAP_UINT32(x)                                                                      \
  (((x) >> 24) | (((x)&0x00FF0000) >> 8) | (((x)&0x0000FF00) << 8) | (WORD32)(((WORD64)x) << 24))

#define MAX_HDR_LEN 2000
#define MAX_MOOV_BOX_SIZE 2000
#define MAX_TRACKS_PER_LAYER 50

typedef struct
{
  UWORD32 *ia_mp4_stsz_size;
  UWORD32 ia_mp4_stsz_entries;
  UWORD32 fill_once;
  UWORD32 movie_time_scale;
  UWORD32 media_time_scale;

  UWORD32 dec_info_init;
  UWORD32 maeg_length;
  UWORD32 maes_length;
  UWORD32 maep_length;
  UWORD32 mael_length;
  UWORD32 g_track_count;
  UWORD32 useEditlist[MAX_TRACKS_PER_LAYER];
  UWORD32 startOffsetInSamples[MAX_TRACKS_PER_LAYER];
  UWORD32 playTimeInSamples[MAX_TRACKS_PER_LAYER];

} metadata_info;

typedef struct ia_mp4_writer_struct
{
  FILE *fp_mp4;
  WORD8 ptr_hdr[MAX_HDR_LEN];
  WORD8 ptr_hdr_maeg[MAX_HDR_LEN];
  WORD8 ptr_hdr_maes[MAX_HDR_LEN];
  WORD8 ptr_hdr_maep[MAX_HDR_LEN];
  WORD8 ptr_hdr_mael[MAX_HDR_LEN];
  WORD8 *ptr_buffer;
  WORD64 mdat_size;
  metadata_info meta_info;
  WORD32 is_mhm1;
  WORD32 maei_present;
  UWORD32 profile_info;
} ia_mp4_writer_struct;

WORD32 impeghe_mp4_writer(ia_mp4_writer_struct *pstr_mp4_writer_io, WORD32 dummy_write);

#endif /* IMPEGHE_MP4_WRITER_H */