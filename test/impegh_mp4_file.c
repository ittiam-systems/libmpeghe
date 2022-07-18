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
#include "impegh_type_def.h"
#include "impegh_mp4_file.h"
#include "impegh_mp4_utils.h"
#include "impegh_demux_error.h"

/**impegh_mp4_fopen
 *
 *  \brief Function for opening file/buffer
 *
 *  \param [in] file_name	Pointer to file name
 *  \param [in] with_file	Flag for mhas file type
 *  \param [in] size	Size
 *
 *  \return it_avi_file_ctxt
 */
it_avi_file_ctxt *impegh_mp4_fopen(pUWORD8 file_name, UWORD8 with_file, WORD32 size)
{
  it_avi_file_ctxt *it_file =
      (it_avi_file_ctxt *)impegh_mp4_malloc_wrapper(sizeof(it_avi_file_ctxt));
  it_file->with_file = with_file;
  if (it_file->with_file)
  {
    it_file->fp = fopen((void *)file_name, "rb");
    if (it_file->fp)
      return it_file;
    else
    {
      impegh_mp4_free_wrapper((pVOID)it_file);
      return NULL;
    }
  }
  else
  {
    it_file->buf = (pWORD8)file_name;
    it_file->pos = (pWORD8)file_name;
    it_file->size = size;
    if (it_file->buf)
      return it_file;
    else
    {
      impegh_mp4_free_wrapper(it_file);
      return NULL;
    }
  }
}

/**impegh_mp4_fread
 *
 *  \brief Function for opening file/buffer
 *
 *  \param [in] buffer	VOID pointer
 *  \param [in] size	Size
 *  \param [in] count	Count
 *  \param [in,out] itf	Pointer to it_avi_file_ctxt structure
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_fread(pVOID buffer, WORD32 size, WORD32 count, it_avi_file_ctxt *itf)
{
  if (itf->with_file)
  {
    return (WORD32)fread(buffer, size, count, itf->fp);
  }
  else
  {
    WORD32 avail = itf->size - (WORD32)(itf->pos - itf->buf);
    WORD32 toread = size * count;
    WORD32 allowed = (toread < avail) ? toread : avail;

    if (allowed < 0)
      return -1;
    memcpy(buffer, itf->pos, allowed);
    itf->pos += allowed;
    return allowed / size;
  }
}

/**impegh_mp4_fseek
 *
 *  \brief Function for seeking in file/buffer
 *
 *  \param [in,out] itf Pointer to it_avi_file_ctxt structure
 *  \param [in] offset	Offset
 *  \param [in] origin	Origin
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_fseek(it_avi_file_ctxt *itf, WORD32 offset, WORD32 origin)
{
  if (itf->with_file)
  {
    if (!((offset == 0) && (origin == SEEK_CUR)))
      return fseek(itf->fp, offset, origin);
    else
      return (0);
  }
  else
  {
    switch (origin)
    {
    case SEEK_CUR:
      itf->pos += offset;
      break;
    case SEEK_END:
      itf->pos = itf->buf + itf->size + offset;
      break;
    case SEEK_SET:
      itf->pos = itf->buf + offset;
      break;
    }
    return 0;
  }
}

/**impegh_mp4_fclose
 *
 *  \brief Function for closing file/buffer
 *
 *  \param [in,out] itf	Pointer to it_avi_file_ctxt structure
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_fclose(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
  {
    if (fclose(itf->fp))
      return -1;
    impegh_mp4_free_wrapper((pVOID)itf);
    return -1;
  }
  else
  {
    itf->buf = NULL;
    itf->pos = NULL;
    itf->size = 0;
    return 0;
  }
}

/**impegh_mp4_ftell
 *
 *  \brief Function for geting current position in file/buffer
 *
 *  \param [in,out] itf	Pointer to it_avi_file_ctxtstructure
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_ftell(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
    return (WORD32)ftell(itf->fp);
  else
    return (WORD32)(itf->pos - itf->buf);
}

/**impegh_mp4_feof
 *
 *  \brief Function to check if end of file is reached
 *
 *  \param [in,out] itf	Pointer to it_avi_file_ctxt structure
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_feof(it_avi_file_ctxt *itf)
{
  if (itf->with_file)
    return feof(itf->fp);
  else
    return !(itf->pos - (itf->buf + itf->size));
}

/**impegh_mp4_fread_buf
 *
 *  \brief Function to read into buffer
 *
 *  \param [in] buffer  Pointer to buffer
 *  \param [in] size  Buffer size
 *  \param [in] count Count
 *  \param [in,out] itf Pointer to it_avi_file_ctxt structure
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_fread_buf(WORD32 **buffer, WORD32 size, WORD32 count, it_avi_file_ctxt *itf)
{
  return impegh_mp4_fread(*buffer, size, count, itf);
}

/**impegh_mp4_find_mdat
 *
 *  \brief Function to find mdat
 *
 *  \param [in,out] itf Pointer to it_avi_file_ctxt structure
 *  \param [in] offset  Offset
 *  \param [in,out] mdat_size	Pointer to mdat size
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_find_mdat(it_avi_file_ctxt *itf, WORD32 *offset, WORD32 *mdat_size)
{
  WORD8 buf_size[4];
  WORD8 buf_test;
  WORD32 bytes_test;
  UWORD32 *data_size = (UWORD32 *)&buf_size[0];
  *mdat_size = 0;
  while (!feof(itf->fp))
  {
    bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
    if (buf_test == 'm')
    {
      bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
      if (buf_test == 'd')
      {
        bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 'a')
          bytes_test = impegh_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 't')
        {
          impegh_mp4_fseek(itf, -8, SEEK_CUR);
          bytes_test = impegh_mp4_fread(buf_size, 1, 4, itf);
          if (bytes_test < 4)
          {
            *mdat_size = -1;
            return IMPEGH_DEMUX_MDAT_FIND;
          }
          *mdat_size = impegh_mp4_rev32(*data_size);
          bytes_test = impegh_mp4_fread(buf_size, 1, 4, itf);
          *offset = ftell(itf->fp);
          *mdat_size = *mdat_size - 8;
          return IA_NO_ERROR;
        }
      }
      if (buf_test == 'm')
        impegh_mp4_fseek(itf, -1, SEEK_CUR);
    }
  }
  return IA_NO_ERROR;
}

/**impegh_mp4_read_mdat
 *
 *  \brief Function to read mdat
 *
 *  \param [in] itf Pointer to it_avi_file_ctxt structure
 *  \param [in] buffer	Pointer to buffer
 *  \param [in] buf_size Buffer size
 *  \param [out] size Pointer to size
 *  \param [out] length Pointer to length
 *  \param [out] offset Pointer to offset
 *
 *  \return WORD32
 */
WORD32 impegh_mp4_read_mdat(it_avi_file_ctxt *itf, pUWORD8 buffer, WORD32 buf_size, WORD32 *size,
                            UWORD32 *length, WORD32 *offset)
{
  if (*size >= 0)
  {
    buf_size = (*size < buf_size) ? *size : buf_size;
    *length = (UWORD32)fread(buffer, 1, buf_size, itf->fp);
    *size -= *length;
    *offset = ftell(itf->fp);
  }
  return 0;
}
