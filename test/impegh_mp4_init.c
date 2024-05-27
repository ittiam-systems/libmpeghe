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
#include <string.h>
#include "impegh_type_def.h"
#include "impegh_mp4_file.h"
#include "impegh_mp4_utils.h"
#include "impegh_mp4_odf.h"
#include "impegh_mp4_defs.h"
#include "impegh_mp4_atoms.h"
#include "impegh_mp4_object_type.h"
#include "impegh_mp4_proto.h"
#include "impegh_mp4_init.h"
#include "impegh_mp4_tags.h"
#include "impegh_mp4_demux_utils.h"
extern WORD32 g_mhm1_tag;

/**impegh_mp4_init_wrap
 *
 *  \brief Total initialization of MP4 structures
 *
 *  \param [in,out] m	Pointer to mp4_info structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_init_wrap(mp4_info *m)
{
  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 ret, cont = 1;
  ia_mp4_trak_init *n_temp;
  UWORD8 track_count = 0;
  m->udta_info = NULL;
  error = impegh_mp4_init(m);
  if (error)
  {
    return IT_ERROR;
  }
  m->imp_trak_info[0] =
      (trak_info *)impegh_mp4_mem_node_calloc(&(m->ptr_mem), 1, sizeof(trak_info));

  cont = 1;
  n_temp = m->trak_init_info;
  do
  {
    ret = impegh_mp4_get_needed_trak(m->imp_trak_info[0], &n_temp, IT_MP4V);
    impegh_mp4_clear_buffer(m->imp_trak_info[0]);
    if (ret == IT_OK)
    {
      ret = impegh_mp4_fill_imp_trak_info(m->fp, m->imp_trak_info[0], IT_ALL, &(m->ptr_mem),
                                          &m->st_maei_info);

      if (ret == IT_OK)
      {
        if (m->imp_trak_info[0]->sample_type == IT_MJPA)
        {
          cont = 0;
        }
        else if (CHECK_VIDEO(m->imp_trak_info[0]
                                 ->stsd_atom->ptr_sample_entry->es.es_descr->dec_config_desc
                                 .obj_type_indication))
        {
          cont = 0;
        }
        else
        {
          n_temp = n_temp->next;
        }
      }
      else
      {
        n_temp = n_temp->next;
      }
      m->imp_trak_info[0]->samples_last_stsc_entry =
          m->imp_trak_info[0]
              ->stsc_entries[m->imp_trak_info[0]->read_stsc_entries]
              .samples_per_chunk;
      m->imp_trak_info[0]->read_stsc_entries++;
    }
  } while (ret == IT_OK && cont && n_temp);

  if (ret != IT_OK || !n_temp)
  {
    impegh_mp4_free_mem_node(m->imp_trak_info[0], &(m->ptr_mem));
    m->imp_trak_info[0] = NULL;
  }

  m->imp_trak_info[1] =
      (trak_info *)impegh_mp4_mem_node_calloc(&(m->ptr_mem), 1, sizeof(trak_info));
  cont = 1;
  n_temp = m->trak_init_info;
  do
  {
    ret = impegh_mp4_get_needed_trak(m->imp_trak_info[1], &n_temp, IT_MP4A);
    impegh_mp4_clear_buffer(m->imp_trak_info[1]);
    if (ret == IT_OK)
    {
      ret = impegh_mp4_fill_imp_trak_info(m->fp, m->imp_trak_info[1], IT_ALL, &(m->ptr_mem),
                                          &m->st_maei_info);
      if (ret != IT_OK)
        break;
      if (ret == IT_OK)
        cont = 0;
      else
        n_temp = n_temp->next;
      m->imp_trak_info[1]->samples_last_stsc_entry =
          m->imp_trak_info[1]
              ->stsc_entries[m->imp_trak_info[1]->read_stsc_entries]
              .samples_per_chunk;
      m->imp_trak_info[1]->read_stsc_entries++;
    }
  } while (ret == IT_OK && cont && n_temp);

  if (ret != IT_OK || !n_temp)
  {
    impegh_mp4_free_mem_node(m->imp_trak_info[1], &(m->ptr_mem));
    m->imp_trak_info[1] = NULL;
  }
  if (m->imp_trak_info[0])
    track_count++;
  if (m->imp_trak_info[1])
    track_count++;
  if (track_count)
    return IT_OK;
  else
    return IT_ERROR;
}

/**impegh_mp4_read_header_info
 *
 *  \brief Gives header information
 *
 *  \param [in] buf	Pointer to buffer
 *  \param [in] m	Pointer to trak_info structure
 *
 *  \return UWORD32
 */
UWORD32 impegh_mp4_read_header_info(pUWORD8 buf, trak_info *m)
{
  if (m)
  {
    memcpy(buf, m->dec_info, m->dec_info_length);
    return (m->dec_info_length);
  }
  else
  {
    return 0;
  }
}

/**impegh_mp4_init
 *
 *  \brief Initialize MP4 structures from the input file
 *
 *  \param [in] m Pointer to mp4_info structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_init(mp4_info *m)
{
  WORD32 ret;
  UWORD32 moov_atom, size;
  UWORD8 charbuf[5], skip;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  ia_mp4_node *temp = NULL;
  ia_mp4_movie_header_atom *movie_header = NULL;
  moov_atom = 0;
  size = 0;
  skip = 1;
  while (!impegh_mp4_feof(m->fp))
  {
    ret = impegh_mp4_fread(charbuf, 1, 4, m->fp);
    if (ret < 4)
      break;
    size = impegh_mp4_rev32(*data_size);
    if (size <= 0)
      return IT_ERROR;
    ret = impegh_mp4_fread(charbuf, 1, 4, m->fp);
    if (ret < 4)
      break;
    moov_atom = impegh_mp4_rev32(*data_size);
    if (moov_atom == IT_MOOV)
    {
      ret = impegh_mp4_fseek(m->fp, -8, SEEK_CUR);
      if (ret < 0)
        return IT_ERROR;
      skip = 0;
      break;
    }
    ret = impegh_mp4_fseek(m->fp, size - 8, SEEK_CUR);
    if (ret < 0)
      break;
  }

  if (skip)
    ret = impegh_mp4_fseek(m->fp, 0, SEEK_SET);

  if ((impegh_mp4_find_str_file(&(m->fp), (pWORD8) "moov")) != IT_ERROR)
  {
    ret = impegh_mp4_fseek(m->fp, -8, SEEK_CUR);
    m->ptr_mem = NULL;
    m->trak_init_info = NULL;
    m->root = (ia_mp4_node *)impegh_mp4_mem_node_malloc(&(m->ptr_mem), sizeof(ia_mp4_node));
    if (impegh_mp4_read_atom(m, &(m->fp), &(m->root), &(m->trak_init_info), &(m->ptr_mem),
                             IT_NULL) != IT_OK)
    {
      return IT_ERROR;
    }
  }
  else
  {
    return IT_ERROR;
  }

  temp = m->root->child;
  while (temp)
  {
    if (*(temp->descr) == 'm')
      break;
    temp = temp->sibling;
  }
  if (temp)
  {
    movie_header = (ia_mp4_movie_header_atom *)temp->data;
    switch (movie_header->version)
    {
    case 1:
      m->time_scale = movie_header->uinfo.version1.time_scale;
      m->duration = (WORD32)movie_header->uinfo.version1.duration;
      break;
    default:
      m->time_scale = movie_header->uinfo.version0.time_scale;
      m->duration = movie_header->uinfo.version0.duration;
      break;
    }
  }
  else
  {
    return IT_ERROR;
  }

  return IT_OK;
}

/**impegh_mp4_read_atom
 *
 *  \brief Reads MP4 Atoms from the input MP4 file.
 *
 *  \param [in,out] m_info	Pointer to mp4_info structure
 *  \param [in,out] fp	VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_node structure
 *  \param [in,out] p	Pointer to ia_mp4_trak_init structure
 *  \param [in,out] m	Pointer to ia_mp4_mem_node structure
 *  \param [in] prev_state	Previous state
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_read_atom(mp4_info *m_info, VOID **fp, ia_mp4_node **n,
                                  ia_mp4_trak_init **p, ia_mp4_mem_node **m, UWORD32 prev_state)
{
  UWORD32 i, j, ret, type, handler_type, size = 0;
  UWORD8 charbuf[10], version;
  ia_mp4_atom *a = NULL;
  UWORD32 *sptr = NULL, ref_type;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];
  ia_mp4_node *next_child = NULL;

  a = (ia_mp4_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_atom));

  if (impegh_mp4_feof(*fp))
    return IT_ERROR;

  ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
  if (ret < 4)
    return IT_ERROR;
  a->size = impegh_mp4_rev32(*data_size);
  size = a->size;
  if (size == 0)
    return IT_ERROR;
  size -= ret;

  ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  a->type = impegh_mp4_rev32(*data_size);
  type = a->type;
  size -= ret;
  if (a->size == 1)
  {
    ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
    if (ret < 8)
    {
      return IT_ERROR;
    }
    a->large_size = impegh_mp4_rev32(*data_size);
    size = (UWORD32)a->large_size;
    size -= ret;
  }

  if (a->type == IT_MOOV)
  {
    if (prev_state != IT_NULL)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    (*n)->data = (pVOID)a;
    strcpy((*n)->descr, "moov");
    (*n)->parent = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));

    next_child = (*n)->child;
    next_child->parent = (*n);
    while (impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_MOOV) == IT_OK)
    {
      next_child->sibling = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
      next_child->sibling->parent = next_child->parent;
      next_child->sibling->prev = next_child;
      next_child = next_child->sibling;
    }
    next_child->prev->sibling = NULL;
    impegh_mp4_free_mem_node(next_child, m);
    return IT_OK;
  }

  else if (a->type == IT_MVHD)
  {
    ia_mp4_movie_header_atom *fa = (ia_mp4_movie_header_atom *)impegh_mp4_mem_node_malloc(
        m, sizeof(ia_mp4_movie_header_atom));
    if (prev_state != IT_MOOV)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }

    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);

    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    switch (fa->version)
    {
    case 1:
      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->uinfo.version1.creation_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->uinfo.version1.modification_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->uinfo.version1.time_scale = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
        return IT_ERROR;
      fa->uinfo.version1.duration = impegh_mp4_rev32(*data_size);

      break;
    default:
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->uinfo.version0.creation_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->uinfo.version0.modification_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->uinfo.version0.time_scale = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->uinfo.version0.duration = impegh_mp4_rev32(*data_size);

      break;
    }
    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->reserved1 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->reserved2 = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->reserved3 = impegh_mp4_rev16(*data_size_16);

    for (i = 0; i < 2; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->reserved4[i] = impegh_mp4_rev32(*data_size);
    }

    for (i = 0; i < 9; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->reserved5[i] = impegh_mp4_rev32(*data_size);
    }

    for (i = 0; i < 6; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->reserved6[i] = impegh_mp4_rev32(*data_size);
    }

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->next_track_ID = impegh_mp4_rev32(*data_size);

    if ((*n) == NULL)
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "mvhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_IODS)
  {
    ia_mp4_obj_desc_atom *fa =
        (ia_mp4_obj_desc_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_obj_desc_atom));
    if (prev_state != IT_MOOV)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);
    fa->iod.tag = 0;
    fa->od.tag = 0;
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->iod.tag = charbuf[0];

    if (fa->iod.tag == MP4_IOD_TAG)
    {
      impegh_mp4_get_iod(fp, &(fa->iod), m);
    }
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", fa->type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "iods");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }

  else if (a->type == IT_TRAK)
  {
    if (prev_state != IT_MOOV)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    if (*p == NULL)
    {
      (*p) = (ia_mp4_trak_init *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_trak_init));
      (*p)->udta_info = NULL;
      (*p)->next = NULL;
    }
    else
    {
      ia_mp4_trak_init *temp =
          (ia_mp4_trak_init *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_trak_init));
      temp->next = (*p);
      (*p) = temp;
      (*p)->udta_info = NULL;
    }
    m_info->trak_end_offset = impegh_mp4_ftell(*fp) + size;
    (*n)->data = (pVOID)a;
    strcpy((*n)->descr, "trak");
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    next_child = (*n)->child;
    next_child->parent = (*n);
    while ((impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_TRAK)) == IT_OK)
    {
      next_child->sibling = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
      next_child->sibling->parent = next_child->parent;
      next_child->sibling->prev = next_child;
      next_child = next_child->sibling;
    }
    next_child->prev->sibling = NULL;
    impegh_mp4_free_mem_node(next_child, m);
    return IT_OK;
  }

  else if (a->type == IT_TKHD)
  {
    ia_mp4_trak_header_atom *fa =
        (ia_mp4_trak_header_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_trak_header_atom));
    if (prev_state != IT_TRAK)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    (*p)->trak_type = IT_NONE;
    (*p)->handler_type = IT_NONE;
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    version = (UWORD8)charbuf[0];
    fa->version = (UWORD8)charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    switch (version)
    {
    case 1:
      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->unfo.version1.creation_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->unfo.version1.modification_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*p)->trak_id = impegh_mp4_rev32(*data_size);

      fa->unfo.version1.track_id = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->unfo.version1.creserved = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->unfo.version1.duration = impegh_mp4_rev32(*data_size);

      break;

    default:

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->unfo.version0.creation_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->unfo.version0.modification_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*p)->trak_id = impegh_mp4_rev32(*data_size);

      fa->unfo.version0.track_id = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->unfo.version0.creserved = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->unfo.version0.duration = impegh_mp4_rev32(*data_size);

      break;
    }

    for (i = 0; i < 3; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->reserved1[i] = impegh_mp4_rev32(*data_size);
    }

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->reserved2 = impegh_mp4_rev16(*data_size_16);
    if (fa->reserved2)
      (*p)->trak_type = IT_MP4A;

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->reserved3 = impegh_mp4_rev16(*data_size_16);

    for (i = 0; i < 9; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->reserved4[i] = impegh_mp4_rev32(*data_size);
    }

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->reserved5 = impegh_mp4_rev32(*data_size);
    if (fa->reserved5)
      (*p)->trak_type = IT_MP4V;

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->reserved6 = impegh_mp4_rev32(*data_size);

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "tkhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }

  else if (a->type == IT_EDTS)
  {
    if (prev_state != IT_TRAK)
    {
      UWORD8 atom_type[4];
      WORD32 word_value;
      WORD32 total_read = 0;
      total_read = impegh_mp4_fread(&size, 1, 4, *fp);
      total_read += impegh_mp4_fread(&atom_type, 1, 4, *fp);
      if (atom_type[0] == 'e' && atom_type[1] == 'l' && atom_type[2] == 's' &&
          atom_type[3] == 't')
      {
        total_read += impegh_mp4_fread(&atom_type[0], 1, 4, *fp);
        total_read += impegh_mp4_fread(&atom_type[0], 1, 4, *fp);
        total_read += impegh_mp4_fread(&atom_type[0], 1, 4, *fp);
        word_value = atom_type[0];
        word_value = (word_value << 8) + atom_type[1];
        word_value = (word_value << 8) + atom_type[2];
        word_value = (word_value << 8) + atom_type[3];
        total_read += impegh_mp4_fread(&atom_type[0], 4, 1, *fp);
        word_value = atom_type[0];
        word_value = (word_value << 8) + atom_type[1];
        word_value = (word_value << 8) + atom_type[2];
        word_value = (word_value << 8) + atom_type[3];
        total_read += impegh_mp4_fread(&atom_type[0], 4, 1, *fp);
        word_value = atom_type[0];
        word_value = (word_value << 8) + atom_type[1];
        word_value = (word_value << 8) + atom_type[2];
        word_value = (word_value << 8) + atom_type[3];
      }
      impegh_mp4_fseek(*fp, -total_read, SEEK_CUR);

      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    impegh_mp4_fseek(*fp, size, SEEK_CUR);
    return IT_OK;
  }

  else if (a->type == IT_MDIA)
  {
    if (prev_state != IT_TRAK)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    (*n)->data = (void *)a;
    strcpy((*n)->descr, "mdia");
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    next_child = (*n)->child;
    next_child->parent = (*n);
    while ((impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_MDIA)) == IT_OK)
    {
      next_child->sibling = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
      next_child->sibling->parent = next_child->parent;
      next_child->sibling->prev = next_child;
      next_child = next_child->sibling;
    }
    next_child->prev->sibling = NULL;
    impegh_mp4_free_mem_node(next_child, m);
    return IT_OK;
  }

  else if (a->type == IT_MDHD)
  {
    ia_mp4_media_header_atom *fa = (ia_mp4_media_header_atom *)impegh_mp4_mem_node_malloc(
        m, sizeof(ia_mp4_media_header_atom));
    if (prev_state != IT_MDIA)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }

    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    version = charbuf[0];
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    switch (version)
    {
    case 1:
      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->ufo.version1.creation_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      fa->ufo.version1.modification_time = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*p)->time_scale = impegh_mp4_rev32(*data_size);
      fa->ufo.version1.time_scale = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
      if (ret < 8)
      {
        return IT_ERROR;
      }
      (*p)->duration = impegh_mp4_rev32(*data_size);
      fa->ufo.version1.duration = impegh_mp4_rev32(*data_size);
      break;
    default:
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->ufo.version0.creation_time = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      fa->ufo.version0.modification_time = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*p)->time_scale = impegh_mp4_rev32(*data_size);
      fa->ufo.version0.time_scale = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*p)->duration = impegh_mp4_rev32(*data_size);
      fa->ufo.version0.duration = impegh_mp4_rev32(*data_size);
      break;
    }

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->pad = impegh_mp4_rev16(*data_size_16);
    fa->lang = impegh_mp4_rev16(*data_size_16);
    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->reserved = impegh_mp4_rev16(*data_size_16);
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }

    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "mdhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_HDLR)
  {
    ia_mp4_handler_ref_atom *fa =
        (ia_mp4_handler_ref_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_handler_ref_atom));
    if (prev_state != IT_MDIA && prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    if (prev_state == IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, (WORD32)fa->large_size - 16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, (WORD32)fa->size - 8, SEEK_CUR);
      }
      return IT_OK;
    }
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    version = charbuf[0];
    fa->version = charbuf[0];
    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);
    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->reserved1 = impegh_mp4_rev32(*data_size);
    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    handler_type = impegh_mp4_rev32(*data_size);
    size -= 4;
    fa->handler_type = impegh_mp4_rev32(*data_size);
    (*p)->handler_type = handler_type;
    if (handler_type == IT_ODSM)
    {
      (*p)->trak_type = IT_MP4S;
    }
    else if (handler_type == IT_SDSM)
    {
      (*p)->trak_type = IT_MP4S;
    }
    else if (handler_type == IT_VIDE)
    {
      (*p)->trak_type = IT_MP4V;
    }
    else if (handler_type == IT_SOUN)
    {
      (*p)->trak_type = IT_MP4A;
    }
    else if (handler_type == IT_HINT)
    {
      (*p)->trak_type = IT_HINT;
    }
    else
    {
      (*p)->trak_type = (*p)->trak_type;
    }
    for (i = 0; i < 12; i++)
    {
      ret = (int)impegh_mp4_fread(charbuf, 1, 1, *fp);
      if (ret < 1)
      {
        return IT_ERROR;
      }
      fa->reserved2[i] = (WORD8)charbuf[0];
    }

    fa->name = impegh_mp4_mem_node_malloc(m, sizeof(WORD8) * (fa->size - 0x20));
    ret = impegh_mp4_fread(fa->name, 1, (fa->size - 0x20), *fp);
    if (ret < (fa->size - 0x20))

    {
      return IT_ERROR;
    }

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }

    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "hdlr");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_MINF)
  {
    if (prev_state != IT_MDIA)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    (*n)->data = (void *)a;
    strcpy((*n)->descr, "minf");
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    next_child = (*n)->child;
    next_child->parent = (*n);
    while ((impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_MINF)) == IT_OK)
    {
      next_child->sibling = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
      next_child->sibling->parent = next_child->parent;
      next_child->sibling->prev = next_child;
      next_child = next_child->sibling;
    }
    next_child->prev->sibling = NULL;
    impegh_mp4_free_mem_node(next_child, m);
    return IT_OK;
  }
  else if (a->type == IT_DINF)
  {
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    (*n)->data = (void *)a;
    strcpy((*n)->descr, "dinf");
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    next_child = (*n)->child;
    next_child->parent = (*n);
    impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_DINF);
    return IT_OK;
  }
  else if (a->type == IT_DREF)
  {
    ia_mp4_data_ref_atom *fa =
        (ia_mp4_data_ref_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_data_ref_atom));
    if (prev_state != IT_DINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->entry_count = impegh_mp4_rev32(*data_size);

    impegh_mp4_fseek(*fp, fa->size - 0x10, SEEK_CUR);
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "dref");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_STBL)
  {
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      return IT_ERROR;
    }
    (*p)->stbl_offset = impegh_mp4_ftell(*fp) - 0x08;
    impegh_mp4_fseek(*fp, (a->size - 8), SEEK_CUR);
    return IT_OK;
  }
  else if (a->type == IT_NMHD)
  {
    ia_mp4_mpeg_media_header_atom *fa =
        (ia_mp4_mpeg_media_header_atom *)impegh_mp4_mem_node_malloc(
            m, sizeof(ia_mp4_mpeg_media_header_atom));
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];
    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "nmhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_VMHD)
  {
    ia_mp4_video_media_header_atom *fa =
        (ia_mp4_video_media_header_atom *)impegh_mp4_mem_node_malloc(
            m, sizeof(ia_mp4_video_media_header_atom));
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }

    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
    if (ret < 8)
    {
      return IT_ERROR;
    }
    fa->reserved = impegh_mp4_rev32(*data_size);
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }

    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "vmhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }

  else if (a->type == IT_SMHD)
  {
    ia_mp4_sound_media_header_atom *fa =
        (ia_mp4_sound_media_header_atom *)impegh_mp4_mem_node_malloc(
            m, sizeof(ia_mp4_sound_media_header_atom));
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->reserved = impegh_mp4_rev32(*data_size);
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "smhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }

  else if (a->type == IT_HMHD)
  {
    ia_mp4_hint_media_header_atom *fa =
        (ia_mp4_hint_media_header_atom *)impegh_mp4_mem_node_malloc(
            m, sizeof(ia_mp4_hint_media_header_atom));
    if (prev_state != IT_MINF)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    fa->version = charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, *fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    fa->flags = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->max_pdu_size = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    fa->avg_pdu_size = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->max_bitrate = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->avg_bitrate = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->sliding_avg_bitrate = impegh_mp4_rev32(*data_size);

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "hmhd");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_TREF)
  {
    ia_mp4_trak_ref_atom *fa =
        (ia_mp4_trak_ref_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_trak_ref_atom));
    if (prev_state != IT_TRAK)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    i = impegh_mp4_rev32(*data_size) - 8;

    fa->ref_type.size = impegh_mp4_rev32(*data_size);
    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    ref_type = impegh_mp4_rev32(*data_size);
    fa->ref_type.type = ref_type;
    j = 0;
    while (i)
    {
      j++;
      i = i - 4;
    }
    sptr = (pUWORD32)impegh_mp4_mem_node_malloc(m, sizeof(UWORD32) * (j));
    fa->ref_type.track_Ids = sptr;
    if (ref_type == IT_HINT)
      (*p)->tref.ref_type = TREF_HINT;
    else if (ref_type == IT_DPND)
      (*p)->tref.ref_type = TREF_DPND;
    else if (ref_type == IT_IPIR)
      (*p)->tref.ref_type = TREF_IPIR;
    else if (ref_type == IT_MPOD)
      (*p)->tref.ref_type = TREF_MPOD;
    else if (ref_type == IT_SYNC)
      (*p)->tref.ref_type = TREF_SYNC;
    (*p)->tref.count = (UWORD16)j;
    (*p)->tref.track_id = sptr;

    while (j)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      *sptr = impegh_mp4_rev32(*data_size);

      sptr++;
      j--;
    }
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (void *)fa;
    strcpy((*n)->descr, "tref");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }

  else if (a->type == IT_FREE)
  {
    ia_mp4_free_atom *fa =
        (ia_mp4_free_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_free_atom));
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    impegh_mp4_fseek(*fp, fa->size - 8, SEEK_CUR);

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "free");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_SKIP)
  {
    ia_mp4_skip_atom *fa =
        (ia_mp4_skip_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_skip_atom));
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    impegh_mp4_fseek(*fp, fa->size - 8, SEEK_CUR);

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "skip");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_UDTA)
  {
    ia_mp4_udta_atom *fa =
        (ia_mp4_udta_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_udta_atom));
    UWORD8 offset = 8;
    if (a->size == 1)
    {
      offset = 16;
    }
    if (((prev_state == IT_TRAK)) &&
        ((impegh_mp4_ftell(*fp) - offset) >= m_info->trak_end_offset))
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    if (!((prev_state == IT_MOOV) || (prev_state == IT_TRAK)))
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;

    impegh_mp4_free_mem_node(a, m);

    if (prev_state == IT_TRAK)
    {
      (*p)->udta_info =
          (ia_mp4_udta_info *)impegh_mp4_mem_node_calloc(m, 1, sizeof(ia_mp4_udta_info));
    }
    else if (prev_state == IT_MOOV)
    {
      m_info->udta_info =
          (ia_mp4_udta_info *)impegh_mp4_mem_node_calloc(m, 1, sizeof(ia_mp4_udta_info));
    }
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "udta");
    (*n)->child = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
    (*n)->sibling = NULL;
    (*n)->prev = NULL;

    next_child = (*n)->child;
    next_child->parent = (*n);
    while ((impegh_mp4_read_atom(m_info, fp, &next_child, p, m, IT_UDTA)) == IT_OK)
    {
      next_child->sibling = (ia_mp4_node *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_node));
      next_child->sibling->parent = next_child->parent;
      next_child->sibling->prev = next_child;
      next_child = next_child->sibling;
    }
    impegh_mp4_free_mem_node(next_child, m);
    return IT_OK;
  }
  else if (a->type == IT_NAME)
  {
    ia_mp4_udta_name_atom *fa =
        (ia_mp4_udta_name_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_udta_name_atom));
    UWORD8 offset = 8;
    if (a->size == 1)
    {
      offset = 16;
    }
    if (((impegh_mp4_ftell(*fp) - offset) >= m_info->trak_end_offset))
      m_info->udta_info->name = fa;
    else
      (*p)->udta_info->name = fa;

    if (prev_state != IT_UDTA)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;
    impegh_mp4_free_mem_node(a, m);
    fa->tracks_name = (pUWORD8)impegh_mp4_mem_node_malloc(m, (sizeof(UWORD8)) * size);
    impegh_mp4_fread(fa->tracks_name, 1, size, *fp);
    if (ret < size)
    {
      return IT_ERROR;
    }
    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "name");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_HINF)
  {
    ia_mp4_udta_hinf_atom *fa =
        (ia_mp4_udta_hinf_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_udta_hinf_atom));
    UWORD8 offset = 8;
    if (a->size == 1)
    {
      offset = 16;
    }
    if (((impegh_mp4_ftell(*fp) - offset) >= m_info->trak_end_offset))
      m_info->udta_info->hinf = fa;
    else
      (*p)->udta_info->hinf = fa;
    if (prev_state != IT_UDTA)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;

    impegh_mp4_free_mem_node(a, m);

    while (size)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      i = impegh_mp4_rev32(*data_size);
      size -= 4;
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      type = impegh_mp4_rev32(*data_size);
      size -= 4;
      if (type == IT_TRPY)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->trpy = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_NUMP)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->nump = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_TPYL)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->tpyl = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_MAXR)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->maxr_g = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->maxr_m = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_DMED)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->dmed = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_DIMM)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->dimm = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_DREP)
      {
        ret = impegh_mp4_fread(charbuf, 1, 8, *fp);
        if (ret < 8)
        {
          return IT_ERROR;
        }
        fa->drep = impegh_mp4_rev32(*data_size);
        size -= 8;
      }
      else if (type == IT_TMIN)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->tmin = impegh_mp4_rev32(*data_size);
        size -= 4;
      }
      else if (type == IT_TMAX)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->tmax = impegh_mp4_rev32(*data_size);
        size -= 4;
      }
      else if (type == IT_PMAX)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->pmax = impegh_mp4_rev32(*data_size);
        size -= 4;
      }
      else if (type == IT_DMAX)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->dmax = impegh_mp4_rev32(*data_size);
        size -= 4;
      }
      else if (type == IT_PAYT)
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        fa->payt = impegh_mp4_rev32(*data_size);
        size -= 4;

        ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
        if (ret < 1)
        {
          return IT_ERROR;
        }
        fa->pay_str_count = (WORD8)charbuf[0];
        size -= 1;

        fa->pay_str = impegh_mp4_mem_node_malloc(m, ((sizeof(WORD8)) * (fa->pay_str_count)) + 1);
        ret = impegh_mp4_fread(fa->pay_str, 1, fa->pay_str_count, *fp);
        if (ret < fa->pay_str_count)
          return IT_ERROR;
        size -= fa->pay_str_count;
        fa->pay_str[fa->pay_str_count] = 0;
      }
      else
      {
        impegh_mp4_fseek(*fp, i - 8, SEEK_CUR);
        size -= (i - 8);
        break;
      }
    }

    if ((*n) == NULL)
    {
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    }
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "hinf");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;
    return IT_OK;
  }
  else if (a->type == IT_HNTI)
  {
    ia_mp4_udta_hnti_atom *fa =
        (ia_mp4_udta_hnti_atom *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_udta_hnti_atom));
    UWORD8 offset = 8, offset1 = 8;
    if (a->size == 1)
    {
      offset = 16;
    }
    if (((impegh_mp4_ftell(*fp) - offset) >= m_info->trak_end_offset))
      m_info->udta_info->hnti = fa;
    else
      (*p)->udta_info->hnti = fa;

    if (prev_state != IT_UDTA)
    {
      if (a->size == 1)
      {
        impegh_mp4_fseek(*fp, -16, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(*fp, -8, SEEK_CUR);
      }
      impegh_mp4_free_mem_node(fa, m);
      return IT_ERROR;
    }
    fa->size = a->size;
    fa->type = a->type;
    fa->large_size = a->large_size;

    impegh_mp4_free_mem_node(a, m);

    fa->hnti_sdp = impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_hnti_sdp_atom));

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    fa->hnti_sdp->size = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    if (((impegh_mp4_ftell(*fp) - offset) >= m_info->trak_end_offset))
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      offset1 = 12;
    }

    fa->hnti_sdp->type = impegh_mp4_rev32(*data_size);

    fa->hnti_sdp->sdp_info =
        impegh_mp4_mem_node_malloc(m, (sizeof(WORD8) * (fa->hnti_sdp->size - offset1 + 1)));
    ret = impegh_mp4_fread(fa->hnti_sdp->sdp_info, 1, fa->hnti_sdp->size - offset1, *fp);
    if (ret < fa->hnti_sdp->size - offset1)
      return IT_ERROR;
    fa->hnti_sdp->sdp_info[fa->hnti_sdp->size - offset1] = 0;

    if ((*n) == NULL)
      impegh_mp4_error_hdl(NULL, (pWORD8) "Error: NULL pointer encountered %x", type);
    (*n)->data = (pVOID)fa;
    strcpy((*n)->descr, "hnti");
    (*n)->child = NULL;
    (*n)->sibling = NULL;
    (*n)->prev = NULL;

    return IT_OK;
  }
  else if (prev_state == IT_UDTA)
  {
    impegh_mp4_fseek(*fp, size, SEEK_CUR);
    return IT_OK;
  }
  else
  {
    impegh_mp4_fseek(*fp, size, SEEK_CUR);
    return IT_OK;
  }
}

/**impegh_mp4_read_samples
 *
 *  \brief Reads the sample entries in 'stsd' ia_mp4_atom
 *
 *  \param [in] fp	VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_sample_entry structure
 *  \param [in] m	 Pointer to ia_mp4_mem_node structure
 *  \param [in] p	Pointer to trak_info structure
 *  \param [in,out] ptr_mae_info pointer to mae info structure
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_read_samples(pVOID fp, ia_mp4_sample_entry **n, ia_mp4_mem_node **m,
                                     trak_info *p, ia_maei_info *ptr_mae_info)
{
  ia_mp4_es_desc *ptr1 = NULL;
  ia_mp4_mh_desc *ptr2 = NULL;
  UWORD32 ret, data_tag, data;
  WORD32 es_len = 0;
  WORD32 len = -1;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];
  UWORD32 i;
  WORD32 tag_mhm1 = 0;
  WORD32 tag_maei = 0;
  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  (*n)->sample.size = impegh_mp4_rev32(*data_size);
  len = (*n)->sample.size - 4;

  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  (*n)->sample.type = impegh_mp4_rev32(*data_size);
  len -= 4;

  if (p->handler_type == IT_NONE)
  {
    if ((*n)->sample.type == IT_MP4A)
      p->handler_type = IT_SOUN;
    else if ((*n)->sample.type == IT_MP4V || (*n)->sample.type == IT_MJPA)
      p->handler_type = IT_VIDE;
    else if ((*n)->sample.type == IT_RTP)
      p->handler_type = IT_HINT;
  }
  p->sample_type = (*n)->sample.type;
  if ((*n)->sample.size == 1)
  {
    ret = impegh_mp4_fread(charbuf, 1, 8, fp);
    if (ret < 8)
    {
      return IT_ERROR;
    }
    (*n)->sample.large_size = impegh_mp4_rev32(*data_size);
    len -= 8;
  }

  if (p->sample_type == IT_MHM1)
  {
    tag_mhm1 = 1;
    g_mhm1_tag = 1;
  }
  for (i = 0; i < 6; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sample.reserved[i] = (WORD8)charbuf[0];
  }
  len -= 6;

  ret = impegh_mp4_fread(charbuf, 1, 2, fp);
  if (ret < 2)
  {
    return IT_ERROR;
  }
  (*n)->sample.data_ref_idx = impegh_mp4_rev16(*data_size_16);
  len -= 2;

  if (p->handler_type == IT_SOUN)
  {
    for (i = 0; i < 2; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*n)->stream.audio.reserved1[i] = impegh_mp4_rev32(*data_size);
    }
    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.audio.reserved2 = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.audio.reserved3 = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.audio.reserved4 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.audio.time_scale = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.audio.reserved5 = impegh_mp4_rev16(*data_size_16);
    len -= 20;
  }
  else if (p->handler_type == IT_VIDE)
  {
    for (i = 0; i < 4; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*n)->stream.video.reserved1[i] = impegh_mp4_rev32(*data_size);
    }
    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved2 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved3 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved4 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved5 = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved6 = impegh_mp4_rev16(*data_size_16);

    for (i = 0; i < 32; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 1, fp);
      if (ret < 1)
      {
        return IT_ERROR;
      }
      (*n)->stream.video.reserved7[i] = (*((pWORD8)charbuf));
    }

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved8 = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.video.reserved9 = impegh_mp4_rev16(*data_size_16);
    len -= 70;
  }
  else if (p->handler_type == IT_HINT)
  {

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.hint.ht_ver = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->stream.hint.last_compatible_ht_ver = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->stream.hint.max_packet_size = impegh_mp4_rev32(*data_size);
    len -= 8;
    (*n)->stream.hint.tims = 0;
    (*n)->stream.hint.tsro = 0;
    (*n)->stream.hint.snro = 0;
    while (len)
    {
      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      ret = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      data_tag = impegh_mp4_rev32(*data_size);

      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      data = impegh_mp4_rev32(*data_size);
      len -= 12;

      if (data_tag == IT_TIMS)
      {
        (*n)->stream.hint.tims = data;
      }
      else if (data_tag == IT_TSRO)
      {
        (*n)->stream.hint.tsro = data;
      }
      else if (data_tag == IT_SNRO)
      {
        (*n)->stream.hint.snro = data;
      }
      else
      {
        return IT_ERROR;
      }
    }
    return IT_OK;
  }
  if (len == 0)
  {
    return IT_OK;
    //no child atoms of mhm1
  }
  //reading child atoms of mhm1

  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  (*n)->es.size = impegh_mp4_rev32(*data_size);

  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  (*n)->es.type = impegh_mp4_rev32(*data_size);
  UWORD32 temp_len = 0;
  len -= 8;
  temp_len += 8;
  if (tag_mhm1 && ((*n)->es.type == IT_MHAC))
  {
    impegh_mp4_fseek(fp, (*n)->es.size - 8, SEEK_CUR);
    return IT_OK;
  }
  if ((*n)->es.type == IT_MHAC)
  {
    (*n)->mh.size = (*n)->es.size;
    (*n)->mh.type = (*n)->es.type;
  }
  if (!((*n)->es.type == IT_MHAC))
  {

    if ((*n)->es.type != IT_ESDS)
    {
      if ((*n)->es.type == IT_BTRT)
      {
        impegh_mp4_fseek(fp, (*n)->es.size - 8, SEEK_CUR);
        return IT_OK;
      }
      if ((*n)->sample.type == IT_MJPA)
      {
        impegh_mp4_fseek(fp, len, SEEK_CUR);
        return IT_OK;
      }
    }
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->es.version = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, fp);
    if (ret < 3)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    (*n)->es.flags = impegh_mp4_rev32(*data_size);

    len -= 4;
    temp_len += 4;
    ptr1 = (ia_mp4_es_desc *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_es_desc));
    (*n)->es.es_descr = ptr1;

    ptr1->dec_config_desc.dec_specific_info.length = 0;
    ptr1->dec_config_desc.dec_specific_info.dec_info = 0;
    (*p).dec_info_length = 0;
    (*p).dec_info = 0;
    if ((es_len = impegh_mp4_get_es(fp, &ptr1, m, p)) == IT_ERROR)
      return (IT_ERROR);
    impegh_mp4_fseek(fp, len - es_len, SEEK_CUR);
  }
  else
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->mh.version = (WORD8)charbuf[0];
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    ptr2 = (ia_mp4_mh_desc *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_mh_desc));
    (*n)->mh.mh_descr = ptr2;
    ptr2->dec_specific_info.length = 0;
    ptr2->dec_specific_info.dec_info = 0;
    (*p).dec_info_length = 0;
    (*p).dec_info = 0;
    (*n)->mh.mh_descr->mpegh_3da_profile_lvl_indication = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->mh.mh_descr->ref_ch_layout = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(&(charbuf[2]), 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    charbuf[0] = 0;
    charbuf[1] = 0;
    (*n)->mh.mh_descr->mpegh_3da_config_length = impegh_mp4_rev32(*data_size);

    len -= 5;
    temp_len += 5;
    (*n)->mh.mh_descr->dec_specific_info.length = (*n)->mh.mh_descr->mpegh_3da_config_length;
    pUWORD8 str = (pUWORD8)impegh_mp4_mem_node_malloc(
        m, sizeof(UWORD8) * ((*n)->mh.mh_descr->dec_specific_info.length));
    ret = impegh_mp4_fread(str, 1, (*n)->mh.mh_descr->dec_specific_info.length, fp);
    if (ret < (*n)->mh.mh_descr->dec_specific_info.length)
    {
      return IT_ERROR;
    }
    (*n)->mh.mh_descr->dec_specific_info.dec_info = str;
    p->dec_info_length = (*n)->mh.mh_descr->dec_specific_info.length;
    p->dec_info = str;
    len -= p->dec_info_length;
    temp_len += p->dec_info_length;
    if ((!tag_mhm1) && (ptr_mae_info->init == 1))
    {
      impegh_mp4_fseek(fp, ((*n)->es.size - temp_len), SEEK_CUR);
      len -= ((*n)->es.size - temp_len);
      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*n)->es.size = impegh_mp4_rev32(*data_size);
      ret = impegh_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      (*n)->es.type = impegh_mp4_rev32(*data_size);
      len -= 8;

      if ((*n)->es.type == IT_MAEI)
      {
        (*n)->maei.size = (*n)->es.size;
        (*n)->maei.type = (*n)->es.type;
        tag_maei = 1;
      }
      if ((tag_maei))
      {
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        (*n)->maei.version = *data_size;
        len -= 4;
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        (*n)->es.size = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        (*n)->es.type = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 1, fp);
        if ((*n)->es.type == IT_MAEG)
        {
          (*n)->maeg.size = (*n)->es.size;
          (*n)->maeg.type = (*n)->es.type;
        }
        (*n)->maeg.version = (WORD8)charbuf[0];
        ia_bit_buf_struct_e ptr_read_buff;
        ia_bit_buf_struct_d ptr_write_buff;

        ret = impegh_mp4_fread(&(*n)->maei.maei_read_buf[0], 1, (*n)->maeg.size - 9, fp);
        if (ret < ((*n)->maeg.size - 9))
        {
          return IT_ERROR;
        }
        impegh_create_read_buf(&ptr_read_buff, &((*n)->maei.maei_read_buf[0]),
                               sizeof((*n)->maei.maei_read_buf));

        impegh_create_bit_buffer(&ptr_write_buff, &ptr_mae_info->maei_write_buf[0],
                                 sizeof(ptr_mae_info->maei_write_buf), 1);
        WORD32 bits_written = 0;
        bits_written +=
            impegh_mp4_get_maeg(&ptr_read_buff, &ptr_write_buff, ((*n)->maeg.size - 9));
        len -= (*n)->maeg.size;
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        (*n)->es.size = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        (*n)->es.type = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 1, fp);
        (*n)->maes.version = (WORD8)charbuf[0];
        if ((*n)->es.type == IT_MAES)
        {
          (*n)->maes.size = (*n)->es.size;
          (*n)->maes.type = (*n)->es.type;
        }
        ret = impegh_mp4_fread(&(*n)->maei.maei_read_buf[0], 1, (*n)->maes.size - 9, fp);
        if (ret < ((*n)->maes.size - 9))
        {
          return IT_ERROR;
        }
        impegh_create_read_buf(&ptr_read_buff, &((*n)->maei.maei_read_buf[0]),
                               sizeof((*n)->maei.maei_read_buf));
        bits_written +=
            impegh_mp4_get_maes(&ptr_read_buff, &ptr_write_buff, ((*n)->maes.size - 9));
        len -= (*n)->maes.size;
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        (*n)->es.size = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        (*n)->es.type = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 1, fp);
        (*n)->maep.version = (WORD8)charbuf[0];
        if ((*n)->es.type == IT_MAEP)
        {
          (*n)->maep.size = (*n)->es.size;
          (*n)->maep.type = (*n)->es.type;
        }

        ret = impegh_mp4_fread(&(*n)->maei.maei_read_buf[0], 1, (*n)->maep.size - 9, fp);
        if (ret < ((*n)->maep.size - 9))
        {
          return IT_ERROR;
        }
        impegh_create_read_buf(&ptr_read_buff, &((*n)->maei.maei_read_buf[0]),
                               sizeof((*n)->maei.maei_read_buf));
        bits_written +=
            impegh_mp4_get_maep(&ptr_read_buff, &ptr_write_buff, ((*n)->maep.size - 9));
        len -= (*n)->maep.size;
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        (*n)->es.size = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 4, fp);
        if (ret < 4)
        {
          return IT_ERROR;
        }
        (*n)->es.type = impegh_mp4_rev32(*data_size);
        ret = impegh_mp4_fread(charbuf, 1, 1, fp);
        (*n)->mael.version = (WORD8)charbuf[0];
        if ((*n)->es.type == IT_MAEL)
        {
          (*n)->mael.size = (*n)->es.size;
          (*n)->mael.type = (*n)->es.type;
        }
        ret = impegh_mp4_fread(&(*n)->maei.maei_read_buf[0], 1, (*n)->mael.size - 9, fp);
        if (ret < ((*n)->mael.size - 9))
        {
          return IT_ERROR;
        }
        impegh_create_read_buf(&ptr_read_buff, &((*n)->maei.maei_read_buf[0]),
                               sizeof((*n)->maei.maei_read_buf));
        bits_written +=
            impegh_mp4_get_mael(&ptr_read_buff, &ptr_write_buff, ((*n)->mael.size - 9));
        len -= (*n)->mael.size;
        ptr_mae_info->maei_bytes_written = (bits_written + 7) >> 3;
        impegh_mp4_fseek(fp, len, SEEK_CUR);
      }
      else
      {
        impegh_mp4_fseek(fp, len, SEEK_CUR);
      }
    }
    else
    {
      impegh_mp4_fseek(fp, len, SEEK_CUR);
    }
  }
  return IT_OK;
}

/**impegh_mp4_get_es
 *
 *  \brief Reads the elementary streams
 *
 *  \param [in] fp VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_es_desc structure
 *  \param [in] m	Pointer to ia_mp4_mem_node structure
 *  \param [in] p	Pointer to trak_info structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_es(pVOID fp, ia_mp4_es_desc **n, ia_mp4_mem_node **m, trak_info *p)
{
  UWORD8 *ptr1 = NULL, charbuf[10];
  ia_mp4_ipmp_desc_pointer *ptr2 = NULL;
  UWORD32 i, count = 0, length, es_len = 0;
  UWORD8 tag;
  WORD32 ret = 0;
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)
  {
    return IT_ERROR;
  }
  (*n)->tag = (WORD8)charbuf[0];
  es_len += 1;
  length = 0;
  for (i = 0; i < 4; i++)
  {
    es_len += 1;
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }

  if (i == 4)
  {
    return IT_ERROR;
  }
  (*n)->length = length;
  es_len += length;
  ret = impegh_mp4_fread(charbuf, 1, 2, fp);
  if (ret < 2)
  {
    return IT_ERROR;
  }
  (*n)->es_id = impegh_mp4_rev16(*data_size_16);
  (*p).es_id = (*n)->es_id;
  count = count + 2;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)
  {
    return IT_ERROR;
  }
  count = count + 1;

  (*n)->stream_dependence_flag = ((WORD8)charbuf[0]) >> 7;
  (*n)->stream_priority = (((WORD8)charbuf[0]) << 3) >> 3;
  (*n)->url_flag = (((WORD8)charbuf[0]) << 1) >> 7;
  (*n)->ocr_stream_flag = (((WORD8)charbuf[0]) << 2) >> 7;
  if ((*n)->stream_dependence_flag)
  {
    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->depends_on_es_id = impegh_mp4_rev16(*data_size_16);
    count = count + 2;
  }

  if ((*n)->url_flag)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->url_len = (WORD8)charbuf[0];
    count = count + 1;

    ptr1 = (pUWORD8)impegh_mp4_mem_node_malloc(m, sizeof(UWORD8) * ((*n)->url_len));
    (*n)->url_string = ptr1;
    for (i = 0; i < (*n)->url_len; i++)
    {
      ret = impegh_mp4_fread(charbuf, 1, 1, fp);
      if (ret < 1)
      {
        return IT_ERROR;
      }
      count = count + 2;
      *ptr1 = charbuf[0];
      ptr1++;
    }
  }

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)
  {
    return IT_ERROR;
  }
  tag = (WORD8)charbuf[0];

  ptr2 =
      (ia_mp4_ipmp_desc_pointer *)impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_ipmp_desc_pointer));
  if (ptr2 == NULL)
    return IT_ERROR;
  (*n)->ipmp_desc_ptr = ptr2;
  ptr2->next = NULL;

  while ((*n)->length - count)
  {
    switch (tag)
    {
    case DEC_CONFIG_DESC_TAG:
      count = count + 1;
      (*n)->dec_config_desc.tag = DEC_CONFIG_DESC_TAG;
      ret = impegh_mp4_get_decoder_config_descr(fp, n, m, p);
      if (ret != IT_ERROR)
        count = count + ret;
      else
      {
        return IT_ERROR;
      }
      break;

    case SL_CONFIG_DESC_TAG:
      count = count + 1;
      (*n)->sl_config_desc.tag = SL_CONFIG_DESC_TAG;
      ret = impegh_mp4_get_sl_config_descr(fp, n);
      if (ret)
        count = count + ret;
      else

      {
        return IT_ERROR;
      }
      break;

    case IPI_DESC_POINTER_TAG:
      count = count + 1;
      (*n)->ipi_point.tag = IPI_DESC_POINTER_TAG;
      ret = impegh_mp4_get_ipi_descr_pointer(fp, n);
      if (ret)
        count = count + ret;
      else
      {
        return IT_ERROR;
      }
      break;

    case IPMP_DESC_POINTER_TAG:
      ptr2->next = (ia_mp4_ipmp_desc_pointer *)impegh_mp4_mem_node_malloc(
          m, sizeof(ia_mp4_ipmp_desc_pointer));
      if (ptr2->next == NULL)
        return IT_ERROR;
      ptr2 = ptr2->next;
      ptr2->next = NULL;
      count = count + 1;
      (*n)->ipmp_desc_ptr->tag = IPMP_DESC_POINTER_TAG;
      ret = impegh_mp4_get_ipmp_desc_pointer(fp, &ptr2);
      if (ret)
        count = count + ret;
      else
      {
        return IT_ERROR;
      }

      break;

    default:
    {
      return IT_ERROR;
    }
    }

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)
    {
      break;
    }
    tag = (WORD8)charbuf[0];
  }
  impegh_mp4_fseek(fp, -1, SEEK_CUR);
  return es_len;
}

/**impegh_mp4_get_decoder_config_descr
 *
 *  \brief Reads the Decoder Configuration Descriptor
 *
 *  \param [in] fp	VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_es_desc structure
 *  \param [in] m	Pointer to ia_mp4_mem_node structure
 *  \param [in] p	Pointer to trak_info structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_decoder_config_descr(pVOID fp, ia_mp4_es_desc **n,
                                                 ia_mp4_mem_node **m, trak_info *p)
{
  UWORD32 count = 0, length = 0, tag = 0, i, ret;
  UWORD8 charbuf[10];
  ia_mp4_profile_lvl_indication_idc_desc *ptr1 = NULL;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];

  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    if (ret < 1)

    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  (*n)->dec_config_desc.length = length;
  length = length + 1;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)

  {
    return IT_ERROR;
  }
  (*n)->dec_config_desc.obj_type_indication = (WORD8)charbuf[0];
  p->object_type = (*n)->dec_config_desc.obj_type_indication;
  count = count + 1;
  length = length - 1;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)

  {
    return IT_ERROR;
  }
  (*n)->dec_config_desc.stream_type = ((WORD8)charbuf[0]) >> 2;
  (*n)->dec_config_desc.up_stream = ((((WORD8)charbuf[0]) << 6) >> 7);
  p->stream_type = (*n)->dec_config_desc.stream_type;
  count = count + 1;
  length = length - 1;

  ret = impegh_mp4_fread(&(charbuf[1]), 1, 3, fp);
  if (ret < 3)
  {
    return IT_ERROR;
  }
  charbuf[0] = 0;
  ret = (UWORD32)charbuf[0];
  (*n)->dec_config_desc.buf_size_db = impegh_mp4_rev24(*data_size);
  count = count + 3;
  length = length - 3;

  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)

  {
    return IT_ERROR;
  }
  (*n)->dec_config_desc.max_bitrate = impegh_mp4_rev32(*data_size);
  count = count + 4;
  length = length - 4;

  ret = impegh_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)

  {
    return IT_ERROR;
  }
  (*n)->dec_config_desc.avg_bitrate = impegh_mp4_rev32(*data_size);
  count = count + 4;
  length = length - 4;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)

  {
    return IT_ERROR;
  }
  tag = (WORD8)charbuf[0];
  count = count + 1;
  length = length - 1;

  p->dec_info_length = 0;
  p->dec_info = NULL;

  ptr1 = (ia_mp4_profile_lvl_indication_idc_desc *)impegh_mp4_mem_node_malloc(
      m, sizeof(ia_mp4_profile_lvl_indication_idc_desc));
  if (ptr1 == NULL)
    return (IT_ERROR);
  ptr1->tag = 0;
  (*n)->dec_config_desc.profile_lvl_indication_idx_desc = ptr1;
  ptr1->next_profile_lvl_indication_entry = NULL;
  (*n)->dec_config_desc.dec_specific_info.tag = 0;
  while (length)
  {
    switch (tag)
    {
    case DEC_SPECIFIC_INFO_TAG:
      (*n)->dec_config_desc.dec_specific_info.tag = DEC_SPECIFIC_INFO_TAG;
      ret = 0;
      ret = impegh_mp4_decoder_specific_info(fp, n, m, p);
      if (ret)
      {
        length = length - ret;
        count = count + ret;
      }
      else

      {
        return IT_ERROR;
      }
      break;

    case PROF_LVL_INDICATION_INDEX_DESC_TAG: /* ProfileLevelIndicationIndexDescriptor */
      ret = impegh_mp4_fread(charbuf, 1, 1, fp);
      if (ret < 1)
        return IT_ERROR;
      ptr1->tag = PROF_LVL_INDICATION_INDEX_DESC_TAG;
      ptr1->profile_lvl_indication_idx = (WORD8)charbuf[0];
      count = count + 1;
      length = length - 1;

      ptr1->next_profile_lvl_indication_entry =
          (ia_mp4_profile_lvl_indication_idc_desc *)impegh_mp4_mem_node_malloc(
              m, sizeof(ia_mp4_profile_lvl_indication_idc_desc));
      if (ptr1->next_profile_lvl_indication_entry == NULL)
        return (IT_ERROR);
      ptr1 = ptr1->next_profile_lvl_indication_entry;
      ptr1->tag = 0;
      ptr1->next_profile_lvl_indication_entry = NULL;
      break;

    default:
      return IT_ERROR;
    }
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    if (ret < 1)

    {
      return IT_ERROR;
    }
    tag = (WORD8)charbuf[0];
    count = count + 1;
    length = length - 1;
  }
  impegh_mp4_fseek(fp, -1, SEEK_CUR);
  count = count - 1;
  return count;
}

/**impegh_mp4_decoder_specific_info
 *
 *  \brief Reads the Decoder Specific Information
 *
 *  \param [in] fp VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_es_desc
 *  \param [in] m	Pointer to ia_mp4_mem_node
 *  \param [in] p	Pointer to trak_info
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_decoder_specific_info(pVOID fp, ia_mp4_es_desc **n, ia_mp4_mem_node **m,
                                              trak_info *p)
{
  UWORD32 ret = 0, i, count = 0, length = 0;
  UWORD8 *str, charbuf[10];
  length = 0;
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }

  (*n)->dec_config_desc.dec_specific_info.length = length;

  str = (pUWORD8)impegh_mp4_mem_node_malloc(
      m, sizeof(UWORD8) * ((*n)->dec_config_desc.dec_specific_info.length));

  ret = impegh_mp4_fread(str, 1, (*n)->dec_config_desc.dec_specific_info.length, fp);
  if (ret < (*n)->dec_config_desc.dec_specific_info.length)
  {
    return IT_ERROR;
  }
  count = count + (*n)->dec_config_desc.dec_specific_info.length;

  (*n)->dec_config_desc.dec_specific_info.dec_info = str;

  p->dec_info_length = (*n)->dec_config_desc.dec_specific_info.length;
  p->dec_info = str;
  p->avg_bitrate = (*n)->dec_config_desc.avg_bitrate;
  if (ret < (*n)->dec_config_desc.dec_specific_info.length)
  {
    return IT_ERROR;
  }
  return (count);
}

/**impegh_mp4_get_sl_config_descr
 *
 *  \brief Reads the ia_mp4_sl_config_desc
 *
 *  \param [in] fp  VOID pointer
 *  \param [in,out] n Pointer to ia_mp4_es_desc structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_sl_config_descr(pVOID fp, ia_mp4_es_desc **n)
{
  UWORD32 ret, length, i, count = 0;
  UWORD8 charbuf[10], timestamplength = 0;
  UWORD16 temp;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];
  length = 0;
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  (*n)->sl_config_desc.length = length;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  count = count + 1;
  length -= 1;
  if (ret < 1)
  {
    return IT_ERROR;
  }
  (*n)->sl_config_desc.predefined = (WORD8)charbuf[0];

  if ((*n)->sl_config_desc.predefined == 0)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    length -= 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.use_access_unit_start_flag = (WORD8)((charbuf[0]) >> 7);
    (*n)->sl_config_desc.use_access_unit_end_flag = (WORD8)((charbuf[0] << 1) >> 7);
    (*n)->sl_config_desc.use_random_access_point_flag = (WORD8)((charbuf[0] << 2) >> 7);
    (*n)->sl_config_desc.has_rand_access_units_only_flag = (WORD8)((charbuf[0] << 3) >> 7);
    (*n)->sl_config_desc.use_padding_flag = (WORD8)((charbuf[0] << 4) >> 7);
    (*n)->sl_config_desc.use_time_stamps_flag = (WORD8)((charbuf[0] << 5) >> 7);
    (*n)->sl_config_desc.use_idle_flag = (WORD8)((charbuf[0] << 6) >> 7);
    (*n)->sl_config_desc.duration_flag = (WORD8)((charbuf[0] << 7) >> 7);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    count = count + 4;
    length -= 4;
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.time_stamp_res = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    count = count + 4;
    length -= 4;
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.ocr_res = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    length -= 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.time_stamp_len = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    length -= 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.ocr_len = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    length -= 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.au_len = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    length -= 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.instant_bitrate_len = (WORD8)charbuf[0];

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    count = count + 2;
    length -= 2;
    if (ret < 2)

    {
      return IT_ERROR;
    }
    temp = impegh_mp4_rev16(*data_size_16);
    (*n)->sl_config_desc.degradation_priority_length = (UWORD16)(temp >> 12);
    (*n)->sl_config_desc.au_seq_num_len = (UWORD16)((temp << 4) >> 11);
    (*n)->sl_config_desc.packet_seq_num_len = (UWORD16)((temp << 9) >> 11);
    (*n)->sl_config_desc.reserved = (UWORD16)((temp << 14) >> 14);
  }
  else if ((*n)->sl_config_desc.predefined == 1)
  {
    (*n)->sl_config_desc.use_access_unit_start_flag = 0;
    (*n)->sl_config_desc.use_access_unit_end_flag = 0;
    (*n)->sl_config_desc.use_random_access_point_flag = 0;
    (*n)->sl_config_desc.use_padding_flag = 0;
    (*n)->sl_config_desc.use_time_stamps_flag = 0;
    (*n)->sl_config_desc.use_idle_flag = 0;
    (*n)->sl_config_desc.time_stamp_res = 1000;
    (*n)->sl_config_desc.time_stamp_len = 32;
    (*n)->sl_config_desc.au_len = 0;
    (*n)->sl_config_desc.degradation_priority_length = 0;
    (*n)->sl_config_desc.au_seq_num_len = 0;
    (*n)->sl_config_desc.packet_seq_num_len = 0;
  }
  else if ((*n)->sl_config_desc.predefined == 2)
  {
    (*n)->sl_config_desc.use_access_unit_start_flag = 0;
    (*n)->sl_config_desc.use_access_unit_end_flag = 0;
    (*n)->sl_config_desc.use_random_access_point_flag = 0;
    (*n)->sl_config_desc.use_padding_flag = 0;
    (*n)->sl_config_desc.use_time_stamps_flag = 1;
    (*n)->sl_config_desc.use_idle_flag = 0;
    (*n)->sl_config_desc.duration_flag = 0;
    (*n)->sl_config_desc.time_stamp_len = 0;
    (*n)->sl_config_desc.ocr_len = 0;
    (*n)->sl_config_desc.au_len = 0;
    (*n)->sl_config_desc.instant_bitrate_len = 0;
    (*n)->sl_config_desc.degradation_priority_length = 0;
    (*n)->sl_config_desc.au_seq_num_len = 0;
    (*n)->sl_config_desc.packet_seq_num_len = 0;
  }
  if ((*n)->sl_config_desc.duration_flag)
  {
    ret = impegh_mp4_fread(charbuf, 1, 4, fp);
    count = count + 4;
    length -= 4;
    if (ret < 4)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.time_scale = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    count = count + 2;
    length -= 2;
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.access_unit_duration = impegh_mp4_rev16(*data_size_16);

    ret = impegh_mp4_fread(charbuf, 1, 2, fp);
    count = count + 2;
    length -= 2;
    if (ret < 2)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.composition_unit_duration = impegh_mp4_rev16(*data_size_16);
  }

  timestamplength = (*n)->sl_config_desc.time_stamp_len >> 3;
  for (i = 0; i < 8; i++)
    charbuf[i] = 0;
  if (!(*n)->sl_config_desc.use_time_stamps_flag)
  {
    ret = impegh_mp4_fread(charbuf, 1, timestamplength, fp);
    count = count + timestamplength;
    length -= timestamplength;
    if (ret < timestamplength)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.start_decoding_time_stamp = impegh_mp4_rev32(*data_size);

    ret = impegh_mp4_fread(charbuf, 1, timestamplength, fp);
    count = count + timestamplength;
    length -= timestamplength;
    if (ret < timestamplength)
    {
      return IT_ERROR;
    }
    (*n)->sl_config_desc.start_composition_time_stamp = impegh_mp4_rev32(*data_size);
  }
  impegh_mp4_fseek(fp, length, SEEK_CUR);
  count = count + length;
  return count;
}

/**impegh_mp4_get_ipi_descr_pointer
 *
 *  \brief Reads the ia_mp4_ipi_desc_pointer
 *
 *  \param [in] fp	VOID pointer
 *  \param [in,out] n	pointer to ia_mp4_es_desc structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_ipi_descr_pointer(pVOID fp, ia_mp4_es_desc **n)
{
  UWORD32 ret, count = 0, i = 0, length = 0;
  UWORD8 charbuf[10];
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  (*n)->ipi_point.length = length;

  ret = impegh_mp4_fread(charbuf, 1, 2, fp);
  count = count + 2;
  if (ret < 2)
  {
    return IT_ERROR;
  }
  (*n)->ipi_point.ipi_es_id = impegh_mp4_rev16(*data_size_16);

  return count;
}

/**impegh_mp4_get_ipmp_desc_pointer
 *
 *  \brief Brief Reads the ia_mp4_ipmp_desc_pointer
 *
 *  \param [in] fp VOID pointer
 *  \param [in] n Pointer to ia_mp4_ipmp_desc_pointer structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_ipmp_desc_pointer(pVOID fp, ia_mp4_ipmp_desc_pointer **n)
{
  UWORD32 ret, i, length = 0, count = 0;
  UWORD8 charbuf[10];
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, fp);
    count = count + 1;
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  (*n)->length = length;

  ret = impegh_mp4_fread(charbuf, 1, 1, fp);
  count = count + 1;
  if (ret < 1)
  {
    return IT_ERROR;
  }
  (*n)->ipmp_desc_id = (WORD8)charbuf[0];
  return count;
}

/**impegh_mp4_get_iod
 *
 *  \brief Reads the ia_mp4_initial_obj_desc info from the file
 *
 *  \param [in] fp	VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_initial_obj_desc structure
 *  \param [in] m	Pointer to ia_mp4_mem_node structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_iod(VOID **fp, ia_mp4_initial_obj_desc *n, ia_mp4_mem_node **m)
{
  UWORD8 charbuf[10], tag;
  UWORD32 i, ret, count = 0, length;
  UWORD16 temp;
  ia_mp4_es_id_inc_desc *ptr1 = NULL;
  UWORD16 *data_size_16 = (UWORD16 *)&charbuf[0];
  length = 0;
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  if (i == 4)
  {
    return IT_ERROR;
  }
  n->length = length;

  ret = impegh_mp4_fread(charbuf, 1, 2, *fp);
  if (ret < 2)
  {
    return IT_ERROR;
  }
  temp = impegh_mp4_rev16(*data_size_16);
  count += 2;

  n->obj_descriptor_id = temp >> 6;
  n->url_flag = (temp << 10) >> 15;
  n->inc_inline_profile_flag = (temp << 11) >> 15;
  n->reserved = (temp << 12) >> 12;

  if (n->url_flag)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    n->iod.url.url_len = (WORD8)charbuf[0];
    count += 1;
  }
  else
  {
    ret = impegh_mp4_fread(charbuf, 1, 5, *fp);
    if (ret < 5)
    {
      return IT_ERROR;
    }
    count += 5;
    n->iod.profile.od_profile_lvl_indication = (WORD8)charbuf[0];
    n->iod.profile.scene_profile_lvl_indication = (WORD8)charbuf[1];
    n->iod.profile.audio_profile_lvl_indication = (WORD8)charbuf[2];
    n->iod.profile.visual_profile_lvl_indication = (WORD8)charbuf[3];
    n->iod.profile.graphics_profile_lvl_indication = (WORD8)charbuf[4];

    ptr1 = impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_es_id_inc_desc));
    n->iod.profile.es_id_inc = ptr1;
    ptr1->tag = 0;
    ptr1->next = NULL;

    while (n->length - count)
    {
      ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
      if (ret < 1)
      {
        return IT_ERROR;
      }
      tag = (WORD8)charbuf[0];
      count += 1;

      switch (tag)
      {
      case ES_ID_INC_TAG:
        ptr1->tag = ES_ID_INC_TAG;
        ret = impegh_mp4_get_es_id_inctag(fp, ptr1);
        if (!ret)

        {
          return IT_ERROR;
        }
        count += ret;
        ptr1->next = impegh_mp4_mem_node_malloc(m, sizeof(ia_mp4_es_id_inc_desc));
        ptr1 = ptr1->next;
        ptr1->tag = 0;
        ptr1->next = NULL;
        break;
      default:
        length = 0;
        for (i = 0; i < 4; i++)
        {
          ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
          if (ret < 1)
          {
            return IT_ERROR;
          }
          count++;
          if ((charbuf[0]) & 0x80)
            length = (length << 7) + (charbuf[0] & 0x7f);
          else
          {
            length = (length << 7) + (charbuf[0] & 0x7f);
            break;
          }
        }
        count += length;
        impegh_mp4_fseek(*fp, length, SEEK_CUR);
      }
    };
  }
  return IT_OK;
}

/**impegh_mp4_get_es_id_inctag
 *
 *  \brief Reads the ia_mp4_es_id_inc_desc info from the file
 *
 *  \param [in] fp	VOID pointer
 *  \param [in,out] n	Pointer to ia_mp4_es_id_inc_desc structure
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_get_es_id_inctag(VOID **fp, ia_mp4_es_id_inc_desc *n)
{
  UWORD8 charbuf[10];
  UWORD32 i, ret, count = 0, length;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  length = 0;
  for (i = 0; i < 4; i++)
  {
    ret = impegh_mp4_fread(charbuf, 1, 1, *fp);
    if (ret < 1)
    {
      return IT_ERROR;
    }
    count++;
    if ((charbuf[0]) & 0x80)
      length = (length << 7) + (charbuf[0] & 0x7f);
    else
    {
      length = (length << 7) + (charbuf[0] & 0x7f);
      break;
    }
  }
  if (i == 4)
  {
    return IT_ERROR;
  }
  n->length = length;

  ret = impegh_mp4_fread(charbuf, 1, 4, *fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  n->track_id = impegh_mp4_rev32(*data_size);
  count = count + 4;

  return count;
}
