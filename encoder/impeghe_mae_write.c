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
#include "impeghe_error_standards.h"
#include "impeghe_mae_write.h"

/**
 *  impeghe_mae_asi_group_def
 *
 *  \brief Audio screen info group definition of metadata audio element
 *
 *  \param [out] it_bit_buf				Pointer to bit-buffer structure
 *  \param [in]  ptr_group_definition		Pointer to group definition structure
 *  \param [in]  num_groups					Number of groups
 *
 *  \return WORD32	Number of bits written
 */
static WORD32 impeghe_mae_asi_group_def(ia_bit_buf_struct *it_bit_buf,
                                        ia_mae_group_def *ptr_group_definition, WORD32 num_groups)
{
  WORD32 i, tmp, bits_written = 0;
  for (i = 0; i < num_groups; i++)
  {
    tmp = ptr_group_definition[i].group_id;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

    tmp = ptr_group_definition[i].allow_on_off;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    tmp = ptr_group_definition[i].default_on_off;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    tmp = ptr_group_definition[i].allow_pos_interact;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].min_az_offset;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

      tmp = ptr_group_definition[i].max_az_offset;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

      tmp = ptr_group_definition[i].min_el_offset;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

      tmp = ptr_group_definition[i].max_el_offset;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

      tmp = ptr_group_definition[i].min_dist_factor;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 4);

      tmp = ptr_group_definition[i].max_dist_factor;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 4);
    }
    tmp = ptr_group_definition[i].allow_gain_interact;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].min_gain;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 6);

      tmp = ptr_group_definition[i].max_gain;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);
    }
    tmp = ptr_group_definition[i].group_num_members - 1;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

    tmp = ptr_group_definition[i].has_conjunct_members;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_group_definition[i].start_id;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
    }
    else
    {
      WORD32 j = 0;
      for (j = 0; j < ptr_group_definition[i].group_num_members; j++)
      {
        tmp = ptr_group_definition[i].metadata_ele_id[j];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
      }
    }
  }
  return bits_written;
}

/**
 *  impeghe_mae_asi_switch_group_def
 *
 *  \brief Audio screen info switch group definition of metadata audio element
 *
 *  \param [out] it_bit_buf					Pointer to bit-buffer structure
 *  \param [in]  ptr_switch_group_definition	Pointer to MAE switch group definition structure
 *  \param [in]  num_switch_groups				Number of switch groups
 *
 *  \return WORD32	Number of bits written
 */
static WORD32
impeghe_mae_asi_switch_group_def(ia_bit_buf_struct *it_bit_buf,
                                 ia_mae_switch_group_def *ptr_switch_group_definition,
                                 WORD32 num_switch_groups)
{
  WORD32 i, j, tmp, bits_written = 0;
  for (i = 0; i < num_switch_groups; i++)
  {
    tmp = ptr_switch_group_definition[i].group_id;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

    tmp = ptr_switch_group_definition[i].allow_on_off;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_switch_group_definition[i].default_on_off;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);
    }

    tmp = ptr_switch_group_definition[i].group_num_members - 1;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

    for (j = 0; j < ptr_switch_group_definition[i].group_num_members; j++)
    {
      tmp = ptr_switch_group_definition[i].member_id[j];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
    }
    tmp = ptr_switch_group_definition[i].default_group_id;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
  }
  return bits_written;
}

/**
 *  impeghe_mae_asi_group_presets_def
 *
 *  \brief Audio screen info group preset definition of
 *			metadata audio element
 *
 *  \param [out] it_bit_buf					Pointer to bit-buffer structure
 *  \param [in]  ptr_group_presets_definition	Pointer to MAE group presets definition structure
 *  \param [in]  num_group_presets				Number of groups
 *
 *  \return WORD32 Number of bits written
 */
static WORD32
impeghe_mae_asi_group_presets_def(ia_bit_buf_struct *it_bit_buf,
                                  ia_mae_group_presets_def *ptr_group_presets_definition,
                                  WORD32 num_group_presets)
{
  WORD32 i, tmp, j, num_conditions, bits_written = 0;
  for (i = 0; i < num_group_presets; i++)
  {
    tmp = ptr_group_presets_definition[i].group_id;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

    tmp = ptr_group_presets_definition[i].preset_kind;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

    num_conditions = ptr_group_presets_definition[i].num_conditions[0] - 1;
    bits_written += impeghe_write_bits_buf(it_bit_buf, num_conditions, 4);

    for (j = 0; j < num_conditions + 1; j++)
    {
      tmp = ptr_group_presets_definition[i].reference_id[j];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

      tmp = ptr_group_presets_definition[i].cond_on_off[j];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

      if (tmp)
      {
        tmp = ptr_group_presets_definition[i].disable_gain_interact[j];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

        tmp = ptr_group_presets_definition[i].gain_flag[j];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);
        if (tmp)
        {
          tmp = ptr_group_presets_definition[i].gain[j];
          bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 8);
        }
        tmp = ptr_group_presets_definition[i].disable_pos_interact[j];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

        tmp = ptr_group_presets_definition[i].position_interact[j];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

        if (tmp)
        {
          tmp = ptr_group_presets_definition[i].azimuth_offset[j];
          bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 8);

          tmp = ptr_group_presets_definition[i].elevation_offset[j];
          bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 6);

          tmp = ptr_group_presets_definition[i].dist_factor[j];
          bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 4);
        }
      }
    }
  }
  return bits_written;
}

/**
 *  impeghe_mae_write_description_data
 *
 *  \brief Write description of metadata audio element
 *
 *  \param [out] it_bit_buf			Pointer to bit-buffer structure
 *  \param [in]  ptr_description_data	Pointer to description data structure
 *  \param [in]  group_id_bits			Group ID bits
 *  \param [out] ptr_bit_cnt			Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_write_description_data(ia_bit_buf_struct *it_bit_buf,
                                               ia_description_data *ptr_description_data,
                                               WORD32 group_id_bits, WORD32 *ptr_bit_cnt)
{
  WORD32 n, num_descr_blocks, tmp, i, c, bits_written = 0;
  num_descr_blocks = ptr_description_data->num_desc_blocks - 1;
  bits_written += impeghe_write_bits_buf(it_bit_buf, num_descr_blocks, 7);

  for (n = 0; n < num_descr_blocks + 1; n++)
  {
    tmp = ptr_description_data->group_id[n];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, group_id_bits);

    tmp = ptr_description_data->num_descr_languages[n];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 4);

    for (i = 0; i < ptr_description_data->num_descr_languages[n] + 1; i++)
    {
      tmp = ptr_description_data->descr_language[n][i];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 24);

      tmp = ptr_description_data->descr_data_length[n][i];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 8);

      for (c = 0; c < ptr_description_data->descr_data_length[n][i] + 1; c++)
      {
        tmp = ptr_description_data->descr_data[n][i][c];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 8);
      }
    }
  }
  *ptr_bit_cnt = bits_written;
}

/**
 *  impeghe_mae_write_drc_user_interface_info
 *
 *  \brief Write DRC user interface of metadata audio element
 *
 *  \param [out]    it_bit_buf				Pointer to bit-buffer structure
 *  \param [in,out] ptr_drc_user_ix_info	Pointer to DRC user interface information
 * structure
 *  \param [out]    ptr_bit_cnt				Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID
impeghe_mae_write_drc_user_interface_info(ia_bit_buf_struct *it_bit_buf,
                                          ia_drc_user_interface_info *ptr_drc_user_ix_info,
                                          WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, i;

  tmp = ptr_drc_user_ix_info->version;
  impeghe_write_bits_buf(it_bit_buf, tmp, 2);

  if (tmp == 0)
  {
    tmp = ptr_drc_user_ix_info->num_tgt_loudness_cnd;
    impeghe_write_bits_buf(it_bit_buf, tmp, 3);
    ptr_drc_user_ix_info->tgt_loudness_value[0] = -63;
    for (i = 0; i < ptr_drc_user_ix_info->num_tgt_loudness_cnd; i++)
    {
      tmp = ptr_drc_user_ix_info->tgt_loudness_value[i + 1];
      impeghe_write_bits_buf(it_bit_buf, tmp, 6);

      tmp = ptr_drc_user_ix_info->set_effect_available[i];
      impeghe_write_bits_buf(it_bit_buf, tmp, 16);
    }
  }
  *ptr_bit_cnt = it_bit_buf->cnt_bits;
}

/**
 *  impeghe_mae_write_loudness_comp_data
 *
 *  \brief Write loudess compensation of metadata audio element
 *
 *  \param [out] it_bit_buf Pointer to bit-buffer structure
 *  \param [in]  ptr_mae_asi Pointer to MAE audio scene information structure
 *  \param [out] ptr_bit_cnt Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_write_loudness_comp_data(ia_bit_buf_struct *it_bit_buf,
                                                 ia_mae_audio_scene_info *ptr_mae_asi,
                                                 WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, i;
  ia_loudness_compensation_data *ptr_loud_comp = &ptr_mae_asi->loud_comp_data;
  WORD32 num_groups = ptr_mae_asi->num_groups;
  WORD32 num_group_presets = ptr_mae_asi->num_group_presets;
  tmp = ptr_loud_comp->group_loudness_present;
  impeghe_write_bits_buf(it_bit_buf, tmp, 1);
  if (tmp)
  {
    for (i = 0; i < num_groups; i++)
    {
      tmp = ptr_loud_comp->group_loudness[i];
      impeghe_write_bits_buf(it_bit_buf, tmp, 8);
    }
  }
  tmp = ptr_loud_comp->default_params_present;
  impeghe_write_bits_buf(it_bit_buf, tmp, 1);

  if (tmp)
  {
    for (i = 0; i < num_groups; i++)
    {
      tmp = ptr_loud_comp->default_include_group[i];
      impeghe_write_bits_buf(it_bit_buf, tmp, 1);
    }
    tmp = ptr_loud_comp->default_min_max_gain_present;
    impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_loud_comp->default_min_gain;
      impeghe_write_bits_buf(it_bit_buf, tmp, 4);

      tmp = ptr_loud_comp->default_max_gain;
      impeghe_write_bits_buf(it_bit_buf, tmp, 4);
    }
  }
  for (i = 0; i < num_group_presets; i++)
  {
    tmp = ptr_loud_comp->preset_params_present[i];
    impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      WORD32 j;
      for (j = 0; j < num_groups; j++)
      {
        tmp = ptr_loud_comp->preset_include_group[i][j];
        impeghe_write_bits_buf(it_bit_buf, tmp, 1);
      }
      tmp = ptr_loud_comp->preset_min_max_gain_present[i];
      impeghe_write_bits_buf(it_bit_buf, tmp, 1);

      if (tmp)
      {
        tmp = ptr_loud_comp->preset_min_gain[i];
        impeghe_write_bits_buf(it_bit_buf, tmp, 4);

        tmp = ptr_loud_comp->preset_max_gain[i];
        impeghe_write_bits_buf(it_bit_buf, tmp, 4);
      }
    }
  }
  *ptr_bit_cnt = it_bit_buf->cnt_bits;
}

/**
 *  impeghe_mae_write_content_data
 *
 *  \brief Writes MAE content data
 *
 *  \param [out] it_bit_buf		Pointer to bit-buffer structure
 *  \param [in]  ptr_content_data	Pointer to content data structure
 *  \param [out] ptr_bit_cnt		Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_write_content_data(ia_bit_buf_struct *it_bit_buf,
                                           ia_content_data *ptr_content_data, WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, n, num_data_blocks, bits_written = 0;
  num_data_blocks = ptr_content_data->num_data_blocks - 1;
  bits_written += impeghe_write_bits_buf(it_bit_buf, num_data_blocks, 7);
  for (n = 0; n < num_data_blocks + 1; n++)
  {
    tmp = ptr_content_data->data_group_id[n];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

    tmp = ptr_content_data->content_kind[n];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 4);

    tmp = ptr_content_data->has_content_language[n];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_content_data->content_language[n];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 24);
    }
  }
  *ptr_bit_cnt = bits_written;
}

/**
 *  impeghe_mae_write_composite_pair
 *
 *  \brief Writes composite pair information
 *
 *  \param [out] it_bit_buf		Pointer to bit-buffer structure
 *  \param [in]  ptr_content_data	Pointer to composite pair data structure
 *  \param [out] ptr_bit_cnt		Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_write_composite_pair(ia_bit_buf_struct *it_bit_buf,
                                             ia_composite_pair_data *ptr_content_data,
                                             WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, i, num_comp_pairs, bits_written = 0;
  num_comp_pairs = ptr_content_data->num_pairs;
  bits_written += impeghe_write_bits_buf(it_bit_buf, num_comp_pairs - 1, 7);

  for (i = 0; i < num_comp_pairs; i++)
  {
    tmp = ptr_content_data->ele_id[i][0];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

    tmp = ptr_content_data->ele_id[i][1];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
  }
  *ptr_bit_cnt = bits_written;
}

/**
 *  impeghe_mae_write_prod_screen_sz_data
 *
 *  \brief Writes production screen size data
 *
 *  \param [out] it_bit_buf		Pointer to bit-buffer structure
 *  \param [in]  ptr_screen_sz_data	Pointer to production screen size data structure
 *  \param [out] ptr_bit_cnt		Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID
impeghe_mae_write_prod_screen_sz_data(ia_bit_buf_struct *it_bit_buf,
                                      ia_production_screen_size_data *ptr_screen_sz_data,
                                      WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, bits_written = 0;

  tmp = ptr_screen_sz_data->has_non_std_screen_size;
  bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

  if (tmp)
  {
    tmp = ptr_screen_sz_data->screen_size_az;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);

    tmp = ptr_screen_sz_data->screen_size_el;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);

    tmp = ptr_screen_sz_data->screen_size_bot_el;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);
  }
  *ptr_bit_cnt = bits_written;
}

/**
 *  impeghe_mae_write_prod_screen_sz_data_ext
 *
 *  \brief Writes production screen size extension data
 *
 *  \param [out] it_bit_buf			Pointer to bit-buffer structure
 *  \param [in]  ptr_screen_sz_ext_data Pointer to production screen size extension data structure
 *  \param [out] ptr_bit_cnt            Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_write_prod_screen_sz_data_ext(
    ia_bit_buf_struct *it_bit_buf, ia_production_screen_size_ext_data *ptr_screen_sz_ext_data,
    WORD32 *ptr_bit_cnt)
{
  WORD32 tmp, i, bits_written = 0;
  tmp = ptr_screen_sz_ext_data->overwrite_prod_screen_size_data;
  bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);
  if (tmp)
  {
    tmp = ptr_screen_sz_ext_data->default_screen_sz_left_az;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 10);

    tmp = ptr_screen_sz_ext_data->default_screen_sz_right_az;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 10);
  }
  tmp = ptr_screen_sz_ext_data->num_preset_prod_screens;
  bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

  for (i = 0; i < ptr_screen_sz_ext_data->num_preset_prod_screens; i++)
  {
    tmp = ptr_screen_sz_ext_data->screen_grp_preset_id[i];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);

    tmp = ptr_screen_sz_ext_data->has_non_std_screen_sz[i];
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

    if (tmp)
    {
      tmp = ptr_screen_sz_ext_data->centered_in_az[i];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

      if (tmp)
      {
        tmp = ptr_screen_sz_ext_data->screen_sz_left_az[i];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);
      }
      else
      {
        tmp = ptr_screen_sz_ext_data->screen_sz_left_az[i];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 10);

        tmp = ptr_screen_sz_ext_data->screen_sz_right_az[i];
        bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 10);
      }
      tmp = ptr_screen_sz_ext_data->screen_sz_top_el[i];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);
      tmp = ptr_screen_sz_ext_data->screen_sz_bot_el[i];
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 9);
    }
  }
  *ptr_bit_cnt = bits_written;
}

/**
 *  impeghe_mae_asi_group_presets_def_ext
 *
 *  \brief Writes ASI group presets definition extension
 *
 *  \param [out] it_bit_buf					Pointer to bit-buffer structure
 *  \param [in]  ptr_group_presets_definition	Pointer to group presets definition structure
 *  \param [in]  num_group_presets				Number of groups
 *  \param [out] ptr_bit_cnt					Pointer to number of bits written
 *
 *  \return VOID
 */
static VOID impeghe_mae_asi_group_presets_def_ext(
    ia_bit_buf_struct *it_bit_buf, ia_mae_group_presets_def_ext *pstr_group_presets_def_ext,
    WORD32 num_group_presets, WORD32 *ptr_bit_cnt, WORD32 *num_conditions)
{
  WORD32 bits_written = 0;
  WORD32 grp, cnt, idx;
  for (grp = 0; grp < num_group_presets; grp++)
  {
    ia_mae_group_presets_def_ext pstr_presets_def_ext = pstr_group_presets_def_ext[grp];
    bits_written +=
        impeghe_write_bits_buf(it_bit_buf, pstr_presets_def_ext.has_switch_group_conditions, 1);
    if (pstr_presets_def_ext.has_switch_group_conditions)
    {
      for (cnt = 0; cnt < num_conditions[0] + 1; cnt++)
      {
        bits_written += impeghe_write_bits_buf(
            it_bit_buf, pstr_presets_def_ext.is_switch_group_condition[cnt][0], 1);
      }
    }
    bits_written += impeghe_write_bits_buf(
        it_bit_buf, pstr_presets_def_ext.has_downmix_id_group_preset_extensions, 1);

    if (pstr_presets_def_ext.has_downmix_id_group_preset_extensions)
    {
      bits_written += impeghe_write_bits_buf(
          it_bit_buf, pstr_presets_def_ext.num_dmx_id_group_preset_ext - 1, 5);

      for (cnt = 0; cnt < pstr_presets_def_ext.num_dmx_id_group_preset_ext; cnt++)
      {
        bits_written += impeghe_write_bits_buf(
            it_bit_buf, pstr_presets_def_ext.group_preset_downmix_id[cnt] - 1, 7);
        bits_written += impeghe_write_bits_buf(it_bit_buf, num_conditions[cnt] - 1, 4);

        for (idx = 0; idx < num_conditions[cnt] - 1; idx++)
        {
          bits_written += impeghe_write_bits_buf(
              it_bit_buf, pstr_presets_def_ext.is_switch_group_condition[cnt][idx], 1);
          if (pstr_presets_def_ext.is_switch_group_condition[cnt][idx])
          {
            bits_written += impeghe_write_bits_buf(
                it_bit_buf, pstr_presets_def_ext.group_preset_switch_group_id[cnt][idx] - 1, 5);
          }
          else
          {
            bits_written += impeghe_write_bits_buf(
                it_bit_buf, pstr_presets_def_ext.group_preset_group_id[cnt][idx] - 1, 7);
          }
          bits_written += impeghe_write_bits_buf(
              it_bit_buf, pstr_presets_def_ext.group_preset_condition_on_off[cnt][idx] - 1, 1);
          if (pstr_presets_def_ext.group_preset_condition_on_off[cnt][idx])
          {
            bits_written += impeghe_write_bits_buf(
                it_bit_buf,
                pstr_presets_def_ext.group_preset_disable_gain_interactivity[cnt][idx] - 1, 1);
            bits_written += impeghe_write_bits_buf(
                it_bit_buf, pstr_presets_def_ext.group_preset_gain_flag[cnt][idx] - 1, 1);
            if (pstr_presets_def_ext.group_preset_gain_flag[cnt][idx])
            {
              bits_written += impeghe_write_bits_buf(
                  it_bit_buf, pstr_presets_def_ext.group_preset_gain[cnt][idx] - 1, 8);
            }
            bits_written += impeghe_write_bits_buf(
                it_bit_buf,
                pstr_presets_def_ext.group_preset_disable_position_interactivity[cnt][idx] - 1,
                1);
            bits_written += impeghe_write_bits_buf(
                it_bit_buf, pstr_presets_def_ext.group_preset_position_flag[cnt][idx] - 1, 1);
            if (pstr_presets_def_ext.group_preset_position_flag[cnt][idx])
            {
              bits_written += impeghe_write_bits_buf(
                  it_bit_buf, pstr_presets_def_ext.group_preset_az_offset[cnt][idx] - 1, 8);
              bits_written += impeghe_write_bits_buf(
                  it_bit_buf, pstr_presets_def_ext.group_preset_el_offset[cnt][idx] - 1, 6);
              bits_written += impeghe_write_bits_buf(
                  it_bit_buf, pstr_presets_def_ext.group_preset_dist_factor[cnt][idx] - 1, 4);
            }
          }
        }
      }
    }
  }
  *ptr_bit_cnt = bits_written;
  return;
}

/**
 *  impeghe_mae_asi_data_write
 *
 *  \brief Write audio scene info data of metadata audio element
 *
 *  \param [out] it_bit_buf	Pointer to bit-buffer structure
 *  \param [in]  ptr_mae_asi	Pointer to MAE ASI structure
 *
 *  \return WORD32 Number of bits written
 */
WORD32 impeghe_mae_asi_data_write(ia_bit_buf_struct *it_bit_buf,
                                  ia_mae_audio_scene_info *ptr_mae_asi)
{
  WORD32 i = 0, data_type, j, bits_written = 0;
  WORD32 num_data_sets = 0;
  WORD32 bit_cnt_ext;
  ia_bit_buf_struct it_bit_buf_local;
  ia_mae_data *ptr_mae_data = &ptr_mae_asi->mae_data;
  num_data_sets = ptr_mae_data->num_data_sets;
  bits_written += impeghe_write_bits_buf(it_bit_buf, num_data_sets, 4);
  for (i = 0; i < num_data_sets; i++)
  {
    data_type = ptr_mae_data->data_type[i];
    switch (data_type)
    {
    case ID_MAE_GROUP_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->group_desc_data, 7,
                                         &bit_cnt_ext);

      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_SWITCHGROUP_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->switch_group_desc_data,
                                         5, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_GROUP_CONTENT:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_content_data(&it_bit_buf_local, &ptr_mae_asi->content_data, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_GROUP_COMPOSITE:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_composite_pair(&it_bit_buf_local, &ptr_mae_asi->composite_pair_data,
                                       &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_SCREEN_SIZE:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_prod_screen_sz_data(&it_bit_buf_local, &ptr_mae_asi->screen_size_data,
                                            &bit_cnt_ext);

      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_GROUP_PRESET_DESCRIPTION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_description_data(&it_bit_buf_local, &ptr_mae_asi->preset_desc_data, 5,
                                         &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_DRC_UI_INFO:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_drc_user_interface_info(&it_bit_buf_local,
                                                &ptr_mae_asi->drc_interface_info, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_SCREEN_SIZE_EXTENSION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_prod_screen_sz_data_ext(&it_bit_buf_local,
                                                &ptr_mae_asi->screen_size_ext_data, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_GROUP_PRESET_EXTENSION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_asi_group_presets_def_ext(
          &it_bit_buf_local, &ptr_mae_asi->group_presets_definition_ext[i],
          ptr_mae_asi->num_group_presets, &bit_cnt_ext,
          ptr_mae_asi->group_presets_definition[i].num_conditions);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    case ID_MAE_LOUDNESS_COMPENSATION:
    {
      bit_cnt_ext = 0;
      impeghe_create_bit_buffer(&it_bit_buf_local, &(ptr_mae_asi->mae_info_buf[i][0]),
                                sizeof(ptr_mae_asi->mae_info_buf[i]));
      impeghe_mae_write_loudness_comp_data(&it_bit_buf_local, ptr_mae_asi, &bit_cnt_ext);
      ptr_mae_data->data_length[i] = (bit_cnt_ext + 7) >> 3;
      break;
    }
    default:
      break;
    }
    bits_written += impeghe_write_bits_buf(it_bit_buf, data_type, 4);
    bits_written += impeghe_write_bits_buf(it_bit_buf, ptr_mae_data->data_length[i], 16);
    for (j = 0; j < ptr_mae_data->data_length[i]; j++)
    {
      bits_written +=
          impeghe_write_bits_buf(it_bit_buf, (UWORD32)ptr_mae_asi->mae_info_buf[i][j], 8);
    }
  }
  return bits_written;
}
/**
 *  impeghe_signal_group_info
 *
 *  \brief Write Signal group information to the bit buffer
 *
 *  \param [out]    it_bit_buf             Pointer to bit-buffer structure
 *  \param [out]    ptr_bit_cnt             Pointer to number of bits written
 *  \param [in]     pstr_signal_grp_info    Pointer to signal group info structure
 *  \param [in]     num_signal_grp          Num of signal group
 *
 *  \return WORD32 Number of Bits written
 *
 */
WORD32 impeghe_signal_group_info(ia_bit_buf_struct *it_bit_buf,
                                 ia_signal_grp_info *pstr_signal_grp_info, WORD32 num_signal_grp,
                                 WORD32 *ptr_bit_cnt)
{
  for (WORD32 i = 0; i < num_signal_grp + 1; i++)
  {
    impeghe_write_bits_buf(it_bit_buf, pstr_signal_grp_info->grp_priority[i], 3);
    impeghe_write_bits_buf(it_bit_buf, pstr_signal_grp_info->fixed_pos[i], 1);
  }
  *ptr_bit_cnt = it_bit_buf->cnt_bits;
  return 0;
}

/**
 *  impeghe_mae_asi_write
 *
 *  \brief Write audio scene info of metadata audio element
 *
 *  \param [out]    it_bit_buf	Pointer to bit-buffer structure
 *  \param [in,out] ptr_mae_asi	Pointer to MAE ASI structure
 *  \param [out]    ptr_bit_cnt Pointer to number of bits written
 *
 *  \return WORD32
 */
WORD32 impeghe_mae_asi_write(ia_bit_buf_struct *it_bit_buf, ia_mae_audio_scene_info *ptr_mae_asi,
                             WORD32 *ptr_bit_cnt)
{
  WORD32 tmp = 0, bits_written = 0;
  tmp = ptr_mae_asi->main_stream_flag;
  bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);

  if (tmp)
  {
    tmp = ptr_mae_asi->asi_id_present;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 1);
    if (tmp)
    {
      tmp = ptr_mae_asi->asi_id;
      bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 8);
    }
    /* ASI - Group Definition*/
    tmp = ptr_mae_asi->num_groups;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
    bits_written += impeghe_mae_asi_group_def(it_bit_buf, &ptr_mae_asi->group_definition[0], tmp);
    /* ASI - Switch Group Definition*/
    tmp = ptr_mae_asi->num_switch_groups;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);
    bits_written += impeghe_mae_asi_switch_group_def(
        it_bit_buf, &ptr_mae_asi->switch_group_definition[0], tmp);

    /* ASI - Group presets Definition*/
    tmp = ptr_mae_asi->num_group_presets;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 5);
    bits_written += impeghe_mae_asi_group_presets_def(
        it_bit_buf, &ptr_mae_asi->group_presets_definition[0], tmp);

    /* ASI - MAE Data*/
    bits_written += impeghe_mae_asi_data_write(it_bit_buf, ptr_mae_asi);

    ptr_mae_asi->mae_id_offset = 0;
    tmp = ptr_mae_asi->mae_id_max_avail;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
    ptr_mae_asi->mae_id_offset = 0;
  }
  else
  {
    tmp = ptr_mae_asi->mae_id_offset;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);

    tmp = ptr_mae_asi->mae_id_max_avail;
    bits_written += impeghe_write_bits_buf(it_bit_buf, tmp, 7);
  }
  *ptr_bit_cnt = bits_written;
  return 0;
}
