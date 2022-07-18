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

#ifndef IMPEGHE_MAE_WRITE_H
#define IMPEGHE_MAE_WRITE_H

#define MAX_MAE_NUM_DATASETS (15)

#define ID_MAE_GROUP_DESCRIPTION (0)
#define ID_MAE_SWITCHGROUP_DESCRIPTION (1)
#define ID_MAE_GROUP_CONTENT (2)
#define ID_MAE_GROUP_COMPOSITE (3)
#define ID_MAE_SCREEN_SIZE (4)
#define ID_MAE_GROUP_PRESET_DESCRIPTION (5)
#define ID_MAE_DRC_UI_INFO (6)
#define ID_MAE_SCREEN_SIZE_EXTENSION (7)
#define ID_MAE_GROUP_PRESET_EXTENSION (8)
#define ID_MAE_LOUDNESS_COMPENSATION (9)

// Values obtained from Table 11 of section 4.8.2.2 of specification - 23008-3
// Restrictions applicable for LC profile
#define MAX_NUM_GROUPS (28)
#define MAX_NUM_SWITCH_GROUPS (14)
#define MAX_NUM_GROUPS_PRESETS (16)
#define MAX_GROUP_PRESET_NUM_CONDITIONS (16)
#define MAX_NUM_PRESET_GROUP_EXTENSIONS (16)
#define MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT (32)

// Group Definition
#define MAX_GROUP_NUM_MEMBERS (128)
#define MAX_SWITCH_GROUP_NUM_MEMBERS (32)

#define MAX_NUM_CONTENT_DATA_BLOCKS (128)

#define MAX_NUM_DESCRIPTOIN_BLOCKS (128)
#define MAX_NUM_DESCR_LANGUAGES (16)
#define MAX_DESCR_LANGUAGE_DATA_LEN (16)
#define MAX_DESCRIPTON_DATA_LEN (256)
#define MAX_NUM_COMPOSITE_PAIRS (128)
#define MAX_NUM_TGT_LOUDNESS_CONDITIONS (7)
#define MAX_NUM_PRESET_PROD_SCREENS (31)
#define MAX_MAE_CONFIG_EXTENSIONS (16)
// error codes
#define IMPEGHD_MHAS_SYNCWORD_MISMATCH (-1)

// structures

typedef struct
{
  WORD32 group_id;
  WORD32 allow_on_off;
  WORD32 default_on_off;
  WORD32 allow_pos_interact;
  WORD32 min_az_offset;
  WORD32 max_az_offset;
  WORD32 min_el_offset;
  WORD32 max_el_offset;
  WORD32 min_dist_factor;
  WORD32 max_dist_factor;
  WORD32 allow_gain_interact;
  WORD32 min_gain;
  WORD32 max_gain;
  WORD32 group_num_members;
  WORD32 has_conjunct_members;
  WORD32 start_id;
  WORD32 metadata_ele_id[MAX_GROUP_NUM_MEMBERS];
} ia_mae_group_def;

typedef struct
{
  WORD32 group_id;
  WORD32 allow_on_off;
  WORD32 default_on_off;
  WORD32 group_num_members;
  WORD32 member_id[MAX_SWITCH_GROUP_NUM_MEMBERS];
  WORD32 default_group_id;
} ia_mae_switch_group_def;

typedef struct
{
  WORD32 group_id;
  WORD32 preset_kind;
  WORD32 num_conditions[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 reference_id[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 cond_on_off[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 disable_gain_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 gain_flag[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 gain[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 disable_pos_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 position_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 azimuth_offset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 elevation_offset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 dist_factor[MAX_GROUP_PRESET_NUM_CONDITIONS];
} ia_mae_group_presets_def;

typedef struct
{
  WORD32 num_data_sets;
  WORD32 data_type[MAX_MAE_NUM_DATASETS];
  WORD32 data_length[MAX_MAE_NUM_DATASETS];
} ia_mae_data;

typedef struct
{
  WORD32 num_data_blocks;
  WORD32 data_group_id[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 content_kind[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 has_content_language[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 content_language[MAX_NUM_CONTENT_DATA_BLOCKS];
} ia_content_data;

typedef struct
{
  WORD32 num_desc_blocks;
  WORD32 group_id[MAX_NUM_DESCRIPTOIN_BLOCKS];
  WORD32 num_descr_languages[MAX_NUM_DESCRIPTOIN_BLOCKS];
  WORD32 descr_language[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD32 descr_data_length[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD32 descr_data[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES][MAX_DESCRIPTON_DATA_LEN];
} ia_description_data;

typedef struct
{
  WORD32 num_pairs;
  WORD32 ele_id[MAX_NUM_COMPOSITE_PAIRS][2];
} ia_composite_pair_data;

typedef struct
{
  WORD32 has_non_std_screen_size;
  WORD32 screen_size_az;
  WORD32 screen_size_el;
  WORD32 screen_size_bot_el;
} ia_production_screen_size_data;

typedef struct
{
  WORD32 overwrite_prod_screen_size_data;
  WORD32 num_preset_prod_screens;
  WORD32 default_screen_sz_left_az;
  WORD32 default_screen_sz_right_az;
  WORD32 screen_grp_preset_id[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 has_non_std_screen_sz[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 centered_in_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_left_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_right_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_top_el[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_bot_el[MAX_NUM_PRESET_PROD_SCREENS];
} ia_production_screen_size_ext_data;

typedef struct
{
  WORD32 group_loudness_present;
  WORD32 group_loudness[MAX_NUM_GROUPS];
  WORD32 default_params_present;
  WORD32 default_include_group[MAX_NUM_GROUPS];
  WORD32 default_min_max_gain_present;
  WORD32 default_min_gain;
  WORD32 default_max_gain;
  WORD32 preset_params_present[MAX_NUM_GROUPS_PRESETS];
  WORD32 preset_include_group[MAX_NUM_GROUPS_PRESETS][MAX_NUM_GROUPS];
  WORD32 preset_min_max_gain_present[MAX_NUM_GROUPS_PRESETS];
  WORD32 preset_min_gain[MAX_NUM_GROUPS_PRESETS];
  WORD32 preset_max_gain[MAX_NUM_GROUPS_PRESETS];
} ia_loudness_compensation_data;

typedef struct
{
  WORD32 version;
  WORD32 num_tgt_loudness_cnd;
  WORD32 tgt_loudness_value[MAX_NUM_TGT_LOUDNESS_CONDITIONS + 1];
  WORD32 set_effect_available[MAX_NUM_TGT_LOUDNESS_CONDITIONS];
} ia_drc_user_interface_info;

typedef struct
{
  WORD32 has_switch_group_conditions;
  WORD32 is_switch_group_condition[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                  [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 has_downmix_id_group_preset_extensions;
  WORD32 num_dmx_id_group_preset_ext;
  WORD32 group_preset_downmix_id[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_switch_group_id[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                     [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_group_id[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                              [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_condition_on_off[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                      [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_disable_gain_interactivity[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                                [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_gain_flag[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                               [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_gain[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_disable_position_interactivity[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                                    [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_position_flag[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                   [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_az_offset[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                               [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_el_offset[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                               [MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD32 group_preset_dist_factor[MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                 [MAX_GROUP_PRESET_NUM_CONDITIONS];
} ia_mae_group_presets_def_ext;

typedef struct
{
  // asi - audio scene in for
  // mae - metadata audio element
  WORD32 ei_present; // element interaction data
  WORD32 asi_present;
  WORD32 asi_config_set;
  WORD32 main_stream_flag;
  WORD32 asi_id_present;
  WORD32 asi_id;
  WORD32 num_groups;
  WORD32 num_switch_groups;
  WORD32 num_group_presets;
  WORD32 mae_id_offset;
  WORD32 mae_id_max_avail;
  UWORD8 mae_info_buf[MAX_MAE_CONFIG_EXTENSIONS][6144 / 8];
  ia_mae_group_def group_definition[MAX_NUM_GROUPS];
  ia_mae_switch_group_def switch_group_definition[MAX_NUM_SWITCH_GROUPS];
  ia_mae_group_presets_def group_presets_definition[MAX_NUM_GROUPS_PRESETS];
  ia_mae_group_presets_def_ext group_presets_definition_ext[MAX_NUM_GROUPS_PRESETS];
  ia_mae_data mae_data;
  ia_description_data group_desc_data;
  ia_description_data switch_group_desc_data;
  ia_description_data preset_desc_data;
  ia_content_data content_data;
  ia_composite_pair_data composite_pair_data;
  ia_production_screen_size_data screen_size_data;
  ia_loudness_compensation_data loud_comp_data;
  ia_drc_user_interface_info drc_interface_info;
  ia_production_screen_size_ext_data screen_size_ext_data;
} ia_mae_audio_scene_info;
typedef struct
{
  UWORD32 grp_priority[MAX_NUM_GROUPS];
  UWORD32 fixed_pos[MAX_NUM_GROUPS];

} ia_signal_grp_info;
WORD32 impeghe_signal_group_info(ia_bit_buf_struct *it_bit_buf,
                                 ia_signal_grp_info *pstr_signal_grp_info, WORD32 num_signal_grp,
                                 WORD32 *ptr_bit_cnt);

WORD32 impeghe_mae_asi_write(ia_bit_buf_struct *it_bit_buf, ia_mae_audio_scene_info *pstr_mae_asi,
                             WORD32 *ptr_bit_cnt);

#endif /*IMPEGHE_MAE_WRITE_H*/