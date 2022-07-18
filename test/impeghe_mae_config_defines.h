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

#ifndef IMPEGHE_MAE_CONFIG_DEFINES_H
#define IMPEGHE_MAE_CONFIG_DEFINES_H

#define STRING_MAIN_STREAM_FLAG "main_stream_flag:"
#define STRING_MAE_ID_OFFSET "mae_id_offset:"
#define STRING_MAE_ID_MAX_AVAIL "mae_id_max_avail:"
#define STRING_NUM_GROUPS "num_groups:"
#define STRING_GROUP_DFN_GROUP_ID "group_definition.group_id:"
#define STRING_GROUP_DFN_ALLOW_ON_OFF "group_definition.allow_on_off:"
#define STRING_GROUP_DFN_DEFAULT_ON_OFF "group_definition.default_on_off:"
#define STRING_GROUP_DFN_ALLOW_POS_INTERACT "group_definition.allow_pos_interact:"
#define STRING_GROUP_DFN_MIN_AZ_OFFSET "group_definition.min_az_offset:"
#define STRING_GROUP_DFN_MAX_AZ_OFFSET "group_definition.max_az_offset:"
#define STRING_GROUP_DFN_MIN_EL_OFFSET "group_definition.min_el_offset:"
#define STRING_GROUP_DFN_MAX_EL_OFFSET "group_definition.max_el_offset:"
#define STRING_GROUP_DFN_MIN_DIST_FACTOR "group_definition.min_dist_factor:"
#define STRING_GROUP_DFN_MAX_DIST_FACTOR "group_definition.max_dist_factor:"
#define STRING_GROUP_DFN_ALLOW_GAIN_FACTOR "group_definition.allow_gain_interact:"
#define STRING_GROUP_DFN_MIN_GAIN "group_definition.min_gain:"
#define STRING_GROUP_DFN_MAX_GAIN "group_definition.max_gain:"
#define STRING_GROUP_DFN_START_ID "group_definition.start_id:"
#define STRING_GROUP_DFN_HAS_CONJUNCT_MEM "group_definition.has_conjunct_member:"
#define STRING_GROUP_DFN_GRP_NUM "group_definition.group_num:"
#define STRING_GROUP_DFN_META_ELE_ID "group_definition.metadata_ele_id:"
#define STRING_NUM_SWITCH_GRP "num_switch_groups:"
#define STRING_SWITCH_GRP_DEFN_GRP_ID "switch_group_definition.group_id:"
#define STRING_SWITCH_GRP_DEFN_ALLOW_ON_OFF "switch_group_definition.allow_on_off:"
#define STRING_SWITCH_GRP_DEFN_GRP_NUM_MEM "switch_group_definition.group_num_members:"
#define STRING_SWITCH_GRP_DEFN_MEM_ID "switch_group_definition.member_id:"
#define STRING_SWITCH_GRP_DEFN_DEFAULT_ON_OFF "switch_group_definition.default_on_off:"
#define STRING_SWITCH_GRP_DEFN_DEFAULT_GRP_ID "switch_group_definition.default_grp_id:"
#define STRING_NUM_GRP_PRESETS "num_group_presets:"
#define STRING_GRP_PRESET_DEFN_GRP_ID "group_preset_definition.grp_id:"
#define STRING_GRP_PRESET_DEFN_PRESET_KIND "group_preset_definition.preset_kind:"
#define STRING_GRP_PRESET_DEFN_NUM_CONDITION "group_preset_definition.num_conditions:"
#define STRING_GRP_PRESET_DEFN_REF_ID "group_preset_definition.reference_id:"
#define STRING_GRP_PRESET_DEFN_CON_ON_OFF "group_preset_definition.cond_on_off:"
#define STRING_GRP_PRESET_DEFN_GAIN_FLAG "group_preset_definition.gain_flag:"
#define STRING_GRP_PRESET_DEFN_GAIN "group_preset_definition.gain:"
#define STRING_GRP_PRESET_DEFN_DISABLE_GAIN_INTRCT                                               \
  "group_preset_definition.disable_gain_interact:"
#define STRING_GRP_PRESET_DEFN_DISABLE_POSITION_INTRCT                                           \
  "group_preset_definition.disable_position_interact:"
#define STRING_GRP_PRESET_DEFN_DISABLE_POS_INTRCT "group_preset_definition.position_interact:"
#define STRING_GRP_PRESET_DEFN_AZ_OFFSET "group_preset_definition.azimuth_offset:"
#define STRING_GRP_PRESET_DEFN_EL_OFFSET "group_preset_definition.elevation_offset:"
#define STRING_GRP_PRESET_DEFN_DIST_FACTOR "group_preset_definition.dist_factor:"
#define STRING_NUM_DATA_SETS "num_data_sets:"
#define STRING_DATA_TYPE "data_type:"
#define STRING_NUM_GROUP_DEF_DSCRPTN_BLOCKS "num_grp_def_decription_blocks:"
#define STRING_GROUP_DEF_DSCRPTN_GRP_ID "grp_def_decription_grp_id:"
#define STRING_GROUP_DEF_NUM_DSCRPTN_LANGUAGES "num_grp_def_decription_language:"
#define STRING_GROUP_DEF_DSCRPTN_LANGUAGES "grp_def_decription_languages:"
#define STRING_GROUP_DEF_DSCRPTN_DATA_LENGTH "grp_def_decription_data_length:"
#define STRING_GROUP_DEF_DSCRPTN_DATA "grp_def_decription_data:"
#define STRING_NUM_SWITCH_GRP_DSCRPTN_BLOCKS "num_switch_grp_decription_blocks:"
#define STRING_SWITCH_GRP_DSCRPTN_GRP_ID "switch_grp_decription_grp_id:"
#define STRING_SWITCH_GRP_NUM_DSCRPTN_LANGUAGES "num_switch_grp_decription_language:"
#define STRING_SWITCH_GRP_DSCRPTN_LANGUAGES "switch_grp_decription_languages:"
#define STRING_SWITCH_GRP_DSCRPTN_DATA_LENGTH "switch_grp_decription_data_length:"
#define STRING_SWITCH_GRP_DSCRPTN_DATA "switch_grp_decription_data:"
#define STRING_NUM_PRESET_DSCRPTN_BLOCKS "num_preset_decription_blocks:"
#define STRING_PRESET_DSCRPTN_GRP_ID "preset_decription_grp_id:"
#define STRING_PRESET_NUM_DSCRPTN_LANGUAGES "num_preset_decription_language:"
#define STRING_PRESET_DSCRPTN_LANGUAGES "preset_decription_languages:"
#define STRING_PRESET_DSCRPTN_DATA_LENGTH "preset_decription_data_length:"
#define STRING_PRESET_DSCRPTN_DATA "preset_decription_data:"
#define STRING_NUM_CONTENT_DATA_BLOCKS "num_content_data_blocks:"
#define STRING_CONTENT_GRP_ID "content_group_id:"
#define STRING_CONTENT_KIND "content_kind:"
#define STRING_HAS_CONTENT_LANGUAGE "has_content_language:"
#define STRING_CONTENT_LANGUAGE "content_language:"
#define STRING_NUM_COMP_PAIRS "num_comp_pairs:"
#define STRING_ELEMENT_ID "element_id:"
#define STRING_HAS_NON_STD_SCRN_SIZE "has_non_std_screen_size:"
#define STRING_SCRN_SIZE_AZ "screen_size_az:"
#define STRING_SCRN_SIZE_EL "screen_size_el:"
#define STRING_SCRN_SIZE_BOT_EL "screen_size_bot_el:"
#define STRING_OVERWRITE_PRO_SCRN_SIZE_DATA "overwrite_prod_screen_size_data:"
#define STRING_DFLT_SCRN_SZ_LEFT_AZ "default_screen_sz_left_az:"
#define STRING_DFLT_SCRN_SZ_RIGHT_AZ "default_screen_sz_right_az:"
#define STRING_NUM_PRESET_PROD_SCRNS "num_preset_prod_screens:"
#define STRING_SCRN_GRP_PRESET_ID "screen_grp_preset_id:"
#define STRING_CENTERED_IN_AZ "centered_in_az:"
#define STRING_SCRN_SZ_LEFT_AZ "screen_sz_left_az:"
#define STRING_SCRN_SZ_RIGHT_AZ "screen_sz_right_az:"
#define STRING_SCRN_SZ_TOP_EL "screen_sz_top_el:"
#define STRING_GRP_LOUDNESS "grp_loudness:"
#define STRING_DFLT_PARAM_PRESENT "defaultlt_param_present:"
#define STRING_DFLT_INCL_GRP "default_include_group:"
#define STRING_DFLT_MIN_MAX_GAIN_PRESENT "default_min_max_gain_present:"
#define STRING_DFLT_MIN_GAIN "default_min_gain:"
#define STRING_DFLT_MAX_GAIN "default_max_gain:"
#define STRING_PRESET_PARAM_PRESENT "preset_param_present:"
#define STRING_PRESET_INCLUDE_GRP "preset_include_group:"
#define STRING_PRESET_MIN_MAX_PRESENT "preset_min_max_present:"
#define STRING_PRESET_MIN_GAIN "preset_min_gain:"
#define STRING_PRESET_MAX_GAIN "preset_max_gain:"

#endif /*IMPEGHE_MAE_CONFIG_DEFINES_H*/
