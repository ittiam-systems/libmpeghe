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

#ifndef IMPEGH_MP4_DEMUX_UTILS_H
#define IMPEGH_MP4_DEMUX_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "impegh_type_def.h"
#include "impegh_mp4_file.h"
#include "impegh_mp4_utils.h"

#define IT_MP4_BUF_SIZE 16
#define MHAS_PAC_TYP_MPEGH3DACFG (1)
#define MHAS_PAC_TYP_MPEGH3DAFRAME (2)
#define MHAS_PAC_TYP_SYNC (6)
#define MHAS_PAC_TYP_FILLDATA (0)
#define MHAS_PAC_TYP_MPEGH3DACFG (1)
#define MHAS_PAC_TYP_MPEGH3DAFRAME (2)
#define MHAS_PAC_TYP_AUDIOSCENEINFO (3)
#define MHAS_PAC_TYP_SYNC (6)
#define MHAS_PAC_TYP_SYNCGAP (7)
#define MHAS_PAC_TYP_MARKER (8)
#define MHAS_PAC_TYP_CRC16 (9)
#define MHAS_PAC_TYP_CRC32 (10)
#define MHAS_PAC_TYP_DESCRIPTOR (11)
#define MHAS_PAC_TYP_USERINTERACTION (12)
#define MHAS_PAC_TYP_LOUDNESS_DRC (13)
#define MHAS_PAC_TYP_BUFFERINFO (14)
#define MHAS_PAC_TYP_GLOBAL_CRC16 (15)
#define MHAS_PAC_TYP_GLOBAL_CRC32 (16)
#define MHAS_PAC_TYP_AUDIOTRUNCATION (17)
#define MHAS_PAC_TYP_GENDATA (18)
#define MHAS_PAC_TYP_EARCON (19)
#define MHAS_PAC_TYP_PCMCONFIG (20)
#define MHAS_PAC_TYP_PCMDATA (21)
#define MHAS_PAC_TYP_LOUDNESS (22)

#define DEFAULT_REF_DIST (177)

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

#define MAX_MAE_NUM_DATASETS (15)
#define MAX_NUM_EARCONS (16)
#define MAX_NUM_EARCON_LANGUAGES (16)
#define MAX_NUM_EARCON_TXT_DATA_LENTGH (256)
#define MAX_NUM_PCM_SAMPLES (1024)
#define MHAS_PAC_TYP_UNDEF (518)
#define MHAS_SYNC_BYTE (0xA5)
#define MAX_MAE_CONFIG_EXTENSIONS (3)
// Values obtained from Table 11 of section 4.8.2.2 of specification - 23008-3
// Restrictions applicable for LC profile
#define MAX_GROUP_PRESET_NUM_CONDITIONS (16)
#define MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT (32)
#define MAX_NUM_GROUP_CONDITIONS (16)
#define MAX_NUM_GROUPS (16)
#define MAX_NUM_GROUPS_PRESETS (8)
#define MAX_NUM_PRESET_GROUP_EXTENSIONS (8)
#define MAX_NUM_SWITCH_GROUPS (8)

// Group Definition
#define MAX_GROUP_NUM_MEMBERS (128)
#define MAX_DESCR_LANGUAGE_DATA_LEN (16)
#define MAX_DESCRIPTON_DATA_LEN (16)
#define MAX_NUM_COMPOSITE_PAIRS (128)
#define MAX_NUM_CONTENT_DATA_BLOCKS (128)
#define MAX_NUM_DESCRIPTOIN_BLOCKS (16)
#define MAX_NUM_DESCR_LANGUAGES (4)
#define MAX_NUM_PRESET_PROD_SCREENS (31)
#define MAX_NUM_TGT_LOUDNESS_CONDITIONS (7)
#define MAX_SWITCH_GROUP_NUM_MEMBERS (32)
#define MAX_VALUE (MAX_NUM_DESCRIPTOIN_BLOCKS * MAX_NUM_DESCR_LANGUAGES * MAX_DESCRIPTON_DATA_LEN)

#define MAX_MAEI_LENGTH (7000)
#define MAX_HEADER_LENGTH (150)
#define MHM1_TYPE 0x6D686D31
#define MHA1_TYPE 0x6D686131

// structures
typedef struct
{
  WORD32 packet_lbl;
  WORD32 packet_length;
  WORD32 packet_type;
} ia_mhas_pac_info;

typedef struct
{
  WORD8 allow_gain_interact;
  WORD8 allow_on_off;
  WORD8 allow_pos_interact;
  WORD8 default_on_off;
  WORD8 group_id;
  WORD8 group_num_members;
  WORD8 has_conjunct_members;
  WORD8 max_az_offset;
  WORD8 max_dist_factor;
  WORD8 max_el_offset;
  WORD8 has_contentLanguage;
  WORD8 max_gain;
  WORD8 min_az_offset;
  WORD8 min_dist_factor;
  WORD8 min_el_offset;
  WORD8 min_gain;
  WORD8 start_id;
  WORD8 metadata_ele_id[MAX_GROUP_NUM_MEMBERS];
} ia_mae_group_def;

typedef struct
{
  WORD8 allow_on_off;
  WORD8 default_on_off;
  WORD8 group_id;
  WORD8 group_num_members;
  WORD8 member_id[MAX_SWITCH_GROUP_NUM_MEMBERS];
  WORD32 default_group_id;
} ia_mae_switch_group_def;

typedef struct
{
  WORD32 group_id;
  WORD32 preset_kind;
  WORD32 num_conditions;
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
  WORD8 num_data_sets;
  WORD8 data_type[MAX_MAE_NUM_DATASETS];
  WORD16 data_length[MAX_MAE_NUM_DATASETS];
} ia_mae_data;

typedef struct
{
  WORD8 array[50000];

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
  UWORD8 mae_info_buf[MAX_MAE_CONFIG_EXTENSIONS][6144];
  ia_mae_group_def group_definition[MAX_NUM_GROUPS];
  ia_mae_switch_group_def switch_group_definition[MAX_NUM_SWITCH_GROUPS];
  ia_mae_group_presets_def group_presets_definition[MAX_NUM_GROUPS_PRESETS];
  ia_mae_data mae_data;
  ia_description_data group_desc_data;
  ia_description_data switch_group_desc_data;
  ia_description_data preset_desc_data;
} ia_mae_audio_scene_info;
typedef struct ia_bit_buf_struct_e
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;

  WORD32 bit_pos;
  WORD32 cnt_bits;
  WORD32 size;
  WORD32 error;

  WORD32 bit_count;
  WORD32 valid_bits;
  UWORD8 byte;
  UWORD8 *byte_ptr;
  UWORD8 *ptr_start;
  WORD32 write_bit_count;
  WORD32 max_size;

} ia_bit_buf_struct_e;
typedef struct ia_bit_buf_struct_d
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;
  UWORD8 *ptr_write_next;

  WORD32 read_position;
  WORD32 write_position;
  WORD32 cnt_bits;
  WORD32 size;

} ia_bit_buf_struct_d;

WORD32 impegh_mp4_get_mael(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size);
WORD32 impegh_mp4_get_maep(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size);
WORD32 impegh_mp4_get_maes(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size);
WORD32 impegh_mp4_get_maeg(ia_bit_buf_struct_e *ptr_read_buff,
                           ia_bit_buf_struct_d *ptr_write_buff, UWORD32 read_size);
WORD32 impegh_mp4_write_maeg_buf(ia_bit_buf_struct_d *ptr_bit_buf,
                                 ia_mae_group_def *ptr_group_definition, WORD32 num_groups);
ia_bit_buf_struct_d *impegh_create_bit_buffer(ia_bit_buf_struct_d *it_bit_buf,
                                              UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size,
                                              WORD32 init);
ia_bit_buf_struct_e *impegh_create_read_buf(ia_bit_buf_struct_e *it_bit_buff,
                                            UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size);
UWORD8 impegh_write_bits_buf(ia_bit_buf_struct_d *it_bit_buf, UWORD32 write_val, UWORD8 num_bits);
WORD32 impegh_mhas_write_cfg_header(ia_bit_buf_struct_d *it_bit_buff, UWORD8 *data,
                                    UWORD32 num_bits);
WORD32 impegh_mhas_write_maei_header(ia_bit_buf_struct_d *it_bit_buff, UWORD8 *data,
                                     UWORD32 num_bits);
WORD32 impegh_mhas_write_frame_header(ia_bit_buf_struct_d *it_bit_buff, UWORD32 num_bits);
WORD32 impegh_write_mhas_pkt_header(ia_mhas_pac_info *pkt_info, ia_bit_buf_struct_d *it_bit_buff);
WORD32 impegh_write_mhas_pkt(ia_bit_buf_struct_d *it_bit_buff, ia_mhas_pac_info *pkt_info,
                             UWORD8 *data, UWORD32 num_bits);
WORD32 impegh_write_escape_value(ia_bit_buf_struct_d *it_bit_buff, UWORD32 value,
                                 UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3);
IA_ERRORCODE impegh_mp4_find_stsz(it_avi_file_ctxt *itf, WORD32 *offset, WORD32 *stsz_size);

WORD32 impegh_mp4_fread(pVOID buffer, WORD32 size, WORD32 count, it_avi_file_ctxt *itf);
WORD32 impegh_mp4_fseek(it_avi_file_ctxt *itf, WORD32 offset, WORD32 origin);
WORD32 impegh_mp4_feof(it_avi_file_ctxt *itf);
#endif /* IMPEGH_MP4_DEMUX_UTILS_H */