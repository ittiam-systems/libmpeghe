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

#ifndef IMPEGHE_MEMORY_STANDARDS_H
#define IMPEGHE_MEMORY_STANDARDS_H

/*****************************************************************************/
/* type definitions                                                          */
/*****************************************************************************/

#define OAM_MAX_NUM_OBJECTS (24)
#define HOA_MAX_MATRIX (6)
#define HOA_MAX_COEFFS (49)
#define IMPEGHE_NUM_LS_CFGS (20)

#define OAM_HEADER_SIZE_BYTES (4)
#define OAM_VERSION_SIZE_BYTES (2)
#define OAM_DESCRIPTION_SIZE_BYTES (32)
#define OAM_CH_FILE_NAME_SIZE_BYTES (64)
#define OAM_OBJ_DESCRIPTION_SIZE_BYTES (64)

#define IMPEGHE_MAX_DMX_MATRICES_PER_ID (8)
#define IMPEGHE_MAX_DMX_MAX_GROUPS_ASSIGNED (16)
#define IMPEGHE_MAX_DMX_MATRIX_SIZE (256)   /* bytes */
#define IMPEGHE_MAX_DMX_MATRIX_ELEMENTS (6) /* Max number of downmix matrices one can embed */
#define MAX_NUM_OF_SPEAKERS (24)
#define MAX_NUM_ELEMENTS (128)
#define MAX_NUM_GRPS (16)
#define MAX_MAE_CONFIG_LINE_LEN (512)
#ifndef ABS
#define ABS(A) ((A) < 0 ? (-A) : (A))
#endif
/* standard memory table descriptor for libraries */
typedef struct
{
  UWORD32 ui_size;      /* size of the memory in bytes	*/
  UWORD32 ui_alignment; /* alignment in bytes 			*/
  UWORD32 ui_type;      /* type of memory 				*/
  UWORD32 ui_priority;  /* the importance for placement	*/
  UWORD32 ui_placed[2]; /* the o_red location for placement	*/
} ia_mem_info_struct;

typedef struct
{
  UWORD32 ui_size;
  UWORD32 ui_alignment;
  UWORD32 ui_type;
  pVOID mem_ptr;
} ia_mem_info_table;

typedef struct
{
  WORD32 i_channel_mask;
  WORD32 length;
  WORD32 n_channels;
  WORD32 sample_rate;
  WORD32 pcm_sz;
} ia_pcm_config;
typedef struct
{
  WORD8 ec_present;
  WORD32 ec_start_frame;
  WORD32 ec_frame_cnt;
  WORD32 ec_count;
  WORD32 *ptr_ec_buff;
  WORD8 ec_active;
  ia_pcm_config str_pcm_config;
} ia_ec_info_struct;
typedef struct
{
  WORD32 flex_spk_azi[MAX_NUM_OF_SPEAKERS];
  WORD32 flex_spk_ele[MAX_NUM_OF_SPEAKERS];
  WORD8 flex_spk_islfe[MAX_NUM_OF_SPEAKERS];
  WORD32 num_speaker;

} ia_flexi_spk_config;
typedef struct
{
  WORD32 main_stream_flag;
  WORD32 mae_id_offset;
  WORD32 mae_id_max_avail;
  WORD32 num_data_sets;
  WORD32 num_groups;
  WORD32 num_switch_groups;
  WORD32 num_group_presets;
  WORD32 grp_def_grp_id[MAX_NUM_ELEMENTS];
  WORD32 grp_def_allow_on_off[MAX_NUM_ELEMENTS];
  WORD32 grp_def_default_on_off[MAX_NUM_ELEMENTS];
  WORD32 grp_def_allow_pos_interact[MAX_NUM_ELEMENTS];
  WORD32 grp_def_min_az_offset[MAX_NUM_ELEMENTS];
  WORD32 grp_def_max_az_offset[MAX_NUM_ELEMENTS];
  WORD32 grp_def_min_el_offset[MAX_NUM_ELEMENTS];
  WORD32 grp_def_max_el_offset[MAX_NUM_ELEMENTS];
  WORD32 grp_def_min_dist_factor[MAX_NUM_ELEMENTS];
  WORD32 grp_def_max_dist_factor[MAX_NUM_ELEMENTS];
  WORD32 grp_def_allow_gain_interact[MAX_NUM_ELEMENTS];
  WORD32 grp_def_min_gain[MAX_NUM_ELEMENTS];
  WORD32 grp_def_max_gain[MAX_NUM_ELEMENTS];
  WORD32 grp_def_group_num[MAX_NUM_ELEMENTS];
  WORD32 grp_def_start_id[MAX_NUM_ELEMENTS];
  WORD32 grp_def_metadata_ele_id[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];

  WORD32 switch_grp_def_grp_id[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_def_allow_on_off[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_def_default_on_off[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_def_grp_num_member[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_def_grp_member_id[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 switch_grp_def_default_group_id[MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_grp_id[MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_preset_kind[MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_num_conditions[MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_disable_gain_interact[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_position_interact[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_reference_id[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_gain_flag[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_gain[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_disable_position_interact[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_cond_on_off[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_azimuth_offset[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_elevation_offset[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_preset_def_dist_factor[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];

  WORD32 group_loudness_present;
  WORD32 group_loudness[MAX_NUM_ELEMENTS];
  WORD32 default_params_present;
  WORD32 default_include_group[MAX_NUM_ELEMENTS];
  WORD32 default_min_max_gain_present;
  WORD32 default_min_gain;
  WORD32 default_max_gain;
  WORD32 preset_params_present[MAX_NUM_ELEMENTS];
  WORD32 preset_include_group[MAX_NUM_ELEMENTS][MAX_NUM_ELEMENTS];
  WORD32 preset_min_max_gain_present[MAX_NUM_ELEMENTS];
  WORD32 preset_min_gain[MAX_NUM_ELEMENTS];
  WORD32 preset_max_gain[MAX_NUM_ELEMENTS];
  WORD32 num_grp_def_decription_blocks;
  WORD32 grp_def_decription_grp_id[MAX_NUM_ELEMENTS];
  WORD32 num_grp_def_decription_languages[MAX_NUM_ELEMENTS];
  WORD32 grp_def_decription_languages[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_def_decription_data_length[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 grp_def_decription_data[MAX_NUM_GRPS][MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 num_switch_grp_decription_blocks;
  WORD32 switch_grp_decription_grp_id[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_num_decription_languages[MAX_NUM_ELEMENTS];
  WORD32 switch_grp_decription_languages[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 switch_grp_decription_data_length[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 switch_grp_decription_data[MAX_NUM_GRPS][MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 num_preset_decription_blocks;
  WORD32 preset_decription_grp_id[MAX_NUM_ELEMENTS];
  WORD32 preset_num_decription_languages[MAX_NUM_ELEMENTS];
  WORD32 preset_decription_languages[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 preset_decription_data_length[MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 preset_decription_data[MAX_NUM_GRPS][MAX_NUM_GRPS][MAX_NUM_ELEMENTS];
  WORD32 data_type[MAX_NUM_ELEMENTS];
  WORD32 decription_data[MAX_NUM_ELEMENTS];
  WORD32 num_content_data_blocks;
  WORD32 content_group_id[MAX_NUM_ELEMENTS];
  WORD32 content_kind[MAX_NUM_ELEMENTS];
  WORD32 has_content_language[MAX_NUM_ELEMENTS];
  WORD32 has_conjunct_members[MAX_NUM_ELEMENTS];
  WORD32 content_language[MAX_NUM_ELEMENTS];
  WORD32 num_comp_pairs;
  WORD32 element_id[MAX_NUM_ELEMENTS];
  WORD32 has_non_std_screen_size[MAX_NUM_ELEMENTS];
  WORD32 screen_size_az;
  WORD32 screen_size_el;
  WORD32 screen_size_bot_el[MAX_NUM_ELEMENTS];
  WORD32 default_screen_sz_left_az;
  WORD32 default_screen_sz_right_az;
  WORD32 num_preset_prod_screens;
  WORD32 screen_grp_preset_id[MAX_NUM_ELEMENTS];
  WORD32 centered_in_az[MAX_NUM_ELEMENTS];
  WORD32 screen_sz_left_az[MAX_NUM_ELEMENTS];
  WORD32 screen_sz_right_az[MAX_NUM_ELEMENTS];
  WORD32 screen_sz_top_el[MAX_NUM_ELEMENTS];
  WORD32 overwrite_prod_screen_size_data;

  WORD8 asi_enable;
} ia_asi_config;
typedef struct
{
  WORD32 dmx_id;
  WORD32 dmx_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 downmix_mtx_count;
  WORD32 num_assigned_group_ids[IMPEGHE_MAX_DMX_MATRICES_PER_ID];
  WORD32 signal_group_id[IMPEGHE_MAX_DMX_MATRICES_PER_ID][IMPEGHE_MAX_DMX_MAX_GROUPS_ASSIGNED];
  ia_dmx_matrix_enc_cfg_struct str_dmx_mtx_cfg;
} ia_mpeghe_ext_cfg_dmx_mtx_input_struct;

typedef struct
{
  UWORD32 dmx_config_type;
  UWORD32 passive_dmx_flag;
  UWORD32 phase_align_strength;
  UWORD32 immersive_downmix_flag;
  UWORD32 downmix_id_count;
  ia_mpeghe_ext_cfg_dmx_mtx_input_struct str_dmx_matrix[IMPEGHE_MAX_DMX_MATRIX_ELEMENTS];
} ia_mpeghe_ext_cfg_downmix_input_struct;

typedef struct
{
  WORD32 cplx_pred;
  WORD32 coding_mode;
  WORD32 fill_elem;
  WORD32 iframe_interval;
  WORD32 sample_rate;
  ia_pcm_config aud_ch_pcm_cfg;
  ia_pcm_config aud_obj_pcm_cfg;
  ia_pcm_config hoa_pcm_cfg;
  WORD32 num_trans_ch;
  WORD32 num_oam_ch;
  WORD32 codec_mode;
  WORD32 out_fmt;
  WORD32 mhas_pkt;
  WORD32 crc16;
  WORD32 crc32;
  WORD32 global_crc16;
  WORD32 global_crc32;
  WORD32 bitrate;
  WORD32 ltpf_enable;
  WORD32 num_ch_sig_groups;
  WORD32 num_ch_per_sig_grp[16];
  WORD32 num_ch_idx_per_grp[56];
  WORD32 num_obj_sig_groups;
  WORD32 num_objs_per_sig_grp[16];
  WORD32 num_hoa_sig_groups;
  WORD32 num_hoas_per_sig_grp[16];
  WORD32 enhanced_noise_filling;
  WORD32 igf_start_freq;
  WORD32 igf_stop_freq;
  WORD32 igf_start_freq_flag;
  WORD32 igf_stop_freq_flag;
  WORD32 igf_after_tns_synth;
  WORD32 fdp_enable;
  WORD32 noise_filling;
  WORD32 tns_enable;
  WORD32 prof_level;
  WORD32 prof_level_flag;
  WORD32 kernel;
  WORD32 prev_aliasing_symmetry;
  WORD32 curr_aliasing_symmetry;
  WORD32 full_band_lpd;
  WORD32 stereo_lpd;
  WORD32 mct_mode;
  UWORD16 num_objects;
  UWORD16 extra_objects;
  UWORD16 num_channels;
  WORD16 oam_version;
  WORD16 has_dyn_obj_priority;
  WORD16 has_uniform_spread;
  WORD32 use_oam_element;
  WORD32 user_specified_sig_grp;
  FLAG oam_high_rate;
  WORD32 cicp_index;
  WORD32 oam_replace_radius;
  FLAG oam_fixed_values[6];
  FLAG oam_has_core_length;
  FLAG oam_has_scrn_rel_objs;
  FLAG oam_is_scrn_rel_obj[OAM_MAX_NUM_OBJECTS];
  VOID *oam_data_hndl;
  WORD32 (*oam_read_data)(VOID *oam_data_hndl, UWORD8 *buff, WORD32 bytes_to_read);
  WORD32 (*oam_skip_data)(VOID *oam_data_hndl, WORD32 bytes_to_skip);
  WORD8 item_prefix[64];
  IA_ERRORCODE err_code;

  WORD8 asi_enable;
  ia_asi_config str_asi_config;
  WORD8 flexi_spk_enable;
  ia_flexi_spk_config str_flexi_spk_config;
  WORD32 use_hoa_element;
  WORD32 hoa_order;
  WORD32 uses_nfc;
  FLOAT32 nfc_distance;
  WORD32 num_hoa_coeffs;
  UWORD32 use_vec_est;
  // HOA matrix related
  WORD32 use_hoa_matrix;
  WORD32 num_hoa_matrix;
  WORD32 hoa_rend_id[HOA_MAX_MATRIX];
  WORD32 hoa_cicp[HOA_MAX_MATRIX];
  WORD32 hoa_matrix_in_dim[HOA_MAX_MATRIX];
  WORD32 hoa_matrix_out_dim[HOA_MAX_MATRIX];
  FLOAT64 hoa_matrix[HOA_MAX_MATRIX][HOA_MAX_COEFFS * MAX_NUM_OF_SPEAKERS];
  FLAG use_drc_element;
  ia_drc_input_config str_drc_cfg;
  FLAG use_downmix_ext_config;
  ia_mpeghe_ext_cfg_downmix_input_struct str_ext_cfg_downmix_input;
  ia_ec_info_struct str_ec_info_struct;
} ia_input_config;

typedef struct
{
  WORD32 i_samp_freq;
  WORD32 i_num_chan;
  WORD32 i_pcm_wd_sz;
  WORD32 i_channel_mask;
  UWORD32 ui_init_done;
  WORD32 num_out_bytes;
  WORD32 i_bytes_consumed;
  UWORD32 ui_inp_buf_size;
  UWORD32 malloc_count;
  UWORD32 ui_rem;
  UWORD32 ui_proc_mem_tabs_size;

  pVOID pv_ia_process_api_obj;
  pVOID arr_alloc_memory[100];

  WORD8 *p_lib_name;
  WORD8 *p_version_num;

  pVOID (*malloc_xaac)(UWORD32, UWORD32);
  pVOID (*de_malloc_xc)(pVOID);

  ia_mem_info_table mem_info_table[5];
  WORD32 i_dec_len;
  WORD32 i_total_length;
  WORD32 start_offset_samples;
  WORD32 input_size;
  WORD32 i_out_bytes;
  UWORD32 profile_info;
  WORD32 in_frame_length;
  WORD32 hoa_mtx_status;
} ia_output_config;

typedef struct
{
  ia_input_config input_config;
  ia_output_config output_config;
} ia_mpeghe_config_struct;

/*****************************************************************************/
/* Constant hash defines                                                     */
/*****************************************************************************/
/* when you don't need alignment, pass this to memory library */
#define IA_MEM_NO_ALIGN 0x01

/* ittiam standard memory types */
/* to be used inter frames */
#define IA_MEMTYPE_PERSIST 0x00
/* read write, to be used intra frames */
#define IA_MEMTYPE_SCRATCH 0x01
/* read only memory, intra frame */
#define IA_MEMTYPE_INPUT 0x02
/* read-write memory, for usable output, intra frame */
#define IA_MEMTYPE_OUTPUT 0x03
/* input buffer for HOA data */
#define IA_MEMTYPE_INPUT_HOA 0x04
/* readonly memory, inter frame */
#define IA_MEMTYPE_TABLE 0x05
/* input buffer before mem tabs allocation */
#define IA_MEMTYPE_PRE_FRAME_INPUT 0x06
/* input buffer before mem tabs allocation */
#define IA_MEMTYPE_PRE_FRAME_SCRATCH 0x07
/* for local variables */
#define IA_MEMTYPE_AUTO_VAR 0x80

/* ittiam standard memory priorities */
#define IA_MEMPRIORITY_ANYWHERE 0x00
#define IA_MEMPRIORITY_LOWEST 0x01
#define IA_MEMPRIORITY_LOW 0x02
#define IA_MEMPRIORITY_NORM 0x03
#define IA_MEMPRIORITY_ABOVE_NORM 0x04
#define IA_MEMPRIORITY_HIGH 0x05
#define IA_MEMPRIORITY_HIGHER 0x06
#define IA_MEMPRIORITY_CRITICAL 0x07

#define USAC_SWITCHED (0)
#define USAC_ONLY_FD (1)
#define USAC_ONLY_TD (2)

#define PROFILE_LC_LVL1 (0)
#define PROFILE_LC_LVL2 (1)
#define PROFILE_LC_LVL3 (2)
#define PROFILE_LC_LVL4 (3)
#define PROFILE_BL_LVL1 (4)
#define PROFILE_BL_LVL2 (5)
#define PROFILE_BL_LVL3 (6)

#define CONFIG_PROFILE_LC_LVL1 (0x0B)
#define CONFIG_PROFILE_LC_LVL2 (0x0C)
#define CONFIG_PROFILE_LC_LVL3 (0x0D)
#define CONFIG_PROFILE_LC_LVL4 (0x0E)
#define CONFIG_PROFILE_BL_LVL1 (0x10)
#define CONFIG_PROFILE_BL_LVL2 (0x11)
#define CONFIG_PROFILE_BL_LVL3 (0x12)

#endif /* IMPEGHE_MEMORY_STANDARDS_H */
