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

   In addition to the warranty disclaimers contained in these licenses,
   Ittiam Systems Pvt. Ltd. makes the following disclaimers regarding the
   third-party components on behalf of itself, its affiliates and investors,
   its contributors, the copyright holders, and the licensors of the
   third-party components:

   YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT LICENSE(S) BY THIRD
   PARTIES, INCLUDING, WITHOUT LIMITATION, THE PATENT OWNERS LISTED AT
   [LINK TO POOL LICENSORS -
   https://www.via-corp.com/licensing/mpeg-h/mpeg-h-3d-licensors/] TO THE
   FULLEST EXTENT PERMITTED UNDER APPLICABLE LAW, THE SOFTWARE IS PROVIDED
   “AS IS,” AND ANY REPRESENTATIONS OR WARRANTIES OF ANY KIND, WHETHER ORAL
   OR WRITTEN, WHETHER EXPRESS, IMPLIED, OR ARISING BY STATUTE, CUSTOM, COURSE
   OF DEALING, OR TRADE USAGE, INCLUDING WITHOUT LIMITATION THE IMPLIED
   WARRANTIES OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
   AND NONINFRINGEMENT, ARE DISCLAIMED. IN NO EVENT WILL ITTIAM (AND ITS
   AFFILIATES AND INVESTORS), THE COPYRIGHT OWNERS, CONTRIBUTORS, OR LICENSORS,
   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION), HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.

---------------------------------------------------------------
*/

// For strnlen
#define _POSIX_C_SOURCE 200809L

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "libavutil/opt.h"
#include "codec_internal.h"
#include "encode.h"
#include "internal.h"
#include "audio_frame_queue.h"

#include "impeghe_type_def.h"
#include "impeghe_api.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_handler.h"
#include "impeghe_apicmd_standards.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_drc_user_config.h"
#include "impeghe_error_codes.h"
#include "impeghe_mp4_writer.h"

// Constant hash defines
#define IA_MAX_CMD_LINE_LENGTH 300
#define MAX_HOA_IN_FILES 50

#define IA_MHAC_CONFIG_VERSION 1
#define IA_MHAC_PROFILE_LEVEL_INDICATION 1
#define IA_MHAC_REFRENCE_CHANNEL_LAYOUT 1
#define IA_MHAC_LENGTH 2

#define IA_OUTPUT_FMT 1
// Global Variables
ia_input_config *pv_input;
ia_output_config *pv_output;
pVOID p_ia_mpeghe_obj;
int flag = 1;
FILE *g_pf_inps[56];
WORD8 pb_oam_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
WORD8 pb_oam_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
FILE *g_pf_inp_ham = NULL;
FILE *g_pf_inp_hoa_mtx = NULL;
FILE *g_pf_hoa_input[50];
FLAG g_is_hoa_input = 0;
WORD32 g_num_hoa_coeffs = 0;
ia_pcm_config g_inp_hoa_config[MAX_HOA_IN_FILES];
WORD8 pb_hoa_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
WORD8 g_pb_hoa_input_file_names[MAX_HOA_IN_FILES][IA_MAX_CMD_LINE_LENGTH];

// FOr MHA1 or MHM
int temp_op_fmt;

typedef struct config_params {
	int bit_rate;
	int output_format;
	int cicp_layout_index;
	int n_channels;
	int sample_rate;
	int pcm_sz;
	char *oam_file;
	char *hoa_file;
} config_params;

typedef struct IA_MPEGH_Context {

	const AVClass *class;
	void *pv_ia_process_api_obj;
	pWORD8 pb_inp_buf;
	pWORD8 pb_out_buf;
	pWORD8 pb_inp_hoa_buf;
	pWORD8 temp_buf;
	int LEN;
	// For OAM
	FILE * g_oam_inp;
	pWORD8 pb_oam_buf;
	WORD32 num_channels_to_encode;
	// For HOA

	config_params params;
	AudioFrameQueue afq;

} IA_MPEGH_Context;

static const AVOption ia_mpegh_enc_options[] = {
	{"ia_bit_rate","Stream Bit Rate",offsetof(IA_MPEGH_Context,params.bit_rate),AV_OPT_TYPE_INT,{.i64 = 64000},32000,512000,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_op_fmt","Output Format",offsetof(IA_MPEGH_Context,params.output_format),AV_OPT_TYPE_INT,{.i64 = 1},1,3,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_cicp","CICP Layout Index",offsetof(IA_MPEGH_Context,params.cicp_layout_index),AV_OPT_TYPE_INT,{.i64 = 1},1,20,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_chans","No of Channels",offsetof(IA_MPEGH_Context,params.n_channels),AV_OPT_TYPE_INT,{.i64 = 1},1,24,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_sample_rate","Sample Rate",offsetof(IA_MPEGH_Context,params.sample_rate),AV_OPT_TYPE_INT,{.i64 = 48000},14700,48000,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_pcm_sz","PCM Word Size",offsetof(IA_MPEGH_Context,params.pcm_sz),AV_OPT_TYPE_INT,{.i64 = 16},16,32,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_oam_file","OAM File Name",offsetof(IA_MPEGH_Context,params.oam_file),AV_OPT_TYPE_STRING,{.str = NULL},0,0,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{"ia_hoa_file","First HOA File Name that ends with 00+.wav",offsetof(IA_MPEGH_Context,params.hoa_file),AV_OPT_TYPE_STRING,{.str = NULL},0,0,AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM},
	{NULL}
};

static const AVClass ia_mpegh_enc_class = {
	.class_name = "ia_mpeghe",
	.item_name = av_default_item_name,
	.option = ia_mpegh_enc_options,
	.version = LIBAVUTIL_VERSION_INT,
};

//
extern VOID impeghe_error_handler_init();
extern VOID impeghe_testbench_error_handler_init();

extern ia_error_info_struct ia_testbench_error_info;
extern ia_error_info_struct ia_mpeghe_error_info;

IA_ERRORCODE ia_mpegh_encode_set_default_param(ia_input_config *pstr_input_config);
IA_ERRORCODE ia_mpegh_encode_set_config_param(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx);
pVOID malloc_global(UWORD32 size, UWORD32 alignment);
void ia_reconfigure_mhaC_tag_data(pWORD8 pb_out_buf,int profile_info,AVCodecContext *avctx);
WORD32 impeghe_fread(VOID *buf, WORD32 size, WORD32 bytes, FILE *fp);
static WORD32 impeghe_read_oam_data(VOID *hndl, UWORD8 *buff, WORD32 bytes_to_read);
static WORD32 impeghe_skip_oam_data(VOID *hndl, WORD32 bytes_to_skip);
static IA_ERRORCODE impeghe_read_oam_header(FILE *oam_file, ia_input_config *ptr_in_cfg,WORD32 *num_channels_to_encode);
IA_ERRORCODE impeghe_wav_header_decode(FILE *in_file, pVOID ptr_pcm_cfg);
WORD32 open_in_files(WORD8 *first_hoa_file_name, WORD32 *hoa_order, WORD32 *num_hoa_coeffs);
void ia_mpegh_handle_oam(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx);
void ia_mpegh_handle_hoa(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx);

//
IA_ERRORCODE ia_mpegh_encode_set_default_param(ia_input_config *pstr_input_config)
{
	LOOPIDX idx;
	pstr_input_config->aud_ch_pcm_cfg.pcm_sz = 16;
	pstr_input_config->aud_ch_pcm_cfg.sample_rate = 44100;

	pstr_input_config->aud_obj_pcm_cfg.pcm_sz = 16;
	pstr_input_config->aud_obj_pcm_cfg.sample_rate = 44100;

	pstr_input_config->hoa_pcm_cfg.pcm_sz = 16;
	pstr_input_config->hoa_pcm_cfg.sample_rate = 44100;

	pstr_input_config->codec_mode = USAC_ONLY_FD;
	pstr_input_config->bitrate = 32000;
	pstr_input_config->fdp_enable = 0;
	pstr_input_config->noise_filling = 0;
	pstr_input_config->enhanced_noise_filling = 1;
	pstr_input_config->igf_after_tns_synth = 1;
	pstr_input_config->tns_enable = 1;
	pstr_input_config->cplx_pred = 0;
	pstr_input_config->fill_elem = 1;
	pstr_input_config->prof_level = PROFILE_LC_LVL1;
	pstr_input_config->mhas_pkt = 1;
	pstr_input_config->crc16 = 0;
	pstr_input_config->crc32 = 0;
	pstr_input_config->global_crc16 = 0;
	pstr_input_config->global_crc32 = 0;
	pstr_input_config->mct_mode = -1;
	// OAM Params
	pstr_input_config->use_oam_element = 0;
	pstr_input_config->use_drc_element = 0;
	pstr_input_config->use_hoa_element = 0;
	pstr_input_config->oam_high_rate = 1;
	pstr_input_config->oam_replace_radius = 0;
	for (idx = 0; idx < 6; idx++)
	{
		pstr_input_config->oam_fixed_values[idx] = 0;
	}
	pstr_input_config->oam_has_core_length = 0;
	pstr_input_config->oam_has_scrn_rel_objs = 0;
	for (idx = 0; idx < OAM_MAX_NUM_OBJECTS; idx++)
	{
		pstr_input_config->oam_is_scrn_rel_obj[idx] = 0;
	}
	pstr_input_config->oam_data_hndl = 0;
	pstr_input_config->oam_read_data = 0;
	pstr_input_config->oam_skip_data = 0;
	pstr_input_config->kernel = 0;
	return IA_NO_ERROR;
}

IA_ERRORCODE ia_mpegh_encode_set_config_param(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx)
{

	pstr_input_config->bitrate = ctx->params.bit_rate;
	pstr_input_config->out_fmt = ctx->params.output_format;
	pstr_input_config->cicp_index = ctx->params.cicp_layout_index;
	pstr_input_config->aud_ch_pcm_cfg.n_channels = ctx->params.n_channels;
	pstr_input_config->aud_ch_pcm_cfg.sample_rate = ctx->params.sample_rate;
	pstr_input_config->aud_ch_pcm_cfg.pcm_sz = ctx->params.pcm_sz;
	pstr_input_config->mct_mode = -1;
	pstr_input_config->use_vec_est = -1;

	temp_op_fmt = ctx->params.output_format;

	if(ctx->params.oam_file != NULL)
	{
		ia_mpegh_handle_oam(pstr_input_config,ctx);
	}
	if(ctx->params.hoa_file != NULL)
	{
		ia_mpegh_handle_hoa(pstr_input_config,ctx);
	}
	// 1 - RAW_MHAS, 2 - MP4_MHA1 and 3 - MP4_MHM1
	if(pstr_input_config->out_fmt == 2)
	{
		pstr_input_config->mhas_pkt = 0;
	}
	else
	{
		pstr_input_config->mhas_pkt = 1;
	}
	return IA_NO_ERROR;
}

pVOID malloc_global(UWORD32 size, UWORD32 alignment)
{
	return malloc(size + alignment);
}

void ia_reconfigure_mhaC_tag_data(pWORD8 pb_out_buf,int profile_info,AVCodecContext *avctx)
{
	// Config version = 1
	WORD8 *word8_ptr;
	WORD16 *word16_ptr;
	WORD32 byte_align4=0;

	word8_ptr = (WORD8 *)avctx->extradata;
	*word8_ptr++ = 0x01;

	// mpegh3daProfileLevelIndication
	*word8_ptr++ = profile_info;

	// referenceChannelLayout
	*word8_ptr++ = 0x02;

	word16_ptr = (WORD16 *)word8_ptr;
	// length
	// BYTE_SWAP_UINT16
	*word16_ptr++ = ((avctx->extradata_size >> 8) | (avctx->extradata_size << 8));

	word8_ptr = (WORD8 *)word16_ptr;

	// mpegh3daConfig
	memcpy(word8_ptr,pb_out_buf,avctx->extradata_size);

	word8_ptr+=avctx->extradata_size;

	// dummy to make 4 byte aligned
	byte_align4 = (4-((avctx->extradata_size + IA_MHAC_CONFIG_VERSION + IA_MHAC_PROFILE_LEVEL_INDICATION + IA_MHAC_REFRENCE_CHANNEL_LAYOUT + IA_MHAC_LENGTH)%4));
	for(int i=0;i<byte_align4;++i)
	{
		*word8_ptr++ = 0;
	}
	avctx->extradata_size += byte_align4;
	avctx->extradata_size += IA_MHAC_CONFIG_VERSION;
	avctx->extradata_size += IA_MHAC_PROFILE_LEVEL_INDICATION;
	avctx->extradata_size += IA_MHAC_REFRENCE_CHANNEL_LAYOUT;
	avctx->extradata_size += IA_MHAC_LENGTH;
	avctx->extradata_size += IA_OUTPUT_FMT;
}

WORD32 impeghe_fread(VOID *buf, WORD32 size, WORD32 bytes, FILE *fp)
{
	return (WORD32)fread(buf, size, bytes, fp);
}

static WORD32 impeghe_read_oam_data(VOID *hndl, UWORD8 *buff, WORD32 bytes_to_read)
{
	if (!hndl || !buff || 0 >= bytes_to_read)
	{
		return 0;
	}

	return impeghe_fread(buff, 1, bytes_to_read, (FILE *)hndl);
}

static WORD32 impeghe_skip_oam_data(VOID *hndl, WORD32 bytes_to_skip)
{
	if (!hndl)
	{
		return -1;
	}

	fseek((FILE *)hndl, bytes_to_skip, SEEK_CUR);

	return 0;
}

static IA_ERRORCODE impeghe_read_oam_header(FILE *oam_file, ia_input_config *pstr_input_config,WORD32 *num_channels_to_encode)
{
	WORD32 idx;
	WORD32 bytes_read;
	UWORD8 temp_buff[OAM_CH_FILE_NAME_SIZE_BYTES];
	UWORD16 *ptr_16_temp_buff;
	WORD16 *ptr_16_word_temp_buff;
	const char object_idx[32][8] = {
		"000.wav", "001.wav", "002.wav", "003.wav", "004.wav", "005.wav", "006.wav", "007.wav",
		"008.wav", "009.wav", "010.wav", "011.wav", "012.wav", "013.wav", "014.wav", "015.wav",
		"016.wav", "017.wav", "018.wav", "019.wav", "020.wav", "021.wav", "022.wav", "023.wav",
		"024.wav", "025.wav", "026.wav", "027.wav", "028.wav", "029.wav", "030.wav"};

	bytes_read = impeghe_fread(temp_buff, 1, OAM_HEADER_SIZE_BYTES + OAM_VERSION_SIZE_BYTES, oam_file);
	if (bytes_read != OAM_HEADER_SIZE_BYTES + OAM_VERSION_SIZE_BYTES)
	{
		return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
	}
	/* OAM Header: id */
	temp_buff[OAM_HEADER_SIZE_BYTES - 1] = '\0';
	if (strncmp((const char *)temp_buff, "OAM", OAM_HEADER_SIZE_BYTES) != 0)
	{
		/* Invalid OAM header */
		return IMPEGHE_CONFIG_FATAL_OAM_INVALID_HEADER;
	}

	/* OAM Header: version */
	ptr_16_word_temp_buff = ((WORD16 *)&temp_buff[OAM_HEADER_SIZE_BYTES]);
	pstr_input_config->oam_version = *ptr_16_word_temp_buff;
	if (pstr_input_config->oam_version > 4)
	{
		/* Invalid OAM header */
		return IMPEGHE_CONFIG_FATAL_OAM_INVALID_HEADER;
	}

	if (pstr_input_config->oam_version > 2)
	{
		bytes_read = impeghe_fread(&pstr_input_config->has_dyn_obj_priority, 1, 2, oam_file);
		if (bytes_read != 2)
		{
		/* Invalid OAM header */
			return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
		}

		if (pstr_input_config->has_dyn_obj_priority)
		{
			pstr_input_config->has_dyn_obj_priority = 1;
		}
	}

	if (pstr_input_config->oam_version > 3)
	{
		bytes_read = impeghe_fread(&pstr_input_config->has_uniform_spread, 1, 2, oam_file);
		if (bytes_read != 2)
		{
		/* Invalid OAM header */
			return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
		}

		if (pstr_input_config->has_uniform_spread)
		{
			pstr_input_config->has_uniform_spread = 1;
		}
	}
	else
	{
		pstr_input_config->has_uniform_spread = 1;
	}

	/* OAM Header: num_channels and num_objects */
	bytes_read = impeghe_fread(temp_buff, 1, 4, oam_file);
	if (bytes_read != 4)
	{
		/* Invalid OAM header */
		return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
	}

	ptr_16_temp_buff = ((UWORD16 *)temp_buff);
	pstr_input_config->num_channels = *ptr_16_temp_buff;
	ptr_16_temp_buff = ((UWORD16 *)(temp_buff + 2));
	pstr_input_config->num_objects = *ptr_16_temp_buff;

	if (pstr_input_config->num_objects > 24)
	{
		pstr_input_config->extra_objects = pstr_input_config->num_objects - 24;
		pstr_input_config->num_objects = 24;
		pstr_input_config->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
	}

	if ((pstr_input_config->num_objects + pstr_input_config->num_channels) > 24)
	{
		pstr_input_config->extra_objects = pstr_input_config->num_objects + pstr_input_config->num_channels - 24;
		pstr_input_config->num_objects = 24 - pstr_input_config->num_channels;
		pstr_input_config->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
	}

	/* OAM Header: description */
	bytes_read = impeghe_fread((pWORD8 *)temp_buff, 1, OAM_DESCRIPTION_SIZE_BYTES, oam_file);
	if (bytes_read != OAM_DESCRIPTION_SIZE_BYTES)
	{
		/* Invalid OAM header */
		return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
	}

	/* OAM Header: channel filenames */
	for (idx = 0; idx < pstr_input_config->num_channels; idx++)
	{
		UWORD8 item_name_buf[64] = {0};
		UWORD8 oam_file_path[300] = {0};
		bytes_read = impeghe_fread(temp_buff, 1, OAM_CH_FILE_NAME_SIZE_BYTES, oam_file);
		if (bytes_read != OAM_CH_FILE_NAME_SIZE_BYTES)
		{
			/* Invalid OAM header */
			return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
		}
		memcpy(oam_file_path, pb_oam_file_path, strlen((const char *)pb_oam_file_path));
		memcpy(item_name_buf, pstr_input_config->item_prefix, 6);
		strcat((char *)item_name_buf, (const char *)temp_buff);
		strncat((char *)oam_file_path, (char *)item_name_buf,strnlen((const char *)item_name_buf, 64));
		g_pf_inps[idx] = fopen((const char *)oam_file_path, "rb");
		if (NULL == g_pf_inps[idx])
		{
			printf("channel input file open failed\n");
			return -1;
		}
	}

	/* OAM Header: object describtions */
	for (idx = 0; idx < pstr_input_config->num_objects; idx++)
	{
		UWORD8 item_name_buf[64] = {0};
		UWORD8 oam_file_path[300] = {0};
		bytes_read = impeghe_fread(temp_buff, 1, OAM_OBJ_DESCRIPTION_SIZE_BYTES, oam_file);
		if (bytes_read != OAM_CH_FILE_NAME_SIZE_BYTES)
		{
			/* Invalid OAM header */
			return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
		}
		memcpy(oam_file_path, pb_oam_file_path, strlen((const char *)pb_oam_file_path));
		memcpy(item_name_buf, pstr_input_config->item_prefix,strlen((const char *)pstr_input_config->item_prefix) - 4);
		strncat((char *)item_name_buf, "_", 1);
		strncat((char *)item_name_buf, object_idx[idx], 7);
		strncat((char *)oam_file_path, (char *)item_name_buf,strnlen((const char *)item_name_buf, 64));
		g_pf_inps[pstr_input_config->num_channels + idx] = fopen((const char *)oam_file_path, "rb");
		if (NULL == g_pf_inps[pstr_input_config->num_channels + idx])
		{
			fseek(oam_file, -OAM_OBJ_DESCRIPTION_SIZE_BYTES, SEEK_CUR);
			pstr_input_config->extra_objects += pstr_input_config->num_objects - idx;
			pstr_input_config->num_objects = idx > 24 ? 24 : idx;
			pstr_input_config->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
		}
	}

	for (idx = 0; idx < pstr_input_config->extra_objects; idx++)
	{
		bytes_read = impeghe_fread(temp_buff, 1, OAM_OBJ_DESCRIPTION_SIZE_BYTES, oam_file);
		if (bytes_read != OAM_CH_FILE_NAME_SIZE_BYTES)
		{
			/* Invalid OAM header */
			return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
		}
	}

	*num_channels_to_encode = pstr_input_config->num_channels + pstr_input_config->num_objects;

	return 0;

}

IA_ERRORCODE impeghe_wav_header_decode(FILE *in_file, pVOID ptr_pcm_cfg)
{
	ia_pcm_config *pstr_pcm_cfg = (ia_pcm_config *)ptr_pcm_cfg;
	WORD8 wav_hdr[40 + 36];
	WORD8 data_start[4];
	WORD16 num_ch;
	UWORD32 f_samp;
	WORD16 output_format;
	WORD32 check, count = 0;
	FLAG wav_format_pcm = 0, wav_format_extensible = 0;
	UWORD16 cbSize = 0;
	WORD8 type;
	UWORD8 offset;
	UWORD8 fmt_tag_found = 0;
	UWORD16 data_offset = 0;

	pstr_pcm_cfg->i_channel_mask = 0;

	fseek(in_file, 0, SEEK_SET);

	if (fread(wav_hdr, 1, 76, in_file) != 76)
		return 1;

	if (wav_hdr[0] != 'R' && wav_hdr[1] != 'I' && wav_hdr[2] != 'F' && wav_hdr[3] != 'F')
	{
		return 1;
	}

	/* Search for "fmt" tag as rest of config needs to be read after this tag */
	offset = 4;
	while (((offset + 4) < 76))
	{
		type = wav_hdr[offset];
		(offset)++;
		if (type == 'f')
		{
		if (wav_hdr[offset] == 'm')
		{
			(offset)++;
			if (wav_hdr[offset] == 't')
			{
			offset += 2; /* Last byte in 'fmt' is invalid (1) */
			fmt_tag_found = 1;
			offset += 4; /* size of fmt subchunk size */
			break;
			}
		}
		}
	}

	if (fmt_tag_found != 1)
		return 1;

	if (wav_hdr[offset] == 01 && wav_hdr[offset + 1] == 00)
	{
		wav_format_pcm = 1;
	}
	else if (wav_hdr[offset] == ((WORD8)0xFE) && wav_hdr[offset + 1] == ((WORD8)0xFF))
	{
		wav_format_extensible = 1;
	}
	else if (wav_hdr[offset] == ((WORD8)0x03) && wav_hdr[offset + 1] == ((WORD8)0x00))
	{
		wav_format_extensible = 2;
	}
	else
	{
		return 1;
	}

	num_ch = (WORD16)((UWORD8)wav_hdr[offset + 3] * 256 + (UWORD8)wav_hdr[offset + 2]);
	f_samp = ((UWORD8)wav_hdr[offset + 7] * 256 * 256 * 256);
	f_samp += ((UWORD8)wav_hdr[offset + 6] * 256 * 256);
	f_samp += ((UWORD8)wav_hdr[offset + 5] * 256);
	f_samp += ((UWORD8)wav_hdr[offset + 4]);

	output_format = ((UWORD8)wav_hdr[offset + 15] * 256);
	output_format += ((UWORD8)wav_hdr[offset + 14]);

	pstr_pcm_cfg->n_channels = num_ch;
	pstr_pcm_cfg->sample_rate = f_samp;
	pstr_pcm_cfg->pcm_sz = output_format;

	if ((wav_format_pcm) || (2 == wav_format_extensible))
	{
		data_start[0] = wav_hdr[offset + 16];
		data_start[1] = wav_hdr[offset + 17];
		data_start[2] = wav_hdr[offset + 18];
		data_start[3] = wav_hdr[offset + 19];
		data_offset = offset + 20;
	}
	else if (1 == wav_format_extensible)
	{
		cbSize |= ((UWORD8)wav_hdr[offset + 17] << 8);
		cbSize |= ((UWORD8)wav_hdr[offset + 16]);

		pstr_pcm_cfg->i_channel_mask = 0;
		pstr_pcm_cfg->i_channel_mask |= (UWORD8)wav_hdr[offset + 23] << 24;
		pstr_pcm_cfg->i_channel_mask |= (UWORD8)wav_hdr[offset + 22] << 16;
		pstr_pcm_cfg->i_channel_mask |= (UWORD8)wav_hdr[offset + 21] << 8;
		pstr_pcm_cfg->i_channel_mask |= (UWORD8)wav_hdr[offset + 20];

		data_start[0] = wav_hdr[offset + 20 + cbSize - 2 + 0];
		data_start[1] = wav_hdr[offset + 20 + cbSize - 2 + 1];
		data_start[2] = wav_hdr[offset + 20 + cbSize - 2 + 2];
		data_start[3] = wav_hdr[offset + 20 + cbSize - 2 + 3];
		data_offset = offset + 20 + cbSize - 2 + 4;
	}

	check = 1;
	fseek(in_file, data_offset, SEEK_SET);
	while (check)
	{
		if (data_start[0] == 'd' && data_start[1] == 'a' && data_start[2] == 't' &&
			data_start[3] == 'a')
		{
			check = impeghe_fread(&pstr_pcm_cfg->length, 1, 4, in_file);
			check = 0;
		}
		else
		{
			data_start[0] = data_start[1];
			data_start[1] = data_start[2];
			data_start[2] = data_start[3];
			check = impeghe_fread(&data_start[3], 1, 1, in_file);
		}
		count++;
		if (1 == wav_format_extensible)
		{
			if (count > 40)
			{
				pstr_pcm_cfg->length = 0xffffffff;
				return (1);
			}
		}
		else if (2 == wav_format_extensible)
		{
			if (count > 64)
			{
				pstr_pcm_cfg->length = 0xffffffff;
				return (1);
			}
		}
	}
	return IA_NO_ERROR;
}

WORD32 open_in_files(WORD8 *first_hoa_file_name, WORD32 *hoa_order, WORD32 *num_hoa_coeffs)
{
	// hoa_input_file_names
	CHAR8 prefix[256] = {0};
	WORD8 char_hoa_order[2];
	pCHAR8 in_file = (pCHAR8)first_hoa_file_name;
	WORD32 idx = (WORD32)(strnlen(in_file, IA_MAX_CMD_LINE_LENGTH) - 8);
	//"_00+.wav" is 8 characters
	WORD32 ret = strncmp(in_file + idx, "_00+.wav", 8);
	if (0 != ret)
	{
		printf("ERROR:For HOA, the input file name should end with _00+.wav\n");
		return -1;
	}
	if ((idx >= 0) && (idx < 256))
	{
		strncpy(prefix, in_file, idx);
		prefix[idx] = '\0';
	}
	strncpy((pCHAR8)char_hoa_order, (pCHAR8)(in_file + idx - 1), 1);
	char_hoa_order[1] = '\0';
	*hoa_order = atoi((pCHAR8)char_hoa_order);
	printf("\nHOA_ORDER : %d\n",*hoa_order);

	g_pf_hoa_input[0] = NULL;
	g_pf_hoa_input[0] = fopen((pCHAR8)first_hoa_file_name, "rb");
	if (NULL == g_pf_hoa_input[0])
	{
		printf("Error opening %s\n", first_hoa_file_name);
		return -1;
	}
	// Set file to data section
	if (impeghe_wav_header_decode(g_pf_hoa_input[0], &g_inp_hoa_config[0]))
	{
		printf("Failed to parse input file WAV header\n");
		return -1;
	}

	WORD32 in_file_cnt = 1;
	for (WORD32 order = 1; order <= (WORD32)*hoa_order; order++)
	{
		for (WORD32 m = -order; m < 0; m++)
		{
			pCHAR8 file_name = (pCHAR8)g_pb_hoa_input_file_names[in_file_cnt];
			strcpy(file_name, (pCHAR8)prefix);
			strcat(file_name, "_");
			/* max irder will fit in a byte */
			CHAR8 c[2];
			c[0] = (CHAR8)(order + '0');
			c[1] = '\0';
			strncat(file_name, c, 2);
			c[0] = (CHAR8)(-m + '0');
			strncat(file_name, c, 2);
			strcat(file_name, "-.wav");
			printf("\nfile_name - %s\n",file_name);

			g_pf_hoa_input[in_file_cnt] = fopen(file_name, "rb");
			if (NULL == g_pf_hoa_input[in_file_cnt])
			{
				printf("Error opening %s, order is %d, idx is %d\n", file_name, *hoa_order, order);
				return -1;
			}

			// Parse WAV header
			if (impeghe_wav_header_decode(g_pf_hoa_input[in_file_cnt], &g_inp_hoa_config[in_file_cnt]))
			{
				printf("Failed to parse input file WAV header\n");
				return -1;
			}
			else
			{
				if (g_inp_hoa_config[in_file_cnt].n_channels != g_inp_hoa_config[0].n_channels)
				{
				printf("Number of channels in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].sample_rate != g_inp_hoa_config[0].sample_rate)
				{
				printf("Sampling rate in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].pcm_sz != g_inp_hoa_config[0].pcm_sz)
				{
				printf("Bits per sample in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].length != g_inp_hoa_config[0].length)
				{
				printf("Sizes of input file does not match for %s\n", file_name);
				return -1;
				}
			}
			in_file_cnt++;
		}
		for (WORD32 m = 0; m <= order; m++)
		{
			pCHAR8 file_name = (pCHAR8)g_pb_hoa_input_file_names[in_file_cnt];
			strcpy(file_name, prefix);
			strcat(file_name, "_");
			/* max order will fit in a byte */
			CHAR8 c[2];
			c[0] = (WORD8)(order + '0');
			c[1] = '\0';
			strncat(file_name, c, 2);
			c[0] = (WORD8)(m + '0');
			strncat(file_name, c, 2);
			strcat(file_name, "+.wav");
			printf("\nfile_name - %s\n",file_name);

			g_pf_hoa_input[in_file_cnt] = fopen(file_name, "rb");
			if (NULL == g_pf_hoa_input[in_file_cnt])
			{
				printf("Error opening %s, order is %d, idx is %d\n", file_name, *hoa_order, order);
				return -1;
			}
			// Parse WAV header
			if (impeghe_wav_header_decode(g_pf_hoa_input[in_file_cnt], &g_inp_hoa_config[in_file_cnt]))
			{
				printf("Failed to parse input file WAV header\n");
				return -1;
			}
			else
			{
				if (g_inp_hoa_config[in_file_cnt].n_channels != g_inp_hoa_config[0].n_channels)
				{
				printf("Number of channels in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].sample_rate != g_inp_hoa_config[0].sample_rate)
				{
				printf("Sampling rate in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].pcm_sz != g_inp_hoa_config[0].pcm_sz)
				{
				printf("Bits per sample in input file does not match for %s\n", file_name);
				return -1;
				}

				if (g_inp_hoa_config[in_file_cnt].length != g_inp_hoa_config[0].length)
				{
				printf("Sizes of input file does not match for %s\n", file_name);
				return -1;
				}
			}
			in_file_cnt++;
		}
	}
	g_num_hoa_coeffs = in_file_cnt--;
	*num_hoa_coeffs = g_num_hoa_coeffs;

	return 0;
}

void ia_mpegh_handle_oam(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx)
{
	const char temp1 = '\\';
	const char temp2 = '/';
	ctx->num_channels_to_encode = 0;

	for (WORD32 i = (WORD32)strlen((char *)ctx->params.oam_file) - 1; i >= 0; i--)
	{
		if ((!strncmp((const char *)&ctx->params.oam_file[i], (const char *)&temp1, 1)) ||
			(!strncmp((const char *)&ctx->params.oam_file[i], (const char *)&temp2, 1)))
		{
			memcpy(pb_oam_file_path, ctx->params.oam_file, i + 1);
			memcpy(pb_oam_file_name, ctx->params.oam_file + i + 1, strlen((char *)ctx->params.oam_file) - i - 1);
			break;
		}
	}

	ctx->g_oam_inp = fopen(ctx->params.oam_file,"rb");

	if(ctx->g_oam_inp != NULL)
	{
		pstr_input_config->oam_read_data = impeghe_read_oam_data;
		pstr_input_config->oam_skip_data = impeghe_skip_oam_data;
		pstr_input_config->oam_data_hndl = (VOID *)ctx->g_oam_inp;

		pstr_input_config->oam_high_rate = 1;
		pstr_input_config->oam_replace_radius = 0;
		memcpy(pstr_input_config->item_prefix,pb_oam_file_name,64);
		for(int idx = 0;idx < 6; idx++)
		{
			pstr_input_config->oam_fixed_values[idx] = 0;
		}
		pstr_input_config->oam_has_core_length = 0;
		pstr_input_config->oam_has_scrn_rel_objs = 0;

		for(int idx = 0;idx < OAM_MAX_NUM_OBJECTS; idx++)
		{
			pstr_input_config->oam_is_scrn_rel_obj[idx] = 0;
		}
		pstr_input_config->use_oam_element = 1;

		// To read OAM Header Data
		// Do Error handling as well
		impeghe_read_oam_header(ctx->g_oam_inp, pstr_input_config, &ctx->num_channels_to_encode);
		for(int i=0;i<ctx->num_channels_to_encode;++i)
		{
			if (impeghe_wav_header_decode(g_pf_inps[i], &pstr_input_config->aud_obj_pcm_cfg) == 1)
			{
				fprintf(stdout, "Unable to Read Input WAV File\n");
				return;
			}
		}
		pstr_input_config->num_oam_ch = ctx->num_channels_to_encode;
		pstr_input_config->aud_obj_pcm_cfg.length *= ctx->num_channels_to_encode;
		pstr_input_config->aud_ch_pcm_cfg.n_channels = 0;
		pstr_input_config->aud_ch_pcm_cfg.length = pstr_input_config->aud_obj_pcm_cfg.length;
		pstr_input_config->aud_ch_pcm_cfg.sample_rate = pstr_input_config->aud_obj_pcm_cfg.sample_rate;
	}
}

void ia_mpegh_handle_hoa(ia_input_config *pstr_input_config,IA_MPEGH_Context *ctx)
{
	const char temp1 = '\\';
	const char temp2 = '/';

	for (WORD32 i = (WORD32)strlen((char *)ctx->params.hoa_file) - 1; i >= 0; i--)
	{
		if ((!strncmp((const char *)&ctx->params.hoa_file[i], (const char *)&temp1, 1)) ||
			(!strncmp((const char *)&ctx->params.hoa_file[i], (const char *)&temp2, 1)))
		{
			memcpy(pb_hoa_file_path, ctx->params.hoa_file, i + 1);
			break;
		}
	}
	strcpy((char *)g_pb_hoa_input_file_names[0],ctx->params.hoa_file);

	g_is_hoa_input = 1;
	if(open_in_files(g_pb_hoa_input_file_names[0], &(pv_input->hoa_order),&(pv_input->num_hoa_coeffs)))
	{
		printf("Failed to open all HOA input files\n");
		return;
	}
	pstr_input_config->use_hoa_element = 1;
	// Default to use direction estimation
	if (-1 == pstr_input_config->use_vec_est)
		pstr_input_config->use_vec_est = 0;
	/* HAM data parsing */
	if (g_pf_inp_ham)
	{
		/* Parsing HOA info */
		CHAR8 tmp_buffer[50];
		if (NULL != fgets(tmp_buffer, 50, g_pf_inp_ham))
		{
			if (0 == strncmp(tmp_buffer, "NFCflag=", 8))
			{
				char c[2];
				strncpy(c, tmp_buffer + 8, 1);
				c[1] = '\0';

				pv_input->uses_nfc = (atoi(c) == 1);
			}
			else if (0 == strncmp(tmp_buffer, "NFCflag = ", 10))
			{
				char c[2];
				strncpy(c, tmp_buffer + 10, 1);
				c[1] = '\0';

				pv_input->uses_nfc = (atoi(c) == 1);
			}
		}

		if (pv_input->uses_nfc)
		{
			if (NULL != fgets(tmp_buffer, 50, g_pf_inp_ham))
			{
				if (0 == strncmp(tmp_buffer, "NFCrefDist=", 11))
				{
					pv_input->nfc_distance = (FLOAT32)atof(tmp_buffer + 11);
				}
				else if (0 == strncmp(tmp_buffer, "NFCrefDist = ", 13))
				{
					pv_input->nfc_distance = (FLOAT32)atof(tmp_buffer + 13);
				}
				else
				{
					pv_input->nfc_distance = 0;
				}
			}
		}
	}
	/* HOA renderer matrix parsing */
	if (g_pf_inp_hoa_mtx)
	{
		int k;
		/* Syntax of HOA matrix file */
		/* Number of HOA rendering Matrix */
		/* Then for each matrix:         */
		/* Renderer ID */
		/* Output CICP */
		/* Number of Rendered channels */
		/* Number of HOA coefficients */
		/* HOA matrix */
		if (fscanf(g_pf_inp_hoa_mtx, "%d", &pv_input->num_hoa_matrix))
		{
			int ret = 0;
			if (pv_input->num_hoa_matrix > HOA_MAX_MATRIX)
			{
				pv_input->num_hoa_matrix = HOA_MAX_MATRIX;
				printf("Number of HOA matrices is high. Will restrict to first %d\n", HOA_MAX_MATRIX);
			}
			for (k = 0; k < pv_input->num_hoa_matrix; k++)
			{
				ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pv_input->hoa_rend_id[k]);
				if (!ret)
					break;
				ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pv_input->hoa_cicp[k]);
				if (!ret)
					break;
				ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pv_input->hoa_matrix_out_dim[k]);
				if (!ret)
					break;
				ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pv_input->hoa_matrix_in_dim[k]);
				if (!ret)
					break;

				if ((pv_input->hoa_matrix_in_dim[k] > HOA_MAX_COEFFS) ||
						(pv_input->hoa_matrix_out_dim[k] > MAX_NUM_OF_SPEAKERS))
				{
					printf("Incorrect HOA matrix dimension. Not sending any HOA matrix\n");
					ret = 0;
					break;
				}

				for (WORD32 i = 0; i < pv_input->hoa_matrix_in_dim[k]; ++i)
				{
					for (WORD32 j = 0; j < pv_input->hoa_matrix_out_dim[k]; ++j)
					{
						ret = fscanf(
								g_pf_inp_hoa_mtx, " %lf",
								&(pv_input->hoa_matrix[k]
																	[i * pv_input->hoa_matrix_out_dim[k] + j]));
						if (!ret)
							break;
					}
				}
				if (!ret)
					break;
			}
			if ((pv_input->num_hoa_matrix > 0) && (ret))
			{
				pv_input->use_hoa_matrix = 1;
			}
			else
			{
				printf("Error in HOA matrix file. Not sending HOA matrix file\n");
			}
		}
	}
	if (pv_input->use_hoa_element)
	{
		if (impeghe_wav_header_decode(g_pf_hoa_input[0], &pv_input->hoa_pcm_cfg) ==
			1)
		{
		fprintf(stdout, "Unable to Read Input WAV File\n");
		return;
		}
	}
	pstr_input_config->aud_ch_pcm_cfg.n_channels = 0;
	pstr_input_config->aud_ch_pcm_cfg.length = g_inp_hoa_config[0].length;
	pstr_input_config->aud_ch_pcm_cfg.sample_rate = g_inp_hoa_config[0].sample_rate;
}
//
static av_cold ia_mpegh_encode_init(AVCodecContext *avctx)
{
	IA_MPEGH_Context *ctx = avctx->priv_data;

	// Error Code
	IA_ERRORCODE err_code = IA_NO_ERROR;

	pv_input = (ia_input_config *)malloc(sizeof(ia_input_config));
	pv_output = (ia_output_config *)malloc(sizeof(ia_output_config));

	//
	ia_mpegh_encode_set_default_param(pv_input);
	//
	ia_mpegh_encode_set_config_param(pv_input,ctx);

	//
	pv_output->malloc_count = 0;
	pv_output->malloc_xaac = &malloc_global;
	//
	err_code = impeghe_create((pVOID)pv_input, (pVOID)pv_output);
	p_ia_mpeghe_obj = pv_output->pv_ia_process_api_obj;

	ctx->pb_inp_buf = (pWORD8)pv_output->mem_info_table[IA_MEMTYPE_INPUT].mem_ptr;
	ctx->pb_out_buf = (pWORD8)pv_output->mem_info_table[IA_MEMTYPE_OUTPUT].mem_ptr;

	if (pv_input->use_hoa_element)
	{
		ctx->pb_inp_hoa_buf = (pWORD8)pv_output->mem_info_table[IA_MEMTYPE_INPUT_HOA].mem_ptr;
	}

	// Initialize the API
	// impeghe_init function call
	impeghe_init(p_ia_mpeghe_obj, (pVOID)pv_input, (pVOID)pv_output);

  if ((pv_input->use_hoa_element) && (pv_input->use_hoa_matrix))
  {
    if (pv_output->hoa_mtx_status)
    {
      printf("Not sending HOA matrix due to incorrect input/config \n");
    }
  }

	// declare frame size
	avctx->frame_size = 1024;

	// Pass handle to avctx (AVCodecContext)
	ctx->pv_ia_process_api_obj = p_ia_mpeghe_obj;

	// Initialize AudioFrameQueue
	ff_af_queue_init(avctx, &ctx->afq);

	ctx->LEN = pv_output->i_dec_len;

	// Flag = 0 if output format is MHAS
	// 1 - RAW_MHAS, 2 - MP4_MHA1 and 3 - MP4_MHM1
	if(ctx->params.output_format == 1)
	{
		ctx->temp_buf = (pWORD8)malloc(pv_output->i_dec_len);
		memcpy(ctx->temp_buf, ctx->pb_out_buf, pv_output->i_dec_len);
	}
	else if(ctx->params.output_format == 2)
	{
		flag = 0;
		avctx->extradata_size = pv_output->i_dec_len;
		avctx->extradata = av_mallocz( avctx->extradata_size +  IA_MHAC_CONFIG_VERSION + IA_MHAC_PROFILE_LEVEL_INDICATION + IA_MHAC_REFRENCE_CHANNEL_LAYOUT + IA_MHAC_LENGTH + 4 + IA_OUTPUT_FMT);
		ia_reconfigure_mhaC_tag_data(ctx->pb_out_buf,pv_output->profile_info, avctx);
		(avctx->extradata)[avctx->extradata_size -1] = temp_op_fmt;

	}
	else
	{
		ctx->temp_buf = (pWORD8)malloc(pv_output->i_dec_len);
		memcpy(ctx->temp_buf, ctx->pb_out_buf, pv_output->i_dec_len);
		avctx->extradata_size = pv_output->i_dec_len;
		avctx->extradata = av_mallocz( avctx->extradata_size +  IA_MHAC_CONFIG_VERSION + IA_MHAC_PROFILE_LEVEL_INDICATION + IA_MHAC_REFRENCE_CHANNEL_LAYOUT + IA_MHAC_LENGTH + 4 + IA_OUTPUT_FMT);
		ia_reconfigure_mhaC_tag_data(ctx->pb_out_buf,pv_output->profile_info, avctx);
		(avctx->extradata)[avctx->extradata_size -1] = temp_op_fmt;
	}

	return err_code;
}

static av_cold ia_mpegh_encode_close(AVCodecContext *avctx)
{
	IA_MPEGH_Context *ctx = avctx->priv_data;

	// Calling delete of MPEGH Enc
	impeghe_delete(pv_output);

	// For OAM
	if (pv_input->use_oam_element == 1)
	{
		for (int i = 0; i < ctx->num_channels_to_encode; i++)
		{
			if (g_pf_inps[i])
			{
				fclose(g_pf_inps[i]);
				g_pf_inps[i] = NULL;
			}
		}
	}

	if (g_pf_inp_ham)
	{
		fclose(g_pf_inp_ham);
		g_pf_inp_ham = NULL;
	}
	if (g_pf_inp_hoa_mtx)
	{
		fclose(g_pf_inp_hoa_mtx);
		g_pf_inp_hoa_mtx = NULL;
	}
	for (int i = 0; i < g_num_hoa_coeffs; i++)
	{
		if (g_pf_hoa_input[i])
		{
			fclose(g_pf_hoa_input[i]);
			g_pf_hoa_input[i] = NULL;
		}
	}

	// Free all allocated memory
	free(pv_input);
	free(pv_output);
	free(ctx->temp_buf);

	// Close AudioFrameQueue.
	ff_af_queue_close(&ctx->afq);
	return 0;
}

static int ia_mpegh_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
	const AVFrame *frame, int *got_packet_ptr)
{
	// Process each frame
	IA_MPEGH_Context *ctx = avctx->priv_data;
	int ret;
	WORD32 i_out_bytes = 0;
	if (!frame)
		return 0;

	if ((ret = ff_af_queue_add(&ctx->afq, frame)) < 0)
		return ret;
	// allocating output pkt
	if ((ret = ff_alloc_packet(avctx, avpkt, 8096)) < 0)
		return ret;

	if (flag)
	{
		memcpy(avpkt->data, ctx->temp_buf, ctx->LEN);
	}

	// If OAM is present
	if(pv_input->use_oam_element == 1)
	{
		WORD32 idx = 0;
		WORD32 i_bytes_read = 0;
		for (WORD32 i = 0; i < 1024; i++)
		{
			for (WORD32 j = 0; j < ctx->num_channels_to_encode; j++)
			{
				i_bytes_read += impeghe_fread((pVOID)&ctx->pb_inp_buf[idx], sizeof(WORD8),
											(pv_input->aud_obj_pcm_cfg.pcm_sz >> 3), g_pf_inps[j]);
				idx += (pv_input->aud_obj_pcm_cfg.pcm_sz >> 3);
			}
		}
	}
	// If HOA is Present
	else if (g_is_hoa_input)
	{
		WORD32 bytes_read;
		WORD32 hoa_bytes_read = 0;
		WORD32 input_hoa_size = avctx->frame_size * g_inp_hoa_config[0].pcm_sz >> 3;
		for (WORD32 i = 0; i < g_num_hoa_coeffs; i++)
		{
			bytes_read = impeghe_fread(((UWORD8 *)ctx->pb_inp_hoa_buf) + hoa_bytes_read,
									sizeof(WORD8), input_hoa_size, g_pf_hoa_input[i]);
			hoa_bytes_read += input_hoa_size;
		}
	}
	// If no OAM or no HOA are present
	else
	{
		// copy one frame from frame->data[0] to ctx->pb_inp_buf
		memcpy(ctx->pb_inp_buf, frame->data[0], avctx->frame_size * (ctx->params.n_channels) * (ctx->params.pcm_sz/8));
	}

	if (((frame->nb_samples)*(ctx->params.n_channels)*(ctx->params.pcm_sz/8) != avctx->frame_size*(ctx->params.n_channels) * (ctx->params.pcm_sz/8)) && (!g_is_hoa_input) && (pv_input->use_oam_element != 1))
	{
		memset((ctx->pb_inp_buf + ((frame->nb_samples)*(ctx->params.n_channels) * (ctx->params.pcm_sz/8))), 0, (avctx->frame_size*(ctx->params.n_channels) * (ctx->params.pcm_sz/8) - (frame->nb_samples)*(ctx->params.n_channels) * (ctx->params.pcm_sz/8)));
	}

	// Encode Frame
	impeghe_execute(p_ia_mpeghe_obj, pv_input, pv_output);

	// Get the output bytes
	i_out_bytes = pv_output->i_out_bytes;

	// Copy i_out_bytes to avpkt->data
	if (flag)
	{
		memcpy(avpkt->data+ctx->LEN, ctx->pb_out_buf, i_out_bytes);
	}
	else
	{
		memcpy(avpkt->data, ctx->pb_out_buf, i_out_bytes);
	}
	// Get the next frame pts and duration
	ff_af_queue_remove(&ctx->afq, avctx->frame_size, &avpkt->pts,
		&avpkt->duration);
	if(flag)
	{
		avpkt->size = i_out_bytes + ctx->LEN;
		flag = 0;
	}
	else
	{
		avpkt->size = i_out_bytes;
	}

	*got_packet_ptr = 1;

	return 0;
}

FFCodec ff_ia_mpegh_encoder = {
	.p.name = "ia_mpeghe",
	.p.long_name = NULL_IF_CONFIG_SMALL("Ittiam MPEGH Encoder"),
	.p.type = AVMEDIA_TYPE_AUDIO,
	.p.id	= AV_CODEC_ID_MPEGH_3D_AUDIO,
	.priv_data_size = sizeof(IA_MPEGH_Context),
	.init = ia_mpegh_encode_init,
	FF_CODEC_ENCODE_CB(ia_mpegh_encode_frame),
	.close = ia_mpegh_encode_close,
	.p.sample_fmts = (const enum AVSampleFormat[]) {
 AV_SAMPLE_FMT_S16,AV_SAMPLE_FMT_S32,
AV_SAMPLE_FMT_NONE
},
.p.capabilities = AV_CODEC_CAP_SMALL_LAST_FRAME | AV_CODEC_CAP_DELAY,
.p.priv_class = &ia_mpegh_enc_class,
};