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

/*****************************************************************************/
/* File includes                                                             */
/*****************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

WORD32 impeghe_read_asi(ia_asi_config *pstr_asi_config, FILE *file);
VOID impeghe_error_handler_init();
VOID impeghe_testbench_error_handler_init();

extern ia_error_info_struct ia_testbench_error_info;
extern ia_error_info_struct ia_mpeghe_error_info;

/*****************************************************************************/
/* Constant hash defines                                                     */
/*****************************************************************************/
#define MAX_STACK_PROC 10
#define MAX_MEM_ALLOCS 100
#define IA_MAX_CMD_LINE_LENGTH 300
#define IA_MAX_ARGS 20
#define IA_SCREEN_WIDTH 80
#define MAX_HOA_IN_FILES 50
#define MAX_VECTOR_SIZE 10
#define PARAMFILE "paramfilesimple.txt"
#define DRC_CONFIG_FILE "impeghe_drc_config_params.txt"

/*****************************************************************************/
/* Error codes for the testbench                                             */
/*****************************************************************************/
#define IA_TESTBENCH_MFMAN_FATAL_MEM_ALLOC_FAILED 0xFFFF8000
#define IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED 0xFFFF8001
#define IA_HEAAC_ENC_TABLE_RELOCATABLE_ENABLE 0

/*****************************************************************************/
/* Output formats                                                            */
/*****************************************************************************/
typedef enum impeghe_op_fmts
{
  RAW_MHAS = 1,
  MP4_MHA1,
  MP4_MHM1
} impeghe_op_fmts;

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/

FILE *g_pf_inps[56], *g_pf_inp, *g_pf_out, *g_pf_meta, *g_pf_spk, *g_asi;
FILE *g_pf_ec; // earcon inputfile
WORD8 ec_present = 0;
WORD32 array_ec[1024] = {0};
WORD8 pb_oam_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
WORD8 pb_oam_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
WORD8 pb_drc_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
impeghe_op_fmts op_fmt = RAW_MHAS;

pVOID g_ops_buf[32768];

FILE *g_oam_inp = 0;
FILE *g_drc_inp = NULL;
FILE *g_dmx_inp = NULL;
WORD8 g_pb_hoa_input_file_names[MAX_HOA_IN_FILES][IA_MAX_CMD_LINE_LENGTH];
FILE *g_pf_inp_ham = NULL;
FILE *g_pf_inp_hoa_mtx = NULL;
FILE *g_pf_hoa_input[50];
FLAG g_is_hoa_input = 0;
WORD32 g_num_hoa_coeffs = 0;
ia_pcm_config g_inp_hoa_config[MAX_HOA_IN_FILES];

/**
 *  impeghe_fread
 *
 *  \brief Brief description
 *
 *  \param buf
 *  \param size
 *  \param bytes
 *  \param fp
 *
 *  \return WORD32
 *
 */
WORD32 impeghe_fread(VOID *buf, WORD32 size, WORD32 bytes, FILE *fp)
{
  return (WORD32)fread(buf, size, bytes, fp);
}

/**
 *  impeghe_fwrite
 *
 *  \brief Brief description
 *
 *  \param pb_buf
 *  \param g_pf_out
 *  \param i_out_bytes
 *
 *  \return WORD32
 *
 */
WORD32 impeghe_fwrite(VOID *pb_buf, FILE *g_pf_out, WORD32 i_out_bytes)
{
  fwrite(pb_buf, sizeof(WORD8), i_out_bytes, g_pf_out);
  return 1;
}

/**
 *  impeghe_wav_header_decode
 *
 *  \brief Brief description
 *
 *  \param in_file
 *  \param ptr_enc_api
 *
 *  \return IA_ERRORCODE
 *
 */
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

/**
 *  impeghe_display_id_message
 *
 *  \brief Brief description
 *
 *  \param lib_name
 *  \param lib_version
 *
 *  \return VOID
 *
 */
VOID impeghe_display_id_message(WORD8 lib_name[], WORD8 lib_version[])
{
  WORD8 str[4][IA_SCREEN_WIDTH] = {"ITTIAM SYSTEMS PVT LTD, BANGALORE\n",
                                   "http:\\\\www.ittiam.com\n", "", ""};
  WORD8 spaces[IA_SCREEN_WIDTH / 2 + 1];
  WORD32 i, spclen;

  strcpy((pCHAR8)str[2], (pCHAR8)lib_name);
  strcat((pCHAR8)str[2], (pCHAR8)lib_version);
  strcat((pCHAR8)str[2], "\n");
  strcat((pCHAR8)str[4 - 1], "\n");

  for (i = 0; i < IA_SCREEN_WIDTH / 2 + 1; i++)
  {
    spaces[i] = ' ';
  }

  for (i = 0; i < 4; i++)
  {
    spclen = IA_SCREEN_WIDTH / 2 - (WORD32)strlen((pCHAR8)str[i]) / 2;
    spaces[spclen] = '\0';
    printf("%s", (pCHAR8)spaces);
    spaces[spclen] = ' ';
    printf("%s", (pCHAR8)str[i]);
  }
}
/**
 *  impeghe_print_usage
 *
 *  \brief Brief description
 *
 *  \return VOID
 *
 */
VOID impeghe_print_usage()
{
  ia_output_config str_output_config = {0};

  /* Get library id and version number */
  impeghe_get_lib_id_strings(&str_output_config);

  impeghe_display_id_message(str_output_config.p_lib_name, str_output_config.p_version_num);

  printf("\nUsage:\n");
  printf("\n<executable> -ifile:<inputfile> -ofile:<outputfile> [options]\n");
  printf("\n[options] can be,");
  printf("\n[-br:<bitrate>]");
  printf("\n[-op_fmt:<output_format>]");
  printf("\n[-cicp:<cicp_layout_index>]");
  printf("\n[-oam_file:<oam_file>]");
  printf("\n[-hoa_file:<first_hoa_file_name>]");
  printf("\n\nwhere,");
  printf("\n<inputfile> is the input wav file name");
  printf("\n<outputfile> is the output MP4/MHAS file name");
  printf("\n<bitrate> is the bit-rate in bits per second"
         "\n \tRange: 32000 to (6 * Sample rate * Number of channels)"
         "\n \tDefault value is"
         "\n \t32000 for mono,"
         "\n \t64000 for stereo,"
         "\n \t192000 for 6-channel,"
         "\n \t256000 for 8-channel,"
         "\n \t320000 for 10-channel");
  printf("\n<output_format> is the output format. (1 - MHAS, 2 - MHA1, 3 - MHM1). Default is "
         "1 (MHAS)");
  printf("\n<cicp_layout_index> is the channel configuration index. Range: 1 to 20 except 8 "
         "\n Description in format Front/Surr.LFE"
         "\n \t1: 1/0.0    - C"
         "\n \t2: 2/0.0    - L, R"
         "\n \t3: 3/0.0    - C, L, R"
         "\n \t4: 3/1.0    - C, L, R, Cs"
         "\n \t5: 3/2.0    - C, L, R, Ls, Rs"
         "\n \t6: 3/2.1    - C, L, R, Ls, Rs, LFE"
         "\n \t7: 5/2.1    - C, Lc, Rc, L, R, Ls, Rs, LFE"
         "\n \t8: NA"
         "\n \t9: 2/1.0    - L, R, Cs"
         "\n \t10: 2/2.0   - L, R, Ls, Rs"
         "\n \t11: 3/3.1   - C, L, R, Ls, Rs, Cs, LFE"
         "\n \t12: 3/4.1   - C, L, R, Ls, Rs, Lsr, Rsr, LFE"
         "\n \t13: 11/11.2 - C, Lc, Rc, L, R, Lss, Rss, Lsr, Rsr, Cs, LFE, LFE2, Cv, Lv, Rv, "
         "Lvss, Rvss, Ts, Lvr, Rvr, Cvr, Cb, Lb, Rb"
         "\n \t14: 5/2.1   - C, L, R, Ls, Rs, LFE, Lv, Rv"
         "\n \t15: 5/5.2   - C, L, R, Lss, Rss, Ls, Rs, Lv, Rv, Cvr, LFE, LFE2"
         "\n \t16: 5/4.1   - C, L, R, Ls, Rs, LFE, Lv, Rv, Lvs, Rvs"
         "\n \t17: 6/5.1   - C, L, R, Ls, Rs, LFE, Lv, Rv, Cv, Lvs, Rvs, Ts"
         "\n \t18: 6/7.1   - C, L, R, Ls, Rs, Lbs, Rbs, LFE, Lv, Rv, Cv, Lvs, Rvs, Ts"
         "\n \t19: 5/6.1   - C, L, R, Lss, Rss, Lsr, Rsr, LFE, Lv, Rv, Lvr, Rvr"
         "\n \t20: 7/6.1   - C, Leos, Reos, L, R, Lss, Rss, Lsr, Rsr, LFE, Lv, Rv, Lvs, Rvs"
         "\n \tIf the parameter is not set or set to 0, default value is assigned based on "
         "number of input channels.");
  printf("\n<oam_file> file containing object metadata.");
  printf("\n<first_hoa_file_name> first hoa file name that ends with 00+.wav\n");

  exit(1);
}

/**
 *  impeghe_skip_oam_data
 *
 *  \brief Brief description
 *
 *  \param hndl
 *  \param bytes_to_skip
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_skip_oam_data(VOID *hndl, WORD32 bytes_to_skip)
{
  if (!hndl)
  {
    return -1;
  }

  fseek((FILE *)hndl, bytes_to_skip, SEEK_CUR);

  return 0;
}

/**
 *  impeghe_read_oam_data
 *
 *  \brief Brief description
 *
 *  \param hndl
 *  \param buff
 *  \param bytes_to_read
 *
 *  \return WORD32
 *
 */
static WORD32 impeghe_read_oam_data(VOID *hndl, UWORD8 *buff, WORD32 bytes_to_read)
{
  if (!hndl || !buff || 0 >= bytes_to_read)
  {
    return 0;
  }

  return impeghe_fread(buff, 1, bytes_to_read, (FILE *)hndl);
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

/**
 *  impeghe_set_default_config_param
 *
 *  \brief Brief description
 *
 *  \param pstr_input_config
 *
 *  \return VOID
 *
 */
static VOID impeghe_set_default_config_param(ia_input_config *pstr_input_config)
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
  return;
}

/**
 *  impeghe_parse_config_param
 *
 *  \brief Brief description
 *
 *  \param argc
 *  \param argv
 *  \param ptr_enc_api
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghe_parse_config_param(WORD32 argc, pWORD8 argv[], pVOID ptr_enc_api)
{

  LOOPIDX i;

  ia_mpeghe_config_struct *pstr_enc_api = (ia_mpeghe_config_struct *)ptr_enc_api;
  pstr_enc_api->input_config.use_vec_est = -1;
  // Setting default MCT Mode value for input config as 0 is a valid value
  pstr_enc_api->input_config.mct_mode = -1;
  for (i = 0; i < argc; i++)
  {
    if (!strncmp((pCHAR8)argv[i], "-help", 5))
    {
      impeghe_print_usage();
    }
    /* Stream bit rate */
    if (!strncmp((pCHAR8)argv[i], "-br:", 4))
    {
      pCHAR8 pb_arg_val = (pCHAR8)argv[i] + 4;
      pstr_enc_api->input_config.bitrate = atoi(pb_arg_val);
    }
    /*op fmt*/
    if (!strncmp((pCHAR8)argv[i], "-op_fmt:", 8))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
      pstr_enc_api->input_config.out_fmt = atoi(pb_arg_val);
      if (pstr_enc_api->input_config.out_fmt == RAW_MHAS ||
          pstr_enc_api->input_config.out_fmt == MP4_MHM1)
      {
        pstr_enc_api->input_config.mhas_pkt = 1;
      }
      else
      {
        pstr_enc_api->input_config.mhas_pkt = 0;
      }
    }

    /* OAM file */
    if (!strncmp((pCHAR8)argv[i], "-oam_file:", 10))
    {
      LOOPIDX idx;

      if (g_oam_inp != 0)
      {
        pstr_enc_api->input_config.oam_read_data = impeghe_read_oam_data;
        pstr_enc_api->input_config.oam_skip_data = impeghe_skip_oam_data;
        pstr_enc_api->input_config.oam_data_hndl = (VOID *)g_oam_inp;

        pstr_enc_api->input_config.oam_high_rate = 1;
        pstr_enc_api->input_config.oam_replace_radius = 0;
        memcpy(pstr_enc_api->input_config.item_prefix, pb_oam_file_name, 64);
        for (idx = 0; idx < 6; idx++)
        {
          pstr_enc_api->input_config.oam_fixed_values[idx] = 0;
        }
        pstr_enc_api->input_config.oam_has_core_length = 0;
        pstr_enc_api->input_config.oam_has_scrn_rel_objs = 0;

        for (idx = 0; idx < OAM_MAX_NUM_OBJECTS; idx++)
        {
          pstr_enc_api->input_config.oam_is_scrn_rel_obj[idx] = 0;
        }

        pstr_enc_api->input_config.use_oam_element = 1;
      }
    }
    /* CICP layout index */
    if (!strncmp((pCHAR8)argv[i], "-cicp:", 6))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 6);
      pstr_enc_api->input_config.cicp_index = atoi(pb_arg_val);
    }
  }
  return IA_NO_ERROR;
}

static WORD32 impeghe_read_spk_pos(ia_input_config *handle, FILE *file)
{
  ia_input_config *rh_handle = (ia_input_config *)handle;
  WORD32 num_positions = 0;
  WORD8 ch, is_lfe, lfe_pos, sign_ele, sign_azi;
  WORD32 elevation, azimuth;
  WORD8 line[512];
  WORD32 ii;
  while ((ch = fgetc(file)) != EOF)
  {
    if (ch == '\n')
      num_positions++;
  }
  fseek(file, 0, SEEK_SET);

  rh_handle->str_flexi_spk_config.num_speaker = num_positions;
  for (ii = 0; ii < num_positions; ii++)
  {
    memset(line, 0, 512);
    if (fgets((pCHAR8)line, 512, file) == NULL)
    {
      printf("speaker file read failed\n");
      return -1;
    }

    if (strncmp((pCHAR8)line, "LFE_", 4) != 0)
    {
      is_lfe = 0;
      lfe_pos = 6;
    }
    else
    {
      is_lfe = 1;
      lfe_pos = 10;
    }

    WORD32 j = 0;
    sign_ele = line[lfe_pos + 7 + j];
    j++;
    sign_azi = line[lfe_pos + j];
    elevation = ((int)line[lfe_pos + 7 + j] - '0') * 10;

    j++;
    elevation += ((int)line[lfe_pos + 7 + j] - '0');
    azimuth = ((int)line[lfe_pos + j] - '0') * 100;

    j++;
    azimuth += ((int)line[lfe_pos + j] - '0') * 10;
    j++;
    azimuth += ((int)line[lfe_pos + j] - '0');
    if (elevation)
    {
      if (!strncmp((pCHAR8)&sign_ele, "-", 1))
        elevation = -elevation;
    }
    if (azimuth != 0 && azimuth != 180)
    {
      if (!(strncmp((pCHAR8)&sign_azi, "-", 1)))
        azimuth = -azimuth;
    }

    rh_handle->str_flexi_spk_config.flex_spk_azi[ii] = azimuth;
    rh_handle->str_flexi_spk_config.flex_spk_ele[ii] = elevation;
    rh_handle->str_flexi_spk_config.flex_spk_islfe[ii] = is_lfe;
  }
  return 0;
}

/**
 *  malloc_global
 *
 *  \brief Brief description
 *
 *  \param size
 *  \param alignment
 *
 *  \return pVOID
 *
 */
pVOID malloc_global(UWORD32 size, UWORD32 alignment) { return malloc(size + alignment); }

/**
 *  impeghe_read_oam_header
 *
 *  \brief Brief description
 *
 *  \param oam_file
 *  \param ptr_in_cfg
 *  \param num_channels_to_encode
 *
 *  \return WORD32
 *
 */
static IA_ERRORCODE impeghe_read_oam_header(FILE *oam_file, ia_input_config *ptr_in_cfg,
                                            WORD32 *num_channels_to_encode)
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

  bytes_read =
      impeghe_fread(temp_buff, 1, OAM_HEADER_SIZE_BYTES + OAM_VERSION_SIZE_BYTES, oam_file);
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
  ptr_in_cfg->oam_version = *ptr_16_word_temp_buff;
  if (ptr_in_cfg->oam_version > 4)
  {
    /* Invalid OAM header */
    return IMPEGHE_CONFIG_FATAL_OAM_INVALID_HEADER;
  }

  if (ptr_in_cfg->oam_version > 2)
  {
    bytes_read = impeghe_fread(&ptr_in_cfg->has_dyn_obj_priority, 1, 2, oam_file);
    if (bytes_read != 2)
    {
      /* Invalid OAM header */
      return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
    }

    if (ptr_in_cfg->has_dyn_obj_priority)
    {
      ptr_in_cfg->has_dyn_obj_priority = 1;
    }
  }

  if (ptr_in_cfg->oam_version > 3)
  {
    bytes_read = impeghe_fread(&ptr_in_cfg->has_uniform_spread, 1, 2, oam_file);
    if (bytes_read != 2)
    {
      /* Invalid OAM header */
      return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
    }

    if (ptr_in_cfg->has_uniform_spread)
    {
      ptr_in_cfg->has_uniform_spread = 1;
    }
  }
  else
  {
    ptr_in_cfg->has_uniform_spread = 1;
  }

  /* OAM Header: num_channels and num_objects */
  bytes_read = impeghe_fread(temp_buff, 1, 4, oam_file);
  if (bytes_read != 4)
  {
    /* Invalid OAM header */
    return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
  }

  ptr_16_temp_buff = ((UWORD16 *)temp_buff);
  ptr_in_cfg->num_channels = *ptr_16_temp_buff;
  ptr_16_temp_buff = ((UWORD16 *)(temp_buff + 2));
  ptr_in_cfg->num_objects = *ptr_16_temp_buff;

  if (ptr_in_cfg->num_objects > 24)
  {
    ptr_in_cfg->extra_objects = ptr_in_cfg->num_objects - 24;
    ptr_in_cfg->num_objects = 24;
    ptr_in_cfg->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
  }

  if ((ptr_in_cfg->num_objects + ptr_in_cfg->num_channels) > 24)
  {
    ptr_in_cfg->extra_objects = ptr_in_cfg->num_objects + ptr_in_cfg->num_channels - 24;
    ptr_in_cfg->num_objects = 24 - ptr_in_cfg->num_channels;
    ptr_in_cfg->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
  }

  /* OAM Header: description */
  bytes_read = impeghe_fread((pWORD8 *)temp_buff, 1, OAM_DESCRIPTION_SIZE_BYTES, oam_file);
  if (bytes_read != OAM_DESCRIPTION_SIZE_BYTES)
  {
    /* Invalid OAM header */
    return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
  }

  /* OAM Header: channel filenames */
  for (idx = 0; idx < ptr_in_cfg->num_channels; idx++)
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
    memcpy(item_name_buf, ptr_in_cfg->item_prefix, 6);
    strcat((char *)item_name_buf, (const char *)temp_buff);
    strncat((char *)oam_file_path, (char *)item_name_buf,
            strnlen((const char *)item_name_buf, 64));
    g_pf_inps[idx] = fopen((const char *)oam_file_path, "rb");
    if (NULL == g_pf_inps[idx])
    {
      printf("channel input file open failed\n");
      return -1;
    }
  }

  /* OAM Header: object describtions */
  for (idx = 0; idx < ptr_in_cfg->num_objects; idx++)
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
    memcpy(item_name_buf, ptr_in_cfg->item_prefix,
           strlen((const char *)ptr_in_cfg->item_prefix) - 4);
    strncat((char *)item_name_buf, "_", 1);
    strncat((char *)item_name_buf, object_idx[idx], 7);
    strncat((char *)oam_file_path, (char *)item_name_buf,
            strnlen((const char *)item_name_buf, 64));
    g_pf_inps[ptr_in_cfg->num_channels + idx] = fopen((const char *)oam_file_path, "rb");
    if (NULL == g_pf_inps[ptr_in_cfg->num_channels + idx])
    {
      fseek(oam_file, -OAM_OBJ_DESCRIPTION_SIZE_BYTES, SEEK_CUR);
      ptr_in_cfg->extra_objects += ptr_in_cfg->num_objects - idx;
      ptr_in_cfg->num_objects = idx > 24 ? 24 : idx;
      ptr_in_cfg->err_code = IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED;
    }
  }

  for (idx = 0; idx < ptr_in_cfg->extra_objects; idx++)
  {
    bytes_read = impeghe_fread(temp_buff, 1, OAM_OBJ_DESCRIPTION_SIZE_BYTES, oam_file);
    if (bytes_read != OAM_CH_FILE_NAME_SIZE_BYTES)
    {
      /* Invalid OAM header */
      return IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED;
    }
  }

  *num_channels_to_encode = ptr_in_cfg->num_channels + ptr_in_cfg->num_objects;

  return 0;
}

/**
 *  impehge_copy_config_params
 *
 *  \brief Copy config paramters values.
 *
 *  \param[in]    pstr_input_config
 *  \param[out]   pstr_output_config
 *
 *  \return VOID
 *
 */
static VOID impehge_copy_config_params(ia_input_config *pstr_input_config,
                                       ia_input_config *pstr_output_config)
{
  pstr_output_config->bitrate = pstr_input_config->bitrate;
  pstr_output_config->tns_enable = pstr_input_config->tns_enable;
  pstr_output_config->codec_mode = pstr_input_config->codec_mode;
  pstr_output_config->noise_filling = pstr_input_config->noise_filling;
  pstr_output_config->fill_elem = pstr_input_config->fill_elem;
  pstr_output_config->prof_level = pstr_input_config->prof_level;
  pstr_output_config->cplx_pred = pstr_input_config->cplx_pred;
  pstr_output_config->out_fmt = pstr_input_config->out_fmt;
  pstr_output_config->cicp_index = pstr_input_config->cicp_index;
  pstr_output_config->oam_high_rate = pstr_input_config->oam_high_rate;
  pstr_output_config->use_vec_est = pstr_input_config->use_vec_est;
  pstr_output_config->enhanced_noise_filling = pstr_input_config->enhanced_noise_filling;
  pstr_output_config->igf_after_tns_synth = pstr_input_config->igf_after_tns_synth;
  pstr_output_config->igf_start_freq = pstr_input_config->igf_start_freq;
  pstr_output_config->igf_stop_freq = pstr_input_config->igf_stop_freq;
  pstr_output_config->igf_start_freq_flag = pstr_input_config->igf_start_freq_flag;
  pstr_output_config->igf_stop_freq_flag = pstr_input_config->igf_stop_freq_flag;
  pstr_output_config->ltpf_enable = pstr_input_config->ltpf_enable;
  pstr_output_config->fdp_enable = pstr_input_config->fdp_enable;
  pstr_output_config->kernel = pstr_input_config->kernel;
  pstr_output_config->full_band_lpd = pstr_input_config->full_band_lpd;
  pstr_output_config->stereo_lpd = pstr_input_config->stereo_lpd;
  pstr_output_config->mct_mode = pstr_input_config->mct_mode;
  pstr_output_config->str_ec_info_struct.ec_start_frame =
      pstr_input_config->str_ec_info_struct.ec_start_frame;
  pstr_output_config->str_ec_info_struct.ec_frame_cnt =
      pstr_input_config->str_ec_info_struct.ec_frame_cnt;
  pstr_output_config->str_ec_info_struct.ec_count =
      pstr_input_config->str_ec_info_struct.ec_count;
  pstr_output_config->crc16 = pstr_input_config->crc16;
  pstr_output_config->crc32 = pstr_input_config->crc32;
  pstr_output_config->global_crc16 = pstr_input_config->global_crc16;
  pstr_output_config->global_crc32 = pstr_input_config->global_crc32;
}

/**
 *  impehge_print_config_params
 *
 *  \brief Print config paramters values after init and also warnings if they are set to Default
 *
 *  \param[in]  pstr_input_config
 *  \param[in]  pstr_input_config_prev
 *
 *  \return VOID
 *
 */
static VOID impehge_print_config_params(ia_input_config *pstr_input_config,
                                        ia_input_config *pstr_input_config_prev)
{
  printf("\n*************************************************************************************"
         "***********\n");
  printf("\nParameters Taken:\n");
  if (pstr_input_config_prev->bitrate == pstr_input_config->bitrate)
  {
    printf("\nBitrate : %d bps", pstr_input_config->bitrate);
  }
  else
  {
    printf("\nBitrate (Invalid config value, setting to default) : %d bps",
           pstr_input_config->bitrate);
  }
  if (pstr_input_config_prev->tns_enable == pstr_input_config->tns_enable)
  {
    printf("\nTNS Flag : %d", pstr_input_config->tns_enable);
  }
  else
  {
    printf("\nTNS Flag (Invalid config value, setting to default)  : %d",
           pstr_input_config->tns_enable);
  }
  printf("\nCodec Mode");
  if (pstr_input_config_prev->codec_mode != pstr_input_config->codec_mode)
  {
    if (pstr_input_config->use_hoa_element &&
        ((pstr_input_config_prev->codec_mode == USAC_ONLY_TD) ||
         (pstr_input_config_prev->codec_mode == USAC_SWITCHED)))
    {
      printf(" (Setting to FD Mode, HOA is always in FD MODE)");
    }
    else
    {
      printf(" (Invalid config value, setting to default)");
    }
  }
  if (pstr_input_config->codec_mode == USAC_SWITCHED)
  {
    printf(" : Switched Mode");
  }
  else if (pstr_input_config->codec_mode == USAC_ONLY_FD)
  {
    printf(" : FD Mode");
  }
  else if (pstr_input_config->codec_mode == USAC_ONLY_TD)
  {
    printf(" : TD Mode");
  }
  if (pstr_input_config_prev->noise_filling == pstr_input_config->noise_filling)
  {
    printf("\nNoise Filling Flag : %d", pstr_input_config->noise_filling);
  }
  else
  {
    printf("\nNoise Filling Flag (Invalid config value, setting to default) : %d",
           pstr_input_config->noise_filling);
  }
  if (pstr_input_config_prev->fill_elem == pstr_input_config->fill_elem)
  {
    printf("\nFill Elements Flag : %d", pstr_input_config->fill_elem);
  }
  else
  {
    printf("\nFill Elements Flag (Invalid config value, setting to default) : %d",
           pstr_input_config->fill_elem);
  }
  printf("\nProfile Level");

  if ((pstr_input_config_prev->prof_level != pstr_input_config->prof_level) &&
      pstr_input_config->prof_level_flag)
  {
    printf(" (Invalid config value, setting to minimal default)");
  }
  if (pstr_input_config->prof_level == PROFILE_LC_LVL1)
  {
    printf(" : 0 - Low-complexity profile Level 1");
  }
  else if (pstr_input_config->prof_level == PROFILE_LC_LVL2)
  {
    printf(" : 1 - Low-complexity profile Level 2");
  }
  else if (pstr_input_config->prof_level == PROFILE_LC_LVL3)
  {
    printf(" : 2 - Low-complexity profile Level 3");
  }
  else if (pstr_input_config->prof_level == PROFILE_LC_LVL4)
  {
    printf(" : 3 - Low-complexity profile Level 4");
  }
  else if (pstr_input_config->prof_level == PROFILE_BL_LVL1)
  {
    printf(" : 4 - Baseline profile Level 1");
  }
  else if (pstr_input_config->prof_level == PROFILE_BL_LVL2)
  {
    printf(" : 5 - Baseline profile Level 2");
  }
  else if (pstr_input_config->prof_level == PROFILE_BL_LVL3)
  {
    printf(" : 6 - Baseline profile Level 3");
  }

  if (pstr_input_config_prev->cplx_pred == pstr_input_config->cplx_pred)
  {
    printf("\nComplex Prediction Flag : %d", pstr_input_config->cplx_pred);
  }
  else
  {
    printf("\nComplex Prediction Flag (Invalid config value, setting to default) : %d",
           pstr_input_config->cplx_pred);
  }
  printf("\nOutput Format");
  if (pstr_input_config_prev->out_fmt != pstr_input_config->out_fmt)
  {
    printf(" (Invalid config value, setting to default)");
  }
  if (pstr_input_config->out_fmt == 1)
  {
    printf(" : 1 - MHAS");
  }
  else if (pstr_input_config->out_fmt == 2)
  {
    printf(" : 2 - MHA1");
  }
  else if (pstr_input_config->out_fmt == 3)
  {
    printf(" : 3 - MHM1");
  }

  if (pstr_input_config_prev->cicp_index == pstr_input_config->cicp_index)
  {
    printf("\nCICP Layout Index : %d", pstr_input_config->cicp_index);
  }
  else
  {
    printf("\nCICP Layout Index (Invalid config value, setting to default) : %d",
           pstr_input_config->cicp_index);
  }
  if (pstr_input_config->use_oam_element)
  {
    printf("\nOAM : Enabled");
    printf("\nOAM High Rate");

    if (pstr_input_config_prev->oam_high_rate != pstr_input_config->oam_high_rate)
    {
      printf(" (Invalid config value, setting to default)");
    }
    if (pstr_input_config->oam_high_rate == 0)
    {
      printf(" : Low - 0");
    }
    else if (pstr_input_config->oam_high_rate == 1)
    {
      printf(" : High - 1");
    }
  }
  else
  {
    printf("\nOAM : Disabled");
  }

  if (pstr_input_config->use_hoa_element)
  {
    printf("\nHOA : Enabled");
    printf("\nUse Vector Estimation");

    if (pstr_input_config_prev->use_vec_est != pstr_input_config->use_vec_est)
    {
      printf(" (Invalid config value, setting to default)");
    }
    if (pstr_input_config->use_vec_est == 0)
    {
      printf(" : Direction Estimation - 0");
    }
    else if (pstr_input_config->use_vec_est == 1)
    {
      printf(" : Vector Estimation - 1");
    }
    if (pstr_input_config->use_hoa_matrix)
    {
      printf("\nSending custom HOA render matrix");
    }
  }
  else
  {
    printf("\nHOA : Disabled");
  }

  if (pstr_input_config->use_drc_element)
  {
    printf("\nDRC : Enabled");
    printf("\nDRC Effects : ");
    printf("Night, Noisy, Limited, Low level, Dialog, General, Expand, Artistic");
  }
  else
  {
    printf("\nDRC : Disabled");
  }
  if (pstr_input_config->use_downmix_ext_config)
  {
    printf("\nDownmix : Enabled");
  }
  else
  {
    printf("\nDownmix : Disabled");
  }

  printf("\nEnhanced Noise Filling Flag");
  if (pstr_input_config_prev->enhanced_noise_filling != pstr_input_config->enhanced_noise_filling)
  {
    if ((pstr_input_config_prev->enhanced_noise_filling != 1) &&
        (pstr_input_config_prev->enhanced_noise_filling != 0))
    {
      if (pstr_input_config->enhanced_noise_filling == 1)
      {
        printf(" (Invalid config value, setting to default)");
      }
      else
      {
        if (pstr_input_config->codec_mode != USAC_ONLY_FD &&
            pstr_input_config->full_band_lpd == 0)
        {
          printf(" (Invalid config value, disabling IGF as FLPD is disabled)");
        }
        else
        {
          printf(" (Invalid config value, disabling IGF for high bitrate)");
        }
      }
    }
    else
    {
      if (pstr_input_config->codec_mode != USAC_ONLY_FD && pstr_input_config->full_band_lpd == 0)
      {
        printf(" (Invalid config value, disabling IGF as FLPD is disabled)");
      }
      else
      {
        printf(" (Disabling IGF for high bitrate)");
      }
    }
  }
  printf(" : %d", pstr_input_config->enhanced_noise_filling);
  printf("\nAfter TNS Filling Flag");
  if (pstr_input_config_prev->igf_after_tns_synth != pstr_input_config->igf_after_tns_synth)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->igf_after_tns_synth);
  printf("\nIGF Start Fequency");
  if (pstr_input_config->igf_start_freq_flag == 1)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d Khz", pstr_input_config->igf_start_freq);

  printf("\nIGF Stop Fequency");
  if (pstr_input_config->igf_stop_freq_flag == 1)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d Khz", pstr_input_config->igf_stop_freq);

  printf("\nLTPF Flag");
  if (pstr_input_config_prev->ltpf_enable != pstr_input_config->ltpf_enable)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->ltpf_enable);

  printf("\nFDP Flag");
  if (pstr_input_config_prev->fdp_enable != pstr_input_config->fdp_enable)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->fdp_enable);

  printf("\nAliasing Symmetry Kernel");
  if (pstr_input_config_prev->kernel != pstr_input_config->kernel)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->kernel);

  printf("\nFLPD Flag");
  if (pstr_input_config_prev->full_band_lpd != pstr_input_config->full_band_lpd)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->full_band_lpd);
  printf("\nSLPD Flag");
  if (pstr_input_config_prev->stereo_lpd != pstr_input_config->stereo_lpd)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->stereo_lpd);
  printf("\nMCT Mode");
  if (pstr_input_config_prev->mct_mode != pstr_input_config->mct_mode)
  {
    printf(" (Invalid config value, setting to default)");
  }
  if (pstr_input_config->mct_mode == -1)
  {
    printf(" : Disabled");
  }
  else
  {
    printf(" : %d", pstr_input_config->mct_mode);
  }

  printf("\nCRC16 Flag");
  if (pstr_input_config_prev->crc16 != pstr_input_config->crc16)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->crc16);
  printf("\nCRC32 Flag");
  if (pstr_input_config_prev->crc32 != pstr_input_config->crc32)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->crc32);
  printf("\nGlobal CRC16 Flag");
  if (pstr_input_config_prev->global_crc16 != pstr_input_config->global_crc16)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->global_crc16);
  printf("\nGlobal CRC32 Flag");
  if (pstr_input_config_prev->global_crc32 != pstr_input_config->global_crc32)
  {
    printf(" (Invalid config value, setting to default)");
  }
  printf(" : %d", pstr_input_config->global_crc32);
  if (pstr_input_config->str_ec_info_struct.ec_present)
  {
    printf("\nEarcon : Enabled");
    printf("\nEarcon Start Frame");
    if (pstr_input_config_prev->str_ec_info_struct.ec_start_frame !=
        pstr_input_config->str_ec_info_struct.ec_start_frame)
    {
      printf(" (Invalid config value, setting to default)");
    }
    printf(" : %d", pstr_input_config->str_ec_info_struct.ec_start_frame);

    printf("\nEarcon Frame Count");
    if (pstr_input_config_prev->str_ec_info_struct.ec_frame_cnt !=
        pstr_input_config->str_ec_info_struct.ec_frame_cnt)
    {
      printf(" (Invalid config value, setting to default)");
    }
    printf(" : %d", pstr_input_config->str_ec_info_struct.ec_frame_cnt);

    printf("\nEarcon Count");
    if (pstr_input_config_prev->str_ec_info_struct.ec_count !=
        pstr_input_config->str_ec_info_struct.ec_count)
    {
      printf(" (Invalid config value, setting to default)");
    }
    printf(" : %d\n", pstr_input_config->str_ec_info_struct.ec_count);
  }
  else
  {
    printf("\nEarcon : Disabled\n");
  }
  printf("\n*************************************************************************************"
         "***********\n\n");
}

/**
 *  impeghe_main_process
 *
 *  \brief Brief description
 *
 *  \param argc
 *  \param argv
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghe_main_process(WORD32 argc, pWORD8 argv[])
{
  UWORD32 *ia_stsz_size = NULL;

  LOOPIDX frame_count = 0;

  /* Error code */
  IA_ERRORCODE err_code = IA_NO_ERROR;

  /* API obj */
  pVOID pv_ia_process_api_obj;

  /* First part                                        */
  /* Error Handler Init                                */
  /* Get Library Name, Library Version and API Version */
  /* Initialize API structure + Default config set     */
  /* Set config params from user                       */
  /* Initialize memory tables                          */
  /* Get memory information and allocate memory        */

  /* Process initing done query variable */
  pWORD8 pb_inp_buf = NULL, pb_out_buf = NULL, pb_inp_hoa_buf = NULL;
  WORD32 i_bytes_read = 0;

  WORD32 input_size = 0;
  WORD32 input_hoa_size = 0;
  WORD32 input_oam_size = 0;
  WORD32 expected_frame_count = 0;
  WORD32 start_offset_samples;
  ia_mp4_writer_struct mp4_writer_io = {0};

  /* The error init function */
  VOID (*p_error_init)();

  /* The process error info structure */
  ia_error_info_struct *p_proc_err_info;

  // ia_mpeghe_config_struct str_enc_api = {{0}};
  ia_mpeghe_config_struct *pstr_enc_api =
      (ia_mpeghe_config_struct *)malloc_global(sizeof(ia_mpeghe_config_struct), 0);
  memset(pstr_enc_api, 0, sizeof(ia_mpeghe_config_struct));
  ia_input_config *pstr_in_cfg = &pstr_enc_api->input_config;
  ia_input_config *pstr_in_cfg_prev = malloc_global(sizeof(ia_input_config), 0);
  ia_output_config *pstr_out_cfg = &pstr_enc_api->output_config;
  WORD32 num_channels_to_encode = 0, i;

  /* Stack process struct initing */
  p_error_init = impeghe_error_handler_init;
  p_proc_err_info = &ia_mpeghe_error_info;
  /* Stack process struct initing end */

  /* ******************************************************************/
  /* Initialize the error handler                                     */
  /* ******************************************************************/
  (*p_error_init)();

  impeghe_set_default_config_param(&pstr_enc_api->input_config);

  /* ******************************************************************/
  /* Get the library name, library version and API version            */
  /* ******************************************************************/

  if (g_is_hoa_input)
  {
    // hoa related file handling - open all hoa input files
    if (open_in_files(g_pb_hoa_input_file_names[0], &(pstr_enc_api->input_config.hoa_order),
                      &(pstr_enc_api->input_config.num_hoa_coeffs)))
    {
      printf("Failed to open all HOA input files\n");
      return -1;
    }
    pstr_enc_api->input_config.use_hoa_element = 1;

    // Default to use direction estimation
    if (-1 == pstr_enc_api->input_config.use_vec_est)
      pstr_enc_api->input_config.use_vec_est = 0;

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

          pstr_enc_api->input_config.uses_nfc = (atoi(c) == 1);
        }
        else if (0 == strncmp(tmp_buffer, "NFCflag = ", 10))
        {
          char c[2];
          strncpy(c, tmp_buffer + 10, 1);
          c[1] = '\0';

          pstr_enc_api->input_config.uses_nfc = (atoi(c) == 1);
        }
      }

      if (pstr_enc_api->input_config.uses_nfc)
      {
        if (NULL != fgets(tmp_buffer, 50, g_pf_inp_ham))
        {
          if (0 == strncmp(tmp_buffer, "NFCrefDist=", 11))
          {
            pstr_enc_api->input_config.nfc_distance = (FLOAT32)atof(tmp_buffer + 11);
          }
          else if (0 == strncmp(tmp_buffer, "NFCrefDist = ", 13))
          {
            pstr_enc_api->input_config.nfc_distance = (FLOAT32)atof(tmp_buffer + 13);
          }
          else
          {
            pstr_enc_api->input_config.nfc_distance = 0;
          }
        }
      }
    }

    /* HOA renderer matrix parsing */
    if (g_pf_inp_hoa_mtx)
    {
      int j, k;
      /* Syntax of HOA matrix file */
      /* Number of HOA rendering Matrix */
      /* Then for each matrix:         */
      /* Renderer ID */
      /* Output CICP */
      /* Number of Rendered channels */
      /* Number of HOA coefficients */
      /* HOA matrix */
      if (fscanf(g_pf_inp_hoa_mtx, "%d", &pstr_enc_api->input_config.num_hoa_matrix))
      {
        int ret = 0;
        if (pstr_enc_api->input_config.num_hoa_matrix > HOA_MAX_MATRIX)
        {
          pstr_enc_api->input_config.num_hoa_matrix = HOA_MAX_MATRIX;
          printf("Number of HOA matrices is high. Will restrict to first %d\n", HOA_MAX_MATRIX);
        }
        for (k = 0; k < pstr_enc_api->input_config.num_hoa_matrix; k++)
        {
          ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pstr_enc_api->input_config.hoa_rend_id[k]);
          if (!ret)
            break;
          ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pstr_enc_api->input_config.hoa_cicp[k]);
          if (!ret)
            break;
          ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pstr_enc_api->input_config.hoa_matrix_out_dim[k]);
          if (!ret)
            break;
          ret = fscanf(g_pf_inp_hoa_mtx, "%d", &pstr_enc_api->input_config.hoa_matrix_in_dim[k]);
          if (!ret)
            break;

          if ((pstr_enc_api->input_config.hoa_matrix_in_dim[k] > HOA_MAX_COEFFS) ||
              (pstr_enc_api->input_config.hoa_matrix_out_dim[k] > MAX_NUM_OF_SPEAKERS))
          {
            printf("Incorrect HOA matrix dimension. Not sending any HOA matrix\n");
            ret = 0;
            break;
          }

          for (i = 0; i < pstr_enc_api->input_config.hoa_matrix_in_dim[k]; ++i)
          {
            for (j = 0; j < pstr_enc_api->input_config.hoa_matrix_out_dim[k]; ++j)
            {
              ret = fscanf(
                  g_pf_inp_hoa_mtx, " %lf",
                  &(pstr_enc_api->input_config
                        .hoa_matrix[k]
                                   [i * pstr_enc_api->input_config.hoa_matrix_out_dim[k] + j]));
              if (!ret)
                break;
            }
          }
          if (!ret)
            break;
        }
        if ((pstr_enc_api->input_config.num_hoa_matrix > 0) && (ret))
        {
          pstr_enc_api->input_config.use_hoa_matrix = 1;
        }
        else
        {
          printf("Error in HOA matrix file. Not sending HOA matrix file\n");
        }
      }
    }
  }

  impeghe_parse_config_param(argc, argv, pstr_enc_api);
  if (pstr_enc_api->input_config.use_hoa_element)
  {
    if (impeghe_wav_header_decode(g_pf_hoa_input[0], &pstr_enc_api->input_config.hoa_pcm_cfg) ==
        1)
    {
      fprintf(stdout, "Unable to Read Input WAV File\n");
      return -1;
    }
  }

  if (g_pf_inp)
  {
    if (impeghe_wav_header_decode(g_pf_inp, &pstr_enc_api->input_config.aud_ch_pcm_cfg) == 1)
    {
      fprintf(stdout, "Unable to Read Input WAV File\n");
      return -1;
    }
  }

  if (pstr_enc_api->input_config.use_oam_element == 1)
  {
    impeghe_read_oam_header(g_oam_inp, &pstr_enc_api->input_config, &num_channels_to_encode);

    _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", pstr_enc_api->input_config.err_code);

    for (i = 0; i < num_channels_to_encode; i++)
    {
      if (impeghe_wav_header_decode(g_pf_inps[i], &pstr_enc_api->input_config.aud_obj_pcm_cfg) ==
          1)
      {
        fprintf(stdout, "Unable to Read Input WAV File\n");
        return -1;
      }
    }

    pstr_enc_api->input_config.num_oam_ch = num_channels_to_encode;
    pstr_enc_api->input_config.aud_obj_pcm_cfg.length *= num_channels_to_encode;
  }

  /*Downmix*/
  if (g_dmx_inp != 0)
  {
    /* Syntax of Downmix Config params file */
    /* Downmix Config Type (0 - Active downmix parameters, 1 - Downmix matrix) */
    /* Passive downmix flag */
    /* Phase align strength */
    /* Immersive downmix flag */
    /* Downmix ID count */
    /* Then for each ID : */
    /* Downmix ID */
    /* Downmix Type ( 0 - Default downmix matrix, 1 - Transmitted downmix matrix) */
    /* CICP speaker layout index */
    /* Downmix matrix count */
    /* Output CICP index */
    /* Input CICP index */
    /* Flat downmix matrix */

    LOOPIDX j, m;
    UWORD32 i;
    WORD32 ret = 0, incorrect_data = 0;
    ia_mpeghe_ext_cfg_dmx_mtx_input_struct *pstr_dmx_matrix;

    memset(&pstr_enc_api->input_config.str_ext_cfg_downmix_input, 0,
           sizeof(pstr_enc_api->input_config.str_ext_cfg_downmix_input));

    ia_mpeghe_ext_cfg_downmix_input_struct *ptr_downmix_input =
        &pstr_enc_api->input_config.str_ext_cfg_downmix_input;
    ret = fscanf(g_dmx_inp, "%d", &ptr_downmix_input->dmx_config_type);
    if (!ret)
      incorrect_data++;

    if (ptr_downmix_input->dmx_config_type == 0 || ptr_downmix_input->dmx_config_type == 2)
    {
      ret = fscanf(g_dmx_inp, "%d", &ptr_downmix_input->passive_dmx_flag);
      if (!ret)
        incorrect_data++;
      ret = fscanf(g_dmx_inp, "%d", &ptr_downmix_input->phase_align_strength);
      if (!ret)
        incorrect_data++;
      ret = fscanf(g_dmx_inp, "%d", &ptr_downmix_input->immersive_downmix_flag);
      if (!ret)
        incorrect_data++;
    }

    if (ptr_downmix_input->dmx_config_type == 1 || ptr_downmix_input->dmx_config_type == 2)
    {
      ret = fscanf(g_dmx_inp, "%d", &ptr_downmix_input->downmix_id_count);
      if (!ret)
        incorrect_data++;

      if (ptr_downmix_input->downmix_id_count > IMPEGHE_MAX_DMX_MATRIX_ELEMENTS)
      {
        printf("Downmix matrix elements are supported upto %d, Ignoring excess matrix elements",
               IMPEGHE_MAX_DMX_MATRIX_ELEMENTS);
        ptr_downmix_input->downmix_id_count = IMPEGHE_MAX_DMX_MATRIX_ELEMENTS;
      }
    }
    if (ptr_downmix_input->dmx_config_type == 1 || ptr_downmix_input->dmx_config_type == 2)
    {
      for (i = 0; i < ptr_downmix_input->downmix_id_count; i++)
      {
        if (!ret)
          break;
        pstr_dmx_matrix = &ptr_downmix_input->str_dmx_matrix[i];
        ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->dmx_id);
        if (!ret)
          break;
        ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->dmx_type);
        if (!ret)
          break;
        ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->cicp_spk_layout_idx);
        if (!ret)
          break;

        if (pstr_dmx_matrix->dmx_type == 1)
        {
          ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->downmix_mtx_count);
          if (!ret)
            break;

          for (j = 0; j < pstr_dmx_matrix->downmix_mtx_count; j++)
          {
            pstr_dmx_matrix->num_assigned_group_ids[j] = 2;
            for (m = 0; m < pstr_dmx_matrix->num_assigned_group_ids[j]; m++)
            {
              pstr_dmx_matrix->signal_group_id[j][m] = m;
            }
          }

          ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->str_dmx_mtx_cfg.output_config_index);
          if (!ret)
            break;
          ret = fscanf(g_dmx_inp, "%d", &pstr_dmx_matrix->str_dmx_mtx_cfg.input_config_index);
          if (!ret)
            break;

          for (j = 0;
               j < sizeof(pstr_dmx_matrix->str_dmx_mtx_cfg.flat_downmix_matrix) / sizeof(FLOAT32);
               j++)
          {
            ret =
                fscanf(g_dmx_inp, "%f", &pstr_dmx_matrix->str_dmx_mtx_cfg.flat_downmix_matrix[j]);
            if (!ret)
              break;
          }

          pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.num_eq = 1;
          for (j = 0; j < pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.num_eq; j++)
          {
            pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j].num_pk_filter = 1;
            pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j].global_gain_db = 1;

            for (m = 0;
                 m < pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j].num_pk_filter;
                 m++)
            {
              pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j]
                  .pk_filter_params[m]
                  .peak_freq_hz = 2.0f;
              pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j]
                  .pk_filter_params[m]
                  .peak_q_factor = 2.5f;
              pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_params[j]
                  .pk_filter_params[m]
                  .peak_gain_db = 1.2f;
            }

            pstr_dmx_matrix->str_dmx_mtx_cfg.pstr_eq_config.eq_map[j] = 0;
          }
        }
      }
    }
    if ((ptr_downmix_input->downmix_id_count > 0) && (ret) && (incorrect_data == 0))
    {
      pstr_enc_api->input_config.use_downmix_ext_config = 1;
    }
    else
    {
      printf("Error in Downmix config params file. Downmix will be disabled.\n");
    }
  }
  if (pstr_enc_api->input_config.use_drc_element == 1)
  {
    LOOPIDX k;
    CHAR8 drc_config_file_name[IA_MAX_CMD_LINE_LENGTH];
    strcpy(drc_config_file_name, (const char *)pb_drc_file_path);
    strcat(drc_config_file_name, DRC_CONFIG_FILE);
    g_drc_inp = fopen(drc_config_file_name, "rt");

    if (!g_drc_inp)
    {
      printf("\nError in opening DRC configuration file\n\n");
      pstr_enc_api->input_config.use_drc_element = 0;
    }
    if (g_drc_inp != 0)
    {
      memset(&pstr_enc_api->input_config.str_drc_cfg, 0, sizeof(ia_drc_input_config));
      impeghe_read_drc_config_params(
          g_drc_inp, &pstr_enc_api->input_config.str_drc_cfg.str_enc_params,
          &pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config,
          &pstr_enc_api->input_config.str_drc_cfg.str_enc_loudness_info_set,
          &pstr_enc_api->input_config.str_drc_cfg.str_enc_gain_extension,
          &pstr_enc_api->input_config.str_ext_cfg_downmix_input);

      pstr_enc_api->input_config.str_drc_cfg.str_enc_params.gain_sequence_present = FALSE;
      for (k = 0; k < pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config
                          .drc_coefficients_uni_drc_count;
           k++)
      {
        if (pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config
                .str_drc_coefficients_uni_drc[k]
                .drc_location == 1)
        {
          if (pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config
                  .str_drc_coefficients_uni_drc[k]
                  .gain_set_count > 0)
          {
            pstr_enc_api->input_config.str_drc_cfg.str_enc_params.gain_sequence_present = TRUE;
            break;
          }
        }
      }

      if (pstr_enc_api->input_config.str_drc_cfg.str_enc_params.gain_sequence_present == FALSE)
      {
        for (k = 0; k < pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config
                            .str_uni_drc_config_ext.drc_coefficients_uni_drc_v1_count;
             k++)
        {
          if (pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config.str_uni_drc_config_ext
                  .str_drc_coefficients_uni_drc_v1[k]
                  .drc_location == 1)
          {
            if (pstr_enc_api->input_config.str_drc_cfg.str_uni_drc_config.str_uni_drc_config_ext
                    .str_drc_coefficients_uni_drc_v1[k]
                    .gain_sequence_count > 0)
            {
              pstr_enc_api->input_config.str_drc_cfg.str_enc_params.gain_sequence_present = TRUE;
              break;
            }
          }
        }
      }
      pstr_enc_api->input_config.use_drc_element = 1;
    }
  }

  // n_channels indicates number of static channels.
  if (NULL == g_pf_inp)
  {
    pstr_enc_api->input_config.aud_ch_pcm_cfg.n_channels = 0;
  }
  if (ec_present)
  {
    pstr_enc_api->input_config.str_ec_info_struct.ec_present = ec_present;
    if (impeghe_wav_header_decode(
            g_pf_ec, &pstr_enc_api->input_config.str_ec_info_struct.str_pcm_config) == 1)
    {
      fprintf(stdout, "Earcon file reading failed\n");
    }
  }
  if (pstr_enc_api->input_config.flexi_spk_enable == 1)
  {
    if (impeghe_read_spk_pos(&pstr_enc_api->input_config, g_pf_spk) != 0)
    {
      fprintf(stdout, "Speaker file reading failed\n");
    }
  }
  if (pstr_enc_api->input_config.asi_enable == 1)
  {
    if (impeghe_read_asi(&pstr_enc_api->input_config.str_asi_config, g_asi) != 0)
    {
      fprintf(stdout, "asi file reading failed\n");
    }
  }
  pstr_out_cfg->malloc_xaac = &malloc_global;

  impehge_copy_config_params(pstr_in_cfg, pstr_in_cfg_prev);

  err_code = impeghe_create((pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);
  _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

  impeghe_display_id_message(pstr_out_cfg->p_lib_name, pstr_out_cfg->p_version_num);

  pv_ia_process_api_obj = pstr_out_cfg->pv_ia_process_api_obj;

  pb_inp_buf = (pWORD8)pstr_out_cfg->mem_info_table[IA_MEMTYPE_INPUT].mem_ptr;
  pb_out_buf = (pWORD8)pstr_out_cfg->mem_info_table[IA_MEMTYPE_OUTPUT].mem_ptr;

  if (pstr_in_cfg->use_hoa_element)
  {
    pb_inp_hoa_buf = (pWORD8)pstr_out_cfg->mem_info_table[IA_MEMTYPE_INPUT_HOA].mem_ptr;
  }

  /* End first part */

  /* Second part        */
  /* Initialize process */
  /* Get config params  */
  err_code = impeghe_init(pv_ia_process_api_obj, (pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);
  _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

  if ((pstr_in_cfg->use_hoa_element) && (pstr_in_cfg->use_hoa_matrix))
  {
    if (pstr_out_cfg->hoa_mtx_status)
    {
      printf("Not sending HOA matrix due to incorrect input/config \n");
    }
  }

  impehge_print_config_params(pstr_in_cfg, pstr_in_cfg_prev);

  start_offset_samples = 1600;
  input_size = pstr_out_cfg->in_frame_length * pstr_in_cfg->aud_ch_pcm_cfg.n_channels *
               (pstr_in_cfg->aud_ch_pcm_cfg.pcm_sz >> 3);
  if (input_size)
  {
    expected_frame_count = (pstr_in_cfg->aud_ch_pcm_cfg.length + (input_size - 1)) / input_size;
  }

  if (1 == pstr_enc_api->input_config.use_oam_element)
  {
    WORD32 num_oam_frames = 0;
    input_oam_size = pstr_out_cfg->in_frame_length * num_channels_to_encode *
                     (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3);

    if (0 == input_size)
    {
      input_size = input_oam_size;
      pstr_in_cfg->aud_ch_pcm_cfg.length = pstr_in_cfg->aud_obj_pcm_cfg.length;
      pstr_in_cfg->aud_ch_pcm_cfg.sample_rate = pstr_in_cfg->aud_obj_pcm_cfg.sample_rate;
    }

    // ceil
    num_oam_frames =
        (pstr_in_cfg->aud_obj_pcm_cfg.length + (input_oam_size - 1)) / input_oam_size;
    expected_frame_count =
        (num_oam_frames > expected_frame_count) ? num_oam_frames : expected_frame_count;
  }

  if (1 == pstr_enc_api->input_config.use_hoa_element)
  {
    WORD32 num_hoa_frames = 0;
    input_hoa_size = pstr_out_cfg->in_frame_length * (g_inp_hoa_config[0].pcm_sz >> 3);

    if ((0 == input_size) || (0 != input_oam_size))
    {
      input_size = input_hoa_size;
      pstr_in_cfg->aud_ch_pcm_cfg.length = g_inp_hoa_config[0].length;
      pstr_in_cfg->aud_ch_pcm_cfg.sample_rate = g_inp_hoa_config[0].sample_rate;
    }

    // ceil
    num_hoa_frames = (pstr_in_cfg->hoa_pcm_cfg.length + (input_hoa_size - 1)) / input_hoa_size;
    expected_frame_count =
        (num_hoa_frames > expected_frame_count) ? num_hoa_frames : expected_frame_count;
  }

  if (ec_present)
  {
    expected_frame_count += pstr_in_cfg->str_ec_info_struct.ec_frame_cnt;
  }
  if (0 == ia_stsz_size)
  {
    ia_stsz_size = malloc_global((expected_frame_count + 2) * sizeof(WORD32), 0);
    memset(ia_stsz_size, 0, (expected_frame_count + 2) * sizeof(WORD32));
  }

  /*Write o/p File headers if any*/
  if (op_fmt == RAW_MHAS)
  {
    impeghe_fwrite(pb_out_buf, g_pf_out, pstr_out_cfg->i_dec_len);
  }
  else
  {
    // copy header bytes
    if (pstr_out_cfg->i_dec_len > MAX_HDR_LEN)
    {
      fprintf(stderr, "header length too large\n");
      return IA_FATAL_ERROR;
    }
    memcpy(mp4_writer_io.ptr_hdr, pb_out_buf, pstr_out_cfg->i_dec_len);

    // dummy write mp4 header
    // meta data
    mp4_writer_io.meta_info.mhac_length = pstr_out_cfg->i_dec_len;
    mp4_writer_io.meta_info.g_track_count = 1;
    mp4_writer_io.meta_info.ia_mp4_stsz_entries = expected_frame_count;
    mp4_writer_io.meta_info.media_time_scale = pstr_in_cfg->aud_ch_pcm_cfg.sample_rate;
    mp4_writer_io.meta_info.movie_time_scale = pstr_in_cfg->aud_ch_pcm_cfg.sample_rate;
    mp4_writer_io.meta_info.playTimeInSamples[0] =
        pstr_in_cfg->aud_ch_pcm_cfg.length /
        ((pstr_in_cfg->aud_ch_pcm_cfg.pcm_sz >> 3) * pstr_out_cfg->i_num_chan);
    mp4_writer_io.meta_info.ia_mp4_stsz_size = ia_stsz_size;

    // init size
    mp4_writer_io.mdat_size = 0;

    // file ptr
    mp4_writer_io.fp_mp4 = g_pf_out;

    // is_mhm1
    mp4_writer_io.is_mhm1 = 0;
    if (op_fmt == MP4_MHM1)
      mp4_writer_io.is_mhm1 = 1;

    // profile/level information
    mp4_writer_io.profile_info = pstr_out_cfg->profile_info;

    // gen mp4 file
    impeghe_mp4_writer(&mp4_writer_io, 1);

    // write MHAS_PAC_TYP_MPEGH3DACFG
    if (op_fmt == MP4_MHM1)
    {
      impeghe_fwrite(mp4_writer_io.ptr_hdr, g_pf_out, pstr_out_cfg->i_dec_len);
    }
  }

  if (pstr_in_cfg->use_oam_element == 1)
  {
    WORD32 idx = 0;
    i_bytes_read = 0;
    for (WORD32 i = 0; i < 1024; i++)
    {
      for (WORD32 j = 0; j < num_channels_to_encode; j++)
      {
        i_bytes_read += impeghe_fread((pVOID)&pb_inp_buf[idx], sizeof(WORD8),
                                      (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3), g_pf_inps[j]);
        idx += (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3);
      }
    }
  }
  if (g_pf_inp)
  {
    i_bytes_read = impeghe_fread((pVOID)pb_inp_buf, sizeof(WORD8), input_size, g_pf_inp);
  }

  if (g_is_hoa_input)
  {
    for (i = 0; i < g_num_hoa_coeffs; i++)
    {
      i_bytes_read = impeghe_fread(((UWORD8 *)pb_inp_hoa_buf) + (i * input_hoa_size),
                                   sizeof(WORD8), input_hoa_size, g_pf_hoa_input[i]);

      if (i_bytes_read != input_hoa_size)
      {
        memset((((UWORD8 *)pb_inp_hoa_buf) + (i * input_hoa_size) + i_bytes_read), 0,
               (input_hoa_size - i_bytes_read));
        // ITTIAM: Zero padding for the last frame
      }
    }
  }

  UWORD8 u_is_last_frame_encoded = 0;
  if (0 == i_bytes_read)
  {
    u_is_last_frame_encoded = 1;
  }
  while ((i_bytes_read) || (!u_is_last_frame_encoded))
  {
    u_is_last_frame_encoded =
        (pstr_enc_api->input_config.use_drc_element) ? (i_bytes_read == 0) : 1;
    frame_count++;

    if ((i_bytes_read != input_size) && (!g_is_hoa_input) && (pstr_in_cfg->use_oam_element != 1)

    )
    {
      memset((pb_inp_buf + i_bytes_read), 0, (input_size - i_bytes_read));
      // ITTIAM: Zero padding for the last frame
    }

    // Reset Bytes read
    i_bytes_read = 0;

    /*****************************************************************************/
    /* Perform Encoding of frame data */
    /*****************************************************************************/

    err_code = impeghe_execute(pv_ia_process_api_obj, (pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);

    _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

    if (pstr_out_cfg->i_out_bytes)
      ia_stsz_size[frame_count - 1] = pstr_out_cfg->i_out_bytes;
    else
      frame_count--;

    impeghe_fwrite(pb_out_buf, g_pf_out, pstr_out_cfg->i_out_bytes);
    fflush(g_pf_out);

    /*print the frame count on the stdout*/
    fprintf(stderr, "Frames Processed :%d\r", frame_count);
    if (ec_present)
    {

      if ((frame_count >= pstr_in_cfg->str_ec_info_struct.ec_start_frame) &&
          (pstr_in_cfg->str_ec_info_struct.ec_frame_cnt))
      {
        pstr_in_cfg->str_ec_info_struct.ec_active = !pstr_in_cfg->str_ec_info_struct.ec_active;
      }
    }
    if (pstr_in_cfg->str_ec_info_struct.ec_active)
    {

      i_bytes_read = 0;
      WORD32 idx = 0;
      pstr_in_cfg->str_ec_info_struct.ec_frame_cnt--;
      for (WORD32 i = 0; i < 1024; i++)
      {
        i_bytes_read +=
            impeghe_fread((pVOID)&pstr_in_cfg->str_ec_info_struct.ptr_ec_buff[idx], sizeof(WORD8),
                          (pstr_in_cfg->str_ec_info_struct.str_pcm_config.pcm_sz >> 3), g_pf_ec);
        idx++;
      }

      if (i_bytes_read == 0)
      {
        pstr_in_cfg->str_ec_info_struct.ec_active = 0;
      }
    }
    if (pstr_in_cfg->use_oam_element == 1)
    {
      WORD32 idx = 0;
      WORD32 bytes_read;
      i_bytes_read = 0;
      for (WORD32 i = 0; i < 1024; i++)
      {
        for (WORD32 j = 0; j < num_channels_to_encode; j++)
        {
          bytes_read = impeghe_fread((pVOID)&pb_inp_buf[idx], sizeof(WORD8),
                                     (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3), g_pf_inps[j]);
          i_bytes_read += bytes_read;
          if (bytes_read != (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3))
          {
            memset(&pb_inp_buf[idx + bytes_read], 0, (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3));

            if (feof(g_pf_inps[j]))
            {
              // fprintf(stderr, "EOF reached for OAM input\r");
            }
          }
          idx += (pstr_in_cfg->aud_obj_pcm_cfg.pcm_sz >> 3);
        }
      }
    }
    if ((g_pf_inp) && (!pstr_in_cfg->str_ec_info_struct.ec_active))
    {
      i_bytes_read = impeghe_fread((pVOID)pb_inp_buf, sizeof(WORD8), input_size, g_pf_inp);
    }
    if (g_is_hoa_input)
    {
      WORD32 bytes_read;
      WORD32 hoa_bytes_read = 0;
      for (i = 0; i < g_num_hoa_coeffs; i++)
      {
        bytes_read = impeghe_fread(((UWORD8 *)pb_inp_hoa_buf) + hoa_bytes_read, sizeof(WORD8),
                                   input_hoa_size, g_pf_hoa_input[i]);

        if (bytes_read != input_hoa_size)
        {
          memset((((UWORD8 *)pb_inp_hoa_buf) + hoa_bytes_read + bytes_read), 0,
                 (input_hoa_size - bytes_read));
          // ITTIAM: Zero padding for the last frame
        }
        hoa_bytes_read += input_hoa_size;

        /* if i_bytes_read is 0 when reached here, then no OAM or ifile data */
        if (bytes_read)
        {
          i_bytes_read = input_hoa_size;
        }
      }
    }

  } /* End of encoder loop*/
clean_return:

  impeghe_delete((pVOID)pstr_out_cfg);

  if (pstr_in_cfg_prev)
  {
    free(pstr_in_cfg_prev);
  }

  if (g_pf_ec)
  {
    fclose(g_pf_ec);
    g_pf_ec = NULL;
  }

  if (pstr_in_cfg->use_oam_element == 1)
  {
    for (i = 0; i < num_channels_to_encode; i++)
    {
      if (g_pf_inps[i])
      {
        fclose(g_pf_inps[i]);
        g_pf_inps[i] = NULL;
      }
    }
  }

  if (op_fmt == MP4_MHA1 || op_fmt == MP4_MHM1)
  {
    // frame_count
    if (mp4_writer_io.meta_info.ia_mp4_stsz_entries != frame_count)
    {
      fprintf(stderr, "Frame count mismatch, correcting\n");
      frame_count = mp4_writer_io.meta_info.ia_mp4_stsz_entries;
    }

    // for mhm1 - mhas hdr added to 1st frame

    for (int idx = 0; idx < frame_count; idx++)
    {
      mp4_writer_io.max_frame_data_size = mp4_writer_io.meta_info.ia_mp4_stsz_size[idx] > mp4_writer_io.max_frame_data_size ?
        mp4_writer_io.meta_info.ia_mp4_stsz_size[idx] : mp4_writer_io.max_frame_data_size;
      mp4_writer_io.total_frame_data_size += mp4_writer_io.meta_info.ia_mp4_stsz_size[idx];
    }
    mp4_writer_io.frame_count = frame_count;
    mp4_writer_io.sampling_freq = pstr_in_cfg->aud_ch_pcm_cfg.sample_rate;
    if (op_fmt == MP4_MHM1)
    {
      ia_stsz_size[0] += pstr_out_cfg->i_dec_len;
    }

    mp4_writer_io.meta_info.ia_mp4_stsz_entries = frame_count;

    // startOffsetInSamples
    mp4_writer_io.meta_info.startOffsetInSamples[0] = start_offset_samples;
    // playTimeInSamples
    mp4_writer_io.meta_info.playTimeInSamples[0] =
        pstr_in_cfg->aud_ch_pcm_cfg.length /
        ((pstr_in_cfg->aud_ch_pcm_cfg.pcm_sz >> 3) * pstr_out_cfg->i_num_chan);

    // init size
    mp4_writer_io.mdat_size = 0;

    // seek to start of file
    fseek(g_pf_out, 0, SEEK_SET);

    // gen mp4 file
    impeghe_mp4_writer(&mp4_writer_io, 0);
  }

  pstr_enc_api->input_config.use_drc_element = 0;

  if (pstr_enc_api)
  {
    free(pstr_enc_api);
  }
  if (ia_stsz_size)
  {
    free(ia_stsz_size);
  }

  return IA_NO_ERROR;
}

/**
 *  main
 *
 *  \brief Brief description
 *
 *  \param argc
 *  \param argv
 *
 *  \return WORD32
 *
 */
WORD32 main(WORD32 argc, char *argv[])
{
  FILE *param_file_id = NULL;
  //  LOOPIDX i;

  WORD8 curr_cmd[IA_MAX_CMD_LINE_LENGTH];
  WORD32 fargc, curpos;
  WORD32 processcmd = 0;

  WORD8 fargv[IA_MAX_ARGS][IA_MAX_CMD_LINE_LENGTH];

  pWORD8 pargv[IA_MAX_ARGS];

  WORD8 pb_output_file_path[IA_MAX_CMD_LINE_LENGTH] = "";

  WORD8 pb_input_file_path[IA_MAX_CMD_LINE_LENGTH] = "";

  WORD8 pb_hoa_file_path[IA_MAX_CMD_LINE_LENGTH] = "";

  g_oam_inp = 0;

  impeghe_testbench_error_handler_init();

  if (argc < 3)
  {
    if ((argc == 2) && (!strncmp((const char *)argv[1], "-paramfile:", 11)))
    {
      pWORD8 paramfile = (pWORD8)argv[1] + 11;

      param_file_id = fopen((const char *)paramfile, "r");
      if (param_file_id == NULL)
      {
        impeghe_print_usage();
        return IA_NO_ERROR;
      }
    }
    else
    {
      param_file_id = fopen(PARAMFILE, "r");
      if (param_file_id == NULL)
      {
        impeghe_print_usage();
        return IA_NO_ERROR;
      }
    }

    /* Process one line at a time */
    while (fgets((char *)curr_cmd, IA_MAX_CMD_LINE_LENGTH, param_file_id))
    {
      curpos = 0;
      fargc = 0;
      /* if it is not a param_file command and if */
      /* CLP processing is not enabled */
      if (curr_cmd[0] != '@' && !processcmd)
      { /* skip it */
        continue;
      }

      while (sscanf((char *)curr_cmd + curpos, "%s", fargv[fargc]) != EOF)
      {
        if (fargv[0][0] == '/' && fargv[0][1] == '/')
          break;
        if (strcmp((pCHAR8)fargv[0], "@echo") == 0)
          break;
        if (strcmp((const char *)fargv[fargc], "@New_line") == 0)
        {
          if (fgets((char *)curr_cmd + curpos, IA_MAX_CMD_LINE_LENGTH, param_file_id) == NULL)
          {
            impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Paramfile new line ",
                                  IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
            exit(1);
          }
          continue;
        }
        curpos += (WORD32)strlen((pCHAR8)fargv[fargc]);
        while (*(curr_cmd + curpos) == ' ' || *(curr_cmd + curpos) == '\t')
          curpos++;
        fargc++;
      }

      if (fargc < 1) /* for blank lines etc. */
        continue;

      if (strcmp((pCHAR8)fargv[0], "@Output_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_output_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_output_file_path, "");
        continue;
      }

      if (strcmp((pCHAR8)fargv[0], "@Input_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_input_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_input_file_path, "");
        continue;
      }

      if (strcmp((pCHAR8)fargv[0], "@Oam_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_oam_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_oam_file_path, "");
        continue;
      }
      if (strcmp((pCHAR8)fargv[0], "@Hoa_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_hoa_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_hoa_file_path, "");
        continue;
      }
      if (strcmp((pCHAR8)fargv[0], "@Start") == 0)
      {
        processcmd = 1;
        continue;
      }

      if (strcmp((pCHAR8)fargv[0], "@Stop") == 0)
      {
        processcmd = 0;
        continue;
      }
      strcpy((char *)pb_drc_file_path, (const char *)pb_input_file_path);

      /* otherwise if this a normal command and its enabled for execution */
      if (processcmd)
      {
        WORD32 i;
        WORD32 err_code = IA_NO_ERROR;
        WORD8 pb_output_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
        WORD32 file_count = 0;
        for (i = 0; i < fargc; i++)
        {
          printf("%s ", fargv[i]);
          pargv[i] = fargv[i];

          if (!strncmp((pCHAR8)fargv[i], "-ifile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;
            WORD8 pb_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
            strcat((char *)pb_input_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_input_file_name, (const char *)pb_arg_val);

            g_pf_inp = NULL;
            g_pf_inp = fopen((const char *)pb_input_file_name, "rb");
            if (g_pf_inp == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input File", err_code);
            }
            file_count++;
          }

          if (!strncmp((pCHAR8)fargv[i], "-ofile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;

            strcat((char *)pb_output_file_name, (const char *)pb_output_file_path);
            strcat((char *)pb_output_file_name, (const char *)pb_arg_val);

            g_pf_out = NULL;
            g_pf_out = fopen((const char *)pb_output_file_name, "wb+");
            if (g_pf_out == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Output File", err_code);
            }
            file_count++;
          }

          if (!strncmp((pCHAR8)fargv[i], "-oam_file:", 10))
          {
            pWORD8 pb_arg_val = fargv[i] + 10;
            WORD8 pb_oam_file_local[IA_MAX_CMD_LINE_LENGTH] = "";

            memcpy(pb_oam_file_name, pb_arg_val, strlen((const char *)pb_arg_val));

            strcat((char *)pb_oam_file_local, (const char *)pb_oam_file_path);
            strcat((char *)pb_oam_file_local, (const char *)pb_arg_val);

            g_oam_inp = NULL;
            g_oam_inp = fopen((const char *)pb_oam_file_local, "rb");
            if (g_oam_inp == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "OAM File", err_code);
            }
            file_count++;
          }
          /* hoa related */
          if (!strncmp((pCHAR8)fargv[i], "-hoa_file:", 10))
          {
            pWORD8 pb_arg_val = fargv[i] + 10;
            strcpy((char *)g_pb_hoa_input_file_names[0], (const char *)pb_hoa_file_path);
            strcat((char *)g_pb_hoa_input_file_names[0], (const char *)pb_arg_val);

            g_is_hoa_input = 1;
            file_count++;
          }
          /*op fmt*/
          if (!strncmp((pCHAR8)fargv[i], "-op_fmt:", 8))
          {
            pCHAR8 pb_arg_val = (pCHAR8)(fargv[i] + 8);
            op_fmt = atoi(pb_arg_val);
          }
        }
        printf("\n");

        if (file_count < 2)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input or Output File",
                                err_code);
        }
        if (err_code == IA_NO_ERROR)
          impeghe_main_process(fargc, pargv);

        if (g_pf_inp)
        {
          fclose(g_pf_inp);
          g_pf_inp = NULL;
        }

        if (g_pf_meta)
        {
          fclose(g_pf_meta);
          g_pf_meta = NULL;
        }

        if (g_oam_inp)
        {
          fclose(g_oam_inp);
          g_oam_inp = NULL;
        }

        if (g_drc_inp)
        {
          fclose(g_drc_inp);
          g_drc_inp = NULL;
        }
        if (g_dmx_inp)
        {
          fclose(g_dmx_inp);
          g_dmx_inp = NULL;
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
        for (i = 0; i < g_num_hoa_coeffs; i++)
        {
          if (g_pf_hoa_input[i])
          {
            fclose(g_pf_hoa_input[i]);
            g_pf_hoa_input[i] = NULL;
          }
        }

        ec_present = 0;
        if (g_pf_ec)
        {
          fclose(g_pf_ec);
          g_pf_ec = NULL;
        }
        if (g_pf_spk)
        {
          fclose(g_pf_spk);
          g_pf_spk = NULL;
        }
        if (g_pf_out)
        {
          fclose(g_pf_out);
          g_is_hoa_input = 0;
          g_pf_out = NULL;
        }
      }
      printf("\nEncoding process complete\n");
    }
    if (param_file_id)
    {
      fclose(param_file_id);
    }
  }
  else
  {
    WORD32 i;
    WORD32 err_code = IA_NO_ERROR;
    WORD8 pb_output_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
    WORD32 file_count = 0;
    for (i = 1; i < argc; i++)
    {
      printf("%s ", argv[i]);

      if (!strncmp((const char *)argv[i], "-ifile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;
        WORD8 pb_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";

        strcat((char *)pb_input_file_name, (const char *)pb_input_file_path);
        strcat((char *)pb_input_file_name, (const char *)pb_arg_val);

        g_pf_inp = NULL;
        g_pf_inp = fopen((const char *)pb_input_file_name, "rb");
        if (g_pf_inp == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input File", err_code);
        }
        file_count++;
      }

      if (!strncmp((const char *)argv[i], "-ofile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;

        strcat((char *)pb_output_file_name, (const char *)pb_output_file_path);
        strcat((char *)pb_output_file_name, (const char *)pb_arg_val);

        g_pf_out = NULL;
        g_pf_out = fopen((const char *)pb_output_file_name, "wb+");
        if (g_pf_out == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Output File", err_code);
        }
        file_count++;
      }

      if (!strncmp((const char *)argv[i], "-oam_file:", 10))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 10;
        WORD8 pb_oam_file_local[IA_MAX_CMD_LINE_LENGTH] = "";
        const char temp1 = '\\';
        const char temp2 = '/';

        for (WORD32 i = (WORD32)strlen((char *)pb_arg_val) - 1; i >= 0; i--)
        {
          if ((!strncmp((const char *)&pb_arg_val[i], (const char *)&temp1, 1)) ||
              (!strncmp((const char *)&pb_arg_val[i], (const char *)&temp2, 1)))
          {
            memcpy(pb_oam_file_path, pb_arg_val, i + 1);
            memcpy(pb_oam_file_name, pb_arg_val + i + 1, strlen((char *)pb_arg_val) - i - 1);
            break;
          }
        }

        strcat((char *)pb_oam_file_local, (const char *)pb_oam_file_path);
        strcat((char *)pb_oam_file_local, (const char *)pb_oam_file_name);

        g_oam_inp = NULL;
        g_oam_inp = fopen((const char *)pb_oam_file_local, "rb");
        if (g_oam_inp == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "OAM File", err_code);
        }
        file_count++;
      }
      /* HOA related */
      if (!strncmp(argv[i], "-hoa_file:", 10))
      {
        pCHAR8 pb_arg_val = argv[i] + 10;
        strcpy((char *)g_pb_hoa_input_file_names[0], (const char *)pb_hoa_file_path);
        strcat((char *)g_pb_hoa_input_file_names[0], pb_arg_val);

        g_is_hoa_input = 1;
        file_count++;
      }
      /*op fmt*/
      if (!strncmp((pCHAR8)argv[i], "-op_fmt:", 8))
      {
        pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
        op_fmt = atoi(pb_arg_val);
      }

      if (!strncmp((const char *)argv[i], "-help", 5))
      {
        impeghe_print_usage();
      }
    }
    printf("\n");
    if (file_count < 2)
    {
      err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
      impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input or Output File", err_code);
    }

    if (err_code == IA_NO_ERROR)
      impeghe_main_process(argc - 1, (pWORD8 *)&argv[1]);

    if (g_pf_inp)
    {
      fclose(g_pf_inp);
      g_pf_inp = NULL;
    }
    if (g_pf_ec)
    {
      ec_present = 0;
      fclose(g_pf_ec);
      g_pf_ec = NULL;
    }
    if (g_pf_spk)
    {
      fclose(g_pf_spk);
      g_pf_spk = NULL;
    }
    if (g_pf_out)
    {
      fclose(g_pf_out);
      g_is_hoa_input = 0;
      g_pf_out = NULL;
    }

    if (g_oam_inp)
    {
      fclose(g_oam_inp);
      g_oam_inp = NULL;
    }

    if (g_drc_inp)
    {
      fclose(g_drc_inp);
      g_drc_inp = NULL;
    }
    if (g_dmx_inp)
    {
      fclose(g_dmx_inp);
      g_dmx_inp = NULL;
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
    for (i = 0; i < g_num_hoa_coeffs; i++)
    {
      if (g_pf_hoa_input[i])
      {
        fclose(g_pf_hoa_input[i]);
        g_pf_hoa_input[i] = NULL;
      }
    }

    if (g_pf_meta)
    {
      fclose(g_pf_meta);
      g_pf_meta = NULL;
    }
    printf("\nEncoding process complete\n");
  }

  return IA_NO_ERROR;
} /* end ia_param_file_process */
