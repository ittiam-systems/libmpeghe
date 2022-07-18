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
#include "impegh_mp4_demux_utils.h"
#include "impegh_error_standards.h"
#include "impegh_error_handler.h"
#include "impegh_mp4_parser.h"
#include "impegh_mp4_file_wrapper.h"
#include "impegh_mp4_odf.h"
#include "impegh_mp4_atoms.h"
#include "impegh_mp4_proto.h"
#include "impegh_mp4_init.h"
#include "impegh_demux_error.h"
#include "impegh_demux_define.h"

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/

WORD32 raw_testing = 0;
FILE *g_pf_inp;
ia_file_wrapper *g_pf_inp_str;
FILE *g_pf_out;
extern ia_error_info_struct impeghd_ia_testbench_error_info;
extern ia_error_info_struct impeghd_error_info;
VOID impegh_error_handler_init();
VOID ia_testbench_error_handler_init();

/**impegh_print_usage
 *
 *  \brief Prints the usage of command line arguments
 *
 *  \return VOID
 *
 */

VOID impegh_print_usage()
{
  printf("\nUsage:\n");
  printf("\n<executable> -ifile:<inputfile> -ofile:<outputfile> [options]\n");
  printf("\n[options] can be,");

  printf("\n  <inputfile> is the input MP4 file name");
  printf("\n  <outputfile> is the output MHAS file name");
  exit(1);
}

/**impegh_mp4_demultiplex
 *
 *  \brief Demultiplex function
 *
 *
 *  \return IA_ERRORCODE
 */
IA_ERRORCODE impegh_mp4_demultiplex()
{

  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 offset = 0;
  WORD32 size_cfg_header;
  UWORD32 number_entries;
  WORD32 *length_store;
  UWORD8 *buff;
  UWORD8 charbuf[10];
  UWORD32 *data_size;
  WORD32 fread_size = 0;
  WORD32 stsz_size;
  mp4_info *m_mp4;
  it_avi_file_ctxt *itf;
  ia_bit_buf_struct_d ia_asc_bit_buf;
  m_mp4 = (mp4_info *)impegh_mp4_malloc_wrapper(sizeof(mp4_info));
  m_mp4->imp_trak_info[0] = NULL;
  m_mp4->imp_trak_info[1] = NULL;
  m_mp4->fp = (pVOID)g_pf_inp_str->file_cntxt;
  m_mp4->st_maei_info.init = 1;
  m_mp4->st_maei_info.maei_bytes_written = 0;
  impegh_mp4_init_wrap(m_mp4);

  number_entries = m_mp4->imp_trak_info[1]->stsz_count;
  length_store = malloc(number_entries * sizeof(WORD32));
  UWORD32 bytes = m_mp4->st_maei_info.maei_bytes_written;
  data_size = (UWORD32 *)&charbuf[0];
  itf = (it_avi_file_ctxt *)g_pf_inp_str->file_cntxt;
  impegh_mp4_fseek(itf, 0, SEEK_SET);

  error = impegh_mp4_find_stsz(itf, &offset, &stsz_size);
  if (error)
  {
    return error;
  }
  /* Seeking to a position after version, sample_size and sample_count */
  fseek(itf->fp, 12, SEEK_CUR);
  for (UWORD32 j = 0; j < number_entries; j++)
  {
    fread_size = (WORD32)fread(&charbuf, 4, 1, itf->fp);
    (*data_size) = (*data_size) * fread_size;
    length_store[j] = impegh_mp4_rev32(*data_size);
  }
  buff = malloc(sizeof(UWORD8) * MAX_HEADER_LENGTH); /* char array to store sync + cfg packets */
  if (buff == NULL)
  {
    return IMPEGH_DEMUX_FATAL_INVALID_MEMORY;
  }
  impegh_create_bit_buffer(&ia_asc_bit_buf, buff, MAX_HEADER_LENGTH, 1);
  impegh_mhas_write_cfg_header(&ia_asc_bit_buf, NULL,
                               (m_mp4->imp_trak_info[1]->dec_info_length << 3));

  for (UWORD32 i = 0; i < m_mp4->imp_trak_info[1]->dec_info_length; i++)
  {
    impegh_write_bits_buf(&ia_asc_bit_buf, (m_mp4->imp_trak_info[1]->dec_info[i]),
                          8); /* write sync, cfg frame */
  }
  size_cfg_header = (WORD32)(ia_asc_bit_buf.ptr_write_next - ia_asc_bit_buf.ptr_bit_buf_base);
  if (size_cfg_header > MAX_HEADER_LENGTH)
  {
    return IMPEGH_DEMUX_INVALID_HEADER_LENGTH;
  }

  if (m_mp4->imp_trak_info[1]->stsd_atom->ptr_sample_entry->sample.type == MHA1_TYPE)
  {
    WORD32 offset = 0;
    ia_bit_buf_struct_d ia_asc_bit_buf_frame;
    UWORD8 *buff_frame = malloc(sizeof(UWORD8) * MAX_HEADER_LENGTH);
    if (buff_frame == NULL)
    {
      return IMPEGH_DEMUX_FATAL_INVALID_MEMORY;
    }
    WORD32 mdat_size;
    for (WORD32 j = 0; j < size_cfg_header; j++)
    {
      fwrite(&buff[j], 1, 1, g_pf_out);
    }
    if (bytes)
    {
      ia_bit_buf_struct_d ia_asc_bit_buf_maei;
      UWORD8 *buff_maei = malloc(sizeof(UWORD8) * MAX_MAEI_LENGTH);
      if (buff_maei == NULL)
      {
        return IMPEGH_DEMUX_FATAL_INVALID_MEMORY;
      }
      impegh_create_bit_buffer(&ia_asc_bit_buf_maei, buff_maei, MAX_MAEI_LENGTH, 1);
      impegh_mhas_write_maei_header(&ia_asc_bit_buf_maei, NULL, (bytes << 3));
      for (UWORD32 i = 0; i < bytes; i++)
      {
        impegh_write_bits_buf(&ia_asc_bit_buf_maei, (m_mp4->st_maei_info.maei_write_buf[i]),
                              8); /* write sync, cfg frame */
      }

      WORD32 size_maei_header =
          (WORD32)(ia_asc_bit_buf_maei.ptr_write_next - ia_asc_bit_buf_maei.ptr_bit_buf_base);
      if (size_maei_header > MAX_MAEI_LENGTH)
      {
        return IMPEGH_DEMUX_INVALID_HEADER_LENGTH;
      }
      for (WORD32 j = 0; j < size_maei_header; j++)
      {
        fwrite(&buff_maei[j], 1, 1, g_pf_out);
      }
      free(buff_maei);
    }

    impegh_mp4_fseek(itf, 0, SEEK_SET);
    offset = 0;
    error = impegh_mp4_find_mdat(itf, &offset, &mdat_size);
    if (error)
    {
      return error;
    }

    for (UWORD32 i = 0; i < number_entries; i++)
    {
      WORD32 size_frm_header;
      impegh_create_bit_buffer(&ia_asc_bit_buf_frame, buff_frame, MAX_HEADER_LENGTH, 1);
      impegh_mhas_write_frame_header(&ia_asc_bit_buf_frame, (length_store[i] << 3));
      size_frm_header =
          (WORD32)(ia_asc_bit_buf_frame.ptr_write_next - ia_asc_bit_buf_frame.ptr_bit_buf_base);
      for (WORD32 j = 0; j < size_frm_header; j++)
      {
        fwrite(&buff_frame[j], 1, 1, g_pf_out);
      }

      for (WORD32 j = 0; j < length_store[i]; j++)
      {
        UWORD8 cTemp_mdat;
        fread_size = (WORD32)fread(&cTemp_mdat, 1, 1, itf->fp);
        fwrite(&cTemp_mdat, 1, fread_size, g_pf_out);
      }
    }
    free(buff_frame);
  }
  else if (m_mp4->imp_trak_info[1]->stsd_atom->ptr_sample_entry->sample.type == MHM1_TYPE)
  {
    WORD32 offset = 0;
    UWORD8 cTemp;
    WORD32 mdat_size;
    impegh_mp4_fseek(itf, 0, SEEK_SET);
    error = impegh_mp4_find_mdat(itf, &offset, &mdat_size);
    if (error)
    {
      return error;
    }
    while (fread(&cTemp, 1, 1, itf->fp) == 1)
    {
      fwrite(&cTemp, 1, 1, g_pf_out);
    }
  }
  if (m_mp4)
  {
    impegh_mp4_parser_close(m_mp4);
  }
  free(length_store);
  free(buff);
  return IA_NO_ERROR;
}

/**
 *  main
 *
 *  \brief Main function
 *
 *  \param [in] argc Argument count
 *  \param [in] argv Command line arguments
 *
 *  \return WORD32
 *
 */
WORD32 main(WORD32 argc, CHAR8 *argv[])
{
  FILE *param_file_id = NULL;

  WORD8 curr_cmd[IA_MAX_CMD_LINE_LENGTH];
  WORD32 fargc, curpos;
  WORD32 processcmd = 0;

  WORD8 fargv[IA_MAX_ARGS][IA_MAX_CMD_LINE_LENGTH];

  WORD8 pb_input_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
  WORD8 pb_input_file_name_copy[IA_MAX_CMD_LINE_LENGTH] = "";
  WORD8 pb_output_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
  ia_testbench_error_handler_init();
  impegh_error_handler_init();
  if (argc < 3)
  {
    if ((argc == 2) && (!strncmp((const char *)argv[1], "-paramfile:", 11)))
    {
      pWORD8 paramfile = (pWORD8)argv[1] + 11;

      param_file_id = fopen((const char *)paramfile, "r");
      if (param_file_id == NULL)
      {
        impegh_print_usage();
        return IA_NO_ERROR;
      }
    }
    else
    {
      param_file_id = fopen(PARAMFILE, "r");
      if (param_file_id == NULL)
      {
        impegh_print_usage();
        return IA_NO_ERROR;
      }
    }
    /* Process one line at a time */
    while (fgets((CHAR8 *)curr_cmd, IA_MAX_CMD_LINE_LENGTH, param_file_id))
    {
      curpos = 0;
      fargc = 0;
      /* if it is not a param_file command and if */
      /* CLP processing is not enabled */
      if (curr_cmd[0] != '@' && !processcmd)
      { /* skip it */
        continue;
      }

      while (sscanf((CHAR8 *)curr_cmd + curpos, "%s", fargv[fargc]) != EOF)
      {
        if (fargv[0][0] == '/' && fargv[0][1] == '/')
          break;
        if (strcmp((pCHAR8)fargv[0], "@echo") == 0)
          break;
        if (strcmp((const CHAR8 *)fargv[fargc], "@New_line") == 0)
        {
          if (fgets((CHAR8 *)curr_cmd + curpos, IA_MAX_CMD_LINE_LENGTH, param_file_id) == NULL)
          {
            impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Paramfile new line ",
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
          strcpy((CHAR8 *)pb_output_file_path, (const CHAR8 *)fargv[1]);
        else
          strcpy((CHAR8 *)pb_output_file_path, "");
        continue;
      }

      if (strcmp((pCHAR8)fargv[0], "@Input_path") == 0)
      {
        if (fargc > 1)
          strcpy((CHAR8 *)pb_input_file_path, (const CHAR8 *)fargv[1]);
        else
          strcpy((CHAR8 *)pb_input_file_path, "");
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

          if (!strncmp((pCHAR8)fargv[i], "-ifile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;
            WORD8 pb_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";

            strcat((CHAR8 *)pb_input_file_name, (const CHAR8 *)pb_input_file_path);
            strcat((CHAR8 *)pb_input_file_name, (const CHAR8 *)pb_arg_val);

            strcpy((CHAR8 *)pb_input_file_name_copy, (const CHAR8 *)pb_input_file_name);

            g_pf_inp = NULL;
            g_pf_inp = fopen((const CHAR8 *)pb_input_file_name, "rb");

            if (g_pf_inp == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input File",
                                   err_code);
            }
            file_count++;
          }

          if (!strncmp((pCHAR8)fargv[i], "-ofile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;

            strcat((CHAR8 *)pb_output_file_name, (const CHAR8 *)pb_output_file_path);
            strcat((CHAR8 *)pb_output_file_name, (const CHAR8 *)pb_arg_val);

            g_pf_out = NULL;
            g_pf_out = fopen((const CHAR8 *)pb_output_file_name, "wb+");
            if (g_pf_out == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Output File",
                                   err_code);
            }
            file_count++;
          }
        }

        printf("\n");

        if (file_count != 2)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input or Output File",
                               err_code);
        }
        if (err_code == IA_NO_ERROR)
        {
          g_pf_inp_str = impegh_mp4_fw_open((pWORD8)pb_input_file_name_copy);
          if (g_pf_inp_str == NULL)
          {
            err_code = IMPEGH_DEMUX_MP4_INIT_FAIL;
            impegh_error_handler(&impeghd_error_info, (pWORD8) "", err_code);
          }
        }
        if (err_code == IA_NO_ERROR)
        {
          err_code = impegh_mp4_demultiplex();
          impegh_error_handler(&impeghd_error_info, (pWORD8) "", err_code);
        }

        if (g_pf_inp)
        {
          fclose(g_pf_inp);
          g_pf_inp = NULL;
        }
        if (g_pf_inp_str)
        {
          impegh_mp4_fw_close(g_pf_inp_str);
        }

        if (g_pf_out)
        {
          fclose(g_pf_out);
          g_pf_out = NULL;
        }
      }
      printf("Decoding process complete\n");
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

      if (!strncmp((const CHAR8 *)argv[i], "-ifile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;
        WORD8 pb_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";

        strcat((CHAR8 *)pb_input_file_name, (const CHAR8 *)pb_input_file_path);
        strcat((CHAR8 *)pb_input_file_name, (const CHAR8 *)pb_arg_val);

        strcpy((CHAR8 *)pb_input_file_name_copy, (const CHAR8 *)pb_input_file_name);

        g_pf_inp = NULL;
        g_pf_inp = fopen((const CHAR8 *)pb_input_file_name, "rb");
        if (g_pf_inp == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input File", err_code);
        }
        file_count++;
      }

      if (!strncmp((const CHAR8 *)argv[i], "-ofile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;

        strcat((CHAR8 *)pb_output_file_name, (const CHAR8 *)pb_output_file_path);
        strcat((CHAR8 *)pb_output_file_name, (const CHAR8 *)pb_arg_val);

        g_pf_out = NULL;
        g_pf_out = fopen((const CHAR8 *)pb_output_file_name, "wb+");
        if (g_pf_out == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Output File",
                               err_code);
        }
        file_count++;
      }

      if (!strncmp((const CHAR8 *)argv[i], "-help", 5))
      {
        impegh_print_usage();
      }
    }

    printf("\n");
    if (file_count != 2)
    {
      err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
      impegh_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input or Output File",
                           err_code);
    }

    if (err_code == IA_NO_ERROR)
    {
      g_pf_inp_str = impegh_mp4_fw_open((pWORD8)pb_input_file_name_copy);
      if (g_pf_inp_str == NULL)
      {
        err_code = IMPEGH_DEMUX_MP4_INIT_FAIL;
        impegh_error_handler(&impeghd_error_info, (pWORD8) "", err_code);
      }
    }
    if (err_code == IA_NO_ERROR)
    {
      err_code = impegh_mp4_demultiplex();
      impegh_error_handler(&impeghd_error_info, (pWORD8) "", err_code);
    }

    if (g_pf_inp)
    {
      fclose(g_pf_inp);
      g_pf_inp = NULL;
    }
    if (g_pf_inp_str)
    {
      impegh_mp4_fw_close(g_pf_inp_str);
    }

    if (g_pf_out)
    {
      fclose(g_pf_out);
      g_pf_out = NULL;
    }
    printf("Decoding process complete\n");
  }
  if (param_file_id)
  {
    fclose(param_file_id);
  }
  return IA_NO_ERROR;
} /* end ia_param_file_process */
