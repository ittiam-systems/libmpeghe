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
#include "impegh_mp4_mux_utils.h"
#include "impeghe_error_standards.h"
#include "impeghe_mp4_writer.h"
#include "impeghe_error_handler.h"
#include "impegh_mp4_mux_define.h"

/**impegh_print_usage
 *
 *  \brief Print usage
 *
 *  \return VOID
 *
 */
VOID impegh_print_usage()
{
  printf("\nUsage:\n");
  printf("\n<executable> -ifile:<inputfile> -ofile:<outputfile> [options]\n");
  printf("\n[options] can be,");

  printf("\n[-op_fmt:<output format>]");

  printf("\n  <inputfile> is the input MHAS file name");
  printf("\n  <outputfile> is the output MP4 file name");
  printf("\n  <output_format> is the output format. (2 - MHA1, 3 - MHM1). Default is MHM1");
  exit(1);
}

/**impegh_mp4_mux
 *
 *  \brief MP4 Mux
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impegh_mp4_mux(packet_info *header_info)
{
  WORD32 start_offset_samples;
  WORD32 i_dec_len = 0;
  WORD32 skip_bytes = 0;
  UWORD8 *chtemp;
  ia_mp4_writer_struct *mp4_writer_io = (ia_mp4_writer_struct *)malloc(sizeof(*mp4_writer_io));
  memset(mp4_writer_io, 0, sizeof(ia_mp4_writer_struct));

  if (op_fmt == MP4_MHM1)
  {
    mp4_writer_io->is_mhm1 = 1;
  }

  /* File pointer for input file */
  fseek(g_pf_inp, 0, SEEK_SET);
  start_offset_samples = 1600;

  i_dec_len = header_info->config_packet_start_position + header_info->mhac_content_size; /* Length till config data ends*/

  // copy header bytes
  if (i_dec_len > MAX_HDR_LEN)
  {
    return IMPEGHE_MUX_NON_FATAL_INVALID_HEADER_LENGTH;
  }
  chtemp = malloc(sizeof(chtemp[0]) * i_dec_len);
  for (WORD32 i = 0; i < i_dec_len; i++)
  {
    chtemp[i] = fgetc(g_pf_inp);
  }

  memcpy(mp4_writer_io->ptr_hdr, chtemp + header_info->config_packet_start_position, header_info->mhac_content_size);
  mp4_writer_io->meta_info.mhac_length = header_info->mhac_content_size;
  mp4_writer_io->maei_present = header_info->maei_present;
  mp4_writer_io->mhaP_data_present = header_info->mhaP_data_present;
  mp4_writer_io->mhaD_data_present = header_info->mhaD_data_present;

  if (mp4_writer_io->mhaP_data_present)
  {
    memcpy(mp4_writer_io->ptr_hdr_mhaP, &header_info->mhaP_buff[0],
      (header_info->mhaP_bits + 7) >> 3);
    mp4_writer_io->meta_info.mhaP_length = (header_info->mhaP_bits + 7) >> 3;
  }

  if (mp4_writer_io->mhaD_data_present)
  {
    memcpy(mp4_writer_io->ptr_hdr_mhaD, &header_info->mhaD_buff[0],
      (header_info->mhaD_bits + 7) >> 3);
    mp4_writer_io->meta_info.mhaD_length = (header_info->mhaD_bits + 7) >> 3;
  }
  if (mp4_writer_io->maei_present)
  {
    memcpy(mp4_writer_io->ptr_hdr_maeg, &header_info->maeg_buff[0],
           (header_info->maeg_bits + 7) >> 3);
    memcpy(mp4_writer_io->ptr_hdr_maes, &header_info->maes_buff[0],
           (header_info->maes_bits + 7) >> 3);
    memcpy(mp4_writer_io->ptr_hdr_maep, &header_info->maep_buff[0],
           (header_info->maep_bits + 7) >> 3);
    memcpy(mp4_writer_io->ptr_hdr_mael, &header_info->mael_buff[0],
           (header_info->mael_bits + 7) >> 3);
    mp4_writer_io->meta_info.maeg_length = (header_info->maeg_bits + 7) >> 3;
    mp4_writer_io->meta_info.maes_length = (header_info->maes_bits + 7) >> 3;
    mp4_writer_io->meta_info.maep_length = (header_info->maep_bits + 7) >> 3;
    mp4_writer_io->meta_info.mael_length = (header_info->mael_bits + 7) >> 3;
  }

  // meta data
  mp4_writer_io->meta_info.g_track_count = 1;
  mp4_writer_io->meta_info.ia_mp4_stsz_entries =
      frame_count; // ia_mp4_stsz_entries--> no. of frames

  mp4_writer_io->meta_info.media_time_scale = header_info->sampling_freq;
  mp4_writer_io->meta_info.movie_time_scale = header_info->sampling_freq;
  mp4_writer_io->meta_info.playTimeInSamples[0] =
      frame_count * 1024; // Future this will have to change accd to config frame
  mp4_writer_io->meta_info.ia_mp4_stsz_size = (UWORD32 *)frame_lengths;

  // init size
  mp4_writer_io->mdat_size = 0;

  mp4_writer_io->fp_mp4 = g_pf_out; // outFile

  // profile/level information
  mp4_writer_io->profile_info = header_info->profile_info;
  mp4_writer_io->spaker_layout = header_info->spaker_layout;

  //bitrat related
  mp4_writer_io->sampling_freq = header_info->sampling_freq;
  mp4_writer_io->total_frame_data_size = header_info->total_frame_data_size;
  mp4_writer_io->max_frame_data_size = header_info->max_frame_data_size;
  mp4_writer_io->frame_count = header_info->frame_count;

  // startOffsetInSamples
  mp4_writer_io->meta_info.startOffsetInSamples[0] = start_offset_samples;
  // playTimeInSamples
  mp4_writer_io->meta_info.playTimeInSamples[0] = frame_count * 1024;

  // gen mp4 file
  impeghe_mp4_writer(mp4_writer_io, 1); // mp4 header

  if (op_fmt == MP4_MHM1)
  {
    UWORD8 cTemp;
    fseek(g_pf_inp, 0, SEEK_SET);
    // append the mhas file to g_pf_out
    while (fread(&cTemp, 1, 1, g_pf_inp) == 1)
    {
      fwrite(&cTemp, 1, 1, g_pf_out);
    }
  }
  else
  {
    WORD32 i = 0;
    WORD32 local_counter = 0;
    UWORD8 cTemp;
    fseek(g_pf_inp, 0, SEEK_SET);

    fseek(g_pf_inp,frame_header_lengths[i],
      SEEK_SET);

    // append the mhas file to g_pf_out
    while (fread(&cTemp, 1, 1, g_pf_inp) == 1)
    {
      fwrite(&cTemp, 1, 1, g_pf_out);
      local_counter++;
      if (local_counter == frame_lengths[i])
      {
        i++;
        if (i == frame_count)
          i--;
        /* seek to skip header bytes */
        fseek(g_pf_inp, frame_header_lengths[i], SEEK_CUR);

        local_counter = 0;
      }
    }
  }
  free(chtemp);
  free(mp4_writer_io);
  return IA_NO_ERROR;
}

/**impegh_mp4_multiplex
 *
 *  \brief MP4 Multiplex
 *
 *  \return IA_ERRORCODE
 */

IA_ERRORCODE impegh_mp4_multiplex()
{
  /* File pointer for input file */
  IA_ERRORCODE error = IA_NO_ERROR;
  WORD32 init = 1;
  WORD32 packet_length = 0;
  WORD32 packet_length_cpy = 0;
  WORD32 fread_size = 0;
  WORD32 inp_bytes = IN_ARR_SIZE;
  WORD32 file_size;
  jmp_buf mux_jmp_buf;
  ia_bit_buf_struct str_bit_buf;
  ia_mhas_pac_info str_pac_info;
  packet_info header_info;

  UWORD8 *arr =
      malloc(sizeof(UWORD8) *
             IN_ARR_SIZE); /* Array used to store 12500 values from input file at a time */
  fseek(g_pf_inp, 0L, SEEK_END);
  file_size = ftell(g_pf_inp);
  fseek(g_pf_inp, 0L, SEEK_SET);
  memset(&str_pac_info, 0, sizeof(str_pac_info));
  memset(&header_info, 0, sizeof(header_info));
  header_info.profile_info = 0x0D; // default value for profile_info
  error = setjmp(mux_jmp_buf);
  if (error != IA_NO_ERROR)
  {
    return IMPEGHE_MUX_NON_FATAL_INSUFFICIENT_INPUT_BYTES;
  }
  do
  {
    if (init == 1)
    {
      fread_size = (WORD32)fread(arr, 1, (IN_ARR_SIZE), g_pf_inp);
      inp_bytes = fread_size;
      impegh_create_init_bit_buf(&str_bit_buf, arr, fread_size);
      str_bit_buf.impeghe_jmp_buf = &mux_jmp_buf;
      init = 0;
    }

    error = impegh_file_parse(&str_bit_buf); /* Main Pasing Function */
    if (error)
    {
      return error;
    }
    frame_count++;

    packet_length =
        (((inp_bytes << 3) - str_bit_buf.cnt_bits) >> 3); /* no of bytes processed till now */

    if (packet_length + packet_length_cpy >=
        file_size) /* Exit Condition: If the total bytes processed value exceeds the input File
                      size break the loop */
    {
      break;
    }

    if (packet_length >= 8500) /* If the current packet length exceeds 8500 reload the arr buffer
                                  with the updated SEEK_SET pointer */
    {
      fseek(g_pf_inp, packet_length_cpy + packet_length, SEEK_SET);

      fread_size = (WORD32)fread(arr, 1, (IN_ARR_SIZE), g_pf_inp);
      inp_bytes = fread_size;
      impegh_create_init_bit_buf(&str_bit_buf, arr,
                                 fread_size); /* Load the next fread_size values */
      packet_length_cpy += packet_length;
    }
  } while (1); /* Read the entire file to get the number of frames */

  /* Initialize the memory for frame_lengths and frame_header_lengths based on number of frames */
  frame_lengths = (WORD32 *)malloc(sizeof(frame_lengths[0]) * frame_count);
  frame_header_lengths = (WORD32 *)malloc(sizeof(frame_header_lengths[0]) * frame_count);
  memset(frame_lengths, 0, sizeof(frame_lengths[0]) * frame_count);
  memset(frame_header_lengths, 0, sizeof(frame_header_lengths[0]) * frame_count);

  init = 1;
  fseek(g_pf_inp, 0L, SEEK_SET);
  frame_count = 0;
  packet_length = 0;
  packet_length_cpy = 0;

  do
  {
    if (init == 1)
    {
      fread_size = (WORD32)fread(arr, 1, (IN_ARR_SIZE), g_pf_inp);
      inp_bytes = fread_size;
      impegh_create_init_bit_buf(&str_bit_buf, arr, fread_size);
      str_bit_buf.impeghe_jmp_buf = &mux_jmp_buf;
      init = 0;
    }
    error = impegh_mhas_parse(&str_bit_buf, &str_pac_info, &header_info); /* Main Pasing Function */
    if (error)
    {
      return error;
    }

    if (op_fmt == MP4_MHM1)
    {
      frame_lengths[frame_count] += ((header_info.frame_packet_bits + 7) >> 3)
        + ((header_info.other_packet_bits + 7) >> 3)
        + ((header_info.asi_packet_bits + 7) >> 3)
        + ((header_info.sync_packet_bits + 7) >> 3)
        + ((header_info.config_packet_bits + 7) >> 3);
      header_info.sync_packet_bits = 0;
      header_info.config_packet_bits = 0;
      header_info.other_packet_bits = 0;
      header_info.asi_packet_bits = 0;

    }
    else /* MHA1 condition */
    {
      frame_lengths[frame_count] = str_pac_info.packet_length; // frame data

      //config packets
      frame_header_lengths[frame_count] += ((header_info.other_packet_bits + 7) >> 3)
        + ((header_info.asi_packet_bits + 7) >> 3)
        + ((header_info.sync_packet_bits + 7) >> 3)
        + ((header_info.config_packet_bits + 7) >> 3);
      //frame packet header
      frame_header_lengths[frame_count] += ((header_info.frame_packet_bits + 7) >> 3) - str_pac_info.packet_length;
      header_info.other_packet_bits = 0;
      header_info.sync_packet_bits = 0;
      header_info.config_packet_bits = 0;
      header_info.asi_packet_bits = 0;
    }

    header_info.max_frame_data_size = header_info.max_frame_data_size > frame_lengths[frame_count] ? header_info.max_frame_data_size : frame_lengths[frame_count];
    header_info.total_frame_data_size = header_info.total_frame_data_size + frame_lengths[frame_count];
    frame_count++;
    header_info.frame_count = frame_count;

    packet_length =
        (((inp_bytes << 3) - str_bit_buf.cnt_bits) >> 3); /* no of bytes processed till now */

    if (packet_length + packet_length_cpy >=
        file_size) /* Exit Condition: If the total bytes processed value exceeds the input File
                      size break the loop */
    {
      break;
    }

    if (packet_length >= 8500) /* If the current packet length exceeds 8500 reload the arr buffer
                                  with the updated SEEK_SET pointer */
    {
      fseek(g_pf_inp, packet_length_cpy + packet_length, SEEK_SET);

      fread_size = (WORD32)fread(arr, 1, (IN_ARR_SIZE), g_pf_inp);
      inp_bytes = fread_size;
      impegh_create_init_bit_buf(&str_bit_buf, arr,
                                 fread_size); /* Load the next fread_size values */
      packet_length_cpy += packet_length;
    }
  } while (1);


  error = impegh_mp4_mux(&header_info);
  if (error)
  {
    return error;
  }
  /* Reset to 0 for next file */
  frame_count = 0;
  if (frame_lengths)
  {
    free(frame_lengths);
  }

  if (frame_header_lengths)
  {
    free(frame_header_lengths);
  }
  if (arr)
  {
    free(arr);
  }
  return IA_NO_ERROR;
}

/**main
 *
 *  \brief Main
 *
 *  \param [in] argc
 *  \param [in] argv
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
  impeghe_error_handler_init();
  impeghe_testbench_error_handler_init();
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
              impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input File", err_code);
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
              impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Output File", err_code);
            }
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

        if (file_count != 2)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input or Output File",
                                err_code);
        }
        if (err_code == IA_NO_ERROR)
        {
          err_code = impegh_mp4_multiplex();
          impeghe_error_handler(&ia_mpeghe_error_info, (pWORD8) "", err_code);
        }

        if (g_pf_inp)
        {
          fclose(g_pf_inp);
          g_pf_inp = NULL;
        }

        if (g_pf_out)
        {
          fclose(g_pf_out);
          g_pf_out = NULL;
        }
      }

      printf("Encoding process complete\n");
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
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input File", err_code);
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
          impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Output File", err_code);
        }
        file_count++;
      }

      /*op fmt*/
      if (!strncmp((pCHAR8)argv[i], "-op_fmt:", 8))
      {
        pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
        op_fmt = atoi(pb_arg_val);
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
      impeghe_error_handler(&ia_testbench_error_info, (pWORD8) "Input or Output File", err_code);
    }

    if (err_code == IA_NO_ERROR)
    {
      err_code = impegh_mp4_multiplex();
      impeghe_error_handler(&ia_mpeghe_error_info, (pWORD8) "", err_code);
    }

    if (g_pf_inp)
    {
      fclose(g_pf_inp);
      g_pf_inp = NULL;
    }

    if (g_pf_out)
    {
      fclose(g_pf_out);
      g_pf_out = NULL;
    }
    printf("Encoding process complete\n");
  }

  return IA_NO_ERROR;
} /* end ia_param_file_process */
