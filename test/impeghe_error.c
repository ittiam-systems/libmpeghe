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
#include <stdio.h>
#include "impeghe_type_def.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_handler.h"

/*****************************************************************************/
/* Global memory constants                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* Ittiam mpeghe ErrorCode Definitions                             */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: API Errors                                                       */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 ppb_ia_mpeghe_api_non_fatal[] = {
    (pWORD8) "",
};
/* Fatal Errors */
pWORD8 ppb_ia_mpeghe_api_fatal[] = {
    (pWORD8) "NULL Pointer: Memory Allocation Error",
    (pWORD8) "Invalid Config Param",
};
/*****************************************************************************/
/* Class 1: Configuration Errors                                             */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 ppb_ia_mpeghe_config_non_fatal[] = {
    (pWORD8) "OAM out of range value",
    (pWORD8) "OAM not available",
    (pWORD8) "OAM number of objects unsupported, restricting to 24 objects or the number of "
             "object files present",
    (pWORD8) "HOA input data seems to be corrupted",
    (pWORD8) "Invalid HOA matrix input",
    (pWORD8) "Missing Config for DRC",
    (pWORD8) "Invalid downmix config",
};
/* Fatal Errors */
pWORD8 ppb_ia_mpeghe_config_fatal[] = {
    (pWORD8) "Invalid Sampling Frequency", (pWORD8) "Invalid Stream bit rate",
    (pWORD8) "Invalid Codec mode",         (pWORD8) "Invalid PCM size",
    (pWORD8) "Invalid OAM config",         (pWORD8) "OAM file read failed",
    (pWORD8) "OAM header invalid",         (pWORD8) "OAM frame invalid",
    (pWORD8) "Invalid ASI config",         (pWORD8) "Invalid DRC config",
    (pWORD8) "Unsupported DRC config",     (pWORD8) "DRC parameter out of range",
    (pWORD8) "DRC gain calculation error",
};

/*****************************************************************************/
/* Class 2: Initialization Errors                                             */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 ppb_ia_mpeghe_init_non_fatal[] = {
    (pWORD8) "",
};
/* Fatal Errors */
pWORD8 ppb_ia_mpeghe_init_fatal[] = {
    (pWORD8) "Insufficient write buffer size",
    (pWORD8) "Insufficient OAM buffer size",
    (pWORD8) "Insufficient DRC buffer size",
    (pWORD8) "Invalid HOA order",
    (pWORD8) "Invalid HOA quantization",
    (pWORD8) "Invalid HOA interpolation sample size",
    (pWORD8) "Invalid HOA ambient component initialization",
    (pWORD8) "Invalid HOA mode matrix",
    (pWORD8) "Invalid HOA vector size",
};

/*****************************************************************************/
/* Class 3: Execution Errors                                             */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 ppb_ia_mpeghe_exe_non_fatal[] = {
    (pWORD8) "Insufficient write buffer size",
    (pWORD8) "Insufficient MCT buffer size",
    (pWORD8) "HOA vector estimation error",
};
/* Fatal Errors */
pWORD8 ppb_ia_mpeghe_exe_fatal[] = {
    (pWORD8) "Invalid fac len",
};

/*****************************************************************************/
/* Class 4: Multiplexer Errors                                             */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 ppb_ia_mpeghe_mux_non_fatal[] = {
    (pWORD8) "Invalid bit position",     (pWORD8) "Sampling rate not supported",
    (pWORD8) "Sync word mismatch",       (pWORD8) "Header length too large",
    (pWORD8) "Insufficient input bytes", (pWORD8) "Invalid ASI configuration value"};
/* Fatal Errors */
pWORD8 ppb_ia_mpeghe_mux_fatal[] = {
    (pWORD8) "",
};

/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct ia_mpeghe_error_info = {
    /* The Module Name	*/
    (pWORD8) "Ittiam mpegh enc",
    {/* The Class Names	*/
     (pWORD8) "API", (pWORD8) "Configuration", (pWORD8) "Initialization", (pWORD8) "Execution",
     (pWORD8) "Multiplexer ", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) ""},
    {/* The Message Pointers	*/
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL},
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL}}};

/**
 *  impeghe_error_handler_init
 *
 *  \brief Brief description
 *
 *  \return VOID
 *
 */
VOID impeghe_error_handler_init()
{
  /* The Message Pointers	*/
  ia_mpeghe_error_info.ppppb_error_msg_pointers[0][0] = ppb_ia_mpeghe_api_non_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[1][0] = ppb_ia_mpeghe_api_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[0][1] = ppb_ia_mpeghe_config_non_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[1][1] = ppb_ia_mpeghe_config_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[0][2] = ppb_ia_mpeghe_init_non_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[1][2] = ppb_ia_mpeghe_init_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[0][3] = ppb_ia_mpeghe_exe_non_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[1][3] = ppb_ia_mpeghe_exe_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[0][4] = ppb_ia_mpeghe_mux_non_fatal;
  ia_mpeghe_error_info.ppppb_error_msg_pointers[1][4] = ppb_ia_mpeghe_mux_fatal;
}

/**
 *  impeghe_error_handler
 *
 *  \brief Brief description
 *
 *  \param p_mod_err_info
 *  \param pb_context
 *  \param code
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghe_error_handler(ia_error_info_struct *p_mod_err_info, WORD8 *pb_context,
                                   IA_ERRORCODE code)
{
  if (code == IA_NO_ERROR)
  {
    return IA_NO_ERROR;
  }
  {
    WORD32 is_fatal = (((UWORD32)code & 0x8000) >> 15);
    WORD32 err_class = (((UWORD32)code & 0x7800) >> 11);
    WORD32 err_sub_code = (((UWORD32)code & 0x07FF));

    printf("\n");
    if (!is_fatal)
    {
      printf("non ");
    }
    printf("fatal error: ");

    if (p_mod_err_info->pb_module_name != NULL)
    {
      printf("%s: ", p_mod_err_info->pb_module_name);
    }
    if (p_mod_err_info->ppb_class_names[err_class] != NULL)
    {
      printf("%s: ", p_mod_err_info->ppb_class_names[err_class]);
    }
    if (pb_context != NULL)
    {
      printf("%s: ", pb_context);
    }

    if (err_sub_code >= IA_MAX_ERROR_SUB_CODE ||
        p_mod_err_info->ppppb_error_msg_pointers[is_fatal][err_class][err_sub_code] == NULL)
    {
      printf("error unlisted");
    }
    else
    {
      printf("%s\n", p_mod_err_info->ppppb_error_msg_pointers[is_fatal][err_class][err_sub_code]);
    }
  }
  return IA_NO_ERROR;
}

/*****************************************************************************/
/* ia_testbench ErrorCode Definitions                                        */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: Memory & File Manager Errors                                     */
/*****************************************************************************/
/* Non Fatal Errors */
/* Fatal Errors */
pWORD8 ppb_ia_testbench_mem_file_man_fatal[] = {(pWORD8) "Memory Allocation Error",
                                                (pWORD8) "File Open Failed"};

/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct ia_testbench_error_info = {
    /* The Module Name	*/
    (pWORD8) "ia_testbench",
    {/* The Class Names	*/
     (pWORD8) "Memory & File Manager", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) ""},
    {/* The Message Pointers	*/
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL},
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL}}};

/**
 *  impeghe_testbench_error_handler_init
 *
 *  \brief Brief description
 *
 *
 *  \return VOID
 *
 */
VOID impeghe_testbench_error_handler_init()
{
  /* The Message Pointers	*/
  ia_testbench_error_info.ppppb_error_msg_pointers[1][0] = ppb_ia_testbench_mem_file_man_fatal;
}
