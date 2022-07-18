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
#include <impegh_type_def.h>
#include "impegh_error_standards.h"
#include "impegh_error_handler.h"

/*****************************************************************************/
/* Global memory constants                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* Ittiam enhaacplus_dec ErrorCode Definitions                               */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: API Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_api_non_fatal[IA_MAX_ERROR_SUB_CODE] = {(pWORD8) "No Error",
                                                           (pWORD8) "API Command not supported"};
/* Fatal Errors */
pWORD8 impeghd_ppb_api_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid Memory Table Index",
    (pWORD8) "Invalid Library ID String Index",
    (pWORD8) "NULL Pointer: Memory Allocation Error",
    (pWORD8) "Invalid API command",
    (pWORD8) "Invalid Config Param",
    (pWORD8) "Invalid Execute type",
    (pWORD8) "Invalid Command",
    (pWORD8) "Memory Allocation Error: Alignment requirement not met"};
/*****************************************************************************/
/* Class 1: Configuration Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_config_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid pcm word size value", (pWORD8) "Invalid mhas flag value",
    (pWORD8) "Invalid target loudness value", (pWORD8) "Invalid effect type"};
/* Fatal Errors */
pWORD8 impeghd_ppb_config_fatal[IA_MAX_ERROR_SUB_CODE] = {(pWORD8) ""};
/*****************************************************************************/
/* Class 2: Initialization Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_init_non_fatal[IA_MAX_ERROR_SUB_CODE] = {(pWORD8) "", (pWORD8) "",
                                                            (pWORD8) ""};
/* Fatal Errors */
pWORD8 impeghd_ppb_init_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Decoder initialization failed",
    (pWORD8) "No. of channels in stream greater than max channels defined",
    (pWORD8) "AudioObjectType is not supported",
    (pWORD8) "Sample rate is not supported",
    (pWORD8) "Zero Receiver Compensation Delay not valid for multisignal groups",
    (pWORD8) "Invalid fill bytes",
    (pWORD8) "3D audio config data not found",
    (pWORD8) "Invalid SBR framelength index",
    (pWORD8) "Decoder configuration failed",
    (pWORD8) "Binaural Renderer failed",
    (pWORD8) "OAM Initialization failed",
    (pWORD8) "HOA Initialization failed",
    (pWORD8) "HOA Render MatrixInitialization failed",
    (pWORD8) "HOA Ambient synthesis initialization failed",
    (pWORD8) "HOA Render initialization failed",
};
/*****************************************************************************/
/* Class 3: Execution Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
pWORD8 impeghd_ppb_exe_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "",
    (pWORD8) "",
    (pWORD8) "",
    (pWORD8) "",
    (pWORD8) "Input bytes insufficient for decoding",
};
/* Fatal Errors */
pWORD8 impeghd_ppb_exe_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Decode frame error",
    (pWORD8) "Unsupported LC Profile Bitstream param encountered",
    (pWORD8) "MDP process error",
    (pWORD8) "Channel Configuration error",
    (pWORD8) "",
    (pWORD8) "Unsupported Object Metadata decoding",
    (pWORD8) "3D audio frame data not found",
    (pWORD8) "HOA spatial process failed",
    (pWORD8) "HOA decode process failed",
    (pWORD8) "",
    (pWORD8) "",
    (pWORD8) "MCT process failed",
};

pWORD8 impeghd_ppb_demux_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Invalid Memory allocation",
};

pWORD8 impeghd_ppb_demux_non_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "HeaderLength greater than MAX_HEADER_LENGTH",
    (pWORD8) "MP4 Init failed",
    (pWORD8) "Unable to find mdat",
    (pWORD8) "Unable to find stsz",
};
/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct impeghd_error_info = {
    /* The Module Name  */
    (pWORD8) "Ittiam mpegh_dec",
    {/* The Class Names  */
     (pWORD8) "API", (pWORD8) "Configuration", (pWORD8) "Initialization", (pWORD8) "Execution",
     (pWORD8) "Demultiplexer", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "MPEGHD"},
    {/* The Message Pointers  */
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL},
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL}}};

/*****************************************************************************/
/*                                                                           */
/*  Function name : impegh_error_handler_init                     */
/*                                                                           */
/*  Description   : Initialize the error struct with string pointers         */
/*                                                                           */
/*  Inputs        : none                                                     */
/*                                                                           */
/*  Globals       : ia_error_info_struct impeghd_error_info        */
/*                  pWORD8 impeghd_ppb_api_non_fatal               */
/*                  pWORD8 impeghd_ppb_api_fatal                   */
/*                  pWORD8 impeghd_ppb_config_non_fatal            */
/*                  pWORD8 impeghd_ppb_config_fatal                */
/*                  pWORD8 impeghd_ppb_init_non_fatal              */
/*                  pWORD8 impeghd_ppb_init_fatal                  */
/*                  pWORD8 impeghd_ppb_exe_non_fatal               */
/*                  pWORD8 impeghd_ppb_exe_fatal                   */
/*                                                                           */
/*  Processing    : Init the struct with error string pointers               */
/*                                                                           */
/*  Outputs       : none                                                     */
/*                                                                           */
/*  Returns       : none                                                     */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*  Revision history :                                                       */
/*                                                                           */
/*        DD MM YYYY       Author                Changes                     */
/*        29 07 2005       Ittiam                Created                     */
/*                                                                           */
/*****************************************************************************/

VOID impegh_error_handler_init()
{
  /* The Message Pointers  */
  impeghd_error_info.ppppb_error_msg_pointers[0][0] = impeghd_ppb_api_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][0] = impeghd_ppb_api_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][1] = impeghd_ppb_config_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][1] = impeghd_ppb_config_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][2] = impeghd_ppb_init_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][2] = impeghd_ppb_init_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][3] = impeghd_ppb_exe_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][3] = impeghd_ppb_exe_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[0][4] = impeghd_ppb_demux_non_fatal;
  impeghd_error_info.ppppb_error_msg_pointers[1][4] = impeghd_ppb_demux_fatal;
}

/*****************************************************************************/
/* ia_testbench ErrorCode Definitions                                        */
/*****************************************************************************/
/*****************************************************************************/
/* Class 0: Memory & File Manager Errors
 */
/*****************************************************************************/
/* Non Fatal Errors */
/* Fatal Errors */
pWORD8 impeghd_ppb_ia_testbench_mem_file_man_fatal[IA_MAX_ERROR_SUB_CODE] = {
    (pWORD8) "Memory Allocation Error", (pWORD8) "File Open Failed"};

/*****************************************************************************/
/* error info structure                                                      */
/*****************************************************************************/
/* The Module's Error Info Structure */
ia_error_info_struct impeghd_ia_testbench_error_info = {
    /* The Module Name  */
    (pWORD8) "ia_testbench",
    {/* The Class Names  */
     (pWORD8) "Memory & File Manager", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) "",
     (pWORD8) "", (pWORD8) "", (pWORD8) "", (pWORD8) ""},
    {/* The Message Pointers  */
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL},
     {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL}}};

/*****************************************************************************/
/*                                                                           */
/*  Function name : ia_testbench_error_handler_init                          */
/*                                                                           */
/*  Description   : Initialize the error struct with string pointers         */
/*                                                                           */
/*  Inputs        : none                                                     */
/*                                                                           */
/*  Globals       : ia_error_info_struct impeghd_ia_testbench_error_info */
/*                  pWORD8 impeghd_ppb_ia_testbench_mem_file_man_fatal */
/*                                                                           */
/*  Processing    : Init the struct with error string pointers               */
/*                                                                           */
/*  Outputs       : none                                                     */
/*                                                                           */
/*  Returns       : none                                                     */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*  Revision history :                                                       */
/*                                                                           */
/*        DD MM YYYY       Author                Changes                     */
/*        29 07 2005       Ittiam                Created                     */
/*                                                                           */
/*****************************************************************************/

VOID ia_testbench_error_handler_init()
{
  /* The Message Pointers  */
  impeghd_ia_testbench_error_info.ppppb_error_msg_pointers[1][0] =
      impeghd_ppb_ia_testbench_mem_file_man_fatal;
}

/*****************************************************************************/
/*                                                                           */
/*  Function name : impegh_error_handler */
/*                                                                           */
/*  Description   : Called Prints the status error code from the err_info    */
/*                                                                           */
/*  Inputs        : ia_error_info_struct *p_mod_err_info (Error info struct) */
/*                  WORD8 *pb_context (Context of error)                     */
/*                  IA_ERRORCODE code (Error code)                           */
/*                                                                           */
/*  Globals       : none                                                     */
/*                                                                           */
/*  Processing    : whenever any module calls the errorhandler,  it  informs */
/*                  it about the module for which it is called and a context */
/*                  in which it was  called  in addition to  the  error_code */
/*                  the message is displayed  based  on the  module's  error */
/*                  message  array  that maps to  the error_code the context */
/*                  gives specific info in which the error occured  e.g. for */
/*                  testbench   module,  memory  allocator   can   call  the */
/*                  errorhandler   for  memory  inavailability  in   various */
/*                  contexts like input_buf or output_buf e.g.  for  mp3_enc */
/*                  module, there can be various instances running.  context */
/*                  can be used to  identify  the  particular  instance  the */
/*                  error handler is being called for                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : IA_ERRORCODE error_value  (Error value)                  */
/*                                                                           */
/*  Issues        : none                                                     */
/*                                                                           */
/*  Revision history :                                                       */
/*                                                                           */
/*        DD MM YYYY       Author                Changes                     */
/*        29 07 2005       Tejaswi/Vishal        Created                     */
/*                                                                           */
/*****************************************************************************/

IA_ERRORCODE impegh_error_handler(ia_error_info_struct *p_mod_err_info, WORD8 *pb_context,
                                  IA_ERRORCODE code)
{
  if (code == IA_NO_ERROR)
  {
    return IA_NO_ERROR;
  }
  {
    WORD is_fatal = (((UWORD)code & 0x8000) >> 15);
    WORD err_class = (((UWORD)code & 0x7800) >> 11);
    WORD err_sub_code = (((UWORD)code & 0x07FF));
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
      printf("error unlisted\n");
    }
    else
    {
      printf("%s\n", p_mod_err_info->ppppb_error_msg_pointers[is_fatal][err_class][err_sub_code]);
    }
  }
  return IA_NO_ERROR;
}
