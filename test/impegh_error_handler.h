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

#ifndef IMPEGH_ERROR_HANDLER_H
#define IMPEGH_ERROR_HANDLER_H

#define IA_ERROR_NON_FATAL_IDX 0x0
#define IA_ERROR_FATAL_IDX 0x1

#define IA_ERROR_CLASS_0 0x0
#define IA_ERROR_CLASS_1 0x1
#define IA_ERROR_CLASS_2 0x2
#define IA_ERROR_CLASS_3 0x3
#define IA_ERROR_CLASS_4 0x4
#define IA_ERROR_CLASS_5 0x5
#define IA_ERROR_CLASS_6 0x6
#define IA_ERROR_CLASS_7 0x7
#define IA_ERROR_CLASS_8 0x8
#define IA_ERROR_CLASS_9 0x9
#define IA_ERROR_CLASS_A 0xA
#define IA_ERROR_CLASS_B 0xB
#define IA_ERROR_CLASS_C 0xC
#define IA_ERROR_CLASS_D 0xD
#define IA_ERROR_CLASS_E 0xE
#define IA_ERROR_CLASS_F 0xF

#define IA_MAX_ERROR_SUB_CODE 28

typedef struct
{
  pWORD8 pb_module_name;
  pWORD8 ppb_class_names[16];
  WORD8 **ppppb_error_msg_pointers[2][16];
} ia_error_info_struct;

IA_ERRORCODE impegh_error_handler(ia_error_info_struct *p_mod_err_info, WORD8 *pb_context,
                                  IA_ERRORCODE code);

#define _IA_HANDLE_ERROR(p_mod_err_info, context, e)                                             \
  if ((e) != IA_NO_ERROR)                                                                        \
  {                                                                                              \
    impegh_error_handler((p_mod_err_info), (context), (e));                                      \
  }

#endif /* IMPEGH_ERROR_HANDLER_H */
