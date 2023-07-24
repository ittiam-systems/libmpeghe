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

#ifndef IMPEGHE_ERROR_CODES_H
#define IMPEGHE_ERROR_CODES_H

/*****************************************************************************/
/* Constant hashdefines                                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Ittiam MPEGH-LC encoder error code Definitions                             */
/*****************************************************************************/

typedef enum{
  /* Class 0: API Errors */

  /* Fatal Errors */
  IMPEGHE_API_FATAL_MEM_ALLOC = 0xFFFF8001,

  /* Class 1: Configuration Errors */
  /* Non Fatal Errors */

  // OAM Related
  IMPEGHE_CONFIG_NONFATAL_OAM_OUT_OF_RANGE_VALUE = 0x00000800,
  IMPEGHE_CONFIG_NONFATAL_OAM_NOT_AVAILABLE,
  IMPEGHE_CONFIG_NONFATAL_NUM_OBJECTS_UNSUPPORTED,

  // HOA Related
  IMPEGHE_CONFIG_NONFATAL_INPUT_OUT_OF_RANGE,
  IMPEGHE_CONFIG_NONFATAL_HOA_MATRIX_INVALID_INPUT,

  // DRC Related
  IMPEGHE_CONFIG_NONFATAL_DRC_MISSING_CONFIG,

  // Downmix Related
  IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG,

  /* Fatal Errors */
  IMPEGHE_CONFIG_FATAL_SAMP_FREQ = 0xFFFF8800,
  IMPEGHE_CONFIG_FATAL_BITRATE,
  IMPEGHE_CONFIG_FATAL_CODEC_MODE,
  IMPEGHE_CONFIG_FATAL_PCM_SIZE,
  IMPEGHE_CONFIG_FATAL_NUM_CHANNELS_UNSUPPORTED,

  // OAM Related
  IMPEGHE_CONFIG_FATAL_OAM_INVALID_CONFIG,
  IMPEGHE_CONFIG_FATAL_OAM_READ_FAILED,
  IMPEGHE_CONFIG_FATAL_OAM_INVALID_HEADER,
  IMPEGHE_CONFIG_FATAL_OAM_INVALID_FRAME,
  IMPEGHE_CONFIG_FATAL_ASI_INVALID_CONFIG,

  // DRC Related
  IMPEGHE_CONFIG_FATAL_DRC_INVALID_CONFIG,
  IMPEGHE_CONFIG_FATAL_DRC_UNSUPPORTED_CONFIG,
  IMPEGHE_CONFIG_FATAL_DRC_PARAM_OUT_OF_RANGE,
  IMPEGHE_CONFIG_FATAL_DRC_COMPAND_FAIL,

  /* Class 2: Initialization Errors */
  /* Non Fatal Errors */

  /* Fatal Errors */
  IMPEGHE_INIT_FATAL_INSUFFICIENT_WRITE_BUFFER_SIZE = 0xFFFF9000,
  IMPEGHE_INIT_FATAL_INSUFFICIENT_OAM_WRITE_BUFFER_SIZE,
  IMPEGHE_INIT_FATAL_INSUFFICIENT_DRC_WRITE_BUFFER_SIZE,

  // HOA Related
  IMPEGHE_INIT_FATAL_INVALID_HOA_ORDER,
  IMPEGHE_INIT_FATAL_INVALID_QUANT,
  IMPEGHE_INIT_FATAL_INVALID_INTERP_SAMPLE_SIZE,
  IMPEGHE_INIT_FATAL_INVALID_AMB_INIT,
  IMPEGHE_INIT_FATAL_INVALID_MATRIX,
  IMPEGHE_INIT_FATAL_INVALID_VEC_ELE,

  /* Class 3: Execution Errors */
  /* Non Fatal Errors */
  IMPEGHE_EXE_NONFATAL_INSUFFICIENT_WRITE_BUFFER_SIZE = 0x00001800,
  IMPEGHE_EXE_NONFATAL_INSUFFICIENT_MCT_WRITE_BUFFER_SIZE,

  // HOA Related
  IMPEGHE_EXE_NONFATAL_HOA_VEC_EST_ERROR,

  /* Fatal Errors */
  IMPEGHE_EXE_FATAL_INVALID_FAC_LEN = 0xFFFF9800,
}IMPEGHE_ERROR_CODE;

#endif /* IMPEGHE_ERROR_CODES_H */
