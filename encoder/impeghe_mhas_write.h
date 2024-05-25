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

#ifndef IMPEGHE_MHAS_WRITE_H
#define IMPEGHE_MHAS_WRITE_H

#define MAX_MHAS_PKT_HDR_SZ (15)

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
#define MHAS_PAC_TYP_PCM_CONFIG (20)
#define MHAS_PAC_TYP_PCM_DATA (21)
#define MHAS_PAC_TYP_UNDEF (518)

#define MHAS_SYNC_BYTE (0xA5)

#define MAX_MAE_NUM_DATASETS (15)

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

// Values obtained from Table 11 of section 4.8.2.2 of specification - 23008-3
// Restrictions applicable for LC profile
#define MAX_NUM_GROUPS (28)
#define MAX_NUM_SWITCH_GROUPS (14)
#define MAX_NUM_GROUPS_PRESETS (16)
#define MAX_GROUP_PRESET_NUM_CONDITIONS (16)
#define MAX_NUM_PRESET_GROUP_EXTENSIONS (16)

// Group Definition
#define MAX_GROUP_NUM_MEMBERS (128)
#define MAX_SWITCH_GROUP_NUM_MEMBERS (32)

#define MAX_NUM_CONTENT_DATA_BLOCKS (128)

#define MAX_NUM_DESCRIPTOIN_BLOCKS (128)
#define MAX_DESCR_LANGUAGE_DATA_LEN (16)
#define MAX_DESCRIPTON_DATA_LEN (256)
#define MAX_NUM_COMPOSITE_PAIRS (128)
#define MAX_NUM_TGT_LOUDNESS_CONDITIONS (7)
#define MAX_NUM_PRESET_PROD_SCREENS (31)

#define UNIDRCLOUDEXT_TERM (0x0)
#define UNIDRCLOUDEXT_EQ   (0x1)

#define UNIDRCCONFEXT_TERM      (0x0)
#define UNIDRCCONFEXT_PARAM_DRC (0x1)
#define UNIDRCCONFEXT_V1        (0x2)

// structures
typedef struct
{
  WORD32 packet_type;
  WORD32 packet_lbl;
  WORD32 packet_length;
} ia_mhas_pac_info;

#endif /*IMPEGHE_MHAS_WRITE_H*/