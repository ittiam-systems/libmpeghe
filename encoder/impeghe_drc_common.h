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

#ifndef IMPEGHE_DRC_COMMON_H
#define IMPEGHE_DRC_COMMON_H

#define MAX_DRC_PAYLOAD_BYTES (2048)
#define MAX_SPEAKER_POS_COUNT (128)
#define MAX_DOWNMIX_COEFF_COUNT (32 * 32)
#define MAX_CHANNEL_COUNT (128)
#define MAX_BAND_COUNT (8)
#define MAX_SEQUENCE_COUNT (8)
#define MAX_MEASUREMENT_COUNT (16)
#define MAX_DOWNMIX_INSTRUCTION_COUNT (16)
#define MAX_DRC_COEFF_COUNT (8)
#define MAX_DRC_INSTRUCTIONS_COUNT (MAX_DOWNMIX_INSTRUCTION_COUNT + 16)
#define MAX_LOUDNESS_INFO_COUNT (MAX_DOWNMIX_INSTRUCTION_COUNT + 16)
#define MAX_AUDIO_CODEC_FRAME_SIZE (2048)
#define MAX_DRC_CODEC_FRAME_SIZE (MAX_AUDIO_CODEC_FRAME_SIZE / 8)
#define MAX_NODE_COUNT (MAX_DRC_CODEC_FRAME_SIZE)
#define MAX_CHANNEL_GROUP_COUNT (MAX_SEQUENCE_COUNT)
#define MAX_ADDITIONAL_DOWNMIX_ID (8)
#define DELAY_MODE_REGULAR_DELAY (0)
#define DELAY_MODE_LOW_DELAY (1)
#define DELAY_MODE_DEFAULT (DELAY_MODE_REGULAR_DELAY)
#define MAX_EXT_COUNT (2)

#define UNIDRC_GAIN_EXT_TERM (0x0)
#define UNIDRC_LOUD_EXT_TERM (0x0)
#define UNIDRC_CONF_EXT_TERM (0x0)
#define UNIDRC_CONF_EXT_PARAM_DRC (0x1)
#define UNIDRC_CONF_EXT_V1 (0x2)
#define UNIDRC_LOUD_EXT_EQ (0x1)

#define MAX_PARAM_DRC_INSTRUCTIONS_COUNT (8)

#define PARAM_DRC_TYPE_FF (0x0)
#define MAX_PARAM_DRC_TYPE_FF_NODE_COUNT (9)

#define PARAM_DRC_TYPE_LIM (0x1)
#define PARAM_DRC_TYPE_LIM_THRESHOLD_DEFAULT (-1.f)
#define PARAM_DRC_TYPE_LIM_ATTACK_DEFAULT (5)
#define PARAM_DRC_TYPE_LIM_RELEASE_DEFAULT (50)

#define SUBBAND_DOMAIN_MODE_OFF (0)
#define SUBBAND_DOMAIN_MODE_QMF64 (1)
#define SUBBAND_DOMAIN_MODE_QMF71 (2)
#define SUBBAND_DOMAIN_MODE_STFT256 (3)

#define QMF64_AUDIO_CODEC_SUBBAND_COUNT (64)
#define QMF64_AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR (64)

#define QMF71_AUDIO_CODEC_SUBBAND_COUNT (71)
#define QMF71_AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR (64)

#define STFT256_AUDIO_CODEC_SUBBAND_COUNT (256)
#define STFT256_AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR (256)

#define TIME_DOMAIN (1)
#define SUBBAND_DOMAIN (2)
#define SLOPE_FACTOR_DB_TO_LINEAR (0.1151f) /* ln(10) / 20 */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#endif /*IMPEGHE_DRC_COMMON_H*/
