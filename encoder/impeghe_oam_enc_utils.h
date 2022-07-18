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

#ifndef IMPEGHE_OAM_ENC_UTILS_H
#define IMPEGHE_OAM_ENC_UTILS_H

#define NO_BITS_AZI_OAM (8)
#define NO_BITS_ELE_OAM (6)
#define NO_BITS_RAD_OAM (4)
#define NO_BITS_GAIN_OAM (7)
#define NO_BITS_SPREAD_OAM (7)
#define NO_BITS_SPREAD_HEIGHT_OAM (5)
#define NO_BITS_SPREAD_DEPTH_OAM (4)
#define NO_BITS_PRIORITY_OAM (3)

#ifndef DB
#define DB(x) (20.0 * log10((x)))
#endif
#ifndef LOG2
#define LOG2(x) (log((x)) / log(2.0))
#endif
#ifndef ROUND
#define ROUND(x) (((x) < (0)) ? (WORD32)((x)-0.5f) : (WORD32)((x) + 0.5f))
#endif

// Table 132 — Units of the decoded object metadata components
#define AZIMUTH_MIN_DEGREE (-180.0f)
#define AZIMUTH_MAX_DEGREE (180.0f)
#define ELEVATION_MIN_DEGREE (-90.0f)
#define ELEVATION_MAX_DEGREE (90.0f)
#define RADIUS_MIN_METER (0.5f)
#define RADIUS_MAX_METER (16.0f)
#define GAIN_MIN_VALUE (0.004f)
#define GAIN_MAX_VALUE (5.957f)
#define SPREAD_MIN_DEGREE (0.0f)
#define SPREAD_MAX_DEGREE (180.0f)
#define SPREAD_WIDTH_MIN_DEGREE (0.0f)
#define SPREAD_WIDTH_MAX_DEGREE (180.0f)
#define SPREAD_HEIGHT_MIN_DEGREE (0.0f)
#define SPREAD_HEIGHT_MAX_DEGREE (90.0f)
#define SPREAD_DEPTH_MIN_METER (0.0f)
#define SPREAD_DEPTH_MAX_METER (15.5f)
#define DYN_OBJ_PRIORITY_MIN_VALUE (0.0f)
#define DYN_OBJ_PRIORITY_MAX_VALUE (7.0f)

// Scaling formulae
#define SCALE_AZIMUTH(x) ((x) / 1.5f)
#define SCALE_ELEVATION(x) ((x) / 3.0f)
#define SCALE_RADIUS(x) (3.0f * (FLOAT32)LOG2(2.0 * (x)))
#define SCALE_GAIN(x) (2.0f * (FLOAT32)DB((x)) + 32.0f)
#define SCALE_SPREAD(x) ((x) / 1.5f)
#define SCALE_SPREAD_HEIGHT(x) ((x) / 3.0f)
#define SCALE_SPREAD_DEPTH(x) (3.0f * (FLOAT32)LOG2(2.0 * (x) + 0.5f))

// Bit value range
#define OAM_BIT_LIMIT_SIGN_AZIMUTH_MIN (-128.0f)       // Min value in NO_BITS_AZI_OAM
#define OAM_BIT_LIMIT_SIGN_AZIMUTH_MAX (127.0f)        // Max value in NO_BITS_AZI_OAM
#define OAM_BIT_LIMIT_SIGN_ELEVATION_MIN (-32.0f)      // Min value in NO_BITS_ELE_OAM
#define OAM_BIT_LIMIT_SIGN_ELEVATION_MAX (31.0f)       // Max value in NO_BITS_ELE_OAM
#define OAM_BIT_LIMIT_SIGN_GAIN_MIN (-64.0f)           // Min value in NO_BITS_GAIN_OAM
#define OAM_BIT_LIMIT_SIGN_GAIN_MAX (63.0f)            // Max value in NO_BITS_GAIN_OAM
#define OAM_BIT_LIMIT_UNSIGN_RADIUS_MAX (15.0f)        // Max value in NO_BITS_RAD_OAM
#define OAM_BIT_LIMIT_UNSIGN_SPREAD_MAX (127.0f)       // Max value in NO_BITS_SPREAD_OAM
#define OAM_BIT_LIMIT_UNSIGN_SPREAD_HEIGHT_MAX (31.0f) // Max value in NO_BITS_SPREAD_HEIGHT_OAM
#define OAM_BIT_LIMIT_UNSIGN_SPREAD_DEPTH_MAX (15.0f)  // Max value in NO_BITS_SPREAD_DEPTH_OAM
#define OAM_BIT_LIMIT_UNSIGN_PRIORITY_MAX (7.0f)       // Max value in NO_BITS_PRIORITY_OAM

#define OAM_BIT_LIMIT_SIGN_VALUE(x, min, max) (MIN(MAX((x), (min)), (max)))
#define OAM_BIT_LIMIT_UNSIGN_VALUE(x, max) (OAM_BIT_LIMIT_SIGN_VALUE((x), (0.0f), (max)))

#define OAM_BIT_SIGN_MIN_VALUE(x) (-(1 << ((x)-1)))
#define OAM_BIT_SIGN_MAX_VALUE(x) (((1 << (x)) - 1) + OAM_BIT_SIGN_MIN_VALUE(x))

IA_ERRORCODE impeghe_obj_md_get_scaled_chunk_ld(ia_oam_enc_state_struct *ptr_oam_enc_state,
                                                ia_oam_enc_multidata_struct *pstr_oam_data,
                                                WORD32 num_samples);

WORD32 impeghe_obj_md_bitbuf_write(ia_bit_buf_struct *it_bit_buf, UWORD8 *ptr_buf,
                                   UWORD32 num_bits);

#endif /*IMPEGHE_OAM_ENC_UTILS_H*/
