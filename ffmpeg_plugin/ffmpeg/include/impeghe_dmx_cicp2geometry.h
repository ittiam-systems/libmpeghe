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

#ifndef IMPEGHE_DMX_CICP2GEOMETRY_H
#define IMPEGHE_DMX_CICP2GEOMETRY_H

#define CICP2GEOMETRY_MAX_LOUDSPEAKERS (32)

// CICP index
#define CICP2GEOMETRY_CICP_FROM_FILE (-1)
#define CICP2GEOMETRY_CICP_1_0_0 (1)
#define CICP2GEOMETRY_CICP_2_0_0 (2)
#define CICP2GEOMETRY_CICP_3_0_0 (3)
#define CICP2GEOMETRY_CICP_3_1_0 (4)
#define CICP2GEOMETRY_CICP_3_2_0 (5)
#define CICP2GEOMETRY_CICP_3_2_1 (6)
#define CICP2GEOMETRY_CICP_5_2_1 (7)
#define CICP2GEOMETRY_CICP_1_1 (8)
#define CICP2GEOMETRY_CICP_2_1_0 (9)
#define CICP2GEOMETRY_CICP_2_2_0 (10)
#define CICP2GEOMETRY_CICP_3_3_1 (11)
#define CICP2GEOMETRY_CICP_3_4_1 (12)
#define CICP2GEOMETRY_CICP_11_11_2 (13)
#define CICP2GEOMETRY_CICP_5_2_1_ELEVATION (14)
#define CICP2GEOMETRY_CICP_5_5_2 (15)
#define CICP2GEOMETRY_CICP_5_4_1 (16)
#define CICP2GEOMETRY_CICP_6_5_1 (17)
#define CICP2GEOMETRY_CICP_6_7_1 (18)
#define CICP2GEOMETRY_CICP_7_4_1 (19)
#define CICP2GEOMETRY_CICP_9_4_1 (20)
#define CICP2GEOMETRY_CICP_INVALID (-1000)

typedef struct
{
  WORD16 elevation;
  WORD16 azimuth;
  WORD16 is_lfe;
} ia_dmx_channel_geometry_struct;

IA_ERRORCODE impeghe_dmx_get_geometry_from_cicp(
    WORD32 cicp_index,
    ia_dmx_channel_geometry_struct channel_geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS],
    WORD32 *ptr_num_channels, WORD32 *ptr_num_lfe);

#endif /* IMPEGHE_DMX_CICP2GEOMETRY_H */
