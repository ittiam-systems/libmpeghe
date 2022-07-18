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

#include <string.h>
#include "impeghe_error_codes.h"
#include "impeghe_error_standards.h"
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_dmx_cicp2geometry.h"

/**
 *  impeghe_dmx_get_geometry_from_cicp
 *
 *  \brief Gets geometry from CICP index
 *
 *  \param [in] 	cicp_index       CICP index
 *  \param [out]	channel_geometry Pointer to channel geometry array of loudspeakers
 *  \param [out] 	ptr_num_channels Pointer to number of channels
 *  \param [out]	ptr_num_lfe      Pointer to number of LFE channels
 *
 *  \return IA_ERRORCODE Error code
 */
IA_ERRORCODE impeghe_dmx_get_geometry_from_cicp(
    WORD32 cicp_index,
    ia_dmx_channel_geometry_struct channel_geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS],
    WORD32 *ptr_num_channels, WORD32 *ptr_num_lfe)
{

  switch (cicp_index)
  {

  case CICP2GEOMETRY_CICP_1_1:
    *ptr_num_channels = -1;
    *ptr_num_lfe = -1;
    break;

  case CICP2GEOMETRY_CICP_1_0_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 0;
      channel_geometry[0].is_lfe = 0;
    }

    *ptr_num_channels = 1;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_2_0_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;
    }

    *ptr_num_channels = 2;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_2_1_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 180;
      channel_geometry[2].is_lfe = 0;
    }

    *ptr_num_channels = 3;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_2_2_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 110;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = 0;
      channel_geometry[3].azimuth = -110;
      channel_geometry[3].is_lfe = 0;
    }

    *ptr_num_channels = 4;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_3_0_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;
    }

    *ptr_num_channels = 3;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_3_1_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = 0;
      channel_geometry[3].azimuth = 180;
      channel_geometry[3].is_lfe = 0;
    }

    *ptr_num_channels = 4;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_3_2_0:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = 0;
      channel_geometry[3].azimuth = 110;
      channel_geometry[3].is_lfe = 0;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = -110;
      channel_geometry[4].is_lfe = 0;
    }

    *ptr_num_channels = 5;
    *ptr_num_lfe = 0;
    break;

  case CICP2GEOMETRY_CICP_3_2_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;
    }

    *ptr_num_channels = 5;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_3_3_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 180;
      channel_geometry[6].is_lfe = 0;
    }

    *ptr_num_channels = 6;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_3_4_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 135;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -135;
      channel_geometry[7].is_lfe = 0;
    }

    *ptr_num_channels = 7;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_5_2_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 60;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -60;
      channel_geometry[7].is_lfe = 0;
    }

    *ptr_num_channels = 7;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_5_2_1_ELEVATION:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 45;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 35;
      channel_geometry[6].azimuth = 30;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 35;
      channel_geometry[7].azimuth = -30;
      channel_geometry[7].is_lfe = 0;
    }

    *ptr_num_channels = 7;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_5_4_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 35;
      channel_geometry[6].azimuth = 30;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 35;
      channel_geometry[7].azimuth = -30;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 35;
      channel_geometry[8].azimuth = 110;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = -110;
      channel_geometry[9].is_lfe = 0;
    }

    *ptr_num_channels = 9;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_5_5_2:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 45;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 135;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -135;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = -15;
      channel_geometry[6].azimuth = -45;
      channel_geometry[6].is_lfe = 1;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = 90;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 0;
      channel_geometry[8].azimuth = -90;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = 45;
      channel_geometry[9].is_lfe = 0;

      channel_geometry[10].elevation = 35;
      channel_geometry[10].azimuth = -45;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 35;
      channel_geometry[11].azimuth = 180;
      channel_geometry[11].is_lfe = 0;
    }

    *ptr_num_channels = 10;
    *ptr_num_lfe = 2;
    break;

  case CICP2GEOMETRY_CICP_6_5_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 35;
      channel_geometry[6].azimuth = 30;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 35;
      channel_geometry[7].azimuth = -30;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 35;
      channel_geometry[8].azimuth = 0;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = 110;
      channel_geometry[9].is_lfe = 0;

      channel_geometry[10].elevation = 35;
      channel_geometry[10].azimuth = -110;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 90;
      channel_geometry[11].azimuth = 0;
      channel_geometry[11].is_lfe = 0;
    }

    *ptr_num_channels = 11;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_6_7_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 110;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -110;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 150;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -150;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 35;
      channel_geometry[8].azimuth = 30;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = -30;
      channel_geometry[9].is_lfe = 0;

      channel_geometry[10].elevation = 35;
      channel_geometry[10].azimuth = 0;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 35;
      channel_geometry[11].azimuth = 110;
      channel_geometry[11].is_lfe = 0;

      channel_geometry[12].elevation = 35;
      channel_geometry[12].azimuth = -110;
      channel_geometry[12].is_lfe = 0;

      channel_geometry[13].elevation = 90;
      channel_geometry[13].azimuth = 0;
      channel_geometry[13].is_lfe = 0;
    }

    *ptr_num_channels = 13;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_7_4_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 135;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -135;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 90;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -90;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 35;
      channel_geometry[8].azimuth = 30;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = -30;
      channel_geometry[9].is_lfe = 0;

      channel_geometry[10].elevation = 35;
      channel_geometry[10].azimuth = 135;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 35;
      channel_geometry[11].azimuth = -135;
      channel_geometry[11].is_lfe = 0;
    }

    *ptr_num_channels = 11;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_9_4_1:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 30;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -30;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 0;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 135;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -135;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 90;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -90;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 35;
      channel_geometry[8].azimuth = 45;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = 35;
      channel_geometry[9].azimuth = -45;
      channel_geometry[9].is_lfe = 0;

      channel_geometry[10].elevation = 35;
      channel_geometry[10].azimuth = 135;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 35;
      channel_geometry[11].azimuth = -135;
      channel_geometry[11].is_lfe = 0;

      channel_geometry[12].elevation = 0;
      channel_geometry[12].azimuth = 60;
      channel_geometry[12].is_lfe = 0;

      channel_geometry[13].elevation = 0;
      channel_geometry[13].azimuth = -60;
      channel_geometry[13].is_lfe = 0;
    }

    *ptr_num_channels = 13;
    *ptr_num_lfe = 1;
    break;

  case CICP2GEOMETRY_CICP_11_11_2:

    if (channel_geometry != NULL)
    {
      channel_geometry[0].elevation = 0;
      channel_geometry[0].azimuth = 60;
      channel_geometry[0].is_lfe = 0;

      channel_geometry[1].elevation = 0;
      channel_geometry[1].azimuth = -60;
      channel_geometry[1].is_lfe = 0;

      channel_geometry[2].elevation = 0;
      channel_geometry[2].azimuth = 0;
      channel_geometry[2].is_lfe = 0;

      channel_geometry[3].elevation = -15;
      channel_geometry[3].azimuth = 45;
      channel_geometry[3].is_lfe = 1;

      channel_geometry[4].elevation = 0;
      channel_geometry[4].azimuth = 135;
      channel_geometry[4].is_lfe = 0;

      channel_geometry[5].elevation = 0;
      channel_geometry[5].azimuth = -135;
      channel_geometry[5].is_lfe = 0;

      channel_geometry[6].elevation = 0;
      channel_geometry[6].azimuth = 30;
      channel_geometry[6].is_lfe = 0;

      channel_geometry[7].elevation = 0;
      channel_geometry[7].azimuth = -30;
      channel_geometry[7].is_lfe = 0;

      channel_geometry[8].elevation = 0;
      channel_geometry[8].azimuth = 180;
      channel_geometry[8].is_lfe = 0;

      channel_geometry[9].elevation = -15;
      channel_geometry[9].azimuth = -45;
      channel_geometry[9].is_lfe = 1;

      channel_geometry[10].elevation = 0;
      channel_geometry[10].azimuth = 90;
      channel_geometry[10].is_lfe = 0;

      channel_geometry[11].elevation = 0;
      channel_geometry[11].azimuth = -90;
      channel_geometry[11].is_lfe = 0;

      channel_geometry[12].elevation = 35;
      channel_geometry[12].azimuth = 45;
      channel_geometry[12].is_lfe = 0;

      channel_geometry[13].elevation = 35;
      channel_geometry[13].azimuth = -45;
      channel_geometry[13].is_lfe = 0;

      channel_geometry[14].elevation = 35;
      channel_geometry[14].azimuth = 0;
      channel_geometry[14].is_lfe = 0;

      channel_geometry[15].elevation = 90;
      channel_geometry[15].azimuth = 0;
      channel_geometry[15].is_lfe = 0;

      channel_geometry[16].elevation = 35;
      channel_geometry[16].azimuth = 135;
      channel_geometry[16].is_lfe = 0;

      channel_geometry[17].elevation = 35;
      channel_geometry[17].azimuth = -135;
      channel_geometry[17].is_lfe = 0;

      channel_geometry[18].elevation = 35;
      channel_geometry[18].azimuth = 90;
      channel_geometry[18].is_lfe = 0;

      channel_geometry[19].elevation = 35;
      channel_geometry[19].azimuth = -90;
      channel_geometry[19].is_lfe = 0;

      channel_geometry[20].elevation = 35;
      channel_geometry[20].azimuth = 180;
      channel_geometry[20].is_lfe = 0;

      channel_geometry[21].elevation = -15;
      channel_geometry[21].azimuth = 0;
      channel_geometry[21].is_lfe = 0;

      channel_geometry[22].elevation = -15;
      channel_geometry[22].azimuth = 45;
      channel_geometry[22].is_lfe = 0;

      channel_geometry[23].elevation = -15;
      channel_geometry[23].azimuth = -45;
      channel_geometry[23].is_lfe = 0;
    }

    *ptr_num_channels = 22;
    *ptr_num_lfe = 2;
    break;

  default:
    *ptr_num_channels = -1;
    *ptr_num_lfe = -1;
    return IMPEGHE_CONFIG_NONFATAL_DMX_INVALID_CONFIG;
  }

  return IA_NO_ERROR;
}
