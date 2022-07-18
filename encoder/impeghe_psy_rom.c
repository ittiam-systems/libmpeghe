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

#include "impeghe_type_def.h"
#include "impeghe_cnst.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_psy_utils.h"

const WORD16 impeghe_sfb_96_1024[] = {4,   8,   12,  16,  20,  24,  28,  32,  36,  40,  44,
                                      48,  52,  56,  64,  72,  80,  88,  96,  108, 120, 132,
                                      144, 156, 172, 188, 212, 240, 276, 320, 384, 448, 512,
                                      576, 640, 704, 768, 832, 896, 960, 1024};

const WORD16 impeghe_sfb_96_128[] = {4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128};

const WORD16 impeghe_sfb_48_1024[] = {
    4,   8,   12,  16,  20,  24,  28,  32,  36,  40,  48,  56,  64,  72,  80,  88,  96,
    108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480,
    512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 1024};

const WORD16 impeghe_sfb_48_128[] = {4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128};

const WORD16 impeghe_sfb_32_1024[] = {
    4,   8,   12,  16,  20,  24,  28,  32,  36,  40,  48,  56,  64,  72,  80,  88,  96,
    108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480,
    512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024};

const WORD16 impeghe_sfb_24_1024[] = {4,   8,   12,  16,  20,  24,  28,  32,  36,  40,  44,  52,
                                      60,  68,  76,  84,  92,  100, 108, 116, 124, 136, 148, 160,
                                      172, 188, 204, 220, 240, 260, 284, 308, 336, 364, 396, 432,
                                      468, 508, 552, 600, 652, 704, 768, 832, 896, 960, 1024};

const WORD16 impeghe_sfb_24_128[] = {4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 64, 76, 92, 108, 128};

const WORD16 impeghe_sfb_16_1024[] = {8,   16,  24,  32,  40,  48,  56,  64,  72,  80,  88,
                                      100, 112, 124, 136, 148, 160, 172, 184, 196, 212, 228,
                                      244, 260, 280, 300, 320, 344, 368, 396, 424, 456, 492,
                                      532, 572, 616, 664, 716, 772, 832, 896, 960, 1024};

const WORD16 impeghe_sfb_16_128[] = {4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 60, 72, 88, 108, 128};

const WORD16 impeghe_sfb_8_1024[] = {12,  24,  36,  48,  60,  72,  84,  96,  108, 120,
                                     132, 144, 156, 172, 188, 204, 220, 236, 252, 268,
                                     288, 308, 328, 348, 372, 396, 420, 448, 476, 508,
                                     544, 580, 620, 664, 712, 764, 820, 880, 944, 1024};

const WORD16 impeghe_sfb_8_128[] = {4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 60, 72, 88, 108, 128};

ia_sfb_info_struct impeghe_sfb_info_1024[6] = {
    {16000, 43, 15, impeghe_sfb_16_1024, impeghe_sfb_16_128, {0}, {0}},
    {22050, 47, 15, impeghe_sfb_24_1024, impeghe_sfb_24_128, {0}, {0}},
    {24000, 47, 15, impeghe_sfb_24_1024, impeghe_sfb_24_128, {0}, {0}},
    {32000, 51, 14, impeghe_sfb_32_1024, impeghe_sfb_48_128, {0}, {0}},
    {44100, 49, 14, impeghe_sfb_48_1024, impeghe_sfb_48_128, {0}, {0}},
    {48000, 49, 14, impeghe_sfb_48_1024, impeghe_sfb_48_128, {0}, {0}}};

const ia_sfb_info_tables impeghe_sfb_info_tables[6] = {
    {16000,
     {

         8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  12, 12, 12, 12, 12, 12, 12, 12, 12, 16, 16,
         16, 16, 20, 20, 20, 24, 24, 28, 28, 32, 36, 40, 40, 44, 48, 52, 56, 60, 64, 64, 64},
     {4, 4, 4, 4, 4, 4, 4, 4, 8, 8, 12, 12, 16, 20, 20}},
    {22050,
     {4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,
      8,  8,  8,  8,  8,  12, 12, 12, 12, 16, 16, 16, 20, 20, 24, 24,
      28, 28, 32, 36, 36, 40, 44, 48, 52, 52, 64, 64, 64, 64, 64},
     {4, 4, 4, 4, 4, 4, 4, 8, 8, 8, 12, 12, 16, 16, 20}},

    {24000,
     {4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,
      8,  8,  8,  8,  8,  12, 12, 12, 12, 16, 16, 16, 20, 20, 24, 24,
      28, 28, 32, 36, 36, 40, 44, 48, 52, 52, 64, 64, 64, 64, 64},
     {4, 4, 4, 4, 4, 4, 4, 8, 8, 8, 12, 12, 16, 16, 20}},
    {32000,
     {4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,
      12, 12, 12, 12, 16, 16, 20, 20, 24, 24, 28, 28, 32, 32, 32, 32, 32,
      32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32},
     {4, 4, 4, 4, 4, 8, 8, 8, 12, 12, 12, 16, 16, 16}},

    {44100,
     {4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,
      12, 12, 12, 12, 16, 16, 20, 20, 24, 24, 28, 28, 32, 32, 32, 32, 32,
      32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 96},
     {4, 4, 4, 4, 4, 8, 8, 8, 12, 12, 12, 16, 16, 16}},

    {48000,
     {4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,
      12, 12, 12, 12, 16, 16, 20, 20, 24, 24, 28, 28, 32, 32, 32, 32, 32,
      32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 96},
     {4, 4, 4, 4, 4, 8, 8, 8, 12, 12, 12, 16, 16, 16}}};