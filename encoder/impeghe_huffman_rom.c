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

const WORD32 impeghe_huffman_code_table[121][2] = {
    {18, 262120}, {18, 262118}, {18, 262119}, {18, 262117}, {19, 524277}, {19, 524273},
    {19, 524269}, {19, 524278}, {19, 524270}, {19, 524271}, {19, 524272}, {19, 524284},
    {19, 524285}, {19, 524287}, {19, 524286}, {19, 524279}, {19, 524280}, {19, 524283},
    {19, 524281}, {18, 262116}, {19, 524282}, {18, 262115}, {17, 131055}, {17, 131056},
    {16, 65525},  {17, 131054}, {16, 65522},  {16, 65523},  {16, 65524},  {16, 65521},
    {15, 32758},  {15, 32759},  {14, 16377},  {14, 16373},  {14, 16375},  {14, 16371},
    {14, 16374},  {14, 16370},  {13, 8183},   {13, 8181},   {12, 4089},   {12, 4087},
    {12, 4086},   {11, 2041},   {12, 4084},   {11, 2040},   {10, 1017},   {10, 1015},
    {10, 1013},   {9, 504},     {9, 503},     {8, 250},     {8, 248},     {8, 246},
    {7, 121},     {6, 58},      {6, 56},      {5, 26},      {4, 11},      {3, 4},
    {1, 0},       {4, 10},      {4, 12},      {5, 27},      {6, 57},      {6, 59},
    {7, 120},     {7, 122},     {8, 247},     {8, 249},     {9, 502},     {9, 505},
    {10, 1012},   {10, 1014},   {10, 1016},   {11, 2037},   {11, 2036},   {11, 2038},
    {11, 2039},   {12, 4085},   {12, 4088},   {13, 8180},   {13, 8182},   {13, 8184},
    {14, 16376},  {14, 16372},  {16, 65520},  {15, 32756},  {16, 65526},  {15, 32757},
    {18, 262114}, {19, 524249}, {19, 524250}, {19, 524251}, {19, 524252}, {19, 524253},
    {19, 524254}, {19, 524248}, {19, 524242}, {19, 524243}, {19, 524244}, {19, 524245},
    {19, 524246}, {19, 524274}, {19, 524255}, {19, 524263}, {19, 524264}, {19, 524265},
    {19, 524266}, {19, 524267}, {19, 524262}, {19, 524256}, {19, 524257}, {19, 524258},
    {19, 524259}, {19, 524260}, {19, 524261}, {19, 524247}, {19, 524268}, {19, 524276},
    {19, 524275},
};
