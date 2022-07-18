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

#ifndef IMPEGHE_WINDOWING_H
#define IMPEGHE_WINDOWING_H

IA_ERRORCODE impeghe_calc_window(FLOAT64 **pptr_win, WORD32 win_sz, WORD32 win_sel);
VOID impeghe_windowing_long(FLOAT64 *ptr_out_buf, FLOAT64 *ptr_win_long, FLOAT64 *ptr_win_buf,
                            FLOAT64 *ptr_in_data, WORD32 n_long);
VOID impeghe_windowing_long_start(FLOAT64 *ptr_out_buf, FLOAT64 *ptr_win_long,
                                  FLOAT64 *ptr_win_buf, FLOAT64 *ptr_in_data, WORD32 n_long,
                                  WORD32 nflat_ls, FLOAT64 *ptr_win_med, WORD32 win_sz);
VOID impeghe_windowing_long_stop(FLOAT64 *ptr_out_buf, FLOAT64 *ptr_win_long,
                                 FLOAT64 *ptr_win_buf, FLOAT64 *ptr_in_data, WORD32 n_long,
                                 WORD32 nflat_ls, FLOAT64 *ptr_win_med, WORD32 win_sz);
VOID impeghe_windowing_stop_start(FLOAT64 *ptr_out_buf, FLOAT64 *ptr_win_buf,
                                  FLOAT64 *ptr_win_med, WORD32 win_sz, WORD32 n_long);
WORD32 impeghe_fd_mdct(ia_usac_data_struct *pstr_usac_data,
                       ia_usac_encoder_config_struct *pstr_usac_config, WORD32 ch_idx);

#endif /* IMPEGHE_WINDOWING_H */
