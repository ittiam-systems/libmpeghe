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

#ifndef IMPEGHE_TNS_USAC_H
#define IMPEGHE_TNS_USAC_H

#define TNS_MAX_ORDER 31
#define DEF_TNS_GAIN_THRESH 1.41
#define DEF_TNS_COEFF_THRESH 0.1
#define DEF_TNS_COEFF_RES 4
#define DEF_TNS_RES_OFFSET 3

#ifndef PI
#define PI 3.14159265358979323846
#endif

/************************/ /*
                            * Structure definitions
                            **************************/

typedef struct
{
  WORD32 order;                        /**< Filter order */
  WORD32 direction;                    /**< Filtering direction */
  WORD32 coef_compress;                /**< Are coeffs compressed? */
  WORD32 length;                       /**< Length, in bands */
  FLOAT64 a_coeffs[TNS_MAX_ORDER + 1]; /**< AR Coefficients */
  FLOAT64 k_coeffs[TNS_MAX_ORDER + 1]; /**< Reflection Coefficients */
  WORD32 index[TNS_MAX_ORDER + 1];     /**< Coefficient indices */
} ia_tns_filter_data;

typedef struct
{
  WORD32 n_filt;                    /**< number of filters */
  WORD32 coef_res;                  /**< Coefficient resolution */
  ia_tns_filter_data tns_filter[3]; /**< TNS filters */
  FLOAT64 tns_pred_gain;
  WORD32 tns_active;
} ia_tns_window_data;

typedef struct
{
  WORD32 tns_data_present;
  WORD32 tns_min_band_number_long;
  WORD32 tns_min_band_number_short;
  WORD32 tns_max_bands_long;
  WORD32 tns_max_bands_short;
  WORD32 tns_max_order_long;
  WORD32 tns_max_order_short;
  WORD32 lpc_start_band_long;
  WORD32 lpc_start_band_short;
  WORD32 lpc_stop_band_long;
  WORD32 lpc_stop_band_short;
  ia_tns_window_data window_data[MAX_SHORT_WINDOWS]; /**< TNS data per window */
  WORD32 *sfb_offset_table_short;                    /**< Scalefactor band offset table */
  WORD32 *sfb_offset_table_short_tcx;                /**< Scalefactor band offset table */
  WORD32 *sfb_offset_table_long;                     /**< Scalefactor band offset table */
  WORD32 max_sfb_short;                              /**< max_sfb */
  WORD32 max_sfb_long;                               /**< max_sfb */
  FLOAT32 threshold;
  FLOAT32 tns_time_res_short;
  FLOAT32 tns_time_res_long;
  FLOAT64 win_short[8];
  FLOAT64 win_long[16];
  WORD32 block_type;      /**< block type */
  WORD32 number_of_bands; /**< number of bands per window */
  FLOAT64 *spec;          /**< Spectral data array */
} ia_tns_info;

IA_ERRORCODE impeghe_tns_init(WORD32 sampling_rate, WORD32 bit_rate, ia_tns_info *pstr_tns_info,
                              WORD32 num_channels);
VOID impeghe_tns_encode(ia_tns_info *pstr_tns_info_ch2, ia_tns_info *pstr_tns_info,
                        FLOAT32 *ptr_sfb_energy, WORD32 w, WORD32 i_ch, WORD32 low_pass_line,
                        ia_igf_config_struct *pstr_igf_config, FLOAT64 *ptr_scratch_tns_filter,
                        WORD32 core_mode, FLOAT64 *ptr_tns_scratch);

#endif /* IMPEGHE_TNS_USAC_H */