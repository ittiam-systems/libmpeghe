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

#include <float.h>
#include <math.h>
#include "impeghe_hoa_struct.h"
#include "impeghe_hoa_common_functions.h"

/**
 *  impeghe_hoa_dyn_correction_process
 *
 *  \brief dynamic correction processing
 *
 *  \param [in,out] pstr_se_handle Spatial encoder handle
 *  \param [in] ch_idx Channel index
 *
 *  \return VOID
 */
VOID impeghe_hoa_dyn_correction_process(ia_spatial_enc_str *pstr_se_handle, UWORD32 ch_idx)
{
  ia_hoa_dec_frame_param_str *pstr_frm_prm = &(pstr_se_handle->frm_prm_hdl);
  ia_spatial_enc_dyn_correction_str *pstr_dyn_cor = &(pstr_se_handle->dyn_corrctn_hdl[ch_idx]);
  UWORD32 frm_sz = pstr_dyn_cor->frame_size;
  const pFlOAT64 ptr_in_pred_nxt_sam =
      pstr_se_handle->penult_smtd_pre_dom_frm + (ch_idx * frm_sz);
  pFlOAT64 ptr_out_sam =
      pstr_se_handle->trans_ch[pstr_se_handle->scd_lst_ip_idx] + (ch_idx * frm_sz);
  pWORD32 is_exception = &(pstr_frm_prm->is_exception[ch_idx]);
  pWORD32 exponent = &(pstr_frm_prm->exponents[ch_idx]);

  FLOAT64 max_abs_val_inp, max_abs_val_pred_nxt_ip;
  FLOAT64 max_abs_val_tot_ip, cor_max_abs_val;
  UWORD32 max_abs_idx_tot_ip, max_abs_idx_ip;
  UWORD32 max_abs_idx_pred_nxt_ip, max_range_first_index, max_range_second_index;
  ULOOPIDX i;
  WORD32 is_out_of_range;
  pFlOAT64 lst_gain = &(pstr_dyn_cor->last_gain);
  pWORD32 lst_exp_base2 = &(pstr_dyn_cor->last_exp_base2);
  pWORD32 prev_exp_base2 = &(pstr_dyn_cor->prev_exp_base2);

  impeghe_hoa_compute_max_abs_val(ptr_out_sam, frm_sz, &max_abs_val_inp, &max_abs_idx_ip);

  impeghe_hoa_compute_max_abs_val(ptr_in_pred_nxt_sam, frm_sz, &max_abs_val_pred_nxt_ip,
                                  &max_abs_idx_pred_nxt_ip);

  if (max_abs_val_pred_nxt_ip > max_abs_val_inp)
  {
    max_abs_val_tot_ip = max_abs_val_pred_nxt_ip;
    max_abs_idx_tot_ip = frm_sz - 1;
  }
  else
  {
    max_abs_val_tot_ip = max_abs_val_inp;
    max_abs_idx_tot_ip = max_abs_idx_ip;
  }

  for (i = 0; i < frm_sz; i++)
  {
    ptr_out_sam[i] *= *lst_gain;
  }

  max_range_first_index = frm_sz - 1;
  max_range_second_index = max_abs_idx_tot_ip;
  is_out_of_range = 0;

  for (i = 0; i < frm_sz; i++)
  {
    if ((ptr_out_sam[i] > (((FLOAT64)(1.0) - pstr_dyn_cor->smallest_delta))) ||
        (ptr_out_sam[i] < (FLOAT64)(-1.0)))
    {
      max_range_first_index = i;
      is_out_of_range = 1;
      break;
    }
  }

  for (i = 0; i < frm_sz; i++)
  {
    if ((*lst_gain * ptr_in_pred_nxt_sam[i] >
         (((FLOAT64)(1.0) - pstr_dyn_cor->smallest_delta))) ||
        (*lst_gain * ptr_in_pred_nxt_sam[i] < (FLOAT64)(-1.0)))
    {
      is_out_of_range = 1;
    }
  }

  if (!is_out_of_range)
  {
    *is_exception = 0;

    if (*lst_exp_base2 == (WORD32)(pstr_dyn_cor->max_amp_exp) || max_abs_val_tot_ip < DBL_MIN)
    {
      *exponent = 0;
      *prev_exp_base2 = *lst_exp_base2;
      return;
    }
    else
    {
      FLOAT64 max_abs_val_after_gain_increase;

      FLOAT64 smoothed_max_abs_val_total_input =
          pstr_dyn_cor->max_val_absorption * max_abs_val_tot_ip +
          ((FLOAT64)(1.0) - pstr_dyn_cor->max_val_absorption) * pstr_dyn_cor->last_max_abs_value;

      pstr_dyn_cor->last_max_abs_value = smoothed_max_abs_val_total_input;

      max_abs_val_after_gain_increase =
          (MAX(smoothed_max_abs_val_total_input, max_abs_val_tot_ip)) * (*lst_gain) *
          (FLOAT64)(2.0);

      if (max_abs_val_after_gain_increase < ((FLOAT64)(1.0) - pstr_dyn_cor->smallest_delta))
      {
        *exponent = -1;
      }
      else
      {
        *exponent = 0;
      }
    }
  }
  else
  {
    pstr_dyn_cor->last_max_abs_value = max_abs_val_tot_ip;
  }

  cor_max_abs_val = pstr_dyn_cor->last_max_abs_value * *lst_gain;

  if (is_out_of_range)
  {
    if (max_range_second_index == 0)
    {
      *is_exception = 1;
    }
    else
    {
      WORD32 is_success = 0;

      FLOAT64 req_exp = -log(cor_max_abs_val + pstr_dyn_cor->smallest_delta) /
                        log(pstr_dyn_cor->win_func[max_range_second_index]);

      *exponent = (WORD32)(ceil(req_exp));

      while (!is_success)
      {
        is_success = 1;

        for (i = max_range_first_index; i <= max_range_second_index; i++)
        {
          FLOAT64 comp_sig_val =
              ptr_out_sam[i] * (FLOAT64)(pow((FLOAT64)(pstr_dyn_cor->win_func[i]), *exponent));

          if (comp_sig_val > ((FLOAT64)(1.0) - pstr_dyn_cor->smallest_delta) ||
              comp_sig_val < (FLOAT64)(-1.0))
          {
            is_success = 0;
            break;
          }
        }

        if (!is_success)
        {
          *exponent += 1;
        }

        if (*exponent > (WORD32)(pstr_dyn_cor->max_attn_exp))
        {
          *is_exception = 1;
          break;
        }
      }

      if (is_success)
      {
        *is_exception = 0;
      }
    }
  }

  if (!*is_exception)
  {
    for (i = 0; i < frm_sz; i++)
    {
      ptr_out_sam[i] *= (FLOAT64)(pow((FLOAT64)(pstr_dyn_cor->win_func[i]), *exponent));
    }

    *lst_gain *= pow((FLOAT64)(pstr_dyn_cor->win_func[frm_sz - 1]), *exponent);
  }
  else
  {
    FLOAT64 req_exp =
        (FLOAT64)(-log((FLOAT64)(cor_max_abs_val + pstr_dyn_cor->smallest_delta)) / log(2.0));
    FLOAT64 gain;

    *exponent = (WORD32)(-floor(req_exp));

    gain = pow((FLOAT64)(2.0), -(*exponent));
    for (i = 0; i < frm_sz; i++)
    {
      ptr_out_sam[i] *= gain;
    }

    *lst_gain *= pow((FLOAT64)(2.0), -(*exponent));
  }

  *prev_exp_base2 = *lst_exp_base2;
  *lst_exp_base2 -= *exponent;
  return;
}
