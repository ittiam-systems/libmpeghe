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
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_rom.h"

/**
 *  impeghe_fmult
 *
 *  \brief Multiply two FLOAT32 number
 *
 *  \param [in] a
 *  \param [in] b
 *
 *  \return FLOAT32 multiplication result
 */
static FLOAT32 impeghe_fmult(FLOAT32 a, FLOAT32 b) { return (a * b); }

/**
 *  impeghe_fadd
 *
 *  \brief Add two FLOAT32 number
 *
 *  \param [in] a
 *  \param [in] b
 *
 *  \return FLOAT32 addition result
 */
static FLOAT32 impeghe_fadd(FLOAT32 a, FLOAT32 b) { return (a + b); }

/**
 *  impeghe_init_block_switching
 *
 *  \brief Block switching initialization
 *
 *  \param [out]	pstr_blk_switch_ctrl	Pointer to block switch control structure
 *  \param [in]		bit_rate		Bit rate
 *  \param [in]		num_chans		Number of channels
 *
 *  \return VOID
 *
 */
VOID impeghe_init_block_switching(ia_block_switch_ctrl *pstr_blk_switch_ctrl,
                                  const WORD32 bit_rate, const WORD32 num_chans)
{
  LOOPIDX i;

  if ((num_chans == 1 && bit_rate > 24000) || (num_chans > 1 && bit_rate / num_chans > 16000))
  {
    pstr_blk_switch_ctrl->inv_attack_ratio = INV_ATTACK_RATIO_HIGH_BR;
  }
  else
  {
    pstr_blk_switch_ctrl->inv_attack_ratio = INV_ATTACK_RATIO_LOW_BR;
  }

  for (i = 0; i < BLK_SWITCH_FILT_LEN; i++)
  {
    pstr_blk_switch_ctrl->iir_states[i] = 0;
  }

  /* Clear Filtered Window Energies */
  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {
    pstr_blk_switch_ctrl->win_energy_filt[0][i] = 0;
    pstr_blk_switch_ctrl->win_energy_filt[1][i] = 0;
    pstr_blk_switch_ctrl->win_energy[0][i] = 0;
    pstr_blk_switch_ctrl->win_energy[1][i] = 0;
  }
  pstr_blk_switch_ctrl->acc_win_energy = 0;

  pstr_blk_switch_ctrl->window_seq = ONLY_LONG_SEQUENCE;
  pstr_blk_switch_ctrl->next_win_seq = ONLY_LONG_SEQUENCE;

  pstr_blk_switch_ctrl->attack = 0;
  pstr_blk_switch_ctrl->lastattack = 0;
  pstr_blk_switch_ctrl->attack_idx = 0;
  pstr_blk_switch_ctrl->last_attack_idx = 0;

  return;
}

/**
 *  impeghe_srch_max_with_idx
 *
 *  \brief Searches maximum value in array and return index and value
 *
 *  \param [out]	index	Pointer to index of maximum value found in array
 *  \param [in]		ptr_in		Pointer to input array
 *
 *  \return FLOAT32 Maximum value in array
 *
 */
static FLOAT32 impeghe_srch_max_with_idx(const FLOAT32 *ptr_in, WORD32 *index)
{
  FLOAT32 max;
  LOOPIDX i;
  WORD32 idx;

  max = 0;
  idx = 0;

  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {
    if (ptr_in[i + 1] > max)
    {
      max = ptr_in[i + 1];
      idx = i;
    }
  }
  *index = idx;

  return max;
}

/**
 *  impeghe_blk_switch_iir_filt
 *
 *  \brief Block switching IIR filter
 *
 *  \param [in]		ptr_in	Pointer to input time signal
 *  \param [in]		ptr_iir_coeff		Pointer to IIR coefficients
 *  \param [in,out]	ptr_iir_states		Pointer	to IIR filter states
 *  \param [in]		w				Block switch window
 *  \param [out]	ptr_energy_accu		Pointer to accumulated energy
 *
 *  \return VOID
 *
 */
static VOID impeghe_blk_switch_iir_filt(const FLOAT32 *ptr_in, const FLOAT32 *ptr_iir_coeff,
                                        const WORD32 w, FLOAT32 *ptr_iir_states,
                                        FLOAT32 *ptr_energy_accu)
{
  FLOAT32 accu1;

  LOOPIDX i;

  FLOAT32 accu_unfilt = 0.0f;
  FLOAT32 accu_filt = 0.0f;
  FLOAT32 accu2, temp2, temp1;

  FLOAT32 state0 = ptr_iir_states[0];
  FLOAT32 state1 = ptr_iir_states[1];

  FLOAT32 coeff0 = ptr_iir_coeff[0];
  FLOAT32 coeff1 = ptr_iir_coeff[1];

  const FLOAT32 *p_time_signal = &ptr_in[(FRAME_LEN_SHORT * w)];

  for (i = 0; i < FRAME_LEN_SHORT; i++)
  {
    accu2 = impeghe_fmult(state0, coeff1);
    accu1 = impeghe_fmult(state1, coeff0);
    accu1 += accu2;

    state0 = p_time_signal[i];
    state1 = impeghe_fmult(state0, coeff1);
    state1 = (state1 - accu1);

    temp1 = impeghe_fmult(state0, state0);
    temp2 = impeghe_fmult(state1, state1);

    accu_unfilt = impeghe_fadd(accu_unfilt, temp1);
    accu_filt = impeghe_fadd(accu_filt, temp2);
  }

  ptr_energy_accu[0] = accu_unfilt;
  ptr_energy_accu[1] = accu_filt;

  ptr_iir_states[0] = state0;
  ptr_iir_states[1] = state1;

  return;
}

/**
 *  impeghe_calc_window_energy
 *
 *  \brief Calculates window energy
 *
 *  \param [in,out]	ptr_blk_switch_ctrl	Pointer to block switch control structure
 *  \param [in]		ptr_in		Pointer to input time signal
 *  \param [out]	max					Pointer to max window energy
 *
 *  \return VOID
 *
 */
static VOID impeghe_calc_window_energy(ia_block_switch_ctrl *ptr_blk_switch_ctrl,
                                       const FLOAT32 *ptr_in, FLOAT32 *max)
{
  LOOPIDX i;

  FLOAT32 energy_accu[2];
  *max = 0.0f;

  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {
    impeghe_blk_switch_iir_filt(ptr_in, impeghe_iir_hipass_coeffs, i,
                                ptr_blk_switch_ctrl->iir_states, &energy_accu[0]);

    ptr_blk_switch_ctrl->win_energy[1][i] = energy_accu[0];
    ptr_blk_switch_ctrl->win_energy_filt[1][i] = energy_accu[1];

    if (ptr_blk_switch_ctrl->win_energy_filt[1][i] > *max)
      *max = ptr_blk_switch_ctrl->win_energy_filt[1][i];
  }
  return;
}

/**
 *  impeghe_block_switching
 *
 *  \brief Performs block switching
 *
 *  \param [in,out]	ptr_blk_switch_ctrl	Pointer to block switch control structure
 *  \param [in]		ptr_in		Pointer to input time signal
 *
 *  \return VOID
 *
 */
VOID impeghe_block_switching(ia_block_switch_ctrl *ptr_blk_switch_ctrl, const FLOAT32 *ptr_in)
{
  LOOPIDX i;

  FLOAT32 temp1, temp2;
  FLOAT32 max;
  FLOAT32 energy, energy_max;

  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {
    ptr_blk_switch_ctrl->group_len[i] = 0;
  }

  ptr_blk_switch_ctrl->max_win_energy =
      impeghe_srch_max_with_idx(&ptr_blk_switch_ctrl->win_energy[0][MAX_SHORT_WINDOWS - 1],
                                &ptr_blk_switch_ctrl->attack_idx);

  ptr_blk_switch_ctrl->attack_idx = ptr_blk_switch_ctrl->last_attack_idx;
  ptr_blk_switch_ctrl->tot_grps_cnt = MAX_NUM_WIN_GROUPS;

  for (i = 0; i < MAX_NUM_WIN_GROUPS; i++)
  {
    ptr_blk_switch_ctrl->group_len[i] =
        impeghe_suggested_grouping_table[ptr_blk_switch_ctrl->attack_idx][i];
  }

  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {
    ptr_blk_switch_ctrl->win_energy[0][i] = ptr_blk_switch_ctrl->win_energy[1][i];
    ptr_blk_switch_ctrl->win_energy_filt[0][i] = ptr_blk_switch_ctrl->win_energy_filt[1][i];
  }

  impeghe_calc_window_energy(ptr_blk_switch_ctrl, ptr_in, &max);

  ptr_blk_switch_ctrl->attack = FALSE;

  energy_max = 0.0f;

  energy = ptr_blk_switch_ctrl->win_energy_filt[0][MAX_SHORT_WINDOWS - 1];

  for (i = 0; i < MAX_SHORT_WINDOWS; i++)
  {

    temp1 = impeghe_fmult(ONE_MINUS_ACC_WINDOW_NRG_FAC, ptr_blk_switch_ctrl->acc_win_energy);
    temp2 = impeghe_fmult(ACC_WINDOW_NRG_FAC, energy);
    ptr_blk_switch_ctrl->acc_win_energy = impeghe_fadd(temp1, temp2);

    temp1 = impeghe_fmult(ptr_blk_switch_ctrl->win_energy_filt[1][i],
                          ptr_blk_switch_ctrl->inv_attack_ratio);
    if (temp1 > ptr_blk_switch_ctrl->acc_win_energy)
    {
      ptr_blk_switch_ctrl->attack = TRUE;
      ptr_blk_switch_ctrl->last_attack_idx = i;
    }

    energy = ptr_blk_switch_ctrl->win_energy_filt[1][i];
    if (energy_max < energy)
      energy_max = energy;
  }

  if (energy_max < MIN_ATTACK_NRG)
  {
    ptr_blk_switch_ctrl->attack = FALSE;
  }

  if ((!ptr_blk_switch_ctrl->attack) && (ptr_blk_switch_ctrl->lastattack))
  {
    if (ptr_blk_switch_ctrl->attack_idx == MAX_SHORT_WINDOWS - 1)
    {
      ptr_blk_switch_ctrl->attack = TRUE;
    }
    ptr_blk_switch_ctrl->lastattack = FALSE;
  }
  else
  {
    ptr_blk_switch_ctrl->lastattack = ptr_blk_switch_ctrl->attack;
  }
  ptr_blk_switch_ctrl->window_seq = ptr_blk_switch_ctrl->next_win_seq;

  if (ptr_blk_switch_ctrl->attack)
  {
    ptr_blk_switch_ctrl->next_win_seq = EIGHT_SHORT_SEQUENCE;
  }
  else
  {
    ptr_blk_switch_ctrl->next_win_seq = ONLY_LONG_SEQUENCE;
  }
  if (ptr_blk_switch_ctrl->next_win_seq == EIGHT_SHORT_SEQUENCE)
  {
    if (ptr_blk_switch_ctrl->window_seq == ONLY_LONG_SEQUENCE)
    {
      ptr_blk_switch_ctrl->window_seq = LONG_START_SEQUENCE;
    }

    if (ptr_blk_switch_ctrl->window_seq == LONG_STOP_SEQUENCE)
    {
      ptr_blk_switch_ctrl->window_seq = EIGHT_SHORT_SEQUENCE;
      ptr_blk_switch_ctrl->tot_grps_cnt = 3;
      ptr_blk_switch_ctrl->group_len[0] = 3;
      ptr_blk_switch_ctrl->group_len[1] = 3;
      ptr_blk_switch_ctrl->group_len[2] = 2;
    }
  }

  if (ptr_blk_switch_ctrl->next_win_seq == ONLY_LONG_SEQUENCE)
  {
    if (ptr_blk_switch_ctrl->window_seq == EIGHT_SHORT_SEQUENCE)
    {
      ptr_blk_switch_ctrl->next_win_seq = LONG_STOP_SEQUENCE;
    }
  }
  return;
}

/**
 *  impeghe_sync_block_switching
 *
 *  \brief Synchronizes Left Right block switching
 *
 *  \param [in,out]	ptr_blk_switch_left_ctrl	Pointer to block switch control structure
 * -
 * Left
 *  \param [in,out]	ptr_blk_switch_right_ctrl	Pointer to block switch control structure
 * -
 * Right
 *
 *  \return VOID
 *
 */
VOID impeghe_sync_block_switching(ia_block_switch_ctrl *ptr_blk_switch_left_ctrl,
                                  ia_block_switch_ctrl *ptr_blk_switch_right_ctrl)
{
  LOOPIDX i;
  WORD32 patch_type = ONLY_LONG_SEQUENCE;

  patch_type = impeghe_synchronized_block_types[patch_type][ptr_blk_switch_left_ctrl->window_seq];
  patch_type =
      impeghe_synchronized_block_types[patch_type][ptr_blk_switch_right_ctrl->window_seq];

  ptr_blk_switch_left_ctrl->window_seq = patch_type;
  ptr_blk_switch_right_ctrl->window_seq = patch_type;

  if (patch_type != EIGHT_SHORT_SEQUENCE)
  { /* Long Blocks */
    ptr_blk_switch_left_ctrl->tot_grps_cnt = 1;
    ptr_blk_switch_right_ctrl->tot_grps_cnt = 1;
    ptr_blk_switch_left_ctrl->group_len[0] = 1;
    ptr_blk_switch_right_ctrl->group_len[0] = 1;

    for (i = 1; i < MAX_SHORT_WINDOWS; i++)
    {
      ptr_blk_switch_left_ctrl->group_len[i] = 0;
      ptr_blk_switch_right_ctrl->group_len[i] = 0;
    }
  }
  else
  { /* Short Blocks */
    if (ptr_blk_switch_left_ctrl->max_win_energy > ptr_blk_switch_right_ctrl->max_win_energy)
    {
      ptr_blk_switch_right_ctrl->tot_grps_cnt = ptr_blk_switch_left_ctrl->tot_grps_cnt;
      for (i = 0; i < ptr_blk_switch_right_ctrl->tot_grps_cnt; i++)
      {
        ptr_blk_switch_right_ctrl->group_len[i] = ptr_blk_switch_left_ctrl->group_len[i];
      }
    }
    else
    {
      ptr_blk_switch_left_ctrl->tot_grps_cnt = ptr_blk_switch_right_ctrl->tot_grps_cnt;
      for (i = 0; i < ptr_blk_switch_left_ctrl->tot_grps_cnt; i++)
      {
        ptr_blk_switch_left_ctrl->group_len[i] = ptr_blk_switch_right_ctrl->group_len[i];
      }
    }
  }

  return;
}