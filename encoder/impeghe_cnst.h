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

#ifndef IMPEGHE_CNST_H
#define IMPEGHE_CNST_H

#define CORE_MODE_FD (0)
#define CORE_MODE_TD (1)
#define USAC_SWITCHED (0)
#define USAC_ONLY_FD (1)
#define USAC_ONLY_TD (2)

#define ONLY_LONG_SEQUENCE (0)
#define LONG_START_SEQUENCE (1)
#define EIGHT_SHORT_SEQUENCE (2)
#define LONG_STOP_SEQUENCE (3)
#define STOP_START_SEQUENCE (4)
#define NSFB_SHORT (16)
#define MAX_SHORT_WINDOWS (8)
#define MAX_NUM_SFB_LONG 51
#define MAX_NUM_SFB_SHORT 15
#define FRAME_LEN_LONG 1024
#define FRAME_LEN_SHORT (FRAME_LEN_LONG / MAX_SHORT_WINDOWS)
#define MAX_CHANNEL_BITS 6144
#define MAX_TIME_CHANNELS (56)
#define MAX_SF_BANDS ((NSFB_SHORT + 1) * MAX_SHORT_WINDOWS)
#define MAX_SHIFT_LEN_LONG (4096)
#define TD_BUFFER_OFFSET (448 + 64)
#define MAX_EXTENSION_PAYLOADS MAX_TIME_CHANNELS
#define MAX_CH_TYPE_SIG_GROUP (MAX_TIME_CHANNELS / 2)
#define MAX_EXTENSION_PAYLOAD_LEN ((MAX_CHANNEL_BITS / 8) * MAX_TIME_CHANNELS)

#define WIN_SEL_0 0
#define WIN_SEL_1 1

#define LEN_WIN_PLUS (512)

#define ORDER (16)
#define ORDER_BY_2 (8)

#define LEN_FRAME (256)
#define NUM_FRAMES (4)
#define MAX_NUM_SUBFR (4)
#define LEN_SUBFR (64)

#define FAC_LENGTH (LEN_FRAME / 2)
#define BPF_DELAY (1)
#define NUM_SUBFR_SUPERFRAME (NUM_FRAMES * MAX_NUM_SUBFR)
#define FDNS_RESOLUTION (64)

#define LEN_NEXT_HIGH_RATE (288)
#define LEN_LPC0 (256)
#define LEN_LP_WINDOW (448)
#define LEN_LP_WINDOW_HIGH_RATE (512)

#define SR_MIN (14700)
#define SR_MAX (32000)

#define FSCALE_DENOM (12800)
#define FAC_FSCALE_MAX (24000)

#define LEN_TOTAL_HIGH_RATE (ORDER + FRAME_LEN_LONG + LEN_NEXT_HIGH_RATE)

#define TMIN (34)
#define TFR2 (128)
#define TFR1 (160)
#define TMAX (231)

#define T_UP_SAMP (4)
#define INTER_LP_FIL_ORDER (16)
#define INTER_LP_FIL_LEN (T_UP_SAMP * INTER_LP_FIL_ORDER + 1)

/* upto 410 for 24k sampling rate */
#define MAX_PITCH                                                                                \
  (TMAX + (6 * ((((FAC_FSCALE_MAX * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN)))

/* upto 536 for 32k sampling rate */
#define MAX_PITCH1 (TMAX + (6 * ((((32000 * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN)))

#define LEN_INTERPOL (16 + 1)
#define OPL_DECIM (2)
#define PREEMPH_FILT_FAC (0.68f)
#define IGAMMA1 (0.92f)
#define TILT_FAC (0.68f)
#define PIT_SHARP (0.85f)
#define TILT_CODE (0.3f)
#define L_MEANBUF (3)

/* AMR_WB+ mode relative to AMR-WB core */
#define ACELP_CORE_MODE_9k6 (0)
#define ACELP_CORE_MODE_11k2 (1)
#define ACELP_CORE_MODE_12k8 (2)
#define ACELP_CORE_MODE_14k4 (3)
#define ACELP_CORE_MODE_16k (4)
#define ACELP_CORE_MODE_18k4 (5)
#define ACELP_CORE_MODE_8k0 (6)
#define ACELP_CORE_MODE_8k8 (7)

#define ACELP_NUM_BITS_12 (12)
#define ACELP_NUM_BITS_16 (16)
#define ACELP_NUM_BITS_20 (20)
#define ACELP_NUM_BITS_28 (28)
#define ACELP_NUM_BITS_36 (36)
#define ACELP_NUM_BITS_44 (44)
#define ACELP_NUM_BITS_52 (52)
#define ACELP_NUM_BITS_64 (64)
#define ACELP_NUM_BITS_72 (72)
#define ACELP_NUM_BITS_88 (88)

#define NUM_ACELP_CORE_MODES (8)
#define NBITS_MAX (48 * 80 + 46)

#define NBITS_MODE (4 * 2)
#define NBITS_LPC (46)

#define NUM_RE8_PRM (FRAME_LEN_LONG + (FRAME_LEN_LONG / 8))

#define NUM_TCX80_PRM (FAC_LENGTH + 2 + NUM_RE8_PRM)
#define NUM_TCX40_PRM (FAC_LENGTH + 2 + (NUM_RE8_PRM / 2))
#define NUM_TCX20_PRM (FAC_LENGTH + 2 + (320 + 320 / 8))

#define NUM_LPC_PRM (256)
#define MAX_NUM_TCX_PRM_PER_DIV (NUM_TCX20_PRM)

#define L_OLD_SPEECH_HIGH_RATE LEN_TOTAL_HIGH_RATE - FRAME_LEN_LONG

#define HP_ORDER (3)
#define LEN_INTERPOL1 (4)
#define LEN_INTERPOL2 (16)

#define MASK 0x0001
#define NUM_OPEN_LOOP_LAGS (5)
#define OPEN_LOOP_LAG_MEDIAN (3)
#define DECIM2_FIR_FILT_MEM_SIZE (3)

#define NUM_QUANTIZATION_LEVEL (128)
#define LEV_DUR_MAX_ORDER (24)
#define PI_BY_6400 (PI / 6400.0)
#define LEN_FRAME_16K 320
#define ORDER_LP_FILT_16K (20)
#define LSP_2_LSF_SCALE (6400.0 / PI)
#define LSF_2_LSP_SCALE (PI / 6400.0)
#define FREQ_MAX (6400.0f)
#define FREQ_DIV (400.0f)
#define CHEBYSHEV_NUM_ITER (4)
#define CHEBYSHEV_NUM_POINTS (100)
#define LSF_GAP (50.0f)

#define MAX_NUM_PULSES (24)
#define NPMAXPT ((MAX_NUM_PULSES + 4 - 1) / 4)
#define MODE_23k (7)
#define ACELP_GAIN_TBL_OFFSET (64)
#define ACELP_RANGE_GAIN_PT_IDX_SEARCH (NUM_QUANTIZATION_LEVEL - ACELP_GAIN_TBL_OFFSET)
#define ACELP_SEARCH_RANGE_QUANTIZER_IDX (128)

#define MAX_FLT_VAL (3.402823466e+38F)
#define MIN_FLT_VAL (1.175494351e-38F)
#define MIN_SHRT_VAL (-32768)
#define MAX_SHRT_VAL (32767)

/*LTPF*/
#define R12K8_MEM_OUT_LEN 24
#define MAX_PITCH_12K8 228
#define LTPF_MEMIN_LEN (MAX_PITCH_12K8 + 4)
#define KMAX_MAX 414
#define LAG_MIN 64  // if 48k is max sr-- corresponding pitch_min/2
#define LAG_MAX 408 // if 48k is max sr-- corresponding pitch_max/2
#define LEN_CORR_R LAG_MAX - LAG_MIN + 1
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define CODE_BOOK_ALPHA_LAV 121
#define CODE_BOOK_BETA_LAV 65
#endif /* IMPEGHE_CNST_H */
