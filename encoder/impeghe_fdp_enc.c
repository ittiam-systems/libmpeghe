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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_cnst.h"

#include "impeghe_fd_quant.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_drc_common.h"
#include "impeghe_drc_uni_drc.h"
#include "impeghe_drc_api.h"
#include "impeghe_drc_uni_drc_eq.h"
#include "impeghe_drc_uni_drc_filter_bank.h"
#include "impeghe_drc_gain_enc.h"
#include "impeghe_drc_struct_def.h"
#include "impeghe_dmx_cicp2geometry.h"
#include "impeghe_dmx_matrix_common.h"
#include "impeghe_memory_standards.h"
#include "impeghe_mae_write.h"
#include "impeghe_config.h"
#include "impeghe_fft.h"
#include "impeghe_block_switch_const.h"
#include "impeghe_block_switch_struct_def.h"
#include "impeghe_oam_enc_struct_def.h"
#include "impeghe_signal_classifier.h"
#include "impeghe_igf_enc.h"
#include "impeghe_tns_usac.h"
#include "impeghe_psy_mod.h"
#include "impeghe_ms.h"
#include "impeghe_fd_qc_util.h"
#include "impeghe_enc_mct.h"
#include "impeghe_stereo_lpd_defines.h"
#include "impeghe_stereo_lpd.h"
#include "impeghe_tbe_defines.h"
#include "impeghe_tbe_enc.h"
#include "impeghe_resampler.h"
#include "impeghe_main.h"

#define ia_abs_int abs
#define ia_max_int(a, b) (max(a, b))
#define ia_min_int(a, b) (min(a, b))
#define ia_mul_flt(a, b) (a * b)
#define ia_mac_flt(x, a, b) (x + a * b)

#define FDP_NUM_BANDS 7
#define FDP_NUM_BINS_IN_BAND 20
#define FDP_MAX_PRED_BW 132

#define QUANT_SPEC_MAX 31775
#define QUANT_SPEC_MIN -31775
#define UNCODED_LINES 172

const UWORD16 fdp_exp[182] =
    {/* 64 * (0:181) .^ (4/3) */
     0,     64,    161,   277,   406,   547,   698,   857,   1024,  1198,  1379,  1566,  1758,
     1956,  2159,  2368,  2580,  2798,  3019,  3245,  3474,  3708,  3945,  4186,  4431,  4678,
     4930,  5184,  5442,  5702,  5966,  6232,  6502,  6774,  7049,  7327,  7608,  7891,  8176,
     8464,  8755,  9048,  9344,  9641,  9941,  10244, 10548, 10855, 11164, 11476, 11789, 12104,
     12422, 12741, 13063, 13386, 13712, 14039, 14369, 14700, 15033, 15368, 15705, 16044, 16384,
     16726, 17070, 17416, 17763, 18113, 18463, 18816, 19170, 19526, 19883, 20242, 20603, 20965,
     21329, 21694, 22061, 22430, 22800, 23171, 23544, 23919, 24295, 24672, 25051, 25431, 25813,
     26196, 26581, 26966, 27354, 27742, 28132, 28524, 28917, 29311, 29706, 30103, 30501, 30900,
     31301, 31703, 32106, 32511, 32916, 33323, 33732, 34141, 34552, 34964, 35377, 35791, 36207,
     36624, 37042, 37461, 37881, 38303, 38725, 39149, 39574, 40000, 40427, 40856, 41285, 41716,
     42147, 42580, 43014, 43449, 43885, 44323, 44761, 45200, 45641, 46082, 46525, 46968, 47413,
     47859, 48306, 48753, 49202, 49652, 50103, 50555, 51008, 51462, 51916, 52372, 52829, 53287,
     53746, 54206, 54667, 55128, 55591, 56055, 56520, 56985, 57452, 57920, 58388, 58858, 59328,
     59799, 60271, 60745, 61219, 61694, 62170, 62647, 63124, 63603, 64083, 64563, 65044, 65527};

const UWORD16 fdp_scf[63] = {/* 2 .^ ((0:62 + 1) / 4) */
                             1,     1,     2,    2,     2,     3,     3,     4,     5,     6,
                             7,     8,     10,   11,    13,    16,    19,    23,    27,    32,
                             38,    45,    54,   64,    76,    91,    108,   128,   152,   181,
                             215,   256,   304,  362,   431,   512,   609,   724,   861,   1024,
                             1218,  1448,  1722, 2048,  2435,  2896,  3444,  4096,  4871,  5793,
                             6889,  8192,  9742, 11585, 13777, 16384, 19484, 23170, 27554, 32768,
                             38968, 46341, 55109}; /*65536*/

const UWORD16 impeghe_fdp_s1[129] =
    {/*  49152 * fdp_scl .* fdp_sin */
     0,     726,   1451,  2176,  2901,  3625,  4348,  5071,  5792,  6512,  7230,  7947,  8662,
     9376,  10087, 10796, 11503, 12207, 12908, 13607, 14303, 14995, 15685, 16371, 17054, 17732,
     18408, 19079, 19746, 20409, 21068, 21722, 22372, 23017, 23658, 24293, 24924, 25549, 26169,
     26784, 27394, 27998, 28596, 29189, 29776, 30357, 30932, 31501, 32064, 32621, 33171, 33715,
     34253, 34784, 35309, 35827, 36339, 36844, 37282, 37698, 38105, 38503, 38892, 39272, 39643,
     40005, 40359, 40703, 41039, 41366, 41684, 41994, 42296, 42589, 42874, 43151, 43419, 43680,
     43933, 44179, 44416, 44646, 44869, 45085, 45293, 45494, 45689, 45876, 46057, 46232, 46400,
     46562, 46717, 46867, 47011, 47149, 47281, 47408, 47529, 47645, 47756, 47862, 47963, 48059,
     48150, 48237, 48319, 48397, 48471, 48540, 48605, 48666, 48724, 48777, 48826, 48872, 48914,
     48953, 48988, 49019, 49047, 49072, 49093, 49111, 49126, 49137, 49146, 49150, 49152};

const WORD16 impeghe_fdp_s2[129] =
    {/* -18432 * fdp_scl .* fdp_scl */
     -26681, -26680, -26678, -26675, -26670, -26665, -26658, -26650, -26641, -26630, -26618,
     -26605, -26591, -26576, -26559, -26541, -26522, -26502, -26481, -26459, -26436, -26411,
     -26386, -26359, -26331, -26303, -26273, -26242, -26211, -26178, -26145, -26110, -26075,
     -26039, -26002, -25964, -25926, -25887, -25847, -25806, -25765, -25723, -25680, -25637,
     -25593, -25549, -25504, -25458, -25413, -25366, -25320, -25273, -25225, -25178, -25130,
     -25082, -25033, -24985, -24856, -24710, -24564, -24417, -24272, -24126, -23981, -23836,
     -23691, -23547, -23404, -23262, -23121, -22980, -22841, -22702, -22565, -22429, -22295,
     -22162, -22030, -21900, -21771, -21644, -21519, -21396, -21274, -21155, -21037, -20921,
     -20808, -20697, -20587, -20481, -20376, -20274, -20174, -20077, -19982, -19889, -19799,
     -19712, -19627, -19546, -19466, -19390, -19316, -19245, -19177, -19112, -19049, -18990,
     -18933, -18879, -18829, -18781, -18736, -18695, -18656, -18620, -18588, -18558, -18532,
     -18508, -18488, -18471, -18457, -18446, -18438, -18434, -18432};

/**
 * @defgroup CoreEncProc Core Encoder processing
 * @ingroup  CoreEncProc
 * @brief Core Encoder processing
 *
 * @{
 */

/**
 *  impeghe_get_peak_idx
 *
 *  \brief Helper function to find peak index value in a buffer.
 *
 *  \param [in ] ptr_mdct       Pointer to MDCT buffer.
 *  \param [in ] band       Band Value
 *  \param [out] idx        Pointer to index value
 *  \param [in ] ptr_sfb_offset Pointer to SFB table.
 *
 *  \return VOID
 */
static VOID impeghe_get_peak_idx(FLOAT64 *ptr_mdct, WORD32 band, WORD32 *idx,
                                 WORD32 *ptr_sfb_offset)
{
  WORD32 max_i, i = 0;
  FLOAT64 max;

  max = ptr_mdct[ptr_sfb_offset[band]];
  max_i = ptr_sfb_offset[band];

  for (i = ptr_sfb_offset[band]; i < ptr_sfb_offset[band + 1]; i++)
  {
    if (ptr_mdct[i] > max)
    {
      max = ptr_mdct[i];
      max_i = i;
    }
  }

  *idx = max_i;

  return;
}

/**
 *  impeghe_find_max_energy_bin
 *
 *  \brief Helper function to find peak energy value index.
 *
 *  \param [in ] ptr_mdct_out      Pointer to MDCT buffer.
 *  \param [in ] sfb_offset    Pointer to SFB offset table.
 *  \param [out] fdp_last_sfb  Pointer to freq. domain predictor last SFB value.
 *  \param [in] pstr_scratch  Pointer to scratch memory.
 *
 *  \return WORD32 Maximum band value
 */
static WORD32 impeghe_find_max_energy_bin(FLOAT64 *ptr_mdct_out, WORD32 *ptr_sfb_offset,
                                          WORD32 *fdp_last_sfb, impeghe_scratch_mem *pstr_scratch)
{
  WORD32 sfb, i, max_band = 0;
  FLOAT64 max = 0;
  FLOAT64 *energy = pstr_scratch->p_fdp_href;

  memset(energy, 0, MAX_NUM_SFB_LONG * sizeof(FLOAT64));

  for (sfb = 0; sfb < MAX_NUM_SFB_LONG; sfb++)
  {
    if (ptr_sfb_offset[sfb] >= FDP_MAX_PRED_BW)
    {
      *fdp_last_sfb = sfb;
      break;
    }

    for (i = ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 1]; i++)
    {
      energy[sfb] += ABS(ptr_mdct_out[i]) * ABS(ptr_mdct_out[i]);
    }
    if (energy[sfb] > max)
    {
      max = energy[sfb];
      max_band = sfb;
    }
  }

  return max_band;
}

/**
 *  impeghe_find_href
 *
 *  \brief Helper function to find Harmonic reference.
 *
 *  \param [out] ptr_href  Pointer to harmonic reference buffer.
 *  \param [in ] ptr_mdct  Pointer to MDCT data
 *  \param [in ] x1    Start index value.
 *  \param [in ] x2    Stop index value.
 *
 *  \return VOID
 */

static VOID impeghe_find_href(FLOAT64 *ptr_href, FLOAT64 *ptr_mdct, WORD32 x1, WORD32 x2)
{
  WORD32 i;
  FLOAT64 b = pow((ptr_mdct[x2] / ptr_mdct[x1]), (1.0 / (x1 - x2)));
  FLOAT64 a = ptr_mdct[x1] * (pow(b, x1));
  for (i = x1; i <= x2; i++)
  {
    ptr_href[i] = a * (pow(b, (-i)));
  }

  return;
}

/**
 *  impeghe_fd_chn_fdp_decode
 *
 *  \brief Frequency Domain Prediction decoding.
 *
 *  \param [in,out] pstr_usac_data       Pointer to USAC data structure.
 *  \param [in,out] ptr_fdp_int           Pointer to estimated fdp values.
 *  \param [in ]  fdp_spacing_index Spacing index value extracted from bit stream.
 *  \param [in ]  pred_bw           Prediction bandwidth.
 *  \param [in ]  max_sfb           Max Scalefactor band value.
 *  \param [in ]  chn               Processing channel index.
 *
 *  \return VOID
 *
 */
static VOID impeghe_fd_chn_fdp_decode(ia_usac_data_struct *pstr_usac_data,
                                      WORD32 fdp_spacing_index, WORD32 pred_bw, WORD32 max_sfb,
                                      WORD32 chn, WORD32 *ptr_fdp_int)
{
  const WORD32 fdp_spacing_value = 894 / 3 - fdp_spacing_index;
  WORD32 i = 0, s1, s2, sfb, harmonic_spacing = 0;
  WORD32 harmonic_idx = -128, compare_idx = 256;
  WORD32 is_eight_short_sequence = 0;
  const WORD32 *ptr_sfb_offset = pstr_usac_data->str_psy_mod.str_psy_long_config[chn].sfb_offset;

  FLOAT64 *ptr_coef = pstr_usac_data->spectral_line_vector[chn];

  WORD16 *ptr_quant_spec_prev2 = pstr_usac_data->fdp_data.fdp_quant_spec_prev[chn];
  WORD16 *ptr_quant_spec_prev1 = pstr_usac_data->fdp_data.fdp_quant_spec_prev[chn] + 172;

  if (is_eight_short_sequence || (max_sfb <= 0))
  { /* no FD predictor */
    max_sfb = 0;
  }
  else
  {
    if (pred_bw < ptr_sfb_offset[max_sfb])
    { /* ISO 23003-3, Table 109 */
      max_sfb = 1;
      while (ptr_sfb_offset[max_sfb] < pred_bw)
        max_sfb++;

      if ((fdp_spacing_index >= 0) && (fdp_spacing_index < 256))
      {
        harmonic_spacing = (894 * 512 + fdp_spacing_value) / (2 * fdp_spacing_value);
      }
    }
  }

  s1 = 0;
  s2 = 0; /* reset coefficients, adapt both for each harmonic */

  /* start decoding: for each quantized bin compute predictor estimate */
  for (sfb = 0; sfb < max_sfb; sfb++)
  {
    if (harmonic_spacing)
    { /* FDP active and allowed, compute estimate */
      for (i = (sfb <= 0) ? 0 : ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 1]; i++)
      {
        if (ia_abs_int(i * 256 - harmonic_idx) >= 384)
        { /* bin is not harmonic */
          ptr_fdp_int[i] = 0;
        }
        else
        { /* bin is part of the currently active harmonic line */
          const WORD32 reg32 =
              s1 * (WORD32)ptr_quant_spec_prev1[i] + s2 * (WORD32)ptr_quant_spec_prev2[i];
          ptr_fdp_int[i] = (WORD32)(((UWORD32)ia_abs_int(reg32) + 16384) >> 15);
          if (reg32 < 0)
          {
            ptr_fdp_int[i] *= -1;
          }

          ptr_coef[i] -= (512.0f * (FLOAT32)ptr_fdp_int[i]); /* actual subtract */
        }
        if (i * 256 == compare_idx)
        { /* update indices and LPC coeffs */
          harmonic_idx += harmonic_spacing;
          compare_idx = harmonic_idx & 255;
          if (compare_idx > 128)
          {
            compare_idx = 256 - compare_idx; /* exploit trigonom. symmetry */
          }

          s1 = impeghe_fdp_s1[compare_idx];
          s2 = impeghe_fdp_s2[compare_idx];

          compare_idx = harmonic_idx >> 8; /* integer unscaled harm. index */
          if ((compare_idx & 1) == 0)
          {
            s1 *= -1; /* negate first LPC coeff for even harm. indices */
          }
          compare_idx = 256 + ((harmonic_idx + 128) >> 8) * 256;
        }
      }
    }
  }

  return;
}

/**
 *  impeghe_fd_chn_fdp_decode_update
 *
 *  \brief FDP data updating function.
 *
 *  \param [in] pstr_usac_data  Pointer to USAC data structure.
 *  \param [in] max_sfb    Maximum scalefactor band value.
 *  \param [in] chn        Channel index value.
 *  \param [in] is_long    Long window flag.
 *  \param [in] ptr_fdp_int    Pointer to FDP estimated values.
 *  \param [in] idx     Channel index 0 or 1.
 *
 *  \return VOID
 */
VOID impeghe_fd_chn_fdp_decode_update(ia_usac_data_struct *pstr_usac_data, WORD32 max_sfb,
                                      WORD32 chn, WORD32 is_long, WORD32 *ptr_fdp_int, WORD32 idx)
{
  WORD32 pred_bw = 132;
  WORD32 fdp_spacing_index = pstr_usac_data->fdp_data.fdp_spacing_idx[chn];
  const WORD32 fdp_spacing_value = 894 / 3 - fdp_spacing_index;
  WORD32 i = 0, s1, s2, sfb, harmonic_spacing = 0;
  WORD32 harmonic_idx = -128, compare_idx = 256;
  WORD32 is_eight_short_sequence = !is_long;
  const WORD32 *ptr_sfb_offset = pstr_usac_data->str_psy_mod.str_psy_long_config[chn].sfb_offset;

  WORD32 *ptr_quant = pstr_usac_data->str_quant_info[idx].quant_degroup;

  WORD32 *ptr_quant_scf_curr = pstr_usac_data->str_quant_info[idx].scale_factor;
  WORD16 *ptr_quant_spec_prev2 = pstr_usac_data->fdp_data.fdp_quant_spec_prev[chn];
  WORD16 *ptr_quant_spec_prev1 = pstr_usac_data->fdp_data.fdp_quant_spec_prev[chn] + 172;

  if (is_eight_short_sequence || (max_sfb <= 0))
  { /* no FD predictor */
    max_sfb = 0;
  }
  else
  {
    if (pred_bw < ptr_sfb_offset[max_sfb])
    { /* ISO 23003-3, Table 109 */
      max_sfb = 1;
      while (ptr_sfb_offset[max_sfb] < pred_bw)
        max_sfb++;

      if ((fdp_spacing_index >= 0) && (fdp_spacing_index < 256))
      {
        harmonic_spacing = (894 * 512 + fdp_spacing_value) / (2 * fdp_spacing_value);
      }
    }
  }

  s1 = 0;
  s2 = 0; /* reset coefficients, adapt both for each harmonic */

  /* start decoding: for each quantized bin compute predictor estimate */
  for (sfb = 0; sfb < max_sfb; sfb++)
  {
    WORD32 scf16 = ptr_quant_scf_curr[sfb] - SF_OFFSET - 21; /* shifted sf[sfb] */

    if (scf16 < 0)
    {
      scf16 = 0;
    }
    else
    {
      scf16 = scf16 > 62 ? 65536 : (WORD32)fdp_scf[scf16]; /* scf look-up */
    }

    if (harmonic_spacing)
    { /* FDP active and allowed, compute estimate */
      for (i = (sfb <= 0) ? 0 : ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 1]; i++)
      {
        if (ia_abs_int(i * 256 - harmonic_idx) >= 384)
        { /* bin is not harmonic */
          ptr_fdp_int[i] = 0;
        }
        else
        { /* bin is part of the currently active harmonic line */
          const WORD32 reg32 =
              s1 * (WORD32)ptr_quant_spec_prev1[i] + s2 * (WORD32)ptr_quant_spec_prev2[i];
          ptr_fdp_int[i] = (WORD32)(((UWORD32)ia_abs_int(reg32) + 16384) >> 15);
          if (reg32 < 0)
          {
            ptr_fdp_int[i] *= -1;
          }
        }
        if (i * 256 == compare_idx)
        { /* update indices and LPC coeffs */
          harmonic_idx += harmonic_spacing;
          compare_idx = harmonic_idx & 255;
          if (compare_idx > 128)
          {
            compare_idx = 256 - compare_idx; /* exploit trigonom. symmetry */
          }

          s1 = impeghe_fdp_s1[compare_idx];
          s2 = impeghe_fdp_s2[compare_idx];

          compare_idx = harmonic_idx >> 8; /* integer unscaled harm. index */
          if ((compare_idx & 1) == 0)
          {
            s1 *= -1; /* negate first LPC coeff for even harm. indices */
          }
          compare_idx = 256 + ((harmonic_idx + 128) >> 8) * 256;
        }
      }
    }

    /* start update: compute integer sum of each line and its estimate */
    for (i = (sfb <= 0) ? 0 : ptr_sfb_offset[sfb]; i < ptr_sfb_offset[sfb + 1]; i++)
    {
      WORD32 x_int = fdp_exp[ia_min_int(ia_abs_int(ptr_quant[i]), 181)]; /* look-up */

      x_int = (WORD32)((512 + (UWORD32)x_int * (UWORD32)scf16) >> 10);
      if (ptr_quant[i] < 0)
      {
        x_int *= -1;
      }
      if (harmonic_spacing)
      {
        x_int += ptr_fdp_int[i]; /* add previously computed FDP estimate */
      }

      ptr_quant_spec_prev2[i] = ptr_quant_spec_prev1[i];
      ptr_quant_spec_prev1[i] =
          (short)ia_min_int(ia_max_int(x_int, QUANT_SPEC_MIN), QUANT_SPEC_MAX);
    }
  }

  /* finalize update: reset states of currently uncoded spectral lines */
  for (i = ia_min_int(i, pred_bw); i < UNCODED_LINES; i++)
  {
    ptr_quant_spec_prev2[i] = ptr_quant_spec_prev1[i] = 0;
  }
}

/**
 *  impeghe_tcx_fdp_encode
 *
 *  \brief Encodes FDP data in TCX module.
 *
 *  \param [in ] pstr_fdp_data              Pointer to FDP data structure.
 *  \param [out] ptr_out_spec_curr   Pointer to output spectral data.
 *  \param [in ] ptr_tcx_quant       Pointer to TCX data buffer.
 *  \param [in ] quant_gain_curr Quantization gain value.
 *  \param [in ] i_gain          Gain value.
 *  \param [in ] lg              Length of a signal.
 *  \param [in ] max_lines       Maximum spectral lines.
 *  \param [in ] pred_bw         Prediction bandwidth.
 *  \param [in ] chn             Channel value.
 *  \param [in] ptr_fdp_int    Pointer to FDP estimated values.
 *
 *  \return VOID
 */
VOID impeghe_tcx_fdp_encode(ia_usac_fdp_data_struct *pstr_fdp_data, FLOAT32 *ptr_out_spec_curr,
                            FLOAT32 *ptr_tcx_quant, WORD32 quant_gain_curr, FLOAT32 i_gain,
                            WORD32 lg, WORD32 max_lines, WORD32 pred_bw, WORD32 chn,
                            WORD32 *ptr_fdp_int)
{
  WORD32 i, s1, s2, x, reg32, fdp_spacing_value, harmonic_spacing;
  WORD32 harmonic_idx = -128;
  WORD32 compare_idx = 256;
  memset(ptr_fdp_int, 0, sizeof(WORD32) * 172);

  /* step 1 */
  fdp_spacing_value = (894 / 3) - pstr_fdp_data->fdp_spacing_idx[chn];
  harmonic_spacing = (894 * 512 + fdp_spacing_value) / (2 * fdp_spacing_value);

  /* step 2 */
  pred_bw = ia_min_int(lg, pred_bw);

  /* step 3*/
  s1 = 0;
  s2 = 0;
  pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][1][0] = 0;

  if (pstr_fdp_data->fdp_active[chn])
  {
    for (i = 0; i < pred_bw; i++)
    {
      if (ia_abs_int(i * 256 - harmonic_idx) >= 384)
      {
        ptr_fdp_int[i] = 0;
      }
      else
      {
        reg32 = s1 * pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][0][i] +
                s2 * pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][1][i];
        ptr_fdp_int[i] = (WORD32)(((UWORD32)ia_abs_int(reg32) + 16384) >> 15);
        if (reg32 < 0)
        {
          ptr_fdp_int[i] *= -1;
        }

        ptr_out_spec_curr[i] = ptr_out_spec_curr[i] - ia_mul_flt(i_gain, (FLOAT32)ptr_fdp_int[i]);
      }
      if (i * 256 == compare_idx)
      {
        harmonic_idx += harmonic_spacing;
        compare_idx = harmonic_idx & 255;
        if (compare_idx > 128)
        {
          compare_idx = 256 - compare_idx;
        }

        s1 = impeghe_fdp_s1[compare_idx];
        s2 = impeghe_fdp_s2[compare_idx];

        compare_idx = harmonic_idx >> 8;
        if ((compare_idx & 1) == 0)
        {
          s1 *= -1;
        }
        compare_idx = 256 + ((harmonic_idx + 128) >> 8) * 256;
      }
    }
  }

  /* step 4 */
  for (i = 0; i < pred_bw; i++)
  {

    x = (WORD32)(ptr_tcx_quant[i] * quant_gain_curr);
    if (pstr_fdp_data->fdp_active[chn])
    {
      x += ptr_fdp_int[i];
    }

    pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][1][i] =
        pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][0][i];
    pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][0][i] =
        (WORD16)ia_min_int(ia_max_int(x, QUANT_SPEC_MIN), QUANT_SPEC_MAX);
  }

  for (i = ia_min_int(i, max_lines); i < 172; i++)
  {
    pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][1][i] =
        pstr_fdp_data->fdp_quant_spec_prev_tcx[chn][0][i] = 0;
  }
  return;
}

/**
 *  impeghe_fdp_encode
 *
 *  \brief FDP encoding interface function.
 *
 *  \param [in,out] pstr_usac_data   Pointer to USAC data structure.
 *  \param [in] pstr_usac_config     Pointer to USAC configuration structure.
 *  \param [in] i_ch                Channel index value.
 *  \param [in] max_sfb             Maximum SFB value.
 *  \param [in] core_mode           Core mode value.
 *
 *  \return VOID
 */
VOID impeghe_fdp_encode(ia_usac_data_struct *pstr_usac_data,
                        ia_usac_encoder_config_struct *pstr_usac_config, WORD32 i_ch,
                        WORD32 max_sfb, WORD32 core_mode)
{
  WORD32 fdp_last_sfb = 0;
  WORD32 x1, x2, band, i;
  impeghe_scratch_mem *pstr_scratch = &pstr_usac_data->str_scratch;
  FLOAT64 *ptr_href = pstr_usac_data->str_scratch.p_fdp_href;
  FLOAT64 *ptr_mdct_out = pstr_usac_data->str_scratch.p_fdp_mdct_out;
  WORD32 *ptr_sfb_offset = pstr_usac_data->str_psy_mod.str_psy_long_config[i_ch].sfb_offset;

  /* reset */
  pstr_usac_data->fdp_data.fdp_active[i_ch] = 0;

  /* skip LFE */
  if (i_ch == 3)
    return;

  impeghe_fft_based_mdct(pstr_usac_data->ptr_time_data[i_ch], ptr_mdct_out, 1024,
                         ((pstr_usac_config->prev_aliasing_symmetry << 1) |
                          pstr_usac_config->curr_aliasing_symmetry),
                         pstr_scratch);
  /*Mdct calculated in fd_frm_enc is with windowed input which cant be reused to find f0*/

  for (i = 0; i < 1024; i++)
  {
    ptr_mdct_out[i] = ABS(ptr_mdct_out[i]); /*grabbing only magnitude part*/
  }

  band = impeghe_find_max_energy_bin(ptr_mdct_out, ptr_sfb_offset, &fdp_last_sfb, pstr_scratch);
  impeghe_get_peak_idx(ptr_mdct_out, band, &x1,
                       ptr_sfb_offset); /* x1 is bin corresponding to fundamental frequency */

  if (ABS(x1 - pstr_usac_data->fdp_data.fdp_prev_s0[i_ch]) > 1)
  {
    pstr_usac_data->fdp_data.fdp_prev_s0[i_ch] = x1; /* update prev */
    pstr_usac_data->fdp_data.fdp_spacing_idx[i_ch] = -1;
    return; /* no FDP */
  }
  else
  {
    pstr_usac_data->fdp_data.fdp_prev_s0[i_ch] = x1; /* update prev */
  }

  if (x1 <= 20 && x1 >= 3) /* To ensure fundamental frequency is within the range-470Hz */
  {
    impeghe_get_peak_idx(ptr_mdct_out, fdp_last_sfb - 1, &x2,
                         ptr_sfb_offset); /* Find peak in last band */
    impeghe_find_href(ptr_href, ptr_mdct_out, x1,
                      x2); /* Get the harmonic reference between 2 peaks- 0th and 5th band */

    for (i = x1 + x1; i < x2; i += x1)
    {
      if (ptr_mdct_out[i] > ptr_href[i])
      {
        pstr_usac_data->fdp_data.fdp_active[i_ch] =
            1; /* Set fdp to active if any harmonic coeffs cross the harmonic reference */
        break;
      }
    }

    if (pstr_usac_data->fdp_data.fdp_active[i_ch])
    {
      pstr_usac_data->fdp_data.fdp_spacing_idx[i_ch] = 298 - (894 / x1); /* to bitstream */
      /* removing harmonics */
      if (core_mode == CORE_MODE_FD)
      {
        impeghe_fd_chn_fdp_decode(pstr_usac_data, pstr_usac_data->fdp_data.fdp_spacing_idx[i_ch],
                                  FDP_MAX_PRED_BW, max_sfb, i_ch, pstr_scratch->ptr_fdp_int);
      }
    }
    else
    {
      pstr_usac_data->fdp_data.fdp_spacing_idx[i_ch] = -1;
    }
  }
  else
  {
    pstr_usac_data->fdp_data.fdp_active[i_ch] = 0; /* to bitstream */
    pstr_usac_data->fdp_data.fdp_spacing_idx[i_ch] = -1;
  }

  return;
}

/** @} */ /* End of CoreEncProc */
