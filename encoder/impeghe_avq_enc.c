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
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_avq_enc.h"

/**
 * @defgroup CoreEncROM Core Encoder ROM Tables
 * @ingroup  CoreEncROM
 * @brief Core Encoder ROM Tables
 *
 * @{
 */

/**
 *  impeghe_gosset_compute_rank_and_sign
 *
 *
 *  \brief Albegraic Vector Quantization(AVQ) module's utility function.
 *         Computes rank and sign of vector in Gosset Lattice.
 *
 *  \param [in ] x         Pointer to 8-dimensional vector.
 *  \param [out] rank      Pointer to computed rank value.
 *  \param [out] sign_code Pointer to computed sign code value.
 *
 *  \return VOID
 */
static VOID impeghe_gosset_compute_rank_and_sign(WORD32 *x, WORD32 *rank, WORD32 *sign_code)
{
  WORD32 xs[8], a[8], q, d[8], w[8], A, B, idx, tmp, abs_i, abs_j;
  WORD32 i, j, k;
  for (i = 0; i < 8; i++)
  {
    xs[i] = x[i];
  }
  for (k = 0; k < 7; k++)
  {
    j = k;
    for (i = k + 1; i < 8; i++)
    {
      abs_j = abs(xs[j]);
      abs_i = abs(xs[i]);
      if (abs_i >= abs_j)
      {
        if (abs_i > xs[j])
        {
          j = i;
        }
      }
    }
    if (j > k)
    {
      tmp = xs[k];
      xs[k] = xs[j];
      xs[j] = tmp;
    }
  }
  *sign_code = 0;
  for (i = 0; i < 8; i++)
  {
    if (xs[i] < 0)
    {
      *sign_code += impeghe_pow2_table[i];
    }
  }
  a[0] = xs[0];
  q = 1;
  for (i = 1; i < 8; i++)
  {
    if (xs[i] != xs[i - 1])
    {
      a[q] = xs[i];
      q++;
    }
  }
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < q; j++)
    {
      if (x[i] == a[j])
      {
        d[i] = j;
        break;
      }
    }
  }
  *rank = 0;
  for (j = 0; j < q; j++)
  {
    w[j] = 0;
  }
  B = 1;
  for (i = 7; i >= 0; i--)
  {
    idx = d[i];
    w[idx]++;
    B *= w[idx];
    A = 0;
    for (j = 0; j < idx; j++)
    {
      A += w[j];
    }
    if (A > 0)
    {
      *rank += A * impeghe_factorial_table[i] / B;
    }
  }
}

/**
 *  impeghe_gosset_compute_base_idx
 *
 *  \brief Albegraic Vector Quantization(AVQ) module's utility function.
 *         Computes index of vector in the Gosset Lattice.
 *
 *  \param [in ] x    Pointer to 8-dimensional vector.
 *  \param [in ] ka   Index for table used by the function.
 *  \param [out] idx  Pointer to index value.
 *
 *  \return VOID
 */
static VOID impeghe_gosset_compute_base_idx(WORD32 *x, WORD32 ka, WORD32 *idx)
{
  WORD32 rank, offset, code, i, ks;
  impeghe_gosset_compute_rank_and_sign(x, &rank, &code);
  ks = -1;
  for (i = impeghe_iso_code_index_table[ka]; i < LEN_SIGN_LEADER; i++)
  {
    if (code == impeghe_iso_code_data_table[i])
    {
      ks = i;
      break;
    }
  }
  if (ks >= 0)
  {
    offset = impeghe_signed_leader_is[ks];
    *idx = offset + rank;
  }
  return;
}

/**
 *  impeghe_find_absolute_leader
 *
 *  \brief Albegraic Vector Quantization(AVQ) module's utility function.
 *         Absolute leader finder, among the vectors in Gosset lattice.
 *
 *  \param [in] y Pointer to input 8-dimensional vector.
 *
 *  \return Absolute leader vector index value.
 *
 */
static WORD32 impeghe_find_absolute_leader(WORD32 *y)
{
  WORD32 i, nb, pos, ka;
  WORD64 s, C[8], id;
  for (i = 0; i < 8; i++)
  {
    C[i] = (WORD64)y[i] * y[i];
  }
  s = 0;
  for (i = 0; i < 8; i++)
  {
    s += C[i];
  }
  s >>= 3;
  ka = LEN_ABS_LEADER + 1;
  if (s == 0)
  {
    ka = LEN_ABS_LEADER;
  }
  else
  {
    if (s <= NB_SPHERE)
    {
      id = 0;
      for (i = 0; i < 8; i++)
      {
        id += C[i] * C[i];
      }
      id = id >> 3;
      nb = impeghe_da_num_bits[s - 1];
      pos = impeghe_da_pos[s - 1];
      for (i = 0; i < nb; i++)
      {
        if (id == (long)impeghe_da_id[pos])
        {
          ka = pos;
          break;
        }
        pos++;
      }
    }
  }
  return (ka);
}

/**
 *  impeghe_nearest_neighbor_2d
 *
 *  \brief Albegraic Vector Quantization(AVQ) module's utility function.
 *         Utility function to find nearest neighbor to a vector.
 *
 *  \param [in ] x Pointer to 8-dimensional vector.
 *  \param [out] y Pointer to the resulting nearest neighbour vector.
 *
 *  \return VOID
 */
static VOID impeghe_nearest_neighbor_2d(FLOAT32 *x, WORD32 *y)
{
  WORD32 i, j, sum;
  FLOAT32 diff[8], em;
  sum = 0;
  for (i = 0; i < 8; i++)
  {
    if (x[i] < 0)
    {
      y[i] = -2 * (((WORD32)(1.0 - x[i])) >> 1);
    }
    else
    {
      y[i] = 2 * (((WORD32)(1.0 + x[i])) >> 1);
    }
    sum += y[i];
  }
  if (sum % 4)
  {
    FLOAT32 s;
    em = 0;
    j = 0;
    for (i = 0; i < 8; i++)
    {
      diff[i] = x[i] - y[i];
      s = (FLOAT32)fabs(diff[i]);
      if (em < s)
      {
        em = s;
        j = i;
      }
    }
    if (diff[j] < 0)
    {
      y[j] -= 2;
    }
    else
    {
      y[j] += 2;
    }
  }
  return;
}

/**
 *  impeghe_find_nearest_neighbor
 *
 *  \brief Albegraic Vector Quantization(AVQ) module's utility function.
 *         Utility function to find nearest neighbor to a vector.
 *
 *  \param [in ] bk Pointer to 8-dimensional vector.
 *  \param [out] ck Pointer to nearest vector neighbour.
 *
 *  \return VOID
 */
VOID impeghe_find_nearest_neighbor(FLOAT32 *bk, WORD32 *ck)
{
  WORD32 i, y1[8], y2[8];
  FLOAT32 e1k, e2k, x1[8], tmp;

  impeghe_nearest_neighbor_2d(bk, y1);

  for (i = 0; i < 8; i++)
  {
    x1[i] = bk[i] - 1.0f;
  }
  impeghe_nearest_neighbor_2d(x1, y2);
  for (i = 0; i < 8; i++)
  {
    y2[i] += 1;
  }
  /* Compute e1k = (Bk - y1k)^2 and e2k = (Bk � y2k)^2 */
  e1k = e2k = 0.0;
  for (i = 0; i < 8; i++)
  {
    tmp = bk[i] - y1[i];
    e1k += tmp * tmp;
    tmp = bk[i] - y2[i];
    e2k += tmp * tmp;
  }

  /* Select best lattice point */
  if (e1k < e2k)
  {
    for (i = 0; i < 8; i++)
    {
      ck[i] = y1[i];
    }
  }
  else
  {
    for (i = 0; i < 8; i++)
    {
      ck[i] = y2[i];
    }
  }
  return;
}

/**
 *  impeghe_vononoi_idx
 *
 *  \brief Computes Voronoi extension index
 *
 *  \param [out] kv Pointer to voroni extension vector.
 *  \param [in ] m  Index value.
 *  \param [out] y  Pointer to output 8-dim vector.
 *
 *  \return VOID
 */
static VOID impeghe_vononoi_idx(WORD32 *kv, WORD32 m, WORD32 *y)
{
  WORD32 i, v[8], tmp, sum, *ptr1, *ptr2;
  FLOAT32 z[8];
  for (i = 0; i < 8; i++)
  {
    y[i] = kv[7];
  }
  z[7] = (FLOAT32)y[7] / m;
  sum = 0;
  for (i = 6; i >= 1; i--)
  {
    tmp = kv[i] << 1;
    sum += tmp;
    y[i] += tmp;
    z[i] = (FLOAT32)y[i] / m;
  }
  y[0] += (4 * kv[0] + sum);
  z[0] = (FLOAT32)(y[0] - 2) / m;
  impeghe_find_nearest_neighbor(z, v);
  ptr1 = y;
  ptr2 = v;
  for (i = 0; i < 8; i++)
  {
    *ptr1++ -= m * *ptr2++;
  }
}

/**
 *  impeghe_compute_coord
 *
 *  \brief Helper function that computes 8-dim vector coordinates
 *
 *  \param [in ] y Pointer to input array.
 *  \param [out] k Pointer to the computed vector.
 *
 *  \return VOID
 */
static VOID impeghe_compute_coord(WORD32 *y, WORD32 *k)
{
  WORD32 i, tmp, sum;
  k[7] = y[7];
  tmp = y[7];
  sum = 5 * y[7];
  for (i = 6; i >= 1; i--)
  {
    k[i] = (y[i] - tmp) >> 1;
    sum -= y[i];
  }
  k[0] = (y[0] + sum) >> 2;
}

/**
 *  impeghe_apply_voronoi_ext
 *
 *  \brief Performs Voronoi Extension of AVQ.
 *
 *  \param [in ] x   Pointer to 8-dimensional input vector.
 *  \param [out] n   Pointer to number of quantization bits value.
 *  \param [out] idx Pointer to gosset lattice vector index value.
 *  \param [out] k   Pointer to index value.
 *
 *  \return VOID
 */
VOID impeghe_apply_voronoi_ext(WORD32 *x, WORD32 *n, WORD32 *idx, WORD32 *k)
{
  WORD32 ka, c[8];
  WORD32 i, r, m, v[8], c_tmp[8], k_mod[8], k_tmp[8], iter, ka_tmp, n_tmp, mask;
  FLOAT32 sphere;
  ka = impeghe_find_absolute_leader(x);
  *n = impeghe_da_nq[ka];
  if (*n <= 4)
  {
    for (i = 0; i < 8; i++)
    {
      c[i] = x[i];
    }
  }
  else
  {
    sphere = 0.0;
    for (i = 0; i < 8; i++)
    {
      sphere += (FLOAT32)x[i] * (FLOAT32)x[i];
    }
    sphere *= 0.125;
    r = 1;
    sphere *= 0.25;
    while (sphere > 11.0)
    {
      r++;
      sphere *= 0.25;
    }
    impeghe_compute_coord(x, k_mod);
    m = 1 << r;
    mask = m - 1;
    for (iter = 0; iter < 2; iter++)
    {
      for (i = 0; i < 8; i++)
      {
        k_tmp[i] = k_mod[i] & mask;
      }
      impeghe_vononoi_idx(k_tmp, m, v);
      for (i = 0; i < 8; i++)
      {
        c_tmp[i] = (x[i] - v[i]) / m;
      }
      ka_tmp = impeghe_find_absolute_leader(c_tmp);
      n_tmp = impeghe_da_nq[ka_tmp];
      if (n_tmp > 4)
      {
        r++;
        m = m << 1;
        mask = ((mask << 1) + 1);
      }
      else
      {
        if (n_tmp < 3)
        {
          n_tmp = 3;
        }
        ka = ka_tmp;
        *n = n_tmp + 2 * r;
        for (i = 0; i < 8; i++)
        {
          k[i] = k_tmp[i];
        }
        for (i = 0; i < 8; i++)
        {
          c[i] = c_tmp[i];
        }
        r--;
        m = m >> 1;
        mask = mask >> 1;
      }
    }
  }

  if (*n > 0)
  {
    impeghe_gosset_compute_base_idx(c, ka, idx);
  }
  return;
}

/**
 *  impeghe_alg_vec_quant
 *
 *  \brief Algebraic Vector Quantization(AVQ) main interface function.
 *
 *  \param [in ] ptr_input   Pointer to input data.
 *  \param [out] ptr_out     Pointer to output data.
 *  \param [out] ptr_lpc_idx Pointer to LPC indices data.
 *
 *  \return VOID
 */
VOID impeghe_alg_vec_quant(FLOAT32 *ptr_input, WORD32 *ptr_out, WORD32 *ptr_lpc_idx)
{
  WORD32 i, l, nq, pos, c[8], kv[8];
  FLOAT32 x1[8];
  WORD32 lpc_index;

  pos = 2;
  for (l = 0; l < 2; l++)
  {
    for (i = 0; i < 8; i++)
    {
      x1[i] = ptr_input[l * 8 + i];
      kv[i] = 0;
    }

    impeghe_find_nearest_neighbor(x1, c);

    impeghe_apply_voronoi_ext(c, &nq, &lpc_index, kv);

    for (i = 0; i < 8; i++)
    {
      ptr_out[l * 8 + i] = c[i];
    }

    ptr_lpc_idx[l] = nq;

    if (nq > 0)
    {
      ptr_lpc_idx[pos++] = lpc_index;
      for (i = 0; i < 8; i++)
      {
        ptr_lpc_idx[pos++] = kv[i];
      }
    }
  }

  return;
}
/** @} */ /* End of CoreEncROM */
