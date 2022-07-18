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
#include <math.h>
#include "impeghe_type_def.h"
#include "impeghe_hoa_common_values.h"
#include "impeghe_error_standards.h"
#include "impeghe_error_codes.h"
#include "impeghe_simple_matrix.h"

#define CHSIGN(v1, v2) ((v2) >= 0.0 ? fabs(v1) : -fabs(v1))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

/**
 *  impeghe_hoa_ren_simple_mtrx_init_with_data_ptr
 *
 *  \brief Initialize matrix with given data array.
 *
 *  \param [out] pstr_hoa_matrix	Pointer to HOA matrix info
 *  \param [in]  rows		Number of rows
 *  \param [in]  cols		Number of columns
 *  \param [in]	ptr_data	Pointer to data
 *
 *  \return VOID
 *
 */
VOID impeghe_hoa_ren_simple_mtrx_init_with_data_ptr(
    ia_render_hoa_simple_mtrx_str *pstr_hoa_matrix, WORD32 rows, WORD32 cols, pFlOAT64 ptr_data)
{
  pstr_hoa_matrix->cols = cols;
  pstr_hoa_matrix->rows = rows;
  pstr_hoa_matrix->mtrx = ptr_data;
  return;
}

/**
 *  impeghe_hoa_ren_simple_mtrx_init_with_matrix
 *
 *  \brief Initialize matrix with given matrix
 *
 *  \param [in]	pstr_hoa_matrix_src	Input matrix
 *  \param [out]	pstr_hoa_matrix_dst		Output matrix
 *
 *  \return VOID
 *
 */
static VOID
impeghe_hoa_ren_simple_mtrx_init_with_matrix(ia_render_hoa_simple_mtrx_str *pstr_hoa_matrix_src,
                                             ia_render_hoa_simple_mtrx_str *pstr_hoa_matrix_dst)
{
  pstr_hoa_matrix_dst->cols = pstr_hoa_matrix_src->cols;
  pstr_hoa_matrix_dst->rows = pstr_hoa_matrix_src->rows;
  memcpy(pstr_hoa_matrix_dst->mtrx, pstr_hoa_matrix_src->mtrx,
         sizeof(FLOAT64) * (pstr_hoa_matrix_dst->cols) * (pstr_hoa_matrix_dst->rows));
  return;
}

/**
 *  impeghe_hoa_ren_simple_mtrx_transpose
 *
 *  \brief Transpose matrix and store in same matrix using temproary matrix.
 *
 *  \param [in,out]	pstr_hoa_mtx	Matrix for inplace transpose
 *  \param [out]	ptr_scratch	Pointer to scratch buffer for intermediate processing
 *
 *  \return VOID
 *
 */
static VOID impeghe_hoa_ren_simple_mtrx_transpose(ia_render_hoa_simple_mtrx_str *pstr_hoa_mtx,
                                                  pVOID ptr_scratch)
{
  WORD32 t;
  pFlOAT64 ptr_tmp = (pFlOAT64)ptr_scratch;
  for (WORD32 r = 0; r < pstr_hoa_mtx->rows; r++)
  {
    for (WORD32 c = 0; c < pstr_hoa_mtx->cols; c++)
    {
      ptr_tmp[c * pstr_hoa_mtx->rows + r] = pstr_hoa_mtx->mtrx[r * pstr_hoa_mtx->cols + c];
    }
  }
  t = pstr_hoa_mtx->rows;
  pstr_hoa_mtx->rows = pstr_hoa_mtx->cols;
  pstr_hoa_mtx->cols = t;
  for (WORD32 r = 0; r < (pstr_hoa_mtx->rows) * (pstr_hoa_mtx->cols); r++)
  {
    pstr_hoa_mtx->mtrx[r] = ptr_tmp[r];
  }
}

/**
 *  impeghe_hoa_ren_simple_mtrx_swap_cols
 *
 *  \brief Reaggrange matrix columns with given set of columns
 *
 *  \param [in,out]	pstr_hoa_mtx	Matrix handle
 *  \param [in]		idx		Array of indices
 *  \param [in]		ptr_scratch	Pointer to scratch buffer for intermediate processing
 *
 *  \return VOID
 *
 */
static VOID impeghe_hoa_ren_simple_mtrx_swap_cols(ia_render_hoa_simple_mtrx_str *pstr_hoa_mtx,
                                                  pWORD32 idx, pVOID ptr_scratch)
{
  pFlOAT64 ptr_m2 = (pFlOAT64)ptr_scratch;
  for (WORD32 rc = 0; rc < MAX_MATRIX_SIZE; rc++)
    ptr_m2[rc] = 0.0;
  for (WORD32 r = 0; r < pstr_hoa_mtx->rows; r++)
  {
    for (WORD32 c = 0; c < pstr_hoa_mtx->cols; c++)
    {
      ptr_m2[r * pstr_hoa_mtx->cols + c] = pstr_hoa_mtx->mtrx[r * pstr_hoa_mtx->cols + idx[c]];
    }
  }
  for (WORD32 i = 0; i < (pstr_hoa_mtx->rows) * (pstr_hoa_mtx->cols); i++)
  {
    pstr_hoa_mtx->mtrx[i] = ptr_m2[i];
  }
  return;
}

/**
 *  impeghe_hoa_get_sort_idx
 *
 *  \brief Bubble sorting and stores sorted index in given array.
 *
 *  \param [in]		ptr_arr	Array to be sorted
 *  \param [in]		n	Size of array
 *  \param [out]	idx	Sorted array index sequence of original array
 *
 *  \return VOID
 *
 */
static VOID impeghe_hoa_get_sort_idx(const pFlOAT64 ptr_arr, WORD32 n, pWORD32 idx)
{
  WORD32 i, j;
  for (i = 0; i < n; i++)
  {
    idx[i] = i;
  }

  for (i = 0; i < n; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (ptr_arr[idx[i]] < ptr_arr[idx[j]])
      {
        WORD32 ival = idx[j];
        idx[j] = idx[i];
        idx[i] = ival;
      }
    }
  }
  return;
}

/**
 *  impeghe_hoa_pythagoras
 *
 *  \brief Calculates pythagoras value
 *
 *  \param [in]	v1	Value a
 *  \param [in]	v2	Value b
 *
 *  \return FLOAT64	Pythagoras value
 *
 */
static FLOAT64 impeghe_hoa_pythagoras(FLOAT64 v1, FLOAT64 v2)
{
  FLOAT64 at = (FLOAT64)fabs(v1), bt = (FLOAT64)fabs(v2);
  FLOAT64 ct, res;
  if (at > bt)
  {
    ct = bt / at;
    res = (FLOAT64)(at * sqrt(1.0f + ct * ct));
  }
  else if (bt > 0.0)
  {
    ct = at / bt;
    res = (FLOAT64)(bt * sqrt(1.0f + ct * ct));
  }
  else
    res = 0.0;
  return (res);
}

/**
 *  impeghe_hoa_dsvd
 *
 *  \brief Singular value decomposition
 *
 *  \param [out]	pstr_hoa_mtx_a	Matix U
 *  \param [in]		rows		Number of rows
 *  \param [in]		cols		Number of columns
 *  \param [out]	ptr_w			Values array of diagonal matrix
 *  \param [out]	pstr_hoa_mtx_v	Matix V
 *  \param [in]		ptr_scratch Pointer to scratch buffer for intermediate processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE impeghe_hoa_dsvd(ia_render_hoa_simple_mtrx_str *pstr_hoa_mtx_a, WORD32 rows,
                                     WORD32 cols, pFlOAT64 ptr_w,
                                     ia_render_hoa_simple_mtrx_str *pstr_hoa_mtx_v,
                                     pVOID ptr_scratch)
{
  WORD32 flag, i, its, j, jj, k;
  WORD32 nm = 0;
  WORD32 l = 0;
  FLOAT64 c, f, h, s, x, y, z;
  FLOAT64 anorm = 0.0, g = 0.0, scale = 0.0;
  pFlOAT64 ptr_rv1 = (FLOAT64 *)ptr_scratch;
  memset(ptr_scratch, 0, sizeof(FLOAT64) * cols);
  for (i = 0; i < cols; i++)
  {
    l = i + 1;
    ptr_rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < rows)
    {
      for (k = i; k < rows; k++)
        scale += (FLOAT64)fabs(pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i]);
      if (scale)
      {
        for (k = i; k < rows; k++)
        {
          pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] =
              pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] / scale;
          s += pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] *
               pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i];
        }
        f = pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + i];
        g = (FLOAT64)(-CHSIGN(sqrt(s), f));
        h = f * g - s;
        pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + i] = f - g;
        if (i != cols - 1)
        {
          for (j = l; j < cols; j++)
          {
            for (s = 0.0, k = i; k < rows; k++)
              s += pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] *
                   pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + j];
            f = s / h;
            for (k = i; k < rows; k++)
              pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + j] +=
                  (f * pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i]);
          }
        }
        for (k = i; k < rows; k++)
          pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] =
              pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] * scale;
      }
    }
    ptr_w[i] = (scale * g);
    g = s = scale = 0.0;
    if (i < rows && i != cols - 1)
    {
      for (k = l; k < cols; k++)
        scale += (FLOAT64)fabs(pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k]);
      if (scale)
      {
        for (k = l; k < cols; k++)
        {
          pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] =
              pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] / scale;
          s += pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] *
               pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k];
        }
        f = pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + l];
        g = (FLOAT64)(-CHSIGN(sqrt(s), f));
        h = f * g - s;
        pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + l] = f - g;
        for (k = l; k < cols; k++)
          ptr_rv1[k] = pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] / h;
        if (i != rows - 1)
        {
          for (j = l; j < rows; j++)
          {
            for (s = 0.0, k = l; k < cols; k++)
              s += pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + k] *
                   pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k];
            for (k = l; k < cols; k++)
              pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + k] += s * ptr_rv1[k];
          }
        }
        for (k = l; k < cols; k++)
          pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] =
              pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] * scale;
      }
    }
    anorm = (FLOAT64)(MAX(anorm, (fabs(ptr_w[i]) + fabs(ptr_rv1[i]))));
  }
  for (i = cols - 1; i >= 0; i--)
  {
    if (i < cols - 1)
    {
      if (g)
      {
        for (j = l; j < cols; j++)
          pstr_hoa_mtx_v->mtrx[j * pstr_hoa_mtx_v->cols + i] =
              (pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + j] /
               pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + l]) /
              g;
        for (j = l; j < cols; j++)
        {
          for (s = 0.0, k = l; k < cols; k++)
            s += pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + k] *
                 pstr_hoa_mtx_v->mtrx[k * pstr_hoa_mtx_v->cols + j];
          for (k = l; k < cols; k++)
            pstr_hoa_mtx_v->mtrx[k * pstr_hoa_mtx_v->cols + j] +=
                s * pstr_hoa_mtx_v->mtrx[k * pstr_hoa_mtx_v->cols + i];
        }
      }
      for (j = l; j < cols; j++)
        pstr_hoa_mtx_v->mtrx[i * pstr_hoa_mtx_v->cols + j] =
            pstr_hoa_mtx_v->mtrx[j * pstr_hoa_mtx_v->cols + i] = 0.0;
    }
    pstr_hoa_mtx_v->mtrx[i * pstr_hoa_mtx_v->cols + i] = 1.0;
    g = ptr_rv1[i];
    l = i;
  }
  for (i = cols - 1; i >= 0; i--)
  {
    l = i + 1;
    g = ptr_w[i];
    if (i < cols - 1)
      for (j = l; j < cols; j++)
        pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + j] = 0.0;
    if (g)
    {
      g = 1.0f / g;
      if (i != cols - 1)
      {
        for (j = l; j < cols; j++)
        {
          for (s = 0.0, k = l; k < rows; k++)
            s += pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i] *
                 pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + j];
          f = (s / pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + i]) * g;
          for (k = i; k < rows; k++)
            pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + j] +=
                f * pstr_hoa_mtx_a->mtrx[k * pstr_hoa_mtx_a->cols + i];
        }
      }
      for (j = i; j < rows; j++)
        pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + i] =
            pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + i] * g;
    }
    else
    {
      for (j = i; j < rows; j++)
        pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + i] = 0.0;
    }
    ++pstr_hoa_mtx_a->mtrx[i * pstr_hoa_mtx_a->cols + i];
  }
  for (k = cols - 1; k >= 0; k--)
  {
    for (its = 0; its < 30; its++)
    {
      flag = 1;
      for (l = k; l >= 0; l--)
      {
        nm = l - 1;
        if (fabs(ptr_rv1[l]) + anorm == anorm)
        {
          flag = 0;
          break;
        }
        if (fabs(ptr_w[nm]) + anorm == anorm)
          break;
      }
      if (flag)
      {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++)
        {
          f = s * ptr_rv1[i];
          if (fabs(f) + anorm != anorm)
          {
            g = ptr_w[i];
            h = impeghe_hoa_pythagoras(f, g);
            ptr_w[i] = h;
            h = 1.0f / h;
            c = g * h;
            s = (-f * h);
            for (j = 0; j < rows; j++)
            {
              y = pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + nm];
              z = pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + i];
              pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + nm] = (y * c + z * s);
              pstr_hoa_mtx_a->mtrx[j * pstr_hoa_mtx_a->cols + i] = (z * c - y * s);
            }
          }
        }
      }
      z = ptr_w[k];
      if (l == k)
      {
        if (z < 0.0)
        {
          ptr_w[k] = (-z);
          for (j = 0; j < cols; j++)
            pstr_hoa_mtx_v->mtrx[j * pstr_hoa_mtx_v->cols + k] =
                -pstr_hoa_mtx_v->mtrx[j * pstr_hoa_mtx_v->cols + k];
        }
        break;
      }
      if (its >= 30)
      {
        return (0);
      }
      x = ptr_w[l];
      nm = k - 1;
      y = ptr_w[nm];
      g = ptr_rv1[nm];
      h = ptr_rv1[k];
      f = (FLOAT64)(((y - z) * (y + z) + (g - h) * (g + h)) / (2.0f * h * y));
      g = impeghe_hoa_pythagoras(f, 1.0f);
      f = (FLOAT64)(((x - z) * (x + z) + h * ((y / (f + CHSIGN(g, f))) - h)) / x);
      c = s = 1.0;
      for (j = l; j <= nm; j++)
      {
        i = j + 1;
        g = ptr_rv1[i];
        y = ptr_w[i];
        h = s * g;
        g = c * g;
        z = impeghe_hoa_pythagoras(f, h);
        ptr_rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y = y * c;
        for (jj = 0; jj < cols; jj++)
        {
          x = pstr_hoa_mtx_v->mtrx[jj * pstr_hoa_mtx_v->cols + j];
          z = pstr_hoa_mtx_v->mtrx[jj * pstr_hoa_mtx_v->cols + i];
          pstr_hoa_mtx_v->mtrx[jj * pstr_hoa_mtx_v->cols + j] = (x * c + z * s);
          pstr_hoa_mtx_v->mtrx[jj * pstr_hoa_mtx_v->cols + i] = (z * c - x * s);
        }
        z = impeghe_hoa_pythagoras(f, h);
        ptr_w[j] = z;
        if (z)
        {
          z = 1.0f / z;
          c = f * z;
          s = h * z;
        }
        f = (c * g) + (s * y);
        x = (c * y) - (s * g);
        for (jj = 0; jj < rows; jj++)
        {
          y = pstr_hoa_mtx_a->mtrx[jj * pstr_hoa_mtx_a->cols + j];
          z = pstr_hoa_mtx_a->mtrx[jj * pstr_hoa_mtx_a->cols + i];
          pstr_hoa_mtx_a->mtrx[jj * pstr_hoa_mtx_a->cols + j] = (y * c + z * s);
          pstr_hoa_mtx_a->mtrx[jj * pstr_hoa_mtx_a->cols + i] = (z * c - y * s);
        }
      }
      ptr_rv1[l] = 0.0;
      ptr_rv1[k] = f;
      ptr_w[k] = x;
    }
  }
  return (1);
}

/**
 *  impeghe_hoa_ren_simple_mtrx_svd_init
 *
 *  \brief Initialize simple matrix for svd
 *
 *  \param [in,out]	pstr_hoa_svd		Matrix svd handle to init
 *  \param [out]	pstr_hoa_src	Matrix handle
 *  \param [in]		ptr_scratch			Pointer to scratch buffer for intermediate
 * processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghe_hoa_ren_simple_mtrx_svd_init(ia_render_hoa_simple_mtrx_svd_str *pstr_hoa_svd,
                                                  ia_render_hoa_simple_mtrx_str *pstr_hoa_src,
                                                  pVOID ptr_scratch)
{
  pWORD8 ptr_buf = (pWORD8)ptr_scratch;
  WORD32 scratch_idx = 0, transpose_flag = 0, rows, cols, rc, i;
  pFlOAT64 wsx;
  pWORD32 idx;
  ia_render_hoa_simple_mtrx_str *pstr_hoa_mtx =
      (ia_render_hoa_simple_mtrx_str *)(ptr_buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  pstr_hoa_mtx->mtrx = (pFlOAT64)(ptr_buf + scratch_idx);
  scratch_idx += sizeof(FLOAT64) * (pstr_hoa_src->rows) * (pstr_hoa_src->cols);

  impeghe_hoa_ren_simple_mtrx_init_with_matrix(pstr_hoa_src, pstr_hoa_mtx);
  if (pstr_hoa_mtx->rows < pstr_hoa_mtx->cols)
  {
    transpose_flag = 1;
    impeghe_hoa_ren_simple_mtrx_transpose(pstr_hoa_mtx, (ptr_buf + scratch_idx));
  }
  rows = pstr_hoa_mtx->rows;
  cols = pstr_hoa_mtx->cols;

  wsx = (FLOAT64 *)(ptr_buf + scratch_idx);
  scratch_idx += sizeof(FLOAT64) * (MAX_MATRIX_SIZE);
  for (rc = 0; rc < MAX_MATRIX_SIZE; rc++)
    wsx[rc] = 0.0;
  pstr_hoa_svd->w_s_sz = cols;
  pstr_hoa_svd->v.rows = pstr_hoa_svd->v.cols = cols;

  if (impeghe_hoa_dsvd(pstr_hoa_mtx, rows, cols, wsx, &(pstr_hoa_svd->v),
                       (ptr_buf + scratch_idx)) == 0)
  {
    return IMPEGHE_EXE_NONFATAL_HOA_VEC_EST_ERROR;
  }
  idx = (WORD32 *)(ptr_buf + scratch_idx);
  scratch_idx += sizeof(WORD32) * (MAX_MATRIX_SIZE);

  for (rc = 0; rc < MAX_MATRIX_SIZE; rc++)
    idx[rc] = 0;
  impeghe_hoa_get_sort_idx(wsx, cols, idx);
  pstr_hoa_svd->s.cols = pstr_hoa_svd->s.rows = cols;
  for (i = 0; i < cols; i++)
  {
    pstr_hoa_svd->w_s[i] = wsx[idx[i]];
    pstr_hoa_svd->s.mtrx[i * pstr_hoa_svd->s.cols + i] = pstr_hoa_svd->w_s[i];
  }
  impeghe_hoa_ren_simple_mtrx_swap_cols(&(pstr_hoa_svd->v), idx, (ptr_buf + scratch_idx));
  impeghe_hoa_ren_simple_mtrx_swap_cols(pstr_hoa_mtx, idx, (ptr_buf + scratch_idx));
  if (transpose_flag)
  {
    impeghe_hoa_ren_simple_mtrx_init_with_matrix(&(pstr_hoa_svd->v), &(pstr_hoa_svd->u));
    impeghe_hoa_ren_simple_mtrx_init_with_matrix(pstr_hoa_mtx, &(pstr_hoa_svd->v));
  }
  else
    impeghe_hoa_ren_simple_mtrx_init_with_matrix(pstr_hoa_mtx, &(pstr_hoa_svd->u));

  return IA_NO_ERROR;
}
