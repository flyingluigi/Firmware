//
// File: svd1.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "svd1.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xscal.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"

// Function Definitions

//
// Arguments    : const float A[9]
//                float U[9]
//                float S[9]
//                float V[9]
// Return Type  : void
//
void svd(const float A[9], float U[9], float S[9], float V[9])
{
  float b_A[9];
  int kase;
  float s[3];
  float e[3];
  float work[3];
  float Vf[9];
  int q;
  int m;
  int qs;
  boolean_T apply_transform;
  float ztest0;
  int ii;
  float ztest;
  int iter;
  float snorm;
  float rt;
  int exitg3;
  boolean_T exitg2;
  float f;
  float varargin_1[5];
  float mtmp;
  boolean_T exitg1;
  float sqds;
  for (kase = 0; kase < 9; kase++) {
    b_A[kase] = A[kase];
  }

  for (kase = 0; kase < 3; kase++) {
    s[kase] = 0.0F;
    e[kase] = 0.0F;
    work[kase] = 0.0F;
  }

  for (kase = 0; kase < 9; kase++) {
    U[kase] = 0.0F;
    Vf[kase] = 0.0F;
  }

  for (q = 0; q < 2; q++) {
    qs = q + 3 * q;
    apply_transform = false;
    ztest0 = xnrm2(3 - q, b_A, qs + 1);
    if (ztest0 > 0.0F) {
      apply_transform = true;
      if (b_A[qs] < 0.0F) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabsf(s[q]) >= 9.86076132E-32F) {
        ztest = 1.0F / s[q];
        kase = (qs - q) + 3;
        for (ii = qs; ii + 1 <= kase; ii++) {
          b_A[ii] *= ztest;
        }
      } else {
        kase = (qs - q) + 3;
        for (ii = qs; ii + 1 <= kase; ii++) {
          b_A[ii] /= s[q];
        }
      }

      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0F;
    }

    for (ii = q + 1; ii + 1 < 4; ii++) {
      kase = q + 3 * ii;
      if (apply_transform) {
        xaxpy(3 - q, -(xdotc(3 - q, b_A, qs + 1, b_A, kase + 1) / b_A[q + 3 * q]),
              qs + 1, b_A, kase + 1);
      }

      e[ii] = b_A[kase];
    }

    for (ii = q; ii + 1 < 4; ii++) {
      U[ii + 3 * q] = b_A[ii + 3 * q];
    }

    if (q + 1 <= 1) {
      ztest0 = b_xnrm2(2, e, 2);
      if (ztest0 == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          ztest = -ztest0;
        } else {
          ztest = ztest0;
        }

        if (e[1] < 0.0F) {
          e[0] = -ztest0;
        } else {
          e[0] = ztest0;
        }

        if (fabsf(e[0]) >= 9.86076132E-32F) {
          ztest = 1.0F / e[0];
          for (ii = 1; ii + 1 < 4; ii++) {
            e[ii] *= ztest;
          }
        } else {
          for (ii = 1; ii + 1 < 4; ii++) {
            e[ii] /= ztest;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (ii = 2; ii < 4; ii++) {
          work[ii - 1] = 0.0F;
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          b_xaxpy(2, e[ii], b_A, 3 * ii + 2, work, 2);
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          c_xaxpy(2, -e[ii] / e[1], work, 2, b_A, 3 * ii + 2);
        }
      }

      for (ii = 1; ii + 1 < 4; ii++) {
        Vf[ii] = e[ii];
      }
    }
  }

  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0F;
  for (ii = 0; ii < 3; ii++) {
    U[6 + ii] = 0.0F;
  }

  U[8] = 1.0F;
  for (q = 1; q >= 0; q += -1) {
    qs = q + 3 * q;
    if (s[q] != 0.0F) {
      for (ii = q + 1; ii + 1 < 4; ii++) {
        kase = (q + 3 * ii) + 1;
        xaxpy(3 - q, -(xdotc(3 - q, U, qs + 1, U, kase) / U[qs]), qs + 1, U,
              kase);
      }

      for (ii = q; ii + 1 < 4; ii++) {
        U[ii + 3 * q] = -U[ii + 3 * q];
      }

      U[qs]++;
      ii = 1;
      while (ii <= q) {
        U[3] = 0.0F;
        ii = 2;
      }
    } else {
      for (ii = 0; ii < 3; ii++) {
        U[ii + 3 * q] = 0.0F;
      }

      U[qs] = 1.0F;
    }
  }

  for (q = 2; q >= 0; q += -1) {
    if ((q + 1 <= 1) && (e[0] != 0.0F)) {
      for (ii = 2; ii < 4; ii++) {
        kase = 3 * (ii - 1) + 2;
        xaxpy(2, -(xdotc(2, Vf, 2, Vf, kase) / Vf[1]), 2, Vf, kase);
      }
    }

    for (ii = 0; ii < 3; ii++) {
      Vf[ii + 3 * q] = 0.0F;
    }

    Vf[q + 3 * q] = 1.0F;
  }

  for (q = 0; q < 3; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0F) {
      rt = fabsf(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 3) {
        ztest0 = e[q] / ztest;
      }

      xscal(ztest, U, 1 + 3 * q);
    }

    if ((q + 1 < 3) && (ztest0 != 0.0F)) {
      rt = fabsf(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      xscal(ztest, Vf, 1 + 3 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0F;
  for (ii = 0; ii < 3; ii++) {
    snorm = fmaxf(snorm, fmaxf(fabsf(s[ii]), fabsf(e[ii])));
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    ii = m;
    do {
      exitg3 = 0;
      q = ii + 1;
      if (ii + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabsf(e[ii]);
        if ((ztest0 <= 1.1920929E-7F * (fabsf(s[ii]) + fabsf(s[ii + 1]))) ||
            (ztest0 <= 9.86076132E-32F) || ((iter > 20) && (ztest0 <=
              1.1920929E-7F * snorm))) {
          e[ii] = 0.0F;
          exitg3 = 1;
        } else {
          ii--;
        }
      }
    } while (exitg3 == 0);

    if (ii + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg2 = false;
      while ((!exitg2) && (kase >= ii + 1)) {
        qs = kase;
        if (kase == ii + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (kase < m + 2) {
            ztest0 = fabsf(e[kase - 1]);
          }

          if (kase > ii + 2) {
            ztest0 += fabsf(e[kase - 2]);
          }

          ztest = fabsf(s[kase - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[kase - 1] = 0.0F;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == ii + 1) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (ii = m; ii + 1 >= q + 1; ii--) {
        xrotg(&s[ii], &f, &ztest0, &ztest);
        if (ii + 1 > q + 1) {
          f = -ztest * e[0];
          e[0] *= ztest0;
        }

        xrot(Vf, 1 + 3 * ii, 1 + 3 * (m + 1), ztest0, ztest);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (ii = q; ii + 1 <= m + 2; ii++) {
        xrotg(&s[ii], &f, &ztest0, &ztest);
        f = -ztest * e[ii];
        e[ii] *= ztest0;
        xrot(U, 1 + 3 * ii, 1 + 3 * (q - 1), ztest0, ztest);
      }
      break;

     case 3:
      varargin_1[0] = fabsf(s[m + 1]);
      varargin_1[1] = fabsf(s[m]);
      varargin_1[2] = fabsf(e[m]);
      varargin_1[3] = fabsf(s[q]);
      varargin_1[4] = fabsf(e[q]);
      kase = 1;
      mtmp = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        ii = 2;
        exitg1 = false;
        while ((!exitg1) && (ii < 6)) {
          kase = ii;
          if (!rtIsNaNF(varargin_1[ii - 1])) {
            mtmp = varargin_1[ii - 1];
            exitg1 = true;
          } else {
            ii++;
          }
        }
      }

      if (kase < 5) {
        while (kase + 1 < 6) {
          if (varargin_1[kase] > mtmp) {
            mtmp = varargin_1[kase];
          }

          kase++;
        }
      }

      f = s[m + 1] / mtmp;
      ztest0 = s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0F;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0F) || (ztest0 != 0.0F)) {
        ztest = sqrtf(rt * rt + ztest0);
        if (rt < 0.0F) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0F;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      rt = sqds * (e[q] / mtmp);
      for (ii = q + 1; ii <= m + 1; ii++) {
        xrotg(&f, &rt, &ztest0, &ztest);
        if (ii > q + 1) {
          e[0] = f;
        }

        f = ztest0 * s[ii - 1] + ztest * e[ii - 1];
        e[ii - 1] = ztest0 * e[ii - 1] - ztest * s[ii - 1];
        rt = ztest * s[ii];
        s[ii] *= ztest0;
        xrot(Vf, 1 + 3 * (ii - 1), 1 + 3 * ii, ztest0, ztest);
        s[ii - 1] = f;
        xrotg(&s[ii - 1], &rt, &ztest0, &ztest);
        f = ztest0 * e[ii - 1] + ztest * s[ii];
        s[ii] = -ztest * e[ii - 1] + ztest0 * s[ii];
        rt = ztest * e[ii];
        e[ii] *= ztest0;
        xrot(U, 1 + 3 * (ii - 1), 1 + 3 * ii, ztest0, ztest);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        xscal(-1.0F, Vf, 1 + 3 * q);
      }

      kase = q + 1;
      while ((q + 1 < 3) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        xswap(Vf, 1 + 3 * q, 1 + 3 * (q + 1));
        xswap(U, 1 + 3 * q, 1 + 3 * (q + 1));
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ii = 0; ii < 3; ii++) {
    e[ii] = s[ii];
    for (kase = 0; kase < 3; kase++) {
      V[kase + 3 * ii] = Vf[kase + 3 * ii];
    }
  }

  for (kase = 0; kase < 9; kase++) {
    S[kase] = 0.0F;
  }

  for (ii = 0; ii < 3; ii++) {
    S[ii + 3 * ii] = e[ii];
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
