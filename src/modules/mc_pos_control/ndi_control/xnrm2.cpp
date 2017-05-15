//
// File: xnrm2.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "xnrm2.h"

// Function Definitions

//
// Arguments    : int n
//                const float x[3]
//                int ix0
// Return Type  : float
//
float b_xnrm2(int n, const float x[3], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = fabsf(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

//
// Arguments    : int n
//                const float x[9]
//                int ix0
// Return Type  : float
//
float xnrm2(int n, const float x[9], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = fabsf(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
