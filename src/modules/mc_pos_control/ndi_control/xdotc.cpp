//
// File: xdotc.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "xdotc.h"

// Function Definitions

//
// Arguments    : int n
//                const float x[9]
//                int ix0
//                const float y[9]
//                int iy0
// Return Type  : float
//
float xdotc(int n, const float x[9], int ix0, const float y[9], int iy0)
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// File trailer for xdotc.cpp
//
// [EOF]
//
