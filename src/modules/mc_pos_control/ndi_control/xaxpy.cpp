//
// File: xaxpy.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "xaxpy.h"

// Function Definitions

//
// Arguments    : int n
//                float a
//                const float x[9]
//                int ix0
//                float y[3]
//                int iy0
// Return Type  : void
//
void b_xaxpy(int n, float a, const float x[9], int ix0, float y[3], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                float a
//                const float x[3]
//                int ix0
//                float y[9]
//                int iy0
// Return Type  : void
//
void c_xaxpy(int n, float a, const float x[3], int ix0, float y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                float a
//                int ix0
//                float y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, float a, int ix0, float y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
