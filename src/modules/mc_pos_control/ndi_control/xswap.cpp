//
// File: xswap.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "xswap.h"

// Function Definitions

//
// Arguments    : float x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
void xswap(float x[9], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// File trailer for xswap.cpp
//
// [EOF]
//
