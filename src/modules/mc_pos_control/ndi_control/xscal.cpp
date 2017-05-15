//
// File: xscal.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "xscal.h"

// Function Definitions

//
// Arguments    : float a
//                float x[9]
//                int ix0
// Return Type  : void
//
void xscal(float a, float x[9], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

//
// File trailer for xscal.cpp
//
// [EOF]
//
