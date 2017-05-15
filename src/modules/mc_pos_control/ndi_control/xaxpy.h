//
// File: xaxpy.h
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//
#ifndef XAXPY_H
#define XAXPY_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ndi_control_types.h"

// Function Declarations
extern void b_xaxpy(int n, float a, const float x[9], int ix0, float y[3], int
                    iy0);
extern void c_xaxpy(int n, float a, const float x[3], int ix0, float y[9], int
                    iy0);
extern void xaxpy(int n, float a, int ix0, float y[9], int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
