//
// File: ndi_control.h
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//
#ifndef NDI_CONTROL_H
#define NDI_CONTROL_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ndi_control_types.h"

// Function Declarations
extern void ndi_control(const float x[12], const float vel_dot_ref[3], const
  float vel_ff[3], float max_bank, float mass, float L, float delta, float
  kT_max, float kMT, float gf, float acc_com[3]);

#endif

//
// File trailer for ndi_control.h
//
// [EOF]
//
