/*
 * File: _coder_ndi_control_api.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-May-2017 13:59:48
 */

#ifndef _CODER_NDI_CONTROL_API_H
#define _CODER_NDI_CONTROL_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_ndi_control_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void ndi_control(real32_T x[12], real32_T vel_dot_ref[3], real32_T
  vel_ff[3], real32_T max_bank, real32_T mass, real32_T L, real32_T delta,
  real32_T kT_max, real32_T kMT, real32_T gf, real32_T acc_com[3]);
extern void ndi_control_api(const mxArray * const prhs[10], const mxArray *plhs
  [1]);
extern void ndi_control_atexit(void);
extern void ndi_control_initialize(void);
extern void ndi_control_terminate(void);
extern void ndi_control_xil_terminate(void);

#endif

/*
 * File trailer for _coder_ndi_control_api.h
 *
 * [EOF]
 */
