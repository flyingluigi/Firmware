/*
 * File: quad_ndi_types.h
 *
 * Code generated for Simulink model 'quad_ndi'.
 *
 * Model version                  : 1.78
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Thu Sep 01 16:38:06 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_quad_ndi_types_h_
#define RTW_HEADER_quad_ndi_types_h_
#include "rtwtypes.h"
#ifndef _DEFINED_TYPEDEF_FOR_struct_qBkO5CuRvZhP3suhbfp4LG_
#define _DEFINED_TYPEDEF_FOR_struct_qBkO5CuRvZhP3suhbfp4LG_

typedef struct {
  real_T L;
  real_T att_max;
  real_T rates_max;
  real_T yawrate_max;
  real_T umax;
  real_T tau;
  real_T r1[3];
  real_T r2[3];
  real_T r3[3];
  real_T r4[3];
  real_T kT;
  real_T kMT;
  real_T rpm2pwm;
  real_T pwm_zero;
  real_T kv;
  real_T cD;
  real_T cmD;
  real_T g;
  real_T mass;
  real_T Ib[9];
  real_T Ibi[9];
  real_T n2trim;
  real_T Tmn[16];
  real_T Tnm[16];
  real_T Lsb[3];
} struct_qBkO5CuRvZhP3suhbfp4LG;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_quad_ndi_T RT_MODEL_quad_ndi_T;

#endif                                 /* RTW_HEADER_quad_ndi_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
