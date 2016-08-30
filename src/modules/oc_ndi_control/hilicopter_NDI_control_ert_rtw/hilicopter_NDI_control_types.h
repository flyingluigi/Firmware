/*
 * File: hilicopter_NDI_control_types.h
 *
 * Code generated for Simulink model 'hilicopter_NDI_control'.
 *
 * Model version                  : 1.176
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Tue Aug 30 09:26:35 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_hilicopter_NDI_control_types_h_
#define RTW_HEADER_hilicopter_NDI_control_types_h_
#include "rtwtypes.h"
#ifndef _DEFINED_TYPEDEF_FOR_struct_LsE4pJyaGkFOziDPkuTr6_
#define _DEFINED_TYPEDEF_FOR_struct_LsE4pJyaGkFOziDPkuTr6_

typedef struct {
  real_T L;
  real_T R[24];
  real_T delta;
  real_T kT;
  real_T kMT;
  real_T rpm2pwm;
  real_T kP;
  real_T kv;
  real_T cD;
  real_T cmD;
  real_T g;
  real_T mass;
  real_T Ib[9];
  real_T Ibi[9];
  real_T tau;
  real_T Lsb[3];
  real_T n2trim;
  real_T unmax;
  real_T Tmn[48];
  real_T Tnm[48];
} struct_LsE4pJyaGkFOziDPkuTr6;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_hilicopter_NDI_contro_T RT_MODEL_hilicopter_NDI_contr_T;

#endif                                 /* RTW_HEADER_hilicopter_NDI_control_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
