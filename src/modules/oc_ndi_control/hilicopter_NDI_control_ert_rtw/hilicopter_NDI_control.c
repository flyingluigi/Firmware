/*
 * File: hilicopter_NDI_control.c
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

#include "hilicopter_NDI_control.h"
#include "hilicopter_NDI_control_private.h"

/* Block states (auto storage) */
DW_hilicopter_NDI_control_T hilicopter_NDI_control_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_hilicopter_NDI_control_T hilicopter_NDI_control_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_hilicopter_NDI_control_T hilicopter_NDI_control_Y;

/* Real-time model */
RT_MODEL_hilicopter_NDI_contr_T hilicopter_NDI_control_M_;
RT_MODEL_hilicopter_NDI_contr_T *const hilicopter_NDI_control_M =
  &hilicopter_NDI_control_M_;

/* Forward declaration for local functions */
static void hilicopter_NDI_control_mldivide(const real32_T A[36], real32_T B[6]);

/* Function for MATLAB Function: '<Root>/NDI_control_law' */
static void hilicopter_NDI_control_mldivide(const real32_T A[36], real32_T B[6])
{
  real32_T temp;
  real32_T b_A[36];
  int8_T ipiv[6];
  int32_T j;
  int32_T ix;
  real32_T s;
  int32_T k;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int32_T b_kAcol;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real32_T));
  for (j = 0; j < 6; j++) {
    ipiv[j] = (int8_T)(1 + j);
  }

  for (j = 0; j < 5; j++) {
    b_kAcol = j * 7;
    iy = 0;
    ix = b_kAcol;
    temp = (real32_T)fabs(b_A[b_kAcol]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[b_kAcol + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 6;
          iy += 6;
        }
      }

      iy = (b_kAcol - j) + 6;
      for (ix = b_kAcol + 1; ix + 1 <= iy; ix++) {
        b_A[ix] /= b_A[b_kAcol];
      }
    }

    iy = b_kAcol;
    ix = b_kAcol + 6;
    for (k = 1; k <= 5 - j; k++) {
      temp = b_A[ix];
      if (b_A[ix] != 0.0F) {
        c_ix = b_kAcol + 1;
        d = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= d; ijA++) {
          b_A[ijA] += b_A[c_ix] * -temp;
          c_ix++;
        }
      }

      ix += 6;
      iy += 6;
    }
  }

  for (j = 0; j < 5; j++) {
    if (j + 1 != ipiv[j]) {
      temp = B[j];
      B[j] = B[ipiv[j] - 1];
      B[ipiv[j] - 1] = temp;
    }
  }

  for (j = 0; j < 6; j++) {
    b_kAcol = 6 * j;
    if (B[j] != 0.0F) {
      for (iy = j + 1; iy + 1 < 7; iy++) {
        B[iy] -= b_A[iy + b_kAcol] * B[j];
      }
    }
  }

  for (j = 5; j >= 0; j += -1) {
    b_kAcol = 6 * j;
    if (B[j] != 0.0F) {
      B[j] /= b_A[j + b_kAcol];
      for (iy = 0; iy + 1 <= j; iy++) {
        B[iy] -= b_A[iy + b_kAcol] * B[j];
      }
    }
  }
}

real32_T rt_roundf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = 0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void hilicopter_NDI_control_step(void)
{
  real32_T K_euler[9];
  real32_T sphi;
  real32_T cphi;
  real32_T sthe;
  real32_T cthe;
  real32_T spsi;
  real32_T cpsi;
  real32_T dhdx_fx[72];
  real_T xdot_s[12];
  real32_T Ibi[9];
  real32_T b_sphi;
  real32_T b_cphi;
  real32_T b_sthe;
  real32_T b_cthe;
  real32_T b_spsi;
  real32_T b_cpsi;
  real32_T tthe;
  real32_T Tbn[9];
  real32_T b_k[8];
  real32_T km[8];
  real32_T kL[8];
  static const int8_T b[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  static const int8_T c[12] = { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const int8_T d[12] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 };

  real32_T absxk;
  real32_T t;
  real32_T y[8];
  real32_T rtb_omega_ref[3];
  real32_T rtb_Add;
  real32_T rtb_Add2;
  real32_T rtb_Add1;
  real32_T rtb_NOut;
  real32_T rtb_Product;
  real32_T rtb_NOut_m;
  real32_T rtb_Sum_d;
  real32_T rtb_Add3;
  real32_T rtb_NOut_a;
  uint8_T rtb_mode;
  boolean_T rtb_Compare;
  real32_T rtb_Saturate;
  real32_T rtb_Add4;
  real32_T rtb_Sum_m;
  real32_T rtb_Saturate_n;
  real32_T rtb_Add5;
  boolean_T rtb_Compare_f;
  real32_T rtb_Sum_j;
  real32_T rtb_Saturate_a;
  real32_T rtb_um_com[6];
  real32_T rtb_TmpSignalConversionAtSFunct[3];
  real32_T rtb_euler_ref[3];
  real32_T rtb_um[6];
  int32_T i;
  real32_T cthe_0[36];
  real32_T tmp[3];
  real_T Ibi_0[3];
  real_T Tbn_0[3];
  real32_T tmp_0[9];
  real_T tmp_1[3];
  real32_T tmp_2[3];
  real32_T tmp_3[9];
  real32_T tmp_4[24];
  real32_T tmp_5[24];
  real32_T tmp_6[6];
  real32_T rtb_euler_ref_0[6];
  real32_T b_k_0[24];
  real32_T km_0[24];
  real32_T K_euler_0[24];
  real32_T b_k_1[96];
  real32_T b_k_2[96];
  real32_T b_k_3[72];
  int32_T i_0;
  int32_T j;
  uint16_T u0;

  /* MATLAB Function: '<Root>/mode select' incorporates:
   *  Inport: '<Root>/pwm_in'
   */
  /* MATLAB Function 'mode select': '<S3>:1' */
  /* Define modes according to switch position */
  if (hilicopter_NDI_control_U.pwm_in[5] > 0.5F) {
    /* '<S3>:1:4' */
    /* '<S3>:1:5' */
    rtb_mode = 2U;
  } else {
    /* '<S3>:1:7' */
    rtb_mode = 1U;
  }

  /* End of MATLAB Function: '<Root>/mode select' */

  /* MATLAB Function: '<Root>/mode_logic' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  Product: '<Root>/Product1'
   */
  /* MATLAB Function 'mode_logic': '<S4>:1' */
  /*  select commanded angles according to mode */
  /*  Mode 1 = Stabilize */
  /*  Mode 2 = Velocity Control */
  if (rtb_mode == 2) {
    /* '<S4>:1:6' */
    /* '<S4>:1:7' */
    rtb_euler_ref[0] = 0.0F;
    rtb_euler_ref[1] = 0.0F;
    rtb_euler_ref[2] = 0.0F;
  } else {
    /* '<S4>:1:9' */
    rtb_euler_ref[0] = hilicopter_NDI_control_U.param[34] *
      hilicopter_NDI_control_U.pwm_in[0];
    rtb_euler_ref[1] = hilicopter_NDI_control_U.param[34] *
      hilicopter_NDI_control_U.pwm_in[1];
    rtb_euler_ref[2] = hilicopter_NDI_control_U.param[34] * 0.0F;
  }

  /* End of MATLAB Function: '<Root>/mode_logic' */

  /* MATLAB Function: '<Root>/att_control' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   */
  /* MATLAB Function 'att_control': '<S2>:1' */
  /* '<S2>:1:4' */
  for (i = 0; i < 9; i++) {
    K_euler[i] = 0.0F;
  }

  /* '<S2>:1:5' */
  /* '<S2>:1:6' */
  K_euler[0] = hilicopter_NDI_control_U.param[12];
  rtb_Add = rtb_euler_ref[0] - hilicopter_NDI_control_U.state[9];
  K_euler[4] = hilicopter_NDI_control_U.param[13];
  rtb_NOut = rtb_euler_ref[1] - hilicopter_NDI_control_U.state[10];
  K_euler[8] = hilicopter_NDI_control_U.param[14];
  rtb_Add1 = rtb_euler_ref[2] - hilicopter_NDI_control_U.state[11];
  for (i = 0; i < 3; i++) {
    rtb_omega_ref[i] = K_euler[i + 6] * rtb_Add1 + (K_euler[i + 3] * rtb_NOut +
      K_euler[i] * rtb_Add);
  }

  /* End of MATLAB Function: '<Root>/att_control' */

  /* Sum: '<S5>/Add' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add = rtb_omega_ref[0] - hilicopter_NDI_control_U.state[6];

  /* Product: '<S13>/NOut' incorporates:
   *  DiscreteIntegrator: '<S13>/Filter'
   *  Inport: '<Root>/param'
   *  Product: '<S13>/DOut'
   *  Sum: '<S13>/SumD'
   */
  rtb_NOut = (rtb_Add * hilicopter_NDI_control_U.param[6] -
              hilicopter_NDI_control_DW.Filter_DSTATE) *
    hilicopter_NDI_control_U.param[9];

  /* Sum: '<S5>/Add1' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add1 = rtb_omega_ref[1] - hilicopter_NDI_control_U.state[7];

  /* Product: '<S14>/NOut' incorporates:
   *  DiscreteIntegrator: '<S14>/Filter'
   *  Inport: '<Root>/param'
   *  Product: '<S14>/DOut'
   *  Sum: '<S14>/SumD'
   */
  rtb_NOut_m = (rtb_Add1 * hilicopter_NDI_control_U.param[7] -
                hilicopter_NDI_control_DW.Filter_DSTATE_o) *
    hilicopter_NDI_control_U.param[10];

  /* Product: '<Root>/Product' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  rtb_Product = hilicopter_NDI_control_U.pwm_in[3] *
    hilicopter_NDI_control_U.param[33];

  /* Sum: '<S5>/Add2' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add2 = rtb_Product - hilicopter_NDI_control_U.state[8];

  /* Product: '<S15>/NOut' incorporates:
   *  DiscreteIntegrator: '<S15>/Filter'
   *  Inport: '<Root>/param'
   *  Product: '<S15>/DOut'
   *  Sum: '<S15>/SumD'
   */
  rtb_NOut_a = (rtb_Add2 * hilicopter_NDI_control_U.param[8] -
                hilicopter_NDI_control_DW.Filter_DSTATE_oj) *
    hilicopter_NDI_control_U.param[11];

  /* MATLAB Function: '<Root>/rotate_u' incorporates:
   *  Inport: '<Root>/state'
   */
  /* MATLAB Function 'rotate_u': '<S6>:1' */
  /* Rotate commanded velocities according to heading */
  /* '<S6>:1:4' */
  /* '<S6>:1:8' */
  Ibi[0] = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  Ibi[3] = -(real32_T)sin(hilicopter_NDI_control_U.state[11]);
  Ibi[6] = 0.0F;
  Ibi[1] = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  Ibi[4] = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  Ibi[7] = 0.0F;
  Ibi[2] = 0.0F;

  /* Product: '<Root>/Product2' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  b_spsi = hilicopter_NDI_control_U.pwm_in[1] * hilicopter_NDI_control_U.param
    [35];

  /* MATLAB Function: '<Root>/rotate_u' */
  Ibi[5] = 0.0F;

  /* Product: '<Root>/Product2' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  b_sthe = hilicopter_NDI_control_U.pwm_in[0] * hilicopter_NDI_control_U.param
    [35];

  /* MATLAB Function: '<Root>/rotate_u' */
  Ibi[8] = 1.0F;

  /* Product: '<Root>/Product2' incorporates:
   *  Fcn: '<S7>/Fcn'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  b_cpsi = (hilicopter_NDI_control_U.pwm_in[2] - 0.5F) * 2.0F *
    hilicopter_NDI_control_U.param[35];

  /* MATLAB Function: '<Root>/rotate_u' */
  for (i = 0; i < 3; i++) {
    rtb_euler_ref[i] = Ibi[i + 6] * b_cpsi + (Ibi[i + 3] * b_sthe + Ibi[i] *
      b_spsi);
  }

  /* Sum: '<S11>/Add3' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add3 = rtb_euler_ref[0] - hilicopter_NDI_control_U.state[12];

  /* RelationalOperator: '<S18>/Compare' incorporates:
   *  Constant: '<S18>/Constant'
   */
  rtb_Compare = (rtb_mode == 2);

  /* DiscreteIntegrator: '<S19>/Integrator' */
  if (rtb_Compare && (hilicopter_NDI_control_DW.Integrator_PrevResetState <= 0))
  {
    hilicopter_NDI_control_DW.Integrator_DSTATE_m = 0.0F;
  }

  /* Sum: '<S19>/Sum' incorporates:
   *  DiscreteIntegrator: '<S19>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S19>/POut'
   */
  rtb_Sum_d = rtb_Add3 * hilicopter_NDI_control_U.param[24] +
    hilicopter_NDI_control_DW.Integrator_DSTATE_m;

  /* Saturate: '<S19>/Saturate' */
  if (rtb_Sum_d > 5.0F) {
    rtb_Saturate = 5.0F;
  } else if (rtb_Sum_d < -5.0F) {
    rtb_Saturate = -5.0F;
  } else {
    rtb_Saturate = rtb_Sum_d;
  }

  /* End of Saturate: '<S19>/Saturate' */

  /* Sum: '<S11>/Add4' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add4 = rtb_euler_ref[1] - hilicopter_NDI_control_U.state[13];

  /* DiscreteIntegrator: '<S20>/Integrator' */
  if (rtb_Compare && (hilicopter_NDI_control_DW.Integrator_PrevResetState_h <= 0))
  {
    hilicopter_NDI_control_DW.Integrator_DSTATE_k = 0.0F;
  }

  /* Sum: '<S20>/Sum' incorporates:
   *  DiscreteIntegrator: '<S20>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S20>/POut'
   */
  rtb_Sum_m = rtb_Add4 * hilicopter_NDI_control_U.param[25] +
    hilicopter_NDI_control_DW.Integrator_DSTATE_k;

  /* Saturate: '<S20>/Saturate' */
  if (rtb_Sum_m > 5.0F) {
    rtb_Saturate_n = 5.0F;
  } else if (rtb_Sum_m < -5.0F) {
    rtb_Saturate_n = -5.0F;
  } else {
    rtb_Saturate_n = rtb_Sum_m;
  }

  /* End of Saturate: '<S20>/Saturate' */

  /* Sum: '<S11>/Add5' incorporates:
   *  Gain: '<S11>/Gain'
   *  Inport: '<Root>/state'
   */
  rtb_Add5 = -rtb_euler_ref[2] - hilicopter_NDI_control_U.state[14];

  /* RelationalOperator: '<S17>/Compare' incorporates:
   *  Constant: '<S17>/Constant'
   *  Inport: '<Root>/pwm_in'
   */
  rtb_Compare_f = (hilicopter_NDI_control_U.pwm_in[4] >= 0.5F);

  /* DiscreteIntegrator: '<S21>/Integrator' */
  if (rtb_Compare_f && (hilicopter_NDI_control_DW.Integrator_PrevResetState_i <=
                        0)) {
    hilicopter_NDI_control_DW.Integrator_DSTATE_ah = 35.0F;
  }

  /* Sum: '<S21>/Sum' incorporates:
   *  DiscreteIntegrator: '<S21>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S21>/POut'
   */
  rtb_Sum_j = rtb_Add5 * hilicopter_NDI_control_U.param[26] +
    hilicopter_NDI_control_DW.Integrator_DSTATE_ah;

  /* Saturate: '<S21>/Saturate' */
  if (rtb_Sum_j > 35.0F) {
    rtb_Saturate_a = 35.0F;
  } else if (rtb_Sum_j < -35.0F) {
    rtb_Saturate_a = -35.0F;
  } else {
    rtb_Saturate_a = rtb_Sum_j;
  }

  /* End of Saturate: '<S21>/Saturate' */

  /* MATLAB Function: '<Root>/vel_mode_logic' incorporates:
   *  SignalConversion: '<S12>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'vel_mode_logic': '<S12>:1' */
  /*  select commanded velocities according to mode */
  /*  Mode 1 = Stabilize */
  /*  Mode 2 = Velocity Control */
  if (rtb_mode == 2) {
    /* '<S12>:1:6' */
    /* '<S12>:1:7' */
    rtb_euler_ref[0] = rtb_Saturate;
    rtb_euler_ref[1] = rtb_Saturate_n;
    rtb_euler_ref[2] = rtb_Saturate_a;
  } else {
    /* '<S12>:1:9' */
    rtb_euler_ref[0] = 0.0F;
    rtb_euler_ref[1] = 0.0F;
    rtb_euler_ref[2] = rtb_Saturate_a;
  }

  /* End of MATLAB Function: '<Root>/vel_mode_logic' */

  /* MATLAB Function: '<Root>/NDI_control_law' incorporates:
   *  DiscreteIntegrator: '<S13>/Integrator'
   *  DiscreteIntegrator: '<S14>/Integrator'
   *  DiscreteIntegrator: '<S15>/Integrator'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S13>/POut'
   *  Product: '<S14>/POut'
   *  Product: '<S15>/POut'
   *  Sum: '<S13>/Sum'
   *  Sum: '<S14>/Sum'
   *  Sum: '<S15>/Sum'
   */
  /* MATLAB Function 'NDI_control_law': '<S1>:1' */
  /*  NDI controller for p,q,r and un,vn,wn of OC model */
  /*  Returns control input um = ["uroll; upitch; uyaw;" Fx; Fy; Fz] for given */
  /*  state and desired velocity v_ref and  omega_ref */
  /*  Revision history: */
  /*  2016|02|19: renamed some variables, updated comments */
  /* '<S1>:1:8' */
  /* '<S1>:1:9' */
  /*  +/-5 deg max angle for velocity */
  /*  Frequently used functions */
  /* '<S1>:1:14' */
  sphi = (real32_T)sin(hilicopter_NDI_control_U.state[9]);

  /* '<S1>:1:14' */
  cphi = (real32_T)cos(hilicopter_NDI_control_U.state[9]);

  /* '<S1>:1:15' */
  sthe = (real32_T)sin(hilicopter_NDI_control_U.state[10]);

  /* '<S1>:1:15' */
  cthe = (real32_T)cos(hilicopter_NDI_control_U.state[10]);

  /* '<S1>:1:16' */
  spsi = (real32_T)sin(hilicopter_NDI_control_U.state[11]);

  /* '<S1>:1:16' */
  cpsi = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  if ((real32_T)fabs(cthe) <= 1.0E-5F) {
    /* '<S1>:1:17' */
    /*  Hack to avoid singulariy at +/- pi/2 */
    /* '<S1>:1:18' */
    if (cthe < 0.0F) {
      cthe = -1.0F;
    } else {
      if (cthe > 0.0F) {
        cthe = 1.0F;
      }
    }

    cthe *= 1.0E-5F;
  }

  /*  Controlled variables */
  /*  h=[ un */
  /*      vn */
  /*      wn */
  /*      p  */
  /*      q  */
  /*      r  */
  /* '<S1>:1:28' */
  dhdx_fx[0] = cthe * cpsi;
  dhdx_fx[6] = sphi * sthe * cpsi - cphi * spsi;
  dhdx_fx[12] = cphi * sthe * cpsi + sphi * spsi;
  dhdx_fx[18] = 0.0F;
  dhdx_fx[24] = 0.0F;
  dhdx_fx[30] = 0.0F;
  dhdx_fx[36] = 0.0F;
  dhdx_fx[42] = 0.0F;
  dhdx_fx[48] = 0.0F;
  dhdx_fx[54] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * cpsi +
                  hilicopter_NDI_control_U.state[1] * sphi * spsi) -
                 hilicopter_NDI_control_U.state[2] * sphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * spsi;
  dhdx_fx[60] = (hilicopter_NDI_control_U.state[1] * sphi * cthe * cpsi +
                 -hilicopter_NDI_control_U.state[0] * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * cpsi;
  dhdx_fx[66] = (((-hilicopter_NDI_control_U.state[0] * cthe * spsi -
                   hilicopter_NDI_control_U.state[1] * sphi * sthe * spsi) -
                  hilicopter_NDI_control_U.state[1] * cphi * cpsi) -
                 hilicopter_NDI_control_U.state[2] * cphi * sthe * spsi) +
    hilicopter_NDI_control_U.state[2] * sphi * cpsi;
  dhdx_fx[1] = cthe * spsi;
  dhdx_fx[7] = sphi * sthe * spsi + cphi * cpsi;
  dhdx_fx[13] = cphi * sthe * spsi - sphi * cpsi;
  dhdx_fx[19] = 0.0F;
  dhdx_fx[25] = 0.0F;
  dhdx_fx[31] = 0.0F;
  dhdx_fx[37] = 0.0F;
  dhdx_fx[43] = 0.0F;
  dhdx_fx[49] = 0.0F;
  dhdx_fx[55] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * spsi -
                  hilicopter_NDI_control_U.state[1] * sphi * cpsi) -
                 hilicopter_NDI_control_U.state[2] * sphi * sthe * spsi) -
    hilicopter_NDI_control_U.state[2] * cphi * cpsi;
  dhdx_fx[61] = ((hilicopter_NDI_control_U.state[1] * sphi * cthe * spsi +
                  -hilicopter_NDI_control_U.state[0] * sthe * spsi) +
                 hilicopter_NDI_control_U.state[1] * cphi * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * spsi;
  dhdx_fx[67] = (((hilicopter_NDI_control_U.state[1] * sphi * sthe * cpsi +
                   hilicopter_NDI_control_U.state[0] * cthe * cpsi) -
                  hilicopter_NDI_control_U.state[1] * cphi * spsi) +
                 hilicopter_NDI_control_U.state[2] * cphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * sphi * spsi;
  dhdx_fx[2] = -sthe;
  dhdx_fx[8] = sphi * cthe;
  dhdx_fx[14] = cphi * cthe;
  dhdx_fx[20] = 0.0F;
  dhdx_fx[26] = 0.0F;
  dhdx_fx[32] = 0.0F;
  dhdx_fx[38] = 0.0F;
  dhdx_fx[44] = 0.0F;
  dhdx_fx[50] = 0.0F;
  dhdx_fx[56] = hilicopter_NDI_control_U.state[1] * cphi * cthe -
    hilicopter_NDI_control_U.state[2] * sphi * cthe;
  dhdx_fx[62] = (-hilicopter_NDI_control_U.state[0] * cthe -
                 hilicopter_NDI_control_U.state[1] * sthe * sphi) -
    hilicopter_NDI_control_U.state[2] * cphi * sthe;
  dhdx_fx[68] = 0.0F;
  for (i = 0; i < 12; i++) {
    dhdx_fx[3 + 6 * i] = d[i];
    dhdx_fx[4 + 6 * i] = c[i];
    dhdx_fx[5 + 6 * i] = b[i];
  }

  /* '<S1>:1:35' */
  if (hilicopter_NDI_control_U.state[9] >= -0.0873F) {
    cphi = hilicopter_NDI_control_U.state[9];
  } else {
    cphi = -0.0873F;
  }

  if (!(cphi <= 0.0873F)) {
    cphi = 0.0873F;
  }

  /* '<S1>:1:36' */
  if (hilicopter_NDI_control_U.state[10] >= -0.0873F) {
    cthe = hilicopter_NDI_control_U.state[10];
  } else {
    cthe = -0.0873F;
  }

  if (!(cthe <= 0.0873F)) {
    cthe = 0.0873F;
  }

  /*  Frequently used functions */
  /* '<S1>:1:39' */
  sphi = (real32_T)sin(cphi);

  /* '<S1>:1:39' */
  cphi = (real32_T)cos(cphi);

  /* '<S1>:1:40' */
  sthe = (real32_T)sin(cthe);

  /* '<S1>:1:40' */
  cthe = (real32_T)cos(cthe);

  /* '<S1>:1:41' */
  if ((real32_T)fabs(cthe) <= 1.0E-5F) {
    /* '<S1>:1:42' */
    /*  Hack to avoid singulariy at +/- pi/2 */
    /* '<S1>:1:43' */
    if (cthe < 0.0F) {
      cthe = -1.0F;
    } else if (cthe > 0.0F) {
      cthe = 1.0F;
    } else {
      cthe = 0.0F;
    }

    cthe *= 1.0E-5F;
  }

  /* '<S1>:1:46' */
  /* '<S1>:1:53' */
  /*  state part of state derivatives xdot = f(x)+g(x)*u */
  /*  Author: Bernd Messnarz, Last Revision: 2015|03|22 */
  /*  definition of states */
  /*  x =[u; v; w; x; y; z; p; q; r; phi; theta; psi] */
  /*  Kinematic velocity in body frame */
  /*  Angular velocity of the body in body frame */
  for (i = 0; i < 9; i++) {
    K_euler[i] = 0.0F;
    Ibi[i] = 0.0F;
  }

  K_euler[0] = hilicopter_NDI_control_U.param[36];
  K_euler[4] = hilicopter_NDI_control_U.param[37];
  K_euler[8] = hilicopter_NDI_control_U.param[38];
  Ibi[0] = 1.0F / hilicopter_NDI_control_U.param[36];
  Ibi[4] = 1.0F / hilicopter_NDI_control_U.param[37];
  Ibi[8] = 1.0F / hilicopter_NDI_control_U.param[38];

  /*  Initialization */
  memset(&xdot_s[0], 0, 12U * sizeof(real_T));

  /*  DCM to transform wind into body axes               */
  b_sphi = (real32_T)sin(hilicopter_NDI_control_U.state[9]);
  b_cphi = (real32_T)cos(hilicopter_NDI_control_U.state[9]);
  b_sthe = (real32_T)sin(hilicopter_NDI_control_U.state[10]);
  b_cthe = (real32_T)cos(hilicopter_NDI_control_U.state[10]);
  b_spsi = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  b_cpsi = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  if ((real32_T)fabs(b_cthe) <= 1.0E-5F) {
    /*  Hack to avoid singulariy at +/- pi/2 */
    if (b_cthe < 0.0F) {
      b_cthe = -1.0F;
    } else {
      if (b_cthe > 0.0F) {
        b_cthe = 1.0F;
      }
    }

    b_cthe *= 1.0E-5F;
  }

  tthe = b_sthe / b_cthe;
  Tbn[0] = b_cthe * b_cpsi;
  Tbn[3] = b_cthe * b_spsi;
  Tbn[6] = -b_sthe;
  Tbn[1] = b_sphi * b_sthe * b_cpsi - b_cphi * b_spsi;
  Tbn[4] = b_sphi * b_sthe * b_spsi + b_cphi * b_cpsi;
  Tbn[7] = b_sphi * b_cthe;
  Tbn[2] = b_cphi * b_sthe * b_cpsi + b_sphi * b_spsi;
  Tbn[5] = b_cphi * b_sthe * b_spsi - b_sphi * b_cpsi;
  Tbn[8] = b_cphi * b_cthe;

  /*  Body drag force and moment (damping) */
  /*  for drag force */
  /*  for drag moment */
  /*  Drag force */
  b_cpsi = 1.17549435E-38F;
  absxk = (real32_T)fabs(hilicopter_NDI_control_U.state[0]);
  if (absxk > 1.17549435E-38F) {
    b_spsi = 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / 1.17549435E-38F;
    b_spsi = t * t;
  }

  absxk = (real32_T)fabs(hilicopter_NDI_control_U.state[1]);
  if (absxk > b_cpsi) {
    t = b_cpsi / absxk;
    b_spsi = b_spsi * t * t + 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / b_cpsi;
    b_spsi += t * t;
  }

  absxk = (real32_T)fabs(hilicopter_NDI_control_U.state[2]);
  if (absxk > b_cpsi) {
    t = b_cpsi / absxk;
    b_spsi = b_spsi * t * t + 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / b_cpsi;
    b_spsi += t * t;
  }

  b_spsi = b_cpsi * (real32_T)sqrt(b_spsi);
  b_spsi *= -0.062F;

  /*  Drag moment */
  /*  Gravity in body frame */
  /*  EOM */
  /*  H_eulerdot_omega */
  /*  Force (drag, gravity and inertia) */
  xdot_s[0] = (b_spsi * hilicopter_NDI_control_U.state[0] / 3.5F + 9.80665F *
               -b_sthe) - (hilicopter_NDI_control_U.state[7] *
    hilicopter_NDI_control_U.state[2] - hilicopter_NDI_control_U.state[8] *
    hilicopter_NDI_control_U.state[1]);
  xdot_s[1] = (b_sphi * b_cthe * 9.80665F + b_spsi *
               hilicopter_NDI_control_U.state[1] / 3.5F) -
    (hilicopter_NDI_control_U.state[8] * hilicopter_NDI_control_U.state[0] -
     hilicopter_NDI_control_U.state[6] * hilicopter_NDI_control_U.state[2]);
  xdot_s[2] = (b_cphi * b_cthe * 9.80665F + b_spsi *
               hilicopter_NDI_control_U.state[2] / 3.5F) -
    (hilicopter_NDI_control_U.state[6] * hilicopter_NDI_control_U.state[1] -
     hilicopter_NDI_control_U.state[7] * hilicopter_NDI_control_U.state[0]);

  /*  Moment (drag, gravity and inertia) */
  for (i = 0; i < 3; i++) {
    tmp[i] = hilicopter_NDI_control_U.state[6 + i];
    rtb_TmpSignalConversionAtSFunct[i] = K_euler[i + 6] *
      hilicopter_NDI_control_U.state[8] + (K_euler[i + 3] *
      hilicopter_NDI_control_U.state[7] + K_euler[i] *
      hilicopter_NDI_control_U.state[6]);
  }

  b_spsi = -0.01F * tmp[0] - (hilicopter_NDI_control_U.state[7] *
    rtb_TmpSignalConversionAtSFunct[2] - hilicopter_NDI_control_U.state[8] *
    rtb_TmpSignalConversionAtSFunct[1]);
  b_sthe = -0.01F * tmp[1] - (hilicopter_NDI_control_U.state[8] *
    rtb_TmpSignalConversionAtSFunct[0] - hilicopter_NDI_control_U.state[6] *
    rtb_TmpSignalConversionAtSFunct[2]);
  b_cpsi = -0.01F * tmp[2] - (hilicopter_NDI_control_U.state[6] *
    rtb_TmpSignalConversionAtSFunct[1] - hilicopter_NDI_control_U.state[7] *
    rtb_TmpSignalConversionAtSFunct[0]);

  /*  Position */
  /*  Attitude */
  tmp_0[0] = 1.0F;
  tmp_0[3] = b_sphi * tthe;
  tmp_0[6] = b_cphi * tthe;
  tmp_0[1] = 0.0F;
  tmp_0[4] = b_cphi;
  tmp_0[7] = -b_sphi;
  tmp_0[2] = 0.0F;
  tmp_0[5] = b_sphi / b_cthe;
  tmp_0[8] = b_cphi / b_cthe;

  /* '<S1>:1:54' */
  /*  Control part of state derivatives xdot = f(x)+gmatrix(x)*um */
  /*  Author: Bernd Messnarz */
  /*  Revision History:  */
  /*  2015|03|22: using parameter structure in workspace */
  /*  2015|04|30: normalization of G matrix (un = n^2/n_trim^2) */
  /*  2015|11|05: introduction of new control inputs um (m ... mixed)  */
  /*  2016|02|11: change to octocopter configuration */
  /*  definition of states */
  /*  x =[u; v; w; x; y; z; p; q; r; phi; theta; psi] */
  /*  definition of control vector:  */
  /*  u = [u1; u2; u3; u4] * n2trim;   */
  /*  Kinematic velocity in body frame */
  /*  Angular velocity vector in body frame */
  /*  Data */
  /* L   = 0.25;      % Halfdistance between rotors, m */
  /* r1 = [L;0;0]; r2 = [0;L;0]; r3=[-L;0;0]; r4 = [0;-L;0]; % rotor pos. */
  /* kT  = 7e-7;      % RPM^2 to thrust factor, N min^2    */
  /* kMT = 0.0383;    % Thrust to moment conversion Factor, Nm/N   */
  /* kv  = 0.05;      % Thrust reduction factor, s/m; */
  /*  Mass and Inertia */
  /* mass = p.mass;        % Total mass, kg  */
  /*  Ixx  = 0.02; Iyy  = 0.02;  Izz  = 0.03;     % kgm^2 */
  /*  Inverse matrix of inertia */
  /* Ibi = [1/Ixx  0       0; */
  /*        0      1/Iyy   0; */
  /*        0      0    1/Izz];    */
  /*  DCM to tranform wind into body axes              */
  /*  Velocities of rotors with respect to air */
  /*  VA = VK - VW   */
  /*  define cross product matrix  */
  /*    v1 = VKb+cross(omegab,param.r1) - (VWb+cross(OWb,param.r1));   */
  /*    v2 = VKb+cross(omegab,param.r2) - (VWb+cross(OWb,param.r2)); */
  /*    v3 = VKb+cross(omegab,param.r3) - (VWb+cross(OWb,param.r3)); */
  /*    v4 = VKb+cross(omegab,param.r4) - (VWb+cross(OWb,param.r4)); */
  /*    v5 = VKb+cross(omegab,param.r1) - (VWb+cross(OWb,param.r1));   */
  /*    v6 = VKb+cross(omegab,param.r2) - (VWb+cross(OWb,param.r2)); */
  /*    v7 = VKb+cross(omegab,param.r3) - (VWb+cross(OWb,param.r3)); */
  /*    v8 = VKb+cross(omegab,param.r4) - (VWb+cross(OWb,param.r4)); */
  /*  geometry factor for diagonal rotors  */
  /*  Matrix for forces:  Frotb = Fmat * un;  */
  tmp_2[0] = hilicopter_NDI_control_U.state[0];
  tmp_2[1] = hilicopter_NDI_control_U.state[1];
  tmp_2[2] = hilicopter_NDI_control_U.state[2];
  tmp_3[0] = 0.0F;
  tmp_3[3] = -hilicopter_NDI_control_U.state[8];
  tmp_3[6] = hilicopter_NDI_control_U.state[7];
  tmp_3[1] = hilicopter_NDI_control_U.state[8];
  tmp_3[4] = 0.0F;
  tmp_3[7] = -hilicopter_NDI_control_U.state[6];
  tmp_3[2] = -hilicopter_NDI_control_U.state[7];
  tmp_3[5] = hilicopter_NDI_control_U.state[6];
  tmp_3[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    Ibi_0[i] = (Ibi[i + 3] * b_sthe + Ibi[i] * b_spsi) + Ibi[i + 6] * b_cpsi;
    xdot_s[6 + i] = Ibi_0[i];
    Tbn_0[i] = (Tbn[3 * i + 1] * hilicopter_NDI_control_U.state[1] + Tbn[3 * i] *
                hilicopter_NDI_control_U.state[0]) + Tbn[3 * i + 2] *
      hilicopter_NDI_control_U.state[2];
    xdot_s[3 + i] = Tbn_0[i];
    tmp_1[i] = (tmp_0[i + 3] * hilicopter_NDI_control_U.state[7] + tmp_0[i] *
                hilicopter_NDI_control_U.state[6]) + tmp_0[i + 6] *
      hilicopter_NDI_control_U.state[8];
    xdot_s[9 + i] = tmp_1[i];
    for (j = 0; j < 8; j++) {
      K_euler_0[i + 3 * j] = tmp_2[i];
      tmp_4[i + 3 * j] = 0.0F;
      tmp_4[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j] * tmp_3[i];
      tmp_4[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j + 1] * tmp_3[i + 3];
      tmp_4[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j + 2] * tmp_3[i + 6];
    }
  }

  /*  Rot  1        2     3        4     5         6     7          8 */
  /* Fxb */
  /* Fyb        */
  /* Fzb */
  /*  Matrix for moments:  Mrotb = Mmat * un;  */
  for (i = 0; i < 8; i++) {
    tmp_5[3 * i] = K_euler_0[3 * i] + tmp_4[3 * i];
    tmp_5[1 + 3 * i] = K_euler_0[3 * i + 1] + tmp_4[3 * i + 1];
    tmp_5[2 + 3 * i] = K_euler_0[3 * i + 2] + tmp_4[3 * i + 2];
    b_sphi = (tmp_5[3 * i + 2] * 0.05F + 1.0F) * 1.28E-7F;
    km[i] = 0.0383F * b_sphi;
    kL[i] = 0.45F * b_sphi;
    b_k[i] = b_sphi;
  }

  /*  Rot  1         2         3         4         5         6         7         8 */
  /*  Mxb */
  /*  Myb        */
  /*  Mzb   */
  /*  gmatrix dimension = 12 (outputs) x 6 (inputs um)  */
  /*  [u;v;w] dot    */
  /*  [x;y;z] dot             */
  /*  [p;q;r] dot */
  for (i = 0; i < 9; i++) {
    K_euler[i] = 0.0F;
  }

  K_euler[0] = 1.0F / hilicopter_NDI_control_U.param[36];
  K_euler[4] = 1.0F / hilicopter_NDI_control_U.param[37];
  K_euler[8] = 1.0F / hilicopter_NDI_control_U.param[38];

  /*  [phi;theta;psi] dot */
  /*  Normalization of gmatrix  */
  /*  gmatrix = gmatrix_n2*param.n2trim; */
  /* '<S1>:1:56' */
  /* '<S1>:1:58' */
  /*  NDI control law */
  /* '<S1>:1:61' */
  rtb_euler_ref_0[0] = rtb_euler_ref[0];
  rtb_euler_ref_0[1] = rtb_euler_ref[1];
  rtb_euler_ref_0[2] = rtb_euler_ref[2];
  rtb_euler_ref_0[3] = (rtb_Add * hilicopter_NDI_control_U.param[0] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE) + rtb_NOut;
  rtb_euler_ref_0[4] = (rtb_Add1 * hilicopter_NDI_control_U.param[1] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE_l) +
    rtb_NOut_m;
  rtb_euler_ref_0[5] = (rtb_Add2 * hilicopter_NDI_control_U.param[2] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE_a) +
    rtb_NOut_a;
  for (i = 0; i < 6; i++) {
    b_sphi = 0.0F;
    for (j = 0; j < 12; j++) {
      b_sphi += dhdx_fx[6 * j + i] * (real32_T)xdot_s[j];
    }

    tmp_6[i] = 0.0F - b_sphi;
    rtb_um[i] = tmp_6[i] + rtb_euler_ref_0[i];
  }

  b_k_0[0] = -b_k[0] * 0.42261827F;
  b_k_0[3] = 0.0F;
  b_k_0[6] = 0.0F;
  b_k_0[9] = 0.0F;
  b_k_0[12] = b_k[4] * 0.42261827F;
  b_k_0[15] = 0.0F;
  b_k_0[18] = 0.0F;
  b_k_0[21] = 0.0F;
  b_k_0[1] = 0.0F;
  b_k_0[4] = 0.0F;
  b_k_0[7] = -b_k[2] * 0.42261827F;
  b_k_0[10] = 0.0F;
  b_k_0[13] = 0.0F;
  b_k_0[16] = 0.0F;
  b_k_0[19] = b_k[6] * 0.42261827F;
  b_k_0[22] = 0.0F;
  b_k_0[2] = -b_k[0] * 0.906307817F;
  b_k_0[5] = -b_k[1];
  b_k_0[8] = -b_k[2] * 0.906307817F;
  b_k_0[11] = -b_k[3];
  b_k_0[14] = -b_k[4] * 0.906307817F;
  b_k_0[17] = -b_k[5];
  b_k_0[20] = -b_k[6] * 0.906307817F;
  b_k_0[23] = -b_k[7];
  km_0[0] = -km[0] * 0.42261827F;
  km_0[3] = -kL[1] * 0.707106769F;
  km_0[6] = -kL[2] * 0.906307817F;
  km_0[9] = -kL[3] * 0.707106769F;
  km_0[12] = km[4] * 0.42261827F;
  km_0[15] = kL[5] * 0.707106769F;
  km_0[18] = kL[6] * 0.906307817F;
  km_0[21] = kL[7] * 0.707106769F;
  km_0[1] = kL[0] * 0.906307817F;
  km_0[4] = kL[1] * 0.707106769F;
  km_0[7] = -km[2] * 0.42261827F;
  km_0[10] = -kL[3] * 0.707106769F;
  km_0[13] = -kL[4] * 0.906307817F;
  km_0[16] = -kL[5] * 0.707106769F;
  km_0[19] = km[6] * 0.42261827F;
  km_0[22] = kL[7] * 0.707106769F;
  km_0[2] = -km[0] * 0.906307817F;
  km_0[5] = km[1];
  km_0[8] = -km[2] * 0.906307817F;
  km_0[11] = km[3];
  km_0[14] = -km[4] * 0.906307817F;
  km_0[17] = km[5];
  km_0[20] = -km[6] * 0.906307817F;
  km_0[23] = km[7];
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 8; j++) {
      K_euler_0[i + 3 * j] = 0.0F;
      K_euler_0[i + 3 * j] += km_0[3 * j] * K_euler[i];
      K_euler_0[i + 3 * j] += km_0[3 * j + 1] * K_euler[i + 3];
      K_euler_0[i + 3 * j] += km_0[3 * j + 2] * K_euler[i + 6];
    }
  }

  for (i = 0; i < 8; i++) {
    b_k_1[12 * i] = b_k_0[3 * i] / 3.5F;
    b_k_1[1 + 12 * i] = b_k_0[3 * i + 1] / 3.5F;
    b_k_1[2 + 12 * i] = b_k_0[3 * i + 2] / 3.5F;
    b_k_1[3 + 12 * i] = 0.0F;
    b_k_1[4 + 12 * i] = 0.0F;
    b_k_1[5 + 12 * i] = 0.0F;
    b_k_1[6 + 12 * i] = K_euler_0[3 * i];
    b_k_1[7 + 12 * i] = K_euler_0[3 * i + 1];
    b_k_1[8 + 12 * i] = K_euler_0[3 * i + 2];
    b_k_1[9 + 12 * i] = 0.0F;
    b_k_1[10 + 12 * i] = 0.0F;
    b_k_1[11 + 12 * i] = 0.0F;
  }

  for (i = 0; i < 8; i++) {
    for (j = 0; j < 12; j++) {
      b_k_2[j + 12 * i] = b_k_1[12 * i + j] * 3.3518824E+7F;
    }
  }

  dhdx_fx[0] = cthe * cpsi;
  dhdx_fx[6] = sphi * sthe * cpsi - cphi * spsi;
  dhdx_fx[12] = cphi * sthe * cpsi + sphi * spsi;
  dhdx_fx[18] = 0.0F;
  dhdx_fx[24] = 0.0F;
  dhdx_fx[30] = 0.0F;
  dhdx_fx[36] = 0.0F;
  dhdx_fx[42] = 0.0F;
  dhdx_fx[48] = 0.0F;
  dhdx_fx[54] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * cpsi +
                  hilicopter_NDI_control_U.state[1] * sphi * spsi) -
                 hilicopter_NDI_control_U.state[2] * sphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * spsi;
  dhdx_fx[60] = (hilicopter_NDI_control_U.state[1] * sphi * cthe * cpsi +
                 -hilicopter_NDI_control_U.state[0] * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * cpsi;
  dhdx_fx[66] = (((-hilicopter_NDI_control_U.state[0] * cthe * spsi -
                   hilicopter_NDI_control_U.state[1] * sphi * sthe * spsi) -
                  hilicopter_NDI_control_U.state[1] * cphi * cpsi) -
                 hilicopter_NDI_control_U.state[2] * cphi * sthe * spsi) +
    hilicopter_NDI_control_U.state[2] * sphi * cpsi;
  dhdx_fx[1] = cthe * spsi;
  dhdx_fx[7] = sphi * sthe * spsi + cphi * cpsi;
  dhdx_fx[13] = cphi * sthe * spsi - sphi * cpsi;
  dhdx_fx[19] = 0.0F;
  dhdx_fx[25] = 0.0F;
  dhdx_fx[31] = 0.0F;
  dhdx_fx[37] = 0.0F;
  dhdx_fx[43] = 0.0F;
  dhdx_fx[49] = 0.0F;
  dhdx_fx[55] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * spsi -
                  hilicopter_NDI_control_U.state[1] * sphi * cpsi) -
                 hilicopter_NDI_control_U.state[2] * sphi * sthe * spsi) -
    hilicopter_NDI_control_U.state[2] * cphi * cpsi;
  dhdx_fx[61] = ((hilicopter_NDI_control_U.state[1] * sphi * cthe * spsi +
                  -hilicopter_NDI_control_U.state[0] * sthe * spsi) +
                 hilicopter_NDI_control_U.state[1] * cphi * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * spsi;
  dhdx_fx[67] = (((hilicopter_NDI_control_U.state[1] * sphi * sthe * cpsi +
                   hilicopter_NDI_control_U.state[0] * cthe * cpsi) -
                  hilicopter_NDI_control_U.state[1] * cphi * spsi) +
                 hilicopter_NDI_control_U.state[2] * cphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * sphi * spsi;
  dhdx_fx[2] = -sthe;
  dhdx_fx[8] = sphi * cthe;
  dhdx_fx[14] = cphi * cthe;
  dhdx_fx[20] = 0.0F;
  dhdx_fx[26] = 0.0F;
  dhdx_fx[32] = 0.0F;
  dhdx_fx[38] = 0.0F;
  dhdx_fx[44] = 0.0F;
  dhdx_fx[50] = 0.0F;
  dhdx_fx[56] = hilicopter_NDI_control_U.state[1] * cphi * cthe -
    hilicopter_NDI_control_U.state[2] * sphi * cthe;
  dhdx_fx[62] = (-hilicopter_NDI_control_U.state[0] * cthe -
                 hilicopter_NDI_control_U.state[1] * sthe * sphi) -
    hilicopter_NDI_control_U.state[2] * cphi * sthe;
  dhdx_fx[68] = 0.0F;
  for (i = 0; i < 12; i++) {
    dhdx_fx[3 + 6 * i] = d[i];
    dhdx_fx[4 + 6 * i] = c[i];
    dhdx_fx[5 + 6 * i] = b[i];
    for (j = 0; j < 6; j++) {
      b_k_3[i + 12 * j] = 0.0F;
      for (i_0 = 0; i_0 < 8; i_0++) {
        b_k_3[i + 12 * j] += b_k_2[12 * i_0 + i] * (real32_T)
          hilicopter_NDI_control_ConstP.pooled1.Tnm[(j << 3) + i_0];
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      cthe_0[i + 6 * j] = 0.0F;
      for (i_0 = 0; i_0 < 12; i_0++) {
        cthe_0[i + 6 * j] += dhdx_fx[6 * i_0 + i] * b_k_3[12 * j + i_0];
      }
    }
  }

  hilicopter_NDI_control_mldivide(cthe_0, rtb_um);

  /*  u = G^(-1)*[rdot - F + K*e]; */
  /* '<S1>:1:62' */
  if (rtb_um[0] < -4.0F) {
    sphi = -4.0F;
  } else {
    sphi = rtb_um[0];
  }

  if (sphi > 4.0F) {
    rtb_um[0] = 4.0F;
  } else {
    rtb_um[0] = sphi;
  }

  /* '<S1>:1:63' */
  if (rtb_um[1] < -4.0F) {
    sphi = -4.0F;
  } else {
    sphi = rtb_um[1];
  }

  if (sphi > 4.0F) {
    rtb_um[1] = 4.0F;
  } else {
    rtb_um[1] = sphi;
  }

  /* End of MATLAB Function: '<Root>/NDI_control_law' */

  /* MATLAB Function: '<Root>/um_mode_logic' */
  /*  Test = Max um F = +/-4 N for X and Y  Max M = 5 Nm for X and Y */
  /* MATLAB Function 'um_mode_logic': '<S10>:1' */
  /*  select commanded forces and moments according to mode */
  /*  Mode 1 = Stabilize */
  /*  Mode 2 = Velocity Control */
  if (rtb_mode == 2) {
    /* '<S10>:1:6' */
    /* '<S10>:1:7' */
    for (i = 0; i < 6; i++) {
      rtb_um_com[i] = rtb_um[i];
    }
  } else {
    /* '<S10>:1:9' */
    b_cpsi = 1.17549435E-38F;
    absxk = (real32_T)fabs(rtb_um[0]);
    if (absxk > 1.17549435E-38F) {
      b_spsi = 1.0F;
      b_cpsi = absxk;
    } else {
      t = absxk / 1.17549435E-38F;
      b_spsi = t * t;
    }

    absxk = (real32_T)fabs(rtb_um[1]);
    if (absxk > b_cpsi) {
      t = b_cpsi / absxk;
      b_spsi = b_spsi * t * t + 1.0F;
      b_cpsi = absxk;
    } else {
      t = absxk / b_cpsi;
      b_spsi += t * t;
    }

    absxk = (real32_T)fabs(rtb_um[2]);
    if (absxk > b_cpsi) {
      t = b_cpsi / absxk;
      b_spsi = b_spsi * t * t + 1.0F;
      b_cpsi = absxk;
    } else {
      t = absxk / b_cpsi;
      b_spsi += t * t;
    }

    b_spsi = b_cpsi * (real32_T)sqrt(b_spsi);

    /* '<S10>:1:10' */
    rtb_um_com[0] = 0.0F;
    rtb_um_com[1] = 0.0F;
    if (rtb_um[2] < 0.0F) {
      sphi = -1.0F;
    } else if (rtb_um[2] > 0.0F) {
      sphi = 1.0F;
    } else {
      sphi = rtb_um[2];
    }

    rtb_um_com[2] = sphi * b_spsi;
    rtb_um_com[3] = rtb_um[3];
    rtb_um_com[4] = rtb_um[4];
    rtb_um_com[5] = rtb_um[5];
  }

  /* End of MATLAB Function: '<Root>/um_mode_logic' */
  /* MATLAB Function 'um2pwm/un2n': '<S16>:1' */
  /* '<S16>:1:4' */
  /* '<S16>:1:5' */
  for (j = 0; j < 8; j++) {
    /* Gain: '<S9>/Tnm' incorporates:
     *  Saturate: '<S9>/Saturation 0 - unmax'
     */
    km[j] = 0.0F;
    for (i = 0; i < 6; i++) {
      km[j] += hilicopter_NDI_control_ConstP.Tnm_Gain[(i << 3) + j] *
        rtb_um_com[i];
    }

    /* Saturate: '<S9>/Saturation 0 - unmax' incorporates:
     *  Gain: '<S9>/Tnm'
     */
    if (km[j] > 5.0F) {
      kL[j] = 5.0F;
    } else if (km[j] < 0.0F) {
      kL[j] = 0.0F;
    } else {
      kL[j] = km[j];
    }

    /* MATLAB Function: '<S9>/un2n' */
    b_k[j] = kL[j];
    if (b_k[j] > 0.0F) {
      b_k[j] = 1.0F;
    }

    y[j] = kL[j];
    y[j] *= 3.3518824E+7F;
    y[j] = (real32_T)sqrt(y[j]);
    b_sphi = rt_roundf(b_k[j] * y[j] * 0.0939F + 1000.0F);
    if (b_sphi < 65536.0F) {
      u0 = (uint16_T)b_sphi;
    } else {
      u0 = MAX_uint16_T;
    }

    /* Saturate: '<Root>/Output_Limits2' incorporates:
     *  MATLAB Function: '<S9>/un2n'
     */
    if (u0 > 2000) {
      /* Outport: '<Root>/pwm_out' */
      hilicopter_NDI_control_Y.pwm_out[j] = 2000U;
    } else if (u0 < 1000) {
      /* Outport: '<Root>/pwm_out' */
      hilicopter_NDI_control_Y.pwm_out[j] = 1000U;
    } else {
      /* Outport: '<Root>/pwm_out' */
      hilicopter_NDI_control_Y.pwm_out[j] = u0;
    }

    /* End of Saturate: '<Root>/Output_Limits2' */
  }

  /* Outport: '<Root>/debug' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   */
  hilicopter_NDI_control_Y.debug[0] = rtb_omega_ref[0];
  hilicopter_NDI_control_Y.debug[1] = rtb_omega_ref[1];
  hilicopter_NDI_control_Y.debug[2] = rtb_Product;
  hilicopter_NDI_control_Y.debug[3] = rtb_Saturate;
  hilicopter_NDI_control_Y.debug[4] = rtb_Saturate_n;
  hilicopter_NDI_control_Y.debug[5] = rtb_Saturate_a;
  for (i = 0; i < 6; i++) {
    hilicopter_NDI_control_Y.debug[i + 6] = rtb_um[i];
  }

  hilicopter_NDI_control_Y.debug[12] = rtb_mode;

  /* End of Outport: '<Root>/debug' */

  /* Update for DiscreteIntegrator: '<S13>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S13>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE += rtb_Add *
    hilicopter_NDI_control_U.param[3] * 0.004F;

  /* Update for DiscreteIntegrator: '<S13>/Filter' */
  hilicopter_NDI_control_DW.Filter_DSTATE += 0.004F * rtb_NOut;

  /* Update for DiscreteIntegrator: '<S14>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S14>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_l += rtb_Add1 *
    hilicopter_NDI_control_U.param[4] * 0.004F;

  /* Update for DiscreteIntegrator: '<S14>/Filter' */
  hilicopter_NDI_control_DW.Filter_DSTATE_o += 0.004F * rtb_NOut_m;

  /* Update for DiscreteIntegrator: '<S15>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S15>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_a += rtb_Add2 *
    hilicopter_NDI_control_U.param[5] * 0.004F;

  /* Update for DiscreteIntegrator: '<S15>/Filter' */
  hilicopter_NDI_control_DW.Filter_DSTATE_oj += 0.004F * rtb_NOut_a;

  /* Update for DiscreteIntegrator: '<S19>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S19>/IOut'
   *  Sum: '<S19>/SumI1'
   *  Sum: '<S19>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_m += (rtb_Add3 *
    hilicopter_NDI_control_U.param[27] + (rtb_Saturate - rtb_Sum_d)) * 0.004F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState = (int8_T)rtb_Compare;

  /* Update for DiscreteIntegrator: '<S20>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S20>/IOut'
   *  Sum: '<S20>/SumI1'
   *  Sum: '<S20>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_k += (rtb_Add4 *
    hilicopter_NDI_control_U.param[28] + (rtb_Saturate_n - rtb_Sum_m)) * 0.004F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState_h = (int8_T)rtb_Compare;

  /* Update for DiscreteIntegrator: '<S21>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S21>/IOut'
   *  Sum: '<S21>/SumI1'
   *  Sum: '<S21>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_ah += (rtb_Add5 *
    hilicopter_NDI_control_U.param[29] + (rtb_Saturate_a - rtb_Sum_j)) * 0.004F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState_i = (int8_T)rtb_Compare_f;
}

/* Model initialize function */
void hilicopter_NDI_control_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(hilicopter_NDI_control_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&hilicopter_NDI_control_DW, 0,
                sizeof(DW_hilicopter_NDI_control_T));

  /* external inputs */
  (void) memset((void *)&hilicopter_NDI_control_U, 0,
                sizeof(ExtU_hilicopter_NDI_control_T));

  /* external outputs */
  (void) memset((void *)&hilicopter_NDI_control_Y, 0,
                sizeof(ExtY_hilicopter_NDI_control_T));

  /* InitializeConditions for DiscreteIntegrator: '<S19>/Integrator' */
  hilicopter_NDI_control_DW.Integrator_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S20>/Integrator' */
  hilicopter_NDI_control_DW.Integrator_PrevResetState_h = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S21>/Integrator' */
  hilicopter_NDI_control_DW.Integrator_DSTATE_ah = 35.0F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState_i = 2;
}

/* Model terminate function */
void hilicopter_NDI_control_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
