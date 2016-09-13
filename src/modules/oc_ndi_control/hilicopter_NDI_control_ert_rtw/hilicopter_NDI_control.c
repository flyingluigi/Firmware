/*
 * File: hilicopter_NDI_control.c
 *
 * Code generated for Simulink model 'hilicopter_NDI_control'.
 *
 * Model version                  : 1.370
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Sat Sep 10 17:04:28 2016
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

/* Model step function */
void hilicopter_NDI_control_step(void)
{
  real32_T sphi;
  real32_T cphi;
  real32_T sthe;
  real32_T cthe;
  real32_T spsi;
  real32_T cpsi;
  real32_T dhdx_fx[72];
  real_T xdot_s[12];
  real32_T Ib[9];
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
  real32_T Mmat[24];
  static const int8_T b[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  static const int8_T c[12] = { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const int8_T d[12] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 };

  real32_T absxk;
  real32_T t;
  real32_T y[8];
  real32_T rtb_euler_ref[3];
  real32_T rtb_Add;
  real32_T rtb_Sum;
  real32_T rtb_error_n;
  real32_T rtb_NOut;
  real32_T rtb_Sum_j;
  real32_T rtb_Add1;
  real32_T rtb_NOut_a;
  real32_T rtb_error_k;
  real32_T rtb_NOut_bl;
  real32_T rtb_error_c;
  real32_T rtb_Product;
  real32_T rtb_NOut_b;
  real32_T rtb_Sum2_e;
  real32_T rtb_error_l;
  real32_T rtb_NOut_o;
  uint8_T rtb_mode;
  boolean_T rtb_Compare;
  real32_T rtb_NOut_g;
  real32_T rtb_Saturation;
  real32_T rtb_NOut_j;
  real32_T rtb_Saturation_k;
  boolean_T rtb_Compare_f;
  real32_T rtb_NOut_jg;
  real32_T rtb_Saturation_o;
  real32_T rtb_um_com[6];
  real32_T rtb_TmpSignalConversionAtSFunct[3];
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
  real32_T Ib_0[24];
  real32_T b_k_1[96];
  real32_T b_k_2[96];
  real32_T b_k_3[72];
  int32_T i_0;
  int32_T j;

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
  } else {
    /* '<S4>:1:9' */
    rtb_euler_ref[0] = hilicopter_NDI_control_U.param[40] *
      hilicopter_NDI_control_U.pwm_in[0];
    rtb_euler_ref[1] = hilicopter_NDI_control_U.param[40] *
      hilicopter_NDI_control_U.pwm_in[1];
  }

  /* End of MATLAB Function: '<Root>/mode_logic' */

  /* Sum: '<S2>/Add' incorporates:
   *  Gain: '<S2>/+-5 deg offset'
   *  Inport: '<Root>/pwm_in'
   *  Inport: '<Root>/state'
   */
  rtb_Add = (0.0873F * hilicopter_NDI_control_U.pwm_in[6] + rtb_euler_ref[0]) -
    hilicopter_NDI_control_U.state[9];

  /* Product: '<S15>/NOut' incorporates:
   *  DiscreteIntegrator: '<S15>/Filter'
   *  Inport: '<Root>/param'
   *  Product: '<S15>/DOut'
   *  Sum: '<S15>/SumD'
   */
  rtb_NOut = (rtb_Add * hilicopter_NDI_control_U.param[18] -
              hilicopter_NDI_control_DW.Filter_DSTATE) *
    hilicopter_NDI_control_U.param[21];

  /* Sum: '<S15>/Sum' incorporates:
   *  DiscreteIntegrator: '<S15>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S15>/POut'
   */
  rtb_Sum = (rtb_Add * hilicopter_NDI_control_U.param[12] +
             hilicopter_NDI_control_DW.Integrator_DSTATE) + rtb_NOut;

  /* Sum: '<S23>/error' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_error_n = rtb_Sum - hilicopter_NDI_control_U.state[6];

  /* Product: '<S23>/NOut' incorporates:
   *  DiscreteIntegrator: '<S23>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S23>/Dout'
   *  Sum: '<S23>/Sum1'
   */
  rtb_NOut_a = ((0.0F - hilicopter_NDI_control_U.state[6] *
                 hilicopter_NDI_control_U.param[6]) -
                hilicopter_NDI_control_DW.FilterState_DSTATE) *
    hilicopter_NDI_control_U.param[9];

  /* Sum: '<S2>/Add1' incorporates:
   *  Gain: '<S2>/+-5 deg offset'
   *  Inport: '<Root>/pwm_in'
   *  Inport: '<Root>/state'
   */
  rtb_Add1 = (0.0873F * hilicopter_NDI_control_U.pwm_in[7] + rtb_euler_ref[1]) -
    hilicopter_NDI_control_U.state[10];

  /* Product: '<S16>/NOut' incorporates:
   *  DiscreteIntegrator: '<S16>/Filter'
   *  Inport: '<Root>/param'
   *  Product: '<S16>/DOut'
   *  Sum: '<S16>/SumD'
   */
  rtb_NOut_bl = (rtb_Add1 * hilicopter_NDI_control_U.param[19] -
                 hilicopter_NDI_control_DW.Filter_DSTATE_a) *
    hilicopter_NDI_control_U.param[22];

  /* Sum: '<S16>/Sum' incorporates:
   *  DiscreteIntegrator: '<S16>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S16>/POut'
   */
  rtb_Sum_j = (rtb_Add1 * hilicopter_NDI_control_U.param[13] +
               hilicopter_NDI_control_DW.Integrator_DSTATE_n) + rtb_NOut_bl;

  /* Sum: '<S24>/error' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_error_k = rtb_Sum_j - hilicopter_NDI_control_U.state[7];

  /* Product: '<S24>/NOut' incorporates:
   *  DiscreteIntegrator: '<S24>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S24>/Dout'
   *  Sum: '<S24>/Sum1'
   */
  rtb_NOut_b = ((0.0F - hilicopter_NDI_control_U.state[7] *
                 hilicopter_NDI_control_U.param[7]) -
                hilicopter_NDI_control_DW.FilterState_DSTATE_k) *
    hilicopter_NDI_control_U.param[10];

  /* Product: '<Root>/Product' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  rtb_Product = hilicopter_NDI_control_U.pwm_in[3] *
    hilicopter_NDI_control_U.param[39];

  /* Sum: '<S25>/error' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_error_c = rtb_Product - hilicopter_NDI_control_U.state[8];

  /* Product: '<S25>/NOut' incorporates:
   *  DiscreteIntegrator: '<S25>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S25>/Dout'
   *  Sum: '<S25>/Sum1'
   */
  rtb_NOut_o = ((0.0F - hilicopter_NDI_control_U.state[8] *
                 hilicopter_NDI_control_U.param[8]) -
                hilicopter_NDI_control_DW.FilterState_DSTATE_n) *
    hilicopter_NDI_control_U.param[11];

  /* MATLAB Function: '<Root>/rotate_u1' incorporates:
   *  Inport: '<Root>/state'
   */
  /* MATLAB Function 'rotate_u1': '<S8>:1' */
  /* Rotate commanded velocities according to heading */
  /* '<S8>:1:4' */
  /* '<S8>:1:8' */
  Ib[0] = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  Ib[3] = -(real32_T)sin(hilicopter_NDI_control_U.state[11]);
  Ib[6] = 0.0F;
  Ib[1] = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  Ib[4] = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  Ib[7] = 0.0F;
  Ib[2] = 0.0F;
  Ib[5] = 0.0F;
  Ib[8] = 1.0F;

  /* SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  Fcn: '<S9>/Fcn'
   *  Gain: '<Root>/gain1'
   *  Gain: '<Root>/max z velocity2'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  MATLAB Function: '<Root>/rotate_u1'
   *  Product: '<Root>/Product4'
   *  Product: '<Root>/Product5'
   */
  b_spsi = hilicopter_NDI_control_U.param[41] *
    -hilicopter_NDI_control_U.pwm_in[1];
  b_sthe = hilicopter_NDI_control_U.param[41] * hilicopter_NDI_control_U.pwm_in
    [0];
  b_cpsi = -((hilicopter_NDI_control_U.pwm_in[2] - 0.5F) * 2.0F);

  /* MATLAB Function: '<Root>/rotate_u1' */
  for (i = 0; i < 3; i++) {
    rtb_euler_ref[i] = Ib[i + 6] * b_cpsi + (Ib[i + 3] * b_sthe + Ib[i] * b_spsi);
  }

  /* RelationalOperator: '<S31>/Compare' incorporates:
   *  Constant: '<S31>/Constant'
   */
  rtb_Compare = (rtb_mode == 2);

  /* DiscreteIntegrator: '<S29>/Filter State' */
  if (rtb_Compare && (hilicopter_NDI_control_DW.FilterState_PrevResetState <= 0))
  {
    hilicopter_NDI_control_DW.FilterState_DSTATE_l = 0.0F;
  }

  /* Product: '<S29>/NOut' incorporates:
   *  DiscreteIntegrator: '<S29>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S29>/Dout'
   *  Sum: '<S29>/Sum1'
   */
  rtb_NOut_g = ((0.0F - hilicopter_NDI_control_U.state[12] *
                 hilicopter_NDI_control_U.param[30]) -
                hilicopter_NDI_control_DW.FilterState_DSTATE_l) *
    hilicopter_NDI_control_U.param[33];

  /* Sum: '<S29>/Sum2' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S29>/Pout'
   *  Sum: '<S29>/error'
   */
  rtb_Saturation = (rtb_euler_ref[0] - hilicopter_NDI_control_U.state[12]) *
    hilicopter_NDI_control_U.param[24] + rtb_NOut_g;

  /* Saturate: '<S29>/Saturation' */
  if (rtb_Saturation > 1.2F) {
    rtb_Saturation = 1.2F;
  } else {
    if (rtb_Saturation < -1.2F) {
      rtb_Saturation = -1.2F;
    }
  }

  /* End of Saturate: '<S29>/Saturation' */

  /* DiscreteIntegrator: '<S28>/Filter State' */
  if (rtb_Compare && (hilicopter_NDI_control_DW.FilterState_PrevResetState_h <=
                      0)) {
    hilicopter_NDI_control_DW.FilterState_DSTATE_h = 0.0F;
  }

  /* Product: '<S28>/NOut' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S28>/Dout'
   *  Sum: '<S28>/Sum1'
   */
  rtb_NOut_j = ((0.0F - hilicopter_NDI_control_U.state[13] *
                 hilicopter_NDI_control_U.param[31]) -
                hilicopter_NDI_control_DW.FilterState_DSTATE_h) *
    hilicopter_NDI_control_U.param[34];

  /* Sum: '<S28>/Sum2' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S28>/Pout'
   *  Sum: '<S28>/error'
   */
  rtb_Saturation_k = (rtb_euler_ref[1] - hilicopter_NDI_control_U.state[13]) *
    hilicopter_NDI_control_U.param[25] + rtb_NOut_j;

  /* Saturate: '<S28>/Saturation' */
  if (rtb_Saturation_k > 1.2F) {
    rtb_Saturation_k = 1.2F;
  } else {
    if (rtb_Saturation_k < -1.2F) {
      rtb_Saturation_k = -1.2F;
    }
  }

  /* End of Saturate: '<S28>/Saturation' */

  /* Sum: '<S27>/error' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_error_l = rtb_euler_ref[2] - hilicopter_NDI_control_U.state[14];

  /* RelationalOperator: '<S30>/Compare' incorporates:
   *  Constant: '<S30>/Constant'
   *  Inport: '<Root>/pwm_in'
   */
  rtb_Compare_f = (hilicopter_NDI_control_U.pwm_in[4] >= 0.5F);

  /* DiscreteIntegrator: '<S27>/Integrator' */
  if (rtb_Compare_f && (hilicopter_NDI_control_DW.Integrator_PrevResetState <= 0))
  {
    hilicopter_NDI_control_DW.Integrator_DSTATE_ig = 15.0F;
  }

  /* DiscreteIntegrator: '<S27>/Filter State' */
  if (rtb_Compare_f && (hilicopter_NDI_control_DW.FilterState_PrevResetState_hn <=
                        0)) {
    hilicopter_NDI_control_DW.FilterState_DSTATE_lx = 0.0F;
  }

  /* Product: '<S27>/NOut' incorporates:
   *  DiscreteIntegrator: '<S27>/Filter State'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S27>/Dout'
   *  Sum: '<S27>/Sum1'
   */
  rtb_NOut_jg = ((0.0F - hilicopter_NDI_control_U.state[14] *
                  hilicopter_NDI_control_U.param[32]) -
                 hilicopter_NDI_control_DW.FilterState_DSTATE_lx) *
    hilicopter_NDI_control_U.param[35];

  /* Sum: '<S27>/Sum2' incorporates:
   *  DiscreteIntegrator: '<S27>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S27>/Pout'
   */
  rtb_Sum2_e = (rtb_error_l * hilicopter_NDI_control_U.param[26] +
                hilicopter_NDI_control_DW.Integrator_DSTATE_ig) + rtb_NOut_jg;

  /* Saturate: '<S27>/Saturation' */
  if (rtb_Sum2_e > 15.0F) {
    rtb_Saturation_o = 15.0F;
  } else if (rtb_Sum2_e < -30.0F) {
    rtb_Saturation_o = -30.0F;
  } else {
    rtb_Saturation_o = rtb_Sum2_e;
  }

  /* End of Saturate: '<S27>/Saturation' */

  /* MATLAB Function: '<Root>/vel_mode_logic' incorporates:
   *  SignalConversion: '<S14>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'vel_mode_logic': '<S14>:1' */
  /*  select commanded velocities according to mode */
  /*  Mode 1 = Stabilize */
  /*  Mode 2 = Velocity Control */
  if (rtb_mode == 2) {
    /* '<S14>:1:6' */
    /* '<S14>:1:7' */
    rtb_euler_ref[0] = rtb_Saturation;
    rtb_euler_ref[1] = rtb_Saturation_k;
    rtb_euler_ref[2] = rtb_Saturation_o;
  } else {
    /* '<S14>:1:9' */
    rtb_euler_ref[0] = 0.0F;
    rtb_euler_ref[1] = 0.0F;
    rtb_euler_ref[2] = rtb_Saturation_o;
  }

  /* End of MATLAB Function: '<Root>/vel_mode_logic' */

  /* MATLAB Function: '<Root>/NDI_control_law' incorporates:
   *  DiscreteIntegrator: '<S23>/Integrator'
   *  DiscreteIntegrator: '<S24>/Integrator'
   *  DiscreteIntegrator: '<S25>/Integrator'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S23>/Pout'
   *  Product: '<S24>/Pout'
   *  Product: '<S25>/Pout'
   *  Sum: '<S23>/Sum2'
   *  Sum: '<S24>/Sum2'
   *  Sum: '<S25>/Sum2'
   */
  /* MATLAB Function 'NDI_control_law': '<S1>:1' */
  /*  NDI controller for p,q,r and un,vn,wn of OC model */
  /*  Returns control input um = ["uroll; upitch; uyaw;" Fx; Fy; Fz] for given */
  /*  state and desired velocity v_ref and  omega_ref */
  /*  Revision history: */
  /*  2016|02|19: renamed some variables, updated comments */
  /* '<S1>:1:8' */
  /* '<S1>:1:9' */
  /*  +/-5 deg 0.0873 rad max angle for velocity */
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
  if (hilicopter_NDI_control_U.state[9] >= -0.0524F) {
    cphi = hilicopter_NDI_control_U.state[9];
  } else {
    cphi = -0.0524F;
  }

  if (!(cphi <= 0.0524F)) {
    cphi = 0.0524F;
  }

  /* '<S1>:1:36' */
  if (hilicopter_NDI_control_U.state[10] >= -0.0524F) {
    cthe = hilicopter_NDI_control_U.state[10];
  } else {
    cthe = -0.0524F;
  }

  if (!(cthe <= 0.0524F)) {
    cthe = 0.0524F;
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
    Ib[i] = 0.0F;
    Ibi[i] = 0.0F;
  }

  Ib[0] = hilicopter_NDI_control_U.param[42];
  Ib[4] = hilicopter_NDI_control_U.param[43];
  Ib[8] = hilicopter_NDI_control_U.param[44];
  Ibi[0] = 1.0F / hilicopter_NDI_control_U.param[42];
  Ibi[4] = 1.0F / hilicopter_NDI_control_U.param[43];
  Ibi[8] = 1.0F / hilicopter_NDI_control_U.param[44];

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
    rtb_TmpSignalConversionAtSFunct[i] = Ib[i + 6] *
      hilicopter_NDI_control_U.state[8] + (Ib[i + 3] *
      hilicopter_NDI_control_U.state[7] + Ib[i] *
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
      Ib_0[i + 3 * j] = tmp_2[i];
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
    tmp_5[3 * i] = Ib_0[3 * i] + tmp_4[3 * i];
    tmp_5[1 + 3 * i] = Ib_0[3 * i + 1] + tmp_4[3 * i + 1];
    tmp_5[2 + 3 * i] = Ib_0[3 * i + 2] + tmp_4[3 * i + 2];
    b_sphi = (tmp_5[3 * i + 2] * 0.05F + 1.0F) * 1.28E-7F;
    km[i] = 0.0383F * b_sphi;
    kL[i] = 0.45F * b_sphi;
    b_k[i] = b_sphi;
  }

  /*  Rot  1         2         3         4         5         6         7         8 */
  /*  bialternating configuration ++--++--++-- */
  /*  Rot  1         2         3         4         5         6         7         8 */
  /*  Mxb */
  /*  Myb        */
  Mmat[0] = -km[0] * 0.642787635F;
  Mmat[3] = -kL[1] * 0.707106769F;
  Mmat[6] = -kL[2] * 0.766044438F;
  Mmat[9] = -kL[3] * 0.707106769F;
  Mmat[12] = km[4] * 0.642787635F;
  Mmat[15] = kL[5] * 0.707106769F;
  Mmat[18] = kL[6] * 0.766044438F;
  Mmat[21] = kL[7] * 0.707106769F;
  Mmat[1] = kL[0] * 0.766044438F;
  Mmat[4] = kL[1] * 0.707106769F;
  Mmat[7] = km[2] * 0.642787635F;
  Mmat[10] = -kL[3] * 0.707106769F;
  Mmat[13] = -kL[4] * 0.766044438F;
  Mmat[16] = -kL[5] * 0.707106769F;
  Mmat[19] = -km[6] * 0.642787635F;
  Mmat[22] = kL[7] * 0.707106769F;
  Mmat[2] = -km[0] * 0.766044438F;
  Mmat[5] = -km[1];
  Mmat[8] = km[2] * 0.766044438F;
  Mmat[11] = km[3];
  Mmat[14] = -km[4] * 0.766044438F;
  Mmat[17] = -km[5];
  Mmat[20] = km[6] * 0.766044438F;
  Mmat[23] = km[7];

  /*  Mzb    */
  /*  gmatrix dimension = 12 (outputs) x 6 (inputs um)  */
  /*  [u;v;w] dot    */
  /*  [x;y;z] dot             */
  /*  [p;q;r] dot */
  for (i = 0; i < 9; i++) {
    Ib[i] = 0.0F;
  }

  Ib[0] = 1.0F / hilicopter_NDI_control_U.param[42];
  Ib[4] = 1.0F / hilicopter_NDI_control_U.param[43];
  Ib[8] = 1.0F / hilicopter_NDI_control_U.param[44];

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
  rtb_euler_ref_0[3] = (rtb_error_n * hilicopter_NDI_control_U.param[0] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE_f) +
    rtb_NOut_a;
  rtb_euler_ref_0[4] = (rtb_error_k * hilicopter_NDI_control_U.param[1] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE_i) +
    rtb_NOut_b;
  rtb_euler_ref_0[5] = (rtb_error_c * hilicopter_NDI_control_U.param[2] +
                        hilicopter_NDI_control_DW.Integrator_DSTATE_g) +
    rtb_NOut_o;
  for (i = 0; i < 6; i++) {
    b_sphi = 0.0F;
    for (j = 0; j < 12; j++) {
      b_sphi += dhdx_fx[6 * j + i] * (real32_T)xdot_s[j];
    }

    tmp_6[i] = 0.0F - b_sphi;
    rtb_um[i] = tmp_6[i] + rtb_euler_ref_0[i];
  }

  b_k_0[0] = -b_k[0] * 0.642787635F;
  b_k_0[3] = 0.0F;
  b_k_0[6] = 0.0F;
  b_k_0[9] = 0.0F;
  b_k_0[12] = b_k[4] * 0.642787635F;
  b_k_0[15] = 0.0F;
  b_k_0[18] = 0.0F;
  b_k_0[21] = 0.0F;
  b_k_0[1] = 0.0F;
  b_k_0[4] = 0.0F;
  b_k_0[7] = -b_k[2] * 0.642787635F;
  b_k_0[10] = 0.0F;
  b_k_0[13] = 0.0F;
  b_k_0[16] = 0.0F;
  b_k_0[19] = b_k[6] * 0.642787635F;
  b_k_0[22] = 0.0F;
  b_k_0[2] = -b_k[0] * 0.766044438F;
  b_k_0[5] = -b_k[1];
  b_k_0[8] = -b_k[2] * 0.766044438F;
  b_k_0[11] = -b_k[3];
  b_k_0[14] = -b_k[4] * 0.766044438F;
  b_k_0[17] = -b_k[5];
  b_k_0[20] = -b_k[6] * 0.766044438F;
  b_k_0[23] = -b_k[7];
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 8; j++) {
      Ib_0[i + 3 * j] = 0.0F;
      Ib_0[i + 3 * j] += Mmat[3 * j] * Ib[i];
      Ib_0[i + 3 * j] += Mmat[3 * j + 1] * Ib[i + 3];
      Ib_0[i + 3 * j] += Mmat[3 * j + 2] * Ib[i + 6];
    }
  }

  for (i = 0; i < 8; i++) {
    b_k_1[12 * i] = b_k_0[3 * i] / 3.5F;
    b_k_1[1 + 12 * i] = b_k_0[3 * i + 1] / 3.5F;
    b_k_1[2 + 12 * i] = b_k_0[3 * i + 2] / 3.5F;
    b_k_1[3 + 12 * i] = 0.0F;
    b_k_1[4 + 12 * i] = 0.0F;
    b_k_1[5 + 12 * i] = 0.0F;
    b_k_1[6 + 12 * i] = Ib_0[3 * i];
    b_k_1[7 + 12 * i] = Ib_0[3 * i + 1];
    b_k_1[8 + 12 * i] = Ib_0[3 * i + 2];
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
  if (rtb_um[0] < -5.0F) {
    sphi = -5.0F;
  } else {
    sphi = rtb_um[0];
  }

  if (sphi > 5.0F) {
    rtb_um[0] = 5.0F;
  } else {
    rtb_um[0] = sphi;
  }

  /* '<S1>:1:63' */
  if (rtb_um[1] < -5.0F) {
    sphi = -5.0F;
  } else {
    sphi = rtb_um[1];
  }

  if (sphi > 5.0F) {
    rtb_um[1] = 5.0F;
  } else {
    rtb_um[1] = sphi;
  }

  /* End of MATLAB Function: '<Root>/NDI_control_law' */

  /* MATLAB Function: '<Root>/um_mode_logic' */
  /*  Test = Max um F = +/-4 N for X and Y  Max M = 5 Nm for X and Y */
  /* MATLAB Function 'um_mode_logic': '<S12>:1' */
  /*  select commanded forces and moments according to mode */
  /*  Mode 1 = Stabilize */
  /*  Mode 2 = Velocity Control */
  if (rtb_mode == 2) {
    /* '<S12>:1:6' */
    /* '<S12>:1:7' */
    for (i = 0; i < 6; i++) {
      rtb_um_com[i] = rtb_um[i];
    }
  } else {
    /* '<S12>:1:9' */
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

    /* '<S12>:1:10' */
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
  /* MATLAB Function 'um2pwm/un2n': '<S26>:1' */
  /* '<S26>:1:4' */
  /* pwm = uint16(param.rpm2pwm.*n + 1000); */
  /* '<S26>:1:6' */
  for (j = 0; j < 8; j++) {
    /* Gain: '<S11>/Tnm' incorporates:
     *  Saturate: '<S11>/Saturation 0 - unmax'
     */
    km[j] = 0.0F;
    for (i = 0; i < 6; i++) {
      km[j] += hilicopter_NDI_control_ConstP.Tnm_Gain[(i << 3) + j] *
        rtb_um_com[i];
    }

    /* Saturate: '<S11>/Saturation 0 - unmax' incorporates:
     *  Gain: '<S11>/Tnm'
     */
    if (km[j] > 5.0F) {
      kL[j] = 5.0F;
    } else if (km[j] < 0.0F) {
      kL[j] = 0.0F;
    } else {
      kL[j] = km[j];
    }

    /* MATLAB Function: '<S11>/un2n' */
    b_k[j] = kL[j];
    if (b_k[j] > 0.0F) {
      b_k[j] = 1.0F;
    }

    y[j] = kL[j];
    y[j] *= 3.3518824E+7F;
    y[j] = (real32_T)sqrt(y[j]);
    b_k[j] = b_k[j] * y[j] * 0.0939F + 1000.0F;

    /* End of MATLAB Function: '<S11>/un2n' */

    /* Saturate: '<Root>/Output_Limits2' */
    if (b_k[j] > 2000.0F) {
      sphi = 2000.0F;
    } else if (b_k[j] < 1200.0F) {
      sphi = 1200.0F;
    } else {
      sphi = b_k[j];
    }

    /* Outport: '<Root>/pwm_out' incorporates:
     *  Saturate: '<Root>/Output_Limits2'
     */
    hilicopter_NDI_control_Y.pwm_out[j] = (uint16_T)(real32_T)fmod((real32_T)
      floor(sphi), 65536.0F);
  }

  /* Outport: '<Root>/debug' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   */
  hilicopter_NDI_control_Y.debug[0] = rtb_Sum;
  hilicopter_NDI_control_Y.debug[1] = rtb_Sum_j;
  hilicopter_NDI_control_Y.debug[2] = rtb_Product;
  hilicopter_NDI_control_Y.debug[3] = rtb_Saturation;
  hilicopter_NDI_control_Y.debug[4] = rtb_Saturation_k;
  hilicopter_NDI_control_Y.debug[5] = rtb_Saturation_o;
  for (i = 0; i < 6; i++) {
    hilicopter_NDI_control_Y.debug[i + 6] = rtb_um[i];
  }

  hilicopter_NDI_control_Y.debug[12] = rtb_mode;

  /* End of Outport: '<Root>/debug' */

  /* Update for DiscreteIntegrator: '<S15>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S15>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE += rtb_Add *
    hilicopter_NDI_control_U.param[15] * 0.004F;

  /* Update for DiscreteIntegrator: '<S15>/Filter' */
  hilicopter_NDI_control_DW.Filter_DSTATE += 0.004F * rtb_NOut;

  /* Update for DiscreteIntegrator: '<S23>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S23>/Iout'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_f += rtb_error_n *
    hilicopter_NDI_control_U.param[3] * 0.004F;

  /* Update for DiscreteIntegrator: '<S23>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE += 0.004F * rtb_NOut_a;

  /* Update for DiscreteIntegrator: '<S16>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S16>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_n += rtb_Add1 *
    hilicopter_NDI_control_U.param[16] * 0.004F;

  /* Update for DiscreteIntegrator: '<S16>/Filter' */
  hilicopter_NDI_control_DW.Filter_DSTATE_a += 0.004F * rtb_NOut_bl;

  /* Update for DiscreteIntegrator: '<S24>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S24>/Iout'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_i += rtb_error_k *
    hilicopter_NDI_control_U.param[4] * 0.004F;

  /* Update for DiscreteIntegrator: '<S24>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE_k += 0.004F * rtb_NOut_b;

  /* Update for DiscreteIntegrator: '<S25>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S25>/Iout'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_g += rtb_error_c *
    hilicopter_NDI_control_U.param[5] * 0.004F;

  /* Update for DiscreteIntegrator: '<S25>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE_n += 0.004F * rtb_NOut_o;

  /* Update for DiscreteIntegrator: '<S29>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE_l += 0.004F * rtb_NOut_g;
  hilicopter_NDI_control_DW.FilterState_PrevResetState = (int8_T)rtb_Compare;

  /* Update for DiscreteIntegrator: '<S28>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE_h += 0.004F * rtb_NOut_j;
  hilicopter_NDI_control_DW.FilterState_PrevResetState_h = (int8_T)rtb_Compare;

  /* Update for DiscreteIntegrator: '<S27>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S27>/Iout'
   *  Sum: '<S27>/Sum3'
   *  Sum: '<S27>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_ig += (rtb_error_l *
    hilicopter_NDI_control_U.param[29] + (rtb_Saturation_o - rtb_Sum2_e)) *
    0.004F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState = (int8_T)rtb_Compare_f;

  /* Update for DiscreteIntegrator: '<S27>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_DSTATE_lx += 0.004F * rtb_NOut_jg;
  hilicopter_NDI_control_DW.FilterState_PrevResetState_hn = (int8_T)
    rtb_Compare_f;
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

  /* InitializeConditions for DiscreteIntegrator: '<S29>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S28>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_PrevResetState_h = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S27>/Integrator' */
  hilicopter_NDI_control_DW.Integrator_DSTATE_ig = 15.0F;
  hilicopter_NDI_control_DW.Integrator_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S27>/Filter State' */
  hilicopter_NDI_control_DW.FilterState_PrevResetState_hn = 2;
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