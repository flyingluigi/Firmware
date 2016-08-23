/*
 * File: hilicopter_NDI_control.c
 *
 * Code generated for Simulink model 'hilicopter_NDI_control'.
 *
 * Model version                  : 1.25
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Tue Aug 23 13:58:34 2016
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

/* Function for MATLAB Function: '<Root>/Rate_Velocity_Controller' */
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

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
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
  real32_T dhdx[72];
  real_T xdot_s[12];
  real32_T Ibi[9];
  real32_T b_cpsi;
  real32_T Tbn[9];
  real32_T VAb[3];
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
  real32_T rtb_Product;
  real32_T rtb_Add3;
  real32_T rtb_Add4;
  real32_T rtb_Sum;
  real32_T rtb_Saturate_n;
  real32_T rtb_Add5;
  real32_T rtb_Sum_n;
  real32_T rtb_Saturate_a;
  real32_T rtb_um[6];
  int32_T i;
  real32_T dhdx_0[36];
  int32_T j;
  real32_T tmp[3];
  real32_T tmp_0[9];
  real32_T tmp_1[9];
  real32_T tmp_2[24];
  real32_T tmp_3[24];
  real32_T dhdx_1[6];
  real32_T tmp_4[6];
  real32_T b_k_0[24];
  real32_T km_0[24];
  real32_T K_euler_0[24];
  real32_T b_k_1[96];
  real32_T b_k_2[96];
  real32_T b_k_3[72];
  int32_T i_0;
  real32_T tmp_5[6];
  uint16_T u0;
  real32_T u0_0;
  real32_T VAb_0;

  /* MATLAB Function: '<Root>/Attitude_Controller' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  Inport: '<Root>/state'
   *  Product: '<Root>/Product1'
   */
  /* MATLAB Function 'Attitude_Controller': '<S1>:1' */
  /* '<S1>:1:4' */
  for (i = 0; i < 9; i++) {
    K_euler[i] = 0.0F;
  }

  K_euler[0] = hilicopter_NDI_control_U.param[0];
  K_euler[4] = hilicopter_NDI_control_U.param[1];
  K_euler[8] = hilicopter_NDI_control_U.param[2];

  /* '<S1>:1:5' */
  /* '<S1>:1:7' */
  /* '<S1>:1:8' */
  /* '<S1>:1:9' */
  cpsi = hilicopter_NDI_control_U.param[22] * hilicopter_NDI_control_U.pwm_in[0]
    - hilicopter_NDI_control_U.state[9];
  sthe = hilicopter_NDI_control_U.param[22] * hilicopter_NDI_control_U.pwm_in[1]
    - hilicopter_NDI_control_U.state[10];
  b_cpsi = hilicopter_NDI_control_U.param[22] * 0.0F -
    hilicopter_NDI_control_U.state[11];
  for (i = 0; i < 3; i++) {
    rtb_omega_ref[i] = K_euler[i + 6] * b_cpsi + (K_euler[i + 3] * sthe +
      K_euler[i] * cpsi);
  }

  /* End of MATLAB Function: '<Root>/Attitude_Controller' */

  /* Product: '<Root>/Product' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   */
  rtb_Product = hilicopter_NDI_control_U.pwm_in[3] *
    hilicopter_NDI_control_U.param[21];

  /* Sum: '<S8>/Add3' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add3 = hilicopter_NDI_control_ConstB.max_vel_com[0] -
    hilicopter_NDI_control_U.state[12];

  /* Sum: '<S14>/Sum' incorporates:
   *  DiscreteIntegrator: '<S14>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S14>/POut'
   */
  u0_0 = rtb_Add3 * hilicopter_NDI_control_U.param[12] +
    hilicopter_NDI_control_DW.Integrator_DSTATE;

  /* Sum: '<S8>/Add4' incorporates:
   *  Inport: '<Root>/state'
   */
  rtb_Add4 = hilicopter_NDI_control_ConstB.max_vel_com[1] -
    hilicopter_NDI_control_U.state[13];

  /* Sum: '<S15>/Sum' incorporates:
   *  DiscreteIntegrator: '<S15>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S15>/POut'
   */
  rtb_Sum = rtb_Add4 * hilicopter_NDI_control_U.param[13] +
    hilicopter_NDI_control_DW.Integrator_DSTATE_k;

  /* Saturate: '<S15>/Saturate' */
  if (rtb_Sum > 2.0F) {
    rtb_Saturate_n = 2.0F;
  } else if (rtb_Sum < -2.0F) {
    rtb_Saturate_n = -2.0F;
  } else {
    rtb_Saturate_n = rtb_Sum;
  }

  /* End of Saturate: '<S15>/Saturate' */

  /* Sum: '<S8>/Add5' incorporates:
   *  Fcn: '<S5>/Fcn'
   *  Gain: '<S8>/Gain'
   *  Inport: '<Root>/pwm_in'
   *  Inport: '<Root>/state'
   */
  rtb_Add5 = (hilicopter_NDI_control_U.pwm_in[2] - 0.5F) * 2.0F * -2.0F -
    hilicopter_NDI_control_U.state[14];

  /* Sum: '<S16>/Sum' incorporates:
   *  DiscreteIntegrator: '<S16>/Integrator'
   *  Inport: '<Root>/param'
   *  Product: '<S16>/POut'
   */
  rtb_Sum_n = rtb_Add5 * hilicopter_NDI_control_U.param[14] +
    hilicopter_NDI_control_DW.Integrator_DSTATE_a;

  /* Saturate: '<S16>/Saturate' */
  if (rtb_Sum_n > 35.0F) {
    rtb_Saturate_a = 35.0F;
  } else if (rtb_Sum_n < -35.0F) {
    rtb_Saturate_a = -35.0F;
  } else {
    rtb_Saturate_a = rtb_Sum_n;
  }

  /* End of Saturate: '<S16>/Saturate' */

  /* MATLAB Function: '<Root>/Rate_Velocity_Controller' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/state'
   *  Product: '<S10>/POut'
   *  Product: '<S11>/POut'
   *  Product: '<S12>/POut'
   *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport4'
   *  Sum: '<S4>/Add'
   *  Sum: '<S4>/Add1'
   *  Sum: '<S4>/Add2'
   */
  /* MATLAB Function 'Rate_Velocity_Controller': '<S2>:1' */
  /* '<S2>:1:4' */
  /*  NDI controller for p,q,r and un,vn,wn of OC model */
  /*  Returns control input um = ["uroll; upitch; uyaw;" Fx; Fy; Fz] for given */
  /*  state and desired velocity v_ref and  omega_ref */
  /*  Revision history: */
  /*  2016|02|19: renamed some variables, updated comments */
  /*  Frequently used functions */
  sphi = (real32_T)sin(hilicopter_NDI_control_U.state[9]);
  cphi = (real32_T)cos(hilicopter_NDI_control_U.state[9]);
  sthe = (real32_T)sin(hilicopter_NDI_control_U.state[10]);
  cthe = (real32_T)cos(hilicopter_NDI_control_U.state[10]);
  spsi = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  cpsi = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  if ((real32_T)fabs(cthe) <= 1.0E-5F) {
    /*  Hack to avoid singulariy at +/- pi/2 */
    if (cthe < 0.0F) {
      cthe = -1.0F;
    } else if (cthe > 0.0F) {
      cthe = 1.0F;
    } else {
      if (cthe == 0.0F) {
        cthe = 0.0F;
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
  dhdx[0] = cthe * cpsi;
  dhdx[6] = sphi * sthe * cpsi - cphi * spsi;
  dhdx[12] = cphi * sthe * cpsi + sphi * spsi;
  dhdx[18] = 0.0F;
  dhdx[24] = 0.0F;
  dhdx[30] = 0.0F;
  dhdx[36] = 0.0F;
  dhdx[42] = 0.0F;
  dhdx[48] = 0.0F;
  dhdx[54] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * cpsi +
               hilicopter_NDI_control_U.state[1] * sphi * spsi) -
              hilicopter_NDI_control_U.state[2] * sphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * spsi;
  dhdx[60] = (hilicopter_NDI_control_U.state[1] * sphi * cthe * cpsi +
              -hilicopter_NDI_control_U.state[0] * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * cpsi;
  dhdx[66] = (((-hilicopter_NDI_control_U.state[0] * cthe * spsi -
                hilicopter_NDI_control_U.state[1] * sphi * sthe * spsi) -
               hilicopter_NDI_control_U.state[1] * cphi * cpsi) -
              hilicopter_NDI_control_U.state[2] * cphi * sthe * spsi) +
    hilicopter_NDI_control_U.state[2] * sphi * cpsi;
  dhdx[1] = cthe * spsi;
  dhdx[7] = sphi * sthe * spsi + cphi * cpsi;
  dhdx[13] = cphi * sthe * spsi - sphi * cpsi;
  dhdx[19] = 0.0F;
  dhdx[25] = 0.0F;
  dhdx[31] = 0.0F;
  dhdx[37] = 0.0F;
  dhdx[43] = 0.0F;
  dhdx[49] = 0.0F;
  dhdx[55] = ((hilicopter_NDI_control_U.state[1] * cphi * sthe * spsi -
               hilicopter_NDI_control_U.state[1] * sphi * cpsi) -
              hilicopter_NDI_control_U.state[2] * sphi * sthe * spsi) -
    hilicopter_NDI_control_U.state[2] * cphi * cpsi;
  dhdx[61] = ((hilicopter_NDI_control_U.state[1] * sphi * cthe * spsi +
               -hilicopter_NDI_control_U.state[0] * sthe * spsi) +
              hilicopter_NDI_control_U.state[1] * cphi * cpsi) +
    hilicopter_NDI_control_U.state[2] * cphi * cthe * spsi;
  dhdx[67] = (((hilicopter_NDI_control_U.state[1] * sphi * sthe * cpsi +
                hilicopter_NDI_control_U.state[0] * cthe * cpsi) -
               hilicopter_NDI_control_U.state[1] * cphi * spsi) +
              hilicopter_NDI_control_U.state[2] * cphi * sthe * cpsi) +
    hilicopter_NDI_control_U.state[2] * sphi * spsi;
  dhdx[2] = -sthe;
  dhdx[8] = sphi * cthe;
  dhdx[14] = cphi * cthe;
  dhdx[20] = 0.0F;
  dhdx[26] = 0.0F;
  dhdx[32] = 0.0F;
  dhdx[38] = 0.0F;
  dhdx[44] = 0.0F;
  dhdx[50] = 0.0F;
  dhdx[56] = hilicopter_NDI_control_U.state[1] * cphi * cthe -
    hilicopter_NDI_control_U.state[2] * sphi * cthe;
  dhdx[62] = (-hilicopter_NDI_control_U.state[0] * cthe -
              hilicopter_NDI_control_U.state[1] * sthe * sphi) -
    hilicopter_NDI_control_U.state[2] * cphi * sthe;
  dhdx[68] = 0.0F;
  for (i = 0; i < 12; i++) {
    dhdx[3 + 6 * i] = d[i];
    dhdx[4 + 6 * i] = c[i];
    dhdx[5 + 6 * i] = b[i];
  }

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

  K_euler[0] = hilicopter_NDI_control_U.param[9];
  K_euler[4] = hilicopter_NDI_control_U.param[10];
  K_euler[8] = hilicopter_NDI_control_U.param[11];
  Ibi[0] = 1.0F / hilicopter_NDI_control_U.param[9];
  Ibi[4] = 1.0F / hilicopter_NDI_control_U.param[10];
  Ibi[8] = 1.0F / hilicopter_NDI_control_U.param[11];

  /*  Initialization */
  memset(&xdot_s[0], 0, 12U * sizeof(real_T));

  /*  DCM to transform wind into body axes               */
  sphi = (real32_T)sin(hilicopter_NDI_control_U.state[9]);
  cphi = (real32_T)cos(hilicopter_NDI_control_U.state[9]);
  sthe = (real32_T)sin(hilicopter_NDI_control_U.state[10]);
  cthe = (real32_T)cos(hilicopter_NDI_control_U.state[10]);
  cpsi = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  b_cpsi = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  if ((real32_T)fabs(cthe) <= 1.0E-5F) {
    /*  Hack to avoid singulariy at +/- pi/2 */
    if (cthe < 0.0F) {
      cthe = -1.0F;
    } else if (cthe > 0.0F) {
      cthe = 1.0F;
    } else {
      if (cthe == 0.0F) {
        cthe = 0.0F;
      }
    }

    cthe *= 1.0E-5F;
  }

  spsi = sthe / cthe;
  Tbn[0] = cthe * b_cpsi;
  Tbn[3] = cthe * cpsi;
  Tbn[6] = -sthe;
  Tbn[1] = sphi * sthe * b_cpsi - cphi * cpsi;
  Tbn[4] = sphi * sthe * cpsi + cphi * b_cpsi;
  Tbn[7] = sphi * cthe;
  Tbn[2] = cphi * sthe * b_cpsi + sphi * cpsi;
  Tbn[5] = cphi * sthe * cpsi - sphi * b_cpsi;
  Tbn[8] = cphi * cthe;

  /*  Body drag force and moment (damping) */
  /*  for drag force */
  /*  for drag moment */
  /*  Drag force */
  cpsi = 0.0F;
  b_cpsi = 1.17549435E-38F;
  for (j = 0; j < 3; j++) {
    VAb_0 = hilicopter_NDI_control_U.state[j] - ((Tbn[j + 3] * 0.0F + Tbn[j] *
      0.0F) + Tbn[j + 6] * 0.0F);
    absxk = (real32_T)fabs(VAb_0);
    if (absxk > b_cpsi) {
      t = b_cpsi / absxk;
      cpsi = cpsi * t * t + 1.0F;
      b_cpsi = absxk;
    } else {
      t = absxk / b_cpsi;
      cpsi += t * t;
    }

    VAb[j] = VAb_0;
  }

  cpsi = b_cpsi * (real32_T)sqrt(cpsi);
  cpsi *= -0.062F;

  /*  Drag moment */
  /*  Gravity in body frame */
  /*  EOM */
  /*  H_eulerdot_omega */
  /*  Force (drag, gravity and inertia) */
  xdot_s[0] = (cpsi * VAb[0] / 3.5F + 9.80665F * -sthe) -
    (hilicopter_NDI_control_U.state[7] * hilicopter_NDI_control_U.state[2] -
     hilicopter_NDI_control_U.state[8] * hilicopter_NDI_control_U.state[1]);
  xdot_s[1] = (sphi * cthe * 9.80665F + cpsi * VAb[1] / 3.5F) -
    (hilicopter_NDI_control_U.state[8] * hilicopter_NDI_control_U.state[0] -
     hilicopter_NDI_control_U.state[6] * hilicopter_NDI_control_U.state[2]);
  xdot_s[2] = (cphi * cthe * 9.80665F + cpsi * VAb[2] / 3.5F) -
    (hilicopter_NDI_control_U.state[6] * hilicopter_NDI_control_U.state[1] -
     hilicopter_NDI_control_U.state[7] * hilicopter_NDI_control_U.state[0]);

  /*  Moment (drag, gravity and inertia) */
  for (i = 0; i < 3; i++) {
    tmp[i] = hilicopter_NDI_control_U.state[6 + i] - ((Tbn[i + 3] * 0.0F + Tbn[i]
      * 0.0F) + Tbn[i + 6] * 0.0F);
    VAb[i] = K_euler[i + 6] * hilicopter_NDI_control_U.state[8] + (K_euler[i + 3]
      * hilicopter_NDI_control_U.state[7] + K_euler[i] *
      hilicopter_NDI_control_U.state[6]);
  }

  cpsi = (-0.01F * tmp[0] - (hilicopter_NDI_control_U.state[7] * 0.0F -
           hilicopter_NDI_control_U.state[8] * 0.0F)) -
    (hilicopter_NDI_control_U.state[7] * VAb[2] -
     hilicopter_NDI_control_U.state[8] * VAb[1]);
  sthe = (-0.01F * tmp[1] - (hilicopter_NDI_control_U.state[8] * 0.0F -
           hilicopter_NDI_control_U.state[6] * 0.0F)) -
    (hilicopter_NDI_control_U.state[8] * VAb[0] -
     hilicopter_NDI_control_U.state[6] * VAb[2]);
  b_cpsi = (-0.01F * tmp[2] - (hilicopter_NDI_control_U.state[6] * 0.0F -
             hilicopter_NDI_control_U.state[7] * 0.0F)) -
    (hilicopter_NDI_control_U.state[6] * VAb[1] -
     hilicopter_NDI_control_U.state[7] * VAb[0]);

  /*  Position */
  /*  Attitude */
  tmp_0[0] = 1.0F;
  tmp_0[3] = sphi * spsi;
  tmp_0[6] = cphi * spsi;
  tmp_0[1] = 0.0F;
  tmp_0[4] = cphi;
  tmp_0[7] = -sphi;
  tmp_0[2] = 0.0F;
  tmp_0[5] = sphi / cthe;
  tmp_0[8] = cphi / cthe;
  for (i = 0; i < 3; i++) {
    xdot_s[6 + i] = (Ibi[i + 3] * sthe + Ibi[i] * cpsi) + Ibi[i + 6] * b_cpsi;
    xdot_s[3 + i] = (Tbn[3 * i + 1] * hilicopter_NDI_control_U.state[1] + Tbn[3 *
                     i] * hilicopter_NDI_control_U.state[0]) + Tbn[3 * i + 2] *
      hilicopter_NDI_control_U.state[2];
    xdot_s[9 + i] = (tmp_0[i + 3] * hilicopter_NDI_control_U.state[7] + tmp_0[i]
                     * hilicopter_NDI_control_U.state[6]) + tmp_0[i + 6] *
      hilicopter_NDI_control_U.state[8];
  }

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
  sphi = (real32_T)sin(hilicopter_NDI_control_U.state[9]);
  cphi = (real32_T)cos(hilicopter_NDI_control_U.state[9]);
  sthe = (real32_T)sin(hilicopter_NDI_control_U.state[10]);
  cthe = (real32_T)cos(hilicopter_NDI_control_U.state[10]);
  spsi = (real32_T)sin(hilicopter_NDI_control_U.state[11]);
  cpsi = (real32_T)cos(hilicopter_NDI_control_U.state[11]);
  if ((real32_T)fabs(cthe) <= 1.0E-5F) {
    /*  Hack to avoid singulariy at +/- pi/2 */
    if (cthe < 0.0F) {
      cthe = -1.0F;
    } else if (cthe > 0.0F) {
      cthe = 1.0F;
    } else {
      if (cthe == 0.0F) {
        cthe = 0.0F;
      }
    }

    cthe *= 1.0E-5F;
  }

  Tbn[0] = cthe * cpsi;
  Tbn[3] = cthe * spsi;
  Tbn[6] = -sthe;
  Tbn[1] = sphi * sthe * cpsi - cphi * spsi;
  Tbn[4] = sphi * sthe * spsi + cphi * cpsi;
  Tbn[7] = sphi * cthe;
  Tbn[2] = cphi * sthe * cpsi + sphi * spsi;
  Tbn[5] = cphi * sthe * spsi - sphi * cpsi;
  Tbn[8] = cphi * cthe;

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
  for (i = 0; i < 3; i++) {
    VAb[i] = hilicopter_NDI_control_U.state[6 + i] - ((Tbn[i + 3] * 0.0F + Tbn[i]
      * 0.0F) + Tbn[i + 6] * 0.0F);
    tmp[i] = hilicopter_NDI_control_U.state[i] - ((Tbn[i + 3] * 0.0F + Tbn[i] *
      0.0F) + Tbn[i + 6] * 0.0F);
  }

  tmp_1[0] = 0.0F;
  tmp_1[3] = -VAb[2];
  tmp_1[6] = VAb[1];
  tmp_1[1] = VAb[2];
  tmp_1[4] = 0.0F;
  tmp_1[7] = -VAb[0];
  tmp_1[2] = -VAb[1];
  tmp_1[5] = VAb[0];
  tmp_1[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 8; j++) {
      K_euler_0[i + 3 * j] = tmp[i];
      tmp_2[i + 3 * j] = 0.0F;
      tmp_2[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j] * tmp_1[i];
      tmp_2[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j + 1] * tmp_1[i + 3];
      tmp_2[i + 3 * j] += (real32_T)hilicopter_NDI_control_ConstP.pooled1.R[3 *
        j + 2] * tmp_1[i + 6];
    }
  }

  /*  Rot  1        2     3        4     5         6     7          8 */
  /* Fxb */
  /* Fyb        */
  /* Fzb */
  /*  Matrix for moments:  Mrotb = Mmat * un;  */
  for (i = 0; i < 8; i++) {
    tmp_3[3 * i] = K_euler_0[3 * i] + tmp_2[3 * i];
    tmp_3[1 + 3 * i] = K_euler_0[3 * i + 1] + tmp_2[3 * i + 1];
    tmp_3[2 + 3 * i] = K_euler_0[3 * i + 2] + tmp_2[3 * i + 2];
    sphi = (tmp_3[3 * i + 2] * 0.05F + 1.0F) * 1.28E-7F;
    km[i] = 0.0383F * sphi;
    kL[i] = 0.45F * sphi;
    b_k[i] = sphi;
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

  /*  [phi;theta;psi] dot */
  /*  Normalization of gmatrix  */
  /*  gmatrix = gmatrix_n2*param.n2trim; */
  /*  NDI control law */
  K_euler[0] = 1.0F / hilicopter_NDI_control_U.param[9];
  K_euler[4] = 1.0F / hilicopter_NDI_control_U.param[10];
  K_euler[8] = 1.0F / hilicopter_NDI_control_U.param[11];
  tmp_4[0] = 0.0F;
  tmp_4[1] = 0.0F;
  tmp_4[2] = rtb_Saturate_a;
  tmp_4[3] = (rtb_omega_ref[0] - hilicopter_NDI_control_U.state[6]) *
    hilicopter_NDI_control_U.param[3];
  tmp_4[4] = (rtb_omega_ref[1] - hilicopter_NDI_control_U.state[7]) *
    hilicopter_NDI_control_U.param[4];
  tmp_4[5] = (rtb_Product - hilicopter_NDI_control_U.state[8]) *
    hilicopter_NDI_control_U.param[5];
  for (i = 0; i < 6; i++) {
    dhdx_1[i] = 0.0F;
    for (j = 0; j < 12; j++) {
      dhdx_1[i] += dhdx[6 * j + i] * (real32_T)xdot_s[j];
    }

    rtb_um[i] = (0.0F - dhdx_1[i]) + tmp_4[i];
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

  for (i = 0; i < 12; i++) {
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
      dhdx_0[i + 6 * j] = 0.0F;
      for (i_0 = 0; i_0 < 12; i_0++) {
        dhdx_0[i + 6 * j] += dhdx[6 * i_0 + i] * b_k_3[12 * j + i_0];
      }
    }
  }

  hilicopter_NDI_control_mldivide(dhdx_0, rtb_um);

  /* End of MATLAB Function: '<Root>/Rate_Velocity_Controller' */

  /* MATLAB Function: '<S3>/norm' */
  /*  u = G^(-1)*[rdot - F + K*e]; */
  /*  Test = Max um F = +/-4 N for X and Y  Max M = 5 Nm for X and Y */
  /* MATLAB Function 'add_man_thrust_vel/norm': '<S9>:1' */
  /* '<S9>:1:2' */
  b_cpsi = 1.17549435E-38F;
  absxk = (real32_T)fabs(rtb_um[0]);
  if (absxk > 1.17549435E-38F) {
    cpsi = 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / 1.17549435E-38F;
    cpsi = t * t;
  }

  absxk = (real32_T)fabs(rtb_um[1]);
  if (absxk > b_cpsi) {
    t = b_cpsi / absxk;
    cpsi = cpsi * t * t + 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / b_cpsi;
    cpsi += t * t;
  }

  absxk = (real32_T)fabs(rtb_um[2]);
  if (absxk > b_cpsi) {
    t = b_cpsi / absxk;
    cpsi = cpsi * t * t + 1.0F;
    b_cpsi = absxk;
  } else {
    t = absxk / b_cpsi;
    cpsi += t * t;
  }

  cpsi = b_cpsi * (real32_T)sqrt(cpsi);

  /* SignalConversion: '<S7>/TmpSignal ConversionAtTnmInport1' incorporates:
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/pwm_in'
   */
  tmp_5[0] = 0.0F;
  tmp_5[1] = -3.0F * hilicopter_NDI_control_U.pwm_in[6];

  /* MATLAB Function: '<S3>/norm' */
  if (rtb_um[2] < 0.0F) {
    sphi = -1.0F;
  } else if (rtb_um[2] > 0.0F) {
    sphi = 1.0F;
  } else if (rtb_um[2] == 0.0F) {
    sphi = 0.0F;
  } else {
    sphi = rtb_um[2];
  }

  /* SignalConversion: '<S7>/TmpSignal ConversionAtTnmInport1' incorporates:
   *  Gain: '<S7>/Tnm'
   *  MATLAB Function: '<S3>/norm'
   */
  tmp_5[2] = sphi * cpsi;
  tmp_5[3] = rtb_um[3];
  tmp_5[4] = rtb_um[4];
  tmp_5[5] = rtb_um[5];

  /* MATLAB Function 'um2pwm/un2n': '<S13>:1' */
  /* '<S13>:1:4' */
  /* '<S13>:1:5' */
  for (j = 0; j < 8; j++) {
    /* Gain: '<S7>/Tnm' incorporates:
     *  Saturate: '<S7>/Saturation 0 - unmax'
     */
    km[j] = 0.0F;
    for (i = 0; i < 6; i++) {
      km[j] += hilicopter_NDI_control_ConstP.Tnm_Gain[(i << 3) + j] * tmp_5[i];
    }

    /* Saturate: '<S7>/Saturation 0 - unmax' incorporates:
     *  Gain: '<S7>/Tnm'
     */
    if (km[j] > 5.0F) {
      kL[j] = 5.0F;
    } else if (km[j] < 0.0F) {
      kL[j] = 0.0F;
    } else {
      kL[j] = km[j];
    }

    /* MATLAB Function: '<S7>/un2n' */
    b_k[j] = kL[j];
    if (b_k[j] > 0.0F) {
      b_k[j] = 1.0F;
    } else {
      if (b_k[j] == 0.0F) {
        b_k[j] = 0.0F;
      }
    }

    y[j] = kL[j];
    y[j] *= 3.3518824E+7F;
    y[j] = (real32_T)sqrt(y[j]);
    sphi = rt_roundf_snf(b_k[j] * y[j] * 0.0939F + 1000.0F);
    if (sphi < 65536.0F) {
      u0 = (uint16_T)sphi;
    } else {
      u0 = MAX_uint16_T;
    }

    /* Saturate: '<Root>/Output_Limits2' incorporates:
     *  MATLAB Function: '<S7>/un2n'
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

  /* Outport: '<Root>/debug' */
  hilicopter_NDI_control_Y.debug[0] = rtb_omega_ref[0];
  hilicopter_NDI_control_Y.debug[1] = rtb_omega_ref[1];
  hilicopter_NDI_control_Y.debug[2] = rtb_Product;

  /* Saturate: '<S14>/Saturate' */
  if (u0_0 > 2.0F) {
    /* Outport: '<Root>/debug' */
    hilicopter_NDI_control_Y.debug[3] = 2.0F;
  } else if (u0_0 < -2.0F) {
    /* Outport: '<Root>/debug' */
    hilicopter_NDI_control_Y.debug[3] = -2.0F;
  } else {
    /* Outport: '<Root>/debug' */
    hilicopter_NDI_control_Y.debug[3] = u0_0;
  }

  /* End of Saturate: '<S14>/Saturate' */

  /* Outport: '<Root>/debug' */
  hilicopter_NDI_control_Y.debug[4] = rtb_Saturate_n;
  hilicopter_NDI_control_Y.debug[5] = rtb_Saturate_a;
  for (i = 0; i < 6; i++) {
    hilicopter_NDI_control_Y.debug[i + 6] = rtb_um[i];
  }

  hilicopter_NDI_control_Y.debug[12] = 0.0F;

  /* Update for DiscreteIntegrator: '<S14>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S14>/IOut'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE += rtb_Add3 *
    hilicopter_NDI_control_U.param[15] * 0.004F;

  /* Update for DiscreteIntegrator: '<S15>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S15>/IOut'
   *  Sum: '<S15>/SumI1'
   *  Sum: '<S15>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_k += (rtb_Add4 *
    hilicopter_NDI_control_U.param[16] + (rtb_Saturate_n - rtb_Sum)) * 0.004F;

  /* Update for DiscreteIntegrator: '<S16>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Product: '<S16>/IOut'
   *  Sum: '<S16>/SumI1'
   *  Sum: '<S16>/SumI2'
   */
  hilicopter_NDI_control_DW.Integrator_DSTATE_a += (rtb_Add5 *
    hilicopter_NDI_control_U.param[17] + (rtb_Saturate_a - rtb_Sum_n)) * 0.004F;
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
