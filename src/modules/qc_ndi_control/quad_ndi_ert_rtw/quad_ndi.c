/*
 * File: quad_ndi.c
 *
 * Code generated for Simulink model 'quad_ndi'.
 *
 * Model version                  : 1.315
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Tue Nov 22 18:03:18 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "quad_ndi.h"
#include "quad_ndi_private.h"

/* Block states (auto storage) */
DW_quad_ndi_T quad_ndi_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_quad_ndi_T quad_ndi_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_quad_ndi_T quad_ndi_Y;

/* Real-time model */
RT_MODEL_quad_ndi_T quad_ndi_M_;
RT_MODEL_quad_ndi_T *const quad_ndi_M = &quad_ndi_M_;

/* Forward declaration for local functions */
static void quad_ndi_euler_to_R(const real32_T rot[3], real32_T data[9]);

/* Function for MATLAB Function: '<Root>/angle_control' */
static void quad_ndi_euler_to_R(const real32_T rot[3], real32_T data[9])
{
  real32_T cp;
  real32_T sp;
  real32_T sr;
  real32_T cr;
  real32_T sy;
  real32_T cy;
  cp = (real32_T)cos(rot[1]);
  sp = (real32_T)sin(rot[1]);
  sr = (real32_T)sin(rot[0]);
  cr = (real32_T)cos(rot[0]);
  sy = (real32_T)sin(rot[2]);
  cy = (real32_T)cos(rot[2]);
  data[0] = cp * cy;
  data[3] = sr * sp * cy - cr * sy;
  data[6] = cr * sp * cy + sr * sy;
  data[1] = cp * sy;
  data[4] = sr * sp * sy + cr * cy;
  data[7] = cr * sp * sy - sr * cy;
  data[2] = -sp;
  data[5] = sr * cp;
  data[8] = cr * cp;
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void quad_ndi_step(void)
{
  real32_T R[9];
  real32_T R_sp[9];
  real32_T e_R[3];
  real32_T e_R_z_sin;
  real32_T yaw_w;
  real32_T e_R_z_angle;
  real_T e_R_cp[9];
  real32_T R_rp[9];
  real32_T q_error[4];
  int8_T I[9];
  int32_T i;
  int32_T b_r;
  real32_T roll_pitch_scale;
  real32_T max_thrust_diff;
  static const real32_T mix[16] = { -0.707107F, 0.707107F, 0.707107F, -0.707107F,
    0.707107F, -0.707107F, 0.707107F, -0.707107F, 1.0F, 1.0F, -1.0F, -1.0F, 1.0F,
    1.0F, 1.0F, 1.0F };

  boolean_T exitg2;
  uint8_T rtb_mode;
  int32_T dcm_i;
  real32_T e_R_cp_0[9];
  real32_T I_0[9];
  real32_T e_R_z_axis_idx_2;
  real32_T e_R_z_axis_idx_0;
  real32_T rtb_att_control_idx_1;
  real32_T u1;
  real32_T tmp;

  /* MATLAB Function: '<Root>/angle_control' incorporates:
   *  Inport: '<Root>/state'
   */
  /* MATLAB Function 'angle_control': '<S8>:1' */
  /* '<S8>:1:4' */
  /*  time constant */
  /* '<S8>:1:5' */
  /*  time constant */
  /* '<S8>:1:7' */
  /* [6.5 * roll_tc ; 6.5 * pitch_tc; 2.8]; */
  /*  Rotation Matrix of measurement and desired rotation angles */
  /* '<S8>:1:11' */
  quad_ndi_euler_to_R(*(real32_T (*)[3])&quad_ndi_U.state[9], R);

  /* SignalConversion: '<S8>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  MATLAB Function: '<Root>/angle_control'
   *  Product: '<Root>/Product7'
   *  Product: '<Root>/Product8'
   */
  /* '<S8>:1:12' */
  e_R[0] = quad_ndi_U.pwm_in[0] * quad_ndi_U.param[40];
  e_R[1] = -quad_ndi_U.pwm_in[1] * quad_ndi_U.param[40];
  e_R[2] = 0.0F;

  /* MATLAB Function: '<Root>/angle_control' */
  quad_ndi_euler_to_R(e_R, R_sp);

  /*  try to move thrust vector shortest way, because yaw response is slower than roll/pitch % */
  /* '<S8>:1:16' */
  /* '<S8>:1:17' */
  /*  axis and sin(angle) of desired rotation % */
  /* '<S8>:1:20' */
  e_R_z_axis_idx_0 = R[7] * R_sp[8] - R[8] * R_sp[7];
  e_R_z_axis_idx_2 = R[8] * R_sp[6] - R[6] * R_sp[8];
  roll_pitch_scale = R[6] * R_sp[7] - R[7] * R_sp[6];

  /* calculate angle error % */
  /* '<S8>:1:24' */
  e_R_z_sin = 0.0F;
  rtb_att_control_idx_1 = 1.17549435E-38F;
  for (dcm_i = 0; dcm_i < 3; dcm_i++) {
    u1 = R[3 * dcm_i + 2] * roll_pitch_scale + (R[3 * dcm_i + 1] *
      e_R_z_axis_idx_2 + R[3 * dcm_i] * e_R_z_axis_idx_0);
    yaw_w = (real32_T)fabs(u1);
    if (yaw_w > rtb_att_control_idx_1) {
      e_R_z_angle = rtb_att_control_idx_1 / yaw_w;
      e_R_z_sin = e_R_z_sin * e_R_z_angle * e_R_z_angle + 1.0F;
      rtb_att_control_idx_1 = yaw_w;
    } else {
      e_R_z_angle = yaw_w / rtb_att_control_idx_1;
      e_R_z_sin += e_R_z_angle * e_R_z_angle;
    }

    e_R[dcm_i] = u1;
  }

  e_R_z_sin = rtb_att_control_idx_1 * (real32_T)sqrt(e_R_z_sin);

  /* '<S8>:1:25' */
  e_R_z_axis_idx_2 = (R[6] * R_sp[6] + R[7] * R_sp[7]) + R[8] * R_sp[8];

  /*  calculate weight for yaw control % */
  /* '<S8>:1:28' */
  /*  calculate rotation matrix after roll/pitch only rotation % */
  if (e_R_z_sin > 0.0F) {
    /* '<S8>:1:33' */
    /*  get axis-angle representation % */
    /* '<S8>:1:35' */
    e_R_z_angle = rt_atan2f_snf(e_R_z_sin, e_R_z_axis_idx_2);

    /* '<S8>:1:36' */
    /* '<S8>:1:38' */
    yaw_w = e_R[0] / e_R_z_sin;
    e_R[0] = yaw_w * e_R_z_angle;
    e_R_z_axis_idx_0 = yaw_w;
    yaw_w = e_R[1] / e_R_z_sin;
    e_R[1] = yaw_w * e_R_z_angle;
    e_R_z_angle = yaw_w;
    yaw_w = e_R[2] / e_R_z_sin;

    /*  cross product matrix for e_R_axis % */
    /* '<S8>:1:41' */
    /* '<S8>:1:42' */
    for (i = 0; i < 9; i++) {
      e_R_cp[i] = 0.0;
      I[i] = 0;
    }

    e_R_cp[3] = -yaw_w;

    /* '<S8>:1:43' */
    e_R_cp[6] = e_R_z_angle;

    /* '<S8>:1:44' */
    e_R_cp[1] = yaw_w;

    /* '<S8>:1:45' */
    e_R_cp[7] = -e_R_z_axis_idx_0;

    /* '<S8>:1:46' */
    e_R_cp[2] = -e_R_z_angle;

    /* '<S8>:1:47' */
    e_R_cp[5] = e_R_z_axis_idx_0;

    /*  rotation matrix for roll/pitch only rotation % */
    /* '<S8>:1:50' */
    for (dcm_i = 0; dcm_i < 3; dcm_i++) {
      I[dcm_i + 3 * dcm_i] = 1;
      for (i = 0; i < 3; i++) {
        e_R_cp_0[dcm_i + 3 * i] = (real32_T)((e_R_cp[3 * i + 1] * e_R_cp[dcm_i +
          3] + e_R_cp[3 * i] * e_R_cp[dcm_i]) + e_R_cp[3 * i + 2] * e_R_cp[dcm_i
          + 6]);
      }
    }

    for (i = 0; i < 3; i++) {
      I_0[3 * i] = ((real32_T)e_R_cp[3 * i] * e_R_z_sin + (real32_T)I[3 * i]) +
        e_R_cp_0[3 * i] * (1.0F - e_R_z_axis_idx_2);
      I_0[1 + 3 * i] = ((real32_T)e_R_cp[3 * i + 1] * e_R_z_sin + (real32_T)I[3 *
                        i + 1]) + e_R_cp_0[3 * i + 1] * (1.0F - e_R_z_axis_idx_2);
      I_0[2 + 3 * i] = ((real32_T)e_R_cp[3 * i + 2] * e_R_z_sin + (real32_T)I[3 *
                        i + 2]) + e_R_cp_0[3 * i + 2] * (1.0F - e_R_z_axis_idx_2);
    }

    for (i = 0; i < 3; i++) {
      for (dcm_i = 0; dcm_i < 3; dcm_i++) {
        R_rp[i + 3 * dcm_i] = 0.0F;
        R_rp[i + 3 * dcm_i] += I_0[3 * dcm_i] * R[i];
        R_rp[i + 3 * dcm_i] += I_0[3 * dcm_i + 1] * R[i + 3];
        R_rp[i + 3 * dcm_i] += I_0[3 * dcm_i + 2] * R[i + 6];
      }
    }
  } else {
    /*  zero roll/pitch rotation $/ */
    /* '<S8>:1:54' */
    for (i = 0; i < 9; i++) {
      R_rp[i] = R[i];
    }
  }

  /*  R_rp and R_sp has the same Z axis, calculate yaw error % */
  /* '<S8>:1:60' */
  /* '<S8>:1:61' */
  /* '<S8>:1:62' */
  if (e_R_z_axis_idx_2 < 0.0F) {
    /* '<S8>:1:64' */
    /*  for large thrust vector rotations use another rotation method: */
    /*  calculate angle and axis for R -> R_sp rotation directly % */
    /* '<S8>:1:67' */
    for (i = 0; i < 3; i++) {
      for (dcm_i = 0; dcm_i < 3; dcm_i++) {
        R_rp[i + 3 * dcm_i] = 0.0F;
        R_rp[i + 3 * dcm_i] += R[3 * i] * R_sp[3 * dcm_i];
        R_rp[i + 3 * dcm_i] += R[3 * i + 1] * R_sp[3 * dcm_i + 1];
        R_rp[i + 3 * dcm_i] += R[3 * i + 2] * R_sp[3 * dcm_i + 2];
      }
    }

    q_error[1] = 0.0F;
    e_R_z_sin = (R_rp[0] + R_rp[4]) + R_rp[8];
    if (e_R_z_sin > 0.0F) {
      e_R_z_sin = (real32_T)sqrt(e_R_z_sin + 1.0F);
      q_error[0] = e_R_z_sin * 0.5F;
      e_R_z_sin = 0.5F / e_R_z_sin;
      q_error[1] = (R_rp[5] - R_rp[7]) * e_R_z_sin;
    } else {
      /*  Find maximum diagonal element in dcm */
      /*  store index in dcm_i $/ */
      dcm_i = 0;
      if (R_rp[4] > R_rp[0]) {
        dcm_i = 1;
      }

      if (R_rp[8] > R_rp[3 * dcm_i + dcm_i]) {
        dcm_i = 2;
      }

      i = (dcm_i - (int32_T)(real32_T)floor(((real32_T)dcm_i + 1.0F) / 3.0F) * 3)
        + 1;
      b_r = (dcm_i - (int32_T)(real32_T)floor(((real32_T)dcm_i + 2.0F) / 3.0F) *
             3) + 2;
      e_R_z_sin = (real32_T)sqrt(((R_rp[3 * dcm_i + dcm_i] - R_rp[3 * i + i]) -
        R_rp[3 * b_r + b_r]) + 1.0F);
      q_error[dcm_i + 1] = e_R_z_sin * 0.5F;
      e_R_z_sin = 0.5F / e_R_z_sin;
      q_error[i + 1] = (R_rp[3 * i + dcm_i] + R_rp[3 * dcm_i + i]) * e_R_z_sin;
      q_error[b_r + 1] = (R_rp[3 * dcm_i + b_r] + R_rp[3 * b_r + dcm_i]) *
        e_R_z_sin;
      q_error[0] = (R_rp[3 * i + b_r] - R_rp[3 * b_r + i]) * e_R_z_sin;
    }

    if (q_error[0] >= 0.0F) {
      /* '<S8>:1:69' */
      /* '<S8>:1:70' */
      e_R_z_sin = q_error[1] * 2.0F;
    } else {
      /* '<S8>:1:72' */
      e_R_z_sin = -q_error[1] * 2.0F;
    }

    /*  use fusion of Z axis based rotation and direct rotation % */
    /* '<S8>:1:76' */
    rtb_att_control_idx_1 = e_R_z_axis_idx_2 * e_R_z_axis_idx_2 * (R_sp[8] *
      R_sp[8]);

    /* '<S8>:1:77' */
    e_R_z_sin *= rtb_att_control_idx_1;
    e_R[0] = (1.0F - rtb_att_control_idx_1) * e_R[0] + e_R_z_sin;
    e_R[1] = (1.0F - rtb_att_control_idx_1) * e_R[1] + e_R_z_sin;
  }

  /* SignalConversion: '<S21>/TmpSignal ConversionAt SFunction Inport2' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  MATLAB Function: '<Root>/angle_control'
   *  MATLAB Function: '<Root>/rate_control'
   *  Product: '<Root>/Product6'
   */
  /*  calculate angular rates setpoint % */
  /* '<S8>:1:82' */
  e_R_z_axis_idx_0 = quad_ndi_U.param[13] / 0.2F * quad_ndi_U.param[12] * e_R[0];
  e_R_z_angle = quad_ndi_U.param[13] / 0.2F * quad_ndi_U.param[12] * e_R[1];
  e_R_z_axis_idx_2 = quad_ndi_U.pwm_in[3] * quad_ndi_U.param[39];

  /* MATLAB Function: '<Root>/rate_control' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/pwm_in'
   *  Inport: '<Root>/state'
   */
  /* MATLAB Function 'rate_control': '<S21>:1' */
  /*  Parameters % */
  /* '<S21>:1:5' */
  /* '<S21>:1:6' */
  /*  Min 0 Max 2 */
  /* '<S21>:1:7' */
  /*  Min 0 Max 1 */
  /* '<S21>:1:9' */
  /* '<S21>:1:10' */
  e_R_z_sin = quad_ndi_U.param[13] / 0.2F;

  /*  time constant */
  /* '<S21>:1:11' */
  rtb_att_control_idx_1 = quad_ndi_U.param[13] / 0.2F;

  /*  time constant */
  /* '<S21>:1:13' */
  /* [0.15*roll_tc;0.15*pitch_tc;0.2]; */
  /* '<S21>:1:14' */
  /* [0.003*roll_tc;0.003*pitch_tc;0]; */
  /* '<S21>:1:15' */
  /*  [0.05;0.05;0.1]; */
  /* '<S21>:1:16' */
  /* '<S21>:1:24' */
  if (1.0F > 1.0F - ((real32_T)fabs(quad_ndi_U.pwm_in[2]) - 1.0F)) {
    yaw_w = 1.0F - ((real32_T)fabs(quad_ndi_U.pwm_in[2]) - 1.0F);
  } else {
    yaw_w = 1.0F;
  }

  if (!(0.0F < yaw_w)) {
    yaw_w = 0.0F;
  }

  /* '<S21>:1:26' */
  e_R[0] = e_R_z_axis_idx_0 - quad_ndi_U.state[6];
  e_R[1] = e_R_z_angle - quad_ndi_U.state[7];

  /* '<S21>:1:28' */
  e_R_z_sin = ((quad_ndi_U.param[6] * e_R_z_sin * (quad_ndi_DW.rates_prev[0] -
    quad_ndi_U.state[6]) / 0.004F + quad_ndi_U.param[0] * e_R_z_sin * (e_R[0] *
    yaw_w)) + quad_ndi_DW.rates_int[0]) + 0.0F * e_R_z_axis_idx_0;
  rtb_att_control_idx_1 = ((quad_ndi_U.param[6] * rtb_att_control_idx_1 *
    (quad_ndi_DW.rates_prev[1] - quad_ndi_U.state[7]) / 0.004F +
    quad_ndi_U.param[0] * rtb_att_control_idx_1 * (e_R[1] * yaw_w)) +
    quad_ndi_DW.rates_int[1]) + 0.0F * e_R_z_angle;
  e_R_z_angle = (((e_R_z_axis_idx_2 - quad_ndi_U.state[8]) * yaw_w *
                  quad_ndi_U.param[2] + (quad_ndi_DW.rates_prev[2] -
    quad_ndi_U.state[8]) * 0.0F / 0.004F) + quad_ndi_DW.rates_int[2]) + 0.0F *
    e_R_z_axis_idx_2;

  /* '<S21>:1:30' */
  quad_ndi_DW.rates_prev[0] = quad_ndi_U.state[6];
  quad_ndi_DW.rates_prev[1] = quad_ndi_U.state[7];
  quad_ndi_DW.rates_prev[2] = quad_ndi_U.state[8];
  if (quad_ndi_U.pwm_in[2] > 0.2F) {
    /* '<S21>:1:32' */
    /* '<S21>:1:33' */
    if (e_R_z_sin < quad_ndi_U.pwm_in[2]) {
      /* '<S21>:1:34' */
      /* '<S21>:1:35' */
      quad_ndi_DW.rates_int[0] += 0.0F * e_R[0] * 0.004F;
    }

    /* '<S21>:1:33' */
    if (rtb_att_control_idx_1 < quad_ndi_U.pwm_in[2]) {
      /* '<S21>:1:34' */
      /* '<S21>:1:35' */
      quad_ndi_DW.rates_int[1] += 0.0F * e_R[1] * 0.004F;
    }

    /* '<S21>:1:33' */
  }

  /* MATLAB Function: '<Root>/mixer' incorporates:
   *  Inport: '<Root>/pwm_in'
   */
  /* MATLAB Function 'mixer': '<S11>:1' */
  /*            Roll        Pitch  Yaw Thrust */
  /*  Motor 1 */
  /*  Motor 2 */
  /*  Motor 3 */
  /*  Motor 4 */
  /* '<S11>:1:11' */
  /*  thrust boost parameters */
  /* '<S11>:1:15' */
  /* '<S11>:1:16' */
  /* '<S11>:1:17' */
  /* '<S11>:1:19' */
  if (1.0F > e_R_z_sin) {
    yaw_w = e_R_z_sin;
  } else {
    yaw_w = 1.0F;
  }

  if (-1.0F < yaw_w) {
    e_R_z_sin = yaw_w;
  } else {
    e_R_z_sin = -1.0F;
  }

  /* '<S11>:1:20' */
  if (!(1.0F > rtb_att_control_idx_1)) {
    rtb_att_control_idx_1 = 1.0F;
  }

  if (!(-1.0F < rtb_att_control_idx_1)) {
    rtb_att_control_idx_1 = -1.0F;
  }

  /* '<S11>:1:21' */
  if (1.0F > e_R_z_angle) {
    yaw_w = e_R_z_angle;
  } else {
    yaw_w = 1.0F;
  }

  if (!(-1.0F < yaw_w)) {
    yaw_w = -1.0F;
  }

  /* '<S11>:1:22' */
  if (1.0F > quad_ndi_U.pwm_in[2]) {
    e_R_z_angle = quad_ndi_U.pwm_in[2];
  } else {
    e_R_z_angle = 1.0F;
  }

  if (!(0.0F < e_R_z_angle)) {
    e_R_z_angle = 0.0F;
  }

  /*  perform initial mix pass yielding unbounded outputs, ignore yaw % */
  /* '<S11>:1:25' */
  for (i = 0; i < 4; i++) {
    u1 = mix[i + 12] * e_R_z_angle + (mix[i + 8] * 0.0F + (mix[i + 4] *
      rtb_att_control_idx_1 + mix[i] * e_R_z_sin));
    q_error[i] = u1;
  }

  /* calculate min and max output values $/ */
  /* '<S11>:1:27' */
  dcm_i = 1;
  e_R_z_axis_idx_2 = q_error[0];
  if (rtIsNaNF(q_error[0])) {
    i = 2;
    exitg2 = false;
    while ((!exitg2) && (i < 5)) {
      dcm_i = i;
      if (!rtIsNaNF(q_error[i - 1])) {
        e_R_z_axis_idx_2 = q_error[i - 1];
        exitg2 = true;
      } else {
        i++;
      }
    }
  }

  if (dcm_i < 4) {
    while (dcm_i + 1 < 5) {
      if (q_error[dcm_i] < e_R_z_axis_idx_2) {
        e_R_z_axis_idx_2 = q_error[dcm_i];
      }

      dcm_i++;
    }
  }

  /* '<S11>:1:28' */
  dcm_i = 1;
  u1 = q_error[0];
  if (rtIsNaNF(q_error[0])) {
    i = 2;
    exitg2 = false;
    while ((!exitg2) && (i < 5)) {
      dcm_i = i;
      if (!rtIsNaNF(q_error[i - 1])) {
        u1 = q_error[i - 1];
        exitg2 = true;
      } else {
        i++;
      }
    }
  }

  if (dcm_i < 4) {
    while (dcm_i + 1 < 5) {
      if (q_error[dcm_i] > u1) {
        u1 = q_error[dcm_i];
      }

      dcm_i++;
    }
  }

  /* '<S11>:1:30' */
  e_R_z_axis_idx_0 = 0.0F;

  /*  value added to demanded thrust (can also be negative) */
  /* '<S11>:1:31' */
  roll_pitch_scale = 1.0F;

  /*  scale for demanded roll and pitch */
  if ((e_R_z_axis_idx_2 < 0.0F) && (u1 < 1.0F) && (-e_R_z_axis_idx_2 <= 1.0F -
       u1)) {
    /* '<S11>:1:33' */
    /* '<S11>:1:34' */
    max_thrust_diff = e_R_z_angle * 1.5F - e_R_z_angle;
    if (max_thrust_diff >= -e_R_z_axis_idx_2) {
      /* '<S11>:1:36' */
      /* '<S11>:1:37' */
      e_R_z_axis_idx_0 = -e_R_z_axis_idx_2;
    } else {
      /* '<S11>:1:40' */
      e_R_z_axis_idx_0 = max_thrust_diff;

      /* '<S11>:1:41' */
      roll_pitch_scale = (e_R_z_angle + max_thrust_diff) / (e_R_z_angle -
        e_R_z_axis_idx_2);
    }
  } else if ((u1 > 1.0F) && (e_R_z_axis_idx_2 > 0.0F) && (e_R_z_axis_idx_2 >= u1
              - 1.0F)) {
    /* '<S11>:1:44' */
    /* '<S11>:1:45' */
    max_thrust_diff = e_R_z_angle - 0.6F * e_R_z_angle;
    if (max_thrust_diff >= u1 - 1.0F) {
      /* '<S11>:1:47' */
      /* '<S11>:1:48' */
      e_R_z_axis_idx_0 = -(u1 - 1.0F);
    } else {
      /* '<S11>:1:50' */
      e_R_z_axis_idx_0 = -max_thrust_diff;

      /* '<S11>:1:51' */
      roll_pitch_scale = (1.0F - (e_R_z_angle + -max_thrust_diff)) / (u1 -
        e_R_z_angle);
    }
  } else if ((e_R_z_axis_idx_2 < 0.0F) && (u1 < 1.0F) && (-e_R_z_axis_idx_2 >
              1.0F - u1)) {
    /* '<S11>:1:54' */
    /* '<S11>:1:55' */
    /* '<S11>:1:56' */
    roll_pitch_scale = e_R_z_angle * 1.5F - e_R_z_angle;
    u1 = -e_R_z_axis_idx_2 - (1.0F - u1) / 2.0F;
    if ((roll_pitch_scale <= u1) || rtIsNaNF(u1)) {
      u1 = roll_pitch_scale;
    }

    if (0.0F < u1) {
      e_R_z_axis_idx_0 = u1;
    }

    /* '<S11>:1:57' */
    roll_pitch_scale = (e_R_z_angle + e_R_z_axis_idx_0) / (e_R_z_angle -
      e_R_z_axis_idx_2);
  } else if ((u1 > 1.0F) && (e_R_z_axis_idx_2 > 0.0F) && (e_R_z_axis_idx_2 < u1
              - 1.0F)) {
    /* '<S11>:1:59' */
    /* '<S11>:1:60' */
    /* '<S11>:1:61' */
    e_R_z_axis_idx_2 = -((u1 - 1.0F) - e_R_z_axis_idx_2) / 2.0F;
    if (!(0.0F > e_R_z_axis_idx_2)) {
      e_R_z_axis_idx_2 = 0.0F;
    }

    e_R_z_axis_idx_0 = -(e_R_z_angle - 0.6F * e_R_z_angle);
    if (!(e_R_z_axis_idx_0 >= e_R_z_axis_idx_2)) {
      e_R_z_axis_idx_0 = e_R_z_axis_idx_2;
    }

    /* '<S11>:1:62' */
    roll_pitch_scale = (1.0F - (e_R_z_angle + e_R_z_axis_idx_0)) / (u1 -
      e_R_z_angle);
  } else {
    if ((e_R_z_axis_idx_2 < 0.0F) && (u1 > 1.0F)) {
      /* '<S11>:1:64' */
      /* '<S11>:1:65' */
      roll_pitch_scale = 1.5F * e_R_z_angle - e_R_z_angle;
      e_R_z_axis_idx_0 = -((u1 - 1.0F) + e_R_z_axis_idx_2) / 2.0F;
      if ((roll_pitch_scale <= e_R_z_axis_idx_0) || rtIsNaNF(e_R_z_axis_idx_0))
      {
        e_R_z_axis_idx_0 = roll_pitch_scale;
      }

      roll_pitch_scale = 0.6F * e_R_z_angle - e_R_z_angle;
      if ((roll_pitch_scale >= e_R_z_axis_idx_0) || rtIsNaNF(e_R_z_axis_idx_0))
      {
        e_R_z_axis_idx_0 = roll_pitch_scale;
      }

      /* '<S11>:1:66' */
      roll_pitch_scale = (e_R_z_angle + e_R_z_axis_idx_0) / (e_R_z_angle -
        e_R_z_axis_idx_2);
    }
  }

  /*  notify if saturation has occurred */
  /*  mix again but now with thrust boost, scale roll/pitch and also add yaw */
  /* '<S11>:1:78' */
  /* '<S11>:1:79' */
  /*  add yaw and scale outputs to range idle_speed...1 % */
  /* '<S11>:1:107' */
  /* '<S11>:1:79' */
  /* '<S11>:1:80' */
  u1 = (((-0.707107F * e_R_z_sin + 0.707107F * rtb_att_control_idx_1) *
         roll_pitch_scale + yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /*  scale yaw if it violates limits. inform about yaw limit reached */
  if (u1 < 0.0F) {
    /* '<S11>:1:83' */
    /* '<S11>:1:88' */
    yaw_w = -(((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 * 0.707107F) *
               roll_pitch_scale + e_R_z_angle) + e_R_z_axis_idx_0);
  } else {
    if (u1 > 1.0F) {
      /* '<S11>:1:91' */
      /*  allow to reduce thrust to get some yaw response */
      /* '<S11>:1:93' */
      /* '<S11>:1:94' */
      if (0.15F <= u1 - 1.0F) {
        e_R_z_axis_idx_2 = 0.15F;
      } else {
        e_R_z_axis_idx_2 = u1 - 1.0F;
      }

      e_R_z_angle -= e_R_z_axis_idx_2;

      /* '<S11>:1:100' */
      yaw_w = 1.0F - (((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 *
                        0.707107F) * roll_pitch_scale + e_R_z_angle) +
                      e_R_z_axis_idx_0);
    }
  }

  /* '<S11>:1:79' */
  /* '<S11>:1:80' */
  u1 = (((0.707107F * e_R_z_sin + -0.707107F * rtb_att_control_idx_1) *
         roll_pitch_scale + yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /*  scale yaw if it violates limits. inform about yaw limit reached */
  if (u1 < 0.0F) {
    /* '<S11>:1:83' */
    /* '<S11>:1:88' */
    yaw_w = -(((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 * -0.707107F) *
               roll_pitch_scale + e_R_z_angle) + e_R_z_axis_idx_0);
  } else {
    if (u1 > 1.0F) {
      /* '<S11>:1:91' */
      /*  allow to reduce thrust to get some yaw response */
      /* '<S11>:1:93' */
      /* '<S11>:1:94' */
      if (0.15F <= u1 - 1.0F) {
        e_R_z_axis_idx_2 = 0.15F;
      } else {
        e_R_z_axis_idx_2 = u1 - 1.0F;
      }

      e_R_z_angle -= e_R_z_axis_idx_2;

      /* '<S11>:1:100' */
      yaw_w = 1.0F - (((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 *
                        -0.707107F) * roll_pitch_scale + e_R_z_angle) +
                      e_R_z_axis_idx_0);
    }
  }

  /* '<S11>:1:79' */
  /* '<S11>:1:80' */
  u1 = (((0.707107F * e_R_z_sin + 0.707107F * rtb_att_control_idx_1) *
         roll_pitch_scale + -yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /*  scale yaw if it violates limits. inform about yaw limit reached */
  if (u1 < 0.0F) {
    /* '<S11>:1:83' */
    /* '<S11>:1:88' */
    yaw_w = -(-(((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 * 0.707107F) *
                 roll_pitch_scale + e_R_z_angle) + e_R_z_axis_idx_0));
  } else {
    if (u1 > 1.0F) {
      /* '<S11>:1:91' */
      /*  allow to reduce thrust to get some yaw response */
      /* '<S11>:1:93' */
      /* '<S11>:1:94' */
      if (0.15F <= u1 - 1.0F) {
        e_R_z_axis_idx_2 = 0.15F;
      } else {
        e_R_z_axis_idx_2 = u1 - 1.0F;
      }

      e_R_z_angle -= e_R_z_axis_idx_2;

      /* '<S11>:1:100' */
      yaw_w = -(1.0F - (((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 *
                          0.707107F) * roll_pitch_scale + e_R_z_angle) +
                        e_R_z_axis_idx_0));
    }
  }

  /* '<S11>:1:79' */
  /* '<S11>:1:80' */
  u1 = (((-0.707107F * e_R_z_sin + -0.707107F * rtb_att_control_idx_1) *
         roll_pitch_scale + -yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /*  scale yaw if it violates limits. inform about yaw limit reached */
  if (u1 < 0.0F) {
    /* '<S11>:1:83' */
    /* '<S11>:1:88' */
    yaw_w = -(-(((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 * -0.707107F) *
                 roll_pitch_scale + e_R_z_angle) + e_R_z_axis_idx_0));
  } else {
    if (u1 > 1.0F) {
      /* '<S11>:1:91' */
      /*  allow to reduce thrust to get some yaw response */
      /* '<S11>:1:93' */
      /* '<S11>:1:94' */
      if (0.15F <= u1 - 1.0F) {
        e_R_z_axis_idx_2 = 0.15F;
      } else {
        e_R_z_axis_idx_2 = u1 - 1.0F;
      }

      e_R_z_angle -= e_R_z_axis_idx_2;

      /* '<S11>:1:100' */
      yaw_w = -(1.0F - (((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 *
                          -0.707107F) * roll_pitch_scale + e_R_z_angle) +
                        e_R_z_axis_idx_0));
    }
  }

  /* '<S11>:1:79' */
  /* '<S11>:1:108' */
  /* MATLAB Function 'MATLAB Function': '<S4>:1' */
  /* '<S4>:1:2' */
  /* '<S4>:1:4' */
  /* '<S4>:1:5' */
  /* '<S4>:1:7' */
  /* '<S4>:1:8' */
  /* '<S11>:1:108' */
  /* '<S11>:1:109' */
  u1 = (((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 * 0.707107F) *
         roll_pitch_scale + yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /* '<S11>:1:111' */
  if (!(1.0F > u1)) {
    u1 = 1.0F;
  }

  /*  slew rate limiting */
  /* '<S11>:1:108' */
  if (0.0F >= u1) {
    e_R_z_axis_idx_2 = 0.0F;
  } else {
    e_R_z_axis_idx_2 = u1;
  }

  /* '<S11>:1:108' */
  /* '<S11>:1:109' */
  u1 = (((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 * -0.707107F) *
         roll_pitch_scale + yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /* '<S11>:1:111' */
  if (!(1.0F > u1)) {
    u1 = 1.0F;
  }

  /*  slew rate limiting */
  /* '<S11>:1:108' */
  if (0.0F >= u1) {
    max_thrust_diff = 0.0F;
  } else {
    max_thrust_diff = u1;
  }

  /* '<S11>:1:108' */
  /* '<S11>:1:109' */
  u1 = (((e_R_z_sin * 0.707107F + rtb_att_control_idx_1 * 0.707107F) *
         roll_pitch_scale + -yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /* '<S11>:1:111' */
  if (!(1.0F > u1)) {
    u1 = 1.0F;
  }

  /*  slew rate limiting */
  /* '<S11>:1:108' */
  if (0.0F >= u1) {
    tmp = 0.0F;
  } else {
    tmp = u1;
  }

  /* '<S11>:1:108' */
  /* '<S11>:1:109' */
  u1 = (((e_R_z_sin * -0.707107F + rtb_att_control_idx_1 * -0.707107F) *
         roll_pitch_scale + -yaw_w) + e_R_z_angle) + e_R_z_axis_idx_0;

  /* '<S11>:1:111' */
  if (!(1.0F > u1)) {
    u1 = 1.0F;
  }

  /* Outport: '<Root>/pwm' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  /*  slew rate limiting */
  /* '<S11>:1:108' */
  quad_ndi_Y.pwm[0] = (e_R_z_axis_idx_2 - -1.0F) / 2.0F * 2047.0F;
  quad_ndi_Y.pwm[1] = (max_thrust_diff - -1.0F) / 2.0F * 2047.0F;
  quad_ndi_Y.pwm[2] = (tmp - -1.0F) / 2.0F * 2047.0F;

  /* MATLAB Function: '<Root>/mixer' */
  if (0.0F >= u1) {
    u1 = 0.0F;
  }

  /* Outport: '<Root>/pwm' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  quad_ndi_Y.pwm[3] = (u1 - -1.0F) / 2.0F * 2047.0F;

  /* MATLAB Function: '<Root>/mode select' incorporates:
   *  Inport: '<Root>/pwm_in'
   */
  /* MATLAB Function 'mode select': '<S12>:1' */
  /* Define modes according to switch position */
  /*  Mode 1 = MANUAL */
  /*  Mode 2 = ALT HOLD */
  if (quad_ndi_U.pwm_in[5] > 0.5F) {
    /* '<S12>:1:7' */
    /* '<S12>:1:8' */
    rtb_mode = 3U;
  } else if (quad_ndi_U.pwm_in[5] > -0.5F) {
    /* '<S12>:1:9' */
    /* '<S12>:1:10' */
    rtb_mode = 2U;
  } else {
    /* '<S12>:1:12' */
    rtb_mode = 1U;
  }

  /* End of MATLAB Function: '<Root>/mode select' */

  /* Outport: '<Root>/debug' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion1'
   */
  quad_ndi_Y.debug[0] = rtb_mode;
  quad_ndi_Y.debug[1] = 0.0F;
  quad_ndi_Y.debug[2] = 0.0F;
  quad_ndi_Y.debug[3] = 0.0F;

  /* MATLAB Function 'commander_logic': '<S10>:1' */
  /* Define modes according to switch position */
  /*  Mode 1 = MANUAL */
  /*  Mode 2 = ALT HOLD */
}

/* Model initialize function */
void quad_ndi_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(quad_ndi_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&quad_ndi_DW, 0,
                sizeof(DW_quad_ndi_T));

  /* external inputs */
  (void) memset((void *)&quad_ndi_U, 0,
                sizeof(ExtU_quad_ndi_T));

  /* external outputs */
  (void) memset((void *)&quad_ndi_Y, 0,
                sizeof(ExtY_quad_ndi_T));

  /* InitializeConditions for MATLAB Function: '<Root>/rate_control' */
  quad_ndi_DW.rates_int[0] = 0.0F;
  quad_ndi_DW.rates_prev[0] = 0.0F;
  quad_ndi_DW.rates_int[1] = 0.0F;
  quad_ndi_DW.rates_prev[1] = 0.0F;
  quad_ndi_DW.rates_int[2] = 0.0F;
  quad_ndi_DW.rates_prev[2] = 0.0F;
}

/* Model terminate function */
void quad_ndi_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
