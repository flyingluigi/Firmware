//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: quad_ndi.cpp
//
// Code generated for Simulink model 'quad_ndi'.
//
// Model version                  : 1.644
// Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
// C/C++ source code generated on : Tue Mar 27 21:22:40 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "quad_ndi.h"

// Model step function
void quad_ndi_class::step(const real32_T arg_vehicle_attitude[6], const real32_T
  arg_vehicle_attitude_setpoint[4], const real32_T arg_param[18], real32_T
  arg_cmd[4], real32_T arg_debug[4])
{
  static const int8_T b[16] = { -1, 1, 0, 0, 0, 0, -1, 1, -1, -1, 0, 0, 0, 0, 1,
    1 };

  real32_T rtb_Sum2_n;
  real32_T rtb_Sum3;
  real32_T rtb_Sum2;
  real32_T rtb_Sum1;
  real32_T rtb_Sum3_f;
  real32_T rtb_NOut_e;
  real32_T rtb_y[4];
  int32_T i;
  real32_T rtb_Sum1_idx_0;
  real32_T rtb_Sum1_idx_1;
  real32_T rtb_Sum1_idx_2;
  real32_T rtb_y_d;

  // Sum: '<S1>/Sum6' incorporates:
  //   Abs: '<S1>/Abs'
  //   Abs: '<S1>/Abs1'
  //   Gain: '<S1>/Gain2'
  //   Gain: '<S1>/Gain3'
  //   Gain: '<S1>/Gain4'
  //   Inport: '<Root>/vehicle_attitude_setpoint'

  rtb_Sum1 = (0.5F * (real32_T)fabs((real_T)arg_vehicle_attitude_setpoint[2]) +
              arg_vehicle_attitude_setpoint[3]) + 0.5F * (real32_T)fabs((real_T)
    -arg_vehicle_attitude_setpoint[0]);

  // Sum: '<S1>/Sum' incorporates:
  //   Inport: '<Root>/vehicle_attitude_setpoint'

  rtb_Sum2_n = rtb_Sum1 - arg_vehicle_attitude_setpoint[1];

  // Sum: '<S1>/Sum1' incorporates:
  //   Inport: '<Root>/vehicle_attitude_setpoint'

  rtb_Sum1 += arg_vehicle_attitude_setpoint[1];

  // Sum: '<S1>/Sum4' incorporates:
  //   Constant: '<S1>/Constant'
  //   Gain: '<S1>/Gain'
  //   Gain: '<S1>/Gain2'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Sum: '<S1>/Sum3'

  rtb_Sum3 = (-arg_vehicle_attitude_setpoint[0] - arg_vehicle_attitude_setpoint
              [2]) * 0.5F + 0.5F;

  // Saturate: '<Root>/Saturation1'
  if (rtb_Sum3 > 1.0F) {
    // Outport: '<Root>/debug'
    arg_debug[0] = 1.0F;
  } else if (rtb_Sum3 < 0.0F) {
    // Outport: '<Root>/debug'
    arg_debug[0] = 0.0F;
  } else {
    // Outport: '<Root>/debug'
    arg_debug[0] = rtb_Sum3;
  }

  // Sum: '<S1>/Sum5' incorporates:
  //   Constant: '<S1>/Constant'
  //   Gain: '<S1>/Gain1'
  //   Gain: '<S1>/Gain2'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Sum: '<S1>/Sum2'

  rtb_Sum3 = ((0.0F - arg_vehicle_attitude_setpoint[2]) -
              (-arg_vehicle_attitude_setpoint[0])) * 0.5F + 0.5F;

  // Saturate: '<Root>/Saturation1'
  if (rtb_Sum3 > 1.0F) {
    // Outport: '<Root>/debug'
    arg_debug[1] = 1.0F;
  } else if (rtb_Sum3 < 0.0F) {
    // Outport: '<Root>/debug'
    arg_debug[1] = 0.0F;
  } else {
    // Outport: '<Root>/debug'
    arg_debug[1] = rtb_Sum3;
  }

  if (rtb_Sum2_n > 1.0F) {
    // Outport: '<Root>/debug'
    arg_debug[2] = 1.0F;
  } else if (rtb_Sum2_n < 0.0F) {
    // Outport: '<Root>/debug'
    arg_debug[2] = 0.0F;
  } else {
    // Outport: '<Root>/debug'
    arg_debug[2] = rtb_Sum2_n;
  }

  if (rtb_Sum1 > 1.0F) {
    // Outport: '<Root>/debug'
    arg_debug[3] = 1.0F;
  } else if (rtb_Sum1 < 0.0F) {
    // Outport: '<Root>/debug'
    arg_debug[3] = 0.0F;
  } else {
    // Outport: '<Root>/debug'
    arg_debug[3] = rtb_Sum1;
  }

  // Outputs for Atomic SubSystem: '<Root>/stab_controller'
  // Sum: '<S4>/Sum1' incorporates:
  //   Gain: '<S5>/Gain'
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Product: '<S11>/POut'
  //   Sum: '<S5>/Sum5'

  rtb_Sum1 = (0.52359879F * arg_vehicle_attitude_setpoint[0] -
              arg_vehicle_attitude[0]) * arg_param[15] - arg_vehicle_attitude[3];

  // End of Outputs for SubSystem: '<Root>/stab_controller'

  // Product: '<S8>/NOut' incorporates:
  //   DiscreteIntegrator: '<S8>/Filter'
  //   Inport: '<Root>/param'
  //   Product: '<S8>/DOut'
  //   Sum: '<S8>/SumD'

  rtb_Sum3 = (rtb_Sum1 * arg_param[6] - rtDW.Filter_DSTATE) * arg_param[9];

  // Outputs for Atomic SubSystem: '<Root>/stab_controller'
  // Sum: '<S4>/Sum2' incorporates:
  //   Gain: '<S5>/Gain1'
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Product: '<S10>/POut'
  //   Sum: '<S5>/Sum6'

  rtb_Sum2_n = (0.52359879F * arg_vehicle_attitude_setpoint[1] -
                arg_vehicle_attitude[1]) * arg_param[16] - arg_vehicle_attitude
    [4];

  // End of Outputs for SubSystem: '<Root>/stab_controller'

  // Product: '<S7>/NOut' incorporates:
  //   DiscreteIntegrator: '<S7>/Filter'
  //   Inport: '<Root>/param'
  //   Product: '<S7>/DOut'
  //   Sum: '<S7>/SumD'

  rtb_Sum2 = (rtb_Sum2_n * arg_param[7] - rtDW.Filter_DSTATE_n) * arg_param[10];

  // Outputs for Atomic SubSystem: '<Root>/stab_controller'
  // Sum: '<S4>/Sum3' incorporates:
  //   Gain: '<S5>/Gain2'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'

  rtb_Sum3_f = 3.14159274F * arg_vehicle_attitude_setpoint[2] -
    arg_vehicle_attitude[5];

  // End of Outputs for SubSystem: '<Root>/stab_controller'

  // Product: '<S9>/NOut' incorporates:
  //   DiscreteIntegrator: '<S9>/Filter'
  //   Inport: '<Root>/param'
  //   Product: '<S9>/DOut'
  //   Sum: '<S9>/SumD'

  rtb_NOut_e = (rtb_Sum3_f * arg_param[8] - rtDW.Filter_DSTATE_a) * arg_param[11];

  // SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   DiscreteIntegrator: '<S7>/Integrator'
  //   DiscreteIntegrator: '<S8>/Integrator'
  //   DiscreteIntegrator: '<S9>/Integrator'
  //   Inport: '<Root>/param'
  //   MATLAB Function: '<Root>/mixer'
  //   Product: '<S7>/POut'
  //   Product: '<S8>/POut'
  //   Product: '<S9>/POut'
  //   Sum: '<S7>/Sum'
  //   Sum: '<S8>/Sum'
  //   Sum: '<S9>/Sum'

  // MATLAB Function 'mixer': '<S3>:1'
  // '<S3>:1:11'
  rtb_Sum1_idx_0 = (rtb_Sum1 * arg_param[0] + rtDW.Integrator_DSTATE) + rtb_Sum3;
  rtb_Sum1_idx_1 = (rtb_Sum2_n * arg_param[1] + rtDW.Integrator_DSTATE_m) +
    rtb_Sum2;
  rtb_Sum1_idx_2 = (rtb_Sum3_f * arg_param[2] + rtDW.Integrator_DSTATE_d) +
    rtb_NOut_e;

  // MATLAB Function: '<Root>/mixer' incorporates:
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport1'

  for (i = 0; i < 4; i++) {
    rtb_y_d = (real32_T)b[i + 12] * arg_vehicle_attitude_setpoint[3] +
      ((real32_T)b[i + 8] * rtb_Sum1_idx_2 + ((real32_T)b[i + 4] *
        rtb_Sum1_idx_1 + (real32_T)b[i] * rtb_Sum1_idx_0));
    rtb_y[i] = rtb_y_d;
  }

  // '<S3>:1:14'
  rtb_y[0] = 0.5F * rtb_y[0] + 0.5F;

  // '<S3>:1:15'
  rtb_y[1] = 0.5F * rtb_y[1] + 0.5F;

  // Saturate: '<Root>/saturation 0-1'
  if (rtb_y[0] > 1.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[0] = 1.0F;
  } else if (rtb_y[0] < 0.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[0] = 0.0F;
  } else {
    // Outport: '<Root>/cmd'
    arg_cmd[0] = rtb_y[0];
  }

  if (rtb_y[1] > 1.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[1] = 1.0F;
  } else if (rtb_y[1] < 0.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[1] = 0.0F;
  } else {
    // Outport: '<Root>/cmd'
    arg_cmd[1] = rtb_y[1];
  }

  if (rtb_y[2] > 1.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[2] = 1.0F;
  } else if (rtb_y[2] < 0.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[2] = 0.0F;
  } else {
    // Outport: '<Root>/cmd'
    arg_cmd[2] = rtb_y[2];
  }

  if (rtb_y[3] > 1.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[3] = 1.0F;
  } else if (rtb_y[3] < 0.0F) {
    // Outport: '<Root>/cmd'
    arg_cmd[3] = 0.0F;
  } else {
    // Outport: '<Root>/cmd'
    arg_cmd[3] = rtb_y[3];
  }

  // End of Saturate: '<Root>/saturation 0-1'

  // Update for DiscreteIntegrator: '<S8>/Integrator' incorporates:
  //   Inport: '<Root>/param'
  //   Product: '<S8>/IOut'

  rtDW.Integrator_DSTATE += rtb_Sum1 * arg_param[3] * 0.004F;

  // Update for DiscreteIntegrator: '<S8>/Filter'
  rtDW.Filter_DSTATE += 0.004F * rtb_Sum3;

  // Update for DiscreteIntegrator: '<S7>/Integrator' incorporates:
  //   Inport: '<Root>/param'
  //   Product: '<S7>/IOut'

  rtDW.Integrator_DSTATE_m += rtb_Sum2_n * arg_param[4] * 0.004F;

  // Update for DiscreteIntegrator: '<S7>/Filter'
  rtDW.Filter_DSTATE_n += 0.004F * rtb_Sum2;

  // Update for DiscreteIntegrator: '<S9>/Integrator' incorporates:
  //   Inport: '<Root>/param'
  //   Product: '<S9>/IOut'

  rtDW.Integrator_DSTATE_d += rtb_Sum3_f * arg_param[5] * 0.004F;

  // Update for DiscreteIntegrator: '<S9>/Filter'
  rtDW.Filter_DSTATE_a += 0.004F * rtb_NOut_e;
}

// Model initialize function
void quad_ndi_class::initialize()
{
  // (no initialization code required)
}

// Constructor
quad_ndi_class::quad_ndi_class()
{
}

// Destructor
quad_ndi_class::~quad_ndi_class()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * quad_ndi_class::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
