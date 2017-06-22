//
// File: quad_ndi.cpp
//
// Code generated for Simulink model 'quad_ndi'.
//
// Model version                  : 1.577
// Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
// C/C++ source code generated on : Thu Jun 22 10:09:51 2017
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
  arg_cmd[4])
{
  real32_T y[3];
  static const real32_T a[9] = { 0.002F, 0.0F, 0.0F, 0.0F, 0.002F, 0.0F, 0.0F,
    0.0F, 0.003F };

  real32_T rtb_error;
  real32_T rtb_FilterState;
  real32_T rtb_NOut;
  real32_T rtb_Iout_j;
  real32_T rtb_NOut_o;
  real32_T rtb_NOut_a;
  real32_T rtb_Sum2;
  real32_T rtb_Sum2_i;
  real32_T rtb_Sum2_l;
  int32_T i;
  real32_T a_0[3];

  // Outputs for Atomic SubSystem: '<Root>/control_attitude_rates'
  // Outputs for Atomic SubSystem: '<Root>/control_attitude'
  // Sum: '<S15>/error' incorporates:
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Product: '<S12>/POut'
  //   Sum: '<S7>/Sum5'

  rtb_FilterState = (arg_vehicle_attitude_setpoint[0] - arg_vehicle_attitude[0])
    * arg_param[15] - arg_vehicle_attitude[3];

  // End of Outputs for SubSystem: '<Root>/control_attitude'

  // Product: '<S15>/NOut' incorporates:
  //   DiscreteIntegrator: '<S15>/Filter State'
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Product: '<S15>/Dout'
  //   Sum: '<S15>/Sum1'

  rtb_NOut = ((0.0F - arg_vehicle_attitude[3] * arg_param[6]) -
              rtDW.FilterState_DSTATE) * arg_param[9];

  // Sum: '<S15>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S15>/Integrator'
  //   Inport: '<Root>/param'
  //   Product: '<S15>/Pout'

  rtb_Sum2 = (rtb_FilterState * arg_param[0] + rtDW.Integrator_DSTATE) +
    rtb_NOut;

  // Outputs for Atomic SubSystem: '<Root>/control_attitude'
  // Sum: '<S16>/error' incorporates:
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   Product: '<S11>/POut'
  //   Sum: '<S7>/Sum6'

  rtb_error = (arg_vehicle_attitude_setpoint[1] - arg_vehicle_attitude[1]) *
    arg_param[16] - arg_vehicle_attitude[4];

  // End of Outputs for SubSystem: '<Root>/control_attitude'

  // Product: '<S16>/Iout' incorporates:
  //   Inport: '<Root>/param'

  rtb_Iout_j = rtb_error * arg_param[4];

  // Product: '<S16>/NOut' incorporates:
  //   DiscreteIntegrator: '<S16>/Filter State'
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Product: '<S16>/Dout'
  //   Sum: '<S16>/Sum1'

  rtb_NOut_o = ((0.0F - arg_vehicle_attitude[4] * arg_param[7]) -
                rtDW.FilterState_DSTATE_e) * arg_param[10];

  // Sum: '<S16>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S16>/Integrator'
  //   Inport: '<Root>/param'
  //   Product: '<S16>/Pout'

  rtb_Sum2_i = (rtb_error * arg_param[1] + rtDW.Integrator_DSTATE_f) +
    rtb_NOut_o;

  // Sum: '<S17>/error' incorporates:
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'

  rtb_error = arg_vehicle_attitude_setpoint[2] - arg_vehicle_attitude[5];

  // Product: '<S17>/NOut' incorporates:
  //   DiscreteIntegrator: '<S17>/Filter State'
  //   Inport: '<Root>/param'
  //   Inport: '<Root>/vehicle_attitude'
  //   Product: '<S17>/Dout'
  //   Sum: '<S17>/Sum1'

  rtb_NOut_a = ((0.0F - arg_vehicle_attitude[5] * arg_param[8]) -
                rtDW.FilterState_DSTATE_f) * arg_param[10];

  // Sum: '<S17>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S17>/Integrator'
  //   Inport: '<Root>/param'
  //   Product: '<S17>/Pout'

  rtb_Sum2_l = (rtb_error * arg_param[2] + rtDW.Integrator_DSTATE_fp) +
    rtb_NOut_a;

  // Update for DiscreteIntegrator: '<S15>/Filter State'
  rtDW.FilterState_DSTATE += 0.004F * rtb_NOut;

  // Update for DiscreteIntegrator: '<S15>/Integrator' incorporates:
  //   Inport: '<Root>/param'
  //   Product: '<S15>/Iout'

  rtDW.Integrator_DSTATE += rtb_FilterState * arg_param[3] * 0.004F;

  // Update for DiscreteIntegrator: '<S16>/Filter State'
  rtDW.FilterState_DSTATE_e += 0.004F * rtb_NOut_o;

  // Update for DiscreteIntegrator: '<S16>/Integrator'
  rtDW.Integrator_DSTATE_f += 0.004F * rtb_Iout_j;

  // Update for DiscreteIntegrator: '<S17>/Filter State'
  rtDW.FilterState_DSTATE_f += 0.004F * rtb_NOut_a;

  // Update for DiscreteIntegrator: '<S17>/Integrator' incorporates:
  //   Inport: '<Root>/param'
  //   Product: '<S17>/Iout'

  rtDW.Integrator_DSTATE_fp += rtb_error * arg_param[5] * 0.004F;

  // End of Outputs for SubSystem: '<Root>/control_attitude_rates'

  // MATLAB Function: '<Root>/NDI LAW' incorporates:
  //   Inport: '<Root>/vehicle_attitude'
  //   SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport1'

  // MATLAB Function 'NDI LAW': '<S3>:1'
  // '<S3>:1:4'
  //  NDI control Law
  // '<S3>:1:4'
  for (i = 0; i < 3; i++) {
    y[i] = a[i + 6] * arg_vehicle_attitude[5] + (a[i + 3] *
      arg_vehicle_attitude[4] + a[i] * arg_vehicle_attitude[3]);
    a_0[i] = a[i + 6] * rtb_Sum2_l + (a[i + 3] * rtb_Sum2_i + a[i] * rtb_Sum2);
  }

  // SignalConversion: '<Root>/TmpSignal ConversionAtGain1Inport1' incorporates:
  //   Gain: '<Root>/Gain2'
  //   Inport: '<Root>/vehicle_attitude'
  //   Inport: '<Root>/vehicle_attitude_setpoint'
  //   MATLAB Function: '<Root>/NDI LAW'

  rtb_FilterState = 9.80665F * arg_vehicle_attitude_setpoint[3];
  rtb_NOut = (arg_vehicle_attitude[4] * y[2] - arg_vehicle_attitude[5] * y[1]) +
    a_0[0];
  rtb_Sum2 = (arg_vehicle_attitude[5] * y[0] - arg_vehicle_attitude[3] * y[2]) +
    a_0[1];
  rtb_error = (arg_vehicle_attitude[3] * y[1] - arg_vehicle_attitude[4] * y[0])
    + a_0[2];

  // Outport: '<Root>/cmd' incorporates:
  //   Gain: '<Root>/Gain1'
  //   SignalConversion: '<Root>/TmpSignal ConversionAtGain1Inport1'

  for (i = 0; i < 4; i++) {
    arg_cmd[i] = 0.0F;
    arg_cmd[i] += rtConstP.Gain1_Gain[i] * rtb_FilterState;
    arg_cmd[i] += rtConstP.Gain1_Gain[i + 4] * rtb_NOut;
    arg_cmd[i] += rtConstP.Gain1_Gain[i + 8] * rtb_Sum2;
    arg_cmd[i] += rtConstP.Gain1_Gain[i + 12] * rtb_error;
  }

  // End of Outport: '<Root>/cmd'
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
