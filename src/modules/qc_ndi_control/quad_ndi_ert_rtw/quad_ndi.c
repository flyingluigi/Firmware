/*
 * File: quad_ndi.c
 *
 * Code generated for Simulink model 'quad_ndi'.
 *
 * Model version                  : 1.66
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Sat Aug 27 12:55:15 2016
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

/* Model step function */
void quad_ndi_step(void)
{
  real32_T Ib[9];
  real32_T y[3];
  real32_T rtb_POut_l;
  real32_T rtb_POut_f;
  real32_T rtb_NOut;
  real32_T rtb_Sum4;
  real32_T rtb_NOut_h;
  real32_T Ib_0[3];
  int32_T j;
  real32_T rtb_POut_d_idx_0;
  real32_T rtb_POut_d_idx_1;
  real32_T rtb_POut_d_idx_2;
  real32_T tmp;
  uint16_T u0;
  real32_T rtb_Saturation0unmax;
  real32_T rtb_pwm;

  /* Product: '<S5>/POut' incorporates:
   *  Inport: '<Root>/attitude'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/u_cmd'
   *  Product: '<Root>/Product'
   *  Sum: '<Root>/Sum5'
   */
  rtb_POut_l = (quad_ndi_U.u_cmd[0] * quad_ndi_U.param[22] -
                quad_ndi_U.attitude[0]) * quad_ndi_U.param[0];

  /* Product: '<S1>/NOut' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  Product: '<S1>/DOut'
   *  Product: '<S1>/cOut'
   *  Sum: '<S1>/Sum3'
   *  Sum: '<S1>/SumD'
   */
  rtb_NOut = ((rtb_POut_l * quad_ndi_U.param[7] - quad_ndi_U.rates[0]) *
              quad_ndi_U.param[16] - quad_ndi_DW.Filter_DSTATE) *
    quad_ndi_U.param[17];

  /* Product: '<S4>/POut' incorporates:
   *  Gain: '<Root>/Gain2'
   *  Inport: '<Root>/attitude'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/u_cmd'
   *  Product: '<Root>/Product1'
   *  Sum: '<Root>/Sum6'
   */
  rtb_POut_f = (-quad_ndi_U.u_cmd[1] * quad_ndi_U.param[22] -
                quad_ndi_U.attitude[1]) * quad_ndi_U.param[0];

  /* Product: '<S2>/NOut' incorporates:
   *  DiscreteIntegrator: '<S2>/Filter'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  Product: '<S2>/DOut'
   *  Product: '<S2>/cOut'
   *  Sum: '<S2>/Sum3'
   *  Sum: '<S2>/SumD'
   */
  rtb_NOut_h = ((rtb_POut_f * quad_ndi_U.param[7] - quad_ndi_U.rates[1]) *
                quad_ndi_U.param[16] - quad_ndi_DW.Filter_DSTATE_a) *
    quad_ndi_U.param[17];

  /* Sum: '<Root>/Sum4' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  Inport: '<Root>/u_cmd'
   *  Product: '<Root>/Product2'
   */
  rtb_Sum4 = quad_ndi_U.u_cmd[2] * quad_ndi_U.param[21] - quad_ndi_U.rates[2];

  /* MATLAB Function: '<Root>/NDI LAW' incorporates:
   *  Inport: '<Root>/param'
   */
  /* MATLAB Function 'NDI LAW': '<S3>:1' */
  /*  NDI control Law */
  /* '<S3>:1:3' */
  for (j = 0; j < 9; j++) {
    Ib[j] = 0.0F;
  }

  Ib[0] = quad_ndi_U.param[9];
  Ib[4] = quad_ndi_U.param[10];
  Ib[8] = quad_ndi_U.param[11];

  /* SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  DiscreteIntegrator: '<S2>/Integrator'
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  MATLAB Function: '<Root>/NDI LAW'
   *  Product: '<S1>/POut'
   *  Product: '<S1>/bOut'
   *  Product: '<S2>/POut'
   *  Product: '<S2>/bOut'
   *  Product: '<S6>/POut'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S2>/Sum'
   *  Sum: '<S2>/Sum1'
   */
  /* '<S3>:1:4' */
  rtb_POut_d_idx_0 = ((rtb_POut_l * quad_ndi_U.param[6] - quad_ndi_U.rates[0]) *
                      quad_ndi_U.param[3] + quad_ndi_DW.Integrator_DSTATE) +
    rtb_NOut;
  rtb_POut_d_idx_1 = ((rtb_POut_f * quad_ndi_U.param[6] - quad_ndi_U.rates[1]) *
                      quad_ndi_U.param[3] + quad_ndi_DW.Integrator_DSTATE_k) +
    rtb_NOut_h;
  rtb_POut_d_idx_2 = rtb_Sum4 * quad_ndi_U.param[5];

  /* MATLAB Function: '<Root>/NDI LAW' incorporates:
   *  Inport: '<Root>/rates'
   */
  for (j = 0; j < 3; j++) {
    y[j] = Ib[j + 6] * quad_ndi_U.rates[2] + (Ib[j + 3] * quad_ndi_U.rates[1] +
      Ib[j] * quad_ndi_U.rates[0]);
    Ib_0[j] = Ib[j + 6] * rtb_POut_d_idx_2 + (Ib[j + 3] * rtb_POut_d_idx_1 +
      Ib[j] * rtb_POut_d_idx_0);
  }

  /* SignalConversion: '<S7>/TmpSignal ConversionAtTnmInport1' incorporates:
   *  Gain: '<Root>/Gain3'
   *  Inport: '<Root>/rates'
   *  Inport: '<Root>/u_cmd'
   *  MATLAB Function: '<Root>/NDI LAW'
   */
  rtb_POut_d_idx_0 = -7.84532F * quad_ndi_U.u_cmd[3];
  rtb_POut_d_idx_1 = (quad_ndi_U.rates[1] * y[2] - quad_ndi_U.rates[2] * y[1]) +
    Ib_0[0];
  rtb_POut_d_idx_2 = (quad_ndi_U.rates[2] * y[0] - quad_ndi_U.rates[0] * y[2]) +
    Ib_0[1];
  tmp = (quad_ndi_U.rates[0] * y[1] - quad_ndi_U.rates[1] * y[0]) + Ib_0[2];

  /* MATLAB Function 'um2pwm/un2n': '<S8>:1' */
  /* '<S8>:1:4' */
  /* '<S8>:1:5' */
  for (j = 0; j < 4; j++) {
    /* Gain: '<S7>/Tnm' incorporates:
     *  Saturate: '<S7>/Saturation 0 - unmax'
     *  SignalConversion: '<S7>/TmpSignal ConversionAtTnmInport1'
     */
    rtb_Saturation0unmax = quad_ndi_ConstP.Tnm_Gain[j + 12] * tmp +
      (quad_ndi_ConstP.Tnm_Gain[j + 8] * rtb_POut_d_idx_2 +
       (quad_ndi_ConstP.Tnm_Gain[j + 4] * rtb_POut_d_idx_1 +
        quad_ndi_ConstP.Tnm_Gain[j] * rtb_POut_d_idx_0));

    /* Saturate: '<S7>/Saturation 0 - unmax' */
    if (rtb_Saturation0unmax > 5.0F) {
      rtb_Saturation0unmax = 5.0F;
    } else {
      if (rtb_Saturation0unmax < 0.0F) {
        rtb_Saturation0unmax = 0.0F;
      }
    }

    /* MATLAB Function: '<S7>/un2n' */
    rtb_pwm = rtb_Saturation0unmax;
    if (rtb_Saturation0unmax > 0.0F) {
      rtb_pwm = 1.0F;
    } else {
      if (rtb_Saturation0unmax == 0.0F) {
        rtb_pwm = 0.0F;
      }
    }

    rtb_pwm = rtb_pwm * (real32_T)sqrt(1.56598208E+8F * rtb_Saturation0unmax) *
      0.0638F + 804.0F;

    /* End of MATLAB Function: '<S7>/un2n' */

    /* DataTypeConversion: '<S7>/Data Type Conversion' */
    rtb_Saturation0unmax = (real32_T)floor(rtb_pwm);
    if (rtIsNaNF(rtb_Saturation0unmax) || rtIsInfF(rtb_Saturation0unmax)) {
      u0 = 0U;
    } else {
      u0 = (uint16_T)(real32_T)fmod(rtb_Saturation0unmax, 65536.0F);
    }

    /* End of DataTypeConversion: '<S7>/Data Type Conversion' */

    /* Saturate: '<Root>/wrap' */
    if (u0 > 2000) {
      /* Outport: '<Root>/pwm' */
      quad_ndi_Y.pwm[j] = 2000U;
    } else if (u0 < 1000) {
      /* Outport: '<Root>/pwm' */
      quad_ndi_Y.pwm[j] = 1000U;
    } else {
      /* Outport: '<Root>/pwm' */
      quad_ndi_Y.pwm[j] = u0;
    }

    /* End of Saturate: '<Root>/wrap' */
  }

  /* Outport: '<Root>/debug' incorporates:
   *  Inport: '<Root>/rates'
   *  Sum: '<Root>/Add'
   *  Sum: '<Root>/Add1'
   */
  quad_ndi_Y.debug[0] = rtb_POut_l - quad_ndi_U.rates[0];
  quad_ndi_Y.debug[1] = rtb_POut_f - quad_ndi_U.rates[1];
  quad_ndi_Y.debug[2] = rtb_Sum4;

  /* Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  Product: '<S1>/IOut'
   *  Sum: '<S1>/Sum2'
   */
  quad_ndi_DW.Integrator_DSTATE += (rtb_POut_l - quad_ndi_U.rates[0]) *
    quad_ndi_U.param[15] * 0.004F;

  /* Update for DiscreteIntegrator: '<S1>/Filter' */
  quad_ndi_DW.Filter_DSTATE += 0.004F * rtb_NOut;

  /* Update for DiscreteIntegrator: '<S2>/Integrator' incorporates:
   *  Inport: '<Root>/param'
   *  Inport: '<Root>/rates'
   *  Product: '<S2>/IOut'
   *  Sum: '<S2>/Sum2'
   */
  quad_ndi_DW.Integrator_DSTATE_k += (rtb_POut_f - quad_ndi_U.rates[1]) *
    quad_ndi_U.param[15] * 0.004F;

  /* Update for DiscreteIntegrator: '<S2>/Filter' */
  quad_ndi_DW.Filter_DSTATE_a += 0.004F * rtb_NOut_h;
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
