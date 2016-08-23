 /*
 * File: test.c
 *
 * Code generated for Simulink model 'test'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Sat Aug  6 13:27:13 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "test.h"
#include "test_private.h"

/* External inputs (root inport signals with auto storage) */
ExtU_test_T test_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_test_T test_Y;

/* Real-time model */
RT_MODEL_test_T test_M_;
RT_MODEL_test_T *const test_M = &test_M_;

/* Model step function */
void test_step(void)
{
  /* Outport: '<Root>/Out1' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/In1'
   */
  test_Y.Out1 = 2.0 * test_U.In1;
}

/* Model initialize function */
void test_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(test_M, (NULL));

  /* external inputs */
  test_U.In1 = 0.0;

  /* external outputs */
  test_Y.Out1 = 0.0;
}

/* Model terminate function */
void test_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
