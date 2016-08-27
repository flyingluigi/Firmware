/*
 * File: quad_ndi.h
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

#ifndef RTW_HEADER_quad_ndi_h_
#define RTW_HEADER_quad_ndi_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef quad_ndi_COMMON_INCLUDES_
# define quad_ndi_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* quad_ndi_COMMON_INCLUDES_ */

#include "quad_ndi_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S1>/Integrator' */
  real32_T Filter_DSTATE;              /* '<S1>/Filter' */
  real32_T Integrator_DSTATE_k;        /* '<S2>/Integrator' */
  real32_T Filter_DSTATE_a;            /* '<S2>/Filter' */
} DW_quad_ndi_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Computed Parameter: Tnm_Gain
   * Referenced by: '<S7>/Tnm'
   */
  real32_T Tnm_Gain[16];
} ConstP_quad_ndi_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T attitude[3];                /* '<Root>/attitude' */
  real32_T rates[3];                   /* '<Root>/rates' */
  real32_T u_cmd[4];                   /* '<Root>/u_cmd' */
  real32_T param[24];                  /* '<Root>/param' */
} ExtU_quad_ndi_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  uint16_T pwm[4];                     /* '<Root>/pwm' */
  real32_T debug[3];                   /* '<Root>/debug' */
} ExtY_quad_ndi_T;

/* Real-time Model Data Structure */
struct tag_RTM_quad_ndi_T {
  const char_T * volatile errorStatus;
};

/* Block states (auto storage) */
extern DW_quad_ndi_T quad_ndi_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_quad_ndi_T quad_ndi_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_quad_ndi_T quad_ndi_Y;

/* Constant parameters (auto storage) */
extern const ConstP_quad_ndi_T quad_ndi_ConstP;

/* Model entry point functions */
extern void quad_ndi_initialize(void);
extern void quad_ndi_step(void);
extern void quad_ndi_terminate(void);

/* Real-time Model object */
extern RT_MODEL_quad_ndi_T *const quad_ndi_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'quad_ndi'
 * '<S1>'   : 'quad_ndi/Discrete PID Controller (2DOF)'
 * '<S2>'   : 'quad_ndi/Discrete PID Controller (2DOF)1'
 * '<S3>'   : 'quad_ndi/NDI LAW'
 * '<S4>'   : 'quad_ndi/PID Controller1'
 * '<S5>'   : 'quad_ndi/PID Controller2'
 * '<S6>'   : 'quad_ndi/PID Controller5'
 * '<S7>'   : 'quad_ndi/um2pwm'
 * '<S8>'   : 'quad_ndi/um2pwm/un2n'
 */
#endif                                 /* RTW_HEADER_quad_ndi_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
