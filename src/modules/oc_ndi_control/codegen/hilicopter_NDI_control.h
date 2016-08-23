/*
 * File: hilicopter_NDI_control.h
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

#ifndef RTW_HEADER_hilicopter_NDI_control_h_
#define RTW_HEADER_hilicopter_NDI_control_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef hilicopter_NDI_control_COMMON_INCLUDES_
# define hilicopter_NDI_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* hilicopter_NDI_control_COMMON_INCLUDES_ */

#include "hilicopter_NDI_control_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S14>/Integrator' */
  real32_T Integrator_DSTATE_k;        /* '<S15>/Integrator' */
  real32_T Integrator_DSTATE_a;        /* '<S16>/Integrator' */
} DW_hilicopter_NDI_control_T;

/* Invariant block signals (auto storage) */
typedef struct {
  const real32_T max_vel_com[2];       /* '<S8>/max_vel_com' */
} ConstB_hilicopter_NDI_control_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: param)
   * Referenced by:
   *   '<Root>/Rate_Velocity_Controller'
   *   '<S7>/un2n'
   */
  struct_LsE4pJyaGkFOziDPkuTr6 pooled1;

  /* Computed Parameter: Tnm_Gain
   * Referenced by: '<S7>/Tnm'
   */
  real32_T Tnm_Gain[48];
} ConstP_hilicopter_NDI_control_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T state[15];                  /* '<Root>/state' */
  real32_T pwm_in[8];                  /* '<Root>/pwm_in' */
  real32_T param[24];                  /* '<Root>/param' */
} ExtU_hilicopter_NDI_control_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  uint16_T pwm_out[8];                 /* '<Root>/pwm_out' */
  real32_T debug[13];                  /* '<Root>/debug' */
} ExtY_hilicopter_NDI_control_T;

/* Real-time Model Data Structure */
struct tag_RTM_hilicopter_NDI_contro_T {
  const char_T * volatile errorStatus;
};

/* Block states (auto storage) */
extern DW_hilicopter_NDI_control_T hilicopter_NDI_control_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_hilicopter_NDI_control_T hilicopter_NDI_control_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_hilicopter_NDI_control_T hilicopter_NDI_control_Y;
extern const ConstB_hilicopter_NDI_control_T hilicopter_NDI_control_ConstB;/* constant block i/o */

/* Constant parameters (auto storage) */
extern const ConstP_hilicopter_NDI_control_T hilicopter_NDI_control_ConstP;

/* Model entry point functions */
extern void hilicopter_NDI_control_initialize(void);
extern void hilicopter_NDI_control_step(void);
extern void hilicopter_NDI_control_terminate(void);

/* Real-time Model object */
extern RT_MODEL_hilicopter_NDI_contr_T *const hilicopter_NDI_control_M;

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
 * '<Root>' : 'hilicopter_NDI_control'
 * '<S1>'   : 'hilicopter_NDI_control/Attitude_Controller'
 * '<S2>'   : 'hilicopter_NDI_control/Rate_Velocity_Controller'
 * '<S3>'   : 'hilicopter_NDI_control/add_man_thrust_vel'
 * '<S4>'   : 'hilicopter_NDI_control/rate_control'
 * '<S5>'   : 'hilicopter_NDI_control/select_u'
 * '<S6>'   : 'hilicopter_NDI_control/select_x'
 * '<S7>'   : 'hilicopter_NDI_control/um2pwm'
 * '<S8>'   : 'hilicopter_NDI_control/vel_control'
 * '<S9>'   : 'hilicopter_NDI_control/add_man_thrust_vel/norm'
 * '<S10>'  : 'hilicopter_NDI_control/rate_control/rate_p'
 * '<S11>'  : 'hilicopter_NDI_control/rate_control/rate_q'
 * '<S12>'  : 'hilicopter_NDI_control/rate_control/rate_r'
 * '<S13>'  : 'hilicopter_NDI_control/um2pwm/un2n'
 * '<S14>'  : 'hilicopter_NDI_control/vel_control/vel_u'
 * '<S15>'  : 'hilicopter_NDI_control/vel_control/vel_v'
 * '<S16>'  : 'hilicopter_NDI_control/vel_control/vel_w'
 */
#endif                                 /* RTW_HEADER_hilicopter_NDI_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
