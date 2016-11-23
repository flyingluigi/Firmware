/*
 * File: hilicopter_NDI_control.h
 *
 * Code generated for Simulink model 'hilicopter_NDI_control'.
 *
 * Model version                  : 1.494
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Thu Oct 20 15:08:47 2016
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
  real32_T Integrator_DSTATE;          /* '<S19>/Integrator' */
  real32_T Filter_DSTATE;              /* '<S19>/Filter' */
  real32_T Integrator_DSTATE_n;        /* '<S21>/Integrator' */
  real32_T Filter_DSTATE_a;            /* '<S21>/Filter' */
  real32_T Integrator_DSTATE_o;        /* '<S20>/Integrator' */
  real32_T Filter_DSTATE_f;            /* '<S20>/Filter' */
  real32_T Integrator_DSTATE_f;        /* '<S32>/Integrator' */
  real32_T FilterState_DSTATE;         /* '<S32>/Filter State' */
  real32_T Integrator_DSTATE_i;        /* '<S33>/Integrator' */
  real32_T FilterState_DSTATE_k;       /* '<S33>/Filter State' */
  real32_T Integrator_DSTATE_g;        /* '<S34>/Integrator' */
  real32_T FilterState_DSTATE_n;       /* '<S34>/Filter State' */
  real32_T FilterState_DSTATE_l;       /* '<S24>/Filter State' */
  real32_T FilterState_DSTATE_h;       /* '<S23>/Filter State' */
  real32_T Integrator_DSTATE_ig;       /* '<S22>/Integrator' */
  real32_T FilterState_DSTATE_lx;      /* '<S22>/Filter State' */
  int8_T FilterState_PrevResetState;   /* '<S24>/Filter State' */
  int8_T FilterState_PrevResetState_h; /* '<S23>/Filter State' */
  int8_T Integrator_PrevResetState;    /* '<S22>/Integrator' */
  int8_T FilterState_PrevResetState_hn;/* '<S22>/Filter State' */
} DW_hilicopter_NDI_control_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: param)
   * Referenced by:
   *   '<Root>/NDI_control_law'
   *   '<S18>/un2n'
   */
  struct_0N1ZcU4DUQBmM8WiLk7GQE pooled1;

  /* Computed Parameter: Tnm_Gain
   * Referenced by: '<S18>/Tnm'
   */
  real32_T Tnm_Gain[48];
} ConstP_hilicopter_NDI_control_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T state[15];                  /* '<Root>/state' */
  real32_T pwm_in[8];                  /* '<Root>/pwm_in' */
  real32_T param[45];                  /* '<Root>/param' */
} ExtU_hilicopter_NDI_control_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  uint16_T pwm_out[8];                 /* '<Root>/pwm_out' */
  real32_T debug[13];                  /* '<Root>/debug' */
} ExtY_hilicopter_NDI_control_T;

/* Real-time Model Data Structure */
struct tag_RTM_hilicopter_NDI_contro_T {
  const char_T *errorStatus;
};

/* Block states (auto storage) */
extern DW_hilicopter_NDI_control_T hilicopter_NDI_control_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_hilicopter_NDI_control_T hilicopter_NDI_control_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_hilicopter_NDI_control_T hilicopter_NDI_control_Y;

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
 * '<S1>'   : 'hilicopter_NDI_control/NDI_control_law'
 * '<S2>'   : 'hilicopter_NDI_control/angle_control'
 * '<S3>'   : 'hilicopter_NDI_control/horizontal_vel_control'
 * '<S4>'   : 'hilicopter_NDI_control/manual_pos_control'
 * '<S5>'   : 'hilicopter_NDI_control/mode select'
 * '<S6>'   : 'hilicopter_NDI_control/mode_logic_acc'
 * '<S7>'   : 'hilicopter_NDI_control/mode_logic_euler_ref'
 * '<S8>'   : 'hilicopter_NDI_control/mode_logic_horizontal_velocity'
 * '<S9>'   : 'hilicopter_NDI_control/mode_logic_position'
 * '<S10>'  : 'hilicopter_NDI_control/mode_logic_rate'
 * '<S11>'  : 'hilicopter_NDI_control/mode_logic_um'
 * '<S12>'  : 'hilicopter_NDI_control/pos_control'
 * '<S13>'  : 'hilicopter_NDI_control/rate_control'
 * '<S14>'  : 'hilicopter_NDI_control/rotate_u'
 * '<S15>'  : 'hilicopter_NDI_control/rotate_u1'
 * '<S16>'  : 'hilicopter_NDI_control/select_u'
 * '<S17>'  : 'hilicopter_NDI_control/select_x'
 * '<S18>'  : 'hilicopter_NDI_control/um2pwm'
 * '<S19>'  : 'hilicopter_NDI_control/angle_control/angle_control_phi'
 * '<S20>'  : 'hilicopter_NDI_control/angle_control/angle_control_psi'
 * '<S21>'  : 'hilicopter_NDI_control/angle_control/angle_control_theta'
 * '<S22>'  : 'hilicopter_NDI_control/horizontal_vel_control/Discrete_PID'
 * '<S23>'  : 'hilicopter_NDI_control/horizontal_vel_control/Discrete_PID1'
 * '<S24>'  : 'hilicopter_NDI_control/horizontal_vel_control/Discrete_PID2'
 * '<S25>'  : 'hilicopter_NDI_control/horizontal_vel_control/Reset integral on arming'
 * '<S26>'  : 'hilicopter_NDI_control/horizontal_vel_control/Reset integral on mode change'
 * '<S27>'  : 'hilicopter_NDI_control/manual_pos_control/position_integral'
 * '<S28>'  : 'hilicopter_NDI_control/pos_control/x_pos'
 * '<S29>'  : 'hilicopter_NDI_control/pos_control/y_pos'
 * '<S30>'  : 'hilicopter_NDI_control/pos_control/z_pos'
 * '<S31>'  : 'hilicopter_NDI_control/rate_control/Discrete_PID_error_D'
 * '<S32>'  : 'hilicopter_NDI_control/rate_control/Discrete_PID_measurement_D'
 * '<S33>'  : 'hilicopter_NDI_control/rate_control/Discrete_PID_measurement_D1'
 * '<S34>'  : 'hilicopter_NDI_control/rate_control/Discrete_PID_measurement_D2'
 * '<S35>'  : 'hilicopter_NDI_control/um2pwm/un2n'
 */
#endif                                 /* RTW_HEADER_hilicopter_NDI_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
