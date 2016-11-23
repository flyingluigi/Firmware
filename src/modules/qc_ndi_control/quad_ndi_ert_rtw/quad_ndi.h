/*
 * File: quad_ndi.h
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
#include "rt_defines.h"
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
  real32_T rates_int[3];               /* '<Root>/rate_control' */
  real32_T rates_prev[3];              /* '<Root>/rate_control' */
} DW_quad_ndi_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T state[15];                  /* '<Root>/state' */
  real32_T pwm_in[8];                  /* '<Root>/pwm_in' */
  real32_T param[45];                  /* '<Root>/param' */
} ExtU_quad_ndi_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T pwm[4];                     /* '<Root>/pwm' */
  real32_T debug[4];                   /* '<Root>/debug' */
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
 * '<S1>'   : 'quad_ndi/Deadband'
 * '<S2>'   : 'quad_ndi/Deadband1'
 * '<S3>'   : 'quad_ndi/Discrete_PID_measurement_D'
 * '<S4>'   : 'quad_ndi/MATLAB Function'
 * '<S5>'   : 'quad_ndi/NDI LAW'
 * '<S6>'   : 'quad_ndi/Subsystem'
 * '<S7>'   : 'quad_ndi/Subsystem1'
 * '<S8>'   : 'quad_ndi/angle_control'
 * '<S9>'   : 'quad_ndi/attitude_controler'
 * '<S10>'  : 'quad_ndi/commander_logic'
 * '<S11>'  : 'quad_ndi/mixer'
 * '<S12>'  : 'quad_ndi/mode select'
 * '<S13>'  : 'quad_ndi/mode_logic_attitude'
 * '<S14>'  : 'quad_ndi/mode_logic_autoland'
 * '<S15>'  : 'quad_ndi/mode_logic_velocity'
 * '<S16>'  : 'quad_ndi/mode_select_thrust'
 * '<S17>'  : 'quad_ndi/mode_select_z'
 * '<S18>'  : 'quad_ndi/omega_control'
 * '<S19>'  : 'quad_ndi/position_control'
 * '<S20>'  : 'quad_ndi/position_logic'
 * '<S21>'  : 'quad_ndi/rate_control'
 * '<S22>'  : 'quad_ndi/reset_logic'
 * '<S23>'  : 'quad_ndi/saturation'
 * '<S24>'  : 'quad_ndi/select_u'
 * '<S25>'  : 'quad_ndi/select_x1'
 * '<S26>'  : 'quad_ndi/um2pwm'
 * '<S27>'  : 'quad_ndi/velocity_control'
 * '<S28>'  : 'quad_ndi/attitude_controler/PID Controller1'
 * '<S29>'  : 'quad_ndi/attitude_controler/PID Controller2'
 * '<S30>'  : 'quad_ndi/attitude_controler/PID Controller3'
 * '<S31>'  : 'quad_ndi/mode_logic_attitude/Compare To Constant3'
 * '<S32>'  : 'quad_ndi/mode_logic_velocity/Compare To Constant3'
 * '<S33>'  : 'quad_ndi/mode_logic_velocity/VKn2VKpsi'
 * '<S34>'  : 'quad_ndi/omega_control/Discrete_PID_measurement_D'
 * '<S35>'  : 'quad_ndi/omega_control/Discrete_PID_measurement_D1'
 * '<S36>'  : 'quad_ndi/omega_control/Discrete_PID_measurement_D2'
 * '<S37>'  : 'quad_ndi/position_control/position_setpoint'
 * '<S38>'  : 'quad_ndi/position_control/x_pos'
 * '<S39>'  : 'quad_ndi/position_control/y_pos'
 * '<S40>'  : 'quad_ndi/position_control/z_pos'
 * '<S41>'  : 'quad_ndi/reset_logic/Detect Fall Nonpositive'
 * '<S42>'  : 'quad_ndi/reset_logic/Detect Rise Positive'
 * '<S43>'  : 'quad_ndi/reset_logic/Detect Fall Nonpositive/Nonpositive'
 * '<S44>'  : 'quad_ndi/reset_logic/Detect Rise Positive/Positive'
 * '<S45>'  : 'quad_ndi/um2pwm/un2n'
 * '<S46>'  : 'quad_ndi/velocity_control/MATLAB Function'
 * '<S47>'  : 'quad_ndi/velocity_control/PID Controller'
 * '<S48>'  : 'quad_ndi/velocity_control/PID Controller1'
 * '<S49>'  : 'quad_ndi/velocity_control/PID Controller2'
 * '<S50>'  : 'quad_ndi/velocity_control/PID Controller/Clamping circuit'
 * '<S51>'  : 'quad_ndi/velocity_control/PID Controller1/Clamping circuit'
 * '<S52>'  : 'quad_ndi/velocity_control/PID Controller2/Clamping circuit'
 */
#endif                                 /* RTW_HEADER_quad_ndi_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
