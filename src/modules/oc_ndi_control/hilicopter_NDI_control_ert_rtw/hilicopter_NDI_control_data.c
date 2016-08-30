/*
 * File: hilicopter_NDI_control_data.c
 *
 * Code generated for Simulink model 'hilicopter_NDI_control'.
 *
 * Model version                  : 1.176
 * Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
 * C/C++ source code generated on : Tue Aug 30 09:26:35 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "hilicopter_NDI_control.h"
#include "hilicopter_NDI_control_private.h"

/* Constant parameters (auto storage) */
const ConstP_hilicopter_NDI_control_T hilicopter_NDI_control_ConstP = {
  /* Pooled Parameter (Expression: param)
   * Referenced by:
   *   '<Root>/NDI_control_law'
   *   '<S9>/un2n'
   */
  {
    0.45,

    { 0.45, 0.0, 0.0, 0.31819805153394637, 0.31819805153394637, 0.0, 0.0, 0.45,
      0.0, -0.31819805153394637, 0.31819805153394637, 0.0, -0.45, 0.0, 0.0,
      -0.31819805153394637, -0.31819805153394637, 0.0, 0.0, -0.45, 0.0,
      0.31819805153394637, -0.31819805153394637, 0.0 },
    0.43633231299858238,
    1.28e-7,
    0.0383,
    0.0939,
    5.1337812749862008e-10,
    0.05,
    0.062,
    0.01,
    9.80665,
    3.5,

    { 0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.4 },

    { 5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 2.5 },
    0.1,

    { 0.0, 0.0, 0.0 },
    33518823.242187493,
    5.0,

    { -1.8132053522185005, 0.0, -3.8884314261375459, -0.069445764989968573,
      1.7497941417618956, -0.14892692362106802, 0.0, 0.0, -4.2904093749999994,
      -1.3651999034079765, 1.3651999034079765, 0.16432267906249998, 0.0,
      -1.8132053522185005, -3.8884314261375459, -1.7497941417618956,
      -0.069445764989968573, -0.14892692362106802, 0.0, 0.0, -4.2904093749999994,
      -1.3651999034079765, -1.3651999034079765, 0.16432267906249998,
      1.8132053522185005, 0.0, -3.8884314261375459, 0.069445764989968573,
      -1.7497941417618956, -0.14892692362106802, 0.0, 0.0, -4.2904093749999994,
      1.3651999034079765, -1.3651999034079765, 0.16432267906249998, 0.0,
      1.8132053522185005, -3.8884314261375459, 1.7497941417618956,
      0.069445764989968573, -0.14892692362106802, 0.0, 0.0, -4.2904093749999994,
      1.3651999034079765, 1.3651999034079765, 0.16432267906249998 },

    { -0.27575475628738788, 0.18373282032262669, -9.03551738091901e-17,
      -0.16970557057539509, 0.27575475628738783, -0.18373282032262669,
      1.2301581373595716e-16, 0.1697055705753952, 4.5564540001926093e-17,
      0.16970557057539548, -0.27575475628738821, 0.18373282032262714,
      -1.1749946673989377e-16, -0.16970557057539548, 0.27575475628738816,
      -0.18373282032262714, -0.032146638657368547, -0.029134748942226524,
      -0.032146638657368561, -0.029134748942226524, -0.032146638657368554,
      -0.029134748942226507, -0.03214663865736854, -0.029134748942226528,
      -1.82880470064371e-17, -0.18312336484636435, 2.4365519786606077e-16,
      -0.18312336484636446, 5.977450941251922e-17, 0.18312336484636429,
      -1.8655622417618741e-16, 0.18312336484636443, 1.747088173642737e-17,
      0.18312336484636416, 2.4938324199421288e-17, -0.1831233648463641,
      9.5968634587513972e-18, -0.18312336484636407, 1.344578117107493e-17,
      0.18312336484636421, -0.83933782395218182, 0.76069840580225956,
      -0.83933782395218237, 0.76069840580225978, -0.83933782395218193,
      0.76069840580225878, -0.839337823952181, 0.76069840580225878 }
  },

  /* Computed Parameter: Tnm_Gain
   * Referenced by: '<S9>/Tnm'
   */
  { -0.27575475F, 0.183732823F, -9.0355175E-17F, -0.16970557F, 0.27575475F,
    -0.183732823F, 1.23015813E-16F, 0.16970557F, 4.55645393E-17F, 0.16970557F,
    -0.27575475F, 0.183732823F, -1.17499471E-16F, -0.16970557F, 0.27575475F,
    -0.183732823F, -0.0321466401F, -0.0291347485F, -0.0321466401F,
    -0.0291347485F, -0.0321466401F, -0.0291347485F, -0.0321466401F,
    -0.0291347485F, -1.82880468E-17F, -0.183123365F, 2.43655195E-16F,
    -0.183123365F, 5.97745063E-17F, 0.183123365F, -1.86556227E-16F, 0.183123365F,
    1.74708817E-17F, 0.183123365F, 2.49383242E-17F, -0.183123365F,
    9.59686344E-18F, -0.183123365F, 1.34457812E-17F, 0.183123365F, -0.839337826F,
    0.760698378F, -0.839337826F, 0.760698378F, -0.839337826F, 0.760698378F,
    -0.839337826F, 0.760698378F }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
