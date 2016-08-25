/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following coQCtions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of coQCtions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of coQCtions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IQCRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file oc_ndi_control.params.c
 * Quadcopter Control algorithm parameters.
 *
 * @author Marcell Mocher <marcell.mocher@fh-joanneum.at>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_ROLL_P, 0.5f);

/**
 * Rollrate P gain
 *
 * Rollrate proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_ROLLRATE_P, 3.0f);

/**
 * Roll Feedforward P gain
 *
 * Roll Feedforward proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_ROLLRATE_FF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_PITCH_P, 0.5f);

/**
 * Pitchrate P gain
 *
 * Pitchrate proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_PITCHRATE_P, 3.0f);

/**
 * Pitch Feedforward P gain
 *
 * Pitch Feedforward proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_PITCHRATE_FF, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_YAW_P, 0.5f);

/**
 * Yawrate P gain
 *
 * Yawrate proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_YAWRATE_P, 3.0f);

/**
 * Yaw Feedforward P gain
 *
 * Yaw Feedforward proportional gain
 *
 *
 * @min 0.00
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_YAWRATE_FF, 0.2f);


/**
 * Moment of inertia IXX
 *
 * Moment of inertia diagonal entry (1,1)
 *
 * @unit kg *m^2
 * @min 0.00
 * @max 1
 * @decimal 4
 * @increment 0.0001
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MODEL_IXX, 0.1f);

/**
 * Moment of inertia IYY
 *
 * Moment of inertia diagonal entry (2,2)
 *
 * @unit kg *m^2
 * @min 0.00
 * @max 1
 * @decimal 4
 * @increment 0.0001
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MODEL_IYY, 0.1f);

/**
 * Moment of inertia IZZ
 *
 * Moment of inertia diagonal entry (3,3)
 *
 * @unit kg *m^2
 * @min 0.00
 * @max 1
 * @decimal 4
 * @increment 0.0001
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MODEL_IZZ, 0.2f);


/**
 *  X Velocity Control P gain
 *
 * X Velocity Control proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELX_P, 1.0f);


/**
 *  Y Velocity Control P gain
 *
 * Y Velocity Control proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELY_P, 1.0f);


/**
 *  Z Velocity Control P gain
 *
 * Z Velocity Control proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELZ_P, 1.0f);



/**
 *  X Velocity Control I gain
 *
 * X Velocity Control integral gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELX_I, 0.1f);

/**
 *  Y Velocity Control I gain
 *
 * Y Velocity Control integral gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELY_I, 0.1f);

/**
 *  Z Velocity Control I gain
 *
 * Z Velocity Control integral gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_VELZ_I, 0.1f);


/**
 *  X Position Control P gain
 *
 * X Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_POSX_P, 1.0f);

/**
 *  Y Position Control P gain
 *
 * Y Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_POSY_P, 1.0f);

/**
 *  Z Position Control P gain
 *
 * Z Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_POSZ_P, 1.0f);


/**
 *  Maximum Rates
 *
 * Maximum rate command in deg
 *
 * @min 0
 * @max 160
 * @decimal 0
 * @increment 1
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MAX_RATE, 90.0f);



/**
 *  Maximum Angles
 *
 * Maximum angle command in deg
 *
 * @min 0
 * @max 60
 * @decimal 0
 * @increment 1
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MAX_ANGLE, 45.0f);


/**
 *  Maximum Velocity
 *
 * Maximum velocity command in m/s
 *
 * @min 0
 * @max 15
 * @decimal 1
 * @increment 0.1
 * @group Octocopter QC Control
 */
PARAM_DEFINE_FLOAT(QC_MAX_Velocity, 5.0f);
