/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
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
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 * @file oc_ndi_control_params.c
 * Octocopter NDI Control algorithm parameters.
 *
 * @author Marcell Mocher <marcell.mocher@fh-joanneum.at>
 */

/**
 * Roll PID gain
 *
 * Roll PID gains
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_ROLL_P, 0.4f);
PARAM_DEFINE_FLOAT(NDI_ROLL_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_ROLL_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_ROLL_N, 50.0f);

/**
 * Rollrate PID gains
 *
 * Rollrate proportional gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_ROLLRATE_P, 0.125f);
PARAM_DEFINE_FLOAT(NDI_ROLLRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_ROLLRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_ROLLRATE_N, 50.0f);



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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_PITCH_P, 0.4f);
PARAM_DEFINE_FLOAT(NDI_PITCH_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_PITCH_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_PITCH_N, 50.0f);
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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_PITCHRATE_P, 0.125f);
PARAM_DEFINE_FLOAT(NDI_PITCHRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_PITCHRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_PITCHRATE_N, 50.0f);



/**
 * Yaw PID gain
 *
 * Yaw pid gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_YAW_P, 0.5f);
PARAM_DEFINE_FLOAT(NDI_YAW_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_YAW_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_YAW_N, 50.0f);

/**
 * Yawrate PID gain
 *
 * Yawrate pid gain
 *
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_YAWRATE_P, 0.25f);
PARAM_DEFINE_FLOAT(NDI_YAWRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(NDI_YAWRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(NDI_YAWRATE_N, 50.0f);


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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MODEL_IXX, 0.1f);

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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MODEL_IYY, 0.1f);

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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MODEL_IZZ, 0.2f);


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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELX_P, 1.0f);


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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELY_P, 1.0f);


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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELZ_P, 1.0f);



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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELX_I, 0.1f);

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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELY_I, 0.1f);

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
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_VELZ_I, 0.1f);


/**
 *  X Position Control P gain
 *
 * X Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_POSX_P, 1.0f);

/**
 *  Y Position Control P gain
 *
 * Y Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_POSY_P, 1.0f);

/**
 *  Z Position Control P gain
 *
 * Z Position Control proportional gain
 *
 * @min 0.00
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_POSZ_P, 1.0f);


/**
 *  Maximum Rates
 *
 * Maximum rate command in deg
 *
 * @min 0
 * @max 160
 * @decimal 0
 * @increment 1
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MAX_RATE, 90.0f);



/**
 *  Maximum Angles
 *
 * Maximum angle command in deg
 *
 * @min 0
 * @max 60
 * @decimal 0
 * @increment 1
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MAX_ANGLE, 45.0f);


/**
 *  Maximum Velocity
 *
 * Maximum velocity command in m/s
 *
 * @min 0
 * @max 15
 * @decimal 1
 * @increment 0.1
 * @group Octocopter NDI Control
 */
PARAM_DEFINE_FLOAT(NDI_MAX_Velocity, 5.0f);
