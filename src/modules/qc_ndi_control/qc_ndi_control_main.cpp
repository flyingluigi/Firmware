/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file qc_ndi_control.cpp
 * Octocopter control of all states using ndi control law
 *
 * @author Marcell Mocher <marcell.mocher@fh-joanneum.at>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <math.h>
#include <errno.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>

#include "uORB/topics/simulink_app_pwm.h"
#include "uORB/topics/simulink_app_debug.h"

#include <lib/mathlib/mathlib.h>

#include <DevMgr.hpp>

using namespace DriverFramework;

#ifdef __cplusplus
extern "C" {
#endif
#include "quad_ndi_ert_rtw/quad_ndi.h"
#include "quad_ndi_ert_rtw/rtwtypes.h"
#ifdef __cplusplus
}
#endif

static bool qc_thread_should_exit = false;		/**< qc_ndi_control exit flag */
static bool qc_thread_running = false;		/**< qc_ndi_control status flag */
static int qc_ndi_control_task;			/**< qc_ndi_control of daemon task / thread */

struct {
        param_t roll_p;
        param_t roll_rate_p;
        param_t roll_rate_ff;
        param_t pitch_p;
        param_t pitch_rate_p;
        param_t pitch_rate_ff;
        param_t yaw_p;
        param_t yaw_rate_p;
        param_t yaw_rate_ff;
        param_t model_ixx;
        param_t model_iyy;
        param_t model_izz;
        param_t vel_x_kp;
        param_t vel_y_kp;
        param_t vel_z_kp;
        param_t vel_x_ki;
        param_t vel_y_ki;
        param_t vel_z_ki;
        param_t pos_x_kp;
        param_t pos_y_kp;
        param_t pos_z_kp;
        param_t max_rate;
        param_t max_vel;
        param_t max_angle;
        }   _params_qc_handles_qc;

struct {
        float att_p[3]; /**< P gain for angular error */
        float rate_p[3];    /**< P gain for angular rate error */
        float rate_ff[3];   /**< Feedforward gain for desired rates */
        float model_i[3];
        float vel_p[3];
        float vel_i[3];
        float pos_p[3];
        float max_val[3];
} _params_qc;

int qc_rc_channels_sub;
int qc_vehicle_attitude_sub;
int qc_vehicle_local_position_sub;
int qc_control_state_sub;
int qc_params_qc_sub;

orb_advert_t qc_simulink_app_pwm_pub;
orb_advert_t qc_simulink_app_debug_pub;

struct rc_channels_s qc_rc_in;
struct vehicle_attitude_s qc_attitude;
struct vehicle_local_position_s qc_local_position;
struct control_state_s qc_control_state;
struct simulink_app_pwm_s qc_pwm_out;
struct simulink_app_debug_s qc_ndi_debug;

extern "C" __EXPORT int qc_ndi_control_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;


static DevHandle qc_h_rgbleds;


/**
 * Mainloop of qc_ndi_control.
 */
int qc_ndi_control_thread(int argc, char *argv[]);

/**
 * Print the correct qc_usage.
 */
static void qc_usage(const char *reason);

/**
 * Update our local parameter cache.
 */
int qc_parameters_update();

/**
 * Check for parameter update and handle it.
 */
void    qc_parameter_update_poll();

/**
 * Check for rc in updates.
 */
void    qc_rc_channels_poll();

/**
 * Check for attitude updates.
 */


void   qc_attitude_poll();

/**
 * Check for local position updates.
 */
void   qc_vehicle_local_position_poll();

/**
 * Check for attitude updates.
 */
void   qc_control_state_poll();


static void
qc_usage(const char *reason)
{
	if (reason) {
                fprintf(stderr, "%s\n", reason);
	}

        fprintf(stderr, "qc_usage: qc_ndi_control {start|stop|status} [-p <additional params>]\n\n");
}

int qc_parameters_update(){
    float v;

    /* roll gains */
    param_get(_params_qc_handles_qc.roll_p, &v);
    _params_qc.att_p[0] = 1/v;
    param_get(_params_qc_handles_qc.roll_rate_p, &v);
    _params_qc.rate_p[0] = 1/v;
    param_get(_params_qc_handles_qc.roll_rate_ff, &v);
    _params_qc.rate_ff[0] = v;

    /* pitch gains */
    param_get(_params_qc_handles_qc.pitch_p, &v);
    _params_qc.att_p[1] = 1/v;
    param_get(_params_qc_handles_qc.pitch_rate_p, &v);
    _params_qc.rate_p[1]  = 1/v;
    param_get(_params_qc_handles_qc.pitch_rate_ff, &v);
    _params_qc.rate_ff[1]  = v;

    param_get(_params_qc_handles_qc.yaw_p, &v);
    _params_qc.att_p[2] = 1/v;
    param_get(_params_qc_handles_qc.yaw_rate_p, &v);
    _params_qc.rate_p[2] = 1/v;
    param_get(_params_qc_handles_qc.yaw_rate_ff, &v);
    _params_qc.rate_ff[2] = v;

    /* velocity gains */
    param_get(_params_qc_handles_qc.vel_x_kp, &v);
    _params_qc.vel_p[0] = 1/v;
    param_get(_params_qc_handles_qc.vel_y_kp, &v);
    _params_qc.vel_p[1] = 1/v;
    param_get(_params_qc_handles_qc.vel_z_kp, &v);
    _params_qc.vel_p[2] = 1/v;


    param_get(_params_qc_handles_qc.vel_x_ki, &v);
    _params_qc.vel_i[0] = v;
    param_get(_params_qc_handles_qc.vel_y_ki, &v);
    _params_qc.vel_i[1] = v;
    param_get(_params_qc_handles_qc.vel_z_ki, &v);
    _params_qc.vel_i[2] = v;


    /* position gains */
    param_get(_params_qc_handles_qc.pos_x_kp, &v);
    _params_qc.pos_p[0] = 1/v;
    param_get(_params_qc_handles_qc.pos_y_kp, &v);
    _params_qc.pos_p[1] = 1/v;
    param_get(_params_qc_handles_qc.pos_z_kp, &v);
    _params_qc.pos_p[2] = 1/v;


    /* model parameters */
    param_get(_params_qc_handles_qc.model_ixx, &v);
    _params_qc.model_i[0] = v;
    param_get(_params_qc_handles_qc.model_iyy, &v);
    _params_qc.model_i[1] = v;
    param_get(_params_qc_handles_qc.model_izz, &v);
    _params_qc.model_i[2] = v;


    /* model max values */
    param_get(_params_qc_handles_qc.max_rate, &v);
    _params_qc.max_val[0] = v * M_PI_F/180.0f;
    param_get(_params_qc_handles_qc.max_angle, &v);
    _params_qc.max_val[1] = v * M_PI_F/180.0f;
    param_get(_params_qc_handles_qc.max_vel, &v);
    _params_qc.max_val[2] = v;

    return OK;
}

void    qc_parameter_update_poll(){

    bool updated;

    /* Check if parameters have changed */
     orb_check(qc_params_qc_sub, &updated);

    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), qc_params_qc_sub, &param_update);
        qc_parameters_update();
        param_save_default();
    }
}

void    qc_rc_channels_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(qc_rc_channels_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(rc_channels), qc_rc_channels_sub, &qc_rc_in);
    }
}


void   qc_attitude_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(qc_vehicle_attitude_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_attitude), qc_vehicle_attitude_sub, &qc_attitude);
    }

}

void   qc_vehicle_local_position_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(qc_vehicle_local_position_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_local_position), qc_vehicle_local_position_sub, &qc_local_position);
    }

}

void   qc_control_state_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(qc_control_state_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(control_state), qc_control_state_sub, &qc_control_state);
    }

}

/**
 * The qc_ndi_control app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int qc_ndi_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		qc_usage("missing command");
                return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (qc_thread_running) {
                        warnx("already running\n");
			/* this is not an error */
                        return 0;
		}

		qc_thread_should_exit = false;
                qc_ndi_control_task = px4_task_spawn_cmd("qc_ndi_control",
					     SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             7000,
                                             qc_ndi_control_thread,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
                return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		qc_thread_should_exit = true;
                return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (qc_thread_running) {
			warnx("\trunning\n");
                        return 0;

		} else {
                        warnx("\tnot started\n");
                        return 1;

		}

                return 0;
	}

	qc_usage("unrecognized command");
        return 1;
}



int qc_ndi_control_thread(int argc, char *argv[])
{

    /* register the perf counter */
    perf_counter_t ndi_loop_perf = perf_alloc(PC_ELAPSED, "qc_ndi_control");

    quad_ndi_initialize();

    bool qc_ndi_arm = false;

    /* subscriptions */
    qc_rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    qc_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    qc_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    qc_control_state_sub = orb_subscribe(ORB_ID(control_state));
    qc_params_qc_sub = orb_subscribe(ORB_ID(parameter_update));

    /* publications */
    qc_simulink_app_pwm_pub = orb_advertise(ORB_ID(simulink_app_pwm), &qc_pwm_out);
    qc_simulink_app_debug_pub  = orb_advertise(ORB_ID(simulink_app_debug), &qc_ndi_debug);

    // declare output device
    DevMgr::getHandle("/dev/rgbled0",qc_h_rgbleds);
    if (!qc_h_rgbleds.isValid()) {
            PX4_WARN("No RGB LED found at " RGBLED0_DEVICE_PATH);
    }


    memset(&qc_rc_in, 0, sizeof(qc_rc_in));
    memset(&qc_attitude, 0, sizeof(qc_attitude));
    memset(&qc_local_position, 0, sizeof(qc_local_position));
    memset(&qc_control_state, 0, sizeof(qc_control_state));
    memset(&qc_pwm_out, 0, sizeof(qc_pwm_out));
    memset(&qc_ndi_debug, 0, sizeof(qc_ndi_debug));

    /*Find Parameters*/
    _params_qc_handles_qc.roll_p  =   param_find("QC_ROLL_P");
    _params_qc_handles_qc.roll_rate_p =   param_find("QC_ROLLRATE_P");
    _params_qc_handles_qc.roll_rate_ff    =   param_find("QC_ROLLRATE_FF");

    _params_qc_handles_qc.pitch_p =   param_find("QC_PITCH_P");
    _params_qc_handles_qc.pitch_rate_p    =   param_find("QC_PITCHRATE_P");
    _params_qc_handles_qc.pitch_rate_ff   =   param_find("QC_PITCHRATE_FF");

    _params_qc_handles_qc.yaw_p   =   param_find("QC_YAW_P");
    _params_qc_handles_qc.yaw_rate_p  =   param_find("QC_YAWRATE_P");
    _params_qc_handles_qc.yaw_rate_ff =   param_find("QC_YAWRATE_FF");

    _params_qc_handles_qc.model_ixx   =   param_find("QC_MODEL_IXX");
    _params_qc_handles_qc.model_iyy   =   param_find("QC_MODEL_IYY");
    _params_qc_handles_qc.model_izz   =   param_find("QC_MODEL_IZZ");

    _params_qc_handles_qc.vel_x_kp   =    param_find("QC_VELX_P");
    _params_qc_handles_qc.vel_y_kp   =    param_find("QC_VELY_P");
    _params_qc_handles_qc.vel_z_kp   =    param_find("QC_VELZ_P");

    _params_qc_handles_qc.vel_x_ki   =    param_find("QC_VELX_I");
    _params_qc_handles_qc.vel_y_ki   =    param_find("QC_VELY_I");
    _params_qc_handles_qc.vel_z_ki   =    param_find("QC_VELZ_I");

    _params_qc_handles_qc.pos_x_kp   =    param_find("QC_POSX_P");
    _params_qc_handles_qc.pos_y_kp   =    param_find("QC_POSY_P");
    _params_qc_handles_qc.pos_z_kp   =    param_find("QC_POSZ_P");

    _params_qc_handles_qc.max_rate =     param_find("QC_MAX_RATE");
    _params_qc_handles_qc.max_angle =     param_find("QC_MAX_ANGLE");
    _params_qc_handles_qc.max_vel =     param_find("QC_MAX_Velocity");

    /* initialize parameters cache*/
        qc_parameters_update();


    // orb_set_interval(qc_control_state_sub, 4); // Fundamental step size in ms

    /* wakeup source: vehicle attitude */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = qc_vehicle_attitude_sub;
    fds[0].events = POLLIN;


    qc_thread_running = true;
    /* set leds to disarmed*/
    qc_h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
    qc_h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
    while (!qc_thread_should_exit) {
        /* wait for up to 100ms for data */
        int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            continue;
        }
        if (poll_ret < 0) {
            /* this is seriously bqc_qc_qc_qc_qc_qc_qc_qc_qc_qc_qc_qc_qc_rc_inad - should be an emergency */
            warn("qc_ndi_control: poll error %d, %d", poll_ret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;

        }

        if (fds[0].revents & POLLIN) {
            perf_begin(ndi_loop_perf);

            static uint64_t last_run = 0;
            float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
            last_run = hrt_absolute_time();

            /* guard against too small (< 2ms) and too large (> 20ms) dt's */
            if (dt < 0.002f) {
                    dt = 0.002f;

            } else if (dt > 0.02f) {
                    dt = 0.02f;
            }

            /* copy attitude topic */
            orb_copy(ORB_ID(vehicle_attitude), qc_vehicle_attitude_sub, &qc_attitude);

            /* check for updates in other topics */
            qc_parameter_update_poll();
            qc_rc_channels_poll();
            qc_vehicle_local_position_poll();
            qc_control_state_poll();
			
            quad_ndi_U.param[0] = _params_qc.att_p[0];
            quad_ndi_U.param[1] = _params_qc.att_p[1];
            quad_ndi_U.param[2] = _params_qc.att_p[2];
            quad_ndi_U.param[3] = _params_qc.rate_p[0];
            quad_ndi_U.param[4] = _params_qc.rate_p[1];
            quad_ndi_U.param[5] = _params_qc.rate_p[2];
            quad_ndi_U.param[6] = _params_qc.rate_ff[0];
            quad_ndi_U.param[7] = _params_qc.rate_ff[1];
            quad_ndi_U.param[8] = _params_qc.rate_ff[2];
            quad_ndi_U.param[9] = _params_qc.model_i[0];
            quad_ndi_U.param[10] = _params_qc.model_i[1];
            quad_ndi_U.param[11] = _params_qc.model_i[2];
            quad_ndi_U.param[12] = _params_qc.vel_p[0];
            quad_ndi_U.param[13] = _params_qc.vel_p[1];
            quad_ndi_U.param[14] = _params_qc.vel_p[2];
            quad_ndi_U.param[15] = _params_qc.vel_i[0];
            quad_ndi_U.param[16] = _params_qc.vel_i[1];
            quad_ndi_U.param[17] = _params_qc.vel_i[2];
            quad_ndi_U.param[18] = _params_qc.pos_p[0];
            quad_ndi_U.param[19] = _params_qc.pos_p[1];
            quad_ndi_U.param[20] = _params_qc.pos_p[2];
            quad_ndi_U.param[21] = _params_qc.max_val[0];
            quad_ndi_U.param[22] = _params_qc.max_val[1];
            quad_ndi_U.param[23] = _params_qc.max_val[2];
          
            quad_ndi_U.attitude[0] = qc_attitude.roll;
            quad_ndi_U.attitude[1] = qc_attitude.pitch;
            quad_ndi_U.attitude[2] = qc_attitude.yaw;
		  
            quad_ndi_U.rates[0] = qc_attitude.rollspeed;
            quad_ndi_U.rates[1] = qc_attitude.pitchspeed;
            quad_ndi_U.rates[2] = qc_attitude.yawspeed;
			
            quad_ndi_U.u_cmd[0] = qc_rc_in.channels[0];
            quad_ndi_U.u_cmd[1] = qc_rc_in.channels[1];
            quad_ndi_U.u_cmd[2] = qc_rc_in.channels[3];
			quad_ndi_U.u_cmd[3] = qc_rc_in.channels[2];
					  
					  
            //main control loop
            /*Check arming status*/
			 if(qc_rc_in.channels[4] > 0.5f)
                qc_ndi_arm = true;
			 else
				 qc_ndi_arm = false;
			
			
			quad_ndi_step();
				
            /* publish actuator controls */

            if(qc_ndi_arm == true){
                qc_pwm_out.arm = true;
                qc_pwm_out.timestamp = hrt_absolute_time();
                qc_pwm_out.pwm[0] = quad_ndi_Y.pwm[0];
                qc_pwm_out.pwm[1] = quad_ndi_Y.pwm[1];
                qc_pwm_out.pwm[2] = quad_ndi_Y.pwm[2];
                qc_pwm_out.pwm[3] = quad_ndi_Y.pwm[3];
                qc_h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BLINK_FAST);
                qc_h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_RED);
            } else {
                qc_pwm_out.arm = false;
                qc_pwm_out.timestamp = hrt_absolute_time();
                qc_pwm_out.pwm[0] = 1000;
                qc_pwm_out.pwm[1] = 1000;
                qc_pwm_out.pwm[2] = 1000;
                qc_pwm_out.pwm[3] = 1000;
                qc_h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
                qc_h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
            }
            orb_publish(ORB_ID(simulink_app_pwm), qc_simulink_app_pwm_pub, &qc_pwm_out);

            /* publish debug controls */
            qc_ndi_debug.timestamp = hrt_absolute_time();
            qc_ndi_debug.debug[0] = quad_ndi_Y.debug[0];
            qc_ndi_debug.debug[1] = quad_ndi_Y.debug[1];
            qc_ndi_debug.debug[2] = quad_ndi_Y.debug[2];

            orb_publish(ORB_ID(simulink_app_debug), qc_simulink_app_debug_pub, &qc_ndi_debug);

            
            perf_end(ndi_loop_perf);
            PX4_WARN("%0.1f %0.1f %0.1f %0.1f",(double)qc_ndi_debug.debug[0],(double)qc_ndi_debug.debug[1],(double)qc_ndi_debug.debug[3],(double)quad_ndi_U.u_cmd[3]);
            }

    }


    warnx("[qc_ndi_control] exiting.\n");

    qc_thread_running = false;

    return 0;
}
