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
 * @file oc_ndi_control.cpp
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
#include "hilicopter_NDI_control_ert_rtw/hilicopter_NDI_control.h"
#include "hilicopter_NDI_control_ert_rtw/rtwtypes.h"
#ifdef __cplusplus
}
#endif

static bool thread_should_exit = false;		/**< oc_ndi_control exit flag */
static bool thread_running = false;		/**< oc_ndi_control status flag */
static int oc_ndi_control_task;			/**< oc_ndi_control of daemon task / thread */

struct {
        param_t roll_p;
		param_t roll_i;
		param_t roll_d;
		param_t roll_N;
        param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_N;
        param_t pitch_p;
		param_t pitch_i;
		param_t pitch_d;
		param_t pitch_N;
        param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_N;
        param_t yaw_p;
		param_t yaw_i;
		param_t yaw_d;
		param_t yaw_N;
        param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_N;
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
        }   _params_handles;

struct {
        float att_p[3]; /**< P gain for angular error */
		float att_i[3];
		float att_d[3];
		float att_N[3];
        float rate_p[3];    /**< P gain for angular rate error */
		float rate_i[3]; 
		float rate_d[3]; 
		float rate_N[3]; 
        float model_i[3];
        float vel_p[3];
        float vel_i[3];
        float pos_p[3];
        float max_val[3];
} _params;

int rc_channels_sub;
int vehicle_attitude_sub;
int vehicle_local_position_sub;
int control_state_sub;
int params_sub;

orb_advert_t simulink_app_pwm_pub;
orb_advert_t simulink_app_debug_pub;

struct rc_channels_s rc_in;
struct vehicle_attitude_s attitude;
struct vehicle_local_position_s local_position;
struct control_state_s control_state;
struct simulink_app_pwm_s pwm_out;
struct simulink_app_debug_s ndi_debug;

extern "C" __EXPORT int oc_ndi_control_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;


static DevHandle h_rgbleds;


/**
 * Mainloop of oc_ndi_control.
 */
int oc_ndi_control_thread(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Update our local parameter cache.
 */
int parameters_update();

/**
 * Check for parameter update and handle it.
 */
void    parameter_update_poll();

/**
 * Check for rc in updates.
 */
void    rc_channels_poll();

/**
 * Check for attitude updates.
 */


void   attitude_poll();

/**
 * Check for local position updates.
 */
void   vehicle_local_position_poll();

/**
 * Check for attitude updates.
 */
void   control_state_poll();


static void
usage(const char *reason)
{
	if (reason) {
                fprintf(stderr, "%s\n", reason);
	}

        fprintf(stderr, "usage: oc_ndi_control {start|stop|status} [-p <additional params>]\n\n");
}

int parameters_update(){
    float v;

    /* roll gains */
    param_get(_params_handles.roll_p, &v);
	if(v==0)
		_params.att_p[0] = 0;
	else
    	_params.att_p[0] = 1/v;
	
    param_get(_params_handles.roll_i, &v);
    _params.att_i[0] = v;
    param_get(_params_handles.roll_d, &v);
    _params.att_d[0] = v;
    param_get(_params_handles.roll_N, &v);
    _params.att_N[0] = v;
	
    param_get(_params_handles.roll_rate_p, &v);
	if(v==0)
    	_params.rate_p[0] = 0;
	else
		_params.rate_p[0] = 1/v;
	
    param_get(_params_handles.roll_rate_i, &v);
    _params.rate_i[0] = v;	
    param_get(_params_handles.roll_rate_d, &v);
    _params.rate_d[0] = v;	
    param_get(_params_handles.roll_rate_N, &v);
    _params.rate_N[0] = v;
	

    /* pitch gains */
    param_get(_params_handles.pitch_p, &v);
	if(v==0)
    	_params.att_p[1] = 0;
	else
		_params.att_p[1] = 1/v;
	
    param_get(_params_handles.pitch_i, &v);
    _params.att_i[1] = v;
    param_get(_params_handles.pitch_d, &v);
    _params.att_d[1] = v;
    param_get(_params_handles.pitch_N, &v);
    _params.att_N[1] = v;
    param_get(_params_handles.pitch_rate_p, &v);
	if(v==0)
    	_params.rate_p[1]  = 0;
	else
		_params.rate_p[1]  = 1/v;
	
    param_get(_params_handles.pitch_rate_i, &v);
    _params.rate_i[1]  = v;
    param_get(_params_handles.pitch_rate_d, &v);
    _params.rate_d[1]  = v;
    param_get(_params_handles.pitch_rate_N, &v);
    _params.rate_N[1]  = v;
	

    param_get(_params_handles.yaw_p, &v);
	if(v==0)
    	_params.att_p[2] = 0;
	else
		_params.att_p[2] = 1/v;
    param_get(_params_handles.yaw_i, &v);
    _params.att_i[2] = v;
    param_get(_params_handles.yaw_d, &v);
    _params.att_d[2] = v;
    param_get(_params_handles.yaw_N, &v);
    _params.att_N[2] = v;
    param_get(_params_handles.yaw_rate_p, &v);
	if(v==0)
    	_params.rate_p[2] = 0;
	else
		_params.rate_p[2] = 1/v;
    param_get(_params_handles.yaw_rate_i, &v);
    _params.rate_i[2] = v;
    param_get(_params_handles.yaw_rate_d, &v);
    _params.rate_d[2] = v;
    param_get(_params_handles.yaw_rate_N, &v);
    _params.rate_N[2] = v;

    /* velocity gains */
    param_get(_params_handles.vel_x_kp, &v);
    _params.vel_p[0] = v;
    param_get(_params_handles.vel_y_kp, &v);
    _params.vel_p[1] = v;
    param_get(_params_handles.vel_z_kp, &v);
    _params.vel_p[2] = v;


    param_get(_params_handles.vel_x_ki, &v);
    _params.vel_i[0] = v;
    param_get(_params_handles.vel_y_ki, &v);
    _params.vel_i[1] = v;
    param_get(_params_handles.vel_z_ki, &v);
    _params.vel_i[2] = v;


    /* position gains */
    param_get(_params_handles.pos_x_kp, &v);
    _params.pos_p[0] = v;
    param_get(_params_handles.pos_y_kp, &v);
    _params.pos_p[1] = v;
    param_get(_params_handles.pos_z_kp, &v);
    _params.pos_p[2] = v;


    /* model parameters */
    param_get(_params_handles.model_ixx, &v);
    _params.model_i[0] = v;
    param_get(_params_handles.model_iyy, &v);
    _params.model_i[1] = v;
    param_get(_params_handles.model_izz, &v);
    _params.model_i[2] = v;


    /* model max values */
    param_get(_params_handles.max_rate, &v);
    _params.max_val[0] = v * M_PI_F/180.0f;
    param_get(_params_handles.max_angle, &v);
    _params.max_val[1] = v * M_PI_F/180.0f;
    param_get(_params_handles.max_vel, &v);
    _params.max_val[2] = v;

    return OK;
}

void    parameter_update_poll(){

    bool updated;

    /* Check if parameters have changed */
     orb_check(params_sub, &updated);

    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), params_sub, &param_update);
        parameters_update();
        param_save_default(); // this is not save during flight!!
    }
}

void    rc_channels_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(rc_channels_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_in);
    }
}


void   attitude_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(vehicle_attitude_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude);
    }

}

void   vehicle_local_position_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(vehicle_local_position_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_position);
    }

}

void   control_state_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(control_state_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(control_state), control_state_sub, &control_state);
    }

}

/**
 * The oc_ndi_control app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int oc_ndi_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
                return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
                        warnx("already running\n");
			/* this is not an error */
                        return 0;
		}

		thread_should_exit = false;
                oc_ndi_control_task = px4_task_spawn_cmd("oc_ndi_control",
					     SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             7000,
                                             oc_ndi_control_thread,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
                return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
                return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
                        return 0;

		} else {
                        warnx("\tnot started\n");
                        return 1;

		}

                return 0;
	}

	usage("unrecognized command");
        return 1;
}



int oc_ndi_control_thread(int argc, char *argv[])
{

    /* register the perf counter */
    perf_counter_t ndi_loop_perf = perf_alloc(PC_ELAPSED, "oc_ndi_control");

    hilicopter_NDI_control_initialize();

    bool oc_ndi_arm = false;

    /* subscriptions */
    rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    control_state_sub = orb_subscribe(ORB_ID(control_state));
    params_sub = orb_subscribe(ORB_ID(parameter_update));

    /* publications */
    simulink_app_pwm_pub = orb_advertise(ORB_ID(simulink_app_pwm), &pwm_out);
    simulink_app_debug_pub  = orb_advertise(ORB_ID(simulink_app_debug), &ndi_debug);

    // declare output device
    DevMgr::getHandle("/dev/rgbled0",h_rgbleds);
    if (!h_rgbleds.isValid()) {
            PX4_WARN("No RGB LED found at " RGBLED0_DEVICE_PATH);
    }


    memset(&rc_in, 0, sizeof(rc_in));
    memset(&attitude, 0, sizeof(attitude));
    memset(&local_position, 0, sizeof(local_position));
    memset(&control_state, 0, sizeof(control_state));
    memset(&pwm_out, 0, sizeof(pwm_out));
    memset(&ndi_debug, 0, sizeof(ndi_debug));

    /*Find Parameters*/
    _params_handles.roll_p  =   param_find("NDI_ROLL_P");
	_params_handles.roll_i  =   param_find("NDI_ROLL_I");
	_params_handles.roll_d  =   param_find("NDI_ROLL_D");
	_params_handles.roll_N  =   param_find("NDI_ROLL_N");
	
    _params_handles.roll_rate_p =   param_find("NDI_ROLLRATE_P");
	_params_handles.roll_rate_i =   param_find("NDI_ROLLRATE_I");
	_params_handles.roll_rate_d =   param_find("NDI_ROLLRATE_D");
	_params_handles.roll_rate_N =   param_find("NDI_ROLLRATE_N");

    _params_handles.pitch_p =   param_find("NDI_PITCH_P");
	_params_handles.pitch_i =   param_find("NDI_PITCH_I");
	_params_handles.pitch_d =   param_find("NDI_PITCH_D");
	_params_handles.pitch_N =   param_find("NDI_PITCH_N");
	
    _params_handles.pitch_rate_p    =   param_find("NDI_PITCHRATE_P");
	_params_handles.pitch_rate_i    =   param_find("NDI_PITCHRATE_I");
	_params_handles.pitch_rate_d    =   param_find("NDI_PITCHRATE_D");
	_params_handles.pitch_rate_N    =   param_find("NDI_PITCHRATE_N");

    _params_handles.yaw_p   =   param_find("NDI_YAW_P");
	_params_handles.yaw_i   =   param_find("NDI_YAW_I");
	_params_handles.yaw_d   =   param_find("NDI_YAW_D");
	_params_handles.yaw_N   =   param_find("NDI_YAW_N");
	
    _params_handles.yaw_rate_p  =   param_find("NDI_YAWRATE_P");
	_params_handles.yaw_rate_i  =   param_find("NDI_YAWRATE_I");
	_params_handles.yaw_rate_d  =   param_find("NDI_YAWRATE_D");
	_params_handles.yaw_rate_N  =   param_find("NDI_YAWRATE_N");

    _params_handles.model_ixx   =   param_find("NDI_MODEL_IXX");
    _params_handles.model_iyy   =   param_find("NDI_MODEL_IYY");
    _params_handles.model_izz   =   param_find("NDI_MODEL_IZZ");

    _params_handles.vel_x_kp   =    param_find("NDI_VELX_P");
    _params_handles.vel_y_kp   =    param_find("NDI_VELY_P");
    _params_handles.vel_z_kp   =    param_find("NDI_VELZ_P");

    _params_handles.vel_x_ki   =    param_find("NDI_VELX_I");
    _params_handles.vel_y_ki   =    param_find("NDI_VELY_I");
    _params_handles.vel_z_ki   =    param_find("NDI_VELZ_I");

    _params_handles.pos_x_kp   =    param_find("NDI_POSX_P");
    _params_handles.pos_y_kp   =    param_find("NDI_POSY_P");
    _params_handles.pos_z_kp   =    param_find("NDI_POSZ_P");

    _params_handles.max_rate =     param_find("NDI_MAX_RATE");
    _params_handles.max_angle =     param_find("NDI_MAX_ANGLE");
    _params_handles.max_vel =     param_find("NDI_MAX_Velocity");

    /* initialize parameters cache*/
        parameters_update();


    // orb_set_interval(control_state_sub, 4); // Fundamental step size in ms

    /* wakeup source: vehicle attitude */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = vehicle_attitude_sub;
    fds[0].events = POLLIN;


    thread_running = true;
    /* set leds to disarmed*/
    h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
    h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
    while (!thread_should_exit) {
        /* wait for up to 100ms for data */
        int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            continue;
        }
        if (poll_ret < 0) {
            /* this is seriously brc_inad - should be an emergency */
            warn("oc_ndi_control: poll error %d, %d", poll_ret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;

        }

        if (fds[0].revents & POLLIN) {
            perf_begin(ndi_loop_perf);
           
            /* copy attitude topic */
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude);

            /* check for updates in other topics */
            parameter_update_poll();
            rc_channels_poll();
            vehicle_local_position_poll();
            control_state_poll();

            hilicopter_NDI_control_U.param[0] = _params.rate_p[0];
            hilicopter_NDI_control_U.param[1] = _params.rate_p[1];
            hilicopter_NDI_control_U.param[2] = _params.rate_p[2];
            hilicopter_NDI_control_U.param[3] = _params.rate_i[0];
            hilicopter_NDI_control_U.param[4] = _params.rate_i[1];
            hilicopter_NDI_control_U.param[5] = _params.rate_i[2];
            hilicopter_NDI_control_U.param[6] = _params.rate_d[0];
            hilicopter_NDI_control_U.param[7] = _params.rate_d[1];
            hilicopter_NDI_control_U.param[8] = _params.rate_d[2];
            hilicopter_NDI_control_U.param[9] = _params.rate_N[0];
            hilicopter_NDI_control_U.param[10] = _params.rate_N[1];
            hilicopter_NDI_control_U.param[11] = _params.rate_N[2];
			
            hilicopter_NDI_control_U.param[12] = _params.att_p[0];
            hilicopter_NDI_control_U.param[13] = _params.att_p[1];
            hilicopter_NDI_control_U.param[14] = _params.att_p[2];
            hilicopter_NDI_control_U.param[15] = _params.att_i[0];
            hilicopter_NDI_control_U.param[16] = _params.att_i[1];
            hilicopter_NDI_control_U.param[17] = _params.att_i[2];
            hilicopter_NDI_control_U.param[18] = _params.att_d[0];
            hilicopter_NDI_control_U.param[19] = _params.att_d[1];
            hilicopter_NDI_control_U.param[20] = _params.att_d[2];
            hilicopter_NDI_control_U.param[21] = _params.att_N[0];
            hilicopter_NDI_control_U.param[22] = _params.att_N[1];
            hilicopter_NDI_control_U.param[23] = _params.att_N[2];
			
            hilicopter_NDI_control_U.param[24] = _params.model_i[0];
            hilicopter_NDI_control_U.param[25] = _params.model_i[1];
            hilicopter_NDI_control_U.param[26] = _params.model_i[2];
            hilicopter_NDI_control_U.param[27] = _params.vel_p[0];
            hilicopter_NDI_control_U.param[28] = _params.vel_p[1];
            hilicopter_NDI_control_U.param[29] = _params.vel_p[2];
            hilicopter_NDI_control_U.param[30] = _params.vel_i[0];
            hilicopter_NDI_control_U.param[31] = _params.vel_i[1];
            hilicopter_NDI_control_U.param[32] = _params.vel_i[2];
            hilicopter_NDI_control_U.param[33] = _params.pos_p[0];
            hilicopter_NDI_control_U.param[34] = _params.pos_p[1];
            hilicopter_NDI_control_U.param[35] = _params.pos_p[2];
            hilicopter_NDI_control_U.param[36] = _params.max_val[0];
            hilicopter_NDI_control_U.param[37] = _params.max_val[1];
            hilicopter_NDI_control_U.param[38] = _params.max_val[2];
			
            //Quaternion _q = control_state.q;
            //Vector<3> euler = _q.to_euler();

            hilicopter_NDI_control_U.state[0] = control_state.x_vel;
            hilicopter_NDI_control_U.state[1] = control_state.y_vel;
            hilicopter_NDI_control_U.state[2] = control_state.z_vel;
            hilicopter_NDI_control_U.state[3] = local_position.x;
            hilicopter_NDI_control_U.state[4] = local_position.y;
            hilicopter_NDI_control_U.state[5] = local_position.z;
            hilicopter_NDI_control_U.state[6] = attitude.rollspeed;
            hilicopter_NDI_control_U.state[7] = attitude.pitchspeed;
            hilicopter_NDI_control_U.state[8] = attitude.yawspeed;
            hilicopter_NDI_control_U.state[9] = attitude.roll;
            hilicopter_NDI_control_U.state[10] = attitude.pitch;
            hilicopter_NDI_control_U.state[11] = attitude.yaw;
            hilicopter_NDI_control_U.state[12] = local_position.vx;
            hilicopter_NDI_control_U.state[13] = local_position.vy;
            hilicopter_NDI_control_U.state[14] = local_position.vz;

            hilicopter_NDI_control_U.pwm_in[0] = rc_in.channels[0];
            hilicopter_NDI_control_U.pwm_in[1] = rc_in.channels[1];
            hilicopter_NDI_control_U.pwm_in[2] = rc_in.channels[2];
            hilicopter_NDI_control_U.pwm_in[3] = rc_in.channels[3];
            hilicopter_NDI_control_U.pwm_in[4] = rc_in.channels[4];
            hilicopter_NDI_control_U.pwm_in[5] = rc_in.channels[5];
            hilicopter_NDI_control_U.pwm_in[6] = rc_in.channels[6];
            hilicopter_NDI_control_U.pwm_in[7] = rc_in.channels[7];

            /*Check arming status*/
            if(rc_in.channels[4] > 0.5f)
                oc_ndi_arm = true;
            else
                oc_ndi_arm = false;

            /* Run code from codegeneration */
            hilicopter_NDI_control_step();

            /* publish actuator controls */

            if(oc_ndi_arm == true){
                pwm_out.arm = true;
                pwm_out.timestamp = hrt_absolute_time();
                pwm_out.pwm[0] = hilicopter_NDI_control_Y.pwm_out[0];
                pwm_out.pwm[1] = hilicopter_NDI_control_Y.pwm_out[1];
                pwm_out.pwm[2] = hilicopter_NDI_control_Y.pwm_out[2];
                pwm_out.pwm[3] = hilicopter_NDI_control_Y.pwm_out[3];
                pwm_out.pwm[4] = hilicopter_NDI_control_Y.pwm_out[4];
                pwm_out.pwm[5] = hilicopter_NDI_control_Y.pwm_out[5];
                pwm_out.pwm[6] = hilicopter_NDI_control_Y.pwm_out[6];
                pwm_out.pwm[7] = hilicopter_NDI_control_Y.pwm_out[7];
                h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BLINK_FAST);
                h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_RED);
            } else {
                pwm_out.arm = false;
                pwm_out.timestamp = hrt_absolute_time();
                pwm_out.pwm[0] = 1000;
                pwm_out.pwm[1] = 1000;
                pwm_out.pwm[2] = 1000;
                pwm_out.pwm[3] = 1000;
                pwm_out.pwm[4] = 1000;
                pwm_out.pwm[5] = 1000;
                pwm_out.pwm[6] = 1000;
                pwm_out.pwm[7] = 1000;
                h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
                h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
            }
            orb_publish(ORB_ID(simulink_app_pwm), simulink_app_pwm_pub, &pwm_out);

            /* publish debug controls */
            ndi_debug.timestamp = hrt_absolute_time();
            ndi_debug.debug[0] = hilicopter_NDI_control_Y.debug[0];
            ndi_debug.debug[1] = hilicopter_NDI_control_Y.debug[1];
            ndi_debug.debug[2] = hilicopter_NDI_control_Y.debug[2];
            ndi_debug.debug[3] = hilicopter_NDI_control_Y.debug[3];
            ndi_debug.debug[4] = hilicopter_NDI_control_Y.debug[4];
            ndi_debug.debug[5] = hilicopter_NDI_control_Y.debug[5];
            ndi_debug.debug[6] = hilicopter_NDI_control_Y.debug[6];
            ndi_debug.debug[7] = hilicopter_NDI_control_Y.debug[7];
            ndi_debug.debug[8] = hilicopter_NDI_control_Y.debug[8];
            ndi_debug.debug[9] = hilicopter_NDI_control_Y.debug[9];
            ndi_debug.debug[10] = hilicopter_NDI_control_Y.debug[10];
            ndi_debug.debug[11] = hilicopter_NDI_control_Y.debug[11];
            ndi_debug.debug[12] = hilicopter_NDI_control_Y.debug[12];

            orb_publish(ORB_ID(simulink_app_debug), simulink_app_debug_pub, &ndi_debug);

            perf_end(ndi_loop_perf);
            //PX4_WARN("%0.5f",(double)dt);
            }

    }


    warnx("[oc_ndi_control] exiting.\n");

    thread_running = false;

    return 0;
}
