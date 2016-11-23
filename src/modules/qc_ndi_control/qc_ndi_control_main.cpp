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

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>

//#include <sys/types.h>
//#include <sys/ioctl.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>

#include "uORB/topics/simulink_app_pwm.h"
#include "uORB/topics/simulink_app_debug.h"

#include <lib/mathlib/mathlib.h>




#ifdef __cplusplus
extern "C" {
#endif
#include "quad_ndi_ert_rtw/quad_ndi.h"
#include "quad_ndi_ert_rtw/rtwtypes.h"
#ifdef __cplusplus
}
#endif


/**
 * qc_ndi_control management function.
 */
extern "C" __EXPORT int qc_ndi_control_main(int argc, char *argv[]);


class QcNdiControl
{
public:
	/**
 	* Constructor
 	*/
	QcNdiControl();
	
	/**
 	* Destructor, also kills the main task
 	*/	
	~QcNdiControl();
	
	
	/**
 	* Start the px4 pwm out task.
 	*
 	* @return		OK on success.
 	*/
	int start();

private:
	
	bool _task_should_exit;		/**< if true, task_main() should exit */
	int _control_task;			/**< qc_ndi_control of task */

	int _rc_channels_sub;
	int _vehicle_attitude_sub;
	int _vehicle_local_position_sub;
	int _control_state_sub;
	int _params_sub;

	orb_advert_t _simulink_app_pwm_pub;
	orb_advert_t _simulink_app_debug_pub;
   	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

   	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

    bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

        
	struct rc_channels_s _rc_in;
	struct vehicle_attitude_s _attitude;
	struct vehicle_local_position_s _local_position;
	struct control_state_s _control_state;
	struct simulink_app_pwm_s _pwm_out;
	struct simulink_app_debug_s _ndi_debug;
    struct parameter_update_s param_update;
    struct actuator_controls_s			_actuators;			/**< actuator controls */

	perf_counter_t _loop_perf;

	
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
        param_t vel_x_kd;
        param_t vel_y_kd;
        param_t vel_z_kd;
        param_t vel_x_N;
        param_t vel_y_N;
        param_t vel_z_N;
        param_t pos_x_kp;
        param_t pos_y_kp;
        param_t pos_z_kp;
        param_t max_rate;
        param_t max_vel;
        param_t max_angle;
        }   _params_handles;


struct {
        math::Vector<3> att_p; /**< P gain for angular error */
		math::Vector<3>  att_i;
		math::Vector<3>  att_d;
		math::Vector<3>  att_N;
        math::Vector<3>  rate_p;    /**< P gain for angular rate error */
		math::Vector<3>  rate_i; 
		math::Vector<3>  rate_d; 
		math::Vector<3>  rate_N; 
        math::Vector<3>  model_i;
        math::Vector<3>  vel_p;
        math::Vector<3>  vel_i;
        math::Vector<3>  vel_d;
        math::Vector<3>  vel_N;
        math::Vector<3>  pos_p;
        math::Vector<3>  max_val;
} _params;


/**
 * Update our local parameter cache.
 */
int parameters_update();

/**
 * save parameters.
 */
void parameters_save();

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

/**
* Shim for calling task_main from task_create.
*/
static void	task_main_trampoline(int argc, char *argv[]);

/**
 * Main qc_ndi_control out task.
 */
void task_main();



};


namespace qc_ndi_control
{
	
	QcNdiControl *g_control;
}


QcNdiControl::QcNdiControl() :

	_task_should_exit(false),
	_control_task(-1),
	
	/* subscriptions */
	_rc_channels_sub(-1),
	_vehicle_attitude_sub(-1),
	_vehicle_local_position_sub(-1),
	_control_state_sub(-1),
	_params_sub(-1),
		
	/* publications */	
	_simulink_app_pwm_pub(nullptr),
	_simulink_app_debug_pub(nullptr),
    _actuators_0_pub(nullptr),
    _actuators_id(0),
	_actuators_0_circuit_breaker_enabled(false),

            
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "qc_ndi_control"))
	
	

{
	memset(&_rc_in, 0, sizeof(_rc_in));
	memset(&_attitude, 0, sizeof(_attitude));
	memset(&_local_position, 0, sizeof(_local_position));
	memset(&_control_state, 0, sizeof(_control_state));
	memset(&_pwm_out, 0, sizeof(_pwm_out));	
	memset(&_ndi_debug, 0, sizeof(_ndi_debug));			
    memset(&_actuators, 0, sizeof(_actuators));

    _params.att_p.zero(); /**< P gain for angular error */
	_params.att_i.zero();
	_params.att_d.zero();
	_params.att_N.zero();
    _params.rate_p.zero();    /**< P gain for angular rate error */
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_N.zero();
    _params.model_i.zero();
    _params.vel_p.zero();
    _params.vel_i.zero();
    _params.vel_d.zero();
    _params.vel_N.zero();
    _params.pos_p.zero();
    _params.max_val.zero();
	

    /*Find Parameters*/
    _params_handles.roll_p  =   param_find("QC_ROLL_P");
	_params_handles.roll_i  =   param_find("QC_ROLL_I");
	_params_handles.roll_d  =   param_find("QC_ROLL_D");
	_params_handles.roll_N  =   param_find("QC_ROLL_N");
	
    _params_handles.roll_rate_p =   param_find("QC_ROLLRATE_P");
	_params_handles.roll_rate_i =   param_find("QC_ROLLRATE_I");
	_params_handles.roll_rate_d =   param_find("QC_ROLLRATE_D");
	_params_handles.roll_rate_N =   param_find("QC_ROLLRATE_N");

    _params_handles.pitch_p =   param_find("QC_PITCH_P");
	_params_handles.pitch_i =   param_find("QC_PITCH_I");
	_params_handles.pitch_d =   param_find("QC_PITCH_D");
	_params_handles.pitch_N =   param_find("QC_PITCH_N");
	
    _params_handles.pitch_rate_p    =   param_find("QC_PITCHRATE_P");
	_params_handles.pitch_rate_i    =   param_find("QC_PITCHRATE_I");
	_params_handles.pitch_rate_d    =   param_find("QC_PITCHRATE_D");
	_params_handles.pitch_rate_N    =   param_find("QC_PITCHRATE_N");

    _params_handles.yaw_p   =   param_find("QC_YAW_P");
	_params_handles.yaw_i   =   param_find("QC_YAW_I");
	_params_handles.yaw_d   =   param_find("QC_YAW_D");
	_params_handles.yaw_N   =   param_find("QC_YAW_N");
	
    _params_handles.yaw_rate_p  =   param_find("QC_YAWRATE_P");
	_params_handles.yaw_rate_i  =   param_find("QC_YAWRATE_I");
	_params_handles.yaw_rate_d  =   param_find("QC_YAWRATE_D");
	_params_handles.yaw_rate_N  =   param_find("QC_YAWRATE_N");

    _params_handles.model_ixx   =   param_find("QC_MODEL_IXX");
    _params_handles.model_iyy   =   param_find("QC_MODEL_IYY");
    _params_handles.model_izz   =   param_find("QC_MODEL_IZZ");

    _params_handles.vel_x_kp   =    param_find("QC_VELX_P");
    _params_handles.vel_y_kp   =    param_find("QC_VELY_P");
    _params_handles.vel_z_kp   =    param_find("QC_VELZ_P");

    _params_handles.vel_x_ki   =    param_find("QC_VELX_I");
    _params_handles.vel_y_ki   =    param_find("QC_VELY_I");
    _params_handles.vel_z_ki   =    param_find("QC_VELZ_I");

    _params_handles.vel_x_kd   =    param_find("QC_VELX_D");
    _params_handles.vel_y_kd   =    param_find("QC_VELY_D");
    _params_handles.vel_z_kd   =    param_find("QC_VELZ_D");
    
    _params_handles.vel_x_N   =    param_find("QC_VELX_N");
    _params_handles.vel_y_N   =    param_find("QC_VELY_N");
    _params_handles.vel_z_N   =    param_find("QC_VELZ_N");
    
    _params_handles.pos_x_kp   =    param_find("QC_POSX_P");
    _params_handles.pos_y_kp   =    param_find("QC_POSY_P");
    _params_handles.pos_z_kp   =    param_find("QC_POSZ_P");

    _params_handles.max_rate =     param_find("QC_MAX_RATE");
    _params_handles.max_angle =     param_find("QC_MAX_ANGLE");
    _params_handles.max_vel =     param_find("QC_MAX_VEL");

    /* initialize parameters cache */
    parameters_update();
	
}


QcNdiControl::~QcNdiControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}
	
	qc_ndi_control::g_control = nullptr;
	
}

int 
QcNdiControl::parameters_update(){
    float v;

    /* roll gains */
    param_get(_params_handles.roll_p, &v);
	if(v==0)
		_params.att_p(0) = 0;
	else
    	_params.att_p(0) = v;
	
    param_get(_params_handles.roll_i, &v);
    _params.att_i(0) = v;
    param_get(_params_handles.roll_d, &v);
    _params.att_d(0) = v;
    param_get(_params_handles.roll_N, &v);
    _params.att_N(0) = v;
	
    param_get(_params_handles.roll_rate_p, &v);
	if(v==0)
    	_params.rate_p(0) = 0;
	else
		_params.rate_p(0) = v;
	
    param_get(_params_handles.roll_rate_i, &v);
    _params.rate_i(0) = v;	
    param_get(_params_handles.roll_rate_d, &v);
    _params.rate_d(0) = v;	
    param_get(_params_handles.roll_rate_N, &v);
    _params.rate_N(0) = v;
	

    /* pitch gains */
    param_get(_params_handles.pitch_p, &v);
	if(v==0)
    	_params.att_p(1) = 0;
	else
		_params.att_p(1) = v;
	
    param_get(_params_handles.pitch_i, &v);
    _params.att_i(1) = v;
    param_get(_params_handles.pitch_d, &v);
    _params.att_d(1) = v;
    param_get(_params_handles.pitch_N, &v);
    _params.att_N(1) = v;
    param_get(_params_handles.pitch_rate_p, &v);
	if(v==0)
    	_params.rate_p(1)  = 0;
	else
		_params.rate_p(1)  = v;
	
    param_get(_params_handles.pitch_rate_i, &v);
    _params.rate_i(1)  = v;
    param_get(_params_handles.pitch_rate_d, &v);
    _params.rate_d(1)  = v;
    param_get(_params_handles.pitch_rate_N, &v);
    _params.rate_N(1)  = v;
	

    param_get(_params_handles.yaw_p, &v);
	if(v==0)
    	_params.att_p(2) = 0;
	else
		_params.att_p(2) = v;
    param_get(_params_handles.yaw_i, &v);
    _params.att_i(2) = v;
    param_get(_params_handles.yaw_d, &v);
    _params.att_d(2) = v;
    param_get(_params_handles.yaw_N, &v);
    _params.att_N(2) = v;
    param_get(_params_handles.yaw_rate_p, &v);
	if(v==0)
    	_params.rate_p(2) = 0;
	else
		_params.rate_p(2) = v;
    param_get(_params_handles.yaw_rate_i, &v);
    _params.rate_i(2) = v;
    param_get(_params_handles.yaw_rate_d, &v);
    _params.rate_d(2) = v;
    param_get(_params_handles.yaw_rate_N, &v);
    _params.rate_N(2) = v;

    /* velocity gains */
    param_get(_params_handles.vel_x_kp, &v);
	if(v==0)
    	_params.vel_p(0) = 0;
	else
    	_params.vel_p(0) = v;
    param_get(_params_handles.vel_y_kp, &v);
	if(v==0)
    	_params.vel_p(1) = 0;
	else
    	_params.vel_p(1) = v;
    param_get(_params_handles.vel_z_kp, &v);
	if(v==0)
    	_params.vel_p(2) = 0;
	else
    	_params.vel_p(2) = v;


    param_get(_params_handles.vel_x_ki, &v);
    _params.vel_i(0) = v;
    param_get(_params_handles.vel_y_ki, &v);
    _params.vel_i(1) = v;
    param_get(_params_handles.vel_z_ki, &v);
    _params.vel_i(2) = v;

    param_get(_params_handles.vel_x_kd, &v);
    _params.vel_d(0) = v;
    param_get(_params_handles.vel_y_kd, &v);
    _params.vel_d(1) = v;
    param_get(_params_handles.vel_z_kd, &v);
    _params.vel_d(2) = v;
    
    param_get(_params_handles.vel_x_N, &v);
    _params.vel_N(0) = v;
    param_get(_params_handles.vel_y_N, &v);
    _params.vel_N(1) = v;
    param_get(_params_handles.vel_z_N, &v);
    _params.vel_N(2) = v;
    
   
    /* position gains */
    param_get(_params_handles.pos_x_kp, &v);
    _params.pos_p(0) = v;
    param_get(_params_handles.pos_y_kp, &v);
    _params.pos_p(1) = v;
    param_get(_params_handles.pos_z_kp, &v);
    _params.pos_p(2) = v;


    /* model parameters */
    param_get(_params_handles.model_ixx, &v);
    _params.model_i(0) = v;
    param_get(_params_handles.model_iyy, &v);
    _params.model_i(1) = v;
    param_get(_params_handles.model_izz, &v);
    _params.model_i(2) = v;


    /* model max values */
    param_get(_params_handles.max_rate, &v);
    _params.max_val(0) = v * M_PI_F/180.0f;
    param_get(_params_handles.max_angle, &v);
    _params.max_val(1) = v * M_PI_F/180.0f;
    param_get(_params_handles.max_vel, &v);
    _params.max_val(2) = v;

    return OK;
}

void    
QcNdiControl::parameter_update_poll(){
    bool updated;
    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
        parameters_update();
        //usleep(10);
        parameters_save();
    }
}

void    
QcNdiControl::parameters_save(){
int ret = param_save_default(); // this is not save during flight!!
        if (ret == OK){
            warnx("Parameters Saved");
        } else warnx("Parameters NOT Saved");    
}
    
    
void    
QcNdiControl::rc_channels_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_rc_channels_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(rc_channels), _rc_channels_sub, &_rc_in);
    }
}

void   
QcNdiControl::attitude_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_vehicle_attitude_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
    }

}

void   
QcNdiControl::vehicle_local_position_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_vehicle_local_position_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_local_position);
    }

}

void   
QcNdiControl::control_state_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_control_state_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(control_state), _control_state_sub, &_control_state);
    }

}

void
QcNdiControl::task_main_trampoline(int argc, char *argv[])
{
	qc_ndi_control::g_control->task_main();
}

void 
QcNdiControl::task_main()
{

    quad_ndi_initialize();

    bool qc_ndi_arm = false;

    /* do subscriptions */
    _rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _control_state_sub = orb_subscribe(ORB_ID(control_state));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* initialize parameters cache */
    parameters_update();
	
    /* publications */
    _simulink_app_pwm_pub = orb_advertise(ORB_ID(simulink_app_pwm), &_pwm_out);
    _simulink_app_debug_pub  = orb_advertise(ORB_ID(simulink_app_debug), &_ndi_debug);
	_actuators_id = ORB_ID(actuator_controls_0);
    _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);

    /* wakeup source: vehicle attitude */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = _vehicle_attitude_sub;
    fds[0].events = POLLIN;

    while (!_task_should_exit) {
        /* wait for up to 100ms for data */
        int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            continue;
        }
        if (poll_ret < 0) {
            /* this is seriously brc_inad - should be an emergency */
            warn("qc_ndi_control: poll error %d, %d", poll_ret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;

        }

        if (fds[0].revents & POLLIN) {
            perf_begin(_loop_perf);
           
            /* copy attitude topic */
            orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);

            /* check for updates in other topics */
            parameter_update_poll();
            rc_channels_poll();
            vehicle_local_position_poll();
			
            quad_ndi_U.param[0] = _params.rate_p(0);
            quad_ndi_U.param[1] = _params.rate_p(1);
            quad_ndi_U.param[2] = _params.rate_p(2);
            quad_ndi_U.param[3] = _params.rate_i(0);
            quad_ndi_U.param[4] = _params.rate_i(1);
            quad_ndi_U.param[5] = _params.rate_i(2);
            quad_ndi_U.param[6] = _params.rate_d(0);
            quad_ndi_U.param[7] = _params.rate_d(1);
            quad_ndi_U.param[8] = _params.rate_d(2);
            quad_ndi_U.param[9] = _params.rate_N(0);
            quad_ndi_U.param[10] = _params.rate_N(1);
            quad_ndi_U.param[11] = _params.rate_N(2);
			
            quad_ndi_U.param[12] = _params.att_p(0);
            quad_ndi_U.param[13] = _params.att_p(1);
            quad_ndi_U.param[14] = _params.att_p(2);
            quad_ndi_U.param[15] = _params.att_i(0);
            quad_ndi_U.param[16] = _params.att_i(1);
            quad_ndi_U.param[17] = _params.att_i(2);
            quad_ndi_U.param[18] = _params.att_d(0);
            quad_ndi_U.param[19] = _params.att_d(1);
            quad_ndi_U.param[20] = _params.att_d(2);
            quad_ndi_U.param[21] = _params.att_N(0);
            quad_ndi_U.param[22] = _params.att_N(1);
            quad_ndi_U.param[23] = _params.att_N(2);
		
            quad_ndi_U.param[24] = _params.vel_p(0);
            quad_ndi_U.param[25] = _params.vel_p(1);
            quad_ndi_U.param[26] = _params.vel_p(2);
            quad_ndi_U.param[27] = _params.vel_i(0);
            quad_ndi_U.param[28] = _params.vel_i(1);
            quad_ndi_U.param[29] = _params.vel_i(2);
            quad_ndi_U.param[30] = _params.vel_d(0);
            quad_ndi_U.param[31] = _params.vel_d(1);
            quad_ndi_U.param[32] = _params.vel_d(2);
            quad_ndi_U.param[33] = _params.vel_N(0);
            quad_ndi_U.param[34] = _params.vel_N(1);
            quad_ndi_U.param[35] = _params.vel_N(2);
            quad_ndi_U.param[36] = _params.pos_p(0);
            quad_ndi_U.param[37] = _params.pos_p(1);
            quad_ndi_U.param[38] = _params.pos_p(2);
            quad_ndi_U.param[39] = _params.max_val(0);
            quad_ndi_U.param[40] = _params.max_val(1);
            quad_ndi_U.param[41] = _params.max_val(2);
            quad_ndi_U.param[42] = _params.model_i(0);
            quad_ndi_U.param[43] = _params.model_i(1);
            quad_ndi_U.param[44] = _params.model_i(2);
			
            
            math::Matrix<3,3> R = _attitude.R;
            math::Vector<3> v_ned(_local_position.vx,_local_position.vy,_local_position.vz);
            math::Vector<3> v_body = R.transposed() * v_ned;

            quad_ndi_U.state[0] = v_body(0);                      // Body vx
            quad_ndi_U.state[1] = v_body(1);                      // Body vy
            quad_ndi_U.state[2] = v_body(2);              
            quad_ndi_U.state[3] = _local_position.x;                // NED x
            quad_ndi_U.state[4] = _local_position.y;                // NED y
            quad_ndi_U.state[5] = _local_position.z;                // NED z
            quad_ndi_U.state[6] = _attitude.rollspeed;              // Body p
            quad_ndi_U.state[7] = _attitude.pitchspeed;             // Body q
            quad_ndi_U.state[8] = _attitude.yawspeed;               // Body r
            quad_ndi_U.state[9] = _attitude.roll;                   // Roll
            quad_ndi_U.state[10] = _attitude.pitch;                 // Pitch
            quad_ndi_U.state[11] = _attitude.yaw;                   // Yaw
            quad_ndi_U.state[12] = _local_position.vx ;             // NED vx
            quad_ndi_U.state[13] = _local_position.vy;              // NED vy
            quad_ndi_U.state[14] = _local_position.vz ;             // NED vz

            quad_ndi_U.pwm_in[0] = _rc_in.channels[0];
            quad_ndi_U.pwm_in[1] = _rc_in.channels[1];
            quad_ndi_U.pwm_in[2] = _rc_in.channels[2];
            quad_ndi_U.pwm_in[3] = _rc_in.channels[3];
            quad_ndi_U.pwm_in[4] = _rc_in.channels[4];
            quad_ndi_U.pwm_in[5] = _rc_in.channels[5];
            quad_ndi_U.pwm_in[6] = _rc_in.channels[6];
            quad_ndi_U.pwm_in[7] = _rc_in.channels[7];

			
            /*Check arming status*/
			
            if(_rc_in.channels[4] > 0.5f && !_rc_in.signal_lost)
                qc_ndi_arm = true;
            else
                qc_ndi_arm = false;

            /* Run code from codegeneration */
             quad_ndi_step();

            /* publish actuator controls */

            if(qc_ndi_arm == true){
                _pwm_out.arm = true;
                _pwm_out.timestamp = hrt_absolute_time();
                _pwm_out.pwm[0] = quad_ndi_Y.pwm[0];
                _pwm_out.pwm[1] = quad_ndi_Y.pwm[1];
                _pwm_out.pwm[2] = quad_ndi_Y.pwm[2];
                _pwm_out.pwm[3] = quad_ndi_Y.pwm[3];

            } else {
                _pwm_out.arm = false;
                _pwm_out.timestamp = hrt_absolute_time();
                _pwm_out.pwm[0] = 900;
                _pwm_out.pwm[1] = 900;
                _pwm_out.pwm[2] = 900;
                _pwm_out.pwm[3] = 900;
               
            }
            orb_publish(ORB_ID(simulink_app_pwm), _simulink_app_pwm_pub, &_pwm_out);

            /* publish debug controls */
            _ndi_debug.timestamp = hrt_absolute_time();
            _ndi_debug.debug[0] = quad_ndi_Y.debug[0];
            _ndi_debug.debug[1] = quad_ndi_Y.debug[1];
            _ndi_debug.debug[2] = quad_ndi_Y.debug[2];
            _ndi_debug.debug[3] = quad_ndi_Y.debug[3];
            
            orb_publish(ORB_ID(simulink_app_debug), _simulink_app_debug_pub, &_ndi_debug);

            _actuators.timestamp = hrt_absolute_time();
            _actuators.control[3] = 1;
            orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
            perf_end(_loop_perf);
			
            
            /*
            PX4_WARN("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",(double)tpa \
								  ,(double)e_R(1)  \
								  ,(double)e_R(2)  \
								  ,(double)R_z(0) \
                                  ,(double)R_z(1) \
                                  ,(double)R_z(2));
            
            PX4_WARN("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",(double)quad_ndi_U.pwm_in[0] \
								  ,(double)quad_ndi_U.pwm_in[1] \
								  ,(double)quad_ndi_U.pwm_in[2] \
								  ,(double)quad_ndi_U.pwm_in[3] \
								  ,(double)quad_ndi_U.state[10] \
								  ,(double)quad_ndi_U.state[11]);
			
            PX4_WARN("%0.2f %0.2f %0.2f",(double)v_body(0) \
                                        ,(double)v_body(1) \
                                        ,(double)v_body(2));
			*/
            }

    }


	_control_task = -1;
    return;
}



int
QcNdiControl::start()
{
	ASSERT(_control_task == -1);

	_control_task = px4_task_spawn_cmd("qc_ndi_control",
			           SCHED_DEFAULT,
			           SCHED_PRIORITY_MAX - 5,
			           7000,
					   (px4_main_t)&QcNdiControl::task_main_trampoline,
					   nullptr);
					   
	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}



/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
        usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }
    
    errx(1, "usage: qc_ndi_control {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The qc_ndi_control_main only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int qc_ndi_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
	return 1;
    }
    
    if (!strcmp(argv[1], "start")) {
        
		if (qc_ndi_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}
		
		qc_ndi_control::g_control = new QcNdiControl;
        
		if (qc_ndi_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != qc_ndi_control::g_control->start()) {
			delete qc_ndi_control::g_control;
			qc_ndi_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	   
    }
	if (!strcmp(argv[1], "stop")) {
		if (qc_ndi_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete qc_ndi_control::g_control;
		qc_ndi_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (qc_ndi_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

