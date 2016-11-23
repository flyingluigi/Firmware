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
 * @file px4_pwm_out.cpp
 * Application to write directly to ESCs
 *
 * @author Mocher Marcell <marcell.mocher@fh-joanneum.at>
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>


#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include "systemlib/perf_counter.h"
#include "uORB/topics/simulink_app_pwm.h"
#include "uORB/topics/vehicle_land_detected.h"
#include "uORB/topics/vehicle_control_mode.h"

#include <sys/types.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>

#include "DevMgr.hpp"

using namespace DriverFramework;


/**
 * px4_pwm_out management function.
 */
extern "C" __EXPORT int px4_pwm_out_main(int argc, char *argv[]);

class Px4PwmOut
{
public:
	/**
 	* Constructor
 	*/
	Px4PwmOut();
	
	/**
 	* Destructor, also kills the main task
 	*/	
	~Px4PwmOut();
	
	
	/**
 	* Start the px4 pwm out task.
 	*
 	* @return		OK on success.
 	*/
	int start();
	
private:

	bool _task_should_exit;	/**< if true, task_main() should exit */
	int _control_task;		/**< task handle */
	int _g_pwm_fd;
	int _g_pwm_aux_fd;

	int _simulink_app_pwm_sub;	/**< pwm bus subscription */
	//int _vehicle_land_detected_sub;
	
	orb_advert_t _vehicle_control_mode_pub;

	char const *g_pwm_device = PWM_OUTPUT0_DEVICE_PATH;
	char const *g_pwm_aux = "/dev/pwm_output1";
	
	struct simulink_app_pwm_s	_simulink_app_pwm;		/**< simulink_app_pwm */
	//struct vehicle_land_detected_s _vehicle_land_detected;
	struct vehicle_control_mode_s _vehicle_control_mode;
	
	/**
 	* Shim for calling task_main from task_create.
 	*/
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main px4 pwm out task.
	 */
	void		task_main();


};

namespace px4_pwm_out
{

Px4PwmOut	*g_control;

}

Px4PwmOut::Px4PwmOut() :

	_task_should_exit(false),
	_control_task(-1),
	_g_pwm_fd(-1),
	_g_pwm_aux_fd(-1),
	/* subscriptions */
	_simulink_app_pwm_sub(-1),
	
	_vehicle_control_mode_pub(nullptr)
	//_vehicle_land_detected_sub(-1)
{
	memset(&_simulink_app_pwm, 0, sizeof(_simulink_app_pwm));
	//memset(&_vehicle_land_detected, 0, sizeof(_vehicle_land_detected));
	memset(&_vehicle_control_mode, 0, sizeof(_vehicle_control_mode));	
}



Px4PwmOut::~Px4PwmOut()
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
	px4_pwm_out::g_control = nullptr;
	
}


void
Px4PwmOut::task_main_trampoline(int argc, char *argv[])
{
	px4_pwm_out::g_control->task_main();
}



void
Px4PwmOut::task_main()
{
	/*
 	* do subscriptions
 	*/
    _simulink_app_pwm_sub = orb_subscribe(ORB_ID(simulink_app_pwm));
    
	
	_vehicle_control_mode_pub  = orb_advertise(ORB_ID(vehicle_control_mode), &_vehicle_control_mode);
	
	
	
    /* wakeup source: simulink_app_pwm */
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _simulink_app_pwm_sub;
    fds[0].events = POLLIN;
    
    int rc;
    
	DevHandle h_rgbleds;
    // declare output device
    DevMgr::getHandle("/dev/rgbled0",h_rgbleds);
	 
    if (!h_rgbleds.isValid()) {
            PX4_WARN("No RGB LED found at " RGBLED0_DEVICE_PATH);
    }

    /* set leds to disarmed*/
    h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
    h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
	
    /* enable pwm outputs, set to arm */
    _g_pwm_fd = px4_open(g_pwm_device, 0);
	//_g_pwm_aux_fd = px4_open(g_pwm_aux, 0);
	
	
	/* init control channels */
		
    /* tell safety that its ok to disable it with the switch */
    rc = px4_ioctl(_g_pwm_fd, PWM_SERVO_SET_ARM_OK, 0);
    if (rc != OK)
        err(1, "PWM_SERVO_SET_ARM_OK");
    rc = px4_ioctl(_g_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, 400);
    if (rc != OK)
        err(1, "PWM_SERVO_SET_UPDATE_RATE");
    rc = px4_ioctl(_g_pwm_fd, PWM_SERVO_ARM, 0);
    if (rc != OK)
        err(1, "PWM_SERVO_ARM");
    else{
		_vehicle_control_mode.timestamp = hrt_absolute_time();
		_vehicle_control_mode.flag_armed = false;
		_vehicle_control_mode.flag_control_manual_enabled = true;
		orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(0), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(1), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(2), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(3), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(4), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(5), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(6), 1000);
        px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(7), 1000);
        printf("***ARMED*** PWM fd = %d\n", _g_pwm_fd);
    }

	/* init aux channels
	
    rc = px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET_ARM_OK, 0);
    if (rc != OK)
		err(1, "AUX_PWM_SERVO_SET_ARM_OK");
	rc = px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET_UPDATE_RATE, 50);
    if (rc != OK)
        err(1, "AUX_PWM_SERVO_SET_UPDATE_RATE");
    rc = px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_ARM, 0);
    if (rc != OK)
        err(1, "AUX_PWM_SERVO_ARM");
    else{
		for(int i = 0 ; i < 6 ; i++)
		{
        px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET(0), 1000);
		usleep(1000000);
		px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET(0), 2000);
		usleep(1000000);
		}
		
	}
		
	*/
	
	
    int error_counter = 0;
	
    while (!_task_should_exit) {
        
		/*bool updated;
		orb_check(_vehicle_land_detected_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
			h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BLINK_FAST);
			h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_RED);
		}
		
		*/
		
        int poll_ret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            continue;
		}
		
        if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                printf("ERROR return value from poll(): %d\n"
                        , poll_ret);
            }
            
            error_counter++;
            
        } 
		if (fds[0].revents & POLLIN) {
               
                /* copy data into local buffer */
                orb_copy(ORB_ID(simulink_app_pwm), _simulink_app_pwm_sub, &_simulink_app_pwm);
                
                if (_simulink_app_pwm.arm == true) {
					//px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET(0), 2000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(0), _simulink_app_pwm.pwm[0]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(1), _simulink_app_pwm.pwm[1]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(2), _simulink_app_pwm.pwm[2]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(3), _simulink_app_pwm.pwm[3]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(4), _simulink_app_pwm.pwm[4]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(5), _simulink_app_pwm.pwm[5]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(6), _simulink_app_pwm.pwm[6]);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(7), _simulink_app_pwm.pwm[7]);
					h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BLINK_FAST);
					h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_RED);
					_vehicle_control_mode.timestamp = hrt_absolute_time();
					_vehicle_control_mode.flag_armed = true;
					_vehicle_control_mode.flag_control_manual_enabled = true;
					orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
                } else {
					//px4_ioctl(_g_pwm_aux_fd, PWM_SERVO_SET(0), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(0), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(1), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(2), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(3), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(4), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(5), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(6), 1000);
                    px4_ioctl(_g_pwm_fd, PWM_SERVO_SET(7), 1000);
					h_rgbleds.ioctl(RGBLED_SET_MODE, RGBLED_MODE_BREATHE);
					h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_GREEN);
					_vehicle_control_mode.timestamp = hrt_absolute_time();
					_vehicle_control_mode.flag_armed = false;
					_vehicle_control_mode.flag_control_manual_enabled = true;
					orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
                }
            }
            
        
    }
    
    _control_task = -1;
    
    return ;
}



int
Px4PwmOut::start()
{
	ASSERT(_control_task == -1);

	_control_task = px4_task_spawn_cmd("px4_pwm_out",
			           SCHED_DEFAULT,
			           SCHED_PRIORITY_MAX - 5,
			           2000,
					   (px4_main_t)&Px4PwmOut::task_main_trampoline,
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
    
    errx(1, "usage: px4_pwm_out {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The px4_pwm_out app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_pwm_out_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
	return 1;
    }
    
    if (!strcmp(argv[1], "start")) {
        
		if (px4_pwm_out::g_control != nullptr) {
			warnx("already running");
			return 1;
		}
		
		px4_pwm_out::g_control = new Px4PwmOut;
        
		if (px4_pwm_out::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != px4_pwm_out::g_control->start()) {
			delete px4_pwm_out::g_control;
			px4_pwm_out::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	   
    }
	if (!strcmp(argv[1], "stop")) {
		if (px4_pwm_out::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete px4_pwm_out::g_control;
		px4_pwm_out::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (px4_pwm_out::g_control) {
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

