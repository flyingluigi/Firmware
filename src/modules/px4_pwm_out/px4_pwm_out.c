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
 * @file px4_daemon_app.c
 * application example for PX4 autopilot
 *
 * @author Mocher Marcell <mail@example.com>
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


#include <sys/types.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int px4_pwm_out_task;				/**< Handle of daemon task / thread */

perf_counter_t _pwm_out_perf;
const char *g_pwm_device = PWM_OUTPUT0_DEVICE_PATH;
int g_pwm_fd = -1;
bool g_pwm_enabled = false;


/**
 * daemon management function.
 */
__EXPORT int px4_pwm_out_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_pwm_out_thread_main(int argc, char *argv[]);

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
 * The daemon app only briefly exists to start
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
        
        if (thread_running) {
            warnx("px4_pwm_out already running\n");
            /* this is not an error */
            return 0;
        }
        
        thread_should_exit = false;

        px4_pwm_out_task = px4_task_spawn_cmd("px4_pwm_out",
                                                         SCHED_DEFAULT,
                                                         SCHED_PRIORITY_MAX - 5,
                                                         2000,
                                                         px4_pwm_out_thread_main,
                                                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;

    }
    
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }
    
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");
            
        } else {
            warnx("\tnot started\n");
        }
        
        return 0;
    }
    
    usage("unrecognized command");
    return 1;
}

int px4_pwm_out_thread_main(int argc, char *argv[])
{
    
    int _pwm_sub_fd = orb_subscribe(ORB_ID(simulink_app_pwm));
    orb_set_interval(_pwm_sub_fd, 4);
    

    /* wait for topics */ 
    struct pollfd fds[1];
    fds[0].fd = _pwm_sub_fd;
    fds[0].events = POLLIN;
    
    
    warnx("Starting PWM output\n");
    thread_running = true;
    
    
    int rc;
    
    /* enable pwm outputs, set to disarm  */
    g_pwm_fd = px4_open(g_pwm_device, 0);
    printf("OPEN PWM fd = %d\n", g_pwm_fd);
    
    /* tell safety that its ok to disable it with the switch */
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_SET_ARM_OK, 0);
    if (rc != OK)
        err(1, "PWM_SERVO_SET_ARM_OK");
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, 400);
    if (rc != OK)
        err(1, "PWM_SERVO_SET_UPDATE_RATE");
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
    if (rc != OK)
        err(1, "PWM_SERVO_DISARM");
    g_pwm_enabled = false;
    
    int error_counter = 0;
    
    while (!thread_should_exit) {
        
        int poll_ret = poll(fds, 1, 4);
        
        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            
        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                printf("ERROR return value from poll(): %d\n"
                        , poll_ret);
            }
            
            error_counter++;
            
        } else {
            
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct simulink_app_pwm_s pwm_val;
                /* copy data into local buffer */
                orb_copy(ORB_ID(simulink_app_pwm), _pwm_sub_fd, &pwm_val);
                
                if (pwm_val.arm == true) {
                    if (g_pwm_enabled == false) {
                        
                        /* arm system */
                        rc = px4_ioctl(g_pwm_fd, PWM_SERVO_ARM, 0);
                        if (rc != OK)
                            err(1,"PWM_SERVO_ARM");
                        else {
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(4), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(5), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(6), 1000);
                            px4_ioctl(g_pwm_fd, PWM_SERVO_SET(7), 1000);
                            g_pwm_enabled = true;
                            printf("***ARMED*** PWM fd = %d\n", g_pwm_fd);
                        }
                    }
                } else {
                    if (g_pwm_enabled == true) {
                        
                        /* disarm system if enabled */
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(4), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(5), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(6), 1000);
                        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(7), 1000);
                        rc = px4_ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
                        g_pwm_enabled = false;
                        if (rc != OK)
                            err(1, "PWM_SERVO_DISARM");
                        else
                            printf("***DISARMED*** PWM fd = %d\n", g_pwm_fd);
                    }
                }
                
                if (g_pwm_enabled) {
                    /* output the PWM signals */
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), pwm_val.pwm[0]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), pwm_val.pwm[1]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), pwm_val.pwm[2]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), pwm_val.pwm[3]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(4), pwm_val.pwm[4]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(5), pwm_val.pwm[5]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(6), pwm_val.pwm[6]);
                    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(7), pwm_val.pwm[7]);
                }
                
            }
            
        }
    }
    
    warnx("[daemon] exiting.\n");
    
    thread_running = false;
    
    return 0;
}
