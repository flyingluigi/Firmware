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
 * @file simulink_example.cpp
 * Octocopter control of all states using ndi control law
 *
 * @author Marcell Mocher <marcell.mocher@fh-joanneum.at>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>



#ifdef __cplusplus
extern "C" {
#endif
#include "codegen/test.c"
#include "codegen/rtwtypes.h"
#ifdef __cplusplus
}
#endif


static bool thread_should_exit = false;		/**< simulink_example exit flag */
static bool thread_running = false;		/**< simulink_example status flag */
static int simulink_example_task;			/**< simulink_example of daemon task / thread */


extern "C" __EXPORT int simulink_example_main(int argc, char *argv[]);

/**
 * Mainloop of simulink_example.
 */
int simulink_example_thread(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
                fprintf(stderr, "%s\n", reason);
	}

        fprintf(stderr, "usage: simulink_example {start|stop|status} [-p <additional params>]\n\n");
}



/**
 * The simulink_example app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int simulink_example_main(int argc, char *argv[])
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
                simulink_example_task = px4_task_spawn_cmd("simulink_example",
					     SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             7000,
                                             simulink_example_thread,
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





int simulink_example_thread(int argc, char *argv[])
{



    test_initialize();
	
    warnx("[daemon] starting\n");

	
    thread_running = true;

    while (!thread_should_exit) {
        test_U.In1 = 1.0f;
        test_step();
        printf("%f",test_Y.Out1);
        sleep(10);
        }


    warnx("[simulink_example] exiting.\n");

    thread_running = false;

    return 0;
}
