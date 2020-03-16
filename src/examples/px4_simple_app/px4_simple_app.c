/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
    struct position_setpoint_triplet_s raw;
    struct vehicle_global_position_s raw2;

    memset(&raw, 0, sizeof(raw));
    memset(&raw2, 0, sizeof(raw2));

    bool glob_position_updated = false;

    /* subscribe to sensor_combined topic */
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int sensor_sub_fd = orb_subscribe(ORB_ID(position_setpoint_triplet));
    int glob_position = orb_subscribe(ORB_ID(vehicle_global_position));

    int error_counter = 0;

    orb_set_interval(sensor_sub_fd, 200);
    orb_set_interval(glob_position, 200);

    /* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = att_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};


	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");
			error_counter++;

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */


				orb_check(glob_position,&glob_position_updated);

				if (glob_position_updated){
                    PX4_INFO("%i",glob_position_updated);
                    orb_copy(ORB_ID(vehicle_global_position), glob_position, &raw2);
                }

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(position_setpoint_triplet), sensor_sub_fd, &raw);

				double diff_lat = raw.current.lat - raw2.lat;
				double diff_lon = raw.current.lon - raw2.lon;

				if ((diff_lat <= 0.000100) & (diff_lon <= 0.000100) & (diff_lat >= -0.000100) & (diff_lon >= -0.000100) ){
				    PX4_INFO("ALMOST THERE! \t%f\t%f",raw.current.lat - raw2.lat,raw.current.lon - raw2.lon);
				}


				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/

			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
        if(error_counter > 50){
            break;
        }
	}



	PX4_INFO("exiting");

	return 0;
}
