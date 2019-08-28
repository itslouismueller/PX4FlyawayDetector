/**
 * @file flyaway_monitor.c
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
#include <uORB/topics/position_controller_status.h>

extern "C" __EXPORT int flyaway_monitor_main(int argc, char *argv[]);

int flyaway_monitor_main(int argc, char *argv[])
{
    double current_dist_m = 0.0;
    double current_time_us = 0.0;
    double current_time_s = 0.0;
    double prev_dist_m = 0.0;
    double prev_time_s = 0.0;
    int windowSize = 240;
    double buffer [windowSize] = { };
    double progress_rate_m_s = 0.0;
    double movingAvg = 0.0;
    double rollingTot = 0.0;
    int i = 0;
    double xtrack_err = 0.0;
    double nav_roll = 0.0;




    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(position_controller_status));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 200);

    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
    };


    int error_counter = 0;

    while(true) {

        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        }
        else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }


            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct position_controller_status_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(position_controller_status), sensor_sub_fd, &raw);


                nav_roll = (double)raw.nav_roll;
                //veh_roll = (double)raw.
                xtrack_err = (double)raw.xtrack_error;
                current_dist_m = (double)raw.wp_dist;
                current_time_us = (double)raw.timestamp;
                current_time_s = current_time_us / 1000000;

                progress_rate_m_s = (prev_dist_m - current_dist_m)/(prev_time_s - current_time_s);


                //avoiding spikes from new waypoint
                if (progress_rate_m_s > 100 || progress_rate_m_s < -100){
                    progress_rate_m_s = movingAvg;
                    buffer[i%windowSize] = movingAvg;
                }
                else{
                    buffer[i%windowSize] = progress_rate_m_s;
                }


                //clearing rollingTot
                rollingTot = 0.0;

                for(int j = 0; j < windowSize; j++){
                    rollingTot += buffer[j];
                }

                movingAvg = rollingTot/(double)windowSize;

                if (movingAvg < 0){
                    PX4_INFO("PROGRESS ---- WP Dist (m): %8.4f  MA Progress Rate (m/s): %8.4f  Inst Rate (m/s): %8.4f  Xtrack Err: %8.4f  Nav Roll: %8.4f", current_dist_m, movingAvg,  buffer[i%windowSize], xtrack_err, nav_roll);
                }
                else{
                    PX4_INFO("NO PROGRESS - WP Dist (m): %8.4f  MA Progress Rate (m/s): %8.4f  Inst Rate (m/s): %8.4f  Xtrack Err: %8.4f  Nav Roll: %8.4f", current_dist_m, movingAvg,  buffer[i%windowSize], xtrack_err, nav_roll);
                }

                prev_dist_m = current_dist_m;
                prev_time_s = current_time_s;
                i++;
            }
        }
    }

    return 0;
}
