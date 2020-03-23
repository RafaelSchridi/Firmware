//
// Created by sages on 23-3-20.
//

#include <math.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>

extern "C" __EXPORT int waypointdetect_main(int argc, char *argv[]);

class WayPointDetect: public ModuleBase<WayPointDetect>{
public:
    WayPointDetect();
    ~WayPointDetect();

    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static WayPointDetect *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;


private:


    bool glob_position_updated = false;


    int error_counter = 0;
};

WayPointDetect::WayPointDetect()
{

}

WayPointDetect::~WayPointDetect() {

}

void WayPointDetect::run() {
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int sensor_sub_fd = orb_subscribe(ORB_ID(position_setpoint_triplet));
    int glob_position = orb_subscribe(ORB_ID(vehicle_global_position));


    px4_pollfd_struct_t fds[] = {
            { .fd = att_sub,   .events = POLLIN },
    };

    struct position_setpoint_triplet_s raw;
    struct vehicle_global_position_s raw2;

    while(!should_exit()){
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
                    orb_copy(ORB_ID(vehicle_global_position), glob_position, &raw2);
                }

                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(position_setpoint_triplet), sensor_sub_fd, &raw);

                double diff_lat = raw.current.lat - raw2.lat;
                double diff_lon = raw.current.lon - raw2.lon;

                if ((diff_lat <= 0.000100) & (diff_lon <= 0.000100) & (diff_lat >= -0.000100) & (diff_lon >= -0.000100) ){
                    PX4_INFO("ALMOST THERE! \t%f\t%f",raw.current.lat - raw2.lat,raw.current.lon - raw2.lon);
                }

            }


        }
        if(error_counter > 50){
            break;
        }
    }

    orb_unsubscribe(att_sub);
    orb_unsubscribe(sensor_sub_fd);
    orb_unsubscribe(glob_position);
}


int WayPointDetect::task_spawn(int argc, char **argv) {
    /* start the task */
    _task_id = px4_task_spawn_cmd("waypointdetect",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

int WayPointDetect::print_status() {
    PX4_INFO("all seems to be going well");
    return 0;
}

int WayPointDetect::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int WayPointDetect::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels
  to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and
  `manual_control_setpoint`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sensors", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

WayPointDetect *WayPointDetect::instantiate(int argc, char *argv[])
{
    //int example_param = 0;
    //bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'p':
                //example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                //example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    WayPointDetect *instance = new WayPointDetect();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int waypointdetect_main(int argc, char *argv[])
{
    return WayPointDetect::main(argc, argv);
}
