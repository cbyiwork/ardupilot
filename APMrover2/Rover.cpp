/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Randy Mackay, Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See http://dev.ardupilot.org for details
*/

#include "Rover.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include "AP_Gripper/AP_Gripper.h"
#include "../libraries/AP_InertialSensor/AP_InertialSensor_NRF24L01.h"


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(Rover, &rover, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio,             50,    200),

	// get gps data, and update location info
    SCHED_TASK(ahrs_update,           400,    400),

	// 测距�?
    SCHED_TASK(read_rangefinders,      50,    200),

	// 执行Mode各个子类相应的update方法，驱动电�?
    SCHED_TASK(update_current_mode,   400,    200),

	// 设置各路pwm, Steering.cpp 中定�?
    SCHED_TASK(set_servos,            400,    200),

	// 更新 gps 数据 
    SCHED_TASK(update_GPS,             50,    300),

	// 气压�? 有用到经纬度信息
    // SCHED_TASK_CLASS(AP_Baro,             &rover.barometer,        update,         10,  200),

	// 围栏�?测，边界�?�?
    SCHED_TASK_CLASS(AP_Beacon,           &rover.g2.beacon,        update,         50,  200),

	// 靠近�?测？
    SCHED_TASK_CLASS(AP_Proximity,        &rover.g2.proximity,     update,         50,  200),
    // 风�?�检测？
    SCHED_TASK_CLASS(AP_WindVane,         &rover.g2.windvane,      update,         20,  100),
    // 里程�?
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_VisualOdom,       &rover.g2.visual_odom,   update,         50,  200),
#endif
    SCHED_TASK_CLASS(AC_Fence,            &rover.g2.fence,         update,         10,  100),

	// sensors.cpp 中定义， 
    SCHED_TASK(update_wheel_encoder,   50,    200),

	// compass 数据更新，用以调整姿�?
    SCHED_TASK(update_compass,         10,    200),

	// Rover.cpp 中定义， 更新命令，启停控�?
    SCHED_TASK(update_mission,         50,    200),

	// add by ycb, for nrf24l01 command
	SCHED_TASK(update_rc_nrf24g,     50,     200),

	// 写入相关信息，包括经纬度，接近�?�等
    SCHED_TASK(update_logging1,        10,    200),

	// streering�? vibrate等log信息
    SCHED_TASK(update_logging2,        10,    200),

	// GCS_MAVLink 相关，在GCS_Common.cpp中定义�?�GCS_MAVLINK::update_receive
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_receive,                    400,    500),

	// GCS_MAVLINK::update_send
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_send,                       400,   1000),

	// RC_Channel.cpp中定义，似乎是读取ppm相关数据，做判断 RC_Channel::read_mode_switch
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_mode_switch,        7,    200),

	// RC_Channel::read_aux_all 
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_aux_all,           10,    200),
    SCHED_TASK_CLASS(AP_BattMonitor,      &rover.battery,          read,           10,  300),

	// GCS_MAVLINK command relay
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents, update_events,  50,  200),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &rover.g2.gripper,      update,         10,   75),
#endif
    SCHED_TASK(rpm_update,             10,    100),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &rover.camera_mount,     update,         50,  200),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &rover.camera,           update_trigger, 50,  200),
#endif
    SCHED_TASK(gcs_failsafe_check,     10,    200),
    SCHED_TASK(fence_check,            10,    200),
    SCHED_TASK(ekf_check,              10,    100),

	// mode_smart_rtl.cpp 定义
    SCHED_TASK_CLASS(ModeSmartRTL,        &rover.mode_smartrtl,    save_position,   3,  200),
    SCHED_TASK_CLASS(AP_Notify,           &rover.notify,           update,         50,  300),

	// 驱动更新
    SCHED_TASK(one_second_loop,         1,   1500),
    SCHED_TASK_CLASS(AC_Sprayer,          &rover.g2.sprayer,           update,      3,  90),

	// �?螺仪更新，AP_Compass_Calibration.cpp中定�?
    SCHED_TASK_CLASS(Compass,          &rover.compass,              cal_update, 50, 200),
    SCHED_TASK(compass_save,           0.1,   200),

	// 加�?�检�?
    SCHED_TASK(accel_cal_update,       10,    200),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger,     &rover.logger,        periodic_tasks, 50,  300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &rover.ins,              periodic,      400,  200),

	// AP_Scheduler.cpp AP_Scheduler::update_logging(), write some logger
    SCHED_TASK_CLASS(AP_Scheduler,        &rover.scheduler,        update_logging, 0.1, 200),

	// 按键事件
    SCHED_TASK_CLASS(AP_Button,           &rover.button,           update,          5,  200),
#if STATS_ENABLED == ENABLED
	// update_flighttime
    SCHED_TASK(stats_update,            1,    200),
#endif
    SCHED_TASK(crash_check,            10,    200),
    SCHED_TASK(cruise_learn_update,    50,    200),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    200),
#endif
    SCHED_TASK(read_airspeed,          10,    100),
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info,        1,     10),
#endif
};

constexpr int8_t Rover::_failsafe_priorities[7];

Rover::Rover(void) :
    AP_Vehicle(),
    param_loader(var_info),
    channel_steer(nullptr),
    channel_throttle(nullptr),
    channel_lateral(nullptr),
    logger{g.log_bitmask},
    modes(&g.mode1),
    control_mode(&mode_initializing),
    G_Dt(0.02f)
{
}

#if STATS_ENABLED == ENABLED
/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
	//hal.uartB->printf("%s enter\n", __func__);
    g2.stats.set_flying(g2.motors.active());
    g2.stats.update();
}
#endif

/*
  setup is called when the sketch starts
 */
void Rover::setup()
{
	//hal.uartE->printf("%s enter\n", __func__);
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);

	// add by cbyi
	//set_mode(mode_nrf24g, MODE_REASON_MISSION_COMMAND);

	//hal.uartB->flush();

	//hal.uartB->printf("%s exit\n", __func__);

}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}

// update AHRS system
void Rover::ahrs_update()
{
	//hal.uartB->printf("%s enter\n", __func__);
	
    arming.update_soft_armed();

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs().update();
#endif

    // AHRS may use movement to calculate heading
    update_ahrs_flyforward();

    ahrs.update();

    // update position
    have_position = ahrs.get_position(current_loc);

    // set home from EKF if necessary and possible
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = norm(velocity.x, velocity.y);
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
	//hal.uartB->printf("%s enter\n", __func__);
	
    if (!g.fs_gcs_enabled) {
        // gcs failsafe disabled
        return;
    }

    // check for updates from GCS within 2 seconds
    failsafe_trigger(FAILSAFE_EVENT_GCS, failsafe.last_heartbeat_ms != 0 && (millis() - failsafe.last_heartbeat_ms) > 2000);
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
	//hal.uartE->printf("%s enter\n", __func__);
	
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_THR)) {
        Log_Write_Throttle();
        logger.Write_Beacon(g2.beacon);
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
    }

    if (should_log(MASK_LOG_RANGEFINDER)) {
        logger.Write_Proximity(g2.proximity);
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
	//hal.uartE->printf("%s enter\n", __func__);
	
    if (should_log(MASK_LOG_STEERING)) {
        Log_Write_Steering();
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
        g2.wheel_encoder.Log_Write();
    }

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_Vibration();
    }
}


/*
  once a second events
 */
void Rover::one_second_loop(void)
{
	//hal.uartE->printf("%s enter\n", __func__);
    // allow orientation change at runtime to aid config
    ahrs.update_orientation();

    set_control_channels();

    // cope with changes to aux functions
    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;
    AP_Notify::flags.flying = hal.util->get_soft_armed();

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // attempt to update home position and baro calibration if not armed:
    if (!hal.util->get_soft_armed()) {
        update_home();
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());

    // send latest param values to wp_nav
    g2.wp_nav.set_turn_params(g.turn_max_g, g2.turn_radius, g2.motors.have_skid_steering());
}

void Rover::update_GPS(void)
{
	//hal.uartB->printf("%s enter\n", __func__);
    gps.update();
    if (gps.last_message_time_ms() != last_gps_msg_ms) {
        last_gps_msg_ms = gps.last_message_time_ms();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Rover::update_current_mode(void)
{
	//hal.uartB->printf("%s enter\n", __func__);
    // check for emergency stop
    if (SRV_Channels::get_emergency_stop()) {
        // relax controllers, motor stopping done at output level
        g2.attitude_control.relax_I();
    }

	// Mode 的各个子类实现了update方法，更�? steering �? throttle
    control_mode->update();
}

// update mission including starting or stopping commands. called by scheduler at 10Hz
void Rover::update_mission(void)
{
	//hal.uartB->printf("%s enter\n", __func__);
	//hal.uartE->printf("%s enter, mode: %s\n", __func__, control_mode->name4());
    if (control_mode == &mode_auto) {
        if (ahrs.home_is_set() && mode_auto.mission.num_commands() > 1) {
            mode_auto.mission.update();
        }
    }
}

void Rover::update_rc_nrf24g(void) 
{

	/*static int8_t cnt = 0;
	if (cnt ++ > 30) {
		cnt = 0;
		hal.uartE->printf("%s\n", __func__);
	}*/
	
	OperaType mode;
	AP_InertialSensor_NRF24L01::getInstance()->get_nrf24l01_mode(mode);

	if (mode == OPERA_M) {
		if (control_mode != &mode_nrf24g) {
			//hal.uartE->printf("change mode to nrf24g\n");
			if (!hal.util->get_soft_armed()) {
		        hal.util->set_soft_armed(true);
		    }
			set_mode(mode_nrf24g, MODE_REASON_MISSION_COMMAND);
		}
	} else if (mode == OPERA_H) {
		if (control_mode != &mode_rtl) {
			//hal.uartE->printf("change mode to rtl\n");
			if (!hal.util->get_soft_armed()) {
		        hal.util->set_soft_armed(true);
		    }
			set_mode(mode_rtl, MODE_REASON_MISSION_COMMAND);
		}
		 
	} else if (mode == OPERA_L) {
		if (control_mode != &mode_hold) {
			//hal.uartE->printf("change mode to HOLD\n");
			set_mode(mode_hold, MODE_REASON_MISSION_COMMAND);
		}
		
	}

	
}

#if OSD_ENABLED == ENABLED
void Rover::publish_osd_info()
{
    AP_OSD::NavInfo nav_info {0};

	//hal.uartB->printf("%s enter\n", __func__);
	
    if (control_mode == &mode_loiter) {
        nav_info.wp_xtrack_error = control_mode->get_distance_to_destination();
    } else {
        nav_info.wp_xtrack_error = control_mode->crosstrack_error();
    }
    nav_info.wp_distance = control_mode->get_distance_to_destination();
    nav_info.wp_bearing = control_mode->wp_bearing() * 100.0f;
    if (control_mode == &mode_auto) {
         nav_info.wp_number = mode_auto.mission.get_current_nav_index();
    }
    osd.set_nav_info(nav_info);
}
#endif

Rover rover;

AP_HAL_MAIN_CALLBACKS(&rover);
