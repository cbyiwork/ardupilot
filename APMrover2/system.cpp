/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
    We will determine later if we are actually on the ground and process a
    ground start in that case.

*****************************************************************************/

#include "Rover.h"
#include <AP_Common/AP_FWVersion.h>

static void mavlink_delay_cb_static()
{
    rover.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    rover.failsafe_check();
}

void Rover::init_ardupilot()
{
    // initialise console serial port
	//hal.uartB->printf("%s enter\n", __func__);
	
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM.
    //

    load_parameters();

	//while (1);
	
#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();

	//hal.uartB->printf("gcs().setup_console()\n");
    // setup first port early to allow BoardConfig to report errors
    gcs().setup_console();

	//hal.uartB->printf("hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5)\n");
    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);


	//hal.uartB->printf("BoardConfig.init()\n");
    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // init gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

	//hal.uartB->printf("g2.fence.init()\n");
    g2.fence.init();

    // initialise notify system
    notify.init();
    notify_mode(control_mode);

	//hal.uartB->printf("battery.init()\n");
    battery.init();

	//hal.uartB->printf("rpm_sensor.init()\n");
    // Initialise RPM sensor
    rpm_sensor.init();

    rssi.init();

	//hal.uartB->printf("g2.airspeed.init()\n");
    g2.airspeed.init();

	//hal.uartB->printf("g2.windvane.init()\n");
    g2.windvane.init(serial_manager);

    // init baro before we start the GCS, so that the CLI baro test works
    // hal.uartB->printf("barometer.init()\n");
    //  barometer.init();

    // setup telem slots with serial ports
    //hal.uartB->printf("gcs().setup_uarts()\n");
    gcs().setup_uarts();

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise compass
    //hal.uartB->printf("initialise compass\n");
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // initialise rangefinder
    //hal.uartB->printf("initialise rangefinder\n");
    rangefinder.init(ROTATION_NONE);

    // init proximity sensor
    //hal.uartB->printf("init proximity sensor\n");
    init_proximity();

    // init beacons used for non-gps position estimation
    //hal.uartB->printf("init beacons used for non-gps position estimation\n");
    init_beacon();

    // init visual odometry
    //hal.uartB->printf("init_visual_odom()\n");
    init_visual_odom();

    // and baro for EKF
    // hal.uartB->printf("and baro for EKF\n");
    // barometer.set_log_baro_bit(MASK_LOG_IMU);
    // barometer.calibrate();

    // Do GPS init
    //hal.uartB->printf("gps.set_log_gps_bit(MASK_LOG_GPS)\n");
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

	//hal.uartB->printf("set_control_channels()\n");
    set_control_channels();  // setup radio channels and outputs ranges

	//hal.uartB->printf("init_rc_in()\n");
    init_rc_in();            // sets up rc channels deadzone

	//hal.uartB->printf("g2.motors.init()\n");
    g2.motors.init();        // init motors including setting servo out channels ranges

	//hal.uartB->printf("SRV_Channels::enable_aux_servos()\n");
    SRV_Channels::enable_aux_servos();

    // init wheel encoders
    //hal.uartB->printf("g2.wheel_encoder.init()\n");
    g2.wheel_encoder.init();

    relay.init();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    //hal.uartB->printf("failsafe_check_static\n");
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // initialize SmartRTL
    //hal.uartB->printf("initialize SmartRTL\n");
    g2.smart_rtl.init();

    // initialise object avoidance
    g2.oa.init();

	//hal.uartB->printf("startup_ground()\n");
    startup_ground();

	//hal.uartB->printf("mode_from_mode_num()\n");
    Mode *initial_mode = mode_from_mode_num((enum Mode::Number)g.initial_mode.get());
    if (initial_mode == nullptr) {
        initial_mode = &mode_initializing;
    }

	//hal.uartB->printf("set_mode()\n");
    set_mode(*initial_mode, MODE_REASON_INITIALISED);

    // initialise rc channels
    //hal.uartB->printf("rc().init()\n");
    rc().init();

	//hal.uartB->printf("rover.g2.sailboat.init()\n");
    rover.g2.sailboat.init();

    // disable safety if requested
    //hal.uartB->printf("BoardConfig.init_safety()\n");
    BoardConfig.init_safety();

    // flag that initialisation has completed
    initialised = true;

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif

	//hal.uartB->printf("%s exit\n", __func__);
}

//*********************************************************************************
// This function does all the calibrations, etc. that we need during a ground start
//*********************************************************************************
void Rover::startup_ground(void)
{
	//hal.uartB->printf("%s exit\n", __func__);
    set_mode(mode_initializing, MODE_REASON_INITIALISED);

    gcs().send_text(MAV_SEVERITY_INFO, "<startup_ground> Ground start");

    #if(GROUND_START_DELAY > 0)
        gcs().send_text(MAV_SEVERITY_NOTICE, "<startup_ground> With delay");
        delay(GROUND_START_DELAY * 1000);
    #endif

    // IMU ground start
    //------------------------
    //

	//hal.uartB->printf("%s startup_INS_ground()\n", __func__);
	// 此处校正各个仪器
    startup_INS_ground();

	//hal.uartB->printf("initialise mission library\n");
    // initialise mission library
    mode_auto.mission.init();

    // initialise AP_Logger library
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&rover, &Rover::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

	//hal.uartB->printf("g2.scripting.init()\n");

#ifdef ENABLE_SCRIPTING
    if (!g2.scripting.init()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Scripting failed to start");
    }
#endif // ENABLE_SCRIPTING

    // we don't want writes to the serial port to cause us to pause
    // so set serial ports non-blocking once we are ready to drive
    //hal.uartB->printf("set_blocking_writes_all()\n");
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO, "Ready to drive");
}

// update the ahrs flyforward setting which can allow
// the vehicle's movements to be used to estimate heading
void Rover::update_ahrs_flyforward()
{
    bool flyforward = false;

    // boats never use movement to estimate heading
    if (!is_boat()) {
        // throttle threshold is 15% or 1/2 cruise throttle
        bool throttle_over_thresh = g2.motors.get_throttle() > MIN(g.throttle_cruise * 0.50f, 15.0f);
        // desired speed threshold of 1m/s
        bool desired_speed_over_thresh = g2.attitude_control.speed_control_active() && (g2.attitude_control.get_desired_speed() > 0.5f);
        if (throttle_over_thresh || (is_positive(g2.motors.get_throttle()) && desired_speed_over_thresh)) {
            uint32_t now = AP_HAL::millis();
            // if throttle over threshold start timer
            if (flyforward_start_ms == 0) {
                flyforward_start_ms = now;
            }
            // if throttle over threshold for 2 seconds set flyforward to true
            flyforward = (now - flyforward_start_ms > 2000);
        } else {
            // reset timer
            flyforward_start_ms = 0;
        }
    }

    ahrs.set_fly_forward(flyforward);
}

bool Rover::set_mode(Mode &new_mode, mode_reason_t reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        return true;
    }

    Mode &old_mode = *control_mode;
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE,
                                 LogErrorCode(new_mode.mode_number()));
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");
        return false;
    }

    control_mode = &new_mode;

    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    g2.fence.manual_recovery_start();

#if CAMERA == ENABLED
    camera.set_is_auto_mode(control_mode->mode_number() == Mode::Number::AUTO);
#endif

    old_mode.exit();

    control_mode_reason = reason;
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    gcs().send_message(MSG_HEARTBEAT);

    notify_mode(control_mode);
    return true;
}

void Rover::startup_INS_ground(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Beginning INS calibration. Do not move vehicle");
    hal.scheduler->delay(100);

	//hal.uartB->printf("%s ahrs.init()\n", __func__);
    ahrs.init();
    // say to EKF that rover only move by going forward
    //hal.uartB->printf("ahrs.set_fly_forward(true)\n");
    ahrs.set_fly_forward(true);

	//hal.uartB->printf("ahrs.set_vehicle_class()\n");
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);

	//hal.uartB->printf("ins.init()\n");
    ins.init(scheduler.get_loop_rate_hz());

	//hal.uartB->printf("ahrs.reset()\n");
    ahrs.reset();
}

// update notify with mode change
void Rover::notify_mode(const Mode *mode)
{
    AP_Notify::flags.autopilot_mode = mode->is_autopilot_mode();
    notify.flags.flight_mode = mode->mode_number();
    notify.set_flight_mode_str(mode->name4());
}

/*
  check a digital pin for high,low (1/0)
 */
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    // ensure we are in input mode
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(pin, 1);

    return hal.gpio->read(pin);
}

/*
  should we log a message type now?
 */
bool Rover::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}

// returns true if vehicle is a boat
// this affects whether the vehicle tries to maintain position after reaching waypoints
bool Rover::is_boat() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BOAT);
}
