#include "mode.h"
#include "Rover.h"
#include "../libraries/AP_InertialSensor/AP_InertialSensor_NRF24L01.h"

void ModeNrf24g::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeNrf24g::update()
{
/*
	static int cnt = 0;

	if (cnt++>20) {
		cnt = 0;
		hal.uartE->printf("ModeNrf24g::update()\n");
	}*/
	//static float last_steering = 0, last_throttle = 0;
	
    float desired_steering, desired_throttle, desired_lateral = 0;

	AP_InertialSensor_NRF24L01::getInstance()->get_nrf24l01_input(desired_throttle, desired_steering);

    //get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    //get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    /*
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }*/

	/*
	if (((int16_t)last_steering == (int16_t)desired_steering) && ((int16_t)last_throttle == (int16_t)desired_throttle)) {
		return;
	}

	last_steering = desired_steering;
	last_throttle = desired_throttle;
	*/

	//hal.uartE->printf("steering[%d], throttle[%d]\n", (int16_t)desired_steering, (int16_t)desired_throttle);

    // set sailboat mainsail
    float desired_mainsail;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail);
    g2.motors.set_mainsail(desired_mainsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, false);
    g2.motors.set_lateral(desired_lateral);
}


