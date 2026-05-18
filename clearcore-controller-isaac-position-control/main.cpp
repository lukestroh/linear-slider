/*
	* Author: Luke Strohbehn
	* Created: 09/21/2023
	* 
	* Requirements:
	*
	* ETHERNET COMMUNICATION:
	* 1. The PC should be running software capable of sending and receiving UDP 
	*    packets. See `udp_client.py` for simple testing.
	*
	*
	* MOTOR:
	* 1. A ClearPath motor must be connected to Connector M-0.
	* 2. The connected ClearPath motor must be configured through the MSP software
	*    for step and direction position control.
	* 3. The ROS2 hardware interface sends commands as "status,pos_steps" UDP
	*    payloads. This firmware sends feedback as JSON with "status" and
	*    "pos_steps" fields.
	*    * Set the HLFB mode to "ASG-Velocity w/Measured Torque" with a PWM carrier
	*      frequency of 482 Hz through the MSP software (select Advanced>>High
	*      Level Feedback [Mode]... then choose "ASG-Velocity w/Measured Torque"
	*      from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier
	*      Frequency" dropdown, and hit the OK button).
	*
	* LIMIT SWITCHES:
	* 1. Limit switches should be connected to the DI8 and DI7 inputs on the controller.
	*
	*
	* EMERGENCY STOP:
	* 1. Emergency stop should be connected to the DI-6 input on the controller.
	*
	* Links:
	* ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
	* ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
	*
	*
	* Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
	* the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 0
#endif

#include "ClearCore.h"
#include "EthUDP.h"
#include "clearpathmc.h"
#include "system.h"

#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__ || __CPMC_DEBUG__
#define __SET_UP_SERIAL__ 1
#else
#define __SET_UP_SERIAL__ 0
#endif


// System state variables
volatile bool neg_lim_switch_flag = false;
volatile bool pos_lim_switch_flag = false;
volatile bool e_stop_flag = false;
constexpr uint8_t DEBOUNCE_TIME = 10;



#if __SERIAL_DEBUG__ || __SET_UP_SERIAL__
void set_up_serial(void) {
	/* Set up Serial communication with computer for debugging */
	ConnectorUsb.Mode(Connector::USB_CDC);
	ConnectorUsb.Speed(115200);
	ConnectorUsb.PortOpen();
	uint32_t serial_timeout = 5000;
	uint32_t start_time = Milliseconds();
	while (!ConnectorUsb && Milliseconds() - start_time < serial_timeout) {
		continue;
	}
}
#endif // __SERIAL_DEBUG__

bool read_switch(DigitalIn* switch_pin, volatile bool* p_interrupt_flag) {
	/* Returns true if the interrupt has been triggered */
	if (*p_interrupt_flag) {
		bool reading = switch_pin->State();
		static unsigned long prev_time;
		static bool change_pending = false;
		if (reading) {
			change_pending = true;
		}
		if (!reading && change_pending) {
			if (Milliseconds() - prev_time > DEBOUNCE_TIME) {
				*p_interrupt_flag = false;
				change_pending = false;
				return true;
			}
		}
	}	
	return false;
}

bool poll_switch(DigitalIn* switch_pin) {
	return switch_pin->State();
}


int main(void) {
#if __SET_UP_SERIAL__
	set_up_serial();
#endif

	// Set static address for the ClearCore Controller
	IpAddress local_ip(169, 254, 57, 177);
	// Set remote (host) computer address
	IpAddress remote_ip(169, 254, 57, 209);

	EthUDP eth(local_ip, 8888, remote_ip, 44644);

	ClearPathMC motor0(0);

	eth.begin();
	motor0.begin();
	
	ConnectorUsb.SendLine("BOOT OK");
	
	if (motor0.emergency_stop_active()) {
		motor0.state_.system_status = slidersystem::E_STOP;
		e_stop_flag = true;
	}
	else if (motor0.negative_limit_active()) {
		motor0.state_.system_status = slidersystem::NEG_LIM;
	}
	else if (motor0.positive_limit_active()) {
		motor0.state_.system_status = slidersystem::POS_LIM;
	}
	eth.send_packet(&motor0.state_);
	
	uint32_t last_time_us = Microseconds();
	const uint32_t send_period_us = 2000;
		
	// Main loop
	while (true) {
		
		static uint32_t last = 0;
		if (Milliseconds() - last > 1000) {
			last = Milliseconds();
			ConnectorUsb.SendLine("RUNNING");
		}
		
		// Read data from the ROS2 hardware interface, store in motor command interface.
		eth.read_packet(&motor0.command_);
				
		// If new data, parse for new motor control
		if (eth.new_data) {
			
			#if __SERIAL_DEBUG__
			//ConnectorUsb.Send("Command - status: ");
			//ConnectorUsb.Send(motor0.command_.system_status);
			//ConnectorUsb.Send(" rpm: ");
			//ConnectorUsb.SendLine(motor0.command_.vel);
			//
			//ConnectorUsb.Send("State - status: ");
			//ConnectorUsb.Send(motor0.state_.system_status);
			//ConnectorUsb.Send(" rpm: ");
			//ConnectorUsb.SendLine(motor0.state_.vel);
			#endif
			switch (motor0.command_.system_status) {
				

					case slidersystem::E_STOP:
					e_stop_flag = true;
					break;

					case slidersystem::SYSTEM_OK:
					motor0.state_.system_status = slidersystem::SYSTEM_OK;
					motor0.set_position_steps(motor0.command_.pos_steps);
					break;

					case slidersystem::SYSTEM_STANDBY:
					motor0.state_.system_status = slidersystem::SYSTEM_STANDBY;
					motor0.stop_position_move();
					break;

					case slidersystem::SYSTEM_CALIBRATING:
				 // TODO: implement non-blocking homing
					motor0.state_.system_status = slidersystem::SYSTEM_CALIBRATING;
					motor0.stop_position_move();      // ensure we don't move unexpectedly
					break;

					case slidersystem::NEG_LIM:
					motor0.state_.system_status = slidersystem::NEG_LIM;
					motor0.stop_position_move();
					break;

					case slidersystem::POS_LIM:
					motor0.state_.system_status = slidersystem::POS_LIM;
					motor0.stop_position_move();
					break;
				
			}
			eth.new_data = false;
		}
		
		// Limit switch check
		if (neg_lim_switch_flag) {
			motor0.stop_position_move();
			motor0.state_.system_status = slidersystem::NEG_LIM;
			neg_lim_switch_flag = false;
		}
		if (pos_lim_switch_flag) {
			motor0.stop_position_move();
			motor0.state_.system_status = slidersystem::POS_LIM;
			pos_lim_switch_flag = false;
		}

		// E stop check
		if (e_stop_flag) {
			motor0.stop_position_move();
			motor0.state_.system_status = slidersystem::E_STOP;
#if __SERIAL_DEBUG__
			ConnectorUsb.SendLine("EMERGENCY STOP TRIGGERED. CHECK ALL HARDWARE.");
#endif
		}
		
		if (motor0.negative_limit_active()) {
			motor0.stop_position_move();
			motor0.state_.system_status = slidersystem::NEG_LIM;
		}
		if (motor0.positive_limit_active()) {
			motor0.stop_position_move();
			motor0.state_.system_status = slidersystem::POS_LIM;
		}
		
		// Poll E-stop so user knows to properly reset the switch TODO: move all e-stop stuff to interrupt
		if (motor0.emergency_stop_active()) {
			motor0.state_.system_status = slidersystem::E_STOP;
			e_stop_flag = true;
		}
		
		
		

		motor0.service_position_move();



		// Send status and position data to the ROS2 node without blocking the control loop.
		uint32_t now_us = Microseconds();
		if (now_us - last_time_us >= send_period_us)	{
			last_time_us = now_us;
			eth.send_packet(&motor0.state_);
		}
		
	}
}
