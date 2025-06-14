/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);
		pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose_cmd", 10,
            std::bind(&OffboardControl::cmd_pose_callback, this, std::placeholders::_1)
        );
		cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel", 10,
			std::bind(&OffboardControl::cmd_vel_callback, this, std::placeholders::_1)
		);

		offboard_setpoint_counter_ = 0;


		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}


	TrajectorySetpoint cmd_msg; 
	bool reciv_pos_cmd = false; // Flag to check if a position command has been received
	bool first = true; // Flag to check if this is the first time we are sending a setpoint

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_; 

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void cmd_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg); 
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
*/

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); 

	RCLCPP_INFO(this->get_logger(), "Disarm command send"); 
}


void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};

	// Publish position cmd
	if (!reciv_pos_cmd && first){
		// If no position command has been received, use the default position
		msg.position = {0.0, 0.0, -5.0}; // Negate z for PX4
		msg.yaw = -3.14; // [-PI:PI]
		first = false; 
	} else {
		// Use the position from the received command
		msg.position = cmd_msg.position; // Use the position from the received command
		msg.yaw = cmd_msg.yaw; // Use the yaw from the received command
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::cmd_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
	// This callback can be used to receive pose commands from another node
	// For example, you can use this to set the desired position of the vehicle
	RCLCPP_INFO(this->get_logger(), "Received pose command: [x: %f, y: %f, z: %f]",
	            msg->position.x, msg->position.y, msg->position.z);

	reciv_pos_cmd = true; 

	cmd_msg.position = {msg->position.x, msg->position.y, -msg->position.z}; // Negate z for PX4
	// TODO: Set the orientation if needed 
	cmd_msg.yaw = 0.0; // Set yaw to 0, or you can use msg->orientation if needed
	trajectory_setpoint_publisher_->publish(cmd_msg);
	RCLCPP_INFO(this->get_logger(), "Published trajectory setpoint based on received pose command.");
}

void OffboardControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	// This callback can be used to receive velocity commands from another node
	// For example, you can use this to set the desired velocity of the vehicle
	RCLCPP_INFO(this->get_logger(), "Received velocity command: [vx: %f, vy: %f, vz: %f]",
	            msg->linear.x, msg->linear.y, msg->linear.z);

	// You can also publish a trajectory setpoint based on the received velocity command
	TrajectorySetpoint vel_msg{};
	vel_msg.velocity = {msg->linear.x, msg->linear.y, msg->linear.z}; // Negate z for PX4
	vel_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	//trajectory_setpoint_publisher_->publish(vel_msg);
	RCLCPP_INFO(this->get_logger(), "Published trajectory setpoint based on received velocity command.");
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
