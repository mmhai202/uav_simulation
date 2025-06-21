#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DroneControl : public rclcpp::Node
{
public:
    DroneControl();
    void arm();
    void disarm();
    void offboard_mode();
    void manual_mode();
    void land();

private:
    rclcpp::Publisher<VehicleCommand>::SharedPtr            vehicle_command_pub_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr       offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr        trajectory_setpoint_pub_;
    rclcpp::Publisher<ManualControlSetpoint>::SharedPtr     manual_control_setpoint_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_;
    
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_manual_control_setpoint(float roll, float pitch, float yaw, float thrust);
};

#endif // DRONE_CONTROL_H
