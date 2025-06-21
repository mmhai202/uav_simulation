#include "drone_control.h"

DroneControl::DroneControl() : Node("offboard_control")
{
    offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {

        if (offboard_setpoint_counter_ == 10) {
            // Change to offboard mode after 10 setpoints
            this->offboard_mode();

            // Arm the vehicle
            this->arm();
        }

        if (offboard_setpoint_counter_ == 150) {
            // Land after 150 setpoints
            this->land();
        }

        // Stop the counter after 150 setpoints
        if (offboard_setpoint_counter_ < 151) {
            offboard_setpoint_counter_++;
        }

        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    };
    // Create a timer that calls the callback every 100 milliseconds
    timer_ = this->create_wall_timer(100ms, timer_callback);
}

void DroneControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void DroneControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void DroneControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Land command sent");
}

void DroneControl::offboard_mode() 
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

void DroneControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
}

void DroneControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
}

void DroneControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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
    vehicle_command_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControl>());
    rclcpp::shutdown();
    return 0;
}