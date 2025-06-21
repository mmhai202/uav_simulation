#include "drone_control.h"

DroneControl::DroneControl() : Node("manual_control")
{
    vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    manual_control_setpoint_pub_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {

        if (offboard_setpoint_counter_ < 10) {
            // Publish manual control input with low throttle
            this->publish_manual_control_setpoint(0.0, 0.0, 0.0, -1.0);
        }

        if (offboard_setpoint_counter_ == 10) {
            // Switch to manual mode
            this->manual_mode();

            // Arm the vehicle
            this->arm();
        }

        if (offboard_setpoint_counter_ > 10) {
            // Publish manual control input
            this->publish_manual_control_setpoint(0.0, 0.0, 0.0, 0.5);
        }

        if (offboard_setpoint_counter_ == 150) {
            // Land after 150 setpoints
            this->land();
        }

        // Stop the counter after 150 setpoints
        if (offboard_setpoint_counter_ < 151) {
            offboard_setpoint_counter_++;
        }
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

void DroneControl::manual_mode()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
    RCLCPP_INFO(this->get_logger(), "Manual mode command sent");
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

void DroneControl::publish_manual_control_setpoint(float roll, float pitch, float yaw, float throttle)
{
    ManualControlSetpoint msg{};
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    msg.throttle = throttle;
    msg.valid = true;
    msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = msg.timestamp;
    manual_control_setpoint_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting manual control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControl>());
    rclcpp::shutdown();
    return 0;
}