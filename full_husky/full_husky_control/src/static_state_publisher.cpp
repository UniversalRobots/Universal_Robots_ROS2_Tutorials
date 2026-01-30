#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class GripperController : public rclcpp::Node
{
public:
    GripperController()
        : Node("gripper_controller"), 
          open_position_(0.0), 
          close_position_(0.7), 
          is_opening_(true),
          wheel_position_(0.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&GripperController::update_joint_state, this));

        // Initialize joint state message
        joint_state_msg_.name = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel", "finger_joint"};
        joint_state_msg_.position = {wheel_position_, wheel_position_, wheel_position_, wheel_position_, open_position_};
    }

private:
    void update_joint_state()
    {
        // Update wheel positions (example: simple incrementing values for demonstration)
        joint_state_msg_.position[0] += 0.10; // front_left_wheel
        joint_state_msg_.position[1] += 0.10; // front_right_wheel
        joint_state_msg_.position[2] += 0.10; // rear_left_wheel
        joint_state_msg_.position[3] += 0.10; // rear_right_wheel

        // Set timestamp and publish joint states
        joint_state_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(joint_state_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
    double open_position_;
    double close_position_;
    bool is_opening_;
    double wheel_position_; // To control wheel positions
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperController>());
    rclcpp::shutdown();
    return 0;
}
