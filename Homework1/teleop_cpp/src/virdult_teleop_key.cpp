#include <iostream>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/bool.hpp" //This is for autonomous mode

class TeleopKeyNode : public rclcpp::Node
{
public:
    TeleopKeyNode()
        : Node("virdult_teleop_key"), autonomous_mode_(false), running_(true)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("autonomous_mode", 10); //For Autonomous Mode
        input_thread_ = std::thread(&TeleopKeyNode::keyboardLoop, this);
        RCLCPP_INFO(this->get_logger(), "Teleop controller started!");
        RCLCPP_INFO(this->get_logger(), "Use W/A/S/D to move, X to toggle mode.");
    }

    ~TeleopKeyNode()
    {
        running_ = false;
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_; //For Autonomous mode
    std::thread input_thread_;
    std::atomic<bool> autonomous_mode_;
    std::atomic<bool> running_;
    std::unordered_map<int, bool> key_state_;

    // Change this if your keyboard event file differs (check with `ls /dev/input/by-path/`)
    const std::string keyboard_device_ = "/dev/input/event11";

    void keyboardLoop()
{
    int fd = open(keyboard_device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open keyboard device: %s", keyboard_device_.c_str());
        RCLCPP_ERROR(this->get_logger(), "Use `ls /dev/input/by-path/` to find the correct event file.");
        return;
    }

    geometry_msgs::msg::Twist twist;
    struct input_event ev;
    rclcpp::Rate rate(200); // 200Hz for responsiveness
    bool x_pressed_once = false;

    while (rclcpp::ok() && running_)
    {
        ssize_t n = read(fd, &ev, sizeof(struct input_event));
        if (n > 0 && ev.type == EV_KEY)
        {
            if (ev.value == 1) // press
            {
                key_state_[ev.code] = true;

                if (ev.code == KEY_X && !x_pressed_once)
                {
                    autonomous_mode_ = !autonomous_mode_;

                    // Create and publish Boolean message -> For Autonomous Mode
                    std_msgs::msg::Bool mode_msg;
                    mode_msg.data = autonomous_mode_;
                    mode_pub_->publish(mode_msg);
                    RCLCPP_INFO(this->get_logger(), "Published autonomous_mode = %d", mode_msg.data);

                    RCLCPP_INFO(this->get_logger(), "Switched to %s mode",
                                autonomous_mode_ ? "AUTONOMOUS" : "MANUAL");
                    x_pressed_once = true; // prevent holding spam
                }
            }
            else if (ev.value == 0) // release
            {
                key_state_[ev.code] = false;

                if (ev.code == KEY_X)
                    x_pressed_once = false; // ready for next press
            }
        }

        if (autonomous_mode_)
        {
            // Do NOT publish when autonomous mode is active
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        else
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            if (key_state_[KEY_W]) twist.linear.x += 1.0;
            if (key_state_[KEY_S]) twist.linear.x -= 1.0;
            if (key_state_[KEY_A]) twist.angular.z += 1.0;
            if (key_state_[KEY_D]) twist.angular.z -= 1.0;
        }

        pub_->publish(twist);
        rate.sleep();
    }

    close(fd);
}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
