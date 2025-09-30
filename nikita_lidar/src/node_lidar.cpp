/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#include <lidarlite_v3.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class NodeLidar : public rclcpp::Node {
   public:
    NodeLidar() : Node("node_lidar") {
        lidarLite_.i2c_init();

        // Optionally configure LIDAR-Lite
        lidarLite_.configure(0);

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);
        auto timer_callback = [this]() -> void {
            auto busyFlag = lidarLite_.getBusyFlag();

            if (busyFlag == 0x00) {
                // When no longer busy, immediately initialize another measurement
                // and then read the distance data from the last measurement.
                // This method will result in faster I2C rep rates.
                lidarLite_.takeRange();
                auto distance = lidarLite_.readDistance();
                auto message = std_msgs::msg::Float32();
                message.data = float(distance) / 100.0;  // convert cm to m
                RCLCPP_INFO_STREAM(this->get_logger(), "Distance: : " << message.data);
                this->publisher_->publish(message);
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

   private:
    LIDARLite_v3 lidarLite_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeLidar>());
    rclcpp::shutdown();
    return 0;
}