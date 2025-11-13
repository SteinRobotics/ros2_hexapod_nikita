#pragma once

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

enum class EGaitState { Starting, Running, Stopping, Stopped };

class IGait {
   public:
    virtual ~IGait() = default;

    virtual void start() = 0;
    virtual void update(const geometry_msgs::msg::Twist& velocity, const CPose& body) = 0;
    virtual void requestStop() = 0;
    virtual void cancelStop() = 0;
    virtual EGaitState state() const = 0;
};

}  // namespace nikita_movement
