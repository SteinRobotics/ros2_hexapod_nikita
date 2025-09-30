/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace brain {

class IHandler {
   public:
    virtual ~IHandler() {
    }

    virtual void update() = 0;
    virtual void cancel() = 0;

    bool done() {
        return isDone_;
    };

    void setDone(bool state) {
        isDone_ = state;
    }

    // replace protected with private and use the setDone() method
   protected:
    bool isDone_ = true;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace brain