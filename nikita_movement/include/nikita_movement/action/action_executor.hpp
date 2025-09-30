/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>
#include <list>

#include "handler/servohandler.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/requests.hpp"

class CActionExecutor {
   public:
    CActionExecutor(std::shared_ptr<rclcpp::Node> node);
    virtual ~CActionExecutor() = default;

    void request(std::vector<std::shared_ptr<CRequestBase>> requests_v);
    void requestWithoutQueue(std::vector<std::shared_ptr<CRequestBase>> requests_v);
    void cancelRunningRequest();
    bool isDone();

   private:
    void executeNextPendingRequest();
    void execute(std::vector<std::shared_ptr<CRequestBase>>& requests_v);
    void onServoHandlerDone();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CServoHandler> handlerServo_;
    std::list<std::vector<std::shared_ptr<CRequestBase>>> m_pendingRequests_l;
    std::vector<std::shared_ptr<CRequestBase>> m_activeRequest_v;
};
