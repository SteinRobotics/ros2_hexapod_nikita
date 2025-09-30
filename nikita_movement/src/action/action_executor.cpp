/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "action/action_executor.hpp"

using namespace nikita_interfaces::msg;

CActionExecutor::CActionExecutor(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    handlerServo_ = std::make_shared<CServoHandler>(node);

    handlerServo_->setOnDoneCallback([this]() { this->onServoHandlerDone(); });
}

void CActionExecutor::onServoHandlerDone() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionExecutor:: Received callback: Servo action is done.");
    executeNextPendingRequest();
}

void CActionExecutor::requestWithoutQueue(std::vector<std::shared_ptr<CRequestBase>> requests_v) {
    if (!m_activeRequest_v.empty()) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "CActionExecutor:: requestWithoutQueue but queue is not empty");
    }
    execute(requests_v);
}

void CActionExecutor::request(std::vector<std::shared_ptr<CRequestBase>> requests_v) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CActionExecutor:: new request ");
    m_pendingRequests_l.push_back(requests_v);
    if (isDone()) executeNextPendingRequest();
}

bool CActionExecutor::isDone() {
    return m_activeRequest_v.empty();
}

void CActionExecutor::executeNextPendingRequest() {
    if (!m_pendingRequests_l.empty()) {
        m_activeRequest_v = m_pendingRequests_l.front();
        m_pendingRequests_l.pop_front();
    } else {
        m_activeRequest_v.clear();
    }
    if (m_activeRequest_v.empty()) {
        return;
    }
    execute(m_activeRequest_v);
}

void CActionExecutor::execute(std::vector<std::shared_ptr<CRequestBase>>& requests_v) {
    for (const auto& request : requests_v) {
        if (auto requestLeg = std::dynamic_pointer_cast<CRequestLeg>(request)) {
            handlerServo_->run(requestLeg);
        } else if (auto requestLegs = std::dynamic_pointer_cast<CRequestLegs>(request)) {
            handlerServo_->run(requestLegs);
        } else if (auto requestHead = std::dynamic_pointer_cast<CRequestHead>(request)) {
            handlerServo_->run(requestHead);
        } else if (auto requestSendAndWait = std::dynamic_pointer_cast<CRequestSendDuration>(request)) {
            handlerServo_->run(requestSendAndWait);
        } else {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "CActionExecutor: RequestType unknown: " << typeid(*request).name());
        }
    }
}

void CActionExecutor::cancelRunningRequest() {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CActionExecutor::cancelRunningRequest");
    m_activeRequest_v.clear();
    handlerServo_->cancel();
}
