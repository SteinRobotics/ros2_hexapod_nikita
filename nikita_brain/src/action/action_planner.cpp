/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "action/action_planner.hpp"

namespace brain {
CActionPlanner::CActionPlanner(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    handlerSystem_ = std::make_shared<CSystem>(node);
    handlerCommunication_ = std::make_shared<CCommunication>(node);
    handlerMovement_ = std::make_shared<CMovement>(node);

    handlers_.push_back(handlerSystem_);
    handlers_.push_back(handlerCommunication_);
    handlers_.push_back(handlerMovement_);
}

void CActionPlanner::request(std::vector<std::shared_ptr<RequestBase>> requests, Prio prio) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner:: new requests with prio " << (int)prio);

    // Priority handling:
    // - Highest: Interrupts everything, clears all queues, cancels running requests, and executes immediately.
    // - High: If a Highest-priority request is active, ignore. If High is active, queue. If Normal is active, promote it to queue and execute High.
    // - Normal: If Highest is active, ignore. Otherwise, queue.
    // - Background: Only queued if nothing else is running.

    switch (prio) {
        case Prio::Highest:
            // Clear all other queues and background, cancel running request, and push to highest-priority queue
            requestsHighPrio_.clear();
            requestsNormalPrio_.clear();
            requestBackground_.clear();
            cancelRunningRequest();
            requestsHighestPrio_.push_back(requests);
            break;

        case Prio::High:
            // If a Highest-priority request is active, ignore this one
            if (activePrio_ == Prio::Highest) break;
            // If a High-priority request is active, queue this one
            if (activePrio_ == Prio::High) {
                requestsHighPrio_.push_back(requests);
                break;
            }
            // TODO I think better deactivate the following feature and remove the activeRequests_
            // If a Normal-priority request is active, promote it to the normal queue and execute this High-priority request
            // if (activePrio_ == Prio::Normal) {
            //     requestsNormalPrio_.push_back(activeRequests_);
            // }
            cancelRunningRequest();
            requestsHighPrio_.push_back(requests);
            break;

        case Prio::Normal:
            // If a Highest-priority request is active, ignore this one
            if (activePrio_ == Prio::Highest) break;
            // Otherwise, queue this normal-priority request
            requestsNormalPrio_.push_back(requests);
            break;

        case Prio::Background:
            // If a Highest-priority request is active, ignore this one
            if (activePrio_ == Prio::Highest) break;
            // Set as background request (executed when nothing else is running)
            requestBackground_ = requests;
            break;

        default:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "unknown prio of the new request");
            break;
    }
}

void CActionPlanner::update() {
    bool isDone = true;
    for (auto handler : handlers_) {
        handler->update();
        if (!handler->done()) {
            isDone = false;
        }
    }
    isDone_ = isDone;
    if (!isDone_) {
        return;
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "schedule new request");
    if (!requestsHighestPrio_.empty()) {
        activeRequests_ = requestsHighestPrio_.front();
        requestsHighestPrio_.pop_front();
        activePrio_ = Prio::Highest;
    } else if (!requestsHighPrio_.empty()) {
        activeRequests_ = requestsHighPrio_.front();
        requestsHighPrio_.pop_front();
        activePrio_ = Prio::High;
    } else if (!requestsNormalPrio_.empty()) {
        activeRequests_ = requestsNormalPrio_.front();
        requestsNormalPrio_.pop_front();
        activePrio_ = Prio::Normal;
    } else {
        activeRequests_ = requestBackground_;
        activePrio_ = Prio::Background;
    }
    execute(activeRequests_);
}

/// here the old code from request_executor.cpp
void CActionPlanner::execute(std::vector<std::shared_ptr<RequestBase>>& requests_v) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner::execute");
    for (const auto& request : requests_v) {
        if (auto requestSystem = std::dynamic_pointer_cast<RequestSystem>(request)) {
            handlerSystem_->run(requestSystem);
        } else if (auto requestTalking = std::dynamic_pointer_cast<RequestTalking>(request)) {
            handlerCommunication_->run(requestTalking);
        } else if (auto requestChat = std::dynamic_pointer_cast<RequestChat>(request)) {
            handlerCommunication_->run(requestChat);
        } else if (auto requestMusic = std::dynamic_pointer_cast<RequestMusic>(request)) {
            handlerCommunication_->run(requestMusic);
        } else if (auto requestListening = std::dynamic_pointer_cast<RequestListening>(request)) {
            handlerCommunication_->run(requestListening);
        } else if (auto requestMove = std::dynamic_pointer_cast<CRequestMove>(request)) {
            handlerMovement_->run(requestMove);
        } else {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "CActionPlanner: RequestType unknown: " << typeid(*request).name());
        }
    }
}

void CActionPlanner::cancelRunningRequest() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner::cancelRunningRequest");
    for (auto handler : handlers_) {
        handler->cancel();
    }
}

bool CActionPlanner::done() {
    return isDone_;
}

}  // namespace brain
