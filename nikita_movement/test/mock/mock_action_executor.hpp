#pragma once

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "action/action_executor.hpp"
#include "requester/requests.hpp"

class CMockActionExecutor : public CActionExecutor {
  public:
    CMockActionExecutor(std::shared_ptr<rclcpp::Node> node) : CActionExecutor(node), last_requested_count(0), done(true) {}
    void request(std::vector<std::shared_ptr<CRequestBase>> requests_v) override {
        last_requested_count = requests_v.size();
        last_requests = requests_v;
    }
    void requestWithoutQueue(std::vector<std::shared_ptr<CRequestBase>> requests_v) override {
        last_requested_count = requests_v.size();
        last_requests = requests_v;
    }
    void cancelRunningRequest() override {
        last_requested_count = 0;
        last_requests.clear();
    }
    bool isDone() override {
        return done;
    }

    // Typed helpers to inspect stored requests in tests
    template<typename T>
    std::shared_ptr<T> getRequestAs(size_t idx) const {
        if (idx >= last_requests.size()) return nullptr;
        return std::dynamic_pointer_cast<T>(last_requests[idx]);
    }

    // Count how many stored requests are of given derived type
    template<typename T>
    size_t countOfType() const {
        size_t c = 0;
        for (auto &r : last_requests) {
            if (std::dynamic_pointer_cast<T>(r)) ++c;
        }
        return c;
    }

    // Convenience getters for common request types used in tests
    std::shared_ptr<CRequestLegs> getLegsRequest(size_t idx = 0) const { return getRequestAs<CRequestLegs>(idx); }
    std::shared_ptr<CRequestSendDuration> getDurationRequest(size_t idx = 0) const { return getRequestAs<CRequestSendDuration>(idx); }
    std::shared_ptr<CRequestHead> getHeadRequest(size_t idx = 0) const { return getRequestAs<CRequestHead>(idx); }

    // Find first index of given request type in the stored vector, or -1 if not found
    template<typename T>
    ssize_t findFirstIndexOfType() const {
        for (size_t i = 0; i < last_requests.size(); ++i) {
            if (std::dynamic_pointer_cast<T>(last_requests[i])) return static_cast<ssize_t>(i);
        }
        return -1;
    }

    // Convenience: get first occurrence of common request types
    std::shared_ptr<CRequestLegs> getFirstLegsRequest() const {
        ssize_t idx = findFirstIndexOfType<CRequestLegs>();
        if (idx < 0) return nullptr;
        return getLegsRequest(static_cast<size_t>(idx));
    }
    std::shared_ptr<CRequestSendDuration> getFirstDurationRequest() const {
        ssize_t idx = findFirstIndexOfType<CRequestSendDuration>();
        if (idx < 0) return nullptr;
        return getDurationRequest(static_cast<size_t>(idx));
    }
    std::shared_ptr<CRequestHead> getFirstHeadRequest() const {
        ssize_t idx = findFirstIndexOfType<CRequestHead>();
        if (idx < 0) return nullptr;
        return getHeadRequest(static_cast<size_t>(idx));
    }

    // Extractors for payload data
    std::map<ELegIndex, CLegAngles> getLegAnglesMap(size_t idx = 0) const {
        // if the requested index doesn't contain a legs request, return the first legs request
        auto r = getLegsRequest(idx);
        if (!r) r = getFirstLegsRequest();
        if (!r) return {};
        return r->angles();
    }

    uint32_t getDurationMs(size_t idx = 0) const {
        // return the duration from the request at idx if it exists and is the right type;
        // otherwise return the first duration request found.
        auto r = getDurationRequest(idx);
        if (!r) r = getFirstDurationRequest();
        if (!r) return 0u;
        return r->durationMs();
    }

    // Helper to get head angles if present
    std::pair<float, float> getHeadAngles() const {
        auto r = getFirstHeadRequest();
        if (!r) return {0.0f, 0.0f};
        return {r->angleHorizontal(), r->angleVertical()};
    }

    size_t last_requested_count;
    std::vector<std::shared_ptr<CRequestBase>> last_requests;
    bool done;
};
