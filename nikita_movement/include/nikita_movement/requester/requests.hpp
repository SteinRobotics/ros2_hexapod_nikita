/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
//
#include "nikita_interfaces/msg/movement_request.hpp"
#include "requester/kinematics.hpp"

class CRequestBase {
   public:
    CRequestBase() {};
    virtual ~CRequestBase() = default;
};

class CRequestLegs : public CRequestBase {
   public:
    CRequestLegs(std::map<ELegIndex, CLegAngles> angles) : angles_(angles) {
    }

    std::map<ELegIndex, CLegAngles> angles() {
        return angles_;
    };

   private:
    std::map<ELegIndex, CLegAngles> angles_;
};

class CRequestLeg : public CRequestBase {
   public:
    CRequestLeg(ELegIndex legIndex, CLegAngles angles) : legIndex_(legIndex), angles_(angles) {
    }
    CRequestLeg(ELegIndex legIndex, float degCoxa, float degFemur, float degTibia) : legIndex_(legIndex) {
        angles_.degCoxa = degCoxa;
        angles_.degFemur = degFemur;
        angles_.degTibia = degTibia;
    }
    ELegIndex legIndex() {
        return legIndex_;
    };
    CLegAngles angles() {
        return angles_;
    };

   private:
    ELegIndex legIndex_;
    CLegAngles angles_;
};

class CRequestHead : public CRequestBase {
   public:
    CRequestHead() = default;
    CRequestHead(float angleHorizontal, float angleVertical)
        : angleHorizontal_(angleHorizontal), angleVertical_(angleVertical) {
    }
    CRequestHead(CHead head) : angleHorizontal_(head.degYaw), angleVertical_(head.degPitch) {
    }

    float angleHorizontal() {
        return angleHorizontal_;
    };

    float angleVertical() {
        return angleVertical_;
    };

   private:
    float angleHorizontal_ = float(0);
    float angleVertical_ = float(0);
};

class CRequestSendDuration : public CRequestBase {
   public:
    CRequestSendDuration(uint32_t durationMs, bool blocking) : durationMs_(durationMs), blocking_(blocking) {
    }
    CRequestSendDuration(uint32_t durationMs) : durationMs_(durationMs), blocking_(true) {
    }
    uint32_t durationMs() {
        return durationMs_;
    }
    bool blocking() {
        return blocking_;
    }

   private:
    uint32_t durationMs_ = uint32_t(0);
    bool blocking_ = true;
};
