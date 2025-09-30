/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"

namespace brain {

enum class Prio {
    // is executed immediately, stops all other requests, cancel all other/pending/queued requests
    Highest = 0,
    // is executed immediately, stops all other requests, other request are continued after this request has finished
    High = 1,
    // is queued and executed as soon as no other request is active
    Normal = 2,
    // is executed only if no other pending request exists, only one (the latest) Request with Prio Background can exist. It is stored and can only be set to OFF by highest Prio.
    Background = 3,
};

class RequestBase {
   public:
    RequestBase(float minDuration) : minDuration_(minDuration) {};
    virtual ~RequestBase() = default;

    float minDuration() {
        return minDuration_;
    };

   protected:
    float minDuration_ = float(0);
};

class RequestSystem : public RequestBase {
   public:
    RequestSystem(bool turnOffServoRelay, bool systemShutdown, float minDuration = 0.0)
        : RequestBase(minDuration), turnOffServoRelay_(turnOffServoRelay), systemShutdown_(systemShutdown) {
    }
    bool turnOffServoRelay() {
        return turnOffServoRelay_;
    };
    bool systemShutdown() {
        return systemShutdown_;
    };

   private:
    bool turnOffServoRelay_ = false;
    bool systemShutdown_ = false;
};

class RequestTalking : public RequestBase {
   public:
    RequestTalking(std::string text, std::string language = "de", float minDuration = 0.0)
        : RequestBase(minDuration), text_(text), language_(language) {
    }
    std::string text() {
        return text_;
    };
    std::string language() {
        return language_;
    };

   private:
    std::string text_ = "";
    std::string language_ = "de";
};

class RequestChat : public RequestBase {
   public:
    RequestChat(std::string text, std::string language = "de", float minDuration = 0.0)
        : RequestBase(minDuration), text_(text), language_(language) {
    }
    std::string text() {
        return text_;
    };
    std::string language() {
        return language_;
    };

   private:
    std::string text_ = "";
    std::string language_ = "de";
};

class RequestMusic : public RequestBase {
   public:
    RequestMusic(std::string song, int volumePercent = 50, float minDuration = 0.0)
        : RequestBase(minDuration), song_(song), volumePercent_(volumePercent) {
    }
    std::string song() {
        return song_;
    };
    int volumePercent() {
        return volumePercent_;
    };

   private:
    std::string song_ = "";
    int volumePercent_ = 50;
};

class RequestListening : public RequestBase {
   public:
    RequestListening(bool active, float minDuration = 0.0) : RequestBase(minDuration), active_(active) {
    }
    bool active() {
        return active_;
    };

   private:
    bool active_ = false;
};

class RequestMovementHead : public RequestBase {
   public:
    RequestMovementHead(float agHorizontal, float velHorizontal, float agVertical, float velVertical,
                        float minDuration = 0.0)
        : RequestBase(minDuration),
          agHorizontal_(agHorizontal),
          velHorizontal_(velHorizontal),
          agVertical_(agVertical),
          velVertical_(velVertical) {
    }
    float agHorizontal() {
        return agHorizontal_;
    };
    float velHorizontal() {
        return velHorizontal_;
    };
    float agVertical() {
        return agVertical_;
    };
    float velVertical() {
        return velVertical_;
    };

   private:
    float agHorizontal_ = 0.0;
    float velHorizontal_ = 0.0;
    float agVertical_ = 0.0;
    float velVertical_ = 0.0;
};

class RequestMovementBody : public RequestBase {
   public:
    RequestMovementBody(double velLinearX, double velLinearY, double velAngular, float minDuration = 0.0)
        : RequestBase(minDuration),
          velLinearX_(velLinearX),
          velLinearY_(velLinearY),
          velAngular_(velAngular) {
    }
    double velLinearX() {
        return velLinearX_;
    };
    double velLinearY() {
        return velLinearY_;
    };
    double velAngular() {
        return velAngular_;
    };

   private:
    double velLinearX_ = 0.0;
    double velLinearY_ = 0.0;
    double velAngular_ = 0.0;
};

class CRequestMove : public RequestBase {
   public:
    CRequestMove(nikita_interfaces::msg::MovementRequest movementRequest, float minDuration = 0.0)
        : RequestBase(minDuration), movementRequest_(movementRequest) {
    }
    nikita_interfaces::msg::MovementRequest movementRequest() {
        return movementRequest_;
    };

   private:
    nikita_interfaces::msg::MovementRequest movementRequest_ =
        nikita_interfaces::msg::MovementRequest();
};

class IRequester {
   public:
    virtual ~IRequester() {
    }
    virtual void update() = 0;

    void setDone(bool done) {
        isDone_ = done;
    }
    bool isDone() {
        return isDone_;
    }

   private:
    bool isDone_ = false;
};

}  // namespace brain
