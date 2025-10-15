/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/requester.hpp"

using namespace nikita_interfaces::msg;
using std::placeholders::_1;

CRequester::CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionExecutor> actionExecutor)
    : node_(node), actionExecutor_(actionExecutor) {
    kinematics_ = std::make_shared<CKinematics>(node);
    gaitController_ = std::make_shared<CGaitController>(node, kinematics_);
    actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node);

    initializeRequestHandlers();

    m_subMovementRequest = node_->create_subscription<MovementRequest>(
        "movement_request", 10, std::bind(&CRequester::onMovementRequest, this, _1));
}

// ------------------------------------------------------------------------------------------------
// private methods for the CRequester class
// ------------------------------------------------------------------------------------------------------------
void CRequester::initializeRequestHandlers() {
    auto bind = [this](auto fn) { return [this, fn](const MovementRequest& msg) { (this->*fn)(msg); }; };

    requestHandlers_ = {
        {MovementRequest::NO_REQUEST,
         [this](const MovementRequest& msg) {
             RCLCPP_INFO_STREAM(node_->get_logger(),
                                "CRequester::onMovementRequest: NO_REQUEST: " << msg.name);
         }},
        {MovementRequest::LAYDOWN, bind(&CRequester::requestLayDown)},
        {MovementRequest::STAND_UP, bind(&CRequester::requestStandUp)},
        {MovementRequest::WAITING, bind(&CRequester::requestWaiting)},
        {MovementRequest::MOVE, bind(&CRequester::requestMove)},
        {MovementRequest::MOVE_TO_STAND, bind(&CRequester::requestMoveToStand)},
        {MovementRequest::WATCH, bind(&CRequester::requestSequence)},
        {MovementRequest::LOOK_LEFT, bind(&CRequester::requestSequence)},
        {MovementRequest::LOOK_RIGHT, bind(&CRequester::requestSequence)},
        {MovementRequest::DANCE, bind(&CRequester::requestDance)},
        {MovementRequest::HIGH_FIVE, bind(&CRequester::requestHighFive)},
        {MovementRequest::LEGS_WAVE, bind(&CRequester::requestLegsWave)},
        {MovementRequest::BODY_ROLL, bind(&CRequester::requestBodyRoll)},
        {MovementRequest::BITE, bind(&CRequester::requestBite)},
        {MovementRequest::STOMP, bind(&CRequester::requestStomp)},
        {MovementRequest::CLAP, bind(&CRequester::requestClap)},
        {MovementRequest::TRANSPORT, bind(&CRequester::requestTransport)},
        {MovementRequest::TESTBODY, bind(&CRequester::requestTestBody)},
        {MovementRequest::TESTLEGS, bind(&CRequester::requestTestLegs)},
        {MovementRequest::NEUTRAL, bind(&CRequester::requestNeutral)},
        {MovementRequest::CALIBRATE, bind(&CRequester::requestCalibrate)},
        // Add more handlers as needed
    };
}

void CRequester::requestLayDown(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::LAYDOWN;

    kinematics_->setHead(0.0, -20.0);
    kinematics_->moveBody(kinematics_->getLegsLayDownPositions());
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestStandUp(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::STAND_UP;

    kinematics_->setHead(0.0, 0.0);
    kinematics_->moveBody(kinematics_->getLegsStandingPositions());
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestMoveToStand(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::MOVE_TO_STAND;

    kinematics_->setHead(0.0, 0.0);
    // set the legs to the standing position
    //TODO improve the calculation of the lifted legs
    bool tripodFirstGroupLegsLifted = kinematics_->getLegsPositions().at(ELegIndex::LeftFront).z >
                                      kinematics_->getLegsStandingPositions().at(ELegIndex::LeftFront).z;
    gaitController_->liftLegsTripodGroup(tripodFirstGroupLegsLifted);
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms / 2.0)});
    // set the legs to the standing position
    kinematics_->moveBody(kinematics_->getLegsStandingPositions());
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms / 2.0)});
}

void CRequester::requestWaiting(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::WAITING;

    auto currentLegsPositions = kinematics_->getLegsPositions();
    auto currentHeadPostion = kinematics_->getHead();

    kinematics_->setHead(0.0, 0.0);

    auto zPostionUp = kinematics_->getLegsStandingPositions().at(ELegIndex::LeftFront).z + 0.02;
    auto zPostionDown = zPostionUp - 0.04;

    std::map<ELegIndex, CPosition> waitingPositionUp;
    std::map<ELegIndex, CPosition> waitingPositionDown;

    for (auto& [legIndex, position] : kinematics_->getLegsStandingPositions()) {
        waitingPositionUp[legIndex] = CPosition(position.x, position.y, zPostionUp);
        waitingPositionDown[legIndex] = CPosition(position.x, position.y, zPostionDown);
    }

    int numberOfIterations = msg.duration_ms / 2000;  // 2000ms = 2 seconds per iteration
    int waitingTime = msg.duration_ms / numberOfIterations;

    for (int i = 0; i < numberOfIterations; ++i) {
        kinematics_->moveBody(waitingPositionUp);
        actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                                  std::make_shared<CRequestHead>(kinematics_->getHead()),
                                  std::make_shared<CRequestSendDuration>(waitingTime)});

        kinematics_->moveBody(waitingPositionDown);
        actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                                  std::make_shared<CRequestHead>(kinematics_->getHead()),
                                  std::make_shared<CRequestSendDuration>(waitingTime)});
    }
    //
    kinematics_->moveBody(currentLegsPositions);
    kinematics_->setHead(currentHeadPostion);
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(waitingTime)});
}

void CRequester::requestMove(const MovementRequest& msg) {
    // at the beginning of the movement we have to lift the legs
    // TODO make the 500ms dependent on the velocity
    if (activeRequest_ != MovementRequest::MOVE) {
        transitionToMoveActive_ = true;
        gaitController_->liftLegsTripodGroup(true);
        actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                                  std::make_shared<CRequestHead>(kinematics_->getHead()),
                                  std::make_shared<CRequestSendDuration>(500)});
    }
    activeRequest_ = MovementRequest::MOVE;
    velocity_ = msg.velocity;
    poseBody_ = msg.body;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::requestMove: velocity: "
    //                                             << velocity_.linear.x << ", " << velocity_.linear.y << ", "
    //                                             << velocity_.angular.z);
}

// void CRequester::requestLookSideways(const MovementRequest& msg) {
//     float direction = 1.0;
//     if (msg.type == MovementRequest::LOOK_RIGHT) {
//         activeRequest_ = MovementRequest::LOOK_RIGHT;
//     } else {
//         activeRequest_ = MovementRequest::LOOK_LEFT;
//         direction = -1.0;
//     }
//     auto requestedBody = CPose();
//     requestedBody.orientation.yaw = direction * 20.0;
//     kinematics_->setHead(direction * 30.0, 0.0);

//     kinematics_->moveBody(kinematics_->getLegsStandingPositions(), requestedBody);
//     actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
//                               std::make_shared<CRequestHead>(kinematics_->getHead()),
//                               std::make_shared<CRequestSendDuration>(msg.duration_ms / 3)});

//     // wait a bit
//     actionExecutor_->request({std::make_shared<CRequestSendDuration>(msg.duration_ms / 3)});

//     // move back to the neutral position
//     requestedBody.orientation.yaw = 0.0;
//     kinematics_->setHead(0.0, 0.0);
//     kinematics_->moveBody(kinematics_->getLegsStandingPositions(), requestedBody);
//     actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
//                               std::make_shared<CRequestHead>(kinematics_->getHead()),
//                               std::make_shared<CRequestSendDuration>(msg.duration_ms / 3)});
// }

void CRequester::requestSequence(const MovementRequest& msg) {
    activeRequest_ = msg.type;

    auto sequence = actionPackagesParser_->getRequests(msg.name);

    for (const auto& action : sequence) {
        if (action.legAngles.has_value()) {
            const auto& legAnglesMap = action.legAngles.value();
            // Apply angles for all legs contained in the map
            for (const auto& [legIndex, anglesData] : legAnglesMap) {
                CLegAngles legAngles(anglesData.degCoxa, anglesData.degFemur, anglesData.degTibia);
                kinematics_->setLegAngles(legIndex, legAngles);
            }
        }
        if (action.footPositions.has_value()) {
            const auto& footPositionsMap = action.footPositions.value();
            // Apply positions for all legs contained in the map
            for (const auto& [legIndex, positionData] : footPositionsMap) {
                kinematics_->setCartesianFeet(legIndex, positionData);
            }
        }
        if (action.body.has_value()) {
            kinematics_->moveBody(kinematics_->getLegsPositions(), *(action.body));
            // RCLCPP_INFO_STREAM(node_->get_logger(),
            //                    "CRequester::requestSequence: body: "
            //                        << action.body->position.x << ", " << action.body->position.y << ", "
            //                        << action.body->position.z << ", " << action.body->orientation.roll << ", "
            //                        << action.body->orientation.pitch << ", " << action.body->orientation.yaw);
        }
        if (action.head.has_value()) {
            kinematics_->setHead(action.head->degYaw, action.head->degPitch);
            // RCLCPP_INFO_STREAM(node_->get_logger(),
            //                    "CRequester::requestSequence: head: " << action.head->degYaw << ", "
            //                                                          << action.head->degPitch);
        }
        double duration_ms = msg.duration_ms * action.factorDuration;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::requestSequence: duration: " << duration_ms);

        actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                                  std::make_shared<CRequestHead>(kinematics_->getHead()),
                                  std::make_shared<CRequestSendDuration>(duration_ms)});
    }
}

void CRequester::requestDance(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::DANCE;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestHighFive(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::HIGH_FIVE;

    const CLegAngles anglesRightFrontBefore = kinematics_->getAngles(ELegIndex::RightFront);

    kinematics_->setLegAngles(ELegIndex::RightFront, CLegAngles(20.0, 50.0, 60.0));

    actionExecutor_->request(                                           //
        {std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),  //
         std::make_shared<CRequestHead>(0.0, -20.0),                    //
         std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});

    // wait a bit
    actionExecutor_->request({std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});

    // move back to the original position
    kinematics_->setLegAngles(ELegIndex::RightFront, anglesRightFrontBefore);
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});
}

void CRequester::requestLegsWave(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::LEGS_WAVE;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestBodyRoll(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::BODY_ROLL;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning

    // for (int i = 0; i <= 360; i += 10) {
    //     auto x = sin(deg2rad(double(i))) * 4.0;
    //     auto y = sin(deg2rad(double(i))) * 4.0;
    //     auto requestedBody = CPose(x, y, 0.0);
    //     kinematics_->setHead(0.0, 0.0);
    // }

    // auto requestedBody = CPose();
    // requestedBody.orientation.roll = 20.0;
    // kinematics_->setHead(0.0, 10.0);
}

void CRequester::requestBite(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::BITE;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestStomp(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::STOMP;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestClap(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::CLAP;
    auto requestedBody = CPose();
    requestedBody.orientation.pitch = 20.0;
    kinematics_->setHead(0.0, 10.0);

    kinematics_->moveBody(kinematics_->getLegsStandingPositions(), requestedBody);
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms / 3)});
}

void CRequester::requestTransport(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TRANSPORT;

    for (auto& [legIndex, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(legIndex, CLegAngles(0.0, 30.0, -20.0));
    }
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestNeutral(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::NEUTRAL;
    // set the head to the neutral position
    kinematics_->setHead(0.0, 0.0);
    // set all legs to the neutral position
    for (auto& [legIndex, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(legIndex, CLegAngles(0.0, 0.0, 0.0));
    }
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestCalibrate(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::CALIBRATE;

    // set all legs to the kiss ground position
    auto positionsKissGround = kinematics_->getLegsStandingPositions();
    for (auto& [legIndex, position] : positionsKissGround) {
        positionsKissGround[legIndex] = CPosition(position.x, position.y, 0.0);
    }
    kinematics_->moveBody(positionsKissGround);

    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestTestBody(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TESTBODY;

    auto requestedBody = CPose(msg.body);
    kinematics_->moveBody(kinematics_->getLegsStandingPositions(), requestedBody);
    actionExecutor_->request({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                              std::make_shared<CRequestHead>(kinematics_->getHead()),
                              std::make_shared<CRequestSendDuration>(msg.duration_ms)});
}

void CRequester::requestTestLegs(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TESTLEGS;

    for (int idx = static_cast<int>(ELegIndex::RightFront); idx <= static_cast<int>(ELegIndex::LeftBack);
         ++idx) {
        ELegIndex legIndex = static_cast<ELegIndex>(idx);

        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "CRequester:: requestTestLegs: " << legIndexToName.at(legIndex));

        CLegAngles legAngles = kinematics_->getAngles(legIndex);
        CLegAngles origLegAngles = legAngles;
        legAngles.degFemur += 10.0;
        legAngles.degTibia += 10.0;
        legAngles.degCoxa += 10.0;
        kinematics_->setLegAngles(legIndex, legAngles);
        actionExecutor_->request(                                           //
            {std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),  //
             std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});

        actionExecutor_->request({std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});
        kinematics_->setLegAngles(legIndex, origLegAngles);
        actionExecutor_->request(                                           //
            {std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),  //
             std::make_shared<CRequestSendDuration>(msg.duration_ms / 3.0)});
    }
}

// ------------------------------------------------------------------------------------------------------------
// public methods for the CRequester class
// ------------------------------------------------------------------------------------------------------------
void CRequester::onMovementRequest(const MovementRequest& msg) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::onMovementRequest: " << msg.name);
    // check that the new request is inside the requestHandlers_ map
    if (requestHandlers_.find(msg.type) == requestHandlers_.end()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "CRequester::onMovementRequest: " << msg.name << " not known");
        return;
    }
    requestHandlers_[msg.type](msg);
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    // the update is only relevant if the movement request MovementRequest::MOVE is active
    if (activeRequest_ != MovementRequest::MOVE) {
        return;
    }

    // if the movement request is active and we are in the transition to movement
    if (transitionToMoveActive_) {
        if (actionExecutor_->isDone()) {
            RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::transition to Movement done");
            gaitController_->setPhaseNeutral();
            transitionToMoveActive_ = false;
        }
        return;
    }

    // while the movement request is active update the gait controller
    gaitController_->updateCombinedTripodGait(velocity_, poseBody_);
    actionExecutor_->requestWithoutQueue({std::make_shared<CRequestLegs>(kinematics_->getLegsAngles()),
                                          std::make_shared<CRequestHead>(kinematics_->getHead()),
                                          std::make_shared<CRequestSendDuration>(timeslice.count(), false)});
}
