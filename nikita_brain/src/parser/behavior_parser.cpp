/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "nikita_brain/parser/behavior_parser.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace brain {

// Static map of movement type names to their enum values
const std::map<std::string, uint32_t> CBehaviorParser::movementTypeNameMap_ = {
    {"NO_REQUEST", nikita_interfaces::msg::MovementRequest::NO_REQUEST},
    {"LAYDOWN", nikita_interfaces::msg::MovementRequest::LAYDOWN},
    {"STAND_UP", nikita_interfaces::msg::MovementRequest::STAND_UP},
    {"WAITING", nikita_interfaces::msg::MovementRequest::WAITING},
    {"MOVE_TRIPOD", nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD},
    {"WATCH", nikita_interfaces::msg::MovementRequest::WATCH},
    {"LOOK", nikita_interfaces::msg::MovementRequest::LOOK},
    {"DANCE", nikita_interfaces::msg::MovementRequest::DANCE},
    {"HIGH_FIVE", nikita_interfaces::msg::MovementRequest::HIGH_FIVE},
    {"LEGS_WAVE", nikita_interfaces::msg::MovementRequest::LEGS_WAVE},
    {"BODY_ROLL", nikita_interfaces::msg::MovementRequest::BODY_ROLL},
    {"BITE", nikita_interfaces::msg::MovementRequest::BITE},
    {"STOMP", nikita_interfaces::msg::MovementRequest::STOMP},
    {"CLAP", nikita_interfaces::msg::MovementRequest::CLAP},
    {"TESTBODY", nikita_interfaces::msg::MovementRequest::TESTBODY},
    {"TESTLEGS", nikita_interfaces::msg::MovementRequest::TESTLEGS},
    {"NEUTRAL", nikita_interfaces::msg::MovementRequest::NEUTRAL},
    {"CALIBRATE", nikita_interfaces::msg::MovementRequest::CALIBRATE},
};

CBehaviorParser::CBehaviorParser(rclcpp::Node::SharedPtr node) : node_(node) {
}

bool CBehaviorParser::parseFile(const std::string& filePath) {
    try {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open behavior file: %s", filePath.c_str());
            return false;
        }

        json jsonData;
        file >> jsonData;
        file.close();

        return parseString(jsonData.dump());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing file: %s", e.what());
        return false;
    }
}

bool CBehaviorParser::parseString(const std::string& jsonString) {
    try {
        json jsonData = json::parse(jsonString);

        // Parse behavior name
        if (jsonData.contains("name") && jsonData["name"].is_string()) {
            behaviorName_ = jsonData["name"].get<std::string>();
        } else {
            RCLCPP_WARN(node_->get_logger(), "Behavior name not found or invalid");
            behaviorName_ = "UNKNOWN";
        }

        // Parse actions
        if (jsonData.contains("actions") && jsonData["actions"].is_array()) {
            return parseActions(&jsonData["actions"]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Actions array not found in behavior JSON");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing JSON string: %s", e.what());
        return false;
    }
}

std::string CBehaviorParser::getBehaviorName() const {
    return behaviorName_;
}

const std::vector<std::vector<std::shared_ptr<RequestBase>>>& CBehaviorParser::getActionGroups() const {
    return actionGroups_;
}

bool CBehaviorParser::parseActions(const void* actionsJson) {
    const json* actionsArray = static_cast<const json*>(actionsJson);
    actionGroups_.clear();

    for (const auto& actionItem : *actionsArray) {
        if (actionItem.is_object()) {
            auto actionGroup = parseActionGroup(&actionItem);
            if (!actionGroup.empty()) {
                actionGroups_.push_back(actionGroup);
            }
        }
    }

    return !actionGroups_.empty();
}

std::vector<std::shared_ptr<RequestBase>> CBehaviorParser::parseActionGroup(const void* actionJson) {
    const json* actionObject = static_cast<const json*>(actionJson);
    std::vector<std::shared_ptr<RequestBase>> requests;

    // Iterate through all keys in the action object
    for (auto it = actionObject->begin(); it != actionObject->end(); ++it) {
        const std::string& requestType = it.key();
        const json& requestValue = it.value();

        std::shared_ptr<RequestBase> request = nullptr;

        if (requestType == "RequestTalking") {
            request = createRequestTalking(&requestValue);
        } else if (requestType == "RequestChat") {
            request = createRequestChat(&requestValue);
        } else if (requestType == "RequestMusic") {
            request = createRequestMusic(&requestValue);
        } else if (requestType == "RequestListening") {
            request = createRequestListening(&requestValue);
        } else if (requestType == "RequestSystem") {
            request = createRequestSystem(&requestValue);
        } else if (requestType == "CRequestMovementType") {
            request = createRequestMovementType(&requestValue);
        } else if (requestType == "CRequestMoveBody") {
            request = createRequestMoveBody(&requestValue);
        } else if (requestType == "CRequestMoveVelocity") {
            request = createRequestMoveVelocity(&requestValue);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown request type: %s", requestType.c_str());
        }

        if (request != nullptr) {
            requests.push_back(request);
        }
    }

    return requests;
}

std::shared_ptr<RequestTalking> CBehaviorParser::createRequestTalking(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_string()) {
            // Simple string value
            std::string text = jsonValue->get<std::string>();
            return std::make_shared<RequestTalking>(text);
        } else if (jsonValue->is_object()) {
            // Object with text, language, and minDuration
            std::string text = jsonValue->value("text", "");
            std::string language = jsonValue->value("language", "de");
            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<RequestTalking>(text, language, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestTalking: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestChat> CBehaviorParser::createRequestChat(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_string()) {
            std::string text = jsonValue->get<std::string>();
            return std::make_shared<RequestChat>(text);
        } else if (jsonValue->is_object()) {
            std::string text = jsonValue->value("text", "");
            std::string language = jsonValue->value("language", "de");
            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<RequestChat>(text, language, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestChat: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMusic> CBehaviorParser::createRequestMusic(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_string()) {
            std::string song = jsonValue->get<std::string>();
            return std::make_shared<RequestMusic>(song);
        } else if (jsonValue->is_object()) {
            std::string song = jsonValue->value("song", "");
            float volume = jsonValue->value("volume", 0.8f);
            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<RequestMusic>(song, volume, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMusic: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestListening> CBehaviorParser::createRequestListening(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_boolean()) {
            bool active = jsonValue->get<bool>();
            return std::make_shared<RequestListening>(active);
        } else if (jsonValue->is_object()) {
            bool active = jsonValue->value("active", false);
            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<RequestListening>(active, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestListening: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSystem> CBehaviorParser::createRequestSystem(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_object()) {
            bool turnOffServoRelay = jsonValue->value("turnOffServoRelay", false);
            bool systemShutdown = jsonValue->value("systemShutdown", false);
            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<RequestSystem>(turnOffServoRelay, systemShutdown, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSystem: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<CRequestMovementType> CBehaviorParser::createRequestMovementType(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_object()) {
            nikita_interfaces::msg::MovementRequest movementRequest;

            // Parse movement type from "name" field (or fallback to "type" for backward compatibility)
            std::string typeStr = "";
            if (jsonValue->contains("name") && (*jsonValue)["name"].is_string()) {
                typeStr = (*jsonValue)["name"].get<std::string>();
            } else if (jsonValue->contains("type") && (*jsonValue)["type"].is_string()) {
                typeStr = (*jsonValue)["type"].get<std::string>();
            }

            if (!typeStr.empty()) {
                // Look up movement type in map
                auto it = movementTypeNameMap_.find(typeStr);
                if (it != movementTypeNameMap_.end()) {
                    movementRequest.type = it->second;
                    movementRequest.name = typeStr;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Unknown movement type: %s", typeStr.c_str());
                }
            }

            // Parse direction if present
            if (jsonValue->contains("direction") && (*jsonValue)["direction"].is_string()) {
                std::string directionStr = (*jsonValue)["direction"].get<std::string>();
                if (directionStr == "CLOCKWISE") {
                    movementRequest.direction = nikita_interfaces::msg::MovementRequest::CLOCKWISE;
                } else if (directionStr == "ANTICLOCKWISE") {
                    movementRequest.direction = nikita_interfaces::msg::MovementRequest::ANTICLOCKWISE;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Unknown direction: %s", directionStr.c_str());
                }
            }
            if (jsonValue->contains("duration_s")) {
                movementRequest.duration_s = (*jsonValue)["duration_s"].get<double>();
            }

            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<CRequestMovementType>(movementRequest, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create CRequestMovementType: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<CRequestMoveBody> CBehaviorParser::createRequestMoveBody(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_object()) {
            nikita_interfaces::msg::Pose pose;

            // Parse position values (geometry_msgs/Vector3)
            if (jsonValue->contains("position") && (*jsonValue)["position"].is_object()) {
                const auto& position = (*jsonValue)["position"];
                if (position.contains("x")) pose.position.x = position["x"].get<double>();
                if (position.contains("y")) pose.position.y = position["y"].get<double>();
                if (position.contains("z")) pose.position.z = position["z"].get<double>();
            }

            // Parse orientation values (Orientation with roll, pitch, yaw)
            if (jsonValue->contains("orientation") && (*jsonValue)["orientation"].is_object()) {
                const auto& orientation = (*jsonValue)["orientation"];
                if (orientation.contains("roll")) pose.orientation.roll = orientation["roll"].get<double>();
                if (orientation.contains("pitch"))
                    pose.orientation.pitch = orientation["pitch"].get<double>();
                if (orientation.contains("yaw")) pose.orientation.yaw = orientation["yaw"].get<double>();
            }

            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<CRequestMoveBody>(pose, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create CRequestMoveBody: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<CRequestMoveVelocity> CBehaviorParser::createRequestMoveVelocity(const void* value) {
    const json* jsonValue = static_cast<const json*>(value);

    try {
        if (jsonValue->is_object()) {
            geometry_msgs::msg::Twist velocity;

            // Parse linear velocities
            if (jsonValue->contains("linear")) {
                const auto& linear = (*jsonValue)["linear"];
                if (linear.contains("x")) velocity.linear.x = linear["x"].get<double>();
                if (linear.contains("y")) velocity.linear.y = linear["y"].get<double>();
                if (linear.contains("z")) velocity.linear.z = linear["z"].get<double>();
            }

            // Parse angular velocities
            if (jsonValue->contains("angular")) {
                const auto& angular = (*jsonValue)["angular"];
                if (angular.contains("x")) velocity.angular.x = angular["x"].get<double>();
                if (angular.contains("y")) velocity.angular.y = angular["y"].get<double>();
                if (angular.contains("z")) velocity.angular.z = angular["z"].get<double>();
            }

            double minDuration = jsonValue->value("minDuration", 0.0);
            return std::make_shared<CRequestMoveVelocity>(velocity, minDuration);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create CRequestMoveVelocity: %s", e.what());
    }

    return nullptr;
}

}  // namespace brain
