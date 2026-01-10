/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nikita_brain/requester/irequester.hpp"
#include "rclcpp/rclcpp.hpp"

namespace brain {

/**
 * @brief Parser for behaviors.json file that creates Request objects
 * 
 * This class parses a JSON file containing behavior definitions and creates
 * appropriate Request objects (RequestTalking, RequestMusic, CRequestMovementType, etc.)
 */
class CBehaviorParser {
   public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node for logging
     */
    explicit CBehaviorParser(rclcpp::Node::SharedPtr node);

    /**
     * @brief Destructor
     */
    ~CBehaviorParser() = default;

    /**
     * @brief Parse a behaviors.json file
     * @param filePath Path to the JSON file
     * @return true if parsing was successful, false otherwise
     */
    bool parseFile(const std::string& filePath);

    /**
     * @brief Parse a JSON string containing behavior definitions
     * @param jsonString JSON string to parse
     * @return true if parsing was successful, false otherwise
     */
    bool parseString(const std::string& jsonString);

    /**
     * @brief Get the behavior name
     * @return Name of the behavior
     */
    std::string getBehaviorName() const;

    /**
     * @brief Get all action groups (each action group contains multiple requests)
     * @return Vector of action groups, where each group is a vector of Request base pointers
     */
    const std::vector<std::vector<std::shared_ptr<RequestBase>>>& getActionGroups() const;

   private:
    /**
     * @brief Parse actions from JSON
     * @param actionsJson JSON value containing actions array
     * @return true if parsing was successful
     */
    bool parseActions(const void* actionsJson);

    /**
     * @brief Parse a single action group
     * @param actionJson JSON value containing a single action object
     * @return Vector of Request objects for this action group
     */
    std::vector<std::shared_ptr<RequestBase>> parseActionGroup(const void* actionJson);

    /**
     * @brief Create RequestTalking from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestTalking object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestTalking> createRequestTalking(const void* value);

    /**
     * @brief Create RequestChat from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestChat object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestChat> createRequestChat(const void* value);

    /**
     * @brief Create RequestMusic from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestMusic object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestMusic> createRequestMusic(const void* value);

    /**
     * @brief Create RequestListening from JSON value
     * @param value JSON value (boolean or object)
     * @return Shared pointer to RequestListening object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestListening> createRequestListening(const void* value);

    /**
     * @brief Create RequestSystem from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestSystem object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestSystem> createRequestSystem(const void* value);

    /**
     * @brief Create CRequestMovementType from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to CRequestMovementType object, or nullptr if parsing failed
     */
    std::shared_ptr<CRequestMovementType> createRequestMovementType(const void* value);

    /**
     * @brief Create CRequestMoveBody from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to CRequestMoveBody object, or nullptr if parsing failed
     */
    std::shared_ptr<CRequestMoveBody> createRequestMoveBody(const void* value);

    /**
     * @brief Create CRequestMoveVelocity from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to CRequestMoveVelocity object, or nullptr if parsing failed
     */
    std::shared_ptr<CRequestMoveVelocity> createRequestMoveVelocity(const void* value);

    rclcpp::Node::SharedPtr node_;
    std::string behaviorName_;
    std::vector<std::vector<std::shared_ptr<RequestBase>>> actionGroups_;

    // Map of movement type names to their enum values
    static const std::map<std::string, uint32_t> movementTypeNameMap_;
};

}  // namespace brain
