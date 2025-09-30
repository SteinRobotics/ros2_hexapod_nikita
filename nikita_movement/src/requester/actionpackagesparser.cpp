/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/actionpackagesparser.hpp"

using json = nlohmann::json;

CActionPackagesParser::CActionPackagesParser(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    readJson();
}

void CActionPackagesParser::readJson() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nikita_movement");
    std::string file_path = package_share_directory + "/config/actionpackages.json";
    json json_data;
    std::ifstream json_file(file_path);

    if (!json_file.is_open()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to open file: " << file_path);
        return;
    }

    try {
        json_file >> json_data;
        json_file.close();
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error parsing : " << file_path << e.what());
        return;
    }

    for (const auto& [key, value] : json_data.items()) {
        std::vector<CActionPackage> actionPackage;
        for (const auto& step : value) {
            parseStep(step, actionPackage);
        }
        actionPackages_[key] = actionPackage;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << actionPackages_.size() << " action packages.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "keys:");
    for (const auto& [key, _] : actionPackages_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
}

void CActionPackagesParser::parseStep(const json& step, std::vector<CActionPackage>& actionPackage) {
    CActionPackage action;

    // Parse "duration"
    if (step.contains("factorDuration")) {
        action.factorDuration = step["factorDuration"];
    } else {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "CActionPackagesParser::parseStep: 'factorDuration' not found, defaulting to 1.0");
        action.factorDuration = 1.0;
    }

    // Parse "head"
    if (step.contains("head")) {
        double yaw = 0.0, pitch = 0.0;
        for (const auto& headEntry : step["head"]) {
            if (headEntry.contains("yaw")) yaw = headEntry["yaw"];
            if (headEntry.contains("pitch")) pitch = headEntry["pitch"];
        }
        action.head = CHead(yaw, pitch);
    }

    // Parse "body"
    if (step.contains("body")) {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double x = 0.0, y = 0.0, z = 0.0;
        for (const auto& bodyEntry : step["body"]) {
            if (bodyEntry.contains("orientation")) {
                for (const auto& orientationEntry : bodyEntry["orientation"]) {
                    if (orientationEntry.contains("roll")) roll = orientationEntry["roll"];
                    if (orientationEntry.contains("pitch")) pitch = orientationEntry["pitch"];
                    if (orientationEntry.contains("yaw")) yaw = orientationEntry["yaw"];
                }
            }
            if (bodyEntry.contains("direction")) {
                for (const auto& directionEntry : bodyEntry["direction"]) {
                    if (directionEntry.contains("x")) x = directionEntry["x"];
                    if (directionEntry.contains("y")) y = directionEntry["y"];
                    if (directionEntry.contains("z")) z = directionEntry["z"];
                }
            }
        }
        action.body = CPose(x, y, z, roll, pitch, yaw);
    }

    // Parse "legs"
    if (step.contains("legs")) {
        const std::map<std::string, ELegIndex> legNameToIndex = {
            {"RightFront", ELegIndex::RightFront}, {"RightMid", ELegIndex::RightMid},
            {"RightBack", ELegIndex::RightBack},   {"LeftFront", ELegIndex::LeftFront},
            {"LeftMid", ELegIndex::LeftMid},       {"LeftBack", ELegIndex::LeftBack},
        };

        std::map<ELegIndex, CLeg> legsMap;
        for (const auto& leg : step["legs"]) {
            for (auto it = leg.begin(); it != leg.end(); ++it) {
                std::string legName = it.key();
                if (legNameToIndex.count(legName)) {
                    double coxa = 0.0, femur = 0.0, tibia = 0.0;
                    for (const auto& joint : it.value()) {
                        if (joint.contains("coxa")) coxa = joint["coxa"];
                        if (joint.contains("femur")) femur = joint["femur"];
                        if (joint.contains("tibia")) tibia = joint["tibia"];
                    }
                    legsMap[legNameToIndex.at(legName)] = CLeg(CLegAngles(coxa, femur, tibia), CPosition());
                }
            }
        }
        if (!legsMap.empty()) {
            action.legs = legsMap;
        }
    }

    // Add the filled action to the vector
    actionPackage.push_back(action);
}

std::vector<CActionPackage>& CActionPackagesParser::getRequests(const std::string& packageName) {
    if (actionPackages_.find(packageName) != actionPackages_.end()) {
        return actionPackages_.at(packageName);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Action package not found: " << packageName);
        static std::vector<CActionPackage> empty_vector;
        return empty_vector;
    }
}