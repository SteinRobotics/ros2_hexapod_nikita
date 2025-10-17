/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/actionpackagesparser.hpp"

#include <yaml-cpp/yaml.h>

CActionPackagesParser::CActionPackagesParser(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    readYaml();
}

void CActionPackagesParser::readYaml() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nikita_movement");
    std::string yaml_path = package_share_directory + "/config/actionpackages.yaml";

    try {
        YAML::Node yaml_data = YAML::LoadFile(yaml_path);

        for (const auto& it : yaml_data) {
            std::string key = it.first.as<std::string>();
            // Only parse top-level entries that are sequences (action package lists).
            // Skip entries like 'presets' which are mappings and not action sequences.
            if (!it.second.IsSequence()) {
                RCLCPP_DEBUG_STREAM(node_->get_logger(), "Skipping non-sequence top-level key: " << key);
                continue;
            }
            std::vector<CActionPackage> actionPackage;
            for (const auto& step : it.second) {
                // each step should be a map describing head/body/legs/footPositions
                if (!step.IsMap()) {
                    RCLCPP_WARN_STREAM(node_->get_logger(),
                                       "Skipping invalid step (not a map) in package: " << key);
                    continue;
                }
                parseYamlStep(step, actionPackage);
            }
            actionPackages_[key] = actionPackage;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error parsing YAML: " << yaml_path << " : " << e.what());
        return;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << actionPackages_.size() << " action packages.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "keys:");
    for (const auto& [key, _] : actionPackages_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
}

void CActionPackagesParser::parseYamlStep(const YAML::Node& step,
                                          std::vector<CActionPackage>& actionPackage) {
    CActionPackage action;

    if (step["factorDuration"]) {
        action.factorDuration = step["factorDuration"].as<double>();
    } else {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "CActionPackagesParser::parseYamlStep: 'factorDuration' not found, defaulting to 1.0");
        action.factorDuration = 1.0;
    }

    if (step["head"]) {
        double yaw = 0.0, pitch = 0.0;
        YAML::Node headNode = step["head"];
        if (headNode.IsSequence()) {
            for (const auto& headEntry : headNode) {
                if (headEntry["yaw"]) yaw = headEntry["yaw"].as<double>();
                if (headEntry["pitch"]) pitch = headEntry["pitch"].as<double>();
            }
        } else if (headNode.IsMap()) {
            if (headNode["yaw"]) yaw = headNode["yaw"].as<double>();
            if (headNode["pitch"]) pitch = headNode["pitch"].as<double>();
        }
        action.head = CHead(yaw, pitch);
    }

    if (step["body"]) {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double x = 0.0, y = 0.0, z = 0.0;
        YAML::Node bodyNode = step["body"];
        // body can be a sequence of maps (current style) or a single map
        std::vector<YAML::Node> bodyEntries;
        if (bodyNode.IsSequence()) {
            for (const auto& n : bodyNode) bodyEntries.push_back(n);
        } else if (bodyNode.IsMap()) {
            bodyEntries.push_back(bodyNode);
        }

        for (const auto& bodyEntry : bodyEntries) {
            // orientation may be a sequence of single-key maps or a single map
            if (bodyEntry["orientation"]) {
                YAML::Node orientNode = bodyEntry["orientation"];
                if (orientNode.IsSequence()) {
                    for (const auto& orientationEntry : orientNode) {
                        if (orientationEntry["roll"]) roll = orientationEntry["roll"].as<double>();
                        if (orientationEntry["pitch"]) pitch = orientationEntry["pitch"].as<double>();
                        if (orientationEntry["yaw"]) yaw = orientationEntry["yaw"].as<double>();
                    }
                } else if (orientNode.IsMap()) {
                    if (orientNode["roll"]) roll = orientNode["roll"].as<double>();
                    if (orientNode["pitch"]) pitch = orientNode["pitch"].as<double>();
                    if (orientNode["yaw"]) yaw = orientNode["yaw"].as<double>();
                }
            }
            if (bodyEntry["direction"]) {
                YAML::Node dirNode = bodyEntry["direction"];
                if (dirNode.IsSequence()) {
                    for (const auto& directionEntry : dirNode) {
                        if (directionEntry["x"]) x = directionEntry["x"].as<double>();
                        if (directionEntry["y"]) y = directionEntry["y"].as<double>();
                        if (directionEntry["z"]) z = directionEntry["z"].as<double>();
                    }
                } else if (dirNode.IsMap()) {
                    if (dirNode["x"]) x = dirNode["x"].as<double>();
                    if (dirNode["y"]) y = dirNode["y"].as<double>();
                    if (dirNode["z"]) z = dirNode["z"].as<double>();
                }
            }
        }
        action.body = CPose(x, y, z, roll, pitch, yaw);
    }

    if (step["legs"]) {
        std::map<ELegIndex, CLegAngles> legsMap;

        // Recursive processor to handle maps, sequences, and YAML merge key '<<'
        std::function<void(const YAML::Node&)> processLegsNode;
        processLegsNode = [&](const YAML::Node& node) {
            if (node.IsNull()) return;
            if (node.IsSequence()) {
                for (const auto& elem : node) {
                    processLegsNode(elem);
                }
                return;
            }
            if (node.IsMap()) {
                for (const auto& kv : node) {
                    std::string key = kv.first.as<std::string>();
                    const YAML::Node& val = kv.second;

                    if (key == "<<") {
                        // YAML merge key: recursively process merged content
                        processLegsNode(val);
                        continue;
                    }

                    if (key == "All") {
                        double coxa = val["coxa"] ? val["coxa"].as<double>() : 0.0;
                        double femur = val["femur"] ? val["femur"].as<double>() : 0.0;
                        double tibia = val["tibia"] ? val["tibia"].as<double>() : 0.0;
                        for (const auto& [name, idx] : legNameToIndex) {
                            legsMap[idx] = CLegAngles(coxa, femur, tibia);
                        }
                    } else if (legNameToIndex.count(key)) {
                        double coxa = val["coxa"] ? val["coxa"].as<double>() : 0.0;
                        double femur = val["femur"] ? val["femur"].as<double>() : 0.0;
                        double tibia = val["tibia"] ? val["tibia"].as<double>() : 0.0;
                        legsMap[legNameToIndex.at(key)] = CLegAngles(coxa, femur, tibia);
                    } else {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in legs map: " << key);
                    }
                }
                return;
            }
        };

        processLegsNode(step["legs"]);
        if (!legsMap.empty()) {
            action.legAngles = legsMap;
        }
    }

    if (step["footPositions"]) {
        std::map<ELegIndex, CPosition> posMap;

        std::function<void(const YAML::Node&)> processPosNode;
        processPosNode = [&](const YAML::Node& node) {
            if (node.IsNull()) return;
            if (node.IsSequence()) {
                for (const auto& elem : node) processPosNode(elem);
                return;
            }
            if (node.IsMap()) {
                for (const auto& kv : node) {
                    std::string key = kv.first.as<std::string>();
                    const YAML::Node& val = kv.second;
                    if (key == "<<") {
                        processPosNode(val);
                        continue;
                    }
                    if (key == "All") {
                        double x = val["x"] ? val["x"].as<double>() : 0.0;
                        double y = val["y"] ? val["y"].as<double>() : 0.0;
                        double z = val["z"] ? val["z"].as<double>() : 0.0;
                        for (const auto& [name, idx] : legNameToIndex) {
                            posMap[idx] = CPosition(x, y, z);
                        }
                    } else if (legNameToIndex.count(key)) {
                        double x = val["x"] ? val["x"].as<double>() : 0.0;
                        double y = val["y"] ? val["y"].as<double>() : 0.0;
                        double z = val["z"] ? val["z"].as<double>() : 0.0;
                        posMap[legNameToIndex.at(key)] = CPosition(x, y, z);
                    } else {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in footPositions: " << key);
                    }
                }
                return;
            }
        };

        processPosNode(step["footPositions"]);
        if (!posMap.empty()) {
            action.footPositions = posMap;
        }
    }

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