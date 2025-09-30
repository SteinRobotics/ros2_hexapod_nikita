/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/requests.hpp"

class CActionPackage {
   public:
    std::optional<CHead> head;
    std::optional<CPose> body;
    std::optional<std::map<ELegIndex, CLeg>> legs;
    double factorDuration = 1.0;
};

class CActionPackagesParser {
   public:
    CActionPackagesParser(std::shared_ptr<rclcpp::Node> node);
    virtual ~CActionPackagesParser() = default;

    std::vector<CActionPackage>& getRequests(const std::string& packageName);

   private:
    void readJson();
    void parseStep(const nlohmann::json& step, std::vector<CActionPackage>& actionPackage);

    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::vector<CActionPackage>> actionPackages_;
};
