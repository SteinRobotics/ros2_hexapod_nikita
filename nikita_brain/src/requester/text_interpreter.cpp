/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/text_interpreter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

using json = nlohmann::json;

namespace brain {

CTextInterpreter::CTextInterpreter(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    readInterpretation();
}

void CTextInterpreter::readInterpretation() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nikita_brain");
    std::string file_path = package_share_directory + "/config/interpretation.json";
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

    std::unordered_map<std::string, json> interpretationJson;

    for (const auto& [key, value] : json_data.items()) {
        interpretationJson[key] = value;
    }

    // copy the content of interpretationJson as CWord to vocabulary_
    uint32_t index = 0;
    for (const auto& [key, value] : interpretationJson) {
        interpretations_[key] = CInterpretation();
        for (const auto& [wordType, values] : value.items()) {
            std::vector<uint32_t> indicesForWordType;
            for (const auto& value : values) {
                if (vocabulary_.find(value) != vocabulary_.end()) {
                    indicesForWordType.push_back(vocabulary_[value].index);
                    continue;
                }
                vocabulary_[value] = CWord();
                vocabulary_[value].value = value;
                vocabulary_[value].index = ++index;
                vocabulary_[value].type = wordType;

                indicesForWordType.push_back(index);
            }
            interpretations_[key].wordType2WordIndices[wordType] = indicesForWordType;
        }
    }
    // for debugging:
    // RCLCPP_INFO_STREAM(node_->get_logger(), "*** vocabulary_: ***");
    // for (const auto& [key, value] : vocabulary_) {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), value.value << ", index: " << value.index << ", type: "
    //                                                         << value.type);
    // }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "*** interpretations_: ***");
    // for (const auto& [key, value] : interpretations_) {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "key: " << key);
    //     for (const auto& [wordType, indices] : value.wordType2WordIndices) {
    //         RCLCPP_INFO_STREAM(node_->get_logger(), "wordType: " << wordType << ", indices: ");
    //         for (const auto& index : indices) {
    //             RCLCPP_INFO_STREAM(node_->get_logger(), index);
    //         }
    //     }
    // }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "-------------------------------------------------------");
}

std::vector<CWord> CTextInterpreter::parseText(std::string& text) {
    std::istringstream iss(text);
    std::vector<CWord> identifiedWords;
    do {
        std::string singleWord;
        iss >> singleWord;

        if (singleWord.empty()) continue;

        //     // check word is numeric
        //     if (std::isdigit(firstCharacter) || firstCharacter == '-') {
        //         // convert to int or float??
        //         word_t number = {wordHeard, WordIdentifier::numeric, WordType::numeric};
        //         detectedValues.push_back(number);
        //         continue;
        //     }

        // convert to lowercase
        // TODO: maybe it is fine to just transform the first letter to lowercase
        std::transform(singleWord.begin(), singleWord.end(), singleWord.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        //     // check last char is an e, remove it
        //     // because of german geh(e), sag(e), ...
        //     if (wordHeard.back() == 'e') {
        //         wordHeard.pop_back();
        //     }

        // check if the word is in the vocabulary_
        if (vocabulary_.find(singleWord) == vocabulary_.end()) {
            RCLCPP_INFO_STREAM(node_->get_logger(), "word not found in vocabulary_: " << singleWord);
            continue;
        }
        identifiedWords.push_back(vocabulary_[singleWord]);

    } while (iss);

    // For debugging:
    // RCLCPP_INFO_STREAM(node_->get_logger(), "identifiedWords: ");
    // for (const auto& word : identifiedWords) {
    //     RCLCPP_INFO_STREAM(node_->get_logger(),
    //                        "word: " << word.value << ", index: " << word.index << ", type: " << word.type);
    // }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "-------------------------------------------------------");

    return identifiedWords;
}

std::string CTextInterpreter::searchInterpretation(const std::vector<CWord>& identifiedWords) {
    // search the identifiedWords in interpretations_ and look up the command
    // for each word type at least one word is found
    for (const auto& [key, interpretation] : interpretations_) {
        // RCLCPP_INFO_STREAM(node_->get_logger(), "compare incoming words with interpretation: " << key);

        uint32_t numberOfWordTypesFound = 0;
        for (const auto& [wordType, indices] : interpretation.wordType2WordIndices) {
            for (const auto& word : identifiedWords) {
                if (word.type != wordType) {
                    continue;
                }
                if (isIndexIn(word.index, indices)) {
                    RCLCPP_INFO_STREAM(node_->get_logger(),
                                       "found word: " << word.value << " for wordType: " << wordType);
                    numberOfWordTypesFound++;
                }
                //  else {
                //     RCLCPP_INFO_STREAM(node_->get_logger(),
                //                        "word: " << word.value << " not found for wordType: " << wordType
                //                                 << " with index: " << word.index);
                // }
            }
            if (numberOfWordTypesFound == interpretation.wordType2WordIndices.size()) {
                return key;
            }
        }
    }
    return "notFound";
}

bool CTextInterpreter::isIndexIn(const uint32_t index, const std::vector<uint32_t>& list) {
    auto it = std::find(list.begin(), list.end(), index);
    return (it != list.end());
}

CWord CTextInterpreter::letters2Word(const std::string& letters) {
    if (vocabulary_.find(letters) != vocabulary_.end()) {
        return vocabulary_[letters];
    }
    return CWord();
}

bool CTextInterpreter::isWordIn(const CWord& word, const std::vector<CWord>& words) {
    return std::any_of(words.begin(), words.end(), [&word](const CWord& w) { return w.index == word.index; });
}

bool CTextInterpreter::lettersIdentified(const std::string& letters, const std::vector<CWord>& words) {
    CWord word = letters2Word(letters);
    if (word.value.empty()) {
        return false;
    }
    return isWordIn(word, words);
}

/// helper functions to check if a string is an integer, float or german float
bool CTextInterpreter::isInt(const std::string& str) {
    return str.find_first_not_of("-0123456789") == std::string::npos;
}

bool CTextInterpreter::isFloat(const std::string& str) {
    return str.find_first_not_of("-0123456789.") == std::string::npos;
}

bool CTextInterpreter::isGermanFloat(const std::string& str) {
    return str.find_first_not_of("-0123456789,") == std::string::npos;
}

}  // namespace brain
