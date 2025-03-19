/*
 * This file is part of VRto3D.
 *
 * VRto3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * VRto3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with VRto3D. If not, see <http://www.gnu.org/licenses/>.
 */

#include "json_manager.h"
#include "driverlog.h"
#include "key_mappings.h"

#include <windows.h>
#include <shlobj.h>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <iomanip>
#include <sstream>

// Include the nlohmann/json library
#include <nlohmann/json.hpp>

JsonManager::JsonManager() {
    vrto3dFolder = getDocumentsFolderPath();
    if (vrto3dFolder != "")
    {
        createFolderIfNotExist(vrto3dFolder);
    }
}


//-----------------------------------------------------------------------------
// Purpose: Get path to user's Documents folder
//-----------------------------------------------------------------------------
std::string JsonManager::getDocumentsFolderPath() {
    PWSTR path = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Documents, 0, NULL, &path);
    if (SUCCEEDED(hr)) {
        char charPath[MAX_PATH];
        size_t convertedChars = 0;
        wcstombs_s(&convertedChars, charPath, MAX_PATH, path, _TRUNCATE);
        CoTaskMemFree(path);
        return std::string(charPath) + "\\My Games\\vrto3d";
    }
    else {
        DriverLog("Failed to get Documents folder path\n");
        return "";
    }
}


//-----------------------------------------------------------------------------
// Purpose: Create vrto3d folder if it doesn't exist
//-----------------------------------------------------------------------------
void JsonManager::createFolderIfNotExist(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
}


//-----------------------------------------------------------------------------
// Purpose: Write a JSON to Documents/My Games/vrto3d
//-----------------------------------------------------------------------------
void JsonManager::writeJsonToFile(const std::string& fileName, const nlohmann::ordered_json& jsonData) {
    std::string filePath = vrto3dFolder + "\\" + fileName;
    std::ofstream file(filePath);
    if (file.is_open()) {
        file << jsonData.dump(4); // Pretty-print the JSON with an indent of 4 spaces
        file.close();
        DriverLog("Saved profile: %s\n", fileName.c_str());
    }
    else {
        DriverLog("Failed to save profile: %s\n", fileName.c_str());
    }
}


//-----------------------------------------------------------------------------
// Purpose: Read a JSON from Documents/My Games/vrto3d
//-----------------------------------------------------------------------------
nlohmann::json JsonManager::readJsonFromFile(const std::string& fileName) {
    std::string filePath = vrto3dFolder + "\\" + fileName;
    std::ifstream file(filePath);
    if (file.is_open()) {
        nlohmann::json jsonData;
        file >> jsonData;
        file.close();
        return jsonData;
    }
    else {
        return {};
    }
}


//-----------------------------------------------------------------------------
// Purpose: Split a string by a delimiter
//-----------------------------------------------------------------------------
std::vector<std::string> JsonManager::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}


//-----------------------------------------------------------------------------
// Purpose: Create default_config.json if it doesn't exist
//-----------------------------------------------------------------------------
void JsonManager::EnsureDefaultConfigExists()
{
    // Check if the file exists
    std::string filePath = vrto3dFolder + "\\" + DEF_CFG;
    if (!std::filesystem::exists(filePath)) {
        DriverLog("%s does not exist. Writing default config to file...\n", DEF_CFG.c_str());

        // Create the example default JSON
        nlohmann::ordered_json defaultConfig = {
            {"window_width", 0},
            {"window_height", 0},
            {"render_width", 1920},
            {"render_height", 1080},
            {"hmd_height", 1.0},
            {"aspect_ratio", 1.77778},
            {"fov", 120.0},
            {"depth", 0.0},
            {"convergence", 0.0},
            {"tab_enable", false},
            {"reverse_enable", false},
            {"depth_gauge", false},
            {"debug_enable", true},
            {"display_latency", 0.011},
            {"display_frequency", 60.0},
        };

        // Write the default JSON to file
        std::ofstream file(filePath);
        if (file.is_open()) {
            file << defaultConfig.dump(4); // Pretty-print with 4 spaces of indentation
            file.close();
            DriverLog("Default config written to %s\n", DEF_CFG.c_str());
        }
        else {
            DriverLog("Failed to open %s for writing\n", DEF_CFG.c_str());
        }
    }
    else {
        DriverLog("Default config already exists\n");
    }
}


//-----------------------------------------------------------------------------
// Purpose: Load the VRto3D display from a JSON file
//-----------------------------------------------------------------------------
void JsonManager::LoadParamsFromJson(StereoDisplayDriverConfiguration& config)
{
    // Read the JSON configuration from the file
    nlohmann::json jsonConfig = readJsonFromFile(DEF_CFG);
    
    try {
        // Load values directly from the base level of the JSON
        config.window_width = jsonConfig.at("window_width").get<int>();
        config.window_height = jsonConfig.at("window_height").get<int>();
        config.render_width = jsonConfig.at("render_width").get<int>();
        config.render_height = jsonConfig.at("render_height").get<int>();

        config.aspect_ratio = jsonConfig.at("aspect_ratio").get<float>();
        config.fov = jsonConfig.at("fov").get<float>();

        config.debug_enable = jsonConfig.at("debug_enable").get<bool>();
        config.tab_enable = jsonConfig.at("tab_enable").get<bool>();
        config.reverse_enable = jsonConfig.at("reverse_enable").get<bool>();
        config.depth_gauge = jsonConfig.at("depth_gauge").get<bool>();

        config.display_latency = jsonConfig.at("display_latency").get<float>();
        config.display_frequency = jsonConfig.at("display_frequency").get<float>();
        config.sleep_count_max = (int)(floor(1600.0 / (1000.0 / config.display_frequency)));
    }
    catch (const nlohmann::json::exception& e) {
        DriverLog("Error reading default_config.json: %s\n", e.what());
    }
}


//-----------------------------------------------------------------------------
// Purpose: Load a VRto3D profile from a JSON file
//-----------------------------------------------------------------------------
bool JsonManager::LoadProfileFromJson(const std::string& filename, StereoDisplayDriverConfiguration& config)
{
    // Read the JSON configuration from the file
    nlohmann::json jsonConfig = readJsonFromFile(filename);

    if (jsonConfig.is_null() && filename != DEF_CFG) {
        DriverLog("No profile found for %s\n", filename.c_str());
        return false;
    }

    try {
        // Profile settings
        config.hmd_height = jsonConfig.at("hmd_height").get<float>();
        config.depth = jsonConfig.at("depth").get<float>();
        config.convergence = jsonConfig.at("convergence").get<float>();
    }
    catch (const nlohmann::json::exception& e) {
        DriverLog("Error reading config from %s: %s\n", filename.c_str(), e.what());
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------
// Purpose: Save Game Specific Settings to Documents\My games\vrto3d\app_name_config.json
//-----------------------------------------------------------------------------
void JsonManager::SaveProfileToJson(const std::string& filename, StereoDisplayDriverConfiguration& config)
{
    // Create a JSON object to hold all the configuration data
    nlohmann::ordered_json jsonConfig;

    // Populate the JSON object with settings
    jsonConfig["hmd_height"] = config.hmd_height;
    jsonConfig["depth"] = config.depth;
    jsonConfig["convergence"] = config.convergence;
    writeJsonToFile(filename, jsonConfig);
}
