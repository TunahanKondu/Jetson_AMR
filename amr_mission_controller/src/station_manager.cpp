#include "amr_mission_controller/station_manager.hpp"

#include "amr_mission_controller/location_action_utils.hpp"

#include <cmath>
#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>


bool StationManager::loadStations(
    const std::string &filePath)
{
    std::ifstream file(
        filePath);


    if (!file.is_open()) {
        return false;
    }


    nlohmann::json document;


    try
    {
        file >> document;
    }
    catch (
        const nlohmann::json::exception &)
    {
        return false;
    }


    if (!document.is_array()) {
        return false;
    }


    std::vector<StationInfo>
        loadedStations;


    loadedStations.reserve(
        document.size());


    for (const auto &object :
         document)
    {
        if (!object.is_object()) {
            return false;
        }


        // ----------------------------------------------------
        // Required fields
        // ----------------------------------------------------

        if (!object.contains("qrCode") ||
            !object["qrCode"].is_string())
        {
            return false;
        }


        if (!object.contains("stationName") ||
            !object["stationName"].is_string())
        {
            return false;
        }


        if (!object.contains("stationType") ||
            !object["stationType"].is_string())
        {
            return false;
        }


        if (!object.contains("x") ||
            !object["x"].is_number())
        {
            return false;
        }


        if (!object.contains("y") ||
            !object["y"].is_number())
        {
            return false;
        }


        if (!object.contains("yaw") ||
            !object["yaw"].is_number())
        {
            return false;
        }


        if (!object.contains(
                "linkedWaypointId") ||
            !object["linkedWaypointId"]
                 .is_number_integer())
        {
            return false;
        }


        StationInfo station;


        station.qrCode =
            object["qrCode"]
                .get<std::string>();


        station.stationName =
            object["stationName"]
                .get<std::string>();


        station.stationType =
            object["stationType"]
                .get<std::string>();


        station.x =
            object["x"]
                .get<double>();


        station.y =
            object["y"]
                .get<double>();


        station.yaw =
            object["yaw"]
                .get<double>();


        station.linkedWaypointId =
            object["linkedWaypointId"]
                .get<int>();


        // ----------------------------------------------------
        // Arrival action
        // ----------------------------------------------------

        std::string arrivalActionName =
            "None";


        if (object.contains(
                "arrivalAction"))
        {
            if (!object["arrivalAction"]
                     .is_string())
            {
                return false;
            }


            arrivalActionName =
                object["arrivalAction"]
                    .get<std::string>();
        }


        if (!locationActionFromString(
                arrivalActionName,
                station.arrivalAction))
        {
            return false;
        }


        // ----------------------------------------------------
        // Validate coordinates
        // ----------------------------------------------------

        if (!std::isfinite(station.x) ||
            !std::isfinite(station.y) ||
            !std::isfinite(station.yaw))
        {
            return false;
        }


        loadedStations.push_back(
            std::move(station));
    }


    // Only replace the active stations
    // after the complete JSON passed validation.

    stations_ =
        std::move(
            loadedStations);


    return true;
}


const StationInfo *
StationManager::findStationByName(
    const std::string &stationName) const
{
    for (const auto &station :
         stations_)
    {
        if (station.stationName ==
            stationName)
        {
            return &station;
        }
    }


    return nullptr;
}


const StationInfo *
StationManager::findStationByQrCode(
    const std::string &qrCode) const
{
    for (const auto &station :
         stations_)
    {
        if (station.qrCode ==
            qrCode)
        {
            return &station;
        }
    }


    return nullptr;
}


const std::vector<StationInfo> &
StationManager::stations() const
{
    return stations_;
}
