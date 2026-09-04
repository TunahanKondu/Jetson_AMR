#ifndef AMR_MISSION_CONTROLLER__STATION_TYPES_HPP_
#define AMR_MISSION_CONTROLLER__STATION_TYPES_HPP_

#include <string>

#include "amr_mission_controller/graph_types.hpp"


struct StationInfo
{
    std::string qrCode;

    std::string stationName;

    std::string stationType;

    double x = 0.0;

    double y = 0.0;

    double yaw = 0.0;

    LocationAction arrivalAction =
        LocationAction::None;

    int linkedWaypointId = -1;
};


#endif
