#ifndef AMR_MISSION_CONTROLLER__LOCATION_ACTION_UTILS_HPP_
#define AMR_MISSION_CONTROLLER__LOCATION_ACTION_UTILS_HPP_

#include <string>

#include "amr_mission_controller/graph_types.hpp"

inline std::string locationActionToString(LocationAction action)
{
    switch (action)
    {
        case LocationAction::StartLineTracking:
            return "StartLineTracking";

        case LocationAction::OpenDoor:
            return "OpenDoor";

        case LocationAction::CloseDoor:
            return "CloseDoor";

        case LocationAction::LiftUp:
            return "LiftUp";

        case LocationAction::LiftDown:
            return "LiftDown";

        case LocationAction::None:
        default:
            return "None";
    }
}

inline bool locationActionFromString(
    const std::string &name,
    LocationAction &action)
{
    if (name == "None")
        action = LocationAction::None;

    else if (name == "StartLineTracking")
        action = LocationAction::StartLineTracking;

    else if (name == "OpenDoor")
        action = LocationAction::OpenDoor;

    else if (name == "CloseDoor")
        action = LocationAction::CloseDoor;

    else if (name == "LiftUp")
        action = LocationAction::LiftUp;

    else if (name == "LiftDown")
        action = LocationAction::LiftDown;

    else
        return false;

    return true;
}

#endif
