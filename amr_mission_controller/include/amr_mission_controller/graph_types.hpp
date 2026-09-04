#ifndef AMR_MISSION_CONTROLLER__GRAPH_TYPES_HPP_
#define AMR_MISSION_CONTROLLER__GRAPH_TYPES_HPP_

#include <string>

enum class LocationAction
{
    None,
    StartLineTracking,
    OpenDoor,
    CloseDoor,
    LiftUp,
    LiftDown
};

struct GraphNode
{
    int id = 0;
    std::string name;
    double x = 0.0;
    double y = 0.0;
    LocationAction arrivalAction = LocationAction::None;
};

struct GraphEdge
{
    int startNodeId = 0;
    int endNodeId = 0;
};

#endif
