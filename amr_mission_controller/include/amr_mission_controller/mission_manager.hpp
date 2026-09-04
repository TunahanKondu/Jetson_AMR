#ifndef AMR_MISSION_CONTROLLER__MISSION_MANAGER_HPP_
#define AMR_MISSION_CONTROLLER__MISSION_MANAGER_HPP_

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "amr_mission_controller/navigation_manager.hpp"


enum class MissionState
{
    NoMission,
    PendingApproval,
    Approved,
    Running,
    Paused,
    Completed,
    Cancelled,
    MissionError
};


enum class MissionStage
{
    None,

    NavigatingToPickup,
    PickupLineFollowing,
    Lifting,

    NavigatingToDropoff,
    DropoffLineFollowing,
    Lowering
};


struct MissionData
{
    std::string missionId;

    std::string pickupStation;

    std::string dropoffStation;
};


class MissionManager
{
public:

    using NavigateToStationHandler =
        std::function<bool(
            const std::string &stationName)>;


    using RobotActionRequestHandler =
        std::function<void(
            const std::string &action)>;


    using MotionStopHandler =
        std::function<void()>;


    MissionManager(
        NavigationManager *navigationManager,
        rclcpp::Node *node,
        NavigateToStationHandler navigateToStationHandler,
        RobotActionRequestHandler robotActionRequestHandler,
        MotionStopHandler motionStopHandler);


    bool createManualMission(
        const std::string &pickupStation,
        const std::string &dropoffStation);


    bool approveMission();

    void startMission();

    void pauseMission();

    void resumeMission();

    void cancelMission();


    void setEmergencyStopActive(
        bool active);


    void handleFinalStationReached(
        const std::string &stationName);


    void handleNavigationFailed(
        const std::string &reason);


    void handleRobotActionResult(
        const std::string &action);


    MissionState missionState() const;

    MissionStage missionStage() const;


    std::string missionId() const;

    std::string pickupStation() const;

    std::string dropoffStation() const;

    std::string activeOperation() const;

    std::string missionStatusText() const;


private:

    void setMissionState(
        MissionState state);


    void setActiveOperation(
        const std::string &operation);


    bool canCreateMission() const;


    NavigationManager *navigation_manager_ =
        nullptr;


    rclcpp::Node *node_ =
        nullptr;


    NavigateToStationHandler
        navigate_to_station_handler_;


    RobotActionRequestHandler
        robot_action_request_handler_;


    MotionStopHandler
        motion_stop_handler_;


    MissionData mission_;


    MissionState mission_state_ =
        MissionState::NoMission;


    MissionStage mission_stage_ =
        MissionStage::None;


    std::string active_operation_ =
        "Idle";


    bool emergency_stop_active_ =
        false;


    int next_mission_number_ =
        1;
};


#endif
