#include "amr_mission_controller/mission_manager.hpp"

#include <utility>


MissionManager::MissionManager(
    NavigationManager *navigationManager,
    rclcpp::Node *node,
    NavigateToStationHandler navigateToStationHandler,
    RobotActionRequestHandler robotActionRequestHandler,
    MotionStopHandler motionStopHandler)
    :
    navigation_manager_(
        navigationManager),

    node_(
        node),

    navigate_to_station_handler_(
        std::move(
            navigateToStationHandler)),

    robot_action_request_handler_(
        std::move(
            robotActionRequestHandler)),

    motion_stop_handler_(
        std::move(
            motionStopHandler))
{
}


bool MissionManager::canCreateMission() const
{
    if (emergency_stop_active_)
    {
        return false;
    }


    if (navigation_manager_ &&
        navigation_manager_->
            navigationActive())
    {
        return false;
    }


    return
        mission_state_ ==
            MissionState::NoMission ||

        mission_state_ ==
            MissionState::Completed ||

        mission_state_ ==
            MissionState::Cancelled ||

        mission_state_ ==
            MissionState::MissionError;
}


bool MissionManager::createManualMission(
    const std::string &pickupStation,
    const std::string &dropoffStation)
{
    if (!canCreateMission())
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Cannot create mission in current state");

        return false;
    }


    if (pickupStation.empty() ||
        dropoffStation.empty())
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Pickup or dropoff station is empty");

        return false;
    }


    // Pickup stations are A...
    if (pickupStation.front() != 'A')
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid pickup station: %s",
            pickupStation.c_str());

        return false;
    }


    // Drop-off stations are B...
    if (dropoffStation.front() != 'B')
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid dropoff station: %s",
            dropoffStation.c_str());

        return false;
    }


    mission_ =
        MissionData{};


    mission_.missionId =
        "#" +
        std::to_string(
            next_mission_number_++);


    mission_.pickupStation =
        pickupStation;


    mission_.dropoffStation =
        dropoffStation;


    mission_stage_ =
        MissionStage::None;


    setMissionState(
        MissionState::
            PendingApproval);


    setActiveOperation(
        "Mission created");


    RCLCPP_INFO(
        node_->get_logger(),
        "Mission %s created: %s -> %s",
        mission_.missionId.c_str(),
        mission_.pickupStation.c_str(),
        mission_.dropoffStation.c_str());


    return true;
}


bool MissionManager::approveMission()
{
    if (emergency_stop_active_ ||
        mission_state_ !=
            MissionState::PendingApproval)
    {
        return false;
    }


    setMissionState(
        MissionState::Approved);


    setActiveOperation(
        "Mission approved");


    RCLCPP_INFO(
        node_->get_logger(),
        "Mission %s approved",
        mission_.missionId.c_str());


    return true;
}


void MissionManager::startMission()
{
    if (emergency_stop_active_ ||
        mission_state_ !=
            MissionState::Approved)
    {
        return;
    }


    mission_stage_ =
        MissionStage::
            NavigatingToPickup;


    setMissionState(
        MissionState::Running);


    RCLCPP_INFO(
        node_->get_logger(),
        "Mission %s starting",
        mission_.missionId.c_str());


    if (!navigate_to_station_handler_ ||
        !navigate_to_station_handler_(
            mission_.pickupStation))
    {
        mission_stage_ =
            MissionStage::None;


        setActiveOperation(
            "Navigation to pickup failed");


        setMissionState(
            MissionState::
                MissionError);


        return;
    }


    setActiveOperation(
        "Navigating to " +
        mission_.pickupStation);
}


void MissionManager::
handleFinalStationReached(
    const std::string &stationName)
{
    if (emergency_stop_active_ ||
        mission_state_ !=
            MissionState::Running)
    {
        return;
    }


    // ============================================================
    // PICKUP STATION REACHED
    // ============================================================

    if (mission_stage_ ==
            MissionStage::
                NavigatingToPickup &&

        stationName ==
            mission_.pickupStation)
    {
        mission_stage_ =
            MissionStage::
                PickupLineFollowing;


        setActiveOperation(
            "Pickup line following");


        RCLCPP_INFO(
            node_->get_logger(),
            "Pickup station %s reached. Starting line following.",
            stationName.c_str());


        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_START");
        }


        return;
    }


    // ============================================================
    // DROPOFF STATION REACHED
    // ============================================================

    if (mission_stage_ ==
            MissionStage::
                NavigatingToDropoff &&

        stationName ==
            mission_.dropoffStation)
    {
        mission_stage_ =
            MissionStage::
                DropoffLineFollowing;


        setActiveOperation(
            "Dropoff line following");


        RCLCPP_INFO(
            node_->get_logger(),
            "Dropoff station %s reached. Starting line following.",
            stationName.c_str());


        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_START");
        }


        return;
    }
}


void MissionManager::
handleRobotActionResult(
    const std::string &action)
{
    if (emergency_stop_active_ ||
        mission_state_ !=
            MissionState::Running ||
        mission_.missionId.empty())
    {
        return;
    }


    // ============================================================
    // PICKUP LINE COMPLETE
    // ============================================================

    if (action == "LINE_COMPLETE" &&
        mission_stage_ ==
            MissionStage::
                PickupLineFollowing)
    {
        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_STOP");
        }


        mission_stage_ =
            MissionStage::Lifting;


        setActiveOperation(
            "Lifting load");


        RCLCPP_INFO(
            node_->get_logger(),
            "Pickup line completed. Lifting load.");


        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LIFT_UP");
        }


        return;
    }


    // ============================================================
    // LIFT UP COMPLETE
    //
    // NO REVERSE MODE.
    // GO DIRECTLY TO DROPOFF.
    // ============================================================

    if (action == "LIFT_UP_COMPLETE" &&
        mission_stage_ ==
            MissionStage::Lifting)
    {
        mission_stage_ =
            MissionStage::
                NavigatingToDropoff;


        setActiveOperation(
            "Load lifted");


        RCLCPP_INFO(
            node_->get_logger(),
            "Lift complete. Navigating directly to %s",
            mission_.dropoffStation.c_str());


        if (!navigate_to_station_handler_ ||
            !navigate_to_station_handler_(
                mission_.dropoffStation))
        {
            mission_stage_ =
                MissionStage::None;


            setActiveOperation(
                "Navigation to dropoff failed");


            setMissionState(
                MissionState::
                    MissionError);


            return;
        }


        setActiveOperation(
            "Navigating to " +
            mission_.dropoffStation);


        return;
    }


    // ============================================================
    // DROPOFF LINE COMPLETE
    // ============================================================

    if (action == "LINE_COMPLETE" &&
        mission_stage_ ==
            MissionStage::
                DropoffLineFollowing)
    {
        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_STOP");
        }


        mission_stage_ =
            MissionStage::Lowering;


        setActiveOperation(
            "Lowering load");


        RCLCPP_INFO(
            node_->get_logger(),
            "Dropoff line completed. Lowering load.");


        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LIFT_DOWN");
        }


        return;
    }


    // ============================================================
    // LIFT DOWN COMPLETE
    // ============================================================

    if (action == "LIFT_DOWN_COMPLETE" &&
        mission_stage_ ==
            MissionStage::Lowering)
    {
        mission_stage_ =
            MissionStage::None;


        setActiveOperation(
            "Mission completed");


        setMissionState(
            MissionState::Completed);


        RCLCPP_INFO(
            node_->get_logger(),
            "Mission %s completed",
            mission_.missionId.c_str());


        return;
    }


    // ============================================================
    // ACTION ERROR
    // ============================================================

    if (action == "ACTION_ERROR")
    {
        mission_stage_ =
            MissionStage::None;


        setActiveOperation(
            "Robot action error");


        setMissionState(
            MissionState::
                MissionError);


        RCLCPP_ERROR(
            node_->get_logger(),
            "Robot action error during mission");


        return;
    }
}


void MissionManager::
handleNavigationFailed(
    const std::string &reason)
{
    if (mission_state_ !=
            MissionState::Running &&
        mission_state_ !=
            MissionState::Paused)
    {
        return;
    }


    mission_stage_ =
        MissionStage::None;


    setActiveOperation(
        "Mission navigation error");


    setMissionState(
        MissionState::
            MissionError);


    RCLCPP_ERROR(
        node_->get_logger(),
        "Mission navigation failed: %s",
        reason.c_str());
}


void MissionManager::pauseMission()
{
    if (mission_state_ !=
        MissionState::Running)
    {
        return;
    }


    // At the moment pause is supported
    // while Nav2 navigation is active.
    if (!navigation_manager_ ||
        !navigation_manager_->
            navigationActive() ||
        navigation_manager_->
            navigationPaused())
    {
        return;
    }


    navigation_manager_->
        pauseNavigation();


    setMissionState(
        MissionState::Paused);


    setActiveOperation(
        "Mission paused");
}


void MissionManager::resumeMission()
{
    if (emergency_stop_active_ ||
        mission_state_ !=
            MissionState::Paused)
    {
        return;
    }


    if (!navigation_manager_)
    {
        return;
    }


    if (navigation_manager_->
        navigationActive() &&
        navigation_manager_->
        navigationPaused())
    {
        setMissionState(
            MissionState::Running);


        navigation_manager_->
            resumeNavigation();


        setActiveOperation(
            "Mission resumed");
    }
}


void MissionManager::cancelMission()
{
    if (mission_stage_ ==
            MissionStage::
                PickupLineFollowing ||
        mission_stage_ ==
            MissionStage::
                DropoffLineFollowing)
    {
        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_STOP");
        }
    }


    if (navigation_manager_ &&
        navigation_manager_->
            navigationActive())
    {
        navigation_manager_->
            cancelNavigation(
                "Mission canceled");
    }


    if (motion_stop_handler_)
    {
        motion_stop_handler_();
    }


    mission_stage_ =
        MissionStage::None;


    if (mission_state_ !=
        MissionState::NoMission)
    {
        setMissionState(
            MissionState::
                Cancelled);
    }


    mission_ =
        MissionData{};


    setActiveOperation(
        "Idle");


    RCLCPP_WARN(
        node_->get_logger(),
        "Mission canceled");
}


void MissionManager::
setEmergencyStopActive(
    bool active)
{
    emergency_stop_active_ =
        active;


    if (!active)
    {
        return;
    }


    if (mission_stage_ ==
            MissionStage::
                PickupLineFollowing ||
        mission_stage_ ==
            MissionStage::
                DropoffLineFollowing)
    {
        if (robot_action_request_handler_)
        {
            robot_action_request_handler_(
                "LINE_STOP");
        }
    }


    if (mission_state_ ==
            MissionState::Approved ||
        mission_state_ ==
            MissionState::Running ||
        mission_state_ ==
            MissionState::Paused)
    {
        mission_stage_ =
            MissionStage::None;


        setMissionState(
            MissionState::
                Cancelled);
    }


    if (motion_stop_handler_)
    {
        motion_stop_handler_();
    }


    setActiveOperation(
        "Emergency stop");
}


void MissionManager::setMissionState(
    MissionState state)
{
    mission_state_ =
        state;
}


void MissionManager::setActiveOperation(
    const std::string &operation)
{
    active_operation_ =
        operation;


    RCLCPP_INFO(
        node_->get_logger(),
        "Mission operation: %s",
        active_operation_.c_str());
}


MissionState MissionManager::
missionState() const
{
    return mission_state_;
}


MissionStage MissionManager::
missionStage() const
{
    return mission_stage_;
}


std::string MissionManager::
missionId() const
{
    return mission_.missionId;
}


std::string MissionManager::
pickupStation() const
{
    return mission_.pickupStation;
}


std::string MissionManager::
dropoffStation() const
{
    return mission_.dropoffStation;
}


std::string MissionManager::
activeOperation() const
{
    return active_operation_;
}


std::string MissionManager::
missionStatusText() const
{
    switch (mission_state_)
    {
        case MissionState::
            PendingApproval:

            return "Pending approval";


        case MissionState::
            Approved:

            return "Approved";


        case MissionState::
            Running:

            return "Running";


        case MissionState::
            Paused:

            return "Paused";


        case MissionState::
            Completed:

            return "Completed";


        case MissionState::
            Cancelled:

            return "Cancelled";


        case MissionState::
            MissionError:

            return "Mission error";


        case MissionState::
            NoMission:

        default:

            return "No mission";
    }
}
