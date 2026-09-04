#ifndef AMR_MISSION_CONTROLLER__NAVIGATION_MANAGER_HPP_
#define AMR_MISSION_CONTROLLER__NAVIGATION_MANAGER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "amr_mission_controller/graph_manager.hpp"


enum class NavigationTargetType
{
    None,
    GraphWaypoint,
    FinalStation
};


struct NavigationStationTarget
{
    std::string name;

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    LocationAction arrivalAction =
        LocationAction::None;

    int linkedWaypointId = -1;
};


class NavigationManager
{
public:
    
    using NavigateToPose =
        nav2_msgs::action::NavigateToPose;

    using GoalHandleNavigateToPose =
        rclcpp_action::ClientGoalHandle<
            NavigateToPose>;


    using ArrivalActionHandler =
        std::function<bool(LocationAction)>;

    using MotionAllowedHandler =
        std::function<bool()>;
        
    using FinalStationReachedHandler =
        std::function<void(const std::string &)>;

    using NavigationFailedHandler =
        std::function<void(const std::string &)>;
        
    void setFinalStationReachedHandler(
        FinalStationReachedHandler handler);

    void setNavigationFailedHandler(
        NavigationFailedHandler handler);

    NavigationManager(
        GraphManager *graphManager,
        rclcpp::Node *node,
        ArrivalActionHandler arrivalActionHandler,
        MotionAllowedHandler motionAllowedHandler);


    bool navigateToStation(
        const NavigationStationTarget &station,
        double robotX,
        double robotY);


    void pauseNavigation();

    void resumeNavigation();

    void cancelNavigation(
        const std::string &status =
            "Navigation canceled");

    void abortForEmergencyStop();

    void continueAfterArrivalAction();


    bool navigationActive() const;

    bool navigationPaused() const;

    int currentGraphNodeId() const;

    int waypointIndex() const;

    std::string navigationStatus() const;

    std::string currentTargetName() const;


private:

    void sendNextGraphWaypoint();

    void rejectNavigationStart(
        const std::string &stationName,
        const std::string &reason);

    void finishNavigation(
        const std::string &status);


    GraphManager *graph_manager_ =
        nullptr;

    rclcpp::Node *node_ =
        nullptr;


    ArrivalActionHandler
        arrival_action_handler_;

    MotionAllowedHandler
        motion_allowed_handler_;

	FinalStationReachedHandler
	    final_station_reached_handler_;

	NavigationFailedHandler
	    navigation_failed_handler_;

    rclcpp_action::Client<
        NavigateToPose>::SharedPtr
        navigate_to_pose_client_;


    std::vector<int>
        navigation_waypoints_;


    std::size_t waypoint_index_ = 0;

    int current_graph_node_id_ = -1;


    bool navigation_active_ = false;
    bool navigation_paused_ = false;

    bool awaiting_arrival_action_ = false;
    bool awaiting_final_station_action_ = false;


    std::string navigation_status_ =
        "Idle";

    NavigationStationTarget
        target_station_;

    NavigationTargetType
        current_target_type_ =
            NavigationTargetType::None;

    std::string
        current_target_name_;


    std::uint64_t goal_generation_ = 0;


    GoalHandleNavigateToPose::SharedPtr
        current_goal_handle_;
};


#endif
