#include "amr_mission_controller/navigation_manager.hpp"

#include <chrono>
#include <cmath>
#include <utility>


NavigationManager::NavigationManager(
    GraphManager *graphManager,
    rclcpp::Node *node,
    ArrivalActionHandler arrivalActionHandler,
    MotionAllowedHandler motionAllowedHandler)
    :
    graph_manager_(graphManager),
    node_(node),
    arrival_action_handler_(
        std::move(arrivalActionHandler)),
    motion_allowed_handler_(
        std::move(motionAllowedHandler))
{
    navigate_to_pose_client_ =
        rclcpp_action::create_client<
            NavigateToPose>(
                node_,
                "/navigate_to_pose");
}


bool NavigationManager::navigateToStation(
    const NavigationStationTarget &station,
    double robotX,
    double robotY)
{
    if (navigation_active_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Navigation already active");

        return false;
    }


    if (!std::isfinite(robotX) ||
        !std::isfinite(robotY)) {

        rejectNavigationStart(
            station.name,
            "robot pose unavailable");

        return false;
    }


    if (!graph_manager_ ||
        graph_manager_->nodes().empty()) {

        rejectNavigationStart(
            station.name,
            "graph is empty");

        return false;
    }


    if (!std::isfinite(station.x) ||
        !std::isfinite(station.y) ||
        !std::isfinite(station.yaw)) {

        rejectNavigationStart(
            station.name,
            "station coordinates invalid");

        return false;
    }


    if (station.linkedWaypointId == -1) {

        rejectNavigationStart(
            station.name,
            "station has no linked waypoint");

        return false;
    }


    if (!graph_manager_->findNodeById(
            station.linkedWaypointId)) {

        rejectNavigationStart(
            station.name,
            "linked waypoint does not exist");

        return false;
    }


    const int startNodeId =
        graph_manager_->findNearestNode(
            robotX,
            robotY);


    navigation_waypoints_ =
        graph_manager_->findGraphPath(
            startNodeId,
            station.linkedWaypointId);


    if (navigation_waypoints_.empty()) {

        rejectNavigationStart(
            station.name,
            "no graph path to station");

        return false;
    }


    waypoint_index_ = 0;

    current_graph_node_id_ =
        navigation_waypoints_.front();

    navigation_active_ = true;
    navigation_paused_ = false;

    awaiting_arrival_action_ = false;
    awaiting_final_station_action_ = false;

    target_station_ = station;


    navigation_status_ =
        "Navigating to station " +
        station.name;


    RCLCPP_INFO(
        node_->get_logger(),
        "Starting navigation to %s",
        station.name.c_str());


    std::string pathText;

    for (int id : navigation_waypoints_) {

        if (!pathText.empty())
            pathText += " -> ";

        pathText += std::to_string(id);
    }


    RCLCPP_INFO(
        node_->get_logger(),
        "Graph route: %s",
        pathText.c_str());


    sendNextGraphWaypoint();

    return true;
}


void NavigationManager::sendNextGraphWaypoint()
{
    if ((motion_allowed_handler_ &&
         !motion_allowed_handler_()) ||
        !navigation_active_ ||
        navigation_paused_ ||
        awaiting_arrival_action_) {

        return;
    }


    if (!navigate_to_pose_client_) {

        finishNavigation(
            "Nav2 action client unavailable");

        return;
    }


    if (!navigate_to_pose_client_
             ->wait_for_action_server(
                 std::chrono::seconds(1))) {

        finishNavigation(
            "Nav2 action server unavailable");

        return;
    }


    const bool isStationGoal =
        waypoint_index_ >=
        navigation_waypoints_.size();


    double targetX =
        target_station_.x;

    double targetY =
        target_station_.y;

    double yaw = 0.0;


    LocationAction arrivalAction =
        LocationAction::None;


    int waypointNodeId = -1;


    if (isStationGoal)
    {
        yaw =
            target_station_.yaw;

        arrivalAction =
            target_station_.arrivalAction;


        current_target_type_ =
            NavigationTargetType::
                FinalStation;

        current_target_name_ =
            target_station_.name;


        RCLCPP_INFO(
            node_->get_logger(),
            "Sending FINAL station goal %s: x=%.3f y=%.3f yaw=%.3f",
            target_station_.name.c_str(),
            targetX,
            targetY,
            yaw);
    }
    else
    {
        waypointNodeId =
            navigation_waypoints_[
                waypoint_index_];


        const GraphNode *waypointNode =
            graph_manager_->findNodeById(
                waypointNodeId);


        if (!waypointNode) {

            finishNavigation(
                "route waypoint disappeared");

            return;
        }


        targetX =
            waypointNode->x;

        targetY =
            waypointNode->y;


        arrivalAction =
            waypointNode->arrivalAction;


        if (waypoint_index_ + 1 <
            navigation_waypoints_.size())
        {
            const GraphNode *nextNode =
                graph_manager_->findNodeById(
                    navigation_waypoints_[
                        waypoint_index_ + 1]);


            if (nextNode) {

                yaw =
                    std::atan2(
                        nextNode->y -
                            waypointNode->y,

                        nextNode->x -
                            waypointNode->x);
            }
        }
        else
        {
            yaw =
                std::atan2(
                    target_station_.y -
                        waypointNode->y,

                    target_station_.x -
                        waypointNode->x);
        }


        current_graph_node_id_ =
            waypointNodeId;

        current_target_type_ =
            NavigationTargetType::
                GraphWaypoint;

        current_target_name_ =
            waypointNode->name;


        RCLCPP_INFO(
            node_->get_logger(),
            "Sending waypoint %d (%s): x=%.3f y=%.3f yaw=%.3f",
            waypointNodeId,
            waypointNode->name.c_str(),
            targetX,
            targetY,
            yaw);
    }


    const std::uint64_t goalGeneration =
        ++goal_generation_;


    current_goal_handle_.reset();


    NavigateToPose::Goal goalMessage;


    goalMessage.pose.header.stamp =
        node_->now();

    goalMessage.pose.header.frame_id =
        "map";


    goalMessage.pose.pose.position.x =
        targetX;

    goalMessage.pose.pose.position.y =
        targetY;


    goalMessage.pose.pose.orientation.z =
        std::sin(yaw / 2.0);

    goalMessage.pose.pose.orientation.w =
        std::cos(yaw / 2.0);


    auto options =
        rclcpp_action::Client<
            NavigateToPose>::
            SendGoalOptions();


    options.goal_response_callback =
        [this, goalGeneration](
            const GoalHandleNavigateToPose::
                SharedPtr &goalHandle)
        {
            if (goalGeneration !=
                goal_generation_) {

                if (goalHandle &&
                    navigate_to_pose_client_) {

                    navigate_to_pose_client_
                        ->async_cancel_goal(
                            goalHandle);
                }

                return;
            }


            if (!goalHandle) {

                finishNavigation(
                    "Nav2 rejected goal");

                return;
            }


            current_goal_handle_ =
                goalHandle;


            RCLCPP_INFO(
                node_->get_logger(),
                "Nav2 goal accepted");
        };


    options.feedback_callback =
        [this](
            GoalHandleNavigateToPose::
                SharedPtr,

            const std::shared_ptr<
                const NavigateToPose::
                    Feedback> feedback)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "Distance remaining: %.2f m",
                feedback->distance_remaining);
        };


    options.result_callback =
        [this,
         goalGeneration,
         isStationGoal,
         arrivalAction,
         waypointNodeId](
            const GoalHandleNavigateToPose::
                WrappedResult &result)
        {
            if (goalGeneration !=
                    goal_generation_ ||
                !navigation_active_) {

                return;
            }


            current_goal_handle_.reset();


            if (result.code ==
                rclcpp_action::
                    ResultCode::SUCCEEDED)
            {
                if (!isStationGoal)
                {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Waypoint %d reached",
                        waypointNodeId);
                }


                awaiting_final_station_action_ =
                    isStationGoal;


                if (arrival_action_handler_ &&
                    arrival_action_handler_(
                        arrivalAction))
                {
                    awaiting_arrival_action_ =
                        true;

                    return;
                }


                if (isStationGoal)
                {
                    const std::string stationName =
			    target_station_.name;

			finishNavigation(
			    "Arrived at station " +
			    stationName);

			RCLCPP_INFO(
			    node_->get_logger(),
			    "FINAL STATION %s REACHED",
			    stationName.c_str());

			if (final_station_reached_handler_)
			{
			    final_station_reached_handler_(
				stationName);
			}

			return;
                }


                ++waypoint_index_;


                sendNextGraphWaypoint();

                return;
            }


            if (result.code ==
                rclcpp_action::
                    ResultCode::CANCELED)
            {
                finishNavigation(
                    "Nav2 goal canceled");
            }
            else
            {
                finishNavigation(
                    "Nav2 goal failed");
            }
        };


    navigate_to_pose_client_
        ->async_send_goal(
            goalMessage,
            options);
}


void NavigationManager::
continueAfterArrivalAction()
{
    if (!navigation_active_ ||
        !awaiting_arrival_action_) {

        return;
    }


    awaiting_arrival_action_ =
        false;


    if (awaiting_final_station_action_)
    {
        awaiting_final_station_action_ =
            false;


        finishNavigation(
            "Arrived at station " +
            target_station_.name);

        return;
    }


    ++waypoint_index_;


    if (!navigation_paused_) {
        sendNextGraphWaypoint();
    }
}


void NavigationManager::pauseNavigation()
{
    if (!navigation_active_ ||
        navigation_paused_) {

        return;
    }


    navigation_paused_ = true;

    ++goal_generation_;


    const auto goalHandle =
        current_goal_handle_;


    current_goal_handle_.reset();


    if (goalHandle &&
        navigate_to_pose_client_) {

        navigate_to_pose_client_
            ->async_cancel_goal(
                goalHandle);
    }


    navigation_status_ =
        "Navigation paused";


    RCLCPP_WARN(
        node_->get_logger(),
        "Navigation paused");
}


void NavigationManager::resumeNavigation()
{
    if (!navigation_active_ ||
        !navigation_paused_) {

        return;
    }


    navigation_paused_ = false;


    navigation_status_ =
        "Navigation resumed";


    RCLCPP_INFO(
        node_->get_logger(),
        "Navigation resumed");


    sendNextGraphWaypoint();
}


void NavigationManager::cancelNavigation(
    const std::string &status)
{
    if (!navigation_active_) {
        return;
    }


    ++goal_generation_;


    const auto goalHandle =
        current_goal_handle_;


    current_goal_handle_.reset();


    finishNavigation(status);


    if (goalHandle &&
        navigate_to_pose_client_) {

        navigate_to_pose_client_
            ->async_cancel_goal(
                goalHandle);
    }
}


void NavigationManager::
abortForEmergencyStop()
{
    ++goal_generation_;


    navigation_active_ = false;
    navigation_paused_ = false;

    waypoint_index_ = 0;

    current_graph_node_id_ = -1;

    current_goal_handle_.reset();

    awaiting_arrival_action_ = false;
    awaiting_final_station_action_ = false;


    current_target_type_ =
        NavigationTargetType::None;

    current_target_name_.clear();


    navigation_status_ =
        "Aborted by emergency stop";


    if (navigate_to_pose_client_) {

        navigate_to_pose_client_
            ->async_cancel_all_goals();
    }


    RCLCPP_ERROR(
        node_->get_logger(),
        "Navigation aborted by emergency stop");
}


void NavigationManager::
rejectNavigationStart(
    const std::string &stationName,
    const std::string &reason)
{
    navigation_waypoints_.clear();

    waypoint_index_ = 0;

    current_graph_node_id_ = -1;

    navigation_active_ = false;
    navigation_paused_ = false;


    target_station_ =
        NavigationStationTarget{};

    target_station_.name =
        stationName;


    current_target_type_ =
        NavigationTargetType::None;

    current_target_name_.clear();


    navigation_status_ =
        "Error: " + reason;


    RCLCPP_ERROR(
        node_->get_logger(),
        "Navigation start rejected: %s",
        reason.c_str());
        
    if (navigation_failed_handler_)
	{
	    navigation_failed_handler_(
		navigation_status_);
	}
}


void NavigationManager::finishNavigation(
    const std::string &status)
{
    navigation_active_ = false;
    navigation_paused_ = false;

    current_graph_node_id_ = -1;

    current_goal_handle_.reset();

    awaiting_arrival_action_ = false;
    awaiting_final_station_action_ = false;

    current_target_type_ =
        NavigationTargetType::None;

    current_target_name_.clear();

    navigation_status_ =
        status;


    RCLCPP_INFO(
        node_->get_logger(),
        "%s",
        status.c_str());
}
void NavigationManager::setFinalStationReachedHandler(
    FinalStationReachedHandler handler)
{
    final_station_reached_handler_ =
        std::move(handler);
}


void NavigationManager::setNavigationFailedHandler(
    NavigationFailedHandler handler)
{
    navigation_failed_handler_ =
        std::move(handler);
}

bool NavigationManager::
navigationActive() const
{
    return navigation_active_;
}


bool NavigationManager::
navigationPaused() const
{
    return navigation_paused_;
}


int NavigationManager::
currentGraphNodeId() const
{
    return current_graph_node_id_;
}


int NavigationManager::
waypointIndex() const
{
    return static_cast<int>(
        waypoint_index_);
}


std::string NavigationManager::
navigationStatus() const
{
    return navigation_status_;
}


std::string NavigationManager::
currentTargetName() const
{
    return current_target_name_;
}
