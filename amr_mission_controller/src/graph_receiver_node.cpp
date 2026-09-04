#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>

#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <utility>

#include "amr_mission_controller/station_manager.hpp"
#include "amr_mission_controller/graph_manager.hpp"
#include "amr_mission_controller/navigation_manager.hpp"
#include "amr_mission_controller/mission_manager.hpp"


class GraphReceiverNode : public rclcpp::Node
{
public:

    GraphReceiverNode()
        : Node("graph_receiver")
    {
        // ============================================================
        // Graph storage directory
        // ============================================================

        const char *home =
            std::getenv("HOME");


        const std::string defaultDirectory =
            home
                ? std::string(home) +
                    "/.amr_mission_controller"
                : "/tmp/amr_mission_controller";


        graph_directory_ =
            declare_parameter<std::string>(
                "graph_directory",
                defaultDirectory);


        std::filesystem::create_directories(
            graph_directory_);


        nodes_file_ =
            graph_directory_ +
            "/graph_nodes.json";


        edges_file_ =
            graph_directory_ +
            "/graph_edges.json";

        stations_file_ =
	    graph_directory_ +
	    "/qr_stations.json";
        // ============================================================
        // TF parameters
        // ============================================================

        declare_parameter<std::string>(
            "map_frame",
            "map");


        declare_parameter<std::string>(
            "base_frame",
            "base_link");


        // ============================================================
        // TF
        // ============================================================

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                this->get_clock());


        tf_listener_ =
            std::make_shared<
                tf2_ros::TransformListener>(
                    *tf_buffer_);


        // ============================================================
        // Load graph already stored on Jetson
        // ============================================================

        loadStoredGraph();
        loadStoredStations();

        // ============================================================
        // NavigationManager
        // ============================================================

        navigation_manager_ =
            std::make_unique<NavigationManager>(
                &graph_manager_,
                this,

                // ----------------------------------------------------
                // Arrival action handler
                // ----------------------------------------------------
                [this](LocationAction action)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Arrival action received: %d",
                        static_cast<int>(action));


                    // Mission/RobotActionManager is not moved yet.
                    // Returning false means:
                    // no special action blocks navigation.
                    return false;
                },

                // ----------------------------------------------------
                // Motion allowed handler
                // ----------------------------------------------------
                [this]()
                {
                    // Later this will check:
                    // E-stop
                    // safety
                    // mission state
                    // etc.

                    return true;
                });


        // ============================================================
        // Mission command/action ROS interfaces
        // ============================================================

        robot_action_publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/robot_action",
                10);

        cmd_vel_stop_publisher_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10);

        const auto missionStatusQos =
            rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable()
                .transient_local();

        mission_status_publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/amr/mission_status_json",
                missionStatusQos);


        // ============================================================
        // MissionManager
        // ============================================================

        mission_manager_ =
            std::make_unique<MissionManager>(
                navigation_manager_.get(),
                this,

                // Navigate to a station by its stored station name.
                [this](const std::string &stationName)
                {
                    return navigateToStationByName(
                        stationName);
                },

                // Send line/lift commands to the existing robot action topic.
                [this](const std::string &action)
                {
                    std_msgs::msg::String msg;
                    msg.data = action;

                    robot_action_publisher_->publish(
                        msg);

                    RCLCPP_INFO(
                        get_logger(),
                        "Robot action requested: %s",
                        action.c_str());
                },

                // Hard motion stop requested by MissionManager.
                [this]()
                {
                    geometry_msgs::msg::Twist stopMsg;
                    cmd_vel_stop_publisher_->publish(
                        stopMsg);

                    RCLCPP_WARN(
                        get_logger(),
                        "MissionManager requested motion stop");
                });


        // NavigationManager -> MissionManager callbacks
        navigation_manager_->setFinalStationReachedHandler(
            [this](const std::string &stationName)
            {
                if (mission_manager_)
                {
                    mission_manager_->handleFinalStationReached(
                        stationName);
                }
            });

        navigation_manager_->setNavigationFailedHandler(
            [this](const std::string &reason)
            {
                if (mission_manager_)
                {
                    mission_manager_->handleNavigationFailed(
                        reason);
                }
            });


        // ============================================================
        // Graph JSON subscriptions
        // ============================================================

        auto graphQos =
            rclcpp::QoS(
                rclcpp::KeepLast(1));


        graphQos.reliable();
        graphQos.transient_local();


        nodes_subscription_ =
            create_subscription<
                std_msgs::msg::String>(
                    "/amr/graph_nodes_json",
                    graphQos,

                    std::bind(
                        &GraphReceiverNode::
                            nodesCallback,
                        this,
                        std::placeholders::_1));


        edges_subscription_ =
            create_subscription<
                std_msgs::msg::String>(
                    "/amr/graph_edges_json",
                    graphQos,

                    std::bind(
                        &GraphReceiverNode::
                            edgesCallback,
                        this,
                        std::placeholders::_1));

        stations_subscription_ =
	    create_subscription<
		std_msgs::msg::String>(
		    "/amr/qr_stations_json",
		    graphQos,

		    [this](
		        const std_msgs::msg::String::
		            SharedPtr msg)
		    {
		        receiveStations(
		            msg->data);
		    });
		// ============================================================
        // Temporary NavigationManager test subscription
        // ============================================================

        test_navigation_sub_ =
            create_subscription<
                std_msgs::msg::String>(
                    "/amr/test_navigation",
                    10,

                    [this](
                        const std_msgs::msg::String::
                            SharedPtr msg)
                    {
                        handleTestNavigation(
                            msg->data);
                    });


        // ============================================================
        // Mission command subscription
        //
        // CREATE A1 B1
        // APPROVE
        // START
        // PAUSE
        // RESUME
        // CANCEL
        // STATUS
        // ============================================================

        mission_command_subscription_ =
            create_subscription<std_msgs::msg::String>(
                "/amr/mission_command",
                10,
                [this](
                    const std_msgs::msg::String::SharedPtr msg)
                {
                    handleMissionCommand(
                        msg->data);
                });


        // Mission/robot action completion input.
        // Examples: LINE_COMPLETE, LIFT_UP_COMPLETE,
        //           LIFT_DOWN_COMPLETE, ACTION_ERROR
        robot_action_result_subscription_ =
            create_subscription<std_msgs::msg::String>(
                "/amr/robot_action_result",
                10,
                [this](
                    const std_msgs::msg::String::SharedPtr msg)
                {
                    if (!mission_manager_)
                    {
                        return;
                    }

                    RCLCPP_INFO(
                        get_logger(),
                        "Robot action result received: %s",
                        msg->data.c_str());

                    mission_manager_->handleRobotActionResult(
                        msg->data);
                });

        mission_status_timer_ =
            create_wall_timer(
                std::chrono::milliseconds(250),
                [this]()
                {
                    publishMissionStatus();
                });


        // ============================================================
        // Startup logs
        // ============================================================

        RCLCPP_INFO(
            get_logger(),
            "Graph receiver started");


        RCLCPP_INFO(
            get_logger(),
            "Graph directory: %s",
            graph_directory_.c_str());


        RCLCPP_INFO(
            get_logger(),
            "Navigation test topic: /amr/test_navigation");


        RCLCPP_INFO(
            get_logger(),
            "Available navigation commands: POSE, GO <station>, PAUSE, RESUME, CANCEL");

        RCLCPP_INFO(
            get_logger(),
            "Mission command topic: /amr/mission_command");

        RCLCPP_INFO(
            get_logger(),
            "Controller commands: GO <station>, CREATE <A*> <B*>, APPROVE, START, PAUSE, RESUME, CANCEL, STATUS");

        RCLCPP_INFO(
            get_logger(),
            "Robot action result topic: /amr/robot_action_result");

        RCLCPP_INFO(
            get_logger(),
            "Mission status topic: /amr/mission_status_json");
    }


private:

    void publishMissionStatus()
    {
        if (!mission_manager_ ||
            !mission_status_publisher_)
        {
            return;
        }

        nlohmann::json status;

        status["state"] =
            static_cast<int>(
                mission_manager_->missionState());

        status["stage"] =
            static_cast<int>(
                mission_manager_->missionStage());

        status["id"] =
            mission_manager_->missionId();

        status["pickup"] =
            mission_manager_->pickupStation();

        status["dropoff"] =
            mission_manager_->dropoffStation();

        status["operation"] =
            mission_manager_->activeOperation();

        status["statusText"] =
            mission_manager_->missionStatusText();

        // Navigation state is mirrored to the GUI as well.
        // This covers both mission navigation and direct GO <station> navigation.
        if (navigation_manager_)
        {
            status["navigationActive"] =
                navigation_manager_->navigationActive();

            status["navigationPaused"] =
                navigation_manager_->navigationPaused();

            status["navigationStatus"] =
                navigation_manager_->navigationStatus();

            status["currentTargetName"] =
                navigation_manager_->currentTargetName();

            status["currentGraphNodeId"] =
                navigation_manager_->currentGraphNodeId();

            status["waypointIndex"] =
                static_cast<int>(navigation_manager_->waypointIndex());
        }
        else
        {
            status["navigationActive"] = false;
            status["navigationPaused"] = false;
            status["navigationStatus"] = "Idle";
            status["currentTargetName"] = "";
            status["currentGraphNodeId"] = -1;
            status["waypointIndex"] = 0;
        }

        std_msgs::msg::String msg;
        msg.data = status.dump();

        mission_status_publisher_->publish(msg);
    }

    void receiveStations(const std::string &json)
	{
	    const std::string tempFile =
		stations_file_ + ".tmp";


	    if (!writeFile(
		    tempFile,
		    json))
	    {
		RCLCPP_ERROR(
		    get_logger(),
		    "Could not write temporary station file");

		return;
	    }


	    StationManager candidateStations;


	    if (!candidateStations.loadStations(
		    tempFile))
	    {
		RCLCPP_ERROR(
		    get_logger(),
		    "Received station JSON is invalid. "
		    "Old stations remain active.");

		std::filesystem::remove(
		    tempFile);

		return;
	    }


	    try
	    {
		std::filesystem::rename(
		    tempFile,
		    stations_file_);
	    }
	    catch (
		const std::filesystem::
		    filesystem_error &ex)
	    {
		RCLCPP_ERROR(
		    get_logger(),
		    "Could not save station file: %s",
		    ex.what());

		std::filesystem::remove(
		    tempFile);

		return;
	    }


	    station_manager_ =
		std::move(
		    candidateStations);


	    RCLCPP_INFO(
		get_logger(),
		"New stations activated. Count=%zu",
		station_manager_
		    .stations()
		    .size());


	    for (const auto &station :
		 station_manager_.stations())
	    {
		RCLCPP_INFO(
		    get_logger(),
		    "Station: %s  x=%.3f y=%.3f yaw=%.3f linkedNode=%d",
		    station.stationName.c_str(),
		    station.x,
		    station.y,
		    station.yaw,
		    station.linkedWaypointId);
	    }
	}
	void loadStoredStations()
	{
	    if (!std::filesystem::exists(
		    stations_file_))
	    {
		RCLCPP_WARN(
		    get_logger(),
		    "No stored stations found yet");

		return;
	    }


	    if (!station_manager_.loadStations(
		    stations_file_))
	    {
		RCLCPP_ERROR(
		    get_logger(),
		    "Stored stations could not be loaded");

		return;
	    }


	    RCLCPP_INFO(
		get_logger(),
		"Stored stations loaded. Count=%zu",
		station_manager_
		    .stations()
		    .size());
	}
    // ================================================================
    // Graph node JSON callback
    // ================================================================

    void nodesCallback(
        const std_msgs::msg::String::
            SharedPtr msg)
    {
        pending_nodes_json_ =
            msg->data;


        nodes_received_ =
            true;


        RCLCPP_INFO(
            get_logger(),
            "Received graph_nodes.json (%zu bytes)",
            pending_nodes_json_.size());


        tryUpdateGraph();
    }


    // ================================================================
    // Graph edge JSON callback
    // ================================================================

    void edgesCallback(
        const std_msgs::msg::String::
            SharedPtr msg)
    {
        pending_edges_json_ =
            msg->data;


        edges_received_ =
            true;


        RCLCPP_INFO(
            get_logger(),
            "Received graph_edges.json (%zu bytes)",
            pending_edges_json_.size());


        tryUpdateGraph();
    }


    // ================================================================
    // Write file
    // ================================================================

    bool writeFile(
        const std::string &path,
        const std::string &content)
    {
        std::ofstream file(
            path,
            std::ios::out |
            std::ios::trunc);


        if (!file.is_open()) {

            return false;
        }


        file << content;


        file.close();


        return file.good();
    }


    // ================================================================
    // Validate + activate newly received graph
    // ================================================================

    void tryUpdateGraph()
    {
        if (!nodes_received_ ||
            !edges_received_) {

            return;
        }


        const std::string tempNodes =
            nodes_file_ +
            ".tmp";


        const std::string tempEdges =
            edges_file_ +
            ".tmp";


        // ------------------------------------------------------------
        // Write temporary files
        // ------------------------------------------------------------

        if (!writeFile(
                tempNodes,
                pending_nodes_json_) ||
            !writeFile(
                tempEdges,
                pending_edges_json_))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Could not write temporary graph files");


            resetPending();


            return;
        }


        // ------------------------------------------------------------
        // Validate graph BEFORE replacing active graph
        // ------------------------------------------------------------

        GraphManager candidateGraph;


        if (!candidateGraph.loadGraph(
                tempNodes,
                tempEdges))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Received graph is invalid. Old graph remains active.");


            std::filesystem::remove(
                tempNodes);


            std::filesystem::remove(
                tempEdges);


            resetPending();


            return;
        }


        // ------------------------------------------------------------
        // Replace stored files
        // ------------------------------------------------------------

        try
        {
            std::filesystem::rename(
                tempNodes,
                nodes_file_);


            std::filesystem::rename(
                tempEdges,
                edges_file_);
        }
        catch (
            const std::filesystem::
                filesystem_error &ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Could not replace stored graph files: %s",
                ex.what());


            std::filesystem::remove(
                tempNodes);


            std::filesystem::remove(
                tempEdges);


            resetPending();


            return;
        }


        // ------------------------------------------------------------
        // Activate graph in RAM
        // ------------------------------------------------------------

        graph_manager_ =
            std::move(
                candidateGraph);


        RCLCPP_INFO(
            get_logger(),
            "New graph activated. Nodes=%zu Edges=%zu",
            graph_manager_.nodes().size(),
            graph_manager_.edges().size());


        // ------------------------------------------------------------
        // Print loaded nodes
        // ------------------------------------------------------------

        for (const auto &node :
             graph_manager_.nodes())
        {
            RCLCPP_INFO(
                get_logger(),
                "Node %d: %s (%.3f, %.3f)",
                node.id,
                node.name.c_str(),
                node.x,
                node.y);
        }


        resetPending();
    }


    // ================================================================
    // Load graph from Jetson disk
    // ================================================================

    void loadStoredGraph()
    {
        if (!std::filesystem::exists(
                nodes_file_) ||
            !std::filesystem::exists(
                edges_file_))
        {
            RCLCPP_WARN(
                get_logger(),
                "No stored graph found yet");


            return;
        }


        if (!graph_manager_.loadGraph(
                nodes_file_,
                edges_file_))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Stored graph could not be loaded");


            return;
        }


        RCLCPP_INFO(
            get_logger(),
            "Stored graph loaded. Nodes=%zu Edges=%zu",
            graph_manager_.nodes().size(),
            graph_manager_.edges().size());
    }


    // ================================================================
    // Reset graph-transfer state
    // ================================================================

    void resetPending()
    {
        nodes_received_ =
            false;


        edges_received_ =
            false;


        pending_nodes_json_.clear();
        pending_edges_json_.clear();
    }


    // ================================================================
    // Get robot position from TF
    //
    // map -> base_link
    // ================================================================

    bool getRobotPose(
        double &robotX,
        double &robotY)
    {
        const std::string mapFrame =
            get_parameter(
                "map_frame")
                .as_string();


        const std::string baseFrame =
            get_parameter(
                "base_frame")
                .as_string();


        try
        {
            const auto transform =
                tf_buffer_->lookupTransform(
                    mapFrame,
                    baseFrame,
                    tf2::TimePointZero);


            robotX =
                transform.transform
                    .translation.x;


            robotY =
                transform.transform
                    .translation.y;


            return true;
        }
        catch (
            const tf2::
                TransformException &ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot get TF %s -> %s: %s",
                mapFrame.c_str(),
                baseFrame.c_str(),
                ex.what());


            return false;
        }
    }


    // ================================================================
    // Navigate to a station stored in StationManager
    // ================================================================

    bool navigateToStationByName(
        const std::string &stationName)
    {
        if (!navigation_manager_)
        {
            RCLCPP_ERROR(
                get_logger(),
                "NavigationManager is not available");

            return false;
        }


        const StationInfo *stationInfo =
            station_manager_.findStationByName(
                stationName);


        if (!stationInfo)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Station not found: %s",
                stationName.c_str());

            return false;
        }


        if (stationInfo->linkedWaypointId == -1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Station %s has no linked waypoint",
                stationName.c_str());

            return false;
        }


        if (!graph_manager_.findNodeById(
                stationInfo->linkedWaypointId))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Station %s references missing graph node %d",
                stationName.c_str(),
                stationInfo->linkedWaypointId);

            return false;
        }


        double robotX = 0.0;
        double robotY = 0.0;


        if (!getRobotPose(
                robotX,
                robotY))
        {
            return false;
        }


        NavigationStationTarget target;

        target.name =
            stationInfo->stationName;

        target.x =
            stationInfo->x;

        target.y =
            stationInfo->y;

        target.yaw =
            stationInfo->yaw;

        target.arrivalAction =
            stationInfo->arrivalAction;

        target.linkedWaypointId =
            stationInfo->linkedWaypointId;


        RCLCPP_INFO(
            get_logger(),
            "Starting station navigation: %s | robot=(%.3f, %.3f) station=(%.3f, %.3f, %.3f) linkedNode=%d",
            target.name.c_str(),
            robotX,
            robotY,
            target.x,
            target.y,
            target.yaw,
            target.linkedWaypointId);


        return navigation_manager_->navigateToStation(
            target,
            robotX,
            robotY);
    }


    // ================================================================
    // Mission commands
    // ================================================================

    void handleMissionCommand(
        const std::string &command)
    {
        if (!mission_manager_)
        {
            RCLCPP_ERROR(
                get_logger(),
                "MissionManager is not available");

            return;
        }


        RCLCPP_INFO(
            get_logger(),
            "Mission command received: %s",
            command.c_str());


        // ============================================================
        // Direct station navigation: GO <station>
        //
        // This is NOT a mission. Jetson NavigationManager owns it.
        // ============================================================

        if (command.rfind("GO ", 0) == 0)
        {
            std::string stationName =
                command.substr(3);

            while (!stationName.empty() &&
                   stationName.front() == ' ')
            {
                stationName.erase(
                    stationName.begin());
            }

            while (!stationName.empty() &&
                   stationName.back() == ' ')
            {
                stationName.pop_back();
            }

            if (stationName.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "GO command requires a station name");
                return;
            }

            const MissionState state =
                mission_manager_->missionState();

            const bool missionOwnsControl =
                state == MissionState::PendingApproval ||
                state == MissionState::Approved ||
                state == MissionState::Running ||
                state == MissionState::Paused;

            if (missionOwnsControl)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Direct GO rejected because a mission is active/pending");
                return;
            }

            if (!navigateToStationByName(
                    stationName))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Direct navigation rejected: %s",
                    stationName.c_str());
            }

            return;
        }


        if (command.rfind("CREATE ", 0) == 0)
        {
            std::istringstream stream(
                command);

            std::string verb;
            std::string pickup;
            std::string dropoff;
            std::string extra;

            stream >> verb >> pickup >> dropoff >> extra;


            if (pickup.empty() ||
                dropoff.empty() ||
                !extra.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Usage: CREATE <pickup> <dropoff>  (example: CREATE A1 B1)");

                return;
            }


            if (!station_manager_.findStationByName(
                    pickup))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Pickup station does not exist: %s",
                    pickup.c_str());

                return;
            }


            if (!station_manager_.findStationByName(
                    dropoff))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Dropoff station does not exist: %s",
                    dropoff.c_str());

                return;
            }


            if (!mission_manager_->createManualMission(
                    pickup,
                    dropoff))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Mission creation rejected");
            }

            return;
        }


        if (command == "APPROVE")
        {
            if (!mission_manager_->approveMission())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Mission approval rejected");
            }

            return;
        }


        if (command == "START")
        {
            mission_manager_->startMission();
            return;
        }


        if (command == "PAUSE")
        {
            const MissionState state =
                mission_manager_->missionState();

            if (state == MissionState::Running)
            {
                mission_manager_->pauseMission();
                return;
            }

            if (navigation_manager_ &&
                navigation_manager_->navigationActive() &&
                !navigation_manager_->navigationPaused())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Pausing direct station navigation");

                navigation_manager_->pauseNavigation();
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "PAUSE ignored: no active mission/direct navigation");
            return;
        }


        if (command == "RESUME")
        {
            const MissionState state =
                mission_manager_->missionState();

            if (state == MissionState::Paused)
            {
                mission_manager_->resumeMission();
                return;
            }

            if (navigation_manager_ &&
                navigation_manager_->navigationActive() &&
                navigation_manager_->navigationPaused())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Resuming direct station navigation");

                navigation_manager_->resumeNavigation();
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "RESUME ignored: no paused mission/direct navigation");
            return;
        }


        if (command == "CANCEL")
        {
            const MissionState state =
                mission_manager_->missionState();

            const bool missionOwnsControl =
                state == MissionState::PendingApproval ||
                state == MissionState::Approved ||
                state == MissionState::Running ||
                state == MissionState::Paused;

            if (missionOwnsControl)
            {
                mission_manager_->cancelMission();
                return;
            }

            if (navigation_manager_ &&
                navigation_manager_->navigationActive())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Canceling direct station navigation");

                navigation_manager_->cancelNavigation(
                    "Direct navigation canceled by operator");
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "CANCEL ignored: no active mission/direct navigation");
            return;
        }


        if (command == "STATUS")
        {
            RCLCPP_INFO(
                get_logger(),
                "Mission status | id=%s state=%d stage=%d pickup=%s dropoff=%s operation=%s",
                mission_manager_->missionId().c_str(),
                static_cast<int>(
                    mission_manager_->missionState()),
                static_cast<int>(
                    mission_manager_->missionStage()),
                mission_manager_->pickupStation().c_str(),
                mission_manager_->dropoffStation().c_str(),
                mission_manager_->activeOperation().c_str());

            return;
        }


        RCLCPP_WARN(
            get_logger(),
            "Unknown mission command: %s",
            command.c_str());
    }


    // ================================================================
    // Temporary NavigationManager test commands
    // ================================================================

    void handleTestNavigation(
	    const std::string &command)
	{
	    // ============================================================
	    // POSE
	    // ============================================================

	    if (command == "POSE")
	    {
		double robotX = 0.0;
		double robotY = 0.0;

		if (!getRobotPose(
		        robotX,
		        robotY))
		{
		    return;
		}


		RCLCPP_INFO(
		    get_logger(),
		    "Robot position: x=%.3f y=%.3f",
		    robotX,
		    robotY);


		if (graph_manager_.nodes().empty())
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "Cannot calculate nearest node: graph is empty");

		    return;
		}


		const int nearest =
		    graph_manager_.findNearestNode(
		        robotX,
		        robotY);


		RCLCPP_INFO(
		    get_logger(),
		    "Nearest graph node = %d",
		    nearest);

		return;
	    }


	    // ============================================================
	    // GO <station name>
	    //
	    // Example:
	    // GO A1
	    // GO B1
	    // ============================================================

	    if (command.rfind("GO ", 0) == 0)
	    {
		if (!navigation_manager_)
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "NavigationManager is not available");

		    return;
		}


		std::string stationName =
		    command.substr(3);


		// Remove simple leading/trailing spaces.
		while (!stationName.empty() &&
		       stationName.front() == ' ')
		{
		    stationName.erase(
		        stationName.begin());
		}


		while (!stationName.empty() &&
		       stationName.back() == ' ')
		{
		    stationName.pop_back();
		}


		if (stationName.empty())
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "GO command requires a station name");

		    return;
		}


		// --------------------------------------------------------
		// Find station from Jetson StationManager
		// --------------------------------------------------------

		const StationInfo *stationInfo =
		    station_manager_.findStationByName(
		        stationName);


		if (!stationInfo)
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "Station not found: %s",
		        stationName.c_str());

		    return;
		}


		// --------------------------------------------------------
		// Validate station link
		// --------------------------------------------------------

		if (stationInfo->linkedWaypointId == -1)
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "Station %s has no linked waypoint",
		        stationName.c_str());

		    return;
		}


		if (!graph_manager_.findNodeById(
		        stationInfo->linkedWaypointId))
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "Station %s references missing graph node %d",
		        stationName.c_str(),
		        stationInfo->linkedWaypointId);

		    return;
		}


		// --------------------------------------------------------
		// Get real robot position from TF
		// --------------------------------------------------------

		double robotX = 0.0;
		double robotY = 0.0;


		if (!getRobotPose(
		        robotX,
		        robotY))
		{
		    return;
		}


		// --------------------------------------------------------
		// Convert StationInfo -> NavigationStationTarget
		// --------------------------------------------------------

		NavigationStationTarget target;


		target.name =
		    stationInfo->stationName;


		target.x =
		    stationInfo->x;


		target.y =
		    stationInfo->y;


		target.yaw =
		    stationInfo->yaw;


		target.arrivalAction =
		    stationInfo->arrivalAction;


		target.linkedWaypointId =
		    stationInfo->linkedWaypointId;


		// --------------------------------------------------------
		// Debug information
		// --------------------------------------------------------

		RCLCPP_INFO(
		    get_logger(),
		    "GO command received: %s",
		    stationName.c_str());


		RCLCPP_INFO(
		    get_logger(),
		    "Robot position: x=%.3f y=%.3f",
		    robotX,
		    robotY);


		RCLCPP_INFO(
		    get_logger(),
		    "Station %s: x=%.3f y=%.3f yaw=%.3f linkedNode=%d",
		    target.name.c_str(),
		    target.x,
		    target.y,
		    target.yaw,
		    target.linkedWaypointId);


		// --------------------------------------------------------
		// Start NavigationManager
		// --------------------------------------------------------

		const bool started =
		    navigation_manager_->navigateToStation(
		        target,
		        robotX,
		        robotY);


		if (!started)
		{
		    RCLCPP_ERROR(
		        get_logger(),
		        "NavigationManager rejected navigation to %s",
		        stationName.c_str());
		}


		return;
	    }


	    // ============================================================
	    // PAUSE
	    // ============================================================

	    if (command == "PAUSE")
	    {
		if (navigation_manager_)
		{
		    navigation_manager_->
		        pauseNavigation();
		}

		return;
	    }


	    // ============================================================
	    // RESUME
	    // ============================================================

	    if (command == "RESUME")
	    {
		if (navigation_manager_)
		{
		    navigation_manager_->
		        resumeNavigation();
		}

		return;
	    }


	    // ============================================================
	    // CANCEL
	    // ============================================================

	    if (command == "CANCEL")
	    {
		if (navigation_manager_)
		{
		    navigation_manager_->
		        cancelNavigation(
		            "Navigation canceled by command");
		}

		return;
	    }


	    // ============================================================
	    // Unknown command
	    // ============================================================

	    RCLCPP_WARN(
		get_logger(),
		"Unknown navigation command: %s",
		command.c_str());
	}


    StationManager station_manager_;

	std::string stations_file_;

	rclcpp::Subscription<
	    std_msgs::msg::String>::SharedPtr
	    stations_subscription_;
    // ================================================================
    // Managers
    // ================================================================

    GraphManager graph_manager_;


    std::unique_ptr<
        NavigationManager>
        navigation_manager_;


    std::unique_ptr<
        MissionManager>
        mission_manager_;


    // ================================================================
    // Mission / robot action ROS interfaces
    // ================================================================

    rclcpp::Publisher<
        std_msgs::msg::String>::SharedPtr
        robot_action_publisher_;

    rclcpp::Publisher<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_stop_publisher_;

    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr
        mission_command_subscription_;

    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr
        robot_action_result_subscription_;

    rclcpp::Publisher<
        std_msgs::msg::String>::SharedPtr
        mission_status_publisher_;

    rclcpp::TimerBase::SharedPtr
        mission_status_timer_;


    // ================================================================
    // TF
    // ================================================================

    std::unique_ptr<
        tf2_ros::Buffer>
        tf_buffer_;


    std::shared_ptr<
        tf2_ros::TransformListener>
        tf_listener_;


    // ================================================================
    // File paths
    // ================================================================

    std::string graph_directory_;

    std::string nodes_file_;

    std::string edges_file_;


    // ================================================================
    // Pending graph data
    // ================================================================

    std::string pending_nodes_json_;

    std::string pending_edges_json_;


    bool nodes_received_ =
        false;


    bool edges_received_ =
        false;


    // ================================================================
    // ROS subscriptions
    // ================================================================

    rclcpp::Subscription<
        std_msgs::msg::String>::
        SharedPtr
        nodes_subscription_;


    rclcpp::Subscription<
        std_msgs::msg::String>::
        SharedPtr
        edges_subscription_;


    rclcpp::Subscription<
        std_msgs::msg::String>::
        SharedPtr
        test_navigation_sub_;
};


// ====================================================================
// Main
// ====================================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);


    rclcpp::spin(
        std::make_shared<
            GraphReceiverNode>());


    rclcpp::shutdown();


    return 0;
}

