#include "amr_mission_controller/graph_manager.hpp"

#include "amr_mission_controller/location_action_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>


using json = nlohmann::json;


bool GraphManager::loadGraph(
    const std::string &nodesFile,
    const std::string &edgesFile)
{
    std::ifstream nodesStream(nodesFile);

    if (!nodesStream.is_open()) {
        return false;
    }

    std::ifstream edgesStream(edgesFile);

    if (!edgesStream.is_open()) {
        return false;
    }


    json nodeArray;
    json edgeArray;

    try {
        nodesStream >> nodeArray;
        edgesStream >> edgeArray;
    }
    catch (const json::exception &) {
        return false;
    }


    if (!nodeArray.is_array() ||
        !edgeArray.is_array()) {
        return false;
    }


    std::vector<GraphNode> loadedNodes;


    try {

        for (const auto &object : nodeArray) {

            if (!object.is_object()) {
                return false;
            }

            if (!object.contains("id") ||
                !object.contains("name") ||
                !object.contains("x") ||
                !object.contains("y")) {
                return false;
            }


            GraphNode node;

            node.id =
                object.at("id").get<int>();

            node.name =
                object.at("name").get<std::string>();

            node.x =
                object.at("x").get<double>();

            node.y =
                object.at("y").get<double>();


            const std::string actionName =
                object.value(
                    "arrivalAction",
                    std::string("None"));


            if (!locationActionFromString(
                    actionName,
                    node.arrivalAction)) {
                return false;
            }


            if (!std::isfinite(node.x) ||
                !std::isfinite(node.y)) {
                return false;
            }


            const bool duplicateId =
                std::any_of(
                    loadedNodes.begin(),
                    loadedNodes.end(),
                    [&node](const GraphNode &loadedNode)
                    {
                        return loadedNode.id == node.id;
                    });


            if (duplicateId) {
                return false;
            }


            loadedNodes.push_back(node);
        }
    }
    catch (const json::exception &) {
        return false;
    }


    const auto nodeExists =
        [&loadedNodes](int id)
        {
            return std::any_of(
                loadedNodes.begin(),
                loadedNodes.end(),
                [id](const GraphNode &node)
                {
                    return node.id == id;
                });
        };


    std::vector<GraphEdge> loadedEdges;


    try {

        for (const auto &object : edgeArray) {

            if (!object.is_object()) {
                return false;
            }


            if (!object.contains("startNodeId") ||
                !object.contains("endNodeId")) {
                return false;
            }


            GraphEdge edge;

            edge.startNodeId =
                object.at("startNodeId").get<int>();

            edge.endNodeId =
                object.at("endNodeId").get<int>();


            if (!nodeExists(edge.startNodeId) ||
                !nodeExists(edge.endNodeId)) {
                return false;
            }


            if (edge.startNodeId ==
                edge.endNodeId) {
                return false;
            }


            const bool duplicateEdge =
                std::any_of(
                    loadedEdges.begin(),
                    loadedEdges.end(),
                    [&edge](const GraphEdge &loadedEdge)
                    {
                        return
                            (loadedEdge.startNodeId ==
                                 edge.startNodeId &&
                             loadedEdge.endNodeId ==
                                 edge.endNodeId)
                            ||
                            (loadedEdge.startNodeId ==
                                 edge.endNodeId &&
                             loadedEdge.endNodeId ==
                                 edge.startNodeId);
                    });


            if (!duplicateEdge) {
                loadedEdges.push_back(edge);
            }
        }
    }
    catch (const json::exception &) {
        return false;
    }


    // Only replace the active graph after
    // BOTH files were successfully validated.
    graph_nodes_ = std::move(loadedNodes);
    graph_edges_ = std::move(loadedEdges);

    return true;
}


const GraphNode *GraphManager::findNodeById(
    int id) const
{
    const auto iterator =
        std::find_if(
            graph_nodes_.begin(),
            graph_nodes_.end(),
            [id](const GraphNode &node)
            {
                return node.id == id;
            });


    return iterator == graph_nodes_.end()
        ? nullptr
        : &(*iterator);
}


int GraphManager::findNearestNode(
    double x,
    double y) const
{
    int nearestNodeId = -1;

    double nearestDistanceSquared =
        std::numeric_limits<double>::infinity();


    for (const GraphNode &node : graph_nodes_) {

        const double deltaX =
            node.x - x;

        const double deltaY =
            node.y - y;


        const double distanceSquared =
            deltaX * deltaX +
            deltaY * deltaY;


        if (distanceSquared <
            nearestDistanceSquared) {

            nearestDistanceSquared =
                distanceSquared;

            nearestNodeId =
                node.id;
        }
    }


    return nearestNodeId;
}


std::vector<int> GraphManager::findGraphPath(
    int startNodeId,
    int goalNodeId) const
{
    std::vector<int> result;


    std::unordered_map<
        int,
        const GraphNode *> nodesById;


    for (const GraphNode &node :
         graph_nodes_) {

        nodesById[node.id] =
            &node;
    }


    if (nodesById.find(startNodeId) ==
            nodesById.end() ||
        nodesById.find(goalNodeId) ==
            nodesById.end()) {

        return result;
    }


    if (startNodeId ==
        goalNodeId) {

        result.push_back(
            startNodeId);

        return result;
    }


    struct Neighbor
    {
        int nodeId;
        double cost;
    };


    std::unordered_map<
        int,
        std::vector<Neighbor>>
        adjacency;


    for (const GraphEdge &edge :
         graph_edges_) {

        const auto startIterator =
            nodesById.find(
                edge.startNodeId);

        const auto endIterator =
            nodesById.find(
                edge.endNodeId);


        if (startIterator ==
                nodesById.end() ||
            endIterator ==
                nodesById.end()) {

            continue;
        }


        const double deltaX =
            endIterator->second->x -
            startIterator->second->x;

        const double deltaY =
            endIterator->second->y -
            startIterator->second->y;


        const double distance =
            std::hypot(
                deltaX,
                deltaY);


        adjacency[
            edge.startNodeId]
            .push_back({
                edge.endNodeId,
                distance
            });


        adjacency[
            edge.endNodeId]
            .push_back({
                edge.startNodeId,
                distance
            });
    }


    using QueueEntry =
        std::pair<double, int>;


    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        std::greater<QueueEntry>>
        queue;


    std::unordered_map<
        int,
        double> distances;


    std::unordered_map<
        int,
        int> previous;


    for (const auto &nodeEntry :
         nodesById) {

        distances[nodeEntry.first] =
            std::numeric_limits<
                double>::infinity();
    }


    distances[startNodeId] =
        0.0;


    queue.push({
        0.0,
        startNodeId
    });


    while (!queue.empty()) {

        const auto [
            currentDistance,
            currentNodeId] =
            queue.top();

        queue.pop();


        if (currentDistance >
            distances[currentNodeId]) {

            continue;
        }


        if (currentNodeId ==
            goalNodeId) {

            break;
        }


        const auto neighborIterator =
            adjacency.find(
                currentNodeId);


        if (neighborIterator ==
            adjacency.end()) {

            continue;
        }


        for (const Neighbor &neighbor :
             neighborIterator->second) {

            const double candidateDistance =
                currentDistance +
                neighbor.cost;


            if (candidateDistance <
                distances[
                    neighbor.nodeId]) {

                distances[
                    neighbor.nodeId] =
                    candidateDistance;


                previous[
                    neighbor.nodeId] =
                    currentNodeId;


                queue.push({
                    candidateDistance,
                    neighbor.nodeId
                });
            }
        }
    }


    if (!std::isfinite(
            distances[goalNodeId])) {

        return result;
    }


    std::vector<int>
        pathFromGoal;


    int currentNodeId =
        goalNodeId;


    pathFromGoal.push_back(
        currentNodeId);


    while (currentNodeId !=
           startNodeId) {

        const auto previousIterator =
            previous.find(
                currentNodeId);


        if (previousIterator ==
            previous.end()) {

            return {};
        }


        currentNodeId =
            previousIterator->second;


        pathFromGoal.push_back(
            currentNodeId);
    }


    for (auto iterator =
             pathFromGoal.rbegin();
         iterator !=
             pathFromGoal.rend();
         ++iterator) {

        result.push_back(
            *iterator);
    }


    return result;
}


const std::vector<GraphNode> &
GraphManager::nodes() const
{
    return graph_nodes_;
}


const std::vector<GraphEdge> &
GraphManager::edges() const
{
    return graph_edges_;
}
