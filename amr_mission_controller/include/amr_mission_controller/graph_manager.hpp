#ifndef AMR_MISSION_CONTROLLER__GRAPH_MANAGER_HPP_
#define AMR_MISSION_CONTROLLER__GRAPH_MANAGER_HPP_

#include <string>
#include <vector>

#include "amr_mission_controller/graph_types.hpp"

class GraphManager
{
public:

    bool loadGraph(
        const std::string &nodesFile,
        const std::string &edgesFile);

    int findNearestNode(
        double x,
        double y) const;

    std::vector<int> findGraphPath(
        int startNodeId,
        int goalNodeId) const;

    const GraphNode *findNodeById(
        int id) const;

    const std::vector<GraphNode> &nodes() const;

    const std::vector<GraphEdge> &edges() const;

private:

    std::vector<GraphNode> graph_nodes_;
    std::vector<GraphEdge> graph_edges_;
};

#endif
