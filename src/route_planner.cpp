#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates and store into RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// CalculateHValue using the distance to the end_node by using distance method for Node objects
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value, add it to open_list and set the node's visited attribute to true.
    for(auto *node : (*current_node).neighbors) {
        node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);
        // Use CalculateHValue to implement the h-Value calculation.
        node->h_value = this->CalculateHValue(node);
        this->open_list.push_back(node);
        node->visited = true;
    }
}


// Sort the open_list according to the sum of the h value and g value.
bool Compare(RouteModel::Node *node_a, RouteModel::Node *node_b) {
    return (node_a->g_value+node_a->h_value) > (node_b->g_value+node_b->h_value);
}

// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), &Compare);
    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *next_node = this->open_list.back();
    // Remove that node from the open_list.
    this->open_list.pop_back();
    return next_node;
}


// ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // From current (final) node iteratively follow the chain of parents of nodes until starting node is found
    while(current_node != this->start_node) {
        // Add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    if(current_node->parent == nullptr)  path_found.push_back(*(this->start_node));
    // The returned vector should be in the correct order: the start node should be the first element, the end node should be the last element.
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
    // Mark start_node as visited and push it into the openlist.
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);
    
    RouteModel::Node *current_node = nullptr;
    // Until the open_list is not empty (i.e, we haven't run out of valid neighboring nodes)
    while(!this->open_list.empty()) {
        // Use the NextNode() method to sort the open_list and return the next node.
        current_node = this->NextNode();
        // When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
        if(current_node == this->end_node) {
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            this->m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // If not then expand the search to the current node's neighbors.
        else  this->AddNeighbors(current_node);
    }
}