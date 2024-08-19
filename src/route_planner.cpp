#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    // Start node
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    // end node
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


void RoutePlanner::SortOpenList(std::vector<RouteModel::Node*>& open_list) {
    std::sort(open_list.begin(), open_list.end(),
        [this](const RouteModel::Node* node1, const RouteModel::Node* node2) {
            // compare f values 
            float f1 = node1->g_value + node1->h_value;
            float f2 = node2->g_value + node2->h_value;
            // in case the f values are the same, take the g value (closest to start instead)
            if (f1 == f2) {
                return node1->g_value < node2->g_value; 
            }
            // return in ascending order
            return f1 < f2;
        });
}
// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const* node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
    // - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    for (auto* node : current_node->neighbors) {
        // - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node);
        // - Use CalculateHValue below to implement the h-Value calculation.
        node->h_value = CalculateHValue(node);
        // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(node);
        node->visited = true;
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node* RoutePlanner::NextNode() {
    // - Sort the open_list according to the sum of the h value and g value.
    SortOpenList(open_list);
    // - Create a pointer to the node in the list with the lowest sum.
    auto current = open_list.front();  
    // - Remove that node from the open_list.
    open_list.erase(open_list.begin());  
    // - Return the pointer.
    return current;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
    // Create path_found vector
    this->distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    //set placeInPath to current/end node
    RouteModel::Node* placeInPath = current_node;
    path_found.push_back(*placeInPath);

    while (placeInPath != start_node) {
        // define parent
        RouteModel::Node* parent_node = placeInPath->parent;
        // increase distance
        distance += placeInPath->distance(*parent_node);
        //replace parent
        placeInPath = parent_node;
        // increase path
        path_found.push_back(*placeInPath);
    }

    // Reverse the vector to get the correct order
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = nullptr;

    // Add start_node
    open_list.push_back(start_node);
    // start node set to visited
    start_node->visited = true;

    while (!open_list.empty()) {
        //next node
        current_node = NextNode();

        // Check if we reached the end node
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else {
            // If not, explore neighbors
            AddNeighbors(current_node);
        }
    }
}