#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node=&m_Model.FindClosestNode(start_x,start_y);
    end_node=&m_Model.FindClosestNode(end_x,end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* i: current_node->neighbors){
        i->parent=current_node;
        i->h_value= CalculateHValue(i);
        i->g_value=i->distance(*current_node)+current_node->g_value;
        i->visited=true;
        open_list.push_back(i);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),[](RouteModel::Node *nd1, RouteModel::Node *nd2){
       return nd1->g_value+nd1->h_value > nd2->g_value+nd2->h_value;
    });
    RouteModel::Node *ptr_node = open_list.back();
    if(!open_list.empty()) {
        open_list.pop_back();
    }
    return ptr_node;

}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    if (current_node == nullptr)
        return {};

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != start_node){
        path_found.push_back(*current_node);
        if(current_node->parent != nullptr) {
            distance += current_node->distance(*current_node->parent);
        }
        current_node=current_node->parent;
    }

    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    if (start_node == nullptr)
        abort();

    RouteModel::Node *path_node=start_node;
    start_node->visited=true;
    open_list.push_back(path_node);
    while(!open_list.empty()) {
        path_node = NextNode();
        if(path_node==end_node){
            m_Model.path = ConstructFinalPath(path_node);
            break;
        }else{
            AddNeighbors(path_node);
        }
    }

}