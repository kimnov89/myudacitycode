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
        i->g_value=i->distance(*current_node);
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

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    int i=0;
    float path=current_node->distance(*current_node->parent);
    path_found.push_back(*current_node);
    while(current_node->parent!=start_node){
        current_node=current_node->parent;
        path+=current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
    }

    distance = m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    while(current_node!= end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
        m_Model.path = ConstructFinalPath(current_node);
}