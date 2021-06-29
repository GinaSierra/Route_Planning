#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

     start_node = &m_Model.FindClosestNode(start_x, start_y);
     end_node = &m_Model.FindClosestNode(end_x, end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto neighbor : current_node->neighbors){
        if(!neighbor->visited){
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->distance(*neighbor)+current_node->g_value;
            open_list.push_back(neighbor);
            neighbor->visited = true; 
        }
    }
}

bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  float f1 = a->g_value + a->h_value;
  float f2 = b->g_value + b->h_value;
  return f1 > f2; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node * node = open_list.back();
    open_list.pop_back();
    return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node *node = current_node;
    path_found.push_back(*current_node);

    while(node->x != start_node->x && node->y != start_node->y){   
        distance += node->distance(*(node->parent));
        node = node->parent;
        path_found.insert(path_found.begin(),*node);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    // Initialize the starting node.
    start_node->visited = true;
    current_node = start_node;
    AddNeighbors(current_node);

    while (open_list.size() > 0) {
        // Get the next node
        current_node = NextNode();
        if(current_node->x==end_node->x && current_node->y==end_node->y)
            break;
        //  Expand search to current node's neighbors. 
        AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(current_node);

    return;
}