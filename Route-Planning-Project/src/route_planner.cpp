#include "route_planner.h"
#include <algorithm>


bool sortFunc (RouteModel::Node* i, RouteModel::Node* j) { 
  return ((i->g_value + i->h_value) > (j->g_value + j->h_value));
   }

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store then nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return((node)->distance(*end_node));
}


// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    (*current_node).FindNeighbors();
    for (RouteModel::Node* neighbour : current_node->neighbors ){
        neighbour->parent = current_node;
        neighbour->g_value = current_node->g_value + neighbour->distance(*current_node) ;
        neighbour->h_value = CalculateHValue(neighbour) ;
        this-> open_list.push_back(neighbour) ;
        neighbour->visited = true ;
    }

}


// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this-> open_list.begin(), this-> open_list.end(), sortFunc) ;
    RouteModel::Node* closest_node = this-> open_list.back() ;
    this-> open_list.pop_back() ;
    return(closest_node) ;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found ;
    while(current_node){
        path_found.push_back(*current_node) ;
        if(current_node-> parent){
            this-> distance += current_node->distance(*(current_node->parent)) ;}
        current_node = (*current_node).parent ;
    }
    std::reverse(path_found.begin(), path_found.end()) ;
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr ;
    this->start_node->visited = true ;
    this->open_list.push_back(this->start_node) ;
    while(!open_list.empty()) {
        current_node = this-> NextNode() ;
        if(current_node->distance(*(this->end_node)) == 0){
            m_Model.path = this-> ConstructFinalPath(current_node) ;

            return ;
        }
        AddNeighbors(current_node) ;
        
    }
}