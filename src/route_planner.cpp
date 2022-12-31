#include "route_planner.h"
#include <algorithm>
#include <iostream>

/* 
NOTE: m_Model(model) - inititate the value of m_Model with model parameter - this way of initializing the variables is called initializer list  
REF: https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
*/
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find closest nodes to the startting and ending coordinates. 
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// calcuate hvalue - the distance of the node to the end point. 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// add all unvisited neighbors of the current node to the open list. 
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    /*
    NOTE: there is no difference between using any of the following because the neighbors are pointers.
        - for(auto neighbor: current_node->neighbors)
        - for(auto* neighbor: current_node->neighbors) -preferable - to stress that we are iterating on pointers
    using reference here provides the same functionality while also memory efficient (by avoiding unnecessary copying)
    */
    for (auto& neighbor: current_node->neighbors){
        if (neighbor->visited == false){
            neighbor->parent = current_node;
            neighbor->g_value  = current_node->g_value + neighbor->distance(*current_node);
            neighbor->h_value = this->CalculateHValue(neighbor);
            neighbor->visited = true;
            // add neighbor to open list - use emplace_back() instead of push_back() to avoid unnecessary copying
            this->open_list.emplace_back(neighbor);
        }
    }
}



// sort the open_list and return the next node, and remove it from open_list 
RouteModel::Node *RoutePlanner::NextNode() {
  /*
  REF - HOW TO SORT USING LAMBDA: https://stackoverflow.com/questions/5122804/how-to-sort-with-a-lambda
  */
    std::sort(this->open_list.begin(), this->open_list.end(),
        [](const auto& node1, const auto& node2) -> bool
        {
            return ((node1->g_value + node1->h_value) > (node2->g_value + node2->h_value));
        });

    RouteModel::Node* selectedNode = this->open_list.back();
    //delete the selected node
    this->open_list.pop_back();
    return selectedNode;
}



// take the final node, iteratively follow the chain of parents until the starting node is found, add distance, and return the vector of paths in correct order
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* activeNode = current_node;
   
    while (activeNode != this->start_node){
        float distanceToParent = activeNode->distance(*(activeNode->parent));
        distance += distanceToParent;
        path_found.push_back(*activeNode);
        activeNode = activeNode->parent;
    }
    path_found.push_back(*activeNode);
  
	//REFERENCE - HOW TO REVERSE THE VECTOR: https://stackoverflow.com/questions/8877448/how-do-i-reverse-a-c-vector
    // reverse vector, in that way the start node will be the first element. 
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale();  // Multiply the distance by the scale of the map to get meters 
  	std::cout << "PATH FOUND SIZE: " << path_found.size() << "\n";
  	return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    this->start_node->visited = true;
    current_node = this->start_node;
    // add the neighbors of the node to open_list, sort the list and return the next node until the search ends to the end_node.
  	while (current_node != end_node){
      this->AddNeighbors(current_node);
      current_node = this->NextNode();
    }
    //Store the final path in the m_Model.path attribute - 
    this->m_Model.path = this->ConstructFinalPath(current_node);
   
}