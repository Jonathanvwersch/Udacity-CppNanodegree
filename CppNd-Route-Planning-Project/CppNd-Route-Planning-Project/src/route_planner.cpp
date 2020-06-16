#include "route_planner.h"


// Set Constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closest nodes to starting and ending nodes
  	start_node = &m_Model.FindClosestNode(start_x, start_y); 
    end_node =&m_Model.FindClosestNode(end_x, end_y);

}


// Calculate Heuristic function
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
	return node->distance(*end_node);
    
}

// Add neighbouring nodes to current node Node
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	// Populate current_node with all its neighbors
	current_node->FindNeighbors();
  
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
 	for (RouteModel::Node *neighbor : current_node->neighbors){
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->parent = current_node;
      neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
      
    }
  
    // - For each node in current_node add the neighbor to open_list and set the node's visited attribute to true.
	for (RouteModel::Node *neighbor : current_node->neighbors){
     
      	open_list.push_back(neighbor);
		neighbor->visited = true;
    }
}


// Sort the open list and return the next node (node with lowest g+h value)
RouteModel::Node *RoutePlanner::NextNode() {
	
  	// Sort the open_list according to the sum of the h value and g value.
  	for (int i = 0; i < open_list.size(); ++i){
      
      for (int j = 0; j <(open_list.size()) - i - 1; ++j){
      	if ((open_list[j+1]->h_value + open_list[j+1]->g_value) > (open_list[j]->h_value + open_list[j]->g_value)){
      		auto temp = open_list[j];
       		open_list[j] = open_list[j+1];
         	open_list[j+1] = temp;  
       }
         
      }
      
      // Create a pointer to the node in the list with the lowest sum.
      RouteModel::Node* next_node = open_list.back();
      
      // Remove that node from the open_list.
      open_list.pop_back();
      
      // Return the pointer.
      return next_node;
    }
         
}


// Return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    // For each node in the chain, add the distance from the node to its parent, to the distance variable
    // and organise the vector such that the start node is the first element
  	while (current_node->parent != nullptr){
      distance += current_node->distance(*(current_node->parent));
      path_found.insert(path_found.begin(),*current_node);
      current_node = current_node->parent;
    }
	
    // Add starting node, which is not included in while loop, to path_found vector
  	path_found.insert(path_found.begin(), *current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Use A* Search algorithm to compute path
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

  	// Add starting node to open_list 
  	start_node->visited = true;
  	open_list.push_back(start_node);

  	// Loop through while the size of open list is greater than 	
    while (open_list.size() > 0) {  
  		current_node = NextNode(); // compute next node
       
      	// if h value of the current node is 0 (signifying end node), construct final path
     	if (current_node->h_value == 0)
    		m_Model.path = ConstructFinalPath(current_node); 
     		      
     	// Otherwise run AddNeighbors method and continue loop
    	AddNeighbors(current_node);
    }
 	
}