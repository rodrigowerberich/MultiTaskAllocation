#include <RobotsPositionEdgeCollisionChecker.h>

// Private methods ----------------------------------------------------

bool RobotsPositionEdgeCollisionChecker::collisionPoint(const Point& p) const{
    return m_problem_representation.getObstructedArea()->containsPoint(p);
}
bool RobotsPositionEdgeCollisionChecker::hasVisibility(const Point& p1, const Point& p2) const{
    Point new_edge_vector = p2 - p1;
    int num_of_points = new_edge_vector.absolute()/m_check_size;
    for(int i = 0; i < num_of_points; i++){
        Point part_of_edge = new_edge_vector*(static_cast<double>(i+1)/static_cast<double>(num_of_points));
        Point new_position = p1 + part_of_edge;
        if( collisionPoint(new_position)){
            return false;
        }
    }
    return true;
}
bool RobotsPositionEdgeCollisionChecker::hasConnectionToRouter(const Point& p) const{
    return m_problem_representation.getConnectivityFunction()->containsPoint(p);
}
bool RobotsPositionEdgeCollisionChecker::checkPathCollisionAndMapConnection(
    const RobotsPosition& rp1, 
    const RobotsPosition& rp2,
    std::vector<Points>& robots_path_points,
    std::vector<std::vector<bool>>& robots_path_connection,
    std::vector<bool>& robots_fully_connected, 
    RobotPositionCollisionCheckVertices& collision_check_vertices,
    bool& is_any_robot_fully_connected,
    bool& are_all_robots_fully_connected) const{
    for(int robot_i = 0; robot_i < rp1.size(); robot_i++){
        robots_fully_connected[robot_i] = true;
        Point p1 = rp2[robot_i];
        Point p2 = rp1[robot_i];
        Point new_edge_vector = p2 - p1;
        int num_of_points = std::ceil(new_edge_vector.absolute()/m_check_size);
        for(int i = 0; i <= num_of_points; i++){
            Point part_of_edge = new_edge_vector*(static_cast<double>(i)/static_cast<double>(num_of_points));
            Point new_position = p1 + part_of_edge;
            if( collisionPoint(new_position)){
                return true;
            }
            robots_path_points[robot_i].push_back(new_position);
            auto connected = hasConnectionToRouter(new_position);
            robots_path_connection[robot_i].push_back(connected);
            collision_check_vertices.insert(robot_i, i);
            robots_fully_connected[robot_i] = robots_fully_connected[robot_i] &&  connected;
        }
        is_any_robot_fully_connected = is_any_robot_fully_connected || robots_fully_connected[robot_i];
        are_all_robots_fully_connected = are_all_robots_fully_connected && robots_fully_connected[robot_i];
    }
    return false;
}
bool RobotsPositionEdgeCollisionChecker::checkIfLastPositionIsInvalid(
    std::vector<Points>& robots_path_points,
    std::vector<std::vector<bool>>& robots_path_connection,
    RobotPositionCollisionCheckVertices& collision_check_vertices,
    EdgeStorage& visible_edges,
    EdgeStorage& not_visible_edges
) const{
    std::vector<int> connected;
    std::set<int> uncertain;
    for(int robot_i=0; robot_i < robots_path_points.size(); robot_i++){
        int last_position_index = robots_path_points[robot_i].size()-1;
        bool has_connection = robots_path_connection[robot_i][last_position_index];
        // std::cout << "Checking robot " << robot_i << std::endl;
        int robot_position_vertex = collision_check_vertices.getIndex(robot_i, last_position_index);                        
        if(has_connection){
            connected.push_back(robot_position_vertex);
        }else{
            uncertain.insert(robot_position_vertex);
            for(int other_robot_i=0; other_robot_i < robots_path_points.size(); other_robot_i++){
                if(robot_i != other_robot_i){
                    // std::cout << "Checking link with robot " << other_robot_i << std::endl;
                    int other_last_position_index = robots_path_points[other_robot_i].size()-1;
                    int other_robot_position_vertex = collision_check_vertices.getIndex(other_robot_i, other_last_position_index);
                    bool visible = visible_edges.containsEdge({robot_position_vertex, other_robot_position_vertex});
                    bool not_visible = not_visible_edges.containsEdge({robot_position_vertex, other_robot_position_vertex});
                    if( !visible && ! not_visible ){
                        visible = hasVisibility(robots_path_points[robot_i][last_position_index], robots_path_points[other_robot_i][other_last_position_index]);
                        if(visible){
                            visible_edges.addEdge({robot_position_vertex, other_robot_position_vertex});
                        }else{
                            not_visible_edges.addEdge({robot_position_vertex, other_robot_position_vertex});
                        }
                    }
                }
            }
            auto this_robot_connection_edges = visible_edges.getEdgesWith(robot_position_vertex);
            if(this_robot_connection_edges.empty()){
                // Lonely unconnected node configures unconnected node
                // std::cout << "Lonely node\n";
                return true;
            }
        }
    }
    if(connected.size() != robots_path_points.size()){
        for(int connected_i = 0; connected_i < connected.size(); connected_i++){
            int connected_robot_i = connected[connected_i];
            // std::cout << "Sending connection from " << connected_robot_i << std::endl;
            std::vector<int> remove_list;
            for(int uncertain_robot_i: uncertain){
                // std::cout << "Checking connection from " << connected_robot_i << " to " << uncertain_robot_i << std::endl; 
                if( visible_edges.containsEdge({connected_robot_i, uncertain_robot_i}) ){
                    // std::cout << uncertain_robot_i << " is now connected " << std::endl;
                    remove_list.push_back(uncertain_robot_i);
                    connected.push_back(uncertain_robot_i);
                }
            }
            for(int remove_i: remove_list){
                uncertain.erase(remove_i);
            }
        }
        if(!uncertain.empty()){
            // std::cout << "Last position is invalid\n";
            return true;
        }
    }
    return false;
}

// Public methods -----------------------------------------------------

RobotsPositionEdgeCollisionChecker::RobotsPositionEdgeCollisionChecker(const ProblemRepresentation& problem_representation, double check_size):
    m_problem_representation{problem_representation}, m_check_size{check_size}{
}
bool RobotsPositionEdgeCollisionChecker::operator()(const RobotsPosition& rp1, const RobotsPosition& rp2) const{
    std::vector<Points> robots_path_points(rp1.size());
    std::vector<std::vector<bool>> robots_path_connection(rp1.size());
    std::vector<bool> robots_fully_connected(rp1.size());
    RobotPositionCollisionCheckVertices collision_check_vertices;
    bool is_any_robot_fully_connected = false;
    bool are_all_robots_fully_connected = true;
    bool collides = checkPathCollisionAndMapConnection(rp1,rp2, robots_path_points, robots_path_connection, robots_fully_connected, collision_check_vertices, is_any_robot_fully_connected, are_all_robots_fully_connected);
    // std::cout << "Collides? " << collides << std::endl;
    if( collides ){
        // std::cout << "Robot path collides\n";
        return true;            
    }
    if(!is_any_robot_fully_connected){
        // std::cout << "No robot is fully inside connection area\n";
        return true;
    }
    if(are_all_robots_fully_connected){
        // std::cout << "All robots inside connection area\n";
        return false;
    }

    // Check if last position is valid
    EdgeStorage visible_edges;
    EdgeStorage not_visible_edges;
    bool lastPositionInvalid = checkIfLastPositionIsInvalid(robots_path_points, robots_path_connection, collision_check_vertices, visible_edges, not_visible_edges);
    if(lastPositionInvalid){
        return true;
    }
    std::vector<int> robot_connected_points;
    std::set<int> robot_uncertain_points;
    for(int robot_i = 0; robot_i < rp1.size(); robot_i++){
        // std::cout << "Robot " << robot_i << " is " << (robots_fully_connected[robot_i]?"connected":"not connected") << std::endl;
        for(int point_i = 0; point_i < robots_path_points[robot_i].size(); point_i++){
            auto robot_connection_vertex = collision_check_vertices.getIndex(robot_i, point_i);
            if( robots_path_connection[robot_i][point_i] ){
                robot_connected_points.push_back(robot_connection_vertex);
            }else{
                robot_uncertain_points.insert(robot_connection_vertex);
                for(int other_robot_i=0; other_robot_i < rp1.size(); other_robot_i++){
                    if(robot_i != other_robot_i){
                        for(int other_robot_point_i = 0; other_robot_point_i < robots_path_points[other_robot_i].size(); other_robot_point_i++){
                            // std::cout << "Connection from " << robot_i << ", " << point_i << " to " << other_robot_i << ", " << other_robot_point_i << std::endl;
                            int other_robot_connection_vertex = collision_check_vertices.getIndex(other_robot_i, other_robot_point_i);
                            bool is_inside_visible = visible_edges.containsEdge({robot_connection_vertex, other_robot_connection_vertex});
                            bool is_inside_not_visible = not_visible_edges.containsEdge({robot_connection_vertex, other_robot_connection_vertex});
                            if( !is_inside_visible && !is_inside_not_visible ){
                                bool visible = hasVisibility(robots_path_points[robot_i][point_i], robots_path_points[other_robot_i][other_robot_point_i]);
                                if(visible){
                                    visible_edges.addEdge({robot_connection_vertex, other_robot_connection_vertex});
                                }else{
                                    not_visible_edges.addEdge({robot_connection_vertex, other_robot_connection_vertex});
                                }
                            }
                        }
                    }
                }
                auto this_robot_position_connection_edges = visible_edges.getEdgesWith(robot_connection_vertex);
                if(this_robot_position_connection_edges.empty()){
                    // Lonely unconnected node configures unconnected node
                    // std::cout << "Lonely node\n";
                    return true;
                }
            }
        }
    }
    if(!robot_uncertain_points.empty()){
        for(int robot_connected_points_index = 0; robot_connected_points_index < robot_connected_points.size(); robot_connected_points_index++){
            int connected_robot_point_i = robot_connected_points[robot_connected_points_index];
            // std::cout << "Sending connection from " << connected_robot_point_i << std::endl;
            std::vector<int> remove_list;
            for(int uncertain_robot_point_i: robot_uncertain_points){
                // std::cout << "Checking connection from " << connected_robot_point_i << " to " << uncertain_robot_point_i << std::endl; 
                if( visible_edges.containsEdge({connected_robot_point_i, uncertain_robot_point_i}) ){
                    // std::cout << uncertain_robot_point_i << " is now connected " << std::endl;
                    remove_list.push_back(uncertain_robot_point_i);
                    robot_connected_points.push_back(uncertain_robot_point_i);
                }
            }
            for(int remove_i: remove_list){
                robot_uncertain_points.erase(remove_i);
            }
            if(robot_uncertain_points.empty()){
                break;
            }
        }
        if(!robot_uncertain_points.empty()){
            // std::cout << "Not all points are connected!\n";
            return true;
        }
    }
    return false;        
}