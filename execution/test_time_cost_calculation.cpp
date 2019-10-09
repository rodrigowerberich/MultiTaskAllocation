#include "test_time_cost_calculation.h"
#include <iostream>
#include <ProblemRepresentation.h>
#include <AlphaTasksParser.h>
#include <sys/stat.h>
#include <TrigDefinitions.h>
#include <RRT.h>
#include <GnuPlotRenderer.h>
#include <chrono>
#include <BiRRTSolver.h>
#include <PathSmoother.h>
#include <RRTTemplate.h>
#include <RRTSolverTemplate.h>
#include <unordered_map>
#include <map>
#include <EdgeStorage.h>

static inline std::string generate_alpha_task_file_name(const std::string& file_name){
    std::string alpha_file_name = file_name;
    return alpha_file_name.replace(std::begin(alpha_file_name)+alpha_file_name.find_last_of('.'), std::end(alpha_file_name), ".alpha.json");
}

static inline bool file_exists(const std::string& name){
    struct stat buffer;
    return ( stat (name.c_str(), &buffer) == 0 );
}

static inline void warn_alpha_task_file_does_not_exist(const std::string& alpha_file_name){
    std::cout << "Missing alpha task file\n";
    std::cout << alpha_file_name << " not found!\n";
}

static inline void warn_error_in_problem_representation_file(const ProblemRepresentation& problemRepresentation){
    std::cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << std::endl;
    std::cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << std::endl;
}

static inline void warn_error_in_alpha_task_file(const AlphaTasksParser& alpha_tasks_parser){
    std::cout << "\033[1;31mSomething is wrong with the alpha tasks file\033[0m" << std::endl;
    std::cout << "\033[1;31m"<< alpha_tasks_parser.getErrorMessage() << "\033[0m" << std::endl;
}

static inline void warn_task_does_not_exist(const std::string& task_name ){
    std::cout << "\033[1;31mTask with name \"" << task_name << "\" does not exist in this problem!\033[0m" << std::endl;
}

static inline Points get_complete_alpha_task_position(ProblemRepresentation& problemRepresentation, AlphaTasks& alpha_tasks, std::string task_name){
    Points complete_task_points;
    for(const auto& alpha_task:alpha_tasks){
        if( alpha_task.getTask() == task_name ){
            for(const auto& position: alpha_task){
                complete_task_points.push_back(position);
            }
        }
    }
    complete_task_points.push_back(problemRepresentation.getTasks()->getTask(task_name)->getPosition());
    return complete_task_points;
}

static double calculate_path_cost(const Points& path){
    double cost = 0;
    for(size_t i=0; i < path.size()-1;i++){
        cost += Point::euclideanDistance(path[i], path[i+1]);
    }
    return cost;
}

static Points calculate_path_from_robot_to_point(const std::unique_ptr<Robot>& robot, const Point& task, const ProblemRepresentation& problemRepresentation){
    const auto & search_area = problemRepresentation.getSearchArea();
    const auto& bounding_box = search_area->getDrawable()->getBoundingBox();
    BiRRTSolver<RRT, ObstructedArea> solver(robot->getPosition(), task, (*problemRepresentation.getObstructedArea()), {bounding_box.lower_left_x, bounding_box.lower_left_y}, {bounding_box.top_right_x, bounding_box.top_right_y}, 0.5);
    Points path = solver.solve();
    return path;          
}

static Points calculate_best_of_n_paths_from_robot_to_point(const std::unique_ptr<Robot>& robot, const Point& point, const ProblemRepresentation& problemRepresentation, size_t n){
    Points path;
    double best_cost = std::numeric_limits<double>::max();
    for(size_t i=0; i < n; i++){
        Points current_path = calculate_path_from_robot_to_point(robot, point, problemRepresentation);
        double current_cost = calculate_path_cost(current_path);
        if(current_cost < best_cost){
            path = current_path;
            best_cost = current_cost;
        }
    } 
    return path;
}


using CostMap = std::tuple<std::vector<std::vector<double>>, Points, std::vector<std::string>, std::vector<std::vector<Points>>>;
static CostMap calculate_cost_map(const Points& tasks, const ProblemRepresentation& problemRepresentation){
    std::vector<std::string> robots_names;
    std::vector<std::vector<double>> costs;
    std::vector<std::vector<Points>> paths;
    int robot_number = 0;
    for(const auto& robot: (*problemRepresentation.getRobots())){
        robots_names.push_back(robot->getName());
        costs.push_back(std::vector<double>());
        paths.push_back(std::vector<Points>());
        for(const auto& task: tasks){
            Points path = calculate_best_of_n_paths_from_robot_to_point(robot, task, problemRepresentation, 5);
            std::cout << "Robot " << robot_number << " called " << robots_names[robot_number] << " doing task " << task << std::endl;
            paths[robot_number].push_back(path);
            costs[robot_number].push_back(calculate_path_cost(path));
        }   
        robot_number++;
    }
    return std::make_tuple(costs, tasks, robots_names, paths);
}


static void calculate_minimun_cost_recursive(int num_of_task, std::set<int> choices, std::vector<int>& current_choice, const std::vector<std::vector<double>>& costs, std::tuple<std::vector<int>, double>& min_cost){
    if(num_of_task > 0){
        num_of_task--;
        for(int choice: choices){
            std::set<int> new_choices(choices);
            current_choice.push_back(choice);
            new_choices.erase(choice);
            calculate_minimun_cost_recursive(num_of_task, new_choices, current_choice, costs, min_cost);
            current_choice.pop_back();
        }
    }else{
        std::cout << "Choices: ";
        double curr_cost = 0;
        int task = 0;
        for(int robot: current_choice){
            curr_cost += costs[robot][task];
            std::cout << robot << " ,";
            task++;
        }
        std::cout << " cost = "<< curr_cost <<" ----\n";
        if(curr_cost < std::get<1>(min_cost)){
            std::get<0>(min_cost) = current_choice;
            std::get<1>(min_cost) = curr_cost;
        }
    }
}

static std::tuple<std::vector<int>, double> calculate_minimun_cost(const CostMap& costmap){
    const std::vector<std::vector<double>>& costs = std::get<0>(costmap);
    std::tuple<std::vector<int>, double> min_cost = std::make_tuple(std::vector<int>(), std::numeric_limits<double>::max());
    std::set<int> robots;
    std::set<int> tasks;
    for(size_t i=0; i < costs.size(); i++){
        robots.insert(i);
    }
    for(size_t i=0; i < costs[0].size(); i++){
        tasks.insert(i);
    }
    std::vector<int> curr_choice;
    calculate_minimun_cost_recursive(tasks.size(), robots, curr_choice, costs, min_cost);
    std::cout << std::get<1>(min_cost) << std::endl;
    return min_cost;
}

#include <RobotsPosition.h>
#include <RandomRobotsPositionGenerator.h>

static int nearestPointRp(const std::vector<RobotsPosition>& points, const RobotsPosition& point){
    double min_distance = std::numeric_limits<double>::max();
    int min_point_index = -1;
    for(auto point_index=0; point_index < points.size(); point_index++){
        double distance = RobotsPosition::distance(point, points[point_index]);
        if(distance < min_distance){
            min_distance = distance;
            min_point_index = point_index;
        }
    }
    return min_point_index;
}

static RobotsPosition stepRp(const RobotsPosition& original, const RobotsPosition& source, double step_size){
    RobotsPosition new_robots_position(original.size());
    for(int robot_i = 0; robot_i < original.size(); robot_i++){
        Point growth_vector = original[robot_i] - source[robot_i];
        if( step_size < growth_vector.absolute() ){
            growth_vector = growth_vector*(((double)step_size)/growth_vector.absolute());   
            new_robots_position[robot_i] = source[robot_i]+growth_vector;
        }else{
            new_robots_position[robot_i] = original[robot_i];
        }
    }
    return new_robots_position;
}

class RobotPositionCollisionCheckVertices{
private:
    typedef std::tuple<int, int> key_t;
    typedef std::map<const key_t, int> map_t;
    std::vector<std::tuple<int, int>> m_vertices;
    map_t m_reverse_search_engine;

public:
    bool insert(std::tuple<int, int> rp_i){
        if( contains(rp_i) ){
            return false;
        }
        int new_index = m_vertices.size();
        m_vertices.push_back(rp_i);
        m_reverse_search_engine[rp_i] = new_index;
        return true;
    }

    bool insert(int r_i, int p_i){
        return insert(std::make_tuple(r_i, p_i));
    }
    
    bool contains(std::tuple<int,int> rp_i) const{
        return (m_reverse_search_engine.find(rp_i) != m_reverse_search_engine.end());
    }
    bool contains(int r_i, int p_i) const{
        return contains(std::make_tuple(r_i, p_i));
    }
    int getIndex(std::tuple<int,int> rp_i) const{
        if(contains(rp_i)){
            return m_reverse_search_engine.at(rp_i);
        }else{
            return -1;
        }
    }
    int getIndex(int r_i, int p_i) const{
        return getIndex(std::make_tuple(r_i, p_i));
    }
    const std::tuple<int, int>& operator[](int i) const{
        return m_vertices[i];
    }
    size_t size() const{
        return m_vertices.size();
    }
    std::vector<std::tuple<int,int>>::const_iterator begin() const{
        return m_vertices.begin();
    }
    std::vector<std::tuple<int,int>>::const_iterator end() const{
        return m_vertices.end();
    }
};

class CollisionEdge{
private:
    const ProblemRepresentation& m_problem_representation;
    double m_check_size;
    bool collisionPoint(const Point& p) const{
        return m_problem_representation.getObstructedArea()->containsPoint(p);
    }
    bool hasVisibility(const Point& p1, const Point& p2) const{
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
    bool hasConnectionToRouter(const Point& p) const{
        return m_problem_representation.getConnectivityFunction()->containsPoint(p);
    }
    bool checkPathCollisionAndMapConnection(
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
    bool checkIfLastPositionIsInvalid(
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
public:
    CollisionEdge(const ProblemRepresentation& problem_representation, double check_size):
        m_problem_representation{problem_representation},
        m_check_size{check_size}{}
    bool operator()(const RobotsPosition& rp1, const RobotsPosition& rp2) const{
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
};

static bool collision_pointRp(const Point& p){
    return false;
}

static bool collision_edgeRp(const RobotsPosition& rp1, const RobotsPosition& rp2){
    for(int robot_i = 0; robot_i < rp1.size(); robot_i++){
        Point p1 = rp1[robot_i];
        Point p2 = rp2[robot_i];
        Point new_edge_vector = p2 - p1;
        constexpr int NUM_OF_POINTS = 500;
        for(int i = 0; i < NUM_OF_POINTS; i++){
            Point part_of_edge = new_edge_vector*(static_cast<double>(i+1)/static_cast<double>(NUM_OF_POINTS));
            Point new_position = p1 + part_of_edge;
            if( collision_pointRp(new_position) ){
                return true;
            }
        }
    }
    return false;
}

static void drawConfiguration(const RobotsPosition& rp, const drawable::Color color, const GnuPlotRenderer& renderer){
    Points p1s;
    Points p2s;
    for(size_t i=0; i < rp.size(); i++){
        renderer.drawNamedPoint(drawable::NamedPoint(rp[i], std::to_string(i), color));
        if( i < (rp.size()-1)){
            p1s.push_back(rp[i]);
            p2s.push_back(rp[i+1]);
        }
    }
    // renderer.drawLines({ p1s, p2s, color });
}

static void drawConfigurationEdge(const RobotsPosition& rp1, const RobotsPosition& rp2 , const drawable::Color color, const GnuPlotRenderer& renderer){
    Points p1s;
    Points p2s;
    for(size_t i=0; i < rp1.size(); i++){
        renderer.drawLine(drawable::Line(rp1[i], rp2[i], color));
    }
}

static void drawConfigurationEdgeMulticolor(const RobotsPosition& rp1, const RobotsPosition& rp2 , const std::vector<drawable::Color> colors, const GnuPlotRenderer& renderer){
    Points p1s;
    Points p2s;
    for(size_t i=0; i < rp1.size(); i++){
        renderer.drawLine(drawable::Line(rp1[i], rp2[i], colors[i]));
    }
}



using RRTMutiple = RRTTemplate<RobotsPosition, RandomRobotsPositionGenerator, int(const std::vector<RobotsPosition>&, const RobotsPosition&), RobotsPosition(const RobotsPosition&, const RobotsPosition& source, double), double , CollisionEdge>;

bool test_edge(const RobotsPosition& rp1, const RobotsPosition& rp2, const CollisionEdge& collision_function, const drawable::Color& color, GnuPlotRenderer& renderer){
    std::cout << rp1 << std::endl;
    std::cout << rp2 << std::endl;
    bool collides = collision_function(rp2, rp1); 
    if(!collides){
        drawConfiguration(rp2, color, renderer);
        drawConfiguration(rp1, color, renderer);
        drawConfigurationEdge(rp1, rp2, drawable::Color::Red, renderer);
    }
    std::cout << (collides?"Invalid":"Valid") << std::endl;
    return collides;
}

void test_time_cost_calculation(std::tuple <std::string, std::string> values){
    using namespace std::chrono;

    
    auto file_name = std::get<0>(values);
    auto task_name = std::get<1>(values);

    auto alpha_file_name = generate_alpha_task_file_name(file_name);
    if( !file_exists(alpha_file_name) ){
        warn_alpha_task_file_does_not_exist(alpha_file_name);
        return;
    }
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        warn_error_in_problem_representation_file(problemRepresentation);
        return;
    }
    if(problemRepresentation.getTasks()->getTask(task_name) == nullptr){
        warn_task_does_not_exist(task_name);
        return;
    }
    AlphaTasksParser alpha_tasks_parser{alpha_file_name, problemRepresentation};
    if(!alpha_tasks_parser.isValid()){
        warn_error_in_alpha_task_file(alpha_tasks_parser);
        return;
    }
    AlphaTasks alpha_tasks = alpha_tasks_parser.getAlphaTasks();

    Points complete_task_points = get_complete_alpha_task_position(problemRepresentation, alpha_tasks, task_name);
    std::cout << complete_task_points << std::endl;

    GnuPlotRenderer renderer;
    renderer.holdOn();
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    renderer.setAxisRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);


    problemRepresentation.draw(renderer);

    // auto cost_map = calculate_cost_map(complete_task_points, problemRepresentation);
    // auto min_cost = calculate_minimun_cost(cost_map);
    // auto min_paths = std::get<0>(min_cost);
    // auto paths = std::get<3>(cost_map);
    // for(int i = 0; i < min_paths.size(); i++){
    //     const auto& path = paths[min_paths[i]][i];
    //     for(int p_i = 0; p_i < (path.size()-1); p_i++){
    //         renderer.drawLine({path[p_i], path[p_i+1]});
    //     }
    // }


    auto origin = RobotsPosition(problemRepresentation.getRobots()->size());
    int i=0; 
    for(const auto& robot: (*problemRepresentation.getRobots())){
        origin[i] = robot->getPosition();
        i++;
    }
    auto goal = RobotsPosition(problemRepresentation.getRobots()->size(), complete_task_points);
    std::cout << origin << std::endl;
    std::cout << goal << std::endl;
    auto generator = RandomRobotsPositionGenerator(3, bounding_box.lower_left_x, bounding_box.top_right_x, bounding_box.lower_left_y, bounding_box.top_right_y);
    auto collision_checker = CollisionEdge(problemRepresentation, 0.05);
    RRTMutiple rrt( origin, goal, generator, nearestPointRp, stepRp, 0.5, collision_checker, 0.5);
    RRTMutiple rrt2( goal, origin, generator, nearestPointRp, stepRp, 0.5, collision_checker, 0.35);
    auto start = high_resolution_clock::now(); 
    auto found = rrt_solver::findBi(rrt, rrt2, goal, RobotsPosition::distance, 0.1);
    auto stop = high_resolution_clock::now(); 
    auto duration = duration_cast<microseconds>(stop - start); 
    std::cout << "Took " << duration.count() << " microseconds\n";

    for(int i=0; i < found.size(); i++){
        std::cout << found[i] << ", ";
        // drawConfiguration(rrt.getVertices()[found[i]], drawable::Color::Blue, renderer);
        if( i < (found.size()-1) ){
            // drawConfigurationEdge(found[i], found[i+1], drawable::Color::Red, renderer);
            drawConfigurationEdgeMulticolor(found[i], found[i+1], {drawable::Color::Red, drawable::Color::Green, drawable::Color::Blue}, renderer);
        }
    }

    // auto generator = RandomRobotsPositionGenerator(3, bounding_box.lower_left_x, bounding_box.top_right_x, bounding_box.lower_left_y, bounding_box.top_right_y);
    // auto collision_checker = CollisionEdge(problemRepresentation, 0.05);

    // auto origin0 = RobotsPosition{3, {{-3,-4}, {-3,1}, {3,1}}}; 
    // auto point0 = RobotsPosition{3, {{-3,-3.5}, {-3,2.5}, {2.5,1}}};
    // auto origin1 = RobotsPosition{3, {{0.5,2.7}, {0.5,2.3}, {0.5,1.9}}};
    // auto point1 = RobotsPosition{3, {{1,2.7}, {1,2.3}, {1,1.9}}};
    // auto origin2 = RobotsPosition{3, {{-6,2.3}, {-5.5,0}, {-6,-2}}};
    // auto point2 = RobotsPosition{3, {{-4,2.3}, {-5.8,0}, {-6,-4}}};
    // auto origin3 = RobotsPosition{3, {{-2,-4}, {-1.8,-4}, {4,-4}}};
    // auto point3 = RobotsPosition{3, {{-2,-3.5}, {-1.8,-3.5}, {4,-3.5}}};
    // auto origin4 = RobotsPosition{3, {{0.5,-1.5}, {-3,-1.5}, {5,-2}}};
    // auto point4 = RobotsPosition{3, {{2.4,-1.5}, {-3,-1}, {5,-3}}}; 
    // auto origin5 = RobotsPosition{3, {{-1.7,0.3}, {0.5, -0.3}, {-1,-1.3}}};
    // auto point5 = RobotsPosition{3, {{-1.5,0.3}, {6, -0.3}, {6,-1.3}}}; 
    // auto origin6 = RobotsPosition{3, {{-6,0.5}, {-4.7, 2.6}, {-6,3}}};
    // auto point6 = RobotsPosition{3, {{-5.5,0.5}, {-4.7, 4}, {-5.5,3}}}; 

    // auto point = RobotsPosition{3, {{-1.76219,-0.647357}, {2,-0.5}, {-1.59339,0.886397}}}; //generator(1);

    // point = stepRp(point, origin, 1);
    // std::cout << "Test 0 - Should fail, obstacle collision in path of robot 1\n";
    // test_edge(origin0, point0, collision_checker, drawable::Color::Green, renderer);
    // std::cout << "Test 1 - Should fail, all robots outside connection area\n";
    // test_edge(origin1, point1, collision_checker, drawable::Color::Blue, renderer);
    // std::cout << "Test 2 - Should pass, all robots inside connection area\n";
    // test_edge(origin2, point2, collision_checker, drawable::Color::Black, renderer);
    // std::cout << "Test 3 - Should fail, robot 2 can't see anybody in its final configuration and is outside connection\n";
    // test_edge(origin3, point3, collision_checker, drawable::Color::Orange, renderer);
    // std::cout << "Test 4 - Should pass\n";
    // test_edge(origin4, point4, collision_checker, drawable::Color::Brown, renderer);
    // std::cout << "Test 5 - Should fail, robot 2 can not perform its path without losing connection\n";
    // test_edge(origin5, point5, collision_checker, drawable::Color::Cyan, renderer);
    // std::cout << "Test 6 - Should fail, robots 1 and 2 with end points outside connection area and no visibility\n";
    // test_edge(origin6, point6, collision_checker, drawable::Color::DarkBlue, renderer);

    // for(int i=0; i < 10; i++){
    //     std::cout << "---------------------" << i << "-------------------------\n";
    //     // auto origin = generator(2*i);
    //     auto destiny = generator(2*i+1);
    //     destiny = stepRp(destiny, origin, 1);
    //     test_edge(origin, destiny, collision_checker, static_cast<drawable::Color>(i), renderer);
    // }

    // std::cout << point << std::endl;
    // drawConfiguration(origin, drawable::Color::Blue, renderer);
    // drawConfiguration(point, drawable::Color::Blue, renderer);
    // drawConfigurationEdge(origin, point, drawable::Color::Red, renderer);
    // std::cout << (collision_checker(point, origin)?"Invalid":"Valid") << std::endl;
    renderer.drawPoints({complete_task_points, drawable::Color::DarkBlue});

}
