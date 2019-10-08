#ifndef RRT_H__
#define RRT_H__

#include <TrigDefinitions.h>
#include <PointMap.h>
#include <EdgeStorage.h>
#include <iostream>

template <typename CollisionFunction>
class RRT{
private:
    Point m_origin;
    Point m_goal;
    const CollisionFunction& m_is_in_obstacle;
    Point m_min_point;
    Point m_max_point;
    PointMap m_points;
    EdgeStorage m_edges;
    PointsI m_parents;
    double m_max_step;
    double m_random_percentage;
    
    Point generateRandomPoint();
    Point generateNewCandidate();
    PointI nearestVertex(const Point& new_point);
    Point newConfiguration(PointI nearest_point, Point new_point);
    Point stoppingConfiguration(PointI nearest_point, Point new_point);
public:
    RRT(Point origin, Point goal, const CollisionFunction& collision_function, Point min_point, Point max_point, double max_step = 0.0, double random_percentage=0.5);
    PointI grow();
    const PointMap& getVertices();
    const EdgeStorage& getEdges();
    void setGoal(const Point& goal);
    void setRandomPercentage(double random_percentage);
    void setStep(double step_size);
    Points getPathToRoot(const PointI& p_i);
};

template <typename CollisionFunction>
RRT<CollisionFunction>::RRT(Point origin, Point goal, const CollisionFunction& collision_function, Point min_point, Point max_point, double max_step, double random_percentage):
    m_origin{origin},
    m_goal{goal},
    m_is_in_obstacle{collision_function},
    m_min_point{min_point},
    m_max_point{max_point},
    m_points{min_point.x, max_point.x, min_point.y, max_point.y, 10, 10},
    m_max_step{max_step},
    m_random_percentage{random_percentage}{
        m_points.insert(origin);
        // Initialize root
        m_parents.push_back(-1);
        
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::generateRandomPoint(){
    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(m_min_point.x, m_max_point.x);
    std::uniform_real_distribution<> dist_y(m_min_point.y, m_max_point.y);
    Point p = {dist_x(e2), dist_y(e2)};
    return p;
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::generateNewCandidate(){
    static std::random_device rd;
    static std::mt19937 random_engine(rd());
    std::uniform_real_distribution<> dist_01(0,1);
    double choose_candidate_method_number = dist_01(random_engine);

    Point p;
    if( choose_candidate_method_number < m_random_percentage ){
        do{
            p = generateRandomPoint();
        }while( m_points.hasPointInRange(p, .2) );
    }else{
        p = m_goal;
    }
    return p;
}

template <typename CollisionFunction>
PointI RRT<CollisionFunction>::nearestVertex(const Point& new_point){
    double min_dist = std::numeric_limits<double>::max();
    EdgeI min_edge_i = {-1,-1};
    Point min_point;
    Point curr_point;
    double curr_value;
    for(const auto& edge_i: m_edges.toEdges() ){
        std::tie(curr_point, curr_value) = calculateClosestPointInSegment(new_point, {m_points[edge_i[0]], m_points[edge_i[1]]});
        if ( curr_value < min_dist ){
            min_dist = curr_value;
            min_edge_i = edge_i;
            min_point = curr_point;
        }
    }
    if((min_edge_i[0] == -1) && (min_edge_i[1] == -1)){
        return m_points.nearestPointIndex(new_point);
    }else if((min_point-m_points[min_edge_i[0]]).absolute() < 0.01){
        return min_edge_i[0];
    }else if((min_point-m_points[min_edge_i[1]]).absolute() < 0.01){
        return min_edge_i[1];
    }else{
        PointI new_point_i = m_points.size();
        m_points.insert(min_point);   
        m_edges.removeEdge(min_edge_i);
        m_edges.addEdge({new_point_i, min_edge_i[0]});
        m_edges.addEdge({new_point_i, min_edge_i[1]});

        if(m_parents[min_edge_i[0]] == min_edge_i[1]){
            m_parents.push_back(min_edge_i[1]);
            m_parents[min_edge_i[0]] = new_point_i;
        }else{
            m_parents.push_back(min_edge_i[0]);
            m_parents[min_edge_i[1]] = new_point_i;
        }
        return new_point_i;
    }
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::newConfiguration(PointI nearest_point, Point new_point){
    Point growth_vector = new_point - m_points[nearest_point];
    if( m_max_step < growth_vector.absolute() ){
        growth_vector = growth_vector*(m_max_step/growth_vector.absolute());
        return m_points[nearest_point]+growth_vector;   
    }
    return new_point;
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::stoppingConfiguration(PointI nearest_point, Point new_point){
    Point new_edge_vector = new_point - m_points[nearest_point];
    constexpr int NUM_OF_POINTS = 500;
    Point previous_position = m_points[nearest_point];
    for(int i = 0; i < NUM_OF_POINTS; i++){
        Point part_of_edge = new_edge_vector*(static_cast<double>(i+1)/static_cast<double>(NUM_OF_POINTS));

        auto length = part_of_edge.absolute();
        Point new_position = m_points[nearest_point] + part_of_edge;
        if( m_is_in_obstacle(new_position.x, new_position.y) ){
            return previous_position;
        }
        if( (length >= m_max_step) && (m_max_step != 0.0) ){
            return new_position;
        }
        previous_position = new_position;
    }
    return new_point;
}


template <typename CollisionFunction>
PointI RRT<CollisionFunction>::grow(){
    Point new_point = generateNewCandidate();
    PointI nearest_point_index = nearestVertex(new_point);
    new_point = stoppingConfiguration(nearest_point_index, new_point);
    if( (new_point - m_points[nearest_point_index]).absolute() < 0.01 ){
        return -1;
    }
    PointI new_point_index = m_points.size();
    m_points.insert(new_point);
    m_edges.addEdge({nearest_point_index, new_point_index});
    m_parents.push_back(nearest_point_index);
    return new_point_index;
}

template <typename CollisionFunction>
const PointMap& RRT<CollisionFunction>::getVertices(){
    return m_points;
}

template <typename CollisionFunction>
const EdgeStorage& RRT<CollisionFunction>::getEdges(){
    return m_edges;
}

template <typename CollisionFunction>
void RRT<CollisionFunction>::setGoal(const Point& goal){
    m_goal = goal;
}

template <typename CollisionFunction>
void RRT<CollisionFunction>::setRandomPercentage(double random_percentage){
    m_random_percentage = random_percentage;
}

template <typename CollisionFunction>
void RRT<CollisionFunction>::setStep(double step_size){
    m_max_step = step_size;
}

template <typename CollisionFunction>
Points RRT<CollisionFunction>::getPathToRoot(const PointI& p_i){
    Points path;
    PointI current_p_i = p_i;
    do {
        path.push_back(m_points[current_p_i]);
        current_p_i = m_parents[current_p_i];
    }while(m_parents[current_p_i] != -1);
    path.push_back(m_points[current_p_i]);
    return path;
}

#endif //RRT_H__