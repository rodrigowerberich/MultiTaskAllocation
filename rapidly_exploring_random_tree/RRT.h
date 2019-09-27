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
    CollisionFunction m_collision_function;
    Point m_min_point;
    Point m_max_point;
    PointMap m_points;
    EdgeStorage m_edges;
    double m_max_step;
    
    Point generateNewCandidate();
    PointI nearestVertex(const Point& new_point);
    Point newConfiguration(PointI nearest_point, Point new_point);
public:
    RRT(Point origin, Point goal, CollisionFunction collision_function, Point min_point, Point max_point, double max_step);
    PointI grow();
    const PointMap& getVertices();
    const EdgeStorage& getEdges();
    // Point getVertex(PointI index);
};

template <typename CollisionFunction>
RRT<CollisionFunction>::RRT(Point origin, Point goal, CollisionFunction collision_function, Point min_point, Point max_point, double max_step):
    m_origin{origin},
    m_goal{goal},
    m_collision_function{collision_function},
    m_min_point{min_point},
    m_max_point{max_point},
    m_points{min_point.x, max_point.x, min_point.y, max_point.y, 10, 10},
    m_max_step{max_step}{
        m_points.insert(origin);
        
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::generateNewCandidate(){
    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(m_min_point.x, m_max_point.x);
    std::uniform_real_distribution<> dist_y(m_min_point.y, m_max_point.y);
    Point p = {dist_x(e2), dist_y(e2)}; 
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
        return new_point_i;
    }
    // return m_points.nearestPointIndex(new_point);
}

template <typename CollisionFunction>
Point RRT<CollisionFunction>::newConfiguration(PointI nearest_point, Point new_point){
    return new_point;
}

template <typename CollisionFunction>
PointI RRT<CollisionFunction>::grow(){
    Point new_point = generateNewCandidate();
    PointI nearest_point_index = nearestVertex(new_point);
    new_point = newConfiguration(nearest_point_index, new_point);
    PointI new_point_index = m_points.size();
    m_points.insert(new_point);
    m_edges.addEdge({nearest_point_index, new_point_index});
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

// template <typename CollisionFunction>
// Point RRT<CollisionFunction>::getVertex(PointI index){

// }

// template <typename CollisionFunction>
// class RRTSolver{
// private:
//     Point m_origin;
//     Point m_goal;
//     CollisionFunction m_collision_function;
//     Point m_min_point;
//     Point m_max_point;
//     Points m_solution;

// public:
//     RRTSolver(Point origin, Point goal, CollisionFunction collision_function, Point min_point, Point max_point);
//     Points solve();
//     Points getSolution();
// };

#endif //RRT_H__