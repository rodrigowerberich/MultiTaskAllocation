#include <DijkstraPathFinder.h>

namespace dijkstra{

PathFinder::PathFinder(PointMap point_map, EdgeStorage edge_storage, dijkstra::Result result):m_point_map{point_map},m_edge_storage{edge_storage},m_result{result}{}

// Points getPath(const Point& interest_point){
//     Points path;
//     path.insert(std::begin(path),interest_point);
//     PointI nearest_point_index = m_point_map.nearestPointIndex(interest_point);
//     path.insert(std::begin(path),m_point_map[nearest_point_index]);
//     PointI next_point_index = nearest_point_index;
//     while(std::get<1>(m_result[next_point_index])!=-1){
//         next_point_index = std::get<1>(m_result[next_point_index]);
//         path.insert(std::begin(path),m_point_map[next_point_index]);
//     }
//     return path;
// }
Points PathFinder::getPath(const Point& interest_point){
    Points path;
    path.insert(std::begin(path),interest_point);
    PointI nearest_point_index = m_point_map.nearestPointIndex(interest_point);
    if(nearest_point_index == -1){
        return Points();
    }
    double radius = Point::euclideanDistance(interest_point, m_point_map[nearest_point_index]);
    PointsI nearest_points_index = m_point_map.pointsInRangeByIndex(interest_point, 2*radius);
    double min_value = std::numeric_limits<double>::max();
    for(const auto& pi: nearest_points_index){
        double new_value = Point::euclideanDistance(interest_point, m_point_map[pi]) + std::get<0>(m_result[pi]);
        if(new_value < min_value){
            min_value = new_value;
            nearest_point_index = pi;
        }
    }
    path.insert(std::begin(path),m_point_map[nearest_point_index]);
    PointI next_point_index = nearest_point_index;
    while(std::get<1>(m_result[next_point_index])!=-1){
        next_point_index = std::get<1>(m_result[next_point_index]);
        path.insert(std::begin(path),m_point_map[next_point_index]);
    }
    return path;
}

}
