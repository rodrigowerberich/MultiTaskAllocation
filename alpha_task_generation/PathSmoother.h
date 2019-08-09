#ifndef PATHSMOOTHER_H__
#define PATHSMOOTHER_H__

#include <TrigDefinitions.h>
#include <deque>

template <typename CollisionFunction>
bool checkEdgeCollision(const Edge& edge, const CollisionFunction& collision_fn){
    const Point& a = edge[0];
    const Point& b = edge[1];
    Point c = b-a;
    int connection_count = 0;
    for(int i=0; i<100; i++){
        Point d = a + (c*i)/100;
        if(collision_fn(d.x, d.y)){
            return true;
        }
    }
    return false;
}

template <typename CollisionFunction>
Points smoothPath(const Points& path, const CollisionFunction& coll_fn){
    if (path.size() < 2) return Points();
    std::deque<Point> smoothed_path;
    smoothed_path.push_back(path.back());
    for(int start_path_point=0, curr_end_point=path.size()-1; start_path_point != curr_end_point;){
        for(int curr_path_point = start_path_point; curr_path_point < curr_end_point; curr_path_point++){
            if (!checkEdgeCollision({path[curr_path_point], path[curr_end_point]}, coll_fn)){
                curr_end_point = curr_path_point;
                smoothed_path.push_front(path[curr_path_point]);
                break;
            }
        }
    }
    return {std::begin(smoothed_path), std::end(smoothed_path)};
}

#endif //PATHSMOOTHER_H__