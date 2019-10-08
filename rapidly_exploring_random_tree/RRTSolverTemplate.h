#ifndef RRTSOLVERTEMPLATE_H__
#define RRTSOLVERTEMPLATE_H__

#include <vector>
#include <deque>

namespace rrt_solver{

template <typename RRTClass, typename ConfigurationPoint, typename DistanceFunction>
std::deque<ConfigurationPoint> find(RRTClass& rrt, const ConfigurationPoint& goal, const DistanceFunction& distance_function, double threshhold){
    while(true){
        auto i = rrt.grow();
        if(i != -1){
            auto last_point = rrt.getVertices()[i];
            if( distance_function(goal, last_point) < threshhold ){
                return rrt.getPathToRoot(i);
            }
        }
    }
}

template <typename RRTClass, typename ConfigurationPoint, typename DistanceFunction>
std::vector<ConfigurationPoint> findBi(RRTClass& rrt1, RRTClass& rrt2, const ConfigurationPoint& goal, const DistanceFunction& distance_function, double threshhold){
    std::deque<ConfigurationPoint> path1;
    std::deque<ConfigurationPoint> path2;

    auto* Ta = &rrt1;
    auto* Tb = &rrt2;

    int i=0;
    bool met = false;
    for(; !met; i++){
        Ta->setGoalSearchPercentage(0);
        Tb->setGoalSearchPercentage(1);
        auto new_point_Ta_i = Ta->grow();
        if( new_point_Ta_i != -1 ){
            Tb->setGoal(Ta->getVertices()[new_point_Ta_i]);
            auto new_point_Tb_i = Tb->grow();
            if( new_point_Tb_i != -1 ){
                if(  distance_function(Ta->getVertices()[new_point_Ta_i], Tb->getVertices()[new_point_Tb_i]) < threshhold ){
                    path1 = Ta->getPathToRoot(new_point_Ta_i);
                    path2 = Tb->getPathToRoot(new_point_Tb_i);
                    met = true;
                    break;
                }
            }
        }
        if( Ta->getVertices().size() > Tb->getVertices().size() ){
            std::swap(Ta, Tb);
        }
    }
    std::vector<ConfigurationPoint> path;
    path.insert(std::end(path), std::begin(path1), std::end(path1)-1);
    path.insert(std::end(path), std::rbegin(path2), std::rend(path2));
    return path;
}

}

#endif //RRTSOLVERTEMPLATE_H__