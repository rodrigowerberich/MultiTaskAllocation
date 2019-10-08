#ifndef BIRRTSOLVER_H__
#define BIRRTSOLVER_H__

#include <TrigDefinitions.h>
#include <PathSmoother.h>

template <template<class CollisionFunction> class RRTClass, class CollisionFunction>
class BiRRTSolver{
private:
    Point m_origin;
    Point m_goal;
    const CollisionFunction& m_is_in_obstacle;
    Point m_min_point;
    Point m_max_point;
    double m_max_step;
public:
    BiRRTSolver(Point origin, Point goal, const CollisionFunction& collision_function, Point min_point, Point max_point, double max_step = 0.0);
    Points solve();
};

template <template<class CollisionFunction> class RRTClass, class CollisionFunction>
BiRRTSolver<RRTClass, CollisionFunction>::BiRRTSolver(Point origin, Point goal, const CollisionFunction& collision_function, Point min_point, Point max_point, double max_step):
    m_origin{origin},
    m_goal{goal},
    m_is_in_obstacle{collision_function},
    m_min_point{min_point},
    m_max_point{max_point},
    m_max_step{max_step}
{
    
}

template <template<class CollisionFunction> class RRTClass, class CollisionFunction>
Points BiRRTSolver<RRTClass, CollisionFunction>::solve(){
    RRTClass<CollisionFunction> rrt1 {m_origin, m_goal, m_is_in_obstacle, m_min_point, m_max_point, m_max_step};
    RRTClass<CollisionFunction> rrt2 {m_goal, m_origin, m_is_in_obstacle, m_min_point, m_max_point, m_max_step};

    Points path1;
    Points path2;
    auto* Ta = &rrt1;
    auto* Tb = &rrt2;    

    int i=0;
    bool met = false;
    for(; !met ; i++){
        Ta->setRandomPercentage(1);
        Tb->setRandomPercentage(0);
        auto new_point_Ta_i = Ta->grow();
        if( new_point_Ta_i != -1 ){
            Tb->setGoal(Ta->getVertices()[new_point_Ta_i]);
            auto new_point_Tb_i = Tb->grow();
            if( new_point_Tb_i != -1 ){
                if ((Ta->getVertices()[new_point_Ta_i] - Tb->getVertices()[new_point_Tb_i]).absolute() < 0.1){
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

    Points path;
    path.insert(std::end(path), std::rbegin(path2), std::rend(path2)-1);
    path.insert(std::end(path), std::begin(path1), std::end(path1));
    return smoothPath(path, m_is_in_obstacle);
}

#endif //BIRRTSOLVER_H__