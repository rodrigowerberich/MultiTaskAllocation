#ifndef RRTSTARTEMPLATE_H__
#define RRTSTARTEMPLATE_H__

#include <vector>
#include <array>
#include <random>

template <typename ConfigurationPoint, typename RandomConfigurationGenerator, typename NearestFunction, typename StepFunction, typename StepType, typename CollisionFunction, typename NearFunction, typename ChooseParentFunction, typename RewireFunction>
class RRTStarTemplate{
private:
    ConfigurationPoint m_goal;
    std::vector<ConfigurationPoint> m_vertices;
    std::vector<std::array<int,2>> m_edges;
    std::vector<double> m_costs;
    const RandomConfigurationGenerator& m_random_generator;
    const NearestFunction& m_nearest;
    const StepFunction& m_step_function;
    StepType m_step_size;
    const CollisionFunction& m_collision_function;
    const NearFunction& m_near_function;
    const ChooseParentFunction& m_choose_parent_function;
    const RewireFunction& m_rewire_function;
    size_t m_growth_operation_number;
    double m_goal_search_percentage;

    bool chooseGoal() const{
        return ((((double)rand())/RAND_MAX) < m_goal_search_percentage);

    }
    void addOrigin(const ConfigurationPoint& origin){
        m_vertices.push_back(origin);
        m_costs.push_back(0);
    }
    int addVertixAndEdge(const ConfigurationPoint& point, int parent_index){
        int new_point_index = m_vertices.size();
        m_vertices.push_back(point);
        m_edges.push_back({parent_index, new_point_index});
        return new_point_index;
    }
public:
    RRTStarTemplate(const ConfigurationPoint& origin, const ConfigurationPoint& goal, const RandomConfigurationGenerator& random_generator, const NearestFunction& nearest, const StepFunction& step_function,  StepType step_size, const CollisionFunction& collision_function, const NearFunction& near_function, const ChooseParentFunction& choose_parent_function, const RewireFunction& rewire, double goal_search_percentage):
        m_goal{goal},
        m_vertices{},
        m_edges{},
        m_random_generator{random_generator},
        m_nearest{nearest},
        m_step_function{step_function},
        m_step_size{std::move(step_size)},
        m_collision_function{collision_function},
        m_near_function{near_function},
        m_choose_parent_function(choose_parent_function),
        m_rewire_function{rewire},
        m_growth_operation_number{0}
    {
        addOrigin(origin);
        setGoalSearchPercentage(goal_search_percentage);
    }

    int grow(){
        ConfigurationPoint candidate_point;
        int nearest_point_index;
        if(chooseGoal()){
            candidate_point = m_goal;
        }else{
            candidate_point = m_random_generator(m_growth_operation_number);
        }
        nearest_point_index = m_nearest(m_vertices, candidate_point);
        candidate_point = m_step_function(candidate_point,m_vertices[nearest_point_index], m_step_size);
        if(m_collision_function(candidate_point, m_vertices[nearest_point_index])){
            return -1;
        }
        std::vector<int> points_near = m_near_function(m_vertices, candidate_point);
        int parent_index = m_choose_parent_function(points_near, nearest_point_index, candidate_point, m_vertices, m_costs);
        int new_point_index = addVertixAndEdge(candidate_point, parent_index);
        m_rewire_function(m_vertices, m_edges, m_costs, points_near, parent_index, candidate_point);
        m_growth_operation_number++;
        return new_point_index;
    }

    const std::vector<ConfigurationPoint>& getVertices(){
        return m_vertices;
    }

    const std::vector<std::array<int,2>>& getEdges(){
        return m_edges;
    }
    void setGoalSearchPercentage(double goal_search_percentage){
        if( goal_search_percentage < 0){
            m_goal_search_percentage = 0;
        }else if( goal_search_percentage > 1){
            m_goal_search_percentage = 1;
        }else{
            m_goal_search_percentage = goal_search_percentage;
        }
    }
    void setGoal(const ConfigurationPoint& goal){
        m_goal = goal;
    }
    std::deque<ConfigurationPoint> getPathToRoot(int point_index) const{
        std::deque<ConfigurationPoint> path;
        path.push_front(m_vertices[point_index]);
        while( point_index != 0 ){
            for(const auto& e: m_edges){
                if( e[1] == point_index ){
                    point_index = e[0];
                    path.push_front(m_vertices[point_index]);
                    break;
                }
            }
        }
        return path;
    }
};

#endif //RRTSTARTEMPLATE_H__