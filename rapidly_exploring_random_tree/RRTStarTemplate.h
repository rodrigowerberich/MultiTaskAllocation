#ifndef RRTSTARTEMPLATE_H__
#define RRTSTARTEMPLATE_H__

#include <vector>
#include <array>

template <typename ConfigurationPoint, typename RandomConfigurationGenerator, typename NearestFunction, typename StepFunction, typename StepType, typename CollisionFunction, typename NearFunction, typename ChooseParentFunction, typename RewireFunction>
class RRTStarTemplate{
private:
    std::vector<ConfigurationPoint> m_vertices;
    std::vector<std::array<int,2>> m_edges;
    std::vector<double> m_costs;
    const RandomConfigurationGenerator& m_random_generator;
    const NearestFunction& m_nearest;
    const StepFunction& m_step_function;
    const StepType& m_step_size;
    const CollisionFunction& m_collision_function;
    const NearFunction& m_near_function;
    const ChooseParentFunction& m_choose_parent_function;
    const RewireFunction& m_rewire_function;
    size_t m_growth_operation_number;

    void addOrigin(const ConfigurationPoint& origin){
        m_vertices.push_back(origin);
        m_costs.push_back(0);
    }
    void addVertixAndEdge(const ConfigurationPoint& point, int parent_index){
        int new_point_index = m_vertices.size();
        m_vertices.push_back(point);
        m_edges.push_back({parent_index, new_point_index});
    }
public:
    RRTStarTemplate(const ConfigurationPoint& origin, const RandomConfigurationGenerator& random_generator, const NearestFunction& nearest, const StepFunction& step_function,  const StepType& step_size, const CollisionFunction& collision_function, const NearFunction& near_function, const ChooseParentFunction& choose_parent_function, const RewireFunction& rewire):
        m_random_generator{random_generator},
        m_nearest{nearest},
        m_step_function{step_function},
        m_step_size{step_size},
        m_collision_function{collision_function},
        m_near_function{near_function},
        m_choose_parent_function(choose_parent_function),
        m_rewire_function{rewire},
        m_growth_operation_number{0}
    {
        addOrigin(origin);
    }

    void grow(){
        ConfigurationPoint candidate_point;
        int nearest_point_index;
        do{
            candidate_point = m_random_generator(m_growth_operation_number);
            nearest_point_index = m_nearest(m_vertices, candidate_point);
            candidate_point = m_step_function(candidate_point,m_vertices[nearest_point_index], m_step_size);
        }while(m_collision_function(candidate_point, m_vertices[nearest_point_index]));
        std::vector<int> points_near = m_near_function(m_vertices, candidate_point);
        int parent_index = m_choose_parent_function(points_near, nearest_point_index, candidate_point, m_vertices, m_costs);
        addVertixAndEdge(candidate_point, parent_index);
        m_rewire_function(m_vertices, m_edges, m_costs, points_near, parent_index, candidate_point);
        m_growth_operation_number++;
    }

    const std::vector<ConfigurationPoint>& getVertices(){
        return m_vertices;
    }

    const std::vector<std::array<int,2>>& getEdges(){
        return m_edges;
    }
};

#endif //RRTSTARTEMPLATE_H__