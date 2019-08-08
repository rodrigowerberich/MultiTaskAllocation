#ifndef DIJKSTRA_H__
#define DIJKSTRA_H__

#include <vector>
#include <tuple>
#include <TrigDefinitions.h>
#include <range.h>


namespace dijkstra{
int generate_next_point(std::vector<std::tuple<double,bool,PointI,EdgesI>>& points_infos);

template <typename T1, typename T2, typename T3>
std::vector<std::tuple<double, PointI>> dijkstra(const T1& origins, const T2& points, const T3& edge_storage){
    assert(origins.size() == points.size());
    std::vector<std::tuple<double,bool, PointI, EdgesI>> points_infos(points.size());
    for(const auto& i: util::lang::indices(points)){
        auto value = origins[i]?0:std::numeric_limits<double>::max();
        points_infos[i] = std::make_tuple(value, false, -1, edge_storage.getEdgesWith(i));
    }
    for(int curr_point_index = generate_next_point(points_infos); curr_point_index != -1; curr_point_index = generate_next_point(points_infos)){
        for(const auto& neighbour_edge_index: std::get<3>(points_infos[curr_point_index])){
            auto other_point_index = (neighbour_edge_index[0] == curr_point_index)? neighbour_edge_index[1]: neighbour_edge_index[0];
            auto new_cost = std::get<0>(points_infos[curr_point_index]) + Point::euclideanDistance(points[curr_point_index], points[other_point_index]);
            if(  new_cost < std::get<0>(points_infos[other_point_index]) ){
                std::get<0>(points_infos[other_point_index]) = new_cost;
                std::get<2>(points_infos[other_point_index]) = curr_point_index;
            }
        }
        std::get<1>(points_infos[curr_point_index]) = true;
    }
    std::vector<std::tuple<double, PointI>> result(points.size());
    for(const auto& i: util::lang::indices(points_infos)){
        result[i] = std::make_tuple(std::get<0>(points_infos[i]), std::get<2>(points_infos[i]));
    }
    return result;
}

}

#endif //DIJKSTRA_H__