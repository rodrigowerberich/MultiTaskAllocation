#include <Dijkstra.h>


namespace dijkstra{

int generate_next_point(std::vector<std::tuple<double,bool,PointI,EdgesI>>& points_infos){
    std::tuple<int, double> min_value = std::make_tuple(-1, std::numeric_limits<double>::infinity());
    for(const auto& i: util::lang::indices(points_infos)){
        double point_value = std::get<0>(points_infos[i]);
        bool visited = std::get<1>(points_infos[i]);
        
        if(!visited && (point_value < std::get<1>(min_value))){
            min_value = std::make_tuple(i, point_value);
        }
    }
    return std::get<0>(min_value);
}

}