#include <DistanceMap.h>
#include <range.h>
#include <TrigHelper.h>
#include <unordered_set>

size_t DistanceMap::calculateIndexX(const Point& p) const{
    return std::floor((p.getX()-m_x_min)/(1.0000000001*m_x_max-m_x_min)*m_N);
}

size_t DistanceMap::calculateIndexY(const Point& p) const{
    return std::floor((p.getY()-m_y_min)/(1.0000000001*m_y_max-m_y_min)*m_M);
}

size_t DistanceMap::calculateIndex(const Point& p) const{
    return calculateIndexX(p)+m_M*calculateIndexY(p);

}
bool DistanceMap::insideMap(const Point& p) const{
    return ( (m_x_min <= p.getX() && p.getX() <= m_x_max) && (m_y_min <= p.getY() && p.getY() <= m_y_max)  );
}


DistanceMap::DistanceMap(double x_min, double x_max, double y_min, double y_max, size_t N, size_t M):
    m_point_map{x_min, x_max, y_min, y_max, N, M},m_x_min{x_min}, m_x_max{x_max}, m_y_min{y_min}, m_y_max{y_max}, m_N{N}, m_M{M}, m_edges_mapping{N*M}{
}

bool DistanceMap::insert(const Point& p){
    return m_point_map.insert(p);
}
bool DistanceMap::insert(double x, double y){
    return m_point_map.insert(x,y);
}
bool DistanceMap::insert(std::tuple<double, double> p){
    return m_point_map.insert(p);
}

bool DistanceMap::insertEdge(const EdgeI& edge){
    PointI p1_i = edge[0];
    PointI p2_i = edge[1];
    // Check if points are valid points in our point map
    if( (p1_i >= m_point_map.size()) || (p2_i >= m_point_map.size()) ){
        return false;
    }
    // Transforming from global coordinates to local coordinates
    auto p1_x = calculateIndexX(m_point_map[p1_i]);
    auto p1_y = calculateIndexY(m_point_map[p1_i]);
    auto p2_x = calculateIndexX(m_point_map[p2_i]);
    auto p2_y = calculateIndexY(m_point_map[p2_i]);

    // An edge point can be in any place of the square
    // delimited by (min_x, min_y) (max_x, max_y)
    auto min_x = std::min(p1_x, p2_x);
    auto max_x = std::max(p1_x, p2_x);
    auto min_y = std::min(p1_y, p2_y);
    auto max_y = std::max(p1_y, p2_y);

    // std::cout << "min_x:" << min_x << ", min_y:" << min_y << std::endl;
    // std::cout << "max_x:" << max_x << ", max_y:" << max_y << std::endl;

    auto edge_index = m_edges.size();
    m_edges.push_back(edge);
    for(auto x=min_x; x <= max_x; x++){
        for(auto y=min_y; y <= max_y; y++){
            // std::cout << "x:" << x << ", y:" << y << std::endl;
            m_edges_mapping[x+m_M*y].push_back(edge_index);
        }
    }
    return true;
}

static void increment_limit(int& curr_value, int limit){
    if(curr_value == -1){
        return;
    }else if(curr_value < limit){
        curr_value++;
    }else{
        curr_value = -1;
    }
}

static void decrement_limit(int& curr_value, int limit){
    if(curr_value == -1){
        return;
    }else if(curr_value >= limit){
        curr_value--;
    }else{
        curr_value = -1;
    }
}

template <typename Mapping, typename PointsType, typename EdgesType>
std::tuple <double, int> closestEdge(const Mapping& mapping, int index_to_search, const Point& p, const PointsType& points, const EdgesType& edges, std::unordered_set<int>& checked_edge){
    std::vector<double> distances;
    std::vector<int> indexes;
    for(const auto index: mapping[index_to_search]){
        if( checked_edge.count(index) == 0 ){
            distances.push_back(distancePointAndVector(p, {points[edges[index][0]], points[edges[index][1]]}));
            indexes.push_back(index);
            checked_edge.insert(index);
        }
    }
    auto result = std::min_element(std::begin(distances), std::end(distances));
    if(result != std::end(distances)){
        return std::make_tuple(*result, indexes[std::distance(std::begin(distances), result)]);
    }
    return std::make_tuple(-1.0, -1);
}

template <typename Distances, typename EdgesIndexes, typename Mapping, typename PointsType, typename EdgesType>
void add_min_distance(int x, int y, Distances& distances, EdgesIndexes& edges_indexes, const Mapping& mapping, int M, const Point& p, const PointsType& points, const EdgesType& edges, std::unordered_set<int>& checked_edge){
        double min_value = std::numeric_limits<double>::max();
        int min_index = -1;
        std::tie(min_value, min_index) = closestEdge(mapping, M*y+x, p, points, edges, checked_edge);
        if(min_index != -1){
            distances.push_back(min_value);
            edges_indexes.push_back(min_index);
        }
}

EdgeI DistanceMap::nearestEdgeSmall(const Point& p) const{
    using util::lang::indices;
    double min_dist = std::numeric_limits<double>::max();
    EdgeI min_edge = {-1,-1};
    if(!insideMap(p)) return min_edge;
    for(const auto& i: indices(m_edges)){
        double curr_value = distancePointAndVector(p, {m_point_map[m_edges[i][0]], m_point_map[m_edges[i][1]]});
        if( curr_value < min_dist){
            min_dist = curr_value;
            min_edge = m_edges[i];
        }
    }
    return min_edge;
}
EdgeI DistanceMap::nearestEdgeBig(const Point& p) const{
    using util::lang::range;
    if(!insideMap(p)) return {-1,-1};
    int origin_x = calculateIndexX(p);
    int origin_y = calculateIndexY(p);
    std::unordered_set<int> checked_edge;
    // compared, distance, index
    std::tuple<bool, double, int> closest_edge(false, -1.0, -1);
    for(int left_border = origin_x, right_border = origin_x, top_border=origin_y, bottom_border=origin_y; (left_border != -1) || (right_border != -1) || (bottom_border != -1) || (top_border != -1);){
        if(left_border == right_border && top_border==bottom_border){
            double min_value = std::numeric_limits<double>::max();
            int min_index = -1;
            std::tie(min_value, min_index) = closestEdge(m_edges_mapping, m_M*top_border+left_border, p, m_point_map, m_edges, checked_edge); 
            if(min_index != -1){
                std::get<0>(closest_edge) = true;
                std::get<1>(closest_edge) = min_value;
                std::get<2>(closest_edge) = min_index;
            }
        }else{
            std::vector<double> min_distances;
            std::vector<int> min_edges;
            if(left_border != -1){
                for(auto y: range((bottom_border == -1)?0:(bottom_border+1), (top_border == -1)?((int)m_N-1):top_border)){
                    add_min_distance(left_border, y, min_distances, min_edges, m_edges_mapping, m_M, p, m_point_map, m_edges, checked_edge);
                }
            }
            if(right_border != -1){
                for(auto y: range((bottom_border == -1)?0:(bottom_border+1), (top_border == -1)?((int)m_N-1):top_border)){
                    add_min_distance(right_border, y, min_distances, min_edges, m_edges_mapping, m_M, p, m_point_map, m_edges, checked_edge);
                }
            }
            if(bottom_border != -1){
                for(auto x: range((left_border == -1)?0:left_border, (right_border == -1)?((int)m_M):right_border+1)){
                    add_min_distance(x, bottom_border, min_distances, min_edges, m_edges_mapping, m_M, p, m_point_map, m_edges, checked_edge);
                }
            }
            if(top_border != -1){
                for(auto x: range((left_border == -1)?0:left_border, (right_border == -1)?((int)m_M):right_border+1)){
                    add_min_distance(x, top_border, min_distances, min_edges, m_edges_mapping, m_M, p, m_point_map, m_edges, checked_edge);
                }
            }
            if(!min_distances.empty()){
                auto min_element = std::min_element(std::begin(min_distances), std::end(min_distances));
                auto min_index = min_edges[std::distance(std::begin(min_distances), min_element)];
                if(std::get<0>(closest_edge)){
                    if(std::get<1>(closest_edge) < *min_element){
                        return m_edges[std::get<2>(closest_edge)];
                    }else{
                        return m_edges[min_index];
                    }
                }else{
                    std::get<0>(closest_edge) = true;
                    std::get<1>(closest_edge) = *min_element;
                    std::get<2>(closest_edge) = min_index;
                }
            }
        }
        decrement_limit(left_border, 0);
        increment_limit(right_border, m_N-1);
        decrement_limit(bottom_border, 0);
        increment_limit(top_border, m_M-1);

    }
    if(std::get<0>(closest_edge)){
        return m_edges[std::get<2>(closest_edge)];    
    }else{
        return {-1,-1};
    }
}

EdgeI DistanceMap::nearestEdgeIndex(const Point& p) const{
     if(m_edges.size() < 250){
        std::cout << "nearest Small\n";
        return nearestEdgeSmall(p);
    }else{
        std::cout << "nearest Big\n";
        return nearestEdgeBig(p);
    }
}


PointI DistanceMap::nearestPointIndex(const Point& p) const{
    return m_point_map.nearestPointIndex(p);
}


Point DistanceMap::nearestPoint(const Point& p) const{
    return m_point_map.nearestPoint(p);
}


Points DistanceMap::pointsInRange(const Point& p, double radius){
    return m_point_map.pointsInRange(p, radius);
}

PointsI DistanceMap::pointsInRangeByIndex(const Point& p, double radius) const{
    return m_point_map.pointsInRangeByIndex(p, radius);
}


bool DistanceMap::hasPointInRange(const Point& p, double radius){
    return m_point_map.hasPointInRange(p, radius);
}



void DistanceMap::show(){
    std::cout << "Points: \n";
    m_point_map.show();
    std::cout << "Edges: \n";
    for(size_t j = 0; j < m_M; j++){
        for(size_t i = 0; i < m_N; i++){
            size_t k = m_M*j+i;
            std::cout << (k) << ":\n";
            if(m_edges_mapping[k].empty()){
                std::cout << "Is empty";
            }else{
                for(const auto& e_index: m_edges_mapping[k]){
                    // std::cout << "(" << m_edges[e_index][0] <<","<< m_edges[e_index][1] << "), ";
                    std::cout << "(" << m_point_map[m_edges[e_index][0]] <<","<< m_point_map[ m_edges[e_index][1]] << "), ";
                }
            }
            std::cout << std::endl;
        }
    }
}