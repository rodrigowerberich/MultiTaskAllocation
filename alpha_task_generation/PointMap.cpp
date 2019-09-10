#include <PointMap.h>
#include <range.h>

template <typename T>
T limit_min(T value, T min, T alternative_min){
    return (value < min)?alternative_min: value;
}
template <typename T>
T limit_min(T value, T min){
    return limit_min(value,min,min);
}
template <typename T>
T limit_max(T value, T max, T alternative_max){
    return (value >= max)?alternative_max: value;
}
template <typename T>
T limit_max(T value, T max){
    return limit_max(value, max, max);
}

size_t PointMap::calculateIndexX(const Point& p) const{
    return std::floor((p.getX()-m_x_min)/(1.0000000001*m_x_max-m_x_min)*m_N);
}

size_t PointMap::calculateIndexY(const Point& p) const{
    return std::floor((p.getY()-m_y_min)/(1.0000000001*m_y_max-m_y_min)*m_M);
}

size_t PointMap::calculateIndex(const Point& p) const{
    return calculateIndexX(p)+m_M*calculateIndexY(p);

}
bool PointMap::insideMap(const Point& p) const{
    return ( (m_x_min <= p.getX() && p.getX() <= m_x_max) && (m_y_min <= p.getY() && p.getY() <= m_y_max)  );
}


PointMap::PointMap(double x_min, double x_max, double y_min, double y_max, size_t N, size_t M):
    m_x_min{x_min}, m_x_max{x_max}, m_y_min{y_min}, m_y_max{y_max}, m_N{N}, m_M{M},m_points_mapping{N*M}{
}

bool PointMap::insert(const Point& p){
    if(!insideMap(p)) return false;
    m_points.push_back(p);
    size_t point_position_in_vector = m_points.size()-1;
    size_t quadrant_index = calculateIndex(p);
    m_points_mapping[quadrant_index].push_back(point_position_in_vector);
    return true;
}
bool PointMap::insert(double x, double y){
    return insert(Point{x,y});
}
bool PointMap::insert(std::tuple<double, double> p){
    return insert(std::get<0>(p), std::get<1>(p));
}

// double distanceToNearestPoint(const Point& p){

// }

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

template <typename Mapping, typename PointsType>
std::tuple <double, int> closestPoint(const Mapping& mapping, int index_to_search, const Point& p, const PointsType& points){
    std::vector<double> distances(mapping[index_to_search].size());
    std::transform(std::begin(mapping[index_to_search]), std::end(mapping[index_to_search]), std::begin(distances), [&p,&points](const auto&index){ return Point::euclideanDistance(p, points[index]); });
    auto result = std::min_element(std::begin(distances), std::end(distances));
    if(result != std::end(distances)){
        return std::make_tuple(*result, mapping[index_to_search][std::distance(std::begin(distances), result)]);
    }
    return std::make_tuple(-1.0, -1);
}

template <typename Distances, typename PointIndexes, typename Mapping, typename PointsType>
void add_min_distance(int x, int y, Distances& distances, PointIndexes& point_indexes, const Mapping& mapping, int M, const Point& p, const PointsType& points){
        double min_value = std::numeric_limits<double>::max();
        int min_index = -1;
        std::tie(min_value, min_index) = closestPoint(mapping, M*y+x, p, points);
        if(min_index != -1){
            distances.push_back(min_value);
            point_indexes.push_back(min_index);
        }
}

PointI PointMap::nearestPointSmall(const Point& interest_point) const{
    using util::lang::indices;
    double min_dist = std::numeric_limits<double>::max();
    int min_index = -1;
    for(const auto& i: indices(m_points)){
        double curr_value = Point::euclideanDistance(m_points[i], interest_point);
        if( curr_value < min_dist){
            min_dist = curr_value;
            min_index = i;
        }
    }
    return min_index;
}
PointI PointMap::nearestPointBig(const Point& p) const{
    using util::lang::range;
    if(!insideMap(p)) return -1;
    int origin_x = calculateIndexX(p);
    int origin_y = calculateIndexY(p);
    std::tuple<bool, double, int> closest_point(false, -1.0, -1);
    for(int left_border = origin_x, right_border = origin_x, top_border=origin_y, bottom_border=origin_y; (left_border != -1) || (right_border != -1) || (bottom_border != -1) || (top_border != -1);){
        if(left_border == right_border && top_border==bottom_border){
            double min_value = std::numeric_limits<double>::max();
            int min_index = -1;
            std::tie(min_value, min_index) = closestPoint(m_points_mapping, m_M*top_border+left_border, p, m_points); 
            if(min_index != -1){
                std::get<0>(closest_point) = true;
                std::get<1>(closest_point) = min_value;
                std::get<2>(closest_point) = min_index;
            }
        }else{
            std::vector<double> min_distances;
            std::vector<int> min_points;
            if(left_border != -1){
                for(auto y: range((bottom_border == -1)?0:(bottom_border+1), (top_border == -1)?((int)m_N-1):top_border)){
                    add_min_distance(left_border, y, min_distances, min_points, m_points_mapping, m_M, p, m_points);
                }
            }
            if(right_border != -1){
                for(auto y: range((bottom_border == -1)?0:(bottom_border+1), (top_border == -1)?((int)m_N-1):top_border)){
                    add_min_distance(right_border, y, min_distances, min_points, m_points_mapping, m_M, p, m_points);
                }
            }
            if(bottom_border != -1){
                for(auto x: range((left_border == -1)?0:left_border, (right_border == -1)?((int)m_M):right_border+1)){
                    add_min_distance(x, bottom_border, min_distances, min_points, m_points_mapping, m_M, p, m_points);
                }
            }
            if(top_border != -1){
                for(auto x: range((left_border == -1)?0:left_border, (right_border == -1)?((int)m_M):right_border+1)){
                    add_min_distance(x, top_border, min_distances, min_points, m_points_mapping, m_M, p, m_points);
                }
            }
            if(!min_distances.empty()){
                auto min_element = std::min_element(std::begin(min_distances), std::end(min_distances));
                auto min_index = min_points[std::distance(std::begin(min_distances), min_element)];
                if(std::get<0>(closest_point)){
                    if(std::get<1>(closest_point) < *min_element){
                        return std::get<2>(closest_point);
                    }else{
                        return min_index;
                    }
                }else{
                    std::get<0>(closest_point) = true;
                    std::get<1>(closest_point) = *min_element;
                    std::get<2>(closest_point) = min_index;
                }
            }
        }
        decrement_limit(left_border, 0);
        increment_limit(right_border, m_N-1);
        decrement_limit(bottom_border, 0);
        increment_limit(top_border, m_M-1);

    }
    if(std::get<0>(closest_point)){
        return std::get<2>(closest_point);    
    }else{
        return -1;
    }
}


PointI PointMap::nearestPointIndex(const Point& p) const{
    if(m_points.size() < 250){
        return nearestPointSmall(p);
    }else{
        return nearestPointBig(p);
    }
}


Point PointMap::nearestPoint(const Point& p) const{
    PointI index = nearestPointIndex(p);
    if(index == -1) return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    else return m_points[index];
}


Points PointMap::pointsInRange(const Point& p, double radius){
    if(!insideMap(p)) return Points();
    Points points_in_range;
    int d_x = ((radius/(m_x_max-m_x_min))*m_N)+1;
    int d_y = ((radius/(m_y_max-m_y_min))*m_M)+1;
    int origin_x = calculateIndexX(p);
    int origin_y = calculateIndexY(p);
    int min_x = limit_min<int>(origin_x-d_x, 0);
    int max_x = limit_max<int>(origin_x+d_x, m_N, m_N-1);
    int min_y = limit_min<int>(origin_y-d_y, 0);
    int max_y = limit_max<int>(origin_y+d_y, m_M, m_M-1);

    for(int x = min_x; x <= max_x; x++){
        for(int y = min_y; y <=max_y; y++){
            for(size_t index: m_points_mapping[m_M*y+x]){
                if(Point::euclideanDistance(p, m_points[index]) < radius){
                    points_in_range.push_back(m_points[index]);
                }
            }
        }
    }
    return points_in_range;
}

PointsI PointMap::pointsInRangeByIndex(const Point& p, double radius) const{
    if(!insideMap(p)) return PointsI();
    PointsI points_in_range;
    int d_x = ((radius/(m_x_max-m_x_min))*m_N)+1;
    int d_y = ((radius/(m_y_max-m_y_min))*m_M)+1;
    int origin_x = calculateIndexX(p);
    int origin_y = calculateIndexY(p);
    int min_x = limit_min<int>(origin_x-d_x, 0);
    int max_x = limit_max<int>(origin_x+d_x, m_N, m_N-1);
    int min_y = limit_min<int>(origin_y-d_y, 0);
    int max_y = limit_max<int>(origin_y+d_y, m_M, m_M-1);

    for(int x = min_x; x <= max_x; x++){
        for(int y = min_y; y <=max_y; y++){
            for(size_t index: m_points_mapping[m_M*y+x]){
                if(Point::euclideanDistance(p, m_points[index]) < radius){
                    points_in_range.push_back(index);
                }
            }
        }
    }
    return points_in_range;
}


bool PointMap::hasPointInRange(const Point& p, double radius){
    if(!insideMap(p)) return false;
    Points points_in_range;
    int d_x = ((radius/(m_x_max-m_x_min))*m_N)+1;
    int d_y = ((radius/(m_y_max-m_y_min))*m_M)+1;
    int origin_x = calculateIndexX(p);
    int origin_y = calculateIndexY(p);
    int min_x = limit_min<int>(origin_x-d_x, 0);
    int max_x = limit_max<int>(origin_x+d_x, m_N, m_N-1);
    int min_y = limit_min<int>(origin_y-d_y, 0);
    int max_y = limit_max<int>(origin_y+d_y, m_M, m_M-1);

    for(int x = min_x; x <= max_x; x++){
        for(int y = min_y; y <=max_y; y++){
            for(size_t index: m_points_mapping[m_M*y+x]){
                if(Point::euclideanDistance(p, m_points[index]) < radius){
                    return true;
                }
            }
        }
    }
    return false;
}



void PointMap::show(){
    for(size_t j = 0; j < m_M; j++){
        for(size_t i = 0; i < m_N; i++){
            size_t k = m_M*j+i;
            std::cout << (k) << ":\n";
            if(m_points_mapping[k].empty()){
                std::cout << "Is empty";
            }else{
                for(const auto& p_index: m_points_mapping[k]){
                    std::cout << m_points[p_index] << ", ";
                }
            }
            std::cout << std::endl;
        }
    }
}