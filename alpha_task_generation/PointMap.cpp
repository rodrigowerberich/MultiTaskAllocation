#include <PointMap.h>

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

size_t PointMap::calculateIndexX(const Point& p){
    return std::floor((p.getX()-m_x_min)/(1.0000000001*m_x_max-m_x_min)*m_N);
}

size_t PointMap::calculateIndexY(const Point& p){
    return std::floor((p.getY()-m_y_min)/(1.0000000001*m_y_max-m_y_min)*m_M);
}

size_t PointMap::calculateIndex(const Point& p){
    return calculateIndexX(p)+m_M*calculateIndexY(p);

}
bool PointMap::insideMap(const Point& p){
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
