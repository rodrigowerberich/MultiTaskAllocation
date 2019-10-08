#include <RobotsPosition.h>
#include <iostream>

Point RobotsPosition::NullPoint = {0,0}; 
Point RobotsPosition::PlaceHolderPoint = {0,0}; 


RobotsPosition::RobotsPosition(size_t num_of_robots, const Points& points): m_num_of_robots{num_of_robots}, m_points(num_of_robots){
    if(points.size() > m_num_of_robots){
        std::cerr << "RobotsPosition: constructor <<  Vector greather than number of robots!!" << std::endl;
    }
    for(size_t point_i = 0; point_i < m_num_of_robots ; point_i++){
        if( point_i < points.size() ){
            m_points[point_i] = points[point_i];
        }else{
            m_points[point_i] = 0;
        }
    }
}

Point& RobotsPosition::operator[](size_t i){
    if(i < m_num_of_robots){
        return m_points[i];
    }else{
        std::cerr << "RobotsPosition: operator[] <<  Index exceed bounds!!" << std::endl;
        RobotsPosition::PlaceHolderPoint = {0,0};
        return RobotsPosition::PlaceHolderPoint;
    }
}
const Point& RobotsPosition::operator[](size_t i) const{
    if(i < m_num_of_robots){
        return m_points[i];
    }else{
        std::cerr << "RobotsPosition: operator[] const <<  Index exceed bounds!!" << std::endl;
        return RobotsPosition::NullPoint;
    }
}

Points::iterator RobotsPosition::begin() noexcept{
    return m_points.begin();
}
Points::const_iterator RobotsPosition::begin() const noexcept{
    return m_points.begin();
}
Points::iterator RobotsPosition::end() noexcept{
    return m_points.end();
}
Points::const_iterator RobotsPosition::end() const noexcept{
    return m_points.end();
}


const Points& RobotsPosition::asPoints() const{
    return m_points;
}

size_t RobotsPosition::size() const{
    return m_num_of_robots;
}


double RobotsPosition::distance(const RobotsPosition& rp1, const RobotsPosition& rp2){
    if(rp1.size()!= rp2.size()){
        return std::numeric_limits<double>::max();
    }
    double distance = 0;
    for(int rp_i=0; rp_i < rp1.size(); rp_i++){
        distance += Point::euclideanDistance(rp1[rp_i], rp2[rp_i]);
    }
    return distance;
}


std::ostream& operator<<(std::ostream& os, const RobotsPosition& rp){
    os << "{";
    for(size_t rp_i = 0; rp_i < rp.size(); rp_i++){
        os << rp[rp_i];
        if(rp_i < (rp.size()-1)){
            os << ", ";
        }
    }
    os << "}";
    return os;
}

