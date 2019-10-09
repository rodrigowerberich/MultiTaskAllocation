#include <RobotsPositionCollisionCheckVertices.h>

bool RobotPositionCollisionCheckVertices::insert(std::tuple<int, int> rp_i){
    if( contains(rp_i) ){
        return false;
    }
    int new_index = m_vertices.size();
    m_vertices.push_back(rp_i);
    m_reverse_search_engine[rp_i] = new_index;
    return true;
}

bool RobotPositionCollisionCheckVertices::insert(int r_i, int p_i){
    return insert(std::make_tuple(r_i, p_i));
}

bool RobotPositionCollisionCheckVertices::contains(std::tuple<int,int> rp_i) const{
    return (m_reverse_search_engine.find(rp_i) != m_reverse_search_engine.end());
}
bool RobotPositionCollisionCheckVertices::contains(int r_i, int p_i) const{
    return contains(std::make_tuple(r_i, p_i));
}
int RobotPositionCollisionCheckVertices::getIndex(std::tuple<int,int> rp_i) const{
    if(contains(rp_i)){
        return m_reverse_search_engine.at(rp_i);
    }else{
        return -1;
    }
}
int RobotPositionCollisionCheckVertices::getIndex(int r_i, int p_i) const{
    return getIndex(std::make_tuple(r_i, p_i));
}
const std::tuple<int, int>& RobotPositionCollisionCheckVertices::operator[](int i) const{
    return m_vertices[i];
}
size_t RobotPositionCollisionCheckVertices::size() const{
    return m_vertices.size();
}
std::vector<std::tuple<int,int>>::const_iterator RobotPositionCollisionCheckVertices::begin() const{
    return m_vertices.begin();
}
std::vector<std::tuple<int,int>>::const_iterator RobotPositionCollisionCheckVertices::end() const{
    return m_vertices.end();
}