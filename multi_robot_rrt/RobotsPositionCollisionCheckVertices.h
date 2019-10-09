#ifndef ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__
#define ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__

#include <tuple>
#include <map>
#include <vector>

class RobotPositionCollisionCheckVertices{
private:
    typedef std::tuple<int, int> key_t;
    typedef std::map<const key_t, int> map_t;
    std::vector<std::tuple<int, int>> m_vertices;
    map_t m_reverse_search_engine;

public:
    bool insert(std::tuple<int, int> rp_i){
        if( contains(rp_i) ){
            return false;
        }
        int new_index = m_vertices.size();
        m_vertices.push_back(rp_i);
        m_reverse_search_engine[rp_i] = new_index;
        return true;
    }

    bool insert(int r_i, int p_i){
        return insert(std::make_tuple(r_i, p_i));
    }
    
    bool contains(std::tuple<int,int> rp_i) const{
        return (m_reverse_search_engine.find(rp_i) != m_reverse_search_engine.end());
    }
    bool contains(int r_i, int p_i) const{
        return contains(std::make_tuple(r_i, p_i));
    }
    int getIndex(std::tuple<int,int> rp_i) const{
        if(contains(rp_i)){
            return m_reverse_search_engine.at(rp_i);
        }else{
            return -1;
        }
    }
    int getIndex(int r_i, int p_i) const{
        return getIndex(std::make_tuple(r_i, p_i));
    }
    const std::tuple<int, int>& operator[](int i) const{
        return m_vertices[i];
    }
    size_t size() const{
        return m_vertices.size();
    }
    std::vector<std::tuple<int,int>>::const_iterator begin() const{
        return m_vertices.begin();
    }
    std::vector<std::tuple<int,int>>::const_iterator end() const{
        return m_vertices.end();
    }
};

#endif //ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__