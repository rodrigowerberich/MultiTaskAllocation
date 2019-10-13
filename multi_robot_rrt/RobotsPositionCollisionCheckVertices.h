#ifndef ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__
#define ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__

#include <tuple>
#include <map>
#include <vector>

class RobotPositionCollisionCheckVertices{
private:
    using key_t = std::tuple<int, int>;
    using map_t = std::map<const key_t, int>;
    using container_t = std::vector<std::tuple<int, int>>;
    container_t m_vertices;
    map_t m_reverse_search_engine;
public:
    bool insert(std::tuple<int, int> rp_i);
    bool insert(int r_i, int p_i);
    bool contains(std::tuple<int,int> rp_i) const;
    bool contains(int r_i, int p_i) const;
    int getIndex(std::tuple<int,int> rp_i) const;
    int getIndex(int r_i, int p_i) const;
    const std::tuple<int, int>& operator[](int i) const;
    size_t size() const;
    container_t::const_iterator begin() const;
    container_t::const_iterator end() const;
};

#endif //ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__