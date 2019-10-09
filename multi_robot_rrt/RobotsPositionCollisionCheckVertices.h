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
    bool insert(std::tuple<int, int> rp_i);
    bool insert(int r_i, int p_i);
    bool contains(std::tuple<int,int> rp_i) const;
    bool contains(int r_i, int p_i) const;
    int getIndex(std::tuple<int,int> rp_i) const;
    int getIndex(int r_i, int p_i) const;
    const std::tuple<int, int>& operator[](int i) const;
    size_t size() const;
    std::vector<std::tuple<int,int>>::const_iterator begin() const;
    std::vector<std::tuple<int,int>>::const_iterator end() const;
};

#endif //ROBOTSPOSITIONCOLLISIONCHECKVERTICES_H__