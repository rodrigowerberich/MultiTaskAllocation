#ifndef EDGESTORAGE_H__
#define EDGESTORAGE_H__

#include <map>
#include <set>
#include <string>
#include <TrigHelper.h>

class EdgeStorage{
private:
    std::map<int,std::set<int>> m_storage;
    EdgeI orderEdge(const EdgeI& edge);
public:
    EdgeStorage(){};
    bool containsEdge(const EdgeI& edge);
    void addEdge(const EdgeI& edge);
    void addTriangle(const TriangleI& triangle);
    void removeEdge(const EdgeI& edge);
    std::string toString();
    EdgesI toEdges();
};

#endif //EDGESTORAGE_H__