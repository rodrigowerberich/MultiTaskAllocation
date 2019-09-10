#ifndef DIJKSTRAPATHFINDER_H__
#define DIJKSTRAPATHFINDER_H__

#include <Dijkstra.h>
#include <PointMap.h>
#include <EdgeStorage.h>

namespace dijkstra{

class PathFinder{
private:
    PointMap m_point_map;
    EdgeStorage m_edge_storage;
    dijkstra::Result m_result;
public:
    PathFinder(PointMap point_map, EdgeStorage edge_storage, dijkstra::Result result);
    Points getPath(const Point& interest_point) const;
};

}

#endif //DIJKSTRAPATHFINDER_H__