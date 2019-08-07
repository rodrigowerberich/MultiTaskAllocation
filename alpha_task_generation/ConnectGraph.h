#ifndef CONNECT_GRAPH_H_ 
#define CONNECT_GRAPH_H_

#include <vector>
#include <TrigDefinitions.h>
#include <EdgeStorage.h>
#include <delaunator.h>


TriangleI orderIndexes(const TriangleI& indexes);

template <typename CollisionFunction, typename ConnectionFunction>
bool checkEdgeCollisionOrConnection(const Edge& edge, const CollisionFunction& collision_fn, const ConnectionFunction& connection_fn){
    const Point& a = edge[0];
    const Point& b = edge[1];
    Point c = b-a;
    int connection_count = 0;
    for(int i=0; i<100; i++){
        Point d = a + (c*i)/100;
        if(collision_fn(d.x, d.y)){
            return true;
        }
        if(connection_fn(d.x, d.y)){
            connection_count++;
            if(connection_count > 90){
                return true;
            }
        }
    }
    return false;
}

template <typename CollisionFunction, typename ConnectionFunction>
void EdgeStorage_addTriangleCheckingCollision(EdgeStorage& edge_storage, const TriangleI& triangle, const Points& points, const CollisionFunction& coll_fn, const ConnectionFunction& conn_fn){
    EdgeI edge_i0 = {triangle[0], triangle[1]};
    EdgeI edge_i1 = {triangle[0], triangle[2]};
    EdgeI edge_i2 = {triangle[1], triangle[2]};
    Edge edge0 = { points[edge_i0[0]], points[edge_i0[1]] };
    Edge edge1 = { points[edge_i1[0]], points[edge_i1[1]] };
    Edge edge2 = { points[edge_i2[0]], points[edge_i2[1]] };
    if(!checkEdgeCollisionOrConnection(edge0, coll_fn, conn_fn)){
        edge_storage.addEdge(edge_i0);
    }
    if(!checkEdgeCollisionOrConnection(edge1, coll_fn, conn_fn)){
        edge_storage.addEdge(edge_i1);
    }
    if(!checkEdgeCollisionOrConnection(edge2, coll_fn, conn_fn)){
        edge_storage.addEdge(edge_i2);
    }
}


template <typename CollisionFunction, typename ConnectionFunction>
EdgeStorage connectGraph(const Points& points, const CollisionFunction& collision_fn, const ConnectionFunction& connection_fn){
    std::vector<double> coords;
    for(const auto& p: points){
        coords.push_back(p[0]);
        coords.push_back(p[1]);
    }
    delaunator::Delaunator d(coords);
    EdgeStorage edge_storage;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
        TriangleI new_triangle = orderIndexes({d.triangles[i], d.triangles[i+1], d.triangles[i+2]});
        EdgeStorage_addTriangleCheckingCollision(edge_storage, new_triangle, points, collision_fn, connection_fn);
    }    
    return edge_storage;
}

#endif