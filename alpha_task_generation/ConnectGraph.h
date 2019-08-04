#ifndef CONNECT_GRAPH_H_ 
#define CONNECT_GRAPH_H_

#include <vector>
#include <TrigDefinitions.h>
#include <EdgeStorage.h>

// typedef bool (*collision_function_t)(float, float);
// typedef float (*connection_function_t)(float, float);
// TrianglesI connectGraph(const Points& points);
// EdgeStorage connectGraph2(const Points& points);
// EdgeStorage connectGraph3(const Points& points, collision_function_t collision_fn, connection_function_t connection_fn);
// template <typename CollisionFunction, typename ConnectionFunction>
// EdgeStorage connectGraph(const Points& points, collision_function_t collision_fn, connection_function_t connection_fn);
// template <typename T1, typename T2>
// EdgeStorage connectGraph(const Points& points, T1 collision_fn, T2 connection_fn);

TriangleI orderIndexes(const TriangleI& indexes);
PointsI neighborhood_k(int index, const Points& points, int k);
bool edgeInMoreThanOneTriangle(const EdgeI& edge, const TrianglesI& triangles);
PointsI getOpositionPointsByDistance(const EdgeI& edge_i, const PointI& oposing_point_i, const Points& points);
TrianglesI getTrianglesToLook(const EdgeI& edge_i, const PointI& oposing_point_i, const TrianglesI& triangles, const Points& points);
bool checkTriangleIntersection(const EdgeI& edge_i, const PointI& oposing_point_i, const TrianglesI& triangles_to_check_i, const Points& points);

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
            if(connection_count > 50){
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
    int initial_point = 0;
    TriangleI initial_triangle = orderIndexes(PI2TI(neighborhood_k(initial_point, points, 3)));
    TrianglesI triangles = {initial_triangle};
    EdgeStorage edge_storage;
    edge_storage.addTriangle(initial_triangle);
    for(size_t i=0; i < triangles.size(); i++){
        EdgesI edges = { EdgeI{triangles[i][0], triangles[i][1]},
                         EdgeI{triangles[i][0], triangles[i][2]},
                         EdgeI{triangles[i][1], triangles[i][2]} };
        PointsI edge_oposing_points = { PointI{triangles[i][2]},
                                        PointI{triangles[i][1]},
                                        PointI{triangles[i][0]} };
        for(size_t j = 0; j < 3; j++){
            if(! edgeInMoreThanOneTriangle(edges[j], triangles)){
                EdgeI& edge = edges[j];
                PointI& edge_oposing_point = edge_oposing_points[j];
                PointsI candidate_points = getOpositionPointsByDistance(edge, edge_oposing_point, points);
                TrianglesI triangles_to_look = getTrianglesToLook(edge, edge_oposing_point, triangles, points);
                for (const auto& candidate_point_i: candidate_points){
                    if(!checkTriangleIntersection(edge, candidate_point_i, triangles_to_look, points)){
                        TriangleI new_triangle = orderIndexes({edge[0], edge[1], candidate_point_i});
                        triangles.push_back(new_triangle);
                        EdgeStorage_addTriangleCheckingCollision(edge_storage, new_triangle, points, collision_fn, connection_fn);
                        break;
                    }
                }
            }
        }
    }
    return edge_storage;
}

#endif