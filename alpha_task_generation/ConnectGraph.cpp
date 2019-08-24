#include <ConnectGraph.h>
#include <array>
#include <memory>
#include <QuickBinaryTree.h>
#include <TrigHelper.h>
#include <algorithm>

using std::vector;
using std::array;


TriangleI orderIndexes(const TriangleI& indexes){
    int i1 = indexes[0];
    int i2 = indexes[1];
    int i3 = indexes[2];
    if (i1 > i2){
        if (i2 > i3){
            return {i3, i2, i1};
        }else if (i1 > i3){
            return {i2, i3, i1};
        }else{
            return {i2, i1, i3};
        }
    }else if (i2 > i3){
        if (i1 > i3){
            return {i3, i1, i2};
        }
        else{
            return {i1, i3, i2};
        }
    }else{
        return {i1, i2, i3};
    }
}

PointsI neighborhood_k(int index, const Points& points, int){
    std::unique_ptr<Node> node = nullptr;
    const Point& target_point = points[index];
    for(size_t i=0; i< points.size(); i++){
        const Point& curr_point = points[i];
        if(i==0){
            node = std::make_unique<Node>(NodeData{euc_dist(curr_point, target_point),static_cast<int>(i)});
        }else{
            node->insert({euc_dist(curr_point, target_point),static_cast<int>(i)});
        }
    }
    vector<NodeData> triangle_data;
    node->sortedUntil(3, triangle_data);
    PointsI triangle;
    for(const auto& data: triangle_data){
        triangle.push_back(data.data);
    }
    return triangle;
 }

bool edgeInMoreThanOneTriangle(const EdgeI& edge, const TrianglesI& triangles){
    int outter_couter = 0;
    for(const auto& triangle: triangles){
        int inner_counter = 0;
        for(const auto& point: triangle){
            if((point == edge[0]) || (point == edge[1])){
                inner_counter++;
                if(inner_counter > 1){
                    outter_couter++;
                    if (outter_couter > 1){
                        return true;
                    }
                    break;
                }
            }
        }
    }
    return false;
 }

static bool on_semi_plan_opposed(const Edge& edge, const Point& po, const Point& p){
    const Point& r1 = edge[0];
    const Point& r2 = edge[1];
    if (fabs(r2.x - r1.x) > 0.000001){
        double a = (r2.y-r1.y)/(r2.x-r1.x);
        double b = r1.y - a*r1.x;
        if( a*po.x+b < po.y && a*p.x+b > p.y ){
            return true;
        }
        if( a*po.x+b > po.y && a*p.x+b < p.y ){
            return true;
        }
    }else{
        if( r1.x < po.x && r1.x > p.x ){
            return true;
        }
        if( r1.x > po.x && r1.x < p.x ){
            return true;
        }
    }
    return false;
}

static double distance_to_segment(const Edge& e, const Point& p){
    // return std::max(euc_dist(e[0], p),euc_dist(e[1], p));
    return euc_dist(e[0], p)+euc_dist(e[1], p);
}

PointsI getOpositionPointsByDistance(const EdgeI& edge_i, const PointI& oposing_point_i, const Points& points){
    const Edge& edge = {points[edge_i[0]], points[edge_i[1]]};
    const Point& oposing_point = points[oposing_point_i];
    std::unique_ptr<Node> node = nullptr;
    int k = 0;
    for(size_t point_i=0; point_i< points.size(); point_i++){
        if( !inside(edge_i, point_i) and on_semi_plan_opposed(edge, oposing_point, points[point_i])){
            if(k == 0){
                node = std::make_unique<Node>(NodeData{distance_to_segment(edge, points[point_i]), static_cast<int>(point_i)});
            }else{
                node->insert({distance_to_segment(edge, points[point_i]), static_cast<int>(point_i)});
            }
            k++;
        }
    }
    vector<NodeData> oposing_points_data;
    if(node){
        node->sortedUntil(k, oposing_points_data);
    }
    PointsI oposing_points;
    for(const auto& data: oposing_points_data){
        oposing_points.push_back(data.data);
    }
    return oposing_points;
 }

TrianglesI getTrianglesToLook(const EdgeI& edge_i, const PointI& oposing_point_i, const TrianglesI& triangles, const Points& points){
    TrianglesI triangles_to_look;
    Edge edge = { points[edge_i[0]], points[edge_i[1]] };
    Point oposing_point = points[oposing_point_i];
    for(const auto& triangle: triangles){
        for(const auto& point_i: triangle ){
            if(on_semi_plan_opposed(edge, oposing_point, points[point_i])){
                triangles_to_look.push_back(triangle);
                break;
            }
        }
    }
    return triangles_to_look;
}

static bool triangle_in_radius(const Circle& circle, const TriangleI& triangle_i, const Points& points){
    Triangle triangle = {points[triangle_i[0]], points[triangle_i[1]], points[triangle_i[2]]};
    for(const Point& point: triangle){
        if((euc_dist(point, circle.center) - circle.radius) <=  0.01){
            return true;
        }
    }
    return false;
}

struct TriangleEvaluation{
    PointsI t1;
    PointsI t2;
    PointsI common;
};

static TriangleEvaluation evaluateTriangle(const TriangleI& t1, const TriangleI& t2){
    TriangleEvaluation evaluation;
    evaluation.t1 = {t1[0], t1[1], t1[2]};
    evaluation.t2 = {t2[0], t2[1], t2[2]};
    for(const auto& pt1: t1){
        for(const auto& pt2: t2){
            if(pt1 == pt2){
                evaluation.common.push_back(pt1);
                evaluation.t1.erase(std::find(evaluation.t1.begin(), evaluation.t1.end(), pt1));
                evaluation.t2.erase(std::find(evaluation.t2.begin(), evaluation.t2.end(), pt2));
                break;
            }
        }     
    }
    return evaluation;
}

static bool checkNoSharingVertexColision(const EdgeI& edge_i, const PointI& oposing_point_i, double t1_area, const TriangleI& triangle_i, const Points& points){
    Point a1 = points[edge_i[0]];
    Point b1 = points[edge_i[1]];
    Point c1 = points[oposing_point_i];
    Point a2 = points[triangle_i[0]];
    Point b2 = points[triangle_i[1]];
    Point c2 = points[triangle_i[2]];
    
    vector<Points> segments_to_verify = {
        {a1, c1, a2, b2},
        {a1, c1, a2, c2},
        {a1, c1, b2, c2},
        {b1, c1, a2, b2},
        {b1, c1, a2, c2},
        {b1, c1, b2, c2}
    };

    for (const auto& segment: segments_to_verify){
        if (segmentsDoIntersect(segment[0], segment[1], segment[2], segment[3])){
            return true;
        }
    }

    // Check if c1 is inside the triangle a2,b2,c2
    double t2_area = triangleArea({a2,b2,c2});
    double p1_area = triangleArea({a2,b2,c1});
    double p2_area = triangleArea({a2,c2,c1});
    double p3_area = triangleArea({b2,c2,c1});
    if( fabs( t2_area-(p1_area+p2_area+p3_area) ) < 0.000001 ){
        return true;
    }

    Triangle t2 = {a2,b2,c2};

    for(const auto& p: t2){
        p1_area = triangleArea({a1,b1,p});
        p2_area = triangleArea({a1,c1,p});
        p3_area = triangleArea({b1,c1,p});
        if( fabs( t1_area-(p1_area+p2_area+p3_area) ) < 0.000001 ){
            return true;
        }   
    }

    return false;
}

static bool checkOneSharingVertexColision(const TriangleEvaluation& triangle_evaluation, const Points& points){
    Point shared_v = points[triangle_evaluation.common[0]];
    Points t1 = {points[triangle_evaluation.t1[0]], points[triangle_evaluation.t1[1]]};
    Points t2 = {points[triangle_evaluation.t2[0]], points[triangle_evaluation.t2[1]]};
    vector<double> angs1;
    vector<double> angs2;
    for(const auto& v:t1){
        angs1.push_back(atan2(v.y-shared_v.y,v.x-shared_v.x));
    }
    for(const auto& v:t2){
        angs2.push_back(atan2(v.y-shared_v.y,v.x-shared_v.x));
    }
    for(const auto& ang:angs2){
        if (checkAngleBetween(angs1[0], ang, angs1[1])){
            return true;
        }
    }
    for(const auto& ang:angs1){
        if (checkAngleBetween(angs2[0], ang, angs2[1])){
            return true;
        }
    }
    return false;
}

static bool checkTwoSharingVertexColision(const TriangleEvaluation& triangle_evaluation, const Points& points){
    Points shared_v = {points[triangle_evaluation.common[0]], points[triangle_evaluation.common[1]]};
    Point t1 = points[triangle_evaluation.t1[0]];
    Point t2 = points[triangle_evaluation.t2[0]];
    int orientation1 = pointsOrientation(shared_v[0], shared_v[1], t1);
    int orientation2 = pointsOrientation(shared_v[0], shared_v[1], t2);
    if( orientation1 == orientation2){
        return true;
    }
    return false;
}

bool checkTriangleIntersection(const EdgeI& edge_i, const PointI& oposing_point_i, const TrianglesI& triangles_to_check_i, const Points& points){
    Triangle new_triangle = {points[edge_i[0]], points[edge_i[1]], points[oposing_point_i]};
    Circle circle = circumscribedCircle(new_triangle);
    double new_triangle_area = triangleArea(new_triangle);
    for(const TriangleI& triangle_i: triangles_to_check_i){
        if(triangle_in_radius(circle, triangle_i, points)){
            TriangleEvaluation triangle_evaluation = evaluateTriangle({edge_i[0], edge_i[1], oposing_point_i}, triangle_i);
            switch (triangle_evaluation.common.size()){
                case 0:
                    if(checkNoSharingVertexColision(edge_i, oposing_point_i, new_triangle_area, triangle_i, points)){
                        return true;
                    }
                break;
                case 1:
                    if(checkOneSharingVertexColision(triangle_evaluation, points)){
                        return true;
                    }
                break;
                case 2:
                    if(checkTwoSharingVertexColision(triangle_evaluation, points)){
                        return true;
                    }
                break;
                default:
                break;
            }
        }
    }
    return false;
}

// TrianglesI connectGraph(const Points& points){
//     int initial_point = 0;
//     TriangleI initial_triangle = orderIndexes(PI2TI(neighborhood_k(initial_point, points, 3)));
//     TrianglesI triangles = {initial_triangle};
//     for(size_t i=0; i < triangles.size(); i++){
//         EdgesI edges = { EdgeI{triangles[i][0], triangles[i][1]},
//                          EdgeI{triangles[i][0], triangles[i][2]},
//                          EdgeI{triangles[i][1], triangles[i][2]} };
//         PointsI edge_oposing_points = { PointI{triangles[i][2]},
//                                         PointI{triangles[i][1]},
//                                         PointI{triangles[i][0]} };
//         for(size_t j = 0; j < 3; j++){
//             if(! edgeInMoreThanOneTriangle(edges[j], triangles)){
//                 EdgeI& edge = edges[j];
//                 PointI& edge_oposing_point = edge_oposing_points[j];
//                 PointsI candidate_points = getOpositionPointsByDistance(edge, edge_oposing_point, points);
//                 TrianglesI triangles_to_look = getTrianglesToLook(edge, edge_oposing_point, triangles, points);
//                 for (const auto& candidate_point_i: candidate_points){
//                     if(!checkTriangleIntersection(edge, candidate_point_i, triangles_to_look, points)){
//                         TriangleI new_triangle = orderIndexes({edge[0], edge[1], candidate_point_i});
//                         triangles.push_back(new_triangle);
//                         break;
//                     }
//                 }
//             }
//         }
        
//     }
//     return triangles;
// }

// EdgeStorage connectGraph2(const Points& points){
//     int initial_point = 0;
//     TriangleI initial_triangle = orderIndexes(PI2TI(neighborhood_k(initial_point, points, 3)));
//     TrianglesI triangles = {initial_triangle};
//     EdgeStorage edge_storage;
//     edge_storage.addTriangle(initial_triangle);
//     for(size_t i=0; i < triangles.size(); i++){
//         EdgesI edges = { EdgeI{triangles[i][0], triangles[i][1]},
//                          EdgeI{triangles[i][0], triangles[i][2]},
//                          EdgeI{triangles[i][1], triangles[i][2]} };
//         PointsI edge_oposing_points = { PointI{triangles[i][2]},
//                                         PointI{triangles[i][1]},
//                                         PointI{triangles[i][0]} };
//         for(size_t j = 0; j < 3; j++){
//             if(! edgeInMoreThanOneTriangle(edges[j], triangles)){
//                 EdgeI& edge = edges[j];
//                 PointI& edge_oposing_point = edge_oposing_points[j];
//                 PointsI candidate_points = getOpositionPointsByDistance(edge, edge_oposing_point, points);
//                 TrianglesI triangles_to_look = getTrianglesToLook(edge, edge_oposing_point, triangles, points);
//                 for (const auto& candidate_point_i: candidate_points){
//                     if(!checkTriangleIntersection(edge, candidate_point_i, triangles_to_look, points)){
//                         TriangleI new_triangle = orderIndexes({edge[0], edge[1], candidate_point_i});
//                         triangles.push_back(new_triangle);
//                         edge_storage.addTriangle(new_triangle);
//                         break;
//                     }
//                 }
//             }
//         }
        
//     }
//     return edge_storage;
// }

// bool checkEdgeCollision(const Edge& edge, collision_function_t collision_fn){
//     const Point& a = edge[0];
//     const Point& b = edge[1];
//     Point c = b-a;
//     for(int i=0; i<100; i++){
//         Point d = a + (c*i)/100;
//         if(collision_fn(d.x, d.y)){
//             return true;
//         }
//     }
//     return false;
// }

// template <typename CollisionFunction, typename ConnectionFunction>
// bool checkEdgeCollisionOrConnection(const Edge& edge, CollisionFunction collision_fn, ConnectionFunction connection_fn){
//     const Point& a = edge[0];
//     const Point& b = edge[1];
//     Point c = b-a;
//     int connection_count = 0;
//     for(int i=0; i<100; i++){
//         Point d = a + (c*i)/100;
//         if(collision_fn(d.x, d.y)){
//             return true;
//         }
//         if(connection_fn(d.x, d.y)){
//             connection_count++;
//             if(connection_count > 50){
//                 return true;
//             }
//         }
//     }
//     return false;
// }

// template <typename CollisionFunction, typename ConnectionFunction>
// void EdgeStorage_addTriangleCheckingCollision(EdgeStorage& edge_storage, const TriangleI& triangle, const Points& points, CollisionFunction coll_fn, ConnectionFunction conn_fn){
//     EdgeI edge_i0 = {triangle[0], triangle[1]};
//     EdgeI edge_i1 = {triangle[0], triangle[2]};
//     EdgeI edge_i2 = {triangle[1], triangle[2]};
//     Edge edge0 = { points[edge_i0[0]], points[edge_i0[1]] };
//     Edge edge1 = { points[edge_i1[0]], points[edge_i1[1]] };
//     Edge edge2 = { points[edge_i2[0]], points[edge_i2[1]] };
//     if(!checkEdgeCollisionOrConnection(edge0, coll_fn, conn_fn)){
//         edge_storage.addEdge(edge_i0);
//     }
//     if(!checkEdgeCollisionOrConnection(edge1, coll_fn, conn_fn)){
//         edge_storage.addEdge(edge_i1);
//     }
//     if(!checkEdgeCollisionOrConnection(edge2, coll_fn, conn_fn)){
//         edge_storage.addEdge(edge_i2);
//     }
// }

// template <typename CollisionFunction, typename ConnectionFunction>
// EdgeStorage connectGraph(const Points& points, collision_function_t collision_fn, connection_function_t connection_fn){
//     int initial_point = 0;
//     TriangleI initial_triangle = orderIndexes(PI2TI(neighborhood_k(initial_point, points, 3)));
//     TrianglesI triangles = {initial_triangle};
//     EdgeStorage edge_storage;
//     edge_storage.addTriangle(initial_triangle);
//     for(size_t i=0; i < triangles.size(); i++){
//         EdgesI edges = { EdgeI{triangles[i][0], triangles[i][1]},
//                          EdgeI{triangles[i][0], triangles[i][2]},
//                          EdgeI{triangles[i][1], triangles[i][2]} };
//         PointsI edge_oposing_points = { PointI{triangles[i][2]},
//                                         PointI{triangles[i][1]},
//                                         PointI{triangles[i][0]} };
//         for(size_t j = 0; j < 3; j++){
//             if(! edgeInMoreThanOneTriangle(edges[j], triangles)){
//                 EdgeI& edge = edges[j];
//                 PointI& edge_oposing_point = edge_oposing_points[j];
//                 PointsI candidate_points = getOpositionPointsByDistance(edge, edge_oposing_point, points);
//                 TrianglesI triangles_to_look = getTrianglesToLook(edge, edge_oposing_point, triangles, points);
//                 for (const auto& candidate_point_i: candidate_points){
//                     if(!checkTriangleIntersection(edge, candidate_point_i, triangles_to_look, points)){
//                         TriangleI new_triangle = orderIndexes({edge[0], edge[1], candidate_point_i});
//                         triangles.push_back(new_triangle);
//                         EdgeStorage_addTriangleCheckingCollision(edge_storage, new_triangle, points, collision_fn, connection_fn);
//                         // edge_storage.addTriangle(new_triangle);
//                         break;
//                     }
//                 }
//             }
//         }
//     }
//     // auto edges = edge_storage.toEdges();
//     // for(const auto& edge: edges){
//     //     if(checkEdgeCollisionOrConnection({points[edge[0]], points[edge[1]]}, collision_fn, connection_fn)){
//     //         edge_storage.removeEdge(edge);
//     //     }
//     // }
//     return edge_storage;
// }
