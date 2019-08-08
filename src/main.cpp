
#define ALTERNATIVE 2

#if ALTERNATIVE == 0

#include <GnuPlotRenderer.h>

int main(int argc, char *argv[]) {

    GnuPlotRenderer renderer;
    renderer.setAxisRange(-20.0,20.0, -20.0, 20.0);

    renderer.holdOn();
    renderer.draw(drawable::Rectangle{-1.0,-1.0,5.0,3.0});
    renderer.drawRectangle({-10.0,-10.0,20.0,20.0});
    renderer.draw(drawable::Circle{-1,-5,0.5});
    renderer.drawCircle({1,1,2});
    renderer.drawCircle({0,0,10});
    renderer.drawPoint({-1,3});
    renderer.draw(drawable::Point{1,-3});
    renderer.drawNamedPoint({1,3, "R1"});
    renderer.draw(drawable::NamedPoint{-1,-3, "R2"});

    return 0;
}

#elif ALTERNATIVE == 1

#include <delaunator.h>
#include <cstdio>
#include <GnuPlotRenderer.h>
#include <TrigDefinitions.h>

static Point alpha(int i){

    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(-5, 5);
    std::uniform_real_distribution<> dist_y(-5, 5);
    Point p;
    p = {dist_x(e2), dist_y(e2)}; 
    return p;
}

int main() {
    Points points;
    std::vector<double> coords;
    for(int i = 0; i < 5; i++){
        auto p = alpha(i);
        coords.push_back(p[0]);
        coords.push_back(p[1]);
        points.push_back(Point{p[0],p[1]});
    }
    /* x0, y0, x1, y1, ... */
    GnuPlotRenderer renderer;
    renderer.setAxisRange(-5.0,5.0, -5.0, 5.0);

    renderer.holdOn();

    //triangulation happens here
    delaunator::Delaunator d(coords);

    std::vector<std::pair<double, double>> p1s;
    std::vector<std::pair<double, double>> p2s;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i]],d.coords[2 * d.triangles[i] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]));

        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i]],d.coords[2 * d.triangles[i] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]));

        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i+1]],d.coords[2 * d.triangles[i+1] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]));

        std::cout << d.coords[2 * d.triangles[i]] << ", " << d.coords[2 * d.triangles[i] + 1] << " <-> " << points[d.triangles[i]] << std::endl;
        // printf(
        //     "Triangle points: [[%f, %f], [%f, %f], [%f, %f]]\n",
        //     d.coords[2 * d.triangles[i]],        //tx0
        //     d.coords[2 * d.triangles[i] + 1],    //ty0
        //     d.coords[2 * d.triangles[i + 1]],    //tx1
        //     d.coords[2 * d.triangles[i + 1] + 1],//ty1
        //     d.coords[2 * d.triangles[i + 2]],    //tx2
        //     d.coords[2 * d.triangles[i + 2] + 1] //ty2
        // );
    }
    renderer.draw(drawable::Lines{p1s,p2s});
}

#elif ALTERNATIVE == 2

#include <PlannerCLI.h>
#include <iostream>
#include <functional>
#include <tuple>
#include <deque>
#include <PlannerCLIFunction.h>


int main(int argc, char *argv[]){
    // Planner CLI will process the input in order from left to right, ignoring unknow commands
    PlannerCLI planner(argc, argv);    
    return planner.run();
}

#elif ALTERNATIVE == 3

#include <delaunator.h>
#include <cstdio>
#include <GnuPlotRenderer.h>
#include <TrigDefinitions.h>
#include <EdgeStorage.h>
#include <ConnectGraph.h>
#include <range.h>
#include <chrono>
#include <unordered_map>
#include <Dijkstra.h>

static Point alpha(int i){

    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(-5, 5);
    std::uniform_real_distribution<> dist_y(-5, 5);
    Point p;
    p = {dist_x(e2), dist_y(e2)}; 
    return p;
}

// static std::vector<std::tuple<double, PointI>> dijkstra(const std::vector<bool>& origins, const Points& points, const EdgeStorage& edge_storage){
//     assert(origins.size() == points.size());
//     std::vector<std::tuple<double,bool, PointI, EdgesI>> points_infos(points.size());
//     for(const auto& i: util::lang::indices(points)){
//         auto value = origins[i]?0:std::numeric_limits<double>::max();
//         points_infos[i] = std::make_tuple(value, false, -1, edge_storage.getEdgesWith(i));
//     }
//     for(int curr_point_index = generate_next_point(points_infos); curr_point_index != -1; curr_point_index = generate_next_point(points_infos)){
//         for(const auto& neighbour_edge_index: std::get<3>(points_infos[curr_point_index])){
//             auto other_point_index = (neighbour_edge_index[0] == curr_point_index)? neighbour_edge_index[1]: neighbour_edge_index[0];
//             auto new_cost = std::get<0>(points_infos[curr_point_index]) + Point::euclideanDistance(points[curr_point_index], points[other_point_index]);
//             if(  new_cost < std::get<0>(points_infos[other_point_index]) ){
//                 std::get<0>(points_infos[other_point_index]) = new_cost;
//                 std::get<2>(points_infos[other_point_index]) = curr_point_index;
//             }
//         }
//         std::get<1>(points_infos[curr_point_index]) = true;
//     }
//     std::vector<std::tuple<double, PointI>> result(points.size());
//     for(const auto& i: util::lang::indices(points_infos)){
//         result[i] = std::make_tuple(std::get<0>(points_infos[i]), std::get<2>(points_infos[i]));
//     }
//     return result;
// }

int main() {
    using namespace std::chrono;
    using namespace std;
    Points points;
    std::vector<double> coords;
    constexpr int num_of_points = 30;
    std::array<bool, num_of_points> origins;
    for(int i = 0; i < num_of_points; i++){
        constexpr int num_of_origins = std::ceil(num_of_points/10.0);
        if(i < num_of_origins){
            origins[i] = true;
        }else{
            origins[i] = false;
        }
        auto p = alpha(i);
        coords.push_back(p[0]);
        coords.push_back(p[1]);
        points.push_back(Point{p[0],p[1]});
    }
    /* x0, y0, x1, y1, ... */
    GnuPlotRenderer renderer;
    renderer.setAxisRange(-5.0,5.0, -5.0, 5.0);

    renderer.holdOn();

    //triangulation happens here
    delaunator::Delaunator d(coords);

    std::vector<std::pair<double, double>> p1s;
    std::vector<std::pair<double, double>> p2s;
    EdgeStorage edge_storage;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
        TriangleI new_triangle = orderIndexes({d.triangles[i], d.triangles[i+1], d.triangles[i+2]});
        edge_storage.addTriangle(new_triangle);
        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i]],d.coords[2 * d.triangles[i] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]));

        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i]],d.coords[2 * d.triangles[i] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]));

        p1s.push_back(std::make_pair(d.coords[2 * d.triangles[i+1]],d.coords[2 * d.triangles[i+1] + 1]));
        p2s.push_back(std::make_pair(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]));
    }
    constexpr int num_of_executions = 1;
    auto start = high_resolution_clock::now(); 
    std::vector<std::tuple<double,PointI>> result;
    for(int i:util::lang::range(0,num_of_executions)){
        result = dijkstra::dijkstra(origins, points, edge_storage);
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count()/num_of_executions << endl;

    renderer.draw(drawable::Lines{p1s,p2s});
    
    for(const auto& i:util::lang::indices(points)){
        renderer.drawNamedPoint({points[i],std::to_string(i)+"->"+std::to_string(std::get<1>(result[i]))});
    }

}

#endif  