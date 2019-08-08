
#define ALTERNATIVE 3

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
#include <PointMap.h>

class PathFinder{
private:
    PointMap m_point_map;
    EdgeStorage m_edge_storage;
    dijkstra::Result m_result;
public:
    PathFinder(PointMap point_map, EdgeStorage edge_storage, dijkstra::Result result):m_point_map{point_map},m_edge_storage{edge_storage},m_result{result}{}
    // Points getPath(const Point& interest_point){
    //     Points path;
    //     path.insert(std::begin(path),interest_point);
    //     PointI nearest_point_index = m_point_map.nearestPointIndex(interest_point);
    //     path.insert(std::begin(path),m_point_map[nearest_point_index]);
    //     PointI next_point_index = nearest_point_index;
    //     while(std::get<1>(m_result[next_point_index])!=-1){
    //         next_point_index = std::get<1>(m_result[next_point_index]);
    //         path.insert(std::begin(path),m_point_map[next_point_index]);
    //     }
    //     return path;
    // }
    Points getPath(const Point& interest_point){
        Points path;
        path.insert(std::begin(path),interest_point);
        PointI nearest_point_index = m_point_map.nearestPointIndex(interest_point);
        double radius = Point::euclideanDistance(interest_point, m_point_map[nearest_point_index]);
        PointsI nearest_points_index = m_point_map.pointsInRangeByIndex(interest_point, 2*radius);
        double min_value = std::numeric_limits<double>::max();
        for(const auto& pi: nearest_points_index){
            double new_value = Point::euclideanDistance(interest_point, m_point_map[pi]) + std::get<0>(m_result[pi]);
            if(new_value < min_value){
                min_value = new_value;
                nearest_point_index = pi;
            }
        }
        path.insert(std::begin(path),m_point_map[nearest_point_index]);
        PointI next_point_index = nearest_point_index;
        while(std::get<1>(m_result[next_point_index])!=-1){
            next_point_index = std::get<1>(m_result[next_point_index]);
            path.insert(std::begin(path),m_point_map[next_point_index]);
        }
        return path;
    }
};

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
    using namespace std::chrono;
    using namespace std;
    Points points;
    std::vector<double> coords;
    constexpr int num_of_points = 1000;
    constexpr int num_of_origins = std::ceil(num_of_points/10.0);
    std::array<bool, num_of_points> origins;
    for(int i = 0; i < num_of_points; i++){
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
    auto result = dijkstra::dijkstra(origins, points, edge_storage);

    renderer.draw(drawable::Lines{p1s,p2s});

    PointMap point_map(-5,5,-5,5,10,10);
    point_map.insert(std::begin(points), std::end(points));
    
    constexpr int num_of_executions = 10;
    auto start = high_resolution_clock::now(); 
    for(int i:util::lang::range(0,num_of_executions)){
        point_map.nearestPoint({0,0});
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count()/num_of_executions << endl;

    // for(const auto& i:util::lang::indices(points)){
    //     renderer.drawNamedPoint({points[i],std::to_string(i)});
    // }
    PathFinder path_finder(point_map, edge_storage, result);
    auto path = path_finder.getPath({0,0});
    for(const auto& i:util::lang::indices(path)){
        if( i < (path.size()-1) ){
            renderer.drawLine({path[i], path[i+1], drawable::Color::Green});
        }
        // renderer.drawNamedPoint({path[i],std::to_string(i)});
        renderer.drawPoint({path[i]});
    }
    // auto path2 = path_finder.getPath2({0,0});
    // for(const auto& i:util::lang::indices(path2)){
    //     if( i < (path2.size()-1) ){
    //         renderer.drawLine({path2[i], path2[i+1], drawable::Color::Blue});
    //     }
    //     renderer.drawNamedPoint({path[i],std::to_string(i)});
    //     renderer.drawPoint({path2[i], drawable::Color::Blue});
    // }
    for(const auto& i:util::lang::range(0,num_of_origins)){
        renderer.drawPoint({points[i], drawable::Color::Red});
    }

}

#endif  