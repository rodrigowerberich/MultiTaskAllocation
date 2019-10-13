
#define ALTERNATIVE 0

#if ALTERNATIVE == 0

#include <TrigHelper.h>
#include <RRTTemplate.h>
#include <RRTStarTemplate.h>
#include <chrono>
#include <random>
#include <GnuPlotRenderer.h>
#include <cmath>
#include <vector>
#include <RobotsPosition.h>
#include <RandomRobotsPositionGenerator.h>
#include <RobotsPositionCollisionCheckVertices.h>
#include <NamedPoint.h>
#include <deque>
#include <RRTSolverTemplate.h>
#include <TimeFunction.h>

static int nearestPointRp(const std::vector<RobotsPosition>& points, const RobotsPosition& point){
    double min_distance = std::numeric_limits<double>::max();
    int min_point_index = -1;
    for(auto point_index=0; point_index < points.size(); point_index++){
        double distance = RobotsPosition::distance(point, points[point_index]);
        if(distance < min_distance){
            min_distance = distance;
            min_point_index = point_index;
        }
    }
    return min_point_index;
}

static RobotsPosition stepRp(const RobotsPosition& original, const RobotsPosition& source, int step_size){
    RobotsPosition new_robots_position(original.size());
    for(int robot_i = 0; robot_i < original.size(); robot_i++){
        Point growth_vector = original[robot_i] - source[robot_i];
        if( step_size < growth_vector.absolute() ){
            growth_vector = growth_vector*(((double)step_size)/growth_vector.absolute());   
            new_robots_position[robot_i] = source[robot_i]+growth_vector;
        }else{
            new_robots_position[robot_i] = original[robot_i];
        }
    }
    return new_robots_position;
}

static bool collision_pointRp(const Point& p){
    return false;
    // return (std::pow(p.x-300 ,2) + std::pow(p.y-300 ,2)) < 40000;
}

static bool collision_edgeRp(const RobotsPosition& rp1, const RobotsPosition& rp2){
    for(int robot_i = 0; robot_i < rp1.size(); robot_i++){
        Point p1 = rp1[robot_i];
        Point p2 = rp2[robot_i];
        Point new_edge_vector = p2 - p1;
        constexpr int NUM_OF_POINTS = 100;
        for(int i = 0; i < NUM_OF_POINTS; i++){
            Point part_of_edge = new_edge_vector*(static_cast<double>(i+1)/static_cast<double>(NUM_OF_POINTS));
            Point new_position = p1 + part_of_edge;
            if( collision_pointRp(new_position) ){
                return true;
            }
        }
    }
    return false;
}

static double randNumber(double interval_limits){
    return ((((double)rand())/RAND_MAX)-0.5)*2*interval_limits;
}

Point generateNewCandidate(size_t){
    // static std::random_device rd;
    // static std::mt19937 e2(rd());
    // std::uniform_real_distribution<> dist_x(-800, 800);
    // std::uniform_real_distribution<> dist_y(-800, 800);
    // Point p = {dist_x(e2), dist_y(e2)}; 
    Point p = {randNumber(800), randNumber(800)}; 
    return p;
}


PointI nearestPoint(const std::vector<Point>& points, const Point& point){
    double min_distance = std::numeric_limits<double>::max();
    int min_point_index = -1;
    for(auto point_index=0; point_index < points.size(); point_index++){
        double distance = Point::euclideanDistance(point, points[point_index]);
        if(distance < min_distance){
            min_distance = distance;
            min_point_index = point_index;
        }
    }
    return min_point_index;
}

Point step(const Point& original, const Point& source, double step_size){
    Point growth_vector = original - source;
    if( step_size < growth_vector.absolute() ){
        growth_vector = growth_vector*(((double)step_size)/growth_vector.absolute());
        return source+growth_vector;   
    }
    return original;
}

bool collision_point(const Point& p){
    return (std::pow(p.x-300 ,2) + std::pow(p.y-300 ,2)) < 40000;
}

bool collision_edge(const Point& p1, const Point& p2){
    Point new_edge_vector = p2 - p1;
    constexpr int NUM_OF_POINTS = 500;
    for(int i = 0; i < NUM_OF_POINTS; i++){
        Point part_of_edge = new_edge_vector*(static_cast<double>(i+1)/static_cast<double>(NUM_OF_POINTS));
        Point new_position = p1 + part_of_edge;
        if( collision_point(new_position) ){
            return true;
        }
    }
    return false;
}

std::vector<int> near(const std::vector<Point>& points, const Point& candidate_point){
    double n = points.size();
    double gamma = 3200;
    double k = gamma*std::sqrt(std::log(n)/n);
    std::vector<int> points_near_index;
    for(auto point_index = 0; point_index < n; point_index++ ){
        const auto& point = points[point_index];
        if( Point::euclideanDistance(point, candidate_point) < k ){
            points_near_index.push_back(point_index);
        }
    }
    return points_near_index;
}

int choose_parent(const std::vector<int>& points_near_index, int nearest_point_index, const Point& candidate_point, const std::vector<Point>& points, std::vector<double>& costs){
    double min_cost = std::numeric_limits<double>::max();
    int min_point_index = -1;
    for(int point_near_index: points_near_index){
        double cost = costs[point_near_index] + Point::euclideanDistance(points[point_near_index], candidate_point);
        if( (cost < min_cost) && !collision_edge(points[point_near_index], candidate_point) ){
            min_cost = cost;
            min_point_index = point_near_index;
        }
    }
    double nearest_point_cost = costs[nearest_point_index] + Point::euclideanDistance(points[nearest_point_index], candidate_point);
    if( nearest_point_cost < min_cost ){
        min_cost = nearest_point_cost;
        min_point_index = nearest_point_index;
    }
    costs.push_back(min_cost);
    return min_point_index;
}

void rewire(std::vector<Point>& vertices, std::vector<std::array<int,2>>& edges, std::vector<double> costs, const std::vector<int>& points_near_index, int parent_index, const Point& candidate_point){
    int new_point_index = vertices.size()-1;
    for(int point_near_index: points_near_index){
        double cost = costs[new_point_index] + Point::euclideanDistance(vertices[point_near_index], candidate_point);
        if( (cost < costs[point_near_index]) && !collision_edge(vertices[point_near_index], candidate_point) ){
            costs[point_near_index] = cost;
            for(auto& edge: edges){
                if( edge[1] == point_near_index ){
                    edge[0] = new_point_index;
                }
            }
        }
    }
}

using RRT = RRTTemplate<Point, decltype(generateNewCandidate), PointI(const std::vector<Point>& points, const Point& point), Point(const Point& original, const Point& source, double step_size), double , bool(const Point& p1, const Point& p2)>;
using RRTMutiple = RRTTemplate<RobotsPosition, RandomRobotsPositionGenerator, int(const std::vector<RobotsPosition>&, const RobotsPosition&), RobotsPosition(const RobotsPosition&, const RobotsPosition& source, int), int , bool(const RobotsPosition&, const RobotsPosition&)>;
using RRTStar = RRTStarTemplate<Point, decltype(generateNewCandidate), PointI(const std::vector<Point>&, const Point&), Point(const Point&, const Point&, double), double , bool(const Point&, const Point&), std::vector<int>(const std::vector<Point>&, const Point&), int(const std::vector<int>&, int, const Point&, const std::vector<Point>&, std::vector<double>&), void(std::vector<Point>&, std::vector<std::array<int,2>>&, std::vector<double>, const std::vector<int>&, int, const Point&)>;

static void drawConfiguration(const RobotsPosition& rp, const drawable::Color color, const GnuPlotRenderer& renderer){
    Points p1s;
    Points p2s;
    for(size_t i=0; i < rp.size(); i++){
        renderer.drawNamedPoint(drawable::NamedPoint(rp[i], std::to_string(i), color));
        if( i < (rp.size()-1)){
            p1s.push_back(rp[i]);
            p2s.push_back(rp[i+1]);
        }
    }
    renderer.drawLines({ p1s, p2s, color });
}

static void drawConfigurationEdge(const RobotsPosition& rp1, const RobotsPosition& rp2 , const drawable::Color color, const GnuPlotRenderer& renderer){
    Points p1s;
    Points p2s;
    for(size_t i=0; i < rp1.size(); i++){
        renderer.drawLine(drawable::Line(rp1[i], rp2[i], color));
    }
}

// template <typename RRTClass, typename ConfigurationPoint, typename DistanceFunction>
// std::deque<ConfigurationPoint> find(RRTClass& rrt, const ConfigurationPoint& goal, const DistanceFunction& distance_function, double threshhold){
//     while(true){
//         auto i = rrt.grow();
//         if(i != -1){
//             auto last_point = rrt.getVertices()[i];
//             // std::cout << "LP: " << last_point << std::endl;
//             // std::cout << distance_function(goal, last_point) << std::endl;
//             if( distance_function(goal, last_point) < threshhold ){
//                 return rrt.getPathToRoot(i);
//             }
//         }
//     }
// }

void test1(){
    using namespace std::chrono;
    auto origin = Point{0,0};
    auto goal = Point{750,750};

    RRTStar rrt_star{origin, goal, generateNewCandidate, nearestPoint, step, 100, collision_edge, near, choose_parent, rewire, 0.7};
    RRTStar rrt_star2{goal, origin, generateNewCandidate, nearestPoint, step, 100, collision_edge, near, choose_parent, rewire, 0.5};
    GnuPlotRenderer renderer;

    renderer.setAxisRange(-800,800, -800, 800);

    renderer.holdOn();
    renderer.drawCircle({300,300,200});

    auto&& distance_function = Point::euclideanDistance;
    auto threshhold = 1;

    std::deque<Point> path1;
    std::deque<Point> path2;

    auto* Ta = &rrt_star;
    auto* Tb = &rrt_star2;
    int i=0;
    bool met = false;
    for(; !met; i++){
        std::cout << "i: " << i << std::endl;
        std::cout << "Press enter to continue " << i << std::endl;
        std::cin.get();
        Ta->setGoalSearchPercentage(0);
        Tb->setGoalSearchPercentage(1);
        auto new_point_Ta_i = Ta->grow();
        std::cout << "new_point_Ta_i: " << new_point_Ta_i << std::endl;
        if( new_point_Ta_i != -1 ){
            auto&& Ta_vertex = Ta->getVertices()[new_point_Ta_i];
            std::cout << "new_point_Ta_i: " << Ta_vertex << std::endl;
            renderer.drawPoint(Ta_vertex);
            auto&& Ta_edges = Ta->getEdges();
            renderer.drawLine({Ta->getVertices()[Ta_edges.back()[0]], Ta->getVertices()[Ta_edges.back()[1]]});
            Tb->setGoal(Ta->getVertices()[new_point_Ta_i]);
            auto new_point_Tb_i = Tb->grow();
            std::cout << "new_point_Tb_i: " << new_point_Tb_i << std::endl;
            if( new_point_Tb_i != -1 ){
                auto&& Tb_vertex = Tb->getVertices()[new_point_Tb_i];
                std::cout << "new_point_Tb_i: " << Tb_vertex << std::endl;
                renderer.drawPoint(Tb_vertex);
                auto&& Tb_edges = Tb->getEdges();
                renderer.drawLine({Tb->getVertices()[Tb_edges.back()[0]], Tb->getVertices()[Tb_edges.back()[1]]});
                auto distance = distance_function(Ta->getVertices()[new_point_Ta_i], Tb->getVertices()[new_point_Tb_i]);
                std::cout << "distance: " << distance << std::endl; 
                if(  distance < threshhold ){
                    path1 = Ta->getPathToRoot(new_point_Ta_i);
                    path2 = Tb->getPathToRoot(new_point_Tb_i);
                    met = true;
                    break;
                }
            }
        }
        if( Ta->getVertices().size() > Tb->getVertices().size() ){
            std::swap(Ta, Tb);
            std::cout << "Swapping!\n";
        }
    }
    // std::vector<ConfigurationPoint> path;
    // path.insert(std::end(path), std::begin(path1), std::end(path1)-1);
    // path.insert(std::end(path), std::rbegin(path2), std::rend(path2));
}

int main(int argc, char *argv[]) {
    srand(time(0));
    using namespace std::chrono;

    // // // auto origin = RobotsPosition(3, {{100,-100},{200,-100}, {300, -100}});
    // // // auto goal = RobotsPosition(3, {{550,700},{650,700}, {750, 700}});
    auto origin = Point{0,0};
    auto goal = Point{750,750};

    // // // // // RRTMutiple rrt( origin, goal, RandomRobotsPositionGenerator(3, 800), nearestPointRp, stepRp, 100, collision_edgeRp, 0.35);
    // // // // // RRTMutiple rrt2( goal, origin, RandomRobotsPositionGenerator(3, 800), nearestPointRp, stepRp, 100, collision_edgeRp, 0.35);


    // // // // constexpr int num_of_attempts = 1;
    // // // // auto start = high_resolution_clock::now(); 
    // // // // auto generator = RandomRobotsPositionGenerator(3, -800, 800, -800, 800);
    // // // // // for(int i=0; i < num_of_attempts; i++){
    // // //     // RRT rrt{origin, goal, generateNewCandidate, nearestPoint, step, 100, collision_edge, 0.5};
    // // //     // RRT rrt2{goal, origin, generateNewCandidate, nearestPoint, step, 100, collision_edge, 0.5};
    RRTStar rrt_star{origin, goal, generateNewCandidate, nearestPoint, step, 100, collision_edge, near, choose_parent, rewire, 0};
    RRTStar rrt_star2{goal, origin, generateNewCandidate, nearestPoint, step, 100, collision_edge, near, choose_parent, rewire, 0.5};
    // //     // auto found = timeFunction(rrt_solver::find<RRT, Point, double(const Point&, const Point&)>, rrt, Point{750,750}, Point::euclideanDistance, 1);
    auto found = timeFunction(rrt_solver::findBi<RRTStar, Point, decltype(Point::euclideanDistance)>, rrt_star, rrt_star2, goal, Point::euclideanDistance, 1);
    // //     // auto found = rrt_solver::findBi(rrt, rrt2, goal, Point::euclideanDistance, 1);
    // // auto found = rrt_solver::findBi(rrt_star, rrt_star2, goal, Point::euclideanDistance, 1);

    // // //     // RRTMutiple rrt( origin, goal, generator, nearestPointRp, stepRp, 100, collision_edgeRp, 0.5);
    // // //     // RRTMutiple rrt2( goal, origin, generator, nearestPointRp, stepRp, 100, collision_edgeRp, 0.35);
    // // //     // auto found = find(rrt, goal, RobotsPosition::distance, 1);
    // //     // auto found = rrt_solver::findBi(rrt, rrt2, goal, RobotsPosition::distance, 1);

    // // // // }
    // // // auto stop = high_resolution_clock::now(); 
    // // // auto duration = duration_cast<microseconds>(stop - start); 

    GnuPlotRenderer renderer;
    renderer.setAxisRange(-800,800, -800, 800);


    renderer.holdOn();

    for(int i=0; i < found.size(); i++){
        std::cout << found[i] << ", ";
        renderer.drawPoint(found[i]);
        if( i < (found.size()-1) ){
            renderer.drawLine({found[i], found[i+1]});
        }
    }

    // // // for(int i=0; i < found.size(); i++){
    // // //     std::cout << found[i] << ", ";
    // // //     // drawConfiguration(rrt.getVertices()[found[i]], drawable::Color::Blue, renderer);
    // // //     if( i < (found.size()-1) ){
    // // //         drawConfigurationEdge(found[i], found[i+1], drawable::Color::Red, renderer);
    // // //     }
    // // // }

    // // // std::cout << std::endl;
    // // // std::cout << "Took " << duration.count()/num_of_attempts << " microseconds\n";


    renderer.drawCircle({300,300,200});



    // rrt_star.setGoalSearchPercentage(1);
    // auto start_star = high_resolution_clock::now(); 
    // for(int i=0; i < 1000; i++){
    //     rrt_star.grow();
    // }
    // auto stop_star = high_resolution_clock::now(); 
    // auto duration_star = duration_cast<microseconds>(stop_star - start_star); 

    // // const auto & points = rrt.getVertices();
    // // const auto & edges_i = rrt.getEdges();

    // const auto & points_star = rrt_star.getVertices();
    // const auto & edges_star_i = rrt_star.getEdges();


    // // Points p1s;
    // // Points p2s;

    // Points p1_stars;
    // Points p2_stars;

    // // for(const auto& e: edges_i){
    // //     // std::cout << e << std::endl;
    // //     p1s.push_back(points[e[0]]);
    // //     p2s.push_back(points[e[1]]);
    // // }

    // for(const auto& e: edges_star_i){
    //     p1_stars.push_back(points_star[e[0]]);
    //     p2_stars.push_back(points_star[e[1]]);
    // }

    // // std::cout << duration.count() << std::endl;
    // std::cout << duration_star.count() << std::endl;
    // GnuPlotRenderer renderer;
    // renderer.setAxisRange(-800,800, -800, 800);

    // renderer.holdOn();

    // // for(int i=0; i < rrt.getVertices().size(); i++){
    // //     drawConfiguration(rrt.getVertices()[i], drawable::Color::Blue, renderer);
    // // }
    // // for(int i=0; i < rrt.getEdges().size(); i++){
    // //     drawConfigurationEdge(rrt.getVertices()[rrt.getEdges()[i][0]], rrt.getVertices()[rrt.getEdges()[i][1]], drawable::Color::Red, renderer);
    // // }

    // // // renderer.drawPoints(points);
    // // // renderer.drawLines({p1s,p2s});

    // renderer.drawPoints(points_star);
    // renderer.drawLines({p1_stars,p2_stars});

    // renderer.drawCircle({300,300,200});

    // // auto generator = GenerateRandomRobotPosition(3);

    // // std::cout << generator(1).position << std::endl;
    // // std::cout << generator(1).position << std::endl;
    // // std::cout << generator(1).position << std::endl;

    // // renderer.drawPoints(generator(1).position);
    

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
#include <DijkstraPathFinder.h>
#include <PointMap.h>

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
    constexpr int num_of_points = 10000;
    constexpr int num_of_origins = 1;//std::ceil(num_of_points/10.0);
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
    dijkstra::PathFinder path_finder(point_map, edge_storage, result);
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

#elif ALTERNATIVE == 4  

#include <delaunator.h>
#include <cstdio>
#include <GnuPlotRenderer.h>
#include <TrigDefinitions.h>
#include <TrigHelper.h>
#include <EdgeStorage.h>
#include <ConnectGraph.h>
#include <range.h>
#include <chrono>
#include <unordered_map>

void calculateAndPlotIntersection(const Point& p1, const Point& q1, const Point& p2, const Point& q2, GnuPlotRenderer & renderer){
    Point p{0,0};
    bool valid = false;

    std::tie(p, valid) = calculateIntersection(p1,q1,p2,q2);
    if(!valid){
        std::cout << "Lines {" << p1 << ", " << q1 << "} and {" << p2 << ", " << q2 <<"} do not intersect" << std::endl;
    }else{
        std::cout << "Lines {" << p1 << ", " << q1 << "} and {" << p2 << ", " << q2 <<"} intersect at " << p << std::endl;
        renderer.drawLine({p1,q1});
        renderer.drawLine({p2,q2});
        renderer.drawPoint({p});
    }
}


int main() {
    using namespace std::chrono;
    using namespace std;
    GnuPlotRenderer renderer;
    renderer.setAxisRange(-5.0,5.0, -5.0, 5.0);

    renderer.holdOn();

    calculateAndPlotIntersection({2,3}, {2,2}, {-1,4}, {-1,-4}, renderer);
    calculateAndPlotIntersection({1,2}, {3,4}, {3,2}, {5,1}, renderer);
    calculateAndPlotIntersection({2,3}, {2,2}, {3,2}, {5,1}, renderer);
    calculateAndPlotIntersection({1,2}, {3,4}, {-1,4}, {-1,-4}, renderer);
    calculateAndPlotIntersection({0,3}, {1,4}, {0,7}, {1,6}, renderer);
    calculateAndPlotIntersection({0,3}, {1,4}, {1,2}, {3,4}, renderer);

}

#endif  