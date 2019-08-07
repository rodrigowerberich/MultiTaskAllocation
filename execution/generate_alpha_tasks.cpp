#include <generate_alpha_tasks.h>
#include <iostream>
#include <ProblemRepresentation.h>
#include <TrigDefinitions.h>
#include <random>
#include <GnuPlotRenderer.h>
#include <Points.h>
#include <ConnectGraph.h>
#include <gnuplot-iostream.h>
#include <PointMap.h>
#include <chrono>
#include <range.h>
#include <delaunator.h>



static Point alpha(int i, double x_min, double x_max, double y_min, double y_max){
    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(x_min, x_max);
    std::uniform_real_distribution<> dist_y(y_min, y_max);
    Point p = {dist_x(e2), dist_y(e2)}; 
    return p;
}

static PointMap generate_points(const ProblemRepresentation& problemRepresentation){
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    
    double x_min = bounding_box.lower_left_x;
    double x_max = bounding_box.top_right_x;
    double y_min = bounding_box.lower_left_y;
    double y_max = bounding_box.top_right_y;

    constexpr double num_of_samples = 10000;
    double circle_size = (x_max-x_min)*(y_max-y_min)/(num_of_samples);

    PointMap point_map(x_min, x_max, y_min, y_max, 10, 10);
    for(int i=0; i < num_of_samples; i++){
        for(int num_of_attempts = 0; num_of_attempts < 10 ; num_of_attempts++){
            auto point = alpha(i, x_min, x_max, y_min, y_max);
            if(!problemRepresentation.getObstructedArea()->containsPoint(point) && problemRepresentation.getSearchArea()->containsPoint(point) && !point_map.hasPointInRange(point, circle_size)){
                point_map.insert(point);
                break;
            }
        }
    }
    return point_map;
}

static PointMap generate_edge_points_of_connection_area(const ProblemRepresentation& problemRepresentation){
    const auto & connectivity_function = problemRepresentation.getConnectivityFunction();
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = connectivity_function->getDrawable()->getBoundingBox();
    
    double x_min = bounding_box.lower_left_x;
    double x_max = bounding_box.top_right_x;
    double y_min = bounding_box.lower_left_y;
    double y_max = bounding_box.top_right_y;

    constexpr double num_of_samples = 5000;
    double circle_size = (x_max-x_min)*(y_max-y_min)/(num_of_samples);

    PointMap point_map(x_min, x_max, y_min, y_max, 10, 10);
    for(int i=0; i < num_of_samples; i++){
        for(int num_of_attempts = 0; num_of_attempts < 100 ; num_of_attempts++){
            auto point = alpha(i, x_min, x_max, y_min, y_max);
            if((*connectivity_function)(point) && problemRepresentation.getSearchArea()->containsPoint(point) && !point_map.hasPointInRange(point, circle_size)){
                point_map.insert(point);
                break;
            }
        }
    }
    return point_map;
}

void generate_alpha_tasks(std::tuple <std::string> values){
    using namespace std;
    using namespace std::chrono;
    using namespace util::lang;
    auto file_name = std::get<0>(values);
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
        return;
    }
    GnuPlotRenderer renderer;
    renderer.holdOn();
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    renderer.setAxisRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);


    auto point_map = generate_points(problemRepresentation);
    Points points{point_map.begin(), point_map.end()};
    renderer.drawPoints({points});
    auto start = high_resolution_clock::now(); 
    auto edge_storage = connectGraph2(points, (*problemRepresentation.getObstructedArea()), (*problemRepresentation.getConnectivityFunction()));
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count() << endl;
    std::vector<std::pair<double,double>> p1s;
    std::vector<std::pair<double,double>> p2s;
    for(const auto& edge_i:edge_storage.toEdges()){
        p1s.push_back(std::make_pair(points[edge_i[0]][0], points[edge_i[0]][1]));
        p2s.push_back(std::make_pair(points[edge_i[1]][0], points[edge_i[1]][1]));
    }
    renderer.drawLines({p1s,p2s, drawable::Color::Yellow});
    problemRepresentation.draw(renderer);
    std::cout << "Done\n";
}

void generate_alpha_tasks2(std::tuple <std::string> values){
    using namespace std;
    using namespace std::chrono;
    using namespace util::lang;
    auto file_name = std::get<0>(values);
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
        return;
    }
    GnuPlotRenderer renderer;
    renderer.holdOn();
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    renderer.setAxisRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);
#if 1
    auto point_map = generate_edge_points_of_connection_area(problemRepresentation);
    Points points{point_map.begin(), point_map.end()};
    renderer.drawPoints({points});
    problemRepresentation.draw(renderer);
#else
    auto point_map = generate_points(problemRepresentation);
    Points points{point_map.begin(), point_map.end()};
    renderer.drawPoints({points});
    auto start = high_resolution_clock::now(); 
    auto edge_storage = connectGraph2(points, (*problemRepresentation.getObstructedArea()), (*problemRepresentation.getConnectivityFunction()));
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count() << endl;
    std::vector<std::pair<double,double>> p1s;
    std::vector<std::pair<double,double>> p2s;
    for(const auto& edge_i:edge_storage.toEdges()){
        p1s.push_back(std::make_pair(points[edge_i[0]][0], points[edge_i[0]][1]));
        p2s.push_back(std::make_pair(points[edge_i[1]][0], points[edge_i[1]][1]));
    }
    renderer.drawLines({p1s,p2s, drawable::Color::Yellow});
    problemRepresentation.draw(renderer);
    std::cout << "Done\n";
#endif
}