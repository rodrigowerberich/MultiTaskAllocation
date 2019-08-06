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



static Point alpha(int i, const ProblemRepresentation& problemRepresentation){
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();

    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(bounding_box.lower_left_x, bounding_box.top_right_x);
    std::uniform_real_distribution<> dist_y(bounding_box.lower_left_y, bounding_box.top_right_y);
    Point p;
    do{
        p = {dist_x(e2), dist_y(e2)}; 
    }while(problemRepresentation.getObstructedArea()->containsPoint(p) || !problemRepresentation.getSearchArea()->containsPoint(p));
    return p;
}

static PointMap generate_points(const ProblemRepresentation& problemRepresentation){
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    
    constexpr double num_of_samples = 1000;
    double circle_size = (bounding_box.top_right_x-bounding_box.lower_left_x)*(bounding_box.top_right_y-bounding_box.lower_left_y)/(0.5*num_of_samples);
    std::cout << circle_size << std::endl;
    std::cout << bounding_box.top_right_x-bounding_box.lower_left_x << std::endl;
    std::cout << bounding_box.top_right_y-bounding_box.lower_left_y << std::endl;

    PointMap point_map(bounding_box.lower_left_x, bounding_box.top_right_x, bounding_box.lower_left_y, bounding_box.top_right_y, 10, 10);
    for(int i=0; i < 1000; i++){
        for(bool pushed = false; !pushed;){
            auto point = alpha(i, problemRepresentation);
            if(!point_map.hasPointInRange(point, circle_size)){
                point_map.insert(point);
                pushed = true;
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
    auto start = high_resolution_clock::now();
    Points points{point_map.begin(), point_map.end()};
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count() << endl; 
    renderer.drawPoints({points});
    auto edge_storage = connectGraph(points, (*problemRepresentation.getObstructedArea()), (*problemRepresentation.getConnectivityFunction()));
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