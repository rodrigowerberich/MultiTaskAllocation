#include <generate_alpha_tasks.h>
#include <iostream>
#include <ProblemRepresentation.h>
#include <TrigDefinitions.h>
#include <TrigHelper.h>
#include <random>
#include <GnuPlotRenderer.h>
#include <Points.h>
#include <ConnectGraph.h>
#include <gnuplot-iostream.h>
#include <PointMap.h>
#include <chrono>
#include <range.h>
#include <Dijkstra.h>
#include <DijkstraPathFinder.h>
#include <PathSmoother.h>
#include <unordered_set>

static Point alpha(int , double x_min, double x_max, double y_min, double y_max){
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
    const auto & connectivity_function = problemRepresentation.getConnectivityFunction();

    
    double x_min = bounding_box.lower_left_x;
    double x_max = bounding_box.top_right_x;
    double y_min = bounding_box.lower_left_y;
    double y_max = bounding_box.top_right_y;

    constexpr double num_of_samples = 1000;
    double circle_size = (x_max-x_min)*(y_max-y_min)/(num_of_samples);

    PointMap point_map(x_min, x_max, y_min, y_max, 10, 10);
    for(int i=0; i < num_of_samples; i++){
        for(int num_of_attempts = 0; num_of_attempts < 10 ; num_of_attempts++){
            auto point = alpha(i, x_min, x_max, y_min, y_max);
            if(!(*connectivity_function)(point) && !problemRepresentation.getObstructedArea()->containsPoint(point) && problemRepresentation.getSearchArea()->containsPoint(point) && !point_map.hasPointInRange(point, circle_size)){
                point_map.insert(point);
                break;
            }
        }
    }
    return point_map;
}

static void alpha_figure_add_edge(EdgeStorage& edges, std::size_t i_s, std::size_t j_s, bool only_outer){
    int i = static_cast<int>(i_s);
    int j = static_cast<int>(j_s);
    if( edges.containsEdge({i,j}) || edges.containsEdge({j,i})){
        if(only_outer){
            edges.removeEdge({j,i});
        }
        return;
    }
    edges.addEdge({i,j});
}

static EdgeStorage alpha_figure(const PointMap& points, double alpha, bool only_outer = true){
    std::vector<double> coords;
    for(const auto& p: points){
        coords.push_back(p[0]);
        coords.push_back(p[1]);
    }
    delaunator::Delaunator d(coords);
    EdgeStorage edges;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
        auto ia = d.triangles[i];
        auto ib = d.triangles[i+1];
        auto ic = d.triangles[i+2];
        auto pa = points[ia];
        auto pb = points[ib];
        auto pc = points[ic];
        auto circumscribed_circle = circumscribedCircle({pa,pb,pc});
        if( circumscribed_circle.radius < alpha ){
            alpha_figure_add_edge(edges, ia, ib, only_outer);
            alpha_figure_add_edge(edges, ib, ic, only_outer);
            alpha_figure_add_edge(edges, ic, ia, only_outer);
        }
    }
    return edges;
} 

static std::tuple<PointMap,EdgeStorage> generate_edge_points_of_connection_area(const ProblemRepresentation& problemRepresentation){
    const auto & connectivity_function = problemRepresentation.getConnectivityFunction();
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

    auto alpha_fig = alpha_figure(point_map, 10*circle_size);
    std::map<std::size_t, std::size_t> index_reverse_mapping;
    std::map<std::size_t, std::size_t> index_mapping;
    int new_index_count = 0;
    for(const auto& edge_i:alpha_fig.toEdges()){
        if(index_reverse_mapping.count(edge_i[0]) == 0){
            index_reverse_mapping.insert(std::make_pair(edge_i[0], new_index_count));
            index_mapping.insert(std::make_pair(new_index_count, edge_i[0]));
            new_index_count++;
        }
        if(index_reverse_mapping.count(edge_i[1]) == 0){
            index_reverse_mapping.insert(std::make_pair(edge_i[1], new_index_count));
            index_mapping.insert(std::make_pair(new_index_count, edge_i[1]));
            new_index_count++;
        }
    }
    PointMap edge_map(x_min, x_max, y_min, y_max, 10, 10);
    EdgeStorage edge_edges;
    for(auto index: util::lang::range(0,new_index_count)){
        edge_map.insert(point_map[index_mapping[index]]);
    }
    for(const auto& edge_i:alpha_fig.toEdges()){
        edge_edges.addEdge({static_cast<int>(index_reverse_mapping[edge_i[0]]),static_cast<int>(index_reverse_mapping[edge_i[1]])});
    }
    return std::make_tuple(edge_map, edge_edges);
}

Points reducePathNumberOfPoints(const Points& old_path, const ProblemRepresentation& problemRepresentation){
    if (old_path.size() < 4){
        return old_path;
    }
    Points all_points(std::begin(old_path), std::end(old_path));
    std::vector<PointI> all_points_i;
    for (size_t i = 0; i < all_points.size(); i++){
        all_points_i.push_back(i);
    }

    for(int i = 0; i < (static_cast<int32_t>(all_points_i.size()) - 3);){
        Point p{0,0};
        bool valid = false;
        std::tie(p, valid) = calculateIntersection(old_path[all_points_i[i]],old_path[all_points_i[i+1]],old_path[all_points_i[i+2]],old_path[all_points_i[i+3]]);
        bool search_area_do_not_contain_point = !problemRepresentation.getSearchArea()->containsPoint(p);
        bool point_inside_obstacle = problemRepresentation.getObstructedArea()->containsPoint(p);
        bool new_edge_colides_with_obstacle =  checkEdgeCollision({old_path[all_points_i[i]], p}, (*problemRepresentation.getObstructedArea())) || 
                                               checkEdgeCollision({old_path[all_points_i[i+3]], p}, (*problemRepresentation.getObstructedArea()));
        if(!valid || search_area_do_not_contain_point || point_inside_obstacle  || new_edge_colides_with_obstacle){
            i++;
        }else{
            all_points.push_back(p);
            all_points_i.erase(std::begin(all_points_i)+i+1, std::begin(all_points_i)+i+3);
            all_points_i.insert(std::begin(all_points_i)+i+1, all_points.size()-1);
        }
    }
    Points new_path;
    for(const auto& p_i: all_points_i){
        new_path.push_back(all_points[p_i]);
    }
    return new_path;
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
    PointMap edge_point_map;
    std::tie(edge_point_map, std::ignore) = generate_edge_points_of_connection_area(problemRepresentation);
    auto point_map = generate_points(problemRepresentation);
    Points points{point_map.begin(), point_map.end()};
    points.insert(std::begin(points), std::begin(edge_point_map), std::end(edge_point_map));
    std::vector<bool> origins(points.size());
    std::fill_n(std::begin(origins), edge_point_map.size(), true);
    renderer.drawPoints({points});
    auto start = high_resolution_clock::now(); 
    auto edge_storage = connectGraph(points, (*problemRepresentation.getObstructedArea()), (*problemRepresentation.getConnectivityFunction()));
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start); 
    cout << duration.count() << endl;
    
    auto start1 = high_resolution_clock::now(); 
    auto distance_mapping = dijkstra::dijkstra(origins, points, edge_storage);
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start1); 
    cout << duration1.count() << endl;

    point_map = PointMap(point_map.getXMin(), point_map.getXMax(), point_map.getYMin(), point_map.getYMax(), point_map.getN(), point_map.getM());
    point_map.insert(std::begin(points), std::end(points));

    dijkstra::PathFinder path_finder(point_map, edge_storage, distance_mapping);

    std::vector<std::pair<double,double>> p1s;
    std::vector<std::pair<double,double>> p2s;
    for(const auto& edge_i:edge_storage.toEdges()){
        p1s.push_back(std::make_pair(points[edge_i[0]][0], points[edge_i[0]][1]));
        p2s.push_back(std::make_pair(points[edge_i[1]][0], points[edge_i[1]][1]));
    }
    renderer.drawLines({p1s,p2s, drawable::Color::Yellow});
    problemRepresentation.draw(renderer);
    // for(const auto& i:util::lang::indices(points)){
    //     if( i < edge_point_map.size()){
    //         renderer.drawNamedPoint({points[i],std::to_string(i)});
    //     }else{
    //         renderer.drawNamedPoint({points[i],std::to_string(i)+"->"+std::to_string(std::get<1>(distance_mapping[i]))});
    //     }
    // }
    for(const auto& task: *(problemRepresentation.getTasks())){
        auto target = task->getPosition();
        if((*problemRepresentation.getConnectivityFunction())(target)){
            continue;
        }
        auto start2 = high_resolution_clock::now(); 
        auto path = smoothPath(path_finder.getPath(target), (*problemRepresentation.getObstructedArea()));
        auto stop2 = high_resolution_clock::now();
        auto duration2 = duration_cast<microseconds>(stop2 - start2); 
        cout << duration2.count() << endl;

        auto path2 = reducePathNumberOfPoints(path, problemRepresentation);

        for(const auto& i:util::lang::indices(path)){
            if( i < (path.size()-1) ){
                renderer.drawLine({path[i], path[i+1], drawable::Color::Green});
            }
            renderer.drawPoint({path[i]});
        }

        for(const auto& i:util::lang::indices(path2)){
            if( i < (path2.size()-1) ){
                renderer.drawLine({path2[i], path2[i+1], drawable::Color::DeepPink});
            }
            renderer.drawPoint({path2[i], drawable::Color::Black});
        }
    }
    std::cout << "Done\n";
}