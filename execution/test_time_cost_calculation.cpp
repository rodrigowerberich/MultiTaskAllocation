#include "test_time_cost_calculation.h"
#include <iostream>
#include <ProblemRepresentation.h>
#include <AlphaTasksParser.h>
#include <sys/stat.h>
#include <TrigDefinitions.h>
#include <RRT.h>
#include <GnuPlotRenderer.h>


static inline std::string generate_alpha_task_file_name(const std::string& file_name){
    std::string alpha_file_name = file_name;
    return alpha_file_name.replace(std::begin(alpha_file_name)+alpha_file_name.find_last_of('.'), std::end(alpha_file_name), ".alpha.json");
}

static inline bool file_exists(const std::string& name){
    struct stat buffer;
    return ( stat (name.c_str(), &buffer) == 0 );
}

static inline void warn_alpha_task_file_does_not_exist(const std::string& alpha_file_name){
    std::cout << "Missing alpha task file\n";
    std::cout << alpha_file_name << " not found!\n";
}

static inline void warn_error_in_problem_representation_file(const ProblemRepresentation& problemRepresentation){
    std::cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << std::endl;
    std::cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << std::endl;
}

static inline void warn_error_in_alpha_task_file(const AlphaTasksParser& alpha_tasks_parser){
    std::cout << "\033[1;31mSomething is wrong with the alpha tasks file\033[0m" << std::endl;
    std::cout << "\033[1;31m"<< alpha_tasks_parser.getErrorMessage() << "\033[0m" << std::endl;
}

static inline void warn_task_does_not_exist(const std::string& task_name ){
    std::cout << "\033[1;31mTask with name \"" << task_name << "\" does not exist in this problem!\033[0m" << std::endl;
}

static inline Points get_complete_alpha_task_position(ProblemRepresentation& problemRepresentation, AlphaTasks& alpha_tasks, std::string task_name){
    Points complete_task_points;
    for(const auto& alpha_task:alpha_tasks){
        if( alpha_task.getTask() == task_name ){
            for(const auto& position: alpha_task){
                complete_task_points.push_back(position);
            }
        }
    }
    complete_task_points.push_back(problemRepresentation.getTasks()->getTask(task_name)->getPosition());
    return complete_task_points;
}

void test_time_cost_calculation(std::tuple <std::string, std::string> values){
    auto file_name = std::get<0>(values);
    auto task_name = std::get<1>(values);

    auto alpha_file_name = generate_alpha_task_file_name(file_name);
    if( !file_exists(alpha_file_name) ){
        warn_alpha_task_file_does_not_exist(alpha_file_name);
        return;
    }
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        warn_error_in_problem_representation_file(problemRepresentation);
        return;
    }
    if(problemRepresentation.getTasks()->getTask(task_name) == nullptr){
        warn_task_does_not_exist(task_name);
        return;
    }
    AlphaTasksParser alpha_tasks_parser{alpha_file_name, problemRepresentation};
    if(!alpha_tasks_parser.isValid()){
        warn_error_in_alpha_task_file(alpha_tasks_parser);
        return;
    }
    AlphaTasks alpha_tasks = alpha_tasks_parser.getAlphaTasks();

    Points complete_task_points = get_complete_alpha_task_position(problemRepresentation, alpha_tasks, task_name);
    std::cout << complete_task_points << std::endl;

    GnuPlotRenderer renderer;
    renderer.holdOn();
    const auto & search_area = problemRepresentation.getSearchArea();
    auto bounding_box = search_area->getDrawable()->getBoundingBox();
    renderer.setAxisRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);

    RRT<int> rrt {{0,0}, {0,0}, 1, {bounding_box.lower_left_x, bounding_box.lower_left_y}, {bounding_box.top_right_x, bounding_box.top_right_y}, 10};
    
    renderer.drawPoint({rrt.getVertices().back()});

    for(int i=0; i < 1000; i++){
        rrt.grow();
        renderer.drawPoint({rrt.getVertices().back()});
        auto edgesI = rrt.getEdges().getEdgesWith(rrt.getVertices().size()-1);
        renderer.drawLine({rrt.getVertices()[edgesI[0][0]], rrt.getVertices()[edgesI[0][1]]});
    }

}
