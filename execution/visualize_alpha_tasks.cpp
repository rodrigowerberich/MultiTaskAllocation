#include <visualize_alpha_tasks.h>
#include <iostream>
#include <sys/stat.h>
#include <ProblemRepresentation.h>
#include <AlphaTasksParser.h>
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

void visualize_alpha_tasks(std::tuple <std::string> values){
    auto file_name = std::get<0>(values);
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
    AlphaTasksParser alpha_tasks_parser{alpha_file_name, problemRepresentation};
    if(!alpha_tasks_parser.isValid()){
        warn_error_in_alpha_task_file(alpha_tasks_parser);
        return;
    }
    AlphaTasks alpha_tasks = alpha_tasks_parser.getAlphaTasks();

    GnuPlotRenderer renderer;
    renderer.holdOn();
    auto bounding_box = problemRepresentation.getSearchArea()->getDrawable()->getBoundingBox();
    renderer.setAxisRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);
    problemRepresentation.draw(renderer);
    for(const auto& alpha_task: alpha_tasks){
        std::vector<Position2d> p1s;
        std::vector<Position2d> p2s;
        std::vector<Position2d> points;
        for(size_t i = 0; i < alpha_task.size(); i++){
            const auto& position = alpha_task[i];
            points.push_back(position);
            if( i < (alpha_task.size() - 1) ){
                const auto& next_position = alpha_task[i+1];
                p1s.push_back(position);
                p2s.push_back(next_position);
            }else{
                const auto& next_position = problemRepresentation.getTasks()->getTask(alpha_task.getTask())->getPosition();
                p1s.push_back(position);
                p2s.push_back(next_position);                
                points.push_back(next_position);
            }
        }
        renderer.drawLines({p1s, p2s, drawable::Color::DeepPink});
        renderer.drawPoints({points, drawable::Color::Black});
    }
    renderer.holdOn(false);
    

}