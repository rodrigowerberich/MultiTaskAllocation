#include <visualize_alpha_tasks.h>
#include <iostream>
#include <sys/stat.h>
#include <ProblemRepresentation.h>
#include <AlphaTasksParser.h>

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

    std::cout << alpha_tasks_parser.isValid() << std::endl;
    std::cout << alpha_tasks_parser.getErrorMessage() << std::endl;
}