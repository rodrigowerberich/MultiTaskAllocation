#ifndef TASKS_H__
#define TASKS_H__

#include <Task.h>
#include <MappedVectorOfPointers.h>
#include <tuple>
#include <set>

class Tasks: public MappedVectorOfPointers<Task>{
public:
    const std::unique_ptr<Task> & getTask(const std::string& task_name);    
    static std::tuple<std::set<std::string>, bool> generatePrerequisiteList(const std::unique_ptr<Tasks> & tasks, const std::unique_ptr<Task> & task, const std::unique_ptr<Task> & original_task, std::set<std::string> prerequisite_list = std::set<std::string>()){
        if(task == nullptr || original_task == nullptr){
            return std::make_tuple(prerequisite_list, false);
        }
        auto prerequisites = task->getPrerequisites();
        if(prerequisite_list.count(original_task->getName()) > 0){
            return std::make_tuple(prerequisite_list, false);
        }
        bool valid = true;
        for(const auto& prerequisite: prerequisites){
            if( prerequisite_list.count(prerequisite) > 0 ){
                valid = false;
                break;
            }
            prerequisite_list.insert(prerequisite);
            std::set<std::string> list;
            bool is_valid;
            tie(list, is_valid) = generatePrerequisiteList(tasks, tasks->getTask(prerequisite), original_task, prerequisite_list);
            prerequisite_list.insert(list.begin(), list.end());
            valid &= is_valid;
        }
        return std::make_tuple(prerequisite_list, valid);
    }
}; 

#endif // TASKS_H__