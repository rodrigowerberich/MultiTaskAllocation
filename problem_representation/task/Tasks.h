#ifndef TASKS_H__
#define TASKS_H__

#include <vector>
#include <Task.h>
#include <memory>
#include <algorithm>

// using Tasks = std::vector<std::unique_ptr<Task>>;

class Tasks: public std::vector<std::unique_ptr<Task>>{
public:
    const std::unique_ptr<Task> & getTask(const std::string& task_name);
};


#endif // TASKS_H__