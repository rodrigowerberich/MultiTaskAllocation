#ifndef TASKS_H__
#define TASKS_H__

#include <Task.h>
#include <MappedVectorOfPointers.h>

class Tasks: public MappedVectorOfPointers<Task>{
public:
    const std::unique_ptr<Task> & getTask(const std::string& task_name);    
};

#endif // TASKS_H__