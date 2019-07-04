#include <Tasks.h>

static std::unique_ptr<Task> m_null = nullptr;

const std::unique_ptr<Task> & Tasks::getTask(const std::string& task_id){
    for (const auto & task: *this) {
        if(task->getId() == task_id){
            return task;
        }
    }  
    return m_null;
}