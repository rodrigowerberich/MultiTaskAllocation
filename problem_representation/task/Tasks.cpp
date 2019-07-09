#include <Tasks.h>

const std::unique_ptr<Task> & Tasks::getTask(const std::string& task_id){
    return getValueByName(task_id);
}