#ifndef TASKFACTORY_H__
#define TASKFACTORY_H__

#include <Task.h>
#include <JSON.h>
#include <memory>

class TaskFactory{
public:
    static std::unique_ptr<Task> make(const std::string& json);
    static std::unique_ptr<Task> make(JsonObject& json);
};

#endif //TASKFACTORY_H__