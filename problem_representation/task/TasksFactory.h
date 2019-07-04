#ifndef TASKSFACTORY_H__
#define TASKSFACTORY_H__

#include <Tasks.h>
#include <JSON.h>
#include <memory>

class TasksFactory{
public:
    static std::unique_ptr<Tasks> make(const std::string& json);
    static std::unique_ptr<Tasks> make(JsonObject& json);
};

#endif //TASKSFACTORY_H__