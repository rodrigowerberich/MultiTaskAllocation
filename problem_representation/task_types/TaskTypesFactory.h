#ifndef TaskTYPESFACTORY_H__
#define TaskTYPESFACTORY_H__

#include <TaskTypes.h>
#include <JSON.h>
#include <memory>

class TaskTypesFactory{
public:
    static std::unique_ptr<TaskTypes> make(const std::string& json);
    static std::unique_ptr<TaskTypes> make(JsonObject& json);
};

#endif //TaskTYPESFACTORY_H__