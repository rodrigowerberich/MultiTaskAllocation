#ifndef EFFORTFUNCTIONFACTORY_H__
#define EFFORTFUNCTIONFACTORY_H__

#include <EffortFunction.h>
#include <JSON.h>
#include <memory>

class EffortFunctionFactory{
public:
    static std::unique_ptr<EffortFunction> make(const std::string& json);
    static std::unique_ptr<EffortFunction> make(JsonObject& json);
};

#endif //EFFORTFUNCTIONFACTORY_H__