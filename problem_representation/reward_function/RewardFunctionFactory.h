#ifndef REWARDFUNCTIONFACTORY_H__
#define REWARDFUNCTIONFACTORY_H__

#include <RewardFunction.h>
#include <JSON.h>
#include <memory>

class RewardFunctionFactory{
public:
    static std::unique_ptr<RewardFunction> make(const std::string& json);
    static std::unique_ptr<RewardFunction> make(JsonObject& json);
};

#endif //REWARDFUNCTIONFACTORY_H__