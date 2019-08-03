#ifndef HELPCLIFUNCTION_H__
#define HELPCLIFUNCTION_H__

#include <vector>
#include <unordered_map>
#include <string>
#include <PlannerCLIFunction.h>
#include <tuple>

class HelpCLIFunction: public PlannerCLIFunction{
private:
    using FunctionContainer = std::unordered_map<std::string, PtrPlannerCLIFunction>;
    std::vector<std::tuple<std::string, size_t, std::string>> m_functions;
public:
    HelpCLIFunction(std::string name, const FunctionContainer& functions);
    virtual std::string execute(std::deque<std::string> arguments) const;
    virtual const std::string & getDescription() const;
};

#endif //HELPCLIFUNCTION_H__