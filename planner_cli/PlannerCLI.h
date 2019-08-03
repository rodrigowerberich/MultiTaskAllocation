#ifndef PLANNERCLI_H__
#define PLANNERCLI_H__

#include <deque>
#include <vector>
#include <string>
#include <PlannerCLIFunction.h>
#include <unordered_map>

class PlannerCLI{
private:
    std::deque<std::string> m_arguments;
    std::unordered_map<std::string, PtrPlannerCLIFunction> m_functions;
    bool m_started_empty;
    template <typename T ,typename... Args>
    void addCLIFunction(const Args&... args){
        auto cli_function = std::make_unique<T>(args...);
        m_functions[cli_function->getName()] = std::move(cli_function);
    }
public:
    PlannerCLI(int argc, char *argv[]);
    int run();
};

#endif //PLANNERCLI_H__