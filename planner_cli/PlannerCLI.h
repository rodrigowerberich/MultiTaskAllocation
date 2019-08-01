#ifndef PLANNERCLI_H__
#define PLANNERCLI_H__

#include <deque>
#include <vector>
#include <string>
#include <PlannerCLIFunction.h>

class PlannerCLI{
private:
    std::deque<std::string> m_arguments;
    std::vector<PtrPlannerCLIFunction> m_functions;
    bool m_started_empty;
    template <typename T ,typename... Args>
    void addCLIFunction(const Args&... args){
        m_functions.push_back(std::move(std::make_unique<T>(args...)));
    }
public:
    PlannerCLI(int argc, char *argv[]);
    int run();
};

#endif //PLANNERCLI_H__