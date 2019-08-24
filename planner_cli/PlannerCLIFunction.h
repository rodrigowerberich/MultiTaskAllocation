#ifndef PLANNERCLIFUNCTION_H__
#define PLANNERCLIFUNCTION_H__

#include <string>
#include <deque>
#include <functional>
#include <tuple>
#include <memory>

class PlannerCLIFunction{
protected:
    std::string m_name;
    size_t m_num_of_arguments;
public:
    PlannerCLIFunction(std::string name, int num_of_args):m_name{name}, m_num_of_arguments{(num_of_args<0)?0: static_cast<size_t>(num_of_args)}{}
    virtual std::string execute(std::deque<std::string> arguments) const = 0;
    const std::string & getName() const{return m_name;}
    virtual const std::string & getDescription() const = 0;
    size_t getNumOfArguments() const{return m_num_of_arguments;}
};

using PtrPlannerCLIFunction = std::unique_ptr<PlannerCLIFunction>;

template <typename ArgumentType, ArgumentType(*argument_parser)(std::deque<std::string>), std::tuple<bool, std::string>(*argument_verifier)(std::deque<std::string>)>
class PlannerCLIFunctionTemplate: public PlannerCLIFunction{
private:
    std::function<void(ArgumentType)> m_function;
    std::string m_description;
public:
    PlannerCLIFunctionTemplate(std::string name, std::string description, std::function<void(ArgumentType)> function):PlannerCLIFunction{name, std::tuple_size<ArgumentType>::value}, m_function{function}, m_description{description}{
    }
    std::string execute(std::deque<std::string> arguments) const override{
        if(  arguments.size() < PlannerCLIFunction::getNumOfArguments() ){
            return "Missing input arguments";
        }
        bool is_valid;
        std::string error_message;
        std::tie(is_valid, error_message) = argument_verifier(arguments);
        if(!is_valid){
            return error_message;
        }
        ArgumentType parsed_arguments = argument_parser(arguments);
        m_function(parsed_arguments);
        return "";
    }
    const std::string & getDescription() const override{
        return m_description;
    }


};

#endif //PLANNERCLIFUNCTION_H__