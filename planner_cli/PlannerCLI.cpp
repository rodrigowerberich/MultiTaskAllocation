#include <PlannerCLI.h>
#include <iostream>
#include <array>
#include <OneStringCLIFunction.h>
#include <HelpCLIFunction.h>
#include <parse.h>
#include <visualize.h>
#include <generate_alpha_tasks.h>

constexpr const char* kNoErrorMessage = "";

PlannerCLI::PlannerCLI(int argc, char *argv[]):m_started_empty{false}{
    if(argc == 1){
        m_started_empty = true;
    }else{
        for(int i = 1; i < argc; i++){
            m_arguments.push_back(argv[i]);
        }
    }
    // These are the possible commands
    addCLIFunction<OneStringCLIFunction>("parse", "Parse a file and prints out if it is correctly formated\nUsage: parse filename", parse);
    addCLIFunction<OneStringCLIFunction>("visualize", "Visualize the problem representation file in a 2d plot\nUsage: visualize filename", visualize);
    addCLIFunction<OneStringCLIFunction>("generate_alpha_tasks", "Calculate the alpha tasks of a given problem representation\nUsage: generate_alpha_tasks filename", generate_alpha_tasks);

    // After adding all comands, create help command
    addCLIFunction<HelpCLIFunction>("help", m_functions);
}

int PlannerCLI::run(){
    if(m_started_empty){
        m_functions["help"]->execute(m_arguments);
        return 0;
    }

    while (!m_arguments.empty()){
        // Get the output to check for a command
        auto argument = m_arguments.front();
        m_arguments.pop_front();
        auto name_function_pair = m_functions.find(argument);
        if(name_function_pair != m_functions.end()){
            // If we find a match for the argument, we process it
            auto error_message = name_function_pair->second->execute(m_arguments);
            if(error_message == kNoErrorMessage){
                // If error message is empty, we successfully processed out input and must remove the arguments used in the execution from the queue
                for(int arguments_to_remove = name_function_pair->second->getNumOfArguments(); arguments_to_remove > 0; arguments_to_remove--){
                    m_arguments.pop_front();
                }
            }else{
                // Printing out the error message
                std::cout << "\033[34m" << argument <<  ": " << "\033[31m" << error_message << "\033[m" << std::endl;
            }
        }else{
            // If not we just say it is not valid and move on to the next
            std::cout << "\033[35m" << argument << "\033[m" << " is not a valid command\n"
                << "Try help to see available commands\n ";
        }
    }
    return 0;
}
