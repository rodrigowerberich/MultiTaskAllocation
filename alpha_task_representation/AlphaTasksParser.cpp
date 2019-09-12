#include <AlphaTasksParser.h>
#include <fstream>
#include <JSON.h>
#include <AlphaTaskToJson.h>
#include <sstream>
#include <PathSmoother.h>

static std::string read_file(std::string file_name){
    try{
        std::ifstream file(file_name);
        std::string content;
        while(file.good()){
            std::string line;
            file >> line;
            content.append(line);
        }
        file.close();
        return content;   
    }
    catch(const std::ios_base::failure& e){
        return "";
    }
}

bool AlphaTasksParser::parseJsonRepresentation(const std::string & json_object){
    m_alpha_tasks = jsonifier::fromJson<AlphaTasks>(json_object);
    if(m_alpha_tasks.empty()){
        return false;
    }
    for(const auto& task: m_alpha_tasks){
        if(!task.isValid()){
            return false;
        }
    }
    return true;
}

void AlphaTasksParser::evaluateAlphaTasks(){
    for(const auto& task: m_alpha_tasks){
        if( m_problem_representation.getTasks()->getTask(task.getTask()) == nullptr ){
            m_error_message = "\"" + task.getTask() + "\" is not a valid task in the problem representation file provided!";
            m_is_valid = false;
            return;
        }
        for( size_t i = 0; i < task.size(); i++){
            const auto& position = task[i];
            if( m_problem_representation.getObstructedArea()->containsPoint(position) ){
                std::stringstream string_stream;
                string_stream << "\"" << task.getTask() << "\"" << " task with position " << position << " is colliding with an obstacle";
                m_error_message = string_stream.str();
                m_is_valid = false;
                return;
            }
            if( !m_problem_representation.getSearchArea()->containsPoint(position) ){
                std::stringstream string_stream;
                string_stream << "\"" << task.getTask() << "\"" << " task with position " << position << " is outside search area";
                m_error_message = string_stream.str();
                m_is_valid = false;
                return;
            }
            if( i < (task.size() - 1)){
                const auto& next_position = task[i+1];
                if(checkEdgeCollision({ position, next_position }, (*m_problem_representation.getObstructedArea()))){
                    std::stringstream string_stream;
                    string_stream << "\"" << task.getTask() << "\"" << " task with edge" << position << " and " << next_position << " collides with an obstacle";
                    m_error_message = string_stream.str();
                    m_is_valid = false;
                    return;
                }
            } else {
                const auto& next_position = m_problem_representation.getTasks()->getTask(task.getTask())->getPosition();
                if(checkEdgeCollision({ position, next_position }, (*m_problem_representation.getObstructedArea()))){
                    std::stringstream string_stream;
                    string_stream << "\"" << task.getTask() << "\"" << " task with edge" << position << " and " << next_position << " collides with an obstacle";
                    m_error_message = string_stream.str();
                    m_is_valid = false;
                    return;
                }               
            }
        }
    }
    m_is_valid = true;
}


AlphaTasksParser::AlphaTasksParser(const std::string& file_name, const ProblemRepresentation& problem_representation):m_problem_representation{problem_representation}, m_is_valid{false}{
    auto file_content = read_file(file_name);
    m_is_valid = parseJsonRepresentation(file_content);
    if(m_is_valid){
        evaluateAlphaTasks();
    }else{
        m_error_message = "Problem parsing file, file is not a valid JSON!";
    }
}

bool AlphaTasksParser::isValid() const{
    return m_is_valid;
}

AlphaTasks AlphaTasksParser::getAlphaTasks() const{
    return m_alpha_tasks;
}

const std::string& AlphaTasksParser::getErrorMessage() const{
    return m_error_message;
}

