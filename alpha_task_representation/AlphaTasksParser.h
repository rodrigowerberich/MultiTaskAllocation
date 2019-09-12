#ifndef ALPHATASKSPARSER_H__
#define ALPHATASKSPARSER_H__

#include <JSON.h>
#include <AlphaTasks.h>
#include <ProblemRepresentation.h>

class AlphaTasksParser{
private:
    const ProblemRepresentation& m_problem_representation;
    bool m_is_valid;
    AlphaTasks m_alpha_tasks;
    std::string m_error_message;

    bool parseJsonRepresentation(const std::string & json_object);
    void evaluateAlphaTasks();
public:
    AlphaTasksParser(const std::string& file_name, const ProblemRepresentation& problem_representation);
    bool isValid() const;
    AlphaTasks getAlphaTasks() const;
    const std::string& getErrorMessage() const;
};


#endif //ALPHATASKSPARSER_H__