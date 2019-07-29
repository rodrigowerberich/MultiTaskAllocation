#ifndef INPUTPARSER_H__
#define INPUTPARSER_H__

#include <string>

class InputParser
{
private:
    std::string m_file_name;
    bool m_show_problem;
public:
    InputParser(int argc, char* argv[]);
    ~InputParser();
    bool inputValid() const;
    const std::string& getFileName() const;
    bool showProblem() const;
};
#endif //INPUTPARSER_H__