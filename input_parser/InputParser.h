#ifndef INPUTPARSER_H__
#define INPUTPARSER_H__

#include <string>

class InputParser
{
private:
    std::string file_name;
public:
    InputParser(int argc, char* argv[]);
    ~InputParser();
    bool inputValid() const;
    const std::string& getFileName() const;
};
#endif //INPUTPARSER_H__