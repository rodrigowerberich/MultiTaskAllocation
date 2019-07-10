#ifndef MISSION_H__
#define MISSION_H__

#include <string>
#include <set>

class Mission{
public:
    virtual const std::string & getName() const = 0;
    virtual int getPriority() const = 0;
    virtual double getDeadline() const = 0;
    virtual const std::set<std::string> & getTasks() const = 0;
    virtual bool hasTask(const std::string& task) const = 0;   
    ~Mission(){}
};
#endif // MISSION_H__