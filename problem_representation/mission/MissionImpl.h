#ifndef MISSION_IMPL_H__
#define MISSION_IMPL_H__

#include <Mission.h>

class MissionImpl: public Mission {
private:
    std::string m_name;
    int m_priority;
    double m_deadline;
    std::set<std::string> m_tasks_names;
public:
    MissionImpl(const std::string & name, int priority, double deadline, const std::set<std::string> & tasks_names);
    const std::string & getName() const;    
    int getPriority() const;
    double getDeadline() const;
    const std::set<std::string> & getTasks() const;
    bool hasTask(const std::string& task) const;
};

#endif