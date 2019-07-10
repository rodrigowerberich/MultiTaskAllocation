#include <MissionImpl.h>

MissionImpl::MissionImpl(const std::string & name, int priority, double deadline, const std::set<std::string> & tasks_names):
    m_name {name},
    m_priority {priority},
    m_deadline {deadline},
    m_tasks_names {tasks_names}
 {}
const std::string & MissionImpl::getName() const{
    return m_name;
}
int MissionImpl::getPriority() const{
    return m_priority;
}
double MissionImpl::getDeadline() const{
    return m_deadline;
}

const std::set<std::string> & MissionImpl::getTasks() const{
    return m_tasks_names;
}

bool MissionImpl::hasTask(const std::string& task) const{
    return m_tasks_names.count(task) > 0;
}
