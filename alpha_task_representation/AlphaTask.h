#ifndef ALPHATASK_H__
#define ALPHATASK_H__

#include <Position.h>
#include <vector>
#include <string>

class AlphaTask{
private:
    using Container = std::vector<Position2d>;
    std::string m_task;
    Container m_tasks_positions;
public:
    AlphaTask(){}
    AlphaTask(std::string task, const Container& tasks_positions):m_task{task},m_tasks_positions{tasks_positions}{}
    template <typename T>
    AlphaTask(std::string task, const T& task_positions){
        m_task = task;
        m_tasks_positions.insert(std::begin(task_positions), std::end(m_tasks_positions));
    }
    ~AlphaTask(){}
    size_t size() const {return m_tasks_positions.size();}
    Container::iterator begin(){
        return m_tasks_positions.begin();
    }
    Container::const_iterator begin() const{
        return m_tasks_positions.begin();
    }
    Container::iterator end(){
        return m_tasks_positions.end();
    }
    Container::const_iterator end() const{
        return m_tasks_positions.end();
    }
    Container::reference operator[](size_t pos){
        return m_tasks_positions[pos];
    }
    Container::const_reference operator[](size_t pos) const{
        return m_tasks_positions[pos];
    }
    const std::string & getTask() const{
        return m_task;
    }
};

#endif //ALPHATASK_H__