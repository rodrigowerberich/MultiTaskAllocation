#ifndef MISSIONS_H__
#define MISSIONS_H__

#include <Mission.h>
#include <MappedVectorOfPointers.h>

class Missions: public MappedVectorOfPointers<Mission>{
public:
    const std::unique_ptr<Mission> & getMission(const std::string& task_name);    
};

#endif // MISSIONS_H__