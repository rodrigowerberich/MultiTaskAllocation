#ifndef OBSTRUCTEDAREAFACTORY_H__
#define OBSTRUCTEDAREAFACTORY_H__

#include <ObstructedArea.h>
#include <JSON.h>
#include <memory>

class ObstructedAreaFactory{
public:
    static PtrObstructedArea make(const std::string& json);
    static PtrObstructedArea make(JsonObject& json);
};

#endif //OBSTRUCTEDAREAFACTORY_H__