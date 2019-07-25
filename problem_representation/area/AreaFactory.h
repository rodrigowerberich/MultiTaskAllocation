#ifndef AREAFACTORY_H__
#define AREAFACTORY_H__

#include <Area.h>
#include <JSON.h>
#include <memory>

class AreaFactory{
public:
    static PtrArea make(const std::string& json);
    static PtrArea make(JsonObject& json);
};


#endif //AREAFACTORY_H__