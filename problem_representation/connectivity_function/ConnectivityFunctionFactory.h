#ifndef CONNECTIVITYFUNCTIONFACTORY_H__
#define CONNECTIVITYFUNCTIONFACTORY_H__

#include <ConnectivityFunction.h>
#include <JSON.h>
#include <memory>

class ConnectivityFunctionFactory{
public:
    static PtrConnectivityFunction make(const std::string& json);
    static PtrConnectivityFunction make(JsonObject& json);
};

#endif //CONNECTIVITYFUNCTIONFACTORY_H__