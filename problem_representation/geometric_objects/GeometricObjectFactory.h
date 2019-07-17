#ifndef GEOMETRICOBJECTFACTORY_H__
#define GEOMETRICOBJECTFACTORY_H__

#include <GeometricObject.h>
#include <JSON.h>
#include <memory>

class GeometricObjectFactory{
public:
    static std::unique_ptr<GeometricObject> make(const std::string& json);
    static std::unique_ptr<GeometricObject> make(JsonObject& json);
};

#endif //GEOMETRICOBJECTFACTORY_H__