#ifndef GEOMETRICOBJECT_H__
#define GEOMETRICOBJECT_H__

#include <string>
#include <Position.h>
#include <memory>
#include <Drawable.h>

class GeometricObject{
public:
    virtual bool containsPoint(const Position2d & position) const = 0;
    virtual std::string toStringRepresentation() const = 0;
    virtual std::unique_ptr<GeometricObject> clone() const = 0;
    virtual const drawable::Drawable* getDrawable() const = 0;
    ~GeometricObject(){}
};
using PtrGeometricObject = std::unique_ptr<GeometricObject>;

#endif // GEOMETRICOBJECT_H__