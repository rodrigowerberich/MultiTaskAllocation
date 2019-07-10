#ifndef GEOMETRICOBJECT_H__
#define GEOMETRICOBJECT_H__

#include <string>

class GeometricObject{
public:
    virtual bool containsPoint(const Position2d & position) const = 0;
    virtual const std::string & toStringRepresentation() const = 0;
    ~GeometricObject(){}
};
#endif // GEOMETRICOBJECT_H__