#ifndef AREA_H__
#define AREA_H__

#include <string>
#include <Position.h>
#include <memory>

class Area
{
public:
    virtual std::string toStringRepresentation() const = 0;
    virtual bool containsPoint(const Position2d & pos) const = 0;
    ~Area(){};
};

using PtrArea = std::unique_ptr<Area>;

#endif //AREA_H__