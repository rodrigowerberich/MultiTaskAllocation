#ifndef OBSTRUCTEDAREA_H__
#define OBSTRUCTEDAREA_H__

#include <string>
#include <Position.h>
#include <memory>

class ObstructedArea
{
public:
    virtual std::string toStringRepresentation() const = 0;
    virtual bool containsPoint(const Position2d & pos) const = 0;
    ~ObstructedArea(){};
};

using PtrObstructedArea = std::unique_ptr<ObstructedArea>;

#endif //OBSTRUCTEDAREA_H__