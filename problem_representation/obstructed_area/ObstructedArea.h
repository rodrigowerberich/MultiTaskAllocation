#ifndef OBSTRUCTEDAREA_H__
#define OBSTRUCTEDAREA_H__

#include <string>
#include <Position.h>
#include <memory>
#include <Area.h>

class ObstructedArea{
private:
    PtrArea m_area;
public:
    ObstructedArea(PtrArea && area):m_area{std::move(area)}{}
    std::string toStringRepresentation() const{ return m_area->toStringRepresentation(); };
    bool containsPoint(const Position2d & pos) const{ return m_area->containsPoint(pos); };
    ~ObstructedArea(){};
};

using PtrObstructedArea = std::unique_ptr<ObstructedArea>;

#endif //OBSTRUCTEDAREA_H__