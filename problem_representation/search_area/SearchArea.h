#ifndef SEARCHAREA_H__
#define SEARCHAREA_H__

#include <string>
#include <Position.h>

class SearchArea{
public:
    bool containsPoint(const Position2d & position) const = 0;
    const std::string & toStringRepresentation() const = 0;
    ~SearchArea(){}
};
#endif // SEARCHAREA_H__