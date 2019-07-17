#ifndef SEARCHAREA_H__
#define SEARCHAREA_H__

#include <string>
#include <Position.h>
#include <GeometricObject.h>

class SearchArea{
private:
    std::unique_ptr<GeometricObject>  m_geometric_representation;
public:
    SearchArea(std::unique_ptr<GeometricObject> && geometric_representation);
    bool containsPoint(const Position2d & position) const;
    std::string toStringRepresentation() const;
    ~SearchArea(){}
};
#endif // SEARCHAREA_H__