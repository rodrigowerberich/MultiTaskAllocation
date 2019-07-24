#ifndef OBSTRUCTEDAREACOLLECTION_H__
#define OBSTRUCTEDAREACOLLECTION_H__

#include <ObstructedArea.h>
#include <vector>
#include <GeometricObject.h>

class ObstructedAreaCollection: public ObstructedArea{
private:
    std::vector<PtrGeometricObject> m_obstructed_areas;
public:
    ObstructedAreaCollection(const std::vector<PtrGeometricObject> & obstructed_areas);
    ObstructedAreaCollection(std::vector<PtrGeometricObject> && obstructed_areas);
    std::string toStringRepresentation() const override;
    bool containsPoint(const Position2d & pos) const override;
};

#endif //OBSTRUCTEDAREACOLLECTION_H__