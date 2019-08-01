#ifndef GEOMETRICAREA_H__
#define GEOMETRICAREA_H__

#include <Area.h>
#include <vector>
#include <GeometricObject.h>
#include <DrawableCollection.h>

class GeometricArea: public Area{
private:
    std::vector<PtrGeometricObject> m_areas;
    drawable::DrawableCollection m_collection;
public:
    GeometricArea(const std::vector<PtrGeometricObject> & areas);
    GeometricArea(std::vector<PtrGeometricObject> && areas);
    std::string toStringRepresentation() const override;
    bool containsPoint(const Position2d & pos) const override;
    virtual const drawable::Drawable* getDrawable() const;
};

#endif //GEOMETRICAREA_H__