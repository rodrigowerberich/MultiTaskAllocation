#ifndef CONNECTIVITYFUNCTION_H__
#define CONNECTIVITYFUNCTION_H__

#include <Area.h>

class ConnectivityFunction{
private:
    PtrArea m_area;
public:
    ConnectivityFunction(PtrArea && area);
    std::string toStringRepresentation() const{ return m_area->toStringRepresentation(); };
    bool containsPoint(const Position2d & pos) const{ return m_area->containsPoint(pos); };
    float operator()(const Position2d & pos) const{ return containsPoint(pos); }
    float operator()(float x, float y) const{ return containsPoint({x,y}); }
    const drawable::Drawable* getDrawable() const;
    ~ConnectivityFunction(){}
};

using PtrConnectivityFunction = std::unique_ptr<ConnectivityFunction>;

#endif //CONNECTIVITYFUNCTION_H__