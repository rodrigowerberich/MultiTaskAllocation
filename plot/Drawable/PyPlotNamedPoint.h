#ifndef PYPLOTNAMEDPOINT_H__
#define PYPLOTNAMEDPOINT_H__

#include <PyPlotPoint.h>

namespace PyPlot{

class NamedPoint: public PyPlot::Point{
std::string m_name;
public:
    NamedPoint(double x, double y, std::string name, std::string parameters="bo"):PyPlot::Point(x,y,parameters), m_name{name}{
    }
    void draw() override{
        matplotlibcpp::text(m_x_points[0], m_y_points[0], m_name);
        PyPlot::Point::draw();
    }
};

}


#endif //PYPLOTNAMEDPOINT_H__