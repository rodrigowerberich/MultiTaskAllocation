#ifndef PYPLOTPOINT_H__
#define PYPLOTPOINT_H__

#include <PyPlotDrawable.h>

namespace PyPlot{

class Point: public PyPlot::Drawable{
protected:
std::vector<double> m_x_points;
std::vector<double> m_y_points;
public:
    Point(double x, double y, std::string parameters="bo"):PyPlot::Drawable(parameters), m_x_points(1),m_y_points(1){
        m_x_points[0] = x;
        m_y_points[0] = y;
    }
    void draw() override{
        m_plot.update(m_x_points, m_y_points);
    }
    bool addDrawable(Basic::Drawable* drawable) override{
        return false;
    }
    bool removeDrawable(Basic::Drawable* drawable) override{
        return false;
    }
    void update(double x, double y){
        m_x_points[0] = x;
        m_y_points[0] = y;
    }
};

}

#endif //PYPLOTPOINT_H__