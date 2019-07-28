#ifndef PYPLOTRECTANGLE_H__
#define PYPLOTRECTANGLE_H__

#include <PyPlotDrawable.h>

namespace PyPlot{

class Rectangle: public PyPlot::Drawable{
std::vector<double> m_x_points;
std::vector<double> m_y_points;
public:
    Rectangle(double x, double y, int width, int height, std::string parameters="r-"):PyPlot::Drawable(parameters), m_x_points(5),m_y_points(5){
        m_x_points[0] = x;
        m_x_points[1] = x+width;
        m_x_points[2] = x+width;
        m_x_points[3] = x;
        m_x_points[4] = x;
        m_y_points[0] = y;
        m_y_points[1] = y;
        m_y_points[2] = y+height;
        m_y_points[3] = y+height;
        m_y_points[4] = y;
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
};

}

#endif //PYPLOTRECTANGLE_H__