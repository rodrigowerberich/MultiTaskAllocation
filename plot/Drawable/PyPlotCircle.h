#ifndef PYPLOTCIRCLE_H__
#define PYPLOTCIRCLE_H__

#include <PyPlotDrawable.h>

namespace PyPlot{

class Circle: public PyPlot::Drawable{
protected:
    std::vector<double> m_x_points;
    std::vector<double> m_y_points;
    double m_center_x;
    double m_center_y;
    double m_radius;
    int m_resolution;
    void calculatePoints();
public:
    Circle(double x, double y, double radius, std::string parameters="r-", int resolution=100);
    void draw() override;
    bool addDrawable(Basic::Drawable* drawable) override;
    bool removeDrawable(Basic::Drawable* drawable) override;
};

}

#endif //PYPLOTCIRCLE_H__