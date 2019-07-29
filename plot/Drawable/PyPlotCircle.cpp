#include <PyPlotCircle.h>
#include <cmath>

#define _USE_MATH_DEFINES

void PyPlot::Circle::calculatePoints(){
    auto theta = std::vector<double>(m_resolution+1);
    for (size_t i = 0; i < theta.size(); i++){
        theta[i] = ((double)i/(double)m_resolution)*2*M_PI;
        m_x_points[i] = m_center_x + m_radius*std::cos(theta[i]);
        m_y_points[i] = m_center_y + m_radius*std::sin(theta[i]);
    }
}


PyPlot::Circle::Circle(double x, double y, double radius, std::string parameters, int resolution):
    PyPlot::Drawable(parameters), 
    m_x_points(resolution+1),
    m_y_points(resolution+1), 
    m_center_x{x}, 
    m_center_y{y}, 
    m_radius{radius},
    m_resolution{resolution}{
    calculatePoints();    
}

void PyPlot::Circle::draw() {
    m_plot.update(m_x_points, m_y_points);
}
bool PyPlot::Circle::addDrawable(Basic::Drawable* drawable){
    return false;
}
bool PyPlot::Circle::removeDrawable(Basic::Drawable* drawable){
    return false;
}
