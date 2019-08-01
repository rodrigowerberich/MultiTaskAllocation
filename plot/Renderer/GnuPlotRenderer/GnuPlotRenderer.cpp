#include <GnuPlotRenderer.h>
#include <gnuplot-iostream.h>
#include <cmath>
#include <array>
#include <tuple>

Gnuplot GnuPlotRenderer::m_gp;

std::string GnuPlotRenderer::gnuplotColor(drawable::Color color) const{
    std::string color_command = "rgb ";
    switch (color){
    case drawable::Color::Blue:
        return color_command+"'#0000FF'";
    case drawable::Color::Red:
        return color_command+"'#FF0000'";
    case drawable::Color::Green:
        return color_command+"'#00FF00'";
    }
    return color_command+"#000000";
}


void GnuPlotRenderer::holdOn(bool hold) const{
    m_gp << (hold?"set":"unset") << " multiplot\n";
}


void GnuPlotRenderer::drawRectangle(const drawable::Rectangle& rect) const{
    auto x = rect.getBottomLeftX();
    auto y = rect.getBottomLeftY();
    auto width = rect.getWidth();
    auto height = rect.getHeight();
    std::vector<std::pair<double, double>> xy_pts = {
        {x,         y},
        {x+width,   y},
        {x+width,   y+height},
        {x,         y+height},
        {x,         y}
    };
    m_gp << "plot" << m_gp.file1d(xy_pts) << "with lines notitle lt "<< gnuplotColor(rect.getColor()) << std::endl;
}

namespace renderer{
    template <>
    void draw<drawable::Rectangle, GnuPlotRenderer>(const drawable::Rectangle& drawable, const GnuPlotRenderer& renderer){
        renderer.drawRectangle(drawable);
    }
}

void GnuPlotRenderer::drawCircle(const drawable::Circle& circle) const{
    auto x = circle.getCenterX();
    auto y = circle.getCenterY();
    auto radius = circle.getRadius();
    constexpr int resolution = 100;
    std::array<std::pair<double, double>, resolution+1> points;
    for(int i=0; i <=resolution; i++){
        double theta = ((double)i)/((double)resolution)*2*M_PI;
        points[i] = std::make_pair(x+radius*std::sin(theta), y+radius*std::cos(theta));
    }
    m_gp << "plot" << m_gp.file1d(points) << "with lines notitle lt "<< gnuplotColor(circle.getColor()) << std::endl;
}

namespace renderer{
    template <>
    void draw<drawable::Circle, GnuPlotRenderer>(const drawable::Circle& drawable, const GnuPlotRenderer& renderer){
        renderer.drawCircle(drawable);
    }
}


void GnuPlotRenderer::drawPoint(const drawable::Point& point) const{
    auto x = point.getX();
    auto y = point.getY();
    m_gp << "plot" << m_gp.file1d(std::array<std::pair<double, double>,1>{std::make_pair(x, y)}) << "with points notitle lt " << gnuplotColor(point.getColor()) << std::endl;
}

namespace renderer{
    template <>
    void draw<drawable::Point, GnuPlotRenderer>(const drawable::Point& drawable, const GnuPlotRenderer& renderer){
        renderer.drawPoint(drawable);
    }
}

#define CREATE_TEMPLATE_DRAW(drawable_name_)\
namespace renderer{\
    template <>\
    void draw(const drawable::drawable_name_& drawable, const GnuPlotRenderer& renderer){\
        renderer.draw##drawable_name_(drawable);\
    }\
}

void GnuPlotRenderer::drawNamedPoint(const drawable::NamedPoint& named_point) const{
    auto x = named_point.getX();
    auto y = named_point.getY();
    auto name = named_point.getName();
    auto color = gnuplotColor(named_point.getColor());
    m_gp << "plot" << m_gp.file1d(std::array<std::tuple<double, double, std::string>,1>{std::make_tuple(x, y, name)}) << "with labels notitle point lc " << color << " offset 0,0.5 textcolor " << color << std::endl;
}
CREATE_TEMPLATE_DRAW(NamedPoint)