#include <GnuPlotRenderer.h>
#include <gnuplot-iostream.h>
#include <cmath>
#include <array>
#include <tuple>
#include <range.h>

#define CREATE_TEMPLATE_DRAW(drawable_name_)\
namespace renderer{\
    template <>\
    void draw(const drawable::drawable_name_& drawable, const GnuPlotRenderer& renderer){\
        renderer.draw##drawable_name_(drawable);\
    }\
}

Gnuplot GnuPlotRenderer::m_gp;

std::string GnuPlotRenderer::gnuplotColor(drawable::Color color) const{
    std::string color_command = "rgb ";
    switch (color){
    case drawable::Color::Black:
        return color_command+"'#000000'";
    case drawable::Color::Red:
        return color_command+"'#FF0000'";
    case drawable::Color::Blue:
        return color_command+"'#0000FF'";
    case drawable::Color::Green:
        return color_command+"'#00FF00'";
    case drawable::Color::DarkGreen:
        return color_command+"'#006400'";
    case drawable::Color::Orange:
        return color_command+"'#FFA500'";
    case drawable::Color::Yellow:
        return color_command+"'#FFFF00'";
    case drawable::Color::Cyan:
        return color_command+"'#00FFFF'";
    case drawable::Color::DarkBlue:
        return color_command+"'#00008B'";
    case drawable::Color::Purple:
        return color_command+"'#800080'";
    case drawable::Color::Magenta:
        return color_command+"'#FF00FF'";
    case drawable::Color::Violet:
        return color_command+"'#EE82EE'";
    case drawable::Color::Pink:
        return color_command+"'#FFC0CB'";
    case drawable::Color::DeepPink:
        return color_command+"'#FF1493'";
    case drawable::Color::Gray:
        return color_command+"'#808080'";
    case drawable::Color::Brown:
        return color_command+"'#A52A2A'";
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
CREATE_TEMPLATE_DRAW(Rectangle)

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
CREATE_TEMPLATE_DRAW(Circle)

void GnuPlotRenderer::drawLine(const drawable::Line& line) const{
    auto x1 = line.getX1();
    auto y1 = line.getY1();
    auto x2 = line.getX2();
    auto y2 = line.getY2();
    std::array<std::pair<double, double>,2> xy_pts = {
        {{x1, y1}, {x2, y2}}
    };
    m_gp << "plot" << m_gp.file1d(xy_pts) << "with lines notitle lt "<< gnuplotColor(line.getColor()) << std::endl;
}
CREATE_TEMPLATE_DRAW(Line)

void GnuPlotRenderer::drawLines(const drawable::Lines& lines) const{
    auto p1s = lines.getP1s();
    auto p2s = lines.getP2s();
    std::vector<std::vector<std::pair<double,double>>> all_segments;
    for(const auto& i:util::lang::indices(p1s)){
        std::vector<std::pair<double, double>> segment;
        segment.push_back(std::make_pair(p1s[i].first, p1s[i].second));
        segment.push_back(std::make_pair(p2s[i].first, p2s[i].second));
        all_segments.push_back(segment);
    }
    m_gp << "plot" << m_gp.file2d(all_segments) << "with lines notitle lt "<< gnuplotColor(lines.getColor()) << std::endl;
}
CREATE_TEMPLATE_DRAW(Lines)

void GnuPlotRenderer::drawPoint(const drawable::Point& point) const{
    auto x = point.getX();
    auto y = point.getY();
    m_gp << "plot" << m_gp.file1d(std::array<std::pair<double, double>,1>{std::make_pair(x, y)}) << "with points notitle lt " << gnuplotColor(point.getColor()) << std::endl;
}
CREATE_TEMPLATE_DRAW(Point)

void GnuPlotRenderer::drawPoints(const drawable::Points& points) const{
    m_gp << "plot" << m_gp.file1d(points.getPoints()) << "with points notitle lt " << gnuplotColor(points.getColor()) << std::endl;
}
CREATE_TEMPLATE_DRAW(Points)

void GnuPlotRenderer::drawNamedPoint(const drawable::NamedPoint& named_point) const{
    auto x = named_point.getX();
    auto y = named_point.getY();
    auto name = named_point.getName();
    auto color = gnuplotColor(named_point.getColor());
    m_gp << "plot" << m_gp.file1d(std::array<std::tuple<double, double, std::string>,1>{std::make_tuple(x, y, name)}) << "with labels notitle point lc " << color << " offset 0,0.5 textcolor " << color << std::endl;
}
CREATE_TEMPLATE_DRAW(NamedPoint)