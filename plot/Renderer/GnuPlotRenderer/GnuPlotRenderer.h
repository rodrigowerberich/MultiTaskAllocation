#ifndef GNUPLOTRENDERER_H__
#define GNUPLOTRENDERER_H__

#include <Renderer.h>
#include <Rectangle.h>
#include <Circle.h>
#include <Point.h>
#include <NamedPoint.h>
#include <gnuplot-iostream.h>

class GnuPlotRenderer: public drawable::Renderer{
    static Gnuplot m_gp;
    std::string gnuplotColor(drawable::Color color) const;
public:
    void holdOn(bool hold=true) const;
    template <typename Numeric>
    void setXRange(Numeric x_min, Numeric x_max) const{
        if (x_min < x_max){
            m_gp << "set xrange ["<< x_min <<  ":" << x_max << "]\n";
        }
    }
    template <typename Numeric>
    void setYRange(Numeric y_min, Numeric y_max) const{
        if (y_min < y_max){
            m_gp << "set yrange ["<< y_min <<  ":" << y_max << "]\n";
        }
    }
    template <typename Numeric>
    void setAxixRange(Numeric x_min, Numeric x_max, Numeric y_min, Numeric y_max) const{
        setXRange(x_min, x_max);
        setYRange(y_min, y_max);
    }
    template <typename T>
    void draw(const T& drawable) const{
        renderer::draw(drawable, *this);
    }
    void drawRectangle(const drawable::Rectangle& rect) const;
    void drawCircle(const drawable::Circle& circle) const;
    // void drawLine(const drawable::Line& rect);
    void drawPoint(const drawable::Point& point) const;
    void drawNamedPoint(const drawable::NamedPoint& named_point) const;
};

#endif //GNUPLOTRENDERER_H__