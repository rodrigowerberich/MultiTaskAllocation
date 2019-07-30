#ifndef GNUPLOTRENDERER_H__
#define GNUPLOTRENDERER_H__

#include <Rectangle.h>
#include <gnuplot-iostream.h>

class GnuPlotRenderer{
    static Gnuplot m_gp;
    std::string gnuplotColor(Drawable::Color color);
public:
    void holdOn(bool hold=true);
    template <typename Numeric>
    void setXRange(Numeric x_min, Numeric x_max){
        if (x_min < x_max){
            m_gp << "set xrange ["<< x_min <<  ":" << x_max << "]\n";
        }
    }
    template <typename Numeric>
    void setYRange(Numeric y_min, Numeric y_max){
        if (y_min < y_max){
            m_gp << "set yrange ["<< y_min <<  ":" << y_max << "]\n";
        }
    }
    template <typename Numeric>
    void setAxixRange(Numeric x_min, Numeric x_max, Numeric y_min, Numeric y_max){
        setXRange(x_min, x_max);
        setYRange(y_min, y_max);
    }
    void drawRectangle(const Drawable::Rectangle& rect);
    // void drawCircle(const Drawable::Circle& rect);
    // void drawLine(const Drawable::Line& rect);
    // void drawPoint(const Drawable::Point& rect);
    // void drawNamedPoint(const Drawable::NamedPoint& rect);
};

#endif //GNUPLOTRENDERER_H__