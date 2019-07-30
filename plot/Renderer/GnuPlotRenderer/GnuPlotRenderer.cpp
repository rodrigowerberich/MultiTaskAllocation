#include <GnuPlotRenderer.h>
#include <gnuplot-iostream.h>

Gnuplot GnuPlotRenderer::m_gp;

std::string GnuPlotRenderer::gnuplotColor(Drawable::Color color){
    std::string color_command = "rgb ";
    switch (color){
    case Drawable::Color::Blue:
        return color_command+"'#0000FF'";
    case Drawable::Color::Red:
        return color_command+"'#FF0000'";
    case Drawable::Color::Green:
        return color_command+"'#00FF00'";
    }
    return color_command+"#000000";
}


void GnuPlotRenderer::holdOn(bool hold){
    m_gp << (hold?"set":"unset") << " multiplot\n";
}


void GnuPlotRenderer::drawRectangle(const Drawable::Rectangle& rect){
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
    m_gp << "plot" << m_gp.file1d(xy_pts) << "with lines notitle lt "<< gnuplotColor(Drawable::Color::Red) << std::endl;
}
