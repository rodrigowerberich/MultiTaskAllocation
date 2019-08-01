#include <PyPlotRenderer.h>
#include <Polygon.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

static std::string color_to_matplotlib_string(drawable::Color color){
    switch (color){
    case drawable::Color::Green :
        return "g";
    case drawable::Color::Blue :
        return "b";
    case drawable::Color::Red :
        return "r";    
    }
    return "g";
}

template <int N, typename Numeric>
void drawPolygon(const drawable::Polygon<N,Numeric>& polygon, std::string stroke_style){
    std::cout << polygon.getXPoints().size() << ", " << polygon.getYPoints().size() <<  ", " << color_to_matplotlib_string(polygon.getColor())+stroke_style;
    plt::plot(polygon.getXPoints(), polygon.getYPoints(), color_to_matplotlib_string(polygon.getColor())+stroke_style);
}

void PyPlotRenderer::drawRectangle(const drawable::Rectangle& rect){
    std::cout << "Aqui drawRectangle\n";
    plt::plot({2.0},{3.0},"r*");
    // plt::plot(polygon.getXPoints(), polygon.getYPoints(), color_to_matplotlib_string(polygon.getColor())+stroke_style);
    // drawPolygon(drawable::Polygon<5, double>(
    //         {rect.getBottomLeftX(), rect.getBottomLeftX()+rect.getWidth(), rect.getBottomLeftX()+rect.getWidth(), rect.getBottomLeftX(), rect.getBottomLeftX()}, 
    //         {rect.getBottomLeftY(), rect.getBottomLeftY(), rect.getBottomLeftY()+rect.getHeight(), rect.getBottomLeftY()+rect.getHeight(), rect.getBottomLeftY()},
    //         rect.getColor()    
    //     ),
    //     "-"
    // );
}
