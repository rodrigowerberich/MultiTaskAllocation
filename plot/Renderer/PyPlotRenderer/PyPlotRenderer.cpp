#include <PyPlotRenderer.h>
#include <Polygon.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

static std::string color_to_matplotlib_string(Drawable::Color color){
    switch (color){
    case Drawable::Color::Green :
        return "g";
    case Drawable::Color::Blue :
        return "b";
    case Drawable::Color::Red :
        return "r";    
    }
    return "g";
}

template <int N, typename Numeric>
void drawPolygon(const Drawable::Polygon<N,Numeric>& polygon, std::string stroke_style){
    std::cout << polygon.getXPoints().size() << ", " << polygon.getYPoints().size() <<  ", " << color_to_matplotlib_string(polygon.getColor())+stroke_style;
    plt::plot(polygon.getXPoints(), polygon.getYPoints(), color_to_matplotlib_string(polygon.getColor())+stroke_style);
}

void PyPlotRenderer::drawRectangle(const Drawable::Rectangle& rect){
    std::cout << "Aqui drawRectangle\n";
    plt::plot({2.0},{3.0},"r*");
    // plt::plot(polygon.getXPoints(), polygon.getYPoints(), color_to_matplotlib_string(polygon.getColor())+stroke_style);
    // drawPolygon(Drawable::Polygon<5, double>(
    //         {rect.getBottomLeftX(), rect.getBottomLeftX()+rect.getWidth(), rect.getBottomLeftX()+rect.getWidth(), rect.getBottomLeftX(), rect.getBottomLeftX()}, 
    //         {rect.getBottomLeftY(), rect.getBottomLeftY(), rect.getBottomLeftY()+rect.getHeight(), rect.getBottomLeftY()+rect.getHeight(), rect.getBottomLeftY()},
    //         rect.getColor()    
    //     ),
    //     "-"
    // );
}
