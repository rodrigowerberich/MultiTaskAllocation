#ifndef PYPLOTRENDERER_H__
#define PYPLOTRENDERER_H__

#include <Rectangle.h>

class PyPlotRenderer{
private:
public: 
    void drawRectangle(const drawable::Rectangle& rect);
    // void drawCircle(const drawable::Circle& rect);
    // void drawLine(const drawable::Line& rect);
    // void drawPoint(const drawable::Point& rect);
    // void drawNamedPoint(const drawable::NamedPoint& rect);
};


#endif //PYPLOTRENDERER_H__