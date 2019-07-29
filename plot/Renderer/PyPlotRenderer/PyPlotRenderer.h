#ifndef PYPLOTRENDERER_H__
#define PYPLOTRENDERER_H__

#include <Rectangle.h>

class PyPlotRenderer{
private:
public: 
    void drawRectangle(const Drawable::Rectangle& rect);
    // void drawCircle(const Drawable::Circle& rect);
    // void drawLine(const Drawable::Line& rect);
    // void drawPoint(const Drawable::Point& rect);
    // void drawNamedPoint(const Drawable::NamedPoint& rect);
};


#endif //PYPLOTRENDERER_H__