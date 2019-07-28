#ifndef PYPLOTDRAWABLE_H__
#define PYPLOTDRAWABLE_H__

#include <Drawable.h>
#include <string>
#include "matplotlibcpp.h"

namespace PyPlot{

class Drawable: public Basic::Drawable{
protected:
    matplotlibcpp::Plot m_plot;
public:
    Drawable(std::string parameters, std::string name=""):m_plot{name, parameters}{}

    virtual void clear() override{
        m_plot.clear();
    }
    virtual void erase() override{
        m_plot.clear();
    }
    ~Drawable(){
        m_plot.remove();
    }
};

}

#endif //PYPLOTDRAWABLE_H__