#ifndef DRAWABLE_H__
#define DRAWABLE_H__

#include <Renderer.h>
#include <Color.h>
#include <memory>
#include <array>

namespace drawable{
struct BoundingBox{
    double lower_left_x;    
    double lower_left_y;
    double top_right_x;
    double top_right_y;
    BoundingBox(double lower_left_x, double lower_left_y, double top_right_x, double top_right_y):lower_left_x{lower_left_x},lower_left_y{lower_left_y},top_right_x{top_right_x},top_right_y{top_right_y}{}
    template <typename T1, typename T2>
    BoundingBox(const T1& lower_left, const T1& top_right):BoundingBox{lower_left[0], lower_left[1], top_right[0], top_right[1]}{}
};

class Drawable{
public:
    virtual void draw(const Renderer& renderer) const = 0;
    virtual bool addDrawable(const Drawable* drawable) = 0;
    virtual bool removeDrawable(const Drawable* drawable) = 0;
    virtual void setColor(drawable::Color color) = 0;
    virtual BoundingBox getBoundingBox() const = 0;
    virtual std::unique_ptr<Drawable> clone() const = 0;
    virtual ~Drawable(){}
};
}

#endif //DRAWABLE_H__