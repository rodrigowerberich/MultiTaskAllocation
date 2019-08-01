#ifndef DRAWABLE_H__
#define DRAWABLE_H__

#include <Renderer.h>
#include <Color.h>
#include <memory>

namespace drawable{
class Drawable{
public:
    virtual void draw(const Renderer& renderer) const = 0;
    virtual bool addDrawable(const Drawable* drawable) = 0;
    virtual bool removeDrawable(const Drawable* drawable) = 0;
    virtual void setColor(drawable::Color color) = 0;
    virtual std::unique_ptr<Drawable> clone() const = 0;
    virtual ~Drawable(){}
};
}

#endif //DRAWABLE_H__