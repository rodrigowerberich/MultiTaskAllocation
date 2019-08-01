#ifndef DRAWABLE_H__
#define DRAWABLE_H__

#include <Renderer.h>

namespace drawable{
class Drawable{
public:
    virtual void draw(const Renderer& renderer) const = 0;
    virtual bool addDrawable(const Drawable* drawable) = 0;
    virtual bool removeDrawable(const Drawable* drawable) = 0;
    virtual ~Drawable(){}
};
}

#endif //DRAWABLE_H__