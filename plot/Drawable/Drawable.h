#ifndef DRAWABLE_H__
#define DRAWABLE_H__

#include <memory>

namespace Basic{

class Drawable{
public:
    virtual ~Drawable(){};
    virtual void draw() = 0;
    virtual void clear() = 0;
    virtual void erase() = 0;
    virtual bool addDrawable(Drawable* drawable) = 0;
    virtual bool removeDrawable(Drawable* drawable) = 0;
};

using PtrDrawable = std::unique_ptr<Drawable>;

}

#endif //DRAWABLE_H__