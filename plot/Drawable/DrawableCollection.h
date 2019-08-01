#ifndef DRAWABLECOLLECTION_H__
#define DRAWABLECOLLECTION_H__

#include <Drawable.h>
#include<set>

namespace drawable{
class DrawableCollection: public drawable::Drawable{
private:
    std::set<const Drawable*> m_collection;
public:
    virtual void draw(const Renderer& renderer) const override;
    virtual bool addDrawable(const Drawable* drawable) override;
    virtual bool removeDrawable(const Drawable* drawable) override;
};
}

#endif //DRAWABLECOLLECTION_H__