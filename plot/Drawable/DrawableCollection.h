#ifndef DRAWABLECOLLECTION_H__
#define DRAWABLECOLLECTION_H__

#include <Drawable.h>
#include<set>

namespace drawable{
class DrawableCollection: public drawable::Drawable{
private:
    std::set<std::unique_ptr<Drawable>> m_collection;
    drawable::Color m_color;
public:
    DrawableCollection(drawable::Color color = drawable::Color::Green):m_color{color}{}
    virtual void draw(const Renderer& renderer) const override;
    virtual bool addDrawable(const Drawable* drawable) override;
    virtual bool removeDrawable(const Drawable* drawable) override;
    virtual void setColor(drawable::Color color);
    virtual std::unique_ptr<Drawable> clone()  const;
};
}

#endif //DRAWABLECOLLECTION_H__