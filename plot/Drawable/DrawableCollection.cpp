#include <DrawableCollection.h>

void drawable::DrawableCollection::draw(const Renderer& renderer) const {
    for(const auto& drawable: m_collection){
        drawable->draw(renderer);
    }
}
bool drawable::DrawableCollection::addDrawable(const Drawable* drawable) {
    auto new_drawable = drawable->clone();
    new_drawable->setColor(m_color);
    m_collection.insert(std::move(new_drawable));
    return true;
}
bool drawable::DrawableCollection::removeDrawable(const Drawable* drawable) {
    return false;
}

void drawable::DrawableCollection::setColor(drawable::Color color){
    m_color = color;
    for(const auto& drawable: m_collection){
        drawable->setColor(m_color);
    }
}

std::unique_ptr<drawable::Drawable> drawable::DrawableCollection::clone()  const{
    auto new_collection = std::make_unique<DrawableCollection>(m_color);
}

