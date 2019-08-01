#include <DrawableCollection.h>

void drawable::DrawableCollection::draw(const Renderer& renderer) const {
    for(const auto& drawable: m_collection){
        drawable->draw(renderer);
    }
}
bool drawable::DrawableCollection::addDrawable(const Drawable* drawable) {
    if(m_collection.count(drawable) > 0){
        return false;
    }
    m_collection.insert(drawable);
    return true;
}
bool drawable::DrawableCollection::removeDrawable(const Drawable* drawable) {
    if(m_collection.count(drawable) == 0){
        return false;
    }
    m_collection.erase(drawable);
    return true;
}