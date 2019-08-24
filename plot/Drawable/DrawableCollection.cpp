#include <DrawableCollection.h>

void drawable::DrawableCollection::adjustBoundingBox(drawable::BoundingBox new_bounding_box){
    m_bounding_box.lower_left_x = std::min(m_bounding_box.lower_left_x, new_bounding_box.lower_left_x); 
    m_bounding_box.lower_left_y = std::min(m_bounding_box.lower_left_y, new_bounding_box.lower_left_y); 
    m_bounding_box.top_right_x = std::max(m_bounding_box.top_right_x, new_bounding_box.top_right_x); 
    m_bounding_box.top_right_y = std::max(m_bounding_box.top_right_y, new_bounding_box.top_right_y); 
}

void drawable::DrawableCollection::draw(const Renderer& renderer) const {
    for(const auto& drawable: m_collection){
        drawable->draw(renderer);
    }
}
bool drawable::DrawableCollection::addDrawable(const Drawable* drawable) {
    auto new_drawable = drawable->clone();
    new_drawable->setColor(m_color);
    adjustBoundingBox(new_drawable->getBoundingBox());
    m_collection.insert(std::move(new_drawable));
    return true;
}
bool drawable::DrawableCollection::removeDrawable(const Drawable*) {
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
    for(const auto& drawable: m_collection){
        new_collection->addDrawable(&*drawable);
    }
    return new_collection;
}


drawable::BoundingBox drawable::DrawableCollection::getBoundingBox() const{
    return m_bounding_box;
}