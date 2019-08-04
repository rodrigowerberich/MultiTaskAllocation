#include <Lines.h>

void drawable::Lines::calculateBoundingBox(){
    for(const auto & point:m_p1s){
        if(point.first < m_bounding_box.lower_left_x){
            m_bounding_box.lower_left_x = point.first;
        }
        if(point.first > m_bounding_box.top_right_x){
            m_bounding_box.top_right_x = point.first;
        }
        if(point.second < m_bounding_box.lower_left_y){
            m_bounding_box.lower_left_y = point.second;
        }
        if(point.second > m_bounding_box.top_right_y){
            m_bounding_box.top_right_y = point.second;
        }
    }
    for(const auto & point:m_p2s){
        if(point.first < m_bounding_box.lower_left_x){
            m_bounding_box.lower_left_x = point.first;
        }
        if(point.first > m_bounding_box.top_right_x){
            m_bounding_box.top_right_x = point.first;
        }
        if(point.second < m_bounding_box.lower_left_y){
            m_bounding_box.lower_left_y = point.second;
        }
        if(point.second > m_bounding_box.top_right_y){
            m_bounding_box.top_right_y = point.second;
        }
    }
}


drawable::Lines::Lines(std::vector<std::pair<double, double>> p1s, std::vector<std::pair<double, double>> p2s, drawable::Color color):m_p1s{p1s}, m_p2s{p2s},m_color{color},m_bounding_box{0,0,0,0}{
    calculateBoundingBox();
}



void drawable::Lines::draw(const drawable::Renderer& renderer) const{
    renderer.drawLines(*this);
}
const std::vector<std::pair<double, double>> drawable::Lines::getP1s() const{
    return m_p1s;
}
const std::vector<std::pair<double, double>> drawable::Lines::getP2s() const{
    return m_p2s;
}

drawable::Color drawable::Lines::getColor() const{
    return m_color;
}
std::unique_ptr<drawable::Drawable> drawable::Lines::clone() const{
    return std::make_unique<drawable::Lines>(m_p1s, m_p2s, m_color);
}
drawable::BoundingBox drawable::Lines::getBoundingBox() const{
    return m_bounding_box;
}
