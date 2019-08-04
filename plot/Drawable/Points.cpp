#include <Points.h>

void drawable::Points::calculateBoundingBox(){
    for(const auto & point:m_points){
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


drawable::Points::Points(std::vector<std::pair<double, double>> points, drawable::Color color):m_points{points},m_color{color},m_bounding_box{0,0,0,0}{
    calculateBoundingBox();
}



void drawable::Points::draw(const drawable::Renderer& renderer) const{
    renderer.drawPoints(*this);
}
const std::vector<std::pair<double, double>> & drawable::Points::getPoints() const{
    return m_points;
}

drawable::Color drawable::Points::getColor() const{
    return m_color;
}
std::unique_ptr<drawable::Drawable> drawable::Points::clone() const{
    return std::make_unique<drawable::Points>(m_points, m_color);
}
drawable::BoundingBox drawable::Points::getBoundingBox() const{
    return m_bounding_box;
}
