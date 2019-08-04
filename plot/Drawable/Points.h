#ifndef PointsS_H__
#define PointsS_H__

#include <Drawable.h>
#include <Color.h>
#include <vector>

namespace drawable{
class Points: public drawable::Drawable{
private:
    std::vector<std::pair<double, double>> m_points;
    drawable::Color m_color;
    drawable::BoundingBox m_bounding_box;
    void calculateBoundingBox();
public:
    Points(std::vector<std::pair<double, double>> points, drawable::Color color = drawable::Color::Green);
    template <typename T>
    Points(T points, drawable::Color color = drawable::Color::Green):m_color{color},m_bounding_box{0,0,0,0}{
        for(const auto& point:points){
            m_points.push_back({point[0], point[1]});
        }
        calculateBoundingBox();
    }
    ~Points(){}
    virtual void draw(const Renderer& renderer) const override;
    const std::vector<std::pair<double, double>> & getPoints() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_color = color;};
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
    virtual BoundingBox getBoundingBox() const;
};
}

#endif //PointsS_H__