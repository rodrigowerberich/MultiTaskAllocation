#ifndef LINES_H__
#define LINES_H__

#include <Drawable.h>
#include <Color.h>
#include <vector>

namespace drawable{
class Lines: public drawable::Drawable{
private:
    std::vector<std::pair<double, double>> m_p1s;
    std::vector<std::pair<double, double>> m_p2s;
    drawable::Color m_color;
    drawable::BoundingBox m_bounding_box;
    void calculateBoundingBox();
public:
    Lines(std::vector<std::pair<double, double>> p1s, std::vector<std::pair<double, double>> p2s, drawable::Color color = drawable::Color::Black);
    template <typename T1, typename T2>
    Lines(T1 p1s, T2 p2s, drawable::Color color = drawable::Color::Black):m_color{color},m_bounding_box{0,0,0,0}{
        for(const auto& p1:p1s){
            m_p1s.push_back({p1[0], p1[1]});
        }
        for(const auto& p2:p2s){
            m_p2s.push_back({p2[0], p2[1]});
        }
        calculateBoundingBox();
    }
    ~Lines(){}
    virtual void draw(const Renderer& renderer) const override;
    const std::vector<std::pair<double, double>> getP1s() const;
    const std::vector<std::pair<double, double>> getP2s() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_color = color;};
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
    virtual BoundingBox getBoundingBox() const;
};
}

#endif //LINES_H__