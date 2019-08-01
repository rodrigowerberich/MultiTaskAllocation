#include <Robots.h>
#include <GnuPlotRenderer.h>
#include <NamedPoint.h>

const std::unique_ptr<Robot> & Robots::getRobot(const std::string& robot_name){
    return getValueByName(robot_name);
}

namespace renderer{
template <>
void draw(const std::unique_ptr<Robots>& drawable, const GnuPlotRenderer& renderer){
    for(const auto& robot: *drawable){
        auto robot_drawable = drawable::NamedPoint{robot->getPosition(), robot->getName()};
        robot_drawable.setColor(drawable::Color::Purple);
        renderer.draw(robot_drawable);
    }
}
}