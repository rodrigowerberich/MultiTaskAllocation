#include <Tasks.h>
#include <GnuPlotRenderer.h>

const std::unique_ptr<Task> & Tasks::getTask(const std::string& task_id){
    return getValueByName(task_id);
}

namespace renderer{
template <>
void draw(const std::unique_ptr<Tasks>& drawable, const GnuPlotRenderer& renderer){
    for(const auto& task: *drawable){
        auto task_drawable = drawable::NamedPoint{task->getPosition(), task->getName()};
        task_drawable.setColor(drawable::Color::DarkGreen);
        renderer.draw(task_drawable);
    }
}
}