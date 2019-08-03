#include <visualize.h>
#include <ProblemRepresentation.h>
#include <iostream>
#include <GnuPlotRenderer.h>

void visualize(std::tuple <std::string> values){
    using namespace std;
    auto file_name = std::get<0>(values);
    cout << "Parsing " << file_name << "...\n";
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
        return;
    }
    GnuPlotRenderer renderer;
    renderer.holdOn();
    auto bounding_box = problemRepresentation.getSearchArea()->getDrawable()->getBoundingBox();
    renderer.setAxixRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);
    problemRepresentation.draw(renderer);
    renderer.holdOn(false);
}