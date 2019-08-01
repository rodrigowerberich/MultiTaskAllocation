
#define ALTERNATIVE 2

#if ALTERNATIVE == 0

#include <GnuPlotRenderer.h>

int main(int argc, char *argv[]) {

    GnuPlotRenderer renderer;
    renderer.setAxixRange(-20.0,20.0, -20.0, 20.0);

    renderer.holdOn();
    renderer.draw(drawable::Rectangle{-1.0,-1.0,5.0,3.0});
    renderer.drawRectangle({-10.0,-10.0,20.0,20.0});
    renderer.draw(drawable::Circle{-1,-5,0.5});
    renderer.drawCircle({1,1,2});
    renderer.drawCircle({0,0,10});
    renderer.drawPoint({-1,3});
    renderer.draw(drawable::Point{1,-3});
    renderer.drawNamedPoint({1,3, "R1"});
    renderer.draw(drawable::NamedPoint{-1,-3, "R2"});

    return 0;
}

#elif ALTERNATIVE == 1

#include <InputParser.h>
#include <iostream>
#include <string>
#include <ProblemRepresentation.h>
#include <GnuPlotRenderer.h>
#include <Drawable.h>
using namespace std;

int main(int argc, char *argv[]){
    InputParser inputParser{argc, argv};
    if(!inputParser.inputValid()){
        return -1;
    }
    GnuPlotRenderer gp;

    string file_name = inputParser.getFileName();
    cout << "Parsing " << file_name << endl;
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
        return -1;
    }
    cout << "Parsed with success!!" << endl;
    if(inputParser.showProblem()){
        gp.holdOn();
        auto bounding_box = problemRepresentation.getSearchArea()->getDrawable()->getBoundingBox();
        gp.setAxixRange(1.1*bounding_box.lower_left_x, 1.1*bounding_box.top_right_x, 1.1*bounding_box.lower_left_y, 1.1*bounding_box.top_right_y);
        problemRepresentation.draw(gp);
        gp.holdOn(false);
        cout << "Type in something to progress...\n";
        cin.get();
        cout << "\33[1A\33[2K\r";

    }

    cout << "Done, bye!" << endl;
    return 0;
}

#elif ALTERNATIVE == 2

#include <PlannerCLI.h>
#include <iostream>
#include <functional>
#include <tuple>
#include <deque>
#include <PlannerCLIFunction.h>


int main(int argc, char *argv[]){
    // Planner CLI will process the input in order from left to right, ignoring unknow commands
    PlannerCLI planner(argc, argv);    
    return planner.run();
    // return 0;
}

#endif