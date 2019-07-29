
#define TESTING 0

#if TESTING

#include "matplotlibcpp.h"
// #include "JSON.h"
// #include <InputParser.h>
// #include <Position.h>
#include <PyPlotRectangle.h>
#include <PyPlotPoint.h>
#include <PyPlotNamedPoint.h>
#include <PyPlotCircle.h>
namespace plt = matplotlibcpp;
using namespace std;
// #include <RobotTypeTest.h>
// #include <TaskTypeTest.h>
// #include <EffortFunctionTest.h>
// #include <RewardFunctionTest.h>
// #include <ProblemRepresentationTest.h>
#include <RectangleGeometricObject.h>

void quick_draw(){
    auto rect = PyPlot::Rectangle(-2,-2,4,4);
    rect.draw();
    plt::pause(1);
}

int main(int argc, char *argv[]) {
    // robot_type_test();
    // task_type_test();
    // effort_function_test();
    // reward_function_test();
    // problem_definition_test();
    // InputParser inputParser{argc, argv};
    // if(!inputParser.inputValid()){
    //     return -1;
    // }
    // string file_name = inputParser.getFileName();
    // cout << file_name << endl;
    
    // Prepare data.
    // int n = 5000; // number of data points
    // vector<double> x(n),y(n); 
    // for(int i=0; i<n; ++i) {
    //     double t = 2*M_PI*i/n;
    //     x.at(i) = 16*sin(t)*sin(t)*sin(t);
    //     y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
    // }

    // // plot() takes an arbitrary number of (x,y,format)-triples. 
    // // x must be iterable (that is, anything providing begin(x) and end(x)),
    // // y must either be callable (providing operator() const) or iterable. 
    // plt::plot(x, y, "r-", x, [](double d) { return 12.5+abs(sin(d)); }, "k-");
    PyPlot::Rectangle rect(-10,-10,20,20);
    rect.draw();
    PyPlot::Circle circle(-2,-2, 1);
    circle.draw();
    plt::xlim(-15,15);
    plt::ylim(-15,15);
    plt::pause(1);
    // show plots
    plt::show(false);
    auto dot = PyPlot::NamedPoint(1,1, "R1");
    // plt::plot(std::vector<int>{3},std::vector<int>{3},"kx");
    for(int i=3; i < 11; i++){
        // plt::plot(std::vector<int>{i},std::vector<int>{i},"kx");
        dot.update(i,i);
        dot.draw();
        std::cout << "Updating position " << i << std::endl;
        plt::pause(0.5);
        // plt::show(false);
        // mySleep(1000);
    }
    quick_draw();
    plt::pause(0.5);
    dot.erase();
    // RectangleGeometricObject obj{{-1,-4},2,2};
    // const_cast<Basic::Drawable*>(obj.getDrawable())->draw();

    // cout << "WTF 1?\n";
    // plt::plot({2},{3},"r+");
    // cout << "WTF 2?\n";
    // std::vector<double> x = {1,2,3};
    // std::vector<double> y = {3,6,9};
    // std::vector<double> x1 = {1,2,3};
    // std::vector<double> y1 = {7,8,9};
    // auto test = plt::Plot();
    // test.update(x,y);
    // plt::show();
    // test.remove();
    // test = plt::Plot();
    // test.update(x1,y1);
    // PyPlot::Rectangle rect(-10,-10,20,20);
    // rect.draw();
    // plt::xlim(-15,15);
    // plt::ylim(-15,15);
    // plt::pause(0.5);
    rect.erase();
    rect = PyPlot::Rectangle(-1,-1,2,2);
    rect.draw();
    plt::pause(0.5);
    // const_cast<Basic::Drawable*>(obj.getDrawable())->draw();
    // plt::xlim(-15,15);
    // plt::ylim(-15,15);

    plt::show();
    // cout << "WTF 3?\n";


    return 0;
}

#else

#include <InputParser.h>
#include <iostream>
#include <string>
#include <ProblemRepresentation.h>
#include "matplotlibcpp.h"
using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[]){
    InputParser inputParser{argc, argv};
    if(!inputParser.inputValid()){
        return -1;
    }
    // plt::xlim(-15,15);
    // plt::ylim(-15,15);

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
        std::cout << "main.cpp\n";
        problemRepresentation.demonstrate();
        // plt::pause(true);
    }


    return 0;
}

#endif