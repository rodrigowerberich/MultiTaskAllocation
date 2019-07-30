
#define TESTING 1

#if TESTING

#include <vector>
#include <cmath>
#include <tuple>
#include "gnuplot-iostream.h"

#include <GnuPlotRenderer.h>

int main(int argc, char *argv[]) {

    GnuPlotRenderer renderer;
    renderer.setAxixRange(-20.0,20.0, -20.0, 20.0);

    renderer.holdOn();
    renderer.drawRectangle({-1.0,-1.0,2.0,3.0});
    renderer.drawRectangle({-10.0,-10.0,20.0,20.0});

    return 0;
}

#else

#include <InputParser.h>
#include <iostream>
#include <string>
#include <ProblemRepresentation.h>
#include "matplotlibcpp.h"
#include <PyPlotRenderer.h>
using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char *argv[]){
    InputParser inputParser{argc, argv};
    if(!inputParser.inputValid()){
        return -1;
    }
    plt::xlim(-15,15);
    plt::ylim(-15,15);

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
        // plt::plot({-1.0,1.0,1.0,-1.0},{-1.0,-1.0,1.0,1.0},"r-");
        PyPlotRenderer().drawRectangle({-1.0,-1.0,2.0,3.0});
        plt::show();
        // plt::pause(true);
    }


    return 0;
}

#endif