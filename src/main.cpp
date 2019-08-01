
#define TESTING 0

#if TESTING

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

#else

#include <InputParser.h>
#include <iostream>
#include <string>
#include <ProblemRepresentation.h>
#include <GnuPlotRenderer.h>

using namespace std;

int main(int argc, char *argv[]){
    InputParser inputParser{argc, argv};
    if(!inputParser.inputValid()){
        return -1;
    }
    GnuPlotRenderer gp;
    gp.setAxixRange(-15,15,-15,15);

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
        problemRepresentation.draw(gp);
    }


    return 0;
}

#endif