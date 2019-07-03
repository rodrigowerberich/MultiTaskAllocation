#include "matplotlibcpp.h"
#include "JSON.h"
#include <InputParser.h>
namespace plt = matplotlibcpp;
using namespace std;

#define TESTING 0

#if TESTING

int main(int argc, char *argv[]) {
    // InputParser inputParser{argc, argv};
    // if(!inputParser.inputValid()){
    //     return -1;
    // }
    // string file_name = inputParser.getFileName();
    // cout << file_name << endl;
    
    // Prepare data.
    int n = 5000; // number of data points
    vector<double> x(n),y(n); 
    for(int i=0; i<n; ++i) {
        double t = 2*M_PI*i/n;
        x.at(i) = 16*sin(t)*sin(t)*sin(t);
        y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
    }

    // plot() takes an arbitrary number of (x,y,format)-triples. 
    // x must be iterable (that is, anything providing begin(x) and end(x)),
    // y must either be callable (providing operator() const) or iterable. 
    plt::plot(x, y, "r-", x, [](double d) { return 12.5+abs(sin(d)); }, "k-");

    JsonObject object = JSON::parseObject(R"({"id":"Banana"}")");
    cout << object.toStringUnformatted() << endl;
    JSON::deleteObject(object);

    // show plots
    plt::show();
    return 0;
}

#else

#include <RobotTypeTest.h>
#include <TaskTypeTest.h>
#include <EffortFunctionTest.h>
#include <RewardFunctionTest.h>
#include <ProblemRepresentationTest.h>

int main(){
    robot_type_test();
    task_type_test();
    effort_function_test();
    reward_function_test();
    problem_definition_test();
}

#endif