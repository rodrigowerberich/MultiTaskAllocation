
#define TESTING 1

#if TESTING

#include <vector>
#include <cmath>
#include <tuple>

#include "gnuplot-iostream.h"

int main(int argc, char *argv[]) {
	Gnuplot gp;
	// Create a script which can be manually fed into gnuplot later:
	//    Gnuplot gp(">script.gp");
	// Create script and also feed to gnuplot:
	//    Gnuplot gp("tee plot.gp | gnuplot -persist");
	// Or choose any of those options at runtime by setting the GNUPLOT_IOSTREAM_CMD
	// environment variable.

	// Gnuplot vectors (i.e. arrows) require four columns: (x,y,dx,dy)
	std::vector<std::tuple<double, double, double, double> > pts_A;

	// You can also use a separate container for each column, like so:
	std::vector<double> pts_B_x;
	std::vector<double> pts_B_y;
	std::vector<double> pts_B_dx;
	std::vector<double> pts_B_dy;

	// You could also use:
	//   std::vector<std::vector<double> >
	//   boost::tuple of four std::vector's
	//   std::vector of std::tuple (if you have C++11)
	//   arma::mat (with the Armadillo library)
	//   blitz::Array<blitz::TinyVector<double, 4>, 1> (with the Blitz++ library)
	// ... or anything of that sort

	for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
		double theta = alpha*2.0*3.14159;
		pts_A.push_back(std::make_tuple(
			 cos(theta),
			 sin(theta),
			-cos(theta)*0.1,
			-sin(theta)*0.1
		));

		pts_B_x .push_back( cos(theta)*0.8);
		pts_B_y .push_back( sin(theta)*0.8);
		pts_B_dx.push_back( sin(theta)*0.1);
		pts_B_dy.push_back(-cos(theta)*0.1);
	}

	// Don't forget to put "\n" at the end of each line!
	gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "plot '-' with vectors title 'pts_A', '-' with vectors title 'pts_B'\n";
	gp.send1d(pts_A);
	gp.send1d(std::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));

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