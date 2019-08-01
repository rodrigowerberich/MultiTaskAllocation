#ifndef TWODOUBLES_CPP__
#define TWODOUBLES_CPP__

#include <tuple>
#include <string>
#include <deque>
#include <PlannerCLIFunction.h>

std::tuple <bool, std::string> two_double_verifier(std::deque<std::string> deque);
std::tuple <double, double> two_double_parser(std::deque<std::string> deque);
using TwoDoubleCLIFunction = PlannerCLIFunctionTemplate<std::tuple<double, double>, two_double_parser, two_double_verifier>;


#endif //TWODOUBLES_CPP__