#ifndef ONESTRINGCLIFUNCTION_H__
#define ONESTRINGCLIFUNCTION_H__

#include <tuple>
#include <string>
#include <deque>
#include <PlannerCLIFunction.h>

std::tuple <bool, std::string> one_string_verifier(std::deque<std::string> deque);
std::tuple <std::string> one_string_parser(std::deque<std::string> deque);
using OneStringCLIFunction = PlannerCLIFunctionTemplate<std::tuple<std::string>, one_string_parser, one_string_verifier>;

#endif //ONESTRINGCLIFUNCTION_H__