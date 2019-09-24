#ifndef TWOSTRINGSCLIFUNCTION_H__
#define TWOSTRINGSCLIFUNCTION_H__

#include <tuple>
#include <string>
#include <deque>
#include <PlannerCLIFunction.h>

std::tuple <bool, std::string> two_string_verifier(std::deque<std::string> deque);
std::tuple <std::string, std::string> two_string_parser(std::deque<std::string> deque);
using TwoStringCLIFunction = PlannerCLIFunctionTemplate<std::tuple<std::string, std::string>, two_string_parser, two_string_verifier>;

#endif //TWOSTRINGSCLIFUNCTION_H__