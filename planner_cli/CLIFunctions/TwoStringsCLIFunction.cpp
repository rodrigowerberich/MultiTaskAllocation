#include <OneStringCLIFunction.h>

std::tuple <bool, std::string> two_string_verifier(std::deque<std::string>){
    return std::make_tuple<bool, std::string>(true, "");
}
std::tuple <std::string, std::string> two_string_parser(std::deque<std::string> deque){
    std::string arg = deque[0];
    std::string arg1 = deque[1];
    return std::tuple<std::string, std::string>(arg, arg1);
}
