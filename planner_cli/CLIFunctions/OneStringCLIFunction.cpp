#include <OneStringCLIFunction.h>

std::tuple <bool, std::string> one_string_verifier(std::deque<std::string> deque){
    return std::make_tuple<bool, std::string>(true, "");
}
std::tuple <std::string> one_string_parser(std::deque<std::string> deque){
    std::string arg = deque[0];
    return std::tuple<std::string>(arg);
}
