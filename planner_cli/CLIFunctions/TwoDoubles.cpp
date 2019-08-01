#include <TwoDoubles.h>
#include <sstream>


bool isFloat( std::string myString ) {
    std::istringstream iss(myString);
    float f;
    iss >> std::noskipws >> f; // noskipws considers leading whitespace invalid
    // Check the entire string was consumed and if either failbit or badbit is set
    return iss.eof() && !iss.fail(); 
}

std::tuple <bool, std::string> two_double_verifier(std::deque<std::string> deque){
    auto first = deque[0];
    auto second = deque[1];
    if(!isFloat(first)){
        return std::make_tuple<bool, std::string>(false, "First argument should be a double value");
    }
    if(!isFloat(second)){
        return std::make_tuple<bool, std::string>(false, "Second argument should be a double value");
    }
    return std::make_tuple<bool, std::string>(true, "");
}

std::tuple <double, double> two_double_parser(std::deque<std::string> deque){
    auto first = deque[0];
    auto second = deque[1];
    
    return std::make_tuple<double, double>(::atof(first.c_str()), ::atof(second.c_str()));
}