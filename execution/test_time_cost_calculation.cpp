#include "test_time_cost_calculation.h"
#include <iostream>

void test_time_cost_calculation(std::tuple <std::string, std::string> values){
    auto file_name = std::get<0>(values);
    auto task_name = std::get<1>(values);

    std::cout << file_name << std::endl << task_name << std::endl;
}
