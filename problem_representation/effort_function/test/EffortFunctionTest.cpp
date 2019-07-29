#include <EffortFunctionTest.h>
#include <EffortFunctionMap.h>
#include <iostream>

static bool EffortFunctionMapTest(){
    EffortFunctionMap effort_function;
    effort_function.addMapping("T1", "R1", 7.3);
    effort_function.addMapping("T2", "R2", 5.2);
    effort_function.addMapping("T3", "R4", 1.4);
    std::cout << effort_function("T1", "R1") << std::endl;
    std::cout << effort_function("T2", "R2") << std::endl;
    std::cout << effort_function("T3", "R4") << std::endl;
    std::cout << effort_function("T4", "R3") << std::endl;
    return true;
}

void effort_function_test(){
    EffortFunctionMapTest();
}