#include <RewardFunctionTest.h>
#include <RewardFunctionMap.h>
#include <iostream>

bool RewardFunctionMapTest(){
    RewardFunctionMap reward_function;
    reward_function.addMapping("T1", 7.3);
    reward_function.addMapping("T2", 5.2);
    reward_function.addMapping("T3", 1.4);
    std::cout << "Reward function test" << std::endl;
    std::cout << reward_function("T1") << std::endl;
    std::cout << reward_function("T2") << std::endl;
    std::cout << reward_function("T3") << std::endl;
    std::cout << reward_function("T4") << std::endl;
    return true;
}

void reward_function_test(){
    RewardFunctionMapTest();
}