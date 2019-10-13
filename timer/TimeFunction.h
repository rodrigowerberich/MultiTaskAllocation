#ifndef TIMEFUNCTION_H__
#define TIMEFUNCTION_H__

#include <chrono>
#include <iostream>
#include <tuple>

template <typename FuncType, typename... TArgs>
decltype(auto) timeFunction(FuncType func, TArgs&&... params){
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    auto result = func(std::forward<TArgs>(params)...);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    std::cout << "Took " << duration.count() << " microseconds\n";
    return result;
}

template <typename Timecast, typename FuncType, typename... TArgs>
decltype(auto)  timeFunctionCast(FuncType func, TArgs&&... params){
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    auto result = func(std::forward<TArgs>(params)...);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<Timecast>(stop-start);
    return std::make_tuple(duration, result);
}

#endif //TIMEFUNCTION_H__