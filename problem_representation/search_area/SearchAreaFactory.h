#ifndef SEARCHAREAFACTORY_H__
#define SEARCHAREAFACTORY_H__

#include <SearchArea.h>
#include <JSON.h>
#include <memory>

class SearchAreaFactory{
public:
    static std::unique_ptr<SearchArea> make(const std::string& json);
    static std::unique_ptr<SearchArea> make(JsonObject& json);
};

#endif //SEARCHAREAFACTORY_H__