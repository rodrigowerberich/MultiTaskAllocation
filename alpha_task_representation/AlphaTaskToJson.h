#ifndef ALPHATASKTOJSON_H__
#define ALPHATASKTOJSON_H__

#include <string>
#include <AlphaTask.h>
#include <AlphaTasks.h>

namespace jsonifier{

template <typename T>
std::string toJson(const T& jsonable);

std::string prettify(const std::string& json);

}


#endif //ALPHATASKTOJSON_H__