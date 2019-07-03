#ifndef ROBOTS_H__
#define ROBOTS_H__

#include <vector>
#include <Robot.h>
#include <memory>

using Robots = std::vector<std::unique_ptr<Robot>>;

#endif // ROBOTS_H__