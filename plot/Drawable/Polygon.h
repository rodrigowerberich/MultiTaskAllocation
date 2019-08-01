#ifndef POLYGON_H__
#define POLYGON_H__

#include <vector>
#include <Color.h>
#include <cassert>
#include <iostream>

namespace drawable{

template <int N, typename Numeric>
class Polygon{
protected:
    std::vector<Numeric> m_x_points;
    std::vector<Numeric> m_y_points;
    drawable::Color m_color;
public:
    Polygon(): m_x_points{N},
    m_y_points{N},
    m_color{drawable::Color::Blue}{

    }    
    Polygon(std::vector<Numeric> x_points, std::vector<Numeric> y_points, drawable::Color color):
    m_x_points{N},
    m_y_points{N},
    m_color{color}{
        std::cout << "Polygon 1\n";
        assert(x_points.size() == y_points.size());
        std::cout << "Polygon 2\n";
        assert(x_points.size() == N);
        std::cout << "Polygon 3\n";
        std::copy(std::begin(x_points), std::end(x_points), std::begin(m_x_points));
        std::cout << "Polygon 4\n";
        std::copy(std::begin(y_points), std::end(y_points), std::begin(m_y_points));
        std::cout << "Polygon 5\n";
    }
    const std::vector<Numeric>& getXPoints() const{
        return m_x_points;
    }
    const std::vector<Numeric>& getYPoints() const{
        return m_y_points;
    }
    drawable::Color getColor() const{
        return m_color;
    }

};

}


#endif //POLYGON_H__